#!/usr/bin/env python3
"""
Test for issue #350: zone (priority N) and island_removal_mode parsing, and
their two consumers.

1. Parser: extract_zones reads priority / island_removal_mode / island_area_min
   from the zone header (absent tokens -> KiCad defaults 0 / 0 / 0.0).
2. _real_fill_point: foreign-outline rejection is priority-exact -- a point in
   an own-net island zone poured at HIGHER priority than an overlapping
   board-wide foreign zone is accepted (the old code rejected any foreign
   outline hit); a foreign zone at equal or higher priority still rejects.
3. _island_kept_by_filler: anchor-less patches are joined only when the owning
   zone's island_removal_mode keeps isolated islands (mode 1, or mode 2 with
   the patch at/above island_area_min); mode 0 (the KiCad default) skips.

Run:
    python3 tests/test_zone_priority_islands.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Zone, Net, Via, extract_zones
from routing_config import GridCoord
from plane_region_connector import _real_fill_point, _island_kept_by_filler

ZONE_TEXT = '''
(kicad_pcb
\t(zone
\t\t(net 1)
\t\t(net_name "GND")
\t\t(layer "In1.Cu")
\t\t(uuid "aaaa")
\t\t(priority 2)
\t\t(fill yes
\t\t\t(thermal_gap 0.5)
\t\t\t(island_removal_mode 2)
\t\t\t(island_area_min 10.5)
\t\t)
\t\t(polygon
\t\t\t(pts
\t\t\t\t(xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10)
\t\t\t)
\t\t)
\t)
\t(zone
\t\t(net 2)
\t\t(net_name "+3V3")
\t\t(layer "In1.Cu")
\t\t(uuid "bbbb")
\t\t(polygon
\t\t\t(pts
\t\t\t\t(xy 20 0) (xy 30 0) (xy 30 10) (xy 20 10)
\t\t\t)
\t\t)
\t)
)
'''


def _square(cx, cy, half):
    return [(cx - half, cy - half), (cx + half, cy - half),
            (cx + half, cy + half), (cx - half, cy + half)]


def _zone(net_id, name, poly, priority=0, mode=0, area_min=0.0):
    return Zone(net_id=net_id, net_name=name, layer='In1.Cu', polygon=poly,
                priority=priority, island_removal_mode=mode,
                island_area_min=area_min)


def _pcb(zones):
    return PCBData(board_info=None, nets={1: Net(1, 'GND'), 2: Net(2, '+3V3')},
                   footprints={}, vias=[], segments=[], pads_by_net={},
                   zones=zones)


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- 1: parser ---------------------------------------------------------
    print("Parser: priority / island_removal_mode / island_area_min")
    zones = extract_zones(ZONE_TEXT)
    z1 = next(z for z in zones if z.net_id == 1)
    z2 = next(z for z in zones if z.net_id == 2)
    check("explicit tokens parsed", z1.priority == 2 and
          z1.island_removal_mode == 2 and abs(z1.island_area_min - 10.5) < 1e-9)
    check("absent tokens -> KiCad defaults", z2.priority == 0 and
          z2.island_removal_mode == 0 and z2.island_area_min == 0.0)

    # -- 2: priority-exact foreign-outline rejection ------------------------
    print("\n_real_fill_point: priority-exact foreign rejection")
    big_foreign = _square(5, 5, 5)      # net 2 covers everything
    own_island = _square(5, 5, 1.5)     # net 1 island inside it
    pt = (5.0, 5.0)
    margin = 0.3

    # Own island outranks the foreign pour -> accepted (old code rejected)
    pcb = _pcb([_zone(2, '+3V3', big_foreign, priority=0),
                _zone(1, 'GND', own_island, priority=1)])
    check("own higher-priority island accepted",
          _real_fill_point(pt, 1, pcb, [own_island], 'In1.Cu', margin))

    # Foreign zone at HIGHER priority -> rejected
    pcb = _pcb([_zone(2, '+3V3', big_foreign, priority=3),
                _zone(1, 'GND', own_island, priority=1)])
    check("foreign higher-priority still rejected",
          not _real_fill_point(pt, 1, pcb, [own_island], 'In1.Cu', margin))

    # Equal priority overlap -> conservative rejection (undefined in KiCad)
    pcb = _pcb([_zone(2, '+3V3', big_foreign, priority=1),
                _zone(1, 'GND', own_island, priority=1)])
    check("equal-priority overlap stays rejected",
          not _real_fill_point(pt, 1, pcb, [own_island], 'In1.Cu', margin))

    # No foreign outline at all -> plain acceptance path unchanged
    pcb = _pcb([_zone(1, 'GND', own_island)])
    check("no foreign zone accepted",
          _real_fill_point(pt, 1, pcb, [own_island], 'In1.Cu', margin))

    # -- 3: orphan-patch island gating --------------------------------------
    print("\n_island_kept_by_filler: island_removal_mode gating")
    coord = GridCoord(0.5)
    step = 0.5
    # a 2x2mm patch of 0.5mm cells centred in the zone: 16 cells = 4 mm^2
    patch = {(gx, gy) for gx in range(8, 12) for gy in range(8, 12)}
    own = _square(5, 5, 5)

    pcb = _pcb([_zone(1, 'GND', own, mode=0)])
    check("mode 0 (always remove) skips a truly bare patch",
          not _island_kept_by_filler(pcb, 1, 'In1.Cu', patch, coord, step))

    # The castor class: mode 0 but a same-net via sits on the patch (its cells
    # blocked in the model, so the flood never saw it) -> not isolated, kept.
    pcb = _pcb([_zone(1, 'GND', own, mode=0)])
    pcb.vias.append(Via(x=5.0, y=5.0, size=0.5, drill=0.3,
                        layers=['F.Cu', 'B.Cu'], net_id=1))
    check("mode 0 with same-net via on patch is kept (castor #217 class)",
          _island_kept_by_filler(pcb, 1, 'In1.Cu', patch, coord, step))

    pcb = _pcb([_zone(1, 'GND', own, mode=0)])
    pcb.vias.append(Via(x=9.0, y=9.0, size=0.5, drill=0.3,
                        layers=['F.Cu', 'B.Cu'], net_id=1))
    check("mode 0 with same-net via far away still skips",
          not _island_kept_by_filler(pcb, 1, 'In1.Cu', patch, coord, step))

    pcb = _pcb([_zone(1, 'GND', own, mode=1)])
    check("mode 1 (never remove) keeps the patch",
          _island_kept_by_filler(pcb, 1, 'In1.Cu', patch, coord, step))

    pcb = _pcb([_zone(1, 'GND', own, mode=2, area_min=3.0)])
    check("mode 2 keeps patch above area_min (4mm^2 >= 3mm^2)",
          _island_kept_by_filler(pcb, 1, 'In1.Cu', patch, coord, step))

    pcb = _pcb([_zone(1, 'GND', own, mode=2, area_min=6.0)])
    check("mode 2 skips patch below area_min (4mm^2 < 6mm^2)",
          not _island_kept_by_filler(pcb, 1, 'In1.Cu', patch, coord, step))

    pcb = _pcb([])
    check("patch outside every parsed outline keeps old always-join",
          _island_kept_by_filler(pcb, 1, 'In1.Cu', patch, coord, step))

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
