"""A plane-fragility refresh republishes only the pour nets it re-eroded, and
sorts a net's rows only when two of its islands can emit one cell.

Every commit, rip and restore refreshes the field. Republishing EVERY pour
net's rows through `_dedup_max` after each one re-sorted ~10^6 rows of a
board-sized pour thousands of times, the largest cost of a route step on
boards with planes. The rows a net publishes must still be exactly what the
full publish gave: one row per cell, the largest cost where a net's zones
overlap.

Rows:
  - after each of a series of refreshes, every net's published rows equal the
    full sort-and-dedup of its islands' rows (as a set; the map takes a
    per-cell max, so row order is not read);
  - a refresh that touches one pour only leaves the other's array as it was;
  - a net with two overlapping zones on one layer is found, and its rows stay
    one per cell; a net whose islands are disjoint is not.

    python3 tests/test_fragility_publish_changed.py
"""

import contextlib
import io
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import plane_fragility as pf  # noqa: E402
from kicad_parser import BoardInfo, Net, PCBData, Segment, Via, Zone  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

GND, VCC, SIG = 1, 2, 3


def _board():
    """GND and VCC pours side by side on F.Cu, and two overlapping GND zones
    on B.Cu. No source file, so the field is rasterized from the outlines."""
    nets = {GND: Net(GND, 'GND'), VCC: Net(VCC, 'VCC'), SIG: Net(SIG, 'SIG')}
    zones = [Zone(GND, 'GND', 'F.Cu', [(0, 0), (10, 0), (10, 10), (0, 10)]),
             Zone(VCC, 'VCC', 'F.Cu', [(20, 0), (30, 0), (30, 10), (20, 10)]),
             Zone(GND, 'GND', 'B.Cu', [(0, 0), (12, 0), (12, 10), (0, 10)]),
             Zone(GND, 'GND', 'B.Cu', [(8, 0), (20, 0), (20, 10), (8, 10)])]
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    return PCBData(board_info=bi, nets=nets, footprints={}, vias=[], segments=[],
                   pads_by_net={}, zones=zones)


def _rowset(a):
    return {tuple(int(v) for v in r) for r in a}


def _full(field, net_id):
    """What the full publish wrote for one net: every island's rows, one per
    cell, the largest cost kept."""
    rows = [st.rows for st in field.states
            if st.net_id == net_id and st.rows is not None]
    return pf._dedup_max(np.vstack(rows))


def main():
    fails = []

    def ok(label, cond, detail=''):
        print(f"  [{'ok' if cond else 'FAIL'}] {label}"
              f"{'' if cond else ': ' + str(detail)}")
        if not cond:
            fails.append(label)

    pcb, cache = _board(), {}
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.2)
    with contextlib.redirect_stdout(io.StringIO()):
        pf.register_plane_fragility(pcb, cfg, cache)
    field = cfg._fragility_field
    gk, vk = pf.fragility_cache_key(GND), pf.fragility_cache_key(VCC)

    ok('the overlapping GND zones are found, the lone VCC pour is not',
       getattr(field, '_shared', None) == {GND}, getattr(field, '_shared', None))

    # foreign copper landing across the GND pours only, one change at a time
    changes = [([Segment(5.0, 0.5, 5.0, 9.5, 0.2, 'F.Cu', SIG)], []),
               ([Segment(9.0, 1.0, 11.0, 9.0, 0.2, 'B.Cu', SIG)], []),
               ([], [Via(3.0, 3.0, 0.45, 0.2, ['F.Cu', 'B.Cu'], SIG)])]
    for n, (segs, vias) in enumerate(changes, 1):
        vcc_before = cache[vk]
        pcb.segments.extend(segs)
        pcb.vias.extend(vias)
        pf.fragility_on_copper_change(cfg, pcb, segs, vias)
        ok(f'change {n}: GND rows equal the full publish',
           _rowset(cache[gk]) == _rowset(_full(field, GND)))
        ok(f'change {n}: GND rows are one per cell',
           len(_rowset(r[:3] for r in cache[gk])) == len(cache[gk]))
        ok(f'change {n}: VCC rows equal the full publish',
           _rowset(cache[vk]) == _rowset(_full(field, VCC)))
        if not vias:
            ok(f'change {n}: VCC, untouched, is not republished',
               cache[vk] is vcc_before)

    field.publish()
    ok('a full publish keeps every key',
       {k for k in cache if isinstance(k, tuple)} == {gk, vk})

    print('ALL PASS' if not fails else f'{len(fails)} FAILED')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
