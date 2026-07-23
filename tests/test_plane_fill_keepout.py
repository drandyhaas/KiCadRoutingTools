#!/usr/bin/env python3
"""#477: copperpour keep-out areas must split the plane fill model.

KiCad's filler subtracts (copperpour not_allowed) rule areas from a zone's
fill, so a keep-out crossing a plane splits it into real islands (mokya's
antenna keep-outs cut the In1 GND plane). ZoneFillModel ignored them: the
model graded those islands as one component, region repair never saw the
splits, and only the kicad-oracle recheck caught the resulting opens.

Run:
    python3 tests/test_plane_fill_keepout.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, BoardInfo, Zone, extract_keepouts
from plane_fill_model import ZoneFillModel


def _board(keepouts):
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 20.0, 10.0))
    bi.keepouts = keepouts
    return PCBData(board_info=bi, nets={}, footprints={}, vias=[],
                   segments=[], pads_by_net={})


def _zone(layer='F.Cu'):
    return Zone(net_id=1, net_name='GND', layer=layer,
                polygon=[(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)],
                clearance=0.3, min_thickness=0.25)


# A vertical strip crossing the zone top-to-bottom at x = 9..11.
def _strip_ko(layers, copper_pour_allowed):
    return {'polygon': [(9.0, -1.0), (11.0, -1.0), (11.0, 11.0), (9.0, 11.0)],
            'holes': [], 'layers': layers,
            'tracks_allowed': True, 'vias_allowed': True,
            'copper_pour_allowed': copper_pour_allowed}


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    print("#477: copperpour keep-outs split the fill model")

    # Baseline: empty board, one solid zone -> one component.
    m = ZoneFillModel(_board([]), _zone())
    left, right = m.query_component(5.0, 5.0), m.query_component(15.0, 5.0)
    check("no keep-out: one component spans the zone",
          m.ok and left and left == right)

    # Copperpour-banned strip crossing the zone -> two components.
    m = ZoneFillModel(_board([_strip_ko({'F.Cu'}, False)]), _zone())
    left, right = m.query_component(5.0, 5.0), m.query_component(15.0, 5.0)
    mid = m.query_component(10.0, 5.0)
    check("banned keep-out splits the plane",
          m.ok and left and right and left != right)
    check("no fill inside the keep-out", mid == 0)

    # Same strip with copperpour allowed -> no split.
    m = ZoneFillModel(_board([_strip_ko({'F.Cu'}, True)]), _zone())
    left, right = m.query_component(5.0, 5.0), m.query_component(15.0, 5.0)
    check("pour-allowed keep-out does not split", left and left == right)

    # Old-style dict without the key (pre-#477 producers) -> no split.
    ko = _strip_ko({'F.Cu'}, True)
    del ko['copper_pour_allowed']
    m = ZoneFillModel(_board([ko]), _zone())
    left, right = m.query_component(5.0, 5.0), m.query_component(15.0, 5.0)
    check("missing copper_pour_allowed key defaults to allowed",
          left and left == right)

    # Keep-out on another layer only -> no split.
    m = ZoneFillModel(_board([_strip_ko({'B.Cu'}, False)]), _zone('F.Cu'))
    left, right = m.query_component(5.0, 5.0), m.query_component(15.0, 5.0)
    check("other-layer keep-out ignored", left and left == right)

    # Composite layer tokens (#369 A5): '*.Cu' and F&B.Cu reach the zone layer.
    m = ZoneFillModel(_board([_strip_ko({'*.Cu'}, False)]), _zone('F.Cu'))
    check("'*.Cu' keep-out splits",
          m.query_component(5.0, 5.0) != m.query_component(15.0, 5.0))
    m = ZoneFillModel(_board([_strip_ko({'F&B.Cu'}, False)]), _zone('B.Cu'))
    check("'F&B.Cu' keep-out splits B.Cu zone",
          m.query_component(5.0, 5.0) != m.query_component(15.0, 5.0))

    # Keep-out with a HOLE (even-odd): fill survives inside the hole.
    ring = {'polygon': [(8.0, 2.0), (14.0, 2.0), (14.0, 8.0), (8.0, 8.0)],
            'holes': [[(9.5, 3.5), (12.5, 3.5), (12.5, 6.5), (9.5, 6.5)]],
            'layers': {'F.Cu'}, 'tracks_allowed': True, 'vias_allowed': True,
            'copper_pour_allowed': False}
    m = ZoneFillModel(_board([ring]), _zone())
    check("keep-out hole keeps fill (even-odd)",
          m.query_component(11.0, 5.0) not in (0, None))
    check("keep-out ring body has no fill",
          m.query_component(8.7, 5.0) == 0)

    # Parser records the token (v6/v10 and absent forms).
    print("\nextract_keepouts records copperpour")
    body = ('(kicad_pcb\n'
            '\t(zone (net 0) (net_name "") (layers "F.Cu") (tstamp 0)\n'
            '\t\t(keepout (tracks allowed) (vias allowed) (copperpour not_allowed))\n'
            '\t\t(polygon (pts (xy 0 0) (xy 5 0) (xy 5 5) (xy 0 5)))\n'
            '\t)\n'
            '\t(zone (net 0) (net_name "") (layers "F.Cu") (tstamp 1)\n'
            '\t\t(keepout (tracks not_allowed) (vias not_allowed))\n'
            '\t\t(polygon (pts (xy 10 0) (xy 15 0) (xy 15 5) (xy 10 5)))\n'
            '\t)\n'
            ')\n')
    kos = extract_keepouts(body)
    check("copperpour not_allowed parsed",
          len(kos) == 2 and kos[0]['copper_pour_allowed'] is False)
    check("absent copperpour clause defaults to allowed",
          kos[1]['copper_pour_allowed'] is True)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
