#!/usr/bin/env python3
"""#1564: the raster fill model fills only the BOARD, at KiCad's edge gap.

KiCad's zone filler clips every fill to the board outline and knocks each
Edge.Cuts item out at the board's min_copper_edge_clearance (the edge rule
alone, whatever the zone's clearance: probed for #1643) before its
min-thickness opening. ZoneFillModel
clipped only to the ZONE polygon. A plane drawn as the board's bounding box
-- which route_planes draws on every board -- overhangs each notch of a
non-rectangular board, and the model poured the notch and the edge band:
two pours KiCad keeps apart read as one, so the split was never repaired and
only KiCad's refill saw the open. What each case pins:

* a pour split by a foreign track that runs into a notch of the outline is
  two components: no fill in the notch, none around the track's end there;
* the edge gap is the board's copper-to-edge rule: a strip between the
  edge and a foreign track that is wide enough at a 0.05 mm rule, but
  thinner than min_thickness at a 0.3 mm rule, joins the two sides only at
  the 0.05 rule;
* a rectangular board (no outline ring, only bounds) is clipped to its
  bounds, and a board object with neither is left as it was.

    python3 tests/test_1564_fill_model_board_outline.py [case-substring ...]
"""
import json
import os
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from plane_fill_model import ZoneFillModel  # noqa: E402

# 20 x 10 board with a 4 x 4 notch out of its top edge (x 8..12, y 0..4);
# the zone is the board's bounding box, as route_planes draws it.
NOTCHED = [(0.0, 0.0), (8.0, 0.0), (8.0, 4.0), (12.0, 4.0), (12.0, 0.0),
           (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
BBOX = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]


def _seg(y_top, net=2):
    """A foreign 0.2 mm track down x=10 from y_top to below the board."""
    return SimpleNamespace(net_id=net, layer='B.Cu', width=0.2,
                           start_x=10.0, start_y=y_top,
                           end_x=10.0, end_y=10.5)


def _board(outlines=(), bounds=None, segments=(), source_path=''):
    bi = SimpleNamespace(keepouts=[], board_outlines=[list(o) for o in outlines],
                         board_cutouts=[], board_edge_contours=[],
                         board_bounds=bounds)
    return SimpleNamespace(vias=[], segments=list(segments), pads_by_net={},
                           zones=[], board_info=bi, source_path=source_path)


def _zone():
    return SimpleNamespace(net_id=1, layer='B.Cu', clearance=0.1,
                           min_thickness=0.1, priority=0, polygon=list(BBOX))


def _same(m, a, b):
    ca, cb = m.query_component(*a), m.query_component(*b)
    assert ca and cb, f"fixture: no fill at {a} ({ca}) or {b} ({cb})"
    return ca == cb


LEFT, RIGHT = (5.0, 7.0), (15.0, 7.0)


def test_notch_does_not_join_the_pour():
    # the track's top end sits inside the notch, off the board
    m = ZoneFillModel(_board([NOTCHED], (0, 0, 20, 10), [_seg(3.8)]),
                      _zone())
    assert m.ok
    assert m.query_component(10.0, 2.0) == 0, \
        "the model pours the notch, which is off the board"
    assert not _same(m, LEFT, RIGHT), \
        "the two sides of the track join through the off-board notch"
    print("  PASS: no fill in the notch; the track splits the pour")


def _edge_strip_board(td, rule):
    """Track copper top 0.45 below the notch's bottom edge (y=4), whose
    zone-clearance band ends at 4.35: free from 4.05 at a 0.05 rule, from
    4.3 at a 0.3 rule (the edge gap is the rule alone, #1643)."""
    pcb = os.path.join(td, f'b{rule}.kicad_pcb')
    with open(pcb, 'w') as f:
        f.write('(kicad_pcb)\n')
    with open(os.path.splitext(pcb)[0] + '.kicad_pro', 'w') as f:
        json.dump({'board': {'design_settings': {'rules': {
            'min_copper_edge_clearance': rule}}}}, f)
    return _board([NOTCHED], (0, 0, 20, 10), [_seg(4.55)], source_path=pcb)


def test_edge_rule_closes_a_thin_strip():
    with tempfile.TemporaryDirectory() as td:
        m0 = ZoneFillModel(_edge_strip_board(td, 0.05), _zone())
        m3 = ZoneFillModel(_edge_strip_board(td, 0.3), _zone())
    assert m0.ok and m3.ok
    assert _same(m0, LEFT, RIGHT), \
        "fixture: at a 0.05 edge rule the 0.30 mm strip must join the sides"
    assert not _same(m3, LEFT, RIGHT), \
        "the board's 0.3 mm copper-to-edge rule is not applied to the fill"
    print("  PASS: the strip joins at a 0.05 mm edge rule, not at a "
          "0.3 mm one")


def test_rectangle_bounds_and_no_outline():
    # A plain rectangle parses with no outline ring: its bounds are the
    # ring, so a zone larger than the board pours nothing past them.
    big = SimpleNamespace(net_id=1, layer='B.Cu', clearance=0.1,
                          min_thickness=0.1, priority=0,
                          polygon=[(-2.0, -2.0), (22.0, -2.0), (22.0, 12.0),
                                   (-2.0, 12.0)])
    m = ZoneFillModel(_board((), (0, 0, 20, 10)), big)
    assert m.ok and m.query_component(-1.0, 5.0) == 0, \
        "fill past the bounds of a rectangular board"
    assert m.query_component(5.0, 5.0), "no fill on the board"
    # neither outline nor bounds: the zone polygon is the only clip
    m = ZoneFillModel(_board(), big)
    assert m.ok and m.query_component(-1.0, 5.0), \
        "a board object without an outline lost its fill"
    print("  PASS: bounds clip a rectangular board; no outline, no clip")


TESTS = [test_notch_does_not_join_the_pour,
         test_edge_rule_closes_a_thin_strip,
         test_rectangle_bounds_and_no_outline]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
