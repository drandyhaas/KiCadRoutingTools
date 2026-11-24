#!/usr/bin/env python3
"""place_fanout_clearance: a part a track ends on is not moved.

The pass moves passives off foreign copper and toward same-net balls, and
leaves every track where it was. A part with a track of its own net ending on
one of its pads -- a lane's end, a pull-up joined to its lane -- moved 0.8 mm
and left that track's end on no pad: the net open, its copper laid.

`routed_refs` names those parts; `_Repair` and `movable_cap_refs` take them
out of the movers, static obstacles as a locked part is.

  1. a track of the pad's own net ending on the pad, on a layer of the pad:
     the part is named; at either end of the track;
  2. NOT named: a track of another net ending there, a track of the net
     ending off the pad, one ending on the pad's place on another layer, a
     track that only passes over the pad, a via on the pad (the pass itself
     pulls a pad onto a via of its net);
  3. a through-hole pad is on every layer.
"""
import os
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_placer', 'py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo                                  # noqa: E402
from synth import make_pad, make_pcb, make_seg, make_via            # noqa: E402
from placement.fanout_clearance import routed_refs                  # noqa: E402


def _board(segments=(), vias=(), layers=('F.Cu',), drill=0.0):
    p1 = make_pad(1, 2.0, 2.0, ref='R1', num='1', net_name='A', size_x=0.4, size_y=0.4,
                  layers=layers, drill=drill, pad_type='thru_hole' if drill else 'smd')
    p2 = make_pad(2, 3.0, 2.0, ref='R1', num='2', net_name='B', size_x=0.4, size_y=0.4)
    fps = {'R1': SimpleNamespace(reference='R1', layer='F.Cu', x=2.5, y=2.0, rotation=0.0, pads=[p1, p2])}
    bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'], board_bounds=(0.0, 0.0, 6.0, 6.0))
    return make_pcb(board_info=bi, footprints=fps, segments=list(segments), vias=list(vias), zones=[])


class TestAPartATrackEndsOn(unittest.TestCase):
    def test_named_at_either_end_of_the_track(self):
        self.assertEqual(routed_refs(_board([make_seg(2.1, 2.0, 2.0, 5.0, net_id=1)])), {'R1'})
        self.assertEqual(routed_refs(_board([make_seg(2.0, 5.0, 2.1, 2.1, net_id=1)])), {'R1'})

    def test_not_named(self):
        for what, kw in (
                ('a track of another net ending on the pad', dict(segments=[make_seg(2.0, 2.0, 2.0, 5.0, net_id=2)])),
                ('a track of the net ending off the pad', dict(segments=[make_seg(2.3, 2.0, 2.0, 5.0, net_id=1)])),
                ('one ending at the pad on another layer',
                 dict(segments=[make_seg(2.0, 2.0, 2.0, 5.0, net_id=1, layer='B.Cu')])),
                ('a track that only passes over the pad', dict(segments=[make_seg(2.0, 1.0, 2.0, 5.0, net_id=1)])),
                ('a via on the pad', dict(vias=[make_via(2.0, 2.0, net_id=1)])),
                ('no copper at all', {})):
            with self.subTest(what):
                self.assertEqual(routed_refs(_board(**kw)), set())

    def test_a_through_hole_pad_is_on_every_layer(self):
        seg = make_seg(2.0, 2.0, 2.0, 5.0, net_id=1, layer='B.Cu')
        self.assertEqual(routed_refs(_board([seg], layers=('*.Cu', '*.Mask'), drill=0.3)), {'R1'})


if __name__ == '__main__':
    unittest.main()
