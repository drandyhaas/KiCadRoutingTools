#!/usr/bin/env python3
"""place_fanout_clearance: a move may not wall in a pad of the part it moves.

The pass prices clearance alone, so a pose between two locked tracks is legal
to it. A pull toward a same-net ball slid a decoupling capacitor into the
corridor of two locked lanes: the part's other pad the corridor's end, no via
site on the layers above, and the pad's net shipped open at any size --
reachable where the part stood, walled where it was put.

`_OwnPadGate` asks `placement.reachability` (the fab ladder's finest rung, the
copper no later step moves) about the pads of the part at a pose, and refuses
a pose that walls a pad the part's seed left reachable. A refused pose is
priced infinite by `_Repair.cost`, which the descent and every fallback read.

THE RIG: a two-layer board, a two-pad part C1 on F.Cu (pad 1 on net 1, pad 2
on net 2, each net with a second pad to reach), and a dead-end corridor of
locked F.Cu tracks of another net, 0.7 mm wide inside, a locked B.Cu track
beneath it so no via leaves it. C1 fits in the corridor with 0.15 mm above and
below its pads: no track of any rung passes its outer pad to the inner one.

WHAT THIS FILE HOLDS:
  1. in the corridor the inner pad is walled and the outer is not; turned
     half way round it is the other pad; outside the corridor neither is;
  2. `refuses`: the corridor pose from a seed outside it, recorded where
     cost() reads it; a pose outside the corridor is not;
  3. a part whose SEED stands in the corridor is no worse for staying;
  4. walls a router may rip (not locked) wall nothing in;
  5. `_Repair.cost` prices a refused pose infinite before any other term.
"""
import os
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_placer', 'py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo                                  # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg            # noqa: E402
import placement.fanout_clearance as FC                             # noqa: E402

SEED = (2.0, 5.0, 0.0)
IN_CORRIDOR = (5.0, 5.0, 0.0)
OUTSIDE = (3.0, 5.0, 0.0)
OTHER = 9                       # the corridor's net


def _board(locked=True, seed=SEED):
    """(pcb, st): the rig, C1 at `seed`; `locked` False leaves the corridor
    copper a router's to rip."""
    x, y, _rot = seed
    p1 = make_pad(1, x - 0.3, y, ref='C1', num='1', net_name='A', size_x=0.3, size_y=0.4)
    p2 = make_pad(2, x + 0.3, y, ref='C1', num='2', net_name='B', size_x=0.3, size_y=0.4)
    t1 = make_pad(1, 5.0, 7.0, ref='T1', num='1', net_name='A')
    t2 = make_pad(2, 5.0, 3.0, ref='T2', num='1', net_name='B')
    fps = {'C1': SimpleNamespace(reference='C1', layer='F.Cu', x=x, y=y, rotation=0.0, pads=[p1, p2]),
           'T1': SimpleNamespace(reference='T1', layer='F.Cu', x=5.0, y=7.0, rotation=0.0, pads=[t1]),
           'T2': SimpleNamespace(reference='T2', layer='F.Cu', x=5.0, y=3.0, rotation=0.0, pads=[t2])}
    wall = dict(net_id=OTHER, width=0.1, locked=locked)
    segs = [make_seg(4.0, 5.4, 6.0, 5.4, **wall), make_seg(4.0, 4.6, 6.0, 4.6, **wall),
            make_seg(6.0, 4.6, 6.0, 5.4, **wall),
            make_seg(3.6, 5.0, 6.4, 5.0, layer='B.Cu', net_id=OTHER, width=1.4, locked=locked)]
    nets = {1: make_net(1, 'A', pads=[p1, t1]), 2: make_net(2, 'B', pads=[p2, t2]),
            OTHER: make_net(OTHER, 'W', pads=[])}
    bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'], board_bounds=(0.0, 0.0, 10.0, 10.0))
    pcb = make_pcb(board_info=bi, nets=nets, footprints=fps, segments=segs,
                   pads_by_net={1: [p1, t1], 2: [p2, t2]}, zones=[])
    cap = SimpleNamespace(x=x, y=y, rot=0.0, seed_x=x, seed_y=y, seed_rot=0.0)
    st = SimpleNamespace(caps={'C1': cap}, own_pad_refused=set())
    return pcb, st


class TestAPoseThatWallsAPadIn(unittest.TestCase):
    def test_the_inner_pad_of_the_corridor_is_walled(self):
        pcb, st = _board()
        g = FC._OwnPadGate(st, pcb)
        self.assertEqual(g.walled('C1', *IN_CORRIDOR), frozenset({'2'}))
        self.assertEqual(g.walled('C1', 5.0, 5.0, 180.0), frozenset({'1'}),
                         'turned half way round the other pad is the inner one')
        self.assertEqual(g.walled('C1', *OUTSIDE), frozenset())
        self.assertEqual(g.seed_walled('C1'), frozenset())

    def test_refused_and_recorded_where_cost_reads_it(self):
        pcb, st = _board()
        g = FC._OwnPadGate(st, pcb)
        self.assertFalse(g.refuses('C1', *OUTSIDE))
        self.assertEqual(st.own_pad_refused, set())
        self.assertTrue(g.refuses('C1', *IN_CORRIDOR))
        self.assertEqual(st.own_pad_refused, {FC._pose_key('C1', *IN_CORRIDOR)})
        self.assertEqual(g.refused, {'C1': [(5.0, 5.0, 0.0, ['2'])]})

    def test_a_seed_in_the_corridor_is_no_worse_for_staying(self):
        pcb, st = _board(seed=IN_CORRIDOR)
        g = FC._OwnPadGate(st, pcb)
        self.assertEqual(g.seed_walled('C1'), frozenset({'2'}))
        self.assertFalse(g.refuses('C1', *IN_CORRIDOR))
        self.assertEqual(st.own_pad_refused, set())

    def test_copper_a_router_may_rip_walls_nothing_in(self):
        pcb, st = _board(locked=False)
        g = FC._OwnPadGate(st, pcb)
        self.assertEqual(g.walled('C1', *IN_CORRIDOR), frozenset())
        self.assertFalse(g.refuses('C1', *IN_CORRIDOR))


class TestCostPricesARefusedPoseInfinite(unittest.TestCase):
    def test_before_any_other_term(self):
        def never(*_a, **_k):
            raise AssertionError('cost() read past the refused pose')
        st = SimpleNamespace(_tethers=None, _blocked_geom=lambda *a: False,
                             own_pad_refused={FC._pose_key('C1', *IN_CORRIDOR)},
                             _seg_shortfalls=never, _pad_shortfalls=never, _via_shortfalls=never)
        self.assertEqual(FC._Repair.cost(st, 'C1', None, *IN_CORRIDOR), float('inf'))
        with self.assertRaises(AssertionError):
            FC._Repair.cost(st, 'C1', None, *OUTSIDE)


if __name__ == '__main__':
    unittest.main()
