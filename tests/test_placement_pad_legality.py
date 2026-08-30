"""Unit tests for the pad+drill legality layer (placement/legality.py).

Synthetic-geometry tests use lightweight fakes (only the attributes the layer
reads); two smoke tests run on real boards when present.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from placement.legality import (PartPads, LegalityContext, build_part_pads,
                                grade_pad_legality, PairShortfall,
                                _circle_rect_penetration)
import routing_defaults as defaults


class FakePad:
    def __init__(self, gx, gy, sx=1.0, sy=1.0, net=1, layers=('F.Cu',),
                 drill=0.0, pad_type='smd', rect_rotation=0.0):
        self.global_x = gx
        self.global_y = gy
        self.size_x = sx
        self.size_y = sy
        self.net_id = net
        self.layers = list(layers)
        self.drill = drill
        self.pad_type = pad_type
        self.rect_rotation = rect_rotation


class FakeFP:
    def __init__(self, ref, x=0.0, y=0.0, rotation=0.0, layer='F.Cu', pads=()):
        self.reference = ref
        self.x = x
        self.y = y
        self.rotation = rotation
        self.layer = layer
        self.pads = list(pads)


def ctx_for(fps, clearance=0.1, poses=None, seeds=None):
    parts = build_part_pads({f.reference: f for f in fps}, clearance)
    poses = poses or {f.reference: (f.x, f.y, f.rotation) for f in fps}
    seeds = seeds or dict(poses)
    return LegalityContext(parts, None, clearance,
                           pose_of=lambda r: poses[r],
                           seed_of=lambda r: seeds[r]), poses, seeds


class TestPartPads(unittest.TestCase):
    def test_pad_rect_axis_aligned(self):
        fp = FakeFP('R1', 10, 20, pads=[FakePad(10.5, 20.0, 1.0, 0.6)])
        pp = PartPads(fp, 0.1)
        (x0, y0, x1, y1, net, side) = pp.pad_rects(10, 20, 0)[0]
        self.assertAlmostEqual(x0, 10.0)
        self.assertAlmostEqual(x1, 11.0)
        self.assertAlmostEqual(y0, 19.7)
        self.assertAlmostEqual(y1, 20.3)
        self.assertEqual(side, 'F')

    def test_delta_rot_90_swaps_extents_and_rotates_offsets(self):
        fp = FakeFP('R1', 0, 0, rotation=0.0,
                    pads=[FakePad(1.0, 0.0, 1.0, 0.5)])
        pp = PartPads(fp, 0.1)
        rects = pp.pad_rects(0, 0, 90.0)
        (x0, y0, x1, y1, _n, _s) = rects[0]
        # offset (1,0) under KiCad-sign delta 90 -> (0,-1); extents swap
        self.assertAlmostEqual((x0 + x1) / 2, 0.0, places=6)
        self.assertAlmostEqual((y0 + y1) / 2, -1.0, places=6)
        self.assertAlmostEqual(x1 - x0, 0.5, places=6)
        self.assertAlmostEqual(y1 - y0, 1.0, places=6)

    def test_rect_rotation_inflates(self):
        fp = FakeFP('R1', 0, 0, pads=[FakePad(0, 0, 2.0, 1.0,
                                              rect_rotation=45.0)])
        pp = PartPads(fp, 0.1)
        (x0, y0, x1, y1, _n, _s) = pp.pad_rects(0, 0, 0)[0]
        w = x1 - x0
        c = abs(math.cos(math.radians(45)))
        self.assertAlmostEqual(w, 2 * (1.0 * c + 0.5 * c), places=6)

    def test_npth_hole_inflation_and_STANDOFF_match_the_fanout_convention(self):
        """Renamed by #761, because the old name was for the half that was
        right.

        It asserted the INFLATION -- which did match fanout_clearance -- and
        stopped there, at `clearance = 0.09` only, below the 0.20 fab floor.
        The comparison did not match: legality tested that circle against raw
        pad rects and added nothing, so the modelled standoff was
        `max(0, requirement - clearance)` and collapsed to zero at and above
        the requirement. This arm now pins BOTH halves, and the second
        clearance is above the floor, where the old assertion is vacuous."""
        for clearance in (0.09, 0.25):
            fp = FakeFP('H1', 5, 5, pads=[FakePad(5, 5, 3.2, 3.2, net=0,
                                                  layers=('*.Cu',), drill=2.2,
                                                  pad_type='np_thru_hole')])
            pp = PartPads(fp, clearance)
            with self.subTest(clearance=clearance):
                self.assertEqual(pp.n_pads, 0)          # no copper
                self.assertEqual(len(pp.holes_local), 1)
                (_ox, _oy, r) = pp.holes_local[0]
                grow = max(0.0, defaults.NPTH_TO_TRACK_CLEARANCE - clearance)
                self.assertAlmostEqual(r, 2.2 / 2.0 + grow, places=9)
                # ...and the standoff the consumers actually charge.
                (_kx, _ky, kr) = pp.hole_keepouts(5, 5, 0)[0]
                self.assertAlmostEqual(
                    kr - 2.2 / 2.0,
                    max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE),
                    places=9)

    def test_paste_only_aperture_skipped(self):
        fp = FakeFP('C1', 0, 0, pads=[FakePad(0, 0, 1, 1, layers=('F.Paste',))])
        pp = PartPads(fp, 0.1)
        self.assertEqual(pp.n_pads, 0)
        self.assertIsNone(pp.extent(0, 0, 0))


class TestPairShortfall(unittest.TestCase):
    def test_same_net_exempt_but_net0_conflicts(self):
        a = FakeFP('A', 0, 0, pads=[FakePad(0, 0, 1, 1, net=5)])
        b = FakeFP('B', 1.05, 0, pads=[FakePad(1.05, 0, 1, 1, net=5)])
        ctx, _, _ = ctx_for([a, b])
        self.assertEqual(ctx.pair_shortfall('A', 'B').pad, 0.0)
        # same geometry, net 0 on both: conflicts (unconnected pads are
        # pseudo-nets, the run-1 J10 U3.36-vs-R19.1 case)
        a0 = FakeFP('A', 0, 0, pads=[FakePad(0, 0, 1, 1, net=0)])
        b0 = FakeFP('B', 0.95, 0, pads=[FakePad(0.95, 0, 1, 1, net=0)])
        ctx0, _, _ = ctx_for([a0, b0])
        sf = ctx0.pair_shortfall('A', 'B')
        self.assertGreater(sf.pad, 0.0)
        self.assertTrue(sf.pad_overlap)     # edges 0.5 vs -0.45: 0.05 overlap

    def test_opposite_smd_sides_do_not_interact_but_tht_does(self):
        a = FakeFP('A', 0, 0, layer='F.Cu', pads=[FakePad(0, 0, 1, 1, net=1)])
        b = FakeFP('B', 0, 0, layer='B.Cu',
                   pads=[FakePad(0, 0, 1, 1, net=2, layers=('B.Cu',))])
        ctx, _, _ = ctx_for([a, b])
        self.assertEqual(ctx.pair_shortfall('A', 'B'), PairShortfall(0, False, 0))
        # a through-hole pad reaches the back side
        t = FakeFP('T', 0, 0, layer='F.Cu',
                   pads=[FakePad(0, 0, 1.6, 1.6, net=3, layers=('*.Cu',),
                                 drill=0.8, pad_type='thru_hole')])
        ctx2, _, _ = ctx_for([t, b])
        self.assertGreater(ctx2.pair_shortfall('T', 'B').pad, 0.0)

    def test_hole_penetration(self):
        self.assertAlmostEqual(
            _circle_rect_penetration(0, 0, 1.0, (2.0, -1, 3.0, 1)), 0.0)
        self.assertAlmostEqual(
            _circle_rect_penetration(0, 0, 1.0, (0.5, -1, 3.0, 1)), 0.5)
        h = FakeFP('H1', 0, 0, pads=[FakePad(0, 0, 3.2, 3.2, net=0,
                                             layers=('*.Cu',), drill=2.0,
                                             pad_type='np_thru_hole')])
        c = FakeFP('C1', 1.0, 0, pads=[FakePad(1.0, 0, 0.8, 0.8, net=7)])
        ctx, _, _ = ctx_for([h, c], clearance=0.2)
        self.assertGreater(ctx.pair_shortfall('H1', 'C1').hole, 0.0)


class TestPadsOkBaseline(unittest.TestCase):
    def make(self):
        # A and B seeded 0.05 apart edge-to-edge (shortfall at clearance 0.1)
        a = FakeFP('A', 0, 0, pads=[FakePad(0, 0, 1, 1, net=1)])
        b = FakeFP('B', 1.05, 0, pads=[FakePad(1.05, 0, 1, 1, net=2)])
        return a, b

    def test_preexisting_violation_does_not_freeze(self):
        a, b = self.make()
        ctx, poses, _ = ctx_for([a, b])
        # move A parallel to B: shortfall unchanged -> allowed
        self.assertTrue(ctx.pads_ok('A', 0.0, 0.5, 0.0, ['B']))

    def test_worsening_rejected_improving_allowed(self):
        a, b = self.make()
        ctx, _, _ = ctx_for([a, b])
        self.assertFalse(ctx.pads_ok('A', 0.5, 0.0, 0.0, ['B']))   # closer
        self.assertTrue(ctx.pads_ok('A', -0.5, 0.0, 0.0, ['B']))   # farther

    def test_new_overlap_never_admitted(self):
        # seed poses CLEAR (gap 0.5 > clearance): baseline zero; a candidate
        # that intersects B is rejected outright
        a = FakeFP('A', 0, 0, pads=[FakePad(0, 0, 1, 1, net=1)])
        b = FakeFP('B', 1.6, 0, pads=[FakePad(1.6, 0, 1, 1, net=2)])
        ctx, _, _ = ctx_for([a, b])
        self.assertFalse(ctx.pads_ok('A', 1.3, 0.0, 0.0, ['B']))

    def test_exclude_and_unknown_ref(self):
        a, b = self.make()
        ctx, _, _ = ctx_for([a, b])
        self.assertTrue(ctx.pads_ok('A', 0.5, 0.0, 0.0, ['B'], exclude={'B'}))
        self.assertTrue(ctx.pads_ok('ZZ', 0, 0, 0, ['B']))


class TestGradeRealBoards(unittest.TestCase):
    def _grade(self, path, clearance):
        from kicad_parser import parse_kicad_pcb
        return grade_pad_legality(parse_kicad_pcb(path), clearance)

    def test_control_tigard_clean(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'kicad_files',
                            'tigard.kicad_pcb')
        if not os.path.exists(path):
            self.skipTest('tigard fixture not present')
        g = self._grade(path, 0.09)
        self.assertEqual(g['pad_conflicts'], 0, g['worst'])
        self.assertEqual(g['hole_conflicts'], 0)

    def test_perturbed_tigard_flags(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'wk', 'b2',
                            'tigard__swap', 'd0', 'perturbed.kicad_pcb')
        if not os.path.exists(path):
            self.skipTest('perturbed corpus board not present')
        g = self._grade(path, 0.09)
        self.assertGreater(g['pad_conflicts'], 0)
        self.assertTrue(g['exact'])


if __name__ == '__main__':
    unittest.main()
