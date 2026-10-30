"""#1520 follow-up: the fanout-clearance pass must measure a ROUND-CORNERED
pad (roundrect, circle, oval) as the shape check_drc grades, not as its box.

`_Cap` priced every pad by its sharp rectangle. A via whose centre sits in a
roundrect pad's box corner but off the corner ARC was priced at the full
keep-out while check_drc measured a smaller graze, so the #445 per-net
no-worse gate compared a box-priced seed with the real candidate: a move that
put the pad's face on the via (a deeper, real graze) passed as "no worse",
and the step wrote a part worse than its seed (measured: a via graze 0.227 ->
0.277, and 0.142 -> 0.177). The same simplification priced a BGA ball (a
circle) and an NPTH hole as squares.

The truth here is check_drc's own geometry on the REAL pads
(`point_to_pad_distance`, `segment_pad_gap`, `pad_copper_gap`), with the pads
turned by `legality.footprint_at_pose`; nothing is mirrored from the module
under test.

Runs in-process in a few seconds; nothing shells out.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True

import contextlib
import copy
import io
import math
import os
import sys
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from kicad_parser import parse_kicad_pcb
from check_drc import (pad_copper_gap, point_to_pad_distance,
                       segment_pad_gap)
from synth import make_seg, make_via
from placement.legality import footprint_at_pose, pad_rect
from placement.fanout_clearance import _Repair, _pad_pair_shortfall

# Declares no netclass, no .kicad_dru and no pad override: the clearance model
# is inert, so every keep-out is the flat CLEAR and nothing else moves.
BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
CLEAR = 0.1
CAP = 'C16'          # an orthogonal 0402 with roundrect pads (rratio 0.25)
BALL_FP = 'U2'       # the BGA beside it: circle balls on the cap's side
FOREIGN = 5          # neither of the cap's own nets
VIA = 0.3
KEEP = VIA / 2.0 + CLEAR


def _copper(fp):
    return [p for p in fp.pads if any(str(l).endswith('.Cu') for l in p.layers)]


def _real_pads(fp_seed, pose):
    """The cap's copper pads at `pose`, as the shapes they are."""
    return [copy.copy(p) for p in _copper(footprint_at_pose(fp_seed, pose))]


def _rig(vias=(), segs=(), keep_pads=False):
    """The real board, `vias` / `segs` the only fanout copper. Returns
    (st, cap, seed footprint)."""
    pcb = parse_kicad_pcb(BOARD)
    pcb.vias[:] = list(vias)
    pcb.segments[:] = list(segs)
    with contextlib.redirect_stdout(io.StringIO()):
        st = _Repair(pcb, BOARD, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C,R,FB', set())
    return st, st.caps[CAP], pcb.footprints[CAP]


class TestTheRigIsRoundCornered(unittest.TestCase):
    def test_the_cap_moves_and_its_pads_are_roundrect(self):
        st, cap, fp = _rig([make_via(0.0, 0.0, net_id=FOREIGN, size=VIA)])
        self.assertIn(CAP, st.caps, 'the cap is not a mover -- nothing below '
                                    'grades the pass')
        pads = _copper(fp)
        self.assertTrue(pads and all(p.shape == 'roundrect' and
                                     p.roundrect_rratio > 0.0 for p in pads),
                        'the cap pads are not round-cornered: this file tests '
                        'nothing')
        self.assertTrue(all(abs(p.rect_rotation) < 1e-9 for p in pads),
                        'the cap is tilted: this file is about corners alone')


class TestViaShortfallIsCheckDrcsDistance(unittest.TestCase):
    """`_via_shortfalls` -- the #445 gate's currency -- against check_drc's
    distance to the real pads, over a grid of vias that covers the corners."""

    def test_every_quarter_turn(self):
        pcb = parse_kicad_pcb(BOARD)
        x0, y0 = pcb.footprints[CAP].x, pcb.footprints[CAP].y
        pts = [(x0 + dx * 0.07, y0 + dy * 0.07)
               for dx in range(-8, 9) for dy in range(-8, 9)]
        nets = list(range(1000, 1000 + len(pts)))
        vias = [make_via(px, py, net_id=n, size=VIA)
                for (px, py), n in zip(pts, nets)]
        st, cap, fp = _rig(vias)
        st.cap_vias = {k: st.vias for k in st.caps}
        corner_hits = 0
        for rot in (0.0, 90.0, 180.0, 270.0):
            got = st._via_shortfalls(CAP, cap, x0, y0, rot)
            pads = _real_pads(fp, (x0, y0, rot))
            for (px, py), n in zip(pts, nets):
                want = sum(max(0.0, KEEP - point_to_pad_distance(px, py, p))
                           for p in pads)
                box = sum(max(0.0, KEEP - math.hypot(
                    max(pad_rect(p)[0] - px, px - pad_rect(p)[2], 0.0),
                    max(pad_rect(p)[1] - py, py - pad_rect(p)[3], 0.0)))
                    for p in pads)
                if box > want + 1e-3:
                    corner_hits += 1
                self.assertAlmostEqual(
                    got.get(n, 0.0), want, places=6,
                    msg='rot %g: via at (%.3f, %.3f) priced %.4f, check_drc '
                        'measures %.4f on the real pad (its box: %.4f)'
                        % (rot, px, py, got.get(n, 0.0), want, box))
        self.assertGreater(corner_hits, 20, 'no via sits where the box and the '
                                            'rounded pad differ -- the grid '
                                            'tests nothing')


class TestTrackShortfallIsCheckDrcsDistance(unittest.TestCase):
    def test_tracks_past_the_corners(self):
        pcb = parse_kicad_pcb(BOARD)
        fp0 = pcb.footprints[CAP]
        x0, y0 = fp0.x, fp0.y
        width = 0.1
        reach = width / 2.0 + CLEAR
        # at every pad's every box corner, a short track square to the
        # diagonal, at a few depths into the box corner's keep-out
        segs, nets = [], []
        for p in _copper(fp0):
            box = pad_rect(p)
            for cx, cy in ((box[0], box[1]), (box[2], box[1]),
                           (box[2], box[3]), (box[0], box[3])):
                ux = -1.0 if cx == box[0] else 1.0
                uy = -1.0 if cy == box[1] else 1.0
                for depth in (0.02, 0.06, 0.1):
                    o = (reach - depth) / math.sqrt(2.0)
                    mx, my = cx + ux * o, cy + uy * o
                    n = 2000 + len(segs)
                    segs.append(make_seg(mx - uy * 0.08, my + ux * 0.08,
                                         mx + uy * 0.08, my - ux * 0.08,
                                         net_id=n, width=width, layer='F.Cu'))
                    nets.append(n)
        st, cap, fp = _rig([make_via(x0 + 5.0, y0, net_id=FOREIGN, size=VIA)],
                           segs)
        over = 0.0
        for rot in (0.0, 90.0):
            got = st._seg_shortfalls(CAP, cap, x0, y0, rot)
            pads = _real_pads(fp, (x0, y0, rot))
            boxes = [copy.copy(p) for p in pads]
            for q in boxes:
                q.shape = 'rect'   # the same pad with sharp corners
            for s, n in zip(segs, nets):
                # check_drc's EXACT centreline-to-copper gap
                want = sum(max(0.0, s.width / 2.0 + CLEAR - segment_pad_gap(
                    s.start_x, s.start_y, s.end_x, s.end_y, p)) for p in pads)
                self.assertAlmostEqual(got.get(n, 0.0), want, places=6,
                                       msg='rot %g net %d' % (rot, n))
                over += sum(max(0.0, s.width / 2.0 + CLEAR - segment_pad_gap(
                    s.start_x, s.start_y, s.end_x, s.end_y, q))
                    for q in boxes) - want
        self.assertGreater(over, 0.1, 'the tracks do not reach where the box '
                                      'and the rounded pad differ')


class TestPadChannelsMeasureTheRoundShapes(unittest.TestCase):
    """The pad channels: a cap pad against a foreign BGA BALL (a circle), and
    two movable caps against each other, against check_drc's exact
    pad-to-pad gap."""

    def test_cap_against_the_ball_field(self):
        st, cap, fp = _rig([make_via(0.0, 0.0, net_id=FOREIGN, size=VIA)])
        pcb = parse_kicad_pcb(BOARD)
        balls = [p for p in _copper(pcb.footprints[BALL_FP])]
        self.assertTrue(balls and all(p.shape == 'circle' for p in balls))
        cap_nets = {p.net_id for p in _copper(fp)}
        # step the cap toward the nearest foreign ball until it grazes it
        ball = min((b for b in balls if b.net_id not in cap_nets),
                   key=lambda b: math.hypot(b.global_x - fp.x,
                                            b.global_y - fp.y))
        checked = 0
        for k in range(0, 41):
            t = k / 40.0
            x = fp.x + (ball.global_x - fp.x) * t * 0.9
            y = fp.y + (ball.global_y - fp.y) * t * 0.9
            for rot in (0.0, 90.0):
                pads = _real_pads(fp, (x, y, rot))
                gaps = [(p, b, pad_copper_gap(p, b)) for p in pads
                        for b in balls if b.net_id != p.net_id]
                if any(g <= 1e-9 for _p, _b, g in gaps):
                    continue   # copper on copper: no gap to compare
                want = {}
                for p, b, g in gaps:
                    if g < CLEAR:
                        want[b.net_id] = want.get(b.net_id, 0.0) + CLEAR - g
                got = st._pad_shortfalls(CAP, cap, x, y, rot)
                for net in set(want) | {n for n in got
                                        if any(b.net_id == n for b in balls)}:
                    self.assertAlmostEqual(
                        got.get(net, 0.0), want.get(net, 0.0), places=6,
                        msg='rot %g at %.3f: ball net %d priced %.4f, '
                            'check_drc measures %.4f'
                            % (rot, t, net, got.get(net, 0.0),
                               want.get(net, 0.0)))
                    checked += want.get(net, 0.0) > 0.0
        self.assertGreater(checked, 3, 'no pose grazed a ball -- the walk '
                                       'tests nothing')

    def test_cap_against_cap(self):
        st, cap, fp = _rig([make_via(0.0, 0.0, net_id=FOREIGN, size=VIA)])
        a = _real_pads(fp, (fp.x, fp.y, 0.0))
        rects_a = cap.pad_rects(fp.x, fp.y, 0.0)
        frames_a = cap.pad_frames(fp.x, fp.y, 0.0)
        checked = 0
        # a second copy of the part on a ring of offsets around the first,
        # diagonal ones included (corner against corner)
        poses = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)),
                  rot)
                 for r in (0.5, 0.6, 0.7, 1.0, 1.2)
                 for a in range(0, 360, 15) for rot in (0.0, 90.0)]
        for dx, dy, rot in poses:
            x, y = fp.x + dx, fp.y + dy
            b = _real_pads(fp, (x, y, rot))
            gaps = [pad_copper_gap(p, q) for p in a for q in b
                    if p.net_id != q.net_id]
            if any(g <= 1e-9 for g in gaps):
                continue   # copper on copper: no gap to compare
            want = sum(max(0.0, CLEAR - g) for g in gaps)
            got = _pad_pair_shortfall(list(rects_a), cap.pad_rects(x, y, rot),
                                      CLEAR, None, frames_a,
                                      cap.pad_frames(x, y, rot))
            self.assertAlmostEqual(got, want, places=6,
                                   msg='offset (%g, %g) rot %g' % (dx, dy, rot))
            checked += want > 0.0
        self.assertGreater(checked, 5, 'no pair grazes -- the rig tests nothing')


class TestTheNoWorseGateHoldsAtTheSeedsRealGraze(unittest.TestCase):
    """The defect's mechanism on an orthogonal part: a via in a pad's box
    corner but off its corner arc, and a translation that puts the pad's
    face on it."""

    def _setup(self):
        pcb = parse_kicad_pcb(BOARD)
        fp0 = pcb.footprints[CAP]
        pads = _real_pads(fp0, (fp0.x, fp0.y, fp0.rotation))
        corner = None
        for target in pads:
            box = pad_rect(target)
            for cx, cy in ((box[0], box[1]), (box[2], box[1]),
                           (box[2], box[3]), (box[0], box[3])):
                vx = cx + (0.005 if cx == box[0] else -0.005)
                vy = cy + (0.005 if cy == box[1] else -0.005)
                d = point_to_pad_distance(vx, vy, target)
                others = min(point_to_pad_distance(vx, vy, p)
                             for p in pads if p is not target)
                if 0.02 < d and others > KEEP:
                    corner = (target, vx, vy, d)
                    break
            if corner:
                break
        self.assertIsNotNone(corner, 'no box corner sits off the corner arc '
                                     'clear of the other pad -- the rig tests '
                                     'nothing')
        target, vx, vy, d_seed = corner
        st, cap, fp = _rig([make_via(vx, vy, net_id=FOREIGN, size=VIA)])
        # Isolate the via gate: no tracks and no courtyard neighbours.
        st.cap_segs = {k: [] for k in st.caps}
        st.cap_static[CAP] = []
        st.cap_caps[CAP] = []
        return st, cap, fp, target, (vx, vy, d_seed)

    def test_the_seed_baseline_is_the_real_graze(self):
        st, cap, fp, target, (vx, vy, d_seed) = self._setup()
        self.assertAlmostEqual(st.base_via[CAP].get(FOREIGN, 0.0),
                               KEEP - d_seed, places=6,
                               msg='the seed baseline is not the real graze -- '
                                   'the box prices a via off the corner arc at '
                                   'the full keep-out %.3f' % KEEP)

    def test_a_pose_that_puts_the_pad_ON_the_via_is_refused(self):
        st, cap, fp, target, (vx, vy, d_seed) = self._setup()
        x = cap.x + (vx - target.global_x)
        y = cap.y + (vy - target.global_y)
        pads = _real_pads(fp, (x, y, cap.rot))
        d_cand = min(point_to_pad_distance(vx, vy, p) for p in pads)
        self.assertLess(d_cand, d_seed - 0.015,
                        'the candidate is not really worse than the seed')
        self.assertFalse(st._blocked_geom(CAP, cap, x, y, cap.rot),
                         'a courtyard/edge gate refuses the candidate -- the '
                         'via gate is not what this asserts about')
        self.assertFalse(st._worsens_any_net(
            st._pad_shortfalls(CAP, cap, x, y, cap.rot),
            st.base_pad.get(CAP, {})))
        self.assertEqual(st.cost(CAP, cap, x, y, cap.rot), float('inf'),
                         'the gate accepted a pose whose real pad sits on the '
                         'via (real gap %.3f, seed %.3f) as no worse'
                         % (d_cand, d_seed))

    def test_NEGATIVE_CONTROL_a_pose_that_clears_the_via_is_accepted(self):
        st, cap, fp, target, (vx, vy, d_seed) = self._setup()
        ux, uy = cap.x - vx, cap.y - vy
        n = math.hypot(ux, uy)
        for k in range(1, 20):
            x, y = cap.x + ux / n * 0.05 * k, cap.y + uy / n * 0.05 * k
            pads = _real_pads(fp, (x, y, cap.rot))
            if min(point_to_pad_distance(vx, vy, p) for p in pads) > d_seed + 0.05:
                break
        self.assertFalse(st._blocked_geom(CAP, cap, x, y, cap.rot))
        self.assertLess(st.cost(CAP, cap, x, y, cap.rot), float('inf'),
                        'the gate refuses a pose that is BETTER than the seed')


if __name__ == '__main__':
    unittest.main(verbosity=2)
