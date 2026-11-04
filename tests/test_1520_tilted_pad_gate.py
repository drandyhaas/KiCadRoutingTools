"""#1520: the fanout-clearance pass must measure a TILTED pad as the rotated
rectangle it is, at the seed and at every pose it offers.

`_Cap` modelled every pad by its axis-aligned box (`pad_copper_box`). For a
part on a non-orthogonal angle the box over-states the pad by a different
amount in each direction, so the #445 per-net no-worse gate compared two
different errors: a part seeded at 110 degrees with a via centre in its box's
empty corner had a real graze of 0.085 mm priced at the full keep-out 0.215,
and a pose that put the real pad face on the via (0.215, a short) passed as
"no worse than the seed". The part was written there and reported unresolved.
A turn off the quarter-turn lattice (110 -> 270, an undeclared part offered
the absolute ROTATIONS) also kept the seed box's half extents instead of the
turned pad's.

The truth here is independent of `_Cap`: `legality.footprint_at_pose` turns
the REAL pads (the parser's size_x/size_y/rect_rotation transform), and
check_drc measures them as they are -- round corners included, which `_Cap`
models too (`tests/test_1520_round_corner_gate.py` isolates that half).

Runs in-process in a few seconds; nothing shells out.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True

import copy
import io
import contextlib
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
from check_drc import point_to_pad_distance, segment_pad_gap
from synth import make_seg, make_via
from placement.legality import footprint_at_pose, pad_rect
from placement.fanout_clearance import _Repair

# Declares no netclass, no .kicad_dru and no pad override: the clearance model
# is inert, so a via's keep-out is its radius + CLEAR and nothing else moves.
BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
CLEAR = 0.1
CAP = 'C16'          # a decoupling cap with clear board on both sides
TILT = 110.0         # the seed angle: its pads tilt 70 degrees
FOREIGN = 5          # neither of the cap's own nets
VIA = 0.3


def _copper(fp):
    return [p for p in fp.pads if any(str(l).endswith('.Cu') for l in p.layers)]


def _real_pads(fp_seed, pose):
    """The cap's copper pads at `pose`, turned by the parser's own transform,
    graded as the shapes they are."""
    return [copy.copy(p) for p in _copper(footprint_at_pose(fp_seed, pose))]


def _rig(vias=(), segs=()):
    """The real board with the cap turned to TILT, and `vias` / `segs` the
    only fanout copper. Returns (st, cap, seed footprint)."""
    pcb = parse_kicad_pcb(BOARD)
    fp = footprint_at_pose(pcb.footprints[CAP], (pcb.footprints[CAP].x,
                                                 pcb.footprints[CAP].y, TILT))
    pcb.footprints[CAP] = fp
    pcb.vias[:] = list(vias)
    pcb.segments[:] = list(segs)
    with contextlib.redirect_stdout(io.StringIO()):
        st = _Repair(pcb, BOARD, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C,R,FB', set())
    return st, st.caps[CAP], fp


class TestTheRigIsOnTheBranch(unittest.TestCase):
    def test_the_cap_moves_and_its_pads_are_tilted(self):
        st, cap, fp = _rig([make_via(0.0, 0.0, net_id=FOREIGN, size=VIA)])
        self.assertIn(CAP, st.caps, 'the turned cap is not a mover -- '
                                    'nothing below grades the pass')
        tilts = [p.rect_rotation % 90.0 for p in _copper(fp)]
        self.assertTrue(all(abs(t) > 1.0 for t in tilts),
                        'the seed pads are not tilted (%s): this file tests '
                        'nothing' % tilts)
        self.assertEqual(len(cap.pad_rects()), len(_copper(fp)))


class TestViaShortfallIsMeasuredOnTheRealPad(unittest.TestCase):
    """`_via_shortfalls` -- the #445 gate's currency -- against check_drc's
    distance to the real turned pad, at the seed and at turned poses."""

    def test_every_pose_prices_what_check_drc_measures(self):
        pcb = parse_kicad_pcb(BOARD)
        x0, y0 = pcb.footprints[CAP].x, pcb.footprints[CAP].y
        # one via per foreign net, so a per-net value is one via's shortfall
        pts = [(x0 + dx * 0.11, y0 + dy * 0.11)
               for dx in range(-5, 6) for dy in range(-5, 6)]
        nets = list(range(1000, 1000 + len(pts)))
        vias = [make_via(px, py, net_id=n, size=VIA)
                for (px, py), n in zip(pts, nets)]
        st, cap, fp = _rig(vias)
        st.cap_segs = {k: [] for k in st.caps}
        st.cap_vias = {k: st.vias for k in st.caps}
        keep = VIA / 2.0 + CLEAR
        worst = 0.0
        for rot in (TILT, 0.0, 90.0, 180.0, 270.0, 200.0, 155.0):
            got = st._via_shortfalls(CAP, cap, x0, y0, rot)
            pads = _real_pads(fp, (x0, y0, rot))
            for (px, py), n in zip(pts, nets):
                # summed over the part's pads, as the pass prices it
                want = sum(max(0.0, keep - point_to_pad_distance(px, py, p))
                           for p in pads)
                worst = max(worst, abs(got.get(n, 0.0) - want))
                self.assertAlmostEqual(
                    got.get(n, 0.0), want, places=6,
                    msg='rot %g: via at (%.3f, %.3f) priced %.4f, check_drc '
                        'measures %.4f on the real pad'
                        % (rot, px, py, got.get(n, 0.0), want))
        self.assertLess(worst, 1e-6)
    # MUTATION: price a tilted pad by its box again (`_pad_point_dist`
    # ignoring the frame) -> the seed pose over-prices the corner vias.


class TestTheNoWorseGateHoldsAtTheSeedsRealGraze(unittest.TestCase):
    """The issue's mechanism, at the issue's angle: a via in the seed box's
    empty corner, and a translation that puts the real pad face on it."""

    def _setup(self):
        pcb = parse_kicad_pcb(BOARD)
        fp0 = footprint_at_pose(pcb.footprints[CAP], (pcb.footprints[CAP].x,
                                                      pcb.footprints[CAP].y,
                                                      TILT))
        pads = _real_pads(fp0, (fp0.x, fp0.y, TILT))
        target = pads[0]
        box = pad_rect(target)
        # a box corner that lies OUTSIDE the real pad, nudged into the box
        corner = None
        for cx, cy in ((box[0], box[1]), (box[2], box[1]),
                       (box[2], box[3]), (box[0], box[3])):
            vx = cx + (0.02 if cx == box[0] else -0.02)
            vy = cy + (0.02 if cy == box[1] else -0.02)
            d = point_to_pad_distance(vx, vy, target)
            others = min(point_to_pad_distance(vx, vy, p) for p in pads[1:])
            if 0.05 < d < VIA / 2.0 + CLEAR and others > VIA / 2.0 + CLEAR:
                corner = (vx, vy, d)
                break
        self.assertIsNotNone(corner, 'no box corner sits off the real pad '
                                     'within reach -- the rig tests nothing')
        vx, vy, d_seed = corner
        st, cap, fp = _rig([make_via(vx, vy, net_id=FOREIGN, size=VIA)])
        # Isolate the via gate: no tracks, and no courtyard neighbours (the
        # turned cap's courtyard already overlaps its neighbours, which would
        # refuse every move for a reason this file is not about).
        st.cap_segs = {k: [] for k in st.caps}
        st.cap_static[CAP] = []
        st.cap_caps[CAP] = []
        return st, cap, fp, target, (vx, vy, d_seed)

    def test_the_seed_baseline_is_the_real_graze(self):
        st, cap, fp, target, (vx, vy, d_seed) = self._setup()
        keep = VIA / 2.0 + CLEAR
        self.assertAlmostEqual(st.base_via[CAP].get(FOREIGN, 0.0),
                               keep - d_seed, places=6,
                               msg='the seed baseline is not the real graze -- '
                                   'the box over-prices a via in its empty corner '
                                   '(keep-out %.3f)'
                                   % keep)

    def test_a_pose_that_puts_the_pad_ON_the_via_is_refused(self):
        st, cap, fp, target, (vx, vy, d_seed) = self._setup()
        # translate so the target pad's centre lands on the via
        x = cap.x + (vx - target.global_x)
        y = cap.y + (vy - target.global_y)
        pads = _real_pads(fp, (x, y, TILT))
        d_cand = min(point_to_pad_distance(vx, vy, p) for p in pads)
        self.assertLess(d_cand, d_seed - 0.03,
                        'the candidate is not really worse than the seed')
        # ON THE BRANCH: no other gate refuses this pose
        self.assertFalse(st._blocked_geom(CAP, cap, x, y, TILT),
                         'a courtyard/edge gate refuses the candidate -- the '
                         'via gate is not what this asserts about')
        self.assertFalse(st._worsens_any_net(
            st._pad_shortfalls(CAP, cap, x, y, TILT), st.base_pad.get(CAP, {})))
        self.assertEqual(st.cost(CAP, cap, x, y, TILT), float('inf'),
                         'the gate accepted a pose whose real pad sits on the '
                         'via (real gap %.3f, seed %.3f) as no worse'
                         % (d_cand, d_seed))

    def test_NEGATIVE_CONTROL_a_pose_that_clears_the_via_is_accepted(self):
        st, cap, fp, target, (vx, vy, d_seed) = self._setup()
        # step the part directly away from the via until its pads clear it
        ux, uy = cap.x - vx, cap.y - vy
        n = math.hypot(ux, uy)
        for k in range(1, 20):
            x, y = cap.x + ux / n * 0.05 * k, cap.y + uy / n * 0.05 * k
            pads = _real_pads(fp, (x, y, TILT))
            if min(point_to_pad_distance(vx, vy, p) for p in pads) > d_seed + 0.05:
                break
        self.assertFalse(st._blocked_geom(CAP, cap, x, y, TILT))
        self.assertLess(st.cost(CAP, cap, x, y, TILT), float('inf'),
                        'the gate refuses a pose that is BETTER than the seed')


class TestATurnOffTheLatticeTurnsThePad(unittest.TestCase):
    def test_box_extents_follow_the_turned_pad(self):
        st, cap, fp = _rig([make_via(0.0, 0.0, net_id=FOREIGN, size=VIA)])
        for rot in (0.0, 90.0, 180.0, 270.0, 200.0):
            got = cap.pad_rects(cap.x, cap.y, rot)
            pads = _real_pads(fp, (cap.x, cap.y, rot))
            for g, p in zip(got, pads):
                w = pad_rect(p)
                for a, b in zip(g[:4], w):
                    self.assertAlmostEqual(
                        a, b, places=6,
                        msg='rot %g: pad box %s, the turned pad is %s'
                            % (rot, [round(v, 3) for v in g[:4]],
                               [round(v, 3) for v in w]))
    # MUTATION: restore the half-extent swap for every delta -> at 0/180 (a
    # turn of 250/70 degrees) the seed box's extents are kept.


class TestTrackShortfallIsMeasuredOnTheRealPad(unittest.TestCase):
    def test_a_track_across_the_box_corner(self):
        pcb = parse_kicad_pcb(BOARD)
        x0, y0 = pcb.footprints[CAP].x, pcb.footprints[CAP].y
        segs, nets = [], []
        for k, (dx, dy) in enumerate(((-0.5, -0.3), (-0.4, 0.35), (0.45, -0.3),
                                      (0.3, 0.42), (0.0, 0.55))):
            n = 2000 + k
            segs.append(make_seg(x0 + dx - 0.6, y0 + dy, x0 + dx + 0.6,
                                 y0 + dy + 0.1, net_id=n, width=0.1,
                                 layer='F.Cu'))
            nets.append(n)
        st, cap, fp = _rig([make_via(x0 + 5.0, y0, net_id=FOREIGN, size=VIA)],
                           segs)
        for rot in (TILT, 0.0, 200.0):
            got = st._seg_shortfalls(CAP, cap, x0, y0, rot)
            pads = _real_pads(fp, (x0, y0, rot))
            for s, n in zip(segs, nets):
                want = 0.0
                for p in pads:
                    # check_drc's EXACT centreline-to-copper gap (its graded
                    # pad-segment pass samples the segment, so it differs in
                    # the 5th decimal)
                    d = segment_pad_gap(s.start_x, s.start_y, s.end_x,
                                        s.end_y, p)
                    want += max(0.0, s.width / 2.0 + CLEAR - d)
                self.assertAlmostEqual(got.get(n, 0.0), want, places=6,
                                       msg='rot %g net %d' % (rot, n))


class TestAnOrthogonalPartIsUnchanged(unittest.TestCase):
    """The un-turned cap's pads are axis-aligned, and are priced at what
    check_drc measures on them at every quarter turn."""

    def test_untilted_cap(self):
        pcb = parse_kicad_pcb(BOARD)
        fp = pcb.footprints[CAP]
        self.assertTrue(all(abs(p.rect_rotation) < 1e-9 for p in _copper(fp)))
        vias = [make_via(fp.x + 0.3, fp.y + 0.25, net_id=FOREIGN, size=VIA)]
        pcb.vias[:] = vias
        with contextlib.redirect_stdout(io.StringIO()):
            st = _Repair(pcb, BOARD, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3,
                         'C,R,FB', set())
        cap = st.caps[CAP]
        st.cap_vias = {k: st.vias for k in st.caps}
        for rot in (0.0, 90.0, 180.0, 270.0):
            got = st._via_shortfalls(CAP, cap, cap.x, cap.y, rot)
            v = vias[0]
            want = sum(max(0.0, VIA / 2.0 + CLEAR
                           - point_to_pad_distance(v.x, v.y, p))
                       for p in _real_pads(fp, (cap.x, cap.y, rot)))
            self.assertAlmostEqual(got.get(FOREIGN, 0.0), want, places=9)


class TestPadToPadGapOnFrames(unittest.TestCase):
    """The pad channel's gap between two pads either of which may be tilted,
    against a brute-force measurement on the two outlines. (A unit of the new
    helper: on the old code this test cannot import it.)"""

    @staticmethod
    def _outline(fr, n=200):
        cx, cy, hx, hy, c, s = fr[:6]
        pts = []
        for (ax, ay), (bx, by) in (((-hx, -hy), (hx, -hy)), ((hx, -hy), (hx, hy)),
                                   ((hx, hy), (-hx, hy)), ((-hx, hy), (-hx, -hy))):
            for k in range(n):
                t = k / n
                lx, ly = ax + t * (bx - ax), ay + t * (by - ay)
                pts.append((cx + lx * c - ly * s, cy + lx * s + ly * c))
        return pts

    def test_against_brute_force(self):
        from placement.fanout_clearance import _pad_gap, _tilt_frame
        cases = [((0.0, 0.0, 0.3, 0.2, 30.0), (0.9, 0.1, 0.25, 0.25, 0.0)),
                 ((0.0, 0.0, 0.3, 0.2, 70.0), (0.0, 0.75, 0.2, 0.1, 45.0)),
                 ((0.0, 0.0, 0.3, 0.2, 20.0), (0.55, 0.3, 0.2, 0.1, 0.0)),
                 ((0.0, 0.0, 0.3, 0.2, 110.0), (0.2, 0.1, 0.2, 0.2, 15.0))]
        for a, b in cases:
            fa = _tilt_frame(*a) or (a[0], a[1], a[2], a[3], 1.0, 0.0, 0.0)
            fb = _tilt_frame(*b) or (b[0], b[1], b[2], b[3], 1.0, 0.0, 0.0)
            ra = (a[0] - a[2], a[1] - a[3], a[0] + a[2], a[1] + a[3])
            rb = (b[0] - b[2], b[1] - b[3], b[0] + b[2], b[1] + b[3])
            got = _pad_gap(ra, _tilt_frame(*a), rb, _tilt_frame(*b))
            pa, pb = self._outline(fa), self._outline(fb)
            brute = min(math.hypot(x1 - x2, y1 - y2)
                        for x1, y1 in pa for x2, y2 in pb)
            # inside each other's outline -> overlapping
            def inside(p, fr):
                dx, dy = p[0] - fr[0], p[1] - fr[1]
                lx, ly = dx * fr[4] + dy * fr[5], -dx * fr[5] + dy * fr[4]
                return abs(lx) < fr[2] and abs(ly) < fr[3]
            overlap = any(inside(p, fb) for p in pa) or any(inside(p, fa) for p in pb)
            if overlap:
                self.assertLess(got, 0.0, (a, b, got))
            else:
                self.assertAlmostEqual(got, brute, delta=0.005, msg=(a, b))


if __name__ == '__main__':
    unittest.main(verbosity=2)
