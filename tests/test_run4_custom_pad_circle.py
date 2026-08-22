"""Run-4 B6: custom-pad circle primitives must carry clearance-grade geometry.

Run 3's DRC verifier lens found a 0.0884mm-vs-0.09 track-to-pad graze that
kicad-cli flagged and check_drc missed: the pad's copper is a gr_circle
PRIMITIVE, and the fixed inscribed 32-gon under-reached the true circle by
~2.4um at r=0.5 -- more than the 1.6um defect. Run-4 B6 moved the primitive
path to adaptive tessellation (1um sagitta, capped 64 vertices); that change
has since been REVERTED -- see below, which is what this file now pins.

Both directions are pinned: a real sub-floor gap is flagged, and a gap
safely above the floor is NOT (the polygon stays inscribed, so tightening
it cannot manufacture phantom violations).

THE ADAPTIVE TESSELLATION WAS REVERTED in 6166a98b ("revert six
placement-only routing changes that lose to main on the corpus"): the parser
is back to a fixed inscribed 32-gon, because the adaptive version moved
obstacle geometry AND DRC grading together on every board with
circle-primitive custom pads -- a baseline shift no re-grade can separate
(measured: real DRC 71 -> 49 across 73 boards). `_adaptive_circle_n` is
RETAINED but WIRED TO NOTHING, per that commit's "restore if the accuracy is
wanted, but land it as a deliberate, measured baseline change".

So this file pins the SHIPPED geometry, not the reverted one: the 32-gon's
vertex count and inscribed-ness, and the consequence on the run-3 board --
the graze is masked at the 0.09 floor and reappears once the floor clears the
sagitta. Restoring the adaptive path fails these, which is the point: it
should be a decision, not a drift.
"""

import math
import os
import re
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
# #522: the engine lives in py_router/, the placer in py_placer/ --
# ROOT alone does not make `import kicad_parser` work.
for _p522 in ('py_router', 'py_placer'):
    _d522 = os.path.join(ROOT, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)
# #522 reorg + skill merge: the engine moved to py_router/, the placer to
# py_placer/, and board_score.py into the placement-and-routing skill. Tests
# that shell out to or import them need those roots on sys.path.
for _p in ('py_router', 'py_placer',
           os.path.join('.claude', 'skills', 'plan-pcb-placement-and-routing',
                        'scripts')):
    _d = os.path.join(ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)

RUN3_BOARD = os.path.join(ROOT, 'wk', 'run3', 'final2.kicad_pcb')


def board_with_gap(gap_mm: float) -> str:
    """A custom pad whose copper is ONE gr_circle primitive (r=0.5) beside a
    1x1 rect pad on another net, true edge-to-edge gap ``gap_mm``."""
    # circle copper edge at x = 50 + 0.5; rect pad left edge at that + gap
    rect_cx = 50 + 0.5 + gap_mm + 0.5
    return (
        '(kicad_pcb (version 20221018) (generator pcbnew)\n'
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
        '  (net 0 "") (net 1 "A") (net 2 "B")\n'
        '  (gr_rect (start 40 40) (end 70 60) (stroke (width 0.1) (type default))'
        ' (layer "Edge.Cuts"))\n'
        '  (footprint "t:JP" (layer "F.Cu") (at 50 50)\n'
        '    (property "Reference" "JP1" (at 0 0) (layer "F.SilkS"))\n'
        '    (pad "1" smd custom (at 0 0) (size 0.1 0.1) (layers "F.Cu")'
        ' (net 1 "A")\n'
        '      (options (clearance outline) (anchor circle))\n'
        '      (primitives (gr_circle (center 0 0) (end 0.45 0)'
        ' (width 0.1) (fill yes)))))\n'
        f'  (footprint "t:R" (layer "F.Cu") (at {rect_cx:.4f} 50)\n'
        '    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))\n'
        '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 2 "B")))\n'
        ')\n')


def run_drc(board_path, *argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8')
    return subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
         board_path, *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestAdaptivePrimitiveCircle(unittest.TestCase):
    def _drc_at(self, gap):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'b.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as f:
                f.write(board_with_gap(gap))
            return run_drc(p, '-c', '0.09', '--clearance-margin', '0',
                           '--max-print', '0')

    def test_sub_floor_gap_is_flagged(self):
        # NOTE: this pair covers check_drc's custom-pad path, NOT tessellation.
        # board_with_gap puts the rect due EAST of the circle centre, and the
        # polygon's vertex k=0 sits at angle 0 -- exactly the true rightmost
        # point (measured: max_x is 50.500000 at N=32). So the measured gap
        # equals the true gap for any N and both directions pass whatever the
        # tessellation does. The vertex count is pinned directly in
        # TestShippedTessellation below.
        # The run-3 geometry: true gap 0.0884 vs floor 0.09 (1.6um short).
        r = self._drc_at(0.0884)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn('PAD-PAD', r.stdout)

    def test_above_floor_gap_stays_clean(self):
        # Inscribed polygons cannot manufacture phantom grazes: 6um of margin
        # is far above the shipped 32-gon's 2.4um sagitta at r=0.5.
        r = self._drc_at(0.096)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('NO DRC VIOLATIONS', r.stdout)

    def test_retained_adaptive_helper_still_computes_its_budget(self):
        """`_adaptive_circle_n` is dead code since 6166a98b -- retained on
        purpose, called by nothing (asserted in TestShippedTessellation). Its
        arithmetic is kept under test so restoring it stays a one-line change
        to a working helper rather than a rewrite."""
        from kicad_parser import _adaptive_circle_n
        # 1um sagitta wherever the 64-vertex cap does not bite (r <= ~0.9)...
        for r in (0.2, 0.5, 0.8):
            n = _adaptive_circle_n(r)
            sagitta = r * (1 - math.cos(math.pi / n))
            self.assertLessEqual(sagitta, 0.001 + 1e-9, (r, n, sagitta))
        # ...and bounded growth past it: at the cap the sagitta is r*0.0012,
        # so a r=1mm jumper-pad circle stays within ~1.3um.
        self.assertLessEqual(_adaptive_circle_n(10.0), 64)
        s1 = 1.0 * (1 - math.cos(math.pi / _adaptive_circle_n(1.0)))
        self.assertLessEqual(s1, 0.0013)


class TestShippedTessellation(unittest.TestCase):
    """The fixed 32-gon 6166a98b went back to, pinned where it is decided."""

    PAD_TEXT = ('(pad "1" smd custom (at 0 0) (size 0.1 0.1) (layers "F.Cu")'
                ' (net 1 "A")\n'
                '  (options (clearance outline) (anchor circle))\n'
                '  (primitives (gr_circle (center 0 0) (end 0.45 0)'
                ' (width 0.1) (fill yes))))')

    def _outer(self):
        from kicad_parser import _custom_pad_global_polygons
        polys = _custom_pad_global_polygons(self.PAD_TEXT, 50.0, 50.0, 0.0)
        return max(polys, key=len)

    def test_circle_primitive_is_a_fixed_32gon(self):
        self.assertEqual(len(self._outer()), 32,
                         'the parser must use the fixed inscribed 32-gon '
                         'restored by 6166a98b, not the adaptive count')

    def test_the_polygon_is_inscribed(self):
        """Inscribed = never outside the true copper, so the model can only
        UNDER-report a graze, never manufacture one."""
        r_max = max(math.hypot(x - 50.0, y - 50.0) for x, y in self._outer())
        self.assertLessEqual(r_max, 0.5 + 1e-9, 'polygon escapes the circle')
        self.assertAlmostEqual(r_max, 0.5, places=6,
                               msg='vertices must sit ON the circle')

    def test_the_adaptive_helper_is_wired_to_nothing(self):
        """The revert is only real if nothing calls it."""
        with open(os.path.join(ROOT, 'py_router', 'kicad_parser.py'),
                  encoding='utf-8') as f:
            src = f.read()
        calls = (src.count('_adaptive_circle_n(')
                 - src.count('def _adaptive_circle_n('))
        self.assertEqual(calls, 0,
                         'kicad_parser calls _adaptive_circle_n again -- that '
                         'reverses 6166a98b and shifts DRC grading on every '
                         'circle-primitive board; update this file first')


class TestRun3ArchivedBoard(unittest.TestCase):
    """wk/run3/final2.kicad_pcb is the archived pre-fix board carrying the
    /ICE_CDONE-vs-JP2.2 0.0884mm graze only kicad-cli saw."""

    def test_the_graze_is_masked_at_the_floor_and_visible_above_it(self):
        """The defect is still in the copper; the 32-gon cannot see it at 0.09.

        Measured: the true gap is 0.0884 (0.0016 short of the floor) and the
        32-gon's sagitta at r=0.5 is 0.002408 -- LARGER than the shortfall, so
        the modelled gap lands at 0.0908 and grades clean. Raise the floor past
        the sagitta and the same pair reappears.

        This arm asserted exit 1 at 0.09, which was true while the adaptive
        tessellation shipped. It is re-pinned rather than skipped, so it still
        fails if either the tessellation or check_drc moves."""
        if not os.path.exists(RUN3_BOARD):
            self.skipTest('run-3 archive board not present')
        masked = run_drc(RUN3_BOARD, '-c', '0.09', '--clearance-margin', '0',
                         '--max-print', '0')
        self.assertEqual(masked.returncode, 0, masked.stdout[-800:])
        self.assertIn('NO DRC VIOLATIONS', masked.stdout)

        sagitta = 0.5 * (1 - math.cos(math.pi / 32))
        self.assertGreater(sagitta, 0.09 - 0.0884,
                           'the masking claim only holds while the sagitta '
                           'exceeds the shortfall')

        seen = run_drc(RUN3_BOARD, '-c', '0.095', '--clearance-margin', '0',
                       '--max-print', '0')
        self.assertEqual(seen.returncode, 1, seen.stdout[-800:])
        self.assertRegex(seen.stdout, r'ICE_CDONE.*SWDIO|SWDIO.*ICE_CDONE')


if __name__ == '__main__':
    unittest.main()
