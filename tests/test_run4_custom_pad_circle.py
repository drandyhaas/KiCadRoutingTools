"""Custom-pad circle primitives: what the SHIPPED tessellation does and does
not see (run-4 B6, reverted in 6166a98b, re-pinned by #718 item 3).

Run 3's DRC verifier lens found a 0.0884mm-vs-0.09 track-to-pad graze that
kicad-cli flagged and check_drc missed: the pad's copper is a gr_circle
PRIMITIVE, and the fixed inscribed 32-gon under-reaches the true circle by
2.408um at r=0.5 -- MORE than the 1.6um shortfall, so the model shows 0.0908
where the copper is at 0.0884. Run-4 B6 replaced it with adaptive tessellation
(1um sagitta, capped 64 vertices).

`6166a98b` then REVERTED that, deliberately: the tessellation lives in the
PARSER, so it moved obstacle geometry and DRC grading together on every board
with circle-primitive custom pads, making cross-era comparisons on those boards
unresolvable by re-grading (both sides shift). The 32-gon is what ships.

This file therefore pins the SHIPPED geometry, not the reverted one:

* the polygon is a fixed, inscribed 32-gon, with its sagitta stated;
* `_adaptive_circle_n` is dead code -- restoring it must be a DECISION;
* the blind spot itself is reproduced synthetically, so the file fails loudly
  the moment the tessellation changes in either direction.

That last one replaces the old `wk/run3/final2.kicad_pcb` arm as the thing
actually holding the line. #718 item 5: that board is gitignored, so on every
machine but the reporter's the arm skipped and the file still exited 0 -- which
is why a test pinning reverted behaviour sat green for a whole release. The
synthetic construction below needs no artifact and runs everywhere.

Both checker directions stay pinned too: a real sub-floor gap is flagged, and a
gap safely above the floor is NOT.
"""

import ast
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


class TestPrimitiveCircleGrading(unittest.TestCase):
    """Both checker directions on a gr_circle-primitive custom pad.

    Renamed from TestAdaptivePrimitiveCircle (#718 item 3): 6166a98b reverted
    the adaptive tessellation, so the old name described geometry the parser
    stopped using -- the same drift that left this file pinning it.
    """

    def _drc_at(self, gap):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'b.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as f:
                f.write(board_with_gap(gap))
            return run_drc(p, '-c', '0.09', '--clearance-margin', '0',
                           '--max-print', '0')

    def test_sub_floor_gap_is_flagged(self):
        # The run-3 geometry: true gap 0.0884 vs floor 0.09 (1.6um short).
        #
        # #718 item 3: this arm and its sibling below are INSENSITIVE TO
        # TESSELLATION by construction, and that is not a defect -- they pin
        # the CHECKER in both directions. `board_with_gap` puts the rect pad
        # due east of the circle centre, and the polygon's vertex k=0 sits at
        # global angle 0, i.e. exactly the true rightmost point (measured
        # max_x = 50.500000 at N=32), so the measured gap equals the true gap
        # for ANY N. Tessellation is pinned by TestShippedTessellation.
        r = self._drc_at(0.0884)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn('PAD-PAD', r.stdout)

    def test_above_floor_gap_stays_clean(self):
        # Inscribed polygons cannot manufacture phantom grazes: 6um of margin
        # is far above even the 32-gon's 2.4um sagitta. Also tessellation-
        # insensitive -- see the note on the arm above.
        r = self._drc_at(0.096)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('NO DRC VIOLATIONS', r.stdout)


class TestShippedTessellation(unittest.TestCase):
    """The geometry that actually ships, asserted directly (#718 item 3).

    This replaces `test_polygon_radial_error_budget`, which called
    `_adaptive_circle_n` -- a helper `6166a98b` left with ONE definition and
    ZERO call sites, so the arm measured a function no board geometry goes
    through. It passed while the parser did something else entirely.
    """

    R = 0.45 + 0.1 / 2.0          # primitive radius + half its stroke width
    N = 32

    def _circle_polygon(self):
        from kicad_parser import parse_kicad_pcb
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'b.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as f:
                f.write(board_with_gap(0.2))
            pcb = parse_kicad_pcb(p)
        fp = pcb.footprints['JP1']
        polys = [q for q in (fp.pads[0].polygons or []) if len(q) == self.N]
        self.assertEqual(len(polys), 1, 'expected one 32-gon for the primitive')
        return polys[0]

    def test_the_polygon_is_a_fixed_inscribed_32gon(self):
        poly = self._circle_polygon()
        self.assertEqual(len(poly), self.N, 'the shipped count is FIXED at 32')
        cx, cy = 50.0, 50.0
        radii = [math.hypot(x - cx, y - cy) for x, y in poly]
        for r in radii:
            self.assertAlmostEqual(r, self.R, places=9,
                                   msg='vertices must lie ON the circle '
                                       '(inscribed), never outside it')

    def test_the_sagitta_is_stated_not_discovered(self):
        """2.408um at r=0.5 -- larger than the 1.6um run-3 shortfall.

        This is the accuracy 6166a98b knowingly accepted. Asserted so the
        number is on the record rather than rediscovered by the next reader.
        """
        sagitta = self.R * (1 - math.cos(math.pi / self.N))
        self.assertAlmostEqual(sagitta, 0.002408, places=6)
        self.assertGreater(sagitta, 0.09 - 0.0884,
                           'the model error still exceeds the run-3 shortfall '
                           '-- that graze is invisible BY DESIGN')

    def test_adaptive_tessellation_has_no_call_sites(self):
        """`_adaptive_circle_n` is dead -- restoring it must be a DECISION.

        6166a98b reverted the call, not the helper. A live-looking helper that
        nothing calls is how a test came to pin behaviour the parser had
        stopped exhibiting; this asserts the zero so re-wiring it is deliberate
        and shows up here.
        """
        src = os.path.join(ROOT, 'py_router', 'kicad_parser.py')
        with open(src, encoding='utf-8') as fh:
            tree = ast.parse(fh.read())
        defs = [n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef)
                and n.name == '_adaptive_circle_n']
        calls = [n for n in ast.walk(tree)
                 if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name)
                 and n.func.id == '_adaptive_circle_n']
        self.assertEqual(len(defs), 1, 'the helper should still be defined')
        self.assertEqual(
            len(calls), 0,
            'kicad_parser now CALLS _adaptive_circle_n. That reverses '
            '6166a98b, which moves obstacle geometry and DRC grading together '
            'on every board with circle-primitive custom pads -- land it as a '
            'measured baseline change and update this file, do not let it '
            'drift back in.')


class TestRetainedAdaptiveHelper(unittest.TestCase):
    """`_adaptive_circle_n` is dead code, and kept under test anyway.

    6166a98b reverted the CALL, not the helper, explicitly so the accuracy can
    be restored later "as a deliberate, measured baseline change". Keeping its
    arithmetic pinned makes that a one-line rewire of a working helper rather
    than a rewrite from the commit message. The zero call sites are asserted
    separately in TestShippedTessellation -- the two together say "correct, and
    deliberately unused". (Kept on edgehero's argument in PR #719; the first
    draft of this file deleted this coverage along with the stale arm.)
    """

    def test_it_still_computes_its_stated_budget(self):
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


class TestTessellationBlindSpot(unittest.TestCase):
    """The run-3 defect, reproduced WITHOUT the gitignored archive board.

    Two circle-primitive pads at the same true gap (0.0884, the run-3 number),
    one pair placed along a POLYGON VERTEX ray and one along the ray exactly
    between two vertices. The 32-gon is exact on the first and under-reaches by
    a full sagitta on each pad on the second, so check_drc flags one and misses
    the other -- the whole defect, on a board this file builds itself.

    It is also the change detector: restore adaptive tessellation and the
    second arm starts flagging, so the revert cannot silently come undone.
    """

    R = 0.45 + 0.1 / 2.0
    N = 32
    GAP = 0.0884                       # the run-3 true copper gap

    def _two_circles(self, theta_deg):
        d = 2 * self.R + self.GAP
        th = math.radians(theta_deg)
        cx, cy = 50 + d * math.cos(th), 50 + d * math.sin(th)
        prim = ('(options (clearance outline) (anchor circle))\n'
                '      (primitives (gr_circle (center 0 0) (end 0.45 0)'
                ' (width 0.1) (fill yes)))')
        return (
            '(kicad_pcb (version 20221018) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "A") (net 2 "B")\n'
            '  (gr_rect (start 40 40) (end 70 60) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n'
            '  (footprint "t:JP" (layer "F.Cu") (at 50 50)\n'
            '    (property "Reference" "JP1" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd custom (at 0 0) (size 0.1 0.1) (layers "F.Cu")'
            ' (net 1 "A")\n      ' + prim + '))\n'
            f'  (footprint "t:JQ" (layer "F.Cu") (at {cx:.6f} {cy:.6f})\n'
            '    (property "Reference" "JP2" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd custom (at 0 0) (size 0.1 0.1) (layers "F.Cu")'
            ' (net 2 "B")\n      ' + prim + '))\n)\n')

    def _drc_along(self, theta_deg):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, 'b.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as f:
                f.write(self._two_circles(theta_deg))
            return run_drc(p, '-c', '0.09', '--clearance-margin', '0',
                           '--max-print', '0')

    def test_a_vertex_ray_sees_the_graze(self):
        """Along theta=0 a vertex sits exactly on the true circle: gap exact."""
        r = self._drc_along(0.0)
        self.assertEqual(r.returncode, 1,
                         'the 0.0884 graze must be visible where the polygon '
                         'is exact:\n' + (r.stdout + r.stderr)[-600:])

    def test_the_between_vertex_ray_misses_it(self):
        """Along theta=180/N BOTH polygons under-reach by a full sagitta.

        Modelled gap = 0.0884 + 2 * 0.002408 = 0.0932 > 0.09, so check_drc
        reports clean on copper that is 1.6um short. This is not a checker bug
        -- it is the accuracy 6166a98b accepted, stated as a fact.

        If this arm ever FAILS, the tessellation got finer (adaptive
        tessellation restored, or N raised). That is a legitimate change; make
        it deliberately and re-pin this file.
        """
        r = self._drc_along(180.0 / self.N)
        self.assertEqual(r.returncode, 0,
                         'the shipped 32-gon cannot see this graze; a flag '
                         'here means the tessellation changed:\n'
                         + (r.stdout + r.stderr)[-600:])


class TestRun3ArchivedBoard(unittest.TestCase):
    """The original artifact, re-pinned on the SHIPPED geometry (#718 item 3).

    This arm asserted `check_drc -c 0.09` exits 1 on the archived board. Since
    6166a98b it exits 0, because the 32-gon models the 0.0884 gap as 0.0908 --
    the test pinned reverted behaviour and nothing reconciled it. It stayed
    invisible because `wk/` is gitignored: absent the board it skips and the
    file exits 0 (#718 item 5, now disclosed by test_457_fresh_clone_fixtures).

    So pin BOTH ends of the mask instead of one: clean at 0.09 (the model
    over-reports by more than the shortfall) and flagged at 0.095 (raise the
    floor past the modelled 0.0908 and the same pair reappears). One arm alone
    cannot tell "correctly masked" from "checker broken".
    """

    def _drc_at(self, clearance):
        return run_drc(RUN3_BOARD, '-c', clearance, '--clearance-margin', '0',
                       '--max-print', '0')

    def test_the_graze_is_masked_at_the_routed_floor(self):
        if not os.path.exists(RUN3_BOARD):
            self.skipTest('run-3 archive board not present (wk/ is gitignored)')
        r = self._drc_at('0.09')
        self.assertEqual(r.returncode, 0,
                         'the shipped 32-gon models this 0.0884 gap as 0.0908, '
                         'so 0.09 must grade clean:\n' + r.stdout[-800:])

    def test_and_reappears_once_the_floor_clears_the_model(self):
        if not os.path.exists(RUN3_BOARD):
            self.skipTest('run-3 archive board not present (wk/ is gitignored)')
        r = self._drc_at('0.095')
        self.assertEqual(r.returncode, 1, r.stdout[-800:])
        self.assertRegex(r.stdout, r'ICE_CDONE.*SWDIO|SWDIO.*ICE_CDONE')


if __name__ == '__main__':
    unittest.main()
