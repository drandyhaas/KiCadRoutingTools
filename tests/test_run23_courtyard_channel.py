"""The run-23 courtyard channel: census absolute, gate moved-vs-baseline.

Run 23 shipped a board every gate called buildable while J4 stood 0.90mm
inside U6's courtyard (5.56mm2), J3 interpenetrated R13 and RN3 still sat
0.83mm2 into U5 -- residue of the staged damage the repair moved but never
cleared. The courtyard channel existed and was advisory everywhere.

Why the gate is moved-vs-baseline and NOT absolute, measured on this repo's
own corpus before this landed: 5 of 34 healthy human boards (glasgow_revC,
orangecrab_ext_pll, rp2350_fpga_eensy_prePlane, ulx3s, watchy) ship unwaived
courtyard interpenetrations past any sane area/depth floor -- ulx3s GPDI1<->
U11 at 38.5mm2 / depth 5.1, rp2350 U3 frac-1.0 inside J2 -- all by design
(parts under connector shells). An absolute conjunct flips them all NOT
BUILDABLE. A pair therefore gates only when a MEMBER MOVED relative to
--baseline: a pristine board graded against itself can never flip, while a
repair run owns every pair its moves created or failed to clear.

And why moved-MEMBER rather than new-PAIR: RN3<->U5 exists in the damaged
baseline too (the staged containment). A membership test calls it
pre-existing and misses it; the repair moved RN3 3.28mm, so the moved test
charges it. That distinction is pinned here because it is exactly the kind
of clause a cleanup would simplify away.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')
DAMAGED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                       'tigard_damaged.kicad_pcb')
ULX3S = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')


def _run(*argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_assembly.py'), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


def _grade(board, *extra):
    with tempfile.TemporaryDirectory() as td:
        jp = os.path.join(td, 'a.json')
        r = _run(board, '--json', jp, *extra)
        return r, json.load(open(jp, encoding='utf-8'))


class TestRun23Board(unittest.TestCase):
    """The board run 23 actually shipped, graded the way the run should."""

    def test_gates_against_the_damaged_baseline(self):
        r, doc = _grade(PLACED, '--baseline', DAMAGED)
        self.assertEqual(r.returncode, 4, r.stdout[-600:])
        self.assertFalse(doc['buildable'])
        self.assertIn('NOT BUILDABLE', r.stdout)
        self.assertEqual(doc['courtyard_blocking_gating'], 9)
        self.assertEqual(doc['courtyard_gating_basis'], 'moved-vs-baseline')
        gating = {(q['a'], q['b'])
                  for q in doc['courtyard_blocking_gating_pairs']}
        self.assertEqual(gating, {('J4', 'U6'), ('J3', 'R13'),
                                  ('RN3', 'U5'), ('SW1', 'U1'),
                                  ('J1', 'SW1'), ('J1', 'SW2'),
                                  ('J1', 'R5'), ('J4', 'R21'),
                                  ('H2', 'J3')})
        # `blocking` (pad intersections) must NOT have moved -- it has three
        # consumers and this board has none.
        self.assertEqual(doc['blocking'], 0)

    def test_moved_member_charges_a_baseline_resident_pair(self):
        """RN3<->U5 is IN the damaged baseline; a new-pair test misses it."""
        r, doc = _grade(PLACED, '--baseline', DAMAGED)
        gating = {(q['a'], q['b'])
                  for q in doc['courtyard_blocking_gating_pairs']}
        self.assertIn(('RN3', 'U5'), gating)
        # And the stdout names WHO moved (RN3; U5 stood still).
        self.assertIn('GATES (moved: RN3)', r.stdout)

    def test_without_baseline_census_reports_but_never_gates(self):
        r, doc = _grade(PLACED)
        self.assertEqual(r.returncode, 0, r.stdout[-600:])
        self.assertTrue(doc['buildable'])
        self.assertEqual(doc['courtyard_blocking'], 10)
        self.assertIsNone(doc['courtyard_blocking_gating'])
        self.assertEqual(doc['courtyard_gating_basis'],
                         'no-baseline: report-only')
        self.assertIn('REPORT-ONLY', r.stdout)

    def test_synthetic_courtyards_never_block(self):
        """G***'s +/-0.5mm fictional box manufactured a 0.537mm2 'pair'."""
        _r, doc = _grade(PLACED, '--baseline', DAMAGED)
        self.assertIn('G***', doc['courtyard_synthetic_refs'])
        blocked = {(q['a'], q['b']) for q in doc['courtyard_blocking_pairs']}
        self.assertNotIn(('G***', 'J5'), blocked)
        # ...but the pair stays VISIBLE in the census (disclosure, not gate).
        census = {(q['a'], q['b']) for q in doc['courtyard_pairs']}
        self.assertIn(('G***', 'J5'), census)

    def test_floors_hold_the_true_slivers_out(self):
        """The RELATIVE floor (user finding): J4<->R21's 0.445mm2 slid
        under the 0.5mm2 absolute floor while consuming 25.5% of R21's
        courtyard -- it blocks now. The true slivers (D3<->SW1 0.059mm2 at
        depth 0.02, frac 0.014) stay advisory: every floor must hold
        SOMETHING out or it is not a floor."""
        _r, doc = _grade(PLACED, '--baseline', DAMAGED)
        blocked = {(q['a'], q['b']) for q in doc['courtyard_blocking_pairs']}
        self.assertIn(('J4', 'R21'), blocked)
        self.assertNotIn(('D3', 'SW1'), blocked)
        self.assertNotIn(('D4', 'SW2'), blocked)
        census = {(q['a'], q['b']) for q in doc['courtyard_pairs']}
        self.assertIn(('D3', 'SW1'), census)

    def test_dead_edge_waiver_does_not_hide_the_switches(self):
        """The user's own finding, pinned: SW1/SW2 collided with parts and
        NOTHING said so, because edge_class -- a class lookup with no
        geometry -- waived every pair. The waiver now stands only for a
        member whose pose is edge-LIVE (overhanging, or within seat
        tolerance): SW1 sat 2.0mm interior, SW2 8.33mm, so SW1<->U1 and
        FB1<->SW2 join the blocking census. And a LIVE waiver covers only
        the MATING ZONE (second user finding): J1 is legitimately at the
        edge, but R5 sat 45.7% inside J1's INTERIOR courtyard -- under the
        connector body, 1.5mm inside the outline -- so J1<->R5, J1<->SW1
        and J1<->SW2 block too; only an overlap that leaves or hugs the
        outline is mating volume."""
        _r, doc = _grade(PLACED, '--baseline', DAMAGED)
        blocked = {(q['a'], q['b']) for q in doc['courtyard_blocking_pairs']}
        self.assertIn(('SW1', 'U1'), blocked)
        self.assertIn(('FB1', 'SW2'), blocked)
        self.assertIn(('J1', 'R5'), blocked)
        self.assertIn(('J1', 'SW1'), blocked)
        # FB1<->SW2 is the moved-currency's NAMED blind spot: the damage
        # placed both and the repair never touched either, so no movement
        # test can charge it without flipping pristine boards. It must stay
        # visible in the census while NOT gating.
        gating = {(q['a'], q['b'])
                  for q in doc['courtyard_blocking_gating_pairs']}
        self.assertNotIn(('FB1', 'SW2'), gating)

    def test_locked_and_mount_hole_pairs_face_the_floors(self):
        """User finding #3, two rules in one pair: H2<->J3 (4.63mm2, depth
        1.47) hid behind the blanket marker_class waiver. A mounting hole's
        courtyard is the SCREW-HEAD keepout -- physical, unlike a fiducial's
        -- and H2 is KiCad-LOCKED, and no class waiver blesses contact with
        a locked part (the run-8 E6 principle, extended here). Both rules
        route the pair to the ordinary floors; J3 moved 3.07mm, so it
        gates. Fiducial/testpoint markers keep the blanket exemption
        (G***<->TP1 stays out of blocking)."""
        _r, doc = _grade(PLACED, '--baseline', DAMAGED)
        blocked = {(q['a'], q['b']) for q in doc['courtyard_blocking_pairs']}
        self.assertIn(('H2', 'J3'), blocked)
        self.assertNotIn(('G***', 'TP1'), blocked)
        gating = {(q['a'], q['b'])
                  for q in doc['courtyard_blocking_gating_pairs']}
        self.assertIn(('H2', 'J3'), gating)

    def test_opposite_side_xy_overlap_is_not_a_pair(self):
        """User question, answered by measurement and pinned: JP2 (a
        B-side SMD jumper) sits exactly over F-side R16/C25 in XY -- pad
        gap 0.000mm -- and that is NOT a conflict: opposite faces. The
        side-aware census must never pair them."""
        _r, doc = _grade(PLACED)
        census = {(q['a'], q['b']) for q in doc['courtyard_pairs']}
        self.assertNotIn(('JP2', 'R16'), census)
        self.assertNotIn(('C25', 'JP2'), census)

    def test_full_census_is_carried(self):
        """The 15-pair census the user SAW must be in the JSON, waivers and
        all -- J1<->SW1 (7.0mm2, edge-waived) is the one a reader asks about
        first."""
        _r, doc = _grade(PLACED)
        census = {(q['a'], q['b']) for q in doc['courtyard_pairs']}
        self.assertIn(('J1', 'SW1'), census)
        self.assertGreaterEqual(len(census), 15)


class TestRenderCourtyardTruth(unittest.TestCase):
    """render_placement must SHOW the courtyard channel, not only key it.

    Run 23's board was viewed at L3 with 'overlap 26.30mm2' in the banner and
    still read clean: courtyards drew as thin gray outlines (overlap looks
    like tight packing) and the checklist had no key for the courtyard
    census -- b_body_overlap_pairs is PAD intersections.
    """

    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.TemporaryDirectory()
        jp = os.path.join(cls.td.name, 'r.json')
        png = os.path.join(cls.td.name, 'r.png')
        sheet = os.path.join(cls.td.name, 'sheet.png')
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
                   KRT_NO_BANNER='1')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_tools', 'render_placement.py'),
             PLACED, '--clearance', '0.15', '--json-out', jp, '-o', png,
             '--review-sheet', sheet],
            capture_output=True, text=True, env=env, cwd=ROOT)
        assert r.returncode == 0, r.stdout[-600:] + r.stderr[-600:]
        cls.doc = json.load(open(jp, encoding='utf-8'))
        cls.png_f = png.replace('.png', '_F.png')
        cls.sheet = sheet

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_checklist_carries_the_courtyard_keys(self):
        cl = self.doc['checklist']
        blocked = {(a, b) for a, b, _m, _d in
                   cl['b_courtyard_blocking_pairs']}
        self.assertEqual(blocked, {('J4', 'U6'), ('J3', 'R13'),
                                   ('RN3', 'U5'), ('SW1', 'U1'),
                                   ('FB1', 'SW2'), ('J1', 'SW1'),
                                   ('J1', 'SW2'), ('J1', 'R5'),
                                   ('J4', 'R21'), ('H2', 'J3')})
        census = {(a, b) for a, b, _m, _d in cl['b_courtyard_overlap_pairs']}
        # J4<->R21 blocks via the RELATIVE floor (25.5% of R21's courtyard);
        # the true slivers stay out of the blocking list.
        self.assertIn(('J4', 'R21'), census)
        self.assertNotIn(('D3', 'SW1'), blocked)
        self.assertGreater(cl['b_courtyard_overlap_mm2'], 20.0)

    def test_the_defect_is_in_the_pixels(self):
        """The C_COURT_OVL fill must be present INSIDE the J4<->U6
        intersection region -- the picture, not only the key."""
        from PIL import Image
        img = Image.open(self.png_f).convert('RGB')
        w, h = img.size
        hits = sum(1 for _x in range(0, w, 7) for _y in range(0, h, 7)
                   if img.getpixel((_x, _y)) == (255, 120, 40))
        self.assertGreater(
            hits, 20, 'no courtyard-interpenetration fill in the render')

    def test_review_sheet_exists_and_is_wide(self):
        from PIL import Image
        self.assertTrue(os.path.exists(self.sheet))
        img = Image.open(self.sheet)
        # F+B side by side over a facts strip: wider than tall, and taller
        # than either bare panel (the strip).
        self.assertGreater(img.width, img.height)
        self.assertEqual(self.doc.get('review_sheet'), self.sheet)


class TestPristineBoards(unittest.TestCase):
    """The corpus lesson: healthy boards carry big by-design censuses."""

    def test_pristine_board_vs_itself_never_flips(self):
        r, doc = _grade(ULX3S, '--baseline', ULX3S)
        self.assertEqual(r.returncode, 0, r.stdout[-600:])
        self.assertTrue(doc['buildable'])
        # The census is large and REAL (GPDI1's shell over its passives) --
        # pin that it exists, so a future "fix" that empties the census to
        # make the gate quiet is caught here.
        self.assertGreaterEqual(doc['courtyard_blocking'], 10)
        self.assertEqual(doc['courtyard_blocking_gating'], 0)

    def test_pristine_board_without_baseline_stays_buildable(self):
        r, doc = _grade(ULX3S)
        self.assertEqual(r.returncode, 0, r.stdout[-600:])
        self.assertTrue(doc['buildable'])

    def test_cross_side_stacks_are_named_not_paired(self):
        """User finding on the ulx3s review: BAT1 sits BEHIND the buttons
        (B4: 68.7mm2 of XY overlap, opposite faces) and the panels make that
        visually indistinguishable from a collision. The census correctly
        refuses to pair opposite faces; the render's checklist now NAMES the
        stacks so the sheet pre-answers the eye instead of looking blind."""
        import subprocess as _sp
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
                   KRT_NO_BANNER='1')
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'r.json')
            r = _sp.run([sys.executable, '-X', 'utf8',
                         os.path.join(ROOT, 'py_tools',
                                      'render_placement.py'),
                         ULX3S, '--json-out', jp,
                         '-o', os.path.join(td, 'r.png')],
                        capture_output=True, text=True, env=env, cwd=ROOT)
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            doc = json.load(open(jp, encoding='utf-8'))
            stacks = {(a, b) for a, b, _m in
                      doc['checklist']['b_cross_side_stacks']}
            self.assertIn(('B4', 'BAT1'), stacks)
            # ...and the same pair is NOT in the courtyard census: opposite
            # faces never pair.
            census = {(a, b) for a, b, *_ in
                      doc['checklist']['b_courtyard_overlap_pairs']}
            self.assertNotIn(('B4', 'BAT1'), census)


if __name__ == '__main__':
    unittest.main(verbosity=1)
