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
        # THREE POPULATIONS, and they are not nested -- the key names say
        # which is which (run-23 review: 'overlap' implied a superset).
        adv = {(a, b) for a, b, _m, _d in cl['b_courtyard_advisory_pairs']}
        self.assertNotIn('b_courtyard_overlap_pairs', cl,
                         'the ambiguous name must not come back')
        # J4<->R21 blocks via the RELATIVE floor (25.5% of R21's courtyard);
        # the true slivers stay out of the blocking list.
        self.assertIn(('J4', 'R21'), adv)
        # `blocking` is NOT inside `advisory`: a waiver-VOIDED pair blocks
        # while staying labelled waived, so the advisory list is smaller and
        # misses some blockers. Pinned, because a reader who assumes nesting
        # audits the wrong list.
        self.assertTrue(blocked - adv,
                        'blocking pairs absent from advisory: '
                        f'{sorted(blocked - adv)}')
        # And the mm2 figure sums EVERY courtyard pair, waived included --
        # not the sum of either list.
        self.assertIsNone(cl['b_courtyard_census_error'])
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
                      doc['checklist']['b_courtyard_advisory_pairs']}
            self.assertNotIn(('B4', 'BAT1'), census)


def _render(*argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'render_placement.py'), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestReviewSheetShowsTheAfterBoard(unittest.TestCase):
    """The sheet must compose the board the facts strip describes.

    Found by review: the sheet picked "the first two panels with no view",
    and under --pair the panel list is [BEFORE F, BEFORE B, AFTER F, AFTER B]
    with view=None on all four -- so the strip's AFTER census sat under the
    BEFORE picture. Measured then: the sheet's top-left 600x600 was
    pixel-identical to the BEFORE render and differed from the AFTER render
    in 170480 px. A reviewer reading that sheet dispositions defects against
    the wrong copper.
    """

    @classmethod
    def setUpClass(cls):
        from PIL import Image                                    # noqa: F401
        cls.td = tempfile.TemporaryDirectory()
        d = cls.td.name
        cls.sheet = os.path.join(d, 'sheet.png')
        r = _render(PLACED, '--before', DAMAGED, '--pair',
                    '--clearance', '0.15', '--quiet',
                    '--json-out', os.path.join(d, 'r.json'),
                    '-o', os.path.join(d, 'r.png'),
                    '--review-sheet', cls.sheet)
        assert r.returncode == 0, r.stdout[-600:] + r.stderr[-600:]
        cls.doc = json.load(open(os.path.join(d, 'r.json'), encoding='utf-8'))

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def _panel(self, label_prefix, side):
        hits = [p['path'] for p in self.doc['panels']
                if (p.get('label') or '').startswith(label_prefix)
                and p.get('side') == side]
        self.assertEqual(len(hits), 1, str(self.doc['panels']))
        return hits[0]

    def test_pair_really_wrote_four_full_board_panels(self):
        """The precondition that made the bug possible, pinned: all four
        panels carry view=None, so `view is None` cannot select the AFTER
        pair and a future 'simplification' back to it is caught here."""
        full = [p for p in self.doc['panels'] if p.get('view') is None]
        self.assertEqual(len(full), 4, str(self.doc['panels']))
        self.assertTrue((full[0].get('label') or '').startswith('BEFORE'))

    def test_sheet_region_is_the_after_panel_not_the_before(self):
        from PIL import Image, ImageChops
        sheet = Image.open(self.sheet).convert('RGB')
        before = Image.open(self._panel('BEFORE', 'F')).convert('RGB')
        after = Image.open(self._panel('AFTER', 'F')).convert('RGB')
        box = (0, 0, 600, 600)
        reg = sheet.crop(box)

        def ndiff(a, b):
            # Non-black pixels of the difference: histogram bucket 0 is
            # "identical", everything above it is a changed pixel.
            return sum(ImageChops.difference(a, b).convert('L')
                       .histogram()[1:])

        d_after = ndiff(reg, after.crop(box))
        d_before = ndiff(reg, before.crop(box))
        self.assertEqual(d_after, 0,
                         f'sheet region differs from the AFTER panel in '
                         f'{d_after} px')
        self.assertGreater(d_before, 1000,
                           'the AFTER and BEFORE panels are indistinguishable '
                           'here, so this test proves nothing -- pick a '
                           'region that changed')


class TestMovedCourtyardGateArmsOnBefore(unittest.TestCase):
    """--gate's moved-relative courtyard key must arm on --before ALONE.

    It guarded on the --pair model instead, so `--before --gate` (the way
    every driver in this repo calls it) never produced the key: the whole
    block was deletable with the suite green. Measured: adding --pair to the
    same command yielded b_courtyard_blocking_pairs(moved)=9.
    """

    def _gate(self, *extra):
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'r.json')
            r = _render(PLACED, '--clearance', '0.15', '--quiet', '--gate',
                        '--json-out', jp, '-o', os.path.join(td, 'r.png'),
                        *extra)
            return r, json.load(open(jp, encoding='utf-8'))

    def test_before_alone_arms_the_moved_key(self):
        r, _doc = self._gate('--before', DAMAGED)
        self.assertEqual(r.returncode, 4, r.stderr[-400:])
        self.assertIn('b_courtyard_blocking_pairs(moved)=9', r.stderr)

    def test_pair_and_before_agree(self):
        """--pair changes the PANELS, not the gate: same key, same count."""
        r, _doc = self._gate('--before', DAMAGED, '--pair')
        self.assertIn('b_courtyard_blocking_pairs(moved)=9', r.stderr)

    def test_without_before_the_key_is_absent(self):
        """No baseline, no moved-relative verdict -- report-only, as the
        corpus census demands (5 of 34 healthy boards would flip)."""
        r, doc = self._gate()
        self.assertNotIn('b_courtyard_blocking_pairs(moved)', r.stderr)
        # ...but the census is still in the checklist, unarmed.
        self.assertTrue(doc['checklist']['b_courtyard_blocking_pairs'])

    def test_gate_line_discloses_the_cross_side_stacks(self):
        """b_cross_side_stacks had no reader anywhere. The verdict line is
        the one place a reader is guaranteed to look, and these are exactly
        the pairs that read as collisions in the panels."""
        r, doc = self._gate('--before', DAMAGED)
        self.assertTrue(doc['checklist']['b_cross_side_stacks'])
        self.assertIn('front<->back stack(s)', r.stderr)


def _floorplan(*argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestWithheldBudgetIsVisibleAtGradeTime(unittest.TestCase):
    """A withheld budget must ABSTAIN, out loud, not vanish.

    emit_intent withholds `overlap_area` when the emitting board carries
    unwaived courtyard interpenetrations past the blocking floors -- baking
    it would bless the board it was measured on. But the grade then had a
    bare `continue` on the missing key and the run printed "0 error(s)" with
    the overlap channel never measured. The withholding note lived only in
    the emitted file, where the grade never looked.
    """

    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.TemporaryDirectory()
        cls.intent = os.path.join(cls.td.name, 'i.json')
        r = _floorplan(PLACED, '--emit-intent', cls.intent,
                       '--declare-classes')
        assert r.returncode == 0, r.stdout[-400:]
        cls.doc = json.load(open(cls.intent, encoding='utf-8'))

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_the_emitter_withheld_overlap_area(self):
        # The key is on `context`, which is where emit_intent writes it.
        self.assertIn('overlap_area', self.doc['context']['budget_withheld'])
        self.assertNotIn('overlap_area', self.doc['legality_budget'])

    def test_grade_reports_it_as_not_derivable(self):
        g = os.path.join(self.td.name, 'g.json')
        r = _floorplan(PLACED, '--intent', self.intent, '--json', g)
        self.assertIn('NOT DERIVABLE', r.stdout)
        self.assertIn('overlap_area', r.stdout)
        # --json carries the full report: {key: reason}.
        doc = json.load(open(g, encoding='utf-8'))
        self.assertEqual(sorted(doc['budget_abstained']), ['overlap_area'])
        self.assertIn('courtyard', doc['budget_abstained']['overlap_area'])
        # ...and the flat JSON_SUMMARY carries the machine-readable count.
        s = json.loads(r.stdout.split('JSON_SUMMARY: ', 1)[1].splitlines()[0])
        self.assertEqual(s['budget_abstained'], 1)
        self.assertEqual(s['budget_abstained_keys'], ['overlap_area'])
        # An abstention is NOT a violation and must not inflate one.
        self.assertEqual(s['errors'], 0)

    def test_a_hand_declared_budget_overrides_the_withholding(self):
        """Declaring the number by hand is the documented escape hatch, and
        a declared key is graded, not abstained."""
        hand = json.loads(json.dumps(self.doc))
        hand['legality_budget']['overlap_area'] = 999.0
        p = os.path.join(self.td.name, 'hand.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(hand, fh)
        g = os.path.join(self.td.name, 'g2.json')
        r = _floorplan(PLACED, '--intent', p, '--json', g)
        self.assertNotIn('NOT DERIVABLE', r.stdout)
        doc = json.load(open(g, encoding='utf-8'))
        self.assertEqual(doc['budget_abstained'], {})

    def test_an_empty_budget_names_the_withholding_in_the_skip_reason(self):
        """When NOTHING was derivable the legality rule does not run at all,
        and "the intent declares no legality_budget" reads as "nobody wanted
        one". It must say the emitter refused."""
        none = json.loads(json.dumps(self.doc))
        none['legality_budget'] = {}
        p = os.path.join(self.td.name, 'none.json')
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(none, fh)
        r = _floorplan(PLACED, '--intent', p)
        self.assertIn('WITHHELD', r.stdout)
        self.assertIn('- legality:', r.stdout)


class TestCensusFailureIsNotCleanliness(unittest.TestCase):
    """A census that could not be built must say so, not read as clean.

    The whole courtyard block sat under a bare `except Exception: pass`, so
    an exception anywhere in it left every key at its empty default: the
    checklist read clean, the review sheet headlined "blocking: none", and
    --gate said "checklist a/b/c clear" about a board nothing had measured.
    """

    def setUp(self):
        for p in (ROOT, os.path.join(ROOT, 'py_router'),
                  os.path.join(ROOT, 'py_placer'),
                  os.path.join(ROOT, 'py_tools')):
            if p not in sys.path:
                sys.path.insert(0, p)

    def _findings(self, broken):
        import render_placement as RP
        from kicad_parser import parse_kicad_pcb
        from placement import legality as LEG
        pcb = parse_kicad_pcb(PLACED)
        model = RP.PlacementModel(pcb, PLACED, quench_kwargs={
            'clearance': 0.15, 'board_edge_clearance': 0.55})
        orig = LEG.grade_body_overlap
        if broken:
            def boom(*_a, **_k):
                raise RuntimeError('census unavailable')
            LEG.grade_body_overlap = boom
        try:
            return RP.legality_findings(model)
        finally:
            LEG.grade_body_overlap = orig

    def test_a_broken_census_is_recorded_not_swallowed(self):
        fnd = self._findings(broken=True)
        self.assertIn('census unavailable',
                      fnd['courtyard_census_error'] or '')
        # ...and the keys really are at their empty defaults, which is
        # exactly why silence was unsafe.
        self.assertEqual(fnd['courtyard_blocking_pairs_refs'], [])
        self.assertEqual(fnd['courtyard_overlap_mm2'], 0.0)

    def test_the_sheet_says_not_measured_instead_of_blocking_none(self):
        import render_placement as RP
        from PIL import Image
        fnd = self._findings(broken=True)
        lines = []
        with tempfile.TemporaryDirectory() as td:
            panel = os.path.join(td, 'p.png')
            Image.new('RGB', (400, 400), (0, 0, 0)).save(panel)
            sheet = os.path.join(td, 's.png')

            class _Spy:
                def __init__(self, inner):
                    self._inner = inner

                def __getattr__(self, name):
                    return getattr(self._inner, name)

                def text(self, xy, s, **kw):
                    lines.append(s)
                    return self._inner.text(xy, s, **kw)

            _orig = RP.ImageDraw.Draw

            def _draw(img, *a, **k):
                return _Spy(_orig(img, *a, **k))

            RP.ImageDraw.Draw = _draw
            try:
                RP.write_review_sheet(sheet, [panel], fnd, [])
            finally:
                RP.ImageDraw.Draw = _orig
            self.assertTrue(os.path.isfile(sheet))
        joined = ' '.join(lines)
        self.assertIn('NOT MEASURED', joined)
        self.assertNotIn('blocking: none', joined)

    def test_a_working_census_reports_no_error(self):
        fnd = self._findings(broken=False)
        self.assertIsNone(fnd['courtyard_census_error'])
        self.assertTrue(fnd['courtyard_blocking_pairs_refs'])


if __name__ == '__main__':
    unittest.main(verbosity=1)
