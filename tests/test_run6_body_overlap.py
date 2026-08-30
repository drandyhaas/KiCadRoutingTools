"""Run-6 body-overlap channel (the assembly gate's primitive).

The calibration contract: the BLOCKING channel (cross-footprint pad
intersection) reads ZERO pairs on every healthy in-repo board, and catches
the run-5 shipped defect (C14 stacked on R14, same-net pads intersecting).
Advisory channels (fab/courtyard) are labeled, never blocking.
"""

import glob
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
FINAL5 = os.path.join(ROOT, 'wk', 'run5', 'final5.kicad_pcb')
HUMAN = os.path.join(ROOT, 'wk', 'run2', 'original', 'tigard_v10.kicad_pcb')


def _grade(board, **kw):
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_body_overlap
    return grade_body_overlap(parse_kicad_pcb(board), 0.09,
                              pcb_file=board, **kw)


class TestCorpusCalibration(unittest.TestCase):
    def test_all_healthy_boards_grade_zero_blocking(self):
        """THE calibration gate: pad_intersection must be 0 on every corpus
        board, or the channel may not gate anywhere (run-6 invariant)."""
        boards = sorted(glob.glob(os.path.join(ROOT, 'kicad_files',
                                               '*.kicad_pcb')))
        # The TRACKED corpus is 22 boards; generated fixture chains add more on a
        # developed tree. Guard against an empty or half-checked-out corpus, not
        # against the generated surplus (a fresh clone has exactly 22).
        self.assertGreaterEqual(len(boards), 22)
        bad = []
        for b in boards:
            try:
                g = _grade(b)
            except Exception as e:
                bad.append((os.path.basename(b), f'ERROR {e}'))
                continue
            if g['blocking']:
                bad.append((os.path.basename(b),
                            [(p.a, p.b) for p in g['blocking_pairs']]))
        self.assertEqual(bad, [], f'blocking pairs on healthy boards: {bad}')


class TestKnownDefect(unittest.TestCase):
    def test_run5_deliverable_is_caught(self):
        """The board run 5 shipped: C14 stacked on R14 must be BLOCKING via
        pad_intersection (same-net copper physically intersecting -- the
        channel pair_shortfall's same-net skip is blind to), with courtyard
        and fab advisory entries for the same pair."""
        if not os.path.exists(FINAL5):
            self.skipTest('run-5 deliverable not present')
        g = _grade(FINAL5)
        self.assertEqual(g['blocking'], 1)
        p = g['blocking_pairs'][0]
        self.assertEqual((p.a, p.b, p.kind),
                         ('C14', 'R14', 'pad_intersection'))
        kinds = {q.kind for q in g['pairs'] if (q.a, q.b) == ('C14', 'R14')}
        self.assertEqual(kinds, {'pad_intersection', 'courtyard', 'fab'})

    def test_human_board_calibrates_clean(self):
        """The human tigard: 0 blocking; its two by-design courtyard pairs
        (mount holes under connector shells) waive by marker class."""
        if not os.path.exists(HUMAN):
            self.skipTest('human board not present')
        g = _grade(HUMAN)
        self.assertEqual(g['blocking'], 0)
        waived = {(p.a, p.b): p.waiver for p in g['pairs'] if p.waived}
        self.assertEqual(waived.get(('H4', 'J2')), 'marker_class')
        self.assertEqual(waived.get(('H3', 'J7')), 'marker_class')


class TestSynthetic(unittest.TestCase):
    BOARD = (
        '(kicad_pcb (version 20221018) (generator pcbnew)\n'
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
        ' (44 "Edge.Cuts" user))\n'
        '  (net 0 "") (net 1 "VCC")\n'
        '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1)'
        ' (type default)) (layer "Edge.Cuts"))\n'
        '  (footprint "t:A" (layer "{la}") (at 10 10)\n'
        '    (property "Reference" "CA" (at 0 0) (layer "F.SilkS"))\n'
        '    (fp_rect (start -1 -0.6) (end 1 0.6) (stroke (width 0.05)'
        ' (type default)) (layer "{la_pref}.CrtYd"))\n'
        '    (pad "1" smd rect (at -0.5 0) (size 0.6 0.6)'
        ' (layers "{la}") (net 1 "VCC"))\n'
        '    (pad "2" smd rect (at 0.5 0) (size 0.6 0.6)'
        ' (layers "{la}") (net 1 "VCC")))\n'
        '  (footprint "t:B" (layer "{lb}") (at {bx} 10)\n'
        '    (property "Reference" "CB" (at 0 0) (layer "F.SilkS"))\n'
        '    (fp_rect (start -1 -0.6) (end 1 0.6) (stroke (width 0.05)'
        ' (type default)) (layer "{lb_pref}.CrtYd"))\n'
        '    (pad "1" smd rect (at -0.5 0) (size 0.6 0.6)'
        ' (layers "{lb}") (net 1 "VCC"))\n'
        '    (pad "2" smd rect (at 0.5 0) (size 0.6 0.6)'
        ' (layers "{lb}") (net 1 "VCC")))\n'
        ')\n')

    def _board(self, bx, la='F.Cu', lb='F.Cu'):
        text = self.BOARD.format(bx=bx, la=la, lb=lb,
                                 la_pref=la[0], lb_pref=lb[0])
        td = tempfile.mkdtemp()
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        return path

    def test_same_net_stack_is_blocking(self):
        """Two footprints stacked so their SAME-NET pads intersect: the
        short detector skips the pair by design; the assembly channel must
        flag it (the C14/R14 class)."""
        g = _grade(self._board(bx=10.4))
        self.assertEqual(g['blocking'], 1)
        self.assertEqual(g['blocking_pairs'][0].kind, 'pad_intersection')

    def test_opposite_sides_do_not_interact(self):
        """The same XY stack on OPPOSITE board sides is legal (the JP1/SW1
        lesson: side-blind flattening manufactured a phantom)."""
        g = _grade(self._board(bx=10.4, lb='B.Cu'))
        self.assertEqual(g['blocking'], 0)
        self.assertEqual(g['advisory'], 0)

    def test_clear_parts_grade_clean(self):
        g = _grade(self._board(bx=13.0))
        self.assertEqual(g['blocking'], 0)
        self.assertEqual(g['advisory'], 0)

    def test_courtyard_kiss_is_advisory_not_blocking(self):
        """Pads clear, courtyards intersecting: advisory (the corpus
        measured 6 real boards shipping exactly this; it must not block)."""
        g = _grade(self._board(bx=11.8))
        self.assertEqual(g['blocking'], 0)
        self.assertEqual(g['advisory'], 1)
        self.assertEqual(g['advisory_pairs'][0].kind, 'courtyard')

    def test_intent_waiver_labels_the_pair(self):
        g = _grade(self._board(bx=11.8), intent_waivers=[('CA', 'CB')])
        self.assertEqual(g['advisory'], 0)
        waived = [p for p in g['pairs'] if p.waived]
        self.assertEqual(len(waived), 1)
        self.assertEqual(waived[0].waiver, 'intent_declared')


class TestIntentKey(unittest.TestCase):
    def test_overlap_waivers_load_and_validate(self):
        from placement import floorplan
        doc = {'schema': 1, 'kind': 'floorplan-intent',
               'overlap_waivers': [{'pair': ['A1', 'B2'],
                                    'reason': 'shield overhang'}]}
        td = tempfile.mkdtemp()
        p = os.path.join(td, 'i.json')
        with open(p, 'w', encoding='utf-8') as f:
            json.dump(doc, f)
        intent = floorplan.load_intent(p)
        self.assertEqual(intent.waiver_pairs(), (('A1', 'B2'),))
        doc['overlap_waivers'] = [{'pair': ['only-one']}]
        with open(p, 'w', encoding='utf-8') as f:
            json.dump(doc, f)
        with self.assertRaises(floorplan.IntentError):
            floorplan.load_intent(p)


if __name__ == '__main__':
    unittest.main()


class TestContainment(unittest.TestCase):
    """The containment channel: run-22's defect, and why it may not gate.

    Run 22 shipped a board every gate called buildable while RN3 sat wholly
    inside U5's body and RN7 inside U6's -- reported as `fab 2.0mm2`, which is
    also what a large connector's by-design graze measures. `area_mm2` cannot
    tell a KISS from a part WHOLLY INSIDE another; `contained_frac` can.
    """

    #: A big part with a .Fab body, and a small one whose body sits inside it.
    #: The pads are deliberately clear of each other, so NOTHING in the
    #: pad_intersection channel fires -- that is the whole point. This is the
    #: defect shape that reported blocking 0.
    BOARD = '''(kicad_pcb (version 20221018) (generator pcbnew)
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "VCC") (net 2 "GND")
  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))
  (footprint "t:BIG" (layer "F.Cu") (at 10 10)
    (property "Reference" "{big}" (at 0 0) (layer "F.SilkS"))
    (fp_rect (start -4 -4) (end 4 4) (stroke (width 0.05) (type default)) (layer "F.Fab"))
    (fp_rect (start -4.2 -4.2) (end 4.2 4.2) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
    (pad "1" smd rect (at -3.7 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "VCC"))
    (pad "2" smd rect (at 3.7 0) (size 0.4 0.4) (layers "F.Cu") (net 1 "VCC")))
  (footprint "t:SMALL" (layer "F.Cu") (at {sx} 10)
    (property "Reference" "{small}" (at 0 0) (layer "F.SilkS"))
    (fp_rect (start -0.5 -0.3) (end 0.5 0.3) (stroke (width 0.05) (type default)) (layer "F.Fab"))
    (fp_rect (start -0.7 -0.5) (end 0.7 0.5) (stroke (width 0.05) (type default)) (layer "F.CrtYd"))
    (pad "1" smd rect (at 0 0) (size 0.2 0.2) (layers "F.Cu") (net 2 "GND")))
)
'''

    def _board(self, sx, big='U1', small='RN1'):
        text = self.BOARD.format(sx=sx, big=big, small=small)
        td = tempfile.mkdtemp()
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        return path

    def _fab(self, g):
        return [p for p in g['pairs'] if p.kind == 'fab']

    def test_containment_is_measured(self):
        """The small part's body wholly inside the big one's."""
        g = _grade(self._board(sx=10.0))
        fab = self._fab(g)
        self.assertEqual(len(fab), 1)
        self.assertEqual(fab[0].contained_frac, 1.0)
        self.assertTrue(fab[0].contained)
        self.assertEqual(g['contained'], 1)
        self.assertEqual([(p.a, p.b) for p in g['containment_pairs']],
                         [('RN1', 'U1')])

    def test_a_kiss_is_not_a_containment(self):
        """The run-4 lesson pinned into the new channel: two bodies meeting at
        their edges is not a part inside a part, and must not report as one."""
        g = _grade(self._board(sx=14.4))
        fab = self._fab(g)
        self.assertEqual(len(fab), 1)
        self.assertLess(fab[0].contained_frac, 0.5)
        self.assertFalse(fab[0].contained)
        self.assertEqual(g['contained'], 0)

    def test_a_waived_containment_is_still_disclosed(self):
        """THE run-22 hole. `_waiver_for` is a part-class lookup with no
        geometry in it, so a part sitting WHOLLY inside an edge_actuator gets
        the same label as a 0.01mm2 graze and then leaves `advisory`,
        `advisory_pairs` AND `new_advisory_pairs` in one step. D4-inside-SW2
        vanished exactly that way, twice, and nothing in the chain reported it.
        `containment_pairs` is the one list a waiver cannot empty."""
        g = _grade(self._board(sx=10.0), intent_waivers=[('U1', 'RN1')])
        self.assertEqual(g['advisory'], 0)
        self.assertTrue(all(p.waived for p in self._fab(g)))
        self.assertEqual(g['contained'], 1)
        self.assertTrue(g['containment_pairs'][0].waived)

    def test_containment_does_not_change_the_verdict(self):
        """Zero blast radius, pinned. The corpus ships legitimate frac-1.0
        containments (orangecrab FID2/J5), so this channel may not gate."""
        g = _grade(self._board(sx=10.0))
        self.assertEqual(g['contained'], 1)
        self.assertEqual(g['blocking'], 0)
        self.assertEqual(g['blocking_pairs'], [])

    def test_a_nonexempt_containment_GATES(self):
        """The verdict change: a part wholly inside another part's body makes
        the board NOT BUILDABLE, unless something says it is by design."""
        g = _grade(self._board(sx=10.0))
        self.assertEqual(g['containment_blocking'], 1, g['containment_pairs'])
        # ...and `blocking` is UNTOUCHED. Three consumers read that count --
        # board_score, the seeder's repair census, and placement_driver's
        # _guard_damage with INVERTED polarity ("run repair only if blocking").
        # Folding containment in would change all three.
        self.assertEqual(g['blocking'], 0)

    def test_an_AUTHORED_waiver_lifts_the_gate(self):
        """The escape hatch, and the only one: name the pair in the intent.
        That is written down by a person; a class waiver is inherited."""
        g = _grade(self._board(sx=10.0), intent_waivers=[('U1', 'RN1')])
        self.assertEqual(g['contained'], 1)          # still DISCLOSED
        self.assertEqual(g['containment_blocking'], 0)   # but not blocking

    def test_the_corpus_still_grades_buildable(self):
        """0 boards may gate. The only corpus containments are orangecrab's
        FID2/J5 (frac 1.000) and FID1/J4 (0.867), both marker_class and both
        correct -- fiducials under a connector body."""
        boards = sorted(glob.glob(os.path.join(ROOT, 'kicad_files',
                                               '*.kicad_pcb')))
        self.assertGreaterEqual(len(boards), 30)
        gating = {os.path.basename(b): [(q.a, q.b, q.waiver)
                                        for q in _grade(b)['containment_blocking_pairs']]
                  for b in boards}
        offenders = {k: v for k, v in gating.items() if v}
        self.assertEqual(offenders, {}, offenders)

    def test_bodyless_footprints_are_disclosed(self):
        """A part drawing no .Fab outline cannot be judged by this channel.
        Measured 10-25 per corpus board, so it is a large limit rather than a
        corner case -- and an unjudged part is not a clean part."""
        g = _grade(os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb'))
        self.assertGreater(g['fab_unjudged'], 0)
        self.assertEqual(len(g['fab_unjudged_refs']), g['fab_unjudged'])

    def test_the_bodyless_hole_is_exactly_what_was_measured(self):
        """144 of 1583 footprints draw no .Fab body, and that is the FINAL
        answer, not a TODO.

        Measured over all 33 boards: every one of those 144 draws ZERO .Fab
        geometric primitives -- there is no footprint whose .Fab geometry the
        parser fails to read. So a tolerance or polygon-closure fix moves
        nothing (313 footprints DO have non-closing .Fab chains, tigard Q1
        among them, and all 313 are judged correctly because a bbox is a
        min/max over points).

        This test exists so a future "helpful" parser change that alters the
        count has to come and argue with the number. The dangerous direction
        is a FALLBACK body for bodyless parts: measured, that adds 77 fab
        pairs, 65 above the threshold, and breaks the calibration gate below.
        """
        boards = sorted(glob.glob(os.path.join(ROOT, 'kicad_files',
                                               '*.kicad_pcb')))
        self.assertGreaterEqual(len(boards), 30)
        total_unjudged = sum(_grade(b)['fab_unjudged'] for b in boards)
        self.assertEqual(total_unjudged, 144,
                         f'corpus fab_unjudged moved to {total_unjudged}; if '
                         f'that was deliberate, re-measure the 4-pair census '
                         f'below and this number together')

    def test_no_unjudged_part_has_fab_geometry_to_read(self):
        """The INVARIANT behind the count above, and the stronger claim.

        `fab_unjudged == 144` is a census of THIS corpus, and 11 of the 33
        boards it sweeps are untracked generated fixtures, so the number moves
        when someone regenerates one. This assertion does not depend on corpus
        membership: whatever the set is, every unjudged part must draw NO .Fab
        geometry at all.

        That is what licenses "a parser tolerance or polygon-closure fix moves
        zero footprints" -- not the 144. If this ever fails, the parser really
        is failing to read a body it was handed, and the disclosure-not-a-fix
        conclusion has to be revisited.
        """
        from placement.parser import extract_fab_sides
        boards = sorted(glob.glob(os.path.join(ROOT, 'kicad_files',
                                               '*.kicad_pcb')))
        self.assertGreaterEqual(len(boards), 30)
        readable, checked = [], 0
        for b in boards:
            sides = extract_fab_sides(b)
            for ref in _grade(b)['fab_unjudged_refs']:
                checked += 1
                if sides.get(ref):
                    readable.append((os.path.basename(b), ref))
        self.assertGreater(checked, 0, 'nothing was checked')
        self.assertEqual(readable, [], f'{len(readable)} part(s) are reported '
                                       f'unjudged while their .Fab geometry '
                                       f'parses fine: {readable[:5]}')

    def test_corpus_carries_no_nonexempt_body_containment(self):
        """THE calibration gate for the threshold, sibling of
        test_all_healthy_boards_grade_zero_blocking.

        Measured over all 33 boards: the fab census is exactly 4 pairs, and
        every non-exempt one is a shell KISS three orders of magnitude below
        the threshold (GPDI1/J5 at 0.011, GPDI1/SW1 at 0.001) against a
        measured defect of 1.000. That ~90x separation is what licenses
        CONTAINMENT_FRAC.

        It is also why the ENGINE predicate may use the fab currency and never
        the courtyard: the courtyard ships frac-1.0 containment on four healthy
        boards (esp_prog, orangecrab_ext_pll, rp2350_fpga_eensy_prePlane,
        ulx3s), so a courtyard-based predicate would false-veto legitimate
        poses on 12% of the corpus -- the run-4 lesson in a new costume.
        """
        from placement.legality import CONTAINMENT_FRAC
        boards = sorted(glob.glob(os.path.join(ROOT, 'kicad_files',
                                               '*.kicad_pcb')))
        self.assertGreaterEqual(len(boards), 30)
        census = []
        for b in boards:
            for p in _grade(b)['pairs']:
                if p.kind == 'fab':
                    census.append((os.path.basename(b), p.a, p.b,
                                   p.contained_frac, bool(p.waiver)))
        self.assertEqual(len(census), 4, census)
        offenders = [c for c in census
                     if c[3] >= CONTAINMENT_FRAC and not c[4]]
        self.assertEqual(offenders, [], offenders)
