"""#1101: the seeder honours the project's own courtyard waiver.

On KiCad's StickHub demo placed from a pile, `place_seed` never seated the
USB port capacitors C14-C20: the column between the JST ports is 1.14 mm
between their courtyards, and a 2012 cap needs at least 1.45. The human
layout overlaps those courtyards by 0.68 mm -- which the project allows:
StickHub's `.kicad_pro` sets `courtyards_overlap` to `ignore`, and since
#1095 check_assembly grades it that way. The seeder refused anyway, so the
generator was stricter than the checker on exactly the board that says so.

At `ignore` (the author's, `legality.courtyard_severity_of`: not a
tool-written legacy ignore) the seat now spaces PAD COPPER at the seat
clearance instead of courtyards; pads, holes and .Fab bodies are still
checked. Measured on StickHub's locked pile, seed 1: unseated 6 -> 1 (C17,
whose census now names C16 as the blocker), 50 min -> 5 min.

And the no-pose census stops naming parts on the OTHER face as blockers of
an SMD part (it named StickHub's back-side U1, C23, C27 for a front cap).
"""

import json
import os
import random
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

#: A 12 x 4 board and two 5 x 3 parts whose PADS are small and central. Side
#: by side they need 10 mm plus clearance of courtyard, which the board has;
#: stacked on one axis they need their courtyards to overlap... so the board
#: is made 9 mm wide: both fit only if the courtyards may overlap (the pads
#: stay 3+ mm apart).
BOARD = (
    '(kicad_pcb (version 20240108) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (net 0 "") (net 1 "A") (net 2 "B")\n'
    '  (gr_rect (start 0 0) (end 9 4) (stroke (width 0.1) (type default))'
    ' (layer "Edge.Cuts"))\n'
    '{parts})\n')
PART = (
    '  (footprint "t:{ref}" (layer "F.Cu") (at {x} 20)\n'
    '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -2.5 -1.5) (end 2.5 1.5) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "1" smd rect (at -0.4 0) (size 0.5 0.5) (layers "F.Cu")'
    ' (net 1 "A"))\n'
    '    (pad "2" smd rect (at 0.4 0) (size 0.5 0.5) (layers "F.Cu")'
    ' (net 2 "B")))\n')


def seed(severity):
    from kicad_parser import parse_kicad_pcb
    from placement import seeder
    from placement.floorplan import empty_intent
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(BOARD.format(parts=PART.format(ref='A', x=30)
                                  + PART.format(ref='B', x=40)))
        sev = {} if severity is None else {'courtyards_overlap': severity}
        with open(os.path.join(td, 'b.kicad_pro'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'board': {'design_settings':
                                 {'rule_severities': sev}}}, fh)
        return seeder.seed_from_intent(
            parse_kicad_pcb(path), path, empty_intent(path),
            random.Random('1'), clearance=0.15, board_edge_clearance=0.1,
            grid_step=0.1)


class TestWaivedCourtyards(unittest.TestCase):
    def test_the_projects_ignore_lets_both_parts_seat(self):
        self.assertEqual(seed('ignore')['unseated'], [])

    def test_at_error_the_courtyards_still_keep_them_apart(self):
        for sev in ('error', 'warning', None):
            self.assertEqual(len(seed(sev)['unseated']), 1, sev)

    def test_pads_are_still_spaced(self):
        """With courtyards waived the pad boxes keep the clearance: the
        seated parts' pad copper is at least 0.15 mm apart."""
        res = seed('ignore')
        pose = {p['reference']: (p['new_x'], p['new_y'])
                for p in res['placements']}
        (ax, ay), (bx, by) = pose['A'], pose['B']
        # pad boxes are +-0.65 x +-0.25 around each centre
        gap_x, gap_y = abs(ax - bx) - 1.3, abs(ay - by) - 0.5
        self.assertGreaterEqual(max(gap_x, gap_y), 0.15 - 1e-6, pose)


class TestPadRequirementNotSeatClearance(unittest.TestCase):
    """#1101 verifier: pad boxes priced at the SEAT clearance let pairs sit
    under their net class (StickHub at --clearance 0.1, class 0.15). The
    waived-courtyard seat now asks each pair's own pad requirement."""

    def test_a_pad_gap_under_the_net_class_is_refused(self):
        import pose_score
        from kicad_parser import parse_kicad_pcb
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'b.kicad_pcb')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(BOARD.format(
                    parts=PART.format(ref='A', x=30).replace('(at 30 20)',
                                                             '(at 2.5 2)')
                    # B STARTS on A's pads, as a pile part does: the
                    # seed-relative pad layer then admits anything no worse
                    # than that, so only the absolute test can refuse.
                    + PART.format(ref='B', x=40).replace('(at 40 20)',
                                                         '(at 3.3 2)')))
            with open(os.path.join(td, 'b.kicad_pro'), 'w',
                      encoding='utf-8') as fh:
                json.dump({'board': {'design_settings': {'rule_severities': {
                    'courtyards_overlap': 'ignore'}}},
                    'net_settings': {'classes': [
                        {'name': 'Default', 'clearance': 0.3,
                         'track_width': 0.2, 'via_diameter': 0.6,
                         'via_drill': 0.3}]}}, fh)
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.1,
                                       board_edge_clearance=0.1)
            self.assertTrue(st.courtyards_ignored)
            # B's pads 0.2 mm from A's: above the seat's 0.1, under the
            # class's 0.3.
            near = st.candidate_valid('B', 2.5 + 1.3 + 0.2, 2.0, 0.0)
            far = st.candidate_valid('B', 2.5 + 1.3 + 0.5, 2.0, 0.0)
        self.assertFalse(near)
        self.assertTrue(far)


class TestEscapeBranchChecksBodies(unittest.TestCase):
    """A part coming in from the pile takes candidate_valid's off-board
    escape branch, which accepted any pose that lowered the off-board amount
    with no courtyard overlap -- and never asked about BODIES. StickHub's
    lying-down C38 (a 6.3 x 11.5 mm .Fab over a pad-sized courtyard) was
    seated over eleven parts that way, and inside Y1 before #1101. Any
    board, whatever its courtyard severity."""

    def test_a_large_body_is_not_seated_over_a_small_part(self):
        import pose_score
        from kicad_parser import parse_kicad_pcb
        big = (
            '  (footprint "t:L" (layer "F.Cu") (at 40 20)\n'
            '    (property "Reference" "L" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -0.6 -0.4) (end 0.6 0.4) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (fp_rect (start -3 -1.5) (end 3 1.5) (stroke (width 0.05)'
            ' (type default)) (layer "F.Fab"))\n'
            '    (pad "1" smd rect (at -0.3 0) (size 0.3 0.3) (layers "F.Cu")'
            ' (net 1 "A"))\n'
            '    (pad "2" smd rect (at 0.3 0) (size 0.3 0.3) (layers "F.Cu")'
            ' (net 2 "B")))\n')
        small = (
            '  (footprint "t:S" (layer "F.Cu") (at 7 2)\n'
            '    (property "Reference" "S" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -0.5 -0.3) (end 0.5 0.3) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (fp_rect (start -0.4 -0.2) (end 0.4 0.2) (stroke (width 0.05)'
            ' (type default)) (layer "F.Fab"))\n'
            '    (pad "1" smd rect (at -0.25 0) (size 0.2 0.2) (layers "F.Cu")'
            ' (net 1 "A"))\n'
            '    (pad "2" smd rect (at 0.25 0) (size 0.2 0.2) (layers "F.Cu")'
            ' (net 2 "B")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'b.kicad_pcb')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(BOARD.format(parts=big + small))
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.15,
                                       board_edge_clearance=0.1)
            # L's courtyard clears S at (5, 2) by 0.9 mm; its body does not.
            over = st.candidate_valid('L', 5.0, 2.0, 0.0, exclude=set())
            clear = st.candidate_valid('L', 5.0, 2.0, 0.0, exclude={'S'})
        self.assertFalse(over)
        self.assertTrue(clear)


HOLE = (
    '  (footprint "t:H" (layer "F.Cu") (at {x} {y})\n'
    '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_circle (center 0 0) (end 1 0) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "" np_thru_hole circle (at 0 0) (size 1.5 1.5) (drill 1.5)'
    ' (layers "*.Cu" "*.Mask")))\n')


def ignore_board(td, parts, extra_pro=None):
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD.format(parts=parts))
    pro = {'board': {'design_settings': {'rule_severities': {
        'courtyards_overlap': 'ignore'}}}}
    pro.update(extra_pro or {})
    with open(os.path.join(td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(pro, fh)
    return path


class TestHolesAreNotWaived(unittest.TestCase):
    """#1101 review: with the courtyard waived, nothing refused two drill
    holes on one spot -- the courtyard was the only check that did."""

    def test_stacked_holes_are_refused(self):
        import pose_score
        from kicad_parser import parse_kicad_pcb
        with tempfile.TemporaryDirectory() as td:
            p = ignore_board(td, HOLE.format(ref='H1', x=3, y=2)
                             + HOLE.format(ref='H2', x=3.1, y=2))
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.15,
                                       board_edge_clearance=0.1)
            self.assertTrue(st.courtyards_ignored)
            stacked = st.candidate_valid('H2', 3.5, 2.0, 0.0, exclude=set())
            apart = st.candidate_valid('H2', 6.5, 2.0, 0.0, exclude=set())
        self.assertFalse(stacked)
        self.assertTrue(apart)

    def test_a_declared_pose_on_a_hole_is_refused(self):
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement.seeder import _fixed_pose_check
        with tempfile.TemporaryDirectory() as td:
            p = ignore_board(td, HOLE.format(ref='H1', x=3, y=2)
                             + HOLE.format(ref='H2', x=6.5, y=2))
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.15,
                                       board_edge_clearance=0.1)
            _how, _why, stacked = _fixed_pose_check(
                st, 'H2', (3.1, 2.0, 0.0), {'H1': (3.0, 2.0, 0.0)})
            _how, _why, apart = _fixed_pose_check(
                st, 'H2', (6.5, 2.0, 0.0), {'H1': (3.0, 2.0, 0.0)})
        self.assertIn('H1', stacked)
        self.assertIn('drill', stacked['H1'])
        self.assertNotIn('H1', apart)


BODY = (
    '  (footprint "t:{ref}" (layer "F.Cu") (at {x} 2 {rot})\n'
    '    (property "Reference" "{ref}" (at 0 0 {rot}) (layer "F.SilkS"))\n'
    '    (fp_rect (start -{c} -{c}) (end {c} {c}) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (fp_rect (start -{f} -{f}) (end {f} {f}) (stroke (width 0.05)'
    ' (type default)) (layer "F.Fab"))\n'
    '    (pad "1" smd rect (at 0 0 {rot}) (size 0.3 0.3) (layers "F.Cu")'
    ' (net {n} "{net}")))\n')


def body(ref, x, n, rot=0, c=1.0, f=1.0):
    return BODY.format(ref=ref, x=x, rot=rot, c=c, f=f, n=n,
                       net='AB'[n - 1])


class TestBodiesAreNotWaived(unittest.TestCase):
    """#1101 review: a courtyard is body + margin, and the project waives
    the MARGIN. With courtyards waived the only body check left was
    containment (half the smaller body or more), so two bodies could be
    seated a quarter inside each other with their pads clear -- StickHub
    from a pile: 14 .Fab overlaps (C17 45% inside J2) on a `buildable`
    board, where the human layout has none."""

    def _state(self, td, parts, size=None):
        import pose_score
        from kicad_parser import parse_kicad_pcb
        p = ignore_board(td, parts)
        if size:
            with open(p, encoding='utf-8') as fh:
                text = fh.read().replace('(end 9 4)', '(end %s %s)' % size)
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(text)
        st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.15,
                                   board_edge_clearance=0.1)
        self.assertTrue(st.courtyards_ignored)
        return st

    def test_overlapping_bodies_are_refused_abutting_ones_are_not(self):
        # 2 x 2 bodies, pads 0.3 mm. A's body spans x 1..3; B at 3.5
        # overlaps it 0.5 x 2 (a quarter), at 4.0 the bodies abut.
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td, body('A', 2, 1) + body('B', 7, 2))
            inside = st.candidate_valid('B', 3.5, 2.0, 0.0)
            abut = st.candidate_valid('B', 4.0, 2.0, 0.0)
        self.assertFalse(inside)
        self.assertTrue(abut)

    def test_a_diagonal_body_is_measured_as_drawn(self):
        """The drawn bodies decide, as check_assembly's fab channel measures
        them. Two 2 x 2 bodies at 45 degrees offset (1.5, 1.5) clear each
        other by 0.12 mm as diamonds while their 2.83 mm boxes overlap by
        1.33 x 1.33; at (1.2, 1.2) the diamonds overlap too."""
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td, body('A', 4, 1, rot=45, c=0.4)
                             .replace('(at 4 2 45)', '(at 4 4 45)')
                             + body('B', 9, 2, rot=45, c=0.4),
                             size=(12, 12))
            clear = st.candidate_valid('B', 5.5, 5.5, 45.0)
            hit = st.candidate_valid('B', 5.2, 5.2, 45.0)
        self.assertTrue(clear)
        self.assertFalse(hit)


class TestOffsetCopperIsSeen(unittest.TestCase):
    """#1101 review: a padbox is built from pad ANCHORS, so copper offset
    from its drill (castellated paddles) lay outside it; a padbox prefilter
    let a pad seat on that copper. S starts on O's copper (a pile), so
    the seed-relative pads_ok admits the move and only the waived seat
    can refuse it."""

    def test_a_pad_on_offset_copper_is_refused(self):
        import pose_score
        from kicad_parser import parse_kicad_pcb
        parts = (
            '  (footprint "t:O" (layer "F.Cu") (at 3 2)\n'
            '    (property "Reference" "O" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -0.6 -0.6) (end 0.6 0.6) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" thru_hole rect (at 0 0) (size 1 1)'
            ' (drill 0.5 (offset 2 0)) (layers "*.Cu" "*.Mask")'
            ' (net 1 "A")))\n'
            '  (footprint "t:S" (layer "F.Cu") (at 5.3 2)\n'
            '    (property "Reference" "S" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -0.4 -0.4) (end 0.4 0.4) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu")'
            ' (net 2 "B")))\n')
        with tempfile.TemporaryDirectory() as td:
            p = ignore_board(td, parts)
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.15,
                                       board_edge_clearance=0.1)
            on_copper = st.candidate_valid('S', 5.3, 2.0, 0.0, exclude=set())
            clear = st.candidate_valid('S', 8.0, 2.0, 0.0, exclude=set())
        self.assertFalse(on_copper)
        self.assertTrue(clear)


class TestIgnoreBoardEmitsNoOverlapBudget(unittest.TestCase):
    def test_overlap_area_is_withheld_with_the_reason(self):
        from kicad_parser import parse_kicad_pcb
        from placement.floorplan import emit_intent
        with tempfile.TemporaryDirectory() as td:
            p = ignore_board(td, PART.format(ref='A', x=30).replace(
                '(at 30 20)', '(at 3 2)'))
            doc = emit_intent(parse_kicad_pcb(p), p)
        self.assertNotIn('overlap_area', doc.get('legality_budget') or {})
        self.assertIn('courtyards_overlap to ignore',
                      doc['context']['budget_withheld']['overlap_area'])


class TestHolePairsRecorded(unittest.TestCase):
    """#1100: the hole channel names its pairs too (it recorded none)."""

    def test_a_pad_in_a_hole_keepout_is_a_named_pair(self):
        from kicad_parser import parse_kicad_pcb
        from placement.legality import grade_pad_legality
        with tempfile.TemporaryDirectory() as td:
            p = ignore_board(td, HOLE.format(ref='H1', x=3, y=2)
                             + PART.format(ref='A', x=30).replace(
                                 '(at 30 20)', '(at 3.5 2)'))
            g = grade_pad_legality(parse_kicad_pcb(p), 0.15, worst_n=0,
                                   pcb_file=p)
        self.assertGreaterEqual(g['hole_conflicts'], 1)
        self.assertIn(['A', 'H1'], g['hole_conflict_pairs'])


class TestCensusSides(unittest.TestCase):
    def test_a_back_side_part_is_not_a_blocker_of_a_front_part(self):
        """The eviction census lists only parts sharing a face (or drilled);
        it named StickHub's back-side U1, C23, C27 for a front cap."""
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        back = PART.replace('(layer "F.Cu") (at', '(layer "B.Cu") (at')             .replace('"F.CrtYd"', '"B.CrtYd"').replace(
            '(layers "F.Cu")', '(layers "B.Cu")')
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'b.kicad_pcb')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(BOARD.format(
                    parts=PART.format(ref='A', x=30).replace('(at 30 20)',
                                                             '(at 3 2)')
                    + PART.format(ref='F', x=5).replace('(at 5 20)',
                                                        '(at 5 2)')
                    + back.format(ref='K', x=4).replace('(at 4 20)',
                                                        '(at 4 2)')))
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.15,
                                       board_edge_clearance=0.1)
            got = seeder._evict_candidates(st, 'A', 3.0, 2.0, {'F', 'K'},
                                           set())
        self.assertIn('F', got)
        self.assertNotIn('K', got)


if __name__ == '__main__':
    unittest.main()
