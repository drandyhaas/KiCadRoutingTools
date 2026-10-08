#!/usr/bin/env python3
"""#1106: a part with no drawn body may not sit under another part's body.

StickHub's J9 (a 1-pad GND land) and JP1 (a solder jumper) draw no courtyard
and no .Fab body. On a project that waives courtyard overlap (#1101), every
body check left reads drawn .Fab bodies only, so run 38 seeded both inside
U1's LQFP-48 body and check_assembly, check_floorplan and render_placement
all said buildable; only a hand-written keep-out caught it, after the run's
best board had been built on it.

A body-less part's COPPER PADS are what it occupies. `pads_under_body_frac`
(the share of that copper under a same-face drawn body, at CONTAINMENT_FRAC)
is now one predicate, read by check_assembly's containment channel and by
the waived seat in quench -- one-directional, so a body-less module never
"contains" its neighbours (the run-6 Teensy40 lesson).
"""
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BOARD = (
    '(kicad_pcb (version 20240108) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user)\n'
    '    (49 "F.Fab" user))\n'
    '  (net 0 "") (net 1 "A") (net 2 "GND")\n'
    '  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default))'
    ' (layer "Edge.Cuts"))\n'
    # U: a 7 x 7 body with pads only on its rim (a QFP's inner floor is empty)
    '  (footprint "t:U" (layer "F.Cu") (at 8 10)\n'
    '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -4.5 -4.5) (end 4.5 4.5) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (fp_rect (start -3.5 -3.5) (end 3.5 3.5) (stroke (width 0.1)'
    ' (type default)) (layer "F.Fab"))\n'
    '    (pad "1" smd rect (at -4 0) (size 0.8 0.4) (layers "F.Cu") (net 1 "A"))\n'
    '    (pad "2" smd rect (at 4 0) (size 0.8 0.4) (layers "F.Cu") (net 2 "GND")))\n'
    # J: one pad, no courtyard, no fab -- StickHub's J9
    '  (footprint "t:J" (layer "F.Cu") (at {jx} {jy})\n'
    '    (property "Reference" "J9" (at 0 0) (layer "F.SilkS"))\n'
    '    (pad "1" smd rect (at 0 0) (size 1.5 1.5) (layers "F.Cu") (net 2 "GND")))\n'
    ')\n')


def make(td, jx, jy, severity='ignore'):
    p = os.path.join(td, 'b.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(BOARD.format(jx=jx, jy=jy))
    with open(os.path.join(td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rule_severities': {
            'courtyards_overlap': severity}}}}, fh)
    return p


def grade(path):
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_body_overlap
    return grade_body_overlap(parse_kicad_pcb(path), 0.15, pcb_file=path)


class TestChecker(unittest.TestCase):
    def test_a_pad_under_the_body_gates(self):
        with tempfile.TemporaryDirectory() as td:
            g = grade(make(td, 8, 10))
        under = [p for p in g['containment_blocking_pairs']
                 if p.kind == 'pads_under_body']
        self.assertEqual([(p.a, p.b) for p in under], [('J9', 'U1')])

    def test_beside_the_body_is_clean(self):
        with tempfile.TemporaryDirectory() as td:
            g = grade(make(td, 20, 10))
        self.assertFalse([p for p in g['pairs']
                          if p.kind == 'pads_under_body'])

    def test_any_severity_gates(self):
        # The checker does not depend on the courtyard waiver: a pad under a
        # body cannot be assembled whatever the project says of courtyards.
        with tempfile.TemporaryDirectory() as td:
            g = grade(make(td, 8, 10, severity='error'))
        self.assertTrue([p for p in g['containment_blocking_pairs']
                         if p.kind == 'pads_under_body'])


class TestGenerator(unittest.TestCase):
    def _state(self, td, jx, jy, severity='ignore'):
        import pose_score
        from kicad_parser import parse_kicad_pcb
        p = make(td, jx, jy, severity)
        return pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.15,
                                     board_edge_clearance=0.1)

    def test_the_waived_seat_refuses_a_pad_under_the_body(self):
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td, 20, 10)
            self.assertTrue(st.courtyards_ignored)
            under = st.candidate_valid('J9', 8.0, 10.0, 0.0, exclude=set())
            beside = st.candidate_valid('J9', 20.0, 10.0, 0.0, exclude=set())
        self.assertFalse(under)
        self.assertTrue(beside)

    def test_coming_home_from_the_pile_is_refused_too(self):
        # J9 starts OFF the board (a pile), so a rejected candidate reaches
        # candidate_valid's escape branch, which licenses moves toward the
        # board -- measured on run 38's pile, it accepted J9 under U1's body
        # after the waived seat had refused it.
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td, 40, 10)
            under = st.candidate_valid('J9', 8.0, 10.0, 0.0, exclude=set())
            beside = st.candidate_valid('J9', 20.0, 10.0, 0.0, exclude=set())
        self.assertFalse(under)
        self.assertTrue(beside)

    def test_at_error_severity_the_seat_still_refuses(self):
        # Review of #1106: the checker grades at every severity, and a body
        # drawn LARGER than its courtyard leaves room the courtyard test does
        # not see. U1's courtyard cut to +-4.5 x +-1, its body still +-3.5:
        # J9 at (8, 12.5) is clear of the courtyard and under the body.
        import pose_score
        from kicad_parser import parse_kicad_pcb
        with tempfile.TemporaryDirectory() as td:
            p = make(td, 20, 10, severity='error')
            txt = open(p, encoding='utf-8').read().replace(
                '(fp_rect (start -4.5 -4.5) (end 4.5 4.5)',
                '(fp_rect (start -4.5 -1) (end 4.5 1)')
            with open(p, 'w', encoding='utf-8') as fh:
                fh.write(txt)
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.15,
                                       board_edge_clearance=0.1)
            self.assertFalse(st.courtyards_ignored)
            under = st.candidate_valid('J9', 8.0, 12.5, 0.0, exclude=set())
            beside = st.candidate_valid('J9', 20.0, 10.0, 0.0, exclude=set())
        self.assertFalse(under)
        self.assertTrue(beside)

    def test_moving_the_body_over_the_pad_is_refused_too(self):
        with tempfile.TemporaryDirectory() as td:
            st = self._state(td, 22, 10)
            over = st.candidate_valid('U1', 22.0, 10.0, 0.0, exclude=set())
        self.assertFalse(over)


if __name__ == '__main__':
    unittest.main()
