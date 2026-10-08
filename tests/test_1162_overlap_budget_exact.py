"""#1162: the floorplan overlap budget is graded on check_assembly's geometry.

`legality_budget.overlap_area` was graded on the search's RECTS while
check_assembly grades drawn outlines (#1094). On glasgow g4 check_floorplan
said `4.909 exceeds the declared budget 0.000` -- 3.93 mm2 of it MK1-4's
r = 5 mm circle courtyards graded as 10 x 10 mm squares -- and check_assembly
said buildable. The error named no pair, so an agent rebuilt them from a
different universe (logos counted as 1 x 1 mm bodies) and threw a legal
layout away.

What must hold:

  * the budget is graded on the drawn outlines (`overlap_area_exact`), and
    the error names every pair with BOTH readings;
  * the emitter bakes the reading the rule grades, or the budget fails the
    board it was emitted from;
  * a plan's locked-pair overlap is measured the same way;
  * the public routes (`courtyard_overlap_pairs`, `courtyard_census`) answer
    in the grade's universe: no synthetic logo body, no container.
"""

import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan, legality                    # noqa: E402

RUN_ALL_TIMEOUT = 900


def _mk(ref, x, y, locked=False):
    """MountingHole-style: one PTH pad and an r = 5 circle F.CrtYd."""
    lock = '    (locked yes)\n' if locked else ''
    return (f'  (footprint "t:MH" (layer "F.Cu") (at {x} {y})\n' + lock +
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_circle (center 0 0) (end 5 0) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" thru_hole circle (at 0 0) (size 6 6) (drill 3.5)'
            ' (layers "*.Cu" "*.Mask") (net 1 "N1")))\n')


def _rect(ref, x, y, half=(1.5, 1.0), locked=False):
    lock = '    (locked yes)\n' if locked else ''
    hx, hy = half
    return (f'  (footprint "t:R" (layer "F.Cu") (at {x} {y})\n' + lock +
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-hx} {-hy}) (end {hx} {hy}) (stroke'
            ' (width 0.05) (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu")'
            ' (net 2 "N2")))\n')


def _logo(ref, x, y):
    return (f'  (footprint "t:LOGO" (layer "F.Cu") (at {x} {y})\n'
            '    (locked yes)\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -3 -1) (end 3 1) (stroke (width 0.1)'
            ' (type default)) (layer "F.SilkS")))\n')


def _board(td, name, parts):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                 ' (44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
                 '  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return path


def _intent(path, budget=0.0):
    return floorplan.intent_from_dict(
        {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
         'legality_budget': {'overlap_area': budget}}, path)


#: R spans (13, 14)-(16, 16): its corner (16, 16) is inside MK's 10 x 10
#: square (1.0 mm2 of rect overlap) and 5.66 mm from MK's centre (20, 20) --
#: outside the r = 5 circle.
CORNER = (14.5, 15.0)


class TheBudget(unittest.TestCase):

    def test_a_corner_in_the_square_not_the_circle_passes(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'c', [_mk('MK', 20, 20), _rect('R', *CORNER)])
            pcb = parse_kicad_pcb(path)
            res = floorplan.grade(_intent(path), pcb, path)
        self.assertFalse([v for v in res.violations if v.rule == 'legality'],
                         res.violations)
        self.assertGreater(res.legality['overlap_area'], 0.0)
        self.assertEqual(res.legality['overlap_area_exact'], 0.0)

    def test_a_real_overlap_fails_and_names_its_pair(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'r', [_mk('MK', 20, 20), _rect('R', 16.5, 17.0)])
            pcb = parse_kicad_pcb(path)
            res = floorplan.grade(_intent(path), pcb, path)
        v = [x for x in res.violations if x.rule == 'legality']
        self.assertEqual(len(v), 1, res.violations)
        m = v[0].measured
        self.assertGreater(m['overlap_area'], 0.0)
        self.assertGreater(m['overlap_area_rect'], m['overlap_area'])
        self.assertEqual([p[:2] for p in m['pairs']], [['MK', 'R']])
        self.assertAlmostEqual(m['pairs'][0][3], m['overlap_area'], places=4)
        self.assertIn('MK/R', v[0].message)

    def test_the_emitter_bakes_the_graded_reading(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'e', [_mk('MK', 20, 20), _rect('R', *CORNER)])
            pcb = parse_kicad_pcb(path)
            doc = floorplan.emit_intent(pcb, path)
        self.assertEqual(doc['legality_budget'].get('overlap_area'), 0.0, doc)


class APlansLockedPairs(unittest.TestCase):

    def test_locked_parts_are_measured_on_their_outlines(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'p', [_mk('MK', 20, 20, locked=True),
                                    _rect('R', *CORNER, locked=True)])
            pcb = parse_kicad_pcb(path)
            found, _meas = floorplan.plan_check(_intent(path), pcb, path)
        self.assertFalse([v for v in found
                          if v.rule.startswith('plan_fixed_overlap')], found)


class TheUniverse(unittest.TestCase):

    def test_logos_are_not_bodies(self):
        """J5 against two locked pad-less logos (g8's shape): the grade and
        both public routes say 0.0; the reconstruction #1162's agent used
        says 2.0."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'g', [_rect('J5', 20, 20, half=(4.0, 3.0)),
                                    _logo('REF**', 19, 20),
                                    _logo('REF**', 21, 20)])
            pcb = parse_kicad_pcb(path)
            res = floorplan.grade(_intent(path), pcb, path)
            census = legality.CourtyardCensus(pcb, path)
            pub = legality.courtyard_overlap_pairs(census, list(census.lbs))
            cg = legality.courtyard_census(pcb, path)
            old = legality.placement_overlap_area(
                legality.graded_parts_from_file(pcb, path))
        self.assertEqual(res.legality['overlap_area_exact'], 0.0)
        self.assertEqual(pub[0], 0.0)
        self.assertEqual(cg.overlap_exact, 0.0)
        self.assertGreater(old, 0.0, 'the fixture no longer reproduces the '
                           'reconstruction the issue measured')

    def test_the_three_routes_agree_on_the_corpus(self):
        import run_utils
        boards = run_utils.corpus_boards()
        if not boards:
            self.skipTest('git cannot list the corpus here')
        for b in boards:
            path = os.path.join(ROOT, b)
            pcb = parse_kicad_pcb(path)
            ctx = floorplan._grade_ctx(_intent(path, 1e9), pcb, path)[0]
            ex = floorplan._ctx_overlap_exact(ctx)[0]
            census = legality.CourtyardCensus(pcb, path)
            pub = legality.courtyard_overlap_pairs(census, list(census.lbs))[0]
            self.assertAlmostEqual(ex, pub, places=3, msg=b)
            self.assertAlmostEqual(ex, census.grade().overlap_exact, places=3,
                                   msg=b)


class APoseComparison(unittest.TestCase):

    def test_legality_at_carries_the_exact_reading(self):
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'l', [_mk('MK', 20, 20), _rect('R', *CORNER)])
            pcb = parse_kicad_pcb(path)
            st = pose_score.make_state(pcb, path, clearance=0.2)
            intent = _intent(path)
            g = floorplan.PoseGrader(intent, st, blocks={}, clearance=0.2,
                                     board_edge_clearance=0.5)
            here = g.legality_at()
            moved = g.legality_at(poses={'R': (16.5, 17.0, 0.0)})
        self.assertEqual(here['overlap_area_exact'], 0.0)
        self.assertGreater(here['overlap_area'], 0.0)
        self.assertGreater(moved['overlap_area_exact'], 0.0)

    def test_a_seat_that_buys_exact_overlap_is_worse(self):
        """`_grade_worse` from the corner pose (rect 1.0, exact 0.0) to one
        beside MK's middle (rect still 1.0, now inside the circle): the
        rect reading cannot see it; the exact one must."""
        import pose_score
        from placement import seeder
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'w', [_mk('MK', 20, 20), _rect('R', *CORNER)])
            pcb = parse_kicad_pcb(path)
            st = pose_score.make_state(pcb, path, clearance=0.2)
            g = floorplan.PoseGrader(floorplan.intent_from_dict(
                {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'},
                path), st, blocks={}, clearance=0.2,
                board_edge_clearance=0.5)
            rows = seeder._grade_worse(g, 'R', 0.0, CORNER, (14.0, 20.0),
                                       set(), {})
        keys = {r.get('budget') for r in rows}
        self.assertEqual(keys, {'overlap_area_exact'}, rows)


if __name__ == '__main__':
    unittest.main()
