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


def _pro_waives_courtyards(path):
    import json
    with open(os.path.splitext(path)[0] + '.kicad_pro', 'w',
              encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rule_severities': {
            'courtyards_overlap': 'ignore'}}}}, fh)


def _plan(path, raw):
    import contextlib
    import io as _io
    base = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm'}
    base.update(raw)
    with contextlib.redirect_stdout(_io.StringIO()):
        return floorplan.plan_check(floorplan.intent_from_dict(base, path),
                                    parse_kicad_pcb(path), path)


def _sized_board(td, name, size, parts):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                 ' (37 "F.SilkS" user) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
                 f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (stroke'
                 ' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return path


class ThePhase5VerifiersCases(unittest.TestCase):

    def test_a_waived_project_reads_no_exact_overlap(self):
        """courtyards_overlap ignored: the budget, the pose grader and the
        plan read zero (#1104) -- on the exact key too, or `_grade_worse`
        refuses moves on it. The control, unwaived, reads the overlap."""
        import pose_score
        got = {}
        for waived in (False, True):
            with tempfile.TemporaryDirectory() as td:
                path = _board(td, 'w', [_mk('MK1', 12, 15, locked=True),
                                        _mk('MK2', 19, 15, locked=True)])
                if waived:
                    _pro_waives_courtyards(path)
                res = floorplan.grade(_intent(path), parse_kicad_pcb(path),
                                      path)
                st = pose_score.make_state(parse_kicad_pcb(path), path,
                                           clearance=0.2)
                la = floorplan.PoseGrader(
                    _intent(path), st, blocks={}, clearance=0.2,
                    board_edge_clearance=0.5).legality_at()
            got[waived] = (res.legality['overlap_area_exact'],
                           la['overlap_area_exact'],
                           bool([v for v in res.violations
                                 if v.rule == 'legality']))
        self.assertGreater(got[False][0], 0.0, got)
        self.assertGreater(got[False][1], 0.0, got)
        self.assertTrue(got[False][2], got)
        self.assertEqual(got[True], (0.0, 0.0, False))

    def test_a_posed_view_leaves_a_pin_frame_out(self):
        """rp2350's U8 is a pin frame: a posed view of the state prices the
        overlap the state prices (it read 425.18 with U8's rect, 18.45
        without)."""
        import pose_score
        p = os.path.join(ROOT, 'kicad_files',
                         'rp2350_fpga_eensy_prePlane.kicad_pcb')
        st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.2)
        self.assertIn('U8', set(st.container_refs))
        view = floorplan._PosedState(st)
        self.assertAlmostEqual(view.legality_metrics()['overlap_area'],
                               st.legality_metrics()['overlap_area'],
                               places=4)

    def test_the_decap_rung_compares_the_exact_reading(self):
        """A seat that grows ONLY the exact overlap is reverted."""
        import test_repair_decap_honesty as t
        from placement import seeder
        real = floorplan.PoseGrader.legality_at
        with tempfile.TemporaryDirectory() as td:
            intent, src = t._splitflap3(td)
            home = parse_kicad_pcb(src).footprints['C2']

            def grows(self, *a, **kw):
                got = dict(real(self, *a, **kw))
                p = self.state.parts['C2']
                if abs(p.x - home.x) + abs(p.y - home.y) > 1e-6:
                    got['overlap_area_exact'] = float(
                        got.get('overlap_area_exact') or 0) + 1
                return got
            floorplan.PoseGrader.legality_at = grows
            try:
                on = seeder.repair_placement(
                    parse_kicad_pcb(src), src, intent,
                    group_sources=t.SOURCES, clearance=t.CLEARANCE,
                    repair_decaps=True)
            finally:
                floorplan.PoseGrader.legality_at = real
        row = on['decap_rung']['C2']['tried'][0]
        self.assertEqual(row['result'], 'reverted', row)
        self.assertIn('legality.overlap_area_exact', row['added'])

    def test_the_message_names_the_worst_five_first(self):
        xs = (3, 11, 18, 24, 29, 33, 36)
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'm', [_mk(f'MK{i}', x, 20)
                                    for i, x in enumerate(xs)])
            res = floorplan.grade(_intent(path), parse_kicad_pcb(path), path)
        v = [x for x in res.violations if x.rule == 'legality'][0]
        pairs = v.measured['pairs']
        self.assertGreater(len(pairs), 5)
        self.assertEqual(pairs, sorted(pairs, key=lambda p: (-p[3], -p[2],
                                                             p[0], p[1])))
        self.assertEqual(v.measured['pairs_total'], len(pairs))
        named = v.message.split(' -- worst: ')[1]
        self.assertTrue(named.startswith(', '.join(
            f"{a}/{b} {e:.3f} (rect {r:.3f})"
            for a, b, r, e in pairs[:5])), named)
        self.assertTrue(v.message.endswith(f", +{len(pairs) - 5} more"))

    def test_the_rect_reading_is_the_callers(self):
        """`rect_of` is the RECT reading's source: R's rect moved 50 mm away
        reads no rect overlap, and the exact reading does not move."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'rc', [_mk('MK', 20, 20),
                                     _rect('R', 16.5, 17.0)])
            census = legality.CourtyardCensus(parse_kicad_pcb(path), path)

            def shifted(r):
                g = census.graded_part(r)
                dx = 50.0 if r == 'R' else 0.0
                x0, y0, x1, y1 = g.rect
                return (g.sides, g.side, (x0 + dx, y0, x1 + dx, y1),
                        g.tht_rect if r != 'R' else None)
            own = legality.courtyard_overlap_pairs(census, ['MK', 'R'])
            far = legality.courtyard_overlap_pairs(census, ['MK', 'R'],
                                                   rect_of=shifted)
        self.assertGreater(own[1], 0.0)
        self.assertEqual(far[1], 0.0)
        self.assertAlmostEqual(far[0], own[0], places=6)

    def test_the_zone_bound_is_on_the_outlines(self):
        """Three r = 5 circles in a 20 x 10 zone: their RECTS force 100
        mm2, their outlines about 35.5 -- and the grade passes them at
        61.4. A budget of 62 is satisfiable (no ERROR); 30 is not -- and
        MK2 is LOCKED: a locked member counts (the grade counts its
        overlap), and without it the outlines fit (157 < 200)."""
        for budget, want in ((62.0, False), (30.0, True)):
            with tempfile.TemporaryDirectory() as td:
                path = _board(td, 'z', [_mk('MK1', 5, 5),
                                        _mk('MK2', 10, 5, locked=True),
                                        _mk('MK3', 15, 5)])
                found, meas = _plan(path, {
                    'blocks': [{'name': 'mk', 'refs': ['MK1', 'MK2', 'MK3'],
                                'zone': [0, 0, 20, 10], 'tolerance_mm': 0}],
                    'legality_budget': {'overlap_area': budget}})
            over = [v for v in found if v.rule == 'plan_zone_overfull']
            self.assertEqual(bool(over), want,
                             (budget, [(v.rule, v.message) for v in found]))
        row = meas['zones'][0]
        self.assertLess(row['members_outline_area_mm2'],
                        row['members_area_mm2'])
        self.assertAlmostEqual(over[0].measured['forced_overlap_mm2'],
                               row['members_outline_area_mm2'] - 200.0,
                               places=3)

    def test_the_board_bound_is_on_the_outlines(self):
        """Two r = 5 circles on a 15 x 10 board: rects force 50 mm2, the
        outlines about 7. A budget of 20 is satisfiable; 2 is not."""
        for budget, want in ((20.0, False), (2.0, True)):
            with tempfile.TemporaryDirectory() as td:
                path = _sized_board(td, 'b', (15, 10), [_mk('MK1', 5, 5),
                                                        _mk('MK2', 10, 5)])
                found, meas = _plan(path, {'legality_budget': {
                    'overlap_area': budget, 'oob_count': 0}})
            over = [v for v in found if v.rule == 'plan_board_overfull']
            self.assertEqual(bool(over), want,
                             (budget, [(v.rule, v.message) for v in found],
                              meas.get('outline_area_per_face_mm2')))

    def test_a_silk_only_part_is_named_as_excluded(self):
        """Two parts drawing only silk, crossed: #896 keeps a silk body out
        of every gate, so the budget does not see them -- and says so."""
        part = ('  (footprint "t:SILK" (layer "F.Cu") (at 15 15 {rot})\n'
                '    (property "Reference" "{ref}" (at 0 0) (layer'
                ' "F.SilkS"))\n'
                '    (fp_rect (start -3.5 -1) (end 3.5 1) (stroke (width 0.12)'
                ' (type default)) (layer "F.SilkS"))\n'
                '    (pad "1" smd rect (at -3 0) (size 1 1.5) (layers "F.Cu")'
                ' (net 1 "N1"))\n'
                '    (pad "2" smd rect (at 3 0) (size 1 1.5) (layers "F.Cu")'
                ' (net 2 "N2")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _sized_board(td, 's', (30, 30), [
                part.format(ref='A1', rot=0), part.format(ref='B1', rot=90)])
            res = floorplan.grade(_intent(path), parse_kicad_pcb(path), path)
        self.assertEqual(res.legality['overlap_area_exact'], 0.0)
        self.assertEqual(res.legality['overlap_area_excluded'], 2)


class TheRoundThreeCases(unittest.TestCase):

    def test_the_finding_names_the_parts_it_left_out(self):
        """Two crossed silk-only parts and an overlapping MK pair: the
        finding fails on the MKs and names the silk parts as excluded."""
        part = ('  (footprint "t:SILK" (layer "F.Cu") (at 25 25 {rot})\n'
                '    (property "Reference" "{ref}" (at 0 0) (layer'
                ' "F.SilkS"))\n'
                '    (fp_rect (start -3.5 -1) (end 3.5 1) (stroke (width 0.12)'
                ' (type default)) (layer "F.SilkS"))\n'
                '    (pad "1" smd rect (at -3 0) (size 1 1.5) (layers "F.Cu")'
                ' (net 1 "N1"))\n'
                '    (pad "2" smd rect (at 3 0) (size 1 1.5) (layers "F.Cu")'
                ' (net 2 "N2")))\n')
        with tempfile.TemporaryDirectory() as td:
            path = _sized_board(td, 'x', (40, 40), [
                part.format(ref='A1', rot=0), part.format(ref='B1', rot=90),
                _mk('MK1', 8, 8), _mk('MK2', 14, 8)])
            res = floorplan.grade(_intent(path), parse_kicad_pcb(path), path)
        v = [x for x in res.violations if x.rule == 'legality']
        self.assertEqual(len(v), 1, res.violations)
        self.assertEqual(v[0].measured['excluded'], {'silk': ['A1', 'B1']})

    def test_the_pairs_are_capped(self):
        """Twelve parts on one spot: 66 pairs, the worst 50 written."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'c', [_mk(f'MK{i}', 20, 20) for i in range(12)])
            res = floorplan.grade(_intent(path), parse_kicad_pcb(path), path)
        m = [x for x in res.violations if x.rule == 'legality'][0].measured
        self.assertEqual((len(m['pairs']), m['pairs_total']), (50, 66))
        self.assertEqual(floorplan.MEASURED_PAIRS_CAP, 50)

    def test_a_container_is_named_as_excluded(self):
        p = os.path.join(ROOT, 'kicad_files',
                         'rp2350_fpga_eensy_prePlane.kicad_pcb')
        pcb = parse_kicad_pcb(p)
        census = legality.CourtyardCensus(pcb, p)
        _gp, excluded = legality.courtyard_budget_universe(
            census, list(census.lbs))
        self.assertEqual(excluded['containers'], ['U8'])

    def test_a_waived_project_forces_no_overlap(self):
        """courtyards_overlap ignored: the grade prices no overlap, so the
        zone bound stands down to its WARN (35.5 forced on the outlines)."""
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'zw', [_mk('MK1', 5, 5), _mk('MK2', 10, 5),
                                     _mk('MK3', 15, 5)])
            _pro_waives_courtyards(path)
            found, _m = _plan(path, {
                'blocks': [{'name': 'mk', 'refs': ['MK1', 'MK2', 'MK3'],
                            'zone': [0, 0, 20, 10], 'tolerance_mm': 0}],
                'legality_budget': {'overlap_area': 30.0}})
        self.assertFalse([v for v in found if v.rule == 'plan_zone_overfull'])
        self.assertTrue([v for v in found if v.rule == 'plan_zone_crowded'])

    def test_a_part_at_45_degrees_is_anchor_graded(self):
        """Two 10 x 4 courtyards at 45 degrees in a 12 x 5 zone: no
        rotation of their lattice fits, so the grade reads the zone as an
        anchor and the bound charges nothing; at 0 degrees they fit, and
        80 mm2 in 60 is an ERROR at budget 0."""
        def rot_rect(ref, x, y, rot):
            return (f'  (footprint "t:R" (layer "F.Cu") (at {x} {y} {rot})\n'
                    f'    (property "Reference" "{ref}" (at 0 0) (layer'
                    f' "F.SilkS"))\n'
                    '    (fp_rect (start -5 -2) (end 5 2) (stroke (width 0.05)'
                    ' (type default)) (layer "F.CrtYd"))\n'
                    f'    (pad "1" smd rect (at 0 0 {rot}) (size 0.6 0.6)'
                    ' (layers "F.Cu") (net 2 "N2")))\n')
        got = {}
        for rot in (45, 0):
            with tempfile.TemporaryDirectory() as td:
                path = _board(td, f'r{rot}', [rot_rect('R1', 10, 10, rot),
                                              rot_rect('R2', 25, 25, rot)])
                found, _m = _plan(path, {
                    'blocks': [{'name': 'z', 'refs': ['R1', 'R2'],
                                'zone': [4, 4, 16, 9], 'tolerance_mm': 0}],
                    'legality_budget': {'overlap_area': 0.0}})
            got[rot] = bool([v for v in found
                             if v.rule == 'plan_zone_overfull'])
        self.assertEqual(got, {45: False, 0: True})

    def test_a_declared_rotation_set_can_make_the_zone_an_anchor(self):
        """Seven 10 x 1 bars at 45 degrees in an 8 x 8 zone: on their own
        lattice they fit (a 7.8 mm box), so 70 mm2 in 64 is forced -- an
        ERROR at budget 0. When the block declares rotation_candidates
        [0, 90], the seed may seat them where they do not fit and the grade
        reads the zone as an anchor: the bound must stand down."""
        def bar(ref, x, y):
            return (f'  (footprint "t:BAR" (layer "F.Cu") (at {x} {y} 45)\n'
                    f'    (property "Reference" "{ref}" (at 0 0) (layer'
                    f' "F.SilkS"))\n'
                    '    (fp_rect (start -5 -0.5) (end 5 0.5) (stroke (width'
                    ' 0.05) (type default)) (layer "F.CrtYd"))\n'
                    '    (pad "1" smd rect (at 0 0 45) (size 0.4 0.4) (layers'
                    ' "F.Cu") (net 2 "N2")))\n')
        refs = [f'B{i}' for i in range(7)]
        got = {}
        for cands in (None, [0, 90]):
            block = {'name': 'z', 'refs': refs, 'zone': [16, 16, 24, 24],
                     'tolerance_mm': 0}
            if cands:
                block['rotation_candidates'] = cands
            with tempfile.TemporaryDirectory() as td:
                path = _board(td, 'bars', [bar(r, 20, 5 + 4 * i)
                                           for i, r in enumerate(refs)])
                found, _m = _plan(path, {
                    'blocks': [block],
                    'legality_budget': {'overlap_area': 0.0}})
            got[bool(cands)] = bool([v for v in found
                                     if v.rule == 'plan_zone_overfull'])
        self.assertEqual(got, {False: True, True: False})


def _bar(ref, x, y, rot, half=(5, 0.5), locked=False):
    hx, hy = half
    lock = '    (locked yes)\n' if locked else ''
    return (f'  (footprint "t:BAR" (layer "F.Cu") (at {x} {y} {rot})\n' + lock
            + f'    (property "Reference" "{ref}" (at 0 0) (layer'
            f' "F.SilkS"))\n'
            f'    (fp_rect (start {-hx} {-hy}) (end {hx} {hy}) (stroke (width'
            ' 0.05) (type default)) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at 0 0 {rot}) (size 0.4 0.4) (layers'
            ' "F.Cu") (net 2 "N2")))\n')


class TheZoneBoundsRotationClaims(unittest.TestCase):
    """`_anchor_reachable` takes the seeder's own claims
    (`rotations_for_ref`): a block's decision or candidate set, from ANY
    block, and a locked part's own lattice. Seven 10 x 1 bars in an 8 x 8
    zone: on their 45-degree lattice they fit (a 7.8 mm box) and 70 mm2 in
    64 is forced; at 0 or 90 they do not fit, and the grade reads the zone
    as an anchor."""

    REFS = [f'B{i}' for i in range(7)]

    def _overfull(self, blocks, locked=False, rot=45, half=(5, 0.5),
                  zone=(16, 16, 24, 24)):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, 'zr', [_bar(r, 20, 5 + 4 * i, rot, half,
                                          locked=locked)
                                     for i, r in enumerate(self.REFS)])
            for b in blocks:
                b.setdefault('refs', self.REFS)
            blocks[0]['zone'] = list(zone)
            blocks[0]['tolerance_mm'] = 0
            found, _m = _plan(path, {'blocks': blocks,
                                     'legality_budget': {'overlap_area': 0.0}})
        return bool([v for v in found if v.rule == 'plan_zone_overfull'])

    def test_a_declared_angle_where_they_do_not_fit(self):
        self.assertTrue(self._overfull([{'name': 'z'}]))
        self.assertFalse(self._overfull([{'name': 'z', 'rotation': 0}]))

    def test_any_candidate_where_they_do_not_fit(self):
        """[45, 0]: they fit at 45 and not at 0 -- the seed may pick 0."""
        self.assertFalse(self._overfull([
            {'name': 'z', 'rotation_candidates': [45, 0]}]))

    def test_another_blocks_claim(self):
        """The zone's block declares nothing; a zone-less block naming the
        same parts declares the candidates the seeder honours."""
        self.assertFalse(self._overfull([
            {'name': 'z'}, {'name': 'r', 'rotation_candidates': [0, 90]}]))

    def test_a_locked_part_keeps_its_own_lattice(self):
        """Locked bars never turn, whatever a block declares: at 45 they
        fit, so the forced overlap stands."""
        self.assertTrue(self._overfull([{'name': 'z', 'rotation': 0}],
                                       locked=True))

    def test_an_off_lattice_angle_is_turned_not_looked_up(self):
        """4 x 4 squares at 0 in a 5 x 5 zone, candidates [45]: at 45 their
        box is 5.66 and does not fit. The part's rotation cache has no 45
        entry (and answers a miss with the 0-degree box, which fits)."""
        self.assertTrue(self._overfull([{'name': 'z'}], rot=0, half=(2, 2),
                                       zone=(16, 16, 21, 21)))
        self.assertFalse(self._overfull(
            [{'name': 'z', 'rotation_candidates': [45]}], rot=0, half=(2, 2),
            zone=(16, 16, 21, 21)))


if __name__ == '__main__':
    unittest.main()
