"""#1182: the movers see the courtyard pairs check_assembly gates.

One-Air-Max draws a .Fab body and no courtyard on 197 of 204 parts. The search
seated against PAD BOXES while check_assembly --baseline graded courtyards on
the occupancy (fab body plus pads), so seeds the generator called legal graded
NOT BUILDABLE -- and nothing could fix them:

  * `--repair` read no courtyard pair at all (`Repair census: 0 conflict
    pair(s)` while C27/L1, U5/U8, D6/D7 and JP3/U12 gated);
  * `--reseat` judged its gate's overlap on the search's own rects
    (`overlap 0.4323->0.4323`);
  * `body_model` (#916) had no CLI.

What must hold now:

  * `place_seed --body-model` reaches EVERY search state a place_seed run
    builds (seed, polish, post-polish re-seat, repair, reseat) -- and only the
    neighbour/body currency moves: an armed PoseGrader grades exactly what an
    unarmed one does, because the floorplan grade reads the courtyard ladder;
  * `--repair --baseline B` charges check_assembly's gating courtyard pairs
    and re-grades them after the moves: armed it clears them, unarmed it
    says UNRESOLVED and why; without a baseline it reports and charges none;
  * `--reseat`'s gate measures overlap on check_assembly's geometry;
  * a declared fixed pose is checked on that geometry too (a pad-box rect
    used to screen out two bodies meeting outside their pads);
  * armed, the missing-courtyard warning names only real pad-box parts.
"""

import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import legality                               # noqa: E402

RUN_ALL_TIMEOUT = 1800


def _part(ref, x, y, half=(2.0, 1.0), net=1):
    """A courtyard-LESS SMD part: a .Fab body `2*half`, two 0.6 mm pads
    0.6 mm apart at its centre -- its pad box is a sliver of its body."""
    hx, hy = half
    return (f'  (footprint "t:B" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-hx} {-hy}) (end {hx} {hy}) (stroke'
            f' (width 0.1) (type default)) (layer "F.Fab"))\n'
            f'    (pad "1" smd rect (at -0.3 0) (size 0.4 0.6) (layers "F.Cu")'
            f' (net {net} "N{net}"))\n'
            f'    (pad "2" smd rect (at 0.3 0) (size 0.4 0.6) (layers "F.Cu")'
            f' (net {net + 1} "N{net + 1}")))\n')


def _write(td, name, parts, size=(40, 20)):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                 ' (44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1") (net 2 "N2") (net 3 "N3")'
                 ' (net 4 "N4")\n'
                 f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (stroke'
                 ' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return path


def _intent(td, refs, size=(40, 20)):
    path = os.path.join(td, 'intent.json')
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                   'envelope': {'rect': [0.0, 0.0, float(size[0]),
                                         float(size[1])],
                                'tolerance_mm': 0.5},
                   'blocks': [{'name': 'all', 'refs': list(refs),
                               'zone': [0.5, 0.5, size[0] - 0.5,
                                        size[1] - 0.5],
                               'tolerance_mm': 0.5}]}, fh)
    return path


def _fixture(td):
    """The issue's minimal input: A and B overlap by their BODIES (0.5 x 2 =
    1.0 mm2, depth 0.5 mm) and not by their pad boxes, both MOVED against
    the baseline -- one gating courtyard pair, nothing else wrong."""
    base = _write(td, 'base', [_part('A', 8, 10), _part('B', 30, 10, net=3)])
    moved = _write(td, 'moved', [_part('A', 10, 10),
                                 _part('B', 13.5, 10, net=3)])
    return base, moved, _intent(td, ['A', 'B'])


def _run(argv):
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, timeout=1200)


def _gating(board, baseline):
    pcb = parse_kicad_pcb(board)
    g = legality.CourtyardCensus(pcb, board).grade(
        moved=legality.moved_refs(pcb, parse_kicad_pcb(baseline)))
    return [(q.a, q.b) for q in g.gating]


class TheFixture(unittest.TestCase):

    def test_one_gating_pair_on_bodies_and_none_on_pad_boxes(self):
        with tempfile.TemporaryDirectory() as td:
            base, moved, _ = _fixture(td)
            self.assertEqual(_gating(moved, base), [('A', 'B')])
            g = legality.grade_body_overlap(parse_kicad_pcb(moved), 0.2,
                                            pcb_file=moved)
            self.assertEqual(g['blocking'], 0)


class TheRepair(unittest.TestCase):

    def _repair(self, td, *extra):
        base, moved, intent = _fixture(td)
        out = os.path.join(td, 'out_%d.kicad_pcb' % len(extra))
        r = _run([os.path.join(ROOT, 'py_placer', 'place_seed.py'), moved, out,
                  '--intent', intent, '--repair', '--clearance', '0.2',
                  '--board-edge-clearance', '0.5'] + list(extra))
        return r, out, base

    def test_without_a_baseline_it_reports_and_charges_nothing(self):
        with tempfile.TemporaryDirectory() as td:
            r, _out, _ = self._repair(td)
        self.assertIn('Courtyard census: 1 blocking pair(s), none charged',
                      r.stdout, r.stdout[-1500:])

    def test_with_a_baseline_armed_it_clears_the_pair(self):
        with tempfile.TemporaryDirectory() as td:
            base, *_ = _fixture(td)
            r, out, base = self._repair(td, '--baseline', base,
                                        '--body-model')
            self.assertIn('1 gating against base.kicad_pcb, charged',
                          r.stdout, r.stdout[-1500:])
            self.assertNotIn('UNRESOLVED', r.stdout)
            self.assertEqual(_gating(out, base), [])

    def test_with_a_baseline_unarmed_it_says_unresolved_and_why(self):
        with tempfile.TemporaryDirectory() as td:
            base, *_ = _fixture(td)
            r, out, base = self._repair(td, '--baseline', base)
        self.assertIn('1 gating against base.kicad_pcb, charged', r.stdout)
        self.assertIn('UNRESOLVED', r.stdout, r.stdout[-1500:])
        self.assertIn('--body-model seats on', r.stdout)

    def test_a_baseline_without_repair_is_refused(self):
        from run_utils import check
        with tempfile.TemporaryDirectory() as td:
            base, moved, intent = _fixture(td)
            check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_placer', 'place_seed.py'), moved,
                   os.path.join(td, 'o.kicad_pcb'), '--intent', intent,
                   '--baseline', base],
                  refuse='--baseline only applies to --repair', code=2)


class TheReseatRuler(unittest.TestCase):

    def test_the_gate_overlap_is_check_assemblys(self):
        """`reseat_scope`'s gate tuple ends in check_assembly's courtyard
        overlap at the state's poses: 1.0 mm2 on the fixture, where the
        search's pad boxes read 0."""
        from placement import floorplan, reconstruct, seeder
        import pose_score
        with tempfile.TemporaryDirectory() as td:
            _base, moved, intent = _fixture(td)
            pcb = parse_kicad_pcb(moved)
            st = pose_score.make_state(pcb, moved, clearance=0.2)
            self.assertAlmostEqual(reconstruct.measure(st)[-1], 0.0)
            census = legality.CourtyardCensus(pcb, moved)
            ov = reconstruct.measure(st, overlap=lambda s: census.grade(
                {r: (p.x, p.y, p.rot) for r, p in s.parts.items()}
            ).overlap_exact)[-1]
            self.assertAlmostEqual(ov, 1.0, places=3)
            # ...and reseat_scope measures with it: its gate_before says so.
            res = seeder.reseat_scope(pcb, moved,
                                      floorplan.load_intent(intent),
                                      refs=['B'], clearance=0.2,
                                      board_edge_clearance=0.5)
            self.assertAlmostEqual(res['gate_before'][-1], 1.0, places=3)


class AFixedPoseOnTheGradersGeometry(unittest.TestCase):

    def test_bodies_meeting_outside_their_pads_are_an_overlap(self):
        import pose_score
        from placement import seeder
        with tempfile.TemporaryDirectory() as td:
            _base, moved, _ = _fixture(td)
            st = pose_score.make_state(parse_kicad_pcb(moved), moved,
                                       clearance=0.2)
            area, _w, _h = seeder._courtyard_overlap(
                st, 'B', (13.5, 10.0, 0.0), 'A', (10.0, 10.0, 0.0))
        self.assertAlmostEqual(area, 1.0, places=3)


class ArmedOnlyMovesTheNeighbourCurrency(unittest.TestCase):

    def test_an_armed_pose_grader_grades_what_an_unarmed_one_does(self):
        """The floorplan grade reads the courtyard ladder (`_grade_ctx`
        builds a default state), so an armed search state must be graded on
        it too -- `_PosedState` reads `grade_rect`. It used to refuse."""
        import pose_score
        from placement import floorplan
        for name in ('esp_prog.kicad_pcb', 'watchy.kicad_pcb'):
            path = os.path.join(ROOT, 'kicad_files', name)
            pcb = parse_kicad_pcb(path)
            intent = floorplan.intent_from_dict(
                floorplan.emit_intent(pcb, path), path)
            blocks, _ = floorplan.resolve_blocks(intent, pcb,
                                                 ('kicad', 'sheet'))
            claims = {}
            for armed in (False, True):
                st = pose_score.make_state(pcb, path, clearance=0.2,
                                           body_model=armed)
                pg = floorplan.PoseGrader(intent, st, blocks=blocks,
                                          clearance=0.2,
                                          board_edge_clearance=0.55)
                claims[armed] = sorted(floorplan.violation_claim(v)
                                       for v in pg.violations())
            self.assertEqual(claims[True], claims[False], name)

    def test_armed_the_intent_rect_is_the_grade_rect(self):
        import pose_score
        path = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        off = pose_score.make_state(pcb, path, clearance=0.2)
        on = pose_score.make_state(pcb, path, clearance=0.2, body_model=True)
        grew = 0
        for ref, p_on in on.parts.items():
            p_off = off.parts[ref]
            self.assertEqual(p_on.grade_rect(), p_off.rect(), ref)
            grew += p_on.rect() != p_off.rect()
        self.assertGreater(grew, 0, 'armed changed no search rect')
        self.assertIs(off.parts[ref].grade_by_rot, off.parts[ref].bounds_by_rot)


class ArmedIntentAndBoardQuestionsKeepTheLadder(unittest.TestCase):
    """Armed, A's seat box is its 4 x 2 body; the floorplan grade still
    reads its pad box for the board term and keep-outs. So a pose the grade
    accepts -- body past the edge, body over a keep-out, pads clear of
    both -- must stay admitted, or the armed seeder refuses seats the grade
    accepts and the A/B measures the refusal, not the body model."""

    def _st(self, td, **kw):
        import pose_score
        _base, moved, _ = _fixture(td)
        return pose_score.make_state(parse_kicad_pcb(moved), moved,
                                     clearance=0.2, board_edge_clearance=0.0,
                                     body_model=True, **kw)

    def test_a_body_past_the_edge(self):
        from placement import seeder
        with tempfile.TemporaryDirectory() as td:
            st = self._st(td)
            self.assertGreater(st.parts['A'].rect(39.2, 10.0, 0.0)[2], 40.0)
            self.assertTrue(st.candidate_valid('A', 39.2, 10.0, 0.0,
                                               exclude={'B'}))
            self.assertTrue(seeder.pose_ok(st, 'A', 39.2, 10.0, 0.0,
                                           exclude={'B'}))

    def test_a_body_over_a_keepout(self):
        from placement import seeder
        k = [{'name': 'k', 'rect': [11.0, 9.5, 11.8, 10.5]}]
        with tempfile.TemporaryDirectory() as td:
            st = self._st(td, keepouts=k)
            self.assertTrue(st.candidate_valid('A', 10.0, 10.0, 0.0,
                                               exclude={'B'}))
            self.assertTrue(seeder.pose_ok(st, 'A', 10.0, 10.0, 0.0,
                                           exclude={'B'}))
            # The control: the pads on the keep-out ARE refused.
            self.assertFalse(seeder.pose_ok(st, 'A', 11.4, 10.0, 0.0,
                                            exclude={'B'}))

    def test_the_posed_view_reads_the_ladder(self):
        import pose_score
        from placement import floorplan
        path = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        on = pose_score.make_state(pcb, path, clearance=0.2, body_model=True)
        off = pose_score.make_state(pcb, path, clearance=0.2)
        got = {g.ref: g.rect for g in floorplan._PosedState(on).graded_parts()}
        want = {g.ref: g.rect for g in
                floorplan._PosedState(off).graded_parts()}
        self.assertEqual(got, want)


class OnlyTheMovedMemberIsCharged(unittest.TestCase):

    def test_the_part_at_its_baseline_pose_stays(self):
        """A is where the baseline has it; B moved onto it. The pair gates
        because of B, so B is the one the repair moves -- not A, which
        sorts first by name."""
        with tempfile.TemporaryDirectory() as td:
            base = _write(td, 'base', [_part('A', 10, 10),
                                       _part('B', 30, 10, net=3)])
            moved = _write(td, 'moved', [_part('A', 10, 10),
                                         _part('B', 13.5, 10, net=3)])
            intent = _intent(td, ['A', 'B'])
            out = os.path.join(td, 'o.kicad_pcb')
            r = _run([os.path.join(ROOT, 'py_placer', 'place_seed.py'),
                      moved, out, '--intent', intent, '--repair',
                      '--clearance', '0.2', '--board-edge-clearance', '0.5',
                      '--baseline', base, '--body-model'])
            self.assertTrue(os.path.isfile(out), r.stdout[-1500:])
            got = parse_kicad_pcb(out).footprints
            self.assertEqual((round(got['A'].x, 3), round(got['A'].y, 3)),
                             (10.0, 10.0), r.stdout[-1500:])
            # B moves (a turn in place clears it too: 2 x 4 beside A).
            self.assertNotEqual((round(got['B'].x, 3), round(got['B'].y, 3),
                                 round(got['B'].rotation or 0.0, 3) % 360),
                                (13.5, 10.0, 0.0))
            self.assertEqual(_gating(out, base), [])


class EveryPlaceSeedBuildIsHandedTheFlag(unittest.TestCase):
    """The structural half of the spy below: in place_seed.py every call that
    builds or runs a search -- make_state, quench, seed_from_intent,
    repair_placement, reseat_scope -- passes `body_model=args.body_model`.
    The spy can only see the builds a fixture happens to reach (the
    post-polish re-seat runs only when the polish leaves a zone error)."""

    def test_every_call_site(self):
        import ast
        path = os.path.join(ROOT, 'py_placer', 'place_seed.py')
        with open(path, encoding='utf-8') as fh:
            tree = ast.parse(fh.read())
        names = {'make_state', 'quench', 'seed_from_intent',
                 'repair_placement', 'reseat_scope'}
        seen, bad = [], []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            name = f.attr if isinstance(f, ast.Attribute) else getattr(
                f, 'id', None)
            if name not in names:
                continue
            seen.append(name)
            kw = {k.arg: k.value for k in node.keywords}
            v = kw.get('body_model')
            if not (isinstance(v, ast.Attribute) and v.attr == 'body_model'
                    and getattr(v.value, 'id', None) == 'args'):
                bad.append((name, node.lineno))
        self.assertEqual(sorted(set(seen)), sorted(names))
        self.assertEqual(bad, [])


class TheFlagReachesEveryBuild(unittest.TestCase):
    """`--body-model` arms every search state a place_seed run builds; only
    the floorplan grade's own state (`_grade_ctx`) stays on the courtyard
    ladder, by design."""

    def _states(self, argv):
        import inspect
        import place_seed
        from placement import quench
        seen = []
        real = quench.QuenchState.__init__

        def spy(self, *a, **k):
            callers = [f.function for f in inspect.stack()[1:8]]
            seen.append((bool(k.get('body_model', False)), callers))
            return real(self, *a, **k)
        quench.QuenchState.__init__ = spy
        old = sys.argv
        sys.argv = ['place_seed.py'] + argv
        try:
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                try:
                    place_seed.main()
                except SystemExit:
                    pass
        finally:
            sys.argv = old
            quench.QuenchState.__init__ = real
        return [(armed, c) for armed, c in seen
                if '_grade_ctx' not in c and 'grade_seat' not in c]

    def test_seed_and_polish(self):
        with tempfile.TemporaryDirectory() as td:
            _base, moved, intent = _fixture(td)
            got = self._states([moved, os.path.join(td, 's.kicad_pcb'),
                                '--intent', intent, '--force',
                                '--body-model'])
        self.assertTrue(got)
        self.assertEqual([c for armed, c in got if not armed], [])

    def test_repair_and_reseat(self):
        with tempfile.TemporaryDirectory() as td:
            base, moved, intent = _fixture(td)
            got = self._states([moved, os.path.join(td, 'r.kicad_pcb'),
                                '--intent', intent, '--repair',
                                '--reseat', 'B', '--baseline', base,
                                '--body-model'])
        names = {n for _a, c in got for n in c}
        self.assertIn('repair_placement', names)
        self.assertIn('reseat_scope', names)
        self.assertEqual([c for armed, c in got if not armed], [])


class TheWarning(unittest.TestCase):

    def test_armed_it_names_only_pad_box_parts(self):
        import pose_score
        path = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            st = pose_score.make_state(pcb, path, clearance=0.2,
                                       body_model=True)
        text = buf.getvalue()
        from placement.body import board_bodies
        src = {r: g.source for r, g in board_bodies(pcb, path).items()}
        drawn = sorted(r for r in st.parts if src.get(r) not in
                       (None, 'pad_bbox', 'courtyard'))
        self.assertTrue(drawn)
        self.assertIn(f'{len(drawn)} footprint(s) without a courtyard are '
                      f'spaced on their drawn body', text)
        warn = [line for line in text.splitlines() if 'WARNING [quench]' in line]
        for r in drawn:
            self.assertFalse(any(f' {r},' in w or w.endswith(f' {r}')
                                 for w in warn), (r, warn))


if __name__ == '__main__':
    unittest.main()
