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
        """Armed, on the body the search seats; unarmed, on the search's
        own rects (pad boxes here), as before #1182."""
        import pose_score
        from placement import seeder
        got = {}
        with tempfile.TemporaryDirectory() as td:
            _base, moved, _ = _fixture(td)
            for armed in (True, False):
                st = pose_score.make_state(parse_kicad_pcb(moved), moved,
                                           clearance=0.2, body_model=armed)
                got[armed] = seeder._courtyard_overlap(
                    st, 'B', (13.5, 10.0, 0.0), 'A', (10.0, 10.0, 0.0))[0]
        self.assertAlmostEqual(got[True], 1.0, places=3)
        self.assertEqual(got[False], 0.0)


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


ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
#: The rect / grade_rect read inventory (`TheRectInventory`).
RECT_INVENTORY = os.path.join(ROOT, 'tests', 'fixtures',
                              '1182_rect_inventory.json')
#: Where the inventory is taken: every placement module that reads a
#: search part's rects.
INVENTORY_FILES = ('quench.py', 'seeder.py', 'floorplan.py', 'reseat.py',
                   'arrays.py', 'reconstruct.py')
INVENTORY_NAMES = ('rect', 'rects', 'grade_rect', 'grade_rects',
                   'bounds_by_rot', 'grade_by_rot')


def _summary(stdout):
    line = next(ln for ln in stdout.splitlines()
                if ln.startswith('JSON_SUMMARY:'))
    return json.loads(line[len('JSON_SUMMARY:'):])


def rect_inventory():
    """`{'file::Qual.name': {name: count}}`: every call of `.rect(`,
    `.rects(`, `.grade_rect(`, `.grade_rects(` and every read of
    `bounds_by_rot` / `grade_by_rot`, per function."""
    import ast
    out = {}
    pl = os.path.join(ROOT, 'py_placer', 'placement')
    for name in INVENTORY_FILES:
        with open(os.path.join(pl, name), encoding='utf-8') as fh:
            tree = ast.parse(fh.read())

        def walk(node, qual):
            for ch in ast.iter_child_nodes(node):
                if isinstance(ch, (ast.FunctionDef, ast.AsyncFunctionDef,
                                   ast.ClassDef)):
                    walk(ch, qual + [ch.name])
                    continue
                _count(ch, qual)
                walk(ch, qual)

        def _count(node, qual):
            hit = None
            if isinstance(node, ast.Call) and isinstance(
                    node.func, ast.Attribute) and node.func.attr in (
                    'rect', 'rects', 'grade_rect', 'grade_rects'):
                hit = node.func.attr
            elif isinstance(node, ast.Attribute) and node.attr in (
                    'bounds_by_rot', 'grade_by_rot'):
                hit = node.attr
            if hit:
                key = name + '::' + ('.'.join(qual) or '<module>')
                d = out.setdefault(key, {})
                d[hit] = d.get(hit, 0) + 1
        walk(tree, [])
    return {k: dict(sorted(v.items())) for k, v in sorted(out.items())}


class AnUnarmedFixedPoseKeepsItsScreen(unittest.TestCase):
    """The phase-4 verifier's regression: on the default (unarmed) path a
    declared pose is screened on the search's own rects, as before #1182 --
    esp_prog's human poses for R1, U2 and CON2 seat, as they did."""

    def test_esp_progs_human_poses_are_not_refused(self):
        import pose_score
        from placement import seeder
        pcb = parse_kicad_pcb(ESP)
        st = pose_score.make_state(pcb, ESP, clearance=0.2)
        poses = {r: (pcb.footprints[r].x, pcb.footprints[r].y,
                     pcb.footprints[r].rotation or 0.0)
                 for r in ('R1', 'U2', 'CON2')}
        for r, pose in poses.items():
            others = {o: p for o, p in poses.items() if o != r}
            _how, _reasons, conflicts = seeder._fixed_pose_check(
                st, r, pose, others)
            self.assertEqual(conflicts, {}, (r, _reasons))


class TheRepairHelpers(unittest.TestCase):
    """`courtyard_charges` and `courtyard_regrade_notes`, the repair's two
    courtyard halves, alone."""

    def _pair(self, a, b, depth=0.5, area=1.0):
        from types import SimpleNamespace as NS
        return NS(a=a, b=b, depth_mm=depth, area_mm2=area)

    def _state(self, locked=()):
        from types import SimpleNamespace as NS
        return NS(parts={r: NS(locked=r in locked) for r in 'ABC'})

    def test_the_charge(self):
        from placement import seeder
        key = lambda r: r                                   # noqa: E731
        st = self._state()
        # both moved: the first charged, the other its partner
        [(_q, order, w)] = seeder.courtyard_charges(
            [self._pair('A', 'B', depth=0.5)], st, {'A', 'B'}, key)
        self.assertEqual((order, w), (['A', 'B'], 1.0))
        # only B moved: B alone
        [(_q, order, w)] = seeder.courtyard_charges(
            [self._pair('A', 'B', depth=2.5)], st, {'B'}, key)
        self.assertEqual((order, w), (['B'], 2.5))
        # a locked member is never charged; both locked: nobody
        [(_q, order, _w)] = seeder.courtyard_charges(
            [self._pair('A', 'B')], self._state(locked='A'), {'A', 'B'},
            key)
        self.assertEqual(order, ['B'])
        [(_q, order, _w)] = seeder.courtyard_charges(
            [self._pair('A', 'B')], self._state(locked='AB'), {'A'}, key)
        self.assertEqual(order, [])

    def test_the_regrade(self):
        from placement import seeder
        ab, bc = self._pair('A', 'B'), self._pair('B', 'C')
        got = seeder.courtyard_regrade_notes(
            [ab], [ab, bc], {'A': {frozenset('AB')}}, {'C'}, False)
        self.assertEqual([r for r, _w in got], ['A', 'C'])
        self.assertIn('still gates', got[0][1])
        self.assertIn('created a gating courtyard pair with B', got[1][1])
        self.assertIn('--body-model', got[0][1])
        armed = seeder.courtyard_regrade_notes(
            [ab], [ab, bc], {'A': {frozenset('AB')}}, {'C'}, True)
        self.assertFalse(any('--body-model' in w for _r, w in armed))


class TheRepairRecords(unittest.TestCase):
    """JSON_SUMMARY carries the gate's before/after and the claim."""

    def test_unarmed_the_claim_and_the_counts_are_written(self):
        with tempfile.TemporaryDirectory() as td:
            base, moved, intent = _fixture(td)
            r = _run([os.path.join(ROOT, 'py_placer', 'place_seed.py'),
                      moved, os.path.join(td, 'o.kicad_pcb'), '--intent',
                      intent, '--repair', '--clearance', '0.2',
                      '--board-edge-clearance', '0.5', '--baseline', base])
        s = _summary(r.stdout)
        self.assertEqual(s['courtyard_gating_before'], 1, s)
        self.assertEqual(s['courtyard_gating_after'], 1, s)
        self.assertEqual(s['unresolved_by_rule'].get('courtyard_blocking'),
                         1, s)

    def test_a_missing_baseline_is_refused(self):
        from run_utils import check
        with tempfile.TemporaryDirectory() as td:
            _base, moved, intent = _fixture(td)
            check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_placer', 'place_seed.py'), moved,
                   os.path.join(td, 'o.kicad_pcb'), '--intent', intent,
                   '--repair', '--baseline', os.path.join(td, 'no.kicad_pcb')],
                  refuse='no such board file', code=2)


class TheReseatRulers(unittest.TestCase):

    def test_every_measure_in_reseat_scope_is_on_the_graders_ruler(self):
        """`reseat_scope` measures its gate five times (the empty gate,
        before, the prune, after, the refusal revert); each must pass
        check_assembly's overlap, or the gate compares two rulers."""
        import ast
        with open(os.path.join(ROOT, 'py_placer', 'placement', 'seeder.py'),
                  encoding='utf-8') as fh:
            tree = ast.parse(fh.read())
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                  and n.name == 'reseat_scope')
        calls = [c for c in ast.walk(fn) if isinstance(c, ast.Call)
                 and isinstance(c.func, ast.Attribute)
                 and c.func.attr in ('measure', 'prune_assignment')]
        self.assertGreaterEqual(len(calls), 5)
        for c in calls:
            kw = {k.arg: k.value for k in c.keywords}
            self.assertIn('overlap', kw, ast.dump(c)[:200])
            self.assertEqual(getattr(kw['overlap'], 'id', None),
                             '_cy_overlap', ast.dump(c)[:200])


class TheGeometryHelpers(unittest.TestCase):

    def test_the_occupancy_rect_turns_with_the_part(self):
        pcb = parse_kicad_pcb(ESP)
        lbs, _b = legality._part_local_bounds_and_bodies(pcb, ESP)
        ref = next(r for r, lb in sorted(lbs.items())
                   if abs((lb.local[2] - lb.local[0])
                          - (lb.local[3] - lb.local[1])) > 1.0)
        lb = lbs[ref]
        at0 = legality.occupancy_rect_at(pcb, ref, (10.0, 20.0, 0.0))
        at90 = legality.occupancy_rect_at(pcb, ref, (10.0, 20.0, 90.0))
        self.assertEqual(at0, (10.0 + lb.local[0], 20.0 + lb.local[1],
                               10.0 + lb.local[2], 20.0 + lb.local[3]))
        r = legality.rotate_local_bounds(*lb.local, 90.0)
        self.assertEqual(at90, (10.0 + r[0], 20.0 + r[1], 10.0 + r[2],
                                20.0 + r[3]))
        self.assertNotEqual(at0, at90)

    def test_moved_refs_at_reads_the_poses(self):
        pcb = parse_kicad_pcb(ESP)
        self.assertEqual(legality.moved_refs_at(pcb, pcb), set())
        fp = pcb.footprints['R1']
        self.assertEqual(legality.moved_refs_at(
            pcb, pcb, {'R1': (fp.x + 1.0, fp.y, fp.rotation or 0.0)}),
            {'R1'})


class ArmedAndUnarmedAskTheSameGradeQuestions(unittest.TestCase):
    """With every other part excluded the neighbour currency vanishes, so
    an armed and an unarmed state must agree on every question that is not
    a neighbour's: the seat predicate, `candidate_valid`, the board term,
    a declared pose with no obstacles, the zone gate. The phase-4 verifier
    ran this over 12,348 samples; here esp_prog's grown parts, sampled."""

    def test_esp_prog(self):
        import contextlib
        import pose_score
        from placement import floorplan, seeder
        pcb = parse_kicad_pcb(ESP)
        with contextlib.redirect_stdout(io.StringIO()):
            intent = floorplan.intent_from_dict(
                floorplan.emit_intent(pcb, ESP), ESP)
            blocks, _ = floorplan.resolve_blocks(intent, pcb,
                                                 ('kicad', 'sheet'))
            gate, _ = floorplan.resolve_intent_gate(intent, pcb,
                                                    ('kicad', 'sheet'))
            ze = floorplan.zone_entries(intent, blocks)
            st = {armed: pose_score.make_state(
                pcb, ESP, clearance=0.2, board_edge_clearance=0.3,
                keepouts=intent.keepouts, intent_zones=gate.get('zones'),
                exclusive_zones=ze, body_model=armed)
                for armed in (False, True)}
        zone_of = {}
        for z in intent.blocks:
            if z.rect is not None:
                for r in blocks.get(z.name, ()):
                    zone_of.setdefault(r, z)
        s0, s1 = st[False], st[True]
        grew = [r for r in sorted(s0.parts) if not s0.parts[r].locked
                and s1.parts[r].rect() != s0.parts[r].rect()]
        self.assertGreater(len(grew), 3)
        everyone = set(s0.parts)
        offs = [(dx * 1.5, dy * 1.5) for dx in range(-2, 3)
                for dy in range(-2, 3)]
        bad, n = [], 0
        for ref in grew:
            p0, p1 = s0.parts[ref], s1.parts[ref]
            ex = everyone - {ref}
            for rot in (p0.rot, (p0.rot + 90) % 360):
                for dx, dy in offs:
                    x, y = p0.x + dx, p0.y + dy
                    n += 1
                    with contextlib.redirect_stdout(io.StringIO()):
                        got = {
                            'pose_ok': [seeder.pose_ok(s, ref, x, y, rot, ex)
                                        for s in (s0, s1)],
                            'candidate_valid': [s.candidate_valid(
                                ref, x, y, rot, exclude=ex) for s in (s0, s1)],
                            'board': [round(s.violation_parts(
                                ref, x, y, rot, exclude=ex)[0], 6)
                                for s in (s0, s1)],
                            'fixed': [seeder._fixed_pose_check(
                                s, ref, (x, y, rot), {})[0]
                                for s in (s0, s1)]}
                        z = zone_of.get(ref)
                        if z is not None:
                            tol = intent.zone_tolerance(z)
                            got['zone'] = [seeder.zone_gate(
                                p, z.rect, tol)[0](x, y, rot)
                                for p in (p0, p1)]
                    for k, (a, b) in got.items():
                        if a != b:
                            bad.append((ref, x, y, rot, k, a, b))
        self.assertEqual(bad[:10], [], f'{len(bad)} of {n} samples')


class TheRectInventory(unittest.TestCase):
    """Every read of a search part's rect, by function. Under `body_model`
    `rect` is the NEIGHBOUR currency (occupancy) and `grade_rect` the GRADE
    ladder (courtyard, else pad box); a site that asks a zone, keep-out,
    edge or board question of `rect` refuses seats the grade accepts, and
    only an armed run on the right board shows it -- the phase-4 verifier's
    mutants swapped 25 of them unseen. A changed count fails here: decide
    which question the new read asks, then re-pin with
    `python3 tests/test_1182_body_model_movers.py --write-rect-inventory`."""

    def test_the_inventory_is_pinned(self):
        with open(RECT_INVENTORY, encoding='utf-8') as fh:
            want = json.load(fh)
        got = rect_inventory()
        diff = {k: (want.get(k), got.get(k)) for k in sorted(set(want)
                                                            | set(got))
                if want.get(k) != got.get(k)}
        self.assertEqual(diff, {}, 'rect reads moved -- see the docstring')


if __name__ == '__main__':
    if '--write-rect-inventory' in sys.argv:
        os.makedirs(os.path.dirname(RECT_INVENTORY), exist_ok=True)
        with open(RECT_INVENTORY, 'w', encoding='utf-8') as fh:
            json.dump(rect_inventory(), fh, indent=1, sort_keys=True)
            fh.write('\n')
        print('wrote', RECT_INVENTORY)
        sys.exit(0)
    unittest.main()
