"""check_reachability auto-widens NO-TARGET; net_forensics emits JSON.

Run 23, measured: U3.30 (/TXLED, partner D4.1 20.27mm away) returned exit 2
NO-TARGET at the default +/-4mm view; the manual retry ladder cost 128s at
--margin 12, 306s at 18, and a 10-minute timeout WITH NO DATA at 25, because
cells scale as (span/step)^2 at the fixed 0.01 step. The auto-widen locates
the nearest other island by vector union-find, widens the view to hold it,
and coarsens the step so the grid stays ~1200^2 -- one invocation whose cost
is bounded by the grid cap, not by a clock. And the island gap itself (the
number rip-set decisions need) was text-only in net_forensics; --json-out
makes it consumable.

The rest of this file is the review's list, each item pinned by a test that
fails without its fix: stdout stays a machine channel under --json, a coarsened
step is declared coarse instead of passed off as a measurement, the floors are
wrapped at the fab floor because this tool PREDICTS, and the auto-widen no
longer reaches into a sibling CLI for a private name.
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
PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')


def _run(tool, *argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', tool), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


def _load(path):
    with open(path, encoding='utf-8') as fh:
        return json.load(fh)


class TestAutoWiden(unittest.TestCase):
    def test_u3_30_answers_in_one_bounded_invocation(self):
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'r.json')
            r = _run('check_reachability.py', PLACED, '--pad', 'U3.30',
                     '--json-out', jp)
            # Exit 0 = PASSABLE, the true verdict run 23 needed three manual
            # retries to reach.
            self.assertEqual(r.returncode, 0, r.stdout[-500:])
            # STDERR. A notice on stdout is a JSONDecodeError at character 0
            # for --json, which the loop driver documents as a machine channel.
            self.assertIn('auto-widening', r.stderr)
            self.assertNotIn('auto-widening', r.stdout)
            self.assertIn('JSON ->', r.stderr)
            self.assertNotIn('JSON ->', r.stdout)
            doc = _load(jp)
            self.assertEqual(doc['verdict'], 'PASSABLE')
            aw = doc['auto_widened']
            self.assertAlmostEqual(aw['nearest_island_mm'], 20.27, delta=0.1)
            # The coarsened step is DISCLOSED (readings are only comparable
            # at their own step), and it is the step the field was built at,
            # not a rounded copy of it. The grid cap is what bounds the cost.
            self.assertGreater(aw['step_mm'], 0.01)
            self.assertEqual(aw['step_mm'], doc['step_mm'])
            self.assertLessEqual(max(doc['grid']), 1201)
            self.assertGreater(doc['margin_um'], 0)
            # A throat is a place on the board; a wide-open result has none,
            # and a result with no path has none. Never the last free cell the
            # search happened to touch.
            self.assertEqual(doc['throat'] is None,
                             doc['wide_open'] or doc['bottleneck_mm'] is None)
            if doc['wide_open']:
                self.assertEqual(doc['near'], [])
                self.assertIsNone(doc['gap_mm'])
                self.assertNotIn('throat     (', r.stdout)

            # A step this fine relative to the track is NOT coarse, and the
            # verdict says so rather than staying silent about it.
            self.assertFalse(aw['coarse'], aw)

    def test_explicit_view_is_never_overridden(self):
        r = _run('check_reachability.py', PLACED, '--pad', 'U3.30',
                 '--view', '55,58,60,63')
        self.assertNotIn('auto-widening', r.stdout)
        self.assertNotIn('auto-widening', r.stderr)

    def test_a_coarsened_step_is_declared_coarse(self):
        """The widened step grows with the island distance; nothing refused it.

        Measured A/B on this very pad at one view: the native 0.01 step gave
        bottleneck 0.4403, the widened 0.02021 gave 0.46904 -- 29 um
        OPTIMISTIC, more than one whole step, and optimistic is the direction
        that hides a cage. At --track 0.05 (pinned up to the 0.0762 fab floor)
        the step is more than a quarter of the track and the reading is a
        locate, not a measurement.
        """
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'r.json')
            r = _run('check_reachability.py', PLACED, '--pad', 'U3.30',
                     '--track', '0.05', '--json-out', jp)
            self.assertEqual(r.returncode, 0, r.stdout[-500:])
            aw = _load(jp)['auto_widened']
            self.assertTrue(aw['coarse'], aw)
            self.assertIn('coarse_note', aw)
            self.assertIn('COARSE', r.stderr)
            # ...and it reaches the reader of the document too, not only the
            # operator watching the terminal.
            self.assertIn('COARSE', _load(jp)['note'])
            self.assertIn('COARSE', r.stdout)      # via the `note` line

    def test_json_stdout_carries_nothing_but_json(self):
        """--json prints the document and nothing else; notices go to stderr."""
        r = _run('check_reachability.py', PLACED, '--pad', 'U3.30', '--json')
        self.assertEqual(r.returncode, 0, r.stderr[-500:])
        doc = json.loads(r.stdout)                 # char 0 must be `{`
        self.assertEqual(doc['verdict'], 'PASSABLE')
        self.assertIn('auto-widening', r.stderr)


class TestFabFloorWrap(unittest.TestCase):
    """A PREDICTIVE tool must not promise a lane no fab can etch.

    `list_nets.board_floor` is board-authoritative, not raise-only, and its
    docstring says a predictive consumer has to wrap at the fab floor the way
    `check_channels._fab` does. This tool predicts: its verdict is a claim that
    a route EXISTS. Two paths are covered here, and neither was exercised by
    any board in the repo -- every tracked board already sits above its floors.
    """
    # A tight explicit --view: the floors are resolved and reported before any
    # field is built, so the assertions do not need (or want) a whole widened
    # raster. Exit 2 is NO-TARGET in that box, which is the honest answer and
    # not what this test is about.
    VIEW = '184,32,188,36'

    def test_cli_values_below_the_floor_are_pinned(self):
        board = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
        r = _run('check_reachability.py', board, '--pad', 'U1.1',
                 '--clearance', '0.02', '--track', '0.02', '--view', self.VIEW)
        self.assertEqual(r.returncode, 2, r.stderr[-500:])
        # 2-layer JLC floors: clearance 0.1, track 0.1. Pinned regardless of
        # SOURCE -- a 0.02 typed on the command line is as unetchable as a
        # declared one, which is how enforce_fab_floors treats a flag too.
        self.assertIn('below the 0.1mm fab floor', r.stderr)
        self.assertIn('[cli]', r.stderr)
        self.assertIn('clearance 0.1mm, track 0.1mm', r.stdout)
        self.assertIn('clearance: fab floor', r.stdout)
        self.assertIn('track width: fab floor', r.stdout)

    def test_board_constraint_fallback_is_read_and_then_wrapped(self):
        """The .kicad_pro min_track_width / min_via_diameter fallback.

        New in this PR and inert on every board in the tree, so it is staged: a
        project declaring NO Default netclass, only the two Board Setup minima,
        both below the fab floor. The notice has to name `board constraint` as
        the source it read and then pin the value up.

        The staged project also carries a sub-floor NET CLASS, because the
        field is a min() over the classes: pinning the base while leaving one
        class at 0.02 would still measure that net's part of the field at a
        spacing no fab can etch.
        """
        with tempfile.TemporaryDirectory() as td:
            src = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
            dst = os.path.join(td, 'staged.kicad_pcb')
            with open(src, encoding='utf-8') as fh:
                _txt = fh.read()
            with open(dst, 'w', encoding='utf-8') as fh:
                fh.write(_txt)
            with open(os.path.join(td, 'staged.kicad_pro'), 'w',
                      encoding='utf-8') as fh:
                json.dump({'board': {'design_settings': {'rules': {
                    'min_track_width': 0.05, 'min_via_diameter': 0.2}}},
                    'net_settings': {
                        'classes': [{'name': 'Fine', 'clearance': 0.02}],
                        'netclass_assignments': {'GND': 'Fine'}}}, fh)
            r = _run('check_reachability.py', dst, '--pad', 'U1.1',
                     '--view', self.VIEW)
            self.assertEqual(r.returncode, 2, r.stderr[-500:])
            self.assertIn('track width 0.05mm [board constraint]', r.stderr)
            self.assertIn('via diameter 0.2mm [board constraint]', r.stderr)
            self.assertIn('track 0.1mm', r.stdout)
            self.assertIn('via 0.25mm', r.stdout)
            self.assertIn('net-class clearance(s) below the 0.1mm fab floor',
                          r.stderr)
            # The report carries both numbers, not just the stderr notice: a
            # reader of the text cannot otherwise tell a wrapped floor from a
            # declared one that happens to equal the fab minimum.
            self.assertIn('0.05 declared [board constraint] -> 0.1', r.stdout)

    def test_the_per_net_map_is_wrapped_in_EFFECT_not_just_in_the_notice(self):
        """Assert the consequence, because the message is not the fix.

        Deleting the per-net wrap and keeping its stderr line passed the
        message-matching test. `via_legal_fraction` is computed from the same
        field the verdict is, and it moves with the per-net clearances, so a
        sub-floor class that was NOT wrapped shows up here.

        Three staged projects, one pad, one explicit view: a `*` class at 0.02
        (below tigard's 4-layer 0.09 fab clearance) must measure IDENTICALLY
        to one declared at exactly 0.09, and both must differ from one at 0.30
        so the equality cannot pass by measuring nothing. Measured: 0.3298 /
        0.3298 / 0.2261, and 0.3707 for the 0.02 class with the wrap deleted.
        """
        with tempfile.TemporaryDirectory() as td:
            dst = os.path.join(td, 's.kicad_pcb')
            with open(PLACED, encoding='utf-8') as fh:
                _txt = fh.read()
            with open(dst, 'w', encoding='utf-8') as fh:
                fh.write(_txt)
            frac = {}
            for clr in (0.02, 0.09, 0.30):
                with open(os.path.join(td, 's.kicad_pro'), 'w',
                          encoding='utf-8') as fh:
                    json.dump({'net_settings': {
                        'classes': [{'name': 'Fine', 'clearance': clr}],
                        'netclass_patterns': [{'pattern': '*',
                                               'netclass': 'Fine'}]}}, fh)
                jp = os.path.join(td, f'o{clr}.json')
                r = _run('check_reachability.py', dst, '--pad', 'U3.30',
                         '--view', '55,58,60,63', '--json-out', jp)
                self.assertIn(r.returncode, (0, 1, 2), r.stderr[-400:])
                frac[clr] = _load(jp)['via_legal_fraction']
            self.assertEqual(
                frac[0.02], frac[0.09],
                f'a 0.02 net class measured {frac[0.02]} where the 0.09 fab '
                f'floor measures {frac[0.09]}: the per-net map was not wrapped')
            self.assertNotEqual(
                frac[0.09], frac[0.30],
                'the fixture measures nothing that depends on the per-net '
                'clearances, so the equality above proves nothing')


class TestFabTierIsTellable(unittest.TestCase):
    """The floor this tool ENFORCES has to be one the operator can set.

    `--fab-tier` / `--fab-overrides` come from `fab_tiers.add_fab_tier_args`,
    the same helper every routing and DRC CLI uses, so the floor measured
    against here is the floor the chain routed to. Without them a board built
    with the documented override escape hatch is measured CAGED at a floor its
    fab beats, and CAGED is the loop's most expensive verdict.
    """
    BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    VIEW = '184,32,188,36'

    def test_an_override_moves_the_wrapped_floor(self):
        with tempfile.TemporaryDirectory() as td:
            ov = os.path.join(td, 'fab.txt')
            with open(ov, 'w', encoding='utf-8') as fh:
                fh.write('clearance = 0.05\ntrack_width = 0.05\n')
            args = (self.BOARD, '--pad', 'U1.1', '--clearance', '0.06',
                    '--track', '0.06', '--view', self.VIEW)
            plain = _run('check_reachability.py', *args)
            self.assertIn('below the 0.1mm fab floor', plain.stderr)
            self.assertIn('clearance 0.1mm, track 0.1mm', plain.stdout)
            over = _run('check_reachability.py', *args, '--fab-overrides', ov)
            self.assertNotIn('fab floor', over.stderr)
            self.assertIn('clearance 0.06mm, track 0.06mm', over.stdout)
            self.assertIn('clearance: cli', over.stdout)

    def test_a_missing_override_file_is_exit_2_not_the_CAGED_verdict(self):
        r = _run('check_reachability.py', self.BOARD, '--pad', 'U1.1',
                 '--fab-overrides', os.path.join(ROOT, 'no-such-fab.txt'))
        self.assertEqual(r.returncode, 2, r.stderr[-300:])
        self.assertIn('cannot resolve', r.stderr)


class TestNoSiblingCliImport(unittest.TestCase):
    """The auto-widen must not import `net_forensics._components`.

    A sibling CLI's private name, from a directory `_path` does not put on
    sys.path -- so the ImportError escaped as exit **1**, which in this tool is
    the CAGED geometry verdict and re-enters placement, discarding every routed
    board. The walk lives in `list_nets.net_islands` now.
    """
    def test_auto_widen_runs_with_net_forensics_unimportable(self):
        import types
        for _p in (os.path.join(ROOT, 'py_tools'),
                   os.path.join(ROOT, 'py_router'),
                   os.path.join(ROOT, 'py_placer')):
            if _p not in sys.path:
                sys.path.insert(0, _p)
        try:
            import scipy  # noqa: F401
        except ImportError:
            self.skipTest('reachability needs scipy')
        import check_reachability
        saved = sys.modules.get('net_forensics')
        # A module object with no `_components`: `from net_forensics import
        # _components` raises ImportError against it, exactly as it would if
        # py_tools were not on the path.
        sys.modules['net_forensics'] = types.ModuleType('net_forensics')
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                rc = check_reachability.main([PLACED, '--pad', 'U3.30'])
        finally:
            if saved is None:
                sys.modules.pop('net_forensics', None)
            else:
                sys.modules['net_forensics'] = saved
        self.assertEqual(rc, 0, buf.getvalue()[-400:])
        self.assertIn('PASSABLE', buf.getvalue())

    def test_help_does_not_promise_a_banner_it_never_prints(self):
        r = _run('check_reachability.py', '--help')
        self.assertEqual(r.returncode, 0, r.stderr[-300:])
        self.assertNotIn('CMD:', r.stdout)
        self.assertNotIn('banner', r.stdout)


class TestWideOpenHasNoThroat(unittest.TestCase):
    """A wide-open result carries no throat, in the JSON or the text.

    The first version returned the last cell Kruskal activated as the throat
    even when the path never neared copper, so this pad printed "There is no
    throat here to measure" and then "throat (...) gap 34.18mm vs 0.55mm
    needed" two lines later, and --json-out carried throat/near/gap_mm for a
    PASSABLE verdict.
    """
    BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

    def test_splitflap_u1_1(self):
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'r.json')
            dp = os.path.join(td, 'd.json')
            r = _run('check_reachability.py', self.BOARD, '--pad', 'U1.1',
                     '--json-out', jp, '--defect-json', dp)
            self.assertEqual(r.returncode, 0, r.stdout[-500:])
            doc = _load(jp)
            self.assertEqual(doc['verdict'], 'PASSABLE')
            self.assertTrue(doc['wide_open'], doc)
            self.assertIsNone(doc['throat'])
            self.assertEqual(doc['near'], [])
            self.assertIsNone(doc['gap_mm'])
            self.assertIn('There is no throat here to measure', r.stdout)
            self.assertNotIn('throat     (', r.stdout)
            self.assertNotIn('look at it', r.stdout)
            self.assertFalse(os.path.exists(dp),
                             'a record describes a defect; PASSABLE has none')


def _islands_all_pairs(pcb, net_id, tol=0.05):
    """`net_islands` written the obvious way: compare every point to every other.

    The REFERENCE, kept deliberately dumb. `net_islands` buckets its pair scan
    on a uniform grid, which is a separate optimisation riding in a shared
    py_router module, and the review found that shrinking its neighbourhood
    from 3x3 to the point's own cell passed every test in the tree. Nothing
    pinned the bucketing to the thing it is meant to be equivalent to; this
    does.
    """
    import math as _m
    from collections import defaultdict as _dd
    items = []
    for s in pcb.segments:
        if s.net_id == net_id:
            items.append(('seg', s, [(s.start_x, s.start_y), (s.end_x, s.end_y)]))
    for v in pcb.vias:
        if v.net_id == net_id:
            items.append(('via', v, [(v.x, v.y)]))
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.net_id == net_id:
                items.append(('pad', p, [(p.global_x, p.global_y)]))
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    pts = []
    for idx, (_k, obj, ps) in enumerate(items):
        r = (max(obj.size_x, obj.size_y) / 2 if hasattr(obj, 'size_x')
             else getattr(obj, 'size', 0) / 2 or 0.06)
        for (x, y) in ps:
            pts.append((x, y, idx, r))
    for i in range(len(pts)):
        x1, y1, a, r1 = pts[i]
        for j in range(i + 1, len(pts)):
            x2, y2, b, r2 = pts[j]
            if a != b and _m.hypot(x1 - x2, y1 - y2) <= max(tol, r1, r2):
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[rb] = ra
    comps = _dd(list)
    for idx in range(len(items)):
        comps[find(idx)].append(items[idx])
    return sorted(comps.values(), key=len, reverse=True)


class TestNetIslandsMatchesAllPairs(unittest.TestCase):
    """`net_islands`' bucketed pair scan must equal the all-pairs walk.

    Every case here is a boundary the 3x3 neighbourhood exists to cover, so
    each one fails if the neighbourhood shrinks. `cell` is the LARGEST join
    radius on the net, which is why a single big pad among small ones is its
    own case: it makes the cell wide and every small pair land in one bucket,
    and it is also the case where a partner sits two cells away.
    """
    def setUp(self):
        for _p in (ROOT, os.path.join(ROOT, 'py_router'),
                   os.path.join(ROOT, 'tests')):
            if _p not in sys.path:
                sys.path.insert(0, _p)

    @staticmethod
    def _key(comps):
        return sorted(tuple(sorted(o.pad_number for _k, o, _p in c))
                      for c in comps)

    def _pcb(self, pads):
        import synth
        from kicad_parser import Footprint
        fps = {}
        for i, (x, y, sx, sy) in enumerate(pads):
            num = f'p{i}'
            pad = synth.make_pad(1, x, y, ref='U1', num=num, net_name='N',
                                 size_x=sx, size_y=sy)
            fps[f'F{i}'] = Footprint(reference=f'F{i}', footprint_name='t:x',
                                     x=x, y=y, rotation=0.0, layer='F.Cu',
                                     pads=[pad])
        return synth.make_pcb(nets={1: synth.make_net(1, 'N')},
                              footprints=fps)

    # (label, tol, [(x, y, size_x, size_y), ...])
    CASES = [
        # Exactly at the join radius, straddling a cell boundary at x=0.05.
        ('exact radius across a cell boundary', 0.05,
         [(0.03, 0.0, 0.0, 0.0), (0.08, 0.0, 0.0, 0.0)]),
        # Both points sitting ON the boundary line.
        ('both on the boundary', 0.05,
         [(0.05, 0.05, 0.0, 0.0), (0.05, 0.09, 0.0, 0.0)]),
        # Diagonal neighbours: the case a 4-neighbourhood would miss.
        ('diagonal across a cell corner', 0.05,
         [(0.049, 0.049, 0.0, 0.0), (0.051, 0.051, 0.0, 0.0)]),
        # Two items at the very same coordinate.
        ('coincident duplicates', 0.05,
         [(1.0, 1.0, 0.2, 0.2), (1.0, 1.0, 0.2, 0.2), (1.0, 1.0, 0.2, 0.2)]),
        ('a single item', 0.05, [(3.0, 3.0, 0.5, 0.5)]),
        # Zero-size pads exactly at tol and just past it.
        ('zero-size pads at tol', 0.05,
         [(0.0, 0.0, 0.0, 0.0), (0.05, 0.0, 0.0, 0.0)]),
        ('zero-size pads just past tol', 0.05,
         [(0.0, 0.0, 0.0, 0.0), (0.050001, 0.0, 0.0, 0.0)]),
        ('negative coordinates', 0.05,
         [(-1.02, -1.0, 0.0, 0.0), (-0.98, -1.0, 0.0, 0.0),
          (-1.0, -3.0, 0.0, 0.0)]),
        # One big pad sets `cell`; its partners are 2 and 3 cells away.
        ('one big pad spanning several cells', 0.05,
         [(0.0, 0.0, 6.0, 6.0), (2.5, 0.0, 0.1, 0.1), (5.5, 0.0, 0.1, 0.1),
          (9.0, 0.0, 0.1, 0.1)]),
        ('mixed radii, chain of three', 0.05,
         [(0.0, 0.0, 1.0, 1.0), (0.45, 0.0, 0.1, 0.1), (0.9, 0.0, 2.0, 0.2)]),
        # tol=0 with every radius 0: `cell` is 0 and the bucket index divides
        # by it. This used to raise ZeroDivisionError inside a shared module.
        ('tol 0 and every radius 0', 0.0,
         [(1.0, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0)]),
    ]

    def test_adversarial_points(self):
        import list_nets
        for label, tol, pads in self.CASES:
            with self.subTest(label):
                pcb = self._pcb(pads)
                got = list_nets.net_islands(pcb, 1, tol=tol)
                want = _islands_all_pairs(pcb, 1, tol=tol)
                self.assertEqual(self._key(got), self._key(want), label)

    def test_the_two_tracked_boards_every_net(self):
        import list_nets
        from kicad_parser import parse_kicad_pcb
        for board in (PLACED, os.path.join(ROOT, 'kicad_files',
                                           'splitflap_driver.kicad_pcb')):
            pcb = parse_kicad_pcb(board)
            for nid in pcb.nets:
                got = list_nets.net_islands(pcb, nid)
                want = _islands_all_pairs(pcb, nid)
                self.assertEqual(self._key(got), self._key(want),
                                 f'{board} net {nid}')


class TestForensicsJson(unittest.TestCase):
    def test_gap_is_machine_readable(self):
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'nf.json')
            # --json-out, the name every other tool in the repo uses for
            # "write the document to this path".
            r = _run('net_forensics.py', PLACED, '--nets', '/TXLED',
                     '--json-out', jp)
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            doc = _load(jp)
            net = doc['nets'][0]
            self.assertEqual(net['net'], '/TXLED')
            self.assertEqual(len(net['islands']), 2)
            self.assertAlmostEqual(net['gap']['mm'], 20.267, delta=0.05)

    def test_the_island_walk_is_paid_exactly_once_per_net(self):
        """`net_data` repeated the walk `_describe` had just done.

        It was called unconditionally, outside any --json guard, so every
        text-only run -- which is nearly all of them -- paid for the island
        walk twice and threw the second one away. Guarding it fixed the
        text-only run; passing `_describe`'s components in fixes the
        --json-out run, which still walked twice. Counted at `net_islands`,
        the walk itself, so neither half can regress unseen.
        """
        for _p in (os.path.join(ROOT, 'py_tools'),
                   os.path.join(ROOT, 'py_router')):
            if _p not in sys.path:
                sys.path.insert(0, _p)
        import net_forensics
        import list_nets
        walks, data_calls = [], []
        real_walk = list_nets.net_islands
        real_data = net_forensics.net_data

        def counted_walk(*a, **k):
            walks.append(1)
            return real_walk(*a, **k)

        def counted_data(*a, **k):
            data_calls.append(1)
            return real_data(*a, **k)

        # net_forensics imported the name, so both bindings have to move.
        net_forensics.net_islands = counted_walk
        list_nets.net_islands = counted_walk
        net_forensics.net_data = counted_data
        argv = sys.argv
        try:
            with tempfile.TemporaryDirectory() as td:
                buf = io.StringIO()
                sys.argv = ['net_forensics.py', PLACED, '--nets', '/TXLED']
                with contextlib.redirect_stdout(buf):
                    net_forensics.main()
                self.assertEqual(data_calls, [],
                                 'a text-only run built the JSON document')
                self.assertEqual(len(walks), 1,
                                 f'a text-only run walked {len(walks)} times '
                                 f'for one net')
                walks.clear()
                jp = os.path.join(td, 'nf.json')
                sys.argv = ['net_forensics.py', PLACED, '--nets', '/TXLED',
                            '--json-out', jp]
                with contextlib.redirect_stdout(buf):
                    net_forensics.main()
                self.assertEqual(len(data_calls), 1)
                self.assertEqual(len(walks), 1,
                                 f'a --json-out run walked {len(walks)} times '
                                 f'for one net; `_describe` returns the '
                                 f'components so `net_data` need not re-walk')
                self.assertTrue(os.path.exists(jp))
                doc = _load(jp)
                # ...and reusing them changed no number.
                self.assertAlmostEqual(doc['nets'][0]['gap']['mm'], 20.267,
                                       delta=0.05)
                self.assertEqual(len(doc['nets'][0]['islands']), 2)
        finally:
            net_forensics.net_islands = real_walk
            list_nets.net_islands = real_walk
            net_forensics.net_data = real_data
            sys.argv = argv

    def test_a_single_island_net_and_a_missing_net_still_serialise(self):
        """The two early-return paths through `_describe`, which now RETURNS."""
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'nf.json')
            r = _run('net_forensics.py', PLACED, '--nets', 'GND', 'NOPE',
                     '--json-out', jp)
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            nets = {n['net']: n for n in _load(jp)['nets']}
            self.assertEqual(nets['NOPE'].get('error'), 'not found')
            self.assertGreater(len(nets['GND']['islands']), 1)


if __name__ == '__main__':
    unittest.main(verbosity=1)
