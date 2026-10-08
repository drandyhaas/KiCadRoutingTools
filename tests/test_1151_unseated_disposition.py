"""#1151: a part the seed cannot seat is not written on top of a neighbour.

An unseated part kept the pose it came in with, while every later seat
excluded it as part of "the pile" -- so its neighbours were packed onto copper
that was then written exactly there. Measured: all six of StickHub's OFF-seed
`body_blocking` pairs involve J6, and on an rp2350 pile seeded under the
pre-#1213 gate all 17 blocking pad pairs involve U6.

`seeder._dispose_unseated` decides, AFTER the search, where each such part is
written: left where it came in when that pose is legal against what was
seated (or it is locked, or already off the board), otherwise STAGED on a
deterministic row below the board. What must hold:

  * the four dispositions each happen for their own reason (unit arms);
  * a staged part lands below the board and below every part, and two
    staged parts do not overlap;
  * on the rp2350 pile under the old gate, U6 is staged and no blocking pair
    involves it -- and with the disposition switched off, the pairs come back
    (the arm that shows this test can fail);
  * every SEATED pose is bit-identical between the two arms: the fix decides
    nothing the search decided.
"""

import os
import random
import sys
import tempfile
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import legality, seeder                       # noqa: E402

TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
RP2350 = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')


def _full(rects_a, ea, rects_b, reach):
    return list(enumerate(rects_a)), list(enumerate(rects_b))


class Dispositions(unittest.TestCase):
    """`_dispose_unseated` on the designer's tigard, one reason at a time."""

    def setUp(self):
        import pose_score
        self.pcb = parse_kicad_pcb(TIGARD)
        self.st = pose_score.make_state(self.pcb, TIGARD, clearance=0.2)
        self.bb = self.pcb.board_info.board_bounds

    def _movable_legal(self):
        for r in sorted(self.st.parts):
            p = self.st.parts[r]
            if not p.locked and seeder._input_pose_conflict(
                    self.st, r, (p.seed_x, p.seed_y, p.orig_rot),
                    set()) is None:
                return r
        self.fail('no movable part is clear at its own designer pose')

    def test_clear_at_input_is_left(self):
        r = self._movable_legal()
        d = seeder._dispose_unseated(self.st, [r])[r]
        self.assertEqual(d['disposition'], 'clear_at_input')
        self.assertEqual(d['written'], d['input'])

    def test_locked_at_input_is_left(self):
        r = next(r for r in sorted(self.st.parts)
                 if not self.st.parts[r].locked)
        self.st.parts[r].locked = True
        d = seeder._dispose_unseated(self.st, [r])[r]
        self.assertEqual(d['disposition'], 'locked_at_input')

    def test_off_board_is_left(self):
        r = self._movable_legal()
        p = self.st.parts[r]
        p.seed_x, p.seed_y = self.bb[2] + 50.0, self.bb[3] + 50.0
        d = seeder._dispose_unseated(self.st, [r])[r]
        self.assertEqual(d['disposition'], 'off_board')

    def test_a_stack_is_staged_below_everything(self):
        """Two parts whose input pose is ON another part: both staged, below
        the board and every part, side by side, rotation kept."""
        movable = [r for r in sorted(self.st.parts)
                   if not self.st.parts[r].locked]
        host = self.st.parts[movable[0]]
        victims = movable[1:3]
        for r in victims:
            p = self.st.parts[r]
            p.seed_x, p.seed_y = host.x, host.y
        lowest = max(max(p.rect()[3] for p in self.st.parts.values()),
                     self.bb[3])
        out = seeder._dispose_unseated(self.st, victims)
        rects = []
        for r in victims:
            d = out[r]
            self.assertEqual(d['disposition'], 'staged', d)
            self.assertIsNotNone(d['refused_by'])
            p = self.st.parts[r]
            # `written` is the JSON record, rounded to 1e-4 mm.
            self.assertAlmostEqual(p.x, d['written'][0], places=4)
            self.assertAlmostEqual(p.y, d['written'][1], places=4)
            self.assertEqual(p.rot, p.orig_rot)
            rect = p.rect()
            self.assertGreater(rect[1], lowest)
            rects.append(rect)
        self.assertLessEqual(legality.rect_overlap_area(*rects), 0.0)

    def test_two_unseated_parts_are_never_both_left_on_one_spot(self):
        """Each part LEFT where it came in is an obstacle to the ones decided
        after it: two parts that would each be legal alone at one spot are
        not both left there."""
        r = self._movable_legal()
        p = self.st.parts[r]
        twin = next(o for o in sorted(self.st.parts)
                    if o != r and not self.st.parts[o].locked
                    and self.st.parts[o].orig_rot == p.orig_rot)
        q = self.st.parts[twin]
        q.seed_x, q.seed_y = p.seed_x, p.seed_y
        out = seeder._dispose_unseated(self.st, [twin, r])
        left = [k for k, d in out.items()
                if d['disposition'] == 'clear_at_input']
        self.assertLessEqual(len(left), 1, out)
        # Decided in SORTED order whatever order they came in: the first by
        # name is the one left, so the choice is not a hash accident.
        self.assertEqual(left, [min(r, twin)], out)

    def test_the_staging_row_clears_parts_below_the_board(self):
        """A part already sitting BELOW the board (a pile staged off it) is
        cleared too: the row starts under the lowest part, not the board."""
        movable = [r for r in sorted(self.st.parts)
                   if not self.st.parts[r].locked]
        below, host, victim = movable[0], movable[1], movable[2]
        b = self.st.parts[below]
        self.st.apply_move(below, b.x, self.bb[3] + 6.0, b.rot)
        v = self.st.parts[victim]
        h = self.st.parts[host]
        v.seed_x, v.seed_y = h.x, h.y
        out = seeder._dispose_unseated(self.st, [victim])
        self.assertEqual(out[victim]['disposition'], 'staged', out)
        self.assertGreater(self.st.parts[victim].rect()[1],
                           self.st.parts[below].rect()[3])


class AnEdgeConnectorAtItsEdge(unittest.TestCase):
    """The A/B's finding, as a test: a part that overhangs the board edge at
    its input pose -- an edge connector where the designer put it -- is NOT
    staged when it stacks on nothing. The seat predicate would call it
    illegal (it demands full containment), and staging it moved ulx3s's
    J1/J2 off the board for no defect at all."""

    def test_an_overhanging_part_with_no_stack_stays(self):
        import pose_score
        pcb = parse_kicad_pcb(os.path.join(ROOT, 'kicad_files',
                                           'ulx3s.kicad_pcb'))
        path = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
        st = pose_score.make_state(pcb, path, clearance=0.2)
        for r in ('J1', 'J2'):
            p = st.parts[r]
            pose = (p.seed_x, p.seed_y, p.orig_rot)
            # The premise: the SEAT predicate refuses it there...
            self.assertFalse(seeder.pose_ok(st, r, *pose, exclude=set()),
                             f'{r}: the fixture no longer overhangs')
            # ...and the disposition leaves it.
            d = seeder._dispose_unseated(st, [r])[r]
            self.assertEqual(d['disposition'], 'clear_at_input', d)


class OnThePile(unittest.TestCase):
    """rp2350 staged as an unaided pile (the A/B harness's own
    `_pile_inputs`), seeded under the pre-#1213 gate so U6 cannot seat."""

    @classmethod
    def setUpClass(cls):
        import test_placement_ab as ab
        from placement.writer import write_placed_output
        cls._td = tempfile.TemporaryDirectory()
        pile, intent, _doc, _refs = ab._pile_inputs(
            RP2350, cls._td.name, require_decaps=False)
        cls.res = {}
        cls.blocking = {}
        real_w, real_d = legality._pad_windows, seeder._dispose_unseated
        legality._pad_windows = _full
        try:
            for arm in ('off', 'on'):
                if arm == 'off':
                    seeder._dispose_unseated = lambda state, refs, **_k: {}
                else:
                    seeder._dispose_unseated = real_d
                res = seeder.seed_from_intent(
                    parse_kicad_pcb(pile), pile, intent, random.Random('0'),
                    clearance=0.2, board_edge_clearance=0.55, grid_step=0.1)
                out = os.path.join(cls._td.name, f'{arm}.kicad_pcb')
                write_placed_output(pile, out, res['placements'])
                g = legality.grade_body_overlap(parse_kicad_pcb(out), 0.2,
                                                pcb_file=out)
                cls.res[arm] = res
                cls.blocking[arm] = g['blocking_pairs']
        finally:
            legality._pad_windows, seeder._dispose_unseated = real_w, real_d

    @classmethod
    def tearDownClass(cls):
        cls._td.cleanup()

    def test_u6_is_unseated_in_both_arms(self):
        for arm in ('off', 'on'):
            self.assertIn('U6', self.res[arm]['unseated'], arm)

    def test_off_arm_stacks_on_u6(self):
        """The control: without the disposition, U6's input pose is written
        and the seated parts sit on it."""
        n = sum(1 for p in self.blocking['off'] if 'U6' in (p.a, p.b))
        self.assertGreater(n, 0)

    def test_on_arm_stages_u6_and_nothing_stacks_on_it(self):
        d = self.res['on']['unseated_disposition']['U6']
        self.assertEqual(d['disposition'], 'staged')
        self.assertFalse([p for p in self.blocking['on']
                          if 'U6' in (p.a, p.b)])

    def test_seated_poses_are_bit_identical(self):
        staged = {r for r, d in
                  self.res['on']['unseated_disposition'].items()
                  if d['disposition'] == 'staged'}
        off = {p['reference']: p for p in self.res['off']['placements']}
        on = {p['reference']: p for p in self.res['on']['placements']
              if p['reference'] not in staged}
        self.assertEqual(off, on)


class TheCliWritesTheStagedPose(unittest.TestCase):
    """`place_seed` end to end, polish included, on the rp2350 pile stripped
    of copper: every staged part is written where its record says, off the
    board -- the polish and the post-polish re-seat hold it -- and the
    `placed` count does not include it."""

    def test_staged_parts_are_written_where_recorded(self):
        import json
        import subprocess
        import test_placement_ab as ab
        with tempfile.TemporaryDirectory() as td:
            src = os.path.join(td, 'src', os.path.basename(RP2350))
            os.makedirs(os.path.dirname(src))
            subprocess.run([sys.executable, '-X', 'utf8', os.path.join(
                ROOT, 'tests', 'stress', 'strip_copper_only.py'), RP2350,
                src], check=True, capture_output=True, cwd=ROOT)
            pile, _intent, _doc, _refs = ab._pile_inputs(
                src, td, require_decaps=False)
            out = os.path.join(td, 'out', 'seed.kicad_pcb')
            os.makedirs(os.path.dirname(out))
            p = subprocess.run(
                [sys.executable, '-X', 'utf8', os.path.join(
                    ROOT, 'py_placer', 'place_seed.py'), pile, out,
                 '--intent', os.path.join(td, 'pile_intent.json')],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', cwd=ROOT)
            summary = None
            for line in p.stdout.splitlines():
                if line.startswith('JSON_SUMMARY: '):
                    summary = json.loads(line[len('JSON_SUMMARY: '):])
            self.assertIsNotNone(summary, p.stdout[-1500:] + p.stderr[-800:])
            disp = summary['unseated_disposition']
            staged = {r: d for r, d in disp.items()
                      if d['disposition'] == 'staged'}
            self.assertTrue(staged, 'nothing was staged, so this arm tests '
                            f'nothing: {disp}')
            self.assertEqual(sorted(disp), sorted(summary['unseated_refs']))
            written = parse_kicad_pcb(out)
            bb = written.board_info.board_bounds
            for r, d in staged.items():
                fp = written.footprints[r]
                self.assertAlmostEqual(fp.x, d['written'][0], places=3)
                self.assertAlmostEqual(fp.y, d['written'][1], places=3)
                self.assertTrue(all(pad.global_y > bb[3] for pad in fp.pads),
                                f'{r} is not below the board')
            seeded = int(next(l for l in p.stdout.splitlines()
                              if l.startswith('Seeded ')).split()[1])
            self.assertEqual(seeded, summary['placed'])




# ---------------------------------------------------------------------------
# The phase-1 verifier's fixtures: a 16 x 14 board, parts with F.CrtYd rects
# and a few SMD pads, and an intent with one board-wide zone.
# ---------------------------------------------------------------------------

def _part(ref, x, y, half_w, half_h, npads, pad_y=0.0):
    pads = ''.join(
        f'    (pad "{i + 1}" smd rect (at {i * 0.2 - 0.2} {pad_y})'
        f' (size 0.3 0.3) (layers "F.Cu")'
        f' (net {1 if i == 0 else 2} "N{1 if i == 0 else 2}"))\n'
        for i in range(npads))
    return (f'  (footprint "test:P{ref}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0))\n'
            f'    (fp_rect (start {-half_w} {-half_h}) (end {half_w} {half_h})'
            f' (layer "F.CrtYd"))\n' + pads + '  )\n')


def _load(ipath):
    from placement import floorplan
    return floorplan.load_intent(ipath)


def _small_board(td, parts, refs, must_lock=()):
    import json
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229)\n  (net 0 "") (net 1 "N1")'
                 ' (net 2 "N2")\n  (gr_rect (start 0 0) (end 16 14)'
                 ' (layer "Edge.Cuts"))\n' + ''.join(parts) + ')\n')
    intent = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
              "envelope": {"rect": [0.0, 0.0, 16.0, 14.0],
                           "tolerance_mm": 0.5},
              "blocks": [{"name": "everything", "refs": list(refs),
                          "zone": [0.5, 0.5, 15.5, 13.5],
                          "tolerance_mm": 0.5}]}
    if must_lock:
        intent['must_lock'] = list(must_lock)
    ipath = os.path.join(td, 'b.json')
    with open(ipath, 'w', encoding='utf-8') as fh:
        json.dump(intent, fh)
    return path, ipath


class TheReseatPath(unittest.TestCase):
    """`--reseat` re-seats a SCOPE inside a placed board; it keeps a scope ref
    it cannot seat where it was. Staging that ref there (the seed's
    disposition, reached through `seed_from_intent`) made the whole pass
    refuse on "off-outline part count GREW" and threw away MID's valid
    re-seat (phase-1 verifier, fixture "four")."""

    def test_reseat_keeps_its_valid_reseat(self):
        import subprocess
        with tempfile.TemporaryDirectory() as td:
            path, ipath = _small_board(td, [
                _part('BIG', 9.7, 7, 5.0, 5.0, 2, pad_y=1.5),
                _part('SMALL', 9.5, 8.5, 0.5, 0.5, 4),
                _part('MID', 9.9, 8.6, 0.5, 0.5, 3)], ['BIG', 'SMALL', 'MID'])
            out = os.path.join(td, 'out.kicad_pcb')
            p = subprocess.run(
                [sys.executable, '-X', 'utf8', os.path.join(
                    ROOT, 'py_placer', 'place_seed.py'), path, out,
                 '--intent', ipath, '--clearance', '0.2',
                 '--board-edge-clearance', '0.5', '--reseat', 'BIG', 'MID'],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', cwd=ROOT)
            text = p.stdout + p.stderr
            self.assertNotIn('STAGED', text)
            self.assertNotIn('off-outline part count GREW', text)
            mid = parse_kicad_pcb(out).footprints['MID']
            self.assertNotEqual((round(mid.x, 3), round(mid.y, 3)),
                                (9.9, 8.6), 'MID was not re-seated:\n'
                                + text[-1500:])

    def test_seed_from_intent_can_skip_the_disposition(self):
        with tempfile.TemporaryDirectory() as td:
            path, ipath = _small_board(td, [
                _part('BIG', 8, 7, 5.0, 5.0, 2),
                _part('SMALL', 9.5, 8.5, 0.5, 0.5, 4)], ['BIG', 'SMALL'])
            res = seeder.seed_from_intent(
                parse_kicad_pcb(path), path, _load(ipath), random.Random('0'),
                clearance=0.2, board_edge_clearance=0.5, grid_step=0.1,
                dispose_unseated=False)
            self.assertIn('BIG', res['unseated'])
            self.assertEqual(res['unseated_disposition'], {})
            self.assertNotIn('BIG', {p['reference']
                                     for p in res['placements']})


class AMustLockPartIsNeverStaged(unittest.TestCase):
    """A part the intent must_locks is stamped `(locked yes)` by place_seed;
    staged, it would be frozen off the board with nothing downstream to move
    it (phase-1 verifier, fixture "lock")."""

    def test_must_lock_is_locked_at_input(self):
        with tempfile.TemporaryDirectory() as td:
            path, ipath = _small_board(td, [
                _part('BIG', 8, 7, 9.0, 8.0, 2),
                _part('SMALL', 8, 7, 0.5, 0.5, 4),
                _part('S2', 8, 7, 0.5, 0.5, 3)], ['BIG', 'SMALL', 'S2'],
                must_lock=['BIG'])
            res = seeder.seed_from_intent(
                parse_kicad_pcb(path), path, _load(ipath), random.Random('0'),
                clearance=0.2, board_edge_clearance=0.5, grid_step=0.1)
            self.assertIn('BIG', res['unseated'])
            self.assertEqual(
                res['unseated_disposition']['BIG']['disposition'],
                'locked_at_input')


def _fp(ref, x, y, rot, pads, fab=None, crt=None, layer='F'):
    s = (f'  (footprint "t:{ref}" (layer "{layer}.Cu") (at {x} {y} {rot})\n'
         f'    (property "Reference" "{ref}" (at 0 0)'
         f' (layer "{layer}.SilkS"))\n')
    for lay, r in (('Fab', fab), ('CrtYd', crt)):
        if r:
            s += (f'    (fp_rect (start {r[0]} {r[1]}) (end {r[2]} {r[3]}) '
                  f'(stroke (width 0.1) (type default)) '
                  f'(layer "{layer}.{lay}"))\n')
    for (num, px, py, shape, sx, sy, net) in pads:
        s += (f'    (pad "{num}" smd {shape} (at {px} {py} {rot}) '
              f'(size {sx} {sy}) (layers "{layer}.Cu") (net {net} "N{net}"))\n')
    return s + '  )\n'


def _two_part_board(td, name, parts):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                 ' (44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1") (net 2 "N2") (net 3 "N3")\n'
                 '  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return path


#: The phase-2 verifier's cases (C6): the disposition's old mirror of the
#: grader staged the first two, which check_assembly calls buildable, and
#: left the last two, which it calls NOT BUILDABLE.
GRADER_CASES = {
    # round pads on a diagonal: the AABBs overlap, the circles are 1.13 apart
    'round_diag': ([_fp('A', 10, 10, 0, [('1', 0, 0, 'circle', 1.0, 1.0, 1),
                                         ('2', -3, 0, 'rect', 0.5, 0.5, 3)],
                        crt=(-3.5, -0.6, 0.6, 0.6)),
                    _fp('B', 10.8, 10.8, 0, [('1', 0, 0, 'circle', 1.0, 1.0, 2),
                                             ('2', 3, 0, 'rect', 0.5, 0.5, 3)],
                        crt=(-0.6, -0.6, 3.5, 0.6))], 'B', 'clear_at_input'),
    # a 10 x 10 body turned 45 degrees, S in its rect's corner, outside it
    'rot_fab': ([_fp('U', 20, 20, 45, [('1', 0, 0, 'rect', 1, 1, 1)],
                     fab=(-5, -5, 5, 5), crt=(-5.2, -5.2, 5.2, 5.2)),
                 _fp('S', 25.5, 25.5, 0, [('1', -0.3, 0, 'rect', 0.3, 0.3, 2),
                                          ('2', 0.3, 0, 'rect', 0.3, 0.3, 3)],
                     fab=(-0.5, -0.4, 0.5, 0.4), crt=(-0.6, -0.5, 0.6, 0.5))],
                'S', 'clear_at_input'),
    # two parts at ONE origin, pads clear of each other
    'coincident': ([_fp('J', 20, 20, 0, [('1', -5, 0, 'rect', 1, 1, 1),
                                         ('2', 5, 0, 'rect', 1, 1, 3)],
                        crt=(-6, -1, 6, 1)),
                    _fp('S', 20, 20, 0, [('1', -0.5, 0, 'rect', 0.4, 0.4, 2),
                                         ('2', 0.5, 0, 'rect', 0.4, 0.4, 3)],
                        crt=(-0.8, -0.4, 0.8, 0.4))], 'S', 'staged'),
}


class TheDispositionAsksTheGrader(unittest.TestCase):
    """`_dispose_unseated` stages a part exactly when check_assembly, grading
    the board as it would be written, gates it against a neighbour."""

    def _dispose(self, path, ref, **kw):
        import pose_score
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=0.2)
        return seeder._dispose_unseated(st, [ref], **kw)[ref]

    def _verdict(self, path):
        import json
        import subprocess
        jp = path + '.json'
        subprocess.run([sys.executable, '-X', 'utf8', os.path.join(
            ROOT, 'py_tools', 'check_assembly.py'), path, '--clearance',
            '0.2', '--json', jp], capture_output=True, text=True,
            encoding='utf-8', errors='replace', cwd=ROOT)
        with open(jp, encoding='utf-8') as fh:
            return json.load(fh)['buildable']

    def test_the_verifiers_cases(self):
        with tempfile.TemporaryDirectory() as td:
            for name, (parts, ref, want) in sorted(GRADER_CASES.items()):
                path = _two_part_board(td, name, parts)
                d = self._dispose(path, ref)
                self.assertEqual(d['disposition'], want, (name, d))
                # ...and check_assembly agrees on the same file.
                self.assertEqual(self._verdict(path), want != 'staged', name)

    def test_a_part_on_a_plugs_mating_region_is_staged(self):
        import test_1098_mating_keepout as t1098
        with tempfile.TemporaryDirectory() as td:
            path = t1098.board(td, r1=(15, 26, 'B.Cu'))
            d = self._dispose(path, 'R1')
            self.assertEqual(d['disposition'], 'staged', d)
            self.assertEqual(d['refused_by'][0], 'mating', d)
            self.assertFalse(self._verdict(path))

    def test_a_declared_waiver_is_honoured(self):
        """S wholly inside U's body: a gating containment, staged -- unless
        the intent's overlap_waivers names the pair, as check_assembly reads
        it."""
        parts = [_fp('U', 20, 20, 0, [('1', -4, 0, 'rect', 1, 1, 1)],
                     fab=(-5, -5, 5, 5), crt=(-5.2, -5.2, 5.2, 5.2)),
                 _fp('S', 21, 21, 0, [('1', -0.3, 0, 'rect', 0.3, 0.3, 2),
                                      ('2', 0.3, 0, 'rect', 0.3, 0.3, 3)],
                     fab=(-0.5, -0.4, 0.5, 0.4), crt=(-0.6, -0.5, 0.6, 0.5))]
        with tempfile.TemporaryDirectory() as td:
            path = _two_part_board(td, 'inside', parts)
            d = self._dispose(path, 'S')
            self.assertEqual(d['disposition'], 'staged', d)
            self.assertEqual(d['refused_by'], ['containment', 'U'], d)
            d = self._dispose(path, 'S', waivers=[('S', 'U')])
            self.assertEqual(d['disposition'], 'clear_at_input', d)

    def test_a_part_on_a_frame_pin_is_staged(self):
        """#1212's channel: P's courtyard over a pin frame's hole, nothing
        else wrong (test_1212's pin-only fixture)."""
        from test_1212_container_pins import _frame_board
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [('P', 3.0, 3.1, 2.0)])
            d = self._dispose(path, 'P')
        self.assertEqual(d['disposition'], 'staged', d)
        self.assertEqual(d['refused_by'], ['pin_in_courtyard', 'FR'], d)

    def test_the_seed_hands_over_the_intents_waivers_and_keepouts(self):
        """seed_from_intent passes the intent's overlap_waivers and keep-outs
        to the disposition, which grades with them as check_assembly does."""
        import json
        seen = {}
        real = seeder._dispose_unseated

        def spy(state, refs, **kw):
            seen.update(kw)
            return real(state, refs, **kw)
        with tempfile.TemporaryDirectory() as td:
            path, ipath = _small_board(td, [
                _part('BIG', 8, 7, 9.0, 8.0, 2),
                _part('SMALL', 8, 7, 0.5, 0.5, 4)], ['BIG', 'SMALL'])
            with open(ipath, encoding='utf-8') as fh:
                doc = json.load(fh)
            doc['overlap_waivers'] = [{'pair': ['BIG', 'SMALL'],
                                       'reason': 'test'}]
            doc['keepouts'] = [{'name': 'k', 'rect': [0.0, 0.0, 0.4, 0.4]}]
            with open(ipath, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh)
            seeder._dispose_unseated = spy
            try:
                res = seeder.seed_from_intent(
                    parse_kicad_pcb(path), path, _load(ipath),
                    random.Random('0'), clearance=0.2,
                    board_edge_clearance=0.5, grid_step=0.1)
            finally:
                seeder._dispose_unseated = real
        self.assertIn('BIG', res['unseated'])
        self.assertIn(frozenset(('BIG', 'SMALL')),
                      {frozenset(w) for w in seen.get('waivers', ())}, seen)
        self.assertEqual([k.get('name') for k in seen.get('keepouts', ())],
                         ['k'], seen)


FID = ('  (footprint "Fiducial:Fiducial_1mm_Mask2mm" (layer "F.Cu") (at 20 20)\n'
       '    (property "Reference" "FID1" (at 0 0) (layer "F.SilkS"))\n'
       '    (attr smd exclude_from_pos_files exclude_from_bom)\n'
       '    (fp_circle (center 0 0) (end 1.25 0) (stroke (width 0.05)'
       ' (type default)) (layer "F.CrtYd"))\n'
       '    (pad "" smd circle (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask")))\n')


def _wide(ref, layer='F'):
    return _fp(ref, 20, 20, 0, [('1', -3, 0, 'rect', 0.5, 0.5, 1),
                                ('2', 3, 0, 'rect', 0.5, 0.5, 2)],
               crt=(-3.5, -0.5, 3.5, 0.5), layer=layer)


def _narrow(ref, layer='F'):
    return _fp(ref, 20, 20, 0, [('1', -1.5, 0, 'rect', 0.5, 0.5, 1),
                                ('2', 1.5, 0, 'rect', 0.5, 0.5, 2)],
               crt=(-2, -0.5, 2, 0.5), layer=layer)


class TheSecondVerifiersCases(unittest.TestCase):
    """Each channel and input of the disposition, alone."""

    def _dispose(self, path, refs, **kw):
        import pose_score
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=0.2)
        return st, seeder._dispose_unseated(st, list(refs), **kw)

    def test_a_pad_intersection_alone_stages(self):
        """No body, no shared origin: pad copper meeting pad copper."""
        parts = [_fp('A', 10, 10, 0, [('1', 0, 0, 'rect', 1, 1, 1)],
                     crt=(-0.6, -0.6, 0.6, 0.6)),
                 _fp('S', 10.6, 10, 0, [('1', 0, 0, 'rect', 1, 1, 2)],
                     crt=(-0.6, -0.6, 0.6, 0.6))]
        with tempfile.TemporaryDirectory() as td:
            _st, d = self._dispose(_two_part_board(td, 'pads', parts), ['S'])
        self.assertEqual(d['S']['refused_by'], ['pads', 'A'], d)
        self.assertEqual(d['S']['basis'], 'check_assembly')

    def test_a_declared_mating_keepout_reaches_the_grade(self):
        """mating:J1 declared (not derived) over R1: staged only when the
        intent's keep-outs are handed to the grade."""
        import test_1098_mating_keepout as t1098
        k = [{'name': 'mating:J1', 'rect': [20, 5, 30, 15]}]
        with tempfile.TemporaryDirectory() as td:
            path = t1098.board(td, r1=(25, 10, 'B.Cu'))
            _st, d = self._dispose(path, ['R1'], keepouts=k)
            _st, d0 = self._dispose(path, ['R1'])
        self.assertEqual(d['R1']['refused_by'], ['mating', 'mating:J1'], d)
        self.assertEqual(d0['R1']['disposition'], 'clear_at_input', d0)

    def test_the_projects_rules_reach_the_grade(self):
        """The siblings travel with the scratch board: a project ignoring
        pth_inside_courtyard leaves a part on a frame pin where it is, as
        check_assembly does."""
        from test_1212_container_pins import _frame_board, _pro
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [('P', 3.0, 3.1, 2.0)])
            _pro(path, pth_inside_courtyard='ignore')
            _st, d = self._dispose(path, ['P'])
        self.assertEqual(d['P']['disposition'], 'clear_at_input', d)

    def test_a_marker_is_not_a_stack_partner(self):
        """A and B unseated with FID1 at one origin: A's only real partner
        is B, still undecided -- A stays, B is staged on A."""
        with tempfile.TemporaryDirectory() as td:
            path = _two_part_board(td, 'mk', [_wide('A'), _narrow('B'), FID])
            _st, d = self._dispose(path, ['A', 'B'])
        self.assertEqual(d['A']['disposition'], 'clear_at_input', d)
        self.assertEqual(d['B']['refused_by'], ['coincident', 'A'], d)

    def test_a_stack_is_per_face(self):
        """A and B unseated on F, C and D seated on B, one origin: A is in
        no F-side stack until B is decided."""
        with tempfile.TemporaryDirectory() as td:
            path = _two_part_board(td, 'faces', [
                _wide('A'), _narrow('B'), _wide('C', 'B'), _narrow('D', 'B')])
            _st, d = self._dispose(path, ['A', 'B'])
        self.assertEqual(d['A']['disposition'], 'clear_at_input', d)
        self.assertEqual(d['B']['refused_by'], ['coincident', 'A'], d)

    def test_a_locked_part_is_a_partner_whatever_the_names(self):
        """BIG is locked and never decided; the unseated part on it is
        staged whether its name sorts before BIG or after."""
        for name in ('A1', 'Z1'):
            parts = [_fp('BIG', 20, 20, 0, [('1', 0, 0, 'rect', 2, 2, 1)],
                         crt=(-9, -8, 9, 8)),
                     _fp(name, 20, 20, 0, [('1', 0, 0, 'rect', 2, 2, 2)],
                         crt=(-7.5, -6.5, 7.5, 6.5))]
            with tempfile.TemporaryDirectory() as td:
                path = _two_part_board(td, 'lk', parts)
                _st, d = self._dispose(path, ['BIG', name],
                                       locked={'BIG'})
            self.assertEqual(d['BIG']['disposition'], 'locked_at_input')
            self.assertEqual(d[name]['disposition'], 'staged', (name, d))

    def test_the_fallback_says_so(self):
        """No board file: the search's mirror decides, and the record
        says which answer it is."""
        with tempfile.TemporaryDirectory() as td:
            path = _two_part_board(td, 'fb', [_wide('A'), _narrow('B')])
            import pose_score
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.2)
        st.pcb_file = None
        d = seeder._dispose_unseated(st, ['B'])
        self.assertTrue(d['B']['basis'].startswith('search predicates'), d)


class TheStackRule(unittest.TestCase):
    """`placement_state.coincident_stack_groups`, the rule check_assembly
    and the disposition share: markers exempt, two real parts a stack."""

    def test_one_part_on_a_fiducial_is_no_stack(self):
        from placement import placement_state
        with tempfile.TemporaryDirectory() as td:
            path = _two_part_board(td, 'f1', [_narrow('R'), FID])
            self.assertEqual(placement_state.coincident_stack_groups(
                parse_kicad_pcb(path), path), [])
            path = _two_part_board(td, 'f2', [_narrow('R'), _wide('C')])
            groups = placement_state.coincident_stack_groups(
                parse_kicad_pcb(path), path)
        self.assertEqual([sorted(g['refs']) for g in groups], [['C', 'R']])


class PlaceSeedHelpers(unittest.TestCase):

    def test_staged_placed_and_repairable(self):
        import place_seed
        res = {'placements': [{'reference': r} for r in ('A', 'B', 'X')],
               'unseated_disposition': {
                   'X': {'disposition': 'staged'},
                   'Y': {'disposition': 'clear_at_input'}}}
        self.assertEqual(place_seed.staged_refs(res), ['X'])
        self.assertEqual(place_seed.placed_count(res), 2)
        errs = [types.SimpleNamespace(rule='zone_containment', ref='X'),
                types.SimpleNamespace(rule='zone_containment', ref='A'),
                types.SimpleNamespace(rule='keepout', ref=None),
                types.SimpleNamespace(rule='something_else', ref='B')]
        self.assertEqual(place_seed.repairable_refs(
            errs, {'zone_containment', 'keepout'}, ['X']), ['A'])


#: FID1's twin on the back.
FID_B = FID.replace('FID1', 'FID2').replace('"F.', '"B.')


class TheSecondPhase3VerifiersCases(unittest.TestCase):
    """The stack rule asked about the parts present, and an off-board part
    as a partner whatever the names."""

    def _dispose(self, path, refs, **kw):
        import pose_score
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=0.2)
        return seeder._dispose_unseated(st, list(refs), **kw)

    def test_a_part_and_a_marker_on_each_face_stack(self):
        """A over FID1 on F, C over FID2 on B, one origin: each face holds a
        part and a marker, so check_assembly gates all four, and A is
        staged on C."""
        with tempfile.TemporaryDirectory() as td:
            path = _two_part_board(td, 'xf', [_wide('A'), FID,
                                              _wide('C', 'B'), FID_B])
            d = self._dispose(path, ['A'])
            gated = TheDispositionAsksTheGrader._verdict(None, path)
        self.assertEqual(d['A']['refused_by'], ['coincident', 'C'], d)
        self.assertFalse(gated)

    def test_a_part_alone_on_its_face_is_not_in_the_far_stack(self):
        """A on F over C and D on B: check_assembly's group is C and D."""
        with tempfile.TemporaryDirectory() as td:
            path = _two_part_board(td, 'fb', [_wide('A'), _wide('C', 'B'),
                                              _narrow('D', 'B')])
            d = self._dispose(path, ['A'])
        self.assertEqual(d['A']['disposition'], 'clear_at_input', d)

    def test_an_off_board_partner_is_a_partner_whatever_the_names(self):
        """A reaches into the board, its partner sits wholly off it, one
        origin: the partner is never moved, so A is staged on it whether
        the partner's name sorts after A or before."""
        for name in ('B', '0B'):
            parts = [_fp('A', 41, 20, 0, [('1', -3, 0, 'rect', 0.5, 0.5, 1),
                                          ('2', 3, 0, 'rect', 0.5, 0.5, 2)],
                         crt=(-3.5, -0.5, 3.5, 0.5)),
                     _fp(name, 41, 20, 0,
                         [('1', -0.25, 0, 'rect', 0.3, 0.3, 3),
                          ('2', 0.25, 0, 'rect', 0.3, 0.3, 3)],
                         crt=(-0.5, -0.5, 0.5, 0.5))]
            with tempfile.TemporaryDirectory() as td:
                path = _two_part_board(td, 'ob', parts)
                d = self._dispose(path, ['A', name])
            self.assertEqual(d[name]['disposition'], 'off_board', (name, d))
            self.assertEqual(d['A']['refused_by'], ['coincident', name],
                             (name, d))


if __name__ == '__main__':
    unittest.main()
