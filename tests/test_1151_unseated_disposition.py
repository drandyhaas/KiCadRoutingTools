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
            if not p.locked and seeder.pose_ok(
                    self.st, r, p.seed_x, p.seed_y, p.orig_rot, set()):
                return r
        self.fail('no movable part is legal at its own designer pose')

    def test_legal_at_input_is_left(self):
        r = self._movable_legal()
        d = seeder._dispose_unseated(self.st, [r])[r]
        self.assertEqual(d['disposition'], 'legal_at_input')
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
        out = seeder._dispose_unseated(self.st, sorted([r, twin]))
        left = [k for k, d in out.items()
                if d['disposition'] == 'legal_at_input']
        self.assertLessEqual(len(left), 1, out)


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
                    seeder._dispose_unseated = lambda state, refs: {}
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


if __name__ == '__main__':
    unittest.main()
