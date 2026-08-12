#!/usr/bin/env python3
"""stage_blind must not publish a draw whose damage never landed (run 14).

Run 14 staged castor_pollux with a requested dose of >= 4.248 mm and an APPLIED
dose of 0.100 mm -- `perturb()`'s own `grid_step` -- because the drawn block sat
hard against the outline and `on_overflow='clip'` ate 98% of the dose.
`stage_blind` printed `damage applied` and exited 0 without reading
`rec['clipped']`, `rec['status']` or `rec['dose_mm_applied']`, so the run spent
an hour proving that an undamaged board was undamaged, and its recovery half was
void before it began.

These tests pin the three behaviours that close it:

  1. a draw that lands is accepted unchanged, and says nothing extra;
  2. a draw that does NOT land is redrawn, and the disclosure stays fence-safe
     (a redraw happened -- not how many, not the kind, not the dose);
  3. a board that will not take a material dose at all is REFUSED, rather than
     emitting a subject whose damage is at the grid step.

`perturb` is stubbed: the point under test is stage_blind's acceptance rule, and
driving the real mutation would make the test a perturbation test instead.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _fake_record(out_board, control_out, applied, *, status='ok',
                 clipped=False):
    """The subset of `perturb()`'s record that stage_blind reads."""
    shutil.copyfile(BOARD, out_board)
    if control_out:
        shutil.copyfile(BOARD, control_out)
    return {'status': status, 'clipped': clipped,
            'dose_mm_applied': applied, 'max_feasible_dose_mm': applied,
            'block': {'name': 'stub', 'members': []},
            'original_poses': {}}


class StageBlindDoseTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='sb14_')
        self.work = os.path.join(self.tmp, 'work')
        self.truth = os.path.join(self.tmp, 'truth')
        import stage_blind
        self.sb = stage_blind
        self._real = stage_blind.P.perturb
        self.calls = []

    def tearDown(self):
        self.sb.P.perturb = self._real
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _install(self, applied_seq):
        """Stub perturb to return each applied dose in turn, then the last."""
        seq = list(applied_seq)

        def fake(src, out_board, **kw):
            a = seq[len(self.calls)] if len(self.calls) < len(seq) else seq[-1]
            self.calls.append({'dose_mm': kw.get('dose_mm'), 'applied': a})
            return _fake_record(out_board, kw.get('control_out'), a,
                                clipped=a < kw.get('dose_mm', 0))
        self.sb.P.perturb = fake

    def _draw(self):
        with open(os.path.join(self.truth, 'draw.json'), encoding='utf-8') as f:
            return json.load(f)

    # ------------------------------------------------------------------
    def test_landing_draw_is_accepted_first_time(self):
        self._install([40.0])
        self.sb.main(BOARD, self.work, self.truth)
        self.assertEqual(len(self.calls), 1, 'a landing draw must not redraw')
        d = self._draw()
        self.assertEqual(len(d['draws']), 1)
        self.assertTrue(d['draws'][0]['accepted'])
        self.assertEqual(d['dose_mm_applied'], 40.0)

    def test_grid_step_draw_is_redrawn(self):
        """The run-14 case: applied 0.1 mm against a multi-mm request."""
        self._install([0.1, 0.1, 40.0])
        self.sb.main(BOARD, self.work, self.truth)
        self.assertEqual(len(self.calls), 3, 'must keep drawing until it lands')
        d = self._draw()
        self.assertEqual([a['accepted'] for a in d['draws']],
                         [False, False, True])
        # the accepted draw is the one written to the top-level keys
        self.assertEqual(d['dose_mm_applied'], 40.0)

    def test_redraw_disclosure_is_fence_safe(self):
        """One line, no count, no kind, no dose, no block."""
        self._install([0.1, 0.1, 0.1, 40.0])
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.sb.main(BOARD, self.work, self.truth)
        out = buf.getvalue()
        line = [l for l in out.splitlines() if 'redrew' in l]
        self.assertEqual(len(line), 1, 'exactly one redraw line')
        line = line[0]
        self.assertIn('dose infeasible', line)
        d = self._draw()
        # nothing that identifies the draw may reach the disclosure
        self.assertNotIn(str(d['kind']), line)
        self.assertNotIn('%.1f' % d['dose_mm'], line)
        self.assertNotIn(str(d['seed']), line)
        # ...and no digit at all, so it cannot carry the rejected-draw count
        self.assertFalse(any(c.isdigit() for c in line),
                         'the redraw line must carry no number: %r' % line)
        # the board-shape line is a deliberate, pre-existing disclosure and is
        # NOT what this test guards -- assert the two are separate lines.
        self.assertNotIn('parts', line)

    def test_board_that_never_lands_is_refused(self):
        self._install([0.1])
        with self.assertRaises(SystemExit) as cm:
            self.sb.main(BOARD, self.work, self.truth)
        msg = str(cm.exception)
        self.assertIn('no draw landed', msg)
        self.assertEqual(len(self.calls), self.sb.MAX_DRAWS,
                         'must exhaust the redraw budget before refusing')

    def test_material_bar_is_relative_as_well_as_absolute(self):
        """1.2 mm clears the 1.0 mm absolute floor but not the relative one.

        The requested dose is BOARD-SCALED, so it cannot be assumed: this
        fixture's short span is 30.5 mm, which puts the request at 3.0-6.7 mm,
        and at the bottom of that range 1.2 mm legitimately clears
        `0.35 * dose` too. (Caught by running this suite eight times, not once
        -- it failed 2/8 before the scale was pinned.) Pin the scale so the
        relative bar is the only thing under test.
        """
        real_scale = self.sb.board_scale_mm
        self.addCleanup(setattr, self.sb, 'board_scale_mm', real_scale)
        self.sb.board_scale_mm = lambda pcb: 200.0      # -> dose 12-44 mm
        self._install([1.2, 40.0])
        self.sb.main(BOARD, self.work, self.truth)
        d = self._draw()
        self.assertGreater(self.sb.MATERIAL_FRAC * d['draws'][0]['dose_mm_requested'],
                           1.2, 'the relative bar must be the binding one here')
        self.assertGreater(1.2, self.sb.MIN_MATERIAL_MM,
                           'and the absolute floor must NOT be')
        self.assertEqual(len(self.calls), 2,
                         'a dose far below what was asked for is not a subject')

    def test_cli_still_runs_end_to_end(self):
        """The real perturb, unstubbed, through the command line."""
        self.sb.P.perturb = self._real
        w = os.path.join(self.tmp, 'w2')
        t = os.path.join(self.tmp, 't2')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'tests', 'stress', 'stage_blind.py'),
             BOARD, w, t],
            capture_output=True, text=True, timeout=900)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('damage applied', r.stdout)
        self.assertTrue(os.path.isfile(os.path.join(w, 'board.kicad_pcb')))
        d = json.load(open(os.path.join(t, 'draw.json'), encoding='utf-8'))
        applied = d['dose_mm_applied']
        need = max(self.sb.MIN_MATERIAL_MM,
                   self.sb.MATERIAL_FRAC * d['dose_mm'])
        self.assertGreaterEqual(
            applied, need,
            'the shipped subject must carry a dose that actually landed')


if __name__ == '__main__':
    unittest.main(verbosity=2)
