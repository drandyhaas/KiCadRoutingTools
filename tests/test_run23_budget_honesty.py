"""emit-intent must not bake a budget that blesses the board it came from.

Run 23: the intent was emitted mid-repair from a board carrying J4 0.90mm
inside U6's courtyard. It baked legality_budget.overlap_area = 30.1085 (the
board's own number, ceil'd), and the final board's 26.302 then graded PASS --
the budget blessed the defect it was emitted over. The run-6 withholding
(body-blocking pairs freeze overlap_area) did not fire because no PAD pair
blocked.

The extension pinned here: unwaived courtyard interpenetrations past the
blocking floors (legality.COURTYARD_BLOCKING_MIN_*) also withhold
overlap_area, and the reason is recorded in context.budget_withheld so a
reader can tell "withheld" from "forgot". Healthy boards without such pairs
bake budgets exactly as before.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')
SPLITFLAP = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
RUN5 = os.path.join(ROOT, 'wk', 'run5', 'final5.kicad_pcb')


def _emit(board):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    with tempfile.TemporaryDirectory() as td:
        jp = os.path.join(td, 'intent.json')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_tools', 'check_floorplan.py'),
             board, '--emit-intent', jp],
            capture_output=True, text=True, env=env, cwd=ROOT)
        assert r.returncode == 0, r.stdout[-400:] + r.stderr[-400:]
        return json.load(open(jp, encoding='utf-8'))


class TestBudgetWithholding(unittest.TestCase):
    def test_courtyard_blocking_board_gets_no_overlap_budget(self):
        doc = _emit(PLACED)
        self.assertNotIn('overlap_area', doc['legality_budget'])
        withheld = doc['context']['budget_withheld']
        self.assertIn('overlap_area', withheld)
        self.assertIn('courtyard', withheld['overlap_area'])
        # The rest of the budget survives: oob is independently judged.
        self.assertIn('oob_count', doc['legality_budget'])

    def test_healthy_board_bakes_budget_unchanged(self):
        doc = _emit(SPLITFLAP)
        self.assertIn('overlap_area', doc['legality_budget'])
        self.assertEqual(doc['context']['budget_withheld'], {})

    def test_run6_body_blocking_withholding_still_fires_first(self):
        """The older, stronger reason must not be shadowed by the new one."""
        if not os.path.exists(RUN5):
            self.skipTest('run-5 deliverable not present')
        doc = _emit(RUN5)
        self.assertNotIn('overlap_area', doc['legality_budget'])
        withheld = doc['context']['budget_withheld']
        self.assertIn('blocking body', withheld['overlap_area'])


if __name__ == '__main__':
    unittest.main(verbosity=1)
