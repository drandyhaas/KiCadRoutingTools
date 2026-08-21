"""RUN_STATE.json: the machine-readable NOW view of a run (run-23).

Run 23 measured 74% of its wall clock as agent context work, much of it
re-deriving "where are we" from ledger + journal + logs -- history existed,
a snapshot did not. Pinned here: converge.record refreshes RUN_STATE.json;
both drivers stamp their stage onto it; `loop_driver --status` renders it
and degrades honestly on an empty workdir; and the placement-lap record
templates now carry --score-file (the run-23 L5 "plateau NOT ANSWERABLE"
fix -- a score-less lap is invisible to every comparison downstream).
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

LOOP = os.path.join(ROOT, '.claude', 'skills',
                    'plan-pcb-placement-and-routing', 'scripts',
                    'loop_driver.py')
PLACE = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                     'scripts', 'placement_driver.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _run(*argv, cwd=ROOT):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run([sys.executable, '-X', 'utf8', *argv],
                          capture_output=True, text=True, env=env, cwd=cwd)


class TestRunState(unittest.TestCase):
    def test_record_refreshes_run_state(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            score = os.path.join(td, 'score.json')
            json.dump({'blocking': 7, 'blocking_by': {'unrouted': 7},
                       'quality': {'vias': 0}, 'board_sha': 'x'},
                      open(score, 'w'))
            r = _run(os.path.join(ROOT, 'py_placer', 'converge.py'),
                     'record', '--ledger', ledger, '--board', BOARD,
                     '--kind', 'placement', '--lever', 'test lap',
                     '--score-file', score)
            self.assertEqual(r.returncode, 0, r.stderr[-400:])
            sp = os.path.join(td, 'RUN_STATE.json')
            self.assertTrue(os.path.exists(sp), 'record did not write state')
            st = json.load(open(sp, encoding='utf-8'))
            self.assertEqual(st['source_ledger_row'], 1)
            self.assertEqual(st['blocking'], 7)
            self.assertEqual(st['phase'], 'placement')
            self.assertIn('written_at', st)

    def test_driver_stage_stamps_the_state(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            # L1 on a real board: emits the delegation prompt and must stamp.
            r = _run(LOOP, '--stage', 'L1', '--board', BOARD,
                     '--ledger', ledger, '--workdir', td)
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            st = json.load(open(os.path.join(td, 'RUN_STATE.json'),
                                encoding='utf-8'))
            self.assertEqual(st['last_stage'], 'L1')
            self.assertFalse(st['stage_refused'])
            # placement_driver stamps with its own namespace.
            r2 = _run(PLACE, '--stage', 'P0', '--board', BOARD,
                      '--workdir', td)
            self.assertIn(r2.returncode, (0, 4), r2.stdout[-400:])
            st2 = json.load(open(os.path.join(td, 'RUN_STATE.json'),
                                 encoding='utf-8'))
            self.assertEqual(st2['last_stage'], 'placement:P0')
            # ...and journals itself (the loop_driver asymmetry, closed).
            self.assertTrue(os.path.exists(
                os.path.join(td, 'placement_driver.log')))

    def test_status_renders_on_an_empty_workdir(self):
        with tempfile.TemporaryDirectory() as td:
            r = _run(LOOP, '--status',
                     '--ledger', os.path.join(td, 'ledger.jsonl'))
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            self.assertIn('nothing recorded yet', r.stdout)

    def test_status_renders_a_real_ledger(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            score = os.path.join(td, 'score.json')
            json.dump({'blocking': 2, 'quality': {}}, open(score, 'w'))
            _run(os.path.join(ROOT, 'py_placer', 'converge.py'),
                 'record', '--ledger', ledger, '--board', BOARD,
                 '--kind', 'placement', '--lever', 'lap', '--score-file',
                 score)
            r = _run(LOOP, '--status', '--ledger', ledger)
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            self.assertIn('"blocking": 2', r.stdout)


class TestScoreFileTemplates(unittest.TestCase):
    """T3: the placement record templates the drivers PRINT carry a score.

    Run 23's 17 score-less placement laps made L5's plateau question NOT
    ANSWERABLE. The engine change is nothing; the templates are the surface
    agents actually follow, so the templates are what is pinned.
    """

    def test_placement_driver_lap_template_carries_score_file(self):
        src = open(PLACE, encoding='utf-8').read()
        i = src.find('Then record it, before starting the next lap')
        self.assertGreater(i, 0)
        block = src[i:i + 900]
        self.assertIn('--score-file', block)
        self.assertIn('board_score.py', block)

    def test_loop_driver_templates_carry_score_file(self):
        src = open(LOOP, encoding='utf-8').read()
        # The L2 freeze record and the L2 not-in-ledger refusal both must.
        i = src.find('L2 freeze: <n> refs')
        self.assertGreater(i, 0)
        self.assertIn('--score-file', src[max(0, i - 900):i])
        j = src.find('is not in {a.ledger}')
        self.assertGreater(j, 0)
        self.assertIn('--score-file', src[j:j + 1600])


if __name__ == '__main__':
    unittest.main(verbosity=1)
