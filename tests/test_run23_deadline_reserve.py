"""Deadline reserve band + cancellable tail + heartbeat file (run-23).

R4 overshot its --deadline 1.74x (732s of 420) because the routing loop
consumed the whole budget and the tail -- write, plane finalize, oracle,
reconcile laps -- ran deadline-blind after it. Pinned here: the routing
closure carries a reserve (max(30, 20%)); the tail legs get a reserve-0
closure; the reconcile refuses to START on a spent deadline (with the
reason printed); stdout_progress can heartbeat to $KRT_PROGRESS_FILE; and
a run WITHOUT --deadline behaves exactly as before.
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _route(out, *argv, env_extra=None):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1', **(env_extra or {}))
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_router', 'route.py'), BOARD, out, *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestDeadline(unittest.TestCase):
    def test_tight_deadline_is_bounded_and_honest(self):
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'r.kicad_pcb')
            hb = os.path.join(td, 'hb.json')
            t0 = time.time()
            r = _route(out, '--deadline', '5',
                       env_extra={'KRT_PROGRESS_FILE': hb})
            wall = time.time() - t0
            # deadline 5 + reserve 30 + parse/write slack. The measured
            # failure mode was 1.74x overshoot on a 420s budget; the bound
            # here is deliberately loose for CI, tight against that.
            self.assertLess(wall, 120)
            self.assertEqual(r.returncode, 7, r.stdout[-400:])  # DEADLINE_EXIT
            # The partial run still emits the authoritative MIN line, and it
            # says INCOMPLETE -- a cancelled run yields a real gated verdict.
            line = [ln for ln in r.stdout.splitlines()
                    if ln.startswith('JSON_SUMMARY_MIN: ')]
            self.assertEqual(len(line), 1)
            doc = json.loads(line[0].split(': ', 1)[1])
            self.assertFalse(doc['complete'])
            # Heartbeat file ticked at least once.
            self.assertTrue(os.path.exists(hb))
            self.assertIn('label', json.load(open(hb, encoding='utf-8')))

    def test_no_deadline_behavior_unchanged(self):
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'r.kicad_pcb')
            r = _route(out)
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            self.assertNotIn('Final reconciliation SKIPPED', r.stdout)
            line = [ln for ln in r.stdout.splitlines()
                    if ln.startswith('JSON_SUMMARY_MIN: ')]
            doc = json.loads(line[0].split(': ', 1)[1])
            self.assertTrue(doc['complete'])
            self.assertEqual(doc['failed'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=1)
