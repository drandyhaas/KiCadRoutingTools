"""JSON_SUMMARY_MIN: one compact authoritative line per outermost run.

Run 23's route logs carried 53 JSON_SUMMARY lines, the longest 19.8KB, with
scope semantics the log itself warns about ("never scrape the LAST
JSON_SUMMARY") -- and every agent consuming them paid that in context, per
lap. The MIN line is the merged verdict in <1KB, printed exactly once per
outermost batch_route (final_reconcile gates it, so plane-finalize and
reconcile sub-runs can never emit a second one).
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


class TestSummaryMin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.TemporaryDirectory()
        out = os.path.join(cls.td.name, 'routed.kicad_pcb')
        cls.json_out = os.path.join(cls.td.name, 'sum.json')
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
                   KRT_NO_BANNER='1')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_router', 'route.py'),
             BOARD, out, '--json-out', cls.json_out, '--deadline', '240'],
            capture_output=True, text=True, env=env, cwd=ROOT)
        assert r.returncode == 0, r.stdout[-800:]
        cls.log = r.stdout

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_exactly_one_min_line_and_it_is_last(self):
        from route_summary import SUMMARY_MIN_RE, SUMMARY_RE
        mins = SUMMARY_MIN_RE.findall(self.log)
        self.assertEqual(len(mins), 1, f'{len(mins)} MIN lines')
        # Authoritative-last: the MIN line must come after every big summary,
        # or a tail-reader can still scrape a subset-scope line by accident.
        self.assertGreater(self.log.rfind('JSON_SUMMARY_MIN: '),
                           max((self.log.rfind('JSON_SUMMARY: '), -1)))
        # And the two regexes must not eat each other.
        for big in SUMMARY_RE.findall(self.log):
            json.loads(big)   # every big match is still valid standalone JSON

    def test_min_matches_the_merged_json_out(self):
        from route_summary import SUMMARY_MIN_RE, summary_min
        got = json.loads(SUMMARY_MIN_RE.findall(self.log)[0])
        merged = json.load(open(self.json_out, encoding='utf-8'))
        self.assertEqual(got, summary_min(merged))
        self.assertEqual(got['scope'], 'merged')

    def test_min_is_small(self):
        from route_summary import SUMMARY_MIN_RE
        line = SUMMARY_MIN_RE.findall(self.log)[0]
        self.assertLess(len(line), 2048, f'{len(line)} bytes')

    def test_run23_partial_log_reduces_honestly(self):
        """The line must carry deadline-partial truth: complete=false,
        status=deadline, the failing nets by name -- validated against the
        log run 23 actually produced (R4, the 732s/420s overshoot), when
        that log is present."""
        log_path = os.path.join(ROOT, 'wk', 'run23', 'tigard', 'logs',
                                'R4_route.log')
        if not os.path.exists(log_path):
            self.skipTest('run-23 work dir not present')
        from route_summary import merge_route_summaries, summary_min
        m = merge_route_summaries(
            open(log_path, encoding='utf-8', errors='replace').read())
        got = summary_min(m)
        self.assertFalse(got['complete'])
        self.assertEqual(got['status'], 'deadline')
        self.assertIn('/TXLED', got['failed_single'])
        self.assertLess(len(json.dumps(got)), 1024)


if __name__ == '__main__':
    unittest.main(verbosity=1)
