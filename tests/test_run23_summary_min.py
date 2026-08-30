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
             BOARD, out, '--json-out', cls.json_out],
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
        # Content, not `summary_min(merged)` compared with itself: the
        # counts on the line must be the MERGED tally's, read two
        # independent ways -- the --json-out file and the printed
        # JSON_SUMMARY lines merged by the same reader a consumer uses.
        from route_summary import SUMMARY_MIN_RE, merge_route_summaries
        got = json.loads(SUMMARY_MIN_RE.findall(self.log)[0])
        with open(self.json_out, encoding='utf-8') as f:
            merged = json.load(f)
        from_log = merge_route_summaries(self.log)
        self.assertEqual(got['scope'], 'merged')
        self.assertGreater(merged['successful'], 0)
        self.assertEqual(got['routed'], merged['successful'])
        self.assertEqual(got['routed'], from_log['successful'])
        self.assertEqual(got['failed'], merged['failed'])
        self.assertEqual(got['failed'], from_log['failed'])
        self.assertEqual(got['failed_single'],
                         merged.get('failed_single') or [])
        self.assertEqual(got['open_single'], merged.get('open_single') or [])
        self.assertEqual(got['vias'], merged['total_vias'])
        self.assertEqual(got['min_clearance_used'],
                         merged.get('min_clearance_used'))
        # A first route onto a copper-free board is not gated, so the gate
        # key must be absent (it appears only when the gate reverted).
        self.assertNotIn('improvement_gate', got)
        self.assertNotIn('finalize_excluded_nets', got)

    def test_min_is_small(self):
        from route_summary import SUMMARY_MIN_RE
        line = SUMMARY_MIN_RE.findall(self.log)[0]
        self.assertLess(len(line), 2048, f'{len(line)} bytes')


class TestSummaryMinShape(unittest.TestCase):
    """Pure-function checks on `summary_min` (no routing)."""

    def test_counts_and_caps(self):
        from route_summary import summary_min
        merged = {'successful': 7, 'failed': 3, 'total_vias': 4,
                  'total_time': 1.5, 'min_clearance_used': 0.2,
                  'failed_single': [f'N{i}' for i in range(25)],
                  'pad_pairs_open': [{'net': 'P'}, {'net': 'P'}, {'net': 'Q'}],
                  'multipoint_pads_total': 10, 'multipoint_pads_connected': 8,
                  'terminal_restores': {'R': 'full', 'S': 'stub',
                                        'T': 'full_open'}}
        got = summary_min(merged)
        self.assertEqual((got['routed'], got['failed']), (7, 3))
        self.assertEqual(got['failed_single'][-1], '+5 more')
        self.assertEqual(len(got['failed_single']), 21)
        self.assertEqual(got['pad_pairs_open'], {'count': 3, 'nets': ['P', 'Q']})
        self.assertEqual(got['multipoint_deficit'], 2)
        self.assertEqual(got['terminal_restores_broken'], ['S', 'T'])
        self.assertNotIn('finalize_excluded_nets', got)
        self.assertNotIn('improvement_gate', got)
        # The time key is named for what `total_time` counts (single-ended
        # loop + reroute loop), not for wall clock. Measured 0.35 against
        # 20.72 s of real time on splitflap_driver, so a key called
        # `duration_s` on a verdict line is a false claim, not a shorter one.
        self.assertEqual(got['main_loop_time_s'], 1.5)
        self.assertNotIn('duration_s', got)

    def test_finalize_excluded_nets_only_when_present(self):
        from route_summary import summary_min
        got = summary_min({'successful': 1, 'failed': 0,
                           'finalize_excluded_nets': ['GND', 'VCC']})
        self.assertEqual(got['finalize_excluded_nets'], ['GND', 'VCC'])
        self.assertNotIn('finalize_excluded_nets',
                         summary_min({'successful': 1, 'failed': 0,
                                      'finalize_excluded_nets': []}))


class TestMergeCarriesFinalizeExcluded(unittest.TestCase):
    """`finalize_excluded_nets` must survive a reconcile sub-run.

    merge_summaries is last-wins on summaries[-1]. The key is stamped on the
    parent summary AFTER its JSON_SUMMARY printed, so it lives only on
    summaries[0]; without an explicit carry it vanished from the merged tally
    -- and therefore from --json-out and from this line -- on exactly the runs
    that reconciled, i.e. the ones with failures, where "the finalize declined
    these BY PLAN" is the thing a grader needs to know.
    """

    FIRST = {'successful': 5, 'failed': 2, 'failed_single': ['/A', '/B'],
             'scope': 'run', 'finalize_excluded_nets': ['/GND', '/VCC']}
    SUB = {'successful': 1, 'failed': 1, 'failed_single': ['/A'],
           'scope': 'reconciliation-subset'}

    def test_carried_across_the_sub_run(self):
        from route_summary import merge_summaries, summary_min
        merged = merge_summaries([self.FIRST, self.SUB])
        self.assertEqual(merged['finalize_excluded_nets'], ['/GND', '/VCC'])
        self.assertEqual(summary_min(merged)['finalize_excluded_nets'],
                         ['/GND', '/VCC'])

    def test_single_summary_unchanged(self):
        from route_summary import merge_summaries
        self.assertEqual(
            merge_summaries([self.FIRST])['finalize_excluded_nets'],
            ['/GND', '/VCC'])

    def test_absent_stays_absent(self):
        from route_summary import merge_summaries
        first = {k: v for k, v in self.FIRST.items()
                 if k != 'finalize_excluded_nets'}
        self.assertNotIn('finalize_excluded_nets',
                         merge_summaries([first, self.SUB]))


class TestNoOpRunStillEmits(unittest.TestCase):
    """A step that legitimately routed nothing still emits its MIN line.

    "All nets are already fully connected" is a routine chain state -- a
    replay, a retry, a re-run of a finished step. That path returns from
    batch_route long before the end-of-run emit, so the line was missing
    exactly there, and a consumer holding the "one MIN line per run" contract
    reads a missing line as a crashed run.
    """

    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.TemporaryDirectory()
        first = os.path.join(cls.td.name, 'routed.kicad_pcb')
        again = os.path.join(cls.td.name, 'again.kicad_pcb')
        env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
                   KRT_NO_BANNER='1')

        def _run(src, dst):
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_router', 'route.py'), src, dst],
                capture_output=True, text=True, env=env, cwd=ROOT)
            assert r.returncode == 0, r.stdout[-800:]
            return r.stdout

        _run(BOARD, first)
        # Re-route the finished board: every net is already connected.
        cls.log = _run(first, again)

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_no_op_run_emits_exactly_one_min_line_with_status(self):
        from route_summary import SUMMARY_MIN_RE
        self.assertIn('nothing to route', self.log)
        mins = SUMMARY_MIN_RE.findall(self.log)
        self.assertEqual(len(mins), 1, f'{len(mins)} MIN lines on a no-op run')
        got = json.loads(mins[0])
        self.assertEqual(got['status'], 'already_connected')
        self.assertEqual(got['routed'], 0)
        self.assertEqual(got['failed'], 0)

    def test_ordinary_run_carries_no_status_key(self):
        """`status` marks the no-op paths only, so it stays a signal."""
        from route_summary import summary_min
        self.assertNotIn('status', summary_min({'successful': 1, 'failed': 0}))


if __name__ == '__main__':
    unittest.main(verbosity=1)
