#!/usr/bin/env python3
"""`run_all.py` must not report a test that declined to run as a pass.

Two runner behaviours are pinned here, both of which already have producers
in this repo:

  * SELF-SKIP. `tests/test_run22_containment_repair.py` and
    `tests/test_run22_seat_containment_gate.py` both `return SKIP_EXIT` (77)
    when the fixture they need is absent. The runner had no bucket for that
    code, so a genuine "cannot run" landed in the summary as
    `FAIL ... (exit 77)` -- which reads as a product failure and sends the
    next reader to debug a test that never started. It now gets its own
    SELF-SKIPPED bucket, printed with the reason the test gave, and it is
    still not a pass.

  * A DECLARED BUDGET. A test may raise its own subprocess budget above the
    runner's `--timeout` with a module-level `RUN_ALL_TIMEOUT`. Read from
    the SOURCE, never by importing -- importing a test runs it -- and it can
    only raise, never lower: a test may say it is slow, it may not say the
    runner should give up on it sooner.

    python3 -X utf8 tests/test_run_all_buckets.py
"""
import io
import contextlib
import os
import sys
import tempfile
import unittest

TESTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TESTS)

import run_all                                                  # noqa: E402


def _write(d, name, body):
    p = os.path.join(d, name)
    with open(p, 'w', encoding='utf-8') as f:
        f.write(body)
    return p


def _run_runner(files):
    """Drive `run_all.main()` over exactly `files`. Returns (code, stdout)."""
    old_discover, old_argv = run_all.discover, sys.argv
    run_all.discover = lambda _filters: list(files)
    sys.argv = ['run_all.py', '-j', '1']
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            code = run_all.main()
    finally:
        run_all.discover, sys.argv = old_discover, old_argv
    return code, buf.getvalue()


class TestDeclaredBudget(unittest.TestCase):
    def test_a_declared_budget_is_read_from_the_source(self):
        with tempfile.TemporaryDirectory() as d:
            p = _write(d, 'test_slow.py', 'RUN_ALL_TIMEOUT = 1800\n')
            self.assertEqual(run_all._declared_budget(p, 600.0), 1800.0)

    def test_it_can_only_raise_the_budget(self):
        """A test may say it is slow; it may not shorten the runner's rope."""
        with tempfile.TemporaryDirectory() as d:
            p = _write(d, 'test_liar.py', 'RUN_ALL_TIMEOUT = 5\n')
            self.assertEqual(run_all._declared_budget(p, 600.0), 600.0)

    def test_no_declaration_leaves_the_default_alone(self):
        with tempfile.TemporaryDirectory() as d:
            p = _write(d, 'test_plain.py', 'print("hi")\n')
            self.assertEqual(run_all._declared_budget(p, 600.0), 600.0)

    def test_an_unreadable_file_does_not_crash_the_runner(self):
        self.assertEqual(
            run_all._declared_budget(os.path.join(TESTS, 'no-such-file'),
                                     600.0), 600.0)


class TestSelfSkipBucket(unittest.TestCase):
    #: The two shapes a runner has to tell apart, plus a real pass so the
    #: summary line has something green to be wrong about.
    PASS_SRC = 'print("all good")\n'
    SKIP_SRC = ('import sys\n'
                'print("SKIP: the corpus board is not in this checkout")\n'
                'sys.exit(77)\n')
    FAIL_SRC = 'import sys\nprint("boom")\nsys.exit(1)\n'
    #: 77 is an ordinary exit code. A test that CRASHES with it, or that
    #: propagates a child's 77, says nothing on stdout -- and a bucket that
    #: trusts the number alone turns that into a green suite.
    CRASH77_SRC = '''
import sys
sys.stderr.write("Traceback (most recent call last)")
sys.exit(77)
'''

    def test_a_self_skip_is_not_a_pass(self):
        with tempfile.TemporaryDirectory() as d:
            files = [_write(d, 'test_ok.py', self.PASS_SRC),
                     _write(d, 'test_skips.py', self.SKIP_SRC)]
            code, out = _run_runner(files)
            self.assertIn('SKIP  test_skips.py', out)
            self.assertIn('SELF-SKIPPED (1)', out)
            self.assertIn('1 passed', out, out[-400:])
            self.assertIn('+1 self-skipped', out, out[-400:])
            # It asserted nothing, so it is not counted as a pass -- and it
            # is not a failure either, so the run is still green.
            self.assertEqual(code, 0, out[-400:])

    def test_the_reason_the_test_gave_is_printed(self):
        """A bucket with no reason in it just relocates the mystery."""
        with tempfile.TemporaryDirectory() as d:
            files = [_write(d, 'test_skips.py', self.SKIP_SRC)]
            _code, out = _run_runner(files)
            self.assertIn('the corpus board is not in this checkout', out)

    #: A crash that exits 77 while being CHATTY. This repo's tools print
    #: lines like "skipping d3 (already routed)", so a bare `SKIP` prefix
    #: match re-buckets a real failure as a skip and the suite goes green.
    CHATTY77_SRC = """
import sys
print("skipping net /GND (already routed)")
sys.stderr.write("Traceback (most recent call last)")
sys.exit(77)
"""

    def test_ordinary_skipping_prose_does_not_excuse_a_crash(self):
        """The declaration is `SKIP:` with the colon.

        Every self-skipping test in the tree prints the colon form; ordinary
        prose does not. Without it, a crash-77 that happened to mention
        skipping anything was bucketed as a self-skip.
        """
        with tempfile.TemporaryDirectory() as d:
            files = [_write(d, 'test_chatty77.py', self.CHATTY77_SRC)]
            code, out = _run_runner(files)
            self.assertIn('FAIL  test_chatty77.py  (exit 77)', out, out)
            self.assertNotIn('SELF-SKIPPED', out, out)
            self.assertEqual(code, 1, out[-400:])

    def test_exit_77_without_a_SKIP_line_is_a_failure(self):
        """The bucket keys on the DECLARATION, not on the number.

        Before this guard, a crash that happened to exit 77 printed
        `SKIP test_crash77.py (self-skipped)` with a blank reason and the
        runner exited 0, where main gives `FAIL (exit 77)` and exits 1. That
        is strictly worse than the behaviour the bucket replaced.
        """
        with tempfile.TemporaryDirectory() as d:
            files = [_write(d, 'test_crash77.py', self.CRASH77_SRC)]
            code, out = _run_runner(files)
            self.assertIn('FAIL  test_crash77.py  (exit 77)', out, out)
            self.assertIn('no "SKIP: <reason>" line', out, out)
            self.assertNotIn('SELF-SKIPPED', out, out)
            self.assertEqual(code, 1, out[-400:])

    def test_a_declared_skip_still_lands_in_its_bucket(self):
        """The guard must not take the honest case with it."""
        with tempfile.TemporaryDirectory() as d:
            files = [_write(d, 'test_skips.py', self.SKIP_SRC),
                     _write(d, 'test_crash77.py', self.CRASH77_SRC)]
            code, out = _run_runner(files)
            self.assertIn('SELF-SKIPPED (1)', out, out)
            self.assertIn('SKIP  test_skips.py', out, out)
            self.assertIn('FAIL  test_crash77.py', out, out)
            self.assertEqual(code, 1, out[-400:])

    def test_a_real_failure_is_still_a_failure(self):
        """The new bucket must not swallow the one that decides the exit."""
        with tempfile.TemporaryDirectory() as d:
            files = [_write(d, 'test_skips.py', self.SKIP_SRC),
                     _write(d, 'test_broken.py', self.FAIL_SRC)]
            code, out = _run_runner(files)
            self.assertEqual(code, 1, out[-400:])
            self.assertIn('Failed: test_broken.py', out)
            self.assertNotIn('Failed: test_skips.py', out)


class TestScratchDir(unittest.TestCase):
    """Every test runs with TMPDIR/TEMP/TMP pointed at a scratch dir of its own,
    and run_all removes it afterwards -- a full local run used to leave ~525
    fixture copies (~220 MB) in the system temp dir."""

    LEAKY_SRC = (
        "import os, sys, tempfile\n"
        "fd, p = tempfile.mkstemp(suffix='.kicad_pcb')\n"
        "os.close(fd)\n"
        "open(os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])),\n"
        "                  'where.txt'), 'w').write(p)\n")

    def test_a_tests_temp_files_live_and_die_in_its_own_scratch_dir(self):
        with tempfile.TemporaryDirectory() as d:
            code, out = _run_runner([_write(d, 'test_leaky.py', self.LEAKY_SRC)])
            self.assertEqual(code, 0, out[-400:])
            with open(os.path.join(d, 'where.txt')) as f:
                made = f.read()
            self.assertIn(os.sep + 'krt_', made,
                          f'the test did not see its scratch TEMP: {made}')
            self.assertFalse(os.path.exists(made),
                             'the file the test left behind survived its run')
            self.assertNotIn('WARN', out)

    def _held(self, seconds):
        """A scratch dir whose one file a child process holds open."""
        import subprocess
        d = tempfile.mkdtemp(prefix='krt_held_')
        f = os.path.join(d, 'held.bin')
        ready = os.path.join(tempfile.gettempdir(), os.path.basename(d) + '.ready')
        child = subprocess.Popen(
            [sys.executable, '-c',
             'import sys, time\n'
             'fh = open(sys.argv[1], "wb")\n'
             'open(sys.argv[2], "w").close()\n'
             'time.sleep(float(sys.argv[3]))\n',
             f, ready, str(seconds)])
        for _ in range(200):
            if os.path.exists(ready):
                break
            import time
            time.sleep(0.02)
        os.remove(ready)
        return d, child

    def test_a_file_a_child_still_holds_is_retried_not_leaked(self):
        """Measured: a run left two scratch dirs behind, one holding KiCad's
        single-instance lock from a kicad-cli the test spawned; a rerun left
        none. The child lets go a moment later, so the removal retries."""
        d, child = self._held(0.8)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            run_all._rmtree_scratch(d)
        child.wait()
        self.assertFalse(os.path.exists(d), 'the held scratch dir was left behind')
        self.assertNotIn('WARN', buf.getvalue())

    @unittest.skipUnless(os.name == 'nt', 'only Windows refuses to remove an open file')
    def test_negative_control_without_the_retry_it_leaks_and_says_so(self):
        d, child = self._held(0.8)
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                run_all._rmtree_scratch(d, waits=())
            self.assertTrue(os.path.exists(d), 'the control did not hold the file')
            self.assertIn('WARN  could not fully remove scratch dir', buf.getvalue())
        finally:
            child.wait()
            run_all._rmtree_scratch(d)


if __name__ == '__main__':
    unittest.main(verbosity=1)
