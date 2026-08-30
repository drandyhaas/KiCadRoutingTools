"""tee_cmd writes logs/<label>.done -- ONE canonical completion signal.

Run 23's orchestrator waited on `[tee_cmd] <label> exit=` -- a line tee_cmd
prints to STDOUT only -- by grepping the LOG, which carries `EXIT=N`
instead. The waiter timed out; ~25 minutes of wall were lost to a completion
signal split across two streams. The `.done` file holds the exit code and
appears exactly when the child exits; a waiter polls os.path.exists of it
and nothing else.
"""

import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEE = os.path.join(ROOT, 'tests', 'stress', 'tee_cmd.py')


def _tee(td, label, *cmd):
    return subprocess.run(
        [sys.executable, '-X', 'utf8', TEE, '--workdir', td, label,
         '--', sys.executable, '-c', *cmd],
        capture_output=True, text=True, cwd=ROOT)


class TestTeeDone(unittest.TestCase):
    def test_done_carries_the_exit_code(self):
        with tempfile.TemporaryDirectory() as td:
            _tee(td, 'ok', 'print("hi")')
            _tee(td, 'bad', 'import sys; sys.exit(3)')
            self.assertEqual(
                open(os.path.join(td, 'logs', 'ok.done')).read().strip(), '0')
            self.assertEqual(
                open(os.path.join(td, 'logs', 'bad.done')).read().strip(),
                '3')

    def test_reused_label_gets_its_own_done(self):
        """Reused labels get numbered logs (lap semantics); the .done files
        must pair 1:1 with them or a waiter watches lap 1's marker while
        lap 2 runs. Run 24 (finding A-10) sharpened the canonical marker:
        the stage advice says "wait on logs/<label>.done", and the unsuffixed
        file used to keep the OLDEST attempt's exit forever -- a waiter armed
        during a retry returned instantly with a stale verdict. It is now
        last-attempt-wins: absent while a retry runs, then the newest code."""
        with tempfile.TemporaryDirectory() as td:
            _tee(td, 'lap', 'pass')
            self.assertEqual(
                open(os.path.join(td, 'logs', 'lap.done')).read().strip(),
                '0')
            _tee(td, 'lap', 'import sys; sys.exit(2)')
            self.assertEqual(
                open(os.path.join(td, 'logs', 'lap.done')).read().strip(),
                '2', 'canonical marker must carry the NEWEST attempt')
            self.assertEqual(
                open(os.path.join(td, 'logs', 'lap.2.done')).read().strip(),
                '2')

    def test_canonical_marker_absent_while_a_retry_runs(self):
        """The stale-read hazard itself: while attempt 2 is in flight the
        canonical marker must be GONE, or a waiter reads attempt 1's exit."""
        import threading
        import time as _time
        with tempfile.TemporaryDirectory() as td:
            _tee(td, 'lap', 'pass')
            marker = os.path.join(td, 'logs', 'lap.done')
            timeline = []

            def watch():
                # Sample across the retry. The retry's own process startup
                # is allowed to still show the old marker (tee_cmd cannot
                # delete it before it runs); the contract is that the marker
                # then VANISHES for the duration and never returns stale.
                for _ in range(300):
                    timeline.append(
                        open(marker).read().strip()
                        if os.path.exists(marker) else None)
                    _time.sleep(0.01)

            t = threading.Thread(target=watch)
            t.start()
            _tee(td, 'lap',
                 'import time, sys; time.sleep(1.0); sys.exit(5)')
            t.join()
            self.assertIn(None, timeline,
                          'the canonical marker never vanished during the '
                          'retry -- a waiter would read the stale exit')
            after_gone = timeline[timeline.index(None):]
            self.assertNotIn('0', after_gone,
                             'the stale attempt-1 exit reappeared after the '
                             'marker vanished')
            self.assertEqual(open(marker).read().strip(), '5')


class TestTheMarkerAlwaysArrives(unittest.TestCase):
    """`<label>.done` appearing IS the waiting protocol.

    The canonical marker is REMOVED up front on a retry, so "absent means
    still in flight" is what a waiter believes. Any path that exits without
    writing one leaves that true forever, which is the hang this file exists
    to prevent -- and the paths that did so were the ones a run actually hits:
    a command that cannot start, and an interrupted wait.
    """

    def _done(self, td, label):
        return os.path.join(td, 'logs', label + '.done')

    def test_a_command_that_never_starts_still_marks_done(self):
        with tempfile.TemporaryDirectory() as td:
            r = subprocess.run(
                [sys.executable, '-X', 'utf8', TEE, '--workdir', td,
                 'nosuch', '--',
                 os.path.join(td, 'definitely-not-a-program')],
                capture_output=True, text=True, cwd=ROOT)
            marker = self._done(td, 'nosuch')
            self.assertTrue(os.path.exists(marker),
                            'no marker: a waiter blocks forever on a process '
                            'that will never exist')
            with open(marker, encoding='utf-8') as f:
                self.assertEqual(f.read().strip(), '127')
            self.assertEqual(r.returncode, 127, r.stdout[-300:])

    def test_the_log_says_what_happened(self):
        with tempfile.TemporaryDirectory() as td:
            subprocess.run(
                [sys.executable, '-X', 'utf8', TEE, '--workdir', td,
                 'nosuch', '--',
                 os.path.join(td, 'definitely-not-a-program')],
                capture_output=True, text=True, cwd=ROOT)
            with open(os.path.join(td, 'logs', 'nosuch.log'),
                      encoding='utf-8') as f:
                log = f.read()
            self.assertIn('could not start', log)
            # The EXIT= half of the anchor contract, which a parser reads.
            self.assertIn('EXIT=127', log)

    def test_an_interrupt_while_the_child_is_PRINTING_still_marks_done(self):
        """The streaming loop, not `wait()`, is where a long run lives.

        The guard used to wrap `proc.wait()` only, so a Ctrl-C while the child
        was printing propagated out with no marker -- leaving "absent means
        still in flight" permanently true, which is the hang this file exists
        to prevent, in the path that runs almost all of the time.
        """
        with tempfile.TemporaryDirectory() as td:
            child = ('import sys, time\n'
                     'for i in range(200):\n'
                     '    print(i, flush=True)\n'
                     '    time.sleep(0.05)\n')
            # A wrapper that raises INSIDE the streaming loop, which is what
            # a Ctrl-C at that moment does.
            harness = (
                'import runpy, sys, builtins\n'
                'sys.argv = ["tee_cmd.py", "--workdir", %r, "lap", "--",\n'
                '            %r, "-c", %r]\n'
                'real = builtins.print\n'
                'n = [0]\n'
                'def boom(*a, **k):\n'
                '    n[0] += 1\n'
                '    if n[0] > 3:\n'
                '        raise KeyboardInterrupt\n'
                '    return real(*a, **k)\n'
                'sys.stdout.write = lambda s: (_ for _ in ()).throw('
                'KeyboardInterrupt()) if s.strip() == "3" else None\n'
                'try:\n'
                '    runpy.run_path(%r, run_name="__main__")\n'
                'except BaseException:\n'
                '    pass\n' % (td, sys.executable, child, TEE))
            hp = os.path.join(td, 'harness.py')
            with open(hp, 'w', encoding='utf-8') as f:
                f.write(harness)
            subprocess.run([sys.executable, '-X', 'utf8', hp],
                           capture_output=True, text=True, cwd=ROOT,
                           timeout=120)
            marker = os.path.join(td, 'logs', 'lap.done')
            self.assertTrue(os.path.exists(marker),
                            'interrupted mid-stream and wrote no marker: a '
                            'waiter blocks forever')
            with open(marker, encoding='utf-8') as f:
                self.assertEqual(f.read().strip(), '-1')

    def test_workdir_with_an_equals_sign_is_parsed(self):
        """`--workdir=DIR` fell through into the LABEL, so the rows landed in
        this file's own directory under a label like `--workdir=wk/run24`."""
        with tempfile.TemporaryDirectory() as td:
            subprocess.run(
                [sys.executable, '-X', 'utf8', TEE, '--workdir=' + td,
                 'lap', '--', sys.executable, '-c', 'print(1)'],
                capture_output=True, text=True, cwd=ROOT)
            self.assertTrue(os.path.exists(self._done(td, 'lap')),
                            os.listdir(td))
            self.assertTrue(
                os.path.exists(os.path.join(td, 'cmd_timing.jsonl')),
                'the row did not land in the given work dir')


if __name__ == '__main__':
    unittest.main(verbosity=1)
