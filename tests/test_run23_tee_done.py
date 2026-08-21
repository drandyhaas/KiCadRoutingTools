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


if __name__ == '__main__':
    unittest.main(verbosity=1)
