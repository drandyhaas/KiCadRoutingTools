#!/usr/bin/env python3
"""`tests/gui_parity/run_gui_parity.py` must survive a gate that times out.

A gate that outlives `--timeout` is killed with its whole process tree and
reported as TIMEOUT, and the sweep goes on to the next gate. On Windows the
kill used `os.killpg` / `signal.SIGKILL`, which Windows does not have, so the
first timeout ended the whole sweep with an AttributeError.

Drives the runner's own `main()` over two tiny gates, no wx and no KiCad: one
starts a child process and sleeps past a 1 s timeout, the next exits 0.

    python3 -X utf8 tests/test_gui_parity_runner_timeout.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

TESTS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(TESTS, 'gui_parity'))

import run_gui_parity                                           # noqa: E402

SLOW = '''import os, subprocess, sys, time
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
with open(sys.argv[0] + '.pid', 'w') as f:
    f.write(str(child.pid))
time.sleep(60)
'''


def _alive(pid):
    if os.name == 'nt':
        out = subprocess.run(['tasklist', '/FI', f'PID eq {pid}', '/NH'],
                             capture_output=True, text=True).stdout
        return str(pid) in out
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:   # a killed process nobody reaped yet (a zombie) is gone
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return True


class TestGateTimeout(unittest.TestCase):
    def test_a_timed_out_gate_is_reported_and_the_sweep_goes_on(self):
        d = tempfile.mkdtemp()
        try:
            slow = os.path.join(d, 'test_slow_gate.py')
            fast = os.path.join(d, 'test_fast_gate.py')
            with open(slow, 'w', encoding='utf-8') as f:
                f.write(SLOW)
            with open(fast, 'w', encoding='utf-8') as f:
                f.write('raise SystemExit(0)\n')
            old_gates, old_argv = run_gui_parity.gates, sys.argv
            run_gui_parity.gates = lambda _filters: [slow, fast]
            sys.argv = ['run_gui_parity.py', '--timeout', '1', '--log-dir', d]
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    code = run_gui_parity.main()
            finally:
                run_gui_parity.gates, sys.argv = old_gates, old_argv
            out = buf.getvalue()
            self.assertEqual(code, 1, out)
            self.assertRegex(out, r'TIMEOUT\s+test_slow_gate', out)
            self.assertRegex(out, r'PASS\s+test_fast_gate', out)
            with open(slow + '.pid') as f:
                child = int(f.read())
            for _ in range(50):
                if not _alive(child):
                    break
                time.sleep(0.1)
            self.assertFalse(_alive(child), 'the timed-out gate left its child running')
        finally:   # on Windows a killed gate can hold its log open a moment
            shutil.rmtree(d, ignore_errors=True)

if __name__ == '__main__':
    sys.exit(0 if unittest.main(exit=False).result.wasSuccessful() else 1)
