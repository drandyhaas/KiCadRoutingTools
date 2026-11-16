#!/usr/bin/env python3
"""Run every tests/gui_parity gate and report each one's exit, one at a time.

`run_all.py` does not collect this directory, and the suite's cloud image has
no KiCad, so nothing else runs these gates: a gate that went stale on main
(test_666's bare-ball arm after #1778) failed there unseen. Run this from a
machine with KiCad installed:

    python3 tests/gui_parity/run_gui_parity.py                 # every gate
    python3 tests/gui_parity/run_gui_parity.py --filter 1698 666
    python3 tests/gui_parity/run_gui_parity.py --timeout 900 --log-dir /tmp/gp

Each gate runs under plain python3 and relaunches itself into KiCad's python.
The verdict is the gate's own exit code: 0 PASS, 77 a declared SKIP (asserted
nothing, so it is NOT a pass and is listed apart), anything else FAIL. A gate
that outlives --timeout is killed with its whole process group (it relaunched,
so the interpreter doing the work is a child) and counts as a FAIL. Gates run
one at a time: wx gates that share a machine's window server interfere.

Exits 0 when no gate failed or timed out, else 1. A gate's full output is in
--log-dir (default: a fresh temp dir, printed at the end).
"""
import argparse
import glob
import os
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))


def gates(filters):
    paths = sorted(glob.glob(os.path.join(HERE, 'test_*.py')))
    if filters:
        paths = [p for p in paths if any(f in os.path.basename(p) for f in filters)]
    return paths


def run_one(path, timeout, log_path):
    t0 = time.monotonic()
    with open(log_path, 'w', encoding='utf-8', errors='replace') as log:
        proc = subprocess.Popen([sys.executable, path], stdout=log,
                                stderr=subprocess.STDOUT, cwd=REPO,
                                start_new_session=True)
        try:
            rc = proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                proc.kill()
            proc.wait()
            rc = None
    return rc, time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--filter', nargs='*', default=[],
                    help='run only gates whose file name contains one of these')
    ap.add_argument('--timeout', type=float, default=1800,
                    help='seconds before a gate is killed (default 1800)')
    ap.add_argument('--log-dir', default=None,
                    help='where each gate\'s output goes (default: a temp dir)')
    args = ap.parse_args()

    paths = gates(args.filter)
    if not paths:
        print('no gate matches')
        return 1
    log_dir = args.log_dir or tempfile.mkdtemp(prefix='gui_parity_')
    os.makedirs(log_dir, exist_ok=True)

    passed, skipped, failed = [], [], []
    for path in paths:
        name = os.path.basename(path)[:-3]
        rc, secs = run_one(path, args.timeout, os.path.join(log_dir, name + '.log'))
        if rc == 0:
            verdict = 'PASS'
            passed.append(name)
        elif rc == 77:
            verdict = 'SKIP'
            skipped.append(name)
        else:
            verdict = 'TIMEOUT' if rc is None else f'FAIL rc={rc}'
            failed.append((name, verdict))
        print(f'  {verdict:<12} {name}  ({secs:.0f}s)', flush=True)

    print(f'\n{len(passed)} passed, {len(failed)} failed, {len(skipped)} '
          f'declared SKIP (asserted nothing) of {len(paths)}; logs: {log_dir}')
    for name, verdict in failed:
        print(f'  {verdict}: {name}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
