#!/usr/bin/env python3
"""
Regression test for issue #383: ab_replay_grade's grading path used a
process-global redirect_stdout inside worker threads, so concurrent boards'
prints (result lines) were swallowed into each other's capture buffers; and a
worker BaseException killed the whole wave with no summary.

This test proves:
  1. run_drc(quiet=True, print_summary=False) returns violations and prints
     NOTHING (so run_check_drc needs no stdout redirect).
  2. Many run_check_drc calls concurrent with sibling threads that print do NOT
     lose the siblings' output (the race is gone).
  3. run_drc(quiet=True) still prints the OK/FAILED summary by default (the CLI
     / subprocess grade path is unchanged).

Run:
    python3 tests/test_383_grade_stdout_race.py
"""

import io
import os
import sys
import threading
from contextlib import redirect_stdout

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "tests", "stress"))

from check_drc import run_drc

# A clean, DRC-clean board from the reference wave (any small routed board works)
BOARD = os.path.expanduser(
    "~/Documents/kicad_stress_test/ab_265_0712/new_set1/urchin/urchin_repaired.kicad_pcb")


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    if not os.path.exists(BOARD):
        print(f"  SKIP: reference board missing ({BOARD})")
        return 0

    # -- 1: print_summary=False emits nothing --------------------------------
    print("1: run_drc(quiet=True, print_summary=False) is silent")
    buf = io.StringIO()
    with redirect_stdout(buf):
        v = run_drc(BOARD, quiet=True, print_summary=False)
    check("returns a violations list", isinstance(v, list))
    check("printed zero characters", buf.getvalue() == "")

    # -- 2: no cross-thread swallow under concurrency ------------------------
    print("\n2: concurrent run_check_drc does not steal sibling prints")
    from kicad_drc_compare import run_check_drc
    import time

    def grader():
        for _ in range(3):
            run_check_drc(BOARD)  # in-process; must NOT swap sys.stdout

    def printer(i):
        # a sibling worker's result-line print, repeated to widen the window
        for _ in range(5):
            print(f"SIBLING-{i}")
            time.sleep(0.001)

    threads = [threading.Thread(target=grader) for _ in range(4)]
    threads += [threading.Thread(target=printer, args=(i,)) for i in range(20)]

    # Swap stdout to a capture ONCE (this is the harness's real stdout during a
    # wave). If run_check_drc still did its own redirect_stdout, a grader would
    # transiently swap sys.stdout to its private buffer and steal whatever a
    # printer wrote in that window -> missing SIBLING lines here.
    cap = io.StringIO()
    old = sys.stdout
    sys.stdout = cap
    try:
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    finally:
        sys.stdout = old
    text = cap.getvalue()
    survived = sum(1 for i in range(20) if f"SIBLING-{i}" in text)
    check(f"all 20 sibling prints survived ({survived}/20)", survived == 20)

    # -- 3: default quiet summary unchanged ----------------------------------
    print("\n3: run_drc(quiet=True) still prints its summary (subprocess path)")
    buf = io.StringIO()
    with redirect_stdout(buf):
        run_drc(BOARD, quiet=True)
    out = buf.getvalue().strip()
    check("prints OK/FAILED summary by default", out.startswith("OK") or out.startswith("FAILED"))

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
