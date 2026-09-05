#!/usr/bin/env python3
"""#530: qfn_fanout's --width is this run's track-width request, so a board
minimum ABOVE it is stale for the run (drop_stale_board_floors), exactly as
route.py treats --track-width. Before the alias in qfn_fanout.main neither
the stale rule nor enforce_fab_floors saw `--width`, and under the default
--escalation board a stock 0.2 mm min_track_width pinned 0.1 mm escape stubs
up to 0.2 (the GUI half of the same bug fed the policy the Basic tab's width
instead of the QFN panel's; test_fanout_rotated_gui covers that front).

Stages haasoscope_pro_max_test (U2, QFN-76) with a .kicad_pro declaring
min_track_width 0.2:

  --width 0.1  -> the stale-minimum line is printed, NO 'clamping escape
                  stubs up' line, every emitted stub is 0.1 wide
  --width 0.25 -> no stale line, stubs 0.25 (negative control: a request
                  above the minimum leaves the minimum alone)
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
SEG_W = re.compile(r'\(segment\b.*?\(width ([0-9.]+)\)', re.S)


def _run(board, out, width):
    # --escalation board explicitly: the stale-minimum rule is the BOARD
    # policy's (under the default `fab` there is no board floor to drop).
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'qfn_fanout.py'), board,
                        '-o', out, '-c', 'U2', '-w', str(width),
                        '--escalation', 'board'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    widths = sorted({float(w) for w in SEG_W.findall(open(out).read())}) \
        if os.path.exists(out) else []
    return r, widths


def main():
    if not os.path.exists(BOARD):
        print(f"SKIP: {os.path.relpath(BOARD, ROOT)} not found")
        return 0
    fails = []
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'h.kicad_pcb')
        shutil.copy(BOARD, board)
        with open(os.path.join(td, 'h.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump({"board": {"design_settings": {"rules": {"min_track_width": 0.2}}},
                       "meta": {"filename": "h.kicad_pro", "version": 3}}, f)
        # the input board's widths, so the assertion is about the STUBS only
        base = {float(w) for w in SEG_W.findall(open(board).read())}

        r, widths = _run(board, os.path.join(td, 'o1.kicad_pcb'), 0.1)
        new = [w for w in widths if w not in base]
        if r.returncode != 0:
            fails.append(f"--width 0.1 exited {r.returncode}:\n{r.stdout[-1500:]}\n{r.stderr[-800:]}")
        if 'treating that minimum as stale' not in r.stdout:
            fails.append("--width 0.1 below the declared 0.2 did not announce the stale minimum")
        if 'clamping escape stubs up' in r.stdout:
            fails.append("--width 0.1 was clamped up to the board minimum")
        if new != [0.1]:
            fails.append(f"--width 0.1 emitted stub widths {new}, expected [0.1]")

        r2, widths2 = _run(board, os.path.join(td, 'o2.kicad_pcb'), 0.25)
        new2 = [w for w in widths2 if w not in base]
        if 'treating that minimum as stale' in r2.stdout:
            fails.append("--width 0.25 above the declared 0.2 still dropped the minimum")
        if new2 != [0.25]:
            fails.append(f"--width 0.25 emitted stub widths {new2}, expected [0.25]")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: qfn_fanout --width takes part in the stale-board-minimum rule; "
          "0.1 mm stubs under a stock 0.2 mm minimum stay 0.1, and 0.25 stays 0.25")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
