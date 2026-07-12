#!/usr/bin/env python3
"""Issue #129 + #367: escape priority for multi-ball nets, coverage-gated.

Escape channels are the scarce resource on a dense BGA (#122), and a net only
NEEDS one escape. But reshuffling the escape competition on a board where the
legacy single pass already escapes everything only butterflies the downstream
chain (#367: ottercast_audio +10 disconnected nets). So the LEGACY SINGLE
PASS always runs first and is kept verbatim when it drops nothing; the
escape-priority orchestration (one ball per net first, extras second, strap
fallback) is strictly a RESCUE, kept only when it strictly reduces dropped
balls. No CLI options.

Two checks:
  1. interf_u U9 (uncontended PGA, multi-ball VCC/12V): single pass escapes
     everything -> NO priority machinery runs, output is legacy.
  2. ulx3s U1 with GND fanned (contended): single pass drops balls -> the
     priority rescue runs and wins (dropped -> 0). Skips if the corpus board
     is absent.

    python3 tests/test_bga_escape_priority.py
"""
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
IU_BOARD = os.path.join(ROOT, "kicad_files", "interf_u_unrouted_placed.kicad_pcb")
ULX_BOARD = os.path.expanduser(
    "~/Documents/kicad_stress_test/boards_unrouted_set2/ulx3s.kicad_pcb")


def run_fanout(board, args):
    out = tempfile.mktemp(suffix=".kicad_pcb")
    cmd = [sys.executable, "bga_fanout.py", board, "--output", out] + args
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if os.path.exists(out):
        os.remove(out)
    return txt


def main():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # --- 1. uncontended: single pass wins, no priority machinery ---
    txt = run_fanout(IU_BOARD, [
        "--component", "U9", "--nets", "*", "!GND",
        "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
        "--track-width", "0.2", "--clearance", "0.2",
        "--via-size", "0.5", "--via-drill", "0.3"])
    check("uncontended board: no priority machinery (legacy verbatim, #367)",
          "Escape priority" not in txt)
    m = re.search(r'"escaped": (\d+), "failed": (\d+)', txt)
    check("uncontended summary parses", m is not None)
    if m:
        check("uncontended: everything escapes", int(m.group(2)) == 0)

    # --- 2. contended: single pass drops balls -> priority rescue wins ---
    if not os.path.exists(ULX_BOARD):
        print("  SKIP: corpus board not present, rescue-path check skipped "
              f"({ULX_BOARD})")
    else:
        txt2 = run_fanout(ULX_BOARD, [
            "--component", "U1", "--nets", "*", "!+3V3", "!+1V1",
            "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
            "--escape-method", "underpad",
            "--via-size", "0.35", "--via-drill", "0.2",
            "--track-width", "0.12", "--clearance", "0.1"])
        m_drop = re.search(r"single pass dropped (\d+) ball", txt2)
        check("contended board: single pass drops balls", m_drop is not None)
        m_win = re.search(r"Escape priority wins: (\d+) -> (\d+) dropped", txt2)
        check("priority rescue runs and wins", m_win is not None)
        if m_win:
            check("rescue strictly reduces dropped balls",
                  int(m_win.group(2)) < int(m_win.group(1)),
                  f"{m_win.group(1)} -> {m_win.group(2)}")
        m2 = re.search(r'"escaped": (\d+), "failed": (\d+)', txt2)
        check("contended summary parses", m2 is not None)
        if m2:
            check("contended: full net coverage after rescue",
                  int(m2.group(2)) == 0, f"failed={m2.group(2)}")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
