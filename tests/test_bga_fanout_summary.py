#!/usr/bin/env python3
"""Regression test for issue #122 part 1: bga_fanout emits a structured
JSON_SUMMARY so downstream tooling can detect dropped balls and retry at a
tighter clearance instead of scraping per-net FAILED lines.

    python3 tests/test_bga_fanout_summary.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad_files", "interf_u_unrouted_placed.kicad_pcb")


def run_fanout():
    out = tempfile.mktemp(suffix=".kicad_pcb")
    cmd = [sys.executable, "bga_fanout.py", BOARD, "--component", "U9",
           "--nets", "*", "!GND", "!VCC",
           "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
           "--track-width", "0.2", "--clearance", "0.2",
           "--via-size", "0.5", "--via-drill", "0.3", "--output", out]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    if os.path.exists(out):
        os.remove(out)
    return r.stdout + r.stderr


def main():
    print("=" * 60)
    print("Issue #122 bga_fanout JSON_SUMMARY regression test")
    print("=" * 60)
    txt = run_fanout()
    checks = []

    m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", txt)
    checks.append(("JSON_SUMMARY line present", m is not None))
    if m:
        s = json.loads(m.group(1))
        for key in ("component", "requested", "escaped", "failed",
                    "unescaped_nets", "skipped_nc", "clearance",
                    "track_width", "layers"):
            checks.append((f"summary has '{key}'", key in s))
        checks.append(("skipped_nc is an int >= 0",
                       isinstance(s.get("skipped_nc"), int) and s["skipped_nc"] >= 0))
        if all(k in s for k in ("requested", "escaped", "failed")):
            checks.append(("requested == escaped + failed",
                           s["requested"] == s["escaped"] + s["failed"]))
            checks.append(("failed == len(unescaped_nets)",
                           s["failed"] == len(s.get("unescaped_nets", []))))
            checks.append(("escaped > 0 (board fanned out)", s["escaped"] > 0))
            checks.append(("component is U9", s.get("component") == "U9"))

    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    n_pass = sum(1 for _, ok in checks if ok)
    print(f"\n{n_pass}/{len(checks)} checks passed")
    print("=" * 60)
    return 0 if n_pass == len(checks) and checks else 1


if __name__ == "__main__":
    sys.exit(main())
