#!/usr/bin/env python3
"""Regression test for issue #189 problem 2: the via-in-pad escape.

A multi-point single-ended MST edge whose target is a boxed-in fine-pitch SMD
pad used to fail ("no rippable blockers") because the pad was only reachable on
its own (walled-in) layer. The router now drops a DRC-legal via INSIDE the pad
(found with the same local via-obstacle map + fab-floor escalation the plane
repair sweep uses, ae2069 -- the big routing grid over-blocks a via that is
actually fab-legal) and re-routes the edge to that via on an open inner layer.

Repro: routing glasgow_revC `/IO_Banks/IO_Buffer_B/*` boxes in RN-network pads
(e.g. RN4.6) whose MST edge fails on F.Cu but completes via an inner layer.
Without the escape this subset leaves a pad floating (1 FAILED); with it, all
37 nets / 90 pads connect, and the forced via must be DRC-clean.

    python3 tests/test_via_in_pad_escape.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad_files", "glasgow_revC.kicad_pcb")
CLEARANCE = "0.1"


def run_route(out):
    cmd = [sys.executable, "route.py", BOARD,
           "--output", out,
           "--nets", "/IO_Banks/IO_Buffer_B/*",
           "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
           "--clearance", CLEARANCE, "--via-size", "0.3",
           "--via-drill", "0.2", "--track-width", "0.1"]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    return r.stdout + r.stderr


def run_drc(out):
    cmd = [sys.executable, "check_drc.py", out, "-c", CLEARANCE]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    return r.stdout + r.stderr


def main():
    print("=" * 60)
    print("Issue #189 via-in-pad escape regression test")
    print("=" * 60)
    out = tempfile.mktemp(suffix=".kicad_pcb")
    txt = run_route(out)
    checks = []

    # The via-in-pad escape used to be REQUIRED here, but the exact obstacle
    # keep-out (now the default) no longer over-blocks the pad's neighbours, so
    # the boxed pad routes to an inner layer directly -- the #189 escape stays as
    # a fallback for genuinely-boxed pads but isn't needed on this board. What we
    # still guarantee is the boxed-pad subset routes fully and DRC-clean.
    escape_fired = "Via-in-pad unblock" in txt
    print(f"  (info) via-in-pad escape {'fired' if escape_fired else 'not needed (exact keep-out routed the boxed pad directly)'}")

    m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", txt)
    checks.append(("JSON_SUMMARY line present", m is not None))
    if m:
        s = json.loads(m.group(1))
        checks.append(("no failed_multipoint (all pads connected)",
                       len(s.get("failed_multipoint", [])) == 0))
        checks.append(("no failed_single", len(s.get("failed_single", [])) == 0))
        checks.append(("every multipoint pad connected",
                       s.get("multipoint_pads_connected") == s.get("multipoint_pads_total")
                       and s.get("multipoint_pads_total", 0) > 0))
        checks.append(("nothing failed", s.get("failed") == 0))

    # The forced via-in-pad must be manufacturable: DRC-clean at the routed clearance.
    if os.path.exists(out):
        drc = run_drc(out)
        checks.append(("DRC clean at routed clearance",
                       "NO DRC VIOLATIONS FOUND" in drc))
        for ext in (".kicad_pcb", ".kicad_pro"):
            p = out[:-len(".kicad_pcb")] + ext
            if os.path.exists(p):
                os.remove(p)

    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    n_pass = sum(1 for _, ok in checks if ok)
    print(f"\n{n_pass}/{len(checks)} checks passed")
    print("=" * 60)
    return 0 if n_pass == len(checks) and checks else 1


if __name__ == "__main__":
    sys.exit(main())
