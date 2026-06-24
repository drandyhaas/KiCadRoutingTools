#!/usr/bin/env python3
"""Regression test for issue #189: route.py must NOT report a multipoint
single-ended net as routed (`routed_single`) when one of its MST edges failed
to route, silently leaving pads disconnected.

`routed_single` was built from `net_id in routed_results` -- which a partially
routed multipoint net is, even after a failed MST edge leaves a pad floating --
*before* the authoritative `check_net_connectivity` sweep. So a disconnected net
showed up in BOTH `routed_single` and `failed_multipoint` (a contradiction). It
must appear only in `failed_multipoint` (and the headline `failed` count), never
in `routed_single`.

Repro: routing the congested `/IO_Banks/*` block of glasgow_revC leaves a
multi-point net with an unroutable MST edge (a pad too boxed in even for the
via-in-pad escape). Routing is deterministic, so the failing net is stable.

    python3 tests/test_multipoint_failed_edge_reporting.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad_files", "glasgow_revC.kicad_pcb")


def run_route():
    out = tempfile.mktemp(suffix=".kicad_pcb")
    cmd = [sys.executable, "route.py", BOARD,
           "--output", out,
           "--nets", "/IO_Banks/*",
           "--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
           "--clearance", "0.1", "--via-size", "0.3",
           "--via-drill", "0.2", "--track-width", "0.1"]
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    for ext in (".kicad_pcb", ".kicad_pro"):
        p = out[:-len(".kicad_pcb")] + ext
        if os.path.exists(p):
            os.remove(p)
    return r.stdout + r.stderr


def main():
    print("=" * 60)
    print("Issue #189 multipoint failed-edge reporting regression test")
    print("=" * 60)
    txt = run_route()
    checks = []

    m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", txt)
    checks.append(("JSON_SUMMARY line present", m is not None))
    if m:
        s = json.loads(m.group(1))
        routed = set(s.get("routed_single", []))
        failed_mp = {x["net_name"] for x in s.get("failed_multipoint", [])}

        # The whole point: this run must actually exercise a failed MST edge,
        # otherwise the test proves nothing.
        checks.append(("run produced at least one failed-edge multipoint net",
                       len(failed_mp) > 0))

        # Issue #189: a net with disconnected pads must never be claimed routed.
        overlap = routed & failed_mp
        checks.append(("routed_single is disjoint from failed_multipoint",
                       not overlap))
        if overlap:
            print(f"    contradiction: {sorted(overlap)} reported BOTH routed and failed")

        # Headline accounting must agree: failed >= number of disconnected nets,
        # and routed_single nets are all in the success set.
        checks.append(("failed count >= disconnected multipoint nets",
                       s.get("failed", 0) >= len(failed_mp)))
        checks.append(("successful == len(routed_single)",
                       s.get("successful") == len(routed)))

    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    n_pass = sum(1 for _, ok in checks if ok)
    print(f"\n{n_pass}/{len(checks)} checks passed")
    print("=" * 60)
    return 0 if n_pass == len(checks) and checks else 1


if __name__ == "__main__":
    sys.exit(main())
