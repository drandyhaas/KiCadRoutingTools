#!/usr/bin/env python3
"""
Pass/fail test for differential-pair routing correctness.

Routes individual differential pairs with `route_diff.py` and asserts each is
**fully connected** and **DRC-clean** (scoped to the pair). Unlike the big
`test_fanout_and_route.py` pipeline, this is a small, fast, focused regression
guard for the diff-pair router on its own: a routed pair must actually connect
its two endpoints without introducing clearance violations.

Board: `kicad_files/routed_output.kicad_pcb` — the post-fanout board that feeds
the LVDS differential stage of `test_fanout_and_route.py`. Each
`/fpga_adc/lvds_rx1_NN` pair is a clean 2-pad point-to-point on F.Cu.

Note: the board has pre-existing DRC violations among OTHER already-routed nets,
so DRC is scoped to each pair's own nets (`--nets "*lvds_rx1_NN*"`), matching how
`test_fanout_and_route.py` checks the LVDS stage (`--nets "*lvds*"`).

Run:
    python3 tests/test_diff_pair_route.py        # uses KiCad's python (needs the Rust router)
    python3 tests/test_diff_pair_route.py -v      # verbose routing output
"""

import argparse
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

BOARD = os.path.join(ROOT_DIR, "kicad_files", "routed_output.kicad_pcb")
# Same geometry as test_fanout_and_route.py's LVDS diff stage.
GEOM = ["--track-width", "0.1", "--clearance", "0.1", "--via-size", "0.3", "--via-drill", "0.2",
        "--layers", "F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "B.Cu",
        "--impedance", "100", "--proximity-heuristic-factor", "0.0"]
CLEARANCE = "0.1"
# Pairs confirmed to route fully-connected + DRC-clean in isolation on this board.
# (lvds_rx1_14 does NOT route alone, so it's intentionally excluded.)
PAIRS = ["lvds_rx1_11", "lvds_rx1_10", "lvds_rx1_12"]


def route_with_nets(out, nets):
    """Route with explicit --nets args *nets*; return (routed_ok, output)."""
    cmd = [sys.executable, "route_diff.py", BOARD, out, "--nets", *nets] + GEOM
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    return ('"successful": 1' in txt and '"failed": 0' in txt), txt


def route_pair(pattern, out):
    """Route the single pair matching *pattern*; return (routed_ok, output)."""
    return route_with_nets(out, [f"*{pattern}*"])


def is_connected(board, pattern):
    """True if check_connected reports the pair's nets fully connected."""
    cmd = [sys.executable, "check_connected.py", board, "--nets", f"*{pattern}*"]
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return "ALL NETS FULLY CONNECTED" in (r.stdout + r.stderr)


def drc_clean(board, pattern):
    """True if the pair has no DRC violations (scoped to its own nets)."""
    cmd = [sys.executable, "check_drc.py", board, "--clearance", CLEARANCE,
           "--nets", f"*{pattern}*", "--clearance-margin", "0.1"]
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    return "NO DRC VIOLATIONS" in (r.stdout + r.stderr)


def scenario_pair(pattern, verbose):
    log = []
    fd, out = tempfile.mkstemp(suffix=".kicad_pcb", prefix=f"diffroute_{pattern}_")
    os.close(fd)
    try:
        routed, txt = route_pair(pattern, out)
        if not routed or not os.path.exists(out):
            return f"DP {pattern}", False, ["route_diff did not report 1/1 routed"]
        conn = is_connected(out, pattern)
        clean = drc_clean(out, pattern)
        log.append(f"routed=1/1  connected={conn}  drc_clean(pair-scoped)={clean}")
        passed = routed and conn and clean
        return f"DP /fpga_adc/{pattern} routes connected + DRC-clean", passed, log
    finally:
        if os.path.exists(out):
            os.remove(out)


def scenario_selector(label, nets, pattern, verbose):
    """Route a pair selected via *nets* (a one-sided glob or explicit base
    name), then assert BOTH halves connect -- regression for issue #120 where
    such selectors silently matched 0 pairs on hierarchical net names."""
    log = []
    fd, out = tempfile.mkstemp(suffix=".kicad_pcb", prefix="diffroute_sel_")
    os.close(fd)
    try:
        routed, txt = route_with_nets(out, nets)
        if not routed or not os.path.exists(out):
            return label, False, [f"route_diff did not report 1/1 routed for --nets {nets}"]
        conn = is_connected(out, pattern)
        log.append(f"--nets {nets}  routed=1/1  connected={conn}")
        return label, (routed and conn), log
    finally:
        if os.path.exists(out):
            os.remove(out)


def main():
    parser = argparse.ArgumentParser(description="Differential-pair routing correctness test")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose routing output")
    args = parser.parse_args()

    if not os.path.exists(BOARD):
        print(f"ERROR: board not found: {BOARD}")
        return 2

    print("=" * 70)
    print("Differential-pair routing correctness test")
    print("=" * 70)

    results = []
    for pattern in PAIRS:
        name, passed, log = scenario_pair(pattern, args.verbose)
        status = "PASS" if passed else "FAIL"
        print(f"\n[{status}] {name}")
        for line in log:
            print(f"        {line}")
        results.append((name, passed))

    # issue #120: a one-sided '*_P' glob and an explicit hierarchical base name
    # must each select the whole pair (and route both halves).
    selectors = [
        ("issue#120 one-sided glob '*lvds_rx1_11_P' routes full pair",
         ["*lvds_rx1_11_P"], "lvds_rx1_11"),
        ("issue#120 explicit base name '/fpga_adc/lvds_rx1_10' routes full pair",
         ["/fpga_adc/lvds_rx1_10"], "lvds_rx1_10"),
    ]
    for label, nets, pattern in selectors:
        name, passed, log = scenario_selector(label, nets, pattern, args.verbose)
        status = "PASS" if passed else "FAIL"
        print(f"\n[{status}] {name}")
        for line in log:
            print(f"        {line}")
        results.append((name, passed))

    print("\n" + "=" * 70)
    n_pass = sum(1 for _, p in results if p)
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    print(f"\n{n_pass}/{len(results)} pairs passed")
    print("=" * 70)
    return 0 if n_pass == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
