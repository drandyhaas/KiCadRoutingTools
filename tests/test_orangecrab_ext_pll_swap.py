#!/usr/bin/env python3
"""Issue #168: route_diff's pre-routing layer swap must not graze the partner
polarity's single-ended test point.

On orangecrab the EXT_PLL+/EXT_PLL- pair breaks out to per-net test points
(EXT_PLL- -> TP20). route_diff's stub layer-swap optimisation moved the EXT_PLL+
fanout stubs onto B.Cu -- the same layer as TP20.1 -- because validate_swap
exempted BOTH of the pair's nets from its foreign-pad clearance check, so the
+stub grazing the -net's test-point pad was never caught. The swap then committed
sub-clearance copper (6 Pad<->Seg violations) even though the pair was deferred
to single-ended.

The fix checks each polarity's swapped stub against the OTHER polarity's pads
(only the stub's own net + a swap-partner pair stay exempt; the coupled body is
segments, not pads, so legitimate parallel running is unaffected). route_diff
must then ADD no different-net EXT_PLL+/- pad-clearance violations beyond what was
already in the fanout input.

Run:
    python3 tests/test_orangecrab_ext_pll_swap.py [-v]
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, "kicad_files", "orangecrab_ext_pll.kicad_pcb")
CLR = "0.0889"
GEOM = ["--diff-pair-gap", "0.25", "--track-width", "0.089", "--clearance", CLR,
        "--via-size", "0.3", "--via-drill", "0.15",
        "--layers", "F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "B.Cu",
        "--no-gnd-vias"]
# Different-net violations between the two EXT_PLL polarities (the bug).
VIOL = re.compile(r"EXT_PLL- \(TP20\.1\) <-> (?:Seg|Via):EXT_PLL\+")


def _run(args, verbose):
    r = subprocess.run([sys.executable, *args], cwd=ROOT, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if verbose:
        print(txt)
    return txt


def _ext_pll_violations(board, verbose):
    return len(VIOL.findall(_run(["check_drc.py", "-c", CLR, board], verbose)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    if not os.path.exists(BOARD):
        print(f"FAIL: board not found: {BOARD}")
        return 1

    fails = []
    before = _ext_pll_violations(BOARD, args.verbose)  # the pre-existing fanout via
    fd, out = tempfile.mkstemp(suffix=".kicad_pcb", prefix="oc_ext_pll_")
    os.close(fd)
    try:
        txt = _run(["route_diff.py", BOARD, "--nets", "EXT_PLL+", "EXT_PLL-",
                    *GEOM, "--output", out], args.verbose)
        # The harmful swap must be rejected (no layer swap applied here).
        m = re.search(r'"layer_swaps":\s*(\d+)', txt)
        if m and int(m.group(1)) != 0:
            fails.append(f"route_diff applied {m.group(1)} layer swap(s) that graze TP20 "
                         f"(should reject the swap onto B.Cu -- issue #168)")
        if not (os.path.exists(out) and os.path.getsize(out) > 0):
            fails.append("route_diff wrote no output")
        else:
            after = _ext_pll_violations(out, args.verbose)
            if after > before:
                fails.append(f"route_diff ADDED EXT_PLL+/- pad-clearance violations "
                             f"({before} -> {after}); the layer swap grazed TP20 (issue #168)")
    finally:
        if os.path.exists(out):
            os.remove(out)

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print(f"PASS: route_diff adds no EXT_PLL+/- pad-clearance violations "
          f"(stayed at {before}, the fanout via); harmful B.Cu layer swap rejected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
