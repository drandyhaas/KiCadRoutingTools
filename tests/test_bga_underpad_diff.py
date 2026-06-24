#!/usr/bin/env python3
"""Regression test for issue #182: coupled differential pairs in the dense BGA
under-pad escape.

  python3 tests/test_bga_underpad_diff.py [-v]

When a dense BGA can only escape via `--escape-method underpad` (the channel
router floors out), the under-pad escape used to route every ball single-ended -
including the two halves of a differential pair, on whatever inner layer/exit
each A* picked. route_diff then could not re-pair them ("no matching P/N
endpoints on same layer" / "no valid position at any setback"), so every pair
degraded to single-ended.

The fix escapes each pair COUPLED: broadside edge pairs run straight off the
boundary on F.Cu (no vias); stacked edge-row pairs escape END-ON on F.Cu (the
trail ball bulges through the pad gap, no vias); deeper stacked pairs escape
END-ON on an inner layer (via-in-pad, same layer for P and N) running under the
via-free edge rows; side-rim pairs take a coupled inner escape. Keeping the edge
pairs via-free is what lets the deeper pairs run underneath them.

This test runs the FPGA Z-pairs of `glasgow_revC` (BGA U30, the board from the
issue, 11x11 / 0.8mm pitch, 13 Z* IO pairs in the bottom-left corner) through:
  - the engine (generate_bga_fanout, escape_method='underpad', diff patterns):
    every signal escapes, every pair's P and N land on the SAME layer, and the
    escape is DRC-clean;
  - route_diff end-to-end: every Z-pair is coupled-routed (the real pickup test).
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from bga_fanout import generate_bga_fanout
from bga_fanout.diff_pair import find_differential_pairs

BOARD = os.path.join(ROOT, "kicad_files", "glasgow_revC.kicad_pcb")
COMPONENT = "U30"
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
PARAMS = dict(track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2,
              diff_pair_gap=0.101)


def _run(args, verbose):
    r = subprocess.run([sys.executable, *args], cwd=ROOT,
                       capture_output=True, text=True)
    if verbose:
        print(r.stdout + r.stderr)
    return r.stdout + r.stderr


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    verbose = args.verbose

    print("=" * 60)
    print("Issue #182 coupled-diff under-pad escape regression test")
    print("=" * 60)
    if not os.path.exists(BOARD):
        print(f"  [SKIP] board not present: {BOARD}")
        return 0

    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints[COMPONENT]
    pairs = find_differential_pairs(fp, ["*"])
    npairs = len(pairs)

    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    # --- Engine level -------------------------------------------------------
    tracks, vias, _vrem, failed = generate_bga_fanout(
        fp, pcb, layers=LAYERS, escape_method="underpad",
        diff_pair_patterns=["*"], **PARAMS)
    check("all signal balls escape (0 failed)", len(failed) == 0,
          "" if not failed else f"failed: {failed}")

    # Every diff pair's P and N must land on the same single layer (coupled),
    # otherwise route_diff cannot pick the pair up.
    seg_layers = {}
    for t in tracks:
        seg_layers.setdefault(t["net_id"], set()).add(t["layer"])
    same_layer = 0
    for pair in pairs.values():
        pl = seg_layers.get(pair.p_pad.net_id, set())
        nl = seg_layers.get(pair.n_pad.net_id, set())
        if pl and pl == nl and len(pl) == 1:
            same_layer += 1
    check(f"all {npairs} diff pairs escape coupled (P/N on one shared layer)",
          same_layer == npairs, f"{same_layer}/{npairs} same-layer")

    # --- End-to-end: route_diff must couple every Z-pair --------------------
    tmp = []

    def _tmp(prefix):
        fd, p = tempfile.mkstemp(suffix=".kicad_pcb", prefix=prefix)
        os.close(fd)
        tmp.append(p)
        return p

    try:
        fan = _tmp("glasgow_fan_")
        _run(["bga_fanout.py", BOARD, "--component", COMPONENT,
              "--escape-method", "underpad", "--diff-pairs", "*",
              "--track-width", "0.12", "--via-size", "0.35", "--via-drill", "0.2",
              "--clearance", "0.1", "--diff-pair-gap", "0.101",
              "--layers", *LAYERS, "--output", fan], verbose)

        # Escape itself is DRC-clean (track/track, via/track) among the routed nets.
        drc = _run(["check_drc.py", fan, "--clearance", "0.1"], verbose)
        check("under-pad escape is DRC-clean", "NO DRC VIOLATIONS" in drc)

        nets = []
        for base in pairs:
            short = base.split("/")[-1].replace("IO_Banks/", "")
            nets += [f"/IO_Banks/{short}_P", f"/IO_Banks/{short}_N"]
        routed = _tmp("glasgow_rd_")
        out = _run(["route_diff.py", fan, routed, "--nets", *nets,
                    "--track-width", "0.12", "--diff-pair-gap", "0.101",
                    "--clearance", "0.1", "--layers", *LAYERS, "--no-gnd-vias"],
                   verbose)
        ok = f"{npairs}/{npairs} routed" in out
        check(f"route_diff couples all {npairs} Z-pairs end-to-end", ok,
              "" if ok else "some pairs failed/deferred")
    finally:
        for p in tmp:
            if os.path.exists(p):
                os.remove(p)

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
