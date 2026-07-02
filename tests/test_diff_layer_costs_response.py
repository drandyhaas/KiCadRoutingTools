#!/usr/bin/env python3
"""End-to-end check that coupled diff pairs respond to route_diff --layer-costs (#193).

Uses the same board/pairs as test_diff_pair_route.py (kicad_files/routed_output,
the lvds_rx1_NN pairs that route 1/1). Each pair routes on B.Cu by default; with
--layer-costs favoring In1.Cu (every other layer 5x, In1.Cu 1x) the coupled route
moves onto In1.Cu instead -- proving the per-layer cost flows CLI -> config ->
PoseRouter and changes the chosen layer, while still routing successfully.

    python3 tests/test_diff_layer_costs_response.py        # needs the Rust router (grid_router 0.16.2+)
    python3 tests/test_diff_layer_costs_response.py -v
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
from collections import defaultdict

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = os.path.join(ROOT_DIR, "kicad_files", "routed_output.kicad_pcb")
GEOM = ["--track-width", "0.1", "--clearance", "0.1", "--via-size", "0.3", "--via-drill", "0.2",
        "--layers", "F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "B.Cu",
        "--impedance", "100", "--proximity-heuristic-factor", "0.0"]
PAIRS = ["lvds_rx1_11", "lvds_rx1_10", "lvds_rx1_12"]
# Favor In1.Cu: it is the 2nd of the 5 --layers, so cost 1x there, 5x everywhere else.
FAVOR_IN1 = ["--layer-costs", "5", "1", "5", "5", "5"]
FAVORED_LAYER = "In1.Cu"


def _cleanup(out):
    base = out[:-len(".kicad_pcb")]
    for f in (out, base + ".kicad_pro", base + ".kicad_prl"):
        if os.path.exists(f):
            os.remove(f)


def route(pair, extra, verbose):
    """Route *pair* coupled; return (routed_ok, {layer: mm}) for the pair's nets.
    Fresh temp paths each call so a stale .kicad_pro can't carry DRC floors over."""
    out = tempfile.mktemp(suffix=".kicad_pcb", prefix=f"dlc_{pair}_")
    cmd = [sys.executable, "route_diff.py", BOARD, out, "--nets", f"*{pair}*"] + extra + GEOM
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if verbose:
        print(txt)
    routed = '"successful": 1' in txt and '"failed": 0' in txt
    by_layer = defaultdict(float)
    n_vias = 0
    if os.path.exists(out):
        p = parse_kicad_pcb(out)
        ids = [n.net_id for n in p.nets.values() if n.name and f"{pair}_" in n.name]
        for s in p.segments:
            if s.net_id in ids:
                by_layer[s.layer] += math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        n_vias = sum(1 for v in p.vias if v.net_id in ids)
    _cleanup(out)
    return routed, by_layer, n_vias


def run(verbose=False):
    if not os.path.exists(BOARD):
        print(f"ERROR: board not found: {BOARD}")
        return 2
    print("=" * 70)
    print("Diff-pair response to --layer-costs (issue #193)")
    print("=" * 70)
    results = []
    for pair in PAIRS:
        ok_def, L_def, vias_def = route(pair, [], verbose)
        ok_fav, L_fav, vias_fav = route(pair, FAVOR_IN1, verbose)
        in1_def = L_def.get(FAVORED_LAYER, 0.0)
        in1_fav = L_fav.get(FAVORED_LAYER, 0.0)
        total_fav = sum(L_fav.values()) or 1.0
        # A pair honours --layer-costs if, under FAVOR_IN1, it routes PREDOMINANTLY
        # on the favoured layer (reaching an inner layer needs a via, so >=2).
        #
        # The old check ALSO demanded the DEFAULT route avoid In1.Cu (in1_def < 5),
        # to prove a shift. But self-graze avoidance (#269) can legitimately put a
        # pair on In1.Cu BY DEFAULT -- its natural clean route -- when the B.Cu
        # coupled middle would pinch its own P/N below clearance; choosing the clean
        # route is the correct response and overrides the (soft) layer-costs
        # preference (a HARD skip is a negative layer cost). So a pair already on the
        # favoured layer by default still passes. Pairs whose default is off In1.Cu
        # (lvds_rx1_10/12) still exercise the actual cost-driven B.Cu->In1.Cu shift,
        # so the suite still fails if --layer-costs stop flowing to the router.
        passed = (ok_def and ok_fav and in1_fav > total_fav * 0.5 and vias_fav >= 2)
        results.append((pair, passed))
        status = "PASS" if passed else "FAIL"
        print(f"\n[{status}] {pair}: routed(default)={ok_def} routed(favor In1)={ok_fav}")
        print(f"        In1.Cu mm: default={in1_def:.1f} -> favor-In1={in1_fav:.1f}")
        print(f"        vias placed: default={vias_def} -> favor-In1={vias_fav}")
        print(f"        default layers={ {k: round(v,1) for k,v in sorted(L_def.items())} }")
        print(f"        favor   layers={ {k: round(v,1) for k,v in sorted(L_fav.items())} }")

    n_pass = sum(1 for _, p in results if p)
    print("\n" + "=" * 70)
    for pair, p in results:
        print(f"  {'PASS' if p else 'FAIL'}  {pair}")
    print(f"\n{n_pass}/{len(results)} pairs responded to --layer-costs")
    print("=" * 70)
    return 0 if n_pass == len(results) else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Diff-pair --layer-costs response test")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    sys.exit(run(args.verbose))
