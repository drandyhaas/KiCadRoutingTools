#!/usr/bin/env python3
"""Final stress-test step: compare OUR routed board against the ORIGINAL
(downloaded, human-routed) board, and emit actionable suggestions about our
routing geometry and approach.

The original routing is ground truth for "what produces a manufacturable,
DRC-clean board". This compares copper usage, via count, track-width strategy,
layer balance and clearance, then prints suggestions + a JSON blob suitable for
the `comparison` / `suggestions` fields of the results JSON.

Usage:
  python3 compare_to_original.py --ours <our_final.kicad_pcb> \
        --orig <boards/<board>.kicad_pcb> [--clearance <netclass Default cl>] [--json]
"""
import sys
import os
import json
import argparse
import math
from collections import Counter, defaultdict

_here = os.path.dirname(os.path.abspath(__file__))
for _cand in (os.path.abspath(os.path.join(_here, "..", "..")),
              os.path.expanduser("~/Documents/KiCadRoutingTools")):
    if os.path.exists(os.path.join(_cand, "kicad_parser.py")):
        sys.path.insert(0, _cand)
        break
from kicad_parser import parse_kicad_pcb
from list_nets import read_design_rules


def seg_len(s):
    return math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)


def profile(path):
    pcb = parse_kicad_pcb(path)
    length_by_layer = defaultdict(float)
    width_count = Counter()
    seg_nets = set()
    for s in pcb.segments:
        length_by_layer[s.layer] += seg_len(s)
        width_count[round(s.width, 3)] += 1
        if s.net_id:
            seg_nets.add(s.net_id)
    via_count = Counter()
    for v in pcb.vias:
        via_count[(round(v.size, 3), round(v.drill, 3))] += 1
    return {
        "layers": list(pcb.board_info.copper_layers),
        "n_segments": len(pcb.segments),
        "n_vias": len(pcb.vias),
        "total_length_mm": round(sum(length_by_layer.values()), 1),
        "length_by_layer": {k: round(v, 1) for k, v in length_by_layer.items()},
        "width_count": dict(width_count),
        "via_count": {f"{s}/{d}": c for (s, d), c in via_count.items()},
        "distinct_widths": len(width_count),
        "nets_with_copper": len(seg_nets),
        "n_nets": len(pcb.nets),
    }


def design_clearance(orig_path):
    # The manufacturing floor DRC is actually graded at (#111), not the inflated
    # net-class clearance. Mirrors list_nets.effective_floors / the RUNBOOK.
    dr = read_design_rules(orig_path)
    if dr and dr.get("effective"):
        return dr["effective"].get("drc_clearance")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ours", required=True)
    ap.add_argument("--orig", required=True)
    ap.add_argument("--clearance", type=float, default=None,
                    help="clearance to note (defaults to orig manufacturing floor / DRC grade clearance)")
    ap.add_argument("--json", action="store_true", help="also print a JSON blob")
    args = ap.parse_args()

    cl = args.clearance if args.clearance is not None else design_clearance(args.orig)
    ours = profile(args.ours)
    orig = profile(args.orig)

    def ratio(a, b):
        return round(a / b, 2) if b else None

    suggestions = []

    # 1. Via usage
    if orig["n_vias"] and ours["n_vias"] > 1.3 * orig["n_vias"]:
        suggestions.append(
            f"VIAS: we placed {ours['n_vias']} vias vs original {orig['n_vias']} "
            f"({ratio(ours['n_vias'], orig['n_vias'])}x). Excess layer changes — consider "
            f"stronger single-layer escape / planning to reduce via count.")
    elif orig["n_vias"]:
        suggestions.append(
            f"VIAS: {ours['n_vias']} vs original {orig['n_vias']} ({ratio(ours['n_vias'], orig['n_vias'])}x) — comparable.")

    # 2. Total copper length (detour / efficiency)
    if orig["total_length_mm"] and ours["total_length_mm"] > 1.25 * orig["total_length_mm"]:
        suggestions.append(
            f"LENGTH: our total track length {ours['total_length_mm']}mm vs original "
            f"{orig['total_length_mm']}mm ({ratio(ours['total_length_mm'], orig['total_length_mm'])}x) — "
            f"we detour more; tighter heuristic / less ripup churn could shorten routes.")

    # 3. Track-width strategy
    ours_widths = sorted(ours["width_count"], key=lambda w: -ours["width_count"][w])
    orig_widths = sorted(orig["width_count"], key=lambda w: -orig["width_count"][w])
    orig_max_w = max(orig["width_count"]) if orig["width_count"] else 0
    ours_max_w = max(ours["width_count"]) if ours["width_count"] else 0
    if ours["distinct_widths"] <= 2 and orig["distinct_widths"] >= 3:
        suggestions.append(
            f"WIDTHS: original uses {orig['distinct_widths']} track widths "
            f"(up to {orig_max_w}mm, likely power/high-current) but we use only "
            f"{ours['distinct_widths']} ({ours_widths[:3]}). We under-build power nets — "
            f"add per-net width (wide traces for power/GND-return) instead of one uniform width.")
    if orig_max_w >= 2 * ours_max_w and orig_max_w >= 0.8:
        suggestions.append(
            f"WIDTHS: original has traces up to {orig_max_w}mm wide vs our max {ours_max_w}mm — "
            f"high-current/power nets need widening (--power-nets with explicit widths).")

    # 4. Layer balance
    if len(orig["layers"]) == 2 and orig["length_by_layer"] and ours["length_by_layer"]:
        def bal(p):
            vals = list(p["length_by_layer"].values())
            return round(min(vals) / max(vals), 2) if len(vals) >= 2 and max(vals) else 1.0
        suggestions.append(
            f"LAYER BALANCE: ours {bal(ours)} vs original {bal(orig)} "
            f"(min/max layer length; nearer 1.0 = more balanced 2-layer usage).")

    # 5. Completion vs ground truth (original connects everything)
    if ours["nets_with_copper"] < orig["nets_with_copper"]:
        suggestions.append(
            f"COMPLETION: we put copper on {ours['nets_with_copper']} nets vs original "
            f"{orig['nets_with_copper']} — {orig['nets_with_copper'] - ours['nets_with_copper']} "
            f"net(s) the human routed that we did not.")

    print(f"\n{'='*78}\nCOMPARE  ours={os.path.basename(args.ours)}  vs  orig={os.path.basename(args.orig)}")
    print(f"  manufacturing floor (DRC grade clearance): {cl}")
    print(f"  {'metric':24s} {'OURS':>16s} {'ORIGINAL':>16s}")
    print(f"  {'layers':24s} {str(len(ours['layers'])):>16s} {str(len(orig['layers'])):>16s}")
    print(f"  {'segments':24s} {ours['n_segments']:>16d} {orig['n_segments']:>16d}")
    print(f"  {'vias':24s} {ours['n_vias']:>16d} {orig['n_vias']:>16d}")
    print(f"  {'total length mm':24s} {ours['total_length_mm']:>16.1f} {orig['total_length_mm']:>16.1f}")
    print(f"  {'distinct track widths':24s} {ours['distinct_widths']:>16d} {orig['distinct_widths']:>16d}")
    print(f"  {'nets with copper':24s} {ours['nets_with_copper']:>16d} {orig['nets_with_copper']:>16d}")
    print(f"  ours widths:   {dict(sorted(ours['width_count'].items()))}")
    print(f"  orig widths:   {dict(sorted(orig['width_count'].items()))}")
    print(f"  ours vias:     {ours['via_count']}")
    print(f"  orig vias:     {orig['via_count']}")
    print(f"\n  SUGGESTIONS:")
    for s in suggestions:
        print(f"   - {s}")

    if args.json:
        blob = {"design_clearance": cl, "ours": ours, "original": orig, "suggestions": suggestions}
        print("\nJSON_COMPARISON: " + json.dumps(blob))


if __name__ == "__main__":
    main()
