#!/usr/bin/env python3
"""Curate the sweep's board portfolio by measured information-per-dollar.

    python3 curate_boards.py <results_snapshot_dir> [--min-arms 5]

Reads a downloaded results-volume snapshot (<dir>/<arm>/<set>__<board>.json),
scores every board on DISCRIMINATION (population stdev of the pad-verdict
proxy nets_incomplete+conn across arms -- a board where all arms tie carries
no decision information, whether it ties at 100% or in equal failure) per
CPU-hour, and writes board_value.json next to this script:

  screen   ~12 cheapest high-discrimination boards spanning regimes -- every
           arm runs these first (~1-2 CPU-h/arm); clearly-worse arms die here
  exclude  boards with ZERO spread at good arm coverage -- pure waste
  (everything else is the full set, implicitly)

Regenerate whenever a big sweep finishes: coverage grows, values sharpen.
The 2026-08-10 numbers (25-arm #584 sweep, 43 boards at >=3 arms): info/$
ranges 46 (eis) to 0 (five all-tie boards); the screen set costs ~$0.15/arm
vs ~$1/arm for the full 150.
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import os
import statistics
import sys
from pathlib import Path

SCREEN_SIZE = 12
SCREEN_MAX_COST_H = 0.35   # keep the screen CHEAP; monsters stay full-set-only


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("snapshot")
    ap.add_argument("--min-arms", type=int, default=5,
                    help="boards with fewer arms measured are left unjudged")
    a = ap.parse_args()

    by_board = collections.defaultdict(dict)
    for f in glob.glob(os.path.join(a.snapshot, "*", "*.json")):
        r = json.load(open(f))
        r = r[0] if isinstance(r, list) else r
        arm = os.path.basename(os.path.dirname(f))
        if r.get("chain_complete"):
            v = (r.get("nets_incomplete") or 0) + (r.get("conn") or 0)
            by_board[r["board"]][arm] = (v, r.get("total_seconds") or 0)

    scored = []
    for b, arms in by_board.items():
        if len(arms) < a.min_arms:
            continue
        vals = [v for v, _ in arms.values()]
        cost = statistics.mean(t for _, t in arms.values()) / 3600
        sd = statistics.pstdev(vals)
        entry = {"board": b, "arms": len(arms), "cost_h": round(cost, 3),
                 "spread": max(vals) - min(vals),
                 "stdev": round(sd, 2),
                 "info_per_h": round(sd / max(cost, 0.01), 1)}
        if "baseline" in arms:
            # prescreen picks want boards NOT perfectly routed at baseline:
            # verdict 0 leaves no room to see improvement and hides small
            # damage (Andy, after the hw prescreen tied 0-0-0-0-0).
            entry["baseline_verdict"] = arms["baseline"][0]
        scored.append(entry)
    scored.sort(key=lambda s: -s["info_per_h"])

    exclude = [s["board"] for s in scored if s["spread"] == 0]
    screen = [s["board"] for s in scored
              if s["spread"] > 0 and s["cost_h"] <= SCREEN_MAX_COST_H][:SCREEN_SIZE]
    out = {
        "generated_from": str(Path(a.snapshot).resolve()),
        "boards_scored": len(scored),
        "screen": screen,
        "exclude": exclude,
        "table": scored,
    }
    dest = Path(__file__).resolve().parent / "board_value.json"
    dest.write_text(json.dumps(out, indent=1))
    print(f"scored {len(scored)} boards -> {dest}")
    print(f"screen ({len(screen)}): {', '.join(screen)}")
    print(f"exclude ({len(exclude)}): {', '.join(exclude)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
