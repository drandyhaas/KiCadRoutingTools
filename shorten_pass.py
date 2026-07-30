#!/usr/bin/env python3
"""Post-route path-quality shorten pass (gh#75).

The A* keeps whatever path it first found through then-congested space; once
the rest of the board settles, a detour (hairpin, horseshoe) may no longer
be necessary -- but nothing revisits it. This pass re-routes each net IN
ISOLATION (all other copper frozen) via route.py --rip-existing-nets (#71)
and keeps the result only if the net's total copper length got strictly
shorter by --min-gain-mm. Worst case a net stays as it was.

Usage:
    python3 shorten_pass.py board.kicad_pcb --output out.kicad_pcb \
        --layers F.Cu In2.Cu In3.Cu B.Cu [--nets N ...] \
        [--min-gain-mm 1.0] [--route-args "--grid-step 0.05 ..."]
"""
from __future__ import annotations

import argparse
import math
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from typing import Dict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "rust_router"))

from kicad_parser import parse_kicad_pcb

HERE = os.path.dirname(os.path.abspath(__file__))


def net_lengths(board: str) -> Dict[str, float]:
    pcb = parse_kicad_pcb(board)
    out: Dict[str, float] = {}
    for s in pcb.segments:
        n = pcb.nets.get(s.net_id)
        if not n or not n.name:
            continue
        out[n.name] = out.get(n.name, 0.0) + math.hypot(
            s.end_x - s.start_x, s.end_y - s.start_y)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("board")
    ap.add_argument("--output", required=True)
    ap.add_argument("--layers", nargs="+", required=True)
    ap.add_argument("--nets", nargs="*", default=None,
                    help="nets to consider (default: every routed net)")
    ap.add_argument("--exclude-nets", nargs="*", default=[])
    ap.add_argument("--min-gain-mm", type=float, default=1.0)
    ap.add_argument("--route-args", default="",
                    help="extra args passed through to route.py")
    args = ap.parse_args()

    work = args.output
    shutil.copyfile(args.board, work)
    pro_src = os.path.splitext(args.board)[0] + ".kicad_pro"
    pro_work = os.path.splitext(work)[0] + ".kicad_pro"
    if os.path.exists(pro_src) and pro_src != pro_work:
        shutil.copyfile(pro_src, pro_work)

    lengths = net_lengths(work)
    pcb = parse_kicad_pcb(work)
    zone_nets = {pcb.nets[z.net_id].name for z in pcb.zones
                 if z.net_id in pcb.nets}
    candidates = sorted(
        (n for n in lengths
         if n not in zone_nets and n not in set(args.exclude_nets)
         and (args.nets is None or n in set(args.nets))),
        key=lambda n: -lengths[n])

    kept = 0
    total_gain = 0.0
    extra = shlex.split(args.route_args)
    for net in candidates:
        before = net_lengths(work).get(net, 0.0)
        if before <= args.min_gain_mm:
            continue
        with tempfile.NamedTemporaryFile(
                suffix=".kicad_pcb", delete=False) as tf:
            trial = tf.name
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, "route.py"), work,
             "--nets", net, "--rip-existing-nets", net,
             "--layers", *args.layers, "--output", trial] + extra,
            capture_output=True, text=True)
        ok = ('"failed": 0' in r.stdout and os.path.exists(trial)
              and os.path.getsize(trial) > 0)
        if ok:
            after = net_lengths(trial).get(net, 0.0)
            if after > 0 and before - after >= args.min_gain_mm:
                shutil.copyfile(trial, work)
                kept += 1
                total_gain += before - after
                print(f"  {net}: {before:.1f} -> {after:.1f}mm "
                      f"(-{before - after:.1f})")
        os.unlink(trial)
    print(f"shorten pass: improved {kept}/{len(candidates)} net(s), "
          f"-{total_gain:.1f}mm total")


if __name__ == "__main__":
    main()
