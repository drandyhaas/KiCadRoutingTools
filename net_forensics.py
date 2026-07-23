#!/usr/bin/env python3
"""Net blockage/connectivity forensics (#424 debug machinery).

For each requested net on a board, reports to stdout (and --log FILE):
  1. ISLANDS: connected components of the net's copper (segments/vias/pads
     joined by touching endpoints / pad proximity), which pads sit in each,
     island sizes and layers.
  2. GAPS: for the two largest islands, the closest approach between them
     (the unclosed MST edge, endpoint coordinates and distance).
  3. WALLS: the foreign-copper inventory around each gap endpoint within
     --radius (default 1.0mm): net name, item kind, layer, distance --
     sorted nearest-first, so the boxing copper is named per layer.
  4. HISTORY (--route-log): the net's routing/rip/restore trail grepped
     from a chain or step log (MST edge attempts, stuck iterations,
     Blocking obstacles, rip ladder events, rescue outcomes).

Usage:
  python3 net_forensics.py board.kicad_pcb --nets BT_PCM_SYNC USB_D_P \
      [--radius 1.0] [--route-log step11.log] [--log forensics.txt]

Born from the 2026-07-21 ottercast pocket forensics: every residual
failure traced to escape-stub tips boxed on all four layers by neighbors'
destination traffic; this tool automates that diagnosis.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from collections import defaultdict


def _components(pcb, net_id, tol=0.05):
    """Union-find over the net's segments/vias/pads by coincident points."""
    items = []
    for s in pcb.segments:
        if s.net_id == net_id:
            items.append(('seg', s, [(s.start_x, s.start_y), (s.end_x, s.end_y)]))
    for v in pcb.vias:
        if v.net_id == net_id:
            items.append(('via', v, [(v.x, v.y)]))
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.net_id == net_id:
                items.append(('pad', p, [(p.global_x, p.global_y)]))
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    pts = []
    for idx, (_, obj, ps) in enumerate(items):
        r = (max(obj.size_x, obj.size_y) / 2 if hasattr(obj, 'size_x')
             else getattr(obj, 'size', 0) / 2 or 0.06)
        for (x, y) in ps:
            pts.append((x, y, idx, r))
    for i in range(len(pts)):
        x1, y1, a, r1 = pts[i]
        for j in range(i + 1, len(pts)):
            x2, y2, b, r2 = pts[j]
            if a != b and math.hypot(x1 - x2, y1 - y2) <= max(tol, r1, r2):
                union(a, b)
    comps = defaultdict(list)
    for idx in range(len(items)):
        comps[find(idx)].append(items[idx])
    return sorted(comps.values(), key=len, reverse=True)


def _describe(pcb, net, radius, out):
    comps = _components(pcb, net.net_id)
    out(f"\n=== {net.name} (net {net.net_id}): {len(comps)} island(s) ===")
    for i, c in enumerate(comps):
        pads = [f"{o.component_ref}.{o.pad_number}" for k, o, _ in c if k == 'pad']
        layers = sorted({o.layer for k, o, _ in c if k == 'seg'})
        out(f"  island {i}: {sum(1 for k,_,_ in c if k=='seg')} segs, "
            f"{sum(1 for k,_,_ in c if k=='via')} vias, pads={pads} layers={layers}")
    if len(comps) < 2:
        out("  (fully connected)")
        return
    # gap between the two largest islands
    def points(c):
        for k, o, ps in c:
            for p in ps:
                yield p
    best = (9e9, None, None)
    for p1 in points(comps[0]):
        for p2 in points(comps[1]):
            d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
            if d < best[0]:
                best = (d, p1, p2)
    d, p1, p2 = best
    out(f"  GAP island0<->island1: {d:.2f}mm  from ({p1[0]:.2f},{p1[1]:.2f}) "
        f"to ({p2[0]:.2f},{p2[1]:.2f})")
    for label, (gx, gy) in (('island0-end', p1), ('island1-end', p2)):
        inv = defaultdict(list)
        for s in pcb.segments:
            if s.net_id == net.net_id:
                continue
            dd = min(math.hypot(s.start_x - gx, s.start_y - gy),
                     math.hypot(s.end_x - gx, s.end_y - gy),
                     math.hypot((s.start_x + s.end_x) / 2 - gx,
                                (s.start_y + s.end_y) / 2 - gy))
            if dd < radius:
                nm = pcb.nets[s.net_id].name if s.net_id in pcb.nets else str(s.net_id)
                inv[nm].append((round(dd, 2), f"seg {s.layer}"))
        for v in pcb.vias:
            if v.net_id != net.net_id and math.hypot(v.x - gx, v.y - gy) < radius:
                nm = pcb.nets[v.net_id].name if v.net_id in pcb.nets else str(v.net_id)
                inv[nm].append((round(math.hypot(v.x - gx, v.y - gy), 2), "via ALL"))
        for fp in pcb.footprints.values():
            for p in fp.pads:
                if p.net_id == net.net_id:
                    continue
                dd = math.hypot(p.global_x - gx, p.global_y - gy)
                if dd < radius:
                    nm = (pcb.nets[p.net_id].name if p.net_id in pcb.nets
                          else f"net{p.net_id}")
                    kind = 'PTH' if p.drill else p.layers[0] if p.layers else '?'
                    inv[nm].append((round(dd, 2),
                                    f"pad {fp.reference}.{p.pad_number} {kind}"))
        out(f"  WALL around {label} ({gx:.2f},{gy:.2f}), r={radius}mm:")
        for nm in sorted(inv, key=lambda n: min(x[0] for x in inv[n]))[:8]:
            entries = sorted(inv[nm])[:3]
            out(f"    {nm}: " + ", ".join(f"{k}@{d}mm" for d, k in entries))


def _history(net_name, log_path, out):
    pat = re.compile(r'\x1b\[[0-9;]*m')
    keep = re.compile(r'MST edge|stuck|Blocking obstacles|Ripping|Ripped|'
                      r'restor|RESTORE|RELOCATION|rescue|Retry|FAILED|abandon|'
                      r'Coverage:', re.I)
    out(f"\n--- history of {net_name} in {log_path} ---")
    n = 0
    try:
        with open(log_path, errors='replace') as f:
            in_section = False
            for line in f:
                line = pat.sub('', line.rstrip())
                if net_name in line:
                    in_section = True
                elif in_section and line.startswith(('[', '=')) and net_name not in line:
                    in_section = False
                if (net_name in line or in_section) and keep.search(line):
                    out(f"  {line.strip()[:150]}")
                    n += 1
                    if n > 60:
                        out("  ... (truncated)")
                        return
    except OSError as e:
        out(f"  (log unreadable: {e})")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board')
    ap.add_argument('--nets', nargs='+', required=True)
    ap.add_argument('--radius', type=float, default=1.0)
    ap.add_argument('--route-log', action='append', default=[])
    ap.add_argument('--log', help='also append the report to this file')
    args = ap.parse_args()

    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(args.board)
    sink = open(args.log, 'a') if args.log else None

    def out(line):
        print(line)
        if sink:
            sink.write(line + '\n')

    out(f"# net_forensics: {args.board}")
    for nm in args.nets:
        net = next((x for x in pcb.nets.values() if x.name == nm), None)
        if net is None:
            out(f"\n=== {nm}: NOT FOUND ===")
            continue
        _describe(pcb, net, args.radius, out)
        for lp in args.route_log:
            _history(nm, lp, out)
    if sink:
        sink.close()


if __name__ == '__main__':
    main()
