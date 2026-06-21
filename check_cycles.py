#!/usr/bin/env python3
"""check_cycles.py -- robustly detect redundant copper LOOPS (cycles) per net.

A routed signal net should be a TREE. Rip-reroute / failed-edge retry can re-add
same-net copper that closes loops (redundant cycles), producing the "crazy" wandering
traces and via stacks seen on dense boards. This reports the cyclomatic number
(number of independent loops = E - V + components) for each net, using a
geometry-correct connectivity graph:

  * tolerance endpoint matching (size-aware for vias/pads),
  * T-JUNCTION detection -- an endpoint that lands on another segment's interior
    splits that segment (this is the case a naive shared-endpoint graph misses),
  * via / through-hole-pad layer joins (a via connects all copper at its location).

Copper pour / zone nets (planes) are meshes by design, so they are listed
separately and not flagged.

Usage:
    python3 check_cycles.py board.kicad_pcb [--net NAME | --nets PATTERN]
                                            [--all] [--verbose]
"""
import argparse
import math
import fnmatch
import sys
from collections import defaultdict

from kicad_parser import parse_kicad_pcb
from check_connected import points_match, point_on_segment

TOL = 0.02  # mm endpoint coincidence tolerance (matches check_connected)


def _union_find(n):
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    return find, union


def net_cyclomatic(segs, vias, pads, tol=TOL):
    """Return (cyclomatic, components, edges, nodes) for one net's copper.

    cyclomatic = E - V + C is the number of independent loops; 0 means the net is
    a tree (or forest). Handles tolerance, vias/TH-pads (layer bridges) and
    T-junctions (endpoint on another segment's interior splits it)."""
    if not segs:
        return 0, 0, 0, 0

    # --- endpoint ports: one per segment end ---
    P = []  # (x, y, layer)
    seg_ep = []  # (start_port, end_port) per segment
    for s in segs:
        i = len(P); P.append((s.start_x, s.start_y, s.layer))
        j = len(P); P.append((s.end_x, s.end_y, s.layer))
        seg_ep.append((i, j))
    n = len(P)
    find, union = _union_find(n)

    # cluster coincident endpoints on the same layer
    for a in range(n):
        xa, ya, la = P[a]
        for b in range(a + 1, n):
            if P[b][2] == la and abs(P[b][0] - xa) < tol and abs(P[b][1] - ya) < tol:
                union(a, b)

    # vias / through-hole pads bridge layers: union every port within copper reach
    def join_near(cx, cy, reach):
        near = [i for i in range(n) if math.hypot(P[i][0] - cx, P[i][1] - cy) < reach]
        for k in near[1:]:
            union(near[0], k)
    for v in vias:
        join_near(v.x, v.y, max(getattr(v, 'size', 0.6) / 4.0, tol))
    for p in pads:
        if getattr(p, 'drill', 0) and p.drill > 0:
            join_near(p.global_x, p.global_y, max(max(p.size_x, p.size_y) / 4.0, tol))

    # cluster representative location (any member; coincident within tol) and the
    # set of copper layers the cluster occupies (its member endpoints' layers --
    # a via/TH-pad join merges endpoints from several layers into one cluster).
    reps = {}
    rep_layers = defaultdict(set)
    for i in range(n):
        r = find(i)
        reps.setdefault(r, P[i])
        rep_layers[r].add(P[i][2])

    # --- T-junctions: a cluster lying on a segment's interior splits it ---
    # Only a cluster that actually has copper on the segment's layer can be a real
    # T-junction; a cluster sitting on the line geometrically but only on another
    # layer (no via there) is NOT connected and must not split it.
    extra_splits = defaultdict(set)
    rep_items = list(reps.items())
    for si, s in enumerate(segs):
        ra, rb = find(seg_ep[si][0]), find(seg_ep[si][1])
        for r, (cx, cy, _cl) in rep_items:
            if r == ra or r == rb or s.layer not in rep_layers[r]:
                continue
            if point_on_segment(cx, cy, s.start_x, s.start_y, s.end_x, s.end_y, tol) \
               and not points_match(cx, cy, s.start_x, s.start_y, tol) \
               and not points_match(cx, cy, s.end_x, s.end_y, tol):
                extra_splits[si].add(r)

    # --- cyclomatic on the (split) graph ---
    roots = list(reps.keys())
    ridx = {r: k for k, r in enumerate(roots)}
    cfind, cunion = _union_find(len(roots))
    for si in range(len(segs)):
        ra, rb = find(seg_ep[si][0]), find(seg_ep[si][1])
        cunion(ridx[ra], ridx[rb])
        for r in extra_splits[si]:
            cunion(ridx[ra], ridx[r])  # split node sits on the segment
    V = len(roots)
    E = sum(1 + len(extra_splits[si]) for si in range(len(segs)))
    C = len(set(cfind(k) for k in range(len(roots))))
    return E - V + C, C, E, V


def overlapping_vias(vias):
    """Pairs of same-net vias whose copper PADS physically overlap (centre
    distance < sum of radii). A redundant via stack from rip-reroute / fanout
    shows up here; intentionally-spaced stitching vias (centres >= pad sum) do
    not."""
    out = []
    for i in range(len(vias)):
        for j in range(i + 1, len(vias)):
            d = math.hypot(vias[i].x - vias[j].x, vias[i].y - vias[j].y)
            if d < (vias[i].size + vias[j].size) / 2.0:
                out.append((vias[i], vias[j], d))
    return out


def main():
    ap = argparse.ArgumentParser(description="Robustly detect redundant copper loops (cycles) per net.")
    ap.add_argument("board", help="Path to the .kicad_pcb file")
    ap.add_argument("--net", help="Check a single net by exact name")
    ap.add_argument("--nets", help="Glob pattern of net names to check (e.g. 'RAM_*')")
    ap.add_argument("--all", action="store_true", help="Also list zoned/plane nets (meshes, normally hidden)")
    ap.add_argument("--verbose", action="store_true", help="Show via overlaps and per-net detail")
    args = ap.parse_args()

    pcb = parse_kicad_pcb(args.board)
    zoned = {z.net_id for z in (getattr(pcb, 'zones', []) or [])}
    segs_by_net = defaultdict(list)
    for s in pcb.segments:
        segs_by_net[s.net_id].append(s)

    rows = []
    for net_id, segs in segs_by_net.items():
        net = pcb.nets.get(net_id)
        name = net.name if net else f"net{net_id}"
        if args.net and name != args.net:
            continue
        if args.nets and not fnmatch.fnmatch(name, args.nets):
            continue
        is_zone = net_id in zoned
        if is_zone and not args.all and not args.net:
            continue
        vias = [v for v in pcb.vias if v.net_id == net_id]
        pads = pcb.pads_by_net.get(net_id, [])
        cyc, comps, E, V = net_cyclomatic(segs, vias, pads)
        ov = overlapping_vias(vias)
        rows.append((cyc, name, len(segs), len(vias), comps, len(ov), is_zone))

    rows.sort(key=lambda r: (-r[0], -r[5], r[1]))
    flagged = [r for r in rows if (r[0] > 0 or r[5] > 0) and not r[6]]

    print(f"{'net':28s} {'segs':>5} {'vias':>5} {'loops':>6} {'comp':>5} {'ovlVias':>8}")
    print("-" * 62)
    for cyc, name, ns, nv, comps, nov, is_zone in rows:
        if not args.verbose and cyc == 0 and nov == 0 and not args.net:
            continue
        tag = "  (mesh/plane)" if is_zone else ("  <== LOOPS" if cyc > 0 else "")
        if nov:
            tag += f"  <== {nov} OVERLAPPING VIA PAIR(S)"
        print(f"{name:28s} {ns:5d} {nv:5d} {cyc:6d} {comps:5d} {nov:8d}{tag}")

    print("-" * 62)
    print(f"signal nets with loops: {sum(1 for r in flagged if r[0] > 0)}; "
          f"total independent loops: {sum(r[0] for r in flagged)}; "
          f"nets with overlapping vias: {sum(1 for r in flagged if r[5] > 0)}")

    if args.verbose:
        for net_id, segs in segs_by_net.items():
            net = pcb.nets.get(net_id)
            name = net.name if net else f"net{net_id}"
            if args.net and name != args.net:
                continue
            vias = [v for v in pcb.vias if v.net_id == net_id]
            for a, b, d in overlapping_vias(vias):
                print(f"  {name}: vias ({a.x:.3f},{a.y:.3f}) & ({b.x:.3f},{b.y:.3f}) "
                      f"dist={d:.4f}mm size={a.size}/{b.size}")

    return 1 if flagged else 0


if __name__ == "__main__":
    sys.exit(main())
