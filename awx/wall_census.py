#!/usr/bin/env python3
"""Which berth PADS are walled by other nets' stubs riding through the
array: for every run net's pad in DEST, on each layer, the other nets'
stub segments (longer than `minlen`, inside the array box) passing
within `reach` of the pad centre, by side (N/S/E/W of the pad). A pad
with rides on two opposite sides on a layer is SANDWICHED there.
usage: wall_census.py FO K [--dest DU1]"""
import argparse, os, sys, collections, math
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router')); sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb
import surgical as sg, topo_strings as ts
ap = argparse.ArgumentParser(); ap.add_argument('fo'); ap.add_argument('k')
ap.add_argument('--dest', default='DU1'); ap.add_argument('--reach', type=float, default=0.55)
ap.add_argument('--minlen', type=float, default=1.0); ap.add_argument('--only', default='')
a = ap.parse_args()
names = sg.k_nets(a.k).split(','); p = parse_kicad_pcb(a.fo)
byname = {n.name.split('/')[-1]: (i, n) for i, n in p.nets.items()}
fp = p.footprints[a.dest]
xs = [q.global_x for q in fp.pads]; ys = [q.global_y for q in fp.pads]
box = (min(xs) - 0.3, min(ys) - 0.3, max(xs) + 0.3, max(ys) + 0.3)
rides = []   # (net, layer, a, b)
for nm in names:
    nid = byname[nm][0]
    for s in p.segments:
        if s.net_id != nid: continue
        A, B = (s.start_x, s.start_y), (s.end_x, s.end_y)
        if not (box[0] <= A[0] <= box[2] and box[1] <= A[1] <= box[3]): continue
        if math.hypot(B[0]-A[0], B[1]-A[1]) >= a.minlen:
            rides.append((nm, s.layer, A, B))
print(f'{len(rides)} ride segment(s) >= {a.minlen} mm inside {a.dest}: '
      + ', '.join(f'{nm}/{L[0]}' for nm, L, _, _ in rides))
only = set(a.only.split(',')) if a.only else None
for nm in names:
    if only and nm not in only: continue
    nid = byname[nm][0]
    pads = [q for q in fp.pads if q.net_id == nid]
    if not pads: continue
    q = pads[0]; c = (q.global_x, q.global_y)
    walls = collections.defaultdict(set)
    for om, L, A, B in rides:
        if om == nm: continue
        d = ts.seg_pt_dist(A, B, c)
        if d < a.reach:
            # side of the pad the ride passes
            mx, my = (A[0]+B[0])/2, (A[1]+B[1])/2
            if abs(B[1]-A[1]) < abs(B[0]-A[0]):    # horizontal ride
                side = 'N' if my < c[1] else 'S'
            else:
                side = 'W' if mx < c[0] else 'E'
            walls[L[0]].add((om, side, round(d, 2)))
    if walls:
        flag = [L for L, w in walls.items() if {s for _, s, _ in w} >= {'N', 'S'} or {s for _, s, _ in w} >= {'E', 'W'}]
        print(f'  {nm:6s} pad ({c[0]:.1f},{c[1]:.1f}) ' + ('SANDWICHED on ' + ','.join(flag) + '  ' if flag else '') +
              '  '.join(f'{L}: ' + ', '.join(f'{om}{s}@{d}' for om, s, d in sorted(w)) for L, w in sorted(walls.items())))
