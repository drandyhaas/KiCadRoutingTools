#!/usr/bin/env python3
"""Do the taut pre-routes find the split the hand-written rule declares?

Prints the detected bus clusters at a checkpoint beside the nets the
'south river' rule would have taken, so the two can be compared.

usage: probe_buses.py [K] [width] [thresh]
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import detect_buses as db  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '21'
WIDTH = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
THRESH = float(sys.argv[3]) if len(sys.argv) > 3 else 0.55

names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
comps = {ends[n][2] for n in names}
pads = [p for c in comps for p in pcb.footprints[c].pads]
rows = sorted({round(p.global_y, 3) for p in pads})
river = {n for n in names if ends[n][0][1] > rows[-1] + 0.8}

cache = {}


def obs_for(nm):
    nid = byname[nm][0]
    if nid not in cache:
        # a taut pre-route sees the STATIC world: pads, foreign copper.
        # Other bus nets are not obstacles -- the point is to find who
        # wants to go the same way.
        cache[nid] = te.build_obstacles(pcb, nid, kids, 'F.Cu')
    return cache[nid]


t0 = time.time()
paths = db.taut_paths(names, ends, obs_for)
t1 = time.time()
groups = db.cluster(names, paths, WIDTH, THRESH)
print(f'K={K}  width {WIDTH} mm  threshold {THRESH:.2f}   '
      f'[{t1 - t0:.1f}s pre-route, {time.time() - t1:.1f}s cluster]')
print(f'detected {len(groups)} bus(es):')
for g in groups:
    marked = ' '.join(('*' + n if n in river else n)
                      for n in sorted(g, key=lambda n: ends[n][0][1]))
    print(f'  [{len(g):2d}] {marked}')
print(f'\n(* = a net the hand-written rule sends to the separate '
      f'builder; {len(river)} of them)')
same = [g for g in groups if river and river.issubset(set(g))]
if len(groups) == 1:
    print('VERDICT: one bus -- detection does NOT reproduce the split')
elif any(set(g) == river for g in groups):
    print('VERDICT: a detected bus EXACTLY equals the hand-written river')
else:
    for g in groups:
        ov = len(set(g) & river)
        if ov:
            print(f'  overlap: a {len(g)}-net bus contains {ov} of the '
                  f'{len(river)} river nets')
