#!/usr/bin/env python3
"""What B.Cu obstacles sit near a horizontal line segment?"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import topo_strings as ts  # noqa: E402

nm, x0_, x1_, y_ = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), \
    float(sys.argv[4])
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
nid = byname[nm][0]
obs = te.build_obstacles(pcb, nid, {nid}, 'B.Cu')
hits = set()
steps = int((x1_ - x0_) / 0.05)
for k in range(steps + 1):
    x = x0_ + k * 0.05
    for i in obs.near_discs((x, y_)):
        dx, dy, r, name = obs.discs[i]
        import math
        if math.hypot(x - dx, y_ - dy) < r + 0.05:
            hits.add((name, round(dx, 2), round(dy, 2), round(r, 2)))
    for (a, b, r, name) in obs.caps:
        if ts.seg_pt_dist(a, b, (x, y_)) < r + 0.05:
            hits.add((name, round(a[0], 2), round(a[1], 2), round(b[0], 2), round(b[1], 2), round(r, 2)))
for h in sorted(hits):
    print(h)
