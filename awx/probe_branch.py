#!/usr/bin/env python3
"""Show a net's branch points and the segments meeting there, so a
'branch' can be judged: a real spur (wasted copper / an antenna) or a
legitimate T (e.g. a via feeding two directions)."""
import math
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402

SNAP = 0.005


def key(p):
    return (round(p[0] / SNAP), round(p[1] / SNAP))


board, NM = sys.argv[1], sys.argv[2]
pcb = parse_kicad_pcb(board)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
nid, net = byname[NM]
segs = [s for s in pcb.segments if s.net_id == nid]
vias = [v for v in pcb.vias if v.net_id == nid]
deg = defaultdict(list)
for s in segs:
    deg[key((s.start_x, s.start_y))].append(s)
    deg[key((s.end_x, s.end_y))].append(s)
vk = {key((v.x, v.y)) for v in vias}
pk = {key((p.global_x, p.global_y)) for p in net.pads}
print(f'{NM}: {len(segs)} segs, {len(vias)} vias')
for p, ss in deg.items():
    if len(ss) <= 2:
        continue
    x, y = p[0] * SNAP, p[1] * SNAP
    what = []
    if p in vk:
        what.append('VIA')
    if p in pk:
        what.append('PAD')
    print(f'  branch at ({x:.3f},{y:.3f}) degree {len(ss)} '
          f'{"/".join(what) or "bare"}')
    for s in ss:
        ln = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        print(f'     {s.layer:5s} ({s.start_x:8.3f},{s.start_y:8.3f})'
              f' -> ({s.end_x:8.3f},{s.end_y:8.3f})  len {ln:5.3f}')
