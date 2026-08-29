#!/usr/bin/env python3
"""Per-segment lane deviation + layer for a net (K2/K4 debugging)."""
import json
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

board = sys.argv[1]
names = sys.argv[2].split(',')
pcb = parse_kicad_pcb(board)
sj = json.load(open(board + '.attract.json'))
lanes = {k.split('/')[-1]: v for k, v in sj['targets'].items()}


def d_pt_poly(x, y, poly):
    best = 1e9
    for (x1, y1), (x2, y2) in zip(poly, poly[1:]):
        vx, vy = x2 - x1, y2 - y1
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - x1) * vx
                                                      + (y - y1) * vy) / L2))
        best = min(best, math.hypot(x - (x1 + t * vx), y - (y1 + t * vy)))
    return best


for nm in names:
    nid = next(i for i, n in pcb.nets.items() if n.name.endswith('/' + nm))
    print(f'--- {nm} (net {nid}) segments (L>=0.3mm) with lane deviation:')
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        d1 = d_pt_poly(s.start_x, s.start_y, lanes[nm])
        d2 = d_pt_poly(s.end_x, s.end_y, lanes[nm])
        L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        if L < 0.3:
            continue
        print(f'  {s.layer:5s} ({s.start_x:.1f},{s.start_y:.1f})->'
              f'({s.end_x:.1f},{s.end_y:.1f}) L={L:.1f} dev {d1:.2f}/{d2:.2f}')
    for v in pcb.vias:
        if v.net_id == nid:
            print(f'  VIA at ({v.x:.1f},{v.y:.1f})')
