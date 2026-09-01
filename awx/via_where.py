#!/usr/bin/env python3
"""Where are a net's vias, relative to the two arrays? Prints each via
with its board position and which zone it falls in: inside DU1 window,
inside U1 window, corridor (between them), or elsewhere.
usage: via_where.py BOARD NET,NET,..."""
import sys
import os

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

pcb = parse_kicad_pcb(sys.argv[1])
byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}


def window(ref, m=1.0):
    fp = pcb.footprints[ref]
    xs = [p.global_x for p in fp.pads]
    ys = [p.global_y for p in fp.pads]
    return (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)


DU = window('DU1')
U1 = window('U1')
print(f'DU1 window {tuple(round(v, 1) for v in DU)}')
print(f'U1  window {tuple(round(v, 1) for v in U1)}')


def zone(x, y):
    if DU[0] <= x <= DU[2] and DU[1] <= y <= DU[3]:
        return 'DU1'
    if U1[0] <= x <= U1[2] and U1[1] <= y <= U1[3]:
        return 'U1'
    return 'mid'


for nm in sys.argv[2].split(','):
    nid = byname.get(nm)
    if nid is None:
        print(f'{nm}: NOT ON BOARD')
        continue
    vs = [v for v in pcb.vias if v.net_id == nid]
    ball = next((p for p in pcb.footprints['DU1'].pads
                 if p.net_id == nid), None)
    bs = (f'ball=({ball.global_x:.1f},{ball.global_y:.1f})'
          if ball else 'no ball')
    row = ' '.join(f'({v.x:.1f},{v.y:.1f})[{zone(v.x, v.y)}]'
                   for v in sorted(vs, key=lambda v: v.x))
    print(f'{nm:7s} {len(vs)} via(s) {bs}  {row}')
