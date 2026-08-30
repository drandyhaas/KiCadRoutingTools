#!/usr/bin/env python3
"""Every board item within R of a point: segments, vias, pads, zones,
graphics. usage: near_pt.py BOARD X Y [R]"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

board, X, Y = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
R = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
pcb = parse_kicad_pcb(board)
name = {i: n.name for i, n in pcb.nets.items()}


def seg_d(ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0 if L2 < 1e-12 else max(0, min(1, ((X - ax) * dx + (Y - ay) * dy) / L2))
    return math.hypot(X - (ax + t * dx), Y - (ay + t * dy))


for s in pcb.segments:
    d = seg_d(s.start_x, s.start_y, s.end_x, s.end_y)
    if d <= R:
        print(f'seg  {name.get(s.net_id, s.net_id):12} {s.layer:6} w{s.width} '
              f'({s.start_x:.3f},{s.start_y:.3f})->({s.end_x:.3f},{s.end_y:.3f}) d={d:.3f}')
for v in pcb.vias:
    d = math.hypot(v.x - X, v.y - Y)
    if d <= R:
        print(f'via  {name.get(v.net_id, v.net_id):12} ({v.x:.3f},{v.y:.3f}) '
              f'size {v.size} d={d:.3f}')
for ref, fp in pcb.footprints.items():
    for p in fp.pads:
        d = math.hypot(p.global_x - X, p.global_y - Y)
        if d <= R + max(p.size_x, p.size_y):
            print(f'pad  {ref}.{p.pad_number} {name.get(p.net_id, p.net_id)} '
                  f'({p.global_x:.3f},{p.global_y:.3f}) {p.size_x}x{p.size_y} '
                  f'{p.pad_type} {p.layers} d={d:.3f}')
print(f'zones: {len(getattr(pcb, "zones", []) or [])}')
for z in getattr(pcb, 'zones', []) or []:
    print('  zone', getattr(z, 'net_id', '?'), getattr(z, 'layer', '?'),
          getattr(z, 'layers', '?'))
txt = open(board, encoding='utf-8').read()
for m in re.finditer(r'\(gr_(line|rect|poly|arc|circle)[^\n]*\n(?:[^\n]*\n){0,6}', txt):
    blk = m.group(0)
    xs = [float(v) for v in re.findall(r'\(start ([-\d.]+) ([-\d.]+)\)', blk)
          for v in v]
    if xs and abs(xs[0] - X) < 3 and abs(xs[1] - Y) < 3:
        print('gr:', blk.split('\n')[0][:120])
