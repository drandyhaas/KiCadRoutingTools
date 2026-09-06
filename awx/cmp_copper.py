#!/usr/bin/env python3
"""Do two boards carry the SAME copper? (UUIDs differ every write, so a
file diff or a hash says nothing -- compare the geometry.)

usage: cmp_copper.py A.kicad_pcb B.kicad_pcb
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402


def copper(path):
    p = parse_kicad_pcb(path)
    segs = {(round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
             round(s.end_y, 4), round(s.width, 4), s.layer, s.net_id)
            for s in p.segments}
    vias = {(round(v.x, 4), round(v.y, 4), round(v.size, 4),
             round(v.drill, 4), v.net_id) for v in p.vias}
    return segs, vias


sa, va = copper(sys.argv[1])
sb, vb = copper(sys.argv[2])
print(f'A: {len(sa)} segments, {len(va)} vias')
print(f'B: {len(sb)} segments, {len(vb)} vias')
ds, dv = sa ^ sb, va ^ vb
if not ds and not dv:
    print('IDENTICAL copper')
else:
    print(f'DIFFER: {len(ds)} segment(s) and {len(dv)} via(s) in one but '
          f'not the other')
    for x in sorted(ds)[:6]:
        print(f'   seg {"A" if x in sa else "B"} {x}')
    for x in sorted(dv)[:6]:
        print(f'   via {"A" if x in va else "B"} {x}')
sys.exit(0 if not ds and not dv else 1)
