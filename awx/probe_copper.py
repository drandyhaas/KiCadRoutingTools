#!/usr/bin/env python3
"""Dump full copper (pads/segments/vias) for the named nets."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

nets = sys.argv[1].split(',')
pcb = parse_kicad_pcb(os.path.join(HERE, os.environ.get('PROBE_BOARD', 'fb_t2q_base.kicad_pcb')))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
for nm in nets:
    nid, net = byname[nm]
    print(nm, 'pads:', [(p.component_ref, p.pad_number,
                         round(p.global_x, 2), round(p.global_y, 2))
                        for p in net.pads])
    for s in pcb.segments:
        if s.net_id == nid:
            print(f'   ({s.start_x:.3f},{s.start_y:.3f})->'
                  f'({s.end_x:.3f},{s.end_y:.3f}) {s.layer}')
    for v in pcb.vias:
        if v.net_id == nid:
            print(f'   via ({v.x:.3f},{v.y:.3f})')
