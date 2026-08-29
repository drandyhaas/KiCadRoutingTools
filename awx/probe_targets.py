#!/usr/bin/env python3
"""Print K-net target balls sorted by y (rows/columns view)."""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

K = 'SDQ15 SDQ14 SDQ13 SDQ11 SDQ0 SDQM0 SDQ12 SDQ8'.split()
pcb = parse_kicad_pcb(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: n for n in pcb.nets.values()}
rows = []
for nm in K:
    for p in byname[nm].pads:
        if p.component_ref == 'DU1':
            rows.append((p.global_y, p.global_x, nm, p.pad_number))
for y, x, nm, pn in sorted(rows):
    print(f'{nm:6s} DU1.{pn:4s} ({x:.3f},{y:.3f})')
