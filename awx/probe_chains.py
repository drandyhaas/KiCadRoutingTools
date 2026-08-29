#!/usr/bin/env python3
"""Print full stub segment chains for representative nets of each
overhang group."""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

pcb = parse_kicad_pcb(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
for nm in ('SA14', 'SDQ15', 'SDQ6', 'SBA1', 'SA10', 'SDQ8'):
    nid, net = byname[nm]
    print(nm)
    for s in pcb.segments:
        if s.net_id == nid:
            print(f'  ({s.start_x:.3f},{s.start_y:.3f})->'
                  f'({s.end_x:.3f},{s.end_y:.3f}) {s.layer} w={s.width}')
