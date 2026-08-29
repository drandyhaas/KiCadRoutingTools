#!/usr/bin/env python3
"""Probe teeth + ball geometry for a K-net list (default = R5 adds)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402

nets = sys.argv[1].split(',') if len(sys.argv) > 1 else \
    ['SRAS', 'SCAS', 'SA7', 'SA9']
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, nets, byname)
for nm in nets:
    (sx, sy), (bx, by), comp = ends[nm]
    print(f'{nm:6s} tooth ({sx:.3f},{sy:.3f})  ball ({bx:.3f},{by:.3f}) '
          f'-> {comp}')
