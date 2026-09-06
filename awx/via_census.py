#!/usr/bin/env python3
"""Board-total via census for a net list: vias + segments per net."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

board = sys.argv[1]
nets = sys.argv[2].split(',')
pcb = parse_kicad_pcb(board)
byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
tot_v = tot_s = 0
rows = []
for nm in nets:
    nid = byname[nm]
    nv = sum(1 for v in pcb.vias if v.net_id == nid)
    ns = sum(1 for s in pcb.segments if s.net_id == nid)
    tot_v += nv
    tot_s += ns
    rows.append(f'{nm}:{nv}')
print(os.path.basename(board), f'K={len(nets)}',
      f'TOTAL vias={tot_v} segs={tot_s}')
print('  ', ' '.join(rows))
