#!/usr/bin/env python3
"""The human's via count for each COHERENT ladder checkpoint, so our
braid is compared against the same nets it routed -- not against the
whole board. Usage: human_at_k.py [K ...]"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

board = os.path.expanduser('~/Downloads/bus/00_human_original.kicad_pcb')
if not os.path.isfile(board):
    print(f'no human reference at {board}')
    sys.exit(2)
pcb = parse_kicad_pcb(board)
byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
Ks = [int(a) for a in sys.argv[1:]] or [4, 11, 15, 19, 21, 32, 47, 51]
for K in Ks:
    nets = subprocess.run([sys.executable,
                           os.path.join(HERE, 'coherent_nets.py'), str(K)],
                          capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    tot_v = 0
    miss = []
    for nm in names:
        if nm not in byname:
            miss.append(nm)
            continue
        nid = byname[nm]
        tot_v += sum(1 for v in pcb.vias if v.net_id == nid)
    print(f'human K={K:3d}: vias={tot_v}'
          + (f'  (not on board: {",".join(miss)})' if miss else ''))
