#!/usr/bin/env python3
"""Print a net's segments at full precision near a point."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

board, NM = sys.argv[1], sys.argv[2]
cx, cy = float(sys.argv[3]), float(sys.argv[4])
r = float(sys.argv[5]) if len(sys.argv) > 5 else 0.05
pcb = parse_kicad_pcb(board)
byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
nid = byname[NM]
for s in pcb.segments:
    if s.net_id != nid:
        continue
    if min(math.hypot(s.start_x - cx, s.start_y - cy),
           math.hypot(s.end_x - cx, s.end_y - cy)) > r:
        continue
    L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
    print(f'  {s.layer:5s} ({s.start_x!r}, {s.start_y!r}) -> '
          f'({s.end_x!r}, {s.end_y!r})  len {L * 1000:.4f} um')
