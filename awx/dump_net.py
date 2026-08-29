#!/usr/bin/env python3
"""Every segment and via of one net, in walk order, marked as BASE
(fanout stub already on the input board) or NEW (emitted). Use this to
read a net's route the way you would trace it on the board."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402

board, NM = sys.argv[1], sys.argv[2]
pcb = parse_kicad_pcb(board)
base = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
nid, net = byname[NM]
bkey = {(round(s.start_x, 3), round(s.start_y, 3),
         round(s.end_x, 3), round(s.end_y, 3), s.layer)
        for s in base.segments if s.net_id == byname[NM][0]}


def tag(s):
    k = (round(s.start_x, 3), round(s.start_y, 3),
         round(s.end_x, 3), round(s.end_y, 3), s.layer)
    kr = (k[2], k[3], k[0], k[1], k[4])
    return 'BASE' if (k in bkey or kr in bkey) else 'NEW '


segs = [s for s in pcb.segments if s.net_id == nid]
vias = [v for v in pcb.vias if v.net_id == nid]
print(f'{NM}: {len(segs)} segments, {len(vias)} vias')
print('  pads: ' + ', '.join(f'{p.component_ref}.{p.pad_number}'
                             f'({p.global_x:.2f},{p.global_y:.2f})'
                             for p in net.pads))
for s in sorted(segs, key=lambda s: min(s.start_x, s.end_x)):
    ln = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
    dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
    ang = math.degrees(math.atan2(dy, dx)) % 45.0
    oct_ok = min(ang, 45.0 - ang) <= 0.6
    print(f'  {tag(s)} {s.layer:5s} ({s.start_x:8.3f},{s.start_y:8.3f})'
          f' -> ({s.end_x:8.3f},{s.end_y:8.3f})  len {ln:5.2f}'
          + ('' if oct_ok else '   <-- NOT octilinear'))
for v in sorted(vias, key=lambda v: v.x):
    print(f'  VIA        ({v.x:8.3f},{v.y:8.3f})  {v.layers}')
