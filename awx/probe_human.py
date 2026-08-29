#!/usr/bin/env python3
"""How does the HUMAN route the west corridor? Per west net: vias,
F/B length, and the layer occupancy per station across the corridor
(x0..x1) so the structure (which layer carries what, where dives
happen) can be compared with the L2 plan."""
import math
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

board = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/Downloads/bus/00_human_original.kicad_pcb')
pcb = parse_kicad_pcb(board)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
W = ('SDQM1,SDQ9,SDQ10,SDQ11,SDQS1N,SDQ8,SDQS1P,SDQ12,SDQ13,SDQM0,SDQ14,'
     'SA14,SDQ15,SDQ0,SA10,SA11,SDQ2,SDQ1,SA15,SA12,SDQS0P,SDQS0N,SDQ4,SA0,'
     'SDQ5,SBA1,SDQ6,SDQ3,SA3,SDQ7,SA1,SA4,SCS0,SCS1,SCKE1,SA6').split(',')
S = ('SA13,SA2,SA5,SA7,SA8,SA9,SBA0,SBA2,SCAS,SCKE0,SODT0,SODT1,SRAS,SRST,'
     'SWE').split(',')
x0, x1 = 127.81, 133.29
tot = {'v': 0, 'F': 0.0, 'B': 0.0}
rows = []
for nm in W + S:
    nid = byname[nm][0]
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    lf = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in segs if s.layer == 'F.Cu')
    lb = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in segs if s.layer == 'B.Cu')
    # layer at corridor stations: which layer has copper of this net at x
    prof = ''
    for k in range(12):
        x = x0 + (x1 - x0) * (k + 0.5) / 12
        lay = set()
        for s in segs:
            if min(s.start_x, s.end_x) - 0.05 <= x <= \
                    max(s.start_x, s.end_x) + 0.05:
                lay.add(s.layer[0])
        prof += ''.join(sorted(lay)) if lay else '.'
        prof += ' '
    vx = ' '.join(f'{v.x:.1f}' for v in sorted(vias, key=lambda v: v.x))
    rows.append((nm, len(vias), lf, lb, prof, vx))
    tot['v'] += len(vias)
    tot['F'] += lf
    tot['B'] += lb
print(f'{"net":8s} vias  F_mm   B_mm   layer@corridor stations (x0..x1)   via xs')
for nm, nv, lf, lb, prof, vx in rows:
    print(f'{nm:8s} {nv:3d}  {lf:5.1f}  {lb:5.1f}   {prof}  {vx}')
print(f'TOTAL vias {tot["v"]}  F {tot["F"]:.0f} mm  B {tot["B"]:.0f} mm  '
      f'({len(W)} west + {len(S)} south)')
