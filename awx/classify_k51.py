#!/usr/bin/env python3
"""K51 = the 51 two-pad U1<->DU1 DDR nets. Classify by escape flank:
'west' (tooth on U1's east comb -> braid corridor), else 'defer'."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402

pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
names = [nm for nm, (i, n) in byname.items()
         if '/DDR3' in n.name and len(n.pads) == 2
         and {p.component_ref for p in n.pads} == {'DU1', 'U1'}]
names.sort()
ends = te.endpoints(pcb, names, byname)
u1 = pcb.footprints['U1']
xs = sorted({round(p.global_x, 2) for p in u1.pads})
ys = sorted({round(p.global_y, 2) for p in u1.pads})
west, defer = [], []
for nm in names:
    (sx, sy), (bx, by), comp = ends[nm]
    if sx > xs[-1] + 0.2 and ys[0] - 0.5 < sy < ys[-1] + 0.5:
        west.append(nm)
    else:
        defer.append(nm)
west.sort(key=lambda nm: ends[nm][0][1])
print(f'{len(names)} nets: west {len(west)}, defer {len(defer)}')
print('WEST=' + ','.join(west))
print('DEFER=' + ','.join(defer))
for nm in defer:
    (sx, sy), (bx, by), comp = ends[nm]
    print(f'  {nm:7s} tooth ({sx:.2f},{sy:.2f}) -> ball ({bx:.2f},{by:.2f})')
