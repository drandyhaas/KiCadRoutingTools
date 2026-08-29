#!/usr/bin/env python3
"""Print a render --view box that covers a checkpoint's nets (teeth,
balls and the corridor between), with a margin. Hardcoding one view
crops the larger checkpoints, which is how a K51 render ends up showing
a K21-sized window."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '21'
M = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
xs, ys = [], []
for nm in names:
    for (x, y) in (ends[nm][0], ends[nm][1]):
        xs.append(x)
        ys.append(y)
# include the destination array in full, so escapes to any edge are visible
for ref in {ends[n][2] for n in names}:
    for p in pcb.footprints[ref].pads:
        xs.append(p.global_x)
        ys.append(p.global_y)
print(f'{min(xs) - M:.1f},{min(ys) - M:.1f},'
      f'{max(xs) + M:.1f},{max(ys) + M:.1f}')
