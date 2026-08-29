#!/usr/bin/env python3
"""What blocks a net's horizontal B approach? Names the obstacles the
run touches, so "B run hits static copper" becomes a specific object."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402

NM = sys.argv[1] if len(sys.argv) > 1 else 'SBA1'
K = sys.argv[2] if len(sys.argv) > 2 else '32'
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
comps = {ends[n][2] for n in names}
pads = [p for c in comps for p in pcb.footprints[c].pads]
fx0 = min(p.global_x for p in pads)
x1 = fx0 - 0.64
bx, by = ends[NM][1]
obs = te.build_obstacles(pcb, byname[NM][0], kids, 'B.Cu')
print(f'{NM}: ball ({bx:.2f},{by:.2f})  splice x1={x1:.2f}')
for sy in (by, by - 0.4, by + 0.4):
    sx = bx if sy == by else bx - 0.4
    run = ((x1, sy), (sx, sy))
    ok = obs.seg_clear(*run)
    print(f'  B run y={sy:.2f} from x={x1:.2f} to {sx:.2f}: '
          f'{"CLEAR" if ok else "BLOCKED"}')
    if not ok:
        hugs = sorted(obs.hugs([run[0], run[1]], slack=0.0))
        print(f'      touches: {hugs[:8]}')
# where along the run does it first fail?
sy = by
step = 0.1
x = x1
prev_ok = True
while x < bx:
    seg = ((x, sy), (min(x + step, bx), sy))
    ok = obs.seg_clear(*seg)
    if not ok and prev_ok:
        print(f'  first blockage near x={x:.2f} '
              f'(field starts at {min(p.global_x for p in pcb.footprints["DU1"].pads):.2f})')
        print('      ', sorted(obs.hugs([seg[0], seg[1]], slack=0.0))[:6])
        break
    prev_ok = ok
    x += step
