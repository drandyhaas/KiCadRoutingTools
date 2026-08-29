#!/usr/bin/env python3
"""Field-entry demand vs supply at a coherent-ladder checkpoint.

The braid's first hard failure as K grows is `no street/dogbone/VIP for
<net>` -- it runs out of ways INTO the destination field, not out of
corridor. This prints, per destination row, how many nets want in, and
what the three entry kinds can supply, so the ceiling is a number
rather than an assertion."""
import os
import subprocess
import sys
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '32'
nets = subprocess.run([sys.executable,
                       os.path.join(HERE, 'coherent_nets.py'), K],
                      capture_output=True, text=True).stdout.strip()
names = [n for n in nets.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
comp = ends[names[0]][2]
fp = pcb.footprints[comp]
xs = sorted({round(p.global_x, 2) for p in fp.pads})
ys = sorted({round(p.global_y, 2) for p in fp.pads})
print(f'K={K}  destination {comp}: {len(xs)} cols x {len(ys)} rows')
rows, cols = Counter(), Counter()
for nm in names:
    bx, by = ends[nm][1]
    ri = min(range(len(ys)), key=lambda i: abs(ys[i] - by))
    ci = min(range(len(xs)), key=lambda i: abs(xs[i] - bx))
    rows[ri] += 1
    cols[ci] += 1
print('nets per row :', dict(sorted(rows.items())))
print('nets per col :', dict(sorted(cols.items())))
# An F street is the gap on either side of a ball's row; each street
# carries ONE net past a given column. Rows adjacent to a row gap share
# the street, so supply per row-gap is 1 lane.
print(f'\nrow gaps available: {len(ys) + 1} (one lane each) '
      f'vs {len(names)} nets wanting in')
print('=> F streets alone cap the field at ~%d nets; the rest need a '
      'B dogbone (diagonal inter-ball cell) or via-in-pad'
      % (len(ys) + 1))
deep = sum(1 for nm in names
           if min(range(len(xs)),
                  key=lambda i: abs(xs[i] - ends[nm][1][0]))
           >= 0.35 * (len(xs) - 1))
print(f'targets deep in the array (col >= 35%): {deep} of {len(names)}')
