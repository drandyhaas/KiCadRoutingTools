#!/usr/bin/env python3
"""What does endpoints() make of a board where BOTH ends are fanned out?

The braid was written for a bench where only the source is fanned out:
it routes a free stub end to a raw BALL, and the whole field-entry
machinery exists to get into that ball field. Fan the destination out
too and there are two free ends per net, so `src` and `tgt` are no
longer what the braid assumes -- and the failure would be silent
(a route in the wrong direction, or into the source's own field).

usage: probe_ends_chained.py BOARD [K]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402

board = sys.argv[1]
K = sys.argv[2] if len(sys.argv) > 2 else '11'
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(board)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
try:
    ends = te.endpoints(pcb, names, byname)
except AssertionError as e:
    print(f'endpoints() REFUSED: {e}')
    sys.exit(1)

import math
from collections import Counter
print(f'{board}: {len(pcb.segments)} segments, {len(pcb.vias)} vias\n')
print(f'{"net":9s} {"free ends":>9s}  {"src":>17s} -> {"tgt":>17s}  '
      f'tgt is')
bad = 0
for nm in names:
    nid, net = byname[nm]
    segs = [s for s in pcb.segments if s.net_id == nid]
    cnt = Counter()
    for s in segs:
        cnt[(round(s.start_x, 3), round(s.start_y, 3))] += 1
        cnt[(round(s.end_x, 3), round(s.end_y, 3))] += 1
    anchors = [(p.global_x, p.global_y) for p in net.pads] + \
        [(v.x, v.y) for v in pcb.vias if v.net_id == nid]
    free = [pt for pt, c in cnt.items() if c == 1 and
            all(math.hypot(pt[0] - ax, pt[1] - ay) > 0.02
                for (ax, ay) in anchors)]
    src, tgt, ref = ends[nm]
    # is src near the SOURCE component or the DESTINATION one?
    def near(pt):
        p = min(net.pads, key=lambda q: (q.global_x - pt[0]) ** 2
                + (q.global_y - pt[1]) ** 2)
        return p.component_ref
    s_at, t_at = near(src), ref
    flag = ''
    if s_at == t_at:
        flag = '  <- SAME component: the braid would route into its own field'
        bad += 1
    print(f'{nm:9s} {len(free):9d}  ({src[0]:7.2f},{src[1]:7.2f}) -> '
          f'({tgt[0]:7.2f},{tgt[1]:7.2f})  {ref} pad{flag}')
print(f'\n{bad} of {len(names)} nets have src and tgt on the SAME component')
print('NOTE: tgt is always a PAD -- endpoints() has no notion of '
      'delivering to the destination\'s own stub end.')
