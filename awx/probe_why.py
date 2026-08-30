#!/usr/bin/env python3
"""Why did each berth pad get the escape direction it got?

Prints, per net: the tooth it comes from, the ball, the chosen move and
its cost breakdown, and the best alternative in each OTHER direction --
so a direction that looks wrong can be checked against what it was
compared with.

usage: probe_why.py [K]
"""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
import select_moves as sm  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '21'
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}
cache = {}


def obs(nid, layer):
    if (nid, layer) not in cache:
        cache[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
    return cache[(nid, layer)]


LAYERS = ('F.Cu', 'B.Cu')
menu, launch = {}, {}
for nm in names:
    nid = byname[nm][0]
    fp = pcb.footprints[ends[nm][2]]
    bx, by = ends[nm][1]
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)
    menu[nm] = em.enumerate_moves(
        pad, em.grid_of(fp), LAYERS,
        lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
        lambda p, L, _n=nid: not (obs(_n, L).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
    launch[nm] = ends[nm][0]
grid0 = em.grid_of(pcb.footprints[ends[names[0]][2]])
import detect_buses as db
paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
buses = db.cluster(names, paths)
print('buses: ' + ' | '.join(','.join(b) for b in buses))
choice, unplaced = sm.select(menu, launch, keep_out=grid0.bbox,
                             buses=buses)

VW, CW = 3.0, 2.0
print(f'cost = {VW}*vias + {CW}*escape_len + reach(launch->exit), '
      f'reach measured AROUND the array\n')
print(f'{"net":9s} {"tooth":>13s} {"ball":>13s}  chosen'
      f'          esc  reach  cost | best per direction')
for nm in sorted(names, key=lambda n: ends[n][1][1]):
    tx, ty = ends[nm][0]
    bx, by = ends[nm][1]
    m = choice.get(nm)

    def brk(mv):
        el = sum(math.hypot(q[0] - p[0], q[1] - p[1])
                 for (p, q, _L) in mv.legs)
        # the SAME reach the selector uses: around the array, not
        # through it
        rr = sm.around_box((tx, ty), mv.exit_pt, grid0.bbox)
        return el, rr, VW * mv.vias + CW * el + rr

    alts = []
    for d in ('left', 'right', 'up', 'down'):
        cands = [x for x in menu[nm] if x.direction == d]
        if not cands:
            alts.append(f'{d[0]}:--')
            continue
        best = min(cands, key=lambda x: brk(x)[2])
        alts.append(f'{d[0]}:{brk(best)[2]:.0f}')
    if m is None:
        print(f'{nm:9s} ({tx:6.2f},{ty:6.2f}) ({bx:6.2f},{by:6.2f})  '
              f'UNPLACED                    | {" ".join(alts)}')
        continue
    el, rr, c = brk(m)
    print(f'{nm:9s} ({tx:6.2f},{ty:6.2f}) ({bx:6.2f},{by:6.2f})  '
          f'{m.kind[:7]:7s}/{m.direction:5s} {el:5.1f} {rr:6.1f} '
          f'{c:5.1f} | {" ".join(alts)}')
