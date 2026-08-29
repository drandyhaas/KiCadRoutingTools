#!/usr/bin/env python3
"""Why does the `down` corridor's geometry not behave like the model?

The model says a corridor is a bundle wrapping one corner, so its via
floor is 2*(K - LIS) of the launch->exit permutation. On the synthetic
board that is exact. On this bench the `down` group reaches only 3
crossings where the model predicts 22, so one of the model's premises
does not hold. Print the premises: where each launch is relative to the
array, which corner its leg wraps, and whether it wraps at all.

usage: probe_down_geom.py [K] [side]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402
import escape_moves as em  # noqa: E402
import select_moves as sm  # noqa: E402
import detect_buses as db  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '21'
SIDE = sys.argv[2] if len(sys.argv) > 2 else 'down'
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
paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
buses = db.cluster(names, paths)
tooth_layer = {}
for nm in names:
    nid = byname[nm][0]
    tp = ends[nm][0]
    tooth_layer[nm] = next(
        (s.layer for s in pcb.segments if s.net_id == nid
         and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
              or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1]) < 0.005)),
        'F.Cu')
geo = sm.Corridor(grid0.bbox, launch)
choice, _ = sm.select(menu, launch, keep_out=grid0.bbox, buses=buses,
                      tooth_layer=tooth_layer)

x0, y0, x1, y1 = grid0.bbox
print(f'array bbox (pad centres) x {x0:.2f}..{x1:.2f}   '
      f'y {y0:.2f}..{y1:.2f}   pitch {grid0.pitch_x:.3f} x '
      f'{grid0.pitch_y:.3f}')
lx = [launch[n][0] for n in names]
ly = [launch[n][1] for n in names]
print(f'launches            x {min(lx):.2f}..{max(lx):.2f}   '
      f'y {min(ly):.2f}..{max(ly):.2f}\n')

HALF = min(grid0.pitch_x, grid0.pitch_y) / 2.0
grp = [n for n in names if choice[n].direction == SIDE]
print(f'{SIDE} corridor, {len(grp)} nets, in launch order:')
print(f'{"net":10s} {"launch":>16s} {"exit":>16s}  rel  wrap')
for n in sorted(grp, key=lambda n: launch[n][1]):
    lp, ep = launch[n], choice[n].exit_pt
    p = sm.around_box_path(lp, ep, grid0.bbox, pad=0.35 * HALF)
    if len(p) == 2:
        wrap = 'STRAIGHT (never touches the array)'
    else:
        wrap = ' -> '.join(f'({c[0]:.1f},{c[1]:.1f})' for c in p[1:-1])
    rel = ('N' if lp[1] < y0 else 'S' if lp[1] > y1 else '=')
    print(f'{n:10s} ({lp[0]:7.2f},{lp[1]:7.2f}) '
          f'({ep[0]:7.2f},{ep[1]:7.2f})   {rel}   {wrap}')
