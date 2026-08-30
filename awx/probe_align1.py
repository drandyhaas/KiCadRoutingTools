#!/usr/bin/env python3
"""Why does a mismatched net not switch to an escape on the layer the
corridor delivers it on? Prints its tooth layer, the layer it is handed
over on, and every candidate on its bus side with the cost the selector
would give it."""
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
import detect_buses as db  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '21'
WHO = sys.argv[2] if len(sys.argv) > 2 else None
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
dl = sm.delivered_layers(choice, sm.corridors(choice), geo, tooth_layer)
bad = [n for n in names if n in choice and dl.get(n)
       and dl[n] != choice[n].layer]
print(f'tooth layers on the mismatched nets: '
      + ', '.join(f'{n}:{tooth_layer[n][0]}' for n in bad))
targets = [WHO] if WHO else bad[:2]
for nm in targets:
    m = choice[nm]
    side = m.direction
    lx, ly = launch[nm]
    print(f'\n{nm}: tooth {tooth_layer[nm]}, corridor delivers '
          f'{dl[nm]}, escape starts {m.layer}  -> MISMATCH')
    print(f'  chosen: {m}')
    print(f'  candidates on side {side}:')
    for c in sorted(menu[nm], key=lambda c: c.vias):
        if c.direction != side:
            continue
        el = sum(math.hypot(q[0] - p[0], q[1] - p[1])
                 for (p, q, _L) in c.legs)
        reach = sm.around_box((lx, ly), c.exit_pt, grid0.bbox)
        base = 3.0 * c.vias + 2.0 * el + reach
        pen = 0.0 if c.layer == dl[nm] else 4.0
        print(f'    {str(c):58s} cost {base:6.1f} + pen {pen:.0f} '
              f'= {base + pen:6.1f}')
