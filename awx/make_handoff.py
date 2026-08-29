#!/usr/bin/env python3
"""Render the HANDOFF: U1 tooth -> corridor -> berth escape -> ball, and
mark where that handoff does not work.

The intent overlay shows the taut homotopy and the chosen escapes, but
not the join between them -- which is where the plan currently fails.
This draws the join and its failures:

  Eco1  white   the CORRIDOR LEG the braid must draw: tooth to the
                escape's exit point, routed around the destination
                array (the corridor cannot cross it).
  Eco2  yellow  the ESCAPE itself: exit point back to the ball, plus a
                marker at the exit.
  Cmts  orange  the FAILURES. A large cross at an exit the braid has no
                mechanism to reach (any side but the corridor's), and a
                small box at an exit where the corridor arrives on the
                opposite layer from the one the escape starts on, which
                costs a via nothing has counted.

usage: make_handoff.py OUT.kicad_pcb [K]
"""
import math
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

out_path = sys.argv[1]
K = sys.argv[2] if len(sys.argv) > 2 else '21'
base = os.path.join(HERE, 'fb_t2q_base.kicad_pcb')
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(base)
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
print('taut pre-routes...')
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
choice, _un = sm.select(menu, launch, keep_out=grid0.bbox, buses=buses,
                        tooth_layer=tooth_layer)
# which nets the corridor would make divers, per bus
delivered = sm.delivered_layers(choice, buses, launch, tooth_layer)

lines = []


def gr(p, q, layer):
    lines.append(f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                 f'(end {q[0]:.4f} {q[1]:.4f}) '
                 f'(stroke (width 0.05) (type solid)) (layer "{layer}"))\n')


def cross(c, r, layer):
    gr((c[0] - r, c[1] - r), (c[0] + r, c[1] + r), layer)
    gr((c[0] - r, c[1] + r), (c[0] + r, c[1] - r), layer)


def box(c, r, layer):
    p = [(c[0] - r, c[1] - r), (c[0] + r, c[1] - r),
         (c[0] + r, c[1] + r), (c[0] - r, c[1] + r)]
    for a, b in zip(p, p[1:] + p[:1]):
        gr(a, b, layer)


CORRIDOR_SIDE = 'left'
n_place = n_layer = 0
for nm in names:
    m = choice.get(nm)
    if m is None:
        continue
    leg = sm.around_box_path(launch[nm], m.exit_pt, grid0.bbox)
    for a, b in zip(leg, leg[1:]):
        gr(a, b, 'Eco1.User')
    for (a, b, _L) in m.legs:
        gr(a, b, 'Eco2.User')
    box(m.exit_pt, 0.16, 'Eco2.User')
    if m.direction != CORRIDOR_SIDE:
        cross(m.exit_pt, 0.42, 'Cmts.User')
        n_place += 1
    elif delivered.get(nm) and delivered[nm] != m.layer:
        box(m.exit_pt, 0.30, 'Cmts.User')
        n_layer += 1

txt = open(base, encoding='utf-8').read()
k = txt.rstrip().rfind(')')
open(out_path, 'w').write(txt[:k] + ''.join(lines) + txt[k:])
pro = os.path.splitext(base)[0] + '.kicad_pro'
if os.path.exists(pro):
    import shutil
    shutil.copy(pro, os.path.splitext(out_path)[0] + '.kicad_pro')
print(f'wrote {out_path}: {len(lines)} lines; '
      f'{n_place} exits the corridor cannot reach (orange X), '
      f'{n_layer} layer mismatches (orange box)')
