#!/usr/bin/env python3
"""Draw the router's INTENT onto the Eco layers, so it can be checked by
eye rather than only by counters.

Today the emitter puts a copy of the emitted copper on Eco2, which shows
nothing the copper itself does not. The two mechanisms that now decide
the route are invisible, and they are exactly the ones worth seeing:

  Eco1.User   the TAUT PRE-ROUTES -- one string per net, tooth to ball,
              pulled tight against the static obstacles. This is the
              homotopy the router is working from, and the thing the bus
              clustering is computed on. Where these run together is a
              bus.
  Eco2.User   the CHOSEN ESCAPE MOVE per net: the legs from the ball out
              to its exit point, plus a diamond at the exit and a cross
              at any via site. This is what the selector picked out of
              the menu, and whether it points the right way is obvious
              at a glance.

usage: make_overlay.py OUT.kicad_pcb [K] [--board BASE]
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
import detect_buses as db  # noqa: E402

out_path = sys.argv[1]
K = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith('-') \
    else '21'
base = os.path.join(HERE, 'fb_t2q_base.kicad_pcb')
if '--board' in sys.argv:
    base = sys.argv[sys.argv.index('--board') + 1]

names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(base)
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
kids = {byname[n][0] for n in names}

obs_cache = {}


def obs(nid, layer):
    if (nid, layer) not in obs_cache:
        obs_cache[(nid, layer)] = te.build_obstacles(pcb, nid, kids, layer)
    return obs_cache[(nid, layer)]


print('taut pre-routes...')
paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
groups = db.cluster(names, paths)
print(f'  {len(groups)} bus(es): ' + ', '.join(str(len(g)) for g in groups))

print('escape menu + selection...')
LAYERS = ('F.Cu', 'B.Cu')
menu, launch = {}, {}
for nm in names:
    nid = byname[nm][0]
    ref = ends[nm][2]
    fp = pcb.footprints[ref]
    bx, by = ends[nm][1]
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)
    menu[nm] = em.enumerate_moves(
        pad, em.grid_of(fp), LAYERS,
        lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
        lambda p, L, _n=nid: not (
            obs(_n, L).point_violation(p, pad=(te.VIA_SIZE - te.TRACK) / 2)
            or [0])[0])
    launch[nm] = ends[nm][0]
grid0 = em.grid_of(pcb.footprints[ends[names[0]][2]])
tooth_layer = {}
for nm in names:
    _nid = byname[nm][0]
    _tp = ends[nm][0]
    tooth_layer[nm] = next(
        (s.layer for s in pcb.segments if s.net_id == _nid
         and (abs(s.start_x - _tp[0]) + abs(s.start_y - _tp[1]) < 0.005
              or abs(s.end_x - _tp[0]) + abs(s.end_y - _tp[1]) < 0.005)),
        'F.Cu')
geo = sm.Corridor(grid0.bbox, launch)
choice, unplaced = sm.select(menu, launch, keep_out=grid0.bbox,
                             buses=groups, tooth_layer=tooth_layer)
print('  ' + sm.summarise(choice))

lines = []


def gr(p, q, layer):
    lines.append(f'  (gr_line (start {p[0]:.4f} {p[1]:.4f}) '
                 f'(end {q[0]:.4f} {q[1]:.4f}) '
                 f'(stroke (width 0.05) (type solid)) '
                 f'(layer "{layer}"))\n')


def marker(c, r, layer, sides=4):
    pts = [(c[0] + r * math.cos(2 * math.pi * i / sides + math.pi / 4),
            c[1] + r * math.sin(2 * math.pi * i / sides + math.pi / 4))
           for i in range(sides)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        gr(a, b, layer)


for nm in names:
    for a, b in zip(paths[nm], paths[nm][1:]):
        gr(a, b, 'Eco1.User')
for nm, m in choice.items():
    for (a, b, _L) in m.legs:
        gr(a, b, 'Eco2.User')
    marker(m.exit_pt, 0.18, 'Eco2.User')
    if m.site:
        marker(m.site, 0.10, 'Eco2.User', sides=4)

txt = open(base, encoding='utf-8').read()
k = txt.rstrip().rfind(')')
open(out_path, 'w').write(txt[:k] + ''.join(lines) + txt[k:])
pro = os.path.splitext(base)[0] + '.kicad_pro'
if os.path.exists(pro):
    import shutil
    shutil.copy(pro, os.path.splitext(out_path)[0] + '.kicad_pro')
print(f'wrote {out_path}: {len(lines)} overlay lines '
      f'(Eco1 = taut strings, Eco2 = chosen escape moves)')
