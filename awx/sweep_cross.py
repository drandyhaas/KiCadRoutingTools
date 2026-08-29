#!/usr/bin/env python3
"""Dose-response for the inter-corridor crossing weight.

A single good value proves nothing -- it could be a lucky point. This
runs the whole selection at a range of cross_weight and reports what
each buys: the crossings inside corridors (which the floor prices), the
crossings BETWEEN them (which nothing prices yet), the floor, and the
true via count. cross_weight 0 is the negative control -- the behaviour
before the term existed.

usage: sweep_cross.py [K] [weights...]
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
WS = [float(a) for a in sys.argv[2:]] or [0.0, 1.0, 2.0, 3.0, 6.0, 12.0]
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

print(f'K={K}, {len(names)} nets, {len(buses)} taut-path buses\n')
print(f'{"cross_wt":>8s} {"corridors":>22s} {"intra":>6s} {"inter":>6s} '
      f'{"floor":>6s} {"vias":>5s} {"PLAN":>5s}')
for w in WS:
    choice, unplaced = sm.select(menu, launch, keep_out=grid0.bbox,
                                 buses=buses, tooth_layer=tooth_layer,
                                 cross_weight=w)
    corr = sm.corridors(choice)
    of = {n: i for i, g in enumerate(corr) for n in g}
    intra = inter = 0
    ns = [n for n in names if n in choice]
    for i, a in enumerate(ns):
        for b in ns[i + 1:]:
            if geo.crosses(a, b, choice):
                if of[a] == of[b]:
                    intra += 1
                else:
                    inter += 1
    tv, fl, mm = sm.score(choice, corr, geo, tooth_layer)
    pf = sm.plan_floor(choice, geo)
    shape = ','.join(f'{choice[g[0]].direction}:{len(g)}' for g in corr)
    tag = '   <- control' if w == 0 else ''
    print(f'{w:8.1f} {shape:>22s} {intra:6d} {inter:6d} {fl:6d} '
          f'{tv:5d} {pf:5d}{tag}')
    if unplaced:
        print(f'{"":8s} UNPLACED: {",".join(unplaced)}')
