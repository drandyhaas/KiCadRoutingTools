#!/usr/bin/env python3
"""What do the corridors cost EACH OTHER?

Every via floor is computed inside one corridor. But the corridors share
a board: a leg of the `left` corridor and a leg of the `down` corridor
can cross, and that crossing is as real as any inside a corridor and is
priced by nobody. This counts them, from the same drawn legs the floors
use, and says what it would take to separate each pair.

Two corridors that cross can be separated three ways, in rising cost:

  BY LAYER    if one corridor's legs can all ride one layer and the
              other's the other layer, their crossings cost nothing.
              This is the human's deep-net pattern -- run under the
              array on B and conflict with nothing on F.
  BY ORDER    if the crossings come from the two corridors' bundles
              being interleaved at the source, re-ordering the U1 teeth
              (or swapping which bus takes which side) removes them.
  BY VIAS     2 per leg that has to dive out of the way.

usage: probe_inter_corridor.py [K]
"""
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
corr = sm.corridor_groups(choice)
of = {n: i for i, g in enumerate(corr) for n in g}
dl = sm.delivered_layers(choice, corr, geo, tooth_layer)

intra = 0
pairs = {}
for i, a in enumerate(names):
    if a not in choice:
        continue
    for b in names[i + 1:]:
        if b not in choice or not geo.crosses(a, b, choice):
            continue
        if of[a] == of[b]:
            intra += 1
        else:
            k = tuple(sorted((of[a], of[b])))
            pairs.setdefault(k, []).append((a, b))

floors = {i: (sm._floor(g, choice, geo) if len(g) >= 2 else 0)
          for i, g in enumerate(corr)}
print(f'K={K}: {len(choice)} placed nets in {len(corr)} corridors '
      f'({", ".join(f"{choice[g[0]].direction}:{len(g)}" for g in corr)})')
print(f'\nINTRA-corridor crossings: {intra}   '
      f'-> priced, via floor {sum(floors.values())}')
print(f'INTER-corridor crossings: {sum(len(v) for v in pairs.values())}'
      f'   -> priced NOWHERE')
if not pairs:
    print('  (none)')
for (i, j), ps in sorted(pairs.items(),
                         key=lambda kv: -len(kv[1])):
    da, dbd = choice[corr[i][0]].direction, choice[corr[j][0]].direction
    # could a layer separate them? only if neither corridor already
    # needs both layers for its own crossings
    la = {dl[n] for n in corr[i]}
    lb = {dl[n] for n in corr[j]}
    if len(la) == 1 and len(lb) == 1 and la != lb:
        fix = 'already on different layers -- free'
    elif len(la) == 1 and len(lb) == 1:
        fix = 'BY LAYER: both ride one layer, and it is the SAME one'
    else:
        nets = {n for p in ps for n in p}
        fix = (f'BY VIAS: {len(nets)} legs involved, both corridors '
               f'already use both layers')
    print(f'  {da}[{len(corr[i])}] x {dbd}[{len(corr[j])}]: '
          f'{len(ps):3d} crossings   {fix}')
