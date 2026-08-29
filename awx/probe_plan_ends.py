#!/usr/bin/env python3
"""Choosing BOTH ends against choosing only the berth end.

Paired: the same nets, the same menus, the same weights, the only
difference being whether the source escape is a decision or is read off
the stub the fanout already laid. Reports the whole-plan via floor and
the crossings for each arm, so a gain in one is not hiding a loss in
the other.

usage: probe_plan_ends.py [K]
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
import plan_ends as pe  # noqa: E402

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
dmenu, launch, src_pad = {}, {}, {}
for nm in names:
    nid, net = byname[nm]
    fp = pcb.footprints[ends[nm][2]]
    bx, by = ends[nm][1]
    pad = min(fp.pads, key=lambda p: (p.global_x - bx) ** 2
              + (p.global_y - by) ** 2)
    dmenu[nm] = em.enumerate_moves(
        pad, em.grid_of(fp), LAYERS,
        lambda p, q, L, _n=nid: obs(_n, L).seg_clear(p, q),
        lambda p, L, _n=nid: not (obs(_n, L).point_violation(
            p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])
    launch[nm] = ends[nm][0]
    others = [p for p in net.pads if p.component_ref != ends[nm][2]]
    src_pad[nm] = others[0] if others else None

dgrid = em.grid_of(pcb.footprints[ends[names[0]][2]])
refs = {}
for nm in names:
    if src_pad[nm] is not None:
        refs[src_pad[nm].component_ref] = refs.get(
            src_pad[nm].component_ref, 0) + 1
sref = max(refs, key=refs.get)
sgrid = em.grid_of(pcb.footprints[sref])
smenu = {}
for nm in names:
    p = src_pad[nm]
    if p is None or p.component_ref != sref:
        continue
    nid = byname[nm][0]
    smenu[nm] = em.enumerate_moves(
        p, sgrid, LAYERS,
        lambda a, b, L, _n=nid: obs(_n, L).seg_clear(a, b),
        lambda a, L, _n=nid: not (obs(_n, L).point_violation(
            a, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])

paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
buses = db.cluster(names, paths)
tooth0 = {}
for nm in names:
    nid = byname[nm][0]
    tp = ends[nm][0]
    tooth0[nm] = next(
        (s.layer for s in pcb.segments if s.net_id == nid
         and (abs(s.start_x - tp[0]) + abs(s.start_y - tp[1]) < 0.005
              or abs(s.end_x - tp[0]) + abs(s.end_y - tp[1]) < 0.005)),
        'F.Cu')


def grade(choice, lp, tag):
    geo = sm.Corridor(dgrid.bbox, lp)
    placed = [n for n in choice]
    xs = sum(1 for i, a in enumerate(placed) for b in placed[i + 1:]
             if geo.crosses(a, b, choice))
    corr = sm.corridors(choice)
    of = {n: i for i, g in enumerate(corr) for n in g}
    inter = sum(1 for i, a in enumerate(placed) for b in placed[i + 1:]
                if of[a] != of[b] and geo.crosses(a, b, choice))
    shape = ','.join(f'{choice[g[0]].direction}:{len(g)}' for g in corr)
    print(f'{tag:22s} floor {sm.plan_floor(choice, geo):3d}   '
          f'crossings {xs:4d} ({inter} between corridors)   '
          f'{len(placed)} placed   {shape}')


print(f'K={K}: {len(names)} nets, source {sref}, '
      f'{sum(1 for n in smenu if smenu[n])} with a source menu\n')
off, _ = sm.select(dmenu, launch, keep_out=dgrid.bbox, buses=buses,
                   tooth_layer=tooth0)
grade(off, launch, 'berth end only')

sc, dc, lp, report = pe.plan_ends(smenu, dmenu, launch, sgrid.bbox,
                                  dgrid.bbox, buses=buses,
                                  tooth_layer0=tooth0,
                                  log=(print if '-v' in sys.argv else None))
for line in report:
    print(line)
if dc:
    grade(dc, lp, 'both ends')
    moved = sum(1 for n in dc if lp.get(n) != launch.get(n))
    print(f'{"":22s} {moved} of {len(dc)} teeth moved')
