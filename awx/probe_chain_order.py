#!/usr/bin/env python3
"""Did the fanout keep the plan's exit ORDER, or only its direction?

escape_dir_hints carries a SIDE. The via floor is not made of sides --
it is made of the launch->exit permutation, and which exit LINE each
ball takes is what fixes that permutation. If the fanout obeys the
direction and then picks its own row gap, the plan's optimised order is
thrown away and the braid inherits a permutation nobody chose.

This compares, on a chained board:

  the order the PLAN intended     (its chosen exit points)
  the order the FANOUT produced   (where the stubs actually end)

as an inversion count against the launch order, and as the via floor
each implies. usage: probe_chain_order.py FANOUT_BOARD [K]
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
import plan_ends as pe  # noqa: E402

board = sys.argv[1]
K = sys.argv[2] if len(sys.argv) > 2 else '11'
base = os.path.join(HERE, 'fb_t2q_base.kicad_pcb')
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]

# --- the plan, from the bare bench (what chain_k.sh planned)
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
dref = ends[names[0]][2]
dgrid = em.grid_of(pcb.footprints[dref])
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
tooth0 = {n: 'F.Cu' for n in names}
dmenu = {n: [m for m in ms if m.direction == 'left']
         for n, ms in dmenu.items()}
_s, choice, lp, _r = pe.plan_ends(smenu, dmenu, launch, sgrid.bbox,
                                  dgrid.bbox, buses=buses,
                                  tooth_layer0=tooth0)

# --- what the fanout actually produced
fpcb = parse_kicad_pcb(board)
fby = {n.name.split('/')[-1]: (i, n) for i, n in fpcb.nets.items()}
fends = te.endpoints(fpcb, names, fby, dest_ref=dref)


def inversions(order, key):
    ks = [key[n] for n in order]
    return sum(1 for i in range(len(ks)) for j in range(i + 1, len(ks))
               if ks[i] > ks[j])


order = sorted(names, key=lambda n: launch[n][1])
plan_y = {n: choice[n].exit_pt[1] for n in names if n in choice}
real_y = {n: fends[n][1][1] for n in names}
order = [n for n in order if n in plan_y]
pi = inversions(order, plan_y)
ri = inversions(order, real_y)
print(f'K={K}, {len(order)} nets on the west side\n')
print(f'inversions against the launch order:')
print(f'  the plan intended   {pi:4d}   -> via floor about {2 * pi if pi < len(order) else "?"}')
print(f'  the fanout produced {ri:4d}')
print(f'\n{"net":9s} {"plan exit y":>12s} {"actual stub y":>14s}  moved')
n_moved = 0
for n in order:
    d = real_y[n] - plan_y[n]
    if abs(d) > 0.05:
        n_moved += 1
    print(f'{n:9s} {plan_y[n]:12.3f} {real_y[n]:14.3f}  '
          f'{d:+7.3f}{"  <-" if abs(d) > 0.05 else ""}')
print(f'\n{n_moved} of {len(order)} stubs did NOT land on the exit line the '
      f'plan chose.')
print('escape_dir_hints carries a SIDE only, so the fanout picks the row '
      'gap -- and the row gap IS the permutation.')
