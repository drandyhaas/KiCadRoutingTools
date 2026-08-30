#!/usr/bin/env python3
"""How much do the corridors conflict with EACH OTHER?

Every number so far treats a bus in isolation: its own permutation, its
own via floor, its own certified side. But the buses share one board.
Two nets in DIFFERENT buses whose taut routes cross must still be
separated -- by a layer, or by one corridor going around -- and nothing
models that.

This counts crossings between taut pre-routes, split into

  intra-bus   crossings the bus's own braid already resolves, and whose
              cost is the via floor already reported
  inter-bus   crossings BETWEEN corridors, which nothing resolves yet

and reports, per bus pair, whether their source order along the comb
agrees with their destination order around the array. A pair whose
orders agree can be drawn without their corridors crossing at all; a
pair that disagrees has to pay for it.

usage: probe_corridor_conflict.py [K]
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
import topo_strings as ts  # noqa: E402
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


paths = db.taut_paths(names, ends, lambda nm: obs(byname[nm][0], 'F.Cu'))
buses = db.cluster(names, paths)
bus_of = {n: i for i, b in enumerate(buses) for n in b}

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

intra = inter = 0
pair_cross = {}
for i, a in enumerate(names):
    for b in names[i + 1:]:
        c = ts.crossings(paths[a], paths[b])
        n = len(c) if hasattr(c, '__len__') else int(bool(c))
        if not n:
            continue
        if bus_of[a] == bus_of[b]:
            intra += n
        else:
            inter += n
            k = tuple(sorted((bus_of[a], bus_of[b])))
            pair_cross[k] = pair_cross.get(k, 0) + n

print(f'K={K}: {len(names)} nets in {len(buses)} buses')
print(f'  taut-route crossings INTRA-bus: {intra}   '
      f'(the braid resolves these; already in the via floors)')
print(f'  taut-route crossings INTER-bus: {inter}   '
      f'(nothing resolves these yet)')

# order agreement: source position along the comb vs destination
# position around the array boundary
x0, y0, x1, y1 = grid0.bbox


def arc(pt):
    """Position around the array boundary, clockwise from its NW
    corner, so 'order around the destination' is a single number."""
    x, y = pt
    w, h = (x1 - x0), (y1 - y0)
    if abs(y - y0) <= abs(y - y1) and x0 - 1 <= x <= x1 + 1 and y < y0 + h / 2:
        return (x - x0)                      # top edge, left to right
    if x > x0 + w / 2:
        return w + (y - y0)                  # right edge, top to bottom
    if y > y0 + h / 2:
        return w + h + (x1 - x)              # bottom edge, right to left
    return 2 * w + h + (y1 - y)              # left edge, bottom to top


# The unit the CORRIDOR routes is not the taut-path cluster: it is
# every net leaving on one side, since those all share one channel and
# one permutation. Report the floor both ways.
by_side = {}
for n, m in choice.items():
    by_side.setdefault(m.direction, []).append(n)
print('\nvia floor, per taut-path bus vs per SIDE (what a corridor '
      'actually routes):')
per_bus = sum(sm._floor(b, choice, geo) for b in buses
              if len(b) >= 2 and all(n in choice for n in b))
per_side = 0
for d, grp in sorted(by_side.items()):
    if len(grp) < 2:
        continue
    f = sm._floor(grp, choice, geo)
    per_side += f
    print(f'  side {d:5s}: {len(grp):2d} nets, floor {f}')
print(f'  TOTAL per-bus {per_bus}   per-side {per_side}'
      f'   (understated by {per_side - per_bus})')

print('\nbus pairs: does the source order agree with the destination '
      'order?')
info = []
for i, bus in enumerate(buses):
    if not all(n in choice for n in bus):
        continue
    sy = sum(launch[n][1] for n in bus) / len(bus)
    sa = sum(arc(choice[n].exit_pt) for n in bus) / len(bus)
    info.append((i, len(bus), sy, sa, choice[bus[0]].direction))
for a in range(len(info)):
    for b in range(a + 1, len(info)):
        ia, na, sya, saa, da = info[a]
        ib, nb, syb, sab, dbd = info[b]
        agree = (sya < syb) == (saa < sab)
        x = pair_cross.get(tuple(sorted((ia, ib))), 0)
        print(f'  bus[{na:2d}]{da:5s} vs bus[{nb:2d}]{dbd:5s}: '
              f'{"agree" if agree else "DISAGREE"}   '
              f'{x} taut crossings between them')
