#!/usr/bin/env python3
"""How much corridor via cost is the escape CHOICE leaving on the table?

The corridor's via floor is 2*(K - LIS) of the launch->exit permutation.
The selector picks exits on per-net cost and never looks at that, so the
permutation it induces is incidental. Each net has a small number of
feasible exits within its bus's side (which adjacent gap, which kind),
and different combinations give different LIS.

This searches those combinations for the bus and reports the floor the
selector's choice produces against the best reachable one -- i.e. the
corridor vias the homotopy is giving away.

usage: probe_lis_headroom.py [K]
"""
import itertools
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
choice, _un = sm.select(menu, launch, keep_out=grid0.bbox, buses=buses)


def stats(group, exit_of):
    """(inversions, LIS, via floor) of launch order -> exit order."""
    lo = sorted(group, key=lambda n: launch[n][1])
    axis = 1 if exit_of(group[0]).direction in ('left', 'right') else 0
    li = {n: i for i, n in enumerate(lo)}
    # Two nets leaving on the SAME exit line but different layers do not
    # cross, so a tie must not count as an inversion. Breaking ties by
    # launch order expresses exactly that.
    tgt = sorted(group, key=lambda n: (round(exit_of(n).exit_pt[axis], 3),
                                       li[n]))
    tr = {n: i for i, n in enumerate(tgt)}
    ranks = [tr[n] for n in lo]
    inv = sum(1 for i in range(len(ranks))
              for j in range(i + 1, len(ranks)) if ranks[i] > ranks[j])
    lis = len(te.lis_keep(ranks))
    return inv, lis, 2 * (len(group) - lis)


for bus in buses:
    if len(bus) < 3:
        continue
    side = choice[bus[0]].direction
    got = stats(bus, lambda n: choice[n])
    # options for each net WITHIN the bus's chosen side
    opts = {n: [m for m in menu[n] if m.direction == side] for n in bus}
    # collapse to distinct exit coordinates (that is all LIS sees)
    axis = 1 if side in ('left', 'right') else 0
    slim = {}
    for n in bus:
        seen, keep = set(), []
        for m in sorted(opts[n], key=lambda m: m.vias):
            k = round(m.exit_pt[axis], 3)
            if k in seen:
                continue
            seen.add(k)
            keep.append(m)
        slim[n] = keep
    # a shared exit line is legal on different layers, so no
    # distinctness constraint -- only the ORDER matters for the floor
    combos = 1
    for n in bus:
        combos *= max(len(slim[n]), 1)
    print(f'\nbus of {len(bus)} exiting {side}: '
          f'chosen -> {got[0]} inversions, LIS {got[1]}, '
          f'corridor via floor {got[2]}')
    print(f'  distinct exit coordinates per net: '
          + ' '.join(f'{n}:{len(slim[n])}' for n in bus)
          + f'   ({combos} combinations)')
    if combos > 400000:
        print('  too many combinations to search exhaustively')
        continue
    best = None
    for pick in itertools.product(*(slim[n] for n in bus)):
        sel = dict(zip(bus, pick))
        s = stats(bus, lambda n: sel[n])
        if best is None or s[2] < best[0][2]:
            best = (s, sel)
    if best is None:
        print('  no combination gives every net a distinct exit line')
        continue
    print(f'  BEST reachable -> {best[0][0]} inversions, LIS {best[0][1]}, '
          f'corridor via floor {best[0][2]}')
    print(f'  headroom the escape choice is giving away: '
          f'{got[2] - best[0][2]} corridor vias')
