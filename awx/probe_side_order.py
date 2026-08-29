#!/usr/bin/env python3
"""Is the corridor via floor the REAL floor?

The floor is 2*(K - a), where `a` is the largest set of corridor legs
that pairwise do not cross. Corridor.keep() finds that set by proposing
one from a 1-D projection (the transverse axis of the bundle's mean
travel) and then pruning it against the drawn legs, so what it returns
is always genuinely crossing-free -- but it need not be the LARGEST
such set, and if it is smaller the floor is overstated.

This measures the gap. It builds the crossing graph from the same legs
and computes the true maximum independent set by search, then compares.

It also prints how well each candidate 1-D ordering predicts the
geometry, because that is what decides how good the proposal is. Three
of these were tried as THE model and each is wrong somewhere:

  launch-y/exit-*   assumes the launches are a vertical comb. The
                    `down` corridor here launches from a horizontal one.
  angle-about-array assumes the bundle wraps the array. That corridor
                    approaches the bottom edge head-on and wraps
                    nothing.
  transverse        needs neither, and measured best -- but not exact.

usage: probe_side_order.py [K]
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


def max_independent(grp, adj):
    """Largest pairwise non-crossing subset, exactly. Branch on the
    highest-degree net: either it is in the set (and its neighbours are
    out) or it is not."""
    memo = {}

    def rec(rest):
        if not rest:
            return 0
        key = frozenset(rest)
        if key in memo:
            return memo[key]
        v = max(rest, key=lambda n: len(adj[n] & rest))
        if not (adj[v] & rest):                    # isolated: always take
            r = 1 + rec(rest - {v})
        else:
            r = max(1 + rec(rest - {v} - adj[v]), rec(rest - {v}))
        memo[key] = r
        return r

    return rec(frozenset(grp))


def keyed(grp):
    """Candidate 1-D orderings, each as (launch key, exit key)."""
    ex = {n: choice[n].exit_pt for n in grp}
    dx = sum(ex[n][0] - launch[n][0] for n in grp) / len(grp)
    dy = sum(ex[n][1] - launch[n][1] for n in grp) / len(grp)
    h = math.hypot(dx, dy) or 1.0
    tx, ty = -dy / h, dx / h
    cx = (grid0.bbox[0] + grid0.bbox[2]) / 2.0
    cy = (grid0.bbox[1] + grid0.bbox[3]) / 2.0

    def ang(p):
        return math.atan2(p[1] - cy, p[0] - cx)

    return {
        'launch-y/exit-y': ({n: launch[n][1] for n in grp},
                            {n: ex[n][1] for n in grp}),
        'launch-y/exit-x': ({n: launch[n][1] for n in grp},
                            {n: ex[n][0] for n in grp}),
        'angle-about-array': ({n: ang(launch[n]) for n in grp},
                              {n: ang(ex[n]) for n in grp}),
        'transverse': ({n: launch[n][0] * tx + launch[n][1] * ty
                        for n in grp},
                       {n: ex[n][0] * tx + ex[n][1] * ty for n in grp}),
    }, (dx, dy)


ok = True
for grp in sm.corridors(choice):
    if len(grp) < 2:
        continue
    side = choice[grp[0]].direction
    adj = {n: set() for n in grp}
    real = {}
    for i, a in enumerate(grp):
        for b in grp[i + 1:]:
            c = geo.crosses(a, b, choice)
            real[(a, b)] = c
            if c:
                adj[a].add(b)
                adj[b].add(a)
    best = max_independent(grp, adj)
    kept = geo.keep(grp, choice)
    cands, (dx, dy) = keyed(grp)
    print(f'corridor[{len(grp):2d}] {side:5s}: bundle travels '
          f'({dx:+.1f}, {dy:+.1f}) mm, {sum(real.values())} of '
          f'{len(real)} pairs cross')
    print(f'    keep() holds {len(kept):2d} nets -> floor '
          f'{2 * (len(grp) - len(kept)):2d}      '
          f'best possible {best:2d} -> floor {2 * (len(grp) - best):2d}'
          + ('   OPTIMAL' if len(kept) == best
             else f'   OVERSTATED by {2 * (best - len(kept))}'))
    if len(kept) != best:
        ok = False
    for k, (lk, ek) in cands.items():
        n_w = sum(1 for (a, b), r in real.items()
                  if (((lk[a] < lk[b]) != (ek[a] < ek[b])) != r))
        mark = '   <- Corridor proposes with this' if k == 'transverse' \
               else ''
        print(f'      {k:18s} mispredicts {n_w:3d} of {len(real)} '
              f'pairs{mark}')
    print()
print('every corridor floor is the true minimum' if ok
      else 'at least one corridor floor is OVERSTATED')
