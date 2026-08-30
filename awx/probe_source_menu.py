#!/usr/bin/env python3
"""How much is the SOURCE fanout costing, and is there room to choose?

Every launch point is the free end of a stub the U1 fanout already laid.
Nothing in the plan chose it, so the source order is an input, not a
decision -- and the source order is half of the launch->exit permutation
whose inversions are the via floor.

This asks two questions before anything is changed:

  1. is there a menu at the source at all? U1 is an array like the
     destination, so the same escape enumeration applies to its pads;
     print how many moves each net actually has, and how far the
     existing tooth is from the ones on offer.
  2. what would choosing them be worth? Re-pick the source exits (the
     destination choice held fixed) to maximise the crossing-free set,
     and compare the whole-plan floor against today's.

usage: probe_source_menu.py [K]
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
menu, launch, src_pad = {}, {}, {}
for nm in names:
    nid, net = byname[nm]
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
    # the SOURCE pad is the net's pad on the other component
    dst_ref = ends[nm][2]
    others = [p for p in net.pads if p.component_ref != dst_ref]
    src_pad[nm] = others[0] if others else None

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

# --- 1. is there a menu at the source?
refs = {}
for nm in names:
    if src_pad[nm] is not None:
        refs[src_pad[nm].component_ref] = refs.get(
            src_pad[nm].component_ref, 0) + 1
print(f'K={K}: source components ' +
      ', '.join(f'{r}:{c}' for r, c in sorted(refs.items())))
sref = max(refs, key=refs.get)
sfp = pcb.footprints[sref]
sgrid = em.grid_of(sfp)
print(f'{sref}: {len(sfp.pads)} pads, {len(sgrid.xs)}x{len(sgrid.ys)} '
      f'grid, pitch {sgrid.pitch_x:.3f} x {sgrid.pitch_y:.3f}, '
      f'bbox {tuple(round(v, 2) for v in sgrid.bbox)}')

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
opts = [len(smenu.get(n, ())) for n in names]
have = [o for o in opts if o]
print(f'source escape menu: {len(have)}/{len(names)} nets have moves, '
      f'{min(have) if have else 0}..{max(have) if have else 0} each '
      f'(median {sorted(have)[len(have) // 2] if have else 0})')
dirs = {}
for n, ms in smenu.items():
    for m in ms:
        dirs[m.direction] = dirs.get(m.direction, 0) + 1
print(f'  by direction: ' + ', '.join(f'{d}:{c}'
                                      for d, c in sorted(dirs.items())))
# how far is today's tooth from the offered exits?
far = []
for n, ms in smenu.items():
    if not ms:
        continue
    d = min(math.hypot(m.exit_pt[0] - launch[n][0],
                       m.exit_pt[1] - launch[n][1]) for m in ms)
    far.append((d, n))
far.sort()
if far:
    print(f'  today\'s tooth sits {far[0][0]:.2f}..{far[-1][0]:.2f} mm '
          f'from the nearest offered source exit '
          f'(median {far[len(far) // 2][0]:.2f})')

# --- 2. what would choosing them be worth?
placed = [n for n in names if n in choice]


def exact_floor(lp):
    """2*(N - MIS) from the true crossing graph, so a small gain is not
    hidden by keep()'s proposal being imperfect."""
    g = sm.Corridor(grid0.bbox, lp)
    adj = {n: set() for n in placed}
    for i, a in enumerate(placed):
        for b in placed[i + 1:]:
            if g.crosses(a, b, choice):
                adj[a].add(b)
                adj[b].add(a)
    memo = {}

    def rec(rest):
        if not rest:
            return 0
        key = frozenset(rest)
        if key in memo:
            return memo[key]
        v = max(rest, key=lambda n: len(adj[n] & rest))
        if not (adj[v] & rest):
            r = 1 + rec(rest - {v})
        else:
            r = max(1 + rec(rest - {v} - adj[v]), rec(rest - {v}))
        memo[key] = r
        return r

    return 2 * (len(placed) - rec(frozenset(placed))), adj


EXACT = len(placed) <= 24      # the MIS search is exponential


def floor_of(lp):
    if EXACT:
        return exact_floor(lp)[0]
    g = sm.Corridor(grid0.bbox, lp)
    return 2 * (len(placed) - len(g.keep(placed, choice)))


now_h = sm.plan_floor(choice, geo)
xs0 = sum(1 for i, a in enumerate(placed) for b in placed[i + 1:]
          if geo.crosses(a, b, choice))
print(f'\nwhole-plan via floor today: {floor_of(launch)}'
      + (' exact' if EXACT else ' (keep(), too many nets for exact)')
      + f'   [{now_h} from keep(), {xs0} crossings]')

# COORDINATE DESCENT over the source exits, several rounds. One pass is
# not a measurement: the same single-net hill climb stalled in
# refine_lis, because reaching a better assignment needs several nets to
# move together. Ties are accepted so a plateau can be walked across.
best_launch = dict(launch)
cur = floor_of(best_launch)
for rnd in range(6):
    moved_this = 0
    for n in sorted(placed, key=lambda n: -len(smenu.get(n, ()))):
        for m in smenu.get(n, ()):
            if m.exit_pt == best_launch[n]:
                continue
            cand = dict(best_launch)
            cand[n] = m.exit_pt
            f = floor_of(cand)
            if f <= cur and (f < cur or m.exit_pt != launch[n]):
                if f < cur:
                    moved_this += 1
                best_launch, cur = cand, f
                # no break: keep walking this net's remaining moves.
                # Ties are what make this work -- a better source order
                # needs several nets to move together, so stopping at
                # the first acceptable move pins the net wherever the
                # plateau was entered and the search reports nothing to
                # do. (This loop originally had a `break` here guarded
                # on `f < cur` AFTER cur had been set to f, so it could
                # never fire; the plateau walk it was accidentally
                # doing is the behaviour that matters, and is now
                # deliberate here and in plan_ends.refine_source.)
    if not moved_this:
        break
moved = sum(1 for n in placed if best_launch[n] != launch[n])
print(f'whole-plan via floor if the SOURCE exits were chosen too: '
      f'{cur}   ({moved} of {len(placed)} teeth moved, '
      f'{rnd + 1} rounds)')
