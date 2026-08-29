#!/usr/bin/env python3
"""Homotopy entry-flank evaluator (#622 take-3 task 2).

Per net, entry candidates:
  W        west flank (street / direct / dogbone -- today's emitter)
  N        north flank: run along the field's north edge, descend the
           vertical inter-column street west of the ball, half-pitch jog
  S        south flank, symmetric
  AN / AS  AROUND the field (over the top / under the bottom, along the
           east side), entering from the far flank. Zero crossings with
           everything that stays inside -- but only launch-extreme
           PREFIX (top teeth, around-north) / SUFFIX (bottom teeth,
           around-south) groups can leave the comb without crossing a
           sibling. Buys planarity with length (the human F-set move).

Entry order (the boundary cycle at the field):
  [N entrants, farthest turn-x first] + [W entrants, ball order] +
  [S entrants, nearest turn-x first]; around nets leave the permutation.

Cost = 2*|divers| + 2*|promoted W-dogbones| (LIS over the remaining
permutation; W street capacity via the same greedy chain the emitter
uses), reported against total extra L1 length vs all-west. Prints the
Pareto frontier. Plan-space only -- nothing is emitted.
"""
import argparse
import itertools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', default=os.path.join(HERE,
                    'fb_t2q_base.kicad_pcb'))
    ap.add_argument('--nets', default=('SDQ15,SDQ14,SDQ13,SDQ11,'
                                       'SDQ0,SDQM0,SDQ12,SDQ8'))
    ap.add_argument('--max-around', type=int, default=2,
                    help='max nets around each of top/bottom')
    a = ap.parse_args()
    names = [n.strip() for n in a.nets.split(',') if n.strip()]

    pcb = parse_kicad_pcb(a.board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    ends = te.endpoints(pcb, names, byname)

    comps = {ends[nm][2] for nm in names}
    fpads = [p for c in comps for p in pcb.footprints[c].pads]
    rows = sorted({round(p.global_y, 3) for p in fpads})
    cols = sorted({round(p.global_x, 3) for p in fpads})
    pitch = rows[1] - rows[0]
    north_y = rows[0] - pitch
    south_y = rows[-1] + pitch
    east_x = cols[-1] + pitch
    x1 = cols[0] - pitch * 0.8

    launch = sorted(names, key=lambda nm: ends[nm][0][1])
    tooth = {nm: ends[nm][0] for nm in names}
    ball = {nm: ends[nm][1] for nm in names}

    def l1(p, q):
        return abs(p[0] - q[0]) + abs(p[1] - q[1])

    # per-net candidates: (flank, turn_x, extra_len)
    cands = {}
    for nm in names:
        bx, by = ball[nm]
        w_len = l1(tooth[nm], ball[nm])
        c = [('W', None, 0.0)]
        sx = bx - pitch / 2
        n_len = (abs(tooth[nm][1] - north_y) + abs(sx - tooth[nm][0])
                 + (by - north_y) + pitch / 2)
        c.append(('N', sx, n_len - w_len))
        s_len = (abs(tooth[nm][1] - south_y) + abs(sx - tooth[nm][0])
                 + (south_y - by) + pitch / 2)
        c.append(('S', sx, s_len - w_len))
        # around lengths (entered from the far side, along the east edge)
        an_len = (abs(tooth[nm][1] - north_y) + (east_x - tooth[nm][0])
                  + (south_y - north_y) + (east_x - sx)
                  + (south_y - by) + pitch / 2)
        as_len = (abs(tooth[nm][1] - south_y) + (east_x - tooth[nm][0])
                  + (south_y - north_y) + (east_x - sx)
                  + (by - north_y) + pitch / 2)
        c.append(('AN', sx, an_len - w_len))
        c.append(('AS', sx, as_len - w_len))
        cands[nm] = c

    fx0 = cols[0]
    half = pitch / 2

    launch_idx = {nm: i for i, nm in enumerate(launch)}

    def w_capacity(w_nets):
        """Greedy W entry assignment: iterate by (ball row, LAUNCH
        position) and try north street / direct (column A) / south
        street -- launch-aware so same-row groups take streets in
        launch order (fewer manufactured inversions). 0.3 pairwise
        min like the emitter. Returns (dogbones, {nm: entry_y}) with
        dogbones slotted at their real between-lanes y."""
        taken = []
        entry_y = {}
        dogbone = set()
        for nm in sorted(w_nets, key=lambda n: (round(ball[n][1], 2),
                                                launch_idx[n])):
            bx, by = ball[nm]
            opts = [by - half] + ([by] if bx <= fx0 + 0.01 else []) + \
                [by + half]
            for ey in opts:
                if all(abs(ey - v) >= 0.3 for v in taken):
                    taken.append(ey)
                    entry_y[nm] = ey
                    break
            else:
                dogbone.add(nm)
        for nm in dogbone:      # real lane: midway between flanking Fs
            sy = ball[nm][1] + half
            lo = max([v for v in taken if v <= sy], default=sy - 0.4)
            hi = min([v for v in taken if v > sy], default=sy + 0.4)
            entry_y[nm] = (lo + hi) / 2
        return dogbone, entry_y

    def evaluate(assign):
        """assign: {nm: (flank, turn_x, extra)} -> (vias, extra_len) or
        None if invalid."""
        top_i = 0
        while top_i < len(launch) and assign[launch[top_i]][0] == 'AN':
            top_i += 1
        bot_i = len(launch)
        while bot_i > 0 and assign[launch[bot_i - 1]][0] == 'AS':
            bot_i -= 1
        inside = launch[top_i:bot_i]
        for nm in inside:
            if assign[nm][0] in ('AN', 'AS'):
                return None       # around only as launch prefix/suffix
        nblock = [nm for nm in inside if assign[nm][0] == 'N']
        sblock = [nm for nm in inside if assign[nm][0] == 'S']
        wblock = [nm for nm in inside if assign[nm][0] == 'W']
        # distinct streets per off-flank
        if len({assign[nm][1] for nm in nblock}) != len(nblock):
            return None
        if len({assign[nm][1] for nm in sblock}) != len(sblock):
            return None
        dogbone, entry_y = w_capacity(wblock)
        order = (sorted(nblock, key=lambda n: -assign[n][1])
                 + sorted(wblock, key=lambda n: entry_y[n])
                 + sorted(sblock, key=lambda n: assign[n][1]))
        rank = {nm: i for i, nm in enumerate(order)}
        seq = [rank[nm] for nm in inside]
        keep = te.lis_keep(seq) if seq else set()
        divers = {inside[i] for i in range(len(inside)) if i not in keep}
        promoted = dogbone - divers
        # in-field crossings: a flank descent at sx spanning a y-range
        # crosses a W street run (entry_y, x1..bx) F-on-F -> the descent
        # must go on B.Cu (empty under the field), dogbone surface: +2
        b_descent = set()
        for nm in names:
            fl, sx, _e = assign[nm]
            if fl == 'W':
                continue
            by = ball[nm][1]
            if fl in ('N', 'AS'):
                ylo, yhi = north_y, by
            else:
                ylo, yhi = by, south_y
            for wm in wblock:
                if wm in dogbone:
                    continue
                ey = entry_y[wm]
                if ylo < ey < yhi and x1 < sx < ball[wm][0]:
                    b_descent.add(nm)
                    break
        vias = 0
        for nm in names:
            if nm in divers or nm in b_descent or nm in promoted:
                vias += 2
        extra = sum(assign[nm][2] for nm in names)
        return vias, extra, divers, promoted, b_descent

    # enumerate: around prefix/suffix sizes x per-net W/N/S for the rest
    results = []
    n = len(names)
    for k_top in range(0, a.max_around + 1):
        for k_bot in range(0, a.max_around + 1):
            if k_top + k_bot >= n:
                continue
            mid = launch[k_top:n - k_bot]
            for combo in itertools.product(*[
                    [c for c in cands[nm] if c[0] in ('W', 'N', 'S')]
                    for nm in mid]):
                assign = {}
                for i in range(k_top):
                    nm = launch[i]
                    assign[nm] = next(c for c in cands[nm]
                                      if c[0] == 'AN')
                for i in range(k_bot):
                    nm = launch[n - 1 - i]
                    assign[nm] = next(c for c in cands[nm]
                                      if c[0] == 'AS')
                for nm, c in zip(mid, combo):
                    assign[nm] = c
                r = evaluate(assign)
                if r is None:
                    continue
                vias, extra, divers, promoted, b_desc = r
                results.append((vias, extra, assign, divers, promoted,
                                b_desc))

    print(f'{len(results)} valid assignments evaluated')
    # Pareto frontier on (vias, extra)
    results.sort(key=lambda r: (r[0], r[1]))
    frontier = []
    best_extra = 1e18
    seen_v = set()
    for r in results:
        if r[0] in seen_v:
            continue
        seen_v.add(r[0])
        if r[1] < best_extra:
            best_extra = r[1]
        frontier.append(r)
    print('\nPareto frontier (vias, extra length):')
    for vias, extra, assign, divers, promoted, b_desc in frontier:
        moves = {nm: assign[nm][0] for nm in names
                 if assign[nm][0] != 'W'}
        print(f'  {vias:2d} vias  +{extra:6.2f} mm   moves={moves or "none"}'
              f'  divers={sorted(divers)}'
              + (f' promoted={sorted(promoted)}' if promoted else '')
              + (f' B-descent={sorted(b_desc)}' if b_desc else ''))


if __name__ == '__main__':
    main()
