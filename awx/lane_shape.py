#!/usr/bin/env python3
"""Segment decomposition + pairwise crossing count for drawn lanes."""
import json
import math
import sys

board = sys.argv[1]
sj = json.load(open(board + '.attract.json'))
lanes = {k.split('/')[-1]: v for k, v in sj['targets'].items()}
lanes3 = {k.split('/')[-1]: v
          for k, v in (sj.get('targets3') or {}).items()}


def segs_of(tp):
    segs, cur, d0 = [], [tp[0]], None
    for a, b in zip(tp, tp[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        dd = (0 if abs(dx) < 1e-9 else (1 if dx > 0 else -1),
              0 if abs(dy) < 1e-9 else (1 if dy > 0 else -1))
        if dd == (0, 0):
            continue
        if d0 is None or dd == d0:
            cur.append(b)
            d0 = dd
        else:
            segs.append((cur[0], cur[-1], d0))
            cur = [cur[-1], b]
            d0 = dd
    segs.append((cur[0], cur[-1], d0))
    return segs


def crossings(tp1, tp2, l1=None, l2=None):
    """Count SAME-LAYER transversal crossings. A crossing where the
    two lanes are on different planned layers is an under-pass -- the
    physical via-pair resolution -- not a plan defect."""
    def seg_int(p1, p2, p3, p4):
        d1 = (p2[0] - p1[0], p2[1] - p1[1])
        d2 = (p4[0] - p3[0], p4[1] - p3[1])
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-12:
            return False
        t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / den
        u = ((p3[0] - p1[0]) * d1[1] - (p3[1] - p1[1]) * d1[0]) / den
        return 0.01 < t < 0.99 and 0.01 < u < 0.99
    n = 0
    for i, (a, b) in enumerate(zip(tp1, tp1[1:])):
        for j, (c, d) in enumerate(zip(tp2, tp2[1:])):
            if seg_int(a, b, c, d):
                if l1 and l2 and i < len(l1) and j < len(l2) \
                        and l1[i] != l2[j]:
                    continue
                n += 1
    return n


names = list(lanes)
for nm in names:
    print(f'--- {nm}')
    for (a, b, dd) in segs_of(lanes[nm]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L < 0.15:
            continue
        print(f'  ({a[0]:.2f},{a[1]:.2f})->({b[0]:.2f},{b[1]:.2f}) '
              f'dir{dd} L={L:.2f}')
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        li = [p[2] for p in lanes3.get(names[i], [])] or None
        lj = [p[2] for p in lanes3.get(names[j], [])] or None
        n = crossings(lanes[names[i]], lanes[names[j]], li, lj)
        if n:
            print(f'SAME-LAYER CROSSINGS {names[i]} x {names[j]}: {n}')
print('crossing scan done (same-layer only where layers known)')
