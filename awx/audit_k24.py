#!/usr/bin/env python3
"""Detailed K2/K4 plan audit.
Per net: lane endpoint correctness (vs berth tips parsed from LOG),
offset-from-corridor, peel diagonal angle, adherence copper<->lane in
both directions (excluding 1.2mm around endpoints), and the
own-vs-others lane distance matrix.
usage: audit_k24.py BOARD LOG --nets A,B,C"""
import argparse
import json
import math
import re
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('log')
ap.add_argument('--nets', required=True)
a = ap.parse_args()
names = [n.strip() for n in a.nets.split(',')]

pcb = parse_kicad_pcb(a.board)
sj = json.load(open(a.board + '.attract.json'))
lanes = {k.split('/')[-1]: v for k, v in (sj.get('targets') or {}).items()}
corr = list((sj.get('corridors') or {}).values())[0]

tips = {}
for line in open(a.log, encoding='utf-8', errors='replace'):
    m = re.search(r'Berth plan: .*/(\S+) -> tip \(([\d.]+),([\d.]+)\)', line)
    if m:
        tips[m.group(1)] = (float(m.group(2)), float(m.group(3)))


def d_pt_poly(x, y, poly, span_only=False):
    """Distance to polyline. span_only=True returns None for a point
    whose closest approach is the polyline's terminal vertex (i.e. the
    point lies BEYOND the lane's end): the tip->ball exact-leg tail is
    not lane deviation (measured: SDQ14's 2.3mm tail inflated its median
    by counting the leg against a lane that ends at the tip)."""
    best, best_end = 1e9, False
    nseg = len(poly) - 1
    for si, ((x1, y1), (x2, y2)) in enumerate(zip(poly, poly[1:])):
        vx, vy = x2 - x1, y2 - y1
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - x1) * vx
                                                      + (y - y1) * vy) / L2))
        d = math.hypot(x - (x1 + t * vx), y - (y1 + t * vy))
        if d < best:
            best = d
            best_end = (si == nseg - 1 and t >= 1.0 - 1e-9) \
                or (si == 0 and t <= 1e-9)
    if span_only and best_end:
        return None
    return best


def copper_samples(nid):
    out = []
    for s in pcb.segments:
        if s.net_id != nid:
            continue
        L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        n = max(2, int(L / 0.15))
        for i in range(n + 1):
            t = i / n
            out.append((s.start_x + t * (s.end_x - s.start_x),
                        s.start_y + t * (s.end_y - s.start_y)))
    return out


ids = {}
for nm in names:
    ids[nm] = next(i for i, n in pcb.nets.items()
                   if n.name.endswith('/' + nm))

print('=== A. lane endpoint + shape audit ===')
for nm in names:
    tp = lanes.get(nm)
    if not tp:
        print(f'{nm}: NO LANE')
        continue
    tip = tips.get(nm)
    e = tp[-1]
    tip_err = math.hypot(e[0] - tip[0], e[1] - tip[1]) if tip else None
    # peel diagonal: last long jump's angle
    best_j, ang = 0.0, None
    for (x1, y1), (x2, y2) in zip(tp, tp[1:]):
        L = math.hypot(x2 - x1, y2 - y1)
        if L > best_j:
            best_j = L
            dx, dy = abs(x2 - x1), abs(y2 - y1)
            ang = math.degrees(math.atan2(min(dx, dy), max(dx, dy))) if L else 0
    print(f'{nm}: start ({tp[0][0]:.2f},{tp[0][1]:.2f}) '
          f'end ({e[0]:.2f},{e[1]:.2f}) '
          f'tip {tip} err {tip_err if tip_err is None else round(tip_err,3)}mm '
          f'longest-jump {best_j:.2f}mm @{ang:.0f}deg-off-diag45'
          .replace('@', 'angle(min/max)='))

print()
print('=== B. own-vs-others matrix (median copper->lane mm) ===')
hdr = '        ' + '  '.join(f'{nm:>7s}' for nm in names)
print(hdr + '   (cols = lanes)')
for nm in names:
    samples = copper_samples(ids[nm])
    # exclude 1.2mm around the two pads (stub/landing detail)
    pads = pcb.pads_by_net.get(ids[nm], [])
    core = [(x, y) for (x, y) in samples
            if all(math.hypot(x - p.global_x, y - p.global_y) > 1.2
                   for p in pads)]
    row = []
    for lm in names:
        ds = sorted(d_pt_poly(x, y, lanes[lm]) for (x, y) in core)
        row.append(ds[len(ds) // 2] if ds else float('nan'))
    own = row[names.index(nm)]
    best = min(row)
    mark = ' OK' if abs(own - best) < 1e-9 else ' <-- NOT OWN!'
    print(f'{nm:7s} ' + '  '.join(f'{v:7.2f}' for v in row) + mark)

print()
print('=== C. adherence detail (core span, both directions) ===')
for nm in names:
    samples = copper_samples(ids[nm])
    pads = pcb.pads_by_net.get(ids[nm], [])
    core = [(x, y) for (x, y) in samples
            if all(math.hypot(x - p.global_x, y - p.global_y) > 1.2
                   for p in pads)]
    ds = sorted(d for d in (d_pt_poly(x, y, lanes[nm], span_only=True)
                            for (x, y) in core) if d is not None)
    # lane coverage: how much of the lane has copper nearby
    segs_pts = core
    cov = []
    for (lx, ly) in lanes[nm]:
        cov.append(min((math.hypot(lx - x, ly - y) for (x, y) in segs_pts),
                       default=9e9))
    covs = sorted(cov)
    print(f'{nm}: copper->lane med {ds[len(ds)//2]:.2f} p90 '
          f'{ds[int(len(ds)*0.9)]:.2f} max {ds[-1]:.2f} | '
          f'lane->copper med {covs[len(covs)//2]:.2f} '
          f'max {covs[-1]:.2f}mm')
