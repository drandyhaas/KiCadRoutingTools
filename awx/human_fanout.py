#!/usr/bin/env python3
"""How does a board FAN OUT the destination array? Per net: the ball,
the escape kind (via-in-pad / dogbone / far-via / no-via), where the
via sits relative to the ball, the layer the net TRAVELS on when it
leaves the array window, and the face it leaves by. Totals by
kind, layer and face. usage: human_fanout.py BOARD NET,NET,...
[--ref DU1] [--margin 1.6]"""
import argparse
import math
import os
import sys

HERE = '/Users/andy/Documents/KiCadRoutingTools/.claude/worktrees/bus622-take3/awx'
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('nets')
ap.add_argument('--ref', default='DU1')
ap.add_argument('--margin', type=float, default=1.6)
a = ap.parse_args()
pcb = parse_kicad_pcb(a.board)
byname = {n.name.split('/')[-1]: i for i, n in pcb.nets.items()}
fp = pcb.footprints[a.ref]
xs = [p.global_x for p in fp.pads]
ys = [p.global_y for p in fp.pads]
bx0, by0, bx1, by1 = min(xs), min(ys), max(xs), max(ys)
M = a.margin
DIRS = {'left': (-1, 0), 'right': (1, 0), 'up': (0, -1), 'down': (0, 1)}
tot_kind, tot_lay, tot_face = {}, {}, {}
print(f'{a.ref} bbox ({bx0:.1f},{by0:.1f})..({bx1:.1f},{by1:.1f})')
print(f'{"net":8s} {"kind":11s} {"via@":14s} {"travel":6s} face')
for nm in a.nets.split(','):
    nid = byname.get(nm)
    if nid is None:
        print(f'{nm:8s} NOT ON BOARD')
        continue
    ball = min((p for p in fp.pads if p.net_id == nid),
               key=lambda p: 0, default=None)
    if ball is None:
        print(f'{nm:8s} no ball on {a.ref}')
        continue
    bx, by = ball.global_x, ball.global_y
    segs = [s for s in pcb.segments if s.net_id == nid
            and bx0 - M <= (s.start_x + s.end_x) / 2 <= bx1 + M
            and by0 - M <= (s.start_y + s.end_y) / 2 <= by1 + M]
    vias = [v for v in pcb.vias if v.net_id == nid
            and bx0 - M <= v.x <= bx1 + M and by0 - M <= v.y <= by1 + M]
    vnear = min(vias, key=lambda v: math.hypot(v.x - bx, v.y - by),
                default=None)
    if vnear is None:
        kind, vat = 'no-via', '-'
    else:
        d = math.hypot(vnear.x - bx, vnear.y - by)
        kind = ('via-in-pad' if d < 0.22 else
                'dogbone' if d < 0.85 else 'far-via')
        vat = f'{d:.2f} ({vnear.x - bx:+.2f},{vnear.y - by:+.2f})'
    # the copper that LEAVES the window: farthest end from the ball,
    # its layer and face
    if not segs:
        print(f'{nm:8s} {kind:11s} {vat:14s} {"none":6s} -')
        continue
    far, farl = max(((pt, s.layer) for s in segs
                     for pt in ((s.start_x, s.start_y), (s.end_x, s.end_y))),
                    key=lambda q: (q[0][0] - bx) ** 2 + (q[0][1] - by) ** 2)
    dx, dy = far[0] - bx, far[1] - by
    h = math.hypot(dx, dy) or 1
    face = min(DIRS, key=lambda k: (DIRS[k][0] - dx / h) ** 2
               + (DIRS[k][1] - dy / h) ** 2)
    lay = farl[0]
    print(f'{nm:8s} {kind:11s} {vat:14s} {lay:6s} {face}')
    tot_kind[kind] = tot_kind.get(kind, 0) + 1
    tot_lay[lay] = tot_lay.get(lay, 0) + 1
    tot_face[(face, lay)] = tot_face.get((face, lay), 0) + 1
print('kinds:', dict(sorted(tot_kind.items())),
      ' travel:', dict(sorted(tot_lay.items())))
print('face x layer:', dict(sorted(tot_face.items())))
