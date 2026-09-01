#!/usr/bin/env python3
"""Draw the HUMAN's U1 escape paths (thin light red = F.Cu, light
blue = B.Cu, small circle = human via, x = human exit point) on top
of a render of OUR board, so escape conformity can be judged by eye.

usage: hfan_overlay.py K OUR_BOARD OUT.png [--view x0,y0,x1,y1]
       [--human .../00_human_original.kicad_pcb]
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

HUMAN = os.path.expanduser('~/Downloads/bus/00_human_original.kicad_pcb')

ap = argparse.ArgumentParser()
ap.add_argument('K', type=int)
ap.add_argument('board')
ap.add_argument('out')
ap.add_argument('--view', default='112,53,132,74')
ap.add_argument('--human', default=HUMAN)
a = ap.parse_args()

names = subprocess.run(
    [sys.executable, os.path.join(HERE, 'coherent_nets.py'), str(a.K)],
    capture_output=True, text=True).stdout.strip().split(',')[:a.K]

# base image: render_eco on OUR board, same view, K nets highlighted
subprocess.run(
    [sys.executable, os.path.join(HERE, 'render_eco.py'), a.board,
     a.out, '--no-plan', '--nets', ','.join(names),
     '--view', a.view], check=True)

X0, Y0, X1, Y1 = map(float, a.view.split(','))
W = 1800
S = W / (X1 - X0)
M = 26


def px(x, y):
    return ((x - X0) * S + M, (y - Y0) * S + M)


hum = parse_kicad_pcb(a.human)
hby = {n.name.split('/')[-1]: i for i, n in hum.nets.items()}
fp = hum.footprints['U1']
xs = [p.global_x for p in fp.pads]
ys = [p.global_y for p in fp.pads]
m = 1.6
Wn = (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)


def in_w(x, y):
    return Wn[0] <= x <= Wn[2] and Wn[1] <= y <= Wn[3]


img = Image.open(a.out).convert('RGB')
dr = ImageDraw.Draw(img, 'RGBA')
LIGHT = {'F.Cu': (255, 170, 170, 235), 'B.Cu': (170, 215, 255, 235)}
n_seg = 0
for nm in names:
    if nm not in hby:
        continue
    nid = hby[nm]
    for s in hum.segments:
        if s.net_id != nid:
            continue
        if not (in_w(s.start_x, s.start_y) or in_w(s.end_x, s.end_y)):
            continue
        c = LIGHT.get(s.layer)
        if c is None:
            continue
        dr.line([px(s.start_x, s.start_y), px(s.end_x, s.end_y)],
                fill=c, width=2)
        n_seg += 1
    for v in hum.vias:
        if v.net_id != nid or not in_w(v.x, v.y):
            continue
        x, y = px(v.x, v.y)
        r = v.size / 2 * S
        dr.ellipse([x - r, y - r, x + r, y + r],
                   outline=(255, 255, 190, 235), width=2)
img.save(a.out)
print(f'hfan_overlay: {a.out} human segs drawn={n_seg} '
      f'nets={len(names)}')
