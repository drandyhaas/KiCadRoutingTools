#!/usr/bin/env python3
"""Eco-overlay render for plan-vs-copper debugging.

Layers drawn (bottom to top): mm grid + rulers, board outline (Edge.Cuts),
pads (yellow; routed-set pads brighter), bench copper (faint by layer),
ROUTED-set copper (saturated by layer), vias, then the braid's plan
overlay (braid.py writes it): Eco2 connection ENDS in yellow -- the
fanout's free ends (source teeth, stub ends) as an "x", the braid's own
exit points (port leave points, flank-run ends) as a "+" -- Cmts the
planned UNDER-PASSES in orange (where the schedule requires the back
layer), Eco1 the planned lane CENTRELINES in white. Copper that leaves
its white line is the router disagreeing with the plan; a via outside
an orange stretch is one the schedule did not ask for.

usage: render_eco.py BOARD OUT.png [--nets N1,N2,...] [--view x0,y0,x1,y1]
"""
import argparse
import re
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('board')
ap.add_argument('out')
ap.add_argument('--nets', default='',
                help='comma-separated short names to highlight')
ap.add_argument('--view', default='116,52,152,80')
a = ap.parse_args()
X0, Y0, X1, Y1 = map(float, a.view.split(','))
W = 1800
S = W / (X1 - X0)
H = int((Y1 - Y0) * S)
M = 26  # ruler margin px

pcb = parse_kicad_pcb(a.board)
hi_names = {n.strip() for n in a.nets.split(',') if n.strip()}
hi_ids = {i for i, n in pcb.nets.items()
          if n.name.split('/')[-1] in hi_names}

img = Image.new('RGB', (W + M, H + M), (14, 18, 14))
dr = ImageDraw.Draw(img, 'RGBA')


def px(x, y):
    return ((x - X0) * S + M, (y - Y0) * S + M)


# mm grid + rulers
for mm in range(int(X0), int(X1) + 1):
    x, _ = px(mm, Y0)
    major = (mm % 5 == 0)
    dr.line([(x, M), (x, H + M)],
            fill=(255, 255, 255, 26 if major else 10))
    if major:
        dr.text((x - 7, 2), str(mm), fill=(180, 180, 180))
for mm in range(int(Y0), int(Y1) + 1):
    _, y = px(X0, mm)
    major = (mm % 5 == 0)
    dr.line([(M, y), (W + M, y)],
            fill=(255, 255, 255, 26 if major else 10))
    if major:
        dr.text((2, y - 6), str(mm), fill=(180, 180, 180))

# board outline + eco lines from the file text, parsed PER gr_line block
txt = open(a.board, encoding='utf-8', errors='replace').read()
eco1, eco2, cmts, edge = [], [], [], []
for chunk in txt.split('(gr_line')[1:]:
    chunk = chunk[:400]
    ms = re.search(r'\(start ([\-\d.]+) ([\-\d.]+)\)', chunk)
    me = re.search(r'\(end ([\-\d.]+) ([\-\d.]+)\)', chunk)
    ml = re.search(r'\(layer "([^"]+)"\)', chunk)
    if not (ms and me and ml):
        continue
    rec = (float(ms.group(1)), float(ms.group(2)),
           float(me.group(1)), float(me.group(2)))
    if ml.group(1) == 'Eco1.User':
        eco1.append(rec)
    elif ml.group(1) == 'Eco2.User':
        eco2.append(rec)
    elif ml.group(1) == 'Cmts.User':
        cmts.append(rec)
    elif ml.group(1) == 'Edge.Cuts':
        edge.append(rec)
for (x1, y1, x2, y2) in edge:
    dr.line([px(x1, y1), px(x2, y2)], fill=(235, 235, 210, 200), width=2)

# pads: yellow, routed-set pads brighter; TH pads get a drill dot
for ref, fp in pcb.footprints.items():
    for p in fp.pads:
        on_f = any(L in ('F.Cu',) or '*' in L for L in p.layers)
        on_b = any(L in ('B.Cu',) or '*' in L for L in p.layers)
        if not (on_f or on_b):
            continue
        cx, cy = px(p.global_x, p.global_y)
        rx = max(1.5, p.size_x / 2 * S)
        ry = max(1.5, p.size_y / 2 * S)
        bright = p.net_id in hi_ids
        col = (240, 220, 90, 230) if bright else (170, 160, 90, 130)
        if p.shape in ('circle', 'oval'):
            dr.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=col)
        else:
            dr.rectangle([cx - rx, cy - ry, cx + rx, cy + ry], fill=col)
        if p.drill and p.drill > 0:
            r = p.drill / 2 * S
            dr.ellipse([cx - r, cy - r, cx + r, cy + r],
                       fill=(30, 30, 30, 255))

# copper: bench faint, routed set saturated
LCF = {'F.Cu': (170, 60, 60, 90), 'B.Cu': (60, 95, 190, 90)}
LCH = {'F.Cu': (255, 70, 70, 255), 'B.Cu': (80, 140, 255, 255)}
for pass_hi in (False, True):
    for s in pcb.segments:
        if (s.net_id in hi_ids) != pass_hi:
            continue
        c = (LCH if pass_hi else LCF).get(s.layer)
        if c is None:
            continue
        w = max(1, int(s.width * S * (1.0 if pass_hi else 0.6)))
        dr.line([px(s.start_x, s.start_y), px(s.end_x, s.end_y)],
                fill=c, width=w)
for v in pcb.vias:
    x, y = px(v.x, v.y)
    r = v.size / 2 * S
    bright = v.net_id in hi_ids
    dr.ellipse([x - r, y - r, x + r, y + r],
               outline=(255, 255, 255, 255) if bright
               else (200, 200, 200, 120),
               width=2 if bright else 1)

# targets (yellow) under, planned UNDER-PASS windows (orange, the
# other-layer stretches) above them, corridor centerlines (white) on top
for (x1, y1, x2, y2) in eco2:
    dr.line([px(x1, y1), px(x2, y2)], fill=(255, 210, 40, 235), width=2)
for (x1, y1, x2, y2) in cmts:
    dr.line([px(x1, y1), px(x2, y2)], fill=(255, 120, 0, 255), width=3)
for (x1, y1, x2, y2) in eco1:
    dr.line([px(x1, y1), px(x2, y2)], fill=(255, 255, 255, 220), width=4)

img.save(a.out)
print(f"render_eco: {a.out} eco1(lanes)={len(eco1)} "
      f"eco2(ends)={len(eco2) // 2} cmts(underpass)={len(cmts)} "
      f"edge={len(edge)} highlighted nets={sorted(hi_names) or 'none'}")
