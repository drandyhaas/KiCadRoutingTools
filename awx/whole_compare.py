#!/usr/bin/env python3
"""whole_compare.py OURS HUMAN NETS OUT.png [--src U1] [--dest DU1] [--size 900] -- a rung's routed board beside the
human's, for the README: the run's nets' copper alone on each (NETS: N1,N2,.. or @FILE -- a whole_route.py round's
nets.lines), the same view (the two arrays' pads and that copper, on our board), the differential pairs in yellow, each
half captioned with its vias and copper on those nets.

The two boards must share one frame -- the zynq article's human is the original board turned as the bench was
(flow_frame.py turn ... 1 109.3 -103.2): refused when the arrays stand more than a micron apart on the two boards.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from route_render import BoardRenderer  # noqa: E402
import pairs as _pairs  # noqa: E402
import plan_audit as pa  # noqa: E402

YELLOW = (240, 200, 40)


def short(name):
    return (name or '').split('/')[-1]


def half(pcb, names, legs, view, size):
    """(image, vias, copper mm): `pcb` drawn at `view` with only `names`' copper, `legs` (pair legs) in yellow"""
    ids = {i for i, n in pcb.nets.items() if short(n.name) in names}
    pid = {i for i, n in pcb.nets.items() if short(n.name) in legs}
    segs = [s for s in pcb.segments if s.net_id in ids and not getattr(s, 'graphic', False)]
    vias = [v for v in pcb.vias if v.net_id in ids]
    r = BoardRenderer(pcb, size=size, show_zones=False, view=view, theme='light')
    # (the canvas follows the board's own outline; both halves take the VIEW's shape, so they share one scale)
    r.set_canvas(size, max(1, round(size * (view[3] - view[1]) / (view[2] - view[0]))))
    img = r.frame(segments=[s for s in segs if s.net_id not in pid], vias=[v for v in vias if v.net_id not in pid],
                  highlight_segments=[s for s in segs if s.net_id in pid],
                  highlight_vias=[v for v in vias if v.net_id in pid], highlight_color=YELLOW)
    mm = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)
    return img, len(vias), mm


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('ours')
    ap.add_argument('human')
    ap.add_argument('nets')
    ap.add_argument('out')
    ap.add_argument('--src', default='U1')
    ap.add_argument('--dest', default='DU1')
    ap.add_argument('--size', type=int, default=900)
    a = ap.parse_args()
    names = set(pa.read_nets(a.nets))
    legs = {l_ for pr in _pairs.pair_names(sorted(names)).values() for l_ in pr}
    A, H = parse_kicad_pcb(a.ours), parse_kicad_pcb(a.human)
    for ref in (a.src, a.dest):
        fa, fh = A.footprints[ref], H.footprints[ref]
        if math.hypot(fa.x - fh.x, fa.y - fh.y) > 1e-3:
            sys.exit(f'whole_compare: {ref} stands at ({fa.x:.3f}, {fa.y:.3f}) on ours and ({fh.x:.3f}, {fh.y:.3f}) on '
                     f'the human\'s -- not one frame (turn the human\'s board as the bench was: flow_frame.py turn)')
    xs, ys = [], []
    for ref in (a.src, a.dest):
        for p in A.footprints[ref].pads:
            xs.append(p.global_x)
            ys.append(p.global_y)
    ids = {i for i, n in A.nets.items() if short(n.name) in names}
    for s in A.segments:
        if s.net_id in ids:
            xs += [s.start_x, s.end_x]
            ys += [s.start_y, s.end_y]
    m = 1.0
    view = (min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)
    from PIL import Image, ImageDraw
    halves = [(half(A, names, legs, view, a.size), 'the whole route'), (half(H, names, legs, view, a.size), 'the human')]
    w = sum(im.width for (im, _v, _c), _t in halves)
    h = max(im.height for (im, _v, _c), _t in halves) + 28
    out = Image.new('RGB', (w + 8, h), (255, 255, 255))
    d = ImageDraw.Draw(out)
    x = 0
    for (im, nv, mm), title in halves:
        out.paste(im, (x, 28))
        d.text((x + 8, 8), f'{title}: {nv} vias, {mm:.0f} mm', fill=(0, 0, 0))
        x += im.width + 8
    out.save(a.out)
    print(f'whole_compare: {a.out} -- ours {halves[0][0][1]} vias {halves[0][0][2]:.0f} mm, the human '
          f'{halves[1][0][1]} vias {halves[1][0][2]:.0f} mm, {len(names)} nets ({len(legs)} pair legs in yellow)')


if __name__ == '__main__':
    main()
