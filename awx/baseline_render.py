#!/usr/bin/env python3
"""baseline_render.py OUT.png NETS LABEL=BOARD ... [--cols 2] [--no-others] -- boards of one rung drawn alike in a grid,
for baseline_bench.py's comparisons (and the paper's figure of them): the rung's nets (NETS: N1,N2,.. or a file of them)
with F red, B blue, a differential pair's legs orange and every via the SAME marker (its drawn size differs between
routers); every other net's tracks and vias grey -- the obstacles, to see that each router kept off them
(--no-others leaves them out). One view for every panel: the rung's copper on all the boards, a millimetre round it."""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

PAIR = (230, 140, 0)
GREY = (185, 185, 185)
VIA_MM = 0.5                  # every via's marker diameter
VIA_RING = (40, 40, 40)
SIZE, SS = 1800, 3
FONTS = ('/System/Library/Fonts/Helvetica.ttc', 'DejaVuSans.ttf', 'arial.ttf')


def short(name):
    return (name or '').split('/')[-1]


def load(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


def rung(pcb, names):
    """(the rung's segments, its vias, the ids of its pair legs)"""
    ids = {i for i, n in pcb.nets.items() if short(n.name) in names}
    bases = {n[:-1] for n in names if n[-1] in 'PN' and n[:-1] + ('N' if n[-1] == 'P' else 'P') in names}
    legs = {i for i in ids if short(pcb.nets[i].name)[:-1] in bases}
    return [s for s in pcb.segments if s.net_id in ids], [v for v in pcb.vias if v.net_id in ids], legs


def theme():
    import render_theme
    L = render_theme.LIGHT
    return render_theme.Theme('paper', {**L._rgb, 'ground': (255, 255, 255), 'board_body': (255, 255, 255),
                                        'board_edge': (200, 200, 200), 'pad': (175, 160, 120)},
                              L._mark, L.layers, 255)


def panel(pcb, names, view, others):
    from route_render import BoardRenderer
    segs, vias, legs = rung(pcb, names)
    ids = {s.net_id for s in segs} | {v.net_id for v in vias} | {i for i, n in pcb.nets.items() if short(n.name) in names}
    oseg = [s for s in pcb.segments if s.net_id and s.net_id not in ids] if others else []
    ovia = [v for v in pcb.vias if v.net_id and v.net_id not in ids] if others else []
    r = BoardRenderer(pcb, size=SIZE, supersample=SS, show_zones=False, view=view, theme=theme(), layer_alpha=255)

    def grey(d, rr):
        for s in oseg:
            d.line([rr.tf.pt(s.start_x, s.start_y), rr.tf.pt(s.end_x, s.end_y)], fill=GREY,
                   width=max(1, int(round(rr.tf.length(s.width)))))
        for v in ovia:
            x, y = rr.tf.pt(v.x, v.y)
            q = rr.tf.length(v.size / 2)
            d.ellipse([x - q, y - q, x + q, y + q], fill=GREY)

    def markers(d, rr):
        rad = rr.tf.length(VIA_MM / 2)
        w = max(1, int(round(rr.tf.length(0.07))))
        for v in vias:
            x, y = rr.tf.pt(v.x, v.y)
            d.ellipse([x - rad, y - rad, x + rad, y + rad], fill=(255, 255, 255), outline=VIA_RING, width=w)

    img = r.frame(segments=[s for s in segs if s.net_id not in legs], vias=[],
                  highlight_segments=[s for s in segs if s.net_id in legs], highlight_color=PAIR,
                  overlays=[grey, markers])
    (x0, y0), (x1, y1) = r.tf.pt(view[0], view[1]), r.tf.pt(view[2], view[3])
    k = 1 / SS
    return img.crop([int(min(x0, x1) * k), int(min(y0, y1) * k), int(max(x0, x1) * k) + 1, int(max(y0, y1) * k) + 1])


def main(argv=None):
    from PIL import Image, ImageDraw, ImageFont
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('out'), ap.add_argument('nets'), ap.add_argument('boards', nargs='+', help='LABEL=BOARD')
    ap.add_argument('--cols', type=int, default=2)
    ap.add_argument('--no-others', action='store_true')
    a = ap.parse_args(argv)
    txt = open(a.nets).read() if os.path.isfile(a.nets) else a.nets
    names = {n for n in txt.replace(',', ' ').split() if n}
    items = [b.rsplit('=', 1) for b in a.boards]
    pcbs = [load(p) for _, p in items]
    xs, ys = [], []
    for pcb in pcbs:
        segs, vias, _ = rung(pcb, names)
        xs += [s.start_x for s in segs] + [s.end_x for s in segs] + [v.x for v in vias]
        ys += [s.start_y for s in segs] + [s.end_y for s in segs] + [v.y for v in vias]
    view = (min(xs) - 1, min(ys) - 1, max(xs) + 1, max(ys) + 1)
    panels = [panel(pcb, names, view, not a.no_others) for pcb in pcbs]
    # each crop is the same VIEW in millimetres, but the renderer scales a board's whole outline to its canvas, so a
    # larger board (the designer's whole board beside a bench) crops smaller: one size, one scale
    w, h = panels[0].size
    panels = [pn if pn.size == (w, h) else pn.resize((w, h), Image.LANCZOS) for pn in panels]
    font = next((ImageFont.truetype(f, int(h * 0.05)) for f in FONTS if _has_font(ImageFont, f)), ImageFont.load_default())
    top, gap = int(h * 0.08), int(w * 0.03)
    rows = -(-len(panels) // a.cols)
    img = Image.new('RGB', (a.cols * w + (a.cols - 1) * gap, rows * (h + top)), (255, 255, 255))
    d = ImageDraw.Draw(img)
    for i, (pn, (lab, _)) in enumerate(zip(panels, items)):
        x, y = (i % a.cols) * (w + gap), (i // a.cols) * (h + top)
        img.paste(pn, (x, y + top))
        d.text((x + int(w * 0.01), y + int(top * 0.15)), lab, fill=(0, 0, 0), font=font)
    img.save(a.out)
    print('wrote', a.out, img.size)


def _has_font(ImageFont, f):
    try:
        ImageFont.truetype(f, 10)
        return True
    except OSError:
        return False


if __name__ == '__main__':
    main()
