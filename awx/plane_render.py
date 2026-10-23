#!/usr/bin/env python3
"""plane_render.py BOARD OUT.png [--quarter Q] [--bus SUMMARY ..] [--all-nets] [--chips REF[=LABEL],..] [--chip-labels]
[--title T] -- a whole board drawn with its PLANES: each zone's fill as KiCad fills it (kicad_exact_fill: KiCad's own
filler, through its python; the zone's outline where that is not to be had, said so in the legend), each (net, layer)
a colour of its own laid over the others with alpha -- two planes on two layers mix where they overlap, and white is no
plane on either -- each zone outlined thin black. Over them the copper as the bus renders draw it (baseline_render's
theme): the buses' tracks (route_bus summaries, --bus; every net's with --all-nets) each layer its colour, B.Cu first
and F.Cu on top, their vias white markers; every other net grey; the pads tan. --chips boxes the named parts (the
arrays the buses join), --chip-labels names them. The page is turned Q quarter turns counter-clockwise (1: what was
below is on the right) and a legend set beside it: each plane's islands and filled area, the layers' colours."""
KRT_TOOL = {'scope': [], 'kind': 'instrument'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

PALETTE = [(40, 90, 210), (130, 50, 180), (235, 130, 20), (215, 40, 40), (40, 160, 70), (225, 195, 20),
           (20, 170, 185), (205, 60, 165), (120, 120, 40), (90, 60, 30)]
ALPHA = 80                  # each plane's tint over the ones below it
PX = 32                     # pixels per mm, before the supersample is taken down
SS = 2                      # the supersample
GREY = (190, 190, 190)
FONTS = ('/System/Library/Fonts/Helvetica.ttc', 'DejaVuSans.ttf', 'arial.ttf')


def short(name):
    return (name or '').split('/')[-1]


def palette(n):
    """n colours, each its own: PALETTE's, and past its ten hues spread round the wheel (the golden angle apart)"""
    import colorsys
    if n <= len(PALETTE):
        return PALETTE[:n]
    return [tuple(int(255 * c) for c in colorsys.hsv_to_rgb((i * 0.618034) % 1.0, 0.75, 0.85 - 0.2 * (i % 2)))
            for i in range(n)]


def font(size):
    from PIL import ImageFont
    for f in FONTS:
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()


def plane_fills(board, pcb):
    """({(net name, layer): [island polygon]}, how): the zones as KiCad fills them, else their outlines"""
    import kicad_exact_fill as kef
    from copy_board import copy_board
    td = tempfile.mkdtemp(prefix='plane_render_')
    try:
        src = os.path.join(td, 'b.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(board, src)
        filled = os.path.join(td, 'filled.kicad_pcb')
        st = kef.write_filled_board(src, filled)
        if st.ok:
            return kef.parse_filled_islands(open(filled, encoding='utf-8').read()), 'KiCad fill'
    finally:
        shutil.rmtree(td, ignore_errors=True)
    out = {}
    for z in pcb.zones or ():
        if z.net_id in pcb.nets and len(z.polygon) >= 3:
            out.setdefault((pcb.nets[z.net_id].name, z.layer), []).append(list(z.polygon))
    return out, 'zone outlines: no KiCad python to fill them'


def render(board, out, quarter=1, buses=(), all_nets=False, chips='', chip_labels=False, title=''):
    from PIL import Image, ImageDraw
    import render_theme
    import baseline_render as br
    import kicad_exact_fill as kef
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    fills, how = plane_fills(board, pcb)
    lit = set()
    for p in buses:
        d = json.load(open(p))
        names = {short(n) for n in list(d['routed']) + list(d['refused'])}
        lit |= {i for i, n in pcb.nets.items() if short(n.name) in names}
    if all_nets:
        lit = {i for i in pcb.nets if i}

    # the page: the board's bounds, a millimetre round them
    x0, y0, x1, y1 = pcb.board_info.board_bounds
    x0, y0, x1, y1 = x0 - 1, y0 - 1, x1 + 1, y1 + 1
    S = PX * SS
    W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    pt = lambda x, y: ((x - x0) * S, (y - y0) * S)          # noqa: E731
    base = Image.new('RGBA', (W, H), (255, 255, 255, 255))

    # the planes, each (net, layer) its colour at ALPHA; each zone's OUTLINE thin black (not the fill's edge: KiCad
    # fractures a fill's holes open with slits out to its edge)
    order = sorted(fills, key=lambda k: (k[1], k[0]))
    colour = dict(zip(order, palette(len(order))))
    for k in order:
        lay = Image.new('RGBA', (W, H), (0, 0, 0, 0))
        dl = ImageDraw.Draw(lay)
        for poly in fills[k]:
            dl.polygon([pt(x, y) for x, y in poly], fill=colour[k] + (ALPHA,))
        base = Image.alpha_composite(base, lay)
    d = ImageDraw.Draw(base)
    for z in pcb.zones or ():
        if len(z.polygon) >= 3:
            q = [pt(x, y) for x, y in z.polygon]
            d.line(q + q[:1], fill=(0, 0, 0, 255), width=max(1, int(0.05 * S)), joint='curve')

    # the copper: pads tan, every other net grey, the lit nets' tracks each layer its colour (B.Cu first, F.Cu on top),
    # their vias one white marker
    th = br.theme()
    pad = th.rgb('pad')
    lcol = render_theme.layer_palette(list(pcb.board_info.copper_layers), th)
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if not any(L.endswith('.Cu') for L in p.layers) and not (p.drill or 0) > 0:
                continue
            cx, cy = pt(p.global_x, p.global_y)
            hx, hy = p.size_x / 2 * S, p.size_y / 2 * S
            (d.ellipse if p.shape == 'circle' else d.rectangle)([cx - hx, cy - hy, cx + hx, cy + hy], fill=pad + (255,))
    for s in pcb.segments:
        if s.net_id and s.net_id not in lit:
            d.line([pt(s.start_x, s.start_y), pt(s.end_x, s.end_y)], fill=GREY + (255,),
                   width=max(1, int(round(s.width * S))))
    for v in pcb.vias:
        if v.net_id and v.net_id not in lit:
            cx, cy = pt(v.x, v.y)
            r = v.size / 2 * S
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=GREY + (255,))
    stack = {L: i for i, L in enumerate(reversed(list(pcb.board_info.copper_layers)))}
    for s in sorted((s for s in pcb.segments if s.net_id in lit), key=lambda s: stack.get(s.layer, 0)):
        d.line([pt(s.start_x, s.start_y), pt(s.end_x, s.end_y)], fill=lcol.get(s.layer, (0, 0, 0)) + (255,),
               width=max(1, int(round(s.width * S))))
    for v in pcb.vias:
        if v.net_id in lit:
            cx, cy = pt(v.x, v.y)
            r = br.VIA_MM / 2 * S
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(255, 255, 255, 255), outline=br.VIA_RING + (255,),
                      width=max(1, int(0.06 * S)))

    # the chips: a box round each one's pads (its label set after the turn, upright)
    boxes = []
    for c in [x for x in chips.split(',') if x]:
        ref, _, lab = c.partition('=')
        fp = pcb.footprints[ref]
        xs = [p.global_x + sx * p.size_x / 2 for p in fp.pads for sx in (-1, 1)]
        ys = [p.global_y + sy * p.size_y / 2 for p in fp.pads for sy in (-1, 1)]
        bx0, by0 = pt(min(xs) - 0.5, min(ys) - 0.5)
        bx1, by1 = pt(max(xs) + 0.5, max(ys) + 0.5)
        d.rectangle([bx0, by0, bx1, by1], outline=(0, 0, 0, 255), width=int(0.25 * S))
        boxes.append((ref + (f' {lab}' if lab else ''), (bx0 / SS, by0 / SS, bx1 / SS, by1 / SS)))
    img = base.convert('RGB').resize((W // SS, H // SS), Image.LANCZOS)
    w0, h0 = img.size
    img = img.rotate(90 * quarter, expand=True)

    def turned(x, y):
        # (a point of the page before the turn, where it is after: each quarter turn counter-clockwise, (x, y) ->
        # (y, w - x))
        w, h = w0, h0
        for _ in range(quarter % 4):
            x, y, w, h = y, w - x, h, w
        return x, y

    # the sheet: the title, the page, the legend beside it
    big, mid, small = font(44), font(34), font(28)
    lw, top = 700, 80
    sheet = Image.new('RGB', (img.width + lw, max(img.height, 160 + 52 * len(order) + 400) + top), (255, 255, 255))
    sheet.paste(img, (0, top))
    dd = ImageDraw.Draw(sheet)
    for lab, (bx0, by0, bx1, by1) in (boxes if chip_labels else ()):
        cs = [turned(bx0, by0), turned(bx1, by1)]
        tx, ty = min(c[0] for c in cs), min(c[1] for c in cs) + top
        tw = dd.textlength(lab, font=big)
        dd.rectangle([tx, ty - 52, tx + tw + 16, ty - 2], fill=(255, 255, 255), outline=(0, 0, 0), width=2)
        dd.text((tx + 8, ty - 50), lab, fill=(0, 0, 0), font=big)
    dd.text((20, 20), title, fill=(0, 0, 0), font=mid)
    lx, y = img.width + 30, 110
    dd.text((lx, y), f'planes ({how}), alpha over each other:', fill=(0, 0, 0), font=small)
    y += 40
    for k in order:
        area = sum(kef._polygon_area(p) for p in fills[k])
        dd.rectangle([lx, y, lx + 50, y + 36], fill=tuple(int(255 - (255 - c) * ALPHA / 255) for c in colour[k]),
                     outline=(80, 80, 80))
        dd.text((lx + 65, y), f'{short(k[0])}  {k[1]}  ({len(fills[k])} island(s), {area:.0f} mm2)', fill=(0, 0, 0),
                font=small)
        y += 52
    for ln in ('each zone outlined thin black; where planes', 'overlap their tints mix;',
               'white: no plane on either layer'):
        dd.text((lx, y), ln, fill=(60, 60, 60), font=small)
        y += 36
    y += 20
    dd.text((lx, y), ('every net\'s' if all_nets else 'the buses\'') + ' tracks by layer:', fill=(0, 0, 0), font=small)
    y += 40
    rows = [(L, lcol[L]) for L in pcb.board_info.copper_layers if L in lcol]
    rows += ([] if all_nets else [('other nets', GREY)]) + [('pads', pad)]
    for lab, c in rows:
        dd.rectangle([lx, y + 12, lx + 50, y + 22], fill=c)
        dd.text((lx + 65, y), lab, fill=(0, 0, 0), font=small)
        y += 40
    sheet.save(out)
    return {f'{short(k[0])}@{k[1]}': len(v) for k, v in fills.items()}, how


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('board', help='the board (its .kicad_pro beside it)')
    ap.add_argument('out', help='the PNG written')
    ap.add_argument('--quarter', type=int, default=0, help='quarter turns counter-clockwise (default 0)')
    ap.add_argument('--bus', action='append', default=[], help='a route_bus summary (JSON): its nets drawn as a bus')
    ap.add_argument('--all-nets', action='store_true', help='every net\'s tracks by layer, not the buses\' alone')
    ap.add_argument('--chips', default='', help='REF[=LABEL],..: each part boxed round its pads')
    ap.add_argument('--chip-labels', action='store_true', help='each box named (REF and LABEL)')
    ap.add_argument('--title', default='')
    a = ap.parse_args(argv)
    got, how = render(a.board, a.out, a.quarter, a.bus, a.all_nets, a.chips, a.chip_labels, a.title)
    print(f'{a.out}: {how}; ' + ', '.join(f'{k} {n} island(s)' for k, n in sorted(got.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
