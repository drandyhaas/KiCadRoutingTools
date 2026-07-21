#!/usr/bin/env python3
"""
Render a .kicad_pcb to per-layer PNGs (one image per copper layer, each with
Edge.Cuts for the outline) plus the combined all-copper PNG. Per-layer images
make layer roles readable at a glance -- plane vs signal layers, bus rivers,
pour islands -- which the combined render hides behind the top copper (#424
ottercast study). Drops `<board>_<layer>.png` for every copper layer and the
existing `<board>.png` next to the board, so a stress-run final board can be
eyeballed layer by layer without opening KiCad.

This supersedes calling board_image.py directly for stress-run renders; the
single-image machinery (kicad-cli SVG export + headless-Chrome rasterize)
still lives there and is reused here.

Usage:
    python3 tests/stress/board_layer_images.py BOARD.kicad_pcb [-o OUT.png]
        [--size PX] [--no-combined]

`-o OUT.png` names the COMBINED image; per-layer files derive from it
(`OUT_F.Cu.png`, `OUT_In1.Cu.png`, ...). Default: alongside the board.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from board_image import render_board_png, board_copper_layers  # noqa: E402


def _layer_zones(board_path):
    """{layer: [(net_name, [(x,y),...]), ...]} for copper zones with a net.
    Router-chain outputs carry zones WITHOUT stored fills (KiCad refills at
    open), so kicad-cli renders nothing for them -- a solid GND plane looks
    like empty space. Non-fatal: returns {} when the parser is unavailable."""
    try:
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(board_path)
    except Exception:
        return {}
    out = {}
    for z in getattr(pcb, 'zones', []) or []:
        net = getattr(z, 'net_name', None)
        layer = getattr(z, 'layer', None)
        pts = getattr(z, 'polygon', None) or getattr(z, 'points', None)
        if not net or not layer or not pts or len(pts) < 3:
            continue
        out.setdefault(layer, []).append((net, list(pts)))
    return out


_ZONE_HUES = ['#9C27B0', '#00838F', '#E65100', '#2E7D32', '#C2185B', '#5E35B1']


def _clip_to_box(pts, min_x, min_y, max_x, max_y):
    """Sutherland-Hodgman clip of a polygon to an axis box. Router-generated
    zone polygons are often full-board rectangles (the FILL clips to the edge
    at refill, the outline does not) -- unclipped they flood the render."""
    def clip_edge(poly, inside, intersect):
        out = []
        for i, cur in enumerate(poly):
            prev = poly[i - 1]
            cin, pin = inside(cur), inside(prev)
            if cin != pin:
                out.append(intersect(prev, cur))
            if cin:
                out.append(cur)
        return out
    for inside, intersect in (
        (lambda p: p[0] >= min_x, lambda a, b: (min_x, a[1] + (b[1]-a[1]) * (min_x-a[0]) / (b[0]-a[0]))),
        (lambda p: p[0] <= max_x, lambda a, b: (max_x, a[1] + (b[1]-a[1]) * (max_x-a[0]) / (b[0]-a[0]))),
        (lambda p: p[1] >= min_y, lambda a, b: (a[0] + (b[0]-a[0]) * (min_y-a[1]) / (b[1]-a[1]), min_y)),
        (lambda p: p[1] <= max_y, lambda a, b: (a[0] + (b[0]-a[0]) * (max_y-a[1]) / (b[1]-a[1]), max_y)),
    ):
        pts = clip_edge(pts, inside, intersect)
        if not pts:
            return []
    return pts


def _zone_overlay_post(zones, bounds):
    """svg_post callable drawing translucent zone outlines + net labels, one
    stable color per net. KiCad's SVG viewBox is in mm with its origin at the
    board bbox corner; centering the tiny width residual keeps the mapping
    error ~5 um."""
    import re

    def post(svg):
        m = re.search(r'viewBox="([\d.\-]+) ([\d.\-]+) ([\d.]+) ([\d.]+)"', svg)
        if not m or not zones:
            return svg
        vx, vy, vw, vh = (float(g) for g in m.groups())
        min_x, min_y, max_x, max_y = bounds
        offx = min_x - vx - (vw - (max_x - min_x)) / 2
        offy = min_y - vy - (vh - (max_y - min_y)) / 2
        nets = []
        for net, _ in zones:
            if net not in nets:
                nets.append(net)
        color = {n: _ZONE_HUES[i % len(_ZONE_HUES)] for i, n in enumerate(nets)}
        bits = []
        for net, pts in zones:
            pts = _clip_to_box(pts, min_x, min_y, max_x, max_y)
            if len(pts) < 3:
                continue
            c = color[net]
            d = 'M ' + ' L '.join(f'{x - offx:.3f} {y - offy:.3f}' for x, y in pts) + ' Z'
            bits.append(f'<path d="{d}" fill="{c}" fill-opacity="0.10" '
                        f'stroke="{c}" stroke-width="0.15" '
                        f'stroke-dasharray="0.8,0.5" stroke-opacity="0.8"/>')
            cx = sum(p[0] for p in pts) / len(pts) - offx
            cy = sum(p[1] for p in pts) / len(pts) - offy
            bits.append(f'<text x="{cx:.3f}" y="{cy:.3f}" font-size="2" '
                        f'text-anchor="middle" fill="{c}" '
                        f'stroke="#FFFFFF" stroke-width="0.05" '
                        f'font-family="sans-serif" font-weight="bold">{net}</text>')
        return svg.replace('</svg>', ''.join(bits) + '</svg>')

    return post


def _board_bounds(board_path):
    try:
        from kicad_parser import parse_kicad_pcb
        return parse_kicad_pcb(board_path).board_info.board_bounds
    except Exception:
        return None


def render_board_layer_pngs(board_path, out_png=None, size=2000, quiet=False,
                            combined=True, zone_outlines=True):
    """Render one PNG per copper layer (+ Edge.Cuts) and optionally the
    combined all-copper PNG. Unfilled copper zones (the router chain never
    stores fills) are drawn as translucent labeled outlines so plane layers
    read as planes. Returns the list of paths written ([] on total failure --
    e.g. kicad-cli missing)."""
    board_path = os.path.abspath(board_path)
    if out_png is None:
        out_png = os.path.splitext(board_path)[0] + '.png'
    base, ext = os.path.splitext(out_png)
    zones = _layer_zones(board_path) if zone_outlines else {}
    bounds = _board_bounds(board_path) if zones else None
    written = []
    for layer in board_copper_layers(board_path) or ['F.Cu', 'B.Cu']:
        post = (_zone_overlay_post(zones[layer], bounds)
                if bounds and zones.get(layer) else None)
        out = render_board_png(board_path, f'{base}_{layer}{ext}',
                               layers=f'{layer},Edge.Cuts', size=size,
                               quiet=quiet, svg_post=post)
        if out:
            written.append(out)
    if combined:
        out = render_board_png(board_path, out_png, size=size, quiet=quiet)
        if out:
            written.append(out)
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('-o', '--output', default=None,
                    help='combined PNG path; per-layer files derive from it '
                         '(default: alongside board)')
    ap.add_argument('--size', type=int, default=2000,
                    help='max image dimension px')
    ap.add_argument('--no-combined', action='store_true',
                    help='per-layer images only, skip the combined render')
    args = ap.parse_args()
    written = render_board_layer_pngs(args.board, args.output, args.size,
                                      combined=not args.no_combined)
    return 0 if written else 1


if __name__ == '__main__':
    sys.exit(main())
