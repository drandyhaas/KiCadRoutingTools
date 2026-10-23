#!/usr/bin/env python3
"""Draw chosen nets' copper on several routed boards side by side, to compare them track by track.

A placement change shows up net by net: one candidate routes a crystal's
nets in 6 mm, another in 40 mm with five vias around the chip. This draws,
for each board given, one panel of the same window: the chosen nets' tracks
bright in their layer's colour with their vias, their pads pink, and every
other track, pad and via faint behind them. Each panel's title is its label,
the chosen nets' via count and their track length.

    python3 -X utf8 py_tools/render_nets.py -o osc.png --nets PA1,PA2 \\
        human=routed_human.kicad_pcb ours=routed.kicad_pcb
    python3 -X utf8 py_tools/render_nets.py -o gnd.png --nets 'GND' --view 120,60,150,90 a.kicad_pcb

A board is `LABEL=PATH` or a bare path (labelled by its file name). `--nets`
takes exact names or fnmatch patterns. `--view x0,y0,x1,y1` (mm) fixes the
window; the default is the chosen nets' pads and copper on every board, plus
2 mm. Layers are coloured F.Cu red, B.Cu blue, then In1.Cu, In2.Cu, ... in a
fixed order.

Exit codes: 0 drawn, 2 an unreadable board, or a pattern no board's net matches.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import fnmatch
import math
import os
import sys

from kicad_parser import parse_kicad_pcb  # noqa: E402

#: layer -> RGB, F.Cu and B.Cu first; inner layers take the rest in order
LAYER_COLOURS = [(220, 40, 40), (40, 90, 230), (230, 160, 0), (20, 170, 60),
                 (170, 60, 200), (0, 170, 170), (140, 90, 40), (90, 90, 90)]


def layer_colour(layer, layers):
    """`layer`'s colour: F.Cu red, B.Cu blue, inner layers in board order."""
    order = ['F.Cu', 'B.Cu'] + [l for l in layers if l not in ('F.Cu', 'B.Cu')]
    i = order.index(layer) if layer in order else len(order)
    return LAYER_COLOURS[i % len(LAYER_COLOURS)]


def chosen_ids(pcb, patterns):
    """Net ids whose name matches any pattern (exact or fnmatch)."""
    return {nid for nid, n in pcb.nets.items()
            if nid and n.name and any(n.name == p or fnmatch.fnmatchcase(n.name, p)
                                      for p in patterns)}


def panel_stats(pcb, ids):
    """`(vias, track mm)` of the chosen nets."""
    nv = sum(1 for v in pcb.vias if v.net_id in ids)
    mm = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in pcb.segments if s.net_id in ids and not getattr(s, 'graphic', False))
    return nv, mm


def auto_view(pcbs, ids_of, margin=2.0):
    xs, ys = [], []
    for lab, pcb in pcbs:
        ids = ids_of[lab]
        for s in pcb.segments:
            if s.net_id in ids:
                xs += [s.start_x, s.end_x]
                ys += [s.start_y, s.end_y]
        for nid in ids:
            for p in pcb.pads_by_net.get(nid, ()):
                xs.append(p.global_x)
                ys.append(p.global_y)
    if not xs:
        return None
    return min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin


def render(pcbs, patterns, out, view=None, scale=12.0):
    """Draw the panels to `out`; returns `[(label, vias, mm)]`."""
    from PIL import Image, ImageDraw, ImageFont
    ids_of = {lab: chosen_ids(pcb, patterns) for lab, pcb in pcbs}
    if view is None:
        view = auto_view(pcbs, ids_of)
    x0, y0, x1, y1 = view
    W, H = max(1, int((x1 - x0) * scale)), max(1, int((y1 - y0) * scale))
    gap, head = 10, 30
    img = Image.new('RGB', (W * len(pcbs) + gap * (len(pcbs) - 1), H + head), 'white')
    font = None
    for f in ('/System/Library/Fonts/Menlo.ttc', 'DejaVuSansMono.ttf', 'consola.ttf'):
        try:
            font = ImageFont.truetype(f, 18)
            break
        except OSError:
            continue
    font = font or ImageFont.load_default()
    d = ImageDraw.Draw(img)
    rows = []
    for k, (lab, pcb) in enumerate(pcbs):
        ox = k * (W + gap)
        layers = list(pcb.board_info.copper_layers or [])
        ids = ids_of[lab]

        def T(x, y, ox=ox):
            return (ox + (x - x0) * scale, head + (y - y0) * scale)
        for fp in pcb.footprints.values():
            for p in fp.pads:
                hot = p.net_id in ids
                d.rectangle([T(p.global_x - p.size_x / 2, p.global_y - p.size_y / 2),
                             T(p.global_x + p.size_x / 2, p.global_y + p.size_y / 2)],
                            fill=(255, 200, 200) if hot else (225, 225, 225),
                            outline=(150, 0, 0) if hot else (190, 190, 190))
        for hot in (False, True):
            for s in pcb.segments:
                if getattr(s, 'graphic', False) or (s.net_id in ids) != hot:
                    continue
                c = layer_colour(s.layer, layers)
                if not hot:
                    c = tuple(int(255 - (255 - v) * 0.22) for v in c)
                d.line([T(s.start_x, s.start_y), T(s.end_x, s.end_y)], fill=c,
                       width=max(2 if hot else 1, int(s.width * scale)))
        for v in pcb.vias:
            r = v.size / 2 * scale
            cx, cy = T(v.x, v.y)
            if v.net_id in ids:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(0, 0, 0))
                d.ellipse([cx - r / 2, cy - r / 2, cx + r / 2, cy + r / 2], fill='white')
            else:
                d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(170, 170, 170))
        nv, mm = panel_stats(pcb, ids)
        d.text((ox + 5, 5), f'{lab}: {nv} vias {mm:.0f} mm', fill='black', font=font)
        rows.append((lab, nv, mm))
    img.save(out)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boards', nargs='+', help='LABEL=board.kicad_pcb, or a bare path')
    ap.add_argument('-o', '--out', required=True, help='the PNG to write')
    ap.add_argument('--nets', action='append', required=True,
                    help='a net name or fnmatch pattern, or several separated by '
                         'commas; repeat the flag for more')
    ap.add_argument('--view', help='x0,y0,x1,y1 in mm (default: the chosen nets, plus 2 mm)')
    ap.add_argument('--scale', type=float, default=12.0, help='pixels per mm (default 12)')
    a = ap.parse_args(argv)
    patterns = [p for arg in a.nets for p in arg.split(',') if p]
    pcbs = []
    for b in a.boards:
        lab, path = b.split('=', 1) if '=' in b else (os.path.splitext(os.path.basename(b))[0], b)
        try:
            pcbs.append((lab, parse_kicad_pcb(path)))
        except (OSError, ValueError) as e:
            print(f"render_nets: cannot read {path}: {e}", file=sys.stderr)
            return 2
    unmatched = [p for p in patterns if not any(chosen_ids(pcb, [p]) for _l, pcb in pcbs)]
    if unmatched:
        print(f"render_nets: no net on any board matches {', '.join(unmatched)}", file=sys.stderr)
        return 2
    view = tuple(float(v) for v in a.view.split(',')) if a.view else None
    if view is not None and len(view) != 4:
        print("render_nets: --view takes x0,y0,x1,y1", file=sys.stderr)
        return 2
    for lab, nv, mm in render(pcbs, patterns, a.out, view, a.scale):
        print(f"  {lab}: {nv} vias, {mm:.1f} mm on {', '.join(patterns)}")
    print(f"render_nets: wrote {a.out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
