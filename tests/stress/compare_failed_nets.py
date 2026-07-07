#!/usr/bin/env python3
"""
Issue #296: per-FAILED-net comparison of our routing vs the human-routed original.

For every net that is incomplete on OUR final board, answer "how did the human
get this net routed?": which layers, how many vias, what widths, how much
clearance headroom their copper actually has in that neighborhood, and how
congested the region is. Emits a report.md + JSON + one cropped side-by-side
SVG per net (human | ours), so the technique difference is visible exactly
where we failed.

Usage:
    python3 tests/stress/compare_failed_nets.py OURS.kicad_pcb ORIGINAL.kicad_pcb \
        --out DIR [--nets NAME ...] [--max-renders N] [--margin MM]

Net matching is by NAME (net ids differ between the two files). Requires the
original to be the same design (the stress corpus originals in boards_setN/).
"""
import argparse
import json
import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import parse_kicad_pcb
from geometry_utils import segment_to_segment_distance_seg, point_to_segment_distance_seg
from routing_utils import point_to_pad_rect_dist

LAYER_COLORS = {
    'F.Cu': '#d62728', 'B.Cu': '#1f77b4',
    'In1.Cu': '#2ca02c', 'In2.Cu': '#9467bd', 'In3.Cu': '#ff7f0e',
    'In4.Cu': '#17becf', 'In5.Cu': '#bcbd22', 'In6.Cu': '#e377c2',
}
FOREIGN = '#cccccc'
FOREIGN_PAD = '#bbbbbb'


def _net_by_name(pcb):
    return {n.name: nid for nid, n in pcb.nets.items() if n.name}


def _net_copper(pcb, nid):
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    pads = pcb.pads_by_net.get(nid, [])
    return segs, vias, pads


def _bbox(segs, vias, pads, margin):
    xs, ys = [], []
    for s in segs:
        xs += [s.start_x, s.end_x]; ys += [s.start_y, s.end_y]
    for v in vias:
        xs.append(v.x); ys.append(v.y)
    for p in pads:
        xs.append(p.global_x); ys.append(p.global_y)
    if not xs:
        return None
    return (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)


def _in_bbox_seg(s, bb):
    return not (max(s.start_x, s.end_x) < bb[0] or min(s.start_x, s.end_x) > bb[2]
                or max(s.start_y, s.end_y) < bb[1] or min(s.start_y, s.end_y) > bb[3])


def _length(segs):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)


def _min_foreign_gap(pcb, nid, segs, vias):
    """Min edge-to-edge gap from this net's copper to any OTHER net's copper
    (segments + pads + vias, bbox-pruned). This is the clearance the human
    actually needed to pull the route off."""
    best = (float('inf'), '')
    items_bb = _bbox(segs, vias, [], 0.8)
    if items_bb is None:
        return None, ''
    fsegs = [s for s in pcb.segments if s.net_id != nid and _in_bbox_seg(s, items_bb)]
    fpads = [p for plist_nid, plist in pcb.pads_by_net.items() if plist_nid != nid
             for p in plist if items_bb[0] <= p.global_x <= items_bb[2]
             and items_bb[1] <= p.global_y <= items_bb[3]]
    fvias = [v for v in pcb.vias if v.net_id != nid
             and items_bb[0] <= v.x <= items_bb[2] and items_bb[1] <= v.y <= items_bb[3]]
    for s in segs:
        for o in fsegs:
            if o.layer != s.layer:
                continue
            d = segment_to_segment_distance_seg(s, o) - (s.width + o.width) / 2
            if d < best[0]:
                best = (d, f"track {pcb.nets[o.net_id].name if o.net_id in pcb.nets else o.net_id}")
        for p in fpads:
            if s.layer not in (p.layers or []) and '*.Cu' not in (p.layers or []):
                continue
            n = max(2, int(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) / 0.1))
            d = min(point_to_pad_rect_dist(s.start_x + (s.end_x - s.start_x) * i / n,
                                           s.start_y + (s.end_y - s.start_y) * i / n, p)
                    for i in range(n + 1)) - s.width / 2
            if d < best[0]:
                best = (d, f"pad {p.component_ref}.{p.pad_number}")
        for v in fvias:
            d = point_to_segment_distance_seg(v.x, v.y, s) - v.size / 2 - s.width / 2
            if d < best[0]:
                best = (d, f"via {pcb.nets[v.net_id].name if v.net_id in pcb.nets else v.net_id}")
    return (None, '') if best[0] == float('inf') else (round(best[0], 4), best[1])


def _svg_panel(pcb, nid, bb, stray=(), title=''):
    """One SVG <g> of the bbox region: foreign copper gray, net colored by layer."""
    x0, y0, x1, y1 = bb
    out = [f'<text x="{x0 + 0.3:.2f}" y="{y0 + 1.2:.2f}" font-size="1.2" fill="#333">{title}</text>']
    # foreign copper first (underlay)
    for s in pcb.segments:
        if s.net_id != nid and _in_bbox_seg(s, bb):
            out.append(f'<line x1="{s.start_x:.3f}" y1="{s.start_y:.3f}" x2="{s.end_x:.3f}" '
                       f'y2="{s.end_y:.3f}" stroke="{FOREIGN}" stroke-width="{s.width:.3f}" stroke-linecap="round"/>')
    for plist_nid, plist in pcb.pads_by_net.items():
        for p in plist:
            if not (bb[0] <= p.global_x <= bb[2] and bb[1] <= p.global_y <= bb[3]):
                continue
            col = FOREIGN_PAD if plist_nid != nid else '#555555'
            out.append(f'<rect x="{p.global_x - p.size_x / 2:.3f}" y="{p.global_y - p.size_y / 2:.3f}" '
                       f'width="{p.size_x:.3f}" height="{p.size_y:.3f}" fill="{col}" fill-opacity="0.55"'
                       + (f' transform="rotate({-(getattr(p, "rect_rotation", 0) or 0):.1f} {p.global_x:.3f} {p.global_y:.3f})"'
                          if getattr(p, 'rect_rotation', 0) else '') + '/>')
    for v in pcb.vias:
        if v.net_id != nid and bb[0] <= v.x <= bb[2] and bb[1] <= v.y <= bb[3]:
            out.append(f'<circle cx="{v.x:.3f}" cy="{v.y:.3f}" r="{v.size / 2:.3f}" fill="{FOREIGN}"/>')
    # the net itself, colored per layer
    for s in pcb.segments:
        if s.net_id == nid:
            c = LAYER_COLORS.get(s.layer, '#000000')
            out.append(f'<line x1="{s.start_x:.3f}" y1="{s.start_y:.3f}" x2="{s.end_x:.3f}" '
                       f'y2="{s.end_y:.3f}" stroke="{c}" stroke-width="{s.width:.3f}" stroke-linecap="round"/>')
    for v in pcb.vias:
        if v.net_id == nid:
            out.append(f'<circle cx="{v.x:.3f}" cy="{v.y:.3f}" r="{v.size / 2:.3f}" fill="#222222"/>'
                       f'<circle cx="{v.x:.3f}" cy="{v.y:.3f}" r="{v.drill / 2:.3f}" fill="#ffffff"/>')
    for (sx, sy) in stray:
        out.append(f'<g stroke="#ff6600" stroke-width="0.15"><line x1="{sx - 0.5:.2f}" y1="{sy - 0.5:.2f}" '
                   f'x2="{sx + 0.5:.2f}" y2="{sy + 0.5:.2f}"/><line x1="{sx - 0.5:.2f}" y1="{sy + 0.5:.2f}" '
                   f'x2="{sx + 0.5:.2f}" y2="{sy - 0.5:.2f}"/></g>')
    return '\n'.join(out)


def _render(pcb_h, nid_h, pcb_o, nid_o, bb, stray, path, net_name, layers_h):
    w, h = bb[2] - bb[0], bb[3] - bb[1]
    gap = 2.0
    legend = ' '.join(f'<tspan fill="{LAYER_COLORS.get(l, "#000")}">{l}</tspan>'
                      for l in layers_h)
    # Wide regions (long nets) stack the two panels vertically so neither is
    # clipped; compact regions sit side by side.
    stack = w > 1.5 * h
    if stack:
        tw, th = w, 2 * h + gap
        shift = f'translate(0,{h + gap:.2f})'
    else:
        tw, th = 2 * w + gap, h
        shift = f'translate({w + gap:.2f},0)'
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{bb[0]:.2f} {bb[1] - 3:.2f} '
           f'{tw:.2f} {th + 4:.2f}" width="{tw * 20:.0f}" height="{(th + 4) * 20:.0f}">',
           f'<rect x="{bb[0]:.2f}" y="{bb[1] - 3:.2f}" width="{tw:.2f}" height="{th + 4:.2f}" fill="white"/>',
           f'<text x="{bb[0] + 0.3:.2f}" y="{bb[1] - 1.6:.2f}" font-size="1.4" fill="#000">{net_name}   [{legend}]</text>',
           f'<g>{_svg_panel(pcb_o, nid_o, bb, (), "HUMAN")}</g>',
           f'<g transform="{shift}">{_svg_panel(pcb_h, nid_h, bb, stray, "OURS")}</g>',
           '</svg>']
    with open(path, 'w') as f:
        f.write('\n'.join(svg))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ours')
    ap.add_argument('original')
    ap.add_argument('--out', default='failed_net_compare')
    ap.add_argument('--nets', nargs='*', help='explicit net names (else auto-detect incomplete nets)')
    ap.add_argument('--max-renders', type=int, default=20)
    ap.add_argument('--margin', type=float, default=2.0)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print(f"Parsing OURS: {args.ours}")
    ours = parse_kicad_pcb(args.ours)
    print(f"Parsing ORIGINAL: {args.original}")
    orig = parse_kicad_pcb(args.original)

    by_name_o = _net_by_name(orig)
    failed = []
    if args.nets:
        by_name_h = _net_by_name(ours)
        for nm in args.nets:
            if nm in by_name_h:
                failed.append((nm, by_name_h[nm], []))
    else:
        from check_connected import check_net_connectivity
        zones_by_net = {}
        for z in (getattr(ours, 'zones', None) or []):
            zones_by_net.setdefault(getattr(z, 'net_id', -1), []).append(z)
        print("Detecting incomplete nets on OUR board...")
        for nid, net in sorted(ours.nets.items()):
            pads = ours.pads_by_net.get(nid, [])
            if len(pads) < 2 or not net.name:
                continue
            segs = [s for s in ours.segments if s.net_id == nid]
            vias = [v for v in ours.vias if v.net_id == nid]
            r = check_net_connectivity(nid, segs, vias, pads, zones_by_net.get(nid, []))
            if not r.get('connected', True):
                stray = []
                for p in r.get('disconnected_pads', []):
                    if hasattr(p, 'global_x'):
                        stray.append((p.global_x, p.global_y))
                    elif isinstance(p, (list, tuple)) and len(p) >= 2:
                        stray.append((float(p[0]), float(p[1])))
                failed.append((net.name, nid, stray))
    print(f"{len(failed)} incomplete net(s) on our board")

    rows = []
    for i, (name, nid_h, stray) in enumerate(failed):
        nid_o = by_name_o.get(name)
        row = {'net': name}
        segs_h, vias_h, pads_h = _net_copper(ours, nid_h)
        row['ours'] = {'segments': len(segs_h), 'vias': len(vias_h), 'pads': len(pads_h),
                       'length_mm': round(_length(segs_h), 2),
                       'layers': sorted({s.layer for s in segs_h}),
                       'stray_pads': len(stray)}
        if nid_o is None:
            row['human'] = None
            rows.append(row)
            continue
        segs_o, vias_o, pads_o = _net_copper(orig, nid_o)
        gap, gap_vs = _min_foreign_gap(orig, nid_o, segs_o, vias_o)
        layers_o = sorted({s.layer for s in segs_o})
        row['human'] = {'segments': len(segs_o), 'vias': len(vias_o),
                        'length_mm': round(_length(segs_o), 2),
                        'layers': layers_o,
                        'widths': sorted({round(s.width, 3) for s in segs_o}),
                        'via_sizes': sorted({f"{v.size}/{v.drill}" for v in vias_o}),
                        'min_foreign_gap_mm': gap, 'min_gap_vs': gap_vs}
        # congestion context: foreign pads inside the net's bbox
        bb = _bbox(segs_o + segs_h, vias_o + vias_h, pads_o, args.margin)
        if bb:
            row['region'] = {'bbox': [round(c, 1) for c in bb],
                             'foreign_pads_in_region': sum(
                                 1 for pn, pl in orig.pads_by_net.items() if pn != nid_o
                                 for p in pl if bb[0] <= p.global_x <= bb[2] and bb[1] <= p.global_y <= bb[3])}
            if i < args.max_renders:
                fn = os.path.join(args.out, f"net_{i:02d}_{''.join(c if c.isalnum() else '_' for c in name)[:40]}.svg")
                _render(ours, nid_h, orig, nid_o, bb, stray, fn, name, layers_o)
                row['svg'] = fn
                print(f"  rendered {fn}")
        rows.append(row)

    with open(os.path.join(args.out, 'report.json'), 'w') as f:
        json.dump(rows, f, indent=1)
    # markdown table
    lines = ['# Failed-net comparison: how the human routed what we could not',
             '', f'ours: `{args.ours}`  original: `{args.original}`', '',
             '| net | ours (segs/vias/stray) | human segs | human vias | human layers | human min gap (mm) | vs |',
             '|---|---|---|---|---|---|---|']
    for r in rows:
        h = r.get('human')
        o = r['ours']
        if h:
            lines.append(f"| {r['net']} | {o['segments']}/{o['vias']}/{o['stray_pads']} | {h['segments']} "
                         f"| {h['vias']} ({','.join(h['via_sizes'])}) | {','.join(h['layers'])} "
                         f"| {h['min_foreign_gap_mm']} | {h['min_gap_vs']} |")
        else:
            lines.append(f"| {r['net']} | {o['segments']}/{o['vias']}/{o['stray_pads']} | (net absent in original) | | | | |")
    with open(os.path.join(args.out, 'report.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"Wrote {os.path.join(args.out, 'report.md')} and report.json")


if __name__ == '__main__':
    main()
