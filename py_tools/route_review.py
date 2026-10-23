#!/usr/bin/env python3
"""What to move after a route: open nets, costly nets, congested areas and the parts in them.

A routed board shows where its placement went wrong, but only net by net and
copper by copper. This instrument reads one routed board and lists, for a
placement pass that moves parts and routes again:

  OPEN NETS        each net whose pads the copper does not join
                   (`check_connected.run_connectivity_check`), with the parts
                   holding the disconnected pads. A net a pour serves is
                   marked `poured`: the fill, not the placement, decides it.
  COSTLY NETS      signal nets (rails and poured nets left out) ranked by vias,
                   then by detour: routed length over the length of the net's
                   pads' minimum spanning tree. A net routed at 1.5-2x its
                   tree with several vias is usually a part placed away from
                   the pins it serves.
  CONGESTED AREAS  the board on a grid (`--cell`, 2 mm). A cell's occupancy is
                   its track length times the routing pitch (track width plus
                   clearance), plus its vias' footprint on every layer they
                   cross, over the cell's area on the layers that carry signal
                   copper. Cells at `--hot` (0.5) or more are joined into
                   areas, each listed with its parts and the nets through it.
  TURNED PARTS     two-pad parts whose two nets would both be shorter with the
                   part turned 180 degrees (each pad to the nearest pad of its
                   own net: the net's tree, not its centroid). The turn is the
                   two pads trading places, a half turn about their midpoint,
                   and each row carries the `place_pose set REF X Y --rot R`
                   that makes it (`swap_pose`). `place_pose rotate` turns
                   about the footprint origin, which is pad 1 on a
                   through-hole axial, so it swings the body by the pitch
                   instead (#1322).

    python3 -X utf8 py_tools/route_review.py routed.kicad_pcb
    python3 -X utf8 py_tools/route_review.py routed.kicad_pcb --top 12 --json review.json

What to do about each finding is in
`.claude/skills/plan-pcb-placement/references/placement-lessons.md`. Exit codes:
0 reviewed (findings or not), 2 unreadable input.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import contextlib
import io
import json
import math
import statistics
import sys
from collections import defaultdict

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.grouping import BoardView  # noqa: E402


def mst_length(pts):
    """Length of the minimum spanning tree of `pts` (Prim)."""
    n = len(pts)
    if n < 2:
        return 0.0
    best = [math.inf] * n
    done = [False] * n
    done[0] = True
    for j in range(1, n):
        best[j] = math.dist(pts[0], pts[j])
    total = 0.0
    for _ in range(n - 1):
        k = min((j for j in range(n) if not done[j]), key=lambda j: best[j])
        done[k] = True
        total += best[k]
        for j in range(n):
            if not done[j]:
                d = math.dist(pts[k], pts[j])
                if d < best[j]:
                    best[j] = d
    return total


def _seg_len(s):
    return math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)


def review(pcb_file, cell=2.0, hot=0.5, top=8, pcb=None):
    """The review as a dict (the `--json` document)."""
    from check_connected import run_connectivity_check
    from list_nets import board_default_netclass_clearance
    pcb = pcb if pcb is not None else parse_kicad_pcb(pcb_file)
    view = BoardView(pcb)
    name = view.name_of
    poured = {z.net_id for z in (pcb.zones or ()) if z.net_id}
    tracks = [s for s in pcb.segments if not getattr(s, 'graphic', False) and s.net_id]
    clearance = board_default_netclass_clearance(pcb_file) or 0.2
    width = statistics.median([s.width for s in tracks]) if tracks else 0.2
    pitch = width + clearance

    # ---- open nets
    with contextlib.redirect_stdout(io.StringIO()):   # its own report
        issues = run_connectivity_check(pcb_file, quiet=True, pcb_data=pcb)
    opens = []
    for it in issues:
        parts = sorted(view.owners.get(it.get('net_id'), ()))
        copper = bool(it.get('num_segments') or it.get('num_vias'))
        refs = sorted({str(p[3]) for p in (it.get('disconnected_pads') or ()) if len(p) > 3})
        opens.append({'net': it.get('net_name'), 'poured': it.get('net_id') in poured,
                      'copper': copper,
                      # a net with no copper has every pad loose
                      'disconnected_pad_parts': refs if copper or refs else parts,
                      'parts': parts})

    # ---- per-net cost
    vias = defaultdict(int)
    for v in pcb.vias:
        vias[v.net_id] += 1
    length = defaultdict(float)
    for s in tracks:
        length[s.net_id] += _seg_len(s)
    nets = []
    for nid, pads in (pcb.pads_by_net or {}).items():
        if nid == 0 or nid in view.rails or nid in poured or len(pads) < 2:
            continue
        tree = mst_length([(p.global_x, p.global_y) for p in pads])
        nets.append({'net': name(nid), 'pads': len(pads), 'vias': vias[nid],
                     'routed_mm': round(length[nid], 1), 'tree_mm': round(tree, 1),
                     'detour': round(length[nid] / tree, 2) if tree > 0.5 and length[nid] else None,
                     'parts': sorted({p.component_ref for p in pads})})
    by_vias = sorted((n for n in nets if n['vias']), key=lambda n: (-n['vias'], n['net']))[:top]
    by_detour = sorted((n for n in nets if n['detour'] and n['tree_mm'] > 3.0),
                       key=lambda n: (-n['detour'], n['net']))[:top]

    # ---- congestion
    sig = [s for s in tracks if s.net_id not in view.rails]
    per_layer = defaultdict(float)
    for s in sig:
        per_layer[s.layer] += _seg_len(s)
    total = sum(per_layer.values()) or 1.0
    layers = sorted(l for l, v in per_layer.items() if v >= 0.05 * total) or ['F.Cu']
    occ = defaultdict(float)
    netlen = defaultdict(lambda: defaultdict(float))
    for s in sig:
        if s.layer not in layers:
            continue
        L = _seg_len(s)
        k = max(1, int(math.ceil(L / (cell / 4.0))))
        for i in range(k):
            u = (i + 0.5) / k
            c = (int(math.floor((s.start_x + u * (s.end_x - s.start_x)) / cell)),
                 int(math.floor((s.start_y + u * (s.end_y - s.start_y)) / cell)))
            occ[c] += L / k * pitch
            netlen[c][s.net_id] += L / k
    for v in pcb.vias:
        c = (int(math.floor(v.x / cell)), int(math.floor(v.y / cell)))
        span = len(layers)
        occ[c] += (v.size + clearance) ** 2 * span
    cap = cell * cell * len(layers)
    hot_cells = {c for c, a in occ.items() if a / cap >= hot}
    areas = []
    seen = set()
    for c0 in sorted(hot_cells, key=lambda c: -occ[c]):
        if c0 in seen:
            continue
        comp, todo = [], [c0]
        seen.add(c0)
        while todo:
            c = todo.pop()
            comp.append(c)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    q = (c[0] + dx, c[1] + dy)
                    if q in hot_cells and q not in seen:
                        seen.add(q)
                        todo.append(q)
        x0 = min(c[0] for c in comp) * cell
        y0 = min(c[1] for c in comp) * cell
        x1 = (max(c[0] for c in comp) + 1) * cell
        y1 = (max(c[1] for c in comp) + 1) * cell
        parts = sorted({r for r, fp in pcb.footprints.items()
                        if any(x0 <= p.global_x <= x1 and y0 <= p.global_y <= y1
                               for p in fp.pads)})
        through = defaultdict(float)
        for c in comp:
            for nid, L in netlen[c].items():
                through[nid] += L
        nv = sum(1 for v in pcb.vias if x0 <= v.x < x1 and y0 <= v.y < y1)
        areas.append({'box': [round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)],
                      'cells': len(comp), 'peak': round(max(occ[c] for c in comp) / cap, 2),
                      'mean': round(sum(occ[c] for c in comp) / cap / len(comp), 2),
                      'vias': nv, 'parts': parts,
                      'nets': [name(n) for n, _L in sorted(through.items(),
                                                           key=lambda kv: -kv[1])[:8]]})
    areas.sort(key=lambda a: (-(a['mean'] * a['cells']), a['box']))

    # ---- two-pad parts turned against their wiring
    turned = []
    for ref, fp in sorted(pcb.footprints.items()):
        pads = [p for p in fp.pads if p.net_id and p.net_id not in view.rails]
        if len(fp.pads) != 2 or len(pads) != 2 or pads[0].net_id == pads[1].net_id:
            continue

        def nearest(pad, x, y):
            o = [(q.global_x, q.global_y) for q in pcb.pads_by_net.get(pad.net_id, ())
                 if q.component_ref != ref]
            return min((math.dist((x, y), q) for q in o), default=None)
        a, b = pads
        now = (nearest(a, a.global_x, a.global_y), nearest(b, b.global_x, b.global_y))
        flip = (nearest(a, b.global_x, b.global_y), nearest(b, a.global_x, a.global_y))
        if None in now or None in flip:
            continue
        gain = sum(now) - sum(flip)
        if gain > 1.0 and flip[0] <= now[0] + 1e-9 and flip[1] <= now[1] + 1e-9:
            # The pose that swaps the pads: a half turn about their midpoint.
            mx = (a.global_x + b.global_x) / 2.0
            my = (a.global_y + b.global_y) / 2.0
            turned.append({'ref': ref, 'x': round(fp.x, 2), 'y': round(fp.y, 2),
                           'rot': fp.rotation or 0, 'saves_mm': round(gain, 1),
                           'swap_pose': [round(2 * mx - fp.x, 4),
                                         round(2 * my - fp.y, 4),
                                         ((fp.rotation or 0) + 180) % 360]})
    turned.sort(key=lambda t: (-t['saves_mm'], t['ref']))

    return {'board': pcb_file, 'layers_with_signal': layers, 'pitch_mm': round(pitch, 3),
            'cell_mm': cell, 'hot': hot,
            'open_nets': opens, 'most_vias': by_vias, 'longest_detours': by_detour,
            'congested_areas': areas[:top], 'turned_parts': turned[:top],
            'counts': {'open': len(opens), 'open_signal': sum(1 for o in opens if not o['poured']),
                       'congested_areas': len(areas), 'turned_parts': len(turned)}}


def _print(r):
    c = r['counts']
    print(f"route_review: {c['open']} open net(s), {c['open'] - c['open_signal']} of them poured; "
          f"{c['congested_areas']} congested area(s), {c['turned_parts']} turned part(s); "
          f"signal layers {', '.join(r['layers_with_signal'])}, pitch {r['pitch_mm']} mm")
    if r['open_nets']:
        print("== open nets")
        for o in r['open_nets']:
            tag = ' (poured)' if o['poured'] else ('' if o['copper'] else ' (no copper)')
            print(f"   {o['net']}{tag}: disconnected at {', '.join(o['disconnected_pad_parts']) or '?'}"
                  f"; on {', '.join(o['parts'][:8])}")
    if r['most_vias']:
        print("== most vias (signal nets)")
        for n in r['most_vias']:
            print(f"   {n['net'][:36]:36s} vias {n['vias']:2d}  routed {n['routed_mm']:6.1f} mm"
                  f"  tree {n['tree_mm']:6.1f}  {', '.join(n['parts'][:6])}")
    if r['longest_detours']:
        print("== longest detours (routed / tree, tree > 3 mm)")
        for n in r['longest_detours']:
            print(f"   {n['net'][:36]:36s} x{n['detour']:4.1f}  routed {n['routed_mm']:6.1f} mm"
                  f"  tree {n['tree_mm']:6.1f}  vias {n['vias']}  {', '.join(n['parts'][:6])}")
    if r['congested_areas']:
        print(f"== congested areas (occupancy >= {r['hot']} on {r['cell_mm']} mm cells)")
        for a in r['congested_areas']:
            print(f"   box {a['box']}  cells {a['cells']}  mean {a['mean']}  peak {a['peak']}"
                  f"  vias {a['vias']}\n      parts: {', '.join(a['parts'][:14])}"
                  f"\n      nets:  {', '.join(a['nets'])}")
    if r['turned_parts']:
        print("== two-pad parts turned against their wiring (turn 180)")
        for t in r['turned_parts']:
            sx, sy, sr = t['swap_pose']
            print(f"   {t['ref']:6s} at ({t['x']}, {t['y']}) rot {t['rot']:g}: saves {t['saves_mm']} mm"
                  f"  -> place_pose BOARD OUT set {t['ref']} {sx} {sy} --rot {sr:g}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board', help='the routed .kicad_pcb')
    ap.add_argument('--cell', type=float, default=2.0, help='congestion grid cell, mm (default 2)')
    ap.add_argument('--hot', type=float, default=0.5,
                    help='occupancy that makes a cell congested (default 0.5)')
    ap.add_argument('--top', type=int, default=8, help='rows per list (default 8)')
    ap.add_argument('--json', help='write the review here')
    a = ap.parse_args(argv)
    try:
        pcb = parse_kicad_pcb(a.board)
    except (OSError, ValueError) as e:
        print(f"route_review: cannot read {a.board}: {e}", file=sys.stderr)
        return 2
    r = review(a.board, cell=a.cell, hot=a.hot, top=a.top, pcb=pcb)
    _print(r)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(r, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
