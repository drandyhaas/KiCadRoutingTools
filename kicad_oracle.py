#!/usr/bin/env python3
"""KiCad-oracle reconnect pass (#217): satisfy KiCad's own connectivity.

Our raster fill model over-credits (zone-outline union, coarse blocked-cell
stamping), so some gaps KiCad's REAL fill produces are invisible to region
detection: a fill island the model believes is attached (castor +3.3VA at
(45,99)), a pad in a clearance-carved pocket the fill never enters (lumenpnp
U5 GND). Chasing raster fidelity has diminishing returns; KiCad itself is
the authority. This pass runs `kicad-cli pcb drc --refill-zones`, takes each
reported unconnected pair's EXACT endpoints (position + layer), and routes
precisely those missing links with the plane-join router -- repeating until
KiCad reports the processed nets complete or a round makes no progress.

CLI-only by design: it shells out to kicad-cli (auto-detected; skipped with
a note when absent). The GUI runs inside pcbnew where real fills are native
-- a future GUI equivalent should use them directly.
"""
import json
import math
import os
import re
import shutil
import subprocess
import tempfile
from typing import List, Optional, Tuple

KICAD_CLI_CANDIDATES = [
    shutil.which('kicad-cli'),
    '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
    '/usr/lib/kicad/bin/kicad-cli',
]


def find_kicad_cli() -> Optional[str]:
    for c in KICAD_CLI_CANDIDATES:
        if c and os.path.exists(c):
            return c
    return None


_NET_RE = re.compile(r'\[([^\]]+)\]')
_LAYER_RE = re.compile(r'\bon ([A-Za-z0-9_.]+\.Cu)\b')


def _parse_item(item: dict) -> Optional[Tuple[str, float, float, Optional[str]]]:
    """(net, x, y, layer_or_None) from one DRC unconnected sub-item."""
    desc = item.get('description', '')
    pos = item.get('pos') or {}
    m = _NET_RE.search(desc)
    if m is None or 'x' not in pos:
        return None
    lm = _LAYER_RE.search(desc)
    # Vias span layers ("on F.Cu - B.Cu" doesn't match the single-layer re);
    # layer None lets the router stamp all layers at a via position.
    return (m.group(1), float(pos['x']), float(pos['y']),
            lm.group(1) if lm else None)


def kicad_unconnected(board_file: str, kicad_cli: str) -> Optional[List[Tuple]]:
    """[(net, (x,y,layer|None), (x,y,layer|None)), ...] per kicad-cli DRC
    unconnected item, after a zone refill. None on tool failure."""
    with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as f:
        out = f.name
    try:
        r = subprocess.run(
            [kicad_cli, 'pcb', 'drc', board_file, '--format', 'json',
             '-o', out, '--severity-all', '--refill-zones'],
            capture_output=True, text=True, timeout=600)
        if r.returncode not in (0, 5):  # 5 = violations exist, still wrote json
            return None
        with open(out) as f:
            data = json.load(f)
    except Exception:
        return None
    finally:
        try:
            os.unlink(out)
        except OSError:
            pass
    links = []
    for u in data.get('unconnected_items', []):
        items = u.get('items', [])
        if len(items) < 2:
            continue
        a = _parse_item(items[0])
        b = _parse_item(items[1])
        if a and b and a[0] == b[0]:
            links.append((a[0], (a[1], a[2], a[3]), (b[1], b[2], b[3])))
    return links


def _net_track_components(pcb_data, net_id):
    """Track+via components of a net, graded with NO zone-fill credit.
    Returns (comp_of_seg: list[root per segment], comp_of_via, segs, vias,
    comp_len: {root: total_mm})."""
    from check_connected import check_net_connectivity
    from geometry_utils import UnionFind
    from collections import defaultdict
    segs = [s for s in pcb_data.segments if s.net_id == net_id]
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    if not segs:
        return [], [], segs, vias, {}
    r = check_net_connectivity(net_id, segs, vias, [], [], return_graph=True)
    graph = r.get('graph')
    if not graph:
        return [], [], segs, vias, {}
    uf = UnionFind()
    for a, b in graph.get('edges', []):
        uf.union(a, b)
    comp_of_seg = [uf.find(2 * i) for i in range(len(segs))]
    n2 = 2 * len(segs)
    via_ids = graph.get('via_index_repr', {})
    comp_of_via = []
    for j, v in enumerate(vias):
        rep = via_ids.get(j)
        comp_of_via.append(uf.find(rep) if rep is not None else None)
    comp_len = defaultdict(float)
    for i, s in enumerate(segs):
        comp_len[comp_of_seg[i]] += math.hypot(s.end_x - s.start_x,
                                               s.end_y - s.start_y)
    return comp_of_seg, comp_of_via, segs, vias, comp_len


def _component_points(segs, vias, comp_of_seg, comp_of_via, root,
                      max_pts: int = 40):
    """Sample (x, y, layer) / (x, y) points across one component."""
    pts = []
    for i, s in enumerate(segs):
        if comp_of_seg[i] == root:
            pts.append((s.start_x, s.start_y, s.layer))
            pts.append((s.end_x, s.end_y, s.layer))
    for j, v in enumerate(vias):
        if comp_of_via[j] == root:
            pts.append((v.x, v.y))  # via: all layers
    if len(pts) > max_pts:
        stride = len(pts) // max_pts + 1
        pts = pts[::stride]
    return pts


def _cluster_points(pcb_data, net_id, x, y, layer, comps, tol=0.06):
    """Expand a reported endpoint to its WHOLE copper cluster: any copper
    of the island is an equally good attachment, and the reported nib is
    often the worst (boxed into the very pocket that caused the gap). Falls
    back to the single point when no track/via contains it (bare fill)."""
    comp_of_seg, comp_of_via, segs, vias, _ = comps
    root = None
    for i, s in enumerate(segs):
        if layer is not None and s.layer != layer:
            continue
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        L2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((x - s.start_x) * dx + (y - s.start_y) * dy) / L2)) if L2 else 0.0
        if math.hypot(x - (s.start_x + t * dx),
                      y - (s.start_y + t * dy)) <= s.width / 2 + tol:
            root = comp_of_seg[i]
            break
    if root is None:
        for j, v in enumerate(vias):
            if math.hypot(x - v.x, y - v.y) <= v.size / 2 + tol:
                root = comp_of_via[j]
                break
    if root is None:
        return [(x, y, layer)] if layer else [(x, y)], None
    return (_component_points(segs, vias, comp_of_seg, comp_of_via, root)
            or ([(x, y, layer)] if layer else [(x, y)])), root


def _trace_real_island(start, net_id, layer, pcb_data, zone_polys, margin,
                       step=0.25, max_cells=40000):
    """BFS over provably-REAL fill from the ratsnest point: cells inside one
    zone outline whose `margin` disc is clear of all foreign copper (exact
    geometry, spatially bucketed). This maps the actual island KiCad saw,
    so seeds can come from anywhere on it -- not just the one reported
    point, which often sits at the island's edge."""
    from collections import deque
    from check_drc import point_to_pad_distance
    from check_connected import point_in_polygon

    buckets = {}

    def _add_span(x1, y1, x2, y2, reach, obj):
        for bx in range(int(min(x1, x2) - reach) - 1,
                        int(max(x1, x2) + reach) + 2):
            for by in range(int(min(y1, y2) - reach) - 1,
                            int(max(y1, y2) + reach) + 2):
                buckets.setdefault((bx, by), []).append(obj)

    for v in pcb_data.vias:
        if v.net_id != net_id:
            _add_span(v.x, v.y, v.x, v.y, v.size / 2 + margin,
                      ('c', v.x, v.y, v.size / 2))
    for s in pcb_data.segments:
        if s.net_id != net_id and s.layer == layer:
            _add_span(s.start_x, s.start_y, s.end_x, s.end_y,
                      s.width / 2 + margin, ('s', s))
    for pads in pcb_data.pads_by_net.values():
        for p in pads:
            if p.net_id == net_id:
                continue
            if p.drill <= 0 and layer not in p.layers \
                    and '*.Cu' not in p.layers:
                continue
            r = max(p.size_x, p.size_y) / 2
            _add_span(p.global_x, p.global_y, p.global_x, p.global_y,
                      r + margin, ('p', p))

    def clear(x, y):
        probes = ((x, y), (x + margin, y), (x - margin, y),
                  (x, y + margin), (x, y - margin))
        if not any(all(point_in_polygon(px, py, poly) for px, py in probes)
                   for poly in zone_polys):
            return False
        for obj in buckets.get((int(x), int(y)), ()):
            if obj[0] == 'c':
                _, cx, cy, r = obj
                if math.hypot(x - cx, y - cy) < r + margin:
                    return False
            elif obj[0] == 's':
                s = obj[1]
                dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
                L2 = dx * dx + dy * dy
                t = max(0.0, min(1.0, ((x - s.start_x) * dx +
                                       (y - s.start_y) * dy) / L2)) if L2 else 0.0
                if math.hypot(x - (s.start_x + t * dx),
                              y - (s.start_y + t * dy)) < s.width / 2 + margin:
                    return False
            else:
                if point_to_pad_distance(x, y, obj[1]) < margin:
                    return False
        return True

    # The ratsnest anchor often sits at the island EDGE and fails the
    # conservative disc test; spiral to the nearest clear cell within ~1mm.
    g0 = (round(start[0] / step), round(start[1] / step))
    seed = None
    for rr in range(0, 5):
        for dgx in range(-rr, rr + 1):
            for dgy in range(-rr, rr + 1):
                if max(abs(dgx), abs(dgy)) != rr:
                    continue
                gx, gy = g0[0] + dgx, g0[1] + dgy
                if clear(gx * step, gy * step):
                    seed = (gx, gy)
                    break
            if seed:
                break
        if seed:
            break
    if seed is None:
        return set()
    cells = {seed}
    q = deque([seed])
    while q and len(cells) < max_cells:
        gx, gy = q.popleft()
        for dgx, dgy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            n = (gx + dgx, gy + dgy)
            if n in cells:
                continue
            if clear(n[0] * step, n[1] * step):
                cells.add(n)
                q.append(n)
    return cells


def _island_seed_points(cells, step, pcb_data, net_id, layer,
                        routing_layers, start, max_cell_pts=24):
    """Seeds across the traced island: every existing via and THT pad on it
    (the best join points -- they already bond the fill), SMD pads and track
    ends on the zone layer, a subsample of the fill itself, and the reported
    point. Multi-source A* then attaches wherever is cheapest."""
    def inside(x, y):
        return (round(x / step), round(y / step)) in cells
    pts = []
    for v in pcb_data.vias:
        if v.net_id == net_id and inside(v.x, v.y):
            pts.append((v.x, v.y))  # via: all layers
    for p in pcb_data.pads_by_net.get(net_id, []):
        if not inside(p.global_x, p.global_y):
            continue
        if p.drill > 0 or '*.Cu' in p.layers:
            pts.append((p.global_x, p.global_y, routing_layers[0]))
            pts.append((p.global_x, p.global_y, routing_layers[-1]))
        elif layer in p.layers:
            pts.append((p.global_x, p.global_y, layer))
    for s in pcb_data.segments:
        if s.net_id == net_id and s.layer == layer \
                and inside(s.start_x, s.start_y):
            pts.append((s.start_x, s.start_y, layer))
    cell_pts = [(gx * step, gy * step, layer) for gx, gy in cells]
    if len(cell_pts) > max_cell_pts:
        stride = len(cell_pts) // max_cell_pts + 1
        cell_pts = cell_pts[::stride]
    pts.extend(cell_pts)
    pts.append((start[0], start[1], layer))
    return pts


def _merge_collinear(route_points):
    """Collapse same-layer collinear runs of raw grid steps into single
    long segments, matching how other tracks are written."""
    if len(route_points) < 3:
        return route_points
    out = [route_points[0]]
    for p in route_points[1:]:
        if len(out) >= 2:
            x1, y1, l1 = out[-2]
            x2, y2, l2 = out[-1]
            x3, y3, l3 = p
            if l1 == l2 == l3:
                cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
                dot = (x2 - x1) * (x3 - x2) + (y2 - y1) * (y3 - y2)
                if abs(cross) < 1e-9 and dot >= 0:
                    out[-1] = p
                    continue
        out.append(p)
    return out


def _largest_track_component_points(pcb_data, net_id, max_pts: int = 40):
    """Sample points (x, y, layer) on the net's largest track+via copper
    component, graded WITHOUT any zone-fill credit. The biggest genuine
    copper harness is the best available proxy for 'the main cluster'."""
    from check_connected import check_net_connectivity
    from geometry_utils import UnionFind
    segs = [s for s in pcb_data.segments if s.net_id == net_id]
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    if not segs:
        return []
    r = check_net_connectivity(net_id, segs, vias, [], [], return_graph=True)
    graph = r.get('graph')
    if not graph:
        return []
    uf = UnionFind()
    for a, b in graph.get('edges', []):
        uf.union(a, b)
    from collections import defaultdict
    comp_len = defaultdict(float)
    comp_segs = defaultdict(list)
    for i, s in enumerate(segs):
        root = uf.find(2 * i)
        comp_len[root] += math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        comp_segs[root].append(s)
    if not comp_len:
        return []
    best = max(comp_len, key=comp_len.get)
    pts = []
    for s in comp_segs[best]:
        pts.append((s.start_x, s.start_y, s.layer))
        pts.append((s.end_x, s.end_y, s.layer))
    if len(pts) > max_pts:
        stride = len(pts) // max_pts + 1
        pts = pts[::stride]
    return pts


def oracle_reconnect(board_file: str, net_names, config,
                     track_via_clearance: float,
                     hole_to_hole_clearance: float,
                     max_rounds: int = 3,
                     max_iterations: int = 1_000_000,
                     verbose: bool = False) -> dict:
    """Route the exact missing links kicad-cli reports for `net_names` on
    `board_file`, in place, until KiCad is satisfied or no progress.

    Returns {'available': bool, 'rounds': n, 'links_routed': n,
             'links_failed': n, 'remaining': n}.
    """
    from dataclasses import replace
    from kicad_parser import parse_kicad_pcb, is_kicad_10
    from kicad_writer import generate_segment_sexpr, generate_via_sexpr
    from plane_region_connector import (build_base_obstacles,
                                        route_plane_connection_wide)

    kicad_cli = find_kicad_cli()
    if kicad_cli is None:
        print("  KiCad-oracle recheck: kicad-cli not found, skipping")
        return {'available': False, 'rounds': 0, 'links_routed': 0,
                'links_failed': 0, 'remaining': -1}

    names = set(net_names)
    routed = failed = rounds = 0
    remaining = -1
    for rnd in range(max_rounds):
        links = kicad_unconnected(board_file, kicad_cli)
        if links is None:
            print("  KiCad-oracle recheck: kicad-cli DRC failed, skipping")
            break
        ours = [l for l in links if l[0] in names]
        remaining = len(ours)
        if not ours:
            if rnd == 0:
                print("  KiCad-oracle recheck: KiCad reports all processed "
                      "nets complete")
            break
        rounds += 1
        print(f"  KiCad-oracle recheck round {rnd + 1}: KiCad reports "
              f"{len(ours)} missing link(s) on processed nets")

        pcb_data = parse_kicad_pcb(board_file)
        name_to_id = {net.name: nid for nid, net in pcb_data.nets.items()}
        routing_layers = pcb_data.board_info.copper_layers
        layer_map = {name: i for i, name in enumerate(routing_layers)}
        with open(board_file, 'r', encoding='utf-8') as f:
            content = f.read()
        v10 = is_kicad_10(content)
        new_sexprs = []
        progress = False

        for net_name, (ax, ay, al), (bx, by, bl) in ours:
            net_id = name_to_id.get(net_name)
            if net_id is None:
                failed += 1
                continue
            base_obstacles, _ = build_base_obstacles(
                exclude_net_ids={net_id},
                routing_layers=routing_layers,
                pcb_data=pcb_data,
                config=config,
                track_width=config.track_width,
                track_via_clearance=track_via_clearance,
                hole_to_hole_clearance=hole_to_hole_clearance)
            net_vias = [(v.x, v.y) for v in pcb_data.vias
                        if v.net_id == net_id]
            island_fallback = False
            comps = _net_track_components(pcb_data, net_id)
            src, root_a = _cluster_points(pcb_data, net_id, ax, ay, al, comps)
            tgt, root_b = _cluster_points(pcb_data, net_id, bx, by, bl, comps)
            if root_a is not None and root_a == root_b:
                # Both reported points sit on the same track cluster; the
                # split must be fill-side. Fall back to the raw points.
                src = [(ax, ay, al)] if al else [(ax, ay)]
                tgt = [(bx, by, bl)] if bl else [(bx, by)]
            if abs(ax - bx) < 1e-6 and abs(ay - by) < 1e-6:
                # Zone|Zone items carry ONE ratsnest position for both ends
                # (the isolated island). Target copper provably in the MAIN
                # cluster: the net's largest track+via component, graded
                # with NO fill credit (routing to the nearest pad connected
                # the castor island to RV4.3 -- itself a dangling pad -- and
                # the merged cluster still floated). Pads are the fallback
                # when a net has no track copper, on their outer layers only
                # (remove_unused_layers can strip inner PTH annuli).
                island_fallback = True
                tgt = _largest_track_component_points(pcb_data, net_id)
                if not tgt:
                    for p in pcb_data.pads_by_net.get(net_id, []):
                        cu = [l for l in p.layers if l.endswith('.Cu')]
                        if p.drill > 0 or '*.Cu' in p.layers:
                            tgt.append((p.global_x, p.global_y, routing_layers[0]))
                            tgt.append((p.global_x, p.global_y, routing_layers[-1]))
                        elif cu:
                            tgt.append((p.global_x, p.global_y, cu[0]))
                if not tgt:
                    failed += 1
                    continue
                if al:
                    zone_polys = [z.polygon for z in
                                  (getattr(pcb_data, 'zones', []) or [])
                                  if z.net_id == net_id
                                  and getattr(z, 'polygon', None)]
                    if zone_polys:
                        _margin = max(config.clearance, 0.15)
                        _cells = _trace_real_island(
                            (ax, ay), net_id, al, pcb_data, zone_polys,
                            _margin)
                        if _cells:
                            src = _island_seed_points(
                                _cells, 0.25, pcb_data, net_id, al,
                                routing_layers, (ax, ay))
            anchor_layer = layer_map.get(al or bl or routing_layers[0], 0)
            result = None
            used_via_size, used_via_drill = config.via_size, config.via_drill
            # Via-size ladder: a 0.5 via has nowhere to drop in a QFN pocket
            # (lumenpnp U5); the fab-floor 0.45/0.2 rung mirrors the
            # fine-pitch tap escalation.
            for vs, vd in ((config.via_size, config.via_drill), (0.45, 0.2)):
                if vs > config.via_size:
                    continue
                rung_cfg = config if vs == config.via_size else \
                    replace(config, via_size=vs, via_drill=vd)
                rung_obstacles = base_obstacles
                if rung_cfg is not config:
                    rung_obstacles, _ = build_base_obstacles(
                        exclude_net_ids={net_id},
                        routing_layers=routing_layers,
                        pcb_data=pcb_data,
                        config=rung_cfg,
                        track_width=rung_cfg.track_width,
                        track_via_clearance=track_via_clearance,
                        hole_to_hole_clearance=hole_to_hole_clearance)
                result, _iters = route_plane_connection_wide(
                    src, tgt,
                    plane_layer_idx=anchor_layer,
                    routing_layers=routing_layers,
                    base_obstacles=rung_obstacles,
                    config=rung_cfg,
                    net_vias=net_vias,
                    track_margin=0,
                    max_iterations=max_iterations,
                    verbose=verbose)
                if result:
                    used_via_size, used_via_drill = vs, vd
                    break
            used_width = config.track_width
            if result:
                # Width upgrade (join-style): the narrow route found the
                # corridor; a plane link should carry the widest copper that
                # fits (0.127 signal width on an open plane bridge is
                # needlessly thin). Same quantization-guarded margin as the
                # region joins; stop at the first width that no longer fits.
                from single_ended_routing import _track_margin_for_width
                for w in (0.2, 0.4, 0.8):
                    if w <= used_width:
                        continue
                    margin = _track_margin_for_width(
                        w, rung_cfg.track_width, rung_cfg.grid_step)
                    wider, _ = route_plane_connection_wide(
                        src, tgt,
                        plane_layer_idx=anchor_layer,
                        routing_layers=routing_layers,
                        base_obstacles=rung_obstacles,
                        config=rung_cfg,
                        net_vias=net_vias,
                        track_margin=margin,
                        max_iterations=max_iterations,
                        verbose=verbose)
                    if not wider:
                        break
                    result, used_width = wider, w
            if not result:
                print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                      f"<->({bx:.2f},{by:.2f})  FAILED")
                failed += 1
                continue
            route_points, via_positions = result
            route_points = _merge_collinear(route_points)
            n_segs = 0
            for k in range(len(route_points) - 1):
                x1, y1, l1 = route_points[k]
                x2, y2, l2 = route_points[k + 1]
                if l1 != l2 or (abs(x1 - x2) < 1e-9 and abs(y1 - y2) < 1e-9):
                    continue
                new_sexprs.append(generate_segment_sexpr(
                    (x1, y1), (x2, y2), used_width, l1, net_id,
                    net_name if v10 else None))
                n_segs += 1
            for vx, vy in via_positions:
                new_sexprs.append(generate_via_sexpr(
                    vx, vy, used_via_size, used_via_drill,
                    [routing_layers[0], routing_layers[-1]], net_id,
                    net_name=net_name if v10 else None))
            print(f"    {net_name}: ({ax:.2f},{ay:.2f})<->({bx:.2f},{by:.2f})"
                  f"  OK {n_segs} seg(s), {len(via_positions)} via(s), "
                  f"w={used_width:.2f}mm")
            routed += 1
            progress = True

        if new_sexprs:
            idx = content.rfind(')')
            content = content[:idx] + ''.join(new_sexprs) + content[idx:]
            with open(board_file, 'w', encoding='utf-8') as f:
                f.write(content)
        if not progress:
            break
    else:
        # ran all rounds; get the final count
        links = kicad_unconnected(board_file, kicad_cli)
        if links is not None:
            remaining = len([l for l in links if l[0] in names])

    if rounds and remaining > 0:
        print(f"  KiCad-oracle recheck: {remaining} link(s) still "
              f"unconnected per KiCad after {rounds} round(s)")
    return {'available': True, 'rounds': rounds, 'links_routed': routed,
            'links_failed': failed, 'remaining': remaining}
