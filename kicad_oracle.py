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
            if not result:
                print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                      f"<->({bx:.2f},{by:.2f})  FAILED")
                failed += 1
                continue
            route_points, via_positions = result
            n_segs = 0
            for k in range(len(route_points) - 1):
                x1, y1, l1 = route_points[k]
                x2, y2, l2 = route_points[k + 1]
                if l1 != l2 or (abs(x1 - x2) < 1e-9 and abs(y1 - y2) < 1e-9):
                    continue
                new_sexprs.append(generate_segment_sexpr(
                    (x1, y1), (x2, y2), config.track_width, l1, net_id,
                    net_name if v10 else None))
                n_segs += 1
            for vx, vy in via_positions:
                new_sexprs.append(generate_via_sexpr(
                    vx, vy, used_via_size, used_via_drill,
                    [routing_layers[0], routing_layers[-1]], net_id,
                    net_name=net_name if v10 else None))
            print(f"    {net_name}: ({ax:.2f},{ay:.2f})<->({bx:.2f},{by:.2f})"
                  f"  OK {n_segs} seg(s), {len(via_positions)} via(s)")
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
