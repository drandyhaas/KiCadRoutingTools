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
import time
from typing import List, Optional, Tuple

from kicad_parser import Segment as _Seg, Via as _Via

KICAD_CLI_CANDIDATES = [
    shutil.which('kicad-cli'),
    '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
    '/usr/lib/kicad/bin/kicad-cli',
]

# Per-round kicad-cli DRC timeout (#420). Was 600s: some boards
# (lily58: keyboard-shaped zones) made kicad-cli's DRC spend 10+ minutes in
# pathological polygon triangulation (SHAPE_POLY_SET::Collide ->
# cacheTriangulation, driven by the silk/copper-clearance providers), burning
# the whole timeout and losing the oracle data. With the fast-connectivity
# project below a normal DRC is seconds, so a genuinely stuck board is caught
# far sooner and its remaining rounds are skipped.
ORACLE_DRC_TIMEOUT = 240

# realpath() of boards whose kicad-cli DRC blew ORACLE_DRC_TIMEOUT once. The
# oracle is called once per plane-repair step, so remembering a slow board
# stops it re-burning the timeout on every later step of the same run.
_ORACLE_TIMED_OUT = set()

# The oracle reads ONLY unconnected_items, which KiCad's connectivity engine
# computes independently of the geometric DRC rule providers. Forcing every
# such provider to "ignore" (#420) skips the expensive silk/copper-clearance
# polygon triangulation while leaving the ratsnest (unconnected_items) intact.
# Zone-fill geometry is unaffected -- only rule SEVERITIES change here, never
# the clearance values or net classes that drive the fill. Empirically this
# took a lily58 oracle DRC from 10+ min to ~6 s with an identical 13-item
# unconnected report.
_IGNORE_SEVERITIES = [
    "clearance", "creepage", "hole_clearance", "edge_clearance",
    "hole_near_hole", "hole_to_hole", "track_width", "annular_width",
    "drill_out_of_range", "via_diameter", "padstack",
    "microvia_drill_out_of_range", "courtyards_overlap", "malformed_courtyard",
    "pth_inside_courtyard", "npth_inside_courtyard", "item_on_disabled_layer",
    "invalid_outline", "duplicate_footprints", "missing_footprint",
    "net_conflict", "unresolved_variable", "assertion_failure",
    "copper_sliver", "silk_over_copper", "silk_overlap", "silk_edge_clearance",
    "text_height", "text_thickness", "length_out_of_range",
    "skew_out_of_range", "via_count_exceeded", "diff_pair_gap_out_of_range",
    "diff_pair_uncoupled_length_too_long", "footprint_type_mismatch",
    "through_hole_pad_without_hole", "extra_footprint", "solder_mask_bridge",
    "silk_mask_clearance", "starved_thermal", "connection_width",
    "zone_has_empty_net", "lib_footprint_issues", "lib_footprint_mismatch",
    "footprint", "holes_co_located",
]


def find_kicad_cli() -> Optional[str]:
    for c in KICAD_CLI_CANDIDATES:
        if c and os.path.exists(c):
            return c
    return None


def _fast_connectivity_project(board_file: str):
    """Stage `board_file` in a temp dir beside a .kicad_pro whose DRC rule
    severities are all 'ignore' except unconnected_items (#420), so kicad-cli
    skips the pathological silk/copper-clearance polygon triangulation and
    still reports the ratsnest opens the oracle consumes.

    The staged project preserves the real board's design rules and net
    classes (only severities are flipped), so `--refill-zones` produces the
    same fill geometry -- the connectivity result is identical, just fast.
    When the board has no sibling .kicad_pro yet (mid-chain, before the
    project is written), a minimal ignore-only project is synthesized, which
    matches kicad-cli's own default-rule fill it would have used anyway.

    Returns (staged_board_path, tempdir) on success, or (board_file, None) if
    staging fails (caller runs kicad-cli directly on the original board)."""
    try:
        sibling_pro = os.path.splitext(board_file)[0] + '.kicad_pro'
        proj = {}
        if os.path.exists(sibling_pro):
            with open(sibling_pro, 'r', encoding='utf-8') as f:
                proj = json.load(f)
        ds = proj.setdefault('board', {}).setdefault('design_settings', {})
        sev = dict(ds.get('rule_severities', {}))
        for k in _IGNORE_SEVERITIES:
            sev[k] = 'ignore'
        sev['unconnected_items'] = 'error'
        ds['rule_severities'] = sev
        if 'meta' not in proj:
            proj['meta'] = {'filename': 'oracle.kicad_pro', 'version': 1}

        tmpdir = tempfile.mkdtemp(prefix='oracle_drc_')
        stem = os.path.splitext(os.path.basename(board_file))[0]
        staged_pcb = os.path.join(tmpdir, stem + '.kicad_pcb')
        staged_pro = os.path.join(tmpdir, stem + '.kicad_pro')
        shutil.copyfile(board_file, staged_pcb)
        with open(staged_pro, 'w', encoding='utf-8') as f:
            json.dump(proj, f, indent=2)
        return staged_pcb, tmpdir
    except Exception:
        return board_file, None


# Greedy: a net name may itself contain ']' (the old lazy match truncated
# 'BUS[3]' to 'BUS[3' and the link silently vanished). Descriptions carry
# exactly one bracket pair around the net, so greedy is safe.
_NET_RE = re.compile(r'\[(.*)\]')
_LAYER_RE = re.compile(r'\bon ([A-Za-z0-9_.]+\.Cu)\b(?!\s*-)')


def _parse_item(item: dict) -> Optional[Tuple[str, float, float, Optional[str]]]:
    """(net, x, y, layer_or_None) from one DRC unconnected sub-item."""
    desc = item.get('description', '')
    pos = item.get('pos') or {}
    m = _NET_RE.search(desc)
    if m is None or 'x' not in pos:
        return None
    lm = None if ' - ' in desc else _LAYER_RE.search(desc)
    kind = 'zone' if desc.startswith('Zone') else \
        'via' if desc.startswith('Via') else \
        'pad' if 'pad' in desc.lower().split('[')[0] else 'track'
    # Vias span layers ("on F.Cu - B.Cu"); layer None lets the router stamp
    # all layers at a via position.
    return (m.group(1), float(pos['x']), float(pos['y']),
            lm.group(1) if lm else None, kind)


def kicad_unconnected(board_file: str, kicad_cli: str,
                      timeout: int = ORACLE_DRC_TIMEOUT) -> Optional[List[Tuple]]:
    """[(net, (x,y,layer|None), (x,y,layer|None)), ...] per kicad-cli DRC
    unconnected item, after a zone refill. None on tool failure.

    Runs against a fast-connectivity staged project (#420) so the DRC skips
    the expensive geometric providers and returns in seconds; a board that
    still blows `timeout` is remembered in _ORACLE_TIMED_OUT so later steps
    skip it instead of re-burning the wall time."""
    with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as f:
        out = f.name
    staged, tmpdir = _fast_connectivity_project(board_file)
    t0 = time.monotonic()
    try:
        r = subprocess.run(
            [kicad_cli, 'pcb', 'drc', staged, '--format', 'json',
             '-o', out, '--severity-all', '--refill-zones'],
            capture_output=True, text=True, timeout=timeout)
        if r.returncode not in (0, 5):  # 5 = violations exist, still wrote json
            return None
        with open(out) as f:
            data = json.load(f)
    except subprocess.TimeoutExpired:
        dt = time.monotonic() - t0
        _ORACLE_TIMED_OUT.add(os.path.realpath(board_file))
        print(f"  KiCad-oracle recheck: WARNING kicad-cli DRC timed out after "
              f"{dt:.0f}s (>{timeout}s) on {os.path.basename(board_file)}; "
              f"skipping the oracle for this board")
        return None
    except Exception:
        return None
    finally:
        try:
            os.unlink(out)
        except OSError:
            pass
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)
    dt = time.monotonic() - t0
    if dt > 60:
        print(f"  KiCad-oracle recheck: WARNING kicad-cli DRC took {dt:.0f}s "
              f"on {os.path.basename(board_file)}")
    links = []
    for u in data.get('unconnected_items', []):
        items = u.get('items', [])
        if len(items) < 2:
            continue
        a = _parse_item(items[0])
        b = _parse_item(items[1])
        if a and b and a[0] == b[0]:
            links.append((a[0], (a[1], a[2], a[3], a[4]),
                          (b[1], b[2], b[3], b[4])))
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
                        routing_layers, start, max_cell_pts=24,
                        track_half=0.0):
    """Seeds across the traced island: every existing via and THT pad on it
    (the best join points -- they already bond the fill), SMD pads and track
    ends on the zone layer, a subsample of the fill itself, and the reported
    point. Multi-source A* then attaches wherever is cheapest.

    Seeds are stamped source/target cells and override the obstacle map's
    NPTH drill keep-out, so bare-fill seeds (and the raw reported point) are
    filtered to the NPTH-to-track fab floor at `track_half` (#390) -- zone
    fill lawfully sits closer to a no-copper hole than track copper may.
    Existing vias/pads/track-ends are kept as-is: they are already copper."""
    from plane_region_connector import npth_floor_ok

    def inside(x, y):
        return (round(x / step), round(y / step)) in cells

    def floor_ok(x, y):
        return npth_floor_ok(x, y, pcb_data, track_half)
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
    cell_pts = [(gx * step, gy * step, layer) for gx, gy in cells
                if floor_ok(gx * step, gy * step)]
    if len(cell_pts) > max_cell_pts:
        stride = len(cell_pts) // max_cell_pts + 1
        cell_pts = cell_pts[::stride]
    pts.extend(cell_pts)
    if floor_ok(start[0], start[1]):
        pts.append((start[0], start[1], layer))
    return pts


def _snap_zone_anchor(pcb_data, net_id, x, y, layer, clearance):
    """Canonicalize a kicad-reported Zone anchor to a stable point on its
    fill island. kicad-cli's fill/ratsnest anchor coordinates wobble from
    run to run (its zone fill is threaded), and every downstream decision
    keyed on the raw coordinates -- the same-position test, the Zone|Zone
    split, the attempted-retry cap keys, the routed seed sets -- wobbles
    with them, so identical invocations ship different copper. Trace the
    island the anchor lands on and return the lexicographically smallest
    fill cell's center: the same island always yields the same point.

    The trace's margin ladder can succeed at different rungs for different
    anchor positions on the same island (a pocketed anchor needs a finer
    rung than a clear one), which would change the traced cell set -- so
    one refinement pass re-traces from the canonical candidate (open fill,
    coarse rung) to make the result rung-independent. Returns (x, y)
    unchanged when no island is traceable or the trace over-floods."""
    if not layer:
        return x, y
    zp = [z.polygon for z in (getattr(pcb_data, 'zones', []) or [])
          if z.net_id == net_id and z.layer == layer
          and getattr(z, 'polygon', None)]
    if not zp:
        return x, y
    _cap = 40000

    def _trace(sx, sy):
        for _m in (max(clearance, 0.2) + 0.1, 0.2, 0.15):
            cells = _trace_real_island((sx, sy), net_id, layer, pcb_data,
                                       zp, _m, max_cells=_cap)
            if cells:
                return cells
        return None

    cells = _trace(x, y)
    if not cells or len(cells) >= _cap:
        return x, y
    gx, gy = min(cells)
    cells2 = _trace(gx * 0.25, gy * 0.25)
    if cells2 and len(cells2) < _cap:
        gx, gy = min(cells2)
    return gx * 0.25, gy * 0.25


def _merge_collinear(route_points, keep=frozenset()):
    """Collapse same-layer collinear runs of raw grid steps into single
    long segments, matching how other tracks are written. Points in `keep`
    (via positions) always stay as vertices so every via sits on a segment
    endpoint."""
    if len(route_points) < 3:
        return route_points
    out = [route_points[0]]
    for p in route_points[1:]:
        if len(out) >= 2:
            x1, y1, l1 = out[-2]
            x2, y2, l2 = out[-1]
            x3, y3, l3 = p
            if (round(x2, 3), round(y2, 3)) in keep:
                pass  # via vertex: never merged away
            elif l1 == l2 == l3:
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
                     verbose: bool = False,
                     progress_callback=None,
                     cancel_check=None) -> dict:
    """Route the exact missing links kicad-cli reports for `net_names` on
    `board_file`, in place, until KiCad is satisfied or no progress.

    progress_callback(current, total, label) fires per round (0, 0, label:
    the kicad-cli DRC run is indeterminate) and per link (k, N, label) --
    issue #364. cancel_check() returning True aborts between links/rounds.

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

    # A board whose DRC already blew ORACLE_DRC_TIMEOUT once (#420) will do it
    # again on every later plane-repair step of this run -- skip it outright
    # rather than re-burn minutes of wall time for the same lost result.
    if os.path.realpath(board_file) in _ORACLE_TIMED_OUT:
        print("  KiCad-oracle recheck: kicad-cli DRC previously timed out on "
              "this board, skipping")
        return {'available': False, 'rounds': 0, 'links_routed': 0,
                'links_failed': 0, 'remaining': -1}

    # Board-setup copper-to-edge rule (#338): the oracle links route through
    # build_base_obstacles, whose edge band comes from config.board_edge_clearance
    # (0.0 default -> track-clearance fallback). Callers that construct a bare
    # GridRouteConfig shipped oracle copper deep inside the board's edge band
    # (rp2350_dev: 30mm of GND strap 0.375mm from a 0.5mm-rule edge). Resolve
    # from the board's sibling .kicad_pro when present (final boards; a temp
    # save without a project reads 0.0 = no-op, and the explicit config value
    # still wins via max). In-band pads stay reachable through the #338 spoke
    # exemption in obstacle_map.add_board_edge_obstacles.
    try:
        from fix_kicad_drc_settings import effective_board_edge_clearance
        _eff_edge = effective_board_edge_clearance(
            board_file, config.board_edge_clearance)
        if _eff_edge > (config.board_edge_clearance or 0.0):
            print(f"  KiCad-oracle recheck: board edge clearance {_eff_edge}mm "
                  f"(project min_copper_edge_clearance)")
            config = replace(config, board_edge_clearance=_eff_edge)
    except Exception:
        pass

    names = set(net_names)
    routed = failed = rounds = 0
    remaining = -1
    emitted_segments = []  # parser objects, for callers that apply results
    emitted_vias = []      # to a live board instead of reading the file
    attempted = {}  # (net, endpoints) -> attempt count (graduated retries)
    for rnd in range(max_rounds):
        if cancel_check and cancel_check():
            print("  KiCad-oracle recheck: cancelled")
            break
        if progress_callback:
            progress_callback(0, 0, f"KiCad-oracle: running kicad-cli DRC "
                                    f"(round {rnd + 1})...")
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

        # Determinism (#365): kicad-cli's reported anchors jitter between
        # identical invocations (threaded fill/ratsnest). Snap every Zone
        # anchor to its island's canonical point so reports differing only
        # by jitter drive identical decisions (same-pos test, split, retry
        # cap, seeds), then order the links canonically so processing order
        # doesn't depend on the report's ordering either.
        _snapped = []
        for net_name, A, B in ours:
            nid = name_to_id.get(net_name)
            if nid is not None:
                ax_, ay_, al_, ak_ = A
                bx_, by_, bl_, bk_ = B
                if ak_ == 'zone':
                    ax_, ay_ = _snap_zone_anchor(pcb_data, nid, ax_, ay_,
                                                 al_, config.clearance)
                    A = (ax_, ay_, al_, ak_)
                if bk_ == 'zone':
                    bx_, by_ = _snap_zone_anchor(pcb_data, nid, bx_, by_,
                                                 bl_, config.clearance)
                    B = (bx_, by_, bl_, bk_)
            _snapped.append((net_name, A, B))
        ours = sorted(_snapped,
                      key=lambda l: (l[0], l[1][:2], l[2][:2],
                                     l[1][3], l[2][3]))

        # A Zone|Zone link with two DISTINCT ratsnest anchors can span the
        # whole board (castor +3.3V: opposite corners, 63mm -- the A*
        # exhausts 1M iterations twice). Both islands bonding to the net's
        # main track harness connects them transitively with two SHORT
        # links instead of one impossible haul.
        work = []
        for net_name, A, B in ours:
            ax_, ay_, al_, ak_ = A
            bx_, by_, bl_, bk_ = B
            if ak_ == 'zone' and bk_ == 'zone' and \
                    (abs(ax_ - bx_) > 1e-6 or abs(ay_ - by_) > 1e-6):
                work.append((net_name, A, (ax_, ay_, al_, 'main')))
                work.append((net_name, B, (bx_, by_, bl_, 'main')))
            else:
                work.append((net_name, A, B))
        for _w_idx, (net_name, (ax, ay, al, akind), (bx, by, bl, bkind)) \
                in enumerate(work):
            if cancel_check and cancel_check():
                print("  KiCad-oracle recheck: cancelled")
                break
            if progress_callback:
                progress_callback(_w_idx + 1, len(work),
                                  f"KiCad-oracle round {rnd + 1}: "
                                  f"routing {net_name} link")
            net_id = name_to_id.get(net_name)
            if net_id is None:
                failed += 1
                continue
            _key = (net_name, round(ax, 2), round(ay, 2),
                    round(bx, 2), round(by, 2))
            _attempt = attempted.get(_key, 0)
            if _attempt >= 2:
                # Two strategies already spent (expanded, then raw): stacking
                # more copper each round helps nobody. Leave it flagged.
                print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                      f"<->({bx:.2f},{by:.2f})  already attempted, leaving "
                      f"flagged")
                failed += 1
                continue
            attempted[_key] = _attempt + 1
            force_raw = _attempt == 1  # smart expansion didn't satisfy KiCad
            if force_raw:
                print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                      f"<->({bx:.2f},{by:.2f})  retry with raw endpoints")
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

            def _zone_expand(x, y, layer):
                # A Zone endpoint is a whole fill island, wherever KiCad
                # anchored the ratsnest -- trace the REAL fill and seed from
                # everything on it (vias, THT, track ends, fill cells). The
                # same-pos-only trigger missed the two-distinct-points form
                # and shipped a raw 15mm point-to-point shot (castor wave).
                if not layer:
                    return None
                zp = [z.polygon for z in (getattr(pcb_data, 'zones', []) or [])
                      if z.net_id == net_id and z.layer == layer
                      and getattr(z, 'polygon', None)]
                if not zp:
                    return None
                # Fill-flow margin ladder: KiCad's fill cannot pass a neck
                # narrower than its clearance + min_thickness, so the trace
                # prefers a wide disc (0.15 walked straight across the very
                # gap that splits the islands). But a ratsnest anchor in a
                # tight pocket finds NO clear cell at the wide margin
                # (castor +3.3V corner) -- ladder down, and when a trace
                # runs away to the max_cells cap (over-flood), trust only
                # the BFS-local cells near the reported point.
                _cap = 40000
                cells = None
                for _m in (max(config.clearance, 0.2) + 0.1, 0.2, 0.15):
                    cells = _trace_real_island((x, y), net_id, layer,
                                               pcb_data, zp, _m,
                                               max_cells=_cap)
                    if cells:
                        break
                if not cells:
                    return None
                if len(cells) >= _cap:
                    step_ = 0.25
                    cells = {(gx, gy) for (gx, gy) in cells
                             if abs(gx * step_ - x) <= 10
                             and abs(gy * step_ - y) <= 10}
                    if not cells:
                        return None
                return _island_seed_points(cells, 0.25, pcb_data, net_id,
                                           layer, routing_layers, (x, y),
                                           track_half=config.track_width / 2)

            if force_raw:
                # The expanded attempt routed copper KiCad still grades as
                # unconnected (an over-flooded trace can attach island-to-
                # same-island). The RAW reported points are guaranteed to
                # lie on the two real clusters; a direct bridge is ugly but
                # bonding, and only fires after the smart attempt failed.
                src = [(ax, ay, al)] if al else [(ax, ay)]
                if bkind != 'main':
                    tgt = [(bx, by, bl)] if bl else [(bx, by)]
            elif akind == 'zone':
                src = _zone_expand(ax, ay, al) or src
            if bkind == 'main':
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
            elif not force_raw and bkind == 'zone':
                tgt = _zone_expand(bx, by, bl) or tgt
            if root_a is not None and root_a == root_b:
                # Both reported points sit on the same track cluster; the
                # split must be fill-side. Fall back to the raw points.
                src = [(ax, ay, al)] if al else [(ax, ay)]
                tgt = [(bx, by, bl)] if bl else [(bx, by)]
            if abs(ax - bx) < 1e-6 and abs(ay - by) < 1e-6 and not force_raw:
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
                                  if z.net_id == net_id and z.layer == al
                                  and getattr(z, 'polygon', None)]
                    if zone_polys:
                        _margin = max(config.clearance, 0.2) + 0.1
                        _cells = _trace_real_island(
                            (ax, ay), net_id, al, pcb_data, zone_polys,
                            _margin)
                        if _cells:
                            src = _island_seed_points(
                                _cells, 0.25, pcb_data, net_id, al,
                                routing_layers, (ax, ay),
                                track_half=config.track_width / 2)
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
            if len(route_points) < 2 and not via_positions:
                # Degenerate 'success' (source and target expansion overlap:
                # an over-flooded island trace, or a stale report): no copper
                # would be emitted. Retry once with the RAW reported points
                # before giving up -- a point-to-point bridge is better than
                # a silent no-op marked as success.
                raw_src = [(ax, ay, al)] if al else [(ax, ay)]
                raw_tgt = [(bx, by, bl)] if bl else [(bx, by)]
                result2, _ = route_plane_connection_wide(
                    raw_src, raw_tgt,
                    plane_layer_idx=anchor_layer,
                    routing_layers=routing_layers,
                    base_obstacles=rung_obstacles,
                    config=rung_cfg,
                    net_vias=net_vias,
                    track_margin=0,
                    max_iterations=max_iterations,
                    verbose=verbose)
                if result2 and (len(result2[0]) >= 2 or result2[1]):
                    result = result2
                    route_points, via_positions = result
                else:
                    print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                          f"<->({bx:.2f},{by:.2f})  FAILED (degenerate)")
                    failed += 1
                    continue
            # Same-net drill guard (#282 class): the link's obstacle map
            # excludes the net's OWN copper, so a new via can land within
            # hole-to-hole of an existing same-net via (0.355mm overlaps on
            # splitflap GND). Reuse the existing via instead: skip the new
            # hole and bridge to the existing barrel with short connectors
            # on the two layers the path changes between (#340 style).
            _own_vias = [v for v in pcb_data.vias if v.net_id == net_id]
            _extra_conn = []
            _kept_vias = []
            for vx, vy in via_positions:
                _near = None
                for _ev in _own_vias:
                    _lim = (used_via_drill + (_ev.drill or used_via_drill)) / 2 \
                        + hole_to_hole_clearance
                    if math.hypot(vx - _ev.x, vy - _ev.y) < _lim:
                        _near = _ev
                        break
                if _near is None or (abs(_near.x - vx) < 1e-6
                                     and abs(_near.y - vy) < 1e-6):
                    _kept_vias.append((vx, vy))
                    continue
                _lys = set()
                for k in range(len(route_points) - 1):
                    x1, y1, l1 = route_points[k]
                    x2, y2, l2 = route_points[k + 1]
                    if (abs(x1 - vx) < 1e-6 and abs(y1 - vy) < 1e-6) or \
                            (abs(x2 - vx) < 1e-6 and abs(y2 - vy) < 1e-6):
                        _lys.add(l1)
                        _lys.add(l2)
                for _l in _lys:
                    _extra_conn.append(((vx, vy), (_near.x, _near.y), _l))
            via_positions = _kept_vias
            _via_keys = {(round(vx, 3), round(vy, 3)) for vx, vy in via_positions}
            route_points = _merge_collinear(route_points, keep=_via_keys)
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
            for (p1, p2, _l) in _extra_conn:
                new_sexprs.append(generate_segment_sexpr(
                    p1, p2, used_width, _l, net_id,
                    net_name if v10 else None))
                _cobj = _Seg(start_x=p1[0], start_y=p1[1],
                             end_x=p2[0], end_y=p2[1],
                             width=used_width, layer=_l, net_id=net_id)
                pcb_data.segments.append(_cobj)
                emitted_segments.append(_cobj)
                n_segs += 1
            for vx, vy in via_positions:
                new_sexprs.append(generate_via_sexpr(
                    vx, vy, used_via_size, used_via_drill,
                    [routing_layers[0], routing_layers[-1]], net_id,
                    net_name=net_name if v10 else None))
            # Same-round visibility (cross-net short fix): later links in
            # this round rebuild their obstacle maps from pcb_data, so the
            # copper just routed must exist there -- two different-net links
            # squeezing through one congested pocket otherwise cross.
            for k in range(len(route_points) - 1):
                x1, y1, l1 = route_points[k]
                x2, y2, l2 = route_points[k + 1]
                if l1 != l2 or (abs(x1 - x2) < 1e-9 and abs(y1 - y2) < 1e-9):
                    continue
                _sobj = _Seg(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                             width=used_width, layer=l1, net_id=net_id)
                pcb_data.segments.append(_sobj)
                emitted_segments.append(_sobj)
            for vx, vy in via_positions:
                _vobj = _Via(x=vx, y=vy, size=used_via_size,
                             drill=used_via_drill,
                             layers=[routing_layers[0], routing_layers[-1]],
                             net_id=net_id)
                pcb_data.vias.append(_vobj)
                emitted_vias.append(_vobj)
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
            'links_failed': failed, 'remaining': remaining,
            'new_segments': emitted_segments, 'new_vias': emitted_vias}
