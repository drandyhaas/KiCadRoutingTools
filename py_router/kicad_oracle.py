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

Shells out to kicad-cli (auto-detected; skipped with a note when absent).
Both fronts use it: the CLI runs it on the written file, and the GUI stages
its live board to a temp file (batch_route's stage_board_fn in-run;
gui_utils.run_kicad_oracle_on_live_board post-run) -- kicad-cli's exact fill
needs a real file either way.
"""
import env_knobs
import json
import math
import os
import re
import shutil
import subprocess
import sys
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

# Boards whose kicad-cli DRC blew ORACLE_DRC_TIMEOUT once. The oracle is called
# once per plane-repair step, so remembering a slow board stops it re-burning
# the timeout on every later step of the same run.
#
# KEYED ON A FILL-COST SIGNATURE, NOT ON THE PATH. The path key never fired in
# practice: a chain writes a NEW output board each step (r1c, r1b, r2, r3...),
# so every step got a fresh key and re-burned the full 240s. Measured on run 9:
# a board KiCad could not fill in 300s was re-attempted at every step of the
# chain, and each attempt cost the whole timeout.
#
# What actually drives the fill cost is the ZONE geometry, the outline, and the
# pad field -- not the routed copper, which grows between steps. So the memo
# keys on those, which stay stable across a chain while the copper changes.
# It is a heuristic and it is the honest one available: a board that could not
# be filled with these zones and these pads will not become fillable because
# thirty more tracks were added.
_ORACLE_TIMED_OUT = set()


def _fill_cost_key(board_file: str):
    """(zones, pads, footprints, outline-ish) -- stable across a routing chain.

    Falls back to the realpath when the board cannot be read, which restores the
    old behaviour rather than failing closed on a parse error.
    """
    try:
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(board_file)
        bb = pcb.board_info.board_bounds or (0, 0, 0, 0)
        return ('fill', len(getattr(pcb, 'zones', ()) or ()),
                sum(len(f.pads) for f in pcb.footprints.values()),
                len(pcb.footprints),
                tuple(round(v, 2) for v in bb))
    except Exception:                                          # noqa: BLE001
        return ('path', os.path.realpath(board_file))

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
    "via_dangling", "track_dangling",
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
    # The ENV OVERRIDE GOES FIRST. It used to be checked after the unix
    # candidates, so `KICAD_CLI=/my/build/kicad-cli` was ignored on any machine
    # that also had a packaged one -- an override that the presence of a
    # default silently defeats is not an override.
    env = os.environ.get('KICAD_CLI', '')
    if env and os.path.exists(env):
        return env
    for c in KICAD_CLI_CANDIDATES:
        if c and os.path.exists(c):
            return c
    # Versioned Windows installs (not on PATH; newest wins). Run 5 silently
    # skipped every oracle recheck on Windows because none of the unix
    # candidates exist there. The single hard-coded root missed a 32-bit
    # install, a per-user install, and any non-C: drive -- all of which fail
    # exactly like "no KiCad", i.e. silently, because a missing oracle is
    # indistinguishable from a clean one downstream.
    if sys.platform == 'win32':
        import glob
        roots = [os.environ.get('ProgramFiles', r'C:\Program Files'),
                 os.environ.get('ProgramFiles(x86)', r'C:\Program Files (x86)'),
                 os.environ.get('ProgramW6432', ''),
                 os.path.join(os.environ.get('LOCALAPPDATA', ''), 'Programs'),
                 r'C:\Program Files']
        hits = []
        for root in roots:
            if not root:
                continue
            hits += glob.glob(os.path.join(root, 'KiCad', '*', 'bin',
                                           'kicad-cli.exe'))
        if hits:
            # Newest KiCad by version-ish directory name, de-duplicated: the
            # roots overlap (ProgramFiles and ProgramW6432 are usually equal).
            def _ver(p):
                try:
                    return tuple(int(x) for x in
                                 os.path.basename(os.path.dirname(
                                     os.path.dirname(p))).split('.'))
                except ValueError:
                    return (0,)
            return sorted(set(hits), key=_ver)[-1]
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
        _ORACLE_TIMED_OUT.add(_fill_cost_key(board_file))
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
    comp_len: {root: total_mm}).

    The two label lists are ALWAYS parallel to segs/vias -- callers index
    them by enumerate() position (`comp_of_via[j]`), so a short list is an
    IndexError, not a graceful degradation. The no-component paths below
    return `None` per item ("component unknown"), which every caller already
    handles as "fall back to the single reported point". Returning bare []
    crashed the oracle on any net whose copper is vias-only (no segments) --
    usp_obc_v7's Net-(U5-GND), 2 vias + 2 pads, fed entirely by zone fill --
    taking down both exact_clusters() and _cluster_points()'s own via scan.
    """
    from check_connected import check_net_connectivity
    from geometry_utils import UnionFind
    from collections import defaultdict
    segs = [s for s in pcb_data.segments if s.net_id == net_id]
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    if not segs:
        return [None] * len(segs), [None] * len(vias), segs, vias, {}
    r = check_net_connectivity(net_id, segs, vias, [], [], return_graph=True)
    graph = r.get('graph')
    if not graph:
        return [None] * len(segs), [None] * len(vias), segs, vias, {}
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


def _pad_cluster_rung(pcb_data, net_id, x, y, layer, comps, tol):
    """Pad rung of _cluster_points (see its docstring for the motivation).

    Returns (root, pts): a comps root when the containing pad's copper
    reaches tracked copper (weld anywhere on that component), else
    (None, pad-group extent points) when the point sits on a bare pad
    group, else (None, []) when no pad contains the point.
    """
    from check_drc import point_to_pad_distance
    from check_connected import _pads_copper_touch
    comp_of_seg, comp_of_via, segs, vias, _ = comps

    def _pad_on_layer(p):
        if p.drill > 0 or '*.Cu' in p.layers:
            return True
        return layer is None or layer in p.layers

    pads = [p for p in pcb_data.pads_by_net.get(net_id, [])
            if p.pad_type != 'np_thru_hole' and _pad_on_layer(p)]
    hit = None
    for p in pads:
        if point_to_pad_distance(x, y, p) <= tol:
            hit = p
            break
    if hit is None:
        return None, []

    def _share_layer(a, b):
        if a.drill > 0 or b.drill > 0:
            return True
        la, lb = set(a.layers), set(b.layers)
        return bool(la & lb) or '*.Cu' in la or '*.Cu' in lb

    # Overlap group: the hit pad plus same-net pads its copper touches
    # (transitive, but these groups are tiny -- dual-pad footprints).
    # Cheap bounding-circle prefilter before the exact perimeter test.
    group, stack, seen = [hit], [hit], {id(hit)}
    while stack:
        base = stack.pop()
        b_r = math.hypot(base.size_x, base.size_y) / 2
        for p in pads:
            if id(p) in seen or not _share_layer(base, p):
                continue
            reach = b_r + math.hypot(p.size_x, p.size_y) / 2 + tol
            if math.hypot(p.global_x - base.global_x,
                          p.global_y - base.global_y) > reach:
                continue
            if _pads_copper_touch(base, p, tol):
                seen.add(id(p))
                group.append(p)
                stack.append(p)

    # Does the pad group reach TRACKED copper (a seg endpoint or via barrel
    # on/at the pad)? Then the pad is part of that component and any of its
    # points is a valid attachment.
    for p in group:
        p_r = math.hypot(p.size_x, p.size_y) / 2
        for i, s in enumerate(segs):
            if comp_of_seg[i] is None:
                continue
            if p.drill <= 0 and '*.Cu' not in p.layers \
                    and s.layer not in p.layers:
                continue
            reach = p_r + s.width / 2 + tol
            for ex, ey in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
                if abs(ex - p.global_x) <= reach \
                        and abs(ey - p.global_y) <= reach \
                        and point_to_pad_distance(ex, ey, p) \
                        <= s.width / 2 + tol:
                    return comp_of_seg[i], []
        for j, v in enumerate(vias):
            if comp_of_via[j] is None:
                continue
            if point_to_pad_distance(v.x, v.y, p) <= v.size / 2 + tol:
                return comp_of_via[j], []

    # Bare pad group (no tracked copper): its centers stand in for its
    # copper extent -- still better than the single reported nib.
    pts = []
    for p in group:
        if p.drill > 0 or '*.Cu' in p.layers:
            pts.append((p.global_x, p.global_y))  # spans all layers
        else:
            cu = [l for l in p.layers if l.endswith('.Cu')]
            pl = layer if (layer is not None and layer in p.layers) \
                else (cu[0] if cu else None)
            pts.append((p.global_x, p.global_y, pl) if pl
                       else (p.global_x, p.global_y))
    return None, pts


def _cluster_points(pcb_data, net_id, x, y, layer, comps, tol=0.06):
    """Expand a reported endpoint to its WHOLE copper cluster: any copper
    of the island is an equally good attachment, and the reported nib is
    often the worst (boxed into the very pocket that caused the gap).

    Pad-aware (2026-08-05, dilemma-class boards): a reported endpoint can
    land on a bare PAD with no track/via within tol -- dual-pad footprints
    and via-in-pad twins where KiCad's ratsnest anchors on the pad copper
    itself. The a0907dd exact_clusters pad-union fixed link GENERATION for
    these; this rung fixes weld ATTACHMENT quality: previously such a point
    fell through to the bare single-point fallback (root=None), so the weld
    attached at the worst nib instead of anywhere on the cluster. Pads are
    checked shape-accurately (check_drc.point_to_pad_distance <= tol, the
    same geometry check_connected._pads_copper_touch applies to degenerate
    points). A containing pad resolves to a comps component when it -- or
    an overlapping same-net sibling pad (_pads_copper_touch) -- reaches
    tracked copper; a bare pad group instead contributes its pads' centers
    as the copper extent (still root=None: pads carry no comps label).

    Falls back to the single point when nothing contains it (bare fill)."""
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
        try:
            root, _pad_pts = _pad_cluster_rung(
                pcb_data, net_id, x, y, layer, comps, tol)
        except Exception:
            root, _pad_pts = None, []
        if root is None and _pad_pts:
            return _pad_pts, None
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


def clamp_emitted_width(route_points, extra_conn, used_width, nominal_width,
                        pcb_data, net_id, config, ladder=(0.2, 0.4, 0.8)):
    """The widest width <= `used_width` at which the copper ABOUT TO BE WRITTEN
    (`route_points` plus the same-net weld connectors `extra_conn`) actually
    clears foreign copper, the board edge and NPTH drills. Floors at
    `nominal_width`.

    Issue #495/#496. The oracle's width upgrade validates the route the ladder
    produced, but that is not always the route that ships:

    * A DEGENERATE wide route (fewer than 2 points) has no same-layer legs, and
      wide_route_clear returns True vacuously for a leg-less route -- so the
      upgrade is accepted on an empty check. The caller then re-routes with
      track_margin=0 (geometry sized for the NOMINAL width) and that real path
      inherits the upgraded width. The exact-fill tier re-routes after the
      ladder for the same reason.
    * The weld connectors are synthesized after the ladder and were never
      clearance-checked at all.

    On rp2350_fpga_eensy that shipped 8 GND straps 0.026-0.361mm inside J1's
    0.65mm USB-C mounting hole, emitted at 0.8mm against a map stamped for
    0.0762. Stepping DOWN only ever declines the upgrade -- the nominal width is
    what the route was planned at -- so this can never lose a link that would
    otherwise have shipped.
    """
    if used_width <= nominal_width:
        return used_width
    w = used_width
    while w > nominal_width and not emitted_copper_clear(
            route_points, extra_conn, w, pcb_data, net_id, config):
        lower = [c for c in (nominal_width,) + tuple(ladder) if c < w]
        w = max(lower)
    return w


def emitted_copper_clear(route_points, extra_conn, width, pcb_data, net_id,
                         config):
    """True when the copper about to be written -- `route_points` at `width`
    PLUS the same-net weld connectors `extra_conn` -- clears foreign copper,
    the board edge and NPTH drills.

    Split out of clamp_emitted_width so the caller can also ask the question at
    the NOMINAL width, where there is no lower rung left to fall back to.
    """
    from plane_region_connector import wide_route_clear
    bec = getattr(config, 'board_edge_clearance', 0.0)
    if not wide_route_clear(route_points, width, pcb_data, net_id, config,
                            board_edge_clearance=bec):
        return False
    for (p1, p2, layer) in (extra_conn or []):
        if not wide_route_clear([(p1[0], p1[1], layer), (p2[0], p2[1], layer)],
                                width, pcb_data, net_id, config,
                                board_edge_clearance=bec):
            return False
    return True


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


def _delete_stranded_link_fragment(pcb_data, net_id, pt_a, pt_b):
    """When either link endpoint sits on a PAD-LESS copper cluster of the
    net (per the authoritative connectivity graph, vias/zone credit
    included), return (segments, vias) of that cluster for deletion; else
    None. A pad-less cluster is invisible to our pads-only "connected"
    verdict and protected as input copper by every cleanup, so KiCad
    demands a link to it forever -- deleting the dead fragment IS the
    resolution (no cluster, no ratsnest demand). Graphics-art components
    are never touched."""
    import math
    from check_connected import check_net_connectivity
    from geometry_utils import UnionFind
    segs = [s for s in pcb_data.segments if s.net_id == net_id]
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    pads = pcb_data.pads_by_net.get(net_id, [])
    zones = [z for z in (getattr(pcb_data, 'zones', None) or [])
             if z.net_id == net_id]
    if not segs or not pads:
        return None
    r = check_net_connectivity(net_id, segs, vias, pads, zones,
                               return_graph=True, pcb_data=pcb_data)
    g = r.get('graph')
    if not g:
        return None
    uf = UnionFind()
    for a, b in g.get('edges', []):
        uf.union(a, b)
    pad_roots = {uf.find(rep)
                 for rep in (g.get('pad_index_repr') or {}).values()}
    zone_roots = {uf.find(rep)
                  for rep in (g.get('zone_index_repr') or {}).values()}
    via_reprs = g.get('via_index_repr') or {}

    def _cluster_at(pt):
        px, py = pt
        best, bd = None, 0.35
        for i, s in enumerate(segs):
            dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
            L2 = dx * dx + dy * dy
            t = max(0.0, min(1.0, ((px - s.start_x) * dx
                                   + (py - s.start_y) * dy) / L2)) \
                if L2 else 0.0
            d = math.hypot(px - (s.start_x + t * dx),
                           py - (s.start_y + t * dy))
            if d < bd:
                best, bd = i, d
        return None if best is None else uf.find(2 * best)

    for pt in (pt_a, pt_b):
        root = _cluster_at(pt)
        if root is None or root in pad_roots or root in zone_roots:
            continue
        csegs = [s for i, s in enumerate(segs) if uf.find(2 * i) == root]
        if any(getattr(s, 'graphic', False) for s in csegs):
            continue
        cvias = [v for j, v in enumerate(vias)
                 if via_reprs.get(j) is not None
                 and uf.find(via_reprs[j]) == root]
        if csegs or cvias:
            return csegs, cvias
    return None


def _exact_fill_endpoints(pcb_data, net_id, net_name, A, B, exact_map,
                          track_half=0.1):
    """Strap endpoints from KiCad's EXACT fill (kicad_exact_fill): the two
    clusters' nearest approach, as (src_seeds, tgt_seeds, pa, pb, layer).

    The approximate island tracer fails on marginal pinches: it walks
    straight through the very gap KiCad's polygon math splits, seeds both
    sides as one island, and the weld bonds an island to itself. Exact
    filled_polygon islands make the split unambiguous: a same-point
    zone|zone link resolves to (its island) vs (every OTHER island + the
    main track cluster); a distinct-endpoint link resolves each end to its
    containing island when one exists. Returns None when exact geometry
    cannot separate two clusters (both ends in one island = the refill
    flicker class; no fill on this net; approach implausibly far), and the
    string 'cross-board' when the two clusters lie on DIFFERENT board
    outlines: on a multi-outline panel Edge.Cuts clips the fill per
    sub-board, the link is board-to-board and no copper can ever join it
    (g474's pad-less Earth pour, len42's per-board GND -- pad-less nets
    never enter the pad-based per-outline grading, so they surface here).
    Without the guard the A* burns its whole budget against the outline
    gap (546k iterations on g474)."""
    import math as _m
    import numpy as np
    from kicad_exact_fill import point_in_poly, sample_poly_edges
    from check_connected import point_in_polygon as _pip
    ax, ay, al, ak = A
    bx, by, bl, bk = B
    _outlines = pcb_data.board_info.board_outlines or []
    _multi = len(_outlines) >= 2

    def _outline_of(x, y):
        if not _multi:
            return None
        for _oi, _o in enumerate(_outlines):
            if _pip(x, y, _o):
                return _oi
        return None

    islands = []
    for (nn, layer), polys in exact_map.items():
        if nn != net_name:
            continue
        for p in polys:
            islands.append((layer, p))
    if not islands:
        return None

    samples_cache = {}

    def _samples(idx):
        if idx not in samples_cache:
            layer, poly = islands[idx]
            samples_cache[idx] = [(x, y, layer)
                                  for x, y in sample_poly_edges(poly)]
        return samples_cache[idx]

    def _containing(x, y, prefer):
        best = None
        for i, (layer, poly) in enumerate(islands):
            if point_in_poly(x, y, poly):
                if layer == prefer:
                    return i
                if best is None:
                    best = i
        if best is not None:
            return best
        # Edge quantization: the ratsnest anchor can sit ON the island
        # boundary; take the island whose edge passes within 1mm, nearest
        # first, preferring the anchor's layer.
        cand = []
        for i, (layer, poly) in enumerate(islands):
            d = min((_m.hypot(px - x, py - y)
                     for px, py, _ in _samples(i)), default=float('inf'))
            if d <= 1.0:
                cand.append((0 if layer == prefer else 1, d, i))
        return min(cand)[2] if cand else None

    same_pt = abs(ax - bx) < 1e-6 and abs(ay - by) < 1e-6
    ia = _containing(ax, ay, al)
    ib = None if same_pt else _containing(bx, by, bl)
    if ia is None and ib is None:
        return None
    if not same_pt and ia is not None and ia == ib:
        return None  # both ends inside ONE exact island: flicker class

    def _island_outline(idx):
        s = _samples(idx)
        return _outline_of(s[0][0], s[0][1]) if s else None

    # (x, y, layer, origin_island_or_None) point lists per side.
    if same_pt:
        if ia is None:
            return None
        a_pts = [(x, y, l, ia) for x, y, l in _samples(ia)]
        a_out = _island_outline(ia)
        b_pts = []
        _dropped_cross = 0
        for j in range(len(islands)):
            if j == ia:
                continue
            if _multi and _island_outline(j) != a_out:
                _dropped_cross += 1
                continue
            b_pts += [(x, y, l, j) for x, y, l in _samples(j)]
        for x, y, l in _largest_track_component_points(pcb_data, net_id):
            if _multi and _outline_of(x, y) != a_out:
                continue
            b_pts.append((x, y, l, None))
        if not b_pts:
            return 'cross-board' if _dropped_cross else None
    else:
        a_pts = [(x, y, l, ia) for x, y, l in _samples(ia)] \
            if ia is not None else \
            ([(ax, ay, al, None)] if al else [(ax, ay, None, None)])
        b_pts = [(x, y, l, ib) for x, y, l in _samples(ib)] \
            if ib is not None else \
            ([(bx, by, bl, None)] if bl else [(bx, by, None, None)])
        if _multi:
            a_out = _island_outline(ia) if ia is not None \
                else _outline_of(ax, ay)
            b_out = _island_outline(ib) if ib is not None \
                else _outline_of(bx, by)
            if a_out is not None and b_out is not None and a_out != b_out:
                return 'cross-board'
    if not a_pts or not b_pts:
        return None

    aa = np.asarray([(p[0], p[1]) for p in a_pts])
    bb = np.asarray([(p[0], p[1]) for p in b_pts])
    # Nearest approach via KD-tree, not an all-pairs distance matrix (#499).
    # The brute force materialised a (2000 x len(bb)) matrix per chunk: on
    # kbic65 that is aa=3.9k x bb=82k = 317M pairs, 25-90s PER CALL, and the
    # oracle calls it once per missing link (483 of them, mostly for the SAME
    # island pair) -- 99% of that board's 6876s chain. scipy is a declared
    # dependency (requirements.txt) and already used by plane_fill_model.
    #
    # Tie-breaks are resolved explicitly to keep the choice deterministic and
    # identical to the old row-major argmin: smallest aa index among the
    # minimum distances, then smallest bb index among ITS minima.
    from scipy.spatial import cKDTree
    _d, _j = cKDTree(bb).query(aa, k=1)
    _dmin = float(np.min(_d))
    i_best = int(np.flatnonzero(_d <= _dmin + 1e-12)[0])
    _dj = ((bb - aa[i_best]) ** 2).sum(axis=1)
    j_best = int(np.flatnonzero(_dj <= float(np.min(_dj)) + 1e-18)[0])
    pa, pb = a_pts[i_best], b_pts[j_best]
    if _m.hypot(pb[0] - pa[0], pb[1] - pa[1]) > 60.0:
        return None

    # Seed WINDOW: the pinch corridor, pa..pb expanded 8mm each way. The
    # geometric nearest approach can be walled (uhk: the carving /MOTION
    # track's crossing is only legal 4mm south of the minimum-distance
    # point) -- seeding the islands' full interiors within the window lets
    # the A* pick ANY crossing along the facing boundary, not just the
    # closest one. Interior cells (inset by the strap's half-width) because
    # fill-EDGE points are illegal starts: the edge sits exactly zone
    # clearance from the foreign copper that carved it.
    from kicad_exact_fill import rasterize_interior
    _win = (min(pa[0], pb[0]) - 8.0, min(pa[1], pb[1]) - 8.0,
            max(pa[0], pb[0]) + 8.0, max(pa[1], pb[1]) + 8.0)

    def _side_seeds(pt, pts):
        origin = pt[3]
        if origin is not None:
            layer, poly = islands[origin]
            cells = rasterize_interior(
                poly, _win, 0.25, inset=track_half + 0.05,
                edge_samples=_samples(origin))
            if cells:
                return [(x, y, layer) for x, y in cells][:1500]
            # Sliver island with no interior at track width: edge samples.
            return [(x, y, l) for x, y, l in _samples(origin)
                    if _win[0] <= x <= _win[2]
                    and _win[1] <= y <= _win[3]][:400]
        return [(px, py, pl) if pl else (px, py)
                for px, py, pl, _o in pts
                if _win[0] <= px <= _win[2]
                and _win[1] <= py <= _win[3]][:400]

    src = _side_seeds(pa, a_pts)
    tgt = _side_seeds(pb, b_pts)
    if not src or not tgt:
        return None
    layer = pa[2] if pa[2] else (al or bl)
    return src, tgt, (pa[0], pa[1]), (pb[0], pb[1]), layer


def _copper_in_corridor(segs, vias, x0, y0, x1, y1, pad: float = 3.0) -> bool:
    """#570 guard: True iff every emitted seg/via lies inside the link's
    corridor box (inflated by `pad` mm). An escalated weld whose copper lands
    outside the corridor did not weld THIS link -- it routed something else
    (the ecp5 false-claim class) and must not be credited."""
    bx0, bx1 = min(x0, x1) - pad, max(x0, x1) + pad
    by0, by1 = min(y0, y1) - pad, max(y0, y1) + pad
    for s in segs:
        if not (bx0 <= s.start_x <= bx1 and by0 <= s.start_y <= by1
                and bx0 <= s.end_x <= bx1 and by0 <= s.end_y <= by1):
            return False
    for v in vias:
        if not (bx0 <= v.x <= bx1 and by0 <= v.y <= by1):
            return False
    return True


def oracle_reconnect(board_file: str, net_names, config,
                     track_via_clearance: float,
                     hole_to_hole_clearance: float,
                     max_rounds: int = 3,
                     max_iterations: int = 1_000_000,
                     verbose: bool = False,
                     progress_callback=None,
                     cancel_check=None,
                     project_from: str = None) -> dict:
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
    if _fill_cost_key(board_file) in _ORACLE_TIMED_OUT:
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
    routed = failed = rounds = cross_board = 0
    remaining = -1
    links = None
    # True when we never obtained a trustworthy link list (source failure or
    # cancel): `remaining` may then be a STALE count from a previous round,
    # so remaining_links must report None ("unknown"), never [] ("none").
    _links_unavailable = False
    emitted_segments = []  # parser objects, for callers that apply results
    emitted_vias = []      # to a live board instead of reading the file
    # Stranded-fragment deletions (#508 findings 4+15): copper deleted from
    # pcb_data/the file that a live-board caller (planes_gui) must ALSO
    # delete -- the file strip has no pcbnew equivalent.
    removed_board_segments = []
    removed_board_vias = []
    attempted = {}  # (net, endpoints) -> attempt count (graduated retries)
    # #562 custody: links no copper can ever join (cross-board) stay in
    # `links` and keep counting in `remaining`, so a caller that treats
    # `remaining > 0` as "put these nets in custody" would retry a doomed
    # net on EVERY run -- with rip authority. Record their keys and filter
    # them out of remaining_links.
    exempt_keys = set()

    def _link_key(l):
        """The same (net, rounded endpoints) key `attempted` uses."""
        try:
            _n, (_ax, _ay, _al, _ak), (_bx, _by, _bl, _bk) = l
            return (_n, round(_ax, 2), round(_ay, 2),
                    round(_bx, 2), round(_by, 2))
        except Exception:
            return None
    for rnd in range(max_rounds):
        if cancel_check and cancel_check():
            print("  KiCad-oracle recheck: cancelled")
            _links_unavailable = True
            break
        if progress_callback:
            progress_callback(0, 0, f"KiCad-oracle: running kicad-cli DRC "
                                    f"(round {rnd + 1})...")
        links = None
        # DETERMINISTIC link source (#490): kicad-cli DRC's threaded
        # connectivity reported three different link sets on one unchanged
        # board (rp2040); welds chased a different report each round and
        # orangecrab's repair graded 103/65/92 across identical-input runs.
        # pcbnew's ZONE_FILLER is measured-deterministic -- exact_unconnected
        # clusters its fill truth reproducibly and anchors each link at the
        # true nearest approach. KICAD_LEGACY_ORACLE=1 restores kicad-cli.
        if not env_knobs.LEGACY_ORACLE:
            try:
                from kicad_exact_fill import exact_unconnected
                links = exact_unconnected(board_file, names,
                                          project_from=project_from)
                if links is not None and rnd == 0:
                    print("  KiCad-oracle recheck: deterministic exact-fill "
                          "link source (pcbnew refill)")
            except Exception as _xe:
                print(f"  (exact link source failed: {_xe}; falling back "
                      f"to kicad-cli)")
                links = None
        if links is None:
            links = kicad_unconnected(board_file, kicad_cli)
        if links is None:
            print("  KiCad-oracle recheck: kicad-cli DRC failed, skipping")
            _links_unavailable = True
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

        # EXACT-FILL cache (one pcbnew refill per round, fetched lazily only
        # when some link exhausts the approximate tiers below): KiCad's own
        # filled_polygon islands, for nearest-approach strapping.
        _exact_cache = {'fetched': False, 'islands': None}

        def _exact_islands_map():
            if not _exact_cache['fetched']:
                _exact_cache['fetched'] = True
                if not env_knobs.NO_EXACT_FILL:
                    try:
                        from kicad_exact_fill import refill_islands
                        print("  KiCad-oracle recheck: fetching exact fill "
                              "islands (pcbnew refill)...")
                        _exact_cache['islands'] = refill_islands(
                            board_file, verbose=verbose)
                        if _exact_cache['islands'] is not None:
                            _ni = sum(len(v) for v in
                                      _exact_cache['islands'].values())
                            print(f"    exact fill: {_ni} island(s) across "
                                  f"{len(_exact_cache['islands'])} "
                                  f"zone-layer(s)")
                    except Exception as _xe:
                        print(f"    (exact fill unavailable: {_xe})")
            return _exact_cache['islands']
        with open(board_file, 'r', encoding='utf-8') as f:
            content = f.read()
        v10 = is_kicad_10(content)
        new_sexprs = []
        content_dirty = False
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
        # Obstacle-map memo (#499). The base map is a pure function of
        # (net_id, the board's copper, config) and route_plane_connection_wide
        # CLONES it (`clone_fresh`) rather than mutating, so it is reusable
        # across links until copper actually lands. It was rebuilt PER LINK.
        # Measured on kbic65: each rebuild is 0.4-1.9s -- secondary to the
        # nearest-approach fix above, not the main cost -- but it is pure waste
        # on the links that add no copper (93% of that board's links have
        # identical endpoints). Keyed on the copper counts so any route that
        # DOES land invalidates it; cleared on miss so only one map is ever
        # alive (these maps are hundreds of MB).
        _obs_memo = {}

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
            if _attempt >= 99:
                continue  # cross-board exempt: accounted once, stay silent
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
            _obs_key = (net_id, len(pcb_data.segments), len(pcb_data.vias))
            base_obstacles = _obs_memo.get(_obs_key)
            if base_obstacles is None:
                _obs_memo.clear()   # keep exactly one map alive
                base_obstacles, _ = build_base_obstacles(
                    exclude_net_ids={net_id},
                    routing_layers=routing_layers,
                    pcb_data=pcb_data,
                    config=config,
                    track_width=config.track_width,
                    track_via_clearance=track_via_clearance,
                    hole_to_hole_clearance=hole_to_hole_clearance)
                _obs_memo[_obs_key] = base_obstacles
            net_vias = [(v.x, v.y) for v in pcb_data.vias
                        if v.net_id == net_id]
            # #479 reuse-audit gap 1: plated THT barrels are reusable layer
            # transitions for reconnect links exactly as for region joins --
            # route_plane_connection_wide's free-via registration and
            # is_at_via suppression only see what this list carries, so
            # without the barrels the oracle pays a fresh via beside one.
            from kicad_parser import pad_is_plated_through as _pipt
            for _p in pcb_data.pads_by_net.get(net_id, []):
                if _pipt(_p):
                    net_vias.append((_p.global_x, _p.global_y))
            island_fallback = False
            comps = _net_track_components(pcb_data, net_id)
            src, root_a = _cluster_points(pcb_data, net_id, ax, ay, al, comps)
            tgt, root_b = _cluster_points(pcb_data, net_id, bx, by, bl, comps)

            def _try_exact_tier():
                """EXACT-FILL TIER (last resort, reachable from BOTH failure
                paths -- the not-routable tail and the degenerate raw-retry):
                ask KiCad for its actual fill polygons (one refill per round,
                cached) and strap the true nearest approach between the two
                exact clusters. Fixes zero-length zone|zone pinch links (no
                geometry to route) and island traces that walk through the
                real split. Returns ('exempt'|'welded'|'route', payload) or
                (None, None); 'route' payload = (result, cfg, obstacles) for
                the shared emission path."""
                nonlocal cross_board, routed, progress

                def _plain_failed():
                    print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                          f"<->({bx:.2f},{by:.2f})  FAILED")
                    return None, None

                _ex_map = _exact_islands_map()
                if _ex_map is None:
                    return _plain_failed()
                try:
                    _ex = _exact_fill_endpoints(
                        pcb_data, net_id, net_name,
                        (ax, ay, al, akind), (bx, by, bl, bkind),
                        _ex_map, track_half=config.track_width / 2)
                except Exception as _xe2:
                    if verbose:
                        print(f"    (exact-fill tier error: {_xe2})")
                    return _plain_failed()
                if _ex == 'cross-board':
                    print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                          f"<->({bx:.2f},{by:.2f})  EXEMPT (clusters on "
                          f"different board outlines -- board-to-board "
                          f"link, no copper can join it)")
                    attempted[_key] = 99  # never retry
                    exempt_keys.add(_key)
                    cross_board += 1
                    return 'exempt', None
                if _ex is None:
                    return _plain_failed()
                _esrc, _etgt, _pa, _pb, _elayer = _ex
                print(f"    {net_name}: exact-fill tier: strapping "
                      f"nearest approach ({_pa[0]:.2f},{_pa[1]:.2f})<->"
                      f"({_pb[0]:.2f},{_pb[1]:.2f}) [{_elayer}]")
                # Via ladder like the main path: nominal, then the fab-floor
                # rung (a 0.71 via has nowhere to drop in a dense pocket).
                for _vs, _vd in ((config.via_size, config.via_drill),
                                 (0.45, 0.2)):
                    if _vs > config.via_size:
                        continue
                    _ecfg = config if _vs == config.via_size else \
                        replace(config, via_size=_vs, via_drill=_vd)
                    _eobst = base_obstacles
                    if _ecfg is not config:
                        _eobst, _ = build_base_obstacles(
                            exclude_net_ids={net_id},
                            routing_layers=routing_layers,
                            pcb_data=pcb_data,
                            config=_ecfg,
                            track_width=_ecfg.track_width,
                            track_via_clearance=track_via_clearance,
                            hole_to_hole_clearance=hole_to_hole_clearance)
                    _eres, _ = route_plane_connection_wide(
                        _esrc, _etgt,
                        plane_layer_idx=layer_map.get(_elayer, anchor_layer),
                        routing_layers=routing_layers,
                        base_obstacles=_eobst,
                        config=_ecfg,
                        net_vias=net_vias,
                        track_margin=0,
                        max_iterations=max_iterations,
                        verbose=verbose)
                    if _eres and (len(_eres[0]) >= 2 or _eres[1]):
                        return 'route', (_eres, _ecfg, _eobst)
                # Rescue-ladder the approach gap (scoped window, fine grid,
                # fab-floor escalation).
                _esc2 = None
                try:
                    from net_rescue import _attempt_edge
                    _gap2 = (math.hypot(_pb[0] - _pa[0], _pb[1] - _pa[1]),
                             _pa[0], _pa[1], _pb[0], _pb[1])
                    _esc2, _esc2_cfg = _attempt_edge(
                        pcb_data, net_id, _gap2, config, None,
                        strict_endpoints=True)
                except Exception:
                    _esc2 = None
                if _esc2 and not _esc2.get('failed') \
                        and not _copper_in_corridor(
                            _esc2.get('new_segments') or [],
                            _esc2.get('new_vias') or [],
                            _pa[0], _pa[1], _pb[0], _pb[1]):
                    print(f"    {net_name}: exact-fill strap REJECTED "
                          f"(emitted copper outside the link corridor -- "
                          f"#570 false-claim guard)")
                    _esc2 = None
                if _esc2 and not _esc2.get('failed'):
                    _e2segs = _esc2.get('new_segments') or []
                    _e2vias = _esc2.get('new_vias') or []
                    import clearance_ledger
                    clearance_ledger.record(_esc2_cfg.clearance)
                    for _s in _e2segs:
                        new_sexprs.append(generate_segment_sexpr(
                            (_s.start_x, _s.start_y), (_s.end_x, _s.end_y),
                            _s.width, _s.layer, net_id,
                            net_name if v10 else None))
                        pcb_data.segments.append(_s)
                        emitted_segments.append(_s)
                    for _v in _e2vias:
                        new_sexprs.append(generate_via_sexpr(
                            _v.x, _v.y, _v.size, _v.drill,
                            [routing_layers[0], routing_layers[-1]], net_id,
                            net_name=net_name if v10 else None))
                        pcb_data.vias.append(_v)
                        emitted_vias.append(_v)
                    print(f"    {net_name}: exact-fill strap OK (escalated: "
                          f"{len(_e2segs)} seg(s), {len(_e2vias)} via(s))")
                    routed += 1
                    progress = True
                    return 'welded', None
                print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                      f"<->({bx:.2f},{by:.2f})  FAILED "
                      f"(exact-fill strap unroutable)")
                return None, None

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
                    # +1.0 = the #268 stamp-shell quantization guard (see the
                    # region-join caller in plane_region_connector); #156 made
                    # _track_margin_for_width itself exact/fractional. The #505
                    # lattice snap was tried here and reverted -- see that
                    # function's docstring: on the plane ladders it cost trunk
                    # copper and changed no DRC.
                    margin = 1.0 + _track_margin_for_width(
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
                    # Seed cells are exemption-cleared (clone_fresh), so the
                    # margin cannot protect upgraded copper anchored on them
                    # -- verify the wide route's real geometry and refuse the
                    # upgrade on any conflict (orangecrab RAM-via overlaps,
                    # crkbd edge band). Narrow validated copper always wins.
                    from plane_region_connector import wide_route_clear
                    if not wide_route_clear(
                            wider[0], w, pcb_data, net_id, rung_cfg,
                            board_edge_clearance=rung_cfg.board_edge_clearance):
                        break
                    result, used_width = wider, w
            if not result:
                # ESCALATION (quickfeather U6-pocket class): the weld router
                # runs at the step's nominal parameters, and a sub-mm link
                # inside a dense escape field can be provably unroutable
                # there (0.15 track at 0.09 clearance cannot pass a 0.5mm
                # QFN pitch) while trivially routable at the rescue ladder's
                # fine grid / escalated fab floor. Reuse net_rescue's scoped
                # window machinery verbatim: bounded map, fenced A*, its own
                # rung ladder. Below-nominal clearance goes to the ledger so
                # check_drc grades at the true floor (#226).
                _esc = None
                try:
                    from net_rescue import _attempt_edge
                    _gap = (math.hypot(bx - ax, by - ay), ax, ay, bx, by)
                    _esc, _esc_cfg = _attempt_edge(
                        pcb_data, net_id, _gap, config, None,
                        strict_endpoints=True)
                except Exception as _ee:
                    if verbose:
                        print(f"    (weld escalation unavailable: {_ee})")
                    _esc = None
                if _esc and not _esc.get('failed') \
                        and not _copper_in_corridor(
                            _esc.get('new_segments') or [],
                            _esc.get('new_vias') or [], ax, ay, bx, by):
                    print(f"    {net_name}: weld escalation REJECTED "
                          f"(emitted copper outside the link corridor -- "
                          f"#570 false-claim guard)")
                    _esc = None
                if _esc and not _esc.get('failed'):
                    _esegs = _esc.get('new_segments') or []
                    _evias = _esc.get('new_vias') or []
                    import clearance_ledger
                    clearance_ledger.record(_esc_cfg.clearance)
                    for _s in _esegs:
                        new_sexprs.append(generate_segment_sexpr(
                            (_s.start_x, _s.start_y), (_s.end_x, _s.end_y),
                            _s.width, _s.layer, net_id,
                            net_name if v10 else None))
                        pcb_data.segments.append(_s)
                        emitted_segments.append(_s)
                    for _v in _evias:
                        new_sexprs.append(generate_via_sexpr(
                            _v.x, _v.y, _v.size, _v.drill,
                            [routing_layers[0], routing_layers[-1]], net_id,
                            net_name=net_name if v10 else None))
                        pcb_data.vias.append(_v)
                        emitted_vias.append(_v)
                    print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                          f"<->({bx:.2f},{by:.2f})  OK (escalated: grid "
                          f"{_esc_cfg.grid_step}, clearance "
                          f"{_esc_cfg.clearance:.4g}, track "
                          f"{_esc_cfg.track_width:.4g}; {len(_esegs)} "
                          f"seg(s), {len(_evias)} via(s))")
                    routed += 1
                    progress = True
                    continue
                # STRANDED-FRAGMENT DELETION (quickfeather XTAL_O class):
                # a link whose cluster is PAD-LESS copper (rip/reroute debris
                # the connectivity verdict never counts -- "connected" is
                # pads-only -- and cleanup never removes when it is input
                # copper) can never be graded broken by our checker, so no
                # router pass ever touches it, and KiCad demands the link
                # forever. Deleting the dead fragment resolves the link
                # exactly: no cluster, no ratsnest demand. Authoritative
                # graph decides pad-less-ness (vias/zone credit included);
                # graphics-art components are never touched.
                _deleted = _delete_stranded_link_fragment(
                    pcb_data, net_id, (ax, ay), (bx, by))
                if _deleted:
                    _dsegs, _dvias = _deleted
                    from kicad_writer import (remove_segments_from_content,
                                              remove_vias_from_content)
                    # #508 finding 4: the fragment may include copper EMITTED
                    # EARLIER THIS ROUND, whose sexprs are still pending in
                    # new_sexprs (spliced into content only at round end) --
                    # a text removal against `content` cannot see them, so
                    # the round-end splice would resurrect the deleted
                    # copper. Flush the pending emissions first so the
                    # removal below operates on the complete text.
                    if new_sexprs:
                        _idx = content.rfind(')')
                        # '\n'-join + trailing '\n' (#523): the sexprs start
                        # with '\t(' and end with '\t)', so a bare ''.join
                        # emitted ')\t(segment' joined lines and a '\t))'
                        # final line -- legal s-expr that KiCad reads, but it
                        # broke every line-based text walker downstream
                        # (filter_nets_from_content swallowed to EOF and ate
                        # the root paren).
                        content = (content[:_idx] + '\n'.join(new_sexprs)
                                   + '\n' + content[_idx:])
                        new_sexprs = []
                        content_dirty = True
                    content, _nrs = remove_segments_from_content(
                        content, _dsegs,
                        net_id_to_name={net_id: net_name} if v10 else None)
                    if _dvias:
                        content, _nrv = remove_vias_from_content(
                            content, _dvias,
                            net_id_to_name={net_id: net_name} if v10 else None)
                    _ds_ids = {id(x) for x in _dsegs}
                    _dv_ids = {id(x) for x in _dvias}
                    pcb_data.segments[:] = [s for s in pcb_data.segments
                                            if id(s) not in _ds_ids]
                    pcb_data.vias[:] = [v for v in pcb_data.vias
                                        if id(v) not in _dv_ids]
                    # Mirror the deletion into the RESULT emit lists too --
                    # the GUI applies new_segments/new_vias to the live
                    # board, and a deleted object left there ships copper
                    # pcb_data no longer has (#508 findings 4+15).
                    emitted_segments[:] = [s for s in emitted_segments
                                           if id(s) not in _ds_ids]
                    emitted_vias[:] = [v for v in emitted_vias
                                       if id(v) not in _dv_ids]
                    removed_board_segments.extend(_dsegs)
                    removed_board_vias.extend(_dvias)
                    content_dirty = True
                    print(f"    {net_name}: ({ax:.2f},{ay:.2f})"
                          f"<->({bx:.2f},{by:.2f})  RESOLVED (deleted "
                          f"stranded pad-less fragment: {len(_dsegs)} "
                          f"seg(s), {len(_dvias)} via(s))")
                    routed += 1
                    progress = True
                    continue
                _outcome, _payload = _try_exact_tier()
                if _outcome in ('exempt', 'welded'):
                    continue
                if _outcome == 'route':
                    result, rung_cfg, rung_obstacles = _payload
                    used_via_size = rung_cfg.via_size
                    used_via_drill = rung_cfg.via_drill
                    # fall through to the shared emission path below
                else:
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
                    # The degenerate path is how over-flooded island traces
                    # die (source and target expansion overlap) -- exactly
                    # the shape the exact-fill tier resolves. Try it before
                    # giving up; previously this exit bypassed the tier
                    # entirely (scalenode/corax56 never reached it).
                    _outcome, _payload = _try_exact_tier()
                    if _outcome in ('exempt', 'welded'):
                        continue
                    if _outcome == 'route':
                        result, rung_cfg, rung_obstacles = _payload
                        used_via_size = rung_cfg.via_size
                        used_via_drill = rung_cfg.via_drill
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
            # #479 reuse-audit gap 1: same-net PAD barrels are drill holes
            # too -- a new via within hole-to-hole of one is the same #282
            # class (KiCad's hole_to_hole is net-independent). Weld to the
            # barrel exactly like an existing via; its annular ring carries
            # the connectors on every layer.
            from types import SimpleNamespace as _SN
            from kicad_parser import pad_is_plated_through as _pipt2
            for _p in pcb_data.pads_by_net.get(net_id, []):
                if _pipt2(_p):
                    _own_vias.append(_SN(
                        x=(_p.hole_x if _p.hole_x is not None else _p.global_x),
                        y=(_p.hole_y if _p.hole_y is not None else _p.global_y),
                        drill=_p.drill))
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
            # #495/#496: the width ladder validates the route IT produced, which
            # is not always the route that ships (degenerate re-route, exact-fill
            # tier, unvalidated weld connectors). Re-check the exact copper about
            # to be written and step the width back down if it does not clear.
            used_width = clamp_emitted_width(
                route_points, _extra_conn, used_width, config.track_width,
                pcb_data, net_id, config)
            # Even at the nominal width the copper can graze: a `force_raw`
            # retry seeds the A* on KiCad's RAW reported point, and a
            # source/target cell is exemption-cleared in the obstacle map, so
            # the first segment out of that seed can sit inside the clearance
            # floor no matter how thin it is (rp2350: a GND link seeded 0.1414
            # from a +1V1 diagonal needing 0.1662, a 25um graze that survived
            # every width rung). Nothing thinner is left to fall back to, and
            # nudge_grazing_microshift rightly refuses to move the seed vertex
            # because that would detach the link from the island it must bond
            # to -- so DECLINE the link. KiCad already reports it unconnected;
            # shipping a short in its place trades a ratsnest line for a DRC
            # violation, which is the wrong way round (same call as #396's
            # rescue: decline cleanly rather than ship a false connection).
            if not emitted_copper_clear(route_points, _extra_conn, used_width,
                                        pcb_data, net_id, config):
                print(f"    {net_name}: ({ax:.2f},{ay:.2f})<->({bx:.2f},{by:.2f})"
                      f"  DECLINED (copper would violate clearance at "
                      f"{used_width:.4g}mm; left unconnected)")
                failed += 1
                continue
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

        if new_sexprs or content_dirty:
            if new_sexprs:
                idx = content.rfind(')')
                # '\n'-join + trailing '\n' (#523), same as the flush above.
                content = (content[:idx] + '\n'.join(new_sexprs) + '\n'
                           + content[idx:])
            with open(board_file, 'w', encoding='utf-8') as f:
                f.write(content)
        if not progress:
            break
    else:
        # ran all rounds; get the final count (same source as the rounds)
        links = None
        if not env_knobs.LEGACY_ORACLE:
            try:
                from kicad_exact_fill import exact_unconnected
                links = exact_unconnected(board_file, names,
                                          project_from=project_from)
            except Exception:
                links = None
        if links is None:
            links = kicad_unconnected(board_file, kicad_cli)
        if links is not None:
            remaining = len([l for l in links if l[0] in names])

    # DEBRIS PASS (quickfeather XTAL_O class): remaining links on ANY net --
    # including nets outside this pass's scope -- whose cluster is pad-less
    # stranded copper. Deletion only, never routing (foreign nets belong to
    # their own steps; deleting provably-dead copper is net-safe): the
    # fragment is invisible to the pads-only "connected" verdict, protected
    # as input copper by every cleanup, so nothing else will EVER touch it
    # and KiCad demands the link on every future run.
    _debris_resolved = set()
    if rounds and links:
        _stranded_deleted = 0
        _content2 = None
        for lk in links:
            _lnet = lk[0]
            _lax, _lay = lk[1][0], lk[1][1]
            _lbx, _lby = lk[2][0], lk[2][1]
            _lnid = name_to_id.get(_lnet)
            if _lnid is None:
                continue
            _del = _delete_stranded_link_fragment(
                pcb_data, _lnid, (_lax, _lay), (_lbx, _lby))
            if not _del:
                continue
            _dsegs, _dvias = _del
            from kicad_writer import (remove_segments_from_content,
                                      remove_vias_from_content)
            if _content2 is None:
                with open(board_file, 'r', encoding='utf-8') as f:
                    _content2 = f.read()
            _nm_map = {_lnid: _lnet} if v10 else None
            _content2, _n1 = remove_segments_from_content(
                _content2, _dsegs, net_id_to_name=_nm_map)
            if _dvias:
                _content2, _n2 = remove_vias_from_content(
                    _content2, _dvias, net_id_to_name=_nm_map)
            _ds = {id(x) for x in _dsegs}
            _dv = {id(x) for x in _dvias}
            pcb_data.segments[:] = [s for s in pcb_data.segments
                                    if id(s) not in _ds]
            pcb_data.vias[:] = [v for v in pcb_data.vias if id(v) not in _dv]
            # #508 finding 15: a deleted fragment can include copper this
            # oracle run emitted in an earlier round -- the GUI applies the
            # returned new_segments/new_vias to the live board, so leaving
            # the deleted objects in the emit lists ships copper pcb_data
            # (and the file, stripped above) no longer has.
            emitted_segments[:] = [s for s in emitted_segments
                                   if id(s) not in _ds]
            emitted_vias[:] = [v for v in emitted_vias if id(v) not in _dv]
            removed_board_segments.extend(_dsegs)
            removed_board_vias.extend(_dvias)
            _stranded_deleted += 1
            print(f"    {_lnet}: link resolved by deleting stranded "
                  f"pad-less fragment ({len(_dsegs)} seg(s), "
                  f"{len(_dvias)} via(s))")
            if _lnet in names:
                remaining = max(0, remaining - 1)
                _debris_resolved.add(id(lk))
        if _content2 is not None:
            with open(board_file, 'w', encoding='utf-8') as f:
                f.write(_content2)

    if rounds and remaining > 0:
        _xb = f" ({cross_board} cross-board exempt)" if cross_board else ""
        print(f"  KiCad-oracle recheck: {remaining} link(s) still "
              f"unconnected per KiCad after {rounds} round(s){_xb}")
    return {'available': True, 'rounds': rounds, 'links_routed': routed,
            'links_failed': failed, 'remaining': remaining,
            # #562 order swap: the FINAL flagged links with net names, so the
            # caller can scope custody to exactly the stubborn nets and keep
            # the reconcile's hands off KiCad-verified-complete plane nets
            # (ux pf7: a stale failure bucket re-touched gate-connected +3V3
            # and broke it).
            'remaining_links': (None if _links_unavailable else
                                [l for l in (links or [])
                                 if l[0] in names
                                 and id(l) not in _debris_resolved
                                 and _link_key(l) not in exempt_keys]),
            'cross_board': cross_board,
            'new_segments': emitted_segments, 'new_vias': emitted_vias,
            # #508 finding 15: stranded-fragment copper deleted from the
            # file/pcb_data; a live-board caller must delete it too.
            'removed_segments': removed_board_segments,
            'removed_vias': removed_board_vias}
