#!/usr/bin/env python3
"""Read-only checker for "weird" copper hygiene issues that are neither DRC
violations nor opens: dead-end antennas, soft joints, redundant loops,
individually-removable copper, stacked duplicates, and floating vias.

Categories:
  dangling-end       degree-1 segment endpoint not anchored on a same-net pad,
                     via, T-junction, or same-net zone outline (reuses
                     pcb_modification._point_anchored). Includes the
                     half-segment variant: a tail dangling PAST a mid-body
                     anchor (trim_dangles_past_body_anchor geometry,
                     report-only). Reports the dangling length.
  soft-joint         same-net endpoints whose caps overlap but are not
                     coincident (check_drc's 'segment-endpoint-gap' class;
                     reuses routing_constants.SOFT_JOINT_MIN_GAP and approach).
  redundant-cycle    same-net loop edges whose removal leaves connectivity
                     identical (pcb_modification._prune_net_cycles machinery,
                     report-only; zoned nets skipped -- planes are meshes).
  removable-segment  any segment whose individual removal keeps the net's
                     (num_components, disconnected-pad count) unchanged
                     (check_connected.analyze_conn_excluding). Superset of
                     redundant-cycle. Nets with >500 segments are skipped
                     unless --thorough; zoned and <2-pad nets are skipped
                     (their connectivity result is trivially insensitive).
  stacked-copper     exactly-duplicate segments (same endpoints/layer/net
                     within ~1um) and coincident same-net vias (centers
                     within 0.01mm) -- the duplicate-emission bug class.
  unsupported-via    a via with no same-net track copper reaching its barrel,
                     not inside a same-net pad, and not inside a same-net
                     zone polygon (a floating via).
  orphan-island      a connected group of same-net track copper (segments,
                     possibly with vias) that reaches NO pad of the net --
                     dead copper stranded by a rip or a superseded route.
                     Zone-connected copper counts as connected (the island
                     unions with everything in its zone outline). Size = the
                     island's total copper length.

This script NEVER modifies the board (read-only; nothing is written back).
Net 0 (unconnected) copper is skipped -- "same-net" semantics do not apply.

Usage:
    python3 check_weird.py board.kicad_pcb [--nets PATTERN ...] [--thorough]
                                           [--max-print N]

Exit code 0 when clean, 1 when any findings.
"""
import argparse
import math
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

from kicad_parser import parse_kicad_pcb, PCBData
from check_connected import (matches_any_pattern, check_net_connectivity,
                             analyze_conn_excluding, point_in_polygon,
                             _point_in_pad)
from check_drc import point_to_pad_distance
from connectivity import COINCIDENCE_TOL
from routing_constants import SOFT_JOINT_MIN_GAP
from pcb_modification import _point_anchored, _prune_net_cycles, _pt_seg_dist

CATEGORIES = ['dangling-end', 'soft-joint', 'redundant-cycle',
              'removable-segment', 'stacked-copper', 'unsupported-via',
              'orphan-island']
# Cost cap for the per-segment removable scan (spec: skip unless --thorough).
MAX_SEGS_PER_NET = 500
_CELL = 1.0  # spatial-grid cell (mm) fed to _point_anchored, as in the pruner
_VIA_COINCIDENT_MM = 0.01  # stacked-via center distance
_DUP_SEG_DECIMALS = 3      # ~1um endpoint quantization for exact duplicates


def _finding(category, net, layer, x, y, detail, size=None):
    # size = the finding's characteristic magnitude in mm (dangle length,
    # gap, duplicated-copper length, via diameter ...). The --tolerance
    # filter drops findings smaller than the threshold; None = always report.
    return {'category': category, 'net': net, 'layer': layer,
            'x': x, 'y': y, 'detail': detail, 'size': size}


def _net_name(pcb_data: PCBData, net_id: int) -> str:
    net = pcb_data.nets.get(net_id)
    return net.name if net else f"net_{net_id}"


def _via_span(via, copper_layers) -> set:
    """Copper layers a via connects (through vias span everything).

    Blind/buried vias record only their START/END layers in the file; the
    barrel also connects every copper layer BETWEEN them in stackup order
    (a buried F.Cu-In2.Cu via touches In1.Cu) -- treating the span as the
    two endpoints alone manufactured phantom unsupported-via findings for
    mid-span connections."""
    if via.layers and not ('F.Cu' in via.layers and 'B.Cu' in via.layers):
        order = list(copper_layers)
        idx = [order.index(l) for l in via.layers if l in order]
        if len(idx) >= 2:
            lo, hi = min(idx), max(idx)
            return set(order[lo:hi + 1])
        return set(via.layers)
    return set(copper_layers)


def _check_soft_joints(net_id, name, net_segs, net_vias, net_pads, findings):
    """check_drc's 'segment-endpoint-gap' detection (same constants/approach):
    dangling free ends (degree 1, not on a via/own pad) whose caps overlap
    but whose endpoints are not coincident. Returns the set of participating
    endpoint keys (layer, rx, ry) so the dangle check can defer to this,
    more specific, category."""
    via_r = [(v.x, v.y, (getattr(v, 'size', 0) or 0) / 2.0) for v in net_vias]

    def at_anchor(x, y):
        for vx, vy, vr in via_r:
            if math.hypot(x - vx, y - vy) <= vr + 0.01:
                return True
        for p in net_pads:
            if point_to_pad_distance(x, y, p) <= COINCIDENCE_TOL:
                return True
        return False

    def rk(x, y):
        return (round(x, 3), round(y, 3))

    ep_count = defaultdict(int)
    for s in net_segs:
        ep_count[(s.layer, rk(s.start_x, s.start_y))] += 1
        ep_count[(s.layer, rk(s.end_x, s.end_y))] += 1

    dangles = defaultdict(list)  # layer -> [(x, y, width)]
    for s in net_segs:
        for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if ep_count[(s.layer, rk(x, y))] != 1:
                continue  # shared vertex = clean joint
            if at_anchor(x, y):
                continue  # terminates on a via / own pad = legitimate
            dangles[s.layer].append((x, y, s.width))

    soft_pts = set()
    for layer, ends in dangles.items():
        for i in range(len(ends)):
            xi, yi, wi = ends[i]
            for j in range(i + 1, len(ends)):
                xj, yj, wj = ends[j]
                gap = math.hypot(xi - xj, yi - yj)
                cap = (wi + wj) / 2.0
                if SOFT_JOINT_MIN_GAP < gap < cap - 1e-6:
                    # size=None: soft joints bypass the --tolerance filter.
                    # Filtering by GAP inverted the severity metric (small
                    # gap = still fragile) and on <=0.1mm-width routing every
                    # representable soft joint has gap < 0.1 -- the whole
                    # category silently vanished at the default tolerance.
                    findings.append(_finding(
                        'soft-joint', name, layer, xi, yi,
                        f"endpoint gap {gap:.3f}mm to ({xj:.3f}, {yj:.3f}), "
                        f"caps overlap {cap - gap:.3f}mm (fragile near-open)"))
                    k1 = (layer,) + rk(xi, yi)
                    k2 = (layer,) + rk(xj, yj)
                    soft_pts.add(k1)
                    soft_pts.add(k2)
    return soft_pts


def _check_dangles(net_id, name, net_segs, net_vias, net_pads, net_zones,
                   soft_pts, findings, join_tol: float = 0.0):
    """Degree-1 endpoints that _point_anchored calls unanchored and that are
    not inside a same-net zone outline. Half-segment tails past a mid-body
    anchor reuse trim_dangles_past_body_anchor's geometry (report-only)."""
    tol = COINCIDENCE_TOL
    track_segs = [s for s in net_segs if not getattr(s, 'graphic', False)]
    if not track_segs:
        return
    via_pts = [(v.x, v.y, getattr(v, 'size', 0.6) or 0.6) for v in net_vias]
    pad_pts = []
    for p in net_pads:
        px = getattr(p, 'global_x', getattr(p, 'x', 0.0))
        py = getattr(p, 'global_y', getattr(p, 'y', 0.0))
        psize = max(getattr(p, 'size_x', 0.5), getattr(p, 'size_y', 0.5))
        pad_pts.append((px, py, psize, getattr(p, 'layers', [])))

    def key(x, y, layer):
        return (round(x, 3), round(y, 3), layer)

    # Graphics copper counts as an anchor universe member but is never a
    # candidate (immutable input art, #337).
    degree = defaultdict(int)
    seg_index = defaultdict(list)
    for s in net_segs:
        degree[key(s.start_x, s.start_y, s.layer)] += 1
        degree[key(s.end_x, s.end_y, s.layer)] += 1
        lo_x = int(min(s.start_x, s.end_x) // _CELL)
        hi_x = int(max(s.start_x, s.end_x) // _CELL)
        lo_y = int(min(s.start_y, s.end_y) // _CELL)
        hi_y = int(max(s.start_y, s.end_y) // _CELL)
        for cx in range(lo_x, hi_x + 1):
            for cy in range(lo_y, hi_y + 1):
                seg_index[(s.layer, cx, cy)].append(s)

    zones_by_layer = defaultdict(list)
    for z in net_zones:
        zones_by_layer[z.layer].append(z)

    for s in track_segs:
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        L2 = dx * dx + dy * dy
        free_ends = []
        for free_is_start in (True, False):
            fx, fy = (s.start_x, s.start_y) if free_is_start else (s.end_x, s.end_y)
            if degree[key(fx, fy, s.layer)] != 1:
                continue
            if ((s.layer, round(fx, 3), round(fy, 3)) in soft_pts):
                continue  # already reported as the more specific soft-joint
            if _point_anchored(fx, fy, s.layer, via_pts, pad_pts,
                               seg_index, _CELL, s, tol):
                continue
            # _point_anchored's pad test is RADIAL (center distance vs the
            # max half-dimension) and misses rectangular pad corners: a stub
            # ending exactly on a 1.4x1.2 crystal pad's corner copper read
            # as a dangle. Exact outline test (glasgow XTALOUT/C11).
            def _pad_carries(p):
                # NPTH pads have no copper even when layers lists *.Cu; an
                # other-layer SMD pad can't anchor this layer's free end.
                if getattr(p, 'pad_type', '') == 'np_thru_hole':
                    return False
                if p.drill <= 0 and s.layer not in p.layers \
                        and '*.Cu' not in p.layers:
                    return False
                return point_to_pad_distance(fx, fy, p) <= s.width / 2
            if any(_pad_carries(p) for p in net_pads):
                continue
            if any(point_in_polygon(fx, fy, z.polygon)
                   for z in zones_by_layer.get(s.layer, ())):
                continue  # lands in a same-net zone fill outline
            # Two long tracks whose ends miss each other by a few um (a
            # nudge/micro-shift split pair) are OFF BY that microgap, not by
            # their segment lengths: within join_tol they are connected
            # copper, not dangles.
            if join_tol > 0:
                joined = False
                cx0, cy0 = int(fx // _CELL), int(fy // _CELL)
                for ncx in (cx0 - 1, cx0, cx0 + 1):
                    for ncy in (cy0 - 1, cy0, cy0 + 1):
                        for o in seg_index.get((s.layer, ncx, ncy), ()):
                            if o is s:
                                continue
                            _cap = (s.width + o.width) / 2 - 1e-6
                            _jt = min(join_tol, _cap)
                            if (math.hypot(o.start_x - fx, o.start_y - fy) <= _jt
                                    or math.hypot(o.end_x - fx, o.end_y - fy) <= _jt):
                                # Width-aware: two fine tracks 0.09mm apart
                                # do NOT overlap caps (real open) and stay
                                # flagged; a flat 0.1 gate hid them.
                                joined = True
                                break
                        if joined:
                            break
                    if joined:
                        break
                if joined:
                    continue
            free_ends.append((free_is_start, fx, fy))
        if not free_ends:
            continue
        seg_len = math.sqrt(L2)
        if len(free_ends) == 2:
            _, fx, fy = free_ends[0]
            _, ox, oy = free_ends[1]
            findings.append(_finding(
                'dangling-end', name, s.layer, fx, fy,
                f"isolated fragment {seg_len:.3f}mm long, "
                f"other end at ({ox:.3f}, {oy:.3f})", size=seg_len))
            continue
        free_is_start, fx, fy = free_ends[0]
        # Mid-body anchors (trim_dangles_past_body_anchor geometry): a same-net
        # via barrel overlapping the centerline, or another same-net segment
        # endpoint teeing into the body.
        cands = []
        if L2 >= 1e-9:
            for vx, vy, vsize in via_pts:
                t = ((vx - s.start_x) * dx + (vy - s.start_y) * dy) / L2
                if t <= 0.02 or t >= 0.98:
                    continue
                cx_, cy_ = s.start_x + t * dx, s.start_y + t * dy
                if math.hypot(vx - cx_, vy - cy_) < (vsize + s.width) / 2 - 1e-6:
                    cands.append(t)
            for o in net_segs:
                if o is s or o.layer != s.layer:
                    continue
                for ox, oy in ((o.start_x, o.start_y), (o.end_x, o.end_y)):
                    t = ((ox - s.start_x) * dx + (oy - s.start_y) * dy) / L2
                    if t <= 0.02 or t >= 0.98:
                        continue
                    cx_, cy_ = s.start_x + t * dx, s.start_y + t * dy
                    if math.hypot(ox - cx_, oy - cy_) < (o.width + s.width) / 2 - 1e-6:
                        cands.append(t)
        if cands:
            t_anchor = min(cands) if free_is_start else max(cands)
            nx, ny = s.start_x + t_anchor * dx, s.start_y + t_anchor * dy
            tail = math.hypot(fx - nx, fy - ny)
            if tail <= max(tol, 3 * s.width):
                continue  # sub-visible nib past the anchor (as the trim pass)
            findings.append(_finding(
                'dangling-end', name, s.layer, fx, fy,
                f"half-segment tail dangling {tail:.3f}mm past body anchor "
                f"at ({nx:.3f}, {ny:.3f})", size=tail))
        else:
            findings.append(_finding(
                'dangling-end', name, s.layer, fx, fy,
                f"free end, dangling segment {seg_len:.3f}mm "
                f"(rooted at ({s.end_x if free_is_start else s.start_x:.3f}, "
                f"{s.end_y if free_is_start else s.start_y:.3f}))",
                size=seg_len))


def _check_cycles(net_id, name, net_segs, net_vias, net_pads, has_zone,
                  findings):
    """Report-only spanning-tree reduction (prune_redundant_cycles machinery).
    _prune_net_cycles internally validates every proposed removal against
    check_net_connectivity, so reported edges are guaranteed redundant."""
    if has_zone:
        return  # planes / pours are meshes, not trees (as the pruner)
    track_segs = [s for s in net_segs if not getattr(s, 'graphic', False)]
    if len(track_segs) < 3:
        return
    empty_fgrid = defaultdict(list)  # no grazing preference needed for a report
    _, removed = _prune_net_cycles(net_id, track_segs, net_vias, net_pads,
                                   empty_fgrid, _CELL, 0.0, 0.1)
    for s in removed:
        mx, my = (s.start_x + s.end_x) / 2.0, (s.start_y + s.end_y) / 2.0
        findings.append(_finding(
            'redundant-cycle', name, s.layer, mx, my,
            f"loop edge ({s.start_x:.3f}, {s.start_y:.3f})-"
            f"({s.end_x:.3f}, {s.end_y:.3f}); removal leaves connectivity "
            f"identical",
            size=math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)))


def _check_removable(net_id, name, net_segs, net_vias, net_pads, net_zones,
                     has_zone, thorough, findings, skipped_nets):
    """Segments whose individual exclusion keeps (num_components,
    disconnected-pad count) unchanged, via analyze_conn_excluding on one
    prebuilt graph (O(net + candidates), not O(net x candidates))."""
    if has_zone:
        return  # the zone-outline model over-credits connectivity on planes
    if len(net_pads) < 2:
        return  # a <2-pad net's connectivity result is trivially insensitive
    track_idx = [i for i, s in enumerate(net_segs)
                 if not getattr(s, 'graphic', False)]
    if not track_idx:
        return
    if len(net_segs) > MAX_SEGS_PER_NET and not thorough:
        skipped_nets.append((name, len(net_segs)))
        return
    # STRICT width-clamped graph (#322): the physical overlap model credits a
    # one-grid-cell jog as removable because its neighbours' end caps overlap
    # across the 0.07mm gap -- but removing it would SHIP that fragile
    # cap-overlap joint (the class close_soft_joints exists to bridge). Grade
    # removability on width-clamped twins so only copper whose removal leaves
    # a genuinely coincident path counts (median 'removable' length on the
    # 0708b sweep was exactly one 0.05-grid diagonal -- load-bearing jogs).
    import copy as _copy
    from connectivity import COINCIDENCE_TOL as _STRICT_W
    clamped = []
    for s in net_segs:
        c = _copy.copy(s)
        c.width = min(c.width, _STRICT_W)
        clamped.append(c)
    # Clamp via SIZES too: the checker's via->pad and via->endpoint credits
    # scale with the via radius (barrel-overlap semantics, KiCad-true for
    # GRADING), but the strict graph must keep its tight coincidence gate --
    # an off-centre via-in-pad grazing the pad outline is a connection KiCad
    # accepts, not one a removal pass may lean on (0708d lesson).
    clamped_vias = []
    for v in net_vias:
        cv = _copy.copy(v)
        cv.size = min(cv.size, _STRICT_W)
        clamped_vias.append(cv)
    r = check_net_connectivity(net_id, clamped, clamped_vias, net_pads,
                               net_zones, return_graph=True)
    graph = r.get('graph')
    if not graph or not graph['pad_ids']:
        return
    base = analyze_conn_excluding(graph, ())
    base_key = (base['num_components'], len(base['disconnected_pads']))
    if base['num_components'] != 1 or base['disconnected_pads']:
        # A strictly-split net (mid-path soft joint or a real open) makes
        # 'key unchanged' meaningless: load-bearing copper on the broken
        # side would grade as removable. Mirror the mutating twin
        # (collapse_strict_redundant) and skip the net.
        return
    base_copper = base.get('num_copper_components', 1)
    for i in track_idx:
        t = analyze_conn_excluding(graph, (i,))
        if (t['num_components'], len(t['disconnected_pads'])) == base_key \
                and t.get('num_copper_components', 1) <= base_copper:
            s = net_segs[i]
            mx, my = (s.start_x + s.end_x) / 2.0, (s.start_y + s.end_y) / 2.0
            findings.append(_finding(
                'removable-segment', name, s.layer, mx, my,
                f"segment ({s.start_x:.3f}, {s.start_y:.3f})-"
                f"({s.end_x:.3f}, {s.end_y:.3f}) w{s.width:.3f}: removal "
                f"does not change net connectivity",
                size=math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)))


def _check_stacked(net_id, name, net_segs, net_vias, findings):
    """Exactly-duplicate segments (~1um) and coincident same-net vias."""
    groups = defaultdict(list)
    for s in net_segs:
        a = (round(s.start_x, _DUP_SEG_DECIMALS), round(s.start_y, _DUP_SEG_DECIMALS))
        b = (round(s.end_x, _DUP_SEG_DECIMALS), round(s.end_y, _DUP_SEG_DECIMALS))
        lo, hi = (a, b) if a <= b else (b, a)
        groups[(s.layer, lo, hi)].append(s)
    for (layer, lo, hi), ss in groups.items():
        if len(ss) > 1:
            findings.append(_finding(
                'stacked-copper', name, layer, lo[0], lo[1],
                f"{len(ss)} duplicate segments stacked on "
                f"({lo[0]:.3f}, {lo[1]:.3f})-({hi[0]:.3f}, {hi[1]:.3f})",
                size=math.hypot(hi[0] - lo[0], hi[1] - lo[1])))
    # Coincident vias: bucket at the coincidence radius, scan 3x3 neighbors.
    cell = _VIA_COINCIDENT_MM
    grid = defaultdict(list)
    for idx, v in enumerate(net_vias):
        grid[(int(math.floor(v.x / cell)), int(math.floor(v.y / cell)))].append(idx)
    reported = set()
    for i, v in enumerate(net_vias):
        cx = int(math.floor(v.x / cell))
        cy = int(math.floor(v.y / cell))
        for gx in (cx - 1, cx, cx + 1):
            for gy in (cy - 1, cy, cy + 1):
                for j in grid.get((gx, gy), ()):
                    if j <= i:
                        continue
                    o = net_vias[j]
                    d = math.hypot(v.x - o.x, v.y - o.y)
                    if d <= _VIA_COINCIDENT_MM + 1e-9 and (i, j) not in reported:
                        reported.add((i, j))
                        layer_str = ','.join(v.layers) if v.layers else '*.Cu'
                        findings.append(_finding(
                            'stacked-copper', name, layer_str, v.x, v.y,
                            f"coincident vias {d * 1000:.1f}um apart "
                            f"(other at ({o.x:.4f}, {o.y:.4f}))",
                            size=getattr(v, 'size', None)))


def _check_unsupported_vias(net_id, name, net_segs, net_vias, net_pads,
                            net_zones, copper_layers, findings):
    """Floating vias: no same-net track copper reaching the barrel, no
    same-net pad containing the center, no same-net zone polygon around it."""
    for v in net_vias:
        span = _via_span(v, copper_layers)
        r = (getattr(v, 'size', 0.6) or 0.6) / 2.0
        supported = False
        for s in net_segs:
            if s.layer not in span:
                continue
            if _pt_seg_dist(v.x, v.y, s.start_x, s.start_y,
                            s.end_x, s.end_y) < r + s.width / 2 - 1e-6:
                supported = True
                break
        if not supported:
            for p in net_pads:
                if getattr(p, 'pad_type', '') == 'np_thru_hole':
                    continue  # NPTH pads have no copper
                if p.drill and p.drill > 0:
                    on_layer = True  # plated barrel spans all copper layers
                else:
                    pl = set(p.layers or [])
                    on_layer = bool(span & pl) or any('*' in L for L in pl)
                if on_layer and _point_in_pad(v.x, v.y, p, margin=COINCIDENCE_TOL):
                    supported = True
                    break
        if not supported:
            for z in net_zones:
                if z.layer in span and point_in_polygon(v.x, v.y, z.polygon):
                    supported = True
                    break
        if not supported:
            layer_str = ','.join(v.layers) if v.layers else '*.Cu'
            findings.append(_finding(
                'unsupported-via', name, layer_str, v.x, v.y,
                "floating via: no same-net track, pad, or zone reaches it",
                size=getattr(v, 'size', None)))


def _check_orphan_islands(net_id, name, net_segs, net_vias, net_pads,
                          net_zones, findings):
    """Connected components of track copper that reach no pad of the net.
    Built on check_net_connectivity's own graph, so vias, T-junctions, cap
    overlaps, and zone-outline membership all count as connections -- an
    island flagged here is one the AUTHORITATIVE model calls pad-less. A
    via-only island is left to unsupported-via."""
    track_segs = [s for s in net_segs if not getattr(s, 'graphic', False)]
    if not track_segs or not net_pads:
        return
    from geometry_utils import UnionFind
    r = check_net_connectivity(net_id, net_segs, net_vias, net_pads,
                               net_zones, return_graph=True)
    graph = r.get('graph')
    if not graph:
        return
    uf = UnionFind()
    for a, b in graph.get('edges', []):
        uf.union(a, b)
    pad_roots = {uf.find(rep) for rep in graph.get('pad_index_repr', {}).values()}
    islands = defaultdict(list)  # root -> [segment, ...]
    for i, s in enumerate(net_segs):
        if getattr(s, 'graphic', False):
            continue
        islands[uf.find(2 * i)].append(s)
    for root, segs in islands.items():
        if root in pad_roots:
            continue
        total = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                    for s in segs)
        cx = sum((s.start_x + s.end_x) / 2 for s in segs) / len(segs)
        cy = sum((s.start_y + s.end_y) / 2 for s in segs) / len(segs)
        findings.append(_finding(
            'orphan-island', name, segs[0].layer, cx, cy,
            f"{len(segs)} segment(s), {total:.2f}mm of copper connected to "
            f"NO pad of the net", size=total))


def _check_terminal_web(pcb_data, net_id, name, net_segs, net_pads, floor,
                        findings):
    """Flag degree-1 terminal endpoints whose cap overlaps a same-net pad only
    near a CORNER, joining through a copper web thinner than the min-track floor
    (issue #416). DRC-clean and connected, but a manufacturability hazard (the
    joint can etch open); KiCad's connection_width class catches it. Uses the
    SAME closed-form erosion criterion (``terminal_pad_web_shortfall``) as the
    close_soft_joints connector that repairs it, so detection and repair agree.
    Read-only."""
    if floor <= 0 or not net_pads or not net_segs:
        return
    from pcb_modification import (terminal_pad_web_shortfall,
                                  terminal_web_neck_exact)
    from routing_utils import _to_pad_frame
    e = floor / 2.0

    def k(layer, x, y):
        return (layer, round(x, 4), round(y, 4))

    deg = {}
    for s in net_segs:
        if getattr(s, 'graphic', False):
            continue
        deg[k(s.layer, s.start_x, s.start_y)] = deg.get(k(s.layer, s.start_x, s.start_y), 0) + 1
        deg[k(s.layer, s.end_x, s.end_y)] = deg.get(k(s.layer, s.end_x, s.end_y), 0) + 1
    for s in net_segs:
        if getattr(s, 'graphic', False):
            continue
        r = s.width / 2.0
        if r < e - 1e-9:
            continue  # track thinner than the floor: no floor-width web exists
        for (ex, ey, nx, ny) in ((s.start_x, s.start_y, s.end_x, s.end_y),
                                 (s.end_x, s.end_y, s.start_x, s.start_y)):
            if deg.get(k(s.layer, ex, ey), 0) != 1:
                continue  # not a free end
            target = None
            for pad in net_pads:
                if getattr(pad, 'shape', None) not in ('rect', 'roundrect', 'oval'):
                    continue
                if not pad.size_x or not pad.size_y:
                    continue
                if not (s.layer in pad.layers or any('*' in L for L in pad.layers)):
                    continue
                if point_to_pad_distance(ex, ey, pad) < r - 1e-6:
                    target = pad
                    break
            if target is None:
                continue
            elx, ely = _to_pad_frame(ex, ey, target)
            nlx, nly = _to_pad_frame(nx, ny, target)
            is_neck, _ = terminal_pad_web_shortfall(
                nlx, nly, elx, ely, target.size_x / 2.0, target.size_y / 2.0, r, e)
            if is_neck and terminal_web_neck_exact(
                    pcb_data, net_id, s.layer, ex, ey, floor) is not False:
                findings.append(_finding(
                    'narrow_pad_joint', name, s.layer, ex, ey,
                    f"terminal cap joins pad {target.component_ref}."
                    f"{target.pad_number} through a copper web below the "
                    f"{floor:.3f}mm min-track floor (connection_width hazard)",
                    size=None))


def check_weird(pcb_data: PCBData, net_patterns: Optional[List[str]] = None,
                thorough: bool = False, quiet: bool = True,
                tolerance: float = 0.1
                ) -> Tuple[List[Dict], List[Tuple[str, int]]]:
    """Run every check. Returns (findings, skipped_nets) where each finding is
    {'category', 'net', 'layer', 'x', 'y', 'detail', 'size'} and skipped_nets
    lists (net_name, segment_count) nets the removable-segment scan skipped.
    Findings whose characteristic size (mm) is below `tolerance` are dropped
    (0 = report everything). Read-only: pcb_data is not modified."""
    findings: List[Dict] = []
    skipped_nets: List[Tuple[str, int]] = []

    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        segs_by_net[s.net_id].append(s)
    vias_by_net = defaultdict(list)
    for v in pcb_data.vias:
        vias_by_net[v.net_id].append(v)
    zones_by_net = defaultdict(list)
    for z in (pcb_data.zones or []):
        zones_by_net[z.net_id].append(z)

    copper_layers = (getattr(pcb_data.board_info, 'copper_layers', None)
                     or ['F.Cu', 'B.Cu'])

    # Connection-width floor for the terminal-web check (#416): the thinnest
    # track on the board -- KiCad's scan_board_minima min_track_width.
    _widths = [s.width for s in pcb_data.segments
               if not getattr(s, 'graphic', False) and s.width and s.width > 0]
    min_track_w = min(_widths) if _widths else 0.0

    net_ids = set(segs_by_net) | set(vias_by_net)
    check_ids = []
    for net_id in sorted(net_ids):
        if net_id == 0:
            continue  # unconnected copper has no same-net semantics
        if net_patterns is not None and not matches_any_pattern(
                _net_name(pcb_data, net_id), net_patterns):
            continue
        check_ids.append(net_id)

    if not quiet:
        print(f"Checking {len(check_ids)} nets "
              f"({len(pcb_data.segments)} segments, {len(pcb_data.vias)} vias)...")

    for net_id in check_ids:
        name = _net_name(pcb_data, net_id)
        net_segs = segs_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_zones = zones_by_net.get(net_id, [])
        has_zone = bool(net_zones)

        soft_pts = _check_soft_joints(net_id, name, net_segs, net_vias,
                                      net_pads, findings)
        _check_dangles(net_id, name, net_segs, net_vias, net_pads, net_zones,
                       soft_pts, findings, join_tol=tolerance or 0.0)
        _check_orphan_islands(net_id, name, net_segs, net_vias, net_pads,
                              net_zones, findings)
        _check_cycles(net_id, name, net_segs, net_vias, net_pads, has_zone,
                      findings)
        _check_removable(net_id, name, net_segs, net_vias, net_pads,
                         net_zones, has_zone, thorough, findings, skipped_nets)
        _check_stacked(net_id, name, net_segs, net_vias, findings)
        _check_unsupported_vias(net_id, name, net_segs, net_vias, net_pads,
                                net_zones, copper_layers, findings)
        _check_terminal_web(pcb_data, net_id, name, net_segs, net_pads,
                            min_track_w, findings)

    if tolerance and tolerance > 0:
        findings = [f for f in findings
                    if f.get('size') is None or f['size'] + 1e-9 >= tolerance]
    return findings, skipped_nets


def print_report(findings: List[Dict], skipped_nets: List[Tuple[str, int]],
                 max_print: int = 20) -> None:
    by_cat = defaultdict(list)
    for f in findings:
        by_cat[f['category']].append(f)
    if findings:
        print(f"\nFOUND {len(findings)} WEIRD THINGS:\n")
        for cat in CATEGORIES:
            items = by_cat.get(cat, [])
            print(f"  {cat}: {len(items)}")
            limit = len(items) if (max_print is not None and max_print <= 0) \
                else max_print
            for f in items[:limit]:
                print(f"    net {f['net']} ({f['layer']}) at "
                      f"({f['x']:.3f}, {f['y']:.3f}): {f['detail']}")
            if len(items) > limit:
                print(f"    ... and {len(items) - limit} more "
                      f"(use --max-print 0 to show all)")
    else:
        print("\nNO WEIRD THINGS FOUND!")
    if skipped_nets:
        print(f"\n  removable-segment: skipped {len(skipped_nets)} net(s) "
              f"with >{MAX_SEGS_PER_NET} segments "
              f"(pass --thorough to check them):")
        for nm, cnt in skipped_nets[:10]:
            print(f"    {nm}: {cnt} segments")
        if len(skipped_nets) > 10:
            print(f"    ... and {len(skipped_nets) - 10} more")


def main():
    parser = argparse.ArgumentParser(
        description='Check PCB for weird copper hygiene issues '
                    '(dangles, soft joints, loops, removable/stacked copper, '
                    'floating vias). Read-only: never modifies the board.')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Net name patterns to check (fnmatch wildcards '
                             'supported, e.g., "*lvds*")')
    parser.add_argument('--thorough', action='store_true',
                        help='Run the removable-segment scan on nets with '
                             f'>{MAX_SEGS_PER_NET} segments too (slow)')
    parser.add_argument('--tolerance', type=float, default=0.1,
                        help='Minimum finding size in mm (dangle/tail length, '
                             'gap, duplicated-copper length, via diameter); '
                             'smaller findings are dropped. Default 0.1; use '
                             '0 to report everything.')
    parser.add_argument('--max-print', type=int, default=20,
                        help='Max findings printed per category '
                             '(<=0 prints all; default 20)')
    args = parser.parse_args()

    print(f"Loading PCB file: {args.pcb}")
    pcb_data = parse_kicad_pcb(args.pcb)
    findings, skipped_nets = check_weird(pcb_data, args.nets,
                                          tolerance=args.tolerance,
                                         thorough=args.thorough, quiet=False)
    print_report(findings, skipped_nets, max_print=args.max_print)
    sys.exit(1 if findings else 0)


if __name__ == '__main__':
    main()
