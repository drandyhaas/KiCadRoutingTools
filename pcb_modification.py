"""
Route modification utilities for PCB routing.

Functions for adding/removing routes from PCB data and cleaning up
self-intersecting or redundant segments.
"""
from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Tuple

from kicad_parser import PCBData, Segment, Via
from routing_utils import pos_key, POSITION_DECIMALS, into_pad_frame_point


def get_copper_layers_from_segments(segments: List[Segment], existing_segments: List[Segment] = None) -> List[str]:
    """
    Build a list of all copper layers from segments.

    For through-hole vias that connect all layers, we need to know all copper layers
    present in the design. This function extracts them from the segments.

    Args:
        segments: New segments being processed
        existing_segments: Optional existing segments to also consider

    Returns:
        List of copper layer names (always includes F.Cu and B.Cu for through-hole vias)
    """
    all_copper_layers = set()
    for seg in segments:
        all_copper_layers.add(seg.layer)
    if existing_segments:
        for seg in existing_segments:
            all_copper_layers.add(seg.layer)
    # Ensure F.Cu and B.Cu are always included for through-hole vias
    all_copper_layers.add('F.Cu')
    all_copper_layers.add('B.Cu')
    return list(all_copper_layers)


def _point_anchored(x: float, y: float, layer: str, via_pts, pad_pts,
                    seg_index, cell: float, ignore_seg, tol: float) -> bool:
    """A segment endpoint is anchored if it lands on a same-net via (vias span
    layers), on a same-net pad (on a shared layer), or in the middle of another
    same-net segment on the same layer (a T-junction). Anchored endpoints are
    real connections, never dead ends.

    ``seg_index`` is a grid {(layer, cell_x, cell_y): [segments]} (each segment
    bucketed into the cells its bbox covers); the T-junction test scans only the
    3x3 cells around (x, y) instead of every same-net segment -- the interaction
    radius (a trace half-width + tol) is well under one cell, so no landing is
    missed. Without it this was O(endpoints x segments), the plane sweep's cost."""
    # A pad/via anchors a segment endpoint only when the segment's copper actually
    # reaches the pad/via copper: dist < (pad|via)_half + seg_half. The old fixed
    # +0.05 slop anchored a near-miss -- a fine-track (0.0889mm) plane-repair stub
    # that stopped 0.049mm OUTSIDE a 0.27mm BGA ball, its copper 0.0045mm short of
    # touching -- so a useless, GND-grazing dead-end stub was never swept (#209/#216).
    # Tying the slop to the segment's own half-width keeps wide power traces lenient
    # while a fine stub must genuinely reach the copper; the _safe_prune_net
    # connectivity gate still backstops any real connection this would flag.
    seg_half = (getattr(ignore_seg, 'width', 0.0) or 0.0) / 2.0
    for vx, vy, vsize in via_pts:
        if math.hypot(x - vx, y - vy) < vsize / 2 + seg_half:
            return True
    for px, py, psize, players in pad_pts:
        on_layer = (not players) or layer in players or any('*' in L for L in players)
        if on_layer and math.hypot(x - px, y - py) < psize / 2 + seg_half:
            return True
    ig_ends = (((ignore_seg.start_x, ignore_seg.start_y),
                (ignore_seg.end_x, ignore_seg.end_y)) if ignore_seg is not None else ())
    bx, by = int(x // cell), int(y // cell)
    seen = set()
    for gx in (bx - 1, bx, bx + 1):
        for gy in (by - 1, by, by + 1):
            for s in seg_index.get((layer, gx, gy), ()):
                if s is ignore_seg or id(s) in seen:
                    continue
                seen.add(id(s))
                # Skip a segment that shares a vertex with ignore_seg: a dead-end stub
                # that bends sharply lands its loose end near its OWN chain-neighbour,
                # and that fold is not a real T-junction onto independent copper --
                # counting it kept a useless GND-grazing plane-repair stub un-swept
                # (#209/#216 lpddr4 C30.2).
                if any((abs(s.start_x - ax) < tol and abs(s.start_y - ay) < tol) or
                       (abs(s.end_x - ax) < tol and abs(s.end_y - ay) < tol)
                       for ax, ay in ig_ends):
                    continue
                dx = s.end_x - s.start_x
                dy = s.end_y - s.start_y
                seg_len_sq = dx * dx + dy * dy
                if seg_len_sq < 1e-9:
                    continue
                t = ((x - s.start_x) * dx + (y - s.start_y) * dy) / seg_len_sq
                # Strictly interior (endpoints are handled by the degree count) so a
                # shared endpoint isn't double-counted as a T-junction.
                if t <= 0.02 or t >= 0.98:
                    continue
                cx = s.start_x + t * dx
                cy = s.start_y + t * dy
                # A landing anywhere within the trace's copper (half-width) is a real
                # connection; use the wider of tol and the trace half-width so a stub
                # landing inside a wide power trace is not mistaken for a dead end.
                if math.hypot(x - cx, y - cy) < max(tol, getattr(s, 'width', 0.0) / 2 + 0.025):
                    return True
    return False


def prune_dead_end_segments(prunable: List[Segment], anchor_segments: List[Segment] = None,
                            vias: List = None, pads: List = None,
                            tol: float = 0.05,
                            keep_terminal_escapes: bool = True) -> Tuple[List[Segment], List[Segment]]:
    """Iteratively drop a net's dead-end segments (issue #84).

    A dead end is a segment endpoint of degree 1 -- no other same-net segment
    endpoint coincides with it on its layer -- that also does not land on a pad,
    a via, or the interior of another same-net segment (a T-junction). Such an
    endpoint connects nothing, so the segment is dead copper: a tap tail left
    when the branch it fed was superseded, a stub a net routed away from, a
    fragment orphaned by rip-and-reroute. Removing it can never disconnect the
    net (the other end stays joined to the rest), and it exposes the next
    segment of a spur chain, so this iterates to a fixpoint.

    This is the whole-net post-route dead-end trim (#84): it removes dead ends of
    any length and unwinds whole spurs (the per-commit ``collapse_appendices``
    short-spur pass it superseded was removed as redundant in #148).

    Args:
        prunable: segments eligible for removal (one net).
        anchor_segments: extra same-net segments that count toward junctions /
            T-anchoring but are never removed (e.g. original file copper that the
            output writer cannot delete). Endpoints shared with these are kept.
        vias, pads: same-net vias / pads that anchor an endpoint.
        tol: proximity tolerance (mm) for the on-segment / coincidence tests.

    Returns ``(kept, removed)`` from ``prunable``; ``anchor_segments`` are never
    returned (they were never candidates).
    """
    anchor_segments = anchor_segments or []
    via_pts = [(v.x, v.y, getattr(v, 'size', 0.6)) for v in (vias or [])]
    pad_pts = []
    for p in (pads or []):
        px = getattr(p, 'global_x', getattr(p, 'x', 0.0))
        py = getattr(p, 'global_y', getattr(p, 'y', 0.0))
        psize = max(getattr(p, 'size_x', 0.5), getattr(p, 'size_y', 0.5))
        pad_pts.append((px, py, psize, getattr(p, 'layers', [])))

    def key(x, y, layer):
        return (round(x, 3), round(y, 3), layer)

    from collections import defaultdict
    _CELL = 1.0
    kept = list(prunable)
    removed = []
    changed = True
    while changed:
        changed = False
        all_segs = kept + anchor_segments
        degree = {}
        seg_index = defaultdict(list)
        for s in all_segs:
            degree[key(s.start_x, s.start_y, s.layer)] = \
                degree.get(key(s.start_x, s.start_y, s.layer), 0) + 1
            degree[key(s.end_x, s.end_y, s.layer)] = \
                degree.get(key(s.end_x, s.end_y, s.layer), 0) + 1
            lo_x = int(min(s.start_x, s.end_x) // _CELL); hi_x = int(max(s.start_x, s.end_x) // _CELL)
            lo_y = int(min(s.start_y, s.end_y) // _CELL); hi_y = int(max(s.start_y, s.end_y) // _CELL)
            for cx in range(lo_x, hi_x + 1):
                for cy in range(lo_y, hi_y + 1):
                    seg_index[(s.layer, cx, cy)].append(s)

        survivors = []
        for s in kept:
            sk = key(s.start_x, s.start_y, s.layer)
            ek = key(s.end_x, s.end_y, s.layer)
            start_free = (degree[sk] == 1 and
                          not _point_anchored(s.start_x, s.start_y, s.layer,
                                              via_pts, pad_pts, seg_index, _CELL, s, tol))
            end_free = (degree[ek] == 1 and
                        not _point_anchored(s.end_x, s.end_y, s.layer,
                                            via_pts, pad_pts, seg_index, _CELL, s, tol))
            remove = False
            if start_free and end_free:
                remove = True                      # isolated fragment
            elif start_free or end_free:
                # Exactly one free end. The rooted end is either a junction
                # (degree >= 2 -- a spur hanging off the through-path) or a
                # degree-1 pad/via anchor (the net's escape stub).
                root = ek if start_free else sk
                if degree[root] >= 2:
                    remove = True                  # spur off the through-path
                elif not keep_terminal_escapes:
                    # Whole branch back to a pad/via is dead (the chain unwound to
                    # here). It is a dead antenna -- the pad/via connects nothing
                    # through it -- so removing it cannot change connectivity. The
                    # per-commit pass keeps these (the net may still be routing);
                    # the final sweep removes them.
                    remove = True
            if remove:
                removed.append(s)
                changed = True
            else:
                survivors.append(s)
        kept = survivors
    return kept, removed


# Width clamp for the STRICT connectivity gate (#322): overlap connectivity
# is width-dependent (endpoint caps touch when gap < (w1+w2)/2), so a fat
# power track grades "connected" across a 0.28mm hole in its own chain. For
# REMOVAL decisions the cleanup passes also grade a width-clamped twin of the
# net -- 0.02mm keeps quantization-level coincidence (<=10um slack per end,
# matching the cycle prune's tol and SOFT_JOINT_MIN_GAP) while cap lenses and
# wide T-slop no longer count. Grading/checker semantics are untouched: this
# is deliberately asymmetric (measure reality physically; refuse to ship
# fragility). See issue #322 (smartknob +5V: mid-chain removals each passed
# the overlap gate until 5 pads were genuinely disconnected).
from connectivity import COINCIDENCE_TOL as _STRICT_GATE_WIDTH  # one constant (#320)


def _strict_conn_graph(net_id, universe, vias, pads, zones):
    """check_net_connectivity graph over width-clamped copies of ``universe``
    (same order, so analyze_conn_excluding indices are interchangeable with
    the physical graph's). Returns (result_dict, graph_or_None)."""
    import copy as _copy
    from check_connected import check_net_connectivity
    clamped = []
    for s in universe:
        c = _copy.copy(s)
        c.width = min(c.width, _STRICT_GATE_WIDTH)
        clamped.append(c)
    r = check_net_connectivity(net_id, clamped, vias, pads, zones,
                               return_graph=True)
    return r, r.get('graph')


def _safe_prune_net(net_id, prunable, vias, pads, zones,
                    anchor_segments=None, aggressive=False, tol=None):
    """Prune a net's dead ends, but never at the cost of pad connectivity.

    prune_dead_end_segments works on an endpoint-coincidence model that does not
    know about zones (a segment ending on a plane is connected) or segment
    overlap, so on its own it can remove copper that actually carries a pad to a
    plane. This gates it against check_net_connectivity (the authoritative
    union-find the connectivity checker uses).

    Removing any subset of genuinely-dead copper is independently connectivity-
    safe, so rather than accept-or-revert the whole net, each flagged segment is
    validated on its own: drop it only if doing so does not raise the net's
    disconnected-pad count. A dead-end that actually lands on a plane (the
    geometric model's blind spot) fails that check and is kept, while the net's
    true dead ends are still removed. Returns ``(kept_prunable, removed)``.
    """
    if tol is None:
        from connectivity import COINCIDENCE_TOL
        tol = COINCIDENCE_TOL  # #320: the one strict coincidence tolerance
    _, candidates = prune_dead_end_segments(prunable, anchor_segments=anchor_segments,
                                            vias=vias, pads=pads, tol=tol,
                                            keep_terminal_escapes=not aggressive)
    if not candidates:
        return prunable, []

    from check_connected import check_net_connectivity, analyze_conn_excluding
    anchor = anchor_segments or []

    # Every trial below is (anchor + prunable) minus some prunable segments, so
    # build the expensive spatial union graph ONCE and re-evaluate each trial by
    # dropping the excluded segments' edges instead of a full-net rebuild --
    # O(net + trials) instead of O(net x trials), the same cached-graph fast
    # path as prune_grazing_segments (#263; this dead-end sweep was ~60% of a
    # daisho plane-repair run on its 18k-segment GND net). PRUNE_CONN_VERIFY=1
    # checks every fast-path count against a real recompute.
    universe = anchor + list(prunable)
    graph = check_net_connectivity(net_id, universe, vias, pads, zones,
                                   return_graph=True).get('graph')
    # Strict twin (#322): removals must ALSO not worsen coincidence-level
    # connectivity, or fat-cap lenses let a chain be chipped hole by hole.
    _, graph_strict = _strict_conn_graph(net_id, universe, vias, pads, zones)
    seg_pos = {id(s): i for i, s in enumerate(universe)}
    prunable_ids = [id(s) for s in prunable]
    _verify = os.environ.get('PRUNE_CONN_VERIFY')

    def disconnected(segs):
        if graph is not None:
            keep = {id(s) for s in segs}
            excl = {seg_pos[pid] for pid in prunable_ids if pid not in keep}
            n = len(analyze_conn_excluding(graph, excl)['disconnected_pads'])
            if _verify:
                ref = len(check_net_connectivity(net_id, anchor + segs, vias, pads,
                                                 zones)['disconnected_pads'])
                assert n == ref, \
                    f"safe-prune fast-path mismatch: net {net_id} ({n} vs {ref})"
            ns = (len(analyze_conn_excluding(graph_strict, excl)['disconnected_pads'])
                  if graph_strict is not None else 0)
            return (n, ns)
        return (len(check_net_connectivity(net_id, anchor + segs, vias, pads,
                                           zones)['disconnected_pads']), 0)

    base = disconnected(list(prunable))
    kept = list(prunable)
    kept_ids = {id(s) for s in kept}
    removed = []
    # Removing a dead end can expose its neighbour as a new dead end (a chain
    # unwinds one segment at a time), so iterate: re-derive candidates from what
    # is left until a full pass removes nothing. A candidate whose removal would
    # strand a pad stays load-bearing no matter what other dead copper goes, so
    # cache those rejections and never re-test them (keeps it O(dead ends) and
    # guarantees termination).
    #
    # Fast path: try dropping the WHOLE round's candidate batch with one
    # connectivity check. Dead-end removal is monotonic -- removing copper never
    # reconnects a pad -- so if dropping every candidate strands no pad, then so
    # does every subset, and the batch result is identical to validating one at a
    # time. This turns the plane sweep on a big pour net (hundreds of tap dead
    # ends x a full-net union-find each) from O(dead ends) checks into ~O(rounds).
    # Only when the batch DOES strand a pad do we fall back to per-candidate to
    # find the load-bearing one(s).
    rejected = set()
    while True:
        active = [c for c in candidates
                  if id(c) not in rejected and id(c) in kept_ids]
        if not active:
            break
        aids = {id(c) for c in active}
        trial_all = [s for s in kept if id(s) not in aids]
        _d = disconnected(trial_all)
        if _d[0] <= base[0] and _d[1] <= base[1]:
            kept = trial_all
            kept_ids = {id(s) for s in kept}
            removed.extend(active)
        else:
            progress = False
            for c in active:
                trial = [s for s in kept if s is not c]
                _d = disconnected(trial)
                if _d[0] <= base[0] and _d[1] <= base[1]:
                    kept = trial
                    kept_ids.discard(id(c))
                    removed.append(c)
                    progress = True
                else:
                    rejected.add(id(c))
            if not progress:
                break
        _, candidates = prune_dead_end_segments(kept, anchor_segments=anchor_segments,
                                                vias=vias, pads=pads, tol=tol,
                                                keep_terminal_escapes=not aggressive)
    return kept, removed


def _nearest_pad_point(px, py, pad):
    """Nearest point on a pad's (rotated) bounding box to (px, py), and the gap."""
    cx, cy = pad.global_x, pad.global_y
    # size_x/size_y are board-resolved (axis-aligned for orthogonal pads); only
    # the residual rect_rotation tilts the rectangle - NOT the total pad rotation.
    rot = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    ca, sa = math.cos(-rot), math.sin(-rot)
    # into pad-local frame
    lx = (px - cx) * ca - (py - cy) * sa
    ly = (px - cx) * sa + (py - cy) * ca
    hx, hy = pad.size_x / 2, pad.size_y / 2
    clx = max(-hx, min(hx, lx))
    cly = max(-hy, min(hy, ly))
    # back to board frame
    ca2, sa2 = math.cos(rot), math.sin(rot)
    tx = cx + clx * ca2 - cly * sa2
    ty = cy + clx * sa2 + cly * ca2
    return (tx, ty), math.hypot(px - tx, py - ty)


def _duplicate_connector(px: float, py: float, tx: float, ty: float,
                         segs, tol: float = 0.02) -> bool:
    """True if a same-net segment in ``segs`` already directly joins (px,py) and
    (tx,ty) on this layer.

    snap_stub_gaps treats a degree-1 endpoint as a loose stub even when a via /
    TH pad anchors it to another layer, so it can try to bridge that endpoint to
    copper it is ALREADY joined to -- adding a segment coincident with an existing
    one. That duplicate is a degenerate 2-edge cycle, which prune_redundant_cycles
    then "breaks" by deleting load-bearing copper, silently disconnecting the pad
    (issue #209, free_dap +3V3 IC2.13). Such a connector is always redundant (the
    join already exists), so suppressing it can never disconnect anything -- unlike
    skipping the endpoint outright, which can drop a genuinely load-bearing snap."""
    for s in segs:
        a = (abs(s.start_x - px) < tol and abs(s.start_y - py) < tol and
             abs(s.end_x - tx) < tol and abs(s.end_y - ty) < tol)
        b = (abs(s.start_x - tx) < tol and abs(s.start_y - ty) < tol and
             abs(s.end_x - px) < tol and abs(s.end_y - py) < tol)
        if a or b:
            return True
    return False


def close_soft_joints(results, pcb_data: PCBData, scope_net_ids, config,
                      clearance: float = None) -> int:
    """Bridge same-net SOFT JOINTS with a TINY coincident segment (#soft-joint).

    A soft joint is a dangling free end (a segment terminus that is not a shared
    vertex, a via, or an own pad) that reaches the rest of the net ONLY by cap-
    OVERLAPPING another dangling free end -- a fragile near-open left when a
    rip-up deleted the real connecting segment (butterstick DQ5) or a tap landed
    on-grid short of the off-grid endpoint it joined. Rather than rely on the
    overlap (which check_drc flags as 'segment-endpoint-gap') or snap a tap
    endpoint (which distorts its route), add a short segment from endpoint 1 to
    endpoint 2 so the joint becomes a COINCIDENT connection. The bridge spans only
    the gap (< a track width), so it is always tiny. Uses check_drc's exact
    soft-joint definition/tolerance so detection and repair agree. The bridge is
    only added when it clears every OTHER net's copper (it lives inside the two
    overlapping caps, so it normally does). Returns the number of bridges added.
    """
    import math
    from collections import defaultdict
    from check_drc import _SOFT_JOINT_MIN_GAP, point_to_pad_distance
    from single_ended_routing import (_seg_foreign_pad_dist, _seg_foreign_seg_dist,
                                       _seg_foreign_via_dist, _seg_foreign_hole_dist)
    clr = config.clearance if clearance is None else clearance

    def rk(x, y):
        return (round(x, 3), round(y, 3))

    ep_count = defaultdict(int)
    for s in pcb_data.segments:
        if scope_net_ids is not None and s.net_id not in scope_net_ids:
            continue
        if getattr(s, 'graphic', False):
            continue  # #337: immutable art -- its termini are not dangles
        ep_count[(s.net_id, s.layer, rk(s.start_x, s.start_y))] += 1
        ep_count[(s.net_id, s.layer, rk(s.end_x, s.end_y))] += 1
    via_by_net = defaultdict(list)
    for v in pcb_data.vias:
        via_by_net[v.net_id].append((v.x, v.y, (getattr(v, 'size', 0) or 0) / 2.0))

    def at_anchor(nid, x, y):
        for vx, vy, vr in via_by_net.get(nid, []):
            if math.hypot(x - vx, y - vy) <= vr + 0.01:
                return True
        for p in pcb_data.pads_by_net.get(nid, []):
            if point_to_pad_distance(x, y, p) <= 0.02:
                return True
        return False

    dangles = defaultdict(list)  # (net_id, layer) -> [(x, y, width)]
    for s in pcb_data.segments:
        if scope_net_ids is not None and s.net_id not in scope_net_ids:
            continue
        for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if ep_count[(s.net_id, s.layer, rk(x, y))] != 1:
                continue
            if at_anchor(s.net_id, x, y):
                continue
            dangles[(s.net_id, s.layer)].append((x, y, s.width))

    def clears(nid, x1, y1, x2, y2, layer, w):
        d = min(_seg_foreign_pad_dist(pcb_data, nid, x1, y1, x2, y2, layer),
                _seg_foreign_seg_dist(pcb_data, nid, x1, y1, x2, y2, layer),
                _seg_foreign_via_dist(pcb_data, nid, x1, y1, x2, y2, layer),
                _seg_foreign_hole_dist(pcb_data, nid, x1, y1, x2, y2))
        return d >= clr + w / 2.0 - 1e-4

    new_conns = []
    for (net_id, layer), ends in dangles.items():
        used = set()
        for i in range(len(ends)):
            if i in used:
                continue
            xi, yi, wi = ends[i]
            for j in range(i + 1, len(ends)):
                if j in used:
                    continue
                xj, yj, wj = ends[j]
                gap = math.hypot(xi - xj, yi - yj)
                cap = (wi + wj) / 2.0
                if _SOFT_JOINT_MIN_GAP < gap < cap - 1e-6:
                    w = min(wi, wj)
                    if not clears(net_id, xi, yi, xj, yj, layer, w):
                        continue
                    new_conns.append(Segment(start_x=xi, start_y=yi, end_x=xj,
                                             end_y=yj, width=w, layer=layer, net_id=net_id))
                    used.add(i); used.add(j)
                    break
    if new_conns:
        for c in new_conns:
            pcb_data.segments.append(c)
        # Tagged so accounting/summary code can tell this cleanup copper from a
        # net's routed result (it has no net-level identity of its own).
        results.append({'new_segments': new_conns, 'new_vias': [],
                        'cleanup': 'soft_joint_bridge'})
    return len(new_conns)


def snap_stub_gaps(results, pcb_data: PCBData, scope_net_ids, config,
                   max_gap_factor: float = 1.5) -> int:
    """Close small gaps where a routed dead end stopped just short of same-net
    copper (issue #84).

    A route can land up to ~half a grid step shy of its target, leaving a stub
    whose loose end is a fraction of a track width from a same-net pad, via, or
    trace. The connectivity model bridges that with tolerance, but the copper does
    not physically touch -- KiCad's DRC sees a dangling end. Rather than report it
    or loosen the checker, extend the stub with a short connector to the nearest
    same-net copper, provided the connector clears every OTHER net's copper by the
    configured clearance (same gate principle as removal, applied to addition).

    Only gaps up to ``max_gap_factor`` x the stub's track width are closed. Adds
    the connector to ``results`` (and pcb_data) so both the CLI writer and the GUI
    pick it up. Returns the number of connectors added.
    """
    coord_clear = config.clearance
    added = 0
    new_conns = []

    # Board outline / cutouts (#281): a connector is drawn geometrically, not
    # routed, so it must be gated against Edge.Cuts itself -- on sofle_pico a
    # snap toward a reverse-mount LED pad's bbox corner ran straight across
    # the LED's window cutout.
    from check_drc import (board_edge_geometry, _point_on_board,
                           _segment_to_rings_distance)
    edge_rings, edge_outer, edge_cutouts = board_edge_geometry(
        getattr(pcb_data, 'board_info', None))

    # Same-net copper grouped for fast lookup.
    segs_by_net_layer = {}
    for s in pcb_data.segments:
        if getattr(s, 'graphic', False):
            continue  # #337: never snap an immutable-art endpoint
        segs_by_net_layer.setdefault((s.net_id, s.layer), []).append(s)

    for net_id in scope_net_ids:
        net = pcb_data.nets.get(net_id)
        if net is None:
            continue
        net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        for (nid, lyr), segs in list(segs_by_net_layer.items()):
            if nid != net_id:
                continue
            # Degree-1 endpoints on this layer (exact-coord coincidence).
            deg = {}
            for s in segs:
                deg[(s.start_x, s.start_y)] = deg.get((s.start_x, s.start_y), 0) + 1
                deg[(s.end_x, s.end_y)] = deg.get((s.end_x, s.end_y), 0) + 1
            for s in segs:
                for (px, py) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
                    if deg[(px, py)] != 1:
                        continue
                    w = s.width
                    limit = max_gap_factor * w
                    best = None  # (gap, target_point)
                    # nearest same-net trace on this layer (a real T-junction)
                    for o in segs:
                        if o is s:
                            continue
                        dx, dy = o.end_x - o.start_x, o.end_y - o.start_y
                        L2 = dx * dx + dy * dy
                        if L2 < 1e-12:
                            continue
                        t = max(0.0, min(1.0, ((px - o.start_x) * dx + (py - o.start_y) * dy) / L2))
                        fx, fy = o.start_x + t * dx, o.start_y + t * dy
                        g = math.hypot(px - fx, py - fy)
                        if best is None or g < best[0]:
                            best = (g, (fx, fy))
                    # nearest same-net pad on this layer (land on its copper)
                    for pad in net_pads:
                        if not (lyr in pad.layers or any('*' in L for L in pad.layers)):
                            continue
                        # A 'custom'-shaped pad's size is only a bounding box;
                        # its corner can be off-copper (#281: an SK6803 LED
                        # aperture). Prefer the coincident anchor pad (same
                        # component+number, real rect/roundrect) when one
                        # exists -- it is guaranteed copper.
                        if getattr(pad, 'shape', None) == 'custom' and any(
                                q is not pad
                                and getattr(q, 'shape', None) != 'custom'
                                and getattr(q, 'component_ref', None) ==
                                    getattr(pad, 'component_ref', None)
                                and getattr(q, 'pad_number', None) ==
                                    getattr(pad, 'pad_number', None)
                                and abs(q.global_x - pad.global_x) < 1e-6
                                and abs(q.global_y - pad.global_y) < 1e-6
                                for q in net_pads):
                            continue
                        tp, g = _nearest_pad_point(px, py, pad)
                        if best is None or g < best[0]:
                            best = (g, tp)
                    # nearest same-net via (vias span layers): land just inside it
                    for v in net_vias:
                        dc = math.hypot(px - v.x, py - v.y)
                        r = getattr(v, 'size', 0.0) / 2
                        if dc < 1e-9:
                            continue
                        ux, uy = (v.x - px) / dc, (v.y - py) / dc
                        tp = (px + ux * max(0.0, dc - 0.9 * r), py + uy * max(0.0, dc - 0.9 * r))
                        g = math.hypot(px - tp[0], py - tp[1])
                        if best is None or g < best[0]:
                            best = (g, tp)

                    if best is None or not (1e-4 < best[0] <= limit):
                        continue  # already touching, no target, or gap too big
                    tx, ty = best[1]

                    # Don't add a connector coincident with an existing same-net
                    # segment (issue #209): the endpoint only looked loose because
                    # a via/TH pad anchors it to another layer, so the join already
                    # exists. The duplicate would seed a degenerate cycle.
                    if _duplicate_connector(px, py, tx, ty, segs):
                        continue

                    # Board-edge check (#281): the connector's copper must stay
                    # on the board and keep clearance from the outline and
                    # every cutout, exactly as check_drc grades segments.
                    if edge_rings:
                        if not (_point_on_board(px, py, edge_outer, edge_cutouts)
                                and _point_on_board(tx, ty, edge_outer,
                                                    edge_cutouts)):
                            continue
                        if _segment_to_rings_distance(px, py, tx, ty, edge_rings) \
                                < w / 2 + coord_clear - 1e-6:
                            continue

                    # Clearance check: the connector must keep `clearance` from
                    # every OTHER net's copper.
                    if not _connector_clear(px, py, tx, ty, w, lyr, net_id,
                                            pcb_data, coord_clear):
                        continue
                    conn = Segment(start_x=px, start_y=py, end_x=tx, end_y=ty,
                                   width=w, layer=lyr, net_id=net_id)
                    new_conns.append(conn)
                    segs.append(conn)  # so a later endpoint sees it connected
                    deg[(px, py)] = deg.get((px, py), 0) + 1
                    deg[(tx, ty)] = deg.get((tx, ty), 0) + 1
                    added += 1

    if new_conns:
        for c in new_conns:
            pcb_data.segments.append(c)
        # Tagged like close_soft_joints' bridges: cleanup copper, not a route.
        results.append({'new_segments': new_conns, 'new_vias': [],
                        'cleanup': 'stub_gap_snap'})
    return added


def _connector_clear(x1, y1, x2, y2, width, layer, net_id, pcb_data, clearance):
    """True if a candidate connector segment keeps `clearance` from all OTHER
    nets' copper (segments on its layer, vias on any layer, pads on its layer)."""
    from geometry_utils import segment_to_segment_distance, point_to_segment_distance
    from check_drc import segment_to_rect_distance
    half = width / 2
    for s in pcb_data.segments:
        if s.net_id == net_id or s.layer != layer:
            continue
        if segment_to_segment_distance(x1, y1, x2, y2,
                                       s.start_x, s.start_y, s.end_x, s.end_y) \
                < clearance + half + s.width / 2:
            return False
    for v in pcb_data.vias:
        if v.net_id == net_id:
            continue
        if point_to_segment_distance(v.x, v.y, x1, y1, x2, y2) \
                < clearance + half + getattr(v, 'size', 0.0) / 2:
            return False
    for nid, pads in pcb_data.pads_by_net.items():
        if nid == net_id:
            continue
        for pad in pads:
            if not (layer in pad.layers or any('*' in L for L in pad.layers)):
                continue
            # Rotate the segment into the pad's frame so a tilted pad is tested
            # against its true rectangle (distance is rotation-invariant).
            rx1, ry1 = into_pad_frame_point(x1, y1, pad)
            rx2, ry2 = into_pad_frame_point(x2, y2, pad)
            d, _ = segment_to_rect_distance(rx1, ry1, rx2, ry2, pad.global_x, pad.global_y,
                                            pad.size_x / 2, pad.size_y / 2)
            if d < clearance + half:
                return False
    # Other-net copper pours (planes): a connector must not enter or graze them.
    zones = getattr(pcb_data, 'zones', None)
    if zones:
        from obstacle_map import point_in_polygon, point_to_polygon_edge_distance
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
        for z in zones:
            if z.net_id == net_id or z.layer != layer or not z.polygon:
                continue
            for i in range(n + 1):
                t = i / n
                px, py = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
                if point_in_polygon(px, py, z.polygon) or \
                        point_to_polygon_edge_distance(px, py, z.polygon) < clearance + half:
                    return False
    return True


def clean_plane_copper(output_file: str, plane_net_names, clearance: float = 0.1,
                       grid_step: float = 0.05) -> Tuple[int, int]:
    """Run the shared post-route cleanup pipeline on a plane tool's OUTPUT FILE
    (issues #84, #308, #319 restructure).

    route_planes / route_disconnected_planes write copper outside route.py's
    write-list, so they miss the in-route cleanup. This re-parses the written
    board and runs the same run_post_route_cleanup the signal fronts use --
    gap snap, graze prune, octolinear re-bend, micro-shift (full-grid cap,
    #308: urti's GND repair track grazed J3's mounting hole by 17um), cycle
    prune (a no-op on zoned nets by design), dead-end sweep, and the final
    soft-joint bridge -- then rewrites the file (stripping removed segments,
    appending added ones). Because the board here IS a fresh parse of the
    file, the board==file contract holds by construction; the only excluded
    passes are the ones a segment-level file rewrite cannot express (via
    moves) or that need a routing write-list (phantom drop, width neck).
    The connectivity gates use the plane zones, so a tap the fill makes
    redundant can be dropped and nothing that would enter another net's pour
    is added. Returns ``(snapped, removed)``.
    """
    from types import SimpleNamespace
    from kicad_parser import parse_kicad_pcb, is_kicad_10
    from kicad_writer import remove_segments_from_content, generate_segment_sexpr
    # Local import: cleanup_pipeline imports the passes from this module.
    from cleanup_pipeline import run_post_route_cleanup

    pcb = parse_kicad_pcb(output_file)
    with open(output_file, 'r', encoding='utf-8') as f:
        content = f.read()
    names = set(plane_net_names)
    scope = {nid for nid, net in pcb.nets.items() if net.name in names}
    if not scope:
        return 0, 0

    # Run the ONE shared cleanup pipeline on the re-parsed board. In this
    # file-round-trip mode every segment is an "original" (nothing routed in
    # this process), so pass an empty write-list: additions (snap connectors,
    # micro-shift replacements, soft-joint bridges) come back as tagged
    # results entries; removals come back in input_strip_segments.
    # Front-parity switches:
    #   phantom OFF -- no write-list to reconcile;
    #   via_nudge OFF -- an input-via's in-place move cannot be expressed by
    #     this segment-level write-back (#280/#281), it would strand the
    #     dragged segment endpoints;
    #   neck OFF -- width-only fix for ROUTED wide trunks; nothing routed here;
    #   microshift_max_shift = grid_step (not grid_step/2): plane-repair
    #     quantization grazes can be a full grid cell (#308 -- the track is
    #     on-grid, the hole off-grid), and each shift is still verified to
    #     clear all foreign copper and keep the net connected.
    plane_results: list = []
    _cfg = SimpleNamespace(clearance=clearance, grid_step=grid_step)
    _outcome = run_post_route_cleanup(
        plane_results, pcb, scope, _cfg,
        label='Plane ', phantom=False, via_nudge=False, neck=False,
        microshift_max_shift=grid_step)
    snapped = _outcome.counts.get('stub_gaps_snapped', 0)
    to_remove = _outcome.input_strip_segments
    connectors = [s for r in plane_results for s in (r.get('new_segments') or [])]

    if not (connectors or to_remove):
        return 0, 0

    n2n = getattr(pcb, 'net_id_to_name', {}) or {}
    v10 = is_kicad_10(content)
    if to_remove:
        content, _ = remove_segments_from_content(content, to_remove, n2n if v10 else None)
    if connectors:
        sexprs = [generate_segment_sexpr((s.start_x, s.start_y), (s.end_x, s.end_y),
                                         s.width, s.layer, s.net_id,
                                         n2n.get(s.net_id) if v10 else None)
                  for s in connectors]
        lp = content.rfind(')')
        content = content[:lp] + '\n'.join(sexprs) + '\n' + content[lp:]
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)
    return snapped, len(to_remove)


def _rk3(x, y):
    """Endpoint-coincidence key at the soft-joint rounding (1um)."""
    return (round(x, 3), round(y, 3))


def _soft_joint_pairs(segs, vias, pads):
    """Canonical set of SOFT-JOINT pairs among ``segs``: same-layer degree-1
    free ends (not anchored on a same-net via/pad) whose round end caps overlap
    without the endpoints being coincident. Each pair is a frozenset of two
    (layer, rounded-point) keys, so pair sets from different segment subsets of
    the same net are directly comparable."""
    from collections import defaultdict
    from routing_constants import SOFT_JOINT_MIN_GAP
    from check_drc import point_to_pad_distance

    via_pts = [(v.x, v.y, (getattr(v, 'size', 0) or 0) / 2.0) for v in (vias or [])]

    def anchored(x, y):
        for vx, vy, vr in via_pts:
            if math.hypot(x - vx, y - vy) <= vr + 0.01:
                return True
        for p in (pads or []):
            if point_to_pad_distance(x, y, p) <= 0.02:
                return True
        return False

    deg = defaultdict(int)
    for s in segs:
        deg[(s.layer, _rk3(s.start_x, s.start_y))] += 1
        deg[(s.layer, _rk3(s.end_x, s.end_y))] += 1
    dangles = defaultdict(list)  # layer -> [(key, x, y, width)]
    seen = set()
    for s in segs:
        for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            key = (s.layer, _rk3(x, y))
            if deg[key] != 1 or key in seen:
                continue
            if anchored(x, y):
                continue
            seen.add(key)
            dangles[s.layer].append((key, x, y, s.width))
    pairs = set()
    for layer, ends in dangles.items():
        for i in range(len(ends)):
            ki, xi, yi, wi = ends[i]
            for j in range(i + 1, len(ends)):
                kj, xj, yj, wj = ends[j]
                gap = math.hypot(xi - xj, yi - yj)
                if SOFT_JOINT_MIN_GAP < gap < (wi + wj) / 2.0 - 1e-6:
                    pairs.add(frozenset((ki, kj)))
    return pairs


def _restore_soft_joint_bridges(kept, removed, vias, pads):
    """Restrictive guard for the copper-removal passes (issue #319): put back any
    just-removed segment whose removal CREATED a new SOFT JOINT anywhere on the
    net -- two dangling free ends (degree-1, not on a via/pad) held together only
    by cap overlap.

    sweep_dead_ends / prune_redundant_cycles gate removals on the OVERLAP
    connectivity model, which counts cap overlap as "still connected" and so
    happily deletes the coincident bridge between two pieces (butterstick DQ5's
    escape<->tap link). Generalized from the original both-endpoints-of-the-
    removed-segment shape: the confirmed glasgow B1 mechanism is a removal that
    turns a NEIGHBOUR endpoint into a degree-1 dangle which then cap-overlaps a
    THIRD dangle elsewhere on the net -- the old guard missed it, which is why
    mirroring the sweep into pcb_data used to let close_soft_joints plant a
    butterfly bridge. Soft joints that existed BEFORE the removals (router-born)
    never trigger a restore -- repairing those is close_soft_joints' job.

    This pass NEVER removes copper -- it only moves segments back from
    ``removed`` to ``kept`` -- so it cannot regress connectivity or change any
    connectivity definition anywhere; it just stops the subtractive passes from
    manufacturing a soft joint. Returns updated ``(kept, removed)``.
    """
    if not removed:
        return kept, removed
    baseline = _soft_joint_pairs(list(kept) + list(removed), vias, pads)
    for _ in range(len(removed)):
        new_pairs = _soft_joint_pairs(kept, vias, pads) - baseline
        if not new_pairs:
            break
        hot = {key for pair in new_pairs for key in pair}
        restored = False
        for r in list(removed):
            if ((r.layer, _rk3(r.start_x, r.start_y)) in hot
                    or (r.layer, _rk3(r.end_x, r.end_y)) in hot):
                # Restoring r re-anchors the dangle (its degree rises above 1),
                # dissolving the new pair. r's other end may re-add a dangle,
                # but any pair that one forms was already in the baseline.
                kept.append(r)
                removed.remove(r)
                restored = True
                break
        if not restored:
            break  # gap not attributable to these removals; leave it
    return kept, removed


def sweep_dead_ends(results, pcb_data: PCBData, scope_net_ids=None,
                    tol: float = None) -> Tuple[int, int, List[Segment]]:
    """Final whole-net dead-end sweep, after routing has settled (issue #84).

    the per-commit self-intersection clean (removed #159) used to fix
    self-intersections (its short-appendix trim was removed in #148 as redundant
    with this sweep), so dead ends survive on nets that otherwise route
    100% and pass DRC + connectivity: a tap tail superseded by a rip-and-reroute,
    a spur left when a blocker was ripped, and -- the dominant source -- fanout /
    escape stubs from earlier pipeline stages that a net routed away from or never
    completed. This prunes each in-scope net's FULL board copper once via
    prune_dead_end_segments, so original (input-file) dead copper is reached too,
    not only this run's new copper.

    Removed copper is split by origin:
      * segments/vias produced by this run (present in ``results``) are dropped
        from the write-list in place;
      * original input-file segments are returned so the caller can strip them
        from the output (the writer otherwise copies the input verbatim).

    ``scope_net_ids`` limits the sweep to the nets this run was asked to route
    (so untouched planes / excluded nets are never altered); None sweeps every net
    with copper. Returns ``(segments_removed, vias_removed, original_segments_to_remove)``.
    """
    from collections import defaultdict

    if tol is None:
        from connectivity import COINCIDENCE_TOL
        tol = COINCIDENCE_TOL  # #320: the one strict coincidence tolerance

    routed_seg_ids = set()
    for r in results:
        for s in r.get('new_segments') or []:
            routed_seg_ids.add(id(s))

    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        if scope_net_ids is None or s.net_id in scope_net_ids:
            segs_by_net[s.net_id].append(s)

    all_zones = getattr(pcb_data, 'zones', []) or []
    removed_routed_ids = set()
    original_to_remove = []
    kept_segs_by_net = {}
    for net_id, net_segs in segs_by_net.items():
        vias = [v for v in pcb_data.vias if v.net_id == net_id]
        pads = pcb_data.pads_by_net.get(net_id, [])
        zones = [z for z in all_zones if z.net_id == net_id]
        kept, removed = _safe_prune_net(net_id, net_segs, vias, pads, zones,
                                        aggressive=True, tol=tol)
        # #319: never delete a coincident bridge and leave a soft joint.
        kept, removed = _restore_soft_joint_bridges(kept, removed, vias, pads)
        # Copper graphics (#337) participate in the connectivity ANALYSIS above
        # (a routed stub may genuinely continue through one) but are immutable
        # input art: force-keep any the pruner selected (the writer has no
        # (segment) block to strip anyway).
        g = [x for x in removed if getattr(x, 'graphic', False)]
        if g:
            kept = list(kept) + g
            removed = [x for x in removed if not getattr(x, 'graphic', False)]
        kept_segs_by_net[net_id] = kept
        for s in removed:
            if id(s) in routed_seg_ids:
                removed_routed_ids.add(id(s))
            else:
                original_to_remove.append(s)

    if removed_routed_ids:
        for r in results:
            segs = r.get('new_segments')
            if segs:
                r['new_segments'] = [s for s in segs if id(s) not in removed_routed_ids]

    # Drop routed vias left unsupported by the pruning: no kept same-net segment
    # endpoint and no pad lands on them. Original vias are left in place (their
    # dead-end segment, if any, would have anchored on them and not been removed).
    removed_via_ids = set()
    for net_id, kept in kept_segs_by_net.items():
        pad_pts = []
        for p in pcb_data.pads_by_net.get(net_id, []):
            px = getattr(p, 'global_x', getattr(p, 'x', 0.0))
            py = getattr(p, 'global_y', getattr(p, 'y', 0.0))
            psize = max(getattr(p, 'size_x', 0.5), getattr(p, 'size_y', 0.5))
            pad_pts.append((px, py, psize))
        endpoints = []
        for s in kept:
            endpoints.append((s.start_x, s.start_y))
            endpoints.append((s.end_x, s.end_y))
        for r in results:
            for v in r.get('new_vias') or []:
                if v.net_id != net_id or id(v) in removed_via_ids:
                    continue
                # PHYSICAL-attach test, not endpoint coincidence (#320): a
                # segment lands on a via when its endpoint is inside the via
                # barrel copper (radius), same as the pad test below uses the
                # pad size. Grading this at the 0.02 chaining tol trimmed a
                # LOAD-BEARING via whose track endpoint sat ~30um off-center
                # (glasgow Z0: via + 7 segs dropped, pad stranded).
                _v_reach = max(getattr(v, 'size', 0.0) / 2.0, 0.05)
                supported = any(math.hypot(v.x - ex, v.y - ey) < _v_reach for ex, ey in endpoints) \
                    or any(math.hypot(v.x - px, v.y - py) < ps / 2 + 0.05 for px, py, ps in pad_pts)
                if not supported:
                    removed_via_ids.add(id(v))
    if removed_via_ids:
        # The physical-attach heuristic is a proxy and has dropped a LOAD-
        # BEARING via before (glasgow Z0, patched by widening reach -- but any
        # endpoint past _v_reach still trips it). VERIFY per net with the
        # authoritative connectivity check, exactly like prune_redundant_cycles:
        # if dropping a net's "unsupported" vias splits it or strands a pad,
        # keep that net's vias (a floating via is cosmetic; a broken net is
        # not, #329 audit).
        from check_connected import check_net_connectivity
        for net_id, kept in kept_segs_by_net.items():
            net_vias_all, _seen = [], set()
            for v in [v for v in pcb_data.vias if v.net_id == net_id] + \
                     [v for r in results for v in (r.get('new_vias') or []) if v.net_id == net_id]:
                if id(v) not in _seen:
                    _seen.add(id(v))
                    net_vias_all.append(v)
            drop = [v for v in net_vias_all if id(v) in removed_via_ids]
            if not drop:
                continue
            pads = pcb_data.pads_by_net.get(net_id, [])
            zones = [z for z in all_zones if z.net_id == net_id]
            before = check_net_connectivity(net_id, kept, net_vias_all, pads, zones)
            keep_v = [v for v in net_vias_all if id(v) not in removed_via_ids]
            after = check_net_connectivity(net_id, kept, keep_v, pads, zones)
            if (before.get('connected') and not after.get('connected')) or \
               len(after.get('disconnected_pads') or []) > len(before.get('disconnected_pads') or []) or \
               (after.get('num_components') or 1) > (before.get('num_components') or 1):
                for v in drop:
                    removed_via_ids.discard(id(v))
        for r in results:
            vias = r.get('new_vias')
            if vias:
                r['new_vias'] = [v for v in vias if id(v) not in removed_via_ids]

    # Uniform mutation contract (#319): mirror the removals into pcb_data so
    # that after this pass -- like after every other cleanup pass -- pcb_data IS
    # the board that will be written, and everything downstream
    # (close_soft_joints' endpoint degrees, the board-vs-file ledger) reads
    # truth instead of pre-sweep fiction. Two failure paths were found here
    # (the glasgow_revC B1 regression) and are both fixed at the source:
    #   (1) the #220/#284 stale-input strip read live pcb_data as "the final
    #       board" and over-removed load-bearing input copper -- it now
    #       references a frozen, object-copied pre-cleanup snapshot
    #       (route.py freeze hook in the cleanup pipeline);
    #   (2) close_soft_joints would bridge the gaps the sweep opened (glasgow:
    #       a 24.8um bridge on /IO_Banks/Z4_P), perturbing the board the next
    #       chain step starts from (rip-reroute butterfly) -- now prevented at
    #       the source: the generalized _restore_soft_joint_bridges guard above
    #       restores ANY removal that would create a new soft joint (including
    #       the neighbour-dangle shape that caused B1), so the sweep cannot
    #       open a gap for close to see in the first place.
    orig_ids = {id(s) for s in original_to_remove}
    if removed_routed_ids or orig_ids:
        pcb_data.segments = [s for s in pcb_data.segments
                             if id(s) not in removed_routed_ids and id(s) not in orig_ids]
    if removed_via_ids:
        pcb_data.vias = [v for v in pcb_data.vias if id(v) not in removed_via_ids]

    segs_removed = len(removed_routed_ids) + len(original_to_remove)
    return segs_removed, len(removed_via_ids), original_to_remove


def remove_orphan_islands(results, pcb_data: PCBData, scope_net_ids=None
                          ) -> Tuple[int, int, List[Segment]]:
    """Remove same-net track-copper components that reach NO pad of the net
    (#217 orphan-island class): dead copper stranded by rip/reroute churn
    (hackrf VREGMODE: 4 segments / 2.84mm connected to nothing).

    Detection rides check_net_connectivity's own graph -- vias, T-junctions,
    cap overlap, and zone-outline membership all count as connections -- so a
    removed island is one the authoritative model calls pad-less. Removing it
    cannot change any pad's connectivity by construction (maximal component
    with no pad in it). Components containing graphics copper (#337 immutable
    art) are skipped whole. Nets with no pads at all are left alone.

    Returns (islands_removed, segments_removed, original_segments_to_strip);
    this-run segments are dropped from their result's write-list in place,
    original input segments are returned for the writer's strip list. The
    freed this-run vias are dropped by the sweep_dead_ends unsupported-via
    pass that runs right after this one.
    """
    from collections import defaultdict
    from check_connected import check_net_connectivity
    from geometry_utils import UnionFind

    seg_owner = {}
    for r in results:
        for s in r.get('new_segments') or []:
            seg_owner[id(s)] = r

    net_ids = {s.net_id for s in pcb_data.segments
               if scope_net_ids is None or s.net_id in scope_net_ids}
    net_ids.discard(0)

    islands_removed = 0
    removed_ids = set()
    originals: List[Segment] = []
    for net_id in net_ids:
        pads = pcb_data.pads_by_net.get(net_id, [])
        if not pads:
            continue
        net_segs = [s for s in pcb_data.segments if s.net_id == net_id]
        net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
        net_zones = [z for z in (getattr(pcb_data, 'zones', []) or [])
                     if z.net_id == net_id]
        if not net_segs:
            continue
        r = check_net_connectivity(net_id, net_segs, net_vias, pads,
                                   net_zones, return_graph=True)
        graph = r.get('graph')
        if not graph:
            continue
        uf = UnionFind()
        for a, b in graph.get('edges', []):
            uf.union(a, b)
        pad_roots = {uf.find(rep)
                     for rep in graph.get('pad_index_repr', {}).values()}
        comp_segs = defaultdict(list)
        for i, s in enumerate(net_segs):
            comp_segs[uf.find(2 * i)].append(s)
        for root, segs in comp_segs.items():
            if root in pad_roots:
                continue
            if any(getattr(s, 'graphic', False) for s in segs):
                continue  # immutable input art anchors the island
            islands_removed += 1
            for s in segs:
                removed_ids.add(id(s))
                if id(s) in seg_owner:
                    pass  # dropped from the write-list below
                else:
                    originals.append(s)

    if not removed_ids:
        return 0, 0, []
    for r in results:
        segs = r.get('new_segments')
        if segs:
            r['new_segments'] = [s for s in segs if id(s) not in removed_ids]
    pcb_data.segments = [s for s in pcb_data.segments
                         if id(s) not in removed_ids]
    return islands_removed, len(removed_ids), originals


def trim_dangles_past_body_anchor(results, pcb_data: PCBData, scope_net_ids=None,
                                  tol: float = None) -> Tuple[int, List[Segment]]:
    """Shorten a dead-end segment back to the LAST same-net anchor on its BODY
    (#347, core1106 CLK1P tail).

    sweep_dead_ends works at SEGMENT granularity: a segment whose free end
    dangles but whose body is T-anchored mid-span (a via sits ON the trace, or
    another trace tees into it) is load-bearing THROUGH the anchor, so the
    whole-segment prune correctly keeps it -- and the copper past the anchor
    ships as a dangling antenna (a partial-restore kept piece that a reconnect
    joined mid-body). The correct cleanup is a split: trim the free end back
    to the anchor point.

    Only trims when the free end has degree 1 and is itself unanchored (the
    same tests the pruner uses), and only back to a same-net via on the body
    or another same-net segment endpoint teeing into the body. In-run
    segments are shortened in place; original input segments are replaced
    (old one returned for the writer's strip list, shortened copy appended to
    ``results`` as cleanup copper). Returns (n_trimmed, originals_to_strip).
    """
    from collections import defaultdict
    if tol is None:
        from connectivity import COINCIDENCE_TOL
        tol = COINCIDENCE_TOL

    routed_seg_ids = set()
    for r in results:
        for s in r.get('new_segments') or []:
            routed_seg_ids.add(id(s))

    def key(x, y, layer):
        return (round(x, 3), round(y, 3), layer)

    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        if (scope_net_ids is None or s.net_id in scope_net_ids) \
                and not getattr(s, 'graphic', False):
            segs_by_net[s.net_id].append(s)

    n_trimmed = 0
    originals_to_strip: List[Segment] = []
    replacements: List[Segment] = []
    _CELL = 1.0
    for net_id, net_segs in segs_by_net.items():
        vias = [v for v in pcb_data.vias if v.net_id == net_id]
        via_pts = [(v.x, v.y, getattr(v, 'size', 0.6)) for v in vias]
        pad_pts = []
        for p in pcb_data.pads_by_net.get(net_id, []):
            px = getattr(p, 'global_x', getattr(p, 'x', 0.0))
            py = getattr(p, 'global_y', getattr(p, 'y', 0.0))
            psize = max(getattr(p, 'size_x', 0.5), getattr(p, 'size_y', 0.5))
            pad_pts.append((px, py, psize, getattr(p, 'layers', [])))
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

        for s in list(net_segs):
            dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
            L2 = dx * dx + dy * dy
            if L2 < 1e-9:
                continue
            for free_is_start in (True, False):
                fx, fy = (s.start_x, s.start_y) if free_is_start else (s.end_x, s.end_y)
                if degree[key(fx, fy, s.layer)] != 1:
                    continue
                if _point_anchored(fx, fy, s.layer, via_pts, pad_pts,
                                   seg_index, _CELL, s, tol):
                    continue
                # Anchors on the body: same-net via barrels overlapping the
                # centerline, and other same-net segments' endpoints teeing in.
                # t is measured from start; pick the anchor NEAREST the free
                # end (minimal trim keeps everything through-connected).
                cands = []  # t values
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
                if not cands:
                    continue
                t_anchor = min(cands) if free_is_start else max(cands)
                nx, ny = s.start_x + t_anchor * dx, s.start_y + t_anchor * dy
                tail_len = math.hypot(fx - nx, fy - ny)
                if tail_len <= max(tol, 3 * s.width):
                    continue  # sub-visible nib; not worth churn
                if id(s) in routed_seg_ids:
                    if free_is_start:
                        s.start_x, s.start_y = nx, ny
                    else:
                        s.end_x, s.end_y = nx, ny
                else:
                    trimmed = Segment(
                        start_x=(nx if free_is_start else s.start_x),
                        start_y=(ny if free_is_start else s.start_y),
                        end_x=(s.end_x if free_is_start else nx),
                        end_y=(s.end_y if free_is_start else ny),
                        width=s.width, layer=s.layer, net_id=s.net_id)
                    originals_to_strip.append(s)
                    replacements.append(trimmed)
                    pcb_data.segments = [x for x in pcb_data.segments if x is not s]
                    pcb_data.segments.append(trimmed)
                n_trimmed += 1
                break  # one trim per segment is enough per pass

    if replacements:
        results.append({'new_segments': replacements, 'new_vias': [],
                        'cleanup': 'dangle_trim'})
    return n_trimmed, originals_to_strip


def _pt_seg_dist(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Shortest distance from point (px,py) to segment (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def neck_wide_segments_grazing_pads(results, pcb_data, config) -> int:
    """Neck any routed segment wider than its layer default that VIOLATES clearance
    with a foreign-net pad on its layer.

    A wide power trunk that ROUTES SUCCESSFULLY at full width (necked_down=False,
    so the routing-time neck-down never runs) keeps its full width into a fanout
    via-in-pad terminal; the router exempts the terminal region, so the wide copper
    overlaps the neighbouring foreign pad on a fine-pitch part (VSYS->U7.D1 shorting
    GND pad U7.C1). Necking the offending segment to the layer default restores
    clearance without moving the centreline, so connectivity is preserved.

    Only segments that (a) violate at full width AND (b) clear at the default width
    are necked -- a legitimately-clear wide trunk is left alone, and a violation
    necking can't fix is left for the DRC report. Returns the count necked.
    """
    from net_queries import expand_pad_layers
    from collections import defaultdict
    pads_by_layer = defaultdict(list)
    for fp in pcb_data.footprints.values():
        for pad in fp.pads:
            for layer in expand_pad_layers(pad.layers, config.layers):
                pads_by_layer[layer].append(pad)
    clr = config.clearance
    necked = 0
    for r in results:
        for seg in r.get('new_segments', []):
            default_w = config.get_track_width(seg.layer)
            if seg.width <= default_w + 1e-9:
                continue
            for pad in pads_by_layer.get(seg.layer, []):
                if pad.net_id == seg.net_id or pad.net_id == 0:
                    continue
                d = _pt_seg_dist(pad.global_x, pad.global_y,
                                 seg.start_x, seg.start_y, seg.end_x, seg.end_y)
                # Bounding-circle pad half (conservative: never misses a violation).
                pad_half = max(pad.size_x, pad.size_y) / 2.0
                if (d - pad_half - seg.width / 2.0 < clr
                        and d - pad_half - default_w / 2.0 >= clr):
                    seg.width = default_w
                    necked += 1
                    break
    return necked


def _prune_net_cycles(net_id: int, net_segs: List[Segment], net_vias, net_pads,
                      fgrid, fcell: float, fmax_rad: float, clearance: float):
    """Reduce one net's routed copper to a spanning tree (forest if split).

    Builds a spanning tree by union-find over the segments, so every segment that
    would close a cycle (endpoints already connected) is REDUNDANT and removed,
    while every structural (bridge) segment is kept -- connectivity is preserved
    exactly. Nodes are keyed by (x, y, layer); vias and through-hole pads join the
    layer-nodes at their location (a via/TH pad connects all copper there), so
    cross-layer connectivity is modelled and inter-layer loops are found.

    Segments are processed non-grazing-and-short first, so a segment that grazes
    foreign copper (within ``clearance``) is the one left as the redundant cycle
    edge and dropped (removing a graze that sits on a loop, e.g. the RAM_A9 short,
    for free). Returns (kept, removed)."""
    if len(net_segs) < 3:
        return net_segs, []
    from connectivity import COINCIDENCE_TOL
    tol = COINCIDENCE_TOL  # THE endpoint coincidence tolerance (#320)

    def grazes(s):
        # Query only the foreign copper whose CENTRE could lie within
        # (rad + hw + clearance) of the segment; fmax_rad bounds the unknown per-item
        # rad so the exact circle test below still sees every real graze. Formerly an
        # O(all pads+vias) scan per segment -- the cycle prune's dominant cost.
        hw = s.width / 2.0
        margin = fmax_rad + hw + clearance
        lo_x = int((min(s.start_x, s.end_x) - margin) // fcell)
        hi_x = int((max(s.start_x, s.end_x) + margin) // fcell)
        lo_y = int((min(s.start_y, s.end_y) - margin) // fcell)
        hi_y = int((max(s.start_y, s.end_y) + margin) // fcell)
        for gx in range(lo_x, hi_x + 1):
            for gy in range(lo_y, hi_y + 1):
                for cx, cy, rad, n, layers in fgrid.get((gx, gy), ()):
                    if n == net_id:
                        continue
                    if layers is not None and s.layer not in layers:
                        continue
                    if _pt_seg_dist(cx, cy, s.start_x, s.start_y, s.end_x, s.end_y) < rad + hw + clearance:
                        return True
        return False

    # --- Phase 1: cluster segment endpoints into NODES (real connectivity) ---
    # Each segment contributes two "ports" (its endpoints). Ports coincide (same
    # node) when they match on the same layer, OR are bridged across layers by a
    # via / through-hole pad (joined by its copper size, like KiCad). This mirrors
    # check_net_connectivity so the via-pad-to-trace touch that exact-match misses
    # is captured -- without it the net looks split and loops are missed.
    ports = []  # (x, y, layer, seg_index, end 0/1)
    for i, s in enumerate(net_segs):
        ports.append((s.start_x, s.start_y, s.layer, i, 0))
        ports.append((s.end_x, s.end_y, s.layer, i, 1))

    pp = list(range(len(ports)))

    def pfind(x):
        while pp[x] != x:
            pp[x] = pp[pp[x]]
            x = pp[x]
        return x

    def punion(a, b):
        ra, rb = pfind(a), pfind(b)
        if ra != rb:
            pp[ra] = rb

    n = len(ports)
    # Same-layer coincidence via the ONE shared primitive (#320) -- replaces
    # an O(n^2) pairwise loop with spatial hashing, same tolerance.
    from connectivity import cluster_coincident_points
    _roots = cluster_coincident_points([(p[0], p[1], p[2]) for p in ports], tol)
    _first_in_cluster = {}
    for a in range(n):
        r = _roots[a]
        if r in _first_in_cluster:
            punion(_first_in_cluster[r], a)
        else:
            _first_in_cluster[r] = a

    # Vias and through-hole pads bridge layers: union all ports within the
    # connector's copper reach (size/4, >= tol), regardless of layer.
    def join_near(cx, cy, reach):
        near = [i for i in range(n) if math.hypot(ports[i][0] - cx, ports[i][1] - cy) < reach]
        for j in near[1:]:
            punion(near[0], j)

    for v in (net_vias or []):
        join_near(v.x, v.y, max(getattr(v, 'size', 0.6) / 4.0, tol))
    for pad in (net_pads or []):
        if getattr(pad, 'drill', 0) and pad.drill > 0:
            reach = max(max(pad.size_x, pad.size_y) / 4.0, tol)
            join_near(getattr(pad, 'global_x', 0.0), getattr(pad, 'global_y', 0.0), reach)

    # --- Phase 2: T-junction-aware spanning tree; redundant segments removed ---
    # A segment "touches" its two endpoint clusters AND any cluster that lies on
    # its INTERIOR (a T-junction). A segment running collinear on top of another
    # lands on the other's interior, so overlapping copper is caught the same way.
    # Keeping a segment connects every node it touches; a segment all of whose
    # touched nodes are already connected adds no connectivity -- it is a loop /
    # overlap and is removed. Processed non-grazing-and-short first so a grazing or
    # overlapping segment is the redundant one dropped.
    from collections import defaultdict
    from check_connected import point_on_segment, points_match

    reps = {}
    rep_layers = defaultdict(set)
    for i in range(n):
        r = pfind(i)
        reps.setdefault(r, (ports[i][0], ports[i][1]))
        rep_layers[r].add(ports[i][2])
    rep_items = list(reps.items())

    touched = []
    for i, s in enumerate(net_segs):
        ra, rb = pfind(2 * i), pfind(2 * i + 1)
        nodes = {ra, rb}
        if ra != rb:
            for r, (cx, cy) in rep_items:
                if r == ra or r == rb or s.layer not in rep_layers[r]:
                    continue
                if point_on_segment(cx, cy, s.start_x, s.start_y, s.end_x, s.end_y, tol) \
                   and not points_match(cx, cy, s.start_x, s.start_y, tol) \
                   and not points_match(cx, cy, s.end_x, s.end_y, tol):
                    nodes.add(r)
        touched.append(nodes)

    cpar = {r: r for r in reps}

    def cfind(x):
        while cpar[x] != x:
            cpar[x] = cpar[cpar[x]]
            x = cpar[x]
        return x

    order = sorted(range(len(net_segs)),
                   key=lambda i: (grazes(net_segs[i]),
                                  math.hypot(net_segs[i].end_x - net_segs[i].start_x,
                                             net_segs[i].end_y - net_segs[i].start_y)))
    kept, removed = [], []
    for i in order:
        roots = {cfind(r) for r in touched[i]}
        if len(roots) <= 1:
            removed.append(net_segs[i])  # adds no new connectivity -> redundant loop/overlap
        else:
            base = next(iter(touched[i]))
            for r in touched[i]:
                ra, rb = cfind(base), cfind(r)
                if ra != rb:
                    cpar[ra] = rb
            kept.append(net_segs[i])

    if not removed:
        return kept, []
    # Validate each PROPOSED removal against the authoritative connectivity oracle.
    # The clustering above can over-merge (its tolerances differ from
    # check_connected's), so a proposed-redundant segment may actually be
    # load-bearing; checking each removal guarantees we never split the net. Drop
    # grazing, then longer, candidates first.
    from check_connected import check_net_connectivity
    base = check_net_connectivity(net_id, net_segs, net_vias, net_pads)
    if base.get('connected') is False:
        return net_segs, []
    base_comps = base.get('num_components') or 1
    base_disc = len(base.get('disconnected_pads') or [])
    # Pad coverage points, mirroring sweep_dead_ends' via-support model exactly.
    pad_cover = []
    for p in (net_pads or []):
        px = getattr(p, 'global_x', getattr(p, 'x', 0.0))
        py = getattr(p, 'global_y', getattr(p, 'y', 0.0))
        ps = max(getattr(p, 'size_x', 0.5), getattr(p, 'size_y', 0.5))
        pad_cover.append((px, py, ps))

    def supported_vias(seglist):
        # A via survives sweep_dead_ends only if a kept same-net segment endpoint
        # lands within 0.05mm of it, or a pad covers it (see sweep_dead_ends).
        out = set()
        for i, v in enumerate(net_vias or []):
            if any(math.hypot(v.x - sg.start_x, v.y - sg.start_y) < 0.05 or
                   math.hypot(v.x - sg.end_x, v.y - sg.end_y) < 0.05 for sg in seglist) \
               or any(math.hypot(v.x - cx, v.y - cy) < cs / 2 + 0.05
                      for cx, cy, cs in pad_cover):
                out.add(i)
        return out

    cur = list(net_segs)
    cur_supported = supported_vias(cur)
    safe_removed = []
    for s in sorted(removed, key=lambda s: (not grazes(s),
                    -math.hypot(s.end_x - s.start_x, s.end_y - s.start_y))):
        trial = [x for x in cur if x is not s]
        t = check_net_connectivity(net_id, trial, net_vias, net_pads)
        if not (t.get('connected') and (t.get('num_components') or 1) <= base_comps
                and len(t.get('disconnected_pads') or []) <= base_disc):
            continue
        # Don't strip the last segment anchoring a via: check_net_connectivity
        # credits via-copper overlap and reports "still connected", but
        # sweep_dead_ends would then cull the now-unsupported via and disconnect
        # the net (issue #209). Reject any removal that drops a via's support.
        trial_supported = supported_vias(trial)
        if trial_supported < cur_supported:
            continue
        cur = trial
        cur_supported = trial_supported
        safe_removed.append(s)
    return cur, safe_removed


def prune_redundant_cycles(results, pcb_data: PCBData, scope_net_ids=None,
                           clearance: float = 0.1) -> Tuple[int, int, List[Segment]]:
    """Enforce the per-net TREE invariant: remove redundant cycle edges (the cycle
    analog of sweep_dead_ends).

    A multipoint net is routed as an MST (a tree on pads), but the incremental
    repair layer -- rip+restore and failed-edge retry -- re-adds obstacle-exempt
    same-net copper to reconnect pads WITHOUT enforcing acyclicity, so cycles
    accumulate (e.g. RAM_A9: 3 loops / 27 segments for a 3-pad net, with the short
    sitting on a loop edge). This breaks every cycle by dropping a redundant
    (non-bridge) segment, keeping all pads/vias connected, preferring to drop one
    that grazes foreign copper. Nets with a copper pour/zone are skipped (planes
    are meshes, not trees). Mirrors sweep_dead_ends' write-list sync; also drops
    removed routed copper from pcb_data so the later passes see the tree.

    Returns (segments_removed, nets_pruned, original_segments_to_remove)."""
    from collections import defaultdict

    routed_seg_ids = set()
    for r in results:
        for s in r.get('new_segments') or []:
            routed_seg_ids.add(id(s))

    # Foreign copper (other nets' pads + vias), built once for the grazing test.
    copper = set(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    foreign = []
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            rad = max(p.size_x, p.size_y) / 2.0
            if rad <= 0:
                continue
            if p.drill and p.drill > 0:
                layers = None
            else:
                pl = set(p.layers or [])
                on = frozenset(l for l in pl if l in copper)
                layers = None if any(l == '*.Cu' for l in pl) else (on or None)
            foreign.append((p.global_x, p.global_y, rad, p.net_id, layers))
    for v in pcb_data.vias:
        foreign.append((v.x, v.y, v.size / 2.0, v.net_id, None))

    # Spatial index over foreign copper (bucketed by centre cell) so the per-segment
    # grazing-order test in _prune_net_cycles is local, not O(all pads+vias).
    _FCELL = 1.0
    fmax_rad = max((it[2] for it in foreign), default=0.0)
    fgrid = defaultdict(list)
    for it in foreign:
        fgrid[(int(it[0] // _FCELL), int(it[1] // _FCELL))].append(it)

    zoned_nets = {z.net_id for z in (getattr(pcb_data, 'zones', []) or [])}
    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        if scope_net_ids is None or s.net_id in scope_net_ids:
            if getattr(s, 'graphic', False):
                continue  # copper graphics are immutable input art (#337)
            segs_by_net[s.net_id].append(s)

    removed_routed_ids = set()
    original_to_remove = []
    nets_pruned = 0
    from check_connected import check_net_connectivity

    vias_by_net = defaultdict(list)
    for v in pcb_data.vias:
        vias_by_net[v.net_id].append(v)
    for net_id, net_segs in segs_by_net.items():
        if net_id in zoned_nets:  # planes / pours are meshes, not trees
            continue
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        kept, removed = _prune_net_cycles(net_id, net_segs, net_vias,
                                          net_pads, fgrid, _FCELL, fmax_rad, clearance)
        # #319: never break a "loop" whose alternate path is only a soft joint.
        kept, removed = _restore_soft_joint_bridges(kept, removed, net_vias, net_pads)
        if not removed:
            continue
        # Safety: the cycle model uses tolerance clustering which can imperfectly
        # merge nodes -- so VERIFY with the authoritative connectivity check that
        # the prune did not split the net or strand a pad; if it did, revert this
        # net (drop nothing). The pass can then only ever remove truly-redundant
        # copper.
        before = check_net_connectivity(net_id, net_segs, net_vias, net_pads)
        after = check_net_connectivity(net_id, kept, net_vias, net_pads)
        if (before.get('connected') and not after.get('connected')) or \
           len(after.get('disconnected_pads') or []) > len(before.get('disconnected_pads') or []) or \
           (after.get('num_components') or 1) > (before.get('num_components') or 1):
            continue  # revert: keep all of this net's copper
        nets_pruned += 1
        for s in removed:
            if id(s) in routed_seg_ids:
                removed_routed_ids.add(id(s))
            else:
                original_to_remove.append(s)

    if removed_routed_ids:
        for r in results:
            segs = r.get('new_segments')
            if segs:
                r['new_segments'] = [s for s in segs if id(s) not in removed_routed_ids]
    if removed_routed_ids or original_to_remove:
        orig_ids = {id(s) for s in original_to_remove}
        pcb_data.segments = [s for s in pcb_data.segments
                             if id(s) not in removed_routed_ids and id(s) not in orig_ids]

    return len(removed_routed_ids) + len(original_to_remove), nets_pruned, original_to_remove


def _seg_seg_min_dist(ax, ay, bx, by, cx, cy, dx, dy) -> float:
    """Minimum Euclidean distance between segments AB and CD (endpoint sampling,
    exact for the non-crossing case a clearance test cares about; crossing -> ~0
    which still flags)."""
    def pt_seg(px, py, x1, y1, x2, y2):
        vx, vy = x2 - x1, y2 - y1
        l2 = vx * vx + vy * vy
        if l2 <= 0.0:
            return math.hypot(px - x1, py - y1)
        t = ((px - x1) * vx + (py - y1) * vy) / l2
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
        return math.hypot(px - (x1 + t * vx), py - (y1 + t * vy))
    return min(pt_seg(ax, ay, cx, cy, dx, dy), pt_seg(bx, by, cx, cy, dx, dy),
               pt_seg(cx, cy, ax, ay, bx, by), pt_seg(dx, dy, ax, ay, bx, by))


def prune_grazing_segments(results, pcb_data: PCBData, scope_net_ids=None,
                           clearance: float = 0.1,
                           check_foreign_segments: bool = False) -> Tuple[int, int, List[Segment]]:
    """Drop a segment that grazes a FOREIGN pad/via below clearance when the net
    stays fully connected without it (issue #224).

    With ``check_foreign_segments`` (diff-pair cleanup, #215) a segment that grazes
    another NET's track below clearance is also a candidate -- e.g. a redundant P
    connector overshoot that pokes within clearance of the partner N track. Same
    connectivity gate, so a load-bearing track (the partner's own through-run) is
    kept; only the redundant overshoot is dropped.

    The router lays a terminal/tap segment toward its own pad/via through the
    obstacle-exempt endpoint region, so at tight connector / fine pad pitch it can
    sit sub-clearance to a NEIGHBOURING foreign pad -- a route.py launch jog, a
    route_disconnected_planes tap. `_neck_terminal_grazes` only narrows such a
    segment and is floored at the fab track minimum, so a sub-floor graze survives
    to DRC. But these grazing segments are frequently a REDUNDANT detour/appendix:
    the adjacent (wider) copper already overlaps enough to carry the connection, so
    dropping the grazing segment leaves the net fully connected and removes the
    violation outright (e.g. ottercast Net-(R81-Pad1)'s tab poking at the C3 pad).

    A candidate is removed only when the AUTHORITATIVE copper-width-aware
    check_net_connectivity confirms the net is no worse connected without it -- a
    load-bearing graze (e.g. a plane tap that is a pad's sole connection to the
    pour) is kept and left for DRC. Zoned nets are checked WITH their pour, so a
    tap that the fill makes redundant can still be dropped. Mirrors
    prune_redundant_cycles' write-list / pcb_data sync.

    Returns (segments_removed, nets_pruned, original_segments_to_remove)."""
    from collections import defaultdict
    from check_connected import check_net_connectivity, analyze_conn_excluding
    # Accurate (rect-edge, windowed) foreign-pad distance -- the same one the router's
    # terminal-neck uses, so "grazes" matches what DRC flags. The circle model in
    # _prune_net_cycles.grazes only ORDERS cycle-edge drops, but here grazing GATES
    # removal, so an over-approximation would delete legitimate non-violating copper.
    from single_ended_routing import _seg_foreign_pad_dist, _seg_foreign_via_dist

    routed_seg_ids = set()
    for r in results:
        for s in r.get('new_segments') or []:
            routed_seg_ids.add(id(s))

    # Foreign-segment spatial index (only when segment grazing is requested): a
    # uniform grid keyed by (layer, cell_x, cell_y), each segment bucketed into the
    # cells its bounding box covers. Without it, grazes() scanned EVERY same-layer
    # segment per query -- O(scope x layer), the dominant cost on dense boards
    # (~3s of the graze pass on an 12k-segment board). The grid bounds each query to
    # local density. Cell is a few clearances wide so a short track sits in ~1 cell;
    # the query widens by one cell so a graze margin (< cell) can't fall through.
    _CELL = 1.0
    seg_grid = defaultdict(list)
    if check_foreign_segments:
        for o in pcb_data.segments:
            olo_x = int(min(o.start_x, o.end_x) // _CELL); ohi_x = int(max(o.start_x, o.end_x) // _CELL)
            olo_y = int(min(o.start_y, o.end_y) // _CELL); ohi_y = int(max(o.start_y, o.end_y) // _CELL)
            for cx in range(olo_x, ohi_x + 1):
                for cy in range(olo_y, ohi_y + 1):
                    seg_grid[(o.layer, cx, cy)].append(o)

    def grazes(s):
        thr = clearance + s.width / 2.0 - 1e-4
        if (_seg_foreign_pad_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                  s.end_x, s.end_y, s.layer) < thr or
                _seg_foreign_via_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                      s.end_x, s.end_y, s.layer) < thr):
            return True
        if check_foreign_segments:
            slo_x, shi_x = min(s.start_x, s.end_x), max(s.start_x, s.end_x)
            slo_y, shi_y = min(s.start_y, s.end_y), max(s.start_y, s.end_y)
            cx0 = int(slo_x // _CELL) - 1; cx1 = int(shi_x // _CELL) + 1
            cy0 = int(slo_y // _CELL) - 1; cy1 = int(shi_y // _CELL) + 1
            seen = set()
            for cx in range(cx0, cx1 + 1):
                for cy in range(cy0, cy1 + 1):
                    for o in seg_grid.get((s.layer, cx, cy), ()):
                        if o.net_id == s.net_id or id(o) in seen:
                            continue
                        seen.add(id(o))
                        margin = clearance + (s.width + o.width) / 2.0
                        if (max(o.start_x, o.end_x) < slo_x - margin or min(o.start_x, o.end_x) > shi_x + margin or
                                max(o.start_y, o.end_y) < slo_y - margin or min(o.start_y, o.end_y) > shi_y + margin):
                            continue  # bbox prefilter
                        d = _seg_seg_min_dist(s.start_x, s.start_y, s.end_x, s.end_y,
                                              o.start_x, o.start_y, o.end_x, o.end_y)
                        if d - (s.width + o.width) / 2.0 < clearance - 1e-4:
                            return True
        return False

    zones_by_net = defaultdict(list)
    for z in (getattr(pcb_data, 'zones', []) or []):
        zones_by_net[z.net_id].append(z)
    vias_by_net = defaultdict(list)
    for v in pcb_data.vias:
        vias_by_net[v.net_id].append(v)
    # Dedupe by object identity: a segment referenced TWICE in pcb_data.segments
    # would become two graph nodes, and excluding one copy leaves its twin
    # carrying the connection -- every removal then looks safe while the
    # id-based application deletes both entries and guts the net (#195).
    segs_by_net = defaultdict(list)
    _seen_ids = set()
    for s in pcb_data.segments:
        if scope_net_ids is None or s.net_id in scope_net_ids:
            if id(s) in _seen_ids:
                continue
            _seen_ids.add(id(s))
            segs_by_net[s.net_id].append(s)

    def worse(before, after):
        return ((before.get('connected') and not after.get('connected')) or
                len(after.get('disconnected_pads') or []) > len(before.get('disconnected_pads') or []) or
                (after.get('num_components') or 1) > (before.get('num_components') or 1))

    removed_routed_ids = set()
    original_to_remove = []
    nets_pruned = 0
    for net_id, net_segs in segs_by_net.items():
        grazing = [s for s in net_segs if grazes(s)]
        if not grazing:
            continue
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        net_zones = zones_by_net.get(net_id, [])
        # Build the connectivity graph ONCE, then test each candidate removal by
        # dropping that segment's edges instead of rebuilding the (expensive)
        # spatial graph per candidate -- O(net + G) instead of O(net x G) full-net
        # checks, the dominant cost of this cleanup on big plane nets (#263). Falls
        # back to per-candidate recompute if the graph is unavailable, and an
        # env-gated assertion (PRUNE_CONN_VERIFY=1) checks the fast path against a
        # real recompute on every trial during verification.
        before = check_net_connectivity(net_id, net_segs, net_vias, net_pads,
                                        net_zones, return_graph=True)
        graph = before.get('graph')
        # Strict twin (#322): see _STRICT_GATE_WIDTH. A mid-chain removal whose
        # hole is lens-bridged by fat caps passes the physical gate; the strict
        # gate sees the split immediately.
        before_strict, graph_strict = _strict_conn_graph(
            net_id, net_segs, net_vias, net_pads, net_zones)
        seg_pos = {id(s): i for i, s in enumerate(net_segs)}
        _verify = os.environ.get('PRUNE_CONN_VERIFY')
        dropped = []
        dropped_idx = set()
        # Shortest grazing segments first: an appendix tip / tap stub is the most
        # likely to be redundant, and dropping it can only help the longer ones.
        # Soft-joint-aware removal (#318): removing a COINCIDENT-BRIDGE segment
        # whose neighbours' caps overlap passes the overlap-connectivity gate
        # but manufactures a soft joint that close_soft_joints then cannot
        # bridge (the bridge would graze the same foreign copper the removed
        # segment did -- smartknob /STRAIN_S- 141um diagonal vs pad R5.2, the
        # glasgow diff-step jog vs the partner leg). For such a segment,
        # prefer NECKING it to clear the graze (fixes the violation AND keeps
        # the coincident chain); only remove when even the fab floor cannot
        # clear. Baseline joints (pre-existing) never block a removal.
        baseline_joints = _soft_joint_pairs(net_segs, net_vias, net_pads)
        for s in sorted(grazing, key=lambda s: math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)):
            trial_excl = dropped_idx | {seg_pos[id(s)]}
            if graph is not None:
                after = analyze_conn_excluding(graph, trial_excl)
                if _verify:
                    trial = [x for i, x in enumerate(net_segs) if i not in trial_excl]
                    ref = check_net_connectivity(net_id, trial, net_vias, net_pads, net_zones)
                    assert worse(before, after) == worse(before, ref), \
                        f"prune-conn fast-path mismatch: net {net_id}, seg {seg_pos[id(s)]}"
            else:
                trial = [x for i, x in enumerate(net_segs) if i not in trial_excl]
                after = check_net_connectivity(net_id, trial, net_vias, net_pads, net_zones)
            if worse(before, after):
                continue
            if graph_strict is not None and worse(
                    before_strict, analyze_conn_excluding(graph_strict, trial_excl)):
                continue  # #322: coincidence-level connectivity would worsen
            kept_after = [x for i, x in enumerate(net_segs) if i not in trial_excl]
            if _soft_joint_pairs(kept_after, net_vias, net_pads) - baseline_joints:
                # Removal would open a soft joint: neck to clear instead.
                from single_ended_routing import (_seg_foreign_pad_dist as _fpd,
                                                  _seg_foreign_via_dist as _fvd,
                                                  _fab_track_floor)
                d = min(_fpd(pcb_data, s.net_id, s.start_x, s.start_y,
                             s.end_x, s.end_y, s.layer),
                        _fvd(pcb_data, s.net_id, s.start_x, s.start_y,
                             s.end_x, s.end_y, s.layer))
                if check_foreign_segments:
                    for o in pcb_data.segments:
                        if o.net_id == s.net_id or o.layer != s.layer or o is s:
                            continue
                        dd = _seg_seg_min_dist(s.start_x, s.start_y, s.end_x, s.end_y,
                                               o.start_x, o.start_y, o.end_x, o.end_y) - o.width / 2.0
                        if dd < d:
                            d = dd
                allowed = 2.0 * (d - clearance - 1e-4)
                floor = _fab_track_floor(pcb_data)
                if (allowed >= floor - 1e-9 and allowed < s.width - 1e-9
                        and id(s) in routed_seg_ids):
                    # Necking mutates width in place, which only the writer's
                    # re-emit path can express -- so ROUTED segments only. An
                    # ORIGINAL input segment falls through to the defer path
                    # below: the octolinear/microshift passes have proper
                    # strip+replace plumbing for originals (in-place necking
                    # them drifted board vs file, and a strip+re-emit attempt
                    # here interacted badly with the later passes' results
                    # bookkeeping -- sechzig /DRAM_CK, /DRAM_LDQS_P).
                    #
                    # #322: overlap connectivity is WIDTH-DEPENDENT -- a neck
                    # can silently break a cap/T contact elsewhere on the
                    # chain (smartknob +5V: neighbour necked 0.3->0.22 opened
                    # a 0.28mm lens the physical gate had just relied on).
                    # Verify the necked net before committing; revert + defer
                    # to the nudges if connectivity would worsen.
                    _old_w = s.width
                    s.width = round(max(floor, allowed), 4)
                    _kept_now = [x for i, x in enumerate(net_segs)
                                 if i not in dropped_idx]
                    if worse(before, check_net_connectivity(
                            net_id, _kept_now, net_vias, net_pads, net_zones)):
                        s.width = _old_w  # neck would break a contact: defer
                        continue
                    continue  # necked clear; keep the coincident bridge
                if allowed >= s.width - 1e-9:
                    continue  # already clear at current width (stale cache)
                # Even the fab floor cannot clear it. Under the TIGHTENED
                # connectivity definition (#320 direction: cap overlap without
                # coincidence is NOT a connection), this segment is load-
                # bearing -- removing it would really disconnect the net. KEEP
                # it and let the downstream nudge passes (octolinear re-bend /
                # microshift) move it clear; they preserve coincident anchors
                # and are verified + connectivity-gated. If they also cannot
                # fix it, the graze ships as an honest DRC violation instead
                # of a masked near-open (smartknob /STRAIN_S- vs the rotated
                # J5.3 oval: shortfall 49um -- inside the microshift's cap).
                continue
            dropped_idx.add(seg_pos[id(s)])
            dropped.append(s)
        if not dropped:
            continue
        nets_pruned += 1
        for s in dropped:
            if id(s) in routed_seg_ids:
                removed_routed_ids.add(id(s))
            else:
                original_to_remove.append(s)

    if removed_routed_ids:
        for r in results:
            segs = r.get('new_segments')
            if segs:
                r['new_segments'] = [s for s in segs if id(s) not in removed_routed_ids]
    if removed_routed_ids or original_to_remove:
        orig_ids = {id(s) for s in original_to_remove}
        pcb_data.segments = [s for s in pcb_data.segments
                             if id(s) not in removed_routed_ids and id(s) not in orig_ids]

    return len(removed_routed_ids) + len(original_to_remove), nets_pruned, original_to_remove


def _octolinear_bends(A, B):
    """Candidate octolinear (45-degree) polylines from A to B: the direct segment
    (when A->B is already octolinear) and the two single-bend L-elbows (diagonal-
    then-orthogonal and orthogonal-then-diagonal). Each is returned as the list of
    INTERMEDIATE points ([] = direct)."""
    ax, ay = A
    bx, by = B
    dx, dy = bx - ax, by - ay
    adx, ady = abs(dx), abs(dy)
    sx = 1.0 if dx >= 0 else -1.0
    sy = 1.0 if dy >= 0 else -1.0
    out = []
    if adx < 1e-9 or ady < 1e-9 or abs(adx - ady) < 1e-6:
        out.append([])                                   # already octolinear
    if adx >= ady:
        out.append([(round(ax + sx * ady, 4), round(by, 4))])   # diag then horizontal
        out.append([(round(bx - sx * ady, 4), round(ay, 4))])   # horizontal then diag
    else:
        out.append([(round(bx, 4), round(ay + sy * adx, 4))])   # diag then vertical
        out.append([(round(ax, 4), round(by - sy * adx, 4))])   # vertical then diag
    return out


def nudge_grazing_octolinear(results, pcb_data: PCBData, scope_net_ids=None,
                             clearance: float = 0.1) -> Tuple[int, int, List[Segment]]:
    """Re-bend a foreign-pad-grazing octolinear jog so it clears the pad (issue #224).

    The complement to prune_grazing_segments: when a grazing segment is LOAD-BEARING
    (removing it would disconnect the net) it can't be dropped, but the little jog it
    forms can often be re-routed around the pad with a different octolinear bend that
    keeps the SAME two anchor endpoints -- so connectivity is untouched and only the
    poking corner moves (e.g. ottercast Net-(R81-Pad1): the 45-degree apex poking at
    the C3 pad becomes a 45-then-vertical bend that stays clear). All-45-degree
    geometry is preserved; the new segments are verified to clear every foreign pad
    AND track/via before they replace the old jog, and the net's connectivity is
    re-checked, so the pass can only ever remove a graze, never introduce one or
    disconnect a net. A jog with no clearing octolinear bend is left for DRC.

    Returns (segments_changed, nets_changed, original_segments_to_remove)."""
    from collections import defaultdict
    from check_connected import check_net_connectivity
    from single_ended_routing import _seg_foreign_pad_dist, _seg_foreign_seg_dist, _seg_foreign_via_dist

    routed_seg_result = {}
    for r in results:
        for s in r.get('new_segments') or []:
            routed_seg_result[id(s)] = r

    def grazes(s):
        thr = clearance + s.width / 2.0 - 1e-4
        return (_seg_foreign_pad_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                      s.end_x, s.end_y, s.layer) < thr or
                _seg_foreign_via_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                      s.end_x, s.end_y, s.layer) < thr)

    # A re-bent jog must also respect the board edge: the octolinear candidates
    # only clear FOREIGN COPPER, so a bend could otherwise be pushed off-board /
    # across an Edge.Cuts cutout that the original A* route legally skirted
    # (lily58 Net-(LED10-DIN): a dogleg re-bent 1mm INTO a switch cutout, #256).
    from check_drc import board_edge_geometry, _point_on_board, _segment_to_rings_distance
    edge_rings, edge_outer, edge_cutouts = board_edge_geometry(pcb_data.board_info)
    board_bounds = pcb_data.board_info.board_bounds

    def edge_clears(x1, y1, x2, y2, w):
        required = clearance + w / 2.0 - 1e-4
        if edge_rings:
            if not _point_on_board(x1, y1, edge_outer, edge_cutouts) or \
               not _point_on_board(x2, y2, edge_outer, edge_cutouts):
                return False
            return _segment_to_rings_distance(x1, y1, x2, y2, edge_rings) >= required
        if board_bounds:
            # Rectangular fallback: inside a rectangle the segment-to-boundary
            # minimum is attained at an endpoint.
            min_x, min_y, max_x, max_y = board_bounds
            return all(min(x - min_x, max_x - x, y - min_y, max_y - y) >= required
                       for x, y in ((x1, y1), (x2, y2)))
        return True

    def clears(x1, y1, x2, y2, layer, net_id, w):
        # Foreign VIAS must be checked too: grazes() fires on via proximity, so
        # omitting them here let a re-bend that fixed a pad graze land within
        # clearance of (or onto) a foreign via (#254 neo6502 /GPIO1 vs /GPIO2).
        d = min(_seg_foreign_pad_dist(pcb_data, net_id, x1, y1, x2, y2, layer),
                _seg_foreign_seg_dist(pcb_data, net_id, x1, y1, x2, y2, layer),
                _seg_foreign_via_dist(pcb_data, net_id, x1, y1, x2, y2, layer))
        return d >= clearance + w / 2.0 - 1e-4 and edge_clears(x1, y1, x2, y2, w)

    def vk(x, y):
        return (round(x, 3), round(y, 3))

    zones_by_net = defaultdict(list)
    for z in (getattr(pcb_data, 'zones', []) or []):
        zones_by_net[z.net_id].append(z)
    vias_by_net = defaultdict(list)
    for v in pcb_data.vias:
        vias_by_net[v.net_id].append(v)
    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        if scope_net_ids is None or s.net_id in scope_net_ids:
            segs_by_net[s.net_id].append(s)

    def worse(before, after):
        return ((before.get('connected') and not after.get('connected')) or
                len(after.get('disconnected_pads') or []) > len(before.get('disconnected_pads') or []) or
                (after.get('num_components') or 1) > (before.get('num_components') or 1))

    removed_ids = set()
    original_to_remove = []
    added_segments = []
    nets_changed = 0
    MAX_CHAIN = 5

    # Re-bending keeps a jog's two anchor endpoints fixed, so it preserves
    # connectivity on a plane mesh as much as on a signal net -- unlike a cycle
    # prune, it removes no structural edge. So zoned (plane) nets are NOT skipped;
    # their grazing taps (e.g. a GND tap pinched against a connector pad) get
    # re-bent too. The connectivity check is run WITH the net's pour.
    for net_id, net_segs in segs_by_net.items():
        grazing = [s for s in net_segs if grazes(s)]
        if not grazing:
            continue
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        net_zones = zones_by_net.get(net_id, [])
        before = check_net_connectivity(net_id, net_segs, net_vias, net_pads, net_zones)

        # Group the grazing segments into simple chains: a vertex touching exactly
        # one grazing segment is an ANCHOR (it ties into non-grazing copper / a
        # pad / a via and must not move); interior vertices touch two.
        gadj = defaultdict(list)
        for s in grazing:
            gadj[vk(s.start_x, s.start_y)].append(s)
            gadj[vk(s.end_x, s.end_y)].append(s)
        anchors = [v for v, ss in gadj.items() if len(ss) == 1]
        used = set()
        net_changed = False
        for start in anchors:
            seg0 = next((s for s in gadj[start] if id(s) not in used), None)
            if seg0 is None:
                continue
            # Walk the chain from this anchor to the next anchor / junction.
            chain = []
            cur = start
            ok_chain = True
            while True:
                nxt = [s for s in gadj[cur] if id(s) not in used]
                if not nxt:
                    break
                s = nxt[0]
                used.add(id(s))
                chain.append(s)
                other = vk(s.end_x, s.end_y) if vk(s.start_x, s.start_y) == cur else vk(s.start_x, s.start_y)
                cur = other
                if len(gadj[cur]) != 2:          # reached the far anchor / a junction
                    break
                if len(chain) > MAX_CHAIN:
                    ok_chain = False
                    break
            B = cur
            if not ok_chain or len(gadj[B]) > 2 or B == start:
                continue                          # branchy / loop -> skip
            A = start
            w = min(s.width for s in chain)
            layer = chain[0].layer
            if any(s.layer != layer for s in chain):
                continue                          # mixed-layer jog (has a via) -> skip
            # Try each octolinear reconnection; commit the first that clears.
            for inter in _octolinear_bends(A, B):
                pts = [A] + inter + [B]
                if all(clears(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], layer, net_id, w)
                       for i in range(len(pts) - 1)):
                    new = [Segment(start_x=pts[i][0], start_y=pts[i][1],
                                   end_x=pts[i + 1][0], end_y=pts[i + 1][1],
                                   width=w, layer=layer, net_id=net_id)
                           for i in range(len(pts) - 1)
                           if (pts[i][0], pts[i][1]) != (pts[i + 1][0], pts[i + 1][1])]
                    trial = [s for s in net_segs if s not in chain] + new
                    if worse(before, check_net_connectivity(net_id, trial, net_vias, net_pads, net_zones)):
                        continue
                    # Commit: drop the chain, splice in the new octolinear segments.
                    res = None
                    for s in chain:
                        if id(s) in routed_seg_result:
                            removed_ids.add(id(s))
                            res = res or routed_seg_result[id(s)]
                        else:
                            original_to_remove.append(s)
                    if res is None:
                        res = {'new_segments': [], 'new_vias': []}
                        results.append(res)
                    res['new_segments'] = list(res.get('new_segments') or []) + new
                    added_segments.extend(new)
                    net_segs = trial
                    net_changed = True
                    break
        if net_changed:
            nets_changed += 1

    if removed_ids:
        for r in results:
            segs = r.get('new_segments')
            if segs:
                r['new_segments'] = [s for s in segs if id(s) not in removed_ids]
    if removed_ids or original_to_remove:
        orig_ids = {id(s) for s in original_to_remove}
        pcb_data.segments = [s for s in pcb_data.segments
                             if id(s) not in removed_ids and id(s) not in orig_ids]
    pcb_data.segments = list(pcb_data.segments) + added_segments
    # foreign-copper caches read pcb_data.segments; invalidate so later passes /
    # the next call see the spliced geometry.
    if hasattr(pcb_data, '_foreign_seg_arr_cache'):
        pcb_data._foreign_seg_arr_cache = None

    return (len(removed_ids) + len(original_to_remove) + len(added_segments),
            nets_changed, original_to_remove, added_segments)


def _seg_worst_offender(pcb_data, net_id, s, clearance):
    """The single worst foreign-copper offender below clearance of segment `s`:
    returns (shortfall_mm, t, away_x, away_y) or None. t is the parameter of the
    closest approach along `s`; (away_x, away_y) is the unit direction that
    increases the distance. Distances are edge-to-centreline, sampled like the
    _seg_foreign_*_dist trio (pads as their board-axis rect)."""
    import numpy as np
    from single_ended_routing import (_foreign_pad_arrays, _foreign_seg_arrays,
                                      _foreign_via_arrays, _foreign_hole_capsules)
    from routing_defaults import NPTH_TO_TRACK_CLEARANCE
    required = clearance + s.width / 2.0
    # NPTH mounting/mechanical holes carry no copper, so the pad/seg/via terms
    # miss them; a track crossing one is graded at the higher NPTH-to-track floor
    # (issue #308, urti GND vs J3's hole). Their required clearance differs from
    # the copper terms, so the worst offender is chosen by SHORTFALL, not raw
    # edge distance (a hole 0.15 away can out-rank a via 0.12 away).
    hole_required = max(clearance, NPTH_TO_TRACK_CLEARANCE) + s.width / 2.0
    x1, y1, x2, y2 = s.start_x, s.start_y, s.end_x, s.end_y
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.005) + 1)
    ts = np.linspace(0.0, 1.0, n + 1)
    sx = x1 + (x2 - x1) * ts
    sy = y1 + (y2 - y1) * ts
    R = max(required, hole_required) + 0.2
    best = None  # (shortfall, t, qx, qy)

    def consider(dist, i, qx, qy, req=required):
        nonlocal best
        sf = req - dist
        if best is None or sf > best[0]:
            best = (sf, float(ts[i]), float(qx), float(qy))

    nids, cx, cy, hx, hy, cr = _foreign_pad_arrays(pcb_data, s.layer)
    if cx.size:
        near = ((cx + hx >= sx.min() - R) & (cx - hx <= sx.max() + R) &
                (cy + hy >= sy.min() - R) & (cy - hy <= sy.max() + R) &
                (nids != net_id))
        if near.any():
            fcx, fcy, fhx, fhy, fcr = cx[near], cy[near], hx[near], hy[near], cr[near]
            # Closest point on the pad's rounded-rect boundary: clamp to the inner
            # (corner-radius-shrunk) rect, then step out by the radius toward the
            # sample point. Exact circle/oval for round pads (#315), plain rect at
            # fcr=0. qx/qy is the boundary point -> correct "away" direction.
            qxi = fcx[None, :] + np.clip(sx[:, None] - fcx[None, :],
                                         -(fhx[None, :] - fcr[None, :]), fhx[None, :] - fcr[None, :])
            qyi = fcy[None, :] + np.clip(sy[:, None] - fcy[None, :],
                                         -(fhy[None, :] - fcr[None, :]), fhy[None, :] - fcr[None, :])
            vx = sx[:, None] - qxi; vy = sy[:, None] - qyi
            vlen = np.hypot(vx, vy)
            safe = np.where(vlen > 1e-12, vlen, 1.0)
            qx = qxi + fcr[None, :] * vx / safe
            qy = qyi + fcr[None, :] * vy / safe
            d = vlen - fcr[None, :]
            i, j = np.unravel_index(int(np.argmin(d)), d.shape)
            consider(float(d[i, j]), i, qx[i, j], qy[i, j])

    fnid, fax, fay, fbx, fby, fhw = _foreign_seg_arrays(pcb_data, s.layer)
    if fnid.size:
        near = ((np.maximum(fax, fbx) + fhw >= sx.min() - R) &
                (np.minimum(fax, fbx) - fhw <= sx.max() + R) &
                (np.maximum(fay, fby) + fhw >= sy.min() - R) &
                (np.minimum(fay, fby) - fhw <= sy.max() + R) & (fnid != net_id))
        if near.any():
            ax, ay, bx, by, hw = fax[near], fay[near], fbx[near], fby[near], fhw[near]
            abx, aby = bx - ax, by - ay
            L2 = np.where(abx * abx + aby * aby > 0, abx * abx + aby * aby, 1.0)
            tt = np.clip(((sx[:, None] - ax[None, :]) * abx[None, :] +
                          (sy[:, None] - ay[None, :]) * aby[None, :]) / L2[None, :], 0.0, 1.0)
            qx = ax[None, :] + tt * abx[None, :]
            qy = ay[None, :] + tt * aby[None, :]
            d = np.hypot(sx[:, None] - qx, sy[:, None] - qy) - hw[None, :]
            i, j = np.unravel_index(int(np.argmin(d)), d.shape)
            consider(float(d[i, j]), i, qx[i, j], qy[i, j])

    vnid, vx, vy, vr = _foreign_via_arrays(pcb_data)
    if vx.size:
        near = ((np.abs(vx - (sx.min() + sx.max()) / 2) <= R + (sx.max() - sx.min()) / 2 + vr) &
                (np.abs(vy - (sy.min() + sy.max()) / 2) <= R + (sy.max() - sy.min()) / 2 + vr) &
                (vnid != net_id))
        if near.any():
            fcx, fcy, fr = vx[near], vy[near], vr[near]
            d = np.hypot(sx[:, None] - fcx[None, :], sy[:, None] - fcy[None, :]) - fr[None, :]
            i, j = np.unravel_index(int(np.argmin(d)), d.shape)
            consider(float(d[i, j]), i, fcx[j], fcy[j])

    hnid, hax, hay, hbx, hby, hr = _foreign_hole_capsules(pcb_data)
    if hnid.size:
        near = ((np.maximum(hax, hbx) + hr >= sx.min() - R) &
                (np.minimum(hax, hbx) - hr <= sx.max() + R) &
                (np.maximum(hay, hby) + hr >= sy.min() - R) &
                (np.minimum(hay, hby) - hr <= sy.max() + R) & (hnid != net_id))
        if near.any():
            ax, ay, bx, by, rr = hax[near], hay[near], hbx[near], hby[near], hr[near]
            abx, aby = bx - ax, by - ay
            L2 = np.where(abx * abx + aby * aby > 0, abx * abx + aby * aby, 1.0)
            tt = np.clip(((sx[:, None] - ax[None, :]) * abx[None, :] +
                          (sy[:, None] - ay[None, :]) * aby[None, :]) / L2[None, :], 0.0, 1.0)
            qx = ax[None, :] + tt * abx[None, :]
            qy = ay[None, :] + tt * aby[None, :]
            # Edge distance = axis distance - hole radius; direction points away
            # from the axis point (== away from the hole edge).
            d = np.hypot(sx[:, None] - qx, sy[:, None] - qy) - rr[None, :]
            i, j = np.unravel_index(int(np.argmin(d)), d.shape)
            consider(float(d[i, j]), i, qx[i, j], qy[i, j], hole_required)

    if best is None or best[0] <= 1e-4:
        return None
    shortfall, t, qx, qy = best
    px, py = x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
    norm = math.hypot(px - qx, py - qy)
    if norm < 1e-6:
        return None  # centreline inside the offender: an overlap, not a graze
    return (shortfall, t, (px - qx) / norm, (py - qy) / norm)


def nudge_grazing_microshift(results, pcb_data: PCBData, scope_net_ids=None,
                             clearance: float = 0.1,
                             max_shift: float = 0.025) -> Tuple[int, int, List[Segment], List[Segment]]:
    """Micro-shift copper that still grazes after prune / re-bend / neck (#276).

    Complements nudge_grazing_octolinear, which keeps a jog's anchor endpoints
    FIXED -- useless when the closest approach IS an anchor (a terminal joint
    vertex 8-16um inside the required clearance, e.g. ottercast C58.1) or when
    the only fix is a tiny sideways bow mid-segment (butterstick CATG vs a
    foreign via). Two moves, each by the measured shortfall plus a hair -- the
    minimum copper displacement that restores clearance:

      * VERTEX shift (closest approach at/near an endpoint): move that endpoint
        -- and every same-net same-layer segment sharing the exact vertex --
        directly away from the offender. Terminal joints have slack: their
        endpoint sits inside an adjoining stub's copper body, so a tiny slide
        keeps the copper-overlap join; the connectivity gate proves it. A
        vertex carrying a via never moves (layer-stack alignment).
      * BOW (closest approach mid-segment): split at the approach and offset a
        short middle section perpendicular, away from the offender.

    Every candidate is verified to clear ALL foreign copper (pad/track/via)
    and the board edge at its own width, and to not worsen the net's
    connectivity, before it replaces the original geometry -- the pass can only
    remove a graze, never introduce one or disconnect a net. A graze that no
    candidate clears is left for the DRC report.

    ``max_shift`` HARD-caps how far any copper may move (callers pass half the
    routing grid step): this pass is strictly a micrometre-scale touch-up, so a
    graze needing more than that is genuinely mis-routed and must stay visible
    in the DRC report rather than be papered over with a wild move.

    Returns (segments_changed, nets_changed, original_segments_to_remove,
    added_segments) -- same contract as nudge_grazing_octolinear."""
    from collections import defaultdict
    from check_connected import check_net_connectivity
    from single_ended_routing import (_seg_foreign_pad_dist, _seg_foreign_seg_dist,
                                      _seg_foreign_via_dist, _seg_foreign_hole_dist)
    from routing_defaults import NPTH_TO_TRACK_CLEARANCE

    # NPTH (no-copper) drill holes are graded at the higher NPTH-to-track floor,
    # and the copper distance terms don't see them (issue #308, urti GND vs J3).
    npth_clr = max(clearance, NPTH_TO_TRACK_CLEARANCE)

    routed_seg_result = {}
    for r in results:
        for s in r.get('new_segments') or []:
            routed_seg_result[id(s)] = r

    from check_drc import board_edge_geometry, _point_on_board, _segment_to_rings_distance
    edge_rings, edge_outer, edge_cutouts = board_edge_geometry(pcb_data.board_info)
    board_bounds = pcb_data.board_info.board_bounds

    def edge_clears(x1, y1, x2, y2, w):
        required = clearance + w / 2.0 - 1e-4
        if edge_rings:
            if not _point_on_board(x1, y1, edge_outer, edge_cutouts) or \
               not _point_on_board(x2, y2, edge_outer, edge_cutouts):
                return False
            return _segment_to_rings_distance(x1, y1, x2, y2, edge_rings) >= required
        if board_bounds:
            min_x, min_y, max_x, max_y = board_bounds
            return all(min(x - min_x, max_x - x, y - min_y, max_y - y) >= required
                       for x, y in ((x1, y1), (x2, y2)))
        return True

    def clears(x1, y1, x2, y2, layer, net_id, w):
        d = min(_seg_foreign_pad_dist(pcb_data, net_id, x1, y1, x2, y2, layer),
                _seg_foreign_seg_dist(pcb_data, net_id, x1, y1, x2, y2, layer),
                _seg_foreign_via_dist(pcb_data, net_id, x1, y1, x2, y2, layer))
        hd = _seg_foreign_hole_dist(pcb_data, net_id, x1, y1, x2, y2)
        return (d >= clearance + w / 2.0 - 1e-4 and
                hd >= npth_clr + w / 2.0 - 1e-4 and
                edge_clears(x1, y1, x2, y2, w))

    def vk(x, y):
        return (round(x, 3), round(y, 3))

    def worse(before, after):
        return ((before.get('connected') and not after.get('connected')) or
                len(after.get('disconnected_pads') or []) > len(before.get('disconnected_pads') or []) or
                (after.get('num_components') or 1) > (before.get('num_components') or 1))

    zones_by_net = defaultdict(list)
    for z in (getattr(pcb_data, 'zones', []) or []):
        zones_by_net[z.net_id].append(z)
    vias_by_net = defaultdict(list)
    for v in pcb_data.vias:
        vias_by_net[v.net_id].append(v)
    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        if scope_net_ids is None or s.net_id in scope_net_ids:
            segs_by_net[s.net_id].append(s)

    MARGIN = 0.004   # displacement beyond the exact shortfall
    removed_ids = set()
    original_to_remove = []
    added_segments = []
    added_ids = set()  # segments THIS pass created (a later round may replace one)
    nets_changed = 0

    def grazes(s):
        # Cheap 0.02-sampled prefilter (incl. foreign TRACKS -- cynthion's
        # offender is a track); the precise 0.005-sampled offender scan runs
        # only on the handful that fail this.
        thr = clearance + s.width / 2.0 - 1e-4
        if min(_seg_foreign_pad_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                     s.end_x, s.end_y, s.layer),
               _seg_foreign_seg_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                     s.end_x, s.end_y, s.layer),
               _seg_foreign_via_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                     s.end_x, s.end_y, s.layer)) < thr:
            return True
        # NPTH-hole graze uses the higher NPTH-to-track floor (issue #308).
        hole_thr = npth_clr + s.width / 2.0 - 1e-4
        return _seg_foreign_hole_dist(pcb_data, s.net_id, s.start_x, s.start_y,
                                      s.end_x, s.end_y) < hole_thr

    MAX_ROUNDS = 3   # a fixed worst offender can expose the second-worst

    for net_id, net_segs in segs_by_net.items():
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        net_zones = zones_by_net.get(net_id, [])
        via_keys = {vk(v.x, v.y) for v in net_vias}
        before = None
        net_changed = False

        # Rounds re-scan the net's own moved copper (fixing the worst offender
        # can expose the second-worst on the same segment); foreign copper is
        # static during the pass.
        for _round in range(MAX_ROUNDS):
            grazing = [s for s in net_segs if grazes(s)]
            offenders = [(s, _seg_worst_offender(pcb_data, net_id, s, clearance))
                         for s in grazing]
            offenders = [(s, o) for s, o in offenders if o is not None]
            if not offenders:
                break
            if before is None:
                before = check_net_connectivity(net_id, net_segs, net_vias,
                                                net_pads, net_zones)
            round_changed = False

            for s, (shortfall, t, awx, awy) in offenders:
                if s not in net_segs:
                    continue  # replaced while fixing an earlier graze
                seg_len = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                required = clearance + s.width / 2.0
                candidates = []  # (old_segs, new_segs)

                def vertex_candidates(px, py):
                    if vk(px, py) in via_keys:
                        return  # never slide a via off its layer stack
                    incident = [g for g in net_segs
                                if vk(g.start_x, g.start_y) == vk(px, py)
                                or vk(g.end_x, g.end_y) == vk(px, py)]
                    if any(g.layer != s.layer for g in incident):
                        return  # cross-layer joint without a recorded via: leave it
                    for m in (1.0, 1.8, 3.0):
                        d = (shortfall + MARGIN) * m
                        if d > max_shift:
                            continue
                        nx, ny = round(px + awx * d, 4), round(py + awy * d, 4)
                        new = [Segment(
                            start_x=nx if vk(g.start_x, g.start_y) == vk(px, py) else g.start_x,
                            start_y=ny if vk(g.start_x, g.start_y) == vk(px, py) else g.start_y,
                            end_x=nx if vk(g.end_x, g.end_y) == vk(px, py) else g.end_x,
                            end_y=ny if vk(g.end_x, g.end_y) == vk(px, py) else g.end_y,
                            width=g.width, layer=g.layer, net_id=net_id)
                            for g in incident]
                        candidates.append((incident, new))

                def bow_candidates():
                    # Clamp the bow inside the segment: a graze near one end
                    # (butterstick's t=0.15 next to its launch via) still gets
                    # a bow when the near vertex can't move.
                    if seg_len < 1e-6 or not (0.02 < t < 0.98):
                        return
                    half = min(max(required, 0.15),
                               t * seg_len * 0.9, (1.0 - t) * seg_len * 0.9)
                    if half < 0.02:
                        return
                    dxu = (s.end_x - s.start_x) / seg_len
                    dyu = (s.end_y - s.start_y) / seg_len
                    cxp = s.start_x + (s.end_x - s.start_x) * t
                    cyp = s.start_y + (s.end_y - s.start_y) * t
                    for m in (1.0, 1.8, 3.0):
                        h = (shortfall + MARGIN) * m
                        if h > max_shift:
                            continue
                        q1 = (round(cxp - dxu * half + awx * h, 4),
                              round(cyp - dyu * half + awy * h, 4))
                        q2 = (round(cxp + dxu * half + awx * h, 4),
                              round(cyp + dyu * half + awy * h, 4))
                        pts = [(s.start_x, s.start_y), q1, q2, (s.end_x, s.end_y)]
                        new = [Segment(start_x=pts[i][0], start_y=pts[i][1],
                                       end_x=pts[i + 1][0], end_y=pts[i + 1][1],
                                       width=s.width, layer=s.layer, net_id=net_id)
                               for i in range(3)
                               if (pts[i][0], pts[i][1]) != (pts[i + 1][0], pts[i + 1][1])]
                        candidates.append(([s], new))

                # Preference: slide the endpoint nearest the approach, then a
                # bow, then the far endpoint (a short segment may rotate a hair).
                near_v = (s.start_x, s.start_y) if t <= 0.5 else (s.end_x, s.end_y)
                far_v = (s.end_x, s.end_y) if t <= 0.5 else (s.start_x, s.start_y)
                if t <= 0.3 or t >= 0.7:
                    vertex_candidates(*near_v)
                    bow_candidates()
                else:
                    bow_candidates()
                    vertex_candidates(*near_v)
                if seg_len <= 2 * required:
                    vertex_candidates(*far_v)

                for old, new in candidates:
                    if not all(clears(g.start_x, g.start_y, g.end_x, g.end_y,
                                      g.layer, net_id, g.width) for g in new):
                        continue
                    trial = [g for g in net_segs if g not in old] + new
                    if worse(before, check_net_connectivity(net_id, trial, net_vias,
                                                            net_pads, net_zones)):
                        continue
                    res = None
                    for g in old:
                        if id(g) in added_ids:
                            # a segment this pass created in an earlier round:
                            # drop it entirely (strip from its res list below,
                            # never splice it into pcb_data)
                            removed_ids.add(id(g))
                            added_ids.discard(id(g))
                            added_segments.remove(g)
                        elif id(g) in routed_seg_result:
                            removed_ids.add(id(g))
                            res = res or routed_seg_result[id(g)]
                        else:
                            original_to_remove.append(g)
                    if res is None:
                        res = {'new_segments': [], 'new_vias': []}
                        results.append(res)
                    res['new_segments'] = list(res.get('new_segments') or []) + new
                    added_segments.extend(new)
                    added_ids.update(id(g) for g in new)
                    net_segs = trial
                    net_changed = True
                    round_changed = True
                    break

            if not round_changed:
                break
        if net_changed:
            nets_changed += 1

    if removed_ids:
        for r in results:
            segs = r.get('new_segments')
            if segs:
                r['new_segments'] = [s for s in segs if id(s) not in removed_ids]
    if removed_ids or original_to_remove:
        orig_ids = {id(s) for s in original_to_remove}
        pcb_data.segments = [s for s in pcb_data.segments
                             if id(s) not in removed_ids and id(s) not in orig_ids]
    pcb_data.segments = list(pcb_data.segments) + added_segments
    if hasattr(pcb_data, '_foreign_seg_arr_cache'):
        pcb_data._foreign_seg_arr_cache = None

    return (len(removed_ids) + len(original_to_remove) + len(added_segments),
            nets_changed, original_to_remove, added_segments)


def nudge_grazing_vias(results, pcb_data: PCBData, scope_net_ids=None,
                       clearance: float = 0.1, hole_to_hole: float = 0.20,
                       max_shift: float = 0.025,
                       allowed_via_ids=None) -> Tuple[int, int, List[Tuple]]:
    """Sub-grid nudge for a VIA that grazes foreign copper or a drill (#280).

    Two vias snapped to the routing grid can land a few µm inside clearance
    (usb_sniffer: a GND plane-stitch via 7µm short of a signal via). Tracks
    have the microshift pass; vias had nothing -- the microshift deliberately
    never moves a via vertex. This pass moves the VIA ALONE (no segment is
    touched) by the measured shortfall plus a hair, away from the worst
    offender: the attached track ends stay buried deep inside the via body
    (the move is far below the via radius), so KiCad's copper-overlap
    connectivity is untouched, and check_net_connectivity joins a segment end
    to a via within via_size/4 -- exactly the move cap.

    Guardrails (why this is safe where the removed in-routing nudge_grazes was
    not, #147/#70/#130):
      * post-route and via-only -- nothing is ripped, re-routed, or dragged;
      * the move is hard-capped at min(``max_shift``, via_size/4) (callers
        pass grid_step/2): a via needing more is genuinely mis-placed and
        stays visible in DRC;
      * a candidate is committed only when the via clears EVERY foreign
        object at its new spot (segment/via/pad body, drill hole-to-hole
        incl. same-net, and the board edge) -- strictly fewer grazes, never a
        new one;
      * connectivity is re-checked per net and the move reverted if worse.

    Only vias CREATED BY THIS RUN may move: the writers re-emit this run's
    new vias but keep the input file's text for pre-existing ones, so moving
    an input via would silently revert in the output. Candidates come from
    ``results[*]['new_vias']`` (or ``allowed_via_ids``, a set of id(via), for
    the plane wrapper whose results list is empty).

    Returns (vias_moved, nets_changed, moves) with moves =
    [(net_id, old_x, old_y, new_x, new_y)] so plane-script callers can mirror
    the new position into their via write-list dicts.
    """
    from collections import defaultdict
    from check_connected import check_net_connectivity
    from single_ended_routing import (_seg_foreign_pad_dist, _seg_foreign_seg_dist,
                                      _seg_foreign_via_dist)
    from check_drc import (board_edge_geometry, _point_on_board,
                           _point_to_rings_distance)
    from geometry_utils import point_to_segment_distance

    board_info = getattr(pcb_data, 'board_info', None)
    copper_layers = list(getattr(board_info, 'copper_layers', None) or
                         ['F.Cu', 'B.Cu'])
    edge_rings, edge_outer, edge_cutouts = board_edge_geometry(board_info)
    MARGIN = 0.002
    WINDOW = 2.0     # local object window around a flagged via (mm)

    def worse(before, after):
        return ((before.get('connected') and not after.get('connected')) or
                len(after.get('disconnected_pads') or []) > len(before.get('disconnected_pads') or []) or
                (after.get('num_components') or 1) > (before.get('num_components') or 1))

    # Every drill hole on the board (vias + through-hole pads, ANY net --
    # hole-to-hole is a fab rule, not an electrical one).
    import numpy as _np
    hole_list = [(id(v), v.x, v.y, (getattr(v, 'drill', 0) or 0) / 2.0)
                 for v in pcb_data.vias if (getattr(v, 'drill', 0) or 0) > 0]
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            if (p.drill or 0) > 0:
                hole_list.append((id(p), p.global_x, p.global_y, p.drill / 2.0))
    hole_idx = {hid: i for i, (hid, _, _, _) in enumerate(hole_list)}
    hole_x = _np.asarray([h[1] for h in hole_list], dtype=float)
    hole_y = _np.asarray([h[2] for h in hole_list], dtype=float)
    hole_r = _np.asarray([h[3] for h in hole_list], dtype=float)

    vias_by_net = defaultdict(list)
    for v in pcb_data.vias:
        vias_by_net[v.net_id].append(v)
    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        segs_by_net[s.net_id].append(s)
    zones_by_net = defaultdict(list)
    for z in (getattr(pcb_data, 'zones', []) or []):
        zones_by_net[z.net_id].append(z)

    def copper_flagged(v):
        """Cheap numpy prefilter: via body sub-clearance to any foreign copper."""
        thr = clearance + v.size / 2.0 - 1e-4
        if _seg_foreign_via_dist(pcb_data, v.net_id, v.x, v.y, v.x, v.y,
                                 copper_layers[0]) < thr:
            return True
        for lyr in copper_layers:
            if _seg_foreign_seg_dist(pcb_data, v.net_id, v.x, v.y, v.x, v.y,
                                     lyr) < thr:
                return True
            if _seg_foreign_pad_dist(pcb_data, v.net_id, v.x, v.y, v.x, v.y,
                                     lyr) < thr:
                return True
        return False

    def hole_flagged(v):
        vd = (getattr(v, 'drill', 0) or 0) / 2.0
        if vd <= 0 or hole_x.size == 0:
            return False
        d = _np.hypot(hole_x - v.x, hole_y - v.y)
        mask = d < vd + hole_r + hole_to_hole - 1e-4
        i = hole_idx.get(id(v))
        if i is not None:
            mask[i] = False
        return bool(mask.any())

    def gather_near(v):
        """Foreign copper + all holes within WINDOW of the via, evaluated exactly."""
        near_segs, near_vias, near_pads, near_holes = [], [], [], []
        x, y, me = v.x, v.y, id(v)
        for s in pcb_data.segments:
            if s.net_id == v.net_id:
                continue
            if (min(s.start_x, s.end_x) - WINDOW <= x <= max(s.start_x, s.end_x) + WINDOW
                    and min(s.start_y, s.end_y) - WINDOW <= y <= max(s.start_y, s.end_y) + WINDOW):
                near_segs.append(s)
        for o in pcb_data.vias:
            if id(o) == me or o.net_id == v.net_id:
                continue
            if abs(o.x - x) <= WINDOW and abs(o.y - y) <= WINDOW:
                near_vias.append(o)
        for pads in pcb_data.pads_by_net.values():
            for p in pads:
                if p.net_id == v.net_id or getattr(p, 'pad_type', '') == 'np_thru_hole':
                    continue
                if abs(p.global_x - x) <= WINDOW and abs(p.global_y - y) <= WINDOW:
                    near_pads.append(p)
        for hid, hx, hy, hr in hole_list:
            if hid != me and abs(hx - x) <= WINDOW and abs(hy - y) <= WINDOW:
                near_holes.append((hx, hy, hr))
        return near_segs, near_vias, near_pads, near_holes

    def worst_gap(x, y, v, near):
        """(gap, ux, uy): most negative clearance surplus at (x, y) and the unit
        direction AWAY from that offender. Positive gap = fully clear."""
        near_segs, near_vias, near_pads, near_holes = near
        r = v.size / 2.0
        vd = (getattr(v, 'drill', 0) or 0) / 2.0
        best = (float('inf'), 1.0, 0.0)

        def consider(gap, ox, oy):
            nonlocal best
            if gap < best[0]:
                d = math.hypot(x - ox, y - oy)
                if d > 1e-9:
                    best = (gap, (x - ox) / d, (y - oy) / d)
                else:
                    best = (gap, 1.0, 0.0)

        for s in near_segs:
            d, t = _pt_seg_dist_t(x, y, s)
            consider(d - (r + s.width / 2.0 + clearance),
                     s.start_x + t * (s.end_x - s.start_x),
                     s.start_y + t * (s.end_y - s.start_y))
        for o in near_vias:
            d = math.hypot(x - o.x, y - o.y)
            consider(d - (r + o.size / 2.0 + clearance), o.x, o.y)
        for p in near_pads:
            tp, g = _nearest_pad_point(x, y, p)
            consider(g - (r + clearance), tp[0], tp[1])
        if vd > 0:
            for hx, hy, hr in near_holes:
                d = math.hypot(x - hx, y - hy)
                consider(d - (vd + hr + hole_to_hole), hx, hy)
        # board edge: include in the gap so a candidate never trades a copper
        # graze for an edge one (no direction needed -- it's a veto, not a target)
        if edge_rings:
            if not _point_on_board(x, y, edge_outer, edge_cutouts):
                best = (min(best[0], -1.0), best[1], best[2])
            else:
                eg = _point_to_rings_distance(x, y, edge_rings) - (r + clearance)
                if eg < best[0]:
                    best = (eg, best[1], best[2])
        return best

    moved = 0
    nets_changed_set = set()
    moves = []

    own_ids = set(allowed_via_ids or ())
    for r in results:
        for v in r.get('new_vias') or []:
            own_ids.add(id(v))
    scoped = [v for v in pcb_data.vias
              if id(v) in own_ids
              and (scope_net_ids is None or v.net_id in scope_net_ids)]
    for v in scoped:
        if not (copper_flagged(v) or hole_flagged(v)):
            continue
        near = gather_near(v)
        gap, ux, uy = worst_gap(v.x, v.y, v, near)
        if gap >= -1e-4:
            continue    # prefilter false positive
        shortfall = -gap
        # The via moves ALONE: cap the move so the attached track ends stay
        # connected on both models (KiCad copper overlap: move << via radius;
        # check_net_connectivity joins within via_size/4).
        # Two bounds, take the smaller: via_size/4 is the connectivity-safe
        # reach (attached track ends stay buried at the via's mid-radius -- deep
        # overlap, not a graze -- and both KiCad and check_net_connectivity join
        # within it), and max_shift bounds the move to the caller's budget (the
        # route write path passes one grid cell, so a via never wanders more than
        # a cell). A via needing more than this is genuinely mis-placed.
        cap = min(max_shift, v.size / 4.0)
        if shortfall + MARGIN > cap:
            continue    # genuinely mis-placed: leave it visible in DRC

        net_segs = segs_by_net.get(v.net_id, [])
        net_pads = pcb_data.pads_by_net.get(v.net_id, [])
        net_vias = vias_by_net.get(v.net_id, [])
        net_zones = zones_by_net.get(v.net_id, [])
        before = check_net_connectivity(v.net_id, net_segs, net_vias, net_pads,
                                        net_zones)

        for ang in (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0):
            a = math.radians(ang)
            ca = math.cos(a)
            if ca <= 0.1:
                continue
            dx, dy = (ux * math.cos(a) - uy * math.sin(a),
                      ux * math.sin(a) + uy * math.cos(a))
            dist = (shortfall + MARGIN) / ca
            if dist > cap:
                continue
            nx, ny = round(v.x + dx * dist, 4), round(v.y + dy * dist, 4)
            if worst_gap(nx, ny, v, near)[0] < -1e-6:
                continue

            # apply, verify connectivity, revert on regression
            old_x, old_y = v.x, v.y
            v.x, v.y = nx, ny
            after = check_net_connectivity(v.net_id, net_segs, net_vias, net_pads,
                                           net_zones)
            if worse(before, after):
                v.x, v.y = old_x, old_y
                continue
            moved += 1
            nets_changed_set.add(v.net_id)
            moves.append((v.net_id, old_x, old_y, nx, ny))
            # position changed: drop the numpy caches so later queries (and
            # later passes) see the new geometry
            pcb_data._foreign_seg_arr_cache = None
            pcb_data._foreign_via_arr_cache = None
            break

    return moved, len(nets_changed_set), moves


def _pt_seg_dist_t(x, y, s):
    """(distance, t) from a point to segment s."""
    dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(x - s.start_x, y - s.start_y), 0.0
    t = max(0.0, min(1.0, ((x - s.start_x) * dx + (y - s.start_y) * dy) / L2))
    return math.hypot(x - (s.start_x + t * dx), y - (s.start_y + t * dy)), t


def cleanup_plane_taps_grazing(pcb_data: PCBData, all_new_segments: List[Dict],
                               scope_net_ids=None, clearance: float = 0.1,
                               max_shift: float = 0.025,
                               all_new_vias: Optional[List[Dict]] = None,
                               hole_to_hole: float = 0.20):
    """Apply prune_grazing_segments + nudge_grazing_octolinear + sweep_dead_ends to a
    PLANE script's write-list (issue #224).

    route_planes / route_disconnected_planes carry their new copper as
    {'start','end','width','layer','net_id'} DICTS in `all_new_segments` (not the
    route.py `results` list of Segment objects), so the passes -- which operate on
    pcb_data and the route.py results -- are driven here with an empty results list
    and their Segment-level removals/additions are mirrored back into the dict list
    by coordinate signature.

    The plane scripts have no other cleanup of their own copper (route.py excludes
    the plane nets), so this is their only chance to drop bad taps:
      * grazing prune with ``check_foreign_segments`` -- a tap laid through the
        obstacle-exempt endpoint region can sit sub-clearance to a neighbouring
        signal TRACK, not just a pad/via (glasgow +3V3 tap grazing the Y2 track);
      * dead-end sweep -- a superseded/failed reuse-tap (the fill-aware re-check
        force-via path leaves the abandoned tap copper behind) is a dangling
        appendix that never reaches the plane.
    Both are connectivity-gated WITH the pour (check_net_connectivity sees the
    zones), so a load-bearing tap that actually carries a pad to the plane is kept
    and only genuinely redundant/dead copper goes.

    Returns (all_new_segments, n_removed, n_nudged, n_swept).
    """
    def sig(sx, sy, ex, ey, layer):
        a, b = (round(sx, 3), round(sy, 3)), (round(ex, 3), round(ey, 3))
        return (min(a, b), max(a, b), layer)

    def strip(segs, removed):
        if not removed:
            return segs, 0
        rm = {sig(s.start_x, s.start_y, s.end_x, s.end_y, s.layer) for s in removed}
        out = [d for d in segs
               if sig(d['start'][0], d['start'][1], d['end'][0], d['end'][1], d['layer']) not in rm]
        return out, len(segs) - len(out)

    # Drop redundant grazing taps -- against a foreign pad/via OR a foreign track.
    _, _, removed = prune_grazing_segments([], pcb_data, scope_net_ids, clearance,
                                           check_foreign_segments=True)
    all_new_segments, n_removed = strip(all_new_segments, removed)

    # Re-bend the load-bearing ones around the pad.
    _, n_nudged, nudge_removed, nudge_added = nudge_grazing_octolinear(
        [], pcb_data, scope_net_ids, clearance)
    all_new_segments, _ = strip(all_new_segments, nudge_removed)
    for s in nudge_added:
        all_new_segments.append({'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
                                 'width': s.width, 'layer': s.layer, 'net_id': s.net_id})

    # Micro-shift what the re-bend can't reach (#276): a graze whose closest
    # approach IS an anchor vertex, or one needing only a tiny mid-segment bow.
    _, n_shifted, ms_removed, ms_added = nudge_grazing_microshift(
        [], pcb_data, scope_net_ids, clearance, max_shift=max_shift)
    n_nudged += n_shifted
    all_new_segments, _ = strip(all_new_segments, ms_removed)
    for s in ms_added:
        all_new_segments.append({'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
                                 'width': s.width, 'layer': s.layer, 'net_id': s.net_id})

    # Sub-grid via nudge (#280): a grid-snapped stitch/tap via a few um inside
    # clearance of a foreign via/track/hole moves by its shortfall. The via
    # moves ALONE (capped at min(max_shift, via_size/4), so the tap segments
    # still end inside its body); mirror the new position into the plane via
    # write-list by old-coordinate signature.
    def _pt(px, py):
        return (round(px, 3), round(py, 3))
    new_via_pts = {_pt(d['x'], d['y']) for d in (all_new_vias or [])
                   if isinstance(d, dict)}
    allowed = {id(v) for v in pcb_data.vias if _pt(v.x, v.y) in new_via_pts}
    n_via_moved, _, via_moves = nudge_grazing_vias(
        [], pcb_data, scope_net_ids, clearance,
        hole_to_hole=hole_to_hole, max_shift=max_shift,
        allowed_via_ids=allowed)
    if via_moves:
        n_nudged += n_via_moved
        moved_pts = {(net, _pt(ox, oy)): (nx, ny)
                     for net, ox, oy, nx, ny in via_moves}
        for d in (all_new_vias or []):
            if not isinstance(d, dict):
                continue
            hit = moved_pts.get((d.get('net_id'), _pt(d['x'], d['y'])))
            if hit is not None:
                d['x'], d['y'] = hit

    # Sweep dead-end appendices left by a superseded reuse-tap -- but ONLY this
    # run's tap copper is a candidate: the rest of each plane net anchors it. A big
    # pour has hundreds of pre-existing pad taps that look like geometric dead ends
    # (they land on the fill), and validating each against the whole-net union-find
    # is the sweep's dominant cost (~0.5s x hundreds). Restricting candidates to the
    # copper we just added -- the only copper that can be a fresh orphan -- cuts that
    # to the handful of new taps while the anchors keep every real connection intact.
    from collections import defaultdict
    new_sigs = {sig(d['start'][0], d['start'][1], d['end'][0], d['end'][1], d['layer'])
                for d in all_new_segments}
    all_zones = getattr(pcb_data, 'zones', []) or []
    net_segs = defaultdict(list)
    for s in pcb_data.segments:
        if scope_net_ids is None or s.net_id in scope_net_ids:
            net_segs[s.net_id].append(s)
    de_removed = []
    for net_id, segs in net_segs.items():
        prunable = [s for s in segs
                    if sig(s.start_x, s.start_y, s.end_x, s.end_y, s.layer) in new_sigs]
        if not prunable:
            continue
        p_ids = {id(s) for s in prunable}
        anchor = [s for s in segs if id(s) not in p_ids]
        _, removed = _safe_prune_net(
            net_id, prunable,
            [v for v in pcb_data.vias if v.net_id == net_id],
            pcb_data.pads_by_net.get(net_id, []),
            [z for z in all_zones if z.net_id == net_id],
            anchor_segments=anchor, aggressive=True)
        de_removed.extend(removed)
    all_new_segments, n_swept = strip(all_new_segments, de_removed)
    if de_removed:
        rm_ids = {id(s) for s in de_removed}
        pcb_data.segments = [s for s in pcb_data.segments if id(s) not in rm_ids]

    return all_new_segments, n_removed, n_nudged, n_swept


def swap_pad_nets_in_pcb_data(pcb_data: PCBData, pad_a, pad_b) -> None:
    """Swap the net assignments of two pads in pcb_data (net_id, net_name, and
    membership in pads_by_net / Net.pads).

    Used by polarity fixes and target swaps so the in-memory state matches the
    swap that is later applied to the output file or live board.
    """
    net_a, net_b = pad_a.net_id, pad_b.net_id
    pad_a.net_id, pad_b.net_id = net_b, net_a
    pad_a.net_name, pad_b.net_name = pad_b.net_name, pad_a.net_name

    for pad, old_net, new_net in ((pad_a, net_a, net_b), (pad_b, net_b, net_a)):
        old_list = pcb_data.pads_by_net.get(old_net)
        if old_list and pad in old_list:
            old_list.remove(pad)
        pcb_data.pads_by_net.setdefault(new_net, []).append(pad)

        old_net_obj = pcb_data.nets.get(old_net)
        if old_net_obj and pad in old_net_obj.pads:
            old_net_obj.pads.remove(pad)
        new_net_obj = pcb_data.nets.get(new_net)
        if new_net_obj is not None:
            new_net_obj.pads.append(pad)


def add_route_to_pcb_data(pcb_data: PCBData, result: dict, debug_lines: bool = False) -> None:
    """Add routed segments and vias to PCB data for subsequent routes to see."""
    new_segments = result['new_segments']
    if not new_segments:
        return

    # Get all unique net_ids from new segments
    net_ids = set(s.net_id for s in new_segments)

    # Get new vias for appendix checking
    new_vias = result.get('new_vias', [])

    # Process each net separately for same-net cleanup
    cleaned_segments = []
    for net_id in net_ids:
        net_segs = [s for s in new_segments if s.net_id == net_id]
        existing_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        # Include both new vias and existing vias for this net
        net_vias = [v for v in new_vias if v.net_id == net_id]
        net_vias.extend([v for v in pcb_data.vias if v.net_id == net_id])
        # Include pads for this net
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        # Per-commit self-intersection clean (fix_self_intersections /
        # collapse_appendices) removed: it "fixed" same-net crossings by extending a
        # segment to a far off-grid endpoint, creating long non-orthonormal diagonals
        # that crossed foreign copper (#159). prune_redundant_cycles + sweep_dead_ends
        # cover connectivity; the residual cosmetic self-crossings are tracked in #162.
        cleaned_segments.extend(net_segs)

    # Filter out very short (degenerate) segments. A dropped segment whose BOTH
    # ends touch other segments is a micro-bridge (sub-grid geometry like
    # bisector offsets can produce um-scale bridges between a connector and the
    # parallel path) - weld its neighbors together at the midpoint so no gap is
    # left behind. One-ended micro-stubs (e.g. collapsed appendices) are
    # dropped as before, without disturbing the junction they hang off.
    def seg_len(s):
        return math.sqrt((s.end_x - s.start_x)**2 + (s.end_y - s.start_y)**2)

    def touching(seg, ax, ay):
        """Other segments with an endpoint at (ax, ay)."""
        result = []
        for other in cleaned_segments:
            if other is seg or other.net_id != seg.net_id or other.layer != seg.layer:
                continue
            if ((abs(other.start_x - ax) < 0.005 and abs(other.start_y - ay) < 0.005) or
                    (abs(other.end_x - ax) < 0.005 and abs(other.end_y - ay) < 0.005)):
                result.append(other)
        return result

    kept_segments = []
    for seg in cleaned_segments:
        if seg_len(seg) > 0.01:
            kept_segments.append(seg)
            continue
        start_touch = touching(seg, seg.start_x, seg.start_y)
        end_touch = touching(seg, seg.end_x, seg.end_y)
        if len(start_touch) != 1 or len(end_touch) != 1 or start_touch[0] is end_touch[0]:
            # Not a simple chain bridge (dangling micro-stub, junction, or
            # T-tap) - drop it without disturbing the neighbors
            continue
        mid_x = (seg.start_x + seg.end_x) / 2
        mid_y = (seg.start_y + seg.end_y) / 2
        for ax, ay, others in ((seg.start_x, seg.start_y, start_touch),
                               (seg.end_x, seg.end_y, end_touch)):
            for other in others:
                if abs(other.start_x - ax) < 0.005 and abs(other.start_y - ay) < 0.005:
                    other.start_x, other.start_y = mid_x, mid_y
                if abs(other.end_x - ax) < 0.005 and abs(other.end_y - ay) < 0.005:
                    other.end_x, other.end_y = mid_x, mid_y
    cleaned_segments = kept_segments

    # Per-commit dead-end prune (issue #84): drop this route's own dead-end spurs
    # so the dead copper is not left on the board to block following routes. Only
    # the segments being added are prunable; the net's copper already on the board
    # anchors junctions/escapes but is not removed here (the final sweep handles
    # board-wide settle). collapse_appendices above now only fixes
    # self-intersections (#148); this unwinds dead-end spurs and chains via
    # prune_dead_end_segments.
    all_zones = getattr(pcb_data, 'zones', []) or []
    pruned_segments = []
    for net_id in net_ids:
        net_new = [s for s in cleaned_segments if s.net_id == net_id]
        if not net_new:
            continue
        anchor = [s for s in pcb_data.segments if s.net_id == net_id]
        net_vias = [v for v in new_vias if v.net_id == net_id]
        net_vias.extend([v for v in pcb_data.vias if v.net_id == net_id])
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_zones = [z for z in all_zones if z.net_id == net_id]
        kept_net, _ = _safe_prune_net(net_id, net_new, net_vias, net_pads, net_zones,
                                      anchor_segments=anchor, aggressive=False)
        pruned_segments.extend(kept_net)
    cleaned_segments = pruned_segments

    for seg in cleaned_segments:
        pcb_data.segments.append(seg)
    for via in result['new_vias']:
        pcb_data.vias.append(via)
    # Update result so output file also gets cleaned segments
    result['new_segments'] = cleaned_segments


def drop_phantom_copper(results, pcb_data: PCBData,
                        original_segment_ids=None,
                        original_via_ids=None) -> Tuple[int, int]:
    """Reconcile the write-list and the board in BOTH directions (issue #133 /
    #319 restructure).

    Direction 1 -- write-list entries not on the board ("phantoms"): a result's
    ``new_segments`` / ``new_vias`` hold the SAME objects that
    ``add_route_to_pcb_data`` appended to ``pcb_data``; ``remove_route_from_pcb_data``
    drops those objects when a net is ripped. But a result snapshot taken before a
    rip-reroute can keep referencing copper that was later ripped and not restored
    (e.g. a multipoint net's ``completed_result``, built before ``try_phase3_ripup``
    ripped that net's own main route out from under it, is still committed). The
    output is written from these results, so the phantom copper lands on the board --
    including a DIFFERENT-net via at a cell another net legitimately took while this
    net was ripped, an un-manufacturable drill-on-drill short (issue #133:
    EPHY_TX_N / EPHY_RX_P escape vias).

    Direction 2 -- board copper this run created that no result references
    ("orphans", only when ``original_segment_ids``/``original_via_ids`` identify
    the input-file copper): rip/reroute and superseded-result drops can leave a
    routed sliver in pcb_data whose result was discarded, so it will never be
    written. Passes and connectivity gates reading pcb_data would reason about
    copper the file won't have (the glasgow P1 phantom-success class; surfaced
    by the KICAD_BOARD_LEDGER audit as a board-only /DRAM_VDDQ sliver on
    sechzig). Remove it from pcb_data so board == write model.

    Membership is by object identity, so a re-cleaned or re-placed object (same
    position, different object) is never confused with the ripped one, and live
    copper is never dropped. Mutates results and pcb_data in place; returns
    ``(phantom_segments_dropped, phantom_vias_dropped)``.
    """
    board_segs = {id(s) for s in pcb_data.segments}
    board_vias = {id(v) for v in pcb_data.vias}
    phantom_segs = phantom_vias = 0
    for r in results:
        segs = r.get('new_segments')
        if segs:
            kept = [s for s in segs if id(s) in board_segs]
            phantom_segs += len(segs) - len(kept)
            r['new_segments'] = kept
        vias = r.get('new_vias')
        if vias:
            kept = [v for v in vias if id(v) in board_vias]
            phantom_vias += len(vias) - len(kept)
            r['new_vias'] = kept

    if original_segment_ids is not None:
        emitted = {id(s) for r in results for s in (r.get('new_segments') or [])}
        orphan = [s for s in pcb_data.segments
                  if id(s) not in original_segment_ids and id(s) not in emitted]
        if orphan:
            _oids = {id(s) for s in orphan}
            pcb_data.segments = [s for s in pcb_data.segments if id(s) not in _oids]
            print(f"Dropped {len(orphan)} orphan routed segment(s) from the board "
                  f"(rip/reroute copper no result references)")
    if original_via_ids is not None:
        emitted_v = {id(v) for r in results for v in (r.get('new_vias') or [])}
        orphan_v = [v for v in pcb_data.vias
                    if id(v) not in original_via_ids and id(v) not in emitted_v]
        if orphan_v:
            _ovids = {id(v) for v in orphan_v}
            pcb_data.vias = [v for v in pcb_data.vias if id(v) not in _ovids]
            print(f"Dropped {len(orphan_v)} orphan routed via(s) from the board")
    return phantom_segs, phantom_vias


def remove_route_from_pcb_data(pcb_data: PCBData, result: dict) -> None:
    """Remove routed segments and vias from PCB data (for rip-up and reroute)."""
    segments_to_remove = result.get('new_segments', [])
    vias_to_remove = result.get('new_vias', [])

    if not segments_to_remove and not vias_to_remove:
        return

    # Build sets of segment signatures (start, end, layer, net_id) for fast lookup
    seg_signatures = set()
    for seg in segments_to_remove:
        # Normalize segment direction (smaller point first)
        p1 = (round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS))
        p2 = (round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS))
        if p1 > p2:
            p1, p2 = p2, p1
        sig = (p1, p2, seg.layer, seg.net_id)
        seg_signatures.add(sig)

    # Build set of via signatures (x, y, net_id) for fast lookup
    via_signatures = set()
    for via in vias_to_remove:
        sig = (round(via.x, POSITION_DECIMALS), round(via.y, POSITION_DECIMALS), via.net_id)
        via_signatures.add(sig)

    # Remove matching segments
    new_segments = []
    removed_seg_count = 0
    for seg in pcb_data.segments:
        p1 = (round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS))
        p2 = (round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS))
        if p1 > p2:
            p1, p2 = p2, p1
        sig = (p1, p2, seg.layer, seg.net_id)
        if sig in seg_signatures:
            removed_seg_count += 1
        else:
            new_segments.append(seg)
    pcb_data.segments = new_segments

    # Remove matching vias
    new_vias = []
    removed_via_count = 0
    for via in pcb_data.vias:
        sig = (round(via.x, POSITION_DECIMALS), round(via.y, POSITION_DECIMALS), via.net_id)
        if sig in via_signatures:
            removed_via_count += 1
        else:
            new_vias.append(via)
    pcb_data.vias = new_vias


def remove_net_from_pcb_data(pcb_data: PCBData, net_id: int) -> Tuple[List[Segment], List[Via]]:
    """Remove all segments and vias for a net from pcb_data.

    This is a simpler alternative to remove_route_from_pcb_data() when you want
    to remove an entire net rather than specific segments/vias.

    Args:
        pcb_data: PCB data structure to modify
        net_id: Net ID to remove

    Returns:
        (removed_segments, removed_vias) - the removed elements for potential restoration
    """
    # Copper GRAPHICS (#337) are immutable input copper: the writer cannot
    # strip a gr_line from the file, so ripping them from pcb_data would break
    # board==file and a later restore would DUPLICATE them as (segment) copies.
    removed_segments = [s for s in pcb_data.segments
                        if s.net_id == net_id and not getattr(s, 'graphic', False)]
    removed_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    pcb_data.segments = [s for s in pcb_data.segments
                         if s.net_id != net_id or getattr(s, 'graphic', False)]
    pcb_data.vias = [v for v in pcb_data.vias if v.net_id != net_id]

    return removed_segments, removed_vias


def restore_net_to_pcb_data(pcb_data: PCBData, segments: List[Segment], vias: List[Via]) -> None:
    """Restore previously removed segments and vias to pcb_data.

    Args:
        pcb_data: PCB data structure to modify
        segments: Segments to restore
        vias: Vias to restore
    """
    pcb_data.segments.extend(segments)
    pcb_data.vias.extend(vias)
