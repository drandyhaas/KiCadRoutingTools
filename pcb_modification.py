"""
Route modification utilities for PCB routing.

Functions for adding/removing routes from PCB data and cleaning up
self-intersecting or redundant segments.
"""

import math
from typing import List, Optional, Tuple

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


def _segments_cross(seg1: Segment, seg2: Segment) -> Optional[Tuple[float, float]]:
    """Check if two segments cross (not just touch at endpoints). Returns crossing point or None."""
    # Line 1: P1 + t*(P2-P1), Line 2: P3 + u*(P4-P3)
    x1, y1 = seg1.start_x, seg1.start_y
    x2, y2 = seg1.end_x, seg1.end_y
    x3, y3 = seg2.start_x, seg2.start_y
    x4, y4 = seg2.end_x, seg2.end_y

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None  # Parallel

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom

    # Check if intersection is strictly inside both segments (not at endpoints)
    eps = 0.001  # Small margin to exclude endpoint touches
    if eps < t < (1 - eps) and eps < u < (1 - eps):
        cross_x = x1 + t * (x2 - x1)
        cross_y = y1 + t * (y2 - y1)
        return (cross_x, cross_y)
    return None


def _build_layer_context(
    segments: List[Segment],
    existing_segments: List[Segment],
    vias: List[Via]
) -> Tuple[dict, dict, dict, dict]:
    """
    Build per-layer data structures for crossing detection.

    Returns:
        Tuple of (existing_by_layer, existing_endpoints_by_layer, via_locations_by_layer, layer_segments)
    """
    existing_by_layer = {}
    existing_endpoints_by_layer = {}
    if existing_segments:
        for seg in existing_segments:
            if seg.layer not in existing_by_layer:
                existing_by_layer[seg.layer] = []
                existing_endpoints_by_layer[seg.layer] = set()
            existing_by_layer[seg.layer].append(seg)
            existing_endpoints_by_layer[seg.layer].add((round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS)))
            existing_endpoints_by_layer[seg.layer].add((round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS)))

    via_locations_by_layer = {}
    if vias:
        all_copper_layers = get_copper_layers_from_segments(segments, existing_segments)
        for via in vias:
            if via.layers and 'F.Cu' in via.layers and 'B.Cu' in via.layers:
                via_layers = all_copper_layers
            elif via.layers:
                via_layers = via.layers
            else:
                via_layers = all_copper_layers
            via_size = getattr(via, 'size', 0.6)
            for layer in via_layers:
                if layer not in via_locations_by_layer:
                    via_locations_by_layer[layer] = []
                via_locations_by_layer[layer].append((via.x, via.y, via_size))

    layer_segments = {}
    for seg in segments:
        if seg.layer not in layer_segments:
            layer_segments[seg.layer] = []
        layer_segments[seg.layer].append(seg)

    return existing_by_layer, existing_endpoints_by_layer, via_locations_by_layer, layer_segments


def _find_short_segment_crossings(
    layer_segs: List[Segment],
    existing_on_layer: List[Segment],
    max_short_length: float
) -> dict:
    """
    Find all crossings involving short new segments crossing existing segments.

    Returns:
        Dict mapping segment index -> (existing_seg, cross_pt, snap_pt, trim_endpoint)
    """
    segments_to_trim = {}

    for i, seg in enumerate(layer_segs):
        seg_len = math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)
        if seg_len > max_short_length:
            continue

        for existing in existing_on_layer:
            cross_pt = _segments_cross(seg, existing)
            if cross_pt:
                # Choose the existing endpoint closest to the crossing point
                dist_to_ex_start = math.sqrt((cross_pt[0] - existing.start_x)**2 +
                                             (cross_pt[1] - existing.start_y)**2)
                dist_to_ex_end = math.sqrt((cross_pt[0] - existing.end_x)**2 +
                                           (cross_pt[1] - existing.end_y)**2)
                if dist_to_ex_end < dist_to_ex_start:
                    snap_x, snap_y = existing.end_x, existing.end_y
                else:
                    snap_x, snap_y = existing.start_x, existing.start_y

                # Determine which endpoint of the short seg is closer to crossing
                dist_start_to_cross = math.sqrt((seg.start_x - cross_pt[0])**2 +
                                                (seg.start_y - cross_pt[1])**2)
                dist_end_to_cross = math.sqrt((seg.end_x - cross_pt[0])**2 +
                                              (seg.end_y - cross_pt[1])**2)
                if dist_start_to_cross < dist_end_to_cross:
                    trim_endpoint = (round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS))
                else:
                    trim_endpoint = (round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS))

                segments_to_trim[i] = (existing, cross_pt, (snap_x, snap_y), trim_endpoint)
                break

    return segments_to_trim


def _process_layer_crossings(
    layer_segs: List[Segment],
    segments_to_trim: dict,
    endpoint_to_segs: dict,
    layer_vias: List[Tuple[float, float, float]],
    layer_existing_endpoints: set
) -> Tuple[set, dict]:
    """
    Process all crossings for a layer and determine segments to remove/modify.

    Returns:
        Tuple of (segments_to_remove, segment_modifications)
    """
    def point_near_via(px, py, via_list):
        for vx, vy, via_size in via_list:
            if math.sqrt((px - vx)**2 + (py - vy)**2) < via_size / 4:
                return True
        return False

    segments_to_remove = set()
    segment_modifications = {}

    for crossing_idx, (existing, cross_pt, snap_pt, trim_endpoint) in segments_to_trim.items():
        crossing_seg = layer_segs[crossing_idx]
        snap_pt_rounded = (round(snap_pt[0], POSITION_DECIMALS), round(snap_pt[1], POSITION_DECIMALS))

        # Find the upstream segment that connects at trim_endpoint
        upstream_idx = None
        for idx in endpoint_to_segs.get(trim_endpoint, []):
            if idx != crossing_idx and idx not in segments_to_remove:
                upstream_idx = idx
                break

        if upstream_idx is not None:
            upstream_seg = layer_segs[upstream_idx]
            up_start = (round(upstream_seg.start_x, POSITION_DECIMALS), round(upstream_seg.start_y, POSITION_DECIMALS))
            up_end = (round(upstream_seg.end_x, POSITION_DECIMALS), round(upstream_seg.end_y, POSITION_DECIMALS))
            up_other_end = up_end if up_start == trim_endpoint else up_start

            if up_other_end == snap_pt_rounded:
                # The upstream connects from ~snap_pt to trim_endpoint and is
                # essentially redundant. Remove it and modify the crossing
                # segment to start/end at snap_pt instead of trim_endpoint.
                # This eliminates the crossing while preserving connectivity
                # to whatever is at the other end (via, pad, or more segments).
                segments_to_remove.add(upstream_idx)

                crossing_start = (round(crossing_seg.start_x, POSITION_DECIMALS), round(crossing_seg.start_y, POSITION_DECIMALS))
                crossing_end = (round(crossing_seg.end_x, POSITION_DECIMALS), round(crossing_seg.end_y, POSITION_DECIMALS))

                if trim_endpoint == crossing_start:
                    segment_modifications[crossing_idx] = ('start', snap_pt[0], snap_pt[1])
                elif trim_endpoint == crossing_end:
                    segment_modifications[crossing_idx] = ('end', snap_pt[0], snap_pt[1])
            else:
                # Normal case: extend upstream to snap_pt
                if up_end == trim_endpoint:
                    segment_modifications[upstream_idx] = ('end', snap_pt[0], snap_pt[1])
                elif up_start == trim_endpoint:
                    segment_modifications[upstream_idx] = ('start', snap_pt[0], snap_pt[1])

                segments_to_remove.add(crossing_idx)

                # Find and remove orphaned downstream segments
                crossing_start = (round(crossing_seg.start_x, POSITION_DECIMALS), round(crossing_seg.start_y, POSITION_DECIMALS))
                crossing_end = (round(crossing_seg.end_x, POSITION_DECIMALS), round(crossing_seg.end_y, POSITION_DECIMALS))
                downstream_endpoint = crossing_end if trim_endpoint == crossing_start else crossing_start

                visited_endpoints = {trim_endpoint, downstream_endpoint}
                to_check = [downstream_endpoint]

                while to_check:
                    pt = to_check.pop()
                    for idx in endpoint_to_segs.get(pt, []):
                        if idx in segments_to_remove or idx == crossing_idx:
                            continue
                        seg = layer_segs[idx]
                        seg_start = (round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS))
                        seg_end = (round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS))
                        other_end = seg_end if seg_start == pt else seg_start

                        if other_end == snap_pt_rounded:
                            segments_to_remove.add(idx)
                            continue

                        other_connections = [j for j in endpoint_to_segs.get(other_end, [])
                                             if j != idx and j not in segments_to_remove and j != crossing_idx]
                        connects_to_via = point_near_via(other_end[0], other_end[1], layer_vias)
                        connects_to_existing = other_end in layer_existing_endpoints
                        if not other_connections and not connects_to_via and not connects_to_existing:
                            segments_to_remove.add(idx)
                            if other_end not in visited_endpoints:
                                visited_endpoints.add(other_end)
                                to_check.append(other_end)
                        else:
                            if seg_end == pt:
                                segment_modifications[idx] = ('end', snap_pt[0], snap_pt[1])
                            else:
                                segment_modifications[idx] = ('start', snap_pt[0], snap_pt[1])
        else:
            # No upstream segment found - check the other endpoint
            crossing_start = (round(crossing_seg.start_x, POSITION_DECIMALS), round(crossing_seg.start_y, POSITION_DECIMALS))
            crossing_end = (round(crossing_seg.end_x, POSITION_DECIMALS), round(crossing_seg.end_y, POSITION_DECIMALS))
            other_endpoint = crossing_end if crossing_start == trim_endpoint else crossing_start

            downstream_idx = None
            for idx in endpoint_to_segs.get(other_endpoint, []):
                if idx != crossing_idx and idx not in segments_to_remove:
                    downstream_idx = idx
                    break

            if downstream_idx is not None:
                downstream_seg = layer_segs[downstream_idx]
                ds_start = (round(downstream_seg.start_x, POSITION_DECIMALS), round(downstream_seg.start_y, POSITION_DECIMALS))
                ds_end = (round(downstream_seg.end_x, POSITION_DECIMALS), round(downstream_seg.end_y, POSITION_DECIMALS))

                if ds_end == other_endpoint:
                    segment_modifications[downstream_idx] = ('end', snap_pt[0], snap_pt[1])
                elif ds_start == other_endpoint:
                    segment_modifications[downstream_idx] = ('start', snap_pt[0], snap_pt[1])

                segments_to_remove.add(crossing_idx)

    return segments_to_remove, segment_modifications


def _apply_segment_modifications(
    layer_segs: List[Segment],
    segments_to_remove: set,
    segment_modifications: dict
) -> List[Segment]:
    """Apply modifications and build result for a layer."""
    result = []
    for i, seg in enumerate(layer_segs):
        if i in segments_to_remove:
            continue
        if i in segment_modifications:
            mod = segment_modifications[i]
            if mod[0] == 'end':
                new_seg = Segment(
                    start_x=seg.start_x, start_y=seg.start_y,
                    end_x=mod[1], end_y=mod[2],
                    width=seg.width, layer=seg.layer, net_id=seg.net_id
                )
            else:  # 'start'
                new_seg = Segment(
                    start_x=mod[1], start_y=mod[2],
                    end_x=seg.end_x, end_y=seg.end_y,
                    width=seg.width, layer=seg.layer, net_id=seg.net_id
                )
            result.append(new_seg)
        else:
            result.append(seg)
    return result


def fix_self_intersections(segments: List[Segment], existing_segments: List[Segment] = None,
                           max_short_length: float = 1.0, vias: List[Via] = None) -> List[Segment]:
    """Fix self-intersections by trimming short connector segments that cross existing segments.

    When a short connector segment crosses an existing segment, we:
    1. Extend the upstream segment (the one leading to the crossing segment) to connect
       directly to the existing segment's endpoint
    2. Remove the crossing segment and any downstream segments that become orphaned

    This maintains connectivity while eliminating the crossing.

    Args:
        segments: New segments from routing
        existing_segments: Existing segments of the same net to check crossings against
        max_short_length: Maximum length for a segment to be considered "short"
        vias: Vias on this net to check for connectivity
    """
    if not segments:
        return segments

    # Build per-layer context
    existing_by_layer, existing_endpoints_by_layer, via_locations_by_layer, layer_segments = \
        _build_layer_context(segments, existing_segments, vias)

    result_segments = []

    for layer, layer_segs in layer_segments.items():
        existing_on_layer = existing_by_layer.get(layer, [])
        layer_vias = via_locations_by_layer.get(layer, [])
        layer_existing_endpoints = existing_endpoints_by_layer.get(layer, set())

        # Build connectivity map: endpoint -> list of segment indices
        endpoint_to_segs = {}
        for i, seg in enumerate(layer_segs):
            for pt in [(round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS)),
                       (round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS))]:
                if pt not in endpoint_to_segs:
                    endpoint_to_segs[pt] = []
                endpoint_to_segs[pt].append(i)

        # Find crossings
        segments_to_trim = _find_short_segment_crossings(layer_segs, existing_on_layer, max_short_length)

        # Process crossings
        segments_to_remove, segment_modifications = _process_layer_crossings(
            layer_segs, segments_to_trim, endpoint_to_segs, layer_vias, layer_existing_endpoints
        )

        # Apply modifications and build result
        layer_result = _apply_segment_modifications(layer_segs, segments_to_remove, segment_modifications)
        result_segments.extend(layer_result)

    return result_segments


def collapse_appendices(segments: List[Segment], existing_segments: List[Segment] = None,
                        max_appendix_length: float = 1.0, vias: List[Via] = None,
                        pads: List = None, debug_lines: bool = False) -> List[Segment]:
    """Clean a freshly-routed net's segments before they are committed.

    Fixes same-net self-intersections (straightens/deletes self-crossings; #147).

    Historically this ALSO collapsed short dead-end "appendix" spurs onto their
    junction, but that pass was removed (issue #148): the whole-net post-route
    dead-end trim sweep_dead_ends (#84) covers the same spurs, so the per-commit
    collapse was redundant. The name is kept for its call sites.
    """
    if not segments:
        return segments

    # First fix self-intersections with existing segments
    segments = fix_self_intersections(segments, existing_segments, max_appendix_length, vias)

    # Appendix-collapse removed (issue #148): sweep_dead_ends (#84) - the whole-net
    # post-route dead-end trim - covers the same short dead-end spurs this used to
    # slide onto their junction, so the per-commit collapse was redundant churn
    # (validated A/B across the set-3 corpus). Only the self-intersection fix
    # (kept; #147) remains. `pads`/`debug_lines` are accepted for call-site
    # compatibility and now unused.
    return segments


def _point_anchored(x: float, y: float, layer: str, via_pts, pad_pts,
                    all_segs, ignore_seg, tol: float) -> bool:
    """A segment endpoint is anchored if it lands on a same-net via (vias span
    layers), on a same-net pad (on a shared layer), or in the middle of another
    same-net segment on the same layer (a T-junction). Anchored endpoints are
    real connections, never dead ends."""
    for vx, vy, vsize in via_pts:
        if math.hypot(x - vx, y - vy) < vsize / 2 + 0.05:
            return True
    for px, py, psize, players in pad_pts:
        on_layer = (not players) or layer in players or any('*' in L for L in players)
        if on_layer and math.hypot(x - px, y - py) < psize / 2 + 0.05:
            return True
    for s in all_segs:
        if s is ignore_seg or s.layer != layer:
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

    Unlike ``collapse_appendices`` (single pass, only appendices <= 1 mm anchored
    to a junction) this removes dead ends of any length and unwinds whole spurs.

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

    kept = list(prunable)
    removed = []
    changed = True
    while changed:
        changed = False
        all_segs = kept + anchor_segments
        degree = {}
        for s in all_segs:
            degree[key(s.start_x, s.start_y, s.layer)] = \
                degree.get(key(s.start_x, s.start_y, s.layer), 0) + 1
            degree[key(s.end_x, s.end_y, s.layer)] = \
                degree.get(key(s.end_x, s.end_y, s.layer), 0) + 1

        survivors = []
        for s in kept:
            sk = key(s.start_x, s.start_y, s.layer)
            ek = key(s.end_x, s.end_y, s.layer)
            start_free = (degree[sk] == 1 and
                          not _point_anchored(s.start_x, s.start_y, s.layer,
                                              via_pts, pad_pts, all_segs, s, tol))
            end_free = (degree[ek] == 1 and
                        not _point_anchored(s.end_x, s.end_y, s.layer,
                                            via_pts, pad_pts, all_segs, s, tol))
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


def _safe_prune_net(net_id, prunable, vias, pads, zones,
                    anchor_segments=None, aggressive=False, tol=0.05):
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
    _, candidates = prune_dead_end_segments(prunable, anchor_segments=anchor_segments,
                                            vias=vias, pads=pads, tol=tol,
                                            keep_terminal_escapes=not aggressive)
    if not candidates:
        return prunable, []

    from check_connected import check_net_connectivity
    anchor = anchor_segments or []

    def disconnected(segs):
        return len(check_net_connectivity(net_id, anchor + segs, vias, pads, zones)['disconnected_pads'])

    base = disconnected(list(prunable))
    kept = list(prunable)
    removed = []
    # Removing a dead end can expose its neighbour as a new dead end (a chain
    # unwinds one segment at a time), so iterate: re-derive candidates from what
    # is left until a full pass removes nothing. A candidate whose removal would
    # strand a pad stays load-bearing no matter what other dead copper goes, so
    # cache those rejections and never re-test them (keeps it O(dead ends) and
    # guarantees termination).
    rejected = set()
    while True:
        progress = False
        for c in candidates:
            if id(c) in rejected or c not in kept:
                continue
            trial = [s for s in kept if s is not c]
            if disconnected(trial) <= base:
                kept = trial
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

    # Same-net copper grouped for fast lookup.
    segs_by_net_layer = {}
    for s in pcb_data.segments:
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
        results.append({'new_segments': new_conns, 'new_vias': []})
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


def clean_plane_copper(output_file: str, plane_net_names, clearance: float = 0.1) -> Tuple[int, int]:
    """Apply the dead-end sweep + gap-snap to a plane tool's output file (issue #84).

    route_planes / route_disconnected_planes write copper outside route.py's
    write-list, so they miss the in-route cleanup, leaving dead-end stubs (and the
    occasional near-miss) on plane nets. This re-parses the written board, snaps
    small same-net gaps and trims gated-safe dead ends on the given plane nets,
    then rewrites the file (stripping removed segments, appending snap connectors).
    The connectivity gate uses the plane zones, and the snap clearance check
    rejects any connector that would enter another net's pour. Returns
    ``(snapped, removed)``.
    """
    from types import SimpleNamespace
    from kicad_parser import parse_kicad_pcb, is_kicad_10
    from kicad_writer import remove_segments_from_content, generate_segment_sexpr

    pcb = parse_kicad_pcb(output_file)
    with open(output_file, 'r', encoding='utf-8') as f:
        content = f.read()
    names = set(plane_net_names)
    scope = {nid for nid, net in pcb.nets.items() if net.name in names}
    if not scope:
        return 0, 0

    snap_results = []
    snapped = snap_stub_gaps(snap_results, pcb, scope, SimpleNamespace(clearance=clearance))
    connectors = [s for r in snap_results for s in (r.get('new_segments') or [])]
    _, _, to_remove = sweep_dead_ends(snap_results, pcb, scope)
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


def sweep_dead_ends(results, pcb_data: PCBData, scope_net_ids=None,
                    tol: float = 0.05) -> Tuple[int, int, List[Segment]]:
    """Final whole-net dead-end sweep, after routing has settled (issue #84).

    ``collapse_appendices`` runs per route-commit and only trims short appendices
    anchored to a junction, so dead ends survive on nets that otherwise route
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
                supported = any(math.hypot(v.x - ex, v.y - ey) < tol for ex, ey in endpoints) \
                    or any(math.hypot(v.x - px, v.y - py) < ps / 2 + 0.05 for px, py, ps in pad_pts)
                if not supported:
                    removed_via_ids.add(id(v))
    if removed_via_ids:
        for r in results:
            vias = r.get('new_vias')
            if vias:
                r['new_vias'] = [v for v in vias if id(v) not in removed_via_ids]

    segs_removed = len(removed_routed_ids) + len(original_to_remove)
    return segs_removed, len(removed_via_ids), original_to_remove


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
                      foreign, clearance: float):
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
    tol = 0.02  # endpoint coincidence tolerance (mm), matching check_connected

    def grazes(s):
        hw = s.width / 2.0
        for cx, cy, rad, n, layers in foreign:
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
    for a in range(n):
        xa, ya, la = ports[a][0], ports[a][1], ports[a][2]
        for b in range(a + 1, n):
            if ports[b][2] == la and abs(ports[b][0] - xa) < tol and abs(ports[b][1] - ya) < tol:
                punion(a, b)

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
    cur = list(net_segs)
    safe_removed = []
    for s in sorted(removed, key=lambda s: (not grazes(s),
                    -math.hypot(s.end_x - s.start_x, s.end_y - s.start_y))):
        trial = [x for x in cur if x is not s]
        t = check_net_connectivity(net_id, trial, net_vias, net_pads)
        if t.get('connected') and (t.get('num_components') or 1) <= base_comps \
           and len(t.get('disconnected_pads') or []) <= base_disc:
            cur = trial
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

    zoned_nets = {z.net_id for z in (getattr(pcb_data, 'zones', []) or [])}
    segs_by_net = defaultdict(list)
    for s in pcb_data.segments:
        if scope_net_ids is None or s.net_id in scope_net_ids:
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
                                          net_pads, foreign, clearance)
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
        cleaned = collapse_appendices(net_segs, existing_segments, vias=net_vias, pads=net_pads, debug_lines=debug_lines)
        cleaned_segments.extend(cleaned)

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
    # board-wide settle). collapse_appendices above only trims short junction
    # appendices; this unwinds longer spurs and chains via prune_dead_end_segments.
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


def drop_phantom_copper(results, pcb_data: PCBData) -> Tuple[int, int]:
    """Drop, from each result's write-list copper, any segment/via no longer on the board.

    A result's ``new_segments`` / ``new_vias`` hold the SAME objects that
    ``add_route_to_pcb_data`` appended to ``pcb_data``; ``remove_route_from_pcb_data``
    drops those objects when a net is ripped. But a result snapshot taken before a
    rip-reroute can keep referencing copper that was later ripped and not restored
    (e.g. a multipoint net's ``completed_result``, built before ``try_phase3_ripup``
    ripped that net's own main route out from under it, is still committed). The
    output is written from these results, so the phantom copper lands on the board --
    including a DIFFERENT-net via at a cell another net legitimately took while this
    net was ripped, an un-manufacturable drill-on-drill short (issue #133:
    EPHY_TX_N / EPHY_RX_P escape vias).

    Membership is by object identity, so a re-cleaned or re-placed object (same
    position, different object) is never confused with the ripped one, and live
    copper is never dropped. Mutates each result in place; returns
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
    removed_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    removed_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    pcb_data.segments = [s for s in pcb_data.segments if s.net_id != net_id]
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
