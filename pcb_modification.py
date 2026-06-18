"""
Route modification utilities for PCB routing.

Functions for adding/removing routes from PCB data and cleaning up
self-intersecting or redundant segments.
"""

import math
from typing import List, Optional, Tuple

from kicad_parser import PCBData, Segment, Via
from routing_utils import pos_key, POSITION_DECIMALS


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
    """Collapse short appendix segments by moving dead-end vertices to junction points.

    An appendix is a short segment where one endpoint is a dead-end (degree 1) and
    the other endpoint is a junction (degree >= 2). We collapse it by moving the
    dead-end to nearly coincide with the junction (offset by 0.001mm).

    Only collapses segments where the dead-end doesn't connect to existing segments, vias, or pads.
    Also fixes self-intersections where new segments cross existing segments.

    If debug_lines is True, endpoint degrees are counted across all layers.
    """
    if not segments:
        return segments

    # First fix self-intersections with existing segments
    segments = fix_self_intersections(segments, existing_segments, max_appendix_length, vias)

    # Build map of existing segment endpoints by layer (store actual coordinates for proximity check)
    existing_endpoints = {}
    # Also build map of full existing segments by layer (for point-on-segment check)
    existing_segs_by_layer = {}
    if existing_segments:
        for seg in existing_segments:
            if seg.layer not in existing_endpoints:
                existing_endpoints[seg.layer] = []
                existing_segs_by_layer[seg.layer] = []
            existing_endpoints[seg.layer].append((seg.start_x, seg.start_y))
            existing_endpoints[seg.layer].append((seg.end_x, seg.end_y))
            existing_segs_by_layer[seg.layer].append(seg)

    # Build map of via locations by layer (store actual coordinates and size for proximity check)
    via_locations = {}
    if vias:
        all_copper_layers = get_copper_layers_from_segments(segments, existing_segments)
        for via in vias:
            # Through-hole vias connect all layers
            if via.layers and 'F.Cu' in via.layers and 'B.Cu' in via.layers:
                via_layers = all_copper_layers
            elif via.layers:
                via_layers = via.layers
            else:
                via_layers = all_copper_layers
            via_size = getattr(via, 'size', 0.6)  # Default via size if not available
            for layer in via_layers:
                if layer not in via_locations:
                    via_locations[layer] = []
                via_locations[layer].append((via.x, via.y, via_size))

    # Build map of pad locations by layer (store coordinates and size)
    pad_locations = {}
    if pads:
        all_copper_layers = get_copper_layers_from_segments(segments, existing_segments)
        for pad in pads:
            # Get pad position
            pad_x = getattr(pad, 'global_x', getattr(pad, 'x', 0))
            pad_y = getattr(pad, 'global_y', getattr(pad, 'y', 0))
            # Get pad size for proximity check
            pad_size_x = getattr(pad, 'size_x', 0.5)
            pad_size_y = getattr(pad, 'size_y', 0.5)
            pad_size = max(pad_size_x, pad_size_y)
            # Expand wildcard layers like "*.Cu" to actual routing layers
            pad_layers = getattr(pad, 'layers', [])
            if any('*' in layer for layer in pad_layers):
                pad_layers = all_copper_layers
            for layer in pad_layers:
                if layer not in pad_locations:
                    pad_locations[layer] = []
                pad_locations[layer].append((pad_x, pad_y, pad_size))

    # Process each layer separately
    layer_segments = {}
    for seg in segments:
        if seg.layer not in layer_segments:
            layer_segments[seg.layer] = []
        layer_segments[seg.layer].append(seg)

    result_segments = []

    def point_near_any(px, py, points_list, tolerance):
        """Check if point is within tolerance of any point in list."""
        for ex, ey in points_list:
            if math.sqrt((px - ex)**2 + (py - ey)**2) < tolerance:
                return True
        return False

    def point_near_any_via(px, py, vias_list):
        """Check if point lands on any via's copper (within via radius)."""
        for vx, vy, via_size in vias_list:
            # The via's copper extends size/2 from centre; a segment ending
            # anywhere on it is connected. The old size/4 test misjudged
            # edge landings as dead ends and collapsed live connections.
            tolerance = via_size / 2 + 0.05
            if math.sqrt((px - vx)**2 + (py - vy)**2) < tolerance:
                return True
        return False

    def point_near_any_pad(px, py, pads_list):
        """Check if point lands on any pad's copper (within max half-extent)."""
        for pad_x, pad_y, pad_size in pads_list:
            # pad_size is max(size_x, size_y); copper reaches size/2 from
            # centre along the long axis. The old size/4 test judged a tap
            # landing near the end of an elongated pad (0.8 mm from a 1.6 mm
            # pad's centre) as a dead end and collapsed the live connection
            # to 0.001 mm. Over-keeping is harmless (an orphan stub at
            # worst); over-collapsing breaks connectivity.
            tolerance = pad_size / 2 + 0.05
            if math.sqrt((px - pad_x)**2 + (py - pad_y)**2) < tolerance:
                return True
        return False

    def point_on_any_segment(px, py, segs_list, tolerance):
        """Check if point lies on any segment (not just endpoints).

        Returns True if the point is within tolerance of any segment's line.
        This catches tap points that are in the middle of existing segments.
        """
        for seg in segs_list:
            # Vector from segment start to end
            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 0.0001:  # Degenerate segment
                continue
            # Vector from segment start to point
            px_rel = px - seg.start_x
            py_rel = py - seg.start_y
            # Project point onto segment line (parametric t)
            t = (px_rel * dx + py_rel * dy) / seg_len_sq
            # Check if projection is within segment bounds (with small margin)
            if t < -0.01 or t > 1.01:
                continue
            # Calculate closest point on segment
            closest_x = seg.start_x + t * dx
            closest_y = seg.start_y + t * dy
            # Check distance from point to closest point on segment
            dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
            if dist < tolerance:
                return True
        return False

    # When debug_lines is enabled, build endpoint degree map across ALL layers
    # because debug_lines puts turn segments on different layers but they still connect
    global_endpoint_counts = None
    if debug_lines:
        global_endpoint_counts = {}
        for seg in segments:
            start_key = (round(seg.start_x, 4), round(seg.start_y, 4))
            end_key = (round(seg.end_x, 4), round(seg.end_y, 4))
            global_endpoint_counts[start_key] = global_endpoint_counts.get(start_key, 0) + 1
            global_endpoint_counts[end_key] = global_endpoint_counts.get(end_key, 0) + 1

    for layer, layer_segs in layer_segments.items():
        layer_existing = existing_endpoints.get(layer, [])
        layer_existing_segs = existing_segs_by_layer.get(layer, [])
        layer_vias = via_locations.get(layer, [])
        layer_pads = pad_locations.get(layer, [])

        # Build per-layer endpoint counts (used when not in debug_lines mode)
        layer_endpoint_counts = None
        if not debug_lines:
            layer_endpoint_counts = {}
            for seg in layer_segs:
                start_key = (round(seg.start_x, 4), round(seg.start_y, 4))
                end_key = (round(seg.end_x, 4), round(seg.end_y, 4))
                layer_endpoint_counts[start_key] = layer_endpoint_counts.get(start_key, 0) + 1
                layer_endpoint_counts[end_key] = layer_endpoint_counts.get(end_key, 0) + 1

        # Find and collapse appendices
        for seg in layer_segs:
            length = math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)

            if length > max_appendix_length:
                result_segments.append(seg)
                continue

            start_key = (round(seg.start_x, 4), round(seg.start_y, 4))
            end_key = (round(seg.end_x, 4), round(seg.end_y, 4))
            endpoint_counts = global_endpoint_counts if debug_lines else layer_endpoint_counts
            start_degree = endpoint_counts.get(start_key, 0)
            end_degree = endpoint_counts.get(end_key, 0)

            # Check if endpoints connect to existing segments (with proximity tolerance), vias, or pads
            # Use track width / 4 as proximity tolerance for segments, via/pad size / 4 for vias/pads
            # Also check if point lies ON an existing segment (for tap points in middle of segments)
            proximity_tol = seg.width / 4
            start_connects_existing = (point_near_any(seg.start_x, seg.start_y, layer_existing, proximity_tol) or
                                       point_on_any_segment(seg.start_x, seg.start_y, layer_existing_segs, proximity_tol) or
                                       point_near_any_via(seg.start_x, seg.start_y, layer_vias) or
                                       point_near_any_pad(seg.start_x, seg.start_y, layer_pads))
            end_connects_existing = (point_near_any(seg.end_x, seg.end_y, layer_existing, proximity_tol) or
                                     point_on_any_segment(seg.end_x, seg.end_y, layer_existing_segs, proximity_tol) or
                                     point_near_any_via(seg.end_x, seg.end_y, layer_vias) or
                                     point_near_any_pad(seg.end_x, seg.end_y, layer_pads))

            # Appendix: one end is dead-end (degree 1, not connected to existing/vias/pads),
            # other is junction (degree >= 2 OR connected to existing/vias/pads)
            if (start_degree == 1 and not start_connects_existing and
                (end_degree >= 2 or end_connects_existing)):
                # Collapse: move start to nearly coincide with end (junction point)
                new_seg = Segment(
                    start_x=seg.end_x + 0.001,
                    start_y=seg.end_y,
                    end_x=seg.end_x,
                    end_y=seg.end_y,
                    width=seg.width,
                    layer=seg.layer,
                    net_id=seg.net_id
                )
                result_segments.append(new_seg)
            elif (end_degree == 1 and not end_connects_existing and
                  (start_degree >= 2 or start_connects_existing)):
                # Collapse: move end to nearly coincide with start (junction point)
                new_seg = Segment(
                    start_x=seg.start_x,
                    start_y=seg.start_y,
                    end_x=seg.start_x + 0.001,
                    end_y=seg.start_y,
                    width=seg.width,
                    layer=seg.layer,
                    net_id=seg.net_id
                )
                result_segments.append(new_seg)
            else:
                # Not an appendix or connects to existing - keep as is
                result_segments.append(seg)

    return result_segments


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
    rot = math.radians(getattr(pad, 'rotation', 0.0) or 0.0)
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
            d, _ = segment_to_rect_distance(x1, y1, x2, y2, pad.global_x, pad.global_y,
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
