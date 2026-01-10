"""
Route modification utilities for PCB routing.

Functions for adding/removing routes from PCB data and cleaning up
self-intersecting or redundant segments.
"""

import math
from typing import List, Optional, Tuple

from kicad_parser import PCBData, Segment, Via
from routing_utils import pos_key


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
            existing_endpoints_by_layer[seg.layer].add((round(seg.start_x, 3), round(seg.start_y, 3)))
            existing_endpoints_by_layer[seg.layer].add((round(seg.end_x, 3), round(seg.end_y, 3)))

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
                    trim_endpoint = (round(seg.start_x, 3), round(seg.start_y, 3))
                else:
                    trim_endpoint = (round(seg.end_x, 3), round(seg.end_y, 3))

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
        snap_pt_rounded = (round(snap_pt[0], 3), round(snap_pt[1], 3))

        # Find the upstream segment that connects at trim_endpoint
        upstream_idx = None
        for idx in endpoint_to_segs.get(trim_endpoint, []):
            if idx != crossing_idx and idx not in segments_to_remove:
                upstream_idx = idx
                break

        if upstream_idx is not None:
            upstream_seg = layer_segs[upstream_idx]
            up_start = (round(upstream_seg.start_x, 3), round(upstream_seg.start_y, 3))
            up_end = (round(upstream_seg.end_x, 3), round(upstream_seg.end_y, 3))
            up_other_end = up_end if up_start == trim_endpoint else up_start

            if up_other_end == snap_pt_rounded:
                # The "upstream" is actually the stub connector - remove it as redundant
                segments_to_remove.add(upstream_idx)
                segments_to_remove.add(crossing_idx)

                # Find the real upstream at the OTHER endpoint of crossing_seg
                crossing_start = (round(crossing_seg.start_x, 3), round(crossing_seg.start_y, 3))
                crossing_end = (round(crossing_seg.end_x, 3), round(crossing_seg.end_y, 3))
                source_endpoint = crossing_end if trim_endpoint == crossing_start else crossing_start

                real_upstream_idx = None
                for idx in endpoint_to_segs.get(source_endpoint, []):
                    if idx != crossing_idx and idx not in segments_to_remove:
                        real_upstream_idx = idx
                        break

                if real_upstream_idx is not None:
                    real_up_seg = layer_segs[real_upstream_idx]
                    ru_start = (round(real_up_seg.start_x, 3), round(real_up_seg.start_y, 3))
                    ru_end = (round(real_up_seg.end_x, 3), round(real_up_seg.end_y, 3))

                    if ru_end == source_endpoint:
                        segment_modifications[real_upstream_idx] = ('end', snap_pt[0], snap_pt[1])
                    elif ru_start == source_endpoint:
                        segment_modifications[real_upstream_idx] = ('start', snap_pt[0], snap_pt[1])
            else:
                # Normal case: extend upstream to snap_pt
                if up_end == trim_endpoint:
                    segment_modifications[upstream_idx] = ('end', snap_pt[0], snap_pt[1])
                elif up_start == trim_endpoint:
                    segment_modifications[upstream_idx] = ('start', snap_pt[0], snap_pt[1])

                segments_to_remove.add(crossing_idx)

                # Find and remove orphaned downstream segments
                crossing_start = (round(crossing_seg.start_x, 3), round(crossing_seg.start_y, 3))
                crossing_end = (round(crossing_seg.end_x, 3), round(crossing_seg.end_y, 3))
                downstream_endpoint = crossing_end if trim_endpoint == crossing_start else crossing_start

                visited_endpoints = {trim_endpoint, downstream_endpoint}
                to_check = [downstream_endpoint]

                while to_check:
                    pt = to_check.pop()
                    for idx in endpoint_to_segs.get(pt, []):
                        if idx in segments_to_remove or idx == crossing_idx:
                            continue
                        seg = layer_segs[idx]
                        seg_start = (round(seg.start_x, 3), round(seg.start_y, 3))
                        seg_end = (round(seg.end_x, 3), round(seg.end_y, 3))
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
            crossing_start = (round(crossing_seg.start_x, 3), round(crossing_seg.start_y, 3))
            crossing_end = (round(crossing_seg.end_x, 3), round(crossing_seg.end_y, 3))
            other_endpoint = crossing_end if crossing_start == trim_endpoint else crossing_start

            downstream_idx = None
            for idx in endpoint_to_segs.get(other_endpoint, []):
                if idx != crossing_idx and idx not in segments_to_remove:
                    downstream_idx = idx
                    break

            if downstream_idx is not None:
                downstream_seg = layer_segs[downstream_idx]
                ds_start = (round(downstream_seg.start_x, 3), round(downstream_seg.start_y, 3))
                ds_end = (round(downstream_seg.end_x, 3), round(downstream_seg.end_y, 3))

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
            for pt in [(round(seg.start_x, 3), round(seg.start_y, 3)),
                       (round(seg.end_x, 3), round(seg.end_y, 3))]:
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
    if existing_segments:
        for seg in existing_segments:
            if seg.layer not in existing_endpoints:
                existing_endpoints[seg.layer] = []
            existing_endpoints[seg.layer].append((seg.start_x, seg.start_y))
            existing_endpoints[seg.layer].append((seg.end_x, seg.end_y))

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
        """Check if point is within via_size/4 of any via in list."""
        for vx, vy, via_size in vias_list:
            tolerance = via_size / 4
            if math.sqrt((px - vx)**2 + (py - vy)**2) < tolerance:
                return True
        return False

    def point_near_any_pad(px, py, pads_list):
        """Check if point is within pad_size/4 of any pad in list."""
        for pad_x, pad_y, pad_size in pads_list:
            tolerance = pad_size / 4
            if math.sqrt((px - pad_x)**2 + (py - pad_y)**2) < tolerance:
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
            proximity_tol = seg.width / 4
            start_connects_existing = (point_near_any(seg.start_x, seg.start_y, layer_existing, proximity_tol) or
                                       point_near_any_via(seg.start_x, seg.start_y, layer_vias) or
                                       point_near_any_pad(seg.start_x, seg.start_y, layer_pads))
            end_connects_existing = (point_near_any(seg.end_x, seg.end_y, layer_existing, proximity_tol) or
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

    # Filter out very short (degenerate) segments
    def seg_len(s):
        return math.sqrt((s.end_x - s.start_x)**2 + (s.end_y - s.start_y)**2)
    cleaned_segments = [s for s in cleaned_segments if seg_len(s) > 0.01]

    for seg in cleaned_segments:
        pcb_data.segments.append(seg)
    for via in result['new_vias']:
        pcb_data.vias.append(via)
    # Update result so output file also gets cleaned segments
    result['new_segments'] = cleaned_segments


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
        p1 = (round(seg.start_x, 3), round(seg.start_y, 3))
        p2 = (round(seg.end_x, 3), round(seg.end_y, 3))
        if p1 > p2:
            p1, p2 = p2, p1
        sig = (p1, p2, seg.layer, seg.net_id)
        seg_signatures.add(sig)

    # Build set of via signatures (x, y, net_id) for fast lookup
    via_signatures = set()
    for via in vias_to_remove:
        sig = (round(via.x, 3), round(via.y, 3), via.net_id)
        via_signatures.add(sig)

    # Remove matching segments
    new_segments = []
    removed_seg_count = 0
    for seg in pcb_data.segments:
        p1 = (round(seg.start_x, 3), round(seg.start_y, 3))
        p2 = (round(seg.end_x, 3), round(seg.end_y, 3))
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
        sig = (round(via.x, 3), round(via.y, 3), via.net_id)
        if sig in via_signatures:
            removed_via_count += 1
        else:
            new_vias.append(via)
    pcb_data.vias = new_vias
