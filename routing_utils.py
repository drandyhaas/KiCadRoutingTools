"""
Shared utility functions for PCB routing.

Includes connectivity analysis, endpoint finding, MPS ordering, and segment cleanup.
"""

import math
import fnmatch
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import PCBData, Segment, Via, Pad
from routing_config import GridRouteConfig, GridCoord, DiffPair
from chip_boundary import (
    build_chip_list, identify_chip_for_point, compute_far_side,
    compute_boundary_position, crossings_from_boundary_order
)


# Position rounding precision for coordinate comparisons
# All position-based lookups must use this to ensure consistency
POSITION_DECIMALS = 3


def pos_key(x: float, y: float) -> Tuple[float, float]:
    """
    Normalize coordinates for position-based lookups.

    Use this consistently when building position sets or checking position membership
    to avoid floating-point comparison issues.
    """
    return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS))


def extract_diff_pair_base(net_name: str) -> Optional[Tuple[str, bool]]:
    """
    Extract differential pair base name and polarity from net name.

    Looks for common differential pair naming conventions:
    - name_P / name_N
    - nameP / nameN
    - name+ / name-

    Returns (base_name, is_positive) or None if not a diff pair.
    """
    if not net_name:
        return None

    # Try _P/_N suffix (most common for LVDS)
    if net_name.endswith('_P'):
        return (net_name[:-2], True)
    if net_name.endswith('_N'):
        return (net_name[:-2], False)

    # Try P/N suffix without underscore
    if net_name.endswith('P') and len(net_name) > 1:
        # Check it's not just ending in P as part of name
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], True)
    if net_name.endswith('N') and len(net_name) > 1:
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], False)

    # Try +/- suffix
    if net_name.endswith('+'):
        return (net_name[:-1], True)
    if net_name.endswith('-'):
        return (net_name[:-1], False)

    return None


def find_differential_pairs(pcb_data: PCBData, patterns: List[str]) -> Dict[str, DiffPair]:
    """
    Find all differential pairs in the PCB matching the given glob patterns.

    Args:
        pcb_data: PCB data with net information
        patterns: Glob patterns for nets to treat as diff pairs (e.g., '*lvds*')

    Returns:
        Dict mapping base_name to DiffPair with complete P/N pairs
    """
    pairs: Dict[str, DiffPair] = {}

    # Collect all net names from pcb_data
    for net_id, net in pcb_data.nets.items():
        net_name = net.name
        if not net_name or net_id == 0:
            continue

        # Check if this net matches any diff pair pattern
        matched = any(fnmatch.fnmatch(net_name, pattern) for pattern in patterns)
        if not matched:
            continue

        # Try to extract diff pair info
        result = extract_diff_pair_base(net_name)
        if result is None:
            continue

        base_name, is_p = result

        if base_name not in pairs:
            pairs[base_name] = DiffPair(base_name=base_name)

        if is_p:
            pairs[base_name].p_net_id = net_id
            pairs[base_name].p_net_name = net_name
        else:
            pairs[base_name].n_net_id = net_id
            pairs[base_name].n_net_name = net_name

    # Filter to only complete pairs
    complete_pairs = {k: v for k, v in pairs.items() if v.is_complete}

    return complete_pairs


def find_single_ended_nets(
    pcb_data: PCBData,
    patterns: List[str],
    exclude_net_ids: Set[int] = None
) -> List[Tuple[str, int]]:
    """
    Find all single-ended nets matching the given glob patterns.

    Args:
        pcb_data: PCB data with net information
        patterns: Glob patterns for nets (e.g., 'Net-(U2A-BE*)')
        exclude_net_ids: Net IDs to exclude (e.g., diff pair net IDs)

    Returns:
        List of (net_name, net_id) tuples for matching single-ended nets
    """
    if exclude_net_ids is None:
        exclude_net_ids = set()

    result = []
    for net_id, net in pcb_data.nets.items():
        net_name = net.name
        if not net_name or net_id == 0:
            continue

        # Skip excluded nets (e.g., diff pair nets)
        if net_id in exclude_net_ids:
            continue

        # Check if this net matches any pattern
        matched = any(fnmatch.fnmatch(net_name, pattern) for pattern in patterns)
        if matched:
            result.append((net_name, net_id))

    return result


def find_connected_groups(segments: List[Segment], tolerance: float = 0.01) -> List[List[Segment]]:
    """Find groups of connected segments using union-find."""
    if not segments:
        return []

    n = len(segments)
    parent = list(range(n))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    # Check all pairs for shared endpoints
    for i in range(n):
        for j in range(i + 1, n):
            si, sj = segments[i], segments[j]
            pts_i = [(si.start_x, si.start_y), (si.end_x, si.end_y)]
            pts_j = [(sj.start_x, sj.start_y), (sj.end_x, sj.end_y)]
            for pi in pts_i:
                for pj in pts_j:
                    if abs(pi[0] - pj[0]) < tolerance and abs(pi[1] - pj[1]) < tolerance:
                        union(i, j)
                        break

    # Group segments by their root
    groups: Dict[int, List[Segment]] = {}
    for i in range(n):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


def find_stub_free_ends(segments: List[Segment], pads: List[Pad], tolerance: float = 0.05) -> List[Tuple[float, float, str]]:
    """
    Find the free ends of a segment group (endpoints not connected to other segments or pads).

    Args:
        segments: Connected group of segments
        pads: Pads that might be connected to the segments
        tolerance: Distance tolerance for considering points as connected

    Returns:
        List of (x, y, layer) tuples for free endpoints
    """
    if not segments:
        return []

    # Count how many times each endpoint appears
    endpoint_counts: Dict[Tuple[float, float, str], int] = {}
    for seg in segments:
        key_start = (round(seg.start_x, 3), round(seg.start_y, 3), seg.layer)
        key_end = (round(seg.end_x, 3), round(seg.end_y, 3), seg.layer)
        endpoint_counts[key_start] = endpoint_counts.get(key_start, 0) + 1
        endpoint_counts[key_end] = endpoint_counts.get(key_end, 0) + 1

    # Get pad positions
    pad_positions = [(round(p.global_x, 3), round(p.global_y, 3)) for p in pads]

    # Free ends are endpoints that appear only once AND are not near a pad
    free_ends = []
    for (x, y, layer), count in endpoint_counts.items():
        if count == 1:
            # Check if near a pad
            near_pad = False
            for px, py in pad_positions:
                if abs(x - px) < tolerance and abs(y - py) < tolerance:
                    near_pad = True
                    break
            if not near_pad:
                free_ends.append((x, y, layer))

    return free_ends


def get_stub_direction(segments: List[Segment], stub_x: float, stub_y: float, stub_layer: str,
                       tolerance: float = 0.05) -> Tuple[float, float]:
    """
    Find the direction a stub is pointing (from pad toward free end).

    Args:
        segments: List of segments for the net
        stub_x, stub_y: Position of the stub free end
        stub_layer: Layer of the stub
        tolerance: Distance tolerance for matching endpoints

    Returns:
        (dx, dy): Normalized direction vector pointing in the stub direction
    """
    # Find the segment that has this stub endpoint
    for seg in segments:
        if seg.layer != stub_layer:
            continue

        # Check if start matches stub position
        if abs(seg.start_x - stub_x) < tolerance and abs(seg.start_y - stub_y) < tolerance:
            # Direction from end to start (toward the free end)
            dx = seg.start_x - seg.end_x
            dy = seg.start_y - seg.end_y
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                return (dx / length, dy / length)
            return (0, 0)

        # Check if end matches stub position
        if abs(seg.end_x - stub_x) < tolerance and abs(seg.end_y - stub_y) < tolerance:
            # Direction from start to end (toward the free end)
            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                return (dx / length, dy / length)
            return (0, 0)

    # Fallback - no matching segment found
    return (0, 0)


def get_stub_segments(pcb_data: PCBData, net_id: int, stub_x: float, stub_y: float,
                      stub_layer: str, tolerance: float = 0.05) -> List[Segment]:
    """
    Get all segments that form a stub (from free end back to pad).

    Walks backwards from the stub free end through connected segments until
    reaching a pad or the segment chain ends.

    Args:
        pcb_data: PCB data containing segments and pads
        net_id: Net ID of the stub
        stub_x, stub_y: Position of the stub free end
        stub_layer: Layer of the stub
        tolerance: Distance tolerance for matching endpoints

    Returns:
        List of segments forming the stub, ordered from free end to pad
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id and s.layer == stub_layer]
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    pad_positions = [(p.global_x, p.global_y) for p in net_pads]

    result = []
    visited = set()
    current_x, current_y = stub_x, stub_y

    while True:
        # Find segment connected at current position
        found = None
        for seg in net_segments:
            if id(seg) in visited:
                continue

            # Check if start matches current position
            if abs(seg.start_x - current_x) < tolerance and abs(seg.start_y - current_y) < tolerance:
                found = seg
                next_x, next_y = seg.end_x, seg.end_y
                break

            # Check if end matches current position
            if abs(seg.end_x - current_x) < tolerance and abs(seg.end_y - current_y) < tolerance:
                found = seg
                next_x, next_y = seg.start_x, seg.start_y
                break

        if found is None:
            break

        result.append(found)
        visited.add(id(found))

        # Check if next position is at a pad (we've reached the pad end)
        at_pad = False
        for px, py in pad_positions:
            if abs(next_x - px) < tolerance and abs(next_y - py) < tolerance:
                at_pad = True
                break

        if at_pad:
            break

        current_x, current_y = next_x, next_y

    return result


def get_net_endpoints(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                      use_stub_free_ends: bool = False) -> Tuple[List, List, str]:
    """
    Find source and target endpoints for a net, considering segments, pads, and existing vias.

    Args:
        pcb_data: PCB data
        net_id: Net ID to find endpoints for
        config: Grid routing configuration
        use_stub_free_ends: If True, use only stub free ends (for diff pairs).
                           If False, use all segment endpoints (for single-ended).

    Returns:
        (sources, targets, error_message)
        - sources: List of (gx, gy, layer_idx, orig_x, orig_y)
        - targets: List of (gx, gy, layer_idx, orig_x, orig_y)
        - error_message: None if successful, otherwise describes why routing can't proceed
    """
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    # Case 1: Multiple segment groups
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments)
        if len(groups) >= 2:
            groups.sort(key=len, reverse=True)
            source_segs = groups[0]
            target_segs = groups[1]

            if use_stub_free_ends:
                # For diff pairs: use only stub free ends (tips not connected to pads)
                source_free_ends = find_stub_free_ends(source_segs, net_pads)
                target_free_ends = find_stub_free_ends(target_segs, net_pads)

                sources = []
                for x, y, layer in source_free_ends:
                    layer_idx = layer_map.get(layer)
                    if layer_idx is not None:
                        gx, gy = coord.to_grid(x, y)
                        sources.append((gx, gy, layer_idx, x, y))

                targets = []
                for x, y, layer in target_free_ends:
                    layer_idx = layer_map.get(layer)
                    if layer_idx is not None:
                        gx, gy = coord.to_grid(x, y)
                        targets.append((gx, gy, layer_idx, x, y))
            else:
                # For single-ended: use all segment endpoints
                sources = []
                for seg in source_segs:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                targets = []
                for seg in target_segs:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        targets.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            targets.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

            if sources and targets:
                return sources, targets, None

    # Case 2: One segment group + unconnected pads
    if len(net_segments) >= 1 and len(net_pads) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            # Check if any pad is NOT connected to the segment group
            seg_group = groups[0]
            seg_points = set()
            for seg in seg_group:
                seg_points.add((round(seg.start_x, 3), round(seg.start_y, 3)))
                seg_points.add((round(seg.end_x, 3), round(seg.end_y, 3)))

            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, 3), round(pad.global_y, 3))
                # Check if pad is near any segment point
                connected = False
                for sp in seg_points:
                    if abs(pad_pos[0] - sp[0]) < 0.05 and abs(pad_pos[1] - sp[1]) < 0.05:
                        connected = True
                        break
                if not connected:
                    unconnected_pads.append(pad)

            if unconnected_pads:
                # Use all segment endpoints as source, unconnected pad(s) as target
                sources = []
                for seg in seg_group:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                targets = []
                for pad in unconnected_pads:
                    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                    for layer in pad.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

                # Add existing vias as endpoints - they can be routed to on any layer
                # Determine if via is near stub (add to sources) or near unconnected pads (add to targets)
                # Note: All vias are through-hole, connecting ALL routing layers
                for via in net_vias:
                    gx, gy = coord.to_grid(via.x, via.y)
                    # Check if via is near any unconnected pad
                    near_unconnected = False
                    for pad in unconnected_pads:
                        if abs(via.x - pad.global_x) < 0.1 and abs(via.y - pad.global_y) < 0.1:
                            near_unconnected = True
                            break
                    # Add via as endpoint on ALL routing layers (vias are through-hole)
                    for layer in config.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            if near_unconnected:
                                targets.append((gx, gy, layer_idx, via.x, via.y))
                            else:
                                sources.append((gx, gy, layer_idx, via.x, via.y))

                if sources and targets:
                    return sources, targets, None

    # Case 3: No segments, just pads - route between pads
    if len(net_segments) == 0 and len(net_pads) >= 2:
        # Use first pad as source, rest as targets
        sources = []
        pad = net_pads[0]
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        for layer in pad.layers:
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                sources.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

        targets = []
        for pad in net_pads[1:]:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            for layer in pad.layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

        if sources and targets:
            return sources, targets, None

    # Case 4: Single segment, check if it connects two pads already
    if len(net_segments) == 1 and len(net_pads) >= 2:
        # Segment already connects pads - nothing to route
        return [], [], "Net has 1 segment connecting pads - already routed"

    # Determine why we can't route
    if len(net_segments) == 0 and len(net_pads) < 2:
        return [], [], f"Net has no segments and only {len(net_pads)} pad(s) - need at least 2 endpoints"
    if len(net_segments) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            return [], [], "Net segments are already connected (single group) with no unconnected pads"

    return [], [], f"Cannot determine endpoints: {len(net_segments)} segments, {len(net_pads)} pads"


def normalize_endpoints_by_component(
    pcb_data: PCBData,
    sources: List,
    targets: List,
    net_id: int
) -> Tuple[List, List]:
    """
    Normalize source/target endpoints so that source is always from the
    alphabetically-first component. This ensures consistent ordering across
    all nets for crossing detection and distance calculations.

    Args:
        pcb_data: PCB data with pad information
        sources: List of source endpoints (gx, gy, layer_idx, orig_x, orig_y)
        targets: List of target endpoints (gx, gy, layer_idx, orig_x, orig_y)
        net_id: Net ID for looking up pads

    Returns:
        (normalized_sources, normalized_targets) - may be swapped if needed
    """
    if not sources or not targets:
        return sources, targets

    net_pads = pcb_data.pads_by_net.get(net_id, [])
    if len(net_pads) < 2:
        return sources, targets

    # Find which component each endpoint is connected to
    def find_component_for_endpoint(endpoint):
        """Find the component_ref for the pad nearest to this endpoint."""
        ex, ey = endpoint[3], endpoint[4]  # orig_x, orig_y
        for pad in net_pads:
            if abs(pad.global_x - ex) < 1.0 and abs(pad.global_y - ey) < 1.0:
                return pad.component_ref
        return None

    src_component = find_component_for_endpoint(sources[0])
    tgt_component = find_component_for_endpoint(targets[0])

    if src_component and tgt_component:
        # Sort alphabetically - first component is source
        if src_component > tgt_component:
            # Swap so alphabetically-first component is source
            return targets, sources

    return sources, targets


def get_all_unrouted_net_ids(pcb_data: PCBData) -> List[int]:
    """
    Find all net IDs in the PCB that have unrouted stubs (multiple disconnected segment groups).
    These are nets that have partial routing but aren't fully connected.
    """
    unrouted_ids = []

    # Group segments by net ID
    net_segments: Dict[int, List[Segment]] = {}
    for seg in pcb_data.segments:
        if seg.net_id not in net_segments:
            net_segments[seg.net_id] = []
        net_segments[seg.net_id].append(seg)

    # Check each net for multiple disconnected groups
    for net_id, segments in net_segments.items():
        if net_id == 0:  # Skip unassigned net
            continue
        if len(segments) < 2:
            continue
        groups = find_connected_groups(segments)
        if len(groups) >= 2:
            # Net has multiple disconnected stub groups = unrouted
            unrouted_ids.append(net_id)

    return unrouted_ids


def get_stub_endpoints(pcb_data: PCBData, net_ids: List[int]) -> List[Tuple[float, float, str]]:
    """Get free end positions of unrouted net stubs for proximity avoidance.

    Returns list of (x, y, layer) tuples - includes layer for same-layer filtering.
    """
    stubs = []
    for net_id in net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        if len(net_segments) < 2:
            continue
        groups = find_connected_groups(net_segments)
        if len(groups) < 2:
            continue
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        for group in groups:
            free_ends = find_stub_free_ends(group, net_pads)
            if free_ends:
                # Include all free ends with their layers (for same-layer filtering)
                for fe_x, fe_y, fe_layer in free_ends:
                    stubs.append((fe_x, fe_y, fe_layer))
    return stubs


def get_net_stub_centroids(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float]]:
    """
    Get centroids of each connected stub group for a net.
    Returns list of (x, y) centroids, typically 2 for a 2-point net.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    if len(net_segments) < 2:
        return []
    groups = find_connected_groups(net_segments)
    if len(groups) < 2:
        return []

    centroids = []
    for group in groups:
        points = []
        for seg in group:
            points.append((seg.start_x, seg.start_y))
            points.append((seg.end_x, seg.end_y))
        if points:
            cx = sum(p[0] for p in points) / len(points)
            cy = sum(p[1] for p in points) / len(points)
            centroids.append((cx, cy))
    return centroids


def get_net_routing_endpoints(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float]]:
    """
    Get the two routing endpoints for a net, for MPS conflict detection.

    This handles multiple cases:
    1. Two stub groups -> use stub centroids
    2. One stub group + unconnected pads -> use stub centroid + pad centroid
    3. No stubs, just pads -> use pad positions

    Returns list of (x, y) positions, typically 2 for source and target.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])

    # Case 1: Multiple stub groups - use their centroids
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments)
        if len(groups) >= 2:
            centroids = []
            for group in groups:
                points = []
                for seg in group:
                    points.append((seg.start_x, seg.start_y))
                    points.append((seg.end_x, seg.end_y))
                if points:
                    cx = sum(p[0] for p in points) / len(points)
                    cy = sum(p[1] for p in points) / len(points)
                    centroids.append((cx, cy))
            return centroids[:2]

    # Case 2: One stub group + pads - find unconnected pads
    if len(net_segments) >= 1 and len(net_pads) >= 1:
        groups = find_connected_groups(net_segments)
        if len(groups) == 1:
            # Get stub centroid
            group = groups[0]
            seg_points = set()
            for seg in group:
                seg_points.add((round(seg.start_x, 3), round(seg.start_y, 3)))
                seg_points.add((round(seg.end_x, 3), round(seg.end_y, 3)))

            stub_pts = []
            for seg in group:
                stub_pts.append((seg.start_x, seg.start_y))
                stub_pts.append((seg.end_x, seg.end_y))
            stub_cx = sum(p[0] for p in stub_pts) / len(stub_pts)
            stub_cy = sum(p[1] for p in stub_pts) / len(stub_pts)

            # Find unconnected pads
            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, 3), round(pad.global_y, 3))
                connected = False
                for sp in seg_points:
                    if abs(pad_pos[0] - sp[0]) < 0.05 and abs(pad_pos[1] - sp[1]) < 0.05:
                        connected = True
                        break
                if not connected:
                    unconnected_pads.append(pad)

            if unconnected_pads:
                # Compute centroid of unconnected pads
                pad_cx = sum(p.global_x for p in unconnected_pads) / len(unconnected_pads)
                pad_cy = sum(p.global_y for p in unconnected_pads) / len(unconnected_pads)
                return [(stub_cx, stub_cy), (pad_cx, pad_cy)]

    # Case 3: No stubs, just pads - use first pad and centroid of rest
    if len(net_segments) == 0 and len(net_pads) >= 2:
        first_pad = net_pads[0]
        other_pads = net_pads[1:]
        other_cx = sum(p.global_x for p in other_pads) / len(other_pads)
        other_cy = sum(p.global_y for p in other_pads) / len(other_pads)
        return [(first_pad.global_x, first_pad.global_y), (other_cx, other_cy)]

    return []


def find_containing_or_nearest_bga_zone(
    point: Tuple[float, float],
    bga_zones: List[Tuple[float, float, float, float]]
) -> Optional[Tuple[float, float, float, float]]:
    """
    Find the BGA zone containing a point, or the nearest zone if outside all zones.

    Args:
        point: (x, y) position
        bga_zones: List of (min_x, min_y, max_x, max_y) BGA exclusion zones

    Returns:
        The containing/nearest BGA zone, or None if no zones provided
    """
    if not bga_zones:
        return None

    x, y = point

    # First check if inside any zone
    for zone in bga_zones:
        min_x, min_y, max_x, max_y = zone
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return zone

    # Not inside any zone - find nearest
    best_zone = None
    best_dist = float('inf')

    for zone in bga_zones:
        min_x, min_y, max_x, max_y = zone
        # Distance to bounding box
        dx = max(min_x - x, 0, x - max_x)
        dy = max(min_y - y, 0, y - max_y)
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < best_dist:
            best_dist = dist
            best_zone = zone

    return best_zone


def get_source_chip_center(
    pcb_data: PCBData,
    source_pads: List[Pad]
) -> Optional[Tuple[float, float]]:
    """
    Get the center of the source chip/component from source pads.

    Args:
        pcb_data: PCB data with footprint information
        source_pads: List of pads belonging to the source stub

    Returns:
        (x, y) center of the source component, or None if not found
    """
    if not source_pads:
        return None

    # Get component reference from the first pad
    component_ref = source_pads[0].component_ref
    if not component_ref:
        # Fall back to centroid of pads if no component_ref
        cx = sum(p.global_x for p in source_pads) / len(source_pads)
        cy = sum(p.global_y for p in source_pads) / len(source_pads)
        return (cx, cy)

    # Look up footprint bounds
    footprint = pcb_data.footprints.get(component_ref)
    if footprint and footprint.pads:
        # Calculate center from pad extents
        pad_xs = [p.global_x for p in footprint.pads]
        pad_ys = [p.global_y for p in footprint.pads]
        center_x = (min(pad_xs) + max(pad_xs)) / 2
        center_y = (min(pad_ys) + max(pad_ys)) / 2
        return (center_x, center_y)

    # Fall back to pad centroid
    cx = sum(p.global_x for p in source_pads) / len(source_pads)
    cy = sum(p.global_y for p in source_pads) / len(source_pads)
    return (cx, cy)


def compute_routing_aware_distance(
    target_free_end: Tuple[float, float],
    source_chip_center: Tuple[float, float],
    bga_zone: Tuple[float, float, float, float]
) -> float:
    """
    Compute the shortest path distance from target stub free end to source chip center,
    routing around the BGA zone (not through it).

    The algorithm:
    1. Determine which edge of the BGA the target stub is on/near
    2. Compute two candidate paths: around each corner of the BGA edge
    3. Return the shorter path distance

    Args:
        target_free_end: (x, y) position of target stub free end
        source_chip_center: (x, y) position of source chip center
        bga_zone: (min_x, min_y, max_x, max_y) BGA exclusion zone

    Returns:
        Routing-aware distance in mm
    """
    tx, ty = target_free_end
    sx, sy = source_chip_center
    min_x, min_y, max_x, max_y = bga_zone

    def point_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    # Get BGA corners
    corners = {
        'top_left': (min_x, min_y),
        'top_right': (max_x, min_y),
        'bottom_left': (min_x, max_y),
        'bottom_right': (max_x, max_y)
    }

    # Determine which edge the target stub is on/nearest to
    dist_to_left = abs(tx - min_x)
    dist_to_right = abs(tx - max_x)
    dist_to_top = abs(ty - min_y)
    dist_to_bottom = abs(ty - max_y)

    min_dist = min(dist_to_left, dist_to_right, dist_to_top, dist_to_bottom)

    # Determine candidate corners based on stub edge position
    if min_dist == dist_to_top:
        # Stub on top edge - can go around top-left or top-right corner
        corner1, corner2 = corners['top_left'], corners['top_right']
    elif min_dist == dist_to_bottom:
        # Stub on bottom edge
        corner1, corner2 = corners['bottom_left'], corners['bottom_right']
    elif min_dist == dist_to_left:
        # Stub on left edge
        corner1, corner2 = corners['top_left'], corners['bottom_left']
    else:  # dist_to_right
        # Stub on right edge
        corner1, corner2 = corners['top_right'], corners['bottom_right']

    # Path 1: target -> corner1 -> source
    dist1 = point_distance(target_free_end, corner1) + point_distance(corner1, source_chip_center)

    # Path 2: target -> corner2 -> source
    dist2 = point_distance(target_free_end, corner2) + point_distance(corner2, source_chip_center)

    return min(dist1, dist2)


def get_unit_routing_info(
    pcb_data: PCBData,
    unit_net_ids: List[int]
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    Get target stub free end and source chip center for a routing unit.

    For diff pairs, averages the P and N positions.

    Args:
        pcb_data: PCB data
        unit_net_ids: List of net IDs (1 for single net, 2 for diff pair)

    Returns:
        (target_free_end, source_chip_center) or None if not determinable
    """
    target_free_ends = []
    source_chip_centers = []

    for net_id in unit_net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        net_pads = pcb_data.pads_by_net.get(net_id, [])

        if len(net_segments) < 2:
            continue

        groups = find_connected_groups(net_segments)
        if len(groups) < 2:
            continue

        # Find which pad connects to each stub group
        def get_group_pad(group_segs):
            """Find the pad connected to this stub group."""
            group_points = set()
            for seg in group_segs:
                group_points.add(pos_key(seg.start_x, seg.start_y))
                group_points.add(pos_key(seg.end_x, seg.end_y))
            for pad in net_pads:
                pad_pos = pos_key(pad.global_x, pad.global_y)
                for gp in group_points:
                    if abs(pad_pos[0] - gp[0]) < 0.05 and abs(pad_pos[1] - gp[1]) < 0.05:
                        return pad
            return None

        # Get pad for each group
        group_pads = []
        for group in groups[:2]:  # Only first two groups
            pad = get_group_pad(group)
            if pad:
                group_pads.append((pad.component_ref, group, pad))

        if len(group_pads) < 2:
            net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
            print(f"WARNING: Could not find pads for both stub groups in {net_name} (found {len(group_pads)} pads)")
            continue

        # Sort by component_ref alphabetically for consistent source/target assignment
        # First component (alphabetically) = source, second = target
        group_pads.sort(key=lambda x: x[0])
        source_segs = group_pads[0][1]
        target_segs = group_pads[1][1]
        source_pad = group_pads[0][2]

        # Get target stub free end (or pad position if no stub free end)
        target_free = find_stub_free_ends(target_segs, net_pads)
        if target_free:
            target_free_ends.append((target_free[0][0], target_free[0][1]))
        else:
            # Use the target pad position as fallback
            target_pad = group_pads[1][2]  # target pad from earlier
            target_free_ends.append((target_pad.global_x, target_pad.global_y))

        # Get source stub free end (or pad position if no stub free end)
        source_free = find_stub_free_ends(source_segs, net_pads)
        if source_free:
            source_chip_centers.append((source_free[0][0], source_free[0][1]))
        else:
            # Use the source pad position as fallback
            source_pad = group_pads[0][2]  # source pad from earlier
            source_chip_centers.append((source_pad.global_x, source_pad.global_y))

    if not target_free_ends or not source_chip_centers:
        return None

    # Average for diff pairs
    avg_target = (
        sum(p[0] for p in target_free_ends) / len(target_free_ends),
        sum(p[1] for p in target_free_ends) / len(target_free_ends)
    )
    avg_source = (
        sum(p[0] for p in source_chip_centers) / len(source_chip_centers),
        sum(p[1] for p in source_chip_centers) / len(source_chip_centers)
    )

    return (avg_target, avg_source)


def compute_mps_net_ordering(pcb_data: PCBData, net_ids: List[int],
                              center: Tuple[float, float] = None,
                              diff_pairs: Dict = None,
                              use_boundary_ordering: bool = True,
                              bga_exclusion_zones: List[Tuple[float, float, float, float]] = None,
                              reverse_rounds: bool = False,
                              crossing_layer_check: bool = True) -> List[int]:
    """
    Compute optimal net routing order using Maximum Planar Subset (MPS) algorithm.

    The MPS approach identifies crossing conflicts between nets and orders them
    so that non-conflicting nets are routed first. This reduces routing failures
    caused by earlier routes blocking later ones.

    Algorithm:
    1. For each net, find its two stub endpoint centroids
    2. Project all endpoints onto a circular boundary centered on the routing region
    3. Assign each endpoint an angular position on the boundary
    4. Detect crossing conflicts: nets A and B cross if their endpoints alternate
       on the boundary (A1, B1, A2, B2 or B1, A1, B2, A2 ordering)
    5. Build a conflict graph where edges connect crossing nets
    6. Use greedy algorithm: repeatedly select net with fewest active conflicts,
       add to result, and remove its neighbors from consideration for this round
    7. Continue until all nets are ordered (multiple rounds/layers)

    Args:
        pcb_data: PCB data with segments
        net_ids: List of net IDs to order
        center: Optional center point for angular projection (auto-computed if None)
        diff_pairs: Optional dict of pair_name -> DiffPair. If provided, P and N nets
                    of each pair are treated as a single routing unit.
        use_boundary_ordering: If True, use chip boundary unrolling for crossing detection.
                              This respects the physical constraint that routes can't go
                              through BGA chips. Default is False (use angular projection).

    Returns:
        Ordered list of net IDs, with least-conflicting nets first
    """
    # Build mapping from net_id to unit_id (for grouping diff pair P/N nets)
    # unit_id is the canonical ID for the routing unit
    net_to_unit = {}  # net_id -> unit_id
    unit_to_nets = {}  # unit_id -> [net_ids]
    unit_names = {}  # unit_id -> display name

    if diff_pairs:
        for pair_name, pair in diff_pairs.items():
            if pair.p_net_id in net_ids or pair.n_net_id in net_ids:
                # Use P net ID as the canonical unit ID
                unit_id = pair.p_net_id
                net_to_unit[pair.p_net_id] = unit_id
                net_to_unit[pair.n_net_id] = unit_id
                unit_to_nets[unit_id] = [pair.p_net_id, pair.n_net_id]
                unit_names[unit_id] = pair_name

    # Add single nets (not part of any diff pair)
    for net_id in net_ids:
        if net_id not in net_to_unit:
            net_to_unit[net_id] = net_id
            unit_to_nets[net_id] = [net_id]
            unit_names[net_id] = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"

    # Get unique unit IDs from the net_ids list
    unit_ids = []
    seen_units = set()
    for net_id in net_ids:
        unit_id = net_to_unit.get(net_id, net_id)
        if unit_id not in seen_units:
            seen_units.add(unit_id)
            unit_ids.append(unit_id)

    # Step 1: Get routing endpoints for each unit
    # For diff pairs, average the P and N endpoints to get unit endpoints
    unit_endpoints = {}  # unit_id -> [(x1, y1), (x2, y2)]
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])

        if len(unit_net_ids) == 2:
            # Diff pair: combine P and N endpoints
            p_endpoints = get_net_routing_endpoints(pcb_data, unit_net_ids[0])
            n_endpoints = get_net_routing_endpoints(pcb_data, unit_net_ids[1])
            if len(p_endpoints) >= 2 and len(n_endpoints) >= 2:
                # Average source endpoints (P and N) and target endpoints
                src = ((p_endpoints[0][0] + n_endpoints[0][0]) / 2,
                       (p_endpoints[0][1] + n_endpoints[0][1]) / 2)
                tgt = ((p_endpoints[1][0] + n_endpoints[1][0]) / 2,
                       (p_endpoints[1][1] + n_endpoints[1][1]) / 2)
                unit_endpoints[unit_id] = [src, tgt]
        else:
            # Single net
            endpoints = get_net_routing_endpoints(pcb_data, unit_id)
            if len(endpoints) >= 2:
                unit_endpoints[unit_id] = endpoints[:2]

    if not unit_endpoints:
        print("MPS: No units with valid routing endpoints found")
        # Return all net_ids in original order
        return list(net_ids)

    # Step 2: Compute center if not provided (centroid of all endpoints)
    if center is None:
        all_points = []
        for endpoints in unit_endpoints.values():
            all_points.extend(endpoints)
        if all_points:
            center = (
                sum(p[0] for p in all_points) / len(all_points),
                sum(p[1] for p in all_points) / len(all_points)
            )
        else:
            center = (0, 0)

    # Step 3: Compute positions for crossing detection
    # If use_boundary_ordering is True, use chip boundary unrolling
    # Otherwise, use angular projection from center (default)

    unit_boundary_info = {}  # unit_id -> (src_pos, tgt_pos, src_chip, tgt_chip) or None
    unit_angles = {}  # unit_id -> (angle1, angle2)

    if use_boundary_ordering:
        # Use chip boundary ordering - respects physical constraint that routes can't go through chips
        chips = build_chip_list(pcb_data)

        for unit_id, endpoints in unit_endpoints.items():
            src_chip = identify_chip_for_point(endpoints[0], chips)
            tgt_chip = identify_chip_for_point(endpoints[1], chips)

            if src_chip and tgt_chip and src_chip != tgt_chip:
                # Normalize source/target by component reference (alphabetically)
                # This ensures consistent ordering across all units for crossing detection
                # (same normalization as used in target_swap and debug labels)
                if src_chip.reference > tgt_chip.reference:
                    # Swap endpoints so alphabetically-first chip is always source
                    endpoints = [endpoints[1], endpoints[0]]
                    src_chip, tgt_chip = tgt_chip, src_chip
                    unit_endpoints[unit_id] = endpoints

                src_far, tgt_far = compute_far_side(src_chip, tgt_chip)
                # Use OPPOSITE traversal directions: source clockwise, target counter-clockwise
                # This makes the two chips "face each other" when unrolled.
                # With opposite directions, the crossing check works correctly:
                # same geometric order → opposite boundary order → crossing detected
                src_pos = compute_boundary_position(src_chip, endpoints[0], src_far, clockwise=True)
                tgt_pos = compute_boundary_position(tgt_chip, endpoints[1], tgt_far, clockwise=False)
                unit_boundary_info[unit_id] = (src_pos, tgt_pos, src_chip, tgt_chip)

    # Compute angular positions (used as fallback or as primary method)
    def angle_from_center(point: Tuple[float, float]) -> float:
        """Compute angle from center to point in radians [0, 2*pi)."""
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        ang = math.atan2(dy, dx)
        if ang < 0:
            ang += 2 * math.pi
        return ang

    for unit_id, endpoints in unit_endpoints.items():
        # Use angular method for all units (primary or fallback)
        if not use_boundary_ordering or unit_boundary_info.get(unit_id) is None:
            a1 = angle_from_center(endpoints[0])
            a2 = angle_from_center(endpoints[1])
            if a1 > a2:
                a1, a2 = a2, a1
            unit_angles[unit_id] = (a1, a2)

    # Build layer information for each unit from stub segments
    unit_layers = {}  # unit_id -> (source_layers: set, target_layers: set)
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])
        src_layers = set()
        tgt_layers = set()

        for net_id in unit_net_ids:
            # Get segments for this net
            net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
            if net_segments:
                stub_groups = find_connected_groups(net_segments)
                if len(stub_groups) >= 2:
                    # Source stub layers (first group)
                    for seg in stub_groups[0]:
                        src_layers.add(seg.layer)
                    # Target stub layers (second group)
                    for seg in stub_groups[1]:
                        tgt_layers.add(seg.layer)

        unit_layers[unit_id] = (src_layers, tgt_layers)

    # Step 4: Detect crossing conflicts
    def units_cross(unit_a: int, unit_b: int) -> bool:
        """Check if two units have crossing paths."""
        info_a = unit_boundary_info.get(unit_a)
        info_b = unit_boundary_info.get(unit_b)

        # If both have boundary info and boundary ordering is enabled, use it
        if use_boundary_ordering and info_a is not None and info_b is not None:
            # Only compare if same chip pair
            if (info_a[2], info_a[3]) != (info_b[2], info_b[3]):
                return False  # Different chip pairs, can't directly cross

            # Check boundary order inversion
            if not crossings_from_boundary_order(info_a[0], info_a[1], info_b[0], info_b[1]):
                return False

            # Inversion detected, check if layers overlap (if enabled)
            if crossing_layer_check:
                a_src, a_tgt = unit_layers.get(unit_a, (set(), set()))
                b_src, b_tgt = unit_layers.get(unit_b, (set(), set()))
                a_all = a_src | a_tgt
                b_all = b_src | b_tgt
                if a_all and b_all and not (a_all & b_all):
                    return False  # No layer overlap, no real conflict
            return True

        # Use angular method (default or fallback)
        if unit_a in unit_angles and unit_b in unit_angles:
            a1, a2 = unit_angles[unit_a]
            b1, b2 = unit_angles[unit_b]
            angles_cross = (a1 < b1 < a2 < b2) or (b1 < a1 < b2 < a2)
            if not angles_cross:
                return False

            # Check layer overlap (if enabled)
            if crossing_layer_check:
                a_src, a_tgt = unit_layers.get(unit_a, (set(), set()))
                b_src, b_tgt = unit_layers.get(unit_b, (set(), set()))
                a_all = a_src | a_tgt
                b_all = b_src | b_tgt
                if a_all and b_all and not (a_all & b_all):
                    return False

            return True

        return False

    # Build conflict graph
    unit_list = list(unit_endpoints.keys())
    conflicts = {unit_id: set() for unit_id in unit_list}

    for i, unit_a in enumerate(unit_list):
        for unit_b in unit_list[i+1:]:
            if units_cross(unit_a, unit_b):
                conflicts[unit_a].add(unit_b)
                conflicts[unit_b].add(unit_a)

    # Count total conflicts for reporting
    total_conflicts = sum(len(c) for c in conflicts.values()) // 2
    num_diff_pairs = sum(1 for uid in unit_list if len(unit_to_nets.get(uid, [])) == 2)
    num_single = len(unit_list) - num_diff_pairs
    if num_diff_pairs > 0:
        print(f"MPS: {num_diff_pairs} diff pairs + {num_single} single nets = {len(unit_list)} units with {total_conflicts} crossing conflicts")
    else:
        print(f"MPS: {len(unit_list)} nets with {total_conflicts} crossing conflicts detected")

    # Compute route distances for each unit (routing-aware distance around BGA)
    # Used as secondary ordering: shorter routes first within same conflict count
    unit_distances = {}
    for unit_id in unit_list:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])

        # Try to get routing-aware distance
        routing_info = get_unit_routing_info(pcb_data, unit_net_ids)

        if routing_info and bga_exclusion_zones:
            target_free_end, source_chip_center = routing_info

            # Find which BGA zone the target stub is in/near
            bga_zone = find_containing_or_nearest_bga_zone(target_free_end, bga_exclusion_zones)

            if bga_zone:
                # Compute routing-aware distance around BGA
                unit_distances[unit_id] = compute_routing_aware_distance(
                    target_free_end, source_chip_center, bga_zone
                )
            else:
                # No BGA zone found - use straight line from target to source
                dx = source_chip_center[0] - target_free_end[0]
                dy = source_chip_center[1] - target_free_end[1]
                unit_distances[unit_id] = math.sqrt(dx * dx + dy * dy)
        else:
            # Fall back to straight-line distance between endpoints
            if unit_id in unit_endpoints:
                endpoints = unit_endpoints[unit_id]
                dx = endpoints[1][0] - endpoints[0][0]
                dy = endpoints[1][1] - endpoints[0][1]
                unit_distances[unit_id] = math.sqrt(dx * dx + dy * dy)
            else:
                unit_distances[unit_id] = float('inf')

    # Step 5: Greedy ordering - repeatedly pick unit with fewest active conflicts
    # Secondary: pick shorter routes first (easier routes done first, leaving space for longer ones)
    all_rounds = []  # List of (round_winners, round_num) tuples
    remaining = set(unit_list)
    round_num = 0

    while remaining:
        round_num += 1
        round_winners = []
        round_losers = set()

        # Process this round: pick units with minimal conflicts
        round_remaining = set(remaining)
        while round_remaining:
            # Find unit with minimum active conflicts among round_remaining
            # Tiebreaker: shorter route distance first
            best_unit = min(
                round_remaining,
                key=lambda uid: (len(conflicts[uid] & round_remaining), unit_distances.get(uid, 0))
            )

            # This unit wins this round
            round_winners.append(best_unit)
            round_remaining.discard(best_unit)

            # All its conflicting neighbors in round_remaining become losers
            for loser in conflicts[best_unit] & round_remaining:
                round_losers.add(loser)
                round_remaining.discard(loser)

        # Collect this round
        all_rounds.append((round_winners, round_num))
        remaining = round_losers

    # Optionally reverse round order (route smallest/most-conflicting groups first)
    if reverse_rounds:
        all_rounds = list(reversed(all_rounds))
        print("MPS: Reversing round order (routing most-conflicting groups first)")

    # Build ordered list and print round info
    # Sort each round by distance (shortest routes first within each round)
    ordered_units = []
    for round_winners, orig_round_num in all_rounds:
        sorted_winners = sorted(round_winners, key=lambda uid: unit_distances.get(uid, 0))
        ordered_units.extend(sorted_winners)
        if sorted_winners:
            winner_names = [unit_names.get(uid, f"Net {uid}") for uid in sorted_winners]
            names_str = ", ".join(winner_names)
            print(f"MPS Round {orig_round_num}: {len(sorted_winners)} units: {names_str}")

    # Expand ordered units back to net IDs
    ordered = []
    for unit_id in ordered_units:
        ordered.extend(unit_to_nets.get(unit_id, [unit_id]))

    # Add any nets that weren't in unit_angles (no valid endpoints) at the end
    # These are nets we couldn't determine routing endpoints for
    ordered_set = set(ordered)
    nets_without_endpoints = [nid for nid in net_ids if nid not in ordered_set]
    if nets_without_endpoints:
        print(f"MPS: {len(nets_without_endpoints)} nets without valid endpoints appended at end")
        ordered.extend(nets_without_endpoints)

    return ordered


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


def fix_self_intersections(segments: List[Segment], existing_segments: List[Segment] = None,
                           max_short_length: float = 1.0) -> List[Segment]:
    """Fix self-intersections by moving endpoints of short connector segments.

    When a short connector segment crosses a longer segment (either new or existing),
    we move the endpoint of the short segment to eliminate the crossing.

    Args:
        segments: New segments from routing
        existing_segments: Existing segments of the same net to check crossings against
        max_short_length: Maximum length for a segment to be considered "short"
    """
    if not segments:
        return segments

    # Combine existing segments by layer for cross-checking
    existing_by_layer = {}
    if existing_segments:
        for seg in existing_segments:
            if seg.layer not in existing_by_layer:
                existing_by_layer[seg.layer] = []
            existing_by_layer[seg.layer].append(seg)

    # Process each layer separately
    layer_segments = {}
    for seg in segments:
        if seg.layer not in layer_segments:
            layer_segments[seg.layer] = []
        layer_segments[seg.layer].append(seg)

    result_segments = []

    for layer, layer_segs in layer_segments.items():
        # Get existing segments on this layer
        existing_on_layer = existing_by_layer.get(layer, [])

        # Find all crossings involving short NEW segments
        # Map: new segment index -> (other segment, crossing point)
        segments_to_modify = {}

        for i, seg in enumerate(layer_segs):
            seg_len = math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)
            if seg_len > max_short_length:
                continue

            # This is a short new segment - check if it crosses any EXISTING segment
            for existing in existing_on_layer:
                cross_pt = _segments_cross(seg, existing)
                if cross_pt:
                    segments_to_modify[i] = (existing, cross_pt)
                    break

            # Also check crossings with LONG new segments on the same layer
            if i not in segments_to_modify:
                for j, other in enumerate(layer_segs):
                    if i == j:
                        continue
                    other_len = math.sqrt((other.end_x - other.start_x)**2 + (other.end_y - other.start_y)**2)
                    if other_len <= max_short_length:
                        continue  # Only check against longer segments
                    cross_pt = _segments_cross(seg, other)
                    if cross_pt:
                        segments_to_modify[i] = (other, cross_pt)
                        break

        # Process segments - modify short segments that cross existing ones
        for i, seg in enumerate(layer_segs):
            if i in segments_to_modify:
                existing, cross_pt = segments_to_modify[i]

                # Move the short segment's endpoint to an ENDPOINT of the existing segment
                # (not the crossing point, which would create a gap)
                # Choose the existing endpoint closest to the crossing point
                dist_cross_to_ex_start = math.sqrt((cross_pt[0] - existing.start_x)**2 +
                                                    (cross_pt[1] - existing.start_y)**2)
                dist_cross_to_ex_end = math.sqrt((cross_pt[0] - existing.end_x)**2 +
                                                  (cross_pt[1] - existing.end_y)**2)

                if dist_cross_to_ex_end < dist_cross_to_ex_start:
                    snap_x, snap_y = existing.end_x, existing.end_y
                else:
                    snap_x, snap_y = existing.start_x, existing.start_y

                # Determine which endpoint of the short seg to move (the one closer to crossing)
                dist_start_to_cross = math.sqrt((seg.start_x - cross_pt[0])**2 + (seg.start_y - cross_pt[1])**2)
                dist_end_to_cross = math.sqrt((seg.end_x - cross_pt[0])**2 + (seg.end_y - cross_pt[1])**2)

                if dist_start_to_cross < dist_end_to_cross:
                    # Move start to existing endpoint
                    new_seg = Segment(
                        start_x=snap_x, start_y=snap_y,
                        end_x=seg.end_x, end_y=seg.end_y,
                        width=seg.width, layer=seg.layer, net_id=seg.net_id
                    )
                else:
                    # Move end to existing endpoint
                    new_seg = Segment(
                        start_x=seg.start_x, start_y=seg.start_y,
                        end_x=snap_x, end_y=snap_y,
                        width=seg.width, layer=seg.layer, net_id=seg.net_id
                    )
                result_segments.append(new_seg)
            else:
                result_segments.append(seg)

    return result_segments


def collapse_appendices(segments: List[Segment], existing_segments: List[Segment] = None,
                        max_appendix_length: float = 1.0, vias: List[Via] = None,
                        debug_lines: bool = False) -> List[Segment]:
    """Collapse short appendix segments by moving dead-end vertices to junction points.

    An appendix is a short segment where one endpoint is a dead-end (degree 1) and
    the other endpoint is a junction (degree >= 2). We collapse it by moving the
    dead-end to nearly coincide with the junction (offset by 0.001mm).

    Only collapses segments where the dead-end doesn't connect to existing segments or vias.
    Also fixes self-intersections where new segments cross existing segments.

    If debug_lines is True, endpoint degrees are counted across all layers.
    """
    if not segments:
        return segments

    # First fix self-intersections with existing segments
    segments = fix_self_intersections(segments, existing_segments, max_appendix_length)

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
        all_copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
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

            # Check if endpoints connect to existing segments (with proximity tolerance) or vias
            # Use track width / 4 as proximity tolerance for segments, via size / 4 for vias
            proximity_tol = seg.width / 4
            start_connects_existing = point_near_any(seg.start_x, seg.start_y, layer_existing, proximity_tol) or point_near_any_via(seg.start_x, seg.start_y, layer_vias)
            end_connects_existing = point_near_any(seg.end_x, seg.end_y, layer_existing, proximity_tol) or point_near_any_via(seg.end_x, seg.end_y, layer_vias)

            # Appendix: one end is dead-end (degree 1, not connected to existing/vias),
            # other is junction (degree >= 2 OR connected to existing/vias)
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
        cleaned = collapse_appendices(net_segs, existing_segments, vias=net_vias, debug_lines=debug_lines)
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


def find_pad_nearest_to_position(pcb_data: PCBData, net_id: int, x: float, y: float) -> Optional[Pad]:
    """Find the pad for a given net that is nearest to the specified position."""
    pads = pcb_data.pads_by_net.get(net_id, [])
    if not pads:
        return None

    best_pad = None
    best_dist = float('inf')
    for pad in pads:
        dist = (pad.global_x - x) ** 2 + (pad.global_y - y) ** 2
        if dist < best_dist:
            best_dist = dist
            best_pad = pad

    return best_pad


def find_connected_segment_positions(pcb_data: PCBData, start_x: float, start_y: float,
                                      net_id: int, tolerance: float = 0.1) -> set:
    """
    Find all segment endpoint positions connected to a starting position for a given net.

    Uses BFS to traverse the segment chain from the starting position.
    Returns a set of (x, y) tuples for all endpoints in the connected stub chain.
    """
    # Get all segments for this net
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]

    # Build adjacency: position -> list of connected positions
    adjacency = {}
    for seg in net_segments:
        start = pos_key(seg.start_x, seg.start_y)
        end = pos_key(seg.end_x, seg.end_y)
        if start not in adjacency:
            adjacency[start] = []
        if end not in adjacency:
            adjacency[end] = []
        adjacency[start].append(end)
        adjacency[end].append(start)

    # BFS from start position
    start_key = pos_key(start_x, start_y)
    visited = set()
    queue = [start_key]

    while queue:
        pos = queue.pop(0)
        if pos in visited:
            continue
        visited.add(pos)
        for neighbor in adjacency.get(pos, []):
            if neighbor not in visited:
                queue.append(neighbor)

    return visited


def find_connected_segments(pcb_data: PCBData, start_x: float, start_y: float,
                            net_id: int) -> List:
    """
    Find all segments connected to a starting position for a given net.

    Uses BFS to traverse the segment chain from the starting position.
    Returns a list of Segment objects in the connected stub chain.

    This differs from find_connected_segment_positions in that it returns
    the actual segment objects, not just their positions. This allows
    unambiguous identification of which segments belong to which stub chain,
    even when two chains share a common endpoint position.
    """
    # Get all segments for this net
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]

    # Build adjacency: position -> list of segments touching that position
    pos_to_segments = {}
    for seg in net_segments:
        start = pos_key(seg.start_x, seg.start_y)
        end = pos_key(seg.end_x, seg.end_y)
        if start not in pos_to_segments:
            pos_to_segments[start] = []
        if end not in pos_to_segments:
            pos_to_segments[end] = []
        pos_to_segments[start].append(seg)
        pos_to_segments[end].append(seg)

    # BFS from start position, collecting segments
    start_key = pos_key(start_x, start_y)
    visited_positions = set()
    visited_segments = set()
    queue = [start_key]

    while queue:
        pos = queue.pop(0)
        if pos in visited_positions:
            continue
        visited_positions.add(pos)

        for seg in pos_to_segments.get(pos, []):
            if id(seg) in visited_segments:
                continue
            visited_segments.add(id(seg))

            # Add the other endpoint of this segment to the queue
            other_end = pos_key(seg.end_x, seg.end_y) if pos_key(seg.start_x, seg.start_y) == pos else pos_key(seg.start_x, seg.start_y)
            if other_end not in visited_positions:
                queue.append(other_end)

    # Return the actual segment objects
    return [s for s in net_segments if id(s) in visited_segments]
