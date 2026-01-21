"""
Net query utilities for PCB routing.

Functions for querying pads, nets, differential pairs, and computing
MPS (Maximum Planar Subset) net ordering.
"""

import math
import fnmatch
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Set, Union

from kicad_parser import PCBData, Segment, Via, Pad
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from chip_boundary import (
    build_chip_list, identify_chip_for_point, compute_far_side,
    compute_boundary_position, crossings_from_boundary_order
)
from routing_utils import pos_key, segment_length
from connectivity import (
    find_connected_groups, find_stub_free_ends, get_net_routing_endpoints,
    get_net_mst_segments, segments_intersect
)


@dataclass
class MPSResult:
    """Extended result from MPS net ordering with conflict and layer information."""
    ordered_ids: List[int]                                    # Ordered net IDs
    conflicts: Dict[int, Set[int]]                            # unit_id -> set of conflicting unit_ids (layer-filtered)
    unit_layers: Dict[int, Tuple[Set[str], Set[str]]]         # unit_id -> (src_layers, tgt_layers)
    unit_to_nets: Dict[int, List[int]]                        # unit_id -> [net_ids]
    unit_names: Dict[int, str]                                # unit_id -> display name
    round_assignments: Dict[int, int]                         # unit_id -> round_number (1-indexed)
    num_rounds: int
    geometric_conflicts: Dict[int, Set[int]] = None           # All crossings regardless of layer (for swap checking)


def calculate_route_length(segments: List[Segment], vias: List[Via] = None, pcb_data=None) -> float:
    """
    Calculate the total length of a routed path.

    Args:
        segments: List of Segment objects making up the route
        vias: Optional list of Via objects for via barrel length calculation
        pcb_data: Optional PCBData with stackup info for via barrel lengths

    Returns:
        Total route length in mm (sum of all segment lengths + via barrel lengths)
    """
    total = 0.0
    for seg in segments:
        total += segment_length(seg)

    # Add via barrel lengths if pcb_data with stackup is provided
    if vias and pcb_data and hasattr(pcb_data, 'get_via_barrel_length'):
        for via in vias:
            if via.layers and len(via.layers) >= 2:
                barrel_len = pcb_data.get_via_barrel_length(via.layers[0], via.layers[1])
                total += barrel_len

    return total


def calculate_via_barrel_length(vias: List[Via], pcb_data) -> float:
    """
    Calculate total via barrel length for a list of vias.

    Args:
        vias: List of Via objects
        pcb_data: PCBData with stackup info

    Returns:
        Total via barrel length in mm
    """
    if not vias or not pcb_data or not hasattr(pcb_data, 'get_via_barrel_length'):
        return 0.0

    total = 0.0
    for via in vias:
        if via.layers and len(via.layers) >= 2:
            total += pcb_data.get_via_barrel_length(via.layers[0], via.layers[1])
    return total


def find_pad_at_position(pcb_data: PCBData, x: float, y: float, tolerance: float = 0.01) -> Optional[Pad]:
    """Find a pad at the given position within tolerance."""
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if abs(pad.global_x - x) < tolerance and abs(pad.global_y - y) < tolerance:
                return pad
    return None


def expand_net_patterns(pcb_data: PCBData, patterns: List[str],
                        exclude_unconnected: bool = True) -> List[str]:
    """
    Expand wildcard patterns to matching net names.

    Patterns can include * and ? wildcards (fnmatch style).
    Example: "Net-(U2A-DATA_*)" matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.

    Args:
        pcb_data: PCB data with nets and pads
        patterns: List of net name patterns (may include wildcards)
        exclude_unconnected: If True (default), exclude "unconnected-*" nets

    Returns list of unique net names in sorted order for patterns,
    preserving order of non-pattern names.
    """
    # Collect net names from both pcb.nets and pads_by_net
    all_net_names = set(net.name for net in pcb_data.nets.values() if net.name)
    # Also include net names from pads (for nets not in pcb.nets)
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.net_name:
                all_net_names.add(pad.net_name)
                break  # Only need one pad's net_name per net

    # Filter out unconnected nets (KiCad pins not connected in schematic)
    # and empty net names
    if exclude_unconnected:
        all_net_names = {name for name in all_net_names
                        if name and not name.lower().startswith('unconnected-')}

    all_net_names = list(all_net_names)
    result = []
    seen = set()

    for pattern in patterns:
        if '*' in pattern or '?' in pattern:
            # It's a wildcard pattern - find all matching nets
            matches = sorted([name for name in all_net_names if fnmatch.fnmatch(name, pattern)])
            if not matches:
                print(f"Warning: Pattern '{pattern}' matched no nets")
            else:
                print(f"Pattern '{pattern}' matched {len(matches)} nets")
            for name in matches:
                if name not in seen:
                    result.append(name)
                    seen.add(name)
        else:
            # Literal net name - allow even if it would be excluded
            # (user explicitly requested it)
            if pattern not in seen:
                result.append(pattern)
                seen.add(pattern)

    return result


def identify_power_nets(pcb_data: PCBData,
                        patterns: List[str],
                        widths: List[float]) -> Dict[int, float]:
    """
    Identify power nets and map them to track widths based on pattern matching.

    Each pattern in `patterns` corresponds to a width in `widths` at the same index.
    Nets are matched against patterns in order; the first matching pattern determines
    the width. This allows priority ordering (e.g., specific nets before wildcards).

    Args:
        pcb_data: Parsed PCB data with net information
        patterns: Glob patterns for power nets (e.g., ['*GND*', '*VCC*', '+*V'])
        widths: Track widths in mm corresponding to each pattern

    Returns:
        Dict mapping net_id to track width for matched nets

    Example:
        identify_power_nets(pcb, ['*GND*', '*VCC*'], [0.4, 0.5])
        # Returns {5: 0.4, 12: 0.4, 7: 0.5} if nets 5,12 match GND and 7 matches VCC
    """
    if len(patterns) != len(widths):
        raise ValueError(f"patterns ({len(patterns)}) and widths ({len(widths)}) must have same length")

    power_net_widths: Dict[int, float] = {}

    # Collect all net names and IDs
    for net_id, net in pcb_data.nets.items():
        if not net.name or net_id == 0:
            continue

        # Check patterns in order - first match wins
        for pattern, width in zip(patterns, widths):
            if fnmatch.fnmatch(net.name, pattern):
                power_net_widths[net_id] = width
                break

    return power_net_widths


def extract_diff_pair_base(net_name: str) -> Optional[Tuple[str, bool]]:
    """
    Extract differential pair base name and polarity from net name.

    Looks for common differential pair naming conventions:
    - name_P / name_N
    - nameP / nameN
    - name+ / name-
    - name_t / name_c (true/complement, common for DDR)
    - name_t_X / name_c_X (true/complement with suffix, e.g., DQS0_t_A)

    Returns (base_name, is_positive) or None if not a diff pair.
    """
    import re

    if not net_name:
        return None

    # Try _t_X/_c_X pattern (DDR style with channel suffix, e.g., DQS0_t_A / DQS0_c_A)
    # Match _t_ or _c_ followed by any suffix
    tc_match = re.match(r'^(.+)_([tc])_(.+)$', net_name)
    if tc_match:
        base = tc_match.group(1) + '_X_' + tc_match.group(3)  # Keep suffix in base for pairing
        is_positive = tc_match.group(2) == 't'
        return (base, is_positive)

    # Try _t/_c suffix (DDR style, e.g., CK_t / CK_c)
    if net_name.endswith('_t'):
        return (net_name[:-2], True)
    if net_name.endswith('_c'):
        return (net_name[:-2], False)

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


def find_differential_pairs(pcb_data: PCBData, patterns: List[str]) -> Dict[str, DiffPairNet]:
    """
    Find all differential pairs in the PCB matching the given glob patterns.

    Args:
        pcb_data: PCB data with net information
        patterns: Glob patterns for nets to treat as diff pairs (e.g., '*lvds*')

    Returns:
        Dict mapping base_name to DiffPair with complete P/N pairs
    """
    pairs: Dict[str, DiffPairNet] = {}

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
            pairs[base_name] = DiffPairNet(base_name=base_name)

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


def expand_pad_layers(pad_layers: List[str], routing_layers: List[str]) -> List[str]:
    """
    Expand wildcard layer specifications to actual layer names.

    KiCad uses "*.Cu" to mean all copper layers for through-hole pads.
    This function expands such wildcards to the actual routing layers.

    Args:
        pad_layers: List of layer names from the pad (may include wildcards like "*.Cu")
        routing_layers: List of actual routing layer names (e.g., ["F.Cu", "In1.Cu", "B.Cu"])

    Returns:
        List of expanded layer names (no wildcards)
    """
    expanded = []
    for layer in pad_layers:
        if layer == "*.Cu":
            # Expand to all copper routing layers
            expanded.extend(routing_layers)
        elif layer.endswith(".Cu"):
            # Regular copper layer
            expanded.append(layer)
        # Skip non-copper layers like "*.Mask", "*.Paste", etc.
    # Remove duplicates while preserving routing_layers order (deterministic)
    unique = set(expanded)
    layer_order = {layer: i for i, layer in enumerate(routing_layers)}
    return sorted(unique, key=lambda l: layer_order.get(l, len(routing_layers)))


def get_all_unrouted_net_ids(pcb_data: PCBData) -> List[int]:
    """
    Find all net IDs in the PCB that need routing.

    A net is unrouted if it has 2+ pads and they're not all connected.
    This includes:
    1. Nets with multiple disconnected segment groups (partial routing)
    2. Nets with 2+ pads but no segments (completely unrouted)
    3. Nets with 2+ pads but only 1 segment group (stub connects to 1 pad only)
    """
    unrouted_ids = set()

    # Group segments by net ID
    net_segments: Dict[int, List[Segment]] = {}
    for seg in pcb_data.segments:
        if seg.net_id not in net_segments:
            net_segments[seg.net_id] = []
        net_segments[seg.net_id].append(seg)

    # Check each net with 2+ pads
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == 0:  # Skip unassigned net
            continue
        if len(pads) < 2:  # Single-pad nets don't need routing
            continue

        segments = net_segments.get(net_id, [])
        if not segments:
            # No segments at all - completely unrouted
            unrouted_ids.add(net_id)
        else:
            # Check if segments form multiple disconnected groups
            groups = find_connected_groups(segments)
            if len(groups) >= 2:
                # Multiple disconnected stub groups = unrouted
                unrouted_ids.add(net_id)
            elif len(groups) == 1 and len(pads) > 1:
                # Only 1 segment group but multiple pads - stub connects to some but not all
                unrouted_ids.add(net_id)

    return list(unrouted_ids)


def get_chip_pad_positions(pcb_data: PCBData, net_ids: List[int], min_pads: int = 4) -> List[Tuple[float, float, str]]:
    """Get pad positions on chips for unrouted nets, to use as pseudo-stubs for proximity avoidance.

    This treats pads on "chips" (components with many pads) as stubs, discouraging
    routing near them to avoid blocking future routes to those pads.

    Args:
        pcb_data: PCB data
        net_ids: List of unrouted net IDs to get chip pads for
        min_pads: Minimum pads for a footprint to be considered a "chip" (default: 4)

    Returns:
        List of (x, y, layer) tuples for chip pad positions.
    """
    # Build set of chip footprint references
    chip_refs = set()
    for ref, footprint in pcb_data.footprints.items():
        if footprint.pads and len(footprint.pads) >= min_pads:
            chip_refs.add(ref)

    if not chip_refs:
        return []

    # Collect pad positions for unrouted nets that are on chips
    net_id_set = set(net_ids)
    chip_pads = []

    for ref in chip_refs:
        footprint = pcb_data.footprints[ref]
        for pad in footprint.pads:
            # Only include pads for nets we're tracking
            if pad.net_id in net_id_set:
                # Use first copper layer from pad's layers
                pad_layer = None
                for layer in pad.layers:
                    if layer.endswith('.Cu') and not layer.startswith('*'):
                        pad_layer = layer
                        break
                if pad_layer:
                    chip_pads.append((pad.global_x, pad.global_y, pad_layer))

    return chip_pads


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
        min_x, min_y, max_x, max_y = zone[:4]
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return zone

    # Not inside any zone - find nearest
    best_zone = None
    best_dist = float('inf')

    for zone in bga_zones:
        min_x, min_y, max_x, max_y = zone[:4]
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
    min_x, min_y, max_x, max_y = bga_zone[:4]

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


def _build_mps_unit_mappings(
    pcb_data: PCBData,
    net_ids: List[int],
    diff_pairs: Dict
) -> Tuple[Dict[int, int], Dict[int, List[int]], Dict[int, str], List[int]]:
    """
    Build mapping from net_id to unit_id for MPS ordering.

    Diff pair P/N nets are grouped into a single unit.

    Returns:
        Tuple of (net_to_unit, unit_to_nets, unit_names, unit_ids)
    """
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

    return net_to_unit, unit_to_nets, unit_names, unit_ids


def _compute_mps_unit_endpoints(
    pcb_data: PCBData,
    unit_ids: List[int],
    unit_to_nets: Dict[int, List[int]]
) -> Dict[int, List[Tuple[float, float]]]:
    """
    Compute routing endpoints for each unit.

    For diff pairs, averages P and N endpoints.

    Returns:
        Dict mapping unit_id to [source_endpoint, target_endpoint]
    """
    unit_endpoints = {}
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])

        if len(unit_net_ids) == 2:
            # Diff pair: combine P and N endpoints
            p_endpoints = get_net_routing_endpoints(pcb_data, unit_net_ids[0])
            n_endpoints = get_net_routing_endpoints(pcb_data, unit_net_ids[1])
            if len(p_endpoints) >= 2 and len(n_endpoints) >= 2:
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

    return unit_endpoints


def _compute_mps_center(
    unit_endpoints: Dict[int, List[Tuple[float, float]]],
    center: Tuple[float, float] = None
) -> Tuple[float, float]:
    """Compute center point for angular projection if not provided."""
    if center is not None:
        return center

    all_points = []
    for endpoints in unit_endpoints.values():
        all_points.extend(endpoints)
    if all_points:
        return (
            sum(p[0] for p in all_points) / len(all_points),
            sum(p[1] for p in all_points) / len(all_points)
        )
    return (0, 0)


def _compute_mps_unit_layers(
    pcb_data: PCBData,
    unit_ids: List[int],
    unit_to_nets: Dict[int, List[int]]
) -> Dict[int, Tuple[Set[str], Set[str]]]:
    """
    Build layer information for each unit from stub segments.

    Returns:
        Dict mapping unit_id to (source_layers, target_layers)
    """
    unit_layers = {}
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])
        src_layers = set()
        tgt_layers = set()

        for net_id in unit_net_ids:
            net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
            if net_segments:
                stub_groups = find_connected_groups(net_segments)
                if len(stub_groups) >= 2:
                    for seg in stub_groups[0]:
                        src_layers.add(seg.layer)
                    for seg in stub_groups[1]:
                        tgt_layers.add(seg.layer)

        # If no stub layers found, use pad layers as fallback
        if not src_layers and not tgt_layers:
            for net_id in unit_net_ids:
                net_pads = pcb_data.pads_by_net.get(net_id, [])
                for pad in net_pads:
                    for layer in pad.layers:
                        if layer.endswith('.Cu') and not layer.startswith('*'):
                            src_layers.add(layer)
                            tgt_layers.add(layer)

        unit_layers[unit_id] = (src_layers, tgt_layers)

    return unit_layers


def _compute_mps_unit_distances(
    pcb_data: PCBData,
    unit_list: List[int],
    unit_to_nets: Dict[int, List[int]],
    unit_endpoints: Dict[int, List[Tuple[float, float]]],
    bga_exclusion_zones: List[Tuple[float, float, float, float]]
) -> Dict[int, float]:
    """
    Compute routing-aware distances for each unit.

    Used as secondary ordering: shorter routes first within same conflict count.

    Returns:
        Dict mapping unit_id to distance
    """
    unit_distances = {}
    for unit_id in unit_list:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])
        routing_info = get_unit_routing_info(pcb_data, unit_net_ids)

        if routing_info and bga_exclusion_zones:
            target_free_end, source_chip_center = routing_info
            bga_zone = find_containing_or_nearest_bga_zone(target_free_end, bga_exclusion_zones)

            if bga_zone:
                unit_distances[unit_id] = compute_routing_aware_distance(
                    target_free_end, source_chip_center, bga_zone
                )
            else:
                dx = source_chip_center[0] - target_free_end[0]
                dy = source_chip_center[1] - target_free_end[1]
                unit_distances[unit_id] = math.sqrt(dx * dx + dy * dy)
        else:
            if unit_id in unit_endpoints:
                endpoints = unit_endpoints[unit_id]
                dx = endpoints[1][0] - endpoints[0][0]
                dy = endpoints[1][1] - endpoints[0][1]
                unit_distances[unit_id] = math.sqrt(dx * dx + dy * dy)
            else:
                unit_distances[unit_id] = float('inf')

    return unit_distances


def _greedy_order_mps_units(
    unit_list: List[int],
    conflicts: Dict[int, Set[int]],
    unit_distances: Dict[int, float],
    unit_names: Dict[int, str],
    reverse_rounds: bool
) -> Tuple[List[int], Dict[int, int], int]:
    """
    Order units using greedy algorithm: pick unit with fewest conflicts.

    Returns:
        Tuple of (ordered_units, round_assignments, num_rounds)
    """
    all_rounds = []
    round_assignments = {}
    remaining = set(unit_list)
    round_num = 0

    while remaining:
        round_num += 1
        round_winners = []
        round_losers = set()

        round_remaining = set(remaining)
        while round_remaining:
            best_unit = min(
                round_remaining,
                key=lambda uid: (len(conflicts[uid] & round_remaining), unit_distances.get(uid, 0), uid)
            )

            round_winners.append(best_unit)
            round_remaining.discard(best_unit)
            round_assignments[best_unit] = round_num

            for loser in conflicts[best_unit] & round_remaining:
                round_losers.add(loser)
                round_remaining.discard(loser)

        all_rounds.append((round_winners, round_num))
        remaining = round_losers

    if reverse_rounds:
        all_rounds = list(reversed(all_rounds))
        print("MPS: Reversing round order (routing most-conflicting groups first)")

    # Build ordered list, sort each round by distance
    ordered_units = []
    for round_winners, orig_round_num in all_rounds:
        sorted_winners = sorted(round_winners, key=lambda uid: unit_distances.get(uid, 0))
        ordered_units.extend(sorted_winners)
        if sorted_winners:
            winner_names = [unit_names.get(uid, f"Net {uid}") for uid in sorted_winners]
            names_str = ", ".join(winner_names)
            print(f"MPS Round {orig_round_num}: {len(sorted_winners)} units: {names_str}")

    return ordered_units, round_assignments, round_num


def compute_mps_net_ordering(pcb_data: PCBData, net_ids: List[int],
                              center: Tuple[float, float] = None,
                              diff_pairs: Dict = None,
                              use_boundary_ordering: bool = True,
                              bga_exclusion_zones: List[Tuple[float, float, float, float]] = None,
                              reverse_rounds: bool = False,
                              crossing_layer_check: bool = True,
                              return_extended_info: bool = False,
                              use_segment_intersection: bool = None) -> Union[List[int], MPSResult]:
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
        return_extended_info: If True, return MPSResult with conflict and layer info
                             instead of just ordered IDs. Used for MPS-aware layer swaps.
        use_segment_intersection: If True, use MST segment intersection for crossing detection.
                                 If None (default), auto-detect: use segment intersection when
                                 no net endpoints are on chips.

    Returns:
        If return_extended_info=False: Ordered list of net IDs, with least-conflicting nets first
        If return_extended_info=True: MPSResult with full conflict/layer/round info
    """
    # Step 1: Build unit mappings (group diff pair P/N nets)
    net_to_unit, unit_to_nets, unit_names, unit_ids = _build_mps_unit_mappings(
        pcb_data, net_ids, diff_pairs
    )

    # Step 2: Get routing endpoints for each unit
    unit_endpoints = _compute_mps_unit_endpoints(pcb_data, unit_ids, unit_to_nets)

    if not unit_endpoints:
        print("MPS: No units with valid routing endpoints found")
        return list(net_ids)

    # Step 3: Compute center if not provided
    center = _compute_mps_center(unit_endpoints, center)

    # Step 4: Compute positions for crossing detection
    # Methods: boundary ordering (BGA), segment intersection (non-BGA), or angular (fallback)
    unit_boundary_info = {}  # unit_id -> (src_pos, tgt_pos, src_chip, tgt_chip) or None
    unit_angles = {}  # unit_id -> (angle1, angle2)
    unit_mst_segments = {}  # unit_id -> list of (p1, p2) segments

    # Build MST segments for each unit (used for segment intersection method)
    for unit_id in unit_ids:
        unit_net_ids = unit_to_nets.get(unit_id, [unit_id])
        all_segments = []
        for net_id in unit_net_ids:
            all_segments.extend(get_net_mst_segments(pcb_data, net_id))
        if all_segments:
            unit_mst_segments[unit_id] = all_segments

    if use_boundary_ordering:
        # Use chip boundary ordering - respects physical constraint that routes can't go through chips
        chips = build_chip_list(pcb_data)

        for unit_id, endpoints in unit_endpoints.items():
            src_chip = identify_chip_for_point(endpoints[0], chips)
            tgt_chip = identify_chip_for_point(endpoints[1], chips)

            if src_chip and tgt_chip and src_chip != tgt_chip:
                # Normalize source/target by component reference (alphabetically)
                if src_chip.reference > tgt_chip.reference:
                    endpoints = [endpoints[1], endpoints[0]]
                    src_chip, tgt_chip = tgt_chip, src_chip
                    unit_endpoints[unit_id] = endpoints

                src_far, tgt_far = compute_far_side(src_chip, tgt_chip)
                src_pos = compute_boundary_position(src_chip, endpoints[0], src_far, clockwise=True)
                tgt_pos = compute_boundary_position(tgt_chip, endpoints[1], tgt_far, clockwise=False)
                unit_boundary_info[unit_id] = (src_pos, tgt_pos, src_chip, tgt_chip)

    # Auto-detect segment intersection mode if not specified
    if use_segment_intersection is None:
        any_in_bga = False
        if bga_exclusion_zones:
            for unit_id, endpoints in unit_endpoints.items():
                for ep in endpoints:
                    for zone in bga_exclusion_zones:
                        min_x, min_y, max_x, max_y = zone[:4]
                        if min_x <= ep[0] <= max_x and min_y <= ep[1] <= max_y:
                            any_in_bga = True
                            break
                    if any_in_bga:
                        break
                if any_in_bga:
                    break

        use_segment_intersection = not any_in_bga and len(unit_mst_segments) > 0
        if use_segment_intersection:
            print("MPS: Using segment intersection method (no nets on BGA chips)")

    # Compute angular positions (fallback method)
    def angle_from_center(point: Tuple[float, float]) -> float:
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        ang = math.atan2(dy, dx)
        if ang < 0:
            ang += 2 * math.pi
        return ang

    for unit_id, endpoints in unit_endpoints.items():
        if not use_boundary_ordering or unit_boundary_info.get(unit_id) is None:
            a1 = angle_from_center(endpoints[0])
            a2 = angle_from_center(endpoints[1])
            if a1 > a2:
                a1, a2 = a2, a1
            unit_angles[unit_id] = (a1, a2)

    # Step 5: Build layer information for each unit
    unit_layers = _compute_mps_unit_layers(pcb_data, unit_ids, unit_to_nets)

    # Step 6: Detect crossing conflicts
    def units_cross_geometric(unit_a: int, unit_b: int) -> bool:
        """Check if two units have crossing paths (geometric only, ignores layers)."""
        # Use segment intersection method if enabled (takes priority)
        if use_segment_intersection:
            segs_a = unit_mst_segments.get(unit_a, [])
            segs_b = unit_mst_segments.get(unit_b, [])
            # Check if any segment from A crosses any segment from B
            for seg_a in segs_a:
                for seg_b in segs_b:
                    if segments_intersect(seg_a[0], seg_a[1], seg_b[0], seg_b[1]):
                        return True
            return False

        # Use boundary ordering for BGA chips
        info_a = unit_boundary_info.get(unit_a)
        info_b = unit_boundary_info.get(unit_b)
        if use_boundary_ordering and info_a is not None and info_b is not None:
            # Only compare if same chip pair
            if (info_a[2], info_a[3]) != (info_b[2], info_b[3]):
                return False  # Different chip pairs, can't directly cross
            # Check boundary order inversion
            return crossings_from_boundary_order(info_a[0], info_a[1], info_b[0], info_b[1])

        # Use angular method (default or fallback)
        if unit_a in unit_angles and unit_b in unit_angles:
            a1, a2 = unit_angles[unit_a]
            b1, b2 = unit_angles[unit_b]
            return (a1 < b1 < a2 < b2) or (b1 < a1 < b2 < a2)

        return False

    def units_share_layer(unit_a: int, unit_b: int) -> bool:
        """Check if two units share at least one layer.

        If either unit has no layer info (no stubs/unrouted), assume they could
        share any layer and return True.
        """
        a_src, a_tgt = unit_layers.get(unit_a, (set(), set()))
        b_src, b_tgt = unit_layers.get(unit_b, (set(), set()))
        a_all = a_src | a_tgt
        b_all = b_src | b_tgt
        # If either has no layer info, assume potential conflict on any layer
        if not a_all or not b_all:
            return True
        return bool(a_all & b_all)

    # Build conflict graphs - both geometric (all crossings) and layer-filtered
    unit_list = list(unit_endpoints.keys())
    geometric_conflicts = {unit_id: set() for unit_id in unit_list}
    conflicts = {unit_id: set() for unit_id in unit_list}

    for i, unit_a in enumerate(unit_list):
        for unit_b in unit_list[i+1:]:
            if units_cross_geometric(unit_a, unit_b):
                # Always add to geometric conflicts
                geometric_conflicts[unit_a].add(unit_b)
                geometric_conflicts[unit_b].add(unit_a)
                # Only add to layer-filtered conflicts if they share a layer (or layer check disabled)
                if not crossing_layer_check or units_share_layer(unit_a, unit_b):
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

    # Step 7: Compute route distances for each unit
    unit_distances = _compute_mps_unit_distances(
        pcb_data, unit_list, unit_to_nets, unit_endpoints, bga_exclusion_zones
    )

    # Step 8: Greedy ordering - pick units with fewest conflicts
    ordered_units, round_assignments, round_num = _greedy_order_mps_units(
        unit_list, conflicts, unit_distances, unit_names, reverse_rounds
    )

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

    if return_extended_info:
        return MPSResult(
            ordered_ids=ordered,
            conflicts=conflicts,
            unit_layers=unit_layers,
            unit_to_nets=unit_to_nets,
            unit_names=unit_names,
            round_assignments=round_assignments,
            num_rounds=round_num,
            geometric_conflicts=geometric_conflicts
        )
    return ordered
