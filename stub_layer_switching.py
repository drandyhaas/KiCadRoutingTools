"""
Stub layer switching optimization for differential pair routing.

Provides functions to switch stub layers to avoid vias when the
source and target stubs are on different layers.
"""

import math
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig
from routing_utils import get_stub_segments, get_stub_direction
from geometry_utils import segments_intersect_2d, point_to_segment_distance_seg
from typing import Set


@dataclass
class StubInfo:
    """Information about a stub endpoint."""
    net_id: int
    x: float
    y: float
    layer: str
    segments: List[Segment]
    pad_x: float
    pad_y: float
    has_pad_via: bool  # True if pad already has via to other layers


def get_stub_info(pcb_data: PCBData, net_id: int, stub_x: float, stub_y: float,
                  stub_layer: str, tolerance: float = 0.05) -> Optional[StubInfo]:
    """
    Gather information about a stub for layer switching analysis.

    Args:
        pcb_data: PCB data
        net_id: Net ID of the stub
        stub_x, stub_y: Position of the stub free end
        stub_layer: Current layer of the stub
        tolerance: Distance tolerance for matching

    Returns:
        StubInfo with all relevant data, or None if stub can't be analyzed
    """
    # Get segments forming the stub
    segments = get_stub_segments(pcb_data, net_id, stub_x, stub_y, stub_layer, tolerance)
    if not segments:
        return None

    # Find the pad this stub connects to
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    if not net_pads:
        return None

    # First check if stub position is directly at a pad (common for single-ended target stubs)
    pad_x, pad_y = None, None
    for pad in net_pads:
        if abs(pad.global_x - stub_x) < tolerance and abs(pad.global_y - stub_y) < tolerance:
            pad_x, pad_y = pad.global_x, pad.global_y
            break

    # If not at a pad, walk the collected stub segments to find the pad end.
    # get_stub_segments walks from free end toward pad, so follow the chain.
    if pad_x is None and segments:
        current_x, current_y = stub_x, stub_y
        for seg in segments:
            if abs(seg.start_x - current_x) < tolerance and abs(seg.start_y - current_y) < tolerance:
                current_x, current_y = seg.end_x, seg.end_y
            else:
                current_x, current_y = seg.start_x, seg.start_y
        # Find which pad is at the end position
        for pad in net_pads:
            if abs(pad.global_x - current_x) < tolerance and abs(pad.global_y - current_y) < tolerance:
                pad_x, pad_y = pad.global_x, pad.global_y
                break

    # If chain walk didn't find a pad, use BFS from chain end to find a connected pad.
    # This handles cases where get_stub_segments walked wrong direction or chain was incomplete.
    # Using BFS ensures we find a pad that's actually connected to our chain, not a different chain.
    if pad_x is None:
        layer_segments = [s for s in pcb_data.segments if s.net_id == net_id and s.layer == stub_layer]
        # BFS from chain end position to find connected pad
        visited = set()
        queue = [(current_x, current_y)]
        visited.add((round(current_x, 2), round(current_y, 2)))
        while queue and pad_x is None:
            pos_x, pos_y = queue.pop(0)
            # Check if we're at a pad
            for pad in net_pads:
                if abs(pad.global_x - pos_x) < tolerance and abs(pad.global_y - pos_y) < tolerance:
                    pad_x, pad_y = pad.global_x, pad.global_y
                    break
            if pad_x is not None:
                break
            # Find segments connected to this position and add their other endpoints
            for seg in layer_segments:
                if abs(seg.start_x - pos_x) < tolerance and abs(seg.start_y - pos_y) < tolerance:
                    key = (round(seg.end_x, 2), round(seg.end_y, 2))
                    if key not in visited:
                        visited.add(key)
                        queue.append((seg.end_x, seg.end_y))
                elif abs(seg.end_x - pos_x) < tolerance and abs(seg.end_y - pos_y) < tolerance:
                    key = (round(seg.start_x, 2), round(seg.start_y, 2))
                    if key not in visited:
                        visited.add(key)
                        queue.append((seg.start_x, seg.start_y))

    if pad_x is None:
        # For diff pairs (chain walk succeeded but didn't reach pad), use chain end position.
        # For single-ended (where chain walk may go wrong direction), try to find a connected pad.
        # If chain made progress from stub position, trust the chain end as the direction toward pad.
        stub_dist = math.sqrt((current_x - stub_x) ** 2 + (current_y - stub_y) ** 2)
        if stub_dist > tolerance:
            # Chain made progress, use chain end position (like original git code)
            pad_x, pad_y = current_x, current_y
        else:
            # Chain didn't move from stub, use first pad as fallback
            pad_x, pad_y = net_pads[0].global_x, net_pads[0].global_y

    # Check if pad already has a via (connecting to other layers)
    has_pad_via = False
    for via in pcb_data.vias:
        if via.net_id == net_id:
            dist = math.sqrt((via.x - pad_x) ** 2 + (via.y - pad_y) ** 2)
            if dist < tolerance:
                has_pad_via = True
                break

    return StubInfo(
        net_id=net_id,
        x=stub_x,
        y=stub_y,
        layer=stub_layer,
        segments=segments,
        pad_x=pad_x,
        pad_y=pad_y,
        has_pad_via=has_pad_via
    )


def needs_pad_via_for_switch(stub: StubInfo) -> bool:
    """
    Check if switching this stub requires a new pad via.

    A pad via is needed only when:
    1. Stub is currently on F.Cu (top layer)
    2. No via already exists at the pad position

    Returns:
        True if pad via needed for layer switch
    """
    if stub.layer != 'F.Cu':
        return False  # Already has via from pad to stub layer
    return not stub.has_pad_via


def apply_stub_layer_switch(pcb_data: PCBData, stub: StubInfo, new_layer: str,
                             config: GridRouteConfig, debug: bool = True) -> Tuple[List[Via], List[Dict]]:
    """
    Switch a stub to a new layer by modifying segment layers.

    Args:
        pcb_data: PCB data (segments will be modified in place)
        stub: StubInfo for the stub to switch
        new_layer: Target layer name
        config: Routing configuration
        debug: If True, print debug info about modified segments

    Returns:
        Tuple of (new_vias, segment_modifications):
        - new_vias: List of new vias created (pad vias if switching from F.Cu)
        - segment_modifications: List of dicts with segment layer changes for writing to file
    """
    new_vias = []
    segment_mods = []

    # Find ALL segments on this layer for this net that connect to either the stub position
    # or the pad position. This handles cases where get_stub_segments walked the wrong direction.
    tolerance = 0.05
    segments_to_switch = set(id(s) for s in stub.segments)

    for seg in pcb_data.segments:
        if seg.net_id != stub.net_id or seg.layer != stub.layer:
            continue
        if id(seg) in segments_to_switch:
            continue
        # Check if segment connects to pad position
        if (abs(seg.start_x - stub.pad_x) < tolerance and abs(seg.start_y - stub.pad_y) < tolerance) or \
           (abs(seg.end_x - stub.pad_x) < tolerance and abs(seg.end_y - stub.pad_y) < tolerance):
            segments_to_switch.add(id(seg))

    # Get actual segment objects
    all_segments = [s for s in stub.segments]
    for seg in pcb_data.segments:
        if id(seg) in segments_to_switch and seg not in all_segments:
            all_segments.append(seg)

    if debug:
        print(f"      apply_stub_layer_switch: net={stub.net_id}, from {stub.layer} to {new_layer}")
        print(f"        Stub at ({stub.x:.2f}, {stub.y:.2f}), pad at ({stub.pad_x:.2f}, {stub.pad_y:.2f})")
        print(f"        Modifying {len(all_segments)} segments:")

    # Collect modifications and modify segment layers
    for seg in all_segments:
        old_layer = seg.layer
        if debug:
            print(f"          ({seg.start_x:.2f},{seg.start_y:.2f})->({seg.end_x:.2f},{seg.end_y:.2f}) {old_layer} -> {new_layer}")

        # Record modification for file writing
        segment_mods.append({
            'start': (seg.start_x, seg.start_y),
            'end': (seg.end_x, seg.end_y),
            'net_id': seg.net_id,
            'old_layer': old_layer,
            'new_layer': new_layer
        })

        # Modify in memory
        seg.layer = new_layer

    # Create pad via if needed (switching from F.Cu without existing via)
    if needs_pad_via_for_switch(stub):
        via = Via(
            x=stub.pad_x,
            y=stub.pad_y,
            size=config.via_size,
            drill=config.via_drill,
            layers=['F.Cu', 'B.Cu'],  # Through-hole via
            net_id=stub.net_id
        )
        new_vias.append(via)
        pcb_data.vias.append(via)

    return new_vias, segment_mods


def revert_stub_layer_switch(pcb_data: 'PCBData', segment_mods: List[Dict], new_vias: List) -> None:
    """
    Revert a previously applied stub layer switch.

    Args:
        pcb_data: PCB data to modify
        segment_mods: List of segment modifications from apply_stub_layer_switch
        new_vias: List of vias that were added (to be removed)
    """
    # Restore segment layers from modification records
    for mod in segment_mods:
        for seg in pcb_data.segments:
            if (seg.net_id == mod['net_id'] and
                abs(seg.start_x - mod['start'][0]) < 0.001 and
                abs(seg.start_y - mod['start'][1]) < 0.001 and
                abs(seg.end_x - mod['end'][0]) < 0.001 and
                abs(seg.end_y - mod['end'][1]) < 0.001 and
                seg.layer == mod['new_layer']):
                seg.layer = mod['old_layer']
                break

    # Remove added vias
    for via in new_vias:
        if via in pcb_data.vias:
            pcb_data.vias.remove(via)


def check_segments_overlap(segments: List[Segment], other_segments: List[Segment],
                           y_tolerance: float = 0.2) -> bool:
    """
    Check if any segment bounding boxes overlap.

    Uses the same logic as route.py lines 498-518 for consistency.

    Args:
        segments: First list of segments to check
        other_segments: Second list of segments to check against
        y_tolerance: Tolerance added to Y bounds (default 0.2mm)

    Returns:
        True if any overlap found, False otherwise
    """
    for seg in segments:
        seg_y_min = min(seg.start_y, seg.end_y) - y_tolerance
        seg_y_max = max(seg.start_y, seg.end_y) + y_tolerance
        seg_x_min = min(seg.start_x, seg.end_x)
        seg_x_max = max(seg.start_x, seg.end_x)

        for other in other_segments:
            other_y_min = min(other.start_y, other.end_y)
            other_y_max = max(other.start_y, other.end_y)
            other_x_min = min(other.start_x, other.end_x)
            other_x_max = max(other.start_x, other.end_x)

            # Check Y and X overlap
            if other_y_max >= seg_y_min and other_y_min <= seg_y_max:
                if other_x_max >= seg_x_min and other_x_min <= seg_x_max:
                    return True

    return False


def validate_stub_no_overlap(stub_p: StubInfo, stub_n: StubInfo, dest_layer: str,
                              all_stubs_by_layer: Dict[str, List[Tuple[str, List[Segment]]]],
                              pcb_data: PCBData,
                              swap_partner_name: Optional[str] = None) -> Tuple[bool, str]:
    """
    Check that swapped stubs won't overlap with other stubs on destination layer.

    Args:
        stub_p, stub_n: P and N stub info to be swapped
        dest_layer: Destination layer after swap
        all_stubs_by_layer: Dict mapping layer -> list of (pair_name, segments)
        pcb_data: PCB data with all segments
        swap_partner_name: Name of swap partner pair (excluded from overlap check)

    Returns:
        (is_valid, error_message) - True if no overlap, False with explanation otherwise
    """
    our_segments = stub_p.segments + stub_n.segments
    our_net_ids = {stub_p.net_id, stub_n.net_id}

    # Check against all stubs on destination layer
    for pair_name, other_segments in all_stubs_by_layer.get(dest_layer, []):
        # Skip swap partner (their stubs are moving away)
        if swap_partner_name and pair_name == swap_partner_name:
            continue

        if check_segments_overlap(our_segments, other_segments):
            return False, f"overlaps with {pair_name} on {dest_layer}"

    # Also check against all existing segments on destination layer (catches segments not in collection)
    # Use actual 2D segment intersection test, not just bounding box overlap
    for our_seg in our_segments:
        for other in pcb_data.segments:
            if other.layer != dest_layer or other.net_id in our_net_ids:
                continue
            # Check if the 2D line segments actually intersect
            if segments_intersect_2d(
                (our_seg.start_x, our_seg.start_y), (our_seg.end_x, our_seg.end_y),
                (other.start_x, other.start_y), (other.end_x, other.end_y)
            ):
                net = pcb_data.nets.get(other.net_id)
                net_name = net.name if net else f"net {other.net_id}"
                return False, f"stub ({our_seg.start_x:.1f},{our_seg.start_y:.1f})-({our_seg.end_x:.1f},{our_seg.end_y:.1f}) overlaps {net_name} at ({other.start_x:.1f},{other.start_y:.1f})-({other.end_x:.1f},{other.end_y:.1f}) on {dest_layer}"

    return True, ""


def validate_setback_clear(stub_p: StubInfo, stub_n: StubInfo, dest_layer: str,
                           pcb_data: PCBData, config: GridRouteConfig,
                           exclude_net_ids: Set[int] = None) -> Tuple[bool, str]:
    """
    Check that at least one setback position is clear on the destination layer.

    Args:
        stub_p, stub_n: P and N stub info
        dest_layer: Destination layer to check clearance on
        pcb_data: PCB data with all segments
        config: Routing configuration
        exclude_net_ids: Net IDs to exclude from clearance check (own nets, swap partner nets)

    Returns:
        (is_valid, error_message) - True if at least one angle is clear, False otherwise
    """
    if exclude_net_ids is None:
        exclude_net_ids = set()

    # Always exclude our own nets
    exclude_net_ids = exclude_net_ids | {stub_p.net_id, stub_n.net_id}

    # Calculate center point between P and N stubs
    center_x = (stub_p.x + stub_n.x) / 2
    center_y = (stub_p.y + stub_n.y) / 2

    # Get stub direction (average of P and N directions)
    p_dir = get_stub_direction(pcb_data.segments, stub_p.x, stub_p.y, stub_p.layer)
    n_dir = get_stub_direction(pcb_data.segments, stub_n.x, stub_n.y, stub_n.layer)
    dir_x = (p_dir[0] + n_dir[0]) / 2
    dir_y = (p_dir[1] + n_dir[1]) / 2

    # Normalize direction
    dir_len = math.sqrt(dir_x * dir_x + dir_y * dir_y)
    if dir_len > 0:
        dir_x /= dir_len
        dir_y /= dir_len
    else:
        return False, "could not determine stub direction"

    # Calculate setback distance
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2
    if config.diff_pair_centerline_setback is not None:
        setback = config.diff_pair_centerline_setback
    else:
        setback = spacing_mm * 4

    # Check radius around setback position
    check_radius = config.track_width / 2 + config.clearance

    # Generate 9 angles, preferring small angles first: 0, ±max/4, ±max/2, ±3*max/4, ±max
    max_angle = config.max_setback_angle
    angles_deg = [0,
                  max_angle / 4, -max_angle / 4,
                  max_angle / 2, -max_angle / 2,
                  3 * max_angle / 4, -3 * max_angle / 4,
                  max_angle, -max_angle]

    for angle_deg in angles_deg:
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # Rotate direction by angle
        dx = dir_x * cos_a - dir_y * sin_a
        dy = dir_x * sin_a + dir_y * cos_a

        # Calculate setback position
        setback_x = center_x + dx * setback
        setback_y = center_y + dy * setback

        # Check if any segment on dest_layer blocks this position
        blocked = False
        for seg in pcb_data.segments:
            if seg.layer != dest_layer:
                continue
            if seg.net_id in exclude_net_ids:
                continue

            dist = point_to_segment_distance_seg(setback_x, setback_y, seg)
            if dist < check_radius:
                blocked = True
                break

        if not blocked:
            return True, ""

    return False, f"all setback angles blocked on {dest_layer}"


def validate_stub_endpoint_proximity(stub_p: StubInfo, stub_n: StubInfo, dest_layer: str,
                                      stub_endpoints_by_layer: Dict[str, List[Tuple[str, List[Tuple[float, float]]]]],
                                      config: GridRouteConfig,
                                      swap_partner_name: Optional[str] = None) -> Tuple[bool, str]:
    """
    Check that stub endpoints won't be too close to other stub endpoints on destination layer.

    This prevents placing stubs at positions where their endpoints would be within
    clearance distance of other stub endpoints, which would cause routing conflicts.

    Args:
        stub_p, stub_n: P and N stub info to be swapped
        dest_layer: Destination layer after swap
        stub_endpoints_by_layer: Dict mapping layer -> list of (pair_name, [(x, y), ...])
        config: Routing configuration
        swap_partner_name: Name of swap partner (excluded from check)

    Returns:
        (is_valid, error_message) - True if no proximity conflict, False otherwise
    """
    # Minimum distance between stub endpoints (track width + gap + margin)
    min_distance = config.track_width + config.clearance + config.diff_pair_gap

    our_endpoints = [(stub_p.x, stub_p.y), (stub_n.x, stub_n.y)]

    for pair_name, other_endpoints in stub_endpoints_by_layer.get(dest_layer, []):
        # Skip swap partner (their stubs are moving away)
        if swap_partner_name and pair_name == swap_partner_name:
            continue

        for our_x, our_y in our_endpoints:
            for other_x, other_y in other_endpoints:
                dist = math.sqrt((our_x - other_x) ** 2 + (our_y - other_y) ** 2)
                if dist < min_distance:
                    return False, f"stub endpoint too close to {pair_name} on {dest_layer} (dist={dist:.3f}mm < {min_distance:.3f}mm)"

    return True, ""


def validate_swap(stub_p: StubInfo, stub_n: StubInfo, dest_layer: str,
                  all_stubs_by_layer: Dict[str, List[Tuple[str, List[Segment]]]],
                  pcb_data: PCBData, config: GridRouteConfig,
                  swap_partner_name: Optional[str] = None,
                  swap_partner_net_ids: Set[int] = None,
                  stub_endpoints_by_layer: Dict[str, List[Tuple[str, List[Tuple[float, float]]]]] = None) -> Tuple[bool, str]:
    """
    Validate that a stub layer swap is safe to apply.

    Checks:
    1. No stub overlap with other pairs on destination layer
    2. Stub endpoints not too close to other stub endpoints
    3. Setback position is clear on destination layer

    Args:
        stub_p, stub_n: P and N stub info to validate
        dest_layer: Target layer for the swap
        all_stubs_by_layer: Pre-computed stub segments by layer
        pcb_data: PCB data
        config: Routing configuration
        swap_partner_name: Name of swap partner (excluded from overlap check)
        swap_partner_net_ids: Net IDs of swap partner (excluded from setback check)
        stub_endpoints_by_layer: Pre-computed stub endpoints by layer (optional)

    Returns:
        (is_valid, error_message)
    """
    # Check 1: No overlap with other stubs on dest layer
    overlap_valid, overlap_reason = validate_stub_no_overlap(
        stub_p, stub_n, dest_layer, all_stubs_by_layer, pcb_data, swap_partner_name
    )
    if not overlap_valid:
        return False, overlap_reason

    # Check 2: Stub endpoints not too close to other stub endpoints
    if stub_endpoints_by_layer:
        proximity_valid, proximity_reason = validate_stub_endpoint_proximity(
            stub_p, stub_n, dest_layer, stub_endpoints_by_layer, config, swap_partner_name
        )
        if not proximity_valid:
            return False, proximity_reason

    # Check 3: Setback is clear on dest layer
    exclude_nets = swap_partner_net_ids if swap_partner_net_ids else set()
    setback_valid, setback_reason = validate_setback_clear(
        stub_p, stub_n, dest_layer, pcb_data, config, exclude_nets
    )
    if not setback_valid:
        return False, setback_reason

    return True, ""


def collect_stubs_by_layer(pcb_data: PCBData, all_pair_layer_info: Dict,
                           config: GridRouteConfig) -> Dict[str, List[Tuple[str, List[Segment]]]]:
    """
    Pre-collect all stub segments grouped by layer for efficient overlap checking.

    Args:
        pcb_data: PCB data
        all_pair_layer_info: Dict mapping pair_name -> (src_layer, tgt_layer, sources, targets, pair)
        config: Routing configuration

    Returns:
        Dict mapping layer_name -> list of (pair_name, [p_segments + n_segments])
    """
    stubs_by_layer: Dict[str, List[Tuple[str, List[Segment]]]] = {}

    for pair_name, (src_layer, tgt_layer, sources, targets, pair) in all_pair_layer_info.items():
        # Collect source stubs
        src_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                    sources[0][5], sources[0][6], src_layer)
        src_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                    sources[0][7], sources[0][8], src_layer)
        if src_p_stub and src_n_stub:
            segments = src_p_stub.segments + src_n_stub.segments
            if src_layer not in stubs_by_layer:
                stubs_by_layer[src_layer] = []
            stubs_by_layer[src_layer].append((pair_name, segments))

        # Collect target stubs
        tgt_p_stub = get_stub_info(pcb_data, pair.p_net_id,
                                    targets[0][5], targets[0][6], tgt_layer)
        tgt_n_stub = get_stub_info(pcb_data, pair.n_net_id,
                                    targets[0][7], targets[0][8], tgt_layer)
        if tgt_p_stub and tgt_n_stub:
            segments = tgt_p_stub.segments + tgt_n_stub.segments
            if tgt_layer not in stubs_by_layer:
                stubs_by_layer[tgt_layer] = []
            stubs_by_layer[tgt_layer].append((pair_name, segments))

    return stubs_by_layer


def collect_stub_endpoints_by_layer(pcb_data: PCBData, all_pair_layer_info: Dict,
                                     config: GridRouteConfig) -> Dict[str, List[Tuple[str, List[Tuple[float, float]]]]]:
    """
    Pre-collect all stub endpoint positions grouped by layer for proximity checking.

    Args:
        pcb_data: PCB data
        all_pair_layer_info: Dict mapping pair_name -> (src_layer, tgt_layer, sources, targets, pair)
        config: Routing configuration

    Returns:
        Dict mapping layer_name -> list of (pair_name, [(p_x, p_y), (n_x, n_y)])
    """
    endpoints_by_layer: Dict[str, List[Tuple[str, List[Tuple[float, float]]]]] = {}

    for pair_name, (src_layer, tgt_layer, sources, targets, pair) in all_pair_layer_info.items():
        # Collect source stub endpoints
        src_endpoints = [(sources[0][5], sources[0][6]), (sources[0][7], sources[0][8])]
        if src_layer not in endpoints_by_layer:
            endpoints_by_layer[src_layer] = []
        endpoints_by_layer[src_layer].append((pair_name, src_endpoints))

        # Collect target stub endpoints
        tgt_endpoints = [(targets[0][5], targets[0][6]), (targets[0][7], targets[0][8])]
        if tgt_layer not in endpoints_by_layer:
            endpoints_by_layer[tgt_layer] = []
        endpoints_by_layer[tgt_layer].append((pair_name, tgt_endpoints))

    return endpoints_by_layer


# ============================================================================
# Single-ended net layer switching functions
# ============================================================================

def validate_single_stub_no_overlap(stub: StubInfo, dest_layer: str,
                                     all_stubs_by_layer: Dict[str, List[Tuple[str, List[Segment]]]],
                                     pcb_data: PCBData,
                                     swap_partner_name: Optional[str] = None,
                                     swap_partner_net_ids: Set[int] = None) -> Tuple[bool, str]:
    """
    Check that a single swapped stub won't overlap with other stubs on destination layer.

    Args:
        stub: StubInfo for the single-ended stub to be swapped
        dest_layer: Destination layer after swap
        all_stubs_by_layer: Dict mapping layer -> list of (net_name, segments)
        pcb_data: PCB data with all segments (for checking existing stubs not in swap collection)
        swap_partner_name: Name of swap partner net (excluded from overlap check)
        swap_partner_net_ids: Net IDs of swap partner (excluded from existing segment check)

    Returns:
        (is_valid, error_message) - True if no overlap, False with explanation otherwise
    """
    if swap_partner_net_ids is None:
        swap_partner_net_ids = set()

    our_segments = stub.segments

    # Check against stubs in the swap collection
    for net_name, other_segments in all_stubs_by_layer.get(dest_layer, []):
        # Skip swap partner (their stubs are moving away)
        if swap_partner_name and net_name == swap_partner_name:
            continue

        if check_segments_overlap(our_segments, other_segments):
            return False, f"overlaps with {net_name} on {dest_layer}"

    # Also check against ALL existing segments on destination layer
    # This catches overlaps with stubs that aren't being layer-switched
    for our_seg in our_segments:
        our_y_min = min(our_seg.start_y, our_seg.end_y) - 0.2
        our_y_max = max(our_seg.start_y, our_seg.end_y) + 0.2
        our_x_min = min(our_seg.start_x, our_seg.end_x)
        our_x_max = max(our_seg.start_x, our_seg.end_x)

        for other_seg in pcb_data.segments:
            if other_seg.layer != dest_layer:
                continue
            if other_seg.net_id == stub.net_id:
                continue
            if other_seg.net_id in swap_partner_net_ids:
                continue

            other_y_min = min(other_seg.start_y, other_seg.end_y)
            other_y_max = max(other_seg.start_y, other_seg.end_y)
            other_x_min = min(other_seg.start_x, other_seg.end_x)
            other_x_max = max(other_seg.start_x, other_seg.end_x)

            # Check bounding box overlap
            if other_y_max >= our_y_min and other_y_min <= our_y_max:
                if other_x_max >= our_x_min and other_x_min <= our_x_max:
                    return False, f"overlaps with existing segment (net {other_seg.net_id}) on {dest_layer}"

    return True, ""


def validate_single_setback_clear(stub: StubInfo, dest_layer: str,
                                   pcb_data: PCBData, config: GridRouteConfig,
                                   exclude_net_ids: Set[int] = None) -> Tuple[bool, str]:
    """
    Check that at least one setback position is clear for a single-ended stub.

    Uses track_width * 4 as setback distance (simpler than diff pair spacing).

    Args:
        stub: StubInfo for the single-ended stub
        dest_layer: Destination layer to check clearance on
        pcb_data: PCB data with all segments
        config: Routing configuration
        exclude_net_ids: Net IDs to exclude from clearance check

    Returns:
        (is_valid, error_message) - True if at least one angle is clear, False otherwise
    """
    if exclude_net_ids is None:
        exclude_net_ids = set()

    # Always exclude our own net
    exclude_net_ids = exclude_net_ids | {stub.net_id}

    # Get stub direction
    stub_dir = get_stub_direction(pcb_data.segments, stub.x, stub.y, stub.layer)
    dir_x, dir_y = stub_dir

    # Normalize direction
    dir_len = math.sqrt(dir_x * dir_x + dir_y * dir_y)
    if dir_len > 0:
        dir_x /= dir_len
        dir_y /= dir_len
    else:
        return False, "could not determine stub direction"

    # Calculate setback distance (simpler for single-ended: track_width * 4)
    setback = config.track_width * 4

    # Check radius around setback position
    check_radius = config.track_width / 2 + config.clearance

    # Generate 9 angles, preferring small angles first: 0, ±max/4, ±max/2, ±3*max/4, ±max
    max_angle = config.max_setback_angle
    angles_deg = [0,
                  max_angle / 4, -max_angle / 4,
                  max_angle / 2, -max_angle / 2,
                  3 * max_angle / 4, -3 * max_angle / 4,
                  max_angle, -max_angle]

    for angle_deg in angles_deg:
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # Rotate direction by angle
        dx = dir_x * cos_a - dir_y * sin_a
        dy = dir_x * sin_a + dir_y * cos_a

        # Calculate setback position
        setback_x = stub.x + dx * setback
        setback_y = stub.y + dy * setback

        # Check if any segment on dest_layer blocks this position
        blocked = False
        for seg in pcb_data.segments:
            if seg.layer != dest_layer:
                continue
            if seg.net_id in exclude_net_ids:
                continue

            dist = point_to_segment_distance_seg(setback_x, setback_y, seg)
            if dist < check_radius:
                blocked = True
                break

        if not blocked:
            return True, ""

    return False, f"all setback angles blocked on {dest_layer}"


def validate_single_swap(stub: StubInfo, dest_layer: str,
                          all_stubs_by_layer: Dict[str, List[Tuple[str, List[Segment]]]],
                          pcb_data: PCBData, config: GridRouteConfig,
                          swap_partner_name: Optional[str] = None,
                          swap_partner_net_ids: Set[int] = None) -> Tuple[bool, str]:
    """
    Validate that a single-ended stub layer swap is safe to apply.

    Checks both:
    1. No stub overlap with other stubs on destination layer
    2. Setback position is clear on destination layer

    Args:
        stub: StubInfo for the single-ended stub to validate
        dest_layer: Target layer for the swap
        all_stubs_by_layer: Pre-computed stub segments by layer
        pcb_data: PCB data
        config: Routing configuration
        swap_partner_name: Name of swap partner (excluded from overlap check)
        swap_partner_net_ids: Net IDs of swap partner (excluded from setback check)

    Returns:
        (is_valid, error_message)
    """
    # Check 1: No overlap with other stubs on dest layer
    overlap_valid, overlap_reason = validate_single_stub_no_overlap(
        stub, dest_layer, all_stubs_by_layer, pcb_data, swap_partner_name, swap_partner_net_ids
    )
    if not overlap_valid:
        return False, overlap_reason

    # Check 2: Setback is clear on dest layer
    exclude_nets = swap_partner_net_ids if swap_partner_net_ids else set()
    setback_valid, setback_reason = validate_single_setback_clear(
        stub, dest_layer, pcb_data, config, exclude_nets
    )
    if not setback_valid:
        return False, setback_reason

    return True, ""


def collect_single_ended_stubs_by_layer(pcb_data: PCBData, single_net_layer_info: Dict,
                                         config: GridRouteConfig) -> Dict[str, List[Tuple[str, List[Segment]]]]:
    """
    Pre-collect all single-ended stub segments grouped by layer for efficient overlap checking.

    Args:
        pcb_data: PCB data
        single_net_layer_info: Dict mapping net_name -> (src_layer, tgt_layer, sources, targets, net_id)
        config: Routing configuration

    Returns:
        Dict mapping layer_name -> list of (net_name, [stub_segments])
    """
    stubs_by_layer: Dict[str, List[Tuple[str, List[Segment]]]] = {}

    for net_name, (src_layer, tgt_layer, sources, targets, net_id) in single_net_layer_info.items():
        # Collect source stub
        # sources is list of (gx, gy, layer_idx, orig_x, orig_y)
        if sources:
            src_stub = get_stub_info(pcb_data, net_id, sources[0][3], sources[0][4], src_layer)
            if src_stub:
                if src_layer not in stubs_by_layer:
                    stubs_by_layer[src_layer] = []
                stubs_by_layer[src_layer].append((net_name, src_stub.segments))

        # Collect target stub
        if targets:
            tgt_stub = get_stub_info(pcb_data, net_id, targets[0][3], targets[0][4], tgt_layer)
            if tgt_stub:
                if tgt_layer not in stubs_by_layer:
                    stubs_by_layer[tgt_layer] = []
                stubs_by_layer[tgt_layer].append((net_name, tgt_stub.segments))

    return stubs_by_layer
