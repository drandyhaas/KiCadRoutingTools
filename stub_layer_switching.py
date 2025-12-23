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

    # Find which pad the stub connects to (end opposite from stub_x, stub_y)
    pad_x, pad_y = None, None
    for seg in segments:
        # Check which end is farther from stub position
        dist_start = abs(seg.start_x - stub_x) + abs(seg.start_y - stub_y)
        dist_end = abs(seg.end_x - stub_x) + abs(seg.end_y - stub_y)
        if dist_start > dist_end:
            check_x, check_y = seg.start_x, seg.start_y
        else:
            check_x, check_y = seg.end_x, seg.end_y

        for pad in net_pads:
            if abs(pad.global_x - check_x) < tolerance and abs(pad.global_y - check_y) < tolerance:
                pad_x, pad_y = pad.global_x, pad.global_y
                break
        if pad_x is not None:
            break

    if pad_x is None:
        # Use first pad as fallback
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

    if debug:
        print(f"      apply_stub_layer_switch: net={stub.net_id}, from {stub.layer} to {new_layer}")
        print(f"        Stub at ({stub.x:.2f}, {stub.y:.2f}), pad at ({stub.pad_x:.2f}, {stub.pad_y:.2f})")
        print(f"        Modifying {len(stub.segments)} segments:")

    # Collect modifications and modify segment layers
    for seg in stub.segments:
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
                              swap_partner_name: Optional[str] = None) -> Tuple[bool, str]:
    """
    Check that swapped stubs won't overlap with other stubs on destination layer.

    Args:
        stub_p, stub_n: P and N stub info to be swapped
        dest_layer: Destination layer after swap
        all_stubs_by_layer: Dict mapping layer -> list of (pair_name, segments)
        swap_partner_name: Name of swap partner pair (excluded from overlap check)

    Returns:
        (is_valid, error_message) - True if no overlap, False with explanation otherwise
    """
    our_segments = stub_p.segments + stub_n.segments

    # Check against all stubs on destination layer
    for pair_name, other_segments in all_stubs_by_layer.get(dest_layer, []):
        # Skip swap partner (their stubs are moving away)
        if swap_partner_name and pair_name == swap_partner_name:
            continue

        if check_segments_overlap(our_segments, other_segments):
            return False, f"overlaps with {pair_name} on {dest_layer}"

    return True, ""


def point_to_segment_distance(px: float, py: float, seg: Segment) -> float:
    """Calculate the minimum distance from a point to a line segment."""
    x1, y1 = seg.start_x, seg.start_y
    x2, y2 = seg.end_x, seg.end_y

    # Vector from segment start to end
    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy

    if seg_len_sq == 0:
        # Segment is a point
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

    # Project point onto segment line, clamped to [0, 1]
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))

    # Closest point on segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


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

            dist = point_to_segment_distance(setback_x, setback_y, seg)
            if dist < check_radius:
                blocked = True
                break

        if not blocked:
            return True, ""

    return False, f"all setback angles blocked on {dest_layer}"


def validate_swap(stub_p: StubInfo, stub_n: StubInfo, dest_layer: str,
                  all_stubs_by_layer: Dict[str, List[Tuple[str, List[Segment]]]],
                  pcb_data: PCBData, config: GridRouteConfig,
                  swap_partner_name: Optional[str] = None,
                  swap_partner_net_ids: Set[int] = None) -> Tuple[bool, str]:
    """
    Validate that a stub layer swap is safe to apply.

    Checks both:
    1. No stub overlap with other pairs on destination layer
    2. Setback position is clear on destination layer

    Args:
        stub_p, stub_n: P and N stub info to validate
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
    overlap_valid, overlap_reason = validate_stub_no_overlap(
        stub_p, stub_n, dest_layer, all_stubs_by_layer, swap_partner_name
    )
    if not overlap_valid:
        return False, overlap_reason

    # Check 2: Setback is clear on dest layer
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
