"""
Length matching for PCB routes using trombone-style meanders.

Adds length to shorter routes by inserting perpendicular zigzag patterns
at the longest straight segment.
"""

import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

from kicad_parser import Segment
from routing_config import GridRouteConfig
from routing_utils import segment_length


def segments_are_colinear(seg1: Segment, seg2: Segment, tolerance: float = 0.01) -> bool:
    """
    Check if two segments are colinear (same direction and connected).

    Args:
        seg1: First segment
        seg2: Second segment (should start where seg1 ends)
        tolerance: Position tolerance in mm

    Returns:
        True if segments are colinear and connected
    """
    # Check if seg2 starts where seg1 ends
    if abs(seg1.end_x - seg2.start_x) > tolerance or abs(seg1.end_y - seg2.start_y) > tolerance:
        return False

    # Check if same layer
    if seg1.layer != seg2.layer:
        return False

    # Calculate direction vectors
    dx1 = seg1.end_x - seg1.start_x
    dy1 = seg1.end_y - seg1.start_y
    dx2 = seg2.end_x - seg2.start_x
    dy2 = seg2.end_y - seg2.start_y

    # Normalize
    len1 = math.sqrt(dx1*dx1 + dy1*dy1)
    len2 = math.sqrt(dx2*dx2 + dy2*dy2)

    if len1 < 0.001 or len2 < 0.001:
        return False

    dx1, dy1 = dx1/len1, dy1/len1
    dx2, dy2 = dx2/len2, dy2/len2

    # Check if same direction (dot product close to 1)
    dot = dx1*dx2 + dy1*dy2
    return dot > 0.99


def find_longest_straight_run(segments: List[Segment], min_length: float = 1.0) -> Optional[Tuple[int, int, float]]:
    """
    Find the longest run of colinear segments suitable for meander insertion.

    Args:
        segments: List of route segments
        min_length: Minimum run length to consider (mm)

    Returns:
        (start_idx, end_idx, length) of longest run, or None if none found
    """
    if not segments:
        return None

    best_run = None
    best_length = 0.0

    i = 0
    while i < len(segments):
        # Start a new run
        run_start = i
        run_length = segment_length(segments[i])

        # Extend the run while segments are colinear
        j = i + 1
        while j < len(segments) and segments_are_colinear(segments[j-1], segments[j]):
            run_length += segment_length(segments[j])
            j += 1

        if run_length >= min_length and run_length > best_length:
            best_length = run_length
            best_run = (run_start, j - 1, run_length)

        i = j

    return best_run


def find_longest_segment(segments: List[Segment], min_length: float = 1.0) -> Optional[int]:
    """
    Find the index of the longest segment suitable for meander insertion.

    Args:
        segments: List of route segments
        min_length: Minimum segment length to consider (mm)

    Returns:
        Index of longest segment, or None if no suitable segment found
    """
    best_idx = None
    best_length = 0.0

    for i, seg in enumerate(segments):
        length = segment_length(seg)
        if length >= min_length and length > best_length:
            best_length = length
            best_idx = i

    return best_idx


def calculate_meander_params(extra_length: float, amplitude: float, spacing: float) -> Tuple[int, float]:
    """
    Calculate meander parameters to achieve target extra length.

    Args:
        extra_length: Additional length needed (mm)
        amplitude: Height of meander perpendicular to trace (mm)
        spacing: Distance between parallel meander traces (mm)

    Returns:
        (num_periods, actual_extra_length)
    """
    # Each meander period adds approximately 2 * amplitude of extra length
    # (going up and coming back down)
    length_per_period = 2 * amplitude

    # Number of complete periods needed
    num_periods = max(1, int(math.ceil(extra_length / length_per_period)))

    # Actual extra length achieved
    actual_extra = num_periods * length_per_period

    return num_periods, actual_extra


def generate_trombone_meander(
    segment: Segment,
    extra_length: float,
    amplitude: float,
    track_width: float
) -> List[Segment]:
    """
    Generate trombone-style meander segments to replace a straight segment.

    The meander creates simple up-down bumps with 45° chamfered corners:

    Original:  ────────────────────────────────────>

    Meander:   ──╮╭╮╭╮╭──>
                 ││││││
                 ╰╯╰╯╰╯

    Each "bump" consists of:
    - 45° corner going perpendicular
    - Straight segment perpendicular to trace (the "riser" going up)
    - 45° corner at top (U-turn)
    - Straight segment back down (the "riser" going down)
    - 45° corner returning to trace

    Args:
        segment: Original segment to replace
        extra_length: Additional length to add (mm)
        amplitude: Height of meander perpendicular to trace (mm)
        track_width: Track width for new segments (mm)

    Returns:
        List of segments forming the meander
    """
    if extra_length <= 0:
        return [segment]

    # Calculate segment direction
    dx = segment.end_x - segment.start_x
    dy = segment.end_y - segment.start_y
    seg_len = math.sqrt(dx * dx + dy * dy)

    if seg_len < 0.001:
        return [segment]

    # Unit vectors along and perpendicular to segment
    ux = dx / seg_len
    uy = dy / seg_len
    # Perpendicular (90° counterclockwise)
    px = -uy
    py = ux

    # 45° chamfer size - use a small chamfer for smooth corners
    chamfer = 0.1  # mm - small 45° chamfer at corners

    # Each meander bump geometry (simple U-shape):
    # - 45° chamfer going up (covers chamfer in both directions)
    # - Vertical riser up
    # - 45° chamfer at top turning back
    # - Vertical riser down
    # - 45° chamfer returning to centerline

    riser_height = amplitude - 2 * chamfer  # vertical portion (subtract chamfers at both ends)
    if riser_height < 0.1:
        riser_height = 0.1

    chamfer_diag = chamfer * math.sqrt(2)  # diagonal length of 45° chamfer

    # Each bump path length:
    # - 4 chamfers = 4 * chamfer_diag
    # - 2 risers = 2 * riser_height
    bump_path_length = 4 * chamfer_diag + 2 * riser_height

    # Horizontal distance consumed by one bump (just the chamfers' horizontal components)
    bump_width = 4 * chamfer

    # Extra length per bump = path length - horizontal distance
    extra_per_bump = bump_path_length - bump_width

    if extra_per_bump <= 0:
        return [segment]

    # Number of bumps needed
    num_bumps = max(1, int(math.ceil(extra_length / extra_per_bump)))

    # Total horizontal space needed (bumps directly adjacent, no spacing)
    total_width = num_bumps * bump_width

    # Check if segment is long enough
    if total_width > seg_len * 0.9:
        max_bumps = int(seg_len * 0.9 / bump_width)
        if max_bumps < 1:
            return [segment]
        num_bumps = max_bumps
        total_width = num_bumps * bump_width

    # Center the meanders in the segment
    margin = (seg_len - total_width) / 2

    # Build segment list
    new_segments = []

    # Current position
    cx = segment.start_x
    cy = segment.start_y

    # Straight lead-in
    if margin > 0.01:
        end_x = cx + ux * margin
        end_y = cy + uy * margin
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=end_x, end_y=end_y,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = end_x, end_y

    # Generate meander bumps
    direction = 1  # Alternates: 1 = up (positive perpendicular), -1 = down

    for bump in range(num_bumps):
        # 1. 45° chamfer going up-and-forward
        nx = cx + ux * chamfer + px * chamfer * direction
        ny = cy + uy * chamfer + py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # 2. Vertical riser going up
        nx = cx + px * riser_height * direction
        ny = cy + py * riser_height * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # 3. 45° chamfer at top (U-turn: up-and-forward to down-and-forward)
        nx = cx + ux * chamfer + px * chamfer * direction
        ny = cy + uy * chamfer + py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # 4. 45° chamfer continuing the U-turn (now going down)
        nx = cx + ux * chamfer - px * chamfer * direction
        ny = cy + uy * chamfer - py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # 5. Vertical riser going down
        nx = cx - px * riser_height * direction
        ny = cy - py * riser_height * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # 6. 45° chamfer returning to centerline
        nx = cx + ux * chamfer - px * chamfer * direction
        ny = cy + uy * chamfer - py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # No spacing between bumps - they connect directly

    # Straight lead-out to segment end
    # Calculate remaining distance to end
    remaining_x = segment.end_x - cx
    remaining_y = segment.end_y - cy
    remaining_dist = math.sqrt(remaining_x**2 + remaining_y**2)

    if remaining_dist > 0.01:
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=segment.end_x, end_y=segment.end_y,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))

    return new_segments


def apply_meanders_to_route(
    segments: List[Segment],
    extra_length: float,
    config: GridRouteConfig
) -> List[Segment]:
    """
    Apply meanders to a route to add extra length.

    Args:
        segments: Original route segments
        extra_length: Length to add (mm)
        config: Routing configuration

    Returns:
        Modified segment list with meanders
    """
    if extra_length <= 0 or not segments:
        return segments

    # Find longest straight run (colinear segments) for meander insertion
    min_length = config.meander_amplitude * 4
    run = find_longest_straight_run(segments, min_length=min_length)

    if run is None:
        print(f"    Warning: No suitable straight run found for meanders (need >= {min_length:.1f}mm)")
        return segments

    start_idx, end_idx, run_length = run

    # Create a merged segment from the run
    first_seg = segments[start_idx]
    last_seg = segments[end_idx]
    merged_seg = Segment(
        start_x=first_seg.start_x,
        start_y=first_seg.start_y,
        end_x=last_seg.end_x,
        end_y=last_seg.end_y,
        width=first_seg.width,
        layer=first_seg.layer,
        net_id=first_seg.net_id
    )

    print(f"    Inserting meanders at segments {start_idx}-{end_idx} (straight run={run_length:.2f}mm)")

    # Generate meander segments
    meander_segs = generate_trombone_meander(
        merged_seg,
        extra_length,
        config.meander_amplitude,
        config.track_width
    )

    # Replace original segment run with meanders
    new_segments = segments[:start_idx] + meander_segs + segments[end_idx + 1:]

    return new_segments


def apply_length_matching_to_group(
    net_results: Dict[str, dict],
    net_names: List[str],
    config: GridRouteConfig
) -> Dict[str, dict]:
    """
    Apply length matching to a group of nets.

    Args:
        net_results: Dict mapping net name to routing result
        net_names: Net names in this matching group
        config: Routing configuration

    Returns:
        Modified net_results with meanders added
    """
    # Find nets in this group that were successfully routed
    group_results = {}
    for name in net_names:
        if name in net_results:
            result = net_results[name]
            if result and not result.get('failed') and 'route_length' in result:
                group_results[name] = result

    if len(group_results) < 2:
        print(f"  Length matching group: fewer than 2 routed nets, skipping")
        return net_results

    # Find target length (longest route in group)
    target_length = max(r['route_length'] for r in group_results.values())

    print(f"  Length matching group: {len(group_results)} nets, target={target_length:.2f}mm")

    # Apply meanders to shorter routes
    for net_name, result in group_results.items():
        current_length = result['route_length']
        delta = target_length - current_length

        if delta <= config.length_match_tolerance:
            print(f"    {net_name}: {current_length:.2f}mm (OK, within tolerance)")
            continue

        print(f"    {net_name}: {current_length:.2f}mm -> adding {delta:.2f}mm")

        # Apply meanders
        new_segments = apply_meanders_to_route(
            result['new_segments'],
            delta,
            config
        )

        # Update result
        result['new_segments'] = new_segments
        # Recalculate length
        from routing_utils import calculate_route_length
        result['route_length'] = calculate_route_length(new_segments)
        print(f"    {net_name}: new length = {result['route_length']:.2f}mm")

    return net_results


def match_net_pattern(net_name: str, pattern: str) -> bool:
    """
    Check if net name matches a pattern.

    Patterns support:
    - * : matches any characters
    - [0-9] : matches digit range

    Args:
        net_name: Actual net name
        pattern: Pattern with wildcards

    Returns:
        True if matches
    """
    import fnmatch
    import re

    # Convert pattern to regex
    # First, handle [0-9] style ranges
    regex_pattern = pattern

    # Escape special regex chars except * and []
    for char in '.^$+?{}|()\\':
        regex_pattern = regex_pattern.replace(char, '\\' + char)

    # Convert * to .*
    regex_pattern = regex_pattern.replace('*', '.*')

    # Anchor the pattern
    regex_pattern = '^' + regex_pattern + '$'

    try:
        return bool(re.match(regex_pattern, net_name))
    except re.error:
        # Fall back to simple fnmatch
        return fnmatch.fnmatch(net_name, pattern)


def find_nets_matching_patterns(all_net_names: List[str], patterns: List[str]) -> List[str]:
    """
    Find all net names that match any of the given patterns.

    Args:
        all_net_names: All available net names
        patterns: List of patterns to match

    Returns:
        List of matching net names
    """
    matching = []
    for net_name in all_net_names:
        for pattern in patterns:
            if match_net_pattern(net_name, pattern):
                matching.append(net_name)
                break
    return matching


def auto_group_ddr4_nets(net_names: List[str]) -> List[List[str]]:
    """
    Automatically group DDR4 nets by byte lane for length matching.

    Groups:
    - DQ0-7 + DQS0/DQS0_N (byte lane 0)
    - DQ8-15 + DQS1/DQS1_N (byte lane 1)
    - etc.
    - CA/CMD/ADDR as separate group

    Args:
        net_names: List of net names to group

    Returns:
        List of groups, each group is a list of net names
    """
    import re

    groups = {}
    ungrouped = []

    for net_name in net_names:
        # Try to identify DQ nets (e.g., DQ0, DQ15, DQ0_A, etc.)
        dq_match = re.search(r'DQ(\d+)', net_name, re.IGNORECASE)
        if dq_match:
            dq_num = int(dq_match.group(1))
            byte_lane = dq_num // 8
            group_key = f"byte_lane_{byte_lane}"
            if group_key not in groups:
                groups[group_key] = []
            groups[group_key].append(net_name)
            continue

        # Try to identify DQS nets (e.g., DQS0, DQS0_N, DQSP0, DQSN0)
        dqs_match = re.search(r'DQS[PN]?(\d+)', net_name, re.IGNORECASE)
        if dqs_match:
            dqs_num = int(dqs_match.group(1))
            group_key = f"byte_lane_{dqs_num}"
            if group_key not in groups:
                groups[group_key] = []
            groups[group_key].append(net_name)
            continue

        # Try to identify CA/CMD/ADDR nets
        if re.search(r'(CA\d+|CMD|ADDR|A\d+|BA\d+|BG\d+|CK|CS|ODT|CKE|RAS|CAS|WE)', net_name, re.IGNORECASE):
            group_key = "command_address"
            if group_key not in groups:
                groups[group_key] = []
            groups[group_key].append(net_name)
            continue

        ungrouped.append(net_name)

    # Convert to list of lists
    result = list(groups.values())

    # Log grouping
    print(f"  DDR4 auto-grouping found {len(groups)} groups:")
    for key, nets in groups.items():
        print(f"    {key}: {len(nets)} nets")
    if ungrouped:
        print(f"    ungrouped: {len(ungrouped)} nets")

    return result
