"""
Length matching for PCB routes using trombone-style meanders.

Adds length to shorter routes by inserting perpendicular zigzag patterns
at the longest straight segment.
"""

import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

from kicad_parser import Segment, PCBData
from routing_config import GridRouteConfig
from routing_utils import segment_length


def point_to_segment_distance(px: float, py: float,
                               x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate minimum distance from point (px, py) to segment (x1,y1)-(x2,y2)."""
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq < 1e-10:
        # Segment is a point
        return math.sqrt((px - x1)**2 + (py - y1)**2)

    # Project point onto line, clamped to segment
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


def segments_intersect(seg1_x1: float, seg1_y1: float, seg1_x2: float, seg1_y2: float,
                       seg2_x1: float, seg2_y1: float, seg2_x2: float, seg2_y2: float) -> bool:
    """Check if two line segments intersect (cross each other)."""
    # Using cross product method
    def ccw(ax, ay, bx, by, cx, cy):
        """Check if three points are in counter-clockwise order."""
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    # Segments intersect if they straddle each other
    a1x, a1y = seg1_x1, seg1_y1
    a2x, a2y = seg1_x2, seg1_y2
    b1x, b1y = seg2_x1, seg2_y1
    b2x, b2y = seg2_x2, seg2_y2

    # Check if segment 1 straddles segment 2's line and vice versa
    return (ccw(a1x, a1y, b1x, b1y, b2x, b2y) != ccw(a2x, a2y, b1x, b1y, b2x, b2y) and
            ccw(a1x, a1y, a2x, a2y, b1x, b1y) != ccw(a1x, a1y, a2x, a2y, b2x, b2y))


def segment_to_segment_distance(seg1_x1: float, seg1_y1: float, seg1_x2: float, seg1_y2: float,
                                 seg2_x1: float, seg2_y1: float, seg2_x2: float, seg2_y2: float) -> float:
    """Calculate minimum distance between two line segments."""
    # First check if segments intersect - if so, distance is 0
    if segments_intersect(seg1_x1, seg1_y1, seg1_x2, seg1_y2,
                          seg2_x1, seg2_y1, seg2_x2, seg2_y2):
        return 0.0

    # Check distance from each endpoint to the other segment
    d1 = point_to_segment_distance(seg1_x1, seg1_y1, seg2_x1, seg2_y1, seg2_x2, seg2_y2)
    d2 = point_to_segment_distance(seg1_x2, seg1_y2, seg2_x1, seg2_y1, seg2_x2, seg2_y2)
    d3 = point_to_segment_distance(seg2_x1, seg2_y1, seg1_x1, seg1_y1, seg1_x2, seg1_y2)
    d4 = point_to_segment_distance(seg2_x2, seg2_y2, seg1_x1, seg1_y1, seg1_x2, seg1_y2)

    return min(d1, d2, d3, d4)


def get_bump_segments(
    cx: float, cy: float,
    ux: float, uy: float,
    px: float, py: float,
    direction: int,
    amplitude: float,
    chamfer: float = 0.1,
    is_first_bump: bool = True
) -> List[Tuple[float, float, float, float]]:
    """
    Calculate the segments that would form a meander bump.

    Args:
        is_first_bump: If True, includes entry chamfer. If False, bump connects
                       directly from previous bump (no entry chamfer needed).

    Returns list of (x1, y1, x2, y2) tuples for each segment.
    """
    riser_height = amplitude - 2 * chamfer
    if riser_height < 0.1:
        riser_height = 0.1

    segments = []
    x, y = cx, cy

    # Entry 45° chamfer (only for first bump)
    if is_first_bump:
        nx = x + ux * chamfer + px * chamfer * direction
        ny = y + uy * chamfer + py * chamfer * direction
        segments.append((x, y, nx, ny))
        x, y = nx, ny

    # Riser away from centerline
    nx = x + px * riser_height * direction
    ny = y + py * riser_height * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # Top chamfer 1
    nx = x + ux * chamfer + px * chamfer * direction
    ny = y + uy * chamfer + py * chamfer * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # Top chamfer 2
    nx = x + ux * chamfer - px * chamfer * direction
    ny = y + uy * chamfer - py * chamfer * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # Riser back toward centerline
    nx = x - px * riser_height * direction
    ny = y - py * riser_height * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # No exit chamfer - next bump connects directly
    # (Exit chamfer is only added at the very end of all meanders)

    return segments


def get_safe_amplitude_at_point(
    cx: float, cy: float,
    ux: float, uy: float,
    px: float, py: float,
    direction: int,
    max_amplitude: float,
    min_amplitude: float,
    layer: str,
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    is_first_bump: bool = True
) -> float:
    """
    Find the maximum safe amplitude for a meander bump at a specific point.

    Args:
        cx, cy: Center point of the bump on the trace
        ux, uy: Unit vector along the trace direction
        px, py: Perpendicular unit vector
        direction: 1 for positive perpendicular, -1 for negative
        max_amplitude: Maximum amplitude to try
        min_amplitude: Minimum useful amplitude
        layer: Layer of the trace
        pcb_data: PCB data with all segments and vias
        net_id: Net ID of the route (to exclude self)
        config: Routing configuration
        extra_segments: Additional segments to check against (e.g., from other nets in same length-match pass)
        extra_vias: Additional vias to check against (e.g., from other nets in same length-match pass)

    Returns:
        Safe amplitude, or 0 if no safe amplitude found
    """
    # Add margin to account for segment merging/optimization when writing output
    # The routing uses grid-snapped segments, but output merges them into longer
    # segments that may have slightly different coordinates (up to half a grid step)
    meander_clearance_margin = config.grid_step / 2
    required_clearance = config.track_width + config.clearance + meander_clearance_margin
    via_clearance = config.via_size / 2 + config.track_width / 2 + config.clearance + meander_clearance_margin
    chamfer = 0.1

    max_safe = max_amplitude

    # Combine pcb_data segments with extra_segments
    all_segments = list(pcb_data.segments)
    if extra_segments:
        all_segments.extend(extra_segments)

    # Combine pcb_data vias with extra_vias
    all_vias = list(pcb_data.vias)
    if extra_vias:
        all_vias.extend(extra_vias)

    # Binary search for safe amplitude
    # Start with max_amplitude and reduce if there are conflicts
    test_amplitudes = [max_amplitude]
    # Add intermediate values for binary search
    amp = max_amplitude
    while amp > min_amplitude:
        amp *= 0.7
        if amp >= min_amplitude:
            test_amplitudes.append(amp)
    test_amplitudes.append(min_amplitude)

    for test_amp in test_amplitudes:
        # Generate bump segments for this amplitude
        bump_segs = get_bump_segments(cx, cy, ux, uy, px, py, direction, test_amp, chamfer, is_first_bump)

        conflict_found = False

        # Check each bump segment against other segments on the same layer
        for bx1, by1, bx2, by2 in bump_segs:
            if conflict_found:
                break

            for other_seg in all_segments:
                if other_seg.net_id == net_id:
                    continue
                if other_seg.layer != layer:
                    continue

                # Quick distance check
                seg_center_x = (other_seg.start_x + other_seg.end_x) / 2
                seg_center_y = (other_seg.start_y + other_seg.end_y) / 2
                bump_center_x = (bx1 + bx2) / 2
                bump_center_y = (by1 + by2) / 2
                rough_dist = math.sqrt((bump_center_x - seg_center_x)**2 + (bump_center_y - seg_center_y)**2)

                if rough_dist > test_amp + 3:
                    continue

                # Check segment-to-segment distance
                dist = segment_to_segment_distance(
                    bx1, by1, bx2, by2,
                    other_seg.start_x, other_seg.start_y, other_seg.end_x, other_seg.end_y
                )

                if dist < required_clearance:
                    conflict_found = True
                    break

        # Check bump segments against vias
        if not conflict_found:
            for bx1, by1, bx2, by2 in bump_segs:
                if conflict_found:
                    break

                for via in all_vias:
                    if via.net_id == net_id:
                        continue

                    # Distance from via center to bump segment
                    dist = point_to_segment_distance(via.x, via.y, bx1, by1, bx2, by2)

                    if dist < via_clearance:
                        conflict_found = True
                        break

        if not conflict_found:
            return test_amp

    # All amplitudes had conflicts
    return 0


def check_meander_clearance(
    segment: Segment,
    amplitude: float,
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig
) -> bool:
    """
    Check if a meander at this segment would have clearance from other traces/vias.
    This is a quick check - detailed per-bump checking is done during generation.

    Args:
        segment: The segment where meander would be placed
        amplitude: Height of meander perpendicular to trace
        pcb_data: PCB data with all segments and vias
        net_id: Net ID of the route being meandered (to exclude self)
        config: Routing configuration

    Returns:
        True if meander area appears clear, False if obviously blocked
    """
    # Calculate segment direction and perpendicular
    dx = segment.end_x - segment.start_x
    dy = segment.end_y - segment.start_y
    seg_len = math.sqrt(dx * dx + dy * dy)

    if seg_len < 0.001:
        return False

    ux = dx / seg_len
    uy = dy / seg_len
    px = -uy
    py = ux

    margin = config.track_width / 2 + config.clearance * 1.5

    # Quick check: is there ANY space for meanders?
    # Sample a few points along the segment
    for t in [0.25, 0.5, 0.75]:
        cx = segment.start_x + ux * seg_len * t
        cy = segment.start_y + uy * seg_len * t

        # Check both directions
        amp_pos = get_safe_amplitude_at_point(cx, cy, ux, uy, px, py, 1, amplitude, 0.3,
                                               segment.layer, pcb_data, net_id, config)
        amp_neg = get_safe_amplitude_at_point(cx, cy, ux, uy, px, py, -1, amplitude, 0.3,
                                               segment.layer, pcb_data, net_id, config)

        if amp_pos >= 0.3 or amp_neg >= 0.3:
            return True  # At least some meanders possible

    return False


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
    track_width: float,
    pcb_data: PCBData = None,
    config: GridRouteConfig = None,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0
) -> Tuple[List[Segment], int]:
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
        amplitude: Maximum height of meander perpendicular to trace (mm)
        track_width: Track width for new segments (mm)
        pcb_data: PCB data for per-bump clearance checking (optional)
        config: Routing configuration for clearance checking (optional)
        extra_segments: Additional segments to check against (e.g., from other nets already processed)
        extra_vias: Additional vias to check against (e.g., from other nets already processed)
        min_bumps: Minimum number of bumps to generate (0 = use extra_length to determine)

    Returns:
        Tuple of (segments, bump_count) - the meander segments and number of bumps added
    """
    if extra_length <= 0 and min_bumps <= 0:
        return [segment], 0

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
    min_amplitude = 0.3  # mm - minimum useful amplitude

    # Horizontal distance consumed by one bump (just the chamfers' horizontal components)
    bump_width = 4 * chamfer

    # Estimate number of bumps we can fit
    max_bumps = int(seg_len * 0.9 / bump_width)
    if max_bumps < 1:
        return [segment], 0

    # Calculate how much extra length we need per bump on average
    target_extra_per_bump = extra_length / max_bumps

    # Each bump adds approximately 2 * (amplitude - 2*chamfer) extra length
    # So target_amplitude = target_extra_per_bump / 2 + 2*chamfer
    # But we cap at the configured amplitude

    # Build segment list
    new_segments = []

    # Current position - start at segment start
    cx = segment.start_x
    cy = segment.start_y

    # Track how much extra length we've added
    total_extra_added = 0.0

    # Generate meander bumps
    direction = 1  # Alternates: 1 = up (positive perpendicular), -1 = down
    bump_count = 0

    # Leave some margin at start and end
    margin = bump_width

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

    # Keep adding bumps until we've added enough length or run out of space
    # If min_bumps > 0: generate exactly min_bumps (for amplitude scaling - don't let extra_length add more)
    # If min_bumps == 0: use extra_length to determine bump count
    def should_continue():
        if min_bumps > 0:
            return bump_count < min_bumps
        else:
            return total_extra_added < extra_length

    while should_continue():
        # Check if we have room for another bump
        dist_to_end = math.sqrt((segment.end_x - cx)**2 + (segment.end_y - cy)**2)
        if dist_to_end < bump_width + margin:
            break

        # Determine amplitude for this bump
        bump_amplitude = amplitude

        # If we have clearance checking, find safe amplitude at this position
        is_first = (bump_count == 0)
        if pcb_data is not None and config is not None:
            safe_amp = get_safe_amplitude_at_point(
                cx, cy, ux, uy, px, py, direction, amplitude, min_amplitude,
                segment.layer, pcb_data, segment.net_id, config, extra_segments, extra_vias, is_first
            )
            if safe_amp < min_amplitude:
                # Try the other direction
                safe_amp_other = get_safe_amplitude_at_point(
                    cx, cy, ux, uy, px, py, -direction, amplitude, min_amplitude,
                    segment.layer, pcb_data, segment.net_id, config, extra_segments, extra_vias, is_first
                )
                if safe_amp_other >= min_amplitude:
                    direction = -direction
                    safe_amp = safe_amp_other
                else:
                    # No room for a bump here, skip forward with a straight segment
                    skip_dist = 0.2
                    new_x = cx + ux * skip_dist
                    new_y = cy + uy * skip_dist
                    new_segments.append(Segment(
                        start_x=cx, start_y=cy,
                        end_x=new_x, end_y=new_y,
                        width=segment.width, layer=segment.layer, net_id=segment.net_id
                    ))
                    cx, cy = new_x, new_y
                    continue

            bump_amplitude = min(amplitude, safe_amp)

        riser_height = bump_amplitude - 2 * chamfer
        if riser_height < 0.1:
            riser_height = 0.1
            bump_amplitude = riser_height + 2 * chamfer

        # Calculate extra length added by this bump
        # With entry/exit chamfers: 4 chamfers + 2 risers
        # Without inter-bump chamfers: 2 top chamfers + 2 risers (for middle bumps)
        chamfer_diag = chamfer * math.sqrt(2)

        # First bump includes entry chamfer, subsequent bumps don't
        has_entry_chamfer = (bump_count == 0)

        if has_entry_chamfer:
            # Full bump with entry chamfer
            bump_path_length = 3 * chamfer_diag + 2 * riser_height  # entry + 2 top chamfers + risers
        else:
            # No entry chamfer - connect directly from previous bump's end
            bump_path_length = 2 * chamfer_diag + 2 * riser_height  # just 2 top chamfers + risers

        # Horizontal distance consumed (for checking if we have room)
        if has_entry_chamfer:
            this_bump_width = 3 * chamfer  # entry chamfer + 2 top chamfers
        else:
            this_bump_width = 2 * chamfer  # just 2 top chamfers

        extra_this_bump = bump_path_length - this_bump_width

        # Generate this bump

        # Entry chamfer (only for first bump)
        if has_entry_chamfer:
            nx = cx + ux * chamfer + px * chamfer * direction
            ny = cy + uy * chamfer + py * chamfer * direction
            new_segments.append(Segment(
                start_x=cx, start_y=cy,
                end_x=nx, end_y=ny,
                width=segment.width, layer=segment.layer, net_id=segment.net_id
            ))
            cx, cy = nx, ny

        # Riser going away from centerline
        nx = cx + px * riser_height * direction
        ny = cy + py * riser_height * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # Top 45° chamfer 1 (continuing away from centerline + forward)
        nx = cx + ux * chamfer + px * chamfer * direction
        ny = cy + uy * chamfer + py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # Top 45° chamfer 2 (now returning toward centerline + forward)
        nx = cx + ux * chamfer - px * chamfer * direction
        ny = cy + uy * chamfer - py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # Riser going back toward centerline
        nx = cx - px * riser_height * direction
        ny = cy - py * riser_height * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # No exit chamfer - the next bump connects directly here
        # (Exit chamfer will be added after the loop for the last bump)

        # Track progress
        total_extra_added += extra_this_bump
        bump_count += 1

        # Alternate direction for next bump
        direction *= -1

    # Add exit chamfer to return to centerline
    # After any number of bumps, we're at +chamfer above centerline due to the
    # first bump's entry chamfer. Always move -chamfer perpendicular to return.
    if bump_count > 0:
        nx = cx + ux * chamfer - px * chamfer
        ny = cy + uy * chamfer - py * chamfer
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

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

    return new_segments, bump_count


def find_all_straight_runs(segments: List[Segment], min_length: float = 1.0) -> List[Tuple[int, int, float]]:
    """
    Find all straight runs of colinear segments, sorted by length descending.

    Args:
        segments: List of route segments
        min_length: Minimum run length to consider (mm)

    Returns:
        List of (start_idx, end_idx, length) tuples, sorted by length descending
    """
    if not segments:
        return []

    runs = []
    i = 0
    while i < len(segments):
        run_start = i
        run_length = segment_length(segments[i])

        j = i + 1
        while j < len(segments) and segments_are_colinear(segments[j-1], segments[j]):
            run_length += segment_length(segments[j])
            j += 1

        if run_length >= min_length:
            runs.append((run_start, j - 1, run_length))

        i = j

    # Sort by length descending
    runs.sort(key=lambda x: x[2], reverse=True)
    return runs


def apply_meanders_to_route(
    segments: List[Segment],
    extra_length: float,
    config: GridRouteConfig,
    pcb_data: PCBData = None,
    net_id: int = None,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0,
    amplitude_override: float = None
) -> Tuple[List[Segment], int]:
    """
    Apply meanders to a route to add extra length.

    Args:
        segments: Original route segments
        extra_length: Length to add (mm)
        config: Routing configuration
        pcb_data: PCB data for clearance checking (optional)
        net_id: Net ID of this route (optional, for clearance checking)
        extra_segments: Additional segments to check against (e.g., from already-processed nets)
        extra_vias: Additional vias to check against (e.g., from already-processed nets)
        min_bumps: Minimum number of bumps to generate
        amplitude_override: Override amplitude (for scaling down to hit target length)

    Returns:
        Tuple of (modified segment list, bump_count)
    """
    if extra_length <= 0 or not segments:
        return segments, 0

    # Use override amplitude if provided
    amplitude = amplitude_override if amplitude_override is not None else config.meander_amplitude

    # Find all straight runs, sorted by length
    min_length = amplitude * 2  # Reduced minimum for more options
    runs = find_all_straight_runs(segments, min_length=min_length)

    if not runs:
        print(f"    Warning: No suitable straight run found for meanders (need >= {min_length:.1f}mm)")
        return segments, 0

    # Try each straight run until we find one that works
    min_amplitude = 0.3  # Minimum useful amplitude

    for start_idx, end_idx, run_length in runs:
        # Check if this run is long enough for meanders
        if run_length < amplitude * 2:
            continue

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

        # Quick check if there's ANY space for meanders at this location
        if pcb_data is not None and net_id is not None:
            if not check_meander_clearance(merged_seg, amplitude, pcb_data, net_id, config):
                # No space at all - try next run
                continue

        # Generate meanders - per-bump clearance checking will adjust amplitudes
        meander_segs, bump_count = generate_trombone_meander(
            merged_seg,
            extra_length,
            amplitude,
            config.track_width,
            pcb_data=pcb_data,
            config=config,
            extra_segments=extra_segments,
            extra_vias=extra_vias,
            min_bumps=min_bumps
        )

        # Replace original segment run with meanders
        new_segments = segments[:start_idx] + meander_segs + segments[end_idx + 1:]
        return new_segments, bump_count

    print(f"    Warning: No straight run with clearance found for meanders")
    return segments, 0


def _old_apply_meanders_to_route(
    segments: List[Segment],
    extra_length: float,
    config: GridRouteConfig
) -> List[Segment]:
    """Old version without clearance checking - kept for reference."""
    if extra_length <= 0 or not segments:
        return segments

    min_length = config.meander_amplitude * 4
    run = find_longest_straight_run(segments, min_length=min_length)

    if run is None:
        print(f"    Warning: No suitable straight run found for meanders (need >= {min_length:.1f}mm)")
        return segments

    start_idx, end_idx, run_length = run

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
    config: GridRouteConfig,
    pcb_data: PCBData = None,
    prev_group_segments: List[Segment] = None,
    prev_group_vias: List = None
) -> Dict[str, dict]:
    """
    Apply length matching to a group of nets.

    Args:
        net_results: Dict mapping net name to routing result
        net_names: Net names in this matching group
        config: Routing configuration
        pcb_data: PCB data for clearance checking (optional)
        prev_group_segments: Segments from previously processed groups (for cross-group clearance)
        prev_group_vias: Vias from previously processed groups (for cross-group clearance)

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

    # Pre-collect ALL original segments and vias from ALL nets in the group
    # This ensures every net's meander checks against every other net's routing
    # Start with segments/vias from previously processed groups (cross-group clearance)
    already_processed_segments: List[Segment] = list(prev_group_segments) if prev_group_segments else []
    already_processed_vias: List = list(prev_group_vias) if prev_group_vias else []

    # Add all original segments/vias from ALL nets in this group BEFORE any meander processing
    for net_name, result in group_results.items():
        if result.get('new_segments'):
            already_processed_segments.extend(result['new_segments'])
        if result.get('new_vias'):
            already_processed_vias.extend(result['new_vias'])


    # Apply meanders to shorter routes
    from routing_utils import calculate_route_length

    for net_name, result in group_results.items():
        current_length = result['route_length']
        delta = target_length - current_length

        if delta <= config.length_match_tolerance:
            print(f"    {net_name}: {current_length:.2f}mm (OK, within tolerance)")
            continue

        print(f"    {net_name}: {current_length:.2f}mm -> adding {delta:.2f}mm")

        # Get net_id and stub_length from result
        net_id = None
        original_segments = result['new_segments']
        stub_length = result.get('stub_length', 0.0)  # Stub length for pad-to-pad calculation
        if original_segments:
            net_id = original_segments[0].net_id

        # Step 1: Generate initial meanders
        new_segments, bump_count = apply_meanders_to_route(
            original_segments,
            delta,
            config,
            pcb_data=pcb_data,
            net_id=net_id,
            extra_segments=already_processed_segments,
            extra_vias=already_processed_vias
        )
        new_length = calculate_route_length(new_segments) + stub_length

        # Step 2: If we undershot, add more bumps until we overshoot
        max_bump_iterations = 20
        iteration = 0
        while new_length < target_length - config.length_match_tolerance and iteration < max_bump_iterations:
            iteration += 1
            bump_count += 1
            new_segments, actual_bumps = apply_meanders_to_route(
                original_segments,
                delta,
                config,
                pcb_data=pcb_data,
                net_id=net_id,
                extra_segments=already_processed_segments,
                extra_vias=already_processed_vias,
                min_bumps=bump_count
            )
            prev_length = new_length
            new_length = calculate_route_length(new_segments) + stub_length

            # If we couldn't add more bumps (no room), stop
            if actual_bumps < bump_count or new_length <= prev_length:
                print(f"      Can't fit more bumps (got {actual_bumps}, need {bump_count})")
                break

        # Step 3: If we overshot, iteratively scale down amplitude to hit target
        scaled_amplitude = config.meander_amplitude
        for scale_iter in range(5):  # Max 5 scaling iterations
            actual_extra = new_length - current_length
            if abs(new_length - target_length) <= config.length_match_tolerance:
                break  # Within tolerance
            if actual_extra <= 0 or bump_count == 0:
                break  # Can't scale

            # Scale factor to reduce amplitude
            scale = delta / actual_extra
            scaled_amplitude = scaled_amplitude * scale

            # Clamp to reasonable minimum
            if scaled_amplitude < 0.2:
                scaled_amplitude = 0.2

            # Regenerate with scaled amplitude but same bump count
            new_segments, _ = apply_meanders_to_route(
                original_segments,
                delta,
                config,
                pcb_data=pcb_data,
                net_id=net_id,
                extra_segments=already_processed_segments,
                extra_vias=already_processed_vias,
                min_bumps=bump_count,
                amplitude_override=scaled_amplitude
            )
            new_length = calculate_route_length(new_segments) + stub_length

        if scaled_amplitude < config.meander_amplitude:
            print(f"    {net_name}: new length = {new_length:.2f}mm ({bump_count} bumps, amplitude={scaled_amplitude:.3f})")
        else:
            print(f"    {net_name}: new length = {new_length:.2f}mm ({bump_count} bumps)")

        # Update result
        result['new_segments'] = new_segments
        result['route_length'] = new_length

        # Add this net's segments and vias to the already-processed lists
        already_processed_segments.extend(new_segments)
        if result.get('new_vias'):
            already_processed_vias.extend(result['new_vias'])

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
