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

                # Quick distance check - account for segment lengths
                seg_center_x = (other_seg.start_x + other_seg.end_x) / 2
                seg_center_y = (other_seg.start_y + other_seg.end_y) / 2
                seg_half_len = math.sqrt((other_seg.end_x - other_seg.start_x)**2 +
                                         (other_seg.end_y - other_seg.start_y)**2) / 2
                bump_center_x = (bx1 + bx2) / 2
                bump_center_y = (by1 + by2) / 2
                rough_dist = math.sqrt((bump_center_x - seg_center_x)**2 + (bump_center_y - seg_center_y)**2)

                # Skip only if clearly too far (accounting for segment length and meander amplitude)
                if rough_dist > test_amp + seg_half_len + required_clearance + 1.0:
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
    first_bump_direction = None  # Track direction of first bump for exit chamfer
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
            first_bump_direction = direction  # Record direction for exit chamfer
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
    # After any number of bumps, we're at ±chamfer from centerline depending on
    # which direction the first bump went. Move opposite to first_bump_direction.
    if bump_count > 0 and first_bump_direction is not None:
        # Exit chamfer moves opposite to first bump's entry direction
        nx = cx + ux * chamfer - px * chamfer * first_bump_direction
        ny = cy + uy * chamfer - py * chamfer * first_bump_direction
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

    Handles both single-ended nets and differential pairs. For diff pairs,
    meanders are applied to the centerline and P/N paths are regenerated.

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
    processed_result_ids = set()  # Track by id() to avoid processing same diff pair twice

    for name in net_names:
        if name in net_results:
            result = net_results[name]
            if result and not result.get('failed') and 'route_length' in result:
                result_id = id(result)
                if result_id not in processed_result_ids:
                    group_results[name] = result
                    processed_result_ids.add(result_id)

    if len(group_results) < 2:
        print(f"  Length matching group: fewer than 2 routed nets, skipping")
        return net_results

    # Identify diff pairs vs single-ended nets
    diff_pair_results = {}  # net_name -> result for diff pairs
    single_ended_results = {}  # net_name -> result for single-ended

    for net_name, result in group_results.items():
        if result.get('is_diff_pair'):
            diff_pair_results[net_name] = result
        else:
            single_ended_results[net_name] = result

    # Find target length (longest route in group - use centerline_length for diff pairs)
    all_lengths = []
    for result in group_results.values():
        all_lengths.append(result['route_length'])
    target_length = max(all_lengths)

    diff_pair_count = len(diff_pair_results)
    single_count = len(single_ended_results)
    print(f"  Length matching group: {len(group_results)} nets ({diff_pair_count} diff pairs, {single_count} single-ended), target={target_length:.2f}mm")

    # Pre-collect ALL original segments and vias from ALL nets in the group
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
            if result.get('is_diff_pair'):
                print(f"    {net_name} (diff pair): {current_length:.2f}mm (OK, within tolerance)")
            else:
                print(f"    {net_name}: {current_length:.2f}mm (OK, within tolerance)")
            continue

        is_diff_pair = result.get('is_diff_pair', False)

        if is_diff_pair:
            # Differential pair: apply meanders to centerline and regenerate P/N
            print(f"    {net_name} (diff pair): {current_length:.2f}mm -> adding {delta:.2f}mm")

            stub_length = result.get('stub_length', 0.0)

            # Step 1: Generate initial meanders on centerline
            modified_result, bump_count = apply_meanders_to_diff_pair(
                result, delta, config, pcb_data,
                extra_segments=already_processed_segments,
                extra_vias=already_processed_vias
            )
            new_length = modified_result['route_length']

            # Step 2: If we undershot, add more bumps
            max_bump_iterations = 20
            iteration = 0
            while new_length < target_length - config.length_match_tolerance and iteration < max_bump_iterations:
                iteration += 1
                bump_count += 1
                modified_result, actual_bumps = apply_meanders_to_diff_pair(
                    result, delta, config, pcb_data,
                    extra_segments=already_processed_segments,
                    extra_vias=already_processed_vias,
                    min_bumps=bump_count
                )
                prev_length = new_length
                new_length = modified_result['route_length']

                if actual_bumps < bump_count or new_length <= prev_length:
                    print(f"      Can't fit more bumps (got {actual_bumps}, need {bump_count})")
                    break

            # Step 3: If we overshot, scale down amplitude
            scaled_amplitude = config.meander_amplitude
            for scale_iter in range(5):
                actual_extra = new_length - current_length
                if abs(new_length - target_length) <= config.length_match_tolerance:
                    break
                if actual_extra <= 0 or bump_count == 0:
                    break

                scale = delta / actual_extra
                scaled_amplitude = scaled_amplitude * scale
                if scaled_amplitude < 0.2:
                    scaled_amplitude = 0.2

                modified_result, _ = apply_meanders_to_diff_pair(
                    result, delta, config, pcb_data,
                    extra_segments=already_processed_segments,
                    extra_vias=already_processed_vias,
                    min_bumps=bump_count,
                    amplitude_override=scaled_amplitude
                )
                new_length = modified_result['route_length']

            if scaled_amplitude < config.meander_amplitude:
                print(f"    {net_name}: new length = {new_length:.2f}mm ({bump_count} bumps, amplitude={scaled_amplitude:.3f})")
            else:
                print(f"    {net_name}: new length = {new_length:.2f}mm ({bump_count} bumps)")

            # Update the original result in-place (affects all references to it)
            result.update(modified_result)

            # Add this diff pair's segments and vias to already-processed lists
            already_processed_segments.extend(result['new_segments'])
            if result.get('new_vias'):
                already_processed_vias.extend(result['new_vias'])

        else:
            # Single-ended net: use existing meander logic
            print(f"    {net_name}: {current_length:.2f}mm -> adding {delta:.2f}mm")

            net_id = None
            original_segments = result['new_segments']
            stub_length = result.get('stub_length', 0.0)
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

                if actual_bumps < bump_count or new_length <= prev_length:
                    print(f"      Can't fit more bumps (got {actual_bumps}, need {bump_count})")
                    break

            # Step 3: If we overshot, iteratively scale down amplitude to hit target
            scaled_amplitude = config.meander_amplitude
            for scale_iter in range(5):
                actual_extra = new_length - current_length
                if abs(new_length - target_length) <= config.length_match_tolerance:
                    break
                if actual_extra <= 0 or bump_count == 0:
                    break

                scale = delta / actual_extra
                scaled_amplitude = scaled_amplitude * scale
                if scaled_amplitude < 0.2:
                    scaled_amplitude = 0.2

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
    # Process backslash first to avoid re-escaping
    regex_pattern = regex_pattern.replace('\\', '\\\\')
    for char in '.^$+?{}|()':
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


# ============================================================================
# Differential Pair Length Matching
# ============================================================================

def find_straight_runs_in_path(path: List[Tuple[int, int, int]],
                                 coord,
                                 min_length: float = 1.0) -> List[Tuple[int, int, float]]:
    """
    Find all straight runs in a centerline path (grid coordinates).

    Args:
        path: List of (gx, gy, layer) grid coordinates
        coord: GridCoord converter for grid<->float
        min_length: Minimum run length to consider (mm)

    Returns:
        List of (start_idx, end_idx, length) tuples, sorted by length descending
    """
    if len(path) < 2:
        return []

    runs = []
    i = 0

    while i < len(path) - 1:
        run_start = i
        run_length = 0.0

        # Get initial direction
        x1, y1 = coord.to_float(path[i][0], path[i][1])
        x2, y2 = coord.to_float(path[i + 1][0], path[i + 1][1])
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.sqrt(dx * dx + dy * dy)

        if seg_len < 0.001:
            i += 1
            continue

        run_length += seg_len
        ux, uy = dx / seg_len, dy / seg_len

        # Extend run while segments are colinear and on same layer
        j = i + 1
        while j < len(path) - 1:
            # Check same layer
            if path[j][2] != path[run_start][2]:
                break

            # Get next segment direction
            x1, y1 = coord.to_float(path[j][0], path[j][1])
            x2, y2 = coord.to_float(path[j + 1][0], path[j + 1][1])
            dx = x2 - x1
            dy = y2 - y1
            next_len = math.sqrt(dx * dx + dy * dy)

            if next_len < 0.001:
                j += 1
                continue

            next_ux, next_uy = dx / next_len, dy / next_len

            # Check if colinear (dot product close to 1)
            dot = ux * next_ux + uy * next_uy
            if dot < 0.99:
                break

            run_length += next_len
            j += 1

        if run_length >= min_length:
            runs.append((run_start, j, run_length))

        i = j

    # Sort by length descending
    runs.sort(key=lambda x: x[2], reverse=True)
    return runs


def generate_centerline_meander(
    path: List[Tuple[int, int, int]],
    start_idx: int,
    end_idx: int,
    extra_length: float,
    amplitude: float,
    coord,
    config: GridRouteConfig,
    spacing_mm: float,
    pcb_data: PCBData = None,
    p_net_id: int = None,
    n_net_id: int = None,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0
) -> Tuple[List[Tuple[float, float, int]], int]:
    """
    Generate meanders in a centerline path (returns float coordinates).

    Key differences from generate_trombone_meander:
    - Works with (gx, gy, layer) path format
    - Clearance checking accounts for full diff pair width (2 * spacing_mm)
    - Returns modified centerline path in float coordinates

    Args:
        path: Original centerline path in grid coords
        start_idx, end_idx: Indices of the straight run to meander
        extra_length: Target extra length to add (mm)
        amplitude: Maximum amplitude for meanders
        coord: GridCoord converter
        config: Routing configuration
        spacing_mm: P/N offset from centerline (for clearance calculation)
        pcb_data: PCB data for clearance checking
        p_net_id, n_net_id: Net IDs to exclude from clearance check
        extra_segments: Additional segments to check against
        extra_vias: Additional vias to check against
        min_bumps: Minimum number of bumps to generate

    Returns:
        (new_path_float, bump_count) - new centerline path in float coords with meanders
    """
    if extra_length <= 0 and min_bumps <= 0:
        # Just convert to float and return
        return [(coord.to_float(p[0], p[1])[0], coord.to_float(p[0], p[1])[1], p[2])
                for p in path], 0

    # Convert path to float coordinates
    float_path = [(coord.to_float(p[0], p[1])[0], coord.to_float(p[0], p[1])[1], p[2])
                  for p in path]

    # Get start and end points of the straight run
    start_pt = float_path[start_idx]
    end_pt = float_path[end_idx]
    layer = start_pt[2]

    # Calculate direction
    dx = end_pt[0] - start_pt[0]
    dy = end_pt[1] - start_pt[1]
    seg_len = math.sqrt(dx * dx + dy * dy)

    if seg_len < 0.001:
        return float_path, 0

    ux = dx / seg_len
    uy = dy / seg_len
    px = -uy  # Perpendicular
    py = ux

    # Meander parameters - chamfer must be larger than P/N half-spacing to avoid crossings
    chamfer = max(0.1, spacing_mm * 1.2)  # At least 20% more than P/N offset
    min_amplitude = 0.3
    bump_width = 6 * chamfer  # wide entry (2) + wide top chamfers (4)

    # Account for full diff pair width in clearance
    # The meander bump needs clearance for both P and N tracks
    diff_pair_half_width = spacing_mm + config.track_width / 2

    # Build new path with meanders
    new_path = list(float_path[:start_idx])

    # Current position
    cx, cy = start_pt[0], start_pt[1]

    # Meander generation
    direction = 1
    first_bump_direction = None
    bump_count = 0
    total_extra_added = 0.0

    # Leave margin at start and end
    margin = bump_width

    # Add lead-in
    if margin > 0.01:
        cx += ux * margin
        cy += uy * margin
        new_path.append((cx, cy, layer))

    def should_continue():
        if min_bumps > 0:
            return bump_count < min_bumps
        else:
            return total_extra_added < extra_length

    max_skips = 100  # Prevent infinite loop
    skip_count = 0

    while should_continue():
        # Check room for another bump
        dist_to_end = math.sqrt((end_pt[0] - cx)**2 + (end_pt[1] - cy)**2)
        if dist_to_end < bump_width + margin:
            break

        # Find safe amplitude at this position
        bump_amplitude = amplitude
        is_first = (bump_count == 0)

        if pcb_data is not None:
            # Adjust clearance for diff pair width
            safe_amp = get_safe_amplitude_for_diff_pair(
                cx, cy, ux, uy, px, py, direction, amplitude, min_amplitude,
                layer, pcb_data, p_net_id, n_net_id, config, spacing_mm,
                extra_segments, extra_vias, is_first
            )
            if safe_amp < min_amplitude:
                # Try other direction
                safe_amp_other = get_safe_amplitude_for_diff_pair(
                    cx, cy, ux, uy, px, py, -direction, amplitude, min_amplitude,
                    layer, pcb_data, p_net_id, n_net_id, config, spacing_mm,
                    extra_segments, extra_vias, is_first
                )
                if safe_amp_other >= min_amplitude:
                    direction = -direction
                    safe_amp = safe_amp_other
                else:
                    # Skip forward
                    skip_count += 1
                    if skip_count > max_skips:
                        break
                    cx += ux * 0.2
                    cy += uy * 0.2
                    new_path.append((cx, cy, layer))
                    continue

            bump_amplitude = min(amplitude, safe_amp)

        riser_height = bump_amplitude - 2 * chamfer
        if riser_height < 0.1:
            riser_height = 0.1
            bump_amplitude = riser_height + 2 * chamfer

        # Calculate extra length for this bump
        # All chamfers are wider (2:1 ratio) for P/N track spacing
        chamfer_diag_wide = chamfer * math.sqrt(5)  # wider chamfer (2:1)
        has_entry_chamfer = (bump_count == 0)

        if has_entry_chamfer:
            # wide entry chamfer + 2 wide top chamfers + risers
            bump_path_length = 3 * chamfer_diag_wide + 2 * riser_height
            this_bump_width = 2 * chamfer + 4 * chamfer  # entry (2) + top chamfers (4)
        else:
            # 2 wide top chamfers + risers
            bump_path_length = 2 * chamfer_diag_wide + 2 * riser_height
            this_bump_width = 4 * chamfer  # top chamfers only

        extra_this_bump = bump_path_length - this_bump_width

        # Generate bump points
        if has_entry_chamfer:
            first_bump_direction = direction
            # Entry chamfer - wider (2x forward) to give P/N tracks room
            cx += ux * 2 * chamfer + px * chamfer * direction
            cy += uy * 2 * chamfer + py * chamfer * direction
            new_path.append((cx, cy, layer))

        # Riser up
        cx += px * riser_height * direction
        cy += py * riser_height * direction
        new_path.append((cx, cy, layer))

        # Top chamfer 1 - wider (2x forward) for P/N track spacing
        cx += ux * 2 * chamfer + px * chamfer * direction
        cy += uy * 2 * chamfer + py * chamfer * direction
        new_path.append((cx, cy, layer))

        # Top chamfer 2 - wider (2x forward) for P/N track spacing
        cx += ux * 2 * chamfer - px * chamfer * direction
        cy += uy * 2 * chamfer - py * chamfer * direction
        new_path.append((cx, cy, layer))

        # Riser down
        cx -= px * riser_height * direction
        cy -= py * riser_height * direction
        new_path.append((cx, cy, layer))

        total_extra_added += extra_this_bump
        bump_count += 1
        direction *= -1

    # Exit chamfer - wider (2x forward) to match entry
    if bump_count > 0 and first_bump_direction is not None:
        cx += ux * 2 * chamfer - px * chamfer * first_bump_direction
        cy += uy * 2 * chamfer - py * chamfer * first_bump_direction
        new_path.append((cx, cy, layer))

    # Lead-out to end: use wider chamfer (2:1) to smoothly transition back to path
    if bump_count > 0:
        # Calculate perpendicular distance from current point to end_pt
        dx_to_end = end_pt[0] - cx
        dy_to_end = end_pt[1] - cy

        # Project onto perpendicular direction
        perp_dist = dx_to_end * px + dy_to_end * py

        # If we need to move perpendicular to reach end level, use wider chamfer
        if abs(perp_dist) > 0.01:
            # Move forward by 2x the perpendicular distance (2:1 ratio)
            chamfer_x = cx + ux * 2 * abs(perp_dist) + px * perp_dist
            chamfer_y = cy + uy * 2 * abs(perp_dist) + py * perp_dist
            new_path.append((chamfer_x, chamfer_y, layer))
            cx, cy = chamfer_x, chamfer_y

    new_path.append((end_pt[0], end_pt[1], layer))

    # Add remaining path points
    new_path.extend(float_path[end_idx + 1:])

    return new_path, bump_count


def get_safe_amplitude_for_diff_pair(
    cx: float, cy: float,
    ux: float, uy: float,
    px: float, py: float,
    direction: int,
    max_amplitude: float,
    min_amplitude: float,
    layer: int,
    pcb_data: PCBData,
    p_net_id: int,
    n_net_id: int,
    config: GridRouteConfig,
    spacing_mm: float,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    is_first_bump: bool = True
) -> float:
    """
    Find safe amplitude for a diff pair meander bump.

    Accounts for the full diff pair width when checking clearance.
    The centerline meander at amplitude A means P/N tracks extend to A + spacing_mm.
    """
    # Extra clearance for diff pair width
    # The P/N tracks are at ±spacing_mm from centerline
    # So the outer edge of the meander is at amplitude + spacing_mm + track_width/2
    diff_pair_extra = spacing_mm

    # Add margin for segment merging
    meander_clearance_margin = config.grid_step / 2
    required_clearance = config.track_width + config.clearance + meander_clearance_margin + diff_pair_extra
    via_clearance = config.via_size / 2 + config.track_width / 2 + config.clearance + meander_clearance_margin + diff_pair_extra
    chamfer = 0.1

    # Get layer name for comparison
    # config.layers is a list of layer names
    layer_names = config.layers if hasattr(config, 'layers') and isinstance(config.layers, list) else []
    layer_name = layer_names[layer] if layer < len(layer_names) else str(layer)

    # Combine all segments and vias
    all_segments = list(pcb_data.segments)
    if extra_segments:
        all_segments.extend(extra_segments)

    all_vias = list(pcb_data.vias)
    if extra_vias:
        all_vias.extend(extra_vias)

    # Binary search for safe amplitude
    test_amplitudes = [max_amplitude]
    amp = max_amplitude
    while amp > min_amplitude:
        amp *= 0.7
        if amp >= min_amplitude:
            test_amplitudes.append(amp)
    test_amplitudes.append(min_amplitude)

    for test_amp in test_amplitudes:
        # Generate bump segments for this amplitude (at centerline level)
        bump_segs = get_bump_segments(cx, cy, ux, uy, px, py, direction, test_amp, chamfer, is_first_bump)

        conflict_found = False

        # Check each bump segment
        for bx1, by1, bx2, by2 in bump_segs:
            if conflict_found:
                break

            for other_seg in all_segments:
                # Skip own nets
                if other_seg.net_id == p_net_id or other_seg.net_id == n_net_id:
                    continue
                if other_seg.layer != layer_name:
                    continue

                # Quick distance check
                seg_center_x = (other_seg.start_x + other_seg.end_x) / 2
                seg_center_y = (other_seg.start_y + other_seg.end_y) / 2
                seg_half_len = math.sqrt((other_seg.end_x - other_seg.start_x)**2 +
                                         (other_seg.end_y - other_seg.start_y)**2) / 2
                bump_center_x = (bx1 + bx2) / 2
                bump_center_y = (by1 + by2) / 2
                rough_dist = math.sqrt((bump_center_x - seg_center_x)**2 + (bump_center_y - seg_center_y)**2)

                if rough_dist > test_amp + seg_half_len + required_clearance + 1.0:
                    continue

                dist = segment_to_segment_distance(
                    bx1, by1, bx2, by2,
                    other_seg.start_x, other_seg.start_y, other_seg.end_x, other_seg.end_y
                )

                if dist < required_clearance:
                    conflict_found = True
                    break

        # Check vias
        if not conflict_found:
            for bx1, by1, bx2, by2 in bump_segs:
                if conflict_found:
                    break

                for via in all_vias:
                    if via.net_id == p_net_id or via.net_id == n_net_id:
                        continue

                    dist = point_to_segment_distance(via.x, via.y, bx1, by1, bx2, by2)

                    if dist < via_clearance:
                        conflict_found = True
                        break

        if not conflict_found:
            return test_amp

    return 0


def apply_meanders_to_diff_pair(
    result: dict,
    extra_length: float,
    config: GridRouteConfig,
    pcb_data: PCBData,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0,
    amplitude_override: float = None
) -> Tuple[dict, int]:
    """
    Apply meanders to a differential pair by modifying the centerline and regenerating P/N paths.

    Args:
        result: Original diff pair routing result
        extra_length: Length to add to centerline (mm)
        config: Routing configuration
        pcb_data: PCB data for clearance checking
        extra_segments: Additional segments to check against
        extra_vias: Additional vias to check against
        min_bumps: Minimum number of bumps to generate
        amplitude_override: Override amplitude for scaling

    Returns:
        (modified_result, bump_count)
    """
    from diff_pair_routing import create_parallel_path_float
    from routing_config import GridCoord

    # Extract data from result
    centerline_grid = result.get('centerline_path_grid')
    if not centerline_grid or len(centerline_grid) < 2:
        print(f"    Warning: No centerline path available for diff pair meanders")
        return result, 0

    # For routes with vias, we need to be careful - only meander sections that
    # don't affect via positions. Check if route is single-layer.
    first_layer = centerline_grid[0][2]
    is_single_layer = all(p[2] == first_layer for p in centerline_grid)

    p_sign = result.get('p_sign', 1)
    n_sign = result.get('n_sign', -1)
    spacing_mm = result.get('spacing_mm', (config.track_width + config.diff_pair_gap) / 2)
    start_stub_dir = result.get('start_stub_dir')
    end_stub_dir = result.get('end_stub_dir')
    p_net_id = result.get('p_net_id')
    n_net_id = result.get('n_net_id')
    layer_names = result.get('layer_names', [])
    p_start = result.get('p_start')
    n_start = result.get('n_start')
    p_end = result.get('p_end')
    n_end = result.get('n_end')
    src_stub_dir = result.get('src_stub_dir')
    tgt_stub_dir = result.get('tgt_stub_dir')

    coord = GridCoord(config.grid_step)
    amplitude = amplitude_override if amplitude_override is not None else config.meander_amplitude

    # Find straight runs in centerline
    min_length = amplitude * 2
    runs = find_straight_runs_in_path(centerline_grid, coord, min_length)

    if not runs:
        print(f"    Warning: No suitable straight run in diff pair centerline for meanders")
        return result, 0

    # Try each straight run
    for start_idx, end_idx, run_length in runs:
        if run_length < amplitude * 2:
            continue

        # For routes with vias, check if run is on a single layer
        if not is_single_layer:
            run_layer = centerline_grid[start_idx][2]
            # Make sure all points in run are on same layer
            if any(centerline_grid[i][2] != run_layer for i in range(start_idx, end_idx + 1)):
                continue

        # Generate meanders on centerline
        new_centerline_float, bump_count = generate_centerline_meander(
            centerline_grid, start_idx, end_idx,
            extra_length, amplitude, coord, config, spacing_mm,
            pcb_data, p_net_id, n_net_id, extra_segments, extra_vias, min_bumps
        )

        if bump_count == 0:
            continue

        from diff_pair_routing import create_parallel_path_from_float, _float_path_to_geometry, _calculate_parallel_extension, _process_via_positions

        # Unified path: always regenerate full P/N paths from modified centerline
        p_float_path = create_parallel_path_from_float(
            new_centerline_float, sign=p_sign, spacing_mm=spacing_mm,
            start_dir=start_stub_dir, end_dir=end_stub_dir
        )
        n_float_path = create_parallel_path_from_float(
            new_centerline_float, sign=n_sign, spacing_mm=spacing_mm,
            start_dir=start_stub_dir, end_dir=end_stub_dir
        )

        # For multi-layer routes, process via positions to maintain P/N parallelism
        if not is_single_layer:
            simplified_path_grid = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
                                    for x, y, layer in new_centerline_float]
            p_float_path, n_float_path = _process_via_positions(
                simplified_path_grid, p_float_path, n_float_path, coord, config,
                p_sign, n_sign, spacing_mm
            )

        # Recalculate extensions for source and target connectors
        p_src_route = p_float_path[0][:2] if p_float_path else p_start
        n_src_route = n_float_path[0][:2] if n_float_path else n_start
        src_p_ext, src_n_ext = _calculate_parallel_extension(
            p_start, n_start, p_src_route, n_src_route,
            src_stub_dir, p_sign
        )

        p_tgt_route = p_float_path[-1][:2] if p_float_path else p_end
        n_tgt_route = n_float_path[-1][:2] if n_float_path else n_end
        tgt_p_ext, tgt_n_ext = _calculate_parallel_extension(
            p_end, n_end, p_tgt_route, n_tgt_route,
            tgt_stub_dir, p_sign
        )

        # Convert paths to segments with proper connector handling
        new_segments = []
        new_vias = []

        p_segs, p_vias_new, _ = _float_path_to_geometry(
            p_float_path, p_net_id, p_start, p_end, p_sign,
            src_stub_dir, tgt_stub_dir, src_p_ext, tgt_p_ext,
            config, layer_names
        )
        new_segments.extend(p_segs)
        new_vias.extend(p_vias_new)

        n_segs, n_vias_new, _ = _float_path_to_geometry(
            n_float_path, n_net_id, n_start, n_end, n_sign,
            src_stub_dir, tgt_stub_dir, src_n_ext, tgt_n_ext,
            config, layer_names
        )
        new_segments.extend(n_segs)
        new_vias.extend(n_vias_new)

        # Also update the grid version for storage
        new_centerline_grid = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
                               for x, y, layer in new_centerline_float]

        # Recreate GND vias for multi-layer routes
        if not is_single_layer and config.gnd_via_enabled:
            from diff_pair_routing import _create_gnd_vias
            gnd_net_id = result.get('gnd_net_id')
            gnd_via_dirs = result.get('gnd_via_dirs', [])
            gnd_vias = _create_gnd_vias(
                new_centerline_grid, coord, config, layer_names, spacing_mm, gnd_net_id, gnd_via_dirs
            )
            new_vias.extend(gnd_vias)

        # Calculate actual routed length from P segments (includes connectors and via barrels)
        # This is more accurate than centerline length which misses connector segments
        from routing_utils import calculate_route_length
        p_routed_length = calculate_route_length(p_segs, p_vias_new, pcb_data)
        n_routed_length = calculate_route_length(n_segs, n_vias_new, pcb_data)
        avg_routed_length = (p_routed_length + n_routed_length) / 2

        # Update result
        modified_result = dict(result)
        modified_result['new_segments'] = new_segments
        modified_result['new_vias'] = new_vias
        modified_result['centerline_length'] = avg_routed_length  # Use actual routed length
        modified_result['route_length'] = avg_routed_length + result.get('stub_length', 0.0)
        modified_result['simplified_path'] = new_centerline_float
        modified_result['centerline_path_grid'] = new_centerline_grid

        return modified_result, bump_count

    print(f"    Warning: No straight run with clearance found for diff pair meanders")
    return result, 0


def _path_to_segments(float_path: List[Tuple[float, float, int]],
                       net_id: int,
                       track_width: float,
                       layer_names: List[str]) -> List[Segment]:
    """Convert a float path to a list of segments."""
    segments = []
    for i in range(len(float_path) - 1):
        x1, y1, layer1 = float_path[i]
        x2, y2, layer2 = float_path[i + 1]

        # Only create segment if on same layer
        if layer1 == layer2:
            layer_name = layer_names[layer1] if layer1 < len(layer_names) else f"layer_{layer1}"
            segments.append(Segment(
                start_x=x1, start_y=y1,
                end_x=x2, end_y=y2,
                width=track_width,
                layer=layer_name,
                net_id=net_id
            ))
    return segments


def _path_to_vias(float_path: List[Tuple[float, float, int]],
                   net_id: int,
                   config: GridRouteConfig,
                   layer_names: List[str]) -> List:
    """Generate vias at layer changes in a path."""
    from kicad_parser import Via

    vias = []
    for i in range(len(float_path) - 1):
        x1, y1, layer1 = float_path[i]
        x2, y2, layer2 = float_path[i + 1]

        if layer1 != layer2:
            layer1_name = layer_names[layer1] if layer1 < len(layer_names) else f"layer_{layer1}"
            layer2_name = layer_names[layer2] if layer2 < len(layer_names) else f"layer_{layer2}"
            vias.append(Via(
                x=x1, y=y1,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer1_name, layer2_name],
                net_id=net_id
            ))
    return vias


def _create_connector_segment(start_pos: Tuple[float, float],
                               end_pos,
                               net_id: int,
                               track_width: float,
                               layer_names: List[str],
                               layer_override: int = None) -> Optional[Segment]:
    """Create a connector segment from start position to end position.

    Args:
        start_pos: (x, y) tuple
        end_pos: Either (x, y) tuple or (x, y, layer) tuple
        net_id: Net ID
        track_width: Track width
        layer_names: List of layer names
        layer_override: Optional layer index to use instead of from end_pos
    """
    x1, y1 = start_pos[0], start_pos[1]

    # Handle end_pos which can be (x, y) or (x, y, layer)
    if len(end_pos) == 3:
        x2, y2, layer = end_pos
    else:
        x2, y2 = end_pos
        layer = layer_override if layer_override is not None else 0

    if layer_override is not None:
        layer = layer_override

    # Skip if points are very close
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    if dist < 0.01:
        return None

    layer_name = layer_names[layer] if layer < len(layer_names) else f"layer_{layer}"
    return Segment(
        start_x=x1, start_y=y1,
        end_x=x2, end_y=y2,
        width=track_width,
        layer=layer_name,
        net_id=net_id
    )
