"""
Differential pair routing functions.

Routes differential pairs (P and N nets) together using centerline + offset approach.
"""

import math
from typing import List, Optional, Tuple, Dict

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord, DiffPair
from routing_utils import (
    find_connected_groups, find_stub_free_ends, get_stub_direction, get_net_endpoints
)

# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap, GridRouter
except ImportError:
    GridObstacleMap = None
    GridRouter = None


def get_diff_pair_endpoints(pcb_data: PCBData, p_net_id: int, n_net_id: int,
                             config: GridRouteConfig) -> Tuple[List, List, str]:
    """
    Find source and target endpoints for a differential pair.

    Returns:
        (sources, targets, error_message)
        - sources: List of (p_gx, p_gy, n_gx, n_gy, layer_idx)
        - targets: List of (p_gx, p_gy, n_gx, n_gy, layer_idx)
        - error_message: None if successful, otherwise describes why routing can't proceed
    """
    coord = GridCoord(config.grid_step)
    layer_map = {name: idx for idx, name in enumerate(config.layers)}

    # Get endpoints for P and N nets separately
    # Use stub free ends for diff pairs to get the actual stub tips
    p_sources, p_targets, p_error = get_net_endpoints(pcb_data, p_net_id, config, use_stub_free_ends=True)
    n_sources, n_targets, n_error = get_net_endpoints(pcb_data, n_net_id, config, use_stub_free_ends=True)

    if p_error:
        return [], [], f"P net: {p_error}"
    if n_error:
        return [], [], f"N net: {n_error}"

    if not p_sources or not p_targets:
        return [], [], "P net has no valid source/target endpoints"
    if not n_sources or not n_targets:
        return [], [], "N net has no valid source/target endpoints"

    # Determine source vs target based on proximity between P and N endpoints.
    # The "source" side is where P and N endpoints are closest together,
    # and "target" side is where the other P and N endpoints are closest.
    # This ensures P and N agree on which end is source vs target.

    # Combine all P endpoints and all N endpoints (ignoring the source/target
    # classification from get_net_endpoints since it may differ between P and N)
    p_all = p_sources + p_targets
    n_all = n_sources + n_targets

    def find_closest_pair(p_endpoints, n_endpoints):
        """Find the P and N endpoints that are closest to each other on same layer."""
        best_p, best_n = None, None
        best_dist = float('inf')
        for p in p_endpoints:
            p_gx, p_gy, p_layer = p[0], p[1], p[2]
            for n in n_endpoints:
                n_gx, n_gy, n_layer = n[0], n[1], n[2]
                if n_layer != p_layer:
                    continue
                dist = abs(p_gx - n_gx) + abs(p_gy - n_gy)
                if dist < best_dist:
                    best_dist = dist
                    best_p, best_n = p, n
        return best_p, best_n, best_dist

    # Find the closest P-N pair - this becomes one end (we'll call it "source")
    p_src, n_src, src_dist = find_closest_pair(p_all, n_all)
    if p_src is None or n_src is None:
        return [], [], "Could not find matching P and N endpoints on same layer"

    # Remove the matched endpoints from consideration
    p_remaining = [p for p in p_all if p != p_src]
    n_remaining = [n for n in n_all if n != n_src]

    if not p_remaining or not n_remaining:
        return [], [], "Not enough endpoints for source and target"

    # Find the closest P-N pair from remaining - this becomes the other end ("target")
    p_tgt, n_tgt, tgt_dist = find_closest_pair(p_remaining, n_remaining)
    if p_tgt is None or n_tgt is None:
        return [], [], "Could not find matching P and N target endpoints on same layer"

    # Build the paired source and target tuples
    paired_sources = [(
        p_src[0], p_src[1],  # P grid coords
        n_src[0], n_src[1],  # N grid coords
        p_src[2],  # layer
        p_src[3], p_src[4],  # P original coords
        n_src[3], n_src[4]  # N original coords
    )]

    paired_targets = [(
        p_tgt[0], p_tgt[1],  # P grid coords
        n_tgt[0], n_tgt[1],  # N grid coords
        p_tgt[2],  # layer
        p_tgt[3], p_tgt[4],  # P original coords
        n_tgt[3], n_tgt[4]  # N original coords
    )]

    if not paired_sources:
        return [], [], "Could not match P and N source endpoints"
    if not paired_targets:
        return [], [], "Could not match P and N target endpoints"

    return paired_sources, paired_targets, None


def simplify_path(path):
    """
    Simplify a path by removing intermediate points on straight lines.
    Only keeps corner points and endpoints.

    Args:
        path: List of (gx, gy, layer) grid coordinates

    Returns:
        Simplified path with collinear points removed
    """
    if len(path) <= 2:
        return list(path)

    result = [path[0]]

    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        next_pt = path[i + 1]

        # Keep if layer changes
        if prev[2] != curr[2] or curr[2] != next_pt[2]:
            result.append(curr)
            continue

        # Direction vectors
        dx1 = curr[0] - prev[0]
        dy1 = curr[1] - prev[1]
        dx2 = next_pt[0] - curr[0]
        dy2 = next_pt[1] - curr[1]

        # Normalize (handle zero case)
        len1 = math.sqrt(dx1*dx1 + dy1*dy1) or 1
        len2 = math.sqrt(dx2*dx2 + dy2*dy2) or 1

        # Check if directions are the same (collinear)
        ndx1, ndy1 = dx1/len1, dy1/len1
        ndx2, ndy2 = dx2/len2, dy2/len2

        # Not collinear if directions differ (keep this corner point)
        if abs(ndx1 - ndx2) > 0.01 or abs(ndy1 - ndy2) > 0.01:
            result.append(curr)

    result.append(path[-1])
    return result


def smooth_path_for_diffpair(path, min_segment_length=3):
    """
    Smooth a path by removing short zigzag segments that cause diff pair crossing.

    For differential pairs, short alternating segments (e.g., diagonal-horizontal-diagonal)
    cause perpendicular offsets to swing wildly. This function removes intermediate points
    that create very short segments.

    Args:
        path: List of (gx, gy, layer) grid coordinates (already simplified)
        min_segment_length: Minimum segment length in grid units to keep

    Returns:
        Smoothed path with short zigzag segments removed
    """
    if len(path) <= 2:
        return list(path)

    result = [path[0]]

    i = 1
    while i < len(path) - 1:
        curr = path[i]

        # Check segment lengths
        prev = result[-1]
        next_pt = path[i + 1]

        # Distance from previous kept point to current
        dist_prev = math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)

        # Distance from current to next
        dist_next = math.sqrt((next_pt[0] - curr[0])**2 + (next_pt[1] - curr[1])**2)

        # Keep if layer changes
        if prev[2] != curr[2] or curr[2] != next_pt[2]:
            result.append(curr)
            i += 1
            continue

        # Skip if both segments are short (part of a zigzag)
        if dist_prev < min_segment_length and dist_next < min_segment_length:
            # Skip this point, let it connect directly
            i += 1
            continue

        result.append(curr)
        i += 1

    result.append(path[-1])
    return result


def create_parallel_path_float(centerline_path, coord, sign, spacing_mm=0.1, start_dir=None, end_dir=None):
    """
    Create a path parallel to centerline using floating-point perpendicular offsets.

    Uses bisector-based offsets at corners for smooth parallel paths.

    Args:
        centerline_path: List of (gx, gy, layer) grid coordinates
        coord: GridCoord converter for grid<->float
        sign: +1 for one track, -1 for the other
        spacing_mm: Distance from centerline in mm
        start_dir: Optional (dx, dy) direction to use at start point for perpendicular calc
        end_dir: Optional (dx, dy) direction to use at end point for perpendicular calc

    Returns:
        List of (x, y, layer) floating-point coordinates
    """
    if len(centerline_path) < 2:
        return [(coord.to_float(p[0], p[1])[0], coord.to_float(p[0], p[1])[1], p[2])
                for p in centerline_path]

    result = []

    for i in range(len(centerline_path)):
        gx, gy, layer = centerline_path[i]
        x, y = coord.to_float(gx, gy)

        use_corner_scale = True  # Only apply corner scaling for bisector calculations
        if i == 0:
            # First point: use start_dir if provided, else perpendicular to first segment
            if start_dir is not None:
                dx, dy = start_dir
                use_corner_scale = False  # Provided direction, no corner scaling
            else:
                next_x, next_y = coord.to_float(centerline_path[1][0], centerline_path[1][1])
                dx, dy = next_x - x, next_y - y
                use_corner_scale = False  # Single segment, no corner scaling
        elif i == len(centerline_path) - 1:
            # Last point: use end_dir if provided, else perpendicular to last segment
            if end_dir is not None:
                dx, dy = end_dir
                use_corner_scale = False  # Provided direction, no corner scaling
            else:
                prev_x, prev_y = coord.to_float(centerline_path[i-1][0], centerline_path[i-1][1])
                dx, dy = x - prev_x, y - prev_y
                use_corner_scale = False  # Single segment, no corner scaling
        else:
            # Corner: use bisector of incoming and outgoing directions
            prev = centerline_path[i-1]
            next_pt = centerline_path[i+1]

            if prev[2] != layer or next_pt[2] != layer:
                # Layer change - use incoming direction
                prev_x, prev_y = coord.to_float(prev[0], prev[1])
                dx, dy = x - prev_x, y - prev_y
            else:
                prev_x, prev_y = coord.to_float(prev[0], prev[1])
                next_x, next_y = coord.to_float(next_pt[0], next_pt[1])

                dx_in, dy_in = x - prev_x, y - prev_y
                dx_out, dy_out = next_x - x, next_y - y

                len_in = math.sqrt(dx_in*dx_in + dy_in*dy_in) or 1
                len_out = math.sqrt(dx_out*dx_out + dy_out*dy_out) or 1

                # Bisector direction (sum of unit vectors)
                dx = dx_in/len_in + dx_out/len_out
                dy = dy_in/len_in + dy_out/len_out

                if abs(dx) < 0.01 and abs(dy) < 0.01:
                    dx, dy = dx_in, dy_in

        # Normalize and compute perpendicular offset
        length = math.sqrt(dx*dx + dy*dy) or 1
        ndx, ndy = dx/length, dy/length

        # Corner compensation: scale offset by 2/length to maintain perpendicular distance
        # When summing two unit vectors, length = 2*cos(theta/2) where theta is angle between them
        # To maintain spacing_mm perpendicular to each segment, multiply by 2/length
        # Cap the scaling to avoid extreme miter extensions at very sharp corners (>135 deg turn)
        # Only apply at actual corners (bisector calculations), not at endpoints
        if use_corner_scale:
            corner_scale = min(2.0 / length, 3.0) if length > 0.1 else 1.0
        else:
            corner_scale = 1.0

        perp_x = -ndy * sign * spacing_mm * corner_scale
        perp_y = ndx * sign * spacing_mm * corner_scale

        result.append((x + perp_x, y + perp_y, layer))

    return result


def route_diff_pair_with_obstacles(pcb_data: PCBData, diff_pair: DiffPair,
                                    config: GridRouteConfig,
                                    obstacles: GridObstacleMap) -> Optional[dict]:
    """
    Route a differential pair using centerline + offset approach.

    1. Routes a single centerline path using A* (GridRouter)
    2. Simplifies the path by removing collinear points
    3. Creates P and N paths using perpendicular offsets from centerline
    """
    p_net_id = diff_pair.p_net_id
    n_net_id = diff_pair.n_net_id

    # Find endpoints
    sources, targets, error = get_diff_pair_endpoints(pcb_data, p_net_id, n_net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Get P and N source/target coordinates
    src = sources[0]
    tgt = targets[0]

    p_src_gx, p_src_gy = src[0], src[1]
    n_src_gx, n_src_gy = src[2], src[3]
    src_layer = src[4]

    p_tgt_gx, p_tgt_gy = tgt[0], tgt[1]
    n_tgt_gx, n_tgt_gy = tgt[2], tgt[3]
    tgt_layer = tgt[4]

    # Get original stub positions (in mm)
    p_src_x, p_src_y = src[5], src[6]
    n_src_x, n_src_y = src[7], src[8]
    p_tgt_x, p_tgt_y = tgt[5], tgt[6]
    n_tgt_x, n_tgt_y = tgt[7], tgt[8]

    # Calculate spacing from actual P-N stub distance (use average of source and target)
    src_pn_dist = math.sqrt((p_src_x - n_src_x)**2 + (p_src_y - n_src_y)**2)
    tgt_pn_dist = math.sqrt((p_tgt_x - n_tgt_x)**2 + (p_tgt_y - n_tgt_y)**2)
    avg_pn_dist = (src_pn_dist + tgt_pn_dist) / 2
    spacing_mm = avg_pn_dist / 2  # Half-spacing for each track from centerline
    print(f"  P-N spacing: src={src_pn_dist:.3f}mm, tgt={tgt_pn_dist:.3f}mm, using={avg_pn_dist:.3f}mm (offset={spacing_mm:.3f}mm)")

    # Get segments for P and N nets to find stub directions
    p_segments = [s for s in pcb_data.segments if s.net_id == p_net_id]
    n_segments = [s for s in pcb_data.segments if s.net_id == n_net_id]

    src_layer_name = layer_names[src_layer]
    tgt_layer_name = layer_names[tgt_layer]

    # Get stub directions at source and target
    p_src_dir = get_stub_direction(p_segments, p_src_x, p_src_y, src_layer_name)
    n_src_dir = get_stub_direction(n_segments, n_src_x, n_src_y, src_layer_name)
    p_tgt_dir = get_stub_direction(p_segments, p_tgt_x, p_tgt_y, tgt_layer_name)
    n_tgt_dir = get_stub_direction(n_segments, n_tgt_x, n_tgt_y, tgt_layer_name)

    # Average P and N directions at each end (they should be similar for a diff pair)
    src_dir_x = (p_src_dir[0] + n_src_dir[0]) / 2
    src_dir_y = (p_src_dir[1] + n_src_dir[1]) / 2
    tgt_dir_x = (p_tgt_dir[0] + n_tgt_dir[0]) / 2
    tgt_dir_y = (p_tgt_dir[1] + n_tgt_dir[1]) / 2

    # Normalize the averaged directions
    src_dir_len = math.sqrt(src_dir_x*src_dir_x + src_dir_y*src_dir_y)
    tgt_dir_len = math.sqrt(tgt_dir_x*tgt_dir_x + tgt_dir_y*tgt_dir_y)
    if src_dir_len > 0:
        src_dir_x /= src_dir_len
        src_dir_y /= src_dir_len
    if tgt_dir_len > 0:
        tgt_dir_x /= tgt_dir_len
        tgt_dir_y /= tgt_dir_len

    # Calculate centerline midpoints between P and N stubs
    center_src_x = (p_src_x + n_src_x) / 2
    center_src_y = (p_src_y + n_src_y) / 2
    center_tgt_x = (p_tgt_x + n_tgt_x) / 2
    center_tgt_y = (p_tgt_y + n_tgt_y) / 2

    # Get setback range
    min_setback = config.min_diff_pair_centerline_setback
    max_setback = config.max_diff_pair_centerline_setback

    print(f"  Centerline setback: {min_setback}mm to {max_setback}mm")
    print(f"  Source direction: ({src_dir_x:.2f}, {src_dir_y:.2f}), target direction: ({tgt_dir_x:.2f}, {tgt_dir_y:.2f})")

    # Generate candidate points along the setback line (from min to max distance)
    # Sample at grid_step intervals
    num_samples = max(1, int((max_setback - min_setback) / config.grid_step) + 1)

    center_sources = []
    center_targets = []

    allow_radius = 2
    for i in range(num_samples):
        if num_samples > 1:
            setback = min_setback + (max_setback - min_setback) * i / (num_samples - 1)
        else:
            setback = min_setback

        # Calculate source candidate
        if src_dir_len > 0:
            src_x = center_src_x + src_dir_x * setback
            src_y = center_src_y + src_dir_y * setback
            src_gx, src_gy = coord.to_grid(src_x, src_y)

            # Check if blocked
            if not obstacles.is_blocked(src_gx, src_gy, src_layer):
                center_sources.append((src_gx, src_gy, src_layer))
                # Add allowed cells around this point
                for dx in range(-allow_radius, allow_radius + 1):
                    for dy in range(-allow_radius, allow_radius + 1):
                        obstacles.add_allowed_cell(src_gx + dx, src_gy + dy)
                obstacles.add_source_target_cell(src_gx, src_gy, src_layer)

        # Calculate target candidate
        if tgt_dir_len > 0:
            tgt_x = center_tgt_x + tgt_dir_x * setback
            tgt_y = center_tgt_y + tgt_dir_y * setback
            tgt_gx, tgt_gy = coord.to_grid(tgt_x, tgt_y)

            # Check if blocked
            if not obstacles.is_blocked(tgt_gx, tgt_gy, tgt_layer):
                center_targets.append((tgt_gx, tgt_gy, tgt_layer))
                # Add allowed cells around this point
                for dx in range(-allow_radius, allow_radius + 1):
                    for dy in range(-allow_radius, allow_radius + 1):
                        obstacles.add_allowed_cell(tgt_gx + dx, tgt_gy + dy)
                obstacles.add_source_target_cell(tgt_gx, tgt_gy, tgt_layer)

    # Remove duplicate grid coordinates
    center_sources = list(dict.fromkeys(center_sources))
    center_targets = list(dict.fromkeys(center_targets))

    if not center_sources:
        print(f"  No valid source candidates found (all blocked in setback range)")
        return {'failed': True, 'iterations': 0}
    if not center_targets:
        print(f"  No valid target candidates found (all blocked in setback range)")
        return {'failed': True, 'iterations': 0}

    print(f"  Found {len(center_sources)} source candidates, {len(center_targets)} target candidates")

    # Create router for centerline
    router = GridRouter(via_cost=config.via_cost * 1000, h_weight=config.heuristic_weight)

    path, iterations = router.route_multi(obstacles, center_sources, center_targets, config.max_iterations)
    total_iterations = iterations

    if path is None:
        # Try reverse direction
        print(f"No route found after {iterations} iterations, trying backwards...")
        path, iter2 = router.route_multi(obstacles, center_targets, center_sources, config.max_iterations)
        total_iterations += iter2
        if path is not None:
            path = list(reversed(path))

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return {'failed': True, 'iterations': total_iterations}

    # Count layer transitions (via changes) in the path
    layer_transitions = 0
    for i in range(1, len(path)):
        if path[i][2] != path[i-1][2]:
            layer_transitions += 1

    # Limit to single via transition for diff pairs
    if layer_transitions > 1:
        print(f"Route found but has {layer_transitions} layer transitions (max 1 allowed for diff pairs)")
        return {'failed': True, 'iterations': total_iterations}

    # Simplify path by removing collinear points
    simplified_path = simplify_path(path)
    print(f"Route found in {total_iterations} iterations, path: {len(path)} -> {len(simplified_path)} points")

    # Store paths for debug output (converted to float coordinates)
    raw_astar_path = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], layer)
                      for gx, gy, layer in path]
    simplified_path_float = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], layer)
                             for gx, gy, layer in simplified_path]

    # Add turn segments at start and end to face the connector stubs
    # This makes the centerline turn to align with stub direction before connecting
    # Use a shorter turn length to reduce connector segment length
    turn_length_mm = setback * 0.3  # Length of the turn segment in mm
    turn_length_grid = int(turn_length_mm / config.grid_step)

    if len(simplified_path) >= 2 and turn_length_grid > 0:
        # Add turn segment at start (facing source stubs)
        first_gx, first_gy, first_layer = simplified_path[0]
        # Move backward along stub direction to create turn point
        turn_start_gx = first_gx - int(src_dir_x * turn_length_grid)
        turn_start_gy = first_gy - int(src_dir_y * turn_length_grid)
        simplified_path.insert(0, (turn_start_gx, turn_start_gy, first_layer))

        # Add turn segment at end (facing target stubs)
        last_gx, last_gy, last_layer = simplified_path[-1]
        # Move toward stubs (opposite of tgt_dir which points outward from stubs)
        turn_end_gx = last_gx - int(tgt_dir_x * turn_length_grid)
        turn_end_gy = last_gy - int(tgt_dir_y * turn_length_grid)
        simplified_path.append((turn_end_gx, turn_end_gy, last_layer))

        print(f"  Added turn segments: path now {len(simplified_path)} points")


    # Determine which side of the centerline P is on
    # Use the first segment direction of the simplified path
    if len(simplified_path) >= 2:
        first_gx, first_gy, _ = simplified_path[0]
        second_gx, second_gy, _ = simplified_path[1]
        first_x, first_y = coord.to_float(first_gx, first_gy)
        second_x, second_y = coord.to_float(second_gx, second_gy)
        path_dir_x = second_x - first_x
        path_dir_y = second_y - first_y
    else:
        path_dir_x, path_dir_y = 1.0, 0.0

    # Vector from source midpoint to P source position
    src_midpoint_x = (p_src_x + n_src_x) / 2
    src_midpoint_y = (p_src_y + n_src_y) / 2
    to_p_dx = p_src_x - src_midpoint_x
    to_p_dy = p_src_y - src_midpoint_y

    # Cross product: determines which side of the path direction P is on at source
    src_cross = path_dir_x * to_p_dy - path_dir_y * to_p_dx
    src_p_sign = +1 if src_cross >= 0 else -1

    # Calculate target polarity using path direction at target end
    # Path arrives at target, so use negative of last segment direction
    if len(simplified_path) >= 2:
        last_gx1, last_gy1, _ = simplified_path[-2]
        last_gx2, last_gy2, _ = simplified_path[-1]
        last_cx1, last_cy1 = coord.to_float(last_gx1, last_gy1)
        last_cx2, last_cy2 = coord.to_float(last_gx2, last_gy2)
        last_len = math.sqrt((last_cx2 - last_cx1)**2 + (last_cy2 - last_cy1)**2)
        if last_len > 0.001:
            tgt_path_dir_x = (last_cx2 - last_cx1) / last_len
            tgt_path_dir_y = (last_cy2 - last_cy1) / last_len
        else:
            tgt_path_dir_x, tgt_path_dir_y = path_dir_x, path_dir_y
    else:
        tgt_path_dir_x, tgt_path_dir_y = path_dir_x, path_dir_y

    # Vector from target midpoint to P target position
    tgt_midpoint_x = (p_tgt_x + n_tgt_x) / 2
    tgt_midpoint_y = (p_tgt_y + n_tgt_y) / 2
    tgt_to_p_dx = p_tgt_x - tgt_midpoint_x
    tgt_to_p_dy = p_tgt_y - tgt_midpoint_y

    # Cross product: determines which side of the path direction P is on at target
    tgt_cross = tgt_path_dir_x * tgt_to_p_dy - tgt_path_dir_y * tgt_to_p_dx
    tgt_p_sign = +1 if tgt_cross >= 0 else -1

    # Check if polarity differs between source and target
    polarity_swap_needed = (src_p_sign != tgt_p_sign)

    # Check if path has layer changes (vias)
    has_layer_change = False
    for i in range(len(simplified_path) - 1):
        if simplified_path[i][2] != simplified_path[i + 1][2]:
            has_layer_change = True
            break

    # Choose polarity based on whether we have vias for the swap
    if polarity_swap_needed and has_layer_change:
        # Use TARGET polarity - the swap happens on the source/approach side (F.Cu)
        # This keeps the target/exit side (In1.Cu) clean with no crossover needed
        p_sign = tgt_p_sign
        print(f"  Polarity swap with vias: using TARGET polarity p_sign={p_sign}")
    elif polarity_swap_needed and not has_layer_change:
        # WARNING: Polarity swap needed but no layer change (no vias) to perform the swap
        # This will result in crossing tracks on the same layer
        p_sign = src_p_sign
        print(f"  WARNING: Polarity swap needed but no vias - tracks will cross!")
    else:
        # Use source polarity (no swap needed)
        p_sign = src_p_sign
        print(f"  Source polarity: p_sign={p_sign}")

    if polarity_swap_needed:
        print(f"  (src_p_sign={src_p_sign}, tgt_p_sign={tgt_p_sign}, has_vias={has_layer_change})")

    n_sign = -p_sign

    # Create P and N paths using perpendicular offsets from centerline
    # Use stub directions at endpoints so perpendicular offsets align with stub P-N axis
    start_stub_dir = (src_dir_x, src_dir_y)
    end_stub_dir = (-tgt_dir_x, -tgt_dir_y)  # Negate because we arrive at target stubs
    p_float_path = create_parallel_path_float(simplified_path, coord, sign=p_sign, spacing_mm=spacing_mm,
                                               start_dir=start_stub_dir, end_dir=end_stub_dir)
    n_float_path = create_parallel_path_float(simplified_path, coord, sign=n_sign, spacing_mm=spacing_mm,
                                               start_dir=start_stub_dir, end_dir=end_stub_dir)

    # Fix the first/last points to align with actual stub positions
    # This ensures connectors are parallel (same direction) rather than converging
    # Calculate where each path should start/end based on stub positions + stub direction
    connector_length = turn_length_mm  # Distance from stub to turn point
    connector_spread = 0.05  # Spread connector endpoints by this fraction to add clearance margin
    min_track_clearance = config.track_width + config.clearance  # Min center-to-center distance

    if p_float_path and n_float_path and len(p_float_path) >= 2:
        # Calculate P-N offset vectors at source and target
        src_pn_dx = (n_src_x - p_src_x) / 2  # Half-offset from centerline to each track
        src_pn_dy = (n_src_y - p_src_y) / 2
        tgt_pn_dx = (n_tgt_x - p_tgt_x) / 2
        tgt_pn_dy = (n_tgt_y - p_tgt_y) / 2

        # Replace first points with positions directly in front of each stub
        # Spread outward slightly to add clearance margin at transition to centerline
        p_float_path[0] = (p_src_x + src_dir_x * connector_length - src_pn_dx * connector_spread,
                          p_src_y + src_dir_y * connector_length - src_pn_dy * connector_spread,
                          p_float_path[0][2])
        n_float_path[0] = (n_src_x + src_dir_x * connector_length + src_pn_dx * connector_spread,
                          n_src_y + src_dir_y * connector_length + src_pn_dy * connector_spread,
                          n_float_path[0][2])

        # Replace last points with positions directly in front of each target stub
        p_float_path[-1] = (p_tgt_x + tgt_dir_x * connector_length - tgt_pn_dx * connector_spread,
                           p_tgt_y + tgt_dir_y * connector_length - tgt_pn_dy * connector_spread,
                           p_float_path[-1][2])
        n_float_path[-1] = (n_tgt_x + tgt_dir_x * connector_length + tgt_pn_dx * connector_spread,
                           n_tgt_y + tgt_dir_y * connector_length + tgt_pn_dy * connector_spread,
                           n_float_path[-1][2])

        # Check connector endpoints against opposite track and shorten if too close
        def point_to_path_dist(px, py, path):
            """Min distance from point to any segment in path (skipping endpoints)."""
            min_dist = float('inf')
            for i in range(1, len(path) - 1):
                x1, y1, _ = path[i]
                x2, y2, _ = path[i + 1]
                seg_len_sq = (x2 - x1)**2 + (y2 - y1)**2
                if seg_len_sq > 0.0001:
                    t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / seg_len_sq))
                    dist = math.sqrt((px - x1 - t * (x2 - x1))**2 + (py - y1 - t * (y2 - y1))**2)
                    min_dist = min(min_dist, dist)
            return min_dist

        def check_and_shorten_connectors(p_path, n_path, p_idx, n_idx,
                                         p_base, n_base, dir_x, dir_y, pn_dx, pn_dy):
            """Check clearance and shorten connector if needed. Returns True if shortened."""
            p_x, p_y, p_layer = p_path[p_idx]
            n_x, n_y, n_layer = n_path[n_idx]
            min_dist = min(point_to_path_dist(p_x, p_y, n_path),
                          point_to_path_dist(n_x, n_y, p_path))
            deficit = min_track_clearance - min_dist
            if deficit > 0.001:
                new_len = max(0.01, connector_length - deficit * 3)
                p_path[p_idx] = (p_base[0] + dir_x * new_len - pn_dx * connector_spread,
                                p_base[1] + dir_y * new_len - pn_dy * connector_spread, p_layer)
                n_path[n_idx] = (n_base[0] + dir_x * new_len + pn_dx * connector_spread,
                                n_base[1] + dir_y * new_len + pn_dy * connector_spread, n_layer)

        # Check source and target connectors
        check_and_shorten_connectors(p_float_path, n_float_path, 0, 0,
                                     (p_src_x, p_src_y), (n_src_x, n_src_y),
                                     src_dir_x, src_dir_y, src_pn_dx, src_pn_dy)
        check_and_shorten_connectors(p_float_path, n_float_path, -1, -1,
                                     (p_tgt_x, p_tgt_y), (n_tgt_x, n_tgt_y),
                                     tgt_dir_x, tgt_dir_y, tgt_pn_dx, tgt_pn_dy)

    # Process via positions and polarity swaps if needed
    p_float_path, n_float_path = _process_via_positions(
        simplified_path, p_float_path, n_float_path, coord, config,
        p_sign, n_sign, spacing_mm, polarity_swap_needed
    )

    # Convert floating-point paths to segments and vias
    new_segments = []
    new_vias = []

    def float_path_to_geometry(float_path, net_id, original_start, original_end):
        """Convert floating-point path (x, y, layer) to segments and vias."""
        segs = []
        vias = []

        def get_layer_name(layer_idx):
            """Get layer name, handling special debug indices."""
            if layer_idx == -4:
                return 'In4.Cu'
            elif layer_idx == -5:
                return 'In5.Cu'
            elif layer_idx == -6:
                return 'In6.Cu'
            elif layer_idx == -7:
                return 'In7.Cu'
            else:
                return layer_names[layer_idx]

        # Add connecting segment from original start if needed
        if original_start and len(float_path) > 0:
            first_x, first_y, first_layer = float_path[0]
            orig_x, orig_y = original_start
            if abs(orig_x - first_x) > 0.001 or abs(orig_y - first_y) > 0.001:
                seg_layer = get_layer_name(first_layer)
                segs.append(Segment(
                    start_x=orig_x, start_y=orig_y,
                    end_x=first_x, end_y=first_y,
                    width=config.track_width,
                    layer=seg_layer,
                    net_id=net_id
                ))

        # Convert path
        for i in range(len(float_path) - 1):
            x1, y1, layer1 = float_path[i]
            x2, y2, layer2 = float_path[i + 1]

            # Check if this is a real layer change (not debug layer transition)
            real_layer1 = layer1 if layer1 >= 0 else 1  # Map debug layers to In1.Cu for via logic
            real_layer2 = layer2 if layer2 >= 0 else 1

            if real_layer1 != real_layer2:
                vias.append(Via(
                    x=x1, y=y1,
                    size=config.via_size,
                    drill=config.via_drill,
                    layers=[get_layer_name(layer1), get_layer_name(layer2)],
                    net_id=net_id
                ))
            elif abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                seg_layer = get_layer_name(layer1)
                segs.append(Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=seg_layer,
                    net_id=net_id
                ))

        # Add connecting segment to original end if needed
        if original_end and len(float_path) > 0:
            last_x, last_y, last_layer = float_path[-1]
            orig_x, orig_y = original_end
            if abs(orig_x - last_x) > 0.001 or abs(orig_y - last_y) > 0.001:
                seg_layer = layer_names[last_layer]
                segs.append(Segment(
                    start_x=last_x, start_y=last_y,
                    end_x=orig_x, end_y=orig_y,
                    width=config.track_width,
                    layer=seg_layer,
                    net_id=net_id
                ))

        return segs, vias

    # Get original coordinates for P and N nets
    p_start = (sources[0][5], sources[0][6]) if sources else None
    p_end = (targets[0][5], targets[0][6]) if targets else None
    n_start = (sources[0][7], sources[0][8]) if sources else None
    n_end = (targets[0][7], targets[0][8]) if targets else None

    # Convert P path
    p_segs, p_vias = float_path_to_geometry(p_float_path, p_net_id, p_start, p_end)
    new_segments.extend(p_segs)
    new_vias.extend(p_vias)

    # Convert N path
    n_segs, n_vias = float_path_to_geometry(n_float_path, n_net_id, n_start, n_end)
    new_segments.extend(n_segs)
    new_vias.extend(n_vias)

    # Convert float paths back to grid format for return value
    p_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in p_float_path]
    n_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in n_float_path]

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(simplified_path),
        'p_path': p_path,
        'n_path': n_path,
        'raw_astar_path': raw_astar_path,
        'simplified_path': simplified_path_float,
    }


def _process_via_positions(simplified_path, p_float_path, n_float_path, coord, config,
                           p_sign, n_sign, spacing_mm, polarity_swap_needed):
    """
    Process via positions at layer changes to be perpendicular to centerline direction.

    This ensures P and N vias form a line perpendicular to the centerline path and
    handles polarity swaps when needed.
    """
    min_via_spacing = config.via_size + config.clearance  # Minimum center-to-center distance
    track_via_clearance = (config.clearance + config.track_width / 2 + config.via_size / 2) * 1.15

    if not p_float_path or not n_float_path or len(simplified_path) < 2:
        return p_float_path, n_float_path

    for i in range(len(simplified_path) - 1):
        gx1, gy1, layer1 = simplified_path[i]
        gx2, gy2, layer2 = simplified_path[i + 1]

        if layer1 != layer2:
            # Layer change detected - calculate centerline position
            cx, cy = coord.to_float(gx1, gy1)

            # Get incoming direction (from previous point to via)
            if i > 0:
                prev_gx, prev_gy, _ = simplified_path[i - 1]
                prev_x, prev_y = coord.to_float(prev_gx, prev_gy)
                in_dir_x, in_dir_y = cx - prev_x, cy - prev_y
                in_len = math.sqrt(in_dir_x**2 + in_dir_y**2)
                if in_len > 0.001:
                    in_dir_x, in_dir_y = in_dir_x / in_len, in_dir_y / in_len
                else:
                    in_dir_x, in_dir_y = 1.0, 0.0
            else:
                in_dir_x, in_dir_y = None, None

            # Get outgoing direction (from via to next point after layer change)
            if i + 2 < len(simplified_path):
                next_gx, next_gy, _ = simplified_path[i + 2]
                next_x, next_y = coord.to_float(next_gx, next_gy)
                out_dir_x, out_dir_y = next_x - cx, next_y - cy
                out_len = math.sqrt(out_dir_x**2 + out_dir_y**2)
                if out_len > 0.001:
                    out_dir_x, out_dir_y = out_dir_x / out_len, out_dir_y / out_len
                else:
                    out_dir_x, out_dir_y = None, None
            else:
                out_dir_x, out_dir_y = None, None

            # Calculate perpendicular direction for via-via axis
            if in_dir_x is not None and out_dir_x is not None:
                perp_x = (-in_dir_y + -out_dir_y) / 2
                perp_y = (in_dir_x + out_dir_x) / 2
                perp_len = math.sqrt(perp_x**2 + perp_y**2)
                if perp_len > 0.001:
                    perp_x, perp_y = perp_x / perp_len, perp_y / perp_len
                else:
                    perp_x, perp_y = -in_dir_y, in_dir_x
            elif in_dir_x is not None:
                perp_x, perp_y = -in_dir_y, in_dir_x
            elif out_dir_x is not None:
                perp_x, perp_y = -out_dir_y, out_dir_x
            else:
                perp_x, perp_y = 1.0, 0.0

            # Use larger spacing for vias if needed
            min_via_spacing_for_track = track_via_clearance - spacing_mm
            via_spacing = max(spacing_mm, min_via_spacing / 2, min_via_spacing_for_track)

            # Calculate P and N via positions along the via-via axis
            p_via_x = cx + perp_x * p_sign * via_spacing
            p_via_y = cy + perp_y * p_sign * via_spacing
            n_via_x = cx + perp_x * n_sign * via_spacing
            n_via_y = cy + perp_y * n_sign * via_spacing

            # For polarity swap: swap via positions
            if polarity_swap_needed:
                p_via_x, n_via_x = n_via_x, p_via_x
                p_via_y, n_via_y = n_via_y, p_via_y

            # Calculate approach and exit positions
            diff_pair_spacing = spacing_mm * 2

            if in_dir_x is not None and out_dir_x is not None:
                in_perp_x, in_perp_y = -in_dir_y, in_dir_x
                out_perp_x, out_perp_y = -out_dir_y, out_dir_x

                # Determine inner via
                cross = in_dir_x * out_dir_y - in_dir_y * out_dir_x
                if cross < 0:
                    inner_via_x = p_via_x if p_sign < 0 else n_via_x
                    inner_via_y = p_via_y if p_sign < 0 else n_via_y
                    inner_is_p = (p_sign < 0)
                else:
                    inner_via_x = p_via_x if p_sign > 0 else n_via_x
                    inner_via_y = p_via_y if p_sign > 0 else n_via_y
                    inner_is_p = (p_sign > 0)

                if polarity_swap_needed:
                    inner_is_p = not inner_is_p
                    if inner_is_p:
                        inner_via_x, inner_via_y = p_via_x, p_via_y
                    else:
                        inner_via_x, inner_via_y = n_via_x, n_via_y

                if inner_is_p:
                    outer_via_x, outer_via_y = n_via_x, n_via_y
                else:
                    outer_via_x, outer_via_y = p_via_x, p_via_y
                inner_to_outer_x = outer_via_x - inner_via_x
                inner_to_outer_y = outer_via_y - inner_via_y

                perp_dot = inner_to_outer_x * in_perp_x + inner_to_outer_y * in_perp_y
                perp_sign = 1 if perp_dot >= 0 else -1

                if inner_is_p:
                    n_old_x = inner_via_x + in_perp_x * perp_sign * track_via_clearance
                    n_old_y = inner_via_y + in_perp_y * perp_sign * track_via_clearance
                    p_old_x = n_old_x - in_perp_x * perp_sign * diff_pair_spacing
                    p_old_y = n_old_y - in_perp_y * perp_sign * diff_pair_spacing
                else:
                    p_old_x = inner_via_x + in_perp_x * perp_sign * track_via_clearance
                    p_old_y = inner_via_y + in_perp_y * perp_sign * track_via_clearance
                    n_old_x = p_old_x - in_perp_x * perp_sign * diff_pair_spacing
                    n_old_y = p_old_y - in_perp_y * perp_sign * diff_pair_spacing

                out_perp_dot = inner_to_outer_x * out_perp_x + inner_to_outer_y * out_perp_y
                out_perp_sign = 1 if out_perp_dot >= 0 else -1

                if inner_is_p:
                    n_exit_x = inner_via_x + out_perp_x * out_perp_sign * track_via_clearance
                    n_exit_y = inner_via_y + out_perp_y * out_perp_sign * track_via_clearance
                    p_exit_x = n_exit_x - out_perp_x * out_perp_sign * diff_pair_spacing
                    p_exit_y = n_exit_y - out_perp_y * out_perp_sign * diff_pair_spacing
                else:
                    p_exit_x = inner_via_x + out_perp_x * out_perp_sign * track_via_clearance
                    p_exit_y = inner_via_y + out_perp_y * out_perp_sign * track_via_clearance
                    n_exit_x = p_exit_x - out_perp_x * out_perp_sign * diff_pair_spacing
                    n_exit_y = p_exit_y - out_perp_y * out_perp_sign * diff_pair_spacing
            else:
                p_old_x = cx + perp_x * p_sign * spacing_mm
                p_old_y = cy + perp_y * p_sign * spacing_mm
                n_old_x = cx + perp_x * n_sign * spacing_mm
                n_old_y = cy + perp_y * n_sign * spacing_mm
                p_exit_x, p_exit_y = p_old_x, p_old_y
                n_exit_x, n_exit_y = n_old_x, n_old_y

            # Update position at index i to approach position
            p_float_path[i] = (p_old_x, p_old_y, layer1)
            n_float_path[i] = (n_old_x, n_old_y, layer1)

            # Insert via position after old position
            p_float_path.insert(i + 1, (p_via_x, p_via_y, layer1))
            n_float_path.insert(i + 1, (n_via_x, n_via_y, layer1))

            # Update what was i+1 (now i+2) to via position on layer2
            p_float_path[i + 2] = (p_via_x, p_via_y, layer2)
            n_float_path[i + 2] = (n_via_x, n_via_y, layer2)

            # Insert exit position after via on layer2
            p_float_path.insert(i + 3, (p_exit_x, p_exit_y, layer2))
            n_float_path.insert(i + 3, (n_exit_x, n_exit_y, layer2))

            # Handle polarity swap detour if needed
            if polarity_swap_needed and in_dir_x is not None and out_dir_x is not None:
                p_float_path, n_float_path = _handle_polarity_swap_detour(
                    p_float_path, n_float_path, i, layer2, coord, config,
                    p_via_x, p_via_y, n_via_x, n_via_y,
                    out_dir_x, out_dir_y, track_via_clearance, diff_pair_spacing
                )

            break  # Only process first layer change

    return p_float_path, n_float_path


def _handle_polarity_swap_detour(p_float_path, n_float_path, i, layer2, coord, config,
                                  p_via_x, p_via_y, n_via_x, n_via_y,
                                  out_dir_x, out_dir_y, track_via_clearance, diff_pair_spacing):
    """Handle the circular arc detour for polarity swap around via."""
    arc_radius = 1.25 * track_via_clearance
    num_arc_segments = 20

    # Get P continuation info
    p_cont_x, p_cont_y, p_cont_layer = p_float_path[i + 4]
    n_cont_x, n_cont_y, _ = n_float_path[i + 4]

    # N via is the one we need to go around
    n_via_pos_x, n_via_pos_y = n_via_x, n_via_y

    # Via axis
    to_n_x = n_via_pos_x - p_via_x
    to_n_y = n_via_pos_y - p_via_y
    via_axis_len = math.sqrt(to_n_x*to_n_x + to_n_y*to_n_y)
    if via_axis_len > 0.001:
        via_axis_x = to_n_x / via_axis_len
        via_axis_y = to_n_y / via_axis_len
    else:
        via_axis_x, via_axis_y = 1, 0

    # Perpendicular options
    perp1_x, perp1_y = via_axis_y, -via_axis_x
    perp2_x, perp2_y = -via_axis_y, via_axis_x

    # Choose perpendicular away from continuation
    to_cont_x = p_cont_x - p_via_x
    to_cont_y = p_cont_y - p_via_y
    dot1 = perp1_x * to_cont_x + perp1_y * to_cont_y
    if dot1 < 0:
        via_perp_x, via_perp_y = perp1_x, perp1_y
    else:
        via_perp_x, via_perp_y = perp2_x, perp2_y

    # Starting angle
    start_angle = math.atan2(p_via_y - n_via_pos_y, p_via_x - n_via_pos_x)

    # Rotation direction
    cross_sign = via_axis_x * via_perp_y - via_axis_y * via_perp_x
    if cross_sign > 0:
        angle_step = -2 * math.pi / num_arc_segments
    else:
        angle_step = 2 * math.pi / num_arc_segments

    # Arc start
    arc_start_x = n_via_pos_x + arc_radius * math.cos(start_angle)
    arc_start_y = n_via_pos_y + arc_radius * math.sin(start_angle)

    arc_points = [(arc_start_x, arc_start_y)]

    # N continuation direction
    n_exit_x, n_exit_y, _ = n_float_path[i + 3]
    n_cont_dx = n_cont_x - n_exit_x
    n_cont_dy = n_cont_y - n_exit_y
    n_cont_len = math.sqrt(n_cont_dx*n_cont_dx + n_cont_dy*n_cont_dy)
    if n_cont_len > 0.001:
        n_cont_dir_x = n_cont_dx / n_cont_len
        n_cont_dir_y = n_cont_dy / n_cont_len
    else:
        n_cont_dir_x, n_cont_dir_y = out_dir_x, out_dir_y

    # P continuation direction
    p_cont_dir_x, p_cont_dir_y = n_cont_dir_x, n_cont_dir_y

    for seg_idx in range(1, num_arc_segments + 1):
        angle = start_angle + seg_idx * angle_step
        arc_x = n_via_pos_x + arc_radius * math.cos(angle)
        arc_y = n_via_pos_y + arc_radius * math.sin(angle)

        last_x, last_y = arc_points[-1]

        # Check for P continuation line crossing
        to_last_x = last_x - p_cont_x
        to_last_y = last_y - p_cont_y
        to_curr_x = arc_x - p_cont_x
        to_curr_y = arc_y - p_cont_y
        dist_last = to_last_x * p_cont_dir_y - to_last_y * p_cont_dir_x
        dist_curr = to_curr_x * p_cont_dir_y - to_curr_y * p_cont_dir_x

        if dist_last * dist_curr < 0:
            t = dist_last / (dist_last - dist_curr)
            intersect_x = last_x + t * (arc_x - last_x)
            intersect_y = last_y + t * (arc_y - last_y)
            arc_points.append((intersect_x, intersect_y))
            break

        # Check diff pair spacing
        arc_to_nexit_x = arc_x - n_exit_x
        arc_to_nexit_y = arc_y - n_exit_y
        dot_to_n_cont = arc_to_nexit_x * n_cont_dir_x + arc_to_nexit_y * n_cont_dir_y

        if dot_to_n_cont > 0:
            dist_to_n_track = abs(arc_to_nexit_x * n_cont_dir_y - arc_to_nexit_y * n_cont_dir_x)
            if dist_to_n_track < diff_pair_spacing:
                break

        arc_points.append((arc_x, arc_y))

    # Update P path with arc
    p_float_path[i + 3] = (arc_points[0][0], arc_points[0][1], layer2)

    insert_idx = i + 4
    for j, (ax, ay) in enumerate(arc_points[1:]):
        p_float_path.insert(insert_idx + j, (ax, ay, layer2))

    return p_float_path, n_float_path
