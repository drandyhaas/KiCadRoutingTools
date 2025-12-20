"""
Differential pair routing functions.

Routes differential pairs (P and N nets) together using centerline + offset approach.
"""

import math
import random
from typing import List, Optional, Tuple, Dict

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord, DiffPair
from routing_utils import (
    find_connected_groups, find_stub_free_ends, get_stub_direction, get_net_endpoints
)
from obstacle_map import check_line_clearance
from stub_layer_switching import (
    find_layer_switch_for_diff_pair, apply_layer_switch_option, revert_layer_switch_option, LayerSwitchOption
)

# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap, GridRouter, PoseRouter
except ImportError:
    GridObstacleMap = None
    GridRouter = None
    PoseRouter = None


# Map direction (dx, dy) to theta_idx (0-7) for pose-based routing
# DIRECTIONS in Rust: E(1,0)=0, NE(1,-1)=1, N(0,-1)=2, NW(-1,-1)=3, W(-1,0)=4, SW(-1,1)=5, S(0,1)=6, SE(1,1)=7
DIRECTION_TO_THETA = {
    (1, 0): 0,    # East
    (1, -1): 1,   # NE
    (0, -1): 2,   # North
    (-1, -1): 3,  # NW
    (-1, 0): 4,   # West
    (-1, 1): 5,   # SW
    (0, 1): 6,    # South
    (1, 1): 7,    # SE
}


def direction_to_theta_idx(dx: float, dy: float) -> int:
    """Convert a direction vector (dx, dy) to a theta_idx (0-7) for pose-based routing."""
    if abs(dx) < 0.01 and abs(dy) < 0.01:
        return 0  # Default to East if no direction

    # Normalize
    length = math.sqrt(dx*dx + dy*dy)
    ndx = dx / length
    ndy = dy / length

    # Find the angle and map to nearest 45-degree direction
    angle = math.atan2(-ndy, ndx)  # Note: negate y because grid y increases downward
    # Convert to [0, 2*pi) range
    if angle < 0:
        angle += 2 * math.pi

    # Map angle to theta_idx (each sector is 45 degrees = pi/4)
    # Add pi/8 to center each sector around the direction
    theta_idx = int((angle + math.pi/8) / (math.pi/4)) % 8
    return theta_idx


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


def get_diff_pair_connector_regions(pcb_data: PCBData, diff_pair: DiffPair,
                                     config: GridRouteConfig) -> Optional[dict]:
    """
    Compute connector region parameters for a differential pair.

    Returns dict with:
        - src_center: (x, y) center between P and N source stubs
        - src_dir: (dx, dy) normalized direction from stubs toward route
        - src_setback: distance from stub center to route start
        - tgt_center: (x, y) center between P and N target stubs
        - tgt_dir: (dx, dy) normalized direction from stubs toward route
        - tgt_setback: distance from stub center to route start
        - spacing_mm: half-spacing between P and N tracks

    Returns None if endpoints cannot be determined.
    """
    p_net_id = diff_pair.p_net_id
    n_net_id = diff_pair.n_net_id

    # Find endpoints
    sources, targets, error = get_diff_pair_endpoints(pcb_data, p_net_id, n_net_id, config)
    if error or not sources or not targets:
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    src = sources[0]
    tgt = targets[0]

    # Get original stub positions (in mm)
    p_src_x, p_src_y = src[5], src[6]
    n_src_x, n_src_y = src[7], src[8]
    p_tgt_x, p_tgt_y = tgt[5], tgt[6]
    n_tgt_x, n_tgt_y = tgt[7], tgt[8]

    # Calculate spacing from config (track_width + diff_pair_gap is center-to-center)
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2

    # Get segments for P and N nets to find stub directions
    p_segments = [s for s in pcb_data.segments if s.net_id == p_net_id]
    n_segments = [s for s in pcb_data.segments if s.net_id == n_net_id]

    src_layer_name = layer_names[src[4]]
    tgt_layer_name = layer_names[tgt[4]]

    # Get stub directions at source and target
    p_src_dir = get_stub_direction(p_segments, p_src_x, p_src_y, src_layer_name)
    n_src_dir = get_stub_direction(n_segments, n_src_x, n_src_y, src_layer_name)
    p_tgt_dir = get_stub_direction(p_segments, p_tgt_x, p_tgt_y, tgt_layer_name)
    n_tgt_dir = get_stub_direction(n_segments, n_tgt_x, n_tgt_y, tgt_layer_name)

    # Average P and N directions at each end
    src_dir_x = (p_src_dir[0] + n_src_dir[0]) / 2
    src_dir_y = (p_src_dir[1] + n_src_dir[1]) / 2
    tgt_dir_x = (p_tgt_dir[0] + n_tgt_dir[0]) / 2
    tgt_dir_y = (p_tgt_dir[1] + n_tgt_dir[1]) / 2

    # Normalize
    src_dir_len = math.sqrt(src_dir_x*src_dir_x + src_dir_y*src_dir_y)
    tgt_dir_len = math.sqrt(tgt_dir_x*tgt_dir_x + tgt_dir_y*tgt_dir_y)
    if src_dir_len > 0:
        src_dir_x /= src_dir_len
        src_dir_y /= src_dir_len
    if tgt_dir_len > 0:
        tgt_dir_x /= tgt_dir_len
        tgt_dir_y /= tgt_dir_len

    # Calculate centerline midpoints
    center_src_x = (p_src_x + n_src_x) / 2
    center_src_y = (p_src_y + n_src_y) / 2
    center_tgt_x = (p_tgt_x + n_tgt_x) / 2
    center_tgt_y = (p_tgt_y + n_tgt_y) / 2

    # Calculate setback (default to 4x spacing = 2x total P-N distance)
    if config.diff_pair_centerline_setback is not None:
        setback = config.diff_pair_centerline_setback
    else:
        setback = spacing_mm * 4
    src_setback = setback
    tgt_setback = setback

    return {
        'src_center': (center_src_x, center_src_y),
        'src_dir': (src_dir_x, src_dir_y),
        'src_setback': src_setback,
        'tgt_center': (center_tgt_x, center_tgt_y),
        'tgt_dir': (tgt_dir_x, tgt_dir_y),
        'tgt_setback': tgt_setback,
        'spacing_mm': spacing_mm,
    }


def _try_route_direction(src, tgt, pcb_data, config, obstacles, base_obstacles,
                         coord, layer_names, spacing_mm, p_net_id, n_net_id,
                         max_iterations_override=None):
    """
    Attempt to route a diff pair in one direction.

    Returns (route_data, iterations) on success, or (None, iterations) on failure.
    route_data contains all intermediate values needed for result processing.
    """
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

    # Calculate via exclusion radius in grid units
    # When centerline places a via, P/N vias are offset by via_spacing perpendicular to path
    # via_spacing is larger than spacing_mm to ensure via-via clearance
    # We need to prevent centerline from returning near the via such that offset tracks would conflict
    track_via_clearance = (config.clearance + config.track_width / 2 + config.via_size / 2) * 1.15
    min_via_spacing = config.via_size + config.clearance  # Minimum via center-to-center distance
    min_via_spacing_for_track = track_via_clearance - spacing_mm
    via_spacing = max(spacing_mm, min_via_spacing / 2, min_via_spacing_for_track)
    # Exclusion calculation: if centerline is at position X, the N track is at X + spacing_mm
    # N via is at centerline_via + via_spacing
    # For N track to clear N via: (X + spacing_mm) must be track_via_clearance from (centerline_via + via_spacing)
    # So X must be (track_via_clearance + via_spacing - spacing_mm) away from centerline_via
    # Use larger radius to ensure escape path also keeps offset tracks clear of offset vias
    via_exclusion_mm = (track_via_clearance + via_spacing) * 2
    via_exclusion_radius = max(1, int(via_exclusion_mm / config.grid_step + 0.5))  # Round up

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

    # Calculate setback distance (default to 4x spacing = 2x total P-N distance)
    if config.diff_pair_centerline_setback is not None:
        setback = config.diff_pair_centerline_setback
    else:
        setback = spacing_mm * 4  # Default: 4x the P-N half-spacing = 2x total P-N distance

    print(f"  Source direction: ({src_dir_x:.2f}, {src_dir_y:.2f}), target direction: ({tgt_dir_x:.2f}, {tgt_dir_y:.2f})")

    # Use base_obstacles for connector clearance checks if available
    # (connectors are single tracks that don't need diff pair extra clearance)
    connector_obstacles = base_obstacles if base_obstacles is not None else obstacles

    # Find open positions for source and target centerline points
    allow_radius = 2

    def find_open_position(center_x, center_y, dir_x, dir_y, layer_idx, sb, label):
        """Find an open position at the given setback distance.

        Scans angles 0°, ±15°, ±30° from the stub direction, preferring straight first.
        Returns (gx, gy, actual_dir_x, actual_dir_y) or None if all angles blocked.
        """
        # Generate rotated directions: 0°, ±15°, ±30° (prefer straight first)
        angles_deg = [0, 15, -15, 30, -30]

        for angle_deg in angles_deg:
            angle_rad = math.radians(angle_deg)
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            # Rotate (dir_x, dir_y) by angle
            dx = dir_x * cos_a - dir_y * sin_a
            dy = dir_x * sin_a + dir_y * cos_a

            x = center_x + dx * sb
            y = center_y + dy * sb
            gx, gy = coord.to_grid(x, y)

            # Check track blocking at setback position
            if not obstacles.is_blocked(gx, gy, layer_idx):
                # Also check the connector path from stub center to setback position
                if check_line_clearance(connector_obstacles, center_x, center_y, x, y, layer_idx, config):
                    if angle_deg != 0:
                        print(f"  {label.capitalize()} setback: using {angle_deg:+d}° offset")
                    return gx, gy, dx, dy

        print(f"  Error: {label} - no valid position at setback={sb:.2f}mm (all angles blocked)")
        return None

    def collect_setback_blocked_cells(center_x, center_y, dir_x, dir_y, layer_idx, sb):
        """Collect blocked cells at setback positions for rip-up analysis.
        Called only after find_open_position returns None.
        """
        angles_deg = [0, 15, -15, 30, -30]
        blocked = []
        step = config.grid_step / 2

        for angle_deg in angles_deg:
            angle_rad = math.radians(angle_deg)
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            dx = dir_x * cos_a - dir_y * sin_a
            dy = dir_x * sin_a + dir_y * cos_a

            x = center_x + dx * sb
            y = center_y + dy * sb
            gx, gy = coord.to_grid(x, y)

            # Collect blocked setback position
            if obstacles.is_blocked(gx, gy, layer_idx):
                if (gx, gy, layer_idx) not in blocked:
                    blocked.append((gx, gy, layer_idx))

            # Collect blocked cells along connector path
            length = math.sqrt((x - center_x)**2 + (y - center_y)**2)
            if length > 0:
                path_dx = (x - center_x) / length
                path_dy = (y - center_y) / length
                dist = 0.0
                while dist <= length:
                    px = center_x + path_dx * dist
                    py = center_y + path_dy * dist
                    pgx, pgy = coord.to_grid(px, py)
                    if connector_obstacles.is_blocked(pgx, pgy, layer_idx):
                        if (pgx, pgy, layer_idx) not in blocked:
                            blocked.append((pgx, pgy, layer_idx))
                    dist += step

        return blocked

    src_result = find_open_position(center_src_x, center_src_y, src_dir_x, src_dir_y, src_layer, setback, "source")
    if src_result is None:
        blocked = collect_setback_blocked_cells(center_src_x, center_src_y, src_dir_x, src_dir_y, src_layer, setback)
        return None, 0, blocked

    tgt_result = find_open_position(center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y, tgt_layer, setback, "target")
    if tgt_result is None:
        blocked = collect_setback_blocked_cells(center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y, tgt_layer, setback)
        return None, 0, blocked

    src_gx, src_gy, src_actual_dir_x, src_actual_dir_y = src_result
    tgt_gx, tgt_gy, tgt_actual_dir_x, tgt_actual_dir_y = tgt_result

    print(f"  Centerline setback: {setback:.2f}mm")

    # Note: Connector region via blocking is done upfront in route.py for ALL diff pairs
    # before any routing starts. This prevents one pair's vias from interfering with
    # another pair's connector segments (vias span all layers).

    # Add allowed cells around source and target
    for dx in range(-allow_radius, allow_radius + 1):
        for dy in range(-allow_radius, allow_radius + 1):
            obstacles.add_allowed_cell(src_gx + dx, src_gy + dy)
            obstacles.add_allowed_cell(tgt_gx + dx, tgt_gy + dy)
    obstacles.add_source_target_cell(src_gx, src_gy, src_layer)
    obstacles.add_source_target_cell(tgt_gx, tgt_gy, tgt_layer)

    # Convert actual setback directions to theta_idx for pose-based routing
    # (these may be rotated from stub directions if an angled setback was used)
    src_theta_idx = direction_to_theta_idx(src_actual_dir_x, src_actual_dir_y)
    # Target direction is the direction we arrive FROM, so negate it
    tgt_theta_idx = direction_to_theta_idx(-tgt_actual_dir_x, -tgt_actual_dir_y)

    print(f"  Pose routing: src_theta={src_theta_idx} ({src_actual_dir_x:.2f},{src_actual_dir_y:.2f}), tgt_theta={tgt_theta_idx} (arriving from {-tgt_actual_dir_x:.2f},{-tgt_actual_dir_y:.2f})")

    # Calculate turning radius in grid units
    min_radius_grid = config.min_turning_radius / config.grid_step

    # Calculate turn cost: arc length for 45° turn = r * π/4, scaled by 1000
    turn_cost = int(min_radius_grid * math.pi / 4 * 1000)

    # Create pose-based router for centerline
    # Double via cost since diff pairs place two vias per layer change
    # via_proximity_cost is a multiplier on stub proximity cost for vias (0 = block vias near stubs)
    pose_router = PoseRouter(
        via_cost=config.via_cost * 1000 * 2,
        h_weight=config.heuristic_weight,
        turn_cost=turn_cost,
        min_radius_grid=min_radius_grid,
        via_proximity_cost=int(config.via_proximity_cost)
    )

    # Route using pose-based A* with Dubins heuristic
    # Use 8x iterations for diff pairs due to larger pose-based search space
    # Pass via spacing in grid units so router can check P/N via positions
    via_spacing_grid = max(1, int(via_spacing / config.grid_step + 0.5))
    max_iters = max_iterations_override if max_iterations_override is not None else config.max_iterations * 8
    pose_path, iterations, blocked_cells = pose_router.route_pose_with_frontier(
        obstacles,
        src_gx, src_gy, src_layer, src_theta_idx,
        tgt_gx, tgt_gy, tgt_layer, tgt_theta_idx,
        max_iters,
        diff_pair_via_spacing=via_spacing_grid
    )

    if pose_path is None:
        return None, iterations, blocked_cells

    # Return all data needed for result processing (empty blocked_cells on success)
    route_data = {
        'pose_path': pose_path,
        'p_src_x': p_src_x, 'p_src_y': p_src_y,
        'n_src_x': n_src_x, 'n_src_y': n_src_y,
        'p_tgt_x': p_tgt_x, 'p_tgt_y': p_tgt_y,
        'n_tgt_x': n_tgt_x, 'n_tgt_y': n_tgt_y,
        'src_dir_x': src_dir_x, 'src_dir_y': src_dir_y,
        'tgt_dir_x': tgt_dir_x, 'tgt_dir_y': tgt_dir_y,
        'center_src_x': center_src_x, 'center_src_y': center_src_y,
        'center_tgt_x': center_tgt_x, 'center_tgt_y': center_tgt_y,
        'via_spacing': via_spacing,
    }
    return route_data, iterations, []  # Empty blocked_cells on success


def route_diff_pair_with_obstacles(pcb_data: PCBData, diff_pair: DiffPair,
                                    config: GridRouteConfig,
                                    obstacles: GridObstacleMap,
                                    base_obstacles: GridObstacleMap = None,
                                    unrouted_stubs: List[Tuple[float, float]] = None,
                                    unrouted_pairs: List[Tuple[str, DiffPair]] = None) -> Optional[dict]:
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
    original_src = sources[0]
    original_tgt = targets[0]

    # Calculate spacing from config (track_width + diff_pair_gap is center-to-center)
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2
    print(f"  P-N spacing: {spacing_mm * 2:.3f}mm (offset={spacing_mm:.3f}mm from centerline)")

    # Check if layer switch can avoid vias (source and target on different layers)
    layer_switch_vias = []
    layer_switch_option = None
    src_layer_idx = original_src[4]
    tgt_layer_idx = original_tgt[4]
    if src_layer_idx != tgt_layer_idx:
        src_layer = layer_names[src_layer_idx]
        tgt_layer = layer_names[tgt_layer_idx]
        # Extract stub positions from endpoint tuples
        p_src_x, p_src_y = original_src[5], original_src[6]
        n_src_x, n_src_y = original_src[7], original_src[8]
        p_tgt_x, p_tgt_y = original_tgt[5], original_tgt[6]
        n_tgt_x, n_tgt_y = original_tgt[7], original_tgt[8]

        layer_switch_option = find_layer_switch_for_diff_pair(
            pcb_data, p_net_id, n_net_id,
            src_layer, tgt_layer,
            p_src_x, p_src_y, n_src_x, n_src_y,
            p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y,
            obstacles, config,
            unrouted_pairs=unrouted_pairs, debug=True
        )

        if layer_switch_option:
            # Apply the switch now - if routing fails, we'll revert it
            new_vias, new_layer = apply_layer_switch_option(pcb_data, layer_switch_option, config)
            layer_switch_vias = new_vias
            switch_desc = "source" if layer_switch_option.switch_source else "target"
            swap_desc = f" (swap with {layer_switch_option.swap_pair.base_name})" if layer_switch_option.swap_pair else ""
            print(f"    Layer switch: Applying {switch_desc} stubs to {layer_switch_option.new_layer}{swap_desc}")
            # Re-fetch endpoints since layer info may have changed
            sources, targets, error = get_diff_pair_endpoints(pcb_data, p_net_id, n_net_id, config)
            if error:
                print(f"  After layer switch: {error}")
                # Revert the layer switch since we can't proceed
                revert_layer_switch_option(pcb_data, layer_switch_option, config, layer_switch_vias)
                print(f"    Layer switch reverted due to endpoint error")
                return None
            original_src = sources[0]
            original_tgt = targets[0]

    # Determine direction order (same as single_ended_routing)
    if config.direction_order == "random":
        start_backwards = random.choice([True, False])
    elif config.direction_order in ("backwards", "backward"):
        start_backwards = True
    else:  # "forward" or default
        start_backwards = False

    # Set up first and second direction
    if start_backwards:
        first_src, first_tgt, first_label = original_tgt, original_src, "backward"
        second_src, second_tgt, second_label = original_src, original_tgt, "forward"
    else:
        first_src, first_tgt, first_label = original_src, original_tgt, "forward"
        second_src, second_tgt, second_label = original_tgt, original_src, "backward"

    # Quick probe phase: test both directions with limited iterations to detect if stuck
    probe_iterations = config.max_probe_iterations
    stuck_threshold = max(100, probe_iterations // 10)  # If < 10% of probe used, direction is likely stuck

    # Probe first direction
    route_data, first_probe_iters, first_blocked = _try_route_direction(
        first_src, first_tgt, pcb_data, config, obstacles, base_obstacles,
        coord, layer_names, spacing_mm, p_net_id, n_net_id,
        max_iterations_override=probe_iterations
    )

    if route_data is not None:
        # Found route in probe phase
        first_blocked_cells = []
        first_iterations = first_probe_iters
        second_blocked_cells = []
        second_iterations = 0
        total_iterations = first_probe_iters
        routing_backwards = (first_label == "backward")
    else:
        # Probe second direction
        route_data, second_probe_iters, second_blocked = _try_route_direction(
            second_src, second_tgt, pcb_data, config, obstacles, base_obstacles,
            coord, layer_names, spacing_mm, p_net_id, n_net_id,
            max_iterations_override=probe_iterations
        )

        if route_data is not None:
            # Found route in second probe
            first_blocked_cells = first_blocked
            first_iterations = first_probe_iters
            second_blocked_cells = []
            second_iterations = second_probe_iters
            total_iterations = first_probe_iters + second_probe_iters
            routing_backwards = (second_label == "backward")
        else:
            # Both probes failed - check if either direction is completely stuck
            first_stuck = first_probe_iters < stuck_threshold
            second_stuck = second_probe_iters < stuck_threshold

            if first_stuck and second_stuck:
                # Both directions are stuck early - fail fast
                print(f"  Both directions stuck early ({first_label}: {first_probe_iters}, {second_label}: {second_probe_iters} iterations)")
                # Revert layer switch if we applied one
                if layer_switch_option:
                    revert_layer_switch_option(pcb_data, layer_switch_option, config, layer_switch_vias)
                    print(f"    Layer switch reverted due to routing failure")
                return {
                    'failed': True,
                    'iterations': first_probe_iters + second_probe_iters,
                    'blocked_cells_forward': first_blocked if first_label == "forward" else second_blocked,
                    'blocked_cells_backward': second_blocked if first_label == "forward" else first_blocked,
                    'iterations_forward': first_probe_iters if first_label == "forward" else second_probe_iters,
                    'iterations_backward': second_probe_iters if first_label == "forward" else first_probe_iters,
                }

            # At least one direction made progress - do full search on the more promising one
            # Try the direction that used more probe iterations first (made more progress)
            if first_probe_iters >= second_probe_iters:
                promising_src, promising_tgt, promising_label = first_src, first_tgt, first_label
                fallback_src, fallback_tgt, fallback_label = second_src, second_tgt, second_label
                promising_probe_iters, promising_blocked = first_probe_iters, first_blocked
                fallback_probe_iters, fallback_blocked = second_probe_iters, second_blocked
            else:
                promising_src, promising_tgt, promising_label = second_src, second_tgt, second_label
                fallback_src, fallback_tgt, fallback_label = first_src, first_tgt, first_label
                promising_probe_iters, promising_blocked = second_probe_iters, second_blocked
                fallback_probe_iters, fallback_blocked = first_probe_iters, first_blocked

            print(f"  Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {promising_label} with full iterations...")

            # Full search on promising direction
            route_data, full_iters, blocked_cells = _try_route_direction(
                promising_src, promising_tgt, pcb_data, config, obstacles, base_obstacles,
                coord, layer_names, spacing_mm, p_net_id, n_net_id
            )
            total_iterations = first_probe_iters + second_probe_iters + full_iters

            if route_data is not None:
                if promising_label == first_label:
                    first_blocked_cells = []
                    first_iterations = promising_probe_iters + full_iters
                    second_blocked_cells = fallback_blocked
                    second_iterations = fallback_probe_iters
                else:
                    first_blocked_cells = fallback_blocked
                    first_iterations = fallback_probe_iters
                    second_blocked_cells = []
                    second_iterations = promising_probe_iters + full_iters
                routing_backwards = (promising_label == "backward")
            else:
                # Promising direction failed, try fallback if not stuck
                fallback_stuck = fallback_probe_iters < stuck_threshold
                if not fallback_stuck:
                    print(f"  No route found after {full_iters} iterations ({promising_label}), trying {fallback_label}...")
                    route_data, fallback_full_iters, fallback_full_blocked = _try_route_direction(
                        fallback_src, fallback_tgt, pcb_data, config, obstacles, base_obstacles,
                        coord, layer_names, spacing_mm, p_net_id, n_net_id
                    )
                    total_iterations += fallback_full_iters

                    if route_data is not None:
                        if fallback_label == first_label:
                            first_blocked_cells = []
                            first_iterations = fallback_probe_iters + fallback_full_iters
                            second_blocked_cells = blocked_cells
                            second_iterations = promising_probe_iters + full_iters
                        else:
                            first_blocked_cells = blocked_cells
                            first_iterations = promising_probe_iters + full_iters
                            second_blocked_cells = []
                            second_iterations = fallback_probe_iters + fallback_full_iters
                        routing_backwards = (fallback_label == "backward")
                    else:
                        # Both full searches failed
                        if promising_label == first_label:
                            first_blocked_cells = blocked_cells
                            first_iterations = promising_probe_iters + full_iters
                            second_blocked_cells = fallback_full_blocked
                            second_iterations = fallback_probe_iters + fallback_full_iters
                        else:
                            first_blocked_cells = fallback_full_blocked
                            first_iterations = fallback_probe_iters + fallback_full_iters
                            second_blocked_cells = blocked_cells
                            second_iterations = promising_probe_iters + full_iters
                else:
                    # Fallback was stuck, use its probe results
                    print(f"  {fallback_label} direction stuck ({fallback_probe_iters} iterations), skipping full search")
                    if promising_label == first_label:
                        first_blocked_cells = blocked_cells
                        first_iterations = promising_probe_iters + full_iters
                        second_blocked_cells = fallback_blocked
                        second_iterations = fallback_probe_iters
                    else:
                        first_blocked_cells = fallback_blocked
                        first_iterations = fallback_probe_iters
                        second_blocked_cells = blocked_cells
                        second_iterations = promising_probe_iters + full_iters

    if route_data is None:
        print(f"  No route found after {total_iterations} iterations (both directions)")
        # Revert layer switch if we applied one
        if layer_switch_option:
            revert_layer_switch_option(pcb_data, layer_switch_option, config, layer_switch_vias)
            print(f"    Layer switch reverted due to routing failure")
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': first_blocked_cells if first_label == "forward" else second_blocked_cells,
            'blocked_cells_backward': second_blocked_cells if first_label == "forward" else first_blocked_cells,
            'iterations_forward': first_iterations if first_label == "forward" else second_iterations,
            'iterations_backward': second_iterations if first_label == "forward" else first_iterations,
        }

    # Extract route data
    pose_path = route_data['pose_path']
    p_src_x, p_src_y = route_data['p_src_x'], route_data['p_src_y']
    n_src_x, n_src_y = route_data['n_src_x'], route_data['n_src_y']
    p_tgt_x, p_tgt_y = route_data['p_tgt_x'], route_data['p_tgt_y']
    n_tgt_x, n_tgt_y = route_data['n_tgt_x'], route_data['n_tgt_y']
    src_dir_x, src_dir_y = route_data['src_dir_x'], route_data['src_dir_y']
    tgt_dir_x, tgt_dir_y = route_data['tgt_dir_x'], route_data['tgt_dir_y']
    center_src_x, center_src_y = route_data['center_src_x'], route_data['center_src_y']
    center_tgt_x, center_tgt_y = route_data['center_tgt_x'], route_data['center_tgt_y']
    via_spacing = route_data['via_spacing']

    # Convert pose path (gx, gy, theta_idx, layer) to grid path (gx, gy, layer)
    # Filter out in-place turns (same position, different theta)
    path = []
    for i, (gx, gy, theta_idx, layer) in enumerate(pose_path):
        # Skip if same position as previous (in-place turn)
        if path and path[-1][0] == gx and path[-1][1] == gy and path[-1][2] == layer:
            continue
        path.append((gx, gy, layer))

    # Simplify path by removing collinear points
    simplified_path = simplify_path(path)
    print(f"Route found in {total_iterations} iterations, path: {len(pose_path)} poses -> {len(path)} points -> {len(simplified_path)} simplified")

    # Store paths for debug output (converted to float coordinates)
    raw_astar_path = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], layer)
                      for gx, gy, layer in path]
    simplified_path_float = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], layer)
                             for gx, gy, layer in simplified_path]

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

    # Handle polarity fix - mark for pad/stub net swap in output file
    # We DON'T swap coordinates here - instead we'll swap target pad and stub nets
    # This preserves the P→P, N→N geometry and fixes polarity at the schematic level
    polarity_fixed = False
    if polarity_swap_needed and config.fix_polarity:
        print(f"  Polarity swap needed - will swap target pad and stub nets in output")
        polarity_fixed = True

    # Always use source polarity
    p_sign = src_p_sign
    print(f"  Polarity: src_p_sign={src_p_sign}, tgt_p_sign={tgt_p_sign}, swap_needed={polarity_swap_needed}, has_vias={has_layer_change}")

    n_sign = -p_sign

    # Create P and N paths using perpendicular offsets from centerline
    # Use stub directions at endpoints so perpendicular offsets align with stub P-N axis
    start_stub_dir = (src_dir_x, src_dir_y)
    end_stub_dir = (-tgt_dir_x, -tgt_dir_y)  # Negate because we arrive at target stubs
    p_float_path = create_parallel_path_float(simplified_path, coord, sign=p_sign, spacing_mm=spacing_mm,
                                               start_dir=start_stub_dir, end_dir=end_stub_dir)
    n_float_path = create_parallel_path_float(simplified_path, coord, sign=n_sign, spacing_mm=spacing_mm,
                                               start_dir=start_stub_dir, end_dir=end_stub_dir)

    # Process via positions
    p_float_path, n_float_path = _process_via_positions(
        simplified_path, p_float_path, n_float_path, coord, config,
        p_sign, n_sign, spacing_mm
    )

    # Convert floating-point paths to segments and vias
    new_segments = []
    new_vias = []
    debug_connector_lines = []  # For User.3 layer (connectors)

    def float_path_to_geometry(float_path, net_id, original_start, original_end):
        """Convert floating-point path (x, y, layer) to segments and vias."""
        segs = []
        vias = []
        connector_lines = []  # Debug lines for connectors
        num_segments = len(float_path) - 1

        # Add connecting segment from original start if needed
        if original_start and len(float_path) > 0:
            first_x, first_y, first_layer = float_path[0]
            orig_x, orig_y = original_start
            if abs(orig_x - first_x) > 0.001 or abs(orig_y - first_y) > 0.001:
                segs.append(Segment(
                    start_x=orig_x, start_y=orig_y,
                    end_x=first_x, end_y=first_y,
                    width=config.track_width,
                    layer=layer_names[first_layer],
                    net_id=net_id
                ))
                # Collect debug line for connector
                if config.debug_lines:
                    connector_lines.append(((orig_x, orig_y), (first_x, first_y)))

        # Convert path segments
        for i in range(num_segments):
            x1, y1, layer1 = float_path[i]
            x2, y2, layer2 = float_path[i + 1]

            if layer1 != layer2:
                # Layer change - add via
                vias.append(Via(
                    x=x1, y=y1,
                    size=config.via_size,
                    drill=config.via_drill,
                    layers=[layer_names[layer1], layer_names[layer2]],
                    net_id=net_id
                ))
            elif abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
                # Always use actual layer for segment
                segs.append(Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.track_width,
                    layer=layer_names[layer1],
                    net_id=net_id
                ))

        # Add connecting segment to original end if needed
        if original_end and len(float_path) > 0:
            last_x, last_y, last_layer = float_path[-1]
            orig_x, orig_y = original_end
            if abs(orig_x - last_x) > 0.001 or abs(orig_y - last_y) > 0.001:
                segs.append(Segment(
                    start_x=last_x, start_y=last_y,
                    end_x=orig_x, end_y=orig_y,
                    width=config.track_width,
                    layer=layer_names[last_layer],
                    net_id=net_id
                ))
                # Collect debug line for connector
                if config.debug_lines:
                    connector_lines.append(((last_x, last_y), (orig_x, orig_y)))

        return segs, vias, connector_lines

    # Get original coordinates for P and N nets
    p_start = (p_src_x, p_src_y)
    n_start = (n_src_x, n_src_y)
    # Swap target coordinates if polarity was fixed - routes go to opposite stub positions
    # After output file swap, the stub at n_tgt will have P net, stub at p_tgt will have N net
    if polarity_fixed:
        p_end = (n_tgt_x, n_tgt_y)  # P route goes to original N position (will be P after swap)
        n_end = (p_tgt_x, p_tgt_y)  # N route goes to original P position (will be N after swap)
    else:
        p_end = (p_tgt_x, p_tgt_y)
        n_end = (n_tgt_x, n_tgt_y)

    # Convert P path
    p_segs, p_vias, p_conn_lines = float_path_to_geometry(p_float_path, p_net_id, p_start, p_end)
    new_segments.extend(p_segs)
    new_vias.extend(p_vias)
    debug_connector_lines.extend(p_conn_lines)

    # Convert N path
    n_segs, n_vias, n_conn_lines = float_path_to_geometry(n_float_path, n_net_id, n_start, n_end)
    new_segments.extend(n_segs)
    new_vias.extend(n_vias)
    debug_connector_lines.extend(n_conn_lines)

    # Add any vias from layer switching (pad vias when switching from F.Cu)
    new_vias.extend(layer_switch_vias)

    # Convert float paths back to grid format for return value
    p_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in p_float_path]
    n_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in n_float_path]

    # Build stub direction arrows for debug visualization (User.4)
    # Each arrow is 1mm shaft + arrowhead pointing in stub direction from midpoint
    debug_stub_arrows = []
    if config.debug_lines:
        arrow_length = 1.0  # mm
        head_length = 0.2  # mm
        head_angle = 0.5  # radians (~30 degrees)

        for mid_x, mid_y, dir_x, dir_y in [
            (center_src_x, center_src_y, src_dir_x, src_dir_y),
            (center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y)
        ]:
            # Arrow shaft
            tip_x = mid_x + dir_x * arrow_length
            tip_y = mid_y + dir_y * arrow_length
            debug_stub_arrows.append(((mid_x, mid_y), (tip_x, tip_y)))

            # Arrowhead lines (two lines from tip, angled back)
            cos_a = math.cos(head_angle)
            sin_a = math.sin(head_angle)
            # Rotate direction by +/- head_angle and reverse
            for sign in [1, -1]:
                # Rotate (dir_x, dir_y) by sign * head_angle
                rot_x = dir_x * cos_a - sign * dir_y * sin_a
                rot_y = sign * dir_x * sin_a + dir_y * cos_a
                # Point back from tip
                head_end_x = tip_x - rot_x * head_length
                head_end_y = tip_y - rot_y * head_length
                debug_stub_arrows.append(((tip_x, tip_y), (head_end_x, head_end_y)))

    # Build result with polarity fix info
    result = {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(simplified_path),
        'p_path': p_path,
        'n_path': n_path,
        'raw_astar_path': raw_astar_path,
        'simplified_path': simplified_path_float,
        'debug_connector_lines': debug_connector_lines,  # For User.3 layer
        'debug_stub_arrows': debug_stub_arrows,  # For User.4 layer
    }

    # If polarity was fixed, include info about which target pads need net swaps
    if polarity_fixed:
        result['polarity_fixed'] = True
        # The pads to swap are at the ROUTING target end
        # When routing forward: routing target = original target (targets[0])
        # When routing backward: routing target = original source (sources[0])
        if routing_backwards:
            # Routing target is original source
            routing_tgt_end = sources[0]
        else:
            # Routing target is original target
            routing_tgt_end = targets[0]
        orig_p_tgt = (routing_tgt_end[5], routing_tgt_end[6])
        orig_n_tgt = (routing_tgt_end[7], routing_tgt_end[8])
        result['swap_target_pads'] = {
            'p_pos': orig_p_tgt,  # P target stub position (routing target end)
            'n_pos': orig_n_tgt,  # N target stub position (routing target end)
            'p_net_id': p_net_id,  # P net ID to find target pad
            'n_net_id': n_net_id,  # N net ID to find target pad
        }

    return result


def _process_via_positions(simplified_path, p_float_path, n_float_path, coord, config,
                           p_sign, n_sign, spacing_mm):
    """
    Process via positions at layer changes to be perpendicular to centerline direction.

    Adds approach and exit tracks to keep main P/N tracks parallel while routing to vias.
    Handles multiple layer changes by processing in reverse order.
    """
    min_via_spacing = config.via_size + config.clearance  # Minimum center-to-center distance
    track_via_clearance = (config.clearance + config.track_width / 2 + config.via_size / 2) * 1.15

    if not p_float_path or not n_float_path or len(simplified_path) < 2:
        return p_float_path, n_float_path

    # Find all layer change indices first
    layer_change_indices = []
    for i in range(len(simplified_path) - 1):
        if simplified_path[i][2] != simplified_path[i + 1][2]:
            layer_change_indices.append(i)

    # Process in reverse order so insertions don't affect earlier indices
    for i in reversed(layer_change_indices):
        gx1, gy1, layer1 = simplified_path[i]
        gx2, gy2, layer2 = simplified_path[i + 1]

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
            in_dir_x, in_dir_y = 1.0, 0.0

        # Get outgoing direction (from via to next point after layer change)
        if i + 2 < len(simplified_path):
            next_gx, next_gy, _ = simplified_path[i + 2]
            next_x, next_y = coord.to_float(next_gx, next_gy)
            out_dir_x, out_dir_y = next_x - cx, next_y - cy
            out_len = math.sqrt(out_dir_x**2 + out_dir_y**2)
            if out_len > 0.001:
                out_dir_x, out_dir_y = out_dir_x / out_len, out_dir_y / out_len
            else:
                out_dir_x, out_dir_y = in_dir_x, in_dir_y
        else:
            out_dir_x, out_dir_y = in_dir_x, in_dir_y

        # Calculate perpendicular direction for via-via axis (average of in/out perpendiculars)
        perp_x = (-in_dir_y + -out_dir_y) / 2
        perp_y = (in_dir_x + out_dir_x) / 2
        perp_len = math.sqrt(perp_x**2 + perp_y**2)
        if perp_len > 0.001:
            perp_x, perp_y = perp_x / perp_len, perp_y / perp_len
        else:
            perp_x, perp_y = -in_dir_y, in_dir_x

        # Use larger spacing for vias if needed
        min_via_spacing_for_track = track_via_clearance - spacing_mm
        via_spacing = max(spacing_mm, min_via_spacing / 2, min_via_spacing_for_track)

        # Calculate P and N via positions perpendicular to centerline
        p_via_x = cx + perp_x * p_sign * via_spacing
        p_via_y = cy + perp_y * p_sign * via_spacing
        n_via_x = cx + perp_x * n_sign * via_spacing
        n_via_y = cy + perp_y * n_sign * via_spacing

        # Calculate approach and exit positions to keep main tracks parallel
        diff_pair_spacing = spacing_mm * 2
        in_perp_x, in_perp_y = -in_dir_y, in_dir_x
        out_perp_x, out_perp_y = -out_dir_y, out_dir_x

        # Determine which via is on the "inner" side of a turn (if any)
        cross = in_dir_x * out_dir_y - in_dir_y * out_dir_x
        if cross < 0:
            inner_is_p = (p_sign < 0)
        else:
            inner_is_p = (p_sign > 0)

        if inner_is_p:
            inner_via_x, inner_via_y = p_via_x, p_via_y
            outer_via_x, outer_via_y = n_via_x, n_via_y
        else:
            inner_via_x, inner_via_y = n_via_x, n_via_y
            outer_via_x, outer_via_y = p_via_x, p_via_y

        inner_to_outer_x = outer_via_x - inner_via_x
        inner_to_outer_y = outer_via_y - inner_via_y

        # Calculate approach positions (before via, on layer1)
        perp_dot = inner_to_outer_x * in_perp_x + inner_to_outer_y * in_perp_y
        perp_sign = 1 if perp_dot >= 0 else -1

        if inner_is_p:
            n_approach_x = inner_via_x + in_perp_x * perp_sign * track_via_clearance
            n_approach_y = inner_via_y + in_perp_y * perp_sign * track_via_clearance
            p_approach_x = n_approach_x - in_perp_x * perp_sign * diff_pair_spacing
            p_approach_y = n_approach_y - in_perp_y * perp_sign * diff_pair_spacing
        else:
            p_approach_x = inner_via_x + in_perp_x * perp_sign * track_via_clearance
            p_approach_y = inner_via_y + in_perp_y * perp_sign * track_via_clearance
            n_approach_x = p_approach_x - in_perp_x * perp_sign * diff_pair_spacing
            n_approach_y = p_approach_y - in_perp_y * perp_sign * diff_pair_spacing

        # Calculate exit positions (after via, on layer2)
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

        # Update position at index i to approach position
        p_float_path[i] = (p_approach_x, p_approach_y, layer1)
        n_float_path[i] = (n_approach_x, n_approach_y, layer1)

        # Insert via position after approach position
        p_float_path.insert(i + 1, (p_via_x, p_via_y, layer1))
        n_float_path.insert(i + 1, (n_via_x, n_via_y, layer1))

        # Update what was i+1 (now i+2) to via position on layer2
        p_float_path[i + 2] = (p_via_x, p_via_y, layer2)
        n_float_path[i + 2] = (n_via_x, n_via_y, layer2)

        # Insert exit position after via on layer2
        p_float_path.insert(i + 3, (p_exit_x, p_exit_y, layer2))
        n_float_path.insert(i + 3, (n_exit_x, n_exit_y, layer2))

    return p_float_path, n_float_path
