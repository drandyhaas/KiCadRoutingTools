"""
Differential pair routing functions.

Routes differential pairs (P and N nets) together using centerline + offset approach.
"""

import math
import random
from typing import List, Optional, Tuple, Dict

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from routing_utils import (
    find_connected_groups, find_stub_free_ends, get_stub_direction, get_net_endpoints,
    get_stub_segments, segment_length
)
from obstacle_map import check_line_clearance
# Note: Layer switching is now done upfront in route.py, not during routing

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


def _find_open_positions(center_x, center_y, dir_x, dir_y, layer_idx, setback,
                         label, layer_names, spacing_mm, config, obstacles,
                         connector_obstacles, coord, neighbor_stubs):
    """Find open position for setback, preferring 0° but angling away from nearby stubs if needed.

    Only angles away from 0° if the nearest unrouted stub is too close (within
    required diff pair clearance). Returns list with best candidate first.

    Each candidate is (gx, gy, dx, dy, angle_deg, dist_to_nearest_stub).
    """
    current_layer = layer_names[layer_idx]

    # Required clearance from centerline to neighboring stub
    # P/N tracks are at ±spacing_mm from centerline, need config.clearance to other stub
    # Add factor of 2 for extra margin to ensure future routes have room
    required_clearance = 2 * (spacing_mm + config.clearance)

    # Generate angles: 0, then ±max/4, ±max/2, ±3*max/4, ±max
    max_angle = config.max_setback_angle
    angles_deg = [0,
                  max_angle / 4, -max_angle / 4,
                  max_angle / 2, -max_angle / 2,
                  3 * max_angle / 4, -3 * max_angle / 4,
                  max_angle, -max_angle]

    def check_angle_valid(angle_deg):
        """Check if angle is valid (not blocked). Returns (gx, gy, dx, dy, x, y) or None."""
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        dx = dir_x * cos_a - dir_y * sin_a
        dy = dir_x * sin_a + dir_y * cos_a

        x = center_x + dx * setback
        y = center_y + dy * setback
        gx, gy = coord.to_grid(x, y)

        if obstacles.is_blocked(gx, gy, layer_idx):
            return None
        if not check_line_clearance(connector_obstacles, center_x, center_y, x, y, layer_idx, config):
            return None
        # Probe ahead
        probe_dist = config.grid_step * 3
        probe_x = x + dx * probe_dist
        probe_y = y + dy * probe_dist
        probe_gx, probe_gy = coord.to_grid(probe_x, probe_y)
        if obstacles.is_blocked(probe_gx, probe_gy, layer_idx):
            return None
        return (gx, gy, dx, dy, x, y)

    def find_nearest_stub(x, y):
        """Find nearest same-layer stub from position (x, y). Returns (dist, stub_x, stub_y) or (inf, None, None)."""
        min_dist = float('inf')
        closest = (None, None)
        if neighbor_stubs:
            for stub_x, stub_y, stub_layer in neighbor_stubs:
                if stub_layer != current_layer:
                    continue
                dist = math.sqrt((x - stub_x)**2 + (y - stub_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest = (stub_x, stub_y)
        return min_dist, closest[0], closest[1]

    # Check 0° first
    zero_result = check_angle_valid(0)
    if zero_result:
        gx, gy, dx, dy, x, y = zero_result
        dist, stub_x, stub_y = find_nearest_stub(x, y)

        # Collect all valid angles as fallbacks (sorted by angle magnitude)
        other_candidates = []
        for angle_deg in sorted(angles_deg, key=abs):
            if angle_deg == 0:
                continue
            result = check_angle_valid(angle_deg)
            if result:
                ax, ay, adx, ady, ax_pos, ay_pos = result
                a_dist, _, _ = find_nearest_stub(ax_pos, ay_pos)
                other_candidates.append((ax, ay, adx, ady, angle_deg, a_dist))

        if dist >= required_clearance:
            # 0° is valid and has enough clearance - use it as primary
            if config.verbose:
                if stub_x is not None:
                    print(f"      {label} 0.0°: OK (nearest stub at ({stub_x:.1f},{stub_y:.1f}) dist={dist:.2f}mm >= {required_clearance:.2f}mm)")
                else:
                    print(f"      {label} 0.0°: OK (no nearby stubs on {current_layer})")
            return [(gx, gy, dx, dy, 0.0, dist)] + other_candidates

        # 0° is valid but too close to stub - need to angle away
        if config.verbose:
            print(f"      {label} 0.0°: too close to stub at ({stub_x:.1f},{stub_y:.1f}) dist={dist:.2f}mm < {required_clearance:.2f}mm")

        # Determine which direction to angle based on stub position
        # Cross product: positive means stub is on left (+angle side)
        stub_vec_x = stub_x - center_x
        stub_vec_y = stub_y - center_y
        cross = dir_x * stub_vec_y - dir_y * stub_vec_x
        # Angle away from stub: if stub on left (cross > 0), go negative angle
        preferred_sign = -1 if cross > 0 else 1

        # Find the best angle that clears
        best_clearing_angle = None
        for cand in other_candidates:
            if cand[4] * preferred_sign > 0 and cand[5] >= required_clearance:
                best_clearing_angle = cand
                if config.verbose:
                    print(f"      {label} {cand[4]:+.1f}°: OK (cleared to {cand[5]:.2f}mm)")
                break

        if best_clearing_angle:
            # Put clearing angle first, then 0°, then others
            remaining = [c for c in other_candidates if c != best_clearing_angle]
            return [best_clearing_angle, (gx, gy, dx, dy, 0.0, dist)] + remaining
        else:
            # No angle clears - use 0° as primary anyway
            if config.verbose:
                print(f"      {label} using 0.0° (no angle clears {required_clearance:.2f}mm)")
            return [(gx, gy, dx, dy, 0.0, dist)] + other_candidates

    # 0° is blocked - find any valid angle, prefer smaller angles
    if config.verbose:
        print(f"      {label} 0.0°: blocked")

    valid_candidates = []
    for angle_deg in sorted(angles_deg, key=abs):
        if angle_deg == 0:
            continue
        result = check_angle_valid(angle_deg)
        if result:
            gx, gy, dx, dy, x, y = result
            dist, _, _ = find_nearest_stub(x, y)
            valid_candidates.append((gx, gy, dx, dy, angle_deg, dist))
            if config.verbose:
                print(f"      {label} {angle_deg:+.1f}°: OK (dist={dist:.2f}mm)")

    if not valid_candidates:
        print(f"  Error: {label} - no valid position at setback={setback:.2f}mm (all angles blocked)")
        return []

    # Sort by clearance (larger better), then by angle magnitude (smaller better)
    valid_candidates.sort(key=lambda c: (c[5], -abs(c[4])), reverse=True)
    return valid_candidates


def _collect_setback_blocked_cells(center_x, center_y, dir_x, dir_y, layer_idx, setback,
                                    config, obstacles, connector_obstacles, coord):
    """Collect blocked cells at setback positions for rip-up analysis.
    Called only after _find_open_positions returns empty list.
    """
    max_angle = config.max_setback_angle
    angles_deg = [0,
                  max_angle / 4, -max_angle / 4,
                  max_angle / 2, -max_angle / 2,
                  3 * max_angle / 4, -3 * max_angle / 4,
                  max_angle, -max_angle]
    blocked = []
    step = config.grid_step / 2

    for angle_deg in angles_deg:
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        dx = dir_x * cos_a - dir_y * sin_a
        dy = dir_x * sin_a + dir_y * cos_a

        x = center_x + dx * setback
        y = center_y + dy * setback
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

        # Also collect "probe ahead" blocked cells (3 grid cells ahead of setback)
        # This catches cases where setback and connector are clear but routing direction is blocked
        probe_dist = config.grid_step * 3
        probe_x = x + dx * probe_dist
        probe_y = y + dy * probe_dist
        probe_gx, probe_gy = coord.to_grid(probe_x, probe_y)
        if obstacles.is_blocked(probe_gx, probe_gy, layer_idx):
            if (probe_gx, probe_gy, layer_idx) not in blocked:
                blocked.append((probe_gx, probe_gy, layer_idx))

    return blocked


def _detect_polarity(simplified_path, coord,
                     p_src_x, p_src_y, n_src_x, n_src_y,
                     p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y,
                     config):
    """
    Detect which side of the centerline P should be on and whether polarity swap is needed.

    Args:
        simplified_path: List of (gx, gy, layer) representing simplified centerline
        coord: GridCoord for coordinate conversion
        p_src_x, p_src_y: P source position in mm
        n_src_x, n_src_y: N source position in mm
        p_tgt_x, p_tgt_y: P target position in mm
        n_tgt_x, n_tgt_y: N target position in mm
        config: Routing config (for fix_polarity flag)

    Returns:
        (p_sign, polarity_fixed, polarity_swap_needed, has_layer_change)
        - p_sign: +1 or -1, which perpendicular side P should be offset to
        - polarity_fixed: True if we'll swap target pads in output
        - polarity_swap_needed: True if source and target polarities differ
        - has_layer_change: True if path has vias (layer changes)
    """
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

    return p_sign, polarity_fixed, polarity_swap_needed, has_layer_change


def _calculate_parallel_extension(p_stub, n_stub, p_route, n_route, stub_dir, p_sign):
    """Calculate extensions for P and N tracks to make their connectors parallel.

    The key insight: for connectors to be parallel, the extending track must
    extend along the stub direction until its connector is parallel to the
    non-extended track's connector.

    Formula derivation:
    - V_p = R_p - S_p (direct connector for P)
    - V_n = R_n - S_n (direct connector for N)
    - For P extended by E: V_p' = V_p - E*D
    - For V_p' || V_n: (V_p - E*D) × V_n = 0
    - E = (V_p × V_n) / (D × V_n)

    Returns (p_extension, n_extension) where one will be positive, the other 0.
    """
    dx, dy = stub_dir

    # Direct connector vectors
    v_p = (p_route[0] - p_stub[0], p_route[1] - p_stub[1])
    v_n = (n_route[0] - n_stub[0], n_route[1] - n_stub[1])

    # Cross product of connector vectors: tells us if connectors are already parallel
    # and which direction the divergence is
    v_cross = v_p[0] * v_n[1] - v_p[1] * v_n[0]

    # If connectors are nearly parallel, no extension needed
    if abs(v_cross) < 0.0001:
        return 0.0, 0.0

    # Cross product D × V_n (for extending P)
    d_cross_vn = dx * v_n[1] - dy * v_n[0]
    # Cross product D × V_p (for extending N)
    d_cross_vp = dx * v_p[1] - dy * v_p[0]

    # Calculate extensions for both tracks
    # E_p = (V_p × V_n) / (D × V_n) makes V_p' parallel to V_n
    # E_n = (V_n × V_p) / (D × V_p) = -v_cross / d_cross_vp makes V_n' parallel to V_p
    p_ext = v_cross / d_cross_vn if abs(d_cross_vn) > 0.0001 else 0.0
    n_ext = -v_cross / d_cross_vp if abs(d_cross_vp) > 0.0001 else 0.0

    # Use whichever extension is positive (extending in stub direction)
    # If both are positive, prefer the one for the "outside" track
    if p_ext > 0.001 and n_ext > 0.001:
        # Both positive - use the outside track based on geometry
        is_p_outside = (v_cross > 0 and p_sign < 0) or (v_cross < 0 and p_sign > 0)
        if is_p_outside:
            return p_ext, 0.0
        else:
            return 0.0, n_ext
    elif p_ext > 0.001:
        return p_ext, 0.0
    elif n_ext > 0.001:
        return 0.0, n_ext

    return 0.0, 0.0


def _create_gnd_vias(simplified_path, coord, config, layer_names, spacing_mm, gnd_net_id, gnd_via_dirs):
    """Create GND vias at layer changes in the centerline path.

    Args:
        simplified_path: List of (gx, gy, layer_idx) grid points
        coord: GridCoord for conversions
        config: GridRouteConfig
        layer_names: List of layer names
        spacing_mm: P/N offset from centerline
        gnd_net_id: Net ID for GND vias
        gnd_via_dirs: List of directions (+1=ahead, -1=behind) from Rust router

    Returns:
        List of Via objects for GND connections
    """
    gnd_vias = []
    if gnd_net_id is None or len(simplified_path) < 2:
        return gnd_vias

    # Calculate GND via perpendicular offset: track edge + clearance + via radius
    gnd_via_perp_mm = spacing_mm + config.track_width/2 + config.clearance + config.via_size/2
    via_via_dist_mm = config.via_size + config.clearance

    # Track which layer change we're processing to get direction from gnd_via_dirs
    via_idx = 0

    # Find layer changes in centerline path
    for i in range(len(simplified_path) - 1):
        gx1, gy1, layer1 = simplified_path[i]
        gx2, gy2, layer2 = simplified_path[i + 1]

        if layer1 != layer2:
            # Layer change - calculate centerline position and direction
            cx, cy = coord.to_float(gx1, gy1)

            # Get heading direction (from via to next point)
            next_cx, next_cy = coord.to_float(gx2, gy2)
            dx = next_cx - cx
            dy = next_cy - cy
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0.001:
                dx /= length
                dy /= length
            else:
                # Fallback: use direction from previous point
                if i > 0:
                    prev_gx, prev_gy, _ = simplified_path[i - 1]
                    prev_cx, prev_cy = coord.to_float(prev_gx, prev_gy)
                    dx = cx - prev_cx
                    dy = cy - prev_cy
                    length = math.sqrt(dx*dx + dy*dy)
                    if length > 0.001:
                        dx /= length
                        dy /= length
                    else:
                        dx, dy = 1.0, 0.0
                else:
                    dx, dy = 1.0, 0.0

            # Perpendicular direction: rotate 90° -> (-dy, dx)
            perp_x = -dy
            perp_y = dx

            # Get GND via direction from Rust router (1=ahead, -1=behind)
            gnd_dir = gnd_via_dirs[via_idx] if via_idx < len(gnd_via_dirs) else 1
            via_idx += 1

            # GND via positions: perpendicular offset + along-heading offset
            gnd_p_x = cx + perp_x * gnd_via_perp_mm + dx * via_via_dist_mm * gnd_dir
            gnd_p_y = cy + perp_y * gnd_via_perp_mm + dy * via_via_dist_mm * gnd_dir
            gnd_n_x = cx - perp_x * gnd_via_perp_mm + dx * via_via_dist_mm * gnd_dir
            gnd_n_y = cy - perp_y * gnd_via_perp_mm + dy * via_via_dist_mm * gnd_dir

            # Create GND vias (free=True prevents KiCad auto-assigning net)
            gnd_vias.append(Via(
                x=gnd_p_x, y=gnd_p_y,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=gnd_net_id,
                free=True
            ))
            gnd_vias.append(Via(
                x=gnd_n_x, y=gnd_n_y,
                size=config.via_size,
                drill=config.via_drill,
                layers=[layer_names[layer1], layer_names[layer2]],
                net_id=gnd_net_id,
                free=True
            ))

    return gnd_vias


def _make_route_data(pose_path, p_src_x, p_src_y, n_src_x, n_src_y,
                     p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y,
                     src_dir_x, src_dir_y, tgt_dir_x, tgt_dir_y,
                     src_actual_dir, tgt_actual_dir,
                     center_src_x, center_src_y, center_tgt_x, center_tgt_y,
                     via_spacing, src_angle, tgt_angle, gnd_via_dirs=None):
    """Build route_data dictionary with all routing result info."""
    return {
        'pose_path': pose_path,
        'p_src_x': p_src_x, 'p_src_y': p_src_y,
        'n_src_x': n_src_x, 'n_src_y': n_src_y,
        'p_tgt_x': p_tgt_x, 'p_tgt_y': p_tgt_y,
        'n_tgt_x': n_tgt_x, 'n_tgt_y': n_tgt_y,
        'src_dir_x': src_dir_x, 'src_dir_y': src_dir_y,
        'tgt_dir_x': tgt_dir_x, 'tgt_dir_y': tgt_dir_y,
        'src_actual_dir_x': src_actual_dir[0], 'src_actual_dir_y': src_actual_dir[1],
        'tgt_actual_dir_x': tgt_actual_dir[0], 'tgt_actual_dir_y': tgt_actual_dir[1],
        'center_src_x': center_src_x, 'center_src_y': center_src_y,
        'center_tgt_x': center_tgt_x, 'center_tgt_y': center_tgt_y,
        'via_spacing': via_spacing,
        'best_src_angle': src_angle, 'best_tgt_angle': tgt_angle,
        'gnd_via_dirs': gnd_via_dirs if gnd_via_dirs is not None else [],
    }


def _generate_debug_arrows(center_src_x, center_src_y, src_dir_x, src_dir_y,
                           center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y):
    """Generate stub direction arrows for debug visualization (User.4 layer).

    Returns list of ((start_x, start_y), (end_x, end_y)) line segments.
    """
    arrows = []
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
        arrows.append(((mid_x, mid_y), (tip_x, tip_y)))

        # Arrowhead lines (two lines from tip, angled back)
        cos_a = math.cos(head_angle)
        sin_a = math.sin(head_angle)
        for sign in [1, -1]:
            rot_x = dir_x * cos_a - sign * dir_y * sin_a
            rot_y = sign * dir_x * sin_a + dir_y * cos_a
            head_end_x = tip_x - rot_x * head_length
            head_end_y = tip_y - rot_y * head_length
            arrows.append(((tip_x, tip_y), (head_end_x, head_end_y)))

    return arrows


def _float_path_to_geometry(float_path, net_id, original_start, original_end, sign,
                            src_stub_dir, tgt_stub_dir, src_extension, tgt_extension,
                            config, layer_names):
    """Convert floating-point path (x, y, layer) to segments and vias.

    Adds extension segments to ensure P and N connectors are parallel.

    Args:
        float_path: List of (x, y, layer_idx) tuples
        net_id: Network ID for the segments
        original_start: (x, y) start position or None
        original_end: (x, y) end position or None
        sign: +1 or -1 indicating which side of centerline this track is on
        src_stub_dir: (dx, dy) stub direction at source (pointing into route area)
        tgt_stub_dir: (dx, dy) stub direction at target (pointing into route area)
        src_extension: pre-calculated extension length at source end
        tgt_extension: pre-calculated extension length at target end
        config: GridRouteConfig with track_width, via_size, via_drill, debug_lines
        layer_names: List of layer names indexed by layer_idx

    Returns:
        (segments, vias, connector_lines) tuple
    """
    segs = []
    vias = []
    connector_lines = []  # Debug lines for connectors
    num_segments = len(float_path) - 1

    # Add connecting segment from original start if needed
    if original_start and len(float_path) > 0:
        first_x, first_y, first_layer = float_path[0]
        orig_x, orig_y = original_start
        if abs(orig_x - first_x) > 0.001 or abs(orig_y - first_y) > 0.001:
            # Use pre-calculated extension to make P and N connectors parallel
            # But skip extension if route start is between stub and extension point
            # (otherwise we'd create a U-turn that could cross the other net)
            use_extension = False
            if src_extension > 0.001:
                # Check where route start is relative to stub in stub direction
                to_route_x = first_x - orig_x
                to_route_y = first_y - orig_y
                dot = to_route_x * src_stub_dir[0] + to_route_y * src_stub_dir[1]
                # U-turn zone: route is between stub (0) and extension point (src_extension)
                # Only use extension if route is NOT in U-turn zone
                if dot <= 0.001 or dot >= src_extension - 0.001:
                    use_extension = True

            if use_extension:
                # Add extension segment in stub direction
                ext_x = orig_x + src_stub_dir[0] * src_extension
                ext_y = orig_y + src_stub_dir[1] * src_extension

                segs.append(Segment(
                    start_x=orig_x, start_y=orig_y,
                    end_x=ext_x, end_y=ext_y,
                    width=config.track_width,
                    layer=layer_names[first_layer],
                    net_id=net_id
                ))
                # Connector from extension to route
                segs.append(Segment(
                    start_x=ext_x, start_y=ext_y,
                    end_x=first_x, end_y=first_y,
                    width=config.track_width,
                    layer=layer_names[first_layer],
                    net_id=net_id
                ))
                if config.debug_lines:
                    connector_lines.append(((orig_x, orig_y), (ext_x, ext_y)))
                    connector_lines.append(((ext_x, ext_y), (first_x, first_y)))
            else:
                # No extension needed, direct connector
                segs.append(Segment(
                    start_x=orig_x, start_y=orig_y,
                    end_x=first_x, end_y=first_y,
                    width=config.track_width,
                    layer=layer_names[first_layer],
                    net_id=net_id
                ))
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
            # Use pre-calculated extension to make P and N connectors parallel
            # But skip extension if route end is between stub and extension point
            # (otherwise we'd create a U-turn that could cross the other net)
            use_extension = False
            if tgt_extension > 0.001:
                # Check where route end is relative to stub in stub direction
                # Vector from stub to route end
                to_route_x = last_x - orig_x
                to_route_y = last_y - orig_y
                # Dot product with stub direction - positive means route is in stub direction
                dot = to_route_x * tgt_stub_dir[0] + to_route_y * tgt_stub_dir[1]
                # U-turn zone: route is between stub (0) and extension point (tgt_extension)
                # Only use extension if route is NOT in U-turn zone
                # i.e., route is on pad side of stub (dot <= 0) or past extension (dot >= extension)
                if dot <= 0.001 or dot >= tgt_extension - 0.001:
                    use_extension = True

            if use_extension:
                # Extend along stub direction (toward route area)
                ext_x = orig_x + tgt_stub_dir[0] * tgt_extension
                ext_y = orig_y + tgt_stub_dir[1] * tgt_extension

                # Connector from route to extension point
                segs.append(Segment(
                    start_x=last_x, start_y=last_y,
                    end_x=ext_x, end_y=ext_y,
                    width=config.track_width,
                    layer=layer_names[last_layer],
                    net_id=net_id
                ))
                # Extension back to stub endpoint
                segs.append(Segment(
                    start_x=ext_x, start_y=ext_y,
                    end_x=orig_x, end_y=orig_y,
                    width=config.track_width,
                    layer=layer_names[last_layer],
                    net_id=net_id
                ))
                if config.debug_lines:
                    connector_lines.append(((last_x, last_y), (ext_x, ext_y)))
                    connector_lines.append(((ext_x, ext_y), (orig_x, orig_y)))
            else:
                # No extension needed, direct connector
                segs.append(Segment(
                    start_x=last_x, start_y=last_y,
                    end_x=orig_x, end_y=orig_y,
                    width=config.track_width,
                    layer=layer_names[last_layer],
                    net_id=net_id
                ))
                if config.debug_lines:
                    connector_lines.append(((last_x, last_y), (orig_x, orig_y)))

    return segs, vias, connector_lines


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
            # First point: bisector between start_dir (if provided) and first segment
            next_x, next_y = coord.to_float(centerline_path[1][0], centerline_path[1][1])
            seg_dx, seg_dy = next_x - x, next_y - y
            seg_len = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy) or 1
            seg_dx, seg_dy = seg_dx/seg_len, seg_dy/seg_len

            if start_dir is not None:
                # Normalize start_dir
                dir_len = math.sqrt(start_dir[0]**2 + start_dir[1]**2) or 1
                norm_start_dx = start_dir[0] / dir_len
                norm_start_dy = start_dir[1] / dir_len
                # Bisector between start_dir and first segment direction
                dx = norm_start_dx + seg_dx
                dy = norm_start_dy + seg_dy
                use_corner_scale = True  # Apply corner scaling at junction
            else:
                dx, dy = seg_dx, seg_dy
                use_corner_scale = False  # Single segment, no corner scaling
        elif i == len(centerline_path) - 1:
            # Last point: bisector between last segment and end_dir (if provided)
            prev_x, prev_y = coord.to_float(centerline_path[i-1][0], centerline_path[i-1][1])
            seg_dx, seg_dy = x - prev_x, y - prev_y
            seg_len = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy) or 1
            seg_dx, seg_dy = seg_dx/seg_len, seg_dy/seg_len

            if end_dir is not None:
                # Normalize end_dir
                dir_len = math.sqrt(end_dir[0]**2 + end_dir[1]**2) or 1
                norm_end_dx = end_dir[0] / dir_len
                norm_end_dy = end_dir[1] / dir_len
                # Bisector between last segment direction and end_dir
                dx = seg_dx + norm_end_dx
                dy = seg_dy + norm_end_dy
                use_corner_scale = True  # Apply corner scaling at junction
            else:
                dx, dy = seg_dx, seg_dy
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


def create_parallel_path_from_float(centerline_path, sign, spacing_mm=0.1, start_dir=None, end_dir=None):
    """
    Create a path parallel to centerline from float coordinates (no grid conversion).

    Uses bisector-based offsets at corners for smooth parallel paths.

    Args:
        centerline_path: List of (x, y, layer) floating-point coordinates
        sign: +1 for one track, -1 for the other
        spacing_mm: Distance from centerline in mm
        start_dir: Optional (dx, dy) direction to use at start point for perpendicular calc
        end_dir: Optional (dx, dy) direction to use at end point for perpendicular calc

    Returns:
        List of (x, y, layer) floating-point coordinates
    """
    if len(centerline_path) < 2:
        return list(centerline_path)

    result = []

    for i in range(len(centerline_path)):
        x, y, layer = centerline_path[i]

        use_corner_scale = True  # Only apply corner scaling for bisector calculations
        if i == 0:
            # First point: bisector between start_dir (if provided) and first segment
            next_x, next_y, _ = centerline_path[1]
            seg_dx, seg_dy = next_x - x, next_y - y
            seg_len = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy) or 1
            seg_dx, seg_dy = seg_dx/seg_len, seg_dy/seg_len

            if start_dir is not None:
                # Normalize start_dir
                dir_len = math.sqrt(start_dir[0]**2 + start_dir[1]**2) or 1
                norm_start_dx = start_dir[0] / dir_len
                norm_start_dy = start_dir[1] / dir_len
                # Bisector between start_dir and first segment direction
                dx = norm_start_dx + seg_dx
                dy = norm_start_dy + seg_dy
                use_corner_scale = True  # Apply corner scaling at junction
            else:
                dx, dy = seg_dx, seg_dy
                use_corner_scale = False  # Single segment, no corner scaling
        elif i == len(centerline_path) - 1:
            # Last point: bisector between last segment and end_dir (if provided)
            prev_x, prev_y, _ = centerline_path[i-1]
            seg_dx, seg_dy = x - prev_x, y - prev_y
            seg_len = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy) or 1
            seg_dx, seg_dy = seg_dx/seg_len, seg_dy/seg_len

            if end_dir is not None:
                # Normalize end_dir
                dir_len = math.sqrt(end_dir[0]**2 + end_dir[1]**2) or 1
                norm_end_dx = end_dir[0] / dir_len
                norm_end_dy = end_dir[1] / dir_len
                # Bisector between last segment direction and end_dir
                dx = seg_dx + norm_end_dx
                dy = seg_dy + norm_end_dy
                use_corner_scale = True  # Apply corner scaling at junction
            else:
                dx, dy = seg_dx, seg_dy
                use_corner_scale = False  # Single segment, no corner scaling
        else:
            # Corner: use bisector of incoming and outgoing directions
            prev = centerline_path[i-1]
            next_pt = centerline_path[i+1]

            if prev[2] != layer or next_pt[2] != layer:
                # Layer change - use incoming direction
                prev_x, prev_y, _ = prev
                dx, dy = x - prev_x, y - prev_y
            else:
                prev_x, prev_y, _ = prev
                next_x, next_y, _ = next_pt

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


def get_diff_pair_connector_regions(pcb_data: PCBData, diff_pair: DiffPairNet,
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
                         max_iterations_override=None, neighbor_stubs=None,
                         preferred_angles=None, direction_label=None, is_backward=False):
    """
    Attempt to route a diff pair in one direction.

    Args:
        preferred_angles: Optional (src_candidate, tgt_candidate) to use specific angles
                         from a previous probe instead of trying all combinations.

    Returns (route_data, iterations, blocked_cells, best_probe_combo) where:
        - route_data: dict with route info on success, None on failure
        - iterations: total iterations used
        - blocked_cells: list of blocked cells on failure
        - best_probe_combo: (src_idx, tgt_idx, src_candidate, tgt_candidate) when probing fails
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
    track_via_clearance = (config.clearance + config.track_width / 2 + config.via_size / 2) * config.routing_clearance_margin
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

    if direction_label and config.verbose:
        print(f"  Probing {direction_label}...")

    # Use base_obstacles for connector clearance checks if available
    # (connectors are single tracks that don't need diff pair extra clearance)
    connector_obstacles = base_obstacles if base_obstacles is not None else obstacles

    # Allow radius for finding open positions near blocked cells
    allow_radius = 2

    # Get all valid setback positions for source and target, sorted by preference
    src_candidates = _find_open_positions(
        center_src_x, center_src_y, src_dir_x, src_dir_y, src_layer, setback, "source",
        layer_names, spacing_mm, config, obstacles, connector_obstacles, coord, neighbor_stubs
    )
    if not src_candidates:
        blocked = _collect_setback_blocked_cells(
            center_src_x, center_src_y, src_dir_x, src_dir_y, src_layer, setback,
            config, obstacles, connector_obstacles, coord
        )
        return None, 0, blocked, None

    tgt_candidates = _find_open_positions(
        center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y, tgt_layer, setback, "target",
        layer_names, spacing_mm, config, obstacles, connector_obstacles, coord, neighbor_stubs
    )
    if not tgt_candidates:
        blocked = _collect_setback_blocked_cells(
            center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y, tgt_layer, setback,
            config, obstacles, connector_obstacles, coord
        )
        return None, 0, blocked, None

    # Calculate turning radius in grid units
    min_radius_grid = config.min_turning_radius / config.grid_step

    # Calculate turn cost: arc length for 45° turn = r * π/4, scaled by 1000
    turn_cost = int(min_radius_grid * math.pi / 4 * 1000)

    # Create pose-based router for centerline
    # Double via cost since diff pairs place two vias per layer change
    # via_proximity_cost is a multiplier on via cost in stub/BGA proximity zones (0 = block vias)
    # diff_pair_spacing is the P/N offset from centerline in grid units (for self-intersection prevention)
    # Use 2*spacing to prevent P/N tracks from crossing when centerline loops
    diff_pair_spacing_grid = max(1, int(2 * spacing_mm / config.grid_step + 0.5))
    # Convert max_turn_angle from degrees to 45° units
    max_turn_units = int(config.max_turn_angle / 45.0 + 0.5)

    # Calculate GND via spacing if enabled
    # GND via center = P/N track outer edge + clearance + via_radius
    gnd_via_perp_grid = 0
    gnd_via_along_grid = 0
    if config.gnd_via_enabled:
        gnd_via_perp_mm = spacing_mm + config.track_width/2 + config.clearance + config.via_size/2
        via_via_dist_mm = config.via_size + config.clearance
        gnd_via_perp_grid = coord.to_grid_dist(gnd_via_perp_mm)
        gnd_via_along_grid = coord.to_grid_dist(via_via_dist_mm)

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = int(config.vertical_attraction_cost * 1000 / config.grid_step) if config.vertical_attraction_cost > 0 else 0

    pose_router = PoseRouter(
        via_cost=config.via_cost * 1000 * 2,
        h_weight=config.heuristic_weight,
        turn_cost=turn_cost,
        min_radius_grid=min_radius_grid,
        via_proximity_cost=int(config.via_proximity_cost),
        diff_pair_spacing=diff_pair_spacing_grid,
        max_turn_units=max_turn_units,
        gnd_via_perp_offset=gnd_via_perp_grid,
        gnd_via_along_offset=gnd_via_along_grid,
        vertical_attraction_radius=attraction_radius_grid,
        vertical_attraction_bonus=attraction_bonus
    )

    # Route using pose-based A* with Dubins heuristic
    # Use 8x iterations for diff pairs due to larger pose-based search space
    # Pass via spacing in grid units so router can check P/N via positions
    via_spacing_grid = max(1, int(via_spacing / config.grid_step + 0.5))
    max_iters = max_iterations_override if max_iterations_override is not None else config.max_iterations * 8

    # Try angle combinations - only during probe phase (max_iterations_override set)
    # During full search, use preferred_angles if provided, otherwise best angle
    is_probe = max_iterations_override is not None
    total_iterations = 0
    last_blocked_cells = []

    # Determine which angle combinations to try
    if preferred_angles is not None:
        # Full search with specific angles from probe
        src_combos = [preferred_angles[0]]
        tgt_combos = [preferred_angles[1]]
    elif is_probe:
        # Probe: try angles in order, stop when one works (succeeds or reaches max iter)
        src_combos = src_candidates[:3]
        tgt_combos = tgt_candidates[:3]
    else:
        # Full search without preferred angles: use best by separation
        src_combos = src_candidates[:1]
        tgt_combos = tgt_candidates[:1]

    # Helper to build route_data dict (wraps module-level function with closure vars)
    def make_route_data(pose_path, src_actual_dir, tgt_actual_dir, src_angle, tgt_angle, gnd_via_dirs=None):
        return _make_route_data(
            pose_path, p_src_x, p_src_y, n_src_x, n_src_y,
            p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y,
            src_dir_x, src_dir_y, tgt_dir_x, tgt_dir_y,
            src_actual_dir, tgt_actual_dir,
            center_src_x, center_src_y, center_tgt_x, center_tgt_y,
            via_spacing, src_angle, tgt_angle, gnd_via_dirs
        )

    # Helper to setup and run a single route attempt
    def try_route(src_cand, tgt_cand, is_probe_attempt=False):
        s_gx, s_gy, s_dx, s_dy, s_ang, _ = src_cand
        t_gx, t_gy, t_dx, t_dy, t_ang, _ = tgt_cand

        # During probe, clear previous attempt's cells to keep each attempt independent
        if is_probe_attempt:
            obstacles.clear_allowed_cells()
            obstacles.clear_source_target_cells()

        # Add allowed cells around source and target
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(s_gx + dx, s_gy + dy)
                obstacles.add_allowed_cell(t_gx + dx, t_gy + dy)
        obstacles.add_source_target_cell(s_gx, s_gy, src_layer)
        obstacles.add_source_target_cell(t_gx, t_gy, tgt_layer)

        s_theta = direction_to_theta_idx(s_dx, s_dy)
        t_theta = direction_to_theta_idx(-t_dx, -t_dy)

        path, iters, blocked, gnd_via_dirs = pose_router.route_pose_with_frontier(
            obstacles, s_gx, s_gy, src_layer, s_theta,
            t_gx, t_gy, tgt_layer, t_theta,
            max_iters, diff_pair_via_spacing=via_spacing_grid
        )
        return path, iters, blocked, (s_dx, s_dy), (t_dx, t_dy), s_ang, t_ang, gnd_via_dirs

    # Select best angles - during probe, try angles sequentially until one works
    selected_src = src_combos[0]
    selected_tgt = tgt_combos[0]
    stuck_threshold = max(100, max_iters // 10)

    # When routing backward, the "source" in routing terms is the physical target
    # Use physical terminology: source = physical source end, target = physical target end
    first_label = "target" if is_backward else "source"
    second_label = "source" if is_backward else "target"

    if is_probe:
        # Probe angles at the routing source (first endpoint we're routing from)
        # Track the best individual iteration count (not sum) for accurate reporting
        found_first = False
        first_blocked_cells = []
        best_first_iters = 0
        if len(src_combos) > 1:
            for src_cand in src_combos:
                path, iters, blocked, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs = try_route(src_cand, tgt_combos[0], is_probe_attempt=True)
                total_iterations += iters
                best_first_iters = max(best_first_iters, iters)

                if path is not None:
                    return make_route_data(path, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs), iters, [], None
                if iters >= max_iters:
                    selected_src = src_cand
                    found_first = True
                    if config.verbose:
                        print(f"    {first_label} {src_cand[4]:+.1f}° OK ({iters} iters)")
                    break
                if config.verbose:
                    print(f"    {first_label} {src_cand[4]:+.1f}° blocked ({iters} iters)")
                # Collect blocked cells from this attempt
                first_blocked_cells.extend(blocked)

            if not found_first:
                # All source angles blocked - return probe_blocked state for rip-up
                # Return best individual iteration count, not sum
                return None, best_first_iters, first_blocked_cells, ('probe_blocked', 'first', selected_src, selected_tgt)
        else:
            # Only one source option - still need to test it (don't assume it works)
            path, iters, blocked, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs = try_route(src_combos[0], tgt_combos[0], is_probe_attempt=True)
            total_iterations += iters
            best_first_iters = iters

            if path is not None:
                return make_route_data(path, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs), iters, [], None
            if iters >= max_iters:
                if config.verbose:
                    print(f"    {first_label} {src_combos[0][4]:+.1f}° OK ({iters} iters)")
                found_first = True
            else:
                if config.verbose:
                    print(f"    {first_label} {src_combos[0][4]:+.1f}° blocked ({iters} iters)")
                first_blocked_cells.extend(blocked)
                return None, iters, first_blocked_cells, ('probe_blocked', 'first', selected_src, selected_tgt)

        # Probe angles at the routing target (second endpoint)
        found_second = False
        second_blocked_cells = []
        best_second_iters = 0
        if len(tgt_combos) > 1:
            for tgt_cand in tgt_combos:
                path, iters, blocked, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs = try_route(selected_src, tgt_cand, is_probe_attempt=True)
                total_iterations += iters
                best_second_iters = max(best_second_iters, iters)

                if path is not None:
                    return make_route_data(path, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs), best_first_iters + iters, [], None
                if iters >= max_iters:
                    selected_tgt = tgt_cand
                    found_second = True
                    if config.verbose:
                        print(f"    {second_label} {tgt_cand[4]:+.1f}° OK ({iters} iters)")
                    break
                if config.verbose:
                    print(f"    {second_label} {tgt_cand[4]:+.1f}° blocked ({iters} iters)")
                # Collect blocked cells from this attempt
                second_blocked_cells.extend(blocked)

            if not found_second:
                # All target angles blocked - return probe_blocked state for rip-up
                # Return best individual iteration count (min of source success and best target attempt)
                return None, min(best_first_iters, best_second_iters), second_blocked_cells, ('probe_blocked', 'second', selected_src, selected_tgt)
        else:
            # Only one target option - if source had multiple combos, we need to test selected_src with this target
            if len(src_combos) > 1:
                path, iters, blocked, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs = try_route(selected_src, tgt_combos[0], is_probe_attempt=True)
                total_iterations += iters
                best_second_iters = iters

                if path is not None:
                    return make_route_data(path, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs), best_first_iters + iters, [], None
                if iters >= max_iters:
                    if config.verbose:
                        print(f"    {second_label} {tgt_combos[0][4]:+.1f}° OK ({iters} iters)")
                else:
                    if config.verbose:
                        print(f"    {second_label} {tgt_combos[0][4]:+.1f}° blocked ({iters} iters)")
                    second_blocked_cells.extend(blocked)
                    return None, min(best_first_iters, iters), second_blocked_cells, ('probe_blocked', 'second', selected_src, selected_tgt)
            else:
                # Both source and target have only 1 option - we already tested this in source section
                # best_first_iters already has the iteration count from that test
                best_second_iters = best_first_iters
                if config.verbose:
                    print(f"    {second_label} {tgt_combos[0][4]:+.1f}° OK ({best_first_iters} iters)")

        # Return selected angles for full search
        # Return the best iteration count from the successful probes (min of source and target)
        # This represents what the selected combo achieved
        best_probe_iters = min(best_first_iters, best_second_iters)
        return None, best_probe_iters, [], (0, 0, selected_src, selected_tgt)

    # Non-probe: run single route attempt with selected angles
    # Clear any leftover cells from probing and set up fresh for full search
    obstacles.clear_allowed_cells()
    obstacles.clear_source_target_cells()

    src_gx, src_gy, src_actual_dir_x, src_actual_dir_y, src_angle, src_dist = selected_src
    tgt_gx, tgt_gy, tgt_actual_dir_x, tgt_actual_dir_y, tgt_angle, tgt_dist = selected_tgt

    if src_angle != 0 or tgt_angle != 0:
        # Use physical terminology (routing src/tgt swap when backward)
        src_label = "target" if is_backward else "source"
        tgt_label = "source" if is_backward else "target"
        print(f"  {src_label.capitalize()} setback: {src_angle:+.1f}°, {tgt_label} setback: {tgt_angle:+.1f}°")

    src_theta_idx = direction_to_theta_idx(src_actual_dir_x, src_actual_dir_y)
    tgt_theta_idx = direction_to_theta_idx(-tgt_actual_dir_x, -tgt_actual_dir_y)
    print(f"  Pose routing: src_theta={src_theta_idx} ({src_actual_dir_x:.2f},{src_actual_dir_y:.2f}), tgt_theta={tgt_theta_idx} (arriving from {-tgt_actual_dir_x:.2f},{-tgt_actual_dir_y:.2f})")

    path, iters, blocked, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs = try_route(selected_src, selected_tgt)
    total_iterations += iters

    if path is not None:
        return make_route_data(path, src_dir, tgt_dir, s_ang, t_ang, gnd_dirs), total_iterations, [], None
    return None, total_iterations, blocked, None


def route_diff_pair_with_obstacles(pcb_data: PCBData, diff_pair: DiffPairNet,
                                    config: GridRouteConfig,
                                    obstacles: GridObstacleMap,
                                    base_obstacles: GridObstacleMap = None,
                                    unrouted_stubs: List[Tuple[float, float]] = None) -> Optional[dict]:
    """
    Route a differential pair using centerline + offset approach.

    1. Routes a single centerline path using A* (GridRouter)
    2. Simplifies the path by removing collinear points
    3. Creates P and N paths using perpendicular offsets from centerline
    """
    p_net_id = diff_pair.p_net_id
    n_net_id = diff_pair.n_net_id

    # Find GND net ID for GND via placement (if enabled)
    gnd_net_id = None
    if config.gnd_via_enabled:
        for net_id, net in pcb_data.nets.items():
            if net.name.upper() == 'GND':
                gnd_net_id = net_id
                break

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

    # Set endpoint exempt positions for stub proximity costs
    # This allows routes to reach endpoints without being penalized by nearby stubs
    min_stub_pair_spacing = config.track_width + config.diff_pair_gap + config.clearance
    exempt_radius_grid = coord.to_grid_dist(min_stub_pair_spacing)
    endpoint_positions = [
        coord.to_grid(original_src[5], original_src[6]),  # Source P
        coord.to_grid(original_src[7], original_src[8]),  # Source N
        coord.to_grid(original_tgt[5], original_tgt[6]),  # Target P
        coord.to_grid(original_tgt[7], original_tgt[8]),  # Target N
    ]
    obstacles.set_endpoint_exempt(endpoint_positions, exempt_radius_grid)

    # Calculate spacing from config (track_width + diff_pair_gap is center-to-center)
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2

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
    route_data, first_probe_iters, first_blocked, first_best_combo = _try_route_direction(
        first_src, first_tgt, pcb_data, config, obstacles, base_obstacles,
        coord, layer_names, spacing_mm, p_net_id, n_net_id,
        max_iterations_override=probe_iterations, neighbor_stubs=unrouted_stubs,
        direction_label=first_label, is_backward=(first_label == "backward")
    )

    # Check if first probe was blocked (all angles failed)
    if first_best_combo and isinstance(first_best_combo[0], str) and first_best_combo[0] == 'probe_blocked':
        # Map 'first'/'second' to physical 'source'/'target'
        blocked_endpoint = first_best_combo[1]  # 'first' or 'second'
        if first_label == 'backward':
            # backward: first=target, second=source
            physical_endpoint = 'target' if blocked_endpoint == 'first' else 'source'
        else:
            # forward: first=source, second=target
            physical_endpoint = 'source' if blocked_endpoint == 'first' else 'target'
        print(f"  Probe {first_label} blocked at {physical_endpoint} after {first_probe_iters} iterations")
        return {
            'probe_blocked': True,
            'blocked_at': physical_endpoint,
            'blocked_cells': first_blocked,
            'iterations': first_probe_iters,
            'direction': first_label,
        }

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
        route_data, second_probe_iters, second_blocked, second_best_combo = _try_route_direction(
            second_src, second_tgt, pcb_data, config, obstacles, base_obstacles,
            coord, layer_names, spacing_mm, p_net_id, n_net_id,
            max_iterations_override=probe_iterations, neighbor_stubs=unrouted_stubs,
            direction_label=second_label, is_backward=(second_label == "backward")
        )

        # Check if second probe was blocked (all angles failed)
        if second_best_combo and isinstance(second_best_combo[0], str) and second_best_combo[0] == 'probe_blocked':
            # Map 'first'/'second' to physical 'source'/'target'
            blocked_endpoint = second_best_combo[1]  # 'first' or 'second'
            if second_label == 'backward':
                # backward: first=target, second=source
                physical_endpoint = 'target' if blocked_endpoint == 'first' else 'source'
            else:
                # forward: first=source, second=target
                physical_endpoint = 'source' if blocked_endpoint == 'first' else 'target'
            print(f"  Probe {second_label} blocked at {physical_endpoint} after {second_probe_iters} iterations")
            return {
                'probe_blocked': True,
                'blocked_at': physical_endpoint,
                'blocked_cells': second_blocked,
                'iterations': first_probe_iters + second_probe_iters,
                'direction': second_label,
            }

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
                return {
                    'failed': True,
                    'iterations': first_probe_iters + second_probe_iters,
                    'blocked_cells_forward': first_blocked if first_label == "forward" else second_blocked,
                    'blocked_cells_backward': second_blocked if first_label == "forward" else first_blocked,
                    'iterations_forward': first_probe_iters if first_label == "forward" else second_probe_iters,
                    'iterations_backward': second_probe_iters if first_label == "forward" else first_probe_iters,
                }

            # At least one direction made progress - do full search on the more promising one
            # Prefer forward direction when both reached max_iters, otherwise pick higher iteration count
            first_reached_max = first_probe_iters >= probe_iterations
            second_reached_max = second_probe_iters >= probe_iterations

            # Prefer first direction (forward by default) when both succeed or have equal iterations
            if first_reached_max and second_reached_max:
                prefer_first = True  # Both succeeded, prefer forward
            elif first_probe_iters >= second_probe_iters:
                prefer_first = True
            else:
                prefer_first = False

            if prefer_first:
                promising_src, promising_tgt, promising_label = first_src, first_tgt, first_label
                fallback_src, fallback_tgt, fallback_label = second_src, second_tgt, second_label
                promising_probe_iters, promising_blocked = first_probe_iters, first_blocked
                fallback_probe_iters, fallback_blocked = second_probe_iters, second_blocked
                promising_best_combo, fallback_best_combo = first_best_combo, second_best_combo
            else:
                promising_src, promising_tgt, promising_label = second_src, second_tgt, second_label
                fallback_src, fallback_tgt, fallback_label = first_src, first_tgt, first_label
                promising_probe_iters, promising_blocked = second_probe_iters, second_blocked
                fallback_probe_iters, fallback_blocked = first_probe_iters, first_blocked
                promising_best_combo, fallback_best_combo = second_best_combo, first_best_combo

            print(f"  Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {promising_label} with full iterations...")

            # Mix best angles from both probes:
            # - Forward probe's selected_src is best angle at physical source
            # - Backward probe's selected_src is best angle at physical target
            # best_combo is (src_idx, tgt_idx, src_candidate, tgt_candidate)
            if promising_best_combo and fallback_best_combo:
                # Use promising direction's source angle + fallback direction's source angle (as target)
                preferred = (promising_best_combo[2], fallback_best_combo[2])
            elif promising_best_combo:
                preferred = (promising_best_combo[2], promising_best_combo[3])
            else:
                preferred = None

            # Full search on promising direction with best angles from probe
            route_data, full_iters, blocked_cells, _ = _try_route_direction(
                promising_src, promising_tgt, pcb_data, config, obstacles, base_obstacles,
                coord, layer_names, spacing_mm, p_net_id, n_net_id,
                neighbor_stubs=unrouted_stubs, preferred_angles=preferred,
                direction_label=None, is_backward=(promising_label == "backward")
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
                    # Mix best angles: fallback's source + promising's source (as target)
                    if fallback_best_combo and promising_best_combo:
                        fallback_preferred = (fallback_best_combo[2], promising_best_combo[2])
                    elif fallback_best_combo:
                        fallback_preferred = (fallback_best_combo[2], fallback_best_combo[3])
                    else:
                        fallback_preferred = None
                    route_data, fallback_full_iters, fallback_full_blocked, _ = _try_route_direction(
                        fallback_src, fallback_tgt, pcb_data, config, obstacles, base_obstacles,
                        coord, layer_names, spacing_mm, p_net_id, n_net_id,
                        neighbor_stubs=unrouted_stubs, preferred_angles=fallback_preferred,
                        direction_label=None, is_backward=(fallback_label == "backward")
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
    src_actual_dir_x, src_actual_dir_y = route_data['src_actual_dir_x'], route_data['src_actual_dir_y']
    tgt_actual_dir_x, tgt_actual_dir_y = route_data['tgt_actual_dir_x'], route_data['tgt_actual_dir_y']
    center_src_x, center_src_y = route_data['center_src_x'], route_data['center_src_y']
    center_tgt_x, center_tgt_y = route_data['center_tgt_x'], route_data['center_tgt_y']
    via_spacing = route_data['via_spacing']
    gnd_via_dirs = route_data.get('gnd_via_dirs', [])

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

    # Detect polarity (which side of centerline P should be on)
    p_sign, polarity_fixed, polarity_swap_needed, has_layer_change = _detect_polarity(
        simplified_path, coord,
        p_src_x, p_src_y, n_src_x, n_src_y,
        p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y,
        config
    )
    n_sign = -p_sign

    # Create P and N paths using perpendicular offsets from centerline
    # Use actual setback directions at endpoints so perpendicular offsets maintain
    # proper spacing along the connector segments (especially when setback angle is used)
    start_stub_dir = (src_actual_dir_x, src_actual_dir_y)
    end_stub_dir = (-tgt_actual_dir_x, -tgt_actual_dir_y)  # Negate because we arrive at target stubs
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

    # Prepare stub direction tuples for extension calculation
    src_stub_dir_tuple = (src_dir_x, src_dir_y)
    tgt_stub_dir_tuple = (tgt_dir_x, tgt_dir_y)

    # Pre-calculate extensions for source and target connectors
    # These ensure P and N connector segments are parallel
    # Source end
    p_src_route = p_float_path[0][:2] if p_float_path else p_start
    n_src_route = n_float_path[0][:2] if n_float_path else n_start
    src_p_ext, src_n_ext = _calculate_parallel_extension(
        p_start, n_start, p_src_route, n_src_route,
        src_stub_dir_tuple, p_sign
    )

    # Target end
    p_tgt_route = p_float_path[-1][:2] if p_float_path else p_end
    n_tgt_route = n_float_path[-1][:2] if n_float_path else n_end
    tgt_p_ext, tgt_n_ext = _calculate_parallel_extension(
        p_end, n_end, p_tgt_route, n_tgt_route,
        tgt_stub_dir_tuple, p_sign
    )

    # Convert P path
    p_segs, p_vias, p_conn_lines = _float_path_to_geometry(
        p_float_path, p_net_id, p_start, p_end, p_sign,
        src_stub_dir_tuple, tgt_stub_dir_tuple, src_p_ext, tgt_p_ext,
        config, layer_names
    )
    new_segments.extend(p_segs)
    new_vias.extend(p_vias)
    debug_connector_lines.extend(p_conn_lines)

    # Convert N path
    n_segs, n_vias, n_conn_lines = _float_path_to_geometry(
        n_float_path, n_net_id, n_start, n_end, n_sign,
        src_stub_dir_tuple, tgt_stub_dir_tuple, src_n_ext, tgt_n_ext,
        config, layer_names
    )
    new_segments.extend(n_segs)
    new_vias.extend(n_vias)
    debug_connector_lines.extend(n_conn_lines)

    # Create GND vias at layer changes if enabled
    gnd_vias = _create_gnd_vias(
        simplified_path, coord, config, layer_names, spacing_mm, gnd_net_id, gnd_via_dirs
    )
    new_vias.extend(gnd_vias)

    # Convert float paths back to grid format for return value
    p_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in p_float_path]
    n_path = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
              for x, y, layer in n_float_path]

    # Build stub direction arrows for debug visualization (User.4)
    debug_stub_arrows = []
    if config.debug_lines:
        debug_stub_arrows = _generate_debug_arrows(
            center_src_x, center_src_y, src_dir_x, src_dir_y,
            center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y
        )

    # Calculate source and target stub lengths for each net
    # These are based on ORIGINAL positions (before polarity swap affects route targets)
    src_layer_name = layer_names[original_src[4]]
    tgt_layer_name = layer_names[original_tgt[4]]

    p_src_stub_segs = get_stub_segments(pcb_data, p_net_id, p_src_x, p_src_y, src_layer_name)
    p_tgt_stub_segs = get_stub_segments(pcb_data, p_net_id, p_tgt_x, p_tgt_y, tgt_layer_name)
    n_src_stub_segs = get_stub_segments(pcb_data, n_net_id, n_src_x, n_src_y, src_layer_name)
    n_tgt_stub_segs = get_stub_segments(pcb_data, n_net_id, n_tgt_x, n_tgt_y, tgt_layer_name)

    p_src_stub_length = sum(segment_length(s) for s in p_src_stub_segs)
    p_tgt_stub_length = sum(segment_length(s) for s in p_tgt_stub_segs)
    n_src_stub_length = sum(segment_length(s) for s in n_src_stub_segs)
    n_tgt_stub_length = sum(segment_length(s) for s in n_tgt_stub_segs)

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
        # Additional data for length matching meander regeneration
        'is_diff_pair': True,
        'centerline_path_grid': simplified_path,  # Grid coords for meander insertion
        'p_sign': p_sign,
        'n_sign': n_sign,
        'spacing_mm': spacing_mm,
        'start_stub_dir': start_stub_dir,
        'end_stub_dir': end_stub_dir,
        'p_net_id': p_net_id,
        'n_net_id': n_net_id,
        # Connector segment data for regeneration
        'p_start': p_start,
        'n_start': n_start,
        'p_end': p_end,
        'n_end': n_end,
        'src_stub_dir': src_stub_dir_tuple,
        'tgt_stub_dir': tgt_stub_dir_tuple,
        'layer_names': layer_names,
        # Stub layer names for length matching (original source/target, not affected by routing direction)
        'src_layer_name': layer_names[original_src[4]],
        'tgt_layer_name': layer_names[original_tgt[4]],
        # Pre-calculated stub lengths for intra-pair matching (based on original positions, not affected by polarity swap)
        'p_src_stub_length': p_src_stub_length,
        'p_tgt_stub_length': p_tgt_stub_length,
        'n_src_stub_length': n_src_stub_length,
        'n_tgt_stub_length': n_tgt_stub_length,
        # GND via data for regeneration after length matching
        'gnd_net_id': gnd_net_id,
        'gnd_via_dirs': gnd_via_dirs,
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
    track_via_clearance = (config.clearance + config.track_width / 2 + config.via_size / 2) * config.routing_clearance_margin

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
