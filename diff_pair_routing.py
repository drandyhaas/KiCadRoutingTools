"""
Differential pair routing functions.

Routes differential pairs (P and N nets) together using centerline + offset approach.
"""

import math
from typing import List, Optional, Tuple, Dict

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from routing_utils import segment_length, build_layer_map, pos_key
from connectivity import (
    find_connected_groups, find_stub_free_ends, get_stub_direction, get_net_endpoints,
    get_stub_segments, get_stub_vias, calculate_stub_via_barrel_length
)
from obstacle_map import check_line_clearance
from geometry_utils import simplify_path, segments_intersect_tuple
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


def _footprint_pad_centroid(pcb_data: PCBData, net_id: int, x: float, y: float,
                            tolerance: float = 0.05) -> Optional[Tuple[float, float]]:
    """Find the pad of net_id at (x, y) and return the centroid of all pads on its footprint."""
    for pad in pcb_data.pads_by_net.get(net_id, []):
        if abs(pad.global_x - x) < tolerance and abs(pad.global_y - y) < tolerance:
            footprint = pcb_data.footprints.get(pad.component_ref)
            if footprint and footprint.pads:
                cx = sum(p.global_x for p in footprint.pads) / len(footprint.pads)
                cy = sum(p.global_y for p in footprint.pads) / len(footprint.pads)
                return (cx, cy)
    return None


def get_pair_end_direction(pcb_data: PCBData, p_net_id: int, n_net_id: int,
                           p_segments: List[Segment], n_segments: List[Segment],
                           p_x: float, p_y: float, n_x: float, n_y: float,
                           layer_name: str,
                           other_center: Optional[Tuple[float, float]] = None
                           ) -> Tuple[float, float, bool]:
    """
    Compute the escape direction for one end of a differential pair.

    Normally this is the average of the P and N stub directions. When both
    endpoints are bare pads (no stub segments), the stub directions are (0,0)
    and the setback machinery would degenerate to the pad-pair center, letting
    the offset P/N tracks cross the partner net's pads. In that case we
    synthesize a direction perpendicular to the P-N pad axis, pointing away
    from the owning component's pad centroid (falling back to "toward the
    other end of the route").

    Returns (dir_x, dir_y, synthesized). The direction is normalized, or (0,0)
    if no direction could be determined. synthesized is True when the direction
    came from pad geometry rather than stub segments.
    """
    p_dir = get_stub_direction(p_segments, p_x, p_y, layer_name)
    n_dir = get_stub_direction(n_segments, n_x, n_y, layer_name)

    dir_x = (p_dir[0] + n_dir[0]) / 2
    dir_y = (p_dir[1] + n_dir[1]) / 2
    dir_len = math.sqrt(dir_x * dir_x + dir_y * dir_y)
    if dir_len > 1e-6:
        dir_x, dir_y = dir_x / dir_len, dir_y / dir_len
        # get_stub_direction expects the stub FREE END; the multipoint diff-pair
        # path passes the PAD position instead, so for a pad with an escape stub
        # (e.g. a fanned-out QFN pin) it returns the direction reversed - pointing
        # INTO the component body, which backs the centerline setback into the
        # chip and the route dies on the pad field.
        #
        # Correct it ONLY when the query point IS a pad of the owning component
        # (_footprint_pad_centroid returns non-None), which is exactly the buggy
        # case. A true free-end terminal is not at a pad -> centroid is None -> no
        # orientation, so a genuine stub that legitimately doesn't point straight
        # outward is left untouched (avoids false-flipping free-end stubs).
        #
        # When the query point is a pad: orient the escape away from the
        # component's pad centroid; but if that centroid coincides with the
        # terminal center (a 2-pad component - resistor/2-pin part - whose two pads
        # ARE the pair), there is no "body" to point away from, so orient toward
        # the other terminal instead.
        centroid = (_footprint_pad_centroid(pcb_data, p_net_id, p_x, p_y) or
                    _footprint_pad_centroid(pcb_data, n_net_id, n_x, n_y))
        if centroid:
            cx, cy = (p_x + n_x) / 2, (p_y + n_y) / 2
            if math.hypot(cx - centroid[0], cy - centroid[1]) > 0.05:
                away = (cx - centroid[0], cy - centroid[1])          # away from body
            elif other_center is not None:
                away = (other_center[0] - cx, other_center[1] - cy)  # toward other end
            else:
                away = None
            if away is not None and dir_x * away[0] + dir_y * away[1] < 0:
                dir_x, dir_y = -dir_x, -dir_y
        return (dir_x, dir_y, False)

    # Bare pads (or cancelling stub directions): synthesize from pad geometry.
    axis_x, axis_y = p_x - n_x, p_y - n_y
    axis_len = math.sqrt(axis_x * axis_x + axis_y * axis_y)
    if axis_len < 1e-6:
        return (0.0, 0.0, False)
    perp_x, perp_y = -axis_y / axis_len, axis_x / axis_len

    center_x, center_y = (p_x + n_x) / 2, (p_y + n_y) / 2

    # Prefer escaping away from the owning component's body (pad centroid)
    centroid = (_footprint_pad_centroid(pcb_data, p_net_id, p_x, p_y) or
                _footprint_pad_centroid(pcb_data, n_net_id, n_x, n_y))
    if centroid:
        dot = perp_x * (center_x - centroid[0]) + perp_y * (center_y - centroid[1])
        if abs(dot) > 0.05:
            return (perp_x, perp_y, True) if dot > 0 else (-perp_x, -perp_y, True)

    # Symmetric component (e.g. 2-pad resistor): escape toward the other end
    if other_center is not None:
        dot = perp_x * (other_center[0] - center_x) + perp_y * (other_center[1] - center_y)
        if abs(dot) > 0.05:
            return (perp_x, perp_y, True) if dot > 0 else (-perp_x, -perp_y, True)

    return (perp_x, perp_y, True)


def _setback_ladder(setback, spacing_mm, pad_gap_half, config):
    """Candidate setback radii to scan, preferred first (issue #90).

    The old single fixed-radius search failed whole pairs whenever every
    angle on that one ring was blocked - dense terminals (0402 resistor
    rows, SOT-23s, USB-C) often have an open ring nearer or farther.

    The geometric floor is the longitudinal room the P/N connector fan
    needs to taper from the pad separation down to the pair spacing at
    <= ~45 degrees: |pad_gap/2 - spacing| plus a grid step of margin,
    never less than 2*spacing. Below that the connectors would kink.
    Above the configured setback, 1.5x and 2x are also tried - sometimes
    the neighborhood only opens up farther out.
    """
    floor = max(2 * spacing_mm,
                abs(pad_gap_half - spacing_mm) + config.grid_step)
    # The configured/default setback is always rung 1, even below the floor -
    # an explicit --diff-pair-centerline-setback must be honored as given.
    ladder = [setback]
    for s in (0.75 * setback, 0.5 * setback, floor,
              1.5 * setback, 2 * setback):
        s = max(s, floor)
        if all(abs(s - e) > 1e-6 for e in ladder):
            ladder.append(s)
    return ladder


def _launch_assoc_tol(config):
    """Distance within which a via/THT pad is treated as belonging to (escaping)
    a diff-pair terminal: hand- or auto-placed escape vias sit at/near the pad or
    stub tip, offset by up to ~a via radius, not grid-coincident (watchy USB_D).
    Stays below a fine pad pitch so it can't grab a neighbour terminal's via."""
    return max(config.grid_step * 1.5, config.via_size * 0.75 + config.track_width / 2)


def _endpoint_launch_layer_indices(pcb_data, net_id, x, y, config, tol=None):
    """Routing-layer indices a coupled launch may use at endpoint (x, y) of `net_id`.

    A diff-pair terminal that sits on a through-via or a through-hole pad can
    launch on ANY routing layer that copper spans, not just the stub's layer --
    the existing barrel already ties the layers together (issue #195). Returns
    the set of indices into config.layers that an existing via/THT pad of this
    net at (x, y) connects. The router uses this to retry the setback search on
    an open inner layer when the stub layer's corridor is jammed.
    """
    if tol is None:
        tol = _launch_assoc_tol(config)
    copper = getattr(pcb_data.board_info, 'copper_layers', None) or list(config.layers)
    cu_index = {name: i for i, name in enumerate(copper)}
    routing_idx = {name: i for i, name in enumerate(config.layers)}

    def _span_all():
        return {ridx for lname, ridx in routing_idx.items() if lname in cu_index}

    spanned = set()
    # Through-vias: span from their top to bottom copper layer.
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        if abs(via.x - x) > tol or abs(via.y - y) > tol:
            continue
        vlayers = via.layers or ['F.Cu', 'B.Cu']
        v_idxs = [cu_index[l] for l in vlayers if l in cu_index]
        if not v_idxs:
            continue
        lo, hi = min(v_idxs), max(v_idxs)
        for lname, ridx in routing_idx.items():
            if lname in cu_index and lo <= cu_index[lname] <= hi:
                spanned.add(ridx)
    # Through-hole pads (drill > 0) span every copper layer, like a via barrel.
    for pad in pcb_data.pads_by_net.get(net_id, []):
        if getattr(pad, 'drill', 0) and pad.drill > 0 and \
                abs(pad.global_x - x) <= tol and abs(pad.global_y - y) <= tol:
            spanned |= _span_all()
            break
    return spanned


def _pt_seg_dist(px, py, ax, ay, bx, by):
    """Distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _terminal_escape_vias(pcb_data, p_net_id, n_net_id, p_term, n_term, config):
    """The pair's own escape/terminal vias sitting at the P/N launch terminals.

    These are excluded from the routing obstacle map (own-net), so the setback
    search is blind to them -- a per-half offset connector can then graze the
    PARTNER half's escape via (issue #165 / watchy USB_D). Returns a list of
    (x, y, size, net_id) for vias of either net within the launch-association
    tolerance of either terminal.
    """
    tol = _launch_assoc_tol(config)
    out = []
    for via in getattr(pcb_data, 'vias', None) or []:
        if via.net_id not in (p_net_id, n_net_id):
            continue
        if (math.hypot(via.x - p_term[0], via.y - p_term[1]) <= tol or
                math.hypot(via.x - n_term[0], via.y - n_term[1]) <= tol):
            # Guard size like obstacle_map does, so a 0/missing size can't shrink
            # the graze clearance and let a real partner-via graze slip through.
            out.append((via.x, via.y, via.size if via.size > 0 else config.via_size, via.net_id))
    return out


def _make_offset_connector_check(p_term, n_term, escape_vias, spacing_mm, config):
    """Build an offset_check(launch_x, launch_y, dir_x, dir_y) -> bool closure.

    For a candidate setback launch point, the P and N tracks sit at +-spacing
    perpendicular to the launch direction; each terminal runs a short connector
    to its offset launch point. This returns False if either connector would
    clip the *partner* terminal's escape via (the gap the obstacle map misses),
    so the setback search rejects that candidate and picks a clearing one.
    Returns None when there are no escape vias to avoid (no behaviour change).
    """
    if not escape_vias:
        return None

    # A via this close to a terminal IS that terminal's own launch via (it need
    # not sit exactly on the stub tip) -- the connector legitimately starts on
    # it, so don't count it as a graze of its own leg.
    own_tol = _launch_assoc_tol(config)

    def _leg_grazes(term, off):
        for vx, vy, vsize, _ in escape_vias:
            # skip the via at this terminal (its own launch via)
            if math.hypot(vx - term[0], vy - term[1]) <= own_tol:
                continue
            need = config.clearance + config.track_width / 2 + vsize / 2
            if _pt_seg_dist(vx, vy, term[0], term[1], off[0], off[1]) < need:
                return True
        return False

    def check(lx, ly, dx, dy):
        px, py = -dy, dx  # perpendicular
        off_a = (lx + px * spacing_mm, ly + py * spacing_mm)
        off_b = (lx - px * spacing_mm, ly - py * spacing_mm)
        # Assign each terminal to its nearer offset launch point (the route's
        # likely polarity). Rejecting both assignments over-rejects tight-but-
        # legal launches (e.g. #195's 0.5mm-pitch escape vias), so use the
        # nearest assignment only.
        def d2(a, b):
            return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
        if d2(p_term, off_a) + d2(n_term, off_b) <= d2(p_term, off_b) + d2(n_term, off_a):
            legs = ((p_term, off_a), (n_term, off_b))
        else:
            legs = ((p_term, off_b), (n_term, off_a))
        return not (_leg_grazes(*legs[0]) or _leg_grazes(*legs[1]))
    return check


def _find_open_positions_multilayer(center_x, center_y, dir_x, dir_y, layer_idx,
                                    alt_layers, setback, pad_gap_half, label,
                                    layer_names, spacing_mm, config, obstacles,
                                    connector_obstacles, coord, neighbor_stubs,
                                    offset_check=None):
    """_find_open_positions_laddered, but free to launch on an alternate routing
    layer reachable through the endpoint's via/THT barrel (issue #195).

    Tries the stub layer first and keeps it when it yields a clean (non-rotated)
    launch -- unchanged behaviour. Only when the stub layer is fully blocked or
    can launch solely by rotating sideways does it retry the alternate layers
    (e.g. an open inner layer under a chip) for a clean launch. Returns
    (candidates, used_setback, rotated, launch_layer_idx).
    """
    cands, used, rotated = _find_open_positions_laddered(
        center_x, center_y, dir_x, dir_y, layer_idx, setback, pad_gap_half,
        label, layer_names, spacing_mm, config, obstacles, connector_obstacles,
        coord, neighbor_stubs, offset_check=offset_check)
    if cands and not rotated:
        return cands, used, rotated, layer_idx

    fallback = (cands, used, rotated, layer_idx) if cands else None
    for alt in alt_layers:
        if alt == layer_idx:
            continue
        a_cands, a_used, a_rot = _find_open_positions_laddered(
            center_x, center_y, dir_x, dir_y, alt, setback, pad_gap_half,
            label, layer_names, spacing_mm, config, obstacles,
            connector_obstacles, coord, neighbor_stubs, offset_check=offset_check)
        if a_cands and not a_rot:
            print(f"      {label}: {layer_names[layer_idx]} corridor jammed - "
                  f"launching on {layer_names[alt]} (reachable through the endpoint via)")
            return a_cands, a_used, a_rot, alt
        if fallback is None and a_cands:
            fallback = (a_cands, a_used, a_rot, alt)

    if fallback is not None:
        return fallback
    return [], setback, False, layer_idx


def _find_open_positions_laddered(center_x, center_y, dir_x, dir_y, layer_idx,
                                  setback, pad_gap_half, label, layer_names,
                                  spacing_mm, config, obstacles,
                                  connector_obstacles, coord, neighbor_stubs,
                                  offset_check=None):
    """_find_open_positions over a ladder of setback radii (issue #90).

    Returns (candidates, used_setback, rotated); `rotated` is True when the
    position was only found by rotating the launch direction away from the
    escape direction (the centerline route must then wrap around the
    endpoint, like a flipped attempt).
    """
    ladder = _setback_ladder(setback, spacing_mm, pad_gap_half, config)
    for i, s in enumerate(ladder):
        candidates = _find_open_positions(
            center_x, center_y, dir_x, dir_y, layer_idx, s, label,
            layer_names, spacing_mm, config, obstacles,
            connector_obstacles, coord, neighbor_stubs,
            quiet_failure=True, offset_check=offset_check)
        if candidates:
            if i > 0:
                print(f"      {label}: setback {ladder[0]:.2f}mm blocked, "
                      f"using {s:.2f}mm")
            return candidates, s, False

    # Full-sweep fallback: the angle fan only covers +-max_setback_angle
    # around the escape direction, but buried terminals (SOT-23 chains,
    # USB-C row-to-row legs) often only open sideways or backwards.
    # Re-run the ladder with the launch direction rotated 90/180/270 deg -
    # together with the fan this covers the whole circle.
    for rot_deg in (90, -90, 180):
        rot = math.radians(rot_deg)
        rdx = dir_x * math.cos(rot) - dir_y * math.sin(rot)
        rdy = dir_x * math.sin(rot) + dir_y * math.cos(rot)
        for s in ladder:
            candidates = _find_open_positions(
                center_x, center_y, rdx, rdy, layer_idx, s, label,
                layer_names, spacing_mm, config, obstacles,
                connector_obstacles, coord, neighbor_stubs,
                quiet_failure=True, offset_check=offset_check)
            if candidates:
                print(f"      {label}: escape direction blocked at every "
                      f"setback - launching rotated {rot_deg:+d}deg at {s:.2f}mm")
                return candidates, s, True

    print(f"  Error: {label} - no valid position at any setback/direction "
          f"(ladder {', '.join(f'{s:.2f}' for s in ladder)}mm x full sweep)")
    return [], setback, False


def _find_open_positions(center_x, center_y, dir_x, dir_y, layer_idx, setback,
                         label, layer_names, spacing_mm, config, obstacles,
                         connector_obstacles, coord, neighbor_stubs,
                         quiet_failure=False, offset_check=None):
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
        # Reject if a per-half offset connector from this launch would clip a
        # partner escape via (the own-net via the obstacle map excludes) -- #165.
        if offset_check is not None and not offset_check(x, y, dx, dy):
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
        if not quiet_failure:
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

    Detection only - whether the mismatch is resolved by a pad swap, an
    opposite-side connector flip, or a skip is decided by the caller.

    Args:
        simplified_path: List of (gx, gy, layer) representing simplified centerline
        coord: GridCoord for coordinate conversion
        p_src_x, p_src_y: P source position in mm
        n_src_x, n_src_y: N source position in mm
        p_tgt_x, p_tgt_y: P target position in mm
        n_tgt_x, n_tgt_y: N target position in mm
        config: Routing config

    Returns:
        (p_sign, polarity_swap_needed, has_layer_change)
        - p_sign: +1 or -1, which perpendicular side P should be offset to
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

    # Always use source polarity
    p_sign = src_p_sign

    return p_sign, polarity_swap_needed, has_layer_change


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


def _routing_copper_layers(pcb_data, config) -> List[str]:
    """Copper layer names for check_drc's pad/via layer-wildcard expansion: the
    board's copper segment layers unioned with the configured routing layers."""
    layers = {s.layer for s in pcb_data.segments if s.layer.endswith('.Cu')}
    layers.update(config.layers)
    return list(layers)


def _rect_ray_exit(hx: float, hy: float, dx: float, dy: float) -> float:
    """Distance from an axis-aligned rectangle's center to its boundary along the
    unit direction (dx, dy). hx, hy are the half-extents."""
    tx = hx / abs(dx) if abs(dx) > 1e-9 else float('inf')
    ty = hy / abs(dy) if abs(dy) > 1e-9 else float('inf')
    return min(tx, ty)


def _ellipse_ray_exit(a: float, b: float, dx: float, dy: float) -> float:
    """Distance from an ellipse center (semi-axes a, b) to its boundary along the
    unit direction (dx, dy). For a circle (a == b) this is just the radius."""
    denom = (dx / a) ** 2 + (dy / b) ** 2 if a > 1e-9 and b > 1e-9 else 0.0
    return 1.0 / math.sqrt(denom) if denom > 1e-12 else float('inf')


def _pad_edge_launch(pcb_data, net_id, cx, cy, route_x, route_y, config, tol=0.05):
    """Issue #165 (tier-2 root-cause 2): move a bare-pad connector launch point
    from the pad CENTER to the pad edge facing the route, so the connector leaves
    the pad toward the route instead of cutting back across the (long) pad field
    - e.g. a 1.45mm USB-C pad whose center-launched connector grazes a neighbour.

    Only fires when (cx, cy) actually sits on a pad of net_id (a bare-pad
    endpoint - its coords ARE the pad center); a stub-tip launch point is away
    from any pad center, so no pad is found and the launch is left untouched
    (the connector must stay attached to the existing stub). Returns
    (x, y, shift): the shifted launch point (clamped to stay inside the pad copper
    and never past the route start) and how far along the route direction it
    moved (0.0 when no pad / no move) -- the caller subtracts that from the
    centerline setback so the setback point stays put instead of being pushed
    past the pad into a downstream blockage (issue #166).
    """
    pad, best = None, tol
    for p in pcb_data.pads_by_net.get(net_id, []):
        d = math.hypot(p.global_x - cx, p.global_y - cy)
        if d <= best:
            best, pad = d, p
    if pad is None:
        return cx, cy, 0.0

    dx, dy = route_x - cx, route_y - cy
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return cx, cy, 0.0
    dx, dy = dx / norm, dy / norm

    # Rotate the direction into the pad's local frame for diagonal pads, matching
    # check_drc's R(-rect_rotation) convention, so the axis-aligned exit applies.
    ldx, ldy = dx, dy
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        cr, sr = math.cos(rad), math.sin(rad)
        ldx, ldy = dx * cr + dy * sr, -dx * sr + dy * cr

    # Round pads use the ellipse boundary so the launch stays on copper (the rect
    # bounding box would put it in a corner outside a circle/oval).
    if pad.shape in ('circle', 'oval'):
        reach = _ellipse_ray_exit(pad.size_x / 2, pad.size_y / 2, ldx, ldy)
    else:
        reach = _rect_ray_exit(pad.size_x / 2, pad.size_y / 2, ldx, ldy)
    if not math.isfinite(reach) or reach <= 0:
        return cx, cy, 0.0

    # Stay just inside the pad edge so the track endpoint lands on pad copper,
    # and never launch past the route's first point (very short connectors).
    margin = min(config.track_width / 2, reach * 0.25)
    shift = min(reach - margin, max(0.0, norm - config.track_width / 2))
    if shift <= 1e-6:
        return cx, cy, 0.0
    return cx + dx * shift, cy + dy * shift, shift


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
    # Use max track width for clearance since track width varies by layer
    max_track_width = config.get_max_track_width()
    gnd_via_perp_mm = spacing_mm + max_track_width/2 + config.clearance + config.via_size/2
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
                layers=["F.Cu", "B.Cu"],  # Always through-hole
                net_id=gnd_net_id,
                free=True
            ))
            gnd_vias.append(Via(
                x=gnd_n_x, y=gnd_n_y,
                size=config.via_size,
                drill=config.via_drill,
                layers=["F.Cu", "B.Cu"],  # Always through-hole
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
                            config, layer_names, omit_connectors=False):
    """Convert floating-point path (x, y, layer) to segments and vias.

    Adds extension segments to ensure P and N connectors are parallel.

    With omit_connectors=True the terminal connector segments (terminal -> route
    start/end) are skipped: only the coupled middle is emitted, leaving clean
    stub free-ends at each end for a later point-to-point single-ended pass to
    join to the terminal (hybrid diff routing -- watchy USB_D).

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

    if omit_connectors:
        # Hybrid mode: emit only the coupled middle; the terminal joins are done
        # single-ended afterwards, so suppress both connector segments.
        original_start = None
        original_end = None

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

            # Use basic track_width for stub connectors to match stub spacing
            first_layer_name = layer_names[first_layer]
            if use_extension:
                # Add extension segment in stub direction
                ext_x = orig_x + src_stub_dir[0] * src_extension
                ext_y = orig_y + src_stub_dir[1] * src_extension

                segs.append(Segment(
                    start_x=orig_x, start_y=orig_y,
                    end_x=ext_x, end_y=ext_y,
                    width=config.track_width,
                    layer=first_layer_name,
                    net_id=net_id
                ))
                # Connector from extension to route
                segs.append(Segment(
                    start_x=ext_x, start_y=ext_y,
                    end_x=first_x, end_y=first_y,
                    width=config.track_width,
                    layer=first_layer_name,
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
                    layer=first_layer_name,
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
                layers=["F.Cu", "B.Cu"],  # Always through-hole
                net_id=net_id
            ))
        elif abs(x1 - x2) > 0.001 or abs(y1 - y2) > 0.001:
            # Always use actual layer for segment
            layer_name = layer_names[layer1]
            segs.append(Segment(
                start_x=x1, start_y=y1,
                end_x=x2, end_y=y2,
                width=config.get_track_width(layer_name),
                layer=layer_name,
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

            # Use basic track_width for stub connectors to match stub spacing
            last_layer_name = layer_names[last_layer]
            if use_extension:
                # Extend along stub direction (toward route area)
                ext_x = orig_x + tgt_stub_dir[0] * tgt_extension
                ext_y = orig_y + tgt_stub_dir[1] * tgt_extension

                # Connector from route to extension point
                segs.append(Segment(
                    start_x=last_x, start_y=last_y,
                    end_x=ext_x, end_y=ext_y,
                    width=config.track_width,
                    layer=last_layer_name,
                    net_id=net_id
                ))
                # Extension back to stub endpoint
                segs.append(Segment(
                    start_x=ext_x, start_y=ext_y,
                    end_x=orig_x, end_y=orig_y,
                    width=config.track_width,
                    layer=last_layer_name,
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
                    layer=last_layer_name,
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
    layer_map = build_layer_map(config.layers)

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

    # Honor get_net_endpoints' source/target split so a multi-point cluster on
    # one end (e.g. a BGA diff-pair escape stub, returned as many clustered
    # points) can't be picked for BOTH terminals - the old "two closest P-N
    # pairs over the merged set" landed both terminals inside that one cluster,
    # collapsing the pair to a degenerate ~0.1mm corridor that downstream then
    # refused. Each terminal is now the closest P-N pair WITHIN one labeled
    # group, so the two terminals stay on opposite ends.
    #
    # P/N agreement: get_net_endpoints labels each net independently, so N's
    # "source" group may be the end P calls "target" (asymmetric copper). Try
    # both ways of matching P's sides to N's sides and keep the one whose two
    # P-N pairs are tighter (each P endpoint actually adjacent to its N) - the
    # wrong matching pairs opposite ends and comes back far apart.
    def labeled_pair(p_grp, n_grp):
        p, n, d = find_closest_pair(p_grp, n_grp)
        return (p, n, d) if p is not None and n is not None else None

    def matching_cost(src_pair, tgt_pair):
        if src_pair is None or tgt_pair is None:
            return None
        return src_pair[2] + tgt_pair[2]  # sum of the two P-N gaps; smaller = P/N agree

    aligned = (labeled_pair(p_sources, n_sources), labeled_pair(p_targets, n_targets))
    crossed = (labeled_pair(p_sources, n_targets), labeled_pair(p_targets, n_sources))
    cost_a, cost_c = matching_cost(*aligned), matching_cost(*crossed)

    chosen = None
    if cost_a is not None and (cost_c is None or cost_a <= cost_c):
        chosen = aligned
    elif cost_c is not None:
        chosen = crossed

    if chosen is not None:
        (p_src, n_src, _), (p_tgt, n_tgt, _) = chosen
    else:
        # Labels unusable (e.g. a net has no separable target group): fall back
        # to the original two-closest-pairs search over the merged endpoint set.
        p_src, n_src, src_dist = find_closest_pair(p_all, n_all)
        if p_src is None or n_src is None:
            return [], [], "Could not find matching P and N endpoints on same layer"
        p_remaining = [p for p in p_all if p != p_src]
        n_remaining = [n for n in n_all if n != n_src]
        if not p_remaining or not n_remaining:
            return [], [], "Not enough endpoints for source and target"
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
    """Compute connector region parameters for a differential pair (see by-ids variant)."""
    return get_diff_pair_connector_regions_by_ids(
        pcb_data, diff_pair.p_net_id, diff_pair.n_net_id, config)


def get_diff_pair_connector_regions_by_ids(pcb_data: PCBData, p_net_id: int, n_net_id: int,
                                            config: GridRouteConfig,
                                            endpoints: Optional[Tuple] = None) -> Optional[dict]:
    """
    Compute connector region parameters for a differential pair.

    endpoints: optional (source, target) endpoint tuples to compute regions for
    a specific multi-point leg instead of the closest P/N endpoint pair.

    Returns dict with:
        - src_center: (x, y) center between P and N source stubs
        - src_dir: (dx, dy) normalized direction from stubs toward route
        - src_dir_synthesized: True if src_dir was derived from pad geometry
          (bare-pad endpoint); the route may use either +src_dir or -src_dir
        - src_setback: distance from stub center to route start
        - tgt_center: (x, y) center between P and N target stubs
        - tgt_dir: (dx, dy) normalized direction from stubs toward route
        - tgt_dir_synthesized: True if tgt_dir was derived from pad geometry
        - tgt_setback: distance from stub center to route start
        - spacing_mm: half-spacing between P and N tracks

    Returns None if endpoints cannot be determined.
    """
    # Find endpoints (or use explicit leg endpoints)
    if endpoints is not None:
        sources, targets = [endpoints[0]], [endpoints[1]]
    else:
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

    # Calculate centerline midpoints
    center_src_x = (p_src_x + n_src_x) / 2
    center_src_y = (p_src_y + n_src_y) / 2
    center_tgt_x = (p_tgt_x + n_tgt_x) / 2
    center_tgt_y = (p_tgt_y + n_tgt_y) / 2

    # Get escape directions (averaged stub directions, or synthesized from pad
    # geometry for bare-pad endpoints) - must match _try_route_direction
    src_dir_x, src_dir_y, src_dir_synth = get_pair_end_direction(
        pcb_data, p_net_id, n_net_id, p_segments, n_segments,
        p_src_x, p_src_y, n_src_x, n_src_y, src_layer_name,
        other_center=(center_tgt_x, center_tgt_y))
    tgt_dir_x, tgt_dir_y, tgt_dir_synth = get_pair_end_direction(
        pcb_data, p_net_id, n_net_id, p_segments, n_segments,
        p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y, tgt_layer_name,
        other_center=(center_src_x, center_src_y))

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
        'src_dir_synthesized': src_dir_synth,
        'src_setback': src_setback,
        'tgt_center': (center_tgt_x, center_tgt_y),
        'tgt_dir': (tgt_dir_x, tgt_dir_y),
        'tgt_dir_synthesized': tgt_dir_synth,
        'tgt_setback': tgt_setback,
        'spacing_mm': spacing_mm,
    }


def _try_route_direction(src, tgt, pcb_data, config, obstacles, base_obstacles,
                         coord, layer_names, spacing_mm, p_net_id, n_net_id,
                         max_iterations_override=None, neighbor_stubs=None,
                         preferred_angles=None, direction_label=None, is_backward=False,
                         prox_h_cost=0, flip_source=False, flip_target=False,
                         forced_source_dir=None, forced_target_dir=None):
    """
    Attempt to route a diff pair in one direction.

    Args:
        preferred_angles: Optional (src_candidate, tgt_candidate) to use specific angles
                         from a previous probe instead of trying all combinations.
        flip_source/flip_target: Force the escape direction at the physical
                         source/target end to the opposite side (only valid for
                         bare-pad endpoints with synthesized directions). Used to
                         resolve polarity mismatches geometrically when polarity
                         fixing is disabled.
        forced_source_dir/forced_target_dir: Explicit (dx, dy) escape direction
                         for the physical source/target end, overriding stub or
                         synthesized directions. Used by multi-point diff pair
                         routing to continue a chain out the opposite side of a
                         terminal already used by a previous leg. A forced end
                         is never auto-flipped.

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
    # Use max track width for clearance since via connects layers with potentially different widths
    max_track_width = config.get_max_track_width()
    track_via_clearance = (config.clearance + max_track_width / 2 + config.via_size / 2) * config.routing_clearance_margin
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

    # Calculate centerline midpoints between P and N stubs
    center_src_x = (p_src_x + n_src_x) / 2
    center_src_y = (p_src_y + n_src_y) / 2
    center_tgt_x = (p_tgt_x + n_tgt_x) / 2
    center_tgt_y = (p_tgt_y + n_tgt_y) / 2

    # Degenerate pair: both terminal centers (nearly) coincide, so no pair
    # corridor exists - the centerline can only leave and loop back to its
    # own start, and the offset/connector geometry necessarily tangles
    # (neo6502 /D_P //D_N: USB-C row-join nets whose A and B pad pairs share
    # one midpoint). Refuse pair routing with a clear message; these nets
    # are jumpers and should be routed single-ended.
    center_dist = math.hypot(center_tgt_x - center_src_x, center_tgt_y - center_src_y)
    if center_dist < 2 * spacing_mm + config.grid_step:
        print(f"  Pair terminals nearly coincide (centers {center_dist:.2f}mm apart "
              f"< pair width {2 * spacing_mm:.2f}mm) - no pair corridor exists. "
              f"Route these nets single-ended (connector row-join).")
        return None, 0, [], None

    # Get escape directions at source and target (averaged stub directions, or
    # synthesized from pad geometry when the endpoints are bare pads)
    src_dir_x, src_dir_y, src_dir_synth = get_pair_end_direction(
        pcb_data, p_net_id, n_net_id, p_segments, n_segments,
        p_src_x, p_src_y, n_src_x, n_src_y, src_layer_name,
        other_center=(center_tgt_x, center_tgt_y))
    tgt_dir_x, tgt_dir_y, tgt_dir_synth = get_pair_end_direction(
        pcb_data, p_net_id, n_net_id, p_segments, n_segments,
        p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y, tgt_layer_name,
        other_center=(center_src_x, center_src_y))

    # Apply forced directions (physical source/target; this call's src/tgt are
    # swapped when routing backward). Forced ends are not synthesized and are
    # never auto-flipped.
    src_forced = forced_target_dir if is_backward else forced_source_dir
    tgt_forced = forced_source_dir if is_backward else forced_target_dir
    if src_forced is not None:
        src_dir_x, src_dir_y = src_forced
        src_dir_synth = False
    if tgt_forced is not None:
        tgt_dir_x, tgt_dir_y = tgt_forced
        tgt_dir_synth = False

    # Apply forced direction flips (physical source/target; this call's src/tgt
    # are swapped when routing backward). Only bare-pad (synthesized) ends can
    # flip - stub directions are fixed by existing copper.
    src_flip = flip_target if is_backward else flip_source
    tgt_flip = flip_source if is_backward else flip_target
    if src_flip:
        if not src_dir_synth:
            return None, 0, [], None
        src_dir_x, src_dir_y = -src_dir_x, -src_dir_y
    if tgt_flip:
        if not tgt_dir_synth:
            return None, 0, [], None
        tgt_dir_x, tgt_dir_y = -tgt_dir_x, -tgt_dir_y

    # Calculate setback distance (default to 4x spacing = 2x total P-N distance).
    # Computed BEFORE the pad-edge launch so the launch shift can be subtracted
    # from it (issue #166).
    if config.diff_pair_centerline_setback is not None:
        setback = config.diff_pair_centerline_setback
    else:
        setback = spacing_mm * 4  # Default: 4x the P-N half-spacing = 2x total P-N distance

    # Issue #165 root-cause 2 (routing side): for a long pad (USB-C, 1.45mm) the
    # terminal CENTER sits deep inside the pad, so the centerline setback point
    # lands within the pad's own copper (added as an obstacle) and the route can't
    # reach it. Launch the routing terminal from the pad EDGE nearest the escape
    # direction, so the setback sits in open space just past the pad. No-op for
    # stub/small terminals (their launch point isn't a pad center, or the edge ~
    # the center).
    _FAR = 10.0
    p_src_x, p_src_y, p_src_shift = _pad_edge_launch(pcb_data, p_net_id, p_src_x, p_src_y,
                                        p_src_x + src_dir_x * _FAR, p_src_y + src_dir_y * _FAR, config)
    n_src_x, n_src_y, n_src_shift = _pad_edge_launch(pcb_data, n_net_id, n_src_x, n_src_y,
                                        n_src_x + src_dir_x * _FAR, n_src_y + src_dir_y * _FAR, config)
    p_tgt_x, p_tgt_y, p_tgt_shift = _pad_edge_launch(pcb_data, p_net_id, p_tgt_x, p_tgt_y,
                                        p_tgt_x + tgt_dir_x * _FAR, p_tgt_y + tgt_dir_y * _FAR, config)
    n_tgt_x, n_tgt_y, n_tgt_shift = _pad_edge_launch(pcb_data, n_net_id, n_tgt_x, n_tgt_y,
                                        n_tgt_x + tgt_dir_x * _FAR, n_tgt_y + tgt_dir_y * _FAR, config)
    center_src_x, center_src_y = (p_src_x + n_src_x) / 2, (p_src_y + n_src_y) / 2
    center_tgt_x, center_tgt_y = (p_tgt_x + n_tgt_x) / 2, (p_tgt_y + n_tgt_y) / 2

    # The launch moved toward the pad edge; pull the centerline setback in by the
    # same (averaged) amount so the setback POINT stays where a centre launch
    # would have put it, instead of being pushed a full pad-length further out
    # into a downstream blockage (issue #166 esp_prog). Floored so the setback
    # still clears the pad edge it launched from.
    _setback_floor = config.track_width / 2 + config.clearance
    setback_src = max(_setback_floor, setback - (p_src_shift + n_src_shift) / 2)
    setback_tgt = max(_setback_floor, setback - (p_tgt_shift + n_tgt_shift) / 2)

    if direction_label and config.verbose:
        print(f"  Probing {direction_label}...")

    # Use base_obstacles for connector clearance checks if available
    # (connectors are single tracks that don't need diff pair extra clearance)
    connector_obstacles = base_obstacles if base_obstacles is not None else obstacles

    # Allow radius for finding open positions near blocked cells
    allow_radius = 2

    # Half pad separation per terminal, for the ladder's geometric floor
    src_pad_gap_half = math.hypot(p_src_x - n_src_x, p_src_y - n_src_y) / 2
    tgt_pad_gap_half = math.hypot(p_tgt_x - n_tgt_x, p_tgt_y - n_tgt_y) / 2

    # Alternate launch layers reachable through each terminal's via/THT barrel,
    # common to both P and N so the pair can launch coupled there (issue #195).
    # Empty when the terminal isn't via-backed -> multilayer search degenerates
    # to the single-layer ladder (no behaviour change).
    src_alt_layers = sorted(
        (_endpoint_launch_layer_indices(pcb_data, p_net_id, p_src_x, p_src_y, config) &
         _endpoint_launch_layer_indices(pcb_data, n_net_id, n_src_x, n_src_y, config))
        - {src_layer})
    tgt_alt_layers = sorted(
        (_endpoint_launch_layer_indices(pcb_data, p_net_id, p_tgt_x, p_tgt_y, config) &
         _endpoint_launch_layer_indices(pcb_data, n_net_id, n_tgt_x, n_tgt_y, config))
        - {tgt_layer})

    # Reject setback candidates whose per-half offset connector would clip the
    # PARTNER terminal's escape via -- the own-net via the obstacle map excludes,
    # which #165 would otherwise catch only after a full route (watchy USB_D).
    src_offset_check = _make_offset_connector_check(
        (p_src_x, p_src_y), (n_src_x, n_src_y),
        _terminal_escape_vias(pcb_data, p_net_id, n_net_id,
                              (p_src_x, p_src_y), (n_src_x, n_src_y), config),
        spacing_mm, config)
    tgt_offset_check = _make_offset_connector_check(
        (p_tgt_x, p_tgt_y), (n_tgt_x, n_tgt_y),
        _terminal_escape_vias(pcb_data, p_net_id, n_net_id,
                              (p_tgt_x, p_tgt_y), (n_tgt_x, n_tgt_y), config),
        spacing_mm, config)

    # Get all valid setback positions for source and target, sorted by
    # preference, scanning a ladder of radii per terminal (issue #90), free to
    # switch to an open inner layer through the endpoint via (issue #195)
    src_candidates, _, src_rotated, src_layer = _find_open_positions_multilayer(
        center_src_x, center_src_y, src_dir_x, src_dir_y, src_layer, src_alt_layers,
        setback_src, src_pad_gap_half, "source",
        layer_names, spacing_mm, config, obstacles, connector_obstacles, coord, neighbor_stubs,
        offset_check=src_offset_check
    )
    if not src_candidates and src_dir_synth and not src_flip:
        # Synthesized direction blocked - try escaping from the opposite side
        src_dir_x, src_dir_y = -src_dir_x, -src_dir_y
        src_candidates, _, src_rotated, src_layer = _find_open_positions_multilayer(
            center_src_x, center_src_y, src_dir_x, src_dir_y, src_layer, src_alt_layers,
            setback_src, src_pad_gap_half, "source",
            layer_names, spacing_mm, config, obstacles, connector_obstacles, coord, neighbor_stubs,
            offset_check=src_offset_check
        )
    if not src_candidates:
        blocked = _collect_setback_blocked_cells(
            center_src_x, center_src_y, src_dir_x, src_dir_y, src_layer, setback_src,
            config, obstacles, connector_obstacles, coord
        )
        return None, 0, blocked, None
    src_layer_name = layer_names[src_layer]

    tgt_candidates, _, tgt_rotated, tgt_layer = _find_open_positions_multilayer(
        center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y, tgt_layer, tgt_alt_layers,
        setback_tgt, tgt_pad_gap_half, "target",
        layer_names, spacing_mm, config, obstacles, connector_obstacles, coord, neighbor_stubs,
        offset_check=tgt_offset_check
    )
    if not tgt_candidates and tgt_dir_synth and not tgt_flip:
        # Synthesized direction blocked - try escaping from the opposite side
        tgt_dir_x, tgt_dir_y = -tgt_dir_x, -tgt_dir_y
        tgt_candidates, _, tgt_rotated, tgt_layer = _find_open_positions_multilayer(
            center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y, tgt_layer, tgt_alt_layers,
            setback_tgt, tgt_pad_gap_half, "target",
            layer_names, spacing_mm, config, obstacles, connector_obstacles, coord, neighbor_stubs,
            offset_check=tgt_offset_check
        )
    if not tgt_candidates:
        blocked = _collect_setback_blocked_cells(
            center_tgt_x, center_tgt_y, tgt_dir_x, tgt_dir_y, tgt_layer, setback_tgt,
            config, obstacles, connector_obstacles, coord
        )
        return None, 0, blocked, None
    tgt_layer_name = layer_names[tgt_layer]

    # Calculate turning radius in grid units
    min_radius_grid = config.min_turning_radius / config.grid_step

    # Calculate turn cost: arc length for 45° turn = r * π/4, scaled by 1000
    turn_cost = int(min_radius_grid * math.pi / 4 * 1000)

    # Create pose-based router for centerline
    # Double via cost since diff pairs place two vias per layer change
    # via_proximity_cost is a multiplier on via cost in stub/BGA proximity zones (0 = block vias)
    # diff_pair_spacing is the P/N offset from centerline in grid units (for self-intersection prevention)
    # Use 2*spacing to prevent P/N tracks from crossing when centerline loops.
    # Flipped (opposite-side) attempts wrap around their endpoints, so give the
    # centerline extra self-clearance to keep the offset tracks from crossing
    # in the hairpin.
    # Flipped or forced-direction ends (multi-point chain continuations) may
    # need to wrap around their endpoint to approach from the far side
    wrapping = (src_flip or tgt_flip or src_rotated or tgt_rotated or
                src_forced is not None or tgt_forced is not None)
    spacing_multiplier = 3 if wrapping else 2
    diff_pair_spacing_grid = max(1, int(spacing_multiplier * spacing_mm / config.grid_step + 0.5))
    # Convert max_turn_angle from degrees to 45° units
    max_turn_units = int(config.max_turn_angle / 45.0 + 0.5)
    if wrapping:
        # Allow a full loop of cumulative turn so the smooth wrap is reachable
        # (the P/N crossing check rejects bad geometry)
        max_turn_units = max(max_turn_units, 8)

    # Calculate GND via spacing if enabled
    # GND via center = P/N track outer edge + clearance + via_radius
    # Use max track width for clearance since track width varies by layer
    gnd_via_perp_grid = 0
    gnd_via_along_grid = 0
    if config.gnd_via_enabled:
        gnd_via_perp_mm = spacing_mm + max_track_width/2 + config.clearance + config.via_size/2
        via_via_dist_mm = config.via_size + config.clearance
        gnd_via_perp_grid = coord.to_grid_dist(gnd_via_perp_mm)
        gnd_via_along_grid = coord.to_grid_dist(via_via_dist_mm)

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

    # prox_h_cost is now passed as a parameter (checked before endpoint exemptions are set)

    pose_kwargs = dict(
        via_cost=config.via_cost_units() * 2,
        h_weight=config.heuristic_weight,
        turn_cost=turn_cost,
        min_radius_grid=min_radius_grid,
        via_proximity_cost=int(config.via_proximity_cost),
        diff_pair_spacing=diff_pair_spacing_grid,
        max_turn_units=max_turn_units,
        gnd_via_perp_offset=gnd_via_perp_grid,
        gnd_via_along_offset=gnd_via_along_grid,
        vertical_attraction_radius=attraction_radius_grid,
        vertical_attraction_bonus=attraction_bonus,
        proximity_heuristic_cost=prox_h_cost,
    )
    # Per-layer bias (issue #193). Only pass layer_costs when they're non-uniform:
    # a default (all 1.0x) run then omits the kwarg, so it still works on an older
    # grid_router binary that predates PoseRouter layer-cost support (build_router.py
    # installs a prebuilt by default). 1000 == 1.0x in the int cost units.
    _layer_costs = config.get_layer_costs()
    if any(c != 1000 for c in _layer_costs):
        pose_kwargs['layer_costs'] = _layer_costs
    pose_router = PoseRouter(**pose_kwargs)

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
        rd = _build_route_data(pose_path, src_actual_dir, tgt_actual_dir, src_angle, tgt_angle, gnd_via_dirs)
        # Record which physical ends have synthesized (flippable) directions,
        # for the polarity flip retry in route_diff_pair_with_obstacles, and
        # the base escape directions actually used (after any flips), for
        # multi-point chain side tracking
        if is_backward:
            rd['source_dir_synthesized'] = tgt_dir_synth
            rd['target_dir_synthesized'] = src_dir_synth
            rd['source_base_dir'] = (tgt_dir_x, tgt_dir_y)
            rd['target_base_dir'] = (src_dir_x, src_dir_y)
        else:
            rd['source_dir_synthesized'] = src_dir_synth
            rd['target_dir_synthesized'] = tgt_dir_synth
            rd['source_base_dir'] = (src_dir_x, src_dir_y)
            rd['target_base_dir'] = (tgt_dir_x, tgt_dir_y)
        return rd

    def _build_route_data(pose_path, src_actual_dir, tgt_actual_dir, src_angle, tgt_angle, gnd_via_dirs=None):
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


def _routed_length(result: dict) -> float:
    """Total length of a route result's new segments, for comparing polarity
    resolution candidates."""
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
               for s in result.get('new_segments', []))


def _pn_tracks_cross(new_segments, p_net_id: int, n_net_id: int) -> bool:
    """Check whether any P segment crosses (or touches) any N segment on the
    same layer. Used to validate flipped-polarity retry routes, which can wrap
    around far enough that the offset tracks cross."""
    p_segs = [s for s in new_segments if s.net_id == p_net_id]
    n_segs = [s for s in new_segments if s.net_id == n_net_id]
    for ps in p_segs:
        for ns in n_segs:
            if ps.layer != ns.layer:
                continue
            if segments_intersect_tuple((ps.start_x, ps.start_y), (ps.end_x, ps.end_y),
                                        (ns.start_x, ns.start_y), (ns.end_x, ns.end_y)):
                return True
    return False


def _pn_tracks_cross_full(new_segments, pcb_data, p_net_id, n_net_id,
                          p_swap_positions=None, n_swap_positions=None):
    """Crossing check over the pair's FULL copper: the route's new segments PLUS
    the existing pcb_data segments it connects to (source escape stubs and the
    synthesized bare-pad target stubs). A bare-pad target swap injects P/N stubs
    into pcb_data before routing, so a crossing between the route and one of those
    stubs is invisible to new_segments alone (issue #142).

    When p_swap_positions / n_swap_positions are given, segments touching those
    positions are evaluated with P/N EXCHANGED - i.e. as the PENDING polarity swap
    (apply_polarity_swap, run later by the caller) will leave them. That lets a
    swap route's post-relabel self-crossing be detected here, before the swap is
    physically applied to the segments."""
    seg_ids = {id(s) for s in new_segments}
    segs = list(new_segments) + [s for s in pcb_data.segments
                                 if s.net_id in (p_net_id, n_net_id) and id(s) not in seg_ids]
    pp = p_swap_positions or set()
    nn = n_swap_positions or set()

    def eff_net(s):
        sk = pos_key(s.start_x, s.start_y)
        ek = pos_key(s.end_x, s.end_y)
        if s.net_id == p_net_id and (sk in pp or ek in pp):
            return n_net_id
        if s.net_id == n_net_id and (sk in nn or ek in nn):
            return p_net_id
        return s.net_id

    p_segs = [s for s in segs if eff_net(s) == p_net_id]
    n_segs = [s for s in segs if eff_net(s) == n_net_id]
    for ps in p_segs:
        for ns in n_segs:
            if ps.layer != ns.layer:
                continue
            if segments_intersect_tuple((ps.start_x, ps.start_y), (ps.end_x, ps.end_y),
                                        (ns.start_x, ns.start_y), (ns.end_x, ns.end_y)):
                return True
    return False


# Tolerance fraction matching check_drc's default --clearance-margin, so a graze
# reject here is consistent with the DRC checker (won't false-reject a route that
# DRC passes, won't pass one DRC would flag). See issue #165.
_DRC_CLEARANCE_MARGIN = 0.05


def _connector_grazes_foreign_copper(new_segments, pcb_data, p_net_id, n_net_id, config):
    """Issue #165: the pair's connector/setback segments are clearance-checked
    during routing against an obstacle map that excludes BOTH halves of the pair,
    so a connector can graze the PARTNER net's pad/via (or other foreign copper)
    completely unseen (e.g. a USB-C P connector cutting across the adjacent N pad,
    or grazing the partner's underpad escape via).

    Re-validate the final P/N geometry against every foreign pad, via AND track (any
    not on this pair's own nets, which includes the partner net's copper) using the
    SAME geometry and tolerance as check_drc, so a graze reported here is one
    check_drc would also flag - not tight-but-legal packing sitting at clearance.

    The TRACK check (issue #246) matters because the pose router only validates the
    diff-pair CENTERLINE cell during routing -- the P/N OFFSET tracks are generated
    geometrically (create_parallel_path_from_float) and the standard connector/setback
    is centerline-only -- so an offset or a hybrid leg can graze or CROSS an already-
    committed foreign track unseen (D2's offset across D3's committed leg). A foreign
    TRACK excludes BOTH pair nets (the pair's own P/N legitimately run at diff_pair_gap,
    tighter than clearance); the pad/via checks instead exclude only the segment's own
    net so the PARTNER's pad/via is still caught (#165/#241).

    Returns (kind, obj, segment, overlap_mm) where kind is 'pad', 'via' or 'track',
    or None.
    """
    from check_drc import check_pad_segment_overlap, check_via_segment_overlap

    own = (p_net_id, n_net_id)
    pair_segs = [s for s in new_segments if s.net_id in own]
    if not pair_segs:
        return None

    routing_layers = _routing_copper_layers(pcb_data, config)
    clearance = config.clearance
    pads_by_net = getattr(pcb_data, 'pads_by_net', None) or {}
    vias = getattr(pcb_data, 'vias', None) or []
    # Foreign tracks (not on either pair net), bucketed by layer for a cheap prefilter.
    foreign_tracks_by_layer = {}
    for o in (getattr(pcb_data, 'segments', None) or []):
        if o.net_id in own:
            continue
        foreign_tracks_by_layer.setdefault(o.layer, []).append(o)

    for seg in pair_segs:
        sxmin, sxmax = min(seg.start_x, seg.end_x), max(seg.start_x, seg.end_x)
        symin, symax = min(seg.start_y, seg.end_y), max(seg.start_y, seg.end_y)
        seg_margin = clearance + seg.width / 2
        # vs foreign pads (includes the partner net's pads)
        for pad_net, pads in pads_by_net.items():
            if pad_net == seg.net_id:
                continue  # the connector legitimately lands on its own-net pad
            for pad in pads:
                # Coarse bounding-box reject before the exact rect-distance test.
                pm = seg_margin + max(pad.size_x, pad.size_y) / 2
                if (pad.global_x < sxmin - pm or pad.global_x > sxmax + pm or
                        pad.global_y < symin - pm or pad.global_y > symax + pm):
                    continue
                hit, overlap, _ = check_pad_segment_overlap(
                    pad, seg, clearance, routing_layers, _DRC_CLEARANCE_MARGIN)
                if hit:
                    return ('pad', pad, seg, overlap)
        # vs foreign vias (includes the partner net's escape/underpad vias)
        for via in vias:
            if via.net_id == seg.net_id:
                continue  # the connector legitimately lands on its own-net via
            vm = seg_margin + via.size / 2
            if (via.x < sxmin - vm or via.x > sxmax + vm or
                    via.y < symin - vm or via.y > symax + vm):
                continue
            hit, overlap = check_via_segment_overlap(
                via, seg, clearance, _DRC_CLEARANCE_MARGIN)
            if hit:
                return ('via', via, seg, overlap)
        # vs foreign tracks on the same layer (issue #246)
        for o in foreign_tracks_by_layer.get(seg.layer, ()):
            ow = o.width if getattr(o, 'width', 0) else seg.width
            need = clearance + seg.width / 2 + ow / 2 - _DRC_CLEARANCE_MARGIN
            if need <= 0:
                continue
            if (max(o.start_x, o.end_x) < sxmin - need or
                    min(o.start_x, o.end_x) > sxmax + need or
                    max(o.start_y, o.end_y) < symin - need or
                    min(o.start_y, o.end_y) > symax + need):
                continue
            d = _seg_seg_distance(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                                  o.start_x, o.start_y, o.end_x, o.end_y)
            if d < need:
                return ('track', o, seg, need - d)
    return None


def _pad_obstacle_segments(pad, layer_names):
    """Represent a pad as capsule segment(s) -- one per copper layer -- so it can be
    fed to a leg's TRACK-obstacle list (add_segments_list_as_obstacles). A coupled
    hybrid's single-ended leg must keep clearance from a PARTNER-net pad, not just
    keep its via off it (add_pads_via_keepout is via-only), or the leg track grazes
    the partner pad (keks /CK_N leg under the /CK_P pad R41.2). The capsule runs
    along the pad's long axis, width = its short side, approximating the rounded
    rect the same way a track segment is blocked."""
    cx, cy = pad.global_x, pad.global_y
    sx, sy = pad.size_x, pad.size_y
    # Segment spans the FULL long axis (+-long/2): a capsule of that length and
    # width=short side covers the entire pad rectangle (corners included) -- using
    # length = long-short would round the corners and leave a ~0.1*short graze
    # (keks /CK_N still 0.046mm into R41.2's corner). The semicircular caps bulge
    # short/2 past the pad ends, a small, harmless over-block.
    if sx >= sy:
        half = sx / 2.0
        a = (cx - half, cy, cx + half, cy, sy)
    else:
        half = sy / 2.0
        a = (cx, cy - half, cx, cy + half, sx)
    cu = [L for L in (pad.layers or []) if L.endswith('.Cu')]
    if (getattr(pad, 'drill', 0) or 0) > 0 or not cu:
        cu = list(layer_names)  # THT / unknown copper -> block every routing layer
    else:
        cu = [L for L in cu if L in layer_names]
    return [Segment(start_x=a[0], start_y=a[1], end_x=a[2], end_y=a[3],
                    width=a[4], layer=L, net_id=pad.net_id) for L in cu]


def _route_direct_coupled_middle(pcb_data, diff_pair, config, obstacles, layer_names,
                                 leg_obstacles=None, endpoints=None):
    """Hybrid coupled middle (last resort): route a clean parallel P/N pair
    straight from the source via-midpoint to the target via-midpoint on the
    open layer -- no escape-direction setback, no connectors, no polarity stage.

    Both terminals carry through-vias (or THT pads), so the centerline can start
    and end as close as possible to each terminal on whichever spanned layer is
    open. Each terminal is then attached to its middle near-end with a
    point-to-point single-ended leg (_route_hybrid_legs), which routes around the
    partner copper -- that is where polarity is resolved, at the pads. Returns a
    complete (fully-connected) route dict, or None if no clean route was found.
    """
    if PoseRouter is None:
        return None
    coord = GridCoord(config.grid_step)
    p_net_id, n_net_id = diff_pair.p_net_id, diff_pair.n_net_id
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2

    if endpoints is not None:
        src, tgt = endpoints
    else:
        srcs, tgts, err = get_diff_pair_endpoints(pcb_data, p_net_id, n_net_id, config)
        if err or not srcs or not tgts:
            return None
        src, tgt = srcs[0], tgts[0]
    p_src_x, p_src_y, n_src_x, n_src_y = src[5], src[6], src[7], src[8]
    p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y = tgt[5], tgt[6], tgt[7], tgt[8]
    csx, csy = (p_src_x + n_src_x) / 2, (p_src_y + n_src_y) / 2
    ctx, cty = (p_tgt_x + n_tgt_x) / 2, (p_tgt_y + n_tgt_y) / 2

    # Terminal ESCAPE directions (issue #244): the pose A* must launch/arrive at each
    # terminal along the heading the stub/pad actually points, exactly as the standard
    # route's try_route does (s_theta=dir(src_escape), t_theta=dir(-tgt_escape)).
    # Feeding the straight src->tgt line instead makes the coupled A* dead-end on a
    # terminal whose fingers point ACROSS the route (J4's vertical connector pads ->
    # arriving horizontally burns the full iter cap). Computed once here from terminal
    # geometry; the dot-guard below applies them only when they point toward the route.
    nL = len(config.layers)

    def _safe_layer_name(idx):
        return layer_names[idx] if isinstance(idx, int) and 0 <= idx < nL else layer_names[0]
    _p_segs_all = [s for s in (getattr(pcb_data, 'segments', None) or []) if s.net_id == p_net_id]
    _n_segs_all = [s for s in (getattr(pcb_data, 'segments', None) or []) if s.net_id == n_net_id]
    src_ex, src_ey, _ = get_pair_end_direction(
        pcb_data, p_net_id, n_net_id, _p_segs_all, _n_segs_all,
        p_src_x, p_src_y, n_src_x, n_src_y, _safe_layer_name(src[4]),
        other_center=(ctx, cty))
    tgt_ex, tgt_ey, _ = get_pair_end_direction(
        pcb_data, p_net_id, n_net_id, _p_segs_all, _n_segs_all,
        p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y, _safe_layer_name(tgt[4]),
        other_center=(csx, csy))

    # The coupled middle may run on ANY routing layer: each terminal leg is a
    # single-ended A* from the pad's layer to the middle, and it drops a via for
    # the layer change itself -- so an inner-layer middle is reachable even when
    # no via exists yet (stock surface fanout). Prefer the layers an existing
    # via/THT barrel already spans (no extra escape via needed), then inner
    # layers (most likely open under the parts), then F.Cu/B.Cu.
    spanned = ((_endpoint_launch_layer_indices(pcb_data, p_net_id, p_src_x, p_src_y, config) &
                _endpoint_launch_layer_indices(pcb_data, n_net_id, n_src_x, n_src_y, config)) &
               (_endpoint_launch_layer_indices(pcb_data, p_net_id, p_tgt_x, p_tgt_y, config) &
                _endpoint_launch_layer_indices(pcb_data, n_net_id, n_tgt_x, n_tgt_y, config)))
    # TODO1 layer scheme (issue #243): try each terminal's OWN layer first so the
    # coupled middle stays where the terminal already sits -- a terminal carrying an
    # all-layer via-in-pad (e.g. a bare-pad target swap) is then reached with NO
    # extra transition via, instead of detouring to an inner layer and dropping a
    # gratuitous via that can block a neighbour (eis DDMI_D1 -> blocked CK). Order
    # of (start_layer, end_layer) candidates, each a FULL attempt (middle + legs),
    # first that fully routes wins:
    #   1. source layer the whole way
    #   2. target layer the whole way
    #   3. source -> target (one coupled layer change mid-middle; A* drops the via)
    #   4. every remaining single layer (old spanned/inner/outer order)
    # (Shared by the 2-terminal hybrid and each multipoint-leg hybrid. A full
    # remaining-cross-combo sweep is deliberately omitted -- it is nL^2 middle A*
    # runs on already-hard pairs; #3 covers the one useful cross case.)
    #
    # GATE (issue #244): front-load a terminal's own layer ONLY when that terminal
    # actually carries an own-net via (a bare-pad target swap / hand via-in-pad) --
    # the case the scheme was built for, where own-layer coupling saves a transition
    # via (butterstick CK1 on B.Cu). For a plain SURFACE connector terminal with no
    # such via (butterstick GIGABIT ETH_D on F.Cu), front-loading the own layer does
    # NOT save a via and just keeps the coupled pair on the congested escape layer,
    # which boxed out neighbours' reroutes (ETH_B/D1 failed only under --layer-costs,
    # which the #243 stress -- run without layer-costs -- never exercised). Without a
    # terminal via, fall back to the congestion-ordered single_order (pre-#243).
    nL = len(config.layers)
    s_lyr = src[4] if isinstance(src[4], int) and 0 <= src[4] < nL else None
    t_lyr = tgt[4] if isinstance(tgt[4], int) and 0 <= tgt[4] < nL else None
    single_order = sorted(
        range(nL),
        key=lambda i: (i not in spanned, layer_names[i] in ('F.Cu', 'B.Cu'), i))

    _assoc = _launch_assoc_tol(config)

    def _terminal_has_via(px, py, nx, ny):
        """True if an own-net (P or N) via sits at this terminal -- a via-in-pad /
        bare-pad target swap, where coupling on the terminal's own layer avoids an
        extra transition via (the only case worth front-loading the own layer)."""
        for v in (getattr(pcb_data, 'vias', None) or []):
            if v.net_id not in (p_net_id, n_net_id):
                continue
            if (math.hypot(v.x - px, v.y - py) <= _assoc or
                    math.hypot(v.x - nx, v.y - ny) <= _assoc):
                return True
        return False
    src_has_via = _terminal_has_via(p_src_x, p_src_y, n_src_x, n_src_y)
    tgt_has_via = _terminal_has_via(p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y)
    layer_pairs = []

    def _add_pair(a, b):
        if a is not None and b is not None and (a, b) not in layer_pairs:
            layer_pairs.append((a, b))
    if src_has_via:
        _add_pair(s_lyr, s_lyr)         # 1. source layer the whole way
    if tgt_has_via:
        _add_pair(t_lyr, t_lyr)         # 2. target layer the whole way
    if src_has_via or tgt_has_via:
        _add_pair(s_lyr, t_lyr)         # 3. source -> target (one coupled via)
    for _L in single_order:             # 4. every remaining single layer
        _add_pair(_L, _L)

    min_radius_grid = config.min_turning_radius / config.grid_step
    turn_cost = int(min_radius_grid * math.pi / 4 * 1000)
    diff_pair_spacing_grid = max(1, int(2 * spacing_mm / config.grid_step + 0.5))
    max_turn_units = max(int(config.max_turn_angle / 45.0 + 0.5), 1)
    pose_kwargs = dict(
        via_cost=config.via_cost_units() * 2, h_weight=config.heuristic_weight,
        turn_cost=turn_cost, min_radius_grid=min_radius_grid,
        via_proximity_cost=int(config.via_proximity_cost),
        diff_pair_spacing=diff_pair_spacing_grid, max_turn_units=max_turn_units)
    _lc = config.get_layer_costs()
    if any(c != 1000 for c in _lc):
        pose_kwargs['layer_costs'] = _lc
    pr = PoseRouter(**pose_kwargs)
    via_spacing_grid = max(1, int(max(spacing_mm, (config.via_size + config.clearance) / 2)
                                  / config.grid_step + 0.5))
    max_iters = config.max_iterations * 8

    s_gx, s_gy = coord.to_grid(csx, csy)
    t_gx, t_gy = coord.to_grid(ctx, cty)

    # Couple as far as possible: if an endpoint centre is unreachable on EVERY
    # layer (a multipoint terminal whose P/N fanned out far apart, so their
    # midpoint is buried inside the part), shorten the coupled middle toward the
    # other end until it reaches open copper. The single-ended legs span the rest.
    _nlay = len(config.layers)

    def _relax(gx, gy, ox, oy):
        steps = max(int(math.hypot(ox - gx, oy - gy)), 1)
        for k in range(1, steps + 1):
            cx = int(round(gx + (ox - gx) * k / steps))
            cy = int(round(gy + (oy - gy) * k / steps))
            if any(not obstacles.is_blocked(cx, cy, li) for li in range(_nlay)):
                return cx, cy
        return None

    if all(obstacles.is_blocked(s_gx, s_gy, li) for li in range(_nlay)):
        r = _relax(s_gx, s_gy, t_gx, t_gy)
        if r is None:
            return None
        s_gx, s_gy = r
        csx, csy = coord.to_float(s_gx, s_gy)
    if all(obstacles.is_blocked(t_gx, t_gy, li) for li in range(_nlay)):
        r = _relax(t_gx, t_gy, s_gx, s_gy)
        if r is None:
            return None
        t_gx, t_gy = r
        ctx, cty = coord.to_float(t_gx, t_gy)

    # Base (un-setback) endpoints: each candidate couples as close to these as its
    # wide corridor allows (see _closest_launch in the loop).
    s_gx0, s_gy0 = s_gx, s_gy
    t_gx0, t_gy0 = t_gx, t_gy
    # The coupled pair occupies a swath ~diff_pair_spacing_grid wide; a launch point
    # must have that swath clear or the A* dead-ends between the tightly-spaced stub
    # ends. Search inward from the terminal for the CLOSEST cell whose swath is clear.
    _swath_r = max(1, diff_pair_spacing_grid)

    def _closest_launch(gx0, gy0, ox, oy, layer):
        """Closest cell to (gx0,gy0), stepping toward (ox,oy), whose
        diff-pair-wide swath is clear on `layer` -- couple as far as possible, the
        single-ended leg spans the small remaining gap. Returns (cx,cy) or None."""
        steps = max(int(math.hypot(ox - gx0, oy - gy0)), 1)
        for k in range(0, steps + 1):
            cx = int(round(gx0 + (ox - gx0) * k / steps))
            cy = int(round(gy0 + (oy - gy0) * k / steps))
            if not any(obstacles.is_blocked(cx + dxx, cy + dyy, layer)
                       for dxx in range(-_swath_r, _swath_r + 1)
                       for dyy in range(-_swath_r, _swath_r + 1)):
                return cx, cy
        return None

    def _theta_options(ex, ey, slx, sly, arrival):
        """Ordered candidate pose headings (theta_idx) for one coupled-middle end.
        The best launch/arrival heading depends on local congestion and is hard to
        know a priori (issue #244), so SCAN rather than pick one: the terminal
        escape heading first (when it points toward the route), then the straight
        src->tgt line, then a +-45/+-90 fan around it. ``arrival`` flips the escape
        for the target end (the path ARRIVES heading -escape, valid when the escape
        points back toward the source)."""
        opts = []

        def _add(vx, vy):
            if abs(vx) < 1e-9 and abs(vy) < 1e-9:
                return
            t = direction_to_theta_idx(vx, vy)
            if t not in opts:
                opts.append(t)
        _add(slx, sly)                       # straight line FIRST (historical default)
        if ex or ey:
            ehx, ehy = (-ex, -ey) if arrival else (ex, ey)
            if ehx * slx + ehy * sly > 0:    # escape agrees with the straight line
                _add(ehx, ehy)
        # +-45deg fan around the straight line. theta_idx is quantised to 8 dirs (45deg
        # steps), so +-45 is the only meaningful off-axis launch; +-90 was too divergent
        # and produced worse middle geometry (finer angles aren't representable).
        for deg in (45.0, -45.0):
            r = math.radians(deg)
            _add(slx * math.cos(r) - sly * math.sin(r),
                 slx * math.sin(r) + sly * math.cos(r))
        return opts

    probe_cap = max(int(config.max_probe_iterations), 1)

    for a_layer, b_layer in layer_pairs:
        # Couple as close to each terminal as this candidate's launch layer allows.
        _sl = _closest_launch(s_gx0, s_gy0, t_gx0, t_gy0, a_layer)
        _tl = _closest_launch(t_gx0, t_gy0, s_gx0, s_gy0, b_layer)
        if _sl is None or _tl is None:
            continue
        s_gx, s_gy = _sl
        t_gx, t_gy = _tl
        csx, csy = coord.to_float(s_gx, s_gy)
        ctx, cty = coord.to_float(t_gx, t_gy)
        if (s_gx, s_gy) == (t_gx, t_gy):
            continue
        _dxc, _dyc = ctx - csx, cty - csy
        _Lc = math.hypot(_dxc, _dyc) or 1.0
        _slx, _sly = _dxc / _Lc, _dyc / _Lc
        src_thetas = _theta_options(src_ex, src_ey, _slx, _sly, arrival=False)
        tgt_thetas = _theta_options(tgt_ex, tgt_ey, _slx, _sly, arrival=True)
        obstacles.clear_allowed_cells()
        obstacles.clear_source_target_cells()
        for ox in range(-2, 3):
            for oy in range(-2, 3):
                obstacles.add_allowed_cell(s_gx + ox, s_gy + oy)
                obstacles.add_allowed_cell(t_gx + ox, t_gy + oy)
        obstacles.add_source_target_cell(s_gx, s_gy, a_layer)
        obstacles.add_source_target_cell(t_gx, t_gy, b_layer)
        # Middle angle selection (issue #244). The straight src->tgt line is the
        # HISTORICAL heading -- route it FIRST at the full budget so already-routable
        # pairs get the exact same middle (no DRC drift). Only if the straight line
        # finds NO path do we scan the escape heading and a +-45 fan; those are tried
        # at the cheap probe budget first so a WRONG heading can't burn the full iter
        # cap (a bad launch dead-ended D1 for 1.6M iters), then the first "promising"
        # combo (hit the probe cap still searching) is re-run at full budget.
        straight_t = direction_to_theta_idx(_slx, _sly)

        def _pose(st, tt, budget):
            return pr.route_pose_with_frontier(
                obstacles, s_gx, s_gy, a_layer, st, t_gx, t_gy, b_layer, tt,
                budget, diff_pair_via_spacing=via_spacing_grid)
        path, iters, _b, _g = _pose(straight_t, straight_t, max_iters)
        iters = iters or 0
        if path is None:
            promising = None
            for st in src_thetas:
                for tt in tgt_thetas:
                    if st == straight_t and tt == straight_t:
                        continue                 # already tried at full budget
                    p, it, _b, _g = _pose(st, tt, probe_cap)
                    iters += it
                    if p:
                        path = p
                        break
                    if it >= probe_cap and promising is None:
                        promising = (st, tt)
                if path:
                    break
            if path is None and promising is not None:
                st, tt = promising
                p, it, _b, _g = _pose(st, tt, max_iters)
                iters += it
                path = p
        obstacles.clear_allowed_cells()
        obstacles.clear_source_target_cells()
        if not path:
            continue
        grid_path = []
        for gx, gy, _th, lyr in path:
            if grid_path and grid_path[-1] == (gx, gy, lyr):
                continue
            grid_path.append((gx, gy, lyr))
        simplified = simplify_path(grid_path)
        if len(simplified) < 2:
            continue
        centerline_float = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], lyr)
                            for gx, gy, lyr in simplified]
        # Choose which offset side is P (no coupled polarity stage here). Pick the
        # assignment that minimises terminal-leg crossings: put the P track on the
        # side P's vias are actually on. A genuine side-swap (P-via on opposite
        # sides at the two ends) costs one crossing either way, but this never
        # crosses BOTH legs gratuitously.
        off_plus = create_parallel_path_from_float(centerline_float, sign=1, spacing_mm=spacing_mm)
        off_minus = create_parallel_path_from_float(centerline_float, sign=-1, spacing_mm=spacing_mm)

        # Choose which offset track is P by the SIDE the P pad sits on relative to the
        # local centerline direction at each end -- NOT raw distance to the middle's
        # offset endpoints. Distance is unreliable when the middle couples far from the
        # pad (a stepped-back / dense-source launch): the two offset endpoints are only
        # ~diff-pair-spacing apart, so a pad ~1mm away is nearly equidistant to both and
        # noise picks the side, putting P on the WRONG side of the middle and forcing
        # both legs to cross (issue #244, butterstick D1). The side (cross product of
        # the centerline tangent with pad-minus-endpoint) is sign-stable at any range.
        def _cross(ax, ay, bx, by):
            return ax * by - ay * bx

        def _p_on_plus(c_near, c_dir, ppx, ppy, off_pt):
            # True if the P pad is on the same side of the directed centerline as the
            # +offset track at this end.
            dx, dy = c_dir
            plus_side = _cross(dx, dy, off_pt[0] - c_near[0], off_pt[1] - c_near[1])
            pad_side = _cross(dx, dy, ppx - c_near[0], ppy - c_near[1])
            if abs(plus_side) < 1e-9 or abs(pad_side) < 1e-9:
                return None
            return (plus_side > 0) == (pad_side > 0)
        c0 = centerline_float[0]
        c1 = centerline_float[1] if len(centerline_float) > 1 else centerline_float[-1]
        cN = centerline_float[-1]
        cM = centerline_float[-2] if len(centerline_float) > 1 else centerline_float[0]
        src_plus = _p_on_plus(c0, (c1[0] - c0[0], c1[1] - c0[1]),
                              p_src_x, p_src_y, off_plus[0])
        tgt_plus = _p_on_plus(cN, (cN[0] - cM[0], cN[1] - cM[1]),
                              p_tgt_x, p_tgt_y, off_plus[-1])
        votes = [v for v in (src_plus, tgt_plus) if v is not None]
        if votes:
            # Majority of the two ends; a genuine end-to-end side-swap (one True, one
            # False) costs one crossing either way -- default to P=+ there.
            p_sign = 1 if sum(votes) * 2 >= len(votes) else -1
        else:
            # Degenerate (both ends co-linear): fall back to the distance heuristic.
            def _d2(a, b):
                return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            cost_plus = ((_d2((p_src_x, p_src_y), off_plus[0]) > _d2((p_src_x, p_src_y), off_minus[0])) +
                         (_d2((p_tgt_x, p_tgt_y), off_plus[-1]) > _d2((p_tgt_x, p_tgt_y), off_minus[-1])))
            p_sign = 1 if cost_plus <= 1 else -1
        n_sign = -p_sign
        p_float = off_plus if p_sign == 1 else off_minus
        n_float = off_minus if p_sign == 1 else off_plus
        p_float, n_float = _process_via_positions(simplified, p_float, n_float, coord, config, p_sign, n_sign, spacing_mm)
        p_segs, p_vias, _ = _float_path_to_geometry(
            p_float, p_net_id, None, None, p_sign, (0, 0), (0, 0), 0, 0, config, layer_names, omit_connectors=True)
        n_segs, n_vias, _ = _float_path_to_geometry(
            n_float, n_net_id, None, None, n_sign, (0, 0), (0, 0), 0, 0, config, layer_names, omit_connectors=True)
        mid_segs = p_segs + n_segs
        if _pn_tracks_cross_full(mid_segs, pcb_data, p_net_id, n_net_id):
            continue
        if _connector_grazes_foreign_copper(mid_segs, pcb_data, p_net_id, n_net_id, config):
            continue

        # Attach each terminal to its middle near-end with point-to-point single-ended
        # legs. The partner net's copper (its middle + already-committed legs + fanout
        # stub + pads) is an obstacle, so a side-swap leg routes AROUND it -- polarity
        # is fixed at the pads. The legs are SINGLE-ENDED, so route them on a single-
        # ended-clearance map (leg_obstacles) when supplied -- the diff-pair map's
        # coupled extra clearance over-blocks a leg's escape via (watchy USB_D).
        #
        # Try leg-attachment ORDER PLANS in sequence, first that routes all four legs
        # wins (issue #244). Plan 0 is the historical net-by-net order (P fully, then
        # N) -- kept FIRST so already-routable pairs attach identically (no DRC drift).
        # The fallbacks free a convergent leg that the default order boxes out in a
        # dense terminal field (butterstick D1's FPGA source): routing the OTHER net
        # first, or both SRC legs before either TGT leg so a leg isn't blocked by its
        # own pair's far-end leg committed too early.
        total_iters = iters
        pair_vias = [v for v in (getattr(pcb_data, 'vias', None) or [])
                     if v.net_id in (p_net_id, n_net_id)]
        leg_obs = leg_obstacles if leg_obstacles is not None else obstacles
        board_segs = getattr(pcb_data, 'segments', None) or []
        pads_by_net = getattr(pcb_data, 'pads_by_net', None) or {}
        mid_segs_by_net = {p_net_id: p_segs, n_net_id: n_segs}
        mid_vias_by_net = {p_net_id: p_vias, n_net_id: n_vias}
        term_by = {
            (p_net_id, 'src'): ((p_src_x, p_src_y, src[4]), p_float[0]),
            (p_net_id, 'tgt'): ((p_tgt_x, p_tgt_y, tgt[4]), p_float[-1]),
            (n_net_id, 'src'): ((n_src_x, n_src_y, src[4]), n_float[0]),
            (n_net_id, 'tgt'): ((n_tgt_x, n_tgt_y, tgt[4]), n_float[-1]),
        }
        # Mutable per-net committed leg copper, reset per plan; a leg sees the PARTNER's
        # committed legs as obstacles, never its own.
        leg_state = {'s': {p_net_id: [], n_net_id: []}, 'v': {p_net_id: [], n_net_id: []}}

        def _leg_partner_copper(net_id):
            partner = n_net_id if net_id == p_net_id else p_net_id
            ppads = pads_by_net.get(partner, [])
            pseg = (mid_segs_by_net[partner] + leg_state['s'][partner]
                    + [s for s in board_segs if s.net_id == partner]
                    + [seg for pad in ppads for seg in _pad_obstacle_segments(pad, layer_names)])
            pvia = ([v for v in pair_vias if v.net_id == partner]
                    + mid_vias_by_net[partner] + leg_state['v'][partner])
            return pseg, pvia, ppads
        P, N = p_net_id, n_net_id
        leg_plans = [
            [(P, 'src'), (P, 'tgt'), (N, 'src'), (N, 'tgt')],   # 0: historical default
            [(N, 'src'), (N, 'tgt'), (P, 'src'), (P, 'tgt')],   # 1: other net first
            [(P, 'src'), (N, 'src'), (P, 'tgt'), (N, 'tgt')],   # 2: per-side (both src first)
            [(N, 'src'), (P, 'src'), (N, 'tgt'), (P, 'tgt')],   # 3: per-side, swapped
        ]
        ok = False
        leg_segs, leg_vias = [], []
        for plan in leg_plans:
            leg_state['s'] = {p_net_id: [], n_net_id: []}
            leg_state['v'] = {p_net_id: [], n_net_id: []}
            plan_iters = 0
            good = True
            for net_id, _side in plan:
                term, mid_pt = term_by[(net_id, _side)]
                pseg, pvia, ppads = _leg_partner_copper(net_id)
                ls, lv, it = _route_hybrid_leg(
                    pcb_data, net_id, config, leg_obs, layer_names, coord,
                    mid_pt, term, pseg, pvia, pair_vias, partner_pads=ppads)
                plan_iters += it
                if ls is None:
                    good = False
                    break
                leg_state['s'][net_id] += ls
                leg_state['v'][net_id] += lv
            if good:
                ok = True
                total_iters += plan_iters
                leg_segs = leg_state['s'][p_net_id] + leg_state['s'][n_net_id]
                leg_vias = leg_state['v'][p_net_id] + leg_state['v'][n_net_id]
                break
        if not ok:
            continue
        all_segs = mid_segs + leg_segs
        all_vias = p_vias + n_vias + leg_vias
        if _pn_tracks_cross_full(all_segs, pcb_data, p_net_id, n_net_id):
            continue
        # Airtight foreign-copper gate (#246): the assembled route (coupled-middle
        # offsets + legs) must not graze/cross a foreign pad, via OR track, or the
        # hybrid emits a DRC short unseen (D3_N offset x D2_N). Reject -> try next layer.
        if _connector_grazes_foreign_copper(all_segs, pcb_data, p_net_id, n_net_id, config):
            continue
        # Connectivity gate: the hybrid validates crossings/grazes but never that
        # the assembled route actually JOINS terminal-to-terminal -- a leg can fail
        # to bridge to the middle (e.g. a layer-mismatch at a coincident cell) and
        # still be committed, so the pair reports "routed" with one half open
        # (#215 false-success, RX3_N). Re-verify with the copper-aware connectivity
        # check (new copper + the pair's existing fanout stubs) and reject if either
        # half isn't fully connected -- then this layer is retried / the pair fails
        # honestly instead of counting as routed.
        if not _hybrid_route_connects(pcb_data, p_net_id, n_net_id, all_segs, all_vias):
            continue
        _mid_desc = (layer_names[a_layer] if a_layer == b_layer
                     else f"{layer_names[a_layer]}->{layer_names[b_layer]}")
        print(f"  DIRECT HYBRID: coupled middle on {_mid_desc} + "
              f"{len(leg_segs)} leg seg(s) ({total_iters} iters)")
        return {'new_segments': all_segs, 'new_vias': all_vias,
                'iterations': total_iters, 'path_length': len(simplified)}
    return None


def _hybrid_route_connects(pcb_data, p_net_id, n_net_id, new_segs, new_vias) -> bool:
    """True only if BOTH halves of the pair connect terminal-to-terminal with the
    new hybrid copper added to the pair's existing (fanout-stub) copper. Used to
    reject a hybrid route that left a terminal orphaned (#215)."""
    from check_connected import check_net_connectivity
    board_segs = getattr(pcb_data, 'segments', None) or []
    board_vias = getattr(pcb_data, 'vias', None) or []
    pads_by_net = getattr(pcb_data, 'pads_by_net', None) or {}
    for nid in (p_net_id, n_net_id):
        segs = [s for s in board_segs if s.net_id == nid] + [s for s in new_segs if s.net_id == nid]
        vias = [v for v in board_vias if v.net_id == nid] + [v for v in new_vias if v.net_id == nid]
        r = check_net_connectivity(nid, segs, vias, pads_by_net.get(nid, []))
        if not r.get('connected') or r.get('disconnected_pads'):
            return False
    return True


def _seg_to_seglist_min_edge(x1, y1, x2, y2, width, layer, other_segs):
    """Min EDGE gap (centre distance minus the two half-widths) from segment
    (x1,y1)-(x2,y2) of the given ``width`` to any same-layer segment in
    ``other_segs``. inf if there are none on this layer."""
    best = math.inf
    for o in other_segs:
        if o.layer != layer:
            continue
        d = _seg_seg_distance(x1, y1, x2, y2, o.start_x, o.start_y, o.end_x, o.end_y)
        ow = o.width if getattr(o, 'width', 0) else width
        best = min(best, d - width / 2.0 - ow / 2.0)
    return best


def _seg_seg_distance(ax, ay, bx, by, cx, cy, dx, dy):
    """Minimum Euclidean distance between segments AB and CD (endpoint sampling --
    exact for the non-crossing case that matters for a clearance check)."""
    def pt_seg(px, py, x1, y1, x2, y2):
        vx, vy = x2 - x1, y2 - y1
        l2 = vx * vx + vy * vy
        if l2 <= 0.0:
            return math.hypot(px - x1, py - y1)
        t = ((px - x1) * vx + (py - y1) * vy) / l2
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
        return math.hypot(px - (x1 + t * vx), py - (y1 + t * vy))
    return min(pt_seg(ax, ay, cx, cy, dx, dy), pt_seg(bx, by, cx, cy, dx, dy),
               pt_seg(cx, cy, ax, ay, bx, by), pt_seg(dx, dy, ax, ay, bx, by))


def _collapse_leg_attach_join(leg_segs, attach_xy, config, pcb_data, net_id, partner_segs):
    """Collapse a hybrid leg's short grid->float "join" at the coupled-middle
    attach end when it pinches the partner track below clearance (#215).

    The leg's single-ended A* lands on the GRID cell nearest the coupled-middle
    near-end; ``_path_to_segments_vias`` then emits a short join segment from that
    grid stand-in out to the exact off-grid near-end (``attach_xy``). At a tight
    P/N junction the two legs' grid stand-in vertices quantise toward each other
    (the float near-ends are a clean diff-pair-gap apart, but their grid cells
    round ~half a step closer), so the P and N corners at those stand-ins sit
    below clearance even though the float near-ends do not -- the SEGMENT-SEGMENT
    overlaps in #215.

    Fix: drop the join and move the penultimate segment's far vertex onto the
    exact float near-end, so the leg corner sits at the (farther-apart) float
    point. This only fires when (a) the original grid corner is actually
    sub-clearance to the partner copper -- so clean junctions are left untouched
    and downstream pairs are not perturbed -- and (b) the relocated segment
    provably clears the partner (and any foreign pad) by the full clearance, so
    the tilt can never introduce a new graze. Mutates ``leg_segs`` in place."""
    if len(leg_segs) < 2:
        return leg_segs
    ax, ay = attach_xy
    join = leg_segs[-1]
    if abs(join.end_x - ax) > 1e-6 or abs(join.end_y - ay) > 1e-6:
        return leg_segs  # last seg doesn't terminate at the exact near-end
    if math.hypot(join.end_x - join.start_x, join.end_y - join.start_y) > 1.5 * config.grid_step:
        return leg_segs  # not a short grid->float join (don't relocate a real run)
    pen = leg_segs[-2]
    if pen.layer != join.layer:
        return leg_segs  # via between leg body and attach -> can't merge across it
    if abs(pen.end_x - join.start_x) > 1e-6 or abs(pen.end_y - join.start_y) > 1e-6:
        return leg_segs  # join doesn't continue the penultimate segment

    w = config.get_net_track_width(net_id, pen.layer)
    # Only act on a REAL local violation: the grid corner (penultimate's far end)
    # must currently sit below clearance to the partner copper.
    before = _seg_to_seglist_min_edge(pen.start_x, pen.start_y, pen.end_x, pen.end_y,
                                      w, pen.layer, partner_segs)
    if before >= config.clearance - 1e-6:
        return leg_segs  # corner already clears the partner -> nothing to fix
    # The relocated segment must clear the partner by the FULL clearance, or we'd
    # just trade one graze for another (the butterstick D4 tilt-into-partner case).
    after = _seg_to_seglist_min_edge(pen.start_x, pen.start_y, ax, ay, w, pen.layer, partner_segs)
    if after < config.clearance - 1e-6 or after <= before + 1e-9:
        return leg_segs  # collapse doesn't help (or makes it worse)
    if pcb_data is not None:
        from single_ended_routing import _seg_foreign_pad_dist
        fmargin = config.clearance + w / 2.0
        if _seg_foreign_pad_dist(pcb_data, net_id, pen.start_x, pen.start_y,
                                 ax, ay, pen.layer) < fmargin - 1e-6:
            return leg_segs  # collapsed segment would graze a foreign pad
    pen.end_x, pen.end_y = ax, ay
    del leg_segs[-1]
    return leg_segs


def _route_hybrid_leg(pcb_data, net_id, config, obstacles, layer_names, coord,
                      mid_pt, term, partner_segs, partner_vias, pair_vias,
                      partner_pads=None):
    """Route ONE point-to-point single-ended leg joining a terminal to the coupled
    middle's near end (term -> mid_pt), routing around partner copper. Returns
    (segs, vias, iters), ([], [], 0) when the terminal already coincides with the
    middle end (nothing to route), or (None, None, 0) if the leg can't be routed.

    Routing one leg at a time (vs both ends of a net at once) lets the caller
    interleave the two nets' legs PER SIDE and retry the order, so a first leg
    can't gratuitously box out its partner's convergent leg in a dense terminal
    field (issue #244).

    partner_pads: the OTHER pair-net's pads. The diff-pair obstacle map excludes
    both pair nets, so without re-adding them as a VIA keep-out the leg would drop
    its layer-transition via on top of a partner pad (issue #241). Track passage
    is left open (the coupled trace legitimately runs close); only via drops are
    kept clear."""
    from single_ended_routing import _route_leg, _path_to_segments_vias
    from obstacle_map import (add_segments_list_as_obstacles, remove_segments_list_from_obstacles,
                              add_vias_list_as_obstacles, remove_vias_list_from_obstacles,
                              add_pads_via_keepout, remove_pads_via_keepout,
                              get_same_net_through_hole_positions)
    partner_pads = partner_pads or []
    router = GridRouter(
        via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
        layer_costs=config.get_layer_costs())
    nlayers = len(config.layers)
    own_tol = _launch_assoc_tol(config)
    # An existing same-net through-hole (the net's THT pads + any pre-placed via,
    # e.g. a hand-placed escape via) already connects all layers, so a leg whose
    # path lands on that cell must REUSE it -- not stack a second coincident via a
    # hole-to-hole bust. Mirror the multipoint router's reuse set.
    reuse_holes = set(get_same_net_through_hole_positions(pcb_data, net_id, config))
    for _v in (getattr(pcb_data, 'vias', None) or []):
        if _v.net_id == net_id:
            reuse_holes.add(coord.to_grid(_v.x, _v.y))

    # The leg endpoint: the terminal's own-net through-via (its access to the middle
    # layer) if one sits at the terminal, else the pad itself.
    ax, ay, alayer, on_via = term[0], term[1], term[2], False
    best = None
    for v in pair_vias:
        if v.net_id != net_id:
            continue
        d = math.hypot(v.x - term[0], v.y - term[1])
        if d <= own_tol and (best is None or d < best[0]):
            best = (d, v.x, v.y, mid_pt[2])
    if best:
        ax, ay, alayer, on_via = best[1], best[2], best[3], True

    add_segments_list_as_obstacles(obstacles, partner_segs, config)
    add_vias_list_as_obstacles(obstacles, partner_vias, config)
    add_pads_via_keepout(obstacles, partner_pads, config)
    try:
        tgx, tgy = coord.to_grid(ax, ay)
        mgx, mgy = coord.to_grid(mid_pt[0], mid_pt[1])
        if (tgx, tgy) == (mgx, mgy) and (on_via or alayer == mid_pt[2]):
            # Truly coincident: the terminal already sits on the middle's end cell on
            # the SAME layer (or carries a through-via that spans all layers). If the
            # cell coincides but the LAYERS differ (a bare F.Cu escape end vs an
            # inner-layer middle), do NOT skip -- the leg must still drop the
            # F.Cu->inner transition via, or the terminal is left orphaned (#215).
            return [], [], 0
        # On a through-via the leg may leave on any layer; off a bare pad it must
        # start on the pad's own layer.
        sources = ([(tgx, tgy, l) for l in range(nlayers)] if on_via
                   else [(tgx, tgy, alayer)])
        targets = [(mgx, mgy, mid_pt[2])]
        obstacles.clear_allowed_cells()
        obstacles.clear_source_target_cells()
        # Mark ONLY the exact endpoint cells as source/target so the A* can start/end
        # ON them even when foreign clearance blocks them (the terminal pad / coupled-
        # middle end sits in a neighbour's keep-out). Issue #244: do NOT add a +-2
        # allowed-cell HALO -- that exempted a whole 5x5 block near each junction from
        # clearance, letting a leg graze/cross a neighbour pair's leg there (butterstick
        # D3_N x D2_N on In2.Cu). Without the halo the leg must honour every clearance,
        # so it can't create a DRC violation; it just declines to route if truly boxed.
        for sgx, sgy, slayer in sources:
            obstacles.add_source_target_cell(sgx, sgy, slayer)
        obstacles.add_source_target_cell(mgx, mgy, mid_pt[2])
        path, it = _route_leg(router, obstacles, config, sources, targets, 0, pcb_data, net_id)
        obstacles.clear_allowed_cells()
        obstacles.clear_source_target_cells()
        if path is None:
            return None, None, 0
        ps, pv = _path_to_segments_vias(
            path, coord, layer_names, net_id, config,
            (ax, ay, layer_names[path[0][2]]),
            (mid_pt[0], mid_pt[1], layer_names[mid_pt[2]]),
            through_hole_positions=reuse_holes, pcb_data=pcb_data)
        _collapse_leg_attach_join(ps, (mid_pt[0], mid_pt[1]), config, pcb_data, net_id, partner_segs)
        return ps, pv, it
    finally:
        remove_segments_list_from_obstacles(obstacles, partner_segs, config)
        remove_vias_list_from_obstacles(obstacles, partner_vias, config)
        remove_pads_via_keepout(obstacles, partner_pads, config)


def route_diff_pair_with_obstacles(pcb_data: PCBData, diff_pair: DiffPairNet,
                                    config: GridRouteConfig,
                                    obstacles: GridObstacleMap,
                                    base_obstacles: GridObstacleMap = None,
                                    unrouted_stubs: List[Tuple[float, float]] = None,
                                    flip_source: bool = False,
                                    flip_target: bool = False,
                                    endpoints: Optional[Tuple] = None,
                                    forced_source_dir: Optional[Tuple[float, float]] = None,
                                    forced_target_dir: Optional[Tuple[float, float]] = None,
                                    swap_allowed_ends: Tuple[str, ...] = ('source', 'target'),
                                    force_swap: bool = False,
                                    force_no_swap: bool = False,
                                    ) -> Optional[dict]:
    """
    Route a differential pair using centerline + offset approach.

    1. Routes a single centerline path using A* (GridRouter)
    2. Simplifies the path by removing collinear points
    3. Creates P and N paths using perpendicular offsets from centerline

    flip_source/flip_target force the escape direction at one end to the
    opposite side (bare-pad endpoints only) - used to resolve a polarity
    mismatch geometrically when polarity fixing is disabled.

    endpoints: optional (source, target) endpoint tuples (same format as
    get_diff_pair_endpoints returns) to route a specific leg instead of the
    closest P/N endpoint pair - used by multi-point diff pair routing.

    forced_source_dir/forced_target_dir: explicit escape directions for the
    physical source/target end (multi-point chain continuation).

    swap_allowed_ends: physical ends ('source'/'target') where a polarity pad
    swap may be applied. Multi-point legs restrict this to chain-fresh
    terminals - swapping a terminal that already has a routed leg attached
    would break that leg. The swap lands at the ROUTING-target end, so the
    permission is checked against the routing direction. Pass () to forbid
    swaps entirely.

    force_swap: internal - re-route committing to the pad-swap resolution
    (used to generate the swap candidate for length comparison).
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

    # Find endpoints (or use explicit leg endpoints for multi-point routing)
    if endpoints is not None:
        sources, targets = [endpoints[0]], [endpoints[1]]
    else:
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

    # Check proximity zones BEFORE setting endpoint exemptions (which would zero them out)
    p_src_gx, p_src_gy = original_src[0], original_src[1]
    n_src_gx, n_src_gy = original_src[2], original_src[3]
    p_tgt_gx, p_tgt_gy = original_tgt[0], original_tgt[1]
    n_tgt_gx, n_tgt_gy = original_tgt[2], original_tgt[3]
    p_src_prox = obstacles.get_stub_proximity_cost(p_src_gx, p_src_gy)
    n_src_prox = obstacles.get_stub_proximity_cost(n_src_gx, n_src_gy)
    p_tgt_prox = obstacles.get_stub_proximity_cost(p_tgt_gx, p_tgt_gy)
    n_tgt_prox = obstacles.get_stub_proximity_cost(n_tgt_gx, n_tgt_gy)
    src_in_stub = (p_src_prox > 0 or n_src_prox > 0)
    src_in_bga = (obstacles.is_in_bga_proximity(p_src_gx, p_src_gy) or
                  obstacles.is_in_bga_proximity(n_src_gx, n_src_gy))
    tgt_in_stub = (p_tgt_prox > 0 or n_tgt_prox > 0)
    tgt_in_bga = (obstacles.is_in_bga_proximity(p_tgt_gx, p_tgt_gy) or
                  obstacles.is_in_bga_proximity(n_tgt_gx, n_tgt_gy))
    # Diff pairs use 1/10th of the heuristic factor
    prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga) // 10
    if config.verbose:
        zones = []
        if src_in_stub: zones.append("src:stub")
        if src_in_bga: zones.append("src:bga")
        if tgt_in_stub: zones.append("tgt:stub")
        if tgt_in_bga: zones.append("tgt:bga")
        print(f"  proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}] (diff_pair 1/10th)")

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

    # Determine direction order (always deterministic)
    if config.direction_order in ("backwards", "backward"):
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

    # Probe first direction
    route_data, first_probe_iters, first_blocked, first_best_combo = _try_route_direction(
        first_src, first_tgt, pcb_data, config, obstacles, base_obstacles,
        coord, layer_names, spacing_mm, p_net_id, n_net_id,
        max_iterations_override=probe_iterations, neighbor_stubs=unrouted_stubs,
        direction_label=first_label, is_backward=(first_label == "backward"),
        prox_h_cost=prox_h_cost,
        flip_source=flip_source, flip_target=flip_target,
        forced_source_dir=forced_source_dir, forced_target_dir=forced_target_dir
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
            direction_label=second_label, is_backward=(second_label == "backward"),
            prox_h_cost=prox_h_cost,
            flip_source=flip_source, flip_target=flip_target,
        forced_source_dir=forced_source_dir, forced_target_dir=forced_target_dir
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
            # Both probes failed - only try full search if BOTH probes reached max iterations
            first_reached_max = first_probe_iters >= probe_iterations
            second_reached_max = second_probe_iters >= probe_iterations

            if not (first_reached_max and second_reached_max):
                # At least one probe didn't reach max - that direction is stuck, skip full search
                if not first_reached_max and not second_reached_max:
                    print(f"  Both directions stuck ({first_label}={first_probe_iters}, {second_label}={second_probe_iters} < {probe_iterations})")
                elif not first_reached_max:
                    print(f"  {first_label} stuck ({first_probe_iters} < {probe_iterations}), {second_label}={second_probe_iters}")
                else:
                    print(f"  {second_label} stuck ({second_probe_iters} < {probe_iterations}), {first_label}={first_probe_iters}")
                return {
                    'failed': True,
                    'iterations': first_probe_iters + second_probe_iters,
                    'blocked_cells_forward': first_blocked if first_label == "forward" else second_blocked,
                    'blocked_cells_backward': second_blocked if first_label == "forward" else first_blocked,
                    'iterations_forward': first_probe_iters if first_label == "forward" else second_probe_iters,
                    'iterations_backward': second_probe_iters if first_label == "forward" else first_probe_iters,
                }

            # Both probes reached max - do full search on first direction
            promising_src, promising_tgt, promising_label = first_src, first_tgt, first_label
            fallback_src, fallback_tgt, fallback_label = second_src, second_tgt, second_label
            promising_probe_iters, promising_blocked = first_probe_iters, first_blocked
            fallback_probe_iters, fallback_blocked = second_probe_iters, second_blocked
            promising_best_combo, fallback_best_combo = first_best_combo, second_best_combo

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
                direction_label=None, is_backward=(promising_label == "backward"),
                prox_h_cost=prox_h_cost,
                flip_source=flip_source, flip_target=flip_target,
        forced_source_dir=forced_source_dir, forced_target_dir=forced_target_dir
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
                # Promising direction failed, try fallback (we know both reached max, so fallback is worth trying)
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
                    direction_label=None, is_backward=(fallback_label == "backward"),
                    prox_h_cost=prox_h_cost,
                    flip_source=flip_source, flip_target=flip_target,
        forced_source_dir=forced_source_dir, forced_target_dir=forced_target_dir
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
    p_sign, polarity_swap_needed, has_layer_change = _detect_polarity(
        simplified_path, coord,
        p_src_x, p_src_y, n_src_x, n_src_y,
        p_tgt_x, p_tgt_y, n_tgt_x, n_tgt_y,
        config
    )
    n_sign = -p_sign

    # Resolve a polarity mismatch. Two mechanisms:
    # - pad swap: swap the P/N pad nets at the routing-target end (only when
    #   polarity fixing is on AND that physical end permits swaps - multi-point
    #   legs forbid swapping terminals that already have a routed leg)
    # - connector flip: take the connectors out the opposite side at ONE end
    #   (bare-pad ends with synthesized directions only; flipping both ends
    #   would reintroduce the mismatch)
    # When both mechanisms are available, route all candidates and keep the
    # shortest - either one can resolve polarity with a needlessly long route
    # (e.g. a flip that wraps all the way around its own terminal).
    polarity_fixed = False
    # force_swap also enters when the detector saw NO mismatch: _detect_polarity
    # is blind to a route's winding, so the crossing guard at the success exit
    # can request an explicit swap for an undetected odd pad permutation.
    # force_no_swap is the inverse: lay the natural (un-swapped) geometry and
    # skip swap resolution, so the caller can compare it against the swap route.
    if (polarity_swap_needed or force_swap) and not force_no_swap:
        swap_end = 'source' if routing_backwards else 'target'
        swap_allowed = config.fix_polarity and swap_end in swap_allowed_ends

        if force_swap:
            # This recursion IS the committed swap route.
            print(f"  Polarity swap needed - will swap target pad and stub nets in output")
            polarity_fixed = True
        elif not (flip_source or flip_target):
            # _detect_polarity only SUSPECTS a swap, and it false-positives on
            # routes that turn (it compares P's side at the first vs last
            # centerline segment). So route every legal option to completion and
            # keep the SHORTEST crossing-free one: the no-swap natural geometry,
            # the pad swap, and any opposite-side connector flip. Comparing by
            # length means a genuine mismatch still wins (its swap is shorter),
            # while a false positive keeps the shorter no-swap route (issue #142).
            spent_iterations = total_iterations
            candidates = []

            if config.fix_polarity:
                # No-swap candidate: the natural geometry, valid when its P/N
                # (incl. bare-pad target stubs) do not actually cross.
                no_swap = route_diff_pair_with_obstacles(
                    pcb_data, diff_pair, config, obstacles, base_obstacles,
                    unrouted_stubs, endpoints=endpoints,
                    forced_source_dir=forced_source_dir,
                    forced_target_dir=forced_target_dir,
                    swap_allowed_ends=swap_allowed_ends, force_no_swap=True)
                if (no_swap and not no_swap.get('failed')
                        and not no_swap.get('probe_blocked')
                        and not _pn_tracks_cross_full(no_swap.get('new_segments', []),
                                                      pcb_data, p_net_id, n_net_id)):
                    spent_iterations += no_swap.get('iterations', 0)
                    candidates.append(('no swap', no_swap))

            if swap_allowed:
                # Swap candidate: same centerline, completed with the pad swap
                swap_result = route_diff_pair_with_obstacles(
                    pcb_data, diff_pair, config, obstacles, base_obstacles,
                    unrouted_stubs, endpoints=endpoints,
                    forced_source_dir=forced_source_dir,
                    forced_target_dir=forced_target_dir,
                    swap_allowed_ends=swap_allowed_ends, force_swap=True)
                if swap_result and not swap_result.get('failed') and not swap_result.get('probe_blocked'):
                    spent_iterations += swap_result.get('iterations', 0)
                    candidates.append(('pad swap', swap_result))

            for flip_kwargs, end_name, synth_key in (
                    ({'flip_source': True}, 'source', 'source_dir_synthesized'),
                    ({'flip_target': True}, 'target', 'target_dir_synthesized')):
                if not route_data.get(synth_key):
                    continue
                print(f"  Polarity mismatch - trying {end_name} connectors out the opposite side")
                retry = route_diff_pair_with_obstacles(
                    pcb_data, diff_pair, config, obstacles, base_obstacles,
                    unrouted_stubs, endpoints=endpoints,
                    forced_source_dir=forced_source_dir,
                    forced_target_dir=forced_target_dir,
                    swap_allowed_ends=(), **flip_kwargs)
                if not retry:
                    continue
                spent_iterations += retry.get('iterations', 0)
                if retry.get('failed') or retry.get('probe_blocked'):
                    continue
                if retry.get('polarity_swap_needed') and not retry.get('polarity_fixed'):
                    print(f"  Flipped {end_name} route still has mismatched polarity - rejecting")
                    continue
                # The flipped route can wrap around to reach the far side; verify
                # the offset P/N tracks did not end up crossing anywhere
                if _pn_tracks_cross(retry.get('new_segments', []), p_net_id, n_net_id):
                    print(f"  Flipped {end_name} route crosses itself - rejecting")
                    continue
                candidates.append((f'flipped {end_name}', retry))

            if candidates:
                best_name, best = min(candidates, key=lambda c: _routed_length(c[1]))
                if len(candidates) > 1:
                    lengths = ", ".join(f"{name}={_routed_length(r):.2f}mm"
                                        for name, r in candidates)
                    print(f"  Using {best_name} route ({lengths})")
                best['iterations'] = spent_iterations
                return best

            print(f"  WARNING: Polarity mismatch cannot be resolved (pad swap "
                  f"{'failed' if swap_allowed else 'not allowed'}, no opposite-side "
                  f"connector option) - skipping diff pair")
            return {
                'failed': True,
                'polarity_skip': True,
                'iterations': spent_iterations,
                'blocked_cells_forward': [],
                'blocked_cells_backward': [],
            }

    # Create P and N paths using perpendicular offsets from centerline
    # Use actual setback directions at endpoints so perpendicular offsets maintain
    # proper spacing along the connector segments (especially when setback angle is used)
    start_stub_dir = (src_actual_dir_x, src_actual_dir_y)
    end_stub_dir = (-tgt_actual_dir_x, -tgt_actual_dir_y)  # Negate because we arrive at target stubs

    # Un-snap the centerline's terminal endpoints to the exact setback
    # positions: grid snapping can bias an endpoint up to half a grid cell
    # toward one pad of the pair, skewing the connector fan enough to graze
    # the near pad's corner (sub-0.01mm clearance violations)
    centerline_float = [(coord.to_float(gx, gy)[0], coord.to_float(gx, gy)[1], layer)
                        for gx, gy, layer in simplified_path]
    if config.diff_pair_centerline_setback is not None:
        setback_dist = config.diff_pair_centerline_setback
    else:
        setback_dist = spacing_mm * 4
    for idx, (cx, cy, adx, ady) in (
            (0, (center_src_x, center_src_y, src_actual_dir_x, src_actual_dir_y)),
            (-1, (center_tgt_x, center_tgt_y, tgt_actual_dir_x, tgt_actual_dir_y))):
        exact = (cx + adx * setback_dist, cy + ady * setback_dist)
        fx, fy, flayer = centerline_float[idx]
        if math.hypot(fx - exact[0], fy - exact[1]) <= config.grid_step * 0.75:
            centerline_float[idx] = (exact[0], exact[1], flayer)

    p_float_path = create_parallel_path_from_float(centerline_float, sign=p_sign, spacing_mm=spacing_mm,
                                                   start_dir=start_stub_dir, end_dir=end_stub_dir)
    n_float_path = create_parallel_path_from_float(centerline_float, sign=n_sign, spacing_mm=spacing_mm,
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

    # Route's first/last points - the setback positions the connectors run to.
    # The terminal launch points (p_src_x/p_tgt_x etc.) are already shifted to the
    # pad EDGE facing the route by _try_route_direction (issue #165), so the
    # connectors leave the pad toward the route - nothing to re-shift here.
    p_src_route = p_float_path[0][:2] if p_float_path else p_start
    n_src_route = n_float_path[0][:2] if n_float_path else n_start
    p_tgt_route = p_float_path[-1][:2] if p_float_path else p_end
    n_tgt_route = n_float_path[-1][:2] if n_float_path else n_end

    # Pre-calculate extensions for source and target connectors (from the
    # pad-edge launch points) - these ensure P and N connector segments are parallel.
    src_p_ext, src_n_ext = _calculate_parallel_extension(
        p_start, n_start, p_src_route, n_src_route,
        src_stub_dir_tuple, p_sign
    )
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

    # Get stub vias (e.g., pad vias from layer switching) and include their barrel length
    p_src_stub_vias = get_stub_vias(pcb_data, p_net_id, p_src_stub_segs)
    p_tgt_stub_vias = get_stub_vias(pcb_data, p_net_id, p_tgt_stub_segs)
    n_src_stub_vias = get_stub_vias(pcb_data, n_net_id, n_src_stub_segs)
    n_tgt_stub_vias = get_stub_vias(pcb_data, n_net_id, n_tgt_stub_segs)

    p_src_via_len = calculate_stub_via_barrel_length(p_src_stub_vias, src_layer_name, pcb_data)
    p_tgt_via_len = calculate_stub_via_barrel_length(p_tgt_stub_vias, tgt_layer_name, pcb_data)
    n_src_via_len = calculate_stub_via_barrel_length(n_src_stub_vias, src_layer_name, pcb_data)
    n_tgt_via_len = calculate_stub_via_barrel_length(n_tgt_stub_vias, tgt_layer_name, pcb_data)

    p_src_stub_length = sum(segment_length(s) for s in p_src_stub_segs) + p_src_via_len
    p_tgt_stub_length = sum(segment_length(s) for s in p_tgt_stub_segs) + p_tgt_via_len
    n_src_stub_length = sum(segment_length(s) for s in n_src_stub_segs) + n_src_via_len
    n_tgt_stub_length = sum(segment_length(s) for s in n_tgt_stub_segs) + n_tgt_via_len

    if config.verbose:
        print(f"  Stub via barrel lengths (stub_layer->pad):")
        print(f"    P_src ({src_layer_name}): {len(p_src_stub_vias)} vias = {p_src_via_len:.3f}mm")
        print(f"    P_tgt ({tgt_layer_name}): {len(p_tgt_stub_vias)} vias = {p_tgt_via_len:.3f}mm")
        print(f"    N_src ({src_layer_name}): {len(n_src_stub_vias)} vias = {n_src_via_len:.3f}mm")
        print(f"    N_tgt ({tgt_layer_name}): {len(n_tgt_stub_vias)} vias = {n_tgt_via_len:.3f}mm")

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
        # True when source/target polarities differ (used by the flip retry to
        # verify a flipped route actually resolved the mismatch)
        'polarity_swap_needed': polarity_swap_needed,
        # Physical escape directions used at each end (after any flips), for
        # multi-point chain side tracking
        'source_base_dir': route_data.get('source_base_dir'),
        'target_base_dir': route_data.get('target_base_dir'),
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

    def _swap_positions(swap_info):
        """Positions a pending polarity swap (apply_polarity_swap) will relabel,
        so the crossing check can evaluate the FINAL (post-swap) net assignment."""
        if not swap_info:
            return None, None
        from polarity_swap import find_connected_segment_positions
        pp = find_connected_segment_positions(pcb_data, swap_info['p_pos'][0],
                                              swap_info['p_pos'][1], swap_info['p_net_id'])
        nn = find_connected_segment_positions(pcb_data, swap_info['n_pos'][0],
                                              swap_info['n_pos'][1], swap_info['n_net_id'])
        return pp, nn

    # Issue #102 / #142: never report a pair routed whose P and N copper crosses.
    # Check the FULL geometry (route + source/bare-pad stubs) with the pending
    # polarity swap applied VIRTUALLY, so a swap route's post-relabel self-crossing
    # is caught here, before apply_polarity_swap touches the segments. Without the
    # bare-pad stubs + virtual swap, such pairs shipped as claimed-good output with
    # genuine P-to-N shorts (neo6502 /D_P //D_N; eis /DDMI_CK #142).
    res_pp, res_nn = _swap_positions(result.get('swap_target_pads'))
    if _pn_tracks_cross_full(result.get('new_segments', []), pcb_data, p_net_id, n_net_id,
                             p_swap_positions=res_pp, n_swap_positions=res_nn):
        print("  P/N tracks cross in final geometry - rejecting pair route")
        # _detect_polarity is path-local and blind to a route's net winding,
        # so an undetected odd pad permutation can land here. Before failing,
        # try the polarity pad swap explicitly - it untangles interleaved
        # terminals (e.g. USB-C rows) that the detector missed. (Skip for the
        # no-swap probe: its caller checks the crossing and falls back to a swap.)
        swap_end = 'source' if routing_backwards else 'target'
        swap_result = None
        if (not force_swap and not force_no_swap and config.fix_polarity
                and swap_end in swap_allowed_ends):
            print("  Retrying with a polarity pad swap to untangle the crossing...")
            swap_result = route_diff_pair_with_obstacles(
                pcb_data, diff_pair, config, obstacles, base_obstacles,
                unrouted_stubs, endpoints=endpoints,
                forced_source_dir=forced_source_dir,
                forced_target_dir=forced_target_dir,
                swap_allowed_ends=swap_allowed_ends, force_swap=True)
            # Re-verify: the swap retry must not cross either (with ITS pending
            # swap applied virtually) - else we'd just ship the same short relabeled.
            sw_pp, sw_nn = _swap_positions(swap_result.get('swap_target_pads')) if swap_result else (None, None)
            if (swap_result and not swap_result.get('failed')
                    and not swap_result.get('probe_blocked')
                    and not _pn_tracks_cross_full(swap_result.get('new_segments', []),
                                                  pcb_data, p_net_id, n_net_id,
                                                  p_swap_positions=sw_pp, n_swap_positions=sw_nn)):
                swap_result['iterations'] = (swap_result.get('iterations', 0)
                                             + result.get('iterations', 0))
                return swap_result
            print("  Pad-swap retry did not produce a clean route either")
        print("  (terminals likely have opposite P/N pad order; needs a polarity "
              "swap, a layer change, or opposite-side joins)")
        out = {
            'failed': True,
            'iterations': result.get('iterations', 0),
            'pn_crossing': True,
        }
        # If the swap retry was itself rejected for grazing a committed foreign net,
        # propagate that so the caller can rip the grazed net and route this pair
        # first, then re-route the ripped net later (#246) -- the crossing is only
        # resolvable once the neighbour is out of the way.
        if (swap_result and swap_result.get('connector_graze')
                and swap_result.get('graze_net_id') is not None):
            out['connector_graze'] = True
            out['graze_net_id'] = swap_result['graze_net_id']
        return out

    # Issue #165: the connector/setback segments are clearance-checked during
    # routing against an obstacle map that excludes BOTH halves of the pair, so a
    # connector can graze the PARTNER net's pad (or other foreign copper) unseen.
    # Validate the final geometry against foreign pads with the SAME geometry and
    # tolerance as check_drc, so a reject here is a graze check_drc would also flag
    # (not tight-but-legal packing). Honest minimum: don't emit a known short.
    graze = _connector_grazes_foreign_copper(result.get('new_segments', []),
                                             pcb_data, p_net_id, n_net_id, config)
    if graze:
        kind, obj, seg, overlap = graze
        if kind == 'pad':
            what = f"pad {obj.component_ref}.{obj.pad_number} (net {obj.net_name})"
        elif kind == 'track':
            net = pcb_data.nets.get(obj.net_id)
            what = (f"track ({obj.start_x:.2f},{obj.start_y:.2f})-({obj.end_x:.2f},"
                    f"{obj.end_y:.2f}) (net {net.name if net else obj.net_id})")
        else:
            net = pcb_data.nets.get(obj.net_id)
            what = f"via at ({obj.x:.2f},{obj.y:.2f}) (net {net.name if net else obj.net_id})"
        print(f"  Connector grazes foreign {what} by {overlap:.3f}mm on {seg.layer} - "
              f"rejecting pair route (issue #165)")
        print("  (a connector/setback segment violates clearance to the partner or "
              "foreign copper; needs a cleaner connector route or a different "
              "terminal join)")
        return {
            'failed': True,
            'iterations': result.get('iterations', 0),
            'connector_graze': True,
            # The foreign net this route would graze/cross. The caller can rip it as
            # a blocker and re-route this pair clean, then re-route the ripped net
            # later (#246): there are no blocked cells to analyse here -- the route
            # SUCCEEDED but was rejected post-hoc -- so name the blocker explicitly.
            'graze_net_id': getattr(obj, 'net_id', None),
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
    # Use max track width for clearance since via connects layers with potentially different widths
    max_track_width = config.get_max_track_width()
    track_via_clearance = (config.clearance + max_track_width / 2 + config.via_size / 2) * config.routing_clearance_margin

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
