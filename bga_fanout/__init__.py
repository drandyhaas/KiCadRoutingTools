"""
BGA Fanout Strategy - Creates escape routing for BGA packages.

The BGA fanout creates:
1. 45-degree stubs from each pad to a routing channel
2. Horizontal or vertical channel segments to exit the BGA boundary
3. Smart layer assignment to avoid collisions on same channel

Key features:
- Generic: works with any BGA package
- Collision-free: tracks on same channel use different layers
- Assumes pads have vias connecting all layers (so any layer can be used)
- Validates no overlapping segments are created
- Differential pair support: P/N pairs routed together on same layer
"""
from __future__ import annotations

import math
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import routing_defaults as defaults
from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData, find_components_by_type
from net_queries import matches_net_filter
from list_nets import fab_floors, fab_floor_ladder, fab_floor_min, warn_fab_escalation
from kicad_writer import add_tracks_and_vias_to_pcb
from bga_fanout.types import (
    create_track,
    Channel,
    BGAGrid,
    DiffPairPads,
    FanoutRoute,
)
from bga_fanout.layer_balance import rebalance_layers
from bga_fanout.layer_assignment import assign_layers_smart
from bga_fanout.grid import (
    analyze_bga_grid,
    calculate_channels,
    is_edge_pad,
)
from bga_fanout.geometry import (
    create_45_stub,
    calculate_exit_point,
    calculate_jog_end,
    clamp_via_to_pad,
    immovable_foreign_pads,
    via_clears_pad_rects,
)
from bga_fanout.escape import (
    find_escape_channel,
    find_diff_pair_escape,
    assign_pair_escapes,
)
from bga_fanout.reroute import (
    find_existing_fanouts,
    resolve_collisions,
    repair_pad_crossings,
    route_clear_of_foreign_pads,
    clear_escapes_of_foreign_pads,
    segments_clear_of_pads,
)
from obstacle_map import build_base_obstacle_map, build_layer_map
from routing_config import GridRouteConfig
from bga_fanout.collision import check_segment_collision
from bga_fanout.diff_pair import find_differential_pairs
from bga_fanout.tracks import (
    detect_collisions,
    convert_segments_to_tracks,
    generate_tracks_from_routes,
)


# Public API
__all__ = [
    'generate_bga_fanout',
    'main',
    # Types re-exported for external use
    'BGAGrid',
    'Channel',
    'FanoutRoute',
    'DiffPairPads',
]


def calculate_jog_ends_for_routes(
    routes: List[FanoutRoute],
    layers: List[str],
    jog_length: float,
    track_width: float,
    diff_pair_gap: float,
    grid_step: float = 0.0,
    obstacles=None,
    cfg=None,
    layer_map: Dict[str, int] = None
) -> None:
    """
    Calculate jog_end positions for each route based on layer.

    For differential pairs, determines which track is on the outside of the bend
    and extends it appropriately to maintain spacing through the 45° turn.

    Modifies routes in-place, setting jog_end and jog_extension attributes.

    Args:
        routes: List of routes to process
        layers: Available routing layers
        jog_length: Length of the 45° jog at exit
        track_width: Track width
        diff_pair_gap: Gap between P and N traces
    """
    pair_spacing = track_width + diff_pair_gap  # Center-to-center spacing

    for route in routes:
        is_outside = False

        if route.pair_id:
            # Determine jog direction for this layer
            try:
                layer_idx = layers.index(route.layer)
            except ValueError:
                layer_idx = 0
            num_layers = len(layers)
            if num_layers <= 1:
                jog_direction = -1  # left
            else:
                layer_factor = layer_idx / (num_layers - 1)
                jog_direction = 2 * layer_factor - 1  # -1 = left, +1 = right

            # Determine if this route is on the outside of the bend
            if route.escape_dir in ['left', 'right']:
                # Horizontal escape, jog is in Y direction
                for other in routes:
                    if other.pair_id == route.pair_id and other is not route:
                        if route.escape_dir == 'right':
                            if jog_direction < 0:  # Jog goes up (-Y)
                                is_outside = route.exit_pos[1] > other.exit_pos[1]
                            else:  # Jog goes down (+Y)
                                is_outside = route.exit_pos[1] < other.exit_pos[1]
                        else:  # left
                            if jog_direction < 0:  # Jog goes down (+Y)
                                is_outside = route.exit_pos[1] < other.exit_pos[1]
                            else:  # Jog goes up (-Y)
                                is_outside = route.exit_pos[1] > other.exit_pos[1]
                        break
            else:
                # Vertical escape, jog is in X direction
                for other in routes:
                    if other.pair_id == route.pair_id and other is not route:
                        if route.escape_dir == 'down':
                            if jog_direction < 0:  # Jog goes right (+X)
                                is_outside = route.exit_pos[0] < other.exit_pos[0]
                            else:  # Jog goes left (-X)
                                is_outside = route.exit_pos[0] > other.exit_pos[0]
                        else:  # up
                            if jog_direction < 0:  # Jog goes left (-X)
                                is_outside = route.exit_pos[0] > other.exit_pos[0]
                            else:  # Jog goes right (+X)
                                is_outside = route.exit_pos[0] < other.exit_pos[0]
                        break

        jog_end, extension = calculate_jog_end(
            route.exit_pos,
            route.escape_dir,
            route.layer,
            layers,
            jog_length,
            is_diff_pair=route.pair_id is not None,
            is_outside_track=is_outside,
            pair_spacing=pair_spacing,
            grid_step=grid_step
        )
        route.jog_end = jog_end
        route.jog_extension = extension

        # Issue #149 part 2: validate the decorative end-jog against the FULL
        # obstacle map (foreign pads + existing copper + vias) AT CREATION, and
        # drop it if it would extend into a foreign obstacle. The jog does not
        # affect connectivity, so dropping it is connectivity-safe and avoids the
        # reroute cascade of rejecting whole candidates.
        if obstacles is not None and jog_end is not None:
            tail = []
            if extension is not None:
                tail.append({'start': route.exit_pos, 'end': extension})
                tail.append({'start': extension, 'end': jog_end})
            else:
                tail.append({'start': route.exit_pos, 'end': jog_end})
            if not segments_clear_of_pads(tail, route.layer, obstacles, cfg, layer_map):
                route.jog_end = None
                route.jog_extension = None


def print_route_statistics(routes: List[FanoutRoute]) -> None:
    """Print statistics about the generated routes."""
    print(f"  Found {len(routes)} pads to fanout")
    paired_count = sum(1 for r in routes if r.pair_id is not None)
    print(f"    {paired_count} are part of differential pairs")

    # Print escape direction distribution
    escape_counts = defaultdict(int)
    for r in routes:
        escape_counts[r.escape_dir] += 1
    print(f"  Escape direction distribution:")
    for direction in ['left', 'right', 'up', 'down']:
        if escape_counts[direction] > 0:
            print(f"    {direction}: {escape_counts[direction]}")

    # Print all net names being fanned out
    net_names = sorted(set(r.pad.net_name for r in routes if r.pad.net_name))
    print(f"  Nets being fanned out:")
    for name in net_names:
        print(f"    {name}")


POSITION_TOLERANCE = 0.01  # mm tolerance for position comparisons


def count_diff_pair_shorts(tracks: List[Dict], min_spacing: float) -> Tuple[int, Set[str]]:
    """Count diff-pair-vs-diff-pair (Z-Z) shorts among fanout tracks.

    A Z-Z short is an overlap between segments of two DIFFERENT diff-pair
    pair_ids (both non-None) on the SAME layer, within min_spacing.

    Returns (count_of_unordered_pair_id_combinations, set_of_involved_pair_ids).
    """
    short_combos: Set[Tuple[str, str]] = set()
    involved: Set[str] = set()
    for i, t1 in enumerate(tracks):
        pid1 = t1.get('pair_id')
        if not pid1:
            continue
        for t2 in tracks[i + 1:]:
            pid2 = t2.get('pair_id')
            if not pid2 or pid1 == pid2:
                continue
            if t1['layer'] != t2['layer']:
                continue
            if check_segment_collision(t1['start'], t1['end'],
                                       t2['start'], t2['end'],
                                       min_spacing):
                short_combos.add(tuple(sorted((pid1, pid2))))
                involved.add(pid1)
                involved.add(pid2)
    return len(short_combos), involved


def reassign_on_channel_pads(
    routes: List[FanoutRoute],
    channels: List[Channel],
    grid: BGAGrid,
    num_layers: int,
    exit_margin: float,
    footprint: 'Footprint' = None
) -> int:
    """
    Reassign on-channel pads to adjacent channels when their straight path is blocked.

    When a pad is exactly on a channel (stub_end == pad_pos), a straight track
    to the exit may pass through other pads. This function checks each on-channel
    pad for blocking pads and assigns jogs to adjacent channels as needed.

    Through-hole pads (including unconnected ones) block tracks on all layers,
    so we need to consider ALL pads in the footprint as potential blockers.

    Args:
        routes: List of routes (modified in-place)
        channels: Available routing channels
        grid: BGA grid info
        num_layers: Number of available layers (unused but kept for API compatibility)
        exit_margin: Distance past BGA boundary
        footprint: The footprint containing all pads (including unconnected)

    Returns:
        Number of routes reassigned to adjacent channels
    """
    reassigned = 0

    # Build a set of all pad positions - use footprint pads if available (includes unconnected)
    # otherwise fall back to route pad positions
    all_pad_positions = set()
    if footprint is not None:
        for pad in footprint.pads:
            px = round(pad.global_x, 3)
            py = round(pad.global_y, 3)
            all_pad_positions.add((px, py))
    else:
        for route in routes:
            # Round to avoid floating point issues
            px = round(route.pad_pos[0], 3)
            py = round(route.pad_pos[1], 3)
            all_pad_positions.add((px, py))

    # Find on-channel pads (zero-length stubs) that need to be reassigned
    routes_to_reassign = []
    for route in routes:
        if route.is_edge or route.pair_id is not None:
            continue
        dx = abs(route.stub_end[0] - route.pad_pos[0])
        dy = abs(route.stub_end[1] - route.pad_pos[1])
        if dx >= POSITION_TOLERANCE or dy >= POSITION_TOLERANCE:
            continue  # Not an on-channel pad

        if route.channel is None:
            continue

        # Check if there are any pads blocking the straight path to exit
        has_blocking_pad = False
        px, py = route.pad_pos

        if route.escape_dir == 'left':
            # Check for pads at same Y but smaller X (between pad and left exit)
            for other_x, other_y in all_pad_positions:
                if abs(other_y - py) < POSITION_TOLERANCE and other_x < px - POSITION_TOLERANCE:
                    has_blocking_pad = True
                    break
        elif route.escape_dir == 'right':
            # Check for pads at same Y but larger X
            for other_x, other_y in all_pad_positions:
                if abs(other_y - py) < POSITION_TOLERANCE and other_x > px + POSITION_TOLERANCE:
                    has_blocking_pad = True
                    break
        elif route.escape_dir == 'up':
            # Check for pads at same X but smaller Y
            for other_x, other_y in all_pad_positions:
                if abs(other_x - px) < POSITION_TOLERANCE and other_y < py - POSITION_TOLERANCE:
                    has_blocking_pad = True
                    break
        else:  # down
            # Check for pads at same X but larger Y
            for other_x, other_y in all_pad_positions:
                if abs(other_x - px) < POSITION_TOLERANCE and other_y > py + POSITION_TOLERANCE:
                    has_blocking_pad = True
                    break

        if has_blocking_pad:
            routes_to_reassign.append(route)

    if not routes_to_reassign:
        return 0

    # Reassign each blocked route to an adjacent channel
    # Sort h_channels and v_channels once for efficiency
    h_channels = sorted([c for c in channels if c.orientation == 'horizontal'],
                       key=lambda c: c.position)
    v_channels = sorted([c for c in channels if c.orientation == 'vertical'],
                       key=lambda c: c.position)

    for route in routes_to_reassign:
        orientation = route.channel.orientation
        channel_pos = route.channel.position
        escape_dir = route.escape_dir

        if orientation == 'horizontal':
            # Find current channel index
            current_idx = None
            for i, c in enumerate(h_channels):
                if abs(c.position - channel_pos) < POSITION_TOLERANCE:
                    current_idx = i
                    break

            if current_idx is None:
                continue

            # Determine which adjacent channel to use based on pad position relative to grid center
            # Pads above center jog down, pads below center jog up
            if route.pad_pos[1] < grid.center_y:
                # Pad is above center, jog down (higher Y = higher index)
                new_idx = current_idx + 1
            else:
                # Pad is below center, jog up (lower Y = lower index)
                new_idx = current_idx - 1

            # Bounds check and fallback
            if new_idx < 0:
                new_idx = current_idx + 1
            elif new_idx >= len(h_channels):
                new_idx = current_idx - 1

            if new_idx < 0 or new_idx >= len(h_channels):
                continue  # No valid adjacent channel

            new_channel = h_channels[new_idx]

            # Create 45° jog from pad to new channel
            dy = new_channel.position - route.pad_pos[1]
            if escape_dir == 'right':
                dx = abs(dy)  # Move right while jogging
            else:  # left
                dx = -abs(dy)  # Move left while jogging

            jog_point = (route.pad_pos[0] + dx, new_channel.position)

            # Calculate new exit position
            if escape_dir == 'right':
                new_exit = (grid.max_x + exit_margin, new_channel.position)
            else:  # left
                new_exit = (grid.min_x - exit_margin, new_channel.position)

            # Update route
            route.stub_end = route.pad_pos  # No initial stub
            route.pre_channel_jog = jog_point
            route.exit_pos = new_exit
            route.channel = new_channel
            reassigned += 1

        else:  # vertical channels
            # Find current channel index
            current_idx = None
            for i, c in enumerate(v_channels):
                if abs(c.position - channel_pos) < POSITION_TOLERANCE:
                    current_idx = i
                    break

            if current_idx is None:
                continue

            # Determine which adjacent channel to use based on pad position relative to grid center
            # Pads left of center jog right, pads right of center jog left
            if route.pad_pos[0] < grid.center_x:
                # Pad is left of center, jog right (higher X = higher index)
                new_idx = current_idx + 1
            else:
                # Pad is right of center, jog left (lower X = lower index)
                new_idx = current_idx - 1

            # Bounds check and fallback
            if new_idx < 0:
                new_idx = current_idx + 1
            elif new_idx >= len(v_channels):
                new_idx = current_idx - 1

            if new_idx < 0 or new_idx >= len(v_channels):
                continue

            new_channel = v_channels[new_idx]

            # Create 45° jog from pad to new channel
            dx = new_channel.position - route.pad_pos[0]
            if escape_dir == 'down':
                dy = abs(dx)  # Move down while jogging
            else:  # up
                dy = -abs(dx)  # Move up while jogging

            jog_point = (new_channel.position, route.pad_pos[1] + dy)

            # Calculate new exit position
            if escape_dir == 'down':
                new_exit = (new_channel.position, grid.max_y + exit_margin)
            else:  # up
                new_exit = (new_channel.position, grid.min_y - exit_margin)

            # Update route
            route.stub_end = route.pad_pos
            route.pre_channel_jog = jog_point
            route.exit_pos = new_exit
            route.channel = new_channel
            reassigned += 1

    return reassigned


def connect_adjacent_same_net_pads(
    routes: List[FanoutRoute],
    grid: BGAGrid,
    track_width: float,
    clearance: float
) -> int:
    """
    Connect adjacent pads on the same net directly instead of separate fanouts.

    When two pads on the same net are adjacent (within 1 pitch), one can connect
    directly to the other's fanout instead of having its own full fanout to the
    BGA boundary. This simplifies routing by avoiding disconnected stubs.

    Args:
        routes: List of routes (modified in-place)
        grid: BGA grid info
        track_width: Track width for clearance calculations
        clearance: Minimum clearance between tracks

    Returns:
        Number of routes modified to connect to neighbors
    """
    # Group routes by net_id
    routes_by_net: Dict[int, List[FanoutRoute]] = {}
    for route in routes:
        net_id = route.net_id
        if net_id not in routes_by_net:
            routes_by_net[net_id] = []
        routes_by_net[net_id].append(route)

    connected = 0
    pitch = max(grid.pitch_x, grid.pitch_y)
    adjacency_threshold = pitch * 1.5  # Adjacent = within ~1.5 pitch

    for net_id, net_routes in routes_by_net.items():
        if len(net_routes) < 2:
            continue

        # Find adjacent pairs
        # Keep track of which routes have been connected as "secondary"
        connected_as_secondary = set()

        for i, route1 in enumerate(net_routes):
            if i in connected_as_secondary:
                continue

            for j, route2 in enumerate(net_routes[i+1:], i+1):
                if j in connected_as_secondary:
                    continue

                # Check if pads are adjacent
                dx = abs(route1.pad_pos[0] - route2.pad_pos[0])
                dy = abs(route1.pad_pos[1] - route2.pad_pos[1])
                dist = (dx**2 + dy**2) ** 0.5

                if dist > adjacency_threshold:
                    continue

                # Pads are adjacent - connect route2 directly to route1's pad
                # Use a simple direct connection (may be diagonal)
                # The "secondary" route just goes pad2 -> pad1 (no fanout)

                # Mark route2 as connecting to route1
                # Set route2's exit to route1's pad position
                # This creates a short link between the two pads
                route2.stub_end = route1.pad_pos
                route2.exit_pos = route1.pad_pos
                route2.channel = None  # No channel needed
                route2.is_edge = True  # Treat as edge (direct connection)
                route2.neighbor_connection = True  # Mark as neighbor connection
                # Use same layer as route1 for the connection
                route2.layer = route1.layer

                connected_as_secondary.add(j)
                connected += 1

    return connected


def create_single_ended_route(
    pad: Pad,
    grid: BGAGrid,
    channels: List[Channel],
    layers: List[str],
    exit_margin: float,
    force_orientation: Optional[str] = None
) -> FanoutRoute:
    """
    Create a route for a single-ended (non-differential) signal.

    Args:
        pad: The pad to route
        grid: BGA grid parameters
        channels: Available routing channels
        layers: Available routing layers
        exit_margin: Distance past BGA boundary
        force_orientation: If set, force 'horizontal' or 'vertical' escape

    Returns:
        FanoutRoute for this pad
    """
    channel, escape_dir = find_escape_channel(
        pad.global_x, pad.global_y, grid, channels,
        force_orientation=force_orientation
    )
    is_edge = channel is None

    if is_edge:
        if escape_dir == 'right':
            exit_pos = (grid.max_x + exit_margin, pad.global_y)
        elif escape_dir == 'left':
            exit_pos = (grid.min_x - exit_margin, pad.global_y)
        elif escape_dir == 'down':
            exit_pos = (pad.global_x, grid.max_y + exit_margin)
        else:  # up
            exit_pos = (pad.global_x, grid.min_y - exit_margin)
        stub_end = exit_pos
    else:
        stub_end = create_45_stub(pad.global_x, pad.global_y, channel, escape_dir)
        exit_pos = calculate_exit_point(stub_end, channel, escape_dir, grid, exit_margin)

    return FanoutRoute(
        pad=pad,
        pad_pos=(pad.global_x, pad.global_y),
        stub_end=stub_end,
        exit_pos=exit_pos,
        channel=channel,
        escape_dir=escape_dir,
        is_edge=is_edge,
        layer=layers[0],
        pair_id=None,
        is_p=True
    )


def manage_vias(
    routes: List[FanoutRoute],
    pcb_data: 'PCBData',
    top_layer: str,
    via_size: float,
    via_drill: float,
    clearance: float,
) -> Tuple[List[Dict], List[Dict]]:
    """
    Manage vias for fanout routes.

    Determines which vias need to be added (for routes on non-top layers)
    and which can be removed (for routes on top layer).

    Args:
        routes: List of FanoutRoute objects
        pcb_data: PCB data containing existing vias
        top_layer: Name of the top layer (e.g., 'F.Cu')
        via_size: Size of vias to add
        via_drill: Drill size for vias
        clearance: Minimum clearance between vias

    Returns:
        Tuple of (vias_to_add, vias_to_remove)
    """
    def find_nearby_via(x: float, y: float, net_id: int, max_dist: float):
        """Find an existing via on the same net within max_dist of position."""
        for via in pcb_data.vias:
            if via.net_id != net_id:
                continue
            dist = math.sqrt((via.x - x)**2 + (via.y - y)**2)
            if dist <= max_dist:
                return via
        return None

    def would_overlap_existing_via(x: float, y: float, new_via_size: float) -> bool:
        """Check if a new via at (x,y) would overlap within clearance of any existing via."""
        for via in pcb_data.vias:
            dist = math.sqrt((via.x - x)**2 + (via.y - y)**2)
            min_dist = (via.size / 2) + (new_via_size / 2) + clearance
            if dist < min_dist:
                return True
        return False

    vias_to_add: List[Dict] = []
    vias_to_remove: List[Dict] = []
    via_blocked_routes: List['FanoutRoute'] = []

    # Pads no later step can move (locked parts, connectors, test points --
    # #253): a via-in-pad must clear them on EVERY layer, because a through
    # via's ring exists on all of them (a back-side connector pad under the
    # ball field is invisible to the per-layer track checks). Movable caps are
    # deliberately absent: place_fanout_clearance moves them off the vias.
    exclude_ref = routes[0].pad.component_ref if routes else ''
    immovable_pads = immovable_foreign_pads(pcb_data, exclude_ref)

    # Fab floors for the via-in-pad clamp (#202): floor is a board property
    # (2- vs 4-layer), so key off the board's total copper count. Pass the active
    # fab-tier ladder so the clamp escalates standard->advanced when a sub-0.45mm
    # pad can't take the standard via (issue #237).
    copper = len(getattr(pcb_data.board_info, 'copper_layers', None) or []) or 4
    floors = fab_floor_ladder(copper)
    clamped_count = 0
    floor_pads = 0
    escalated_count = 0

    # Check distance threshold: via is "at pad" if within via radius + small tolerance
    via_proximity_threshold = via_size / 2 + 0.1

    for route in routes:
        pad_x, pad_y = route.pad_pos
        existing_via = find_nearby_via(pad_x, pad_y, route.net_id, via_proximity_threshold)

        # Through-hole pads (drill > 0) already connect all layers, no via needed
        is_through_hole = route.pad.drill > 0

        if route.layer == top_layer:
            # Routing on top layer - no via needed at pad
            if existing_via:
                vias_to_remove.append({
                    'x': existing_via.x,
                    'y': existing_via.y
                })
        else:
            # Routing on inner/bottom layer - via needed only for SMD pads
            if not is_through_hole and not existing_via:
                # Size the via to fit its pad up front (#202) so it never bulges
                # past the pad edge into a neighbouring different-net trace.
                v_size, v_drill, status, rung = clamp_via_to_pad(
                    via_size, via_drill, route.pad, floors)
                if status == 'clamped':
                    clamped_count += 1
                elif status == 'floor':
                    floor_pads += 1
                if rung > 0:
                    escalated_count += 1
                if not via_clears_pad_rects(pad_x, pad_y, v_size / 2.0,
                                            clearance, immovable_pads):
                    # No via can go here: an immovable foreign pad (locked
                    # part / connector / test point) overlaps the through
                    # via's ring on some layer (#253, ottercast J2 connector,
                    # orangecrab TP20 test pad). Fail this escape honestly --
                    # the main router can still route the net from the bare
                    # ball on the pad's own layer.
                    via_blocked_routes.append(route)
                    continue
                if not would_overlap_existing_via(pad_x, pad_y, v_size):
                    vias_to_add.append({
                        'x': pad_x,
                        'y': pad_y,
                        'size': v_size,
                        'drill': v_drill,
                        'layers': ['F.Cu', 'B.Cu'],
                        'net_id': route.net_id
                    })

    if vias_to_add:
        print(f"  Adding {len(vias_to_add)} vias at pads on non-top layers")
    if clamped_count:
        print(f"  Clamped {clamped_count} via-in-pad(s) to fit their pad edge (#202)")
    if escalated_count:
        warn_fab_escalation(f"{escalated_count} via-in-pad(s) (sub-0.45mm pads)")
    if floor_pads:
        print(f"  WARNING: {floor_pads} pad(s) smaller than the fab via floor "
              f"({fab_floor_min(copper)['via_diameter']:.2f}mm dia); via held at the "
              f"floor and still bulges past the pad edge")
    if vias_to_remove:
        print(f"  Removing {len(vias_to_remove)} unnecessary vias at pads on top layer")
    if via_blocked_routes:
        names = sorted(r.pad.net_name or f"net{r.net_id}" for r in via_blocked_routes)
        print(f"  WARNING: {len(via_blocked_routes)} escape(s) dropped: via-in-pad "
              f"would hit an immovable foreign pad (locked part/connector/test "
              f"point, #253): {', '.join(names)}")

    return vias_to_add, vias_to_remove, via_blocked_routes


def select_adjacent_channels(
    channel: 'Channel',
    escape_dir: str,
    p_pad: Pad,
    n_pad: Pad,
    channels: List[Channel],
    use_adjacent_channels_h: bool,
    use_adjacent_channels_v: bool,
    is_cross_escape: bool,
    is_edge: bool,
) -> Tuple[Optional['Channel'], Optional['Channel']]:
    """Choose the two channels (p_channel, n_channel) for a diff pair.

    For adjacent-channel mode, returns two channels straddling the pads (one on
    each side), so each track is centered in its own channel. Otherwise both
    legs share the single assigned channel. Behavior identical to the inlined
    logic it replaces.
    """
    p_channel = channel
    n_channel = channel
    needs_adjacent = ((escape_dir in ['left', 'right'] and use_adjacent_channels_h) or
                      (escape_dir in ['up', 'down'] and use_adjacent_channels_v))
    if needs_adjacent and channel and not is_cross_escape and not is_edge:
        # Find two adjacent channels for the diff pair - one above, one below the pads
        if channel.orientation == 'horizontal':
            # Horizontal pads escaping left/right - use channels on OPPOSITE sides of pads
            # One track goes UP to channel above, other goes DOWN to channel below
            h_channels = [c for c in channels if c.orientation == 'horizontal']
            h_channels_sorted = sorted(h_channels, key=lambda c: c.position)
            pad_y = (p_pad.global_y + n_pad.global_y) / 2

            # Find channels above and below the pads
            channels_above = [c for c in h_channels_sorted if c.position < pad_y]
            channels_below = [c for c in h_channels_sorted if c.position > pad_y]

            if channels_above and channels_below:
                # Use closest channel above for one pad, closest below for the other
                ch_above = channels_above[-1]  # closest above
                ch_below = channels_below[0]   # closest below
                # P/t goes to channel below, N/c goes to channel above
                p_channel = ch_below
                n_channel = ch_above
            elif channels_above:
                # Only channels above - use two adjacent ones
                ch_above = channels_above[-1]
                idx = h_channels_sorted.index(ch_above)
                if idx > 0:
                    p_channel = h_channels_sorted[idx - 1]
                    n_channel = ch_above
                else:
                    p_channel = channel
                    n_channel = channel
            elif channels_below:
                # Only channels below - use two adjacent ones
                ch_below = channels_below[0]
                idx = h_channels_sorted.index(ch_below)
                if idx < len(h_channels_sorted) - 1:
                    p_channel = ch_below
                    n_channel = h_channels_sorted[idx + 1]
                else:
                    p_channel = channel
                    n_channel = channel
            else:
                p_channel = channel
                n_channel = channel
        else:
            # Vertical pads escaping up/down - use channels on OPPOSITE sides of pads
            v_channels = [c for c in channels if c.orientation == 'vertical']
            v_channels_sorted = sorted(v_channels, key=lambda c: c.position)
            pad_x = (p_pad.global_x + n_pad.global_x) / 2

            # Find channels left and right of the pads
            channels_left = [c for c in v_channels_sorted if c.position < pad_x]
            channels_right = [c for c in v_channels_sorted if c.position > pad_x]

            if channels_left and channels_right:
                ch_left = channels_left[-1]   # closest left
                ch_right = channels_right[0]  # closest right
                # P/t goes right, N/c goes left (consistent with horizontal)
                p_channel = ch_right
                n_channel = ch_left
            elif channels_left:
                ch_left = channels_left[-1]
                idx = v_channels_sorted.index(ch_left)
                if idx > 0:
                    p_channel = v_channels_sorted[idx - 1]
                    n_channel = ch_left
                else:
                    p_channel = channel
                    n_channel = channel
            elif channels_right:
                ch_right = channels_right[0]
                idx = v_channels_sorted.index(ch_right)
                if idx < len(v_channels_sorted) - 1:
                    p_channel = ch_right
                    n_channel = v_channels_sorted[idx + 1]
                else:
                    p_channel = channel
                    n_channel = channel
            else:
                p_channel = channel
                n_channel = channel

    return p_channel, n_channel


def compute_pair_offsets(
    p_channel: Optional['Channel'],
    n_channel: Optional['Channel'],
    channel: Optional['Channel'],
    is_cross_escape: bool,
    escape_dir: str,
    p_pad: Pad,
    n_pad: Pad,
    half_pair_spacing: float,
) -> Tuple[float, float]:
    """Compute the per-leg P/N offsets from channel center.

    In adjacent-channel mode each track is centered in its own channel (0
    offset). Otherwise the pad closer to the escape edge gets the inner offset
    and the farther pad the outer offset. Edge/cross-escape legs use 0 (they
    converge via 45 stubs). Behavior identical to the inlined logic.
    """
    if p_channel != n_channel:
        # Adjacent channel mode - each track centered in its own channel
        p_offset = 0
        n_offset = 0
    elif channel and channel.orientation == 'horizontal' and not is_cross_escape:
        # Horizontal channel - pads are horizontally adjacent, escaping left/right
        # Traces will be offset in Y (one above, one below channel center)
        # Rule: pad closer to escape edge goes to inner side (closer to pads),
        #       pad further from edge goes to outer side (away from pads)
        channel_above = channel.position < p_pad.global_y
        p_is_left = p_pad.global_x < n_pad.global_x

        if escape_dir == 'left':
            # Escaping left - pad on left (smaller X) is closer to edge
            pad_closer_to_edge_is_p = p_is_left
        else:  # right
            # Escaping right - pad on right (larger X) is closer to edge
            pad_closer_to_edge_is_p = not p_is_left

        if channel_above:
            # Channel is above pads - inner side is below (positive offset)
            if pad_closer_to_edge_is_p:
                p_offset = half_pair_spacing   # P closer to edge -> inner (below)
                n_offset = -half_pair_spacing  # N further -> outer (above)
            else:
                p_offset = -half_pair_spacing  # P further -> outer (above)
                n_offset = half_pair_spacing   # N closer to edge -> inner (below)
        else:
            # Channel is below pads - inner side is above (negative offset)
            if pad_closer_to_edge_is_p:
                p_offset = -half_pair_spacing  # P closer to edge -> inner (above)
                n_offset = half_pair_spacing   # N further -> outer (below)
            else:
                p_offset = half_pair_spacing   # P further -> outer (below)
                n_offset = -half_pair_spacing  # N closer to edge -> inner (above)
    elif channel and channel.orientation == 'vertical' and not is_cross_escape:
        # Vertical channel - pads are vertically adjacent, escaping up/down
        # Traces will be offset in X (one left, one right of channel center)
        # Rule: pad closer to escape edge goes to inner side (closer to pads),
        #       pad further from edge goes to outer side (away from pads)
        channel_right = channel.position > p_pad.global_x
        p_is_above = p_pad.global_y < n_pad.global_y

        if escape_dir == 'up':
            # Escaping up - pad above (smaller Y) is closer to edge
            pad_closer_to_edge_is_p = p_is_above
        else:  # down
            # Escaping down - pad below (larger Y) is closer to edge
            pad_closer_to_edge_is_p = not p_is_above

        if channel_right:
            # Channel is right of pads - inner side is left (negative offset)
            if pad_closer_to_edge_is_p:
                p_offset = -half_pair_spacing  # P closer to edge -> inner (left)
                n_offset = half_pair_spacing   # N further -> outer (right)
            else:
                p_offset = half_pair_spacing   # P further -> outer (right)
                n_offset = -half_pair_spacing  # N closer to edge -> inner (left)
        else:
            # Channel is left of pads - inner side is right (positive offset)
            if pad_closer_to_edge_is_p:
                p_offset = half_pair_spacing   # P closer to edge -> inner (right)
                n_offset = -half_pair_spacing  # N further -> outer (left)
            else:
                p_offset = -half_pair_spacing  # P further -> outer (left)
                n_offset = half_pair_spacing   # N closer to edge -> inner (right)
    else:
        # Edge pads or cross-escape - no offset needed, they converge with 45 stubs
        p_offset = 0
        n_offset = 0

    return p_offset, n_offset


def build_half_edge_route(
    pad_info: Pad,
    is_p_route: bool,
    p_pad: Pad,
    n_pad: Pad,
    actual_escape_dir: str,
    grid: BGAGrid,
    channels: List[Channel],
    layers: List[str],
    exit_margin: float,
    half_pair_spacing: float,
    pair_id: str,
) -> FanoutRoute:
    """Build one leg of a half-edge diff pair (one pad on edge, one inner).

    The edge pad goes straight out; the inner pad makes a tent (45 up to a
    channel, one pitch along it, 45 back down) to converge with the edge pad at
    pair spacing. Behavior identical to the inlined logic.
    """
    # Half-edge pair: one pad on edge, one inner
    # Edge pad: goes straight out to BGA edge
    # Inner pad: 45 up to channel center, then 45 back down to converge
    #            with edge pad at pair spacing
    #
    # The inner pad makes a "tent" shape to go around the via pad

    is_edge_p_check, _ = is_edge_pad(p_pad.global_x, p_pad.global_y, grid)
    is_edge_n_check, _ = is_edge_pad(n_pad.global_x, n_pad.global_y, grid)

    # Identify which pad is edge and which is inner
    if is_edge_p_check:
        edge_pad_info = p_pad
        inner_pad_info = n_pad
        edge_is_p = True
    else:
        edge_pad_info = n_pad
        inner_pad_info = p_pad
        edge_is_p = False

    this_pad_is_edge = (is_p_route and edge_is_p) or (not is_p_route and not edge_is_p)
    pair_spacing_full = 2 * half_pair_spacing

    if actual_escape_dir in ['left', 'right']:
        # Find channel between inner pad and edge pad (horizontally adjacent)
        h_channels = [c for c in channels if c.orientation == 'horizontal']
        inner_y = inner_pad_info.global_y
        edge_y = edge_pad_info.global_y

        # Channel should be between the two pads OR closest to inner going away from edge
        # Since they're on same row, find channel above or below
        channels_above = [c for c in h_channels if c.position < inner_y]
        channels_below = [c for c in h_channels if c.position > inner_y]

        # Choose channel direction based on distance to BGA edge (only used by
        # the same-row tent below; diagonal pairs converge directly).
        dist_to_top = inner_y - grid.min_y
        dist_to_bottom = grid.max_y - inner_y

        if dist_to_top <= dist_to_bottom and channels_above:
            inner_channel = max(channels_above, key=lambda c: c.position)
            channel_above = True
        elif channels_below:
            inner_channel = min(channels_below, key=lambda c: c.position)
            channel_above = False
        else:
            inner_channel = max(channels_above, key=lambda c: c.position)
            channel_above = True

        if this_pad_is_edge:
            # Edge pad: straight out horizontally
            # stub_end = pad position (no stub needed)
            stub_end = (edge_pad_info.global_x, edge_pad_info.global_y)
            if actual_escape_dir == 'right':
                exit_pos = (grid.max_x + exit_margin, edge_pad_info.global_y)
            else:
                exit_pos = (grid.min_x - exit_margin, edge_pad_info.global_y)
            route_channel = None
            channel_pt = None
            channel_pt2 = None
        else:
            # Inner pad tent: 45 to a channel, 1 pitch toward the edge, then 45
            # to converge with the edge pad at pair spacing.
            #
            # Converge on the inner pad's OWN side of the edge pad so the P/N
            # stubs don't cross (#242), and route the tent through the channel
            # BETWEEN the two pads (toward the edge pad) so the return stub
            # doesn't overshoot the BGA edge and loop back toward the BGA
            # (#242). For a same-row pair (no "between" channel) keep the
            # nearest-edge heuristic computed above.
            if inner_y > edge_y + POSITION_TOLERANCE and channels_above:
                inner_channel = max(channels_above, key=lambda c: c.position)
                target_exit_y = edge_pad_info.global_y + pair_spacing_full
            elif inner_y < edge_y - POSITION_TOLERANCE and channels_below:
                inner_channel = min(channels_below, key=lambda c: c.position)
                target_exit_y = edge_pad_info.global_y - pair_spacing_full
            elif channel_above:
                target_exit_y = edge_pad_info.global_y - pair_spacing_full
            else:
                target_exit_y = edge_pad_info.global_y + pair_spacing_full

            channel_y = inner_channel.position
            dy_to_channel = channel_y - inner_pad_info.global_y

            if actual_escape_dir == 'right':
                channel_pt_x = inner_pad_info.global_x + abs(dy_to_channel)
                channel_pt = (channel_pt_x, channel_y)
                channel_pt2_x = channel_pt_x + grid.pitch_x
                channel_pt2 = (channel_pt2_x, channel_y)
                dy_return = target_exit_y - channel_y
                stub_end_x = channel_pt2_x + abs(dy_return)
                stub_end = (stub_end_x, target_exit_y)
                exit_pos = (grid.max_x + exit_margin, target_exit_y)
            else:  # left
                channel_pt_x = inner_pad_info.global_x - abs(dy_to_channel)
                channel_pt = (channel_pt_x, channel_y)
                channel_pt2_x = channel_pt_x - grid.pitch_x
                channel_pt2 = (channel_pt2_x, channel_y)
                dy_return = target_exit_y - channel_y
                stub_end_x = channel_pt2_x - abs(dy_return)
                stub_end = (stub_end_x, target_exit_y)
                exit_pos = (grid.min_x - exit_margin, target_exit_y)

            route_channel = inner_channel

    else:
        # Vertical escape - similar logic but X/Y swapped
        v_channels = [c for c in channels if c.orientation == 'vertical']
        inner_x = inner_pad_info.global_x
        edge_x = edge_pad_info.global_x

        channels_left = [c for c in v_channels if c.position < inner_x]
        channels_right = [c for c in v_channels if c.position > inner_x]

        # Choose channel direction based on distance to BGA edge (only used by
        # the same-column tent below; diagonal pairs converge directly).
        dist_to_left = inner_x - grid.min_x
        dist_to_right = grid.max_x - inner_x

        if dist_to_left <= dist_to_right and channels_left:
            inner_channel = max(channels_left, key=lambda c: c.position)
            channel_left = True
        elif channels_right:
            inner_channel = min(channels_right, key=lambda c: c.position)
            channel_left = False
        else:
            inner_channel = max(channels_left, key=lambda c: c.position)
            channel_left = True

        if this_pad_is_edge:
            stub_end = (edge_pad_info.global_x, edge_pad_info.global_y)
            if actual_escape_dir == 'down':
                exit_pos = (edge_pad_info.global_x, grid.max_y + exit_margin)
            else:
                exit_pos = (edge_pad_info.global_x, grid.min_y - exit_margin)
            route_channel = None
            channel_pt = None
            channel_pt2 = None
        else:
            # Inner pad tent (X/Y swapped vs the horizontal case). Converge on
            # the inner pad's own side of the edge pad and route through the
            # channel between the two pads so the return stub doesn't overshoot
            # and loop back toward the BGA (#242).
            if inner_x > edge_x + POSITION_TOLERANCE and channels_left:
                inner_channel = max(channels_left, key=lambda c: c.position)
                target_exit_x = edge_pad_info.global_x + pair_spacing_full
            elif inner_x < edge_x - POSITION_TOLERANCE and channels_right:
                inner_channel = min(channels_right, key=lambda c: c.position)
                target_exit_x = edge_pad_info.global_x - pair_spacing_full
            elif channel_left:
                target_exit_x = edge_pad_info.global_x - pair_spacing_full
            else:
                target_exit_x = edge_pad_info.global_x + pair_spacing_full

            channel_x = inner_channel.position
            dx_to_channel = channel_x - inner_pad_info.global_x

            if actual_escape_dir == 'down':
                channel_pt_y = inner_pad_info.global_y + abs(dx_to_channel)
                channel_pt = (channel_x, channel_pt_y)
                channel_pt2_y = channel_pt_y + grid.pitch_y
                channel_pt2 = (channel_x, channel_pt2_y)
                dx_return = target_exit_x - channel_x
                stub_end_y = channel_pt2_y + abs(dx_return)
                stub_end = (target_exit_x, stub_end_y)
                exit_pos = (target_exit_x, grid.max_y + exit_margin)
            else:  # up
                channel_pt_y = inner_pad_info.global_y - abs(dx_to_channel)
                channel_pt = (channel_x, channel_pt_y)
                channel_pt2_y = channel_pt_y - grid.pitch_y
                channel_pt2 = (channel_x, channel_pt2_y)
                dx_return = target_exit_x - channel_x
                stub_end_y = channel_pt2_y - abs(dx_return)
                stub_end = (target_exit_x, stub_end_y)
                exit_pos = (target_exit_x, grid.min_y - exit_margin)

            route_channel = inner_channel

    return FanoutRoute(
        pad=pad_info,
        pad_pos=(pad_info.global_x, pad_info.global_y),
        stub_end=stub_end,
        exit_pos=exit_pos,
        channel_point=channel_pt if not this_pad_is_edge else None,
        channel_point2=channel_pt2 if not this_pad_is_edge else None,
        channel=route_channel,
        escape_dir=actual_escape_dir,
        is_edge=this_pad_is_edge,
        layer=layers[0],
        pair_id=pair_id,
        is_p=is_p_route
    )


def build_converge_route(
    pad_info: Pad,
    is_p_route: bool,
    p_pad: Pad,
    n_pad: Pad,
    pads_horizontal: bool,
    escape_dir: str,
    is_edge: bool,
    channel: Optional['Channel'],
    grid: BGAGrid,
    channels: List[Channel],
    layers: List[str],
    exit_margin: float,
    half_pair_spacing: float,
    use_adjacent_channels_h: bool,
    pair_layer_assignments: Dict,
    pair_id: str,
) -> FanoutRoute:
    """Build one leg of an edge / cross-escape diff pair (converging 45 stubs).

    Pads converge to pair spacing via 45 stubs. Handles the horizontal-pads and
    vertical-pads (cross-escape) cases, including the adjacent-channel cross
    variant. Behavior identical to the inlined logic.
    """
    # Edge pads or cross-escape: converge with 45 stubs to meet at pair spacing
    # Cross-escape: horizontal pads escaping vertically, or vertical pads escaping horizontally
    # Calculate the center point between P and N pads
    center_x = (p_pad.global_x + n_pad.global_x) / 2
    center_y = (p_pad.global_y + n_pad.global_y) / 2

    route_ch = channel

    if pads_horizontal:
        # Pads are side by side horizontally (like T9 and T10 in screenshot)
        # They need to converge to pair spacing using 45 stubs
        # Final X positions: center_x +/- half_pair_spacing

        # Determine which pad is on the left vs right
        p_is_left = p_pad.global_x < n_pad.global_x

        # Target X for converged pair - left pad goes to left target, right pad to right target
        if (is_p_route and p_is_left) or (not is_p_route and not p_is_left):
            # This pad is on the left, target is left of center
            target_x = center_x - half_pair_spacing
        else:
            # This pad is on the right, target is right of center
            target_x = center_x + half_pair_spacing

        # Distance each trace needs to move in X (towards center)
        dx_needed = target_x - pad_info.global_x

        # At 45, dy = dx (in absolute terms, direction depends on escape)
        if escape_dir == 'down':
            # Going down: Y increases, stub goes at 45 down
            stub_end_y = pad_info.global_y + abs(dx_needed)
            stub_end_x = target_x
        elif escape_dir == 'up':
            # Going up: Y decreases, stub goes at 45 up
            stub_end_y = pad_info.global_y - abs(dx_needed)
            stub_end_x = target_x
        else:
            # For left/right edge with horizontal pads, shouldn't happen normally
            stub_end_x = target_x
            stub_end_y = pad_info.global_y

        stub_end = (stub_end_x, stub_end_y)

        # Exit position continues in escape direction
        if escape_dir == 'down':
            exit_pos = (stub_end[0], grid.max_y + exit_margin)
        elif escape_dir == 'up':
            exit_pos = (stub_end[0], grid.min_y - exit_margin)
        elif escape_dir == 'right':
            exit_pos = (grid.max_x + exit_margin, stub_end[1])
        else:  # left
            exit_pos = (grid.min_x - exit_margin, stub_end[1])
    else:
        # Pads are vertically adjacent, escaping horizontally (cross-escape)
        # Determine which pad is on the top vs bottom (smaller Y = top in KiCad)
        p_is_top = p_pad.global_y < n_pad.global_y

        if use_adjacent_channels_h and escape_dir in ['left', 'right']:
            # Adjacent-channel mode for cross-escape: use two ADJACENT channels
            # Find the channel between the two pads, then use it and the next one
            # on the escape side (both tracks go same direction, different adjacent channels)
            h_channels = [c for c in channels if c.orientation == 'horizontal']
            h_channels_sorted = sorted(h_channels, key=lambda c: c.position)

            # Find the channel between the two pads (between their Y positions)
            top_pad_y = min(p_pad.global_y, n_pad.global_y)
            bot_pad_y = max(p_pad.global_y, n_pad.global_y)
            channels_between = [c for c in h_channels_sorted
                               if top_pad_y < c.position < bot_pad_y]

            if channels_between:
                # Use the channel between pads and one adjacent to it
                between_ch = channels_between[0]
                between_idx = h_channels_sorted.index(between_ch)

                # Determine which pad is closer to the escape edge
                if escape_dir == 'left':
                    p_closer_to_edge = p_pad.global_x < n_pad.global_x
                else:  # right
                    p_closer_to_edge = p_pad.global_x > n_pad.global_x

                # Pad closer to edge uses between_ch (normal routing)
                # Pad farther from edge jogs to adjacent_ch
                # Choose adjacent_ch on the side of the farther pad
                if p_closer_to_edge:
                    # P uses between, N jogs to adjacent
                    # N is farther, so pick adjacent on N's side (above if N is top, below if N is bottom)
                    n_is_top = n_pad.global_y < p_pad.global_y
                    if n_is_top and between_idx > 0:
                        adjacent_ch = h_channels_sorted[between_idx - 1]
                    elif not n_is_top and between_idx < len(h_channels_sorted) - 1:
                        adjacent_ch = h_channels_sorted[between_idx + 1]
                    elif between_idx > 0:
                        adjacent_ch = h_channels_sorted[between_idx - 1]
                    elif between_idx < len(h_channels_sorted) - 1:
                        adjacent_ch = h_channels_sorted[between_idx + 1]
                    else:
                        adjacent_ch = between_ch
                    p_target_ch = between_ch
                    n_target_ch = adjacent_ch
                else:
                    # N uses between, P jogs to adjacent
                    # P is farther, so pick adjacent on P's side
                    p_is_top_here = p_pad.global_y < n_pad.global_y
                    if p_is_top_here and between_idx > 0:
                        adjacent_ch = h_channels_sorted[between_idx - 1]
                    elif not p_is_top_here and between_idx < len(h_channels_sorted) - 1:
                        adjacent_ch = h_channels_sorted[between_idx + 1]
                    elif between_idx > 0:
                        adjacent_ch = h_channels_sorted[between_idx - 1]
                    elif between_idx < len(h_channels_sorted) - 1:
                        adjacent_ch = h_channels_sorted[between_idx + 1]
                    else:
                        adjacent_ch = between_ch
                    p_target_ch = adjacent_ch
                    n_target_ch = between_ch
            else:
                # No channel between pads - use channels above and below
                channels_above = [c for c in h_channels_sorted if c.position < top_pad_y]
                channels_below = [c for c in h_channels_sorted if c.position > bot_pad_y]
                p_target_ch = channels_above[-1] if channels_above and p_is_top else (channels_below[0] if channels_below else None)
                n_target_ch = channels_below[0] if channels_below and not p_is_top else (channels_above[-1] if channels_above else None)

            # Initialize route_ch with default before conditional assignment
            route_ch = channel
            if is_p_route and p_target_ch:
                target_y = p_target_ch.position
                route_ch = p_target_ch
            elif not is_p_route and n_target_ch:
                target_y = n_target_ch.position
                route_ch = n_target_ch
            else:
                # Fallback to convergence if no separate channel available
                target_y = center_y - half_pair_spacing if (is_p_route and p_is_top) or (not is_p_route and not p_is_top) else center_y + half_pair_spacing
                # route_ch already initialized to channel above

            # Route to target channel via 45 stub
            dy_needed = target_y - pad_info.global_y
            if escape_dir == 'right':
                stub_end_x = pad_info.global_x + abs(dy_needed)
            else:  # left
                stub_end_x = pad_info.global_x - abs(dy_needed)
            stub_end_y = target_y
            stub_end = (stub_end_x, stub_end_y)

            # Exit continues horizontally to BGA edge
            if escape_dir == 'right':
                exit_pos = (grid.max_x + exit_margin, stub_end[1])
            else:  # left
                exit_pos = (grid.min_x - exit_margin, stub_end[1])
        elif escape_dir in ('up', 'down'):
            # Vertical escape: converge in X (perpendicular to the escape) and
            # keep Y monotonic toward the edge. Converging in Y here would send
            # the pad on the escape side *back* toward the BGA before it exits
            # (a loop-back, #242) and leave the pair uncoupled.
            p_is_left = p_pad.global_x < n_pad.global_x
            if (is_p_route and p_is_left) or (not is_p_route and not p_is_left):
                target_x = center_x - half_pair_spacing
            else:
                target_x = center_x + half_pair_spacing

            # At 45, the Y advance toward the edge equals the X convergence.
            dx_needed = target_x - pad_info.global_x
            if escape_dir == 'down':
                stub_end = (target_x, pad_info.global_y + abs(dx_needed))
                exit_pos = (target_x, grid.max_y + exit_margin)
            else:  # up
                stub_end = (target_x, pad_info.global_y - abs(dx_needed))
                exit_pos = (target_x, grid.min_y - exit_margin)

        else:
            # Horizontal escape: converge in Y, keep X monotonic toward the edge.
            # Top pad goes to the top target, bottom pad to the bottom target.
            if (is_p_route and p_is_top) or (not is_p_route and not p_is_top):
                target_y = center_y - half_pair_spacing
            else:
                target_y = center_y + half_pair_spacing

            # At 45, dx = dy (in absolute terms)
            dy_needed = target_y - pad_info.global_y
            if escape_dir == 'right':
                stub_end = (pad_info.global_x + abs(dy_needed), target_y)
                exit_pos = (grid.max_x + exit_margin, target_y)
            else:  # left
                stub_end = (pad_info.global_x - abs(dy_needed), target_y)
                exit_pos = (grid.min_x - exit_margin, target_y)

    # Use pre-assigned layer if available, otherwise default to layers[0]
    assigned_layer = pair_layer_assignments.get(pair_id, layers[0]) if pair_layer_assignments else layers[0]

    return FanoutRoute(
        pad=pad_info,
        pad_pos=(pad_info.global_x, pad_info.global_y),
        stub_end=stub_end,
        exit_pos=exit_pos,
        channel=route_ch,
        escape_dir=escape_dir,
        is_edge=is_edge,
        layer=assigned_layer,
        pair_id=pair_id,
        is_p=is_p_route
    )


def build_inner_aligned_route(
    pad_info: Pad,
    is_p_route: bool,
    offset: float,
    route_ch: Optional['Channel'],
    escape_dir: str,
    is_edge: bool,
    grid: BGAGrid,
    layers: List[str],
    exit_margin: float,
    pair_layer_assignments: Dict,
    pair_id: str,
) -> FanoutRoute:
    """Build one leg of an inner diff pair with aligned escape.

    45 stub to the channel with the per-leg offset, then channel to exit.
    Behavior identical to the inlined logic.
    """
    # Inner pads with aligned escape: 45 stub to channel with offset, then channel to exit
    # In adjacent-channel mode, route_ch is the pad-specific channel
    stub_end = create_45_stub(pad_info.global_x, pad_info.global_y,
                             route_ch, escape_dir, offset)
    exit_pos = calculate_exit_point(stub_end, route_ch, escape_dir,
                                   grid, exit_margin, offset)

    # Use pre-assigned layer if available, otherwise default to layers[0]
    assigned_layer = pair_layer_assignments.get(pair_id, layers[0]) if pair_layer_assignments else layers[0]

    return FanoutRoute(
        pad=pad_info,
        pad_pos=(pad_info.global_x, pad_info.global_y),
        stub_end=stub_end,
        exit_pos=exit_pos,
        channel=route_ch,
        escape_dir=escape_dir,
        is_edge=is_edge,
        layer=assigned_layer,
        pair_id=pair_id,
        is_p=is_p_route
    )


def build_diff_pair_routes(
    pair_id: str,
    pair: DiffPairPads,
    pass_escape_assignments: Dict,
    grid: BGAGrid,
    channels: List[Channel],
    layers: List[str],
    exit_margin: float,
    half_pair_spacing: float,
    use_adjacent_channels_h: bool,
    use_adjacent_channels_v: bool,
    pair_layer_assignments: Dict,
) -> List[FanoutRoute]:
    """Build both P and N routes for one differential pair.

    Resolves the assigned (channel, escape_dir), classifies the pair
    (horizontal/vertical pads, cross-escape, half-edge), selects adjacent
    channels and per-leg offsets, then dispatches each leg to the appropriate
    leg builder (half-edge, converge, or inner-aligned). Behavior identical to
    the inlined per-pad loop body it replaces.
    """
    p_pad = pair.p_pad
    n_pad = pair.n_pad

    # Use pre-assigned escape direction if available, otherwise compute
    if pair_id in pass_escape_assignments:
        channel, escape_dir = pass_escape_assignments[pair_id]
    else:
        channel, escape_dir = find_diff_pair_escape(
            p_pad.global_x, p_pad.global_y,
            n_pad.global_x, n_pad.global_y,
            grid, channels
        )
    is_edge = channel is None

    # Determine if pads are horizontally or vertically adjacent
    pads_horizontal = abs(p_pad.global_x - n_pad.global_x) > abs(p_pad.global_y - n_pad.global_y)

    # Check if escape direction is "cross" to pad orientation
    # Cross case: horizontal pads escaping vertically, or vertical pads escaping horizontally
    # In cross case, pads converge with 45 stubs (like edge pairs)
    is_cross_escape = False
    if channel:
        if pads_horizontal and escape_dir in ['up', 'down']:
            is_cross_escape = True
        elif not pads_horizontal and escape_dir in ['left', 'right']:
            is_cross_escape = True

    # For adjacent-channel mode: find two channels, one on each side of the pads
    p_channel, n_channel = select_adjacent_channels(
        channel, escape_dir, p_pad, n_pad, channels,
        use_adjacent_channels_h, use_adjacent_channels_v,
        is_cross_escape, is_edge
    )

    # Determine which pad is "positive" offset and which is "negative"
    p_offset, n_offset = compute_pair_offsets(
        p_channel, n_channel, channel, is_cross_escape, escape_dir,
        p_pad, n_pad, half_pair_spacing
    )

    # Check for half-edge case
    is_half_edge = escape_dir.startswith('half_edge_')
    if is_half_edge:
        actual_escape_dir = escape_dir.replace('half_edge_', '')
    else:
        actual_escape_dir = escape_dir

    routes: List[FanoutRoute] = []

    # Create routes for both P and N
    # In adjacent-channel mode, p_channel and n_channel are different
    for pad_info, offset, is_p_route, route_ch in [(p_pad, p_offset, True, p_channel), (n_pad, n_offset, False, n_channel)]:
        if is_half_edge:
            routes.append(build_half_edge_route(
                pad_info, is_p_route, p_pad, n_pad, actual_escape_dir,
                grid, channels, layers, exit_margin, half_pair_spacing, pair_id
            ))
            continue  # Skip the normal edge/inner handling below

        if is_edge or is_cross_escape:
            routes.append(build_converge_route(
                pad_info, is_p_route, p_pad, n_pad, pads_horizontal,
                escape_dir, is_edge, channel, grid, channels, layers,
                exit_margin, half_pair_spacing, use_adjacent_channels_h,
                pair_layer_assignments, pair_id
            ))
        else:
            routes.append(build_inner_aligned_route(
                pad_info, is_p_route, offset, route_ch, escape_dir, is_edge,
                grid, layers, exit_margin, pair_layer_assignments, pair_id
            ))

    return routes


def single_pad_net_ids(footprint: Footprint, pcb_data: PCBData) -> Set[int]:
    """Net IDs on `footprint` that have only one pad board-wide, i.e. nothing to
    connect to (spare/NC pins). Fanning these out is pointless and burns escape
    channels that real signals need on a dense BGA (issue #122) - the human
    ulx3s board leaves all 25 of its NC balls unrouted."""
    nc = set()
    for pad in footprint.pads:
        if not pad.net_id or pad.net_id == 0:
            continue
        if len(pcb_data.pads_by_net.get(pad.net_id, [])) < 2:
            nc.add(pad.net_id)
    return nc


def generate_bga_fanout(footprint: Footprint,
                        pcb_data: PCBData,
                        net_filter: Optional[List[str]] = None,
                        diff_pair_patterns: Optional[List[str]] = None,
                        layers: List[str] = None,
                        track_width: float = 0.1,
                        clearance: float = 0.1,
                        diff_pair_gap: float = 0.101,
                        exit_margin: float = 0.5,
                        primary_escape: str = 'horizontal',
                        force_escape_direction: bool = False,
                        rebalance_escape: bool = False,
                        via_size: float = 0.5,
                        via_drill: float = 0.3,
                        check_for_previous: bool = False,
                        no_inner_top_layer: bool = False,
                        escape_method: str = 'auto',
                        grid_step: float = 0.0,
                        layer_costs: Optional[List[float]] = None) -> Tuple[List[Dict], List[Dict], List[Dict]]:
    """
    Generate BGA fanout tracks for a footprint.

    Creates:
    1. 45-degree stubs from pads to channels
    2. Channel segments extending to BGA boundary exit
    3. Differential pairs routed together on same layer
    4. Vias at pads where routing starts on non-top layer

    Args:
        footprint: The BGA footprint
        pcb_data: Full PCB data
        net_filter: Optional list of net patterns to include
        diff_pair_patterns: Glob patterns for differential pair nets (e.g., '*lvds*')
        layers: Available routing layers (all connected via pad vias)
        track_width: Width of fanout tracks
        clearance: Minimum clearance between tracks
        diff_pair_gap: Gap between P and N traces of a differential pair
        exit_margin: How far past BGA boundary to extend
        primary_escape: Primary escape direction preference ('horizontal' or 'vertical')
        via_size: Size of vias to add at pads (default 0.3mm)
        via_drill: Drill size for vias (default 0.2mm)
        check_for_previous: If True, skip pads that already have fanout tracks
        no_inner_top_layer: If True, inner pads cannot use F.Cu (top layer)
        escape_method: 'auto' (default) runs the channel router and, if it drops
            any ball, retries with the under-pad grid escape and keeps whichever
            escapes more (issue #288). 'channel' is the 45-stub + channel router (with
            differential-pair support). 'underpad' is the dense-array grid escape
            (issue #122): every signal ball drops a via in its pad and routes
            straight UNDER the pad field on an inner layer, jogging into a
            between-ball channel only to dodge a via. It escapes fully-populated
            arrays the channel router can't (ulx3s 22x22). When diff_pair_patterns
            are given it escapes those pairs COUPLED (issue #182) - via-free on
            the top layer for edge pairs, on an inner layer with via-in-pad for
            deeper ones - so route_diff picks them up. Power/plane nets are
            skipped (they tap their plane).

    Returns:
        Tuple of (tracks, vias_to_add, vias_to_remove, failed_nets)
    """
    if layers is None:
        layers = ["F.Cu", "B.Cu"]

    # Non-orthogonally-placed parts (issue #137): the grid/escape logic below is
    # global-axis-bound, so rotate the whole board into this footprint's frame
    # (where its balls are axis-aligned), run the pipeline, and map the resulting
    # tracks/vias back. Orthogonal placements skip this and are unaffected.
    from bga_fanout.rotate_frame import (is_orthogonal, to_axis_aligned_frame,
                                         back_transform_results)
    if not is_orthogonal(footprint.rotation):
        print(f"  {footprint.reference} placed at {footprint.rotation:.1f}° - routing "
              f"in the footprint frame and mapping back (issue #137)")
        rp, back = to_axis_aligned_frame(pcb_data, footprint.reference)
        tracks, vias_to_add, vias_to_remove, failed_nets = generate_bga_fanout(
            rp.footprints[footprint.reference], rp,
            net_filter=net_filter, diff_pair_patterns=diff_pair_patterns, layers=layers,
            track_width=track_width, clearance=clearance, diff_pair_gap=diff_pair_gap,
            exit_margin=exit_margin, primary_escape=primary_escape,
            force_escape_direction=force_escape_direction, rebalance_escape=rebalance_escape,
            via_size=via_size, via_drill=via_drill, check_for_previous=check_for_previous,
            no_inner_top_layer=no_inner_top_layer, escape_method=escape_method,
            grid_step=grid_step, layer_costs=layer_costs)
        back_transform_results(tracks, vias_to_add, vias_to_remove, back)
        return tracks, vias_to_add, vias_to_remove, failed_nets

    # --layer-costs (issue #288): same semantics as route.py -- a NEGATIVE cost
    # forbids the layer (no escape copper placed there; e.g. a soon-to-be-plane
    # inner layer), a positive cost >= 1.0 is a preference weight. The channel
    # engine tries layers in list order, so its non-top layers are re-sorted
    # cheapest-first (the top layer stays first: edge escapes are hardwired to
    # it). The under-pad engine keeps the user's physical order (its via spans
    # use layers[0]/layers[-1]) and applies only the forbidden-layer exclusion.
    underpad_layers = layers
    balance_layers = layers
    if layer_costs:
        if len(layer_costs) != len(layers):
            raise ValueError(f"--layer-costs needs one value per layer "
                             f"({len(layers)} layers, got {len(layer_costs)})")
        for lname, cost in zip(layers, layer_costs):
            if cost >= 0 and (cost < 1.0 or cost > 1000):
                raise ValueError(f"Layer cost for {lname} must be negative "
                                 f"(forbidden) or between 1.0 and 1000, got {cost}")
        if layer_costs[0] < 0:
            raise ValueError(f"The top escape layer ({layers[0]}) cannot be "
                             f"forbidden - edge escapes are placed on it")
        keep = [(l, c) for l, c in zip(layers, layer_costs) if c >= 0]
        dropped_layers = [l for l, c in zip(layers, layer_costs) if c < 0]
        if dropped_layers:
            print(f"  Layer costs: excluding forbidden layer(s) "
                  f"{', '.join(dropped_layers)} from the fanout")
        underpad_layers = [l for l, _ in keep]
        # The even-distribution rebalance would spread escapes right back onto
        # the costly layers the greedy assignment just avoided, so it only
        # balances across the cheapest tier; costlier layers keep only the
        # overflow routes the greedy pass could not fit elsewhere.
        min_cost = min(c for _, c in keep)
        balance_layers = [l for l, c in keep if c <= min_cost + 1e-9]
        if keep[0][0] not in balance_layers:
            # rebalance treats its first entry as the top layer (edge escapes)
            balance_layers = [keep[0][0]] + balance_layers
        if escape_method == 'underpad':
            layers = underpad_layers
        else:
            layers = [keep[0][0]] + [l for l, _ in
                                     sorted(keep[1:], key=lambda lc: lc[1])]
            if layers != underpad_layers:
                print(f"  Layer costs: channel escape layer preference: "
                      f"{' > '.join(layers)}")

    # Fab-floor clamp (issue #223): an escape stub thinner than the board's
    # minimum manufacturable track width is un-routable at the stated fab class
    # (usb_sniffer's /T_USB_* bus was emitted at 0.100mm against a 0.127mm
    # 2-layer floor -> a whole bus of TRACK-WIDTH violations). The width is a
    # parameter, not a search outcome, so clamp it up to the fab floor here --
    # mirroring the same clamp route.py / check_drc.py already apply.
    from list_nets import fab_floors
    ncu = len(pcb_data.board_info.copper_layers) if pcb_data.board_info.copper_layers else 2
    fab_min_track = fab_floors(ncu)['track_width']
    if track_width < fab_min_track - 1e-9:
        print(f"  Track width {track_width:.4f}mm is below the {ncu}-layer fab "
              f"floor {fab_min_track:.4f}mm - clamping escape stubs up (issue #223)")
        track_width = fab_min_track

    grid = analyze_bga_grid(footprint)
    if grid is None:
        print(f"Warning: {footprint.reference} doesn't appear to be a BGA")
        return [], [], [], []

    # Sanity-check pad geometry before escaping (see qfn_fanout): overlapping
    # same-footprint pads mean the pad rotation/size is modelled wrong.
    from check_pads import find_pad_overlaps
    _ov = find_pad_overlaps(pcb_data, component=footprint.reference)
    if _ov:
        print(f"  WARNING: {footprint.reference} has {len(_ov)} overlapping "
              f"different-net pad pair(s) - pad geometry looks wrong, fanout "
              f"may be placed across pads. Run: python3 check_pads.py <board> "
              f"--component {footprint.reference}")

    print(f"BGA Grid Analysis for {footprint.reference}:")
    print(f"  Pitch: {grid.pitch_x:.2f} x {grid.pitch_y:.2f} mm")
    print(f"  Grid: {len(grid.rows)} rows x {len(grid.cols)} columns")
    print(f"  Center: ({grid.center_x:.2f}, {grid.center_y:.2f})")
    print(f"  Boundary: X[{grid.min_x:.2f}, {grid.max_x:.2f}], Y[{grid.min_y:.2f}, {grid.max_y:.2f}]")

    # Escape-budget guard (issue #158). The channel engine runs one escape track down
    # the half-pitch between adjacent via columns, so via/2 + track/2 + clearance must
    # fit the half-pitch or EVERY escape grazes the neighbouring column's via by the
    # deficit -- and the run still reports failed:0, since the success metric ignores
    # sub-clearance grazes. We have all four numbers here, so warn (don't silently
    # ship the graze). Doesn't apply to underpad, which routes under the pad field.
    if escape_method != 'underpad':
        half_pitch = min(grid.pitch_x, grid.pitch_y) / 2.0
        need = via_size / 2.0 + track_width / 2.0 + clearance
        if need > half_pitch + 1e-6:
            via_max = 2.0 * (half_pitch - track_width / 2.0 - clearance)
            print(f"  WARNING: escape via {via_size:.3f}mm busts the half-pitch budget "
                  f"(need {need:.4f} > half-pitch {half_pitch:.4f}mm) -> every escape "
                  f"track grazes an adjacent via by ~{(need - half_pitch) * 1000:.0f}um "
                  f"at clearance {clearance:.3f}. Use --via-size <= {via_max:.3f} "
                  f"(>= drill {via_drill:.3f} + annular ring), a narrower track, or more "
                  f"escape layers. See issue #158.")

    # Under-pad grid escape (issue #122) - a separate engine for dense arrays.
    if escape_method == 'underpad':
        from bga_fanout.underpad import generate_underpad_escape
        net_filter_fn = None
        if net_filter:
            net_filter_fn = lambda name: matches_net_filter(name, net_filter)
        # Differential pairs (issue #182): escape each pair coupled so route_diff
        # can pick the two halves up (without this they go single-ended).
        up_diff_pairs = (find_differential_pairs(footprint, diff_pair_patterns)
                         if diff_pair_patterns else {})
        if up_diff_pairs:
            print(f"  Found {len(up_diff_pairs)} differential pair(s) to escape coupled")
        tracks, vias_to_add, failed_nets = generate_underpad_escape(
            footprint, pcb_data, grid, layers,
            track_width=track_width, clearance=clearance,
            via_size=via_size, via_drill=via_drill, exit_margin=exit_margin,
            net_filter_fn=net_filter_fn,
            diff_pairs=up_diff_pairs, diff_pair_gap=diff_pair_gap,
        )
        return tracks, vias_to_add, [], failed_nets

    channels = calculate_channels(grid)
    h_count = len([c for c in channels if c.orientation == 'horizontal'])
    v_count = len([c for c in channels if c.orientation == 'vertical'])
    print(f"  Channels: {h_count} horizontal, {v_count} vertical")
    print(f"  Available layers: {layers}")

    # Check for existing fanouts if requested
    fanned_out_nets: Set[int] = set()
    pre_occupied_exits: Dict[Tuple[str, str, float], str] = {}
    if check_for_previous:
        fanned_out_nets, pre_occupied_exits = find_existing_fanouts(
            pcb_data, footprint, grid, channels
        )
        if fanned_out_nets:
            print(f"  Found {len(fanned_out_nets)} nets with existing fanouts (will skip)")
        if pre_occupied_exits:
            print(f"  Found {len(pre_occupied_exits)} occupied exit positions")

    # Find differential pairs if patterns specified
    diff_pairs: Dict[str, DiffPairPads] = {}
    pair_escape_assignments: Dict[str, Tuple[Optional[Channel], str]] = {}
    if diff_pair_patterns:
        diff_pairs = find_differential_pairs(footprint, diff_pair_patterns)
        original_pair_count = len(diff_pairs)

        # Filter out pairs that already have fanouts
        if check_for_previous and fanned_out_nets:
            pairs_to_remove = []
            for pair_id, pair in diff_pairs.items():
                p_fanned = pair.p_pad and pair.p_pad.net_id in fanned_out_nets
                n_fanned = pair.n_pad and pair.n_pad.net_id in fanned_out_nets
                if p_fanned or n_fanned:
                    pairs_to_remove.append(pair_id)
            for pair_id in pairs_to_remove:
                del diff_pairs[pair_id]
            if pairs_to_remove:
                print(f"  Found {original_pair_count} differential pairs ({len(pairs_to_remove)} already fanned out)")
            else:
                print(f"  Found {len(diff_pairs)} differential pairs")
        else:
            print(f"  Found {len(diff_pairs)} differential pairs")

        # Pre-assign escape directions for all pairs to avoid overlaps
        force_str = " (forced)" if force_escape_direction else ""
        print(f"  Assigning escape directions (primary: {primary_escape}{force_str})...")
        pair_escape_assignments, pair_layer_assignments = assign_pair_escapes(
            diff_pairs, grid, channels, layers,
            primary_orientation=primary_escape,
            track_width=track_width,
            clearance=clearance,
            diff_pair_gap=diff_pair_gap,
            via_size=via_size,
            rebalance=rebalance_escape,
            pre_occupied=pre_occupied_exits,
            force_escape_direction=force_escape_direction
        )

    # Build lookup from net_name to pair info
    net_to_pair: Dict[str, Tuple[str, bool]] = {}  # net_name -> (pair_id, is_p)
    for pair_id, pair in diff_pairs.items():
        if pair.p_pad:
            net_to_pair[pair.p_pad.net_name] = (pair_id, True)
        if pair.n_pad:
            net_to_pair[pair.n_pad.net_name] = (pair_id, False)

    # Calculate half-spacing for differential pairs
    # Each trace is offset from channel center by this amount
    half_pair_spacing = (track_width + diff_pair_gap) / 2

    # Check if channels are wide enough for both tracks of a diff pair
    # If not, route P and N in adjacent channels instead of same channel with offsets
    # The track at offset from channel center must maintain clearance from adjacent vias
    # For inner layers, use via size (not pad size) since that's the actual constraint
    via_radius = via_size / 2

    # Calculate maximum allowed track offset from channel center
    # Channel is midway between via rows, so distance to nearest via center = pitch/2
    # For track to clear via: track_center_offset + track_width/2 + clearance <= pitch/2 - via_radius
    max_offset_h = grid.pitch_y / 2 - via_radius - track_width / 2 - clearance
    max_offset_v = grid.pitch_x / 2 - via_radius - track_width / 2 - clearance

    # Determine per-direction whether adjacent channels are needed
    # For horizontal escape (left/right): tracks in horizontal channels, constrained by pitch_y
    # For vertical escape (up/down): tracks in vertical channels, constrained by pitch_x
    use_adjacent_channels_h = half_pair_spacing > max_offset_h
    use_adjacent_channels_v = half_pair_spacing > max_offset_v

    if use_adjacent_channels_h and use_adjacent_channels_v:
        print(f"  Using adjacent-channel routing for diff pairs in both directions")
    elif use_adjacent_channels_h:
        print(f"  Using adjacent-channel routing for horizontal escape (half_pair_spacing {half_pair_spacing:.3f}mm > max_offset_h {max_offset_h:.3f}mm)")
    elif use_adjacent_channels_v:
        print(f"  Using adjacent-channel routing for vertical escape (half_pair_spacing {half_pair_spacing:.3f}mm > max_offset_v {max_offset_v:.3f}mm)")

    # Build a shared obstacle map (reuses obstacle_map.py) so fanout stubs avoid
    # foreign component pads, existing copper, and vias. All fanned-out net_ids
    # are passed as nets_to_route so their OWN pads/copper are NOT obstacles
    # (a stub legitimately starts on its own ball), while every foreign pad,
    # track and via IS an obstacle. extra_clearance=track_width/2 keeps the
    # stub's edge (not just its centerline) off pads. Through-hole pads block
    # all layers; SMD pads block their layer.
    # Single-pad / NC balls have nothing to connect to; fanning them just burns
    # escape channels real signals need on a dense BGA (issue #122). Skip them.
    nc_net_ids = single_pad_net_ids(footprint, pcb_data)
    if nc_net_ids:
        print(f"  Skipping {len(nc_net_ids)} single-pad/NC net(s) (nothing to connect to)")

    fanned_net_ids: Set[int] = set()
    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue
        if pad.net_name.lower().startswith('unconnected-'):
            continue
        if pad.net_id in nc_net_ids:
            continue
        if net_filter and not matches_net_filter(pad.net_name, net_filter):
            continue
        if check_for_previous and pad.net_id in fanned_out_nets:
            continue
        fanned_net_ids.add(pad.net_id)

    obstacle_cfg = GridRouteConfig(
        layers=list(layers),
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
    )
    obstacle_layer_map = build_layer_map(obstacle_cfg.layers)
    print(f"  Building pad-aware obstacle map ({len(fanned_net_ids)} fanned nets excluded)...")
    obstacles = build_base_obstacle_map(
        pcb_data, obstacle_cfg,
        nets_to_route=list(fanned_net_ids),
        extra_clearance=track_width / 2,
    )

    # Foreign-component pads on copper layers. These supplement the shared map:
    # a foreign pad that shares a net with a BGA ball is dropped from the map
    # (its net is a fanned net) yet a DIFFERENT net's stub must still not cross
    # it (e.g. C83.1 on Net-(C82-Pad1) vs the MIPI_D1_N stub). The per-route
    # checks skip pads on the route's own net, so same-net taps stay legal.
    bga_ref = footprint.reference
    copper_layer_set = set(obstacle_cfg.layers)
    foreign_pads = [
        p for fp in pcb_data.footprints.values() if fp.reference != bga_ref
        for p in fp.pads
        if p.drill > 0 or any(l in copper_layer_set for l in (p.layers or []))
    ]

    # Build routes - process differential pairs together
    def _run_pass(force_secondary_pairs):
        """Run one full route-build + collision-resolution pass.

        Builds routes, assigns layers, generates tracks, and resolves
        collisions. Returns (tracks, vias_to_add, vias_to_remove,
        failed_nets, routes). Reruns are isolated: fresh routes/processed_pairs
        and a private copy of pair_escape_assignments (the caller's copy is
        never mutated).

        force_secondary_pairs: set of pair_ids to force to their orthogonal
        (secondary) escape direction.
        """
        # Private copy so retries don't accumulate / mutate caller state.
        pass_escape_assignments = dict(pair_escape_assignments)

        # Force requested pairs onto their orthogonal (secondary) orientation.
        for fp in force_secondary_pairs:
            if fp not in diff_pairs:
                continue
            cur = pass_escape_assignments.get(fp)
            if not cur:
                continue
            _, cur_dir = cur
            # Skip edge / half-edge pairs (fixed escapes, no orthogonal retry).
            if cur_dir is None or cur_dir.startswith('half_edge_'):
                continue
            cur_orientation = 'horizontal' if cur_dir in ('left', 'right') else 'vertical'
            secondary_orientation = 'vertical' if cur_orientation == 'horizontal' else 'horizontal'
            fpair = diff_pairs[fp]
            new_ch, new_dir = find_diff_pair_escape(
                fpair.p_pad.global_x, fpair.p_pad.global_y,
                fpair.n_pad.global_x, fpair.n_pad.global_y,
                grid, channels, secondary_orientation
            )
            # Only overwrite if we actually got a usable orthogonal direction.
            new_orientation = None
            if new_dir in ('left', 'right'):
                new_orientation = 'horizontal'
            elif new_dir in ('up', 'down'):
                new_orientation = 'vertical'
            if new_orientation == secondary_orientation:
                pass_escape_assignments[fp] = (new_ch, new_dir)

        routes: List[FanoutRoute] = []
        processed_pairs: Set[str] = set()

        for pad in footprint.pads:
            if not pad.net_name or pad.net_id == 0:
                continue

            # Skip unconnected nets (KiCad pins not connected in schematic)
            if pad.net_name.lower().startswith('unconnected-'):
                continue

            # Skip single-pad/NC balls - nothing to route to (issue #122)
            if pad.net_id in nc_net_ids:
                continue

            if net_filter and not matches_net_filter(pad.net_name, net_filter):
                continue

            # Skip if this pad already has a fanout (check_for_previous mode)
            if check_for_previous and pad.net_id in fanned_out_nets:
                continue

            # Check if this pad is part of a differential pair
            pair_id = None
            is_p = True
            if pad.net_name in net_to_pair:
                pair_id, is_p = net_to_pair[pad.net_name]

            # Skip if we already processed this pair
            if pair_id and pair_id in processed_pairs:
                continue

            if pair_id:
                # Process differential pair together
                processed_pairs.add(pair_id)
                pair = diff_pairs[pair_id]
                routes.extend(build_diff_pair_routes(
                    pair_id, pair, pass_escape_assignments, grid, channels,
                    layers, exit_margin, half_pair_spacing,
                    use_adjacent_channels_h, use_adjacent_channels_v,
                    pair_layer_assignments,
                ))
            else:
                # Single-ended signal (not part of a pair)
                force_orient = primary_escape if force_escape_direction else None
                route = create_single_ended_route(
                    pad, grid, channels, layers, exit_margin, force_orient
                )
                routes.append(route)

        print_route_statistics(routes)

        if not routes:
            return [], [], [], []

        # Connect adjacent same-net pads directly (before layer assignment)
        neighbor_connections = connect_adjacent_same_net_pads(routes, grid, track_width, clearance)
        if neighbor_connections > 0:
            print(f"  Connected {neighbor_connections} adjacent same-net pads directly")

        # Convert existing PCB segments to track format for collision checking
        existing_tracks = convert_segments_to_tracks(pcb_data) if check_for_previous else []

        # Reassign on-channel pads to adjacent channels when too many share the same channel
        reassigned_count = reassign_on_channel_pads(routes, channels, grid, len(layers), exit_margin, footprint)
        if reassigned_count > 0:
            print(f"  Reassigned {reassigned_count} on-channel pads to adjacent channels")

        # Smart layer assignment (keeps diff pairs together, avoids existing tracks)
        assign_layers_smart(routes, layers, track_width, clearance, diff_pair_gap, existing_tracks, no_inner_top_layer)

        # Calculate jog length = distance from BGA edge to first pad row/col
        # This is half the pitch (since edge is pitch/2 from first pad)
        jog_length = min(grid.pitch_x, grid.pitch_y) / 2
        print(f"  Jog length: {jog_length:.2f} mm")

        # Calculate jog_end for each route based on layer (snapped to the routing
        # grid when grid_step is set, issue #149); the obstacle map drops a
        # decorative end-jog that would extend into a foreign pad/track/via.
        calculate_jog_ends_for_routes(routes, layers, jog_length, track_width, diff_pair_gap,
                                      grid_step, obstacles, obstacle_cfg, obstacle_layer_map)

        # Generate tracks
        tracks, edge_count, inner_count = generate_tracks_from_routes(routes, track_width, layers[0])

        # Validate no collisions
        min_spacing = track_width + clearance
        collision_count, collision_pairs = detect_collisions(tracks, existing_tracks, min_spacing)
        failed_nets: List = []  # populated by resolve_collisions if there are collisions

        if collision_count > 0:
            print(f"  INFO: {collision_count} potential collisions detected (will attempt to resolve)")
            for t1, t2 in collision_pairs:
                existing_marker = " (existing)" if t2.get('is_existing') else ""
                print(f"    {t1['layer']} net{t1['net_id']}: {t1['start']}->{t1['end']}")
                print(f"    {t2['layer']} net{t2['net_id']}: {t2['start']}->{t2['end']}{existing_marker}")

            # Try to resolve collisions by reassigning layers or using alternate channels
            print(f"  Attempting to resolve collisions...")
            # Build net_id -> net_name mapping for error reporting
            net_id_to_name = {r.net_id: r.pad.net_name for r in routes if r.pad.net_name}
            reassigned, failed_nets = resolve_collisions(routes, tracks, layers, track_width, clearance, diff_pair_gap,
                                            existing_tracks, grid, channels, exit_margin, net_id_to_name, no_inner_top_layer,
                                            obstacles, obstacle_cfg, obstacle_layer_map, foreign_pads)

            if failed_nets:
                print(f"\n  ERROR: Failed to route {len(failed_nets)} net(s):")
                for net_name in failed_nets:
                    print(f"    - {net_name}")
                print(f"  These nets have been removed from the output.\n")

            if reassigned > 0:
                # Recount collisions after resolution
                new_collision_count, _ = detect_collisions(tracks, existing_tracks, min_spacing, max_pairs=0)
                print(f"  After resolution: {new_collision_count} collisions remaining")
                collisions_remaining = new_collision_count
        else:
            print(f"  Validated: No collisions")
            collisions_remaining = 0

        # Post-resolution layer rebalancing for even distribution. With
        # --layer-costs this only balances across the cheapest tier so costly
        # (soon-to-be-plane) layers keep only the greedy pass's overflow (#288).
        if collisions_remaining == 0 and len(balance_layers) > 1:
            rebalanced_count = rebalance_layers(routes, tracks, existing_tracks, balance_layers, min_spacing)
            if rebalanced_count > 0:
                print(f"  Rebalanced {rebalanced_count} routes for even layer distribution")

        # NOTE: pad-aware repair runs ONCE on the selected best result (after the
        # orthogonal re-escape retry loop below), not per-pass: repairing inside
        # a pass would perturb that pass's Z-Z short count and steer the retry
        # loop into worse layer choices.

        # Stats by layer
        layer_counts = defaultdict(int)
        for route in routes:
            layer_counts[route.layer] += 1
        for layer, count in sorted(layer_counts.items()):
            print(f"    {layer}: {count} routes")

        # Via management: add vias where needed, remove unnecessary ones
        vias_to_add, vias_to_remove, via_blocked_routes = manage_vias(
            routes, pcb_data, layers[0], via_size, via_drill, clearance
        )

        # Routes whose required via-in-pad would hit an immovable foreign pad
        # (#253) are dropped: without the via their inner-layer copper is
        # disconnected decoration. Remove their tracks and report the nets as
        # failed so the main router picks them up from the bare ball.
        if via_blocked_routes:
            blocked_net_ids = {r.net_id for r in via_blocked_routes}
            blocked_routes = set(id(r) for r in via_blocked_routes)
            tracks = [t for t in tracks if t.get('net_id') not in blocked_net_ids]
            routes = [r for r in routes if id(r) not in blocked_routes]
            for r in via_blocked_routes:
                name = r.pad.net_name or f"net{r.net_id}"
                if name not in failed_nets:
                    failed_nets.append(name)

        return tracks, vias_to_add, vias_to_remove, list(failed_nets), routes

    # Outer orthogonal re-escape retry loop.
    # Run the pass; if diff-pair-vs-diff-pair (Z-Z) shorts remain, re-run with
    # the offending pairs forced to their orthogonal escape direction. Keep the
    # result with the fewest Z-Z shorts (ties -> earliest pass).
    # A Z-Z "short" means copper actually overlaps: two tracks whose centerlines
    # come within a track width (each contributes half a width). Using
    # track_width here (rather than track_width + clearance) detects genuine
    # shorts instead of mere clearance violations, which keeps the retry from
    # chasing legally-spaced neighbors.
    zz_min_spacing = track_width
    forced_pairs: Set[str] = set()
    best_result = None
    best_zz = None
    prev_zz = None
    for attempt in range(3):  # 1 initial pass + up to 2 retries
        result = _run_pass(set(forced_pairs))
        pass_tracks = result[0]
        zz_count, zz_involved = count_diff_pair_shorts(pass_tracks, zz_min_spacing)

        if attempt > 0:
            orientation_desc = 'vertical' if primary_escape == 'horizontal' else 'horizontal'
            print(f"  Orthogonal re-escape: forced {len(forced_pairs)} pair(s) to "
                  f"{orientation_desc}, Z-Z shorts {prev_zz} -> {zz_count}")

        if best_result is None or zz_count < best_zz:
            best_result = result
            best_zz = zz_count

        prev_zz = zz_count
        if zz_count == 0:
            break

        # Accumulate offenders and retry with them forced to secondary.
        new_offenders = zz_involved - forced_pairs
        if not new_offenders:
            break  # nothing new to flip; further retries won't help
        forced_pairs |= zz_involved

    tracks, vias_to_add, vias_to_remove, failed_nets, best_routes = best_result

    # Pad-aware repair pass (runs ONCE on the chosen best result). Reroutes any
    # fanout stub whose copper crosses a foreign component pad - the obstacle map
    # / foreign-pad list catch crossings the channel-based fanout would otherwise
    # short across (e.g. MIPI_D1 stubs over C83.1). Reuses the same jog/reroute
    # machinery as collision resolution.
    failed_nets = list(failed_nets)
    n_bad = sum(1 for r in best_routes
                if not route_clear_of_foreign_pads(r, foreign_pads, obstacle_layer_map))
    if n_bad > 0:
        print(f"  Pad-aware check: {n_bad} route(s) cross a foreign pad; repairing...")
        existing_tracks = convert_segments_to_tracks(pcb_data) if check_for_previous else []
        net_id_to_name_all = {r.net_id: r.pad.net_name for r in best_routes if r.pad.net_name}
        pad_repaired, pad_failed = repair_pad_crossings(
            best_routes, tracks, layers, track_width, clearance, diff_pair_gap,
            existing_tracks, grid, channels, exit_margin, net_id_to_name_all,
            no_inner_top_layer, obstacles, obstacle_cfg, obstacle_layer_map, foreign_pads)
        if pad_repaired:
            print(f"  Pad-aware: repaired {pad_repaired} route(s)")
        if pad_failed:
            for nm in pad_failed:
                if nm not in failed_nets:
                    failed_nets.append(nm)
            print(f"  Pad-aware: removed {len(pad_failed)} unroutable net(s): {pad_failed}")

    # Clearance-aware escape clearing (issue #123 PAD-SEGMENT). The repair above
    # fires only on true crossings; a route's outer escape can still graze a
    # breakout-region passive within clearance. Trim the decorative jog and/or
    # shorten the escape (without pulling its free end inside the BGA zone) to
    # clear the pad; if even the minimum escape grazes, drop the ball and warn.
    esc_net_name = {r.net_id: r.pad.net_name for r in best_routes if r.pad.net_name}
    n_escfix, esc_dropped = clear_escapes_of_foreign_pads(
        best_routes, tracks, grid, track_width, clearance,
        foreign_pads, obstacle_layer_map, esc_net_name)
    if n_escfix:
        print(f"  Pad-clearance: cleared {n_escfix} escape(s) grazing a foreign "
              f"pad by jog-trim/shorten (issue #123)")
    if esc_dropped:
        drop_ids = {nid for nid, nm in esc_net_name.items() if nm in esc_dropped}
        tracks[:] = [t for t in tracks if t['net_id'] not in drop_ids]
        vias_to_add[:] = [v for v in vias_to_add if v['net_id'] not in drop_ids]
        for nm in esc_dropped:
            if nm not in failed_nets:
                failed_nets.append(nm)
        print(f"  WARNING: dropped {len(esc_dropped)} ball(s) that cannot escape "
              f"the BGA zone without grazing a foreign pad (issue #123): {esc_dropped}")

    # Via-barrel vs foreign-track clearance (issue #123). A fanout via is a
    # through-hole spanning every copper layer, so the track-vs-track layer
    # assignment can't keep it clear of another net's escape track on an inner
    # layer. When the via/track are too large for the BGA pitch the inter-via
    # channel is narrower than track + 2*clearance and the via copper overlaps a
    # neighbour's track (e.g. ottercast_audio 0.5mm via / 0.2mm track on 0.65mm
    # pitch -> 77 VIA-SEGMENT shorts). Rather than silently emit the short, drop
    # those balls and report them unescaped; a smaller --via-size/--track-width/
    # --clearance retry escapes them cleanly (0.3/0.1 -> 0 VIA-SEGMENT). A small
    # tolerance keeps sub-grid rounding (#70) from triggering spurious drops.
    from geometry_utils import point_to_segment_distance
    via_net_name = {r.net_id: r.pad.net_name for r in best_routes if r.pad.net_name}
    via_clear_tol = 0.02
    via_drop_ids = set()
    for via in vias_to_add:
        vr = via['size'] / 2
        for t in tracks:
            if t['net_id'] == via['net_id']:
                continue
            d = point_to_segment_distance(via['x'], via['y'],
                                          t['start'][0], t['start'][1],
                                          t['end'][0], t['end'][1])
            if d < vr + clearance + t['width'] / 2 - via_clear_tol:
                via_drop_ids.add(via['net_id'])
                break
    if via_drop_ids:
        tracks[:] = [t for t in tracks if t['net_id'] not in via_drop_ids]
        vias_to_add[:] = [v for v in vias_to_add if v['net_id'] not in via_drop_ids]
        for nid in via_drop_ids:
            nm = via_net_name.get(nid)
            if nm and nm not in failed_nets:
                failed_nets.append(nm)
        print(f"  Via-clearance: dropped {len(via_drop_ids)} ball(s) whose via "
              f"would short a foreign track (issue #123); retry with smaller "
              f"--via-size / --track-width / --clearance")

    # Under-pad auto-fallback (issue #288). On dense/locked-neighbour arrays the
    # channel engine deterministically drops balls the under-pad grid escape
    # handles (ecp5_mini HyperRAM 11/14 -> 14/14, icepi_zero ECP5 120/124 ->
    # 124/124). When the channel pass dropped any ball, re-run with underpad and
    # keep whichever escapes more; ties keep the channel result (surface
    # routing, no via-in-pad).
    if failed_nets and escape_method == 'auto':
        print(f"\n  Channel escape dropped {len(failed_nets)} ball(s) "
              f"({', '.join(failed_nets)}) - retrying with the under-pad grid "
              f"escape (issue #288)...")
        up_tracks, up_vias, up_vias_rm, up_failed = generate_bga_fanout(
            footprint, pcb_data, net_filter=net_filter,
            diff_pair_patterns=diff_pair_patterns, layers=underpad_layers,
            track_width=track_width, clearance=clearance,
            diff_pair_gap=diff_pair_gap, exit_margin=exit_margin,
            primary_escape=primary_escape,
            force_escape_direction=force_escape_direction,
            rebalance_escape=rebalance_escape,
            via_size=via_size, via_drill=via_drill,
            check_for_previous=check_for_previous,
            no_inner_top_layer=no_inner_top_layer, escape_method='underpad',
            grid_step=grid_step)
        if len(up_failed) < len(failed_nets):
            print(f"  Under-pad escape wins: {len(failed_nets)} -> "
                  f"{len(up_failed)} dropped ball(s); using it")
            return up_tracks, up_vias, up_vias_rm, up_failed
        print(f"  Under-pad escape did not improve ({len(up_failed)} dropped) - "
              f"keeping the channel result")

    return tracks, vias_to_add, vias_to_remove, failed_nets


def main():
    """Run BGA fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate BGA fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='kicad_files/fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layers', '-l', nargs='+', default=['F.Cu', 'B.Cu'],
                        help='Routing layers (default: F.Cu B.Cu)')
    parser.add_argument('--track-width', '-w', type=float, default=defaults.BGA_TRACK_WIDTH,
                        help=f'Track width in mm (default: {defaults.BGA_TRACK_WIDTH})')
    parser.add_argument('--clearance', type=float, default=defaults.BGA_CLEARANCE,
                        help=f'Track clearance in mm (default: {defaults.BGA_CLEARANCE})')
    parser.add_argument('--via-size', type=float, default=defaults.BGA_VIA_SIZE,
                        help=f'Via outer diameter in mm (default: {defaults.BGA_VIA_SIZE})')
    parser.add_argument('--via-drill', type=float, default=defaults.BGA_VIA_DRILL,
                        help=f'Via drill size in mm (default: {defaults.BGA_VIA_DRILL})')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')
    parser.add_argument('--diff-pairs', '-d', nargs='*',
                        help='Differential pair net patterns (e.g., "*lvds*"). '
                             'Matching P/N pairs will be routed together on same layer.')
    parser.add_argument('--diff-pair-gap', type=float, default=defaults.BGA_DIFF_PAIR_GAP,
                        help=f'Gap between differential pair traces in mm (default: {defaults.BGA_DIFF_PAIR_GAP})')
    parser.add_argument('--exit-margin', type=float, default=defaults.BGA_EXIT_MARGIN,
                        help=f'Distance past BGA boundary (default: {defaults.BGA_EXIT_MARGIN})')
    parser.add_argument('--primary-escape', '-p', choices=['horizontal', 'vertical'],
                        default='horizontal',
                        help='Primary escape direction preference (default: horizontal). '
                             'Pairs will use this direction first, then switch if channels are full.')
    parser.add_argument('--force-escape-direction', action='store_true',
                        help='Only use the primary escape direction (horizontal or vertical). '
                             'Do not fall back to the secondary direction.')
    parser.add_argument('--rebalance-escape', action='store_true',
                        help='Rebalance escape directions after initial assignment. '
                             'Pairs near secondary edge but far from primary edge will be '
                             'reassigned to secondary direction for more even distribution.')
    parser.add_argument('--check-for-previous', action='store_true',
                        help='Check for existing fanout tracks and skip pads that are already '
                             'fanned out. Also avoids occupied channel positions.')
    parser.add_argument('--no-inner-top-layer', action='store_true',
                        help='Prevent inner pads from using F.Cu (top layer). '
                             'Use when there is not enough clearance on top layer for inner routes.')
    parser.add_argument('--escape-method', choices=['auto', 'channel', 'underpad'], default='auto',
                        help='Fanout engine (default: auto). "channel" = 45-stub + '
                             'channel router with diff-pair support. "underpad" = dense-array '
                             'grid escape (issue #122): each signal vias in its pad and routes '
                             'under the pad field on inner layers, escaping fully-populated '
                             'arrays (e.g. ulx3s 22x22) the channel router cannot. Use a small '
                             'via/track for dense pitches (e.g. via 0.35, track 0.12 at 0.8mm). '
                             '"auto" = channel first, and if it drops any ball, retry with '
                             'underpad and keep whichever escapes more (issue #288).')
    parser.add_argument('--grid-step', type=float, default=0.1,
                        help='Routing grid step in mm (default: 0.1). Escape stub ends are '
                             'snapped to this grid so the router gets on-grid terminals (issue '
                             '#149); MATCH the --grid-step you pass to route.py.')
    parser.add_argument('--layer-costs', type=float, nargs='+', default=None,
                        help='Per-layer cost, one value per --layers entry, matching '
                             'route.py semantics (issue #288): negative = forbidden (no '
                             'escape copper on that layer, e.g. a soon-to-be-plane inner '
                             'layer), otherwise a weight in [1.0, 1000] - the channel '
                             'engine fills cheaper layers first. Pass the same values '
                             'you give route.py --layer-costs.')

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    args = parser.parse_args()
    set_default_fab_tier(*fab_tier_from_args(args))
    enforce_fab_floors(
        count_copper_layers_in_file(args.pcb),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None))

    print(f"Parsing {args.pcb}...")
    pcb_data = parse_kicad_pcb(args.pcb)

    # Auto-detect BGA component if not specified
    if args.component is None:
        bga_components = find_components_by_type(pcb_data, 'BGA')
        if bga_components:
            args.component = bga_components[0].reference
            print(f"Auto-detected BGA component: {args.component}")
            if len(bga_components) > 1:
                print(f"  (Other BGAs found: {[fp.reference for fp in bga_components[1:]]})")
        else:
            print("Error: No BGA components found in PCB")
            print(f"Available components: {list(pcb_data.footprints.keys())[:20]}...")
            return 1

    if args.component not in pcb_data.footprints:
        print(f"Error: Component {args.component} not found")
        print(f"Available: {list(pcb_data.footprints.keys())[:20]}...")
        return 1

    footprint = pcb_data.footprints[args.component]
    print(f"\nFound {args.component}: {footprint.footprint_name}")
    print(f"  Position: ({footprint.x:.2f}, {footprint.y:.2f})")
    print(f"  Rotation: {footprint.rotation}deg")
    print(f"  Pads: {len(footprint.pads)}")

    tracks, vias_to_add, vias_to_remove, _failed_nets = generate_bga_fanout(
        footprint,
        pcb_data,
        net_filter=args.nets,
        diff_pair_patterns=args.diff_pairs,
        layers=args.layers,
        track_width=args.track_width,
        clearance=args.clearance,
        diff_pair_gap=args.diff_pair_gap,
        exit_margin=args.exit_margin,
        primary_escape=args.primary_escape,
        force_escape_direction=args.force_escape_direction,
        rebalance_escape=args.rebalance_escape,
        via_size=args.via_size,
        via_drill=args.via_drill,
        check_for_previous=args.check_for_previous,
        no_inner_top_layer=args.no_inner_top_layer,
        escape_method=args.escape_method,
        grid_step=args.grid_step,
        layer_costs=args.layer_costs
    )

    if tracks:
        print(f"\nWriting {len(tracks)} tracks to {args.output}...")
        if vias_to_add:
            print(f"  Adding {len(vias_to_add)} vias")
        if vias_to_remove:
            print(f"  Removing {len(vias_to_remove)} vias")
        net_names = {nid: net.name for nid, net in pcb_data.nets.items()}
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias_to_add,
                                   vias_to_remove, net_id_to_name=net_names)
        print("Done!")
    else:
        print("\nNo fanout tracks generated")
        # Still produce the output file (board unchanged) so a multi-step
        # pipeline can continue - otherwise a fanout that finds nothing to do
        # (e.g. all balls already fanned on a retry) leaves the next step with
        # no input file.
        if getattr(args, 'output', None):
            from pcb_io_utils import passthrough_copy
            passthrough_copy(args.pcb, args.output)
            print(f"Wrote board through to {args.output} (unchanged)")

    # Structured summary so downstream tooling (plan-pcb-routing skill, stress
    # harness) can reliably detect when not all requested balls escaped - and
    # retry at a tighter clearance - instead of scraping per-net FAILED lines.
    # `requested` = balls actually attempted (escaped + dropped); skipped power
    # balls and already-fanned nets are not counted. (issue #122)
    import json as _json
    escaped_net_ids = {t['net_id'] for t in tracks if t.get('net_id') is not None}
    unescaped = sorted(set(_failed_nets))
    escaped = len(escaped_net_ids)
    requested = escaped + len(unescaped)
    if unescaped:
        print(f"\n  {len(unescaped)} of {requested} requested ball(s) could NOT be "
              f"escaped at --clearance {args.clearance}mm / --track-width "
              f"{args.track_width}mm and were DROPPED from the output. Retry the "
              f"fanout with a smaller --clearance (toward the manufacturing floor).")
    # DRC the written output at the routed clearance so downstream tooling can
    # detect sub-clearance grazes the escape left behind even when every ball
    # escaped (failed==0): via-over-track / via-over-pad (#130). The planner uses
    # this to retry the fanout with a smaller via / thinner track toward the fab
    # floor. Subprocess + best-effort: a DRC hiccup must never fail the fanout.
    drc_grazes = {}
    out_path = getattr(args, 'output', None)
    if out_path:
        try:
            import io as _io, contextlib as _cl
            from check_drc import run_drc as _run_drc
            with _cl.redirect_stdout(_io.StringIO()):  # keep JSON_SUMMARY output clean
                # check_sizes=False: drc_grazes grades clearance grazes at
                # --clearance (#130); the issue #176 fab-width floor is a separate
                # concern and would change this 'total's meaning.
                _viols = _run_drc(out_path, clearance=args.clearance,
                                  quiet=True, max_print=0, check_sizes=False)
            _by = {}
            for _v in _viols:
                _by[_v['type']] = _by.get(_v['type'], 0) + 1
            drc_grazes = {
                'pad_via': _by.get('pad-via', 0),
                'via_segment': _by.get('via-segment', 0),
                'pad_segment': _by.get('pad-segment', 0),
                'segment_segment': _by.get('segment-segment', 0),
                'total': len(_viols),
            }
        except Exception as _e:
            drc_grazes = {'error': str(_e)}

    # Establish this fanout's clearance floor in the output .kicad_pro so the next
    # pipeline step (route.py etc.) and check_drc grade at the clearance the
    # fanout actually used -- only lowers, never tightens (issue #160). The
    # fanout routes uniformly at --clearance, but read the run ledger too so a
    # tighter clearance from any shared sub-step is honoured.
    import clearance_ledger as _cl
    eff_clearance = _cl.effective(args.clearance)
    if out_path and os.path.isfile(out_path) \
            and not getattr(args, 'no_fix_drc_settings', False):
        try:
            from fix_kicad_drc_settings import fix_project_for_output
            fix_project_for_output(
                out_path, input_pcb=args.pcb,
                clearance=eff_clearance,
                track_width=args.track_width,
                via_diameter=getattr(args, 'via_size', None),
                via_drill=getattr(args, 'via_drill', None))
        except Exception as _e:
            print(f"  (skipped DRC-settings fix: {_e})")
    summary = {
        'component': args.component,
        'requested': requested,
        'escaped': escaped,
        'failed': len(unescaped),
        'unescaped_nets': unescaped,
        'skipped_nc': len(single_pad_net_ids(footprint, pcb_data)),
        'clearance': args.clearance,
        'track_width': args.track_width,
        'layers': list(args.layers) if args.layers else None,
        # grazes graded at --clearance; 'total' counts ALL DRC violations on the
        # output, the via_*/pad_* keys are the fanout-relevant #130 classes.
        'drc_grazes': drc_grazes,
        # Smallest copper clearance any step actually routed at; downstream steps
        # and check_drc grade the board at this floor.
        'min_clearance_used': eff_clearance,
    }
    print(f"JSON_SUMMARY: {_json.dumps(summary)}")

    return 0


if __name__ == '__main__':
    exit(main())
