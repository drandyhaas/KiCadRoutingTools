"""
Rerouting and collision resolution for BGA fanout routing.

Functions for resolving collisions and rerouting signals through alternate channels.
"""

from typing import List, Dict, Tuple, Optional, Set

from kicad_parser import PCBData, Footprint
from bga_fanout.types import (
    create_track,
    Channel,
    BGAGrid,
    FanoutRoute,
)
from bga_fanout.collision import (
    check_segment_collision,
    find_colliding_pairs,
    find_collision_partners,
)
from bga_fanout.constants import FANOUT_DETECTION_TOLERANCE
from bga_fanout.layer_assignment import try_reassign_layer
from bga_fanout.grid import is_edge_pad
from bga_fanout.geometry import (
    create_45_stub,
    calculate_exit_point,
    calculate_jog_end,
)
from bga_fanout.escape import get_alternate_channels_for_pad


def try_reroute_single_ended(route: 'FanoutRoute',
                              alternate_channel: Channel,
                              grid: BGAGrid,
                              exit_margin: float,
                              tracks: List[Dict],
                              existing_tracks: List[Dict],
                              available_layers: List[str],
                              track_width: float,
                              clearance: float,
                              no_inner_top_layer: bool = False) -> Optional[Tuple['FanoutRoute', str]]:
    """
    Try rerouting a single-ended signal through an alternate channel.

    Args:
        route: The current route to reroute
        alternate_channel: The alternate channel to try
        grid: BGA grid
        exit_margin: Exit margin from BGA boundary
        tracks: Current track list (to check for collisions)
        existing_tracks: Existing tracks from PCB
        available_layers: Available layers to try
        track_width: Track width
        clearance: Required clearance
        no_inner_top_layer: If True, inner pads cannot use F.Cu (top layer)

    Returns:
        (new_route, layer) if successful, None if no collision-free option found
    """
    min_spacing = track_width + clearance
    pad_x, pad_y = route.pad_pos

    # Calculate new stub end and exit position with the alternate channel
    new_stub_end = create_45_stub(pad_x, pad_y, alternate_channel, route.escape_dir)
    new_exit_pos = calculate_exit_point(new_stub_end, alternate_channel, route.escape_dir, grid, exit_margin)

    # Create new track segments to check for collisions
    new_segments = [
        {'start': route.pad_pos, 'end': new_stub_end},
        {'start': new_stub_end, 'end': new_exit_pos}
    ]

    # If no_inner_top_layer is set, exclude F.Cu for inner routes
    if no_inner_top_layer:
        candidate_layers = available_layers[1:] if len(available_layers) > 1 else available_layers
    else:
        candidate_layers = available_layers

    # Get other tracks (excluding this route's current tracks)
    other_new_tracks = [t for t in tracks if t.get('net_id') != route.net_id]
    all_other_tracks = other_new_tracks + existing_tracks

    # Try each layer
    for layer in candidate_layers:
        has_collision = False
        for seg in new_segments:
            for other in all_other_tracks:
                if other['layer'] != layer:
                    continue
                if other.get('net_id') == route.net_id:
                    continue
                if check_segment_collision(seg['start'], seg['end'],
                                            other['start'], other['end'],
                                            min_spacing):
                    has_collision = True
                    break
            if has_collision:
                break

        if not has_collision:
            # Found a collision-free route
            new_route = FanoutRoute(
                pad=route.pad,
                pad_pos=route.pad_pos,
                stub_end=new_stub_end,
                exit_pos=new_exit_pos,
                channel=alternate_channel,
                escape_dir=route.escape_dir,
                is_edge=route.is_edge,
                layer=layer,
                pair_id=None,
                is_p=None
            )
            return new_route, layer

    return None


def get_distance_to_escape_edge(route: 'FanoutRoute', grid: BGAGrid) -> float:
    """
    Calculate how far the route's pad is from the escape edge.
    Larger value = farther from edge = should be jogged.
    """
    pad_x, pad_y = route.pad_pos
    escape_dir = route.escape_dir

    if escape_dir == 'right':
        return grid.max_x - pad_x  # farther from right edge = larger distance
    elif escape_dir == 'left':
        return pad_x - grid.min_x  # farther from left edge = larger distance
    elif escape_dir == 'down':
        return grid.max_y - pad_y  # farther from bottom edge = larger distance
    elif escape_dir == 'up':
        return pad_y - grid.min_y  # farther from top edge = larger distance
    return 0.0


def get_farther_channel(route: 'FanoutRoute', channels: List[Channel],
                        grid: BGAGrid) -> Optional[Channel]:
    """
    Get a channel one unit farther from the escape edge than the current channel.

    For a route escaping right with a horizontal channel at Y=105.0,
    this returns the horizontal channel at Y=104.2 (one pitch closer to center,
    i.e., farther from the right edge in terms of distance the track travels).
    """
    if route.channel is None:
        return None

    pitch = grid.pitch_y if route.channel.orientation == 'horizontal' else grid.pitch_x
    pad_x, pad_y = route.pad_pos

    if route.channel.orientation == 'horizontal':
        # For left/right escape, find channel one pitch farther from edge
        # "Farther from edge" means the route has to travel more vertically
        same_orientation = [c for c in channels if c.orientation == 'horizontal' and c != route.channel]

        if route.channel.position < pad_y:
            # Current channel is above pad - farther means even more above (smaller Y)
            candidates = [c for c in same_orientation if c.position < route.channel.position]
            if candidates:
                # Get closest to current channel (largest Y among those above)
                return max(candidates, key=lambda c: c.position)
        else:
            # Current channel is below pad - farther means even more below (larger Y)
            candidates = [c for c in same_orientation if c.position > route.channel.position]
            if candidates:
                # Get closest to current channel (smallest Y among those below)
                return min(candidates, key=lambda c: c.position)
    else:
        # For up/down escape, find channel one pitch farther from edge
        same_orientation = [c for c in channels if c.orientation == 'vertical' and c != route.channel]

        if route.channel.position < pad_x:
            # Current channel is left of pad - farther means even more left (smaller X)
            candidates = [c for c in same_orientation if c.position < route.channel.position]
            if candidates:
                return max(candidates, key=lambda c: c.position)
        else:
            # Current channel is right of pad - farther means even more right (larger X)
            candidates = [c for c in same_orientation if c.position > route.channel.position]
            if candidates:
                return min(candidates, key=lambda c: c.position)

    return None


def try_jogged_route(route: 'FanoutRoute',
                     farther_channel: Channel,
                     grid: BGAGrid,
                     exit_margin: float,
                     tracks: List[Dict],
                     existing_tracks: List[Dict],
                     available_layers: List[str],
                     track_width: float,
                     clearance: float,
                     jog_length: float = None,
                     no_inner_top_layer: bool = False) -> Optional[Tuple['FanoutRoute', str, List[Dict]]]:
    """
    Try rerouting a signal through a farther channel using a jogged path.

    The jogged path is: pad -> 45° stub -> vertical/horizontal jog -> farther channel -> exit -> jog_end

    For example, for a right-escaping route with channel below the pad:
    - 45° stub from pad to original channel position
    - Vertical jog DOWN one pitch to the farther channel
    - Horizontal along farther channel to exit
    - 45° jog at exit

    Args:
        route: The current route to reroute
        farther_channel: The channel one unit farther from the escape edge
        grid: BGA grid
        exit_margin: Exit margin from BGA boundary
        tracks: Current track list (to check for collisions)
        existing_tracks: Existing tracks from PCB
        available_layers: Available layers to try
        track_width: Track width
        clearance: Required clearance
        jog_length: Length of the 45° jog at exit (defaults to half pitch)

    Returns:
        (new_route, layer, new_tracks) if successful, None if no collision-free option found
    """
    min_spacing = track_width + clearance
    pad_x, pad_y = route.pad_pos

    if route.channel is None or farther_channel is None:
        return None

    # Default jog_length to half pitch
    if jog_length is None:
        jog_length = min(grid.pitch_x, grid.pitch_y) / 2

    # Calculate the jogged path
    # Step 1: 45° stub to original channel position (same as normal route)
    stub_end = create_45_stub(pad_x, pad_y, route.channel, route.escape_dir)

    # Step 2: Jog from stub_end to farther channel
    if route.channel.orientation == 'horizontal':
        # Jog is vertical (Y changes, X stays same)
        jog_point = (stub_end[0], farther_channel.position)
    else:
        # Jog is horizontal (X changes, Y stays same)
        jog_point = (farther_channel.position, stub_end[1])

    # Step 3: Calculate exit position on farther channel
    exit_pos = calculate_exit_point(jog_point, farther_channel, route.escape_dir, grid, exit_margin)

    # Create track segments for collision checking
    new_segments = [
        {'start': route.pad_pos, 'end': stub_end},      # 45° stub
        {'start': stub_end, 'end': jog_point},          # Vertical/horizontal jog
        {'start': jog_point, 'end': exit_pos}           # Channel to exit
    ]

    # If no_inner_top_layer is set, exclude F.Cu for inner routes
    if no_inner_top_layer:
        candidate_layers = available_layers[1:] if len(available_layers) > 1 else available_layers
    else:
        candidate_layers = available_layers

    # Get other tracks (excluding this route's current tracks)
    other_new_tracks = [t for t in tracks if t.get('net_id') != route.net_id]
    all_other_tracks = other_new_tracks + existing_tracks

    # Try each layer
    for layer in candidate_layers:
        has_collision = False
        for seg in new_segments:
            for other in all_other_tracks:
                if other['layer'] != layer:
                    continue
                if other.get('net_id') == route.net_id:
                    continue
                if check_segment_collision(seg['start'], seg['end'],
                                           other['start'], other['end'],
                                           min_spacing):
                    has_collision = True
                    break
            if has_collision:
                break

        if not has_collision:
            # Calculate jog_end for the exit
            jog_end, _ = calculate_jog_end(
                exit_pos,
                route.escape_dir,
                layer,
                available_layers,
                jog_length,
                is_diff_pair=route.pair_id is not None,
                is_outside_track=False,
                pair_spacing=0
            )

            # Found a collision-free route
            new_route = FanoutRoute(
                pad=route.pad,
                pad_pos=route.pad_pos,
                stub_end=stub_end,
                exit_pos=exit_pos,
                jog_end=jog_end,
                pre_channel_jog=jog_point,
                channel=farther_channel,
                escape_dir=route.escape_dir,
                is_edge=route.is_edge,
                layer=layer,
                pair_id=route.pair_id,
                is_p=route.is_p
            )

            # Create track dicts for this route (including jog at exit)
            new_tracks = [
                create_track(route.pad_pos, stub_end, track_width, layer, route.net_id, route.pair_id),
                create_track(stub_end, jog_point, track_width, layer, route.net_id, route.pair_id),
                create_track(jog_point, exit_pos, track_width, layer, route.net_id, route.pair_id),
                create_track(exit_pos, jog_end, track_width, layer, route.net_id, route.pair_id),
            ]

            return new_route, layer, new_tracks

    return None


def find_existing_fanouts(pcb_data: PCBData, footprint: Footprint,
                          grid: BGAGrid, channels: List[Channel],
                          tolerance: float = FANOUT_DETECTION_TOLERANCE) -> Tuple[Set[int], Dict[Tuple[str, str, float], str]]:
    """
    Find pads that already have fanout tracks and identify occupied channel positions.

    A pad is considered to have an existing fanout if there's a segment that starts
    or ends at the pad position.

    Args:
        pcb_data: Full PCB data with segments
        footprint: The BGA footprint
        grid: BGA grid structure
        channels: List of routing channels
        tolerance: Position matching tolerance in mm

    Returns:
        Tuple of:
        - Set of net_ids that already have fanouts
        - Dict of occupied exit positions: (layer, direction_axis, position) -> net_name
    """
    fanned_out_nets: Set[int] = set()
    occupied_exits: Dict[Tuple[str, str, float], str] = {}

    # Build a lookup of pad positions by net_id
    pad_positions: Dict[int, Tuple[float, float]] = {}
    net_names: Dict[int, str] = {}
    for pad in footprint.pads:
        if pad.net_id > 0:
            pad_positions[pad.net_id] = (pad.global_x, pad.global_y)
            net_names[pad.net_id] = pad.net_name

    # Check each segment to see if it connects to a BGA pad
    for segment in pcb_data.segments:
        if segment.net_id not in pad_positions:
            continue

        pad_x, pad_y = pad_positions[segment.net_id]

        # Check if segment starts or ends at the pad
        starts_at_pad = (abs(segment.start_x - pad_x) < tolerance and
                         abs(segment.start_y - pad_y) < tolerance)
        ends_at_pad = (abs(segment.end_x - pad_x) < tolerance and
                       abs(segment.end_y - pad_y) < tolerance)

        if starts_at_pad or ends_at_pad:
            fanned_out_nets.add(segment.net_id)

            # Determine the exit direction and position from the segment
            # The "other end" of the segment (not at pad) tells us the escape direction
            if starts_at_pad:
                other_x, other_y = segment.end_x, segment.end_y
            else:
                other_x, other_y = segment.start_x, segment.start_y

            # Determine escape direction based on where the segment goes
            dx = other_x - pad_x
            dy = other_y - pad_y

            # Find which channel/exit this segment uses
            # For vertical escape (up/down), track X position
            # For horizontal escape (left/right), track Y position
            if abs(dy) > abs(dx):
                # Primarily vertical movement
                if dy < 0:
                    direction = 'up'
                else:
                    direction = 'down'
                # Find the vertical channel being used
                for ch in channels:
                    if ch.orientation == 'vertical':
                        if abs(ch.position - other_x) < grid.pitch_x / 2:
                            exit_key = (segment.layer, f'{direction}_v', round(ch.position, 1))
                            occupied_exits[exit_key] = net_names.get(segment.net_id, f"net_{segment.net_id}")
                            break
            else:
                # Primarily horizontal movement
                if dx < 0:
                    direction = 'left'
                else:
                    direction = 'right'
                # Find the horizontal channel being used
                for ch in channels:
                    if ch.orientation == 'horizontal':
                        if abs(ch.position - other_y) < grid.pitch_y / 2:
                            exit_key = (segment.layer, f'{direction}_h', round(ch.position, 1))
                            occupied_exits[exit_key] = net_names.get(segment.net_id, f"net_{segment.net_id}")
                            break

    return fanned_out_nets, occupied_exits


def resolve_collisions(routes: List[FanoutRoute], tracks: List[Dict],
                       available_layers: List[str], track_width: float,
                       clearance: float, diff_pair_spacing: float,
                       existing_tracks: List[Dict] = None,
                       grid: BGAGrid = None,
                       channels: List[Channel] = None,
                       exit_margin: float = 0.5,
                       net_names: Dict[int, str] = None,
                       no_inner_top_layer: bool = False) -> Tuple[int, List[str]]:
    """Try to resolve collisions by reassigning layers for colliding pairs.

    First tries layer reassignment. If that fails for single-ended signals,
    tries routing through the ONE alternate channel on the opposite side of the pad.
    If both options fail, the route is removed and reported as failed.

    Args:
        existing_tracks: Read-only list of existing tracks to check against but not modify
        grid: BGA grid (needed for alternate channel routing)
        channels: List of routing channels (needed for alternate channel routing)
        exit_margin: Exit margin from BGA boundary
        net_names: Dict mapping net_id to net name for error reporting

    Returns:
        Tuple of (number resolved, list of failed net names)
    """
    if existing_tracks is None:
        existing_tracks = []
    if net_names is None:
        net_names = {}

    min_spacing = track_width + clearance
    # Include existing tracks in collision detection
    all_tracks = tracks + existing_tracks
    colliding_pairs = find_colliding_pairs(all_tracks, min_spacing)

    if not colliding_pairs:
        return 0, []

    reassigned = 0
    rerouted = 0
    failed_nets: List[str] = []
    # Track which pairs have been moved this round to avoid moving collision partners to same layer
    moved_this_round: Dict[str, str] = {}

    for identifier in sorted(colliding_pairs):
        # Find collision partners to avoid their new layers
        partners = find_collision_partners(identifier, all_tracks, min_spacing)
        avoid_layers = set()
        for partner in partners:
            if partner in moved_this_round:
                avoid_layers.add(moved_this_round[partner])

        new_layer = try_reassign_layer(identifier, routes, tracks, available_layers,
                                        track_width, clearance, diff_pair_spacing,
                                        avoid_layers, existing_tracks, no_inner_top_layer)
        if new_layer:
            # Determine if this is a single-ended net or a diff pair
            is_single_ended = identifier.startswith('net_')

            if is_single_ended:
                net_id = int(identifier[4:])
                # Update routes for this single-ended net
                for route in routes:
                    if route.net_id == net_id and route.pair_id is None:
                        route.layer = new_layer
                # Update tracks for this single-ended net
                for track in tracks:
                    if track.get('net_id') == net_id and not track.get('pair_id'):
                        track['layer'] = new_layer
            else:
                # Update routes for this diff pair
                for route in routes:
                    if route.pair_id == identifier:
                        route.layer = new_layer
                # Update tracks for this diff pair
                for track in tracks:
                    if track.get('pair_id') == identifier:
                        track['layer'] = new_layer

            moved_this_round[identifier] = new_layer
            reassigned += 1
            print(f"    Reassigned {identifier} to {new_layer}")
        else:
            # Layer reassignment failed - try alternate channel for single-ended signals
            is_single_ended = identifier.startswith('net_')
            resolved = False

            if is_single_ended and grid is not None and channels is not None:
                net_id = int(identifier[4:])
                # Find the route for this net
                route = None
                for r in routes:
                    if r.net_id == net_id and r.pair_id is None:
                        route = r
                        break

                if route and route.channel is not None:
                    # Get THE alternate channel (only one - on the opposite side)
                    alt_channels = get_alternate_channels_for_pad(
                        route.pad_pos[0], route.pad_pos[1],
                        route.channel, route.escape_dir, channels
                    )

                    # Try the alternate channel (there's only one)
                    for alt_channel in alt_channels:
                        result = try_reroute_single_ended(
                            route, alt_channel, grid, exit_margin,
                            tracks, existing_tracks, available_layers,
                            track_width, clearance, no_inner_top_layer
                        )
                        if result:
                            new_route, new_layer = result

                            # Remove old tracks for this net
                            old_track_indices = [i for i, t in enumerate(tracks)
                                                if t.get('net_id') == net_id and not t.get('pair_id')]
                            for idx in sorted(old_track_indices, reverse=True):
                                tracks.pop(idx)

                            # Update route in routes list
                            route_idx = routes.index(route)
                            routes[route_idx] = new_route

                            # Add new tracks
                            tracks.append(create_track(new_route.pad_pos, new_route.stub_end,
                                                       track_width, new_layer, net_id))
                            tracks.append(create_track(new_route.stub_end, new_route.exit_pos,
                                                       track_width, new_layer, net_id))

                            rerouted += 1
                            resolved = True
                            print(f"    Rerouted {identifier} to alternate channel on {new_layer}")
                            break

                # If still not resolved, try to move a conflicting route to a jogged path
                if not resolved and route is not None:
                    # Find routes that conflict with this route's desired channel
                    conflicting_routes = []
                    for other_route in routes:
                        if other_route.net_id == route.net_id:
                            continue
                        if other_route.channel is None:
                            continue
                        # Check if they use the same channel (or the alternate channel we tried)
                        if other_route.channel == route.channel:
                            conflicting_routes.append(other_route)
                        elif alt_channels and other_route.channel in alt_channels:
                            conflicting_routes.append(other_route)

                    # Try to move the route that is FARTHER from escape edge to a jogged path
                    # The route closer to the edge should keep the normal channel
                    route_dist = get_distance_to_escape_edge(route, grid)

                    for conflicting in conflicting_routes:
                        conflicting_dist = get_distance_to_escape_edge(conflicting, grid)

                        # Decide which route to jog based on distance to escape edge
                        if route_dist > conflicting_dist:
                            # route is farther from edge - jog route instead of conflicting
                            to_jog = route
                            to_keep = conflicting
                        else:
                            # conflicting is farther from edge - jog conflicting
                            to_jog = conflicting
                            to_keep = route

                        farther_ch = get_farther_channel(to_jog, channels, grid)
                        if farther_ch is None:
                            continue

                        result = try_jogged_route(
                            to_jog, farther_ch, grid, exit_margin,
                            tracks, existing_tracks, available_layers,
                            track_width, clearance, None, no_inner_top_layer
                        )
                        if result:
                            new_jogged_route, new_layer, new_jogged_tracks = result
                            jogged_net_name = net_names.get(to_jog.net_id, f"net_{to_jog.net_id}")

                            # Remove old tracks for the route being jogged
                            old_track_indices = [i for i, t in enumerate(tracks)
                                                if t.get('net_id') == to_jog.net_id]
                            for idx in sorted(old_track_indices, reverse=True):
                                tracks.pop(idx)

                            # Update route in routes list
                            jog_idx = routes.index(to_jog)
                            routes[jog_idx] = new_jogged_route

                            # Add new tracks for the jogged route
                            tracks.extend(new_jogged_tracks)

                            print(f"    Moved {jogged_net_name} to jogged path on {new_layer}")

                            if to_jog is route:
                                # We jogged the original failing route - it's now resolved
                                rerouted += 1
                                resolved = True
                                break
                            else:
                                # We jogged the conflicting route - now retry the original route
                                # First try the original channel (now freed up)
                                retry_result = try_reroute_single_ended(
                                    route, route.channel, grid, exit_margin,
                                    tracks, existing_tracks, available_layers,
                                    track_width, clearance, no_inner_top_layer
                                )
                                if retry_result:
                                    new_route, retry_layer = retry_result

                                    # Remove old tracks for this net
                                    old_track_indices = [i for i, t in enumerate(tracks)
                                                        if t.get('net_id') == net_id and not t.get('pair_id')]
                                    for idx in sorted(old_track_indices, reverse=True):
                                        tracks.pop(idx)

                                    # Update route in routes list
                                    route_idx = routes.index(route)
                                    routes[route_idx] = new_route

                                    # Add new tracks
                                    tracks.append(create_track(new_route.pad_pos, new_route.stub_end,
                                                               track_width, retry_layer, net_id))
                                    tracks.append(create_track(new_route.stub_end, new_route.exit_pos,
                                                               track_width, retry_layer, net_id))

                                    rerouted += 1
                                    resolved = True
                                    print(f"    Rerouted {identifier} after freeing channel on {retry_layer}")
                                    break

                # If STILL not resolved, remove the route and report failure
                if not resolved and route is not None:
                    net_name = net_names.get(net_id, f"net_{net_id}")
                    failed_nets.append(net_name)

                    # Remove the failed route from routes list
                    if route in routes:
                        routes.remove(route)

                    # Remove tracks for this net
                    old_track_indices = [i for i, t in enumerate(tracks)
                                        if t.get('net_id') == net_id and not t.get('pair_id')]
                    for idx in sorted(old_track_indices, reverse=True):
                        tracks.pop(idx)

                    print(f"    FAILED: Could not route {net_name} - no collision-free path found")

    if rerouted > 0:
        print(f"  Rerouted {rerouted} signals to alternate channels")

    return reassigned + rerouted, failed_nets
