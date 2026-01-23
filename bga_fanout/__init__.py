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

import math
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict
import fnmatch

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData, find_components_by_type
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
)
from bga_fanout.escape import (
    find_escape_channel,
    find_diff_pair_escape,
    assign_pair_escapes,
)
from bga_fanout.reroute import find_existing_fanouts, resolve_collisions
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
    diff_pair_gap: float
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
            pair_spacing=pair_spacing
        )
        route.jog_end = jog_end
        route.jog_extension = extension


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
                if not would_overlap_existing_via(pad_x, pad_y, via_size):
                    vias_to_add.append({
                        'x': pad_x,
                        'y': pad_y,
                        'size': via_size,
                        'drill': via_drill,
                        'layers': ['F.Cu', 'B.Cu'],
                        'net_id': route.net_id
                    })

    if vias_to_add:
        print(f"  Adding {len(vias_to_add)} vias at pads on non-top layers")
    if vias_to_remove:
        print(f"  Removing {len(vias_to_remove)} unnecessary vias at pads on top layer")

    return vias_to_add, vias_to_remove


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
                        no_inner_top_layer: bool = False) -> Tuple[List[Dict], List[Dict], List[Dict]]:
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

    Returns:
        Tuple of (tracks, vias_to_add, vias_to_remove)
    """
    if layers is None:
        layers = ["F.Cu", "B.Cu"]

    grid = analyze_bga_grid(footprint)
    if grid is None:
        print(f"Warning: {footprint.reference} doesn't appear to be a BGA")
        return [], [], []

    print(f"BGA Grid Analysis for {footprint.reference}:")
    print(f"  Pitch: {grid.pitch_x:.2f} x {grid.pitch_y:.2f} mm")
    print(f"  Grid: {len(grid.rows)} rows x {len(grid.cols)} columns")
    print(f"  Center: ({grid.center_x:.2f}, {grid.center_y:.2f})")
    print(f"  Boundary: X[{grid.min_x:.2f}, {grid.max_x:.2f}], Y[{grid.min_y:.2f}, {grid.max_y:.2f}]")

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

    # Build routes - process differential pairs together
    routes: List[FanoutRoute] = []
    processed_pairs: Set[str] = set()

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        if net_filter:
            matched = any(fnmatch.fnmatch(pad.net_name, pattern) for pattern in net_filter)
            if not matched:
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
            p_pad = pair.p_pad
            n_pad = pair.n_pad

            # Use pre-assigned escape direction if available, otherwise compute
            if pair_id in pair_escape_assignments:
                channel, escape_dir = pair_escape_assignments[pair_id]
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
            # In cross case, pads converge with 45° stubs (like edge pairs)
            is_cross_escape = False
            if channel:
                if pads_horizontal and escape_dir in ['up', 'down']:
                    is_cross_escape = True
                elif not pads_horizontal and escape_dir in ['left', 'right']:
                    is_cross_escape = True

            # For adjacent-channel mode: find two channels, one on each side of the pads
            # Check per-direction whether adjacent channels are needed
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

            # Determine which pad is "positive" offset and which is "negative"
            # For horizontal channel (left/right escape): offset is in Y direction
            # For vertical channel (up/down escape): offset is in X direction
            #
            # Key insight: To avoid crossing, the pad that is FURTHER from the channel
            # should get the offset that brings it CLOSER to the channel center,
            # while the pad CLOSER to the channel gets offset AWAY from channel center.
            #
            # When using adjacent channels (p_channel != n_channel), each track is centered
            # in its own channel, so offsets are 0.
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
                # Edge pads or cross-escape - no offset needed, they converge with 45° stubs
                p_offset = 0
                n_offset = 0

            # Check for half-edge case
            is_half_edge = escape_dir.startswith('half_edge_')
            if is_half_edge:
                actual_escape_dir = escape_dir.replace('half_edge_', '')
            else:
                actual_escape_dir = escape_dir

            # Create routes for both P and N
            # In adjacent-channel mode, p_channel and n_channel are different
            for pad_info, offset, is_p_route, route_ch in [(p_pad, p_offset, True, p_channel), (n_pad, n_offset, False, n_channel)]:
                if is_half_edge:
                    # Half-edge pair: one pad on edge, one inner
                    # Edge pad: goes straight out to BGA edge
                    # Inner pad: 45° up to channel center, then 45° back down to converge
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

                        # Choose channel direction based on distance to BGA edge
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
                            # Inner pad: 45° up to channel, horizontal in channel (1 pitch),
                            # then 45° back down to converge with edge pad
                            channel_y = inner_channel.position
                            dy_to_channel = channel_y - inner_pad_info.global_y

                            if actual_escape_dir == 'right':
                                # First 45°: pad -> channel entry point
                                channel_pt_x = inner_pad_info.global_x + abs(dy_to_channel)
                                channel_pt = (channel_pt_x, channel_y)

                                # Horizontal segment in channel: 1 pitch toward edge
                                channel_pt2_x = channel_pt_x + grid.pitch_x
                                channel_pt2 = (channel_pt2_x, channel_y)

                                # Target Y at exit = edge pad Y + offset for pair spacing
                                if channel_above:
                                    target_exit_y = edge_pad_info.global_y - pair_spacing_full
                                else:
                                    target_exit_y = edge_pad_info.global_y + pair_spacing_full

                                # Second 45°: from channel_pt2 back toward target_exit_y
                                dy_return = target_exit_y - channel_y
                                stub_end_x = channel_pt2_x + abs(dy_return)
                                stub_end = (stub_end_x, target_exit_y)

                                exit_pos = (grid.max_x + exit_margin, target_exit_y)

                            else:  # left
                                channel_pt_x = inner_pad_info.global_x - abs(dy_to_channel)
                                channel_pt = (channel_pt_x, channel_y)

                                channel_pt2_x = channel_pt_x - grid.pitch_x
                                channel_pt2 = (channel_pt2_x, channel_y)

                                if channel_above:
                                    target_exit_y = edge_pad_info.global_y - pair_spacing_full
                                else:
                                    target_exit_y = edge_pad_info.global_y + pair_spacing_full

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
                            channel_x = inner_channel.position
                            dx_to_channel = channel_x - inner_pad_info.global_x

                            if actual_escape_dir == 'down':
                                # First 45°: pad -> channel entry point
                                channel_pt_y = inner_pad_info.global_y + abs(dx_to_channel)
                                channel_pt = (channel_x, channel_pt_y)

                                # Vertical segment in channel: 1 pitch toward edge
                                channel_pt2_y = channel_pt_y + grid.pitch_y
                                channel_pt2 = (channel_x, channel_pt2_y)

                                # Target X at exit = edge pad X + offset for pair spacing
                                if channel_left:
                                    target_exit_x = edge_pad_info.global_x - pair_spacing_full
                                else:
                                    target_exit_x = edge_pad_info.global_x + pair_spacing_full

                                # Second 45°: from channel_pt2 back toward target_exit_x
                                dx_return = target_exit_x - channel_x
                                stub_end_y = channel_pt2_y + abs(dx_return)
                                stub_end = (target_exit_x, stub_end_y)

                                exit_pos = (target_exit_x, grid.max_y + exit_margin)

                            else:  # up
                                # First 45°: pad -> channel entry point
                                channel_pt_y = inner_pad_info.global_y - abs(dx_to_channel)
                                channel_pt = (channel_x, channel_pt_y)

                                # Vertical segment in channel: 1 pitch toward edge
                                channel_pt2_y = channel_pt_y - grid.pitch_y
                                channel_pt2 = (channel_x, channel_pt2_y)

                                # Target X at exit = edge pad X + offset for pair spacing
                                if channel_left:
                                    target_exit_x = edge_pad_info.global_x - pair_spacing_full
                                else:
                                    target_exit_x = edge_pad_info.global_x + pair_spacing_full

                                # Second 45°: from channel_pt2 back toward target_exit_x
                                dx_return = target_exit_x - channel_x
                                stub_end_y = channel_pt2_y - abs(dx_return)
                                stub_end = (target_exit_x, stub_end_y)

                                exit_pos = (target_exit_x, grid.min_y - exit_margin)

                            route_channel = inner_channel

                    route = FanoutRoute(
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
                    routes.append(route)
                    continue  # Skip the normal edge/inner handling below

                if is_edge or is_cross_escape:
                    # Edge pads or cross-escape: converge with 45° stubs to meet at pair spacing
                    # Cross-escape: horizontal pads escaping vertically, or vertical pads escaping horizontally
                    # Calculate the center point between P and N pads
                    center_x = (p_pad.global_x + n_pad.global_x) / 2
                    center_y = (p_pad.global_y + n_pad.global_y) / 2

                    if pads_horizontal:
                        # Pads are side by side horizontally (like T9 and T10 in screenshot)
                        # They need to converge to pair spacing using 45° stubs
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

                        # At 45°, dy = dx (in absolute terms, direction depends on escape)
                        if escape_dir == 'down':
                            # Going down: Y increases, stub goes at 45° down
                            stub_end_y = pad_info.global_y + abs(dx_needed)
                            stub_end_x = target_x
                        elif escape_dir == 'up':
                            # Going up: Y decreases, stub goes at 45° up
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

                            # Route to target channel via 45° stub
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
                        else:
                            # Original convergence logic
                            # Target Y for converged pair - top pad goes to top target, bottom to bottom
                            if (is_p_route and p_is_top) or (not is_p_route and not p_is_top):
                                # This pad is on top, target is above center
                                target_y = center_y - half_pair_spacing
                            else:
                                # This pad is on bottom, target is below center
                                target_y = center_y + half_pair_spacing

                            # Distance each trace needs to move in Y (towards center)
                            dy_needed = target_y - pad_info.global_y

                            # At 45°, dx = dy (in absolute terms)
                            if escape_dir == 'right':
                                stub_end_x = pad_info.global_x + abs(dy_needed)
                                stub_end_y = target_y
                            elif escape_dir == 'left':
                                stub_end_x = pad_info.global_x - abs(dy_needed)
                                stub_end_y = target_y
                            else:
                                stub_end_x = pad_info.global_x
                                stub_end_y = target_y

                            stub_end = (stub_end_x, stub_end_y)

                            if escape_dir == 'right':
                                exit_pos = (grid.max_x + exit_margin, stub_end[1])
                            elif escape_dir == 'left':
                                exit_pos = (grid.min_x - exit_margin, stub_end[1])
                            elif escape_dir == 'down':
                                exit_pos = (stub_end[0], grid.max_y + exit_margin)
                            else:  # up
                                exit_pos = (stub_end[0], grid.min_y - exit_margin)
                else:
                    # Inner pads with aligned escape: 45° stub to channel with offset, then channel to exit
                    # In adjacent-channel mode, route_ch is the pad-specific channel
                    stub_end = create_45_stub(pad_info.global_x, pad_info.global_y,
                                             route_ch, escape_dir, offset)
                    exit_pos = calculate_exit_point(stub_end, route_ch, escape_dir,
                                                   grid, exit_margin, offset)

                # Use pre-assigned layer if available, otherwise default to layers[0]
                assigned_layer = pair_layer_assignments.get(pair_id, layers[0]) if pair_layer_assignments else layers[0]

                route = FanoutRoute(
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
                routes.append(route)
        else:
            # Single-ended signal (not part of a pair)
            force_orient = primary_escape if force_escape_direction else None
            route = create_single_ended_route(
                pad, grid, channels, layers, exit_margin, force_orient
            )
            routes.append(route)

    print_route_statistics(routes)

    if not routes:
        return [], [], []

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

    # Calculate jog_end for each route based on layer
    calculate_jog_ends_for_routes(routes, layers, jog_length, track_width, diff_pair_gap)

    # Generate tracks
    tracks, edge_count, inner_count = generate_tracks_from_routes(routes, track_width, layers[0])

    # Validate no collisions
    min_spacing = track_width + clearance
    collision_count, collision_pairs = detect_collisions(tracks, existing_tracks, min_spacing)

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
                                        existing_tracks, grid, channels, exit_margin, net_id_to_name, no_inner_top_layer)

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

    # Post-resolution layer rebalancing for even distribution
    if collisions_remaining == 0 and len(layers) > 1:
        rebalanced_count = rebalance_layers(routes, tracks, existing_tracks, layers, min_spacing)
        if rebalanced_count > 0:
            print(f"  Rebalanced {rebalanced_count} routes for even layer distribution")

    # Stats by layer
    layer_counts = defaultdict(int)
    for route in routes:
        layer_counts[route.layer] += 1
    for layer, count in sorted(layer_counts.items()):
        print(f"    {layer}: {count} routes")

    # Via management: add vias where needed, remove unnecessary ones
    vias_to_add, vias_to_remove = manage_vias(
        routes, pcb_data, layers[0], via_size, via_drill, clearance
    )

    return tracks, vias_to_add, vias_to_remove


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
    parser.add_argument('--track-width', '-w', type=float, default=0.3,
                        help='Track width in mm (default: 0.3)')
    parser.add_argument('--clearance', type=float, default=0.25,
                        help='Track clearance in mm (default: 0.25)')
    parser.add_argument('--via-size', type=float, default=0.5,
                        help='Via outer diameter in mm (default: 0.5)')
    parser.add_argument('--via-drill', type=float, default=0.3,
                        help='Via drill size in mm (default: 0.3)')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')
    parser.add_argument('--diff-pairs', '-d', nargs='*',
                        help='Differential pair net patterns (e.g., "*lvds*"). '
                             'Matching P/N pairs will be routed together on same layer.')
    parser.add_argument('--diff-pair-gap', type=float, default=0.1,
                        help='Gap between differential pair traces in mm')
    parser.add_argument('--exit-margin', type=float, default=0.5,
                        help='Distance past BGA boundary')
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

    args = parser.parse_args()

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

    tracks, vias_to_add, vias_to_remove = generate_bga_fanout(
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
        no_inner_top_layer=args.no_inner_top_layer
    )

    if tracks:
        print(f"\nWriting {len(tracks)} tracks to {args.output}...")
        if vias_to_add:
            print(f"  Adding {len(vias_to_add)} vias")
        if vias_to_remove:
            print(f"  Removing {len(vias_to_remove)} vias")
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias_to_add, vias_to_remove)
        print("Done!")
    else:
        print("\nNo fanout tracks generated")

    return 0


if __name__ == '__main__':
    exit(main())
