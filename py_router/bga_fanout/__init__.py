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
from collections import defaultdict, Counter

import sys
import os
import env_knobs
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
    staggered_lattice_diagnosis,
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
    PendingVias,
    thin_drill_to_clear,
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

# #472: nets deferred from fanout by the last generate_bga_fanout entry call
_direct_route_nets: Set[str] = set()
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
    force_orientation: Optional[str] = None,
    preferred_dir: Optional[str] = None
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
        force_orientation=force_orientation,
        preferred_dir=preferred_dir
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


# Keyed on (path, resolved, declared), because `manage_vias` runs once per
# RETRY pass and the floor cannot change between them -- four identical
# lines per fanout is noise, not information. PROCESS-wide, like
# obstacle_map's `_HOLE_CLR_ANNOUNCED`, and with the same trap: an
# in-process A/B sees the line on ONE arm only.
_H2H_ANNOUNCED = set()


def manage_vias(
    routes: List[FanoutRoute],
    pcb_data: 'PCBData',
    top_layer: str,
    via_size: float,
    via_drill: float,
    clearance: float,
) -> Tuple[List[Dict], List[Dict], List['FanoutRoute']]:
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
        Tuple of (vias_to_add, vias_to_remove, via_blocked_routes)

    D10's SELF-BLINDNESS, CLOSED HERE (#620). `vias_to_add` was appended to
    below and never read back, and BOTH guards that gate an append iterate
    `pcb_data.vias` only: `would_overlap_existing_via` and
    `via_in_pad_conflict`'s drill loop. So two vias added in ONE call were
    never tested against each other. `PendingVias` (bga_fanout/geometry.py) is
    the missing half, and its docstring argues what it tests -- the drill
    always, the ring only for a via the clamp could not fit inside its pad --
    from a sweep rather than from first principles.

    The impact, which the issue deliberately left UNMEASURED, is measured now:
    on an empty board two 0.30mm-pitch balls with 0.25mm pads ship holes
    0.15mm apart against a 0.20mm fab floor, `added=2 blocked=0`, and two
    coincident same-net balls ship TWO `(via ...)` s-exprs at one point. The
    issue's own worked example -- 0.5mm pitch, via 0.45, clearance 0.1 -- was
    right to hedge: `clamp_via_to_pad` shrinks that via to the ball pad first
    and the pair is legal. **No board in this repo reaches the refusal branch
    at CLI defaults**, which is why the evidence is a constructed fixture and
    the corpus arms are a safety check, not a headline.

    The cost is stated where it is paid: a refusal drops the escape (see the
    #756 block below), so the conflict branch descends the fab drill ladder
    before giving one up, and `--fab-overrides` -- which collapses that ladder
    to one hard rung -- cannot descend at all. That is the configuration the
    #620 contributor measured as pure loss, and their measurement is why this
    has a ladder and a same-site rule at all.

    RESIDUAL, named rather than hidden: a conflict is resolved against the vias
    committed SO FAR, while the two whole-net drops that run after this
    function returns (`esc_dropped`, and the via-vs-foreign-track pass) can
    later delete a via a refusal was measured against. Twins are immune -- same
    net, dropped together -- but a distinct-pair loser is not recoverable once
    its tracks are gone.

    `via_in_pad_conflict`'s drill floor was the OTHER gap here -- priced at
    the flat `routing_defaults.HOLE_TO_HOLE_CLEARANCE` rather than the
    board's own `min_hole_to_hole`, the D9/D11 substitute-a-constant class.
    **Closed by #756**, in the shape `qfn_fanout` and `underpad.py` already
    use: board-first through `list_nets.board_floor`, wrapped raise-only at
    the fab floor. Both of its arms take that one number; it deliberately does
    NOT adopt the 0.45 `pad_hole_to_hole` the via-nudge charges for the same
    via-drill-to-pad-drill pair (see the comment at the resolution site).

    **The cost, which the via-nudge half does not pay:** a refusal here
    DROPS the escape rather than searching again, so a tighter floor can
    turn an escaped ball into a failed net. The via-nudge answers that with
    a two-rung ladder; this pass had no second rung to descend to. Stated
    at the resolution site with the numbers. #620 gives the SAME-CALL arm one
    (`thin_drill_to_clear`); the board-facing arm above still has none, so a
    board declaration can still refuse an escape outright.
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

    # #370 B4: physical validation the body-overlap test above misses,
    # mirroring check_drc / _bare_pad_pair_vias_fit. Precomputed once: pads
    # are static and manage_vias runs before any of ITS vias exist.
    from routing_defaults import HOLE_TO_HOLE_CLEARANCE
    from kicad_parser import pad_drill_capsule
    # #756: DRILL FLOOR, board-first and RAISE-only. `via_in_pad_conflict`
    # below spaced every drill it places at the flat HOLE_TO_HOLE_CLEARANCE and
    # read the board nowhere -- the gap this function's own docstring named as
    # "the D9/D11 substitute-a-constant class, unclosed here". So a board
    # declaring min_hole_to_hole 0.3 got its via-in-pad drills spaced at 0.2,
    # while check_drc graded them at 0.3 (`_pin_up`, check_drc.py:3686) and
    # flagged the pairs this pass had just placed.
    #
    # Shape follows the sibling that already does it -- `underpad.py`'s
    # `_h2h_decl`/`_h2h_fab`/`max` block -- so a reviewer diffs the two and
    # sees one rule. NOT verbatim, and an earlier draft of this comment said
    # it was: underpad spells the fab lookup `.get('hole_to_hole', 0.0)` and
    # this one defaults to the packaged floor instead, which is the safer
    # miss. RAISE-ONLY IN THE CODE, not only in this comment: `board_floor`
    # is board-AUTHORITATIVE and will happily hand back a declared 0.10, so
    # the fab floor is wrapped in explicitly.
    #
    # A REFUSAL HERE DROPS THE ESCAPE, and that is the cost of the
    # board-first read. Unlike the via-nudge -- which re-sweeps at the fab
    # floor rather than abandon a repair -- `via_in_pad_conflict`'s reason
    # sends the route to `via_blocked_routes`, whose tracks are removed and
    # whose net joins `failed_nets`. So a tighter floor can turn an escaped
    # ball into a failed net whenever it is the only thing refusing it.
    # Measured on this file's own rig shape: a foreign 0.30 pad drill at
    # 0.45mm separation is ADDED on a board declaring nothing and BLOCKED on
    # one declaring 0.25.
    #
    # ON A REAL BOARD IT IS INERT, and the honest way to say that is a
    # HEAD-vs-BASE comparison rather than a 0.20-vs-0.30 one. An independent
    # re-measurement ran `bga_fanout.py -c U4` on orangecrab_ext_pll with a
    # planted project at 0.20 and at 0.30: the two DECLARATIONS differ (2
    # tracks / 12 vias vs 1 / 12), but that difference is PRE-EXISTING --
    # `underpad.py` already read the board -- and running the same pair on
    # the base tree gives byte-identical summaries on both arms. So #756's
    # change to THIS function is inert there. An earlier version of this
    # comment cited '9 tracks, 20 vias on both arms', which does not
    # reproduce and compared the wrong pair.
    #
    # A ladder like the via-nudge's would need this pass to have a second
    # rung to descend to, and it does not.
    #
    # NARROW ON PURPOSE. Both arms keep the 0.20 `hole_to_hole` fab term; this
    # does NOT adopt the 0.45 `pad_hole_to_hole` that
    # placement/fanout_clearance's via-nudge charges for the very same
    # via-drill-to-pad-drill pair. The two passes disagree by 0.25mm about that
    # pair and #756 reconciles neither: doing so would move keep-outs on every
    # fine-pitch BGA via-in-pad escape and needs its own before/after. Named
    # here rather than silently unified.
    #
    # Layer count off `board_info`, matching the `copper` this function already
    # derives for `fab_floor_ladder` below -- one board, one count.
    from list_nets import board_floor as _board_floor
    _cu_n = len(getattr(pcb_data.board_info, 'copper_layers', None) or []) or 4
    _h2h_fab = fab_floor_min(_cu_n).get('hole_to_hole', HOLE_TO_HOLE_CLEARANCE)
    # GUARDED, unlike the qfn/underpad twins. `board_floor("")` ->
    # `read_design_rules("")` -> `splitext("")[0] + '.kicad_pro'`, probed
    # relative to the PROCESS CWD, so a stray file of that name is read as
    # this board's rules -- and `build_pcb_data_from_board` yields
    # `source_path=""` for an unsaved board, which is the GUI fanout path.
    # Measured before the guard: a CWD project declaring 5.0 made this pass
    # announce "Hole-to-hole 5mm (from the board's own min_hole_to_hole)"
    # for a board it had never read. `isdir` too, for a directory-shaped
    # path, which `splitext` leaves intact.
    _src_path = getattr(pcb_data, 'source_path', "") or ""
    if not _src_path or os.path.isdir(_src_path):
        _h2h_decl, _h2h_src = HOLE_TO_HOLE_CLEARANCE, 'fixed default'
    else:
        _h2h_decl, _h2h_src = _board_floor(_src_path, 'hole_to_hole', None,
                                           HOLE_TO_HOLE_CLEARANCE)
    _h2h = max(_h2h_decl, _h2h_fab)
    # `routes` guard: `manage_vias([])` is a real call shape and used to
    # read the project and announce a floor for a pass with nothing to
    # place. The dedupe is because this runs once per RETRY pass.
    if routes and _h2h_src == 'board constraint':
        _key = (_src_path, _h2h, _h2h_decl)
        if _key in _H2H_ANNOUNCED:
            pass
        elif _h2h_decl > _h2h_fab:
            _H2H_ANNOUNCED.add(_key)
            print(f"  Hole-to-hole {_h2h:g}mm (from the board's own "
                  f"min_hole_to_hole)")
        elif _h2h_decl < _h2h_fab:
            _H2H_ANNOUNCED.add(_key)
            print(f"  Board min_hole_to_hole {_h2h_decl:g}mm is below the "
                  f"{_h2h_fab:g}mm fab hole-to-hole floor; using "
                  f"{_h2h:g}mm.")
    drill_capsules = []
    for _pads in pcb_data.pads_by_net.values():
        for _p in _pads:
            if (getattr(_p, 'drill', 0) or 0) > 0:
                drill_capsules.append(pad_drill_capsule(_p))

    def via_in_pad_conflict(x, y, v_size, v_drill, net_id):
        """Reason a via physically cannot go at (x, y), or None (#370 B4):

        * drill hole-to-hole vs EVERY existing via and TH/NPTH pad drill --
          net-INDEPENDENT, exactly the fab floor (same-net drill overlap is
          still a fab violation, #282); slot/offset drills via the capsule;
        * the via ring vs foreign tracks on ANY layer (a through via's
          barrel spans every copper layer).
        """
        vdr = (v_drill or 0.0) / 2.0
        for ov in pcb_data.vias:
            if math.hypot(ov.x - x, ov.y - y) < \
                    vdr + (getattr(ov, 'drill', 0) or 0) / 2.0 \
                    + _h2h - 1e-6:
                return "drill hole-to-hole vs an existing via"
        for (c1x, c1y), (c2x, c2y), prad in drill_capsules:
            ddx, ddy = c2x - c1x, c2y - c1y
            L2 = ddx * ddx + ddy * ddy
            if L2 > 0:
                t = max(0.0, min(1.0, ((x - c1x) * ddx + (y - c1y) * ddy) / L2))
                d = math.hypot(x - (c1x + t * ddx), y - (c1y + t * ddy))
            else:
                d = math.hypot(x - c1x, y - c1y)
            if d < vdr + prad + _h2h - 1e-6:
                return "drill hole-to-hole vs a pad drill"
        vr = v_size / 2.0
        for s in pcb_data.segments:
            if s.net_id == net_id:
                continue
            need = vr + (s.width or 0.0) / 2.0 + clearance - 1e-6
            if (x < min(s.start_x, s.end_x) - need or
                    x > max(s.start_x, s.end_x) + need or
                    y < min(s.start_y, s.end_y) - need or
                    y > max(s.start_y, s.end_y) + need):
                continue
            ddx, ddy = s.end_x - s.start_x, s.end_y - s.start_y
            L2 = ddx * ddx + ddy * ddy
            if L2 > 0:
                t = max(0.0, min(1.0, ((x - s.start_x) * ddx
                                       + (y - s.start_y) * ddy) / L2))
                d = math.hypot(x - (s.start_x + t * ddx),
                               y - (s.start_y + t * ddy))
            else:
                d = math.hypot(x - s.start_x, y - s.start_y)
            if d < need:
                return "via ring grazes a foreign track"
        return None

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

    # #620: this run's OWN output, queryable as it grows. Every guard above
    # scans `pcb_data`, and `vias_to_add` was appended to and never read back,
    # so two vias placed in one call were spaced against the input board and
    # against nothing else. `PendingVias` is the missing half; what it tests
    # and what it deliberately does not is argued at its definition.
    _pending = PendingVias(_h2h, clearance)
    _twin_shared = 0       # coincident same-net sites served by ONE hole
    _thinned = []          # (from, to) drills a deeper fab rung rescued
    _pending_refused = []  # (net name, reason) escapes no rung could place

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
                    'y': existing_via.y,
                    # #369 A9: lets the writer's removal refuse to delete a
                    # DIFFERENT net's via coincidentally at this position
                    'net_id': existing_via.net_id
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
                # #370 B4: drill hole-to-hole (net-independent) and foreign
                # tracks -- fail the escape honestly like the #253 case above
                # rather than ship a via check_drc / the fab would flag.
                if via_in_pad_conflict(pad_x, pad_y, v_size, v_drill,
                                       route.net_id) is not None:
                    via_blocked_routes.append(route)
                    continue
                # #620: the pair test the input-board guards above cannot do.
                # Placed AFTER them so the board keeps its present precedence
                # -- this only decides among sites THIS call is creating.
                _bulges = status == 'floor'
                _v, _detail = _pending.verdict(pad_x, pad_y, v_size, v_drill,
                                               route.net_id, _bulges)
                if _v == 'twin':
                    # Two routes, one physical hole. Both keep their tracks and
                    # both anchor to the via already committed here: the ball-
                    # anchor test downstream (`_has_copper`) looks for a via
                    # inside the pad, and this is that via. Today's code
                    # appends a second identical dict and the writer, which
                    # does not dedupe, emits both as stacked copper.
                    _twin_shared += 1
                    continue
                if _v == 'conflict':
                    # A refusal here drops the escape -- there is no re-sweep
                    # -- so descend the fab ladder's drill floors first. The
                    # twin branch above cannot re-fire for a thinner drill, so
                    # 'clear' is the right predicate.
                    _thin = thin_drill_to_clear(
                        v_drill, floors, rung,
                        lambda cand: _pending.verdict(
                            pad_x, pad_y, v_size, cand, route.net_id,
                            _bulges)[0] == 'clear')
                    if _thin is None:
                        via_blocked_routes.append(route)
                        _pending_refused.append(
                            (route.pad.net_name or f"net{route.net_id}",
                             _detail[0]))
                        continue
                    _thinned.append((v_drill, _thin))
                    v_drill = _thin
                if not would_overlap_existing_via(pad_x, pad_y, v_size):
                    vias_to_add.append({
                        'x': pad_x,
                        'y': pad_y,
                        'size': v_size,
                        'drill': v_drill,
                        'layers': ['F.Cu', 'B.Cu'],
                        'net_id': route.net_id
                    })
                    _pending.add(pad_x, pad_y, v_size, v_drill, route.net_id,
                                 _bulges)

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
    # #620 disclosure. Separate from the #253/#370 line below because these are
    # this run's own vias meeting each other, which the reader cannot act on in
    # the same way: the levers are the pitch, the drill and the fab tier.
    if _twin_shared:
        print(f"  #620: {_twin_shared} coincident same-net site(s) share one "
              f"via instead of stacking two")
    if _thinned:
        warn_fab_escalation(
            f"{len(_thinned)} via-in-pad drill(s) thinned to "
            f"{min(t[1] for t in _thinned):g}mm to hold the {_h2h:g}mm "
            f"hole-to-hole floor ({_h2h_src}) against this run's own vias")
    if _pending_refused:
        _names = sorted({n for n, _r in _pending_refused})
        _reasons = sorted({r for _n, r in _pending_refused})
        print(f"  WARNING: {len(_pending_refused)} escape(s) dropped: this "
              f"run's own via-in-pads cannot be spaced at the {_h2h:g}mm "
              f"hole-to-hole floor ({_h2h_src}) on this pitch, and no fab rung "
              f"is thin enough ({'; '.join(_reasons)}); retry with "
              f"--escape-method underpad, a smaller --via-drill, or a fab tier "
              f"whose floor this pitch can meet: {', '.join(_names)}")
    if vias_to_remove:
        print(f"  Removing {len(vias_to_remove)} unnecessary vias at pads on top layer")
    if via_blocked_routes:
        names = sorted(r.pad.net_name or f"net{r.net_id}" for r in via_blocked_routes)
        print(f"  WARNING: {len(via_blocked_routes)} escape(s) dropped: via-in-pad "
              f"would hit an immovable foreign pad (locked part/connector/test "
              f"point, #253), a drill within hole-to-hole of another hole, or a "
              f"foreign track (#370): {', '.join(names)}")

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



def _strap_unescaped_extras(footprint: Footprint, pcb_data: PCBData,
                            balls: List, tracks: List[Dict],
                            vias_to_add: List[Dict],
                            track_width: float, clearance: float,
                            via_size: float, via_drill: float,
                            grid_step: float) -> Tuple[int, List[str]]:
    """Issue #129: quick intra-BGA A* for EXTRA same-net balls whose own escape
    failed. Routes each ball on its ball layer to the nearest point of its
    net's fanned copper (the primary escape or an already-strapped sibling),
    free to run inside the BGA area -- the same assumption as routing later
    with --no-bga-zones, just done now while the interior is uncongested.
    A ball that still can't connect is left bare for the main router and
    reported. Returns (strapped_count, still_bare_labels)."""
    if not balls:
        return 0, []
    from route_planes import route_multi_source_to_pad
    from plane_obstacle_builder import build_routing_obstacle_map
    from kicad_parser import Segment, Via

    n_seg0, n_via0 = len(pcb_data.segments), len(pcb_data.vias)
    strapped = 0
    still_bare: List[str] = []
    # Short local routes: a finer grid threads the tight inter-ball corridors.
    strap_grid = min(grid_step if grid_step and grid_step > 0 else 0.1, 0.05)
    # ONLY-BGA zone: a strap must never leave the ball field -- outside it,
    # strap copper occupies the escape corridors and can block the escape
    # stubs themselves. Blocking an impassable ring just outside the field
    # makes everything beyond it unreachable (equivalent to blocking all
    # outside cells, at a fraction of the stamping cost). 3 cells thick so a
    # diagonal step cannot hop it.
    grid = analyze_bga_grid(footprint)

    def _block_outside_field(obs):
        if grid is None:
            return
        # Quarter-pitch margin past the outermost BALL CENTERS: enough for a
        # strap to hug the outer row/column, but keeps the outer half-channel
        # (where the escape runs exit the array) strictly off-limits.
        inv = 1.0 / strap_grid
        g0x = int(math.floor((grid.min_x - grid.pitch_x / 4) * inv))
        g1x = int(math.ceil((grid.max_x + grid.pitch_x / 4) * inv))
        g0y = int(math.floor((grid.min_y - grid.pitch_y / 4) * inv))
        g1y = int(math.ceil((grid.max_y + grid.pitch_y / 4) * inv))
        t = 3
        for gx in range(g0x - t, g1x + t + 1):
            for gy in list(range(g0y - t, g0y)) + list(range(g1y + 1, g1y + t + 1)):
                obs.add_blocked_cell(gx, gy, 0)
        for gy in range(g0y, g1y + 1):
            for gx in list(range(g0x - t, g0x)) + list(range(g1x + 1, g1x + t + 1)):
                obs.add_blocked_cell(gx, gy, 0)
    try:
        for t in tracks:
            pcb_data.segments.append(Segment(
                start_x=t['start'][0], start_y=t['start'][1],
                end_x=t['end'][0], end_y=t['end'][1],
                width=t['width'], layer=t['layer'], net_id=t['net_id']))
        for v in vias_to_add:
            pcb_data.vias.append(Via(
                x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                layers=v.get('layers') or ['F.Cu', 'B.Cu'], net_id=v['net_id']))

        by_net: Dict[int, List] = {}
        for b in balls:
            by_net.setdefault(b.net_id, []).append(b)
        for net_id, net_balls in by_net.items():
            net_name = net_balls[0].net_name or f"net{net_id}"
            ball_layer = None
            for l in net_balls[0].layers:
                if l.endswith('.Cu') and not l.startswith('*'):
                    ball_layer = l
                    break
            if ball_layer is None:
                # A through-hole ball ('*.Cu') conducts on every layer; strap
                # on the footprint's own layer.
                ball_layer = footprint.layer if (footprint.layer or '').endswith('.Cu') else 'F.Cu'
            # Anchors: the net's OTHER balls that carry copper (an escape stub
            # starts on its ball; a strap ends on its ball).
            bare_keys = {(b.global_x, b.global_y) for b in net_balls}
            anchors = [
                (p.global_x, p.global_y) for p in footprint.pads
                if p.net_id == net_id
                and (p.global_x, p.global_y) not in bare_keys]
            if not anchors:
                still_bare.extend(f"{net_name} ball {b.pad_number}" for b in net_balls)
                continue
            cfg = GridRouteConfig(
                layers=[ball_layer], track_width=track_width,
                clearance=clearance, via_size=via_size, via_drill=via_drill,
                grid_step=strap_grid)
            from kicad_dru import install_layer_clearances
            install_layer_clearances(cfg, None, None, pcb_data)  # #498
            routing_obs = build_routing_obstacle_map(
                pcb_data, cfg, net_id, ball_layer,
                skip_pad_blocking=False, verbose=False)
            _block_outside_field(routing_obs)
            pending = list(net_balls)
            while pending:
                pending.sort(key=lambda b: min(
                    (b.global_x - ax) ** 2 + (b.global_y - ay) ** 2
                    for ax, ay in anchors))
                ball = pending.pop(0)
                segs, _pos = route_multi_source_to_pad(
                    anchors, ball, ball_layer, net_id, routing_obs, cfg)
                if not segs:
                    still_bare.append(f"{net_name} ball {ball.pad_number}")
                    continue
                # The A* emits one segment per grid cell; merge collinear runs.
                merged = [dict(segs[0])]
                for sg in segs[1:]:
                    pm = merged[-1]
                    if (pm['layer'] == sg['layer'] and pm['width'] == sg['width']
                            and pm['end'] == sg['start']):
                        dx1 = pm['end'][0] - pm['start'][0]
                        dy1 = pm['end'][1] - pm['start'][1]
                        dx2 = sg['end'][0] - sg['start'][0]
                        dy2 = sg['end'][1] - sg['start'][1]
                        if (abs(dx1 * dy2 - dy1 * dx2) < 1e-9
                                and dx1 * dx2 + dy1 * dy2 >= 0):
                            pm['end'] = sg['end']
                            continue
                    merged.append(dict(sg))
                tracks.extend(merged)
                for sg in merged:
                    pcb_data.segments.append(Segment(
                        start_x=sg['start'][0], start_y=sg['start'][1],
                        end_x=sg['end'][0], end_y=sg['end'][1],
                        width=sg['width'], layer=sg['layer'],
                        net_id=sg['net_id']))
                anchors.append((ball.global_x, ball.global_y))
                strapped += 1
    finally:
        # Appended copper was working state for this pass only; the caller
        # owns pcb_data (the GUI passes a live board's data).
        del pcb_data.segments[n_seg0:]
        del pcb_data.vias[n_via0:]
    return strapped, still_bare


def _underpad_shrink_rescue(footprint, pcb_data, grid, layers, up_kw,
                            tracks, vias_to_add, failed_nets):
    """Retry balls the under-pad escape DROPPED, at the FAB FLOOR (#505).

    One rung, the tightest. Walking the whole ladder costs a full escape
    rebuild per rung -- ~200s each on a 285-ball 0.5mm array, and the
    escape-priority machinery already calls this more than once -- while an
    intermediate rung cannot fit where the floor does not.

    A dropped ball is an unrouted net the downstream router usually cannot
    rescue either -- it has no copper to pick up. The nominal width is a
    preference, not a constraint: the fab floor is what the board can actually
    be built to, so shrink toward it rather than shipping a hole. Each rung
    re-routes ONLY the still-failed balls, against the copper already
    committed (appended to pcb_data so the retry's obstacle build and its
    existing-copper checks both see it), so a narrower escape can thread a
    channel the nominal width could not while never grazing what shipped.

    Track, via AND clearance shrink together, because on a fine-pitch array the
    binding constraint is usually the CLEARANCE, not the track: between adjacent
    0.23mm pads on a 0.5mm pitch the gap is 0.27mm and a track needs
    `track + 2*clearance`, so orangecrab U3 does not fit at 0.1/0.1 (0.30), nor
    even at the 0.0762 track floor (0.276) -- only dropping clearance to the
    0.09 floor (0.256) opens it. Shrinking track/via alone rescued nothing
    there. Each rung is a real fab floor, and the caller's `--clearance` is a
    preference the ladder is explicitly allowed to escalate below (the same
    standard->advanced escalation the via clamp and the plane taps already do);
    the routed floor is what the .kicad_pro writeback records, so the board is
    graded at what actually shipped.
    """
    from bga_fanout.underpad import generate_underpad_escape
    from kicad_parser import Segment as _Seg, Via as _Via

    ncu = (len(pcb_data.board_info.copper_layers)
           if pcb_data.board_info.copper_layers else 4)
    tw0 = up_kw['track_width']
    vs0, vd0 = up_kw['via_size'], up_kw['via_drill']
    cl0 = up_kw['clearance']
    # Ladder rungs strictly smaller than what we just tried.
    rungs = []
    for f in fab_floor_ladder(ncu):
        tw = min(tw0, f['track_width'])
        vs, vd = min(vs0, f['via_diameter']), min(vd0, f['via_drill'])
        cl = min(cl0, f['clearance'])
        if (tw, vs, vd, cl) != (tw0, vs0, vd0, cl0) and (tw, vs, vd, cl) not in rungs:
            rungs.append((tw, vs, vd, cl))
    # Only the TIGHTEST rung (the fab floor). Walking the whole ladder costs a
    # full escape rebuild per rung -- on a 285-ball 0.5mm array that is ~200s
    # EACH, and the escape-priority machinery already calls us more than once.
    # An intermediate rung that fits where the floor does not cannot exist
    # (the floor is a subset of every rung's freedom), so the extra passes buy
    # only slightly wider copper on the rescued balls, at multiples of the cost.
    # Sort so the tightest (smallest track/via/clearance) is last, whatever
    # order the ladder yielded.
    rungs.sort(reverse=True)
    rungs = rungs[-1:]
    if not rungs:
        return tracks, vias_to_add, failed_nets

    _cc = up_kw.get('cancel_check')          # #621: rescue-pass head
    for (tw, vs, vd, cl) in rungs:
        if _cc and _cc():
            break
        still = set(failed_nets)
        keys = {(p.global_x, p.global_y) for p in footprint.pads
                if p.net_name in still}
        if not keys:
            break
        _pcb = up_kw.get('progress_callback')
        if _pcb:
            _pcb(0, 0, f"BGA fanout {getattr(footprint, 'reference', '?')}: "
                       f"fab-floor rescue of {len(still)} dropped ball(s)...")
        n_seg0, n_via0 = len(pcb_data.segments), len(pcb_data.vias)
        try:
            for t in tracks:
                pcb_data.segments.append(_Seg(
                    start_x=t['start'][0], start_y=t['start'][1],
                    end_x=t['end'][0], end_y=t['end'][1],
                    width=t['width'], layer=t['layer'], net_id=t['net_id']))
            for v in vias_to_add:
                pcb_data.vias.append(_Via(
                    x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                    layers=v.get('layers') or ['F.Cu', 'B.Cu'],
                    net_id=v['net_id']))
            kw = dict(up_kw)
            kw.update(track_width=tw, via_size=vs, via_drill=vd, clearance=cl,
                      only_pad_keys=keys, verbose=False)
            t2, v2, f2 = generate_underpad_escape(
                footprint, pcb_data, grid, layers, **kw)
        finally:
            del pcb_data.segments[n_seg0:]
            del pcb_data.vias[n_via0:]
        rescued = still - set(f2)
        # Always report the attempt: a silent no-op rung is indistinguishable
        # from "the rescue never ran", which cost a debugging round.
        print(f"  Under-pad shrink rescue @ track {tw:.4f}mm / via {vs:.2f}mm / "
              f"clearance {cl:.4f}mm (nominal {tw0:.4f}/{vs0:.2f}/{cl0:.4f}): "
              f"rescued {len(rescued)} of {len(still)} dropped ball(s)")
        if rescued:
            tracks = tracks + t2
            vias_to_add = vias_to_add + v2
            failed_nets = [n for n in failed_nets if n not in rescued]
            warn_fab_escalation('under-pad escape rescue')
        if not failed_nets:
            break
    return tracks, vias_to_add, failed_nets


def _surface_gap_escape(footprint, pcb_data, tracks, vias_to_add,
                        failed_nets, track_width, clearance, exit_margin):
    """Vialess surface escape for terminally dropped balls (#652 directive 1).

    When the fab-floor via cannot fit the array pitch at all (orangecrab U3:
    0.25mm floor via on 0.5mm pitch busts the half-pitch budget by 3um, so
    every via site AND via-in-pad is illegal), the only legal escape is a
    TRACK threading the pad gaps on the ball's own layer -- how humans route
    outer-row balls. The legal band for the corridor centerline can be a few
    um wide and off-grid (161.393..161.397 for RAM_LDM), so this is exact
    interval arithmetic over obstacle projections, verified segment-by-
    segment against every pad/track/via -- never the router grid.

    Path shape per attempt: 45-degree jog from the ball centre to the
    corridor line one half-pitch over, then straight along the inter-column
    (or inter-row) corridor to the array edge + exit_margin. Emits plain
    tracks; no via. Tried toward each of the four array edges, nearest
    first."""
    import math as _m
    from geometry_utils import (point_to_segment_distance as _p2s,
                                segment_to_segment_distance as _s2s)

    own_pads = list(footprint.pads)
    xs = sorted({round(p.global_x, 4) for p in own_pads})
    ys = sorted({round(p.global_y, 4) for p in own_pads})
    if len(xs) < 2 or len(ys) < 2:
        return tracks, failed_nets

    def _pitch(v):
        steps = [b - a for a, b in zip(v, v[1:]) if b - a > 1e-3]
        return min(steps) if steps else 0.0
    px, py = _pitch(xs), _pitch(ys)
    if px <= 0 or py <= 0:
        return tracks, failed_nets
    minx, maxx, miny, maxy = xs[0], xs[-1], ys[0], ys[-1]
    tw2 = track_width / 2.0
    eps = 1e-6

    def _seg_clear(ax, ay, bx, by, net_id, lay, skip_pad):
        """Exact clearance check of candidate segment AB (width track_width,
        layer lay) against every foreign pad / track / via, committed write
        lists included. Returns True if legal at `clearance`."""
        # Pads: same-layer SMD + all plated/NPTH holes.
        for fp2 in pcb_data.footprints.values():
            for p in fp2.pads:
                if p is skip_pad or p.net_id == net_id and p.net_id != 0:
                    continue
                on_layer = (p.drill > 0) or (lay in (p.layers or ()))
                if not on_layer:
                    continue
                need_half = (max(p.size_x, p.size_y) / 2.0
                             if p.pad_type != 'np_thru_hole'
                             else p.drill / 2.0)
                cl = max(clearance, p.local_clearance or 0.0)
                hx = p.hole_x if p.hole_x is not None else p.global_x
                hy = p.hole_y if p.hole_y is not None else p.global_y
                cx = p.global_x if p.pad_type != 'np_thru_hole' else hx
                cy = p.global_y if p.pad_type != 'np_thru_hole' else hy
                if _p2s(cx, cy, ax, ay, bx, by) < need_half + tw2 + cl - eps:
                    return False
        # Vias: board + this call's write list (through barrels, all layers).
        for v in pcb_data.vias:
            if v.net_id == net_id:
                continue
            if _p2s(v.x, v.y, ax, ay, bx, by) < v.size / 2.0 + tw2 + clearance - eps:
                return False
        for v in vias_to_add:
            if v['net_id'] == net_id:
                continue
            if _p2s(v['x'], v['y'], ax, ay, bx, by) < v['size'] / 2.0 + tw2 + clearance - eps:
                return False
        # Tracks: board + write list, same layer only.
        for s in pcb_data.segments:
            if s.net_id == net_id or s.layer != lay:
                continue
            if _s2s(ax, ay, bx, by, s.start_x, s.start_y,
                    s.end_x, s.end_y) < s.width / 2.0 + tw2 + clearance - eps:
                return False
        for t in tracks:
            if t['net_id'] == net_id or t['layer'] != lay:
                continue
            if _s2s(ax, ay, bx, by, t['start'][0], t['start'][1],
                    t['end'][0], t['end'][1]) < t['width'] / 2.0 + tw2 + clearance - eps:
                return False
        return True

    def _corridor_band(c0, horiz, lo, hi, net_id, lay):
        """Feasible interval for the corridor centerline coordinate around
        c0 (a half-pitch line): project every pad/via within the corridor's
        run [lo, hi] (the along-axis range) onto the cross axis and subtract
        forbidden bands. Tracks are left to the exact verify. Returns the
        centre of the widest surviving sub-interval, or None."""
        half = (px if horiz else py) / 2.0
        lo_b, hi_b = c0 - half * 0.49, c0 + half * 0.49
        bands = []
        def _add(off, need):
            bands.append((off - need, off + need))
        for fp2 in pcb_data.footprints.values():
            for p in fp2.pads:
                if p.net_id == net_id and p.net_id != 0:
                    continue
                on_layer = (p.drill > 0) or (lay in (p.layers or ()))
                if not on_layer:
                    continue
                along = p.global_x if horiz else p.global_y
                cross = p.global_y if horiz else p.global_x
                if not (lo - 0.3 <= along <= hi + 0.3):
                    continue
                if p.pad_type == 'np_thru_hole':
                    _add(cross, p.drill / 2.0 + tw2 + clearance)
                else:
                    _add(cross, max(p.size_x, p.size_y) / 2.0 + tw2
                         + max(clearance, p.local_clearance or 0.0))
        for coll, get in ((pcb_data.vias,
                           lambda v: (v.x, v.y, v.size, v.net_id)),
                          (vias_to_add,
                           lambda v: (v['x'], v['y'], v['size'], v['net_id']))):
            for v in coll:
                vx, vy, vs, vn = get(v)
                if vn == net_id:
                    continue
                along = vx if horiz else vy
                cross = vy if horiz else vx
                if not (lo - 0.3 <= along <= hi + 0.3):
                    continue
                _add(cross, vs / 2.0 + tw2 + clearance)
        # Sweep: collect candidate centre points = midpoints of free gaps
        # between overlapping forbidden bands, clipped to [lo_b, hi_b].
        bands = [(a, b) for (a, b) in bands if b > lo_b and a < hi_b]
        bands.sort()
        free = []
        cur = lo_b
        for a, b in bands:
            if a > cur:
                free.append((cur, min(a, hi_b)))
            cur = max(cur, b)
            if cur >= hi_b:
                break
        if cur < hi_b:
            free.append((cur, hi_b))
        free = [(a, b) for (a, b) in free if b - a > eps]
        if not free:
            return None
        a, b = max(free, key=lambda ab: ab[1] - ab[0])
        return (a + b) / 2.0

    for fnet in list(failed_nets):
        fpads = [p for p in own_pads if p.net_name == fnet]
        done = False
        for p in fpads:
            lay = next((l for l in (p.layers or []) if l.endswith('.Cu')), None)
            if lay is None:
                continue
            bx, by = p.global_x, p.global_y
            nid = p.net_id
            # Four exits, nearest edge first.
            exits = sorted([
                ('S', maxy - by), ('N', by - miny),
                ('E', maxx - bx), ('W', bx - minx)], key=lambda t: t[1])
            for d, _dist in exits:
                horiz = d in ('E', 'W')
                sgn = 1 if d in ('S', 'E') else -1
                pit = px if horiz else py
                cpit = py if horiz else px
                end = ((maxx if d == 'E' else minx) if horiz
                       else (maxy if d == 'S' else miny))
                end += sgn * max(exit_margin, 0.5)
                along0 = (bx if horiz else by) + sgn * pit / 2.0
                for side in (-1, 1):
                    c0 = (by if horiz else bx) + side * cpit / 2.0
                    lo, hi = sorted((along0, end))
                    c = _corridor_band(c0, horiz, lo, hi, nid, lay)
                    import os as _dbg2
                    if _dbg2.environ.get('KICAD_FANOUT_RESCUE_DEBUG'):
                        print(f"  [rescue-debug] {fnet} dir={d} side={side} "
                              f"c0={c0:.3f} run=[{lo:.2f},{hi:.2f}] band={c}")
                    if c is None:
                        continue
                    if horiz:
                        j = (along0, c)
                        e = (end, c)
                    else:
                        j = (c, along0)
                        e = (c, end)
                    _ok1 = _seg_clear(bx, by, j[0], j[1], nid, lay, p)
                    _ok2 = _ok1 and _seg_clear(j[0], j[1], e[0], e[1], nid, lay, p)
                    if _dbg2.environ.get('KICAD_FANOUT_RESCUE_DEBUG'):
                        print(f"  [rescue-debug]   diag_ok={_ok1} corridor_ok={_ok2}")
                    if not _ok2:
                        continue
                    tracks = tracks + [
                        {'start': (bx, by), 'end': j, 'width': track_width,
                         'layer': lay, 'net_id': nid},
                        {'start': j, 'end': e, 'width': track_width,
                         'layer': lay, 'net_id': nid}]
                    failed_nets = [n for n in failed_nets if n != fnet]
                    print(f"  Surface rescue (#652): {fnet} escaped {d} with "
                          f"a vialess pad-gap track (corridor "
                          f"{'y' if horiz else 'x'}={c:.4f}, band found by "
                          f"exact geometry)")
                    done = True
                    break
                if done:
                    break
            if done:
                break
    return tracks, failed_nets


def _underpad_rip_rescue(footprint, pcb_data, grid, layers, up_kw,
                         tracks, vias_to_add, failed_nets, max_victims=4):
    """Rip-swap rescue (#652 directive 2). A terminally dropped ball at the
    fab floor is usually boxed in by a NEIGHBOUR's committed escape via, not
    by fixed board copper (orangecrab U3 RAM_LDM: the 0.25mm via floor on a
    0.5mm pitch leaves ~one legal gap site per neighbourhood, and whichever
    ball claims it first strands the next). For each dropped ball, evict the
    nearest same-component escaped neighbours one at a time and re-run the
    under-pad escape for the PAIR (failed ball + victim) at the fab floor
    against everything else -- the pair search can trade sites in a way two
    independent single-ball runs never see. Accept only a strict win (both
    escape); otherwise the eviction is reverted, so the swap never ships
    fewer escapes than it started with. Diff-pair halves are never victims
    (a pair re-run would fall back single-ended)."""
    from bga_fanout.underpad import generate_underpad_escape
    from kicad_parser import Segment as _Seg, Via as _Via

    ncu = (len(pcb_data.board_info.copper_layers)
           if pcb_data.board_info.copper_layers else 4)
    # Pair re-runs go straight to the tightest rung (same reasoning as the
    # shrink rescue: an intermediate rung cannot fit where the floor does not).
    floor = None
    for f in fab_floor_ladder(ncu):
        floor = f
    tw = min(up_kw['track_width'], floor['track_width']) if floor else up_kw['track_width']
    vs = min(up_kw['via_size'], floor['via_diameter']) if floor else up_kw['via_size']
    vd = min(up_kw['via_drill'], floor['via_drill']) if floor else up_kw['via_drill']
    cl = min(up_kw['clearance'], floor['clearance']) if floor else up_kw['clearance']

    dp_nets = set()
    for _pr in (up_kw.get('diff_pairs') or {}).values():
        for _pp in (getattr(_pr, 'p_pad', None), getattr(_pr, 'n_pad', None)):
            if _pp is not None and _pp.net_name:
                dp_nets.add(_pp.net_name)

    pads_by_net = {}
    for p in footprint.pads:
        if p.net_name:
            pads_by_net.setdefault(p.net_name, []).append(p)
    nid2name = {p.net_id: p.net_name for p in footprint.pads if p.net_name}
    escaped_via_nets = {}   # net_name -> [(x, y)] of its committed escape vias
    for v in vias_to_add:
        nm = nid2name.get(v['net_id'])
        if nm:
            escaped_via_nets.setdefault(nm, []).append((v['x'], v['y']))

    from geometry_utils import point_to_segment_distance as _p2s_r

    for fnet in list(failed_nets):
        fpads = pads_by_net.get(fnet, [])
        if not fpads:
            continue
        fx = sum(p.global_x for p in fpads) / len(fpads)
        fy = sum(p.global_y for p in fpads) / len(fpads)
        # Rank victims by their COPPER's distance to the failed ball --
        # tracks included, not just vias: the binding blocker can be a
        # neighbour's escape track snaking through the ball's only surface
        # corridor while its via sits far away (orangecrab U3: USER_BUTTON's
        # stub passes 0.09mm from RAM_LDM's corridor; its via is >0.8mm out).
        cand_d = {}
        for nm, vlist in escaped_via_nets.items():
            if nm == fnet or nm in dp_nets or nm in failed_nets:
                continue
            if not vlist:
                # A net can be enrolled with an EMPTY via list (a vialess
                # surface escape, or a drop recorded before its via
                # landed) -- min() over it crashed the whole fanout on
                # daisho + orangecrab in the set3 screen.
                continue
            d = min(math.hypot(vx - fx, vy - fy) for (vx, vy) in vlist)
            cand_d[nm] = min(cand_d.get(nm, 9e9), d)
        for t in tracks:
            nm = nid2name.get(t['net_id'])
            if not nm or nm == fnet or nm in dp_nets or nm in failed_nets:
                continue
            d = _p2s_r(fx, fy, t['start'][0], t['start'][1],
                       t['end'][0], t['end'][1])
            if d < 1.5:
                cand_d[nm] = min(cand_d.get(nm, 9e9), d)
        cands = sorted((d, nm) for nm, d in cand_d.items())
        rescued = False
        for d, victim in cands[:max_victims]:
            vic_nids = {p.net_id for p in pads_by_net.get(victim, ())}
            t2base = [t for t in tracks if nid2name.get(t['net_id']) != victim]
            v2base = [v for v in vias_to_add if nid2name.get(v['net_id']) != victim]
            keys = ({(p.global_x, p.global_y) for p in fpads}
                    | {(p.global_x, p.global_y)
                       for p in pads_by_net.get(victim, ())})
            n_seg0, n_via0 = len(pcb_data.segments), len(pcb_data.vias)
            try:
                for t in t2base:
                    pcb_data.segments.append(_Seg(
                        start_x=t['start'][0], start_y=t['start'][1],
                        end_x=t['end'][0], end_y=t['end'][1],
                        width=t['width'], layer=t['layer'], net_id=t['net_id']))
                for v in v2base:
                    pcb_data.vias.append(_Via(
                        x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                        layers=v.get('layers') or ['F.Cu', 'B.Cu'],
                        net_id=v['net_id']))
                kw = dict(up_kw)
                kw.update(track_width=tw, via_size=vs, via_drill=vd,
                          clearance=cl, only_pad_keys=keys, verbose=False)
                t2, v2, f2 = generate_underpad_escape(
                    footprint, pcb_data, grid, layers, **kw)
            finally:
                del pcb_data.segments[n_seg0:]
                del pcb_data.vias[n_via0:]
            if fnet not in f2 and victim not in f2:
                tracks = t2base + t2
                vias_to_add = v2base + v2
                failed_nets = [n for n in failed_nets if n != fnet]
                escaped_via_nets[victim] = [
                    (v['x'], v['y']) for v in v2 if v['net_id'] in vic_nids]
                escaped_via_nets[fnet] = [
                    (v['x'], v['y']) for v in v2
                    if nid2name.get(v['net_id']) == fnet]
                print(f"  Rip-swap rescue (#652): evicted {victim} "
                      f"(copper {d:.2f}mm away), re-escaped BOTH it and "
                      f"{fnet} at the fab floor")
                warn_fab_escalation('rip-swap escape rescue')
                rescued = True
                break
            # Attempt B: the via world may be closed regardless (the fab-
            # floor via can bust the half-pitch budget outright) -- with
            # this victim evicted, try the vialess SURFACE walk for the
            # failed ball, then re-escape the victim through the engine
            # against the surface track.
            if not env_knobs.FANOUT_SURFACE_RESCUE:
                continue
            t_surf, f_surf = _surface_gap_escape(
                footprint, pcb_data, t2base, v2base, [fnet],
                tw, cl, up_kw.get('exit_margin', 0.5))
            if fnet in f_surf:
                continue
            surf_new = t_surf[len(t2base):]
            n_seg0, n_via0 = len(pcb_data.segments), len(pcb_data.vias)
            try:
                for t in t_surf:
                    pcb_data.segments.append(_Seg(
                        start_x=t['start'][0], start_y=t['start'][1],
                        end_x=t['end'][0], end_y=t['end'][1],
                        width=t['width'], layer=t['layer'], net_id=t['net_id']))
                for v in v2base:
                    pcb_data.vias.append(_Via(
                        x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                        layers=v.get('layers') or ['F.Cu', 'B.Cu'],
                        net_id=v['net_id']))
                kw = dict(up_kw)
                kw.update(track_width=tw, via_size=vs, via_drill=vd,
                          clearance=cl, verbose=False,
                          only_pad_keys={(p.global_x, p.global_y)
                                         for p in pads_by_net.get(victim, ())})
                t3, v3, f3 = generate_underpad_escape(
                    footprint, pcb_data, grid, layers, **kw)
            finally:
                del pcb_data.segments[n_seg0:]
                del pcb_data.vias[n_via0:]
            if victim not in f3:
                tracks = t_surf + t3
                vias_to_add = v2base + v3
                failed_nets = [n for n in failed_nets if n != fnet]
                escaped_via_nets[victim] = [
                    (v['x'], v['y']) for v in v3 if v['net_id'] in vic_nids]
                print(f"  Rip-swap rescue (#652): evicted {victim} "
                      f"(copper {d:.2f}mm away); {fnet} escaped by VIALESS "
                      f"surface walk, {victim} re-escaped around it")
                warn_fab_escalation('rip-swap escape rescue')
                rescued = True
                break
            # The victim may ALSO be surface-escapable (the classic shape:
            # a last-row ball whose original escape was a long surface
            # walkabout through the failed ball's corridor -- orangecrab
            # USER_BUTTON J17). Single-ball victims only: the walk serves
            # one ball per net.
            if len(pads_by_net.get(victim, ())) == 1:
                t_surf2, f_surf2 = _surface_gap_escape(
                    footprint, pcb_data, t_surf, v2base, [victim],
                    tw, cl, up_kw.get('exit_margin', 0.5))
                if victim not in f_surf2:
                    tracks = t_surf2
                    vias_to_add = v2base
                    failed_nets = [n for n in failed_nets if n != fnet]
                    escaped_via_nets.pop(victim, None)
                    print(f"  Rip-swap rescue (#652): evicted {victim} "
                          f"(copper {d:.2f}mm away); BOTH it and {fnet} "
                          f"re-escaped by vialess surface walks")
                    warn_fab_escalation('rip-swap escape rescue')
                    rescued = True
                    break
        if not rescued and cands:
            print(f"  Rip-swap rescue (#652): {fnet} still dropped after "
                  f"trying {min(len(cands), max_victims)} eviction(s)")
    return tracks, vias_to_add, failed_nets


def _generate_bga_fanout_core(footprint: Footprint,
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
                        layer_costs: Optional[List[float]] = None,
                        _pad_filter: Optional[Set[Tuple[float, float]]] = None,
                        _ignore_prefanned: bool = False,
                        _single_pass: bool = False,
                        # #581: > 0 forbids via-in-pad (underpad escapes run
                        # dog-bone); None auto-reads the .kicad_pro record.
                        same_net_pad_clearance: Optional[float] = None,
                        # progress_callback(current, total, label): forwarded
                        # into every recursion (rotated frame, #129 passes,
                        # the underpad retry) and the underpad engine's
                        # per-ball loops, so the GUI status line moves during
                        # the minutes a large array takes. None = silent.
                        progress_callback=None,
                        cancel_check=None) -> Tuple[List[Dict], List[Dict], List[Dict]]:
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
            escapes more (issue #288). KICAD_FANOUT_AUTO_DOGBONE=1 opts the
            retry into a dogbone-first ladder instead -- rejected as the
            default by the #669 sets1-5 corpus A/B (+10 incomplete, +59 DRC).
            'channel' is the 45-stub + channel router (with
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
    # #621 escape-pass head. EVERY escape pass -- the rotated-frame recursion,
    # both escape-priority passes, the single-pass coverage probe and the
    # under-pad auto-fallback -- is a call to THIS function, so one check here
    # is the head of all of them. It returns an empty result rather than
    # raising: an exception here would be eaten by the `except Exception`
    # swallowers this package is full of -- which is the reason the cancel
    # contract is cooperative. Nothing is added to
    # failed_nets: no ball was tried, so nothing failed.
    if cancel_check and cancel_check():
        return [], [], [], []

    if layers is None:
        layers = ["F.Cu", "B.Cu"]

    _ref_prog = getattr(footprint, 'reference', '?')

    def _prog(what):
        if progress_callback:
            progress_callback(0, 0, f"BGA fanout {_ref_prog}: {what}")

    # #581: an active (> 0) same-net pad via clearance forbids via-in-pad, so
    # an under-pad escape runs in DOG-BONE mode (#128: via in the diagonal
    # inter-ball gap) instead. Explicit param wins; None auto-reads the record
    # a chain step persisted into the sibling .kicad_pro. The 'auto' fallback
    # re-enters this function with escape_method='underpad' and the resolved
    # value forwarded, so it is remapped there too. The channel engine places
    # no via-in-pad and is untouched.
    if same_net_pad_clearance is None:
        from protected_nets import read_snpc_for_pcb_data as _read_snpc581
        same_net_pad_clearance = _read_snpc581(pcb_data)
    # #621 head-of-all-escape-work. Every path into this engine -- the
    # rotated-frame recursion, both escape-priority passes, the single-pass
    # probe, the under-pad auto-fallback -- is a call to THIS function, so one
    # check here is the head of all of them. It returns an empty result rather
    # than raising: an exception would be eaten by the `except Exception`
    # swallowers this package is full of -- which is the reason the cancel
    # contract is cooperative. Nothing is added to failed_nets: no ball was
    # tried, so nothing failed.
    if cancel_check and cancel_check():
        return [], [], [], []

    if same_net_pad_clearance > 0 and escape_method == 'underpad':
        print(f"  Same-net pad via clearance {same_net_pad_clearance:g}mm "
              f"(#581): under-pad escape runs dog-bone (no via-in-pad)")
        escape_method = 'dogbone'

    # Non-orthogonally-placed parts (issue #137): the grid/escape logic below is
    # global-axis-bound, so rotate the whole board into this footprint's frame
    # (where its balls are axis-aligned), run the pipeline, and map the resulting
    # tracks/vias back. Orthogonal placements skip this and are unaffected.
    # #498: fanout copper must obey the board's per-layer .kicad_dru clearance
    # rules. The channel/under-pad engines are scalar-clearance throughout, so
    # compose CONSERVATIVELY: floor the scalar at the largest rule on any layer
    # this fanout may use (tighten only -- taking a relaxing rule from one
    # layer would under-space the others). The obstacle maps below get the
    # exact per-layer map; exact per-layer self-spacing inside the escape
    # engines is future work. Placed BEFORE the rotated-frame recursion so the
    # rotated run inherits the floored value (idempotent: re-floor is a no-op).
    from kicad_dru import board_layer_clearance_map
    _lcl_498 = board_layer_clearance_map(pcb_data)
    _mx_498 = max((v for l, v in _lcl_498.items() if l in (layers or [])),
                  default=None)
    if _mx_498 is not None and _mx_498 > clearance:
        print(f"  .kicad_dru: fanout clearance floored {clearance} -> {_mx_498} "
              f"(largest per-layer rule on the escape layers, #498)")
        clearance = _mx_498

    from bga_fanout.rotate_frame import (is_orthogonal, to_axis_aligned_frame,
                                         back_transform_results)
    if not is_orthogonal(footprint.rotation):
        print(f"  {footprint.reference} placed at {footprint.rotation:.1f}° - routing "
              f"in the footprint frame and mapping back (issue #137)")
        rp, back = to_axis_aligned_frame(pcb_data, footprint.reference)
        tracks, vias_to_add, vias_to_remove, failed_nets = _generate_bga_fanout_core(
            rp.footprints[footprint.reference], rp,
            net_filter=net_filter, diff_pair_patterns=diff_pair_patterns, layers=layers,
            track_width=track_width, clearance=clearance, diff_pair_gap=diff_pair_gap,
            exit_margin=exit_margin, primary_escape=primary_escape,
            force_escape_direction=force_escape_direction, rebalance_escape=rebalance_escape,
            via_size=via_size, via_drill=via_drill, check_for_previous=check_for_previous,
            no_inner_top_layer=no_inner_top_layer, escape_method=escape_method,
            grid_step=grid_step, layer_costs=layer_costs,
            same_net_pad_clearance=same_net_pad_clearance,
            cancel_check=cancel_check,
            progress_callback=progress_callback)
        back_transform_results(tracks, vias_to_add, vias_to_remove, back)
        return tracks, vias_to_add, vias_to_remove, failed_nets

    # #472 direct-route deferral (KICAD_FANOUT_DIRECT=1): balls whose nearest
    # target is surface-reachable get NO stub -- the stub carpet is itself the
    # wall that seals the pocket (human ottercast: USB_D pure F.Cu, 0 vias).
    # Applied ONCE at entry as '!' net-filter exclusions so both engines and
    # both escape-priority passes inherit; qualifying diff pairs are dropped
    # at the pair-discovery sites via _direct_route_nets. The route steps'
    # bare-ball zone exemption (setup_bga_exclusion_zones) keeps the deferred
    # balls routable.
    global _direct_route_nets
    if (_pad_filter is None and not _single_pass
            and env_knobs.FANOUT_DIRECT):
        from bga_fanout.escape import direct_route_candidates
        _dp_probe = (find_differential_pairs(footprint, diff_pair_patterns)
                     if diff_pair_patterns else None)
        _names, _notes = direct_route_candidates(
            pcb_data, footprint, net_filter=net_filter,
            diff_pairs=_dp_probe, clearance=clearance)
        if _names:
            print(f"  #472 direct-route deferral: {len(_names)} net(s) skip "
                  f"fanout (surface-reachable):")
            for _nm, _pn, _why in _notes:
                print(f"    {_nm} (ball {_pn}): {_why}")
            net_filter = list(net_filter or ['*']) + ['!' + n for n in _names]
            _direct_route_nets = set(_names)
    elif _pad_filter is None and not _single_pass:
        _direct_route_nets = set()

    # Escape priority for multi-ball nets (issue #129). Escape channels are
    # the scarce resource on a dense array (#122), and a net only NEEDS one
    # escape -- later routing can pick up its other balls inside the BGA
    # (--no-bga-zones). So instead of letting every ball compete at once:
    #   pass 1: fan ONE ball per net (the first in pad order) plus all
    #           single-ball nets and diff pairs -- net coverage first;
    #   pass 2: fan the remaining EXTRA balls with pass-1 copper committed --
    #           they take whatever escape room is left. An extra that fails
    #           is a SOFT failure (its net escaped); an extra that succeeds
    #           for a net whose pass-1 ball failed RESCUES the net.
    #   pass 3: extras that could not escape get a quick intra-BGA A* strap
    #           to their net's fanned copper; still-bare balls are left for
    #           the main router and reported.
    # Boards without multi-ball nets take a single pass, byte-identical to
    # the old behavior.
    if _pad_filter is None and not _single_pass:
        _nc_ids = single_pad_net_ids(footprint, pcb_data)
        _pair_nets: Set[str] = set()
        if diff_pair_patterns:
            for _pr in find_differential_pairs(footprint, diff_pair_patterns).values():
                if _pr.p_pad and _pr.p_pad.net_name:
                    _pair_nets.add(_pr.p_pad.net_name)
                if _pr.n_pad and _pr.n_pad.net_name:
                    _pair_nets.add(_pr.n_pad.net_name)
        # Nets whose balls already carry copper (a prior fanout / partial
        # route) are the existing check_for_previous machinery's business.
        _ball_pts: Dict[int, List[Tuple[float, float]]] = {}
        for _p in footprint.pads:
            if _p.net_id:
                _ball_pts.setdefault(_p.net_id, []).append((_p.global_x, _p.global_y))
        _prefanned: Set[int] = set()
        for _s in pcb_data.segments:
            for (_px, _py) in _ball_pts.get(_s.net_id, ()):
                if ((abs(_s.start_x - _px) < 0.05 and abs(_s.start_y - _py) < 0.05)
                        or (abs(_s.end_x - _px) < 0.05 and abs(_s.end_y - _py) < 0.05)):
                    _prefanned.add(_s.net_id)
                    break
        _seen_nets: Set[int] = set()
        _extras: List = []
        for _p in footprint.pads:
            if not _p.net_name or _p.net_id == 0:
                continue
            if _p.net_name.lower().startswith('unconnected-'):
                continue
            if _p.net_id in _nc_ids or _p.net_id in _prefanned:
                continue
            if _p.net_name in _pair_nets:
                continue
            if net_filter and not matches_net_filter(_p.net_name, net_filter):
                continue
            if _p.net_id in _seen_nets:
                _extras.append(_p)
            else:
                _seen_nets.add(_p.net_id)
        if _extras:
            _kw = dict(
                net_filter=net_filter, diff_pair_patterns=diff_pair_patterns,
                layers=layers, track_width=track_width, clearance=clearance,
                diff_pair_gap=diff_pair_gap, exit_margin=exit_margin,
                primary_escape=primary_escape,
                force_escape_direction=force_escape_direction,
                rebalance_escape=rebalance_escape, via_size=via_size,
                via_drill=via_drill, no_inner_top_layer=no_inner_top_layer,
                escape_method=escape_method, grid_step=grid_step,
                layer_costs=layer_costs,
                same_net_pad_clearance=same_net_pad_clearance,
                cancel_check=cancel_check,
                progress_callback=progress_callback)
            # Coverage gate (issue #367): the legacy single pass runs FIRST.
            # When it escapes every ball there is nothing for prioritization
            # to improve -- reshuffling the escape competition only butterflies
            # the downstream chain (ottercast_audio: 4 needlessly strapped
            # balls cascaded into +10 disconnected nets). Escape priority is
            # strictly a RESCUE for boards where the single pass drops balls.
            t0, v0, vr0, f0 = _generate_bga_fanout_core(
                footprint, pcb_data, check_for_previous=check_for_previous,
                _single_pass=True, **_kw)
            if not f0:
                return t0, v0, vr0, f0
            _n_nets = len({_p.net_id for _p in _extras})
            print(f"  Escape priority (issue #129): single pass dropped "
                  f"{len(f0)} ball(s) and {_n_nets} multi-ball net(s) have "
                  f"{len(_extras)} extra ball(s) -- retrying with one escape "
                  f"per net first, extra escapes second")
            _prog("escape priority pass 1 (one ball per net)...")
            # The fab-floor track clamp (issue #223) lives below this block;
            # the strap helper and cross-pass guard must use the CLAMPED width
            # (each recursive pass clamps its own escape copper internally).
            from list_nets import fab_floors as _ff
            _ncu = (len(pcb_data.board_info.copper_layers)
                    if pcb_data.board_info.copper_layers else 2)
            _tw = max(track_width, _ff(_ncu)['track_width'])
            _all_keys = {(_p.global_x, _p.global_y) for _p in footprint.pads}
            _extra_keys = {(_p.global_x, _p.global_y) for _p in _extras}
            _prog("escape priority pass 1 (one ball per net)...")
            tracks, vias_to_add, vias_to_remove, failed_nets = _generate_bga_fanout_core(
                footprint, pcb_data, check_for_previous=check_for_previous,
                _pad_filter=_all_keys - _extra_keys, **_kw)
            # Pass 2: extras route against pass-1 copper (check_for_previous
            # turns on existing-track collision checks; filter membership
            # overrides its net-level "already fanned" skip).
            _n_seg0, _n_via0 = len(pcb_data.segments), len(pcb_data.vias)
            try:
                from kicad_parser import Segment as _Seg, Via as _Via
                for _t in tracks:
                    pcb_data.segments.append(_Seg(
                        start_x=_t['start'][0], start_y=_t['start'][1],
                        end_x=_t['end'][0], end_y=_t['end'][1],
                        width=_t['width'], layer=_t['layer'], net_id=_t['net_id']))
                for _v in vias_to_add:
                    pcb_data.vias.append(_Via(
                        x=_v['x'], y=_v['y'], size=_v['size'], drill=_v['drill'],
                        layers=_v.get('layers') or ['F.Cu', 'B.Cu'],
                        net_id=_v['net_id']))
                _prog(f"escape priority pass 2 ({len(_extras)} extra ball(s))...")
                t2, v2, vr2, f2 = _generate_bga_fanout_core(
                    footprint, pcb_data, check_for_previous=True,
                    _pad_filter=_extra_keys, _ignore_prefanned=True, **_kw)
            finally:
                del pcb_data.segments[_n_seg0:]
                del pcb_data.vias[_n_via0:]
            # Cross-pass DRC guard: the channel engine's collision primitive
            # misses diagonal-vs-straight grazes, its post-validation passes
            # recheck only foreign PADS, and the underpad engine's via-in-pad
            # placement can land on pass-1 copper -- so a pass-2 escape can
            # ship sub-clearance to pass-1 copper. Extras are best-effort:
            # drop the offending BALL's connected copper (its chain of tracks
            # + via); the ball joins the strap pool below. Grazes within the
            # sub-grid rounding tolerance (#70) are left alone.
            from geometry_utils import (segment_to_segment_distance,
                                        point_to_segment_distance)
            _guard_tol = 0.02

            def _k(pt):
                return (round(pt[0], 3), round(pt[1], 3))

            # Connected components of pass-2 copper (endpoint adjacency,
            # per net): the drop unit for a graze.
            _parent = {}

            def _find(a):
                while _parent[a] != a:
                    _parent[a] = _parent[_parent[a]]
                    a = _parent[a]
                return a

            def _union(a, b):
                _parent.setdefault(a, a)
                _parent.setdefault(b, b)
                ra, rb = _find(a), _find(b)
                if ra != rb:
                    _parent[ra] = rb

            for _i, _t in enumerate(t2):
                a = (_t['net_id'],) + _k(_t['start'])
                b = (_t['net_id'],) + _k(_t['end'])
                _parent.setdefault(a, a)
                _parent.setdefault(b, b)
                _union(a, b)
            _bad_comps = set()
            for _t in t2:
                node = (_t['net_id'],) + _k(_t['start'])
                hit = False
                for _o in tracks:
                    if _o['layer'] == _t['layer'] and _o['net_id'] != _t['net_id']:
                        d = segment_to_segment_distance(
                            _t['start'][0], _t['start'][1], _t['end'][0], _t['end'][1],
                            _o['start'][0], _o['start'][1], _o['end'][0], _o['end'][1])
                        if d < _t['width'] / 2 + _o['width'] / 2 + clearance - _guard_tol:
                            hit = True
                            break
                if not hit:
                    for _ov in vias_to_add:
                        if _ov['net_id'] != _t['net_id']:
                            d = point_to_segment_distance(
                                _ov['x'], _ov['y'], _t['start'][0], _t['start'][1],
                                _t['end'][0], _t['end'][1])
                            if d < _ov['size'] / 2 + _t['width'] / 2 + clearance - _guard_tol:
                                hit = True
                                break
                if hit:
                    _bad_comps.add(_find(node))
            for _v in v2:
                node = (_v['net_id'],) + _k((_v['x'], _v['y']))
                # A bare via-in-pad (no adjacent pass-2 track) is its own
                # drop unit.
                _parent.setdefault(node, node)
                for _o in tracks:
                    if _o['net_id'] == _v['net_id']:
                        continue
                    d = point_to_segment_distance(
                        _v['x'], _v['y'], _o['start'][0], _o['start'][1],
                        _o['end'][0], _o['end'][1])
                    if d < _v['size'] / 2 + _o['width'] / 2 + clearance - _guard_tol:
                        _bad_comps.add(_find(node))
                        break
            if _bad_comps:
                def _t_bad(_t):
                    return _find((_t['net_id'],) + _k(_t['start'])) in _bad_comps

                def _v_bad(_v):
                    node = (_v['net_id'],) + _k((_v['x'], _v['y']))
                    return node in _parent and _find(node) in _bad_comps
                _dropped_balls = sorted({
                    f"{_p.net_name} ball {_p.pad_number}" for _p in _extras
                    if ((_p.net_id,) + _k((_p.global_x, _p.global_y))) in _parent
                    and _find((_p.net_id,) + _k((_p.global_x, _p.global_y))) in _bad_comps})
                print(f"  Escape priority: dropped {len(_bad_comps)} extra "
                      f"escape chain(s) grazing pass-1 copper (balls go to "
                      f"the strap pool): {', '.join(_dropped_balls)}")
                t2 = [_t for _t in t2 if not _t_bad(_t)]
                v2 = [_v for _v in v2 if not _v_bad(_v)]
            _rescued = {_p.net_name for _p in _extras
                        if _p.net_name in failed_nets and _p.net_name not in f2
                        and any(_t['net_id'] == _p.net_id for _t in t2)}
            if _rescued:
                print(f"  Escape priority: extra-ball escape rescued "
                      f"{len(_rescued)} net(s): {', '.join(sorted(_rescued))}")
            tracks.extend(t2)
            vias_to_add.extend(v2)
            vias_to_remove.extend(vr2)
            failed_nets = [n for n in failed_nets if n not in _rescued]
            # Extras whose ball still carries no copper: soft failures if the
            # net escaped -- strap them to the net's copper (pass 3); a net
            # with NO escape at all stays in failed_nets.
            _escaped_names = {n for n in ({_p.net_name for _p in _extras}
                                          | set())
                              if n not in failed_nets}
            def _has_copper(_p):
                # Layer-aware: an SMD ball is connected by a net VIA inside its
                # pad (spans all layers) or a net track endpoint ON ITS OWN
                # layer -- copper crossing the pad on an inner layer is not a
                # connection. Drilled / '*.Cu' balls conduct on every layer.
                tol = max(_p.size_x, _p.size_y) / 2 + 0.01
                if any(_v['net_id'] == _p.net_id
                       and abs(_v['x'] - _p.global_x) < tol
                       and abs(_v['y'] - _p.global_y) < tol
                       for _v in vias_to_add):
                    return True
                _pl = None
                for _l in (_p.layers or []):
                    if _l.endswith('.Cu') and not _l.startswith('*'):
                        _pl = _l
                        break
                any_layer = _pl is None or (_p.drill or 0) > 0
                return any(
                    _t['net_id'] == _p.net_id
                    and (any_layer or _t['layer'] == _pl) and (
                        (abs(_t['start'][0] - _p.global_x) < tol
                         and abs(_t['start'][1] - _p.global_y) < tol)
                        or (abs(_t['end'][0] - _p.global_x) < tol
                            and abs(_t['end'][1] - _p.global_y) < tol))
                    for _t in tracks)
            _bare = [_p for _p in _extras
                     if _p.net_name in _escaped_names and not _has_copper(_p)]
            if _bare:
                _prog(f"strapping {len(_bare)} unescaped extra ball(s)...")
                _n_strap, _still = _strap_unescaped_extras(
                    footprint, pcb_data, _bare, tracks, vias_to_add,
                    _tw, clearance, via_size, via_drill, grid_step)
                print(f"  Escape priority: {len(_bare)} extra ball(s) had no "
                      f"escape room; strapped {_n_strap} to their net's fanout "
                      f"inside the BGA ({len(_still)} left for the router)"
                      + (f": {', '.join(_still)}" if _still else ""))
            # #621: a CANCELLED pass has not measured anything, so it must
            # never win this comparison. Its loops broke early, which makes
            # `failed_nets` artificially SHORT (untried balls are not failures)
            # -- so "fewer dropped balls" would read a truncated pass as the
            # better result and throw away the completed single pass's copper.
            # Measured by neutering this guard and re-running ulx3s U1 with a
            # cancel fired ~4s in: the truncated pass WINS (`Under-pad escape wins:
            # 185 -> 0 dropped ball(s)`) and the run ships 0 signal escapes;
            # with the guard it ships 26. Keep the incumbent.
            if cancel_check and cancel_check():
                print(f"  Escape priority: cancelled mid-pass -- keeping the "
                      f"single-pass result ({len(f0)} dropped ball(s)); a "
                      f"truncated pass has measured nothing to compare. Those "
                      f"balls are UNRESCUED, not clearance failures -- and this "
                      f"copper is kept on the strength of a comparison that "
                      f"never ran, so a completed run may ship a different set.")
                return t0, v0, vr0, f0
            # Keep whichever result covers more nets; ties keep the single
            # pass (issue #367 -- mirror the channel/underpad auto-retry's
            # "ties keep the incumbent").
            # #621: a CANCELLED pass has an artificially SHORT failed list --
            # its loops broke early, so balls it never tried are absent from
            # both ledgers. Comparing lengths would then let a truncated pass
            # look like the better result and throw away the completed pass's
            # copper. Measured by neutering this guard and re-running ulx3s U1
            # with a cancel ~4s in: the truncated pass WINS and the run ships 0
            # signal escapes where it otherwise ships 26. Keep the incumbent.
            if cancel_check and cancel_check():
                print("  Escape priority: cancelled mid-pass -- keeping the "
                      "single-pass result")
                return t0, v0, vr0, f0
            if len(failed_nets) < len(f0):
                print(f"  Escape priority wins: {len(f0)} -> "
                      f"{len(failed_nets)} dropped ball(s); using it")
                return tracks, vias_to_add, vias_to_remove, failed_nets
            print(f"  Escape priority did not improve ({len(failed_nets)} vs "
                  f"{len(f0)} dropped) - keeping the single-pass result")
            return t0, v0, vr0, f0

    # --layer-costs (issue #288): same semantics as route.py -- a NEGATIVE cost
    # forbids the layer (no escape copper placed there; e.g. a soon-to-be-plane
    # inner layer), a positive cost >= 1.0 is a preference weight. The channel
    # engine tries layers in list order, so its non-top layers are re-sorted
    # cheapest-first (the top layer stays first: edge escapes are hardwired to
    # it). The under-pad engine keeps the user's physical order (its via spans
    # use layers[0]/layers[-1]) and applies only the forbidden-layer exclusion.
    underpad_layers = layers
    underpad_layer_costs = None  # costs aligned with underpad_layers (#519)
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
        # #519: hand the auto-fallback retry the costs for the SURVIVING
        # layers (len-aligned with underpad_layers, all >= 0) so the recursive
        # call sees exactly what a direct --escape-method underpad run sees.
        # MEASURED INERT today (529-ball A/B: identical copper): the under-pad
        # engine ignores positive weights by design -- its via spans need the
        # physical layer order (see above) -- so only the forbidden-layer
        # exclusion shapes its copper. This is consistency, not behavior;
        # weighted under-pad layer assignment would be a new measured feature.
        underpad_layer_costs = [c for _, c in keep]
        # The even-distribution rebalance would spread escapes right back onto
        # the costly layers the greedy assignment just avoided, so it only
        # balances across the cheapest tier; costlier layers keep only the
        # overflow routes the greedy pass could not fit elsewhere.
        min_cost = min(c for _, c in keep)
        balance_layers = [l for l, c in keep if c <= min_cost + 1e-9]
        if keep[0][0] not in balance_layers:
            # rebalance treats its first entry as the top layer (edge escapes)
            balance_layers = [keep[0][0]] + balance_layers
        if escape_method in ('underpad', 'dogbone'):
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
        print(f"  For a perimeter/2-row connector or staggered package, try: "
              f"qfn_fanout.py --component {footprint.reference} "
              f"--escape-method underpad --allow-via-in-pad")
        return [], [], [], []

    # Sanity-check pad geometry before escaping (see qfn_fanout): overlapping
    # same-footprint pads mean the pad rotation/size is modelled wrong.
    from check_pads import find_pad_overlaps
    _ov = find_pad_overlaps(pcb_data, component=footprint.reference)
    if _ov:
        print(f"  WARNING: {footprint.reference} has {len(_ov)} overlapping "
              f"different-net pad pair(s) - pad geometry looks wrong, fanout "
              f"may be placed across pads. Run: python3 py_router/check_pads.py <board> "
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
    if escape_method not in ('underpad', 'dogbone'):
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
    # Dog-bone (#128) is the same engine with gap-site vias instead of
    # via-in-pad: ball -> 45-stub -> via in the diagonal inter-ball gap.
    if escape_method in ('underpad', 'dogbone'):
        from bga_fanout.underpad import generate_underpad_escape
        net_filter_fn = None
        if net_filter:
            net_filter_fn = lambda name: matches_net_filter(name, net_filter)
        # Differential pairs (issue #182): escape each pair coupled so route_diff
        # can pick the two halves up (without this they go single-ended).
        up_diff_pairs = (find_differential_pairs(footprint, diff_pair_patterns)
                         if diff_pair_patterns else {})
        if _direct_route_nets:
            up_diff_pairs = {k: v for k, v in up_diff_pairs.items()
                             if not ((v.p_pad and v.p_pad.net_name in _direct_route_nets)
                                     or (v.n_pad and v.n_pad.net_name in _direct_route_nets))}
        if up_diff_pairs:
            print(f"  Found {len(up_diff_pairs)} differential pair(s) to escape coupled")
        _up_kw = dict(
            track_width=track_width, clearance=clearance,
            via_size=via_size, via_drill=via_drill, exit_margin=exit_margin,
            net_filter_fn=net_filter_fn,
            diff_pairs=up_diff_pairs, diff_pair_gap=diff_pair_gap,
            grid_step=grid_step,
            only_pad_keys=_pad_filter,
            dogbone=(escape_method == 'dogbone'),
            no_via_in_pad=(same_net_pad_clearance is not None
                           and same_net_pad_clearance > 0),  # #581
            # Rides _up_kw so the shrink rescue's re-run reports too.
            progress_callback=progress_callback,
            cancel_check=cancel_check,
        )
        tracks, vias_to_add, failed_nets = generate_underpad_escape(
            footprint, pcb_data, grid, layers, **_up_kw)
        # #505 fab-floor rescue. NOT during the _single_pass coverage probe:
        # that run's only job is to answer "did the legacy pass drop anything",
        # and rescuing there both wastes a full escape build and changes the
        # gate's answer. The escape-priority passes below it get the rescue.
        import os as _dbgos
        if _dbgos.environ.get('KICAD_FANOUT_RESCUE_DEBUG'):
            print(f"  [rescue-debug] dogbone branch end: failed={failed_nets} "
                  f"single_pass={_single_pass}")
        if failed_nets and not _single_pass:
            tracks, vias_to_add, failed_nets = _underpad_shrink_rescue(
                footprint, pcb_data, grid, layers, _up_kw,
                tracks, vias_to_add, failed_nets)
        # #652 directive 2: whatever the floor could not fit alongside the
        # committed escapes, try again with the nearest neighbour evicted.
        if failed_nets and not _single_pass and env_knobs.FANOUT_RIP_RESCUE:
            tracks, vias_to_add, failed_nets = _underpad_rip_rescue(
                footprint, pcb_data, grid, layers, _up_kw,
                tracks, vias_to_add, failed_nets)
        # #652 directive 1: the last rung is the SURFACE. A ball the via
        # floor cannot serve at this pitch (a 0.25mm via on a 0.5mm pitch
        # busts the half-pitch budget by 3um, closing every via site AND
        # via-in-pad) may still thread the pad gaps on its own layer with a
        # vialess track -- outer-row territory, the way humans escape it.
        # Exact interval geometry, not the router grid: the legal corridor
        # band can be a few um wide and off-grid.
        if failed_nets and not _single_pass and env_knobs.FANOUT_SURFACE_RESCUE:
            tracks, failed_nets = _surface_gap_escape(
                footprint, pcb_data, tracks, vias_to_add, failed_nets,
                track_width, clearance, exit_margin)
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
    _pair_toward: Dict[str, str] = {}  # #469 pair target-side preference
    if diff_pair_patterns:
        diff_pairs = find_differential_pairs(footprint, diff_pair_patterns)
        if _direct_route_nets:
            diff_pairs = {k: v for k, v in diff_pairs.items()
                          if not ((v.p_pad and v.p_pad.net_name in _direct_route_nets)
                                  or (v.n_pad and v.n_pad.net_name in _direct_route_nets))}
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
        # Target-side preference for PAIRS (#469): one shared direction per
        # pair (coupling untouched), biasing which direction the assigner
        # tries first. Same env gate as the single-ended preference.
        _pair_toward = {}
        if env_knobs.FANOUT_TOWARD_TARGETS:
            from bga_fanout.escape import preferred_pair_dirs
            _pair_toward = preferred_pair_dirs(pcb_data, footprint, diff_pairs)
            if _pair_toward:
                print(f"  Target-side pair escape preference active for "
                      f"{len(_pair_toward)} pair(s)")
        pair_escape_assignments, pair_layer_assignments = assign_pair_escapes(
            diff_pairs, grid, channels, layers,
            primary_orientation=primary_escape,
            track_width=track_width,
            clearance=clearance,
            diff_pair_gap=diff_pair_gap,
            via_size=via_size,
            rebalance=rebalance_escape,
            pre_occupied=pre_occupied_exits,
            force_escape_direction=force_escape_direction,
            pair_preferred=_pair_toward
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
        # Escape-priority pass restriction (issue #129): only the pass's own
        # balls are routed; _ignore_prefanned lets pass 2 fan the extra balls
        # of nets pass 1 just fanned (whose copper now trips the net-level
        # "already fanned" detection).
        if _pad_filter is not None and (pad.global_x, pad.global_y) not in _pad_filter:
            continue
        if check_for_previous and pad.net_id in fanned_out_nets \
                and not _ignore_prefanned:
            continue
        fanned_net_ids.add(pad.net_id)

    obstacle_cfg = GridRouteConfig(
        layers=list(layers),
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
    )
    from kicad_dru import install_layer_clearances
    install_layer_clearances(obstacle_cfg, None, None, pcb_data)  # #498
    obstacle_layer_map = build_layer_map(obstacle_cfg.layers)
    print(f"  Building pad-aware obstacle map ({len(fanned_net_ids)} fanned nets excluded)...")
    _prog("building obstacle map...")
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
            # The forced-orientation retry picks the SIDE within the
            # orthogonal orientation; the target-side preference (#469 v3)
            # chooses it when the preference lies in that orientation.
            new_ch, new_dir = find_diff_pair_escape(
                fpair.p_pad.global_x, fpair.p_pad.global_y,
                fpair.n_pad.global_x, fpair.n_pad.global_y,
                grid, channels, secondary_orientation,
                preferred_dir=_pair_toward.get(fp)
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

        # Target-side escape preference (#469, KICAD_FANOUT_TOWARD_TARGETS=1):
        # each pad's escape direction biases toward its net's nearest
        # off-footprint pad; the smart layer assignment then spreads the
        # extra same-direction competition across layers.
        _toward_targets = {}
        if env_knobs.FANOUT_TOWARD_TARGETS:
            from bga_fanout.escape import preferred_escape_dirs
            _toward_targets = preferred_escape_dirs(pcb_data, footprint)
            if _toward_targets:
                print(f"  Target-side escape preference active for "
                      f"{len(_toward_targets)} pad(s)")

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

            # Escape-priority pass restriction (issue #129): only this pass's
            # balls are routed.
            if _pad_filter is not None and (pad.global_x, pad.global_y) not in _pad_filter:
                continue

            # Skip if this pad already has a fanout (check_for_previous mode).
            # _ignore_prefanned: pass 2 fans the extra balls of nets pass 1
            # just fanned (whose copper trips this net-level detection).
            if check_for_previous and pad.net_id in fanned_out_nets \
                    and not _ignore_prefanned:
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
                    pad, grid, channels, layers, exit_margin, force_orient,
                    preferred_dir=_toward_targets.get(
                        (round(pad.global_x, 3), round(pad.global_y, 3)))
                )
                routes.append(route)

        print_route_statistics(routes)

        if not routes:
            # 5-tuple like the normal return below -- the caller unpacks
            # (tracks, vias_to_add, vias_to_remove, failed_nets, routes); the
            # old 4-tuple crashed any pass whose net filter matched no
            # escapable ball (caught by the #498 e2e's single-net BGA probe).
            return [], [], [], [], []

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
        _prog(f"assigning layers to {len(routes)} route(s)...")
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
            _prog(f"resolving {collision_count} collision(s)...")
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

            # Recount UNCONDITIONALLY: resolve_collisions also strips failed
            # nets' tracks, so the count changes even when reassigned == 0 --
            # and the reassigned-only recount left collisions_remaining
            # UNBOUND on that path, crashing the whole fanout at the
            # rebalance check below (hit by the #519 verification run:
            # collisions found, zero routes reassignable).
            new_collision_count, _ = detect_collisions(tracks, existing_tracks, min_spacing, max_pairs=0)
            print(f"  After resolution: {new_collision_count} collisions remaining")
            collisions_remaining = new_collision_count
        else:
            print(f"  Validated: No collisions")
            collisions_remaining = 0

        # Post-resolution layer rebalancing for even distribution. With
        # --layer-costs this only balances across the cheapest tier so costly
        # (soon-to-be-plane) layers keep only the greedy pass's overflow (#288).
        if collisions_remaining == 0 and len(balance_layers) > 1 \
                and not _ignore_prefanned:  # pass-2 extras keep their validated layers (#129)
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
        _prog("placing vias...")
        vias_to_add, vias_to_remove, via_blocked_routes = manage_vias(
            routes, pcb_data, layers[0], via_size, via_drill, clearance
        )

        # Routes whose required via-in-pad would hit an immovable foreign pad
        # (#253) are dropped: without the via their inner-layer copper is
        # disconnected decoration. Remove their tracks and report the nets as
        # failed so the main router picks them up from the bare ball.
        # #508 finding 6 coherence: the net's OTHER balls' routes are dropped
        # WITH their tracks (the old code removed tracks net-wide but left
        # the sibling routes in `routes` -- still counted escaped, shipping
        # via-in-pad balls with no track).
        if via_blocked_routes:
            from bga_fanout.reroute import _remove_route_tracks
            blocked_net_ids = {r.net_id for r in via_blocked_routes}
            for r in routes:
                if r.net_id in blocked_net_ids:
                    _remove_route_tracks(tracks, r)
            routes = [r for r in routes if r.net_id not in blocked_net_ids]
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

        # #508 finding 7: vias_to_add/vias_to_remove were derived from the
        # PRE-repair route layers, and repair_pad_crossings mutates
        # route.layer -- a route moved to the top layer shipped its (now
        # pointless) via-in-pad next to top-layer-only copper
        # (spartan6_6layer step1: 7 F.Cu balls, via + seglayers=['F.Cu']
        # from a via-less input), and a route moved OFF the top layer never
        # got the via its inner copper needs. Re-derive both lists from the
        # FINAL routes; newly via-blocked routes are dropped exactly like
        # the in-pass path. This also un-stales the via-vs-foreign-track
        # guard below, which previously scanned pre-repair vias against
        # post-repair tracks.
        vias_to_add, vias_to_remove, _reblocked = manage_vias(
            best_routes, pcb_data, layers[0], via_size, via_drill, clearance)
        if _reblocked:
            from bga_fanout.reroute import _remove_route_tracks
            _rb_net_ids = {r.net_id for r in _reblocked}
            for r in best_routes:
                if r.net_id in _rb_net_ids:
                    _remove_route_tracks(tracks, r)
            best_routes[:] = [r for r in best_routes
                              if r.net_id not in _rb_net_ids]
            for r in _reblocked:
                name = r.pad.net_name or f"net{r.net_id}"
                if name not in failed_nets:
                    failed_nets.append(name)
            print(f"  Pad-aware: {len(_reblocked)} repaired route(s) newly "
                  f"via-blocked; dropped (#253 semantics)")

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
    # channel engine deterministically drops balls the grid escapes handle
    # (ecp5_mini HyperRAM 11/14 -> 14/14, icepi_zero ECP5 120/124 ->
    # 124/124). When the channel pass dropped any ball, re-run with underpad
    # and keep whichever escapes strictly more; ties keep the channel result
    # (surface routing, no via-in-pad). KICAD_FANOUT_AUTO_DOGBONE=1 opts the
    # retry into a dogbone-first ladder (underpad only if dogbone still
    # dropped; a dogbone/underpad tie keeps dogbone -- fewer via-in-pad).
    # The ladder was REJECTED as the default by the #669 sets1-5 corpus A/B
    # (+10 incomplete nets, +59 kicad DRC vs underpad-only: dogbone gap vias
    # claim inter-ball streets that chains not authored for dogbone collide
    # with). Explicit --escape-method dogbone stays the populated-array
    # doctrine when the chain's params are chosen for it.
    if failed_nets and escape_method == 'auto':
        def _auto_retry(method):
            return _generate_bga_fanout_core(
                footprint, pcb_data, net_filter=net_filter,
                diff_pair_patterns=diff_pair_patterns, layers=underpad_layers,
                layer_costs=underpad_layer_costs,  # filtered to match (#519)
                track_width=track_width, clearance=clearance,
                diff_pair_gap=diff_pair_gap, exit_margin=exit_margin,
                primary_escape=primary_escape,
                force_escape_direction=force_escape_direction,
                rebalance_escape=rebalance_escape,
                via_size=via_size, via_drill=via_drill,
                check_for_previous=check_for_previous,
                no_inner_top_layer=no_inner_top_layer, escape_method=method,
                grid_step=grid_step, _pad_filter=_pad_filter,
                _ignore_prefanned=_ignore_prefanned, _single_pass=_single_pass,
                same_net_pad_clearance=same_net_pad_clearance,
                progress_callback=progress_callback,
                cancel_check=cancel_check)

        # KICAD_FANOUT_AUTO_DOGBONE=0 reverts to the pre-#669 underpad-only
        # retry (the corpus A/B control arm).
        ladder = (['dogbone', 'underpad'] if env_knobs.FANOUT_AUTO_DOGBONE
                  else ['underpad'])
        best_label, best = None, None
        for method in ladder:
            label = 'Dog-bone' if method == 'dogbone' else 'Under-pad'
            if best is None:
                print(f"\n  Channel escape dropped {len(failed_nets)} ball(s) "
                      f"({', '.join(failed_nets)}) - retrying with the "
                      f"{label.lower()} escape "
                      f"(issue{'s #288/#669' if method == 'dogbone' else ' #288'})...")
                _prog(f"channel escape dropped {len(failed_nets)} ball(s) - "
                      f"retrying {label.lower()}...")
            else:
                # The prior grid escape completed but still dropped balls ->
                # the next rung gets its shot (underpad can serve arrays with
                # no legal inter-ball gap site at all).
                print(f"  {best_label} escape still dropped {len(best[3])} "
                      f"ball(s) ({', '.join(best[3])}) - also retrying with "
                      f"the {label.lower()} escape (issue #288)...")
                _prog(f"{best_label.lower()} still dropped {len(best[3])} "
                      f"ball(s) - retrying {label.lower()}...")
            res = _auto_retry(method)
            # #621: same rule as the escape-priority comparison above -- a
            # retry the budget cut short reports a short failed list because
            # untried balls are not failures, so it must not be allowed to
            # displace a completed result on that basis. cancel_check is
            # level-triggered (once true it stays true), so checking after
            # each pass attributes the truncation to the pass it landed in.
            if cancel_check and cancel_check():
                # No fall-through print here: the "did not improve (0 dropped)"
                # line reads as the retry pass having succeeded perfectly,
                # which is the opposite of what a truncated pass measured.
                #
                # The guard prefers the incumbent's copper over a truncated
                # pass's silence. Measured on ulx3s U1 (cancel ~4s in) by
                # neutering it: with the guard 26 balls escape (96 segments,
                # 10 vias); without it the truncated pass wins and 0 do (29
                # segments, 144 plane barrels). It is WRONG when the completed
                # engine would itself have discarded the incumbent --
                # orangecrab_ext_pll U4, where the under-pad pass legitimately
                # concludes "0/0 signals escaped, 54 already-fanned skipped"
                # and its EMPTY result wins, so the finished run ships no
                # signal escape at all (drc total 937, matching this guard
                # turned OFF) while a cancelled run keeps 22 channel escapes
                # (1108). A cancelled pass and a legitimately-empty one return
                # the same ([], []) from here, so the divergence is DISCLOSED,
                # not guessed.
                if best is None:
                    print(f"  {label} escape: cancelled mid-pass -- keeping "
                          f"the channel result and its {len(failed_nets)} "
                          f"dropped ball(s); a truncated pass has measured "
                          f"nothing. Those balls are UNRESCUED, not clearance "
                          f"failures -- and this copper is kept on the "
                          f"strength of a comparison that never ran, so a "
                          f"completed run may legitimately ship less of it.")
                else:
                    print(f"  {label} escape: cancelled mid-pass -- "
                          f"discarding it (a truncated pass has measured "
                          f"nothing); the completed {best_label.lower()} "
                          f"result stays in the contest.")
                break
            # Strictly-fewer wins; a tie keeps the earlier rung (dogbone over
            # underpad: fewer via-in-pad).
            if best is None or len(res[3]) < len(best[3]):
                best_label, best = label, res
            if not best[3]:
                break
        if best is not None:
            if len(best[3]) < len(failed_nets):
                print(f"  {best_label} escape wins: {len(failed_nets)} -> "
                      f"{len(best[3])} dropped ball(s); using it")
                return best
            print(f"  {best_label} escape did not improve ({len(best[3])} "
                  f"dropped) - keeping the channel result")

    # Write-list invariant (#508 findings 6/7): every surviving FanoutRoute
    # must have at least one track in the write list -- a route still counted
    # as escaped but with no copper ships a via-in-pad ball with NO track (a
    # dead drill that consumes hole-to-hole budget downstream; cparti_fpga
    # +1V8/+1V0). This divergence lives entirely inside the write lists
    # (tracks vs routes vs vias_to_add), so no pcb_data-vs-file ledger can
    # see it -- it is asserted here instead.
    from bga_fanout.types import route_uid as _ruid
    _names_by_net = {r.net_id: (r.pad.net_name or f"net{r.net_id}")
                     for r in best_routes}
    _tracked_uids = {t.get('route_uid') for t in tracks}
    _failed_set = set(failed_nets)
    _orphans = [r for r in best_routes
                if _ruid(r) not in _tracked_uids
                and _names_by_net.get(r.net_id) not in _failed_set]
    if _orphans:
        print(f"  WARNING (#508 invariant): {len(_orphans)} escaped route(s) "
              f"have no track in the write list -- their balls would ship a "
              f"dead via-in-pad: "
              f"{sorted({_names_by_net[r.net_id] for r in _orphans})}")

    return tracks, vias_to_add, vias_to_remove, failed_nets


# Per-call report of the plane-ball drop pass (#424 D2), for JSON_SUMMARY /
# the GUI results panel. Refreshed by every top-level generate_bga_fanout call.
LAST_PLANE_DROP_REPORT: Dict = {}

# #621: balls whose escape was never ATTEMPTED because this run's own
# `cancel_check` stopped it -- in practice the GUI's Cancel button or the plan
# executor's Stop, the only cancel sources there are (the CLI passes None).
# Refreshed by every top-level generate_bga_fanout call and EMPTY unless a
# cancel actually fired, so it is published the same way LAST_PLANE_DROP_REPORT
# is.
#
# Deliberately a separate ledger from failed_nets/unescaped_nets: an unfinished
# search has measured nothing about a ball, and folding untried balls into the
# failure list reports a cancel as a routing defect -- which would send the
# planner (or the user) into a pointless tighter-clearance retry.
LAST_CANCEL_SKIPPED: List[str] = []


def fanout_candidate_nets(footprint: Footprint, pcb_data: PCBData,
                          net_filter: Optional[List[str]] = None,
                          plane_min_pads: int = 6) -> List[str]:
    """Net names this BGA fanout would ATTEMPT to escape on `footprint`.

    The requested ledger, reconstructed from the board rather than from a
    finished run -- which is what a CANCELLED run needs, since it has no
    finished run to read. Mirrors the two engines' own intake rules: a ball is
    a candidate when it has a net, is not `unconnected-*`, is not a single-pad /
    NC net (`single_pad_net_ids`), and passes `net_filter`. On an UNFILTERED run
    the under-pad plane rule also applies: a net with >= plane_min_pads balls on
    this part is a plane -- it taps its plane and was never requested (#218).
    With a filter, the EXCLUSIONS are the plane declaration (`--nets '!GND'`),
    so every filter-passing net is a candidate and no pad-count heuristic
    applies -- the same asymmetry `underpad.is_plane` encodes.

    Verified against the finished ledger it has to agree with: on
    interf_u_unrouted_placed U9 this returns 75 names, and an uncancelled run of
    the same command reports `requested: 75`.
    """
    nc_net_ids = single_pad_net_ids(footprint, pcb_data)
    fp_net_counts = Counter(p.net_name for p in footprint.pads if p.net_id)
    names: Set[str] = set()
    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue
        if pad.net_name.lower().startswith('unconnected-'):
            continue
        if pad.net_id in nc_net_ids:
            continue
        if net_filter and not matches_net_filter(pad.net_name, net_filter):
            continue
        if not net_filter and fp_net_counts[pad.net_name] >= plane_min_pads:
            continue                                   # plane / dense rail ball
        names.add(pad.net_name)
    return sorted(names)


def generate_plane_drops(footprint: Footprint,
                         pcb_data: PCBData,
                         layers: Optional[List[str]] = None,
                         track_width: float = 0.1,
                         clearance: float = 0.1,
                         via_size: float = 0.5,
                         via_drill: float = 0.3,
                         net_filter: Optional[List[str]] = None,
                         grid_step: float = 0.0,
                         plane_min_pads: int = 6,
                         verbose: bool = True,
                         plane_net_layers: Optional[Dict[str, List[str]]] = None,
                         no_via_in_pad: bool = False
                         ) -> Tuple[List[Dict], List[Dict], Dict]:
    """Drop a via for every plane-net ball of `footprint` (#424 D2).

    Plane nets are auto-detected with the SAME classification the escape
    engines use to skip them: a net the caller EXCLUDED via `net_filter` with
    >= `plane_min_pads` balls on this footprint (with no filter, the pad-count
    heuristic alone) -- plus any excluded net that already owns a copper zone,
    so boards with existing pours drop regardless of ball count. Each such SMD
    ball gets a dog-bone stub + via in a free inter-ball gap (#128 allocator),
    falling back to an exact-checked, pad-clamped via-in-pad tap; the ball is
    not routed anywhere -- the plane poured later picks the via up at fill.
    This kills the tap-behind-the-ball-wall class (#360) by claiming the tap
    while the under-package space is still open.

    Returns (tracks, vias_to_add, report). Existing same-net connections are
    respected (idempotent re-runs); failures are reported, never fatal -- the
    plane step's tap search remains the fallback.
    """
    fp_pads = [p for p in footprint.pads if p.net_id]
    counts = Counter(p.net_name for p in fp_pads)
    zone_ids = {z.net_id for z in (pcb_data.zones or []) if z.net_id}
    drop_ids: Set[int] = set()
    for p in fp_pads:
        if net_filter is not None:
            if matches_net_filter(p.net_name, net_filter):
                continue                      # fanned as a signal, not a plane
            if counts[p.net_name] >= plane_min_pads or p.net_id in zone_ids:
                drop_ids.add(p.net_id)
        elif counts[p.net_name] >= plane_min_pads:
            drop_ids.add(p.net_id)
    if not drop_ids:
        return [], [], {}
    try:
        grid = analyze_bga_grid(footprint)
    except Exception as e:
        if verbose:
            print(f"  Plane drops skipped: grid analysis failed ({e})")
        return [], [], {}
    if grid is None:
        # Run-6: analyze_bga_grid returns None (not raises) for perimeter
        # packages -- a QFN's corner gap fails the dominant-pitch vote --
        # and the None then crashed generate_underpad_escape at
        # underpad grid.pitch_x. Same advisory as the escape path.
        print(f"  Plane drops skipped: {footprint.reference} has no BGA "
              f"grid (perimeter package?). For a QFN/QFP use "
              f"qfn_fanout.py --component {footprint.reference}")
        return [], [], {}
    from bga_fanout.underpad import generate_underpad_escape
    net_filter_fn = None
    if net_filter:
        net_filter_fn = lambda name: matches_net_filter(name, net_filter)
    rep: Dict = {}
    d_tracks, d_vias, _failed = generate_underpad_escape(
        footprint, pcb_data, grid, layers or ['F.Cu', 'B.Cu'],
        track_width, clearance, via_size, via_drill, exit_margin=0.5,
        plane_min_pads=plane_min_pads, net_filter_fn=net_filter_fn,
        grid_step=grid_step, only_pad_keys=frozenset(),
        plane_drop_nets=drop_ids, plane_drop_report=rep, verbose=verbose,
        plane_net_layers=plane_net_layers, no_via_in_pad=no_via_in_pad)
    return d_tracks, d_vias, rep


def _plane_drop_pass(footprint, pcb_data, new_tracks, new_vias, net_filter,
                     layers, track_width, clearance, via_size, via_drill,
                     grid_step, plane_net_layers=None, no_via_in_pad=False):
    """Plane-ball drops against the board PLUS this call's fresh copper.

    The signal escape's tracks/vias are only result dicts at this point, so
    they are materialized into pcb_data for the duration of the drop pass
    (the #129 escape-priority pattern) -- the drop-only underpad invocation
    then sees them as ordinary obstacles regardless of which engine (channel,
    underpad, dog-bone) produced them. Rotated parts (#137) run the pass in
    their axis-aligned frame like everything else.
    """
    from kicad_parser import Segment as _Seg, Via as _Via
    n_seg0, n_via0 = len(pcb_data.segments), len(pcb_data.vias)
    try:
        for t in new_tracks:
            pcb_data.segments.append(_Seg(
                start_x=t['start'][0], start_y=t['start'][1],
                end_x=t['end'][0], end_y=t['end'][1],
                width=t['width'], layer=t['layer'], net_id=t['net_id']))
        for v in new_vias:
            pcb_data.vias.append(_Via(
                x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                layers=v.get('layers') or ['F.Cu', 'B.Cu'],
                net_id=v['net_id']))
        from bga_fanout.rotate_frame import (is_orthogonal,
                                             to_axis_aligned_frame,
                                             back_transform_results)
        if not is_orthogonal(footprint.rotation):
            rp, back = to_axis_aligned_frame(pcb_data, footprint.reference)
            d_tracks, d_vias, rep = generate_plane_drops(
                rp.footprints[footprint.reference], rp, layers,
                track_width=track_width, clearance=clearance,
                via_size=via_size, via_drill=via_drill,
                net_filter=net_filter, grid_step=grid_step,
                plane_net_layers=plane_net_layers, no_via_in_pad=no_via_in_pad)
            back_transform_results(d_tracks, d_vias, [], back)
            return d_tracks, d_vias, rep
        return generate_plane_drops(
            footprint, pcb_data, layers,
            track_width=track_width, clearance=clearance,
            via_size=via_size, via_drill=via_drill,
            net_filter=net_filter, grid_step=grid_step,
            plane_net_layers=plane_net_layers, no_via_in_pad=no_via_in_pad)
    finally:
        del pcb_data.segments[n_seg0:]
        del pcb_data.vias[n_via0:]


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
                        layer_costs: Optional[List[float]] = None,
                        plane_drop: str = 'auto',
                        plane_net_layers: Optional[Dict[str, List[str]]] = None,
                        _pad_filter: Optional[Set[Tuple[float, float]]] = None,
                        _ignore_prefanned: bool = False,
                        _single_pass: bool = False,
                        # #581: > 0 forbids via-in-pad (dog-bone escapes/
                        # drops only); None auto-reads the .kicad_pro record.
                        same_net_pad_clearance: Optional[float] = None,
                        # progress_callback(current, total, label) -- the fanout
                        # tab used to pulse ONE static "Running BGA fanout..."
                        # for the whole run (minutes on a big BGA). Phase-level
                        # here; (0, 0, label) means indeterminate.
                        progress_callback=None,
                        cancel_check=None) -> Tuple[List[Dict], List[Dict], List[Dict]]:
    """BGA fanout: the signal escape engines plus the plane-ball drop pass.

    See _generate_bga_fanout_core for the escape-engine parameters. After the
    signal escape completes (whichever engine won), every skipped plane-net
    ball is dropped to a via -- dog-bone gap site preferred, exact-checked
    via-in-pad fallback -- so the plane poured later picks it up directly
    (#424 D2 / #360). `plane_drop='auto'` (default) detects plane nets from
    the fanout's own exclusions and any existing zones; 'off' disables. The
    KICAD_FANOUT_PLANE_DROP env knob overrides both ('0'/'off' or '1'/'auto'),
    so recorded manifests can A/B the pass without editing. The per-net drop
    report is published as bga_fanout.LAST_PLANE_DROP_REPORT.

    `cancel_check` (#621) is the standard zero-arg cooperative predicate
    (`batch_route` / `create_plane` take the same one). It is honoured at every
    escape pass head and at the under-pad engine's four escape loop heads.
    Balls escaped before it fired keep their tracks and vias; balls it never got
    to are published as bga_fanout.LAST_CANCEL_SKIPPED and are NOT in
    failed_nets. Passing None (the default, and what the CLI passes) is fully
    inert. Its only caller is the GUI's Cancel button / the plan executor's
    Stop.
    """
    # Config-parity probe (#493). The plane engines have dumped their kwargs
    # since #362; fanout did not, which is why a GUI/CLI escape divergence here
    # had to be chased by eye. Same contract: only active under
    # KICAD_DUMP_BATCH_KWARGS + ..._CONTINUE=1, never alters routing. footprint
    # and pcb_data are dropped (board payload); the component ref is kept so
    # calls pair up across the two captures. Lives on the public wrapper so
    # plane_drop is part of the compared set; core-internal recursions
    # (escape priority, auto retry, rotation) no longer dump -- symmetrically
    # on both fronts, so pairing is unaffected.
    # cancel_check is dropped alongside the board payload (#621): it is a live
    # closure, not a routing parameter -- it cannot serialise, and the two
    # fronts legitimately hold DIFFERENT objects there (None on the CLI vs the
    # GUI's Cancel button), so comparing it would report a permanent phantom
    # divergence. Dropping it also keeps the dumped key set byte-identical to
    # before this change.
    try:
        from route import _dump_engine_config as _dump
        # cancel_check is dropped alongside the board payload (#621): it is a
        # live closure, not a routing parameter -- it cannot serialise, and the
        # two fronts legitimately hold DIFFERENT objects there (None on the CLI
        # vs the GUI's Cancel button), so comparing it would report a permanent
        # phantom divergence. Dropping it also keeps the dumped key set
        # byte-identical to before this change.
        _cfg = {k: v for k, v in locals().items()
                if k not in ('footprint', 'pcb_data', '_dump', 'cancel_check',
                             'progress_callback')}
        _cfg['component'] = getattr(footprint, 'reference', None)
        _dump('bga_fanout', _cfg)
    except Exception as _e:
        # A missing dep / stale Rust router must NOT be swallowed here (#457):
        # startup failures raise StartupCheckError, which IS an Exception.
        from startup_checks import StartupCheckError
        if isinstance(_e, StartupCheckError):
            raise

    # #581: resolve once in the wrapper so the core and the plane-drop pass
    # share the same value (the core's own auto-read then no-ops).
    if same_net_pad_clearance is None:
        from protected_nets import read_snpc_for_pcb_data as _read_snpc581
        same_net_pad_clearance = _read_snpc581(pcb_data)
    _ref = getattr(footprint, 'reference', '?')
    if progress_callback:
        _nballs = sum(1 for _p in footprint.pads if _p.net_id)
        progress_callback(0, 0, f"BGA fanout {_ref}: escaping {_nballs} ball(s)...")
    # #621: wrap the caller's predicate so we learn whether a cancel actually
    # FIRED, rather than re-asking afterwards -- re-asking would report a run
    # whose loops all completed as cut short if the flag flipped during the
    # bounded tail. `_fired` is the durable record of a real cancel.
    _fired = [False]
    _cc = None
    if cancel_check is not None:
        def _cc():                                    # noqa: F811
            if cancel_check():
                _fired[0] = True
                return True
            return False

    tracks, vias_to_add, vias_to_remove, failed_nets = _generate_bga_fanout_core(
        footprint, pcb_data, net_filter=net_filter,
        diff_pair_patterns=diff_pair_patterns, layers=layers,
        track_width=track_width, clearance=clearance,
        diff_pair_gap=diff_pair_gap, exit_margin=exit_margin,
        primary_escape=primary_escape,
        force_escape_direction=force_escape_direction,
        rebalance_escape=rebalance_escape,
        via_size=via_size, via_drill=via_drill,
        check_for_previous=check_for_previous,
        no_inner_top_layer=no_inner_top_layer, escape_method=escape_method,
        grid_step=grid_step, layer_costs=layer_costs, cancel_check=_cc,
        progress_callback=progress_callback,
        _pad_filter=_pad_filter, _ignore_prefanned=_ignore_prefanned,
        _single_pass=_single_pass,
        same_net_pad_clearance=same_net_pad_clearance)

    # #621 partial ledger, computed ONLY when a cancel actually fired (so an
    # ordinary run does not even build the sets). A candidate ball that carries
    # no copper from this call AND is not in failed_nets was never concluded:
    # that complement is the untried set, and it is published separately so an
    # unfinished search is never counted as a measured escape failure.
    LAST_CANCEL_SKIPPED.clear()
    if _fired[0]:
        _live_ids = ({t.get('net_id') for t in tracks}
                     | {v.get('net_id') for v in vias_to_add})
        _live_names = {n.name for nid, n in pcb_data.nets.items()
                       if nid in _live_ids}
        _failed_names = set(failed_nets)
        LAST_CANCEL_SKIPPED.extend(
            n for n in fanout_candidate_nets(footprint, pcb_data, net_filter)
            if n not in _live_names and n not in _failed_names)

    # #621 partial ledger, computed ONLY when a cancel actually fired (so an
    # ordinary run does not even build the sets). A candidate ball that carries
    # no copper from this call AND is not in failed_nets was never concluded:
    # that complement is the untried set, and it is published separately so an
    # unfinished search is never counted as a measured escape failure.
    LAST_CANCEL_SKIPPED.clear()
    if _fired[0]:
        _live_ids = ({t.get('net_id') for t in tracks}
                     | {v.get('net_id') for v in vias_to_add})
        _live_names = {n.name for nid, n in pcb_data.nets.items()
                       if nid in _live_ids}
        _failed_names = set(failed_nets)
        LAST_CANCEL_SKIPPED.extend(
            n for n in fanout_candidate_nets(footprint, pcb_data, net_filter)
            if n not in _live_names and n not in _failed_names)

    LAST_PLANE_DROP_REPORT.clear()
    _knob = (env_knobs.FANOUT_PLANE_DROP or '').strip().lower()
    _enabled = {'0': False, 'off': False, 'no': False,
                '1': True, 'on': True, 'auto': True}.get(
                    _knob, plane_drop != 'off')
    # #621: the drop pass is a TAIL pass that deliberately runs after the
    # signal escape so signals keep first claim (the measured F-plan ordering).
    # On a cancelled run there is no finished signal escape for it to come
    # after, so running it anyway inverts that ordering and ships a board of
    # plane barrels with no escapes -- measured on ulx3s U1 (cancel ~4s in):
    # 115 via-in-pad drops, 0 escapes, 47 pad-via grazes. Skip it and say so.
    if _enabled and _fired[0]:
        print("  Plane drops (#424): SKIPPED -- the signal escape was "
              "cancelled, and the drop pass must not claim under-package "
              "space ahead of escapes that never ran")
        _enabled = False
    if _enabled and _pad_filter is None and not _single_pass:
        if progress_callback:
            progress_callback(
                0, 0, f"BGA fanout {_ref}: dropping plane balls to vias...")
        d_tracks, d_vias, rep = _plane_drop_pass(
            footprint, pcb_data, tracks, vias_to_add, net_filter,
            layers, track_width, clearance, via_size, via_drill, grid_step,
            plane_net_layers=plane_net_layers,
            no_via_in_pad=(same_net_pad_clearance is not None
                           and same_net_pad_clearance > 0))
        tracks = tracks + d_tracks
        vias_to_add = vias_to_add + d_vias
        LAST_PLANE_DROP_REPORT.update(rep)
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
    # --width is an ALIAS (see the qfn_fanout twin): the two fanout CLIs used
    # to disagree on the name for the same concept. dest stays `track_width`.
    parser.add_argument('--track-width', '--width', '-w', type=float,
                        default=defaults.BGA_TRACK_WIDTH,
                        help=f'Track width in mm '
                             f'(default: {defaults.BGA_TRACK_WIDTH}; '
                             f'--width is an alias)')
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
    parser.add_argument('--same-net-pad-clearance', type=float, default=None,
                        help='#581: > 0 forbids via-in-pad entirely (under-pad escapes and '
                             'plane drops run dog-bone; balls with no legal off-pad via site '
                             'fail visibly) and is recorded in the sibling .kicad_pro so later '
                             'chain steps inherit it. -1 explicitly allows via-in-pad. '
                             'Default: the project record, else allowed.')
    parser.add_argument('--escape-method', choices=['auto', 'channel', 'underpad', 'dogbone'], default='auto',
                        help='Fanout engine (default: auto). "channel" = 45-stub + '
                             'channel router with diff-pair support. "underpad" = dense-array '
                             'grid escape (issue #122): each signal vias in its pad and routes '
                             'under the pad field on inner layers, escaping fully-populated '
                             'arrays (e.g. ulx3s 22x22) the channel router cannot. Use a small '
                             'via/track for dense pitches (e.g. via 0.35, track 0.12 at 0.8mm). '
                             '"dogbone" = underpad with the escape via in the diagonal '
                             'inter-ball gap instead of in the pad (issue #128): ball -> 45 '
                             'stub -> staggered gap via, keeping ball-grid positions free of '
                             'barrels on the inner layers (the standard hand-layout escape). '
                             '"auto" = channel first, and if it drops any ball, retry with '
                             'underpad and keep whichever escapes more (issue #288; '
                             'KICAD_FANOUT_AUTO_DOGBONE=1 opts the retry into a '
                             'dogbone-first ladder, rejected as default by the #669 A/B).')
    parser.add_argument('--plane-net-layers', nargs='+', default=None,
                        metavar='NET:LAYER[,LAYER...]',
                        help="Declare the FUTURE plane plan: "
                             "for each plane net, the layer(s) it will be poured "
                             "on, e.g. GND:In1.Cu,In4.Cu P3.3V:In2.Cu. Balls whose "
                             "OWN layer is declared get a synthetic-fill reach "
                             "prediction; balls the future pour will reach skip "
                             "their drop via entirely (fanout-time pour-direct).")
    parser.add_argument('--plane-drop', choices=['auto', 'off'], default='auto',
                        help='Drop a via for every plane-net ball after the signal escape '
                             '(#424): dog-bone gap via where free, else via-in-pad, so the '
                             'plane poured later picks the ball up directly instead of the '
                             'plane step tapping through the finished ball field (#360). '
                             '"auto" (default) detects plane nets from the --nets exclusions '
                             '(>= 6 balls, or any excluded net that already owns a zone); '
                             '"off" disables. KICAD_FANOUT_PLANE_DROP=0/1 overrides both '
                             '(the recorded-manifest A/B switch).')
    parser.add_argument('--grid-step', type=float, default=defaults.GRID_STEP,
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
    # #489 section 9: fanout is where a teardrop matters most (a 0.1mm trace
    # meeting a 0.25mm via pad), and this step had no way to ask for one.
    parser.add_argument('--add-teardrops', action='store_true',
                        help='Add teardrop settings to all pads and vias in the output file')
    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    # #381 D8: the post-engine DRC-floor writeback below already reads
    # getattr(args, 'no_fix_drc_settings', False), but the flag was never
    # defined -- so --no-fix-drc-settings silently did nothing. Define it (and
    # the shared DRC-fix flags) via the same helper the routing scripts use.
    # Default (store_true) keeps no_fix_drc_settings=False => writeback ON =>
    # identical behavior for existing commands.
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)
    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()
    from fix_kicad_drc_settings import warn_if_missing_project_floor
    warn_if_missing_project_floor(args.pcb)  # #441: a dropped sibling .kicad_pro strands the DRC floor
    set_default_fab_tier(*fab_tier_from_args(args))
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.pcb),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None))
    # Below-floor params are pinned up to the fab floor (warned); apply the clamps
    # (#513 item 1: discarding this dict shipped sub-floor vias after the warning).
    for _pname, _pfloor in _pinned_floors.items():
        setattr(args, _pname, _pfloor)

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

    # Staggered-lattice guard (#500). bga_fanout models a BALL GRID. A staggered
    # multi-row no-lead package (AQFN and friends) is not one: its two offset
    # rows project onto each axis at HALF the real pad spacing, so the detected
    # pitch is half the truth and every downstream escape budget is computed
    # against it. osprey_kb's Nordic_AQFN-73 reports pitch 0.25 where no two
    # pads are closer than 0.5, its escape budget evaluates to `via <= -0.20mm`
    # (impossible for any via), and the run took 2967s of dropping balls and
    # retrying under-pad before finishing. qfn_fanout does the same chip in
    # 2.4s, 39/39, DRC-clean. Fail in a second with that command instead.
    _grid_for_guard = analyze_bga_grid(footprint)
    _stagger_why = staggered_lattice_diagnosis(footprint, _grid_for_guard)
    if _stagger_why and not env_knobs.ALLOW_STAGGERED_BGA:
        print(f"\nERROR: {footprint.reference} ({footprint.footprint_name}) is "
              f"not a grid array: {_stagger_why}.")
        print("  bga_fanout models a ball grid; on a staggered package its "
              "pitch reads half the real pad spacing, so the escape budget is "
              "impossible and the run takes minutes to hours.")
        print("  Use qfn_fanout, which handles these:")
        print(f"    python3 py_router/qfn_fanout.py <board> --component "
              f"{footprint.reference} --escape-method underpad "
              f"--allow-via-in-pad ...")
        print("  Set KICAD_ALLOW_STAGGERED_BGA=1 to run anyway.")
        return 1

    # #621: the CLI passes no cancel_check. The engine's cooperative cancel is
    # the GUI's (its Cancel button / the plan executor's Stop); a CLI-side
    # wall-clock budget was removed deliberately -- no result this tool produces
    # may depend on timing, or the same command stops producing the same board.
    tracks, vias_to_add, vias_to_remove, _failed_nets = generate_bga_fanout(
        footprint,
        pcb_data,
        same_net_pad_clearance=args.same_net_pad_clearance,  # #581
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
        layer_costs=args.layer_costs,
        plane_drop=args.plane_drop,
        plane_net_layers=(
            {spec.split(':', 1)[0]: spec.split(':', 1)[1].split(',')
             for spec in args.plane_net_layers}
            if args.plane_net_layers else None)
    )

    if tracks:
        print(f"\nWriting {len(tracks)} tracks to {args.output}...")
        if vias_to_add:
            print(f"  Adding {len(vias_to_add)} vias")
        if vias_to_remove:
            print(f"  Removing {len(vias_to_remove)} vias")
        net_names = {nid: net.name for nid, net in pcb_data.nets.items()}
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias_to_add,
                                   vias_to_remove, net_id_to_name=net_names,
                                   add_teardrops=args.add_teardrops)
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
    # #424 D2: plane-drop stubs are taps, not escapes -- keep their nets out
    # of the requested/escaped ledger (they were never requested).
    _drop_names = set((LAST_PLANE_DROP_REPORT.get('nets') or {}).keys())
    _drop_ids = {nid for nid, net in pcb_data.nets.items()
                 if net.name in _drop_names}
    escaped_net_ids = {t['net_id'] for t in tracks
                       if t.get('net_id') is not None
                       and t['net_id'] not in _drop_ids}
    unescaped = sorted(set(_failed_nets))
    escaped = len(escaped_net_ids)
    # The CLI never cancels (#621: no --deadline, no other CLI cancel source),
    # so LAST_CANCEL_SKIPPED is empty here by construction and every ball was
    # concluded one way or the other. The partial-ledger arithmetic lives in the
    # engine for the GUI, which does have a cancel.
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
                via_drill=getattr(args, 'via_drill', None),
                clamp_nondefault_netclasses=True)  # #439: fanout escapes route to --clearance; always clamp
        except Exception as _e:
            print(f"  (skipped DRC-settings fix: {_e})")
        # #581: record an ACTIVE same-net pad via clearance so later chain
        # steps keep their vias off same-net pads too.
        try:
            from protected_nets import (persist_same_net_pad_clearance,
                                        pro_path_for_board)
            if args.same_net_pad_clearance is not None \
                    and args.same_net_pad_clearance > 0:
                persist_same_net_pad_clearance(
                    pro_path_for_board(out_path), args.same_net_pad_clearance)
        except Exception as _e:
            print(f"  (skipped same-net pad clearance record: {_e})")
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
        # #472: nets deliberately DEFERRED from fanout (surface-reachable,
        # direct-routed by a later step). Not failures. The route steps'
        # bare-ball zone exemption keeps them routable automatically.
        'deferred_fanout_nets': sorted(_direct_route_nets),
        # #424 D2: per-net plane-ball drop counts (gap/in_pad/existing/failed);
        # empty when the pass is off or the part has no plane balls.
        'plane_drop': dict(LAST_PLANE_DROP_REPORT),
    }
    try:                       # #653: env knobs into the machine-readable
        import env_knobs as _ek653   # summary, so a harness can detect a
        summary['env_knobs'] = _ek653.active_env_knobs()   # dirty baseline
    except Exception:          # without re-reading logs
        pass
    print(f"JSON_SUMMARY: {_json.dumps(summary)}")
    return 0


if __name__ == '__main__':
    exit(main())
