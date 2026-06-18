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

# Pad-aware obstacle checking (reuses obstacle_map.py) - imported lazily where
# needed to avoid a hard dependency / import cycle at module import time.
from obstacle_map import check_line_clearance


def segments_clear_of_pads(segments: List[Dict],
                           layer: str,
                           obstacles,
                           cfg,
                           layer_map: Dict[str, int]) -> bool:
    """Return True if every segment is clear of foreign-pad/track/via obstacles.

    Each segment is a dict with 'start' and 'end' (x, y) tuples. The check is
    done on `layer` using the shared obstacle map (which already encodes
    foreign pads, existing copper and vias; through-hole pads block all layers).

    If obstacles/cfg/layer_map are not provided (None), this is a no-op that
    returns True so callers keep their previous behavior.
    """
    if obstacles is None or cfg is None or layer_map is None:
        return True
    layer_idx = layer_map.get(layer)
    if layer_idx is None:
        return True
    for seg in segments:
        (x1, y1) = seg['start']
        (x2, y2) = seg['end']
        if not check_line_clearance(obstacles, x1, y1, x2, y2, layer_idx, cfg):
            return False
    return True


def _seg_hits_pad(x1, y1, x2, y2, pad, samples=16, margin: float = 0.0) -> bool:
    """True if the segment passes through pad's bounding box (sampled).

    margin expands the box (clearance + track half-width) to turn the raw
    overlap test into a clearance test; default 0.0 keeps the true-short
    semantics most callers rely on.
    """
    hx = pad.size_x / 2.0 + margin
    hy = pad.size_y / 2.0 + margin
    px, py = pad.global_x, pad.global_y
    for t in range(samples + 1):
        f = t / samples
        x = x1 + (x2 - x1) * f
        y = y1 + (y2 - y1) * f
        if abs(x - px) <= hx and abs(y - py) <= hy:
            return True
    return False


def segments_clear_of_foreign_pads(segments: List[Dict], layer: str, net_id: int,
                                   foreign_pads, layer_map: Dict[str, int],
                                   margin: float = 0.0) -> bool:
    """True if no segment (on `layer`) crosses a foreign pad on another net.

    margin (clearance + track half-width) turns the raw crossing test into a
    clearance test; default 0.0 preserves the true-short semantics.
    """
    if not foreign_pads:
        return True
    for seg in segments:
        (x1, y1) = seg['start']
        (x2, y2) = seg['end']
        for pad in foreign_pads:
            if pad.net_id == net_id:
                continue
            th = pad.drill > 0
            if not th and layer not in (pad.layers or []):
                continue
            if _seg_hits_pad(x1, y1, x2, y2, pad, margin=margin):
                return False
    return True


def route_segments(route: 'FanoutRoute') -> List[Dict]:
    """Enumerate the geometric segments of a route (pad -> ... -> jog_end).

    Mirrors generate_tracks_from_routes' geometry so the obstacle check sees the
    same copper that will be written. Returns a list of {'start','end'} dicts.
    """
    segs: List[Dict] = []

    def add(a, b):
        if a is None or b is None:
            return
        if abs(a[0] - b[0]) < 1e-9 and abs(a[1] - b[1]) < 1e-9:
            return
        segs.append({'start': a, 'end': b})

    if getattr(route, 'neighbor_connection', False):
        add(route.pad_pos, route.exit_pos)
        return segs

    if route.channel_point:
        # Half-edge inner pad tent shape
        add(route.pad_pos, route.channel_point)
        if route.channel_point2:
            add(route.channel_point, route.channel_point2)
            add(route.channel_point2, route.stub_end)
        else:
            add(route.channel_point, route.stub_end)
        add(route.stub_end, route.exit_pos)
    else:
        add(route.pad_pos, route.stub_end)
        if route.pre_channel_jog:
            add(route.stub_end, route.pre_channel_jog)
            add(route.pre_channel_jog, route.exit_pos)
        else:
            add(route.stub_end, route.exit_pos)

    # Jog at exit
    if route.jog_end:
        if route.jog_extension:
            add(route.exit_pos, route.jog_extension)
            add(route.jog_extension, route.jog_end)
        else:
            add(route.exit_pos, route.jog_end)

    return segs


def _route_jog_clears_pads(route: 'FanoutRoute', obstacles, cfg,
                           layer_map: Dict[str, int], foreign_pads,
                           margin: float = 0.0) -> bool:
    """True if dropping the exit jog tail makes the route pad-clear.

    The 45 jog past the BGA exit is decorative (it does not affect
    connectivity). When the ONLY pad crossing is in that tail, we can drop it
    instead of moving/removing the whole route. margin>0 makes this a clearance
    check (catch a jog that merely grazes within clearance, not just overlaps).
    """
    if route.jog_end is None:
        return False  # nothing to drop
    saved_end, saved_ext = route.jog_end, route.jog_extension
    route.jog_end = None
    route.jog_extension = None
    try:
        segs = route_segments(route)
        clear = segments_clear_of_foreign_pads(segs, route.layer, route.net_id,
                                               foreign_pads, layer_map, margin=margin)
    finally:
        if not clear:
            route.jog_end, route.jog_extension = saved_end, saved_ext
    return clear


def clear_escapes_of_foreign_pads(routes: List['FanoutRoute'], tracks: List[Dict],
                                  grid: 'BGAGrid', track_width: float, clearance: float,
                                  foreign_pads, layer_map: Dict[str, int],
                                  net_names: Dict[int, str] = None):
    """Make axial escapes clear breakout-region foreign pads without rerouting.

    The pad-aware repair triggers only on true crossings (deliberately, to keep
    legally-spaced routes). This handles the remaining within-CLEARANCE grazes
    (issue #123 PAD-SEGMENT) on a route's outer escape, in escalating, always
    connectivity-safe steps:

      1. Trim the decorative 45 exit jog if that alone clears the route.
      2. Otherwise pull the exit_pos inward along the escape axis to the longest
         length that clears the pad - but never inside the BGA zone (the free
         end must stay outside so the router can land on it).
      3. If even the minimum exit (just outside the zone) still grazes, the ball
         cannot be fanned out here: drop it and warn (a smaller via/track or a
         component nudge is needed).

    Returns (n_fixed, dropped_net_names). Diff-pair legs are handled per-leg;
    dropping one leg is left to the caller's diff-pair handling.
    """
    if not foreign_pads:
        return 0, []
    net_names = net_names or {}
    margin = clearance + track_width / 2
    eps = 0.001
    n_fixed = 0
    dropped: List[str] = []

    for route in routes:
        if route_clear_of_foreign_pads(route, foreign_pads, layer_map, margin=margin):
            continue  # already clear at full clearance

        # Step 1: jog trim.
        if _route_jog_clears_pads(route, None, None, layer_map, foreign_pads, margin=margin):
            _rebuild_track_for_route(tracks, route, track_width)
            n_fixed += 1
            continue

        # Step 2: shorten the outer escape along its axis, floored at the zone edge.
        d = route.escape_dir
        if d in ('left', 'right', 'up', 'down'):
            saved_exit, saved_jog = route.exit_pos, route.jog_end
            saved_ext, saved_stub = route.jog_extension, route.stub_end
            ex, ey = route.exit_pos
            if d == 'right':
                bound, cur, vary = grid.max_x, ex, 'x'
            elif d == 'left':
                bound, cur, vary = grid.min_x, ex, 'x'
            elif d == 'down':
                bound, cur, vary = grid.max_y, ey, 'y'
            else:  # up
                bound, cur, vary = grid.min_y, ey, 'y'
            sign = 1.0 if cur > bound else -1.0
            min_coord = bound + sign * eps  # just outside the zone
            # Walk inward from current exit toward the zone edge; keep the jog off.
            # For edge pads stub_end IS the exit (the grazing segment is
            # pad->stub_end), so move both; for channel routes stub_end sits at
            # the channel inside the zone and must stay put.
            route.jog_end = None
            route.jog_extension = None
            n_steps = max(1, int(abs(cur - min_coord) / (clearance / 2)) + 1)
            fixed = False
            for i in range(1, n_steps + 1):
                c = cur - sign * (abs(cur - min_coord)) * (i / n_steps)
                new_pt = (c, ey) if vary == 'x' else (ex, c)
                route.exit_pos = new_pt
                if route.is_edge:
                    route.stub_end = new_pt
                if route_clear_of_foreign_pads(route, foreign_pads, layer_map, margin=margin):
                    _rebuild_track_for_route(tracks, route, track_width)
                    n_fixed += 1
                    fixed = True
                    break
            if fixed:
                continue
            # Step 3: cannot clear even at the zone edge -> restore and drop.
            route.exit_pos, route.jog_end = saved_exit, saved_jog
            route.jog_extension, route.stub_end = saved_ext, saved_stub

        nm = net_names.get(route.net_id) or (route.pad.net_name if route.pad else None)
        if nm and nm not in dropped:
            dropped.append(nm)

    return n_fixed, dropped


def route_clear_of_foreign_pads(route: 'FanoutRoute',
                                foreign_pads, layer_map: Dict[str, int],
                                margin: float = 0.0) -> bool:
    """Geometric guard: True if no route segment crosses a foreign pad.

    `foreign_pads` is a precomputed list of pads belonging to OTHER components
    (not the BGA being fanned). This catches foreign pads that share a net with
    a BGA ball - those are excluded from the shared obstacle map (their net is a
    fanned net) yet a DIFFERENT net's stub must still not cross them. A pad is
    only a blocker for a route on a different net. Through-hole pads block all
    layers; SMD pads block only their own copper layers. margin>0 turns the raw
    crossing test into a clearance test.
    """
    if not foreign_pads:
        return True
    layer = route.layer
    for seg in route_segments(route):
        (x1, y1) = seg['start']
        (x2, y2) = seg['end']
        for pad in foreign_pads:
            if pad.net_id == route.net_id:
                continue  # same net - allowed to touch
            th = pad.drill > 0
            if not th and layer not in (pad.layers or []):
                continue
            if _seg_hits_pad(x1, y1, x2, y2, pad, margin=margin):
                return False
    return True


def route_clear_of_pads(route: 'FanoutRoute',
                        obstacles,
                        cfg,
                        layer_map: Dict[str, int],
                        foreign_pads=None) -> bool:
    """Return True if the whole route is clear of foreign pads/copper/vias.

    Checks every segment of the route on the route's own layer using the shared
    obstacle map. A through-hole obstacle blocks all layers (already encoded in
    the map). When `foreign_pads` is given, also applies a precise geometric
    guard against foreign-component pads (catches foreign pads excluded from the
    shared map because they share a net with a fanned BGA ball). No-op (True) if
    obstacles/cfg/layer_map are None.
    """
    if obstacles is None or cfg is None or layer_map is None:
        return True
    if not segments_clear_of_pads(route_segments(route), route.layer,
                                  obstacles, cfg, layer_map):
        return False
    if foreign_pads is not None and not route_clear_of_foreign_pads(route, foreign_pads, layer_map):
        return False
    return True


def try_reroute_single_ended(route: 'FanoutRoute',
                              alternate_channel: Channel,
                              grid: BGAGrid,
                              exit_margin: float,
                              tracks: List[Dict],
                              existing_tracks: List[Dict],
                              available_layers: List[str],
                              track_width: float,
                              clearance: float,
                              no_inner_top_layer: bool = False,
                              obstacles=None,
                              cfg=None,
                              layer_map: Dict[str, int] = None,
                              foreign_pads=None) -> Optional[Tuple['FanoutRoute', str]]:
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
            # Reject candidate only if it would actually short a foreign pad
            # (raw overlap, matching the DRC/checker). The clearance-padded map
            # is intentionally NOT used as a hard reject here: doing so diverts
            # legally-spaced candidates and cascades into worse layer choices.
            if not segments_clear_of_foreign_pads(new_segments, layer, route.net_id,
                                                  foreign_pads, layer_map):
                continue
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
                     no_inner_top_layer: bool = False,
                     obstacles=None,
                     cfg=None,
                     layer_map: Dict[str, int] = None,
                     foreign_pads=None) -> Optional[Tuple['FanoutRoute', str, List[Dict]]]:
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
            # Reject candidate if any segment (incl. exit jog) crosses a foreign
            # pad / existing copper / via on this layer.
            jog_end_check, _ = calculate_jog_end(
                exit_pos, route.escape_dir, layer, available_layers, jog_length,
                is_diff_pair=route.pair_id is not None,
                is_outside_track=False, pair_spacing=0
            )
            check_segs = new_segments + [{'start': exit_pos, 'end': jog_end_check}]
            # Reject only a true foreign-pad short (see try_reroute_single_ended).
            if not segments_clear_of_foreign_pads(check_segs, layer, route.net_id,
                                                  foreign_pads, layer_map):
                continue
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
                       no_inner_top_layer: bool = False,
                       obstacles=None,
                       cfg=None,
                       layer_map: Dict[str, int] = None,
                       foreign_pads=None) -> Tuple[int, List[str]]:
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

                alt_channels = []
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
                            track_width, clearance, no_inner_top_layer,
                            None, None, None, None
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
                            track_width, clearance, None, no_inner_top_layer,
                            None, None, None, None
                        )
                        if result:
                            new_jogged_route, new_layer, new_jogged_tracks = result

                            # If to_jog is one half of a diff pair, its partner must
                            # move to the SAME layer too - jogging just one half splits
                            # P and N across layers (and can leave the partner shorting
                            # another net on the vacated channel), making the pair
                            # unroutable downstream. Jog the partner onto new_layer
                            # (layer forced, and seeing to_jog's new tracks so the two
                            # halves don't collide). If the partner can't follow,
                            # abandon this jog rather than split the pair.
                            partner = None
                            partner_result = None
                            if to_jog.pair_id is not None:
                                partner = next((r for r in routes
                                                if r.pair_id == to_jog.pair_id
                                                and r.net_id != to_jog.net_id), None)
                                if partner is None:
                                    continue
                                # Keep the pair ADJACENT: shift the partner by the
                                # SAME delta to_jog just moved, so both halves stay
                                # one channel apart. Jogging the partner to its own
                                # "farther channel" instead splays a vertical pair to
                                # opposite sides (P right, N left), over-separating it
                                # (e.g. 84.1/83.3 -> 84.9/82.5) so it can no longer
                                # route as a tight pair.
                                if to_jog.channel is None or partner.channel is None:
                                    continue
                                delta = farther_ch.position - to_jog.channel.position
                                target_pos = partner.channel.position + delta
                                same_orient = [c for c in channels
                                               if c.orientation == partner.channel.orientation]
                                partner_ch = min(same_orient,
                                                 key=lambda c: abs(c.position - target_pos)) if same_orient else None
                                if partner_ch is None or abs(partner_ch.position - target_pos) > 0.05:
                                    continue
                                partner_result = try_jogged_route(
                                    partner, partner_ch, grid, exit_margin,
                                    tracks + new_jogged_tracks, existing_tracks, [new_layer],
                                    track_width, clearance, None, no_inner_top_layer,
                                    None, None, None, None
                                )
                                if partner_result is None:
                                    continue

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

                            # Move the diff-pair partner onto the same layer
                            if partner_result is not None:
                                p_route, p_layer, p_tracks = partner_result
                                p_old = [i for i, t in enumerate(tracks)
                                         if t.get('net_id') == partner.net_id]
                                for idx in sorted(p_old, reverse=True):
                                    tracks.pop(idx)
                                routes[routes.index(partner)] = p_route
                                tracks.extend(p_tracks)
                                p_name = net_names.get(partner.net_id, f"net_{partner.net_id}")
                                print(f"    Moved {p_name} to jogged path on {p_layer} (diff-pair partner)")

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
                                    track_width, clearance, no_inner_top_layer,
                                    None, None, None, None
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


def _rebuild_track_for_route(tracks: List[Dict], route: 'FanoutRoute',
                             track_width: float) -> None:
    """Replace this route's tracks in `tracks` with freshly generated segments.

    Removes existing track dicts for the route's net_id and re-adds segments
    derived from the (possibly modified) route geometry on its current layer.
    """
    net_id = route.net_id
    old = [i for i, t in enumerate(tracks) if t.get('net_id') == net_id]
    for idx in sorted(old, reverse=True):
        tracks.pop(idx)
    for seg in route_segments(route):
        tracks.append(create_track(seg['start'], seg['end'], track_width,
                                   route.layer, net_id, route.pair_id))


def _tracks_collision_free(segments: List[Dict], layer: str, net_id: int,
                           pair_id, tracks: List[Dict],
                           existing_tracks: List[Dict],
                           min_spacing: float) -> bool:
    """True if `segments` on `layer` clear all other-net tracks (new + existing)."""
    all_other = [t for t in tracks if t.get('net_id') != net_id] + (existing_tracks or [])
    for seg in segments:
        for other in all_other:
            if other['layer'] != layer:
                continue
            if other.get('net_id') == net_id:
                continue
            if pair_id and other.get('pair_id') == pair_id:
                continue
            if check_segment_collision(seg['start'], seg['end'],
                                       other['start'], other['end'], min_spacing):
                return False
    return True


def repair_pad_crossings(routes: List[FanoutRoute], tracks: List[Dict],
                         available_layers: List[str], track_width: float,
                         clearance: float, diff_pair_spacing: float,
                         existing_tracks: List[Dict],
                         grid: BGAGrid, channels: List[Channel],
                         exit_margin: float, net_names: Dict[int, str],
                         no_inner_top_layer: bool,
                         obstacles, cfg, layer_map: Dict[str, int],
                         foreign_pads=None) -> Tuple[int, List[str]]:
    """Reroute fanout routes whose copper crosses a foreign pad/track/via.

    Reuses the obstacle map: a route is "bad" if route_clear_of_pads is False.
    Resolution order, reusing the existing jog/reroute machinery:
      1. Move the route (and its diff-pair partner, kept on the same layer) to a
         layer where it is both pad-clear and track-collision-free.
      2. Single-ended only: try an alternate channel, then a jogged farther
         channel (both helpers already reject pad-crossing candidates).
    Routes that can't be made clear are removed and reported as failed.

    Returns (num_repaired, list_of_failed_net_names).
    """
    if obstacles is None or cfg is None or layer_map is None:
        return 0, []
    if existing_tracks is None:
        existing_tracks = []
    if net_names is None:
        net_names = {}

    min_spacing = track_width + clearance
    repaired = 0
    failed_nets: List[str] = []

    candidate_layers = available_layers
    if no_inner_top_layer and len(available_layers) > 1:
        candidate_layers = available_layers[1:]

    # Group bad routes; handle each diff pair once (both legs together).
    # TRIGGER on a true short (raw pad-overlap) only, matching the DRC/checker
    # semantics. The clearance-padded obstacle map is used only to pick clean
    # ALTERNATIVES (jog/reroute candidate avoidance) - using it as the trigger
    # would drop legally-spaced routes that merely sit within clearance of a pad.
    bad = [r for r in routes if not route_clear_of_foreign_pads(r, foreign_pads, layer_map)]
    handled_pairs: Set[str] = set()

    for route in list(bad):
        if route not in routes:
            continue  # already removed (e.g. as a partner)
        pair_id = route.pair_id

        # ---- Diff pair: try to move both legs to a clean shared layer ----
        if pair_id is not None:
            if pair_id in handled_pairs:
                continue
            handled_pairs.add(pair_id)
            legs = [r for r in routes if r.pair_id == pair_id]
            orig_layer = route.layer

            # 0. Cheapest fix: drop the decorative exit jog on any leg whose only
            #    crossing is that tail. If that clears the whole pair, done.
            for leg in legs:
                if not route_clear_of_foreign_pads(leg, foreign_pads, layer_map):
                    _route_jog_clears_pads(leg, obstacles, cfg, layer_map, foreign_pads)
            if all(route_clear_of_foreign_pads(leg, foreign_pads, layer_map)
                   for leg in legs):
                for leg in legs:
                    _rebuild_track_for_route(tracks, leg, track_width)
                repaired += 1
                print(f"    Pad-aware: trimmed exit jog on diff pair {pair_id}")
                continue

            # Rank candidate layers: prefer those that also satisfy full
            # clearance against the shared obstacle map (cleaner, fewer DRC
            # nits), then fall back to layers that merely remove the true pad
            # short and are track-collision-free.
            def _pair_layer_ok(layer, strict):
                for leg in legs:
                    segs = route_segments(leg)
                    if not segments_clear_of_foreign_pads(segs, layer, leg.net_id,
                                                          foreign_pads, layer_map):
                        return False
                    if strict and not segments_clear_of_pads(segs, layer, obstacles,
                                                             cfg, layer_map):
                        return False
                    if not _tracks_collision_free(segs, layer, leg.net_id, pair_id,
                                                  tracks, existing_tracks, min_spacing):
                        return False
                return True

            moved = False
            for strict in (True, False):
                for layer in candidate_layers:
                    if layer == orig_layer:
                        continue
                    if _pair_layer_ok(layer, strict):
                        for leg in legs:
                            leg.layer = layer
                            _rebuild_track_for_route(tracks, leg, track_width)
                        moved = True
                        repaired += 1
                        print(f"    Pad-aware: moved diff pair {pair_id} to {layer}")
                        break
                if moved:
                    break
            if not moved:
                for leg in legs:
                    name = net_names.get(leg.net_id, f"net_{leg.net_id}")
                    if name not in failed_nets:
                        failed_nets.append(name)
                    if leg in routes:
                        routes.remove(leg)
                    old = [i for i, t in enumerate(tracks) if t.get('net_id') == leg.net_id]
                    for idx in sorted(old, reverse=True):
                        tracks.pop(idx)
                print(f"    Pad-aware FAILED: diff pair {pair_id} crosses a pad on all layers")
            continue

        # ---- Single-ended route ----
        net_id = route.net_id
        resolved = False

        # 0. Cheapest fix: drop the decorative exit jog if that clears it.
        if _route_jog_clears_pads(route, obstacles, cfg, layer_map, foreign_pads):
            _rebuild_track_for_route(tracks, route, track_width)
            repaired += 1
            print(f"    Pad-aware: trimmed exit jog on {net_names.get(net_id, net_id)}")
            continue

        # 1. Layer swap on the same geometry. Prefer a layer that is also fully
        #    clearance-clean against the shared obstacle map, else fall back to
        #    one that just removes the true pad short + is collision-free.
        orig_layer = route.layer
        for strict in (True, False):
            for layer in candidate_layers:
                if layer == orig_layer:
                    continue
                segs = route_segments(route)
                if not segments_clear_of_foreign_pads(segs, layer, net_id,
                                                      foreign_pads, layer_map):
                    continue
                if strict and not segments_clear_of_pads(segs, layer, obstacles,
                                                         cfg, layer_map):
                    continue
                if not _tracks_collision_free(segs, layer, net_id, None,
                                              tracks, existing_tracks, min_spacing):
                    continue
                route.layer = layer
                _rebuild_track_for_route(tracks, route, track_width)
                resolved = True
                repaired += 1
                print(f"    Pad-aware: moved {net_names.get(net_id, net_id)} to {layer}")
                break
            if resolved:
                break

        # 2. Alternate channel (pad-aware helper).
        if not resolved and route.channel is not None:
            alt_channels = get_alternate_channels_for_pad(
                route.pad_pos[0], route.pad_pos[1],
                route.channel, route.escape_dir, channels)
            for alt_channel in alt_channels:
                result = try_reroute_single_ended(
                    route, alt_channel, grid, exit_margin,
                    tracks, existing_tracks, available_layers,
                    track_width, clearance, no_inner_top_layer,
                    obstacles, cfg, layer_map, foreign_pads)
                if result:
                    new_route, new_layer = result
                    routes[routes.index(route)] = new_route
                    route = new_route
                    _rebuild_track_for_route(tracks, route, track_width)
                    resolved = True
                    repaired += 1
                    print(f"    Pad-aware: rerouted {net_names.get(net_id, net_id)} "
                          f"to alternate channel on {new_layer}")
                    break

        # 3. Jogged farther channel (pad-aware helper).
        if not resolved and route.channel is not None:
            farther_ch = get_farther_channel(route, channels, grid)
            if farther_ch is not None:
                result = try_jogged_route(
                    route, farther_ch, grid, exit_margin,
                    tracks, existing_tracks, available_layers,
                    track_width, clearance, None, no_inner_top_layer,
                    obstacles, cfg, layer_map, foreign_pads)
                if result:
                    new_route, new_layer, _ = result
                    routes[routes.index(route)] = new_route
                    route = new_route
                    _rebuild_track_for_route(tracks, route, track_width)
                    resolved = True
                    repaired += 1
                    print(f"    Pad-aware: jogged {net_names.get(net_id, net_id)} "
                          f"to farther channel on {new_layer}")

        # 4. Give up - remove and report.
        if not resolved:
            name = net_names.get(net_id, f"net_{net_id}")
            failed_nets.append(name)
            if route in routes:
                routes.remove(route)
            old = [i for i, t in enumerate(tracks) if t.get('net_id') == net_id]
            for idx in sorted(old, reverse=True):
                tracks.pop(idx)
            print(f"    Pad-aware FAILED: {name} crosses a pad, no clear path")

    return repaired, failed_nets
