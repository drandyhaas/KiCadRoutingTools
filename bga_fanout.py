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
import re
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict
import fnmatch

from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb
from routing_utils import extract_diff_pair_base


@dataclass
class DiffPairPads:
    """A differential pair of pads (P and N) - tracks pad objects."""
    base_name: str  # Common name without _P/_N suffix
    p_pad: Optional[Pad] = None
    n_pad: Optional[Pad] = None

    @property
    def is_complete(self) -> bool:
        return self.p_pad is not None and self.n_pad is not None


@dataclass
class BGAGrid:
    """Represents the BGA ball grid structure."""
    pitch_x: float
    pitch_y: float
    rows: List[float]  # Sorted Y positions
    cols: List[float]  # Sorted X positions
    center_x: float
    center_y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass
class Channel:
    """A routing channel between ball rows/columns."""
    orientation: str  # 'horizontal' or 'vertical'
    position: float   # Y for horizontal, X for vertical
    index: int


@dataclass
class FanoutRoute:
    """Complete fanout: pad -> 45° stub -> channel -> exit -> jog."""
    pad: Pad
    pad_pos: Tuple[float, float]
    stub_end: Tuple[float, float]  # Where stub meets channel (or exit for edge pads)
    exit_pos: Tuple[float, float]  # Where route exits BGA
    jog_end: Tuple[float, float] = None  # End of 45° jog after exit
    jog_extension: Tuple[float, float] = None  # Extension point for outside track of diff pair
    channel_point: Tuple[float, float] = None  # First channel point for half-edge inner pads (45° entry)
    channel_point2: Tuple[float, float] = None  # Second channel point (after horizontal segment)
    pre_channel_jog: Tuple[float, float] = None  # Jog point before channel (for jogged routes to farther channel)
    channel: Optional[Channel] = None  # None for edge pads with direct escape
    escape_dir: str = ''  # 'left', 'right', 'up', 'down'
    is_edge: bool = False  # True for outer row/column pads
    layer: str = "F.Cu"
    pair_id: Optional[str] = None  # Differential pair base name if part of a pair
    is_p: bool = True  # True for P, False for N in differential pair

    @property
    def net_id(self) -> int:
        return self.pad.net_id


def find_differential_pairs(footprint: Footprint,
                           diff_pair_patterns: List[str]) -> Dict[str, DiffPairPads]:
    """
    Find all differential pairs in a footprint matching the given patterns.

    Args:
        footprint: The footprint to search
        diff_pair_patterns: Glob patterns for nets to treat as diff pairs

    Returns:
        Dict mapping base_name to DiffPairPads
    """
    pairs: Dict[str, DiffPairPads] = {}

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        # Check if this net matches any diff pair pattern
        matched = any(fnmatch.fnmatch(pad.net_name, pattern)
                     for pattern in diff_pair_patterns)
        if not matched:
            continue

        # Try to extract diff pair info
        result = extract_diff_pair_base(pad.net_name)
        if result is None:
            continue

        base_name, is_p = result

        if base_name not in pairs:
            pairs[base_name] = DiffPairPads(base_name=base_name)

        if is_p:
            pairs[base_name].p_pad = pad
        else:
            pairs[base_name].n_pad = pad

    # Filter to only complete pairs
    complete_pairs = {k: v for k, v in pairs.items() if v.is_complete}

    return complete_pairs


def analyze_bga_grid(footprint: Footprint) -> Optional[BGAGrid]:
    """Analyze a footprint to extract BGA grid parameters."""
    pads = footprint.pads
    if len(pads) < 4:
        return None

    x_positions = sorted(set(p.global_x for p in pads))
    y_positions = sorted(set(p.global_y for p in pads))

    if len(x_positions) < 2 or len(y_positions) < 2:
        return None

    x_diffs = [x_positions[i+1] - x_positions[i] for i in range(len(x_positions)-1)]
    y_diffs = [y_positions[i+1] - y_positions[i] for i in range(len(y_positions)-1)]

    def get_dominant_pitch(diffs):
        if not diffs:
            return None
        rounded = [round(d, 2) for d in diffs]
        counts = defaultdict(int)
        for d in rounded:
            counts[d] += 1
        if not counts:
            return None
        dominant = max(counts.keys(), key=lambda k: counts[k])
        if counts[dominant] < len(diffs) * 0.5:
            return None
        return dominant

    pitch_x = get_dominant_pitch(x_diffs)
    pitch_y = get_dominant_pitch(y_diffs)

    if pitch_x is None or pitch_y is None:
        return None

    return BGAGrid(
        pitch_x=pitch_x,
        pitch_y=pitch_y,
        rows=y_positions,
        cols=x_positions,
        center_x=(min(x_positions) + max(x_positions)) / 2,
        center_y=(min(y_positions) + max(y_positions)) / 2,
        min_x=min(x_positions) - pitch_x / 2,
        max_x=max(x_positions) + pitch_x / 2,
        min_y=min(y_positions) - pitch_y / 2,
        max_y=max(y_positions) + pitch_y / 2
    )


def calculate_channels(grid: BGAGrid) -> List[Channel]:
    """Calculate routing channels between ball rows and columns."""
    channels = []
    idx = 0

    for i in range(len(grid.rows) - 1):
        y_pos = (grid.rows[i] + grid.rows[i + 1]) / 2
        channels.append(Channel(orientation='horizontal', position=y_pos, index=idx))
        idx += 1

    for i in range(len(grid.cols) - 1):
        x_pos = (grid.cols[i] + grid.cols[i + 1]) / 2
        channels.append(Channel(orientation='vertical', position=x_pos, index=idx))
        idx += 1

    return channels


def is_edge_pad(pad_x: float, pad_y: float, grid: BGAGrid, tolerance: float = 0.01) -> Tuple[bool, str]:
    """
    Check if a pad is on the outer edge of the BGA.
    Returns (is_edge, escape_direction).

    Edge pads are on the outermost row or column and can escape directly
    without needing a 45° stub.
    """
    on_left = abs(pad_x - grid.cols[0]) < tolerance
    on_right = abs(pad_x - grid.cols[-1]) < tolerance
    on_top = abs(pad_y - grid.rows[0]) < tolerance
    on_bottom = abs(pad_y - grid.rows[-1]) < tolerance

    # Determine escape direction based on which edge
    if on_right:
        return True, 'right'
    elif on_left:
        return True, 'left'
    elif on_bottom:
        return True, 'down'
    elif on_top:
        return True, 'up'

    return False, ''


def find_escape_channel(pad_x: float, pad_y: float,
                        grid: BGAGrid,
                        channels: List[Channel],
                        force_orientation: str = None) -> Tuple[Optional[Channel], str]:
    """
    Find the best channel for a pad to escape through.
    Returns (channel, direction). Channel is None for edge pads.

    Args:
        pad_x, pad_y: Pad position
        grid: BGA grid info
        channels: Available routing channels
        force_orientation: If set to 'horizontal' or 'vertical', only allow
                          escapes in that orientation (left/right or up/down)
    """
    # Check if this is an edge pad first
    is_edge, edge_dir = is_edge_pad(pad_x, pad_y, grid)
    if is_edge:
        return None, edge_dir

    dist_left = pad_x - grid.min_x
    dist_right = grid.max_x - pad_x
    dist_up = pad_y - grid.min_y
    dist_down = grid.max_y - pad_y

    # Build list of (distance, direction, orientation) options
    options = [
        (dist_left, 'left', 'horizontal'),
        (dist_right, 'right', 'horizontal'),
        (dist_up, 'up', 'vertical'),
        (dist_down, 'down', 'vertical'),
    ]

    # Filter to forced orientation if specified
    if force_orientation:
        options = [(d, dir, o) for d, dir, o in options if o == force_orientation]

    # Sort by distance (closest first)
    options.sort(key=lambda x: x[0])

    # Pick the best option
    for dist, escape_dir, orientation in options:
        if orientation == 'horizontal':
            h_channels = [c for c in channels if c.orientation == 'horizontal']
            if h_channels:
                best = min(h_channels, key=lambda c: abs(c.position - pad_y))
                return best, escape_dir
        else:  # vertical
            v_channels = [c for c in channels if c.orientation == 'vertical']
            if v_channels:
                best = min(v_channels, key=lambda c: abs(c.position - pad_x))
                return best, escape_dir

    # Fallback: return closest edge direction even without channel
    min_dist = min(dist_left, dist_right, dist_up, dist_down)
    if min_dist == dist_right:
        return None, 'right'
    elif min_dist == dist_left:
        return None, 'left'
    elif min_dist == dist_down:
        return None, 'down'
    else:
        return None, 'up'


def get_pair_escape_options(p_pad_x: float, p_pad_y: float,
                             n_pad_x: float, n_pad_y: float,
                             grid: BGAGrid,
                             channels: List[Channel],
                             include_alternate_channels: bool = False) -> List[Tuple[Optional[Channel], str]]:
    """
    Get all valid escape options for a differential pair, ordered by preference.

    Returns list of (channel, direction) tuples, best options first.
    Returns empty list for edge/half-edge pairs (they have fixed escape).

    Args:
        include_alternate_channels: If True, include neighboring channels as fallback options
    """
    center_x = (p_pad_x + n_pad_x) / 2
    center_y = (p_pad_y + n_pad_y) / 2

    # Check if this is an edge pair - no options, fixed escape
    is_edge_p, _ = is_edge_pad(p_pad_x, p_pad_y, grid)
    is_edge_n, _ = is_edge_pad(n_pad_x, n_pad_y, grid)
    if is_edge_p or is_edge_n:
        return []  # Edge pairs have fixed escape direction

    # Calculate distances to each edge
    dist_left = center_x - grid.min_x
    dist_right = grid.max_x - center_x
    dist_up = center_y - grid.min_y
    dist_down = grid.max_y - center_y

    min_pad_y = min(p_pad_y, n_pad_y)
    max_pad_y = max(p_pad_y, n_pad_y)
    min_pad_x = min(p_pad_x, n_pad_x)
    max_pad_x = max(p_pad_x, n_pad_x)

    options = []

    # Horizontal escape options (left/right)
    h_channels = [c for c in channels if c.orientation == 'horizontal']
    channels_above = [c for c in h_channels if c.position < min_pad_y]
    channels_below = [c for c in h_channels if c.position > max_pad_y]

    # Best horizontal channel
    if dist_up <= dist_down and channels_above:
        h_channel = max(channels_above, key=lambda c: c.position)
    elif channels_below:
        h_channel = min(channels_below, key=lambda c: c.position)
    elif channels_above:
        h_channel = max(channels_above, key=lambda c: c.position)
    else:
        h_channel = min(h_channels, key=lambda c: abs(c.position - center_y)) if h_channels else None

    if h_channel:
        h_dir = 'left' if dist_left <= dist_right else 'right'
        h_dist = min(dist_left, dist_right)
        options.append((h_channel, h_dir, h_dist))

    # Vertical escape options (up/down)
    v_channels = [c for c in channels if c.orientation == 'vertical']
    channels_left = [c for c in v_channels if c.position < min_pad_x]
    channels_right = [c for c in v_channels if c.position > max_pad_x]

    # Best vertical channel
    if dist_left <= dist_right and channels_left:
        v_channel = max(channels_left, key=lambda c: c.position)
    elif channels_right:
        v_channel = min(channels_right, key=lambda c: c.position)
    elif channels_left:
        v_channel = max(channels_left, key=lambda c: c.position)
    else:
        v_channel = min(v_channels, key=lambda c: abs(c.position - center_x)) if v_channels else None

    if v_channel:
        v_dir = 'up' if dist_up <= dist_down else 'down'
        v_dist = min(dist_up, dist_down)
        options.append((v_channel, v_dir, v_dist))

    # Sort by distance (closest edge first)
    options.sort(key=lambda x: x[2])

    # Build result list
    result = [(ch, d) for ch, d, _ in options]

    # If requested, add alternate channels as fallback options
    # IMPORTANT: Alternate channels must be in the OPPOSITE direction from track travel
    # e.g., if tracks go RIGHT to reach channel, alternate must be to the LEFT of pads
    # (so tracks can still reach it going right)
    if include_alternate_channels:
        # Add alternate vertical channels
        if v_channel:
            v_dir = 'up' if dist_up <= dist_down else 'down'
            # If primary channel is to the right, tracks go right - alternate must be to LEFT
            # If primary channel is to the left, tracks go left - alternate must be to RIGHT
            if v_channel.position > center_x:
                # Primary is right of pads - only consider channels to the LEFT as alternates
                alt_candidates = [c for c in v_channels if c != v_channel and c.position < center_x]
            else:
                # Primary is left of pads - only consider channels to the RIGHT as alternates
                alt_candidates = [c for c in v_channels if c != v_channel and c.position > center_x]
            # Sort by closest to pads (furthest into the alternate direction)
            if v_channel.position > center_x:
                alt_candidates.sort(key=lambda c: c.position, reverse=True)  # Closest to pads from left
            else:
                alt_candidates.sort(key=lambda c: c.position)  # Closest to pads from right
            for alt_ch in alt_candidates[:2]:  # Add up to 2 alternates
                result.append((alt_ch, v_dir))

        # Add alternate horizontal channels
        if h_channel:
            h_dir = 'left' if dist_left <= dist_right else 'right'
            # If primary channel is above, tracks go up - alternate must be BELOW
            # If primary channel is below, tracks go down - alternate must be ABOVE
            if h_channel.position < center_y:
                # Primary is above pads - only consider channels BELOW as alternates
                alt_candidates = [c for c in h_channels if c != h_channel and c.position > center_y]
            else:
                # Primary is below pads - only consider channels ABOVE as alternates
                alt_candidates = [c for c in h_channels if c != h_channel and c.position < center_y]
            # Sort by closest to pads
            if h_channel.position < center_y:
                alt_candidates.sort(key=lambda c: c.position)  # Closest to pads from below
            else:
                alt_candidates.sort(key=lambda c: c.position, reverse=True)  # Closest to pads from above
            for alt_ch in alt_candidates[:2]:  # Add up to 2 alternates
                result.append((alt_ch, h_dir))

    return result


def get_alternate_channels_for_pad(pad_x: float, pad_y: float,
                                    primary_channel: Channel,
                                    escape_dir: str,
                                    channels: List[Channel]) -> List[Channel]:
    """
    Get alternate channel for a single-ended signal.

    Returns the ONE channel on the OPPOSITE side of the pad from the primary channel.
    This means the 45° stub will go in the opposite direction.

    For example:
    - If escape_dir is 'right' and primary_channel is BELOW the pad (stub goes 45° down),
      the alternate is the channel ABOVE the pad (stub goes 45° up instead).
    - Only one alternate is returned (the closest channel on the opposite side).
    """
    if primary_channel is None:
        return []

    if primary_channel.orientation == 'horizontal':
        # Horizontal channel used for left/right escape
        # Get other horizontal channels on the OPPOSITE side of the pad
        h_channels = [c for c in channels if c.orientation == 'horizontal' and c != primary_channel]

        if primary_channel.position < pad_y:
            # Primary is ABOVE pad (stub goes up to reach it)
            # Alternate must be BELOW pad (stub goes down instead)
            candidates = [c for c in h_channels if c.position > pad_y]
            # Sort by closest to pad first (smallest Y greater than pad_y)
            candidates.sort(key=lambda c: c.position)
        else:
            # Primary is BELOW pad (stub goes down to reach it)
            # Alternate must be ABOVE pad (stub goes up instead)
            candidates = [c for c in h_channels if c.position < pad_y]
            # Sort by closest to pad first (largest Y less than pad_y)
            candidates.sort(key=lambda c: c.position, reverse=True)

        # Only return the ONE closest alternate on the opposite side
        return candidates[:1]

    else:  # vertical
        # Vertical channel used for up/down escape
        # Get other vertical channels on the OPPOSITE side of the pad
        v_channels = [c for c in channels if c.orientation == 'vertical' and c != primary_channel]

        if primary_channel.position < pad_x:
            # Primary is LEFT of pad (stub goes left to reach it)
            # Alternate must be RIGHT of pad (stub goes right instead)
            candidates = [c for c in v_channels if c.position > pad_x]
            # Sort by closest to pad first
            candidates.sort(key=lambda c: c.position)
        else:
            # Primary is RIGHT of pad (stub goes right to reach it)
            # Alternate must be LEFT of pad (stub goes left instead)
            candidates = [c for c in v_channels if c.position < pad_x]
            # Sort by closest to pad first
            candidates.sort(key=lambda c: c.position, reverse=True)

        # Only return the ONE closest alternate on the opposite side
        return candidates[:1]


def try_reroute_single_ended(route: 'FanoutRoute',
                              alternate_channel: Channel,
                              grid: BGAGrid,
                              exit_margin: float,
                              tracks: List[Dict],
                              existing_tracks: List[Dict],
                              available_layers: List[str],
                              track_width: float,
                              clearance: float) -> Optional[Tuple['FanoutRoute', str]]:
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

    # Check if this is an inner route (should avoid F.Cu)
    is_edge, _ = is_edge_pad(pad_x, pad_y, grid)
    if is_edge:
        candidate_layers = available_layers
    else:
        candidate_layers = available_layers[1:] if len(available_layers) > 1 else available_layers

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
                     jog_length: float = None) -> Optional[Tuple['FanoutRoute', str, List[Dict]]]:
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

    # Check if this is an inner route (should avoid F.Cu)
    is_edge, _ = is_edge_pad(pad_x, pad_y, grid)
    if is_edge:
        candidate_layers = available_layers
    else:
        candidate_layers = available_layers[1:] if len(available_layers) > 1 else available_layers

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
                {
                    'start': route.pad_pos,
                    'end': stub_end,
                    'width': track_width,
                    'layer': layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                },
                {
                    'start': stub_end,
                    'end': jog_point,
                    'width': track_width,
                    'layer': layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                },
                {
                    'start': jog_point,
                    'end': exit_pos,
                    'width': track_width,
                    'layer': layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                },
                {
                    'start': exit_pos,
                    'end': jog_end,
                    'width': track_width,
                    'layer': layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                }
            ]

            return new_route, layer, new_tracks

    return None


def find_existing_fanouts(pcb_data: PCBData, footprint: Footprint,
                          grid: BGAGrid, channels: List[Channel],
                          tolerance: float = 0.05) -> Tuple[Set[int], Dict[Tuple[str, str, float], str]]:
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


def find_diff_pair_escape(p_pad_x: float, p_pad_y: float,
                          n_pad_x: float, n_pad_y: float,
                          grid: BGAGrid,
                          channels: List[Channel],
                          preferred_orientation: str = 'auto') -> Tuple[Optional[Channel], str]:
    """
    Find the best escape channel for a differential pair.

    Args:
        preferred_orientation: 'horizontal', 'vertical', or 'auto'
            - 'horizontal': prefer left/right escape
            - 'vertical': prefer up/down escape
            - 'auto': choose based on closest edge

    Returns (channel, direction). Channel is None for edge pads.
    """
    # Calculate pair center and orientation
    center_x = (p_pad_x + n_pad_x) / 2
    center_y = (p_pad_y + n_pad_y) / 2

    # Check if this is an edge pair
    is_edge_p, edge_dir_p = is_edge_pad(p_pad_x, p_pad_y, grid)
    is_edge_n, edge_dir_n = is_edge_pad(n_pad_x, n_pad_y, grid)

    if is_edge_p and is_edge_n:
        # Both pads on edge - use the common edge direction
        return None, edge_dir_p

    # Check for "half-edge" pair: one pad on edge, one inner
    if is_edge_p or is_edge_n:
        edge_dir = edge_dir_p if is_edge_p else edge_dir_n
        return None, f'half_edge_{edge_dir}'

    # Get all escape options
    options = get_pair_escape_options(p_pad_x, p_pad_y, n_pad_x, n_pad_y, grid, channels)

    if not options:
        # Fallback - shouldn't happen for inner pairs
        return None, 'right'

    if preferred_orientation == 'auto':
        # Use first (best) option
        return options[0]
    elif preferred_orientation == 'horizontal':
        # Prefer horizontal (left/right)
        for ch, d in options:
            if d in ['left', 'right']:
                return ch, d
        return options[0]  # Fallback
    elif preferred_orientation == 'vertical':
        # Prefer vertical (up/down)
        for ch, d in options:
            if d in ['up', 'down']:
                return ch, d
        return options[0]  # Fallback
    else:
        return options[0]


def assign_pair_escapes(diff_pairs: Dict[str, DiffPairPads],
                        grid: BGAGrid,
                        channels: List[Channel],
                        layers: List[str],
                        primary_orientation: str = 'horizontal',
                        track_width: float = 0.1,
                        clearance: float = 0.1,
                        diff_pair_gap: float = 0.1,
                        via_size: float = 0.3,
                        rebalance: bool = False,
                        pre_occupied: Dict[Tuple[str, str, float], str] = None,
                        force_escape_direction: bool = False) -> Dict[str, Tuple[Optional[Channel], str]]:
    """
    Assign escape directions to all differential pairs, avoiding overlaps.

    Strategy:
    1. Sort pairs by distance to nearest edge (closest first - they have fewer options)
    2. Try primary orientation first for each pair
    3. If channel/layer is occupied, try secondary orientation
    4. Track channel occupancy per layer
    5. If rebalance=True, try to balance if one direction is overpopulated

    Args:
        diff_pairs: Dictionary of pair_id -> DiffPairPads
        grid: BGA grid info
        channels: Available channels
        layers: Available routing layers
        primary_orientation: 'horizontal' or 'vertical'
        track_width: Track width for spacing calculation
        clearance: Clearance for spacing calculation
        rebalance: If True, rebalance directions to achieve even H/V mix
        pre_occupied: Dict of already-occupied exit positions from existing fanouts
        force_escape_direction: If True, only use primary orientation (no fallback)

    Returns:
        Dictionary of pair_id -> (channel, escape_direction)
    """
    # Track which exit positions are used per layer
    # Key: (layer, exit_x or exit_y rounded to 0.1mm)
    # For horizontal escape: track exit Y positions
    # For vertical escape: track exit X positions
    used_exits: Dict[Tuple[str, str, float], Set[str]] = defaultdict(set)  # (layer, 'h'/'v', pos) -> set of pair_ids

    pair_spacing = track_width * 2 + clearance  # Space needed for a diff pair

    assignments: Dict[str, Tuple[Optional[Channel], str]] = {}

    # Track occupied exit positions per layer
    # Key: (layer, direction_axis, position) where position is the channel/exit coordinate
    # Must be initialized BEFORE processing edge pairs so they get tracked
    # Start with pre-occupied positions from existing fanouts
    occupied: Dict[Tuple[str, str, float], str] = dict(pre_occupied) if pre_occupied else {}

    pair_spacing = track_width * 2 + clearance  # Minimum spacing between diff pairs
    edge_layer = layers[0]  # Edge pairs go on top layer (F.Cu)

    # Calculate which directions require adjacent-channel routing
    # half_pair_spacing is the offset from channel center for each track
    half_pair_spacing = (track_width + diff_pair_gap) / 2
    via_radius = via_size / 2

    # max_offset is the maximum track offset that fits between vias
    # For horizontal escape (left/right), tracks are in horizontal channels, constrained by pitch_y
    # For vertical escape (up/down), tracks are in vertical channels, constrained by pitch_x
    max_offset_h = grid.pitch_y / 2 - via_radius - track_width / 2 - clearance
    max_offset_v = grid.pitch_x / 2 - via_radius - track_width / 2 - clearance

    # Determine if each direction needs adjacent channels
    horizontal_needs_adjacent = half_pair_spacing > max_offset_h
    vertical_needs_adjacent = half_pair_spacing > max_offset_v

    # Collect pair info with distances
    pair_info = []
    for pair_id, pair in diff_pairs.items():
        if not pair.is_complete:
            continue
        p_pad = pair.p_pad
        n_pad = pair.n_pad
        center_x = (p_pad.global_x + n_pad.global_x) / 2
        center_y = (p_pad.global_y + n_pad.global_y) / 2

        # Check if edge pair
        is_edge_p, edge_dir_p = is_edge_pad(p_pad.global_x, p_pad.global_y, grid)
        is_edge_n, edge_dir_n = is_edge_pad(n_pad.global_x, n_pad.global_y, grid)

        if is_edge_p and is_edge_n:
            # Both on edge - fixed assignment on edge_layer
            edge_dir = edge_dir_p
            assignments[pair_id] = (None, edge_dir)
            # Track occupancy - edge pairs exit at their center position
            if edge_dir in ['left', 'right']:
                # Horizontal exit - track Y position (center_y)
                key = (edge_layer, f'{edge_dir}_h', round(center_y, 1))
            else:
                # Vertical exit - track X position (center_x)
                key = (edge_layer, f'{edge_dir}_v', round(center_x, 1))
            occupied[key] = pair_id
            continue
        elif is_edge_p or is_edge_n:
            # Half-edge - fixed assignment on edge_layer
            edge_dir = edge_dir_p if is_edge_p else edge_dir_n
            assignments[pair_id] = (None, f'half_edge_{edge_dir}')
            # Track occupancy for half-edge pairs too
            if edge_dir in ['left', 'right']:
                key = (edge_layer, f'{edge_dir}_h', round(center_y, 1))
            else:
                key = (edge_layer, f'{edge_dir}_v', round(center_x, 1))
            occupied[key] = pair_id
            continue

        # Calculate distance to nearest edge
        dist_left = center_x - grid.min_x
        dist_right = grid.max_x - center_x
        dist_up = center_y - grid.min_y
        dist_down = grid.max_y - center_y
        min_dist = min(dist_left, dist_right, dist_up, dist_down)

        pair_info.append((pair_id, pair, min_dist, center_x, center_y))

    secondary_orientation = 'vertical' if primary_orientation == 'horizontal' else 'horizontal'

    def get_exit_key(pair: DiffPairPads, channel: Channel, escape_dir: str) -> Tuple[str, float]:
        """Get the exit position key for a pair assignment.

        Returns (direction_and_axis, position) where direction_and_axis encodes
        both the escape direction and axis to prevent false conflicts.
        E.g., 'up_v' vs 'down_v' are different even at same X position.
        """
        p_pad = pair.p_pad
        n_pad = pair.n_pad
        cx = (p_pad.global_x + n_pad.global_x) / 2
        cy = (p_pad.global_y + n_pad.global_y) / 2

        pads_horizontal = abs(p_pad.global_x - n_pad.global_x) > abs(p_pad.global_y - n_pad.global_y)
        is_cross = (pads_horizontal and escape_dir in ['up', 'down']) or \
                   (not pads_horizontal and escape_dir in ['left', 'right'])

        if is_cross:
            # Convergence - exits at pair center
            if escape_dir in ['up', 'down']:
                return (f'{escape_dir}_v', round(cx, 1))
            else:
                return (f'{escape_dir}_h', round(cy, 1))
        else:
            # Channel routing - exits at channel position
            if escape_dir in ['left', 'right']:
                return (f'{escape_dir}_h', round(channel.position, 1))
            else:
                return (f'{escape_dir}_v', round(channel.position, 1))

    def can_assign(pair: DiffPairPads, channel: Channel, escape_dir: str) -> Tuple[bool, Optional[str]]:
        """Check if a pair can be assigned without overlap. Returns (can_assign, best_layer)."""
        if channel is None:
            return False, None

        pos_key, pos = get_exit_key(pair, channel, escape_dir)

        # Inner pairs (using channels) should NOT use F.Cu (first layer) to avoid
        # clearance violations with pads on the top layer. Only edge pairs use F.Cu.
        inner_layers = layers[1:] if len(layers) > 1 else layers

        # Check each layer for availability
        for layer in inner_layers:
            is_free = True
            # Check this position and nearby positions for spacing
            for delta in [-pair_spacing, -pair_spacing/2, 0, pair_spacing/2, pair_spacing]:
                check_pos = round(pos + delta, 1)
                key = (layer, pos_key, check_pos)
                if key in occupied:
                    is_free = False
                    break
            if is_free:
                return True, layer

        return False, None

    def do_assign(pair_id: str, pair: DiffPairPads, channel: Channel, escape_dir: str, layer: str):
        """Record an assignment."""
        assignments[pair_id] = (channel, escape_dir)
        pos_key, pos = get_exit_key(pair, channel, escape_dir)
        key = (layer, pos_key, pos)
        occupied[key] = pair_id

    # Sort pairs by distance to primary exit edge (closest first - they must use primary)
    # For horizontal primary: sort by distance to left/right edge
    # For vertical primary: sort by distance to top/bottom edge
    for pair_id, pair, min_dist, cx, cy in pair_info:
        if primary_orientation == 'horizontal':
            primary_dist = min(cx - grid.min_x, grid.max_x - cx)
        else:
            primary_dist = min(cy - grid.min_y, grid.max_y - cy)
        # Store primary_dist for later use
        pair_info_dict = {p[0]: (p[1], p[3], p[4], primary_dist) for p in pair_info}

    # Rebuild pair_info with primary distance
    pair_info_with_primary = []
    for pair_id, pair, min_dist, cx, cy in pair_info:
        if primary_orientation == 'horizontal':
            primary_dist = min(cx - grid.min_x, grid.max_x - cx)
            secondary_dist = min(cy - grid.min_y, grid.max_y - cy)
        else:
            primary_dist = min(cy - grid.min_y, grid.max_y - cy)
            secondary_dist = min(cx - grid.min_x, grid.max_x - cx)
        pair_info_with_primary.append((pair_id, pair, cx, cy, primary_dist, secondary_dist))

    # Sort by primary distance (closest to primary edge first)
    pair_info_with_primary.sort(key=lambda x: x[4])

    # Check if we should prefer secondary direction for diff pairs
    # If primary needs adjacent channels but secondary doesn't, prefer secondary
    primary_needs_adjacent = horizontal_needs_adjacent if primary_orientation == 'horizontal' else vertical_needs_adjacent
    secondary_needs_adjacent = vertical_needs_adjacent if primary_orientation == 'horizontal' else horizontal_needs_adjacent
    prefer_secondary_for_fit = (primary_needs_adjacent and not secondary_needs_adjacent and not force_escape_direction)

    if prefer_secondary_for_fit:
        print(f"  Note: Primary direction ({primary_orientation}) requires adjacent-channel routing, "
              f"but secondary ({secondary_orientation}) can fit both tracks - preferring secondary where possible")

    # First pass: assign as many as possible to preferred direction
    unassigned = []
    for pair_id, pair, cx, cy, primary_dist, secondary_dist in pair_info_with_primary:
        # Get all escape options including alternate channels
        all_options = get_pair_escape_options(
            pair.p_pad.global_x, pair.p_pad.global_y,
            pair.n_pad.global_x, pair.n_pad.global_y,
            grid, channels, include_alternate_channels=True
        )

        primary_dirs = ['up', 'down'] if primary_orientation == 'vertical' else ['left', 'right']
        secondary_dirs = ['up', 'down'] if secondary_orientation == 'vertical' else ['left', 'right']

        # If preferring secondary for fit, try secondary first
        if prefer_secondary_for_fit:
            secondary_options = [(ch, d) for ch, d in all_options if d in secondary_dirs]
            assigned = False
            for channel, escape_dir in secondary_options:
                can_do, layer = can_assign(pair, channel, escape_dir)
                if can_do:
                    do_assign(pair_id, pair, channel, escape_dir, layer)
                    assigned = True
                    break
            if assigned:
                continue  # Successfully assigned to secondary, skip to next pair

        # Try primary orientation options
        primary_options = [(ch, d) for ch, d in all_options if d in primary_dirs]

        # Try each option in order until one succeeds
        assigned = False
        for channel, escape_dir in primary_options:
            can_do, layer = can_assign(pair, channel, escape_dir)
            if can_do:
                do_assign(pair_id, pair, channel, escape_dir, layer)
                assigned = True
                break

        if not assigned:
            unassigned.append((pair_id, pair, cx, cy, primary_dist, secondary_dist))

    # Second pass: assign remaining to secondary direction (with alternate channels)
    # Skip this pass if force_escape_direction is True
    still_unassigned = []
    if force_escape_direction:
        # When forcing primary direction only, skip secondary and go directly to force assignment
        still_unassigned = unassigned
    else:
        for pair_id, pair, cx, cy, primary_dist, secondary_dist in unassigned:
            # Get all escape options including alternate channels
            all_options = get_pair_escape_options(
                pair.p_pad.global_x, pair.p_pad.global_y,
                pair.n_pad.global_x, pair.n_pad.global_y,
                grid, channels, include_alternate_channels=True
            )

            # Filter to secondary orientation options
            secondary_dirs = ['up', 'down'] if secondary_orientation == 'vertical' else ['left', 'right']
            secondary_options = [(ch, d) for ch, d in all_options if d in secondary_dirs]

            # Try each option in order until one succeeds
            assigned = False
            for channel, escape_dir in secondary_options:
                can_do, layer = can_assign(pair, channel, escape_dir)
                if can_do:
                    do_assign(pair_id, pair, channel, escape_dir, layer)
                    assigned = True
                    break

            if not assigned:
                still_unassigned.append((pair_id, pair, cx, cy, primary_dist, secondary_dist))

    # Third pass: force assign remaining (collision unavoidable)
    for pair_id, pair, cx, cy, primary_dist, secondary_dist in still_unassigned:
        if force_escape_direction:
            # When forcing direction, use primary orientation forced assignment
            channel, escape_dir = find_diff_pair_escape(
                pair.p_pad.global_x, pair.p_pad.global_y,
                pair.n_pad.global_x, pair.n_pad.global_y,
                grid, channels, primary_orientation
            )
            assignments[pair_id] = (channel, escape_dir)
            print(f"    Warning: {pair_id} forced to {primary_orientation} (may have overlaps)")
        else:
            channel, escape_dir = find_diff_pair_escape(
                pair.p_pad.global_x, pair.p_pad.global_y,
                pair.n_pad.global_x, pair.n_pad.global_y,
                grid, channels, 'auto'
            )
            assignments[pair_id] = (channel, escape_dir)
            print(f"    Warning: {pair_id} forced assignment (may have overlaps)")

    # Count direction distribution (excluding edge pairs which are fixed)
    def count_directions():
        dir_counts = defaultdict(int)
        for pid, (ch, d) in assignments.items():
            if d.startswith('half_edge_'):
                continue  # Don't count edge pairs - they're fixed
            if ch is None:
                continue  # Don't count full edge pairs
            dir_counts[d] += 1
        h = dir_counts['left'] + dir_counts['right']
        v = dir_counts['up'] + dir_counts['down']
        return h, v, dir_counts

    h_count, v_count, dir_counts = count_directions()
    print(f"    Initial assignment: horizontal={h_count}, vertical={v_count}")

    # Only rebalance if flag is set
    if rebalance:
        # Try to balance by switching pairs from overpopulated to underpopulated direction
        # Priority: pairs farthest from primary exit edge, but closest to secondary exit edge
        total_switched = 0
        max_iterations = 50

        for iteration in range(max_iterations):
            h_count, v_count, _ = count_directions()

            # Target: roughly equal split
            total = h_count + v_count
            if total == 0:
                break

            target_each = total // 2

            # Determine which direction is overpopulated
            if h_count > v_count + 1:
                overpopulated = 'horizontal'
                underpopulated = 'vertical'
                excess = h_count - target_each
            elif v_count > h_count + 1:
                overpopulated = 'vertical'
                underpopulated = 'horizontal'
                excess = v_count - target_each
            else:
                break  # Already balanced

            if excess <= 0:
                break

            # Find switchable pairs in overpopulated direction
            # Score by: farthest from current (primary) exit, closest to alternative exit
            switchable = []
            for pair_id, pair, cx, cy, primary_dist, secondary_dist in pair_info_with_primary:
                if pair_id not in assignments:
                    continue
                ch, d = assignments[pair_id]
                if d.startswith('half_edge_'):
                    continue
                if ch is None:
                    continue

                current_is_horiz = d in ['left', 'right']
                if (overpopulated == 'horizontal') == current_is_horiz:
                    # Calculate distances for switching priority
                    if overpopulated == 'horizontal':
                        # Currently horizontal, would switch to vertical
                        current_exit_dist = min(cx - grid.min_x, grid.max_x - cx)
                        alt_exit_dist = min(cy - grid.min_y, grid.max_y - cy)
                    else:
                        # Currently vertical, would switch to horizontal
                        current_exit_dist = min(cy - grid.min_y, grid.max_y - cy)
                        alt_exit_dist = min(cx - grid.min_x, grid.max_x - cx)

                    # Score: high current_exit_dist (far from current exit) + low alt_exit_dist (close to alt exit)
                    # We want pairs that are far from primary exit but close to secondary exit
                    switch_score = current_exit_dist - alt_exit_dist
                    switchable.append((pair_id, pair, switch_score, alt_exit_dist, cx, cy))

            if not switchable:
                break

            # Sort by switch_score descending (best candidates first)
            # Best = farthest from current exit, closest to alternative exit
            switchable.sort(key=lambda x: -x[2])

            # Try to switch one pair at a time
            switched_this_round = 0
            for pair_id, pair, score, alt_dist, cx, cy in switchable:
                if switched_this_round >= 1:  # Switch one at a time to recheck balance
                    break

                # Remove from occupied tracking
                old_ch, old_d = assignments[pair_id]
                if old_ch is not None:
                    old_pos_key, old_pos = get_exit_key(pair, old_ch, old_d)
                    # Find and remove from occupied
                    for layer in layers:
                        key = (layer, old_pos_key, old_pos)
                        if key in occupied and occupied[key] == pair_id:
                            del occupied[key]
                            break

                # Try to assign to underpopulated direction
                channel, escape_dir = find_diff_pair_escape(
                    pair.p_pad.global_x, pair.p_pad.global_y,
                    pair.n_pad.global_x, pair.n_pad.global_y,
                    grid, channels, underpopulated
                )

                can_do, layer = can_assign(pair, channel, escape_dir)
                if can_do and channel is not None:
                    do_assign(pair_id, pair, channel, escape_dir, layer)
                    switched_this_round += 1
                    total_switched += 1
                else:
                    # Can't switch, restore old assignment
                    if old_ch is not None:
                        # Re-add to occupied
                        for layer in layers:
                            key = (layer, old_pos_key, old_pos)
                            if key not in occupied:
                                occupied[key] = pair_id
                                break

            if switched_this_round == 0:
                h_count, v_count, _ = count_directions()
                print(f"    Balancing stopped: no switchable pairs found (h={h_count}, v={v_count})")
                break

        if total_switched > 0:
            h_count, v_count, _ = count_directions()
            print(f"    Balanced: switched {total_switched} pairs, now horizontal={h_count}, vertical={v_count}")

    # Build layer assignments from occupied dict
    # Reverse lookup: find which layer each pair was assigned to
    pair_layers: Dict[str, str] = {}
    for (layer, pos_key, pos), pid in occupied.items():
        pair_layers[pid] = layer

    # Edge pairs all go on edge_layer
    for pair_id, (ch, d) in assignments.items():
        if ch is None:  # Edge or half-edge pair
            pair_layers[pair_id] = edge_layer

    return assignments, pair_layers


def create_45_stub(pad_x: float, pad_y: float,
                   channel: Channel,
                   escape_dir: str,
                   channel_offset: float = 0.0) -> Tuple[float, float]:
    """
    Create 45-degree stub from pad to channel.

    For differential pairs, the offset is applied AFTER calculating the 45° stub
    based on the channel center. This ensures both P and N travel the same
    horizontal/vertical distance before applying the parallel offset.

    Args:
        pad_x, pad_y: Pad position
        channel: Target channel
        escape_dir: Direction of escape
        channel_offset: Offset from channel center (for diff pairs)

    Returns:
        End position of the 45° stub
    """
    if channel.orientation == 'horizontal':
        # Target Y includes the offset
        target_y = channel.position + channel_offset
        # For true 45°, dx = |dy|
        dy_to_target = target_y - pad_y
        if escape_dir == 'right':
            dx = abs(dy_to_target)
        else:
            dx = -abs(dy_to_target)

        return (pad_x + dx, target_y)
    else:
        # Target X includes the offset
        target_x = channel.position + channel_offset
        # For true 45°, dy = |dx|
        dx_to_target = target_x - pad_x
        if escape_dir == 'down':
            dy = abs(dx_to_target)
        else:
            dy = -abs(dx_to_target)

        return (target_x, pad_y + dy)


def calculate_exit_point(stub_end: Tuple[float, float],
                         channel: Channel,
                         escape_dir: str,
                         grid: BGAGrid,
                         margin: float = 0.5,
                         channel_offset: float = 0.0) -> Tuple[float, float]:
    """Calculate where the route exits the BGA boundary."""
    if channel.orientation == 'horizontal':
        exit_y = channel.position + channel_offset
        if escape_dir == 'right':
            return (grid.max_x + margin, exit_y)
        else:
            return (grid.min_x - margin, exit_y)
    else:
        exit_x = channel.position + channel_offset
        if escape_dir == 'down':
            return (exit_x, grid.max_y + margin)
        else:
            return (exit_x, grid.min_y - margin)


def calculate_jog_end(exit_pos: Tuple[float, float],
                      escape_dir: str,
                      layer: str,
                      layers: List[str],
                      jog_length: float,
                      is_diff_pair: bool = False,
                      is_outside_track: bool = False,
                      pair_spacing: float = 0.0) -> Tuple[Tuple[float, float], Optional[Tuple[float, float]]]:
    """
    Calculate the end position of the 45° jog at the exit.

    For differential pairs, the outside track needs to extend further before
    bending to maintain constant spacing through the 45° turn.

    Jog direction depends on layer:
    - Top layer (F.Cu): 45° to the left (from perspective walking towards BGA edge)
    - Bottom layer (B.Cu): 45° to the right
    - Middle layers: linear interpolation

    Args:
        exit_pos: Starting point of jog
        escape_dir: Direction of escape ('left', 'right', 'up', 'down')
        layer: Current layer
        layers: List of all available layers
        jog_length: Length of the jog (distance from BGA edge to first pad row/col)
        is_diff_pair: Whether this is part of a differential pair
        is_outside_track: Whether this is the outside track of the pair (needs extension)
        pair_spacing: Spacing between P and N tracks

    Returns:
        (jog_end, extension_point) - extension_point is the intermediate point for outside tracks
    """
    # Calculate layer position: 0 = top (left jog), 1 = bottom (right jog)
    try:
        layer_idx = layers.index(layer)
    except ValueError:
        layer_idx = 0

    num_layers = len(layers)
    if num_layers <= 1:
        layer_factor = 0.0  # Default to left jog
    else:
        layer_factor = layer_idx / (num_layers - 1)  # 0 to 1

    # Jog angle: -1 = left, +1 = right (from perspective of walking towards edge)
    # layer_factor 0 (top) -> -1 (left)
    # layer_factor 1 (bottom) -> +1 (right)
    jog_direction = 2 * layer_factor - 1  # Maps 0->-1, 1->+1

    # Calculate jog components based on escape direction
    # At 45°, both components equal jog_length / sqrt(2)
    # Reduce by factor of 4 for a shorter angled segment at the end of each stub
    diag = jog_length / math.sqrt(2) / 4

    ex, ey = exit_pos
    extension_point = None

    # For differential pairs, outside track extends further before bending
    # To maintain constant perpendicular spacing through a 45° turn:
    # extension = pair_spacing * (sqrt(2) - 1) ≈ 0.414 * pair_spacing
    if is_diff_pair and is_outside_track:
        extension = pair_spacing * (math.sqrt(2) - 1)
        if escape_dir == 'right':
            extension_point = (ex + extension, ey)
            ex = ex + extension
        elif escape_dir == 'left':
            extension_point = (ex - extension, ey)
            ex = ex - extension
        elif escape_dir == 'down':
            extension_point = (ex, ey + extension)
            ey = ey + extension
        else:  # up
            extension_point = (ex, ey - extension)
            ey = ey - extension

    if escape_dir == 'right':
        # Walking right, left is up (-Y), right is down (+Y)
        jog_end = (ex + diag, ey + jog_direction * diag)
    elif escape_dir == 'left':
        # Walking left, left is down (+Y), right is up (-Y)
        jog_end = (ex - diag, ey - jog_direction * diag)
    elif escape_dir == 'down':
        # Walking down, left is right (+X), right is left (-X)
        jog_end = (ex - jog_direction * diag, ey + diag)
    else:  # up
        # Walking up, left is left (-X), right is right (+X)
        jog_end = (ex + jog_direction * diag, ey - diag)

    return jog_end, extension_point


def segments_overlap_on_channel(route1: FanoutRoute, route2: FanoutRoute,
                                min_spacing: float) -> bool:
    """
    Check if two routes on the same channel have overlapping channel segments.

    For horizontal channels going right: segments from stub_end.x to exit.x
    The segments overlap if one starts before the other ends.
    """
    if route1.channel.orientation == 'horizontal':
        # Check if the horizontal channel segments overlap
        # Going right: segment is from stub_end.x to exit_pos.x
        # Going left: segment is from exit_pos.x to stub_end.x
        if route1.escape_dir == 'right':
            # Both going right - segment is stub_end.x to exit.x
            # They overlap since they share the same exit point
            # Route closer to exit (higher x) has segment that overlaps with further one
            x1_start, x1_end = route1.stub_end[0], route1.exit_pos[0]
            x2_start, x2_end = route2.stub_end[0], route2.exit_pos[0]
        else:
            # Both going left
            x1_start, x1_end = route1.exit_pos[0], route1.stub_end[0]
            x2_start, x2_end = route2.exit_pos[0], route2.stub_end[0]

        # Segments overlap if max(starts) < min(ends) + spacing
        if max(x1_start, x2_start) < min(x1_end, x2_end) + min_spacing:
            return True
    else:
        # Vertical channel
        if route1.escape_dir == 'down':
            y1_start, y1_end = route1.stub_end[1], route1.exit_pos[1]
            y2_start, y2_end = route2.stub_end[1], route2.exit_pos[1]
        else:
            y1_start, y1_end = route1.exit_pos[1], route1.stub_end[1]
            y2_start, y2_end = route2.exit_pos[1], route2.stub_end[1]

        if max(y1_start, y2_start) < min(y1_end, y2_end) + min_spacing:
            return True

    return False


def assign_layers_smart(routes: List[FanoutRoute],
                        available_layers: List[str],
                        track_width: float,
                        clearance: float,
                        diff_pair_spacing: float = 0.0,
                        existing_tracks: List[Dict] = None) -> None:
    """
    Assign layers to routes to avoid all collisions.

    Strategy:
    - Edge pads stay on first layer (F.Cu) with direct H/V escape
    - Differential pairs are assigned to the same layer together
    - Inner pads grouped by channel AND escape direction
    - Routes with overlapping channel segments must use different layers
    - Cross-escape routes (vertical exit) must NOT conflict with edge routes on same layer
    - Avoid collisions with existing tracks from the PCB
    """
    min_spacing = track_width + clearance
    if existing_tracks is None:
        existing_tracks = []

    # For diff pairs, we need to consider pair spacing when checking collisions
    # Two traces from the same pair use 2*track_width + diff_pair_spacing
    pair_width = 2 * track_width + diff_pair_spacing

    # Edge routes stay on first layer
    edge_routes = []
    for route in routes:
        if route.is_edge:
            route.layer = available_layers[0]
            edge_routes.append(route)

    # Build lookup from pair_id to routes
    pair_routes: Dict[str, List[FanoutRoute]] = defaultdict(list)
    for route in routes:
        if route.pair_id:
            pair_routes[route.pair_id].append(route)

    # Identify half-edge pairs (pairs where one route is an edge route)
    # These pairs should have ALL routes on F.Cu, not just the edge route
    half_edge_pairs: Set[str] = set()
    for pair_id, routes_in_pair in pair_routes.items():
        if any(r.is_edge for r in routes_in_pair):
            half_edge_pairs.add(pair_id)
            # Set ALL routes in this pair to F.Cu
            for r in routes_in_pair:
                r.layer = available_layers[0]
                if r not in edge_routes:
                    edge_routes.append(r)

    # Group inner routes by (channel_index, escape_direction)
    by_channel_dir: Dict[Tuple[int, str], List[FanoutRoute]] = defaultdict(list)
    for route in routes:
        if route.is_edge:
            continue  # Skip edge pads
        # Skip routes that are part of half-edge pairs (already assigned to F.Cu)
        if route.pair_id in half_edge_pairs:
            continue
        key = (route.channel.index, route.escape_dir)
        by_channel_dir[key].append(route)

    # Track which pairs have been assigned
    assigned_pairs: Set[str] = set()

    # For each group, assign layers to avoid overlapping channel segments
    for (channel_idx, escape_dir), group_routes in by_channel_dir.items():
        if not group_routes:
            continue

        # Sort routes by position along the channel segment
        # This puts routes in order from inner to outer (or vice versa)
        if group_routes[0].channel.orientation == 'horizontal':
            if escape_dir == 'right':
                # Sort by stub_end X descending - routes closer to exit first
                group_routes.sort(key=lambda r: r.stub_end[0], reverse=True)
            else:
                group_routes.sort(key=lambda r: r.stub_end[0])
        else:
            if escape_dir == 'down':
                group_routes.sort(key=lambda r: r.stub_end[1], reverse=True)
            else:
                group_routes.sort(key=lambda r: r.stub_end[1])

        # Greedy layer assignment
        # For each route, find a layer where it doesn't conflict with already-assigned routes
        # Inner pairs (using channels) should NOT use F.Cu (first layer) to avoid
        # clearance violations with pads on the top layer
        inner_layers = available_layers[1:] if len(available_layers) > 1 else available_layers

        for route in group_routes:
            # If this route is part of a pair that's already assigned, use that layer
            if route.pair_id and route.pair_id in assigned_pairs:
                # Find the other route in the pair and use its layer
                for other in pair_routes[route.pair_id]:
                    if other is not route and other.layer:
                        route.layer = other.layer
                        break
                continue

            assigned = False

            for layer in inner_layers:
                conflict = False
                # Check against all previously assigned routes on this layer in this group
                for other in group_routes:
                    if other is route:
                        break  # Only check routes before this one
                    if other.layer != layer:
                        continue
                    # Skip collision check for same diff pair (they route together)
                    if route.pair_id and other.pair_id == route.pair_id:
                        continue
                    # Check if channel segments overlap
                    # Use pair_width for spacing if either route is a diff pair
                    spacing = min_spacing
                    if route.pair_id or other.pair_id:
                        spacing = pair_width + clearance
                    if segments_overlap_on_channel(route, other, spacing):
                        conflict = True
                        break

                # Also check against edge routes if assigning to first layer (F.Cu)
                # Edge routes exit horizontally (left/right), inner vertical-exit routes
                # could potentially cross them
                if not conflict and layer == available_layers[0] and edge_routes:
                    spacing = pair_width + clearance if route.pair_id else min_spacing
                    for edge_route in edge_routes:
                        # Check if this route's exit segment conflicts with edge route
                        # Route goes from stub_end to exit_pos (for cross-escape, this is vertical)
                        # Edge route goes from pad_pos/stub_end to exit_pos (horizontal)
                        if route.escape_dir in ['up', 'down']:
                            # Vertical exit - check if it crosses any horizontal edge route
                            # Route vertical segment: from stub_end to exit_pos
                            route_x = route.stub_end[0]  # X is constant for vertical
                            route_y_min = min(route.stub_end[1], route.exit_pos[1])
                            route_y_max = max(route.stub_end[1], route.exit_pos[1])

                            # Edge route horizontal segment: from pad to exit
                            edge_y = edge_route.exit_pos[1]  # Y is constant for horizontal edge
                            edge_x_min = min(edge_route.pad_pos[0], edge_route.exit_pos[0])
                            edge_x_max = max(edge_route.pad_pos[0], edge_route.exit_pos[0])

                            # Check if vertical segment crosses horizontal segment
                            if (route_y_min <= edge_y <= route_y_max and
                                edge_x_min <= route_x <= edge_x_max):
                                # Check spacing - are they too close?
                                # For diff pairs, check both traces
                                if route.pair_id:
                                    # Both P and N traces at route_x +/- half_pair_spacing
                                    for partner in pair_routes.get(route.pair_id, [route]):
                                        partner_x = partner.stub_end[0]
                                        if abs(partner_x - route_x) < spacing:
                                            conflict = True
                                            break
                                else:
                                    conflict = True
                                if conflict:
                                    break

                # Check against existing tracks from the PCB on this layer
                if not conflict and existing_tracks:
                    # Build approximate segments for this route
                    route_segs = [
                        (route.pad_pos, route.stub_end),
                        (route.stub_end, route.exit_pos)
                    ]
                    # Also add channel_point segments if present (for half-edge routes)
                    if route.channel_point:
                        route_segs.append((route.stub_end, route.channel_point))
                        route_segs.append((route.channel_point, route.exit_pos))
                    if route.jog_extension:
                        route_segs.append((route.stub_end, route.jog_extension))
                        route_segs.append((route.jog_extension, route.exit_pos))

                    for existing in existing_tracks:
                        if existing['layer'] != layer:
                            continue
                        # Skip if same net (e.g., connecting to existing stub)
                        if existing.get('net_id') == route.net_id:
                            continue
                        for seg_start, seg_end in route_segs:
                            if check_segment_collision(seg_start, seg_end,
                                                       existing['start'], existing['end'],
                                                       min_spacing):
                                conflict = True
                                break
                        if conflict:
                            break

                if not conflict:
                    route.layer = layer
                    assigned = True
                    # If this is a diff pair, mark it as assigned and set partner's layer
                    if route.pair_id:
                        assigned_pairs.add(route.pair_id)
                        for partner in pair_routes[route.pair_id]:
                            if partner is not route:
                                partner.layer = layer
                    break

            if not assigned:
                # Couldn't find a conflict-free layer - use first inner layer and warn
                route.layer = inner_layers[0]
                print(f"  Warning: Could not find collision-free layer for route at {route.pad_pos}")
                if route.pair_id:
                    assigned_pairs.add(route.pair_id)
                    for partner in pair_routes[route.pair_id]:
                        if partner is not route:
                            partner.layer = inner_layers[0]


def check_segment_collision(seg1_start: Tuple[float, float], seg1_end: Tuple[float, float],
                            seg2_start: Tuple[float, float], seg2_end: Tuple[float, float],
                            clearance: float) -> bool:
    """Check if two segments are too close."""
    # Horizontal segments
    if abs(seg1_start[1] - seg1_end[1]) < 0.001 and abs(seg2_start[1] - seg2_end[1]) < 0.001:
        if abs(seg1_start[1] - seg2_start[1]) < clearance:
            x1_min, x1_max = min(seg1_start[0], seg1_end[0]), max(seg1_start[0], seg1_end[0])
            x2_min, x2_max = min(seg2_start[0], seg2_end[0]), max(seg2_start[0], seg2_end[0])
            if x1_max > x2_min - clearance and x2_max > x1_min - clearance:
                return True

    # Vertical segments
    if abs(seg1_start[0] - seg1_end[0]) < 0.001 and abs(seg2_start[0] - seg2_end[0]) < 0.001:
        if abs(seg1_start[0] - seg2_start[0]) < clearance:
            y1_min, y1_max = min(seg1_start[1], seg1_end[1]), max(seg1_start[1], seg1_end[1])
            y2_min, y2_max = min(seg2_start[1], seg2_end[1]), max(seg2_start[1], seg2_end[1])
            if y1_max > y2_min - clearance and y2_max > y1_min - clearance:
                return True

    # Check endpoint proximity
    for p1 in [seg1_start, seg1_end]:
        for p2 in [seg2_start, seg2_end]:
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
            if dist < clearance:
                return True

    # Check for general segment intersection (handles diagonal crossings)
    # Using cross product method to detect intersection
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(seg2_start, seg2_end, seg1_start)
    d2 = cross(seg2_start, seg2_end, seg1_end)
    d3 = cross(seg1_start, seg1_end, seg2_start)
    d4 = cross(seg1_start, seg1_end, seg2_end)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True

    # Also check point-to-segment distance for near-misses with clearance
    def point_to_segment_dist(px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        proj_x, proj_y = x1 + t * dx, y1 + t * dy
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    # Check distance from each endpoint of seg1 to seg2
    for p in [seg1_start, seg1_end]:
        if point_to_segment_dist(p[0], p[1], seg2_start[0], seg2_start[1], seg2_end[0], seg2_end[1]) < clearance:
            return True
    # Check distance from each endpoint of seg2 to seg1
    for p in [seg2_start, seg2_end]:
        if point_to_segment_dist(p[0], p[1], seg1_start[0], seg1_start[1], seg1_end[0], seg1_end[1]) < clearance:
            return True

    return False


def find_colliding_pairs(tracks: List[Dict], min_spacing: float) -> Set[str]:
    """Find all differential pairs or single-ended nets involved in collisions.

    Returns set of identifiers (pair_id for diff pairs, or 'net_<net_id>' for single-ended).
    """
    colliding_pairs = set()
    for i, t1 in enumerate(tracks):
        # Skip existing tracks (we can't move them)
        if t1.get('is_existing'):
            continue

        for j, t2 in enumerate(tracks[i+1:], i+1):
            # Skip if different layers
            if t1['layer'] != t2['layer']:
                continue
            # Skip if same net
            if t1['net_id'] == t2['net_id']:
                continue
            # Skip if same pair (P and N of same diff pair)
            if t1.get('pair_id') and t1.get('pair_id') == t2.get('pair_id'):
                continue

            if check_segment_collision(t1['start'], t1['end'],
                                        t2['start'], t2['end'],
                                        min_spacing):
                # Use pair_id if available, otherwise use net_id as identifier
                if t1.get('pair_id'):
                    colliding_pairs.add(t1['pair_id'])
                elif not t1.get('is_existing'):
                    colliding_pairs.add(f"net_{t1['net_id']}")
                if t2.get('pair_id'):
                    colliding_pairs.add(t2['pair_id'])
                elif not t2.get('is_existing'):
                    colliding_pairs.add(f"net_{t2['net_id']}")
    return colliding_pairs


def get_track_identifier(track: Dict) -> Optional[str]:
    """Get the identifier for a track (pair_id or net_XXX for single-ended)."""
    if track.get('pair_id'):
        return track['pair_id']
    elif track.get('net_id') and not track.get('is_existing'):
        return f"net_{track['net_id']}"
    return None


def tracks_match_identifier(track: Dict, identifier: str) -> bool:
    """Check if a track matches the given identifier."""
    if identifier.startswith('net_'):
        net_id = int(identifier[4:])
        return track.get('net_id') == net_id and not track.get('pair_id')
    else:
        return track.get('pair_id') == identifier


def find_collision_partners(identifier: str, tracks: List[Dict], min_spacing: float) -> Set[str]:
    """Find all pairs/nets that the given identifier collides with on the same layer."""
    partners = set()
    id_tracks = [t for t in tracks if tracks_match_identifier(t, identifier)]
    other_tracks = [t for t in tracks if not tracks_match_identifier(t, identifier) and not t.get('is_existing')]

    for pt in id_tracks:
        for ot in other_tracks:
            if pt['layer'] != ot['layer']:
                continue
            if check_segment_collision(pt['start'], pt['end'],
                                        ot['start'], ot['end'],
                                        min_spacing):
                other_id = get_track_identifier(ot)
                if other_id:
                    partners.add(other_id)
    return partners


def try_reassign_layer(identifier: str, routes: List[FanoutRoute], tracks: List[Dict],
                       available_layers: List[str], track_width: float,
                       clearance: float, diff_pair_spacing: float,
                       avoid_layers: Set[str] = None,
                       existing_tracks: List[Dict] = None) -> Optional[str]:
    """Try to find a different layer for a colliding pair/net that has no conflicts.

    Args:
        identifier: pair_id for diff pairs, or 'net_<net_id>' for single-ended
        existing_tracks: Read-only list of existing tracks to check against
    """
    min_spacing = track_width + clearance
    if avoid_layers is None:
        avoid_layers = set()
    if existing_tracks is None:
        existing_tracks = []

    # Handle both pair_id and net_XXX identifiers
    is_single_ended = identifier.startswith('net_')
    if is_single_ended:
        net_id = int(identifier[4:])
        # Find routes for this net
        id_routes = [r for r in routes if r.net_id == net_id and r.pair_id is None]
    else:
        # Find routes for this pair
        id_routes = [r for r in routes if r.pair_id == identifier]

    if not id_routes:
        return None

    current_layer = id_routes[0].layer

    # Check if this has any edge route (either full edge or half-edge)
    # Edge and half-edge can use F.Cu; only fully inner are restricted
    has_edge = any(r.is_edge for r in id_routes)

    # Only fully inner should NOT use F.Cu (first layer)
    # to avoid clearance violations with pads on the top layer
    if has_edge:
        candidate_layers = available_layers
    else:
        candidate_layers = available_layers[1:] if len(available_layers) > 1 else available_layers

    # Get tracks for this identifier
    id_track_indices = [i for i, t in enumerate(tracks) if tracks_match_identifier(t, identifier)]

    # Get other tracks (not this identifier) - includes both new and existing tracks
    other_tracks = [t for i, t in enumerate(tracks) if i not in id_track_indices]
    all_other_tracks = other_tracks + existing_tracks

    # Count tracks per layer for load balancing
    layer_counts = {layer: 0 for layer in candidate_layers}
    for t in other_tracks:
        if t['layer'] in layer_counts:
            layer_counts[t['layer']] += 1

    # Sort layers by count (prefer less crowded layers)
    sorted_layers = sorted(candidate_layers, key=lambda l: layer_counts.get(l, 0))

    # Try each candidate layer
    for candidate_layer in sorted_layers:
        if candidate_layer == current_layer:
            continue
        if candidate_layer in avoid_layers:
            continue

        # Check if moving to this layer would cause collisions with other tracks OR existing tracks
        has_collision = False
        for pt_idx in id_track_indices:
            pt = tracks[pt_idx]
            for ot in all_other_tracks:
                if ot['layer'] != candidate_layer:
                    continue
                if check_segment_collision(pt['start'], pt['end'],
                                            ot['start'], ot['end'],
                                            min_spacing):
                    has_collision = True
                    break
            if has_collision:
                break

        if not has_collision:
            return candidate_layer

    return None


def resolve_collisions(routes: List[FanoutRoute], tracks: List[Dict],
                       available_layers: List[str], track_width: float,
                       clearance: float, diff_pair_spacing: float,
                       existing_tracks: List[Dict] = None,
                       grid: BGAGrid = None,
                       channels: List[Channel] = None,
                       exit_margin: float = 0.5,
                       net_names: Dict[int, str] = None) -> Tuple[int, List[str]]:
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
                                        avoid_layers, existing_tracks)
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
                            track_width, clearance
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
                            tracks.append({
                                'start': new_route.pad_pos,
                                'end': new_route.stub_end,
                                'width': track_width,
                                'layer': new_layer,
                                'net_id': net_id,
                                'pair_id': None
                            })
                            tracks.append({
                                'start': new_route.stub_end,
                                'end': new_route.exit_pos,
                                'width': track_width,
                                'layer': new_layer,
                                'net_id': net_id,
                                'pair_id': None
                            })

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
                            track_width, clearance
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
                                    track_width, clearance
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
                                    tracks.append({
                                        'start': new_route.pad_pos,
                                        'end': new_route.stub_end,
                                        'width': track_width,
                                        'layer': retry_layer,
                                        'net_id': net_id,
                                        'pair_id': None
                                    })
                                    tracks.append({
                                        'start': new_route.stub_end,
                                        'end': new_route.exit_pos,
                                        'width': track_width,
                                        'layer': retry_layer,
                                        'net_id': net_id,
                                        'pair_id': None
                                    })

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
                        via_size: float = 0.3,
                        via_drill: float = 0.2,
                        check_for_previous: bool = False) -> Tuple[List[Dict], List[Dict], List[Dict]]:
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

    Returns:
        Tuple of (tracks, vias_to_add, vias_to_remove)
    """
    if layers is None:
        layers = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]

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

                            if is_p_route and p_target_ch:
                                target_y = p_target_ch.position
                                route_ch = p_target_ch
                            elif not is_p_route and n_target_ch:
                                target_y = n_target_ch.position
                                route_ch = n_target_ch
                            else:
                                # Fallback to convergence if no separate channel available
                                target_y = center_y - half_pair_spacing if (is_p_route and p_is_top) or (not is_p_route and not p_is_top) else center_y + half_pair_spacing
                                route_ch = route_ch if 'route_ch' in dir() else channel

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
            channel, escape_dir = find_escape_channel(pad.global_x, pad.global_y, grid, channels,
                                                       force_orientation=force_orient)
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

            route = FanoutRoute(
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
            routes.append(route)

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

    if not routes:
        return [], [], []

    # Convert existing PCB segments to track format for collision checking
    # These are read-only - we check against them but don't modify them
    existing_tracks = []
    if check_for_previous:
        for seg in pcb_data.segments:
            existing_tracks.append({
                'start': (seg.start_x, seg.start_y),
                'end': (seg.end_x, seg.end_y),
                'width': seg.width,
                'layer': seg.layer,
                'net_id': seg.net_id,
                'pair_id': None,  # Existing tracks don't have pair_id
                'is_existing': True  # Mark as existing so we don't try to modify it
            })

    # Smart layer assignment (keeps diff pairs together, avoids existing tracks)
    assign_layers_smart(routes, layers, track_width, clearance, diff_pair_gap, existing_tracks)

    # Calculate jog length = distance from BGA edge to first pad row/col
    # This is half the pitch (since edge is pitch/2 from first pad)
    jog_length = min(grid.pitch_x, grid.pitch_y) / 2
    print(f"  Jog length: {jog_length:.2f} mm")

    # Calculate jog_end for each route based on layer
    # For differential pairs, determine which track is on the outside of the bend
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
            # "Outside" means the track that is further from the bend direction
            # For horizontal escape (left/right), the jog is in Y direction
            # For vertical escape (up/down), the jog is in X direction
            #
            # For right escape with negative jog (up/-Y): lower Y is outside
            # For right escape with positive jog (down/+Y): higher Y is outside
            # Similar logic for other directions

            if route.escape_dir in ['left', 'right']:
                # Horizontal escape, jog is in Y direction
                # Check if this route has higher or lower Y than its pair partner
                # Find the partner route
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

    # Generate tracks
    tracks = []
    edge_count = 0
    inner_count = 0

    for route in routes:
        if route.is_edge:
            # Edge pad: Check if stub_end differs from pad_pos (differential pair convergence)
            if route.pair_id and (abs(route.stub_end[0] - route.pad_pos[0]) > 0.001 or
                                   abs(route.stub_end[1] - route.pad_pos[1]) > 0.001):
                # Differential edge pair: 45° stub to converge, then straight to exit
                # 45° stub: pad -> stub_end (convergence point)
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.stub_end,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
                # Straight segment: stub_end -> exit
                tracks.append({
                    'start': route.stub_end,
                    'end': route.exit_pos,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
            else:
                # Single-ended edge pad: direct segment to exit
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.exit_pos,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
            # Add jog segment(s)
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_extension,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                    # Then the 45° jog from extension point
                    tracks.append({
                        'start': route.jog_extension,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
            edge_count += 1
        else:
            # Inner pad: 45° stub + channel segment + jog
            # For half-edge pairs: pad -> channel_point -> stub_end -> exit

            if route.channel_point:
                # Half-edge inner pad: tent shape with channel segment
                # Path: pad -> channel_point -> channel_point2 -> stub_end -> exit
                #
                # First 45°: pad -> channel_point (entry to channel)
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.channel_point,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
                # Straight segment in channel: channel_point -> channel_point2 (1 pitch)
                if route.channel_point2:
                    tracks.append({
                        'start': route.channel_point,
                        'end': route.channel_point2,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                    # Second 45°: channel_point2 -> stub_end (exit from channel)
                    tracks.append({
                        'start': route.channel_point2,
                        'end': route.stub_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                else:
                    # Fallback if no channel_point2 (shouldn't happen)
                    tracks.append({
                        'start': route.channel_point,
                        'end': route.stub_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                # Straight segment: stub_end -> exit
                tracks.append({
                    'start': route.stub_end,
                    'end': route.exit_pos,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })
            else:
                # Normal inner pad: single 45° stub to channel, then straight to exit
                # Skip zero-length stubs
                dx = abs(route.stub_end[0] - route.pad_pos[0])
                dy = abs(route.stub_end[1] - route.pad_pos[1])
                if dx < 0.001 and dy < 0.001:
                    inner_count += 1
                    continue

                # 45-degree stub: pad -> stub_end
                tracks.append({
                    'start': route.pad_pos,
                    'end': route.stub_end,
                    'width': track_width,
                    'layer': route.layer,
                    'net_id': route.net_id,
                    'pair_id': route.pair_id
                })

                if route.pre_channel_jog:
                    # Jogged route: stub_end -> jog_point -> exit
                    # Jog segment: stub_end -> pre_channel_jog
                    tracks.append({
                        'start': route.stub_end,
                        'end': route.pre_channel_jog,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                    # Channel segment: pre_channel_jog -> exit
                    tracks.append({
                        'start': route.pre_channel_jog,
                        'end': route.exit_pos,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                else:
                    # Normal route: stub_end -> exit
                    tracks.append({
                        'start': route.stub_end,
                        'end': route.exit_pos,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })

            # Jog segment(s): exit -> jog_end
            if route.jog_end:
                if route.jog_extension:
                    # Outside track of diff pair: extension segment first
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_extension,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                    # Then the 45° jog from extension point
                    tracks.append({
                        'start': route.jog_extension,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
                else:
                    # Inside track or single-ended: direct 45° jog
                    tracks.append({
                        'start': route.exit_pos,
                        'end': route.jog_end,
                        'width': track_width,
                        'layer': route.layer,
                        'net_id': route.net_id,
                        'pair_id': route.pair_id
                    })
            inner_count += 1

    print(f"  Generated {len(tracks)} track segments")
    print(f"    Edge pads: {edge_count} (direct H/V on {layers[0]})")
    print(f"    Inner pads: {inner_count} (45° stub + channel)")

    # Validate no collisions
    min_spacing = track_width + clearance
    collision_count = 0
    collision_pairs = []

    # Combine new tracks with existing tracks for collision checking
    all_tracks_for_checking = tracks + existing_tracks

    for i, t1 in enumerate(tracks):
        # Check against other new tracks
        for j, t2 in enumerate(tracks[i+1:], i+1):
            if t1['layer'] != t2['layer']:
                continue
            if t1['net_id'] == t2['net_id']:
                continue
            # Skip collision check for tracks from the same differential pair
            if t1.get('pair_id') and t1.get('pair_id') == t2.get('pair_id'):
                continue
            if check_segment_collision(t1['start'], t1['end'],
                                       t2['start'], t2['end'],
                                       min_spacing):
                collision_count += 1
                if len(collision_pairs) < 5:
                    collision_pairs.append((t1, t2))

        # Also check against existing tracks
        for t2 in existing_tracks:
            if t1['layer'] != t2['layer']:
                continue
            if t1['net_id'] == t2['net_id']:
                continue
            if check_segment_collision(t1['start'], t1['end'],
                                       t2['start'], t2['end'],
                                       min_spacing):
                collision_count += 1
                if len(collision_pairs) < 5:
                    collision_pairs.append((t1, t2))

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
                                        existing_tracks, grid, channels, exit_margin, net_id_to_name)

        if failed_nets:
            print(f"\n  ERROR: Failed to route {len(failed_nets)} net(s):")
            for net_name in failed_nets:
                print(f"    - {net_name}")
            print(f"  These nets have been removed from the output.\n")

        if reassigned > 0:
            # Recount collisions after resolution
            new_collision_count = 0
            for i, t1 in enumerate(tracks):
                for j, t2 in enumerate(tracks[i+1:], i+1):
                    if t1['layer'] != t2['layer']:
                        continue
                    if t1['net_id'] == t2['net_id']:
                        continue
                    if t1.get('pair_id') and t1.get('pair_id') == t2.get('pair_id'):
                        continue
                    if check_segment_collision(t1['start'], t1['end'],
                                               t2['start'], t2['end'],
                                               min_spacing):
                        new_collision_count += 1
                # Also recheck against existing tracks
                for t2 in existing_tracks:
                    if t1['layer'] != t2['layer']:
                        continue
                    if t1['net_id'] == t2['net_id']:
                        continue
                    if check_segment_collision(t1['start'], t1['end'],
                                               t2['start'], t2['end'],
                                               min_spacing):
                        new_collision_count += 1
            print(f"  After resolution: {new_collision_count} collisions remaining")
            collisions_remaining = new_collision_count
    else:
        print(f"  Validated: No collisions")
        collisions_remaining = 0

    # Post-resolution layer rebalancing for even distribution
    # Only rebalance if there are no collisions remaining
    # Include F.Cu for edge routes, inner layers for non-edge routes
    inner_layers = layers[1:] if len(layers) > 1 else layers
    all_layers = layers  # All layers including F.Cu

    if collisions_remaining == 0 and len(all_layers) > 1:
        max_rebalance_iterations = 100  # Limit total moves

        # Track original layers to count net changes at the end
        original_layers = {id(route): route.layer for route in routes}

        # Identify edge/half-edge pairs BEFORE rebalancing to protect some on F.Cu
        edge_pair_ids = set()
        for route in routes:
            if route.pair_id and route.is_edge:
                edge_pair_ids.add(route.pair_id)
        for route in routes:
            if route.pair_id:
                partner_routes = [r for r in routes if r.pair_id == route.pair_id and r is not route]
                if any(r.is_edge for r in partner_routes):
                    edge_pair_ids.add(route.pair_id)

        # Categorize edge pairs by their escape direction (which edge they're on)
        # Store (position_along_edge, pair_id) for sorting
        edge_pairs_by_dir: Dict[str, List[Tuple[float, str]]] = {'left': [], 'right': [], 'up': [], 'down': []}
        seen_pairs = set()
        for route in routes:
            if route.pair_id in edge_pair_ids and route.is_edge and route.pair_id not in seen_pairs:
                esc_dir = route.escape_dir
                if esc_dir and esc_dir in edge_pairs_by_dir:
                    seen_pairs.add(route.pair_id)
                    # Position along edge: Y for left/right edges, X for up/down edges
                    if esc_dir in ['left', 'right']:
                        pos = route.pad_pos[1]  # Y position
                    else:
                        pos = route.pad_pos[0]  # X position
                    edge_pairs_by_dir[esc_dir].append((pos, route.pair_id))

        # Sort each direction's pairs by position along the edge
        for d in edge_pairs_by_dir:
            edge_pairs_by_dir[d].sort(key=lambda x: x[0])

        # Calculate how many edge pairs should stay on F.Cu for balanced total routes
        total_routes = len(routes)
        avg_routes_per_layer = total_routes / len(all_layers)
        target_pairs_on_fcu = int(avg_routes_per_layer / 2 + 0.5)

        # Distribute target pairs evenly across edge directions
        directions = [d for d in ['up', 'down', 'left', 'right'] if edge_pairs_by_dir[d]]
        pairs_to_keep_per_dir = {}
        if directions:
            base_per_dir = target_pairs_on_fcu // len(directions)
            remainder = target_pairs_on_fcu % len(directions)
            for i, d in enumerate(directions):
                pairs_to_keep_per_dir[d] = min(base_per_dir + (1 if i < remainder else 0),
                                               len(edge_pairs_by_dir[d]))

        # Mark which pairs should stay on F.Cu - select evenly spaced pairs along each edge
        pairs_to_keep_on_fcu = set()
        for d, keep_count in pairs_to_keep_per_dir.items():
            pairs_on_edge = edge_pairs_by_dir[d]
            n = len(pairs_on_edge)
            if keep_count >= n:
                # Keep all
                for _, pair_id in pairs_on_edge:
                    pairs_to_keep_on_fcu.add(pair_id)
            elif keep_count > 0:
                # Select evenly spaced indices
                # E.g., if n=14 and keep_count=3, pick indices at 0, 7, 13 (roughly evenly spaced)
                for i in range(keep_count):
                    idx = (i * (n - 1)) // (keep_count - 1) if keep_count > 1 else n // 2
                    pairs_to_keep_on_fcu.add(pairs_on_edge[idx][1])


        # Cycle through target layers for better spatial distribution
        target_layer_idx = 0

        def check_conflicts_on_layer(pair_routes_to_check, all_route_segs, all_net_ids, target_layer):
            """Check if routes can move to target_layer without conflicts."""
            # Check against other routes on target layer
            for other in routes:
                if other in pair_routes_to_check or other.layer != target_layer:
                    continue
                # Build segments for other route, including channel points for half-edge routes
                other_segs = []
                if other.channel_point:
                    other_segs.append((other.pad_pos, other.channel_point))
                    if other.channel_point2:
                        other_segs.append((other.channel_point, other.channel_point2))
                        other_segs.append((other.channel_point2, other.stub_end))
                    else:
                        other_segs.append((other.channel_point, other.stub_end))
                else:
                    other_segs.append((other.pad_pos, other.stub_end))
                other_segs.append((other.stub_end, other.exit_pos))
                for rs, re in all_route_segs:
                    for os, oe in other_segs:
                        if check_segment_collision(rs, re, os, oe, min_spacing):
                            return True

            # Check against existing tracks on target layer
            existing_on_layer = [e for e in existing_tracks if e['layer'] == target_layer]
            for existing in existing_on_layer:
                for seg_start, seg_end in all_route_segs:
                    if check_segment_collision(seg_start, seg_end,
                                               existing['start'], existing['end'],
                                               min_spacing):
                        return True

            # Check against new tracks already on target layer (from other nets)
            for track in tracks:
                if track['layer'] != target_layer:
                    continue
                if track.get('net_id') in all_net_ids:
                    continue  # Skip our own tracks
                for seg_start, seg_end in all_route_segs:
                    if check_segment_collision(seg_start, seg_end,
                                               track['start'], track['end'],
                                               min_spacing):
                        return True

            return False

        def move_connected_tracks(seed_positions, net_ids, old_layer, new_layer):
            """Walk connected tracks from seed positions and move them to new_layer.

            Uses flood-fill approach: start from pad positions, find connected tracks,
            add their other endpoints to the frontier, repeat until no more found.
            """
            # Use rounded positions for matching
            def round_pos(pos):
                return (round(pos[0], 2), round(pos[1], 2))

            frontier = set(round_pos(p) for p in seed_positions)
            visited_positions = set()
            tracks_to_move = []

            while frontier:
                current_pos = frontier.pop()
                if current_pos in visited_positions:
                    continue
                visited_positions.add(current_pos)

                # Find all tracks on old_layer that connect to current_pos
                for track in tracks:
                    if track.get('net_id') not in net_ids:
                        continue
                    if track['layer'] != old_layer:
                        continue

                    ts = round_pos(track['start'])
                    te = round_pos(track['end'])

                    if ts == current_pos:
                        if track not in tracks_to_move:
                            tracks_to_move.append(track)
                        frontier.add(te)
                    elif te == current_pos:
                        if track not in tracks_to_move:
                            tracks_to_move.append(track)
                        frontier.add(ts)

            # Move all found tracks
            for track in tracks_to_move:
                track['layer'] = new_layer

        for iteration in range(max_rebalance_iterations):
            # Count routes per layer
            layer_counts = {layer: 0 for layer in all_layers}
            for route in routes:
                if route.layer in layer_counts:
                    layer_counts[route.layer] += 1

            if not any(layer_counts.values()):
                break  # No routes to rebalance

            # Find most crowded layer and check if balanced
            max_layer = max(all_layers, key=lambda l: layer_counts[l])
            min_count = min(layer_counts.values())
            max_count = layer_counts[max_layer]

            # Stop if difference is small enough (balanced)
            if max_count - min_count <= 1:
                break

            # Try to move a route from most crowded layer to a less crowded layer
            # Cycle through target layers for spatial distribution
            moved_one = False
            processed_pairs = set()

            for route in routes:
                if route.layer != max_layer:
                    continue

                # For diff pairs, process both together and skip if already processed
                if route.pair_id:
                    if route.pair_id in processed_pairs:
                        continue
                    processed_pairs.add(route.pair_id)
                    # Skip edge pairs that should stay on F.Cu
                    if route.pair_id in pairs_to_keep_on_fcu and max_layer == layers[0]:
                        continue
                    # Find both routes in the pair
                    pair_routes = [r for r in routes if r.pair_id == route.pair_id]
                    # Skip if not all on same layer
                    if not all(r.layer == max_layer for r in pair_routes):
                        continue
                else:
                    pair_routes = [route]

                # Build segments for all routes being moved (including channel points for half-edge)
                all_route_segs = []
                all_net_ids = set()
                for r in pair_routes:
                    if r.channel_point:
                        # Half-edge inner route: pad -> channel_point -> channel_point2 -> stub_end -> exit
                        all_route_segs.append((r.pad_pos, r.channel_point))
                        if r.channel_point2:
                            all_route_segs.append((r.channel_point, r.channel_point2))
                            all_route_segs.append((r.channel_point2, r.stub_end))
                        else:
                            all_route_segs.append((r.channel_point, r.stub_end))
                    else:
                        all_route_segs.append((r.pad_pos, r.stub_end))
                    all_route_segs.append((r.stub_end, r.exit_pos))
                    all_net_ids.add(r.net_id)

                # Try each layer in cycling order, starting from current target_layer_idx
                # Skip layers that are already at or above average, and skip source layer
                avg_count = sum(layer_counts.values()) / len(all_layers)

                for i in range(len(all_layers)):
                    try_layer = all_layers[(target_layer_idx + i) % len(all_layers)]

                    # Skip source layer
                    if try_layer == max_layer:
                        continue

                    # Skip layers that are already crowded (at or above average)
                    if layer_counts[try_layer] >= avg_count:
                        continue

                    # Non-edge routes should NOT move TO F.Cu (would conflict with pads)
                    if not route.is_edge and try_layer == layers[0]:
                        continue

                    # Check for conflicts on this target layer
                    if check_conflicts_on_layer(pair_routes, all_route_segs, all_net_ids, try_layer):
                        continue  # Conflict found, try next layer
                    else:
                        # No conflict - move to this layer
                        old_layer = pair_routes[0].layer
                        for r in pair_routes:
                            r.layer = try_layer
                        # Walk connected tracks from pad positions and move them
                        seed_positions = [r.pad_pos for r in pair_routes]
                        move_connected_tracks(seed_positions, all_net_ids, old_layer, try_layer)
                        moved_one = True
                        # Advance the cycle for next move
                        target_layer_idx = (target_layer_idx + 1) % len(all_layers)
                        break

                if moved_one:
                    break  # Move one at a time, then recount

            if not moved_one:
                break  # No more moves possible

        # Phase 2: Distribute remaining edge/half-edge pairs evenly across all layers
        # (pairs_to_keep_on_fcu was already calculated above and respected in phase 1)
        if edge_pair_ids:
            # Move edge pairs from layers with too many to layers with fewer
            for iteration in range(50):  # Limit iterations
                # Count total routes per layer
                total_layer_counts = {layer: 0 for layer in all_layers}
                for route in routes:
                    if route.layer in total_layer_counts:
                        total_layer_counts[route.layer] += 1

                # Count edge pairs per layer
                edge_layer_counts = {layer: 0 for layer in all_layers}
                processed = set()
                for route in routes:
                    if route.pair_id in edge_pair_ids and route.pair_id not in processed:
                        processed.add(route.pair_id)
                        edge_layer_counts[route.layer] += 1

                # Find layer with most edge pairs
                max_edge_layer = max(all_layers, key=lambda l: edge_layer_counts[l])
                min_edge_count = min(edge_layer_counts.values())

                # Stop if edge pairs are balanced (difference <= 1)
                if edge_layer_counts[max_edge_layer] - min_edge_count <= 1:
                    break

                # Average total route count for balance check
                avg_total = sum(total_layer_counts.values()) / len(all_layers)

                # Try to move one edge pair from max_edge_layer to a layer with fewer edge pairs
                moved_one = False
                processed_pairs = set()

                for route in routes:
                    if route.pair_id not in edge_pair_ids:
                        continue
                    if route.pair_id in processed_pairs:
                        continue
                    if route.layer != max_edge_layer:
                        continue
                    # Skip pairs that should stay on F.Cu
                    if route.pair_id in pairs_to_keep_on_fcu and max_edge_layer == layers[0]:
                        continue

                    processed_pairs.add(route.pair_id)
                    pair_routes = [r for r in routes if r.pair_id == route.pair_id]
                    if not all(r.layer == max_edge_layer for r in pair_routes):
                        continue

                    # Build segments (including channel points for half-edge)
                    all_route_segs = []
                    all_net_ids = set()
                    for r in pair_routes:
                        if r.channel_point:
                            # Half-edge inner route: pad -> channel_point -> channel_point2 -> stub_end -> exit
                            all_route_segs.append((r.pad_pos, r.channel_point))
                            if r.channel_point2:
                                all_route_segs.append((r.channel_point, r.channel_point2))
                                all_route_segs.append((r.channel_point2, r.stub_end))
                            else:
                                all_route_segs.append((r.channel_point, r.stub_end))
                        else:
                            all_route_segs.append((r.pad_pos, r.stub_end))
                        all_route_segs.append((r.stub_end, r.exit_pos))
                        all_net_ids.add(r.net_id)

                    # Try layers with fewer edge pairs, preferring those that won't exceed avg total
                    candidate_layers = [l for l in all_layers if l != max_edge_layer and
                                        edge_layer_counts[l] < edge_layer_counts[max_edge_layer]]
                    # Sort by edge count (fewer first), then by total count (fewer first)
                    candidate_layers.sort(key=lambda l: (edge_layer_counts[l], total_layer_counts[l]))

                    for try_layer in candidate_layers:
                        # Don't move if it would make target layer have too many total routes
                        # (more than 2 above average)
                        if total_layer_counts[try_layer] + len(pair_routes) > avg_total + 2:
                            continue

                        if not check_conflicts_on_layer(pair_routes, all_route_segs, all_net_ids, try_layer):
                            # Move the pair
                            old_layer = pair_routes[0].layer
                            for r in pair_routes:
                                r.layer = try_layer
                            # Walk connected tracks from pad positions and move them
                            seed_positions = [r.pad_pos for r in pair_routes]
                            move_connected_tracks(seed_positions, all_net_ids, old_layer, try_layer)
                            moved_one = True
                            break

                    if moved_one:
                        break

                if not moved_one:
                    break

        # Count routes that ended up on a different layer than they started
        rebalanced_count = sum(1 for route in routes if route.layer != original_layers[id(route)])
        if rebalanced_count > 0:
            print(f"  Rebalanced {rebalanced_count} routes for even layer distribution")

    # Stats by layer
    layer_counts = defaultdict(int)
    for route in routes:
        layer_counts[route.layer] += 1
    for layer, count in sorted(layer_counts.items()):
        print(f"    {layer}: {count} routes")

    # Via management: add vias where needed, remove unnecessary ones
    # Build list of existing vias for proximity checking
    top_layer = layers[0]  # F.Cu typically

    def find_nearby_via(x: float, y: float, net_id: int, max_dist: float) -> Optional['Via']:
        """Find an existing via on the same net within max_dist of position."""
        for via in pcb_data.vias:
            if via.net_id != net_id:
                continue
            dist = math.sqrt((via.x - x)**2 + (via.y - y)**2)
            if dist <= max_dist:
                return via
        return None

    def would_overlap_existing_via(x: float, y: float, new_via_size: float) -> bool:
        """Check if a new via at (x,y) would overlap within clearance of any existing via on any net."""
        for via in pcb_data.vias:
            dist = math.sqrt((via.x - x)**2 + (via.y - y)**2)
            # Overlap if distance is less than sum of radii plus clearance
            min_dist = (via.size / 2) + (new_via_size / 2) + clearance
            if dist < min_dist:
                return True
        return False

    vias_to_add: List[Dict] = []
    vias_to_remove: List[Dict] = []

    # Check distance threshold: via is "at pad" if within via radius + small tolerance
    via_proximity_threshold = via_size / 2 + 0.1  # Via radius plus 0.1mm tolerance

    for route in routes:
        pad_x, pad_y = route.pad_pos
        # Look for existing via on the same net near this pad
        existing_via = find_nearby_via(pad_x, pad_y, route.net_id, via_proximity_threshold)

        if route.layer == top_layer:
            # Routing on top layer - no via needed at pad
            # If there's an existing via at this pad for this net, remove it
            if existing_via:
                vias_to_remove.append({
                    'x': existing_via.x,
                    'y': existing_via.y
                })
        else:
            # Routing on inner/bottom layer - via needed to connect from pad (F.Cu) to route layer
            if not existing_via:
                # Check if adding a via here would overlap with an existing via (on any net)
                if would_overlap_existing_via(pad_x, pad_y, via_size):
                    # Don't add - would overlap with existing via
                    pass
                else:
                    # No existing via nearby - add one (through all layers: F.Cu to B.Cu)
                    vias_to_add.append({
                        'x': pad_x,
                        'y': pad_y,
                        'size': via_size,
                        'drill': via_drill,
                        'layers': ['F.Cu', 'B.Cu'],  # Through-hole via connecting all layers
                        'net_id': route.net_id
                    })
            # If existing via already exists nearby, keep it (don't add duplicate)

    if vias_to_add:
        print(f"  Adding {len(vias_to_add)} vias at pads on non-top layers")
    if vias_to_remove:
        print(f"  Removing {len(vias_to_remove)} unnecessary vias at pads on top layer")

    return tracks, vias_to_add, vias_to_remove


def main():
    """Run BGA fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate BGA fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layers', '-l', nargs='+', default=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                        help='Routing layers')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Track clearance in mm')
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
        track_width=args.width,
        clearance=args.clearance,
        diff_pair_gap=args.diff_pair_gap,
        exit_margin=args.exit_margin,
        primary_escape=args.primary_escape,
        force_escape_direction=args.force_escape_direction,
        rebalance_escape=args.rebalance_escape,
        check_for_previous=args.check_for_previous
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
