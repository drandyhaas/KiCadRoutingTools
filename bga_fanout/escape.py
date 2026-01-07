"""
Escape channel finding and assignment for BGA fanout routing.

Functions for determining escape directions and channels for pads and differential pairs.
"""

from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict

from bga_fanout.types import Channel, BGAGrid, DiffPairPads
from bga_fanout.grid import is_edge_pad


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

        # Diff pairs should NOT use F.Cu (first layer) to avoid clearance violations with pads
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

    # Build pair_info with primary and secondary distances
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
