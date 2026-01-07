"""
Layer assignment for BGA fanout routing.

Assigns routes to layers to avoid collisions during initial fanout creation.
"""

from typing import List, Dict, Set, Tuple, Optional
from collections import defaultdict

from bga_fanout.types import FanoutRoute, Channel
from bga_fanout.collision import check_segment_collision, tracks_match_identifier


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
                        existing_tracks: List[Dict] = None,
                        no_inner_top_layer: bool = False) -> None:
    """
    Assign layers to routes to avoid all collisions.

    Strategy:
    - Edge pads stay on first layer (F.Cu) with direct H/V escape
    - Differential pairs are assigned to the same layer together
    - Inner pads grouped by channel AND escape direction
    - Routes with overlapping channel segments must use different layers
    - Cross-escape routes (vertical exit) must NOT conflict with edge routes on same layer
    - Avoid collisions with existing tracks from the PCB

    Args:
        no_inner_top_layer: If True, inner pads cannot use F.Cu (top layer)
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

            # Determine which layers this route can use:
            # - Diff pairs should NOT use F.Cu (first layer) to avoid clearance violations
            # - If no_inner_top_layer is set, ALL inner routes avoid F.Cu
            # - Single-ended signals can use any layer (unless no_inner_top_layer)
            if route.pair_id or no_inner_top_layer:
                candidate_layers = available_layers[1:] if len(available_layers) > 1 else available_layers
            else:
                candidate_layers = available_layers

            for layer in candidate_layers:
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
                # Couldn't find a conflict-free layer - use first candidate layer and warn
                route.layer = candidate_layers[0]
                print(f"  Warning: Could not find collision-free layer for route at {route.pad_pos}")
                if route.pair_id:
                    assigned_pairs.add(route.pair_id)
                    for partner in pair_routes[route.pair_id]:
                        if partner is not route:
                            partner.layer = candidate_layers[0]


def try_reassign_layer(identifier: str, routes: List[FanoutRoute], tracks: List[Dict],
                       available_layers: List[str], track_width: float,
                       clearance: float, diff_pair_spacing: float,
                       avoid_layers: Set[str] = None,
                       existing_tracks: List[Dict] = None,
                       no_inner_top_layer: bool = False) -> Optional[str]:
    """Try to find a different layer for a colliding pair/net that has no conflicts.

    Args:
        identifier: pair_id for diff pairs, or 'net_<net_id>' for single-ended
        existing_tracks: Read-only list of existing tracks to check against
        no_inner_top_layer: If True, inner pads cannot use F.Cu (top layer)
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

    # Diff pairs should NOT use F.Cu (first layer) to avoid clearance violations with pads.
    # If no_inner_top_layer is set, single-ended signals also avoid F.Cu.
    is_diff_pair = not identifier.startswith('net_')
    if is_diff_pair or no_inner_top_layer:
        candidate_layers = available_layers[1:] if len(available_layers) > 1 else available_layers
    else:
        candidate_layers = available_layers

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
