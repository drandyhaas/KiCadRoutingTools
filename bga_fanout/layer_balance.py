"""
Layer rebalancing for BGA fanout routing.

Distributes routes evenly across available layers after initial assignment.
"""

from typing import List, Dict, Set, Tuple
from bga_fanout.types import FanoutRoute
from bga_fanout.collision import check_segment_collision
from bga_fanout.constants import MAX_REBALANCE_ITERATIONS, MAX_EDGE_PAIR_ITERATIONS


def _build_route_segments(route: FanoutRoute) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Build segment list for a route, including channel points for half-edge routes."""
    segs = []
    if route.channel_point:
        segs.append((route.pad_pos, route.channel_point))
        if route.channel_point2:
            segs.append((route.channel_point, route.channel_point2))
            segs.append((route.channel_point2, route.stub_end))
        else:
            segs.append((route.channel_point, route.stub_end))
    else:
        segs.append((route.pad_pos, route.stub_end))
    segs.append((route.stub_end, route.exit_pos))
    return segs


def _check_conflicts_on_layer(
    pair_routes_to_check: List[FanoutRoute],
    all_route_segs: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    all_net_ids: Set[int],
    target_layer: str,
    routes: List[FanoutRoute],
    tracks: List[Dict],
    existing_tracks: List[Dict],
    min_spacing: float,
) -> bool:
    """Check if routes can move to target_layer without conflicts."""
    # Check against other routes on target layer
    for other in routes:
        if other in pair_routes_to_check or other.layer != target_layer:
            continue
        other_segs = _build_route_segments(other)
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


def _move_connected_tracks(
    seed_positions: List[Tuple[float, float]],
    net_ids: Set[int],
    old_layer: str,
    new_layer: str,
    tracks: List[Dict],
) -> None:
    """Walk connected tracks from seed positions and move them to new_layer.

    Uses flood-fill approach: start from pad positions, find connected tracks,
    add their other endpoints to the frontier, repeat until no more found.
    """
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


def rebalance_layers(
    routes: List[FanoutRoute],
    tracks: List[Dict],
    existing_tracks: List[Dict],
    layers: List[str],
    min_spacing: float,
) -> int:
    """
    Rebalance routes across layers for even distribution.

    Args:
        routes: List of FanoutRoute objects (will be modified in place)
        tracks: List of track dicts (will be modified in place)
        existing_tracks: List of existing track dicts (read-only)
        layers: List of available layer names
        min_spacing: Minimum spacing between tracks

    Returns:
        Number of routes that were moved to different layers
    """
    if len(layers) <= 1:
        return 0

    all_layers = layers

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
    edge_pairs_by_dir: Dict[str, List[Tuple[float, str]]] = {'left': [], 'right': [], 'up': [], 'down': []}
    seen_pairs = set()
    for route in routes:
        if route.pair_id in edge_pair_ids and route.is_edge and route.pair_id not in seen_pairs:
            esc_dir = route.escape_dir
            if esc_dir and esc_dir in edge_pairs_by_dir:
                seen_pairs.add(route.pair_id)
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
            for _, pair_id in pairs_on_edge:
                pairs_to_keep_on_fcu.add(pair_id)
        elif keep_count > 0:
            for i in range(keep_count):
                idx = (i * (n - 1)) // (keep_count - 1) if keep_count > 1 else n // 2
                pairs_to_keep_on_fcu.add(pairs_on_edge[idx][1])

    # Cycle through target layers for better spatial distribution
    target_layer_idx = 0

    # Phase 1: General rebalancing
    for iteration in range(MAX_REBALANCE_ITERATIONS):
        # Count routes per layer
        layer_counts = {layer: 0 for layer in all_layers}
        for route in routes:
            if route.layer in layer_counts:
                layer_counts[route.layer] += 1

        if not any(layer_counts.values()):
            break

        # Find most crowded layer and check if balanced
        max_layer = max(all_layers, key=lambda l: layer_counts[l])
        min_count = min(layer_counts.values())
        max_count = layer_counts[max_layer]

        if max_count - min_count <= 1:
            break

        moved_one = False
        processed_pairs = set()

        for route in routes:
            if route.layer != max_layer:
                continue

            if route.pair_id:
                if route.pair_id in processed_pairs:
                    continue
                processed_pairs.add(route.pair_id)
                if route.pair_id in pairs_to_keep_on_fcu and max_layer == layers[0]:
                    continue
                pair_routes = [r for r in routes if r.pair_id == route.pair_id]
                if not all(r.layer == max_layer for r in pair_routes):
                    continue
            else:
                pair_routes = [route]

            # Build segments for all routes being moved
            all_route_segs = []
            all_net_ids = set()
            for r in pair_routes:
                all_route_segs.extend(_build_route_segments(r))
                all_net_ids.add(r.net_id)

            avg_count = sum(layer_counts.values()) / len(all_layers)

            for i in range(len(all_layers)):
                try_layer = all_layers[(target_layer_idx + i) % len(all_layers)]

                if try_layer == max_layer:
                    continue
                if layer_counts[try_layer] >= avg_count:
                    continue
                if not route.is_edge and try_layer == layers[0]:
                    continue

                if _check_conflicts_on_layer(pair_routes, all_route_segs, all_net_ids, try_layer,
                                             routes, tracks, existing_tracks, min_spacing):
                    continue

                old_layer = pair_routes[0].layer
                for r in pair_routes:
                    r.layer = try_layer
                seed_positions = [r.pad_pos for r in pair_routes]
                _move_connected_tracks(seed_positions, all_net_ids, old_layer, try_layer, tracks)
                moved_one = True
                target_layer_idx = (target_layer_idx + 1) % len(all_layers)
                break

            if moved_one:
                break

        if not moved_one:
            break

    # Phase 2: Distribute remaining edge/half-edge pairs evenly across all layers
    if edge_pair_ids:
        for iteration in range(MAX_EDGE_PAIR_ITERATIONS):
            total_layer_counts = {layer: 0 for layer in all_layers}
            for route in routes:
                if route.layer in total_layer_counts:
                    total_layer_counts[route.layer] += 1

            edge_layer_counts = {layer: 0 for layer in all_layers}
            processed = set()
            for route in routes:
                if route.pair_id in edge_pair_ids and route.pair_id not in processed:
                    processed.add(route.pair_id)
                    edge_layer_counts[route.layer] += 1

            max_edge_layer = max(all_layers, key=lambda l: edge_layer_counts[l])
            min_edge_count = min(edge_layer_counts.values())

            if edge_layer_counts[max_edge_layer] - min_edge_count <= 1:
                break

            avg_total = sum(total_layer_counts.values()) / len(all_layers)

            moved_one = False
            processed_pairs = set()

            for route in routes:
                if route.pair_id not in edge_pair_ids:
                    continue
                if route.pair_id in processed_pairs:
                    continue
                if route.layer != max_edge_layer:
                    continue
                if route.pair_id in pairs_to_keep_on_fcu and max_edge_layer == layers[0]:
                    continue

                processed_pairs.add(route.pair_id)
                pair_routes = [r for r in routes if r.pair_id == route.pair_id]
                if not all(r.layer == max_edge_layer for r in pair_routes):
                    continue

                all_route_segs = []
                all_net_ids = set()
                for r in pair_routes:
                    all_route_segs.extend(_build_route_segments(r))
                    all_net_ids.add(r.net_id)

                candidate_layers = [l for l in all_layers if l != max_edge_layer and
                                    edge_layer_counts[l] < edge_layer_counts[max_edge_layer]]
                candidate_layers.sort(key=lambda l: (edge_layer_counts[l], total_layer_counts[l]))

                for try_layer in candidate_layers:
                    if total_layer_counts[try_layer] + len(pair_routes) > avg_total + 2:
                        continue

                    if not _check_conflicts_on_layer(pair_routes, all_route_segs, all_net_ids, try_layer,
                                                     routes, tracks, existing_tracks, min_spacing):
                        old_layer = pair_routes[0].layer
                        for r in pair_routes:
                            r.layer = try_layer
                        seed_positions = [r.pad_pos for r in pair_routes]
                        _move_connected_tracks(seed_positions, all_net_ids, old_layer, try_layer, tracks)
                        moved_one = True
                        break

                if moved_one:
                    break

            if not moved_one:
                break

    # Count routes that ended up on a different layer than they started
    rebalanced_count = sum(1 for route in routes if route.layer != original_layers[id(route)])
    return rebalanced_count
