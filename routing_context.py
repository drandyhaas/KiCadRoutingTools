"""
Routing context helpers for PCB routing.

This module provides helper functions for common routing operations
like building obstacle maps and recording route results.
"""

from typing import List, Set, Dict, Optional, Tuple, TYPE_CHECKING
import numpy as np
from routing_config import GridCoord, GridRouteConfig
from obstacle_map import (
    add_net_stubs_as_obstacles, add_net_vias_as_obstacles, add_net_pads_as_obstacles,
    add_same_net_via_clearance, add_same_net_pad_drill_via_clearance,
    get_same_net_through_hole_positions
)
from obstacle_costs import (
    add_stub_proximity_costs, merge_track_proximity_costs,
    add_cross_layer_tracks, compute_track_proximity_for_net
)
from obstacle_cache import (
    NetObstacleData, add_net_obstacles_from_cache, remove_net_obstacles_from_cache
)
from connectivity import get_stub_endpoints
from net_queries import get_chip_pad_positions
from pcb_modification import add_route_to_pcb_data


def _add_free_via_positions(obstacles, pcb_data, net_ids: List[int], config):
    """Add through-hole pads as free via positions (zero-cost layer change).

    Args:
        obstacles: GridObstacleMap to add free via positions to
        pcb_data: PCB data with pads_by_net
        net_ids: List of net IDs to add through-hole positions for
        config: Routing config with grid_step
    """
    coord = GridCoord(config.grid_step)
    free_via_positions = []
    for net_id in net_ids:
        for pad in pcb_data.pads_by_net.get(net_id, []):
            if pad.drill and pad.drill > 0:
                gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                free_via_positions.append((gx, gy))
    if free_via_positions:
        obstacles.add_free_vias_batch(free_via_positions)


def merge_ripped_route_costs(obstacles, ripped_route_layer_costs: Dict[int, np.ndarray],
                              ripped_route_via_positions: Dict[int, List[Tuple[int, int]]],
                              config: GridRouteConfig):
    """Merge ripped route avoidance costs into the obstacle map.

    When a net is ripped up, we want to apply soft penalties to its former corridor
    so the current routing avoids that area, increasing the chance the ripped net
    can be rerouted later.

    Args:
        obstacles: GridObstacleMap to add costs to
        ripped_route_layer_costs: Dict of net_id -> numpy array [layer, gx, gy, cost] for segments
        ripped_route_via_positions: Dict of net_id -> list of (gx, gy) for vias
        config: Routing configuration
    """
    if config.ripped_route_avoidance_cost <= 0:
        return  # Feature disabled

    # Merge layer-specific costs (segments) - same as track proximity
    if ripped_route_layer_costs:
        merge_track_proximity_costs(obstacles, ripped_route_layer_costs)

    # Merge via costs into stub_proximity (all-layer)
    # Uses existing add_stub_proximity_costs_batch() with via positions
    if ripped_route_via_positions:
        coord = GridCoord(config.grid_step)
        radius_grid = coord.to_grid_dist(config.ripped_route_avoidance_radius)
        cost_grid = int(config.ripped_route_avoidance_cost * 1000 / config.grid_step)

        all_via_positions = []
        for via_list in ripped_route_via_positions.values():
            all_via_positions.extend(via_list)

        if all_via_positions:
            # Use batch method with block_vias=False since we just want soft costs
            obstacles.add_stub_proximity_costs_batch(all_via_positions, radius_grid, cost_grid, False)


def build_diff_pair_obstacles(
    diff_pair_base_obstacles,
    pcb_data,
    config,
    routed_net_ids: List[int],
    remaining_net_ids: List[int],
    all_unrouted_net_ids: List[int],
    p_net_id: int,
    n_net_id: int,
    gnd_net_id: Optional[int],
    track_proximity_cache: Dict,
    layer_map: Dict,
    extra_clearance: float,
    add_own_stubs_func=None,
    net_obstacles_cache: Optional[Dict[int, NetObstacleData]] = None,
    ripped_route_layer_costs: Dict[int, np.ndarray] = None,
    ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None
):
    """
    Build complete obstacle map for diff pair routing.

    Args:
        diff_pair_base_obstacles: Base obstacle map with extra clearance
        pcb_data: PCB data structure
        config: Routing configuration
        routed_net_ids: List of already routed net IDs
        remaining_net_ids: List of remaining net IDs to route
        all_unrouted_net_ids: All unrouted net IDs for stub proximity
        p_net_id: P net ID of the diff pair
        n_net_id: N net ID of the diff pair
        gnd_net_id: GND net ID (for via obstacles)
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
        extra_clearance: Extra clearance for diff pair routing
        add_own_stubs_func: Optional function to add own stubs as obstacles
        net_obstacles_cache: Optional pre-computed net obstacles for fast batch adding
        ripped_route_layer_costs: Optional dict of ripped route layer-specific costs
        ripped_route_via_positions: Optional dict of ripped route via positions

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    obstacles = diff_pair_base_obstacles.clone_fresh()

    # Add previously routed nets as obstacles
    # Note: Cannot use cache for routed nets because their segments have changed
    for routed_id in routed_net_ids:
        add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)
        add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)
        add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)

    # Add GND vias as obstacles
    if gnd_net_id is not None:
        add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, extra_clearance)

    # Add other unrouted nets as obstacles (can use cache since they haven't changed)
    other_unrouted = [nid for nid in remaining_net_ids
                     if nid != p_net_id and nid != n_net_id]
    for other_net_id in other_unrouted:
        if net_obstacles_cache and other_net_id in net_obstacles_cache:
            add_net_obstacles_from_cache(obstacles, net_obstacles_cache[other_net_id])
        else:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config, extra_clearance)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, extra_clearance)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config, extra_clearance)

    # Add stub proximity costs (includes chip pads as pseudo-stubs)
    stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                               if nid != p_net_id and nid != n_net_id
                               and nid not in routed_net_ids]
    unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
    chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
    all_stubs = unrouted_stubs + chip_pads
    if config.verbose:
        print(f"    stub proximity: {len(stub_proximity_net_ids)} nets, {len(unrouted_stubs)} stubs, {len(chip_pads)} chip pads")
    if all_stubs:
        add_stub_proximity_costs(obstacles, all_stubs, config)

    # Add track proximity costs
    merge_track_proximity_costs(obstacles, track_proximity_cache)

    # Add ripped route avoidance costs
    if ripped_route_layer_costs is not None or ripped_route_via_positions is not None:
        merge_ripped_route_costs(obstacles,
                                  ripped_route_layer_costs or {},
                                  ripped_route_via_positions or {},
                                  config)

    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={p_net_id, n_net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, p_net_id, config)
    add_same_net_via_clearance(obstacles, pcb_data, n_net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, p_net_id, config)
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, n_net_id, config)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(obstacles, pcb_data, [p_net_id, n_net_id], config)

    # Add own stubs as obstacles if function provided
    if add_own_stubs_func:
        add_own_stubs_func(obstacles, pcb_data, p_net_id, n_net_id, config, extra_clearance)

    return obstacles, all_stubs


def build_single_ended_obstacles(
    base_obstacles,
    pcb_data,
    config,
    routed_net_ids: List[int],
    remaining_net_ids: List[int],
    all_unrouted_net_ids: List[int],
    net_id: int,
    gnd_net_id: Optional[int],
    track_proximity_cache: Dict,
    layer_map: Dict,
    diagonal_margin: float = 0.25,
    net_obstacles_cache: Optional[Dict[int, NetObstacleData]] = None,
    ripped_route_layer_costs: Dict[int, np.ndarray] = None,
    ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None
):
    """
    Build complete obstacle map for single-ended routing.

    Args:
        base_obstacles: Base obstacle map
        pcb_data: PCB data structure
        config: Routing configuration
        routed_net_ids: List of already routed net IDs
        remaining_net_ids: List of remaining net IDs to route
        all_unrouted_net_ids: All unrouted net IDs for stub proximity
        net_id: Current net ID being routed
        gnd_net_id: GND net ID (for via obstacles)
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
        diagonal_margin: Margin for diagonal segment clearance
        net_obstacles_cache: Optional pre-computed net obstacles for fast batch adding
        ripped_route_layer_costs: Optional dict of ripped route layer-specific costs
        ripped_route_via_positions: Optional dict of ripped route via positions

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    obstacles = base_obstacles.clone_fresh()

    # Add previously routed nets as obstacles
    # Note: Cannot use cache for routed nets because their segments have changed
    for routed_id in routed_net_ids:
        add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
        add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diagonal_margin=diagonal_margin)
        add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)

    # Add GND vias as obstacles
    if gnd_net_id is not None:
        add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diagonal_margin=diagonal_margin)

    # Add other unrouted nets as obstacles (can use cache since they haven't changed)
    other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
    for other_net_id in other_unrouted:
        if net_obstacles_cache and other_net_id in net_obstacles_cache:
            add_net_obstacles_from_cache(obstacles, net_obstacles_cache[other_net_id])
        else:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diagonal_margin=diagonal_margin)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config)

    # Add stub proximity costs (includes chip pads as pseudo-stubs)
    stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                               if nid != net_id and nid not in routed_net_ids]
    unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
    chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
    all_stubs = unrouted_stubs + chip_pads
    if all_stubs:
        add_stub_proximity_costs(obstacles, all_stubs, config)

    # Add track proximity costs
    merge_track_proximity_costs(obstacles, track_proximity_cache)

    # Add ripped route avoidance costs
    if ripped_route_layer_costs is not None or ripped_route_via_positions is not None:
        merge_ripped_route_costs(obstacles,
                                  ripped_route_layer_costs or {},
                                  ripped_route_via_positions or {},
                                  config)

    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, net_id, config)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(obstacles, pcb_data, [net_id], config)

    return obstacles, all_stubs


def build_incremental_obstacles(
    working_obstacles,
    pcb_data,
    config,
    net_id: int,
    all_unrouted_net_ids: List[int],
    routed_net_ids: List[int],
    track_proximity_cache: Dict,
    layer_map: Dict,
    net_obstacles_cache: Dict[int, NetObstacleData]
):
    """
    Build obstacle map for single-ended routing using incremental approach.

    This is MUCH faster than build_single_ended_obstacles because it:
    1. Clones the working map (which already has all net obstacles)
    2. Only removes the current net's obstacles
    3. Adds dynamic costs (proximity, cross-layer tracks)

    Per-route cost is O(current_net_size) instead of O(all_nets_size).

    Args:
        working_obstacles: Pre-built working map with all net obstacles
        pcb_data: PCB data structure
        config: Routing configuration
        net_id: Current net ID being routed
        all_unrouted_net_ids: All unrouted net IDs for stub proximity
        routed_net_ids: List of already routed net IDs
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
        net_obstacles_cache: Pre-computed net obstacles for removal

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    # Clone the working map (has all net obstacles already)
    obstacles = working_obstacles.clone_fresh()

    # Remove current net's obstacles so we can route through our own stubs
    if net_id in net_obstacles_cache:
        remove_net_obstacles_from_cache(obstacles, net_obstacles_cache[net_id])

    # Add stub proximity costs (includes chip pads as pseudo-stubs)
    stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                               if nid != net_id and nid not in routed_net_ids]
    unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
    chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
    all_stubs = unrouted_stubs + chip_pads
    if all_stubs:
        add_stub_proximity_costs(obstacles, all_stubs, config)

    # Add track proximity costs
    merge_track_proximity_costs(obstacles, track_proximity_cache)

    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, net_id, config)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(obstacles, pcb_data, [net_id], config)

    return obstacles, all_stubs


def prepare_obstacles_inplace(
    working_obstacles,
    pcb_data,
    config,
    net_id: int,
    all_unrouted_net_ids: List[int],
    routed_net_ids: List[int],
    track_proximity_cache: Dict,
    layer_map: Dict,
    net_obstacles_cache: Dict[int, NetObstacleData],
    ripped_route_layer_costs: Dict[int, np.ndarray] = None,
    ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None
) -> Tuple[List[Tuple[float, float]], List[Tuple[int, int]]]:
    """
    Prepare working_obstacles IN-PLACE for routing a single-ended net.

    This modifies working_obstacles directly instead of cloning, saving significant memory.
    Returns data needed for restore_obstacles_inplace after routing.

    Args:
        working_obstacles: Working obstacle map (modified in place)
        pcb_data: PCB data structure
        config: Routing configuration
        net_id: Current net ID being routed
        all_unrouted_net_ids: All unrouted net IDs for stub proximity
        routed_net_ids: List of already routed net IDs
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
        net_obstacles_cache: Pre-computed net obstacles
        ripped_route_layer_costs: Optional dict of ripped route layer-specific costs
        ripped_route_via_positions: Optional dict of ripped route via positions

    Returns:
        Tuple of (unrouted_stubs, same_net_via_clearance_cells) for use by restore
    """
    from routing_config import GridCoord

    # Clear per-route data from previous route
    working_obstacles.clear_stub_proximity()
    working_obstacles.clear_layer_proximity()
    working_obstacles.clear_cross_layer_tracks()
    working_obstacles.clear_free_vias()
    working_obstacles.clear_source_target_cells()  # Clear source/target overrides from previous route

    # Remove current net's obstacles so we can route through our own stubs
    if net_id in net_obstacles_cache:
        remove_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[net_id])

    # Add stub proximity costs (includes chip pads as pseudo-stubs)
    stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                               if nid != net_id and nid not in routed_net_ids]
    unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
    chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
    all_stubs = unrouted_stubs + chip_pads
    if all_stubs:
        add_stub_proximity_costs(working_obstacles, all_stubs, config)

    # Add track proximity costs
    merge_track_proximity_costs(working_obstacles, track_proximity_cache)

    # Add ripped route avoidance costs (soft penalty for routing through ripped corridors)
    if ripped_route_layer_costs is not None or ripped_route_via_positions is not None:
        merge_ripped_route_costs(working_obstacles,
                                  ripped_route_layer_costs or {},
                                  ripped_route_via_positions or {},
                                  config)

    # Add cross-layer track data
    add_cross_layer_tracks(working_obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={net_id})

    # Add same-net via clearance and track which cells were added
    coord = GridCoord(config.grid_step)
    same_net_via_cells = []

    # Via-via clearance
    via_via_expansion_grid = max(1, coord.to_grid_dist(config.via_size + config.clearance))
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for ex in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
            for ey in range(-via_via_expansion_grid, via_via_expansion_grid + 1):
                if ex*ex + ey*ey <= via_via_expansion_grid * via_via_expansion_grid:
                    same_net_via_cells.append((gx + ex, gy + ey))

    # Pad drill hole clearance
    # Skip the pad center - the router can use existing through-holes for layer transitions
    if config.hole_to_hole_clearance > 0:
        hole_clearance_grid = coord.to_grid_dist(config.hole_to_hole_clearance + config.via_drill / 2)
        for pad in pcb_data.pads_by_net.get(net_id, []):
            if pad.drill and pad.drill > 0:
                # Include pad drill radius in clearance calculation
                required_dist = pad.drill / 2 + config.via_drill / 2 + config.hole_to_hole_clearance
                expand = coord.to_grid_dist(required_dist)
                gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                for ex in range(-expand, expand + 1):
                    for ey in range(-expand, expand + 1):
                        if ex*ex + ey*ey <= expand * expand:
                            # Skip the pad center - allow layer transitions at through-holes
                            if ex == 0 and ey == 0:
                                continue
                            same_net_via_cells.append((gx + ex, gy + ey))

    # Batch add same-net via clearance (convert to numpy for Rust FFI)
    if same_net_via_cells:
        same_net_via_arr = np.array(same_net_via_cells, dtype=np.int32)
        working_obstacles.add_blocked_vias_batch(same_net_via_arr)
    else:
        same_net_via_arr = np.empty((0, 2), dtype=np.int32)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(working_obstacles, pcb_data, [net_id], config)

    return all_stubs, same_net_via_arr


def restore_obstacles_inplace(
    working_obstacles,
    net_id: int,
    net_obstacles_cache: Dict[int, NetObstacleData],
    same_net_via_cells: np.ndarray
):
    """
    Restore working_obstacles after routing attempt.

    This clears per-route data and restores the current net's obstacles.
    Should be called after prepare_obstacles_inplace, whether routing succeeded or failed.

    Args:
        working_obstacles: Working obstacle map (modified in place)
        net_id: Net ID that was routed
        net_obstacles_cache: Pre-computed net obstacles (for restoring)
        same_net_via_cells: Numpy array of cells added for same-net via clearance (to remove)
    """
    # Clear per-route data
    working_obstacles.clear_stub_proximity()
    working_obstacles.clear_layer_proximity()
    working_obstacles.clear_cross_layer_tracks()
    working_obstacles.clear_free_vias()

    # Remove same-net via clearance cells
    if len(same_net_via_cells) > 0:
        working_obstacles.remove_blocked_vias_batch(same_net_via_cells)

    # Restore current net's obstacles (from cache - original stubs)
    # Note: If routing succeeded, caller should update cache first with new route data
    if net_id in net_obstacles_cache:
        add_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[net_id])


def record_diff_pair_success(
    pcb_data,
    result: Dict,
    pair,
    pair_name: str,
    config,
    remaining_net_ids: List[int],
    routed_net_ids: List[int],
    routed_net_paths: Dict,
    routed_results: Dict,
    diff_pair_by_net_id: Dict,
    track_proximity_cache: Dict,
    layer_map: Dict
):
    """
    Record a successful diff pair route.

    Args:
        pcb_data: PCB data structure
        result: Routing result dict
        pair: DiffPairNet object
        pair_name: Name of the diff pair
        config: Routing configuration
        remaining_net_ids: List of remaining net IDs (modified in place)
        routed_net_ids: List of routed net IDs (modified in place)
        routed_net_paths: Dict of routed paths (modified in place)
        routed_results: Dict of routed results (modified in place)
        diff_pair_by_net_id: Dict mapping net ID to (pair_name, pair)
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
    """
    add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)

    if pair.p_net_id in remaining_net_ids:
        remaining_net_ids.remove(pair.p_net_id)
    if pair.n_net_id in remaining_net_ids:
        remaining_net_ids.remove(pair.n_net_id)

    routed_net_ids.append(pair.p_net_id)
    routed_net_ids.append(pair.n_net_id)

    # Compute and cache track proximity costs
    track_proximity_cache[pair.p_net_id] = compute_track_proximity_for_net(
        pcb_data, pair.p_net_id, config, layer_map)
    track_proximity_cache[pair.n_net_id] = compute_track_proximity_for_net(
        pcb_data, pair.n_net_id, config, layer_map)

    # Track paths for blocking analysis
    if result.get('p_path'):
        routed_net_paths[pair.p_net_id] = result['p_path']
    if result.get('n_path'):
        routed_net_paths[pair.n_net_id] = result['n_path']

    # Track result for rip-up
    routed_results[pair.p_net_id] = result
    routed_results[pair.n_net_id] = result

    # Track pair mapping
    diff_pair_by_net_id[pair.p_net_id] = (pair_name, pair)
    diff_pair_by_net_id[pair.n_net_id] = (pair_name, pair)


def record_single_ended_success(
    pcb_data,
    result: Dict,
    net_id: int,
    config,
    remaining_net_ids: List[int],
    routed_net_ids: List[int],
    routed_net_paths: Dict,
    routed_results: Dict,
    track_proximity_cache: Dict,
    layer_map: Dict
):
    """
    Record a successful single-ended route.

    Args:
        pcb_data: PCB data structure
        result: Routing result dict
        net_id: Net ID that was routed
        config: Routing configuration
        remaining_net_ids: List of remaining net IDs (modified in place)
        routed_net_ids: List of routed net IDs (modified in place)
        routed_net_paths: Dict of routed paths (modified in place)
        routed_results: Dict of routed results (modified in place)
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
    """
    add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)

    if net_id in remaining_net_ids:
        remaining_net_ids.remove(net_id)

    routed_net_ids.append(net_id)

    # Track result
    routed_results[net_id] = result

    # Track path for blocking analysis
    if result.get('path'):
        routed_net_paths[net_id] = result['path']

    # Compute and cache track proximity costs
    track_proximity_cache[net_id] = compute_track_proximity_for_net(
        pcb_data, net_id, config, layer_map)


def restore_ripped_net(
    pcb_data,
    ripped_saved,
    ripped_ids: List[int],
    was_in_results: bool,
    routed_net_ids: List[int],
    remaining_net_ids: List[int],
    routed_results: Dict,
    results: List[Dict],
    config,
    track_proximity_cache: Optional[Dict] = None,
    layer_map: Optional[Dict] = None
):
    """
    Restore a previously ripped net back to routed state.

    Args:
        pcb_data: PCB data structure
        ripped_saved: The saved routing result to restore
        ripped_ids: List of net IDs that were ripped (e.g., [p_net_id, n_net_id] for diff pair)
        was_in_results: Whether the result was in the results list before ripping
        routed_net_ids: List of routed net IDs (modified in place)
        remaining_net_ids: List of remaining net IDs (modified in place)
        routed_results: Dict of routed results (modified in place)
        results: List of routing results (modified in place)
        config: Routing configuration
        track_proximity_cache: Optional cache of track proximity costs
        layer_map: Optional layer name to index mapping
    """
    if not ripped_saved:
        return

    add_route_to_pcb_data(pcb_data, ripped_saved, debug_lines=config.debug_lines)

    for rid in ripped_ids:
        if rid not in routed_net_ids:
            routed_net_ids.append(rid)
        if rid in remaining_net_ids:
            remaining_net_ids.remove(rid)
        routed_results[rid] = ripped_saved

    if was_in_results and ripped_saved not in results:
        results.append(ripped_saved)

    # Restore track proximity cache
    if track_proximity_cache is not None and layer_map is not None:
        for rid in ripped_ids:
            track_proximity_cache[rid] = compute_track_proximity_for_net(
                pcb_data, rid, config, layer_map)
