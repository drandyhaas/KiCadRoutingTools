"""
Routing context helpers for PCB routing.

This module provides helper functions for common routing operations
like building obstacle maps and recording route results.
"""

from typing import List, Set, Dict, Optional, Tuple
from obstacle_map import (
    add_net_stubs_as_obstacles, add_net_vias_as_obstacles, add_net_pads_as_obstacles,
    add_same_net_via_clearance, add_same_net_pad_drill_via_clearance,
    add_stub_proximity_costs, merge_track_proximity_costs,
    add_cross_layer_tracks, compute_track_proximity_for_net
)
from routing_utils import get_stub_endpoints, get_chip_pad_positions, add_route_to_pcb_data


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
    add_own_stubs_func=None
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

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    obstacles = diff_pair_base_obstacles.clone()

    # Add previously routed nets as obstacles
    for routed_id in routed_net_ids:
        add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)
        add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)
        add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)

    # Add GND vias as obstacles
    if gnd_net_id is not None:
        add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, extra_clearance)

    # Add other unrouted nets as obstacles (excluding current pair)
    other_unrouted = [nid for nid in remaining_net_ids
                     if nid != p_net_id and nid != n_net_id]
    for other_net_id in other_unrouted:
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
    if all_stubs:
        add_stub_proximity_costs(obstacles, all_stubs, config)

    # Add track proximity costs
    merge_track_proximity_costs(obstacles, track_proximity_cache)

    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={p_net_id, n_net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, p_net_id, config)
    add_same_net_via_clearance(obstacles, pcb_data, n_net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, p_net_id, config)
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, n_net_id, config)

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
    diagonal_margin: float = 0.25
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

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    obstacles = base_obstacles.clone()

    # Add previously routed nets as obstacles
    for routed_id in routed_net_ids:
        add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
        add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diagonal_margin=diagonal_margin)
        add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)

    # Add GND vias as obstacles
    if gnd_net_id is not None:
        add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diagonal_margin=diagonal_margin)

    # Add other unrouted nets as obstacles
    other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
    for other_net_id in other_unrouted:
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

    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, net_id, config)

    return obstacles, all_stubs


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
