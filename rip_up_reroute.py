"""
Rip-up and reroute functions for the PCB router.

This module handles removing routed nets from the PCB data and tracking structures,
as well as restoring them when needed (e.g., when a rip-up retry fails).
"""

from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

from kicad_parser import PCBData
from routing_config import GridRouteConfig, DiffPairNet
from route_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from obstacle_costs import compute_track_proximity_for_net
from obstacle_cache import (
    precompute_net_obstacles, add_net_obstacles_from_cache, remove_net_obstacles_from_cache
)

if TYPE_CHECKING:
    from grid_router import GridObstacleMap
    from obstacle_cache import NetObstacleData


def rip_up_net(net_id: int, pcb_data: PCBData, routed_net_ids: List[int],
               routed_net_paths: Dict[int, List], routed_results: Dict[int, dict],
               diff_pair_by_net_id: Dict[int, Tuple[str, DiffPairNet]],
               remaining_net_ids: List[int], results: List[dict],
               config: GridRouteConfig,
               track_proximity_cache: Dict[int, dict] = None,
               working_obstacles: 'GridObstacleMap' = None,
               net_obstacles_cache: Dict[int, 'NetObstacleData'] = None) -> Tuple[Optional[dict], List[int], bool]:
    """Rip up a routed net (or diff pair), removing it from pcb_data and tracking structures.

    Args:
        net_id: The net ID to rip up
        pcb_data: The PCB data structure
        routed_net_ids: List of currently routed net IDs
        routed_net_paths: Dict mapping net IDs to their routed paths
        routed_results: Dict mapping net IDs to their routing results
        diff_pair_by_net_id: Dict mapping net IDs to (pair_name, DiffPair) tuples
        remaining_net_ids: List of net IDs that haven't been routed yet
        results: List of routing results
        config: Routing configuration
        track_proximity_cache: Optional cache for track proximity costs
        working_obstacles: Optional working obstacle map for incremental updates
        net_obstacles_cache: Optional cache of net obstacles for incremental updates

    Returns:
        tuple: (saved_result, ripped_net_ids, was_in_results) for later restoration
               saved_result: the result dict that was removed
               ripped_net_ids: list of net IDs that were ripped (1 for single, 2 for diff pair)
               was_in_results: True if the result was in the results list
    """
    if net_id not in routed_results:
        return None, [], False

    saved_result = routed_results[net_id]
    ripped_net_ids = []
    was_in_results = saved_result in results

    # Remove from pcb_data
    remove_route_from_pcb_data(pcb_data, saved_result)

    # Remove from results list if present
    if was_in_results:
        results.remove(saved_result)

    # Update tracking structures
    if net_id in diff_pair_by_net_id:
        # It's a diff pair - remove both P and N
        _, ripped_pair = diff_pair_by_net_id[net_id]
        ripped_net_ids = [ripped_pair.p_net_id, ripped_pair.n_net_id]

        if ripped_pair.p_net_id in routed_net_ids:
            routed_net_ids.remove(ripped_pair.p_net_id)
        if ripped_pair.n_net_id in routed_net_ids:
            routed_net_ids.remove(ripped_pair.n_net_id)
        routed_net_paths.pop(ripped_pair.p_net_id, None)
        routed_net_paths.pop(ripped_pair.n_net_id, None)
        routed_results.pop(ripped_pair.p_net_id, None)
        routed_results.pop(ripped_pair.n_net_id, None)
        # Remove from track proximity cache
        if track_proximity_cache is not None:
            track_proximity_cache.pop(ripped_pair.p_net_id, None)
            track_proximity_cache.pop(ripped_pair.n_net_id, None)
        # Add back to remaining so stubs are treated as obstacles
        if ripped_pair.p_net_id not in remaining_net_ids:
            remaining_net_ids.append(ripped_pair.p_net_id)
        if ripped_pair.n_net_id not in remaining_net_ids:
            remaining_net_ids.append(ripped_pair.n_net_id)
    else:
        # Single-ended net
        ripped_net_ids = [net_id]

        if net_id in routed_net_ids:
            routed_net_ids.remove(net_id)
        routed_net_paths.pop(net_id, None)
        routed_results.pop(net_id, None)
        # Remove from track proximity cache
        if track_proximity_cache is not None:
            track_proximity_cache.pop(net_id, None)
        # Add back to remaining so stubs are treated as obstacles
        if net_id not in remaining_net_ids:
            remaining_net_ids.append(net_id)

    # Update working_obstacles if provided (for incremental approach)
    # Remove old cache (with route), recompute (stubs only), add new cache
    if working_obstacles is not None and net_obstacles_cache is not None:
        for rid in ripped_net_ids:
            if rid in net_obstacles_cache:
                remove_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[rid])
            # Recompute cache - now only has stubs (route was removed from pcb_data)
            net_obstacles_cache[rid] = precompute_net_obstacles(pcb_data, rid, config)
            add_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[rid])

    return saved_result, ripped_net_ids, was_in_results


def restore_net(net_id: int, saved_result: dict, ripped_net_ids: List[int],
                was_in_results: bool, pcb_data: PCBData, routed_net_ids: List[int],
                routed_net_paths: Dict[int, List], routed_results: Dict[int, dict],
                diff_pair_by_net_id: Dict[int, Tuple[str, DiffPairNet]],
                remaining_net_ids: List[int], results: List[dict],
                config: GridRouteConfig,
                track_proximity_cache: Dict[int, dict] = None,
                layer_map: Dict[str, int] = None,
                working_obstacles: 'GridObstacleMap' = None,
                net_obstacles_cache: Dict[int, 'NetObstacleData'] = None):
    """Restore a previously ripped net to pcb_data and tracking structures.

    Args:
        net_id: The net ID to restore
        saved_result: The saved routing result from rip_up_net
        ripped_net_ids: List of net IDs that were ripped
        was_in_results: Whether the result was in the results list
        pcb_data: The PCB data structure
        routed_net_ids: List of currently routed net IDs
        routed_net_paths: Dict mapping net IDs to their routed paths
        routed_results: Dict mapping net IDs to their routing results
        diff_pair_by_net_id: Dict mapping net IDs to (pair_name, DiffPair) tuples
        remaining_net_ids: List of net IDs that haven't been routed yet
        results: List of routing results
        config: Routing configuration
        track_proximity_cache: Optional cache for track proximity costs
        layer_map: Optional layer name to index mapping
        working_obstacles: Optional working obstacle map for incremental updates
        net_obstacles_cache: Optional cache of net obstacles for incremental updates
    """
    if saved_result is None:
        return

    # Add back to pcb_data
    add_route_to_pcb_data(pcb_data, saved_result, debug_lines=config.debug_lines)

    # Add back to results list if it was there (and not already present)
    if was_in_results and saved_result not in results:
        results.append(saved_result)

    # Restore tracking structures
    if net_id in diff_pair_by_net_id:
        # It's a diff pair
        _, ripped_pair = diff_pair_by_net_id[net_id]

        if ripped_pair.p_net_id not in routed_net_ids:
            routed_net_ids.append(ripped_pair.p_net_id)
        if ripped_pair.n_net_id not in routed_net_ids:
            routed_net_ids.append(ripped_pair.n_net_id)
        # Remove from remaining_net_ids since they're back to routed
        if ripped_pair.p_net_id in remaining_net_ids:
            remaining_net_ids.remove(ripped_pair.p_net_id)
        if ripped_pair.n_net_id in remaining_net_ids:
            remaining_net_ids.remove(ripped_pair.n_net_id)
        if saved_result.get('p_path'):
            routed_net_paths[ripped_pair.p_net_id] = saved_result['p_path']
        if saved_result.get('n_path'):
            routed_net_paths[ripped_pair.n_net_id] = saved_result['n_path']
        routed_results[ripped_pair.p_net_id] = saved_result
        routed_results[ripped_pair.n_net_id] = saved_result
        # Restore track proximity cache
        if track_proximity_cache is not None and layer_map is not None:
            track_proximity_cache[ripped_pair.p_net_id] = compute_track_proximity_for_net(
                pcb_data, ripped_pair.p_net_id, config, layer_map)
            track_proximity_cache[ripped_pair.n_net_id] = compute_track_proximity_for_net(
                pcb_data, ripped_pair.n_net_id, config, layer_map)
    else:
        # Single-ended net
        if net_id not in routed_net_ids:
            routed_net_ids.append(net_id)
        if net_id in remaining_net_ids:
            remaining_net_ids.remove(net_id)
        if saved_result.get('path'):
            routed_net_paths[net_id] = saved_result['path']
        routed_results[net_id] = saved_result
        # Restore track proximity cache
        if track_proximity_cache is not None and layer_map is not None:
            track_proximity_cache[net_id] = compute_track_proximity_for_net(
                pcb_data, net_id, config, layer_map)

    # Update working_obstacles if provided (for incremental approach)
    # Remove stubs-only cache, recompute (with route), add new cache
    if working_obstacles is not None and net_obstacles_cache is not None:
        for rid in ripped_net_ids:
            if rid in net_obstacles_cache:
                remove_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[rid])
            # Recompute cache - now has route again (restored to pcb_data)
            net_obstacles_cache[rid] = precompute_net_obstacles(pcb_data, rid, config)
            add_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[rid])
