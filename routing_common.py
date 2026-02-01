"""
Shared utilities for route.py and route_diff.py.

This module contains common functions used by both single-ended and differential
pair routing to avoid code duplication.
"""

import time
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import (
    PCBData, find_components_by_type, get_footprint_bounds, detect_bga_pitch,
    auto_detect_bga_exclusion_zones
)
from routing_config import GridRouteConfig
from connectivity import get_net_endpoints
from obstacle_map import build_base_obstacle_map
from obstacle_cache import (
    precompute_all_net_obstacles, build_working_obstacle_map, precompute_net_obstacles,
    add_net_obstacles_from_cache, remove_net_obstacles_from_cache
)
from memory_debug import (
    get_process_memory_mb, format_memory_stats,
    estimate_net_obstacles_cache_mb
)
from length_matching import (
    apply_length_matching_to_group, apply_time_matching_to_group,
    find_nets_matching_patterns, auto_group_ddr4_nets
)


def setup_bga_exclusion_zones(
    pcb_data: PCBData,
    disable_bga_zones: Optional[List[str]],
    existing_zones: Optional[List[Tuple[float, float, float, float]]] = None
) -> List[Tuple[float, float, float, float, float]]:
    """
    Handle --no-bga-zones argument and auto-detect BGA exclusion zones.

    Args:
        pcb_data: Parsed PCB data
        disable_bga_zones: List from --no-bga-zones argument (None=auto-detect,
                          []=disable all, ['U1','U3']=disable specific)
        existing_zones: Pre-configured zones (if any)

    Returns:
        List of BGA exclusion zone tuples (x1, y1, x2, y2, edge_tolerance)
    """
    if disable_bga_zones is not None:
        if len(disable_bga_zones) == 0:
            # --no-bga-zones with no args: disable all
            print("BGA exclusion zones disabled (all)")
            return []
        else:
            # --no-bga-zones U1 U3: disable only those components
            bga_components = find_components_by_type(pcb_data, 'BGA')
            bga_exclusion_zones = []
            disabled_refs = set(disable_bga_zones)
            for fp in bga_components:
                if fp.reference not in disabled_refs:
                    bounds = get_footprint_bounds(fp, margin=0.5)
                    pitch = detect_bga_pitch(fp)
                    edge_tolerance = 0.5 + pitch * 1.1
                    bga_exclusion_zones.append((*bounds, edge_tolerance))
            if bga_exclusion_zones:
                print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s) (excluding {', '.join(disable_bga_zones)}):")
                enabled_fps = [fp for fp in bga_components if fp.reference not in disabled_refs]
                for fp, zone in zip(enabled_fps, bga_exclusion_zones):
                    edge_tol = zone[4] if len(zone) > 4 else 1.6
                    print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f}), edge_tol={edge_tol:.2f}mm")
            else:
                print(f"BGA exclusion zones disabled for: {', '.join(disable_bga_zones)}")
            return bga_exclusion_zones

    elif existing_zones is None:
        bga_exclusion_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.5)
        if bga_exclusion_zones:
            bga_components = find_components_by_type(pcb_data, 'BGA')
            print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s):")
            for i, (fp, zone) in enumerate(zip(bga_components, bga_exclusion_zones)):
                edge_tol = zone[4] if len(zone) > 4 else 1.6
                print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f}), edge_tol={edge_tol:.2f}mm")
        else:
            print("No BGA components detected - no exclusion zones needed")
        return bga_exclusion_zones

    return existing_zones or []


def resolve_net_ids(pcb_data: PCBData, net_names: List[str]) -> List[Tuple[str, int]]:
    """
    Resolve net names to (name, id) tuples.

    Checks both pcb.nets and pads_by_net for net ID resolution.

    Args:
        pcb_data: Parsed PCB data
        net_names: List of net names to resolve

    Returns:
        List of (net_name, net_id) tuples for found nets
    """
    net_ids = []
    for net_name in net_names:
        net_id = None
        # First check pcb.nets
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        # If not found, check pads_by_net (for nets not in pcb.nets)
        if net_id is None:
            for nid, pads in pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.net_name == net_name:
                        net_id = nid
                        break
                if net_id is not None:
                    break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_ids.append((net_name, net_id))

    return net_ids


def filter_already_routed(
    pcb_data: PCBData,
    net_ids: List[Tuple[str, int]],
    config: GridRouteConfig
) -> Tuple[List[Tuple[str, int]], List[Tuple[str, str]]]:
    """
    Filter out nets that are already fully connected.

    Uses check_net_connectivity for robust connectivity checking that handles:
    - Track-based connectivity with T-junctions
    - Via connections across layers
    - Zone/plane connectivity
    - Through-hole pads acting as vias

    Args:
        pcb_data: Parsed PCB data
        net_ids: List of (net_name, net_id) tuples
        config: Routing configuration

    Returns:
        Tuple of (nets_to_route, already_routed) where:
        - nets_to_route: List of (net_name, net_id) needing routing
        - already_routed: List of (net_name, reason) for skipped nets
    """
    from check_connected import check_net_connectivity

    # Group data by net for quick lookup
    zones_by_net: Dict[int, List] = {}
    for zone in pcb_data.zones:
        if zone.net_id not in zones_by_net:
            zones_by_net[zone.net_id] = []
        zones_by_net[zone.net_id].append(zone)

    segments_by_net: Dict[int, List] = {}
    for seg in pcb_data.segments:
        if seg.net_id not in segments_by_net:
            segments_by_net[seg.net_id] = []
        segments_by_net[seg.net_id].append(seg)

    vias_by_net: Dict[int, List] = {}
    for via in pcb_data.vias:
        if via.net_id not in vias_by_net:
            vias_by_net[via.net_id] = []
        vias_by_net[via.net_id].append(via)

    already_routed = []
    nets_to_route = []

    for net_name, net_id in net_ids:
        net_segments = segments_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_zones = zones_by_net.get(net_id, [])

        # Need at least 2 pads to route
        if len(net_pads) < 2:
            already_routed.append((net_name, f"Only {len(net_pads)} pad(s)"))
            continue

        # No segments and no zones means not connected
        if len(net_segments) == 0 and len(net_zones) == 0:
            nets_to_route.append((net_name, net_id))
            continue

        # Use check_net_connectivity for robust connectivity check
        result = check_net_connectivity(
            net_id, net_segments, net_vias, net_pads, net_zones,
            tolerance=0.02
        )

        if result['connected']:
            already_routed.append((net_name, "Already fully connected"))
        else:
            nets_to_route.append((net_name, net_id))

    if already_routed:
        print(f"\nSkipping {len(already_routed)} already-routed net(s):")
        for net_name, reason in already_routed:
            print(f"  {net_name}: {reason}")

    return nets_to_route, already_routed


def build_obstacle_infrastructure(
    pcb_data: PCBData,
    config: GridRouteConfig,
    all_net_ids_to_route: List[int],
    all_unrouted_net_ids: List[int],
    debug_memory: bool = False,
    mem_start: float = 0.0
) -> Tuple:
    """
    Build base obstacles, net cache, and working obstacles.

    Args:
        pcb_data: Parsed PCB data
        config: Routing configuration
        all_net_ids_to_route: Net IDs being routed (excluded from base obstacles)
        all_unrouted_net_ids: All unrouted net IDs for cache
        debug_memory: Print memory stats if True
        mem_start: Starting memory for delta calculation

    Returns:
        Tuple of (base_obstacles, net_obstacles_cache, working_obstacles)
    """
    # Build base obstacle map
    print("Building base obstacle map...")
    base_start = time.time()
    base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route)
    base_elapsed = time.time() - base_start
    print(f"Base obstacle map built in {base_elapsed:.2f}s")
    if debug_memory:
        mem_after_base = get_process_memory_mb()
        print(format_memory_stats("After base obstacle map", mem_after_base, mem_after_base - mem_start))

    # Pre-compute net obstacles cache
    print("Pre-computing net obstacle cache...")
    cache_start = time.time()
    net_obstacles_cache = precompute_all_net_obstacles(
        pcb_data, list(all_unrouted_net_ids), config,
        extra_clearance=0.0, diagonal_margin=0.25
    )
    cache_time = time.time() - cache_start
    print(f"Net obstacle cache built in {cache_time:.2f}s ({len(net_obstacles_cache)} nets)")
    if debug_memory:
        mem_after_cache = get_process_memory_mb()
        cache_size = estimate_net_obstacles_cache_mb(net_obstacles_cache)
        print(format_memory_stats("After net obstacles cache", mem_after_cache, mem_after_cache - mem_start))
        print(f"[MEMORY]   Cache estimated size: {cache_size:.1f} MB for {len(net_obstacles_cache)} nets")

    # Build working obstacle map
    working_obstacles = build_working_obstacle_map(base_obstacles, net_obstacles_cache)
    working_obstacles.shrink_to_fit()
    if debug_memory:
        mem_after_working = get_process_memory_mb()
        print(format_memory_stats("After working obstacle map", mem_after_working, mem_after_working - mem_start))

    return base_obstacles, net_obstacles_cache, working_obstacles


def run_length_matching(
    routed_results: Dict[int, Dict],
    length_match_groups: List[List[str]],
    config: GridRouteConfig,
    pcb_data: PCBData
) -> Dict[str, Dict]:
    """
    Apply length or time matching to all configured groups.

    If config.time_matching is True, matches propagation time instead of length,
    accounting for different signal speeds on different layers.

    Args:
        routed_results: Dict of net_id -> routing result
        length_match_groups: List of pattern groups to match
        config: Routing configuration
        pcb_data: Parsed PCB data

    Returns:
        Updated net_name_to_result mapping
    """
    # Select matching function based on config
    if config.time_matching:
        matching_func = apply_time_matching_to_group
        print("\n" + "=" * 60)
        print("Time matching (propagation delay)")
        print("=" * 60)
    else:
        matching_func = apply_length_matching_to_group
        print("\n" + "=" * 60)
        print("Length matching")
        print("=" * 60)

    # Build net_name -> result mapping
    net_name_to_result = {}
    for net_id, result in routed_results.items():
        if net_id in pcb_data.nets:
            net_name = pcb_data.nets[net_id].name
            net_name_to_result[net_name] = result

    all_routed_names = list(net_name_to_result.keys())

    # Track all segments/vias from previously processed groups
    all_processed_segments = []
    all_processed_vias = []

    for group in length_match_groups:
        # Handle "auto" for DDR4 grouping
        if len(group) == 1 and group[0].lower() == 'auto':
            auto_groups = auto_group_ddr4_nets(all_routed_names)
            for auto_group in auto_groups:
                if len(auto_group) >= 2:
                    net_name_to_result = matching_func(
                        net_name_to_result, auto_group, config, pcb_data,
                        all_processed_segments, all_processed_vias
                    )
                    # Collect segments/vias from this group for subsequent groups
                    for net_name in auto_group:
                        if net_name in net_name_to_result:
                            result = net_name_to_result[net_name]
                            if result.get('new_segments'):
                                all_processed_segments.extend(result['new_segments'])
                            if result.get('new_vias'):
                                all_processed_vias.extend(result['new_vias'])
        else:
            # Find nets matching the patterns in this group
            matching_nets = find_nets_matching_patterns(all_routed_names, group)
            if len(matching_nets) >= 2:
                match_type = "Time match" if config.time_matching else "Length match"
                print(f"\n{match_type} group: {group}")
                print(f"  Matched nets: {matching_nets}")
                net_name_to_result = matching_func(
                    net_name_to_result, matching_nets, config, pcb_data,
                    all_processed_segments, all_processed_vias
                )
                # Collect segments/vias from this group for subsequent groups
                for net_name in matching_nets:
                    if net_name in net_name_to_result:
                        result = net_name_to_result[net_name]
                        if result.get('new_segments'):
                            all_processed_segments.extend(result['new_segments'])
                        if result.get('new_vias'):
                            all_processed_vias.extend(result['new_vias'])

    return net_name_to_result


def sync_pcb_data_segments(
    pcb_data: PCBData,
    routed_results: Dict[int, Dict],
    original_segment_ids: Set[int],
    state=None,
    config: GridRouteConfig = None
) -> None:
    """
    Sync routed segments back to pcb_data and update obstacle cache.

    Preserves original stubs (segments from input file) and replaces only
    routed segments.

    Args:
        pcb_data: Parsed PCB data (modified in place)
        routed_results: Dict of net_id -> routing result
        original_segment_ids: Set of id() for original segments to preserve
        state: Optional RoutingState for cache updates
        config: Optional config for cache recomputation
    """
    if not routed_results:
        return

    routed_net_ids_set = set(routed_results.keys())
    seg_count_before = len(pcb_data.segments)

    # Remove only ROUTED segments (not original stubs) for routed nets
    pcb_data.segments = [s for s in pcb_data.segments
                         if s.net_id not in routed_net_ids_set or id(s) in original_segment_ids]
    seg_count_after_remove = len(pcb_data.segments)

    # Add current (possibly meandered) segments
    total_added = 0
    for net_id, result in routed_results.items():
        for seg in result.get('new_segments', []):
            pcb_data.segments.append(seg)
            total_added += 1
    print(f"\nSync pcb_data: {seg_count_before} -> {seg_count_after_remove} (kept stubs) -> {len(pcb_data.segments)} (after adding {total_added})")

    # Sync working_obstacles with the updated pcb_data
    if state is not None and config is not None:
        if state.working_obstacles is not None and state.net_obstacles_cache is not None:
            for net_id in routed_net_ids_set:
                # Remove old cache from working
                if net_id in state.net_obstacles_cache:
                    remove_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])
                # Recompute cache from current pcb_data
                state.net_obstacles_cache[net_id] = precompute_net_obstacles(pcb_data, net_id, config)
                # Add new cache to working
                add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])


def get_common_config_kwargs(
    track_width: float,
    clearance: float,
    via_size: float,
    via_drill: float,
    grid_step: float,
    via_cost: int,
    layers: List[str],
    max_iterations: int,
    max_probe_iterations: int,
    heuristic_weight: float,
    turn_cost: int,
    direction_preference_cost: int,
    bus_enabled: bool,
    bus_detection_radius: float,
    bus_attraction_radius: float,
    bus_attraction_bonus: int,
    proximity_heuristic_factor: float,
    bga_exclusion_zones: List,
    stub_proximity_radius: float,
    stub_proximity_cost: float,
    via_proximity_cost: float,
    bga_proximity_radius: float,
    bga_proximity_cost: float,
    track_proximity_distance: float,
    track_proximity_cost: float,
    debug_lines: bool,
    verbose: bool,
    max_rip_up_count: int,
    crossing_penalty: float,
    crossing_layer_check: bool,
    routing_clearance_margin: float,
    hole_to_hole_clearance: float,
    board_edge_clearance: float,
    vertical_attraction_radius: float,
    vertical_attraction_cost: float,
    ripped_route_avoidance_radius: float,
    ripped_route_avoidance_cost: float,
    length_match_groups: Optional[List[List[str]]],
    length_match_tolerance: float,
    meander_amplitude: float,
    time_matching: bool,
    time_match_tolerance: float,
    debug_memory: bool,
    layer_costs: Optional[List[float]] = None
) -> Dict:
    """
    Build config kwargs dict from common parameters.

    Returns a dict that can be passed to GridRouteConfig constructor,
    with additional diff-pair or single-ended specific kwargs added by caller.
    """
    return dict(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        via_cost=via_cost,
        layers=layers,
        max_iterations=max_iterations,
        max_probe_iterations=max_probe_iterations,
        heuristic_weight=heuristic_weight,
        turn_cost=turn_cost,
        direction_preference_cost=direction_preference_cost,
        bus_enabled=bus_enabled,
        bus_detection_radius=bus_detection_radius,
        bus_attraction_radius=bus_attraction_radius,
        bus_attraction_bonus=bus_attraction_bonus,
        proximity_heuristic_factor=proximity_heuristic_factor,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius,
        stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost,
        bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost,
        track_proximity_distance=track_proximity_distance,
        track_proximity_cost=track_proximity_cost,
        debug_lines=debug_lines,
        verbose=verbose,
        max_rip_up_count=max_rip_up_count,
        target_swap_crossing_penalty=crossing_penalty,
        crossing_layer_check=crossing_layer_check,
        routing_clearance_margin=routing_clearance_margin,
        hole_to_hole_clearance=hole_to_hole_clearance,
        board_edge_clearance=board_edge_clearance,
        vertical_attraction_radius=vertical_attraction_radius,
        vertical_attraction_cost=vertical_attraction_cost,
        ripped_route_avoidance_radius=ripped_route_avoidance_radius,
        ripped_route_avoidance_cost=ripped_route_avoidance_cost,
        length_match_groups=length_match_groups,
        length_match_tolerance=length_match_tolerance,
        meander_amplitude=meander_amplitude,
        time_matching=time_matching,
        time_match_tolerance=time_match_tolerance,
        debug_memory=debug_memory,
        layer_costs=layer_costs,
    )
