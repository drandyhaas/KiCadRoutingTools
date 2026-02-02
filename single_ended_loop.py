"""
Single-ended routing loop.

This module contains the main loop for routing single-ended nets,
extracted from route.py for better maintainability.
"""

import time
from typing import List, Tuple, Optional, Any, Dict, Set


def _sample_path(path: List[Tuple[int, int, int]], step: int = 1) -> List[Tuple[int, int, int]]:
    """
    Sample along a simplified path to create a denser path for bus attraction.

    Interpolates between consecutive waypoints, creating points every `step` grid units.

    Args:
        path: Simplified path as list of (gx, gy, layer) tuples
        step: Grid step between sampled points (default 1 = every grid cell)

    Returns:
        Densely sampled path
    """
    if len(path) < 2:
        return list(path)

    sampled = []
    for i in range(len(path) - 1):
        x1, y1, layer1 = path[i]
        x2, y2, layer2 = path[i + 1]

        # Add start point
        sampled.append((x1, y1, layer1))

        # If layer change (via), just add the endpoint - no interpolation
        if layer1 != layer2:
            continue

        # Interpolate between points
        dx = x2 - x1
        dy = y2 - y1
        dist = max(abs(dx), abs(dy))  # Chebyshev distance

        if dist > step:
            # Normalize direction
            steps = dist // step
            for s in range(1, steps):
                t = s / steps
                ix = int(x1 + dx * t)
                iy = int(y1 + dy * t)
                sampled.append((ix, iy, layer1))

    # Add final point
    sampled.append(path[-1])
    return sampled

from routing_state import RoutingState, record_net_event
from bus_detection import detect_bus_groups, get_bus_routing_order, get_attraction_neighbor, BusGroup
from memory_debug import get_process_memory_mb, estimate_track_proximity_cache_mb
from obstacle_map import (
    add_net_stubs_as_obstacles, add_net_vias_as_obstacles, add_net_pads_as_obstacles,
    add_same_net_via_clearance, add_same_net_pad_drill_via_clearance,
    add_net_obstacles_with_vis, VisualizationData
)
from obstacle_costs import (
    add_stub_proximity_costs, merge_track_proximity_costs,
    add_cross_layer_tracks, compute_track_proximity_for_net
)
from obstacle_cache import (
    update_net_obstacles_after_routing, add_net_obstacles_from_cache,
    remove_net_obstacles_from_cache
)
from connectivity import (
    get_stub_endpoints, get_net_endpoints, calculate_stub_length, get_multipoint_net_pads
)
from net_queries import get_chip_pad_positions, calculate_route_length
from pcb_modification import add_route_to_pcb_data
from single_ended_routing import route_net_with_obstacles, route_net_with_visualization, route_multipoint_main
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers, invalidate_obstacle_cache
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import get_canonical_net_id
from routing_context import (
    build_single_ended_obstacles, build_incremental_obstacles,
    prepare_obstacles_inplace, restore_obstacles_inplace
)
from terminal_colors import RED, GREEN, RESET


def _populate_vis_data_from_cache(vis_data, net_obstacles_cache, exclude_net_id: int):
    """Populate vis_data with blocked cells/vias from other nets in the cache.

    This ensures blocked cells are displayed even when routing all nets (--nets "*"),
    where the base obstacle map excludes all nets being routed.

    Args:
        vis_data: VisualizationData to update (modified in place)
        net_obstacles_cache: Dict mapping net_id to NetObstacleData
        exclude_net_id: Net ID to exclude (the net currently being routed)
    """
    for other_net_id, obstacle_data in net_obstacles_cache.items():
        if other_net_id == exclude_net_id:
            continue  # Skip current net - it's being routed

        # Add blocked cells from this net
        for i in range(len(obstacle_data.blocked_cells)):
            gx, gy, layer_idx = obstacle_data.blocked_cells[i]
            # Ensure we have enough layers
            while layer_idx >= len(vis_data.blocked_cells):
                vis_data.blocked_cells.append(set())
            vis_data.blocked_cells[layer_idx].add((gx, gy))

        # Add blocked vias from this net
        for i in range(len(obstacle_data.blocked_vias)):
            gx, gy = obstacle_data.blocked_vias[i]
            vis_data.blocked_vias.add((gx, gy))


def route_single_ended_nets(
    state: RoutingState,
    single_ended_nets: List[Tuple[str, int]],
    visualize: bool = False,
    vis_callback: Any = None,
    base_vis_data: Any = None,
    route_index_start: int = 0,
    cancel_check: Any = None,
    progress_callback: Any = None,
) -> Tuple[int, int, float, int, int, bool]:
    """
    Route all single-ended nets.

    Args:
        state: The routing state object containing all shared state
        single_ended_nets: List of (net_name, net_id) tuples to route
        visualize: Whether to run in visualization mode
        vis_callback: Visualization callback (if visualize=True)
        base_vis_data: Base visualization data (if visualize=True)
        route_index_start: Starting route index (continues from diff pairs)
        cancel_check: Optional callable that returns True if routing should be cancelled
        progress_callback: Optional callable(current, total, net_name) for progress updates

    Returns:
        Tuple of (successful, failed, total_time, total_iterations, route_index, user_quit)
    """
    # Extract frequently-used state fields as local references
    pcb_data = state.pcb_data
    config = state.config
    routed_net_ids = state.routed_net_ids
    routed_net_paths = state.routed_net_paths
    routed_results = state.routed_results
    diff_pair_by_net_id = state.diff_pair_by_net_id
    track_proximity_cache = state.track_proximity_cache
    layer_map = state.layer_map
    reroute_queue = state.reroute_queue
    queued_net_ids = state.queued_net_ids
    rip_and_retry_history = state.rip_and_retry_history
    remaining_net_ids = state.remaining_net_ids
    results = state.results
    base_obstacles = state.base_obstacles
    gnd_net_id = state.gnd_net_id
    all_unrouted_net_ids = state.all_unrouted_net_ids
    total_routes = state.total_routes

    # Counters (kept as locals)
    successful = 0
    failed = 0
    total_time = 0.0
    total_iterations = 0
    route_index = route_index_start
    user_quit = False

    # Cache for obstacle cells - persists across retry iterations for performance
    obstacle_cache = {}

    # Bus detection: reorder nets so bus members are routed together (middle first, then outward)
    # Also track bus membership for attraction during routing
    bus_net_to_group: Dict[int, BusGroup] = {}  # Maps net_id to its bus group
    bus_routed_paths: Dict[int, List[Tuple[int, int, int]]] = {}  # Stores routed paths for attraction

    if config.bus_enabled:
        net_ids_to_check = [net_id for _, net_id in single_ended_nets]
        bus_groups = detect_bus_groups(
            pcb_data, net_ids_to_check,
            detection_radius=config.bus_detection_radius,
            min_nets=config.bus_min_nets
        )

        if bus_groups:
            print(f"\n=== Bus Detection: Found {len(bus_groups)} bus group(s) ===")
            for bus in bus_groups:
                direction = "targets→sources" if bus.clique_endpoint == "target" else "sources→targets"
                print(f"  {bus.name}: {len(bus.net_ids)} nets ({direction})", end =" ")
                net_names_list = [pcb_data.nets[nid].name for nid in bus.net_ids]
                print(f"physical order: {' → '.join(net_names_list)}")

                if config.verbose:
                # Show routing order with guide track and attraction info
                    route_order = get_bus_routing_order(bus)
                    route_names = [pcb_data.nets[nid].name for nid in route_order]
                    print(f"    Routing order:  {' → '.join(route_names)}")
                    print(f"    Guide track:    {route_names[0]} (routed first, no attraction)")

                    # Show attraction relationships
                    for i, nid in enumerate(route_order[1:], 1):
                        net_name = pcb_data.nets[nid].name
                        # Find which neighbor this net will attract to
                        pos_in_bus = bus.net_ids.index(nid)
                        left_neighbor = bus.net_ids[pos_in_bus - 1] if pos_in_bus > 0 else None
                        right_neighbor = bus.net_ids[pos_in_bus + 1] if pos_in_bus < len(bus.net_ids) - 1 else None
                        # Check which neighbors are already routed (appear earlier in route_order)
                        already_routed = set(route_order[:i])
                        if left_neighbor and left_neighbor in already_routed:
                            attract_to = pcb_data.nets[left_neighbor].name
                        elif right_neighbor and right_neighbor in already_routed:
                            attract_to = pcb_data.nets[right_neighbor].name
                        else:
                            attract_to = "none"
                        print(f"    {net_name} attracts to: {attract_to}")

                # Track bus membership
                for nid in bus.net_ids:
                    bus_net_to_group[nid] = bus

            # Reorder nets: bus nets in routing order first, then non-bus nets
            bus_net_ids_set = set(bus_net_to_group.keys())
            reordered_nets = []

            # Add bus nets in proper routing order (middle first, then outward)
            for bus in bus_groups:
                route_order = get_bus_routing_order(bus)
                for nid in route_order:
                    net_name = pcb_data.nets[nid].name
                    reordered_nets.append((net_name, nid))

            # Add non-bus nets in original order
            for net_name, nid in single_ended_nets:
                if nid not in bus_net_ids_set:
                    reordered_nets.append((net_name, nid))

            single_ended_nets = reordered_nets
            print()

    for net_name, net_id in single_ended_nets:
        if user_quit:
            break

        # Check for cancellation request
        if cancel_check is not None and cancel_check():
            print("\nRouting cancelled by user")
            user_quit = True
            break

        route_index += 1
        failed_str = f" ({failed} failed)" if failed > 0 else ""
        print(f"\n[{route_index}/{total_routes}{failed_str}] Routing {net_name} (id={net_id})")

        # Report progress
        if progress_callback is not None:
            msg = net_name
            if failed > 0:
                msg += f" ({failed} failed)"
            progress_callback(route_index, total_routes, msg)
        print("-" * 40)

        # Periodic memory reporting (every 10 nets)
        if config.debug_memory and (route_index % 10 == 1 or route_index == total_routes):
            current_mem = get_process_memory_mb()
            prox_cache_mb = estimate_track_proximity_cache_mb(track_proximity_cache)
            print(f"[MEMORY] Route {route_index}/{total_routes}: {current_mem:.1f} MB total, "
                  f"track_proximity_cache: {prox_cache_mb:.1f} MB ({len(track_proximity_cache)} nets)")

        start_time = time.time()

        # Build obstacles - use same approach for both visualization and non-visualization
        # to ensure VisualRouter replicates GridRouter behavior exactly
        vis_data = None
        same_net_via_cells = None  # Track cells for in-place restore (only used when not visualizing)
        if visualize:
            # Clone the base vis data for visualization display
            vis_data = VisualizationData(
                blocked_cells=[set(s) for s in base_vis_data.blocked_cells],
                blocked_vias=set(base_vis_data.blocked_vias),
                bga_zones_grid=list(base_vis_data.bga_zones_grid),
                bounds=base_vis_data.bounds
            )
            # Use the SAME obstacle preparation as non-visualization path for consistency
            if state.working_obstacles is not None and state.net_obstacles_cache:
                unrouted_stubs, same_net_via_cells = prepare_obstacles_inplace(
                    state.working_obstacles, pcb_data, config, net_id,
                    all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                    state.net_obstacles_cache,
                    state.ripped_route_layer_costs, state.ripped_route_via_positions
                )
                obstacles = state.working_obstacles  # Use same map as GridRouter would
                # Update vis_data with obstacles from other nets (not the current one)
                # This ensures blocked cells are shown even when routing all nets
                _populate_vis_data_from_cache(vis_data, state.net_obstacles_cache, net_id)
            else:
                # Fallback: build obstacles the same way as non-visualization
                obstacles, unrouted_stubs = build_single_ended_obstacles(
                    base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                    all_unrouted_net_ids, net_id, gnd_net_id, track_proximity_cache, layer_map,
                    net_obstacles_cache=state.net_obstacles_cache,
                    ripped_route_layer_costs=state.ripped_route_layer_costs,
                    ripped_route_via_positions=state.ripped_route_via_positions
                )
        else:
            # Use in-place approach if working map is available (saves memory by not cloning)
            if state.working_obstacles is not None and state.net_obstacles_cache:
                unrouted_stubs, same_net_via_cells = prepare_obstacles_inplace(
                    state.working_obstacles, pcb_data, config, net_id,
                    all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                    state.net_obstacles_cache,
                    state.ripped_route_layer_costs, state.ripped_route_via_positions
                )
                obstacles = state.working_obstacles  # Use directly, no clone!
            else:
                # Fallback to full rebuild (but use cache for unrouted nets)
                obstacles, unrouted_stubs = build_single_ended_obstacles(
                    base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                    all_unrouted_net_ids, net_id, gnd_net_id, track_proximity_cache, layer_map,
                    net_obstacles_cache=state.net_obstacles_cache,
                    ripped_route_layer_costs=state.ripped_route_layer_costs,
                    ripped_route_via_positions=state.ripped_route_via_positions
                )

        # Calculate stub length BEFORE routing (stubs are existing segments for this net)
        stub_length = calculate_stub_length(pcb_data, net_id)

        # Route the net using the prepared obstacles
        if visualize:
            # Get source/target grid coords for visualization
            sources, targets, _ = get_net_endpoints(pcb_data, net_id, config)
            sources_grid = [(s[0], s[1], s[2]) for s in sources] if sources else []
            targets_grid = [(t[0], t[1], t[2]) for t in targets] if targets else []

            # Notify visualizer that net routing is starting
            vis_callback.on_net_start(net_name, route_index, net_id,
                                       sources_grid, targets_grid, obstacles, vis_data)

            # Route with visualization
            result = route_net_with_visualization(pcb_data, net_id, config, obstacles, vis_callback)

            # Notify visualizer that net routing is complete
            if result is None:
                # User quit during routing
                user_quit = True
                break

            path = result.get('path') if result and not result.get('failed') else None
            direction = result.get('direction', 'forward') if result else 'forward'
            iterations = result.get('iterations', 0) if result else 0
            success = result is not None and not result.get('failed')

            if not vis_callback.on_net_complete(net_name, success, path, iterations, direction):
                user_quit = True
                break
        else:
            # Check for multi-point net (3+ pads, no existing segments)
            multipoint_pads = get_multipoint_net_pads(pcb_data, net_id, config)
            if multipoint_pads:
                print(f"  Detected multi-point net with {len(multipoint_pads)} pads (Phase 1: main route only)")
                result = route_multipoint_main(pcb_data, net_id, config, obstacles, multipoint_pads)
                # Track for Phase 3 completion after length matching
                if result and not result.get('failed') and result.get('is_multipoint'):
                    state.pending_multipoint_nets[net_id] = result
            else:
                # Check for bus attraction and routing direction
                attraction_path = None
                reverse_direction = False
                if net_id in bus_net_to_group:
                    bus_group = bus_net_to_group[net_id]
                    attraction_path = get_attraction_neighbor(bus_group, net_id, bus_routed_paths)
                    # Route from clustered endpoints (targets if clique was target-based)
                    reverse_direction = (bus_group.clique_endpoint == "target")
                result = route_net_with_obstacles(pcb_data, net_id, config, obstacles,
                                                  attraction_path=attraction_path,
                                                  reverse_direction=reverse_direction)

        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            routed_length = calculate_route_length(result['new_segments'], result.get('new_vias', []), pcb_data)
            total_length = routed_length + stub_length  # Include stubs for pad-to-pad length
            result['route_length'] = total_length  # Store for length matching
            result['stub_length'] = stub_length  # Store stub length separately
            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations, length={total_length:.2f}mm (stubs={stub_length:.2f}mm) ({elapsed:.2f}s)")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']
            # Record success (inline version to avoid circular import)
            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
            if net_id in remaining_net_ids:
                remaining_net_ids.remove(net_id)
            routed_net_ids.append(net_id)
            routed_results[net_id] = result
            # Store path for bus attraction (if this net is in a bus)
            if net_id in bus_net_to_group:
                simplified_path = result.get('path', [])
                # Sample along simplified path to create dense path for attraction
                sampled_path = _sample_path(simplified_path, step=1)
                bus_routed_paths[net_id] = sampled_path
                if config.verbose:
                    print(f"    Stored bus path: {len(sampled_path)} points (sampled from {len(simplified_path)} waypoints)")
            record_net_event(state, net_id, "initial_route", {
                "type": "single-ended",
                "segments": len(result['new_segments']),
                "vias": len(result.get('new_vias', [])),
                "iterations": result['iterations']
            })
            if result.get('path'):
                routed_net_paths[net_id] = result['path']
            track_proximity_cache[net_id] = compute_track_proximity_for_net(pcb_data, net_id, config, layer_map)
            # Update working obstacles with new route
            if state.working_obstacles is not None and state.net_obstacles_cache is not None:
                # Update cache with new route (recomputes from pcb_data)
                update_net_obstacles_after_routing(pcb_data, net_id, result, config, state.net_obstacles_cache)
                # Restore working obstacles (in-place approach): clears per-route data and adds updated cache
                if same_net_via_cells is not None:
                    restore_obstacles_inplace(state.working_obstacles, net_id,
                                             state.net_obstacles_cache, same_net_via_cells)
                else:
                    # Fallback: directly add new cache (when not using in-place)
                    add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])
            # Invalidate blocking analysis cache since we added segments
            invalidate_obstacle_cache(obstacle_cache, net_id)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            total_iterations += iterations

            # Restore working obstacles before attempting rip-up (in-place approach)
            if same_net_via_cells is not None and state.working_obstacles is not None:
                restore_obstacles_inplace(state.working_obstacles, net_id,
                                         state.net_obstacles_cache, same_net_via_cells)
                same_net_via_cells = None  # Mark as restored

            # Try rip-up and reroute for single-ended nets (similar to diff pairs)
            ripped_up = False
            if routed_net_paths and result:
                # Find the direction that failed faster (likely more constrained)
                fwd_iters = result.get('iterations_forward', 0)
                bwd_iters = result.get('iterations_backward', 0)
                fwd_cells = result.pop('blocked_cells_forward', [])
                bwd_cells = result.pop('blocked_cells_backward', [])

                if fwd_iters > 0 and (bwd_iters == 0 or fwd_iters <= bwd_iters):
                    fastest_dir = 'forward'
                    blocked_cells = fwd_cells
                    dir_iters = fwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")
                elif bwd_iters > 0:
                    fastest_dir = 'backward'
                    blocked_cells = bwd_cells
                    dir_iters = bwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")
                else:
                    # Both directions had 0 iterations - combine all blocked cells
                    blocked_cells = list(set(fwd_cells + bwd_cells))

                if blocked_cells:
                    # Get source/target coordinates for blocking analysis
                    single_sources, single_targets, _ = get_net_endpoints(pcb_data, net_id, config)
                    single_source_xy = None
                    single_target_xy = None
                    if single_sources:
                        single_source_xy = (single_sources[0][3], single_sources[0][4])  # orig_x, orig_y
                    if single_targets:
                        single_target_xy = (single_targets[0][3], single_targets[0][4])  # orig_x, orig_y

                    blockers = analyze_frontier_blocking(
                        blocked_cells, pcb_data, config, routed_net_paths,
                        exclude_net_ids={net_id},
                        target_xy=single_target_xy,
                        source_xy=single_source_xy,
                        obstacle_cache=obstacle_cache
                    )
                    print_blocking_analysis(blockers, blocked_cells=blocked_cells,
                                           pcb_data=pcb_data, config=config,
                                           nets_to_route=remaining_net_ids)

                    # Filter to only rippable blockers (those in routed_results)
                    # and deduplicate by diff pair (P and N count as one)
                    rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
                        blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id
                    )

                    # Progressive rip-up: try N=1, then N=2, etc up to max_rip_up_count
                    ripped_items = []
                    ripped_canonical_ids = set()  # Track which canonicals have been ripped
                    retry_succeeded = False
                    last_retry_blocked_cells = blocked_cells  # Start with initial failure's blocked cells

                    for N in range(1, config.max_rip_up_count + 1):
                        # For N > 1, re-analyze from the last retry's blocked cells
                        # to find the most blocking net from that specific failure
                        if N > 1 and last_retry_blocked_cells:
                            print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                            fresh_blockers = analyze_frontier_blocking(
                                last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                exclude_net_ids={net_id},
                                target_xy=single_target_xy,
                                source_xy=single_source_xy,
                                obstacle_cache=obstacle_cache
                            )
                            print_blocking_analysis(fresh_blockers, prefix="    ")
                            # Find the most-blocking net that isn't already ripped
                            next_blocker = None
                            for b in fresh_blockers:
                                if b.net_id in routed_results:
                                    canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                                    if canonical not in ripped_canonical_ids:
                                        next_blocker = b
                                        break
                            if next_blocker is None:
                                print(f"  No additional rippable blockers from retry analysis")
                                break
                            # Replace the Nth blocker with the one from retry analysis
                            next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                            if next_canonical not in seen_canonical_ids:
                                seen_canonical_ids.add(next_canonical)
                                rippable_blockers.append(next_blocker)
                            # Find and move it to position N-1 if needed
                            for idx, b in enumerate(rippable_blockers):
                                if get_canonical_net_id(b.net_id, diff_pair_by_net_id) == next_canonical:
                                    if idx != N - 1 and N - 1 < len(rippable_blockers):
                                        rippable_blockers[idx], rippable_blockers[N-1] = rippable_blockers[N-1], rippable_blockers[idx]
                                    break

                        if N > len(rippable_blockers):
                            break  # Not enough blockers to rip

                        # Build frozenset of all N blocker canonicals for loop check
                        blocker_canonicals = frozenset(
                            get_canonical_net_id(rippable_blockers[i].net_id, diff_pair_by_net_id)
                            for i in range(N)
                        )
                        if (net_id, blocker_canonicals) in rip_and_retry_history:
                            blocker_names = []
                            for i in range(N):
                                b = rippable_blockers[i]
                                if b.net_id in diff_pair_by_net_id:
                                    blocker_names.append(diff_pair_by_net_id[b.net_id][0])
                                else:
                                    blocker_names.append(b.net_name)
                            if len(blocker_names) == 1:
                                blockers_str = blocker_names[0]
                            else:
                                blockers_str = "{" + ", ".join(blocker_names) + "}"
                            print(f"  Skipping N={N}: already tried ripping {blockers_str} for {net_name}")
                            continue

                        # Rip up only the new blocker(s) for this N level
                        rip_successful = True
                        new_ripped_this_level = []

                        if N == 1:
                            blocker = rippable_blockers[0]
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"  Ripping up diff pair {ripped_pair_name_tmp} (P and N) to retry...")
                            else:
                                print(f"  Ripping up {blocker.net_name} to retry...")
                        else:
                            blocker = rippable_blockers[N-1]
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"  Extending to N={N}: ripping diff pair {ripped_pair_name_tmp}...")
                            else:
                                print(f"  Extending to N={N}: ripping {blocker.net_name}...")

                        for i in range(len(ripped_items), N):
                            blocker = rippable_blockers[i]
                            if blocker.net_id not in routed_results:
                                continue
                            saved_result, ripped_ids, was_in_results = rip_up_net(
                                blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                routed_results, diff_pair_by_net_id, remaining_net_ids,
                                results, config, track_proximity_cache,
                                state.working_obstacles, state.net_obstacles_cache,
                                state.ripped_route_layer_costs, state.ripped_route_via_positions,
                                layer_map
                            )
                            if saved_result is None:
                                rip_successful = False
                                break
                            # Invalidate cache for ripped nets
                            for rid in ripped_ids:
                                invalidate_obstacle_cache(obstacle_cache, rid)
                                # Record rip event for the ripped net
                                record_net_event(state, rid, "ripped_by", {
                                    "ripping_net_id": net_id,
                                    "ripping_net_name": net_name,
                                    "reason": f"rip-up retry N={N}",
                                    "N": N
                                })
                            ripped_items.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                            new_ripped_this_level.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
                            ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                            if was_in_results:
                                successful -= 1
                            if blocker.net_id in diff_pair_by_net_id:
                                ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                print(f"    Ripped diff pair {ripped_pair_name_tmp}")
                            else:
                                print(f"    Ripped {blocker.net_name}")

                        if not rip_successful:
                            for rid, saved_result, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                restore_net(rid, saved_result, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config, track_proximity_cache, layer_map,
                                           state.working_obstacles, state.net_obstacles_cache,
                                           state.ripped_route_layer_costs, state.ripped_route_via_positions)
                                if was_in_results:
                                    successful += 1
                                ripped_items.pop()
                            continue

                        # Prepare obstacles in-place for retry (saves memory vs cloning)
                        retry_via_cells = None
                        if state.working_obstacles is not None and state.net_obstacles_cache:
                            _, retry_via_cells = prepare_obstacles_inplace(
                                state.working_obstacles, pcb_data, config, net_id,
                                all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                                state.net_obstacles_cache,
                                state.ripped_route_layer_costs, state.ripped_route_via_positions
                            )
                            retry_obstacles = state.working_obstacles
                        else:
                            retry_obstacles, _ = build_single_ended_obstacles(
                                base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                                all_unrouted_net_ids, net_id, gnd_net_id, track_proximity_cache, layer_map,
                                ripped_route_layer_costs=state.ripped_route_layer_costs,
                                ripped_route_via_positions=state.ripped_route_via_positions
                            )

                        # Check for multi-point net in retry as well
                        retry_multipoint_pads = get_multipoint_net_pads(pcb_data, net_id, config)
                        if retry_multipoint_pads:
                            retry_result = route_multipoint_main(pcb_data, net_id, config, retry_obstacles, retry_multipoint_pads)
                            # Track for Phase 3 completion after length matching
                            if retry_result and not retry_result.get('failed') and retry_result.get('is_multipoint'):
                                state.pending_multipoint_nets[net_id] = retry_result
                        else:
                            # Check for bus attraction in retry
                            retry_attraction_path = None
                            retry_reverse_direction = False
                            if net_id in bus_net_to_group:
                                retry_bus_group = bus_net_to_group[net_id]
                                retry_attraction_path = get_attraction_neighbor(retry_bus_group, net_id, bus_routed_paths)
                                retry_reverse_direction = (retry_bus_group.clique_endpoint == "target")
                            retry_result = route_net_with_obstacles(pcb_data, net_id, config, retry_obstacles,
                                                                     attraction_path=retry_attraction_path,
                                                                     reverse_direction=retry_reverse_direction)

                        if retry_result and not retry_result.get('failed'):
                            route_length = calculate_route_length(retry_result['new_segments'], retry_result.get('new_vias', []), pcb_data)
                            retry_result['route_length'] = route_length
                            print(f"  RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result['new_vias'])} vias, length={route_length:.2f}mm")
                            results.append(retry_result)
                            successful += 1
                            total_iterations += retry_result['iterations']
                            add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                            if net_id in remaining_net_ids:
                                remaining_net_ids.remove(net_id)
                            routed_net_ids.append(net_id)
                            routed_results[net_id] = retry_result
                            record_net_event(state, net_id, "reroute_succeeded", {
                                "N": N,
                                "segments": len(retry_result['new_segments']),
                                "vias": len(retry_result.get('new_vias', []))
                            })
                            if retry_result.get('path'):
                                routed_net_paths[net_id] = retry_result['path']
                                # Store path for bus attraction
                                if net_id in bus_net_to_group:
                                    bus_routed_paths[net_id] = retry_result['path']
                            track_proximity_cache[net_id] = compute_track_proximity_for_net(pcb_data, net_id, config, layer_map)
                            # Allow re-queuing if this net gets ripped again later
                            queued_net_ids.discard(net_id)
                            # Update working obstacles with new route
                            if state.working_obstacles is not None and state.net_obstacles_cache is not None:
                                # Update cache with new route
                                update_net_obstacles_after_routing(pcb_data, net_id, retry_result, config, state.net_obstacles_cache)
                                # Restore working obstacles (in-place): clears per-route data, adds updated cache
                                if retry_via_cells is not None:
                                    restore_obstacles_inplace(state.working_obstacles, net_id,
                                                             state.net_obstacles_cache, retry_via_cells)
                                else:
                                    add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])
                            # Invalidate blocking analysis cache since we added segments
                            invalidate_obstacle_cache(obstacle_cache, net_id)

                            # Queue all ripped-up nets for rerouting and add to history
                            rip_and_retry_history.add((net_id, blocker_canonicals))

                            for rid, saved_result, ripped_ids, was_in_results in ripped_items:
                                if was_in_results:
                                    successful -= 1
                                if rid in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[rid]
                                    canonical_id = ripped_pair_tmp.p_net_id
                                    if canonical_id not in queued_net_ids:
                                        reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                        queued_net_ids.add(canonical_id)
                                else:
                                    if rid not in queued_net_ids:
                                        ripped_net = pcb_data.nets.get(rid)
                                        ripped_net_name = ripped_net.name if ripped_net else f"Net {rid}"
                                        reroute_queue.append(('single', ripped_net_name, rid))
                                        queued_net_ids.add(rid)

                            ripped_up = True
                            retry_succeeded = True
                            break  # Success! Exit the N loop
                        else:
                            print(f"  RETRY FAILED (N={N})")

                            # Restore working obstacles after failed retry (in-place approach)
                            if retry_via_cells is not None and state.working_obstacles is not None:
                                restore_obstacles_inplace(state.working_obstacles, net_id,
                                                         state.net_obstacles_cache, retry_via_cells)
                                retry_via_cells = None

                            # Store blocked cells from retry for next iteration's analysis
                            if retry_result:
                                retry_fwd_cells = retry_result.pop('blocked_cells_forward', [])
                                retry_bwd_cells = retry_result.pop('blocked_cells_backward', [])
                                last_retry_blocked_cells = list(set(retry_fwd_cells + retry_bwd_cells))
                                del retry_fwd_cells, retry_bwd_cells  # Free memory immediately
                                if last_retry_blocked_cells:
                                    print(f"    Retry had {len(last_retry_blocked_cells)} blocked cells")
                                else:
                                    print(f"    No blocked cells from retry to analyze")

                    # If all N levels failed, restore all ripped nets
                    if not retry_succeeded and ripped_items:
                        # Get top blockers from last analysis for history
                        top_blocker_names = [b.net_name for b in rippable_blockers[:3]] if rippable_blockers else []
                        record_net_event(state, net_id, "reroute_failed", {
                            "reason": "all rip-up attempts failed",
                            "max_N": len(ripped_items),
                            "top_blockers": top_blocker_names
                        })
                        print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                        for rid, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
                            restore_net(rid, saved_result, ripped_ids, was_in_results,
                                       pcb_data, routed_net_ids, routed_net_paths,
                                       routed_results, diff_pair_by_net_id, remaining_net_ids,
                                       results, config, track_proximity_cache, layer_map,
                                       state.working_obstacles, state.net_obstacles_cache,
                                       state.ripped_route_layer_costs, state.ripped_route_via_positions)
                            if was_in_results:
                                successful += 1

            if not ripped_up:
                record_net_event(state, net_id, "reroute_failed", {
                    "reason": "no rippable blockers found"
                })
                print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                failed += 1

    return successful, failed, total_time, total_iterations, route_index, user_quit
