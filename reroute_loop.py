"""
Reroute loop for ripped-up nets.

This module contains the unified reroute loop that handles all nets ripped
during diff pair or single-ended routing, extracted from route.py.
"""

import time
from typing import List, Tuple

from routing_state import RoutingState
from obstacle_costs import compute_track_proximity_for_net
from connectivity import get_net_endpoints, calculate_stub_length, get_multipoint_net_pads
from net_queries import calculate_route_length
from route_modification import add_route_to_pcb_data
from single_ended_routing import route_net_with_obstacles, route_multipoint_main, route_multipoint_taps
from diff_pair_routing import route_diff_pair_with_obstacles, get_diff_pair_endpoints
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers, invalidate_obstacle_cache
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import apply_polarity_swap, get_canonical_net_id
from layer_swap_fallback import try_fallback_layer_swap, add_own_stubs_as_obstacles_for_diff_pair
from routing_context import (
    build_single_ended_obstacles, build_diff_pair_obstacles,
    record_single_ended_success, record_diff_pair_success,
    prepare_obstacles_inplace, restore_obstacles_inplace
)
from obstacle_cache import update_net_obstacles_after_routing
from terminal_colors import RED, GREEN, RESET


def run_reroute_loop(
    state: RoutingState,
    route_index_start: int = 0,
) -> Tuple[int, int, float, int, int]:
    """
    Run the unified reroute loop for all nets ripped during routing.

    Args:
        state: The routing state object containing all shared state
        route_index_start: Starting route index (continues from previous routing phases)

    Returns:
        Tuple of (successful, failed, total_time, total_iterations, route_index)
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
    rip_and_retry_history = state.rip_and_retry_history
    rerouted_pairs = state.rerouted_pairs
    polarity_swapped_pairs = state.polarity_swapped_pairs
    remaining_net_ids = state.remaining_net_ids
    results = state.results
    pad_swaps = state.pad_swaps
    base_obstacles = state.base_obstacles
    diff_pair_base_obstacles = state.diff_pair_base_obstacles
    diff_pair_extra_clearance = state.diff_pair_extra_clearance
    gnd_net_id = state.gnd_net_id
    all_unrouted_net_ids = state.all_unrouted_net_ids
    total_routes = state.total_routes
    enable_layer_switch = state.enable_layer_switch
    target_swaps = state.target_swaps
    all_swap_vias = state.all_swap_vias
    all_segment_modifications = state.all_segment_modifications

    # Counters (kept as locals)
    successful = 0
    failed = 0
    total_time = 0.0
    total_iterations = 0
    route_index = route_index_start
    reroute_index = 0

    # Cache for obstacle cells - persists across retry iterations for performance
    obstacle_cache = {}

    # Get the queued_net_ids tracking set from state
    queued_net_ids = state.queued_net_ids

    # Unified reroute loop - handles all nets ripped during diff pair or single-ended routing
    while reroute_index < len(reroute_queue):
        reroute_item = reroute_queue[reroute_index]
        reroute_index += 1

        if reroute_item[0] == 'single':
            _, ripped_net_name, ripped_net_id = reroute_item

            # Skip if already routed
            if ripped_net_id in routed_results:
                continue

            route_index += 1
            current_total = total_routes + len(reroute_queue)
            failed_str = f" ({failed} failed)" if failed > 0 else ""
            print(f"\n[REROUTE {route_index}/{current_total}{failed_str}] Re-routing ripped net {ripped_net_name}")
            print("-" * 40)

            # Calculate stub length before routing
            stub_length = calculate_stub_length(pcb_data, ripped_net_id)

            start_time = time.time()
            # Use in-place approach if working obstacles available (saves memory)
            reroute_via_cells = None
            if state.working_obstacles is not None and state.net_obstacles_cache:
                unrouted_stubs, reroute_via_cells = prepare_obstacles_inplace(
                    state.working_obstacles, pcb_data, config, ripped_net_id,
                    all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                    state.net_obstacles_cache
                )
                obstacles = state.working_obstacles
            else:
                obstacles, unrouted_stubs = build_single_ended_obstacles(
                    base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                    all_unrouted_net_ids, ripped_net_id, gnd_net_id, track_proximity_cache, layer_map
                )

            # Check for multi-point net (3+ pads, no existing segments since they were ripped)
            multipoint_pads = get_multipoint_net_pads(pcb_data, ripped_net_id, config)
            if multipoint_pads:
                print(f"  Multi-point net with {len(multipoint_pads)} pads - routing main + taps")
                result = route_multipoint_main(pcb_data, ripped_net_id, config, obstacles, multipoint_pads)
                # If Phase 1 succeeded, immediately do Phase 3 (tap routing)
                if result and not result.get('failed') and result.get('is_multipoint'):
                    main_segments_count = len(result['new_segments'])
                    main_vias_count = len(result.get('new_vias', []))
                    tap_result = route_multipoint_taps(pcb_data, ripped_net_id, config, obstacles, result)
                    if tap_result:
                        result = tap_result  # Use combined result
                        tap_segments = len(result['new_segments']) - main_segments_count
                        tap_vias = len(result.get('new_vias', [])) - main_vias_count
                        print(f"    Tap routing: {tap_segments} segments, {tap_vias} vias")
                        # Remove from pending_multipoint_nets since Phase 3 is now complete
                        if ripped_net_id in state.pending_multipoint_nets:
                            del state.pending_multipoint_nets[ripped_net_id]
            else:
                result = route_net_with_obstacles(pcb_data, ripped_net_id, config, obstacles)
            elapsed = time.time() - start_time
            total_time += elapsed

            if result and not result.get('failed'):
                routed_length = calculate_route_length(result['new_segments'], result.get('new_vias', []), pcb_data)
                route_length = routed_length + stub_length
                result['route_length'] = route_length
                result['stub_length'] = stub_length
                print(f"  REROUTE SUCCESS: {len(result['new_segments'])} segments, {len(result.get('new_vias', []))} vias, length={route_length:.2f}mm (stubs={stub_length:.2f}mm) ({elapsed:.2f}s)")
                results.append(result)
                successful += 1
                total_iterations += result['iterations']
                record_single_ended_success(
                    pcb_data, result, ripped_net_id, config,
                    remaining_net_ids, routed_net_ids, routed_net_paths,
                    routed_results, track_proximity_cache, layer_map
                )
                # Allow re-queuing if this net gets ripped again later
                queued_net_ids.discard(ripped_net_id)
                # Update net obstacles cache with new route, then restore working obstacles
                if reroute_via_cells is not None and state.working_obstacles is not None:
                    update_net_obstacles_after_routing(pcb_data, ripped_net_id, result, config, state.net_obstacles_cache)
                    restore_obstacles_inplace(state.working_obstacles, ripped_net_id,
                                             state.net_obstacles_cache, reroute_via_cells)
                    reroute_via_cells = None
            else:
                # Reroute failed - try rip-up and retry
                iterations = result['iterations'] if result else 0
                total_iterations += iterations
                print(f"  REROUTE FAILED: ({elapsed:.2f}s) - attempting rip-up and retry...")

                # Restore working obstacles before rip-up/retry (in-place approach)
                if reroute_via_cells is not None and state.working_obstacles is not None:
                    restore_obstacles_inplace(state.working_obstacles, ripped_net_id,
                                             state.net_obstacles_cache, reroute_via_cells)
                    reroute_via_cells = None

                reroute_succeeded = False
                ripped_items = []
                if routed_net_paths and result:
                    fwd_cells = result.pop('blocked_cells_forward', [])
                    bwd_cells = result.pop('blocked_cells_backward', [])
                    blocked_cells = list(set(fwd_cells + bwd_cells))
                    del fwd_cells, bwd_cells  # Free memory immediately

                    if blocked_cells:
                        # Get source/target coordinates for blocking analysis
                        reroute_sources, reroute_targets, _ = get_net_endpoints(pcb_data, ripped_net_id, config)
                        reroute_source_xy = None
                        reroute_target_xy = None
                        if reroute_sources:
                            reroute_source_xy = (reroute_sources[0][3], reroute_sources[0][4])
                        if reroute_targets:
                            reroute_target_xy = (reroute_targets[0][3], reroute_targets[0][4])

                        blockers = analyze_frontier_blocking(
                            blocked_cells, pcb_data, config, routed_net_paths,
                            exclude_net_ids={ripped_net_id},
                            target_xy=reroute_target_xy,
                            source_xy=reroute_source_xy,
                            obstacle_cache=obstacle_cache
                        )
                        print_blocking_analysis(blockers)

                        # Filter to only rippable blockers
                        rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
                            blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id
                        )
                        ripped_canonical_ids = set()
                        last_retry_blocked_cells = blocked_cells

                        for N in range(1, config.max_rip_up_count + 1):
                            if N > 1 and last_retry_blocked_cells:
                                print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                                fresh_blockers = analyze_frontier_blocking(
                                    last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                    exclude_net_ids={ripped_net_id},
                                    target_xy=reroute_target_xy,
                                    source_xy=reroute_source_xy,
                                    obstacle_cache=obstacle_cache
                                )
                                print_blocking_analysis(fresh_blockers, prefix="    ")
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
                                next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                                if next_canonical not in seen_canonical_ids:
                                    seen_canonical_ids.add(next_canonical)
                                    rippable_blockers.append(next_blocker)
                                for idx, b in enumerate(rippable_blockers):
                                    if get_canonical_net_id(b.net_id, diff_pair_by_net_id) == next_canonical:
                                        if idx != N - 1 and N - 1 < len(rippable_blockers):
                                            rippable_blockers[idx], rippable_blockers[N-1] = rippable_blockers[N-1], rippable_blockers[idx]
                                        break

                            if N > len(rippable_blockers):
                                break

                            blocker_canonicals = frozenset(
                                get_canonical_net_id(rippable_blockers[i].net_id, diff_pair_by_net_id)
                                for i in range(N)
                            )
                            if (ripped_net_id, blocker_canonicals) in rip_and_retry_history:
                                continue

                            rip_successful = True
                            new_ripped_this_level = []

                            if N == 1:
                                blocker = rippable_blockers[0]
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"  Ripping up diff pair {ripped_pair_name_tmp} to retry reroute...")
                                else:
                                    print(f"  Ripping up {blocker.net_name} to retry reroute...")
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
                                saved_result_tmp, ripped_ids, was_in_results = rip_up_net(
                                    blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                    routed_results, diff_pair_by_net_id, remaining_net_ids,
                                    results, config, track_proximity_cache,
                                    state.working_obstacles, state.net_obstacles_cache
                                )
                                if saved_result_tmp is None:
                                    rip_successful = False
                                    break
                                ripped_items.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                new_ripped_this_level.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                                # Invalidate obstacle cache for ripped nets
                                for rid in ripped_ids:
                                    invalidate_obstacle_cache(obstacle_cache, rid)
                                if was_in_results:
                                    successful -= 1

                            if not rip_successful:
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                    restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                               pcb_data, routed_net_ids, routed_net_paths,
                                               routed_results, diff_pair_by_net_id, remaining_net_ids,
                                               results, config, track_proximity_cache, layer_map,
                                               state.working_obstacles, state.net_obstacles_cache)
                                    if was_in_results:
                                        successful += 1
                                    ripped_items.pop()
                                continue

                            # Prepare obstacles in-place for retry (saves memory)
                            retry_via_cells = None
                            if state.working_obstacles is not None and state.net_obstacles_cache:
                                _, retry_via_cells = prepare_obstacles_inplace(
                                    state.working_obstacles, pcb_data, config, ripped_net_id,
                                    all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                                    state.net_obstacles_cache
                                )
                                retry_obstacles = state.working_obstacles
                            else:
                                retry_obstacles, _ = build_single_ended_obstacles(
                                    base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                                    all_unrouted_net_ids, ripped_net_id, gnd_net_id, track_proximity_cache, layer_map
                                )

                            # Check for multi-point net in retry as well
                            retry_multipoint_pads = get_multipoint_net_pads(pcb_data, ripped_net_id, config)
                            if retry_multipoint_pads:
                                retry_result = route_multipoint_main(pcb_data, ripped_net_id, config, retry_obstacles, retry_multipoint_pads)
                                if retry_result and not retry_result.get('failed') and retry_result.get('is_multipoint'):
                                    tap_result = route_multipoint_taps(pcb_data, ripped_net_id, config, retry_obstacles, retry_result)
                                    if tap_result:
                                        retry_result = tap_result
                                        # Remove from pending_multipoint_nets since Phase 3 is now complete
                                        if ripped_net_id in state.pending_multipoint_nets:
                                            del state.pending_multipoint_nets[ripped_net_id]
                            else:
                                retry_result = route_net_with_obstacles(pcb_data, ripped_net_id, config, retry_obstacles)

                            if retry_result and not retry_result.get('failed'):
                                routed_length = calculate_route_length(retry_result['new_segments'], retry_result.get('new_vias', []), pcb_data)
                                route_length = routed_length + stub_length
                                retry_result['route_length'] = route_length
                                retry_result['stub_length'] = stub_length
                                print(f"  REROUTE RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result.get('new_vias', []))} vias, length={route_length:.2f}mm (stubs={stub_length:.2f}mm)")
                                results.append(retry_result)
                                successful += 1
                                total_iterations += retry_result['iterations']
                                add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                                if ripped_net_id in remaining_net_ids:
                                    remaining_net_ids.remove(ripped_net_id)
                                routed_net_ids.append(ripped_net_id)
                                routed_results[ripped_net_id] = retry_result
                                if retry_result.get('path'):
                                    routed_net_paths[ripped_net_id] = retry_result['path']
                                track_proximity_cache[ripped_net_id] = compute_track_proximity_for_net(pcb_data, ripped_net_id, config, layer_map)
                                # Allow re-queuing if this net gets ripped again later
                                queued_net_ids.discard(ripped_net_id)

                                # Update net obstacles cache with new route, then restore working obstacles
                                if retry_via_cells is not None and state.working_obstacles is not None:
                                    update_net_obstacles_after_routing(pcb_data, ripped_net_id, retry_result, config, state.net_obstacles_cache)
                                    restore_obstacles_inplace(state.working_obstacles, ripped_net_id,
                                                             state.net_obstacles_cache, retry_via_cells)
                                    retry_via_cells = None
                                # Invalidate blocking analysis cache since we added segments
                                invalidate_obstacle_cache(obstacle_cache, ripped_net_id)

                                # Queue ripped nets and add to history
                                rip_and_retry_history.add((ripped_net_id, blocker_canonicals))
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in ripped_items:
                                    if was_in_results:
                                        successful -= 1
                                    if net_id_tmp in diff_pair_by_net_id:
                                        ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[net_id_tmp]
                                        canonical_id = ripped_pair_tmp.p_net_id
                                        if canonical_id not in queued_net_ids:
                                            reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                            queued_net_ids.add(canonical_id)
                                    else:
                                        if net_id_tmp not in queued_net_ids:
                                            net = pcb_data.nets.get(net_id_tmp)
                                            net_name_tmp = net.name if net else f"Net {net_id_tmp}"
                                            reroute_queue.append(('single', net_name_tmp, net_id_tmp))
                                            queued_net_ids.add(net_id_tmp)

                                reroute_succeeded = True
                                break
                            else:
                                print(f"  REROUTE RETRY FAILED (N={N})")

                                # Restore working obstacles after retry failure (in-place)
                                if retry_via_cells is not None and state.working_obstacles is not None:
                                    restore_obstacles_inplace(state.working_obstacles, ripped_net_id,
                                                             state.net_obstacles_cache, retry_via_cells)
                                    retry_via_cells = None

                                if retry_result:
                                    retry_fwd = retry_result.pop('blocked_cells_forward', [])
                                    retry_bwd = retry_result.pop('blocked_cells_backward', [])
                                    last_retry_blocked_cells = list(set(retry_fwd + retry_bwd))
                                    del retry_fwd, retry_bwd  # Free memory immediately
                                    if last_retry_blocked_cells:
                                        print(f"    Retry had {len(last_retry_blocked_cells)} blocked cells")
                                    else:
                                        print(f"    No blocked cells from retry to analyze")

                        # If all N levels failed, restore all ripped nets
                        if not reroute_succeeded and ripped_items:
                            print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                            for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(ripped_items):
                                restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config, track_proximity_cache, layer_map,
                                           state.working_obstacles, state.net_obstacles_cache)
                                if was_in_results:
                                    successful += 1

                if not reroute_succeeded:
                    if not ripped_items:
                        print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                    failed += 1

        elif reroute_item[0] == 'diff_pair':
            # Handle diff pairs that were ripped during single-ended routing
            _, ripped_pair_name, ripped_pair = reroute_item

            # Skip if already routed
            if ripped_pair.p_net_id in routed_results and ripped_pair.n_net_id in routed_results:
                continue

            route_index += 1
            current_total = total_routes + len(reroute_queue)
            failed_str = f" ({failed} failed)" if failed > 0 else ""
            print(f"\n[REROUTE {route_index}/{current_total}{failed_str}] Re-routing ripped diff pair {ripped_pair_name}")
            print("-" * 40)

            start_time = time.time()
            obstacles, unrouted_stubs = build_diff_pair_obstacles(
                diff_pair_base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                all_unrouted_net_ids, ripped_pair.p_net_id, ripped_pair.n_net_id, gnd_net_id,
                track_proximity_cache, layer_map, diff_pair_extra_clearance,
                add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair
            )

            # Get source/target coordinates for blocking analysis
            reroute_sources, reroute_targets, _ = get_diff_pair_endpoints(pcb_data, ripped_pair.p_net_id, ripped_pair.n_net_id, config)
            reroute_source_xy = None
            reroute_target_xy = None
            if reroute_sources:
                src = reroute_sources[0]
                reroute_source_xy = ((src[5] + src[7]) / 2, (src[6] + src[8]) / 2)
            if reroute_targets:
                tgt = reroute_targets[0]
                reroute_target_xy = ((tgt[5] + tgt[7]) / 2, (tgt[6] + tgt[8]) / 2)

            result = route_diff_pair_with_obstacles(pcb_data, ripped_pair, config, obstacles, base_obstacles, unrouted_stubs)
            elapsed = time.time() - start_time
            total_time += elapsed

            if result and not result.get('failed') and not result.get('probe_blocked'):
                print(f"  REROUTE SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias ({elapsed:.2f}s)")
                results.append(result)
                successful += 1
                total_iterations += result['iterations']
                rerouted_pairs.add(ripped_pair_name)
                apply_polarity_swap(pcb_data, result, pad_swaps, ripped_pair_name, polarity_swapped_pairs)
                record_diff_pair_success(
                    pcb_data, result, ripped_pair, ripped_pair_name, config,
                    remaining_net_ids, routed_net_ids, routed_net_paths, routed_results,
                    diff_pair_by_net_id, track_proximity_cache, layer_map
                )
                # Allow re-queuing if this pair gets ripped again later
                queued_net_ids.discard(ripped_pair.p_net_id)
                queued_net_ids.discard(ripped_pair.n_net_id)
            else:
                # Reroute failed - try rip-up and retry
                iterations = result['iterations'] if result else 0
                total_iterations += iterations
                print(f"  REROUTE FAILED: ({elapsed:.2f}s) - attempting rip-up and retry...")

                reroute_succeeded = False
                ripped_items = []
                if routed_net_paths and result:
                    # Find blocked cells from the failed route
                    # Pop to free memory from result dict (cells kept in local vars for fallback)
                    fwd_cells = result.pop('blocked_cells_forward', [])
                    bwd_cells = result.pop('blocked_cells_backward', [])
                    if result.get('probe_blocked'):
                        blocked_cells = result.pop('blocked_cells', [])
                    else:
                        fwd_iters = result.get('iterations_forward', 0)
                        bwd_iters = result.get('iterations_backward', 0)
                        if fwd_iters > 0 and (bwd_iters == 0 or fwd_iters <= bwd_iters):
                            blocked_cells = fwd_cells
                        else:
                            blocked_cells = bwd_cells

                    if not blocked_cells:
                        print(f"  No blocked cells from router - cannot analyze blockers")

                    if blocked_cells:
                        blockers = analyze_frontier_blocking(
                            blocked_cells, pcb_data, config, routed_net_paths,
                            exclude_net_ids={ripped_pair.p_net_id, ripped_pair.n_net_id},
                            extra_clearance=diff_pair_extra_clearance,
                            target_xy=reroute_target_xy,
                            source_xy=reroute_source_xy,
                            obstacle_cache=obstacle_cache
                        )
                        print_blocking_analysis(blockers)

                        # Filter to only rippable blockers and deduplicate by diff pair
                        rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
                            blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id
                        )
                        current_canonical = ripped_pair.p_net_id

                        if blockers and not rippable_blockers:
                            unrippable_names = []
                            for b in blockers[:3]:
                                if b.net_id in diff_pair_by_net_id:
                                    unrippable_names.append(diff_pair_by_net_id[b.net_id][0])
                                else:
                                    unrippable_names.append(b.net_name)
                            print(f"  No rippable blockers (not yet routed): {', '.join(unrippable_names)}")
                        ripped_canonical_ids = set()
                        last_retry_blocked_cells = blocked_cells

                        for N in range(1, config.max_rip_up_count + 1):
                            if N > 1 and last_retry_blocked_cells:
                                print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                                fresh_blockers = analyze_frontier_blocking(
                                    last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                    exclude_net_ids={ripped_pair.p_net_id, ripped_pair.n_net_id},
                                    extra_clearance=diff_pair_extra_clearance,
                                    target_xy=reroute_target_xy,
                                    source_xy=reroute_source_xy,
                                    obstacle_cache=obstacle_cache
                                )
                                print_blocking_analysis(fresh_blockers, prefix="    ")
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
                                next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                                if next_canonical not in seen_canonical_ids:
                                    seen_canonical_ids.add(next_canonical)
                                    rippable_blockers.append(next_blocker)
                                for idx, b in enumerate(rippable_blockers):
                                    if get_canonical_net_id(b.net_id, diff_pair_by_net_id) == next_canonical:
                                        if idx != N - 1 and N - 1 < len(rippable_blockers):
                                            rippable_blockers[idx], rippable_blockers[N-1] = rippable_blockers[N-1], rippable_blockers[idx]
                                        break

                            if N > len(rippable_blockers):
                                break

                            blocker_canonicals = frozenset(
                                get_canonical_net_id(rippable_blockers[i].net_id, diff_pair_by_net_id)
                                for i in range(N)
                            )
                            if (current_canonical, blocker_canonicals) in rip_and_retry_history:
                                blocker_names = []
                                for i in range(N):
                                    b = rippable_blockers[i]
                                    if b.net_id in diff_pair_by_net_id:
                                        blocker_names.append(diff_pair_by_net_id[b.net_id][0])
                                    else:
                                        blocker_names.append(b.net_name)
                                print(f"  Skipping rip-up (already tried): {', '.join(blocker_names)}")
                                continue

                            # Rip up blockers
                            rip_successful = True
                            new_ripped_this_level = []

                            if N == 1:
                                blocker = rippable_blockers[0]
                                if blocker.net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, _ = diff_pair_by_net_id[blocker.net_id]
                                    print(f"  Ripping up diff pair {ripped_pair_name_tmp} to retry reroute...")
                                else:
                                    print(f"  Ripping up {blocker.net_name} to retry reroute...")
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
                                saved_result_tmp, ripped_ids, was_in_results = rip_up_net(
                                    blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                                    routed_results, diff_pair_by_net_id, remaining_net_ids,
                                    results, config, track_proximity_cache,
                                    state.working_obstacles, state.net_obstacles_cache
                                )
                                if saved_result_tmp is None:
                                    rip_successful = False
                                    break
                                ripped_items.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                new_ripped_this_level.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                                # Invalidate obstacle cache for ripped nets
                                for rid in ripped_ids:
                                    invalidate_obstacle_cache(obstacle_cache, rid)
                                if was_in_results:
                                    successful -= 1

                            if not rip_successful:
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                    restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                               pcb_data, routed_net_ids, routed_net_paths,
                                               routed_results, diff_pair_by_net_id, remaining_net_ids,
                                               results, config, track_proximity_cache, layer_map,
                                               state.working_obstacles, state.net_obstacles_cache)
                                    if was_in_results:
                                        successful += 1
                                    ripped_items.pop()
                                continue

                            # Rebuild obstacles and retry
                            retry_obstacles, unrouted_stubs = build_diff_pair_obstacles(
                                diff_pair_base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                                all_unrouted_net_ids, ripped_pair.p_net_id, ripped_pair.n_net_id, gnd_net_id,
                                track_proximity_cache, layer_map, diff_pair_extra_clearance,
                                add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair
                            )

                            retry_result = route_diff_pair_with_obstacles(pcb_data, ripped_pair, config, retry_obstacles, base_obstacles, unrouted_stubs)

                            if retry_result and not retry_result.get('failed') and not retry_result.get('probe_blocked'):
                                print(f"  REROUTE RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result['new_vias'])} vias")
                                results.append(retry_result)
                                successful += 1
                                total_iterations += retry_result['iterations']
                                rerouted_pairs.add(ripped_pair_name)
                                apply_polarity_swap(pcb_data, retry_result, pad_swaps, ripped_pair_name, polarity_swapped_pairs)
                                add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                                if ripped_pair.p_net_id in remaining_net_ids:
                                    remaining_net_ids.remove(ripped_pair.p_net_id)
                                if ripped_pair.n_net_id in remaining_net_ids:
                                    remaining_net_ids.remove(ripped_pair.n_net_id)
                                routed_net_ids.append(ripped_pair.p_net_id)
                                routed_net_ids.append(ripped_pair.n_net_id)
                                if retry_result.get('p_path'):
                                    routed_net_paths[ripped_pair.p_net_id] = retry_result['p_path']
                                if retry_result.get('n_path'):
                                    routed_net_paths[ripped_pair.n_net_id] = retry_result['n_path']
                                routed_results[ripped_pair.p_net_id] = retry_result
                                routed_results[ripped_pair.n_net_id] = retry_result
                                diff_pair_by_net_id[ripped_pair.p_net_id] = (ripped_pair_name, ripped_pair)
                                diff_pair_by_net_id[ripped_pair.n_net_id] = (ripped_pair_name, ripped_pair)
                                track_proximity_cache[ripped_pair.p_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.p_net_id, config, layer_map)
                                track_proximity_cache[ripped_pair.n_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.n_net_id, config, layer_map)
                                # Allow re-queuing if this pair gets ripped again later
                                queued_net_ids.discard(ripped_pair.p_net_id)
                                queued_net_ids.discard(ripped_pair.n_net_id)
                                # Invalidate blocking analysis cache since we added segments
                                invalidate_obstacle_cache(obstacle_cache, ripped_pair.p_net_id)
                                invalidate_obstacle_cache(obstacle_cache, ripped_pair.n_net_id)

                                # Queue ripped nets and add to history
                                rip_and_retry_history.add((current_canonical, blocker_canonicals))
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in ripped_items:
                                    if was_in_results:
                                        successful -= 1
                                    if net_id_tmp in diff_pair_by_net_id:
                                        ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[net_id_tmp]
                                        canonical_id = ripped_pair_tmp.p_net_id
                                        if canonical_id not in queued_net_ids:
                                            reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                            queued_net_ids.add(canonical_id)
                                    else:
                                        if net_id_tmp not in queued_net_ids:
                                            net = pcb_data.nets.get(net_id_tmp)
                                            net_name_tmp = net.name if net else f"Net {net_id_tmp}"
                                            reroute_queue.append(('single', net_name_tmp, net_id_tmp))
                                            queued_net_ids.add(net_id_tmp)

                                reroute_succeeded = True
                                break
                            else:
                                print(f"  REROUTE RETRY FAILED (N={N})")
                                if retry_result:
                                    if retry_result.get('probe_blocked'):
                                        last_retry_blocked_cells = retry_result.pop('blocked_cells', [])
                                    else:
                                        retry_fwd = retry_result.pop('blocked_cells_forward', [])
                                        retry_bwd = retry_result.pop('blocked_cells_backward', [])
                                        last_retry_blocked_cells = list(set(retry_fwd + retry_bwd))
                                        del retry_fwd, retry_bwd  # Free memory immediately
                                    if last_retry_blocked_cells:
                                        print(f"    Retry had {len(last_retry_blocked_cells)} blocked cells")
                                    else:
                                        print(f"    No blocked cells from retry to analyze")
                                else:
                                    print(f"    Retry returned no result (endpoint error?)")
                                    last_retry_blocked_cells = []

                        # If all N levels failed, restore all ripped nets
                        if not reroute_succeeded and ripped_items:
                            print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                            for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(ripped_items):
                                restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                           pcb_data, routed_net_ids, routed_net_paths,
                                           routed_results, diff_pair_by_net_id, remaining_net_ids,
                                           results, config, track_proximity_cache, layer_map,
                                           state.working_obstacles, state.net_obstacles_cache)
                                if was_in_results:
                                    successful += 1

                # Fallback layer swap for setback failures in reroute (after rip-up fails)
                # Note: fwd_cells and bwd_cells already extracted above
                if not reroute_succeeded and enable_layer_switch and result:
                    fwd_iters = result.get('iterations_forward', 0)
                    bwd_iters = result.get('iterations_backward', 0)
                    is_setback_failure = (fwd_iters == 0 and bwd_iters == 0 and (fwd_cells or bwd_cells))

                    if is_setback_failure:
                        print(f"  Trying fallback layer swap for setback failure...")
                        swap_success, swap_result, _, _ = try_fallback_layer_swap(
                            pcb_data, ripped_pair, ripped_pair_name, config,
                            fwd_cells, bwd_cells,
                            diff_pair_base_obstacles, base_obstacles,
                            routed_net_ids, remaining_net_ids,
                            all_unrouted_net_ids, gnd_net_id,
                            track_proximity_cache, diff_pair_extra_clearance,
                            all_swap_vias, all_segment_modifications,
                            None, None,  # all_stubs_by_layer, stub_endpoints_by_layer
                            routed_net_paths, routed_results, diff_pair_by_net_id, layer_map,
                            target_swaps, results=results, obstacle_cache=obstacle_cache)

                        if swap_success and swap_result:
                            print(f"  {GREEN}FALLBACK LAYER SWAP SUCCESS{RESET}")
                            results.append(swap_result)
                            successful += 1
                            total_iterations += swap_result['iterations']
                            rerouted_pairs.add(ripped_pair_name)

                            apply_polarity_swap(pcb_data, swap_result, pad_swaps, ripped_pair_name, polarity_swapped_pairs)
                            add_route_to_pcb_data(pcb_data, swap_result, debug_lines=config.debug_lines)
                            if ripped_pair.p_net_id in remaining_net_ids:
                                remaining_net_ids.remove(ripped_pair.p_net_id)
                            if ripped_pair.n_net_id in remaining_net_ids:
                                remaining_net_ids.remove(ripped_pair.n_net_id)
                            routed_net_ids.append(ripped_pair.p_net_id)
                            routed_net_ids.append(ripped_pair.n_net_id)
                            track_proximity_cache[ripped_pair.p_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.p_net_id, config, layer_map)
                            track_proximity_cache[ripped_pair.n_net_id] = compute_track_proximity_for_net(pcb_data, ripped_pair.n_net_id, config, layer_map)
                            if swap_result.get('p_path'):
                                routed_net_paths[ripped_pair.p_net_id] = swap_result['p_path']
                            if swap_result.get('n_path'):
                                routed_net_paths[ripped_pair.n_net_id] = swap_result['n_path']
                            routed_results[ripped_pair.p_net_id] = swap_result
                            routed_results[ripped_pair.n_net_id] = swap_result
                            diff_pair_by_net_id[ripped_pair.p_net_id] = (ripped_pair_name, ripped_pair)
                            diff_pair_by_net_id[ripped_pair.n_net_id] = (ripped_pair_name, ripped_pair)
                            # Invalidate blocking analysis cache since we added segments
                            invalidate_obstacle_cache(obstacle_cache, ripped_pair.p_net_id)
                            invalidate_obstacle_cache(obstacle_cache, ripped_pair.n_net_id)
                            reroute_succeeded = True

                if not reroute_succeeded:
                    if not ripped_items:
                        print(f"  {RED}REROUTE FAILED - could not find route{RESET}")
                    failed += 1

    return successful, failed, total_time, total_iterations, route_index
