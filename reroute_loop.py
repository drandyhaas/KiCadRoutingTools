"""
Reroute loop for ripped-up nets.

This module contains the unified reroute loop that handles all nets ripped
during diff pair or single-ended routing, extracted from route.py.
"""
from __future__ import annotations

import time
from typing import List, Tuple

from routing_state import (RoutingState, record_net_event, record_rip_ancestry,
                           record_pair_rip_ancestry, rip_exclude_set,
                           diff_pair_rip_exclude)
from obstacle_costs import compute_track_proximity_for_net
from connectivity import get_net_endpoints, calculate_stub_length, get_multipoint_net_pads
from net_queries import calculate_route_length
from pcb_modification import add_route_to_pcb_data
from single_ended_routing import route_net_with_obstacles, route_multipoint_main, route_multipoint_taps
from diff_pair_routing import (route_diff_pair_with_obstacles, get_diff_pair_endpoints,
                               _route_direct_coupled_middle)
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers, invalidate_obstacle_cache
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import apply_polarity_swap, get_canonical_net_id, rip_combo_already_tried
from layer_swap_fallback import try_fallback_layer_swap, add_own_stubs_as_obstacles_for_diff_pair
from routing_context import (
    build_single_ended_obstacles, build_diff_pair_obstacles,
    build_diff_pair_leg_obstacles,
    record_single_ended_success, record_diff_pair_success,
    prepare_obstacles_inplace, restore_obstacles_inplace
)
from obstacle_cache import update_net_obstacles_after_routing
from diff_pair_custody import (classify_diff_pair_failure, record_casualty,
                               record_pair_diag, blocker_names)
from terminal_colors import RED, GREEN, RESET


def run_reroute_loop(
    state: RoutingState,
    route_index_start: int = 0,
    cancel_check=None,
    progress_callback=None,
    failed_so_far: int = 0,
) -> Tuple[int, int, float, int, int]:
    """
    Run the unified reroute loop for all nets ripped during routing.

    Args:
        state: The routing state object containing all shared state
        route_index_start: Starting route index (continues from previous routing phases)
        cancel_check: Optional callable returning True if routing should be cancelled
        progress_callback: Optional callable(current, total, net_name) for progress updates
        failed_so_far: Number of failed routes from previous routing phases

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
        # Check for cancellation at start of each reroute iteration
        if cancel_check and cancel_check():
            print("\nReroute cancelled by user")
            break

        reroute_item = reroute_queue[reroute_index]
        reroute_index += 1

        if reroute_item[0] == 'single':
            _, ripped_net_name, ripped_net_id = reroute_item

            # Skip if already routed
            if ripped_net_id in routed_results:
                continue

            route_index += 1
            current_total = total_routes + len(reroute_queue)
            total_failed = failed_so_far + failed
            failed_str = f" ({total_failed} failed)" if total_failed > 0 else ""
            print(f"\n[REROUTE {route_index}/{current_total}{failed_str}] Re-routing ripped net {ripped_net_name}")
            print("-" * 40)

            # Update progress for reroute
            if progress_callback:
                msg = f"Reroute: {ripped_net_name}"
                if total_failed > 0:
                    msg += f" ({total_failed} failed)"
                progress_callback(route_index, current_total, msg)

            # Calculate stub length before routing
            stub_length = calculate_stub_length(pcb_data, ripped_net_id)

            start_time = time.time()
            # Use in-place approach if working obstacles available (saves memory)
            reroute_via_cells = None
            if state.working_obstacles is not None and state.net_obstacles_cache:
                unrouted_stubs, reroute_via_cells = prepare_obstacles_inplace(
                    state.working_obstacles, pcb_data, config, ripped_net_id,
                    all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                    state.net_obstacles_cache,
                    state.ripped_route_layer_costs, state.ripped_route_via_positions
                )
                obstacles = state.working_obstacles
            else:
                obstacles, unrouted_stubs = build_single_ended_obstacles(
                    base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                    all_unrouted_net_ids, ripped_net_id, gnd_net_id, track_proximity_cache, layer_map,
                    ripped_route_layer_costs=state.ripped_route_layer_costs,
                    ripped_route_via_positions=state.ripped_route_via_positions
                )

            # A ripped bus member reroutes WITH its corridor (or a routed
            # neighbor); previously it rerouted blind, so every rip degraded
            # the river. Hoisted above the multipoint split so multipoint
            # members get corridor-spanning main-edge selection + attraction.
            from bus_detection import bus_attraction_context
            _attr, _rev = bus_attraction_context(
                ripped_net_id, getattr(state, 'bus_net_to_group', None),
                getattr(state, 'bus_corridors', None), routed_net_paths)
            # Check for multi-point net (3+ pads, no existing segments since they were ripped)
            multipoint_pads = get_multipoint_net_pads(pcb_data, ripped_net_id, config)
            if multipoint_pads:
                print(f"  Multi-point net with {len(multipoint_pads)} pads - routing main + taps")
                result = route_multipoint_main(pcb_data, ripped_net_id, config, obstacles, multipoint_pads,
                                               attraction_path=_attr)
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
                        # Print red final failure message if there are unconnected pads
                        final_failed_pads = tap_result.get('failed_pads_info', [])
                        if final_failed_pads:
                            net_name = pcb_data.nets[ripped_net_id].name if ripped_net_id in pcb_data.nets else f"Net {ripped_net_id}"
                            for pad in final_failed_pads:
                                print(f"    {RED}NOT CONNECTED (so far): {net_name} - {pad['component_ref']} pad {pad['pad_number']} at ({pad['x']:.2f}, {pad['y']:.2f}) - may still be recovered; final verdict in the end-of-run summary{RESET}")

                        # Remove from pending_multipoint_nets since Phase 3 is now complete
                        if ripped_net_id in state.pending_multipoint_nets:
                            del state.pending_multipoint_nets[ripped_net_id]
            else:
                result = route_net_with_obstacles(pcb_data, ripped_net_id, config, obstacles,
                                                  attraction_path=_attr,
                                                  reverse_direction=_rev)
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
                record_net_event(state, ripped_net_id, "reroute_succeeded", {
                    "context": "reroute_loop",
                    "segments": len(result['new_segments']),
                    "vias": len(result.get('new_vias', []))
                })
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
                    # Fastest-failing direction only (audit #3b): pooling let
                    # the broad flood's thousands of cells swamp the drained
                    # pocket's dozens before attribution ran. Mirrors the SE
                    # loop; falls back to the union when iterations are absent.
                    fwd_it = result.get('iterations_forward', 0)
                    bwd_it = result.get('iterations_backward', 0)
                    if fwd_cells and bwd_cells and (fwd_it > 0 or bwd_it > 0):
                        blocked_cells = list(set(
                            fwd_cells if (fwd_it > 0 and (bwd_it == 0 or fwd_it <= bwd_it))
                            else bwd_cells))
                    else:
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
                            exclude_net_ids=rip_exclude_set(state, ripped_net_id),
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
                                    exclude_net_ids=rip_exclude_set(state, ripped_net_id),
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

                            # Shared rip-history gate (#376): same log+skip as
                            # single_ended_loop (was a silent bare continue here).
                            already_tried, blocker_canonicals = rip_combo_already_tried(
                                rip_and_retry_history, ripped_net_id, ripped_net_name,
                                rippable_blockers, N, diff_pair_by_net_id)
                            if already_tried:
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
                                    state.working_obstacles, state.net_obstacles_cache,
                                    state.ripped_route_layer_costs, state.ripped_route_via_positions,
                                    layer_map
                                )
                                if saved_result_tmp is None:
                                    rip_successful = False
                                    break
                                ripped_items.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                new_ripped_this_level.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                                # Invalidate obstacle cache for ripped nets and record rip events
                                for rid in ripped_ids:
                                    invalidate_obstacle_cache(obstacle_cache, rid)
                                    record_rip_ancestry(state, ripped_net_id, rid)
                                    record_net_event(state, rid, "ripped_by", {
                                        "ripping_net_id": ripped_net_id,
                                        "ripping_net_name": ripped_net_name,
                                        "reason": f"reroute_loop rip-up retry N={N}",
                                        "N": N
                                    })
                                if was_in_results:
                                    successful -= 1

                            if not rip_successful:
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                    restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                               pcb_data, routed_net_ids, routed_net_paths,
                                               routed_results, diff_pair_by_net_id, remaining_net_ids,
                                               results, config, track_proximity_cache, layer_map,
                                               state.working_obstacles, state.net_obstacles_cache,
                                               state.ripped_route_layer_costs, state.ripped_route_via_positions, refused_sink=state.collision_refused_net_ids)
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
                                    state.net_obstacles_cache,
                                    state.ripped_route_layer_costs, state.ripped_route_via_positions
                                )
                                retry_obstacles = state.working_obstacles
                            else:
                                retry_obstacles, _ = build_single_ended_obstacles(
                                    base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                                    all_unrouted_net_ids, ripped_net_id, gnd_net_id, track_proximity_cache, layer_map,
                                    ripped_route_layer_costs=state.ripped_route_layer_costs,
                                    ripped_route_via_positions=state.ripped_route_via_positions
                                )

                            # Bus attraction for the retry (multipoint too)
                            from bus_detection import bus_attraction_context
                            _attr, _rev = bus_attraction_context(
                                ripped_net_id, getattr(state, 'bus_net_to_group', None),
                                getattr(state, 'bus_corridors', None), routed_net_paths)
                            # Check for multi-point net in retry as well
                            retry_multipoint_pads = get_multipoint_net_pads(pcb_data, ripped_net_id, config)
                            if retry_multipoint_pads:
                                retry_result = route_multipoint_main(pcb_data, ripped_net_id, config, retry_obstacles, retry_multipoint_pads,
                                                                     attraction_path=_attr)
                                if retry_result and not retry_result.get('failed') and retry_result.get('is_multipoint'):
                                    tap_result = route_multipoint_taps(pcb_data, ripped_net_id, config, retry_obstacles, retry_result)
                                    if tap_result:
                                        retry_result = tap_result
                                        # Print red final failure message if there are unconnected pads
                                        final_failed_pads = tap_result.get('failed_pads_info', [])
                                        if final_failed_pads:
                                            net_name = pcb_data.nets[ripped_net_id].name if ripped_net_id in pcb_data.nets else f"Net {ripped_net_id}"
                                            for pad in final_failed_pads:
                                                print(f"    {RED}NOT CONNECTED (so far): {net_name} - {pad['component_ref']} pad {pad['pad_number']} at ({pad['x']:.2f}, {pad['y']:.2f}) - may still be recovered; final verdict in the end-of-run summary{RESET}")

                                        # Remove from pending_multipoint_nets since Phase 3 is now complete
                                        if ripped_net_id in state.pending_multipoint_nets:
                                            del state.pending_multipoint_nets[ripped_net_id]
                            else:
                                retry_result = route_net_with_obstacles(pcb_data, ripped_net_id, config, retry_obstacles,
                                                                        attraction_path=_attr,
                                                                        reverse_direction=_rev)

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
                                record_net_event(state, ripped_net_id, "reroute_succeeded", {
                                    "context": "reroute_loop",
                                    "N": N,
                                    "segments": len(retry_result['new_segments']),
                                    "vias": len(retry_result.get('new_vias', []))
                                })
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
                                    # Committed rip: custody of the pre-rip copper
                                    # for the casualties-only final reconcile.
                                    record_casualty(state, net_id_tmp, saved_result_tmp,
                                                    ripped_ids, was_in_results)
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
                                           state.working_obstacles, state.net_obstacles_cache,
                                           state.ripped_route_layer_costs, state.ripped_route_via_positions, refused_sink=state.collision_refused_net_ids)
                                if was_in_results:
                                    successful += 1

                if not reroute_succeeded:
                    if not ripped_items:
                        print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                        from routing_diagnostics import static_boxin_hint
                        hint = static_boxin_hint(result, config, pcb_data)
                        if hint:
                            print(f"  {hint}")
                    # Remove from pending_multipoint_nets to prevent Phase 3 from
                    # trying to route taps for a net with no main route.
                    if ripped_net_id in state.pending_multipoint_nets:
                        del state.pending_multipoint_nets[ripped_net_id]
                    # #468: a terminally-failed rip victim must not ship with
                    # LESS copper than it started with. Conflict-free saved
                    # copper -> FULL restore (the net returns to its pre-rip
                    # routed state); else at least the escape stub survives.
                    from rip_restore import try_terminal_restore
                    _tr = try_terminal_restore(
                        pcb_data, config, ripped_net_id,
                        working_obstacles=state.working_obstacles,
                        net_obstacles_cache=state.net_obstacles_cache)
                    if _tr == 'full':
                        _sv, _rids, _wir = pcb_data._rip_saved[ripped_net_id]
                        restore_net(ripped_net_id, _sv, _rids, _wir,
                                    pcb_data, routed_net_ids, routed_net_paths,
                                    routed_results, diff_pair_by_net_id,
                                    remaining_net_ids, results, config,
                                    track_proximity_cache, layer_map,
                                    state.working_obstacles, state.net_obstacles_cache,
                                    state.ripped_route_layer_costs,
                                    state.ripped_route_via_positions,
                                    refused_sink=state.collision_refused_net_ids)
                        print(f"  RIP-RESTORE (#468): {ripped_net_name} restored "
                              f"to its pre-rip route (reroute failed, corridor "
                              f"still clear)")
                        if _wir:
                            successful += 1
                    else:
                        failed += 1

        elif reroute_item[0] == 'diff_pair':
            # Handle diff pairs that were ripped during single-ended routing
            _, ripped_pair_name, ripped_pair = reroute_item

            # Skip if already routed
            if ripped_pair.p_net_id in routed_results and ripped_pair.n_net_id in routed_results:
                continue

            route_index += 1
            current_total = total_routes + len(reroute_queue)
            total_failed = failed_so_far + failed
            failed_str = f" ({total_failed} failed)" if total_failed > 0 else ""
            print(f"\n[REROUTE {route_index}/{current_total}{failed_str}] Re-routing ripped diff pair {ripped_pair_name}")
            print("-" * 40)

            # Update progress for diff pair reroute
            if progress_callback:
                msg = f"Reroute: {ripped_pair_name}"
                if total_failed > 0:
                    msg += f" ({total_failed} failed)"
                progress_callback(route_index, current_total, msg)

            start_time = time.time()
            obstacles, unrouted_stubs = build_diff_pair_obstacles(
                diff_pair_base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                all_unrouted_net_ids, ripped_pair.p_net_id, ripped_pair.n_net_id, gnd_net_id,
                track_proximity_cache, layer_map, diff_pair_extra_clearance,
                add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions
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
                if result.get('polarity_swap_denied'):
                    state.polarity_swap_denied_pairs.add(ripped_pair_name)
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
                # Classify BEFORE the pops below strip the evidence (per-pair
                # JSON diagnostics; refined to 'congestion' if blockers found).
                rr_fail_reason = classify_diff_pair_failure(result)
                rr_fail_blockers = []

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
                            exclude_net_ids=diff_pair_rip_exclude(
                                state, ripped_pair.p_net_id, ripped_pair.n_net_id),
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
                        if rippable_blockers:
                            rr_fail_reason = 'congestion'
                            rr_fail_blockers = blocker_names(rippable_blockers,
                                                             diff_pair_by_net_id)
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
                                    exclude_net_ids=diff_pair_rip_exclude(
                                        state, ripped_pair.p_net_id, ripped_pair.n_net_id),
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

                            # Shared rip-history gate (#376): unify the log+skip
                            # with single_ended_loop (this site had a divergent
                            # "Skipping rip-up (already tried)" message -- drift).
                            already_tried, blocker_canonicals = rip_combo_already_tried(
                                rip_and_retry_history, current_canonical, ripped_pair_name,
                                rippable_blockers, N, diff_pair_by_net_id)
                            if already_tried:
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
                                    state.working_obstacles, state.net_obstacles_cache,
                                    state.ripped_route_layer_costs, state.ripped_route_via_positions,
                                    layer_map
                                )
                                if saved_result_tmp is None:
                                    rip_successful = False
                                    break
                                ripped_items.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                new_ripped_this_level.append((blocker.net_id, saved_result_tmp, ripped_ids, was_in_results))
                                ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
                                # Invalidate obstacle cache for ripped nets and record rip events
                                for rid in ripped_ids:
                                    invalidate_obstacle_cache(obstacle_cache, rid)
                                    # Cycle guard (issue #219): the rerouting pair
                                    # ripped rid, so rid must not rip either half back.
                                    record_pair_rip_ancestry(state, ripped_pair.p_net_id,
                                                             ripped_pair.n_net_id, rid)
                                    record_net_event(state, rid, "ripped_by", {
                                        "ripping_net_id": ripped_pair.p_net_id,
                                        "ripping_net_name": ripped_pair_name,
                                        "reason": f"reroute_loop diff-pair rip-up retry N={N}",
                                        "N": N
                                    })
                                if was_in_results:
                                    successful -= 1

                            if not rip_successful:
                                for net_id_tmp, saved_result_tmp, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                    restore_net(net_id_tmp, saved_result_tmp, ripped_ids, was_in_results,
                                               pcb_data, routed_net_ids, routed_net_paths,
                                               routed_results, diff_pair_by_net_id, remaining_net_ids,
                                               results, config, track_proximity_cache, layer_map,
                                               state.working_obstacles, state.net_obstacles_cache,
                                               state.ripped_route_layer_costs, state.ripped_route_via_positions, refused_sink=state.collision_refused_net_ids)
                                    if was_in_results:
                                        successful += 1
                                    ripped_items.pop()
                                continue

                            # Rebuild obstacles and retry
                            retry_obstacles, unrouted_stubs = build_diff_pair_obstacles(
                                diff_pair_base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                                all_unrouted_net_ids, ripped_pair.p_net_id, ripped_pair.n_net_id, gnd_net_id,
                                track_proximity_cache, layer_map, diff_pair_extra_clearance,
                                add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair,
                                ripped_route_layer_costs=state.ripped_route_layer_costs,
                                ripped_route_via_positions=state.ripped_route_via_positions
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
                                    # Committed rip: custody of the pre-rip copper
                                    # for the casualties-only final reconcile.
                                    record_casualty(state, net_id_tmp, saved_result_tmp,
                                                    ripped_ids, was_in_results)
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
                                           state.working_obstacles, state.net_obstacles_cache,
                                           state.ripped_route_layer_costs, state.ripped_route_via_positions, refused_sink=state.collision_refused_net_ids)
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

                # Last-resort hybrid (issue #244): a rip-reroute casualty whose lane
                # is gone and whose standard probe + rip-up both dead-ended can still
                # have a coupled-middle + single-ended-leg escape (often to an inner
                # layer). The MAIN routing loop already tries this on a first-pass
                # failure (diff_pair_loop.py); the reroute path did not, so a pair
                # that only fails AFTER being ripped (butterstick /GIGABIT-0/ETH_B,
                # the /SYZYGY1.D1 class) failed honestly instead of taking the same
                # fallback. Rebuild the obstacle maps fresh -- rip-up/restore mutated
                # pcb_data since the reroute's were built. Returns None when it can't
                # lay a clean route, so this can't make things worse.
                if not reroute_succeeded and config.diff_pair_hybrid_escape:
                    hyb_obstacles, _ = build_diff_pair_obstacles(
                        diff_pair_base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                        all_unrouted_net_ids, ripped_pair.p_net_id, ripped_pair.n_net_id, gnd_net_id,
                        track_proximity_cache, layer_map, diff_pair_extra_clearance,
                        add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair,
                        ripped_route_layer_costs=state.ripped_route_layer_costs,
                        ripped_route_via_positions=state.ripped_route_via_positions)
                    # Single-ended leg clearance map. See build_diff_pair_leg_obstacles
                    # for why this uses base_obstacles + 0 clearance (not the coupled
                    # diff_pair_base_obstacles / extra clearance the middle map above
                    # uses) -- previously this site built it inconsistently (#246 review).
                    leg_obstacles = build_diff_pair_leg_obstacles(
                        base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                        all_unrouted_net_ids, ripped_pair.p_net_id, ripped_pair.n_net_id,
                        gnd_net_id, track_proximity_cache, layer_map)
                    hyb = _route_direct_coupled_middle(
                        pcb_data, ripped_pair, config, hyb_obstacles, config.layers,
                        leg_obstacles=leg_obstacles)
                    if hyb and not hyb.get('failed'):
                        print(f"  {GREEN}HYBRID ESCAPE (reroute): direct coupled middle "
                              f"+ point-to-point terminal legs{RESET}")
                        results.append(hyb)
                        successful += 1
                        total_iterations += hyb.get('iterations', 0)
                        rerouted_pairs.add(ripped_pair_name)
                        record_diff_pair_success(
                            pcb_data, hyb, ripped_pair, ripped_pair_name, config,
                            remaining_net_ids, routed_net_ids, routed_net_paths, routed_results,
                            diff_pair_by_net_id, track_proximity_cache, layer_map)
                        queued_net_ids.discard(ripped_pair.p_net_id)
                        queued_net_ids.discard(ripped_pair.n_net_id)
                        invalidate_obstacle_cache(obstacle_cache, ripped_pair.p_net_id)
                        invalidate_obstacle_cache(obstacle_cache, ripped_pair.n_net_id)
                        reroute_succeeded = True

                if not reroute_succeeded:
                    if not ripped_items:
                        print(f"  {RED}REROUTE FAILED - could not find route{RESET}")
                    record_pair_diag(state, ripped_pair_name, outcome='failed',
                                     reason=rr_fail_reason, stage='reroute',
                                     blocking_nets=rr_fail_blockers or None)
                    # Remove from pending_multipoint_nets to prevent Phase 3 from
                    # trying to route taps for a net with no main route.
                    if ripped_pair.p_net_id in state.pending_multipoint_nets:
                        del state.pending_multipoint_nets[ripped_pair.p_net_id]
                    if ripped_pair.n_net_id in state.pending_multipoint_nets:
                        del state.pending_multipoint_nets[ripped_pair.n_net_id]
                    # #468: same terminal-restore leg for pair victims (the
                    # payload is registered under both member ids; restore_net
                    # restores the whole pair from either).
                    from rip_restore import try_terminal_restore
                    _tr = try_terminal_restore(
                        pcb_data, config, ripped_pair.p_net_id,
                        working_obstacles=state.working_obstacles,
                        net_obstacles_cache=state.net_obstacles_cache)
                    if _tr == 'full':
                        _sv, _rids, _wir = pcb_data._rip_saved[ripped_pair.p_net_id]
                        restore_net(ripped_pair.p_net_id, _sv, _rids, _wir,
                                    pcb_data, routed_net_ids, routed_net_paths,
                                    routed_results, diff_pair_by_net_id,
                                    remaining_net_ids, results, config,
                                    track_proximity_cache, layer_map,
                                    state.working_obstacles, state.net_obstacles_cache,
                                    state.ripped_route_layer_costs,
                                    state.ripped_route_via_positions,
                                    refused_sink=state.collision_refused_net_ids)
                        print(f"  RIP-RESTORE (#468): pair {ripped_pair_name} "
                              f"restored to its pre-rip route")
                        if _wir:
                            successful += 1
                    else:
                        failed += 1

    return successful, failed, total_time, total_iterations, route_index
