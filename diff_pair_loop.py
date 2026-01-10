"""
Differential pair routing loop.

This module contains the main loop for routing differential pairs,
extracted from route.py for better maintainability.
"""

import time
from typing import List, Tuple

from routing_state import RoutingState
from routing_config import DiffPairNet
from memory_debug import get_process_memory_mb, estimate_track_proximity_cache_mb
from obstacle_map import (
    add_net_stubs_as_obstacles, add_net_vias_as_obstacles, add_net_pads_as_obstacles,
    add_same_net_via_clearance, add_same_net_pad_drill_via_clearance,
    add_stub_proximity_costs, merge_track_proximity_costs,
    add_cross_layer_tracks, compute_track_proximity_for_net
)
from routing_utils import get_stub_endpoints, get_chip_pad_positions, add_route_to_pcb_data, calculate_route_length
from diff_pair_routing import route_diff_pair_with_obstacles, get_diff_pair_endpoints
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import apply_polarity_swap, get_canonical_net_id
from layer_swap_fallback import try_fallback_layer_swap, add_own_stubs_as_obstacles_for_diff_pair
from routing_context import build_diff_pair_obstacles, restore_ripped_net

# ANSI color codes
RED = '\033[91m'
GREEN = '\033[92m'
RESET = '\033[0m'


def route_diff_pairs(
    state: RoutingState,
    diff_pair_ids_to_route: List[Tuple[str, DiffPairNet]],
) -> Tuple[int, int, float, int, int]:
    """
    Route all differential pairs.

    Args:
        state: The routing state object containing all shared state
        diff_pair_ids_to_route: List of (pair_name, pair) tuples to route

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
    queued_net_ids = state.queued_net_ids
    polarity_swapped_pairs = state.polarity_swapped_pairs
    rip_and_retry_history = state.rip_and_retry_history
    ripup_success_pairs = state.ripup_success_pairs
    remaining_net_ids = state.remaining_net_ids
    results = state.results
    pad_swaps = state.pad_swaps
    all_unrouted_net_ids = state.all_unrouted_net_ids
    gnd_net_id = state.gnd_net_id
    base_obstacles = state.base_obstacles
    diff_pair_base_obstacles = state.diff_pair_base_obstacles
    diff_pair_extra_clearance = state.diff_pair_extra_clearance
    enable_layer_switch = state.enable_layer_switch
    all_swap_vias = state.all_swap_vias
    all_segment_modifications = state.all_segment_modifications
    target_swaps = state.target_swaps
    total_routes = state.total_routes

    # Counters
    successful = 0
    failed = 0
    total_time = 0.0
    total_iterations = 0
    route_index = 0

    for pair_name, pair in diff_pair_ids_to_route:
        route_index += 1
        failed_str = f" ({failed} failed)" if failed > 0 else ""
        print(f"\n[{route_index}/{total_routes}{failed_str}] Routing diff pair {pair_name}")
        print(f"  P: {pair.p_net_name} (id={pair.p_net_id})")
        print(f"  N: {pair.n_net_name} (id={pair.n_net_id})")
        print("-" * 40)

        # Periodic memory reporting (every 5 diff pairs)
        if config.debug_memory and (route_index % 5 == 1 or route_index == len(diff_pair_ids_to_route)):
            current_mem = get_process_memory_mb()
            prox_cache_mb = estimate_track_proximity_cache_mb(track_proximity_cache)
            print(f"[MEMORY] Diff pair {route_index}/{len(diff_pair_ids_to_route)}: {current_mem:.1f} MB total, "
                  f"track_proximity_cache: {prox_cache_mb:.1f} MB ({len(track_proximity_cache)} nets)")

        start_time = time.time()

        # Clone diff pair base obstacles (with extra clearance for centerline routing)
        obstacles = diff_pair_base_obstacles.clone()

        # Add previously routed nets' segments/vias/pads as obstacles (with extra clearance)
        for routed_id in routed_net_ids:
            add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)

        # Add GND vias as obstacles (they were placed with previous diff pair routes)
        if gnd_net_id is not None:
            add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (excluding both P and N)
        other_unrouted = [nid for nid in remaining_net_ids
                         if nid != pair.p_net_id and nid != pair.n_net_id]
        for other_net_id in other_unrouted:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)

        # Add stub proximity costs for ALL unrouted nets in PCB (not just current batch)
        # Includes chip pads as pseudo-stubs to avoid blocking them
        stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                   if nid != pair.p_net_id and nid != pair.n_net_id
                                   and nid not in routed_net_ids]
        unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
        chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
        all_stubs = unrouted_stubs + chip_pads
        if all_stubs:
            add_stub_proximity_costs(obstacles, all_stubs, config)
        # Add track proximity costs for previously routed tracks (same layer only)
        merge_track_proximity_costs(obstacles, track_proximity_cache)

        # Add cross-layer track data for vertical alignment attraction
        add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                                exclude_net_ids={pair.p_net_id, pair.n_net_id})

        # Add same-net via clearance for both P and N
        add_same_net_via_clearance(obstacles, pcb_data, pair.p_net_id, config)
        add_same_net_via_clearance(obstacles, pcb_data, pair.n_net_id, config)

        # Add same-net pad drill hole-to-hole clearance for both P and N
        add_same_net_pad_drill_via_clearance(obstacles, pcb_data, pair.p_net_id, config)
        add_same_net_pad_drill_via_clearance(obstacles, pcb_data, pair.n_net_id, config)

        # Add the diff pair's own stub segments as obstacles to prevent the centerline
        # from routing through them
        add_own_stubs_as_obstacles_for_diff_pair(obstacles, pcb_data, pair.p_net_id, pair.n_net_id, config, diff_pair_extra_clearance)

        # Get source/target coordinates for blocking analysis (center of P/N endpoints)
        sources, targets, _ = get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
        source_xy = None
        target_xy = None
        if sources:
            src = sources[0]
            source_xy = ((src[5] + src[7]) / 2, (src[6] + src[8]) / 2)
        if targets:
            tgt = targets[0]
            target_xy = ((tgt[5] + tgt[7]) / 2, (tgt[6] + tgt[8]) / 2)

        # Route the differential pair
        result = route_diff_pair_with_obstacles(pcb_data, pair, config, obstacles, base_obstacles,
                                                 unrouted_stubs)

        # Handle probe_blocked: try progressive rip-up before full route
        probe_ripup_attempts = 0
        probe_ripped_items = []
        while result and result.get('probe_blocked') and probe_ripup_attempts < config.max_rip_up_count:
            blocked_at = result.get('blocked_at', 'unknown')
            blocked_cells = result.pop('blocked_cells', [])
            probe_direction = result.get('direction', 'unknown')

            if not blocked_cells:
                print(f"  Probe {probe_direction} blocked at {blocked_at} but no blocked cells to analyze")
                break

            # Analyze which routed nets are blocking
            blockers = analyze_frontier_blocking(
                blocked_cells, pcb_data, config, routed_net_paths,
                exclude_net_ids={pair.p_net_id, pair.n_net_id},
                extra_clearance=config.diff_pair_gap / 2,
                target_xy=target_xy,
                source_xy=source_xy
            )

            # Filter to rippable blockers (only those we've routed)
            rippable_blockers = []
            seen_canonical_ids = set()
            for b in blockers:
                canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                if canonical not in seen_canonical_ids and b.net_id in routed_results:
                    seen_canonical_ids.add(canonical)
                    rippable_blockers.append(b)

            if not rippable_blockers:
                print(f"  Probe {probe_direction} blocked at {blocked_at} but no rippable blockers found")
                # Restore any nets that were ripped in previous attempts
                if probe_ripped_items:
                    print(f"    Restoring {len(probe_ripped_items)} previously ripped net(s)")
                    for ripped_blocker, ripped_saved, ripped_ids_item, ripped_was_in_results in reversed(probe_ripped_items):
                        restore_ripped_net(
                            pcb_data, ripped_saved, ripped_ids_item, ripped_was_in_results,
                            routed_net_ids, remaining_net_ids, routed_results, results,
                            config, track_proximity_cache, layer_map
                        )
                    probe_ripped_items.clear()
                # Convert to failed result so normal failure handling takes over
                result = {
                    'failed': True,
                    'iterations': result.get('iterations', 0),
                    'blocked_cells_forward': blocked_cells if probe_direction == 'forward' else [],
                    'blocked_cells_backward': blocked_cells if probe_direction == 'backward' else [],
                }
                break

            probe_ripup_attempts += 1
            print(f"  Probe blocked at {blocked_at}, rip-up attempt {probe_ripup_attempts}/{config.max_rip_up_count}")

            # Rip up the top blocker
            blocker = rippable_blockers[0]
            print(f"    Ripping up {blocker.net_name} ({blocker.blocked_count} blocked cells)")

            saved_result, ripped_ids, was_in_results = rip_up_net(
                blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
                routed_results, diff_pair_by_net_id, remaining_net_ids,
                results, config, track_proximity_cache,
                state.working_obstacles, state.net_obstacles_cache
            )
            probe_ripped_items.append((blocker, saved_result, ripped_ids, was_in_results))

            # Rebuild obstacles without ripped net
            retry_obstacles, _ = build_diff_pair_obstacles(
                diff_pair_base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
                all_unrouted_net_ids, pair.p_net_id, pair.n_net_id, gnd_net_id,
                track_proximity_cache, layer_map, diff_pair_extra_clearance,
                add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair
            )

            # Retry the route
            result = route_diff_pair_with_obstacles(pcb_data, pair, config, retry_obstacles, base_obstacles,
                                                     unrouted_stubs)

            # If retry succeeded, queue ALL ripped nets for re-routing
            if result and not result.get('failed') and not result.get('probe_blocked'):
                for ripped_blocker, ripped_saved, ripped_ids_item, ripped_was_in_results in probe_ripped_items:
                    if ripped_was_in_results:
                        successful -= 1
                    if ripped_blocker.net_id in diff_pair_by_net_id:
                        ripped_pair_name, ripped_pair = diff_pair_by_net_id[ripped_blocker.net_id]
                        canonical_id = ripped_pair.p_net_id
                        if canonical_id not in queued_net_ids:
                            reroute_queue.append(('diff_pair', ripped_pair_name, ripped_pair))
                            queued_net_ids.add(canonical_id)
                    else:
                        if ripped_blocker.net_id not in queued_net_ids:
                            reroute_queue.append(('single', ripped_blocker.net_name, ripped_blocker.net_id))
                            queued_net_ids.add(ripped_blocker.net_id)
                ripup_success_pairs.add(pair_name)
                print(f"    Probe rip-up succeeded, {len(probe_ripped_items)} net(s) queued for re-routing")
            elif result and result.get('probe_blocked'):
                print(f"    Still blocked after rip-up, trying next...")
            else:
                # Route failed completely - restore ALL ripped nets
                print(f"    Probe rip-up didn't help, restoring {len(probe_ripped_items)} net(s)")
                for ripped_blocker, ripped_saved, ripped_ids_item, ripped_was_in_results in reversed(probe_ripped_items):
                    restore_ripped_net(
                        pcb_data, ripped_saved, ripped_ids_item, ripped_was_in_results,
                        routed_net_ids, remaining_net_ids, routed_results, results,
                        config, track_proximity_cache, layer_map
                    )
                probe_ripped_items.clear()
                break

        # If probe rip-up exhausted attempts without success, restore all and try full route
        if result and result.get('probe_blocked'):
            print(f"  Probe blocked after {probe_ripup_attempts} rip-up attempts, restoring {len(probe_ripped_items)} net(s), trying full route...")
            for ripped_blocker, ripped_saved, ripped_ids_item, ripped_was_in_results in reversed(probe_ripped_items):
                restore_ripped_net(
                    pcb_data, ripped_saved, ripped_ids_item, ripped_was_in_results,
                    routed_net_ids, remaining_net_ids, routed_results, results,
                    config, track_proximity_cache, layer_map
                )
            result = {
                'failed': True,
                'iterations': result.get('iterations', 0),
                'blocked_cells_forward': result.get('blocked_cells', []) if result.get('direction') == 'forward' else [],
                'blocked_cells_backward': result.get('blocked_cells', []) if result.get('direction') == 'backward' else [],
            }

        elapsed = time.time() - start_time
        total_time += elapsed

        if result and not result.get('failed'):
            # Calculate actual routed length from segments (includes connectors and via barrels)
            new_segments = result.get('new_segments', [])
            new_vias = result.get('new_vias', [])
            p_segments = [s for s in new_segments if s.net_id == pair.p_net_id]
            n_segments = [s for s in new_segments if s.net_id == pair.n_net_id]
            p_vias = [v for v in new_vias if v.net_id == pair.p_net_id]
            n_vias = [v for v in new_vias if v.net_id == pair.n_net_id]
            p_routed_length = calculate_route_length(p_segments, p_vias, pcb_data)
            n_routed_length = calculate_route_length(n_segments, n_vias, pcb_data)
            centerline_length = (p_routed_length + n_routed_length) / 2

            # Calculate stub lengths for pad-to-pad measurement
            # Use pre-calculated stub lengths from routing, accounting for polarity swap
            p_src_stub = result.get('p_src_stub_length', 0.0)
            p_tgt_stub = result.get('p_tgt_stub_length', 0.0)
            n_src_stub = result.get('n_src_stub_length', 0.0)
            n_tgt_stub = result.get('n_tgt_stub_length', 0.0)

            polarity_fixed = result.get('polarity_fixed', False)
            if polarity_fixed:
                # After swap: P gets P_source + N_target, N gets N_source + P_target
                p_stub_length = p_src_stub + n_tgt_stub
                n_stub_length = n_src_stub + p_tgt_stub
            else:
                p_stub_length = p_src_stub + p_tgt_stub
                n_stub_length = n_src_stub + n_tgt_stub
            avg_stub_length = (p_stub_length + n_stub_length) / 2

            # Store lengths for length matching
            result['centerline_length'] = centerline_length
            result['stub_length'] = avg_stub_length
            result['p_stub_length'] = p_stub_length
            result['n_stub_length'] = n_stub_length
            result['p_routed_length'] = p_routed_length
            result['n_routed_length'] = n_routed_length
            # For inter-pair matching: use max(P,N) total if intra-pair will equalize them, else use average
            if config.diff_pair_intra_match:
                p_total = p_routed_length + p_stub_length
                n_total = n_routed_length + n_stub_length
                result['route_length'] = max(p_total, n_total)
            else:
                result['route_length'] = centerline_length + avg_stub_length

            print(f"  SUCCESS: {len(result['new_segments'])} segments, {len(result['new_vias'])} vias, {result['iterations']} iterations ({elapsed:.2f}s)")
            print(f"    Centerline length: {centerline_length:.3f}mm, stub length: {avg_stub_length:.3f}mm, total: {result['route_length']:.3f}mm")
            results.append(result)
            successful += 1
            total_iterations += result['iterations']

            # Check if polarity was fixed
            apply_polarity_swap(pcb_data, result, pad_swaps, pair_name, polarity_swapped_pairs)

            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)

            if pair.p_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.p_net_id)
            if pair.n_net_id in remaining_net_ids:
                remaining_net_ids.remove(pair.n_net_id)
            routed_net_ids.append(pair.p_net_id)
            routed_net_ids.append(pair.n_net_id)
            track_proximity_cache[pair.p_net_id] = compute_track_proximity_for_net(pcb_data, pair.p_net_id, config, layer_map)
            track_proximity_cache[pair.n_net_id] = compute_track_proximity_for_net(pcb_data, pair.n_net_id, config, layer_map)
            if result.get('p_path'):
                routed_net_paths[pair.p_net_id] = result['p_path']
            if result.get('n_path'):
                routed_net_paths[pair.n_net_id] = result['n_path']
            routed_results[pair.p_net_id] = result
            routed_results[pair.n_net_id] = result
            diff_pair_by_net_id[pair.p_net_id] = (pair_name, pair)
            diff_pair_by_net_id[pair.n_net_id] = (pair_name, pair)
        else:
            iterations = result['iterations'] if result else 0
            print(f"  FAILED: Could not find route ({elapsed:.2f}s)")
            total_iterations += iterations

            # Try rip-up and reroute with progressive N+1
            ripped_up = False
            if routed_net_paths and result:
                # Find the direction that failed faster
                fwd_iters = result.get('iterations_forward', 0)
                bwd_iters = result.get('iterations_backward', 0)
                fwd_cells = result.pop('blocked_cells_forward', [])
                bwd_cells = result.pop('blocked_cells_backward', [])

                # Check for setback failure
                is_setback_failure = (fwd_iters == 0 and bwd_iters == 0 and (fwd_cells or bwd_cells))

                if is_setback_failure:
                    blocked_cells = list(set(fwd_cells + bwd_cells))
                    if blocked_cells:
                        print(f"  Setback blocked ({len(blocked_cells)} blocked cells)")
                elif fwd_iters > 0 and (bwd_iters == 0 or fwd_iters <= bwd_iters):
                    fastest_dir = 'forward'
                    blocked_cells = fwd_cells
                    dir_iters = fwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")
                else:
                    fastest_dir = 'backward'
                    blocked_cells = bwd_cells
                    dir_iters = bwd_iters
                    if blocked_cells:
                        print(f"  {fastest_dir.capitalize()} direction failed fastest ({dir_iters} iterations, {len(blocked_cells)} blocked cells)")

                # Analyze blocking
                if blocked_cells:
                    blockers = analyze_frontier_blocking(
                        blocked_cells, pcb_data, config, routed_net_paths,
                        exclude_net_ids={pair.p_net_id, pair.n_net_id},
                        extra_clearance=diff_pair_extra_clearance,
                        target_xy=target_xy,
                        source_xy=source_xy
                    )
                    print_blocking_analysis(blockers)

                    rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
                        blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id
                    )

                    # Progressive rip-up: try N=1, then N=2, etc
                    current_canonical = pair.p_net_id
                    ripped_items = []
                    ripped_canonical_ids = set()
                    retry_succeeded = False
                    last_retry_blocked_cells = blocked_cells

                    for N in range(1, config.max_rip_up_count + 1):
                        if N > 1 and last_retry_blocked_cells:
                            print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                            fresh_blockers = analyze_frontier_blocking(
                                last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                exclude_net_ids={pair.p_net_id, pair.n_net_id},
                                extra_clearance=diff_pair_extra_clearance,
                                target_xy=target_xy,
                                source_xy=source_xy
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
                            if len(blocker_names) == 1:
                                blockers_str = blocker_names[0]
                            else:
                                blockers_str = "{" + ", ".join(blocker_names) + "}"
                            print(f"  Skipping N={N}: already tried ripping {blockers_str} for {pair_name}")
                            continue

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
                                state.working_obstacles, state.net_obstacles_cache
                            )
                            if saved_result is None:
                                rip_successful = False
                                break
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
                            for net_id, saved_result, ripped_ids, was_in_results in reversed(new_ripped_this_level):
                                restore_net(net_id, saved_result, ripped_ids, was_in_results,
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
                            all_unrouted_net_ids, pair.p_net_id, pair.n_net_id, gnd_net_id,
                            track_proximity_cache, layer_map, diff_pair_extra_clearance,
                            add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair
                        )

                        retry_result = route_diff_pair_with_obstacles(pcb_data, pair, config, retry_obstacles, base_obstacles, unrouted_stubs)

                        if retry_result and not retry_result.get('failed') and not retry_result.get('probe_blocked'):
                            # Calculate actual routed length from segments (includes connectors and via barrels)
                            new_segments = retry_result.get('new_segments', [])
                            new_vias = retry_result.get('new_vias', [])
                            p_segments = [s for s in new_segments if s.net_id == pair.p_net_id]
                            n_segments = [s for s in new_segments if s.net_id == pair.n_net_id]
                            p_vias = [v for v in new_vias if v.net_id == pair.p_net_id]
                            n_vias = [v for v in new_vias if v.net_id == pair.n_net_id]
                            p_routed_length = calculate_route_length(p_segments, p_vias, pcb_data)
                            n_routed_length = calculate_route_length(n_segments, n_vias, pcb_data)
                            centerline_length = (p_routed_length + n_routed_length) / 2

                            # Use pre-calculated stub lengths from routing, accounting for polarity swap
                            p_src_stub = retry_result.get('p_src_stub_length', 0.0)
                            p_tgt_stub = retry_result.get('p_tgt_stub_length', 0.0)
                            n_src_stub = retry_result.get('n_src_stub_length', 0.0)
                            n_tgt_stub = retry_result.get('n_tgt_stub_length', 0.0)

                            polarity_fixed = retry_result.get('polarity_fixed', False)
                            if polarity_fixed:
                                p_stub_length = p_src_stub + n_tgt_stub
                                n_stub_length = n_src_stub + p_tgt_stub
                            else:
                                p_stub_length = p_src_stub + p_tgt_stub
                                n_stub_length = n_src_stub + n_tgt_stub
                            avg_stub_length = (p_stub_length + n_stub_length) / 2

                            retry_result['centerline_length'] = centerline_length
                            retry_result['stub_length'] = avg_stub_length
                            retry_result['p_stub_length'] = p_stub_length
                            retry_result['n_stub_length'] = n_stub_length
                            retry_result['p_routed_length'] = p_routed_length
                            retry_result['n_routed_length'] = n_routed_length
                            # For inter-pair matching: use max(P,N) total if intra-pair will equalize them
                            if config.diff_pair_intra_match:
                                p_total = p_routed_length + p_stub_length
                                n_total = n_routed_length + n_stub_length
                                retry_result['route_length'] = max(p_total, n_total)
                            else:
                                retry_result['route_length'] = centerline_length + avg_stub_length

                            print(f"  RETRY SUCCESS (N={N}): {len(retry_result['new_segments'])} segments, {len(retry_result['new_vias'])} vias")
                            print(f"    Centerline length: {centerline_length:.3f}mm, total: {retry_result['route_length']:.3f}mm")
                            results.append(retry_result)
                            successful += 1
                            total_iterations += retry_result['iterations']

                            apply_polarity_swap(pcb_data, retry_result, pad_swaps, pair_name, polarity_swapped_pairs)

                            add_route_to_pcb_data(pcb_data, retry_result, debug_lines=config.debug_lines)
                            if pair.p_net_id in remaining_net_ids:
                                remaining_net_ids.remove(pair.p_net_id)
                            if pair.n_net_id in remaining_net_ids:
                                remaining_net_ids.remove(pair.n_net_id)
                            routed_net_ids.append(pair.p_net_id)
                            routed_net_ids.append(pair.n_net_id)
                            track_proximity_cache[pair.p_net_id] = compute_track_proximity_for_net(pcb_data, pair.p_net_id, config, layer_map)
                            track_proximity_cache[pair.n_net_id] = compute_track_proximity_for_net(pcb_data, pair.n_net_id, config, layer_map)
                            if retry_result.get('p_path'):
                                routed_net_paths[pair.p_net_id] = retry_result['p_path']
                            if retry_result.get('n_path'):
                                routed_net_paths[pair.n_net_id] = retry_result['n_path']
                            routed_results[pair.p_net_id] = retry_result
                            routed_results[pair.n_net_id] = retry_result
                            diff_pair_by_net_id[pair.p_net_id] = (pair_name, pair)
                            diff_pair_by_net_id[pair.n_net_id] = (pair_name, pair)
                            # Allow re-queuing if this pair gets ripped again later
                            queued_net_ids.discard(pair.p_net_id)
                            queued_net_ids.discard(pair.n_net_id)

                            rip_and_retry_history.add((current_canonical, blocker_canonicals))

                            for net_id, saved_result, ripped_ids, was_in_results in ripped_items:
                                if was_in_results:
                                    successful -= 1
                                if net_id in diff_pair_by_net_id:
                                    ripped_pair_name_tmp, ripped_pair_tmp = diff_pair_by_net_id[net_id]
                                    canonical_id = ripped_pair_tmp.p_net_id
                                    if canonical_id not in queued_net_ids:
                                        reroute_queue.append(('diff_pair', ripped_pair_name_tmp, ripped_pair_tmp))
                                        queued_net_ids.add(canonical_id)
                                else:
                                    if net_id not in queued_net_ids:
                                        net = pcb_data.nets.get(net_id)
                                        net_name = net.name if net else f"Net {net_id}"
                                        reroute_queue.append(('single', net_name, net_id))
                                        queued_net_ids.add(net_id)

                            ripped_up = True
                            retry_succeeded = True
                            ripup_success_pairs.add(pair_name)
                            break
                        else:
                            print(f"  RETRY FAILED (N={N})")

                            if retry_result:
                                if retry_result.get('probe_blocked'):
                                    last_retry_blocked_cells = retry_result.pop('blocked_cells', [])
                                else:
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
                        print(f"  {RED}All rip-up attempts failed: Restoring {len(ripped_items)} net(s){RESET}")
                        for net_id, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
                            restore_net(net_id, saved_result, ripped_ids, was_in_results,
                                       pcb_data, routed_net_ids, routed_net_paths,
                                       routed_results, diff_pair_by_net_id, remaining_net_ids,
                                       results, config, track_proximity_cache, layer_map,
                                       state.working_obstacles, state.net_obstacles_cache)
                            if was_in_results:
                                successful += 1

                # Fallback layer swap for setback failures
                if not ripped_up and is_setback_failure and enable_layer_switch:
                    print(f"  Trying fallback layer swap for setback failure...")
                    swap_success, swap_result, _, _ = try_fallback_layer_swap(
                        pcb_data, pair, pair_name, config,
                        fwd_cells, bwd_cells,
                        diff_pair_base_obstacles, base_obstacles,
                        routed_net_ids, remaining_net_ids,
                        all_unrouted_net_ids, gnd_net_id,
                        track_proximity_cache, diff_pair_extra_clearance,
                        all_swap_vias, all_segment_modifications,
                        None, None,
                        routed_net_paths, routed_results, diff_pair_by_net_id, layer_map,
                        target_swaps, results=results)

                    if swap_success and swap_result:
                        # Calculate actual routed length from segments (includes connectors and via barrels)
                        new_segments = swap_result.get('new_segments', [])
                        new_vias = swap_result.get('new_vias', [])
                        p_segments = [s for s in new_segments if s.net_id == pair.p_net_id]
                        n_segments = [s for s in new_segments if s.net_id == pair.n_net_id]
                        p_vias = [v for v in new_vias if v.net_id == pair.p_net_id]
                        n_vias = [v for v in new_vias if v.net_id == pair.n_net_id]
                        p_routed_length = calculate_route_length(p_segments, p_vias, pcb_data)
                        n_routed_length = calculate_route_length(n_segments, n_vias, pcb_data)
                        centerline_length = (p_routed_length + n_routed_length) / 2

                        # Use pre-calculated stub lengths from routing, accounting for polarity swap
                        p_src_stub = swap_result.get('p_src_stub_length', 0.0)
                        p_tgt_stub = swap_result.get('p_tgt_stub_length', 0.0)
                        n_src_stub = swap_result.get('n_src_stub_length', 0.0)
                        n_tgt_stub = swap_result.get('n_tgt_stub_length', 0.0)

                        polarity_fixed = swap_result.get('polarity_fixed', False)
                        if polarity_fixed:
                            p_stub_length = p_src_stub + n_tgt_stub
                            n_stub_length = n_src_stub + p_tgt_stub
                        else:
                            p_stub_length = p_src_stub + p_tgt_stub
                            n_stub_length = n_src_stub + n_tgt_stub
                        avg_stub_length = (p_stub_length + n_stub_length) / 2

                        swap_result['centerline_length'] = centerline_length
                        swap_result['stub_length'] = avg_stub_length
                        swap_result['p_stub_length'] = p_stub_length
                        swap_result['n_stub_length'] = n_stub_length
                        swap_result['p_routed_length'] = p_routed_length
                        swap_result['n_routed_length'] = n_routed_length
                        # For inter-pair matching: use max(P,N) total if intra-pair will equalize them
                        if config.diff_pair_intra_match:
                            p_total = p_routed_length + p_stub_length
                            n_total = n_routed_length + n_stub_length
                            swap_result['route_length'] = max(p_total, n_total)
                        else:
                            swap_result['route_length'] = centerline_length + avg_stub_length

                        print(f"  {GREEN}FALLBACK LAYER SWAP SUCCESS{RESET}")
                        print(f"    Centerline length: {centerline_length:.3f}mm, total: {swap_result['route_length']:.3f}mm")
                        results.append(swap_result)
                        successful += 1
                        total_iterations += swap_result['iterations']

                        apply_polarity_swap(pcb_data, swap_result, pad_swaps, pair_name, polarity_swapped_pairs)
                        add_route_to_pcb_data(pcb_data, swap_result, debug_lines=config.debug_lines)
                        if pair.p_net_id in remaining_net_ids:
                            remaining_net_ids.remove(pair.p_net_id)
                        if pair.n_net_id in remaining_net_ids:
                            remaining_net_ids.remove(pair.n_net_id)
                        routed_net_ids.append(pair.p_net_id)
                        routed_net_ids.append(pair.n_net_id)
                        track_proximity_cache[pair.p_net_id] = compute_track_proximity_for_net(pcb_data, pair.p_net_id, config, layer_map)
                        track_proximity_cache[pair.n_net_id] = compute_track_proximity_for_net(pcb_data, pair.n_net_id, config, layer_map)
                        if swap_result.get('p_path'):
                            routed_net_paths[pair.p_net_id] = swap_result['p_path']
                        if swap_result.get('n_path'):
                            routed_net_paths[pair.n_net_id] = swap_result['n_path']
                        routed_results[pair.p_net_id] = swap_result
                        routed_results[pair.n_net_id] = swap_result
                        diff_pair_by_net_id[pair.p_net_id] = (pair_name, pair)
                        diff_pair_by_net_id[pair.n_net_id] = (pair_name, pair)
                        # Allow re-queuing if this pair gets ripped again later
                        queued_net_ids.discard(pair.p_net_id)
                        queued_net_ids.discard(pair.n_net_id)
                        ripped_up = True

            if not ripped_up:
                print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                failed += 1

    return successful, failed, total_time, total_iterations, route_index
