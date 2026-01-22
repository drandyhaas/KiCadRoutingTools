"""
Phase 3 multi-point tap routing.

This module handles routing tap connections for multi-point nets.
Phase 3 runs after:
  - Phase 1: Main route between two furthest pads
  - Phase 2: Length matching (if configured)

Phase 3 connects remaining pads to the main route.
"""

import time
from dataclasses import dataclass
from typing import List, Dict, Set, Optional, Tuple, Any

from kicad_parser import PCBData
from routing_config import GridRouteConfig
from routing_state import RoutingState
from routing_context import build_single_ended_obstacles, build_incremental_obstacles
from single_ended_routing import route_multipoint_taps, route_net_with_obstacles, route_multipoint_main
from connectivity import get_multipoint_net_pads
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers, invalidate_obstacle_cache
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import get_canonical_net_id
from pcb_modification import add_route_to_pcb_data
from obstacle_map import add_segments_list_as_obstacles, add_vias_list_as_obstacles
from obstacle_cache import (
    remove_net_obstacles_from_cache, update_net_obstacles_after_routing,
    add_net_obstacles_from_cache, precompute_net_obstacles
)
from obstacle_costs import compute_track_proximity_for_net
from terminal_colors import RED, RESET


@dataclass
class Phase3Stats:
    """Statistics from Phase 3 routing."""
    tap_edges_routed: int = 0
    tap_edges_failed: int = 0
    total_time: float = 0.0


def run_phase3_tap_routing(
    state: RoutingState,
    pcb_data: PCBData,
    config: GridRouteConfig,
    base_obstacles: Any,
    gnd_net_id: Optional[int],
    all_unrouted_net_ids: Set[int],
    routed_net_ids: List[int],
    remaining_net_ids: List[int],
    routed_net_paths: Dict[int, List],
    routed_results: Dict[int, Dict],
    diff_pair_by_net_id: Dict[int, Tuple[str, Any]],
    results: List[Dict],
    track_proximity_cache: Dict[int, Dict],
    layer_map: Dict[str, int],
) -> Phase3Stats:
    """
    Route tap connections for all pending multi-point nets.

    This is the main entry point for Phase 3 routing, called after
    length matching completes.

    Args:
        state: Routing state with pending_multipoint_nets
        pcb_data: PCB data structure
        config: Routing configuration
        base_obstacles: Base obstacle map
        gnd_net_id: GND net ID for via obstacles
        all_unrouted_net_ids: All net IDs that started unrouted
        routed_net_ids: Currently routed net IDs (modified in place)
        remaining_net_ids: Nets still to route (modified in place)
        routed_net_paths: Paths for routed nets (modified in place)
        routed_results: Results for routed nets (modified in place)
        diff_pair_by_net_id: Diff pair mapping (for rip-up)
        results: Results list (modified in place)
        track_proximity_cache: Track proximity cost cache
        layer_map: Layer name to index mapping

    Returns:
        Phase3Stats with routing statistics
    """
    stats = Phase3Stats()

    if not state.pending_multipoint_nets:
        return stats

    start_time = time.time()

    # Count total tap edges across all nets for progress display
    total_tap_edges = sum(
        len(result.get('mst_edges', [])) - 1  # -1 for the main edge routed in Phase 1
        for result in state.pending_multipoint_nets.values()
    )
    global_tap_offset = 0
    global_tap_failed = 0

    print("\n" + "=" * 60)
    print(f"Multi-point Phase 3: Routing {total_tap_edges} tap connections")
    print("=" * 60)

    # Track nets that were ripped during Phase 3 tap routing
    phase3_ripped_nets = []  # List of (net_id, saved_result, ripped_ids, was_in_results)

    # Cache for obstacle cells - persists across retry iterations for performance
    obstacle_cache = {}

    for net_id, main_result in list(state.pending_multipoint_nets.items()):
        # Skip if already processed (removed during a Phase 3 rip-up reroute of another net)
        if net_id not in state.pending_multipoint_nets:
            continue

        net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"net_{net_id}"
        print(f"\n{net_name} (net {net_id}):")
        net_start_time = time.time()

        # Get the length-matched result (with meanders applied)
        lm_result = routed_results.get(net_id, main_result)
        lm_segments = lm_result.get('new_segments', main_result['new_segments'])
        lm_vias = lm_result.get('new_vias', main_result.get('new_vias', []))

        # Check if length matching modified the segments (different object = modified)
        length_matching_active = lm_segments is not main_result['new_segments']

        # Build obstacles - use incremental approach if no length matching and working map available
        if not length_matching_active and state.working_obstacles is not None and state.net_obstacles_cache:
            obstacles, _ = build_incremental_obstacles(
                state.working_obstacles, pcb_data, config, net_id,
                all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                state.net_obstacles_cache
            )
        else:
            # Full rebuild needed when length matching modified segments
            phase3_routed_ids = [rid for rid in routed_net_ids if rid != net_id]
            obstacles, _ = build_single_ended_obstacles(
                base_obstacles, pcb_data, config, phase3_routed_ids, remaining_net_ids,
                all_unrouted_net_ids, net_id, gnd_net_id, track_proximity_cache, layer_map,
                net_obstacles_cache=state.net_obstacles_cache
            )

        # Build input - use LENGTH-MATCHED segments (so taps connect to actual final route)
        # The segment filtering in route_multipoint_taps will avoid meander regions
        tap_input = dict(main_result)
        tap_input['new_segments'] = lm_segments
        tap_input['new_vias'] = lm_vias

        # Route the tap connections
        completed_result = route_multipoint_taps(
            pcb_data, net_id, config, obstacles, tap_input,
            global_offset=global_tap_offset, global_total=total_tap_edges, global_failed=global_tap_failed
        )

        if completed_result:
            # Update global progress counters
            net_edges_attempted = completed_result.get('tap_edges_routed', 0) + completed_result.get('tap_edges_failed', 0) - 1  # -1 for Phase 1 edge
            global_tap_offset += net_edges_attempted
            global_tap_failed += completed_result.get('tap_edges_failed', 0)

            # Check for failed edges and attempt rip-up if no length matching
            failed_edge_blocking = completed_result.get('failed_edge_blocking', {})
            if failed_edge_blocking and not length_matching_active and config.max_rip_up_count > 0:
                # Try rip-up for failed tap routes
                retry_result = _try_phase3_ripup(
                    net_id, completed_result, failed_edge_blocking, lm_segments, lm_vias,
                    pcb_data, config, state, routed_net_ids, remaining_net_ids,
                    all_unrouted_net_ids, routed_net_paths, routed_results,
                    diff_pair_by_net_id, results, track_proximity_cache, layer_map,
                    base_obstacles, gnd_net_id, phase3_ripped_nets,
                    global_tap_offset, total_tap_edges, global_tap_failed,
                    obstacle_cache=obstacle_cache
                )
                if retry_result is not None:
                    completed_result = retry_result

            # Print red final failure message if still have failures after all rip-up attempts
            final_failed_pads = completed_result.get('failed_pads_info', [])
            if final_failed_pads:
                net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
                for pad in final_failed_pads:
                    print(f"  {RED}FAILED: {net_name} - {pad['component_ref']} pad {pad['pad_number']} at ({pad['x']:.2f}, {pad['y']:.2f}) not connected{RESET}")

            # Extract only the NEW tap segments (after the length-matched main route)
            tap_segments = completed_result['new_segments'][len(lm_segments):]
            tap_vias = completed_result['new_vias'][len(lm_vias):]

            if tap_segments or tap_vias:
                tap_result = {'new_segments': tap_segments, 'new_vias': tap_vias}
                add_route_to_pcb_data(pcb_data, tap_result, debug_lines=config.debug_lines)
                print(f"  Added {len(tap_segments)} tap segments, {len(tap_vias)} tap vias")

                # IMPORTANT: Update completed_result['new_segments'] to match what's in pcb_data
                # add_route_to_pcb_data cleans segments via collapse_appendices, updating
                # tap_result['new_segments']. We need completed_result to have the cleaned
                # segments for correct rip-up later. Combine cleaned main (lm_segments)
                # with cleaned tap (tap_result['new_segments']).
                completed_result['new_segments'] = list(lm_segments) + tap_result['new_segments']
                completed_result['new_vias'] = list(lm_vias) + tap_result['new_vias']

                # Update working obstacles with tap segments for subsequent nets
                if state.working_obstacles is not None and state.net_obstacles_cache is not None:
                    # Remove old cache, update with new segments, add back
                    if net_id in state.net_obstacles_cache:
                        remove_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])
                    update_net_obstacles_after_routing(pcb_data, net_id, completed_result, config, state.net_obstacles_cache)
                    add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])

            # IMPORTANT: Replace the Phase 1 result in results list with completed_result
            # The Phase 1 result (main_result) was added to results during Phase 1 routing.
            # We need to remove it and add completed_result so that:
            # 1. The output file contains the combined result (not duplicates)
            # 2. rip_up_net can find routed_results[net_id] in the results list
            if main_result in results:
                results.remove(main_result)
            results.append(completed_result)

            routed_results[net_id] = completed_result

            # Invalidate this net's cache entry since we added tap segments
            # Otherwise subsequent nets would use stale obstacle cells
            invalidate_obstacle_cache(obstacle_cache, net_id)

            stats.tap_edges_routed += completed_result.get('tap_edges_routed', 0) - 1  # -1 for Phase 1
            stats.tap_edges_failed += completed_result.get('tap_edges_failed', 0)

        net_elapsed = time.time() - net_start_time
        net_iterations = completed_result.get('iterations', 0) if completed_result else 0
        print(f"  Net total time: {net_elapsed:.2f}s, {net_iterations} iterations")

    # Re-route nets that were ripped during Phase 3
    if phase3_ripped_nets:
        print(f"\n  Re-routing {len(phase3_ripped_nets)} nets ripped during Phase 3...")
        _reroute_phase3_ripped_nets(
            phase3_ripped_nets, pcb_data, config, state, routed_net_ids, remaining_net_ids,
            all_unrouted_net_ids, routed_net_paths, routed_results, diff_pair_by_net_id,
            results, track_proximity_cache, layer_map, base_obstacles, gnd_net_id
        )

    stats.total_time = time.time() - start_time
    return stats


def _try_phase3_ripup(
    net_id, completed_result, failed_edge_blocking, lm_segments, lm_vias,
    pcb_data, config, state, routed_net_ids, remaining_net_ids,
    all_unrouted_net_ids, routed_net_paths, routed_results,
    diff_pair_by_net_id, results, track_proximity_cache, layer_map,
    base_obstacles, gnd_net_id, phase3_ripped_nets,
    global_tap_offset=0, global_tap_total=0, global_tap_failed=0,
    obstacle_cache=None
):
    """
    Try progressive rip-up and retry for failed Phase 3 tap routes.

    Tries N=1, then N=2, etc up to max_rip_up_count blockers.
    Returns updated completed_result if retry succeeded, None otherwise.
    Appends ripped nets to phase3_ripped_nets for later re-routing.
    """
    # Collect all blocked cells from failed edges
    all_blocked_cells = []
    for edge_key, (blocked_cells, tgt_xy) in failed_edge_blocking.items():
        all_blocked_cells.extend(blocked_cells)

    if not all_blocked_cells:
        return None

    # Analyze blocking
    blockers = analyze_frontier_blocking(
        all_blocked_cells, pcb_data, config, routed_net_paths,
        exclude_net_ids={net_id},
        obstacle_cache=obstacle_cache
    )

    if not blockers:
        return None

    # Filter to rippable blockers
    rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
        blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id
    )

    if not rippable_blockers:
        return None

    # Progressive rip-up: try N=1, then N=2, etc up to max_rip_up_count
    ripped_items = []  # Track nets ripped in this attempt
    ripped_canonical_ids = set()
    last_retry_blocked_cells = all_blocked_cells
    original_failed_count = completed_result.get('tap_edges_failed', 0)

    for N in range(1, config.max_rip_up_count + 1):
        # For N > 1, re-analyze from the last retry's blocked cells
        if N > 1 and last_retry_blocked_cells:
            print(f"    Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
            fresh_blockers = analyze_frontier_blocking(
                last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                exclude_net_ids={net_id},
                obstacle_cache=obstacle_cache
            )
            print_blocking_analysis(fresh_blockers, prefix="      ")

            # Find the most-blocking net that isn't already ripped
            next_blocker = None
            for b in fresh_blockers:
                if b.net_id in routed_results:
                    canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                    if canonical not in ripped_canonical_ids:
                        next_blocker = b
                        break

            if next_blocker is None:
                print(f"    No additional rippable blockers from retry analysis")
                break

            # Add to rippable list if not already there
            next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
            if next_canonical not in seen_canonical_ids:
                seen_canonical_ids.add(next_canonical)
                rippable_blockers.append(next_blocker)

        if N > len(rippable_blockers):
            break  # Not enough blockers to rip

        # Rip up the Nth blocker
        blocker = rippable_blockers[N - 1]
        blocker_name = pcb_data.nets[blocker.net_id].name if blocker.net_id in pcb_data.nets else f"net_{blocker.net_id}"

        if N == 1:
            print(f"    Ripping {blocker_name} (net {blocker.net_id}) to retry tap route...")
        else:
            print(f"    Extending to N={N}: ripping {blocker_name} (net {blocker.net_id})...")

        saved_result, ripped_ids, was_in_results = rip_up_net(
            blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
            routed_results, diff_pair_by_net_id, remaining_net_ids,
            results, config, track_proximity_cache,
            state.working_obstacles, state.net_obstacles_cache
        )

        if saved_result is None:
            print(f"    Failed to rip up {blocker_name}")
            break

        ripped_items.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
        ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
        # Invalidate obstacle cache for ripped nets
        if obstacle_cache is not None:
            for rid in ripped_ids:
                invalidate_obstacle_cache(obstacle_cache, rid)

        # Rebuild obstacles after rip-up
        if state.working_obstacles is not None and state.net_obstacles_cache:
            obstacles, _ = build_incremental_obstacles(
                state.working_obstacles, pcb_data, config, net_id,
                all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                state.net_obstacles_cache
            )
        else:
            phase3_routed_ids = [rid for rid in routed_net_ids if rid != net_id]
            obstacles, _ = build_single_ended_obstacles(
                base_obstacles, pcb_data, config, phase3_routed_ids, remaining_net_ids,
                all_unrouted_net_ids, net_id, gnd_net_id, track_proximity_cache, layer_map,
                net_obstacles_cache=state.net_obstacles_cache
            )

        # Retry tap routing
        tap_input = dict(completed_result)
        tap_input['new_segments'] = list(lm_segments)
        tap_input['new_vias'] = list(lm_vias)
        tap_input.pop('failed_edge_blocking', None)
        # Reset routed_pad_indices to just the main route pads (Phase 1)
        # Otherwise retry thinks other pads are connected but their segments are missing
        mst_edges = completed_result.get('mst_edges', [])
        if mst_edges:
            idx_a, idx_b, _ = mst_edges[0]  # Main route connects these two pads
            tap_input['routed_pad_indices'] = {idx_a, idx_b}
        # Reset tap stats to Phase 1 values (route_multipoint_taps adds to these)
        tap_input['tap_edges_routed'] = 1  # Phase 1 routed 1 edge
        tap_input['tap_edges_failed'] = 0  # Reset failures for retry

        retry_result = route_multipoint_taps(
            pcb_data, net_id, config, obstacles, tap_input,
            global_offset=global_tap_offset, global_total=global_tap_total, global_failed=global_tap_failed
        )

        if retry_result:
            retry_failed = retry_result.get('tap_edges_failed', 0)
            if retry_failed < original_failed_count:
                print(f"    Retry SUCCESS (N={N}): {original_failed_count} -> {retry_failed} failed edges")

                # IMPORTANT: Add retry result's NEW tap segments to obstacles BEFORE re-routing ripped nets
                # Otherwise, ripped nets might route through the same area as our new taps
                # Note: We add directly to working_obstacles since PCB hasn't been updated yet
                if state.working_obstacles is not None:
                    tap_segments = retry_result['new_segments'][len(lm_segments):]
                    tap_vias = retry_result['new_vias'][len(lm_vias):]
                    if tap_segments or tap_vias:
                        print(f"    Adding {len(tap_segments)} tap segments, {len(tap_vias)} tap vias to obstacles before re-routing...")
                        add_segments_list_as_obstacles(state.working_obstacles, tap_segments, config)
                        add_vias_list_as_obstacles(state.working_obstacles, tap_vias, config)

                # Re-route ripped nets IMMEDIATELY (not deferred) to prevent subsequent
                # Phase 3 nets from routing through the ripped area
                if ripped_items:
                    print(f"    Re-routing {len(ripped_items)} ripped net(s)...")
                    _reroute_phase3_ripped_nets(
                        ripped_items, pcb_data, config, state, routed_net_ids, remaining_net_ids,
                        all_unrouted_net_ids, routed_net_paths, routed_results, diff_pair_by_net_id,
                        results, track_proximity_cache, layer_map, base_obstacles, gnd_net_id
                    )
                return retry_result
            else:
                print(f"    Retry FAILED (N={N}): still {retry_failed} failed edges")
                # Store blocked cells from retry for next iteration's analysis
                retry_blocking = retry_result.get('failed_edge_blocking', {})
                if retry_blocking:
                    last_retry_blocked_cells = []
                    for edge_key, (blocked_cells, tgt_xy) in retry_blocking.items():
                        last_retry_blocked_cells.extend(blocked_cells)
                    if last_retry_blocked_cells:
                        print(f"      Retry had {len(last_retry_blocked_cells)} blocked cells")
        else:
            print(f"    Retry FAILED (N={N}): no result")
            break

    # All attempts failed - restore ripped nets
    if ripped_items:
        print(f"    {RED}All rip-up attempts failed, restoring {len(ripped_items)} net(s)...{RESET}")
        for rid, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
            restore_net(
                rid, saved_result, ripped_ids, was_in_results,
                pcb_data, routed_net_ids, routed_net_paths,
                routed_results, diff_pair_by_net_id, remaining_net_ids,
                results, config, track_proximity_cache, layer_map,
                state.working_obstacles, state.net_obstacles_cache
            )

    return None


def _reroute_phase3_ripped_nets(
    phase3_ripped_nets, pcb_data, config, state, routed_net_ids, remaining_net_ids,
    all_unrouted_net_ids, routed_net_paths, routed_results, diff_pair_by_net_id,
    results, track_proximity_cache, layer_map, base_obstacles, gnd_net_id
):
    """
    Re-route nets that were ripped during Phase 3 tap routing.

    This includes routing their main route and any tap connections.
    """
    for ripped_net_id, saved_result, ripped_ids, was_in_results in phase3_ripped_nets:
        net_name = pcb_data.nets[ripped_net_id].name if ripped_net_id in pcb_data.nets else f"net_{ripped_net_id}"
        print(f"\n  Re-routing {net_name} (net {ripped_net_id})...")

        # Skip if already re-routed (by another rip-up)
        if ripped_net_id in routed_results:
            print(f"    Already routed, skipping")
            continue

        # Build obstacles
        if state.working_obstacles is not None and state.net_obstacles_cache:
            obstacles, _ = build_incremental_obstacles(
                state.working_obstacles, pcb_data, config, ripped_net_id,
                all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                state.net_obstacles_cache
            )
        else:
            phase3_routed_ids = [rid for rid in routed_net_ids if rid != ripped_net_id]
            obstacles, _ = build_single_ended_obstacles(
                base_obstacles, pcb_data, config, phase3_routed_ids, remaining_net_ids,
                all_unrouted_net_ids, ripped_net_id, gnd_net_id, track_proximity_cache, layer_map,
                net_obstacles_cache=state.net_obstacles_cache
            )

        # Check if this was originally a multi-point net (from saved_result)
        was_multipoint = saved_result and saved_result.get('mst_edges') and len(saved_result.get('mst_edges', [])) > 1

        if was_multipoint:
            # Use multipoint routing for multi-point nets
            multipoint_pads = get_multipoint_net_pads(pcb_data, ripped_net_id, config)
            if multipoint_pads:
                result = route_multipoint_main(pcb_data, ripped_net_id, config, obstacles, multipoint_pads)
            else:
                result = route_net_with_obstacles(pcb_data, ripped_net_id, config, obstacles)
        else:
            # Route the main path for simple nets
            result = route_net_with_obstacles(pcb_data, ripped_net_id, config, obstacles)

        if result and not result.get('failed') and result.get('path'):
            main_vias = result.get('new_vias', [])
            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
            results.append(result)
            routed_net_ids.append(ripped_net_id)
            if ripped_net_id in remaining_net_ids:
                remaining_net_ids.remove(ripped_net_id)
            routed_net_paths[ripped_net_id] = result['path']
            routed_results[ripped_net_id] = result
            track_proximity_cache[ripped_net_id] = compute_track_proximity_for_net(
                pcb_data, ripped_net_id, config, layer_map
            )
            print(f"    Re-routed main path: {len(result['new_segments'])} segments, {len(main_vias)} vias")

            # CRITICAL: Update pending_multipoint_nets to point to the NEW result
            # This ensures that if tap routing fails, the later Phase 3 loop will have
            # the correct reference to remove from results[] before adding completed_result.
            # Without this, the old result reference in pending_multipoint_nets won't be
            # found in results[], leading to duplicate segments being written to output.
            if ripped_net_id in state.pending_multipoint_nets:
                state.pending_multipoint_nets[ripped_net_id] = result

            # Update working obstacles
            if state.working_obstacles is not None and state.net_obstacles_cache is not None:
                if ripped_net_id in state.net_obstacles_cache:
                    remove_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[ripped_net_id])
                update_net_obstacles_after_routing(pcb_data, ripped_net_id, result, config, state.net_obstacles_cache)
                add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[ripped_net_id])

            # Check if this was a multi-point net that needs tap routing (check saved_result)
            if was_multipoint and result.get('mst_edges') and len(result.get('mst_edges', [])) > 1:
                # Rebuild obstacles for tap routing
                if state.working_obstacles is not None and state.net_obstacles_cache:
                    tap_obstacles, _ = build_incremental_obstacles(
                        state.working_obstacles, pcb_data, config, ripped_net_id,
                        all_unrouted_net_ids, routed_net_ids, track_proximity_cache, layer_map,
                        state.net_obstacles_cache
                    )
                else:
                    tap_obstacles = obstacles

                tap_result = route_multipoint_taps(
                    pcb_data, ripped_net_id, config, tap_obstacles, result,
                    global_offset=0, global_total=0, global_failed=0
                )

                if tap_result:
                    tap_segments = tap_result['new_segments'][len(result['new_segments']):]
                    tap_vias = tap_result['new_vias'][len(result.get('new_vias', [])):]

                    if tap_segments or tap_vias:
                        tap_result_data = {'new_segments': tap_segments, 'new_vias': tap_vias}
                        add_route_to_pcb_data(pcb_data, tap_result_data, debug_lines=config.debug_lines)
                        print(f"    Re-routed {len(tap_segments)} tap segments, {len(tap_vias)} tap vias")

                        # IMPORTANT: Update tap_result['new_segments'] to match what's in pcb_data
                        # add_route_to_pcb_data cleans segments via collapse_appendices, updating
                        # tap_result_data['new_segments']. We need tap_result to have the cleaned
                        # segments for correct rip-up later. Combine cleaned main (from result)
                        # with cleaned tap (from tap_result_data).
                        tap_result['new_segments'] = result['new_segments'] + tap_result_data['new_segments']
                        tap_result['new_vias'] = result.get('new_vias', []) + tap_result_data['new_vias']

                        # Update working obstacles with tap segments
                        if state.working_obstacles is not None and state.net_obstacles_cache is not None:
                            if ripped_net_id in state.net_obstacles_cache:
                                remove_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[ripped_net_id])
                            update_net_obstacles_after_routing(pcb_data, ripped_net_id, tap_result, config, state.net_obstacles_cache)
                            add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[ripped_net_id])

                    # IMPORTANT: Replace the main route result with tap_result in results
                    # The main route result was added above. We need routed_results[net_id] to
                    # point to something that's actually in results for rip_up_net to work correctly.
                    if result in results:
                        results.remove(result)
                    results.append(tap_result)
                    routed_results[ripped_net_id] = tap_result

                    # Print red final failure message if re-routed net still has unconnected pads
                    final_failed_pads = tap_result.get('failed_pads_info', [])
                    if final_failed_pads:
                        net_name = pcb_data.nets[ripped_net_id].name if ripped_net_id in pcb_data.nets else f"Net {ripped_net_id}"
                        for pad in final_failed_pads:
                            print(f"    {RED}FAILED: {net_name} - {pad['component_ref']} pad {pad['pad_number']} at ({pad['x']:.2f}, {pad['y']:.2f}) not connected{RESET}")

                    # Remove from pending_multipoint_nets since Phase 3 is now complete
                    if ripped_net_id in state.pending_multipoint_nets:
                        del state.pending_multipoint_nets[ripped_net_id]
        else:
            print(f"    {RED}Failed to re-route{RESET}")
            # CRITICAL: Remove from pending_multipoint_nets when re-route fails
            # If we don't remove it, Phase 3 will later process this net and start with
            # the OLD stale segments from the pending result (which were ripped and are
            # no longer in pcb_data), leading to duplicate/stale segments in the output.
            if ripped_net_id in state.pending_multipoint_nets:
                del state.pending_multipoint_nets[ripped_net_id]
