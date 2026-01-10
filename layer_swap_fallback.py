"""
Layer swap fallback for differential pair routing.

When routing fails due to blocked cells, this module attempts to swap the blocked
side's stubs to another layer as a fallback strategy. It includes rip-up and
reroute logic if the initial route after swap fails.
"""

from typing import List, Optional, Tuple, Dict

from kicad_parser import PCBData
from routing_config import GridRouteConfig, DiffPairNet
from connectivity import get_stub_endpoints, find_stub_free_ends
from net_queries import get_chip_pad_positions
from route_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from obstacle_map import (
    add_net_stubs_as_obstacles, add_net_vias_as_obstacles, add_net_pads_as_obstacles,
    add_stub_proximity_costs, merge_track_proximity_costs, add_same_net_via_clearance,
    add_same_net_pad_drill_via_clearance, add_diff_pair_own_stubs_as_obstacles,
    compute_track_proximity_for_net, add_vias_list_as_obstacles, add_segments_list_as_obstacles
)
from blocking_analysis import analyze_frontier_blocking
from polarity_swap import get_canonical_net_id


def add_own_stubs_as_obstacles_for_diff_pair(obstacles, pcb_data, p_net_id: int, n_net_id: int,
                                              config, extra_clearance: float):
    """Add a diff pair's own stub segments as obstacles to prevent centerline from crossing them.

    This is a helper function to avoid duplicating code in multiple places.
    """
    p_segments = [s for s in pcb_data.segments if s.net_id == p_net_id]
    n_segments = [s for s in pcb_data.segments if s.net_id == n_net_id]
    if not p_segments and not n_segments:
        return

    p_pads = pcb_data.pads_by_net.get(p_net_id, [])
    n_pads = pcb_data.pads_by_net.get(n_net_id, [])
    p_stub_ends = find_stub_free_ends(p_segments, p_pads)
    n_stub_ends = find_stub_free_ends(n_segments, n_pads)
    stub_endpoints = [(x, y) for x, y, _ in p_stub_ends + n_stub_ends]

    add_diff_pair_own_stubs_as_obstacles(
        obstacles, pcb_data, p_net_id, n_net_id, config,
        exclude_endpoints=stub_endpoints, extra_clearance=extra_clearance
    )


def try_fallback_layer_swap(pcb_data, pair, pair_name: str, config,
                            fwd_cells: list, bwd_cells: list,
                            diff_pair_base_obstacles, base_obstacles,
                            routed_net_ids: list, remaining_net_ids: list,
                            all_unrouted_net_ids, gnd_net_id,
                            track_proximity_cache: dict,
                            diff_pair_extra_clearance: float,
                            all_swap_vias: list, all_segment_modifications: list,
                            all_stubs_by_layer: dict = None,
                            stub_endpoints_by_layer: dict = None,
                            routed_net_paths: dict = None,
                            routed_results: dict = None,
                            diff_pair_by_net_id: dict = None,
                            layer_map: dict = None,
                            target_swaps: dict = None,
                            results: list = None):
    """
    Try to swap the blocked side's stubs to another layer as a fallback when routing fails.
    After applying the swap, attempts rip-up and reroute if the initial route fails.

    Returns:
        (success, result, vias, mods) - success bool, routing result if successful,
        and lists of vias/modifications applied (for tracking).
    """
    from stub_layer_switching import (get_stub_info, apply_stub_layer_switch,
        validate_swap, collect_stubs_by_layer, collect_stub_endpoints_by_layer)
    from diff_pair_routing import get_diff_pair_endpoints, route_diff_pair_with_obstacles

    # Get current endpoint info
    sources, targets, error = get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
    if not sources or not targets or error:
        return False, None, [], []

    src_layer = config.layers[sources[0][4]]
    tgt_layer = config.layers[targets[0][4]]

    # Determine which side(s) to try based on blocked cells
    sides_to_try = []
    if fwd_cells:  # Source side blocked (forward direction)
        sides_to_try.append(('source', sources, src_layer))
    if bwd_cells:  # Target side blocked (backward direction)
        sides_to_try.append(('target', targets, tgt_layer))

    if not sides_to_try:
        return False, None, [], []

    # Use provided stubs_by_layer or build minimal version as fallback
    if all_stubs_by_layer is None:
        pair_info = {pair_name: (src_layer, tgt_layer, sources, targets, pair)}
        stubs_by_layer = collect_stubs_by_layer(pcb_data, pair_info, config)
    else:
        stubs_by_layer = all_stubs_by_layer

    if stub_endpoints_by_layer is None:
        pair_info = {pair_name: (src_layer, tgt_layer, sources, targets, pair)}
        endpoints_by_layer = collect_stub_endpoints_by_layer(pcb_data, pair_info, config)
    else:
        endpoints_by_layer = stub_endpoints_by_layer

    # Track all accumulated modifications from layer swaps (even if routing fails)
    accumulated_mods = []
    accumulated_vias = []

    for side, endpoints, initial_layer in sides_to_try:
        # Track current layer (changes after each failed swap since we don't revert)
        current_layer = initial_layer

        # Determine correct net_ids to use for looking up stubs in pcb_data
        # After target swap, TARGET stubs have the swap partner's net_ids in pcb_data
        lookup_p_net_id = pair.p_net_id
        lookup_n_net_id = pair.n_net_id
        if side == 'target' and target_swaps and pair_name in target_swaps:
            swap_partner_name = target_swaps[pair_name]
            # Find swap partner's net IDs
            if diff_pair_by_net_id:
                for net_id, (pname, ppair) in diff_pair_by_net_id.items():
                    if pname == swap_partner_name:
                        lookup_p_net_id = ppair.p_net_id
                        lookup_n_net_id = ppair.n_net_id
                        break

        # Try each candidate layer
        for candidate_layer in config.layers:
            if candidate_layer == current_layer:
                continue

            # Get fresh stub info on current layer (may have changed from previous swap)
            # Use lookup_*_net_id which may be swap partner's for target stubs with target swap
            p_stub = get_stub_info(pcb_data, lookup_p_net_id,
                                   endpoints[0][5], endpoints[0][6], current_layer)
            n_stub = get_stub_info(pcb_data, lookup_n_net_id,
                                   endpoints[0][7], endpoints[0][8], current_layer)

            if not p_stub or not n_stub:
                print(f"    Could not get stub info for {side} side on {current_layer}")
                break  # Can't continue with this side

            # Validate swap
            valid, reason = validate_swap(
                p_stub, n_stub, candidate_layer, stubs_by_layer,
                pcb_data, config, swap_partner_name=None,
                swap_partner_net_ids=set(),
                stub_endpoints_by_layer=endpoints_by_layer
            )

            if not valid:
                print(f"    {side} swap to {candidate_layer}: {reason}")
                continue

            # Apply the swap
            print(f"    Applying fallback {side} swap: {current_layer} -> {candidate_layer}")
            vias1, mods1 = apply_stub_layer_switch(pcb_data, p_stub, candidate_layer, config, debug=False)
            vias2, mods2 = apply_stub_layer_switch(pcb_data, n_stub, candidate_layer, config, debug=False)
            all_mods = mods1 + mods2
            all_vias = vias1 + vias2

            # Note: No net_id translation needed - we looked up stubs with the correct
            # net_ids (swap partner's for target side), so modifications are recorded correctly

            # Accumulate modifications (even if routing fails, segments are changed)
            accumulated_mods.extend(all_mods)
            accumulated_vias.extend(all_vias)

            # Rebuild obstacles and retry
            retry_obstacles = diff_pair_base_obstacles.clone()
            for routed_id in routed_net_ids:
                add_net_stubs_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                add_net_vias_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                add_net_pads_as_obstacles(retry_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
            if gnd_net_id is not None:
                add_net_vias_as_obstacles(retry_obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)
            other_unrouted = [nid for nid in remaining_net_ids
                             if nid != pair.p_net_id and nid != pair.n_net_id]
            for other_net_id in other_unrouted:
                add_net_stubs_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                add_net_vias_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                add_net_pads_as_obstacles(retry_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
            stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                       if nid != pair.p_net_id and nid != pair.n_net_id
                                       and nid not in routed_net_ids]
            unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
            chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
            all_stubs = unrouted_stubs + chip_pads
            if all_stubs:
                add_stub_proximity_costs(retry_obstacles, all_stubs, config)
            merge_track_proximity_costs(retry_obstacles, track_proximity_cache)
            add_same_net_via_clearance(retry_obstacles, pcb_data, pair.p_net_id, config)
            add_same_net_via_clearance(retry_obstacles, pcb_data, pair.n_net_id, config)
            add_same_net_pad_drill_via_clearance(retry_obstacles, pcb_data, pair.p_net_id, config)
            add_same_net_pad_drill_via_clearance(retry_obstacles, pcb_data, pair.n_net_id, config)
            add_own_stubs_as_obstacles_for_diff_pair(retry_obstacles, pcb_data, pair.p_net_id, pair.n_net_id, config, diff_pair_extra_clearance)

            retry_result = route_diff_pair_with_obstacles(pcb_data, pair, config, retry_obstacles, base_obstacles, unrouted_stubs)

            if retry_result and not retry_result.get('failed') and not retry_result.get('probe_blocked'):
                # SUCCESS - track vias and mods for file writing
                all_swap_vias.extend(accumulated_vias)
                all_segment_modifications.extend(accumulated_mods)

                # Update stubs_by_layer to reflect the layer change (for subsequent fallbacks)
                if stubs_by_layer is not None:
                    combined_segments = p_stub.segments + n_stub.segments
                    # Remove from old layer
                    if current_layer in stubs_by_layer:
                        stubs_by_layer[current_layer] = [
                            s for s in stubs_by_layer[current_layer] if s[0] != pair_name
                        ]
                    # Add to new layer
                    if candidate_layer not in stubs_by_layer:
                        stubs_by_layer[candidate_layer] = []
                    stubs_by_layer[candidate_layer].append((pair_name, combined_segments))

                return True, retry_result, all_vias, all_mods

            # Initial route after layer swap failed - try rip-up and reroute
            print(f"    Initial route failed after {side} swap to {candidate_layer}, trying rip-up...")

            # Check if we have the context needed for rip-up
            if routed_net_paths is not None and routed_results is not None and diff_pair_by_net_id is not None:
                # Get blocked cells from the failed result (diff pair has forward/backward)
                if retry_result:
                    fwd_cells_rip = retry_result.pop('blocked_cells_forward', [])
                    bwd_cells_rip = retry_result.pop('blocked_cells_backward', [])
                    # Also check plain blocked_cells (for probe_blocked cases)
                    plain_cells = retry_result.pop('blocked_cells', [])
                    blocked_cells = list(set(fwd_cells_rip + bwd_cells_rip + plain_cells))
                    del fwd_cells_rip, bwd_cells_rip, plain_cells  # Free memory immediately
                else:
                    blocked_cells = []

                if blocked_cells:
                    # Analyze what's blocking
                    blockers = analyze_frontier_blocking(
                        blocked_cells, pcb_data, config, routed_net_paths,
                        exclude_net_ids={pair.p_net_id, pair.n_net_id},
                        extra_clearance=diff_pair_extra_clearance
                    )

                    # Filter to rippable blockers
                    rippable_blockers = []
                    seen_canonical_ids = set()
                    for b in blockers:
                        if b.net_id in routed_results:
                            canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                            if canonical not in seen_canonical_ids:
                                seen_canonical_ids.add(canonical)
                                rippable_blockers.append(b)

                    # Try ripping up to max_rip_up_count blockers
                    ripped_items = []
                    for N in range(1, min(config.max_rip_up_count + 1, len(rippable_blockers) + 1)):
                        blocker = rippable_blockers[N - 1]
                        blocker_canonical = get_canonical_net_id(blocker.net_id, diff_pair_by_net_id)

                        # Determine net IDs to rip (both P and N for diff pairs)
                        if blocker.net_id in diff_pair_by_net_id:
                            rip_pair_name, rip_pair = diff_pair_by_net_id[blocker.net_id]
                            rip_net_ids = [rip_pair.p_net_id, rip_pair.n_net_id]
                            print(f"      Ripping {rip_pair_name} (N={N})")
                        else:
                            rip_net_ids = [blocker.net_id]
                            print(f"      Ripping {blocker.net_name} (N={N})")

                        # Save and remove the blocking route
                        saved_result = routed_results.get(rip_net_ids[0])
                        if saved_result:
                            # Track if was in results list (for restore and reroute handling)
                            was_in_results = results is not None and saved_result in results
                            if was_in_results:
                                results.remove(saved_result)
                            remove_route_from_pcb_data(pcb_data, saved_result)
                            for rid in rip_net_ids:
                                if rid in routed_net_ids:
                                    routed_net_ids.remove(rid)
                                if rid not in remaining_net_ids:
                                    remaining_net_ids.append(rid)
                            ripped_items.append((blocker, saved_result, rip_net_ids, was_in_results))

                            # Rebuild obstacles and retry
                            rip_obstacles = diff_pair_base_obstacles.clone()
                            for routed_id in routed_net_ids:
                                add_net_stubs_as_obstacles(rip_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                                add_net_vias_as_obstacles(rip_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                                add_net_pads_as_obstacles(rip_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                            if gnd_net_id is not None:
                                add_net_vias_as_obstacles(rip_obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)
                            other_unrouted = [nid for nid in remaining_net_ids
                                             if nid != pair.p_net_id and nid != pair.n_net_id]
                            for other_net_id in other_unrouted:
                                add_net_stubs_as_obstacles(rip_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                                add_net_vias_as_obstacles(rip_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                                add_net_pads_as_obstacles(rip_obstacles, pcb_data, other_net_id, config, diff_pair_extra_clearance)
                            stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                                                       if nid != pair.p_net_id and nid != pair.n_net_id
                                                       and nid not in routed_net_ids]
                            rip_unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
                            rip_chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
                            rip_all_stubs = rip_unrouted_stubs + rip_chip_pads
                            if rip_all_stubs:
                                add_stub_proximity_costs(rip_obstacles, rip_all_stubs, config)
                            merge_track_proximity_costs(rip_obstacles, track_proximity_cache)
                            add_same_net_via_clearance(rip_obstacles, pcb_data, pair.p_net_id, config)
                            add_same_net_via_clearance(rip_obstacles, pcb_data, pair.n_net_id, config)
                            add_same_net_pad_drill_via_clearance(rip_obstacles, pcb_data, pair.p_net_id, config)
                            add_same_net_pad_drill_via_clearance(rip_obstacles, pcb_data, pair.n_net_id, config)
                            add_own_stubs_as_obstacles_for_diff_pair(rip_obstacles, pcb_data, pair.p_net_id, pair.n_net_id, config, diff_pair_extra_clearance)

                            rip_result = route_diff_pair_with_obstacles(pcb_data, pair, config, rip_obstacles, base_obstacles, rip_unrouted_stubs)

                            if rip_result and not rip_result.get('failed') and not rip_result.get('probe_blocked'):
                                print(f"      Rip-up succeeded after {side} swap to {candidate_layer}")
                                # Re-route the ripped nets
                                all_ripped_rerouted = True
                                for ripped_blocker, ripped_saved, ripped_ids, ripped_was_in_results in ripped_items:
                                    if ripped_blocker.net_id in diff_pair_by_net_id:
                                        ripped_pair_name, ripped_pair = diff_pair_by_net_id[ripped_blocker.net_id]
                                        # Rebuild obstacles for rerouting the ripped pair
                                        reroute_obstacles = diff_pair_base_obstacles.clone()
                                        for routed_id in routed_net_ids:
                                            add_net_stubs_as_obstacles(reroute_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                                            add_net_vias_as_obstacles(reroute_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                                            add_net_pads_as_obstacles(reroute_obstacles, pcb_data, routed_id, config, diff_pair_extra_clearance)
                                        # Add the just-routed pair as obstacle
                                        add_net_stubs_as_obstacles(reroute_obstacles, pcb_data, pair.p_net_id, config, diff_pair_extra_clearance)
                                        add_net_vias_as_obstacles(reroute_obstacles, pcb_data, pair.p_net_id, config, diff_pair_extra_clearance)
                                        add_net_stubs_as_obstacles(reroute_obstacles, pcb_data, pair.n_net_id, config, diff_pair_extra_clearance)
                                        add_net_vias_as_obstacles(reroute_obstacles, pcb_data, pair.n_net_id, config, diff_pair_extra_clearance)
                                        if gnd_net_id is not None:
                                            add_net_vias_as_obstacles(reroute_obstacles, pcb_data, gnd_net_id, config, diff_pair_extra_clearance)
                                            # Also add GND vias from rip_result (the just-routed pair) which aren't in pcb_data yet
                                            gnd_vias_from_result = [v for v in rip_result.get('new_vias', []) if v.net_id == gnd_net_id]
                                            if gnd_vias_from_result:
                                                add_vias_list_as_obstacles(reroute_obstacles, gnd_vias_from_result, config, diff_pair_extra_clearance)
                                        # Also add segments from rip_result (the just-routed pair) which aren't in pcb_data yet
                                        if rip_result.get('new_segments'):
                                            add_segments_list_as_obstacles(reroute_obstacles, rip_result['new_segments'], config, diff_pair_extra_clearance)
                                        reroute_stub_net_ids = [nid for nid in all_unrouted_net_ids
                                                                if nid not in routed_net_ids
                                                                and nid != ripped_pair.p_net_id
                                                                and nid != ripped_pair.n_net_id]
                                        reroute_stubs = get_stub_endpoints(pcb_data, reroute_stub_net_ids)
                                        reroute_chip_pads = get_chip_pad_positions(pcb_data, reroute_stub_net_ids)
                                        reroute_all_stubs = reroute_stubs + reroute_chip_pads
                                        if reroute_all_stubs:
                                            add_stub_proximity_costs(reroute_obstacles, reroute_all_stubs, config)
                                        merge_track_proximity_costs(reroute_obstacles, track_proximity_cache)

                                        reroute_result = route_diff_pair_with_obstacles(pcb_data, ripped_pair, config, reroute_obstacles, base_obstacles, reroute_stubs)
                                        if reroute_result and not reroute_result.get('failed') and not reroute_result.get('probe_blocked'):
                                            add_route_to_pcb_data(pcb_data, reroute_result, debug_lines=config.debug_lines)
                                            # Add to results list if the ripped result was in it
                                            if ripped_was_in_results and results is not None:
                                                results.append(reroute_result)
                                            for rid in ripped_ids:
                                                if rid not in routed_net_ids:
                                                    routed_net_ids.append(rid)
                                                if rid in remaining_net_ids:
                                                    remaining_net_ids.remove(rid)
                                                routed_results[rid] = reroute_result
                                            if track_proximity_cache is not None and layer_map is not None:
                                                for rid in ripped_ids:
                                                    track_proximity_cache[rid] = compute_track_proximity_for_net(pcb_data, rid, config, layer_map)
                                            print(f"      Re-routed {ripped_pair_name}")
                                        else:
                                            print(f"      Failed to re-route {ripped_pair_name}")
                                            all_ripped_rerouted = False
                                            break

                                if all_ripped_rerouted:
                                    # SUCCESS
                                    all_swap_vias.extend(accumulated_vias)
                                    all_segment_modifications.extend(accumulated_mods)
                                    if stubs_by_layer is not None:
                                        combined_segments = p_stub.segments + n_stub.segments
                                        if current_layer in stubs_by_layer:
                                            stubs_by_layer[current_layer] = [
                                                s for s in stubs_by_layer[current_layer] if s[0] != pair_name
                                            ]
                                        if candidate_layer not in stubs_by_layer:
                                            stubs_by_layer[candidate_layer] = []
                                        stubs_by_layer[candidate_layer].append((pair_name, combined_segments))
                                    return True, rip_result, all_vias, all_mods
                                else:
                                    # Failed to reroute ripped nets - restore them
                                    # But first, we need to undo any partial successes
                                    # (nets that were rerouted before a subsequent one failed)
                                    for ripped_blocker, ripped_saved, ripped_ids, ripped_was_in_results in ripped_items:
                                        if ripped_saved:
                                            # Check if this net was already successfully rerouted
                                            # (it would have its new result in routed_results, not the old one)
                                            already_rerouted = (ripped_ids[0] in routed_results and
                                                              routed_results[ripped_ids[0]] != ripped_saved)
                                            if already_rerouted:
                                                # Remove the new route that was added during partial success
                                                new_result = routed_results[ripped_ids[0]]
                                                remove_route_from_pcb_data(pcb_data, new_result)
                                                if new_result in results:
                                                    results.remove(new_result)
                                            # Now restore the old route
                                            add_route_to_pcb_data(pcb_data, ripped_saved, debug_lines=config.debug_lines)
                                            if ripped_was_in_results and results is not None:
                                                # Only append if not already in results (prevent duplicates)
                                                if ripped_saved not in results:
                                                    results.append(ripped_saved)
                                            for rid in ripped_ids:
                                                if rid not in routed_net_ids:
                                                    routed_net_ids.append(rid)
                                                if rid in remaining_net_ids:
                                                    remaining_net_ids.remove(rid)
                                                routed_results[rid] = ripped_saved
                                    break
                            else:
                                # Update blocked cells for next iteration's analysis
                                if rip_result:
                                    blocked_cells = rip_result.get('blocked_cells', [])

                    # If rip-up didn't succeed, restore all ripped nets
                    for ripped_blocker, ripped_saved, ripped_ids, ripped_was_in_results in ripped_items:
                        if ripped_saved and ripped_ids[0] not in routed_net_ids:
                            add_route_to_pcb_data(pcb_data, ripped_saved, debug_lines=config.debug_lines)
                            if ripped_was_in_results and results is not None and ripped_saved not in results:
                                results.append(ripped_saved)
                            for rid in ripped_ids:
                                if rid not in routed_net_ids:
                                    routed_net_ids.append(rid)
                                if rid in remaining_net_ids:
                                    remaining_net_ids.remove(rid)
                                routed_results[rid] = ripped_saved
                            if track_proximity_cache is not None and layer_map is not None:
                                for rid in ripped_ids:
                                    track_proximity_cache[rid] = compute_track_proximity_for_net(pcb_data, rid, config, layer_map)

            print(f"    Rip-up failed after {side} swap to {candidate_layer}")
            # Don't revert layer swap - update current_layer so next iteration knows where stubs are
            current_layer = candidate_layer

    # Even on failure, add accumulated modifications to trackers so file output reflects actual state
    if accumulated_mods:
        print(f"    Adding {len(accumulated_mods)} accumulated segment modifications to output")
        all_segment_modifications.extend(accumulated_mods)
    if accumulated_vias:
        all_swap_vias.extend(accumulated_vias)

    return False, None, [], []
