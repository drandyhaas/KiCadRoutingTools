"""
Single-ended routing loop.

This module contains the main loop for routing single-ended nets,
extracted from route.py for better maintainability.
"""
from __future__ import annotations

import os
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

from routing_state import RoutingState, record_net_event, record_rip_ancestry, rip_exclude_set
from bus_detection import detect_bus_groups, get_bus_routing_order, get_attraction_neighbor, bus_attraction_context, bus_stick_config, BusGroup
from memory_debug import get_process_memory_mb, estimate_track_proximity_cache_mb
from obstacle_map import (
    add_net_stubs_as_obstacles, add_net_vias_as_obstacles, add_net_pads_as_obstacles,
    add_same_net_via_clearance, add_same_net_pad_drill_via_clearance,
    VisualizationData
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
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers, invalidate_obstacle_cache, record_frontier_blocking
from rip_up_reroute import rip_up_net, restore_net
from diff_pair_custody import record_casualty
from polarity_swap import get_canonical_net_id, rip_combo_already_tried
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


def _swap_blocking_net_ids(pcb_data, stub, dest_layer, config, moved_segs):
    """Foreign nets whose copper on dest_layer conflicts with the moved stub
    footprint (the identities behind a validate_single_swap failure). Pads
    are excluded -- they are never rippable; only track/via owners return."""
    from geometry_utils import point_to_segment_distance
    need = config.track_width + config.clearance
    hits = set()
    pts = []
    for ms in moved_segs:
        n = max(2, int(((ms.start_x - ms.end_x) ** 2 +
                        (ms.start_y - ms.end_y) ** 2) ** 0.5 / 0.2) + 1)
        for i in range(n + 1):
            t = i / n
            pts.append((ms.start_x + (ms.end_x - ms.start_x) * t,
                        ms.start_y + (ms.end_y - ms.start_y) * t))
    for s in pcb_data.segments:
        if s.net_id == stub.net_id or s.layer != dest_layer:
            continue
        if any(point_to_segment_distance(px, py, s.start_x, s.start_y,
                                         s.end_x, s.end_y) < need
               for px, py in pts):
            hits.add(s.net_id)
    for v in pcb_data.vias:
        if v.net_id == stub.net_id:
            continue
        r = v.size / 2.0 + config.track_width / 2.0 + config.clearance
        if any((px - v.x) ** 2 + (py - v.y) ** 2 < r * r for px, py in pts):
            hits.add(v.net_id)
    return hits


def _tap_relocation_rescue(pcb_data, net_id, config, state, results,
                           routed_net_ids, remaining_net_ids, routed_results,
                           routed_net_paths, track_proximity_cache, layer_map,
                           obstacle_cache, target_xy=None, source_xy=None,
                           blamed_net_ids=None):
    """Single-tap relocation rung (#424 planes-first): when the pocket is
    walled by a PRE-EXISTING plane-net tap via, remove that one via (+ its
    dogbone stub) via exact whole-net cache recompute, retry, and let the
    plane-repair step re-tap the pad from the pour later in the chain.
    Returns the successful route result or None (everything restored)."""
    from tap_relocation import (find_relocation_candidate, relocate_tap,
                                restore_tap, tap_relocation_enabled,
                                tap_relocation_max)
    if not tap_relocation_enabled():
        return None
    if state.working_obstacles is None or state.net_obstacles_cache is None:
        return None
    tried = set()
    for _attempt in range(tap_relocation_max()):
        via = None
        for xy in (target_xy, source_xy):
            via = find_relocation_candidate(
                pcb_data, config, state.net_obstacles_cache, xy,
                blamed_net_ids=blamed_net_ids, exclude_via_ids=tried)
            if via is not None:
                break
        if via is None:
            return None
        tried.add(id(via))
        token = relocate_tap(pcb_data, config, state.working_obstacles,
                             state.net_obstacles_cache, via)
        if token is None:
            continue
        result = route_net_with_obstacles(pcb_data, net_id, config,
                                          state.working_obstacles)
        if result and not result.get('failed'):
            from tap_relocation import retap_pad
            if not retap_pad(pcb_data, config, state.working_obstacles,
                             state.net_obstacles_cache, token):
                # No legal replacement tap -> the relocation is not
                # allowed to stand; drop the route and put the tap back.
                print(f"  TAP RELOCATION: re-tap declined -> reverting "
                      f"(route discarded)")
                restore_tap(pcb_data, state.working_obstacles,
                            state.net_obstacles_cache, token)
                continue
            net = pcb_data.nets.get(net_id)
            nname = net.name if net else str(net_id)
            print(f"  TAP RELOCATION RESCUE: {nname} routed after relocating "
                  f"1 plane tap ({len(result['new_segments'])} segments)")
            record_net_event(state, net_id, "tap_relocation_rescue", {
                "moved_net_id": token['net_id'],
                "via_xy": (token['via'].x, token['via'].y)})
            # Standard success bookkeeping (mirrors the stub-swap rescue).
            result['route_length'] = calculate_route_length(
                result['new_segments'], result.get('new_vias', []), pcb_data)
            results.append(result)
            add_route_to_pcb_data(pcb_data, result,
                                  debug_lines=config.debug_lines)
            if net_id in remaining_net_ids:
                remaining_net_ids.remove(net_id)
            routed_net_ids.append(net_id)
            routed_results[net_id] = result
            if result.get('path'):
                routed_net_paths[net_id] = result['path']
            track_proximity_cache[net_id] = compute_track_proximity_for_net(
                pcb_data, net_id, config, layer_map)
            update_net_obstacles_after_routing(
                pcb_data, net_id, result, config, state.net_obstacles_cache)
            add_net_obstacles_from_cache(
                state.working_obstacles, state.net_obstacles_cache[net_id])
            invalidate_obstacle_cache(obstacle_cache, net_id)
            return result
        restore_tap(pcb_data, state.working_obstacles,
                    state.net_obstacles_cache, token)
    return None


def _stub_swap_rescue(pcb_data, net_id, config, state, results,
                      routed_net_ids, remaining_net_ids, routed_results,
                      routed_net_paths, track_proximity_cache, layer_map,
                      obstacle_cache, diff_pair_by_net_id=None,
                      reroute_queue=None, queued_net_ids=None):
    """Final rung of the SE failure ladder (#424): re-layer the net's OWN
    stub(s) and re-route.

    The upfront optimizer (layer_swap_optimization) swaps stubs BEFORE
    routing; diff pairs additionally get a failure-driven swap in the reroute
    loop (layer_swap_fallback). Single-ended nets had neither once routing
    started: a stub pinned to a congested layer stayed there through every
    rip-up rung. This rung tries each stub on each other routable layer
    (validated: no orphaned SMD pad, no stub overlap, pad-via inserted when
    leaving the pad's layer), re-routes through free space, and keeps the
    first success -- else reverts copper exactly.

    Returns the successful route result, or None. On success all standard
    bookkeeping is done here (results/caches/working map/segment mods for
    the writer)."""
    if not getattr(config, 'stub_layer_swap', True) or len(config.layers) < 2:
        return None
    from connectivity import get_stub_endpoints
    from stub_layer_switching import (get_stub_info, validate_single_swap,
                                      apply_stub_layer_switch,
                                      revert_stub_layer_switch)
    from single_ended_routing import route_net_with_obstacles

    ends = get_stub_endpoints(pcb_data, [net_id])
    if not ends:
        return None
    # Stub-vs-stub overlap registry for the validator: every OTHER net's
    # segments grouped by layer (rescues are rare; a linear scan is fine).
    all_stubs_by_layer = {}
    by_net_layer = {}
    for s in pcb_data.segments:
        if s.net_id == net_id:
            continue
        by_net_layer.setdefault((s.net_id, s.layer), []).append(s)
    for (nid, layer), segs in by_net_layer.items():
        nname = pcb_data.nets[nid].name if nid in pcb_data.nets else str(nid)
        all_stubs_by_layer.setdefault(layer, []).append((nname, segs))

    # Candidate layers cheapest-first, skipping forbidden (negative cost).
    costs = config.get_layer_costs() or [1000] * len(config.layers)
    cand_layers = [l for l, c in sorted(zip(config.layers, costs),
                                        key=lambda lc: lc[1]) if c >= 0]

    from stub_layer_switching import connected_stub_segments_on_layer
    from rip_up_reroute import rip_up_net, restore_net

    def _rip_blocker(bnid):
        """Rip a routed blocker through the custody primitives. Returns the
        restore record or None if not rippable."""
        if bnid not in routed_results or bnid in rip_exclude_set(state, net_id):
            return None
        if diff_pair_by_net_id and bnid in diff_pair_by_net_id:
            return None    # v1: leave diff pairs alone
        saved, ripped_ids, was_in = rip_up_net(
            bnid, pcb_data, routed_net_ids, routed_net_paths,
            routed_results, diff_pair_by_net_id or {}, remaining_net_ids,
            results, config, track_proximity_cache,
            state.working_obstacles, state.net_obstacles_cache,
            state.ripped_route_layer_costs, state.ripped_route_via_positions,
            layer_map)
        if saved is None:
            return None
        return (bnid, saved, ripped_ids, was_in)

    def _restore_rips(rips):
        for bnid, saved, ripped_ids, was_in in reversed(rips):
            restore_net(bnid, saved, ripped_ids, was_in,
                        pcb_data, routed_net_ids, routed_net_paths,
                        routed_results, diff_pair_by_net_id or {},
                        remaining_net_ids, results, config,
                        track_proximity_cache, layer_map,
                        state.working_obstacles, state.net_obstacles_cache,
                        state.ripped_route_layer_costs,
                        state.ripped_route_via_positions,
                        refused_sink=state.collision_refused_net_ids)

    for (sx, sy, slayer) in ends[:2]:
        stub = get_stub_info(pcb_data, net_id, sx, sy, slayer)
        if stub is None:
            continue
        for dest in cand_layers:
            if dest == slayer:
                continue
            rips = []
            ok, _why = validate_single_swap(stub, dest, all_stubs_by_layer,
                                            pcb_data, config)
            if not ok and reroute_queue is not None:
                # The swap itself is blocked (#424): identify the blocking
                # nets and rip the rippable ones (custody primitives only --
                # rip_up_net/restore_net keep the working map, caches and
                # ripped-cost ledgers consistent), then re-validate. Victims
                # are queued for reroute on success, restored on failure.
                moved = connected_stub_segments_on_layer(
                    pcb_data, stub.net_id, stub.layer, stub.segments)
                for bnid in list(_swap_blocking_net_ids(
                        pcb_data, stub, dest, config, moved))[:2]:
                    rec = _rip_blocker(bnid)
                    if rec is not None:
                        rips.append(rec)
                if rips:
                    ok, _why = validate_single_swap(
                        stub, dest, all_stubs_by_layer, pcb_data, config)
            if not ok:
                _restore_rips(rips)
                continue
            new_vias, seg_mods = apply_stub_layer_switch(
                pcb_data, stub, dest, config, debug=False)
            result = route_net_with_obstacles(pcb_data, net_id, config,
                                              state.working_obstacles)
            if result and not result.get('failed'):
                net = pcb_data.nets.get(net_id)
                nname = net.name if net else str(net_id)
                print(f"  STUB-SWAP RESCUE: {nname} stub {slayer} -> {dest} "
                      f"routed ({len(result['new_segments'])} segments, "
                      f"{len(result['new_vias'])} vias)")
                record_net_event(state, net_id, "stub_swap_rescue", {
                    "from_layer": slayer, "to_layer": dest,
                    "pad_vias": len(new_vias)})
                # Persist the swap for the output writer.
                state.all_segment_modifications.extend(seg_mods)
                state.all_swap_vias.extend(new_vias)
                # Standard success bookkeeping (mirrors the retry-success path).
                result['route_length'] = calculate_route_length(
                    result['new_segments'], result.get('new_vias', []), pcb_data)
                results.append(result)
                add_route_to_pcb_data(pcb_data, result,
                                      debug_lines=config.debug_lines)
                if net_id in remaining_net_ids:
                    remaining_net_ids.remove(net_id)
                routed_net_ids.append(net_id)
                routed_results[net_id] = result
                if result.get('path'):
                    routed_net_paths[net_id] = result['path']
                track_proximity_cache[net_id] = compute_track_proximity_for_net(
                    pcb_data, net_id, config, layer_map)
                if state.working_obstacles is not None and \
                        state.net_obstacles_cache is not None:
                    update_net_obstacles_after_routing(
                        pcb_data, net_id, result, config,
                        state.net_obstacles_cache)
                    add_net_obstacles_from_cache(
                        state.working_obstacles,
                        state.net_obstacles_cache[net_id])
                invalidate_obstacle_cache(obstacle_cache, net_id)
                # Ripped swap-blockers: custody + reroute queue (T5 pattern,
                # same as the rip ladder's success path).
                for bnid, saved, ripped_ids, was_in in rips:
                    record_casualty(state, bnid, saved, ripped_ids, was_in)
                    if queued_net_ids is not None and bnid not in queued_net_ids:
                        bnet = pcb_data.nets.get(bnid)
                        reroute_queue.append(
                            ('single', bnet.name if bnet else str(bnid), bnid))
                        queued_net_ids.add(bnid)
                    if was_in:
                        result['_rescue_rip_was_in_results'] = \
                            result.get('_rescue_rip_was_in_results', 0) + 1
                return result
            revert_stub_layer_switch(pcb_data, seg_mods, new_vias)
            _restore_rips(rips)
    return None


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
    bus_corridors: Dict[str, List[Tuple[int, int, int]]] = {}  # group.name -> planned centerline

    _bus_ordering = (config.bus_enabled or
                     getattr(config, 'ordering_strategy', '') == 'bus')
    if _bus_ordering:
        net_ids_to_check = [net_id for _, net_id in single_ended_nets]
        bus_groups = detect_bus_groups(
            pcb_data, net_ids_to_check,
            detection_radius=config.bus_detection_radius,
            min_nets=config.bus_min_nets
        )
        # Strict geometric bus definition (see filter_bus_groups_geometric):
        # only groups that genuinely travel together survive; raw cliques
        # bundled decoupling clusters and power rails and planned corridors
        # for all of them. KICAD_BUS_STRICT=0 reverts.
        from bus_detection import filter_bus_groups_geometric
        _raw_n = len(bus_groups)
        bus_groups = filter_bus_groups_geometric(pcb_data, bus_groups, config)
        if _raw_n != len(bus_groups):
            print(f"  Bus filter: {_raw_n} raw clique group(s) -> "
                  f"{len(bus_groups)} geometric bus(es)")

        if bus_groups:
            print(f"\n=== Bus Detection: Found {len(bus_groups)} bus group(s) ===")
            for bus in bus_groups:
                direction = "targets->sources" if bus.clique_endpoint == "target" else "sources->targets"
                print(f"  {bus.name}: {len(bus.net_ids)} nets ({direction})", end =" ")
                net_names_list = [pcb_data.nets[nid].name for nid in bus.net_ids]
                print(f"physical order: {' -> '.join(net_names_list)}")

                if config.verbose:
                # Show routing order with guide track and attraction info
                    route_order = get_bus_routing_order(bus)
                    route_names = [pcb_data.nets[nid].name for nid in route_order]
                    print(f"    Routing order:  {' -> '.join(route_names)}")
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

                # Track bus membership. Attraction reads bus_net_to_group,
                # so it fills only under --bus; the 'bus' ORDERING strategy
                # alone must not switch attraction on.
                if config.bus_enabled:
                    for nid in bus.net_ids:
                        bus_net_to_group[nid] = bus

            # Corridor planning (#296 R9): probe-route each group's
            # representative at a ladder of inflated clearances and attract
            # EVERY member (including the guide) to the winning centerline,
            # so the corridor is chosen with room for the whole group instead
            # of being whatever the guide's solo path happened to be. Soft:
            # groups with no routable rung keep neighbor attraction.
            if config.bus_enabled:
                from bus_corridor import plan_bus_corridors
                bus_corridors, _demoted = plan_bus_corridors(
                    pcb_data, config, bus_groups, verbose=config.verbose)
                # Demoted groups (corridor needs too many layer changes even
                # at the laminar probe price) lose bus treatment entirely:
                # no corridor, no attraction, no bus ordering -- their
                # members route as plain nets.
                if _demoted:
                    _dset = set(_demoted)
                    for bus in [b for b in bus_groups if b.name in _dset]:
                        for _nid in bus.net_ids:
                            bus_net_to_group.pop(_nid, None)
                    bus_groups = [b for b in bus_groups if b.name not in _dset]

            # Reorder nets: bus nets in routing order first, then non-bus nets
            bus_net_ids_set = {nid for bus in bus_groups for nid in bus.net_ids}
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

            # Rip-resistance + reroute-attraction transport: the ladder rank
            # (blocking_analysis) and the reroute loop run far from this
            # scope, so carry membership on config/state (the
            # corridor_waypoints precedent for config-attached data).
            config.bus_member_net_ids = bus_net_ids_set
            state.bus_net_to_group = bus_net_to_group
            state.bus_corridors = bus_corridors
            # Net-story recording: each member's group/position/corridor.
            for _bnid, _bg in bus_net_to_group.items():
                record_net_event(state, _bnid, "bus_member", {
                    "group": _bg.name,
                    "position": (_bg.net_ids.index(_bnid)
                                 if _bnid in _bg.net_ids else None),
                    "members": len(_bg.net_ids),
                    "corridor_planned": _bg.name in bus_corridors})

    # Checkpoint abort (KICAD_STOP_AFTER / KICAD_STOP_FILE): stop routing at
    # a defined point, skip every later pass, and WRITE the partial board --
    # for inspecting intermediate states (e.g. "the board right after the
    # bus members routed") with the run's exact configuration.
    #   KICAD_STOP_AFTER=bus  -> stop once the last bus-member net has been
    #     attempted (the routing order interleaves, so non-bus nets that come
    #     earlier are included -- that IS the real board state at that
    #     moment).
    #   KICAD_STOP_FILE=path  -> stop before the next net once the file
    #     exists (manual abort: `touch path`).
    _stop_after = os.environ.get('KICAD_STOP_AFTER', '')
    _stop_file = os.environ.get('KICAD_STOP_FILE', '')
    _stop_after_idx = None
    if _stop_after == 'bus' and bus_net_to_group:
        _members = set(bus_net_to_group)
        for _i, (_nm, _nid) in enumerate(single_ended_nets):
            if _nid in _members:
                _stop_after_idx = _i
    _pos = -1
    for net_name, net_id in single_ended_nets:
        _pos += 1
        if _stop_after_idx is not None and _pos > _stop_after_idx:
            print(f"\nCHECKPOINT STOP (KICAD_STOP_AFTER=bus): all "
                  f"{len(bus_net_to_group)} bus members attempted; writing "
                  f"partial board")
            state.checkpoint_stop = True
            break
        if _stop_file and os.path.exists(_stop_file):
            print(f"\nCHECKPOINT STOP (KICAD_STOP_FILE): {_stop_file} exists; "
                  f"writing partial board")
            state.checkpoint_stop = True
            break
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
        # #422 probe: env-gated live-set audit (Rust map get_stats + Python caches)
        if os.environ.get('KICAD_MEM_PROBE') and (route_index % 10 == 1 or route_index == total_routes):
            try:
                import gc as _gc
                from memory_debug import (estimate_net_obstacles_cache_mb as _enc,
                                          format_obstacle_map_stats as _foms)
                _cache_mb = _enc(state.net_obstacles_cache) if state.net_obstacles_cache else 0.0
                print(f"[MEMPROBE] route={route_index}/{total_routes} pid={os.getpid()} "
                      f"net_obstacles_cache={_cache_mb:.1f}MB "
                      f"prox_cache={estimate_track_proximity_cache_mb(track_proximity_cache):.1f}MB "
                      f"routed_paths={len(routed_net_paths)} "
                      f"ripped_layer_costs={len(state.ripped_route_layer_costs)} "
                      f"ripped_via_pos={len(state.ripped_route_via_positions)}", flush=True)
                print("[MEMPROBE] " + _foms(state.working_obstacles).replace("\n", "\n[MEMPROBE] "), flush=True)
            except Exception as _e:
                print(f"[MEMPROBE] error: {_e}", flush=True)

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
            # Bus attraction context, computed BEFORE the multipoint split:
            # planned corridor first (#296 R9), routed-neighbor fallback,
            # clustered-endpoint direction. Multipoint members use it for
            # main-edge selection + attraction too (they used to route blind).
            attraction_path, reverse_direction = bus_attraction_context(
                net_id, bus_net_to_group, bus_corridors, bus_routed_paths)
            # Off-lane surcharge (the stick): members with a corridor pay
            # scaled step costs everywhere EXCEPT near the lane, where the
            # attraction discount compensates -- defection costs real money.
            cfg_route = bus_stick_config(config, attraction_path)
            # Check for multi-point net (3+ pads, no existing segments)
            multipoint_pads = get_multipoint_net_pads(pcb_data, net_id, config)
            if multipoint_pads:
                print(f"  Detected multi-point net with {len(multipoint_pads)} pads (Phase 1: main route only)")
                result = route_multipoint_main(pcb_data, net_id, cfg_route, obstacles, multipoint_pads,
                                               attraction_path=attraction_path, state=state)
                # Track for Phase 3 completion after length matching
                if result and not result.get('failed') and result.get('is_multipoint'):
                    state.pending_multipoint_nets[net_id] = result
            else:
                result = route_net_with_obstacles(pcb_data, net_id, cfg_route, obstacles,
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
                        exclude_net_ids=rip_exclude_set(state, net_id),
                        target_xy=single_target_xy,
                        source_xy=single_source_xy,
                        obstacle_cache=obstacle_cache
                    )
                    # Net-story recording: the named wall at this failure.
                    record_net_event(state, net_id, "blocked_by", {
                        "blockers": [
                            {"net": b.net_name, "cells": b.blocked_count,
                             "unique": b.unique_cells,
                             "near_target": b.near_target_cells,
                             "validator_named": b.validator_named}
                            for b in (blockers or [])[:6]]})
                    record_frontier_blocking(state, net_id, blockers,
                                             "single_ended")
                    print_blocking_analysis(blockers, blocked_cells=blocked_cells,
                                           pcb_data=pcb_data, config=config,
                                           nets_to_route=remaining_net_ids)

                    # Filter to only rippable blockers (those in routed_results)
                    # and deduplicate by diff pair (P and N count as one)
                    rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
                        blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id
                    )
                    # Blocker-selection algorithm (#424 audit; --ripup-blocker-select).
                    _bsel = getattr(config, 'ripup_blocker_select', 'count')
                    if _bsel == 'mincut' and rippable_blockers and \
                            state.working_obstacles is not None and \
                            state.net_obstacles_cache is not None:
                        from blocking_analysis import mincut_probe_order
                        _mc_order, _mc_feasible = mincut_probe_order(
                            pcb_data, config, state.working_obstacles, net_id,
                            rippable_blockers, state.net_obstacles_cache)
                        if not _mc_feasible:
                            # No PATH even with rippable copper soft-costed --
                            # but a ladder retry is more than a re-search:
                            # ripping frees space that lets the pad-via and
                            # swap rescues change the topology, which the
                            # probe does not model. Keep the ladder, legacy
                            # order (measured: skipping here lost nets the
                            # count ladder's side-mechanisms recovered).
                            print(f"  MINCUT probe: no path with rippable "
                                  f"copper soft-costed (wall is static) -- "
                                  f"falling back to count order")
                        elif _mc_order:
                            _by_id = {b.net_id: b for b in rippable_blockers}
                            _front = [_by_id[n] for n in _mc_order if n in _by_id]
                            _rest = [b for b in rippable_blockers
                                     if b.net_id not in set(_mc_order)]
                            rippable_blockers = _front + _rest
                            print(f"  MINCUT probe: true cut = "
                                  f"{[b.net_name for b in _front]}")
                    elif _bsel == 'bidir' and rippable_blockers and result:
                        _fwd = result.get('blocked_cells_forward') or []
                        _bwd = result.get('blocked_cells_backward') or []
                        if _fwd and _bwd:
                            from blocking_analysis import rank_blockers
                            _f_ids = {b.net_id for b in analyze_frontier_blocking(
                                _fwd, pcb_data, config, routed_net_paths,
                                exclude_net_ids={net_id},
                                obstacle_cache=obstacle_cache)}
                            _b_ids = {b.net_id for b in analyze_frontier_blocking(
                                _bwd, pcb_data, config, routed_net_paths,
                                exclude_net_ids={net_id},
                                obstacle_cache=obstacle_cache)}
                            rank_blockers(rippable_blockers, 'bidir',
                                          bidir_both_sets=_f_ids & _b_ids,
                                          config=config)

                    # #331 item 3: a via-in-pad unblock that DECLINED during
                    # this net's attempts recorded WHICH net's copper boxes
                    # the pad (_place_shrunk_via_in_pad). Frontier attribution
                    # cannot see that copper (the search never reaches under
                    # the pad), so it blames adjacent bystanders - put the
                    # named keystone at the FRONT of the rip ladder instead.
                    _blame = getattr(pcb_data, '_via_unblock_blame', None)
                    blame_ids = _blame.pop(net_id, None) if _blame else None
                    if blame_ids:
                        from blocking_analysis import BlockingInfo
                        excluded = rip_exclude_set(state, net_id)
                        _unrippable_blame = []
                        for bid in sorted(blame_ids):
                            if bid in excluded:
                                continue
                            if bid not in routed_results:
                                # Validator-proved blocker that is PRE-EXISTING
                                # copper: surface the rip-existing hint NOW
                                # instead of only after the ladder exhausts.
                                _n = pcb_data.nets.get(bid)
                                _unrippable_blame.append(_n.name if _n else str(bid))
                                continue
                            canonical = get_canonical_net_id(bid, diff_pair_by_net_id)
                            if canonical in seen_canonical_ids:
                                continue
                            seen_canonical_ids.add(canonical)
                            bname = pcb_data.nets[bid].name if bid in pcb_data.nets else str(bid)
                            print(f"  Via-in-pad decline blamed {bname} "
                                  f"(copper under the boxed pad) - ripping it first")
                            rippable_blockers.insert(0, BlockingInfo(
                                net_id=bid, net_name=bname, blocked_count=0,
                                track_cells=0, via_cells=0, unique_cells=0,
                                near_target_cells=0, near_source_cells=0,
                                details="via-in-pad decline blame (#331)"))
                        if _unrippable_blame:
                            print(f"  Via-in-pad decline blamed PRE-EXISTING net(s) "
                                  f"{', '.join(repr(n) for n in _unrippable_blame)} - not rippable "
                                  f"this run; retry with --rip-existing-nets to authorize")

                    # Progressive rip-up: try N=1, then N=2, etc up to max_rip_up_count
                    ripped_items = []
                    ripped_canonical_ids = set()  # Track which canonicals have been ripped
                    retry_succeeded = False
                    last_retry_blocked_cells = blocked_cells  # Start with initial failure's blocked cells

                    # while-form so the slot-0 guard can re-run N=1 once with
                    # a promoted pick (audit, structural fix).
                    slot0_guard_used = False
                    N = 0
                    while N < config.max_rip_up_count:
                        N += 1
                        # For N > 1, re-analyze from the last retry's blocked cells
                        # to find the most blocking net from that specific failure
                        if N > 1 and last_retry_blocked_cells:
                            print(f"  Re-analyzing {len(last_retry_blocked_cells)} blocked cells from N={N-1} retry:")
                            fresh_blockers = analyze_frontier_blocking(
                                last_retry_blocked_cells, pcb_data, config, routed_net_paths,
                                exclude_net_ids=rip_exclude_set(state, net_id),
                                target_xy=single_target_xy,
                                source_xy=single_source_xy,
                                obstacle_cache=obstacle_cache
                            )
                            record_frontier_blocking(state, net_id,
                                                     fresh_blockers,
                                                     "single_ended")
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
                            # Slot-0 guard (audit, structural): the ladder only
                            # grows a PREFIX around slot 0, so a wrong first
                            # pick is a member of every combo it will ever try.
                            # If the singleton just failed and the re-analysis
                            # names a DIFFERENT top pick, restore the refuted
                            # rip and try the fresh pick ALONE once before
                            # extending to pairs.
                            if (N == 2 and not slot0_guard_used
                                    and len(ripped_items) == 1
                                    and next_blocker.net_id in routed_results):
                                _c_next = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
                                _c_slot0 = get_canonical_net_id(rippable_blockers[0].net_id, diff_pair_by_net_id)
                                if _c_next != _c_slot0:
                                    slot0_guard_used = True
                                    print(f"  Slot-0 guard: re-analysis refutes {rippable_blockers[0].net_name} -> "
                                          f"retrying {next_blocker.net_name} ALONE first")
                                    for rid, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
                                        restore_net(rid, saved_result, ripped_ids, was_in_results,
                                                   pcb_data, routed_net_ids, routed_net_paths,
                                                   routed_results, diff_pair_by_net_id, remaining_net_ids,
                                                   results, config, track_proximity_cache, layer_map,
                                                   state.working_obstacles, state.net_obstacles_cache,
                                                   state.ripped_route_layer_costs, state.ripped_route_via_positions, refused_sink=state.collision_refused_net_ids)
                                        if was_in_results:
                                            successful += 1
                                    ripped_items = []
                                    ripped_canonical_ids.discard(_c_slot0)
                                    seen_canonical_ids.add(_c_next)
                                    rippable_blockers = ([next_blocker] +
                                        [b for b in rippable_blockers
                                         if get_canonical_net_id(b.net_id, diff_pair_by_net_id) != _c_next])
                                    N = 0
                                    continue
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

                        # Shared rip-history gate (#376): computes the canonical
                        # blocker set, logs+skips if this combo was already tried.
                        already_tried, blocker_canonicals = rip_combo_already_tried(
                            rip_and_retry_history, net_id, net_name,
                            rippable_blockers, N, diff_pair_by_net_id)
                        if already_tried:
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
                                record_rip_ancestry(state, net_id, rid)
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
                                           state.ripped_route_layer_costs, state.ripped_route_via_positions, refused_sink=state.collision_refused_net_ids)
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

                        # Bus attraction for the retry, multipoint included
                        retry_attraction_path, retry_reverse_direction = bus_attraction_context(
                            net_id, bus_net_to_group, bus_corridors, bus_routed_paths)
                        retry_cfg = bus_stick_config(config, retry_attraction_path)
                        # Check for multi-point net in retry as well
                        retry_multipoint_pads = get_multipoint_net_pads(pcb_data, net_id, config)
                        if retry_multipoint_pads:
                            retry_result = route_multipoint_main(pcb_data, net_id, retry_cfg, retry_obstacles, retry_multipoint_pads,
                                                                 attraction_path=retry_attraction_path, state=state)
                            # Track for Phase 3 completion after length matching
                            if retry_result and not retry_result.get('failed') and retry_result.get('is_multipoint'):
                                state.pending_multipoint_nets[net_id] = retry_result
                        else:
                            retry_result = route_net_with_obstacles(pcb_data, net_id, retry_cfg, retry_obstacles,
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
                                # Committed rip: custody of the pre-rip copper so
                                # the end-of-run casualties-only reconcile can
                                # restore it if the queued reroute never lands
                                # (T5 zero-copper custody; route.py parity with
                                # the diff front's 43e6d10).
                                record_casualty(state, rid, saved_result,
                                                ripped_ids, was_in_results)
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
                                       state.ripped_route_layer_costs, state.ripped_route_via_positions, refused_sink=state.collision_refused_net_ids)
                            if was_in_results:
                                successful += 1

            if not ripped_up:
                record_net_event(state, net_id, "reroute_failed", {
                    "reason": "no rippable blockers found"
                })
                print(f"  {RED}ROUTE FAILED - no rippable blockers found{RESET}")
                from routing_diagnostics import static_boxin_hint, preexisting_blocker_hint
                hint = static_boxin_hint(result, config, pcb_data)
                if hint:
                    print(f"  {hint}")
                # #301: the blockers may be PRE-EXISTING copper (earlier run/
                # step) the rip-up attribution cannot see -- name them and the
                # --rip-existing-nets retry. Cells were popped into
                # blocked_cells when the rip-up branch ran; else read them off
                # the result.
                _cells301 = []
                if result:
                    _cells301 = ((result.get('blocked_cells_forward') or []) +
                                 (result.get('blocked_cells_backward') or []))
                if not _cells301:
                    _cells301 = list(locals().get('blocked_cells') or [])
                hint301, blockers301 = preexisting_blocker_hint(
                    _cells301, config, pcb_data, net_id,
                    routed_net_ids=state.routed_net_ids, return_names=True)
                if hint301:
                    print(f"  {hint301}")
                    record_net_event(state, net_id, "preexisting_blockers", {
                        "hint": hint301, "blockers": blockers301})
                # Tap-relocation rung (#424 planes-first, env-gated): a
                # walled-in pocket whose wall is a pre-existing plane TAP
                # gets one-tap surgery before the stub-swap rung.
                _rescued = _tap_relocation_rescue(
                    pcb_data, net_id, config, state, results,
                    routed_net_ids, remaining_net_ids, routed_results,
                    routed_net_paths, track_proximity_cache, layer_map,
                    obstacle_cache,
                    target_xy=locals().get('single_target_xy'),
                    source_xy=locals().get('single_source_xy'),
                    blamed_net_ids=locals().get('blame_ids'))
                # Final rung (#424): try re-layering the net's own stub(s).
                if _rescued is None:
                    _rescued = _stub_swap_rescue(
                        pcb_data, net_id, config, state, results,
                        routed_net_ids, remaining_net_ids, routed_results,
                        routed_net_paths, track_proximity_cache, layer_map,
                        obstacle_cache,
                        diff_pair_by_net_id=diff_pair_by_net_id,
                        reroute_queue=reroute_queue,
                        queued_net_ids=queued_net_ids)
                if _rescued is not None:
                    successful += 1
                    successful -= _rescued.pop('_rescue_rip_was_in_results', 0)
                    total_iterations += _rescued['iterations']
                else:
                    failed += 1

    return successful, failed, total_time, total_iterations, route_index, user_quit
