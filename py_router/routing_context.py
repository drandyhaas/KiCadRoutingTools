"""
Routing context helpers for PCB routing.

This module provides helper functions for common routing operations
like building obstacle maps and recording route results.
"""
from __future__ import annotations

from typing import List, Set, Dict, Optional, Tuple, TYPE_CHECKING
import math
import numpy as np
import env_knobs
from routing_config import GridCoord, GridRouteConfig
from obstacle_map import (
    add_net_stubs_as_obstacles, add_net_vias_as_obstacles, add_net_pads_as_obstacles,
    add_same_net_via_clearance, add_same_net_pad_drill_via_clearance,
    get_same_net_through_hole_positions
)
from obstacle_costs import (
    add_stub_proximity_costs, apply_stub_proximity,
    merge_track_proximity_costs,
    add_cross_layer_tracks, compute_track_proximity_for_net
)
from obstacle_cache import (
    NetObstacleData, add_net_obstacles_from_cache, remove_net_obstacles_from_cache
)
from connectivity import get_stub_endpoints
from net_queries import get_chip_pad_positions
from pcb_modification import add_route_to_pcb_data


def _add_free_via_positions(obstacles, pcb_data, net_ids: List[int], config):
    """Add through-hole pads as free via positions (zero-cost layer change).

    Args:
        obstacles: GridObstacleMap to add free via positions to
        pcb_data: PCB data with pads_by_net
        net_ids: List of net IDs to add through-hole positions for
        config: Routing config with grid_step
    """
    coord = GridCoord(config.grid_step)
    free_via_positions = []
    for net_id in net_ids:
        for pad in pcb_data.pads_by_net.get(net_id, []):
            if pad.drill and pad.drill > 0:
                gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                free_via_positions.append((gx, gy))
    # Every EXISTING same-net via (fanout via-in-pad, a prior route's via, or a
    # board via) is a reusable zero-cost layer transition, so the router reuses it
    # instead of dropping its own beside it. The emitted layer-change via at such a
    # position must be REUSED, not added (see via dedup at route conversion).
    net_id_set = set(net_ids)
    for v in pcb_data.vias:
        if v.net_id in net_id_set:
            free_via_positions.append(coord.to_grid(v.x, v.y))
    if free_via_positions:
        obstacles.add_free_vias_batch(free_via_positions)


def filter_ripped_ghosts(ghost_dict, config: GridRouteConfig, routed_net_ids=None):
    """C1 filter for ripped-route ghost dicts (layer costs OR via positions).

    Nets that have ROUTED again since being ripped are skipped (soft-knobs
    review C1): once the net has real copper down, the reserved corridor is
    either re-occupied (the real obstacles suffice) or empty and useful, and
    the ghost would repel every remaining net from it for the rest of the run.
    Returns {} when the avoidance feature is off. Both composition entry
    points (merge_track_proximity_costs for the layer ghosts,
    apply_stub_proximity for the via ghosts) require pre-filtered dicts so
    each source is counted exactly once.
    """
    if not ghost_dict or config.ripped_route_avoidance_cost <= 0:
        return {}
    done = set(routed_net_ids or ())
    return {nid: v for nid, v in ghost_dict.items()
            if nid not in done and v is not None and len(v) > 0}


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
    add_own_stubs_func=None,
    net_obstacles_cache: Optional[Dict[int, NetObstacleData]] = None,
    ripped_route_layer_costs: Dict[int, np.ndarray] = None,
    ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None
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
        net_obstacles_cache: Optional pre-computed net obstacles for fast batch adding
        ripped_route_layer_costs: Optional dict of ripped route layer-specific costs
        ripped_route_via_positions: Optional dict of ripped route via positions

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    obstacles = diff_pair_base_obstacles.clone_fresh()

    # Add previously routed nets as obstacles
    # Note: Cannot use cache for routed nets because their segments have changed
    for routed_id in routed_net_ids:
        add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)
        add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)
        add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config, extra_clearance)

    # Add GND vias as obstacles
    if gnd_net_id is not None:
        add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, extra_clearance)

    # Add other unrouted nets as obstacles (can use cache since they haven't changed)
    other_unrouted = [nid for nid in remaining_net_ids
                     if nid != p_net_id and nid != n_net_id]
    for other_net_id in other_unrouted:
        if net_obstacles_cache and other_net_id in net_obstacles_cache:
            add_net_obstacles_from_cache(obstacles, net_obstacles_cache[other_net_id])
        else:
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
    if config.verbose:
        print(f"    stub proximity: {len(stub_proximity_net_ids)} nets, {len(unrouted_stubs)} stubs, {len(chip_pads)} chip pads")
    _ghost_vias = filter_ripped_ghosts(ripped_route_via_positions, config, routed_net_ids)
    _stub_surplus = apply_stub_proximity(obstacles, pcb_data, stub_proximity_net_ids,
                                         all_stubs, config,
                                         ghost_via_groups=_ghost_vias,
                                         layer_map=layer_map)

    # Add track proximity costs (+ ripped-corridor layer ghosts + layer-aware
    # stub surplus, one composition pass). Congestion v2 stays out of the
    # diff-pair path (it never stamped here -- its owner exemption is
    # single-net).
    from history_congestion import add_history_source
    merge_track_proximity_costs(
        obstacles, track_proximity_cache,
        ghost_costs=add_history_source(
            {**filter_ripped_ghosts(ripped_route_layer_costs, config, routed_net_ids),
             **(_stub_surplus or {})}, config),
        config=config)

    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={p_net_id, n_net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, p_net_id, config)
    add_same_net_via_clearance(obstacles, pcb_data, n_net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, p_net_id, config)
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, n_net_id, config)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(obstacles, pcb_data, [p_net_id, n_net_id], config)

    # Add own stubs as obstacles if function provided
    if add_own_stubs_func:
        add_own_stubs_func(obstacles, pcb_data, p_net_id, n_net_id, config, extra_clearance)

    return obstacles, all_stubs


def build_diff_pair_leg_obstacles(base_obstacles, pcb_data, config, routed_net_ids,
                                  remaining_net_ids, all_unrouted_net_ids,
                                  p_net_id, n_net_id, gnd_net_id,
                                  track_proximity_cache, layer_map):
    """Obstacle map for a hybrid pair's SINGLE-ENDED legs (issue #246 review).

    The legs are one-track point-to-point routes, so they must use single-ended
    clearance -- NOT the coupled diff-pair clearance:

    - base = ``base_obstacles`` (no extra clearance baked in), NOT
      ``diff_pair_base_obstacles`` (which bakes in the half-pair-width the coupled
      CENTERLINE needs for its offsets); that inflation over-blocks a one-track leg.
    - ``extra_clearance=0`` for the same reason (over-blocks a leg's escape via --
      watchy USB_D).
    - no ``add_own_stubs_func`` / ``ripped_route_*``: the leg's own near-pad stub must
      not block it, and the partner net's copper is re-added per-leg inside
      ``_route_hybrid_leg``.

    Every hybrid call site (``_maybe_swap_to_hybrid``, the main-loop and reroute-loop
    last-resort hybrids, and the multipoint leg hybrid) builds the leg map through this
    one helper so it means the SAME thing regardless of entry path.
    """
    obstacles, _ = build_diff_pair_obstacles(
        base_obstacles, pcb_data, config, routed_net_ids, remaining_net_ids,
        all_unrouted_net_ids, p_net_id, n_net_id, gnd_net_id,
        track_proximity_cache, layer_map, 0.0)
    return obstacles


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
    diagonal_margin: float = 0.25,
    net_obstacles_cache: Optional[Dict[int, NetObstacleData]] = None,
    ripped_route_layer_costs: Dict[int, np.ndarray] = None,
    ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None
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
        net_obstacles_cache: Optional pre-computed net obstacles for fast batch adding
        ripped_route_layer_costs: Optional dict of ripped route layer-specific costs
        ripped_route_via_positions: Optional dict of ripped route via positions

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    obstacles = base_obstacles.clone_fresh()

    # Add previously routed nets as obstacles
    # Note: Cannot use cache for routed nets because their segments have changed
    for routed_id in routed_net_ids:
        add_net_stubs_as_obstacles(obstacles, pcb_data, routed_id, config)
        add_net_vias_as_obstacles(obstacles, pcb_data, routed_id, config, diagonal_margin=diagonal_margin)
        add_net_pads_as_obstacles(obstacles, pcb_data, routed_id, config)

    # Add GND vias as obstacles
    if gnd_net_id is not None:
        add_net_vias_as_obstacles(obstacles, pcb_data, gnd_net_id, config, diagonal_margin=diagonal_margin)

    # Add other unrouted nets as obstacles (can use cache since they haven't changed)
    other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]
    for other_net_id in other_unrouted:
        if net_obstacles_cache and other_net_id in net_obstacles_cache:
            add_net_obstacles_from_cache(obstacles, net_obstacles_cache[other_net_id])
        else:
            add_net_stubs_as_obstacles(obstacles, pcb_data, other_net_id, config)
            add_net_vias_as_obstacles(obstacles, pcb_data, other_net_id, config, diagonal_margin=diagonal_margin)
            add_net_pads_as_obstacles(obstacles, pcb_data, other_net_id, config)

    # Add stub proximity costs (includes chip pads as pseudo-stubs)
    stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                               if nid != net_id and nid not in routed_net_ids]
    # #658 river: same-bus siblings exert NO soft proximity on a member --
    # hard clearance stays (obstacle stamps), so members can pack to the
    # legal minimum pitch instead of being repelled from the hug zone the
    # follow-the-leader lane points into (measured: hug 0% with the
    # repulsion active -- the proximity field outbids any attraction dose).
    from global_plan import river_sibling_ids
    _sibs = river_sibling_ids(config, net_id)
    import os as _ros
    if _ros.environ.get('KICAD_RIVER_PROX_DEBUG') and _sibs:
        _in_cache = sum(1 for k in track_proximity_cache if k in _sibs)
        print(f"    [river-prox] net {net_id}: {len(_sibs)} sibs, "
              f"cache={len(track_proximity_cache)} entries "
              f"({_in_cache} sib entries filtered)")
    unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
    chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
    all_stubs = unrouted_stubs + chip_pads
    _ghost_vias = filter_ripped_ghosts(ripped_route_via_positions, config, routed_net_ids)
    _stub_surplus = apply_stub_proximity(obstacles, pcb_data, stub_proximity_net_ids,
                                         all_stubs, config,
                                         ghost_via_groups=_ghost_vias,
                                         layer_map=layer_map)

    # Add track proximity costs (+ ripped-corridor layer ghosts + layer-aware
    # stub surplus, one composition pass)
    from congestion_field import congestion2_rows
    from history_congestion import add_history_source
    from global_plan import add_plan_source
    _c2 = congestion2_rows(config, net_id, routed_net_ids)
    merge_track_proximity_costs(
        obstacles,
        ({k: v for k, v in track_proximity_cache.items() if k not in _sibs}
         if _sibs else track_proximity_cache),
        ghost_costs=add_plan_source(add_history_source(
            {**filter_ripped_ghosts(ripped_route_layer_costs, config, routed_net_ids),
             **(_stub_surplus or {}),
             **({('congestion2',): _c2} if _c2 is not None else {})}, config),
            config, net_id, routed_net_ids),
        config=config)
    # Congestion v2 (#424): demand/capacity field, owner-exempt (no-op
    # unless KICAD_CONGESTION2_COST > 0 and the field was built).


    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, net_id, config)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(obstacles, pcb_data, [net_id], config)

    return obstacles, all_stubs


def build_incremental_obstacles(
    working_obstacles,
    pcb_data,
    config,
    net_id: int,
    all_unrouted_net_ids: List[int],
    routed_net_ids: List[int],
    track_proximity_cache: Dict,
    layer_map: Dict,
    net_obstacles_cache: Dict[int, NetObstacleData]
):
    """
    Build obstacle map for single-ended routing using incremental approach.

    This is MUCH faster than build_single_ended_obstacles because it:
    1. Clones the working map (which already has all net obstacles)
    2. Only removes the current net's obstacles
    3. Adds dynamic costs (proximity, cross-layer tracks)

    Per-route cost is O(current_net_size) instead of O(all_nets_size).

    Args:
        working_obstacles: Pre-built working map with all net obstacles
        pcb_data: PCB data structure
        config: Routing configuration
        net_id: Current net ID being routed
        all_unrouted_net_ids: All unrouted net IDs for stub proximity
        routed_net_ids: List of already routed net IDs
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
        net_obstacles_cache: Pre-computed net obstacles for removal

    Returns:
        Tuple of (obstacles, unrouted_stubs)
    """
    # Clone the working map (has all net obstacles already)
    obstacles = working_obstacles.clone_fresh()

    # Remove current net's obstacles so we can route through our own stubs
    if net_id in net_obstacles_cache:
        remove_net_obstacles_from_cache(obstacles, net_obstacles_cache[net_id])

    # Add stub proximity costs (includes chip pads as pseudo-stubs)
    stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                               if nid != net_id and nid not in routed_net_ids]
    # #658 river: same-bus siblings exert NO soft proximity (hard clearance
    # stays) -- this is the FAST main-loop builder, the path that actually
    # prices the hug zone (measured: the exemption in the slow builder was
    # unreachable; only reconcile sub-runs go through it).
    from global_plan import river_sibling_ids
    _sibs = river_sibling_ids(config, net_id)
    if _sibs:
        import os as _ros
        if _ros.environ.get('KICAD_RIVER_PROX_DEBUG'):
            _hit = sum(1 for k in track_proximity_cache if k in _sibs)
            print(f"    [river-prox] net {net_id}: {len(_sibs)} sibs, "
                  f"{_hit} sib cache entr(ies) exempted")
    unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
    chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
    all_stubs = unrouted_stubs + chip_pads
    _stub_surplus = apply_stub_proximity(obstacles, pcb_data,
                                         stub_proximity_net_ids, all_stubs,
                                         config, layer_map=layer_map)

    # Add track proximity costs (+ layer-aware stub surplus, #590 history)
    from history_congestion import add_history_source
    from global_plan import add_plan_source
    merge_track_proximity_costs(
        obstacles,
        ({k: v for k, v in track_proximity_cache.items() if k not in _sibs}
         if _sibs else track_proximity_cache),
        ghost_costs=add_plan_source(
            add_history_source(_stub_surplus or None, config),
            config, net_id, routed_net_ids) or None,
        config=config)

    # Add cross-layer track data
    add_cross_layer_tracks(obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={net_id})

    # Add same-net via clearance
    add_same_net_via_clearance(obstacles, pcb_data, net_id, config)

    # Add same-net pad drill hole-to-hole clearance
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, net_id, config)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(obstacles, pcb_data, [net_id], config)

    return obstacles, all_stubs


def prepare_obstacles_inplace(
    working_obstacles,
    pcb_data,
    config,
    net_id: int,
    all_unrouted_net_ids: List[int],
    routed_net_ids: List[int],
    track_proximity_cache: Dict,
    layer_map: Dict,
    net_obstacles_cache: Dict[int, NetObstacleData],
    ripped_route_layer_costs: Dict[int, np.ndarray] = None,
    ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None
) -> Tuple[List[Tuple[float, float]], List[Tuple[int, int]]]:
    """
    Prepare working_obstacles IN-PLACE for routing a single-ended net.

    This modifies working_obstacles directly instead of cloning, saving significant memory.
    Returns data needed for restore_obstacles_inplace after routing.

    Args:
        working_obstacles: Working obstacle map (modified in place)
        pcb_data: PCB data structure
        config: Routing configuration
        net_id: Current net ID being routed
        all_unrouted_net_ids: All unrouted net IDs for stub proximity
        routed_net_ids: List of already routed net IDs
        track_proximity_cache: Cache of track proximity costs
        layer_map: Layer name to index mapping
        net_obstacles_cache: Pre-computed net obstacles
        ripped_route_layer_costs: Optional dict of ripped route layer-specific costs
        ripped_route_via_positions: Optional dict of ripped route via positions

    Returns:
        Tuple of (unrouted_stubs, same_net_via_clearance_cells) for use by restore
    """
    from routing_config import GridCoord

    # Clear per-route data from previous route
    working_obstacles.clear_stub_proximity()
    working_obstacles.clear_endpoint_exempt()   # C5: previous net's endpoint disks
    working_obstacles.clear_layer_proximity()
    working_obstacles.clear_cross_layer_tracks()
    working_obstacles.clear_free_vias()
    working_obstacles.clear_source_target_cells()  # Clear source/target overrides from previous route

    # Remove current net's obstacles so we can route through our own stubs
    if net_id in net_obstacles_cache:
        remove_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[net_id])

    # Net-tie corridor lift (footprint net_tie_pad_groups): remove exactly
    # the PARTNER copper's recorded stamp rows inside this net's corridor
    # (KiCad's IsNetTieExclusion locality -- see _compute_net_tie_corridors).
    # Only the tie copper's own contributions are removed, so blocking from
    # sibling routes and third nets stays intact; pads are never ripped and
    # the partner trunk's base stamps are only mutated by this balanced
    # remove / restore re-add pair. Via blocking is not recorded, not lifted.
    _tie_lift = getattr(pcb_data, '_net_tie_lift', None)
    if _tie_lift:
        _lifted = [a for a in _tie_lift.get(net_id, []) if len(a)]
        if _lifted:
            for _arr in _lifted:
                working_obstacles.remove_blocked_cells_batch(_arr)
            _TIE_LIFTED[(id(working_obstacles), net_id)] = _lifted
            # #667: the lifted band is legal CELL-BY-CELL but its copper
            # can be illegal as a SEGMENT (KiCad's IsNetTieExclusion
            # waives a (track, partner-pad) pair only when the contact
            # lies on the OWN pad -- cynthion shipped 3 router-introduced
            # tie violations per pass through this exact band). Price the
            # band (radius 1: the cells themselves, no halo) so the A*
            # prefers the clean own-axis approach the human uses; the
            # band stays available as a last resort, so connectivity is
            # never lost (the reject-gate attempt stranded 3 sense nets).
            # clear_stub_proximity() at prepare/finish brackets this like
            # every other proximity cost -- no cross-net leak.
            _tie_cost = env_knobs.TIE_BAND_COST
            if _tie_cost > 0:
                # Differential pricing: the OWN-PAD approach (KiCad-waived
                # contact) stays free; only the off-pad corridor cells are
                # priced. Uniform pricing over the whole lifted band was
                # measured INERT on cynthion (no gradient = no steering).
                _price667 = (getattr(pcb_data, '_net_tie_price', None)
                             or {}).get(net_id)
                if _price667 is None:
                    _pos667 = np.unique(np.concatenate(
                        [np.asarray(_a)[:, :2] for _a in _lifted]), axis=0)
                    _price667 = [(int(gx), int(gy)) for gx, gy in _pos667]
                if _price667:
                    working_obstacles.add_stub_proximity_costs_batch(
                        [(int(gx), int(gy)) for gx, gy in _price667], 1,
                        config.cell_cost(_tie_cost))

    # Add stub proximity costs (includes chip pads as pseudo-stubs)
    stub_proximity_net_ids = [nid for nid in all_unrouted_net_ids
                               if nid != net_id and nid not in routed_net_ids]
    # #658 river: same-bus siblings exert NO soft proximity on a member
    # (hard clearance stays). THIS is the hot in-place path the main loop
    # actually uses -- the slow/incremental builders only serve fallbacks
    # and reconcile sub-runs.
    from global_plan import river_sibling_ids
    _sibs = river_sibling_ids(config, net_id)
    if _sibs:
        import os as _ros
        if _ros.environ.get('KICAD_RIVER_PROX_DEBUG'):
            _hit = sum(1 for k in track_proximity_cache if k in _sibs)
            print(f"    [river-prox] net {net_id}: {len(_sibs)} sibs, "
                  f"{_hit} sib cache entr(ies) exempted")
    unrouted_stubs = get_stub_endpoints(pcb_data, stub_proximity_net_ids)
    chip_pads = get_chip_pad_positions(pcb_data, stub_proximity_net_ids)
    all_stubs = unrouted_stubs + chip_pads
    _ghost_vias = filter_ripped_ghosts(ripped_route_via_positions, config, routed_net_ids)
    _stub_surplus = apply_stub_proximity(working_obstacles, pcb_data,
                                         stub_proximity_net_ids, all_stubs,
                                         config, ghost_via_groups=_ghost_vias,
                                         layer_map=layer_map)

    # Add track proximity costs (+ ripped-corridor layer ghosts + layer-aware
    # stub surplus, one composition pass)
    from congestion_field import congestion2_rows
    from history_congestion import add_history_source
    from global_plan import add_plan_source
    _c2 = congestion2_rows(config, net_id, routed_net_ids)
    merge_track_proximity_costs(
        working_obstacles,
        ({k: v for k, v in track_proximity_cache.items() if k not in _sibs}
         if _sibs else track_proximity_cache),
        ghost_costs=add_plan_source(add_history_source(
            {**filter_ripped_ghosts(ripped_route_layer_costs, config, routed_net_ids),
             **(_stub_surplus or {}),
             **({('congestion2',): _c2} if _c2 is not None else {})}, config),
            config, net_id, routed_net_ids),
        config=config)


    # Add cross-layer track data
    add_cross_layer_tracks(working_obstacles, pcb_data, config, layer_map,
                           exclude_net_ids={net_id})

    # Add same-net via clearance and track which cells were added
    coord = GridCoord(config.grid_step)
    same_net_via_cells = []

    # Via-via clearance
    via_via_expansion_grid = max(1.0, (config.via_size + config.clearance) * coord.inv_step)
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        # Grow by the via's sub-grid offset so an off-grid via-in-pad keeps a NEW
        # same-net via the full clearance from its TRUE centre (issue #70; mirror of
        # add_same_net_via_clearance and the via-obstacle rasterizers).
        off_cells = math.hypot(via.x - gx * coord.grid_step,
                               via.y - gy * coord.grid_step) / coord.grid_step
        radius = via_via_expansion_grid + off_cells
        rng = int(math.ceil(radius))
        radius_sq = radius * radius
        # Sweep item 3 (#625): integer-mask disc, identical cell set (the
        # threshold scalar is unchanged; runs per net per prepare).
        _ax = np.arange(-rng, rng + 1, dtype=np.int64)
        _EX, _EY = np.meshgrid(_ax, _ax, indexing='ij')
        _m = _EX * _EX + _EY * _EY <= radius_sq
        same_net_via_cells.extend(
            zip((_EX[_m] + gx).tolist(), (_EY[_m] + gy).tolist()))

    # Pad drill hole clearance
    # Skip the pad center - the router can use existing through-holes for layer transitions
    if config.hole_to_hole_clearance > 0:
        for pad in pcb_data.pads_by_net.get(net_id, []):
            if pad.drill and pad.drill > 0:
                # Include pad drill radius in clearance calculation. Float radius +
                # ceil bound (not the flooring to_grid_dist) so this circular hole-to-
                # hole keep-out reserves the full clearance instead of ~1 cell short
                # (same grid-quantization fix as the via-via keep-out above / #154).
                required_dist = pad.drill / 2 + config.via_drill / 2 + config.hole_to_hole_clearance
                radius = required_dist * coord.inv_step
                expand = int(math.ceil(radius))
                radius_sq = radius * radius
                gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                # Sweep item 3 (#625): integer-mask disc; the pad centre stays
                # landable (layer transitions at through-holes), identical set.
                _ax = np.arange(-expand, expand + 1, dtype=np.int64)
                _EX, _EY = np.meshgrid(_ax, _ax, indexing='ij')
                _m = (_EX * _EX + _EY * _EY <= radius_sq) & ~((_EX == 0) & (_EY == 0))
                same_net_via_cells.extend(
                    zip((_EX[_m] + gx).tolist(), (_EY[_m] + gy).tolist()))

    # #581: same-net pad via keep-out. When the board carries an active
    # same_net_pad_clearance (flag / persisted .kicad_pro record), the CURRENT
    # net's own SMD pads block via placement at pad-edge + that clearance --
    # so escape vias land off-pad. Rides same_net_via_cells so the restore
    # stays balanced. Track blocking is untouched -- the route may still REACH
    # its pad; only a via may not land on/near it.
    from obstacle_map import same_net_pad_via_keepout_cells
    _pad_cells_581 = same_net_pad_via_keepout_cells(pcb_data, net_id, config)
    if len(_pad_cells_581):
        same_net_via_cells.extend(map(tuple, _pad_cells_581.tolist()))

    # Batch add same-net via clearance (convert to numpy for Rust FFI)
    if same_net_via_cells:
        same_net_via_arr = np.array(same_net_via_cells, dtype=np.int32)
        working_obstacles.add_blocked_vias_batch(same_net_via_arr)
        # #568 MIRROR: these are same-net via-via and pad-drill hole-to-hole
        # keep-outs (#70/#154). A rung-1 search consults ONLY the small map
        # for dynamic copper, so without this mirror it could drop a small
        # via inside its own net's h2h ring -- and hole-to-hole is
        # net-agnostic at the fab, so that is a REAL drill violation, not a
        # bookkeeping leak. The emission backstop cannot catch it either: it
        # checks foreign copper only. Mirrored at FULL size (conservative).
        try:
            from obstacle_map import _rung_small_armed
            if _rung_small_armed():
                working_obstacles.add_blocked_vias_small_batch(same_net_via_arr)
        except (AttributeError, ImportError):
            pass
    else:
        same_net_via_arr = np.empty((0, 2), dtype=np.int32)

    # Add through-hole pads as free via positions (zero-cost layer change)
    _add_free_via_positions(working_obstacles, pcb_data, [net_id], config)

    return all_stubs, same_net_via_arr


# Net-tie corridor stamps lifted by prepare_obstacles_inplace, re-added by
# restore_obstacles_inplace. Keyed by (map id, net id): prepare/restore are
# strictly paired per net route on one thread, so entries live only across
# that window; keying by map id keeps cloned maps independent.
_TIE_LIFTED: Dict[tuple, list] = {}


def restore_obstacles_inplace(
    working_obstacles,
    net_id: int,
    net_obstacles_cache: Dict[int, NetObstacleData],
    same_net_via_cells: np.ndarray
):
    """
    Restore working_obstacles after routing attempt.

    This clears per-route data and restores the current net's obstacles.
    Should be called after prepare_obstacles_inplace, whether routing succeeded or failed.

    Args:
        working_obstacles: Working obstacle map (modified in place)
        net_id: Net ID that was routed
        net_obstacles_cache: Pre-computed net obstacles (for restoring)
        same_net_via_cells: Numpy array of cells added for same-net via clearance (to remove)
    """
    # Clear per-route data
    working_obstacles.clear_stub_proximity()
    working_obstacles.clear_endpoint_exempt()   # C5 hygiene: no stale disks for Phase-3 clones
    working_obstacles.clear_layer_proximity()
    working_obstacles.clear_cross_layer_tracks()
    working_obstacles.clear_free_vias()

    # Remove same-net via clearance cells
    if len(same_net_via_cells) > 0:
        working_obstacles.remove_blocked_vias_batch(same_net_via_cells)
        try:    # #568: mirror of the add-side small stamp (refcount balance)
            from obstacle_map import _rung_small_armed
            if _rung_small_armed():
                working_obstacles.remove_blocked_vias_small_batch(
                    same_net_via_cells)
        except (AttributeError, ImportError):
            pass

    # Re-add the net-tie corridor stamps lifted by prepare (see there).
    _lifted = _TIE_LIFTED.pop((id(working_obstacles), net_id), None)
    if _lifted:
        for _arr in _lifted:
            working_obstacles.add_blocked_cells_batch(_arr)

    # Restore current net's obstacles (from cache - original stubs)
    # Note: If routing succeeded, caller should update cache first with new route data
    if net_id in net_obstacles_cache:
        add_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[net_id])


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
    layer_map: Dict,
    working_obstacles=None,
    net_obstacles_cache: Dict = None,
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
        working_obstacles / net_obstacles_cache: the run's persistent map and
            per-net cache (#806). When given, both members' entries are
            refreshed from the committed copper, the same contract
            record_single_ended_success's callers keep. None = no map.
    """
    add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
    from obstacle_cache import refresh_net_obstacles
    refresh_net_obstacles(working_obstacles, net_obstacles_cache, pcb_data,
                          config, (pair.p_net_id, pair.n_net_id))

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

    # #466: refresh the dynamic fragility field (no-op unless armed)
    from plane_fragility import fragility_on_copper_change
    fragility_on_copper_change(config, pcb_data,
                               result.get('new_segments'),
                               result.get('new_vias'))


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

    # #466: the committed copper may have narrowed a pour -- refresh the
    # dynamic fragility field's dirty window (no-op unless armed).
    from plane_fragility import fragility_on_copper_change
    fragility_on_copper_change(config, pcb_data,
                               result.get('new_segments'),
                               result.get('new_vias'))


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

    # #329 audit: this restore was blind, unlike restore_net (#134). Every
    # caller restores IMMEDIATELY after its own failed attempt (which commits
    # no copper), so a collision "cannot happen" -- but a graze/partial result
    # that ever starts committing copper first would turn the verbatim re-add
    # into a different-net short. Cheap belt-and-braces: skip the copper
    # re-add if it would collide, and leave the net for the reroute queue
    # (unrouted beats shorted); the bookkeeping below still runs so the net
    # is tracked either way.
    from rip_up_reroute import _saved_route_collides
    if _saved_route_collides(ripped_saved, pcb_data, list(ripped_ids), config.clearance):
        names = [pcb_data.nets[r].name if r in pcb_data.nets else str(r) for r in ripped_ids]
        print(f"    restore of {'/'.join(names)} would collide with copper routed "
              f"meanwhile -- leaving unrouted for reroute (#134 guard)")
        for rid in ripped_ids:
            if rid in routed_net_ids:
                routed_net_ids.remove(rid)
            if rid not in remaining_net_ids:
                remaining_net_ids.append(rid)
            routed_results.pop(rid, None)
        if ripped_saved in results:
            results.remove(ripped_saved)
        return

    add_route_to_pcb_data(pcb_data, ripped_saved, debug_lines=config.debug_lines)
    # #466: restored copper re-narrows the pour it had freed
    from plane_fragility import fragility_on_copper_change
    fragility_on_copper_change(config, pcb_data,
                               ripped_saved.get('new_segments'),
                               ripped_saved.get('new_vias'))

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
