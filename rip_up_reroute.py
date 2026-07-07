"""
Rip-up and reroute functions for the PCB router.

This module handles removing routed nets from the PCB data and tracking structures,
as well as restoring them when needed (e.g., when a rip-up retry fails).
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

from kicad_parser import PCBData
from routing_config import GridRouteConfig, DiffPairNet
from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from obstacle_costs import compute_track_proximity_for_net, compute_ripped_route_costs
from obstacle_cache import (
    precompute_net_obstacles, add_net_obstacles_from_cache, remove_net_obstacles_from_cache
)

if TYPE_CHECKING:
    import numpy as np
    from grid_router import GridObstacleMap
    from obstacle_cache import NetObstacleData


def rip_up_net(net_id: int, pcb_data: PCBData, routed_net_ids: List[int],
               routed_net_paths: Dict[int, List], routed_results: Dict[int, dict],
               diff_pair_by_net_id: Dict[int, Tuple[str, DiffPairNet]],
               remaining_net_ids: List[int], results: List[dict],
               config: GridRouteConfig,
               track_proximity_cache: Dict[int, dict] = None,
               working_obstacles: 'GridObstacleMap' = None,
               net_obstacles_cache: Dict[int, 'NetObstacleData'] = None,
               ripped_route_layer_costs: Dict[int, 'np.ndarray'] = None,
               ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None,
               layer_map: Dict[str, int] = None) -> Tuple[Optional[dict], List[int], bool]:
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
        ripped_route_layer_costs: Optional dict to store ripped route layer-specific costs
        ripped_route_via_positions: Optional dict to store ripped route via positions
        layer_map: Optional layer name to index mapping (required for ripped route costs)

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

    # Compute and store ripped route avoidance costs if enabled
    if config.ripped_route_avoidance_cost > 0 and ripped_route_layer_costs is not None and layer_map is not None:
        layer_costs, via_positions = compute_ripped_route_costs(saved_result, config, layer_map)
        for rid in ripped_net_ids:
            ripped_route_layer_costs[rid] = layer_costs
            if ripped_route_via_positions is not None:
                ripped_route_via_positions[rid] = via_positions

    return saved_result, ripped_net_ids, was_in_results


def _pt_seg_dist_sq(px: float, py: float,
                    ax: float, ay: float, bx: float, by: float) -> float:
    """Squared distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def _segs_cross(ax0, ay0, ax1, ay1, bx0, by0, bx1, by1) -> bool:
    """True if the two segments properly intersect (centerlines cross)."""
    def ccw(ox, oy, px, py, qx, qy):
        return (qy - oy) * (px - ox) - (qx - ox) * (py - oy)
    d1 = ccw(bx0, by0, bx1, by1, ax0, ay0)
    d2 = ccw(bx0, by0, bx1, by1, ax1, ay1)
    d3 = ccw(ax0, ay0, ax1, ay1, bx0, by0)
    d4 = ccw(ax0, ay0, ax1, ay1, bx1, by1)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _seg_seg_dist_sq(ax0, ay0, ax1, ay1, bx0, by0, bx1, by1) -> float:
    """Minimum squared distance between two segments (0 if they cross)."""
    if _segs_cross(ax0, ay0, ax1, ay1, bx0, by0, bx1, by1):
        return 0.0
    return min(
        _pt_seg_dist_sq(ax0, ay0, bx0, by0, bx1, by1),
        _pt_seg_dist_sq(ax1, ay1, bx0, by0, bx1, by1),
        _pt_seg_dist_sq(bx0, by0, ax0, ay0, ax1, ay1),
        _pt_seg_dist_sq(bx1, by1, ax0, ay0, ax1, ay1),
    )


def _saved_route_collides(saved_result: dict, pcb_data: PCBData,
                          own_net_ids: List[int], clearance: float) -> bool:
    """Issue #134: return True if restoring saved_result's copper verbatim would
    violate clearance against another net's copper currently in pcb_data.

    A rolled-back net's saved geometry is stale: while it was ripped, another
    net may have been (re)routed through the corridor it used to occupy.
    Re-adding it verbatim then ships a different-net short (the IPS/RESETn
    collinear F.Cu overlap and the RESETn-via-on-EPHY_RX_P inner trace on
    ottercast both arose this way). Restoring exactly where it was only
    collides with copper that MOVED into its space while it was ripped -
    precisely the desync we want to refuse. Mirrors the plane fix (#88.1).
    """
    own = set(own_net_ids)
    segs = saved_result.get('new_segments', []) or []
    vias = saved_result.get('new_vias', []) or []
    if not segs and not vias:
        return False

    # Bounding box of the saved route, expanded, to prefilter pcb_data copper.
    xs, ys = [], []
    for s in segs:
        xs.extend((s.start_x, s.end_x))
        ys.extend((s.start_y, s.end_y))
    for v in vias:
        xs.append(v.x)
        ys.append(v.y)
    margin = 1.0  # widths + clearance are well under 1mm
    minx, maxx = min(xs) - margin, max(xs) + margin
    miny, maxy = min(ys) - margin, max(ys) + margin

    def _in_box(lo_x, hi_x, lo_y, hi_y):
        return not (hi_x < minx or lo_x > maxx or hi_y < miny or lo_y > maxy)

    o_segs = [s for s in pcb_data.segments
              if s.net_id not in own and s.net_id != 0
              and _in_box(min(s.start_x, s.end_x), max(s.start_x, s.end_x),
                          min(s.start_y, s.end_y), max(s.start_y, s.end_y))]
    o_vias = [v for v in pcb_data.vias
              if v.net_id not in own and v.net_id != 0
              and minx <= v.x <= maxx and miny <= v.y <= maxy]

    # Restored segments vs other-net segments (same layer) and vias (all layers).
    for s in segs:
        hw = s.width / 2.0
        for o in o_segs:
            if o.layer != s.layer:
                continue
            thr = hw + o.width / 2.0 + clearance
            if _seg_seg_dist_sq(s.start_x, s.start_y, s.end_x, s.end_y,
                                o.start_x, o.start_y, o.end_x, o.end_y) < thr * thr:
                return True
        for v in o_vias:
            thr = hw + v.size / 2.0 + clearance
            if _pt_seg_dist_sq(v.x, v.y, s.start_x, s.start_y,
                               s.end_x, s.end_y) < thr * thr:
                return True

    # Restored vias (span layers) vs other-net vias and segments.
    for vv in vias:
        vr = vv.size / 2.0
        for v in o_vias:
            thr = vr + v.size / 2.0 + clearance
            if (vv.x - v.x) ** 2 + (vv.y - v.y) ** 2 < thr * thr:
                return True
        for o in o_segs:
            thr = vr + o.width / 2.0 + clearance
            if _pt_seg_dist_sq(vv.x, vv.y, o.start_x, o.start_y,
                               o.end_x, o.end_y) < thr * thr:
                return True

    return False


def restore_net(net_id: int, saved_result: dict, ripped_net_ids: List[int],
                was_in_results: bool, pcb_data: PCBData, routed_net_ids: List[int],
                routed_net_paths: Dict[int, List], routed_results: Dict[int, dict],
                diff_pair_by_net_id: Dict[int, Tuple[str, DiffPairNet]],
                remaining_net_ids: List[int], results: List[dict],
                config: GridRouteConfig,
                track_proximity_cache: Dict[int, dict] = None,
                layer_map: Dict[str, int] = None,
                working_obstacles: 'GridObstacleMap' = None,
                net_obstacles_cache: Dict[int, 'NetObstacleData'] = None,
                ripped_route_layer_costs: Dict[int, 'np.ndarray'] = None,
                ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = None,
                refused_sink: Optional[set] = None):
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
        ripped_route_layer_costs: Optional dict to clear ripped route layer-specific costs
        ripped_route_via_positions: Optional dict to clear ripped route via positions
        refused_sink: Optional set; net IDs of a collision-refused restore are
            added here so the caller can re-route them cleanly later (#134)
    """
    if saved_result is None:
        return

    # Issue #134: collision-aware restoration. If re-adding this net's stale
    # saved copper would short other-net copper that moved into its corridor
    # while it was ripped, refuse: leave the net ripped (rip_up_net already put
    # it in remaining_net_ids with stubs-only obstacles) so it is reported
    # unrouted rather than shorted. The net IDs are recorded in refused_sink so
    # the caller gives them a clean reroute pass afterward (no completion loss).
    if _saved_route_collides(saved_result, pcb_data, ripped_net_ids, config.clearance):
        net_label = '/'.join(str(r) for r in ripped_net_ids) or str(net_id)
        print(f"      restore skipped (net {net_label}): saved copper would short "
              f"other-net copper; left ripped (#134)")
        if refused_sink is not None:
            refused_sink.update(ripped_net_ids)
        # Keep the refused route for the #134 recovery's LAST resort: if the
        # clean reroute pass also fails, a piece-level restore of the
        # non-colliding copper beats shipping the net at zero (parity with the
        # plane tools' settle, 72ca5f9).
        stash = getattr(pcb_data, '_refused_saved_134', None)
        if stash is None:
            stash = {}
            pcb_data._refused_saved_134 = stash
        for rid in ripped_net_ids:
            stash[rid] = saved_result
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

    # Clear ripped route avoidance costs since net is restored
    if ripped_route_layer_costs is not None:
        for rid in ripped_net_ids:
            ripped_route_layer_costs.pop(rid, None)
    if ripped_route_via_positions is not None:
        for rid in ripped_net_ids:
            ripped_route_via_positions.pop(rid, None)
