"""
Rip-up and reroute functions for the PCB router.

This module handles removing routed nets from the PCB data and tracking structures,
as well as restoring them when needed (e.g., when a rip-up retry fails).
"""
from __future__ import annotations

import os

from typing import Dict, List, Optional, Tuple, TYPE_CHECKING

from kicad_parser import PCBData
# E3: the one guarded squared-distance kernel (length_sq < 1e-10 degenerate guard).
from geometry_utils import point_to_segment_dist_sq as _pt_seg_dist_sq
from routing_config import GridRouteConfig, DiffPairNet
from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from obstacle_costs import compute_track_proximity_for_net, compute_ripped_route_costs
from obstacle_cache import (
    precompute_net_obstacles, add_net_obstacles_from_cache, remove_net_obstacles_from_cache
)

# Debug knob, read ONCE: no per-rip environ lookup when it is off.
_RIP_CALLER_DEBUG = os.environ.get('KICAD_RIP_CALLER') == '1'

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
               layer_map: Dict[str, int] = None,
               only_segments: Optional[List] = None,
               history_conflict: bool = True) -> Tuple[Optional[dict], List[int], bool]:
    """Rip up a routed net (or diff pair), removing it from pcb_data and tracking structures.

    #510 PARTIAL (leg-level) RIP: pass `only_segments` to remove just those
    segments -- the branch that actually blocks the route -- instead of the whole
    net. `only_segments=None` (the default) is the historic whole-net behaviour,
    bit-for-bit, so this is a strict superset and the default path is unchanged.

    A partial rip deliberately does NOT drop the net's surviving copper: it stays
    on the board and in the write-list, and the net is re-queued so the
    multipoint router reconnects the orphaned component (route_multipoint_main
    derives pad_components from existing copper, so it routes only the missing
    edge). The saved record covers exactly the removed segments, so a restore
    puts back that branch and nothing else.

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
        history_conflict: #590 -- True (default) if this rip is CONTENTION
            (another net wanted the ground). Pass False for an own-tree
            re-ask (#444 seam dissolution), which rips a net's own copper to
            re-ask it: no second net is competing, so charging the negotiated-
            congestion field there would price a cell nobody contested.

    Returns:
        tuple: (saved_result, ripped_net_ids, was_in_results) for later restoration
               saved_result: the result dict that was removed
               ripped_net_ids: list of net IDs that were ripped (1 for single, 2 for diff pair)
               was_in_results: True if the result was in the results list
    """
    if net_id not in routed_results:
        return None, [], False
    if _RIP_CALLER_DEBUG:
        # KICAD_RIP_CALLER=1: attribute each rip to its CALLING FUNCTION, split
        # into the two kinds that behave completely differently:
        #   BLOCKER  -- "net X is in my way, tear it out" (what #510 targets)
        #   own-tree -- seam_reask_one_net re-asking a net's OWN tree wholesale
        #               (#444); whole-net is correct there by design.
        # Traces record `rip` events with net/net_name/del_s but NOT the call
        # site, so every rip looks alike from the trace. That cost real time on
        # #510: lora_v3 shows 12 rips and reads like an ideal test board, but 11
        # are own-tree re-asks and only ONE is a blocker rip. Anything reasoning
        # about "how often does the router rip" from traces alone repeats it.
        import traceback as _tb
        _fr = _tb.extract_stack()[-2]
        _kind = 'own-tree' if _fr.name == 'seam_reask_one_net' else 'BLOCKER'
        print(f"      [rip-caller] {_kind} {_fr.filename.split('/')[-1]}::{_fr.name}",
              flush=True)

    saved_result = routed_results[net_id]

    # Main-pass pre-existing rip (0805): this victim's copper was committed by
    # an EARLIER run/step (is_existing_route registration, #103 / auto
    # candidacy). Disclose the rip loudly and register it on pcb_data so
    # route.py can (a) pull the net into the #220 stale-strip scope -- a
    # rerouted victim's original input copper must not ship next to its
    # reroute -- and (b) print the per-net outcome at end of run.
    def _note_preexisting_rip(partial_branch: bool) -> None:
        _net_pe = pcb_data.nets.get(net_id)
        _nm_pe = _net_pe.name if _net_pe and _net_pe.name else f"Net {net_id}"
        _reg_pe = getattr(pcb_data, '_preexisting_rips', None)
        if _reg_pe is None:
            _reg_pe = {}
            pcb_data._preexisting_rips = _reg_pe
        _reg_pe[net_id] = _nm_pe
        print(f"      PRE-EXISTING rip: '{_nm_pe}' (earlier-step copper"
              f"{', blocking branch only' if partial_branch else ''}) removed "
              f"-- queued for reroute this run; original restored if the "
              f"reroute fails")
    _preexist_rip = bool(saved_result.get('is_existing_route'))

    # ---- #510 partial (leg-level) rip -------------------------------------
    # Remove ONLY the blocking branch. The net keeps its other legs, stays in the
    # write-list, and is re-queued so the multipoint router reconnects the
    # orphaned component. Returns early: none of the whole-net teardown below
    # (dropping routed_results / paths / caches for the entire net) applies.
    if only_segments:
        _keep_ids = {id(s) for s in only_segments}
        _live = [s for s in pcb_data.segments
                 if s.net_id == net_id and id(s) in _keep_ids]
        if not _live:
            return None, [], False
        # Vias that ONLY this branch uses (an endpoint of a removed segment and
        # of nothing that survives) go with it; a via still serving surviving
        # copper must stay, or the rest of the net is severed at the layer change.
        _rm_pts = {(round(s.start_x, 4), round(s.start_y, 4)) for s in _live} | \
                  {(round(s.end_x, 4), round(s.end_y, 4)) for s in _live}
        _survivor_pts = set()
        for s in pcb_data.segments:
            if s.net_id != net_id or id(s) in _keep_ids:
                continue
            _survivor_pts.add((round(s.start_x, 4), round(s.start_y, 4)))
            _survivor_pts.add((round(s.end_x, 4), round(s.end_y, 4)))
        _live_vias = [v for v in pcb_data.vias
                      if v.net_id == net_id
                      and (round(v.x, 4), round(v.y, 4)) in _rm_pts
                      and (round(v.x, 4), round(v.y, 4)) not in _survivor_pts]

        partial = {'new_segments': _live, 'new_vias': _live_vias,
                   'partial_leg_rip': True, 'net_id': net_id}
        remove_route_from_pcb_data(pcb_data, partial)
        # #466: freed copper may have re-widened a pour strait -- refresh the
        # dynamic fragility field's window (no-op unless armed).
        from plane_fragility import fragility_on_copper_change
        fragility_on_copper_change(config, pcb_data, _live, _live_vias)
        # Keep the owning result in the write-list but shrink it to what remains,
        # or the removed copper ships anyway (#369 A2 / #508 write-list class).
        for _key, _removed in (('new_segments', _keep_ids),
                               ('new_vias', {id(v) for v in _live_vias})):
            if isinstance(saved_result.get(_key), list):
                saved_result[_key] = [o for o in saved_result[_key]
                                      if id(o) not in _removed]
        for _leg in (saved_result.get('leg_results') or []):
            for _key, _removed in (('new_segments', _keep_ids),
                                   ('new_vias', {id(v) for v in _live_vias})):
                if isinstance(_leg.get(_key), list):
                    _leg[_key] = [o for o in _leg[_key] if id(o) not in _removed]
        # Re-queue the net: its surviving copper makes it a partially-routed net,
        # which route_multipoint_main handles natively via pad_components.
        if net_id in routed_net_ids:
            routed_net_ids.remove(net_id)
        routed_net_paths.pop(net_id, None)
        routed_results.pop(net_id, None)
        if track_proximity_cache is not None:
            track_proximity_cache.pop(net_id, None)
        if net_id not in remaining_net_ids:
            remaining_net_ids.append(net_id)
        # Obstacle map: remove the STALE whole-net entry, then RECOMPUTE from
        # what is still on the board and add it back -- the same remove/
        # recompute/add cycle the whole-net path uses. Dropping the entry
        # instead would leave the SURVIVING copper unrepresented in the working
        # map (under-blocking -> the router lays a foreign track over live
        # copper -> a short), and would also break the #309 ref-count balance
        # invariant `working == base + sum(caches)` that
        # tests/test_obstacle_map_balance.py asserts under rip churn.
        if working_obstacles is not None and net_obstacles_cache is not None:
            if net_id in net_obstacles_cache:
                remove_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[net_id])
            net_obstacles_cache[net_id] = precompute_net_obstacles(pcb_data, net_id, config)
            add_net_obstacles_from_cache(working_obstacles, net_obstacles_cache[net_id])
        # Ripped-route avoidance: steer the retry away from the branch we freed,
        # exactly as the whole-net path does for the copper it removed.
        if config.ripped_route_avoidance_cost > 0 and ripped_route_layer_costs is not None \
                and layer_map is not None:
            _lc, _vp = compute_ripped_route_costs(partial, config, layer_map)
            ripped_route_layer_costs[net_id] = _lc
            if ripped_route_via_positions is not None:
                ripped_route_via_positions[net_id] = _vp
        # #590: the branch we just tore out was contested ground -- bump its
        # cells' PERMANENT history cost (no-op unless KICAD_HISTORY_COST > 0).
        # Unlike the ghost above this survives the victim's reroute.
        if history_conflict:
            from history_congestion import record_rip as _record_rip590
            _record_rip590(config, partial, layer_map)
        partial['_owner_result'] = saved_result
        if _preexist_rip:
            _note_preexisting_rip(True)
        return partial, [net_id], False

    ripped_net_ids = []
    # #369 A2: a multi-leg multipoint diff pair registers a MERGED dict in
    # routed_results while the write-list carries its per-LEG dicts -- the
    # old value-equality test (`saved_result in results`) never matched, so
    # the legs stayed in the write-list after the rip and their copper
    # shipped alongside the reroute's. Match by IDENTITY over the merged
    # dict AND its legs (identity also stops equality from removing a
    # different net's identical-valued dict).
    _members = list(saved_result.get('leg_results') or []) + [saved_result]
    _member_ids = {id(r) for r in _members}
    was_in_results = any(id(r) in _member_ids for r in results)

    # Remove from pcb_data
    remove_route_from_pcb_data(pcb_data, saved_result)
    # #466: refresh the dynamic fragility field over the freed copper
    from plane_fragility import fragility_on_copper_change
    fragility_on_copper_change(config, pcb_data,
                               saved_result.get('new_segments'),
                               saved_result.get('new_vias'))

    # Remove from results list if present
    if was_in_results:
        results[:] = [r for r in results if id(r) not in _member_ids]

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

    # #590 history congestion: one conflict event per rip, charged to the
    # cells the ripped copper occupied. Independent of the ghosts above --
    # those are per-NET and vanish when the victim reroutes (C1 filter); this
    # is per-CELL and cumulative for the rest of the call. No-op unless
    # KICAD_HISTORY_COST > 0.
    if history_conflict:
        from history_congestion import record_rip as _record_rip590
        _record_rip590(config, saved_result, layer_map)

    # #468: keep the pre-rip payload reachable for the terminal-failure
    # restore (rip_restore.try_terminal_restore). Keyed under EVERY ripped
    # id so pair members resolve too; popped by restore_net.
    _reg = getattr(pcb_data, '_rip_saved', None)
    if _reg is None:
        _reg = {}
        pcb_data._rip_saved = _reg
    for _rid in ripped_net_ids:
        _reg[_rid] = (saved_result, ripped_net_ids, was_in_results)

    if _preexist_rip:
        _note_preexisting_rip(False)
    return saved_result, ripped_net_ids, was_in_results


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
    return bool(_saved_route_colliders(saved_result, pcb_data, own_net_ids,
                                       clearance, first_only=True))


def _saved_route_colliders(saved_result: dict, pcb_data: PCBData,
                           own_net_ids: List[int], clearance: float,
                           first_only: bool = False) -> list:
    """The items behind a _saved_route_collides verdict (#517 instrumentation):
    every foreign pcb_data segment/via within clearance of the saved copper,
    as ('segment'|'via', obj) pairs, deduplicated. Same geometry as the
    boolean test; first_only preserves its early-exit for the hot path.
    """
    own = set(own_net_ids)
    segs = saved_result.get('new_segments', []) or []
    vias = saved_result.get('new_vias', []) or []
    if not segs and not vias:
        return []

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

    hits = []
    seen = set()

    def _hit(kind, obj):
        if id(obj) not in seen:
            seen.add(id(obj))
            hits.append((kind, obj))

    # Restored segments vs other-net segments (same layer) and vias (all layers).
    for s in segs:
        hw = s.width / 2.0
        for o in o_segs:
            if o.layer != s.layer:
                continue
            thr = hw + o.width / 2.0 + clearance
            if _seg_seg_dist_sq(s.start_x, s.start_y, s.end_x, s.end_y,
                                o.start_x, o.start_y, o.end_x, o.end_y) < thr * thr:
                _hit('segment', o)
                if first_only:
                    return hits
        for v in o_vias:
            thr = hw + v.size / 2.0 + clearance
            if _pt_seg_dist_sq(v.x, v.y, s.start_x, s.start_y,
                               s.end_x, s.end_y) < thr * thr:
                _hit('via', v)
                if first_only:
                    return hits

    # Restored vias (span layers) vs other-net vias and segments.
    for vv in vias:
        vr = vv.size / 2.0
        for v in o_vias:
            thr = vr + v.size / 2.0 + clearance
            if (vv.x - v.x) ** 2 + (vv.y - v.y) ** 2 < thr * thr:
                _hit('via', v)
                if first_only:
                    return hits
        for o in o_segs:
            thr = vr + o.width / 2.0 + clearance
            if _pt_seg_dist_sq(vv.x, vv.y, o.start_x, o.start_y,
                               o.end_x, o.end_y) < thr * thr:
                _hit('segment', o)
                if first_only:
                    return hits

    return hits


def partition_force_restores(force_ripped, pcb_data: PCBData,
                             clearance: float, skip_net_ids=None):
    """Split --force-reroute's saved copper into what may be restored and what
    may not, and APPLY the restores to pcb_data in order.

    Returns ``(restored_ids, refused_ids)``.

    `--force-reroute` strips every named net up front, so the nets it ripped
    contend for one corridor: whichever routes first may legitimately occupy
    space a later one used to hold. Re-adding a failed net's saved copper
    verbatim then ships a DIFFERENT-NET short -- the exact stale-geometry case
    #134 refuses on the rip/reroute path, which this path did not check.
    Measured on ecp5_mini: the step rips /PE26+ and /PE26-, /PE26+ routes a via
    into the vacated corridor at (130.20, 85.10), /PE26- fails, and its original
    via returns at (130.30, 85.10) -- via-via and via-drill in CONTACT plus
    three via-segment grazes, 5 DRC on a board that was otherwise clean.

    The intent of the restore is kept: a failed replan must not be paid for by
    deleting a working route, so every net whose saved copper does NOT collide
    is still restored. Only the colliding ones are refused, and those ship
    unrouted -- honest, and strictly better than copper that is both shorted and
    disconnected.

    Order matters and is deliberate: each restore is applied before the next is
    tested, so two refused-then-restored nets cannot be re-admitted on top of
    each other. `skip_net_ids` are nets whose replan DID land copper; they keep
    it and are not candidates here.
    """
    skip = set(skip_net_ids or ())
    restored, refused = [], []
    for net_id, (segs, vias) in force_ripped.items():
        if net_id in skip:
            continue
        saved = {'new_segments': segs, 'new_vias': vias}
        if _saved_route_collides(saved, pcb_data, [net_id], clearance):
            refused.append(net_id)
            continue
        pcb_data.segments = list(pcb_data.segments) + list(segs)
        pcb_data.vias = list(pcb_data.vias) + list(vias)
        restored.append(net_id)
    return restored, refused


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
    # #468: this payload is being consumed -- drop the registry entries.
    _reg = getattr(pcb_data, '_rip_saved', None)
    if _reg is not None:
        for _rid in ripped_net_ids:
            _reg.pop(_rid, None)
    if saved_result is None:
        return

    # ---- #510 partial (leg-level) restore ---------------------------------
    # Put back exactly the branch that was removed, and nothing else. The #134
    # custody check still applies FIRST: if something moved into the vacated
    # corridor while the branch was out, refuse and leave it ripped (the net is
    # already queued for a clean re-route) rather than ship a different-net short.
    if saved_result.get('partial_leg_rip'):
        if _saved_route_collides(saved_result, pcb_data, [net_id], config.clearance):
            print(f"      restore skipped (net {net_id}): partial-leg copper would "
                  f"short other-net copper; left ripped (#134/#510)")
            if refused_sink is not None:
                refused_sink.add(net_id)
            return
        add_route_to_pcb_data(pcb_data, saved_result, trace_event='restore')
        from plane_fragility import fragility_on_copper_change  # #466
        fragility_on_copper_change(config, pcb_data,
                                   saved_result.get('new_segments'),
                                   saved_result.get('new_vias'))
        owner = saved_result.get('_owner_result')
        if owner is not None:
            # Re-grow the owning result so the restored copper ships with it.
            for _key in ('new_segments', 'new_vias'):
                if isinstance(owner.get(_key), list):
                    owner[_key] = list(owner[_key]) + list(saved_result.get(_key) or [])
            routed_results[net_id] = owner
            if net_id not in routed_net_ids:
                routed_net_ids.append(net_id)
            if net_id in remaining_net_ids:
                remaining_net_ids.remove(net_id)
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

    # Add back to pcb_data (tag as 'restore' for the route trace, #482)
    add_route_to_pcb_data(pcb_data, saved_result, debug_lines=config.debug_lines,
                          trace_event='restore')
    # #466: a restore puts copper BACK, so the fragility field must go back
    # up with it. The rip lowered the costs around the vacated corridor; if
    # only the rip refreshes and the restore does not, the field drifts
    # permanently optimistic -- the wrong direction for a guard whose whole
    # job is to make plane-severing paths expensive.
    from plane_fragility import fragility_on_copper_change
    fragility_on_copper_change(config, pcb_data,
                               saved_result.get('new_segments'),
                               saved_result.get('new_vias'))

    # Add back to results list if it was there (and not already present).
    # #369 A2: restore the per-LEG dicts for a multi-leg multipoint pair --
    # they are what the write-list carried (the merged dict duplicates their
    # copper); identity membership, mirroring rip_up_net.
    if was_in_results:
        _members = list(saved_result.get('leg_results') or []) or [saved_result]
        for _r in _members:
            if not any(_r is _x for _x in results):
                results.append(_r)

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
