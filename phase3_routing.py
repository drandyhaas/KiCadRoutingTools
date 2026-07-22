"""
Phase 3 multi-point tap routing.

This module handles routing tap connections for multi-point nets.
Phase 3 runs after:
  - Phase 1: Main route between two furthest pads
  - Phase 2: Length matching (if configured)

Phase 3 connects remaining pads to the main route.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import List, Dict, Set, Optional, Tuple, Any

from kicad_parser import PCBData
from routing_config import GridRouteConfig
from routing_state import RoutingState, record_net_event
from routing_context import build_single_ended_obstacles, build_incremental_obstacles
from single_ended_routing import route_multipoint_taps, route_net_with_obstacles, route_multipoint_main
from connectivity import get_multipoint_net_pads, get_copper_connected_terminal_groups
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers, invalidate_obstacle_cache
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import get_canonical_net_id
from pcb_modification import add_route_to_pcb_data
from obstacle_map import (add_segments_list_as_obstacles, add_vias_list_as_obstacles,
                         remove_segments_list_from_obstacles, remove_vias_list_from_obstacles)
from obstacle_cache import (
    remove_net_obstacles_from_cache, update_net_obstacles_after_routing,
    add_net_obstacles_from_cache, precompute_net_obstacles
)
from obstacle_costs import compute_track_proximity_for_net
from plane_pad_tap import push_inflight_copper, pop_inflight_copper
from terminal_colors import RED, RESET

# Opt-in canary for the #186 segment-crossing scan (see try_phase3_ripup);
# read once at import -- the abandon path is inside the routing hot loop.
_TAP_CROSS_SCAN = bool(os.environ.get('KICAD_TAP_CROSS_SCAN'))


@dataclass
class Phase3Stats:
    """Statistics from Phase 3 routing."""
    tap_edges_routed: int = 0
    tap_edges_failed: int = 0
    total_time: float = 0.0


def _nets_crossing_segments(segs, pcb_data, exclude_net_id, exclude_ids):
    """Net IDs (other than exclude_net_id / exclude_ids) with a segment that truly
    crosses any segment in `segs` on the same layer (a real different-net short).

    Used when a tap rip-up is abandoned: a net re-routed by a NESTED rip-up cascade
    can have been laid across the original tap we are about to keep -- that net is
    NOT in this frame's ripped_items (it was a nested victim), so the #171 cleanup
    misses it and a SEGMENT-CROSSING short ships (issue #186). Finding it here lets
    the abandon re-route it clear of the original tap too.
    """
    from check_drc import segments_cross
    if not segs:
        return []
    hits = set()
    for ts in segs:
        tminx, tmaxx = sorted((ts.start_x, ts.end_x))
        tminy, tmaxy = sorted((ts.start_y, ts.end_y))
        for other in pcb_data.segments:
            if other.net_id in hits or other.net_id == exclude_net_id \
                    or other.net_id in exclude_ids or other.layer != ts.layer:
                continue
            if (max(other.start_x, other.end_x) < tminx or
                    min(other.start_x, other.end_x) > tmaxx or
                    max(other.start_y, other.end_y) < tminy or
                    min(other.start_y, other.end_y) > tmaxy):
                continue
            crosses, _ = segments_cross(ts, other)
            if crosses:
                hits.add(other.net_id)
    return list(hits)


def _reconcile_multipoint_connectivity(new_result, pcb_data, config, net_id):
    """Re-derive a multipoint result's routed/failed pads from the net's ACTUAL
    copper currently in pcb_data, via the same union-find used for MST seeding.

    routed_pad_indices is otherwise set optimistically (the two endpoints of the
    main MST edge are assumed connected). Across rip-and-reroute, the copper that
    actually reached a pad can be changed or lost while routed_pad_indices keeps
    claiming the pad is connected -- so the pad is never flagged or retried (the
    silent SDC0_D3.J2 drop). Recomputing from real copper keeps the flag honest:
    a pad the result no longer reaches lands in failed_pads_info and gets retried.
    """
    pad_info = new_result.get('multipoint_pad_info')
    if not pad_info:
        return
    if any((pi[5] if len(pi) > 5 else None) is None for pi in pad_info):
        return  # can't map pads reliably; leave as-is
    # Overlap-aware grouping (#317): must match the definition used to seed
    # the component MST, or copper joined by cap overlap would be graded as
    # failed pads here and pointlessly re-tapped.
    groups = get_copper_connected_terminal_groups(pcb_data, net_id, pad_info)
    if not groups:
        return
    from collections import Counter
    main_comp = Counter(groups.values()).most_common(1)[0][0]
    routed = {i for i in range(len(pad_info)) if groups.get(i) == main_comp}
    failed = []
    for i in range(len(pad_info)):
        if i not in routed:
            pad = pad_info[i]
            po = pad[5] if len(pad) > 5 else None
            failed.append({'pad_idx': i, 'x': pad[3], 'y': pad[4],
                           'component_ref': getattr(po, 'component_ref', '?') if po else '?',
                           'pad_number': getattr(po, 'pad_number', '?') if po else '?'})
    new_result['routed_pad_indices'] = routed
    new_result['failed_pads_info'] = failed
    new_result['tap_pads_connected'] = len(routed)
    new_result['tap_pads_total'] = len(pad_info)


def _build_congestion_fn(pcb_data, radius=1.0):
    """1mm-bucket spatial hash of all pads/vias -> congestion(net_id, x, y):
    count of FOREIGN pads/vias within `radius` of (x, y). Static proxy for
    "how boxed in is this point" (few escape corridors). Shared by the #347
    phase-3 net ordering and the congestion-weighted #85 abandon metrics.
    NC pads count too -- they physically wall a terminal regardless of net."""
    buckets = {}

    def _bucket_add(x, y, net_id):
        buckets.setdefault((int(x // radius), int(y // radius)), []).append(
            (x, y, net_id))

    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            _bucket_add(p.global_x, p.global_y, p.net_id)
    for v in pcb_data.vias:
        _bucket_add(v.x, v.y, v.net_id)

    def _congestion(net_id, tx, ty):
        n = 0
        bx, by = int(tx // radius), int(ty // radius)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for (x, y, nid) in buckets.get((bx + dx, by + dy), ()):
                    if nid != net_id and abs(x - tx) <= radius and abs(y - ty) <= radius:
                        n += 1
        return n

    return _congestion


# Choices for config.ripup_abandon_metric (issue #85 arbitration; see
# docs/rip-up-reroute.md "Abandon metrics" for definitions and rationale).
ABANDON_METRICS = ('stranded', 'total-pads', 'complete-nets', 'congestion',
                   'history', 'weighted', 'probe', 'weighted-probe')


def _sink_record(sinks, ripped_ids, saved_result):
    """Record ripped nets in every ancestor rip-tree sink (#354), keeping the
    net's PRE-rip failed-pad list from its first rip in each frame's window
    (first write wins) -- the before-world input for the #85 abandon metrics.
    The saved failed list belongs to the first (blocker) id; a diff-pair
    partner ripped alongside it was fully routed."""
    failed = list((saved_result or {}).get('failed_pads_info', []) or [])
    for s in sinks:
        for i, rid in enumerate(ripped_ids):
            if rid not in s:
                s[rid] = failed if i == 0 else []


def _build_abandon_weight_fn(metric, pcb_data, state):
    """Pad-weight function for the weighted #85 abandon metrics.

    congestion: w = 1 + min(foreign pads/vias within 1mm, 24)/8  -> [1, 4]
      (boxed-in pads are unlikely to be reconnectable by a later pass)
    history:    w = 1 + min(times this net was ripped or failed a re-route
      this run, 3)  -> [1, 4]  (empirically-hard nets cost more to lose)
    weighted/weighted-probe: the product of both."""
    use_c = metric in ('congestion', 'weighted', 'weighted-probe')
    use_h = metric in ('history', 'weighted', 'weighted-probe')
    cong = _build_congestion_fn(pcb_data) if use_c else None
    hist_cache = {}

    def _hist_w(nid):
        if nid not in hist_cache:
            events = state.net_history.get(nid, []) if state is not None else []
            n = sum(1 for e in events
                    if e.get('event') in ('ripped_by', 'reroute_failed'))
            hist_cache[nid] = 1.0 + min(n, 3)
        return hist_cache[nid]

    def _weight(nid, x, y):
        w = 1.0
        if cong is not None:
            w *= 1.0 + min(cong(nid, x, y), 24) / 8.0
        if use_h:
            w *= _hist_w(nid)
        return w

    return _weight


def _probe_abandon_world(stranded_ids, orig_tap_segs, orig_tap_vias, ctx):
    """probe / weighted-probe metrics: for each fully-stranded rip-tree net,
    test whether it could re-route in the ABANDON world (original tap guarded
    into the map) under a capped iteration budget. A net unroutable there too
    is lost EITHER way and must not veto the retry. Returns {net_id: bool}.

    Best-effort fidelity: probes the 2-terminal main route only (a multipoint
    victim's taps are not probed), and nothing is committed -- the guard
    add/remove mirrors the abandon path's (#309: matching diagonal_margin;
    #310: inflight window)."""
    from dataclasses import replace as _dc_replace
    result = {}
    if not stranded_ids:
        return result
    state, config, pcb_data = ctx['state'], ctx['config'], ctx['pcb_data']
    guard = (state.working_obstacles is not None
             and (orig_tap_segs or orig_tap_vias))
    token = None
    if guard:
        add_segments_list_as_obstacles(state.working_obstacles, orig_tap_segs, config)
        add_vias_list_as_obstacles(state.working_obstacles, orig_tap_vias, config, diagonal_margin=0.25)
        token = push_inflight_copper(pcb_data, orig_tap_segs, orig_tap_vias)
    probe_cfg = _dc_replace(config, max_iterations=min(config.max_iterations, 50000))
    try:
        for rid in stranded_ids:
            if state.working_obstacles is not None and state.net_obstacles_cache:
                obstacles, _ = build_incremental_obstacles(
                    state.working_obstacles, pcb_data, config, rid,
                    ctx['all_unrouted_net_ids'], ctx['routed_net_ids'],
                    ctx['track_proximity_cache'], ctx['layer_map'],
                    state.net_obstacles_cache)
            else:
                phase3_routed_ids = [x for x in ctx['routed_net_ids'] if x != rid]
                obstacles, _ = build_single_ended_obstacles(
                    ctx['base_obstacles'], pcb_data, config, phase3_routed_ids,
                    ctx['remaining_net_ids'], ctx['all_unrouted_net_ids'], rid,
                    ctx['gnd_net_id'], ctx['track_proximity_cache'], ctx['layer_map'],
                    net_obstacles_cache=state.net_obstacles_cache,
                    ripped_route_layer_costs=state.ripped_route_layer_costs,
                    ripped_route_via_positions=state.ripped_route_via_positions)
            r = route_net_with_obstacles(pcb_data, rid, probe_cfg, obstacles)
            result[rid] = bool(r and not r.get('failed')
                               and (r.get('path') or r.get('phase1_exhausted')))
    finally:
        if guard:
            remove_segments_list_from_obstacles(state.working_obstacles, orig_tap_segs, config)
            remove_vias_list_from_obstacles(state.working_obstacles, orig_tap_vias, config, diagonal_margin=0.25)
            pop_inflight_copper(pcb_data, token)
    return result


def _phase3_abandon_retry(metric, pcb_data, config, state, net_id,
                          completed_result, retry_result, stranded,
                          subtree_before, routed_results,
                          orig_tap_segs, orig_tap_vias, probe_ctx):
    """Issue #85 arbitration: keep the successful tap RETRY, or abandon it in
    favour of the original tap? Returns (abandon: bool, detail: str).

    All metrics except 'stranded' compare the CURRENT world (retry kept: root
    gained pads, rip-tree victims re-routed around the retry tap) against the
    world BEFORE this rip-up frame started (original tap + every rip-tree net
    as it was when first ripped, from _sink_record). See docs/rip-up-reroute.md
    "Abandon metrics"."""
    pads_lost = sum(lost for _item, lost in stranded)
    pads_gained = (len(completed_result.get('failed_pads_info', []))
                   - len(retry_result.get('failed_pads_info', [])))
    base_detail = f"lost {pads_lost} pad(s) to gain {pads_gained}"
    if metric == 'stranded':
        # Original #85 rule: only fully-stranded victims count against the
        # retry; partial victim regressions are greedy churn that recovers.
        return pads_lost > pads_gained, base_detail

    tree_ids = list(subtree_before.keys())
    lost_either_way = set()
    if metric in ('probe', 'weighted-probe'):
        stranded_now = [rid for rid in tree_ids if rid not in routed_results]
        routable = _probe_abandon_world(stranded_now, orig_tap_segs,
                                        orig_tap_vias, probe_ctx)
        lost_either_way = {rid for rid, ok in routable.items() if not ok}

    weight = _build_abandon_weight_fn(metric, pcb_data, state)

    def _value(nid, failed):
        total = sum(weight(nid, p.global_x, p.global_y)
                    for p in pcb_data.pads_by_net.get(nid, []))
        lost = sum(weight(nid, f.get('x', 0.0), f.get('y', 0.0))
                   for f in (failed or []))
        return max(0.0, total - lost)

    def _pad_count(nid, failed):
        return max(0, len(pcb_data.pads_by_net.get(nid, [])) - len(failed or []))

    before_val = _value(net_id, completed_result.get('failed_pads_info'))
    now_val = _value(net_id, retry_result.get('failed_pads_info'))
    before_pads = _pad_count(net_id, completed_result.get('failed_pads_info'))
    now_pads = _pad_count(net_id, retry_result.get('failed_pads_info'))
    before_nets = 1 if not completed_result.get('failed_pads_info') else 0
    now_nets = 1 if not retry_result.get('failed_pads_info') else 0
    for rid in tree_ids:
        if rid in lost_either_way:
            continue  # unroutable in the abandon world too: no vote
        b_failed = subtree_before.get(rid)
        before_val += _value(rid, b_failed)
        before_pads += _pad_count(rid, b_failed)
        before_nets += 1 if not b_failed else 0
        if rid in routed_results:
            n_failed = routed_results[rid].get('failed_pads_info')
            now_val += _value(rid, n_failed)
            now_pads += _pad_count(rid, n_failed)
            now_nets += 1 if not n_failed else 0

    if metric == 'complete-nets':
        abandon = (now_nets, now_pads) < (before_nets, before_pads)
        detail = (f"{base_detail}; complete nets {now_nets} vs {before_nets} "
                  f"(pads {now_pads} vs {before_pads})")
    else:
        abandon = now_val < before_val
        detail = f"{base_detail}; world value {now_val:.1f} vs {before_val:.1f}"
        if lost_either_way:
            detail += f"; {len(lost_either_way)} net(s) unroutable either way"
    return abandon, f"{detail} [{metric}]"


def _order_nets_by_boxed_in_risk(pending_multipoint_nets, pcb_data):
    """Order Phase 3 nets so the most boxed-in pending terminals route FIRST
    (issue #347, core1106 RST1).

    Phase 3 nets contest shared narrow resources -- a fine-pitch connector
    row's single escape corridor fits one tap. In insertion order, an
    early net's tap can wall in a later net's terminal; the later net then
    only connects by ripping the early tap, whose own re-route may strand
    and veto the trade (#85). Routing the most-constrained terminal first
    plays the contest while the board is emptiest.

    Risk = max over the net's PENDING terminals of the local foreign-copper
    density (pads/vias within 1mm), a static proxy for "how few escape
    corridors does this terminal have". Deterministic: integer risk,
    stable sort preserving insertion order on ties.
    """
    items = list(pending_multipoint_nets.items())
    if len(items) < 2:
        return items

    _congestion = _build_congestion_fn(pcb_data)

    def _risk(net_id, main_result):
        pad_info = main_result.get('multipoint_pad_info') or []
        routed = main_result.get('routed_pad_indices') or set()
        best = 0
        for i, info in enumerate(pad_info):
            if i in routed:
                continue
            best = max(best, _congestion(net_id, info[3], info[4]))
        return best

    risks = {net_id: _risk(net_id, mr) for net_id, mr in items}
    ordered = sorted(items, key=lambda kv: -risks[kv[0]])
    if [k for k, _ in ordered] != [k for k, _ in items]:
        names = [pcb_data.nets[k].name if k in pcb_data.nets else f"net_{k}"
                 for k, _ in ordered]
        print(f"Phase 3 order (boxed-in risk first): {', '.join(names)}")
    return ordered


def _commit_net_result(results, routed_results, net_id, new_result,
                       pcb_data=None, config=None):
    """Make new_result this net's single entry in the write-list `results`.

    Drops the net's PRIOR result (a superseded earlier rip-reroute attempt) so it
    doesn't linger and stack coincident same-net vias at the shared transition
    cell (issue #87). Then reconciles routed_pad_indices with the result's actual
    copper, so a pad a reroute dropped is flagged rather than silently kept.
    """
    prior = routed_results.get(net_id)
    if prior is not None and prior is not new_result:
        while prior in results:
            results.remove(prior)
    routed_results[net_id] = new_result
    if new_result not in results:
        results.append(new_result)
    if pcb_data is not None and config is not None and new_result.get('multipoint_pad_info'):
        _reconcile_multipoint_connectivity(new_result, pcb_data, config, net_id)


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
    progress_callback: Any = None,
    cancel_check: Any = None,
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
        # -1 for the main edge routed in Phase 1; an already-connected net
        # (#317 single-component result) has no edges at all, not -1
        max(0, len(result.get('mst_edges', [])) - 1)
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

    total_multipoint_nets = len(state.pending_multipoint_nets)
    net_index = 0
    for net_id, main_result in _order_nets_by_boxed_in_risk(
            state.pending_multipoint_nets, pcb_data):
        # Check for cancellation at start of each phase 3 iteration
        if cancel_check and cancel_check():
            print("\nPhase 3 cancelled by user")
            break

        # Skip if already processed (removed during a Phase 3 rip-up reroute of another net)
        if net_id not in state.pending_multipoint_nets:
            continue

        net_index += 1
        net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
        if progress_callback:
            progress_callback(net_index, total_multipoint_nets, net_name)

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
                net_obstacles_cache=state.net_obstacles_cache,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions
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
                retry_result = try_phase3_ripup(
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

            # Flag pads still unconnected after this net's rip-up attempts. This
            # is a MID-RUN state, not a verdict: a later retry/fallback can still
            # connect the pad, and the end-of-run summary re-checks every net
            # authoritatively (issue #292: usb_pd printed FAILED here for a net
            # the JSON_SUMMARY correctly reported connected).
            final_failed_pads = completed_result.get('failed_pads_info', [])
            if final_failed_pads:
                net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
                for pad in final_failed_pads:
                    print(f"  {RED}NOT CONNECTED (so far): {net_name} - {pad['component_ref']} pad {pad['pad_number']} at ({pad['x']:.2f}, {pad['y']:.2f}) - may still be recovered; final verdict in the end-of-run summary{RESET}")

            # Extract only the NEW tap segments (after the length-matched main route)
            tap_segments = completed_result['new_segments'][len(lm_segments):]
            tap_vias = completed_result['new_vias'][len(lm_vias):]

            if tap_segments or tap_vias:
                tap_result = {'new_segments': tap_segments, 'new_vias': tap_vias}
                add_route_to_pcb_data(pcb_data, tap_result, debug_lines=config.debug_lines)
                print(f"  Added {len(tap_segments)} tap segments, {len(tap_vias)} tap vias")

                # IMPORTANT: Update completed_result['new_segments'] to match what's in pcb_data
                # add_route_to_pcb_data no longer per-commit-cleans segments (the
                # a self-intersection fix, #148), updating
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
            _commit_net_result(results, routed_results, net_id, completed_result,
                               pcb_data, config)

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


def try_phase3_ripup(
    net_id, completed_result, failed_edge_blocking, lm_segments, lm_vias,
    pcb_data, config, state, routed_net_ids, remaining_net_ids,
    all_unrouted_net_ids, routed_net_paths, routed_results,
    diff_pair_by_net_id, results, track_proximity_cache, layer_map,
    base_obstacles, gnd_net_id, phase3_ripped_nets,
    global_tap_offset=0, global_tap_total=0, global_tap_failed=0,
    obstacle_cache=None,
    reroute_depth: int = 0,
    ancestor_net_ids: frozenset = frozenset(),
    rip_sinks: tuple = ()
):
    """
    Try progressive rip-up and retry for failed Phase 3 tap routes.

    Tries N=1, then N=2, etc up to max_rip_up_count blockers.
    Returns updated completed_result if retry succeeded, None otherwise.
    Appends ripped nets to phase3_ripped_nets for later re-routing.

    rip_sinks (#354): tuple of set()s owned by ANCESTOR rip-up frames. Every
    net id ripped in this frame or below is added to all of them, so each
    frame knows its full nested rip TREE (not just its direct victims).
    """
    net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"net_{net_id}"

    # Collect all blocked cells from failed edges. Keep the failing TARGET
    # position (audit RIPUP_ATTRIBUTION_REVIEW: it was stored per edge and
    # then DISCARDED here, so every phase-3 near_target count was zero and
    # ranking degenerated to pure perimeter cell count -- burying the
    # pad-hugging track, and the #424 synthetic conflict cells, under
    # thousands of far-wall cells).
    all_blocked_cells = []
    _tgt_for_rank = None
    _edge_entries = []  # (cells_for_attribution, tgt_xy) per failed edge
    for edge_key, entry in failed_edge_blocking.items():
        blocked_cells, tgt_xy = entry[0], entry[1]
        all_blocked_cells.extend(blocked_cells)
        if _tgt_for_rank is None:
            _tgt_for_rank = tgt_xy
        # Direction-separated attribution (audit #2ii): when both probe
        # directions reported walls, the TIGHTER side is the drained-pocket
        # perimeter (the walled-in pad's wall, hugging track included); the
        # broad side is the far flood that used to swamp it. The synthetic
        # via-conflict cells ride along either way.
        cells = blocked_cells
        dirs = entry[2] if len(entry) > 2 else None
        if dirs:
            fwd, bwd = dirs.get('fwd') or [], dirs.get('bwd') or []
            if fwd and bwd:
                side = fwd if len(fwd) <= len(bwd) else bwd
                cells = list(side) + list(dirs.get('extra') or [])
        _edge_entries.append((cells, tgt_xy))

    if not all_blocked_cells:
        return None

    # Cycle guard (glasgow Z1): never rip a net that is an ANCESTOR in the current
    # rip-up chain (the root net being routed, or any net whose rip-up cascade led
    # here). Ripping an ancestor re-routes it from under its own in-progress retry
    # -- a circular net -> ... -> net rip that orphans copper and ships the root
    # split. exclude_net_ids removes them from blocker selection at every depth.
    exclude_ids = {net_id} | ancestor_net_ids

    # Rip TREE tracking (#354): every net ripped anywhere in this frame's
    # nested cascade, including by nested rip-up frames. If this frame's
    # retry is later ABANDONED, the ORIGINAL tap it keeps was never in the
    # obstacle map while any of these nets re-routed (only the RETRY tap
    # was), so any of them can hold copper exactly on the original tap's
    # cells -- the abandon path must re-rip the whole subtree.
    # Maps net_id -> that net's failed-pad list at its FIRST rip in this
    # frame's window (the #85 abandon metrics' before-world snapshot).
    subtree_ripped = {}
    child_sinks = rip_sinks + (subtree_ripped,)

    # The shared via-placement decline records the micron-exact copper that
    # boxes a pad (find_via_position_blocker) into pcb_data._via_unblock_blame;
    # the SE ladder already consumes it, but the tap cascade only saw the
    # synthetic conflict CELLS and re-inferred identity by rasterization.
    # Feed the exact identities as validator-named blockers instead -- they
    # sort ahead of every frontier-inferred tier under all select algorithms.
    _blame = getattr(pcb_data, '_via_unblock_blame', None)
    _blame_ids = _blame.pop(net_id, None) if _blame else None
    _known = [(nid, 1) for nid in _blame_ids] if _blame_ids else None

    # PER-EDGE attribution (audit #2i): three tap edges failing in three
    # corners used to become one merged cell soup ranked by whichever net
    # had the most total perimeter -- possibly relevant to none of them.
    # Attribute each edge against ITS OWN target, then merge per net by MAX
    # so the net decisive at one edge isn't diluted by the others.
    from blocking_analysis import (apply_known_blockers, merge_blocking_max,
                                   rank_blockers)
    _merged = {}
    for _cells, _tgt in _edge_entries:
        for b in analyze_frontier_blocking(
                _cells, pcb_data, config, routed_net_paths,
                exclude_net_ids=exclude_ids,
                target_xy=_tgt,
                obstacle_cache=obstacle_cache):
            m = _merged.get(b.net_id)
            _merged[b.net_id] = b if m is None else merge_blocking_max(m, b)
    blockers = list(_merged.values())
    apply_known_blockers(blockers, _known, exclude_ids, pcb_data)
    rank_blockers(blockers, getattr(config, 'ripup_blocker_select', 'count'),
                  config=config)

    if not blockers:
        return None

    # Filter to rippable blockers
    rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
        blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id
    )

    if not rippable_blockers:
        return None

    # mincut probe in the cascade (advisory, like the SE ladder): NET-level
    # approximation -- the probe routes the multipoint net's endpoint gap on
    # a clone with rippable copper soft-costed, so its crossing set names
    # the joint cut for the dominant gap (not per failed edge). Feasible
    # probe reorders; infeasible or empty keeps the ranking above.
    if (getattr(config, 'ripup_blocker_select', 'count') == 'mincut'
            and getattr(state, 'working_obstacles', None) is not None
            and getattr(state, 'net_obstacles_cache', None) is not None):
        from blocking_analysis import mincut_probe_order
        _mc_order, _mc_feasible = mincut_probe_order(
            pcb_data, config, state.working_obstacles, net_id,
            rippable_blockers, state.net_obstacles_cache)
        if _mc_feasible and _mc_order:
            _by_id = {b.net_id: b for b in rippable_blockers}
            _front = [_by_id[n] for n in _mc_order if n in _by_id]
            _rest = [b for b in rippable_blockers
                     if b.net_id not in set(_mc_order)]
            rippable_blockers = _front + _rest
            print(f"    MINCUT probe (net-level): cut = "
                  f"{[b.net_name for b in _front]}")
        elif not _mc_feasible:
            print(f"    MINCUT probe (net-level): no path with rippable "
                  f"copper soft-costed -- keeping per-edge ranking")

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
                exclude_net_ids=exclude_ids,
                target_xy=_tgt_for_rank,
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
            state.working_obstacles, state.net_obstacles_cache,
            state.ripped_route_layer_costs, state.ripped_route_via_positions,
            layer_map
        )

        if saved_result is None:
            print(f"    Failed to rip up {blocker_name}")
            break

        ripped_items.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
        ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
        _sink_record(child_sinks, ripped_ids, saved_result)
        # Invalidate obstacle cache for ripped nets and record rip events
        for rid in ripped_ids:
            if obstacle_cache is not None:
                invalidate_obstacle_cache(obstacle_cache, rid)
            # Record rip event for the ripped net
            record_net_event(state, rid, "ripped_by", {
                "ripping_net_id": net_id,
                "ripping_net_name": net_name,
                "reason": f"Phase 3 tap rip-up N={N}",
                "N": N
            })

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
                net_obstacles_cache=state.net_obstacles_cache,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions
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
                print(f"    {net_name}: Retry SUCCESS (N={N}): {original_failed_count} -> {retry_failed} failed edges")

                # Add retry result's tap segments to obstacles TEMPORARILY for re-routing ripped nets.
                # This prevents ripped nets from routing through our tap area.
                # We will REMOVE them before returning, since the caller will add them properly
                # via the obstacle cache (which handles ref-counting correctly).
                #
                # EXCEPTION: If net_id is in ripped_items, DON'T add its tap segments.
                # The tap routing was based on the OLD Phase 1 path (from completed_result),
                # but when the net is re-routed, Phase 1 will be re-done with a potentially
                # different path.
                ripped_net_ids = {item[0] for item in ripped_items}
                tap_segments_added = []
                tap_vias_added = []
                inflight_token = None
                if state.working_obstacles is not None and net_id not in ripped_net_ids:
                    tap_segments_added = retry_result['new_segments'][len(lm_segments):]
                    tap_vias_added = retry_result['new_vias'][len(lm_vias):]
                    if tap_segments_added or tap_vias_added:
                        print(f"    Adding {len(tap_segments_added)} tap segments, {len(tap_vias_added)} tap vias to obstacles before re-routing...")
                        add_segments_list_as_obstacles(state.working_obstacles, tap_segments_added, config)
                        add_vias_list_as_obstacles(state.working_obstacles, tap_vias_added, config, diagonal_margin=0.25)
                        # This copper is in the obstacle map but NOT in pcb_data
                        # until the caller commits it; register it so pcb_data-
                        # windowed via placement (#189 unblock) sees it too (#310).
                        inflight_token = push_inflight_copper(
                            pcb_data, tap_segments_added, tap_vias_added)
                elif net_id in ripped_net_ids:
                    print(f"    Skipping tap obstacle addition - net will be re-routed")

                # Re-route ripped nets IMMEDIATELY (not deferred) to prevent subsequent
                # Phase 3 nets from routing through the ripped area
                stranded = []
                if ripped_items:
                    print(f"    Re-routing {len(ripped_items)} ripped net(s)...")
                    stranded = _reroute_phase3_ripped_nets(
                        ripped_items, pcb_data, config, state, routed_net_ids, remaining_net_ids,
                        all_unrouted_net_ids, routed_net_paths, routed_results, diff_pair_by_net_id,
                        results, track_proximity_cache, layer_map, base_obstacles, gnd_net_id,
                        reroute_depth=reroute_depth + 1,
                        ancestor_net_ids=exclude_ids,
                        rip_sinks=child_sinks
                    )

                # IMPORTANT: Remove the temporarily added tap segments before returning.
                # The caller will add them properly via obstacle cache update, which handles
                # ref-counting correctly. If we don't remove them here, they'll be added twice
                # and won't be properly removed when the net is later ripped.
                if tap_segments_added or tap_vias_added:
                    remove_segments_list_from_obstacles(state.working_obstacles, tap_segments_added, config)
                    # diagonal_margin MUST match the add above (0.25): the margin
                    # inflates every via's per-layer track disc, so removing
                    # without it leaves a quarter-cell ring ref-counted forever
                    # (the working-map blocked_cells leak of issue #309).
                    remove_vias_list_from_obstacles(state.working_obstacles, tap_vias_added, config, diagonal_margin=0.25)
                    pop_inflight_copper(pcb_data, inflight_token)

                # Issue #85: only commit the rip-up if it is a net improvement.
                # Stranding (totally losing) a previously-routed victim is fine
                # only when this tap connected at least as many pads as the
                # victim(s) lost; trading a whole multipoint net for one tap
                # edge is not. When it doesn't pay off, abandon THIS tap retry:
                # return None so the caller keeps net_id's original (smaller)
                # tap, which never claimed the freed-edge space - then re-route
                # the stranded victims into that now-free space.
                #
                # The victims are re-routed FRESH (not restored from their old
                # copper): the board has moved on since they were ripped, so
                # adding back stale segments would short against it. A fresh
                # route avoids whatever is actually on the board now. We guard
                # it with net_id's original tap so the victim steers clear of
                # the copper the caller is about to commit.
                orig_tap_segs = completed_result['new_segments'][len(lm_segments):]
                orig_tap_vias = completed_result['new_vias'][len(lm_vias):]
                probe_ctx = {
                    'state': state, 'config': config, 'pcb_data': pcb_data,
                    'routed_net_ids': routed_net_ids,
                    'remaining_net_ids': remaining_net_ids,
                    'all_unrouted_net_ids': all_unrouted_net_ids,
                    'track_proximity_cache': track_proximity_cache,
                    'layer_map': layer_map, 'base_obstacles': base_obstacles,
                    'gnd_net_id': gnd_net_id,
                }
                abandon, abandon_detail = _phase3_abandon_retry(
                    getattr(config, 'ripup_abandon_metric', 'stranded'),
                    pcb_data, config, state, net_id, completed_result,
                    retry_result, stranded, subtree_ripped, routed_results,
                    orig_tap_segs, orig_tap_vias, probe_ctx)
                if abandon:
                    stranded_names = ", ".join(
                        pcb_data.nets[item[0]].name if item[0] in pcb_data.nets else f"net_{item[0]}"
                        for item, _lost in stranded
                    )
                    print(f"    {RED}{net_name}: tap rip-up {abandon_detail} "
                          f"({stranded_names}); abandoning tap, "
                          f"re-routing victim(s){RESET}")
                    guard = (state.working_obstacles is not None
                             and (orig_tap_segs or orig_tap_vias))
                    guard_token = None
                    if guard:
                        add_segments_list_as_obstacles(state.working_obstacles, orig_tap_segs, config)
                        add_vias_list_as_obstacles(state.working_obstacles, orig_tap_vias, config, diagonal_margin=0.25)
                        # Same in-flight window as above: the kept original tap
                        # is not in pcb_data while the victims re-route (#310).
                        guard_token = push_inflight_copper(
                            pcb_data, orig_tap_segs, orig_tap_vias)
                    # Issue #171: the victims that DID re-route above avoided the
                    # RETRY tap copper (added to obstacles before the re-route),
                    # but we are now DISCARDING that retry in favour of net_id's
                    # ORIGINAL tap. Those victims' fresh copper can short against
                    # the original tap (it occupies different cells than the retry
                    # tap did). Rip every still-routed victim so the full set is
                    # re-routed clear of the original tap we are about to commit -
                    # not just the stranded ones.
                    reroute_items = list(ripped_items)
                    for item in reroute_items:
                        vic_id = item[0]
                        if vic_id in routed_results:
                            _saved, re_ripped, _wir = rip_up_net(
                                vic_id, pcb_data, routed_net_ids, routed_net_paths,
                                routed_results, diff_pair_by_net_id, remaining_net_ids,
                                results, config, track_proximity_cache,
                                state.working_obstacles, state.net_obstacles_cache,
                                state.ripped_route_layer_costs, state.ripped_route_via_positions,
                                layer_map
                            )
                            if _saved is not None:
                                _sink_record(child_sinks, re_ripped, _saved)
                    existing_ids = {it[0] for it in reroute_items}
                    # Issue #354: re-rip EVERY net ripped anywhere in the nested
                    # cascade beneath this frame (the full rip TREE), not just the
                    # direct victims above. Nested victims re-routed while only the
                    # RETRY tap was in the obstacle map -- the ORIGINAL tap we are
                    # keeping was invisible to them -- so their fresh copper can sit
                    # exactly on it (quickfeather: /SWD.SYS_RST's via drill-on-drill
                    # under /SPI_{DEV}.MOSI's kept tap via). The #186 segment-crossing
                    # scan below cannot see via-on-via or via-under-track overlaps;
                    # the tree covers every net whose copper can postdate the
                    # original tap's obstacle snapshot.
                    for cid in sorted(subtree_ripped):
                        if (cid in existing_ids or cid in exclude_ids
                                or cid not in routed_results):
                            continue
                        c_saved, c_ripped, c_was_in = rip_up_net(
                            cid, pcb_data, routed_net_ids, routed_net_paths,
                            routed_results, diff_pair_by_net_id, remaining_net_ids,
                            results, config, track_proximity_cache,
                            state.working_obstacles, state.net_obstacles_cache,
                            state.ripped_route_layer_costs, state.ripped_route_via_positions,
                            layer_map
                        )
                        if c_saved is not None:
                            print(f"    {net_name}: re-routing {pcb_data.nets[cid].name if cid in pcb_data.nets else f'net_{cid}'} "
                                  f"clear of the kept tap (cascade rip tree; issue #354)")
                            reroute_items.append((cid, c_saved, c_ripped, c_was_in))
                            existing_ids.add(cid)
                            _sink_record(child_sinks, c_ripped, c_saved)
                    # Issue #186: also re-route any OTHER routed net whose copper
                    # crosses the original tap we are keeping. The #354 rip-tree
                    # re-rip above subsumes this (only cascade-ripped nets can
                    # postdate the original tap's obstacle snapshot), so the scan
                    # is OFF by default -- kept as an opt-in canary
                    # (KICAD_TAP_CROSS_SCAN=1): if it ever fires, some code path
                    # moved copper during the cascade without a recorded rip.
                    for cid in (_nets_crossing_segments(orig_tap_segs, pcb_data, net_id, existing_ids)
                                if _TAP_CROSS_SCAN else ()):
                        if cid not in routed_results:
                            continue
                        c_saved, c_ripped, c_was_in = rip_up_net(
                            cid, pcb_data, routed_net_ids, routed_net_paths,
                            routed_results, diff_pair_by_net_id, remaining_net_ids,
                            results, config, track_proximity_cache,
                            state.working_obstacles, state.net_obstacles_cache,
                            state.ripped_route_layer_costs, state.ripped_route_via_positions,
                            layer_map
                        )
                        if c_saved is not None:
                            print(f"    {net_name}: re-routing {pcb_data.nets[cid].name if cid in pcb_data.nets else f'net_{cid}'} "
                                  f"clear of the kept tap (crossed it; issue #186)")
                            reroute_items.append((cid, c_saved, c_ripped, c_was_in))
                            existing_ids.add(cid)
                            _sink_record(child_sinks, c_ripped, c_saved)
                    _reroute_phase3_ripped_nets(
                        reroute_items,
                        pcb_data, config, state, routed_net_ids, remaining_net_ids,
                        all_unrouted_net_ids, routed_net_paths, routed_results, diff_pair_by_net_id,
                        results, track_proximity_cache, layer_map, base_obstacles, gnd_net_id,
                        reroute_depth=reroute_depth + 1,
                        ancestor_net_ids=exclude_ids,
                        rip_sinks=child_sinks
                    )
                    if guard:
                        remove_segments_list_from_obstacles(state.working_obstacles, orig_tap_segs, config)
                        # diagonal_margin must match the add above (0.25) - see #309.
                        remove_vias_list_from_obstacles(state.working_obstacles, orig_tap_vias, config, diagonal_margin=0.25)
                        pop_inflight_copper(pcb_data, guard_token)
                    return None

                return retry_result
            else:
                print(f"    {net_name}: Retry FAILED (N={N}): still {retry_failed} failed edges")
                # Store blocked cells from retry for next iteration's analysis
                retry_blocking = retry_result.get('failed_edge_blocking', {})
                if retry_blocking:
                    last_retry_blocked_cells = []
                    for edge_key, entry in retry_blocking.items():
                        last_retry_blocked_cells.extend(entry[0])
                    if last_retry_blocked_cells:
                        print(f"      Retry had {len(last_retry_blocked_cells)} blocked cells")
        else:
            print(f"    {net_name}: Retry FAILED (N={N}): no result")
            break

    # All attempts failed - restore ripped nets
    if ripped_items:
        print(f"    {RED}{net_name}: All rip-up attempts failed, restoring {len(ripped_items)} net(s)...{RESET}")
        for rid, saved_result, ripped_ids, was_in_results in reversed(ripped_items):
            restore_net(
                rid, saved_result, ripped_ids, was_in_results,
                pcb_data, routed_net_ids, routed_net_paths,
                routed_results, diff_pair_by_net_id, remaining_net_ids,
                results, config, track_proximity_cache, layer_map,
                state.working_obstacles, state.net_obstacles_cache,
                state.ripped_route_layer_costs, state.ripped_route_via_positions,
                refused_sink=state.collision_refused_net_ids
            )

    return None


def _retry_victim_main_with_ripup(
    victim_id, was_multipoint, failed_result,
    pcb_data, config, state, routed_net_ids, remaining_net_ids,
    all_unrouted_net_ids, routed_net_paths, routed_results,
    diff_pair_by_net_id, results, track_proximity_cache, layer_map,
    base_obstacles, gnd_net_id,
    ancestor_net_ids: frozenset = frozenset(),
    rip_sinks: tuple = ()
):
    """Progressive rip-up retry for a rip-VICTIM's failed MAIN re-route
    (issue #347, core1106 RST1).

    try_phase3_ripup rips a blocker, routes the tap, then re-routes the
    victim -- but the victim's main route was attempted BARE: one failure
    and it stranded, the #85 arbitration counted its pads as lost, and the
    ORIGINAL (successful!) tap rip-up was abandoned to restore the victim.
    Giving the victim the same progressive rip-up the root tap got lets
    both nets land and the rip-up pay off.

    Ancestor-guarded like try_phase3_ripup: never rips the root net being
    tapped or any net in the current rip chain. Uses only rip_up_net /
    restore_net and the incremental obstacle builders, so the working-map
    ref-counts stay balanced (#309 -- no raw add/remove pairs here).

    Returns (result_or_None, nested_ripped_items). On success the caller
    must re-route nested_ripped_items AFTER committing the victim's copper;
    on failure everything ripped here has been restored and [] is returned.
    """
    victim_name = pcb_data.nets[victim_id].name if victim_id in pcb_data.nets else f"net_{victim_id}"
    # Fastest-failing direction first (audit #3b): the constrained side's
    # frontier is the drained-pocket perimeter; pooling both directions let
    # the broad flood swamp it. Mirror of the SE loop's selection.
    _fr = failed_result or {}
    _fwd, _bwd = _fr.get('blocked_cells_forward', []), _fr.get('blocked_cells_backward', [])
    _fit, _bit = _fr.get('iterations_forward', 0), _fr.get('iterations_backward', 0)
    if _fwd and _bwd and (_fit > 0 or _bit > 0):
        blocked = list(dict.fromkeys(
            _fwd if (_fit > 0 and (_bit == 0 or _fit <= _bit)) else _bwd))
    else:
        blocked = list(dict.fromkeys(list(_fwd) + list(_bwd)))
    if not blocked:
        return None, []

    # Endpoint proximity for the ranking (audit #2i): the victim's own
    # endpoints; without them every near_* count is zero and the sort
    # degenerates to pure cell count.
    from connectivity import get_net_endpoints as _gne
    _v_src_xy = _v_tgt_xy = None
    try:
        _v_srcs, _v_tgts, _ = _gne(pcb_data, victim_id, config)
        if _v_srcs:
            _v_src_xy = (_v_srcs[0][3], _v_srcs[0][4])
        if _v_tgts:
            _v_tgt_xy = (_v_tgts[0][3], _v_tgts[0][4])
    except Exception:
        pass

    exclude_ids = {victim_id} | ancestor_net_ids
    blockers = analyze_frontier_blocking(
        blocked, pcb_data, config, routed_net_paths,
        exclude_net_ids=exclude_ids,
        target_xy=_v_tgt_xy, source_xy=_v_src_xy)
    if not blockers:
        return None, []
    rippable_blockers, seen_canonical_ids = filter_rippable_blockers(
        blockers, routed_results, diff_pair_by_net_id, get_canonical_net_id)
    if not rippable_blockers:
        print(f"    {victim_name}: main re-route blocked, no rippable blockers")
        return None, []

    nested_ripped = []
    ripped_canonical_ids = set()
    last_blocked = blocked
    for N in range(1, config.max_rip_up_count + 1):
        if N > 1 and last_blocked:
            fresh = analyze_frontier_blocking(
                last_blocked, pcb_data, config, routed_net_paths,
                exclude_net_ids=exclude_ids,
                target_xy=_v_tgt_xy, source_xy=_v_src_xy)
            next_blocker = None
            for b in fresh:
                if b.net_id in routed_results:
                    canonical = get_canonical_net_id(b.net_id, diff_pair_by_net_id)
                    if canonical not in ripped_canonical_ids:
                        next_blocker = b
                        break
            if next_blocker is None:
                break
            next_canonical = get_canonical_net_id(next_blocker.net_id, diff_pair_by_net_id)
            if next_canonical not in seen_canonical_ids:
                seen_canonical_ids.add(next_canonical)
                rippable_blockers.append(next_blocker)
        if N > len(rippable_blockers):
            break

        blocker = rippable_blockers[N - 1]
        blocker_name = pcb_data.nets[blocker.net_id].name \
            if blocker.net_id in pcb_data.nets else f"net_{blocker.net_id}"
        print(f"    {victim_name}: main re-route blocked, ripping {blocker_name} "
              f"(N={N}) to retry...")
        saved_result, ripped_ids, was_in_results = rip_up_net(
            blocker.net_id, pcb_data, routed_net_ids, routed_net_paths,
            routed_results, diff_pair_by_net_id, remaining_net_ids,
            results, config, track_proximity_cache,
            state.working_obstacles, state.net_obstacles_cache,
            state.ripped_route_layer_costs, state.ripped_route_via_positions,
            layer_map
        )
        if saved_result is None:
            print(f"    Failed to rip up {blocker_name}")
            break
        nested_ripped.append((blocker.net_id, saved_result, ripped_ids, was_in_results))
        ripped_canonical_ids.add(get_canonical_net_id(blocker.net_id, diff_pair_by_net_id))
        _sink_record(rip_sinks, ripped_ids, saved_result)
        for rid in ripped_ids:
            record_net_event(state, rid, "ripped_by", {
                "ripping_net_id": victim_id,
                "ripping_net_name": victim_name,
                "reason": f"Phase 3 victim main re-route rip-up N={N}",
                "N": N
            })

        if state.working_obstacles is not None and state.net_obstacles_cache:
            obstacles, _ = build_incremental_obstacles(
                state.working_obstacles, pcb_data, config, victim_id,
                all_unrouted_net_ids, routed_net_ids, track_proximity_cache,
                layer_map, state.net_obstacles_cache)
        else:
            phase3_routed_ids = [rid for rid in routed_net_ids if rid != victim_id]
            obstacles, _ = build_single_ended_obstacles(
                base_obstacles, pcb_data, config, phase3_routed_ids, remaining_net_ids,
                all_unrouted_net_ids, victim_id, gnd_net_id, track_proximity_cache,
                layer_map, net_obstacles_cache=state.net_obstacles_cache,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions)

        # A bus-member victim re-routes with its corridor (no routed-paths
        # dict tracked at Phase-3 depth; the planned corridor is the stable
        # artifact and the usual hit).
        from bus_detection import bus_attraction_context
        _attr, _rev = bus_attraction_context(
            victim_id, getattr(state, 'bus_net_to_group', None),
            getattr(state, 'bus_corridors', None))
        if was_multipoint:
            multipoint_pads = get_multipoint_net_pads(pcb_data, victim_id, config)
            if multipoint_pads:
                result = route_multipoint_main(pcb_data, victim_id, config, obstacles, multipoint_pads,
                                               attraction_path=_attr)
            else:
                result = route_net_with_obstacles(pcb_data, victim_id, config, obstacles,
                                                  attraction_path=_attr, reverse_direction=_rev)
        else:
            result = route_net_with_obstacles(pcb_data, victim_id, config, obstacles,
                                              attraction_path=_attr, reverse_direction=_rev)

        if result and not result.get('failed') \
                and (result.get('path') or result.get('phase1_exhausted')):
            # phase1_exhausted (#348) is a SUCCESS shape: no new main copper,
            # but the net's island copper sources its Phase-3 taps. Reading
            # it as failure burned further rip-up rounds for nothing.
            print(f"    {victim_name}: main re-route rip-up SUCCESS (N={N})")
            return result, nested_ripped
        last_blocked = list(dict.fromkeys(
            (result or {}).get('blocked_cells_forward', [])
            + (result or {}).get('blocked_cells_backward', [])))

    if nested_ripped:
        print(f"    {victim_name}: main re-route rip-up failed, restoring "
              f"{len(nested_ripped)} net(s)...")
        for rid, saved_result, ripped_ids, was_in_results in reversed(nested_ripped):
            restore_net(
                rid, saved_result, ripped_ids, was_in_results,
                pcb_data, routed_net_ids, routed_net_paths,
                routed_results, diff_pair_by_net_id, remaining_net_ids,
                results, config, track_proximity_cache, layer_map,
                state.working_obstacles, state.net_obstacles_cache,
                state.ripped_route_layer_costs, state.ripped_route_via_positions,
                refused_sink=state.collision_refused_net_ids
            )
    return None, []


def _reroute_phase3_ripped_nets(
    phase3_ripped_nets, pcb_data, config, state, routed_net_ids, remaining_net_ids,
    all_unrouted_net_ids, routed_net_paths, routed_results, diff_pair_by_net_id,
    results, track_proximity_cache, layer_map, base_obstacles, gnd_net_id,
    reroute_depth: int = 0,
    ancestor_net_ids: frozenset = frozenset(),
    rip_sinks: tuple = ()
):
    """
    Re-route nets that were ripped during Phase 3 tap routing.

    This includes routing their main route and any tap connections.
    If tap routing fails, attempts rip-up retry (up to config.max_rip_up_count depth).

    Returns the list of victims left TOTALLY unrouted (no copper at all), as
    ((net_id, saved_result, ripped_ids, was_in_results), pads_lost) entries.
    A non-empty return tells the caller the rip-up that produced these victims
    may not have paid off; it weighs pads_lost against what the tap bought and,
    if the trade is negative, abandons the tap and re-routes the victims into
    the freed space (issue #85).
    """
    stranded = []

    for ripped_net_id, saved_result, ripped_ids, was_in_results in phase3_ripped_nets:
        net_name = pcb_data.nets[ripped_net_id].name if ripped_net_id in pcb_data.nets else f"net_{ripped_net_id}"
        print(f"\n  Re-routing {net_name} (net {ripped_net_id})...")

        # Skip if already re-routed (by another rip-up)
        if ripped_net_id in routed_results:
            print(f"    Already routed, skipping")
            continue

        # Connectivity the victim had before being ripped: re-routing must not
        # leave it worse off than this, or ripping it was a net loss.
        saved_failed_count = len(saved_result.get('failed_pads_info', [])) if saved_result else 0

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
                net_obstacles_cache=state.net_obstacles_cache,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions
            )

        # Check if this was originally a multi-point net (from saved_result)
        was_multipoint = saved_result and saved_result.get('mst_edges') and len(saved_result.get('mst_edges', [])) > 1

        # Ripped bus members reroute with their corridor here too
        from bus_detection import bus_attraction_context
        _attr, _rev = bus_attraction_context(
            ripped_net_id, getattr(state, 'bus_net_to_group', None),
            getattr(state, 'bus_corridors', None))
        if was_multipoint:
            # Use multipoint routing for multi-point nets
            multipoint_pads = get_multipoint_net_pads(pcb_data, ripped_net_id, config)
            if multipoint_pads:
                result = route_multipoint_main(pcb_data, ripped_net_id, config, obstacles, multipoint_pads,
                                               attraction_path=_attr)
            else:
                result = route_net_with_obstacles(pcb_data, ripped_net_id, config, obstacles,
                                                  attraction_path=_attr, reverse_direction=_rev)
        else:
            # Route the main path for simple nets
            result = route_net_with_obstacles(pcb_data, ripped_net_id, config, obstacles,
                                              attraction_path=_attr, reverse_direction=_rev)

        # A victim whose main re-route fails BARE strands, and the #85
        # arbitration then abandons the (successful) tap rip-up that ripped
        # it -- give the victim the same progressive rip-up the tap got
        # (#347, core1106 RST1). Ancestor-guarded; depth-capped.
        nested_ripped_items = []
        if ((result is None or result.get('failed') or not result.get('path'))
                and config.max_rip_up_count > 0
                and reroute_depth < config.max_rip_up_count):
            retry, nested_ripped_items = _retry_victim_main_with_ripup(
                ripped_net_id, was_multipoint, result,
                pcb_data, config, state, routed_net_ids, remaining_net_ids,
                all_unrouted_net_ids, routed_net_paths, routed_results,
                diff_pair_by_net_id, results, track_proximity_cache, layer_map,
                base_obstacles, gnd_net_id,
                ancestor_net_ids=ancestor_net_ids | {ripped_net_id},
                rip_sinks=rip_sinks)
            if retry is not None:
                result = retry

        if result and not result.get('failed') \
                and result.get('phase1_exhausted'):
            # #348 island-source synthetic (path=[], no new copper): the
            # victim's pad terminals are boxed in but its existing copper
            # islands can source Phase-3 taps. Mirror the normal-flow
            # handling (single_ended_loop) instead of reading the empty
            # path as failure -- that stranded the exact net the fallback
            # was built to save and killed its pending taps.
            if ripped_net_id in state.pending_multipoint_nets:
                state.pending_multipoint_nets[ripped_net_id] = result
            record_net_event(state, ripped_net_id, "reroute_phase1_exhausted",
                             {"taps_pending": True})
        elif result and not result.get('failed') and result.get('path'):
            main_vias = result.get('new_vias', [])
            add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
            _commit_net_result(results, routed_results, ripped_net_id, result,
                               pcb_data, config)
            routed_net_ids.append(ripped_net_id)
            if ripped_net_id in remaining_net_ids:
                remaining_net_ids.remove(ripped_net_id)
            routed_net_paths[ripped_net_id] = result['path']
            record_net_event(state, ripped_net_id, "reroute_succeeded", {
                "segments": len(result['new_segments']),
                "vias": len(main_vias)
            })
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

            # Re-route nets ripped for THIS victim's main route (#347), now
            # that the victim's copper is committed to pcb_data and the
            # working map (they must route around it, not through it). Their
            # strandings count toward the caller's #85 arbitration.
            if nested_ripped_items:
                print(f"    Re-routing {len(nested_ripped_items)} net(s) ripped for {net_name}'s main route...")
                nested_stranded = _reroute_phase3_ripped_nets(
                    nested_ripped_items, pcb_data, config, state, routed_net_ids,
                    remaining_net_ids, all_unrouted_net_ids, routed_net_paths,
                    routed_results, diff_pair_by_net_id, results,
                    track_proximity_cache, layer_map, base_obstacles, gnd_net_id,
                    reroute_depth=reroute_depth + 1,
                    ancestor_net_ids=ancestor_net_ids | {ripped_net_id},
                    rip_sinks=rip_sinks)
                stranded.extend(nested_stranded)

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

                # Try rip-up retry for failed tap edges (if within depth limit)
                if tap_result:
                    failed_edge_blocking = tap_result.get('failed_edge_blocking', {})
                    original_failed_count = tap_result.get('tap_edges_failed', 0)

                    if failed_edge_blocking and original_failed_count > 0 and reroute_depth < config.max_rip_up_count and config.max_rip_up_count > 0:
                        all_blocked_cells = []
                        for edge_key, entry in failed_edge_blocking.items():
                            all_blocked_cells.extend(entry[0])

                        if all_blocked_cells:
                            print(f"    {net_name}: Attempting rip-up retry for {original_failed_count} failed tap edge(s)...")

                            # lm_segments/lm_vias = Phase 1 segments (before taps)
                            lm_segments = result['new_segments']
                            lm_vias = result.get('new_vias', [])

                            retry_result = try_phase3_ripup(
                                ripped_net_id, tap_result, failed_edge_blocking, lm_segments, lm_vias,
                                pcb_data, config, state, routed_net_ids, remaining_net_ids,
                                all_unrouted_net_ids, routed_net_paths, routed_results,
                                diff_pair_by_net_id, results, track_proximity_cache, layer_map,
                                base_obstacles, gnd_net_id, [],  # unused parameter
                                reroute_depth=reroute_depth + 1,
                                ancestor_net_ids=ancestor_net_ids,
                                rip_sinks=rip_sinks
                            )

                            if retry_result is not None:
                                print(f"    {net_name}: Rip-up retry succeeded!")
                                tap_result = retry_result

                if tap_result:
                    tap_segments = tap_result['new_segments'][len(result['new_segments']):]
                    tap_vias = tap_result['new_vias'][len(result.get('new_vias', [])):]

                    if tap_segments or tap_vias:
                        tap_result_data = {'new_segments': tap_segments, 'new_vias': tap_vias}
                        add_route_to_pcb_data(pcb_data, tap_result_data, debug_lines=config.debug_lines)
                        print(f"    Re-routed {len(tap_segments)} tap segments, {len(tap_vias)} tap vias")

                        # IMPORTANT: Update tap_result['new_segments'] to match what's in pcb_data
                        # add_route_to_pcb_data no longer per-commit-cleans segments (the
                        # just a self-intersection fix, #148), updating
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
                    _commit_net_result(results, routed_results, ripped_net_id, tap_result,
                                       pcb_data, config)

                    # Print red final failure message if re-routed net still has unconnected pads
                    final_failed_pads = tap_result.get('failed_pads_info', [])
                    if final_failed_pads:
                        net_name = pcb_data.nets[ripped_net_id].name if ripped_net_id in pcb_data.nets else f"Net {ripped_net_id}"
                        for pad in final_failed_pads:
                            print(f"    {RED}NOT CONNECTED (so far): {net_name} - {pad['component_ref']} pad {pad['pad_number']} at ({pad['x']:.2f}, {pad['y']:.2f}) - may still be recovered; final verdict in the end-of-run summary{RESET}")

                    # Remove from pending_multipoint_nets since Phase 3 is now complete
                    if ripped_net_id in state.pending_multipoint_nets:
                        del state.pending_multipoint_nets[ripped_net_id]
        else:
            record_net_event(state, ripped_net_id, "reroute_failed", {
                "reason": "no path found during Phase 3 re-route"
            })
            print(f"    {RED}Failed to re-route{RESET}")
            # CRITICAL: Remove from pending_multipoint_nets when re-route fails
            # If we don't remove it, Phase 3 will later process this net and start with
            # the OLD stale segments from the pending result (which were ripped and are
            # no longer in pcb_data), leading to duplicate/stale segments in the output.
            if ripped_net_id in state.pending_multipoint_nets:
                del state.pending_multipoint_nets[ripped_net_id]

        # Did re-routing leave the victim with NO route at all? That total
        # strand is the issue #85 pathology: a previously-routed net wiped out
        # to make room for a tap, with no recovery. Record how many pad
        # connections it lost so the caller can weigh that against what the
        # rip-up bought. Partial regressions (the net still routes but drops a
        # tap pad) are normal greedy churn and recover, so are not flagged -
        # treating them as strandings vetoes beneficial rip-ups and lowers
        # overall connectivity. Entries are
        # ((net_id, saved_result, ripped_ids, was_in_results), pads_lost).
        if routed_results.get(ripped_net_id) is None:
            item = (ripped_net_id, saved_result, ripped_ids, was_in_results)
            num_pads = len(pcb_data.pads_by_net.get(ripped_net_id, []))
            pads_lost = max(1, num_pads - saved_failed_count)
            stranded.append((item, pads_lost))

    return stranded
