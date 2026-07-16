"""Per-pair failure diagnostics + reconciliation casualty custody for
differential-pair routing (Tier-1 lever: diff commits 1+3).

Two engine-side gaps this module closes (inside batch_route_diff_pairs, so the
CLI front and the GUI plugin inherit both for free):

1. DIAGNOSTICS - when a diff pair fails / is deferred, JSON_SUMMARY now carries
   a structured per-pair record saying WHY (reason code + stage + blockers),
   including an explicit 'pose-router-failure' code so PoseRouter search
   exhaustion is never silent again (#346), plus a member audit that verifies
   BOTH members' connectivity claims against actual pad connectivity.

2. CASUALTY CUSTODY - a net ripped during diff routing whose reroute never
   landed used to ship at ZERO copper with no custody. Every committed rip is
   now recorded (net id -> its pre-rip copper); at end of run a casualties-only
   reconciliation (depth 1, no rip authority) runs restore-FIRST semantics:
   verify actually broken, restore the original geometry (collision-aware
   restore_net refuses restores that would short copper routed meanwhile -
   never a blind restore), then re-route the still-broken with the restored
   copper as obstacles; last resort is a piece-level partial restore of the
   non-colliding copper (parity with route.py's #134 last resort).

All imports of routing machinery are lazy (function-level) so this module can
be imported by diff_pair_loop / reroute_loop without cycles.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from terminal_colors import RED, GREEN, RESET


# --------------------------------------------------------------------------
# Part 1: per-pair failure diagnostics
# --------------------------------------------------------------------------

def classify_diff_pair_failure(result: Optional[dict]) -> str:
    """Map a final failed route-result dict to a stable reason code.

    Codes (additive vocabulary - graders match on these strings):
      endpoint-resolution-failure  route returned None (no usable endpoints)
      polarity-swap-denied         P/N mismatch, swap forbidden by policy (#279)
      polarity-mismatch            P/N mismatch, swap attempted but unresolvable
      pn-crossing                  P/N tracks must cross; untangle failed
      connector-graze              emitted geometry grazes committed foreign copper (#165/#246)
      no-escape-path               endpoint escape blocked (probe/setback: 0 search iterations)
      pose-router-failure          PoseRouter centerline search exhausted (#346)
      no-route                     failed with no iterations and no blocked cells
    Call BEFORE the caller pops blocked_cells_* off the result.
    """
    if result is None:
        return 'endpoint-resolution-failure'
    if result.get('polarity_skip'):
        return ('polarity-swap-denied' if result.get('polarity_swap_denied')
                else 'polarity-mismatch')
    if result.get('pn_crossing'):
        return 'pn-crossing'
    if result.get('connector_graze'):
        return 'connector-graze'
    if result.get('probe_blocked'):
        return 'no-escape-path'
    fwd = result.get('iterations_forward', 0)
    bwd = result.get('iterations_backward', 0)
    has_cells = bool(result.get('blocked_cells_forward')
                     or result.get('blocked_cells_backward')
                     or result.get('blocked_cells'))
    if fwd == 0 and bwd == 0 and has_cells:
        # Setback / probe-derived failure: the search never launched - the
        # terminal escape itself is blocked.
        return 'no-escape-path'
    if result.get('iterations', 0) > 0:
        # The coupled centerline is routed by the pose-based A* (PoseRouter);
        # a search that ran and exhausted is a pose-router failure. Explicit
        # code per #346 (these used to be an unreported wall).
        return 'pose-router-failure'
    return 'no-route'


def blocker_names(rippable_blockers, diff_pair_by_net_id, limit: int = 3) -> List[str]:
    """Human names (pair name when applicable) for the top blocking nets."""
    names = []
    for b in rippable_blockers[:limit]:
        if b.net_id in diff_pair_by_net_id:
            names.append(diff_pair_by_net_id[b.net_id][0])
        else:
            names.append(b.net_name)
    return names


def record_pair_diag(state, pair_name: str, **fields):
    """Merge diagnostic fields into a pair's record (most recent wins)."""
    rec = state.pair_diagnostics.setdefault(pair_name, {})
    for k, v in fields.items():
        if v is not None:
            rec[k] = v


def audit_pair_members(pcb_data, diff_pairs_list, tolerance: float = 0.02) -> Dict[str, Dict[str, bool]]:
    """Verify each pair member's connectivity claim against the ACTUAL pad
    connectivity (authoritative union-find, same checker route.py's #189 sweep
    uses) - there are known cases of one member silently incomplete.

    Returns {pair_name: {'p': bool, 'n': bool}} (True = all pads connected).
    Nets with <2 pads are trivially connected.
    """
    from check_connected import check_net_connectivity
    segs_by_net: Dict[int, list] = {}
    vias_by_net: Dict[int, list] = {}
    zones_by_net: Dict[int, list] = {}
    for s in pcb_data.segments:
        segs_by_net.setdefault(s.net_id, []).append(s)
    for v in pcb_data.vias:
        vias_by_net.setdefault(v.net_id, []).append(v)
    for z in getattr(pcb_data, 'zones', []) or []:
        zones_by_net.setdefault(z.net_id, []).append(z)
    audit: Dict[str, Dict[str, bool]] = {}
    for pair_name, pair in diff_pairs_list:
        rec = {}
        for tag, nid in (('p', pair.p_net_id), ('n', pair.n_net_id)):
            pads = pcb_data.pads_by_net.get(nid, [])
            if len(pads) < 2:
                rec[tag] = True
                continue
            r = check_net_connectivity(
                nid, segs_by_net.get(nid, []), vias_by_net.get(nid, []),
                pads, zones_by_net.get(nid, []), tolerance=tolerance)
            rec[tag] = bool(r.get('connected'))
        audit[pair_name] = rec
    return audit


def build_pair_reports(state, diff_pair_ids_to_route, member_audit,
                       skipped_fanout=()) -> List[Dict]:
    """Assemble the per-pair JSON records (additive JSON_SUMMARY key).

    outcome: 'coupled' (both members routed coupled), 'partial' (coupled trunk
    with terminals peeled to the single-ended follow-up), 'deferred' (whole
    pair left for the single-ended follow-up), 'failed', 'skipped' (fanout
    pre-check, #242).
    """
    reports = []
    routed_results = state.routed_results
    sing = state.diff_pair_single_ended_nets
    for pair_name, pair in diff_pair_ids_to_route:
        diag = state.pair_diagnostics.get(pair_name, {})
        aud = member_audit.get(pair_name, {})
        # A member routed by the single-ended fallback is NOT a coupled route:
        # without this exclusion a deferred/failed pair whose members were both
        # connected single-ended would misreport outcome 'coupled'.
        _rr_p = routed_results.get(pair.p_net_id)
        _rr_n = routed_results.get(pair.n_net_id)
        coupled = (_rr_p is not None and _rr_n is not None
                   and not (_rr_p.get('se_member_fallback')
                            or _rr_n.get('se_member_fallback')))
        deferred = pair.p_net_id in sing or pair.n_net_id in sing
        if coupled:
            outcome = 'partial' if deferred else 'coupled'
        elif deferred:
            outcome = 'deferred'
        else:
            outcome = 'failed'
        incomplete = [nm for tag, nm in (('p', pair.p_net_name),
                                         ('n', pair.n_net_name))
                      if not aud.get(tag, True)]
        rep = {
            'pair': pair_name,
            'p_net': pair.p_net_name,
            'n_net': pair.n_net_name,
            'outcome': outcome,
            'failure_reason': (diag.get('reason') or 'unknown'
                               ) if outcome in ('failed', 'deferred') else None,
            'failure_stage': (diag.get('stage')
                              ) if outcome in ('failed', 'deferred') else None,
            'incomplete_members': incomplete,
            # Coupled/partial claim contradicted by actual pad connectivity:
            # the "one member silently incomplete" class.
            'member_audit_mismatch': bool(outcome in ('coupled', 'partial')
                                          and incomplete),
        }
        if diag.get('blocking_nets'):
            rep['blocking_nets'] = diag['blocking_nets']
        if diag.get('casualty'):
            rep['casualty'] = diag['casualty']
        if diag.get('se_fallback'):
            # Per-member single-ended fallback outcome (additive):
            # {'p': 'routed'|'partial'|'failed'|'already-connected', 'n': ...}
            rep['se_fallback'] = diag['se_fallback']
        reports.append(rep)
    for name, p_name, n_name in skipped_fanout:
        reports.append({
            'pair': name, 'p_net': p_name, 'n_net': n_name,
            'outcome': 'skipped', 'failure_reason': 'fanout-self-overlap',
            'failure_stage': 'pre-route',
            'incomplete_members': [p_name, n_name],
            'member_audit_mismatch': False,
        })
    return sorted(reports, key=lambda r: r['pair'])


# --------------------------------------------------------------------------
# Part 1b: single-ended member fallback (Class 2 - BOTH members attempted)
# --------------------------------------------------------------------------

def _member_connected(pcb_data, nid: int) -> bool:
    """Authoritative connectivity of one pair member on the CURRENT board."""
    from check_connected import check_net_connectivity
    pads = pcb_data.pads_by_net.get(nid, [])
    if len(pads) < 2:
        return True
    segs = [s for s in pcb_data.segments if s.net_id == nid]
    vias = [v for v in pcb_data.vias if v.net_id == nid]
    zones = [z for z in (getattr(pcb_data, 'zones', []) or [])
             if z.net_id == nid]
    r = check_net_connectivity(nid, segs, vias, pads, zones, tolerance=0.02)
    return bool(r.get('connected'))


def run_single_ended_member_fallback(state, diff_pairs_list) -> Dict:
    """Single-ended completion pass for non-coupled diff pairs: attempt BOTH
    members (P->P and N->N), engine-side.

    A pair that fails or defers coupled routing is handed to "the single-ended
    follow-up" -- but batch_route_diff_pairs itself never attempted the
    members. A chain (or GUI diff tab) with no downstream single-ended pass
    shipped them unrouted, and downstream churn could route ONE member while
    the other was silently dropped (ulx3s FPDI_D1: FPDI_D1- routed, FPDI_D1+
    shipped at zero copper). This pass gives EVERY disconnected member of
    every pair with NO coupled result one plain single-ended attempt
    (multipoint-aware, no rip-up), records the per-member outcome into the
    pair diagnostics (pair_reports[..]['se_fallback']), and reports honestly.

    Members that fail here keep their copper unchanged and stay listed for the
    downstream single-ended pass (which has the full rip-up arsenal); members
    routed here become already-connected no-ops downstream. DRC-safe: each
    member routes against the standard single-ended obstacle map (all other
    nets' copper priced as obstacles), exactly like the casualty reconcile's
    depth-1 reroute.

    Returns a JSON-ready summary dict ('attempted'/'routed'/'partial'/'failed'
    lists of member net names).
    """
    from routing_context import (build_single_ended_obstacles,
                                 record_single_ended_success)
    from single_ended_routing import (route_net_with_obstacles,
                                      route_multipoint_main,
                                      route_multipoint_taps)
    from connectivity import get_multipoint_net_pads

    summary = {'attempted': [], 'routed': [], 'partial': [], 'failed': []}
    pcb_data, config = state.pcb_data, state.config

    todo = []
    for pair_name, pair in diff_pairs_list:
        if (pair.p_net_id in state.routed_results
                or pair.n_net_id in state.routed_results):
            # A coupled / hybrid / reroute result exists (fully-coupled or
            # 'partial' peeled pairs): the member audit covers those; the
            # fallback owns only pairs with NO coupled copper at all.
            continue
        members = [('p', pair.p_net_id, pair.p_net_name),
                   ('n', pair.n_net_id, pair.n_net_name)]
        if all(_member_connected(pcb_data, nid) for _, nid, _ in members):
            continue
        todo.append((pair_name, pair, members))
    if not todo:
        return summary

    print("\n" + "=" * 60)
    print(f"Single-ended member fallback: {len(todo)} non-coupled pair(s) - "
          f"attempting BOTH members P->P / N->N (no rip-up)")
    print("=" * 60)

    for pair_name, pair, members in todo:
        outcomes = {}
        for tag, nid, nm in members:
            if _member_connected(pcb_data, nid):
                outcomes[tag] = 'already-connected'
                continue
            summary['attempted'].append(nm)
            obstacles, _ = build_single_ended_obstacles(
                state.base_obstacles, pcb_data, config, state.routed_net_ids,
                state.remaining_net_ids, state.all_unrouted_net_ids, nid,
                state.gnd_net_id, state.track_proximity_cache, state.layer_map,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions)
            mp_pads = get_multipoint_net_pads(pcb_data, nid, config)
            if mp_pads:
                res = route_multipoint_main(pcb_data, nid, config, obstacles,
                                            mp_pads)
                if res and not res.get('failed') and res.get('is_multipoint'):
                    tap = route_multipoint_taps(pcb_data, nid, config,
                                                obstacles, res)
                    if tap:
                        res = tap
            else:
                res = route_net_with_obstacles(pcb_data, nid, config, obstacles)
            if res and not res.get('failed'):
                res['se_member_fallback'] = True
                if 'route_length' not in res:
                    from net_queries import calculate_route_length
                    res['route_length'] = calculate_route_length(
                        res.get('new_segments') or [],
                        res.get('new_vias') or [], pcb_data)
                state.results.append(res)
                record_single_ended_success(
                    pcb_data, res, nid, config, state.remaining_net_ids,
                    state.routed_net_ids, state.routed_net_paths,
                    state.routed_results, state.track_proximity_cache,
                    state.layer_map)
                if _member_connected(pcb_data, nid):
                    outcomes[tag] = 'routed'
                    summary['routed'].append(nm)
                    print(f"  {GREEN}SE FALLBACK ROUTED{RESET} {nm} "
                          f"({pair_name} {tag.upper()} member)")
                else:
                    outcomes[tag] = 'partial'
                    summary['partial'].append(nm)
                    print(f"  {RED}SE FALLBACK PARTIAL{RESET} {nm}: copper "
                          f"committed but pad(s) still disconnected")
            else:
                outcomes[tag] = 'failed'
                summary['failed'].append(nm)
                print(f"  {RED}SE FALLBACK FAILED{RESET} {nm} "
                      f"({pair_name} {tag.upper()} member) - left for the "
                      f"downstream single-ended pass")
        record_pair_diag(state, pair_name, se_fallback=outcomes)
    return summary


# --------------------------------------------------------------------------
# Part 2: reconciliation casualty custody
# --------------------------------------------------------------------------

def record_casualty(state, net_id: int, saved_result: dict,
                    ripped_ids: List[int], was_in_results: bool):
    """Custody record for a COMMITTED rip (the ripper's route landed and the
    ripped net was queued for reroute, i.e. its copper is gone from the board
    unless a later reroute succeeds). Keyed by every ripped net id; overwrites
    an older record - the most recent complete geometry is the one to restore.
    """
    if saved_result is None:
        return
    for rid in (ripped_ids or [net_id]):
        state.casualty_custody[rid] = (net_id, saved_result,
                                       list(ripped_ids or [net_id]),
                                       was_in_results)


def run_casualty_reconcile(state) -> Dict:
    """Casualties-only end-of-run reconciliation (final_reconcile=
    'casualties-only', depth 1 - no recursive rip authority).

    Restore-first semantics, never a blind restore:
      A. verify each casualty is ACTUALLY still broken (absent from
         routed_results - a later reroute may have already recovered it);
      B. restore the original pre-rip copper via the collision-aware
         restore_net (it refuses a restore that would short copper routed
         while the net was ripped, so restored copper can never collide with
         newly routed copper);
      C. re-route the still-broken (restore refused) with the restored copper
         already committed as obstacles - one plain attempt, no rip-up;
      D. last resort: piece-level partial restore of the non-colliding copper
         (parity with route.py's #134 last resort) - a partial net beats zero.

    Returns a JSON-ready summary dict.
    """
    summary = {'attempted': 0, 'restored': [], 'rerouted': [],
               'partial': [], 'unrecovered': []}
    custody = getattr(state, 'casualty_custody', None)
    if not custody:
        return summary

    pcb_data, config = state.pcb_data, state.config

    def _label(net_id):
        if net_id in state.diff_pair_by_net_id:
            return state.diff_pair_by_net_id[net_id][0]
        net = pcb_data.nets.get(net_id)
        return net.name if net else f"net_{net_id}"

    def _pair_name(net_id):
        info = state.diff_pair_by_net_id.get(net_id)
        return info[0] if info else None

    # A: verify actually broken; dedupe pair halves sharing one custody record.
    broken, seen = [], set()
    for rid in sorted(custody):
        if rid in state.routed_results:
            continue  # recovered by a later reroute - not a casualty
        net_id, saved, ripped_ids, was_in = custody[rid]
        if id(saved) in seen:
            continue
        seen.add(id(saved))
        broken.append((net_id, saved, ripped_ids, was_in))
    if not broken:
        return summary
    summary['attempted'] = len(broken)

    print("\n" + "=" * 60)
    print(f"Final reconciliation (casualties-only, depth 1): {len(broken)} "
          f"net(s)/pair(s) ripped during routing never re-routed")
    print("=" * 60)

    from rip_up_reroute import restore_net

    # B: restore-FIRST (collision-aware; refused restores stay ripped and are
    # stashed for the piece-level last resort).
    still_broken = []
    for net_id, saved, ripped_ids, was_in in broken:
        restore_net(net_id, saved, ripped_ids, was_in, pcb_data,
                    state.routed_net_ids, state.routed_net_paths,
                    state.routed_results, state.diff_pair_by_net_id,
                    state.remaining_net_ids, state.results, config,
                    state.track_proximity_cache, state.layer_map,
                    state.working_obstacles, state.net_obstacles_cache,
                    state.ripped_route_layer_costs,
                    state.ripped_route_via_positions,
                    refused_sink=state.collision_refused_net_ids)
        if all(r in state.routed_results for r in (ripped_ids or [net_id])):
            name = _label(net_id)
            print(f"  {GREEN}RESTORED{RESET} {name}: original pre-rip copper "
                  f"re-instated (casualty custody)")
            summary['restored'].append(name)
            pn = _pair_name(net_id)
            if pn:
                record_pair_diag(state, pn, casualty='restored-original')
        else:
            still_broken.append((net_id, saved, ripped_ids, was_in))

    # C: depth-1 reroute of the still-broken, with all restored copper as
    # obstacles. No rip-up (no recursive rip authority).
    for net_id, saved, ripped_ids, was_in in still_broken:
        name = _label(net_id)
        rerouted = False
        pair_info = state.diff_pair_by_net_id.get(net_id)
        if pair_info is not None:
            pair_name, pair = pair_info
            from routing_context import (build_diff_pair_obstacles,
                                         record_diff_pair_success)
            from diff_pair_routing import route_diff_pair_with_obstacles
            from layer_swap_fallback import add_own_stubs_as_obstacles_for_diff_pair
            from polarity_swap import apply_polarity_swap
            obstacles, unrouted_stubs = build_diff_pair_obstacles(
                state.diff_pair_base_obstacles, pcb_data, config,
                state.routed_net_ids, state.remaining_net_ids,
                state.all_unrouted_net_ids, pair.p_net_id, pair.n_net_id,
                state.gnd_net_id, state.track_proximity_cache, state.layer_map,
                state.diff_pair_extra_clearance,
                add_own_stubs_func=add_own_stubs_as_obstacles_for_diff_pair,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions)
            res = route_diff_pair_with_obstacles(
                pcb_data, pair, config, obstacles, state.base_obstacles,
                unrouted_stubs)
            if res and not res.get('failed') and not res.get('probe_blocked'):
                state.results.append(res)
                apply_polarity_swap(pcb_data, res, state.pad_swaps, pair_name,
                                    state.polarity_swapped_pairs)
                record_diff_pair_success(
                    pcb_data, res, pair, pair_name, config,
                    state.remaining_net_ids, state.routed_net_ids,
                    state.routed_net_paths, state.routed_results,
                    state.diff_pair_by_net_id, state.track_proximity_cache,
                    state.layer_map)
                record_pair_diag(state, pair_name, casualty='rerouted')
                rerouted = True
        else:
            from routing_context import (build_single_ended_obstacles,
                                         record_single_ended_success)
            from single_ended_routing import route_net_with_obstacles
            obstacles, _ = build_single_ended_obstacles(
                state.base_obstacles, pcb_data, config, state.routed_net_ids,
                state.remaining_net_ids, state.all_unrouted_net_ids, net_id,
                state.gnd_net_id, state.track_proximity_cache, state.layer_map,
                ripped_route_layer_costs=state.ripped_route_layer_costs,
                ripped_route_via_positions=state.ripped_route_via_positions)
            res = route_net_with_obstacles(pcb_data, net_id, config, obstacles)
            if res and not res.get('failed'):
                state.results.append(res)
                record_single_ended_success(
                    pcb_data, res, net_id, config, state.remaining_net_ids,
                    state.routed_net_ids, state.routed_net_paths,
                    state.routed_results, state.track_proximity_cache,
                    state.layer_map)
                rerouted = True
        if rerouted:
            print(f"  {GREEN}REROUTED{RESET} {name}: casualty re-routed with "
                  f"restored copper as obstacles")
            summary['rerouted'].append(name)
            continue

        # D: last resort - piece-level partial restore of non-colliding copper
        # (route.py #134 parity). Piece-wise collision check against the
        # settled board, so nothing restored can short copper routed meanwhile.
        from rip_up_reroute import _saved_route_collides
        from pcb_modification import add_route_to_pcb_data
        stash = getattr(pcb_data, '_refused_saved_134', {}) or {}
        src = stash.get(net_id, saved)
        keep_segs = [sg for sg in (src.get('new_segments') or [])
                     if not _saved_route_collides(
                         {'new_segments': [sg], 'new_vias': []},
                         pcb_data, ripped_ids, config.clearance)]
        keep_vias = [v for v in (src.get('new_vias') or [])
                     if not _saved_route_collides(
                         {'new_segments': [], 'new_vias': [v]},
                         pcb_data, ripped_ids, config.clearance)]
        pn = _pair_name(net_id)
        if keep_segs or keep_vias:
            dropped = (len(src.get('new_segments') or []) - len(keep_segs)
                       + len(src.get('new_vias') or []) - len(keep_vias))
            pruned = dict(src)
            pruned['new_segments'] = keep_segs
            pruned['new_vias'] = keep_vias
            pruned['partial_restore_134'] = True
            add_route_to_pcb_data(pcb_data, pruned,
                                  debug_lines=config.debug_lines)
            state.results.append(pruned)
            print(f"  {RED}PARTIAL{RESET} {name}: reroute failed; restored "
                  f"{len(keep_segs)} segment(s) + {len(keep_vias)} via(s) of "
                  f"its pre-rip route (dropped {dropped} colliding piece(s)); "
                  f"net remains PARTIAL for a later pass")
            summary['partial'].append(name)
            if pn:
                record_pair_diag(state, pn, reason='ripped-not-restored',
                                 stage='casualty-reconcile',
                                 casualty='partial-restore')
        else:
            print(f"  {RED}UNRECOVERED{RESET} {name}: restore collides and "
                  f"reroute failed; net ships unrouted")
            summary['unrecovered'].append(name)
            if pn:
                record_pair_diag(state, pn, reason='ripped-not-restored',
                                 stage='casualty-reconcile',
                                 casualty='unrecovered')
    return summary
