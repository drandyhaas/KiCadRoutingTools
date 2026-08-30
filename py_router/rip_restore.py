"""Terminal-failure restore for rip victims (#468).

A net ripped as a blocker and never successfully rerouted used to ship
STRIPPED: the rip removed its routed copper (including the consumed fanout
escape), the reroute queue's terminal-failure path did nothing, and the
board ended with LESS copper than it started with -- a bare BGA ball where
step 1 had placed an escape (ottercast U1.E2). The "ripped => rerouted"
custody invariant needs its third leg: ripped => rerouted OR RESTORED.

Blind restore is not safe: the ripper's retry may have routed through the
freed corridor (that is WHY the #134 pass leaves such nets ripped). So:

  1. conflict-check the saved copper against everything now on the board;
  2. clean  -> the caller performs a FULL restore_net (the net returns to
     its pre-rip, fully-routed state -- strictly better than failed);
  3. dirty  -> restore the SAFE SUBSET: the escape-stub segments/vias at
     the net's pads that individually clear current copper, so the pad
     keeps its landing site for later passes (pcb_data-only insertion; an
     unrouted net's stubs are stamped per-prepare from pcb_data, so no
     direct obstacle-map surgery and no #309 ref-count exposure).

The saved payload rides pcb_data._rip_saved, recorded by rip_up_net for
every rip and popped by restore_net -- so any net still ripped at terminal
failure has its pre-rip copper available regardless of which ladder ripped
it (singles and diff pairs alike).
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from kicad_parser import PCBData
from routing_config import GridRouteConfig


def _seg_points(s, step_mm: float = 0.2):
    n = max(2, int(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) / step_mm) + 1)
    for i in range(n + 1):
        t = i / n
        yield (s.start_x + (s.end_x - s.start_x) * t,
               s.start_y + (s.end_y - s.start_y) * t)


def _conflict_sweep(pcb_data: PCBData, config: GridRouteConfig,
                    own_ids: set, segments, vias, collect: bool = False):
    """Owners (net_ids) of board copper violating clearance against the
    candidate segments/vias. collect=False returns at the FIRST hit (the
    boolean-speed path `_copper_conflicts` wraps); collect=True sweeps the
    whole board and returns every conflicting owner. Pads are not
    re-checked: the saved copper was DRC-clean against them before the rip
    and pads do not move; only copper routed SINCE the rip can conflict."""
    from geometry_utils import point_to_segment_distance
    clr_of = (config.obstacle_clearance
              if hasattr(config, 'obstacle_clearance') else lambda n: config.clearance)

    def _pair_clr(a_net, b_net, layer=None):
        # KiCad pairwise max of the two nets' resolutions; #498: a .kicad_dru
        # rule for the layer the coppers meet on replaces that (stack max for
        # via-via, where they meet on every layer).
        v = max(clr_of(a_net), clr_of(b_net))
        if layer is not None and hasattr(config, 'layer_clearance'):
            return config.layer_clearance(layer, v)
        if layer is None and hasattr(config, 'stack_clearance'):
            return config.stack_clearance(v)
        return v

    def _track_pair_clr(a_net, b_net, layer):
        # A track-scoped DRU rule raises the SEG-SEG requirement only (#735).
        #
        # NOT `kicad_dru.track_pair_clearance`, and do not unify them (#735).
        # That one is PAIR-EXACT and needs both nets' class memberships; this
        # one reads `config.track_clearances`, the ROUTER's per-obstacle-net
        # OVER-approximation, because that is the map the copper being
        # restored was routed against. Substituting the exact resolver here
        # would price a restore BELOW what the router stamped for it.
        v = _pair_clr(a_net, b_net, layer)
        if hasattr(config, 'track_obstacle_clearance'):
            v = max(config.track_obstacle_clearance(a_net, v),
                    config.track_obstacle_clearance(b_net, v))
        return v

    owners: set = set()
    for cand in segments:
        c_clr_half = cand.width / 2
        for s in pcb_data.segments:
            if s.net_id in own_ids or s.layer != cand.layer \
                    or s.net_id in owners:
                continue
            need = c_clr_half + s.width / 2 + _track_pair_clr(cand.net_id, s.net_id, cand.layer)
            if any(point_to_segment_distance(px, py, s.start_x, s.start_y,
                                             s.end_x, s.end_y) < need
                   for px, py in _seg_points(cand)):
                owners.add(s.net_id)
                if not collect:
                    return owners
        for v in pcb_data.vias:
            if v.net_id in own_ids or v.net_id in owners:
                continue
            need = c_clr_half + v.size / 2 + _pair_clr(cand.net_id, v.net_id, cand.layer)
            if any(math.hypot(px - v.x, py - v.y) < need for px, py in _seg_points(cand)):
                owners.add(v.net_id)
                if not collect:
                    return owners
    for cv in vias:
        for v in pcb_data.vias:
            if v.net_id in own_ids or v.net_id in owners:
                continue
            need = cv.size / 2 + v.size / 2 + _pair_clr(cv.net_id, v.net_id)
            if math.hypot(cv.x - v.x, cv.y - v.y) < need:
                owners.add(v.net_id)
                if not collect:
                    return owners
        for s in pcb_data.segments:
            if s.net_id in own_ids or s.net_id in owners:
                continue
            need = cv.size / 2 + s.width / 2 + _pair_clr(cv.net_id, s.net_id, s.layer)
            if point_to_segment_distance(cv.x, cv.y, s.start_x, s.start_y,
                                         s.end_x, s.end_y) < need:
                owners.add(s.net_id)
                if not collect:
                    return owners
    return owners


def _copper_conflicts(pcb_data: PCBData, config: GridRouteConfig,
                      own_ids: set, segments, vias) -> bool:
    """True if any candidate segment/via violates clearance against any
    foreign segment/via currently on the board."""
    return bool(_conflict_sweep(pcb_data, config, own_ids, segments, vias,
                                collect=False))


def conflict_owner_ids(pcb_data: PCBData, config: GridRouteConfig,
                       own_ids: set, segments, vias) -> set:
    """Every net owning board copper that blocks a full restore of the
    candidate copper (#622 victim-priority restore)."""
    return _conflict_sweep(pcb_data, config, own_ids, segments, vias,
                           collect=True)


def _stub_subset(pcb_data: PCBData, net_id: int, segments, vias,
                 max_len_mm: float = 2.0):
    """The escape-stub portion of the saved copper: short segments starting
    at one of the net's pad centers, walked one hop, plus vias on their
    far ends."""
    pads = pcb_data.pads_by_net.get(net_id, [])
    if not pads:
        return [], []
    pad_pts = [(p.global_x, p.global_y) for p in pads]
    stub_segs = []
    ends = set()
    for s in segments:
        if math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) > max_len_mm:
            continue
        for (px, py) in pad_pts:
            if (abs(s.start_x - px) < 1e-3 and abs(s.start_y - py) < 1e-3):
                stub_segs.append(s)
                ends.add((round(s.end_x, 3), round(s.end_y, 3)))
                break
            if (abs(s.end_x - px) < 1e-3 and abs(s.end_y - py) < 1e-3):
                stub_segs.append(s)
                ends.add((round(s.start_x, 3), round(s.start_y, 3)))
                break
    stub_vias = [v for v in vias
                 if (round(v.x, 3), round(v.y, 3)) in ends
                 or any(abs(v.x - px) < 1e-3 and abs(v.y - py) < 1e-3
                        for px, py in pad_pts)]
    return stub_segs, stub_vias


def try_terminal_restore(pcb_data: PCBData, config: GridRouteConfig,
                         net_id: int, working_obstacles=None,
                         net_obstacles_cache=None) -> Optional[str]:
    """At a rip victim's TERMINAL reroute failure: 'full' when the saved
    copper is conflict-free AND grades connected (caller must then run
    restore_net, which pops the registry and does all bookkeeping);
    'full_open' when it is conflict-free but the restored net would still
    have disconnected pads -- the caller should restore it (more copper
    than nothing) WITHOUT counting a success; 'stub' when only the escape
    subset could be re-inserted (done here, pcb_data-only); None when
    nothing was recoverable."""
    reg = getattr(pcb_data, '_rip_saved', None)
    payload = reg.get(net_id) if reg else None
    if payload is None:
        return None
    saved_result, ripped_ids, _was = payload
    segments = saved_result.get('new_segments') or []
    vias = saved_result.get('new_vias') or []
    if not segments and not vias:
        return None
    own = set(ripped_ids) | {net_id}

    if not _copper_conflicts(pcb_data, config, own, segments, vias):
        # Conflict-free is necessary, not sufficient (run-7 E2): the saved
        # payload can itself be partial -- recorded off a net that was already
        # broken, or one whose copper never covered every pad -- and restoring
        # it while claiming success ships an open net reported as routed.
        # Grade the POST-RESTORE state (the net's copper still on the board
        # plus the payload) with the authoritative union-find, per member net
        # (a pair payload carries both ids in ripped_ids).
        try:
            from check_connected import (check_net_connectivity,
                                         net_break_within_outlines)
            for _nid in sorted(own):
                pads = pcb_data.pads_by_net.get(_nid, [])
                if len(pads) < 2:
                    continue
                segs_all = ([s for s in pcb_data.segments if s.net_id == _nid]
                            + [s for s in segments if s.net_id == _nid])
                vias_all = ([v for v in pcb_data.vias if v.net_id == _nid]
                            + [v for v in vias if v.net_id == _nid])
                zones = [z for z in getattr(pcb_data, 'zones', []) or []
                         if z.net_id == _nid]
                _r = check_net_connectivity(_nid, segs_all, vias_all, pads,
                                            zones, tolerance=0.02,
                                            pcb_data=pcb_data)
                _broken, _dp = net_break_within_outlines(pcb_data, _r)
                if _broken and _dp:
                    return 'full_open'
        except Exception as _e:
            # Grading must never turn a restorable net into a strip, so a
            # grader failure still restores the copper -- but as
            # 'full_open', not 'full' (review finding F5): 'full' counts a
            # SUCCESS and bypasses the coverage gate, so a systematic
            # grader failure (an import error in check_connected) would
            # silently reinstate the run-7 E2 over-claim across every
            # terminal restore of the run. 'full_open' restores the same
            # copper, counts the net failed, and keeps it in the disturbed
            # set -- pessimistic and honest.
            print(f"  WARNING: rip-restore connectivity grading failed for net "
                  f"{net_id} ({type(_e).__name__}: {_e}); restoring the copper "
                  f"as 'full_open' (counted FAILED, unverified) rather than "
                  f"claiming an unchecked success")
            return 'full_open'
        return 'full'

    stub_segs, stub_vias = _stub_subset(pcb_data, net_id, segments, vias)
    kept_s = [s for s in stub_segs
              if not _copper_conflicts(pcb_data, config, own, [s], [])]
    kept_v = [v for v in stub_vias
              if not _copper_conflicts(pcb_data, config, own, [], [v])]
    if not kept_s and not kept_v:
        return None
    # IDEMPOTENCE. The caller does NOT pop the registry on the 'stub' path
    # (only 'full'/'full_open' go through restore_net, which pops), so a net
    # that terminally fails again is re-offered the SAME saved payload and
    # this would extend pcb_data with copper that is already on the board --
    # measured on ulx3s as 4 stub restores of one net, and duplicate object
    # entries in the BOARD ledger. A duplicate entry is never harmless: the
    # write model holds the object once (so the ledger reports phantom
    # board-only copper), obstacles get double-stamped (#208/#309), and
    # gates that treat the list as a node set are defeated (#195).
    # Same-net copper is exempt from _copper_conflicts via `own`, so the
    # conflict check above cannot catch this -- identity is what matters.
    _present_s = {id(s) for s in pcb_data.segments}
    _present_v = {id(v) for v in pcb_data.vias}
    kept_s = [s for s in kept_s if id(s) not in _present_s]
    kept_v = [v for v in kept_v if id(v) not in _present_v]
    if not kept_s and not kept_v:
        return 'stub'   # already restored by an earlier terminal failure
    # Mirror rip_up_net's own cache maintenance (remove entry -> mutate
    # pcb_data -> recompute -> add entry): the restored stubs become map
    # obstacles for every later net this run, the cache entry keeps
    # mirroring the board, and every map op is a complete-entry add/remove
    # -- #309 ref-counts balanced by construction.
    if working_obstacles is not None and net_obstacles_cache is not None \
            and net_id in net_obstacles_cache:
        from obstacle_cache import (add_net_obstacles_from_cache,
                                    precompute_net_obstacles,
                                    remove_net_obstacles_from_cache)
        remove_net_obstacles_from_cache(working_obstacles,
                                        net_obstacles_cache[net_id])
        pcb_data.segments.extend(kept_s)
        pcb_data.vias.extend(kept_v)
        net_obstacles_cache[net_id] = precompute_net_obstacles(
            pcb_data, net_id, config)
        add_net_obstacles_from_cache(working_obstacles,
                                     net_obstacles_cache[net_id])
    else:
        pcb_data.segments.extend(kept_s)
        pcb_data.vias.extend(kept_v)
    name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else str(net_id)
    print(f"  RIP-RESTORE (#468): {name} remains UNROUTED -- kept only its "
          f"escape stub ({len(kept_s)} seg(s), {len(kept_v)} via(s)) so the "
          f"pads keep their landing sites; full restore would short against "
          f"copper routed since the rip")
    return 'stub'
