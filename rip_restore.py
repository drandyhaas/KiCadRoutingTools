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


def _copper_conflicts(pcb_data: PCBData, config: GridRouteConfig,
                      own_ids: set, segments, vias) -> bool:
    """True if any candidate segment/via violates clearance against any
    foreign segment/via currently on the board. Pads are not re-checked:
    the saved copper was DRC-clean against them before the rip and pads do
    not move; only copper routed SINCE the rip can conflict."""
    from geometry_utils import point_to_segment_distance
    clr_of = (config.obstacle_clearance
              if hasattr(config, 'obstacle_clearance') else lambda n: config.clearance)
    for cand in segments:
        c_clr_half = cand.width / 2
        for s in pcb_data.segments:
            if s.net_id in own_ids or s.layer != cand.layer:
                continue
            need = c_clr_half + s.width / 2 + max(clr_of(cand.net_id), clr_of(s.net_id))
            if any(point_to_segment_distance(px, py, s.start_x, s.start_y,
                                             s.end_x, s.end_y) < need
                   for px, py in _seg_points(cand)):
                return True
        for v in pcb_data.vias:
            if v.net_id in own_ids:
                continue
            need = c_clr_half + v.size / 2 + max(clr_of(cand.net_id), clr_of(v.net_id))
            if any(math.hypot(px - v.x, py - v.y) < need for px, py in _seg_points(cand)):
                return True
    for cv in vias:
        for v in pcb_data.vias:
            if v.net_id in own_ids:
                continue
            need = cv.size / 2 + v.size / 2 + max(clr_of(cv.net_id), clr_of(v.net_id))
            if math.hypot(cv.x - v.x, cv.y - v.y) < need:
                return True
        for s in pcb_data.segments:
            if s.net_id in own_ids:
                continue
            need = cv.size / 2 + s.width / 2 + max(clr_of(cv.net_id), clr_of(s.net_id))
            if point_to_segment_distance(cv.x, cv.y, s.start_x, s.start_y,
                                         s.end_x, s.end_y) < need:
                return True
    return False


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
    copper is conflict-free (caller must then run restore_net, which pops
    the registry and does all bookkeeping); 'stub' when only the escape
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
        return 'full'

    stub_segs, stub_vias = _stub_subset(pcb_data, net_id, segments, vias)
    kept_s = [s for s in stub_segs
              if not _copper_conflicts(pcb_data, config, own, [s], [])]
    kept_v = [v for v in stub_vias
              if not _copper_conflicts(pcb_data, config, own, [], [v])]
    if not kept_s and not kept_v:
        return None
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
    print(f"  RIP-RESTORE (#468): {name} kept its escape stub "
          f"({len(kept_s)} seg(s), {len(kept_v)} via(s)) -- full restore "
          f"would short against copper routed since the rip")
    return 'stub'
