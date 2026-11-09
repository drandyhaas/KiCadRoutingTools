#!/usr/bin/env python3
"""Leg-level (partial) rip-up -- rip only the branch that is actually in the way (#510).

When `route.py` rips a blocker to make room it tears out the ENTIRE net, even
when only one leg of a multi-tap net ever obstructed the route. Measured on the
corpus, every rip on five sampled boards was on a multipoint net, and ~65-85% of
the torn-out copper belonged to legs that were never in the way.

This module answers the one question that makes a partial rip possible:

    given the cells that blocked the route, WHICH of the blocker net's copper
    is actually responsible?

The router already computes those cells (`blocked_cells`, from the A* frontier)
and then aggregates them away to a per-net count in `analyze_frontier_blocking`
/ `_identify_blocking_obstacles`. That aggregation is the only reason the rip has
to be whole-net -- the geometry is still right there in `pcb_data`.

WHY GEOMETRY, NOT RECORDED LEG IDS. Phase 3 already slices each tap's copper out
of the flat list (`completed_result['new_segments'][len(lm_segments):]`), so leg
boundaries exist at commit time and could be recorded. But length matching
REPLACES main-path segments after the fact, so recorded ids would have to be
maintained through every later mutation. Deriving the branch from the net's
current copper is immune to that -- it reads what is actually on the board now.

BRANCH, NOT BARE SEGMENTS. Removing only the offending segments would free the
corridor but leave dangling stubs mid-net. Instead we walk the net's segment
graph out from each offending segment and take the whole maximal chain whose
interior points have degree 2, stopping at junctions (degree != 2) and via
locations. That is exactly one "leg"/tap of the tree: other branches keep their
copper, and the re-route reconnects the orphaned component.

The re-route needs no new machinery: `route_multipoint_main` already derives
`pad_components` from EXISTING copper (`get_terminal_component_info`) and routes
only the MST edges needed to join disconnected components -- it even short-
circuits with "All pads already connected by existing copper". A partially
ripped net re-queued for routing therefore reconnects just the missing leg.

Gated by KICAD_LEG_RIP=1; default OFF, so the whole-net path is unchanged.
"""
import os
from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple

# Env-gated (the repo convention for experimental routing knobs): inert unless
# set, so the default path is bit-identical to before.
LEG_RIP_ENABLED = os.environ.get('KICAD_LEG_RIP') == '1'

# Selection rule: a leg whose copper sits in the BLOCKING CORRIDOR is pulled out
# WHOLE. Never bare segments -- cutting a leg mid-span frees the corridor but
# strands a fragment (an orphan stub) the reconnect then has to work around,
# which is not what "rip the tap in the way" means.
#
# A wider-catchment variant (every leg NEAR the blockage, 3x radius) was built and
# measured on rp2350_dev, and LOST on both axes, so it is not kept as an option:
#   * it BROKE CONNECTIVITY -- 1 net left unconnected -- for 4 extra points of
#     copper saving (-73% vs -69%): it rips legs that were never in the way and
#     the reconnect does not recover them all;
#   * it was pathologically SLOW -- 1204ms inside ONE call vs 327ms worst case,
#     because tripling r makes the (2r+1)^2 dilation ~8x bigger.

_PT = 4          # decimals for endpoint identity (nm-grid pad coords round clean)


def _pt(x: float, y: float, layer: str) -> Tuple[float, float, str]:
    return (round(x, _PT), round(y, _PT), layer)


def _dilate(cells: Set[Tuple[int, int]], r: int) -> Set[Tuple[int, int]]:
    """Grow a blocked-cell set by r cells, ONCE per call.

    PERFORMANCE: this runs inside the rip loop, so it must not become the cost it
    is trying to save. The naive form -- for each segment, for each step along it,
    scan a (2r+1)^2 neighbourhood -- is O(segments * steps * r^2) PER RIP, and r
    is ~5 at a 0.05mm grid, i.e. 121 lookups per step. Dilating the corridor once
    up front makes every later test a single O(1) set membership, turning the
    per-rip cost into O(cells*r^2 + total_segment_length). The dilation is shared
    across every segment and via of the net.
    """
    if r <= 0:
        return set(cells)
    offs = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)]
    out = set()
    for gx, gy in cells:
        for dx, dy in offs:
            out.add((gx + dx, gy + dy))
    return out


def _seg_hits_cells(seg, dilated_by_layer: Dict[str, Set[Tuple[int, int]]],
                    coord) -> bool:
    """True if `seg` passes through the DILATED corridor on its layer.

    Walks the segment in grid steps rather than testing endpoints only: a long
    track crossing the corridor has both ends far away from it. One set lookup
    per step (the radius is already baked into `dilated_by_layer`).
    """
    cells = dilated_by_layer.get(seg.layer)
    if not cells:
        return False
    gx0, gy0 = coord.to_grid(seg.start_x, seg.start_y)
    gx1, gy1 = coord.to_grid(seg.end_x, seg.end_y)
    steps = max(abs(gx1 - gx0), abs(gy1 - gy0), 1)
    if (gx0, gy0) in cells or (gx1, gy1) in cells:
        return True                      # cheap ends-first check, common case
    for i in range(1, steps):
        if (gx0 + (gx1 - gx0) * i // steps, gy0 + (gy1 - gy0) * i // steps) in cells:
            return True
    return False


def _build_branches(segs) -> List[List]:
    """Partition a net's segments into maximal branches.

    A branch is a maximal chain whose INTERIOR endpoints have degree 2 within
    the net. Points of degree 1 (a pad/dangling end) or >=3 (a tee) terminate a
    branch, so each tap of a multipoint tree comes out as its own branch.
    Endpoints are matched per LAYER, so a via (layer change) also terminates a
    branch -- which is what we want: a leg that changes layer is two branches,
    and ripping the blocking half is still strictly better than the whole net.
    """
    deg = defaultdict(int)
    at_point = defaultdict(list)
    for s in segs:
        a, b = _pt(s.start_x, s.start_y, s.layer), _pt(s.end_x, s.end_y, s.layer)
        deg[a] += 1
        deg[b] += 1
        at_point[a].append(s)
        at_point[b].append(s)

    branches, seen = [], set()
    for s in segs:
        if id(s) in seen:
            continue
        chain = [s]
        seen.add(id(s))
        # Extend from both ends while the joining point is a plain degree-2 pass-through.
        for end in (_pt(s.start_x, s.start_y, s.layer), _pt(s.end_x, s.end_y, s.layer)):
            pt, prev = end, s
            while deg.get(pt, 0) == 2:
                nxt = next((o for o in at_point[pt] if id(o) not in seen), None)
                if nxt is None:
                    break
                chain.append(nxt)
                seen.add(id(nxt))
                a = _pt(nxt.start_x, nxt.start_y, nxt.layer)
                b = _pt(nxt.end_x, nxt.end_y, nxt.layer)
                pt = b if a == pt else a
                prev = nxt
        branches.append(chain)
    return branches


def select_blocking_branch(pcb_data, net_id: int, blocked_cells, config,
                           verbose: bool = False) -> Optional[List]:
    """Segments of `net_id` forming the branch(es) that block `blocked_cells`.

    Returns None when the rip should stay whole-net:
      * no blocked cells / no copper to reason about,
      * nothing localizable (no segment near the corridor),
      * the blocking branches ARE the whole net (nothing saved).
    Returning None -- not an empty list -- keeps the caller's fallback explicit.
    """
    if not blocked_cells:
        return None
    # A net's copper art is not a branch of its route: nothing lays it again.
    segs = [s for s in pcb_data.segments
            if s.net_id == net_id and not getattr(s, 'graphic', False)]
    if len(segs) < 2:
        return None                      # nothing to subset

    from routing_config import GridCoord
    coord = GridCoord(config.grid_step)
    # Same reach the obstacle stamp uses: half a track + clearance, plus a cell
    # of slack so a track that merely grazes the corridor still counts.
    radius_cells = max(1, coord.to_grid_dist_safe(
        config.track_width / 2.0 + config.clearance) + 1)

    import time as _time
    _t0 = _time.perf_counter()
    cells_by_layer: Dict[str, Set[Tuple[int, int]]] = defaultdict(set)
    layers = list(config.layers)
    for c in blocked_cells:
        if len(c) < 3:
            continue
        gx, gy, li = c[0], c[1], c[2]
        name = layers[li] if isinstance(li, int) and 0 <= li < len(layers) else li
        cells_by_layer[name].add((gx, gy))
    if not cells_by_layer:
        return None
    # Dilate ONCE; every segment/via test below is then an O(1) lookup.
    dilated = {k: _dilate(v, radius_cells) for k, v in cells_by_layer.items()}

    # A net can block through its VIAS as well as its tracks -- a via barrel sits
    # on every layer, so it walls off a corridor no track of this net goes near.
    # Segment-only testing then reports "no branch near the corridor" and declines
    # on exactly the big multi-tap nets where the payoff is largest (rp2350_dev
    # net 6: 200 segments, 53 branches, 0 hits). Treat a segment as blocking if it
    # touches one of this net's vias that sits in the corridor.
    via_pts = set()
    for v in getattr(pcb_data, 'vias', []):
        if v.net_id != net_id:
            continue
        gx, gy = coord.to_grid(v.x, v.y)
        if any((gx, gy) in cells for cells in dilated.values()):
            via_pts.add((round(v.x, _PT), round(v.y, _PT)))

    def _blocks(s):
        if _seg_hits_cells(s, dilated, coord):
            return True
        return bool(via_pts) and (
            (round(s.start_x, _PT), round(s.start_y, _PT)) in via_pts
            or (round(s.end_x, _PT), round(s.end_y, _PT)) in via_pts)

    branches = _build_branches(segs)
    keep: List = []
    nhit = 0
    for br in branches:
        if any(_blocks(s) for s in br):
            keep.extend(br)
            nhit += 1
    # Always report the DECISION, not just the wins: a silent decline is
    # indistinguishable from "the feature is off", which is how a no-op change
    # gets mistaken for a null A/B result.
    if verbose:
        why = ("ok" if keep and len(keep) < len(segs)
               else "DECLINED: nothing near the corridor" if not keep
               else "DECLINED: blocking copper IS the whole net")
        print(f"      leg-rip: net {net_id}: {len(segs)} seg in "
              f"{len(branches)} branch(es), {nhit} blocking"
              f"{f', {len(via_pts)} via(s) in corridor' if via_pts else ''}"
              f" -> {len(keep)} seg | {why} | {1000*(_time.perf_counter()-_t0):.1f}ms")
    if not keep or len(keep) >= len(segs):
        return None                      # not localizable, or it IS the whole net
    return keep
