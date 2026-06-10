"""
Multi-point differential pair routing.

A diff pair cannot tap onto the middle of an existing pair of tracks - the
P/N tracks of the tapping leg would have to cross the existing pair. So
multi-point pairs (3+ pad-pair terminals, e.g. connector -> termination
resistor -> IC pins) are routed as a CHAIN of terminals:

- Each terminal (a P pad + N pad on the same component) has an escape axis
  perpendicular to its pad axis, with two usable sides.
- A terminal can host at most two legs, one out each side. A continuation
  leg leaves on the OPPOSITE side from the leg that arrived, so the chain
  passes "through" the pads without crossing.
- Polarity is resolved per leg geometrically (by choosing the side at the
  fresh terminal). Pad swaps are never used: a swap at a shared terminal
  would break the already-routed leg.
"""

import math
from itertools import permutations
from typing import List, Optional, Tuple

from kicad_parser import PCBData, Pad
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from diff_pair_routing import (
    route_diff_pair_with_obstacles, get_pair_end_direction, _pn_tracks_cross
)
from layer_swap_fallback import add_own_stubs_as_obstacles_for_diff_pair
from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from polarity_swap import apply_polarity_swap, undo_polarity_swap
from routing_context import build_diff_pair_obstacles


def _segments_properly_cross(a, b, eps: float = 1e-6) -> bool:
    """True if segments a and b cross at an interior point.

    Shared endpoints and endpoint touches do NOT count: chain legs meet at
    the same terminal pads, so they legitimately touch there - only a real
    crossing (the loop-around failure mode of issue #56) is an error.
    """
    for (x, y) in ((a.start_x, a.start_y), (a.end_x, a.end_y)):
        for (u, v) in ((b.start_x, b.start_y), (b.end_x, b.end_y)):
            if abs(x - u) < eps and abs(y - v) < eps:
                return False

    def cross(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    d1 = cross(b.start_x, b.start_y, b.end_x, b.end_y, a.start_x, a.start_y)
    d2 = cross(b.start_x, b.start_y, b.end_x, b.end_y, a.end_x, a.end_y)
    d3 = cross(a.start_x, a.start_y, a.end_x, a.end_y, b.start_x, b.start_y)
    d4 = cross(a.start_x, a.start_y, a.end_x, a.end_y, b.end_x, b.end_y)
    return ((d1 > eps) != (d2 > eps)) and ((d3 > eps) != (d4 > eps)) and \
        min(abs(d1), abs(d2), abs(d3), abs(d4)) > eps


def _crosses_committed_legs(new_segments, committed_segments) -> bool:
    """True if any new-leg segment properly crosses a previously committed
    leg's segment on the same layer.

    The obstacle map alone cannot prevent this: corridor exemptions at a
    shared terminal deliberately unblock the previous leg's tracks there so
    the opposite-side setback can leave, which also lets a wrap-around leg
    cross them (issue #56 - shorts and self-crossing loops around a
    termination resistor when an obstacle forces a detour).
    """
    for new_seg in new_segments:
        for old_seg in committed_segments:
            if new_seg.layer == old_seg.layer and \
                    _segments_properly_cross(new_seg, old_seg):
                return True
    return False


def _oriented(terminal: Tuple[Pad, Pad], pair: DiffPairNet) -> Tuple[Pad, Pad]:
    """Return the terminal as (current P pad, current N pad). A polarity pad
    swap flips the pads' net assignments, so the original ordering can be
    stale for subsequent legs."""
    pp, nn = terminal
    if pp.net_id == pair.n_net_id and nn.net_id == pair.p_net_id:
        return (nn, pp)
    return terminal


def get_diff_pair_terminals(pcb_data: PCBData, p_net_id: int, n_net_id: int
                            ) -> List[Tuple[Pad, Pad]]:
    """Group the pair's pads into (p_pad, n_pad) terminals by greedy nearest
    matching. Each terminal is a P pad and its physically adjacent N pad
    (connector pins, IC input pair, termination resistor, ...)."""
    p_pads = pcb_data.pads_by_net.get(p_net_id, [])
    n_pads = pcb_data.pads_by_net.get(n_net_id, [])

    candidates = []
    for pp in p_pads:
        for nn in n_pads:
            dist = math.hypot(pp.global_x - nn.global_x, pp.global_y - nn.global_y)
            candidates.append((dist, id(pp), id(nn), pp, nn))
    candidates.sort(key=lambda c: c[0])

    terminals = []
    used_p, used_n = set(), set()
    for dist, pid, nid, pp, nn in candidates:
        if pid in used_p or nid in used_n:
            continue
        used_p.add(pid)
        used_n.add(nid)
        terminals.append((pp, nn))
    return terminals


def _terminal_center(terminal: Tuple[Pad, Pad]) -> Tuple[float, float]:
    pp, nn = terminal
    return ((pp.global_x + nn.global_x) / 2, (pp.global_y + nn.global_y) / 2)


def candidate_terminal_chains(terminals: List[Tuple[Pad, Pad]],
                              max_candidates: int = 4) -> List[List[Tuple[Pad, Pad]]]:
    """Rank terminal orderings for an open chain (each terminal has only two
    usable sides, so the connection topology must be a path, not a tree).

    Score = total chain length + a penalty for each interior terminal whose
    two neighbors lie on the same side of its escape axis (the chain must
    pass "through" a terminal, so a same-side interior forces a wrap-around
    leg). Returns up to max_candidates orderings, best first - legs are routed
    sequentially with side constraints, so if one ordering fails the next may
    still succeed (a reversed chain constrains different ends).
    """
    n = len(terminals)
    if n <= 2:
        return [list(terminals)]

    centers = [_terminal_center(t) for t in terminals]
    # Escape axis per terminal: perpendicular to the P-N pad axis
    perps = []
    for pp, nn in terminals:
        ax, ay = pp.global_x - nn.global_x, pp.global_y - nn.global_y
        length = math.hypot(ax, ay)
        perps.append((-ay / length, ax / length) if length > 1e-6 else (0.0, 0.0))

    WRAP_PENALTY = 25.0  # mm-equivalent cost for a forced wrap-around leg

    def chain_score(order):
        score = sum(math.hypot(centers[a][0] - centers[b][0],
                               centers[a][1] - centers[b][1])
                    for a, b in zip(order, order[1:]))
        for k in range(1, len(order) - 1):
            i = order[k]
            px, py = perps[i]
            cx, cy = centers[i]
            prev_c = centers[order[k - 1]]
            next_c = centers[order[k + 1]]
            s_prev = px * (prev_c[0] - cx) + py * (prev_c[1] - cy)
            s_next = px * (next_c[0] - cx) + py * (next_c[1] - cy)
            if s_prev * s_next > 0:
                score += WRAP_PENALTY
        return score

    if n <= 7:
        orders = sorted(permutations(range(n)), key=chain_score)
    else:
        # Nearest-neighbor chains from each start, ranked by score
        orders = []
        for start in range(n):
            order = [start]
            left = set(range(n)) - {start}
            while left:
                last = order[-1]
                nxt = min(left, key=lambda j: math.hypot(
                    centers[last][0] - centers[j][0], centers[last][1] - centers[j][1]))
                order.append(nxt)
                left.remove(nxt)
            orders.append(tuple(order))
        orders = sorted(set(orders), key=chain_score)

    return [[terminals[i] for i in order] for order in orders[:max_candidates]]


def _pad_layer(pad: Pad, config: GridRouteConfig) -> str:
    """Pick the routing layer for a pad (first routing layer it exists on;
    through-hole / wildcard pads use the first routing layer)."""
    for layer in pad.layers:
        if layer in config.layers:
            return layer
    return config.layers[0]


def _make_endpoint(terminal: Tuple[Pad, Pad], config: GridRouteConfig) -> Tuple:
    """Build an endpoint tuple in get_diff_pair_endpoints format:
    (p_gx, p_gy, n_gx, n_gy, layer_idx, p_x, p_y, n_x, n_y)."""
    pp, nn = terminal
    coord = GridCoord(config.grid_step)
    layer_idx = config.layers.index(_pad_layer(pp, config))
    p_gx, p_gy = coord.to_grid(pp.global_x, pp.global_y)
    n_gx, n_gy = coord.to_grid(nn.global_x, nn.global_y)
    return (p_gx, p_gy, n_gx, n_gy, layer_idx,
            pp.global_x, pp.global_y, nn.global_x, nn.global_y)


def _terminal_base_dir(pcb_data: PCBData, pair: DiffPairNet, terminal: Tuple[Pad, Pad],
                       config: GridRouteConfig,
                       other_center: Tuple[float, float]) -> Tuple[float, float]:
    """Base escape direction for a terminal (synthesized from pad geometry for
    bare pads; both +/- sides of it are usable)."""
    pp, nn = terminal
    p_segments = [s for s in pcb_data.segments if s.net_id == pair.p_net_id]
    n_segments = [s for s in pcb_data.segments if s.net_id == pair.n_net_id]
    layer_name = _pad_layer(pp, config)
    dx, dy, _synth = get_pair_end_direction(
        pcb_data, pair.p_net_id, pair.n_net_id, p_segments, n_segments,
        pp.global_x, pp.global_y, nn.global_x, nn.global_y, layer_name,
        other_center=other_center)
    return (dx, dy)


def _leg_capsules(leg_ends, config: GridRouteConfig) -> List[Tuple[float, float, float, float, float]]:
    """Build connector-corridor exemption capsules for a leg.

    leg_ends: list of (center, base_dir, both_sides) - both_sides is True for
    fresh terminals (either side may be used), False for chain continuations
    (the direction is forced, only that side needs a corridor).
    """
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2
    if config.diff_pair_centerline_setback is not None:
        setback = config.diff_pair_centerline_setback
    else:
        setback = spacing_mm * 4
    radius = 2 * spacing_mm
    extent = setback + radius + 3 * config.grid_step

    capsules = []
    for (cx, cy), (dx, dy), both_sides in leg_ends:
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            continue
        signs = (1, -1) if both_sides else (1,)
        for sign in signs:
            capsules.append((cx, cy, cx + dx * sign * extent, cy + dy * sign * extent, radius))
    return capsules


def _capsule_cells(capsules, config: GridRouteConfig):
    """Enumerate the grid cells inside the given (x1, y1, x2, y2, radius_mm)
    capsules, for exempting connector corridors from own-track blocking."""
    coord = GridCoord(config.grid_step)
    cells = set()
    for (x1, y1, x2, y2, radius_mm) in capsules:
        radius_grid = max(1, int(radius_mm / config.grid_step + 0.5))
        length = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, int(length / config.grid_step + 0.5))
        for k in range(steps + 1):
            t = k / steps
            gx, gy = coord.to_grid(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)
            for dx in range(-radius_grid, radius_grid + 1):
                for dy in range(-radius_grid, radius_grid + 1):
                    if dx * dx + dy * dy <= radius_grid * radius_grid:
                        cells.add((gx + dx, gy + dy))
    return cells


def route_multipoint_diff_pair(state, pair: DiffPairNet, pair_name: str,
                               terminals: List[Tuple[Pad, Pad]]):
    """Route a multi-point diff pair as a chain of 2-point legs, trying
    alternative chain orderings if one fails (the side constraints at shared
    terminals depend on the routing order, so a reversed chain can succeed
    where the forward one wraps itself into a corner).

    Returns (leg_results, merged_result) on full success, ([], None) on
    failure (failed attempts' legs are ripped back out).
    """
    pcb_data = state.pcb_data
    config = state.config

    chains = candidate_terminal_chains(terminals)
    print(f"  Multi-point pair: {len(terminals)} terminals, {len(terminals) - 1} legs")
    for attempt, chain in enumerate(chains):
        if attempt > 0:
            print(f"  Trying alternative chain order ({attempt + 1}/{len(chains)})...")
        leg_results = _route_chain_attempt(state, pair, pair_name, chain)
        if leg_results is not None:
            # Merge legs into a single result for bookkeeping (rip-up, sync,
            # caches). Segments/vias are already in pcb_data - the merged
            # result must NOT be passed to add_route_to_pcb_data again.
            merged = dict(leg_results[-1])
            merged['new_segments'] = [s for r in leg_results for s in r['new_segments']]
            merged['new_vias'] = [v for r in leg_results for v in r['new_vias']]
            merged['iterations'] = sum(r.get('iterations', 0) for r in leg_results)
            merged['p_path'] = [pt for r in leg_results for pt in (r.get('p_path') or [])]
            merged['n_path'] = [pt for r in leg_results for pt in (r.get('n_path') or [])]
            return leg_results, merged
    return [], None


def _route_chain_attempt(state, pair: DiffPairNet, pair_name: str,
                         chain: List[Tuple[Pad, Pad]]) -> Optional[List[dict]]:
    """Route one chain ordering leg by leg. Returns the list of leg results on
    success, or None on failure (committed legs are ripped back out)."""
    pcb_data = state.pcb_data
    config = state.config

    print("  Chain: " + " -> ".join(
        f"{t[0].component_ref}:{t[0].pad_number}/{t[1].pad_number}" for t in chain))

    centers = [_terminal_center(t) for t in chain]
    leg_results = []
    committed_segments = []  # all committed legs' segments, for crossing checks
    applied_swaps = []  # polarity pad swaps applied by this attempt's legs
    forced_dir_next = None  # forced source direction for the next leg (opposite
                            # side of the previous leg at the shared terminal)

    def rip_attempt():
        for committed in reversed(leg_results):
            remove_route_from_pcb_data(pcb_data, committed)
        # Undo this attempt's pad swaps (their legs no longer exist)
        for entry in reversed(applied_swaps):
            undo_polarity_swap(pcb_data, entry)
            if entry in state.pad_swaps:
                state.pad_swaps.remove(entry)
        if applied_swaps:
            state.polarity_swapped_pairs.discard(pair_name)

    for i in range(len(chain) - 1):
        # A pad swap at a terminal flips its pads' net assignments - re-orient
        # each terminal tuple to (current P pad, current N pad)
        term_a = _oriented(chain[i], pair)
        term_b = _oriented(chain[i + 1], pair)
        print(f"  Leg {i + 1}/{len(chain) - 1}: "
              f"{term_a[0].component_ref}:{term_a[0].pad_number}/{term_a[1].pad_number} -> "
              f"{term_b[0].component_ref}:{term_b[0].pad_number}/{term_b[1].pad_number}")

        endpoints = (_make_endpoint(term_a, config), _make_endpoint(term_b, config))

        # Pad swaps are only safe at chain-fresh terminals: the target terminal
        # is always fresh, the source only on the first leg (a swap at a shared
        # terminal would break the previous leg)
        swap_ends = ('source', 'target') if i == 0 else ('target',)

        # Corridor exemptions: forced side only at a shared terminal, both
        # sides at fresh terminals
        if forced_dir_next is not None:
            src_dir, src_both = forced_dir_next, False
        else:
            src_dir, src_both = _terminal_base_dir(
                pcb_data, pair, term_a, config, other_center=centers[i + 1]), True
        tgt_dir = _terminal_base_dir(pcb_data, pair, term_b, config, other_center=centers[i])
        capsules = _leg_capsules(
            [(centers[i], src_dir, src_both), (centers[i + 1], tgt_dir, True)], config)
        corridor_cells = _capsule_cells(capsules, config)

        def leg_own_obstacles(obstacles, pcb, p_net_id, n_net_id, cfg, extra_clearance):
            # Own stubs/tracks/pads as obstacles, with this leg's connector
            # corridors exempted (a previous leg's connectors at a shared
            # terminal must not block the opposite-side setback)
            add_own_stubs_as_obstacles_for_diff_pair(
                obstacles, pcb, p_net_id, n_net_id, cfg, extra_clearance,
                exclude_cells=corridor_cells, exempt_capsules=capsules)

        obstacles, unrouted_stubs = build_diff_pair_obstacles(
            state.diff_pair_base_obstacles, pcb_data, config,
            state.routed_net_ids, state.remaining_net_ids,
            state.all_unrouted_net_ids, pair.p_net_id, pair.n_net_id,
            state.gnd_net_id, state.track_proximity_cache, state.layer_map,
            state.diff_pair_extra_clearance,
            add_own_stubs_func=leg_own_obstacles,
            net_obstacles_cache=state.net_obstacles_cache,
            ripped_route_layer_costs=state.ripped_route_layer_costs,
            ripped_route_via_positions=state.ripped_route_via_positions)

        result = route_diff_pair_with_obstacles(
            pcb_data, pair, config, obstacles, state.base_obstacles,
            unrouted_stubs, endpoints=endpoints,
            forced_source_dir=forced_dir_next,
            swap_allowed_ends=swap_ends)

        failed_reason = None
        if not result or result.get('failed') or result.get('probe_blocked'):
            failed_reason = "could not route"
        elif _pn_tracks_cross(result.get('new_segments', []), pair.p_net_id, pair.n_net_id):
            # Wrapping legs can fold back on themselves - never commit a crossing leg
            failed_reason = "crosses itself"
        elif _crosses_committed_legs(result.get('new_segments', []), committed_segments):
            # A wrap-around leg can cross an earlier leg's tracks inside the
            # shared terminal's corridor exemption (issue #56)
            failed_reason = "crosses an earlier leg"
        elif result.get('target_base_dir') is None:
            failed_reason = "missing target direction"

        if failed_reason:
            print(f"  Leg {i + 1} FAILED ({failed_reason}) - "
                  f"ripping {len(leg_results)} committed leg(s)")
            rip_attempt()
            return None

        # Apply a polarity pad swap decided by this leg BEFORE committing the
        # route: the appendix cleanup must see the swapped pad nets
        if result.get('polarity_fixed'):
            if apply_polarity_swap(pcb_data, result, state.pad_swaps):
                applied_swaps.append(state.pad_swaps[-1])
                state.polarity_swapped_pairs.add(pair_name)
            else:
                print(f"  Leg {i + 1} FAILED (pad swap could not be applied) - "
                      f"ripping {len(leg_results)} committed leg(s)")
                rip_attempt()
                return None

        # Commit this leg so the next leg sees it (as obstacle and topology)
        add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
        leg_results.append(result)
        committed_segments.extend(result.get('new_segments', []))

        # Next leg must leave the shared terminal on the opposite side
        used_dir = result['target_base_dir']
        forced_dir_next = (-used_dir[0], -used_dir[1])

    return leg_results
