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
from layer_swap_fallback import add_own_stubs_as_obstacles_for_diff_pair, rank_fallback_layers
from stub_layer_switching import apply_bare_pad_target_via
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
    """Group the pair's pads into (p_pad, n_pad) terminals by greedy matching,
    preferring P/N pads on the SAME component before a globally-closer
    cross-component pad. Each terminal is a P pad and its physically adjacent N
    pad (connector pins, IC input pair, termination resistor, ...).

    Same-component-first matters when a connector's two diff pins sit farther
    apart than a neighbouring part's pad: e.g. a USB connector J11 with D+/D-
    pins 3mm apart next to test points TP9/TP10. Plain greedy-nearest would
    cross-couple J11.D+ <-> TP10 (1.7mm) and TP9 <-> J11.D- (1.8mm) instead of
    the connector's real mates J11.D+ <-> J11.D- (3mm) and TP9 <-> TP10 (2.3mm).
    Those bogus terminals then route a coupled leg straight across the partner
    polarity's connector pad, shorting the pair (castor_pollux /MCU/CONN_D).
    Pairing the connector's own pins keeps each leg clear of the partner pad
    (and a too-wide own-pin terminal is later peeled to single-ended)."""
    p_pads = pcb_data.pads_by_net.get(p_net_id, [])
    n_pads = pcb_data.pads_by_net.get(n_net_id, [])

    candidates = []
    for pp in p_pads:
        for nn in n_pads:
            dist = math.hypot(pp.global_x - nn.global_x, pp.global_y - nn.global_y)
            cross_comp = 0 if pp.component_ref == nn.component_ref else 1
            candidates.append((cross_comp, dist, id(pp), id(nn), pp, nn))
    # All same-component pairs first (closest within that class), then
    # cross-component leftovers by distance.
    candidates.sort(key=lambda c: (c[0], c[1]))

    terminals = []
    used_p, used_n = set(), set()
    for cross_comp, dist, pid, nid, pp, nn in candidates:
        if pid in used_p or nid in used_n:
            continue
        used_p.add(pid)
        used_n.add(nid)
        terminals.append((pp, nn))
    return terminals


def _terminal_center(terminal: Tuple[Pad, Pad]) -> Tuple[float, float]:
    pp, nn = terminal
    return ((pp.global_x + nn.global_x) / 2, (pp.global_y + nn.global_y) / 2)


def terminal_separation(terminal: Tuple[Pad, Pad]) -> float:
    """P-to-N pad distance (mm) of a terminal."""
    pp, nn = terminal
    return math.hypot(pp.global_x - nn.global_x, pp.global_y - nn.global_y)


# A coupled diff-pair only earns its keep over an electrically long run. Over a
# leg shorter than a few setbacks there is no real coupled section - you spend
# ~1 setback fanning in and ~1 fanning out - so coupling buys nothing and only
# tangles the pair through clustered connector pads (castor_pollux /MCU/CONN_D, a
# USB connector). Such a leg is routed single-ended instead. 5x the connector
# setback is the floor. (Physics: even a 10 GHz / 10 Gb/s edge is electrically
# short over ~1-3 mm, so a few-mm connector fan-in never needs coupling.)
DIFF_PAIR_MIN_COUPLED_SETBACKS = 5.0


def diff_pair_min_coupled_length(config: GridRouteConfig) -> float:
    """Minimum leg length (mm) below which coupling gains nothing -> single-end.

    = DIFF_PAIR_MIN_COUPLED_SETBACKS * connector setback (the centerline setback
    if configured, else 4x the pair half-spacing - the same setback the corridor
    capsules use)."""
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2
    setback = (config.diff_pair_centerline_setback
               if config.diff_pair_centerline_setback is not None else spacing_mm * 4)
    return DIFF_PAIR_MIN_COUPLED_SETBACKS * setback


def leg_electrically_short(term_a: Tuple[Pad, Pad], term_b: Tuple[Pad, Pad],
                           config: GridRouteConfig) -> bool:
    """True if the leg between two terminals is too short for coupled routing to
    matter. Measured by the SHORTER of the leg's P-run and N-run: if either track
    cannot sustain a coupled section, route the leg single-ended. term_a/term_b
    are (P pad, N pad) tuples (oriented to current polarity)."""
    threshold = diff_pair_min_coupled_length(config)
    p_dist = math.hypot(term_a[0].global_x - term_b[0].global_x,
                        term_a[0].global_y - term_b[0].global_y)
    n_dist = math.hypot(term_a[1].global_x - term_b[1].global_x,
                        term_a[1].global_y - term_b[1].global_y)
    return min(p_dist, n_dist) < threshold


def classify_terminals(terminals: List[Tuple[Pad, Pad]], config: GridRouteConfig
                       ) -> Tuple[List[Tuple[Pad, Pad]], List[Tuple[Pad, Pad]]]:
    """Split terminals into (coupled, uncoupled).

    A terminal is coupled when its P and N pads are close enough to launch a real
    differential pair; uncoupled when they are too far apart to be a coupled
    connection (e.g. test points on opposite sides of a part). Threshold =
    diff_pair_uncouple_factor * (track_width + diff_pair_gap) (issue #121)."""
    threshold = config.diff_pair_uncouple_factor * (config.track_width + config.diff_pair_gap)
    coupled, uncoupled = [], []
    for t in terminals:
        (coupled if terminal_separation(t) <= threshold else uncoupled).append(t)
    return coupled, uncoupled


def _blocked_fraction(config, obstacles, cx, cy, layer_idx, radius_mm=1.5):
    """Fraction of a disk around (cx, cy) that is blocked on layer_idx."""
    coord = GridCoord(config.grid_step)
    r = max(1, int(radius_mm / config.grid_step))
    total = blocked = 0
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            gx, gy = coord.to_grid(cx + dx * config.grid_step, cy + dy * config.grid_step)
            total += 1
            if obstacles.is_blocked(gx, gy, layer_idx):
                blocked += 1
    return blocked / total if total else 1.0


def _fake_pad(ref_pad: Pad, x: float, y: float, layer: str) -> Pad:
    """A stand-in pad at (x, y) on `layer` carrying ref_pad's net, used as a leg
    launch point after a terminal is relocated to an open layer (issue #121)."""
    return Pad(
        component_ref=ref_pad.component_ref, pad_number=ref_pad.pad_number,
        global_x=x, global_y=y, local_x=x, local_y=y,
        size_x=ref_pad.size_x, size_y=ref_pad.size_y, shape=ref_pad.shape,
        layers=[layer], net_id=ref_pad.net_id, net_name=ref_pad.net_name)


def _candidate_relocation_layers(state, terminals, obstacles, max_layers=3):
    """Rank shared target layers for relocating blocked terminals: a layer is a
    candidate if it is clearly more open than at least one terminal's own layer.
    Non-plane (signal) layers first, then by worst-case openness across the
    blocked terminals; planes last (routing them slots the reference). (#121)"""
    config = state.config
    pcb_data = state.pcb_data
    centers = [_terminal_center(t) for t in terminals]
    own_fracs = [_blocked_fraction(config, obstacles, cx, cy,
                                   config.layers.index(_pad_layer(t[0], config)))
                 for t, (cx, cy) in zip(terminals, centers)]
    if not any(f >= 0.45 for f in own_fracs):
        return []  # nothing is meaningfully blocked
    ranked = []
    for li, layer in enumerate(config.layers):
        fracs = [_blocked_fraction(config, obstacles, cx, cy, li) for cx, cy in centers]
        worst = max(fracs)
        # Useful only if it actually unblocks a blocked terminal.
        helps = any(own >= 0.45 and fracs[k] < own - 0.15 and fracs[k] < 0.4
                    for k, own in enumerate(own_fracs))
        if not helps:
            continue
        is_plane = any(_layer_is_plane_for(pcb_data, layer, cx, cy) for cx, cy in centers)
        ranked.append((is_plane, worst, layer))
    ranked.sort(key=lambda t: (t[0], t[1]))
    return [layer for _, _, layer in ranked[:max_layers]]


def _layer_is_plane_for(pcb_data, layer_name, x, y):
    from layer_swap_fallback import _layer_is_plane_at
    return _layer_is_plane_at(pcb_data, layer_name, x, y)


def _relocate_blocked_terminals(state, pair: DiffPairNet, terminals, obstacles, target_layer):
    """Relocate every terminal whose own (pad) layer is too blocked to launch a
    coupled pair onto `target_layer` (when target_layer is open there): drop a
    via on each pad to switch layers and grow a short stub (apply_bare_pad_target_via);
    the leg launches from the stub's free end. Reuses the layer-switch endpoint
    mechanism (issue #121, the user's "drop a via, call the stub an endpoint").

    Returns (new_terminals, fans): new_terminals has relocated terminals replaced
    by stub-end stand-in pads; fans is a list of (via, stub) added to pcb_data
    (attach to the route on success, remove on failure)."""
    config = state.config
    pcb_data = state.pcb_data
    centers = [_terminal_center(t) for t in terminals]
    tgt_idx = config.layers.index(target_layer)
    new_terminals = list(terminals)
    fans = []
    for i, (pp, nn) in enumerate(terminals):
        cx, cy = centers[i]
        pad_layer = _pad_layer(pp, config)
        if pad_layer == target_layer:
            continue
        frac = _blocked_fraction(config, obstacles, cx, cy, config.layers.index(pad_layer))
        tgt_frac = _blocked_fraction(config, obstacles, cx, cy, tgt_idx)
        # Only relocate a genuinely-blocked terminal onto a clearly-open target.
        if frac < 0.45 or tgt_frac > frac - 0.15 or tgt_frac > 0.4:
            continue
        # Aim the new stubs toward the rest of the chain (away from this part).
        others = [centers[j] for j in range(len(centers)) if j != i] or [(cx, cy)]
        ax = sum(c[0] for c in others) / len(others)
        ay = sum(c[1] for c in others) / len(others)
        via_p, stub_p = apply_bare_pad_target_via(pcb_data, pp.net_id, pp.global_x, pp.global_y, target_layer, ax, ay, config)
        via_n, stub_n = apply_bare_pad_target_via(pcb_data, nn.net_id, nn.global_x, nn.global_y, target_layer, ax, ay, config)
        fans += [(via_p, stub_p), (via_n, stub_n)]
        new_terminals[i] = (_fake_pad(pp, stub_p.end_x, stub_p.end_y, target_layer),
                            _fake_pad(nn, stub_n.end_x, stub_n.end_y, target_layer))
        print(f"    Relocated terminal {pp.component_ref}:{pp.pad_number}/{nn.pad_number} "
              f"{pad_layer}({frac*100:.0f}%) -> {target_layer}({tgt_frac*100:.0f}%)")
    return new_terminals, fans


def _attach_fans(merged, fans):
    """Record relocation fan copper (via + stub per pad) on the merged route so
    sync_pcb_data_segments writes it to the output."""
    merged['new_segments'] = list(merged.get('new_segments', [])) + [s for _, s in fans]
    merged['new_vias'] = list(merged.get('new_vias', [])) + [v for v, _ in fans]


def _cleanup_fans(pcb_data, fans):
    """Remove relocation fan copper from pcb_data (failed attempt)."""
    for via, stub in fans:
        if via in pcb_data.vias:
            pcb_data.vias.remove(via)
        if stub in pcb_data.segments:
            pcb_data.segments.remove(stub)


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

    Returns (leg_results, merged_result, peeled). leg_results is None ONLY on
    genuine failure (an empty list means every leg was deferred to single-ended,
    which is success); merged_result is None when no leg was coupled; peeled is
    the list of terminals left for single-ended routing (far-apart peeled and/or
    electrically-short deferred legs). Failed attempts' legs are ripped out.
    """
    config = state.config
    pcb_data = state.pcb_data
    obstacles = state.diff_pair_base_obstacles

    print(f"  Multi-point pair: {len(terminals)} terminals, {len(terminals) - 1} legs")
    # Try the full coupled chain first - a pair that routes fine through a
    # widely-spaced terminal (e.g. a test point reached as a chain end) must not
    # be broken up needlessly (issue #121). Electrically short legs inside the
    # chain are deferred to single-ended and returned in `peeled`.
    leg_results, merged, se = _route_terminal_set(state, pair, pair_name, terminals)
    if leg_results is not None:
        return leg_results, merged, se

    # The chain couldn't launch on the terminals' own layers (jammed outer
    # layers). Relocate the blocked terminals onto an open layer (drop a via,
    # launch from the stub) and retry - trying candidate target layers in turn
    # (non-plane first, then planes, which the router treats as open).
    def _try_relocated(term_set):
        for layer in _candidate_relocation_layers(state, term_set, obstacles):
            reloc, fans = _relocate_blocked_terminals(state, pair, term_set, obstacles, layer)
            if not fans:
                continue
            print(f"  Retrying chain with {len(fans)//2} terminal(s) relocated to {layer}...")
            legs, m, se_legs = _route_terminal_set(state, pair, pair_name, reloc)
            if legs is not None:
                if m is not None:
                    _attach_fans(m, fans)
                return legs, m, se_legs
            _cleanup_fans(pcb_data, fans)
        return None, None, []

    if obstacles is not None:
        leg_results, merged, se = _try_relocated(terminals)
        if leg_results is not None:
            return leg_results, merged, se

    # Still stuck. If some terminals are too far apart to be a coupled
    # differential connection, peel them off and route only the coupled
    # terminals as a pair; the peeled pads are connected single-ended afterward.
    coupled, uncoupled = classify_terminals(terminals, config)
    if uncoupled and len(coupled) >= 2:
        far = min(terminal_separation(t) for t in uncoupled)
        print(f"  Coupled chain failed; peeling {len(uncoupled)} far-apart terminal(s) "
              f"(P/N >= {far:.1f}mm apart) to route single-ended, retrying "
              f"coupled {len(coupled)}-terminal chain...")
        leg_results, merged, se = _route_terminal_set(state, pair, pair_name, coupled)
        if leg_results is not None:
            return leg_results, merged, uncoupled + se
        if obstacles is not None:
            leg_results, merged, se = _try_relocated(coupled)
            if leg_results is not None:
                return leg_results, merged, uncoupled + se
    return None, None, []


def _route_terminal_set(state, pair: DiffPairNet, pair_name: str,
                        terminals: List[Tuple[Pad, Pad]]):
    """Route a fixed set of terminals as a coupled chain, trying alternative
    chain orderings.

    Returns (leg_results, merged, se_terminals). On success leg_results is the
    (possibly empty) list of coupled legs, merged is their combined result (None
    if every leg was electrically short -> deferred), and se_terminals lists the
    terminals deferred to single-ended. On genuine failure returns (None, None,
    []); leg_results is None ONLY on failure (empty list = all-deferred success)."""
    if len(terminals) < 2:
        return None, None, []
    chains = candidate_terminal_chains(terminals)
    for attempt, chain in enumerate(chains):
        if attempt > 0:
            print(f"  Trying alternative chain order ({attempt + 1}/{len(chains)})...")
        leg_results, se_terminals = _route_chain_attempt(state, pair, pair_name, chain)
        if leg_results is not None:
            if not leg_results:
                # Every leg was electrically short - nothing coupled to merge,
                # but this is success: the whole pair goes to single-ended.
                return leg_results, None, se_terminals
            # Merge legs into a single result for bookkeeping (rip-up, sync,
            # caches). Segments/vias are already in pcb_data - the merged
            # result must NOT be passed to add_route_to_pcb_data again.
            merged = dict(leg_results[-1])
            merged['new_segments'] = [s for r in leg_results for s in r['new_segments']]
            merged['new_vias'] = [v for r in leg_results for v in r['new_vias']]
            merged['iterations'] = sum(r.get('iterations', 0) for r in leg_results)
            merged['p_path'] = [pt for r in leg_results for pt in (r.get('p_path') or [])]
            merged['n_path'] = [pt for r in leg_results for pt in (r.get('n_path') or [])]
            return leg_results, merged, se_terminals
    return None, None, []


def _route_chain_attempt(state, pair: DiffPairNet, pair_name: str,
                         chain: List[Tuple[Pad, Pad]]):
    """Route one chain ordering leg by leg.

    Returns (leg_results, se_terminals): leg_results is the list of coupled leg
    results (possibly empty if every leg was electrically short), se_terminals is
    the list of terminals whose connecting leg was deferred to single-ended
    routing. Returns (None, []) on genuine failure (committed legs ripped out)."""
    pcb_data = state.pcb_data
    config = state.config

    print("  Chain: " + " -> ".join(
        f"{t[0].component_ref}:{t[0].pad_number}/{t[1].pad_number}" for t in chain))

    centers = [_terminal_center(t) for t in chain]
    leg_results = []
    se_terminals = []  # terminals whose leg is too short to couple -> single-ended
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

        # An electrically short leg gains nothing from coupling and only tangles
        # the pair through clustered pads - defer it to single-ended routing. The
        # chain breaks here (next leg starts fresh); the deferred terminals are
        # reconnected by the single-ended follow-up pass.
        if leg_electrically_short(term_a, term_b, config):
            print(f"    electrically short (< {diff_pair_min_coupled_length(config):.1f}mm "
                  f"coupled) - deferring leg to single-ended")
            se_terminals.extend([chain[i], chain[i + 1]])
            forced_dir_next = None
            continue

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

        # Only this leg's own terminal pads may be opened by its corridors; a
        # different terminal's pad falling inside the corridor stays blocked so
        # the partner-polarity track can't graze it (castor_pollux J11).
        leg_pad_ids = {id(p) for p in (term_a[0], term_a[1], term_b[0], term_b[1])}

        def leg_own_obstacles(obstacles, pcb, p_net_id, n_net_id, cfg, extra_clearance):
            # Own stubs/tracks/pads as obstacles, with this leg's connector
            # corridors exempted (a previous leg's connectors at a shared
            # terminal must not block the opposite-side setback)
            add_own_stubs_as_obstacles_for_diff_pair(
                obstacles, pcb, p_net_id, n_net_id, cfg, extra_clearance,
                exclude_cells=corridor_cells, exempt_capsules=capsules,
                exempt_pads=leg_pad_ids)

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
            return None, []

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
                return None, []

        # Commit this leg so the next leg sees it (as obstacle and topology)
        add_route_to_pcb_data(pcb_data, result, debug_lines=config.debug_lines)
        leg_results.append(result)
        committed_segments.extend(result.get('new_segments', []))

        # Next leg must leave the shared terminal on the opposite side
        used_dir = result['target_base_dir']
        forced_dir_next = (-used_dir[0], -used_dir[1])

    return leg_results, se_terminals
