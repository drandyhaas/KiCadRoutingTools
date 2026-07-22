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
from __future__ import annotations

import math
from itertools import permutations
from typing import List, Optional, Tuple

try:
    from scipy.optimize import linear_sum_assignment
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

from kicad_parser import PCBData, Pad
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
from diff_pair_routing import (
    route_diff_pair_with_obstacles, get_pair_end_direction, _pn_tracks_cross,
    _DRC_CLEARANCE_MARGIN, _routing_copper_layers, _seg_to_seglist_min_edge,
    _route_direct_coupled_middle
)
from layer_swap_fallback import add_own_stubs_as_obstacles_for_diff_pair, rank_fallback_layers
from stub_layer_switching import apply_bare_pad_target_via
from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from polarity_swap import apply_polarity_swap, undo_polarity_swap
from routing_context import build_diff_pair_obstacles, build_diff_pair_leg_obstacles


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


def _pn_self_overlaps(new_segments, p_net_id, n_net_id, config, pcb_data=None) -> bool:
    """True if this leg's new P/N copper comes below clearance to the partner
    polarity (an intra-pair SEGMENT-SEGMENT overlap, #215). A multipoint leg is a
    standard coupled route, so it can pinch P against N at the terminal connectors
    just like a top-level pair -- but unlike the top-level path there is no hybrid
    swap to fall back on here, so a self-overlapping leg is rejected and the chain
    falls through to single-ended (the right treatment for these short legs).

    The partner copper includes the pair's pre-existing fanout stubs in pcb_data
    (not just this leg's own segments): the leg's coupled middle can land on the
    OTHER net's carried-through escape stub (butterstick USB-PHY/USBD), and the
    diff-pair obstacle map excludes both pair nets so the router never saw it."""
    new_p = [s for s in new_segments if s.net_id == p_net_id]
    new_n = [s for s in new_segments if s.net_id == n_net_id]
    if not new_p and not new_n:
        return False
    own = set(id(s) for s in new_segments)
    all_p, all_n = list(new_p), list(new_n)
    if pcb_data is not None:
        for s in pcb_data.segments:
            if id(s) in own:
                continue
            if s.net_id == p_net_id:
                all_p.append(s)
            elif s.net_id == n_net_id:
                all_n.append(s)
    if not all_p or not all_n:
        return False
    # Only the NEW segments need testing as the moving party -- pre-existing
    # stub-vs-stub spacing was already DRC-valid before this leg.
    for s in new_p:
        if _seg_to_seglist_min_edge(s.start_x, s.start_y, s.end_x, s.end_y,
                                    s.width, s.layer, all_n) < config.clearance - 1e-6:
            return True
    for s in new_n:
        if _seg_to_seglist_min_edge(s.start_x, s.start_y, s.end_x, s.end_y,
                                    s.width, s.layer, all_p) < config.clearance - 1e-6:
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


def _min_cost_matching(cost: List[List[float]]) -> List[Tuple[int, int]]:
    """Minimum-total-cost bipartite matching of a rectangular cost matrix
    (rows x cols). Returns min(rows, cols) (row, col) index pairs, each row and
    each column used at most once. Uses scipy's Hungarian solver when available
    (e.g. CLI), else an exact brute force for the tiny pad counts seen here
    (KiCad's bundled Python has no scipy), with a greedy fallback for the rare
    large case."""
    rows = len(cost)
    cols = len(cost[0]) if rows else 0
    if rows == 0 or cols == 0:
        return []
    if HAS_SCIPY:
        import numpy as np
        ri, ci = linear_sum_assignment(np.array(cost, dtype=float))
        return list(zip(ri.tolist(), ci.tolist()))

    # No scipy: brute-force the smaller side over the larger. Pad counts per net
    # are tiny (connector redundancy is a handful of pins), so this is cheap.
    transpose = rows > cols
    c = [[cost[r][cc] for r in range(rows)] for cc in range(cols)] if transpose else cost
    nr = len(c)            # nr <= nc after any transpose
    nc = len(c[0])
    if math.perm(nc, nr) <= 40320:  # 8! injective assignments
        best_total, best = None, None
        for perm in permutations(range(nc), nr):
            total = sum(c[r][perm[r]] for r in range(nr))
            if best_total is None or total < best_total:
                best_total, best = total, perm
        pairs = [(r, best[r]) for r in range(nr)]
    else:
        # Greedy fallback (suboptimal, but only for unrealistically many pads).
        print(f"  Warning: {nr}x{nc} terminal matching too large for the exact "
              f"solver without scipy - using greedy (pairing may be suboptimal)")
        used_r, used_c, pairs = set(), set(), []
        for _, r, cc in sorted((c[r][cc], r, cc) for r in range(nr) for cc in range(nc)):
            if r in used_r or cc in used_c:
                continue
            used_r.add(r)
            used_c.add(cc)
            pairs.append((r, cc))
    return [(cc, r) for (r, cc) in pairs] if transpose else pairs


def get_diff_pair_terminals(pcb_data: PCBData, p_net_id: int, n_net_id: int
                            ) -> List[Tuple[Pad, Pad]]:
    """Group the pair's pads into (p_pad, n_pad) terminals by minimum-cost
    matching, preferring P/N pads on the SAME component before a globally-closer
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
    (and a too-wide own-pin terminal is later peeled to single-ended).

    Greedy-by-distance is *locally* optimal but not globally: a closest-first
    pick can strand the leftover pads into a far pairing. On a USB-C connector
    whose redundant DP/DN pads interleave (J1 y-order B7,A6,A7,B6), greedy took
    {A6,A7}(0.5mm) first and was then forced into {B6,B7}(1.5mm) - the wide
    terminal drives a long diagonal connector that grazes the partner pad
    (issue #165). Minimum-TOTAL-cost matching instead yields {A6,B7}+{B6,A7}
    (0.5+0.5), each terminal tight so its connector launches straight out."""
    p_pads = pcb_data.pads_by_net.get(p_net_id, [])
    n_pads = pcb_data.pads_by_net.get(n_net_id, [])
    if not p_pads or not n_pads:
        return []

    # Lexicographic cost: cross-component pairings carry a penalty larger than
    # any achievable total distance, so the matcher minimizes the cross-component
    # count first (keeping a part's own diff pins paired), then total distance.
    dist_sum = sum(math.hypot(pp.global_x - nn.global_x, pp.global_y - nn.global_y)
                   for pp in p_pads for nn in n_pads)
    cross_penalty = dist_sum + 1.0
    cost = [[math.hypot(pp.global_x - nn.global_x, pp.global_y - nn.global_y)
             + (0.0 if pp.component_ref == nn.component_ref else cross_penalty)
             for nn in n_pads]
            for pp in p_pads]

    return [(p_pads[i], n_pads[j]) for (i, j) in _min_cost_matching(cost)]


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
# setback is the geometric floor.
DIFF_PAIR_MIN_COUPLED_SETBACKS = 5.0

# Absolute electrical floor (mm): below ~lambda/10 at 5 GHz on FR4 a pair is
# electrically short, so SE vs coupled is indistinguishable no matter how tight
# the geometric fan-in is. v = c/sqrt(eps_eff) ~= 145-173 mm/ns on FR4
# (stripline..microstrip), f = 5 GHz -> lambda ~= 29-35 mm -> lambda/10 ~= 3 mm.
# This catches tight-pitch pairs (width+gap <= 0.3 mm) whose 5x-setback floor is
# under 3 mm; wider pairs are still governed by the larger geometric floor.
# (Assumes a <=5 GHz design; a much faster board would want a smaller floor.)
DIFF_PAIR_MIN_COUPLED_FLOOR_MM = 3.0


def diff_pair_min_coupled_length(config: GridRouteConfig) -> float:
    """Minimum leg length (mm) below which coupling gains nothing -> single-end.

    = max(DIFF_PAIR_MIN_COUPLED_SETBACKS * connector setback,
          DIFF_PAIR_MIN_COUPLED_FLOOR_MM)
    where the setback is the centerline setback if configured, else 4x the pair
    half-spacing (the same setback the corridor capsules use). The geometric term
    keeps short fan-ins from tangling; the absolute lambda/10-at-5GHz floor keeps
    electrically-short pairs single-ended even at very tight pitch."""
    spacing_mm = (config.track_width + config.diff_pair_gap) / 2
    setback = (config.diff_pair_centerline_setback
               if config.diff_pair_centerline_setback is not None else spacing_mm * 4)
    return max(DIFF_PAIR_MIN_COUPLED_SETBACKS * setback,
               DIFF_PAIR_MIN_COUPLED_FLOOR_MM)


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


def diff_pair_is_short(pcb_data: PCBData, p_net_id: int, n_net_id: int,
                       track_width: float, diff_pair_gap: float,
                       centerline_setback: float = None) -> bool:
    """True if a pair would be wholly deferred to single-ended as electrically
    short -- the same test route_diff applies before routing (diff_pair_loop:
    a 2-terminal pair whose single leg is electrically short is left for the
    single-ended pass). Used by the GUI to hide short pairs from the diff-pair
    list and to keep them visible on the single-ended list. Conservative for
    multi-point pairs (3+ terminals): those may still couple some legs, so they
    are never reported short here. Reuses leg_electrically_short so the GUI and
    the router agree on what counts as short.

    centerline_setback mirrors the GUI control: None or <= 0 means auto."""
    from types import SimpleNamespace
    terminals = get_diff_pair_terminals(pcb_data, p_net_id, n_net_id)
    if len(terminals) != 2:
        return False
    cfg = SimpleNamespace(
        track_width=track_width, diff_pair_gap=diff_pair_gap,
        diff_pair_centerline_setback=(centerline_setback
                                      if centerline_setback else None))
    return leg_electrically_short(terminals[0], terminals[1], cfg)


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
        layers=[layer], net_id=ref_pad.net_id, net_name=ref_pad.net_name,
        rect_rotation=getattr(ref_pad, 'rect_rotation', 0.0))


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
    relocated_pads = []
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
        relocated_pads += [pp, nn]
        new_terminals[i] = (_fake_pad(pp, stub_p.end_x, stub_p.end_y, target_layer),
                            _fake_pad(nn, stub_n.end_x, stub_n.end_y, target_layer))
        print(f"    Relocated terminal {pp.component_ref}:{pp.pad_number}/{nn.pad_number} "
              f"{pad_layer}({frac*100:.0f}%) -> {target_layer}({tgt_frac*100:.0f}%)")

    # A relocation drops a through-via on each pad. On a dense connector (USB-C,
    # fine-pitch headers) those per-pad vias can't fit - 0.5mm-pitch pads with a
    # 0.5mm via collide - so relocating there just trades the original blockage
    # for via-via / via-pad violations. Validate the fan vias with the same
    # clearance check as check_drc; if they don't fit, abort the relocation (the
    # pair then falls through to single-ended, the right treatment for a dense
    # connector fan-out). Issue #165.
    if fans and not _fans_fit(pcb_data, fans, relocated_pads, config):
        print(f"    Relocation to {target_layer} would collide (dense pads) - "
              f"skipping (pair falls through to single-ended)")
        _cleanup_fans(pcb_data, fans)
        return list(terminals), []
    return new_terminals, fans


def _fans_fit(pcb_data, fans, relocated_pads, config) -> bool:
    """True if every relocation fan via clears (at check_drc's clearance) the
    other fan vias, the existing vias, all foreign pads, and all foreign tracks.
    The via legitimately sits on its own relocated pad and connects to its own-net
    stub, so own-net pads/segments are excluded."""
    from check_drc import (check_via_via_overlap, check_pad_via_overlap,
                           check_via_segment_overlap, check_via_drill_overlap,
                           check_pad_drill_via_overlap)
    clearance = config.clearance
    h2h = getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0
    margin = _DRC_CLEARANCE_MARGIN
    # A fan entry's via is None when apply_bare_pad_target_via REUSED an existing
    # same-net via (#282) instead of drilling a new one - there is no new via to
    # validate (the reused via is already committed and gets checked as an
    # existing via when the partner fan via is tested). Skip the None entries.
    fan_vias = [v for v, _ in fans if v is not None]
    fan_ids = {id(v) for v in fan_vias}
    reloc_ids = {id(p) for p in relocated_pads}
    routing_layers = _routing_copper_layers(pcb_data, config)

    for i, v in enumerate(fan_vias):
        for w in fan_vias[i + 1:]:
            if check_via_via_overlap(v, w, clearance, margin)[0]:
                return False
            if h2h and check_via_drill_overlap(v, w, h2h, margin)[0]:
                return False
        # Drill hole-to-hole is net-INDEPENDENT at the fab (#282), so unlike the
        # body checks the drill pass excludes nothing (mirrors
        # _bare_pad_pair_vias_fit, which this gate previously omitted).
        for ev in pcb_data.vias:
            if id(ev) in fan_ids:
                continue
            if check_via_via_overlap(v, ev, clearance, margin)[0]:
                return False
            if h2h and check_via_drill_overlap(v, ev, h2h, margin)[0]:
                return False
        for pads in pcb_data.pads_by_net.values():
            for pad in pads:
                if id(pad) in reloc_ids:
                    continue  # the via legitimately sits on its own relocated pad
                if check_pad_via_overlap(pad, v, clearance, routing_layers, margin)[0]:
                    return False
                if h2h and check_pad_drill_via_overlap(pad, v, h2h, margin)[0]:
                    return False
        for seg in pcb_data.segments:
            if seg.net_id == v.net_id:
                continue  # own-net stub the via connects to
            if check_via_segment_overlap(v, seg, clearance, margin)[0]:
                return False
    return True


def _attach_fans(merged, fans, legs=None):
    """Record relocation fan copper (a layer-switch via + inner-layer stub per
    relocated pad) so it is WRITTEN to the output.

    The output file is generated from the per-leg `results` list, and vias in
    particular reach the output ONLY through a result's ``new_vias`` -
    sync_pcb_data_segments syncs segments but not vias. So fan copper recorded
    only on the `merged` bookkeeping dict (which feeds obstacle sync, not the
    writer) is dropped from the output: the relocated route then lands on the
    inner layer with NO via back to its surface pad and is left floating - the
    whole pair disconnected though DRC-clean (issue #165, tigard /USB_D on a
    congested F.Cu that forced relocation to In1.Cu). Put the fan copper on a
    real leg result so the writer emits its vias, and keep `merged` consistent
    for obstacle sync."""
    fan_segs = [s for _, s in fans]
    # A reused existing via (#282) is recorded as None: it is already in
    # pcb_data.vias / an earlier result's new_vias, so it must NOT be re-emitted
    # here (a duplicate via in the writer). Only NEW fan vias reach new_vias.
    fan_vias = [v for v, _ in fans if v is not None]
    sink = legs[0] if legs else merged
    sink['new_segments'] = list(sink.get('new_segments', [])) + fan_segs
    sink['new_vias'] = list(sink.get('new_vias', [])) + fan_vias
    if merged is not sink:
        merged['new_segments'] = list(merged.get('new_segments', [])) + fan_segs
        merged['new_vias'] = list(merged.get('new_vias', [])) + fan_vias


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


def _terminal_stub_endpoint(pcb_data: PCBData, terminal: Tuple[Pad, Pad],
                            config: GridRouteConfig):
    """Endpoint at the terminal's fanout STUB free ends (clear of the BGA) instead
    of the pad midpoint (buried inside the BGA -> blocked on every layer, so a
    coupled middle can't even start there). For each polarity, flood-fill the net's
    copper from the pad and take the farthest free end of that component -- the
    escape tip the fanout already threaded out between the bus vias. Returns a
    get_diff_pair_endpoints-format tuple, or None if either stub tip is missing."""
    from connectivity import find_stub_free_ends
    coord = GridCoord(config.grid_step)
    pp, nn = terminal
    tol = 0.05

    def _near(ax, ay, bx, by):
        return abs(ax - bx) < tol and abs(ay - by) < tol

    def tip(pad):
        segs = [s for s in pcb_data.segments if s.net_id == pad.net_id]
        comp, used = [], [False] * len(segs)
        pts = [(pad.global_x, pad.global_y)]
        changed = True
        while changed:
            changed = False
            for k, s in enumerate(segs):
                if used[k]:
                    continue
                if any(_near(s.start_x, s.start_y, px, py) or _near(s.end_x, s.end_y, px, py)
                       for px, py in pts):
                    used[k] = True
                    comp.append(s)
                    pts.append((s.start_x, s.start_y))
                    pts.append((s.end_x, s.end_y))
                    changed = True
        if not comp:
            return None
        ends = find_stub_free_ends(comp, [pad])
        return [(fx, fy, config.layers.index(flayer))
                for fx, fy, flayer in ends if flayer in config.layers]

    p_tips, n_tips = tip(pp), tip(nn)
    if not p_tips or not n_tips:
        return None
    # #369 A14: the endpoint tuple carries ONE layer slot; the old code took
    # each polarity's farthest tip independently and stamped P's layer on N's
    # coordinates -- tips on different layers made the leg emit N's connector
    # on P's layer with no via (an open). Pick the farthest SAME-LAYER tip
    # pair instead, and bail to the pad-midpoint fallback when no common
    # layer exists (mirrors the 2-terminal path's n_layer == p_layer gate).
    best = None
    for px, py, pl in p_tips:
        for nx, ny, nl in n_tips:
            if pl != nl:
                continue
            score = (math.hypot(px - pp.global_x, py - pp.global_y)
                     + math.hypot(nx - nn.global_x, ny - nn.global_y))
            if best is None or score > best[0]:
                best = (score, px, py, nx, ny, pl)
    if best is None:
        return None
    _, px, py, nx, ny, pl = best
    p_gx, p_gy = coord.to_grid(px, py)
    n_gx, n_gy = coord.to_grid(nx, ny)
    return (p_gx, p_gy, n_gx, n_gy, pl, px, py, nx, ny)


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
                    _attach_fans(m, fans, legs)
                else:
                    # Every leg deferred to single-ended - the relocation buys
                    # nothing (the single-ended pass connects the pads directly),
                    # so drop its fan copper instead of leaving it floating.
                    _cleanup_fans(pcb_data, fans)
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

    # Last resort before deferring: MOVE the blocked terminals' escape stubs
    # to a more-open layer. The switch's pad via sizes itself down the fab
    # ladder (fitting_pad_via) -- the nominal via never fits a dense ball
    # field, the smaller rungs do -- and moving a stub also REMOVES the
    # copper that walls its twin (each USB twin's F.Cu escape is the other's
    # nearest wall, so the pair unboxes itself). Endpoint derivation finds
    # the moved tips naturally: the old-layer copper is gone.
    switched = _switch_blocked_terminal_stubs(state, pair, terminals)
    if switched:
        leg_results, merged, se = _route_terminal_set(state, pair, pair_name,
                                                      terminals)
        if leg_results is not None:
            print(f"  STUB LAYER SWITCH SUCCESS: coupled chain routed after "
                  f"moving {len(switched)} terminal stub pair(s)")
            return leg_results, merged, se
        from stub_layer_switching import revert_stub_layer_switch
        for mods, vias in reversed(switched):
            revert_stub_layer_switch(pcb_data, mods, vias)
        print(f"  Stub layer switch bought nothing - reverted")

    # No coupled chain could be routed and relocation didn't help (e.g. a dense
    # connector fan-out whose pads are too tightly pitched to drop relocation
    # vias - tigard /USB_D on a congested F.Cu). Rather than leave the pair
    # unrouted, defer the WHOLE pair to single-ended: the follow-up pass routes
    # each net as a plain track to its pads (no coupling, no graze). Signalled
    # like the all-electrically-short case (empty legs, no merged, all terminals
    # peeled). Issue #165.
    print(f"  Coupled routing failed and relocation unavailable - deferring the "
          f"whole pair to single-ended")
    return [], None, list(terminals)


def _switch_blocked_terminal_stubs(state, pair: DiffPairNet, terminals):
    """Move the escape stubs of blocked terminals (own layer >=45% walled)
    onto the most-open other layer, validate_swap-checked; the pad vias size
    themselves via fitting_pad_via. Returns [(segment_mods, new_vias)] undo
    tokens, empty when nothing switched. Bare-pad terminals (no stub) are
    left to the relocation path."""
    from stub_layer_switching import (get_stub_info, apply_stub_layer_switch,
                                      validate_swap)
    config = state.config
    pcb_data = state.pcb_data
    obstacles = state.diff_pair_base_obstacles
    switched = []
    for term in terminals:
        pp, nn = term
        cx, cy = _terminal_center(term)
        own_layer = _pad_layer(pp, config)
        own_idx = config.layers.index(own_layer)
        if obstacles is None or _blocked_fraction(config, obstacles, cx, cy,
                                                  own_idx) < 0.45:
            continue
        ep = _terminal_stub_endpoint(pcb_data, term, config)
        if ep is None:
            continue
        stub_layer = config.layers[ep[4]]
        stub_p = get_stub_info(pcb_data, pp.net_id, ep[5], ep[6], stub_layer)
        stub_n = get_stub_info(pcb_data, nn.net_id, ep[7], ep[8], stub_layer)
        if stub_p is None or stub_n is None:
            continue
        ranked = sorted(
            (l for l in config.layers if l != stub_layer),
            key=lambda l: _blocked_fraction(config, obstacles, cx, cy,
                                            config.layers.index(l)))
        for dest in ranked:
            ok, reason = validate_swap(
                stub_p, stub_n, dest, {}, pcb_data, config,
                swap_partner_net_ids={pp.net_id, nn.net_id})
            if not ok:
                continue
            mods_all, vias_all = [], []
            for st in (stub_p, stub_n):
                vias, mods = apply_stub_layer_switch(pcb_data, st, dest,
                                                     config, debug=False)
                vias_all += vias
                mods_all += mods
            print(f"  Stub layer switch: {pp.component_ref}:"
                  f"{pp.pad_number}/{nn.pad_number} {stub_layer} -> {dest}")
            switched.append((mods_all, vias_all))
            break
    return switched


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
            # #369 A2: the write-list carries the per-LEG dicts while rip-up
            # sees only this merged dict via routed_results -- carry the legs
            # so rip_up_net/restore_net can remove/restore the actual
            # write-list members (value-equality never matched the merged
            # dict, so ripped legs' copper shipped alongside the reroute).
            merged['leg_results'] = leg_results
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

        # #277: the standard coupled route must ALWAYS launch from a terminal's
        # fanout STUB FREE END (on the stub's actual layer), not the pad -- exactly
        # like the hybrid path already does (hyb_eps). The stub end is clear of the
        # pad/part congestion, and after a layer swap it is on the swapped layer, so
        # launching from the pad makes the leg double back to the pad on the wrong
        # layer (redundant copper + a dangling swapped stub end; lumenpnp USB_D
        # swapped to B.Cu but the middle returned to the F.Cu pad). Fall back to the
        # pad only when a terminal has no stub (a bare pad).
        def _leg_endpoint(term):
            stub_ep = _terminal_stub_endpoint(pcb_data, term, config)
            return stub_ep if stub_ep is not None else _make_endpoint(term, config)
        endpoints = (_leg_endpoint(term_a), _leg_endpoint(term_b))

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
        elif _pn_self_overlaps(result.get('new_segments', []), pair.p_net_id,
                               pair.n_net_id, config, pcb_data):
            # The leg's P/N pinch below clearance -- against its own connectors or
            # the partner's carried-through fanout stub (#215); no hybrid fallback
            # here, so reject and let it go single-ended
            failed_reason = "P/N graze below clearance"
        elif result.get('target_base_dir') is None:
            failed_reason = "missing target direction"

        if failed_reason:
            # Before giving up, route THIS leg as a 2-terminal HYBRID: a clean
            # coupled middle between the terminals (launched from their fanout STUB
            # TIPS, clear of the BGA) + single-ended escape legs. A congested BGA
            # can't launch a *coupled* pair from the pads (CK1's DDR3-bank escape),
            # but each net escapes individually and couples once clear; the coupled
            # middle shortens until reachable and the legs single-end the rest.
            leg_obstacles = build_diff_pair_leg_obstacles(
                state.base_obstacles, pcb_data, config,
                state.routed_net_ids, state.remaining_net_ids,
                state.all_unrouted_net_ids, pair.p_net_id, pair.n_net_id,
                state.gnd_net_id, state.track_proximity_cache, state.layer_map)
            hyb_eps = (_terminal_stub_endpoint(pcb_data, term_a, config) or endpoints[0],
                       _terminal_stub_endpoint(pcb_data, term_b, config) or endpoints[1])
            # Only THIS leg's two terminals (term_a/term_b are (P pad, N pad)) must
            # connect -- the net's OTHER terminals are separate legs routed later, so
            # the hybrid's connectivity gate must not demand the whole net (#250).
            _leg_pads = {pair.p_net_id: [term_a[0], term_b[0]],
                         pair.n_net_id: [term_a[1], term_b[1]]}
            hyb = _route_direct_coupled_middle(
                pcb_data, pair, config, obstacles, config.layers,
                leg_obstacles=leg_obstacles, endpoints=hyb_eps, terminal_pads=_leg_pads)
            if (hyb is not None and not hyb.get('failed') and
                    not _pn_tracks_cross(hyb.get('new_segments', []), pair.p_net_id, pair.n_net_id) and
                    not _crosses_committed_legs(hyb.get('new_segments', []), committed_segments)):
                print(f"  Leg {i + 1} via hybrid (coupled middle + single-ended escapes)")
                add_route_to_pcb_data(pcb_data, hyb, debug_lines=config.debug_lines)
                leg_results.append(hyb)
                committed_segments.extend(hyb.get('new_segments', []))
                forced_dir_next = None
                continue
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
