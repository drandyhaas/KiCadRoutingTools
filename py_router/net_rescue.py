"""Last-chance per-net fine-parameter rescue pass (issues #331 / #371).

Runs at the very end of batch_route, after the main loop, the rip-up ladder,
the reroute loop, Phase 3 tap completion and the #134 recovery have all had
their shot. Any net still failed outright (no result) or partially connected
(failed pads) gets a SCOPED retry at finer parameters:

  rung 0   - the run's own width/clearance at a finer grid (pure grid-
             resolution failures route cleanly here: rp2350_fpga_eensy
             /T8F49I2X/R failed at 0.05 and routed at 0.025 unchanged)
  rung 1.. - fab-floor track width with the clearance stepped down from
             nominal toward the fab floor (#371: the neck-down ladder was
             power-net-only; a below-layer-width retry gains nothing on the
             shared obstacle map because its inflation is baked at the layer
             width, so the neck-down needs this rebuilt scoped map anyway)

Design constraints (#331/#371 review):
  - NO rip-up here: the rescue routes through free space only, and each
    committed edge is verified to reduce the net's component count on the
    real board (else it is removed again) - a failed rescue leaves the
    board untouched.
  - Scoped, never board-global: a fresh obstacle map is built on a small
    window around the remaining GAP (the #329/#134 partial restores mean
    the gap is usually a short missing link, not the whole net).
    Board-global 0.025 is ~9M cells/layer and would exhaust memory. The
    compute limits (grid, window margin, cell budget, max attempts) are
    the RESCUE_* constants in routing_defaults. Gap LENGTH is deliberately
    unbounded (#516): the cell budget is what bounds compute — a longer
    gap gets a coarser grid, not a bigger search.
  - Always on, no flags: lives inside batch_route so the CLI and the GUI
    plugin share it (CLAUDE.md parity rule). Set KICAD_NET_RESCUE=0 to
    disable for A/B debugging.
  - Every below-nominal clearance actually routed is recorded in the
    clearance ledger, so check_drc grades at the true floor (#226).
"""

import env_knobs
import math
import os
import collections as _c
import time
from dataclasses import replace
from typing import Dict, List, Optional, Tuple

import routing_defaults as defaults
from geometry_utils import UnionFind
from routing_state import record_net_event
from terminal_colors import RED, GREEN, YELLOW, RESET


def _net_component_info(pcb_data, net_id):
    """Connected components of a net's pads+copper on the REAL board.

    Uses the authoritative overlap-aware connectivity graph (cap overlap,
    T-junctions, zones, via-in-pad all count - the same definition the final
    grading uses, #317). Returns (num_pad_components, comp_points, comp_pads):
      comp_points: {component id -> [(x, y), ...]} pad centers plus that
                   component's segment endpoints (candidate join points)
      comp_pads:   {component id -> [Pad, ...]}
    Copper-only islands (no pad) are not counted as components; a pad that
    cannot be tied to any copper gets its own unique (negative) component.
    """
    from check_connected import check_net_connectivity

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
    net_zones = [z for z in (getattr(pcb_data, 'zones', None) or [])
                 if z.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])

    res = check_net_connectivity(net_id, net_segments, net_vias, net_pads,
                                 net_zones, return_graph=True,
                                 pcb_data=pcb_data)
    graph = res.get('graph') or {}
    uf = UnionFind()
    for a, b in graph.get('edges', []):
        uf.union(a, b)

    pad_repr = graph.get('pad_index_repr', {})
    comp_points: Dict[int, List[Tuple[float, float]]] = {}
    comp_pads: Dict[int, list] = {}
    next_unique = -1
    for idx, pad in enumerate(net_pads):
        rep = pad_repr.get(idx)
        if rep is not None:
            cid = uf.find(rep)
        else:
            cid = next_unique
            next_unique -= 1
        comp_points.setdefault(cid, []).append((pad.global_x, pad.global_y))
        comp_pads.setdefault(cid, []).append(pad)
    # Segment endpoints of PAD-bearing components are join points too (the
    # gap usually ends at a stub tip, not at the far pad - bitaxe RST_N's
    # remaining gap was 0.25mm off the pad while the trunk crossed the board).
    for si, seg in enumerate(net_segments):
        cid = uf.find(2 * si)  # segment i's endpoints are point ids 2i/2i+1
        if cid in comp_pads:
            comp_points[cid].append((seg.start_x, seg.start_y))
            comp_points[cid].append((seg.end_x, seg.end_y))
    # #545 F11: the components' VIAS are join points too (graph-authoritative
    # membership, so via-in-pad-only islands and mid-track stitching vias
    # count). Without them _closest_pair measured pad-to-pad / endpoint-to-
    # endpoint only, overstating the true copper gap -- which inflates `half`
    # in _attempt_edge and makes _choose_grid pick a COARSER rescue grid,
    # the opposite of what a tight rescue wants. Vias only, per #545 scope
    # (no zone join points).
    for vidx, vpid in (graph.get('via_index_repr') or {}).items():
        cid = uf.find(vpid)
        if cid in comp_pads:
            try:
                _v = net_vias[vidx]
            except (IndexError, TypeError):
                continue
            comp_points[cid].append((_v.x, _v.y))
    return len(comp_pads), comp_points, comp_pads


def _main_component(comp_pads):
    """Deterministic 'biggest' component: most pads, ties by smallest id."""
    return sorted(comp_pads.items(), key=lambda kv: (-len(kv[1]), kv[0]))[0][0]


def _sampled(points, cap=300):
    if len(points) <= cap:
        return points
    step = len(points) // cap + 1
    return points[::step]


def _closest_pair(points_a, points_b):
    """(dist, ax, ay, bx, by) of the closest pair, or None."""
    best = None
    for ax, ay in _sampled(points_a):
        for bx, by in _sampled(points_b):
            d = math.hypot(ax - bx, ay - by)
            if best is None or d < best[0]:
                best = (d, ax, ay, bx, by)
    return best


def _choose_grid(config, half_size):
    """Finest rescue grid whose window fits the per-layer cell budget.

    May come out COARSER than the run's own grid_step for a very long gap
    (#516): the cell budget, not the gap length, is what bounds rescue
    compute, so the grid keeps doubling until the window fits.
    """
    fine = min(config.grid_step, defaults.RESCUE_GRID_STEP)
    while (2 * half_size / fine) ** 2 > defaults.RESCUE_MAX_WINDOW_CELLS:
        fine *= 2
    return fine


def _rescue_rungs(config, fine_grid, pcb_data, net_id):
    """The scoped retry ladder for one net (see module docstring)."""
    from plane_pad_tap import _clearance_ladder, fab_floor_clearance_track

    # Rescue runs at ITS OWN budget, not the step's: the windowed scope
    # bounds the search spatially, and min()-ing with a 200k step default
    # throttled the fine-grid rungs into false negatives (SDC0_CMD's
    # human-obvious path needs ~1.4M cumulative iterations at 0.025 --
    # found instantly once uncapped; 'no route' at 200k for years of runs).
    max_iters = max(config.max_iterations, defaults.RESCUE_MAX_ITERATIONS)
    rungs = []
    # Rung 0: nominal geometry at the finer grid only. Skipped when the run
    # already routes at (or below) the rescue grid - identical to what failed.
    if fine_grid < config.grid_step - 1e-9:
        rungs.append(replace(config, grid_step=fine_grid,
                             max_iterations=max_iters))

    fab_clear, fab_track = fab_floor_clearance_track(pcb_data)
    nominal_w = config.get_net_track_width(net_id, config.layers[0])
    # Floor rule (2026-08-06): min(nominal, fab_track, netclass width) --
    # a class-declared width is designer intent and may sit below the
    # standard fab floor (advanced-clamped at load; ecp5 /PF37- routes at
    # its class 0.0762 where 0.0889 is sealed).
    rescue_track = min(nominal_w, fab_track,
                       (config.netclass_width_floors or {}).get(
                           net_id, nominal_w))
    power_widths = dict(config.power_net_widths)
    power_widths.pop(net_id, None)  # this net necks down; other nets are obstacles
    floor_clearance = config.clearance
    for clearance in _clearance_ladder(config.clearance, fab_clear,
                                       defaults.RESCUE_CLEARANCE_STEPS):
        floor_clearance = clearance
        rungs.append(replace(config, grid_step=fine_grid, clearance=clearance,
                             track_width=rescue_track, layer_widths={},
                             power_net_widths=power_widths,
                             max_iterations=max_iters))
    # Via step-down rungs: the ladder above never shrinks the VIA, so a
    # trivially-closable cross-layer gap stays unroutable when the run's via
    # (e.g. 0.5/0.3) cannot drop inside a dense pin field -- the tail-triage
    # "missing layer transition" class (same-net copper <0.25mm away on
    # another layer, one via finishes the net). Re-try the floor rung with
    # each smaller via from the fab ladder (0.30/0.15 fine rung, then the
    # advanced 0.25/0.15), mirroring the plane-tap escalation. Only rungs
    # strictly smaller than the run's via are added; the escalation warning
    # matches the tap path's.
    from fab_tiers import fab_floor_ladder, warn_fab_escalation
    n_layers = len(pcb_data.board_info.copper_layers) or 2
    _ladder = fab_floor_ladder(n_layers)
    for floor in _ladder:
        v_dia, v_drill = floor['via_diameter'], floor['via_drill']
        if v_dia >= config.via_size - 1e-9:
            continue
        if v_dia < _ladder[0]['via_diameter'] - 1e-9:
            warn_fab_escalation(f"net rescue net_{net_id}")
        rungs.append(replace(config, grid_step=fine_grid,
                             clearance=floor_clearance,
                             track_width=rescue_track, layer_widths={},
                             power_net_widths=power_widths,
                             via_size=v_dia, via_drill=v_drill,
                             max_iterations=max_iters))
    return rungs


def _fence_window(obstacles, window, cfg):
    """Wall the window edge so the A* cannot leave the stamped region.

    build_base_obstacle_map fences rectangular boards itself (the window's
    bounds are its board_bounds, and make_local_window drops a rectangular
    outline so the rect band applies). A POLYGONAL board keeps its outline,
    and the polygon path blocks outside the OUTLINE, not outside the WINDOW -
    an interior window would leave the search open into space whose copper
    was never stamped. Stamp the rect band at the window bounds explicitly.
    """
    bi = window.board_info
    if not (bi.board_outline or (getattr(bi, 'board_outlines', None) or [])):
        return  # rectangular: already fenced by the base build
    from obstacle_map import _add_rectangular_edge_obstacles
    from routing_config import GridCoord

    coord = GridCoord(cfg.grid_step)
    min_x, min_y, max_x, max_y = bi.board_bounds
    edge_clearance = (cfg.board_edge_clearance if cfg.board_edge_clearance > 0
                      else cfg.clearance)
    track_expand = coord.to_grid_dist(edge_clearance + cfg.track_width / 2)
    via_expand = coord.to_grid_dist_safe(edge_clearance + cfg.via_size / 2)
    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    _add_rectangular_edge_obstacles(obstacles, coord, len(cfg.layers),
                                    gmin_x, gmin_y, gmax_x, gmax_y,
                                    track_expand, via_expand)


def _attempt_edge(pcb_data, net_id, gap, config, net_clearances,
                  strict_endpoints: bool = False, island_points=None):
    """Try to route one gap inside a scoped window. Returns (result, cfg_used)
    or (None, None). Routes through free space only - no rip-up.

    strict_endpoints (#570): when the anchor split cannot resolve the gap's
    two sides, FAIL instead of falling back to the window's largest-two
    fragments. The fallback serves the rescue's own gap analysis (its gaps
    come from net-endpoint structure, so the fragments ARE the gap), but a
    caller welding a SPECIFIC pair of points (the oracle's exact-fill strap)
    must not accept a route between two unrelated fragments: ecp5 GND
    'welded' a 0.35mm In1 pinch with a 0.14mm F.Cu segment 7mm away, claimed
    the link fixed every round, and the identical debris re-stacked forever."""
    from obstacle_map import build_base_obstacle_map
    from plane_pad_tap import make_local_window
    from routing_config import GridCoord
    from single_ended_routing import route_net_with_obstacles

    d, ax, ay, bx, by = gap
    cx, cy = (ax + bx) / 2, (ay + by) / 2
    half = max(d / 2 + defaults.RESCUE_WINDOW_MARGIN,
               defaults.RESCUE_MIN_WINDOW_HALF)
    fine_grid = _choose_grid(config, half)
    window = make_local_window(pcb_data, cx, cy, half)

    # BGA exclusion zones are ADVISORY router policy, not DRC geometry -- and
    # the last-resort rescue is exactly where they turn self-defeating: the
    # human-routable SDC0_CMD path skirts U1's top ball row and crosses U7's
    # sparse 0.9mm field on F.Cu, DRC-clean, but every zoned retry frontier-
    # exhausted in ~6k iterations because we walled the only corridor west
    # ourselves. Rescue routes are windowed, connectivity-verified, and
    # DRC-graded afterwards, so drop the zones here (real copper/clearance
    # obstacles still apply in full).
    if config.bga_exclusion_zones:
        config = replace(config, bga_exclusion_zones=[])
    for cfg in _rescue_rungs(config, fine_grid, pcb_data, net_id):
        # Rung 0 keeps the run's exact clearance semantics (incl. per-netclass
        # spacing). The neck-down rungs' whole point is spacing below nominal
        # FOR THE RESCUED NET, so its own map entry is dropped (the builder's
        # routing-side floor maxes over the routed nets, and the rescued net's
        # nominal class value would snap the necked cfg.clearance back up).
        # Foreign obstacles KEEP their per-net class clearance: the builder
        # prices each obstacle at max(cfg.clearance, its own class), so a
        # POWER_HI via stays 0.25-priced even while the rescued net necks down
        # (dropping the whole map under-blocked exactly that cross-class pair).
        at_nominal = cfg.clearance >= config.clearance - 1e-9
        rung_clearances = net_clearances
        if not at_nominal and net_clearances:
            rung_clearances = {k: v for k, v in net_clearances.items() if k != net_id}
        obstacles = build_base_obstacle_map(
            window, cfg, [net_id],
            net_clearances=rung_clearances)
        _fence_window(obstacles, window, cfg)
        # #470: existing same-net vias/through-holes in the window are FREE
        # layer transitions. The rescue map never registered them (unlike the
        # main-loop working map), so a rescue route paid full via cost
        # everywhere, felt no pull toward the net's own barrels, and dropped a
        # fresh via sub-mm from an existing one (USB_D_P's 0.94mm pair).
        # Conversion suppresses the duplicate emit at these cells
        # (get_same_net_through_hole_positions), completing the reuse.
        from routing_context import _add_free_via_positions
        _add_free_via_positions(obstacles, window, [net_id], cfg)
        # Same-net via/drill spacing (the custody-pass h2h bug, cy GND<->GND
        # drill): seeding alone lets the search drop a NEW via 1-2 cells off
        # an existing same-net barrel -- sub-h2h drills, a real KiCad
        # violation (hole clearance is net-blind). Every OTHER seeding site
        # (routing_context prepare paths, diff pairs) pairs the seeding with
        # these guards; rescue was the one that didn't. The seeded free cells
        # still override AT the barrel, so the semantics are "reuse it
        # exactly, or keep your distance" -- reuse itself is unaffected.
        from obstacle_map import (add_same_net_via_clearance,
                                  add_same_net_pad_drill_via_clearance)
        add_same_net_via_clearance(obstacles, window, net_id, cfg)
        add_same_net_pad_drill_via_clearance(obstacles, window, net_id, cfg)
        # Constrain the route to the window: drop endpoints outside it and keep the
        # source/target overrides from punching the fence, so the A* can never leave
        # the stamped region and cross foreign copper the window never modelled (#396).
        bounds = None
        if window.board_info.board_bounds:
            wcoord = GridCoord(cfg.grid_step)
            _wx0, _wy0, _wx1, _wy1 = window.board_info.board_bounds
            _g0 = wcoord.to_grid(_wx0, _wy0)
            _g1 = wcoord.to_grid(_wx1, _wy1)
            bounds = (_g0[0], _g0[1], _g1[0], _g1[1])
        if env_knobs.RESCUE_DEBUG_VIA:
            # Debug probe: report the window box and whether a named via cell
            # is blocked in THIS rung's map (KICAD_RESCUE_DEBUG_VIA="x,y").
            try:
                from routing_config import GridCoord as _GC
                _dx, _dy = (float(t) for t in
                            env_knobs.RESCUE_DEBUG_VIA.split(','))
                _c = _GC(cfg.grid_step)
                _g = _c.to_grid(_dx, _dy)
                print(f"    RESCUE-DEBUG net={net_id} window="
                      f"{window.board_info.board_bounds} via_size={cfg.via_size} "
                      f"probe=({_dx},{_dy}) blocked={obstacles.is_via_blocked(_g[0], _g[1])} "
                      f"segs_in_window={len(window.segments)}")
            except Exception as _e:
                print(f"    RESCUE-DEBUG failed: {_e}")
        # Aim the route at THIS gap's two islands. The window crop severs
        # copper, and get_net_endpoints' largest-two-groups split then picks
        # two fragments of the same trunk as the route's sides, dropping the
        # island the gap analysis chose (USB_D_P: target set was the far
        # trunk half parked in the fence ring; the BGA ball was never aimed
        # at, so the small-via rungs never even pointed at the pocket).
        from connectivity import get_net_endpoints_anchor_split
        src_over, tgt_over, split_err = get_net_endpoints_anchor_split(
            window, net_id, cfg, (ax, ay), (bx, by),
            island_points=island_points)
        if split_err:
            if strict_endpoints:
                # #570: no anchors for THIS gap's sides -> nothing this
                # window can honestly route. Do not weld a random pair.
                continue
            src_over = tgt_over = None
        result = route_net_with_obstacles(window, net_id, cfg, obstacles, bounds=bounds,
                                          sources_override=src_over,
                                          targets_override=tgt_over)
        if result and not result.get('failed'):
            if _result_escapes_window(result, window, cfg):
                print(f"    rescue rung rejected: route escaped the window "
                      f"bounds (fence leak) net={net_id}")
                continue
            _tie_bad = _tie_band_violations(
                pcb_data, net_id, cfg, result.get('new_segments') or [])
            if _tie_bad:
                _s, _p, _d = _tie_bad[0]
                print(f"    rescue rung rejected: {len(_tie_bad)} segment(s) "
                      f"ride net-tie partner pad "
                      f"{_p.component_ref}.{_p.pad_number} sub-clearance "
                      f"beyond KiCad's waiver (worst {_d:.3f}mm)")
                continue
            return result, cfg
    return None, None


def _tie_band_violations(pcb_data, net_id, cfg, segments):
    """Rescue copper riding a net-tie PARTNER pad's band beyond KiCad's waiver.

    A Kelvin-shunt sense net can only exit its tab through the partner pad's
    copper, and the net-tie corridor legally allows that passage -- but
    KiCad's waiver (DRC_ENGINE::IsNetTieExclusion) is per ITEM: it forgives a
    segment only when its contact with the partner lies on the tied net's OWN
    pad. A rescue approach that then RUNS ALONGSIDE the partner pad
    sub-clearance without touching the own pad ships real violations
    (cynthion R1: kicad-cli clearance 0.011-0.039 actual vs 0.100 +
    shorting_items; the R2/R59 baseline pad-segment DRC is the same class).
    Reuses check_drc's exact geometry AND waiver so this gate matches the
    grader; graded at the RUNG's own clearance, so the accepted neck-to-floor
    residual class (#276) is not re-flagged. Checked against the FULL board
    (the window crop may drop the footprint the waiver needs).

    Returns [(seg, pad, dist)] offenders; empty = clean."""
    try:
        exempt = pcb_data.net_tie_exempt_pad_ids(net_id)
    except Exception:
        return []
    if not exempt:
        return []
    from check_drc import check_pad_segment_overlap, _net_tie_span_waived
    partners = [p for plist in pcb_data.pads_by_net.values() for p in plist
                if id(p) in exempt and p.net_id != net_id]
    if not partners:
        return []
    bad = []
    for seg in segments:
        for pad in partners:
            viol, dist, _pt = check_pad_segment_overlap(
                pad, seg, cfg.clearance, cfg.layers)
            if viol and not _net_tie_span_waived(pcb_data, seg, net_id, pad,
                                                 cfg.clearance):
                bad.append((seg, pad, dist))
    return bad


def _result_escapes_window(result, window, cfg):
    """True when any NEW copper lands outside the rescue window (+1 grid step).

    Belt-and-braces behind the #396 fence: the window obstacle map models
    nothing beyond its bounds, so copper placed out there was routed blind
    (butterstick DQ11: a #338 exemption hole let the A* slip past the fence
    and drop a via on unseen +3V3 copper). Any escape = the rung failed."""
    bb = window.board_info.board_bounds
    if not bb:
        return False
    tol = cfg.grid_step + 1e-6
    x0, y0, x1, y1 = bb[0] - tol, bb[1] - tol, bb[2] + tol, bb[3] + tol

    def _out(x, y):
        return not (x0 <= x <= x1 and y0 <= y <= y1)

    for s in result.get('new_segments', []) or []:
        if _out(s.start_x, s.start_y) or _out(s.end_x, s.end_y):
            return True
    for v in result.get('new_vias', []) or []:
        if _out(v.x, v.y):
            return True
    return False


def _unconnected_pads_info(comp_pads):
    """failed_pads_info rows for every pad outside the main component."""
    main = _main_component(comp_pads)
    out = []
    for cid, pads in sorted(comp_pads.items()):
        if cid == main:
            continue
        for p in pads:
            out.append({'component_ref': p.component_ref,
                        'pad_number': p.pad_number,
                        'x': p.global_x, 'y': p.global_y})
    return out


def rescue_failed_nets(state, single_ended_nets, net_clearances=None,
                       progress_callback=None, cancel_check=None):
    """Scoped fine-parameter rescue for every still-failed/partial net.

    Returns a summary dict ({'attempted', 'recovered', 'improved',
    'unchanged', 'pads_reconnected', 'widths', 'time'}) or None when there was
    nothing to rescue (or KICAD_NET_RESCUE=0).

    'widths' is {net: {'requested_mm', 'delivered_mm', and when the rescue emitted width-bearing copper, rescue_* keys describing THIS RESCUE's segments only}} for every net the
    rescue touched -- the width provenance that used to vanish: the ladder
    re-routes at the floor and reports the net `recovered` with
    `failed_single` empty, so a 0.8mm request silently shipping as 0.15mm
    copper was visible only by measuring the board (test-board run 5,
    journal [6]). Consumers: route.py exports this dict verbatim under
    summary['rescue'].

    progress_callback(i, n, "Rescue: <net>") fires per candidate net (#527 --
    this phase can dominate a step's wall clock and used to sit behind one
    static GUI message); cancel_check is honored at net and edge-attempt
    boundaries -- rescue copper is purely additive, so aborting mid-pass
    ships whatever was recovered so far.
    """
    if not env_knobs.NET_RESCUE:
        return None
    from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
    from plane_pad_tap import note_clearance_used

    pcb_data = state.pcb_data
    config = state.config
    routed_results = state.routed_results

    candidates = []
    for net_name, net_id in sorted(set(single_ended_nets)):
        if net_id not in pcb_data.nets:
            continue
        if len(pcb_data.pads_by_net.get(net_id, [])) < 2:
            continue
        result = routed_results.get(net_id)
        if result is None:
            candidates.append((net_name, net_id, 'failed'))
        elif result.get('failed_pads_info'):
            candidates.append((net_name, net_id, 'partial'))
    if not candidates:
        return None

    print(f"\nPer-net fine-parameter rescue (#331/#371): "
          f"{len(candidates)} candidate net(s)")
    summary = {'attempted': 0, 'recovered': [], 'improved': [],
               'unchanged': [], 'pads_reconnected': 0, 'widths': {},
               'time': 0.0}
    pass_start = time.time()

    for _cand_idx, (net_name, net_id, kind) in enumerate(candidates):
        if cancel_check and cancel_check():
            print("  Rescue cancelled")
            break
        if progress_callback:
            progress_callback(_cand_idx + 1, len(candidates),
                              f"Rescue: {net_name}")
        net_start = time.time()
        num0, comp_points, comp_pads = _net_component_info(pcb_data, net_id)
        if num0 <= 1:
            # Checker says connected (zone credit etc.) - nothing to rescue,
            # and accounting stays whatever the run already decided.
            continue
        summary['attempted'] += 1
        print(f"  Rescuing {net_name} ({kind}, {num0} disconnected parts)")

        edge_results = []
        used_widths = []
        failed_gaps = set()
        attempts = 0
        num = num0
        while attempts < defaults.RESCUE_MAX_EDGES_PER_NET and num > 1:
            if cancel_check and cancel_check():
                break  # additive copper: keep what landed, stop attempting
            main = _main_component(comp_pads)
            gaps = []
            for cid, pts in comp_points.items():
                if cid == main or cid not in comp_pads:
                    continue
                pair = _closest_pair(comp_points[main], pts)
                if pair is None:
                    continue
                key = (round((pair[1] + pair[3]) / 2, 2),
                       round((pair[2] + pair[4]) / 2, 2))
                if key not in failed_gaps:
                    gaps.append((key, pair, cid))
            if not gaps:
                break
            gaps.sort(key=lambda g: g[1][0])
            key, gap, gap_cid = gaps[0]
            attempts += 1
            # The rescue KNOWS the partition -- hand the island's own points
            # to the window split so the route is aimed island<->trunk by
            # MEMBERSHIP. Proximity assignment put trunk cells on both sides
            # of a short gap (dilemma M_COL4_L, 2.48mm): the route joined
            # trunk-to-trunk, the verify below undid it, and the net shipped
            # open despite a routable window.
            _isl = {(round(x, 3), round(y, 3))
                    for x, y in comp_points.get(gap_cid, [])}
            _isl.update((round(p.global_x, 3), round(p.global_y, 3))
                        for p in comp_pads.get(gap_cid, []))
            result, used_cfg = _attempt_edge(pcb_data, net_id, gap, config,
                                             net_clearances,
                                             island_points=_isl)
            if result is None:
                failed_gaps.add(key)
                continue
            add_route_to_pcb_data(pcb_data, result,
                                  debug_lines=config.debug_lines)
            num_after, comp_points, comp_pads = _net_component_info(pcb_data,
                                                                    net_id)
            if num_after >= num:
                # Window connectivity lied (copper outside the window) - undo.
                remove_route_from_pcb_data(pcb_data, result)
                num, comp_points, comp_pads = _net_component_info(pcb_data,
                                                                  net_id)
                failed_gaps.add(key)
                continue
            num = num_after
            if used_cfg.clearance < config.clearance - 1e-9:
                note_clearance_used(pcb_data, used_cfg.clearance)
            edge_results.append(result)
            used_widths.append(used_cfg.track_width)
            print(f"    {GREEN}rescued a gap{RESET}: grid {used_cfg.grid_step:g}, "
                  f"clearance {used_cfg.clearance:g}, track "
                  f"{used_cfg.track_width:g} ({len(result['new_segments'])} segs, "
                  f"{len(result.get('new_vias', []))} vias, "
                  f"{result.get('iterations', 0)} iters)")

        elapsed = time.time() - net_start
        if not edge_results:
            summary['unchanged'].append(net_name)
            record_net_event(state, net_id, "rescue_failed",
                             {"components": num0, "attempts": attempts,
                              "time_s": round(elapsed, 2)})
            print(f"    {RED}rescue failed{RESET} ({elapsed:.1f}s)")
            continue

        merged = {
            'net_name': net_name,
            'net_id': net_id,
            'new_segments': [s for r in edge_results for s in r['new_segments']],
            'new_vias': [v for r in edge_results for v in r.get('new_vias', [])],
            'iterations': sum(r.get('iterations', 0) for r in edge_results),
            'is_rescue': True,
        }
        fully = num <= 1
        if kind == 'failed':
            state.results.append(merged)
            if not fully:
                merged['failed_pads_info'] = _unconnected_pads_info(comp_pads)
            routed_results[net_id] = merged
            if net_id in state.remaining_net_ids:
                state.remaining_net_ids.remove(net_id)
            if net_id not in state.routed_net_ids:
                state.routed_net_ids.append(net_id)
        else:
            # Partial multipoint net: fold the rescue copper INTO the net's
            # AUTHORITATIVE result. A separate `merged` entry in
            # state.results is dropped by route.py's #87 superseded-result
            # filter (only routed_results survive) and the phantom pass then
            # orphan-strips its copper from the board -- successful partial
            # rescues shipped BROKEN (the k_seam SDC0_CMD netstory shows
            # rescue_succeeded fully_connected=True while the file stayed
            # open; reproduced deterministically on the parity board).
            prev = routed_results[net_id]
            prev['new_segments'] = (list(prev.get('new_segments') or [])
                                    + list(merged['new_segments']))
            prev['new_vias'] = (list(prev.get('new_vias') or [])
                                + list(merged['new_vias']))
            prev_failed = prev.get('failed_pads_info') or []
            still = ({} if fully else
                     {(p['component_ref'], p['pad_number'])
                      for p in _unconnected_pads_info(comp_pads)})
            new_failed = [p for p in prev_failed
                          if (p['component_ref'], p['pad_number']) in still]
            reconnected = len(prev_failed) - len(new_failed)
            prev['failed_pads_info'] = new_failed
            if reconnected and 'tap_pads_connected' in prev:
                prev['tap_pads_connected'] = (prev.get('tap_pads_connected', 0)
                                              + reconnected)
            summary['pads_reconnected'] += reconnected

        record_net_event(state, net_id, "rescue_succeeded",
                         {"fully_connected": fully,
                          "edges_routed": len(edge_results),
                          "components_before": num0, "components_after": num,
                          "time_s": round(elapsed, 2)})
        bucket = 'recovered' if fully else 'improved'
        summary[bucket].append(net_name)
        # Width provenance: widths are REQUESTS, and the rescue is the single
        # largest producer of quietly-thinner copper in the chain. Record what
        # was asked and what was delivered so the JSON tells the truth without
        # anyone re-measuring the board.
        _req_w = config.get_net_track_width(net_id, config.layers[0])
        # A HISTOGRAM, not a scalar. `delivered_mm` was min(used_widths) under
        # a name that reads as "what it got", and this net's copper is
        # routinely MIXED: one rail came out 59 segments at 0.0889, 3 at
        # 0.1998 and 67 at 0.2, reported as 0.0889. Both readings mislead in
        # opposite directions -- the minimum makes a mostly-wide rail look
        # uniformly thin, and a mean would hide the thin run entirely.
        #
        # Ampacity is set by the NARROWEST series segment, so `min` stays and
        # keeps its meaning; `by_width` is what tells a reader whether that
        # minimum is the whole net or a short neck. Taken from the emitted
        # segments rather than the requested widths, because a request is not
        # a result -- impedance and neckdown both diverge from it.
        _hist = _c.Counter(round(float(getattr(sg, 'width', 0.0)), 4)
                           for sg in merged['new_segments']
                           if getattr(sg, 'width', None))
        _del_w = min(_hist) if _hist else (min(used_widths) if used_widths
                                           else _req_w)
        _w = {
            'requested_mm': round(_req_w, 4),
            'delivered_mm': round(_del_w, 4),          # the NARROWEST, as before
        }
        if _hist:
            # SCOPED, and named so. This histogram covers the copper THIS
            # RESCUE emitted -- not the net's whole copper. That distinction
            # is the difference between informing a reader and misleading one:
            # the finding that motivated it measured +1V2 across the finished
            # board as 59 segments @ 0.0889, 3 @ 0.1998 and 67 @ 0.2, but only
            # the 0.0889 run came from the rescue. An unscoped name here would
            # show a single 0.0889 bin and look like CORROBORATION of exactly
            # the "uniformly thin" reading the finding was warning against.
            #
            # Emitted widths, not the requested rung: a request is not a
            # result, and neckdown and the oracle's width clamp both diverge
            # from it (measured: a 0.2 rung emitting a 0.1 and a 0.4 segment).
            _w.update({
                'rescue_min_mm': round(_del_w, 4),
                'rescue_max_mm': round(max(_hist), 4),
                'rescue_segments_emitted': sum(_hist.values()),
                'rescue_by_width': {f'{w:g}': n for w, n in sorted(_hist.items())},
                'scope': ('THIS RESCUE ONLY -- not the net. Counts are as '
                          'EMITTED, before cycle-prune drops redundant loop '
                          'segments, so the board can carry fewer. For the '
                          "net's whole width profile, measure the board."),
            })
        else:
            # No emitted segment carried a width (a vias-only edge). Say that,
            # rather than asserting a min and max over an empty histogram.
            _w['scope'] = ('this rescue emitted no width-bearing segment; '
                           'delivered_mm falls back to the requested rung')
        summary['widths'][net_name] = _w
        _thin = (f", {YELLOW}width {_del_w:g} vs requested {_req_w:g}{RESET}"
                 if _del_w < _req_w - 1e-9 else "")
        print(f"    {GREEN}{'fully reconnected' if fully else 'improved'}{RESET} "
              f"({num0} -> {num} parts, {elapsed:.1f}s{_thin})")

    summary['time'] = round(time.time() - pass_start, 2)
    if summary['attempted']:
        print(f"Rescue pass: {len(summary['recovered'])} recovered, "
              f"{len(summary['improved'])} improved, "
              f"{len(summary['unchanged'])} unchanged "
              f"({summary['time']:.1f}s)")
    return summary if summary['attempted'] else None


# ---------------------------------------------------------------------------
# Terminal geometry escalation ("better than shipping opens", 2026-08-05)
# ---------------------------------------------------------------------------

def _escalation_ladder(config, pcb_data, net_id):
    """The terminal escalation rungs for one net: 2-3 whole-net retry configs
    whose track width AND via size march down TOGETHER from the net's current
    geometry to the fab-tier floor (fab_floor_clearance_track for the track,
    fab_floor_ladder for the via -- the same fab-floor resolution and
    standard->advanced escalation policy the rescue rungs use, including
    warn_fab_escalation). Clearance is deliberately NEVER reduced: that is
    the DRC floor's job, and reducing it here would change grading.

    Returns [] when the net's geometry is already at the floor (nothing to
    march) -- callers skip such nets silently.
    """
    from plane_pad_tap import fab_floor_clearance_track
    from fab_tiers import fab_floor_ladder, warn_fab_escalation

    _fab_clear, fab_track = fab_floor_clearance_track(pcb_data)
    w0 = config.get_net_track_width(net_id, config.layers[0])
    w_floor = min(w0, fab_track,
                  (config.netclass_width_floors or {}).get(net_id, w0))
    width_travel = w0 - w_floor > 1e-9

    n_layers = len(pcb_data.board_info.copper_layers) or 2
    ladder = fab_floor_ladder(n_layers)
    via_rungs = [(f['via_diameter'], f['via_drill']) for f in ladder
                 if f['via_diameter'] < config.via_size - 1e-9]
    if not width_travel and not via_rungs:
        return []  # already at the floor: nothing to march

    m = len(via_rungs)
    n = 3 if (width_travel and m) else (2 if width_travel else m)
    max_iters = max(config.max_iterations, defaults.RESCUE_MAX_ITERATIONS)
    # The marched width must actually APPLY to this net: clear every per-net
    # width override that outranks track_width in get_net_track_width (other
    # nets keep theirs -- they are priced as obstacles at their own widths).
    power_widths = dict(config.power_net_widths)
    power_widths.pop(net_id, None)
    net_layer_widths = dict(config.net_layer_widths or {})
    net_layer_widths.pop(net_id, None)
    net_track_widths = dict(config.net_track_widths or {})
    net_track_widths.pop(net_id, None)
    coplanar_ids = set(config.coplanar_net_ids or set())
    coplanar_ids.discard(net_id)

    rungs = []
    prev_key = None
    for k in range(1, n + 1):
        w = w0 + (w_floor - w0) * k / n if width_travel else w0
        if m:
            v_dia, v_drill = via_rungs[min(m - 1, (k * m - 1) // n)]
        else:
            v_dia, v_drill = config.via_size, config.via_drill
        key = (round(w, 6), round(v_dia, 6), round(v_drill, 6))
        if key == prev_key:
            continue  # discrete via ladder can repeat a rung; dedupe
        prev_key = key
        if m and v_dia < ladder[0]['via_diameter'] - 1e-9:
            warn_fab_escalation(f"terminal escalation net_{net_id}")
        rungs.append(replace(config, track_width=w, layer_widths={},
                             power_net_widths=power_widths,
                             net_layer_widths=net_layer_widths,
                             net_track_widths=net_track_widths,
                             coplanar_net_ids=coplanar_ids,
                             via_size=v_dia, via_drill=v_drill,
                             bga_exclusion_zones=[],
                             max_iterations=max_iters))
    return rungs


def _attempt_net_at_geometry(pcb_data, net_id, cfg, net_clearances,
                             cancel_check=None):
    """One escalation rung: a BOARD-GLOBAL obstacle map rebuilt at the rung's
    geometry (a smaller width/via gains nothing on the run's shared map --
    its inflation is baked at the run geometry, the #371 lesson), then the
    net's gaps are closed one edge at a time, whole-net. NO rip authority:
    copper is only ever ADDED for this net, so no casualties are possible.

    Returns (edge_results, fully_connected). The copper of edge_results is
    already committed to pcb_data; a caller rejecting the rung must undo it
    (remove_route_from_pcb_data, reverse order) so a failed rung leaves the
    board untouched.
    """
    from obstacle_map import (build_base_obstacle_map,
                              add_same_net_via_clearance,
                              add_same_net_pad_drill_via_clearance)
    from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
    from routing_context import _add_free_via_positions
    from single_ended_routing import route_net_with_obstacles
    from connectivity import get_net_endpoints_anchor_split

    obstacles = build_base_obstacle_map(pcb_data, cfg, [net_id],
                                        net_clearances=net_clearances)
    # Same-net barrels are free transitions, at h2h distance (the rescue's
    # #470 + custody-h2h pairing; every seeding site carries both).
    _add_free_via_positions(obstacles, pcb_data, [net_id], cfg)
    add_same_net_via_clearance(obstacles, pcb_data, net_id, cfg)
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, net_id, cfg)

    edge_results = []
    failed_gaps = set()
    attempts = 0
    num, comp_points, comp_pads = _net_component_info(pcb_data, net_id)
    while attempts < defaults.RESCUE_MAX_EDGES_PER_NET and num > 1:
        if cancel_check and cancel_check():
            break
        main = _main_component(comp_pads)
        gaps = []
        for cid, pts in comp_points.items():
            if cid == main or cid not in comp_pads:
                continue
            pair = _closest_pair(comp_points[main], pts)
            if pair is None:
                continue
            key = (round((pair[1] + pair[3]) / 2, 2),
                   round((pair[2] + pair[4]) / 2, 2))
            if key not in failed_gaps:
                gaps.append((key, pair, cid))
        if not gaps:
            break
        gaps.sort(key=lambda g: g[1][0])
        key, gap, gap_cid = gaps[0]
        attempts += 1
        _d, ax, ay, bx, by = gap
        # Aim island<->trunk by MEMBERSHIP (the rescue's split); board-global
        # there is no window crop, so on split failure fall back to the
        # standard largest-two-groups derivation instead of giving up.
        _isl = {(round(x, 3), round(y, 3))
                for x, y in comp_points.get(gap_cid, [])}
        _isl.update((round(p.global_x, 3), round(p.global_y, 3))
                    for p in comp_pads.get(gap_cid, []))
        src_over, tgt_over, split_err = get_net_endpoints_anchor_split(
            pcb_data, net_id, cfg, (ax, ay), (bx, by), island_points=_isl)
        if split_err:
            src_over = tgt_over = None
        result = route_net_with_obstacles(pcb_data, net_id, cfg, obstacles,
                                          sources_override=src_over,
                                          targets_override=tgt_over)
        if not result or result.get('failed'):
            failed_gaps.add(key)
            continue
        add_route_to_pcb_data(pcb_data, result, debug_lines=cfg.debug_lines)
        num_after, comp_points, comp_pads = _net_component_info(pcb_data,
                                                                net_id)
        if num_after >= num:
            # The route joined nothing new (authoritative check) - undo.
            remove_route_from_pcb_data(pcb_data, result)
            num, comp_points, comp_pads = _net_component_info(pcb_data, net_id)
            failed_gaps.add(key)
            continue
        num = num_after
        edge_results.append(result)
    return edge_results, num <= 1


def terminal_geometry_escalation(state, single_ended_nets, net_clearances=None,
                                 progress_callback=None, cancel_check=None):
    """Post-rescue terminal geometry escalation ("better than shipping
    opens", Andy's design, 2026-08-05).

    After the #331/#371 rescue pass has had its shot, every net this run
    would still ship failed or open -- failed_single (no result at all),
    open_single (kept result, pads disconnected) and multipoint nets with
    unconnected pads; membership re-derived here per net with the
    authoritative zone-aware check, since the summary buckets are only
    computed after cleanup -- gets a WHOLE-NET retry with track width and
    via size marching DOWN TOGETHER toward the fab-tier floor
    (_escalation_ladder). Clearance is never reduced.

    A rung is accepted only when the net grades FULLY connected afterward
    (the same authoritative check); an accepted net's copper merges through
    the normal result channels exactly as a rescue-recovered net's does. A
    rejected rung is undone completely, so the worst case is "nothing
    changes". No rip authority: only this net's copper is ever added.

    This pass replaces the removed mid-retry via rung
    (single_ended_routing._via_rung_retry: 459 firings / ~2 wins on the
    wave4 set5 logs) at the only point that mechanism ever paid.

    Returns {'attempted', 'nets': {net_name: "recovered at WxS/D" |
    "unrecovered"}, 'time'} or None when nothing was attempted (or
    KICAD_TERMINAL_ESCALATION=0).
    """
    if not env_knobs.TERMINAL_ESCALATION:
        return None
    from pcb_modification import remove_route_from_pcb_data

    pcb_data = state.pcb_data
    config = state.config
    routed_results = state.routed_results

    # Sweep this run's scope for nets still shipping disconnected pads.
    candidates = []
    for net_name, net_id in sorted(set(single_ended_nets)):
        if cancel_check and cancel_check():
            break
        if net_id not in pcb_data.nets:
            continue
        if len(pcb_data.pads_by_net.get(net_id, [])) < 2:
            continue
        num0, _pts, _pads = _net_component_info(pcb_data, net_id)
        if num0 <= 1:
            continue  # authoritative check says connected (zones included)
        rungs = _escalation_ladder(config, pcb_data, net_id)
        if not rungs:
            continue  # geometry already at the floor: nothing to march
        candidates.append((net_name, net_id, num0, rungs))
    if not candidates:
        return None

    print(f"\nTerminal geometry escalation (better than shipping opens): "
          f"{len(candidates)} candidate net(s)")
    outcomes: Dict[str, str] = {}
    summary = {'attempted': 0, 'nets': outcomes, 'time': 0.0}
    pass_start = time.time()

    for _idx, (net_name, net_id, num0, rungs) in enumerate(candidates):
        if cancel_check and cancel_check():
            print("  Terminal escalation cancelled")
            break
        if progress_callback:
            progress_callback(_idx + 1, len(candidates),
                              f"Terminal escalation: {net_name}")
        summary['attempted'] += 1
        net_start = time.time()
        accepted_cfg = None
        edge_results = []
        n_rungs = len(rungs)
        for k, cfg in enumerate(rungs, 1):
            if cancel_check and cancel_check():
                break
            print(f"  terminal escalation: {net_name} at track "
                  f"{cfg.track_width:.4g} via {cfg.via_size:.4g}/"
                  f"{cfg.via_drill:.4g} (rung {k}/{n_rungs})")
            edges, fully = _attempt_net_at_geometry(
                pcb_data, net_id, cfg, net_clearances, cancel_check)
            if fully:
                accepted_cfg = cfg
                edge_results = edges
                break
            # Rejected rung: undo completely so nothing changes.
            for r in reversed(edges):
                remove_route_from_pcb_data(pcb_data, r)

        elapsed = time.time() - net_start
        if accepted_cfg is None:
            outcomes[net_name] = 'unrecovered'
            record_net_event(state, net_id, "terminal_escalation_failed",
                             {"components": num0, "rungs": n_rungs,
                              "time_s": round(elapsed, 2)})
            print(f"    {RED}terminal escalation: {net_name} unrecovered"
                  f"{RESET} ({elapsed:.1f}s)")
            continue

        geom = (f"{accepted_cfg.track_width:.4g}x"
                f"{accepted_cfg.via_size:.4g}/{accepted_cfg.via_drill:.4g}")
        merged = {
            'net_name': net_name,
            'net_id': net_id,
            'new_segments': [s for r in edge_results
                             for s in r['new_segments']],
            'new_vias': [v for r in edge_results
                         for v in r.get('new_vias', [])],
            'iterations': sum(r.get('iterations', 0) for r in edge_results),
            'is_terminal_escalation': True,
        }
        prev = routed_results.get(net_id)
        if prev is None:
            # Was failed_single: the merged result IS the net's result now
            # (same channels as a rescue-recovered failed net).
            state.results.append(merged)
            routed_results[net_id] = merged
            if net_id in state.remaining_net_ids:
                state.remaining_net_ids.remove(net_id)
            if net_id not in state.routed_net_ids:
                state.routed_net_ids.append(net_id)
        else:
            # Kept-but-open / partial multipoint: fold the copper INTO the
            # net's AUTHORITATIVE result (a separate entry is dropped by the
            # #87 superseded-result filter -- the rescue's partial-branch
            # lesson). Fully connected now, so the pad debt clears.
            prev['new_segments'] = (list(prev.get('new_segments') or [])
                                    + list(merged['new_segments']))
            prev['new_vias'] = (list(prev.get('new_vias') or [])
                                + list(merged['new_vias']))
            prev_failed = prev.get('failed_pads_info') or []
            if prev_failed:
                prev['failed_pads_info'] = []
                if 'tap_pads_connected' in prev:
                    prev['tap_pads_connected'] = (
                        prev.get('tap_pads_connected', 0) + len(prev_failed))
        record_net_event(state, net_id, "terminal_escalation_succeeded",
                         {"geometry": geom, "edges_routed": len(edge_results),
                          "components_before": num0,
                          "time_s": round(elapsed, 2)})
        outcomes[net_name] = f"recovered at {geom}"
        print(f"    {GREEN}terminal escalation: {net_name} recovered at "
              f"{geom}{RESET} ({num0} -> 1 parts, {elapsed:.1f}s)")

    summary['time'] = round(time.time() - pass_start, 2)
    if summary['attempted']:
        _rec = sum(1 for v in outcomes.values() if v != 'unrecovered')
        print(f"Terminal escalation: {_rec} recovered, "
              f"{summary['attempted'] - _rec} unrecovered "
              f"({summary['time']:.1f}s)")
    return summary if summary['attempted'] else None
