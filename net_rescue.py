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
    compute limits (grid, window margin, cell budget, max gap span, max
    attempts) are the RESCUE_* constants in routing_defaults.
  - Always on, no flags: lives inside batch_route so the CLI and the GUI
    plugin share it (CLAUDE.md parity rule). Set KICAD_NET_RESCUE=0 to
    disable for A/B debugging.
  - Every below-nominal clearance actually routed is recorded in the
    clearance ledger, so check_drc grades at the true floor (#226).
"""

import math
import os
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
                                 net_zones, return_graph=True)
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
    """Finest rescue grid whose window fits the per-layer cell budget."""
    fine = min(config.grid_step, defaults.RESCUE_GRID_STEP)
    while (fine < config.grid_step - 1e-9
           and (2 * half_size / fine) ** 2 > defaults.RESCUE_MAX_WINDOW_CELLS):
        fine *= 2
    return min(fine, config.grid_step)


def _rescue_rungs(config, fine_grid, pcb_data, net_id):
    """The scoped retry ladder for one net (see module docstring)."""
    from plane_pad_tap import _clearance_ladder, fab_floor_clearance_track

    max_iters = min(config.max_iterations, defaults.RESCUE_MAX_ITERATIONS)
    rungs = []
    # Rung 0: nominal geometry at the finer grid only. Skipped when the run
    # already routes at (or below) the rescue grid - identical to what failed.
    if fine_grid < config.grid_step - 1e-9:
        rungs.append(replace(config, grid_step=fine_grid,
                             max_iterations=max_iters))

    fab_clear, fab_track = fab_floor_clearance_track(pcb_data)
    nominal_w = config.get_net_track_width(net_id, config.layers[0])
    rescue_track = min(nominal_w, fab_track)  # never widen a sub-floor choice
    power_widths = dict(config.power_net_widths)
    power_widths.pop(net_id, None)  # this net necks down; other nets are obstacles
    for clearance in _clearance_ladder(config.clearance, fab_clear,
                                       defaults.RESCUE_CLEARANCE_STEPS):
        rungs.append(replace(config, grid_step=fine_grid, clearance=clearance,
                             track_width=rescue_track, layer_widths={},
                             power_net_widths=power_widths,
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


def _attempt_edge(pcb_data, net_id, gap, config, net_clearances):
    """Try to route one gap inside a scoped window. Returns (result, cfg_used)
    or (None, None). Routes through free space only - no rip-up."""
    from obstacle_map import build_base_obstacle_map
    from plane_pad_tap import make_local_window
    from single_ended_routing import route_net_with_obstacles

    d, ax, ay, bx, by = gap
    cx, cy = (ax + bx) / 2, (ay + by) / 2
    half = max(d / 2 + defaults.RESCUE_WINDOW_MARGIN,
               defaults.RESCUE_MIN_WINDOW_HALF)
    fine_grid = _choose_grid(config, half)
    window = make_local_window(pcb_data, cx, cy, half)

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
        result = route_net_with_obstacles(window, net_id, cfg, obstacles)
        if result and not result.get('failed'):
            return result, cfg
    return None, None


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


def rescue_failed_nets(state, single_ended_nets, net_clearances=None):
    """Scoped fine-parameter rescue for every still-failed/partial net.

    Returns a summary dict ({'attempted', 'recovered', 'improved',
    'unchanged', 'pads_reconnected', 'time'}) or None when there was nothing
    to rescue (or KICAD_NET_RESCUE=0).
    """
    if os.environ.get('KICAD_NET_RESCUE', '1') == '0':
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
               'unchanged': [], 'pads_reconnected': 0, 'time': 0.0}
    pass_start = time.time()

    for net_name, net_id, kind in candidates:
        net_start = time.time()
        num0, comp_points, comp_pads = _net_component_info(pcb_data, net_id)
        if num0 <= 1:
            # Checker says connected (zone credit etc.) - nothing to rescue,
            # and accounting stays whatever the run already decided.
            continue
        summary['attempted'] += 1
        print(f"  Rescuing {net_name} ({kind}, {num0} disconnected parts)")

        edge_results = []
        failed_gaps = set()
        attempts = 0
        num = num0
        while attempts < defaults.RESCUE_MAX_EDGES_PER_NET and num > 1:
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
                    gaps.append((key, pair))
            if not gaps:
                break
            gaps.sort(key=lambda g: g[1][0])
            key, gap = gaps[0]
            if gap[0] > defaults.RESCUE_MAX_GAP_MM:
                print(f"    {YELLOW}gap {gap[0]:.1f}mm exceeds rescue limit "
                      f"{defaults.RESCUE_MAX_GAP_MM:g}mm - skipped{RESET}")
                break
            attempts += 1
            result, used_cfg = _attempt_edge(pcb_data, net_id, gap, config,
                                             net_clearances)
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
        state.results.append(merged)

        fully = num <= 1
        if kind == 'failed':
            if not fully:
                merged['failed_pads_info'] = _unconnected_pads_info(comp_pads)
            routed_results[net_id] = merged
            if net_id in state.remaining_net_ids:
                state.remaining_net_ids.remove(net_id)
            if net_id not in state.routed_net_ids:
                state.routed_net_ids.append(net_id)
        else:
            # Partial multipoint net: patch ITS result's accounting; the
            # rescue copper ships via `merged` in state.results.
            prev = routed_results[net_id]
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
        print(f"    {GREEN}{'fully reconnected' if fully else 'improved'}{RESET} "
              f"({num0} -> {num} parts, {elapsed:.1f}s)")

    summary['time'] = round(time.time() - pass_start, 2)
    if summary['attempted']:
        print(f"Rescue pass: {len(summary['recovered'])} recovered, "
              f"{len(summary['improved'])} improved, "
              f"{len(summary['unchanged'])} unchanged "
              f"({summary['time']:.1f}s)")
    return summary if summary['attempted'] else None
