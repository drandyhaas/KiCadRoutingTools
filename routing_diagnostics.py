"""
Heuristic suggestions for what to adjust when routes or via placements fail.

These are intentionally generic - they don't try to root-cause individual
failures, they just inspect the parameter values the user ran with and
flag the ones most likely to be too aggressive. The output is a list of
short, actionable bullet strings the GUI can append to its completion
dialog.
"""
from __future__ import annotations

from typing import Dict, List, Any


def _g(config: Dict[str, Any], key: str, default=None):
    """Config getter that tolerates missing keys."""
    return config.get(key, default) if config else default


def suggest_route_adjustments(failed: int, total: int,
                              config: Dict[str, Any]) -> List[str]:
    """Suggestions for the Route tab (batch_route) when routes fail.

    Args:
        failed: Number of nets that failed to route.
        total: Total number of nets attempted (successful + failed).
        config: The routing config dict that was used (same keys as
            _build_routing_config).

    Returns suggestions ordered with the most impactful first. Empty list
    if `failed` is 0.
    """
    if failed <= 0:
        return []

    suggestions: List[str] = []
    severity = failed / total if total > 0 else 1.0

    clearance = _g(config, 'clearance')
    track_width = _g(config, 'track_width')
    max_ripup = _g(config, 'max_ripup')
    max_iter = _g(config, 'max_iterations')
    heur = _g(config, 'heuristic_weight')
    via_size = _g(config, 'via_size')
    layers = _g(config, 'layers') or []

    # Rip-up is the single highest-leverage fix for "blocker" failures.
    if max_ripup is not None and max_ripup < 3:
        suggestions.append(
            f"Raise Max Rip-up (currently {int(max_ripup)}) to 3-5 - lets the "
            f"router temporarily remove blocker nets and retry."
        )

    # Track width: anything above 0.2 mm starts crowding dense boards.
    if track_width is not None and track_width > 0.2:
        suggestions.append(
            f"Reduce Track Width (currently {track_width:.2f} mm) toward "
            f"0.15-0.2 mm if the board can carry it - wider tracks have less "
            f"room to weave between pads."
        )

    # Clearance similarly squeezes the routable channel between pads.
    if clearance is not None and clearance > 0.15:
        suggestions.append(
            f"Reduce Clearance (currently {clearance:.2f} mm) toward "
            f"0.1-0.15 mm if your fab and net classes permit it - smaller "
            f"clearance opens up more channels."
        )

    # Iteration budget: hard routes need more time.
    if max_iter is not None and max_iter < 200_000:
        suggestions.append(
            f"Increase Max Iterations (currently {int(max_iter):,}) to "
            f"200,000-1,000,000 for hard-to-find paths."
        )

    # Heuristic weight: high values trade quality for speed and can wall off
    # alternative paths.
    if heur is not None and heur > 2.5:
        suggestions.append(
            f"Lower Heuristic Weight (currently {heur:.1f}) toward 1.5-1.9 - "
            f"high values find routes faster but bias the search away from "
            f"detours that may be the only path."
        )

    # Layer count: 2-layer boards run out of room quickly.
    if len(layers) <= 2 and severity > 0.2:
        suggestions.append(
            "Enable more copper layers if the stackup allows - 2-layer "
            "routing has little freedom to detour around blockers."
        )

    # Vias: 0.6 mm+ vias take a lot of space. Only mention on dense failures.
    if via_size is not None and via_size > 0.6 and severity > 0.2:
        suggestions.append(
            f"Try smaller Via Size (currently {via_size:.2f} mm) if the "
            f"stackup supports it - large vias block routing channels."
        )

    # If nothing matched the heuristics, give a generic hint.
    if not suggestions:
        suggestions.append(
            "Try enabling rip-up, lowering clearance/track width, or "
            "increasing Max Iterations. The Log tab has a per-net failure "
            "history that can point at specific blockers."
        )

    return suggestions


def suggest_plane_adjustments(failed_pads: int, total_pads: int,
                              config: Dict[str, Any]) -> List[str]:
    """Suggestions for the Planes tab when stitching vias couldn't be placed.

    Args:
        failed_pads: Number of pads that couldn't get a via to the plane.
        total_pads: Total pads that needed a via.
        config: The plane-creation config dict (shared params + tab options).
    """
    if failed_pads <= 0:
        return []

    suggestions: List[str] = []
    severity = failed_pads / total_pads if total_pads > 0 else 1.0

    clearance = _g(config, 'clearance')
    via_size = _g(config, 'via_size')
    via_drill = _g(config, 'via_drill')
    hole_to_hole = _g(config, 'hole_to_hole_clearance')
    rip_blocker = _g(config, 'rip_blocker_nets')
    max_search = _g(config, 'max_search_radius')

    if rip_blocker is False:
        suggestions.append(
            "Enable 'Rip up blocking nets' - lets plane creation remove "
            "obstructing nets and retry placing the via."
        )

    if clearance is not None and clearance > 0.15:
        suggestions.append(
            f"Reduce Clearance (currently {clearance:.2f} mm) toward "
            f"0.1-0.15 mm - tight clearance frees up positions near "
            f"existing pads/tracks."
        )

    if via_size is not None and via_size > 0.5:
        suggestions.append(
            f"Try smaller Via Size (currently {via_size:.2f} mm) - smaller "
            f"vias fit between dense pads/tracks."
        )

    if via_drill is not None and via_drill > 0.3:
        suggestions.append(
            f"Try smaller Via Drill (currently {via_drill:.2f} mm)."
        )

    if hole_to_hole is not None and hole_to_hole > 0.2:
        suggestions.append(
            f"Reduce Hole-to-Hole Clearance (currently {hole_to_hole:.2f} mm) "
            f"toward 0.2 mm to allow vias closer to existing drills."
        )

    if max_search is not None and max_search < 15.0 and severity > 0.2:
        suggestions.append(
            f"Increase Max Search Radius (currently {max_search:.1f} mm) - "
            f"lets the placer wander further from each pad to find a slot."
        )

    if not suggestions:
        suggestions.append(
            "Try enabling 'Rip up blocking nets', or reducing Clearance / "
            "Via Size. The Log tab shows which pads failed and why."
        )

    return suggestions


def suggest_diff_pair_adjustments(failed: int, total: int,
                                  config: Dict[str, Any]) -> List[str]:
    """Suggestions for the Differential tab when pairs fail to route.

    Differential pairs are tighter than single-ended nets - small changes
    to the pair width/gap and clearance often unblock them.
    """
    if failed <= 0:
        return []

    suggestions: List[str] = []
    severity = failed / total if total > 0 else 1.0

    dp_width = _g(config, 'diff_pair_width')
    dp_gap = _g(config, 'diff_pair_gap')
    clearance = _g(config, 'clearance')
    max_ripup = _g(config, 'max_ripup')
    max_iter = _g(config, 'max_iterations')
    min_turn_r = _g(config, 'diff_pair_min_turning_radius')

    if dp_width is not None and dp_width > 0.15:
        suggestions.append(
            f"Reduce Diff Pair Width (currently {dp_width:.2f} mm) toward "
            f"0.08-0.12 mm - narrower traces fit through tight channels."
        )
    if dp_gap is not None and dp_gap > 0.12:
        suggestions.append(
            f"Reduce Diff Pair Gap (currently {dp_gap:.3f} mm) toward "
            f"0.10-0.12 mm if your stackup permits - tighter coupling needs "
            f"less channel width."
        )
    if clearance is not None and clearance > 0.15:
        suggestions.append(
            f"Reduce Clearance (currently {clearance:.2f} mm) - the pair's "
            f"required channel = 2*width + gap + 2*clearance, so clearance "
            f"is often the limiting factor."
        )
    if max_ripup is not None and max_ripup < 3:
        suggestions.append(
            f"Raise Max Rip-up (currently {int(max_ripup)}) to 3-5 so the "
            f"router can temporarily remove blockers."
        )
    if max_iter is not None and max_iter < 200_000 and severity > 0.2:
        suggestions.append(
            f"Increase Max Iterations (currently {int(max_iter):,}) - diff "
            f"pair routing is heavier than single-ended; hard pairs need "
            f"more budget."
        )
    if min_turn_r is not None and min_turn_r > 0.3:
        suggestions.append(
            f"Reduce Min Turning Radius (currently {min_turn_r:.2f} mm) - "
            f"larger radii block tight detours."
        )

    if not suggestions:
        suggestions.append(
            "Try narrowing Diff Pair Width/Gap, lowering Clearance, or "
            "enabling rip-up. The Log tab shows per-pair failure details."
        )

    return suggestions


def suggest_bga_fanout_adjustments(failed: int, total: int,
                                   config: Dict[str, Any]) -> List[str]:
    """Suggestions for the Fanout tab (BGA) when some nets fail to fanout.

    The BGA grid is one of the densest regions on the board - track width,
    clearance, and via size dominate the success rate.
    """
    if failed <= 0:
        return []

    suggestions: List[str] = []

    track_width = _g(config, 'track_width')
    clearance = _g(config, 'clearance')
    via_size = _g(config, 'via_size')
    via_drill = _g(config, 'via_drill')
    exit_margin = _g(config, 'exit_margin')

    if track_width is not None and track_width > 0.15:
        suggestions.append(
            f"Reduce Track Width (currently {track_width:.2f} mm) toward "
            f"0.10-0.13 mm - BGA channels are narrow and a track that "
            f"barely fits leaves no room for clearance."
        )
    if clearance is not None and clearance > 0.12:
        suggestions.append(
            f"Reduce Clearance (currently {clearance:.2f} mm) toward "
            f"0.08-0.12 mm - clearance between traces and BGA balls is "
            f"the usual bottleneck."
        )
    if via_size is not None and via_size > 0.45:
        suggestions.append(
            f"Try smaller Via Size (currently {via_size:.2f} mm) - smaller "
            f"vias fit between BGA balls and free up routing channels."
        )
    if via_drill is not None and via_drill > 0.25:
        suggestions.append(
            f"Try smaller Via Drill (currently {via_drill:.2f} mm)."
        )
    if exit_margin is not None and exit_margin > 0.6:
        suggestions.append(
            f"Reduce Exit Margin (currently {exit_margin:.2f} mm) - shorter "
            f"exits before the first bend free up downstream channels."
        )

    if not suggestions:
        suggestions.append(
            "Try narrower Track Width/Clearance and smaller vias. "
            "Crowded BGA fanouts are mostly about channel geometry."
        )

    return suggestions


def suggest_qfn_fanout_adjustments(failed: int, total: int,
                                   config: Dict[str, Any]) -> List[str]:
    """Suggestions for the Fanout tab (QFN/QFP) when stub endpoints collide.

    QFN failure mode: stub endpoints land closer than `track_width + extension`
    to a neighbouring net's stub endpoint. Useful adjustments:
      - Increase Extension so stubs fan out further before bending.
      - Narrow Track Width.
    """
    if failed <= 0:
        return []

    suggestions: List[str] = []

    track_width = _g(config, 'track_width')
    extension = _g(config, 'extension')

    if extension is not None and extension < 0.4:
        suggestions.append(
            f"Increase Extension (currently {extension:.2f} mm) toward "
            f"0.3-0.6 mm - the 45-degree segment then has more room to "
            f"spread stub endpoints apart."
        )
    elif extension is not None:
        suggestions.append(
            f"Try a larger Extension (currently {extension:.2f} mm) - the "
            f"minimum endpoint spacing scales with this value."
        )

    if track_width is not None and track_width > 0.12:
        suggestions.append(
            f"Reduce Track Width (currently {track_width:.2f} mm) toward "
            f"0.08-0.12 mm - narrower stubs need less endpoint spacing."
        )

    if not suggestions:
        suggestions.append(
            "Try increasing Extension and/or reducing Track Width. The "
            "minimum allowed endpoint spacing is roughly "
            "(track_width + extension)."
        )

    return suggestions


def format_suggestions_for_dialog(suggestions: List[str]) -> str:
    """Turn the suggestion list into a 'Suggested adjustments:' block.

    Returns an empty string if there are no suggestions, otherwise a block
    with a header line and one bullet per suggestion.
    """
    if not suggestions:
        return ""
    lines = ["Suggested adjustments before retrying:"]
    for s in suggestions:
        lines.append(f"  - {s}")
    return "\n".join(lines)


def preexisting_blocker_hint(blocked_cells, config, pcb_data, net_id,
                              routed_net_ids=(), rip_existing_names=None,
                              return_names=False):
    """Name the PRE-EXISTING nets whose copper blocks a failed route (#301).

    Copper committed by an earlier run/step lives in the BASE obstacle map, so
    the rip-up blocker attribution (which only knows this run's routed nets)
    cannot see it and the net dies with a bare 'no rippable blockers found'
    (rp2350_dev GPIO4: boxed in by GPIO5/GPIO3 escape stubs from the fanout
    step). Geometric frontier attribution (the plane-repair machinery) names
    them, and --rip-existing-nets (#103) is the existing, connectivity-safe way
    to let this run rip + re-route them. Returns '' when nothing attributable.
    """
    def _ret(text, names):
        return (text, names) if return_names else text
    if not blocked_cells or pcb_data is None:
        return _ret("", [])
    import io
    import math
    from contextlib import redirect_stdout
    from plane_blocker_detection import find_route_blocker_from_frontier
    protected = set(routed_net_ids) | {net_id, 0}
    protected |= {z.net_id for z in (getattr(pcb_data, 'zones', None) or [])}

    # Attribute NEAR-ENDPOINT cells first: the decisive blockers of a boxed-in
    # pad are the couple of stubs at the corridor mouth, while a whole-frontier
    # cell count is dominated by whatever big power stub the search spread
    # along (rp2350_dev GPIO4: global attribution named +3.3V/+1V1, the actual
    # lockout was GPIO5/GPIO3 flanking the pad).
    near_cells = []
    try:
        from connectivity import get_net_endpoints
        srcs, tgts, _err = get_net_endpoints(pcb_data, net_id, config)
        anchors = [(s[3], s[4]) for s in (srcs or [])[:2]] + \
                  [(t[3], t[4]) for t in (tgts or [])[:2]]
        if anchors:
            r_cells = max(4, int(2.0 / config.grid_step))
            for (gx, gy, l) in blocked_cells:
                cx, cy = gx * config.grid_step, gy * config.grid_step
                if any(math.hypot(cx - ax, cy - ay) <= r_cells * config.grid_step
                       for ax, ay in anchors):
                    near_cells.append((gx, gy, l))
    except Exception:
        near_cells = []

    # Rank by blocked-cell count, but the DECISIVE blocker of a boxed pad can
    # be a small stub ranked below bigger bystanders (GPIO5's stub vs GPIO2's
    # on rp2350_dev) -- so name a generous candidate set rather than a top-3.
    names = []
    for cell_set in (near_cells, blocked_cells):
        if not cell_set:
            continue
        for _ in range(8):
            if len(names) >= 6:
                break
            with redirect_stdout(io.StringIO()):  # silence the helper's
                blocker = find_route_blocker_from_frontier(  # protected-net chatter
                    cell_set, pcb_data, config, net_id, protected)
            if blocker is None:
                break
            protected.add(blocker)
            net = pcb_data.nets.get(blocker)
            if net and net.name:
                names.append(net.name)
    # Frontier attribution can only name nets whose copper TOUCHES the
    # recorded frontier; a stub the search never reached (because a bigger
    # blocker stopped it first) is invisible there, yet ripping it may be the
    # actual unlock (rp2350_dev GPIO5). Add every net with copper within ~1mm
    # of the failing net's endpoints -- the candidates that box the pad in.
    try:
        seen = set(names)
        for ax, ay in anchors:
            for s in pcb_data.segments:
                if s.net_id in protected or s.net_id == net_id:
                    continue
                # coarse distance to segment bbox, then exact point-to-segment
                if min(s.start_x, s.end_x) - 1.0 <= ax <= max(s.start_x, s.end_x) + 1.0 and \
                   min(s.start_y, s.end_y) - 1.0 <= ay <= max(s.start_y, s.end_y) + 1.0:
                    from geometry_utils import point_to_segment_distance
                    if point_to_segment_distance(ax, ay, s.start_x, s.start_y,
                                                 s.end_x, s.end_y) <= 1.0:
                        net = pcb_data.nets.get(s.net_id)
                        if net and net.name and net.name not in seen:
                            seen.add(net.name)
                            names.append(net.name)
                            protected.add(s.net_id)
    except Exception:
        pass
    if rip_existing_names:
        names = [n for n in names if n not in rip_existing_names]
    if not names:
        return _ret("", [])
    quoted = " ".join(f"'{n}'" for n in names)
    return _ret(f"Hint: the blocking copper belongs to pre-existing net(s) {quoted} "
            f"(committed by an earlier run/step), which this run is not allowed "
            f"to rip. Retry with --rip-existing-nets {quoted} to rip and "
            f"re-route them in this run (issue #103) -- the decisive blocker "
            f"may be any of them, so start with the full set (each ripped net "
            f"is re-routed and the run reports honestly if one cannot be), "
            f"then bisect if you want a minimal rip.", names)


def static_boxin_hint(result, config, pcb_data=None) -> str:
    """One-line hint when a route died immediately with nothing rippable.

    A failure with almost no A* iterations and no rippable blockers means the
    start/target cells are boxed in by STATIC obstacles (neighboring pads plus
    clearance expansion) - typical for fine-pitch (0.4-0.65 mm) packages at
    coarse grid/clearance settings - not by other nets' routes (issue #95).
    Returns '' when the failure doesn't match that signature.
    """
    if result is None:
        return ""
    iters = result.get('iterations_forward', 0) + result.get('iterations_backward', 0)
    if iters >= 20000:
        return ""
    finer = config.grid_step / 2
    # Don't steer users into an OOM (issue #105): halving the grid step
    # quadruples cell count. Above ~4M cells per layer, suggest scoping the
    # finer grid to the failing nets instead of a board-global parameter.
    grid_advice = f"--grid-step {finer:g}"
    if pcb_data is not None and getattr(pcb_data.board_info, 'board_bounds', None):
        b = pcb_data.board_info.board_bounds
        cells = (b[2] - b[0]) * (b[3] - b[1]) / (finer * finer)
        if cells > 4e6:
            grid_advice = (f"--grid-step {finer:g} scoped to just the failing nets "
                           f"via --nets (board-global it is ~{cells / 1e6:.0f}M "
                           f"cells/layer and may exhaust memory)")
    return (f"Hint: search exhausted after only {iters} iterations with no rippable "
            f"blockers - the start/target pads are boxed in by static obstacles "
            f"(neighboring pads + clearance), not by congestion. For fine-pitch "
            f"parts try a finer grid and/or smaller clearance/track, e.g. "
            f"{grid_advice} --clearance 0.15 --track-width 0.15 "
            f"(current: grid {config.grid_step:g}, clearance {config.clearance:g}, "
            f"track {config.track_width:g} mm)")
