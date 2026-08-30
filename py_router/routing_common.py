"""
Shared utilities for route.py and route_diff.py.

This module contains common functions used by both single-ended and differential
pair routing to avoid code duplication.
"""
from __future__ import annotations

import os
import time
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import (
    PCBData, find_components_by_type, get_footprint_bounds, detect_bga_pitch,
    auto_detect_bga_exclusion_zones
)
from routing_config import GridRouteConfig
from connectivity import get_net_endpoints, _point_in_polygon
from check_drc import OFF_BOARD_TOLERANCE
from obstacle_map import build_base_obstacle_map
from obstacle_cache import (
    precompute_all_net_obstacles, build_working_obstacle_map, precompute_net_obstacles,
    add_net_obstacles_from_cache, remove_net_obstacles_from_cache
)
from memory_debug import (
    get_process_memory_mb, format_memory_stats,
    estimate_net_obstacles_cache_mb
)
from length_matching import (
    apply_length_matching_to_group, apply_time_matching_to_group,
    find_nets_matching_patterns, auto_group_ddr4_nets
)


def package_proximity_zones(pcb_data, bga_proximity_radius: float):
    """(min_x, min_y, max_x, max_y, radius_mm) proximity-field rects for
    EVERY fine-pitch package -- BGA, QFN, QFP (#585 item 4). Unlike the hard
    exclusion zones (BGA-only: a QFN's interior stays routable, just priced),
    these feed only the SOFT proximity field, with a per-package radius
    scaled by escape demand: r = bga_proximity_radius * sqrt(n_pads / 1000),
    clamped to [2.0 mm, bga_proximity_radius] -- a 1000+-ball monster earns
    the full radius, a 100-ball BGA ~2.2 mm, any QFN the 2 mm floor.
    Rects are pad-bounding-box anchored (margin 0), like the hard zones."""
    from kicad_parser import find_components_by_type, get_footprint_bounds
    zones = []
    for pkg in ('BGA', 'QFN', 'QFP'):
        for fp in find_components_by_type(pcb_data, pkg):
            n = len(fp.pads) if fp.pads else 0
            if n < 4:
                continue
            r = bga_proximity_radius * (n / 1000.0) ** 0.5
            r = min(bga_proximity_radius, max(2.0, r))
            bounds = get_footprint_bounds(fp, margin=0.0)
            zones.append((*bounds, r))
    return zones


def setup_bga_exclusion_zones(
    pcb_data: PCBData,
    disable_bga_zones: Optional[List[str]],
    existing_zones: Optional[List[Tuple[float, float, float, float]]] = None,
    selected_net_ids: Optional[List[int]] = None
) -> List[Tuple[float, float, float, float, float]]:
    """
    Handle --no-bga-zones argument and auto-detect BGA exclusion zones.

    Args:
        pcb_data: Parsed PCB data
        disable_bga_zones: List from --no-bga-zones argument (None=auto-detect,
                          []=disable all, ['U1','U3']=disable specific)
        existing_zones: Pre-configured zones (if any)

    Returns:
        List of BGA exclusion zone tuples (x1, y1, x2, y2, edge_tolerance)
    """
    if disable_bga_zones is not None:
        if len(disable_bga_zones) == 0:
            # --no-bga-zones with no args: disable all
            print("BGA exclusion zones disabled (all)")
            return []
        else:
            # --no-bga-zones U1 U3: disable only those components
            bga_components = find_components_by_type(pcb_data, 'BGA')
            bga_exclusion_zones = []
            disabled_refs = set(disable_bga_zones)
            for fp in bga_components:
                if fp.reference not in disabled_refs:
                    # Zone ends at the pad edge (get_footprint_bounds is pad-based);
                    # fanout stubs exit past it, so the keep-out doesn't bury the
                    # escape tips (issue #243). Keep in sync with
                    # auto_detect_bga_exclusion_zones (margin=0).
                    bounds = get_footprint_bounds(fp, margin=0.0)
                    pitch = detect_bga_pitch(fp)
                    edge_tolerance = pitch * 1.1
                    bga_exclusion_zones.append((*bounds, edge_tolerance))
            if bga_exclusion_zones:
                print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s) (excluding {', '.join(disable_bga_zones)}):")
                enabled_fps = [fp for fp in bga_components if fp.reference not in disabled_refs]
                for fp, zone in zip(enabled_fps, bga_exclusion_zones):
                    edge_tol = zone[4] if len(zone) > 4 else 1.6
                    print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f}), edge_tol={edge_tol:.2f}mm")
            else:
                print(f"BGA exclusion zones disabled for: {', '.join(disable_bga_zones)}")
            return bga_exclusion_zones

    elif existing_zones is None:
        # margin=0: zone ends at the BGA pad edge so fanout stubs (which exit past
        # the pads) stay in open copper for leg attachment (issue #243).
        bga_exclusion_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.0)
        if bga_exclusion_zones:
            bga_components = find_components_by_type(pcb_data, 'BGA')
            # #472 bare-ball exemption: a zone that entombs a BARE ball (no
            # attached copper) of a net THIS run must route makes that net
            # unroutable -- the deferred-fanout (direct-route) balls, and any
            # net whose fanout dropped it. Auto-disable that component's zone.
            # Engine-level (batch_route/batch_route_diff_pairs), so the GUI
            # inherits; board-state-driven, so no sidecar file is load-bearing.
            import env_knobs
            if selected_net_ids and env_knobs.BARE_BALL_ZONE_EXEMPT:
                _sel = set(selected_net_ids)
                # Only nets that can actually be ROUTED (>=2 pads): a
                # single-pad net's ball is trivially bare and disabled
                # U6/U7's zones spuriously on ottercast.
                _sel = {n for n in _sel
                        if len(pcb_data.pads_by_net.get(n, [])) >= 2}
                _pts_by_net = {}
                for _s in pcb_data.segments:
                    if _s.net_id in _sel:
                        _pts_by_net.setdefault(_s.net_id, []).append(
                            (_s.start_x, _s.start_y))
                        _pts_by_net[_s.net_id].append((_s.end_x, _s.end_y))
                for _v in pcb_data.vias:
                    if _v.net_id in _sel:
                        _pts_by_net.setdefault(_v.net_id, []).append((_v.x, _v.y))

                def _ball_bare(_p):
                    for (_x, _y) in _pts_by_net.get(_p.net_id, ()):
                        if abs(_x - _p.global_x) < 0.05 and abs(_y - _p.global_y) < 0.05:
                            return False
                    return True
                _keep, _keep_fps = [], []
                for fp, zone in zip(bga_components, bga_exclusion_zones):
                    # Only balls ENTOMBED beyond the zone's edge-tolerance
                    # band trigger: outer-ring bare balls (the #472 deferral
                    # targets) are reachable through the band + the endpoint
                    # exemption machinery (MIPI has routed bare-ball for the
                    # whole record history with zones ON).
                    _x0, _y0, _x1, _y1, _tol = zone
                    _bare = sorted({_p.net_name for _p in fp.pads
                                    if _p.net_id in _sel and not _p.drill
                                    and min(_p.global_x - _x0, _x1 - _p.global_x,
                                            _p.global_y - _y0, _y1 - _p.global_y)
                                    > _tol + 0.65
                                    and _ball_bare(_p)})
                    if _bare:
                        print(f"  {fp.reference}: zone DISABLED -- bare ball(s) of "
                              f"selected net(s) inside it ({', '.join(_bare[:6])}"
                              f"{', ...' if len(_bare) > 6 else ''}) (#472)")
                    else:
                        _keep.append(zone)
                        _keep_fps.append(fp)
                bga_exclusion_zones = _keep
                bga_components = _keep_fps
            if bga_exclusion_zones:
                print(f"Auto-detected {len(bga_exclusion_zones)} BGA exclusion zone(s):")
                for i, (fp, zone) in enumerate(zip(bga_components, bga_exclusion_zones)):
                    edge_tol = zone[4] if len(zone) > 4 else 1.6
                    print(f"  {fp.reference}: ({zone[0]:.1f}, {zone[1]:.1f}) to ({zone[2]:.1f}, {zone[3]:.1f}), edge_tol={edge_tol:.2f}mm")
        else:
            print("No BGA components detected - no exclusion zones needed")
        return bga_exclusion_zones

    return existing_zones or []


def dominant_net_widths(segments) -> Dict[int, float]:
    """Per-net DOMINANT track width, weighted by copper length (#513 item 5).

    A net's design width is the width most of its copper runs at -- max() would
    be skewed by a single wide blob, an unweighted mode by many short neck
    stubs. Used to preserve a ripped net's routed width across a same-run
    reconciliation (a rip-reconcile must not silently change a power net's
    width). Graphic and net-0 segments are ignored."""
    import math as _math
    acc: Dict[int, Dict[float, float]] = {}
    for s in segments:
        if getattr(s, 'graphic', False) or not s.net_id:
            continue
        L = _math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        if L <= 0:
            continue
        acc.setdefault(s.net_id, {})
        w = round(s.width, 4)
        acc[s.net_id][w] = acc[s.net_id].get(w, 0.0) + L
    return {nid: max(wl.items(), key=lambda kv: kv[1])[0]
            for nid, wl in acc.items() if wl}


def resolve_net_ids(pcb_data: PCBData, net_names: List[str]) -> List[Tuple[str, int]]:
    """
    Resolve net names to (name, id) tuples.

    Checks both pcb.nets and pads_by_net for net ID resolution.

    Args:
        pcb_data: Parsed PCB data
        net_names: List of net names to resolve

    Returns:
        List of (net_name, net_id) tuples for found nets
    """
    net_ids = []
    for net_name in net_names:
        net_id = None
        # First check pcb.nets
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        # If not found, check pads_by_net (for nets not in pcb.nets)
        if net_id is None:
            for nid, pads in pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.net_name == net_name:
                        net_id = nid
                        break
                if net_id is not None:
                    break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_ids.append((net_name, net_id))

    return net_ids


def _dist_point_to_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> float:
    """Minimum distance from (x, y) to the closest edge of a closed polygon."""
    best = float('inf')
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1e-12:
            d = ((x - x1) ** 2 + (y - y1) ** 2) ** 0.5
        else:
            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq))
            cx, cy = x1 + t * dx, y1 + t * dy
            d = ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5
        if d < best:
            best = d
    return best


def warn_targets_outside_board(pcb_data: PCBData,
                               net_ids: List[Tuple[str, int]],
                               edge_margin: float = 0.0,
                               label: str = "target") -> List[Tuple]:
    """Warn about pads of the nets being routed that sit at/over the board edge.

    A pad whose centre falls outside the board outline is unroutable -- the
    router blocks routing beyond the edge -- and the only symptom is a silent
    exhaustive-search failure (issue #195: a connector footprint hanging off the
    bottom of the board). A pad just inside but within the edge keep-out is
    likely unroutable too. Both are flagged here, before routing, so the cause is
    obvious instead of looking like a router failure.

    Returns the list of (net_name, where, x, y, kind) flagged pads ('outside' or
    'near-edge'); also prints a human-readable warning block when non-empty.
    """
    bounds = getattr(pcb_data.board_info, 'board_bounds', None)
    if not bounds:
        return []
    outlines = [o for o in (getattr(pcb_data.board_info, 'board_outlines', None) or [])
                if len(o) >= 3]
    if not outlines:
        outline = getattr(pcb_data.board_info, 'board_outline', None)
        outlines = [outline] if outline and len(outline) >= 3 else []
    has_poly = bool(outlines)
    min_x, min_y, max_x, max_y = bounds

    flagged = []
    seen = set()
    for net_name, net_id in net_ids:
        for pad in pcb_data.pads_by_net.get(net_id, []):
            key = (round(pad.global_x, 3), round(pad.global_y, 3), net_id)
            if key in seen:
                continue
            seen.add(key)
            x, y = pad.global_x, pad.global_y
            if has_poly:
                # multi-outline boards (#304): on-board = inside ANY outer ring
                inside = any(_point_in_polygon(x, y, o) for o in outlines)
                dist = (min(_dist_point_to_polygon(x, y, o) for o in outlines)
                        if inside else -1.0)
            else:
                dist = min(x - min_x, max_x - x, y - min_y, max_y - y)
                inside = dist >= 0
            ref = getattr(pad, 'component_ref', '') or ''
            padnum = getattr(pad, 'pad_number', '')
            where = f"{ref}.{padnum}" if ref else f"pad {padnum}"
            if not inside:
                # Distinguish pads the router will actually SKIP (clearly
                # outside, issue #291) from pads grazing the outline (kept
                # routable -- castellated / edge-connector pads). Measure to the
                # NEAREST outer ring (multi-outline boards, #304); `outline`
                # (singular) is only bound on the bbox-fallback path, so use the
                # `outlines` list that `has_poly`/`inside` were computed from --
                # referencing `outline` here was an unbound-local crash whenever
                # a pad fell outside a real polygon outline (issue #475).
                out_dist = (min(_dist_point_to_polygon(x, y, o) for o in outlines)
                            if has_poly else -dist)
                kind = 'outside' if out_dist > OFF_BOARD_TOLERANCE else 'near-edge'
                flagged.append((net_name, where, x, y, kind))
            elif edge_margin > 0 and dist < edge_margin:
                flagged.append((net_name, where, x, y, 'near-edge'))

    if flagged:
        print(f"\nWARNING: {len(flagged)} {label} pad(s) at/over the board outline:")
        for net_name, where, x, y, kind in flagged:
            if kind == 'outside':
                print(f"    {net_name} ({where}) at ({x:.2f}, {y:.2f}) is OUTSIDE the board "
                      f"outline -- unreachable, SKIPPED (no copper will be drawn toward it, #291)")
            else:
                print(f"    {net_name} ({where}) at ({x:.2f}, {y:.2f}) is at/near the board "
                      f"edge (inside the edge keep-out) -- may be unroutable")
    return flagged


def filter_already_routed(
    pcb_data: PCBData,
    net_ids: List[Tuple[str, int]],
    config: GridRouteConfig,
    assume_open: Optional[Set[str]] = None,
    fragment_gate: bool = False,
) -> Tuple[List[Tuple[str, int]], List[Tuple[str, str]]]:
    """
    Filter out nets that are already fully connected.

    Uses check_net_connectivity for robust connectivity checking that handles:
    - Track-based connectivity with T-junctions
    - Via connections across layers
    - Zone/plane connectivity
    - Through-hole pads acting as vias

    Args:
        pcb_data: Parsed PCB data
        net_ids: List of (net_name, net_id) tuples
        config: Routing configuration
        assume_open: Net names carrying an exact-fill OPEN verdict (#572):
            the plane-finalize oracle proved these disconnected with KiCad's
            own fill, so the fill-MODEL zone credit below must not skip them
            -- the model accepts pad<->zone kisses the exact fill denies
            (the ~0.10% residual FP class), which false-cleared oracle-punted
            custody nets as "Already fully connected".
        fragment_gate: #578 -- a ZONE-LESS net whose pads grade connected
            but whose copper is multiple strict fragments within one outline
            is NOT already routed: a plain --nets call must route the joins
            (run 6's VCC3V3 sat in 7 KiCad islands while this filter said
            "Already fully connected"). Zone-owning nets keep today's exact
            behavior (their truth channel is the oracle reconciliation and
            route_planes).

    Returns:
        Tuple of (nets_to_route, already_routed) where:
        - nets_to_route: List of (net_name, net_id) needing routing
        - already_routed: List of (net_name, reason) for skipped nets
    """
    from check_connected import (check_net_connectivity,
                                 net_break_within_outlines)

    # Group data by net for quick lookup
    zones_by_net: Dict[int, List] = {}
    for zone in pcb_data.zones:
        if zone.net_id not in zones_by_net:
            zones_by_net[zone.net_id] = []
        zones_by_net[zone.net_id].append(zone)

    segments_by_net: Dict[int, List] = {}
    for seg in pcb_data.segments:
        if seg.net_id not in segments_by_net:
            segments_by_net[seg.net_id] = []
        segments_by_net[seg.net_id].append(seg)

    vias_by_net: Dict[int, List] = {}
    for via in pcb_data.vias:
        if via.net_id not in vias_by_net:
            vias_by_net[via.net_id] = []
        vias_by_net[via.net_id].append(via)

    already_routed = []
    nets_to_route = []

    for net_name, net_id in net_ids:
        if assume_open and net_name in assume_open:
            print(f"  {net_name}: exact-fill verdict OPEN -- bypassing the "
                  f"model-credit already-routed skip (#572)")
            nets_to_route.append((net_name, net_id))
            continue
        net_segments = segments_by_net.get(net_id, [])
        net_vias = vias_by_net.get(net_id, [])
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        net_zones = zones_by_net.get(net_id, [])

        # Need at least 2 pads to route
        if len(net_pads) < 2:
            already_routed.append((net_name, f"Only {len(net_pads)} pad(s)"))
            continue

        # No segments and no zones means not connected
        if len(net_segments) == 0 and len(net_zones) == 0:
            nets_to_route.append((net_name, net_id))
            continue

        # Use check_net_connectivity for robust connectivity check
        # pcb_data => fill-component-aware zone credit: without it a net
        # whose only "connection" is a pinched pour island graded already-
        # routed and was SKIPPED by every later step (validator parity).
        result = check_net_connectivity(
            net_id, net_segments, net_vias, net_pads, net_zones,
            tolerance=0.02, pcb_data=pcb_data
        )

        # #479 multi-board: a net whose only missing links run BETWEEN board
        # outlines needs no routing -- no copper can ever join them (the
        # board-to-board connector does, at assembly). Grading exempts them
        # (net_break_within_outlines); skipping here keeps the router from
        # burning iterations on impossible edges and reporting phantom fails.
        _broken, _ = net_break_within_outlines(pcb_data, result)
        if not _broken and fragment_gate and not net_zones and net_segments:
            # #578 fragment gate: pads connected != copper whole.
            from check_connected import net_copper_fragments
            frag = net_copper_fragments(net_id, net_segments, net_vias,
                                        net_pads, net_zones,
                                        pcb_data=pcb_data)
            n = _max_fragments_within_one_outline(
                pcb_data, frag['fragment_anchors'])
            # ...but ROUTING only helps when there is live copper to join
            # (#659). Dead copper -- a cluster tied to no pad and no pour --
            # cannot improve connectivity no matter what the router welds to
            # it, and the late orphan sweep deletes it at the end of the run.
            # Measured on spartan6_4layer, three such nets cost 448s, 2.0M A*
            # iterations on one link and 28 neighbours pulled in by the rip
            # escalation, fixed NONE of them, and bred three more debris nets.
            #
            # `frag['padless_fragments']` is the WRONG test here and using it
            # broke this gate's own fixture: STRICT pad-less-ness only says
            # the fragment misses the pad's copper, which is exactly what a
            # phantom split looks like -- the net's real route ending a hair
            # short of the pad, the case #578 was built to route. Ask the
            # authoritative graph which copper is dead, drop it, and re-count:
            # only if the REMAINDER is whole is there nothing left to route.
            if n > 1 and os.environ.get('KICAD_659_DIVERT', '1') != '0':
                from check_connected import net_dead_copper
                _dead_s, _dead_v = net_dead_copper(
                    pcb_data, net_id, net_segments, net_vias, net_pads,
                    net_zones)
                if _dead_s or _dead_v:
                    _ds_ids = {id(s) for s in _dead_s}
                    _dv_ids = {id(v) for v in _dead_v}
                    _frag2 = net_copper_fragments(
                        net_id,
                        [s for s in net_segments if id(s) not in _ds_ids],
                        [v for v in net_vias if id(v) not in _dv_ids],
                        net_pads, net_zones, pcb_data=pcb_data)
                    _n2 = _max_fragments_within_one_outline(
                        pcb_data, _frag2['fragment_anchors'])
                    if _n2 <= 1:
                        print(f"  {net_name}: pads grade connected and every "
                              f"extra fragment is DEAD copper "
                              f"({len(_dead_s)} segment(s), {len(_dead_v)} "
                              f"via(s) tied to no pad or pour) -- not a route "
                              f"(#659); the late orphan sweep removes it")
                        n = _n2
            if n > 1:
                print(f"  {net_name}: pads grade connected, but its copper "
                      f"is {n} separate track fragment(s)"
                      + (f" ({frag['padless_fragments']} pad-less)"
                         if frag['padless_fragments'] else "")
                      + " -- routing to join them")
                nets_to_route.append((net_name, net_id))
                continue
        if not _broken:
            already_routed.append(
                (net_name, "Already fully connected" if result['connected']
                 else "Connected within each board outline"))
        else:
            nets_to_route.append((net_name, net_id))

    if already_routed:
        # Summarize by reason. This list is usually most of the board (a
        # 158-net run skipped 82) and carries only a handful of distinct
        # reasons, so the roster pushed the actual run configuration off the
        # top of the log while telling the reader nothing per line.
        by_reason: Dict[str, List[str]] = {}
        for net_name, reason in already_routed:
            by_reason.setdefault(reason, []).append(net_name)
        # Reason strings are reproduced VERBATIM (not lowercased to read as
        # prose): they are the vocabulary other readers grep these logs for.
        if len(by_reason) == 1:
            breakdown = next(iter(by_reason))
        else:
            breakdown = ", ".join(f"{len(names)} x {reason}"
                                  for reason, names in sorted(by_reason.items()))
        print(f"\nSkipping {len(already_routed)} already-routed net(s): {breakdown}")
        if getattr(config, 'verbose', False):
            for net_name, reason in already_routed:
                print(f"  {net_name}: {reason}")

    return nets_to_route, already_routed


def _max_fragments_within_one_outline(pcb_data, anchors):
    """How many strict fragments share a single board outline (#479 parity:
    fragments split only ACROSS outlines are not joinable by copper)."""
    outlines = getattr(pcb_data.board_info, 'board_outlines', None) or []
    if len(outlines) <= 1:
        return len(anchors)
    from check_connected import point_in_polygon
    counts: Dict[int, int] = {}
    for (x, y) in anchors:
        for i, poly in enumerate(outlines):
            if point_in_polygon(x, y, poly):
                counts[i] = counts.get(i, 0) + 1
                break
    return max(counts.values()) if counts else len(anchors)


def run_length_matching(
    routed_results: Dict[int, Dict],
    length_match_groups: List[List[str]],
    config: GridRouteConfig,
    pcb_data: PCBData
) -> Dict[str, Dict]:
    """
    Apply length or time matching to all configured groups.

    If config.time_matching is True, matches propagation time instead of length,
    accounting for different signal speeds on different layers.

    Args:
        routed_results: Dict of net_id -> routing result
        length_match_groups: List of pattern groups to match
        config: Routing configuration
        pcb_data: Parsed PCB data

    Returns:
        Updated net_name_to_result mapping
    """
    # Select matching function based on config
    if config.time_matching:
        matching_func = apply_time_matching_to_group
        print("\n" + "=" * 60)
        print("Time matching (propagation delay)")
        print("=" * 60)
    else:
        matching_func = apply_length_matching_to_group
        print("\n" + "=" * 60)
        print("Length matching")
        print("=" * 60)

    # Build net_name -> result mapping
    net_name_to_result = {}
    for net_id, result in routed_results.items():
        if net_id in pcb_data.nets:
            net_name = pcb_data.nets[net_id].name
            net_name_to_result[net_name] = result

    all_routed_names = list(net_name_to_result.keys())

    # Track all segments/vias from previously processed groups
    all_processed_segments = []
    all_processed_vias = []

    for group in length_match_groups:
        # Handle "auto" for DDR4 grouping
        if len(group) == 1 and group[0].lower() == 'auto':
            auto_groups = auto_group_ddr4_nets(all_routed_names)
            for auto_group in auto_groups:
                if len(auto_group) >= 2:
                    net_name_to_result = matching_func(
                        net_name_to_result, auto_group, config, pcb_data,
                        all_processed_segments, all_processed_vias
                    )
                    # Collect segments/vias from this group for subsequent groups
                    for net_name in auto_group:
                        if net_name in net_name_to_result:
                            result = net_name_to_result[net_name]
                            if result.get('new_segments'):
                                all_processed_segments.extend(result['new_segments'])
                            if result.get('new_vias'):
                                all_processed_vias.extend(result['new_vias'])
        else:
            # Find nets matching the patterns in this group
            matching_nets = find_nets_matching_patterns(all_routed_names, group)
            if len(matching_nets) >= 2:
                match_type = "Time match" if config.time_matching else "Length match"
                print(f"\n{match_type} group: {group}")
                print(f"  Matched nets: {matching_nets}")
                net_name_to_result = matching_func(
                    net_name_to_result, matching_nets, config, pcb_data,
                    all_processed_segments, all_processed_vias
                )
                # Collect segments/vias from this group for subsequent groups
                for net_name in matching_nets:
                    if net_name in net_name_to_result:
                        result = net_name_to_result[net_name]
                        if result.get('new_segments'):
                            all_processed_segments.extend(result['new_segments'])
                        if result.get('new_vias'):
                            all_processed_vias.extend(result['new_vias'])

    return net_name_to_result


def sync_pcb_data_segments(
    pcb_data: PCBData,
    routed_results: Dict[int, Dict],
    original_segment_ids: Set[int],
    state=None,
    config: GridRouteConfig = None
) -> None:
    """
    Sync routed segments back to pcb_data and update obstacle cache.

    Preserves original stubs (segments from input file) and replaces only
    routed segments.

    Args:
        pcb_data: Parsed PCB data (modified in place)
        routed_results: Dict of net_id -> routing result
        original_segment_ids: Set of id() for original segments to preserve
        state: Optional RoutingState for cache updates
        config: Optional config for cache recomputation
    """
    if not routed_results:
        return

    routed_net_ids_set = set(routed_results.keys())
    seg_count_before = len(pcb_data.segments)

    # Remove only ROUTED segments (not original stubs) for routed nets
    pcb_data.segments = [s for s in pcb_data.segments
                         if s.net_id not in routed_net_ids_set or id(s) in original_segment_ids]
    seg_count_after_remove = len(pcb_data.segments)

    # Add current (possibly meandered) segments. Never add an OBJECT that is
    # already present: if a stale/aliased id survived the removal filter (id()
    # of a garbage-collected original recycled onto a new segment), appending
    # it again would put the same object in the list twice -- duplicate graph
    # nodes then defeat the cleanup passes' connectivity gates (#195).
    present = set(id(s) for s in pcb_data.segments)
    total_added = 0
    for net_id, result in routed_results.items():
        for seg in result.get('new_segments', []):
            if id(seg) in present:
                continue
            present.add(id(seg))
            pcb_data.segments.append(seg)
            total_added += 1
    print(f"\nSync pcb_data: {seg_count_before} -> {seg_count_after_remove} (kept stubs) -> {len(pcb_data.segments)} (after adding {total_added})")

    # Sync working_obstacles with the updated pcb_data
    if state is not None and config is not None:
        if state.working_obstacles is not None and state.net_obstacles_cache is not None:
            for net_id in routed_net_ids_set:
                # Remove old cache from working
                if net_id in state.net_obstacles_cache:
                    remove_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])
                # Recompute cache from current pcb_data
                state.net_obstacles_cache[net_id] = precompute_net_obstacles(pcb_data, net_id, config)
                # Add new cache to working
                add_net_obstacles_from_cache(state.working_obstacles, state.net_obstacles_cache[net_id])


def get_common_config_kwargs(
    track_width: float,
    clearance: float,
    via_size: float,
    via_drill: float,
    grid_step: float,
    via_cost: int,
    layers: List[str],
    max_iterations: int,
    max_probe_iterations: int,
    heuristic_weight: float,
    turn_cost: int,
    direction_preference_cost: int,
    bus_enabled: bool,
    bus_detection_radius: float,
    bus_attraction_radius: float,
    bus_attraction_bonus: int,
    bus_min_nets: int,
    proximity_heuristic_factor: float,
    bga_exclusion_zones: List,
    stub_proximity_radius: float,
    stub_proximity_cost: float,
    via_proximity_cost: float,
    bga_proximity_radius: float,
    bga_proximity_cost: float,
    track_proximity_distance: float,
    track_proximity_cost: float,
    debug_lines: bool,
    verbose: bool,
    max_rip_up_count: int,
    crossing_penalty: float,
    crossing_layer_check: bool,
    routing_clearance_margin: float,
    hole_to_hole_clearance: float,
    board_edge_clearance: float,
    vertical_attraction_radius: float,
    vertical_attraction_cost: float,
    ripped_route_avoidance_radius: float,
    ripped_route_avoidance_cost: float,
    length_match_groups: Optional[List[List[str]]],
    length_match_tolerance: float,
    meander_amplitude: float,
    time_matching: bool,
    time_match_tolerance: float,
    debug_memory: bool,
    layer_costs: Optional[List[float]] = None,
    meander_spacing: float = 2.0,
    guide_corridor_enabled: bool = False,
    guide_corridor_layer: str = "User.1",
    guide_corridor_spacing: float = 0.0,
    keepout_enabled: bool = False,
    keepout_layer: str = "User.2",
    ripup_abandon_metric: str = 'stranded',
    ripup_blocker_select: str = 'count',
) -> Dict:
    """
    Build config kwargs dict from common parameters.

    Returns a dict that can be passed to GridRouteConfig constructor,
    with additional diff-pair or single-ended specific kwargs added by caller.
    """
    return dict(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        via_cost=via_cost,
        layers=layers,
        max_iterations=max_iterations,
        max_probe_iterations=max_probe_iterations,
        heuristic_weight=heuristic_weight,
        turn_cost=turn_cost,
        direction_preference_cost=direction_preference_cost,
        bus_enabled=bus_enabled,
        bus_detection_radius=bus_detection_radius,
        bus_attraction_radius=bus_attraction_radius,
        bus_attraction_bonus=bus_attraction_bonus,
        bus_min_nets=bus_min_nets,
        guide_corridor_enabled=guide_corridor_enabled,
        guide_corridor_layer=guide_corridor_layer,
        guide_corridor_spacing=guide_corridor_spacing,
        keepout_enabled=keepout_enabled,
        keepout_layer=keepout_layer,
        proximity_heuristic_factor=proximity_heuristic_factor,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius,
        stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost,
        bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost,
        track_proximity_distance=track_proximity_distance,
        track_proximity_cost=track_proximity_cost,
        debug_lines=debug_lines,
        verbose=verbose,
        max_rip_up_count=max_rip_up_count,
        ripup_blocker_select=ripup_blocker_select,
        ripup_abandon_metric=ripup_abandon_metric,
        target_swap_crossing_penalty=crossing_penalty,
        crossing_layer_check=crossing_layer_check,
        routing_clearance_margin=routing_clearance_margin,
        hole_to_hole_clearance=hole_to_hole_clearance,
        board_edge_clearance=board_edge_clearance,
        vertical_attraction_radius=vertical_attraction_radius,
        vertical_attraction_cost=vertical_attraction_cost,
        ripped_route_avoidance_radius=ripped_route_avoidance_radius,
        ripped_route_avoidance_cost=ripped_route_avoidance_cost,
        length_match_groups=length_match_groups,
        length_match_tolerance=length_match_tolerance,
        meander_amplitude=meander_amplitude,
        meander_spacing=meander_spacing,
        time_matching=time_matching,
        time_match_tolerance=time_match_tolerance,
        debug_memory=debug_memory,
        layer_costs=layer_costs,
    )
