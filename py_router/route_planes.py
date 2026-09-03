"""
Copper Plane Generator - Creates copper pour zones (the #562 "bare pour").

Creates a solid copper zone on a specified layer, plus optional via
features: thermal via arrays under exposed pads (#487), area stitching
vias (--stitch-vias), and GND return vias (--add-gnd-vias). It places NO
per-pad tap vias and draws NO traces (#562): every pad that needs a via to
reach the plane is deferred to the route step, whose pour-launch welds it
into the fill and whose in-run plane finalize taps whatever the fill
cannot reach. Run route.py over all nets (plane nets included) after this
and grade THAT board, not the bare pour.

Usage:
    python py_router/route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu
"""
from __future__ import annotations

import env_knobs
import re
import sys
import os
import argparse
from typing import List, Optional, Tuple, Dict, Set, Union
from dataclasses import dataclass

# Run startup checks first (validates numpy, scipy, shapely are installed)
from startup_checks import exit_on_error_if_main
# Stays at module scope, ABOVE the heavy imports, so a missing dep is
# reported before numpy/grid_router blow up with something cryptic. But it
# raises instead of exiting when this module is IMPORTED rather than run,
# so pytest can still collect a suite on a checkout with no built router
# (#457 item 3).
exit_on_error_if_main(__name__)

# These imports are guaranteed to work after startup_checks passes
import numpy as np

from kicad_parser import parse_kicad_pcb, PCBData, Pad, Via, Segment, KICAD_10_MIN_VERSION, pad_is_plated_through
from kicad_writer import (generate_zone_sexpr, generate_gr_line_sexpr,
                          zone_overlap_priorities)
from routing_config import GridRouteConfig, GridCoord
from routing_utils import point_in_pad_rect, pad_rect_halfspan
from route import _dump_engine_config
from connectivity import compute_mst_segments

# Import from new refactored modules
from plane_io import (
    ZoneInfo,
    extract_zones,
    check_existing_zones,
    shared_layer_zone_priority,
    resolve_net_id,
    write_plane_output
)
from plane_obstacle_builder import (
    identify_target_pads,
    build_via_obstacle_map,
    build_routing_obstacle_map,
    block_via_position,
    _add_segment_routing_obstacle,
    _add_board_edge_track_obstacles
)
from plane_zone_geometry import (
    compute_zone_boundaries,
    sample_route_for_voronoi
)
from plane_pad_tap import clamp_tap_via_to_edge
from terminal_colors import GREEN, RED, RESET

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads
from grid_router import GridObstacleMap, GridRouter

# Plane resistance calculations
from plane_resistance import (
    analyze_single_net_plane,
    analyze_multi_net_plane,
    print_single_net_resistance,
    print_multi_net_resistance,
    stackup_copper_oz,
    note_resistance_result
)
import routing_defaults as defaults


def find_via_position(
    pad: Pad,
    obstacles: GridObstacleMap,
    coord: GridCoord,
    max_search_radius: float,
    routing_obstacles: GridObstacleMap = None,
    config: GridRouteConfig = None,
    pad_layer: str = None,
    net_id: int = None,
    verbose: bool = False,
    failed_route_positions: Optional[Set[Tuple[int, int]]] = None,
    pending_pads: Optional[List[Dict]] = None,
    router: Optional[GridRouter] = None,
    position_filter=None,
    position_preference=None
) -> Optional[Tuple[float, float]]:
    """
    Find the closest valid position for a via near a pad.

    Uses spiral search outward from pad center, checking clearances.
    If routing_obstacles is provided, also verifies that A* can route from the via to the pad.

    Args:
        pad: Target pad
        obstacles: Via obstacle map
        coord: Grid coordinate converter
        max_search_radius: Maximum distance to search
        routing_obstacles: Optional routing obstacle map for routability check
        config: Optional routing config (required if routing_obstacles provided)
        pad_layer: Layer for routing (required if routing_obstacles provided)
        net_id: Net ID for routing (required if routing_obstacles provided)
        verbose: Print debug output on failure
        failed_route_positions: Optional set of (gx, gy) positions where routing previously
            failed. Positions within 2x via-size will be skipped. Failed positions from
            this call will be added to the set.
        pending_pads: Optional list of pad_info dicts for pads that still need vias.
            Via positions too close to these pads' boundaries will be skipped to ensure
            routes can still reach them.
        position_filter: Optional (x, y) -> bool predicate; positions failing it are
            skipped. Used to keep a plane-tap via INSIDE its net's zone polygon on a
            Voronoi-shared layer (issue #287) -- a via in the inter-cell gap reaches
            no fill and leaves the pad floating while reporting success.
        position_preference: Optional (x, y) -> bool SOFT ranking (never excludes):
            candidates it approves are tried first (nearest-preferred wins over
            nearest); with a routability check, the multi-source A* seeds the
            preferred candidates alone first and falls back to the full set.
            Used to prefer via sites on the PREDICTED main zone-fill component,
            so stitching vias stop landing in clearance-carved pockets that the
            plane repair step must strap later.

    Returns:
        (x, y) position for via, or None if no valid position found
    """
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # In-pad candidates that FAIL the fill preference are remembered, not
    # returned (#483 item 2): a via-in-pad on a predicted fill ISLAND taps
    # the island -- a nearby main-fill via + short trace connects for real.
    # They remain the final fallback (no trace needed) when nothing
    # preferred exists anywhere.
    _inpad_fallback = None

    def _pref_ok(_x, _y):
        return position_preference is None or position_preference(_x, _y)

    # Try pad center first - if not blocked, use it (no routing needed)
    if not obstacles.is_via_blocked(pad_gx, pad_gy):
        if position_filter is None or position_filter(pad.global_x, pad.global_y):
            if _pref_ok(pad.global_x, pad.global_y):
                return (pad.global_x, pad.global_y)
            _inpad_fallback = (pad.global_x, pad.global_y)

    # Then try other positions WITHIN the pad's own copper (still via-in-pad, no
    # trace needed): the exact centre can be blocked by nearby other-net copper
    # while another spot inside the pad is clear. Recovers boxed-in plane pads
    # (BGA balls, decoupling caps) with no room to tap externally, especially
    # with the smaller fine-pitch via (issue #99). Self-gating: when via-in-pad
    # is disabled the whole pad area is blocked in the obstacle map.
    _phw, _phh = pad_rect_halfspan(pad)  # rotated-rect bbox bound
    pad_half_w_grid = max(1, coord.to_grid_dist(_phw))
    pad_half_h_grid = max(1, coord.to_grid_dist(_phh))
    _rotated = bool(getattr(pad, 'rect_rotation', 0.0))
    for r in range(1, max(pad_half_w_grid, pad_half_h_grid) + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if abs(dx) != r and abs(dy) != r:
                    continue  # ring edge only
                if abs(dx) > pad_half_w_grid or abs(dy) > pad_half_h_grid:
                    continue  # outside pad bbox
                gx, gy = pad_gx + dx, pad_gy + dy
                bx, by = coord.to_float(gx, gy)
                if _rotated and not point_in_pad_rect(bx, by, pad):
                    continue  # outside the (rotated) pad copper
                if position_filter is not None and not position_filter(bx, by):
                    continue  # e.g. outside the net's Voronoi cell (issue #287)
                if not obstacles.is_via_blocked(gx, gy):
                    if _pref_ok(bx, by):
                        return (bx, by)
                    if _inpad_fallback is None:
                        _inpad_fallback = (bx, by)

    # Spiral search outward
    max_radius_grid = coord.to_grid_dist(max_search_radius)

    # Skip radius: 2x via-size in grid units
    skip_radius_sq = 0
    if failed_route_positions is not None and config:
        skip_radius = coord.to_grid_dist(config.via_size * 2)
        skip_radius_sq = skip_radius * skip_radius

    # Precompute pending pad exclusion zones (rectangular, in grid coordinates)
    # Each zone ensures a route can still reach the pad from any direction
    # Zone extends: pad_half_size + 1.5*via_size + clearance from pad center
    # (1.5*via_size = via_size/2 for placed via + via_size/2 for future via + via_size/2 extra margin)
    pending_pad_zones = []  # List of (min_gx, min_gy, max_gx, max_gy)
    if pending_pads and config:
        margin = 1.5 * config.via_size + config.clearance
        for pad_info in pending_pads:
            p = pad_info['pad']
            half_w, half_h = pad_rect_halfspan(p, margin)
            min_gx = coord.to_grid(p.global_x - half_w, 0)[0]
            max_gx = coord.to_grid(p.global_x + half_w, 0)[0]
            min_gy = coord.to_grid(0, p.global_y - half_h)[1]
            max_gy = coord.to_grid(0, p.global_y + half_h)[1]
            pending_pad_zones.append((min_gx, min_gy, max_gx, max_gy))

    # Collect all valid via positions, sorted by distance
    valid_positions = []

    # Open via cells within the search radius. The Rust obstacle map returns
    # every non-via-blocked cell nearest-first in one batched FFI call
    # (grid_router 0.16.0), replacing an O(radius^2) per-cell is_via_blocked()
    # spiral - the wide-radius plane-tap search was thousands of FFI calls. Fall
    # back to the spiral if the binary predates the batch query.
    if hasattr(obstacles, 'open_via_cells_within'):
        open_cells = obstacles.open_via_cells_within(pad_gx, pad_gy, max_radius_grid)
    else:
        open_cells = []
        for radius in range(1, max_radius_grid + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    gx, gy = pad_gx + dx, pad_gy + dy
                    if not obstacles.is_via_blocked(gx, gy):
                        open_cells.append((gx, gy))

    # Thin candidate via sites to ~1 per via-radius bin (#263). A wide forced-via
    # search (max_search_radius 10mm at 0.05 grid = ~125k cells) otherwise
    # enumerates and seeds EVERY open cell as an A* source, and repeats it per
    # via-size rung x per fine-clearance config -- the dominant cost on a boxed-in
    # pad amid open space (e.g. daisho U1.W18). Candidate vias closer than a via
    # radius are redundant (a via at either connects the same), so bin by
    # via-radius and keep the NEAREST open cell per bin (open_cells is nearest-
    # first). Cuts the doomed search ~an order of magnitude while preserving spatial
    # coverage across the radius. Only kicks in on large searches so small/normal
    # taps keep exact behavior. (config -- which carries via_size -- is passed by
    # both call sites; the None-guard is just null-safety for the optional param.)
    via_bin = 0
    if config is not None and len(open_cells) > 4000:
        via_bin = max(1, int(round(config.via_size / coord.grid_step / 2)))

    # Vectorized candidate filtering (#263 follow-up): the per-cell Python loop
    # over the batched open-cell query was ~12s of the scalenode route_planes
    # step (125k cells x filters, repeated per pad). The failed-position and
    # pending-pad filters are pure arithmetic -- run them as array passes, in
    # the ORIGINAL order (both run before bin consumption, so a bin is never
    # consumed by a cell one of them would have skipped). Only position_filter
    # (an arbitrary Python predicate) still needs a per-cell loop, and it now
    # runs over the pre-filtered survivors only.
    if open_cells:
        _oc = np.asarray(open_cells, dtype=np.int64).reshape(-1, 2)
        _ogx, _ogy = _oc[:, 0], _oc[:, 1]
        _keep = np.ones(len(_oc), dtype=bool)
        if failed_route_positions and skip_radius_sq > 0:
            for _fgx, _fgy in failed_route_positions:
                _keep &= ((_ogx - _fgx) ** 2 + (_ogy - _fgy) ** 2
                          > skip_radius_sq)
        for _mnx, _mny, _mxx, _mxy in pending_pad_zones:
            _keep &= ~((_ogx >= _mnx) & (_ogx <= _mxx)
                       & (_ogy >= _mny) & (_ogy <= _mxy))
        _sel = np.nonzero(_keep)[0]

        if position_filter is not None:
            # Arbitrary predicate: per-cell loop over survivors, binning after
            # the filter exactly as before.
            seen_bins = set()
            for _i in _sel.tolist():
                gx, gy = int(_ogx[_i]), int(_ogy[_i])
                fx, fy = coord.to_float(gx, gy)
                if not position_filter(fx, fy):
                    continue
                if via_bin:
                    bin_key = (gx // via_bin, gy // via_bin)
                    if bin_key in seen_bins:
                        continue
                    seen_bins.add(bin_key)
                dx, dy = gx - pad_gx, gy - pad_gy
                valid_positions.append(
                    (dx * dx + dy * dy, coord.to_float(gx, gy), gx, gy))
        else:
            if via_bin and _sel.size:
                _bins = np.stack([_ogx[_sel] // via_bin,
                                  _ogy[_sel] // via_bin], axis=1)
                # First occurrence per bin; open_cells is nearest-first, so
                # np.sort(first-index) preserves keep-the-nearest semantics.
                _, _first = np.unique(_bins, axis=0, return_index=True)
                _sel = _sel[np.sort(_first)]
            _sgx, _sgy = _ogx[_sel], _ogy[_sel]
            _dsq = (_sgx - pad_gx) ** 2 + (_sgy - pad_gy) ** 2
            _fxs = _sgx * coord.grid_step
            _fys = _sgy * coord.grid_step
            valid_positions.extend(
                (int(d), (float(x), float(y)), int(g), int(h))
                for d, x, y, g, h in zip(_dsq.tolist(), _fxs.tolist(),
                                         _fys.tolist(), _sgx.tolist(),
                                         _sgy.tolist()))

    # The Rust query is already nearest-first; sort anyway so the fallback path
    # (and any future unordered source) is correct.
    valid_positions.sort(key=lambda x: x[0])

    # Soft preference ranking: stable-partition the nearest _PREF_EVAL_CAP
    # candidates into preferred-first (bounded so an arbitrary predicate can't
    # turn a 100k-cell search into 100k Python calls; beyond the cap candidates
    # keep pure distance order). Preference never EXCLUDES -- a board whose
    # predicted fill is wrong degrades to today's behavior, not to a failure.
    n_preferred = 0
    if position_preference is not None and valid_positions:
        _PREF_EVAL_CAP = 600
        head = valid_positions[:_PREF_EVAL_CAP]
        tail = valid_positions[_PREF_EVAL_CAP:]
        pref, rest = [], []
        for vp in head:
            (pref if position_preference(vp[1][0], vp[1][1])
             else rest).append(vp)
        valid_positions = pref + rest + tail
        n_preferred = len(pref)

    # If no routing check needed, return closest valid position. A preferred
    # external candidate beats the non-preferred in-pad fallback (that is the
    # point of the preference); with no preferred candidate anywhere, the
    # in-pad fallback wins over a non-preferred external one (no trace).
    if routing_obstacles is None or config is None:
        if _inpad_fallback is not None and n_preferred == 0:
            return _inpad_fallback
        if valid_positions:
            return valid_positions[0][1]
        if _inpad_fallback is not None:
            return _inpad_fallback
        if verbose:
            print(f"\n    DEBUG: No valid via positions found (all blocked in obstacle map)")
            print(f"    DEBUG: Searched {max_radius_grid} grid steps ({max_search_radius}mm) from pad center")
        return None

    # Routability check (#259): instead of running one A* per candidate via cell
    # (K searches, K up to thousands, ~99% of them failing on dense boards), seed a
    # SINGLE multi-source A* with ALL candidate cells as sources and route to the
    # pad. The router explores from every candidate at once and returns the shortest
    # via->pad connection; the winning path's source end IS the via position. A
    # genuinely boxed-in pad is proven unreachable in one frontier exhaustion rather
    # than K separate searches. Candidate cells are via-unblocked (via keep-out
    # >= trace keep-out), so seeding them as source_target_cells is trace-safe --
    # the same multi-source pattern route_plane_connection_wide already uses.
    layer_idx = 0
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    source_cells = []
    src_set = set()
    n_pref_sources = 0
    skipped_count = 0
    for cand_i, (dist_sq, via_pos, gx, gy) in enumerate(valid_positions):
        # Skip cells near a position where routing already failed (rip-up retries)
        if failed_route_positions and skip_radius_sq > 0:
            if any((gx - fgx) ** 2 + (gy - fgy) ** 2 <= skip_radius_sq
                   for fgx, fgy in failed_route_positions):
                skipped_count += 1
                continue
        source_cells.append((gx, gy, layer_idx))
        src_set.add((gx, gy))
        if cand_i < n_preferred:
            n_pref_sources += 1

    if not source_cells:
        if verbose:
            print(f"[skipped {skipped_count}, no candidates left]", end=" ")
        return _inpad_fallback

    if router is None:
        router = GridRouter(
            via_cost=config.via_cost_units(),
            h_weight=config.heuristic_weight,
            turn_cost=config.turn_cost,
            via_proximity_cost=0,
            layer_costs=config.get_layer_costs(),
            proximity_heuristic_cost=config.get_proximity_heuristic_cost()
        )

    # No preferred candidate anywhere: the non-preferred in-pad fallback
    # beats a non-preferred external via + trace (#483 item 2).
    if _inpad_fallback is not None and n_preferred == 0:
        return _inpad_fallback

    # Two-phase when a preference partitioned the candidates: seed the
    # PREFERRED cells alone first (a winning source is then guaranteed on the
    # predicted main fill); fall back to the full set so the preference can
    # never cost a connection.
    phases = []
    if 0 < n_pref_sources < len(source_cells):
        phases.append(source_cells[:n_pref_sources])
    phases.append(source_cells)

    path = None
    iterations = 0
    for phase_cells in phases:
        for gx, gy, _ in phase_cells:
            routing_obstacles.add_source_target_cell(gx, gy, layer_idx)
        routing_obstacles.add_source_target_cell(pad_gx, pad_gy, layer_idx)
        # One search, seeded from all candidates; generous budget since it
        # replaces K per-candidate searches.
        ms_iters = max(10000, min(60000, len(phase_cells) * 4))
        path, iterations, _ = router.route_with_frontier(
            routing_obstacles, phase_cells, [(pad_gx, pad_gy, layer_idx)], ms_iters,
            env_knobs.COLLINEAR_VIAS,  # collinear_vias (#487: KICAD_COLLINEAR_VIAS=1)
            0,      # via_exclusion_radius
            None,   # start_direction
            None,   # end_direction
            0       # direction_steps
        )
        routing_obstacles.clear_source_target_cells()
        if path:
            break

    if path:
        # The via is the path endpoint that is one of our candidate sources
        # (the other endpoint is the pad target).
        e0 = (path[0][0], path[0][1])
        via_cell = e0 if e0 in src_set else (path[-1][0], path[-1][1])
        if verbose and (skipped_count or len(source_cells) > 1):
            print(f"[multi-source {len(source_cells)} cand, {iterations}it]", end=" ")
        return coord.to_float(via_cell[0], via_cell[1])

    # No candidate could reach the pad. Record them so a rip-up retry skips them.
    if failed_route_positions is not None:
        for gx, gy, _ in source_cells:
            failed_route_positions.add((gx, gy))
    if verbose:
        print(f"\n    DEBUG: {len(source_cells)} unblocked via positions, none routed to the "
              f"pad on {pad_layer} (multi-source, {iterations}it)", end=" ")
    return _inpad_fallback  # in-pad needs no route; None when absent


@dataclass
class RouteResult:
    """Result of a routing attempt."""
    segments: Optional[List[Dict]]  # Segments if successful, None if failed
    blocked_cells: List[Tuple[int, int, int]]  # Blocked cells from frontier (for blocker analysis)
    success: bool


def _audit_plane_via_map(obstacles, pcb_data, config, net_id,
                         same_net_pad_clearance, session_vias, coord,
                         hole_to_hole_clearance, via_drill, via_size, net_name):
    """KICAD_OBSTACLE_AUDIT (issue #309): whole-board integrity check of the
    per-net via-placement map after a net's pad pass.

    The map is mutated incrementally through rip-ups (ViaPlacementObstacleData
    removal, the #208 desync class) and per-placement block_via_position calls;
    it must end equal to a fresh rebuild from the current pcb_data plus the
    same session-via blocking. Wrongly-OPEN cells are under-blocking (a later
    via can land on ripped-net copper); wrongly-BLOCKED cells are a leak.
    """
    try:
        fresh = build_via_obstacle_map(pcb_data, config, net_id, verbose=False,
                                       same_net_pad_clearance=same_net_pad_clearance)
        for pv in session_vias:
            block_via_position(fresh, pv['x'], pv['y'], coord,
                               hole_to_hole_clearance, via_drill,
                               via_size, config.clearance)
        bb = pcb_data.board_info.board_bounds
        if not bb:
            return
        cgx, cgy = coord.to_grid((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2)
        r = int(max(bb[2] - bb[0], bb[3] - bb[1]) / config.grid_step / 2) + 50
        maintained = obstacles.open_via_cells_within(cgx, cgy, r)
        rebuilt = fresh.open_via_cells_within(cgx, cgy, r)
        if maintained == rebuilt:
            print(f"  [OBSTACLE AUDIT route_planes:{net_name}] via map BALANCED "
                  f"vs fresh rebuild ({len(maintained)} open cells)")
            return
        sm, sr = set(maintained), set(rebuilt)
        under = sm - sr   # open in maintained, blocked in fresh: under-blocked
        over = sr - sm    # blocked in maintained, open in fresh: leaked block
        print(f"  [OBSTACLE AUDIT route_planes:{net_name}] via map DIVERGED: "
              f"{len(under)} wrongly-open (under-blocked), "
              f"{len(over)} wrongly-blocked (leak) "
              f"(maintained {len(sm)} vs rebuilt {len(sr)} open cells)")
        for cell in sorted(under)[:3]:
            print(f"      under-blocked at grid {cell}")
        for cell in sorted(over)[:3]:
            print(f"      leaked block at grid {cell}")
    except Exception as e:
        print(f"  [OBSTACLE AUDIT route_planes] skipped ({e})")


def _path_to_segments(path, via_pos, pad, pad_layer, net_id, config, coord):
    """Convert an A* grid path (oriented via->pad) into trace segment dicts.

    Shared by route_via_to_pad (single source) and route_multi_source_to_pad
    (#259): both build the trace directly from the A* path, adding connecting
    stubs from the exact via float position to the first grid point and from the
    last grid point to the pad centre. `path` must run from the via/source end to
    the pad end.
    """
    segments = []

    # Add connecting segment from via to first path point
    if path:
        first_gx, first_gy, _ = path[0]
        first_x, first_y = coord.to_float(first_gx, first_gy)
        if abs(via_pos[0] - first_x) > 0.001 or abs(via_pos[1] - first_y) > 0.001:
            segments.append({
                'start': via_pos,
                'end': (first_x, first_y),
                'width': config.track_width,
                'layer': pad_layer,
                'net_id': net_id
            })

    # Convert path points to segments
    for i in range(len(path) - 1):
        gx1, gy1, _ = path[i]
        gx2, gy2, _ = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if (x1, y1) != (x2, y2):
            segments.append({
                'start': (x1, y1),
                'end': (x2, y2),
                'width': config.track_width,
                'layer': pad_layer,
                'net_id': net_id
            })

    # Add connecting segment from last path point to pad center
    if path:
        last_gx, last_gy, _ = path[-1]
        last_x, last_y = coord.to_float(last_gx, last_gy)
        if abs(pad.global_x - last_x) > 0.001 or abs(pad.global_y - last_y) > 0.001:
            segments.append({
                'start': (last_x, last_y),
                'end': (pad.global_x, pad.global_y),
                'width': config.track_width,
                'layer': pad_layer,
                'net_id': net_id
            })

    return segments


def route_via_to_pad(
    via_pos: Tuple[float, float],
    pad: Pad,
    pad_layer: str,
    net_id: int,
    routing_obstacles: GridObstacleMap,
    config: GridRouteConfig,
    max_iterations: int = 10000,
    verbose: bool = False,
    return_blocked_cells: bool = False,
    router: Optional[GridRouter] = None
) -> Optional[List[Dict]]:
    """
    Route from via position to pad center using A* pathfinding.

    Args:
        via_pos: (x, y) position of the via
        pad: Target pad
        pad_layer: Layer to route on
        net_id: Net ID for the segments
        routing_obstacles: Obstacle map for the route layer
        config: Routing configuration
        max_iterations: Maximum A* iterations
        verbose: Print debug info on failure
        return_blocked_cells: If True, return RouteResult instead of segments

    Returns:
        List of segment dicts, or None if routing failed
        If return_blocked_cells=True, returns RouteResult instead
    """
    coord = GridCoord(config.grid_step)

    # Check if via is at pad center (within tolerance)
    if abs(via_pos[0] - pad.global_x) < 0.001 and abs(via_pos[1] - pad.global_y) < 0.001:
        if return_blocked_cells:
            return RouteResult(segments=[], blocked_cells=[], success=True)
        return []  # Via is at pad center, no trace needed

    # Set up source (via) and target (pad) - single layer routing
    layer_idx = 0
    via_gx, via_gy = coord.to_grid(via_pos[0], via_pos[1])
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # Add source and target as source_target_cells to override blocking
    # This allows routing to/from positions that might be blocked by nearby pad clearances
    routing_obstacles.add_source_target_cell(via_gx, via_gy, layer_idx)
    routing_obstacles.add_source_target_cell(pad_gx, pad_gy, layer_idx)

    # Debug: check if source/target are blocked (should be False now after adding to source_target_cells)
    source_blocked = routing_obstacles.is_blocked(via_gx, via_gy, layer_idx)
    target_blocked = routing_obstacles.is_blocked(pad_gx, pad_gy, layer_idx)

    if verbose and (source_blocked or target_blocked):
        print(f"\n    DEBUG: source_blocked={source_blocked}, target_blocked={target_blocked}")
        print(f"    DEBUG: via=({via_pos[0]:.2f}, {via_pos[1]:.2f}) grid=({via_gx}, {via_gy})")
        print(f"    DEBUG: pad=({pad.global_x:.2f}, {pad.global_y:.2f}) grid=({pad_gx}, {pad_gy})")

    # Check if all neighbors of the target pad are blocked (target is isolated)
    if verbose:
        blocked_neighbors = 0
        unblocked_dirs = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            if routing_obstacles.is_blocked(pad_gx + dx, pad_gy + dy, layer_idx):
                blocked_neighbors += 1
            else:
                unblocked_dirs.append((dx, dy))
        if blocked_neighbors == 8:
            print(f"    DEBUG: Target pad is ISOLATED - all 8 neighbors blocked")
        elif blocked_neighbors >= 6:
            print(f"    DEBUG: Target pad nearly isolated - {blocked_neighbors}/8 neighbors blocked, open: {unblocked_dirs}")
            # Trace along open direction to find where blockage starts
            for dx, dy in unblocked_dirs[:1]:  # Just check first open direction
                blocked_at = None
                for dist in range(1, 30):  # Check up to 30 cells
                    nx, ny = pad_gx + dx * dist, pad_gy + dy * dist
                    if routing_obstacles.is_blocked(nx, ny, layer_idx):
                        blocked_at = dist
                        break
                if blocked_at:
                    print(f"    DEBUG: Open direction ({dx},{dy}) blocked after {blocked_at} cells at grid ({pad_gx + dx*blocked_at}, {pad_gy + dy*blocked_at})")

    sources = [(via_gx, via_gy, layer_idx)]
    targets = [(pad_gx, pad_gy, layer_idx)]

    # Create or reuse router
    if router is None:
        router = GridRouter(
            via_cost=config.via_cost_units(),
            h_weight=config.heuristic_weight,
            turn_cost=config.turn_cost,
            via_proximity_cost=0,
            layer_costs=config.get_layer_costs(),
            proximity_heuristic_cost=config.get_proximity_heuristic_cost()
        )

    path, iterations, blocked_cells = router.route_with_frontier(
        routing_obstacles, sources, targets, max_iterations,
        env_knobs.COLLINEAR_VIAS,  # collinear_vias (#487: KICAD_COLLINEAR_VIAS=1)
        0,      # via_exclusion_radius
        None,   # start_direction
        None,   # end_direction
        0       # direction_steps
    )

    if path is None:
        if verbose:
            print(f"    DEBUG: A* failed after {iterations} iterations")
        # Clear source_target_cells for next route
        routing_obstacles.clear_source_target_cells()
        if return_blocked_cells:
            return RouteResult(segments=None, blocked_cells=blocked_cells, success=False)
        return None  # Routing failed

    # Clear source_target_cells for next route
    routing_obstacles.clear_source_target_cells()

    # Convert path to segments
    segments = _path_to_segments(path, via_pos, pad, pad_layer, net_id, config, coord)

    if return_blocked_cells:
        return RouteResult(segments=segments, blocked_cells=[], success=True)
    return segments


def route_multi_source_to_pad(
    candidate_positions: List[Tuple[float, float]],
    pad: Pad,
    pad_layer: str,
    net_id: int,
    routing_obstacles: GridObstacleMap,
    config: GridRouteConfig,
    max_iterations: int = 10000,
    verbose: bool = False,
    return_blocked_cells: bool = False,
    router: Optional[GridRouter] = None,
):
    """Route a trace from `pad` to ANY of `candidate_positions` in ONE A* (#259).

    Replaces a per-candidate `route_via_to_pad` loop (one full A* per candidate,
    ~99% of them failing on dense boards) with a single multi-source search: all
    candidate cells are seeded as sources and the pad as the target, so the router
    explores from every candidate at once and returns the shortest routable
    candidate->pad connection. The trace is built **directly from the winning A*
    path** -- NOT by re-routing from the winner single-source (a re-route without
    the other candidates as sources could fail to reproduce a path that threaded
    another candidate's overridden cell; see issue #259).

    Candidate positions are existing same-net copper (vias / escaped pads) that may
    be blocked in the routing map; like route_via_to_pad they are added as
    source_target_cells to override blocking.

    Returns a (result, winning_position) tuple:
      - return_blocked_cells=False: (segments|None, pos|None)
      - return_blocked_cells=True:  (RouteResult, pos|None)
    `pos` is the winning candidate's original float position (for reused_via_pos).
    """
    coord = GridCoord(config.grid_step)
    layer_idx = 0
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # Map candidate grid cells -> original float position, skipping the pad centre
    # (no trace needed there) and duplicate cells.
    cell_to_pos: Dict[Tuple[int, int], Tuple[float, float]] = {}
    source_cells: List[Tuple[int, int, int]] = []
    src_set: Set[Tuple[int, int]] = set()
    for pos in candidate_positions:
        if abs(pos[0] - pad.global_x) < 0.001 and abs(pos[1] - pad.global_y) < 0.001:
            continue
        gx, gy = coord.to_grid(pos[0], pos[1])
        if (gx, gy) == (pad_gx, pad_gy) or (gx, gy) in src_set:
            continue
        src_set.add((gx, gy))
        source_cells.append((gx, gy, layer_idx))
        cell_to_pos[(gx, gy)] = pos

    if not source_cells:
        if return_blocked_cells:
            return RouteResult(segments=None, blocked_cells=[], success=False), None
        return None, None

    for gx, gy, _ in source_cells:
        routing_obstacles.add_source_target_cell(gx, gy, layer_idx)
    routing_obstacles.add_source_target_cell(pad_gx, pad_gy, layer_idx)

    if router is None:
        router = GridRouter(
            via_cost=config.via_cost_units(),
            h_weight=config.heuristic_weight,
            turn_cost=config.turn_cost,
            via_proximity_cost=0,
            layer_costs=config.get_layer_costs(),
            proximity_heuristic_cost=config.get_proximity_heuristic_cost()
        )

    # One search seeded from all candidates; generous budget since it replaces K.
    ms_iters = max(max_iterations, min(60000, len(source_cells) * 4))
    path, iterations, blocked_cells = router.route_with_frontier(
        routing_obstacles, source_cells, [(pad_gx, pad_gy, layer_idx)], ms_iters,
        env_knobs.COLLINEAR_VIAS,  # collinear_vias (#487: KICAD_COLLINEAR_VIAS=1)
        0,      # via_exclusion_radius
        None,   # start_direction
        None,   # end_direction
        0       # direction_steps
    )
    routing_obstacles.clear_source_target_cells()

    if path is None:
        if verbose:
            print(f"    DEBUG: multi-source A* ({len(source_cells)} cand) failed "
                  f"after {iterations} iterations", end=" ")
        if return_blocked_cells:
            return RouteResult(segments=None, blocked_cells=blocked_cells, success=False), None
        return None, None

    # Orient the path so path[0] is the winning source (candidate) end and path[-1]
    # is the pad; the other endpoint is the pad target.
    e0 = (path[0][0], path[0][1])
    if e0 not in src_set:
        path = list(reversed(path))
    win_cell = (path[0][0], path[0][1])
    via_pos = cell_to_pos.get(win_cell, coord.to_float(win_cell[0], win_cell[1]))
    if verbose and len(source_cells) > 1:
        print(f"[multi-source {len(source_cells)} cand, {iterations}it]", end=" ")

    segments = _path_to_segments(path, via_pos, pad, pad_layer, net_id, config, coord)
    if return_blocked_cells:
        return RouteResult(segments=segments, blocked_cells=[], success=True), via_pos
    return segments, via_pos


def _block_route_as_obstacle(obstacles: GridObstacleMap, route_path: List[Tuple[float, float]],
                              coord: 'GridCoord', layer_idx: int, expansion_grid: int):
    """Block a route path as obstacle using batched numpy operations."""
    radius_sq = expansion_grid * expansion_grid
    # Pre-compute the circle template (offsets that fall within the circle)
    circle_offsets = []
    for ex in range(-expansion_grid, expansion_grid + 1):
        for ey in range(-expansion_grid, expansion_grid + 1):
            if ex * ex + ey * ey <= radius_sq:
                circle_offsets.append((ex, ey))
    circle_offsets_arr = np.array(circle_offsets, dtype=np.int32)  # shape (K, 2)

    # Collect all center points along all segments using Bresenham
    centers = []
    for i in range(len(route_path) - 1):
        p1, p2 = route_path[i], route_path[i + 1]
        gx1, gy1 = coord.to_grid(p1[0], p1[1])
        gx2, gy2 = coord.to_grid(p2[0], p2[1])
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        gx, gy = gx1, gy1
        if dx > dy:
            err = dx / 2
            while gx != gx2:
                centers.append((gx, gy))
                err -= dy
                if err < 0:
                    gy += sy
                    err += dx
                gx += sx
        else:
            err = dy / 2
            while gy != gy2:
                centers.append((gx, gy))
                err -= dx
                if err < 0:
                    gx += sx
                    err += dy
                gy += sy
        centers.append((gx2, gy2))  # endpoint

    if not centers:
        return

    # Expand all centers by the circle template using numpy broadcasting
    centers_arr = np.array(centers, dtype=np.int32)  # shape (N, 2)
    # Broadcast: (N, 1, 2) + (1, K, 2) -> (N, K, 2)
    all_cells = centers_arr[:, np.newaxis, :] + circle_offsets_arr[np.newaxis, :, :]
    all_cells = all_cells.reshape(-1, 2)  # shape (N*K, 2)
    # Add layer column
    layer_col = np.full((all_cells.shape[0], 1), layer_idx, dtype=np.int32)
    all_cells_3 = np.hstack([all_cells, layer_col])  # shape (N*K, 3)
    obstacles.add_blocked_cells_batch(all_cells_3)


def build_plane_base_obstacles(
    plane_layer: str,
    net_id: int,
    other_nets_vias: Dict[int, List[Tuple[float, float]]],
    config: GridRouteConfig,
    pcb_data: PCBData,
    proximity_radius: float = 3.0,
    proximity_cost: float = 2.0,
    track_via_clearance: float = defaults.PLANE_TRACK_VIA_CLEARANCE,
    previous_routes: Optional[List[List[Tuple[float, float]]]] = None
) -> GridObstacleMap:
    """
    Build base obstacle map for plane routing (reusable across multiple MST edges).

    Includes: other nets' via blocking + proximity, segment blocking, previous route
    blocking, and board edge blocking. Does NOT include source/target cells.
    """
    coord = GridCoord(config.grid_step)
    layer_idx = 0
    obstacles = GridObstacleMap(1)

    # Block other nets' vias as hard obstacles using batched numpy operations.
    # Ceil the center-to-center radius (matches build_base_obstacles): flooring a
    # circular keep-out under-reserves by ~1 cell, letting tap traces graze
    # foreign vias (#155 follow-up).
    via_radius = max(1, coord.to_grid_dist_safe(track_via_clearance))
    radius_sq = via_radius * via_radius
    # Pre-compute circle template
    circle_offsets = []
    for ex in range(-via_radius, via_radius + 1):
        for ey in range(-via_radius, via_radius + 1):
            if ex * ex + ey * ey <= radius_sq:
                circle_offsets.append((ex, ey))

    all_via_centers = []
    for via_positions in other_nets_vias.values():
        for vx, vy in via_positions:
            gx, gy = coord.to_grid(vx, vy)
            all_via_centers.append((gx, gy))

    if all_via_centers and circle_offsets:
        centers_arr = np.array(all_via_centers, dtype=np.int32)
        offsets_arr = np.array(circle_offsets, dtype=np.int32)
        all_cells = centers_arr[:, np.newaxis, :] + offsets_arr[np.newaxis, :, :]
        all_cells = all_cells.reshape(-1, 2)
        layer_col = np.full((all_cells.shape[0], 1), layer_idx, dtype=np.int32)
        all_cells_3 = np.hstack([all_cells, layer_col])
        obstacles.add_blocked_cells_batch(all_cells_3)

    # Add proximity costs around other nets' vias
    proximity_radius_grid = coord.to_grid_dist(proximity_radius)
    proximity_cost_grid = config.cell_cost(proximity_cost)

    all_other_vias_grid = []
    for via_positions in other_nets_vias.values():
        for vx, vy in via_positions:
            gx, gy = coord.to_grid(vx, vy)
            all_other_vias_grid.append((gx, gy))

    if all_other_vias_grid:
        obstacles.add_stub_proximity_costs_batch(
            all_other_vias_grid,
            proximity_radius_grid,
            proximity_cost_grid
        )

    # Block existing segments on this layer from other nets
    for seg in pcb_data.segments:
        if seg.net_id == net_id:
            continue
        if seg.layer != plane_layer:
            continue
        seg_expansion_mm = config.track_width / 2 + seg.width / 2 + config.clearance
        seg_expansion_grid = max(1, coord.to_grid_dist_safe(seg_expansion_mm))
        _add_segment_routing_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_grid)

    # Block previous routes from other nets
    if previous_routes:
        route_expansion_mm = config.track_width + config.clearance
        route_expansion_grid = max(1, coord.to_grid_dist_safe(route_expansion_mm))
        for route_path in previous_routes:
            _block_route_as_obstacle(obstacles, route_path, coord, layer_idx, route_expansion_grid)

    # Block board edges
    _add_board_edge_track_obstacles(obstacles, pcb_data, config, layer_idx)

    return obstacles


def route_plane_connection(
    via_a: Tuple[float, float],
    via_b: Tuple[float, float],
    plane_layer: str,
    net_id: int,
    other_nets_vias: Dict[int, List[Tuple[float, float]]],
    config: GridRouteConfig,
    pcb_data: PCBData,
    proximity_radius: float = 3.0,
    proximity_cost: float = 2.0,
    track_via_clearance: float = defaults.PLANE_TRACK_VIA_CLEARANCE,
    max_iterations: int = 200000,
    verbose: bool = False,
    previous_routes: Optional[List[List[Tuple[float, float]]]] = None,
    base_obstacles: Optional[GridObstacleMap] = None,
    router: Optional[GridRouter] = None
) -> Optional[List[Tuple[float, float]]]:
    """
    Route a trace on the plane layer between two vias, avoiding other nets' vias.

    Args:
        via_a: (x, y) position of first via
        via_b: (x, y) position of second via
        plane_layer: Layer to route on (e.g., 'In5.Cu')
        net_id: Net ID for this connection
        other_nets_vias: Dict mapping other net_id -> list of (x, y) via positions
        config: Routing configuration
        pcb_data: PCB data for obstacle building
        proximity_radius: Radius around other vias to add proximity cost (mm)
        proximity_cost: Maximum proximity cost (mm equivalent)
        track_via_clearance: Clearance from track center to other nets' via centers (mm).
            This should be large enough to leave room for polygon fill.
        max_iterations: Maximum A* iterations
        verbose: Print debug info
        previous_routes: List of previously routed paths from other nets to avoid (each is a list of (x,y) points)
        base_obstacles: Optional pre-built obstacle map (cloned for this route).
            If None, builds from scratch (backward compatible).
        router: Optional pre-built GridRouter instance to reuse.

    Returns:
        List of (x, y) points along the route, or None if routing fails
    """
    coord = GridCoord(config.grid_step)
    layer_idx = 0

    if base_obstacles is not None:
        obstacles = base_obstacles.clone_fresh()
    else:
        # Backward-compatible: build from scratch
        obstacles = build_plane_base_obstacles(
            plane_layer, net_id, other_nets_vias, config, pcb_data,
            proximity_radius, proximity_cost, track_via_clearance, previous_routes
        )

    # Set up source and target
    via_a_gx, via_a_gy = coord.to_grid(via_a[0], via_a[1])
    via_b_gx, via_b_gy = coord.to_grid(via_b[0], via_b[1])

    # Make sure source and target are not blocked
    obstacles.add_source_target_cell(via_a_gx, via_a_gy, layer_idx)
    obstacles.add_source_target_cell(via_b_gx, via_b_gy, layer_idx)

    sources = [(via_a_gx, via_a_gy, layer_idx)]
    targets = [(via_b_gx, via_b_gy, layer_idx)]

    # Create or reuse router
    if router is None:
        router = GridRouter(
            via_cost=config.via_cost_units(),
            h_weight=config.heuristic_weight,
            turn_cost=config.turn_cost,
            via_proximity_cost=0,
            layer_costs=config.get_layer_costs(),
            proximity_heuristic_cost=config.get_proximity_heuristic_cost()
        )

    path, iterations, _ = router.route_with_frontier(
        obstacles, sources, targets, max_iterations,
        env_knobs.COLLINEAR_VIAS,  # collinear_vias (#487: KICAD_COLLINEAR_VIAS=1)
        0,      # via_exclusion_radius
        None,   # start_direction
        None,   # end_direction
        0       # direction_steps
    )

    if path is None:
        if verbose:
            print(f"    Route between regions failed after {iterations} iterations")
        return None

    if verbose:
        print(f"    Route found in {iterations} iterations, {len(path)} points")

    # Convert path to float coordinates
    route_points = []
    for gx, gy, _ in path:
        x, y = coord.to_float(gx, gy)
        route_points.append((x, y))

    return route_points



def _grammar_cluster(points, link=5.0):
    """Single-linkage clusters of 2D points (union-find, O(N^2))."""
    n = len(points)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    l2 = link * link
    for i in range(n):
        xi, yi = points[i]
        for j in range(i + 1, n):
            dx = points[j][0] - xi
            dy = points[j][1] - yi
            if dx * dx + dy * dy <= l2:
                ra, rb = find(i), find(j)
                if ra != rb:
                    parent[ra] = rb
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(points[i])
    return list(groups.values())


def _grammar_hull(points):
    """Convex hull (monotone chain); returns CCW polygon or None."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return None
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for pt in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], pt) <= 0:
            lower.pop()
        lower.append(pt)
    upper = []
    for pt in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], pt) <= 0:
            upper.pop()
        upper.append(pt)
    hull = lower[:-1] + upper[:-1]
    return hull if len(hull) >= 3 else None


def _grammar_inflate(hull, r):
    """Offset a CCW convex polygon outward by r (edge-normal offset +
    consecutive-line intersection)."""
    import math as _m
    n = len(hull)
    lines = []
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        L = _m.hypot(ex, ey) or 1e-9
        # CCW polygon: outward normal is (ey, -ex)/L
        nx, ny = ey / L, -ex / L
        lines.append((x1 + nx * r, y1 + ny * r, x2 + nx * r, y2 + ny * r))
    out = []
    for i in range(n):
        x1, y1, x2, y2 = lines[i - 1]
        x3, y3, x4, y4 = lines[i]
        d = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3)
        if abs(d) < 1e-12:
            out.append((x3, y3))
            continue
        t = ((x3 - x1) * (y4 - y3) - (y3 - y1) * (x4 - x3)) / d
        out.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    return out


def _grammar_point_in_poly(x, y, poly):
    """Even-odd ray cast; robust for the slightly non-convex polygons that
    per-vertex board-bounds clamping can produce."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and \
                x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _grammar_seg_dist2(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 <= 0:
        ex, ey = px - x1, py - y1
        return ex * ex + ey * ey
    t = ((px - x1) * dx + (py - y1) * dy) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    ex, ey = px - (x1 + t * dx), py - (y1 + t * dy)
    return ex * ex + ey * ey


def _grammar_sheet_raster(sheet_polygon):
    """Coarse raster of the background sheet for the #662 invariant-3b
    check. Returns (x0, y0, sx, sy, nx, ny, free) with free a bytearray
    (1 = sheet cell)."""
    import math as _m
    xs = [p[0] for p in sheet_polygon]
    ys = [p[1] for p in sheet_polygon]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    diag = _m.hypot(x1 - x0, y1 - y0) or 1.0
    step = max(0.6, min(3.0, diag / 100.0))
    nx = max(2, int(round((x1 - x0) / step)))
    ny = max(2, int(round((y1 - y0) / step)))
    sx = (x1 - x0) / nx
    sy = (y1 - y0) / ny
    free = bytearray(nx * ny)
    for i in range(nx):
        cx = x0 + (i + 0.5) * sx
        for j in range(ny):
            if _grammar_point_in_poly(cx, y0 + (j + 0.5) * sy, sheet_polygon):
                free[i * ny + j] = 1
    return (x0, y0, sx, sy, nx, ny, free)


def _grammar_sheet_ok(raster, islands, carve_mm, dom_pts):
    """#662 invariant 3b: True when the background sheet minus the islands
    (each widened by carve_mm, the fill's clearance band) is still ONE
    usable connected region. A detached piece counts as a severing only
    when it holds a dominant-net pad/seed (its copper would ship as an
    open or a weld obligation) or >=25% of the sheet; a source-less sliver
    is culled by fill island removal and is not a severing."""
    from collections import deque
    x0, y0, sx, sy, nx, ny, base = raster
    free = bytearray(base)
    # Half a raster cell rides on the carve: a neck narrower than a cell is
    # sampled unreliably, and a sub-millimetre neck is the confetti
    # anti-pattern this invariant exists to reject, not connectivity.
    carve_mm = carve_mm + 0.5 * max(sx, sy)
    c2 = carve_mm * carve_mm
    for poly in islands:
        pxs = [p[0] for p in poly]
        pys = [p[1] for p in poly]
        i0 = max(0, int((min(pxs) - carve_mm - x0) / sx) - 1)
        i1 = min(nx - 1, int((max(pxs) + carve_mm - x0) / sx) + 1)
        j0 = max(0, int((min(pys) - carve_mm - y0) / sy) - 1)
        j1 = min(ny - 1, int((max(pys) + carve_mm - y0) / sy) + 1)
        n = len(poly)
        for i in range(i0, i1 + 1):
            cx = x0 + (i + 0.5) * sx
            for j in range(j0, j1 + 1):
                k = i * ny + j
                if not free[k]:
                    continue
                cy = y0 + (j + 0.5) * sy
                if _grammar_point_in_poly(cx, cy, poly):
                    free[k] = 0
                    continue
                for e in range(n):
                    ax, ay = poly[e]
                    bx, by = poly[(e + 1) % n]
                    if _grammar_seg_dist2(cx, cy, ax, ay, bx, by) <= c2:
                        free[k] = 0
                        break
    total_free = sum(free)
    if total_free == 0:
        return False
    # label connected components (4-neighbour flood fill)
    comp = [0] * (nx * ny)
    sizes = {}
    ncomp = 0
    for start in range(nx * ny):
        if not free[start] or comp[start]:
            continue
        ncomp += 1
        comp[start] = ncomp
        size = 1
        q = deque([start])
        while q:
            k = q.popleft()
            i, j = divmod(k, ny)
            for kk in ((k - ny) if i > 0 else -1,
                       (k + ny) if i < nx - 1 else -1,
                       (k - 1) if j > 0 else -1,
                       (k + 1) if j < ny - 1 else -1):
                if kk >= 0 and free[kk] and not comp[kk]:
                    comp[kk] = ncomp
                    size += 1
                    q.append(kk)
        sizes[ncomp] = size
    if ncomp <= 1:
        return True
    meaningful = set()
    for c, size in sizes.items():
        if size >= 0.25 * total_free:
            meaningful.add(c)
    for px, py in dom_pts:
        i = int((px - x0) / sx)
        j = int((py - y0) / sy)
        if 0 <= i < nx and 0 <= j < ny and comp[i * ny + j]:
            meaningful.add(comp[i * ny + j])
    return len(meaningful) <= 1


def _grammar_zone_polygons(seeds_by_net, zone_polygon, board_bounds,
                           name_of, verbose=False,
                           inflate_mm=2.0, link_mm=5.0, pads_by_net=None,
                           carve_mm=0.8):
    """#662: build {net_id: [polygons]} as background sheet + hull islands.
    Returns None when degenerate (a single seeded net, or the dominant net
    cannot be identified) so the caller falls back to Voronoi."""
    import math as _m
    seeded = {nid: pts for nid, pts in seeds_by_net.items() if pts}
    if len(seeded) < 2:
        return None
    bx0, by0, bx1, by1 = board_bounds
    bdiag = _m.hypot(bx1 - bx0, by1 - by0) or 1.0
    # dominant = largest spread (board-wide rail), tiebreak seed count
    def spread(pts):
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        return _m.hypot(max(xs) - min(xs), max(ys) - min(ys))
    # dominance = board-wide reach x consumer count (the human gives the
    # layer to the rail with the most board-wide service to deliver, not
    # merely the widest bbox -- spread-only picked P1.35V over the 39-pad
    # P3.3V on orangecrab).
    # Score on PADS (consumers), not augmented seeds -- seed lists carry
    # MST route samples that scale with route length, not importance.
    def score_pts(nid):
        pts = (pads_by_net or {}).get(nid) or seeded[nid]
        return spread(pts) / bdiag * len(pts), len(pts)
    scored = sorted(((score_pts(nid)[0], score_pts(nid)[1], nid)
                     for nid in seeded), reverse=True)
    dom = scored[0][2]
    out = {dom: [list(zone_polygon)]}
    # Invariant 3b: the background sheet must remain ONE connected region
    # after every carve. Checked on a coarse raster against the islands
    # committed so far; a severing island first shrinks its inflation,
    # then demotes to tracks (the route step carries every plane net in
    # its --nets, #562, so a demoted cluster is served by copper, just
    # not by a zone).
    raster = _grammar_sheet_raster(zone_polygon)
    dom_pts = list((pads_by_net or {}).get(dom) or []) + list(seeded[dom])
    committed = []
    n_islands = 0
    n_shrunk = 0
    n_demoted = 0
    for nid, pts in seeded.items():
        if nid == dom:
            continue
        polys = []
        for cl in _grammar_cluster(pts, link=link_mm):
            placed = None
            for r in (inflate_mm, inflate_mm / 2.0, 0.0):
                hull = _grammar_hull(cl)
                if hull is None:
                    # 1-2 points (or collinear): a small box around them
                    xs = [p[0] for p in cl]; ys = [p[1] for p in cl]
                    m = max(r, 0.5)
                    hull = [(min(xs) - m, min(ys) - m), (max(xs) + m, min(ys) - m),
                            (max(xs) + m, max(ys) + m), (min(xs) - m, max(ys) + m)]
                elif r > 0:
                    hull = _grammar_inflate(hull, r)
                # clamp to board bounds (also keeps hulls off the edge band)
                hull = [(min(max(x, bx0 + 0.3), bx1 - 0.3),
                         min(max(y, by0 + 0.3), by1 - 0.3)) for x, y in hull]
                if _grammar_sheet_ok(raster, committed + [hull], carve_mm,
                                     dom_pts):
                    placed = hull
                    if r != inflate_mm:
                        n_shrunk += 1
                        print(f"  Grammar pour: '{name_of.get(nid, nid)}' "
                              f"island shrunk (inflate {inflate_mm} -> {r}) "
                              f"to keep the background sheet connected")
                    break
            if placed is None:
                n_demoted += 1
                print(f"  Grammar pour: '{name_of.get(nid, nid)}' cluster of "
                      f"{len(cl)} point(s) demoted to tracks -- its island "
                      f"severs the background sheet (#662 invariant 3b)")
                continue
            committed.append(placed)
            polys.append(placed)
        if polys:  # a fully-demoted net gets NO zone entry (tracks serve it)
            out[nid] = polys
        n_islands += len(polys)
    print(f"  Grammar pour (#662): '{name_of.get(dom, dom)}' = background "
          f"sheet; {n_islands} hull island(s) across "
          f"{len(seeded) - 1} net(s)"
          + (f"; {n_shrunk} shrunk" if n_shrunk else "")
          + (f"; {n_demoted} demoted to tracks" if n_demoted else "")
          + " (KICAD_GRAMMAR_POUR=0 reverts to Voronoi)")
    return out


def _generate_multinet_layer_zones(
    layer: str,
    nets_on_layer: List[str],
    pcb_data: PCBData,
    all_new_vias: List[Dict],
    zone_polygon: List[Tuple[float, float]],
    board_bounds: Tuple[float, float, float, float],
    config: GridRouteConfig,
    zone_clearance: float,
    min_thickness: float,
    plane_proximity_radius: float,
    plane_proximity_cost: float,
    plane_track_via_clearance: float,
    plane_max_iterations: int,
    voronoi_seed_interval: float,
    board_edge_clearance: float,
    debug_lines: bool,
    verbose: bool,
    thermal_relief: bool = False,
    priority_offset: int = 0
) -> Tuple[List[str], List[str], List[Dict]]:
    """
    Generate Voronoi-based zone boundaries for a multi-net layer.

    Args:
        layer: Layer name (e.g., 'In1.Cu')
        nets_on_layer: List of net names sharing this layer
        pcb_data: PCB data
        all_new_vias: List of newly placed vias
        zone_polygon: Default zone polygon (full board)
        board_bounds: (min_x, min_y, max_x, max_y)
        config: Routing configuration
        zone_clearance: Zone clearance
        min_thickness: Minimum zone thickness
        plane_proximity_radius: Proximity radius for routing
        plane_proximity_cost: Proximity cost for routing
        plane_track_via_clearance: Track-to-via clearance
        plane_max_iterations: Max A* iterations
        voronoi_seed_interval: Sample interval for Voronoi seeds
        board_edge_clearance: Edge clearance for zones
        debug_lines: Whether to generate debug lines
        verbose: Verbose output

    Returns:
        Tuple of (zone_sexprs, debug_line_sexprs, zone_data_list)
    """
    zone_sexprs = []
    debug_line_sexprs = []
    zone_data_list = []

    # Build vias_by_net for this layer
    vias_by_net: Dict[int, List[Tuple[float, float]]] = {}
    vias_by_net_set: Dict[int, Set[Tuple[float, float]]] = {}  # For O(1) dedup
    net_name_to_id = {}
    for net_name in nets_on_layer:
        net_id = next((nid for nid, n in pcb_data.nets.items() if n.name == net_name), None)
        if net_id is not None:
            net_name_to_id[net_name] = net_id
            vias_by_net[net_id] = []
            vias_by_net_set[net_id] = set()

    # Collect via positions for nets on this layer
    for via in all_new_vias:
        nid = via['net_id']
        if nid in vias_by_net:
            pos = (via['x'], via['y'])
            vias_by_net[nid].append(pos)
            vias_by_net_set[nid].add(pos)

    # Also include existing vias from the PCB (dedup with O(1) set lookup)
    for via in pcb_data.vias:
        if via.net_id in vias_by_net:
            via_pos = (via.x, via.y)
            if via_pos not in vias_by_net_set[via.net_id]:
                vias_by_net[via.net_id].append(via_pos)
                vias_by_net_set[via.net_id].add(via_pos)

    # Connection points from pads that physically tie into the plane on this
    # layer: through-hole pads (every copper layer) and SMD pads whose layer is
    # this one. A net can be fully connected to the plane through these alone,
    # with zero stitching vias -- the zone must STILL be poured in that case
    # (issue #114: an all-through-hole ground net got 0 seeds and its zone was
    # silently skipped, leaving the net functionally unrouted).
    pads_on_layer_by_net: Dict[int, List[Tuple[float, float]]] = {}
    for net_name in nets_on_layer:
        net_id = net_name_to_id.get(net_name)
        if net_id is None:
            continue
        pts = []
        for pad in pcb_data.pads_by_net.get(net_id, []):
            on_layer = pad_is_plated_through(pad) or (layer in pad.layers)  # NPTH has no barrel (#328)
            if on_layer:
                pts.append((pad.global_x, pad.global_y))
        pads_on_layer_by_net[net_id] = pts

    # A pad outside the Edge.Cuts outline can never reach the fill (it is
    # clipped to the outline) -- point-seeding it plants a Voronoi cell in dead
    # space (#291 class). Used by the projected-pad seeding just below and by
    # the #107 corridor pass further down.
    from check_drc import make_off_board_test
    _off_board_test = make_off_board_test(pcb_data.board_info)

    # #598: pours run BEFORE any routing (#562), so a virgin INNER layer holds
    # no copper at all -- no vias, and no pads either when every pad is SMD on
    # an outer layer. Both nets of a requested inner split plane then had zero
    # seeds and BOTH zones were silently skipped, shipping a board with no
    # plane where one was explicitly asked for. Fall back to that net's pads
    # PROJECTED onto this layer (their x/y, whatever layer the copper sits on):
    # those positions exist regardless of routing state, they are where the
    # route step's pour-launch vias will come down, and they partition the
    # layer between the nets the way the components themselves are spread over
    # the board. Only nets with NO seeds of their own get this, so a board that
    # already poured correctly is untouched.
    projected_pads_by_net: Dict[int, List[Tuple[float, float]]] = {}
    for net_name in nets_on_layer:
        net_id = net_name_to_id.get(net_name)
        if net_id is None or vias_by_net.get(net_id) or pads_on_layer_by_net.get(net_id):
            continue
        pts = []
        for pad in pcb_data.pads_by_net.get(net_id, []):
            if getattr(pad, 'pad_type', '') == 'np_thru_hole':
                continue  # NPTH has no copper to tie into the plane (#328)
            px, py = pad.global_x, pad.global_y
            if _off_board_test is not None and _off_board_test(px, py):
                continue
            pts.append((px, py))
        projected_pads_by_net[net_id] = pts

    # A net earns a zone if it has ANY connection point on this layer -- a via
    # or a pad -- or, with the layer still bare, its projected pads. nets_with_vias
    # still drives the via-to-via MST routing below; nets_with_seeds (vias, pads,
    # or projected pads) drives whether a zone is created at all.
    nets_with_vias = []
    nets_with_seeds = []
    for net_name in nets_on_layer:
        net_id = net_name_to_id.get(net_name)
        if not net_id:
            continue
        via_count = len(vias_by_net.get(net_id, []))
        pad_count = len(pads_on_layer_by_net.get(net_id, []))
        proj_count = len(projected_pads_by_net.get(net_id, []))
        if via_count == 0 and pad_count == 0 and proj_count == 0:
            print(f"  Warning: Net '{net_name}' has no vias or pads on layer {layer}, "
                  f"and no on-board pads anywhere to project, skipping zone")
            continue
        nets_with_seeds.append(net_name)
        if via_count > 0:
            nets_with_vias.append(net_name)
            print(f"  Net '{net_name}': {via_count} vias")
        elif pad_count > 0:
            print(f"  Net '{net_name}': no vias, {pad_count} pad(s) on {layer} (pad-seeded zone)")
        else:
            print(f"  Net '{net_name}': no copper on {layer} yet, partition seeded "
                  f"from {proj_count} projected pad(s) (#598)")

    if len(nets_with_seeds) < 2:
        # Only one net has connections on this layer: use full board rectangle.
        if nets_with_seeds:
            net_name = nets_with_seeds[0]
            net_id = net_name_to_id[net_name]
            print(f"  Only '{net_name}' has connections, using full board rectangle")
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=layer,
                polygon_points=zone_polygon,
                clearance=zone_clearance,
                min_thickness=min_thickness,
                direct_connect=not thermal_relief,
                use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION
            )
            zone_sexprs.append(zone_sexpr)
            zone_data_list.append({
                'thermal_relief': thermal_relief,
                'net_id': net_id,
                'net_name': net_name,
                'layer': layer,
                'polygon_points': zone_polygon,
                'clearance': zone_clearance,
                'min_thickness': min_thickness,
            })
        return zone_sexprs, debug_line_sexprs, zone_data_list

    # Compute MST edges for each net
    net_mst_edges: Dict[int, List[Tuple[Tuple[float, float], Tuple[float, float]]]] = {}
    net_debug_layers: Dict[int, str] = {}
    for net_idx, net_name in enumerate(nets_with_vias):
        net_id = net_name_to_id[net_name]
        net_vias = vias_by_net.get(net_id, [])
        if len(net_vias) >= 2:
            net_mst_edges[net_id] = compute_mst_segments(net_vias)
            net_debug_layers[net_id] = f"User.{net_idx + 1}"
            print(f"  Net '{net_name}': MST with {len(net_mst_edges[net_id])} edges between {len(net_vias)} vias")
        else:
            print(f"  Net '{net_name}': only {len(net_vias)} via(s), no MST needed")

    # Iteratively route all nets, reordering to put failed nets first
    max_mst_iterations = 5
    net_order = list(net_mst_edges.keys())
    failed_nets: Set[int] = set()
    best_result = None

    for mst_iteration in range(max_mst_iterations):
        if mst_iteration > 0:
            net_order = sorted(net_order, key=lambda nid: (0 if nid in failed_nets else 1))
            failed_net_names = [pcb_data.nets[nid].name for nid in failed_nets if nid in pcb_data.nets]
            print(f"  Retry {mst_iteration + 1}: reordering with failed nets first: {', '.join(failed_net_names)}")

        connection_routes = []
        routed_paths_by_edge: Dict[int, Dict[Tuple[Tuple[float, float], Tuple[float, float]], List[Tuple[float, float]]]] = {
            net_id: {} for net_id in net_mst_edges.keys()
        }
        augmented_vias_by_net = {net_id: list(vias) for net_id, vias in vias_by_net.items()}
        debug_lines_for_layer = []
        failed_nets = set()
        total_failed_edges = 0

        # Create router once per MST iteration (reused across all nets/edges)
        plane_router = GridRouter(
            via_cost=config.via_cost_units(),
            h_weight=config.heuristic_weight,
            turn_cost=config.turn_cost,
            via_proximity_cost=0,
            layer_costs=config.get_layer_costs(),
            proximity_heuristic_cost=config.get_proximity_heuristic_cost()
        )

        for net_id in net_order:
            net = pcb_data.nets.get(net_id)
            net_name = net.name if net else f"net_{net_id}"
            mst_edges = net_mst_edges[net_id]
            debug_layer = net_debug_layers[net_id]

            other_nets_vias: Dict[int, List[Tuple[float, float]]] = {}
            for other_net_id, other_vias in augmented_vias_by_net.items():
                if other_net_id != net_id:
                    other_nets_vias[other_net_id] = other_vias

            # Compute other_nets_routes once per net (only contains routes from OTHER nets,
            # so it doesn't change within this net's MST edge loop)
            other_nets_routes = [
                route for route_net_id, _, route in connection_routes
                if route_net_id != net_id
            ]

            # Build base obstacle map once per net (includes via blocking, segment blocking,
            # other nets' route blocking, and board edges - everything except source/target)
            base_obstacles = build_plane_base_obstacles(
                plane_layer=layer,
                net_id=net_id,
                other_nets_vias=other_nets_vias,
                config=config,
                pcb_data=pcb_data,
                proximity_radius=plane_proximity_radius,
                proximity_cost=plane_proximity_cost,
                track_via_clearance=plane_track_via_clearance,
                previous_routes=other_nets_routes
            )

            routed_count = 0
            failed_count = 0

            for via_a, via_b in mst_edges:
                route_path = route_plane_connection(
                    via_a=via_a,
                    via_b=via_b,
                    plane_layer=layer,
                    net_id=net_id,
                    other_nets_vias=other_nets_vias,
                    config=config,
                    pcb_data=pcb_data,
                    proximity_radius=plane_proximity_radius,
                    proximity_cost=plane_proximity_cost,
                    track_via_clearance=plane_track_via_clearance,
                    max_iterations=plane_max_iterations,
                    verbose=verbose,
                    previous_routes=other_nets_routes,
                    base_obstacles=base_obstacles,
                    router=plane_router
                )

                if route_path:
                    routed_count += 1
                    connection_routes.append((net_id, layer, route_path))
                    routed_paths_by_edge[net_id][(via_a, via_b)] = route_path

                    if debug_lines and len(route_path) >= 2:
                        for i in range(len(route_path) - 1):
                            debug_lines_for_layer.append(generate_gr_line_sexpr(
                                route_path[i], route_path[i + 1],
                                width=0.1, layer=debug_layer
                            ))

                    samples = sample_route_for_voronoi(route_path, sample_interval=voronoi_seed_interval)
                    if samples:
                        augmented_vias_by_net[net_id].extend(samples)
                else:
                    failed_count += 1
                    if verbose:
                        print(f"    {net_name}: ({via_a[0]:.2f},{via_a[1]:.2f}) -> ({via_b[0]:.2f},{via_b[1]:.2f}) FAILED")

            if failed_count > 0:
                failed_nets.add(net_id)
                total_failed_edges += failed_count
                print(f"    {net_name}: {routed_count}/{len(mst_edges)} MST edges ({failed_count} failed)")
            else:
                print(f"    {net_name}: all {routed_count} MST edges routed")

        if best_result is None or total_failed_edges < best_result[0]:
            best_result = (total_failed_edges, connection_routes, augmented_vias_by_net, debug_lines_for_layer, routed_paths_by_edge)

        if total_failed_edges == 0:
            break

    # Use best result
    if best_result:
        _, connection_routes, augmented_vias_by_net, debug_lines_for_layer, routed_paths_by_edge = best_result
        debug_line_sexprs.extend(debug_lines_for_layer)
        if best_result[0] > 0:
            print(f"  Best result: {best_result[0]} failed edge(s)")

    # #107: connect each net's through-hole pads to its plane region. A TH pad ties
    # into the plane on every layer, but the Voronoi seeds are only vias + routed-MST
    # samples, and a Voronoi cell is owned by whichever net has the nearest seed. A TH
    # pad with no nearby same-net seed therefore lands inside ANOTHER net's cell: the
    # local plane copper is the wrong net and the pad is silently disconnected as an
    # island. For each such ORPHANED pad we route a corridor on the plane layer to the
    # nearest same-net seed (avoiding other nets, exactly like the MST routing) and add
    # the corridor's samples as seeds, so the pad's region becomes contiguous with the
    # net's main region. (We deliberately do NOT add TH pads to the via MST: that
    # reorganizes the via-to-via topology and badly regresses large nets like GND.)
    if augmented_vias_by_net is not None:
        # Snapshot all seeds for orphan detection (Voronoi owner == nearest seed). Also
        # snapshot each net's own anchor points (its main region) as corridor targets.
        all_seed_pts = []  # (x, y, net_id)
        net_anchor_pts: Dict[int, List[Tuple[float, float]]] = {}
        for snid, slist in augmented_vias_by_net.items():
            net_anchor_pts[snid] = list(slist)
            for sx, sy in slist:
                all_seed_pts.append((sx, sy, snid))

        def _nearest_seed_net(px: float, py: float) -> Optional[int]:
            best_net, best_d = None, None
            for sx, sy, snid in all_seed_pts:
                d = (sx - px) ** 2 + (sy - py) ** 2
                if best_d is None or d < best_d:
                    best_d, best_net = d, snid
            return best_net

        corridor_router = GridRouter(
            via_cost=config.via_cost_units(),
            h_weight=config.heuristic_weight,
            turn_cost=config.turn_cost,
            via_proximity_cost=0,
            layer_costs=config.get_layer_costs(),
            proximity_heuristic_cost=config.get_proximity_heuristic_cost()
        )

        th_connected = 0
        th_fallback = 0
        th_off_board = 0
        fallback_pad_refs: List[str] = []
        # _off_board_test (built above) skips pads outside the outline: the fill
        # is clipped there, so corridor-routing toward such a pad is wasted and
        # point-seeding it plants a Voronoi cell in dead space (#291 class).
        for nm_on_layer in nets_on_layer:
            nid = net_name_to_id.get(nm_on_layer)
            if nid is None:
                continue
            seeds = augmented_vias_by_net.setdefault(nid, [])
            seen = {(round(x, 3), round(y, 3)) for x, y in seeds}
            anchors = net_anchor_pts.get(nid, [])
            for pad in pcb_data.pads_by_net.get(nid, []):
                if not pad_is_plated_through(pad):
                    continue
                px, py = pad.global_x, pad.global_y
                if _off_board_test is not None and _off_board_test(px, py):
                    th_off_board += 1
                    continue
                # Already inside its own net's cell? Then the plane already covers it.
                if _nearest_seed_net(px, py) == nid:
                    continue
                # Orphaned: route a corridor to the nearest same-net anchor (main region).
                route_path = None
                if anchors:
                    tx, ty = min(anchors, key=lambda t: (t[0] - px) ** 2 + (t[1] - py) ** 2)
                    other_nets_vias = {
                        onid: ov for onid, ov in augmented_vias_by_net.items() if onid != nid
                    }
                    route_path = route_plane_connection(
                        via_a=(px, py), via_b=(tx, ty), plane_layer=layer, net_id=nid,
                        other_nets_vias=other_nets_vias, config=config, pcb_data=pcb_data,
                        proximity_radius=plane_proximity_radius, proximity_cost=plane_proximity_cost,
                        track_via_clearance=plane_track_via_clearance,
                        max_iterations=plane_max_iterations, verbose=verbose,
                        previous_routes=None, base_obstacles=None, router=corridor_router
                    )
                if route_path:
                    connection_routes.append((nid, layer, route_path))
                    for sx, sy in sample_route_for_voronoi(route_path, sample_interval=voronoi_seed_interval):
                        k = (round(sx, 3), round(sy, 3))
                        if k not in seen:
                            seeds.append((sx, sy))
                            seen.add(k)
                    th_connected += 1
                else:
                    # Corridor routing failed: at least point-seed the pad so its own
                    # immediate cell is its net (better than landing in another net).
                    k = (round(px, 3), round(py, 3))
                    if k not in seen:
                        seeds.append((px, py))
                        seen.add(k)
                    th_fallback += 1
                    fallback_pad_refs.append(f"{pad.component_ref}.{pad.pad_number}")
        if th_connected or th_fallback or th_off_board:
            msg = f"  #107: routed corridors for {th_connected} orphaned TH pad(s)"
            if th_fallback:
                msg += (f"; {th_fallback} could not be reached on the plane layer "
                        f"and fell back to point-seed ({', '.join(fallback_pad_refs)})")
            if th_off_board:
                msg += (f"; {th_off_board} off-board pad(s) skipped "
                        f"(outside the outline, no seed)")
            print(msg)

    # #114: a net with no stitching vias gets no MST and therefore no Voronoi
    # seeds from the routing above. Seed it directly from its pads on this layer
    # (through-hole + SMD-on-layer), or -- with the layer still bare -- from its
    # pads projected onto it (#598), so it still receives a Voronoi-partitioned
    # zone. Via-bearing nets are left to the #107 corridor logic above so their
    # via topology is not perturbed. These are partition seeds ONLY: they never
    # enter vias_by_net, so no MST edge or plane route is invented for them.
    if augmented_vias_by_net is not None:
        for net_name in nets_with_seeds:
            net_id = net_name_to_id.get(net_name)
            if net_id is None or vias_by_net.get(net_id):
                continue
            seeds = augmented_vias_by_net.setdefault(net_id, [])
            seen = {(round(x, 3), round(y, 3)) for x, y in seeds}
            for px, py in (pads_on_layer_by_net.get(net_id) or
                           projected_pads_by_net.get(net_id, [])):
                k = (round(px, 3), round(py, 3))
                if k not in seen:
                    seeds.append((px, py))
                    seen.add(k)

    # Deferred under-BGA pads participate as point seeds so each rail's cell
    # reserves its ball territory before the fanout drops the actual vias
    # (pours run first, #562). ONLY nets assigned to THIS layer may receive
    # seeds: setdefault() here used to CREATE Voronoi entries for foreign
    # nets, carving their cells into a layer whose plane they don't own
    # (dilemma: 17 +3V3 confetti zones on the GND layer).
    _dseeds = getattr(pcb_data, '_deferred_bga_seeds', None) \
        if 'pcb_data' in dir() else None
    if _dseeds and augmented_vias_by_net:
        _seeded_nets = 0
        for _nid, _pts in _dseeds.items():
            _lst = augmented_vias_by_net.get(_nid)
            if _lst is None:
                continue  # net's plane lives on another layer
            _seen = {(round(x, 3), round(y, 3)) for x, y in _lst}
            _before = len(_lst)
            for _x, _y in _pts:
                _k = (round(_x, 3), round(_y, 3))
                if _k not in _seen:
                    _lst.append((_x, _y))
                    _seen.add(_k)
            if len(_lst) > _before:
                _seeded_nets += 1
        if _seeded_nets:
            print(f"  Added deferred-BGA point seeds for "
                  f"{_seeded_nets} net(s) to the Voronoi")

    # GRAMMAR POUR (#662, default ON; KICAD_GRAMMAR_POUR=0 reverts to the
    # pad-Voronoi partition): the dominant net becomes a BACKGROUND SHEET
    # (the whole layer) and every other net gets compact INFLATED CLUSTER
    # HULLS. zone_overlap_priorities then ranks the nested hulls above the
    # sheet (smaller area wins), so KiCad's fill does the subtraction --
    # no polygon booleans. Rationale (measured, orangecrab vs its human
    # original): Voronoi cells scored 0.3-2.6mm mean width and split the
    # dominant rail into 7 crumbs (copper confetti with maximal neck-to-
    # interior ratio -- every island is a weld obligation); the human's
    # grammar is one deep sheet (15.8mm mean width) + compact islands
    # (>=5mm). Connectedness is the binding metric, not pad coverage.
    if os.environ.get('KICAD_GRAMMAR_POUR', '1') != '0' \
            and len([n for n, s in augmented_vias_by_net.items() if s]) > 1:
        _gz = _grammar_zone_polygons(
            augmented_vias_by_net, zone_polygon, board_bounds,
            {nid: (pcb_data.nets[nid].name if nid in pcb_data.nets else str(nid))
             for nid in augmented_vias_by_net}, verbose,
            pads_by_net={nid: [(pd.global_x, pd.global_y)
                               for pd in pcb_data.pads_by_net.get(nid, [])]
                         for nid in augmented_vias_by_net},
            carve_mm=zone_clearance + min_thickness / 2.0)
        if _gz is not None:
            zone_polygons = _gz
            _skip_voronoi = True
        else:
            _skip_voronoi = False
    else:
        _skip_voronoi = False

    # Compute final Voronoi zones
    total_seeds = sum(len(vias) for vias in augmented_vias_by_net.values())
    if not _skip_voronoi:
        print(f"  Computing final Voronoi zones with {total_seeds} seed points")

    try:
        if not _skip_voronoi:
            zone_polygons, _, _ = compute_zone_boundaries(
                augmented_vias_by_net, board_bounds,
                return_raw_polygons=True,
                board_edge_clearance=board_edge_clearance,
                verbose=verbose
            )
    except ValueError as e:
        print(f"  Error computing zone boundaries: {e}")
        print(f"  Falling back to full board rectangle for first net")
        net_name = nets_with_seeds[0]
        net_id = net_name_to_id[net_name]
        zone_sexpr = generate_zone_sexpr(
            net_id=net_id,
            net_name=net_name,
            layer=layer,
            polygon_points=zone_polygon,
            clearance=zone_clearance,
            min_thickness=min_thickness,
            direct_connect=not thermal_relief,
            use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION
        )
        zone_sexprs.append(zone_sexpr)
        zone_data_list.append({
            'thermal_relief': thermal_relief,
            'net_id': net_id,
            'net_name': net_name,
            'layer': layer,
            'polygon_points': zone_polygon,
            'clearance': zone_clearance,
            'min_thickness': min_thickness,
        })
        return zone_sexprs, debug_line_sexprs, zone_data_list

    # Voronoi cells are NOT guaranteed disjoint -- usp_obc_v7's In1.Cu came out
    # with the whole Net-(U5-GND) pour nested inside the GND cell. Overlapping
    # zones left at equal priority have no defined winner in KiCad, which then
    # tie-breaks on UUID, so the FILL varied run to run over identical copper.
    # Give contending zones distinct priorities before emitting any of them.
    _flat = [(layer, nid, poly)
             for nid, polys in zone_polygons.items() for poly in polys]
    _prios = zone_overlap_priorities(_flat)
    # priority_offset (>0) lifts this whole layer above a FOREIGN net's pour that
    # was already there, keeping the relative order among our own zones intact.
    _prio_of = {}
    _k = 0
    for nid, polys in zone_polygons.items():
        for poly_idx in range(len(polys)):
            _prio_of[(nid, poly_idx)] = _prios[_k] + priority_offset
            _k += 1
    if any(_prios):
        print(f"  Overlapping zones on {layer}: assigned fill priorities "
              + ", ".join(
                  f"{(pcb_data.nets.get(nid).name if pcb_data.nets.get(nid) else nid)}"
                  f"[{pi}]={_prio_of[(nid, pi)]}"
                  for (nid, pi) in sorted(_prio_of, key=lambda k: -_prio_of[k])
                  if _prio_of[(nid, pi)]))

    # Generate zones for each net
    for net_id, polygons in zone_polygons.items():
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"net_{net_id}"
        for poly_idx, polygon in enumerate(polygons):
            if len(polygons) > 1:
                print(f"  Creating zone {poly_idx+1}/{len(polygons)} for '{net_name}' with {len(polygon)} vertices")
            else:
                print(f"  Creating zone for '{net_name}' with {len(polygon)} vertices")
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=layer,
                polygon_points=polygon,
                clearance=zone_clearance,
                min_thickness=min_thickness,
                direct_connect=not thermal_relief,
                use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION,
                priority=_prio_of.get((net_id, poly_idx), 0)
            )
            zone_sexprs.append(zone_sexpr)
            zone_data_list.append({
                'thermal_relief': thermal_relief,
                'net_id': net_id,
                'net_name': net_name,
                'layer': layer,
                'polygon_points': polygon,
                'clearance': zone_clearance,
                'min_thickness': min_thickness,
                # Carried for the GUI, which builds pcbnew ZONEs from this dict
                # instead of the s-expr -- without it the plugin would re-emit
                # the equal-priority ambiguity the CLI just resolved.
                'priority': _prio_of.get((net_id, poly_idx), 0),
            })

    # Calculate and print resistance. copper_oz comes from the board's OWN
    # stackup: this call omitted it, so every board was graded at 1 oz even when
    # the stackup said 2, while the thickness sat unread in the same PCBData
    # (#489 §6).
    resistance_results = {}
    copper_oz = stackup_copper_oz(pcb_data, layer)
    for net_id, polygons in zone_polygons.items():
        if not polygons:  # every island demoted (#662 invariant 3b)
            continue
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"net_{net_id}"
        mst_edges = net_mst_edges.get(net_id, [])
        edge_routes = routed_paths_by_edge.get(net_id, {})
        largest_polygon = max(polygons, key=lambda p: len(p))
        result = analyze_multi_net_plane(largest_polygon, mst_edges, edge_routes, layer,
                                        copper_oz=copper_oz)
        resistance_results[net_name] = result
        # #487: main() folds these into JSON_SUMMARY (stdout-only before).
        note_resistance_result(net_name, result)

    print_multi_net_resistance(resistance_results)

    # Carry the numbers with the zones instead of print-and-discard, so a caller
    # (or the GUI) can gate on them (#489 §6).
    for entry in zone_data_list:
        entry['resistance_analysis'] = resistance_results.get(entry['net_name'])

    return zone_sexprs, debug_line_sexprs, zone_data_list


def _geometric_plane_verification(
    output_file: str,
    net_ids: List[int],
    net_names: List[str],
    plane_layers: List[str],
) -> Dict[int, Dict]:
    """Geometric truth check of plane connectivity (issues #89 and #107).

    Re-parses the written output and, for each plane net, uses
    check_net_connectivity to count how many of the net's pads are actually
    joined to that net's plane copper. This catches two classes of failure the
    via-placement counters miss:

      * #89: a stitching via was placed/reused but is not electrically joined
        to the net's zone (so the via-placement success counter overcounts).
      * #107: on a multi-net Voronoi layer, a through-hole pad sits inside the
        OTHER net's Voronoi cell, so it never gets a via and is never counted,
        yet its own net's zone does not cover it -> geometrically disconnected.

    Returns a dict mapping net_id -> {name, layer, total, connected, failed,
    disconnected_pads}. Returns {} (and prints a warning) if the check could
    not be run, so callers fall back to the via-placement counters.
    """
    try:
        from check_connected import check_net_connectivity
        out_pcb = parse_kicad_pcb(output_file)
    except Exception as e:  # pragma: no cover - defensive
        print(f"WARNING: geometric plane verification could not run ({e}); "
              f"reported counts are via-placement estimates only.")
        return {}

    segs_by_net: Dict[int, List] = {}
    for s in out_pcb.segments:
        segs_by_net.setdefault(s.net_id, []).append(s)
    vias_by_net: Dict[int, List] = {}
    for v in out_pcb.vias:
        vias_by_net.setdefault(v.net_id, []).append(v)
    zones_by_net: Dict[int, List] = {}
    for z in out_pcb.zones:
        zones_by_net.setdefault(z.net_id, []).append(z)

    results: Dict[int, Dict] = {}
    for net_id, net_name, plane_layer in zip(net_ids, net_names, plane_layers):
        if net_id in results:
            continue  # already verified (same net listed twice)
        pads = out_pcb.pads_by_net.get(net_id, [])
        r = check_net_connectivity(
            net_id,
            segs_by_net.get(net_id, []),
            vias_by_net.get(net_id, []),
            pads,
            zones_by_net.get(net_id, []))
        disconnected = r.get('disconnected_pads', []) or []
        total = len(pads)
        failed = len(disconnected)
        results[net_id] = {
            'name': net_name,
            'layer': plane_layer,
            'total': total,
            'connected': total - failed,
            'failed': failed,
            'disconnected_pads': disconnected,
        }

    # Report the geometric truth so the summary matches reality.
    print(f"\n{'='*60}")
    print("GEOMETRIC VERIFICATION (re-parsed output)")
    print(f"{'='*60}")
    for net_id, info in results.items():
        status = GREEN if info['failed'] == 0 else RED
        print(f"  {status}{info['name']}: {info['connected']}/{info['total']} "
              f"pads connected to plane on {info['layer']}{RESET}")
        if info['failed']:
            # Attribute each geometrically-failed pad with net + location so
            # silently-skipped pads (#107) and unjoined vias (#89) are visible.
            for loc in info['disconnected_pads']:
                # disconnected_pads entries are (x, y, layer, component_ref)
                try:
                    px, py, player, pref = loc[0], loc[1], loc[2], loc[3]
                    print(f"      {RED}unconnected pad {pref} on '{info['name']}' "
                          f"at ({px:.2f}, {py:.2f}) [{player}]{RESET}")
                except (TypeError, ValueError, IndexError):
                    print(f"      {RED}unconnected pad on '{info['name']}': {loc}{RESET}")
    return results


def _neck_plane_segments(all_new_segments, pcb_data, clearance, all_layers,
                         min_width=0.1, net_clearances=None):
    """Neck plane tap segments so they clear foreign vias AND pads by the
    clearance KiCad will actually enforce for each pair: max(step clearance,
    tap net's class, foreign net's class, the foreign pad's resolved local
    override #326). A flat `clearance` under-necked against a pad with a
    larger local_clearance or (no-ceiling mode) a looser foreign class, so
    the neck survived our grading but KiCad flagged the graze.

    The tap router (route_via_to_pad) exempts its source/target cells from the
    obstacle map and appends an un-obstacle-checked stub to the pad centre, so a
    full-width tap can graze foreign copper that sits inside the keep-out near its
    target pad (e.g. a GND pad placed ~0.4mm from a signal via, or a power tap
    landing next to a foreign GND pad). Narrowing a trace only ever REMOVES a
    clearance conflict, never creates one, so we shrink each tap segment to the
    widest value that still clears every nearby foreign via and pad, floored at
    `min_width` (placement-limited residue stays as-is). This is the route_planes
    side of the terminal-graze issue #157.
    """
    import math
    from check_drc import segment_to_rect_distance, _into_pad_frame  # reuse exact DRC geometry
    vias = [(v.x, v.y, v.size / 2.0, v.net_id, v.layers) for v in pcb_data.vias]
    EPS = 1e-4  # stay just inside the rule
    _nc = net_clearances or {}

    def _pair_clearance(tap_nid, foreign_nid, pad=None):
        c = max(clearance, _nc.get(tap_nid, 0.0), _nc.get(foreign_nid, 0.0))
        if pad is not None and getattr(pad, 'local_clearance', 0):
            c = max(c, pad.local_clearance)
        return c

    def pad_on_layer(pad, layer):
        return '*.Cu' in pad.layers or layer in pad.layers

    def pad_corner_radius(pad):
        if pad.shape in ('circle', 'oval'):
            return min(pad.size_x, pad.size_y) / 2
        if pad.shape == 'roundrect':
            return getattr(pad, 'roundrect_rratio', 0.25) * min(pad.size_x, pad.size_y)
        return 0.0

    necked = 0
    for seg in all_new_segments:
        x0, y0 = seg['start']; x1, y1 = seg['end']
        base = seg['width']; layer = seg['layer']; nid = seg['net_id']
        half = base / 2.0
        new_half = half
        dx = x1 - x0; dy = y1 - y0; L2 = dx * dx + dy * dy
        minx, maxx = min(x0, x1), max(x0, x1)
        miny, maxy = min(y0, y1), max(y0, y1)
        # foreign vias
        for vx, vy, vr, vnid, vlayers in vias:
            if vnid == nid:
                continue
            if not (('F.Cu' in vlayers and 'B.Cu' in vlayers) or layer in vlayers):
                continue
            _cl = _pair_clearance(nid, vnid)
            margin = half + vr + _cl
            if vx < minx - margin or vx > maxx + margin or vy < miny - margin or vy > maxy + margin:
                continue
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((vx - x0) * dx + (vy - y0) * dy) / L2))
            d = math.hypot(vx - (x0 + t * dx), vy - (y0 + t * dy))
            allowed = d - vr - _cl - EPS
            if allowed < new_half:
                new_half = allowed
        # foreign pads on this layer (edge-to-edge distance via the DRC geometry)
        for pnid, pads in pcb_data.pads_by_net.items():
            if pnid == nid:
                continue
            for pad in pads:
                if not pad_on_layer(pad, layer):
                    continue
                _cl = _pair_clearance(nid, pnid, pad)
                pext = max(pad.size_x, pad.size_y) / 2.0
                margin = half + pext + _cl
                if pad.global_x < minx - margin or pad.global_x > maxx + margin or \
                   pad.global_y < miny - margin or pad.global_y > maxy + margin:
                    continue
                sx0, sy0, ex0, ey0 = x0, y0, x1, y1
                if pad.rect_rotation:
                    rad = math.radians(pad.rect_rotation)
                    cr, sr = math.cos(rad), math.sin(rad)
                    sx0, sy0 = _into_pad_frame(sx0, sy0, pad, cr, sr)
                    ex0, ey0 = _into_pad_frame(ex0, ey0, pad, cr, sr)
                d, _ = segment_to_rect_distance(sx0, sy0, ex0, ey0,
                                                pad.global_x, pad.global_y,
                                                pad.size_x / 2, pad.size_y / 2,
                                                pad_corner_radius(pad))
                allowed = d - _cl - EPS  # d is already edge-to-edge from the pad
                if allowed < new_half:
                    new_half = allowed
        floor_half = min(base, min_width) / 2.0
        new_half = max(new_half, floor_half)
        if new_half < half - 1e-9:
            seg['width'] = round(2.0 * new_half, 4)
            necked += 1
    if necked:
        print(f"  Necked {necked} plane tap segment(s) to clear foreign vias/pads")
    return necked


def _finalize_plane_copper(all_new_segments, all_new_vias, pcb_data, clearance,
                           all_layers, track_width, grid_step, via_size,
                           via_drill, hole_to_hole_clearance,
                           net_clearances=None, strip_sink=None,
                           same_net_pad_clearance=None):  # #581
    """Run the full pre-write cleanup pipeline on plane tap copper and return the
    resulting all_new_segments.

    This is the copper-finalizing half of the plane write path, factored out so
    the file-writer (_write_plane_output) and the dry-run/GUI path
    (create_plane, which never enters the writer) run the SAME steps and thus
    emit the SAME copper -- the CLI and GUI front-ends must produce identical
    output for identical inputs. The steps, in order:

      1. Neck tap segments grazing foreign vias/pads near their target pads
         (the tap router exempts source/target cells, #157).
      2. Graze prune / re-bend / dead-end sweep: drop a redundant grazing tap or
         re-bend a load-bearing one around the pad, then trim orphans (#224).
      3. Close soft joints: bridge 10-100um same-net endpoint gaps left by tap
         snaps / piece-level restore drops (#334) -- MUST be last, since it needs
         the finalized endpoint picture (steps 1-2 trim copper an early call
         would misread as extra endpoints).

    Returns the (possibly replaced) all_new_segments list; the caller must
    rebind its own reference to it.
    """
    # 0. reuse same-net vias that violate hole-to-hole (two independently-placed
    #    plane taps landing a grid cell apart -> overlapping drills). MUST run before
    #    the graze/soft-joint passes so they see the merged via set + reconnected
    #    segment endpoints.
    from pcb_modification import merge_close_same_net_vias
    merge_close_same_net_vias(all_new_vias, all_new_segments, pcb_data,
                              hole_to_hole_clearance)

    # 1. neck grazing taps (mutates all_new_segments in place)
    _neck_plane_segments(all_new_segments, pcb_data, clearance, all_layers,
                         net_clearances=net_clearances)

    # 2. graze prune / nudge / dead-end sweep (returns a NEW list)
    if all_new_segments:
        from pcb_modification import cleanup_plane_taps_grazing
        scope = {s['net_id'] for s in all_new_segments}
        (all_new_segments, gz_rm, gz_nudge, gz_swept,
         gz_input_strips) = cleanup_plane_taps_grazing(
            pcb_data, all_new_segments, scope, clearance=clearance,
            max_shift=grid_step / 2, all_new_vias=all_new_vias,
            hole_to_hole=hole_to_hole_clearance,
            same_net_pad_clearance=(same_net_pad_clearance
                                    if same_net_pad_clearance is not None
                                    else -1.0))  # #581
        if gz_rm:
            print(f"  Graze prune: removed {gz_rm} grazing tap segment(s)")
        if gz_nudge:
            print(f"  Graze nudge: re-bent grazing tap jog(s) on {gz_nudge} net(s)")
        if gz_swept:
            print(f"  Dead-end sweep: trimmed {gz_swept} orphaned tap segment(s)")
        # #508 finding 2: INPUT copper the passes deleted from pcb_data must
        # reach the writer/applier strip channel or the output re-emits it.
        if gz_input_strips and strip_sink is not None:
            strip_sink.extend(gz_input_strips)
            print(f"  Graze/sweep passes removed {len(gz_input_strips)} "
                  f"input-board segment(s); forwarded to the strip channel")

    # 3. close soft joints (#334) -- last, appends bridges (from_restore=True)
    try:
        from pcb_modification import close_soft_joints
        bridge_results = []
        cfg_sj = GridRouteConfig(track_width=track_width, clearance=clearance,
                                 grid_step=grid_step, via_size=via_size,
                                 via_drill=via_drill, layers=all_layers)
        from kicad_dru import install_layer_clearances
        install_layer_clearances(cfg_sj, None, None, pcb_data)  # #498
        nb = close_soft_joints(bridge_results, pcb_data, None, cfg_sj)
        if nb:
            for br in bridge_results:
                for bs in br.get('new_segments', []):
                    all_new_segments.append({
                        'start': (bs.start_x, bs.start_y),
                        'end': (bs.end_x, bs.end_y),
                        'width': bs.width, 'layer': bs.layer,
                        'net_id': bs.net_id, 'from_restore': True})
            print(f"  Closed {nb} soft joint(s) in plane/restored copper")
    except Exception as e:
        print(f"  (soft-joint close skipped: {e})")

    return all_new_segments


def _write_plane_output(
    input_file: str,
    output_file: str,
    all_zone_sexprs: List[str],
    all_debug_lines: List[str],
    all_new_vias: List[Dict],
    all_new_segments: List[Dict],
    all_ripped_net_ids: List[int],
    zones_to_replace: List[Tuple[int, str]],
    pcb_data: PCBData,
    add_teardrops: bool = False,
    removed_input_segments: Optional[List] = None
) -> bool:
    """
    Write the plane output file. (Renamed from _write_output_and_reroute
    and trimmed from 21 params to 11, review dead-code 6: the "reroute"
    half was the unreachable ripped-net machinery deleted in c126987, and
    14 of the params fed only it.)

    removed_input_segments (#508 finding 2): input-board Segment objects the
    finalize passes deleted from pcb_data -- the writer must strip their text
    or the output re-emits copper the router withdrew (board != file).

    Returns:
        True if output was written successfully
    """
    print(f"\nWriting output to {output_file}...")
    all_sexprs = all_zone_sexprs + all_debug_lines
    combined_zone_sexpr = '\n'.join(all_sexprs) if all_sexprs else None
    if all_debug_lines:
        print(f"  Adding {len(all_debug_lines)} debug lines on User.4")

    kicad_v10_names = pcb_data.net_id_to_name if pcb_data.kicad_version >= KICAD_10_MIN_VERSION else None
    if not write_plane_output(input_file, output_file, combined_zone_sexpr, all_new_vias, all_new_segments,
                              exclude_net_ids=all_ripped_net_ids, zones_to_replace=zones_to_replace,
                              add_teardrops=add_teardrops, net_id_to_name=kicad_v10_names,
                              removed_segments=removed_input_segments):
        print("Error writing output file")
        return False

    print(f"Output written to {output_file}")
    print("Note: Open in KiCad and press 'B' to refill zones")

    # Board-vs-file ledger (KICAD_BOARD_LEDGER=1, #508): the written file must
    # match pcb_data for every net this run touched -- the class of bug where
    # one pass changes the write list without pcb_data (or vice versa) is
    # exactly what this audit exists to catch, and this engine had NO ledger
    # call (the one ledgered engine, route.py, came back clean; every serious
    # #508 finding sat on an unledgered path). No-op unless the env var is set.
    # Runs BEFORE the ripped-net reroute below: that reroute is file-based and
    # deliberately not mirrored into pcb_data.
    from cleanup_pipeline import verify_written_file_parity
    _ledger_scope = sorted({d['net_id'] for d in all_new_segments}
                           | {d['net_id'] for d in all_new_vias}
                           | set(all_ripped_net_ids))
    verify_written_file_parity(output_file, pcb_data, _ledger_scope,
                               label=' planes')

    # RIPPED-NET REROUTE MACHINERY DELETED (#562). `all_ripped_net_ids`
    # is contractually always empty now: the pour places only thermal
    # arrays and cannot rip (the per-net `ripped_net_ids` list is
    # initialized empty and nothing appends to it). The ~160-line block
    # that lived here -- _verify_broken_ripped_nets + a batch_route
    # self-invocation rerouting the broken subset -- was therefore
    # unreachable. The variable itself stays: the GUI 10-tuple contract
    # (#382 E5 / #508) includes ripped_net_ids, and the writer's
    # exclude_net_ids param keeps its (empty) argument.
    if all_ripped_net_ids:
        print(f"WARNING: pour reported {len(all_ripped_net_ids)} ripped "
              f"net(s), but the pour cannot rip since #562 -- a code "
              f"path is filling ripped_net_ids again; restore the "
              f"reroute machinery (deleted 2026-08-04) or fix the "
              f"filler.")

    return True


def _empty_plane_results(return_results: bool):
    """Zero-work create_plane result in the shape the caller expects (#382 E5).

    The GUI (return_results=True) unpacks EXACTLY 10 values; the CLI unpacks 3.
    Every validation-error early return must go through here so a bad-input exit
    can't hand the GUI a short tuple it will ValueError on -- the bug this
    consolidates. The 10-field shape mirrors the full return_results path:
    (vias, traces, pads_needing, new_vias, new_segments, new_zones,
     failed_pads, ripped_net_ids, reconnect_swap_data (#484 H3),
     reconnect_strips (#508)).
    """
    if return_results:
        return (0, 0, 0, [], [], [], 0, [], {}, [])
    return (0, 0, 0)


def _resolve_zone_clearance(zone_clearance, clearance, min_thickness,
                            pcb_data, via_size, net_names=None):
    """Resolve the pour clearance (see call site). None -> the routed
    clearance; then, if the densest BGA via lattice the pour must serve is
    tighter than 2*zc + min_thickness, escalate DOWN to what threads --
    floored at the fab minimum clearance -- with a warning either way.

    The resolved value is recorded in the clearance ledger: a KiCad refill
    regrows the pour at max(zone clearance, netclass), so a sub-nominal pour
    clearance (escalated, or an explicit tight --zone-clearance) only
    survives refill if the project floor drops with it. Without the record,
    the .kicad_pro writeback kept the nominal floor and the refilled pour
    sealed the very BGA corridors the escalation opened."""
    zc = _resolve_zone_clearance_impl(
        zone_clearance, clearance, min_thickness, pcb_data, via_size,
        net_names)
    import clearance_ledger
    clearance_ledger.record(zc)
    return zc


def _resolve_zone_clearance_impl(zone_clearance, clearance, min_thickness,
                                 pcb_data, via_size, net_names=None):
    from kicad_parser import find_components_by_type, detect_bga_pitch
    from list_nets import fab_floors
    explicit = zone_clearance is not None
    zc = zone_clearance if explicit else clearance
    if not explicit:
        print(f"Zone clearance: following routed clearance {zc:g}mm "
              f"(--zone-clearance to override)")
    # Tightest pour channel across BGA fields: adjacent lattice gap
    # pitch - via_size (the via size a field carries is dominated by the
    # routing/fanout vias; use the run's via_size as the estimate).
    plane_ids = {i for i, n in pcb_data.nets.items()
                 if n.name in set(net_names or [])}
    tightest = None
    for fp in find_components_by_type(pcb_data, 'BGA'):
        pitch = detect_bga_pitch(fp)
        if not pitch:
            continue
        # Only fields the pour must actually serve: a BGA with no plane-net
        # via inside it constrains nothing (U6's 0.5mm pitch false-warned).
        _xs = [p2.global_x for p2 in fp.pads]
        _ys = [p2.global_y for p2 in fp.pads]
        if not any(v.net_id in plane_ids
                   and min(_xs) - 0.5 < v.x < max(_xs) + 0.5
                   and min(_ys) - 0.5 < v.y < max(_ys) + 0.5
                   for v in pcb_data.vias):
            continue
        # Gap from the vias ACTUALLY in the field (fanout vias, typically
        # smaller than the run's --via-size); fall back to the run size on
        # a virgin field.
        _in = [v.size for v in pcb_data.vias
               if min(_xs) - 0.5 < v.x < max(_xs) + 0.5
               and min(_ys) - 0.5 < v.y < max(_ys) + 0.5 and v.size > 0]
        field_via = max(_in) if _in else via_size
        gap = pitch - field_via
        if tightest is None or gap < tightest:
            tightest = gap
    if tightest is None:
        return zc
    needed = (tightest - min_thickness) / 2.0
    if needed >= zc:
        return zc
    ncu = (len(pcb_data.board_info.copper_layers)
           if pcb_data.board_info.copper_layers else 2)
    floor = fab_floors(ncu).get('clearance', 0.09)
    if needed < floor:
        print(f"  WARNING: pour cannot thread the densest BGA lattice even "
              f"at the fab floor (needs {needed:.3f}mm < floor {floor:g}); "
              f"zone clearance stays {zc:g} -- interior plane vias in that "
              f"field may be unreachable islands")
        return zc
    if explicit:
        print(f"  WARNING: explicit zone clearance {zc:g} cannot thread the "
              f"densest BGA lattice (gap {tightest:.3f}mm needs <= "
              f"{needed:.3f}); keeping it -- pass a smaller --zone-clearance "
              f"or expect isolated plane islands there")
        return zc
    print(f"  Zone clearance tightened {zc:g} -> {needed:.3f}mm so the pour "
          f"threads the densest BGA lattice (gap {tightest:.3f}mm, min "
          f"width {min_thickness:g}; fab floor {floor:g})")
    return round(needed, 4)


# #487 narrowing (Andy): thermal-via arrays belong on true EP/tab pads of
# HEAT-MAKING parts, not on every big pad -- a 2512 sense resistor or bulk
# cap pad clears 2mm easily. Reference prefixes of active devices:
_THERMAL_REF_PREFIXES = ('U', 'Q', 'D', 'T', 'VR', 'IC', 'PS', 'LED')
# Footprint-name tokens that name an exposed pad / power tab outright
# (KiCad library convention: QFN-32-1EP_5x5mm, TO-252-2, HTSSOP...).
_THERMAL_FP_TOKENS = re.compile(
    r'(?i)(-\d?EP[_\b-]|_EP_|ThermalPad|ThermalVias|D2?PAK|TO-2(20|52|63)|'
    r'PowerPAK|HTSSOP|HSOP|PowerSO|SOT-223|HSOF|Powermite|VREG)')


def is_thermal_pad(pad, pcb_data, min_mm: float = None) -> bool:
    """True when a plane-net pad deserves a thermal-via ARRAY (#487).

    Size alone over-triggers (big cap/resistor pads), so require ALL of:
      * SMD and >= min_mm in both axes (the copper must hold a lattice);
      * an ACTIVE component reference prefix (U/Q/D/T/VR/IC/PS/LED) --
        passives and connectors are excluded by name;
      * the EP/tab SIGNATURE: this is the footprint's largest pad AND at
        least 3x the median pad area (QFN/DFN center EPs, D-Pak drain
        tabs), OR the footprint name itself declares an exposed pad
        (-1EP / TO-252 / HTSSOP / ...).
    """
    if min_mm is None:
        min_mm = defaults.THERMAL_PAD_MIN_MM
    if pad.drill != 0 or min(pad.size_x, pad.size_y) < min_mm:
        return False
    ref = pad.component_ref or ''
    m = re.match(r'([A-Za-z]+)', ref)
    if not m or m.group(1).upper() not in _THERMAL_REF_PREFIXES:
        return False
    fp = pcb_data.footprints.get(ref)
    if fp is None:
        return False
    if _THERMAL_FP_TOKENS.search(getattr(fp, 'footprint_name', '') or ''):
        return True
    areas = sorted(p.size_x * p.size_y for p in fp.pads
                   if p.pad_type != 'np_thru_hole' and p.size_x and p.size_y)
    if len(areas) < 2:
        return False
    my_area = pad.size_x * pad.size_y
    median = areas[len(areas) // 2]
    # 2.5x, not 3.0: corpus scan (406 boards) showed SOT-223 tabs at 2.9x
    # and multi-big-pad power packages just under 3x -- the heat-makers
    # this exists for -- while passives stay excluded by prefix anyway.
    return my_area >= max(areas) - 1e-9 and my_area >= 2.5 * median


def compute_thermal_via_array(pad, obstacles, coord, config, via_size,
                              via_drill, hole_to_hole_clearance, pcb_data,
                              pitch: float = None) -> List[Tuple[float, float]]:
    """Lattice of thermal-via sites over an exposed pad's REAL copper (#487).

    A 5x5mm QFN exposed pad used to get ONE via out of the reuse/strap logic
    ("share ONE via plus short pad-layer straps") -- electrically fine,
    thermally backwards: the exposed pad's whole job is to pump heat into the
    plane through a via field. Grid the pad at ``pitch`` (default: the
    largest of 2 via diameters, drill + hole-to-hole, and 1.0mm -- classic
    thermal-via spacing that also guarantees lattice-internal drill
    clearance), keep only sites whose annulus sits fully on the (rotated)
    pad copper, and cull each through the SAME predicates single-via
    placement uses: obstacles.is_via_blocked + the board-edge clamp. The
    caller re-checks blocked-ness as it commits sites (each placed via
    blocks its hole-to-hole neighborhood). Returns world (x, y) sites,
    possibly empty -- the caller then falls through to the normal path.
    """
    if pitch is None:
        pitch = max(via_size * 2.0, via_drill + hole_to_hole_clearance, 1.0)
    hw, hh = pad_rect_halfspan(pad)
    inset = via_size / 2.0  # annulus fully on the pad copper
    span_x, span_y = hw - inset, hh - inset
    if span_x < 0 or span_y < 0:
        return []
    nx = max(1, int(span_x * 2 / pitch) + 1)
    ny = max(1, int(span_y * 2 / pitch) + 1)
    xs = ([pad.global_x] if nx == 1 else
          [pad.global_x - span_x + i * (2 * span_x / (nx - 1)) for i in range(nx)])
    ys = ([pad.global_y] if ny == 1 else
          [pad.global_y - span_y + i * (2 * span_y / (ny - 1)) for i in range(ny)])
    is_circle = getattr(pad, 'shape', '') == 'circle'
    r_max = min(pad.size_x, pad.size_y) / 2.0 - inset
    sites = []
    for y in ys:
        for x in xs:
            if is_circle:
                if ((x - pad.global_x) ** 2 + (y - pad.global_y) ** 2) > r_max ** 2:
                    continue
            elif getattr(pad, 'rect_rotation', 0.0) and not point_in_pad_rect(x, y, pad):
                continue
            gx, gy = coord.to_grid(x, y)
            if obstacles.is_via_blocked(gx, gy):
                continue
            pos, _ = clamp_tap_via_to_edge((x, y), pad, pcb_data, config, via_size)
            if abs(pos[0] - x) > pitch / 2 or abs(pos[1] - y) > pitch / 2:
                continue  # clamp had to drag it off its lattice cell
            sites.append(pos)
    return sites


def _stitch_pitch_from_freq(pcb_data: PCBData, max_freq_mhz: float,
                            floor_mm: float = 1.0):
    """lambda/20 stitching pitch for a maximum frequency of interest (#485).

    Uses the LARGEST dielectric epsilon_r in the board's stackup
    (conservative: the highest permittivity gives the shortest in-material
    wavelength, hence the tightest pitch); FR-4's 4.5 when the board has no
    stackup section. Floored at ``floor_mm`` -- below that the via-via
    spacing floors reject most sites anyway and the lattice loop cost
    explodes.

    Returns (pitch_mm, epsilon_r, epsilon_from_stackup, lambda_mm).
    """
    import math
    ers = [sl.epsilon_r for sl in (pcb_data.board_info.stackup or [])
           if sl.layer_type in ('core', 'prepreg') and sl.epsilon_r]
    er = max(ers) if ers else 4.5
    lam_mm = 299792.458 / (max_freq_mhz * math.sqrt(er))  # c = 299792.458 mm*MHz
    return max(lam_mm / 20.0, floor_mm), er, bool(ers), lam_mm


def _fence_ring_points(pcb_data: PCBData, inset: float,
                       pitch: float) -> List[Tuple[float, float]]:
    """Candidate sites for a board-edge via fence (#485): the board outline(s)
    offset ``inset`` toward the interior, sampled at ~``pitch`` arc-length
    spacing. Uses the true Edge.Cuts polygon(s) when present -- a panelized
    board fences EVERY outline (#304) -- else the bounding box. Interior
    cutouts are not fenced. An inset that swallows a narrow board region
    simply yields no ring there (shapely negative buffer)."""
    from shapely.geometry import Polygon
    bi = pcb_data.board_info
    rings = [o for o in (getattr(bi, 'board_outlines', None) or [])
             if len(o) >= 3]
    if not rings:
        if not bi.board_bounds:
            return []
        min_x, min_y, max_x, max_y = bi.board_bounds
        rings = [[(min_x, min_y), (max_x, min_y),
                  (max_x, max_y), (min_x, max_y)]]
    pts: List[Tuple[float, float]] = []
    for ring in rings:
        try:
            poly = Polygon(ring)
            if not poly.is_valid:
                poly = poly.buffer(0)
            inner = poly.buffer(-inset, join_style=2)
        except Exception:
            continue
        for g in getattr(inner, 'geoms', [inner]):
            ext = getattr(g, 'exterior', None)
            if g.is_empty or ext is None:
                continue
            n = max(4, int(round(ext.length / pitch)))
            for i in range(n):
                p = ext.interpolate(ext.length * i / n)
                pts.append((p.x, p.y))
    return pts


def _build_exact_stitch_validator(pcb_data, stitch_net_ids, net_display,
                                  owned_layers, input_file, zone_sexprs,
                                  tap_vias, tap_segments, exclude_net_ids,
                                  verbose=False):
    """KiCad-exact stitch-site truth: {net_id: {layer: [(bbox, poly), ...]}}
    of the net's MAIN-cluster filled islands, or None when unavailable.

    Why (#485 kbic65): the ZoneFillModel gate can be over-CONNECTED versus
    KiCad's exact fill (dense 2-layer board, 0.254 min_thickness -> 336 exact
    islands where the model saw one main pour). Every via placed on such a
    site anchors an isolated sliver -- island removal keeps a fragment that
    touches a net item -- so kicad-cli reported one unconnected GND item PER
    STITCH VIA (38/38) and the next repair step's oracle strapped to them for
    2045s, leaving min-web necks. Stitch vias are pure additions: a site the
    exact fill does not place on the main cluster must be REJECTED, never
    placed-then-strapped-to.

    One pcbnew refill of a temp board carrying the input copper (minus nets
    ripped this run), this run's zone s-exprs and tap copper -- everything
    except the stitch vias themselves, so anchoring cannot mask a sliver.
    Returns None (caller falls back to the model-only gate, with a printed
    warning) when there is no source file or no KiCad python.
    """
    import shutil as _sh
    import tempfile as _tf
    src = input_file if input_file and os.path.isfile(input_file) else \
        getattr(pcb_data, 'source_path', None)
    if not src or not os.path.isfile(src):
        return None
    from kicad_exact_fill import exact_clusters, refill_islands
    from plane_io import write_plane_output
    tmpdir = _tf.mkdtemp(prefix='stitch_exact_')
    try:
        tmp_board = os.path.join(tmpdir, os.path.basename(src))
        combined = '\n'.join(zone_sexprs) if zone_sexprs else None
        if not write_plane_output(
                src, tmp_board, combined, list(tap_vias or []),
                list(tap_segments or []),
                exclude_net_ids=list(exclude_net_ids or []),
                net_id_to_name=getattr(pcb_data, 'net_id_to_name', None)):
            return None
        islands_map = refill_islands(tmp_board, project_from=src,
                                     verbose=verbose)
    except Exception as e:
        if verbose:
            print(f"  (exact stitch validation failed: {e})")
        return None
    finally:
        _sh.rmtree(tmpdir, ignore_errors=True)
    # `not`, not `is None` (#713 item 4): an EMPTY map is a refill that ran and
    # poured nothing, and it validates exactly as little as an unavailable one.
    # Letting it through built {net: {}} for every net, and the caller then
    # reported "exact fill has 'GND' main-cluster copper on 0 of its N owned
    # layer(s) (need 2) -- skipped" -- a message that blames the BOARD for what
    # was really an absent verdict, instead of the documented "validation
    # unavailable -- sites gated by the fill model only (#485)" line below.
    if not islands_map:
        return None
    out = {}
    for net_id in stitch_net_ids:
        net_name = net_display[net_id]
        net_islands = [(layer, poly)
                       for (nname, layer), polys in islands_map.items()
                       if nname == net_name for poly in polys]
        per_layer: Dict[str, list] = {}
        if net_islands:
            clusters = exact_clusters(pcb_data, net_id, net_islands)
            # Main cluster = the largest one holding pads (the real plane
            # network); a pad-less "largest" would let a big orphan pour win.
            main = next((c for c in clusters if c.get('has_pads')),
                        clusters[0] if clusters else None)
            for ii in (main['islands'] if main else ()):
                layer, poly = net_islands[ii]
                if layer not in owned_layers[net_id] or len(poly) < 3:
                    continue
                xs = [p[0] for p in poly]
                ys = [p[1] for p in poly]
                per_layer.setdefault(layer, []).append(
                    ((min(xs), min(ys), max(xs), max(ys)), poly))
        out[net_id] = per_layer
    return out


def _stitch_plane_area_vias(
    pcb_data: PCBData,
    net_names: List[str],
    plane_layers: List[str],
    net_ids: List[int],
    all_zone_data: List[Dict],
    config: GridRouteConfig,
    coord: GridCoord,
    stitch_pitch: float,
    via_size: float,
    via_drill: float,
    hole_to_hole_clearance: float,
    same_net_pad_clearance: float,
    stitch_lattice: bool = True,
    stitch_edge_fence: bool = False,
    stitch_fence_pitch: Optional[float] = None,
    stitch_inset: Optional[float] = None,
    board_edge_clearance: float = defaults.PLANE_EDGE_CLEARANCE,
    progress_callback=None,
    cancel_check=None,
    verbose: bool = False,
    input_file: Optional[str] = None,
    zone_sexprs: Optional[List[str]] = None,
    tap_vias: Optional[List[Dict]] = None,
    tap_segments: Optional[List[Dict]] = None,
    exclude_net_ids: Optional[List[int]] = None,
) -> List[Dict]:
    """Area via stitching + board-edge via fence (#485): bond each
    multi-layer plane net's pours across layers with a periodic via lattice
    at ``stitch_pitch`` (``stitch_lattice``), and/or run a via row along the
    board outline (``stitch_edge_fence``) at ``stitch_fence_pitch`` (default:
    the lattice pitch), inset ``stitch_inset`` from the edge (default: the
    board edge clearance plus the fill-margin ring, i.e. as close as a site
    can sit and still pass the fill gate). The fence runs FIRST so the
    lattice's coverage rule treats fence vias as existing bonds instead of
    doubling them up near the rim.

    Nets stitched: exactly the requested plane nets that own >= 2 of the
    requested plane layers (deliberately no CLI selection). Every site --
    fence or lattice -- is gated twice:

    - Predicted fill (ZoneFillModel over this run's zone geometry, or a kept
      existing zone): the via disk PLUS its clearance pocket PLUS a
      min_thickness ring must lie inside the MAIN fill component on >= 2 of
      the net's layers -- so a stitch never necks the pour below
      min_thickness locally and never taps a fill island.
    - The same via obstacle map pad taps use (foreign copper at cross-class
      clearance, drills at hole-to-hole, board edge, per-layer .kicad_dru),
      plus block_via_position between batch placements -- so the lattice
      cannot reintroduce the #274/#125/#271 same-net via-spacing class.

    A site already within pitch/2 of a same-net via or plated barrel is
    coverage-satisfied and skipped (the via-reuse rule); a blocked site is
    nudged outward ring-by-ring up to pitch/4 before being given up.

    Placed vias are appended to pcb_data.vias HERE (later passes must see
    them); the returned dicts are for all_new_vias only -- the caller must
    not append them to pcb_data again. Runs on dry runs too; the caller's
    dry_run split just skips the file write (#485 item 5).
    """
    if stitch_pitch <= 0:
        return []
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return []
    import math
    from types import SimpleNamespace
    from plane_fill_model import ZoneFillModel

    # Which requested nets own >= 2 distinct requested layers?
    owned_layers: Dict[int, List[str]] = {}
    net_display: Dict[int, str] = {}
    for net_name, layer, net_id in zip(net_names, plane_layers, net_ids):
        layers = owned_layers.setdefault(net_id, [])
        if layer not in layers:
            layers.append(layer)
        net_display[net_id] = net_name
    stitch_net_ids = [nid for nid, layers in owned_layers.items()
                      if len(layers) >= 2]
    if not stitch_net_ids:
        print("\nVia stitching: no requested net owns >= 2 plane layers -- "
              "nothing to stitch")
        return []

    # KiCad-exact site truth (#485): one refill, built lazily on the first
    # net that actually reaches placement (the anchor gate below skips
    # floating-pour nets without paying for a refill).
    _exact_state: Dict = {}

    def _get_exact_validator():
        if 'val' not in _exact_state:
            if progress_callback:
                progress_callback(0, 0,
                                  "Validating stitch sites (KiCad refill)...")
            val = _build_exact_stitch_validator(
                pcb_data, stitch_net_ids, net_display, owned_layers,
                input_file, zone_sexprs, tap_vias, tap_segments,
                exclude_net_ids, verbose=verbose)
            if val is None:
                print("Via stitching: KiCad exact-fill validation "
                      "unavailable -- sites gated by the fill model only "
                      "(a model/fill divergence can then anchor isolated "
                      "islands, #485)")
            _exact_state['val'] = val
        return _exact_state['val']

    min_x, min_y, max_x, max_y = board_bounds
    # Center the lattice on the board so coverage is symmetric.
    nx = max(1, int((max_x - min_x) / stitch_pitch))
    ny = max(1, int((max_y - min_y) / stitch_pitch))
    x0 = (min_x + max_x) / 2 - (nx - 1) * stitch_pitch / 2
    y0 = (min_y + max_y) / 2 - (ny - 1) * stitch_pitch / 2
    lattice = [(x0 + i * stitch_pitch, y0 + j * stitch_pitch)
               for j in range(ny) for i in range(nx)]

    # Nudge-search geometry: rings at ~coarse steps out to max_nudge, each
    # candidate snapped to the routing grid (an off-grid site can sit up to
    # half a cell inside an obstacle the grid check calls clear).
    nudge_step = max(config.grid_step, min(0.5, stitch_pitch / 40))

    def _candidates(cx, cy, max_nudge):
        yield (round(cx / config.grid_step) * config.grid_step,
               round(cy / config.grid_step) * config.grid_step)
        r = nudge_step
        while r <= max_nudge:
            n_ang = max(8, int(2 * math.pi * r / nudge_step))
            for k in range(n_ang):
                ang = 2 * math.pi * k / n_ang
                yield (round((cx + r * math.cos(ang)) / config.grid_step)
                       * config.grid_step,
                       round((cy + r * math.sin(ang)) / config.grid_step)
                       * config.grid_step)
            r += nudge_step

    new_via_dicts: List[Dict] = []
    for net_id in stitch_net_ids:
        if cancel_check and cancel_check():
            print("\nVia stitching cancelled")
            break
        net_name = net_display[net_id]
        layers = owned_layers[net_id]
        if progress_callback:
            progress_callback(0, 0, f"Stitching '{net_name}' planes...")

        # Fill models per owned layer: this run's zone geometry first (single
        # net AND Voronoi polys -- all_zone_data carries both), else a kept
        # pre-existing zone in pcb_data. Each entry: (model, main component,
        # required free radius around a site).
        models_by_layer: Dict[str, List[Tuple]] = {}
        for layer in layers:
            zsrc = [SimpleNamespace(net_id=net_id, layer=layer,
                                    clearance=zd.get('clearance'),
                                    min_thickness=zd.get('min_thickness'),
                                    polygon=zd.get('polygon_points'))
                    for zd in all_zone_data
                    if zd.get('net_id') == net_id and zd.get('layer') == layer
                    and zd.get('polygon_points') and not zd.get('keepout')]
            if not zsrc:
                zsrc = [z for z in (getattr(pcb_data, 'zones', None) or [])
                        if z.net_id == net_id and z.layer == layer
                        and getattr(z, 'polygon', None)]
            for zone in zsrc:
                try:
                    model = ZoneFillModel(pcb_data, zone)
                except Exception:
                    continue
                if not model.ok:
                    continue
                main = model.largest_component()
                if not main:
                    continue
                zc = zone.clearance if zone.clearance is not None \
                    else defaults.PLANE_ZONE_CLEARANCE
                mth = zone.min_thickness if zone.min_thickness is not None \
                    else defaults.PLANE_MIN_THICKNESS
                margin_r = via_size / 2 + zc + mth
                models_by_layer.setdefault(layer, []).append(
                    (model, main, margin_r))
        if len(models_by_layer) < 2:
            print(f"\nVia stitching: '{net_name}' has predictable fill on "
                  f"{len(models_by_layer)} layer(s) (need 2) -- skipped")
            continue

        # Floating-pour gate (#485 kbic65, model-only so it also protects
        # KiCad-less environments): stitching bonds a net's pours ACROSS
        # layers, which only means anything if that pour system touches the
        # net's real network. kbic65's GND is 3 pads + 3 tracks inside a
        # copperpour-keepout band -- its full-board pours are floating
        # copper, and every stitch via placed on them became a KiCad
        # ratsnest item disconnected from the pad network (38/38), which
        # the next repair step's oracle then strapped to for 2045s. The
        # fill model's geometry was RIGHT here; the old gate just asked
        # "is there fill" when the load-bearing question is "is the fill
        # anchored".
        _anchored = False
        for p in pcb_data.pads_by_net.get(net_id, []):
            p_layers = layers if pad_is_plated_through(p) else \
                [l for l in layers if l in (p.layers or [])]
            for layer in p_layers:
                for model, main, _mr in models_by_layer.get(layer, ()):
                    if model.query_component(p.global_x, p.global_y) == main:
                        _anchored = True
                        break
                if _anchored:
                    break
            if _anchored:
                break
        if not _anchored:
            for v in pcb_data.vias:
                if v.net_id != net_id:
                    continue
                for layer in layers:
                    for model, main, _mr in models_by_layer.get(layer, ()):
                        if model.query_component(v.x, v.y) == main:
                            _anchored = True
                            break
                    if _anchored:
                        break
                if _anchored:
                    break
        if not _anchored:
            print(f"\nVia stitching: '{net_name}' main pour holds no "
                  f"same-net pad or via -- floating copper, nothing to "
                  f"bond (skipped, #485)")
            continue

        # KiCad-exact gate (#485): a site must land inside a MAIN-cluster
        # exact-fill island on >= 2 owned layers, else the via would anchor
        # an isolated sliver the next repair step's oracle straps to.
        exact_per_layer = None
        exact_validator = _get_exact_validator()
        if exact_validator is not None:
            exact_per_layer = exact_validator.get(net_id) or {}
            _n_exact = sum(1 for l in layers if exact_per_layer.get(l))
            if _n_exact < 2:
                print(f"\nVia stitching: KiCad exact fill has '{net_name}' "
                      f"main-cluster copper on {_n_exact} of its "
                      f"{len(layers)} owned layer(s) (need 2) -- skipped "
                      f"(sites would anchor isolated islands, #485)")
                continue

        from kicad_exact_fill import point_in_poly as _pip

        def _exact_layers_ok(x, y):
            """Owned layers whose exact MAIN-cluster fill contains (x, y).
            Pass-through (2) when the exact fill is unavailable."""
            if exact_per_layer is None:
                return 2
            n_ok = 0
            for layer in layers:
                for (bx0, by0, bx1, by1), poly in \
                        exact_per_layer.get(layer, ()):
                    if bx0 <= x <= bx1 and by0 <= y <= by1 \
                            and _pip(x, y, poly):
                        n_ok += 1
                        break
            return n_ok

        def _fill_layers_ok(x, y):
            """Layers whose MAIN fill component contains the full margin disk
            around (x, y). Ring-sampled at 8 points + center; a sample off the
            main component (island, pocket, outside model coverage) fails
            that model."""
            n_ok = 0
            for layer_models in models_by_layer.values():
                for model, main, margin_r in layer_models:
                    if model.query_component(x, y) != main:
                        continue
                    ok = True
                    for k in range(8):
                        ang = math.pi / 4 * k
                        if model.query_component(
                                x + margin_r * math.cos(ang),
                                y + margin_r * math.sin(ang)) != main:
                            ok = False
                            break
                    if ok:
                        n_ok += 1
                        break  # one passing model per layer is enough
            return n_ok

        # Existing same-net bond points: vias (full-stack barrels) and plated
        # through-hole pads already tie this net's layers together.
        bonds: List[Tuple[float, float]] = [
            (v.x, v.y) for v in pcb_data.vias if v.net_id == net_id]
        bonds.extend((p.global_x, p.global_y)
                     for p in pcb_data.pads_by_net.get(net_id, [])
                     if pad_is_plated_through(p))

        def _nearest_bond(x, y):
            return min((math.hypot(x - bx, y - by) for bx, by in bonds),
                       default=float('inf'))

        obstacles = build_via_obstacle_map(
            pcb_data, config, net_id, verbose=False,
            same_net_pad_clearance=same_net_pad_clearance)
        # Vias stitched for earlier nets in this pass are already in
        # pcb_data.vias (appended below), so the map above blocks them.

        def _try_site(cx, cy, pitch_local, max_nudge):
            """Gate + place one site. Returns (outcome, d0) where outcome is
            'no_fill' | 'off_exact' | 'covered' | 'blocked' | 'placed' and d0
            the pre-place distance to the nearest same-net bond (None for
            no_fill/off_exact)."""
            if _fill_layers_ok(cx, cy) < 2:
                return 'no_fill', None
            if _exact_layers_ok(cx, cy) < 2:
                return 'off_exact', None
            d0 = _nearest_bond(cx, cy)
            if d0 <= pitch_local / 2:
                return 'covered', d0
            for x, y in _candidates(cx, cy, max_nudge):
                gx, gy = coord.to_grid(x, y)
                if obstacles.is_via_blocked(gx, gy):
                    continue
                if _fill_layers_ok(x, y) < 2:
                    continue
                if _exact_layers_ok(x, y) < 2:
                    continue
                new_via_dicts.append({
                    'x': x, 'y': y, 'size': via_size, 'drill': via_drill,
                    'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id,
                })
                pcb_data.vias.append(Via(x=x, y=y, size=via_size,
                                         drill=via_drill,
                                         layers=['F.Cu', 'B.Cu'],
                                         net_id=net_id))
                block_via_position(obstacles, x, y, coord,
                                   hole_to_hole_clearance, via_drill,
                                   via_size, config.clearance)
                bonds.append((x, y))
                return 'placed', d0
            return 'blocked', d0

        # Board-edge via fence first: fence vias then count as bonds for the
        # lattice's coverage rule, so the rim isn't stitched twice.
        if stitch_edge_fence:
            fpitch = stitch_fence_pitch or stitch_pitch
            max_margin = max(m for lms in models_by_layer.values()
                             for (_mod, _comp, m) in lms)
            # Auto inset: edge clearance + the fill-margin ring puts the
            # ring samples exactly ON the pour boundary, where model cell
            # quantization flips sites arbitrarily -- cushion past it.
            inset = stitch_inset if stitch_inset else \
                board_edge_clearance + max_margin + 0.2
            ring = _fence_ring_points(pcb_data, inset, fpitch)
            print(f"\nEdge via fence '{net_name}' at {fpitch:g}mm pitch, "
                  f"{inset:.2f}mm inset ({len(ring)} outline sites)")
            f_counts = {'placed': 0, 'covered': 0, 'no_fill': 0,
                        'off_exact': 0, 'blocked': 0}
            for cx, cy in ring:
                outcome, _d0 = _try_site(cx, cy, fpitch, fpitch / 4)
                f_counts[outcome] += 1
            print(f"  Fence vias placed: {f_counts['placed']}  "
                  f"(covered by existing via/barrel: {f_counts['covered']}, "
                  f"no 2-layer fill: {f_counts['no_fill']}, "
                  f"off exact main fill: {f_counts['off_exact']}, "
                  f"no clear site: {f_counts['blocked']})")

        if stitch_lattice:
            print(f"\nVia stitching '{net_name}' across "
                  f"{'/'.join(sorted(models_by_layer))} at {stitch_pitch:g}mm "
                  f"pitch ({len(lattice)} lattice sites)")
            placed = covered = no_fill = off_exact = blocked = 0
            max_dist_before = max_dist_after = 0.0
            for cx, cy in lattice:
                outcome, d0 = _try_site(cx, cy, stitch_pitch,
                                        stitch_pitch / 4)
                if outcome == 'no_fill':
                    no_fill += 1
                    continue
                if outcome == 'off_exact':
                    off_exact += 1
                    continue
                max_dist_before = max(max_dist_before, d0)
                if outcome == 'covered':
                    covered += 1
                    max_dist_after = max(max_dist_after, d0)
                elif outcome == 'blocked':
                    blocked += 1
                    max_dist_after = max(max_dist_after, d0)
                else:
                    placed += 1
                    max_dist_after = max(max_dist_after,
                                         _nearest_bond(cx, cy))

            print(f"  Stitch vias placed: {placed}  "
                  f"(covered by existing via/barrel: {covered}, "
                  f"no 2-layer fill: {no_fill}, "
                  f"off exact main fill: {off_exact}, "
                  f"no clear site: {blocked})")
            if max_dist_before > 0:
                print(f"  Max lattice-site distance to nearest same-net bond: "
                      f"{max_dist_before:.1f}mm -> {max_dist_after:.1f}mm")

    if new_via_dicts:
        print(f"\nVia stitching total: {len(new_via_dicts)} via(s) added")
    return new_via_dicts


#: The pad tally from the last create_plane() in this process.
#: #487 folded plane RESISTANCE into the machine-readable summary for exactly
#: this reason and stopped one field short: the summary could report the
#: plane's ampacity but not whether it reached its pads. `complete: true,
#: status: "ok", EXIT=0` was the entire machine-readable story of a pour that
#: left 5 pads open -- the number is held three times inside create_plane, and
#: main() calls it as a bare statement, so none of it survived.
_LAST_PAD_TALLY: dict = {}


def consume_pad_tally() -> dict:
    """Take (and clear) the last pour's pad tally."""
    global _LAST_PAD_TALLY
    out, _LAST_PAD_TALLY = dict(_LAST_PAD_TALLY), {}
    return out


def create_plane(
    input_file: str,
    output_file: str,
    net_names: List[str],
    plane_layers: List[str],
    via_size: float = defaults.VIA_SIZE,
    via_drill: float = defaults.VIA_DRILL,
    track_width: float = defaults.TRACK_WIDTH,
    clearance: float = defaults.CLEARANCE,
    zone_clearance: float = None,
    min_thickness: float = defaults.PLANE_MIN_THICKNESS,
    ripup_blocker_select: str = defaults.RIPUP_BLOCKER_SELECT,
    grid_step: float = defaults.GRID_STEP,
    hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE,
    all_layers: List[str] = None,
    verbose: bool = False,
    dry_run: bool = False,
    layer_nets: Dict[str, List[str]] = None,
    plane_proximity_radius: float = 3.0,
    plane_proximity_cost: float = 2.0,
    plane_track_via_clearance: float = defaults.PLANE_TRACK_VIA_CLEARANCE,
    board_edge_clearance: float = defaults.PLANE_EDGE_CLEARANCE,
    voronoi_seed_interval: float = 2.0,
    plane_max_iterations: int = defaults.MAX_ITERATIONS,
    debug_lines: bool = False,
    layer_costs: Optional[List[float]] = None,
    power_nets: Optional[List[str]] = None,
    power_nets_widths: Optional[List[float]] = None,
    add_teardrops: bool = False,
    pcb_data: Optional[PCBData] = None,
    return_results: bool = False,
    # #581: None (default) auto-reads the persisted .kicad_pro record;
    # explicit values keep their meaning (> 0 active, >= 0 legacy
    # stitching semantics, -1 explicitly allows via-in-pad).
    same_net_pad_clearance: Optional[float] = None,
    skip_existing_zones: bool = False,
    no_bga_zone: bool = False,
    progress_callback=None,
    cancel_check=None,
    net_clearances: Optional[dict] = None,
    clamp_netclasses: bool = True,
    clearance_ceiling: Optional[float] = None,
    thermal_relief: bool = False,
    thermal_vias: bool = defaults.THERMAL_VIAS,
    stitch_vias: bool = False,
    stitch_pitch: float = defaults.STITCH_PITCH,
    stitch_edge_fence: bool = False,
    stitch_fence_pitch: Optional[float] = None,
    stitch_inset: Optional[float] = None,
    stitch_max_freq: Optional[float] = None,
) -> Union[Tuple[int, int, int],
           Tuple[int, int, int, list, list, list, int, list]]:
    """
    Create copper pour zones for multiple nets (#562 bare pour), plus optional via
    features (thermal arrays #487, stitch vias #485); per-pad tap vias are deferred
    to the route step.

    Args:
        net_names: List of net names to process (e.g., ['GND', 'VCC'])
        plane_layers: List of layers for each net (e.g., ['In1.Cu', 'In2.Cu'])
        plane_proximity_radius: Radius around other nets' vias for proximity cost (mm).
        plane_proximity_cost: Maximum proximity cost around other nets' vias (mm equivalent).
        plane_track_via_clearance: Clearance from track center to other nets' via centers (mm).
            MST routes will avoid regions within this distance of other nets' vias to
            leave room for polygon fill.
        board_edge_clearance: Clearance from board edge for zone polygons (mm).
        voronoi_seed_interval: Sample interval for Voronoi seed points along routes (mm).
        plane_max_iterations: Max A* iterations for routing plane connections.
        same_net_pad_clearance: Edge-to-edge clearance (mm) between stitching vias and
            same-net pads. -1 (default) allows via-in-pad placement. Any value >= 0
            forces vias to be placed outside same-net pads with that much clearance.
        skip_existing_zones: When True, if a zone already exists on the target layer
            for the same net (in either the input file or the provided pcb_data), do
            not create another one - just place stitching vias to the existing zone.
            When False (CLI default), an existing zone on the target layer for the
            same net is replaced.
        progress_callback: Optional callable(current, total, label) invoked at
            phase milestones and per-pad during via placement, mirroring
            batch_route's callback (issue #364). (0, 0, label) marks an
            indeterminate phase. Called from whatever thread runs the engine;
            GUI callers must marshal to the UI thread themselves.
        cancel_check: Optional callable returning True to abort. Checked at
            net and pad boundaries; on cancel the function returns early with
            the work done so far -- a cancelling caller should DISCARD the
            partial results (the GUI does).

    Returns:
        return_results=False (CLI): the 3-tuple
            (total_vias_placed, total_traces_added, total_pads_needing_vias).
        return_results=True (GUI): the 10-tuple
            (total_vias_placed, total_traces_added, total_pads_needing_vias,
             new_vias, new_segments, new_zones, total_failed_pads,
             ripped_net_ids, reconnect_swap_data (#484 H3),
             reconnect_strips (#508)).
        Every early/error exit returns the caller-appropriate shape via
        _empty_plane_results (never a short tuple).
    """
    _dump_engine_config('create_plane', dict(locals()))
    if all_layers is None:
        all_layers = ['F.Cu', 'B.Cu']

    if len(net_names) != len(plane_layers):
        print(f"Error: Number of nets ({len(net_names)}) must match number of layers ({len(plane_layers)})")
        return _empty_plane_results(return_results)

    # Board-setup copper-to-edge rule (#338): zone outlines and tap copper must
    # honor the sibling .kicad_pro's min_copper_edge_clearance (engine-side so
    # the GUI planes tab inherits it; see batch_route).
    if input_file:
        try:
            from fix_kicad_drc_settings import effective_board_edge_clearance
            _eff_edge = effective_board_edge_clearance(input_file, board_edge_clearance)
            if _eff_edge > (board_edge_clearance or 0.0):
                print(f"Board edge clearance {_eff_edge}mm "
                      f"(project min_copper_edge_clearance)")
                board_edge_clearance = _eff_edge
        except Exception:
            pass

    # Step 1: Load PCB (or use provided pcb_data)
    # Whether the CALLER handed us live board state decides who is authoritative
    # about existing zones further down (see Step 2).
    _live_pcb_data = pcb_data is not None
    if pcb_data is None:
        print(f"Loading PCB from {input_file}...")
        pcb_data = parse_kicad_pcb(input_file)

    # Same canonicalisation as route.batch_route and
    # repair_planes: the two fronts hand this engine identical
    # copper in different ORDER, and list position leaks into decisions.
    from kicad_parser import canonicalize_pcb_data_order
    canonicalize_pcb_data_order(pcb_data)

    # --plane-layers takes BARE copper layer names positionally matched to
    # --nets, but the natural thing to type (and what the routing skill's R1
    # stage used to instruct) is a net:layer pair. The argument is untyped
    # `str`, so "GND:B.Cu" was accepted AS A LAYER NAME, travelled through
    # generate_zone_sexpr, and was written verbatim as (layer "GND:B.Cu").
    # KiCad then refuses the whole file ("Failed to load board") -- while every
    # in-repo checker read it happily, because check_connected's copper-layer
    # census only tested `endswith('.Cu')`. Two full routing laps of run 11
    # were spent on boards nobody could open.
    #
    # Engine-side, not in main(): main() never parses the board so it cannot
    # validate, and this way the CLI, the GUI planes tab and
    # route_disconnected_planes all inherit the guard (same reasoning as the
    # copper-to-edge rule below).
    _copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or ())
    if _copper:
        _bad = [l for l in plane_layers if l not in _copper]
        if _bad:
            print(f"Error: {', '.join(sorted(set(_bad)))} is not a copper layer "
                  f"of this board ({', '.join(_copper)}). --plane-layers takes "
                  f"BARE layer names positionally matched to --nets "
                  f"(e.g. --nets GND GNDA --plane-layers B.Cu B.Cu), "
                  f"not net:layer pairs.")
            return _empty_plane_results(return_results)

    # Route trace (#482): plane creation builds its tap tracks/vias as dicts in
    # all_new_segments/all_new_vias (never touching pcb_data.segments), so record
    # them from those dicts at the end. Local trace, baseline = the input copper.
    from route_trace import start_plane_trace as _start_plane_trace
    _ptrace = _start_plane_trace(pcb_data, output_file)

    # Zone clearance defaults to the step's ROUTED clearance, and escalates
    # DOWN toward the fab floor when even that cannot thread the densest BGA
    # via lattice the pour must serve (ottercast: the old fixed 0.2 default
    # vs 0.1 routing sealed the whole U1 field -- the pour needs
    # 2*zone_clearance + min_thickness of gap to pass between vias, and no
    # gap in a 0.65mm-pitch field is 0.5mm). Explicit values are honored
    # (but still warned about when they cannot thread).
    zone_clearance = _resolve_zone_clearance(
        zone_clearance, clearance, min_thickness, pcb_data, via_size,
        net_names)

    # Resolve all net IDs upfront
    net_ids = []
    for net_name in net_names:
        net_id = resolve_net_id(pcb_data, net_name)
        if net_id is None:
            print(f"Error: Net '{net_name}' not found in PCB")
            return _empty_plane_results(return_results)
        net_ids.append(net_id)
        print(f"Found net '{net_name}' with ID {net_id}")

    # Track failed pads per net for retry passes
    # Each entry is (net_id, net_name, plane_layer, pad_info)

    # Step 2: Check for existing zones on each target layer.
    #
    # When the caller supplied pcb_data it IS the board (the live pcbnew state
    # from the GUI), and it is the ONLY authority. This used to union the input
    # FILE's zones in as well, to catch a zone added live but not yet saved --
    # but a union can only ever ADD, so the opposite edit was invisible: delete a
    # zone by hand in KiCad, hit Create Plane, and the unsaved-to-disk file still
    # supplied the deleted zone, so skip_existing_zones kept "it already exists"
    # and no plane was ever poured. The live board already reports both cases
    # correctly (build_pcb_data_from_board lists the zone before deletion and
    # none after), so trust it alone. Only fall back to the file when no caller
    # supplied board state -- i.e. the CLI, where the file IS the board.
    existing_zones = []
    if _live_pcb_data:
        for z in (getattr(pcb_data, 'zones', None) or []):
            existing_zones.append(ZoneInfo(net_id=z.net_id, net_name=z.net_name, layer=z.layer,
                                           in_footprint=getattr(z, 'in_footprint', False),
                                           priority=int(getattr(z, 'priority', 0) or 0)))
    else:
        try:
            existing_zones = list(extract_zones(input_file))
        except (FileNotFoundError, OSError):
            existing_zones = []
        seen_keys = {(z.net_name, z.layer) for z in existing_zones}
        for z in (getattr(pcb_data, 'zones', None) or []):
            key = (z.net_name, z.layer)
            if key in seen_keys:
                continue
            existing_zones.append(ZoneInfo(net_id=z.net_id, net_name=z.net_name, layer=z.layer,
                                          in_footprint=getattr(z, 'in_footprint', False),
                                          priority=int(getattr(z, 'priority', 0) or 0)))
            seen_keys.add(key)

    should_create_zones = []  # Per-net flag for whether to create zone
    zones_to_replace = []  # List of (net_id, layer) tuples for zones to replace
    # Fill priority per net for a plane that has to SHARE its layer with another
    # net's existing pour: ABOVE the incumbent, so the newly requested plane wins
    # the overlap deterministically (see plane_io.shared_layer_zone_priority --
    # KiCad priorities are non-negative, so it cannot go below). Absent = 0.
    shared_layer_priority: Dict[str, int] = {}
    for i, (net_name, plane_layer, net_id) in enumerate(zip(net_names, plane_layers, net_ids)):
        if skip_existing_zones:
            # GUI path: never error on existing zones of other nets - in KiCad,
            # zones with different nets may coexist on a layer. Only check for a
            # same-net existing zone and skip in that case.
            same_net_zone = next((z for z in existing_zones
                                  if z.layer == plane_layer
                                  and (z.net_name == net_name or
                                       (z.net_id and z.net_id == net_id))),
                                 None)
            if same_net_zone:
                print(f"Note: zone for '{net_name}' already exists on {plane_layer} - "
                      f"keeping it (no new zone; thermal-array/--stitch-vias features "
                      f"still apply; pads are welded by the route step)")
                should_create_zones.append(False)
            else:
                should_create_zones.append(True)
            # Same shared-layer priority rule as the CLI path below, so both
            # fronts fill an already-occupied layer identically.
            _foreign = [z for z in existing_zones
                        if z.layer == plane_layer
                        and not getattr(z, 'in_footprint', False)
                        and z.net_name != net_name
                        and not (z.net_id and z.net_id == net_id)]
            if _foreign:
                shared_layer_priority[net_name] = shared_layer_zone_priority(_foreign)
            continue

        # CLI path. A foreign-net pour on the layer no longer aborts the run: the
        # plane is created alongside it, at a higher fill priority so the overlap
        # resolves deterministically (see check_existing_zones /
        # shared_layer_zone_priority).
        should_create, should_continue, zone_to_replace, foreign_zones = check_existing_zones(
            existing_zones, plane_layer, net_name, net_id, verbose
        )
        if not should_continue:
            print(f"Error: Zone conflict for net '{net_name}' on layer {plane_layer}")
            return _empty_plane_results(return_results)
        should_create_zones.append(should_create)
        if foreign_zones:
            shared_layer_priority[net_name] = shared_layer_zone_priority(foreign_zones)
        if zone_to_replace:
            zones_to_replace.append((zone_to_replace.net_id, zone_to_replace.layer))
            # #487: a REPLACED user pour keeps (at least) its own fill priority.
            # Writing only the shared-layer computed value silently reset e.g.
            # a priority-2 pour to 0, flipping which overlapping zone pulls back.
            _zp = int(getattr(zone_to_replace, 'priority', 0) or 0)
            if _zp > shared_layer_priority.get(net_name, 0):
                shared_layer_priority[net_name] = _zp

    # Step 3: Get board bounds for zone polygon
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds or (board_bounds[2] - board_bounds[0]) <= 0 or (board_bounds[3] - board_bounds[1]) <= 0:
        print("Error: Could not determine board bounds "
              "(no Edge.Cuts drawings found, or they have zero extent). "
              "Add an Edge.Cuts outline to the board before creating planes.")
        return _empty_plane_results(return_results)

    min_x, min_y, max_x, max_y = board_bounds
    print(f"Board bounds: ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})")

    # Create zone polygon from board bounds (with edge clearance applied)
    zone_polygon = [
        (min_x + board_edge_clearance, min_y + board_edge_clearance),
        (max_x - board_edge_clearance, min_y + board_edge_clearance),
        (max_x - board_edge_clearance, max_y - board_edge_clearance),
        (min_x + board_edge_clearance, max_y - board_edge_clearance)
    ]

    # Step 4: Build config and coordinate system
    # Set default layer costs if not specified
    # 4+ layers: all 1.0 (inner layers available for routing)
    # 2 layers: F.Cu=1.0, B.Cu=3.0 (prefer top layer)
    if not layer_costs:
        if len(all_layers) >= 4:
            layer_costs = [1.0] * len(all_layers)
        else:
            layer_costs = [1.0 if layer == 'F.Cu' else 3.0 for layer in all_layers]

    # Full-stack normalization (mirrors batch_route): append board copper
    # layers the caller did not request as FORBIDDEN (-1) so tap/strap via
    # placement always respects copper on every layer (a via spans the whole
    # stack; the CLI's default --layers F.Cu B.Cu on a 6/8-layer board must
    # not blind the maps to inner copper -- butterstick DQ11 class).
    _board_cu = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    _missing_cu = [l for l in _board_cu if l not in all_layers]
    if _missing_cu:
        from routing_constants import FORBIDDEN_LAYER_COST
        all_layers = list(all_layers) + _missing_cu
        layer_costs = list(layer_costs) + [FORBIDDEN_LAYER_COST] * len(_missing_cu)
        print(f"  Full-stack: appended {len(_missing_cu)} unrequested copper layer(s) "
              f"as FORBIDDEN obstacles: {', '.join(_missing_cu)}")

    # Validate layer costs: any negative = forbidden (no copper placed; still an
    # obstacle), otherwise a multiplier in [1.0, 1000].
    for i, cost in enumerate(layer_costs):
        if cost >= 0 and (cost < 1.0 or cost > 1000):
            layer_name = all_layers[i] if i < len(all_layers) else f"layer {i}"
            print(f"ERROR: Layer cost for {layer_name} must be negative (forbidden) or "
                  f"between 1.0 and 1000, got {cost}")
            return _empty_plane_results(return_results)

    costs_str = ', '.join(f"{all_layers[i]}={layer_costs[i]}x" for i in range(min(len(all_layers), len(layer_costs))))
    print(f"  Layer costs: {costs_str}")

    config = GridRouteConfig(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        hole_to_hole_clearance=hole_to_hole_clearance,
        board_edge_clearance=board_edge_clearance,
        layers=all_layers,
        layer_costs=layer_costs,
        ripup_blocker_select=ripup_blocker_select
    )
    # #581: an ACTIVE (> 0) same-net pad via clearance rides the config so
    # every via this step places (stitching already honored it; taps, joins,
    # blocker reroutes now do too) keeps off same-net pads; -1 and 0 keep the
    # pre-#581 behavior exactly. Resolution: an explicit flag value wins
    # (> 0 activates; 0 / -1 explicitly off); None (unset) auto-reads the
    # record an earlier chain step persisted into the sibling .kicad_pro.
    # Normalized to a concrete float here -- downstream stitching/tap call
    # sites compare it numerically.
    if same_net_pad_clearance is None:
        from protected_nets import read_snpc_for_pcb_data as _read_snpc581
        _snpc581 = _read_snpc581(pcb_data, input_file)
        same_net_pad_clearance = _snpc581 if _snpc581 > 0 else -1.0
        if _snpc581 > 0:
            print(f"  Same-net pad via clearance {_snpc581:g}mm (from project "
                  f"record, #581)")
    if same_net_pad_clearance > 0:
        config.same_net_pad_clearance = same_net_pad_clearance
    # #498: per-layer .kicad_dru clearance rules -- tap tracks/vias, region
    # joins and blocker reroutes must obey them like every other routed copper.
    from kicad_dru import install_layer_clearances
    install_layer_clearances(config, None, input_file, pcb_data)
    # Cross-class clearance (#434, mirrors batch_route/repair): auto-read the
    # board's non-Default netclasses from the INPUT's sibling .kicad_pro when
    # no map was passed, so tap tracks/vias and blocker reroutes honor KiCad's
    # pairwise max(classA, classB). All-Default boards -> empty map -> inert.
    if net_clearances is None and input_file and os.path.isfile(input_file):
        try:
            from list_nets import net_clearance_map_by_id
            net_clearances = net_clearance_map_by_id(
                input_file, {nid: n.name for nid, n in pcb_data.nets.items()})
            if net_clearances:
                print(f"  Auto-read netclass clearances for "
                      f"{len(net_clearances)} net(s) (cross-class max(A,B) "
                      f"respected for plane taps).")
        except Exception as _e:
            print(f"  Warning: could not auto-read netclass clearances ({_e}).")
            net_clearances = None
    # #439: --clearance was the CEILING -> cap every class at min(class, ceiling).
    # When not clamping (--clearance omitted), honor the classes in full.
    if net_clearances and clamp_netclasses and clearance_ceiling is not None:
        net_clearances = {nid: min(c, clearance_ceiling)
                          for nid, c in net_clearances.items()}
    if net_clearances:
        config.net_clearances = dict(net_clearances)
    # Publish the SAME map to the fill model (#483 item 5): KiCad refills a
    # zone at max(zone clearance, pairwise netclass), so on honor-classes
    # chains a looser foreign class carves copper the model would otherwise
    # predict as fill. Must precede every ZoneFillModel build on this board --
    # models are cached, and a stale flat-zc model outlives the map.
    from plane_fill_model import set_board_net_clearances
    set_board_net_clearances(pcb_data, net_clearances)
    coord = GridCoord(grid_step)

    # Create reusable router for via-to-pad routing
    via_pad_router = GridRouter(
        via_cost=config.via_cost_units(),
        h_weight=config.heuristic_weight,
        turn_cost=config.turn_cost,
        via_proximity_cost=0,
        layer_costs=config.get_layer_costs(),
        proximity_heuristic_cost=config.get_proximity_heuristic_cost()
    )

    # Accumulated results across all nets
    all_new_vias = []
    all_new_segments = []
    all_zone_sexprs = []
    all_zone_data = []  # Zone data dicts for pcbnew (when return_results=True)
    all_debug_lines = []  # Debug lines for inter-region routes (User.4)
    total_vias_placed = 0
    total_traces_added = 0
    total_failed_pads = 0
    total_pads_needing_vias = 0
    all_ripped_net_ids: List[int] = []

    # Collect ALL pads from ALL power nets that need vias (for cross-net protection)
    # This ensures when routing GND, we also protect +3.3V pad zones and vice versa
    all_power_pads_needing_vias: List[Dict] = []
    for net_id_tmp, plane_layer_tmp in zip(net_ids, plane_layers):
        target_pads_tmp = identify_target_pads(pcb_data, net_id_tmp, plane_layer_tmp)
        for p in target_pads_tmp:
            if p['needs_via']:
                p['_net_id'] = net_id_tmp  # Tag with net ID for filtering later
                all_power_pads_needing_vias.append(p)

    # Set to track pads that have been successfully processed (via placed)
    processed_pad_ids: Set[Tuple[float, float]] = set()  # (global_x, global_y) as key

    # Process each net/layer pair
    for net_idx, (net_name, plane_layer, net_id, should_create_zone) in enumerate(
            zip(net_names, plane_layers, net_ids, should_create_zones)):
        if cancel_check and cancel_check():
            print("\nPlane creation cancelled")
            break

        print(f"\n{'='*60}")
        print(f"Processing net '{net_name}' on layer {plane_layer}")
        print(f"{'='*60}")

        # (KICAD_PLANE_TAP_PREFER_REUSE prototype DELETED, review
        # dead-code 9: it wrote pcb_data._inrun_zones /
        # _pending_inrun_zone, whose only consumer -- the
        # perforation-aware tap-reuse trigger -- died with the tap
        # machinery (8c72da7). The experiment was recorded as REFUTED;
        # even with the env var set the block
        # was a pure no-op.)

        # Step 5: Identify target pads for this net
        if progress_callback:
            progress_callback(0, 0, f"{net_name}: analyzing pads on {plane_layer}...")
        target_pads = identify_target_pads(pcb_data, net_id, plane_layer)

        # THE PLANE STEP DOES NO ROUTING (#562). Every pad that would need
        # a tap via is deferred to the route step, which welds it into the
        # pour with the full routing machinery (pour-launch) and taps what
        # the fill cannot reach in its in-run plane finalize. Exposed/thermal
        # pads are the ONE exception -- their via ARRAY (#487) stamps vias
        # without drawing traces and nothing else in the chain places one.
        _therm = [pd for pd in target_pads
                  if pd['type'] == 'via_needed' and thermal_vias
                  and is_thermal_pad(pd['pad'], pcb_data)]
        _skip = [pd for pd in target_pads
                 if pd['type'] == 'via_needed' and pd not in _therm]
        target_pads = [pd for pd in target_pads
                       if pd['type'] != 'via_needed'] + _therm
        if _therm:
            print(f"  {len(_therm)} exposed/thermal pad(s) get a via ARRAY "
                  f"(#487)")
        if _skip:
            # Of the deferred pads, ONLY those under a BGA seed the
            # split-layer Voronoi: pours run before fanout (#562), so a
            # rail's cell must reserve the ball territory the fanout will
            # drop vias into -- there is no other way to guess it. Every
            # other deferred SMD pad (diode, cap, switch) gets its via from
            # the route step wherever the plane already is; seeding them
            # shattered split planes into per-pad confetti (dilemma:
            # 70 zones from 4 nets on 2 layers).
            _bga_refs = getattr(pcb_data, '_bga_ref_cache', None)
            if _bga_refs is None:
                from kicad_parser import find_components_by_type
                _bga_refs = pcb_data._bga_ref_cache = {
                    fp.reference
                    for fp in find_components_by_type(pcb_data, 'BGA')}
            _bga_skip = [pd for pd in _skip
                         if pd['pad'].component_ref in _bga_refs]
            if _bga_skip:
                _dseeds = getattr(pcb_data, '_deferred_bga_seeds', None)
                if _dseeds is None:
                    _dseeds = pcb_data._deferred_bga_seeds = {}
                _dseeds.setdefault(net_id, []).extend(
                    (pd['pad'].global_x, pd['pad'].global_y)
                    for pd in _bga_skip)
            print(f"  {len(_skip)} pad(s) on '{net_name}' deferred to the "
                  f"route step ({len(_bga_skip)} under-BGA position(s) kept "
                  f"as Voronoi seeds)")

        # (KICAD_PLANE_DEFER_BGA prototype DELETED, review dead-code 9:
        # the unconditional #562 deferral above already removes every
        # non-thermal via_needed pad, so this block could only defer
        # thermal-array pads inside BGA courtyards -- a behavior nobody
        # designed. _deferred_bga_seeds' reader at the Voronoi seeding
        # keeps working; nothing fills it now, which is correct: the
        # #562 deferral keeps the pads in target_pads as classified,
        # so their positions still seed the partition directly.)

        pads_through_hole = sum(1 for p in target_pads if p['type'] == 'through_hole')
        pads_direct = sum(1 for p in target_pads if p['type'] == 'direct')
        pads_already_connected = sum(1 for p in target_pads if p['type'] == 'already_connected')
        pads_need_via = sum(1 for p in target_pads if p['type'] == 'via_needed')
        pads_off_board = sum(1 for p in target_pads if p['type'] == 'off_board')
        total_pads_needing_vias += pads_need_via

        print(f"\nPad analysis for net '{net_name}':")
        print(f"  Through-hole pads (no via needed): {pads_through_hole}")
        print(f"  SMD pads on {plane_layer} (no via needed): {pads_direct}")
        if pads_already_connected:
            print(f"  SMD pads already routed to plane (no via needed): {pads_already_connected}")
        print(f"  SMD pads kept for thermal via arrays (via needed): {pads_need_via}")
        if pads_off_board:
            print(f"  {RED}Pads OUTSIDE the board outline (unreachable, skipped, #291): "
                  f"{pads_off_board}{RESET}")

        # Step 6: Collect existing vias on target net (for reuse)
        existing_net_vias: List[Tuple[float, float]] = []
        for via in pcb_data.vias:
            if via.net_id == net_id:
                existing_net_vias.append((via.x, via.y))

        if verbose and existing_net_vias:
            print(f"  Existing vias on net '{net_name}': {len(existing_net_vias)}")

        # Step 7: Build obstacle map for via placement (exclude current net)
        if pads_need_via > 0:
            print("\nBuilding obstacle map for via placement...")
            if progress_callback:
                progress_callback(0, 0, f"{net_name}: building via obstacle map...")
            obstacles = build_via_obstacle_map(pcb_data, config, net_id,
                                               same_net_pad_clearance=same_net_pad_clearance)
            # Also block positions of vias we've already placed in previous nets.
            # from_restore entries are EXCLUDED: a restored rip victim's vias
            # are already back in pcb_data, so build_via_obstacle_map has
            # priced them at their float centre -- stamping them again here at
            # session-via pricing (grid-snapped centre, <= drill ring) is a
            # conservative over-block, and it is what made the balance audit's
            # fresh rebuild diverge from the correctly-maintained map (the
            # same filter the pcb_data sync below already applies).
            for placed_via in all_new_vias:
                if placed_via.get('from_restore'):
                    continue
                block_via_position(obstacles, placed_via['x'], placed_via['y'], coord,
                                   hole_to_hole_clearance, via_drill,
                                   via_size, config.clearance)
        else:
            obstacles = None

        # Step 8: Build routing obstacle maps (cached per layer, but rebuild for each net)
        routing_obstacles_cache: Dict[str, GridObstacleMap] = {}
        if verbose:
            print(f"  pcb_data has {len(pcb_data.vias)} vias, {len(pcb_data.segments)} segments")

        def get_routing_obstacles(layer: str) -> GridObstacleMap:
            """Get or create routing obstacle map for a layer."""
            if layer not in routing_obstacles_cache:
                if verbose:
                    print(f"\n  Building routing obstacle map for {layer}...")
                routing_obstacles_cache[layer] = build_routing_obstacle_map(
                    pcb_data, config, net_id, layer, skip_pad_blocking=False, verbose=False
                )
            return routing_obstacles_cache[layer]

        # Fill-aware via preference: predict this net's zone fill (the exact
        # polygon create_plane will write, over the copper that exists NOW)
        # and prefer via sites on its MAIN component. Distance-only placement
        # happily drops a stitching via in a clearance-carved pocket or on a
        # fill island -- DRC-clean, self-reports success, and becomes a
        # region-join the plane REPAIR step must strap later. Single-net
        # layers only: a Voronoi-shared layer's cells are seeded FROM the
        # placed vias, so there is no polygon to predict yet (the repair
        # path's oracle + outline filter covers those, #287). Soft preference,
        # never a gate -- a mispredicted fill degrades to today's behavior.
        fill_via_preference = None
        _is_multi_net_layer = bool(layer_nets
                                   and len(layer_nets.get(plane_layer, [])) > 1)
        if should_create_zone and not _is_multi_net_layer:
            try:
                from types import SimpleNamespace
                from plane_fill_model import ZoneFillModel
                _synth_zone = SimpleNamespace(
                    net_id=net_id, layer=plane_layer,
                    clearance=zone_clearance, min_thickness=min_thickness,
                    polygon=zone_polygon)
                _pred_model = ZoneFillModel(pcb_data, _synth_zone)
                if _pred_model.ok:
                    _pred_main = _pred_model.largest_component()
                    if _pred_main:
                        def fill_via_preference(x, y, _m=_pred_model,
                                                _c=_pred_main,
                                                _s=via_size / 2):
                            return _m.query_component(x, y, size=_s) == _c
                        print(f"  Fill-aware via preference: predicted "
                              f"{plane_layer} fill modeled (main component "
                              f"located)")
            except Exception as _fp_e:
                fill_via_preference = None
                if verbose:
                    print(f"  (fill prediction unavailable: {_fp_e})")

        # Step 9: Place vias near each target pad (or reuse existing)
        new_vias = []
        new_segments = []
        vias_placed = 0
        traces_added = 0
        failed_pads = 0
        ripped_net_ids: List[int] = []  # Nets ripped for this net

        # (available_vias / ViaSpatialIndex / via_obstacle_cache
        # scaffolding DELETED, review dead-code 3: all were add-only
        # since the tap machinery went -- nearest-via reuse was a tap
        # feature, and the rip-up obstacle cache fed the deleted
        # rip loop.)

        # Build list of pads needing vias for this net
        pads_needing_vias = [p for p in target_pads if p['needs_via']]

        # Draw all pad exclusion zones on User.9 once at the start of FIRST net (for debugging)
        if debug_lines and net_idx == 0 and all_power_pads_needing_vias:
            margin = 1.5 * via_size + clearance  # via_size/2 for placed via + via_size/2 for future via + via_size/2 extra + clearance
            for pp_info in all_power_pads_needing_vias:
                pp = pp_info['pad']
                half_w = pp.size_x / 2 + margin
                half_h = pp.size_y / 2 + margin
                x1, y1 = pp.global_x - half_w, pp.global_y - half_h
                x2, y2 = pp.global_x + half_w, pp.global_y + half_h
                all_debug_lines.append(generate_gr_line_sexpr((x1, y1), (x2, y1), 0.05, "User.9"))
                all_debug_lines.append(generate_gr_line_sexpr((x2, y1), (x2, y2), 0.05, "User.9"))
                all_debug_lines.append(generate_gr_line_sexpr((x2, y2), (x1, y2), 0.05, "User.9"))
                all_debug_lines.append(generate_gr_line_sexpr((x1, y2), (x1, y1), 0.05, "User.9"))

        # (base_pending_pads / ripped_pending_pads scaffolding
        # DELETED, review dead-code 3: built per net at O(other
        # nets' pads) and never read since the tap loop went.)

        if pads_needing_vias:
            print(f"\nPlacing thermal via arrays under {len(pads_needing_vias)} exposed pad(s) on {plane_layer}:")

        # THERMAL VIA ARRAYS ONLY (#487). The plane step does no routing:
        # it places no tap vias and draws no traces, so every other pad that
        # would need one is deferred above and welded by the route step's
        # pour-launch / in-run plane finalize. An exposed pad's via ARRAY is
        # a pour/fab feature (it stamps vias, draws nothing) and nothing else
        # in the chain places one, so it stays here.
        for pad_idx, pad_info in enumerate(pads_needing_vias):
            if cancel_check and cancel_check():
                print("  (cancelled)")
                break
            pad = pad_info['pad']
            current_pad_key = (pad.global_x, pad.global_y)
            if current_pad_key in processed_pad_ids:
                continue
            if progress_callback:
                progress_callback(pad_idx + 1, len(pads_needing_vias),
                                  f"{net_name}: thermal array for "
                                  f"{pad.component_ref}.{pad.pad_number}")
            print(f"  Pad {pad.component_ref}.{pad.pad_number}...", end=" ")
            _arr = compute_thermal_via_array(
                pad, obstacles, coord, config, via_size, via_drill,
                hole_to_hole_clearance, pcb_data)
            _placed = 0
            for (_ax, _ay) in _arr:
                _agx, _agy = coord.to_grid(_ax, _ay)
                if obstacles.is_via_blocked(_agx, _agy):
                    continue      # a just-placed array via blocks this cell
                new_vias.append({'x': _ax, 'y': _ay, 'size': via_size,
                                 'drill': via_drill,
                                 'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id})
                block_via_position(obstacles, _ax, _ay, coord,
                                   hole_to_hole_clearance, via_drill,
                                   via_size, config.clearance)
                _placed += 1
            processed_pad_ids.add(current_pad_key)
            if _placed:
                vias_placed += _placed
                print(f"thermal via array: {_placed} via(s) over "
                      f"{pad.size_x:.1f}x{pad.size_y:.1f}mm pad")
            else:
                # No lattice site fits -- defer like any other plane pad.
                # NEVER fall back to a single via + trace: that is routing.
                # Seed the Voronoi only for a BGA's pad (ball-territory
                # reservation); a QFN/exposed paddle takes its tap from the
                # route step wherever the plane lands.
                _bga_refs = getattr(pcb_data, '_bga_ref_cache', None)
                if _bga_refs is None:
                    from kicad_parser import find_components_by_type
                    _bga_refs = pcb_data._bga_ref_cache = {
                        fp.reference
                        for fp in find_components_by_type(pcb_data, 'BGA')}
                if pad.component_ref in _bga_refs:
                    _ds = getattr(pcb_data, '_deferred_bga_seeds', None)
                    if _ds is None:
                        _ds = pcb_data._deferred_bga_seeds = {}
                    _ds.setdefault(net_id, []).append(
                        (pad.global_x, pad.global_y))
                print("thermal array did not fit -- deferred to the route step")

        # (The fine-pitch retry pass, issue #104, lived here. It is gone with
        # the tap machinery: the loop above places thermal-via ARRAYS only and
        # never records a failure, so failed_pad_infos stayed empty for this
        # net and the retry could not fire. A pad the array cannot serve is
        # deferred to the route step like every other plane pad -- #562.)

        # Ref-count integrity audit of this net's via-placement map (#309, same
        # class as #208): after all rips/placements the maintained map must
        # equal a fresh rebuild from the CURRENT pcb_data plus the session vias
        # (mirroring its Step-7 construction + per-placement blocking).
        # from_restore vias are filtered for the same reason as at Step 7:
        # they are already in pcb_data, so the fresh rebuild prices them at
        # their float centre -- stamping them AGAIN at snapped-centre session
        # pricing contaminated the audit's reference and flagged the correct
        # maintained map as "under-blocked" (a single boundary cell, false
        # positive). Both filters are required together: filtering only the
        # audit flips the report to the mirror-image "wrongly-blocked" leak.
        if env_knobs.OBSTACLE_AUDIT and obstacles is not None:
            _audit_plane_via_map(obstacles, pcb_data, config, net_id,
                                 same_net_pad_clearance,
                                 [v for v in all_new_vias + new_vias
                                  if not v.get('from_restore')], coord,
                                 hole_to_hole_clearance, via_drill, via_size,
                                 net_name)

        # Step 10: Generate zone for this net (if needed)
        # For multi-net layers, defer zone generation until all nets are processed
        zone_sexpr = None
        is_multi_net_layer = layer_nets and len(layer_nets.get(plane_layer, [])) > 1
        if should_create_zone and not is_multi_net_layer:
            # Single-net layer: use full board rectangle. When another net's pour
            # already shares this layer, fill ABOVE it so the overlap resolves to
            # this plane deterministically (0 otherwise, i.e. unchanged).
            _prio = shared_layer_priority.get(net_name, 0)
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=plane_layer,
                polygon_points=zone_polygon,
                clearance=zone_clearance,
                min_thickness=min_thickness,
                direct_connect=not thermal_relief,
                use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION,
                priority=_prio
            )
            all_zone_sexprs.append(zone_sexpr)

            # Calculate and print resistance for single-net layer, at the copper
            # weight this board's stackup actually specifies (#489 §6).
            result = analyze_single_net_plane(
                zone_polygon, plane_layer,
                copper_oz=stackup_copper_oz(pcb_data, plane_layer))
            print_single_net_resistance(result, net_name)
            # #487: main() folds these into JSON_SUMMARY (stdout-only before).
            note_resistance_result(net_name, result)

            all_zone_data.append({
                'thermal_relief': thermal_relief,
                'net_id': net_id,
                'net_name': net_name,
                'layer': plane_layer,
                'polygon_points': zone_polygon,
                'clearance': zone_clearance,
                'min_thickness': min_thickness,
                'resistance_analysis': result,
                # The GUI builds its pcbnew ZONE from this dict, so the shared-layer
                # priority has to travel with it or the two fronts fill differently.
                'priority': _prio,
            })

        # Print per-net results
        print(f"\nResults for '{net_name}':")
        was_replaced = (net_id, plane_layer) in zones_to_replace
        if should_create_zone and is_multi_net_layer:
            suffix = " (replaced existing)" if was_replaced else ""
            print(f"  Zone on {plane_layer} deferred (multi-net layer){suffix}")
        elif should_create_zone:
            suffix = " (replaced existing)" if was_replaced else ""
            print(f"  Zone created on {plane_layer}{suffix}")
        print(f"  New vias placed: {vias_placed}")
        print(f"  Traces added: {traces_added}")
        if failed_pads > 0:
            print(f"  Failed pads: {failed_pads}")

        # KICAD_PLANE_MAP_PARITY=1: verify the incremental via map still
        # BLOCKS every restored net's live copper (under-blocking here is the
        # via-drilled-on-restored-track class; over-blocking is by-design
        # conservative for dropped pieces). Fresh-recompute each restored
        # net's footprint from CURRENT pcb_data and probe the map.
        if env_knobs.PLANE_MAP_PARITY and ripped_net_ids:
            from obstacle_cache import precompute_via_placement_obstacles as _pv
            for _rid in ripped_net_ids:
                if not any(v.net_id == _rid for v in pcb_data.vias) and \
                   not any(sg.net_id == _rid for sg in pcb_data.segments):
                    continue  # fully ripped, nothing live to verify
                _fresh = _pv(pcb_data, _rid, config, all_layers)
                _miss = sum(1 for (gx, gy) in map(tuple, _fresh.blocked_vias)
                            if not obstacles.is_via_blocked(int(gx), int(gy)))
                _nm = pcb_data.nets[_rid].name if _rid in pcb_data.nets else _rid
                status = 'OK' if _miss == 0 else f'UNDER-BLOCKED {_miss} cells'
                print(f"  MAP-PARITY {_nm}: {len(_fresh.blocked_vias)} live cells, {status}")

        # Accumulate results
        all_new_vias.extend(new_vias)
        all_new_segments.extend(new_segments)
        total_vias_placed += vias_placed
        total_traces_added += traces_added
        total_failed_pads += failed_pads
        for rid in ripped_net_ids:
            if rid not in all_ripped_net_ids:
                all_ripped_net_ids.append(rid)

        # Add new vias/segments to pcb_data so subsequent nets will avoid them.
        # Restored-net emissions are skipped: their copper OBJECTS are already
        # in pcb_data (settle's restore) -- re-adding doubles the net's copper.
        for v in new_vias:
            if v.get('from_restore'):
                continue
            pcb_data.vias.append(Via(
                x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
                layers=v['layers'], net_id=v['net_id']
            ))
        for s in new_segments:
            if s.get('from_restore'):
                continue
            start = s['start']
            end = s['end']
            pcb_data.segments.append(Segment(
                start_x=start[0], start_y=start[1],
                end_x=end[0], end_y=end[1],
                width=s['width'], layer=s['layer'], net_id=s['net_id']
            ))

    # End of per-net loop

    # Generate zones for multi-net layers using Voronoi boundaries
    if layer_nets:
        for layer, nets_on_layer in layer_nets.items():
            if len(nets_on_layer) > 1:
                print(f"\n{'='*60}")
                print(f"Computing zone boundaries for multi-net layer {layer}")
                print(f"Nets: {', '.join(nets_on_layer)}")
                print(f"{'='*60}")
                if progress_callback:
                    progress_callback(0, 0, f"Computing Voronoi zones for {layer}...")

                zone_sexprs, debug_line_sexprs, zone_data = _generate_multinet_layer_zones(
                    thermal_relief=thermal_relief,
                    layer=layer,
                    nets_on_layer=nets_on_layer,
                    pcb_data=pcb_data,
                    all_new_vias=all_new_vias,
                    zone_polygon=zone_polygon,
                    board_bounds=board_bounds,
                    config=config,
                    zone_clearance=zone_clearance,
                    min_thickness=min_thickness,
                    plane_proximity_radius=plane_proximity_radius,
                    plane_proximity_cost=plane_proximity_cost,
                    plane_track_via_clearance=plane_track_via_clearance,
                    plane_max_iterations=plane_max_iterations,
                    voronoi_seed_interval=voronoi_seed_interval,
                    board_edge_clearance=board_edge_clearance,
                    debug_lines=debug_lines,
                    verbose=verbose,
                    # Sit above any FOREIGN pour already on this layer, exactly
                    # like the single-net path. Without it, our Voronoi zones and
                    # the incumbent would both be priority 0 and KiCad would
                    # tie-break the fill on UUIDs.
                    priority_offset=max(
                        (shared_layer_priority.get(n, 0) for n in nets_on_layer),
                        default=0)
                )
                all_zone_sexprs.extend(zone_sexprs)
                all_debug_lines.extend(debug_line_sexprs)
                all_zone_data.extend(zone_data)

    # Area via stitching + edge fence (#485): AFTER every pour's geometry and
    # this run's tap copper exist, BEFORE the finalize/write split so the CLI
    # file write and the GUI results path emit identical stitch vias. The
    # pass appends its vias to pcb_data.vias itself.
    if (stitch_vias or stitch_edge_fence) and \
            not (cancel_check and cancel_check()):
        # Frequency-derived pitch: lambda/20 at the maximum frequency of
        # interest, from the stackup's own dielectric. Overrides
        # --stitch-pitch (the more specific intent wins); the fence pitch
        # follows unless --stitch-fence-pitch is explicit.
        if stitch_max_freq:
            _p, _er, _from_stackup, _lam = _stitch_pitch_from_freq(
                pcb_data, stitch_max_freq)
            _src = "board stackup" if _from_stackup else "FR-4 default"
            print(f"\nStitch pitch from max frequency {stitch_max_freq:g} "
                  f"MHz: lambda = {_lam:.1f}mm (epsilon_r {_er:g}, {_src}) "
                  f"-> lambda/20 pitch {_p:.2f}mm"
                  + (f" (overrides pitch {stitch_pitch:g}mm)"
                     if abs(_p - stitch_pitch) > 1e-9 else ""))
            stitch_pitch = _p
        stitch_via_dicts = _stitch_plane_area_vias(
            pcb_data, net_names, plane_layers, net_ids, all_zone_data,
            config, coord, stitch_pitch, via_size, via_drill,
            hole_to_hole_clearance, same_net_pad_clearance,
            stitch_lattice=stitch_vias,
            stitch_edge_fence=stitch_edge_fence,
            stitch_fence_pitch=stitch_fence_pitch,
            stitch_inset=stitch_inset,
            board_edge_clearance=board_edge_clearance,
            progress_callback=progress_callback, cancel_check=cancel_check,
            verbose=verbose,
            # Exact-fill site validation (#485): the temp refill board =
            # input copper minus this run's rips, plus this run's zones and
            # tap copper -- everything but the stitch vias themselves.
            input_file=input_file,
            zone_sexprs=all_zone_sexprs,
            tap_vias=all_new_vias,
            tap_segments=all_new_segments,
            exclude_net_ids=all_ripped_net_ids)
        all_new_vias.extend(stitch_via_dicts)
        total_vias_placed += len(stitch_via_dicts)

    # Print overall totals only if multiple nets were processed
    if len(net_names) > 1:
        print(f"\n{'='*60}")
        print(f"OVERALL TOTALS")
        print(f"{'='*60}")
        print(f"  Nets processed: {len(net_names)}")
        print(f"  Total new vias placed: {total_vias_placed}")
        print(f"  Total traces added: {total_traces_added}")
        _LAST_PAD_TALLY['failed_pads'] = int(total_failed_pads)
        _LAST_PAD_TALLY['pads_needing_vias'] = int(total_pads_needing_vias)
        if total_failed_pads > 0:
            print(f"  Total failed pads: {total_failed_pads}")

        if all_ripped_net_ids:
            ripped_names = []
            for rid in all_ripped_net_ids:
                net = pcb_data.nets.get(rid)
                ripped_names.append(net.name if net else f"net_{rid}")
            print(f"  Nets excluded from output: {', '.join(ripped_names)}")

    # Via-in-pad is a FAB requirement this run may just have created (#489 §8).
    # Emitted from the shared engine so the GUI planes tab reports it too.
    from fab_notes import print_via_in_pad_note
    print_via_in_pad_note(all_new_vias, pcb_data.pads_by_net,
                          context="plane stitching vias")

    # Finalize plane tap copper ONCE, before the write/dry-run split, so the
    # GUI (dry_run=True, return_results) and the CLI (writes the file) emit
    # identical copper for identical inputs: neck grazes -> graze prune /
    # dead-end sweep -> close soft joints (#334 + follow-up). Previously this
    # lived inside _write_plane_output (then _write_output_and_reroute) and the GUI path never ran it.
    if progress_callback:
        progress_callback(0, 0, "Cleaning up plane tap copper...")
    # #508 finding 2: input-board copper the finalize passes delete from
    # pcb_data; must reach the CLI writer's strip channel / GUI applier.
    _finalize_strips: list = []
    all_new_segments = _finalize_plane_copper(
        all_new_segments, all_new_vias, pcb_data, clearance, all_layers,
        track_width, grid_step, via_size, via_drill, hole_to_hole_clearance,
        net_clearances=net_clearances, strip_sink=_finalize_strips,
        same_net_pad_clearance=same_net_pad_clearance)  # #581

    # Route trace (#482): emit the finalized plane-tap tracks/vias, grouped by
    # net so each plane's taps land as one animation event, then write
    # <output>_routetrace.json. finalize=False: the tap copper is in these dicts,
    # not pcb_data, so the whole-run animator trues up to the step board instead.
    if _ptrace is not None:
        _by_net: Dict[int, list] = {}
        for _s in all_new_segments:
            _by_net.setdefault(_s.get('net_id'), [[], []])[0].append(_s)
        for _v in all_new_vias:
            _by_net.setdefault(_v.get('net_id'), [[], []])[1].append(_v)
        for _nid, (_segs, _vias) in _by_net.items():
            _nm = pcb_data.nets[_nid].name if _nid in pcb_data.nets else ''
            _ptrace.record_dicts(_segs, _vias, 'plane-tap', _nid, _nm)
        _ptrace.dump(output_file, pcb_data, finalize=False)

    # NPTH slot edge keepouts (#448): KiCad's DRC grades copper proximity to a
    # milled NPTH SLOT as copper_edge_clearance, but its zone filler pulls fill
    # back from the slot at only the hole clearance -- so any plane zone poured
    # over a slotted footprint (keyboard switches: sofle_pico SW25) self-flags
    # against the board's edge rule on refill. Emit a copper_pour-only keepout
    # rule area around each slot, dilated to the board's effective edge
    # clearance, whenever this run creates/replaces a zone. Shared here (before
    # the dry_run/write split) so the CLI file write and the GUI results path
    # emit identically.
    if all_zone_sexprs or all_zone_data:
        try:
            from kicad_writer import (npth_slot_keepout_polygons,
                                      generate_keepout_zone_sexpr)
            from fix_kicad_drc_settings import fab_edge_floor
            try:
                from fix_kicad_drc_settings import read_project_edge_clearance
                _rec_edge = read_project_edge_clearance(input_file) or 0.0
            except Exception:
                _rec_edge = 0.0
            _slot_edge = max(board_edge_clearance, _rec_edge,
                             fab_edge_floor(input_file))
            _slot_polys = npth_slot_keepout_polygons(pcb_data, _slot_edge)
            if _slot_polys:
                try:
                    with open(input_file, 'r', encoding='utf-8') as _f:
                        _in_text = _f.read()
                except Exception:
                    _in_text = ''
                _added = 0
                _cu = list(pcb_data.board_info.copper_layers or all_layers)
                for _ref, _pts in _slot_polys:
                    _kname = f"npth-slot-keepout-{_ref}"
                    if _kname in _in_text:
                        continue  # already present from an earlier plane run
                    all_zone_sexprs.append(generate_keepout_zone_sexpr(
                        _cu, _pts, _kname,
                        use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION))
                    all_zone_data.append({
                        'thermal_relief': thermal_relief,
                        'keepout': True, 'name': _kname, 'layers': _cu,
                        'polygon_points': _pts,
                    })
                    _added += 1
                if _added:
                    print(f"  Added {_added} NPTH-slot copper_pour keepout(s) "
                          f"at {_slot_edge:.3f}mm edge clearance (#448)")
        except Exception as _e:  # noqa: BLE001 -- keepouts are additive; never fail the run
            print(f"  (NPTH-slot keepouts skipped: {_e})")

    geo_results: Dict[int, Dict] = {}
    if dry_run:
        print("\nDry run - no output file written")
    else:
        if env_knobs.SETTLE_DEBUG:
            from collections import Counter as _C
            _dups = {k: n for k, n in _C((v.get('net_id'), round(v['x'], 3), round(v['y'], 3))
                                          for v in all_new_vias).items() if n > 1}
            print(f"\n  WRITE-DEBUG: duplicate new_vias: {_dups}")
        _write_plane_output(
            input_file=input_file,
            output_file=output_file,
            all_zone_sexprs=all_zone_sexprs,
            all_debug_lines=all_debug_lines,
            all_new_vias=all_new_vias,
            all_new_segments=all_new_segments,
            all_ripped_net_ids=all_ripped_net_ids,
            zones_to_replace=zones_to_replace,
            pcb_data=pcb_data,
            add_teardrops=add_teardrops,
            removed_input_segments=_finalize_strips
        )

        # Geometric truth check (issues #89 and #107): the via-placement
        # counters above count a pad "done" when a via is placed/reused, but
        # that via may not be electrically joined to the net's plane (#89), and
        # multi-net Voronoi layers can silently skip TH pads that land in the
        # other net's cell (#107). Re-parse the written output and report how
        # many pads are actually connected so the summary matches geometry.
        geo_results = _geometric_plane_verification(
            output_file, net_ids, net_names, plane_layers)
        if geo_results:
            geo_failed = sum(info['failed'] for info in geo_results.values())
            _LAST_PAD_TALLY['geometric_failed'] = int(geo_failed)
            if geo_failed != total_failed_pads:
                print(f"\n  {geo_failed} pad(s) are not connected to their "
                      f"plane by the pour alone -- EXPECTED (#562): the plane "
                      f"step places no taps. They are welded during the route "
                      f"step's normal routing (pour-launch); its in-run "
                      f"finalize then verifies against KiCad's exact fill and "
                      f"repairs any residue. Grade the board AFTER the route "
                      f"step, not here (see per-net breakdown above).")

    if progress_callback:
        progress_callback(1, 1, "Plane creation complete")
    reconnect_swap_data: dict = {}
    # Input-board copper withdrawn from pcb_data by the finalize passes
    # (#508 finding 2) or the in-memory reconnect's cleanup (#508 finding 1):
    # the GUI applier must delete these individually -- the whole-net delete
    # only covers ripped nets.
    reconnect_strips: list = list(_finalize_strips)
    # #581 GUI leg: the CLI main persists an active same-net pad via clearance
    # into the OUTPUT's project after fix_project_for_output; the GUI has no
    # output file, so record it against the live board's own project here.
    if return_results and same_net_pad_clearance is not None \
            and same_net_pad_clearance > 0:
        try:
            from protected_nets import (persist_same_net_pad_clearance,
                                        pro_path_for_board)
            _src581 = getattr(pcb_data, 'source_path', "") or ""
            if _src581:
                persist_same_net_pad_clearance(
                    pro_path_for_board(_src581), same_net_pad_clearance)
        except Exception as _e581:
            print(f"  (skipped same-net pad clearance record: {_e581})")
    if return_results:
        # GUI parity with the CLI's in-run ripped-net reconnect (#347): the
        # file path above reroutes verified-broken casualties after writing;
        # the GUI (dry_run) path reconnects them here, in memory, exactly
        # like repair_planes' return_results block. pcb_data has
        # the ripped copper removed and this run's copper appended, so
        # batch_route routes against the live in-memory board; the new copper
        # is merged into the emit lists (the applier deletes the ripped nets'
        # originals first, then applies these).
        # GUI-SIDE RIPPED-NET RECONNECT DELETED (review dead-code 1).
        # This was the return_results mirror of the ~191-line CLI
        # reroute block c126987 removed, unreachable by the same proof:
        # the pour cannot rip since #562, so all_ripped_net_ids is
        # contractually empty. Same tripwire as the CLI side -- if a
        # future code path fills the list again, say so loudly instead
        # of silently resurrecting nothing.
        if all_ripped_net_ids:
            print(f"WARNING: pour reported {len(all_ripped_net_ids)} "
                  f"ripped net(s), but the pour cannot rip since #562 "
                  f"-- a code path is filling ripped_net_ids again; "
                  f"restore the reconnect machinery (deleted "
                  f"2026-08-04) or fix the filler.")
        # #508 finding 1, second mechanism (the #463 class itself): a partial
        # restore's kept-set was emitted (from_restore dicts) BEFORE the
        # reconnect ran, and the reconnect may have re-routed that same net,
        # deleting the kept copper from pcb_data -- the write list must not
        # still carry it (spartan6_6layer shipped a collinear different-net
        # overlap this way in the sibling engine). pcb_data is authoritative
        # once the reconnect and its restore-on-failure custody have run.
        from plane_write_reconcile import drop_withdrawn_partial_restores
        _rest_s = [d for d in all_new_segments if d.get('from_restore')]
        _rest_v = [d for d in all_new_vias if d.get('from_restore')]
        _n_s, _n_v, _names = drop_withdrawn_partial_restores(
            _rest_s, _rest_v, all_new_segments, all_new_vias, pcb_data)
        if _n_s or _n_v:
            print(f"  dropped {_n_s} stale partial-restore segment(s) and "
                  f"{_n_v} via(s) the reconnect withdrew: {', '.join(_names)}")
        # all_ripped_net_ids after the emit lists for backward compatibility:
        # the GUI must delete these nets' existing board copper before applying
        # new_vias/new_segments (which include the from_restore replacement
        # pieces) -- the CLI writer's exclude_net_ids strip has no pcbnew
        # equivalent, so without this the live board keeps the originals AND
        # gains the emitted copies (duplicated restored copper).
        # reconnect_strips LAST (#508): input copper the reconnect's cleanup
        # withdrew; the applier deletes these individually.
        return (total_vias_placed, total_traces_added, total_pads_needing_vias,
                all_new_vias, all_new_segments, all_zone_data, total_failed_pads,
                all_ripped_net_ids, reconnect_swap_data, reconnect_strips)
    return (total_vias_placed, total_traces_added, total_pads_needing_vias)


def main():
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set
    parser = argparse.ArgumentParser(
        description="Create copper pour zones (#562 bare pour: no per-pad taps -- "
                    "pads are welded by the route step). Optional thermal via arrays, "
                    "area via stitching, and board-edge via fence.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Single net:
    python py_router/route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu

    # Multiple nets (each net paired with corresponding plane layer):
    python py_router/route_planes.py input.kicad_pcb output.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu

"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output KiCad PCB file (default: input_routed.kicad_pcb)")
    parser.add_argument("--ripup-blocker-select",
                        choices=list(defaults.RIPUP_BLOCKER_SELECT_CHOICES),
                        default=defaults.RIPUP_BLOCKER_SELECT,
                        help="""Blocker SELECTION algorithm for the rip-up ladder (see route.py --help / docs/rip-up-reroute.md)""")
    # #381 D9: accept --output FILE like route.py / route_diff.py (flag form of the
    # positional output). Additive: default None, positional still works.
    parser.add_argument("--output", metavar="FILE",
                        help="Output KiCad PCB file (flag alternative to the positional output)")
    parser.add_argument("--overwrite", "-O", action="store_true",
                        help="Overwrite input file instead of creating _routed copy")

    # Required options (can be multiple)
    parser.add_argument("--nets", "-n", nargs="+", required=True,
                        help="Net name(s) for the plane(s) (e.g., GND VCC)")
    parser.add_argument("--plane-layers", "-p", nargs="+", required=True,
                        help="BARE copper layer name(s) for the zone(s), one per net, "
                             "positionally matched to --nets (e.g., In1.Cu In2.Cu). "
                             "NOT net:layer pairs -- 'GND:B.Cu' is not a layer name and "
                             "is refused against the board's own copper layers.")

    # Via and track geometry
    parser.add_argument("--via-size", type=float, default=None, help="Via outer diameter in mm (default: the board Default net-class via, else 0.5). NOT a hard floor: a tap into a fine-pitch pad field can escalate to the --fab-tier via and print a per-via warning, because a 0.5mm via does not fit a 0.5mm-pitch TQFP. Pass --fab-overrides to forbid the escalation -- at the cost of the taps that then cannot be made")
    parser.add_argument("--via-drill", type=float, default=None, help="Via drill size in mm (default: the board Default net-class via drill, else 0.3)")
    parser.add_argument("--track-width", type=float, default=None, help="Track width for via-to-pad connections in mm (default: the board Default net-class width, else 0.3)")
    parser.add_argument("--clearance", type=float, default=None, help="Copper clearance CEILING in mm. When given, every net class (Default included) is capped at min(class, this) and the writeback clamps. When OMITTED, each net routes at its own net-class clearance (base = the board's Default class, else 0.25).")

    # Zone options
    parser.add_argument("--zone-clearance", type=float, default=None, help="Zone (pour) clearance from other copper in mm. Default: follow --clearance, auto-stepping down to the fab floor if the pour cannot thread the densest BGA via lattice")
    parser.add_argument("--min-thickness", type=float, default=defaults.PLANE_MIN_THICKNESS, help="Minimum zone copper thickness in mm (default: 0.1)")
    parser.add_argument("--thermal-vias", action=argparse.BooleanOptionalAction,
                        default=defaults.THERMAL_VIAS,
                        help="Via ARRAY over exposed/thermal pads (SMD plane-net pads wider than "
                             f"{defaults.THERMAL_PAD_MIN_MM}mm both axes) instead of one shared "
                             "via (#487). ON by default; --no-thermal-vias disables")
    parser.add_argument("--thermal-relief", action="store_true",
                        help="Connect pads to the pour with thermal-relief spokes instead of "
                             "solid copper (#487: the writer always supported it; nothing could ask)")

    # Algorithm options
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP, help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=None, help=f"Minimum clearance between drill holes in mm (default: the board min_hole_to_hole, else {defaults.HOLE_TO_HOLE_CLEARANCE})")
    parser.add_argument("--layers", "-l", nargs="+", default=None,
                        help="All copper layers for routing and via span (default: F.Cu + plane-layers + B.Cu)")
    parser.add_argument("--layer-costs", nargs="+", type=float, default=[],
                        help="Per-layer routing cost multipliers (1.0-1000, or any negative value e.g. -1 = "
                             "forbidden: the layer stays an obstacle / via span but gets no routed copper). "
                             "Order matches --layers. Example: --layer-costs 1.0 -1 -1 3.0")

    # #381 D9: accept the plural --no-bga-zones spelling too (route.py uses the
    # plural). Same store_true dest -- additive, old spelling kept.
    parser.add_argument("--no-bga-zone", "--no-bga-zones", action="store_true",
                        help="Disable BGA auto-exclusion zones when re-routing ripped nets "
                             "(issue #88.2). Use when the original signal routing used "
                             "--no-bga-zones, so the reroute uses compatible parameters.")
    parser.add_argument("--power-nets", nargs="+", default=None,
                        help="Glob patterns for power nets to route with wider tracks (e.g., '*GND*' '*VCC*')")
    parser.add_argument("--power-nets-widths", nargs="+", type=float, default=None,
                        help="Track widths in mm for each power-net pattern (must match --power-nets length)")

    # Multi-net layer connection options
    parser.add_argument("--plane-proximity-radius", type=float, default=3.0,
                        help="Radius around other nets' vias to add proximity cost when routing plane connections (mm, default: 3.0)")
    parser.add_argument("--plane-proximity-cost", type=float, default=2.0,
                        help="Maximum proximity cost around other nets' vias when routing plane connections (mm equivalent, default: 2.0)")
    # #381 D9: --track-via-clearance is the same constant repair_planes
    # spells that way; accept both here (dest stays plane_track_via_clearance).
    parser.add_argument("--plane-track-via-clearance", "--track-via-clearance",
                        type=float, default=defaults.PLANE_TRACK_VIA_CLEARANCE,
                        help="Clearance from track center to other nets' via centers when routing MST connections (mm, default: 0.8)")
    parser.add_argument("--voronoi-seed-interval", type=float, default=2.0,
                        help="Sample interval for Voronoi seed points along plane connection routes (mm, default: 2.0)")
    parser.add_argument("--plane-max-iterations", type=int, default=defaults.MAX_ITERATIONS,
                        help="Max A* iterations for routing plane connections (default: 200000)")

    # Board edge clearance
    parser.add_argument("--board-edge-clearance", type=float, default=None,
                        help=f"Clearance from board edge for zones in mm (default: the board "
                             f"min_copper_edge_clearance, else {defaults.PLANE_EDGE_CLEARANCE})")
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)

    # Same-net pad clearance (avoid via-in-pad)
    parser.add_argument("--same-net-pad-clearance", type=float,
                        default=None,
                        help="Edge-to-edge clearance (mm) between placed vias and same-net pads. "
                             "> 0 keeps ALL of this step's vias (stitching, taps, joins) off "
                             "same-net pads and is recorded in the sibling .kicad_pro so later "
                             "chain steps (route/route_diff/fanout/repair) inherit it (#581); "
                             "0 keeps its legacy stitching-only meaning; -1 explicitly allows "
                             "via-in-pad. Default: the project's recorded value, else -1.")

    # Debug options
    parser.add_argument("--dry-run", action="store_true", help="Analyze without writing output")
    parser.add_argument("--skip-existing-zones", action="store_true",
                        help="Keep an existing same-net zone (don't recreate); "
                             "tolerate other-net zones on the same layer (e.g. a GND island under an RF feed). "
                             "Optional via features (--stitch-vias, thermal arrays) still run")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print detailed DEBUG messages")
    parser.add_argument("--debug-lines", action="store_true", help="Output MST routes on User.1, User.2, etc. per net")
    parser.add_argument("--add-teardrops", action="store_true",
        help="Add teardrop settings to all pads and vias in output file")

    # Area via stitching (#485)
    parser.add_argument("--stitch-vias", action="store_true",
        help="Area via stitching: bond each plane net's pours across layers "
             "with a periodic via lattice. Applies to the --nets that own "
             ">= 2 of the --plane-layers (no separate net selection). Every "
             "site is gated by the predicted zone fill (no pour necking, no "
             "island taps), validated against KiCad's own exact fill (one "
             "pcbnew refill; a site off the net's main filled cluster is "
             "rejected rather than anchoring an isolated island), and the "
             "same obstacle/hole-to-hole/edge checks as any routed via.")
    parser.add_argument("--stitch-pitch", type=float, default=defaults.STITCH_PITCH,
        help=f"Lattice pitch for --stitch-vias in mm (default: {defaults.STITCH_PITCH})")
    parser.add_argument("--stitch-max-freq", type=float, default=None,
        help="Maximum frequency of interest in MHz: derives the stitching "
             "pitch as lambda/20 using the largest dielectric epsilon_r in "
             "the board's stackup (FR-4 4.5 if the board has none), "
             "overriding --stitch-pitch. The fence pitch follows unless "
             "--stitch-fence-pitch is given.")
    parser.add_argument("--stitch-edge-fence", action="store_true",
        help="Board-edge via fence: a row of stitching vias tracking the "
             "board outline(s) (EMI guard ring). Same net rule and site "
             "gates as --stitch-vias; works with or without it.")
    parser.add_argument("--stitch-fence-pitch", type=float, default=None,
        help="Via spacing along the edge fence in mm (default: the stitch "
             "pitch)")
    parser.add_argument("--stitch-inset", type=float, default=None,
        help="Fence distance from the board edge to the via centers in mm "
             "(default: auto -- the board edge clearance plus the fill-margin "
             "ring, i.e. as close to the edge as a via can sit and keep the "
             "pour intact)")

    # GND return vias
    parser.add_argument("--add-gnd-vias", action="store_true",
        help="Add GND vias near signal vias for return current path")
    parser.add_argument("--gnd-via-net", type=str, default=defaults.GND_VIA_NET,
        help="Pin GND return vias to this net (default: auto -- match each "
             "signal's own ground domain, which is plain GND on a board with "
             "one ground)")
    parser.add_argument("--gnd-via-distance", type=float, default=defaults.GND_VIA_DISTANCE,
        help="Maximum distance from signal via to place GND via in mm (default: 2.0)")

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()
    # #439: identical net-class/clearance model to route.py. --clearance is the
    # clamp switch: GIVEN -> ceiling, every class capped at min(class, ceiling),
    # writeback clamps (_clamp_netclasses True). OMITTED -> each net routes at its
    # own class (base = board Default class), classes preserved. Geometry flags
    # omitted -> default from the board (track/clearance/via from Default net-class,
    # hole/edge from board constraints, else a routing_defaults constant). Planes
    # keep their larger PLANE_EDGE_CLEARANCE fallback when the board declares none.
    # Resolved here, before enforce_fab_floors and every downstream use.
    from list_nets import (board_default_netclass_clearance, board_default_netclass_param,
                           resolve_cli_floor)
    for _pname, _nckey, _fallback in (('track_width', 'track_width', defaults.TRACK_WIDTH),
                                      ('via_size', 'via_diameter', defaults.VIA_SIZE),
                                      ('via_drill', 'via_drill', defaults.VIA_DRILL)):
        if getattr(args, _pname) is None:
            _v = board_default_netclass_param(args.input_file, _nckey)
            setattr(args, _pname, _v if _v is not None else _fallback)
            print(f"--{_pname.replace('_', '-')} not given; using "
                  f"{'the board Default net-class' if _v is not None else 'the fallback'} "
                  f"{getattr(args, _pname)}mm.")
    _ceiling = args.clearance                       # None iff --clearance omitted
    args._clamp_netclasses = _ceiling is not None
    args._clearance_ceiling = _ceiling
    from fix_kicad_drc_settings import warn_if_missing_project_floor
    warn_if_missing_project_floor(args.input_file)  # #441: a dropped sibling .kicad_pro strands the DRC floor
    _dflt_clr = board_default_netclass_clearance(args.input_file)
    if _ceiling is None:
        args.clearance = _dflt_clr if _dflt_clr is not None else defaults.CLEARANCE
        print(f"--clearance not given; honoring net classes with base = "
              f"{'the board Default net-class' if _dflt_clr is not None else 'the fallback'} "
              f"clearance {args.clearance}mm.")
    else:
        args.clearance = min(_dflt_clr, _ceiling) if _dflt_clr is not None else _ceiling
    # Shared resolver (list_nets.resolve_cli_floor). Planes keep their larger
    # edge keep-out (PLANE_EDGE_CLEARANCE) only when the board declares no edge
    # rule of its own -- and a DECLARED 0.0 is exactly that case, KiCad's "not
    # configured". Read straight it was taken as a rule, dropping the plane
    # inset from 0.5 to 0.0 while the GUI's plane tab held 0.5 (swig_gui.py
    # _effective_plane_edge_clearance already had the `> 1e-9` guard).
    args.hole_to_hole_clearance = resolve_cli_floor(
        args.input_file, 'hole_to_hole', args.hole_to_hole_clearance,
        defaults.HOLE_TO_HOLE_CLEARANCE, '--hole-to-hole-clearance')
    args.board_edge_clearance = resolve_cli_floor(
        args.input_file, 'board_edge_clearance', args.board_edge_clearance,
        defaults.PLANE_EDGE_CLEARANCE, '--board-edge-clearance')
    set_default_fab_tier(*fab_tier_from_args(args))
    __import__('fab_tiers').set_policy_from_args(args, args.input_file)  # #857
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.input_file),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None),
        board_edge_clearance=getattr(args, 'board_edge_clearance', None))
    # Below-floor params are pinned up to the fab floor (warned); apply the clamps.
    for _pname, _pfloor in _pinned_floors.items():
        setattr(args, _pname, _pfloor)

    # #381 D9: --output FILE overrides the positional (matches route.py/route_diff).
    if getattr(args, 'output', None) is not None:
        if args.output_file is not None and args.output_file != args.output:
            parser.error("both a positional output and --output were given and differ")
        args.output_file = args.output
    # Handle output file: use --overwrite, explicit output, or auto-generate with _routed suffix
    if args.output_file is None:
        if args.overwrite:
            args.output_file = args.input_file
        else:
            # Auto-generate output filename: input.kicad_pcb -> input_routed.kicad_pcb
            base, ext = os.path.splitext(args.input_file)
            args.output_file = base + '_routed' + ext
            print(f"Output file: {args.output_file}")

    # Default layers to F.Cu + plane-layers + B.Cu (need outer layers to reach pads)
    if args.layers is None:
        layers = ['F.Cu'] + args.plane_layers + ['B.Cu']
        # Remove duplicates while preserving order
        seen = set()
        args.layers = [l for l in layers if not (l in seen or seen.add(l))]

    # Validate net/plane-layer counts match
    if len(args.nets) != len(args.plane_layers):
        print(f"Error: Number of net arguments ({len(args.nets)}) must match number of plane layers ({len(args.plane_layers)})")
        print("Each net argument needs a corresponding plane layer")
        print("Use | to separate multiple nets on the same layer (e.g., --nets GND 'VA19|VA11' --plane-layers In4.Cu In5.Cu)")
        return

    # Parse --nets arguments: detect | separator for multi-net layers
    # Build data structures:
    #   net_names: List[str] - all individual net names (expanded)
    #   plane_layers: List[str] - layer for each net (expanded to match net_names)
    #   layer_nets: Dict[str, List[str]] - layer → list of nets on that layer
    net_names = []
    plane_layers = []
    layer_nets = {}

    for net_arg, layer in zip(args.nets, args.plane_layers):
        nets_on_layer = [n.strip() for n in net_arg.split('|')]
        for net in nets_on_layer:
            net_names.append(net)
            plane_layers.append(layer)

        # Track nets per layer
        if layer not in layer_nets:
            layer_nets[layer] = []
        layer_nets[layer].extend(nets_on_layer)

    # Report multi-net layers
    for layer, nets in layer_nets.items():
        if len(nets) > 1:
            print(f"Layer {layer} has multiple nets: {', '.join(nets)}")

    # Did the engine actually write a board? Every post-pass below re-reads the
    # OUTPUT file, and an early engine return (bad board bounds, a cancel, a
    # validation exit) writes nothing -- running them anyway raised
    # FileNotFoundError out of clean_plane_copper, burying the real error message
    # under a traceback. Compare against the pre-call state so a leftover file
    # from an EARLIER run isn't mistaken for this run's output.
    _out_before = (os.path.getmtime(args.output_file)
                   if args.output_file and os.path.isfile(args.output_file) else None)

    # The engine's cooperative `cancel_check` / `progress_callback` are the
    # GUI's (the planes tab's Cancel button); the CLI passes neither. There is
    # deliberately no wall-clock budget -- no result this tool produces may
    # depend on timing.
    create_plane(
        input_file=args.input_file,
        output_file=args.output_file,
        ripup_blocker_select=args.ripup_blocker_select,
        net_names=net_names,
        plane_layers=plane_layers,
        via_size=args.via_size,
        via_drill=args.via_drill,
        track_width=args.track_width,
        clearance=args.clearance,
        zone_clearance=args.zone_clearance,
        min_thickness=args.min_thickness,
        thermal_relief=args.thermal_relief,
        thermal_vias=args.thermal_vias,
        stitch_vias=args.stitch_vias,
        stitch_pitch=args.stitch_pitch,
        stitch_edge_fence=args.stitch_edge_fence,
        stitch_fence_pitch=args.stitch_fence_pitch,
        stitch_inset=args.stitch_inset,
        stitch_max_freq=args.stitch_max_freq,
        grid_step=args.grid_step,
        hole_to_hole_clearance=args.hole_to_hole_clearance,
        all_layers=args.layers,
        verbose=args.verbose,
        dry_run=args.dry_run,
        layer_nets=layer_nets,
        plane_proximity_radius=args.plane_proximity_radius,
        plane_proximity_cost=args.plane_proximity_cost,
        plane_track_via_clearance=args.plane_track_via_clearance,
        board_edge_clearance=args.board_edge_clearance,
        voronoi_seed_interval=args.voronoi_seed_interval,
        plane_max_iterations=args.plane_max_iterations,
        debug_lines=args.debug_lines,
        layer_costs=args.layer_costs,
        power_nets=args.power_nets,
        power_nets_widths=args.power_nets_widths,
        add_teardrops=args.add_teardrops,
        same_net_pad_clearance=args.same_net_pad_clearance,
        skip_existing_zones=args.skip_existing_zones,
        no_bga_zone=args.no_bga_zone,
        clamp_netclasses=args._clamp_netclasses,
        clearance_ceiling=args._clearance_ceiling,
    )

    _wrote_output = bool(args.output_file) and os.path.isfile(args.output_file) and (
        _out_before is None or os.path.getmtime(args.output_file) != _out_before)
    if not args.dry_run and not _wrote_output:
        print("\nNo output board was written (see the error above); skipping the "
              "post-route passes: GND return vias, plane copper cleanup and the "
              "DRC-floor writeback.")

    # Add GND return vias if requested
    if args.add_gnd_vias and not args.dry_run and _wrote_output:
        from kicad_parser import parse_kicad_pcb
        from routing_config import GridRouteConfig, GridCoord
        from obstacle_map import build_base_obstacle_map
        from add_gnd_vias import add_gnd_vias_to_existing_board
        from kicad_writer import add_tracks_and_vias_to_pcb
        from fix_kicad_drc_settings import effective_board_edge_clearance

        print(f"\nAdding GND return vias near signal vias...")

        # Parse the output file (which now has planes)
        pcb_data = parse_kicad_pcb(args.output_file)

        # Create config for GND via placement
        gnd_config = GridRouteConfig(
            via_size=args.via_size,
            via_drill=args.via_drill,
            track_width=args.track_width,
            clearance=args.clearance,
            grid_step=args.grid_step,
            # A THROUGH via must clear copper on EVERY board layer, not just
            # the configured routing layers: with --layers omitting an inner
            # layer, stitching vias were placed straight through its tracks
            # (0.3-0.46mm overlaps on In2 in the bitaxe repro).
            layers=list(pcb_data.board_info.copper_layers),
            # Thread the fab hole-to-hole minimum through so GND-via placement
            # enforces the real drill spacing (issue #125), not the 0.2mm default.
            hole_to_hole_clearance=args.hole_to_hole_clearance,
            # Copper-to-EDGE, so build_base_obstacle_map populates the static
            # off-board keep-out (#422) and a return via cannot land outside the
            # outline. Without it this config carried the 0.0 default, the map had
            # no edge keep-out at all, and add_gnd_vias -- which checks track and
            # via-to-via clearance but never the outline -- placed a GND via
            # 1.40mm BEYOND the board edge (test-board: (124.20, 82.40) against a
            # y=81.0 edge), i.e. copper that is milled away at depanelization.
            #
            # NOT args.board_edge_clearance: that one is the plane-zone INSET
            # (see the note at the DRC writeback below), not the enforced
            # copper-to-edge rule. cli=0 reads the board's own
            # min_copper_edge_clearance, floored at the fab minimum -- the same
            # idiom the zone-inset resolution above uses.
            board_edge_clearance=effective_board_edge_clearance(args.input_file, 0.0),
        )
        from kicad_dru import install_layer_clearances
        install_layer_clearances(gnd_config, None, None, pcb_data)  # #498
        coord = GridCoord(gnd_config.grid_step)

        # Build obstacle map
        obstacles = build_base_obstacle_map(pcb_data, gnd_config, [])

        # Add GND vias
        gnd_vias = add_gnd_vias_to_existing_board(
            pcb_data,
            args.gnd_via_net,
            args.gnd_via_distance,
            gnd_config,
            obstacles,
            coord
        )

        if gnd_vias:
            # Convert Via objects to dict format for writer
            via_dicts = [{
                'x': v.x,
                'y': v.y,
                'size': v.size,
                'drill': v.drill,
                'net_id': v.net_id,
                'layers': v.layers,
                'free': getattr(v, 'free', False)
            } for v in gnd_vias]

            # Write vias to output file
            add_tracks_and_vias_to_pcb(
                args.output_file,
                args.output_file,
                tracks=[],
                vias=via_dicts
            )
            print(f"Wrote {len(gnd_vias)} GND vias to {args.output_file}")

    # Dead-end sweep + gap-snap on the plane copper (issue #84): plane routing
    # writes outside route.py's write-list, so its dead-end stubs are not cleaned
    # otherwise. Gated against connectivity + pours, so it never breaks a net.
    if not args.dry_run and _wrote_output:
        from pcb_modification import clean_plane_copper
        _snapped, _removed = clean_plane_copper(args.output_file, net_names,
                                                args.clearance, args.grid_step)
        if _snapped or _removed:
            print(f"Plane cleanup: closed {_snapped} stub gap(s), trimmed {_removed} dead-end segment(s)")
        # Castellated landings (run-6 fix 1.7): tap/join copper that landed in
        # a castellated pad's edge-clearance zone is pulled to its inner reach.
        try:
            from fix_kicad_drc_settings import effective_board_edge_clearance
            from pcb_modification import retract_castellated_landings
            _edge = effective_board_edge_clearance(args.input_file, 0.0)
            if _edge > 0:
                retract_castellated_landings(args.output_file, _edge)
        except Exception as e:
            print(f"  (skipped castellated-landing retract: {e})")

    # NO KiCad-oracle recheck here (#217): the plane this step just poured has NOT
    # yet been stitched -- tying pads/islands into the pour is route_disconnected_
    # planes' job, the very next step. At the route_planes stage KiCad reports every
    # not-yet-stitched pad as a missing link (hackrf: 26 plane links / 42 total),
    # so an oracle pass here thrashes routing links that don't exist as failures
    # (18 routed, 77 failed, 3 kicad-cli rounds) -- a 2-8x route_planes regression
    # for work repair_planes does properly. The oracle runs ONCE, as an
    # end-of-pipeline fallback at the end of repair_planes, on the
    # already-repaired board -- do not re-add it here.

    # Make the output project's KiCad DRC constraints consistent with the routed
    # clearances/sizes (issue #160); only edits the .kicad_pro, never the board.
    if not args.no_fix_drc_settings and not args.dry_run \
            and args.output_file and os.path.isfile(args.output_file):
        try:
            import clearance_ledger
            eff_clearance = clearance_ledger.effective(args.clearance)
            if eff_clearance < args.clearance:
                print(f"  Min clearance used: {eff_clearance:.4g} mm "
                      f"(below nominal {args.clearance:.4g}; fine-pitch taps) - "
                      f"grading at this floor")
            from fix_kicad_drc_settings import (fix_project_for_output, drc_fix_kwargs,
                                                read_project_edge_clearance)
            # #338: record the PROJECT's copper-to-edge rule, never this
            # tool's --board-edge-clearance -- that is the plane-zone INSET
            # (default 0.5), not an enforced DRC floor; writing it into a
            # project with no edge key manufactured a stricter-than-design
            # rule (openstint: design 0.3, recorded 0.5).
            fix_project_for_output(
                args.output_file, input_pcb=args.input_file,
                clearance=eff_clearance, hole_to_hole=args.hole_to_hole_clearance,
                edge_clearance=read_project_edge_clearance(args.input_file),
                track_width=args.track_width,
                via_diameter=args.via_size, via_drill=args.via_drill,
                **drc_fix_kwargs(args))
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")
        # #581: record an ACTIVE same-net pad via clearance in the output's
        # project so every later chain step keeps its vias off same-net pads
        # too. After fix_project_for_output so the .kicad_pro exists.
        try:
            from protected_nets import (persist_same_net_pad_clearance,
                                        pro_path_for_board)
            persist_same_net_pad_clearance(
                pro_path_for_board(args.output_file),
                args.same_net_pad_clearance
                if args.same_net_pad_clearance is not None else -1.0)
        except Exception as e:
            print(f"  (skipped same-net pad clearance record: {e})")

    # Machine-readable summary so an orchestrator and the next pipeline step can
    # read the clearance this step actually used (mirrors route.py/route_diff.py).
    import json as _json, clearance_ledger as _cl
    _summary = {
        "min_clearance_used": _cl.effective(args.clearance),
        "plane_nets": sorted(set(args.nets)),
    }
    # `plane_nets` is a de-duplicated NAME list, and --nets/--plane-layers are
    # matched POSITIONALLY, so `--nets GND GND --plane-layers In1.Cu In2.Cu`
    # collapses two poured zones into one entry and the second one vanishes from
    # the record entirely. Run 14 poured GND on both inner layers and its
    # summary said `"plane_nets": ["GND"]`. Keep that key as it was -- consumers
    # read it -- and state the actual zones alongside.
    try:
        _layers = list(getattr(args, 'plane_layers', None) or [])
        _summary["plane_zones"] = [
            {"net": _n, "layer": (_layers[_i] if _i < len(_layers) else None)}
            for _i, _n in enumerate(args.nets)]
    except Exception:                                       # noqa: BLE001
        pass
    # #487: the plane resistance/ampacity numbers used to live only in stdout
    # ("report-only ... print and discard"). Fold the per-net results the
    # engine noted into the machine-readable summary so chains/graders/skills
    # can gate on them.
    # The pad tally. Without it `complete: true, status: "ok", EXIT=0` is the
    # whole machine-readable story of a pour that left pads open -- the text
    # said "Total failed pads: 5" and the JSON had no channel for it at all.
    _pt = consume_pad_tally()
    if _pt:
        _summary["pads"] = _pt
        # SAY that this is one tally over every zone. The engine aggregates, so
        # `failed_pads: 12` on a two-zone pour does not say which zone failed,
        # and a reader pairing it with a single-entry `plane_nets` would
        # reasonably assume it was scoped to that one net.
        _summary["pads_scope"] = ("all zones in this run, summed -- not per "
                                  "zone; see plane_zones for what was poured")
        # `geometric_failed` is recorded but does NOT flip the status: under
        # the pours-first architecture the plane step places no taps, so pads
        # unreached by the pour alone are EXPECTED here (#562) -- the route
        # step welds them and its finalize verifies/repairs. Only
        # `failed_pads` (the engine's own via-placement failures) marks the
        # pour incomplete.
        if _pt.get('failed_pads'):
            _summary["status"] = "incomplete-pads"

    try:
        from plane_resistance import consume_resistance_results
        _res = consume_resistance_results()
        if _res:
            _summary["plane_resistance"] = [
                {"net": _n,
                 "resistance_ohms": round(_r.get("resistance", 0.0), 6),
                 "max_current_a": round(_r.get("max_current", 0.0), 2),
                 "max_current_ipc2152_a": round(_r.get("max_current_ipc2152", 0.0), 2),
                 "copper_oz": _r.get("copper_oz"),
                 "temp_rise_c": _r.get("temp_rise_c"),
                 "path_length_mm": round(_r.get("path_length", 0.0), 2),
                 "avg_width_mm": round(_r.get("avg_width", 0.0), 3)}
                for _n, _r in sorted(_res.items())]
    except Exception:
        pass

    # The run either finished or raised; there is no partial shape to report
    # (the CLI has no cancel source). `complete`/`status` are kept because
    # consumers read them -- see route_summary's sticky-incompleteness merge.
    import json as _json
    _summary.setdefault('complete', True)
    _summary.setdefault('status', 'ok')
    try:                       # #653: env knobs into the machine-readable
        import env_knobs as _ek653   # summary, so a harness can detect a
        _summary['env_knobs'] = _ek653.active_env_knobs()   # dirty baseline
    except Exception:          # without re-reading logs
        pass
    print('JSON_SUMMARY: ' + _json.dumps(_summary, sort_keys=True, default=str),
          flush=True)
    return 0


if __name__ == "__main__":
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    # CMD/EXIT self-echo (run-3 B1); see route.py for the external-kill caveat.
    # CLI-`__main__`-only: the GUI imports create_plane.
    import cli_banner
    cli_banner.install()
    # `or 0`: main() has one early `return` (the net/plane-layer count
    # mismatch) that returns None, and returning None from sys.exit is 0 --
    # which is what this block did before, so that path is unchanged.
    sys.exit(main() or 0)
