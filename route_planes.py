"""
Copper Plane Generator - Creates filled zones with via stitching to net pads.

Creates a solid copper zone on a specified layer, places vias near target pads,
and routes short traces to connect vias to pads when direct placement is blocked.

Usage:
    python route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu
"""
from __future__ import annotations

import sys
import os
import argparse
from typing import List, Optional, Tuple, Dict, Set, Union
from dataclasses import dataclass

# Run startup checks first (validates numpy, scipy, shapely are installed)
from startup_checks import run_all_checks
run_all_checks()

# These imports are guaranteed to work after startup_checks passes
import numpy as np

from kicad_parser import parse_kicad_pcb, PCBData, Pad, Via, Segment, KICAD_10_MIN_VERSION, pad_is_plated_through
from kicad_writer import generate_zone_sexpr, generate_gr_line_sexpr
from routing_config import GridRouteConfig, GridCoord
from routing_utils import point_in_pad_rect, pad_rect_halfspan
from route import batch_route, _dump_engine_config
from obstacle_cache import ViaPlacementObstacleData
from connectivity import compute_mst_segments

# Import from new refactored modules
from plane_io import (
    ZoneInfo,
    extract_zones,
    check_existing_zones,
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
from plane_blocker_detection import (
    ViaPlacementResult,
    find_via_position_blocker,
    find_route_blocker_from_frontier,
    try_place_via_with_ripup
)
from plane_zone_geometry import (
    compute_zone_boundaries,
    find_polygon_groups,
    sample_route_for_voronoi
)
from plane_pad_tap import (
    pad_is_fine_pitch,
    tap_pad_with_escalation,
    fab_floor_clearance_track,
    clamp_tap_via_to_edge,
    FINE_TAP_GRID_STEP,
)
from terminal_colors import GREEN, RED, RESET

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads
from grid_router import GridObstacleMap, GridRouter

# Plane resistance calculations
from plane_resistance import (
    analyze_single_net_plane,
    analyze_multi_net_plane,
    print_single_net_resistance,
    print_multi_net_resistance
)
import routing_defaults as defaults


class ViaSpatialIndex:
    """Grid-based spatial index for fast nearest-via queries."""

    def __init__(self, bucket_size: float):
        self.bucket_size = bucket_size
        self._buckets: Dict[Tuple[int, int], List[Tuple[float, float]]] = {}

    def _key(self, x: float, y: float) -> Tuple[int, int]:
        return (int(x // self.bucket_size), int(y // self.bucket_size))

    def add(self, x: float, y: float):
        key = self._key(x, y)
        if key not in self._buckets:
            self._buckets[key] = []
        self._buckets[key].append((x, y))

    def add_all(self, vias: List[Tuple[float, float]]):
        for x, y in vias:
            self.add(x, y)

    def find_nearest(self, x: float, y: float, max_radius: float) -> Optional[Tuple[float, float]]:
        """Find nearest via within max_radius of (x, y)."""
        best_via = None
        best_dist_sq = max_radius * max_radius
        # Check all buckets within radius
        r_buckets = int(max_radius / self.bucket_size) + 1
        cx, cy = self._key(x, y)
        for bx in range(cx - r_buckets, cx + r_buckets + 1):
            for by in range(cy - r_buckets, cy + r_buckets + 1):
                bucket = self._buckets.get((bx, by))
                if bucket is None:
                    continue
                for vx, vy in bucket:
                    dx = vx - x
                    dy = vy - y
                    dist_sq = dx * dx + dy * dy
                    if dist_sq <= best_dist_sq:
                        best_dist_sq = dist_sq
                        best_via = (vx, vy)
        return best_via


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
    position_filter=None
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

    Returns:
        (x, y) position for via, or None if no valid position found
    """
    pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

    # Try pad center first - if not blocked, use it (no routing needed)
    if not obstacles.is_via_blocked(pad_gx, pad_gy):
        if position_filter is None or position_filter(pad.global_x, pad.global_y):
            return (pad.global_x, pad.global_y)

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
                    return (bx, by)

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
    seen_bins = set()

    for gx, gy in open_cells:
        dx, dy = gx - pad_gx, gy - pad_gy

        # Outside the caller's allowed region (e.g. the net's own Voronoi
        # cell/zone polygon, issue #287) - not a usable plane tap.
        if position_filter is not None:
            fx, fy = coord.to_float(gx, gy)
            if not position_filter(fx, fy):
                continue

        # Skip if too close to a previously failed route position
        if failed_route_positions and skip_radius_sq > 0:
            too_close = False
            for failed_gx, failed_gy in failed_route_positions:
                fdx = gx - failed_gx
                fdy = gy - failed_gy
                if fdx * fdx + fdy * fdy <= skip_radius_sq:
                    too_close = True
                    break
            if too_close:
                continue

        # Skip if inside a pending pad's exclusion zone
        if pending_pad_zones:
            in_zone = False
            for min_gx, min_gy, max_gx, max_gy in pending_pad_zones:
                if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                    in_zone = True
                    break
            if in_zone:
                continue

        # Thin redundant near-duplicate via sites: keep the nearest PASSING cell
        # per via-radius bin (see via_bin note above). Done here, after the skip
        # checks, so a bin is never consumed by a cell that was itself skipped.
        if via_bin:
            bin_key = (gx // via_bin, gy // via_bin)
            if bin_key in seen_bins:
                continue
            seen_bins.add(bin_key)

        dist_sq = dx * dx + dy * dy
        valid_positions.append((dist_sq, coord.to_float(gx, gy), gx, gy))

    # The Rust query is already nearest-first; sort anyway so the fallback path
    # (and any future unordered source) is correct.
    valid_positions.sort(key=lambda x: x[0])

    # If no routing check needed, return closest valid position
    if routing_obstacles is None or config is None:
        if valid_positions:
            return valid_positions[0][1]
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
    skipped_count = 0
    for dist_sq, via_pos, gx, gy in valid_positions:
        # Skip cells near a position where routing already failed (rip-up retries)
        if failed_route_positions and skip_radius_sq > 0:
            if any((gx - fgx) ** 2 + (gy - fgy) ** 2 <= skip_radius_sq
                   for fgx, fgy in failed_route_positions):
                skipped_count += 1
                continue
        source_cells.append((gx, gy, layer_idx))
        src_set.add((gx, gy))

    if not source_cells:
        if verbose:
            print(f"[skipped {skipped_count}, no candidates left]", end=" ")
        return None

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

    # One search, seeded from all candidates; generous budget since it replaces K.
    ms_iters = max(10000, min(60000, len(source_cells) * 4))
    path, iterations, _ = router.route_with_frontier(
        routing_obstacles, source_cells, [(pad_gx, pad_gy, layer_idx)], ms_iters,
        False,  # collinear_vias
        0,      # via_exclusion_radius
        None,   # start_direction
        None,   # end_direction
        0       # direction_steps
    )
    routing_obstacles.clear_source_target_cells()

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
    return None  # No valid position with routable path


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
        False,  # collinear_vias
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
        False,  # collinear_vias
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
    num_offsets = len(circle_offsets)

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
            proximity_cost_grid,
            False
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
        False,  # collinear_vias
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
    verbose: bool
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

    # A net earns a zone if it has ANY connection point on this layer -- a via
    # or a pad. nets_with_vias still drives the via-to-via MST routing below;
    # nets_with_seeds (vias OR pads) drives whether a zone is created at all.
    nets_with_vias = []
    nets_with_seeds = []
    for net_name in nets_on_layer:
        net_id = net_name_to_id.get(net_name)
        if not net_id:
            continue
        via_count = len(vias_by_net.get(net_id, []))
        pad_count = len(pads_on_layer_by_net.get(net_id, []))
        if via_count == 0 and pad_count == 0:
            print(f"  Warning: Net '{net_name}' has no vias or pads on layer {layer}, skipping zone")
            continue
        nets_with_seeds.append(net_name)
        if via_count > 0:
            nets_with_vias.append(net_name)
            print(f"  Net '{net_name}': {via_count} vias")
        else:
            print(f"  Net '{net_name}': no vias, {pad_count} pad(s) on {layer} (pad-seeded zone)")

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
                direct_connect=True,
                use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION
            )
            zone_sexprs.append(zone_sexpr)
            zone_data_list.append({
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
        fallback_pad_refs: List[str] = []
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
        if th_connected or th_fallback:
            msg = f"  #107: routed corridors for {th_connected} orphaned TH pad(s)"
            if th_fallback:
                msg += (f"; {th_fallback} could not be reached on the plane layer "
                        f"and fell back to point-seed ({', '.join(fallback_pad_refs)})")
            print(msg)

    # #114: a net with no stitching vias gets no MST and therefore no Voronoi
    # seeds from the routing above. Seed it directly from its pads on this layer
    # (through-hole + SMD-on-layer) so it still receives a Voronoi-partitioned
    # zone. Via-bearing nets are left to the #107 corridor logic above so their
    # via topology is not perturbed.
    if augmented_vias_by_net is not None:
        for net_name in nets_with_seeds:
            net_id = net_name_to_id.get(net_name)
            if net_id is None or vias_by_net.get(net_id):
                continue
            seeds = augmented_vias_by_net.setdefault(net_id, [])
            seen = {(round(x, 3), round(y, 3)) for x, y in seeds}
            for px, py in pads_on_layer_by_net.get(net_id, []):
                k = (round(px, 3), round(py, 3))
                if k not in seen:
                    seeds.append((px, py))
                    seen.add(k)

    # Compute final Voronoi zones
    total_seeds = sum(len(vias) for vias in augmented_vias_by_net.values())
    print(f"  Computing final Voronoi zones with {total_seeds} seed points")

    try:
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
            direct_connect=True,
            use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION
        )
        zone_sexprs.append(zone_sexpr)
        zone_data_list.append({
            'net_id': net_id,
            'net_name': net_name,
            'layer': layer,
            'polygon_points': zone_polygon,
            'clearance': zone_clearance,
            'min_thickness': min_thickness,
        })
        return zone_sexprs, debug_line_sexprs, zone_data_list

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
                direct_connect=True,
                use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION
            )
            zone_sexprs.append(zone_sexpr)
            zone_data_list.append({
                'net_id': net_id,
                'net_name': net_name,
                'layer': layer,
                'polygon_points': polygon,
                'clearance': zone_clearance,
                'min_thickness': min_thickness,
            })

    # Calculate and print resistance
    resistance_results = {}
    for net_id, polygons in zone_polygons.items():
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"net_{net_id}"
        mst_edges = net_mst_edges.get(net_id, [])
        edge_routes = routed_paths_by_edge.get(net_id, {})
        largest_polygon = max(polygons, key=lambda p: len(p))
        result = analyze_multi_net_plane(largest_polygon, mst_edges, edge_routes, layer)
        resistance_results[net_name] = result

    print_multi_net_resistance(resistance_results)

    return zone_sexprs, debug_line_sexprs, zone_data_list


def _geometric_plane_verification(
    output_file: str,
    net_ids: List[int],
    net_names: List[str],
    plane_layers: List[str],
    ripped_net_ids: List[int],
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

    ripped_set = set(ripped_net_ids or [])
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


def _neck_plane_segments(all_new_segments, pcb_data, clearance, all_layers, min_width=0.1):
    """Neck plane tap segments so they clear foreign vias AND pads by `clearance`.

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
            margin = half + vr + clearance
            if vx < minx - margin or vx > maxx + margin or vy < miny - margin or vy > maxy + margin:
                continue
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((vx - x0) * dx + (vy - y0) * dy) / L2))
            d = math.hypot(vx - (x0 + t * dx), vy - (y0 + t * dy))
            allowed = d - vr - clearance - EPS
            if allowed < new_half:
                new_half = allowed
        # foreign pads on this layer (edge-to-edge distance via the DRC geometry)
        for pnid, pads in pcb_data.pads_by_net.items():
            if pnid == nid:
                continue
            for pad in pads:
                if not pad_on_layer(pad, layer):
                    continue
                pext = max(pad.size_x, pad.size_y) / 2.0
                margin = half + pext + clearance
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
                allowed = d - clearance - EPS  # d is already edge-to-edge from the pad
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
                           via_drill, hole_to_hole_clearance):
    """Run the full pre-write cleanup pipeline on plane tap copper and return the
    resulting all_new_segments.

    This is the copper-finalizing half of the plane write path, factored out so
    the file-writer (_write_output_and_reroute) and the dry-run/GUI path
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
    _neck_plane_segments(all_new_segments, pcb_data, clearance, all_layers)

    # 2. graze prune / nudge / dead-end sweep (returns a NEW list)
    if all_new_segments:
        from pcb_modification import cleanup_plane_taps_grazing
        scope = {s['net_id'] for s in all_new_segments}
        all_new_segments, gz_rm, gz_nudge, gz_swept = cleanup_plane_taps_grazing(
            pcb_data, all_new_segments, scope, clearance=clearance,
            max_shift=grid_step / 2, all_new_vias=all_new_vias,
            hole_to_hole=hole_to_hole_clearance)
        if gz_rm:
            print(f"  Graze prune: removed {gz_rm} grazing tap segment(s)")
        if gz_nudge:
            print(f"  Graze nudge: re-bent grazing tap jog(s) on {gz_nudge} net(s)")
        if gz_swept:
            print(f"  Dead-end sweep: trimmed {gz_swept} orphaned tap segment(s)")

    # 3. close soft joints (#334) -- last, appends bridges (from_restore=True)
    try:
        from pcb_modification import close_soft_joints
        bridge_results = []
        cfg_sj = GridRouteConfig(track_width=track_width, clearance=clearance,
                                 grid_step=grid_step, via_size=via_size,
                                 via_drill=via_drill, layers=all_layers)
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


def _write_output_and_reroute(
    input_file: str,
    output_file: str,
    all_zone_sexprs: List[str],
    all_debug_lines: List[str],
    all_new_vias: List[Dict],
    all_new_segments: List[Dict],
    all_ripped_net_ids: List[int],
    zones_to_replace: List[Tuple[int, str]],
    pcb_data: PCBData,
    reroute_ripped_nets: bool,
    all_layers: List[str],
    plane_layers: List[str],
    track_width: float,
    clearance: float,
    via_size: float,
    via_drill: float,
    grid_step: float,
    hole_to_hole_clearance: float,
    verbose: bool,
    power_nets: Optional[List[str]] = None,
    power_nets_widths: Optional[List[float]] = None,
    add_teardrops: bool = False,
    no_bga_zone: bool = False,
    clamp_netclasses: bool = True,
    clearance_ceiling: Optional[float] = None
) -> bool:
    """
    Write output file and optionally reroute ripped nets.

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
                              add_teardrops=add_teardrops, net_id_to_name=kicad_v10_names):
        print("Error writing output file")
        return False

    print(f"Output written to {output_file}")
    print("Note: Open in KiCad and press 'B' to refill zones")

    if all_ripped_net_ids:
        # Verify against the WRITTEN output which ripped nets are actually
        # broken (issue #104 item 4: on KiCad 10 boards the writer's net
        # filter only matches numeric-id net refs, so a "ripped" net can
        # remain fully routed in the output). Only the broken subset needs
        # the reroute; connected ones would just churn.
        broken_ids = _verify_broken_ripped_nets(output_file, all_ripped_net_ids,
                                                pcb_data)
        broken_names = [pcb_data.nets[r].name for r in broken_ids
                        if r in pcb_data.nets]

        # #347 (same contract as route_disconnected_planes): a net this run
        # ripped to place plane vias must not depend on a LATER chain step
        # existing to reconnect it -- without this, manifests that stop after
        # the plane step ship those nets OPEN with only a warning (orangecrab
        # shipped 24 such nets). Handle the verified-broken subset in-run
        # even when --reroute-ripped-nets was not passed.
        #
        # ORDER MATTERS (DRC safety by construction, not by collision
        # filtering): the #88 restore is a blind re-add of the net's ORIGINAL
        # copper, so it must run BEFORE any reroute -- the only new copper on
        # the board at that point is this run's plane copper, which
        # restore_failed_reroute_nets already removes where it collides, and
        # originals cannot collide with each other or with untouched input
        # copper (the input board was legal). Rerouting FIRST and restoring
        # after (the old --reroute-ripped-nets order) stacked originals onto
        # sibling reroutes that had claimed the vacated corridors -- the
        # #141/eis hazard (measured on orangecrab: 60+ violations). After the
        # restore, whatever is STILL broken (no original copper to restore,
        # or a partial restore that did not reconnect) gets a FRESH
        # batch_route against the written board; routing is obstacle-aware,
        # so it cannot create collisions.
        if broken_names and not reroute_ripped_nets:
            print(f"\nNote: {len(broken_names)} ripped net(s) are broken in the "
                  f"output; restoring/rerouting them in-run (#347) even though "
                  f"--reroute-ripped-nets was not passed.")

        if broken_names:
            print(f"\n{'='*60}")
            print(f"Reconnecting {len(broken_names)} ripped net(s) "
                  f"(restore originals, then reroute the remainder)...")
            print(f"{'='*60}")
            # 1) Restore the broken nets' original traces (#88), removing this
            #    run's colliding plane copper so the restore reconnects rather
            #    than shorts.
            try:
                from plane_io import restore_failed_reroute_nets
                restored_ids, vias_removed, segs_removed = restore_failed_reroute_nets(
                    input_file=input_file,
                    output_file=output_file,
                    broken_net_ids=broken_ids,
                    # plane copper only: restored signal emissions are
                    # not plane copper and must not be removed-on-collision
                    plane_vias=[x for x in all_new_vias
                                if not x.get('from_restore')],
                    net_id_to_name=kicad_v10_names,
                    via_size=via_size,
                    clearance=clearance,
                    plane_segments=[x for x in all_new_segments
                                    if not x.get('from_restore')],
                    # partial-restore emissions already in the output
                    # must be stripped before the full-original restore
                    partial_copper=(
                        [x for x in all_new_segments if x.get('from_restore')],
                        [x for x in all_new_vias if x.get('from_restore')]),
                )
                if restored_ids:
                    names = [pcb_data.nets[r].name if r in pcb_data.nets
                             else f"net_{r}" for r in restored_ids]
                    print(f"Issue #88: RESTORED {len(restored_ids)} ripped net(s)' "
                          f"original trace (removed {vias_removed} colliding plane "
                          f"via(s), {segs_removed} colliding segment(s)):")
                    print(f"  {', '.join(names)}")
            except Exception as e:
                print(f"  (ripped-net restore step skipped: {e})")

            # 2) Reroute whatever the restore did not reconnect.
            still_broken = _verify_broken_ripped_nets(output_file, broken_ids,
                                                      pcb_data)
            still_names = [pcb_data.nets[r].name for r in still_broken
                           if r in pcb_data.nets]
            if still_names:
                print(f"\nRe-routing {len(still_names)} ripped net(s) the "
                      f"restore could not reconnect...")
                old_recursion_limit = sys.getrecursionlimit()
                sys.setrecursionlimit(max(old_recursion_limit, 100000))
                try:
                    # The reroute must know about copper on EVERY board layer,
                    # not just the plane step's layers: with the CLI's default
                    # --layers F.Cu B.Cu on a 6-layer board, all_layers+planes
                    # missed In3/In4 and batch_route dropped reroute vias onto
                    # pre-existing In3 copper (orangecrab: 22 via-seg
                    # violations). Route across the board's full copper stack.
                    board_layers = list(getattr(pcb_data.board_info,
                                                'copper_layers', None) or [])
                    all_copper_layers = (board_layers if board_layers
                                         else list(set(all_layers + plane_layers)))
                    # Issue #88.2: the reroute must use parameters compatible
                    # with the original signal-routing run, or nets that only
                    # routed with relaxed settings get silently dropped. In
                    # particular BGA auto-exclusion zones (on by default in
                    # batch_route) can re-block escapes the original run
                    # permitted with --no-bga-zone.
                    disable_bga = [] if no_bga_zone else None
                    # #338: this self-invocation routes from OUTPUT_FILE, whose
                    # sibling .kicad_pro may not exist yet, so batch_route's own
                    # edge resolution would read nothing. Resolve the enforced
                    # board-edge rule from the ORIGINAL input's project instead.
                    try:
                        from fix_kicad_drc_settings import effective_board_edge_clearance
                        # #441: pin to the fab floor. Do NOT forward this function's
                        # board_edge_clearance (the plane-zone inset); cli=0 reads the
                        # project rule and floors it at the fab edge minimum, so this
                        # self-invocation re-route never lays copper sub-fab when the
                        # (possibly missing) sibling .kicad_pro declares a 0/sub-fab rule.
                        _edge = effective_board_edge_clearance(input_file, 0.0)
                    except Exception:
                        _edge = 0.0
                    # #434: same missing-sibling-.kicad_pro hazard for the
                    # netclass map -- resolve it from the ORIGINAL input so the
                    # blocker reroutes honor cross-class clearances.
                    try:
                        from list_nets import net_clearance_map_by_id
                        _ncl = net_clearance_map_by_id(
                            input_file,
                            {nid: n.name for nid, n in pcb_data.nets.items()})
                        # #439: cap at the ceiling only when clamping.
                        if _ncl and clamp_netclasses and clearance_ceiling is not None:
                            _ncl = {nid: min(c, clearance_ceiling)
                                    for nid, c in _ncl.items()}
                    except Exception:
                        _ncl = None
                    routed, failed, route_time = batch_route(
                        input_file=output_file,
                        output_file=output_file,
                        net_names=still_names,
                        net_clearances=_ncl,
                        layers=all_copper_layers,
                        track_width=track_width,
                        clearance=clearance,
                        via_size=via_size,
                        via_drill=via_drill,
                        grid_step=grid_step,
                        hole_to_hole_clearance=hole_to_hole_clearance,
                        board_edge_clearance=_edge,
                        verbose=verbose,
                        minimal_obstacle_cache=True,
                        power_nets=power_nets,
                        power_nets_widths=power_nets_widths,
                        disable_bga_zones=disable_bga
                    )
                    print(f"\nRe-routing complete: {routed} routed, {failed} "
                          f"failed in {route_time:.2f}s")
                finally:
                    sys.setrecursionlimit(old_recursion_limit)
                final_broken = _verify_broken_ripped_nets(output_file,
                                                          still_broken, pcb_data)
                if final_broken:
                    names = [pcb_data.nets[r].name if r in pcb_data.nets
                             else f"net_{r}" for r in final_broken]
                    print(f"WARNING: {len(final_broken)} ripped net(s) remain "
                          f"disconnected (no original trace to restore, reroute "
                          f"failed): {', '.join(names)}")
        else:
            print(f"Note: {len(all_ripped_net_ids)} net(s) were ripped during via placement "
                  f"but remain fully connected in the output; no re-routing needed.")

    return True


def _verify_broken_ripped_nets(output_file: str, ripped_net_ids: List[int],
                               pcb_data: PCBData) -> List[int]:
    """Which of the ripped nets are actually broken in the written output.

    On verification failure (unparseable output etc.) every ripped net is
    treated as broken -- rerouting a connected net is harmless churn, but
    skipping a broken one ships it open.
    """
    try:
        from check_connected import check_net_connectivity
        out_pcb = parse_kicad_pcb(output_file)
        segs_by_net: Dict[int, List] = {}
        for s in out_pcb.segments:
            segs_by_net.setdefault(s.net_id, []).append(s)
        vias_by_net: Dict[int, List] = {}
        for v in out_pcb.vias:
            vias_by_net.setdefault(v.net_id, []).append(v)
        zones_by_net: Dict[int, List] = {}
        for z in out_pcb.zones:
            zones_by_net.setdefault(z.net_id, []).append(z)
        broken = []
        for rid in ripped_net_ids:
            result = check_net_connectivity(
                rid,
                segs_by_net.get(rid, []),
                vias_by_net.get(rid, []),
                out_pcb.pads_by_net.get(rid, []),
                zones_by_net.get(rid, []))
            if not result.get('connected', False):
                broken.append(rid)
        return broken
    except Exception:
        return list(ripped_net_ids)


def _empty_plane_results(return_results: bool):
    """Zero-work create_plane result in the shape the caller expects (#382 E5).

    The GUI (return_results=True) unpacks EXACTLY 8 values; the CLI unpacks 3.
    Every validation-error early return must go through here so a bad-input exit
    can't hand the GUI a short (3/6/7) tuple it will ValueError on -- the bug
    this consolidates. The 8-field shape mirrors the full return_results path:
    (vias, traces, pads_needing, new_vias, new_segments, new_zones,
     failed_pads, ripped_net_ids).
    """
    if return_results:
        return (0, 0, 0, [], [], [], 0, [])
    return (0, 0, 0)


def create_plane(
    input_file: str,
    output_file: str,
    net_names: List[str],
    plane_layers: List[str],
    via_size: float = defaults.VIA_SIZE,
    via_drill: float = defaults.VIA_DRILL,
    track_width: float = defaults.TRACK_WIDTH,
    clearance: float = defaults.CLEARANCE,
    zone_clearance: float = defaults.PLANE_ZONE_CLEARANCE,
    min_thickness: float = defaults.PLANE_MIN_THICKNESS,
    grid_step: float = defaults.GRID_STEP,
    max_search_radius: float = defaults.PLANE_MAX_SEARCH_RADIUS,
    max_via_reuse_radius: float = defaults.PLANE_MAX_VIA_REUSE_RADIUS,
    close_via_radius: float = None,
    hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE,
    all_layers: List[str] = None,
    verbose: bool = False,
    dry_run: bool = False,
    rip_blocker_nets: bool = False,
    max_rip_nets: int = defaults.PLANE_MAX_RIP_NETS,
    reroute_ripped_nets: bool = False,
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
    same_net_pad_clearance: float = defaults.SAME_NET_PAD_CLEARANCE,
    skip_existing_zones: bool = False,
    no_bga_zone: bool = False,
    progress_callback=None,
    cancel_check=None,
    net_clearances: Optional[dict] = None,
    clamp_netclasses: bool = True,
    clearance_ceiling: Optional[float] = None,
) -> Union[Tuple[int, int, int],
           Tuple[int, int, int, list, list, list, int, list]]:
    """
    Create copper plane zones and place vias to connect target pads for multiple nets.

    Args:
        net_names: List of net names to process (e.g., ['GND', 'VCC'])
        plane_layers: List of layers for each net (e.g., ['In1.Cu', 'In2.Cu'])
        rip_blocker_nets: If True, identify and temporarily remove nets blocking
                          via placement, then retry. Ripped nets excluded from output.
        max_rip_nets: Maximum number of blocker nets to rip up.
        reroute_ripped_nets: If True, automatically re-route ripped nets after placing vias.
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
        return_results=True (GUI): the 8-tuple
            (total_vias_placed, total_traces_added, total_pads_needing_vias,
             new_vias, new_segments, new_zones, total_failed_pads,
             ripped_net_ids).
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
    if pcb_data is None:
        print(f"Loading PCB from {input_file}...")
        pcb_data = parse_kicad_pcb(input_file)

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
    failed_pad_infos: List[Tuple[int, str, str, Dict]] = []

    # Step 2: Check for existing zones on each target layer.
    # Combine zones from the input file with zones already present in the
    # provided pcb_data (the live pcbnew board state when invoked from the
    # GUI). This way zones that exist on the live board but haven't been
    # saved to disk are also detected.
    try:
        existing_zones = list(extract_zones(input_file))
    except (FileNotFoundError, OSError):
        existing_zones = []
    seen_keys = {(z.net_name, z.layer) for z in existing_zones}
    for z in (getattr(pcb_data, 'zones', None) or []):
        key = (z.net_name, z.layer)
        if key in seen_keys:
            continue
        existing_zones.append(ZoneInfo(net_id=z.net_id, net_name=z.net_name, layer=z.layer))
        seen_keys.add(key)

    should_create_zones = []  # Per-net flag for whether to create zone
    zones_to_replace = []  # List of (net_id, layer) tuples for zones to replace
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
                      f"keeping it and only placing stitching vias")
                should_create_zones.append(False)
            else:
                should_create_zones.append(True)
            continue

        # CLI / strict path
        should_create, should_continue, zone_to_replace = check_existing_zones(
            existing_zones, plane_layer, net_name, net_id, verbose
        )
        if not should_continue:
            print(f"Error: Zone conflict for net '{net_name}' on layer {plane_layer}")
            return _empty_plane_results(return_results)
        should_create_zones.append(should_create)
        if zone_to_replace:
            zones_to_replace.append((zone_to_replace.net_id, zone_to_replace.layer))

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
        layer_costs=layer_costs
    )
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
    total_vias_reused = 0
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

        # Step 5: Identify target pads for this net
        if progress_callback:
            progress_callback(0, 0, f"{net_name}: analyzing pads on {plane_layer}...")
        target_pads = identify_target_pads(pcb_data, net_id, plane_layer)

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
        print(f"  SMD pads on other layers (via needed): {pads_need_via}")
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
            # Also block positions of vias we've already placed in previous nets
            for placed_via in all_new_vias:
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

        # Step 9: Place vias near each target pad (or reuse existing)
        new_vias = []
        new_segments = []
        vias_placed = 0
        vias_reused = 0
        pads_strapped = 0
        traces_added = 0
        failed_pads = 0
        ripped_net_ids: List[int] = []  # Nets ripped for this net

        # Track all available vias (existing + newly placed) for reuse
        available_vias = list(existing_net_vias)
        # Also include vias placed earlier for THIS net (not other nets!)
        for placed_via in all_new_vias:
            if placed_via['net_id'] == net_id:
                available_vias.append((placed_via['x'], placed_via['y']))
        # Build spatial index for fast nearest-via queries
        via_index = ViaSpatialIndex(bucket_size=max_search_radius)
        via_index.add_all(available_vias)

        # Cache for incremental obstacle updates during rip-up
        # Computed lazily when we first encounter each blocker net
        via_obstacle_cache: Dict[int, ViaPlacementObstacleData] = {}

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

        # Pre-build base pending pads list (other power net pads) once per net
        # This avoids rebuilding the same list for every pad in the inner loop
        base_pending_pads = []
        for other_net_id in net_ids:
            if other_net_id == net_id:
                continue
            other_pads = pcb_data.pads_by_net.get(other_net_id, [])
            for op in other_pads:
                base_pending_pads.append({'pad': op, 'needs_via': False})

        # Track ripped net pads incrementally
        ripped_pending_pads = []
        last_ripped_count = 0

        if pads_needing_vias:
            print(f"\nConnecting {len(pads_needing_vias)} pads to {plane_layer} plane:")

        # Index into failed_pad_infos where this net's failures start (for
        # the fine-pitch retry pass below).
        net_failed_start = len(failed_pad_infos)

        for pad_idx, pad_info in enumerate(pads_needing_vias):
            if cancel_check and cancel_check():
                print("  (cancelled)")
                break
            pad = pad_info['pad']
            pad_layer = pad_info.get('pad_layer')
            current_pad_key = (pad.global_x, pad.global_y)
            if progress_callback:
                progress_callback(pad_idx + 1, len(pads_needing_vias),
                                  f"{net_name}: via for {pad.component_ref}.{pad.pad_number}")

            # Skip pads already processed in previous layer passes
            # (a via connects ALL layers, so once placed, pad is done)
            if current_pad_key in processed_pad_ids:
                continue

            # Incrementally add pads from newly ripped nets
            if len(all_ripped_net_ids) > last_ripped_count:
                for ripped_id in all_ripped_net_ids[last_ripped_count:]:
                    ripped_pads = pcb_data.pads_by_net.get(ripped_id, [])
                    for rp in ripped_pads:
                        ripped_pending_pads.append({'pad': rp, 'needs_via': True})
                last_ripped_count = len(all_ripped_net_ids)

            # Filter base + ripped pending pads by current state
            pending_pads = [
                pp for pp in base_pending_pads
                if (pp['pad'].global_x, pp['pad'].global_y) != current_pad_key
                and (pp['pad'].global_x, pp['pad'].global_y) not in processed_pad_ids
            ]
            if ripped_pending_pads:
                pending_pads.extend(
                    pp for pp in ripped_pending_pads
                    if (pp['pad'].global_x, pp['pad'].global_y) != current_pad_key
                    and (pp['pad'].global_x, pp['pad'].global_y) not in processed_pad_ids
                )

            print(f"  Pad {pad.component_ref}.{pad.pad_number}...", end=" ")

            # First, check if there's already a via very close by (within ~2 via diameters)
            # This handles cases like decoupling caps where both pads are on same net
            # Check for nearby existing via before placing a new one
            effective_close_via_radius = close_via_radius if close_via_radius is not None else via_size * 2.5
            nearby_via = via_index.find_nearest(pad.global_x, pad.global_y, effective_close_via_radius)
            if nearby_via:
                # Via already very close - try to reuse it
                via_pos = nearby_via
                dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5

                if pad_layer:
                    routing_obs = get_routing_obstacles(pad_layer)
                    route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                   routing_obs, config, verbose=verbose,
                                                   return_blocked_cells=True, router=via_pad_router)
                    trace_segments = route_result.segments if route_result.success else None
                    if trace_segments is not None:
                        if trace_segments:
                            new_segments.extend(trace_segments)
                            traces_added += len(trace_segments)
                        vias_reused += 1
                        print(f"reused nearby via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {dist:.2f}mm away, routed {len(trace_segments) if trace_segments else 0} segments to pad")
                        processed_pad_ids.add(current_pad_key)
                        continue  # Move to next pad
                else:
                    # No pad layer (through-hole) - just count as reused
                    vias_reused += 1
                    print(f"reused nearby via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {dist:.2f}mm away")
                    processed_pad_ids.add(current_pad_key)
                    continue

            # Issue #349: before drilling, strap to an ADJACENT same-net pad that
            # is already plane-connected (processed earlier this run, or a plated
            # through-hole). Adjacent same-net pad clusters (fine-pitch power
            # pins, GND rows) then share ONE via plus short pad-layer straps
            # instead of a via per pad -- fewer drills and less via congestion
            # near connectors (the #347/#348 corridor-contention class). The
            # strap uses the same obstacle-aware trace as via reuse, so a strap
            # that doesn't fit falls through to normal via placement below.
            if pad_layer:
                strap_cands = []
                for npad in pcb_data.pads_by_net.get(net_id, []):
                    nkey = (npad.global_x, npad.global_y)
                    if nkey == current_pad_key:
                        continue
                    if not (nkey in processed_pad_ids or pad_is_plated_through(npad)):
                        continue
                    if not (pad_layer in npad.layers
                            or any(l.startswith('*') for l in npad.layers)
                            or pad_is_plated_through(npad)):
                        continue
                    d = ((npad.global_x - pad.global_x) ** 2 +
                         (npad.global_y - pad.global_y) ** 2) ** 0.5
                    if d > defaults.PLANE_PAD_STRAP_RADIUS:
                        continue
                    # Anchor the strap on the neighbour's VIA when one sits in
                    # its pad copper (via-in-pad taps land off-center): the
                    # connectivity graph follows exact endpoint/via keys, so a
                    # strap ending at the pad CENTER next to an off-center
                    # in-pad via reads as unconnected and gets re-tapped by the
                    # repair pass. A plated through-hole neighbour conducts at
                    # its center (barrel), so the center is the right anchor.
                    npos = nkey
                    if not pad_is_plated_through(npad):
                        nv = via_index.find_nearest(
                            npad.global_x, npad.global_y,
                            max(npad.size_x, npad.size_y) / 2)
                        if nv is not None:
                            npos = nv
                    strap_cands.append((d, npos))
                strapped = False
                for d, npos in sorted(strap_cands):
                    routing_obs = get_routing_obstacles(pad_layer)
                    route_result = route_via_to_pad(npos, pad, pad_layer, net_id,
                                                    routing_obs, config, verbose=verbose,
                                                    return_blocked_cells=True,
                                                    router=via_pad_router)
                    if route_result.success and route_result.segments:
                        new_segments.extend(route_result.segments)
                        traces_added += len(route_result.segments)
                        pads_strapped += 1
                        print(f"strapped to same-net pad at ({npos[0]:.2f}, {npos[1]:.2f}), "
                              f"{d:.2f}mm away, {len(route_result.segments)} segment(s), no via (#349)")
                        processed_pad_ids.add(current_pad_key)
                        strapped = True
                        break
                if strapped:
                    continue

            # Next, try to place via within pad boundary (preferred - no trace needed)
            # This avoids creating long traces that block other signals
            # Search from pad center outward within pad bounds
            # Skipped when same_net_pad_clearance >= 0 (caller wants vias outside same-net pads).
            pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)
            _phw, _phh = pad_rect_halfspan(pad)  # rotated-rect bbox bound
            pad_half_w_grid = max(1, coord.to_grid_dist(_phw))
            pad_half_h_grid = max(1, coord.to_grid_dist(_phh))
            _rotated = bool(getattr(pad, 'rect_rotation', 0.0))

            via_in_pad = None
            if same_net_pad_clearance >= 0:
                pass  # via-in-pad disabled by same_net_pad_clearance
            # Check pad center first
            elif not obstacles.is_via_blocked(pad_gx, pad_gy):
                via_in_pad = (pad.global_x, pad.global_y)
            else:
                # Spiral search within pad boundary for closest unblocked position
                max_r = max(pad_half_w_grid, pad_half_h_grid)
                for r in range(1, max_r + 1):
                    if via_in_pad:
                        break
                    for dx in range(-r, r + 1):
                        if via_in_pad:
                            break
                        for dy in range(-r, r + 1):
                            if abs(dx) != r and abs(dy) != r:
                                continue  # Only check ring edge
                            # Check if within pad bbox
                            if abs(dx) > pad_half_w_grid or abs(dy) > pad_half_h_grid:
                                continue
                            gx, gy = pad_gx + dx, pad_gy + dy
                            bx, by = gx * config.grid_step, gy * config.grid_step
                            if _rotated and not point_in_pad_rect(bx, by, pad):
                                continue  # outside the (rotated) pad copper
                            if not obstacles.is_via_blocked(gx, gy):
                                # Convert grid back to world coordinates
                                via_in_pad = (bx, by)
                                break

            if via_in_pad:
                # Board-edge clamp: the pad-centre / in-pad via site is placed at
                # its true (off-grid) coordinate, which can sit sub-grid closer to
                # the edge than the obstacle map's grid keep-out blocks. Pull it
                # interior to honor min_copper_edge_clearance while staying inside
                # the pad copper (inert without an edge rule / when it already
                # clears). See plane_pad_tap.clamp_tap_via_to_edge.
                via_in_pad, _ = clamp_tap_via_to_edge(
                    via_in_pad, pad, pcb_data, config, via_size)
                # Found position within pad - place via there (no trace needed)
                # KiCad vias only specify start/end layers, not intermediate
                new_vias.append({
                    'x': via_in_pad[0], 'y': via_in_pad[1],
                    'size': via_size, 'drill': via_drill,
                    'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                })
                available_vias.append(via_in_pad)
                via_index.add(via_in_pad[0], via_in_pad[1])
                vias_placed += 1
                # Block this via position for hole-to-hole clearance
                block_via_position(obstacles, via_in_pad[0], via_in_pad[1], coord,
                                   hole_to_hole_clearance, via_drill,
                                   via_size, config.clearance)
                if via_in_pad == (pad.global_x, pad.global_y):
                    print(f"placed via at pad center (no trace needed)")
                else:
                    print(f"placed via at ({via_in_pad[0]:.2f}, {via_in_pad[1]:.2f}) within pad (no trace needed)")
                processed_pad_ids.add(current_pad_key)
                continue  # Move to next pad

            # Pad center blocked - check if there's an existing via nearby to reuse
            existing_via = via_index.find_nearest(pad.global_x, pad.global_y, max_via_reuse_radius)

            if existing_via:
                # Try to reuse existing via - route trace to connect
                via_pos = existing_via
                reuse_success = False

                if pad_layer:
                    routing_obs = get_routing_obstacles(pad_layer)
                    route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                       routing_obs, config, verbose=verbose,
                                                       return_blocked_cells=True, router=via_pad_router)
                    trace_segments = route_result.segments if route_result.success else None
                    if trace_segments is None:
                        # Routing to existing via failed - fall back to placing new via
                        print(f"can't route to existing via, ", end="")
                        existing_via = None  # Trigger new via placement below
                    elif trace_segments:
                        new_segments.extend(trace_segments)
                        traces_added += len(trace_segments)
                        vias_reused += 1
                        reuse_success = True
                        dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5
                        print(f"reused existing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {dist:.2f}mm away, routed {len(trace_segments)} segments to pad")
                    else:
                        vias_reused += 1
                        reuse_success = True
                        print(f"reused via at pad center")
                else:
                    vias_reused += 1
                    reuse_success = True
                    print(f"reused existing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")

                if reuse_success:
                    processed_pad_ids.add(current_pad_key)
                    continue  # Move to next pad

            # Need to place a new via (pad center blocked, and either no existing via or reuse failed)
            routing_obs = get_routing_obstacles(pad_layer) if pad_layer else None
            failed_route_positions: Set[Tuple[int, int]] = set()  # Track failed positions for this pad
            via_pos = find_via_position(
                pad, obstacles, coord, max_search_radius,
                routing_obstacles=routing_obs,
                config=config,
                pad_layer=pad_layer,
                net_id=net_id,
                verbose=verbose,
                failed_route_positions=failed_route_positions,
                pending_pads=pending_pads,
                router=via_pad_router
            )

            # Board-edge clamp: an in-pad via placed at the pad's true (off-grid)
            # centre can sit sub-grid closer to the edge than the obstacle map's
            # grid keep-out blocks; pull it interior to honor
            # min_copper_edge_clearance (inert without an edge rule / when it
            # already clears). See plane_pad_tap.clamp_tap_via_to_edge.
            edge_moved = False
            if via_pos:
                via_pos, edge_moved = clamp_tap_via_to_edge(
                    via_pos, pad, pcb_data, config, via_size)

            placement_success = False
            trace_segments = None
            via_blocked = via_pos is None
            blocked_cells = []

            if via_pos:
                # An edge-clamped via stays inside the pad copper, so it still
                # connects by overlap (via-in-pad, no trace) even though it is no
                # longer at the exact centre.
                via_at_pad_center = (abs(via_pos[0] - pad.global_x) < 0.001 and
                                     abs(via_pos[1] - pad.global_y) < 0.001) or \
                    (edge_moved and point_in_pad_rect(via_pos[0], via_pos[1], pad, 1e-6))

                if via_at_pad_center:
                    placement_success = True
                elif pad_layer:
                    route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                       routing_obs, config, verbose=verbose,
                                                       return_blocked_cells=True, router=via_pad_router)
                    if route_result.success:
                        trace_segments = route_result.segments
                        placement_success = True
                    else:
                        blocked_cells = route_result.blocked_cells
                else:
                    placement_success = True

            # If fast path failed and rip_blocker_nets enabled, try iterative rip-up
            if not placement_success and rip_blocker_nets:
                print(f"blocked, trying rip-up...", end=" ")
                result = try_place_via_with_ripup(
                    pad, pad_layer, net_id, pcb_data, config, coord,
                    max_search_radius, max_rip_nets,
                    obstacles, routing_obs,
                    via_obstacle_cache, routing_obstacles_cache, all_layers,
                    via_blocked=via_blocked,
                    blocked_cells=blocked_cells,
                    new_vias=new_vias,
                    hole_to_hole_clearance=hole_to_hole_clearance,
                    via_drill=via_drill,
                    protected_net_ids=set(net_ids),  # Protect all nets being routed (don't rip power nets we're routing)
                    verbose=verbose,
                    find_via_position_fn=find_via_position,
                    route_via_to_pad_fn=route_via_to_pad,
                    pending_pads=pending_pads,
                    # Issue #88.1: pass all plane copper placed this run so a
                    # failed rip-up restores only collision-free ripped copper.
                    plane_vias=all_new_vias + new_vias,
                    plane_segments=all_new_segments + new_segments,
                    via_size=via_size,
                    clearance=clearance,
                )

                # Collision-checked restore (#329 route_planes side): every
                # ripped net's kept copper must ALSO be re-emitted as new
                # copper with the net's input copper stripped -- the writer
                # works from the input file, so a pcb_data-only restore would
                # resurrect the dropped (colliding) pieces (#319 board==file).
                # A net left STILL-RIPPED after settle (its restore collided)
                # may have a stale emission from an earlier rip's restore --
                # purge it or the writer ships copper that pcb_data and the
                # obstacle maps no longer track (H9 drilled a via on +1V1's
                # stale-emitted trunk).
                for rid in (result.ripped_net_ids or []):
                    new_segments[:] = [x for x in new_segments if x.get('net_id') != rid]
                    new_vias[:] = [x for x in new_vias if x.get('net_id') != rid]
                    all_new_segments[:] = [x for x in all_new_segments if x.get('net_id') != rid]
                    all_new_vias[:] = [x for x in all_new_vias if x.get('net_id') != rid]
                for rid, ksegs, kvias, _dropped in (result.restored_nets or []):
                    if rid not in ripped_net_ids:
                        ripped_net_ids.append(rid)
                    # A net can be ripped AGAIN by a later pad (its restored
                    # copper is back in pcb_data); purge any earlier emission
                    # of this net or the stale set resurrects pieces the newer
                    # settle dropped (+1V1 ripped at C98.2 then U1.J9: 6 GND
                    # vias landed on a stale-emitted trunk segment).
                    new_segments[:] = [x for x in new_segments if x.get('net_id') != rid]
                    new_vias[:] = [x for x in new_vias if x.get('net_id') != rid]
                    all_new_segments[:] = [x for x in all_new_segments if x.get('net_id') != rid]
                    all_new_vias[:] = [x for x in all_new_vias if x.get('net_id') != rid]
                    # from_restore: settle already put these OBJECTS back in
                    # pcb_data; the net-end sync must not add them again or
                    # every re-rip doubles the net's copper (dup drill pairs).
                    for ks in ksegs:
                        new_segments.append({
                            'start': (ks.start_x, ks.start_y),
                            'end': (ks.end_x, ks.end_y),
                            'width': ks.width, 'layer': ks.layer, 'net_id': rid,
                            'from_restore': True})
                    for kv in kvias:
                        new_vias.append({
                            'x': kv.x, 'y': kv.y, 'size': kv.size,
                            'drill': kv.drill, 'layers': kv.layers, 'net_id': rid,
                            'from_restore': True})
                if result.success:
                    # Track ripped nets and remove their vias/segments from all_new_* lists
                    for rid in result.ripped_net_ids:
                        if rid not in ripped_net_ids:
                            ripped_net_ids.append(rid)
                        # Remove ripped net's vias and segments from accumulator lists
                        # (they were removed from pcb_data during rip-up)
                        all_new_vias[:] = [v for v in all_new_vias if v['net_id'] != rid]
                        all_new_segments[:] = [s for s in all_new_segments if s['net_id'] != rid]
                    # Note: obstacle maps were already updated incrementally in try_place_via_with_ripup

                    # Add via
                    new_vias.append({
                        'x': result.via_pos[0], 'y': result.via_pos[1],
                        'size': via_size, 'drill': via_drill,
                        'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                    })
                    vias_placed += 1
                    processed_pad_ids.add(current_pad_key)
                    available_vias.append(result.via_pos)
                    via_index.add(result.via_pos[0], result.via_pos[1])
                    new_segments.extend(result.segments)
                    traces_added += len(result.segments)
                    block_via_position(obstacles, result.via_pos[0], result.via_pos[1], coord,
                                       hole_to_hole_clearance, via_drill,
                                   via_size, config.clearance)
                    print(f"{GREEN}placed via at ({result.via_pos[0]:.2f}, {result.via_pos[1]:.2f}) after ripping {len(result.ripped_net_ids)} nets{RESET}")
                else:
                    failed_pads += 1
                    failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))
                    for rid in result.ripped_net_ids:
                        if rid not in ripped_net_ids:
                            ripped_net_ids.append(rid)
                    # Issue #88.1: result.ripped_net_ids is now non-empty when a
                    # net was left ripped (not restored) because restoring it
                    # would short onto plane copper. Those nets are excluded from
                    # output and re-routed; restored (collision-free) nets are
                    # absent from the list as before.
                    print(f"{RED}FAILED{RESET}")

            elif placement_success:
                # Fast path succeeded
                via_at_pad_center = (abs(via_pos[0] - pad.global_x) < 0.001 and
                                     abs(via_pos[1] - pad.global_y) < 0.001)
                new_vias.append({
                    'x': via_pos[0], 'y': via_pos[1],
                    'size': via_size, 'drill': via_drill,
                    'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
                })
                vias_placed += 1
                processed_pad_ids.add(current_pad_key)
                available_vias.append(via_pos)
                via_index.add(via_pos[0], via_pos[1])
                if trace_segments:
                    new_segments.extend(trace_segments)
                    traces_added += len(trace_segments)
                block_via_position(obstacles, via_pos[0], via_pos[1], coord,
                                   hole_to_hole_clearance, via_drill,
                                   via_size, config.clearance)

                if via_at_pad_center:
                    print(f"placed via at pad center (no trace needed)")
                else:
                    print(f"placed via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), routed {len(trace_segments) if trace_segments else 0} segments to pad")

            elif not rip_blocker_nets:
                # Fast path failed, no rip-up enabled - try fallback via reuse
                fallback_via = via_index.find_nearest(pad.global_x, pad.global_y, max_search_radius)
                if fallback_via:
                    via_pos = fallback_via
                    if pad_layer:
                        routing_obs = get_routing_obstacles(pad_layer)
                        trace_segments = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                                           routing_obs, config, verbose=verbose,
                                                           router=via_pad_router)
                        if trace_segments is None:
                            print(f"{RED}ROUTING FAILED{RESET}")
                            failed_pads += 1
                            failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))
                        elif trace_segments:
                            new_segments.extend(trace_segments)
                            traces_added += len(trace_segments)
                            vias_reused += 1
                            processed_pad_ids.add(current_pad_key)
                            print(f"reused fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), routed {len(trace_segments)} segments to pad")
                        else:
                            vias_reused += 1
                            processed_pad_ids.add(current_pad_key)
                            print(f"reused fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
                    else:
                        vias_reused += 1
                        processed_pad_ids.add(current_pad_key)
                        print(f"reused fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
                else:
                    print(f"{RED}FAILED - no valid position{RESET}")
                    failed_pads += 1
                    failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))
            else:
                print(f"{RED}FAILED - no valid position{RESET}")
                failed_pads += 1
                failed_pad_infos.append((net_id, net_name, plane_layer, pad_info))

        # Step 9.5: Fine-pitch retry pass (issue #104). Pads whose tap failed
        # at the run parameters get one scoped retry with fine parameters
        # (grid 0.05 / clearance 0.15 / track <= 0.15, verified on
        # castor_pollux) if they are fine-pitch: a same-component neighbor
        # pad within 0.65mm or pad min dimension < 0.35mm. Obstacle maps for
        # the retry are built on a small window around the pad, so the fine
        # grid stays cheap even on large boards.
        net_failed = failed_pad_infos[net_failed_start:]
        fine_candidates = [e for e in net_failed
                           if pad_is_fine_pitch(e[3]['pad'], pcb_data)]
        if fine_candidates:
            _fab_clear, _fab_track = fab_floor_clearance_track(pcb_data)
            print(f"\nRetrying {len(fine_candidates)} failed fine-pitch pad(s) with scoped "
                  f"fine parameters (grid {FINE_TAP_GRID_STEP}mm, clearance stepped down "
                  f"to the fab floor {_fab_clear}mm, track >= {_fab_track}mm):")
            if progress_callback:
                progress_callback(0, 0, f"{net_name}: fine-pitch retry "
                                        f"({len(fine_candidates)} pad(s))...")
            recovered_entries = []
            for entry in fine_candidates:
                pad_info = entry[3]
                pad = pad_info['pad']
                pad_layer = pad_info.get('pad_layer')
                current_pad_key = (pad.global_x, pad.global_y)
                if current_pad_key in processed_pad_ids:
                    continue
                pending_pads = [
                    pp for pp in base_pending_pads
                    if (pp['pad'].global_x, pp['pad'].global_y) != current_pad_key
                    and (pp['pad'].global_x, pp['pad'].global_y) not in processed_pad_ids
                ]
                print(f"  Pad {pad.component_ref}.{pad.pad_number} (fine retry)...", end=" ", flush=True)
                result = tap_pad_with_escalation(
                    pad, pad_layer, net_id, pcb_data, config,
                    max_search_radius, via_size, via_drill,
                    same_net_pad_clearance=same_net_pad_clearance,
                    pending_pads=pending_pads,
                    extra_vias=new_vias,
                    extra_segments=new_segments,
                    verbose=verbose,
                    try_default=False,  # run params already failed for this pad
                )
                if result.success:
                    if result.via is not None:
                        new_vias.append(result.via)
                        vias_placed += 1
                        available_vias.append((result.via['x'], result.via['y']))
                        via_index.add(result.via['x'], result.via['y'])
                        block_via_position(obstacles, result.via['x'], result.via['y'],
                                           coord, hole_to_hole_clearance, via_drill,
                                   via_size, config.clearance)
                    else:
                        vias_reused += 1
                    if result.segments:
                        new_segments.extend(result.segments)
                        traces_added += len(result.segments)
                    processed_pad_ids.add(current_pad_key)
                    failed_pads -= 1
                    recovered_entries.append(entry)
                    if result.via is not None:
                        print(f"{GREEN}placed via at ({result.via['x']:.2f}, {result.via['y']:.2f}), "
                              f"{len(result.segments)} trace segment(s){RESET}")
                    else:
                        print(f"{GREEN}reused via at ({result.reused_via_pos[0]:.2f}, "
                              f"{result.reused_via_pos[1]:.2f}), {len(result.segments)} trace segment(s){RESET}")
                else:
                    print(f"{RED}STILL FAILED{RESET}")
            for entry in recovered_entries:
                failed_pad_infos.remove(entry)
            if recovered_entries:
                print(f"  Fine-pitch retry recovered {len(recovered_entries)}/{len(fine_candidates)} pad(s)")

        # Ref-count integrity audit of this net's via-placement map (#309, same
        # class as #208): after all rips/placements the maintained map must
        # equal a fresh rebuild from the CURRENT pcb_data plus the session vias
        # (mirroring its Step-7 construction + per-placement blocking).
        if os.environ.get("KICAD_OBSTACLE_AUDIT") and obstacles is not None:
            _audit_plane_via_map(obstacles, pcb_data, config, net_id,
                                 same_net_pad_clearance,
                                 all_new_vias + new_vias, coord,
                                 hole_to_hole_clearance, via_drill, via_size,
                                 net_name)

        # Step 10: Generate zone for this net (if needed)
        # For multi-net layers, defer zone generation until all nets are processed
        zone_sexpr = None
        is_multi_net_layer = layer_nets and len(layer_nets.get(plane_layer, [])) > 1
        if should_create_zone and not is_multi_net_layer:
            # Single-net layer: use full board rectangle
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=plane_layer,
                polygon_points=zone_polygon,
                clearance=zone_clearance,
                min_thickness=min_thickness,
                direct_connect=True,
                use_net_name=pcb_data.kicad_version >= KICAD_10_MIN_VERSION
            )
            all_zone_sexprs.append(zone_sexpr)
            all_zone_data.append({
                'net_id': net_id,
                'net_name': net_name,
                'layer': plane_layer,
                'polygon_points': zone_polygon,
                'clearance': zone_clearance,
                'min_thickness': min_thickness,
            })

            # Calculate and print resistance for single-net layer
            result = analyze_single_net_plane(zone_polygon, plane_layer)
            print_single_net_resistance(result, net_name)

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
        print(f"  Existing vias reused: {vias_reused}")
        if pads_strapped > 0:
            print(f"  Pads strapped to adjacent same-net pads (no via, #349): {pads_strapped}")
        print(f"  Traces added: {traces_added}")
        if failed_pads > 0:
            print(f"  Failed pads: {failed_pads}")

        # KICAD_PLANE_MAP_PARITY=1: verify the incremental via map still
        # BLOCKS every restored net's live copper (under-blocking here is the
        # via-drilled-on-restored-track class; over-blocking is by-design
        # conservative for dropped pieces). Fresh-recompute each restored
        # net's footprint from CURRENT pcb_data and probe the map.
        if os.environ.get('KICAD_PLANE_MAP_PARITY') and ripped_net_ids:
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
        total_vias_reused += vias_reused
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
                    verbose=verbose
                )
                all_zone_sexprs.extend(zone_sexprs)
                all_debug_lines.extend(debug_line_sexprs)
                all_zone_data.extend(zone_data)

    # Print overall totals only if multiple nets were processed
    if len(net_names) > 1:
        print(f"\n{'='*60}")
        print(f"OVERALL TOTALS")
        print(f"{'='*60}")
        print(f"  Nets processed: {len(net_names)}")
        print(f"  Total new vias placed: {total_vias_placed}")
        print(f"  Total existing vias reused: {total_vias_reused}")
        print(f"  Total traces added: {total_traces_added}")
        if total_failed_pads > 0:
            print(f"  Total failed pads: {total_failed_pads}")

        if all_ripped_net_ids:
            ripped_names = []
            for rid in all_ripped_net_ids:
                net = pcb_data.nets.get(rid)
                ripped_names.append(net.name if net else f"net_{rid}")
            print(f"  Nets excluded from output: {', '.join(ripped_names)}")

    # Finalize plane tap copper ONCE, before the write/dry-run split, so the
    # GUI (dry_run=True, return_results) and the CLI (writes the file) emit
    # identical copper for identical inputs: neck grazes -> graze prune /
    # dead-end sweep -> close soft joints (#334 + follow-up). Previously this
    # lived inside _write_output_and_reroute and the GUI path never ran it.
    if progress_callback:
        progress_callback(0, 0, "Cleaning up plane tap copper...")
    all_new_segments = _finalize_plane_copper(
        all_new_segments, all_new_vias, pcb_data, clearance, all_layers,
        track_width, grid_step, via_size, via_drill, hole_to_hole_clearance)

    geo_results: Dict[int, Dict] = {}
    if dry_run:
        print("\nDry run - no output file written")
    else:
        if os.environ.get('KICAD_SETTLE_DEBUG'):
            from collections import Counter as _C
            _dups = {k: n for k, n in _C((v.get('net_id'), round(v['x'], 3), round(v['y'], 3))
                                          for v in all_new_vias).items() if n > 1}
            print(f"\n  WRITE-DEBUG: duplicate new_vias: {_dups}")
        _write_output_and_reroute(
            input_file=input_file,
            output_file=output_file,
            all_zone_sexprs=all_zone_sexprs,
            all_debug_lines=all_debug_lines,
            all_new_vias=all_new_vias,
            all_new_segments=all_new_segments,
            all_ripped_net_ids=all_ripped_net_ids,
            zones_to_replace=zones_to_replace,
            pcb_data=pcb_data,
            reroute_ripped_nets=reroute_ripped_nets,
            all_layers=all_layers,
            plane_layers=plane_layers,
            track_width=track_width,
            clearance=clearance,
            via_size=via_size,
            via_drill=via_drill,
            grid_step=grid_step,
            hole_to_hole_clearance=hole_to_hole_clearance,
            verbose=verbose,
            power_nets=power_nets,
            power_nets_widths=power_nets_widths,
            add_teardrops=add_teardrops,
            no_bga_zone=no_bga_zone,
            clamp_netclasses=clamp_netclasses,
            clearance_ceiling=clearance_ceiling
        )

        # Geometric truth check (issues #89 and #107): the via-placement
        # counters above count a pad "done" when a via is placed/reused, but
        # that via may not be electrically joined to the net's plane (#89), and
        # multi-net Voronoi layers can silently skip TH pads that land in the
        # other net's cell (#107). Re-parse the written output and report how
        # many pads are actually connected so the summary matches geometry.
        geo_results = _geometric_plane_verification(
            output_file, net_ids, net_names, plane_layers, all_ripped_net_ids)
        if geo_results:
            geo_failed = sum(info['failed'] for info in geo_results.values())
            if geo_failed != total_failed_pads:
                print(f"\n  NOTE: via-placement counters reported "
                      f"{total_failed_pads} failed pad(s), but geometric check "
                      f"found {geo_failed} pad(s) not connected to their plane "
                      f"(see per-net breakdown above).")

    if progress_callback:
        progress_callback(1, 1, "Plane creation complete")
    if return_results:
        # GUI parity with the CLI's in-run ripped-net reconnect (#347): the
        # file path above reroutes verified-broken casualties after writing;
        # the GUI (dry_run) path reconnects them here, in memory, exactly
        # like route_disconnected_planes' return_results block. pcb_data has
        # the ripped copper removed and this run's copper appended, so
        # batch_route routes against the live in-memory board; the new copper
        # is merged into the emit lists (the applier deletes the ripped nets'
        # originals first, then applies these).
        if all_ripped_net_ids:
            _broken = []
            try:
                from check_connected import check_net_connectivity
                _segs_by, _vias_by, _zones_by = {}, {}, {}
                for _s in pcb_data.segments:
                    _segs_by.setdefault(_s.net_id, []).append(_s)
                for _v in pcb_data.vias:
                    _vias_by.setdefault(_v.net_id, []).append(_v)
                for _z in getattr(pcb_data, 'zones', None) or []:
                    _zones_by.setdefault(_z.net_id, []).append(_z)
                for _rid in all_ripped_net_ids:
                    _res = check_net_connectivity(
                        _rid, _segs_by.get(_rid, []), _vias_by.get(_rid, []),
                        pcb_data.pads_by_net.get(_rid, []), _zones_by.get(_rid, []))
                    if not _res.get('connected', False):
                        _broken.append(_rid)
            except Exception:
                _broken = list(all_ripped_net_ids)
            _cnames = [pcb_data.nets[n].name for n in _broken if n in pcb_data.nets]
            if _cnames:
                print(f"\nReconnecting {len(_cnames)} net(s) this run ripped "
                      f"for plane vias (in-memory): {', '.join(_cnames)}")
                if progress_callback:
                    progress_callback(0, 0, f"Reconnecting {len(_cnames)} ripped net(s)...")
                try:
                    from route import batch_route
                    _board_layers = list(getattr(pcb_data.board_info,
                                                 'copper_layers', None) or [])
                    _ok, _fail, _t, _rdata = batch_route(
                        input_file, "", _cnames,
                        layers=_board_layers or list(set(all_layers + plane_layers)),
                        track_width=track_width, clearance=clearance,
                        via_size=via_size, via_drill=via_drill,
                        grid_step=grid_step,
                        hole_to_hole_clearance=hole_to_hole_clearance,
                        power_nets=power_nets,
                        power_nets_widths=power_nets_widths,
                        disable_bga_zones=([] if no_bga_zone else None),
                        net_clearances=net_clearances,  # #434 cross-class
                        return_results=True, pcb_data=pcb_data)

                    def _sd(_s):
                        return {'start': (_s.start_x, _s.start_y),
                                'end': (_s.end_x, _s.end_y),
                                'width': _s.width, 'layer': _s.layer,
                                'net_id': _s.net_id}

                    def _vd(_v):
                        return {'x': _v.x, 'y': _v.y, 'size': _v.size,
                                'drill': _v.drill, 'layers': _v.layers,
                                'net_id': _v.net_id}
                    for _r in _rdata.get('results', []):
                        all_new_segments.extend(
                            _sd(_s) for _s in (_r.get('new_segments') or []))
                        all_new_vias.extend(
                            _vd(_v) for _v in (_r.get('new_vias') or []))
                    all_new_vias.extend(
                        _vd(_v) for _v in (_rdata.get('all_swap_vias') or []))
                    all_new_segments.extend(
                        _sd(_s) for _s in (_rdata.get('all_swap_segments') or []))
                    if _fail:
                        print(f"  {_fail} ripped net(s) could NOT be reconnected "
                              f"-- their board copper is deleted by the applier "
                              f"and they need the routing tab")
                except Exception as _e:
                    print(f"  in-memory ripped-net reconnect failed: {_e}")
        # all_ripped_net_ids LAST for backward compatibility: the GUI must
        # delete these nets' existing board copper before applying new_vias/
        # new_segments (which include the from_restore replacement pieces) --
        # the CLI writer's exclude_net_ids strip has no pcbnew equivalent, so
        # without this the live board keeps the originals AND gains the
        # emitted copies (duplicated restored copper).
        return (total_vias_placed, total_traces_added, total_pads_needing_vias,
                all_new_vias, all_new_segments, all_zone_data, total_failed_pads,
                all_ripped_net_ids)
    return (total_vias_placed, total_traces_added, total_pads_needing_vias)


def main():
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set
    parser = argparse.ArgumentParser(
        description="Create copper zones with via stitching to net pads",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Single net:
    python route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu

    # Multiple nets (each net paired with corresponding plane layer):
    python route_planes.py input.kicad_pcb output.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu

    # With rip-up and automatic re-routing:
    python route_planes.py input.kicad_pcb output.kicad_pcb --nets GND VCC --plane-layers In1.Cu In2.Cu --rip-blocker-nets --reroute-ripped-nets
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output KiCad PCB file (default: input_routed.kicad_pcb)")
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
                        help="Plane layer(s) for the zone(s), one per net (e.g., In1.Cu In2.Cu)")

    # Via and track geometry
    parser.add_argument("--via-size", type=float, default=None, help="Via outer diameter in mm (default: the board Default net-class via, else 0.5)")
    parser.add_argument("--via-drill", type=float, default=None, help="Via drill size in mm (default: the board Default net-class via drill, else 0.3)")
    parser.add_argument("--track-width", type=float, default=None, help="Track width for via-to-pad connections in mm (default: the board Default net-class width, else 0.3)")
    parser.add_argument("--clearance", type=float, default=None, help="Copper clearance CEILING in mm. When given, every net class (Default included) is capped at min(class, this) and the writeback clamps. When OMITTED, each net routes at its own net-class clearance (base = the board's Default class, else 0.25).")

    # Zone options
    parser.add_argument("--zone-clearance", type=float, default=defaults.PLANE_ZONE_CLEARANCE, help="Zone clearance from other copper in mm (default: 0.2)")
    parser.add_argument("--min-thickness", type=float, default=defaults.PLANE_MIN_THICKNESS, help="Minimum zone copper thickness in mm (default: 0.1)")

    # Algorithm options
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP, help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--max-search-radius", type=float, default=defaults.PLANE_MAX_SEARCH_RADIUS, help="Max radius to search for valid via position in mm (default: 10.0)")
    parser.add_argument("--max-via-reuse-radius", type=float, default=defaults.PLANE_MAX_VIA_REUSE_RADIUS, help="Max radius to reuse existing via instead of placing new one in mm (default: 1.0)")
    parser.add_argument("--close-via-radius", type=float, default=None, help="Radius to check for nearby vias before placing new one (default: 2.5 * via-size)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=None, help=f"Minimum clearance between drill holes in mm (default: the board min_hole_to_hole, else {defaults.HOLE_TO_HOLE_CLEARANCE})")
    parser.add_argument("--layers", "-l", nargs="+", default=None,
                        help="All copper layers for routing and via span (default: F.Cu + plane-layers + B.Cu)")
    parser.add_argument("--layer-costs", nargs="+", type=float, default=[],
                        help="Per-layer routing cost multipliers (1.0-1000, or any negative value e.g. -1 = "
                             "forbidden: the layer stays an obstacle / via span but gets no routed copper). "
                             "Order matches --layers. Example: --layer-costs 1.0 -1 -1 3.0")

    # Blocker rip-up options
    parser.add_argument("--rip-blocker-nets", action="store_true",
                        help="Identify and rip up nets blocking via placement, then retry. Ripped nets are excluded from output.")
    parser.add_argument("--max-rip-nets", type=int, default=defaults.PLANE_MAX_RIP_NETS,
                        help="Maximum number of blocker nets to rip up (default: 3)")
    parser.add_argument("--reroute-ripped-nets", action="store_true",
                        help="Automatically re-route ripped nets after via placement")
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
    # #381 D9: --track-via-clearance is the same constant route_disconnected_planes
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
                        default=defaults.SAME_NET_PAD_CLEARANCE,
                        help="Edge-to-edge clearance (mm) between stitching vias and same-net pads. "
                             "-1 (default) allows via-in-pad placement; any value >= 0 forces vias "
                             "outside same-net pads with that clearance.")

    # Debug options
    parser.add_argument("--dry-run", action="store_true", help="Analyze without writing output")
    parser.add_argument("--skip-existing-zones", action="store_true",
                        help="Keep an existing same-net zone (don't recreate) and only place stitching vias; "
                             "tolerate other-net zones on the same layer (e.g. a GND island under an RF feed)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Print detailed DEBUG messages")
    parser.add_argument("--debug-lines", action="store_true", help="Output MST routes on User.1, User.2, etc. per net")
    parser.add_argument("--add-teardrops", action="store_true", help="Add teardrop settings to all pads in output file")

    # GND return vias
    parser.add_argument("--add-gnd-vias", action="store_true",
        help="Add GND vias near signal vias for return current path")
    parser.add_argument("--gnd-via-net", type=str, default="GND",
        help="Net name for GND vias (default: GND)")
    parser.add_argument("--gnd-via-distance", type=float, default=2.0,
        help="Maximum distance from signal via to place GND via in mm (default: 2.0)")

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    args = parser.parse_args()
    # #439: identical net-class/clearance model to route.py. --clearance is the
    # clamp switch: GIVEN -> ceiling, every class capped at min(class, ceiling),
    # writeback clamps (_clamp_netclasses True). OMITTED -> each net routes at its
    # own class (base = board Default class), classes preserved. Geometry flags
    # omitted -> default from the board (track/clearance/via from Default net-class,
    # hole/edge from board constraints, else a routing_defaults constant). Planes
    # keep their larger PLANE_EDGE_CLEARANCE fallback when the board declares none.
    # Resolved here, before enforce_fab_floors and every downstream use.
    from list_nets import (board_default_netclass_clearance, board_default_netclass_param,
                           board_constraint)
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
    if args.hole_to_hole_clearance is None:
        _h2h = board_constraint(args.input_file, 'min_hole_to_hole')
        args.hole_to_hole_clearance = _h2h if _h2h is not None else defaults.HOLE_TO_HOLE_CLEARANCE
        print(f"--hole-to-hole-clearance not given; using "
              f"{'the board min_hole_to_hole' if _h2h is not None else 'the fallback'} "
              f"{args.hole_to_hole_clearance}mm.")
    if args.board_edge_clearance is None:
        # Planes keep their larger edge keep-out (PLANE_EDGE_CLEARANCE) only when
        # the board declares no edge rule of its own.
        _edge = board_constraint(args.input_file, 'min_copper_edge_clearance')
        args.board_edge_clearance = _edge if _edge is not None else defaults.PLANE_EDGE_CLEARANCE
        print(f"--board-edge-clearance not given; using "
              f"{'the board min_copper_edge_clearance' if _edge is not None else 'the fallback'} "
              f"{args.board_edge_clearance}mm.")
    set_default_fab_tier(*fab_tier_from_args(args))
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

    create_plane(
        input_file=args.input_file,
        output_file=args.output_file,
        net_names=net_names,
        plane_layers=plane_layers,
        via_size=args.via_size,
        via_drill=args.via_drill,
        track_width=args.track_width,
        clearance=args.clearance,
        zone_clearance=args.zone_clearance,
        min_thickness=args.min_thickness,
        grid_step=args.grid_step,
        max_search_radius=args.max_search_radius,
        max_via_reuse_radius=args.max_via_reuse_radius,
        close_via_radius=args.close_via_radius,
        hole_to_hole_clearance=args.hole_to_hole_clearance,
        all_layers=args.layers,
        verbose=args.verbose,
        dry_run=args.dry_run,
        rip_blocker_nets=args.rip_blocker_nets,
        max_rip_nets=args.max_rip_nets,
        reroute_ripped_nets=args.reroute_ripped_nets,
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

    # Add GND return vias if requested
    if args.add_gnd_vias and not args.dry_run:
        from kicad_parser import parse_kicad_pcb
        from routing_config import GridRouteConfig, GridCoord
        from obstacle_map import build_base_obstacle_map
        from add_gnd_vias import add_gnd_vias_to_existing_board
        from kicad_writer import add_tracks_and_vias_to_pcb

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
            hole_to_hole_clearance=args.hole_to_hole_clearance
        )
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
    if not args.dry_run:
        from pcb_modification import clean_plane_copper
        _snapped, _removed = clean_plane_copper(args.output_file, net_names,
                                                args.clearance, args.grid_step)
        if _snapped or _removed:
            print(f"Plane cleanup: closed {_snapped} stub gap(s), trimmed {_removed} dead-end segment(s)")

    # NO KiCad-oracle recheck here (#217): the plane this step just poured has NOT
    # yet been stitched -- tying pads/islands into the pour is route_disconnected_
    # planes' job, the very next step. At the route_planes stage KiCad reports every
    # not-yet-stitched pad as a missing link (hackrf: 26 plane links / 42 total),
    # so an oracle pass here thrashes routing links that don't exist as failures
    # (18 routed, 77 failed, 3 kicad-cli rounds) -- a 2-8x route_planes regression
    # for work route_disconnected_planes does properly. The oracle runs ONCE, as an
    # end-of-pipeline fallback at the end of route_disconnected_planes, on the
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

    # Machine-readable summary so an orchestrator and the next pipeline step can
    # read the clearance this step actually used (mirrors route.py/route_diff.py).
    import json as _json, clearance_ledger as _cl
    _summary = {
        "min_clearance_used": _cl.effective(args.clearance),
    }
    print("JSON_SUMMARY: " + _json.dumps(_summary))


if __name__ == "__main__":
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    main()
