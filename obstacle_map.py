"""
Obstacle map building functions for PCB routing.

Builds GridObstacleMap objects from PCB data, adding obstacles for segments,
vias, pads, BGA exclusion zones, and routed paths.
"""
from __future__ import annotations

from typing import List, Optional, Tuple, Dict, Set, Union
from dataclasses import dataclass, field
import numpy as np
import math

from kicad_parser import PCBData, Segment, Via, Pad, pad_drill_circles, pad_drill_capsule
from routing_config import GridRouteConfig, GridCoord
import routing_defaults as defaults
from routing_utils import build_layer_map, iter_pad_blocked_cells, pad_blocked_cells_array, \
    circle_offsets, segment_blocked_cells_array
from net_queries import expand_pad_layers
from obstacle_costs import add_bga_proximity_costs

# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap
except ImportError:
    # Will fail at runtime if not available
    GridObstacleMap = None


def build_base_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                            nets_to_route: List[int],
                            extra_clearance: float = 0.0,
                            net_clearances: dict = None) -> GridObstacleMap:
    """Build base obstacle map with static obstacles (BGA zones, pads, pre-existing tracks/vias).

    Excludes all nets that will be routed (nets_to_route) - their stubs will be added
    per-net in the routing loop (excluding the current net being routed).

    Args:
        extra_clearance: Additional clearance to add for routing (e.g., for diff pair centerline routing)
        net_clearances: Optional dict mapping net_id to clearance (mm). When building obstacles,
            the effective clearance is max(config.clearance, max(net_clearances.values())).
            This ensures proper spacing when nets have different net class clearance requirements.
    """
    if net_clearances is None:
        net_clearances = {}
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)
    nets_to_route_set = set(nets_to_route)

    # Use the maximum clearance of any net class to ensure proper spacing
    # between nets with different net class clearance requirements
    max_net_clearance = max(net_clearances.values()) if net_clearances else config.clearance
    effective_clearance = max(config.clearance, max_net_clearance)

    obstacles = GridObstacleMap(num_layers)

    # Set BGA proximity radius for is_in_bga_proximity() checks
    bga_prox_radius_grid = coord.to_grid_dist(config.bga_proximity_radius)
    obstacles.set_bga_proximity_radius(bga_prox_radius_grid)

    # Set BGA exclusion zones - block vias AND tracks on ALL layers.
    # set_bga_zone alone enforces this in Rust (is_blocked / is_via_blocked
    # both block in-zone cells unless in allowed_cells). Do NOT also stamp the
    # rectangle into blocked_cells: blocked_cells takes precedence over
    # allowed_cells, so the hard stamp made every allowed-cells window (the
    # +/-10 endpoint windows, the #189 via-in-pad unblock's +/-5) dead code --
    # a boxed pad INSIDE a QFN/BGA zone could never be rescued even with a
    # legally-placed via in it (ottercast Net-(C61-Pad1) under U6).
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)

    # Add BGA proximity costs (penalize routing near BGA edges)
    add_bga_proximity_costs(obstacles, config)

    # Add segments as obstacles (excluding nets we'll route - their stubs added per-net)
    # Use actual segment width for obstacle, and layer-specific width for routing track
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        # Compute expansion: routing track half-width (for this layer) + obstacle half-width + clearance
        layer_track_width = config.get_track_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else layer_track_width
        expansion_mm = layer_track_width / 2 + seg_width / 2 + effective_clearance + extra_clearance
        # For via blocking by segments: via half-size + segment half-width + clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + effective_clearance + extra_clearance
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_mm, via_block_mm)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        # Compute expansion based on actual via size:
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
        # For track blocking by vias: via half-size + max routing track half-width + clearance
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, effective_clearance, extra_clearance)
        # For via-to-via: via size + routing via size + clearance
        via_via_mm = via_size / 2 + config.via_size / 2 + effective_clearance
        # True via-via clearance radius in cells as a FLOAT (no floor): the disc
        # threshold is radius**2, so this blocks exactly the cells within the real
        # clearance. Flooring (to_grid_dist) lost up to ~1 cell and let two vias sit
        # a diagonal cell-offset too close (e.g. (3,2) cells = 0.36mm when 0.39mm is
        # required) -- a real cross-net via-via DRC violation the router never saw.
        via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)
        # diagonal_margin=DIAGONAL_MARGIN to MATCH the per-net cache (_collect_via_obstacles):
        # the base map holds the excluded nets' (GND/P3.3V) fanout vias, and without
        # the diagonal margin a 45deg track grazes them a sub-cell under clearance.
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid,
                          via_via_expansion_grid, diagonal_margin=defaults.DIAGONAL_MARGIN)

    # Add pads as obstacles (excluding nets we'll route - their pads added per-net)
    # Use effective_clearance to ensure proper spacing between nets with different clearance requirements
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance,
                              clearance_override=effective_clearance)

    # Add board edge clearance
    add_board_edge_obstacles(obstacles, pcb_data, config, extra_clearance)

    # Add user-drawn User-layer keepout polygons (issue #27) - block all copper layers
    add_user_keepout_obstacles(obstacles, pcb_data, config, coord, num_layers)

    # Block KiCad native keep-out rule areas (zone (keepout ...)), per-layer (PR #25)
    add_rule_area_keepout_obstacles(obstacles, pcb_data, config)

    # Add hole-to-hole clearance blocking for existing drills
    add_drill_hole_obstacles(obstacles, pcb_data, config, nets_to_route_set,
                             extra_clearance)

    return obstacles


# Cap the size of the (points, edges) float64 broadcast temporaries used by the
# polygon kernels below. Without chunking, a many-vertex board outline times a
# fine grid wants gigabytes in one allocation (issue #81: 432-vertex keyboard
# outline x 2M cells = ~7 GB) and OOMs the machine before routing starts.
_POLY_CHUNK_BYTES = 32 * 1024 * 1024


def _poly_chunk_rows(n_edges: int) -> int:
    """Number of points per chunk so one (chunk, n_edges) float64 fits the cap."""
    return max(1024, _POLY_CHUNK_BYTES // (8 * max(n_edges, 1)))


def _points_inside_polygon(px, py, x1, y1, x2, y2):
    """Even-odd ray cast of points against polygon edges, chunked over points.

    px/py: (N,) point coords in mm. x1/y1/x2/y2: (E,) edge endpoint arrays.
    Returns (N,) bool.
    """
    n = px.shape[0]
    inside = np.empty(n, dtype=bool)
    safe_dy = np.where(y2 - y1 == 0, 1.0, y2 - y1)
    chunk = _poly_chunk_rows(x1.shape[0])
    for s in range(0, n, chunk):
        e = min(s + chunk, n)
        px_col = px[s:e, np.newaxis]
        py_col = py[s:e, np.newaxis]
        cond_y = (y1 > py_col) != (y2 > py_col)
        x_intercept = (x2 - x1) * (py_col - y1) / safe_dy + x1
        inside[s:e] = (np.sum(cond_y & (px_col < x_intercept), axis=1) % 2) == 1
    return inside


def _points_edge_distance(px, py, x1, y1, x2, y2):
    """Min distance from each point to any polygon edge, chunked over points.

    px/py: (N,) point coords in mm. x1/y1/x2/y2: (E,) edge endpoint arrays.
    Returns (N,) float64 distances (vertex distance for zero-length edges).
    """
    n = px.shape[0]
    out = np.empty(n, dtype=np.float64)
    dx_e, dy_e = x2 - x1, y2 - y1
    seg_len_sq = dx_e * dx_e + dy_e * dy_e
    safe_len_sq = np.where(seg_len_sq < 1e-10, 1.0, seg_len_sq)
    degen = seg_len_sq < 1e-10
    any_degen = bool(degen.any())
    chunk = _poly_chunk_rows(x1.shape[0])
    for s in range(0, n, chunk):
        e = min(s + chunk, n)
        px_col = px[s:e, np.newaxis]
        py_col = py[s:e, np.newaxis]
        t = np.clip(((px_col - x1) * dx_e + (py_col - y1) * dy_e) / safe_len_sq, 0.0, 1.0)
        closest_x = x1 + t * dx_e
        closest_y = y1 + t * dy_e
        dist_sq = (px_col - closest_x) ** 2 + (py_col - closest_y) ** 2
        if any_degen:
            degen_dist_sq = (px_col - x1) ** 2 + (py_col - y1) ** 2
            dist_sq = np.where(degen, degen_dist_sq, dist_sq)
        out[s:e] = np.sqrt(np.min(dist_sq, axis=1))
    return out


def _rasterize_polygon(poly_points, coord: GridCoord, margin: float, clip_bounds=None):
    """Rasterize a closed polygon over its grid bounding box (expanded by `margin` mm).

    ``clip_bounds`` (min_x, min_y, max_x, max_y) restricts the rasterized region
    to the obstacle map's actual extent. Without it, a large polygon -- e.g. a
    whole-board ring keep-out -- rasterizes the entire board on EVERY build, even
    a tiny local-window build, dominating route_disconnected_planes and the
    via-in-pad unblock (the map only covers the window, so cells outside it are
    never blocked anyway). Clipping is a pure optimisation: no cell inside the map
    changes. A full-board build passes the board bounds, so nothing is clipped.

    Shared geometry kernel for the polygon obstacle passes (board cutouts, KiCad
    keep-out rule areas, and user-drawn keepout zones). Returns four parallel
    numpy arrays for the candidate cells:
        gx_flat, gy_flat : int32 grid coordinates
        inside           : bool, cell centre inside the polygon (even-odd ray cast)
        edge_dist        : float, mm distance from the cell centre to the nearest edge
    Returns ``(None, None, None, None)`` if the polygon is degenerate (< 3 points)
    or the bounding box is empty. Callers threshold ``edge_dist`` by their own
    clearance to decide which cells to block.
    """
    if len(poly_points) < 3:
        return None, None, None, None
    poly = np.array(poly_points, dtype=np.float64)
    x1 = poly[:, 0]
    y1 = poly[:, 1]
    x2 = np.roll(poly[:, 0], -1)
    y2 = np.roll(poly[:, 1], -1)

    cmin_x, cmax_x = poly[:, 0].min() - margin, poly[:, 0].max() + margin
    cmin_y, cmax_y = poly[:, 1].min() - margin, poly[:, 1].max() + margin
    if clip_bounds is not None:
        cmin_x = max(cmin_x, clip_bounds[0]); cmin_y = max(cmin_y, clip_bounds[1])
        cmax_x = min(cmax_x, clip_bounds[2]); cmax_y = min(cmax_y, clip_bounds[3])
        if cmin_x > cmax_x or cmin_y > cmax_y:
            return None, None, None, None  # polygon doesn't overlap the map
    gx_lo, gy_lo = coord.to_grid(cmin_x, cmin_y)
    gx_hi, gy_hi = coord.to_grid(cmax_x, cmax_y)
    gx_range = np.arange(gx_lo, gx_hi + 1, dtype=np.int32)
    gy_range = np.arange(gy_lo, gy_hi + 1, dtype=np.int32)
    if gx_range.size == 0 or gy_range.size == 0:
        return None, None, None, None

    gx_grid, gy_grid = np.meshgrid(gx_range, gy_range)
    gx_flat = gx_grid.ravel()
    gy_flat = gy_grid.ravel()
    px = gx_flat.astype(np.float64) * coord.grid_step
    py = gy_flat.astype(np.float64) * coord.grid_step

    # Both kernels are chunked over points to bound the (points, edges)
    # broadcast temporaries (issue #81).
    inside = _points_inside_polygon(px, py, x1, y1, x2, y2)
    edge_dist = _points_edge_distance(px, py, x1, y1, x2, y2)

    return gx_flat, gy_flat, inside, edge_dist


def _block_cells_on_layers(obstacles: GridObstacleMap, gx_flat, gy_flat, mask, layer_idxs):
    """Block the masked (gx, gy) cells on each layer in `layer_idxs`."""
    if not mask.any():
        return
    cells = np.column_stack([gx_flat[mask], gy_flat[mask]])
    for li in layer_idxs:
        layer_col = np.full((cells.shape[0], 1), li, dtype=np.int32)
        obstacles.add_blocked_cells_batch(np.hstack([cells, layer_col]))


def _polygon_grid_cells(points_mm, coord: GridCoord):
    """Return the set of (gx, gy) grid cells whose centre is inside the polygon."""
    gx_flat, gy_flat, inside, _ = _rasterize_polygon(points_mm, coord, margin=0.0)
    if gx_flat is None:
        return set()
    return set(zip(gx_flat[inside].tolist(), gy_flat[inside].tolist()))


def add_user_keepout_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               config: GridRouteConfig, coord: GridCoord, num_layers: int):
    """Block all grid cells inside user-drawn keepout polygons on every copper layer.

    Keepout zones (issue #27) are hard blocks: routed tracks cannot enter them. The
    block applies to every net routed in the run. Because cells are blocked
    unconditionally, a zone drawn over a routed net's pad can make that net
    unroutable -- zones are meant for open board area, not over pads.
    """
    if not config.keepout_enabled or not pcb_data.keepout_zones:
        return

    all_cells = set()
    for zone in pcb_data.keepout_zones:
        all_cells |= _polygon_grid_cells(zone.points, coord)

    if not all_cells:
        return

    xy_arr = np.array(sorted(all_cells), dtype=np.int32)
    for layer_idx in range(num_layers):
        layer_col = np.full((xy_arr.shape[0], 1), layer_idx, dtype=np.int32)
        obstacles.add_blocked_cells_batch(np.hstack([xy_arr, layer_col]))
    # Block vias inside the zone too (vias span all layers)
    obstacles.add_blocked_vias_batch(xy_arr)


def point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    """Test if a point is inside a polygon using ray casting algorithm.

    Args:
        x, y: Point coordinates
        polygon: List of (x, y) vertices defining the polygon

    Returns:
        True if point is inside the polygon
    """
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        # Check if ray from point crosses this edge
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i

    return inside


def point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate minimum distance from point to line segment.

    Args:
        px, py: Point coordinates
        x1, y1, x2, y2: Line segment endpoints

    Returns:
        Minimum distance from point to the line segment
    """
    # Vector from p1 to p2
    dx = x2 - x1
    dy = y2 - y1

    # Handle degenerate case (segment is a point)
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-10:
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

    # Project point onto line, clamping to segment
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))

    # Find closest point on segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


def point_to_polygon_edge_distance(x: float, y: float, polygon: List[Tuple[float, float]]) -> float:
    """Calculate minimum distance from point to any polygon edge.

    Args:
        x, y: Point coordinates
        polygon: List of (x, y) vertices

    Returns:
        Minimum distance to any edge
    """
    min_dist = float('inf')
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        dist = point_to_segment_distance(x, y, x1, y1, x2, y2)
        min_dist = min(min_dist, dist)

    return min_dist


def add_rule_area_keepout_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig,
                                    layers: Optional[List[str]] = None):
    """Block tracks/vias inside KiCad keep-out rule areas.

    Each keepout (parsed from `(zone ... (keepout ...))`) blocks track cells on its
    listed copper layers where tracks are not allowed, and via placement where vias
    are not allowed. Mirrors _add_cutout_obstacles but is per-layer and gated on the
    keepout flags. A keepout with no layer list applies to all routing layers.

    Cells whose centre is inside the polygon are blocked, as are cells just outside
    whose track/via copper (half-width plus clearance) would intrude past the keep-out
    boundary, so the copper itself stays out of the region rather than just the centre.
    """
    keepouts = getattr(pcb_data.board_info, 'keepouts', None)
    if not keepouts:
        return

    coord = GridCoord(config.grid_step)
    layer_list = layers if layers is not None else config.layers
    layer_map = build_layer_map(layer_list)
    track_clear = config.clearance + config.track_width / 2
    via_clear = config.clearance + config.via_size / 2
    # Restrict rasterization to the map's extent: on a local-window build a
    # whole-board ring keep-out would otherwise rasterize the entire board per
    # call (the dominant cost of route_disconnected_planes and the via-in-pad
    # unblock). Full-board builds set board_bounds to the whole board -> no clip.
    clip = getattr(pcb_data.board_info, 'board_bounds', None)

    for ko in keepouts:
        poly = ko.get('polygon') or []
        if len(poly) < 3:
            continue
        block_tracks = not ko.get('tracks_allowed', True)
        block_vias = not ko.get('vias_allowed', True)
        if not (block_tracks or block_vias):
            continue

        ko_layers = ko.get('layers') or set()
        if ko_layers:
            layer_idxs = [layer_map[ln] for ln in ko_layers if ln in layer_map]
        else:
            layer_idxs = list(range(len(layer_list)))
        if block_tracks and not layer_idxs:
            block_tracks = False
        if not (block_tracks or block_vias):
            continue

        # Bounding box gets a clearance margin so we also catch cells just outside
        # the polygon whose track/via copper would still intrude past the boundary.
        margin = max(track_clear, via_clear) + coord.grid_step
        gx_flat, gy_flat, inside, edge_dist = _rasterize_polygon(poly, coord, margin, clip_bounds=clip)
        if gx_flat is None:
            continue

        track_mask = inside | (edge_dist < track_clear)
        via_mask = inside | (edge_dist < via_clear)

        # Holes: the keep-out is the outer polygon MINUS its holes (a ring).
        # Cells deep inside a hole (>= the relevant clearance from the hole
        # edge) are routable; cells in the ring, or in the hole but within
        # clearance of the ring copper, stay blocked (issue #95). Evaluated
        # vectorized on the OUTER cell array - the earlier per-cell Python set +
        # generator was O(cells*holes) and dominated every obstacle-map build
        # (76s of an 86s plane pad-repair on glasgow's whole-board ring keepout).
        holes = ko.get('holes') or []
        if holes:
            px = gx_flat.astype(np.float64) * coord.grid_step
            py = gy_flat.astype(np.float64) * coord.grid_step
            for hole in holes:
                hp = np.asarray(hole, dtype=np.float64)
                if hp.shape[0] < 3:
                    continue
                hx1, hy1 = hp[:, 0], hp[:, 1]
                hx2, hy2 = np.roll(hp[:, 0], -1), np.roll(hp[:, 1], -1)
                h_inside = _points_inside_polygon(px, py, hx1, hy1, hx2, hy2)
                h_edge = _points_edge_distance(px, py, hx1, hy1, hx2, hy2)
                track_mask &= ~(h_inside & (h_edge >= track_clear))
                via_mask &= ~(h_inside & (h_edge >= via_clear))

        if block_tracks:
            _block_cells_on_layers(obstacles, gx_flat, gy_flat, track_mask, layer_idxs)
        if block_vias and via_mask.any():
            obstacles.add_blocked_vias_batch(
                np.column_stack([gx_flat[via_mask], gy_flat[via_mask]]))


def add_board_edge_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                              config: GridRouteConfig, extra_clearance: float = 0.0,
                              layers: Optional[List[str]] = None):
    """Block tracks and vias near the board edge.

    Supports both rectangular and non-rectangular board outlines. For non-rectangular
    boards (defined by a polygon in board_outline), uses point-in-polygon testing
    to properly block areas outside the board shape.

    Args:
        obstacles: The obstacle map to add to
        pcb_data: PCB data containing board bounds and optional board_outline polygon
        config: Routing configuration
        extra_clearance: Additional clearance to add
        layers: Optional list of layer names (overrides config.layers if provided)
    """
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return

    coord = GridCoord(config.grid_step)
    layer_list = layers if layers is not None else config.layers
    num_layers = len(layer_list)
    min_x, min_y, max_x, max_y = board_bounds

    # Use board_edge_clearance if set, otherwise use track clearance
    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    # Add track half-width to clearance (tracks need to stay away from edge)
    track_edge_clearance = edge_clearance + config.track_width / 2 + extra_clearance
    via_edge_clearance = edge_clearance + config.via_size / 2 + extra_clearance

    # Convert to grid coordinates. Use to_grid_dist_safe (ceil) for the via
    # keep-out so grid quantization can't leave a via inside the edge clearance
    # (#170) - mirrors the via-clearance rounding in the obstacle cache.
    track_expand = coord.to_grid_dist(track_edge_clearance)
    via_expand = coord.to_grid_dist_safe(via_edge_clearance)

    # Get grid bounds
    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)

    # Check if we have a non-rectangular board outline. Multi-outline boards
    # (#304: split keyboards / panels carry several disjoint outer rings) pass
    # ALL outers so cells inside ANY of them stay routable.
    board_outlines = [o for o in (getattr(pcb_data.board_info, 'board_outlines', None) or [])
                      if len(o) >= 3]
    if not board_outlines:
        board_outline = pcb_data.board_info.board_outline
        if board_outline and len(board_outline) >= 3:
            board_outlines = [board_outline]
    if board_outlines:
        # Use polygon-based blocking for non-rectangular boards
        _add_polygon_edge_obstacles(obstacles, board_outlines, coord, num_layers,
                                     track_edge_clearance, via_edge_clearance,
                                     gmin_x, gmin_y, gmax_x, gmax_y, track_expand, via_expand)
    else:
        # Use simple rectangular blocking
        _add_rectangular_edge_obstacles(obstacles, coord, num_layers,
                                         gmin_x, gmin_y, gmax_x, gmax_y,
                                         track_expand, via_expand)

    # Block areas inside board cutouts (e.g., connector/switch openings)
    board_cutouts = pcb_data.board_info.board_cutouts
    if board_cutouts:
        for cutout in board_cutouts:
            if len(cutout) >= 3:
                _add_cutout_obstacles(obstacles, cutout, coord, num_layers,
                                      track_edge_clearance, via_edge_clearance)


def _add_cutout_obstacles(obstacles: GridObstacleMap, cutout: List[Tuple[float, float]],
                          coord: GridCoord, num_layers: int,
                          track_edge_clearance: float, via_edge_clearance: float):
    """Block tracks and vias inside a board cutout and within clearance of its edges.

    Cells whose centre is inside the cutout polygon are blocked on all layers; cells
    just outside whose track/via copper would intrude past the edge are blocked too.
    """
    margin = max(track_edge_clearance, via_edge_clearance) + coord.grid_step
    gx_flat, gy_flat, inside, edge_dist = _rasterize_polygon(cutout, coord, margin)
    if gx_flat is None:
        return

    _block_cells_on_layers(obstacles, gx_flat, gy_flat,
                           inside | (edge_dist < track_edge_clearance), range(num_layers))
    via_mask = inside | (edge_dist < via_edge_clearance)
    if via_mask.any():
        obstacles.add_blocked_vias_batch(np.column_stack([gx_flat[via_mask], gy_flat[via_mask]]))


def _add_rectangular_edge_obstacles(obstacles: GridObstacleMap, coord: GridCoord, num_layers: int,
                                     gmin_x: int, gmin_y: int, gmax_x: int, gmax_y: int,
                                     track_expand: int, via_expand: int):
    """Add obstacles for simple rectangular board outline.

    The via keep-out band (via_expand) reaches FURTHER inboard than the track
    band (track_expand) because a via is wider than a track. Each edge sweep must
    therefore cover max(track_expand, via_expand) cells in from the edge and block
    track layers / vias per-cell: sweeping only track_expand (the old behaviour)
    never visited the inner via-only band, so route.py dropped vias up to
    (via_expand - track_expand) cells past the via keep-out, intruding into the
    board-edge clearance (#170). The track keep-out and the parallel corner
    handoff to the left/right sweeps are unchanged.
    """
    edge_expand = max(track_expand, via_expand)
    grid_margin = edge_expand + 5

    # Block left edge (full height, so it also covers the via band at both left corners)
    for gx in range(gmin_x - grid_margin, gmin_x + edge_expand + 1):
        block_track = gx <= gmin_x + track_expand
        block_via = gx < gmin_x + via_expand
        if not (block_track or block_via):
            continue
        for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
            if block_track:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
            if block_via:
                obstacles.add_blocked_via(gx, gy)

    # Block right edge (full height)
    for gx in range(gmax_x - edge_expand, gmax_x + grid_margin + 1):
        block_track = gx >= gmax_x - track_expand
        block_via = gx > gmax_x - via_expand
        if not (block_track or block_via):
            continue
        for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
            if block_track:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
            if block_via:
                obstacles.add_blocked_via(gx, gy)

    # Block top edge (middle span; corners covered by the left/right sweeps above)
    for gy in range(gmin_y - grid_margin, gmin_y + edge_expand + 1):
        block_track = gy <= gmin_y + track_expand
        block_via = gy < gmin_y + via_expand
        if not (block_track or block_via):
            continue
        for gx in range(gmin_x + track_expand + 1, gmax_x - track_expand):
            if block_track:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
            if block_via:
                obstacles.add_blocked_via(gx, gy)

    # Block bottom edge (middle span)
    for gy in range(gmax_y - edge_expand, gmax_y + grid_margin + 1):
        block_track = gy >= gmax_y - track_expand
        block_via = gy > gmax_y - via_expand
        if not (block_track or block_via):
            continue
        for gx in range(gmin_x + track_expand + 1, gmax_x - track_expand):
            if block_track:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
            if block_via:
                obstacles.add_blocked_via(gx, gy)


def _add_polygon_edge_obstacles(obstacles: GridObstacleMap, polygons,
                                 coord: GridCoord, num_layers: int,
                                 track_edge_clearance: float, via_edge_clearance: float,
                                 gmin_x: int, gmin_y: int, gmax_x: int, gmax_y: int,
                                 track_expand: int, via_expand: int):
    """Add obstacles for non-rectangular board outline using polygon testing.

    ``polygons`` is one outer ring or a LIST of outer rings (#304): a cell is
    on-board if inside ANY ring, and the edge distance is the minimum over all
    rings. For each grid cell in the bounding box area, checks if it's outside
    the board or too close to an edge (within clearance distance). Uses numpy
    vectorization for all geometry computations.
    """
    if polygons and isinstance(polygons[0], tuple):
        polygons = [polygons]
    grid_margin = max(track_expand, via_expand) + 5

    # Generate all grid coordinates as numpy arrays
    gx_range = np.arange(gmin_x - grid_margin, gmax_x + grid_margin + 1, dtype=np.int32)
    gy_range = np.arange(gmin_y - grid_margin, gmax_y + grid_margin + 1, dtype=np.int32)
    gx_grid, gy_grid = np.meshgrid(gx_range, gy_range)  # shape (ny, nx)
    gx_flat = gx_grid.ravel()  # shape (N,)
    gy_flat = gy_grid.ravel()

    # Convert grid coords to board coords (float mm)
    px = gx_flat.astype(np.float64) * coord.grid_step
    py = gy_flat.astype(np.float64) * coord.grid_step

    # Per-ring edge arrays; on-board = inside ANY ring (ray cast, chunked so
    # the (points, edges) broadcast temporaries stay bounded, issue #81).
    ring_edges = []
    inside = None
    for polygon in polygons:
        poly_arr = np.array(polygon, dtype=np.float64)
        rx1 = poly_arr[:, 0]
        ry1 = poly_arr[:, 1]
        rx2 = np.roll(poly_arr[:, 0], -1)
        ry2 = np.roll(poly_arr[:, 1], -1)
        ring_edges.append((rx1, ry1, rx2, ry2))
        ins = _points_inside_polygon(px, py, rx1, ry1, rx2, ry2)
        inside = ins if inside is None else (inside | ins)
    x1, y1, x2, y2 = (np.concatenate([e[i] for e in ring_edges]) for i in range(4))

    # --- Vectorized point-to-polygon-edge distance ---
    # For inside points, compute min distance to any edge
    # Only compute for points that are inside (saves work for outside points)
    inside_idx = np.where(inside)[0]
    outside_idx = np.where(~inside)[0]

    # Block all outside points (all layers + vias)
    if outside_idx.size > 0:
        out_gx = gx_flat[outside_idx]
        out_gy = gy_flat[outside_idx]
        out_cells = np.column_stack([out_gx, out_gy])
        for layer_idx in range(num_layers):
            layer_col = np.full((out_cells.shape[0], 1), layer_idx, dtype=np.int32)
            obstacles.add_blocked_cells_batch(np.hstack([out_cells, layer_col]))
        obstacles.add_blocked_vias_batch(out_cells)

    # Compute edge distances for inside points (chunked kernel, issue #81)
    if inside_idx.size > 0:
        in_px = px[inside_idx]  # (M,)
        in_py = py[inside_idx]

        min_dist = _points_edge_distance(in_px, in_py, x1, y1, x2, y2)  # (M,)

        in_gx = gx_flat[inside_idx]
        in_gy = gy_flat[inside_idx]

        # Block tracks if too close to edge
        track_mask = min_dist < track_edge_clearance
        if np.any(track_mask):
            track_gx = in_gx[track_mask]
            track_gy = in_gy[track_mask]
            track_cells = np.column_stack([track_gx, track_gy])
            for layer_idx in range(num_layers):
                layer_col = np.full((track_cells.shape[0], 1), layer_idx, dtype=np.int32)
                obstacles.add_blocked_cells_batch(np.hstack([track_cells, layer_col]))

        # Block vias if too close to edge
        via_mask = min_dist < via_edge_clearance
        if np.any(via_mask):
            via_gx = in_gx[via_mask]
            via_gy = in_gy[via_mask]
            obstacles.add_blocked_vias_batch(np.column_stack([via_gx, via_gy]))


def block_via_cells_near_drills(obstacles: GridObstacleMap,
                                 drill_holes, via_drill: float,
                                 hole_to_hole_clearance: float, grid_step: float):
    """Block via-placement cells within the hole-to-hole drill minimum of each
    drill hole.

    A via placed on the grid sits at its cell's real center; block the cell when
    that center is within the required center-to-center distance of the REAL
    drill center, tested in mm -- NOT as a floored/quantized integer-cell disk.
    Flooring the radius (or centering the disk on the quantized drill cell) lets
    a via land a sub-cell inside the hole-to-hole minimum (issue #70 / #125:
    PAD-DRILL-VIA-DRILL at the default 0.1mm grid). Being exact in mm avoids both
    the under-block (a real fab violation) and the over-block (lost routability).

    Shared by the signal router (add_drill_hole_obstacles) and route_planes
    (_add_drill_hole_via_obstacles) so both enforce the keepout identically.

    Args:
        obstacles: the obstacle map to add via-blocks to
        drill_holes: iterable of (x_mm, y_mm, drill_diameter_mm)
        via_drill: drill diameter of the via being placed (mm)
        hole_to_hole_clearance: minimum drill edge-to-edge clearance (mm)
        grid_step: grid resolution (mm)
    """
    if hole_to_hole_clearance <= 0:
        return
    coord = GridCoord(grid_step)
    cells = []
    for hx, hy, drill_dia in drill_holes:
        # Required center-to-center distance = drill/2 + via_drill/2 + clearance.
        required_dist = drill_dia / 2.0 + via_drill / 2.0 + hole_to_hole_clearance
        req_sq = required_dist * required_dist
        gx, gy = coord.to_grid(hx, hy)
        expand = coord.to_grid_dist_safe(required_dist) + 1  # ceil + 1-cell bbox margin
        for ex in range(-expand, expand + 1):
            cx = (gx + ex) * grid_step
            for ey in range(-expand, expand + 1):
                cy = (gy + ey) * grid_step
                if (cx - hx) * (cx - hx) + (cy - hy) * (cy - hy) < req_sq:
                    cells.append((gx + ex, gy + ey))
    if cells:
        obstacles.add_blocked_vias_batch(np.array(cells, dtype=np.int32))


def _pad_has_copper(pad) -> bool:
    """True if the pad has copper on any layer (so its track clearance is enforced
    by the copper-pad obstacle). NPTH mounting holes return False -- they need the
    separate drill track keep-out (issue #233). Most list only *.Mask/*.Paste, but
    some libraries write *.Cu on an np_thru_hole pad (hole keep-out), so trust the
    pad type first: an NPTH pad never carries a copper ring (issue #260)."""
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return False
    return any(l == '*.Cu' or l.endswith('.Cu') for l in pad.layers)


def block_track_cells_near_drills(obstacles: GridObstacleMap, drill_holes,
                                  track_width: float, clearance: float,
                                  grid_step: float, layer_idxs):
    """Block TRACK cells on each layer in ``layer_idxs`` within
    ``drill/2 + track/2 + clearance`` of every drill hole, so a routed track
    cannot cross an NPTH mounting hole or a foreign PTH barrel (issue #233).

    NPTH mounting holes parse as ``drill>0`` with only a ``*.Mask`` layer, so the
    copper-pad blocker (_add_pad_obstacle / the plane pad loop) stamps NO cell for
    them and nothing else stops a track running straight over the hole -- a real
    fab short, which the drill removes. This mirrors block_via_cells_near_drills
    (exact mm-distance disc, not a floored integer disk -- issue #70/#125) but
    stamps track cells on copper layers and uses the copper-to-hole ``clearance``
    rather than the hole-to-hole drill minimum. A drill goes through every layer,
    so the multi-layer signal map passes all copper layer indices; the single-layer
    plane map passes just its own.
    """
    if clearance < 0 or not layer_idxs:
        return
    coord = GridCoord(grid_step)
    cells = []
    for hx, hy, drill_dia in drill_holes:
        # A track centerline must stay this far from the real drill centre.
        required_dist = drill_dia / 2.0 + track_width / 2.0 + clearance
        req_sq = required_dist * required_dist
        gx, gy = coord.to_grid(hx, hy)
        expand = coord.to_grid_dist_safe(required_dist) + 1  # ceil + 1-cell bbox margin
        for ex in range(-expand, expand + 1):
            cx = (gx + ex) * grid_step
            for ey in range(-expand, expand + 1):
                cy = (gy + ey) * grid_step
                if (cx - hx) * (cx - hx) + (cy - hy) * (cy - hy) < req_sq:
                    cells.append((gx + ex, gy + ey))
    if not cells:
        return
    arr = np.array(cells, dtype=np.int32)
    for li in layer_idxs:
        layer_col = np.full((arr.shape[0], 1), li, dtype=np.int32)
        obstacles.add_blocked_cells_batch(np.hstack([arr, layer_col]))


def add_drill_hole_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                              config: GridRouteConfig, nets_to_route_set: set,
                              extra_clearance: float = 0.0):
    """Keep routed copper off existing drill holes -- vias, PTH pad barrels, and
    NPTH mounting holes alike (excluding the net(s) being routed, which must still
    reach their own through-holes).

    Two keep-outs:
      - TRACK crossing of NPTH (no-copper) holes on ALL copper layers, uses
        ``config.clearance`` -- a track can't be routed across a mounting hole
        whose only layer is *.Mask (issue #233). PTH pads and vias carry copper,
        so their track clearance is already enforced by the pad/via copper
        obstacles (with the real pad shape, not a round-drill approximation);
      - VIA placement (hole-to-hole drill minimum) near EVERY drill, uses
        ``config.hole_to_hole_clearance``.

    Args:
        obstacles: The obstacle map to add to
        pcb_data: PCB data containing vias and pads with drills
        config: Routing configuration
        nets_to_route_set: Set of net IDs being routed (excluded from blocking)
    """
    drill_holes = []   # every drill -> via (hole-to-hole) keep-out
    npth_holes = []    # no-copper holes only -> track keep-out

    # Add via drills (excluding nets being routed)
    for via in pcb_data.vias:
        if via.net_id not in nets_to_route_set and via.drill > 0:
            drill_holes.append((via.x, via.y, via.drill))

    # Add pad drills (through-hole AND NPTH mounting pads carry drills)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            if pad.drill > 0:
                # Slot drills expand to circles along the slot axis (short-
                # dimension diameter) instead of one long-dimension disc --
                # a 30.2x1mm milled slot must not become a 30mm round keep-out.
                circles = pad_drill_circles(pad)
                drill_holes.extend(circles)
                if not _pad_has_copper(pad):
                    npth_holes.extend(circles)

    # Track keep-out for NPTH holes on every copper layer (issue #233). The
    # copper-to-NPTH-hole floor is the JLC "NPTH to Track" fab value, never below
    # the routing clearance.
    if npth_holes:
        # extra_clearance covers geometry offset from the routed centerline
        # (diff-pair P/N tracks ride +-(gap+width)/2 off it), matching how every
        # other obstacle in that base map is inflated (issue #268).
        npth_clr = max(config.clearance, defaults.NPTH_TO_TRACK_CLEARANCE) + extra_clearance
        block_track_cells_near_drills(obstacles, npth_holes, config.track_width,
                                      npth_clr, config.grid_step,
                                      list(range(len(config.layers))))

    # Via keep-out (hole-to-hole drill minimum) near every drill.
    if config.hole_to_hole_clearance > 0 and drill_holes:
        block_via_cells_near_drills(obstacles, drill_holes, config.via_drill,
                                    config.hole_to_hole_clearance, config.grid_step)


def add_net_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig,
                                extra_clearance: float = 0.0):
    """Add a net's stub segments as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    # Add segments - use actual segment width and layer-specific routing track width
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        # Use layer-specific track width for routing track portion
        layer_track_width = config.get_track_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else layer_track_width
        expansion_mm = layer_track_width / 2 + seg_width / 2 + config.clearance + extra_clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + config.clearance + extra_clearance
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_mm, via_block_mm)


def add_diff_pair_own_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                          p_net_id: int, n_net_id: int,
                                          config: GridRouteConfig,
                                          exclude_endpoints: List[Tuple[float, float]] = None,
                                          extra_clearance: float = 0.0,
                                          exclude_cells: Set[Tuple[int, int]] = None):
    """Add a diff pair's own stub segments as obstacles to prevent centerline from crossing them.

    This is different from add_net_stubs_as_obstacles which adds OTHER nets' stubs.
    Here we add the SAME pair's stubs so the centerline route avoids crossing them,
    but we exclude the stub endpoints where we need to connect.

    Args:
        obstacles: The obstacle map to modify
        pcb_data: PCB data containing segments
        p_net_id: Net ID of P net
        n_net_id: Net ID of N net
        config: Routing configuration
        exclude_endpoints: List of (x, y) positions to exclude from blocking (stub connection points)
        extra_clearance: Additional clearance to add
        exclude_cells: Additional grid cells to exclude from blocking (e.g.
            connector corridors of a multi-point leg, so a previous leg's
            tracks at a shared terminal don't block the opposite-side setback)
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    # Convert exclude endpoints to grid coordinates with some radius
    # Use max track width for exclusion radius
    exclude_grid_cells = set(exclude_cells) if exclude_cells else set()
    max_track_width = config.get_max_track_width()
    exclude_radius = max(2, coord.to_grid_dist(max_track_width * 2))  # 2x track width radius
    if exclude_endpoints:
        for ex, ey in exclude_endpoints:
            gex, gey = coord.to_grid(ex, ey)
            for dx in range(-exclude_radius, exclude_radius + 1):
                for dy in range(-exclude_radius, exclude_radius + 1):
                    if dx*dx + dy*dy <= exclude_radius * exclude_radius:
                        exclude_grid_cells.add((gex + dx, gey + dy))

    # Add segments - use actual segment width and layer-specific routing track width
    for seg in pcb_data.segments:
        if seg.net_id != p_net_id and seg.net_id != n_net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        # Compute expansion based on actual segment width and layer-specific track width
        layer_track_width = config.get_track_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else layer_track_width
        expansion_mm = layer_track_width / 2 + seg_width / 2 + config.clearance + extra_clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + config.clearance + extra_clearance
        _add_segment_obstacle_with_exclusion(
            obstacles, seg, coord, layer_idx, exclude_grid_cells, expansion_mm, via_block_mm
        )


def add_diff_pair_own_pads_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                         p_net_id: int, n_net_id: int,
                                         config: GridRouteConfig,
                                         exempt_capsules: List[Tuple[float, float, float, float, float]] = None,
                                         extra_clearance: float = 0.0,
                                         exempt_pads: set = None):
    """Add a diff pair's own pads as obstacles for centerline routing.

    The diff pair's nets are excluded from the base obstacle map so the route can
    reach its own pads - but that also lets the centerline (and its offset P/N
    tracks) cross the partner polarity's pads mid-route, creating shorts. This
    blocks the pair's own pads everywhere EXCEPT inside the given exempt capsules,
    which carve out the connector corridors at the route endpoints (from the
    pad-pair center out past the setback position) where the route legitimately
    approaches and fans out to the pads.

    Args:
        obstacles: The obstacle map to modify
        pcb_data: PCB data containing pads
        p_net_id: Net ID of P net
        n_net_id: Net ID of N net
        config: Routing configuration
        exempt_capsules: List of (x1, y1, x2, y2, radius_mm) capsules in mm where
            pad blocking is skipped
        extra_clearance: Additional clearance to add (diff pair half-spacing)
        exempt_pads: Optional set of id(pad) allowed to be opened by the capsules.
            A capsule only carves out the corridor of the leg that built it, so on
            a multi-point pair another terminal's pad that happens to fall inside
            this corridor must stay fully blocked - otherwise the partner-polarity
            track grazes it (castor_pollux: J2's corridor clipped the bottom of
            J11's 4mm connector pad). When None, every own pad is exemptable
            (the old behavior, correct for a single-terminal corridor set).
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    # Convert capsules to grid units once
    capsules_grid = []
    for (x1, y1, x2, y2, radius_mm) in (exempt_capsules or []):
        ax, ay = coord.to_grid(x1, y1)
        bx, by = coord.to_grid(x2, y2)
        capsules_grid.append((ax, ay, bx, by, radius_mm / config.grid_step))

    def in_exempt_capsule(gx: int, gy: int) -> bool:
        for (ax, ay, bx, by, radius_grid) in capsules_grid:
            abx, aby = bx - ax, by - ay
            apx, apy = gx - ax, gy - ay
            ab_len_sq = abx * abx + aby * aby
            if ab_len_sq == 0:
                t = 0.0
            else:
                t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len_sq))
            dx = apx - t * abx
            dy = apy - t * aby
            if dx * dx + dy * dy <= radius_grid * radius_grid:
                return True
        return False

    for net_id in (p_net_id, n_net_id):
        for pad in pcb_data.pads_by_net.get(net_id, []):
            # A foreign pad (one this leg's corridors don't serve) stays fully
            # blocked; only this leg's own terminal pads may be opened.
            skip = (in_exempt_capsule if (exempt_pads is None or id(pad) in exempt_pads)
                    else None)
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config,
                              extra_clearance=extra_clearance,
                              skip_cell=skip)


def _add_segment_obstacle_with_exclusion(obstacles: GridObstacleMap, seg, coord: GridCoord,
                                          layer_idx: int, exclude_cells: Set[Tuple[int, int]],
                                          expansion_mm: float, via_block_mm: float):
    """Add a segment as obstacle (exact capsule keep-out from the true float
    segment), excluding certain grid cells (the stub-endpoint connection cells).
    Track keep-out at `expansion_mm`, via keep-out at `via_block_mm`."""
    for cgx, cgy in segment_blocked_cells_array(seg.start_x, seg.start_y,
                                                seg.end_x, seg.end_y, expansion_mm, coord.grid_step):
        c = (int(cgx), int(cgy))
        if c not in exclude_cells:
            obstacles.add_blocked_cell(c[0], c[1], layer_idx)
    for cgx, cgy in segment_blocked_cells_array(seg.start_x, seg.start_y,
                                                seg.end_x, seg.end_y, via_block_mm, coord.grid_step):
        c = (int(cgx), int(cgy))
        if c not in exclude_cells:
            obstacles.add_blocked_via(c[0], c[1])


def add_net_pads_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0):
    """Add a net's pads as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance)


def add_net_vias_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                               net_id: int, config: GridRouteConfig,
                               extra_clearance: float = 0.0,
                               diagonal_margin: float = 0.0):
    """Add a net's vias as obstacles to the map.

    Args:
        diagonal_margin: Extra margin (in grid units) for track blocking to catch diagonal
                        segments that pass between grid points. Use 0.25 for single-ended routing.
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    # Add vias - use actual via size and max track width (vias span all layers)
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, config.clearance, extra_clearance)
        via_via_mm = via_size / 2 + config.via_size / 2 + config.clearance
        # True via-via clearance radius in cells as a FLOAT (no floor): the disc
        # threshold is radius**2, so this blocks exactly the cells within the real
        # clearance. Flooring (to_grid_dist) lost up to ~1 cell and let two vias sit
        # a diagonal cell-offset too close (e.g. (3,2) cells = 0.36mm when 0.39mm is
        # required) -- a real cross-net via-via DRC violation the router never saw.
        via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid, diagonal_margin)


def _ledger_bracket(obstacles):
    """(cells, vias) counts if the #309 obstacle ledger is armed for this map,
    else None. Zero cost when KICAD_OBSTACLE_LEDGER is off."""
    import obstacle_cache as _oc
    L = _oc._LEDGER
    if L is None or L.get("wid") != id(obstacles):
        return None
    st = obstacles.get_stats()
    return st[0], st[1]


def _ledger_close(obstacles, pre, tag: str):
    if pre is None:
        return
    import obstacle_cache as _oc
    st = obstacles.get_stats()
    site = _oc._ledger_site(depth=3, frames=2)
    _oc.ledger_raw_delta(obstacles, f"{tag} @ {site}", st[0] - pre[0], st[1] - pre[1])


def add_vias_list_as_obstacles(obstacles: GridObstacleMap, vias: list,
                                config: GridRouteConfig,
                                extra_clearance: float = 0.0,
                                diagonal_margin: float = 0.0):
    """Add a list of Via objects as obstacles to the map.

    This is useful for adding vias from a route result before it's committed to pcb_data.

    Args:
        obstacles: The obstacle map to add to
        vias: List of Via objects to add as obstacles
        config: Routing configuration
        extra_clearance: Additional clearance to add (for diff pairs)
        diagonal_margin: Extra margin (in grid units) for track blocking
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    _pre = _ledger_bracket(obstacles)

    # Add vias - use actual via size and max track width (vias span all layers)
    for via in vias:
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, config.clearance, extra_clearance)
        via_via_mm = via_size / 2 + config.via_size / 2 + config.clearance
        # True via-via clearance radius in cells as a FLOAT (no floor): the disc
        # threshold is radius**2, so this blocks exactly the cells within the real
        # clearance. Flooring (to_grid_dist) lost up to ~1 cell and let two vias sit
        # a diagonal cell-offset too close (e.g. (3,2) cells = 0.36mm when 0.39mm is
        # required) -- a real cross-net via-via DRC violation the router never saw.
        via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid, diagonal_margin)
    _ledger_close(obstacles, _pre, "add_vias_list")


def add_segments_list_as_obstacles(obstacles: GridObstacleMap, segments: list,
                                    config: GridRouteConfig,
                                    extra_clearance: float = 0.0):
    """Add a list of Segment objects as obstacles to the map.

    This is useful for adding segments from a route result before it's committed to pcb_data.

    Args:
        obstacles: The obstacle map to add to
        segments: List of Segment objects to add as obstacles
        config: Routing configuration
        extra_clearance: Additional clearance to add (for diff pairs)
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    _pre = _ledger_bracket(obstacles)

    # Add segments - use actual segment width and layer-specific routing track width
    for seg in segments:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is not None:
            # Use layer-specific track width for routing track portion
            layer_track_width = config.get_track_width(seg.layer)
            seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else layer_track_width
            expansion_mm = layer_track_width / 2 + seg_width / 2 + config.clearance + extra_clearance
            via_block_mm = config.via_size / 2 + seg_width / 2 + config.clearance
            _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_mm, via_block_mm)
    _ledger_close(obstacles, _pre, "add_segments_list")


def remove_segments_list_from_obstacles(obstacles: GridObstacleMap, segments: list,
                                         config: GridRouteConfig,
                                         extra_clearance: float = 0.0):
    """Remove a list of Segment objects from the obstacle map.

    This reverses the effect of add_segments_list_as_obstacles. It collects all cells
    that would be blocked and removes them using batch operations.

    Args:
        obstacles: The obstacle map to remove from
        segments: List of Segment objects to remove as obstacles
        config: Routing configuration
        extra_clearance: Additional clearance that was used when adding (for diff pairs)
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    _pre = _ledger_bracket(obstacles)

    # Collect all cells and vias to remove
    cells_to_remove = []  # (gx, gy, layer_idx) tuples
    vias_to_remove = []   # (gx, gy) tuples

    # Remove segments - use actual segment width and layer-specific track width (same as add function)
    for seg in segments:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        # Must match the ADD shape exactly (same capsule from the true float
        # segment) or the Rust ref-counts desync on rip-up.
        layer_track_width = config.get_track_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else layer_track_width
        expansion_mm = layer_track_width / 2 + seg_width / 2 + config.clearance + extra_clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + config.clearance

        for cgx, cgy in segment_blocked_cells_array(
                seg.start_x, seg.start_y, seg.end_x, seg.end_y, expansion_mm, coord.grid_step):
            cells_to_remove.append((int(cgx), int(cgy), layer_idx))
        for cgx, cgy in segment_blocked_cells_array(
                seg.start_x, seg.start_y, seg.end_x, seg.end_y, via_block_mm, coord.grid_step):
            vias_to_remove.append((int(cgx), int(cgy)))

    # Batch remove cells and vias
    if cells_to_remove:
        cells_array = np.array(cells_to_remove, dtype=np.int32)
        obstacles.remove_blocked_cells_batch(cells_array)
    if vias_to_remove:
        vias_array = np.array(vias_to_remove, dtype=np.int32)
        obstacles.remove_blocked_vias_batch(vias_array)
    _ledger_close(obstacles, _pre, "remove_segments_list")


def remove_vias_list_from_obstacles(obstacles: GridObstacleMap, vias: list,
                                     config: GridRouteConfig,
                                     extra_clearance: float = 0.0,
                                     diagonal_margin: float = 0.0):
    """Remove a list of Via objects from the obstacle map.

    This reverses the effect of add_vias_list_as_obstacles. It collects all cells
    that would be blocked and removes them using batch operations.

    Args:
        obstacles: The obstacle map to remove from
        vias: List of Via objects to remove as obstacles
        config: Routing configuration
        extra_clearance: Additional clearance that was used when adding (for diff pairs)
        diagonal_margin: Extra margin that was used when adding
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    _pre = _ledger_bracket(obstacles)

    # Collect all cells and vias to remove
    cells_to_remove = []  # (gx, gy, layer_idx) tuples
    vias_to_remove = []   # (gx, gy) tuples

    for via in vias:
        gx, gy = coord.to_grid(via.x, via.y)
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size

        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, config.clearance, extra_clearance)
        via_via_mm = via_size / 2 + config.via_size / 2 + config.clearance
        # True via-via clearance radius in cells as a FLOAT (no floor): the disc
        # threshold is radius**2, so this blocks exactly the cells within the real
        # clearance. Flooring (to_grid_dist) lost up to ~1 cell and let two vias sit
        # a diagonal cell-offset too close (e.g. (3,2) cells = 0.36mm when 0.39mm is
        # required) -- a real cross-net via-via DRC violation the router never saw.
        via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)

        # Mirror _add_via_obstacle EXACTLY (incl. the sub-grid offset and float
        # radius) so rip-up removes precisely the cells the add placed -- otherwise
        # blocked cells leak across rip/reroute cycles and corrupt the map.
        off_cells = math.hypot(via.x - gx * coord.grid_step,
                               via.y - gy * coord.grid_step) / coord.grid_step

        # Track blocking - PER LAYER (mirror _add_via_obstacle's per-layer list).
        for layer_idx in range(num_layers):
            radius = via_track_expansion_grid[layer_idx] + diagonal_margin + off_cells
            effective_track_block_sq = radius ** 2
            track_block_range = int(math.ceil(radius))
            for ex in range(-track_block_range, track_block_range + 1):
                for ey in range(-track_block_range, track_block_range + 1):
                    if ex*ex + ey*ey <= effective_track_block_sq:
                        cells_to_remove.append((gx + ex, gy + ey, layer_idx))

        # Via blocking cells
        via_radius = via_via_expansion_grid + off_cells
        vr_range = int(math.ceil(via_radius))
        vr_sq = via_radius * via_radius
        for ex in range(-vr_range, vr_range + 1):
            for ey in range(-vr_range, vr_range + 1):
                if ex*ex + ey*ey <= vr_sq:
                    vias_to_remove.append((gx + ex, gy + ey))

    # Batch remove cells and vias
    if cells_to_remove:
        cells_array = np.array(cells_to_remove, dtype=np.int32)
        obstacles.remove_blocked_cells_batch(cells_array)
    if vias_to_remove:
        vias_array = np.array(vias_to_remove, dtype=np.int32)
        obstacles.remove_blocked_vias_batch(vias_array)
    _ledger_close(obstacles, _pre, "remove_vias_list")


def add_same_net_via_clearance(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig):
    """Add via-via clearance blocking for same-net vias.

    This blocks only via placement (not track routing) near existing vias on the same net,
    enforcing DRC via-via clearance even within a single net.
    """
    coord = GridCoord(config.grid_step)

    # Via-via clearance: center-to-center distance must be >= via_size + clearance
    # So we block via placement within this radius of existing vias
    via_via_expansion_grid = max(1.0, (config.via_size + config.clearance) * coord.inv_step)

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        # Grow the ring by the via's sub-grid offset so an off-grid via-in-pad keeps
        # a NEW same-net via the full hole-to-hole distance from its TRUE centre, not
        # its rounded cell (issue #70 -- otherwise a route via lands a sub-cell too
        # close to a BGA fanout via-in-pad). Mirror of the via-obstacle rasterizers.
        off_cells = math.hypot(via.x - gx * coord.grid_step,
                               via.y - gy * coord.grid_step) / coord.grid_step
        radius = via_via_expansion_grid + off_cells
        rng = int(math.ceil(radius))
        radius_sq = radius * radius
        # Only block via placement, not track routing (tracks can pass through same-net vias)
        for ex in range(-rng, rng + 1):
            for ey in range(-rng, rng + 1):
                if ex*ex + ey*ey <= radius_sq:
                    obstacles.add_blocked_via(gx + ex, gy + ey)


def add_same_net_pad_drill_via_clearance(obstacles: GridObstacleMap, pcb_data: PCBData,
                                          net_id: int, config: GridRouteConfig):
    """Add via blocking near same-net pad drill holes (hole-to-hole clearance).

    This blocks via placement near through-hole pads on the same net,
    enforcing manufacturing hole-to-hole clearance even within a single net.
    New vias must maintain hole_to_hole_clearance from existing pad drill holes.

    IMPORTANT: The pad center itself is NOT blocked - the router can use existing
    through-hole pads for layer transitions without placing a new via. Only the
    area around the pad (within clearance distance) is blocked for new vias.
    """
    if config.hole_to_hole_clearance <= 0:
        return

    coord = GridCoord(config.grid_step)

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        if pad.drill <= 0:
            continue  # SMD pad, no drill hole

        # Keep-out radius = (drill_radius) + (new_via_drill/2) + clearance, measured
        # to the drill's real CAPSULE axis (a milled slot is a capsule, not a round
        # hole -- pad.drill/2 as a plain circle mis-models it). mm-distance test (not
        # floored cells) so a via cannot land a sub-cell inside the hole-to-hole
        # minimum (issue #70 / #125). Round drills degenerate to the old centre test.
        (p1x, p1y), (p2x, p2y), prad = pad_drill_capsule(pad)
        required_dist = prad + config.via_drill / 2 + config.hole_to_hole_clearance
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)  # pad centre = capsule midpoint
        step = config.grid_step
        half_len = math.hypot(p2x - p1x, p2y - p1y) / 2.0
        expand = coord.to_grid_dist_safe(required_dist + half_len) + 1  # ceil + 1-cell margin

        for ex in range(-expand, expand + 1):
            cx = (gx + ex) * step
            for ey in range(-expand, expand + 1):
                cy = (gy + ey) * step
                if point_to_segment_distance(cx, cy, p1x, p1y, p2x, p2y) < required_dist:
                    # Skip the pad center - the router can use the existing
                    # through-hole for layer transitions without a new via
                    if ex == 0 and ey == 0:
                        continue
                    obstacles.add_blocked_via(gx + ex, gy + ey)


def get_same_net_through_hole_positions(pcb_data: PCBData, net_id: int,
                                        config: GridRouteConfig) -> Set[Tuple[int, int]]:
    """Get grid positions where this net already has a hole spanning all layers.

    A layer change at one of these cells needs NO new via: an existing through-hole
    pad OR an existing same-net via (fanout via-in-pad, a prior route's via, a board
    via) already connects the layers. The router reuses the hole (see the free-via
    override in the Rust router), and route conversion must SUPPRESS emitting its own
    via there -- otherwise it stacks a second, near-coincident via beside the existing
    one (an un-manufacturable hole-to-hole short). Each cell is to_grid(x, y), matching
    the free-via registration, so the suppressed cell is exactly where the router
    transitions.

    Args:
        pcb_data: PCB data with pads_by_net and vias
        net_id: Net ID to get hole positions for
        config: Grid routing config (for grid_step)

    Returns:
        Set of (gx, gy) grid coordinates where a same-net through-hole exists
    """
    coord = GridCoord(config.grid_step)
    positions = set()

    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        if pad.drill and pad.drill > 0:
            # Offset pads (#325): the reusable BARREL is at the hole anchor,
            # not the (possibly offset) copper centre.
            hx = getattr(pad, 'hole_x', None)
            hy = getattr(pad, 'hole_y', None)
            gx, gy = coord.to_grid(hx if hx is not None else pad.global_x,
                                   hy if hy is not None else pad.global_y)
            positions.add((gx, gy))

    # Existing same-net vias are reusable holes too (see free-via reuse). Without
    # this, only the multipoint tap path augmented the set, so the Phase-1 main edge
    # and single-ended routes dropped a duplicate via onto an existing via-in-pad.
    for via in pcb_data.vias:
        if via.net_id == net_id:
            positions.add(coord.to_grid(via.x, via.y))

    return positions


def _batch_cells_one_layer(obstacles, cells_xy: "np.ndarray", layer_idx: int,
                           blocked_cells=None):
    """Block an (N, 2) array of cells on one layer via the batch API."""
    if len(cells_xy) == 0:
        return
    rows = np.empty((len(cells_xy), 3), dtype=np.int32)
    rows[:, :2] = cells_xy
    rows[:, 2] = layer_idx
    obstacles.add_blocked_cells_batch(np.ascontiguousarray(rows))
    if blocked_cells is not None:
        blocked_cells[layer_idx].update(map(tuple, cells_xy.tolist()))


def _batch_vias(obstacles, vias_xy: "np.ndarray", blocked_vias=None):
    """Block an (N, 2) array of via positions via the batch API."""
    if len(vias_xy) == 0:
        return
    obstacles.add_blocked_vias_batch(np.ascontiguousarray(vias_xy.astype(np.int32)))
    if blocked_vias is not None:
        blocked_vias.update(map(tuple, vias_xy.tolist()))




def _add_segment_obstacle(obstacles: GridObstacleMap, seg, coord: GridCoord,
                          layer_idx: int, expansion_mm: float, via_block_mm: float,
                          blocked_cells: List[Set[Tuple[int, int]]] = None,
                          blocked_vias: Set[Tuple[int, int]] = None):
    """Add a segment as obstacle: an exact point-to-segment (capsule) keep-out
    measured from the TRUE float segment -- the track keep-out at `expansion_mm`
    and the via keep-out at `via_block_mm`. Matches the obstacle cache
    (`segment_blocked_cells_array`, issue #70/B); replaced the square/bresenham
    stamp that over-reached sqrt(2) in diagonal corners (#197) and under-covered
    off-grid lines (#173).

    Args:
        obstacles: The obstacle map to add to
        seg: Segment object with start_x, start_y, end_x, end_y
        coord: GridCoord for coordinate conversion
        layer_idx: Layer index for track blocking
        expansion_mm: track keep-out distance (mm) from the segment centreline
        via_block_mm: via keep-out distance (mm) from the segment centreline
        blocked_cells: Optional per-layer sets to collect blocked cells for visualization
        blocked_vias: Optional set to collect blocked via positions for visualization
    """
    cells = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                        seg.end_x, seg.end_y, expansion_mm, coord.grid_step)
    _batch_cells_one_layer(obstacles, cells, layer_idx, blocked_cells)

    vias = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                       seg.end_x, seg.end_y, via_block_mm, coord.grid_step)
    _batch_vias(obstacles, vias, blocked_vias)


def _via_track_expansion_per_layer(via_size: float, config: GridRouteConfig,
                                   coord: GridCoord, clearance: float,
                                   extra_clearance: float = 0.0):
    """Per-layer via->track keep-out radius (cells): a via blocks tracks on EACH
    layer at THAT layer's routing width. Replaces a single max_track_width value,
    which over-covered thinner layers and double-counted the router's per-net
    track_margin for wide nets. Matches the cache (_collect_via_obstacles) and the
    segment stamp (which already key off the per-layer width)."""
    return [max(1, coord.to_grid_dist_safe(
                via_size / 2 + config.get_track_width(layer) / 2 + clearance + extra_clearance))
            for layer in config.layers]


def _add_via_obstacle(obstacles: GridObstacleMap, via, coord: GridCoord,
                      num_layers: int, via_track_expansion_grid, via_via_expansion_grid: int,
                      diagonal_margin: float = 0.0,
                      blocked_cells: List[Set[Tuple[int, int]]] = None,
                      blocked_vias: Set[Tuple[int, int]] = None):
    """Add a via as obstacle to the map.

    Args:
        via_track_expansion_grid: Either a single int (same for all layers) or a list of ints
                                 (per-layer expansion) for impedance-controlled routing.
        diagonal_margin: Extra margin (in grid units) for track blocking to catch diagonal
                        segments that pass between grid points. Use 0.25 for single-ended routing.
        blocked_cells: Optional per-layer sets to collect blocked cells for visualization
        blocked_vias: Optional set to collect blocked via positions for visualization
    """
    gx, gy = coord.to_grid(via.x, via.y)
    center = np.array([gx, gy], dtype=np.int32)

    # Sub-grid offset: an off-grid via (e.g. a BGA fanout via-in-pad whose ball
    # centre is not on the routing grid) has its blocked region centred on the
    # ROUNDED cell, so foreign copper that clears the rounded centre still grazes
    # the TRUE centre by up to the offset. Grow every blocking radius by this
    # offset (in cells) so the blocked disc covers the real via position. On-grid
    # vias (router-placed, at to_float of a cell) have offset ~0 and are unchanged,
    # so routability is barely perturbed -- only off-grid pre-existing vias expand.
    off_cells = math.hypot(via.x - gx * coord.grid_step,
                           via.y - gy * coord.grid_step) / coord.grid_step

    # Batched rasterization (issue #35) - emits the same cell multiset as the
    # per-cell loops it replaces (each layer gets the full circle pattern).
    if isinstance(via_track_expansion_grid, list):
        # Per-layer blocking (impedance-controlled routing)
        for layer_idx in range(num_layers):
            layer_expansion = via_track_expansion_grid[layer_idx]
            radius = layer_expansion + diagonal_margin + off_cells
            effective_track_block_sq = radius ** 2
            track_block_range = int(math.ceil(radius))
            offs = circle_offsets(track_block_range, effective_track_block_sq)
            _batch_cells_one_layer(obstacles, center + offs, layer_idx, blocked_cells)
    else:
        # Single value for all layers (legacy behavior)
        radius = via_track_expansion_grid + diagonal_margin + off_cells
        effective_track_block_sq = radius ** 2
        track_block_range = int(math.ceil(radius))
        offs = circle_offsets(track_block_range, effective_track_block_sq)
        cells = center + offs
        for layer_idx in range(num_layers):
            _batch_cells_one_layer(obstacles, cells, layer_idx, blocked_cells)

    # Block cells for via placement (also grown by the sub-grid offset).
    via_radius = via_via_expansion_grid + off_cells
    via_offs = circle_offsets(int(math.ceil(via_radius)), via_radius * via_radius)
    _batch_vias(obstacles, center + via_offs, blocked_vias)


def _add_pad_obstacle(obstacles: GridObstacleMap, pad, coord: GridCoord,
                      layer_map: Dict[str, int], config: GridRouteConfig,
                      extra_clearance: float = 0.0,
                      blocked_cells: List[Set[Tuple[int, int]]] = None,
                      blocked_vias: Set[Tuple[int, int]] = None,
                      clearance_override: float = None,
                      skip_cell=None):
    """Add a pad as obstacle to the map.

    Uses rectangular-with-rounded-corners pattern matching other pad blocking functions.

    Args:
        obstacles: The obstacle map to add to
        pad: Pad object with global_x, global_y, size_x, size_y, layers
        coord: GridCoord for coordinate conversion
        layer_map: Mapping of layer names to layer indices
        config: Routing configuration
        extra_clearance: Additional clearance to add
        blocked_cells: Optional per-layer sets to collect blocked cells for visualization
        blocked_vias: Optional set to collect blocked via positions for visualization
        clearance_override: If provided, use this clearance instead of config.clearance
        skip_cell: Optional (gx, gy) -> bool predicate; cells for which it returns
            True are left unblocked (used for connector-region exemptions)
    """
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    # Sub-cell offset of the real pad center from its quantized cell, so blocking
    # is measured from the real center, not the grid cell (issue #70).
    off_x = pad.global_x - gx * coord.grid_step
    off_y = pad.global_y - gy * coord.grid_step
    half_width = pad.size_x / 2
    half_height = pad.size_y / 2
    clearance = clearance_override if clearance_override is not None else config.clearance
    # A pad's own local clearance (e.g. fiducial keep-clear rings) is a hard
    # keep-out floor -- honor it even when an inter-net effective-clearance
    # override was supplied (the main signal-routing loop always passes one),
    # so signal routes also stay outside it, not just plane copper.
    lc = getattr(pad, 'local_clearance', 0.0) or 0.0
    if lc > clearance:
        clearance = lc
    margin = config.track_width / 2 + clearance + extra_clearance

    # Custom comb/finger pads (issue #188): block the REAL copper polygon(s)
    # expanded by margin, leaving the finger channels open, instead of the
    # size_x x size_y bounding box (which fills the notches and the empty side of
    # an off-anchor pad, walling in the pads that route through them).
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        expanded_layers = expand_pad_layers(pad.layers, config.layers)
        layer_idxs = [layer_map[l] for l in expanded_layers if l in layer_map]
        on_copper = any(l.endswith('.Cu') for l in expanded_layers)
        via_margin = config.via_size / 2 + clearance + extra_clearance

        def _emit(poly, m, via_pass):
            gxf, gyf, inside, edist = _rasterize_polygon(poly, coord, m)
            if gxf is None:
                return
            mask = inside | (edist <= m)
            if skip_cell is not None and mask.any():
                idx = np.flatnonzero(mask)
                keep = np.fromiter((not skip_cell(int(gxf[i]), int(gyf[i])) for i in idx),
                                   dtype=bool, count=idx.size)
                mask = np.zeros_like(mask)
                mask[idx[keep]] = True
            if not mask.any():
                return
            if via_pass:
                cells = np.column_stack([gxf[mask], gyf[mask]])
                _batch_vias(obstacles, cells, blocked_vias)
            else:
                _block_cells_on_layers(obstacles, gxf, gyf, mask, layer_idxs)
                if blocked_cells is not None:
                    for li in layer_idxs:
                        if li < len(blocked_cells):
                            blocked_cells[li].update(zip(gxf[mask].tolist(), gyf[mask].tolist()))

        for poly in pad_polys:
            _emit(poly, margin, via_pass=False)
            if on_copper:
                _emit(poly, via_margin, via_pass=True)
        return

    # Compute corner radius based on pad shape:
    # - circle/oval: use min dimension to model as stadium/capsule shape
    # - roundrect: use the roundrect_rratio from pad
    # - rect: no rounding
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(half_width, half_height)
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0

    # Expand wildcard layers like "*.Cu" to actual routing layers
    expanded_layers = expand_pad_layers(pad.layers, config.layers)

    # Diff pair routing (extra_clearance > 0) generates the P/N tracks as
    # sub-grid offsets from the centerline, which adds a few um of deviation
    # on top of grid discretization - use a larger corner buffer so diagonal
    # passes by round pads cannot shave below the clearance
    corner_buffer = config.grid_step * 0.75 if extra_clearance > 0 else None

    # Batched rasterization (issue #35): pad_blocked_cells_array produces the
    # exact cell set of iter_pad_blocked_cells (verified bit-identical). The
    # rare skip_cell path (per-cell Python predicate, used for connector
    # exemptions) filters the array with the same predicate.
    layer_idxs = [layer_map[layer] for layer in expanded_layers if layer in layer_map]
    cells = pad_blocked_cells_array(gx, gy, half_width, half_height, margin,
                                    config.grid_step, corner_radius, corner_buffer,
                                    off_x, off_y, rotation_deg=pad.rect_rotation)
    if skip_cell is not None and len(cells):
        keep = np.fromiter((not skip_cell(int(cx), int(cy)) for cx, cy in cells),
                           dtype=bool, count=len(cells))
        cells = cells[keep]
    for layer_idx in layer_idxs:
        _batch_cells_one_layer(obstacles, cells, layer_idx, blocked_cells)

    # Via blocking near pads - block vias if pad is on any copper layer
    if any(layer.endswith('.Cu') for layer in expanded_layers):
        via_margin = config.via_size / 2 + clearance + extra_clearance
        via_cells = pad_blocked_cells_array(gx, gy, half_width, half_height, via_margin,
                                            config.grid_step, corner_radius, corner_buffer,
                                            off_x, off_y, rotation_deg=pad.rect_rotation)
        if skip_cell is not None and len(via_cells):
            keep = np.fromiter((not skip_cell(int(cx), int(cy)) for cx, cy in via_cells),
                               dtype=bool, count=len(via_cells))
            via_cells = via_cells[keep]
        _batch_vias(obstacles, via_cells, blocked_vias)


def _pad_via_keepout_cells(pad, coord: GridCoord, config: GridRouteConfig,
                           extra_clearance: float = 0.0):
    """The grid cells a via CENTER must avoid to clear `pad` (via half-size +
    clearance from the pad copper). Mirrors the via pass in _add_pad_obstacle so
    the keep-out geometry matches exactly. Returns an Nx2 int array, or None if the
    pad is on no copper layer (so it can't conflict with a through-via)."""
    expanded_layers = expand_pad_layers(pad.layers, config.layers)
    if not any(layer.endswith('.Cu') for layer in expanded_layers):
        return None
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    off_x = pad.global_x - gx * coord.grid_step
    off_y = pad.global_y - gy * coord.grid_step
    half_width = pad.size_x / 2
    half_height = pad.size_y / 2
    clearance = config.clearance
    lc = getattr(pad, 'local_clearance', 0.0) or 0.0
    if lc > clearance:
        clearance = lc
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(half_width, half_height)
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0
    corner_buffer = coord.grid_step * 0.75 if extra_clearance > 0 else None
    via_margin = config.via_size / 2 + clearance + extra_clearance
    return pad_blocked_cells_array(gx, gy, half_width, half_height, via_margin,
                                   config.grid_step, corner_radius, corner_buffer,
                                   off_x, off_y, rotation_deg=pad.rect_rotation)


def add_pads_via_keepout(obstacles: GridObstacleMap, pads: list,
                         config: GridRouteConfig, extra_clearance: float = 0.0):
    """Stamp a VIA-ONLY keep-out (no track keep-out) around each pad, so the
    router won't DROP a via within clearance of it while tracks may still pass.

    Issue #241: the diff-pair obstacle map excludes BOTH pair nets, so a coupled
    pair's single-ended leg can't see the PARTNER net's pads and parks its
    layer-transition via on one (the /SYZYGY1.C2P_CLK leg via grazing C2P_CLK_N's
    J4.36 pad). Adding the partner pads as a via keep-out for the duration of leg
    routing makes the leg A* place that via clear of them, while the coupled trace
    still runs close. Ref-counted -> pair with remove_pads_via_keepout."""
    coord = GridCoord(config.grid_step)
    for pad in pads:
        cells = _pad_via_keepout_cells(pad, coord, config, extra_clearance)
        if cells is not None and len(cells):
            obstacles.add_blocked_vias_batch(np.ascontiguousarray(cells.astype(np.int32)))


def remove_pads_via_keepout(obstacles: GridObstacleMap, pads: list,
                            config: GridRouteConfig, extra_clearance: float = 0.0):
    """Undo add_pads_via_keepout (decrements the ref-counted via keep-out)."""
    coord = GridCoord(config.grid_step)
    for pad in pads:
        cells = _pad_via_keepout_cells(pad, coord, config, extra_clearance)
        if cells is not None and len(cells):
            obstacles.remove_blocked_vias_batch(np.ascontiguousarray(cells.astype(np.int32)))


def build_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                       exclude_net_id: int, unrouted_stubs: Optional[List[Tuple[float, float]]] = None) -> GridObstacleMap:
    """Build Rust obstacle map from PCB data (legacy function for compatibility)."""
    # Build base map excluding just this net
    obstacles = build_base_obstacle_map(pcb_data, config, [exclude_net_id])

    # Add stub proximity costs
    if unrouted_stubs:
        add_stub_proximity_costs(obstacles, unrouted_stubs, config)

    # Add same-net via clearance blocking (for DRC - vias can't be too close even on same net)
    add_same_net_via_clearance(obstacles, pcb_data, exclude_net_id, config)

    # Add same-net pad drill hole-to-hole clearance blocking
    add_same_net_pad_drill_via_clearance(obstacles, pcb_data, exclude_net_id, config)

    return obstacles


# ============================================================================
# Visualization support - captures blocked cell data for rendering
# ============================================================================

@dataclass
class VisualizationData:
    """Data for visualization of obstacle map."""
    blocked_cells: List[Set[Tuple[int, int]]] = field(default_factory=list)  # Per-layer
    blocked_vias: Set[Tuple[int, int]] = field(default_factory=set)
    bga_zones_grid: List[Tuple[int, int, int, int]] = field(default_factory=list)
    bounds: Tuple[float, float, float, float] = (0, 0, 100, 100)  # min_x, min_y, max_x, max_y in mm


def build_base_obstacle_map_with_vis(pcb_data: PCBData, config: GridRouteConfig,
                                      nets_to_route: List[int],
                                      extra_clearance: float = 0.0,
                                      net_clearances: dict = None) -> Tuple[GridObstacleMap, VisualizationData]:
    """Build base obstacle map and capture visualization data.

    Same as build_base_obstacle_map, but also returns VisualizationData
    for rendering blocked cells in the visualizer.
    """
    if net_clearances is None:
        net_clearances = {}
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)
    nets_to_route_set = set(nets_to_route)

    # Use the maximum clearance of any net class to ensure proper spacing
    max_net_clearance = max(net_clearances.values()) if net_clearances else config.clearance
    effective_clearance = max(config.clearance, max_net_clearance)

    obstacles = GridObstacleMap(num_layers)

    # Visualization data
    blocked_cells: List[Set[Tuple[int, int]]] = [set() for _ in range(num_layers)]
    blocked_vias: Set[Tuple[int, int]] = set()
    bga_zones_grid: List[Tuple[int, int, int, int]] = []

    # Set BGA exclusion zones - block vias AND tracks on ALL layers
    # Set BGA proximity radius for vertical attraction exclusion
    bga_prox_radius_grid = coord.to_grid_dist(config.bga_proximity_radius)
    obstacles.set_bga_proximity_radius(bga_prox_radius_grid)

    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
        bga_zones_grid.append((gmin_x, gmin_y, gmax_x, gmax_y))
        for layer_idx in range(num_layers):
            for gx in range(gmin_x, gmax_x + 1):
                for gy in range(gmin_y, gmax_y + 1):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
                    blocked_cells[layer_idx].add((gx, gy))

    # Add BGA proximity costs (penalize routing near BGA edges)
    add_bga_proximity_costs(obstacles, config)

    # Add segments as obstacles (excluding nets we'll route)
    # Use actual segment width and layer-specific routing track width for proper clearance
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        # Compute expansion: layer-specific routing track half-width + obstacle half-width + clearance
        layer_track_width = config.get_track_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else layer_track_width
        expansion_mm = layer_track_width / 2 + seg_width / 2 + effective_clearance + extra_clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + effective_clearance + extra_clearance
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_mm, via_block_mm,
                              blocked_cells, blocked_vias)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, effective_clearance, extra_clearance)
        via_via_mm = via_size / 2 + config.via_size / 2 + effective_clearance
        # True via-via clearance radius in cells as a FLOAT (no floor): the disc
        # threshold is radius**2, so this blocks exactly the cells within the real
        # clearance. Flooring (to_grid_dist) lost up to ~1 cell and let two vias sit
        # a diagonal cell-offset too close (e.g. (3,2) cells = 0.36mm when 0.39mm is
        # required) -- a real cross-net via-via DRC violation the router never saw.
        via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid,
                          diagonal_margin=defaults.DIAGONAL_MARGIN, blocked_cells=blocked_cells, blocked_vias=blocked_vias)

    # Add pads as obstacles (excluding nets we'll route)
    # Use effective_clearance to ensure proper spacing between nets with different clearance requirements
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance,
                              blocked_cells, blocked_vias, clearance_override=effective_clearance)

    # Add board edge clearance
    add_board_edge_obstacles(obstacles, pcb_data, config, extra_clearance)

    # Add user-drawn keepout polygons (#27) and KiCad keep-out rule areas (#25),
    # mirroring build_base_obstacle_map so the visualize path blocks them too.
    add_user_keepout_obstacles(obstacles, pcb_data, config, coord, num_layers)
    add_rule_area_keepout_obstacles(obstacles, pcb_data, config)

    # Add hole-to-hole clearance blocking for existing drills
    add_drill_hole_obstacles(obstacles, pcb_data, config, nets_to_route_set,
                             extra_clearance)

    vis_data = VisualizationData(
        blocked_cells=blocked_cells,
        blocked_vias=blocked_vias,
        bga_zones_grid=bga_zones_grid
    )

    return obstacles, vis_data


def check_line_clearance(obstacles: GridObstacleMap,
                         x1: float, y1: float,
                         x2: float, y2: float,
                         layer_idx: int,
                         config: GridRouteConfig) -> bool:
    """Check if a line segment from (x1,y1) to (x2,y2) is clear of track obstacles on the given layer.

    Uses fine sampling (half grid step) to ensure complete coverage.
    Returns True if the path is clear, False if any cell is blocked.
    """
    coord = GridCoord(config.grid_step)

    dx = x2 - x1
    dy = y2 - y1
    length = (dx * dx + dy * dy) ** 0.5

    if length < 0.001:
        # Point check only
        gx, gy = coord.to_grid(x1, y1)
        return not obstacles.is_blocked(gx, gy, layer_idx)

    # Normalize direction
    dir_x = dx / length
    dir_y = dy / length

    # Sample at half grid step for better coverage
    step = config.grid_step / 2
    checked = set()

    dist = 0.0
    while dist <= length:
        x = x1 + dir_x * dist
        y = y1 + dir_y * dist
        gx, gy = coord.to_grid(x, y)

        if (gx, gy) not in checked:
            checked.add((gx, gy))
            if obstacles.is_blocked(gx, gy, layer_idx):
                return False

        dist += step

    return True


def add_connector_region_via_blocking(obstacles: GridObstacleMap,
                                       center_x: float, center_y: float,
                                       dir_x: float, dir_y: float,
                                       setback_distance: float,
                                       spacing_mm: float,
                                       config: GridRouteConfig,
                                       debug: bool = False):
    """Block vias in the connector region between stub center and setback position.

    The connector region extends from the stub center in the stub direction
    to the setback distance plus margin. This region needs to remain clear
    for the angled connector segments that will be added after routing.
    Vias in this region cause DRC errors due to conflicts with the turn segments.

    Args:
        obstacles: The obstacle map to add via blocking to
        center_x, center_y: Center point between P and N stubs (in mm)
        dir_x, dir_y: Normalized direction from stubs toward route
        setback_distance: Distance from stub center to route start (in mm)
        spacing_mm: Half-spacing between P and N tracks (in mm)
        config: Routing configuration
        debug: If True, print debug info about blocked region
    """
    coord = GridCoord(config.grid_step)

    # Block from stub center to setback + margin for via geometry
    # The margin accounts for via size and clearance requirements
    margin = config.via_size + config.clearance
    total_distance = setback_distance + margin

    # Corridor width should accommodate both P and N tracks plus clearance
    # The turn segments extend perpendicular to the stub direction
    corridor_half_width = spacing_mm + config.track_width / 2 + config.via_size / 2 + config.clearance

    # Perpendicular direction for corridor width
    perp_x = -dir_y
    perp_y = dir_x

    if debug:
        print(f"    Blocking corridor: center=({center_x:.2f},{center_y:.2f}), "
              f"dir=({dir_x:.2f},{dir_y:.2f}), dist={total_distance:.2f}mm, "
              f"half_width={corridor_half_width:.3f}mm")

    # Sample points along the connector region and block vias
    # Use half grid step for better coverage of diagonal corridors
    # (diagonal directions can miss grid cells when using full grid step)
    step = config.grid_step / 2
    dist = 0.0
    blocked_set = set()  # Track unique grid positions to avoid duplicates
    while dist <= total_distance:
        # Center of corridor at this distance
        cx = center_x + dir_x * dist
        cy = center_y + dir_y * dist

        # Block vias across the corridor width (also using half step for width)
        width_step = config.grid_step / 2
        width_steps = int(corridor_half_width / width_step) + 1
        for w in range(-width_steps, width_steps + 1):
            px = cx + perp_x * w * width_step
            py = cy + perp_y * w * width_step
            gx, gy = coord.to_grid(px, py)
            if (gx, gy) not in blocked_set:
                blocked_set.add((gx, gy))
                obstacles.add_blocked_via(gx, gy)

        dist += step

    if debug:
        print(f"    Blocked {len(blocked_set)} via positions")


def get_net_bounds(pcb_data: PCBData, net_ids: List[int], padding: float = 5.0) -> Tuple[float, float, float, float]:
    """Get bounding box around all the nets' components.

    Returns (min_x, min_y, max_x, max_y) in mm.
    """
    xs = []
    ys = []

    for net_id in net_ids:
        pads = pcb_data.pads_by_net.get(net_id, [])
        segments = [s for s in pcb_data.segments if s.net_id == net_id]

        for pad in pads:
            xs.append(pad.global_x)
            ys.append(pad.global_y)

        for seg in segments:
            xs.extend([seg.start_x, seg.end_x])
            ys.extend([seg.start_y, seg.end_y])

    if not xs or not ys:
        return (0, 0, 100, 100)

    return (min(xs) - padding, min(ys) - padding, max(xs) + padding, max(ys) + padding)


def draw_exclusion_zones_debug(config: GridRouteConfig,
                                unrouted_stubs: List[Tuple[float, float]] = None) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Get exclusion zone outline lines for User.5 layer debugging.

    Returns line segments for:
    - Circles around stub proximity zones
    - Rectangles around BGA exclusion zones (inner and outer with proximity radius)

    Args:
        config: Routing configuration with exclusion zone settings
        unrouted_stubs: List of (x, y) or (x, y, layer) tuples for stub positions

    Returns:
        List of ((x1, y1), (x2, y2)) line segment tuples
    """
    import math

    lines = []

    # Draw BGA exclusion zone rectangles and proximity rectangles
    prox_radius = config.bga_proximity_radius
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        # Draw inner rectangle (BGA zone itself)
        corners = [
            (min_x, min_y), (max_x, min_y),
            (max_x, max_y), (min_x, max_y)
        ]
        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            lines.append(((x1, y1), (x2, y2)))

        # Draw outer rectangle (BGA zone expanded by proximity radius)
        if prox_radius > 0:
            outer_corners = [
                (min_x - prox_radius, min_y - prox_radius),
                (max_x + prox_radius, min_y - prox_radius),
                (max_x + prox_radius, max_y + prox_radius),
                (min_x - prox_radius, max_y + prox_radius)
            ]
            for i in range(4):
                x1, y1 = outer_corners[i]
                x2, y2 = outer_corners[(i + 1) % 4]
                lines.append(((x1, y1), (x2, y2)))

    # Draw stub proximity circles
    if unrouted_stubs and config.stub_proximity_radius > 0:
        radius = config.stub_proximity_radius
        num_segments = 16  # Circle approximation segments

        for stub in unrouted_stubs:
            cx, cy = stub[0], stub[1]

            # Draw circle as connected line segments
            for i in range(num_segments):
                angle1 = 2 * math.pi * i / num_segments
                angle2 = 2 * math.pi * (i + 1) / num_segments
                x1 = cx + radius * math.cos(angle1)
                y1 = cy + radius * math.sin(angle1)
                x2 = cx + radius * math.cos(angle2)
                y2 = cy + radius * math.sin(angle2)
                lines.append(((x1, y1), (x2, y2)))

    return lines
