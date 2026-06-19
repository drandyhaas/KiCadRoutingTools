"""
Obstacle map building for copper plane via placement and routing.

Provides functions to build obstacle maps for:
- Via placement (considering all layers)
- Single-layer routing (for via-to-pad traces)
"""

import math
from typing import List, Dict, Tuple, Optional

import numpy as np

from kicad_parser import PCBData, Pad, Segment
from routing_config import GridRouteConfig, GridCoord
from routing_utils import iter_pad_blocked_cells
from obstacle_map import (point_in_polygon, point_to_polygon_edge_distance,
                          add_user_keepout_obstacles, add_rule_area_keepout_obstacles,
                          block_via_cells_near_drills)

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap


def _precompute_circle_offsets(radius_sq: float) -> np.ndarray:
    """Pre-compute circle offsets for a given squared radius.

    Returns numpy array of shape (N, 2) with (dx, dy) offsets.
    """
    radius_int = int(math.ceil(math.sqrt(radius_sq)))
    offsets = []
    for ex in range(-radius_int, radius_int + 1):
        for ey in range(-radius_int, radius_int + 1):
            if ex * ex + ey * ey <= radius_sq:
                offsets.append((ex, ey))
    return np.array(offsets, dtype=np.int32)


def _bresenham_centers(gx1: int, gy1: int, gx2: int, gy2: int) -> List[Tuple[int, int]]:
    """Walk a Bresenham line and return all grid center points."""
    centers = []
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    x, y = gx1, gy1
    if dx > dy:
        err = dx / 2
        while x != gx2:
            centers.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != gy2:
            centers.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    centers.append((gx2, gy2))
    return centers


def _batch_block_circles_via(obstacles: GridObstacleMap, centers: List[Tuple[int, int]],
                              circle_offsets: np.ndarray):
    """Block via positions for multiple centers using batched numpy operations."""
    if not centers:
        return
    centers_arr = np.array(centers, dtype=np.int32)  # (N, 2)
    # Broadcast: (N, 1, 2) + (1, K, 2) -> (N, K, 2) -> (N*K, 2)
    all_cells = (centers_arr[:, np.newaxis, :] + circle_offsets[np.newaxis, :, :]).reshape(-1, 2)
    obstacles.add_blocked_vias_batch(all_cells)


def _batch_block_circles_cell(obstacles: GridObstacleMap, centers: List[Tuple[int, int]],
                               circle_offsets: np.ndarray, layer_idx: int):
    """Block cells for multiple centers using batched numpy operations."""
    if not centers:
        return
    centers_arr = np.array(centers, dtype=np.int32)  # (N, 2)
    all_cells = (centers_arr[:, np.newaxis, :] + circle_offsets[np.newaxis, :, :]).reshape(-1, 2)
    layer_col = np.full((all_cells.shape[0], 1), layer_idx, dtype=np.int32)
    all_cells_3 = np.hstack([all_cells, layer_col])  # (N*K, 3)
    obstacles.add_blocked_cells_batch(all_cells_3)


def block_circle(obstacles: GridObstacleMap, cx: int, cy: int, radius_sq: float,
                 layer_idx: Optional[int] = None, via_mode: bool = False):
    """Block cells in a circular region.

    Args:
        obstacles: The obstacle map to update
        cx, cy: Center of the circle in grid coordinates
        radius_sq: Squared radius in grid units (avoids precision loss)
        layer_idx: Layer index for routing obstacles (ignored if via_mode=True)
        via_mode: If True, use add_blocked_via; if False, use add_blocked_cell
    """
    radius_int = int(math.ceil(math.sqrt(radius_sq)))
    for ex in range(-radius_int, radius_int + 1):
        for ey in range(-radius_int, radius_int + 1):
            if ex*ex + ey*ey <= radius_sq:
                if via_mode:
                    obstacles.add_blocked_via(cx + ex, cy + ey)
                else:
                    obstacles.add_blocked_cell(cx + ex, cy + ey, layer_idx)


def _point_in_pad_copper(pad: Pad, x: float, y: float, extra: float = 0.0) -> bool:
    """Return True if (x, y) lies within the pad's copper rectangle (rotation-aware).

    `extra` expands the pad bounds, e.g. by a via radius so that a via whose
    copper overlaps the pad edge still counts as touching.
    """
    dx = x - pad.global_x
    dy = y - pad.global_y
    # size_x/size_y are board-resolved; the rectangle's residual tilt is
    # rect_rotation (0 for orthogonal pads), not the total pad rotation.
    rot = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    cos_r = math.cos(rot)
    sin_r = math.sin(rot)
    lx = dx * cos_r + dy * sin_r
    ly = -dx * sin_r + dy * cos_r
    return (abs(lx) <= pad.size_x / 2 + extra and
            abs(ly) <= pad.size_y / 2 + extra)


def _smd_pad_reaches_layer(pad: Pad, target_layer: str, net_id: int,
                            pcb_data: PCBData, tolerance: float = 0.01) -> bool:
    """Return True if the SMD `pad` already has an electrical path to
    `target_layer` through existing same-net tracks, vias, and pads.

    Used by identify_target_pads to skip placing a stitching via for pads
    that the plane zone will already pick up via existing copper. The
    walk is over (position, layer) states:
      - Same-net segment endpoints connect their two states.
      - Same-net via positions connect all layers in the via's span.
      - Same-net through-hole pad positions connect all copper layers.

    The walk is seeded from the pad center AND from any same-net via or
    segment endpoint that lands inside the pad's copper (route_planes places
    in-pad stitching vias off-center, which previously went undetected and
    caused duplicate taps on re-runs, issue #104).
    """
    pad_layer = None
    for layer in pad.layers:
        if layer.endswith('.Cu') and not layer.startswith('*'):
            pad_layer = layer
            break
    if pad_layer is None:
        return False
    if pad_layer == target_layer:
        return True

    inv_tol = 1.0 / tolerance
    def pkey(x: float, y: float):
        return (round(x * inv_tol), round(y * inv_tol))

    if pcb_data.board_info and getattr(pcb_data.board_info, 'copper_layers', None):
        all_cu = [l for l in pcb_data.board_info.copper_layers if l.endswith('.Cu')]
    else:
        all_cu = ['F.Cu', 'B.Cu']

    # Segment adjacency keyed by (pos_key, layer).
    seg_adj: Dict[Tuple, set] = {}
    for s in pcb_data.segments:
        if s.net_id != net_id:
            continue
        k1 = (pkey(s.start_x, s.start_y), s.layer)
        k2 = (pkey(s.end_x, s.end_y), s.layer)
        seg_adj.setdefault(k1, set()).add(k2)
        seg_adj.setdefault(k2, set()).add(k1)

    def via_layers(v) -> List[str]:
        """Copper layers a via connects. A through via is written with
        (layers F.Cu B.Cu) in KiCad but spans every copper layer."""
        if not v.layers or ('F.Cu' in v.layers and 'B.Cu' in v.layers):
            return all_cu
        return v.layers

    # Via vertical jumps keyed by pos_key -> set of layers.
    via_at: Dict[Tuple, set] = {}
    for v in pcb_data.vias:
        if v.net_id != net_id:
            continue
        k = pkey(v.x, v.y)
        via_at.setdefault(k, set()).update(via_layers(v))

    # Pads at each position - through-hole pads bridge all copper layers.
    pad_at: Dict[Tuple, set] = {}
    for p in pcb_data.pads_by_net.get(net_id, []):
        k = pkey(p.global_x, p.global_y)
        if p.drill > 0:
            pad_at.setdefault(k, set()).update(all_cu)
        else:
            for pl in p.layers:
                if pl == '*.Cu':
                    pad_at.setdefault(k, set()).update(all_cu)
                elif pl.endswith('.Cu') and not pl.startswith('*'):
                    pad_at.setdefault(k, set()).add(pl)

    start = (pkey(pad.global_x, pad.global_y), pad_layer)
    visited = {start}
    queue = [start]

    # Seed from same-net copper that lands inside this pad's copper:
    # - a via overlapping the pad connects the pad to all the via's layers
    # - a segment endpoint inside the pad (on the pad's layer) connects there
    for v in pcb_data.vias:
        if v.net_id != net_id:
            continue
        if _point_in_pad_copper(pad, v.x, v.y, extra=v.size / 2):
            for vl in via_layers(v):
                state = (pkey(v.x, v.y), vl)
                if state not in visited:
                    visited.add(state)
                    queue.append(state)
    for s in pcb_data.segments:
        if s.net_id != net_id or s.layer != pad_layer:
            continue
        for ex, ey in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if _point_in_pad_copper(pad, ex, ey):
                state = (pkey(ex, ey), pad_layer)
                if state not in visited:
                    visited.add(state)
                    queue.append(state)

    while queue:
        pos, layer = queue.pop(0)
        if layer == target_layer:
            return True
        for nxt in seg_adj.get((pos, layer), ()):
            if nxt not in visited:
                visited.add(nxt)
                queue.append(nxt)
        for other_layer in via_at.get(pos, ()):
            nxt = (pos, other_layer)
            if nxt not in visited:
                visited.add(nxt)
                queue.append(nxt)
        for other_layer in pad_at.get(pos, ()):
            nxt = (pos, other_layer)
            if nxt not in visited:
                visited.add(nxt)
                queue.append(nxt)
    return False


def identify_target_pads(
    pcb_data: PCBData,
    net_id: int,
    plane_layer: str
) -> List[Dict]:
    """
    Identify pads that need via connections to the plane layer.

    Returns list of dicts with pad info and connection type:
    - "through_hole": Through-hole pad - can connect on any layer, no via needed
    - "direct": SMD pad on plane layer - zone connects directly, no via needed
    - "already_connected": SMD pad on another layer but already reaches the
       plane layer via existing tracks/vias/pads on the same net - no via needed
    - "via_needed": SMD pad on opposite layer with no existing path - needs via + trace
    """
    target_pads = []
    pads = pcb_data.pads_by_net.get(net_id, [])

    for pad in pads:
        # Check if pad has drill (through-hole)
        if pad.drill > 0:
            # Through-hole pad - directly connects to all layers including plane
            target_pads.append({
                'pad': pad,
                'type': 'through_hole',
                'needs_via': False,
                'needs_trace': False
            })
        elif plane_layer in pad.layers or "*.Cu" in pad.layers:
            # SMD pad on plane layer - direct zone connection
            target_pads.append({
                'pad': pad,
                'type': 'direct',
                'needs_via': False,
                'needs_trace': False
            })
        else:
            # SMD pad NOT on plane layer - needs via
            # Get the pad's actual layer for trace routing
            pad_layer = None
            for layer in pad.layers:
                if layer.endswith('.Cu') and not layer.startswith('*'):
                    pad_layer = layer
                    break

            # Skip placing a stitching via if the pad already has an
            # electrical path to the plane layer through existing same-net
            # tracks, vias, and through-hole pads.
            if _smd_pad_reaches_layer(pad, plane_layer, net_id, pcb_data):
                target_pads.append({
                    'pad': pad,
                    'type': 'already_connected',
                    'needs_via': False,
                    'needs_trace': False,
                    'pad_layer': pad_layer,
                })
            else:
                target_pads.append({
                    'pad': pad,
                    'type': 'via_needed',
                    'needs_via': True,
                    'needs_trace': True,  # May need trace if via can't be at pad center
                    'pad_layer': pad_layer
                })

    return target_pads


def build_via_obstacle_map(
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int,
    verbose: bool = True,
    same_net_pad_clearance: float = -1.0,
) -> GridObstacleMap:
    """
    Build obstacle map for via placement.

    Blocks:
    - Existing vias (all nets - via-via clearance)
    - Pads on all layers (except target net pads)
    - Tracks on all layers (except target net)
    - Board edge clearance
    - Through-hole pad drills (hole-to-hole clearance)

    Args:
        same_net_pad_clearance: If >= 0, also block target-net pads as via obstacles
            using this edge-to-edge clearance (in mm) in place of config.clearance.
            -1 (default) leaves same-net pads unblocked, allowing via-in-pad placement.
    """
    import time
    t_start = time.time()

    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    # Calculate grid dimensions for info
    board_bounds = pcb_data.board_info.board_bounds
    if board_bounds:
        min_x, min_y, max_x, max_y = board_bounds
        grid_w = int((max_x - min_x) / config.grid_step)
        grid_h = int((max_y - min_y) / config.grid_step)
        if verbose:
            print(f"  Grid: {grid_w} x {grid_h} = {grid_w * grid_h:,} cells (grid_step={config.grid_step}mm)")

    obstacles = GridObstacleMap(num_layers)

    # Precompute expansion for via-via clearance (squared, in grid units)
    # Add half grid step cushion to account for grid discretization
    grid_cushion = config.grid_step / 2
    via_via_expansion_mm = config.via_size + config.clearance + grid_cushion
    via_via_radius_sq = (via_via_expansion_mm / config.grid_step) ** 2
    via_via_radius_int = int(math.ceil(math.sqrt(via_via_radius_sq)))

    # Add existing vias as obstacles (including same net - can't place via too close to another)
    # Batched: pre-compute circle template, collect all centers, expand with numpy
    t0 = time.time()
    circle_offsets = _precompute_circle_offsets(via_via_radius_sq)
    via_centers = [(coord.to_grid(via.x, via.y)) for via in pcb_data.vias]
    _batch_block_circles_via(obstacles, via_centers, circle_offsets)
    if verbose:
        print(f"  Vias: {len(pcb_data.vias)} vias in {time.time() - t0:.2f}s")

    # Add existing segments as obstacles (via can't overlap with tracks on ANY layer)
    # Since vias span all layers, we must check segments on all copper layers, not just config.layers
    t0 = time.time()
    seg_count = 0
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        # Include any copper layer (*.Cu)
        if not seg.layer.endswith('.Cu'):
            continue
        # Use actual segment width for clearance calculation (not config.track_width)
        # Include grid cushion for discretization
        seg_expansion_mm = config.via_size / 2 + seg.width / 2 + config.clearance + grid_cushion
        _add_segment_via_obstacle(obstacles, seg, coord, seg_expansion_mm)
        seg_count += 1
    if verbose:
        print(f"  Segments: {seg_count} tracks in {time.time() - t0:.2f}s")

    # Add pads as obstacles (excluding target net pads, unless same_net_pad_clearance >= 0)
    t0 = time.time()
    pad_count = 0
    for net_id, pads in pcb_data.pads_by_net.items():
        is_target_net = (net_id == exclude_net_id)
        if is_target_net and same_net_pad_clearance < 0:
            continue
        pad_clearance = same_net_pad_clearance if is_target_net else None
        for pad in pads:
            _add_pad_via_obstacle(obstacles, pad, coord, config, clearance_override=pad_clearance)
            pad_count += 1
    if verbose:
        print(f"  Pads: {pad_count} pads in {time.time() - t0:.2f}s")

    # Add board edge via blocking
    t0 = time.time()
    _add_board_edge_via_obstacles(obstacles, pcb_data, config)
    if verbose:
        print(f"  Board edge: {time.time() - t0:.2f}s")

    # Add hole-to-hole clearance blocking for existing drills
    t0 = time.time()
    _add_drill_hole_via_obstacles(obstacles, pcb_data, config, exclude_net_id)
    if verbose:
        print(f"  Drill holes: {time.time() - t0:.2f}s")

    # Keep stitching vias out of user-drawn keepouts (#27) and KiCad keep-out
    # rule areas (#25).
    add_user_keepout_obstacles(obstacles, pcb_data, config, coord, num_layers)
    add_rule_area_keepout_obstacles(obstacles, pcb_data, config)

    if verbose:
        print(f"  Total obstacle build: {time.time() - t_start:.2f}s")

    return obstacles


def _add_segment_via_obstacle(obstacles: GridObstacleMap, seg: Segment,
                               coord: GridCoord, expansion_mm: float):
    """Add a segment as via blocking obstacle using batched numpy operations."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    radius_sq = (expansion_mm / coord.grid_step) ** 2
    circle_offsets = _precompute_circle_offsets(radius_sq)
    centers = _bresenham_centers(gx1, gy1, gx2, gy2)
    _batch_block_circles_via(obstacles, centers, circle_offsets)


def _add_pad_via_obstacle(obstacles: GridObstacleMap, pad: Pad,
                           coord: GridCoord, config: GridRouteConfig,
                           clearance_override: float = None):
    """Add a pad as via blocking obstacle using rectangular shape with rounded corners.

    clearance_override: if not None, use this edge-to-edge clearance instead of
        config.clearance (used for same-net pads when same_net_pad_clearance is set).
    """
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    half_width = pad.size_x / 2
    half_height = pad.size_y / 2
    # Add half grid step buffer to account for grid quantization errors
    clearance = config.clearance if clearance_override is None else clearance_override
    margin = config.via_size / 2 + clearance + config.grid_step / 2
    # Corner radius based on pad shape (circle/oval use min dimension, roundrect uses rratio)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(half_width, half_height)
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0

    for cell_gx, cell_gy in iter_pad_blocked_cells(gx, gy, half_width, half_height, margin, config.grid_step, corner_radius,
                                                   off_x=pad.global_x - gx * coord.grid_step,
                                                   off_y=pad.global_y - gy * coord.grid_step,
                                                   rotation_deg=pad.rect_rotation):
        obstacles.add_blocked_via(cell_gx, cell_gy)


def _is_rectangular_outline(board_outline: List[Tuple[float, float]],
                            board_bounds: Tuple[float, float, float, float],
                            tolerance: float = 0.1) -> bool:
    """Check if board outline is approximately rectangular.

    Returns True if all vertices are within tolerance of the bounding box corners.
    """
    if not board_outline or len(board_outline) < 4:
        return False

    min_x, min_y, max_x, max_y = board_bounds
    corners = [(min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y)]

    # Check each vertex - must be on an edge of the bounding box
    for vx, vy in board_outline:
        on_edge = (abs(vx - min_x) < tolerance or abs(vx - max_x) < tolerance or
                   abs(vy - min_y) < tolerance or abs(vy - max_y) < tolerance)
        if not on_edge:
            return False
    return True


def _add_board_edge_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig):
    """Block via placement near board edges.

    Supports both rectangular and non-rectangular board outlines.
    Optimized to only check cells near the boundary, not the entire grid.
    """
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return

    coord = GridCoord(config.grid_step)
    min_x, min_y, max_x, max_y = board_bounds

    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    via_edge_clearance = edge_clearance + config.via_size / 2
    via_expand = coord.to_grid_dist(via_edge_clearance)

    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    grid_margin = via_expand + 5

    # Check for non-rectangular board outline
    board_outline = pcb_data.board_info.board_outline
    use_polygon = board_outline and len(board_outline) >= 3 and not _is_rectangular_outline(board_outline, board_bounds)

    if use_polygon:
        # Polygon-based via blocking for non-rectangular boards
        # Optimization: only check cells in a band near the boundary, not the entire grid
        band_width = via_expand + 10  # Check cells within this distance of bounding box edges

        # Block all cells outside bounding box (fast)
        for gx in range(gmin_x - grid_margin, gmax_x + grid_margin + 1):
            for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
                if gx < gmin_x or gx > gmax_x or gy < gmin_y or gy > gmax_y:
                    obstacles.add_blocked_via(gx, gy)

        # For cells near the boundary, do detailed polygon check
        for gx in range(gmin_x, gmax_x + 1):
            for gy in range(gmin_y, gmax_y + 1):
                # Skip cells far from any edge (in interior)
                dist_to_edge = min(gx - gmin_x, gmax_x - gx, gy - gmin_y, gmax_y - gy)
                if dist_to_edge > band_width:
                    continue

                x, y = coord.to_float(gx, gy)
                inside = point_in_polygon(x, y, board_outline)
                if not inside:
                    obstacles.add_blocked_via(gx, gy)
                else:
                    edge_dist = point_to_polygon_edge_distance(x, y, board_outline)
                    if edge_dist < via_edge_clearance:
                        obstacles.add_blocked_via(gx, gy)
    else:
        # Rectangular board - use simple bounding box logic
        for gx in range(gmin_x - grid_margin, gmax_x + grid_margin + 1):
            for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
                # Outside board + margin
                if gx < gmin_x + via_expand or gx > gmax_x - via_expand or \
                   gy < gmin_y + via_expand or gy > gmax_y - via_expand:
                    obstacles.add_blocked_via(gx, gy)

    # Block vias inside board cutouts
    for cutout in pcb_data.board_info.board_cutouts:
        if len(cutout) < 3:
            continue
        cut_xs = [p[0] for p in cutout]
        cut_ys = [p[1] for p in cutout]
        cg_min_x, cg_min_y = coord.to_grid(min(cut_xs) - via_edge_clearance, min(cut_ys) - via_edge_clearance)
        cg_max_x, cg_max_y = coord.to_grid(max(cut_xs) + via_edge_clearance, max(cut_ys) + via_edge_clearance)
        for gx in range(cg_min_x, cg_max_x + 1):
            for gy in range(cg_min_y, cg_max_y + 1):
                x, y = coord.to_float(gx, gy)
                if point_in_polygon(x, y, cutout):
                    obstacles.add_blocked_via(gx, gy)
                else:
                    edge_dist = point_to_polygon_edge_distance(x, y, cutout)
                    if edge_dist < via_edge_clearance:
                        obstacles.add_blocked_via(gx, gy)


def _add_board_edge_track_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                     config: GridRouteConfig, layer_idx: int):
    """Block track routing near board edges on a single layer.

    Supports both rectangular and non-rectangular board outlines.
    Optimized to only check cells near the boundary, not the entire grid.
    """
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return

    coord = GridCoord(config.grid_step)
    min_x, min_y, max_x, max_y = board_bounds

    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    track_edge_clearance = edge_clearance + config.track_width / 2
    track_expand = coord.to_grid_dist(track_edge_clearance)

    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    grid_margin = track_expand + 5

    # Check for non-rectangular board outline
    board_outline = pcb_data.board_info.board_outline
    use_polygon = board_outline and len(board_outline) >= 3 and not _is_rectangular_outline(board_outline, board_bounds)

    if use_polygon:
        # Polygon-based track blocking for non-rectangular boards
        # Optimization: only check cells in a band near the boundary, not the entire grid
        band_width = track_expand + 10

        # Block all cells outside bounding box (fast)
        for gx in range(gmin_x - grid_margin, gmax_x + grid_margin + 1):
            for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
                if gx < gmin_x or gx > gmax_x or gy < gmin_y or gy > gmax_y:
                    obstacles.add_blocked_cell(gx, gy, layer_idx)

        # For cells near the boundary, do detailed polygon check
        for gx in range(gmin_x, gmax_x + 1):
            for gy in range(gmin_y, gmax_y + 1):
                # Skip cells far from any edge (in interior)
                dist_to_edge = min(gx - gmin_x, gmax_x - gx, gy - gmin_y, gmax_y - gy)
                if dist_to_edge > band_width:
                    continue

                x, y = coord.to_float(gx, gy)
                inside = point_in_polygon(x, y, board_outline)
                if not inside:
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
                else:
                    edge_dist = point_to_polygon_edge_distance(x, y, board_outline)
                    if edge_dist < track_edge_clearance:
                        obstacles.add_blocked_cell(gx, gy, layer_idx)
    else:
        # Rectangular board - use simple bounding box logic
        for gx in range(gmin_x - grid_margin, gmax_x + grid_margin + 1):
            for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
                # Outside board + margin
                if gx < gmin_x + track_expand or gx > gmax_x - track_expand or \
                   gy < gmin_y + track_expand or gy > gmax_y - track_expand:
                    obstacles.add_blocked_cell(gx, gy, layer_idx)

    # Block tracks inside board cutouts
    for cutout in pcb_data.board_info.board_cutouts:
        if len(cutout) < 3:
            continue
        cut_xs = [p[0] for p in cutout]
        cut_ys = [p[1] for p in cutout]
        cg_min_x, cg_min_y = coord.to_grid(min(cut_xs) - track_edge_clearance, min(cut_ys) - track_edge_clearance)
        cg_max_x, cg_max_y = coord.to_grid(max(cut_xs) + track_edge_clearance, max(cut_ys) + track_edge_clearance)
        for gx in range(cg_min_x, cg_max_x + 1):
            for gy in range(cg_min_y, cg_max_y + 1):
                x, y = coord.to_float(gx, gy)
                if point_in_polygon(x, y, cutout):
                    obstacles.add_blocked_cell(gx, gy, layer_idx)
                else:
                    edge_dist = point_to_polygon_edge_distance(x, y, cutout)
                    if edge_dist < track_edge_clearance:
                        obstacles.add_blocked_cell(gx, gy, layer_idx)


def _add_drill_hole_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig, exclude_net_id: int):
    """Block via placement near existing drill holes."""
    if config.hole_to_hole_clearance <= 0:
        return

    # Collect drill holes
    drill_holes = []

    for via in pcb_data.vias:
        drill_holes.append((via.x, via.y, via.drill))

    # Hole-to-hole is a physical drill-to-drill minimum, independent of net, so
    # include the plane net's OWN through-hole pads too (exclude_net_id is NOT
    # skipped here). Otherwise a stitching via can land within the fab
    # hole-to-hole minimum of a same-net TH pad -- issue #125
    # (PAD-DRILL-VIA-DRILL-SAME-NET). A same-net TH pad already reaches the
    # plane through its own barrel, so blocking vias around it costs nothing.
    for net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            if pad.drill > 0:
                drill_holes.append((pad.global_x, pad.global_y, pad.drill))

    # Enforce the keepout by REAL mm distance from each drill centre (shared with
    # the signal router) rather than a disk centred on the quantized drill cell,
    # so a stitching via cannot land a sub-cell inside the minimum (issue #70).
    block_via_cells_near_drills(obstacles, drill_holes, config.via_drill,
                                config.hole_to_hole_clearance, config.grid_step)


def block_via_position(obstacles: GridObstacleMap, via_x: float, via_y: float,
                        coord: GridCoord, hole_to_hole_clearance: float, via_drill: float,
                        via_size: float = None, clearance: float = None):
    """Block the area around a newly placed via so the next same-net via clears it.

    A second via must clear this one on BOTH measures: drill hole-to-hole AND
    via-ring copper-to-copper. Blocking only the drill distance
    (via_drill + hole_to_hole) is smaller than the copper distance
    (via_size + clearance) for these planes, so it let the pour drop a second
    stitching via close enough to trip same-net via-via copper DRC (the GND/+3V3
    overlaps). When via_size/clearance are given, block the larger of the two.

    Args:
        obstacles: The obstacle map to update
        via_x, via_y: Position of the placed via
        coord: Grid coordinate converter
        hole_to_hole_clearance: Minimum clearance between drill holes
        via_drill: Drill diameter of the via
        via_size: Via outer (copper) diameter; with clearance, also enforce via-via copper clearance
        clearance: Copper-to-copper clearance
    """
    gx, gy = coord.to_grid(via_x, via_y)
    # Drill hole-to-hole: (this_drill/2)+(other_drill/2)+clearance = via_drill + clearance
    required_dist = via_drill + hole_to_hole_clearance
    # Via-ring copper-to-copper (same-size vias): (size/2)+(size/2)+clearance = via_size + clearance
    if via_size is not None and clearance is not None:
        required_dist = max(required_dist, via_size + clearance)
    radius_sq = (required_dist / coord.grid_step) ** 2
    block_circle(obstacles, gx, gy, radius_sq, via_mode=True)


def build_routing_obstacle_map(
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int,
    route_layer: str,
    skip_pad_blocking: bool = False,
    verbose: bool = True
) -> GridObstacleMap:
    """
    Build obstacle map for A* routing on a specific layer.

    Blocks:
    - Pads on the route layer (except target net pads) - skipped if skip_pad_blocking
    - Segments on the route layer (except target net)
    - Vias (they occupy space on all layers)

    Args:
        skip_pad_blocking: If True, don't block based on pad clearances.
                          Use this for plane via connections where DRC is lenient.
        verbose: If True, print timing information for each step.
    """
    import time
    t_start = time.time()

    coord = GridCoord(config.grid_step)
    # Single layer routing
    num_layers = 1
    layer_idx = 0

    # Calculate grid dimensions for info
    board_bounds = pcb_data.board_info.board_bounds
    if board_bounds and verbose:
        min_x, min_y, max_x, max_y = board_bounds
        grid_w = int((max_x - min_x) / config.grid_step)
        grid_h = int((max_y - min_y) / config.grid_step)
        print(f"  Routing grid ({route_layer}): {grid_w} x {grid_h} = {grid_w * grid_h:,} cells")

    obstacles = GridObstacleMap(num_layers)

    # Add pads on this layer as obstacles (excluding target net)
    # Skip this entirely for plane connections where we're more lenient
    t0 = time.time()
    pad_count = 0
    if not skip_pad_blocking:
        for net_id, pads in pcb_data.pads_by_net.items():
            if net_id == exclude_net_id:
                continue
            for pad in pads:
                # Check if pad is on the route layer
                if route_layer in pad.layers or "*.Cu" in pad.layers:
                    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                    half_width = pad.size_x / 2
                    half_height = pad.size_y / 2
                    margin = config.track_width / 2 + config.clearance
                    # Corner radius based on pad shape
                    if pad.shape in ('circle', 'oval'):
                        corner_radius = min(half_width, half_height)
                    elif pad.shape == 'roundrect':
                        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
                    else:
                        corner_radius = 0
                    for cell_gx, cell_gy in iter_pad_blocked_cells(gx, gy, half_width, half_height, margin, config.grid_step, corner_radius,
                                                                   off_x=pad.global_x - gx * coord.grid_step,
                                                                   off_y=pad.global_y - gy * coord.grid_step,
                                                                   rotation_deg=pad.rect_rotation):
                        obstacles.add_blocked_cell(cell_gx, cell_gy, layer_idx)
                    pad_count += 1
    if verbose:
        print(f"  Pads: {pad_count} pads in {time.time() - t0:.2f}s")

    # Add segments on this layer as obstacles (excluding target net)
    # Use actual segment width for proper clearance calculation
    t0 = time.time()
    seg_count = 0
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        if seg.layer != route_layer:
            continue
        # Clearance needed: our track half-width + existing segment half-width + clearance.
        # Round the expansion UP (ceil): flooring leaves the blocked halo short of the
        # real clearance envelope, so connection traces route within clearance of signal
        # copper (#146 — the dominant plane-vs-signal DRC source on dense boards).
        seg_expansion_mm = config.track_width / 2 + seg.width / 2 + config.clearance
        seg_expansion_grid = max(1, coord.to_grid_dist_safe(seg_expansion_mm))
        _add_segment_routing_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_grid)
        seg_count += 1
    if verbose:
        print(f"  Segments: {seg_count} tracks in {time.time() - t0:.2f}s")

    # Add vias as obstacles (they block all layers)
    t0 = time.time()
    via_count = 0
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        via_expansion = coord.to_grid_dist_safe(via.size / 2 + config.track_width / 2 + config.clearance)
        for ex in range(-via_expansion, via_expansion + 1):
            for ey in range(-via_expansion, via_expansion + 1):
                if ex*ex + ey*ey <= via_expansion * via_expansion:
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
        via_count += 1
    if verbose:
        print(f"  Vias: {via_count} vias in {time.time() - t0:.2f}s")

    # Add board edge track blocking (supports non-rectangular boards)
    t0 = time.time()
    _add_board_edge_track_obstacles(obstacles, pcb_data, config, layer_idx)
    if verbose:
        print(f"  Board edge: {time.time() - t0:.2f}s")

    # Keep plane-routing tracks out of keepouts. This map has a single layer
    # (index 0 == route_layer), so scope the rule-area pass to route_layer.
    add_user_keepout_obstacles(obstacles, pcb_data, config, coord, num_layers)
    add_rule_area_keepout_obstacles(obstacles, pcb_data, config, layers=[route_layer])

    if verbose:
        print(f"  Total routing obstacle build: {time.time() - t_start:.2f}s")

    return obstacles


def _add_segment_routing_obstacle(obstacles: GridObstacleMap, seg: Segment,
                                    coord: GridCoord, layer_idx: int, expansion_grid: int):
    """Add a segment as a routing obstacle on a specific layer using batched numpy operations."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    radius_sq = expansion_grid * expansion_grid
    circle_offsets = _precompute_circle_offsets(radius_sq)
    centers = _bresenham_centers(gx1, gy1, gx2, gy2)
    _batch_block_circles_cell(obstacles, centers, circle_offsets, layer_idx)
