"""
Obstacle map building for copper plane via placement and routing.

Provides functions to build obstacle maps for:
- Via placement (considering all layers)
- Single-layer routing (for via-to-pad traces)
"""
from __future__ import annotations

import math
from typing import List, Dict, Tuple, Optional

import numpy as np

from kicad_parser import PCBData, Pad, Segment
from routing_config import GridRouteConfig, GridCoord
from routing_utils import iter_pad_blocked_cells, pad_blocked_cells_array, segment_blocked_cells_array
from obstacle_map import (point_in_polygon, point_to_polygon_edge_distance,
                          add_user_keepout_obstacles, add_rule_area_keepout_obstacles,
                          block_via_cells_near_drills, block_track_cells_near_drills,
                          block_track_cells_near_override_pad_holes,
                          _pad_has_copper,
                          _rasterize_polygon, _points_inside_polygon,
                          _points_edge_distance, _block_cells_on_layers,
                          _batch_cells_one_layer, _batch_vias)

import sys
import os
import routing_defaults as defaults
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


def _points_in_pad_copper_mask(pad: Pad, xs, ys, extra=0.0):
    """Vectorized twin of _point_in_pad_copper: boolean mask over the point
    arrays (xs, ys). `extra` may be a scalar or a per-point array (e.g. via
    radius). Rotation-aware via the pad's residual rect_rotation."""
    dx = xs - pad.global_x
    dy = ys - pad.global_y
    rot = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    cos_r = math.cos(rot)
    sin_r = math.sin(rot)
    lx = dx * cos_r + dy * sin_r
    ly = -dx * sin_r + dy * cos_r
    return (np.abs(lx) <= pad.size_x / 2 + extra) & (np.abs(ly) <= pad.size_y / 2 + extra)


class _NetConnGraph:
    __slots__ = ("seg_adj", "via_at", "pad_at", "via_arrays", "seg_ends_by_layer")


# Cache of _NetConnGraph keyed by (id(pcb_data), net_id). _smd_pad_reaches_layer
# was rebuilding this whole same-net graph on EVERY call (794x / 22s on daisho);
# it depends only on (net_id, copper), so build it once and reuse across every
# pad of the net. Invalidated when the board's segment/via count or tolerance
# changes (copper only grows during routing, so a count change is a real change).
_NET_GRAPH_CACHE: Dict[Tuple, Tuple] = {}


def _net_conn_graph(net_id: int, pcb_data: PCBData, inv_tol: float) -> "_NetConnGraph":
    key = (id(pcb_data), net_id)
    token = (len(pcb_data.segments), len(pcb_data.vias), inv_tol)
    hit = _NET_GRAPH_CACHE.get(key)
    if hit is not None and hit[0] == token:
        return hit[1]

    def pkey(x, y):
        return (round(x * inv_tol), round(y * inv_tol))

    if pcb_data.board_info and getattr(pcb_data.board_info, 'copper_layers', None):
        all_cu = [l for l in pcb_data.board_info.copper_layers if l.endswith('.Cu')]
    else:
        all_cu = ['F.Cu', 'B.Cu']

    # Segment adjacency + per-layer endpoint arrays (for vectorized pad seeding).
    seg_adj: Dict[Tuple, set] = {}
    seg_ends_tmp: Dict[str, Tuple[list, list, list]] = {}
    for s in pcb_data.segments:
        if s.net_id != net_id:
            continue
        k1 = (pkey(s.start_x, s.start_y), s.layer)
        k2 = (pkey(s.end_x, s.end_y), s.layer)
        seg_adj.setdefault(k1, set()).add(k2)
        seg_adj.setdefault(k2, set()).add(k1)
        xs, ys, ks = seg_ends_tmp.setdefault(s.layer, ([], [], []))
        xs.append(s.start_x); ys.append(s.start_y); ks.append(k1[0])
        xs.append(s.end_x);   ys.append(s.end_y);   ks.append(k2[0])

    def via_layers(v) -> List[str]:
        if not v.layers or ('F.Cu' in v.layers and 'B.Cu' in v.layers):
            return all_cu
        return v.layers

    via_at: Dict[Tuple, set] = {}
    vx, vy, vr, vkeys, vlayers = [], [], [], [], []
    for v in pcb_data.vias:
        if v.net_id != net_id:
            continue
        k = pkey(v.x, v.y)
        vl = via_layers(v)
        via_at.setdefault(k, set()).update(vl)
        vx.append(v.x); vy.append(v.y); vr.append(v.size / 2); vkeys.append(k); vlayers.append(vl)

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

    g = _NetConnGraph()
    g.seg_adj = seg_adj
    g.via_at = via_at
    g.pad_at = pad_at
    g.via_arrays = (np.asarray(vx, dtype=float), np.asarray(vy, dtype=float),
                    np.asarray(vr, dtype=float), vkeys, vlayers)
    g.seg_ends_by_layer = {
        L: (np.asarray(xs, dtype=float), np.asarray(ys, dtype=float), ks)
        for L, (xs, ys, ks) in seg_ends_tmp.items()
    }
    _NET_GRAPH_CACHE[key] = (token, g)
    return g


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

    # Same-net connectivity graph (built once per net + copper-state, cached).
    graph = _net_conn_graph(net_id, pcb_data, inv_tol)
    seg_adj = graph.seg_adj
    via_at = graph.via_at
    pad_at = graph.pad_at

    start = (pkey(pad.global_x, pad.global_y), pad_layer)
    visited = {start}
    queue = [start]

    # Seed from same-net copper that lands inside this pad's copper (vectorized):
    # - a via overlapping the pad connects the pad to all the via's layers
    # - a segment endpoint inside the pad (on the pad's layer) connects there
    vx, vy, vr, vkeys, vlayers = graph.via_arrays
    if len(vx):
        mask = _points_in_pad_copper_mask(pad, vx, vy, extra=vr)
        for i in np.nonzero(mask)[0]:
            for vl in vlayers[i]:
                state = (vkeys[i], vl)
                if state not in visited:
                    visited.add(state)
                    queue.append(state)
    seg_ends = graph.seg_ends_by_layer.get(pad_layer)
    if seg_ends is not None:
        ex, ey, ekeys = seg_ends
        mask = _points_in_pad_copper_mask(pad, ex, ey)
        for i in np.nonzero(mask)[0]:
            state = (ekeys[i], pad_layer)
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

    # A pad clearly outside the Edge.Cuts outline can never reach the plane
    # (the fill is clipped to the outline) and any via/trace drawn toward it
    # lands as board-edge DRC (issue #291, framework_dock GND). Classify it
    # off_board so no copper is drawn for it; callers report the count.
    from check_drc import make_off_board_test
    off_board = make_off_board_test(pcb_data.board_info)

    for pad in pads:
        if off_board is not None and off_board(pad.global_x, pad.global_y):
            target_pads.append({
                'pad': pad,
                'type': 'off_board',
                'needs_via': False,
                'needs_trace': False
            })
            continue
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

    # Half grid step cushion to account for grid discretization.
    grid_cushion = config.grid_step / 2

    # Add existing vias as obstacles (including same net - can't place a new via
    # too close to another). The via-to-via clearance is the EXISTING via's
    # radius + the NEW via's radius + clearance, so it must use each existing
    # via's ACTUAL size (issue #173, parity with route.py's add_vias_list_as_
    # obstacles). The old form used config.via_size for both radii, so a larger
    # existing via (e.g. a 0.5mm plane via vs a 0.3mm repair via) was under-
    # blocked and a new via could land within clearance of it. Group by actual
    # via size so each size gets its own keep-out disc.
    t0 = time.time()
    centers_by_size: Dict[float, List[Tuple[int, int]]] = {}
    for via in pcb_data.vias:
        centers_by_size.setdefault(via.size, []).append(coord.to_grid(via.x, via.y))
    for vsize, via_centers in centers_by_size.items():
        via_via_expansion_mm = vsize / 2 + config.via_size / 2 + config.clearance + grid_cushion
        circle_offsets = _precompute_circle_offsets((via_via_expansion_mm / config.grid_step) ** 2)
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
    """Add a segment as via-blocking obstacle: an exact point-to-segment
    (capsule) keep-out from the TRUE float segment, shared with route.py's
    obstacle builder (issue #173). The previous bresenham stamp snapped the
    endpoints to the grid and walked integer cells, so an off-grid/diagonal
    segment's via keep-out under-covered sub-cell and a later via grazed it."""
    vias = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                       seg.end_x, seg.end_y, expansion_mm, coord.grid_step)
    _batch_vias(obstacles, vias)


def _block_custom_pad_polys(obstacles, pad, coord, margin, via_mode, layer_idx=None):
    """Block a custom comb/finger pad by its real copper polygon(s) expanded by
    `margin`, leaving the finger channels open instead of filling the bounding box
    (issue #188). Used by the plane obstacle builder's via- and track-blocking."""
    import numpy as np
    from obstacle_map import _rasterize_polygon
    for poly in pad.polygons:
        gxf, gyf, inside, edist = _rasterize_polygon(poly, coord, margin)
        if gxf is None:
            continue
        mask = inside | (edist <= margin)
        for i in np.flatnonzero(mask):
            if via_mode:
                obstacles.add_blocked_via(int(gxf[i]), int(gyf[i]))
            else:
                obstacles.add_blocked_cell(int(gxf[i]), int(gyf[i]), layer_idx)


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
    # Honor a per-pad local clearance override (fiducial keep-clear rings etc.)
    # unless an explicit same-net override was supplied.
    if clearance_override is None:
        clearance = max(clearance, getattr(pad, 'local_clearance', 0.0) or 0.0)
    margin = config.via_size / 2 + clearance + config.grid_step / 2
    if getattr(pad, 'polygons', None):
        _block_custom_pad_polys(obstacles, pad, coord, margin, via_mode=True)
        return
    # Corner radius based on pad shape (circle/oval use min dimension, roundrect uses rratio)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(half_width, half_height)
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0

    # Vectorized: pad_blocked_cells_array is the bit-identical twin of
    # iter_pad_blocked_cells (verified in tests/test_pad_offset_keepout.py), and
    # one add_blocked_vias_batch replaces a per-cell Python loop + per-cell Rust
    # call. This is the route.py obstacle-builder path (obstacle_map.py), which
    # the plane via map had not yet picked up (#225 -- ~38M scalar cell calls per
    # GND build on daisho).
    cells = pad_blocked_cells_array(gx, gy, half_width, half_height, margin,
                                    config.grid_step, corner_radius,
                                    off_x=pad.global_x - gx * coord.grid_step,
                                    off_y=pad.global_y - gy * coord.grid_step,
                                    rotation_deg=pad.rect_rotation)
    if len(cells):
        obstacles.add_blocked_vias_batch(cells)


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


def _edge_band_grid(gmin_x: int, gmin_y: int, gmax_x: int, gmax_y: int, grid_margin: int):
    """Flattened (gx, gy) int32 arrays over the bounding box expanded by grid_margin."""
    gx_range = np.arange(gmin_x - grid_margin, gmax_x + grid_margin + 1, dtype=np.int32)
    gy_range = np.arange(gmin_y - grid_margin, gmax_y + grid_margin + 1, dtype=np.int32)
    gx_grid, gy_grid = np.meshgrid(gx_range, gy_range)
    return gx_grid.ravel(), gy_grid.ravel()


def _board_edge_cell_mask(coord: GridCoord, board_outline, gmin_x: int, gmin_y: int,
                          gmax_x: int, gmax_y: int, grid_margin: int, edge_clearance: float):
    """Cells to block for a polygon board outline: those whose centre is outside the
    board, or inside but within `edge_clearance` mm of an edge. Vectorized even-odd
    ray cast + edge-distance (the same kernels the signal router uses). Returns
    (gx_flat, gy_flat, mask)."""
    gx_flat, gy_flat = _edge_band_grid(gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
    px = gx_flat.astype(np.float64) * coord.grid_step
    py = gy_flat.astype(np.float64) * coord.grid_step

    # One outer ring or a LIST of them (#304): inside ANY ring is on-board,
    # edge distance is the minimum over all rings' edges.
    rings = [board_outline] if board_outline and isinstance(board_outline[0], tuple) \
        else list(board_outline)
    inside = None
    ex1, ey1, ex2, ey2 = [], [], [], []
    for ring in rings:
        poly = np.array(ring, dtype=np.float64)
        x1 = poly[:, 0]
        y1 = poly[:, 1]
        x2 = np.roll(poly[:, 0], -1)
        y2 = np.roll(poly[:, 1], -1)
        ex1.append(x1); ey1.append(y1); ex2.append(x2); ey2.append(y2)
        ins = _points_inside_polygon(px, py, x1, y1, x2, y2)
        inside = ins if inside is None else (inside | ins)
    x1, y1 = np.concatenate(ex1), np.concatenate(ey1)
    x2, y2 = np.concatenate(ex2), np.concatenate(ey2)
    mask = ~inside
    in_idx = np.nonzero(inside)[0]
    if in_idx.size:
        edge_dist = _points_edge_distance(px[in_idx], py[in_idx], x1, y1, x2, y2)
        mask[in_idx[edge_dist < edge_clearance]] = True
    return gx_flat, gy_flat, mask


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
    via_expand = coord.to_grid_dist_safe(via_edge_clearance)

    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    grid_margin = via_expand + 5

    # Check for non-rectangular board outline; multi-outline boards (#304)
    # pass ALL outer rings so both halves of a split board stay usable.
    board_outlines = [o for o in (getattr(pcb_data.board_info, 'board_outlines', None) or [])
                      if len(o) >= 3]
    board_outline = (board_outlines if len(board_outlines) > 1
                     else pcb_data.board_info.board_outline)
    single = board_outlines[0] if len(board_outlines) == 1 else pcb_data.board_info.board_outline
    use_polygon = bool(len(board_outlines) > 1 or
                       (single and len(single) >= 3
                        and not _is_rectangular_outline(single, board_bounds)))

    if use_polygon:
        # Polygon board: rasterize the outline bbox + margin in one shot. Every
        # cell outside the board (ray-cast) is blocked, plus inside cells within the
        # via edge clearance. Same vectorized kernels as the signal router's
        # obstacle_map._add_polygon_edge_obstacles - the per-cell Python scan and
        # its band optimization are no longer needed (issue #81).
        gx_flat, gy_flat, via_mask = _board_edge_cell_mask(
            coord, board_outline, gmin_x, gmin_y, gmax_x, gmax_y, grid_margin, via_edge_clearance)
        if via_mask.any():
            obstacles.add_blocked_vias_batch(np.column_stack([gx_flat[via_mask], gy_flat[via_mask]]))
    else:
        # Rectangular board - simple bounding box band, vectorized.
        gx_flat, gy_flat = _edge_band_grid(gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
        mask = ((gx_flat < gmin_x + via_expand) | (gx_flat > gmax_x - via_expand) |
                (gy_flat < gmin_y + via_expand) | (gy_flat > gmax_y - via_expand))
        if mask.any():
            obstacles.add_blocked_vias_batch(np.column_stack([gx_flat[mask], gy_flat[mask]]))

    # Block vias inside board cutouts
    for cutout in pcb_data.board_info.board_cutouts:
        if len(cutout) < 3:
            continue
        cgx, cgy, c_inside, c_edge = _rasterize_polygon(cutout, coord, via_edge_clearance)
        if cgx is None:
            continue
        cmask = c_inside | (c_edge < via_edge_clearance)
        if cmask.any():
            obstacles.add_blocked_vias_batch(np.column_stack([cgx[cmask], cgy[cmask]]))


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
    track_expand = coord.to_grid_dist_safe(track_edge_clearance)

    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    grid_margin = track_expand + 5

    # Check for non-rectangular board outline; multi-outline boards (#304)
    # pass ALL outer rings so both halves of a split board stay usable.
    board_outlines = [o for o in (getattr(pcb_data.board_info, 'board_outlines', None) or [])
                      if len(o) >= 3]
    board_outline = (board_outlines if len(board_outlines) > 1
                     else pcb_data.board_info.board_outline)
    single = board_outlines[0] if len(board_outlines) == 1 else pcb_data.board_info.board_outline
    use_polygon = bool(len(board_outlines) > 1 or
                       (single and len(single) >= 3
                        and not _is_rectangular_outline(single, board_bounds)))

    if use_polygon:
        # Polygon board: rasterize the outline bbox + margin once and block (on this
        # one layer) every cell outside the board plus inside cells within the track
        # edge clearance. Mirrors obstacle_map._add_polygon_edge_obstacles (issue #81).
        gx_flat, gy_flat, cell_mask = _board_edge_cell_mask(
            coord, board_outline, gmin_x, gmin_y, gmax_x, gmax_y, grid_margin, track_edge_clearance)
        _block_cells_on_layers(obstacles, gx_flat, gy_flat, cell_mask, [layer_idx])
    else:
        # Rectangular board - simple bounding box band, vectorized.
        gx_flat, gy_flat = _edge_band_grid(gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
        mask = ((gx_flat < gmin_x + track_expand) | (gx_flat > gmax_x - track_expand) |
                (gy_flat < gmin_y + track_expand) | (gy_flat > gmax_y - track_expand))
        _block_cells_on_layers(obstacles, gx_flat, gy_flat, mask, [layer_idx])

    # Block tracks inside board cutouts
    for cutout in pcb_data.board_info.board_cutouts:
        if len(cutout) < 3:
            continue
        cgx, cgy, c_inside, c_edge = _rasterize_polygon(cutout, coord, track_edge_clearance)
        if cgx is None:
            continue
        cmask = c_inside | (c_edge < track_edge_clearance)
        _block_cells_on_layers(obstacles, cgx, cgy, cmask, [layer_idx])


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
    from kicad_parser import pad_drill_circles
    for net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            if pad.drill > 0:
                # slot-aware (see pad_drill_circles): a milled slot blocks along
                # its axis at its short dimension, not as a long-dimension disc
                drill_holes.extend(pad_drill_circles(pad))

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
    # Use THIS layer's routing width for keep-out math (issue #173 parity with
    # route.py's per-layer via/track expansion); == config.track_width unless
    # per-layer widths are set (impedance routing).
    route_track_w = config.get_track_width(route_layer)

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
                    # Honor a per-pad local clearance override (e.g. fiducial
                    # keep-clear rings carry a clearance far larger than the
                    # board global), else copper routes within the pad's
                    # required clearance (no-net fiducial DRC, upduino #146).
                    pad_clr = max(config.clearance, getattr(pad, 'local_clearance', 0.0) or 0.0)
                    margin = route_track_w / 2 + pad_clr
                    if getattr(pad, 'polygons', None):
                        _block_custom_pad_polys(obstacles, pad, coord, margin,
                                                via_mode=False, layer_idx=layer_idx)
                        pad_count += 1
                        continue
                    # Corner radius based on pad shape
                    if pad.shape in ('circle', 'oval'):
                        corner_radius = min(half_width, half_height)
                    elif pad.shape == 'roundrect':
                        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
                    else:
                        corner_radius = 0
                    # Vectorized (bit-identical twin + batch), mirroring route.py's
                    # obstacle builder; the plane routing map had kept the scalar
                    # per-cell loop (#225).
                    cells = pad_blocked_cells_array(gx, gy, half_width, half_height, margin,
                                                    config.grid_step, corner_radius,
                                                    off_x=pad.global_x - gx * coord.grid_step,
                                                    off_y=pad.global_y - gy * coord.grid_step,
                                                    rotation_deg=pad.rect_rotation)
                    if len(cells):
                        layer_col = np.full((cells.shape[0], 1), layer_idx, dtype=np.int32)
                        obstacles.add_blocked_cells_batch(np.hstack([cells, layer_col]))
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
        # The exact capsule keep-out measures from the real segment, so the blocked
        # halo matches the true clearance envelope without the grid-rounding that used
        # to leave connection traces within clearance of signal copper (#146/#173).
        seg_expansion_mm = route_track_w / 2 + seg.width / 2 + config.clearance
        _add_segment_routing_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_mm)
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
        # Block cells by REAL distance to the via centre (not the grid-quantised
        # cell) plus a half-cell buffer for the routed track's own discretisation,
        # so a sub-cell via offset can't let a 0.3 mm plane trace sit inside the
        # clearance envelope (#70). The grid-circle-on-quantised-cell form lost up
        # to ~half a cell on the via side.
        r_mm = via.size / 2 + route_track_w / 2 + config.clearance + config.grid_step / 2
        rg = coord.to_grid_dist_safe(r_mm)
        r_sq = r_mm * r_mm
        # Vectorized real-centre disc (bit-identical to the scalar double loop:
        # same (gx+ex)*step - via.x distance and <= r_sq test), one batch call (#225).
        ex = np.arange(-rg, rg + 1, dtype=np.int32)
        exg, eyg = np.meshgrid(ex, ex, indexing="ij")
        ddx = (gx + exg) * config.grid_step - via.x
        ddy = (gy + eyg) * config.grid_step - via.y
        mask = (ddx * ddx + ddy * ddy) <= r_sq
        if mask.any():
            vcells = np.empty((int(mask.sum()), 3), dtype=np.int32)
            vcells[:, 0] = exg[mask] + gx
            vcells[:, 1] = eyg[mask] + gy
            vcells[:, 2] = layer_idx
            obstacles.add_blocked_cells_batch(vcells)
        via_count += 1
    if verbose:
        print(f"  Vias: {via_count} vias in {time.time() - t0:.2f}s")

    # Keep plane-routing tracks off NPTH (no-copper) drill holes (issue #233).
    # An NPTH mounting pad carries drill>0 but only a *.Mask layer, so the pad loop
    # above stamps no cell for it; without this a plane tap / repair track routes
    # straight across the hole. PTH pads/vias have copper, already blocked above.
    # Single-layer map, so block layer 0 only (the drill goes through every layer).
    t0 = time.time()
    npth_holes = []
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            if pad.drill > 0 and not _pad_has_copper(pad):
                from kicad_parser import pad_drill_circles
                npth_holes.extend(pad_drill_circles(pad))
    npth_clr = max(config.clearance, defaults.NPTH_TO_TRACK_CLEARANCE)
    block_track_cells_near_drills(obstacles, npth_holes, route_track_w,
                                  npth_clr, config.grid_step, [layer_idx])
    # Holes of pads carrying a clearance OVERRIDE (pad.local_clearance): KiCad's
    # hole_clearance rule is net-independent and honors the override, so even
    # own-net (exclude_net_id) copper must keep the override off the hole unless
    # it lands on the pad copper itself (#326 residual, ghoul: zero-ring switch
    # NPTHs at 0.3 were stamped only at the 0.20 NPTH floor above).
    block_track_cells_near_override_pad_holes(
        obstacles, pcb_data, route_track_w, config.clearance,
        config.grid_step, [layer_idx])
    if verbose:
        print(f"  NPTH-hole track keep-out: {len(npth_holes)} holes in {time.time() - t0:.2f}s")

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
                                    coord: GridCoord, layer_idx: int, expansion_mm: float):
    """Add a segment as a routing obstacle on a specific layer: an exact
    point-to-segment (capsule) keep-out from the TRUE float segment, shared with
    route.py's obstacle builder (issue #173). Replaces the bresenham stamp that
    snapped endpoints to the grid and under-covered off-grid/diagonal copper."""
    cells = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                        seg.end_x, seg.end_y, expansion_mm, coord.grid_step)
    _batch_cells_one_layer(obstacles, cells, layer_idx)
