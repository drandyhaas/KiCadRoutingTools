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
                          _rasterize_polygon, _scanline_inside_rows,
                          _banded_edge_distance_rows, _block_cells_on_layers,
                          _batch_cells_one_layer, _batch_vias)

import sys
import os
import routing_defaults as defaults
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads
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


# Cache of _NetConnGraph per net. _smd_pad_reaches_layer was rebuilding this
# whole same-net graph on EVERY call (794x / 22s on daisho); it depends only on
# (net_id, copper), so build it once and reuse across every pad of the net.
#
# #803: this had BOTH cache-invalidation bugs at once.
#
#   1. The token was COUNT-ONLY, justified by "copper only grows during routing,
#      so a count change is a real change". That is no longer true -- the
#      in-loop stub-debris trim REMOVES committed copper, and rip_up_net /
#      restore_net cycles do too. Remove k items and add k others and the token
#      is unchanged while the copper is completely different, so this returned a
#      graph describing copper that no longer exists. The same false assumption
#      in plane_pad_tap._tap_spatial_index put a rescue via through a foreign
#      track on glasgow_revC.
#   2. It was a MODULE-level dict keyed on id(pcb_data). CPython reuses ids
#      after GC, and make_local_window mints a short-lived PCBData per tap/
#      rescue, so a freed window's id is very likely to be handed to the next
#      one -- which then inherits its predecessor's graph. net_cost._cache_for
#      documents this exact hazard and the fix: hang the cache off the board,
#      so its lifetime is tied to the data it describes.
#
# Both are fixed below: the cache lives on the board, and the token is
# content-sensitive (summing element ids costs O(n), the same order as the
# rebuild it guards, and only replaces a hit that was silently wrong).


def _net_conn_graph(net_id: int, pcb_data: PCBData, inv_tol: float) -> "_NetConnGraph":
    segs, vias = pcb_data.segments, pcb_data.vias
    token = (len(segs), len(vias), inv_tol,
             id(segs), id(vias), sum(map(id, segs)), sum(map(id, vias)))
    store = getattr(pcb_data, '_net_graph_cache', None)
    if store is None:
        store = {}
        try:
            pcb_data._net_graph_cache = store
        except AttributeError:
            store = None          # slotted/exotic board: degrade to uncached
    hit = store.get(net_id) if store is not None else None
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
    if store is not None:
        store[net_id] = (token, g)
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
            When the caller leaves it negative, the config's own
            same_net_pad_clearance (#581: set from the route_planes flag or the
            persisted .kicad_pro record) applies — so every via map built from a
            config that carries the constraint honors it, including callers that
            never learned the parameter (repair_planes taps, the #562 finalize).
    """
    import time
    t_start = time.time()
    if same_net_pad_clearance is None or same_net_pad_clearance < 0:
        # #581 compat contract: only an ACTIVE (> 0) config value changes
        # behavior; an explicit caller-passed 0 above keeps its legacy meaning.
        _cfg_snpc = getattr(config, 'same_net_pad_clearance', -1.0)
        same_net_pad_clearance = _cfg_snpc if (_cfg_snpc is not None
                                               and _cfg_snpc > 0) else -1.0

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
    # #434: foreign vias are priced at config.obstacle_clearance(net_id) --
    # max(run clearance, their netclass clearance) -- so tap vias honor KiCad's
    # pairwise max(classA, classB); same-net vias keep the run clearance.
    # Group by (size, clearance); all-Default boards collapse to the old groups.
    centers_by_size: Dict[Tuple[float, float], List[Tuple[int, int]]] = {}
    for via in pcb_data.vias:
        _clr = (config.clearance if via.net_id == exclude_net_id
                else config.obstacle_clearance(via.net_id))
        _clr = config.stack_clearance(_clr)  # #498: barrels meet on every layer
        centers_by_size.setdefault((via.size, _clr), []).append(coord.to_grid(via.x, via.y))
    for (vsize, _clr), via_centers in centers_by_size.items():
        via_via_expansion_mm = vsize / 2 + config.via_size / 2 + _clr + grid_cushion
        circle_offsets = _precompute_circle_offsets((via_via_expansion_mm / config.grid_step) ** 2)
        _batch_block_circles_via(obstacles, via_centers, circle_offsets)
    if verbose:
        print(f"  Vias: {len(pcb_data.vias)} vias in {time.time() - t0:.2f}s")

    # Add existing segments as obstacles (via can't overlap with tracks on ANY layer)
    # Since vias span all layers, we must check segments on all copper layers, not just config.layers
    t0 = time.time()
    seg_count = 0
    _seg_via_arrs = []
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        # Include any copper layer (*.Cu)
        if not seg.layer.endswith('.Cu'):
            continue
        # Use actual segment width for clearance calculation (not config.track_width)
        # Include grid cushion for discretization; price the segment's net at its
        # netclass clearance (#434 cross-class).
        seg_expansion_mm = (config.via_size / 2 + seg.width / 2
                            + config.layer_clearance(  # #498: meet on seg.layer
                                seg.layer, config.obstacle_clearance(seg.net_id))
                            + grid_cushion)
        # FFI batching (2026-08-14): accumulate the memoized capsule arrays
        # and stamp once after the loop -- concatenation preserves the exact
        # row multiset/order and the batch inserts commute, so the map state
        # is byte-identical to the per-segment calls this replaces.
        _va = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                          seg.end_x, seg.end_y,
                                          seg_expansion_mm, coord.grid_step)
        if len(_va):
            _seg_via_arrs.append(_va)
        seg_count += 1
    if _seg_via_arrs:
        _vall = (np.concatenate(_seg_via_arrs) if len(_seg_via_arrs) > 1
                 else _seg_via_arrs[0])
        obstacles.add_blocked_vias_batch(
            np.ascontiguousarray(_vall.astype(np.int32)))
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
    # Add half grid step buffer to account for grid quantization errors.
    # Foreign pads are priced at their net's netclass clearance (#434).
    clearance = (config.obstacle_clearance(pad.net_id)
                 if clearance_override is None else clearance_override)
    # #498: a via barrel meets the pad's copper on each layer the pad carries
    # it -> max over the per-layer resolution ('*.Cu' = the whole stack).
    # FOREIGN pads only: an explicit override (same-net pads) is not a KiCad
    # clearance pair and keeps its value.
    if clearance_override is None and getattr(config, 'layer_clearances', None):
        _pls = [l for l in (pad.layers or []) if l.endswith('.Cu') and l != '*.Cu']
        if '*.Cu' in (pad.layers or []) or not _pls:
            clearance = config.stack_clearance(clearance)
        else:
            clearance = max(config.layer_clearance(l, clearance) for l in _pls)
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


# #546 memory: entry caps for the two mask caches below. Both memoize pure
# functions of static geometry, so eviction is always safe -- an evicted key
# recomputes in milliseconds with the scanline/banded kernels. Before the
# caps, a full-board _edge_mask_cache entry stored ~108 MB (two 48 MB int32
# meshgrid flats that are pure functions of the window bounds, plus the 12 MB
# bool mask), and the clip-window-keyed cutout cache grew one entry set per
# distinct tap window for the whole run (crkbd: 172+ entries and climbing).
_EDGE_MASK_CACHE_MAX = 8       # masks (window grids shared separately)
_EDGE_GRID_CACHE_MAX = 2       # (gx_flat, gy_flat) meshgrids per window
_CUTOUT_CACHE_MAX = 512        # small per-cutout window entries
_CUTOUT_CACHE_MAX_BYTES = 128 * 1024 * 1024


def _lru_get(cache, key):
    """dict-as-LRU: move the hit to the ordered dict's end (most recent)."""
    hit = cache.get(key)
    if hit is not None and next(reversed(cache)) != key:
        del cache[key]
        cache[key] = hit
    return hit


def _lru_put(cache, key, value, cap):
    cache[key] = value
    while len(cache) > cap:
        del cache[next(iter(cache))]


def _edge_band_grid_cached(pcb_data, gmin_x, gmin_y, gmax_x, gmax_y,
                           grid_margin):
    """The flattened window meshgrid, shared across every edge-mask entry
    with the same window (it does not depend on outline or clearance)."""
    cache = getattr(pcb_data, '_edge_grid_cache', None)
    if cache is None:
        cache = {}
        pcb_data._edge_grid_cache = cache
    key = (gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
    hit = _lru_get(cache, key)
    if hit is None:
        hit = _edge_band_grid(gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
        _lru_put(cache, key, hit, _EDGE_GRID_CACHE_MAX)
    return hit


def _board_edge_cell_mask_cached(pcb_data, coord: GridCoord, board_outline,
                                 gmin_x: int, gmin_y: int, gmax_x: int,
                                 gmax_y: int, grid_margin: int,
                                 edge_clearance: float):
    """Memoized _board_edge_cell_mask, cached on pcb_data. The mask is a pure
    function of the (static) board outline, the grid, and the clearance band,
    yet every via/routing obstacle-map build re-rasterized it from scratch --
    171 identical ray-cast + edge-distance passes on scalenode_cm4's polygon
    outline (~1/3 of the whole route_planes step). Consumers only READ the
    returned arrays (boolean indexing / column_stack), so sharing is safe.

    #546 memory: the cache stores the MASK only (LRU-capped); the coordinate
    flats are a pure function of the window and come from the shared
    window-grid cache, so N clearance variants of one window no longer store
    N copies of the same 96 MB meshgrid."""
    cache = getattr(pcb_data, '_edge_mask_cache', None)
    if cache is None:
        cache = {}
        pcb_data._edge_mask_cache = cache
    # Outline identity in the key (same #490 lesson as _cutout_fingerprint):
    # the cache dict rides along on make_local_window's shallow copies, and
    # keys must never assume the caller's outline is the whole board's.
    if not board_outline:
        _ol_fp = None
    elif isinstance(board_outline[0][0], (int, float)):
        _ol_fp = _cutout_fingerprint(board_outline)  # single ring
    else:
        _ol_fp = tuple(_cutout_fingerprint(o)        # #304 multi-outline
                       for o in board_outline)
    key = (_ol_fp, round(coord.grid_step, 9), gmin_x, gmin_y, gmax_x, gmax_y,
           grid_margin, round(edge_clearance, 9))
    gx_flat, gy_flat = _edge_band_grid_cached(
        pcb_data, gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
    mask = _lru_get(cache, key)
    if mask is None:
        _gx, _gy, mask = _board_edge_cell_mask(
            coord, board_outline, gmin_x, gmin_y, gmax_x, gmax_y,
            grid_margin, edge_clearance)
        _lru_put(cache, key, mask, _EDGE_MASK_CACHE_MAX)
    return gx_flat, gy_flat, mask


def _cutout_fingerprint(cutout):
    """Geometry identity for the cutout-mask cache key. NEVER key by list
    index: make_local_window FILTERS board_cutouts to the window, reindexing
    from 0, and the cache dict rides along on the shallow copy -- so window A
    cached cutout #56's mask under index 0 and window B (near cutout #12) hit
    that key and stamped the WRONG cutout's band, leaving its real cutout
    unguarded (crkbd: 105 tap grazes on the 72 key-switch cutouts, #490)."""
    n = len(cutout)
    return (n, cutout[0], cutout[n // 2], cutout[-1])


def _rasterize_cutout_cached(pcb_data, cutout_idx: int, cutout, coord: GridCoord,
                             clearance: float, band_only: bool = False,
                             clip_bounds=None):
    """Memoized cutout rasterization (same rationale as the edge-mask cache):
    board cutouts are static, but each obstacle build re-rasterized every one
    per layer. Returns the (K, 2) int32 array of blocked cells (interior +
    clearance band, or band only) -- or None for a degenerate/clipped-away
    polygon. Keyed by geometry fingerprint, not list index (see
    _cutout_fingerprint).

    ``band_only`` (#505) drops the interior term, leaving just the clearance
    band around the boundary -- for a milled INNER contour, which is a mill line
    rather than an opening: its inside is still board (it encloses pads by
    definition), so blocking the interior would blank the plane.

    ``clip_bounds`` restricts rasterization to the map's real extent. A milled
    inner contour is typically BOARD-SIZED (crkbd's is the whole outline), and
    without clipping every local-window build would rasterize the entire board
    -- the exact cost _rasterize_polygon's own docstring warns about. Clipping
    is a pure optimisation: no cell inside the map changes."""
    cache = getattr(pcb_data, '_cutout_mask_cache', None)
    if cache is None:
        cache = {}
        pcb_data._cutout_mask_cache = cache
    key = (_cutout_fingerprint(cutout), round(coord.grid_step, 9),
           round(clearance, 9), band_only,
           tuple(round(b, 6) for b in clip_bounds) if clip_bounds else None)
    hit = _lru_get(cache, key)
    if hit is None:
        cgx, cgy, c_inside, c_edge = _rasterize_polygon(
            cutout, coord, clearance, clip_bounds=clip_bounds)
        if cgx is None:
            hit = (None,)
        else:
            cmask = (c_edge < clearance) if band_only \
                else (c_inside | (c_edge < clearance))
            # #546 memory: store the MASKED CELLS only -- every consumer
            # reduces the (cgx, cgy, cmask) triple to exactly this array,
            # and the triple for a board-clipped inner contour was ~7 MB
            # against ~1-2 MB of actual band cells (the recheck phase's
            # full-radius windows ballooned the old cache to gigabytes).
            hit = (np.column_stack([cgx[cmask], cgy[cmask]]),)
        _lru_put(cache, key, hit, _CUTOUT_CACHE_MAX)
        # Byte budget on top of the entry cap: entries are usually small,
        # but a pathological board could still stack big ones.
        _bytes = sum(h[0].nbytes for h in cache.values()
                     if h[0] is not None)
        while _bytes > _CUTOUT_CACHE_MAX_BYTES and len(cache) > 1:
            _old = next(iter(cache))
            _oh = cache.pop(_old)
            if _oh[0] is not None:
                _bytes -= _oh[0].nbytes
    return hit[0]


def _board_edge_cell_mask(coord: GridCoord, board_outline, gmin_x: int, gmin_y: int,
                          gmax_x: int, gmax_y: int, grid_margin: int, edge_clearance: float):
    """Cells to block for a polygon board outline: those whose centre is outside the
    board, or inside but within `edge_clearance` mm of an edge. Vectorized even-odd
    ray cast + edge-distance (the same kernels the signal router uses). Returns
    (gx_flat, gy_flat, mask)."""
    gx_flat, gy_flat = _edge_band_grid(gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
    # #546: row-scanline inside test + threshold-banded edge distance (see
    # obstacle_map._scanline_inside_rows) -- the dense (cells x edges) kernels
    # made this the second-hottest call of the crkbd plane repair.
    gx_range = np.arange(gmin_x - grid_margin, gmax_x + grid_margin + 1,
                         dtype=np.int32)
    gy_range = np.arange(gmin_y - grid_margin, gmax_y + grid_margin + 1,
                         dtype=np.int32)
    px_axis = gx_range.astype(np.float64) * coord.grid_step
    py_axis = gy_range.astype(np.float64) * coord.grid_step

    # One outer ring or a LIST of them (#304): inside ANY ring is on-board,
    # edge distance is the minimum over all rings' edges.
    rings = [board_outline] if board_outline and isinstance(board_outline[0], tuple) \
        else list(board_outline)
    inside2d = None
    ex1, ey1, ex2, ey2 = [], [], [], []
    for ring in rings:
        poly = np.array(ring, dtype=np.float64)
        x1 = poly[:, 0]
        y1 = poly[:, 1]
        x2 = np.roll(poly[:, 0], -1)
        y2 = np.roll(poly[:, 1], -1)
        ex1.append(x1); ey1.append(y1); ex2.append(x2); ey2.append(y2)
        ins = _scanline_inside_rows(px_axis, py_axis, x1, y1, x2, y2)
        inside2d = ins if inside2d is None else (inside2d | ins)
    x1, y1 = np.concatenate(ex1), np.concatenate(ey1)
    x2, y2 = np.concatenate(ex2), np.concatenate(ey2)
    inside = inside2d.ravel()
    mask = ~inside
    in_idx = np.nonzero(inside)[0]
    if in_idx.size:
        edge_dist = _banded_edge_distance_rows(
            px_axis, py_axis, x1, y1, x2, y2,
            edge_clearance + coord.grid_step).ravel()[in_idx]
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
        gx_flat, gy_flat, via_mask = _board_edge_cell_mask_cached(
            pcb_data, coord, board_outline, gmin_x, gmin_y, gmax_x, gmax_y,
            grid_margin, via_edge_clearance)
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
    for _ci, cutout in enumerate(pcb_data.board_info.board_cutouts):
        if len(cutout) < 3:
            continue
        _cells = _rasterize_cutout_cached(
            pcb_data, _ci, cutout, coord, via_edge_clearance)
        if _cells is not None and len(_cells):
            obstacles.add_blocked_vias_batch(_cells)

    # Milled inner contours (#505): band only -- a mill line, not an opening.
    for _ei, _ec in enumerate(getattr(pcb_data.board_info,
                                      'board_edge_contours', None) or []):
        if len(_ec) < 3:
            continue
        _cells = _rasterize_cutout_cached(
            pcb_data, _ei, _ec, coord, via_edge_clearance, band_only=True,
            clip_bounds=pcb_data.board_info.board_bounds)
        if _cells is not None and len(_cells):
            obstacles.add_blocked_vias_batch(_cells)


def _add_board_edge_track_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                     config: GridRouteConfig, layer_idx: int,
                                     track_width: Optional[float] = None):
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
    # #447: use the width the map is STAMPED at, not always config.track_width.
    # build_routing_obstacle_map stamps at the per-layer route_track_w (wider than
    # config.track_width on impedance boards); a config.track_width edge band would
    # let those wider tracks graze the outline sub-fab. Callers pass the stamp width.
    _edge_tw = track_width if track_width is not None else config.track_width
    track_edge_clearance = edge_clearance + _edge_tw / 2
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
        gx_flat, gy_flat, cell_mask = _board_edge_cell_mask_cached(
            pcb_data, coord, board_outline, gmin_x, gmin_y, gmax_x, gmax_y,
            grid_margin, track_edge_clearance)
        _block_cells_on_layers(obstacles, gx_flat, gy_flat, cell_mask, [layer_idx])
    else:
        # Rectangular board - simple bounding box band, vectorized.
        gx_flat, gy_flat = _edge_band_grid(gmin_x, gmin_y, gmax_x, gmax_y, grid_margin)
        mask = ((gx_flat < gmin_x + track_expand) | (gx_flat > gmax_x - track_expand) |
                (gy_flat < gmin_y + track_expand) | (gy_flat > gmax_y - track_expand))
        _block_cells_on_layers(obstacles, gx_flat, gy_flat, mask, [layer_idx])

    # Block tracks inside board cutouts
    for _ci, cutout in enumerate(pcb_data.board_info.board_cutouts):
        if len(cutout) < 3:
            continue
        _cells = _rasterize_cutout_cached(
            pcb_data, _ci, cutout, coord, track_edge_clearance)
        if _cells is not None and len(_cells):
            _lcol = np.full((_cells.shape[0], 1), layer_idx, dtype=np.int32)
            obstacles.add_blocked_cells_batch(np.hstack([_cells, _lcol]))

    # Milled inner contours (#505): band only -- a mill line, not an opening.
    for _ei, _ec in enumerate(getattr(pcb_data.board_info,
                                      'board_edge_contours', None) or []):
        if len(_ec) < 3:
            continue
        _cells = _rasterize_cutout_cached(
            pcb_data, _ei, _ec, coord, track_edge_clearance, band_only=True,
            clip_bounds=pcb_data.board_info.board_bounds)
        if _cells is not None and len(_cells):
            _lcol = np.full((_cells.shape[0], 1), layer_idx, dtype=np.int32)
            obstacles.add_blocked_cells_batch(np.hstack([_cells, _lcol]))


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
    # FFI batching, same pattern the SEGMENT loop below already uses (and the
    # via loop above it): accumulate the per-pad cell arrays and stamp once,
    # instead of one Rust call per pad. The 2026-08-14 batching pass fixed the
    # segment and via loops here and missed this one -- measured on glasgow,
    # add_blocked_cells_batch was entered 1,483,841 times per route, ~299 per
    # build_routing_obstacle_map call, i.e. once per pad. Bit-identical:
    # concatenation preserves the exact row multiset, and the batch insert
    # refcounts a given cell the same number of times whether the rows arrive
    # split or joined.
    _pad_cell_arrs = []
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
                    pad_clr = max(config.layer_clearance(  # #498: on route_layer
                                      route_layer, config.obstacle_clearance(net_id)),
                                  getattr(pad, 'local_clearance', 0.0) or 0.0)
                    # Half-grid discretization cushion, matching this file's own
                    # VIA stamps and build_base_obstacles (#173) -- the segment
                    # capsule below deliberately carries NO cushion, same as the
                    # main builder's (see PR #532: adding one there costs grid/2
                    # of tap routability for a um-scale chord-dip class): cells
                    # are blocked by CENTER distance, so without the cushion a
                    # tap trace through a barely-free cell can sit up to
                    # ~grid/2 inside the pad's clearance ring (orangecrab
                    # step11: GND jog 1.5um under the U9 custom pad's ring --
                    # a real KiCad clearance violation).
                    margin = route_track_w / 2 + pad_clr + config.grid_step / 2
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
                        _pad_cell_arrs.append(cells)
                    pad_count += 1
    if _pad_cell_arrs:
        _pall = (np.concatenate(_pad_cell_arrs) if len(_pad_cell_arrs) > 1
                 else _pad_cell_arrs[0])
        _rows = np.empty((len(_pall), 3), dtype=np.int32)
        _rows[:, :2] = _pall
        _rows[:, 2] = layer_idx
        obstacles.add_blocked_cells_batch(np.ascontiguousarray(_rows))
    if verbose:
        print(f"  Pads: {pad_count} pads in {time.time() - t0:.2f}s")

    # Add segments on this layer as obstacles (excluding target net)
    # Use actual segment width for proper clearance calculation
    t0 = time.time()
    seg_count = 0
    _seg_cell_arrs = []
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        if seg.layer != route_layer:
            continue
        # Clearance needed: our track half-width + existing segment half-width + clearance.
        # The exact capsule keep-out measures from the real segment, so the blocked
        # halo matches the true clearance envelope without the grid-rounding that used
        # to leave connection traces within clearance of signal copper (#146/#173).
        seg_expansion_mm = (route_track_w / 2 + seg.width / 2
                            + config.layer_clearance(  # #498 (#434 cross-class)
                                route_layer, config.obstacle_clearance(seg.net_id)))
        # FFI batching (2026-08-14): same pattern as the via loop above --
        # accumulate, then one Rust call for the whole (single-layer) set.
        _ca = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                          seg.end_x, seg.end_y,
                                          seg_expansion_mm, coord.grid_step)
        if len(_ca):
            _seg_cell_arrs.append(_ca)
        seg_count += 1
    if _seg_cell_arrs:
        _call = (np.concatenate(_seg_cell_arrs) if len(_seg_cell_arrs) > 1
                 else _seg_cell_arrs[0])
        _rows = np.empty((len(_call), 3), dtype=np.int32)
        _rows[:, :2] = _call
        _rows[:, 2] = layer_idx
        obstacles.add_blocked_cells_batch(np.ascontiguousarray(_rows))
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
        r_mm = (via.size / 2 + route_track_w / 2
                + config.layer_clearance(  # #498: the route meets it on route_layer
                    route_layer, config.obstacle_clearance(via.net_id))
                + config.grid_step / 2)
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
    # Board-first, exactly as the signal obstacle map does (#D11). The plane
    # engines build their own map and never call add_drill_hole_obstacles, so
    # fixing that one alone left plane taps / region joins / reconnects still
    # pricing NPTH at the flat 0.20 fab floor on a board declaring more.
    # Raise-only: a board declaring nothing is byte-identical.
    from obstacle_map import resolve_hole_clearance
    npth_clr = max(config.clearance, defaults.NPTH_TO_TRACK_CLEARANCE,
                   resolve_hole_clearance(pcb_data, config))
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
    _add_board_edge_track_obstacles(obstacles, pcb_data, config, layer_idx,
                                    track_width=route_track_w)
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
