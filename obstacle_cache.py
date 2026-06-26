"""
Net obstacle caching for PCB routing.

Provides pre-computation and incremental updates for per-net obstacles,
dramatically speeding up routing by avoiding redundant obstacle calculations.
"""

from typing import List, Tuple, Dict, Set
from dataclasses import dataclass, field
import math
import numpy as np

from kicad_parser import PCBData
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map, iter_pad_blocked_cells, \
    pad_blocked_cells_array, segment_blocked_cells_array, circle_offsets
from net_queries import expand_pad_layers


# Import Rust router
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap
except ImportError:
    GridObstacleMap = None


_PACK_OFFSET = 1 << 20  # grid coords stay well within +/-2^20 at any allowed grid step

# Bitmap dedupe allocates one bool per cell in the rows' bounding box; cap the
# allocation (200M bools = 200MB) and fall back to sorting for outliers.
_BITMAP_DEDUPE_MAX_CELLS = 200_000_000


def _unique_rows(arr: np.ndarray) -> np.ndarray:
    """Row-deduplicate an int32 (N,2) or (N,3) cell array.

    Equivalent to np.unique(arr, axis=0) except for row order, which callers
    must not rely on (these are unordered cell sets). np.unique(axis=0) sorts
    rows as void records, which dominated routing time at fine grid steps;
    marking cells in a bounding-box bitmap dedupes without sorting at all.
    """
    mins = arr.min(axis=0)
    rel = arr - mins  # stays int32; ravel_multi_index widens internally
    dims = (rel.max(axis=0) + 1).astype(np.int64)
    total = int(np.prod(dims))
    if total > _BITMAP_DEDUPE_MAX_CELLS:
        # Sparse outlier (huge extent): pack rows into scalar int64 keys so
        # np.unique sorts scalars instead of void records.
        a = arr.astype(np.int64)
        key = ((a[:, 0] + _PACK_OFFSET) << 21) | (a[:, 1] + _PACK_OFFSET)
        if a.shape[1] == 3:
            key = (key << 8) | a[:, 2]
        _, idx = np.unique(key, return_index=True)
        return arr[idx]
    seen = np.zeros(total, dtype=bool)
    seen[np.ravel_multi_index(rel.T, dims)] = True
    uniq = np.flatnonzero(seen)
    out = np.empty((len(uniq), arr.shape[1]), dtype=arr.dtype)
    for i, col in enumerate(np.unravel_index(uniq, dims)):
        out[:, i] = col + mins[i]
    return out


@dataclass
class NetObstacleData:
    """Cached obstacle data for a single net.

    Pre-computed blocked cells and vias for fast batch adding.
    Uses numpy arrays for memory efficiency (12 bytes per cell vs 72+ for tuples).
    """
    # Blocked cells as numpy array of shape (N, 3) with columns [gx, gy, layer_idx]
    # dtype=int32 for efficient storage
    blocked_cells: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.int32))
    # Blocked via positions as numpy array of shape (M, 2) with columns [gx, gy]
    blocked_vias: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.int32))


def precompute_net_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              extra_clearance: float = 0.0,
                              diagonal_margin: float = 0.25) -> NetObstacleData:
    """Pre-compute obstacle cells for a single net.

    Returns cached data that can be quickly added to obstacle maps via batch operations.

    Args:
        pcb_data: PCB data structure
        net_id: Net ID to compute obstacles for
        config: Routing configuration
        extra_clearance: Additional clearance (for diff pair routing)
        diagonal_margin: Extra margin for via blocking near diagonal segments

    Returns:
        NetObstacleData with blocked_cells and blocked_vias lists
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)

    # Collect cell arrays; deduplicated with np.unique at the end (same
    # unique-cell semantics as the per-cell sets this replaces)
    blocked_cells_set: List["np.ndarray"] = []
    blocked_vias_set: List["np.ndarray"] = []

    # Precompute per-layer expansion values for impedance-controlled and power net routing
    # Use to_grid_dist_safe for via-related clearances to avoid grid quantization DRC errors
    expansion_mm_by_layer = {}
    via_block_mm_by_layer = {}
    layer_widths = []  # per-layer future-routing-track width (impedance / power)
    for layer_name in config.layers:
        # Use per-net width for power nets, otherwise layer width (impedance) or default
        layer_width = config.get_net_track_width(net_id, layer_name)
        layer_widths.append(layer_width)
        expansion_mm = layer_width / 2 + config.clearance + config.track_width / 2 + extra_clearance
        # Float keep-out half-width for the capsule segment stamp (no floor): the
        # true perpendicular clearance, so off-grid / diagonal tracks are covered.
        expansion_mm_by_layer[layer_name] = max(coord.grid_step, expansion_mm)
        # Segment via-block: future ROUTE via (config.via_size) near this net's copper.
        via_block_mm = config.via_size / 2 + layer_width / 2 + config.clearance + extra_clearance
        via_block_mm_by_layer[layer_name] = via_block_mm

    # Process segments. Keep-out from the segment's ACTUAL width, not just the
    # net's configured width: a pre-existing wide/diff-pair trace (e.g. a 0.2mm
    # trunk placed by route_diff) under-reserved when stamped at the default
    # 0.127 track width, letting a later track graze its edge (#172). Mirror the
    # via path below, which already keeps out from the via's actual size. Only
    # segments wider than the configured width grow their keep-out, so normal
    # tracks (seg.width == configured) are unaffected - no blanket margin.
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        layer_w = config.get_net_track_width(net_id, seg.layer)
        seg_w = seg.width if (getattr(seg, 'width', 0) and seg.width > 0) else layer_w
        own_half = max(layer_w, seg_w) / 2
        if seg_w <= layer_w:
            expansion_mm = expansion_mm_by_layer.get(seg.layer, coord.grid_step)
            via_block_mm = via_block_mm_by_layer.get(seg.layer)
        else:
            expansion_mm = max(coord.grid_step,
                               own_half + config.clearance + config.track_width / 2 + extra_clearance)
            via_block_mm = config.via_size / 2 + own_half + config.clearance + extra_clearance
        _collect_segment_obstacles(seg, coord, layer_idx, expansion_mm,
                                   blocked_cells_set, blocked_vias_set, via_block_mm)

    # Process vias. Keep-out from the via's ACTUAL size, not config.via_size: a
    # fanout via-in-pad is larger (e.g. 0.45 vs 0.3), and using config under-
    # expanded it by ~half the size difference. At grid 0.1 that was masked by the
    # off-grid offset (off_cells) of those vias; at grid 0.05 the same vias land
    # on-grid (off_cells=0), exposing 74 track-via grazes. Matches the non-cache
    # add_net_vias_as_obstacles, which already uses via.size.
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        vs = via.size if (getattr(via, 'size', 0) and via.size > 0) else config.via_size
        via_track_list = [max(1, coord.to_grid_dist_safe(
            vs / 2 + lw / 2 + config.clearance + extra_clearance)) for lw in layer_widths]
        # Via-via: this via (actual size) vs a future ROUTE via (config.via_size).
        # Float radius (no floor) so the disc threshold blocks the true clearance.
        via_via_radius = max(1.0, (vs / 2 + config.via_size / 2 + config.clearance) * coord.inv_step)
        _collect_via_obstacles(via, coord, num_layers, via_track_list,
                                via_via_radius, diagonal_margin,
                                blocked_cells_set, blocked_vias_set)

    # Process pads
    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _collect_pad_obstacles(pad, coord, layer_map, config, extra_clearance,
                                blocked_cells_set, blocked_vias_set)

    # Concatenate and deduplicate (the Rust map refcounts batch adds, so
    # each cell must appear once per net - same as the old set semantics)
    if blocked_cells_set:
        blocked_cells_arr = _unique_rows(np.concatenate(blocked_cells_set))
    else:
        blocked_cells_arr = np.empty((0, 3), dtype=np.int32)

    if blocked_vias_set:
        blocked_vias_arr = _unique_rows(np.concatenate(blocked_vias_set))
    else:
        blocked_vias_arr = np.empty((0, 2), dtype=np.int32)

    return NetObstacleData(
        blocked_cells=blocked_cells_arr,
        blocked_vias=blocked_vias_arr
    )


def _collect_segment_obstacles(seg, coord: GridCoord, layer_idx: int,
                                track_margin_mm: float,
                                blocked_cells: List["np.ndarray"],
                                blocked_vias: List["np.ndarray"],
                                via_block_mm: float):
    """Collect segment obstacle cells into lists (no obstacle map modification).

    Both keep-outs are an exact capsule measured from the REAL float segment
    (handles off-grid endpoints + diagonals): the track at `track_margin_mm`, the
    via at `via_block_mm`. Matches build_base's _add_segment_obstacle."""
    cells = segment_blocked_cells_array(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                                        track_margin_mm, coord.grid_step)
    rows = np.empty((len(cells), 3), dtype=np.int32)
    rows[:, :2] = cells
    rows[:, 2] = layer_idx
    blocked_cells.append(rows)

    blocked_vias.append(segment_blocked_cells_array(
        seg.start_x, seg.start_y, seg.end_x, seg.end_y, via_block_mm, coord.grid_step))


def _collect_via_obstacles(via, coord: GridCoord, num_layers: int,
                            via_track_expansion_grid, via_via_expansion_grid: int,
                            diagonal_margin: float,
                            blocked_cells: List["np.ndarray"],
                            blocked_vias: List["np.ndarray"]):
    """Collect via obstacle cells into sets (no obstacle map modification).

    Args:
        via_track_expansion_grid: Either a single int or list of ints (per-layer) for impedance control
    """
    gx, gy = coord.to_grid(via.x, via.y)
    center = np.array([gx, gy], dtype=np.int32)

    # Sub-grid offset of the real via centre from its quantized cell. An off-grid
    # via (e.g. a BGA fanout via-in-pad) has its blocked disc centred on the
    # ROUNDED cell, so foreign copper that clears the rounded centre still grazes
    # the TRUE centre by up to the offset (issue #70). Grow each blocking radius by
    # this offset so the disc covers the real via. On-grid (router-placed) vias
    # have offset ~0 and are unchanged. Mirror of obstacle_map._add_via_obstacle.
    off_cells = math.hypot(via.x - gx * coord.grid_step,
                           via.y - gy * coord.grid_step) / coord.grid_step

    def add_layer_cells(offs, layer_idx):
        cells = center + offs
        rows = np.empty((len(cells), 3), dtype=np.int32)
        rows[:, :2] = cells
        rows[:, 2] = layer_idx
        blocked_cells.append(rows)

    # Support per-layer expansion for impedance-controlled routing
    if isinstance(via_track_expansion_grid, list):
        for layer_idx in range(num_layers):
            layer_expansion = via_track_expansion_grid[layer_idx]
            radius = layer_expansion + diagonal_margin + off_cells
            add_layer_cells(circle_offsets(int(math.ceil(radius)), radius ** 2), layer_idx)
    else:
        radius = via_track_expansion_grid + diagonal_margin + off_cells
        offs = circle_offsets(int(math.ceil(radius)), radius ** 2)
        for layer_idx in range(num_layers):
            add_layer_cells(offs, layer_idx)

    via_radius = via_via_expansion_grid + off_cells
    via_offs = circle_offsets(int(math.ceil(via_radius)), via_radius * via_radius)
    blocked_vias.append(center + via_offs)


def _collect_pad_obstacles(pad, coord: GridCoord, layer_map: Dict[str, int],
                            config: GridRouteConfig, extra_clearance: float,
                            blocked_cells: List["np.ndarray"],
                            blocked_vias: List["np.ndarray"]):
    """Collect pad obstacle cells into sets (no obstacle map modification).

    Uses rectangular-with-rounded-corners pattern matching other pad blocking functions.
    """
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    # Sub-cell offset of the real pad center from its grid cell (issue #70).
    off_x = pad.global_x - gx * coord.grid_step
    off_y = pad.global_y - gy * coord.grid_step
    half_width = pad.size_x / 2
    half_height = pad.size_y / 2
    margin = config.track_width / 2 + config.clearance + extra_clearance

    # Custom comb/finger pads: rasterize the real copper polygon(s), leaving the
    # finger channels open, instead of the bounding box (issue #188). This is the
    # per-net obstacle CACHE the routing loop actually uses, so the fix must live
    # here as well as in _add_pad_obstacle.
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        from obstacle_map import _rasterize_polygon
        expanded_layers = expand_pad_layers(pad.layers, config.layers)
        layer_idxs = [layer_map.get(l) for l in expanded_layers]
        layer_idxs = [li for li in layer_idxs if li is not None]
        on_copper = any(l.endswith('.Cu') for l in expanded_layers)
        via_margin = config.via_size / 2 + config.clearance + config.grid_step / 2
        for poly in pad_polys:
            gxf, gyf, inside, edist = _rasterize_polygon(poly, coord, margin)
            if gxf is not None:
                m = inside | (edist <= margin)
                if m.any():
                    cells = np.column_stack([gxf[m], gyf[m]]).astype(np.int32)
                    for li in layer_idxs:
                        rows = np.empty((len(cells), 3), dtype=np.int32)
                        rows[:, :2] = cells
                        rows[:, 2] = li
                        blocked_cells.append(rows)
            if on_copper:
                vgxf, vgyf, vin, ved = _rasterize_polygon(poly, coord, via_margin)
                if vgxf is not None:
                    vm = vin | (ved <= via_margin)
                    if vm.any():
                        blocked_vias.append(np.column_stack([vgxf[vm], vgyf[vm]]).astype(np.int32))
        return

    # Corner radius based on pad shape (circle/oval use min dimension, roundrect uses rratio)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(half_width, half_height)
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0

    expanded_layers = expand_pad_layers(pad.layers, config.layers)

    # Batched rasterization (issue #35): same cell sets as the generator
    # (pad_blocked_cells_array is bit-identical to iter_pad_blocked_cells)
    cells = pad_blocked_cells_array(gx, gy, half_width, half_height, margin,
                                    config.grid_step, corner_radius,
                                    off_x=off_x, off_y=off_y,
                                    rotation_deg=pad.rect_rotation)
    for layer in expanded_layers:
        layer_idx = layer_map.get(layer)
        if layer_idx is not None:
            rows = np.empty((len(cells), 3), dtype=np.int32)
            rows[:, :2] = cells
            rows[:, 2] = layer_idx
            blocked_cells.append(rows)

    # Via blocking around pads
    if any(layer.endswith('.Cu') for layer in expanded_layers):
        # Add half grid step buffer to account for grid quantization errors
        via_margin = config.via_size / 2 + config.clearance + config.grid_step / 2
        blocked_vias.append(pad_blocked_cells_array(gx, gy, half_width, half_height,
                                                    via_margin, config.grid_step, corner_radius,
                                                    off_x=off_x, off_y=off_y,
                                                    rotation_deg=pad.rect_rotation))


def precompute_all_net_obstacles(pcb_data: PCBData, net_ids: List[int], config: GridRouteConfig,
                                   extra_clearance: float = 0.0,
                                   diagonal_margin: float = 0.25) -> Dict[int, NetObstacleData]:
    """Pre-compute obstacles for all nets to route.

    This is called once at the start of routing to build a cache that
    dramatically speeds up per-route obstacle building.

    Args:
        pcb_data: PCB data structure
        net_ids: List of net IDs to precompute obstacles for
        config: Routing configuration
        extra_clearance: Additional clearance (for diff pair routing)
        diagonal_margin: Extra margin for via blocking near diagonal segments

    Returns:
        Dict mapping net_id to NetObstacleData
    """
    cache: Dict[int, NetObstacleData] = {}
    for net_id in net_ids:
        cache[net_id] = precompute_net_obstacles(pcb_data, net_id, config,
                                                   extra_clearance, diagonal_margin)
    return cache


def add_net_obstacles_from_cache(obstacles: GridObstacleMap, cache_data: NetObstacleData):
    """Add a net's obstacles from cache using batch operations.

    This is much faster than the traditional per-segment/via/pad approach
    because it uses batch FFI calls instead of individual cell additions.

    Args:
        obstacles: The obstacle map to add to
        cache_data: Pre-computed NetObstacleData for the net
    """
    if len(cache_data.blocked_cells) > 0:
        # Pass numpy array directly to Rust (no conversion needed)
        obstacles.add_blocked_cells_batch(cache_data.blocked_cells)
    if len(cache_data.blocked_vias) > 0:
        obstacles.add_blocked_vias_batch(cache_data.blocked_vias)


def remove_net_obstacles_from_cache(obstacles: GridObstacleMap, cache_data: NetObstacleData):
    """Remove a net's obstacles from the map using batch operations.

    Used to temporarily remove a net's obstacles before routing it,
    so the router can use the net's own stubs/pads as endpoints.

    Args:
        obstacles: The obstacle map to remove from
        cache_data: Pre-computed NetObstacleData for the net
    """
    if len(cache_data.blocked_cells) > 0:
        obstacles.remove_blocked_cells_batch(cache_data.blocked_cells)
    if len(cache_data.blocked_vias) > 0:
        obstacles.remove_blocked_vias_batch(cache_data.blocked_vias)


def build_working_obstacle_map(base_obstacles: GridObstacleMap,
                                 net_obstacles_cache: Dict[int, NetObstacleData]) -> GridObstacleMap:
    """Build a working obstacle map with base + all cached net obstacles.

    This creates a complete obstacle map at startup that can be incrementally
    updated during routing (remove current net, add new route segments).

    Args:
        base_obstacles: Base obstacle map with static obstacles
        net_obstacles_cache: Pre-computed obstacles for all nets to route

    Returns:
        Working obstacle map ready for incremental updates
    """
    working = base_obstacles.clone_fresh()
    for net_id, cache_data in net_obstacles_cache.items():
        add_net_obstacles_from_cache(working, cache_data)
    return working


def update_net_obstacles_after_routing(pcb_data, net_id: int, result: Dict,
                                         config, net_obstacles_cache: Dict[int, NetObstacleData]):
    """Update the net obstacles cache after a successful route.

    Recomputes the net's obstacles to include the newly routed segments/vias.
    This should be called after add_route_to_pcb_data().

    Args:
        pcb_data: PCB data (already updated with new route)
        net_id: The net that was just routed
        result: Routing result dict with new_segments and new_vias
        config: Routing configuration
        net_obstacles_cache: Cache to update
    """
    # Recompute this net's obstacles (now includes the new route)
    net_obstacles_cache[net_id] = precompute_net_obstacles(
        pcb_data, net_id, config,
        extra_clearance=0.0, diagonal_margin=0.25
    )


# =============================================================================
# Via Placement Obstacle Cache
# =============================================================================
# Separate cache for via placement in route_planes.py. Uses different clearance
# calculations than routing (actual segment widths vs uniform track width).

@dataclass
class ViaPlacementObstacleData:
    """Cached obstacle data for via placement (used by route_planes.py).

    Stores blocked via positions and routing cells per layer for incremental
    updates when ripping up nets during plane via placement.
    """
    # Blocked via positions as numpy array of shape (N, 2) with columns [gx, gy]
    # May contain duplicates to match reference counting in GridObstacleMap
    blocked_vias: np.ndarray
    # Blocked routing cells per layer as dict: layer_name -> numpy array of shape (M, 2) [gx, gy]
    blocked_cells_by_layer: Dict[str, np.ndarray]


def _bresenham_line_points(gx1: int, gy1: int, gx2: int, gy2: int) -> "np.ndarray":
    """Grid cells along the Bresenham line (both endpoints inclusive) as an (L, 2)
    int32 array. Reproduces precompute_via_placement_obstacles' original integer
    midpoint walk (err = d // 2) cell-for-cell so the broadcast circle stamps land
    on exactly the same line as the old scalar loops."""
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1

    pts = []
    x, y = gx1, gy1
    if dx > dy:
        err = dx // 2
        while x != gx2:
            pts.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != gy2:
            pts.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    pts.append((x, y))  # final point
    return np.array(pts, dtype=np.int32)


def precompute_via_placement_obstacles(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    all_copper_layers: List[str]
) -> ViaPlacementObstacleData:
    """
    Pre-compute via placement obstacle positions for a single net.

    Used for incremental updates when ripping up nets - we can quickly remove
    this net's blocked positions instead of rebuilding the entire obstacle map.

    Unlike precompute_net_obstacles (for routing), this uses actual segment widths
    for clearance calculations rather than a uniform track width.

    Args:
        pcb_data: PCB data structure
        net_id: Net ID to compute obstacles for
        config: Routing configuration
        all_copper_layers: All copper layers for routing obstacle cells

    Returns:
        ViaPlacementObstacleData with blocked_vias and blocked_cells_by_layer
    """
    import math
    coord = GridCoord(config.grid_step)

    # Collect blocked positions as numpy chunks (one per segment/via stamp), then
    # concatenate. Duplicate counts are preserved - the obstacle map uses
    # reference counting, so removal must match exactly what was added. Each stamp
    # is the Bresenham line broadcast against a circular offset mask (circle_offsets
    # reproduces the legacy ex/ey<=r^2 loops cell-for-cell, in the same order), so
    # the cell multiset is identical to the old scalar double loops (issue #35 pattern).
    via_chunks: List["np.ndarray"] = []
    cell_chunks: Dict[str, List["np.ndarray"]] = {layer: [] for layer in all_copper_layers}

    # Process this net's segments - they block via placement
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        # Via-segment clearance (via can't overlap segment on any layer)
        seg_expansion_mm = config.via_size / 2 + seg.width / 2 + config.clearance
        seg_expansion_sq = (seg_expansion_mm / config.grid_step) ** 2
        seg_expansion_int = int(math.ceil(math.sqrt(seg_expansion_sq)))

        # Routing clearance for this segment's layer (circular test: use float
        # squared radius + ceil scan bound so we don't under-reserve by ~1 cell)
        route_expansion_mm = config.track_width / 2 + seg.width / 2 + config.clearance
        route_expansion_sq = (route_expansion_mm / config.grid_step) ** 2
        route_expansion_grid = max(1, int(math.ceil(math.sqrt(route_expansion_sq))))

        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        line = _bresenham_line_points(gx1, gy1, gx2, gy2)  # (L, 2) int32

        # Via positions: stamp the via keep-out disc at every line cell.
        via_offs = circle_offsets(seg_expansion_int, seg_expansion_sq)
        via_chunks.append((line[:, None, :] + via_offs[None, :, :]).reshape(-1, 2))

        # Routing cells on this segment's layer (circular, matching build_routing_obstacle_map)
        if seg.layer in cell_chunks:
            route_offs = circle_offsets(route_expansion_grid, route_expansion_sq)
            cell_chunks[seg.layer].append((line[:, None, :] + route_offs[None, :, :]).reshape(-1, 2))

    # Process this net's vias - they block via placement and routing on all layers
    via_via_expansion_mm = config.via_size + config.clearance
    via_via_radius_sq = (via_via_expansion_mm / config.grid_step) ** 2
    via_via_radius_int = int(math.ceil(math.sqrt(via_via_radius_sq)))

    via_route_expansion_sq = ((config.via_size / 2 + config.track_width / 2 + config.clearance) / config.grid_step) ** 2
    via_route_expansion = max(1, int(math.ceil(math.sqrt(via_route_expansion_sq))))

    via_via_offs = circle_offsets(via_via_radius_int, via_via_radius_sq)
    via_route_offs = circle_offsets(via_route_expansion, via_route_expansion_sq)

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        center = np.array([[gx, gy]], dtype=np.int32)  # (1, 2)
        # Block via positions (via-via clearance)
        via_chunks.append(center + via_via_offs)
        # Block routing cells on all layers (via spans all layers)
        route_block = center + via_route_offs
        for layer in all_copper_layers:
            cell_chunks[layer].append(route_block)

    # Concatenate chunks into the final arrays (keep duplicates for reference counting)
    if via_chunks:
        blocked_vias_arr = np.concatenate(via_chunks).astype(np.int32, copy=False)
    else:
        blocked_vias_arr = np.empty((0, 2), dtype=np.int32)

    blocked_cells_arrays = {}
    for layer, chunks in cell_chunks.items():
        if chunks:
            blocked_cells_arrays[layer] = np.concatenate(chunks).astype(np.int32, copy=False)
        else:
            blocked_cells_arrays[layer] = np.empty((0, 2), dtype=np.int32)

    return ViaPlacementObstacleData(
        blocked_vias=blocked_vias_arr,
        blocked_cells_by_layer=blocked_cells_arrays
    )

