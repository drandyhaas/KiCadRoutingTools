"""
Proximity cost functions for PCB routing obstacle maps.

Handles track proximity costs, stub proximity costs, BGA proximity costs,
and cross-layer track alignment tracking.
"""
from __future__ import annotations

from typing import List, Tuple, Dict, Set
import numpy as np

from kicad_parser import PCBData
from routing_config import GridRouteConfig, GridCoord
from bresenham_utils import walk_line

# Module-level cache for pre-computed proximity offset tables
# Key: (radius_grid, cost_grid) -> List of (ex, ey, cost) tuples
_proximity_offset_cache: Dict[Tuple[int, int], List[Tuple[int, int, int]]] = {}
_proximity_offset_np_cache: dict = {}


def _get_proximity_offsets(radius_grid: int, cost_grid: int,
                           half_width_grid: int = 0) -> List[Tuple[int, int, int]]:
    """Get pre-computed proximity offsets and costs for a given radius.

    Returns a list of (ex, ey, cost) tuples for all grid cells within
    radius + half_width. half_width_grid > 0 makes the field WIDTH-AWARE
    (#585 item 3): full cost across the copper (dist <= half_width), then
    the standard linear falloff measured from the copper EDGE -- so a 2 mm
    power trunk casts a wider field than a 0.1 mm signal instead of the
    same centerline disk. Uses caching to avoid recomputing the same table.
    """
    cache_key = (radius_grid, cost_grid, half_width_grid)
    if cache_key in _proximity_offset_cache:
        return _proximity_offset_cache[cache_key]

    offsets = []
    reach = radius_grid + half_width_grid
    reach_sq = reach * reach

    for ex in range(-reach, reach + 1):
        for ey in range(-reach, reach + 1):
            dist_sq = ex * ex + ey * ey
            if dist_sq <= reach_sq:
                dist = dist_sq ** 0.5
                # Falloff from the copper edge; exact original formula when
                # half_width_grid == 0 (float-identical: dist/radius path).
                edge_dist = max(0.0, dist - half_width_grid)
                proximity = 1.0 - (edge_dist / radius_grid) if radius_grid > 0 else 1.0
                if proximity < 0.0:
                    continue
                cost = int(proximity * cost_grid)
                offsets.append((ex, ey, cost))

    _proximity_offset_cache[cache_key] = offsets
    return offsets


def _get_proximity_offsets_np(radius_grid: int, cost_grid: int,
                              half_width_grid: int = 0) -> 'np.ndarray':
    """(O, 3) int64 [ex, ey, cost] mirror of _get_proximity_offsets, same
    table order, cached separately."""
    cache_key = (radius_grid, cost_grid, half_width_grid)
    arr = _proximity_offset_np_cache.get(cache_key)
    if arr is None:
        offsets = _get_proximity_offsets(radius_grid, cost_grid, half_width_grid)
        arr = (np.array(offsets, dtype=np.int64) if offsets
               else np.empty((0, 3), dtype=np.int64))
        _proximity_offset_np_cache[cache_key] = arr
    return arr


# Import Rust router
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads

try:
    from grid_router import GridObstacleMap
except ImportError:
    GridObstacleMap = None

import env_knobs


def add_stub_proximity_costs(obstacles: GridObstacleMap, unrouted_stubs: List[Tuple[float, float]],
                              config: GridRouteConfig):
    """Add stub proximity costs to the obstacle map.

    Uses batch Rust method for performance.
    Also registers zone centers for direction-aware cost calculation.

    via_proximity_cost no longer participates here: 0 used to mean "hard-BLOCK
    vias within the stub radius" (the one knob where 0 was the STRONGEST
    setting, a documented ~200x CPU hazard); 0 now means "no extra via cost
    from proximity" like every other knob's 0-is-off, and the Rust-side ban
    machinery (block_vias + the B2 refcount ledger) is removed.
    """
    if not unrouted_stubs:
        return

    coord = GridCoord(config.grid_step)
    stub_radius_grid = coord.to_grid_dist(config.stub_proximity_radius)
    stub_cost_grid = config.cell_cost(config.stub_proximity_cost)

    # Convert stub positions to grid coordinates
    stub_grid_positions = []
    for stub in unrouted_stubs:
        stub_x, stub_y = stub[0], stub[1]
        gcx, gcy = coord.to_grid(stub_x, stub_y)
        stub_grid_positions.append((gcx, gcy))

    # Use batch Rust method for performance
    obstacles.add_stub_proximity_costs_batch(
        stub_grid_positions, stub_radius_grid, stub_cost_grid
    )


def proximity_max_zone_rects(config: GridRouteConfig, coord: GridCoord):
    """Inclusive grid rectangles of the BGA ESCAPE FIELDS (zone expanded by
    bga_proximity_radius) for 'zoned' sum composition: inside them sources
    compose by MAX (the stacked foreign-stub fields there price a net's
    MANDATORY approach, not an avoidable crowd), outside by SUM. Independent
    of the bga_proximity_cost knob -- this is a composition boundary, not the
    BGA cost. Returns None unless zoned mode is active and zones exist."""
    if env_knobs.PROXIMITY_SUM_MODE != 'zoned':
        return None
    if config.package_proximity_zones is not None:
        zone_list = list(config.package_proximity_zones)
    else:
        zone_list = [(z[0], z[1], z[2], z[3], config.bga_proximity_radius)
                     for z in config.bga_exclusion_zones]
    if not zone_list:
        return None
    rects = []
    for min_x, min_y, max_x, max_y, radius_mm in zone_list:
        r = coord.to_grid_dist(radius_mm)
        gx0, gy0 = coord.to_grid(min_x, min_y)
        gx1, gy1 = coord.to_grid(max_x, max_y)
        rects.append((gx0 - r, gy0 - r, gx1 + r, gy1 + r))
    return rects


def _stub_layer_m(config: GridRouteConfig) -> float:
    """M for layer-aware stub proximity (#585 item 1): other layers pay the
    stub cost / M; the stub's own layer keeps full cost. Default M =
    n_layers / 2 (a 2-layer board is unchanged; a 6-layer board discounts
    3x -- more alternative layers = less protection needed on each).
    Returns 1.0 (no discount, legacy) when the feature is off."""
    if not env_knobs.STUB_LAYER_AWARE:
        return 1.0
    m = env_knobs.STUB_LAYER_M or (len(config.layers) / 2.0)
    return max(1.0, m)


def _stub_surplus_arrays(points_by_key, radius_grid: int, surplus_cost: int,
                         coord: GridCoord, layer_map: Dict[str, int]):
    """Own-layer surplus disks for layer-aware stub proximity: for each
    source key, an (N, 4) [layer, gx, gy, cost] array of the falloff disks
    stamped ONLY on each point's own layer, max-deduped within the source
    (same within-source rule as everything else). Points whose layer is not
    in layer_map (or that carry no layer) contribute no surplus -- they stay
    base-tier-only, slightly under-protected rather than over."""
    out = {}
    if surplus_cost <= 0 or radius_grid <= 0:
        return out
    offsets = _get_proximity_offsets(radius_grid, surplus_cost)
    if not offsets:
        return out
    off = np.array(offsets, dtype=np.int64)  # columns: ex, ey, cost
    _OFF = 1 << 23
    for key, pts in points_by_key.items():
        rows = []
        for p in pts:
            li = layer_map.get(p[2]) if len(p) > 2 else None
            if li is None:
                continue
            gx, gy = coord.to_grid(p[0], p[1])
            r = np.empty((len(off), 4), dtype=np.int64)
            r[:, 0] = li
            r[:, 1] = gx + off[:, 0]
            r[:, 2] = gy + off[:, 1]
            r[:, 3] = off[:, 2]
            rows.append(r)
        if not rows:
            continue
        allr = np.concatenate(rows)
        packed = ((allr[:, 0] << 48) | ((allr[:, 1] + _OFF) << 24)
                  | (allr[:, 2] + _OFF))
        uniq, inv = np.unique(packed, return_inverse=True)
        maxs = np.zeros(len(uniq), dtype=np.int64)
        np.maximum.at(maxs, inv, allr[:, 3])
        arr = np.empty((len(uniq), 4), dtype=np.int32)
        arr[:, 0] = (uniq >> 48).astype(np.int32)
        arr[:, 1] = ((uniq >> 24) & ((1 << 24) - 1)).astype(np.int32) - _OFF
        arr[:, 2] = (uniq & ((1 << 24) - 1)).astype(np.int32) - _OFF
        arr[:, 3] = maxs.astype(np.int32)
        out[('stub_surplus', key)] = arr
    return out


def apply_stub_proximity(obstacles: GridObstacleMap, pcb_data: PCBData,
                         stub_net_ids: List[int],
                         all_stubs: List[Tuple[float, float]],
                         config: GridRouteConfig,
                         ghost_via_groups: Dict[int, List[Tuple[int, int]]] = None,
                         layer_map: Dict[str, int] = None):
    """Single entry point for the STUB proximity map (foreign stubs/chip pads
    + ripped-route via ghosts), composing per the active mode.

    Max mode (default): byte-identical to the historical calls -- the flat
    ``all_stubs`` batch (per-cell max), then the ghost vias as a second flat
    batch at the ripped-route falloff (formerly merge_ripped_route_costs).

    Sum mode (KICAD_PROXIMITY_SUM): each SOURCE -- one net's stubs+chip pads,
    or one ripped net's ghost vias -- is deduped internally by max (a net's
    six connector stubs are one source), then sources ADD per cell. One
    grouped Rust call per stamp sequence; the caller's clear (or fresh clone)
    brackets it exactly like max mode, and stamping the same sequence after a
    clear reproduces identical values (no cross-prepare accumulation).

    Layer-aware mode (#585 item 1, KICAD_STUB_LAYER_AWARE, composable with
    either composition mode): the all-layer stub-map stamp drops to
    cost / M (M = n_layers/2 by default) and each source's OWN-layer surplus
    (cost - cost/M falloff disks) is RETURNED as [layer, gx, gy, cost]
    arrays for the caller to feed into merge_track_proximity_costs'
    ghost_costs -- riding the layer map keeps composition (max/sum/zoned)
    and the via formula consistent: a via INTO a stub's layer pays more
    than one leaving it. Ghost vias stay full-cost all-layer (a barrel
    occupies every layer). Returns {} when not armed.

    ghost_via_groups must already be C1-filtered (re-routed nets dropped) --
    both modes assume each source appears at most once.
    """
    coord = GridCoord(config.grid_step)
    ghost_active = (ghost_via_groups and
                    config.ripped_route_avoidance_cost > 0)

    m = _stub_layer_m(config)
    stub_radius_grid = coord.to_grid_dist(config.stub_proximity_radius)
    full_cost = config.cell_cost(config.stub_proximity_cost)
    base_cost = int(full_cost / m)
    surplus_cost = full_cost - base_cost
    surplus: Dict = {}

    if not env_knobs.PROXIMITY_SUM:
        if all_stubs:
            if m > 1.0:
                grid_pts = [coord.to_grid(p[0], p[1]) for p in all_stubs]
                if base_cost > 0 and stub_radius_grid > 0:
                    obstacles.add_stub_proximity_costs_batch(
                        grid_pts, stub_radius_grid, base_cost)
                if layer_map:
                    surplus = _stub_surplus_arrays(
                        {'all': all_stubs}, stub_radius_grid, surplus_cost,
                        coord, layer_map)
            else:
                add_stub_proximity_costs(obstacles, all_stubs, config)
        if ghost_active:
            all_via_positions = []
            for via_list in ghost_via_groups.values():
                all_via_positions.extend(via_list)
            if all_via_positions:
                obstacles.add_stub_proximity_costs_batch(
                    all_via_positions,
                    coord.to_grid_dist(config.ripped_route_avoidance_radius),
                    config.cell_cost(config.ripped_route_avoidance_cost))
        return surplus

    # softcap (#584): max + alpha*(sum-max) == (1-alpha)*max + alpha*sum,
    # composed with the EXISTING Rust primitives -- the flat batch max-inserts
    # the (1-alpha)-scaled all-sources field (same structure as max mode
    # above), then the grouped call below ADDS the alpha-scaled per-source
    # fields on top. alpha=0 reproduces max mode, alpha=1 sum mode (up to
    # integer-cost rounding). The stub-map SURPLUS (layer-aware m>1) is
    # emitted ONCE at full cost by this branch: it rides the layer map,
    # where merge_track_proximity_costs applies the same blend.
    alpha = (env_knobs.PROXIMITY_SOFTCAP_ALPHA
             if env_knobs.PROXIMITY_SUM_MODE == 'softcap' else None)
    if alpha is not None:
        mscale = 1.0 - alpha
        if all_stubs and stub_radius_grid > 0:
            mcost = int((base_cost if m > 1.0 else full_cost) * mscale)
            if mcost > 0:
                grid_pts = [coord.to_grid(p[0], p[1]) for p in all_stubs]
                obstacles.add_stub_proximity_costs_batch(
                    grid_pts, stub_radius_grid, mcost)
        if ghost_active:
            all_via_positions = []
            for via_list in ghost_via_groups.values():
                all_via_positions.extend(via_list)
            mrra = int(config.cell_cost(config.ripped_route_avoidance_cost)
                       * mscale)
            rra_radius_g = coord.to_grid_dist(
                config.ripped_route_avoidance_radius)
            if all_via_positions and mrra > 0 and rra_radius_g > 0:
                obstacles.add_stub_proximity_costs_batch(
                    all_via_positions, rra_radius_g, mrra)
    sscale = alpha if alpha is not None else 1.0

    # Sum component: (points, radius, cost) per source
    from connectivity import get_stub_endpoints
    from net_queries import get_chip_pad_positions
    groups = []
    pts_by_net = {}
    if full_cost > 0 and stub_radius_grid > 0:
        src_cost = int((base_cost if m > 1.0 else full_cost) * sscale)
        for nid in stub_net_ids:
            pts = (get_stub_endpoints(pcb_data, [nid])
                   + get_chip_pad_positions(pcb_data, [nid]))
            if pts:
                pts_by_net[nid] = pts
                groups.append(([coord.to_grid(p[0], p[1]) for p in pts],
                               stub_radius_grid, src_cost))
    if m > 1.0 and layer_map and pts_by_net:
        surplus = _stub_surplus_arrays(pts_by_net, stub_radius_grid,
                                       surplus_cost, coord, layer_map)
    if ghost_active:
        rra_radius = coord.to_grid_dist(config.ripped_route_avoidance_radius)
        rra_cost = int(config.cell_cost(config.ripped_route_avoidance_cost)
                       * sscale)
        for nid in sorted(ghost_via_groups):
            vias = ghost_via_groups[nid]
            if vias:
                groups.append(([tuple(v) for v in vias], rra_radius, rra_cost))
    if groups:
        obstacles.add_stub_proximity_costs_grouped(
            groups, proximity_max_zone_rects(config, coord))
    return surplus


# (add_bga_proximity_costs, the old base-map stub_proximity stamp, is deleted:
# it was dead in production after the B1 move to the cache below -- the stamp
# was wiped by prepare_obstacles_inplace's clear_stub_proximity before every
# single-ended net. compute_bga_proximity_cost_cells is the live path.)

# Reserved track_proximity_cache key for the BGA proximity cost cells
# (soft-knobs review B1): real net ids are positive, so -1 can never collide
# or be evicted by the per-net cache lifecycle.
BGA_PROXIMITY_CACHE_KEY = -1


def compute_bga_proximity_cost_cells(config: GridRouteConfig,
                                     num_layers: int) -> np.ndarray:
    """BGA proximity costs as an (N, 4) [layer, gx, gy, cost] array.

    Linear falloff from full cost at the zone edge to zero at
    bga_proximity_radius; zone-INTERIOR cells clamp to dist=0 = full
    edge-tier cost (so allowed-cells windows punched through the hard block
    are not free-via holes). Emitted for the LAYER proximity map via the
    track-proximity cache under
    BGA_PROXIMITY_CACHE_KEY (soft-knobs review B1): the old base-map
    stub_proximity stamp was wiped by prepare_obstacles_inplace's
    clear_stub_proximity() before every single-ended net, silently no-op'ing
    the knob in the most common path (and double-counting was the risk for
    diff pairs, whose obstacle build merges the cache on top of the base).
    Registered once; merge_track_proximity_costs re-applies it on every
    prepare in every path. Rows are replicated per layer because the old
    stub_proximity stamp was all-layer.
    """
    if config.bga_proximity_radius <= 0 or config.bga_proximity_cost <= 0 \
            or num_layers <= 0:
        return np.empty((0, 4), dtype=np.int32)

    # #585 item 4: per-package proximity zones (BGA/QFN/QFP, per-zone radius
    # scaled by pad count) when the engine filled them; legacy fallback =
    # the hard BGA zones at the flat radius.
    if config.package_proximity_zones is not None:
        zone_list = [(z[0], z[1], z[2], z[3], z[4])
                     for z in config.package_proximity_zones]
    else:
        zone_list = [(z[0], z[1], z[2], z[3], config.bga_proximity_radius)
                     for z in config.bga_exclusion_zones]
    if not zone_list:
        return np.empty((0, 4), dtype=np.int32)

    coord = GridCoord(config.grid_step)
    cost_grid = config.cell_cost(config.bga_proximity_cost)
    cells: Dict[Tuple[int, int], int] = {}

    for min_x, min_y, max_x, max_y, radius_mm in zone_list:
        radius_grid = coord.to_grid_dist(radius_mm)
        if radius_grid <= 0:
            continue
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        for gx in range(gmin_x - radius_grid, gmax_x + radius_grid + 1):
            for gy in range(gmin_y - radius_grid, gmax_y + radius_grid + 1):
                cx = max(gmin_x, min(gx, gmax_x))
                cy = max(gmin_y, min(gy, gmax_y))
                dist = ((gx - cx) ** 2 + (gy - cy) ** 2) ** 0.5
                if dist <= radius_grid:
                    proximity = 1.0 - (dist / radius_grid) if radius_grid > 0 else 1.0
                    cost = int(proximity * cost_grid)
                    key = (gx, gy)
                    if cost > cells.get(key, 0):
                        cells[key] = cost

    if not cells:
        return np.empty((0, 4), dtype=np.int32)
    base = np.array([[gx, gy, c] for (gx, gy), c in cells.items()], dtype=np.int32)
    rows = []
    for li in range(num_layers):
        layer_col = np.full((base.shape[0], 1), li, dtype=np.int32)
        rows.append(np.hstack([layer_col, base]))
    return np.vstack(rows)


def compute_track_proximity_for_net(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                                     layer_map: Dict[str, int]) -> np.ndarray:
    """Compute track proximity costs for a single net's segments.

    Returns a numpy array with columns [layer, gx, gy, cost] for efficient storage and batch merge.
    This allows incremental updates: compute once when route succeeds, remove when ripped up.

    Args:
        pcb_data: PCB data containing routed segments
        net_id: Net ID to compute proximity for
        config: Routing configuration with track_proximity_distance and track_proximity_cost
        layer_map: Mapping of layer names to layer indices

    Returns:
        numpy array of shape (N, 4) with columns [layer, gx, gy, cost], dtype int32
    """
    # Use dict internally for efficient max tracking, convert to numpy at end
    result: Dict[Tuple[int, int, int], int] = {}  # (layer, gx, gy) -> cost

    if config.track_proximity_distance <= 0 or config.track_proximity_cost <= 0:
        return np.empty((0, 4), dtype=np.int32)  # Feature disabled

    coord = GridCoord(config.grid_step)
    radius_grid = coord.to_grid_dist(config.track_proximity_distance)
    cost_grid = config.cell_cost(config.track_proximity_cost)

    # Sample every ~1mm along segments (not every grid step) for performance
    sample_interval = max(1, int(1.0 / config.grid_step))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue

        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        # Width-aware field (#585 item 3): falloff from the copper edge.
        # The offsets table is cached per (radius, cost, half-width), so the
        # few distinct widths on a board cost one table each.
        offsets = _get_proximity_offsets(
            radius_grid, cost_grid,
            coord.to_grid_dist((seg.width or 0.0) / 2.0))

        # Walk along segment using Bresenham, sampling every sample_interval points
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        err = dx - dy

        gx, gy = gx1, gy1
        step_count = 0

        while True:
            # Only process every sample_interval'th point
            if step_count % sample_interval == 0:
                # Add proximity costs using pre-computed offset table
                for ex, ey, cost in offsets:
                    key = (layer_idx, gx + ex, gy + ey)
                    # Store max cost at each cell
                    if key not in result or cost > result[key]:
                        result[key] = cost

            if gx == gx2 and gy == gy2:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx += sx
            if e2 < dx:
                err += dx
                gy += sy
            step_count += 1

    # Convert to numpy array
    if not result:
        return np.empty((0, 4), dtype=np.int32)

    arr = np.array([[layer, gx, gy, cost] for (layer, gx, gy), cost in result.items()], dtype=np.int32)
    return arr


_MERGE_MEMO: dict = {}
# Cap: entries pin their keyed objects alive (keepalive against id() reuse), so
# the memo must not grow unbounded in a long-lived GUI session. Cleared wholesale
# on overflow -- this is a within-run speedup, not a correctness cache. Bounded
# by bytes too: each entry pins a stacked copy of every source, and a board's
# fragility field alone can be a million rows.
_MERGE_MEMO_MAX = 32
_MERGE_MEMO_BYTES_MAX = 256 * 2 ** 20


def merge_track_proximity_costs(obstacles: GridObstacleMap,
                                 per_net_costs: Dict[int, np.ndarray],
                                 ghost_costs: Dict[int, np.ndarray] = None,
                                 config: GridRouteConfig = None):
    """Merge pre-computed per-net track proximity costs into the obstacle map.

    Args:
        obstacles: The obstacle map to add costs to
        per_net_costs: Dict of net_id -> numpy array with columns [layer, gx, gy, cost]
        ghost_costs: optional ripped-corridor layer ghosts (C1-filtered by the
            caller), composed in the SAME pass. In max mode this is identical
            to the historical separate merge call (per-cell max commutes); the
            single pass exists for SUM mode, where "each source counted
            exactly once" is only enforceable with one composition point.

    Composition (soft-knobs sum experiment): per-cell MAX across sources by
    default; with KICAD_PROXIMITY_SUM, per-cell SUM across sources. Every
    array is one source (a routed net's corridor, a ripped net's ghost, the
    BGA/-1 field, congestion, fragility) and is already deduped internally
    (unique cells per array), so summing rows = summing sources -- a cell
    never inherits a source twice, and sampling density never inflates cost.
    The composed rows stay unique per cell, so the Rust max-insert write is a
    plain write on the cleared map every prepare starts from -- an accidental
    re-merge of the same composition re-writes identical values instead of
    doubling them (double-merge stays harmless in both modes).

    Double-merge guard: a net id appearing in BOTH dicts would count one
    net's copper twice in sum mode -- overlaps are dropped from the ghosts
    (the cache entry means the net has live copper again) with a warning.
    """
    if ghost_costs:
        overlap = per_net_costs.keys() & ghost_costs.keys()
        if overlap:
            print(f"    WARNING: ghost/cache key overlap dropped "
                  f"(double-merge guard): {sorted(overlap)}")
            ghost_costs = {k: v for k, v in ghost_costs.items()
                           if k not in overlap}
    # Concatenate all arrays for efficient batch processing
    arrays_to_merge = [arr for arr in per_net_costs.values() if len(arr) > 0]
    if ghost_costs:
        arrays_to_merge += [arr for arr in ghost_costs.values() if len(arr) > 0]
    if not arrays_to_merge:
        return

    # P5 (soft-knobs review): phase-3 and diff-pair paths re-prepare many
    # times between cache changes; memoize the concatenation keyed on the
    # cache's identity signature so unchanged caches skip the O(all-nets)
    # vstack. Exact-behavior: any entry add/replace changes the signature.
    # KEEPALIVE + BOUNDED. This memo is keyed on id(per_net_costs) and its
    # signature also mixes id(a) of the member arrays. Bare id() keys are unsafe
    # in a long-lived process (the GUI): once the keyed dict is garbage
    # collected, a NEW dict can be allocated at the SAME address, and if the
    # id-derived signature also collides, this returns ANOTHER run's merged
    # costs -- silently wrong track-proximity costs, i.e. different routing with
    # identical inputs. The CLI never sees it (fresh process per step).
    #
    # Two guards, both required:
    #   * store the keyed object (and the arrays) IN the entry, so nothing can
    #     be freed while its id is a live key -- ids can no longer be recycled
    #     out from under us;
    #   * bound the memo, so holding those references cannot grow without limit
    #     across a long GUI session.
    # Measured note: this was NOT the eth_tap step-11 divergence (disabling the
    # memo entirely left it unchanged), but the hazard is real and latent.
    _zone_rects = None
    if env_knobs.PROXIMITY_SUM_MODE == 'zoned' and config is not None:
        _zone_rects = proximity_max_zone_rects(config, GridCoord(config.grid_step))
    key = id(per_net_costs)
    # The member arrays' ids IN FULL, not a sum of their low bits: two
    # different compositions summing to the same value would replay the
    # wrong costs. The entry pins its arrays, so a live id is never reused.
    sig = (len(arrays_to_merge),
           sum(len(a) for a in arrays_to_merge),
           tuple(id(a) for a in arrays_to_merge),
           env_knobs.PROXIMITY_SUM_MODE,
           env_knobs.PROXIMITY_SOFTCAP_ALPHA,
           tuple(_zone_rects) if _zone_rects else None)
    memo = _MERGE_MEMO.get(key)
    if memo is not None and memo[0] == sig:
        all_costs = memo[1]
    else:
        all_costs = np.vstack(arrays_to_merge)
        if env_knobs.PROXIMITY_SUM:
            # Compose the SUM in numpy so the Rust write primitive stays the
            # idempotent max-insert: pack (layer, gx, gy) into one int64 key,
            # group, and sum each cell's per-source costs. Output rows are
            # unique per cell. In 'zoned' mode, cells inside a BGA escape
            # field take the per-cell MAX instead (mandatory approaches are
            # not an avoidable crowd).
            _OFF = 1 << 23  # grid coords are well inside +/-2^23
            packed = ((all_costs[:, 0].astype(np.int64) << 48)
                      | ((all_costs[:, 1].astype(np.int64) + _OFF) << 24)
                      | (all_costs[:, 2].astype(np.int64) + _OFF))
            uniq, inv = np.unique(packed, return_inverse=True)
            sums = np.bincount(inv, weights=all_costs[:, 3].astype(np.float64),
                               minlength=len(uniq))
            composed = np.empty((len(uniq), 4), dtype=np.int32)
            composed[:, 0] = (uniq >> 48).astype(np.int32)
            composed[:, 1] = ((uniq >> 24) & ((1 << 24) - 1)).astype(np.int32) - _OFF
            composed[:, 2] = (uniq & ((1 << 24) - 1)).astype(np.int32) - _OFF
            composed[:, 3] = np.minimum(sums, float(np.iinfo(np.int32).max)).astype(np.int32)
            if env_knobs.PROXIMITY_SUM_MODE == 'softcap':
                # softcap (#584): per-cell max + alpha*(sum - max) -- the
                # max-mode floor plus a fraction of the crowd pressure.
                maxs = np.zeros(len(uniq), dtype=np.int64)
                np.maximum.at(maxs, inv, all_costs[:, 3].astype(np.int64))
                a = env_knobs.PROXIMITY_SOFTCAP_ALPHA
                blended = maxs + a * (sums - maxs)
                composed[:, 3] = np.minimum(
                    blended, float(np.iinfo(np.int32).max)).astype(np.int32)
            if _zone_rects:
                maxs = np.zeros(len(uniq), dtype=np.int32)
                np.maximum.at(maxs, inv, all_costs[:, 3])
                in_zone = np.zeros(len(uniq), dtype=bool)
                for (x0, y0, x1, y1) in _zone_rects:
                    in_zone |= ((composed[:, 1] >= x0) & (composed[:, 1] <= x1)
                                & (composed[:, 2] >= y0) & (composed[:, 2] <= y1))
                composed[in_zone, 3] = maxs[in_zone]
            all_costs = composed
        # Bound first, then store WITH keepalive refs (see the note above):
        # per_net_costs and the member arrays are held so their ids cannot be
        # recycled while this entry is live.
        if (len(_MERGE_MEMO) >= _MERGE_MEMO_MAX
                or sum(e[1].nbytes for e in _MERGE_MEMO.values())
                + all_costs.nbytes > _MERGE_MEMO_BYTES_MAX):
            _MERGE_MEMO.clear()
        _MERGE_MEMO[key] = (sig, all_costs, per_net_costs, arrays_to_merge)
    obstacles.set_layer_proximity_batch(all_costs)


def add_cross_layer_tracks(obstacles: GridObstacleMap, pcb_data: PCBData,
                            config: GridRouteConfig, layer_map: Dict[str, int],
                            exclude_net_ids: Set[int] = None):
    """Populate cross-layer track positions for vertical alignment attraction.

    NET-AGNOSTIC by design (soft-knobs C4): the per-cell layer bitmask carries
    no net id, so vertical attraction pulls the route toward ANY other net's
    tracks on other layers (corridor stacking with strangers included). It
    cannot express "stack with my own group" -- that needs a per-group mask.

    Adds positions of all routed tracks (excluding specified nets) to the
    obstacle map's cross-layer lookup structure. This enables the router
    to give a cost bonus for routing on top of existing tracks on other layers.

    Args:
        obstacles: The obstacle map to populate
        pcb_data: PCB data containing segments
        config: Routing configuration with vertical_attraction_radius
        layer_map: Mapping of layer names to layer indices
        exclude_net_ids: Set of net IDs to exclude (typically the net being routed)
    """
    if config.vertical_attraction_radius <= 0 or config.vertical_attraction_cost <= 0:
        # Feature disabled. The cost gate matters (soft-knobs review P1): the
        # default radius is 1.0 with cost 0.0, so gating on radius alone
        # walked every board segment with one FFI call per 0.5mm sample, on
        # every prepare, to build a map the Rust lookup never reads
        # (early-out on bonus <= 0).
        return

    coord = GridCoord(config.grid_step)
    exclude_set = exclude_net_ids or set()

    # Sample every ~0.5mm along segments for reasonable density
    sample_interval = max(1, int(0.5 / config.grid_step))

    for seg in pcb_data.segments:
        if seg.net_id in exclude_set:
            continue

        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        _add_segment_cross_layer_track(obstacles, seg, coord, layer_idx, sample_interval)


def _add_segment_cross_layer_track(obstacles: GridObstacleMap, seg, coord: GridCoord,
                                    layer_idx: int, sample_interval: int):
    """Add a single segment's positions to the cross-layer track data."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    for step_count, (gx, gy) in enumerate(walk_line(gx1, gy1, gx2, gy2)):
        if step_count % sample_interval == 0:
            obstacles.add_cross_layer_track(gx, gy, layer_idx)


def compute_ripped_route_costs(saved_result: dict, config: GridRouteConfig,
                                layer_map: Dict[str, int]) -> Tuple[np.ndarray, List[Tuple[int, int]]]:
    """Compute avoidance costs for a ripped route's former segment/via locations.

    When a net is ripped up to route another net, we want to apply soft penalties
    to the ripped net's former corridor. This increases the chance the ripped net
    can be successfully rerouted later by keeping its path available.

    Args:
        saved_result: The saved routing result from the ripped net (contains new_segments, new_vias)
        config: Routing configuration with ripped_route_avoidance_radius and _cost
        layer_map: Mapping of layer names to layer indices

    Returns:
        (layer_costs, via_positions) where:
        - layer_costs: numpy array of [layer, gx, gy, cost] for segment proximity (layer-specific)
        - via_positions: list of (gx, gy) grid coordinates for vias
    """
    # Return empty results if feature is disabled
    if config.ripped_route_avoidance_cost <= 0 or config.ripped_route_avoidance_radius <= 0:
        return np.empty((0, 4), dtype=np.int32), []

    coord = GridCoord(config.grid_step)
    radius_grid = coord.to_grid_dist(config.ripped_route_avoidance_radius)
    cost_grid = config.cell_cost(config.ripped_route_avoidance_cost)

    # Sample every ~1mm along segments (not every grid step) for performance
    sample_interval = max(1, int(1.0 / config.grid_step))

    # Emit candidate rows per segment (points x offsets, broadcast), then
    # reduce to per-cell max. The classic implementation walked a
    # points-outer/offsets-inner Python loop into a max dict; this emits the
    # same rows in the same order and reduces in numpy, reproducing the dict
    # exactly: first-occurrence order (np.unique return_index) and max cost
    # per cell (exact int max) -- so the returned array is bit-identical.
    chunks = []
    new_segments = saved_result.get('new_segments', [])
    for seg in new_segments:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        # Width-aware ghost (#585 item 3): a ripped fat trunk vacates -- and
        # reserves -- a correspondingly wider corridor.
        offsets = _get_proximity_offsets_np(
            radius_grid, cost_grid,
            coord.to_grid_dist((seg.width or 0.0) / 2.0))
        if offsets.size == 0:
            continue

        # Walk along segment using walk_line, sampling every sample_interval points
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        pts = [p for i, p in enumerate(walk_line(gx1, gy1, gx2, gy2))
               if i % sample_interval == 0]
        p = np.asarray(pts, dtype=np.int64)                     # (S, 2)
        gx = p[:, 0:1] + offsets[:, 0]                          # (S, O)
        gy = p[:, 1:2] + offsets[:, 1]
        rows = np.empty((gx.size, 4), dtype=np.int64)
        rows[:, 0] = layer_idx
        rows[:, 1] = gx.ravel()
        rows[:, 2] = gy.ravel()
        rows[:, 3] = np.broadcast_to(offsets[:, 2], gx.shape).ravel()
        chunks.append(rows)

    # Reduce to one row per cell: max cost, rows ordered by first occurrence
    if chunks:
        allrows = np.vstack(chunks)
        cells = allrows[:, :3]
        mins = cells.min(axis=0)
        spans = cells.max(axis=0) - mins + 1
        key = (((cells[:, 0] - mins[0]) * spans[1] + (cells[:, 1] - mins[1]))
               * spans[2] + (cells[:, 2] - mins[2]))
        uniq, first_idx, inv = np.unique(key, return_index=True,
                                         return_inverse=True)
        maxcost = np.zeros(len(uniq), dtype=np.int64)
        np.maximum.at(maxcost, inv, allrows[:, 3])
        order = np.argsort(first_idx, kind='stable')
        sel = first_idx[order]
        layer_costs = np.empty((len(uniq), 4), dtype=np.int32)
        layer_costs[:, :3] = allrows[sel, :3]
        layer_costs[:, 3] = maxcost[order]
    else:
        layer_costs = np.empty((0, 4), dtype=np.int32)

    # Extract via positions (grid coords)
    via_positions = []
    new_vias = saved_result.get('new_vias', [])
    for via in new_vias:
        gx, gy = coord.to_grid(via.x, via.y)
        via_positions.append((gx, gy))

    return layer_costs, via_positions
