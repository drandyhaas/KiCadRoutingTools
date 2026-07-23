"""Plane-fragility soft routing costs (#424, planes-first chains).

A signal track only BISECTS a plane where the plane is NARROW: a crossing
through a wide pour region costs nothing (copper flows around it at fill
time), while a crossing at a neck severs the pour into islands the repair
step then has to stitch. Bisection is topological, but local narrowness is
a computable proxy: this module rasterizes each filled zone's outline at
the routing grid, measures every fill cell's distance to the zone boundary
(iterative erosion, no scipy), and stamps a soft per-cell cost on the
zone's layer that is negligible mid-pour and steep at necks.

Delivery reuses the congestion-field transport (#424 Phase D): the field is
computed once at batch start into (N, 4) [layer, gx, gy, cost] rows under a
reserved track_proximity_cache key; merge_track_proximity_costs re-applies
it on every per-net prepare in every path. Vias in fragile cells pay
via_proximity_cost x the cell cost through the Rust via branch (the C6
coupling), which is exactly right: a via is a permanent hole in the plane.

Static v1: the field reflects the pours as they exist at batch start (in a
planes-first chain, the just-poured planes). Necks created later by in-run
copper are invisible; a dynamic refresh is the #466 extension.

Experimental knobs via environment (no CLI flags until the lever proves
itself; promotion requires the full parity wiring per CLAUDE.md):
  KICAD_PLANE_FRAGILITY_COST   mm-equivalent per-cell cost at a zone
                               boundary cell (0 = disabled, the default)
  KICAD_PLANE_FRAGILITY_WIDTH  half-width in mm at which the ramp reaches
                               zero (default 2.0: cells deeper than this
                               into the pour cost nothing)
"""
from __future__ import annotations

import os
from typing import Dict

import numpy as np

from kicad_parser import PCBData
from routing_config import GridCoord, GridRouteConfig

# Reserved track_proximity_cache key (B1 uses -1, congestion -2).
PLANE_FRAGILITY_CACHE_KEY = -3


def fragility_cost_mm() -> float:
    try:
        return float(os.environ.get('KICAD_PLANE_FRAGILITY_COST', '0') or 0)
    except ValueError:
        return 0.0


def _rasterize_polygon(poly, coord, gx0, gy0, W, H) -> np.ndarray:
    """Boolean (H, W) fill mask of a polygon via even-odd crossing test,
    vectorized one scanline row at a time (grid rows are few thousand)."""
    xs = np.array([p[0] for p in poly])
    ys = np.array([p[1] for p in poly])
    mask = np.zeros((H, W), dtype=bool)
    col_x = np.array([coord.to_float(gx0 + i, 0)[0] for i in range(W)])
    x1s, y1s = xs, ys
    x2s, y2s = np.roll(xs, -1), np.roll(ys, -1)
    for j in range(H):
        y = coord.to_float(0, gy0 + j)[1]
        # edges straddling this scanline
        straddle = (y1s <= y) != (y2s <= y)
        if not straddle.any():
            continue
        xi = x1s[straddle] + (y - y1s[straddle]) / (y2s[straddle] - y1s[straddle]) \
            * (x2s[straddle] - x1s[straddle])
        # parity of crossings to the left of each column x
        mask[j] = (col_x[None, :] > xi[:, None]).sum(axis=0) % 2 == 1
    return mask


def compute_plane_fragility_cells(pcb_data: PCBData,
                                  config: GridRouteConfig) -> np.ndarray:
    """(N, 4) [layer, gx, gy, cost] rows for every filled-zone cell within
    the fragility half-width of its zone's boundary. Empty when disabled or
    the board has no filled zones on routing layers."""
    cost_mm = fragility_cost_mm()
    if cost_mm <= 0 or not pcb_data.zones:
        return np.empty((0, 4), dtype=np.int32)
    try:
        width_mm = float(os.environ.get('KICAD_PLANE_FRAGILITY_WIDTH', '2.0') or 2.0)
    except ValueError:
        width_mm = 2.0

    coord = GridCoord(config.grid_step)
    layer_index = {l: i for i, l in enumerate(config.layers)}
    depth = max(1, int(round(width_mm / config.grid_step)))
    rows = []

    for zone in pcb_data.zones:
        li = layer_index.get(zone.layer)
        if li is None or len(zone.polygon) < 3:
            continue
        xs = [p[0] for p in zone.polygon]
        ys = [p[1] for p in zone.polygon]
        gx0, gy0 = coord.to_grid(min(xs), min(ys))
        gx1, gy1 = coord.to_grid(max(xs), max(ys))
        W, H = int(gx1 - gx0 + 1), int(gy1 - gy0 + 1)
        if W <= 2 or H <= 2 or W * H > 40_000_000:
            continue  # degenerate or absurdly large raster
        mask = _rasterize_polygon(zone.polygon, coord, gx0, gy0, W, H)
        if not mask.any():
            continue
        # distance-to-boundary in grid steps by iterative 4-neighbour erosion,
        # capped at `depth` (cells deeper than the ramp are never emitted)
        dist = np.zeros((H, W), dtype=np.int16)
        cur = mask.copy()
        for d in range(1, depth + 1):
            inner = cur.copy()
            inner[1:, :] &= cur[:-1, :]
            inner[:-1, :] &= cur[1:, :]
            inner[:, 1:] &= cur[:, :-1]
            inner[:, :-1] &= cur[:, 1:]
            # boundary band peeled at this depth
            band = cur & ~inner
            dist[band] = d
            cur = inner
            if not cur.any():
                break
        emitted = mask & (dist > 0)
        if not emitted.any():
            continue
        jj, ii = np.nonzero(emitted)
        frag = 1.0 - (dist[jj, ii].astype(np.float64) - 1) / depth  # 1 at edge -> ~0 deep
        cell_cost = config.cell_cost(cost_mm)
        costs = np.maximum(1, (frag * cell_cost).astype(np.int32))
        zone_rows = np.column_stack([
            np.full(len(jj), li, dtype=np.int32),
            (ii + gx0).astype(np.int32),
            (jj + gy0).astype(np.int32),
            costs,
        ])
        rows.append(zone_rows)

    if not rows:
        return np.empty((0, 4), dtype=np.int32)
    out = np.vstack(rows)
    # overlapping zones on one layer: keep the max cost per cell
    order = np.lexsort((out[:, 3], out[:, 2], out[:, 1], out[:, 0]))
    out = out[order]
    keep = np.ones(len(out), dtype=bool)
    same = (np.diff(out[:, 0]) == 0) & (np.diff(out[:, 1]) == 0) & (np.diff(out[:, 2]) == 0)
    keep[:-1][same] = False  # lexsort put max cost last within a cell group
    return out[keep]


def register_plane_fragility(pcb_data: PCBData, config: GridRouteConfig,
                             track_proximity_cache: Dict) -> None:
    """Compute and register the field under the reserved cache key (no-op
    when disabled or no zones)."""
    cells = compute_plane_fragility_cells(pcb_data, config)
    if len(cells):
        track_proximity_cache[PLANE_FRAGILITY_CACHE_KEY] = cells
        n_zones = len({(z.layer, z.net_id) for z in pcb_data.zones})
        print(f"Plane fragility field: {len(cells)} cells near the edges of "
              f"{n_zones} pour(s) (KICAD_PLANE_FRAGILITY_COST="
              f"{fragility_cost_mm()}mm-equiv)")
