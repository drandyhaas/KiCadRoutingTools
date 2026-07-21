"""Congestion-aware soft routing costs (#424 Phase D).

Copper density across ALL layers, binned coarsely, becomes a soft per-cell
cost: routing a track through a regionally congested area costs more, and a
via there costs more still (the Rust via branch multiplies the summed
stub+layer proximity by via_proximity_cost, so a via in a hot cell
automatically pays ~10x the track penalty -- the soft-knobs review's C6
coupling working as intended here).

Delivery reuses the B1 mechanism: the field is computed ONCE at batch start
into an (N, 4) [layer, gx, gy, cost] array and registered under a reserved
key in track_proximity_cache; merge_track_proximity_costs then re-applies it
on every per-net prepare in every path (single-ended, diff pair, Phase 3).
Static v1: input copper (fanout, pre-routes, pads) defines the hot pockets;
in-run refresh is a later extension.

Experimental knobs via environment (no CLI flags until the lever proves
itself; promotion requires the full parity wiring per CLAUDE.md):
  KICAD_CONGESTION_COST      mm-equivalent per-cell cost at FULL density
                             (0 = disabled, the default)
  KICAD_CONGESTION_BIN       bin size in mm (default 1.0)
  KICAD_CONGESTION_THRESHOLD density [0..1] where the ramp starts (default 0.30)
"""
from __future__ import annotations

import math
import os
from typing import Dict

import numpy as np

from kicad_parser import PCBData
from routing_config import GridCoord, GridRouteConfig

# Reserved track_proximity_cache key (B1 uses -1 for BGA proximity).
CONGESTION_CACHE_KEY = -2


def congestion_cost_mm() -> float:
    try:
        return float(os.environ.get('KICAD_CONGESTION_COST', '0') or 0)
    except ValueError:
        return 0.0


def compute_congestion_cells(pcb_data: PCBData, config: GridRouteConfig,
                             num_layers: int) -> np.ndarray:
    """All-layer copper-density field as (N, 4) [layer, gx, gy, cost] rows.

    Density per bin = copper area across all copper layers / (bin area *
    num_layers), clamped to 1. Bins above the threshold emit a linear-ramp
    cost on EVERY routing layer (congestion is a regional property; a hot
    pocket is hot for everyone). Returns an empty array when disabled.
    """
    cost_mm = congestion_cost_mm()
    if cost_mm <= 0 or num_layers <= 0:
        return np.empty((0, 4), dtype=np.int32)
    try:
        bin_mm = float(os.environ.get('KICAD_CONGESTION_BIN', '1.0') or 1.0)
        threshold = float(os.environ.get('KICAD_CONGESTION_THRESHOLD', '0.30') or 0.30)
    except ValueError:
        bin_mm, threshold = 1.0, 0.30
    bin_mm = max(0.25, bin_mm)
    threshold = min(0.95, max(0.0, threshold))

    xs, ys = [], []
    for s in pcb_data.segments:
        xs.extend((s.start_x, s.end_x)); ys.extend((s.start_y, s.end_y))
    for v in pcb_data.vias:
        xs.append(v.x); ys.append(v.y)
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            xs.append(p.global_x); ys.append(p.global_y)
    if not xs:
        return np.empty((0, 4), dtype=np.int32)
    min_x, max_x = min(xs) - bin_mm, max(xs) + bin_mm
    min_y, max_y = min(ys) - bin_mm, max(ys) + bin_mm
    nx = max(1, int(math.ceil((max_x - min_x) / bin_mm)))
    ny = max(1, int(math.ceil((max_y - min_y) / bin_mm)))
    if nx * ny > 4_000_000:
        return np.empty((0, 4), dtype=np.int32)   # degenerate board extent
    area = np.zeros((nx, ny), dtype=np.float64)

    def bin_of(x: float, y: float):
        return (min(nx - 1, max(0, int((x - min_x) / bin_mm))),
                min(ny - 1, max(0, int((y - min_y) / bin_mm))))

    # Segments: copper area = length * width, deposited along the run.
    for s in pcb_data.segments:
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        length = math.hypot(dx, dy)
        w = s.width or 0.0
        n = max(1, int(length / (bin_mm * 0.5)))
        a_per = length * w / (n + 1)
        for i in range(n + 1):
            t = i / n if n else 0.0
            bx, by = bin_of(s.start_x + dx * t, s.start_y + dy * t)
            area[bx, by] += a_per
    # Vias: barrel copper on every layer.
    for v in pcb_data.vias:
        bx, by = bin_of(v.x, v.y)
        area[bx, by] += math.pi * (v.size / 2.0) ** 2 * num_layers
    # Pads: SMD on one layer, plated TH on all.
    for fp in pcb_data.footprints.values():
        for p in fp.pads:
            if p.pad_type == 'np_thru_hole':
                continue
            a = p.size_x * p.size_y
            if p.drill and p.drill > 0:
                a *= num_layers
            bx, by = bin_of(p.global_x, p.global_y)
            area[bx, by] += a

    density = np.clip(area / (bin_mm * bin_mm * num_layers), 0.0, 1.0)
    hot = np.argwhere(density > threshold)
    if hot.size == 0:
        return np.empty((0, 4), dtype=np.int32)

    coord = GridCoord(config.grid_step)
    cost_units_full = config.cell_cost(cost_mm)
    cells_per_bin = max(1, int(round(bin_mm / config.grid_step)))
    rows = []
    for bx, by in hot:
        ramp = (density[bx, by] - threshold) / max(1e-9, 1.0 - threshold)
        cost = int(cost_units_full * ramp)
        if cost <= 0:
            continue
        gx0, gy0 = coord.to_grid(min_x + bx * bin_mm, min_y + by * bin_mm)
        for li in range(num_layers):
            for ix in range(cells_per_bin):
                for iy in range(cells_per_bin):
                    rows.append((li, gx0 + ix, gy0 + iy, cost))
    if not rows:
        return np.empty((0, 4), dtype=np.int32)
    out = np.array(rows, dtype=np.int32)
    print(f"  Congestion field (#424 D): {len(hot)} hot bin(s) of {nx * ny} "
          f"({bin_mm}mm bins, threshold {threshold:.2f}), "
          f"{out.shape[0]} cost cells at up to {cost_mm}mm-equiv/cell "
          f"(vias pay via_proximity_cost x that)")
    return out


def register_congestion_field(pcb_data: PCBData, config: GridRouteConfig,
                              track_proximity_cache: Dict) -> None:
    """Compute and register the field under the reserved cache key (no-op
    when disabled)."""
    cells = compute_congestion_cells(pcb_data, config, len(config.layers))
    if len(cells):
        track_proximity_cache[CONGESTION_CACHE_KEY] = cells
