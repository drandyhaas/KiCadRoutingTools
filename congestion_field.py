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


# ======================= Congestion v2 (#424) =======================
# demand / capacity with owner exemption: congestion(bin) =
# |distinct UNROUTED nets with a terminal in bin| / free_area(bin).
# Demand decays by set arithmetic as nets complete (no copper rescan);
# a net pays NOTHING within KICAD_CONGESTION2_EXEMPT_R (default 1.0mm)
# of its own terminals -- the C5 disk-exemption pattern applied to the
# congestion family, so owners' last-mile approach is never taxed.
# Built once at batch start (build_congestion2 -> attach to config);
# stamped per-net at prepare (stamp_congestion2). Off unless
# KICAD_CONGESTION2_COST > 0.

def congestion2_knobs():
    def _f(name, dflt):
        try:
            return float(os.environ.get(name, str(dflt)) or dflt)
        except ValueError:
            return dflt
    return {
        'cost': _f('KICAD_CONGESTION2_COST', 0.0),
        'thresh': _f('KICAD_CONGESTION2_THRESHOLD', 0.5),   # nets per mm^2 free
        'bin': _f('KICAD_CONGESTION2_BIN', 1.0),
        'exempt_r': _f('KICAD_CONGESTION2_EXEMPT_R', 1.0),
        'ramp_top': _f('KICAD_CONGESTION2_RAMP_TOP', 2.0),  # ratio at full cost
    }


def build_congestion2(pcb_data, config, net_ids_to_route):
    """Precompute bins {(bx,by): (free_area_mm2, owners frozenset)} plus
    per-net terminal coords. Returns None when disabled."""
    k = congestion2_knobs()
    if k['cost'] <= 0:
        return None
    bin_mm = max(0.25, k['bin'])
    num_layers = max(1, len(config.layers))
    to_route = set(net_ids_to_route)

    copper = {}
    for s in pcb_data.segments:
        cx, cy = (s.start_x + s.end_x) / 2, (s.start_y + s.end_y) / 2
        b = (int(cx // bin_mm), int(cy // bin_mm))
        ln = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        copper[b] = copper.get(b, 0.0) + ln * s.width
    for v in pcb_data.vias:
        b = (int(v.x // bin_mm), int(v.y // bin_mm))
        copper[b] = copper.get(b, 0.0) + num_layers * math.pi * (v.size / 2) ** 2
    terminals = {}
    owners = {}
    for nid in to_route:
        pts = []
        for p in pcb_data.pads_by_net.get(nid, []):
            pts.append((p.global_x, p.global_y))
        for s in pcb_data.segments:
            if s.net_id == nid:
                pts.append((s.start_x, s.start_y))
                pts.append((s.end_x, s.end_y))
        for v in pcb_data.vias:
            if v.net_id == nid:
                pts.append((v.x, v.y))
        terminals[nid] = pts
        for (x, y) in pts:
            owners.setdefault((int(x // bin_mm), int(y // bin_mm)), set()).add(nid)
    for p in getattr(pcb_data, 'pads', []) or []:
        pass  # pad copper folded via per-net pads below
    for nid, plist in getattr(pcb_data, 'pads_by_net', {}).items():
        for p in plist:
            b = (int(p.global_x // bin_mm), int(p.global_y // bin_mm))
            copper[b] = copper.get(b, 0.0) + p.size_x * p.size_y
    bin_area_total = bin_mm * bin_mm * num_layers
    bins = {}
    for b, own in owners.items():
        used = copper.get(b, 0.0)
        free = max(bin_area_total * 0.05, bin_area_total - used)
        bins[b] = (free, frozenset(own))
    n_hot = sum(1 for b, (fa, ow) in bins.items()
                if len(ow) / fa > k['thresh'])
    print(f"Congestion2 field: {len(bins)} demand bins, {n_hot} above "
          f"threshold at batch start (cost={k['cost']}mm-equiv, "
          f"thresh={k['thresh']}/mm2, exempt_r={k['exempt_r']}mm)")
    return {'bins': bins, 'bin_mm': bin_mm, 'terminals': terminals, 'k': k}


def stamp_congestion2(obstacles, config, net_id, routed_net_ids):
    """Per-net prepare stamp: demand recomputed by set arithmetic, owner
    disks exempted, rows applied as layer-proximity costs."""
    d2 = getattr(config, '_congestion2', None)
    if d2 is None:
        return
    k = d2['k']
    bin_mm = d2['bin_mm']
    routed = set(routed_net_ids)
    own_pts = d2['terminals'].get(net_id, [])
    coord = GridCoord(config.grid_step)
    cell_cost = config.cell_cost(k['cost'])
    num_layers = len(config.layers)
    span = max(1, int(round(bin_mm / config.grid_step)))
    r2 = k['exempt_r'] ** 2
    rows = []
    for (bx, by), (free, own) in d2['bins'].items():
        demand = len(own - routed) - (1 if net_id in own else 0)
        ratio = demand / free
        if ratio <= k['thresh']:
            continue
        frac = min(1.0, (ratio - k['thresh']) / max(1e-9, k['ramp_top'] - k['thresh']))
        c = int(cell_cost * frac)
        if c <= 0:
            continue
        gx0, gy0 = coord.to_grid(bx * bin_mm, by * bin_mm)
        for dx in range(span):
            for dy in range(span):
                x_mm = (bx + (dx + 0.5) / span) * bin_mm
                y_mm = (by + (dy + 0.5) / span) * bin_mm
                if any((x_mm - px) ** 2 + (y_mm - py) ** 2 <= r2
                       for (px, py) in own_pts):
                    continue
                for li in range(num_layers):
                    rows.append((li, gx0 + dx, gy0 + dy, c))
    if rows:
        obstacles.set_layer_proximity_batch(np.array(rows, dtype=np.int32))
