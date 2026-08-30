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
import env_knobs
from typing import Dict

import numpy as np

from kicad_parser import PCBData
from routing_config import GridCoord, GridRouteConfig

# Reserved track_proximity_cache key (B1 uses -1 for BGA proximity).
CONGESTION_CACHE_KEY = -2


def congestion_cost_mm() -> float:
    try:
        return env_knobs.CONGESTION_COST
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
        bin_mm = env_knobs.CONGESTION_BIN
        threshold = env_knobs.CONGESTION_THRESHOLD
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
    # Read-once in env_knobs; copy so a caller mutating its dict cannot
    # poison the cached values.
    return dict(env_knobs.CONGESTION2)


def congestion_bins(pcb_data, net_ids, num_layers, bin_mm,
                    extra_demand_points=None, include_bins=None):
    """THE demand/capacity census: {(bx,by): (free_area_mm2, owners)} plus
    per-net terminal coords, and the BIN IT ACTUALLY USED.

    That third return value is not decoration: `bin_mm` is floored at 0.25,
    and a caller that builds coordinates from its own requested value puts
    the report on the wrong part of the board (check_pockets at --bin 0.1
    named windows ~39mm from the ones it measured). Ask, do not assume.

    One definition, two consumers (run-23): `build_congestion2` attaches it
    as an A* cost (env-gated), and `check_pockets.py` reports it PRE-ROUTE
    at the placement->routing handoff -- because every per-net gate
    (check_channels, check_reachability) passed run 23's board while two
    nets died in one bin whose demand/free-area no instrument ever printed.

    extra_demand_points (#589): optional {net_id: [(x_mm, y_mm), ...]} of
    predicted-corridor samples (the global plan's rough paths) folded into
    each net's terminal set -- the net then claims demand along its whole
    predicted path instead of only at its endpoints, and its own exemption
    disks follow the corridor (owner-exempt by construction).

    include_bins (#709): optional iterable of (bx, by) keys to report EVEN
    WHEN NOTHING DEMANDS THEM. The result was keyed off `owners`, which only
    gains a key where some net has a terminal -- so a window with nothing in
    it never entered the dict and `check_pockets` could not represent an
    empty region at all. Measured on esp_prog at 2mm: the lattice over
    `board_bounds` is 128 windows and the census returned 44, so 84 of them
    -- 66% -- were not merely un-ranked but absent.
    Such a bin comes back with `owners` empty and its free area computed the
    same way as everyone else's, which is what separates "empty" from "full
    of another net's copper".

    That free area is FLOORED AT 5% of the bin, here as everywhere, and the
    floor is not decoration at small bins: a segment is charged wholly to its
    midpoint bin, so `used` overshoots as the bin shrinks. Measured share of
    no-demand windows returning the floor rather than a real remainder --
    esp_prog 0% at 2mm, 16.9% at 1mm, 100% at 0.25mm; glasgow_revC 1.1% /
    1.3% / 78.0%. At the 0.25mm census floor the empty-vs-occupied
    distinction is therefore saturated away on most windows; ask it at the
    2mm default, which is what `check_pockets` uses.

    THE A* CONSUMER NEVER PASSES THIS. `build_congestion2` omits it, so the
    key sequence is `owners`' own iteration order, byte for byte as before,
    and the extra keys are APPENDED to `list(owners)` rather than unioned
    through a set. To be accurate about why: the np.array `congestion2_rows`
    builds from `bins.items()` reaches consumers that are order-INSENSITIVE
    (the Rust `set_layer_proximity_batch` max-inserts per cell; the SUM
    branch goes through np.unique/bincount), so a permutation would not
    change what the router prices cells with. What insertion order does
    decide is the tie-break in `check_pockets`' stable `sort(key=-ratio)`.
    Preserving it is cheap and keeps the function's contract exact either
    way. (Defensively, a demand-0 bin scores ratio 0 <= thresh and is skipped
    in `congestion2_rows` anyway, at any non-negative threshold.)
    """
    bin_mm = max(0.25, bin_mm)
    num_layers = max(1, num_layers)
    to_route = set(net_ids)

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
        if extra_demand_points:
            pts.extend(extra_demand_points.get(nid, ()))
        terminals[nid] = pts
        for (x, y) in pts:
            owners.setdefault((int(x // bin_mm), int(y // bin_mm)), set()).add(nid)
    for nid, plist in getattr(pcb_data, 'pads_by_net', {}).items():
        for p in plist:
            b = (int(p.global_x // bin_mm), int(p.global_y // bin_mm))
            copper[b] = copper.get(b, 0.0) + p.size_x * p.size_y
    bin_area_total = bin_mm * bin_mm * num_layers
    bins = {}
    if include_bins is None:
        extra = []
    else:
        # Materialise before the emptiness test: `not <numpy array>` raises,
        # and an iterator would be consumed by it.
        extra = list(include_bins)
        bad = next((b for b in extra
                    if not (isinstance(b, tuple) and len(b) == 2)), None)
        if bad is not None:
            # A bare (bx, by) instead of a LIST of them used to insert an int
            # key, which then failed three frames away in a consumer's
            # `for (bx, by), ... in bins.items()`. Fail where the mistake is.
            raise TypeError(
                'congestion_bins: include_bins takes an iterable of (bx, by) '
                'bin keys; got %r. A single key must be wrapped in a list.'
                % (bad,))
    keys = owners.keys() if not extra else (
        list(owners) + [b for b in extra if b not in owners])
    for b in keys:
        own = owners.get(b, frozenset())
        used = copper.get(b, 0.0)
        free = max(bin_area_total * 0.05, bin_area_total - used)
        bins[b] = (free, frozenset(own))
    return bins, terminals, bin_mm


def build_congestion2(pcb_data, config, net_ids_to_route,
                      extra_demand_points=None):
    """Precompute bins {(bx,by): (free_area_mm2, owners frozenset)} plus
    per-net terminal coords. Returns None when disabled.

    extra_demand_points (#589): see `congestion_bins` -- forwarded through."""
    k = congestion2_knobs()
    if k['cost'] <= 0:
        return None
    bin_mm = max(0.25, k['bin'])
    bins, terminals, bin_mm = congestion_bins(
        pcb_data, net_ids_to_route, len(config.layers), bin_mm,
        extra_demand_points=extra_demand_points)
    n_hot = sum(1 for b, (fa, ow) in bins.items()
                if len(ow) / fa > k['thresh'])
    print(f"Congestion2 field: {len(bins)} demand bins, {n_hot} above "
          f"threshold at batch start (cost={k['cost']}mm-equiv, "
          f"thresh={k['thresh']}/mm2, exempt_r={k['exempt_r']}mm)")
    return {'bins': bins, 'bin_mm': bin_mm, 'terminals': terminals, 'k': k}


def congestion2_rows(config, net_id, routed_net_ids):
    """Per-net congestion-v2 rows [layer, gx, gy, cost] (or None): demand
    recomputed by set arithmetic, owner disks exempted. Returned as a SOURCE
    for merge_track_proximity_costs' composition pass (#585 item 7) so sum
    mode SUMS congestion with the other fields instead of max-inserting it
    afterwards (in max mode the two are identical -- max commutes). Rows are
    unique per (layer, cell) by construction = within-source dedupe."""
    d2 = getattr(config, '_congestion2', None)
    if d2 is None:
        return None
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
        return np.array(rows, dtype=np.int32)
    return None


def stamp_congestion2(obstacles, config, net_id, routed_net_ids):
    """Legacy direct stamp (max-insert) -- kept for callers outside the
    composition pass; the routing builders feed congestion2_rows into
    merge_track_proximity_costs instead."""
    rows = congestion2_rows(config, net_id, routed_net_ids)
    if rows is not None:
        obstacles.set_layer_proximity_batch(rows)
