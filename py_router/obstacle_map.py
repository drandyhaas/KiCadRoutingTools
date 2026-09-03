"""
Obstacle map building functions for PCB routing.

Builds GridObstacleMap objects from PCB data, adding obstacles for segments,
vias, pads, BGA exclusion zones, and routed paths.
"""
from __future__ import annotations

from typing import List, Optional, Tuple, Dict, Set, Union
from dataclasses import dataclass, field
import env_knobs
import numpy as np
from collections import OrderedDict
import math

from kicad_parser import PCBData, Segment, Via, Pad, pad_drill_circles, pad_drill_capsule
from routing_config import GridRouteConfig, GridCoord
import routing_defaults as defaults
from routing_utils import build_layer_map, iter_pad_blocked_cells, pad_blocked_cells_array, \
    circle_offsets, segment_blocked_cells_array, segment_blocked_spans, GRID_TIE_EPS
from net_queries import expand_pad_layers

# Import Rust router
import sys
import os
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads

try:
    from grid_router import GridObstacleMap
except ImportError:
    # Will fail at runtime if not available
    GridObstacleMap = None


class _StaticStampProxy:
    """#422: transparent wrapper that redirects every blocked-cell/via ADD to the
    permanent static keep-out bitmap (``add_static_blocked_*``) instead of the
    refcount hashmaps. Used to build a BASE obstacle map whose cells are all
    permanent for the run (non-target, non-rippable copper + board geometry; base
    is never mutated after construction, and target/rippable nets live in the
    per-net caches added to a CLONE of base). Stamping straight to the bitmap
    avoids ever materialising the multi-million-entry dynamic hashmap for base,
    so its later working clone carries base as ~1 bit/cell. All other methods
    (cost maps, BGA zones, source/target, is_blocked, ...) pass through unchanged.
    Byte-identical: is_blocked/is_via_blocked OR the static bitmap, so a
    statically stamped cell blocks exactly as a refcount entry would."""
    __slots__ = ("_real",)

    def __init__(self, real):
        object.__setattr__(self, "_real", real)

    def unwrap(self):
        return self._real

    def add_blocked_cell(self, gx, gy, layer):
        self._real.add_static_blocked_cell(gx, gy, layer)

    def add_blocked_via(self, gx, gy):
        self._real.add_static_blocked_via(gx, gy)

    def add_blocked_cells_batch(self, cells):
        self._real.add_static_blocked_cells_batch(cells)

    def add_blocked_vias_batch(self, vias):
        self._real.add_static_blocked_vias_batch(vias)

    def add_blocked_cell_spans_batch(self, spans):
        self._real.add_static_blocked_cell_spans_batch(spans)

    def add_blocked_via_spans_batch(self, spans):
        self._real.add_static_blocked_via_spans_batch(spans)

    def __getattr__(self, name):
        return getattr(self._real, name)


def _obstacle_progress_reporter(progress_callback):
    """Throttled sub-phase progress for the base obstacle build (#556).

    Returns ``report(phase, i, n, force=False)``: forwards to
    ``progress_callback(i, n, "Obstacles: <phase>")`` at most ~4x/s (the GUI
    renders (i, n) as a percentage and the repeated calls keep its gauge
    alive), and prints a throttled log line at most every ~5s so a
    minutes-long build is visibly progressing in the logfile without
    spamming fast builds (sub-second builds print nothing). Reporting only:
    the map contents are untouched."""
    t0 = time.time()
    last_cb = [0.0]
    last_print = [t0]

    def report(phase: str, i: int, n: int, force: bool = False):
        now = time.time()
        if progress_callback and (force or now - last_cb[0] >= 0.25):
            last_cb[0] = now
            progress_callback(i, n, f"Obstacles: {phase}")
        if now - last_print[0] >= 5.0:
            last_print[0] = now
            print(f"  obstacles: {phase} {i}/{n} ({now - t0:.0f}s elapsed)",
                  flush=True)
    return report


def build_base_obstacle_map(pcb_data: PCBData, config: GridRouteConfig,
                            nets_to_route: List[int],
                            extra_clearance: float = 0.0,
                            net_clearances: dict = None,
                            static_base: bool = False,
                            progress_callback=None) -> GridObstacleMap:
    """Build base obstacle map with static obstacles (BGA zones, pads, pre-existing tracks/vias).

    Excludes all nets that will be routed (nets_to_route) - their stubs will be added
    per-net in the routing loop (excluding the current net being routed).

    Args:
        extra_clearance: Additional clearance to add for routing (e.g., for diff pair centerline routing)
        net_clearances: Optional dict mapping net_id to that net's net-class clearance (mm).
            KiCad's pairwise clearance between nets of different classes is max(classA, classB),
            so each pre-placed obstacle is priced at max(the routing-side clearance floor, the
            obstacle net's own class clearance). The floor maxes over the ROUTED nets only, so a
            foreign class cannot inflate it (that would over-block every routed net). An empty/
            absent map reproduces plain config.clearance behaviour exactly.
    """
    if net_clearances is None:
        net_clearances = {}
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)
    layer_map = build_layer_map(config.layers)
    nets_to_route_set = set(nets_to_route)

    # Cross-class clearance (KiCad semantics): the required spacing between two nets of
    # different net classes is max(classA, classB). effective_clearance is the routing-side
    # (classA) floor: the largest clearance among the nets being routed in THIS call. Obstacles
    # of OTHER classes must not inflate that floor (it would over-block every routed net), so the
    # max is taken over the ROUTED nets only; per obstacle we then raise it to that obstacle net's
    # own class clearance via _obstacle_clearance() below.
    _routed_clearances = [net_clearances[nid] for nid in nets_to_route_set if nid in net_clearances]
    max_net_clearance = max(_routed_clearances) if _routed_clearances else config.clearance
    effective_clearance = max(config.clearance, max_net_clearance)

    def _obstacle_clearance(net_id):
        # KiCad pairwise clearance: max(routing-side floor classA, this obstacle net's own class
        # clearance classB). A net not in the map falls back to config.clearance, so an EMPTY map
        # reproduces the prior behaviour exactly (inert until a per-net clearance map is supplied).
        return max(effective_clearance, net_clearances.get(net_id, config.clearance))

    _real_obstacles = GridObstacleMap(num_layers)
    # #422: when static_base, stamp every base blocked cell/via into the permanent
    # static bitmap (they are all immutable for the run) via a transparent proxy,
    # so the multi-million-entry dynamic hashmap is never materialised for base.
    # Guarded by hasattr so an older Rust binary (no static API) falls back to the
    # normal dynamic path. Byte-identical either way.
    obstacles = (_StaticStampProxy(_real_obstacles)
                 if static_base and hasattr(_real_obstacles, "add_static_blocked_cells_batch")
                 else _real_obstacles)

    # Set BGA proximity radius for is_in_bga_proximity() checks -- armed only
    # when the COST knob is on, so --bga-proximity-cost 0 is a real off-switch.
    # (This used to arm unconditionally; with the old pose-router binary via
    # cliff keyed on the radius alone, zeroing the cost silently left a 10x
    # via multiplier active across the whole 7mm ring for diff pairs.)
    bga_prox_radius_grid = (coord.to_grid_dist(config.bga_proximity_radius)
                            if config.bga_proximity_cost > 0 else 0)
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
    # BGA proximity costs are NOT stamped here anymore (soft-knobs review
    # B1): the base-map stub_proximity stamp was wiped by
    # prepare_obstacles_inplace before every single-ended net. They now
    # live in track_proximity_cache[BGA_PROXIMITY_CACHE_KEY] (registered
    # by the batch flows) and re-merge on every prepare in every path.

    # Net-tie corridors (Kelvin shunts / net-tie parts, see
    # _compute_net_tie_corridors): per tied net, the cells where the tied
    # net's copper may pass its PARTNER PAD. Only the partner PAD's stamp is
    # recorded and lifted -- the partner NET's trunk tracks are never exempt
    # (kicad-cli flags track-track contact between tied nets), and blocking
    # from sibling routes / third nets stays intact, which a query-time
    # overlay cannot express (measured: an overlay let two sense routes
    # short each other). An enclosed sense tab cannot be exited without pad
    # contact -- the human-routed originals carry the same one-per-shunt
    # kicad shorting_items, which grading treats as the accepted net-tie
    # class (like #408's edge-band items).
    _tie_corridors = _compute_net_tie_corridors(pcb_data, config, coord)
    _tie_partner_pad_ids = {pid for c in _tie_corridors.values()
                            for pid in c['partner_pad_ids']}
    _tie_recorded: List[tuple] = []  # ('pad', pad_id, cells array)

    # #556: sub-phase progress (GUI percentage + throttled log lines) so a
    # minutes-long build on a big board is distinguishable from a hang.
    _report = _obstacle_progress_reporter(progress_callback)

    # Add segments as obstacles (excluding nets we'll route - their stubs added per-net)
    # Use actual segment width for obstacle, and layer-specific width for routing track
    _seg_cell_batch: Dict[int, list] = {}
    _seg_via_batch: list = []
    _n_segs = len(pcb_data.segments)
    for _seg_i, seg in enumerate(pcb_data.segments):
        if (_seg_i & 511) == 0:
            _report("copper", _seg_i, _n_segs)
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            # Copper on a layer OUTSIDE config.layers (a 6/8-layer board routed
            # with a subset): tracks cannot go there, but a VIA spans the whole
            # stack and must still respect it -- without this, a rescue/retry
            # via lands straight on the unseen copper (butterstick DQ11: via on
            # In3 +3V3 tap copper, a real kicad clearance violation). Stamp the
            # via keep-out only.
            if seg.layer.endswith('.Cu'):
                seg_width = seg.width if getattr(seg, 'width', 0) > 0 else config.track_width
                # #498: the via meets this copper ON seg.layer, so a .kicad_dru
                # rule for that layer replaces the net/class value.
                seg_clearance = config.layer_clearance(seg.layer, _obstacle_clearance(seg.net_id))
                via_block_mm = config.via_size / 2 + seg_width / 2 + seg_clearance + extra_clearance
                # SPANS, like the main loop below: both producers feed
                # _seg_via_batch and it is concatenated as one array, so a
                # cell (N,2) here beside a span (N,3) there is a hard
                # ValueError at the flush. Only reachable on a board with
                # copper on a layer OUTSIDE config.layers (a 6/8-layer board
                # routed with a subset), which is why the signal/plane boards
                # never tripped it and test_dru_layer_clearance_e2e did.
                vias_arr = segment_blocked_spans(
                    seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                    via_block_mm, coord.grid_step)
                _seg_via_batch.append(vias_arr)
            continue
        # Compute expansion: routing-side reserve half-width (#156: nominal for
        # the single-ended engine -- impedance/power extra rides the per-net
        # track_margin; full layer width when reserve_layer_widths, diff engine)
        # + obstacle half-width + clearance
        reserve_width = config.route_reserve_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else config.get_track_width(seg.layer)
        # #498: a .kicad_dru layer rule REPLACES the pair clearance on seg.layer.
        seg_clearance = config.layer_clearance(seg.layer, _obstacle_clearance(seg.net_id))
        # A track-scoped DRU rule RAISES the seg-vs-seg requirement only (#735)
        # (the track capsule); via_block keeps the resolved value.
        trk_clearance = config.track_obstacle_clearance(seg.net_id, seg_clearance)
        expansion_mm = reserve_width / 2 + seg_width / 2 + trk_clearance + extra_clearance
        # For via blocking by segments: via half-size + segment half-width + clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + seg_clearance + extra_clearance
        # FFI batching (2026-08-14 profiling): one Rust call per segment was
        # 7.5M crossings / ~90s across a rescue-heavy step. Accumulate the
        # (memoized, read-only) cell arrays and stamp once per build below --
        # concatenation preserves the exact row multiset and order, and the
        # batch inserts process rows identically whether split or joined.
        cells_arr = segment_blocked_spans(
            seg.start_x, seg.start_y, seg.end_x, seg.end_y,
            expansion_mm, coord.grid_step)
        if len(cells_arr):
            _seg_cell_batch.setdefault(layer_idx, []).append(cells_arr)
        vias_arr = segment_blocked_spans(
            seg.start_x, seg.start_y, seg.end_x, seg.end_y,
            via_block_mm, coord.grid_step)
        if len(vias_arr):
            _seg_via_batch.append(vias_arr)

    # Flush the accumulated segment stamps: one Rust call per layer for the
    # track keep-outs, one for the via keep-outs.
    for _li, _arrs in sorted(_seg_cell_batch.items()):
        _sp = np.concatenate(_arrs) if len(_arrs) > 1 else _arrs[0]
        _rows = np.empty((len(_sp), 4), dtype=np.int32)
        _rows[:, :3] = _sp
        _rows[:, 3] = _li
        obstacles.add_blocked_cell_spans_batch(np.ascontiguousarray(_rows))
    if _seg_via_batch:
        # Every producer must emit the SAME form (spans, 3 columns). Assert it
        # rather than let np.concatenate raise a dimension error three frames
        # away from the producer that disagreed.
        assert all(a.shape[1] == 3 for a in _seg_via_batch), (
            "_seg_via_batch mixes cell and span rows: "
            + repr(sorted({a.shape[1] for a in _seg_via_batch})))
        _vall = (np.concatenate(_seg_via_batch)
                 if len(_seg_via_batch) > 1 else _seg_via_batch[0])
        obstacles.add_blocked_via_spans_batch(
            np.ascontiguousarray(_vall.astype(np.int32)))

    # Add vias as obstacles (excluding nets we'll route)
    _n_vias = len(pcb_data.vias)
    for _via_i, via in enumerate(pcb_data.vias):
        if (_via_i & 255) == 0:
            _report("vias", _via_i, _n_vias)
        if via.net_id in nets_to_route_set:
            continue
        # Compute expansion based on actual via size:
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
        # Cross-class: price this obstacle via's keepout at max(routing-side clearance, this via's
        # own net-class clearance). A pre-placed POWER_HI via (0.25) that a Default net (0.15) is
        # routed past must keep 0.25, not 0.15, or a new via lands (0.25-0.15) too close (the
        # via-to-via cross-class clearance under-model).
        via_clearance = _obstacle_clearance(via.net_id)
        # For track blocking by vias: via half-size + max routing track half-width + clearance
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, via_clearance, extra_clearance)
        # For via-to-via: via size + routing via size + clearance. #498: two via
        # barrels meet on EVERY stack layer, so the requirement is the stack max.
        via_via_mm = via_size / 2 + config.via_size / 2 + config.stack_clearance(via_clearance)
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
    # Priced per obstacle: max(routing-side clearance, the pad net's own class clearance)
    _n_pad_nets = len(pcb_data.pads_by_net)
    _pad_cell_sink: Dict[int, list] = {}
    _pad_via_sink: list = []
    for _pn_i, (net_id, pads) in enumerate(pcb_data.pads_by_net.items()):
        if (_pn_i & 63) == 0:
            _report("pads", _pn_i, _n_pad_nets)
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            if id(pad) in _tie_partner_pad_ids:
                _rec = _RecordingObstacles(obstacles)
                _add_pad_obstacle(_rec, pad, coord, layer_map, config, extra_clearance,
                                  clearance_override=_obstacle_clearance(net_id))
                if _rec.cell_batches:
                    _tie_recorded.append(('pad', id(pad), _rec.merged_cells()))
                continue
            _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance,
                              clearance_override=_obstacle_clearance(net_id),
                              cell_sink=_pad_cell_sink, via_sink=_pad_via_sink)
    # One Rust call per layer for every pad's track keep-out, one for the via
    # keep-outs -- the segment loop above has done this since 2026-08-14; the
    # pad loop was calling the batch API once PER PAD (measured on glasgow:
    # 1,593 batch entries for 1,136 pads, per base build, 629 builds/route).
    _flush_cell_sink(obstacles, _pad_cell_sink)
    _flush_via_sink(obstacles, _pad_via_sink)

    # Intersect each tied net's corridor with the recorded tie-copper stamps:
    # the per-net lift arrays are EXACT subsets of what the base build added
    # for that copper, so prepare/restore's balanced remove/re-add can never
    # desync a refcount (pads are never ripped; partner trunk copper is
    # pre-existing and excluded from rip candidates while its stamps are
    # lifted only during the tied net's own route).
    pcb_data._net_tie_lift = _assemble_net_tie_lifts(
        _tie_corridors, _tie_recorded, layer_map)
    # #667: the priced band = corridor cells OFF the own pad (the waived
    # own-pad approach stays free -- the gradient IS the steering).
    pcb_data._net_tie_price = {
        nid: sorted(e['cells'] - e.get('safe_cells', set()))
        for nid, e in _tie_corridors.items()}
    if len(nets_to_route_set) == 1:
        for _arr in pcb_data._net_tie_lift.get(next(iter(nets_to_route_set)), []):
            if len(_arr):
                obstacles.remove_blocked_cells_batch(_arr)

    # Add board edge clearance
    _report("board edge", 0, 0, force=True)
    add_board_edge_obstacles(obstacles, pcb_data, config, extra_clearance,
                             progress=_report)

    # Add user-drawn User-layer keepout polygons (issue #27) - block all copper layers
    _report("keepouts", 0, 0, force=True)
    add_user_keepout_obstacles(obstacles, pcb_data, config, coord, num_layers)

    # Block KiCad native keep-out rule areas (zone (keepout ...)), per-layer (PR #25)
    add_rule_area_keepout_obstacles(obstacles, pcb_data, config)

    # Add hole-to-hole clearance blocking for existing drills
    _report("drill holes", 0, 0, force=True)
    add_drill_hole_obstacles(obstacles, pcb_data, config, nets_to_route_set,
                             extra_clearance)

    # #422: return the real map (never the stamp proxy) so downstream clone/
    # cache/rip-up all operate on the genuine GridObstacleMap with its normal
    # dynamic add/remove methods.
    return _real_obstacles


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


def _scanline_inside_rows(px_axis, py_axis, x1, y1, x2, y2):
    """Even-odd inside test for a (rows x cols) cell grid, one scanline per
    row: O(rows x edges) instead of the dense (cells x edges) ray cast (#546
    -- crkbd's 12M-cell x ~500-edge maps put 97s of a 360s profile window in
    the broadcast kernel). Bit-identical to _points_inside_polygon at the
    same cell centres: per edge the x-intercept is the same expression on the
    same operands, and a point is inside iff the number of qualifying
    intercepts STRICTLY greater than its px is odd -- counted here with
    searchsorted on the sorted intercepts (side='right' counts xi <= px, so
    ties are excluded exactly like the kernel's strict `px < xi`).
    px_axis/py_axis must be ascending. Returns (ny, nx) bool."""
    nx = px_axis.size
    ny = py_axis.size
    out = np.zeros((ny, nx), dtype=bool)
    if nx == 0 or x1.size == 0:
        return out
    safe_dy = np.where(y2 - y1 == 0, 1.0, y2 - y1)
    for j in range(ny):
        py = py_axis[j]
        cond = (y1 > py) != (y2 > py)
        if not cond.any():
            continue
        xi = ((x2 - x1) * (py - y1) / safe_dy + x1)[cond]
        xi.sort()
        gt = xi.size - np.searchsorted(xi, px_axis, side='right')
        out[j] = (gt & 1).astype(bool)
    return out


def _banded_edge_distance_rows(px_axis, py_axis, x1, y1, x2, y2, threshold):
    """Min distance from each cell centre to the nearest polygon edge,
    computed only where it can be < `threshold` (#546): each edge stamps
    exact distances into the cells of its bbox expanded by threshold;
    everywhere else the result stays +inf. For a cell whose TRUE min distance
    is < threshold the value is bit-identical to _points_edge_distance -- the
    minimizing edge's expanded bbox necessarily contains the cell (the
    closest point lies on the segment, so |px - closest_x| <= dist <
    threshold bounds px inside bbox_x +- threshold, same for y), the
    per-element arithmetic is the same expression on the same operands, and
    min over the containing-edge subset equals min over all edges. For a cell
    at >= threshold the result is some value >= threshold (possibly inf), so
    every consumer comparison against a clearance <= threshold -- `< clr`
    and `>= clr` alike -- answers exactly like the dense kernel. Cost is
    O(perimeter-band cells) instead of (cells x edges).
    px_axis/py_axis must be ascending. Returns (ny, nx) float64."""
    ny, nx = py_axis.size, px_axis.size
    out_sq = np.full((ny, nx), np.inf)
    if nx == 0 or ny == 0 or x1.size == 0:
        return out_sq
    for k in range(x1.size):
        ex1, ey1, ex2, ey2 = x1[k], y1[k], x2[k], y2[k]
        i0 = int(np.searchsorted(px_axis, min(ex1, ex2) - threshold, side='left'))
        i1 = int(np.searchsorted(px_axis, max(ex1, ex2) + threshold, side='right'))
        j0 = int(np.searchsorted(py_axis, min(ey1, ey2) - threshold, side='left'))
        j1 = int(np.searchsorted(py_axis, max(ey1, ey2) + threshold, side='right'))
        if i0 >= i1 or j0 >= j1:
            continue
        pxw = px_axis[i0:i1][np.newaxis, :]
        pyw = py_axis[j0:j1][:, np.newaxis]
        dx_e = ex2 - ex1
        dy_e = ey2 - ey1
        seg_len_sq = dx_e * dx_e + dy_e * dy_e
        if seg_len_sq < 1e-10:
            dist_sq = (pxw - ex1) ** 2 + (pyw - ey1) ** 2
        else:
            t = np.clip(((pxw - ex1) * dx_e + (pyw - ey1) * dy_e) / seg_len_sq,
                        0.0, 1.0)
            closest_x = ex1 + t * dx_e
            closest_y = ey1 + t * dy_e
            dist_sq = (pxw - closest_x) ** 2 + (pyw - closest_y) ** 2
        np.minimum(out_sq[j0:j1, i0:i1], dist_sq, out=out_sq[j0:j1, i0:i1])
    return np.sqrt(out_sq)


# Exact-key memo for polygon rasterization (2026-08-14 orangecrab
# profiling: 123k calls / 52s -- the 830 rescue/escalation map builds
# re-rasterize every keepout/cutout polygon each time, and those polygons
# are net-independent, so escalation's board-global builds hit across ALL
# nets and reconcile laps). Keyed on the exact polygon bytes + grid +
# margin + clip (absolute-frame math, so translation canonicalization is
# NOT bit-safe -- the #493 class); a hit returns the identical arrays by
# construction, shared READ-ONLY (all consumers build masks / index; none
# mutate -- audited). LRU-evicted on a byte budget (a board-ring keepout
# at a fine grid is millions of cells).
#
# #818: the cache stores the BOX (gx_lo, gy_lo, nx, ny) plus the two
# per-cell arrays, NOT the per-cell gx/gy meshgrid. The meshgrid is a pure
# function of the box -- 8 of the old 17 bytes/cell were recomputable, and
# a glasgow route measured 15,318 misses of which 15,299 (99.9%) were
# evictions against only 11,122 distinct keys, i.e. 37.7% more
# rasterization than the keyspace requires. Consumers want the coordinates
# of the MASKED cells only, which `_box_masked_cells` derives by divmod --
# so the meshgrid is never materialized on the hot paths at all.
_POLY_RASTER_CACHE: "OrderedDict[tuple, tuple]" = OrderedDict()
_POLY_RASTER_BYTES = 0
_POLY_CELL_BYTES = 9    # inside bool + edist float64 (gx/gy derived from the box)
_POLY_EMPTY_BOX = (0, 0, 0, 0, None, None)


def _poly_raster_byte_budget() -> int:
    # 30% of the shared KICAD_RASTER_CACHE_MB budget (#815 rebalance: the three
    # raster memos sum to 100% -- capsule spans 65%, capsule cells 5%, here
    # 30%). Cut from 40% because this cache cannot use the memory: #818
    # measured it 58x oversubscribed, and DOUBLING it moved the hit rate 59.6%
    # -> 60.6% for +100 MB of RSS. The span memo it was given to was pinned at
    # exactly 100% of its budget for an entire route.
    # LRU-evicted, never wholesale-cleared.
    return int(env_knobs.RASTER_CACHE_MB * 0.30 * 1e6)


def _box_masked_cells(gx_lo: int, gy_lo: int, nx: int, mask):
    """Grid coordinates of the True cells of a box raster's `mask`.

    #818: the box raster is stored WITHOUT its gx/gy meshgrid, because the
    meshgrid is `gx_lo + idx % nx`, `gy_lo + idx // nx` under the flattened
    (ny, nx) layout `_rasterize_polygon_box` produces (gx fastest). Deriving
    only the masked cells is both exact and cheaper than materializing the
    full grid and then indexing it with the mask.
    """
    idx = np.flatnonzero(mask)
    gy_off, gx_off = np.divmod(idx, nx)
    return (gx_off.astype(np.int32) + np.int32(gx_lo),
            gy_off.astype(np.int32) + np.int32(gy_lo))


def _box_full_cells(gx_lo: int, gy_lo: int, nx: int, ny: int):
    """The full (gx_flat, gy_flat) meshgrid of a box raster (compat path)."""
    gx_grid, gy_grid = np.meshgrid(
        np.arange(gx_lo, gx_lo + nx, dtype=np.int32),
        np.arange(gy_lo, gy_lo + ny, dtype=np.int32))
    return gx_grid.ravel(), gy_grid.ravel()


def _rasterize_polygon_box(poly_points, coord: GridCoord, margin: float, clip_bounds=None):
    """Rasterize a closed polygon over its grid bounding box (expanded by `margin` mm).

    Returns ``(gx_lo, gy_lo, nx, ny, inside, edge_dist)`` -- the grid-space
    bounding box of the raster plus two flattened per-cell arrays in (ny, nx)
    row-major order with **gx fastest**, so cell ``i`` is
    ``(gx_lo + i % nx, gy_lo + i // nx)``. Use :func:`_box_masked_cells` to
    recover the coordinates of a masked subset.

    ``inside``   : bool, cell centre inside the polygon (even-odd ray cast)
    ``edge_dist``: float, mm distance from the cell centre to the nearest edge

    ``clip_bounds`` (min_x, min_y, max_x, max_y) restricts the rasterized region
    to the obstacle map's actual extent. Without it, a large polygon -- e.g. a
    whole-board ring keep-out -- rasterizes the entire board on EVERY build, even
    a tiny local-window build, dominating repair_planes and the
    via-in-pad unblock (the map only covers the window, so cells outside it are
    never blocked anyway). Clipping is a pure optimisation: no cell inside the map
    changes. A full-board build passes the board bounds, so nothing is clipped.

    Shared geometry kernel for the polygon obstacle passes (board cutouts, KiCad
    keep-out rule areas, and user-drawn keepout zones). Returns
    ``_POLY_EMPTY_BOX`` (``inside is None``) if the polygon is degenerate
    (< 3 points) or the bounding box is empty. Callers threshold ``edge_dist``
    by their own clearance to decide which cells to block.
    """
    global _POLY_RASTER_BYTES
    if len(poly_points) < 3:
        return _POLY_EMPTY_BOX
    poly = np.array(poly_points, dtype=np.float64)
    _mkey = (poly.tobytes(), coord.grid_step, margin,
             tuple(clip_bounds) if clip_bounds is not None else None)
    _mhit = _POLY_RASTER_CACHE.get(_mkey)
    if _mhit is not None:
        _POLY_RASTER_CACHE.move_to_end(_mkey)
        return _mhit
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
            _POLY_RASTER_CACHE[_mkey] = _POLY_EMPTY_BOX
            return _POLY_EMPTY_BOX  # polygon doesn't overlap the map
    gx_lo, gy_lo = coord.to_grid(cmin_x, cmin_y)
    gx_hi, gy_hi = coord.to_grid(cmax_x, cmax_y)
    gx_range = np.arange(gx_lo, gx_hi + 1, dtype=np.int32)
    gy_range = np.arange(gy_lo, gy_hi + 1, dtype=np.int32)
    if gx_range.size == 0 or gy_range.size == 0:
        _POLY_RASTER_CACHE[_mkey] = _POLY_EMPTY_BOX
        return _POLY_EMPTY_BOX

    # #546: row-scanline inside test + threshold-banded edge distance instead
    # of the dense (cells x edges) kernels. edge_dist is exact wherever the
    # true distance is < margin + grid_step and >= that bound (up to +inf)
    # elsewhere; every caller thresholds it against a clearance <= margin, so
    # the blocked sets are identical to the dense kernels'.
    px_axis = gx_range.astype(np.float64) * coord.grid_step
    py_axis = gy_range.astype(np.float64) * coord.grid_step
    inside = _scanline_inside_rows(px_axis, py_axis, x1, y1, x2, y2).ravel()
    edge_dist = _banded_edge_distance_rows(
        px_axis, py_axis, x1, y1, x2, y2, margin + coord.grid_step).ravel()

    result = (int(gx_lo), int(gy_lo), int(gx_range.size), int(gy_range.size),
              inside, edge_dist)
    inside.setflags(write=False)
    edge_dist.setflags(write=False)
    _POLY_RASTER_CACHE[_mkey] = result
    _POLY_RASTER_BYTES += inside.size * _POLY_CELL_BYTES
    budget = _poly_raster_byte_budget()
    while _POLY_RASTER_BYTES > budget and _POLY_RASTER_CACHE:
        _, old_res = _POLY_RASTER_CACHE.popitem(last=False)
        if old_res[4] is not None:
            _POLY_RASTER_BYTES -= old_res[4].size * _POLY_CELL_BYTES
    return result


def _rasterize_polygon(poly_points, coord: GridCoord, margin: float, clip_bounds=None):
    """Compat wrapper: :func:`_rasterize_polygon_box` with the gx/gy meshgrid
    materialized, as ``(gx_flat, gy_flat, inside, edge_dist)`` (or four ``None``
    for a degenerate/empty raster).

    #818: prefer the box form on hot paths -- this rebuilds 8 bytes/cell of
    coordinates the box already implies, and every in-tree consumer only needs
    the coordinates of its MASKED cells (see :func:`_box_masked_cells`).
    """
    gx_lo, gy_lo, nx, ny, inside, edge_dist = _rasterize_polygon_box(
        poly_points, coord, margin, clip_bounds=clip_bounds)
    if inside is None:
        return None, None, None, None
    gx_flat, gy_flat = _box_full_cells(gx_lo, gy_lo, nx, ny)
    return gx_flat, gy_flat, inside, edge_dist



def _block_cells_on_layers(obstacles: GridObstacleMap, gx_flat, gy_flat, mask, layer_idxs,
                           static: bool = False):
    """Block the masked (gx, gy) cells on each layer in `layer_idxs`.

    #422: with static=True the cells go into the permanent static keep-out bitmap
    (1 bit/cell) instead of the refcount hashmap -- used for board geometry
    (cutouts) that never changes during routing.
    """
    if not mask.any():
        return
    cells = np.column_stack([gx_flat[mask], gy_flat[mask]])
    add = (obstacles.add_static_blocked_cells_batch if static
           else obstacles.add_blocked_cells_batch)
    for li in layer_idxs:
        layer_col = np.full((cells.shape[0], 1), li, dtype=np.int32)
        add(np.hstack([cells, layer_col]))


def _block_cells_sel(obstacles: GridObstacleMap, gx_sel, gy_sel, layer_idxs,
                     static: bool = False):
    """:func:`_block_cells_on_layers` for cells already reduced by their mask
    (#818: the box raster derives only the masked coordinates)."""
    if gx_sel.size == 0:
        return
    cells = np.column_stack([gx_sel, gy_sel])
    add = (obstacles.add_static_blocked_cells_batch if static
           else obstacles.add_blocked_cells_batch)
    for li in layer_idxs:
        layer_col = np.full((cells.shape[0], 1), li, dtype=np.int32)
        add(np.hstack([cells, layer_col]))


def _polygon_grid_cells(points_mm, coord: GridCoord):
    """Return the set of (gx, gy) grid cells whose centre is inside the polygon."""
    gx_lo, gy_lo, nx, ny, inside, _ = _rasterize_polygon_box(
        points_mm, coord, margin=0.0)
    if inside is None:
        return set()
    gx_sel, gy_sel = _box_masked_cells(gx_lo, gy_lo, nx, inside)
    return set(zip(gx_sel.tolist(), gy_sel.tolist()))


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
    # call (the dominant cost of repair_planes and the via-in-pad
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
            # #369 A5: expand composite copper tokens -- KiCad writes rule
            # areas with (layers "*.Cu") or (layers F&B.Cu), and resolving
            # only literal layer names left layer_idxs empty, which DISABLED
            # track blocking below: the router routed straight through the
            # user's all-copper keep-out.
            resolved = set()
            for ln in ko_layers:
                if ln in layer_map:
                    resolved.add(layer_map[ln])
                elif ln == '*.Cu':
                    resolved.update(layer_map.values())
                elif ln in ('F&B.Cu', 'F&B'):
                    resolved.update(layer_map[l] for l in ('F.Cu', 'B.Cu')
                                    if l in layer_map)
            layer_idxs = sorted(resolved)
        else:
            layer_idxs = list(range(len(layer_list)))
        if block_tracks and not layer_idxs:
            block_tracks = False
        if not (block_tracks or block_vias):
            continue

        # Bounding box gets a clearance margin so we also catch cells just outside
        # the polygon whose track/via copper would still intrude past the boundary.
        margin = max(track_clear, via_clear) + coord.grid_step
        o_gx_lo, o_gy_lo, o_nx, o_ny, inside, edge_dist = _rasterize_polygon_box(
            poly, coord, margin, clip_bounds=clip)
        if inside is None:
            continue

        # Boundary cells resolve OPEN (GRID_TIE_EPS): a cell centre sitting
        # EXACTLY at the clearance is decided by float rounding otherwise, and
        # since edge_dist is measured in absolute board coordinates the answer
        # varied with the polygon's POSITION (measured: 8 different cell sets
        # for one rectangle at clearance 0.2 on a 0.1 grid).
        track_mask = inside | (edge_dist < track_clear - GRID_TIE_EPS)
        via_mask = inside | (edge_dist < via_clear - GRID_TIE_EPS)

        # Holes: the keep-out is the outer polygon MINUS its holes (a ring).
        # Cells deep inside a hole (>= the relevant clearance from the hole
        # edge) are routable; cells in the ring, or in the hole but within
        # clearance of the ring copper, stay blocked (issue #95).
        # #556: evaluated on the HOLE's own raster (scanline inside + banded
        # edge distance, like every other polygon pass) and mapped back into
        # the outer meshgrid arithmetically. The previous dense
        # (_points_inside_polygon + _points_edge_distance over the whole OUTER
        # array) was the last cells-x-edges kernel in the build: a whole-board
        # ring keep-out paid outer_cells x hole_edges PER HOLE. Byte-identical:
        # a cell outside the hole raster has h_inside False (no mask change),
        # and inside it the banded distance is exact below the threshold while
        # values at/above it still satisfy `>= clear` exactly as the true
        # distance would (threshold strictly exceeds both clearances).
        holes = ko.get('holes') or []
        if holes:
            for hole in holes:
                if len(hole) < 3:
                    continue
                h_margin = max(track_clear, via_clear)
                h_gx_lo, h_gy_lo, h_nx, h_ny, h_inside, h_edge = _rasterize_polygon_box(
                    hole, coord, h_margin, clip_bounds=clip)
                if h_inside is None:
                    continue
                hgx, hgy = _box_masked_cells(h_gx_lo, h_gy_lo, h_nx, h_inside)
                sel = ((hgx >= o_gx_lo) & (hgx < o_gx_lo + o_nx)
                       & (hgy >= o_gy_lo) & (hgy < o_gy_lo + o_ny))
                if not sel.any():
                    continue
                idx = ((hgy[sel].astype(np.int64) - o_gy_lo) * o_nx
                       + (hgx[sel].astype(np.int64) - o_gx_lo))
                h_sel_edge = h_edge[h_inside][sel]
                # Complement of the blocking test above -- shift by the same
                # epsilon or a ring cell could be both blocked and unblocked.
                track_mask[idx[h_sel_edge >= track_clear - GRID_TIE_EPS]] = False
                via_mask[idx[h_sel_edge >= via_clear - GRID_TIE_EPS]] = False

        if block_tracks and track_mask.any():
            t_gx, t_gy = _box_masked_cells(o_gx_lo, o_gy_lo, o_nx, track_mask)
            _block_cells_sel(obstacles, t_gx, t_gy, layer_idxs)
        if block_vias and via_mask.any():
            v_gx, v_gy = _box_masked_cells(o_gx_lo, o_gy_lo, o_nx, via_mask)
            obstacles.add_blocked_vias_batch(np.column_stack([v_gx, v_gy]))


def add_board_edge_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                              config: GridRouteConfig, extra_clearance: float = 0.0,
                              layers: Optional[List[str]] = None,
                              track_width: Optional[float] = None,
                              progress=None):
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
    # Add track half-width to clearance (tracks need to stay away from edge).
    # #447: the edge band must use the width the map is STAMPED at -- build_base_obstacles
    # (repair_planes plane connections) stamps every obstacle at
    # min_track_width, wider than config.track_width, and routes the narrowest
    # connection with track_margin=0; a config.track_width band lets that wider copper
    # graze the outline sub-fab (crkbd/dilemma). Callers pass the stamped width here.
    _edge_tw = track_width if track_width is not None else config.track_width
    track_edge_clearance = edge_clearance + _edge_tw / 2 + extra_clearance
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
    # #441: a pad whose own copper reaches into the edge-clearance band (an edge
    # connector / edge-mounted part -- core1106_cam's U1, whose pads sit 0.05mm
    # from the edge) must stay landable, or every net on it becomes unroutable.
    # Exempt from the TRACK keep-out ONLY the pad's own copper cells, on the pad's
    # OWN layer, that lie ON the board (never a cell past the actual edge). This is
    # self-net-scoped: a foreign net on those cells would short the pad, so it
    # cannot ride the hole, and no edge-parallel corridor is exposed (the reach-
    # exemption disk that let chocofi's row0 hug the edge is gone).
    layer_exempt = _edge_band_pad_layer_exemption(
        pcb_data, coord, edge_clearance, layer_list, board_outlines, board_bounds)

    if board_outlines:
        # Use polygon-based blocking for non-rectangular boards
        _add_polygon_edge_obstacles(obstacles, board_outlines, coord, num_layers,
                                     track_edge_clearance, via_edge_clearance,
                                     gmin_x, gmin_y, gmax_x, gmax_y, track_expand, via_expand,
                                     layer_exempt=layer_exempt, progress=progress)
    else:
        # Use simple rectangular blocking
        _add_rectangular_edge_obstacles(obstacles, coord, num_layers,
                                         gmin_x, gmin_y, gmax_x, gmax_y,
                                         track_expand, via_expand,
                                         layer_exempt=layer_exempt)

    # Block areas inside board cutouts (e.g., connector/switch openings)
    board_cutouts = pcb_data.board_info.board_cutouts
    if board_cutouts:
        for cutout in board_cutouts:
            if len(cutout) >= 3:
                _add_cutout_obstacles(obstacles, cutout, coord, num_layers,
                                      track_edge_clearance, via_edge_clearance)

    # Milled inner contours (#505): Edge.Cuts boundaries that are neither holes
    # nor outer rings -- a pad-containing inner outline reclassified by
    # drop_pad_containing_cutouts. Only a BAND around the boundary is blocked,
    # never the interior: these enclose pads by definition (crkbd's encloses the
    # whole board), so a cutout-style `inside` fill would block everything.
    for contour in (getattr(pcb_data.board_info, 'board_edge_contours', None) or []):
        if len(contour) >= 3:
            _add_edge_contour_obstacles(obstacles, contour, coord, num_layers,
                                        track_edge_clearance, via_edge_clearance)


def _edge_band_pad_layer_exemption(pcb_data, coord: GridCoord, edge_clearance: float,
                                   layer_list, board_outlines, bounds):
    """Per-layer cells of an in-band pad's OWN copper that lie ON the board --
    exempt from the board-edge TRACK keep-out (#441). A pad whose copper reaches
    into the edge-clearance band (an edge connector / edge-mounted part) must stay
    landable, but ONLY the pad's own copper on ITS layer is opened, and never a
    cell past the actual board edge -- so a foreign net cannot ride the hole (it
    would short the pad) and no edge-parallel corridor is exposed. Returns
    ``{layer_idx: set(packed gx<<32 | gy&0xFFFFFFFF)}``; ``{}`` when no pad reaches
    the band (the common edge-honoring board)."""
    min_x, min_y, max_x, max_y = bounds
    layer_index = {name: i for i, name in enumerate(layer_list)}
    step = coord.grid_step
    rings = [np.asarray(o, dtype=np.float64) for o in (board_outlines or []) if len(o) >= 3]
    gmnx, gmny = coord.to_grid(min_x, min_y)
    gmxx, gmxy = coord.to_grid(max_x, max_y)
    exempt: Dict[int, set] = {}
    for fp in pcb_data.footprints.values():
        for pad in fp.pads:
            if getattr(pad, 'pad_type', '') == 'np_thru_hole':
                continue
            hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
            half = max(hx, hy)
            # Coarse pre-filter: only pads whose copper can reach the band matter.
            if not (pad.global_x - half < min_x + edge_clearance or
                    pad.global_x + half > max_x - edge_clearance or
                    pad.global_y - half < min_y + edge_clearance or
                    pad.global_y + half > max_y - edge_clearance):
                continue
            pad_layers = [layer_index[L] for L in expand_pad_layers(pad.layers, layer_list)
                          if L in layer_index]
            if not pad_layers:
                continue
            pgx, pgy = coord.to_grid(pad.global_x, pad.global_y)
            cells = pad_blocked_cells_array(
                pgx, pgy, hx, hy, 0.0, step,
                off_x=pad.global_x - pgx * step, off_y=pad.global_y - pgy * step,
                rotation_deg=getattr(pad, 'rect_rotation', 0.0) or 0.0)
            if len(cells) == 0:
                continue
            gx = cells[:, 0].astype(np.int64)
            gy = cells[:, 1].astype(np.int64)
            # ON-board only: a cell past the actual edge stays hard-blocked.
            if rings:
                onb = None
                px, py = gx * step, gy * step
                for r in rings:
                    ins = _points_inside_polygon(px, py, r[:, 0], r[:, 1],
                                                 np.roll(r[:, 0], -1), np.roll(r[:, 1], -1))
                    onb = ins if onb is None else (onb | ins)
            else:
                onb = (gx >= gmnx) & (gx <= gmxx) & (gy >= gmny) & (gy <= gmxy)
            keys = ((gx[onb] << 32) + (gy[onb] & 0xFFFFFFFF))
            if len(keys) == 0:
                continue
            kset = set(int(k) for k in keys)
            for li in pad_layers:
                exempt.setdefault(li, set()).update(kset)
    return exempt


# #423: max grid cells rasterized at once in the banded board-geometry keep-out
# passes (edge/off-board and cutouts). Bounds the transient numpy temporaries
# (px/py float64 + inside/edge_dist + where-index arrays) to roughly this many
# cells per band instead of the whole board, which on a large sparse board at a
# fine grid was ~2 GB. ~2M cells ≈ a few hundred MB of transient arrays per band;
# the whole bbox is covered in ceil(area/this) bands with byte-identical results.
_EDGE_BAND_CELLS = 2_000_000


def _rasterize_polygon_banded(poly_points, coord: GridCoord, margin: float,
                              clip_bounds=None, band_cells: int = _EDGE_BAND_CELLS):
    """Row-banded generator variant of :func:`_rasterize_polygon` (#423).

    Yields the same ``(gx_flat, gy_flat, inside, edge_dist)`` tuples, but one
    horizontal row-band at a time, so a whole-board-scale polygon never
    materialises tens of millions of cells of numpy temporaries at once. crkbd
    carries a 214x97 mm board-region cutout that rasterized to ~33 M cells
    (~1.2 GB) at grid_step 0.025 -- the dominant build peak after the edge pass.
    Byte-identical: the bands partition the bbox rows so every cell appears in
    exactly one band, with the same per-cell inside/edge_dist as the unbanded
    kernel. Yields nothing for a degenerate polygon or an empty/clipped bbox.
    """
    if len(poly_points) < 3:
        return
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
            return
    gx_lo, gy_lo = coord.to_grid(cmin_x, cmin_y)
    gx_hi, gy_hi = coord.to_grid(cmax_x, cmax_y)
    gx_range = np.arange(gx_lo, gx_hi + 1, dtype=np.int32)
    nx = int(gx_range.size)
    if nx == 0 or gy_hi < gy_lo:
        return
    rows_per_band = max(1, band_cells // nx)
    px_axis = gx_range.astype(np.float64) * coord.grid_step
    for band_lo in range(gy_lo, gy_hi + 1, rows_per_band):
        band_hi = min(band_lo + rows_per_band - 1, gy_hi)
        gy_range = np.arange(band_lo, band_hi + 1, dtype=np.int32)
        gx_grid, gy_grid = np.meshgrid(gx_range, gy_range)
        gx_flat = gx_grid.ravel()
        gy_flat = gy_grid.ravel()
        # #546: scanline + threshold-banded kernels (see _rasterize_polygon);
        # both consumers threshold edge_dist against clearances < margin.
        py_axis = gy_range.astype(np.float64) * coord.grid_step
        inside = _scanline_inside_rows(px_axis, py_axis, x1, y1, x2, y2).ravel()
        edge_dist = _banded_edge_distance_rows(
            px_axis, py_axis, x1, y1, x2, y2,
            margin + coord.grid_step).ravel()
        yield gx_flat, gy_flat, inside, edge_dist


def _add_cutout_obstacles(obstacles: GridObstacleMap, cutout: List[Tuple[float, float]],
                          coord: GridCoord, num_layers: int,
                          track_edge_clearance: float, via_edge_clearance: float):
    """Block tracks and vias inside a board cutout and within clearance of its edges.

    Cells whose centre is inside the cutout polygon are blocked on all layers; cells
    just outside whose track/via copper would intrude past the edge are blocked too.
    #423: rasterized in row bands so a whole-board-scale cutout polygon does not
    spike memory (byte-identical to the unbanded stamp).
    """
    margin = max(track_edge_clearance, via_edge_clearance) + coord.grid_step
    for gx_flat, gy_flat, inside, edge_dist in _rasterize_polygon_banded(cutout, coord, margin):
        ring_track = (~inside) & (edge_dist < track_edge_clearance - GRID_TIE_EPS)
        ring_via = (~inside) & (edge_dist < via_edge_clearance - GRID_TIE_EPS)
        # #422: cutouts are permanent board geometry -> static keep-out bitmap.
        _block_cells_on_layers(obstacles, gx_flat, gy_flat,
                               inside | ring_track, range(num_layers), static=True)
        via_mask = inside | ring_via
        if via_mask.any():
            obstacles.add_static_blocked_vias_batch(
                np.column_stack([gx_flat[via_mask], gy_flat[via_mask]]))


def _add_edge_contour_obstacles(obstacles: GridObstacleMap, contour: List[Tuple[float, float]],
                                coord: GridCoord, num_layers: int,
                                track_edge_clearance: float, via_edge_clearance: float):
    """Block a clearance BAND on both sides of a milled inner contour (#505).

    Unlike :func:`_add_cutout_obstacles` this never blocks the polygon interior.
    These contours are pad-containing by construction -- that is exactly why
    drop_pad_containing_cutouts refused to treat them as holes -- so filling the
    inside would blank the board (crkbd's contour encloses all 870+ pads). What
    the geometry does demand is edge clearance: KiCad mills along the line and
    grades copper against it, so a track may not come within the edge clearance
    from EITHER side. `_points_edge_distance` is unsigned, so one distance test
    covers both sides.
    """
    margin = max(track_edge_clearance, via_edge_clearance) + coord.grid_step
    for gx_flat, gy_flat, _inside, edge_dist in _rasterize_polygon_banded(
            contour, coord, margin):
        band_track = edge_dist < track_edge_clearance - GRID_TIE_EPS
        band_via = edge_dist < via_edge_clearance - GRID_TIE_EPS
        # #422: board geometry is permanent -> static keep-out bitmap.
        _block_cells_on_layers(obstacles, gx_flat, gy_flat, band_track,
                               range(num_layers), static=True)
        if band_via.any():
            obstacles.add_static_blocked_vias_batch(
                np.column_stack([gx_flat[band_via], gy_flat[band_via]]))


def _add_rectangular_edge_obstacles(obstacles: GridObstacleMap, coord: GridCoord, num_layers: int,
                                     gmin_x: int, gmin_y: int, gmax_x: int, gmax_y: int,
                                     track_expand: int, via_expand: int, layer_exempt=None):
    """Add obstacles for simple rectangular board outline.

    The via keep-out band (via_expand) reaches FURTHER inboard than the track
    band (track_expand) because a via is wider than a track. Each edge sweep must
    therefore cover max(track_expand, via_expand) cells in from the edge and block
    track layers / vias per-cell: sweeping only track_expand (the old behaviour)
    never visited the inner via-only band, so route.py dropped vias up to
    (via_expand - track_expand) cells past the via keep-out, intruding into the
    board-edge clearance (#170). The track keep-out and the parallel corner
    handoff to the left/right sweeps are unchanged. ``layer_exempt`` (#441) keeps
    an in-band pad's own copper cells landable on its own layer.
    """
    _le = layer_exempt or {}
    edge_expand = max(track_expand, via_expand)
    grid_margin = edge_expand + 5

    # Sweep item 8 (#625 follow-up): the four edge sweeps called
    # add_static_blocked_cell/add_static_blocked_via once PER CELL per layer
    # (~300k+ FFI calls per build on a bbox-outline board). The bands are
    # rectangles, so each sweep is one meshgrid + the same per-axis masks,
    # exempt keys filtered with packed-int np.isin (the exact key packing the
    # scalar used, including the & 0xFFFFFFFF wrap), and two _batch calls.
    # The static bitmaps are idempotent sets, so equal cell sets = equal
    # state, regardless of add order.
    _le_packed = {li: np.sort(np.fromiter(s, dtype=np.int64, count=len(s)))
                  for li, s in _le.items() if s}

    def _sweep(gx_lo, gx_hi, gy_lo, gy_hi, track_mask_fn, via_mask_fn, by_x):
        xs = np.arange(gx_lo, gx_hi + 1, dtype=np.int64)
        ys = np.arange(gy_lo, gy_hi + 1, dtype=np.int64)
        if not len(xs) or not len(ys):
            return
        axis = xs if by_x else ys
        tmask = track_mask_fn(axis)
        vmask = via_mask_fn(axis)
        GX, GY = np.meshgrid(xs, ys, indexing='ij')
        gxf, gyf = GX.ravel(), GY.ravel()
        cell_t = np.repeat(tmask, len(ys)) if by_x else np.tile(tmask, len(xs))
        cell_v = np.repeat(vmask, len(ys)) if by_x else np.tile(vmask, len(xs))
        if cell_t.any():
            tx, ty = gxf[cell_t], gyf[cell_t]
            keys = (tx << 32) + (ty & 0xFFFFFFFF)
            for layer_idx in range(num_layers):
                ex = _le_packed.get(layer_idx)
                keep = ~np.isin(keys, ex) if ex is not None else slice(None)
                kx, ky = tx[keep], ty[keep]
                if len(kx):
                    obstacles.add_static_blocked_cells_batch(np.column_stack(
                        [kx, ky, np.full(len(kx), layer_idx, dtype=np.int64)]
                    ).astype(np.int32))
        if cell_v.any():
            obstacles.add_static_blocked_vias_batch(np.column_stack(
                [gxf[cell_v], gyf[cell_v]]).astype(np.int32))

    # Left edge (full height, so it also covers the via band at both left
    # corners); right edge (full height); top/bottom middle spans.
    _sweep(gmin_x - grid_margin, gmin_x + edge_expand,
           gmin_y - grid_margin, gmax_y + grid_margin,
           lambda gx: gx <= gmin_x + track_expand,
           lambda gx: gx < gmin_x + via_expand, True)
    _sweep(gmax_x - edge_expand, gmax_x + grid_margin,
           gmin_y - grid_margin, gmax_y + grid_margin,
           lambda gx: gx >= gmax_x - track_expand,
           lambda gx: gx > gmax_x - via_expand, True)
    _sweep(gmin_x + track_expand + 1, gmax_x - track_expand - 1,
           gmin_y - grid_margin, gmin_y + edge_expand,
           lambda gy: gy <= gmin_y + track_expand,
           lambda gy: gy < gmin_y + via_expand, False)
    _sweep(gmin_x + track_expand + 1, gmax_x - track_expand - 1,
           gmax_y - edge_expand, gmax_y + grid_margin,
           lambda gy: gy >= gmax_y - track_expand,
           lambda gy: gy > gmax_y - via_expand, False)


def _add_polygon_edge_obstacles(obstacles: GridObstacleMap, polygons,
                                 coord: GridCoord, num_layers: int,
                                 track_edge_clearance: float, via_edge_clearance: float,
                                 gmin_x: int, gmin_y: int, gmax_x: int, gmax_y: int,
                                 track_expand: int, via_expand: int, layer_exempt=None,
                                 progress=None):
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

    gx_lo = gmin_x - grid_margin
    gx_hi = gmax_x + grid_margin
    gy_lo = gmin_y - grid_margin
    gy_hi = gmax_y + grid_margin
    gx_range = np.arange(gx_lo, gx_hi + 1, dtype=np.int32)
    nx = int(gx_range.size)
    if nx == 0 or gy_hi < gy_lo:
        return

    # Per-ring edge arrays (computed once). on-board = inside ANY ring; edge
    # distance is the min over all rings' edges concatenated.
    ring_edges = []
    for polygon in polygons:
        poly_arr = np.array(polygon, dtype=np.float64)
        ring_edges.append((poly_arr[:, 0], poly_arr[:, 1],
                           np.roll(poly_arr[:, 0], -1), np.roll(poly_arr[:, 1], -1)))
    x1, y1, x2, y2 = (np.concatenate([e[i] for e in ring_edges]) for i in range(4))

    # #423: process the bounding box in horizontal ROW BANDS instead of one
    # whole-board array. On a large sparse board (split keyboard) the off-board
    # keep-out spans nearly the whole bbox, so the full-board (px, py, inside,
    # where-index) temporaries were ~2 GB of numpy at grid_step 0.025 -- the
    # dominant peak of the entire route (build_base_obstacle_map alone hit
    # ~1.8 GB). Banding bounds the transient set to ~_EDGE_BAND_CELLS cells and
    # is byte-identical: each cell falls in exactly one band and is stamped with
    # the same rule (outside -> all layers + via; inside within clearance ->
    # track/via), just in row-band order.
    rows_per_band = max(1, _EDGE_BAND_CELLS // nx)
    px_axis = gx_range.astype(np.float64) * coord.grid_step
    # #546: threshold for the banded distance kernel -- strictly above both
    # consumer clearances so their `<` comparisons are exact.
    _dist_thr = max(track_edge_clearance, via_edge_clearance) + coord.grid_step
    # #556: per-band progress (the edge pass is the long pole on complex
    # outlines; the band index gives the GUI a real percentage).
    _n_bands = (gy_hi - gy_lo) // rows_per_band + 1 if gy_hi >= gy_lo else 0
    for _band_i, band_lo in enumerate(range(gy_lo, gy_hi + 1, rows_per_band)):
        if progress is not None:
            progress("board edge", _band_i, _n_bands)
        band_hi = min(band_lo + rows_per_band - 1, gy_hi)
        gy_range = np.arange(band_lo, band_hi + 1, dtype=np.int32)
        gx_grid, gy_grid = np.meshgrid(gx_range, gy_range)  # (nband, nx)
        gx_flat = gx_grid.ravel()
        gy_flat = gy_grid.ravel()
        py_axis = gy_range.astype(np.float64) * coord.grid_step

        inside = None
        for (rx1, ry1, rx2, ry2) in ring_edges:
            ins = _scanline_inside_rows(px_axis, py_axis,
                                        rx1, ry1, rx2, ry2).ravel()
            inside = ins if inside is None else (inside | ins)

        outside_idx = np.where(~inside)[0]
        inside_idx = np.where(inside)[0]

        # Block all outside (off-board) points (all layers + vias). #422: board
        # geometry is PERMANENT, so stamp it into the static keep-out bitmap
        # (1 bit/cell). (A band-only variant that skips the deep off-board was
        # measured net-negative -- computing each cell's edge distance to decide
        # band membership cost more than the stamping it saved, and it perturbed
        # a few routes -- so the full off-board fill is kept.)
        if outside_idx.size > 0:
            out_cells = np.column_stack([gx_flat[outside_idx], gy_flat[outside_idx]])
            for layer_idx in range(num_layers):
                layer_col = np.full((out_cells.shape[0], 1), layer_idx, dtype=np.int32)
                obstacles.add_static_blocked_cells_batch(np.hstack([out_cells, layer_col]))
            obstacles.add_static_blocked_vias_batch(out_cells)

        # Compute edge distances for inside points (#546: banded kernel --
        # exact below _dist_thr, which caps both consumer clearances)
        if inside_idx.size > 0:
            min_dist = _banded_edge_distance_rows(
                px_axis, py_axis, x1, y1, x2, y2,
                _dist_thr).ravel()[inside_idx]
            in_gx = gx_flat[inside_idx]
            in_gy = gy_flat[inside_idx]

            # Block tracks if too close to edge
            track_mask = min_dist < track_edge_clearance
            if np.any(track_mask):
                track_gx = in_gx[track_mask]
                track_gy = in_gy[track_mask]
                track_cells = np.column_stack([track_gx, track_gy])
                # #441: keys for per-layer pad-own-copper exemption (see
                # _edge_band_pad_layer_exemption). Only layers with an exempt set
                # pay the filter; every other layer stamps the full band.
                _le = layer_exempt or {}
                track_keys = ((track_gx.astype(np.int64) << 32)
                              + (track_gy.astype(np.int64) & 0xFFFFFFFF)) if _le else None
                for layer_idx in range(num_layers):
                    ex = _le.get(layer_idx)
                    if ex:
                        keep = ~np.isin(track_keys, np.fromiter(ex, dtype=np.int64, count=len(ex)))
                        cells_L = track_cells[keep]
                    else:
                        cells_L = track_cells
                    if cells_L.shape[0] == 0:
                        continue
                    layer_col = np.full((cells_L.shape[0], 1), layer_idx, dtype=np.int32)
                    obstacles.add_static_blocked_cells_batch(np.hstack([cells_L, layer_col]))

            # Block vias if too close to edge
            via_mask = min_dist < via_edge_clearance
            if np.any(via_mask):
                via_gx = in_gx[via_mask]
                via_gy = in_gy[via_mask]
                obstacles.add_static_blocked_vias_batch(np.column_stack([via_gx, via_gy]))


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
    chunks = []
    for hx, hy, drill_dia in drill_holes:
        # Required center-to-center distance = drill/2 + via_drill/2 + clearance.
        required_dist = drill_dia / 2.0 + via_drill / 2.0 + hole_to_hole_clearance
        # Boundary cells resolve OPEN (GRID_TIE_EPS): required_dist is often an
        # exact multiple of grid_step (drill 0.4 + via 0.2 + clearance 0.3 = 0.6
        # = 6 x a 0.1 grid), which put cell centres exactly on the disc edge and
        # made the blocked set depend on the hole's board position (measured: 8
        # different sets for one hole).
        req_sq = (required_dist - GRID_TIE_EPS) ** 2
        gx, gy = coord.to_grid(hx, hy)
        expand = coord.to_grid_dist_safe(required_dist) + 1  # ceil + 1-cell bbox margin
        # #546: vectorized disc (was a Python double loop per drill). Same
        # cell-center-in-mm test, same per-drill cell multiset.
        exs = np.arange(gx - expand, gx + expand + 1, dtype=np.int64)
        eys = np.arange(gy - expand, gy + expand + 1, dtype=np.int64)
        dx = exs.astype(np.float64) * grid_step - hx
        dy = eys.astype(np.float64) * grid_step - hy
        mask = (dx * dx)[:, np.newaxis] + (dy * dy)[np.newaxis, :] < req_sq
        ii, jj = np.nonzero(mask)
        if ii.size:
            chunks.append(np.column_stack([exs[ii], eys[jj]]).astype(np.int32))
    if chunks:
        obstacles.add_blocked_vias_batch(np.vstack(chunks))


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
    chunks = []
    for hx, hy, drill_dia in drill_holes:
        # A track centerline must stay this far from the real drill centre.
        required_dist = drill_dia / 2.0 + track_width / 2.0 + clearance
        req_sq = (required_dist - GRID_TIE_EPS) ** 2   # tie -> OPEN, see above
        gx, gy = coord.to_grid(hx, hy)
        expand = coord.to_grid_dist_safe(required_dist) + 1  # ceil + 1-cell bbox margin
        # #546: vectorized disc (was a Python double loop per drill). Same
        # cell-center-in-mm test, same per-drill cell multiset.
        exs = np.arange(gx - expand, gx + expand + 1, dtype=np.int64)
        eys = np.arange(gy - expand, gy + expand + 1, dtype=np.int64)
        dx = exs.astype(np.float64) * grid_step - hx
        dy = eys.astype(np.float64) * grid_step - hy
        mask = (dx * dx)[:, np.newaxis] + (dy * dy)[np.newaxis, :] < req_sq
        ii, jj = np.nonzero(mask)
        if ii.size:
            chunks.append(np.column_stack([exs[ii], eys[jj]]).astype(np.int32))
    if not chunks:
        return
    arr = np.vstack(chunks)
    for li in layer_idxs:
        layer_col = np.full((arr.shape[0], 1), li, dtype=np.int32)
        obstacles.add_blocked_cells_batch(np.hstack([arr, layer_col]))


def override_pad_hole_track_cells(pcb_data: PCBData, track_width: float,
                                  base_clearance: float, grid_step: float,
                                  extra_clearance: float = 0.0,
                                  include_plated: bool = True) -> np.ndarray:
    """Grid cells a routed TRACK must stay out of around the drill holes of
    pads carrying a per-pad clearance OVERRIDE (``pad.local_clearance``) --
    the #326 residual hole_clearance class (ghoul).

    KiCad's hole-clearance rule is NET-INDEPENDENT and honors the pad's
    clearance override, so copper of ANY net -- including the pad's own --
    must stay ``local_clearance`` off the hole wall unless it actually lands
    on (connects to) the pad's copper. Two classes need cells beyond the
    normal keep-outs:

    * NPTH pads (no copper): the standard NPTH keep-out enforces only the
      fab floor (NPTH_TO_TRACK_CLEARANCE, 0.20); an override above that
      (ghoul's zero-ring switch holes carry 0.3) widens the required disc.
    * PTH pads (``include_plated``): own-net pads are skipped by the
      copper-pad blockers entirely (exclude_net_id), so nothing keeps a
      same-net track off the hole of a zero-annular-ring pad. Stamp the
      override radius, but leave cells whose center lies inside the pad
      copper (bounding-disc approximation, ``max(size)/2``) free so a
      direct landing on the pad stays routable -- KiCad exempts copper
      that touches the pad.

    Pads without an override (or whose override is already covered by the
    existing keep-outs) contribute nothing, so behavior for normal pads is
    unchanged. Returns an (N, 2) int32 array of (gx, gy) cells; callers
    stamp them on every routing layer -- a drill goes through the board.
    """
    coord = GridCoord(grid_step)
    npth_floor = max(base_clearance, defaults.NPTH_TO_TRACK_CLEARANCE)
    cells = []
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.drill <= 0:
                continue
            lc = getattr(pad, 'local_clearance', 0.0) or 0.0
            has_copper = _pad_has_copper(pad)
            # Below this, the existing keep-outs (NPTH floor stamp / the
            # copper-pad blocker, whose disc always reaches past the hole
            # since size >= drill) already cover the requirement.
            already_covered = base_clearance if has_copper else npth_floor
            if lc <= already_covered:
                continue
            if has_copper and not include_plated:
                continue
            exempt_r_sq = None
            if has_copper:
                exempt_r = max(pad.size_x, pad.size_y) / 2.0
                # Note the SIGN: this radius exempts cells from being blocked,
                # so resolving its boundary tie OPEN means growing it, not
                # shrinking it like every other epsilon here. Both edges of this
                # predicate therefore move the same way -- toward fewer blocked
                # cells -- which is what "tie -> OPEN" means for the outcome.
                exempt_r_sq = (exempt_r + GRID_TIE_EPS) ** 2
            for hx, hy, drill_dia in pad_drill_circles(pad):
                required = drill_dia / 2.0 + track_width / 2.0 + lc + extra_clearance
                req_sq = (required - GRID_TIE_EPS) ** 2   # tie -> OPEN, see above
                gx, gy = coord.to_grid(hx, hy)
                expand = coord.to_grid_dist_safe(required) + 1
                for ex in range(-expand, expand + 1):
                    cx = (gx + ex) * grid_step
                    for ey in range(-expand, expand + 1):
                        cy = (gy + ey) * grid_step
                        if (cx - hx) ** 2 + (cy - hy) ** 2 >= req_sq:
                            continue
                        if exempt_r_sq is not None and \
                           (cx - pad.global_x) ** 2 + (cy - pad.global_y) ** 2 < exempt_r_sq:
                            continue
                        cells.append((gx + ex, gy + ey))
    if not cells:
        return np.zeros((0, 2), dtype=np.int32)
    return np.array(cells, dtype=np.int32)


def block_track_cells_near_override_pad_holes(obstacles: GridObstacleMap,
                                              pcb_data: PCBData, track_width: float,
                                              base_clearance: float, grid_step: float,
                                              layer_idxs, extra_clearance: float = 0.0,
                                              include_plated: bool = True):
    """Stamp override_pad_hole_track_cells on each layer in ``layer_idxs``."""
    arr = override_pad_hole_track_cells(pcb_data, track_width, base_clearance,
                                        grid_step, extra_clearance, include_plated)
    if arr.shape[0] == 0 or not layer_idxs:
        return
    for li in layer_idxs:
        layer_col = np.full((arr.shape[0], 1), li, dtype=np.int32)
        obstacles.add_blocked_cells_batch(np.hstack([arr, layer_col]))


_HOLE_CLR_CACHE = {}          # board path -> declared min_hole_clearance (mm)
_HOLE_CLR_ANNOUNCED = set()
_HOLE_CLR_ORIGIN = set()      # paths whose floor came from fab_floor_origin,
                              # i.e. a later step's writeback had relaxed the
                              # live rule below what the board declared


def resolve_hole_clearance(pcb_data: PCBData, config,
                           pcb_file: str = None) -> float:
    """The copper-to-HOLE floor this board declares, in mm (0.0 = none).

    Resolved ENGINE-SIDE off ``PCBData.source_path`` (the #498 mechanism built
    for exactly this), so both fronts inherit it with no wiring. An explicit
    ``config.hole_clearance`` wins and stops the read.

    ``pcb_file`` overrides the parsed ``source_path`` when a caller holds the
    authoritative path -- the #498 rule the rest of the toolchain follows,
    "the CALLER's path when it has one, else ``PCBData.source_path``". Added
    for #761: a board staged into a temp dir and parsed from there carries a
    ``source_path`` that is not the board the caller means, and
    ``grade_pad_legality``/``QuenchState`` already thread ``pcb_file`` to
    ``PadClearanceModel.for_board`` for exactly that reason. Default ``None``
    keeps every existing caller bit-identical.

    TWO sources, and the larger wins: ``design_settings.rules`` (what the
    project declares NOW) and ``kicad_routing_tools.fab_floor_origin`` (what it
    declared before this chain touched it). The second is needed because the
    first is not durable -- each writeback clamps the live rule down to the
    clearance that step routed at, so a chain ERASES the author's declaration
    after step 1. Measured on a tigard pour+route chain declaring 0.25: the
    pour left ``rules`` at 0.15 and the route step read 0.15, i.e. below the
    0.20 fab floor, and stopped honouring the board without saying anything.
    Reading the origin too makes the declaration survive the whole chain, which
    is the only reading under which "the board declares 0.25" means what a user
    would expect. See :func:`fix_kicad_drc_settings.declared_fab_floor`.

    WHO ACTUALLY INHERITS IT, precisely -- everything routed through
    ``add_drill_hole_obstacles`` (signal, diff pairs, BGA/QFN fanout, via
    ``build_base_obstacle_map``), plus ``plane_obstacle_builder`` which builds
    its own map and therefore needed the call adding separately. #617 added the
    call at every site that DECIDES WHERE COPPER GOES in the three engines this
    docstring used to name as uncovered: ``plane_region_connector``
    (``npth_floor_ok`` seeds, ``wide_route_clear`` legs, ``build_base_obstacles``
    stamps), ``pcb_modification`` (``_seg_worst_offender``'s shortfall ranking
    and ``nudge_grazing_microshift``'s detector + acceptance gate) and
    ``placement/fanout_clearance`` (``_Repair``'s NPTH keep-out rects).

    STILL AT THE FLAT ``NPTH_TO_TRACK_CLEARANCE``, and deliberately so -- read
    this before "finishing the job":

    * ``pcb_modification.close_soft_joints`` and ``_connector_clear`` gate a
      BRIDGE between two pieces of copper that already exist (a soft joint's
      caps already overlap; a stub snap spans at most 1.5 track widths). When
      such a bridge violates a declared floor the flanking copper almost always
      does too, so raising the gate drops the repair without removing the
      violation -- measured, 99.96% of the refusals it would add.
    * ``pcb_modification.nudge_grazing_octolinear`` and
      ``placement/fanout_clearance.nudge_vias_for_unresolved`` are all-or-
      nothing repairs: refusing their one clearing candidate abandons the
      defect they exist to fix (measured: a -0.1 mm net-to-net overlap left in
      place; a #130 pad-via graze left unrelocated) rather than routing around
      the hole.

      **This bullet survived #756, which tried to raise a floor there and
      found out why not.** That change wanted
      ``nudge_vias_for_unresolved``'s via-DRILL-to-via-DRILL floor to follow
      the board's ``min_hole_to_hole`` (a different key from this helper's --
      ``list_nets._FLOOR_SOURCES``' ``hole_to_hole`` rather than
      ``hole_clearance``), because ``check_drc`` ``_pin_up``s exactly that
      value and both of its drill arms add it, so the pass was emitting drill
      pairs its own grader then flagged.

      A ONE-RUNG RAISE WAS MEASURED AND REJECTED, by this bullet's own
      argument: a review swept 8673 configurations of that pass's rig shape and
      625 lost the repair at the shipped 0.6 mm budget, 13 of them abandoning a
      landing ``check_drc`` grades CLEAN. What shipped is a two-rung ladder --
      prefer the declared floor, re-sweep at the fab floor when nothing clears
      -- whose second rung is the pre-#756 behaviour exactly. So the site
      stopped being all-or-nothing for that floor rather than the rule being
      bent for it, and the copper-to-hole floors this helper serves are
      untouched and still flat.

      **The lesson to carry, and it cost two reviews to get right:** "the
      grader raises this floor too" is a reason to WANT a raise, never on its
      own a licence to take one here. The question this bullet asks -- what
      happens when the one candidate is refused -- still has to be answered,
      and a ladder is how you answer it without giving up either.
    * ``placement/legality.PartPads`` builds its NPTH keep-out radii from a bare
      ``fp``/``clearance`` pair with no board pointer in hand. **#761 threaded
      the parameter rather than the board**: ``legality.resolve_npth_floor``
      calls this helper ONCE in the caller and passes the resolved float down,
      so ``PartPads`` still holds no board pointer while the two call sites
      that read hole keep-outs (``grade_pad_legality``, ``QuenchState``) do
      carry the declared floor. The four that read only pad rects, extents or
      silk deliberately do not. This bullet is kept, corrected rather than
      deleted, because "it cannot reach here" was true for two issues and a
      reader who remembers it needs to see that it stopped being true.

    The rule the first three encode: raise this floor on passes that CHOOSE
    where new copper goes or that MOVE copper by a measured shortfall, not on
    passes whose only alternative to their one candidate is doing nothing.

    Why it exists: this keep-out was priced at a hardcoded
    ``max(clearance, NPTH_TO_TRACK_CLEARANCE)`` -- a flat 0.20 fab floor -- and
    never read the board, while `check_drc` DOES read `min_hole_clearance`
    (:2390). So on a board declaring 0.25 the router would route into a band its
    own checker then flagged. Measured: a route came within 0.2263 mm of BUS1's
    NPTH against a declared 0.25, a real 0.0237 mm violation, routing-introduced
    and confirmed independently by kicad-cli. It was the single DRC failure on
    that board.

    Cached per board path: the obstacle map is rebuilt per net, and this would
    otherwise re-read the project file thousands of times in one run.
    """
    explicit = getattr(config, 'hole_clearance', 0.0) or 0.0
    if explicit > 0:
        return float(explicit)
    path = pcb_file or getattr(pcb_data, 'source_path', "") or ""
    if not path:
        return 0.0
    if path not in _HOLE_CLR_CACHE:
        try:
            from list_nets import board_constraint
            v = board_constraint(path, 'min_hole_clearance')
            v = float(v) if v and v > 0 else 0.0
            # The DECLARED floor outranks the CURRENT rule, because the rule is
            # not durable: every writeback clamps `rules.min_hole_clearance`
            # down to the clearance that step routed at, so from step 2 onward
            # the author's declaration is gone from the only place this used to
            # look. Measured on a tigard pour+route chain declaring 0.25 -- the
            # pour's writeback left rules at 0.15 and the route step then read
            # 0.15, below the 0.20 fab floor, and silently stopped honouring
            # the board. The original survives in `fab_floor_origin` (seeded at
            # the first writeback, carried down with the project), so take the
            # larger of the two. Raise-only, exactly like the rest of this
            # helper: a board with no origin, or an origin at or below the
            # rule, is bit-identical.
            from fix_kicad_drc_settings import declared_fab_floor
            _origin = declared_fab_floor(path, 'min_hole_clearance')
            if _origin and _origin > v:
                _HOLE_CLR_ORIGIN.add(path)
                v = float(_origin)
            _HOLE_CLR_CACHE[path] = v
        except Exception:                                       # noqa: BLE001
            _HOLE_CLR_CACHE[path] = 0.0
    v = _HOLE_CLR_CACHE[path]
    if v > defaults.NPTH_TO_TRACK_CLEARANCE and path not in _HOLE_CLR_ANNOUNCED:
        _HOLE_CLR_ANNOUNCED.add(path)
        _src = ("the floor the board ORIGINALLY declared, which a later step's "
                "writeback relaxed in the project"
                if path in _HOLE_CLR_ORIGIN else "the board's own "
                "min_hole_clearance")
        print(f"Copper-to-hole clearance {v:g}mm (from {_src}, above the "
              f"{defaults.NPTH_TO_TRACK_CLEARANCE}mm fab floor)")
    return v


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
    # The board's own copper-to-hole floor, once per call (cached per board).
    _hole_clr = resolve_hole_clearance(pcb_data, config)

    drill_holes = []   # every drill -> via (hole-to-hole) keep-out
    npth_holes = []    # no-copper holes only -> track keep-out
    npth_slot_holes = []  # milled SLOT subset: board-edge clearance applies (#448)

    # Via drills. The hole-to-hole keep-out is NET-INDEPENDENT (#335): a drill
    # does not care about net membership, and the old own-net exemption let the
    # router drill a new via within h2h of its own fanout via (cparti SPIm_SCK/
    # MOSI, zynq). Ref-count safe: drill keep-outs live only in the static base
    # map -- per-net caches (precompute_net_obstacles) never stamp or remove
    # them. The exemption survives ONLY for the track keep-out below, which is
    # what "reach your own through-holes" actually needs.
    for via in pcb_data.vias:
        if via.drill > 0:
            drill_holes.append((via.x, via.y, via.drill))

    # Add pad drills (through-hole AND NPTH mounting pads carry drills)
    for net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            if pad.drill > 0:
                # Slot drills expand to circles along the slot axis (short-
                # dimension diameter) instead of one long-dimension disc --
                # a 30.2x1mm milled slot must not become a 30mm round keep-out.
                circles = pad_drill_circles(pad)
                drill_holes.extend(circles)
                if not _pad_has_copper(pad):
                    # NPTH holes have no copper: the track keep-out applies to
                    # every net -- including the pad's own nominal net (#328
                    # net-tied mounting holes; copper across one's "own" NPTH
                    # hole is just as cut by the drill).
                    npth_holes.extend(circles)
                    # Milled SLOTS are board edge to KiCad's DRC (#448):
                    # copper near a slot wall is graded at copper_edge_clearance,
                    # not the NPTH hole floor. Track them separately so the
                    # slot keep-out can grow to the edge clearance below.
                    (_c1, _c2, _r) = pad_drill_capsule(pad)
                    if abs(_c2[0] - _c1[0]) > 1e-9 or abs(_c2[1] - _c1[1]) > 1e-9:
                        npth_slot_holes.extend(circles)
                elif (max(pad.size_x, pad.size_y) < pad.drill
                      and net_id not in nets_to_route_set):
                    # #441: a PLATED pad whose copper ring does NOT span its drill
                    # (a near-zero annular mounting hole -- vfo_ctrl's U4 "MH":
                    # 0.001mm copper over a 2.5mm drill) leaves the hole exposed,
                    # so _pad_has_copper is True but the pad-copper obstacle is a
                    # ~1um speck that never keeps a track off the real 2.5mm drill.
                    # Treat it as NPTH for the TRACK keep-out too. Scoped to pads
                    # FOREIGN to the nets being routed so a genuinely reachable (if
                    # degenerate) signal pad is never made unroutable.
                    npth_holes.extend(circles)

    # Track keep-out for NPTH holes on every copper layer (issue #233). The
    # copper-to-NPTH-hole floor is the JLC "NPTH to Track" fab value, never below
    # the routing clearance.
    if npth_holes:
        # extra_clearance covers geometry offset from the routed centerline
        # (diff-pair P/N tracks ride +-(gap+width)/2 off it), matching how every
        # other obstacle in that base map is inflated (issue #268).
        # `_hole_clr` is the BOARD's own min_hole_clearance -- raise-only, so a
        # board declaring nothing is byte-identical (see resolve_hole_clearance).
        npth_clr = (max(config.clearance, defaults.NPTH_TO_TRACK_CLEARANCE,
                        _hole_clr) + extra_clearance)
        block_track_cells_near_drills(obstacles, npth_holes, config.track_width,
                                      npth_clr, config.grid_step,
                                      list(range(len(config.layers))))

    # Milled NPTH SLOTS are board edge to KiCad (#448): its DRC grades copper
    # proximity to the slot wall at copper_edge_clearance (sofle_pico SW25),
    # which is usually ABOVE the NPTH hole floor stamped above. Grow the slot
    # keep-out to the effective edge clearance for tracks, and keep via COPPER
    # (not just its drill) the same distance off the slot wall. No-op when the
    # edge clearance doesn't exceed the NPTH floor (byte-identical maps).
    if npth_slot_holes:
        edge_eff = (config.board_edge_clearance if config.board_edge_clearance > 0
                    else config.clearance)
        slot_clr = edge_eff + extra_clearance
        if slot_clr > max(config.clearance, defaults.NPTH_TO_TRACK_CLEARANCE,
                          _hole_clr) + extra_clearance:
            block_track_cells_near_drills(obstacles, npth_slot_holes,
                                          config.track_width, slot_clr,
                                          config.grid_step,
                                          list(range(len(config.layers))))
        # Via-copper edge band: block_via_cells_near_drills builds its radius as
        # hole_r + clearance + via_drill/2, so passing
        # edge_eff + (via_size - via_drill)/2 yields hole_r + edge_eff + via_size/2
        # -- the via COPPER edge held edge_eff off the slot wall.
        via_slot_clr = edge_eff + (config.via_size - config.via_drill) / 2.0
        if via_slot_clr > config.hole_to_hole_clearance:
            block_via_cells_near_drills(obstacles, npth_slot_holes, config.via_drill,
                                        via_slot_clr, config.grid_step)

    # NPTH holes whose pad carries a clearance OVERRIDE above the fab floor
    # (KiCad's hole_clearance honors it, #326 residual). Plated pads are left
    # to the copper-pad blockers here: stamping their hole annulus would make
    # zero-annular-ring pads unreachable for their own signal net.
    block_track_cells_near_override_pad_holes(
        obstacles, pcb_data, config.track_width, config.clearance,
        config.grid_step, list(range(len(config.layers))),
        extra_clearance=extra_clearance, include_plated=False)

    # Via keep-out (hole-to-hole drill minimum) near every drill.
    if config.hole_to_hole_clearance > 0 and drill_holes:
        block_via_cells_near_drills(obstacles, drill_holes, config.via_drill,
                                    config.hole_to_hole_clearance, config.grid_step)

    # VIA arm of the #326 override (#505). KiCad's hole_clearance holds a via's
    # COPPER -- not merely its drill -- `local_clearance` off the hole wall. For
    # an NPTH pad nothing enforced that: the pad has no copper, so the pad-copper
    # blocker never stamps it, and the only via keep-out is the hole-to-hole
    # DRILL minimum above, which is measured drill-to-drill at a much smaller
    # value. pinci shipped 5 vias 0.76-1.13mm from 0.9/1.3mm-override mounting
    # holes, every one satisfying h2h (1.427mm) while KiCad wanted 2.225/2.625mm.
    # Scoped to NPTH like the track arm: a plated pad's own copper obstacle
    # already keeps vias off, and stamping its annulus would strand
    # zero-annular-ring pads.
    # block_via_cells_near_drills builds hole_r + clearance + via_drill/2, so
    # passing lc + (via_size - via_drill)/2 yields hole_r + lc + via_size/2 --
    # the via COPPER edge held `lc` off the hole wall (same idiom as the #448
    # slot band above).
    _via_override: Dict[float, list] = {}
    for _pads in pcb_data.pads_by_net.values():
        for _pad in _pads:
            if _pad.drill <= 0 or _pad_has_copper(_pad):
                continue
            _lc = getattr(_pad, 'local_clearance', 0.0) or 0.0
            if _lc <= 0:
                continue
            _via_clr = _lc + (config.via_size - config.via_drill) / 2.0
            if _via_clr <= config.hole_to_hole_clearance:
                continue          # the h2h stamp already covers this override
            _via_override.setdefault(round(_via_clr, 9), []).extend(
                pad_drill_circles(_pad))
    for _clr, _circles in _via_override.items():
        block_via_cells_near_drills(obstacles, _circles, config.via_drill,
                                    _clr, config.grid_step)


def add_net_stubs_as_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig,
                                extra_clearance: float = 0.0):
    """Add a net's stub segments as obstacles to the map."""
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    # Cross-class clearance (PR392): price this net's stub copper at its own KiCad
    # pairwise clearance = max(routing-side floor, its netclass). obstacle_clearance
    # is a superset of #326 B5's per-net clearance (identical when the routing-side
    # floor isn't elevated), so a foreign routed net keeps max(classA, classB).
    obs_clearance = config.obstacle_clearance(net_id)

    # Add segments - use actual segment width and the routing-side reserve width (#156)
    # FFI batching (2026-08-14): accumulate + stamp once per layer, byte-identical.
    _nb_cells: Dict[int, list] = {}
    _nb_vias: list = []
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        reserve_width = config.route_reserve_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else config.get_track_width(seg.layer)
        # #498: a .kicad_dru layer rule REPLACES the pair clearance on seg.layer.
        seg_clearance = config.layer_clearance(seg.layer, obs_clearance)
        # A track-scoped DRU rule RAISES the seg-vs-seg requirement only (#735).
        trk_clearance = config.track_obstacle_clearance(seg.net_id, seg_clearance)
        expansion_mm = reserve_width / 2 + seg_width / 2 + trk_clearance + extra_clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + seg_clearance + extra_clearance
        _c = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                         seg.end_x, seg.end_y,
                                         expansion_mm, coord.grid_step)
        if len(_c):
            _nb_cells.setdefault(layer_idx, []).append(_c)
        _v = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                         seg.end_x, seg.end_y,
                                         via_block_mm, coord.grid_step)
        if len(_v):
            _nb_vias.append(_v)
    for _li, _arrs in sorted(_nb_cells.items()):
        _call = np.concatenate(_arrs) if len(_arrs) > 1 else _arrs[0]
        _rows = np.empty((len(_call), 3), dtype=np.int32)
        _rows[:, :2] = _call
        _rows[:, 2] = _li
        obstacles.add_blocked_cells_batch(np.ascontiguousarray(_rows))
    if _nb_vias:
        _vall = np.concatenate(_nb_vias) if len(_nb_vias) > 1 else _nb_vias[0]
        obstacles.add_blocked_vias_batch(np.ascontiguousarray(_vall.astype(np.int32)))


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

        # Compute expansion based on actual segment width and the routing-side reserve (#156)
        reserve_width = config.route_reserve_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else config.get_track_width(seg.layer)
        # #498: a .kicad_dru layer rule REPLACES the pair clearance on seg.layer.
        seg_clearance = config.layer_clearance(seg.layer, config.clearance)
        expansion_mm = reserve_width / 2 + seg_width / 2 + seg_clearance + extra_clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + seg_clearance + extra_clearance
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

    # Cross-class clearance (PR392): price this net's pads at its own KiCad
    # pairwise clearance so a foreign routed net keeps max(classA, classB).
    obs_clearance = config.obstacle_clearance(net_id)
    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, config, extra_clearance,
                          clearance_override=obs_clearance)


def _via_h2h_cells(via, config: GridRouteConfig, coord: GridCoord):
    """int32 (N,2) cells of the net-INDEPENDENT drill hole-to-hole keepout around
    a via (#441): a future ROUTE via must clear this via's DRILL by the fab
    hole-to-hole floor, IN ADDITION to the copper via-via disc (which is
    copper-only). Float-centre, strict < -- mirrors block_via_cells_near_drills /
    the plane path exactly. Returns None when h2h is off or the via has no drill,
    so add/remove twins short-circuit identically and stay ref-count balanced."""
    h2h = getattr(config, 'hole_to_hole_clearance', 0.0) or 0.0
    if h2h <= 0 or getattr(via, 'drill', 0) <= 0:
        return None
    gx, gy = coord.to_grid(via.x, via.y)
    req = via.drill / 2.0 + config.via_drill / 2.0 + h2h
    de = coord.to_grid_dist_safe(req) + 1  # ceil + 1-cell bbox margin
    off = np.arange(-de, de + 1)
    dxg, dyg = np.meshgrid(off, off, indexing="ij")
    cx = (gx + dxg) * config.grid_step
    cy = (gy + dyg) * config.grid_step
    # tie -> OPEN (GRID_TIE_EPS), like the three other implementations of this
    # same keep-out: block_via_cells_near_drills and obstacle_cache's two. All
    # four must agree cell-for-cell or the incremental via maps desync from a
    # fresh rebuild (that is exactly how test_shared_via_maps failed when only
    # one of them had the epsilon).
    dm = ((cx - via.x) ** 2 + (cy - via.y) ** 2) < (req - GRID_TIE_EPS) ** 2
    if not dm.any():
        return None
    return np.column_stack([(gx + dxg)[dm], (gy + dyg)[dm]]).astype(np.int32)


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

    # Cross-class clearance (PR392): price this net's vias at its own KiCad
    # pairwise clearance so a foreign routed net/via keeps max(classA, classB).
    obs_clearance = config.obstacle_clearance(net_id)

    # Add vias - use actual via size and max track width (vias span all layers).
    # obstacle_clearance carries this net's own netclass clearance (#326 B5) plus
    # PR392's routing-side cross-class floor.
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, obs_clearance, extra_clearance)
        # #498: via barrels meet on every stack layer -> stack max.
        via_via_mm = via_size / 2 + config.via_size / 2 + config.stack_clearance(obs_clearance)
        # True via-via clearance radius in cells as a FLOAT (no floor): the disc
        # threshold is radius**2, so this blocks exactly the cells within the real
        # clearance. Flooring (to_grid_dist) lost up to ~1 cell and let two vias sit
        # a diagonal cell-offset too close (e.g. (3,2) cells = 0.36mm when 0.39mm is
        # required) -- a real cross-net via-via DRC violation the router never saw.
        via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid, diagonal_margin)
        # #441: net-independent drill hole-to-hole disc (add-only: every caller of
        # this fn stamps a fresh/clone map that is discarded, never rip-removed).
        _h2h = _via_h2h_cells(via, config, coord)
        if _h2h is not None:
            obstacles.add_blocked_vias_batch(_h2h)


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
    import obstacle_cache as _oc
    # The cell watch is armed by its own env var and must see RAW ops too --
    # they are the other half of a cell's history -- so it runs before the
    # ledger's own early return.
    if _oc._CELL_WATCH != []:
        _oc.ledger_cell_watch(obstacles, f"raw {tag} @ "
                              + _oc._ledger_site(depth=3, frames=2))
    if pre is None:
        return
    st = obstacles.get_stats()
    site = _oc._ledger_site(depth=3, frames=2)
    _oc.ledger_raw_delta(obstacles, f"{tag} @ {site}", st[0] - pre[0], st[1] - pre[1])


def _rung_small_armed():
    """#568 rust mode (KICAD_VIA_RUNG=2): the working map carries a second
    via-legality map at the small fab rung. Raw (non-cache) copper adds must
    not be invisible to it -- a rung-1 search would treat their surroundings
    as small-legal. Raw adds mirror their FULL-size via cells into the small
    map (conservative over-blocking near raw copper; never wrong), and the
    remove twins mirror the same cells so refcounts balance per rung.

    Delegates to obstacle_cache.rung_small_armed() so the raw mirrors honor
    the SAME interlock the cache path does: arming on the env var alone let a
    run with an unfrozen base (the vis branch, which marks itself unsafe)
    stamp a small map containing ONLY raw copper, so rung-1 searches saw
    neither the base nor the caches -- under-blocking, the exact failure the
    interlock exists to prevent."""
    from obstacle_cache import rung_small_armed
    return rung_small_armed()


def _via_raw_block_cells(via, config, coord, num_layers, extra_clearance,
                         diagonal_margin):
    """The (gx, gy) via-block cells add_vias_list_as_obstacles stamps for one
    via (via-via disc + #441 h2h disc), rebuilt exactly like the remove twin.
    Used only for the #568 small-map mirror."""
    gx, gy = coord.to_grid(via.x, via.y)
    via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
    via_clearance = config.obstacle_clearance(getattr(via, 'net_id', 0))
    via_via_mm = (via_size / 2 + config.via_size / 2
                  + config.stack_clearance(via_clearance))
    via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)
    off_cells = math.hypot(via.x - gx * coord.grid_step,
                           via.y - gy * coord.grid_step) / coord.grid_step
    out = []
    via_radius = via_via_expansion_grid + off_cells
    vr_range = int(math.ceil(via_radius))
    vr_sq = via_radius * via_radius
    for ex in range(-vr_range, vr_range + 1):
        for ey in range(-vr_range, vr_range + 1):
            if ex * ex + ey * ey <= vr_sq:
                out.append((gx + ex, gy + ey))
    _h2h = _via_h2h_cells(via, config, coord)
    if _h2h is not None:
        out.extend((int(a), int(b)) for a, b in _h2h)
    return out


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

    # Add vias - use actual via size and max track width (vias span all layers).
    # Cross-class clearance (PR392): price each via at ITS OWN net's KiCad pairwise
    # clearance (max(routing-side floor, that via net's class)); the REMOVE twin
    # derives the same per-via value from via.net_id so the ref-counts stay in sync.
    for via in vias:
        via_size = via.size if hasattr(via, 'size') and via.size > 0 else config.via_size
        via_clearance = config.obstacle_clearance(getattr(via, 'net_id', 0))
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, via_clearance, extra_clearance)
        via_via_mm = (via_size / 2 + config.via_size / 2
                      + config.stack_clearance(via_clearance))  # #498 stack max
        # True via-via clearance radius in cells as a FLOAT (no floor): the disc
        # threshold is radius**2, so this blocks exactly the cells within the real
        # clearance. Flooring (to_grid_dist) lost up to ~1 cell and let two vias sit
        # a diagonal cell-offset too close (e.g. (3,2) cells = 0.36mm when 0.39mm is
        # required) -- a real cross-net via-via DRC violation the router never saw.
        via_via_expansion_grid = max(1.0, via_via_mm * coord.inv_step)
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion_grid, via_via_expansion_grid, diagonal_margin)
        # #441: net-independent drill hole-to-hole disc, MIRRORED in
        # remove_vias_list_from_obstacles so rip-up stays ref-count balanced.
        _h2h = _via_h2h_cells(via, config, coord)
        if _h2h is not None:
            obstacles.add_blocked_vias_batch(_h2h)
    # #568 small-map mirror (see _rung_small_armed)
    if _rung_small_armed() and vias:
        _small = []
        for via in vias:
            _small.extend(_via_raw_block_cells(via, config, coord, num_layers,
                                               extra_clearance, diagonal_margin))
        if _small:
            obstacles.add_blocked_vias_small_batch(
                np.array(_small, dtype=np.int32))
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

    # Add segments - use actual segment width and layer-specific routing track width.
    # Cross-class clearance (PR392): price each segment at ITS OWN net's KiCad
    # pairwise clearance; the REMOVE twin recomputes the same value from seg.net_id.
    # FFI batching (2026-08-14): accumulate the memoized capsule arrays and
    # stamp once per layer after the loop (byte-identical: same rows, same
    # order, commuting inserts). The remove twin below was already batched.
    _cells_by_layer: Dict[int, list] = {}
    _via_arrs: list = []
    _small_arrs: list = []
    for seg in segments:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is not None:
            # Routing-side reserve width for the future track (#156)
            reserve_width = config.route_reserve_width(seg.layer)
            seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else config.get_track_width(seg.layer)
            seg_clearance = config.layer_clearance(  # #498: layer rule replaces
            seg.layer, config.obstacle_clearance(getattr(seg, 'net_id', 0)))
            # The track rule raises the seg-vs-seg capsule; identical line in
            # the REMOVE twin below (ref-count symmetry).
            trk_clearance = config.track_obstacle_clearance(
                getattr(seg, 'net_id', 0), seg_clearance)
            expansion_mm = reserve_width / 2 + seg_width / 2 + trk_clearance + extra_clearance
            via_block_mm = config.via_size / 2 + seg_width / 2 + seg_clearance
            _c = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                             seg.end_x, seg.end_y,
                                             expansion_mm, coord.grid_step)
            if len(_c):
                _cells_by_layer.setdefault(layer_idx, []).append(_c)
            _v = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                             seg.end_x, seg.end_y,
                                             via_block_mm, coord.grid_step)
            if len(_v):
                _via_arrs.append(_v)
            # #568 small-map mirror (see _rung_small_armed): same via capsule
            if _rung_small_armed() and len(_v):
                _small_arrs.append(_v)
    for _li, _arrs in sorted(_cells_by_layer.items()):
        _call = np.concatenate(_arrs) if len(_arrs) > 1 else _arrs[0]
        _rows = np.empty((len(_call), 3), dtype=np.int32)
        _rows[:, :2] = _call
        _rows[:, 2] = _li
        obstacles.add_blocked_cells_batch(np.ascontiguousarray(_rows))
    if _via_arrs:
        _vall = np.concatenate(_via_arrs) if len(_via_arrs) > 1 else _via_arrs[0]
        obstacles.add_blocked_vias_batch(np.ascontiguousarray(_vall.astype(np.int32)))
    if _small_arrs:
        _sall = (np.concatenate(_small_arrs) if len(_small_arrs) > 1
                 else _small_arrs[0])
        obstacles.add_blocked_vias_small_batch(np.asarray(_sall, dtype=np.int32))
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
        # segment, same per-net cross-class clearance, same #156 reserve width)
        # or the Rust ref-counts desync on rip-up.
        reserve_width = config.route_reserve_width(seg.layer)
        seg_width = seg.width if hasattr(seg, 'width') and seg.width > 0 else config.get_track_width(seg.layer)
        seg_clearance = config.layer_clearance(  # #498: layer rule replaces
            seg.layer, config.obstacle_clearance(getattr(seg, 'net_id', 0)))
        # Identical track-rule raise to the ADD twin, or ref-counts desync.
        trk_clearance = config.track_obstacle_clearance(
            getattr(seg, 'net_id', 0), seg_clearance)
        expansion_mm = reserve_width / 2 + seg_width / 2 + trk_clearance + extra_clearance
        via_block_mm = config.via_size / 2 + seg_width / 2 + seg_clearance

        # Sweep item 2 (#625 follow-up): the arrays already exist -- stack a
        # layer column instead of per-row int() tuple appends (this runs on
        # EVERY rip/restore; the batch rows are the identical multiset).
        cell_arr = segment_blocked_cells_array(
            seg.start_x, seg.start_y, seg.end_x, seg.end_y, expansion_mm, coord.grid_step)
        if len(cell_arr):
            cells_to_remove.append(np.column_stack(
                [cell_arr.astype(np.int32),
                 np.full(len(cell_arr), layer_idx, dtype=np.int32)]))
        via_arr = segment_blocked_cells_array(
            seg.start_x, seg.start_y, seg.end_x, seg.end_y, via_block_mm, coord.grid_step)
        if len(via_arr):
            vias_to_remove.append(via_arr.astype(np.int32))

    # Batch remove cells and vias
    if cells_to_remove:
        obstacles.remove_blocked_cells_batch(np.concatenate(cells_to_remove))
    if vias_to_remove:
        vias_array = np.concatenate(vias_to_remove)
        obstacles.remove_blocked_vias_batch(vias_array)
        if _rung_small_armed():  # #568: mirror of the add-side small stamp
            obstacles.remove_blocked_vias_small_batch(vias_array)
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

        # Mirror add_vias_list_as_obstacles EXACTLY: same per-via cross-class
        # clearance from via.net_id, or rip-up over/under-decrements the ref-counts.
        via_clearance = config.obstacle_clearance(getattr(via, 'net_id', 0))
        via_track_expansion_grid = _via_track_expansion_per_layer(via_size, config, coord, via_clearance, extra_clearance)
        via_via_mm = (via_size / 2 + config.via_size / 2
                      + config.stack_clearance(via_clearance))  # #498 stack max
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
        # Sweep item 2 (#625 follow-up): disc enumeration via a mask over the
        # integer offset grid. The threshold stays the scalar's `radius ** 2`
        # (libm pow -- radius*radius rounds 1 ULP apart on rare values and
        # would flip borderline cells); integer ex*ex+ey*ey against that
        # scalar is an exact comparison, so the cell multiset is identical.
        for layer_idx in range(num_layers):
            radius = via_track_expansion_grid[layer_idx] + diagonal_margin + off_cells
            effective_track_block_sq = radius ** 2
            track_block_range = int(math.ceil(radius))
            ax = np.arange(-track_block_range, track_block_range + 1, dtype=np.int32)
            EX, EY = np.meshgrid(ax, ax, indexing='ij')
            m = EX * EX + EY * EY <= effective_track_block_sq
            if m.any():
                cells_to_remove.append(np.column_stack(
                    [EX[m] + gx, EY[m] + gy,
                     np.full(int(m.sum()), layer_idx, dtype=np.int32)]))

        # Via blocking cells
        via_radius = via_via_expansion_grid + off_cells
        vr_range = int(math.ceil(via_radius))
        vr_sq = via_radius * via_radius
        ax = np.arange(-vr_range, vr_range + 1, dtype=np.int32)
        EX, EY = np.meshgrid(ax, ax, indexing='ij')
        m = EX * EX + EY * EY <= vr_sq
        if m.any():
            vias_to_remove.append(np.column_stack([EX[m] + gx, EY[m] + gy]))

        # #441: mirror the drill hole-to-hole disc add_vias_list_as_obstacles
        # stamped (same _via_h2h_cells), so rip-up removes exactly what it added.
        _h2h = _via_h2h_cells(via, config, coord)
        if _h2h is not None and len(_h2h):
            vias_to_remove.append(np.asarray(_h2h, dtype=np.int64))

    # Batch remove cells and vias
    if cells_to_remove:
        cells_array = np.concatenate(cells_to_remove).astype(np.int32)
        obstacles.remove_blocked_cells_batch(cells_array)
    if vias_to_remove:
        vias_array = np.concatenate(vias_to_remove).astype(np.int32)
        obstacles.remove_blocked_vias_batch(vias_array)
        if _rung_small_armed():  # #568: mirror of the add-side small stamp
            obstacles.remove_blocked_vias_small_batch(vias_array)
    _ledger_close(obstacles, _pre, "remove_vias_list")


def same_net_pad_via_keepout_cells(pcb_data: PCBData, net_id: int,
                                   config: GridRouteConfig) -> "np.ndarray":
    """#581: (N, 2) via-block cells over the net's own SMD pads when an active
    (> 0) same_net_pad_clearance is on the config; empty otherwise.

    Blocks VIA placement only (never tracks) at pad-edge + via/2 + clearance,
    mirroring plane_obstacle_builder._add_pad_via_obstacle's geometry.
    Through-hole pads are exempt (their barrel is the layer transition, and
    the #581 concern is SMD reflow)."""
    snpc = getattr(config, 'same_net_pad_clearance', -1.0)
    if snpc is None or snpc <= 0:
        return np.empty((0, 2), dtype=np.int32)
    from routing_utils import pad_blocked_cells_array
    coord = GridCoord(config.grid_step)
    margin = config.via_size / 2 + snpc + config.grid_step / 2
    chunks = []
    for pad in pcb_data.pads_by_net.get(net_id, []):
        if getattr(pad, 'drill', 0):
            continue
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        hw, hh = pad.size_x / 2, pad.size_y / 2
        if pad.shape in ('circle', 'oval'):
            cr = min(hw, hh)
        elif pad.shape == 'roundrect':
            cr = getattr(pad, 'roundrect_rratio', 0.25) * min(pad.size_x,
                                                              pad.size_y)
        else:
            cr = 0
        cells = pad_blocked_cells_array(
            gx, gy, hw, hh, margin, config.grid_step, cr,
            off_x=pad.global_x - gx * coord.grid_step,
            off_y=pad.global_y - gy * coord.grid_step,
            rotation_deg=getattr(pad, 'rect_rotation', 0.0) or 0.0)
        if len(cells):
            chunks.append(cells)
    if not chunks:
        return np.empty((0, 2), dtype=np.int32)
    return np.concatenate(chunks)


def add_same_net_via_clearance(obstacles: GridObstacleMap, pcb_data: PCBData,
                                net_id: int, config: GridRouteConfig):
    """Add via-via clearance blocking for same-net vias.

    This blocks only via placement (not track routing) near existing vias on the same net,
    enforcing DRC via-via clearance even within a single net.
    """
    coord = GridCoord(config.grid_step)

    # #581: keep every new via off this net's own SMD pads when the board
    # carries an active same-net pad via clearance. Callers of this function
    # stamp CLONED per-route maps (Phase 3 taps, the non-incremental builder),
    # so no balanced removal is needed. MIRROR into the small-rung map (#568):
    # a rung-1 search consults ONLY blocked_vias_small for dynamic copper, so
    # without the mirror it drops a small fab-rung via straight into the pad
    # this keep-out exists to protect (neo6502: 0.45mm vias in R14/U6 pads).
    _pad_cells = same_net_pad_via_keepout_cells(pcb_data, net_id, config)
    if len(_pad_cells):
        obstacles.add_blocked_vias_batch(_pad_cells)
        try:
            if _rung_small_armed():
                obstacles.add_blocked_vias_small_batch(_pad_cells)
        except (AttributeError, NameError):
            pass

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
        # Only block via placement, not track routing (tracks can pass through
        # same-net vias). Sweep item 3 (#625): mask over the integer offset
        # grid + one batch call instead of one FFI call per cell (this runs
        # per net per prepare, re-run every rip round); integer ex*ex+ey*ey
        # against the same scalar threshold blocks the identical cell set,
        # and the batch increments refcounts exactly like the per-cell add.
        ax = np.arange(-rng, rng + 1, dtype=np.int32)
        EX, EY = np.meshgrid(ax, ax, indexing='ij')
        m = EX * EX + EY * EY <= radius_sq
        if m.any():
            obstacles.add_blocked_vias_batch(
                np.column_stack([EX[m] + gx, EY[m] + gy]))


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

        # Sweep item 3 (#625): broadcast the capsule distance over the offset
        # grid and batch the adds (was one scalar distance + one FFI call per
        # cell, per net per prepare). The multiply-squared kernel NOMINATES:
        # clearly-inside cells pass, cells within a few ULP of the strict
        # `< required_dist` boundary are re-judged with the scalar (its **2 =
        # libm pow rounds 1 ULP apart on rare values) -- identical cell set.
        ax = np.arange(-expand, expand + 1, dtype=np.int64)
        EX, EY = np.meshgrid(ax, ax, indexing='ij')
        exf, eyf = EX.ravel(), EY.ravel()
        cxs = (gx + exf) * step
        cys = (gy + eyf) * step
        dx_, dy_ = p2x - p1x, p2y - p1y
        len_sq = dx_ * dx_ + dy_ * dy_
        d2 = _pt_seg_d2_arr(cxs, cys, p1x, p1y, dx_, dy_, len_sq)
        req2 = required_dist * required_dist
        lo = req2 * (1 - 1e-12)
        hi = req2 * (1 + 1e-12)
        take = d2 < lo
        border = np.nonzero((d2 >= lo) & (d2 <= hi))[0]
        for i in border:
            take[i] = point_to_segment_distance(
                float(cxs[i]), float(cys[i]), p1x, p1y, p2x, p2y) < required_dist
        take &= ~((exf == 0) & (eyf == 0))  # keep the pad centre landable
        if take.any():
            obstacles.add_blocked_vias_batch(np.column_stack(
                [exf[take] + gx, eyf[take] + gy]).astype(np.int32))


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
    from kicad_parser import pad_is_plated_through
    for pad in pads:
        # pad_is_plated_through, NOT bare drill>0 (#328): a net-tied NPTH
        # mounting hole has no barrel -- suppressing a via there would ship a
        # broken layer transition.
        if pad_is_plated_through(pad):
            # Offset pads (#325): the reusable BARREL is at the hole anchor,
            # not the (possibly offset) copper centre.
            hx = getattr(pad, 'hole_x', None)
            hy = getattr(pad, 'hole_y', None)
            positions.add(coord.to_grid(hx if hx is not None else pad.global_x,
                                        hy if hy is not None else pad.global_y))
            # The COPPER-centre cell suppresses too (#335): endpoints -- and
            # therefore the router's layer transitions -- land on the copper
            # centre, and a plated pad's copper spans every layer, so a
            # transition there needs no via. For offset-drill pads the two
            # cells differ, and suppressing only the hole cell let the router
            # emit a via ON the pad copper within hole-to-hole of the offset
            # barrel (caravel U8.45 vdda / U8.36 UART8_RX).
            positions.add(coord.to_grid(pad.global_x, pad.global_y))

    # Existing same-net vias are reusable holes too (see free-via reuse). Without
    # this, only the multipoint tap path augmented the set, so the Phase-1 main edge
    # and single-ended routes dropped a duplicate via onto an existing via-in-pad.
    for via in pcb_data.vias:
        if via.net_id == net_id:
            positions.add(coord.to_grid(via.x, via.y))

    return positions


def _batch_cells_one_layer(obstacles, cells_xy: "np.ndarray", layer_idx: int,
                           blocked_cells=None, sink=None):
    """Block an (N, 2) array of cells on one layer via the batch API.

    ``sink`` (a {layer_idx: [arrays]} dict) DEFERS the Rust call: the caller
    accumulates every pad's cells and stamps once per layer at the end. Same
    row multiset per layer, so the refcounts land identically whether the rows
    arrive split or joined -- the argument the 2026-08-14 FFI batching pass
    already made for the segment and via loops. Pads were the loop it missed.
    """
    if len(cells_xy) == 0:
        return
    if sink is not None:
        sink.setdefault(layer_idx, []).append(cells_xy)
    else:
        rows = np.empty((len(cells_xy), 3), dtype=np.int32)
        rows[:, :2] = cells_xy
        rows[:, 2] = layer_idx
        obstacles.add_blocked_cells_batch(np.ascontiguousarray(rows))
    if blocked_cells is not None:
        blocked_cells[layer_idx].update(map(tuple, cells_xy.tolist()))


def _flush_cell_sink(obstacles, sink):
    """Stamp everything a ``sink`` accumulated: one Rust call per layer."""
    for layer_idx, arrs in sorted(sink.items()):
        cells = np.concatenate(arrs) if len(arrs) > 1 else arrs[0]
        rows = np.empty((len(cells), 3), dtype=np.int32)
        rows[:, :2] = cells
        rows[:, 2] = layer_idx
        obstacles.add_blocked_cells_batch(np.ascontiguousarray(rows))
    sink.clear()


def _batch_vias(obstacles, vias_xy: "np.ndarray", blocked_vias=None, sink=None):
    """Block an (N, 2) array of via positions via the batch API.

    ``sink`` (a list) defers the Rust call -- see _batch_cells_one_layer."""
    if len(vias_xy) == 0:
        return
    if sink is not None:
        sink.append(vias_xy)
    else:
        obstacles.add_blocked_vias_batch(np.ascontiguousarray(vias_xy.astype(np.int32)))
    if blocked_vias is not None:
        blocked_vias.update(map(tuple, vias_xy.tolist()))


def _flush_via_sink(obstacles, sink):
    """Stamp everything a via ``sink`` accumulated: one Rust call."""
    if sink:
        allv = np.concatenate(sink) if len(sink) > 1 else sink[0]
        obstacles.add_blocked_vias_batch(np.ascontiguousarray(allv.astype(np.int32)))
        del sink[:]




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
    layer at THAT layer's routing-side reserve width (#156: nominal for the
    single-ended engine, where impedance/power extra width rides the routed
    net's own track_margin; the full layer width for the diff engine). Replaces
    a single max_track_width value, which over-covered thinner layers and
    double-counted the router's per-net track_margin for wide nets. Matches the
    cache (_collect_via_obstacles) and the segment stamp.

    #498: `clearance` is the caller's net/class-resolved fallback; each layer
    a .kicad_dru rule covers uses the rule value instead (the via meets that
    layer's copper ON that layer). Add/remove twins share this helper, so the
    per-layer values cannot desync ref-counts."""
    return [max(1, coord.to_grid_dist_safe(
                via_size / 2 + config.route_reserve_width(layer) / 2
                + config.layer_clearance(layer, clearance) + extra_clearance))
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


class _RecordingObstacles:
    """Forwarding proxy that records the exact TRACK-cell batches a stamp
    adds to the map (via blocking passes through unrecorded). The recorded
    arrays are the precise multiset for a later balanced remove/re-add
    (net-tie corridor lift)."""

    def __init__(self, real):
        self._real = real
        self.cell_batches = []

    def add_blocked_cells_batch(self, cells):
        self.cell_batches.append(np.array(cells, copy=True))
        self._real.add_blocked_cells_batch(cells)

    def add_blocked_cell(self, gx, gy, layer):
        self.cell_batches.append(np.array([[gx, gy, layer]], dtype=np.int32))
        self._real.add_blocked_cell(gx, gy, layer)

    def merged_cells(self):
        if not self.cell_batches:
            return np.empty((0, 3), dtype=np.int32)
        return np.vstack(self.cell_batches).astype(np.int32)

    def __getattr__(self, name):
        return getattr(self._real, name)


# #625: cache of the expensive per-(own, partner) pad sampling below --
# the partner-minus-own bad region reduced to its boundary points. It is a
# pure function of the two pads' geometry (fine is a constant), yet it was
# recomputed inside EVERY build_base_obstacle_map call: ~48 s per pass on
# core64_logic's 14 custom-pad solder jumpers (2.16M point_to_pad_distance
# calls -> 134M segment distances), multiplied by every rescue-rung window,
# plane-finalize leg and reconcile sub-run rebuild -- hours of CPU on
# 1.5 mm jumpers. Keyed by pad identity + position + size so a GUI process
# that reloads an edited board never reuses stale samples. Value:
# (bad_x, bad_y, bad_keys) post-boundary-reduction, or None when the
# partner sampling found no in-pad points.
_TIE_PAIR_SAMPLE_CACHE: Dict[tuple, object] = {}


def _tie_pad_key(pad):
    return (pad.component_ref, pad.pad_number, tuple(pad.layers or ()),
            round(pad.global_x, 6), round(pad.global_y, 6),
            round(pad.size_x, 6), round(pad.size_y, 6))


def _pt_seg_d2_arr(px, py, x1, y1, dx, dy, len_sq):
    """Squared point-to-segment distance over point arrays vs ONE segment --
    the point_to_segment_distance formula with its exact proj association
    (multiply-squared: nominate with it, judge borderline cells with the
    scalar, whose **2 is libm pow)."""
    if len_sq < 1e-10:
        ddx = px - x1
        ddy = py - y1
        return ddx * ddx + ddy * ddy
    t = np.clip(((px - x1) * dx + (py - y1) * dy) / len_sq, 0.0, 1.0)
    ddx = px - (x1 + t * dx)
    ddy = py - (y1 + t * dy)
    return ddx * ddx + ddy * ddy


def _pad_dist_le_batch(xs, ys, pad, tol):
    """Vectorized `point_to_pad_distance(x, y, pad) <= tol` over coordinate
    arrays -- the same geometry as check_drc's scalar (custom polygons,
    rounded/rect/circle/oval, rect_rotation frames), without the per-point
    Python. #625: the corridor sampler below ran the scalar ~2M times per
    cold pass (134M segment distances) -- ~44 s that this brings to ~1 s."""
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    polys = getattr(pad, 'polygons', None)
    if polys:
        out = np.zeros(xs.shape, dtype=bool)
        tol_sq = tol * tol
        for poly in polys:
            P = np.asarray(poly, dtype=np.float64)
            if len(P) < 2:
                continue
            x1, y1 = P[:, 0], P[:, 1]
            x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
            ex, ey = x2 - x1, y2 - y1
            len_sq = ex * ex + ey * ey
            # Chunk the (points x edges) broadcasts to bound temporaries.
            _B = 8192
            for s in range(0, xs.size, _B):
                px = xs[s:s + _B, None]
                py = ys[s:s + _B, None]
                # Even-odd ray cast, the scalar _point_in_poly comparisons
                # (its (i, j=i-1) vertex pairs are this same edge set).
                cond = (y1[None, :] > py) != (y2[None, :] > py)
                with np.errstate(divide='ignore', invalid='ignore'):
                    xint = ex[None, :] * (py - y1[None, :]) / (y2 - y1)[None, :] + x1[None, :]
                    crossing = cond & (px < xint)
                inside = (np.count_nonzero(crossing, axis=1) & 1).astype(bool)
                # Min point-segment distance over edges, the scalar
                # point_to_segment_distance formula (degenerate edge -> p1).
                apx = px - x1[None, :]
                apy = py - y1[None, :]
                with np.errstate(divide='ignore', invalid='ignore'):
                    t = np.clip((apx * ex[None, :] + apy * ey[None, :]) / len_sq[None, :],
                                0.0, 1.0)
                t = np.where(len_sq[None, :] < 1e-10, 0.0, t)
                ddx = apx - t * ex[None, :]
                ddy = apy - t * ey[None, :]
                d2 = (ddx * ddx + ddy * ddy).min(axis=1)
                out[s:s + _B] |= inside | (d2 <= tol_sq)
        return out
    # Rounded/rect/circle/oval path, the scalar point_to_pad_distance tail.
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(pad.size_x, pad.size_y) / 2
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0.0
    x, y = xs, ys
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        cos_r, sin_r = math.cos(rad), math.sin(rad)
        dx0 = xs - pad.global_x
        dy0 = ys - pad.global_y
        x = pad.global_x + dx0 * cos_r + dy0 * sin_r
        y = pad.global_y - dx0 * sin_r + dy0 * cos_r
    rel_x = np.abs(x - pad.global_x)
    rel_y = np.abs(y - pad.global_y)
    half_x, half_y = pad.size_x / 2, pad.size_y / 2
    dxe = np.maximum(0.0, rel_x - half_x)
    dye = np.maximum(0.0, rel_y - half_y)
    dist = np.sqrt(dxe * dxe + dye * dye)
    if corner_radius > 0:
        inner_x = half_x - corner_radius
        inner_y = half_y - corner_radius
        corner = (rel_x > inner_x) & (rel_y > inner_y)
        cdx = rel_x - inner_x
        cdy = rel_y - inner_y
        dist = np.where(
            corner,
            np.maximum(0.0, np.sqrt(cdx * cdx + cdy * cdy) - corner_radius),
            dist)
    return dist <= tol


def _compute_net_tie_corridors(pcb_data, config, coord):
    """Per tied net: the (gx, gy) cells where KiCad's net-tie exemption lets
    that net's copper pass its PARTNER copper, plus the partner pad/net ids
    whose stamps the corridor may lift.

    KiCad (DRC_ENGINE::IsNetTieExclusion) waives a (track, partner) pair when
    the pair's deepest contact lies on the track's OWN pad of the group --
    shallower grazes of the same pair are then not re-flagged (the human
    cynthion escape leaves the sense tab with an 11um graze past the
    partner's edge, accepted by kicad-cli). Cell-level equivalent: a
    centerline cell is corridor-legal iff its copper disc cannot touch
    partner copper OUTSIDE the own pad --

        dist(cell, partner_minus_own_pad) >= track_half_width

    -- full overlap inside the own pad, graze-only passage beyond it, hard
    block anywhere the copper would reach partner copper off the own pad.
    The partner-minus-own region is sampled at 0.01mm (guard = one step).

    Returns {tied_net_id: {'cells': set[(gx,gy)], 'partner_pad_ids': set,
    'partner_net_ids': set}}.
    """
    corridors: Dict[int, dict] = {}
    fine = 0.01
    try:
        from check_drc import point_to_pad_distance
    except Exception:
        return corridors
    for fp in pcb_data.footprints.values():
        if not getattr(fp, 'net_tie_groups', None):
            continue
        by_num = {}
        for p in fp.pads:
            by_num.setdefault(p.pad_number, []).append(p)
        for group in fp.net_tie_groups:
            members = [p for num in group for p in by_num.get(num, [])]
            for own in members:
                if own.net_id == 0:
                    continue
                partners = [p for p in members if p.net_id not in (0, own.net_id)]
                if not partners:
                    continue
                half_w = (config.get_net_track_width(own.net_id, config.layers[0]) / 2
                          if hasattr(config, 'get_net_track_width')
                          else config.track_width / 2)
                entry = corridors.setdefault(
                    own.net_id, {'cells': set(), 'safe_cells': set(),
                                 'partner_pad_ids': set(),
                                 'partner_net_ids': set()})
                for partner in partners:
                    pex = partner.size_x / 2 + partner.size_y / 2
                    x0, x1 = partner.global_x - pex, partner.global_x + pex
                    y0, y1 = partner.global_y - pex, partner.global_y + pex
                    # #625: the (bad_x, bad_y, bad_keys) sampling below is a
                    # pure function of the two pads -- serve repeat builds
                    # (rescue windows, finalize legs, reconcile sub-runs)
                    # from the cache instead of re-sampling ~63k points
                    # through custom-pad polygon distances every time.
                    _ck = (_tie_pad_key(own), _tie_pad_key(partner))
                    if _ck in _TIE_PAIR_SAMPLE_CACHE:
                        _cv = _TIE_PAIR_SAMPLE_CACHE[_ck]
                        if _cv is None:
                            continue
                        bad_x, bad_y, _bad_packed = _cv
                    else:
                        sx = np.arange(x0, x1 + fine, fine)
                        sy = np.arange(y0, y1 + fine, fine)
                        SX, SY = np.meshgrid(sx, sy)
                        SXf, SYf = SX.ravel(), SY.ravel()
                        in_p = _pad_dist_le_batch(SXf, SYf, partner, 1e-9)
                        if not in_p.any():
                            _TIE_PAIR_SAMPLE_CACHE[_ck] = None
                            continue
                        px, py = SXf[in_p], SYf[in_p]
                        out_o = ~_pad_dist_le_batch(px, py, own, 1e-9)
                        bad_x, bad_y = px[out_o], py[out_o]
                        # Memory: the dense cells x bad-points distance matrix hit
                        # GB-scale temporaries (a 1.5mm pad sampled at 0.01mm is
                        # ~20k points; 7GB footprint on hackrf's NT jumpers).
                        # Split the test: (a) a cell whose disc CENTER falls in
                        # the bad region fails by set membership (no distances
                        # needed); (b) for the rest, the nearest bad point is on
                        # the region BOUNDARY, so the distance matrix only needs
                        # boundary points -- identical results, ~100x smaller.
                        _bad_packed = None
                        if bad_x.size > 256:
                            _kx = np.round(bad_x / fine).astype(np.int64)
                            _ky = np.round(bad_y / fine).astype(np.int64)
                            # Interior = all 4 lattice neighbors present; test
                            # via packed int64 keys (np.isin) instead of 4 set
                            # probes per point. The packed key array replaces
                            # the old tuple set (item 13): its only other
                            # consumer, the center-in-region kill below, is
                            # an np.isin too.
                            _pk = (_kx << 32) + _ky
                            _bad_packed = np.sort(_pk)
                            _boundary = ~(np.isin(_pk + (1 << 32), _bad_packed)
                                          & np.isin(_pk - (1 << 32), _bad_packed)
                                          & np.isin(_pk + 1, _bad_packed)
                                          & np.isin(_pk - 1, _bad_packed))
                            bad_x, bad_y = bad_x[_boundary], bad_y[_boundary]
                        _TIE_PAIR_SAMPLE_CACHE[_ck] = (bad_x, bad_y, _bad_packed)
                    # Candidate cells: everything a stamp of this partner's
                    # copper could have blocked (bbox + keep-out reach + 1).
                    reach = half_w + config.clearance + coord.grid_step
                    gx0, gy0 = coord.to_grid(x0 - reach, y0 - reach)
                    gx1, gy1 = coord.to_grid(x1 + reach, y1 + reach)
                    xs = np.arange(gx0, gx1 + 1, dtype=np.int32)
                    ys = np.arange(gy0, gy1 + 1, dtype=np.int32)
                    GX, GY = np.meshgrid(xs, ys)
                    cxm = GX.ravel() * coord.grid_step
                    cym = GY.ravel() * coord.grid_step
                    if bad_x.size:
                        # Chunked min-distance: bounds the temporaries to
                        # ~chunk x boundary-points instead of one dense
                        # cells x points matrix.
                        thr = (half_w + fine) ** 2
                        ok = np.empty(cxm.shape, dtype=bool)
                        _B = 2048
                        for _s in range(0, cxm.size, _B):
                            d2 = ((cxm[_s:_s + _B, None] - bad_x[None, :]) ** 2 +
                                  (cym[_s:_s + _B, None] - bad_y[None, :]) ** 2).min(axis=1)
                            ok[_s:_s + _B] = d2 >= thr
                        if _bad_packed is not None:
                            # (a) center-in-region kill (boundary points alone
                            # under-measure distances for interior cells).
                            _ckx = np.round(cxm / fine).astype(np.int64)
                            _cky = np.round(cym / fine).astype(np.int64)
                            _inside = np.isin((_ckx << 32) + _cky, _bad_packed)
                            ok &= ~_inside
                    else:
                        ok = np.ones(cxm.shape, dtype=bool)
                    if not ok.any():
                        continue
                    entry['cells'].update(
                        zip(GX.ravel()[ok].tolist(), GY.ravel()[ok].tolist()))
                    # #667: cells whose CENTER lies on the OWN pad are the
                    # KiCad-waived approach (contact on the own pad); the
                    # rest of the corridor is the segment-level hazard band
                    # the #667 pricing steers away from. Uniform pricing was
                    # measured INERT (no gradient = no steering).
                    _on_own = _pad_dist_le_batch(cxm[ok], cym[ok], own, 1e-9)
                    if _on_own.any():
                        entry['safe_cells'].update(
                            zip(GX.ravel()[ok][_on_own].tolist(),
                                GY.ravel()[ok][_on_own].tolist()))
                    entry['partner_pad_ids'].add(id(partner))
                    entry['partner_net_ids'].add(partner.net_id)
    return {n: e for n, e in corridors.items() if e['cells']}


def _assemble_net_tie_lifts(corridors, recorded, layer_map):
    """{tied_net_id: [cell arrays]} -- for each tied net, the rows of the
    RECORDED tie-copper stamps (partner pads + partner nets' trunk copper)
    that fall inside that net's corridor. Exact subsets of what the base
    build added, so remove/re-add stays refcount-balanced."""
    lifts: Dict[int, List["np.ndarray"]] = {}
    if not corridors or not recorded:
        return lifts
    for net_id, entry in corridors.items():
        cells = entry['cells']
        # Item 13: packed-int membership instead of a tuple-set probe per row.
        packed_cells = np.sort(np.fromiter(
            ((gx << 32) + gy for gx, gy in cells), dtype=np.int64, count=len(cells)))
        for kind, key, arr in recorded:
            if kind == 'pad' and key not in entry['partner_pad_ids']:
                continue
            if kind == 'net' and key not in entry['partner_net_ids']:
                continue
            if not len(arr):
                continue
            a = np.asarray(arr, dtype=np.int64)
            mask = np.isin((a[:, 0] << 32) + a[:, 1], packed_cells)
            if mask.any():
                lifts.setdefault(net_id, []).append(arr[mask])
    return lifts


def _add_pad_obstacle(obstacles: GridObstacleMap, pad, coord: GridCoord,
                      layer_map: Dict[str, int], config: GridRouteConfig,
                      extra_clearance: float = 0.0,
                      blocked_cells: List[Set[Tuple[int, int]]] = None,
                      blocked_vias: Set[Tuple[int, int]] = None,
                      clearance_override: float = None,
                      skip_cell=None,
                      cell_sink=None, via_sink=None):
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

    # #498 per-layer .kicad_dru rules: resolve the pair clearance PER LAYER (a
    # layer rule REPLACES the net/class fallback). A pad OVERRIDE then
    # REPLACES that, floored at rules.min_clearance -- KiCad returns before it
    # looks at a class or a rule (design_rules.override_clearance, measured on
    # KiCad 10). Layers sharing a resolved value share one rasterization, so a
    # board without rules or overrides takes exactly the old single-margin path.
    def _layer_clr(layer_name):
        return config.pad_override_clearance(
            config.layer_clearance(layer_name, clearance), pad)

    def _clr_groups(expanded):
        groups = {}
        for l in expanded:
            if l in layer_map:
                groups.setdefault(_layer_clr(l), []).append(layer_map[l])
        return groups

    def _via_clr(expanded):
        # A via barrel meets this pad's copper on every layer the pad carries
        # it -> stack max of the per-layer resolution.
        return max((_layer_clr(l) for l in expanded if l.endswith('.Cu')),
                   default=max(clearance, lc))

    if lc > clearance:
        clearance = lc

    # Custom comb/finger pads (issue #188): block the REAL copper polygon(s)
    # expanded by margin, leaving the finger channels open, instead of the
    # size_x x size_y bounding box (which fills the notches and the empty side of
    # an off-anchor pad, walling in the pads that route through them).
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        expanded_layers = expand_pad_layers(pad.layers, config.layers)
        clr_groups = _clr_groups(expanded_layers)
        on_copper = any(l.endswith('.Cu') for l in expanded_layers)
        via_margin = config.via_size / 2 + _via_clr(expanded_layers) + extra_clearance

        def _emit(poly, m, via_pass, layer_idxs=None):
            gx_lo, gy_lo, nx, ny, inside, edist = _rasterize_polygon_box(poly, coord, m)
            if inside is None:
                return
            mask = inside | (edist <= m - GRID_TIE_EPS)
            if not mask.any():
                return
            gxs, gys = _box_masked_cells(gx_lo, gy_lo, nx, mask)
            if skip_cell is not None:
                keep = np.fromiter(
                    (not skip_cell(int(gxs[i]), int(gys[i])) for i in range(gxs.size)),
                    dtype=bool, count=gxs.size)
                gxs = gxs[keep]; gys = gys[keep]
                if gxs.size == 0:
                    return
            if via_pass:
                _batch_vias(obstacles, np.column_stack([gxs, gys]), blocked_vias)
            else:
                _block_cells_sel(obstacles, gxs, gys, layer_idxs)
                if blocked_cells is not None:
                    for li in layer_idxs:
                        if li < len(blocked_cells):
                            blocked_cells[li].update(zip(gxs.tolist(), gys.tolist()))

        for poly in pad_polys:
            for g_clr, g_idxs in clr_groups.items():
                _emit(poly, config.track_width / 2 + g_clr + extra_clearance,
                      via_pass=False, layer_idxs=g_idxs)
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
    for g_clr, g_idxs in _clr_groups(expanded_layers).items():
        cells = pad_blocked_cells_array(gx, gy, half_width, half_height,
                                        config.track_width / 2 + g_clr + extra_clearance,
                                        config.grid_step, corner_radius, corner_buffer,
                                        off_x, off_y, rotation_deg=pad.rect_rotation)
        if skip_cell is not None and len(cells):
            keep = np.fromiter((not skip_cell(int(cx), int(cy)) for cx, cy in cells),
                               dtype=bool, count=len(cells))
            cells = cells[keep]
        for layer_idx in g_idxs:
            _batch_cells_one_layer(obstacles, cells, layer_idx, blocked_cells,
                                   sink=cell_sink)

    # Via blocking near pads - block vias if pad is on any copper layer
    if any(layer.endswith('.Cu') for layer in expanded_layers):
        via_margin = config.via_size / 2 + _via_clr(expanded_layers) + extra_clearance
        via_cells = pad_blocked_cells_array(gx, gy, half_width, half_height, via_margin,
                                            config.grid_step, corner_radius, corner_buffer,
                                            off_x, off_y, rotation_deg=pad.rect_rotation)
        if skip_cell is not None and len(via_cells):
            keep = np.fromiter((not skip_cell(int(cx), int(cy)) for cx, cy in via_cells),
                               dtype=bool, count=len(via_cells))
            via_cells = via_cells[keep]
        _batch_vias(obstacles, via_cells, blocked_vias, sink=via_sink)


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
    # Cross-class clearance (PR392): price the keep-out at this pad net's KiCad
    # pairwise clearance. add/remove call this helper identically, so the derived
    # value is the same on both sides -> ref-counts stay balanced.
    clearance = config.obstacle_clearance(getattr(pad, 'net_id', 0))
    # #498: a via barrel meets the pad's copper on every layer it carries ->
    # stack max of the per-layer .kicad_dru resolution (fallback = the value
    # above on unruled layers).
    clearance = max((config.layer_clearance(l, clearance)
                     for l in expanded_layers if l.endswith('.Cu')),
                    default=clearance)
    # A pad override REPLACES the resolved value (floored at the board
    # minimum), it is not a floor on top of it -- KiCad semantics, measured.
    clearance = config.pad_override_clearance(clearance, pad)
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


# ============================================================================
# Line/clearance geometry helpers
# ============================================================================

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
