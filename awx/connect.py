#!/usr/bin/env python3
"""connect(): the ONE braid -> stub connection primitive, on the real router.

The trunk decides order and layers; a connection is the short piece of
copper from where a net leaves the trunk to the free end of its fanout
stub. It used to be four hand-drawn mechanisms (a west-face jog, a
south-port A*, a river climb, a via under a B.Cu stub end), each written
for one face/layer combination met on one board. This is the general
form: route the net between the copper island at `a` and the island at
`b` with the production grid A* (`route_net_with_obstacles`), inside a
fenced window around the two points, against the production obstacle
model -- exact pad shapes, per-net clearances, hole-to-hole, everything
the braid's own disc-and-capsule model approximates. A layer mismatch
is solved by the search placing the via ("A* only for via placement").

Nothing here knows the board: inputs are a PCBData carrying every piece
of copper placed so far, a net, two points with their layers, a config,
and an optional BAND -- two functions of x giving the y interval the
connection may use, stamped as blocked cells, so a connection routed
early can never wander into the corridor a later neighbour needs.
"""
import math
import os
import sys
from typing import List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

import numpy as np  # noqa: E402
from kicad_parser import PCBData, Segment, Via  # noqa: E402
from routing_config import GridRouteConfig, GridCoord  # noqa: E402
from routing_utils import build_layer_map  # noqa: E402
from plane_pad_tap import make_local_window  # noqa: E402
from obstacle_map import (build_base_obstacle_map,  # noqa: E402
                          add_same_net_via_clearance,
                          add_same_net_pad_drill_via_clearance,
                          same_net_pad_via_keepout_cells)
from routing_context import _add_free_via_positions  # noqa: E402
from net_rescue import _fence_window, _result_escapes_window  # noqa: E402
from single_ended_routing import route_net_with_obstacles  # noqa: E402

Point = Tuple[float, float]


def make_config(pcb: PCBData, track: float, clearance: float,
                via_size: float, via_drill: float, grid_step: float = 0.05,
                **kw) -> GridRouteConfig:
    """A routing config for connections: the caller's geometry, the
    board's own copper layers, a fine grid (the trunk is drawn at
    arbitrary angles, so the exit points are not on any coarse grid)."""
    layers = list(pcb.board_info.copper_layers or ['F.Cu', 'B.Cu'])
    return GridRouteConfig(track_width=track, clearance=clearance,
                           via_size=via_size, via_drill=via_drill,
                           grid_step=grid_step, layers=layers, **kw)


def _band_cells(coord: GridCoord, window: PCBData, band,
                layers: List[str], slack: float) -> np.ndarray:
    """Every window cell outside the band, as an (N, 3) int32 array for
    add_blocked_cells_batch.

    `band` is a callable band(xs, ys, layer_name) -> bool mask of shape
    (len(xs), len(ys)), True where the lane may go; a layer the band
    closes everywhere is how a caller REQUIRES the other layer."""
    x0, y0, x1, y1 = window.board_info.board_bounds
    gx0, gy0 = coord.to_grid(x0, y0)
    gx1, gy1 = coord.to_grid(x1, y1)
    gxs = np.arange(gx0, gx1 + 1)
    gys = np.arange(gy0, gy1 + 1)
    xs = np.array([coord.to_float(int(g), 0)[0] for g in gxs])
    ys = np.array([coord.to_float(0, int(g))[1] for g in gys])
    # the rows built int32 a strip of columns at a time, in the order one
    # nonzero over the whole mask gave them (layer, then gx, then gy):
    # the int64 index pair, stack and concatenation of 3.1M cells outside
    # a band were ~200 MB of transient per attempt (README TODO 10)
    STRIP = 64
    parts = []
    for L, lname in enumerate(layers):
        ok = np.asarray(band(xs, ys, lname), dtype=bool)
        for i in range(0, len(gxs), STRIP):
            bi, bj = np.nonzero(~ok[i:i + STRIP])
            if len(bi):
                r = np.empty((len(bi), 3), dtype=np.int32)
                r[:, 0] = gxs[i + bi]
                r[:, 1] = gys[bj]
                r[:, 2] = L
                parts.append(r)
    if not parts:
        return np.zeros((0, 3), dtype=np.int32)
    return np.concatenate(parts)
    # vectorized over gy (the pure-Python double loop was 3.5s of an
    # 18s braid); the per-gx fn(x) and to_grid calls are kept
    # CALL-FOR-CALL identical to the loop they replace, so the cell
    # SET is bit-identical -- only the assembly is numpy


VIRTUAL_NET = 10 ** 7      # foreign net id for virtual copper (no such net)


def connect(pcb: PCBData, net_id: int, a: Point, a_layer: str,
            b: Point, b_layer: str, cfg: GridRouteConfig,
            band=None, margin: float = 1.0,
            band_slack: float = 0.0,
            virtual: Optional[List[Tuple[Point, Point, str]]] = None,
            window_pts: Optional[List[Point]] = None,
            virtual_vias: Optional[List[Point]] = None,
            b_alts: Optional[List[Tuple[float, float, str]]] = None,
            report: Optional[dict] = None,
            soft: Optional[List[Tuple[Point, Point, str, float]]] = None,
            soft_vias: Optional[List[Tuple[float, float, float]]] = None,
            soft_cost: float = 5.0,
            ) -> Optional[Tuple[List[Segment], List[Via]]]:
    """Route `net_id` from the copper end at `a` (on `a_layer`) to the
    copper end at `b` (on `b_layer`).

    `pcb` must carry every piece of copper placed so far -- the trunk of
    every net, the stubs, the connections already made -- because that is
    what the connection is routed against. Returns the new (segments,
    vias), NOT yet appended to `pcb`, or None when no route exists inside
    the window (the caller decides what a refusal means).

    `band`: (lo(x), hi(x)) in board mm, the y interval the connection may
    occupy (either side None), or {layer: fn(x) -> (lo, hi)} per layer
    with lo > hi closing that layer at that x -- which is how a caller
    REQUIRES a layer somewhere. `virtual`: copper that does not exist
    yet but will -- (p, q, layer) centrelines of lanes not routed yet --
    stamped as foreign obstacles so a via is never placed where a later
    lane must pass. `margin`: how far the search window extends past
    the bounding box of the two points -- and of `window_pts`, the
    planned path, when the lane goes somewhere the two points' box
    does not cover (round the far side of an array). `report`: a dict
    that a REFUSAL fills with the search's blocked frontier -- the cells
    the A* tried to expand into and found blocked (`blocked`, absolute
    grid (gx, gy, layer)), the window it searched (`window`) and the
    config (`cfg`) -- what a blocker analysis needs to name the copper
    that boxed the net. `soft`: copper that is a PRICE, not a wall --
    (p, q, layer, width) centrelines and `soft_vias` (x, y, size) whose
    clearance footprint costs `soft_cost` mm-equivalent per cell instead
    of being blocked, so a search through it finds the path that
    crosses the FEWEST such pieces (a min-cut probe); the caller keeps
    that copper OUT of `pcb` and never adds the probe's path.
    """
    coord = GridCoord(cfg.grid_step)
    layer_map = build_layer_map(cfg.layers)
    if a_layer not in layer_map or b_layer not in layer_map:
        raise ValueError(f'layer not routable: {a_layer} / {b_layer}')
    if os.environ.get('MEM_TRACE') == '1':
        import resource as _res

        def _m(tag):
            print(f'      mem {tag}: rss<={_res.getrusage(_res.RUSAGE_SELF).ru_maxrss / 1048576:.0f} MB', flush=True)
    else:
        def _m(tag):
            pass

    pts = [a, b] + list(window_pts or [])
    bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
    by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    half = max(bx1 - bx0, by1 - by0) / 2 + margin
    window = make_local_window(pcb, cx, cy, half)
    if not window.board_info.board_bounds:
        return None
    if virtual:
        w = cfg.track_width
        window.segments = list(window.segments) + [
            Segment(p[0], p[1], q[0], q[1], w, layer, VIRTUAL_NET)
            for (p, q, layer) in virtual if layer in layer_map]
    if virtual_vias:
        # vias that do not exist yet but will: a point a later lane
        # must change layer at (the corner where it turns onto its
        # exit leg), stamped as a foreign via so this connection
        # keeps a via's clearance from it -- a track's band edge is
        # exactly a via's clearance from the neighbour's
        # centreline, so a lane hugging its band edge there left
        # the neighbour's corner no legal via site (K19 SCAS)
        window.vias = list(window.vias) + [
            Via(p[0], p[1], cfg.via_size, cfg.via_drill,
                list(cfg.layers), VIRTUAL_NET) for p in virtual_vias]

    # static_base: the #422 static-bitmap stamp path -- engine-
    # documented byte-identical, hasattr-guarded, and measured
    # ~2x on the cold/large windows the margin-escalated retries
    # build (19.9 -> 10.5 ms; warm small windows equal)
    _m(f'window {len(window.segments)} segs {len(window.vias)} vias '
       f'{(window.board_info.board_bounds[2] - window.board_info.board_bounds[0]) / cfg.grid_step:.0f}x'
       f'{(window.board_info.board_bounds[3] - window.board_info.board_bounds[1]) / cfg.grid_step:.0f} cells')
    obstacles = build_base_obstacle_map(window, cfg, [net_id],
                                        static_base=True)
    _m('base map')
    _fence_window(obstacles, window, cfg)
    # the net's own barrels are free layer changes, and its own
    # via/drill spacing still applies (the rescue recipe, #470 and
    # the h2h guard)
    _add_free_via_positions(obstacles, window, [net_id], cfg)
    add_same_net_via_clearance(obstacles, window, net_id, cfg)
    add_same_net_pad_drill_via_clearance(obstacles, window, net_id, cfg)
    keep = same_net_pad_via_keepout_cells(pcb, net_id, cfg)
    if len(keep):
        obstacles.add_blocked_vias_batch(keep)
    _m('fence, free vias, keepouts')
    if band is not None and (isinstance(band, dict) or callable(band)
                             or band[0] is not None or band[1] is not None):
        cells = _band_cells(coord, window, band, list(cfg.layers),
                            band_slack)
        _m(f'band cells {len(cells)}')
        if len(cells):
            obstacles.add_blocked_cells_batch(cells)
        _m('band stamped')

    if soft or soft_vias:
        _stamp_soft(obstacles, coord, layer_map, cfg, soft or (),
                    soft_vias or (), soft_cost)
        _m('soft stamped')

    x0, y0, x1, y1 = window.board_info.board_bounds
    g0 = coord.to_grid(x0, y0)
    g1 = coord.to_grid(x1, y1)
    bounds = (g0[0], g0[1], g1[0], g1[1])
    ga = coord.to_grid(*a)
    gb = coord.to_grid(*b)
    sources = [(ga[0], ga[1], layer_map[a_layer], a[0], a[1])]
    targets = [(gb[0], gb[1], layer_map[b_layer], b[0], b[1])]
    # b_alts: ALTERNATIVE finish points (earlier stops on the dest
    # stub) -- the search terminates at whichever target it reaches
    # cheapest, so a lane that passes the pad no longer climbs to the
    # stub tip and pays the span twice (#622 berth overshoot)
    for (xx, yy, ll) in (b_alts or ()):
        if ll not in layer_map:
            continue
        gg = coord.to_grid(xx, yy)
        targets.append((gg[0], gg[1], layer_map[ll], xx, yy))
    result = route_net_with_obstacles(window, net_id, cfg, obstacles,
                                      bounds=bounds,
                                      sources_override=sources,
                                      targets_override=targets)
    _m('routed')
    if not result or result.get('failed'):
        if report is not None and result:
            report['blocked'] = (list(result.get('blocked_cells_forward') or [])
                                 + list(result.get('blocked_cells_backward') or []))
            report['window'] = window
            report['cfg'] = cfg
        return None
    if _result_escapes_window(result, window, cfg):
        return None
    return list(result.get('new_segments') or []), \
        list(result.get('new_vias') or [])




def _walk_capsule_cells(pts: np.ndarray, hw: int) -> Tuple[np.ndarray, np.ndarray]:
    """The cells of the union of the integer discs of radius `hw` (every
    (ex, ey) with ex*ex + ey*ey <= hw*hw) centred on each point of a
    Bresenham walk `pts` ((n, 2) int, connected, monotone), as (gx, gy)
    int32 arrays with NO cell twice. Exact: the walk visits every column
    between its ends and consecutive centres differ by at most one cell
    in each coordinate, so in any column the discs' intervals overlap or
    touch and their union is ONE span, [min over the taps of (the walk's
    lowest y in the tapped column - h), max of (highest + h)], with
    h(ex) = isqrt(hw*hw - ex*ex). One disc per point was 197 cells at
    hw 8 for every point of every priced lane -- 12 million rows for a
    K28 min-cut probe, 660 MB at the peak of the braid (README TODO 10)."""
    px = pts[:, 0].astype(np.int64)
    py = pts[:, 1].astype(np.int64)
    xmin = int(px.min())
    n = int(px.max()) - xmin + 1
    BIG = np.int64(1 << 40)
    ymin = np.full(n + 2 * hw, BIG, dtype=np.int64)
    ymax = np.full(n + 2 * hw, -BIG, dtype=np.int64)
    idx = px - xmin + hw
    np.minimum.at(ymin, idx, py)
    np.maximum.at(ymax, idx, py)
    taps = 2 * hw + 1
    ex = np.arange(-hw, hw + 1)
    h = np.array([math.isqrt(hw * hw - int(e) * int(e)) for e in ex], dtype=np.int64)
    # output column c = xmin - hw + i, i in [0, n + 2hw); tap j = hw - ex
    # reads the walk's column c - ex, which sits at padded index i + j
    from numpy.lib.stride_tricks import sliding_window_view
    pad_lo = np.concatenate([np.full(hw, BIG, dtype=np.int64), ymin, np.full(hw, BIG, dtype=np.int64)])
    pad_hi = np.concatenate([np.full(hw, -BIG, dtype=np.int64), ymax, np.full(hw, -BIG, dtype=np.int64)])
    lo = (sliding_window_view(pad_lo, taps) - h[None, ::-1]).min(axis=1)
    hi = (sliding_window_view(pad_hi, taps) + h[None, ::-1]).max(axis=1)
    cols = np.arange(xmin - hw, xmin - hw + n + 2 * hw, dtype=np.int64)
    lens = hi - lo + 1
    total = int(lens.sum())
    gx = np.repeat(cols, lens)
    starts = np.cumsum(lens) - lens
    gy = np.repeat(lo, lens) + (np.arange(total, dtype=np.int64) - np.repeat(starts, lens))
    return gx.astype(np.int32), gy.astype(np.int32)


def _stamp_soft(obstacles, coord: GridCoord, layer_map, cfg: GridRouteConfig,
                soft, soft_vias, soft_cost: float) -> None:
    """Price the clearance footprint of `soft` copper per cell on its
    layer (set_layer_proximity_batch, which keeps the MAX per cell), and
    of `soft_vias` on every layer. The footprint is the obstacle model's
    own: half the copper width + the clearance + half a track of the
    searching net -- a cell whose centre lies inside it is one the hard
    model would have blocked. Each piece's footprint is stamped ONCE per
    cell (_walk_capsule_cells): the map keeps the max per cell, so the
    map is the one a disc per walk point made, in a twentieth of the
    rows."""
    from bresenham_utils import walk_line
    cost = cfg.cell_cost(soft_cost)
    rows = []
    disks = {}

    def disk(r_grid):
        d = disks.get(r_grid)
        if d is None:
            rr = range(-r_grid, r_grid + 1)
            d = np.array([(ex, ey) for ex in rr for ey in rr
                          if ex * ex + ey * ey <= r_grid * r_grid],
                         dtype=np.int32)
            disks[r_grid] = d
        return d

    def stamp(li, gx, gy):
        r = np.empty((gx.size, 4), dtype=np.int32)
        r[:, 0] = li
        r[:, 1] = gx
        r[:, 2] = gy
        r[:, 3] = cost
        rows.append(r)
    for (p, q, layer, w) in soft:
        li = layer_map.get(layer)
        if li is None:
            continue
        hw = coord.to_grid_dist(w / 2 + cfg.clearance + cfg.track_width / 2)
        gx1, gy1 = coord.to_grid(p[0], p[1])
        gx2, gy2 = coord.to_grid(q[0], q[1])
        pts = np.asarray(list(walk_line(gx1, gy1, gx2, gy2)), dtype=np.int64)
        gx, gy = _walk_capsule_cells(pts, int(hw))
        stamp(li, gx, gy)
    for (x, y, size) in soft_vias:
        hw = coord.to_grid_dist(size / 2 + cfg.clearance + cfg.track_width / 2)
        gx0, gy0 = coord.to_grid(x, y)
        off = disk(hw)
        for li in range(len(cfg.layers)):
            stamp(li, gx0 + off[:, 0], gy0 + off[:, 1])
    if not rows:
        return
    # no np.unique(axis=0): the map keeps the MAX cost per cell, so a cell
    # stamped twice at the same cost is the same map, and the row sort
    # was 41 of the K41 braid's 172 profiled seconds (2026-09-08)
    arr = np.ascontiguousarray(np.concatenate(rows))
    obstacles.set_layer_proximity_batch(arr)


def seg_len(segs: List[Segment]) -> float:
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
               for s in segs)
