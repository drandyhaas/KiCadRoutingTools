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
from typing import Callable, List, Optional, Tuple

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
Band = Tuple[Optional[Callable[[float], float]],
             Optional[Callable[[float], float]]]


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


def tube_mask(xs: np.ndarray, ys: np.ndarray, pieces, layer: str
              ) -> np.ndarray:
    """The cells within a tube around a polyline: `pieces` is a list of
    (p, q, half_width, layers) segments, and the mask is True at
    (xs[i], ys[j]) -- shape (len(xs), len(ys)) -- where some piece
    whose `layers` contains `layer` passes within `half_width`. This is
    the general corridor shape: a lane that turns a corner, peels off
    to a face, or goes round the far side of an array is a polyline,
    not a function of x, and its corridor is the tube around it."""
    X = xs[:, None]
    Y = ys[None, :]
    m = np.zeros((len(xs), len(ys)), dtype=bool)
    for (p, q, half, layers) in pieces:
        if layer not in layers:
            continue
        dx, dy = q[0] - p[0], q[1] - p[1]
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            d2 = (X - p[0]) ** 2 + (Y - p[1]) ** 2
        else:
            t = ((X - p[0]) * dx + (Y - p[1]) * dy) / L2
            t = np.clip(t, 0.0, 1.0)
            d2 = (X - (p[0] + t * dx)) ** 2 + (Y - (p[1] + t * dy)) ** 2
        m |= d2 <= half * half
    return m


def _band_cells(coord: GridCoord, window: PCBData, band,
                layers: List[str], slack: float) -> np.ndarray:
    """Every window cell outside the band, as an (N, 3) int32 array for
    add_blocked_cells_batch.

    `band` is one of:
      * (lo(x), hi(x)) applied to every layer;
      * {layer_name: fn(x) -> (lo, hi)} -- a per-layer corridor, where
        lo > hi means the layer is closed at that x. That is how a
        caller REQUIRES a layer: close the other one. A layer absent
        from the dict is closed everywhere;
      * a callable band(xs, ys, layer_name) -> bool mask of shape
        (len(xs), len(ys)), True where the lane may go -- the general
        form, for a corridor that is not a function of x (a lane with
        a corner, a peel leg, a way round an array). `slack` is the
        callable's own business."""
    x0, y0, x1, y1 = window.board_info.board_bounds
    gx0, gy0 = coord.to_grid(x0, y0)
    gx1, gy1 = coord.to_grid(x1, y1)
    if callable(band) and not isinstance(band, (tuple, dict)):
        gxs = np.arange(gx0, gx1 + 1)
        gys = np.arange(gy0, gy1 + 1)
        xs = np.array([coord.to_float(int(g), 0)[0] for g in gxs])
        ys = np.array([coord.to_float(0, int(g))[1] for g in gys])
        parts = []
        for L, lname in enumerate(layers):
            ok = np.asarray(band(xs, ys, lname), dtype=bool)
            bi, bj = np.nonzero(~ok)
            if len(bi):
                parts.append(np.stack([gxs[bi], gys[bj],
                                       np.full(len(bi), L)], axis=1))
        if not parts:
            return np.zeros((0, 3), dtype=np.int32)
        return np.concatenate(parts).astype(np.int32)
    rows = []
    per_layer = isinstance(band, dict)
    for gx in range(gx0, gx1 + 1):
        x = coord.to_float(gx, 0)[0]
        for L, lname in enumerate(layers):
            if per_layer:
                fn = band.get(lname)
                if fn is None:
                    lo, hi = 1e9, -1e9
                else:
                    lo, hi = fn(x)
            else:
                lo_fn, hi_fn = band
                lo = lo_fn(x) if lo_fn is not None else -1e9
                hi = hi_fn(x) if hi_fn is not None else 1e9
            lo, hi = lo - slack, hi + slack
            if lo > hi:
                for gy in range(gy0, gy1 + 1):
                    rows.append((gx, gy, L))
                continue
            glo = coord.to_grid(0.0, lo)[1]
            ghi = coord.to_grid(0.0, hi)[1]
            for gy in range(gy0, gy1 + 1):
                if gy < glo or gy > ghi:
                    rows.append((gx, gy, L))
    if not rows:
        return np.zeros((0, 3), dtype=np.int32)
    return np.asarray(rows, dtype=np.int32)


VIRTUAL_NET = 10 ** 7      # foreign net id for virtual copper (no such net)


def connect(pcb: PCBData, net_id: int, a: Point, a_layer: str,
            b: Point, b_layer: str, cfg: GridRouteConfig,
            band=None, margin: float = 1.0,
            band_slack: float = 0.0, net_clearances: Optional[dict] = None,
            virtual: Optional[List[Tuple[Point, Point, str]]] = None,
            track_width: Optional[float] = None,
            verbose: bool = False,
            window_pts: Optional[List[Point]] = None
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
    does not cover (round the far side of an array).
    """
    coord = GridCoord(cfg.grid_step)
    layer_map = build_layer_map(cfg.layers)
    if a_layer not in layer_map or b_layer not in layer_map:
        raise ValueError(f'layer not routable: {a_layer} / {b_layer}')

    pts = [a, b] + list(window_pts or [])
    bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
    by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    half = max(bx1 - bx0, by1 - by0) / 2 + margin
    window = make_local_window(pcb, cx, cy, half)
    if not window.board_info.board_bounds:
        return None
    if virtual:
        w = track_width if track_width is not None else cfg.track_width
        window.segments = list(window.segments) + [
            Segment(p[0], p[1], q[0], q[1], w, layer, VIRTUAL_NET)
            for (p, q, layer) in virtual if layer in layer_map]

    obstacles = build_base_obstacle_map(window, cfg, [net_id],
                                        net_clearances=net_clearances)
    _fence_window(obstacles, window, cfg)
    # the net's own barrels are free layer changes, and its own via/drill
    # spacing still applies (the rescue recipe, #470 and the h2h guard)
    _add_free_via_positions(obstacles, window, [net_id], cfg)
    add_same_net_via_clearance(obstacles, window, net_id, cfg)
    add_same_net_pad_drill_via_clearance(obstacles, window, net_id, cfg)
    keep = same_net_pad_via_keepout_cells(pcb, net_id, cfg)
    if len(keep):
        obstacles.add_blocked_vias_batch(keep)
    if band is not None and (isinstance(band, dict) or callable(band)
                             or band[0] is not None or band[1] is not None):
        cells = _band_cells(coord, window, band, list(cfg.layers),
                            band_slack)
        if len(cells):
            obstacles.add_blocked_cells_batch(cells)

    x0, y0, x1, y1 = window.board_info.board_bounds
    g0 = coord.to_grid(x0, y0)
    g1 = coord.to_grid(x1, y1)
    bounds = (g0[0], g0[1], g1[0], g1[1])
    ga = coord.to_grid(*a)
    gb = coord.to_grid(*b)
    sources = [(ga[0], ga[1], layer_map[a_layer], a[0], a[1])]
    targets = [(gb[0], gb[1], layer_map[b_layer], b[0], b[1])]
    if verbose:
        print(f'  connect net {net_id}: ({a[0]:.2f},{a[1]:.2f}) {a_layer} -> '
              f'({b[0]:.2f},{b[1]:.2f}) {b_layer}  window '
              f'[{x0:.1f},{y0:.1f}]-[{x1:.1f},{y1:.1f}]')
    result = route_net_with_obstacles(window, net_id, cfg, obstacles,
                                      bounds=bounds,
                                      sources_override=sources,
                                      targets_override=targets)
    debug = verbose or os.environ.get('CONNECT_DEBUG')
    if not result or result.get('failed'):
        if debug and result:
            info = {k: v for k, v in result.items()
                    if k not in ('new_segments', 'new_vias', 'segments',
                                 'vias', 'path')}
            print(f'  connect net {net_id}: router failed: {info}')
        return None
    if _result_escapes_window(result, window, cfg):
        if debug:
            segs = result.get('new_segments') or []
            xs = [v for s in segs for v in (s.start_x, s.end_x)]
            ys = [v for s in segs for v in (s.start_y, s.end_y)]
            print(f'  connect net {net_id}: route escaped the window '
                  f'[{x0:.2f},{y0:.2f}]-[{x1:.2f},{y1:.2f}]: copper spans '
                  f'x [{min(xs):.2f},{max(xs):.2f}] y [{min(ys):.2f},{max(ys):.2f}]'
                  if segs else '  (no segments)')
        return None
    return list(result.get('new_segments') or []), \
        list(result.get('new_vias') or [])


def seg_len(segs: List[Segment]) -> float:
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
               for s in segs)
