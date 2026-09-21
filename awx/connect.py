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
                          same_net_pad_via_keepout_cells,
                          add_board_edge_obstacles)
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
    """Every window cell outside the band, as ONE (N, 3) int32 array
    (the strips of _band_cell_strips concatenated; probes use it)."""
    parts = list(_band_cell_strips(coord, window, band, layers, slack))
    if not parts:
        return np.zeros((0, 3), dtype=np.int32)
    return np.concatenate(parts)


def _band_cell_strips(coord: GridCoord, window: PCBData, band,
                      layers: List[str], slack: float):
    """Every window cell outside the band, as (n, 3) int32 arrays of
    (gx, gy, layer) for add_blocked_cells_batch / the static stamp, a
    strip of columns at a time, in the order one nonzero over the whole
    mask gave them (layer, then gx, then gy).

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
    # the rows built int32 a strip of columns at a time: the int64 index
    # pair, stack and concatenation of 3.1M cells outside a band were
    # ~200 MB of transient per attempt (README TODO 10)
    STRIP = 64
    for L, lname in enumerate(layers):
        ok = np.asarray(band(xs, ys, lname), dtype=bool)
        for i in range(0, len(gxs), STRIP):
            bi, bj = np.nonzero(~ok[i:i + STRIP])
            if len(bi):
                r = np.empty((len(bi), 3), dtype=np.int32)
                r[:, 0] = gxs[i + bi]
                r[:, 1] = gys[bj]
                r[:, 2] = L
                yield r
    # vectorized over gy (the pure-Python double loop was 3.5s of an
    # 18s braid); the per-gx fn(x) and to_grid calls are kept
    # CALL-FOR-CALL identical to the loop they replace, so the cell
    # SET is bit-identical -- only the assembly is numpy


VIRTUAL_NET = 10 ** 7      # foreign net id for virtual copper (no such net)

# ---- the base map, built once per (copper, net, virtual copper), cloned per
# connect (2026-09-18). A rescue ladder's rungs are the same net on the same
# copper with the same virtual lines, only a wider window each; the base
# map was rebuilt for every one (390 builds in a K41 braid, 24 in a probe
# braid, a quarter of the braid). A whole-board build of this board costs
# 97 ms and a clone under a millisecond (measured), so the base is built
# over the WHOLE board and the window's fence goes on the clone: inside
# the A* bounds the cells are the ones the window build stamped.
# MEASURED AND LEFT OFF (2026-09-18): built for every connect, a K41 braid
# went 42.7 -> 48.8 s; built only for a key asked twice (a ladder), 44.1 s
# with 109 whole-board builds serving 62 clones -- the ladders are too
# short (3-5 rungs, the first on its own window) for a 97 ms build to
# repay 20-50 ms window builds. Copper identical either way. Kept as
# CONNECT_MAP_CACHE=1 for a board or ladder shape where it would.
MAP_CACHE = os.environ.get('CONNECT_MAP_CACHE', '0') == '1'
_BASE = {}
_BASE_MAX = 24
_SEEN = {}          # key -> how many connects asked for it (the first builds its own window)
_BASE_STATS = {'hit': 0, 'miss': 0, 'single': 0}


def _copper_sig(pcb):
    return hash((tuple((s.start_x, s.start_y, s.end_x, s.end_y, s.layer, s.net_id, s.width)
                       for s in pcb.segments),
                 tuple((v.x, v.y, v.size, v.drill, v.net_id, tuple(v.layers)) for v in pcb.vias)))


def _cfg_sig(cfg):
    return (cfg.grid_step, cfg.track_width, cfg.clearance, cfg.via_size, cfg.via_drill,
            tuple(cfg.layers), getattr(cfg, 'board_edge_clearance', 0))


def _base_key(pcb, net_id, cfg, virtual, virtual_vias, layer_map):
    return (_copper_sig(pcb), net_id, _cfg_sig(cfg),
            tuple((tuple(p), tuple(q), L) for (p, q, L) in (virtual or ()) if L in layer_map),
            tuple(tuple(p) for p in (virtual_vias or ())))


def _base_map(pcb, net_id, cfg, virtual, virtual_vias, layer_map, key=None):
    """The whole-board map for `net_id` on this copper with this virtual
    copper: static obstacles, the net's free vias and same-net clearances,
    fenced at the board. Cached; the caller clones it. Built only when a
    key is asked for a SECOND time (a rescue ladder's rungs): measured, a
    whole-board build for every single connect made a K41 braid 14 percent
    slower, its 41 first-attempt lanes each paying 97 ms for a map a 20 ms
    window build would have served once."""
    if key is None:
        key = _base_key(pcb, net_id, cfg, virtual, virtual_vias, layer_map)
    b = _BASE.get(key)
    if b is not None:
        _BASE_STATS['hit'] += 1
        return b
    _BASE_STATS['miss'] += 1
    bb = pcb.board_info.board_bounds
    full = make_local_window(pcb, (bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2,
                             max(bb[2] - bb[0], bb[3] - bb[1]))     # clamps to the board: the board as a window
    if virtual:
        w = cfg.track_width + VIRT_SLACK
        full.segments = list(full.segments) + [
            Segment(p[0], p[1], q[0], q[1], w, layer, VIRTUAL_NET)
            for (p, q, layer) in virtual if layer in layer_map]
    if virtual_vias:
        full.vias = list(full.vias) + [
            Via(p[0], p[1], cfg.via_size, cfg.via_drill, list(cfg.layers), VIRTUAL_NET) for p in virtual_vias]
    obstacles = build_base_obstacle_map(full, cfg, [net_id], static_base=True)
    _fence_window(obstacles, full, cfg)
    _add_free_via_positions(obstacles, full, [net_id], cfg)
    add_same_net_via_clearance(obstacles, full, net_id, cfg)
    add_same_net_pad_drill_via_clearance(obstacles, full, net_id, cfg)
    keep = same_net_pad_via_keepout_cells(pcb, net_id, cfg)
    if len(keep):
        obstacles.add_blocked_vias_batch(keep)
    if len(_BASE) >= _BASE_MAX:
        _BASE.pop(next(iter(_BASE)))
    _BASE[key] = obstacles
    return obstacles
VIRT_SLACK = float(os.environ.get('BRAID_VIRT_SLACK', '0') or 0)   # extra width of a virtual stamp (mm); 0 = as ever


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
            own_ids: Optional[List[int]] = None,
            ) -> Optional[Tuple[List[Segment], List[Via]]]:
    """Route `net_id` from the copper end at `a` (on `a_layer`) to the
    copper end at `b` (on `b_layer`). `own_ids`: the nets whose copper is
    the searcher's OWN (exempt, free layer changes at their barrels) --
    default `[net_id]`; a pair's envelope lane names both legs.

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
        # BRAID_VIRT_SLACK (2026-09-15): a virtual line keeps a neighbour's
        # centreline exactly a track plus a clearance away, and the lane
        # the line stands for then needs exactly that from the neighbour
        # -- zero slack, no cell on an unlucky grid (K35 SDQ0 between
        # SDQM0's real copper and SDQ2's line: 0.000 mm free). The stamp
        # is widened by this much, so the lane inherits half of it a side.
        w = cfg.track_width + VIRT_SLACK
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
    _key = _base_key(pcb, net_id, cfg, virtual, virtual_vias, layer_map) if MAP_CACHE else None
    if MAP_CACHE:
        n_seen = _SEEN.get(_key, 0) + 1
        if len(_SEEN) > 4096:
            _SEEN.clear()
        _SEEN[_key] = n_seen
    if MAP_CACHE and n_seen >= 2 and not own_ids:
        # the whole-board base for this net and copper, cloned; the window's
        # own fence on top (what the per-window build stamped at its edge)
        obstacles = _base_map(pcb, net_id, cfg, virtual, virtual_vias, layer_map, key=_key).clone()
        add_board_edge_obstacles(obstacles, window, cfg)
        _m('base map (clone) + window fence')
    else:
        if MAP_CACHE:
            _BASE_STATS['single'] += 1
        own = list(own_ids) if own_ids else [net_id]
        obstacles = build_base_obstacle_map(window, cfg, own,
                                            static_base=True)
        _m('base map')
        _fence_window(obstacles, window, cfg)
        # the net's own barrels are free layer changes, and its own
        # via/drill spacing still applies (the rescue recipe, #470 and
        # the h2h guard)
        _add_free_via_positions(obstacles, window, own, cfg)
        for _oid in own:
            add_same_net_via_clearance(obstacles, window, _oid, cfg)
            add_same_net_pad_drill_via_clearance(obstacles, window, _oid, cfg)
            keep = same_net_pad_via_keepout_cells(pcb, _oid, cfg)
            if len(keep):
                obstacles.add_blocked_vias_batch(keep)
        _m('fence, free vias, keepouts')
    if band is not None and (isinstance(band, dict) or callable(band)
                             or band[0] is not None or band[1] is not None):
        # The band into the map's STATIC bitmap (#422's
        # add_static_blocked_cells_batch, which is_blocked ORs exactly as
        # a refcount entry: same source/target override, same tracking),
        # a strip at a time: a band is three to four million cells
        # outside the lane's corridor, never removed (the map lives one
        # attempt), and as refcount hash entries they were ~300 MB a
        # window -- the braid's largest remaining allocation after the
        # strips (README TODO 10). The bitmap holds them in a bit each.
        # An older binary without the static API takes the hash path.
        stamp = getattr(obstacles, 'add_static_blocked_cells_batch', None) \
            or obstacles.add_blocked_cells_batch
        n_band = 0
        for cells in _band_cell_strips(coord, window, band, list(cfg.layers),
                                       band_slack):
            n_band += len(cells)
            stamp(cells)
        _m(f'band stamped {n_band} cells')

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




def connect_pair(pcb: PCBData, p_id: int, n_id: int,
                 a_p: Point, a_n: Point, a_layer: str,
                 b_p: Point, b_n: Point, b_layer: str,
                 cfg: GridRouteConfig, band=None, margin: float = 1.0,
                 band_slack: float = 0.0,
                 virtual: Optional[List[Tuple[Point, Point, str]]] = None,
                 window_pts: Optional[List[Point]] = None,
                 virtual_vias: Optional[List[Point]] = None,
                 report: Optional[dict] = None,
                 gap: Optional[float] = None,
                 a_dir: Optional[Point] = None, b_dir: Optional[Point] = None,
                 lead: float = 0.15,
                 a_n_layer: Optional[str] = None, b_n_layer: Optional[str] = None,
                 a_conn: Optional[Point] = None, b_conn: Optional[Point] = None,
                 ) -> Optional[Tuple[List[Segment], List[Via]]]:
    """The pair form of `connect` (#622 pairs, 2026-09-20): route the
    DIFFERENTIAL PAIR (p_id, n_id) coupled from its two copper ends at
    `a_p` / `a_n` (both on `a_layer`) to its two ends at `b_p` / `b_n`
    (both on `b_layer`).

    THE DEFAULT (BRAID_PAIR_ROUTER=prod, 2026-09-20, Andy: "use the pose
    routing, constrained to a band") is the production pair router given
    CLEAN ENDS -- see _connect_pair_prod: coupled approach pieces at both
    ends, the escape directions forced, the centreline's map at the pair's
    extra clearance, the band stamped as for a single. BRAID_PAIR_ROUTER=
    envelope keeps the corridor's own way:
    ONE envelope lane -- a track as wide as both legs, vias as wide as two
    barrels side by side -- is routed by `connect` between a point `lead`
    mm in front of the two teeth (along `a_dir`) and one in front of the
    two berths (along `b_dir`), inside the same band, against the same
    virtual copper, with both legs' own copper exempt; then the envelope
    is SPLIT (pairs.split_envelope): P and N at the pair pitch either
    side of the centreline, mitred at the corners, each dive two barrels
    a via pitch apart with the legs jogging out and back, and short
    converge legs onto the real ends. P keeps the side its tooth is on;
    if its berth is on the other side the pair would have to cross, and
    the lane is refused (the joint plan owes consistent sides).

    BRAID_PAIR_ROUTER=prod is the production pair router
    (route_diff_pair_with_obstacles) on the same window and map -- kept
    for comparison; measured on the K34 bench its terminal connectors
    graze neighbouring stubs in this field and its setback search knows
    nothing of the corridor.

    `gap`: the P-to-N edge gap (default the config's diff_pair_gap,
    never below the clearance). Returns (segments, vias) of BOTH nets,
    not yet appended, or None."""
    import copy as _copy
    import pairs as _pairs
    layer_map = build_layer_map(cfg.layers)
    if a_layer not in layer_map or b_layer not in layer_map:
        raise ValueError(f'layer not routable: {a_layer} / {b_layer}')
    g = max(gap if gap is not None else cfg.diff_pair_gap, cfg.clearance)
    half = (cfg.track_width + g) / 2.0                 # a leg's offset from the centreline
    # a barrel's offset from the centreline: the barrels a via pitch apart
    # at least -- and far enough out that the OUTER leg's run past the
    # INNER barrel keeps its clearance when the lane turns at the dive by
    # up to 45 degrees (the distance is half + via_half * cos(turn); at
    # the bare via pitch a 22-degree turn measured 0.02 mm short)
    via_r = cfg.via_size / 2.0
    via_half = max((cfg.via_size + cfg.clearance) / 2.0,
                   (via_r + cfg.clearance + cfg.track_width / 2.0 - half) / 0.7071 + 0.005)
    mid_a, mid_b = _pairs.mid(a_p, a_n), _pairs.mid(b_p, b_n)
    if os.environ.get('BRAID_PAIR_ROUTER', 'prod') != 'envelope':
        return _connect_pair_prod(pcb, p_id, n_id, a_p, a_n, a_layer, b_p, b_n, b_layer,
                                  cfg, band, margin, band_slack, virtual, window_pts,
                                  virtual_vias, report, g, a_dir, b_dir, half,
                                  a_conn=a_conn, b_conn=b_conn)
    if a_dir is None:
        a_dir = _pairs._unit(mid_a, mid_b)
    if b_dir is None:
        b_dir = _pairs._unit(mid_b, mid_a)
    # a crossing (P's berth on the other side of the lane from its tooth)
    # is laid in the LEAD in front of the teeth: P's tooth lead is routed
    # by the real router under N, two vias, so that lead is long enough
    # for two barrels when the ends say a crossing is coming
    # (judged on the ESCAPE directions: leaving along a_dir, arriving
    # against b_dir. Judged on the chord between the two midpoints it
    # called for a crossing that was not there, and the long lead it
    # then took put the search's start under a neighbour's lane.)
    _sp0 = _pairs._cross(a_dir, (a_p[0] - mid_a[0], a_p[1] - mid_a[1])) >= 0
    _sp1 = _pairs._cross((-b_dir[0], -b_dir[1]), (b_p[0] - mid_b[0], b_p[1] - mid_b[1])) >= 0
    need_cross = _sp0 != _sp1
    long = max(lead, 4.0 * cfg.via_size)
    # a crossing is laid at whichever end has the room: the source lead
    # first, then the destination's (the long source lead ran into a
    # neighbour's virtual line on SDQS1 and the pair was refused outright)
    plans = [(long, lead, 'start'), (lead, long, 'end')] if need_cross else [(lead, lead, None)]
    for lead_a, lead_b, cross_at in plans:
        out = _envelope_pair(pcb, p_id, n_id, a_p, a_n, a_layer, b_p, b_n, b_layer, cfg,
                             band, margin, band_slack, virtual, window_pts, virtual_vias,
                             report, g, half, via_half, mid_a, mid_b, a_dir, b_dir,
                             lead_a, lead_b, cross_at, a_n_layer, b_n_layer)
        if out is not None:
            return out
    return None


def _envelope_pair(pcb, p_id, n_id, a_p, a_n, a_layer, b_p, b_n, b_layer, cfg,
                   band, margin, band_slack, virtual, window_pts, virtual_vias,
                   report, g, half, via_half, mid_a, mid_b, a_dir, b_dir,
                   lead_a, lead_b, cross_at, a_n_layer, b_n_layer):
    """One envelope attempt of connect_pair (see there): leads `lead_a` /
    `lead_b` in front of the teeth / berths, the crossing (if any) routed
    at `cross_at` ('start' | 'end' | None)."""
    import copy as _copy
    import pairs as _pairs
    a_pt = (mid_a[0] + a_dir[0] * lead_a, mid_a[1] + a_dir[1] * lead_a)
    b_pt = (mid_b[0] + b_dir[0] * lead_b, mid_b[1] + b_dir[1] * lead_b)
    # the APPROACH: the envelope is searched between two points a via
    # pitch further out along the escape directions, and a straight piece
    # joins each to its lead point -- so the lane leaves the teeth and
    # arrives at the berths ALONG their escapes (the search alone arrived
    # at the berths from the west, travelling east, and the converge legs
    # then crossed each other), and a dive at the very start stands as two
    # barrels side by side square in front of the teeth
    appr = round(cfg.via_size + cfg.clearance, 6)
    a_far = (a_pt[0] + a_dir[0] * appr, a_pt[1] + a_dir[1] * appr)
    b_far = (b_pt[0] + b_dir[0] * appr, b_pt[1] + b_dir[1] * appr)
    ecfg = _copy.copy(cfg)
    ecfg.track_width = round(2 * half + cfg.track_width, 6)
    ecfg.via_size = round(2 * via_half + cfg.via_size, 6)
    ecfg.via_drill = round(ecfg.via_size - (cfg.via_size - cfg.via_drill), 6)
    wp = list(window_pts or []) + [a_p, a_n, b_p, b_n, a_far, b_far]
    res = connect(pcb, p_id, a_far, a_layer, b_far, b_layer, ecfg, band=band,
                  margin=margin, band_slack=band_slack, virtual=virtual,
                  window_pts=wp, virtual_vias=virtual_vias, report=report,
                  own_ids=[p_id, n_id])
    if res is None:
        return None
    segs, vias = res
    segs = ([Segment(a_pt[0], a_pt[1], a_far[0], a_far[1], ecfg.track_width, a_layer, p_id)]
            + list(segs)
            + [Segment(b_far[0], b_far[1], b_pt[0], b_pt[1], ecfg.track_width, b_layer, p_id)])
    # the split, VALIDATED against itself: a hard turn at a dive can put a
    # leg inside the other's clearance (every intra-pair DRC the K36 chain
    # shipped); a failed split is retried with the barrels standing wider
    # and longer jogs, and a pair that cannot be split cleanly is refused
    out = None
    for widen, jog in ((1.0, 0.15), (1.25, 0.25), (1.5, 0.35)):
        cand = _pairs.split_envelope(segs, vias, a_p, a_n, b_p, b_n, a_pt, b_pt,
                                     half, via_half * widen, cfg.track_width, cfg.via_size,
                                     cfg.via_drill, p_id, n_id, cfg.layers, jog=jog,
                                     cross=(cross_at or True),
                                     tip_layers=(a_layer, a_n_layer or a_layer,
                                                 b_layer, b_n_layer or b_layer))
        if cand is None:
            return None
        why = _pairs.intra_ok(cand[0], cand[1], p_id, n_id, cfg.track_width, cfg.via_size, cfg.clearance)
        if why is None:
            out = cand
            break
        if report is not None:
            report['intra'] = why
    if out is None:
        return None
    segs2, vias2, p_start = out
    if p_start is not None:
        # the crossing: P's lead at the chosen end, routed against N's
        # laid copper (it dives under N: two vias)
        pcb2 = _copy.copy(pcb)
        pcb2.segments = list(pcb.segments) + segs2
        pcb2.vias = list(pcb.vias) + vias2
        if cross_at == 'end':
            r2 = connect(pcb2, p_id, p_start, b_layer, b_p, b_layer, cfg, band=None,
                         margin=1.0, virtual=virtual, virtual_vias=virtual_vias,
                         own_ids=[p_id])
        else:
            r2 = connect(pcb2, p_id, a_p, a_layer, p_start, a_layer, cfg, band=None,
                         margin=1.0, virtual=virtual, virtual_vias=virtual_vias,
                         own_ids=[p_id])
        if r2 is None:
            if report is not None:
                report['polarity'] = True
            return None
        segs2 = segs2 + list(r2[0])
        vias2 = vias2 + list(r2[1])
    return segs2, vias2


def _legs_clear(window, legs, own, cfg, virtual, layer_map, band=None, ends=None):
    """Why a connector's legs (Segments) are NOT clean against the
    window's foreign copper and the virtual lines -- a string -- or None.
    Segment-to-segment distances on a shared layer, foreign vias on
    every layer, foreign pads as discs (conservative), and, when `band`
    is a callable band(xs, ys, layer), the legs' END cells inside it
    (the pose search that follows is confined to the band)."""
    import pairs as _pairs
    clr = cfg.clearance
    tw = cfg.track_width
    own = set(own)
    dist = _pairs._seg_seg_dist

    def pt(x, y, L):
        return Segment(x, y, x, y, 0.0, L, 0)
    for s in legs:
        L = s.layer
        for o in window.segments:
            if o.net_id in own or o.layer != L:
                continue
            if dist(s, o) < clr + (s.width + o.width) / 2 - 1e-6:
                return f'leg on {L} grazes net {o.net_id} copper near ({s.start_x:.2f},{s.start_y:.2f})'
        for (p_, q_, vl) in (virtual or []):
            if vl != L:
                continue
            if dist(s, Segment(p_[0], p_[1], q_[0], q_[1], 0.0, L, 0)) < clr + (s.width + tw + VIRT_SLACK) / 2 - 1e-6:
                return f'leg on {L} grazes a virtual line near ({s.start_x:.2f},{s.start_y:.2f})'
        for v in window.vias:
            if v.net_id in own:
                continue
            if dist(s, pt(v.x, v.y, L)) < clr + (s.width + v.size) / 2 - 1e-6:
                return f'leg on {L} grazes net {v.net_id} via at ({v.x:.2f},{v.y:.2f})'
        for fp in window.footprints.values():
            for pad in fp.pads:
                if pad.net_id in own or pad.pad_type == 'np_thru_hole':
                    continue
                if not (pad.drill > 0 or L in pad.layers
                        or any(l_.startswith('*') and l_.endswith('.Cu') for l_ in pad.layers)):
                    continue
                r = math.hypot(pad.size_x, pad.size_y) / 2
                if dist(s, pt(pad.global_x, pad.global_y, L)) < clr + s.width / 2 + r - 1e-6:
                    # the disc is conservative: the rectangle's nearer edge decides
                    ex = max(abs(pad.global_x - (s.start_x + s.end_x) / 2) - pad.size_x / 2, 0.0)
                    ey = max(abs(pad.global_y - (s.start_y + s.end_y) / 2) - pad.size_y / 2, 0.0)
                    if math.hypot(ex, ey) < clr + s.width / 2 - 1e-6:
                        return f'leg on {L} grazes pad {fp.reference}.{pad.pad_number}'
    if band is not None and callable(band) and ends:
        for (x, y, L) in ends:
            ok = np.asarray(band(np.array([x]), np.array([y]), L), dtype=bool)
            if not bool(ok.ravel()[0]):
                return f'leg end ({x:.2f},{y:.2f}) on {L} is outside the band'
    return None


def _geo_connector(tip_p, tip_n, d, t, layer, half, cfg, p_id, n_id, lead=0.3, fwd=0.0):
    """A pair's connector from two tips to a coupled pose running along
    `t` (the planned lane's direction), when the teeth point ACROSS the
    lane (the escape direction `d` has a component across `t`): the leg
    whose tip is AHEAD along `t` runs straight along `t` from its tip;
    the leg behind first runs forward along `d` until it stands a pair
    pitch beyond the ahead leg's line, then turns along `t` -- passing
    the ahead tooth a pitch away, which is at least track + clearance
    (pairs.pitch). Both legs end `lead` past the later turn, at the pair
    pitch, the pose's direction `t`. Returns (segments, end_p, end_n) or
    None when the teeth are not across the lane (|d . n| < 0.5 -- the
    straight approach converges those), when the tips are too close
    along `t` to pass each other, or when a leg would have to run
    backwards into its tooth."""
    import pairs as _pairs
    n = _pairs._left(t)
    dn = d[0] * n[0] + d[1] * n[1]
    if abs(dn) < 0.5:
        return None
    mid = _pairs.mid(tip_p, tip_n)
    tw = cfg.track_width
    pitch = 2 * half

    def along(p):
        return (p[0] - mid[0]) * t[0] + (p[1] - mid[1]) * t[1]

    def u(p):
        return (p[0] - mid[0]) * d[0] + (p[1] - mid[1]) * d[1]
    tips = {'p': tip_p, 'n': tip_n}
    if abs(along(tip_p) - along(tip_n)) < tw + cfg.clearance + 0.02:
        return None
    ahead, behind = ('p', 'n') if along(tip_p) > along(tip_n) else ('n', 'p')
    u_a, u_b = u(tips[ahead]), u(tips[behind])
    # forward (along d) coordinates of the two runs: the behind leg a
    # pitch beyond the ahead leg, neither behind its own tip
    # `fwd`: both legs advance this far along d before the turn, so a
    # run along the row of tips clears the neighbouring tips
    U_a = max(u_a, u_b - pitch / abs(dn)) + fwd
    U_b = U_a + pitch / abs(dn)
    turns = {}
    for leg, U, u0 in ((ahead, U_a, u_a), (behind, U_b, u_b)):
        delta = U - u0
        if delta < -1e-6:
            return None
        tp = tips[leg]
        turns[leg] = (tp[0] + d[0] * delta, tp[1] + d[1] * delta) if delta > 1e-6 else tp
    a_end = max(along(turns[ahead]), along(turns[behind])) + lead
    segs = {}
    ends = {}
    for leg in ('p', 'n'):
        tp, tr = tips[leg], turns[leg]
        run = a_end - along(tr)
        if run < tw:
            return None
        end = (tr[0] + t[0] * run, tr[1] + t[1] * run)
        nid = p_id if leg == 'p' else n_id
        pieces = []
        if tr is not tp:
            pieces.append(Segment(tp[0], tp[1], tr[0], tr[1], tw, layer, nid))
        pieces.append(Segment(tr[0], tr[1], end[0], end[1], tw, layer, nid))
        segs[leg] = pieces
        ends[leg] = end
    # the ends stand a pitch apart across t by construction
    return segs['p'] + segs['n'], ends['p'], ends['n']


def _routed_connector(pcb, p_id, n_id, tip_p, tip_n, d, layer, far, cfg, half, via_half,
                      band, margin, band_slack, virtual, virtual_vias, lead=0.15):
    """One end's ROUTED connector (see _connect_pair_prod): the envelope
    centreline from `lead` mm in front of the tips (along `d`) to `far`,
    routed by `connect` on the tips' layer, split into the two legs
    (pairs.split_envelope, the tips as their starts), the far end left
    open. Returns (segments, vias, end_p, end_n, end_dir) -- the two legs'
    open ends and the direction they leave in -- or None."""
    import copy as _copy
    import pairs as _pairs
    mid = _pairs.mid(tip_p, tip_n)
    a_pt = (mid[0] + d[0] * lead, mid[1] + d[1] * lead)
    ecfg = _copy.copy(cfg)
    ecfg.track_width = round(2 * half + cfg.track_width, 6)
    ecfg.via_size = round(2 * via_half + cfg.via_size, 6)
    ecfg.via_drill = round(ecfg.via_size - (cfg.via_size - cfg.via_drill), 6)
    dbg = os.environ.get('BRAID_PAIR_DEBUG')
    res = None
    # the far point's layer: the one the BAND is open on there (the plan's
    # page past the fan-in; a berth whose lane arrives on the other layer
    # has its connector dive), the tips' layer first when both are open
    order = [layer] + [L for L in cfg.layers if L != layer]
    if band is not None and callable(band):
        def _open(L):
            try:
                return bool(np.asarray(band(np.array([far[0]]), np.array([far[1]]), L), dtype=bool).ravel()[0])
            except Exception:
                return True
        order = [L for L in order if _open(L)] + [L for L in order if not _open(L)]
    for far_layer in order:
        res = connect(pcb, p_id, a_pt, layer, far, far_layer, ecfg, band=band, margin=margin,
                      band_slack=band_slack, virtual=virtual, window_pts=[tip_p, tip_n, far],
                      virtual_vias=virtual_vias, own_ids=[p_id, n_id])
        if dbg:
            print(f"    connector envelope {a_pt[0]:.2f},{a_pt[1]:.2f} {layer} -> {far[0]:.2f},{far[1]:.2f} "
                  f"{far_layer} w={ecfg.track_width} via={ecfg.via_size}: "
                  + ('no route' if res is None else f'{len(res[0])} seg(s) {len(res[1])} via(s)'))
        if res is not None:
            break
    if res is None:
        return None
    segs, vias = res
    runs, left = _pairs._chain(segs, a_pt)
    if left or not runs:
        if dbg:
            print(f"    connector envelope: chain left {len(left)} segment(s) unchained")
        return None
    runs = [(L, _pairs._simplify(pts)) for (L, pts) in runs]
    runs = [(L, pts) for (L, pts) in runs if len(pts) >= 2]
    if not runs:
        return None
    # the open end: the last run's direction, the legs' offsets there
    L_end, pts = runs[-1]
    d_end = _pairs._unit(pts[-2], pts[-1])
    n_end = _pairs._left(d_end)
    d0 = _pairs._unit(runs[0][1][0], runs[0][1][1])
    s_p = 1.0 if _pairs._cross(d0, (tip_p[0] - a_pt[0], tip_p[1] - a_pt[1])) >= 0 else -1.0
    end_p = (pts[-1][0] + s_p * n_end[0] * half, pts[-1][1] + s_p * n_end[1] * half)
    end_n = (pts[-1][0] - s_p * n_end[0] * half, pts[-1][1] - s_p * n_end[1] * half)
    out = _pairs.split_envelope(segs, vias, tip_p, tip_n, end_p, end_n, a_pt, pts[-1],
                                half, via_half, cfg.track_width, cfg.via_size, cfg.via_drill,
                                p_id, n_id, cfg.layers, cross=False,
                                tip_layers=(layer, layer, L_end, L_end))
    if out is None:
        if dbg:
            print("    connector split: none")
        return None
    segs2, vias2, _ = out
    why = _pairs.intra_ok(segs2, vias2, p_id, n_id, cfg.track_width, cfg.via_size, cfg.clearance)
    if why is not None:
        if dbg:
            print(f"    connector split not clean: {why}")
        return None
    return segs2, vias2, end_p, end_n, d_end, L_end


def _connect_pair_prod(pcb, p_id, n_id, a_p, a_n, a_layer, b_p, b_n, b_layer,
                       cfg, band, margin, band_slack, virtual, window_pts,
                       virtual_vias, report, gap, a_dir=None, b_dir=None, half=None,
                       a_conn=None, b_conn=None, appr_scale=1.0, attempt=0):
    """connect_pair on the production pair router (see connect_pair), given
    CLEAN ENDS: a coupled APPROACH is laid first at each end -- each leg
    from its tip straight out along the escape direction for a via pitch,
    converging onto the pair pitch -- and the pose router runs between the
    approaches' ends. Its setback search and terminal connectors, which
    failed among the neighbouring teeth, then start a via pitch clear of
    them. The approach copper is returned with the route."""
    import copy as _copy
    import pairs as _pairs
    from routing_config import DiffPairNet
    from diff_pair_routing import route_diff_pair_with_obstacles
    coord = GridCoord(cfg.grid_step)
    layer_map = build_layer_map(cfg.layers)
    appr = round((cfg.via_size + cfg.clearance) * appr_scale, 6)
    a_p0, a_n0, a_layer0, b_p0, b_n0, b_layer0, a_dir0, b_dir0 = a_p, a_n, a_layer, b_p, b_n, b_layer, a_dir, b_dir
    approach = []
    connectors = []
    conn_vias = []
    connected = set()        # the ends whose routed connector stands
    # the validation window: the same copper the pose router sees (the
    # connectors and the approach are checked against it)
    vw = None
    if half is not None:
        pts0 = [a_p, a_n, b_p, b_n] + list(window_pts or [])
        vw = make_local_window(pcb, (min(p[0] for p in pts0) + max(p[0] for p in pts0)) / 2,
                               (min(p[1] for p in pts0) + max(p[1] for p in pts0)) / 2,
                               max(max(p[0] for p in pts0) - min(p[0] for p in pts0),
                                   max(p[1] for p in pts0) - min(p[1] for p in pts0)) / 2 + margin)
    if a_dir is not None and b_dir is not None and half is not None and (a_conn or b_conn):
        # BETTER CONNECTORS (Andy, 2026-09-20): the fan-in in front of the
        # teeth is where every neighbour's lane crosses, and a straight
        # approach plus the router's setback search found no clear start
        # there. So each end's connector is ROUTED: an envelope piece (one
        # track as wide as both legs, the corridor's own connect, inside
        # the band, both legs' copper exempt) from the tips to a point on
        # the planned lane past the fan-in, split into the two legs; the
        # pose router then runs between the connectors' ends, which stand
        # in the open corridor. Either connector that cannot be routed
        # falls back to the straight approach piece.
        via_r = cfg.via_size / 2.0
        via_half = max((cfg.via_size + cfg.clearance) / 2.0,
                       (via_r + cfg.clearance + cfg.track_width / 2.0 - half) / 0.7071 + 0.005)
        dbg = os.environ.get('BRAID_PAIR_DEBUG')
        for end, tip_p, tip_n, d, layer, far in (('a', a_p, a_n, a_dir, a_layer, a_conn),
                                                   ('b', b_p, b_n, b_dir, b_layer, b_conn)):
            if far is None:
                continue
            # `far`: a point, a (point, lane direction) pair, or a list of
            # such pairs (candidates in order: the lane's leaving run, then
            # the next)
            cands = far if isinstance(far, list) else [far]
            cands = [(f[0], f[1]) if (isinstance(f, tuple) and len(f) == 2 and isinstance(f[0], tuple))
                     else (f, None) for f in cands]
            far = cands[0][0]
            c = None
            for far_c, lane_dir in cands:
                if lane_dir is None or c is not None:
                    continue
                # GEOMETRIC first: the legs turned into the lane's direction
                # (no search), validated against the window's copper, the
                # virtual lines and the band
                gc = None
                for fwd in (cfg.clearance + cfg.track_width, 0.0):
                    gc = _geo_connector(tip_p, tip_n, d, lane_dir, layer, half, cfg, p_id, n_id, fwd=fwd)
                    if gc is None:
                        break
                    segs_g, ep, en = gc
                    why = _legs_clear(vw, segs_g, [p_id, n_id], cfg, virtual, layer_map, band=band,
                                      ends=[(ep[0], ep[1], layer), (en[0], en[1], layer)])
                    if dbg:
                        print(f"    connector {end} (geometric, along ({lane_dir[0]:.2f},{lane_dir[1]:.2f}), fwd {fwd:.3f}): "
                              f"{len(segs_g)} seg(s), ends ({ep[0]:.2f},{ep[1]:.2f})/({en[0]:.2f},{en[1]:.2f}): "
                              + (why or 'clean'))
                    if why is None:
                        c = (segs_g, [], ep, en, lane_dir, layer)
                        break
                if gc is None and dbg:
                    print(f"    connector {end} (geometric, along ({lane_dir[0]:.2f},{lane_dir[1]:.2f})): "
                          f"not this shape (teeth along the lane, or too close)")
            if c is None:
                c = _routed_connector(pcb, p_id, n_id, tip_p, tip_n, d, layer, far, cfg, half, via_half,
                                      band, margin, band_slack, virtual, virtual_vias)
            if os.environ.get('BRAID_PAIR_DEBUG'):
                print(f"    connector {end}: tips ({tip_p[0]:.2f},{tip_p[1]:.2f})/({tip_n[0]:.2f},{tip_n[1]:.2f}) "
                      f"d=({d[0]:.2f},{d[1]:.2f}) {layer} -> far ({far[0]:.2f},{far[1]:.2f}): "
                      + ('NONE (straight approach instead)' if c is None else
                         f"{len(c[0])} seg(s) {len(c[1])} via(s), ends ({c[2][0]:.2f},{c[2][1]:.2f})/"
                         f"({c[3][0]:.2f},{c[3][1]:.2f}) d_end=({c[4][0]:.2f},{c[4][1]:.2f}) {c[5]}"))
            if c is None:
                continue
            segs_c, vias_c, end_p, end_n, end_dir, end_layer = c
            connectors += segs_c
            conn_vias += vias_c
            if end == 'a':
                a_p, a_n, a_dir, a_layer = end_p, end_n, end_dir, end_layer
            else:
                b_p, b_n, b_dir, b_layer = end_p, end_n, end_dir, end_layer
            connected.add(end)
    if a_dir is not None and b_dir is not None and half is not None \
            and os.environ.get('BRAID_PAIR_APPROACH', '1') != '0':
        def _appr(tip_p, tip_n, d, layer, lane=None):
            mid = _pairs.mid(tip_p, tip_n)
            n = _pairs._left(d)
            sgn = 1.0 if _pairs._cross(d, (tip_p[0] - mid[0], tip_p[1] - mid[1])) >= 0 else -1.0
            sep = abs((tip_p[0] - tip_n[0]) * n[0] + (tip_p[1] - tip_n[1]) * n[1])
            why = None
            if lane is not None and vw is not None and sep > 2 * half + 2 * cfg.track_width:
                # ONTO THE PLANNED LANE: tips far apart along a comb, each leg
                # runs on along its escape until the two can bend onto the
                # lane's point `far`, at the pair pitch either side of it,
                # heading the lane's way (a straight convergence along the
                # escape ends in a neighbour's slot: zynq K47, both DQS pairs
                # refused in every order at 2.2 mm past their teeth)
                far, ld = lane
                nl = _pairs._left(ld)
                sg2 = 1.0 if _pairs._cross(ld, (tip_p[0] - far[0], tip_p[1] - far[1])) >= 0 else -1.0
                ep = (far[0] + sg2 * nl[0] * half, far[1] + sg2 * nl[1] * half)
                en = (far[0] - sg2 * nl[0] * half, far[1] - sg2 * nl[1] * half)
                run = 0.0
                while run <= 3.0 + 1e-9:
                    kp = (tip_p[0] + d[0] * run, tip_p[1] + d[1] * run)
                    kn = (tip_n[0] + d[0] * run, tip_n[1] + d[1] * run)
                    legs = []
                    if run > 1e-6:
                        legs += [Segment(tip_p[0], tip_p[1], kp[0], kp[1], cfg.track_width, layer, p_id),
                                 Segment(tip_n[0], tip_n[1], kn[0], kn[1], cfg.track_width, layer, n_id)]
                    legs += [Segment(kp[0], kp[1], ep[0], ep[1], cfg.track_width, layer, p_id),
                             Segment(kn[0], kn[1], en[0], en[1], cfg.track_width, layer, n_id)]
                    # the bends must not cross each other
                    if _pairs._seg_seg_dist(legs[-1], legs[-2]) >= cfg.track_width + cfg.clearance - 1e-6:
                        why = _legs_clear(vw, legs, [p_id, n_id], cfg, virtual, layer_map)
                        if why is None:
                            if os.environ.get('BRAID_PAIR_DEBUG'):
                                print(f"    approach: tips {sep:.2f} mm apart -- legs run {run:.1f} mm, then bend onto the "
                                      f"planned lane at ({far[0]:.2f},{far[1]:.2f}) heading ({ld[0]:.2f},{ld[1]:.2f})")
                            return legs, ep, en, ld
                    run += 0.1
                if os.environ.get('BRAID_PAIR_DEBUG'):
                    print(f"    approach: no clean bend onto the planned lane within 3 mm ({why}); converging along the escape")
            if sep > 2 * half + 2 * cfg.track_width and vw is not None:
                # TIPS FAR APART across the escape (a comb of teeth between
                # them, or two columns' balls): each leg runs on along its
                # escape until the two can converge at 30 degrees without
                # touching anything, then they converge to the pair pitch
                # -- the human's DQS0 at the zynq's U1: two surface teeth
                # 1.5 mm apart on a 0.32 mm comb, coupled after it
                conv = (sep - 2 * half) / 2 / math.tan(math.radians(30))
                run = 0.0
                while run <= 3.0 + 1e-9:
                    kp = (tip_p[0] + d[0] * run, tip_p[1] + d[1] * run)
                    kn = (tip_n[0] + d[0] * run, tip_n[1] + d[1] * run)
                    e = run + conv
                    ep = (mid[0] + d[0] * e + sgn * n[0] * half, mid[1] + d[1] * e + sgn * n[1] * half)
                    en = (mid[0] + d[0] * e - sgn * n[0] * half, mid[1] + d[1] * e - sgn * n[1] * half)
                    legs = []
                    if run > 1e-6:
                        legs += [Segment(tip_p[0], tip_p[1], kp[0], kp[1], cfg.track_width, layer, p_id),
                                 Segment(tip_n[0], tip_n[1], kn[0], kn[1], cfg.track_width, layer, n_id)]
                    legs += [Segment(kp[0], kp[1], ep[0], ep[1], cfg.track_width, layer, p_id),
                             Segment(kn[0], kn[1], en[0], en[1], cfg.track_width, layer, n_id)]
                    why = _legs_clear(vw, legs, [p_id, n_id], cfg, virtual, layer_map)
                    if why is None:
                        if os.environ.get('BRAID_PAIR_DEBUG'):
                            print(f"    approach: tips {sep:.2f} mm apart -- legs run {run:.1f} mm along the escape, "
                                  f"then converge over {conv:.2f} mm")
                        return legs, ep, en, d
                    run += 0.1
                if os.environ.get('BRAID_PAIR_DEBUG'):
                    print(f"    approach: tips {sep:.2f} mm apart -- no clean convergence within 3 mm ({why}); straight approach")
            end_p = (mid[0] + d[0] * appr + sgn * n[0] * half, mid[1] + d[1] * appr + sgn * n[1] * half)
            end_n = (mid[0] + d[0] * appr - sgn * n[0] * half, mid[1] + d[1] * appr - sgn * n[1] * half)
            return [Segment(tip_p[0], tip_p[1], end_p[0], end_p[1], cfg.track_width, layer, p_id),
                    Segment(tip_n[0], tip_n[1], end_n[0], end_n[1], cfg.track_width, layer, n_id)], end_p, end_n, d

        def _lane_of(conn):
            c0 = (conn[0] if isinstance(conn, list) else conn) if conn else None
            if isinstance(c0, tuple) and len(c0) == 2 and isinstance(c0[0], tuple):
                return (c0[0], c0[1])
            return None
        if 'a' not in connected:
            sa, a_p, a_n, a_dir = _appr(a_p, a_n, a_dir, a_layer, lane=_lane_of(a_conn))
            approach += sa
        if 'b' not in connected:
            sb, b_p, b_n, b_dir = _appr(b_p, b_n, b_dir, b_layer, lane=_lane_of(b_conn))
            approach += sb
    pts = [a_p, a_n, b_p, b_n] + list(window_pts or [])
    # ...and 1.5 mm past each end along its direction: a connector's ends
    # can stand beyond the planned lane's extent, and the pose search
    # (setbacks to 1.09 mm) must not meet the window's fence (K36 SDQS1:
    # every pose past the berth connector was a fence cell)
    for e_, dd_ in ((a_p, a_dir), (b_p, b_dir)):
        if dd_ is not None:
            pts.append((e_[0] + dd_[0] * 1.5, e_[1] + dd_[1] * 1.5))
    bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
    by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
    cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
    # (the window's half-size under its own name: it shadowed the pair's
    # half pitch and the map was built with an 11 mm extra clearance)
    whalf = max(bx1 - bx0, by1 - by0) / 2 + margin
    window = make_local_window(pcb, cx, cy, whalf)
    if not window.board_info.board_bounds:
        return None
    window.segments = list(window.segments) + connectors + approach
    window.vias = list(window.vias) + conn_vias
    if virtual:
        w = cfg.track_width + VIRT_SLACK
        window.segments = list(window.segments) + [
            Segment(p[0], p[1], q[0], q[1], w, layer, VIRTUAL_NET)
            for (p, q, layer) in virtual if layer in layer_map]
    if virtual_vias:
        window.vias = list(window.vias) + [
            Via(p[0], p[1], cfg.via_size, cfg.via_drill,
                list(cfg.layers), VIRTUAL_NET) for p in virtual_vias]
    own = [p_id, n_id]
    # the CENTRELINE's map carries the pair's EXTRA clearance -- half the
    # pair pitch, what the production batch gives its diff-pair map -- so
    # the legs generated either side of the centreline keep their own
    # clearance (the pose router validates the centreline cell only, and
    # at a single's clearance the offset legs grazed a stub by 0.047 mm)
    extra = (half if half is not None else (cfg.track_width + gap) / 2.0)
    obstacles = build_base_obstacle_map(window, cfg, own, extra, static_base=True)
    _fence_window(obstacles, window, cfg)
    _add_free_via_positions(obstacles, window, own, cfg)
    for nid in own:
        add_same_net_via_clearance(obstacles, window, nid, cfg)
        add_same_net_pad_drill_via_clearance(obstacles, window, nid, cfg)
        keep = same_net_pad_via_keepout_cells(pcb, nid, cfg)
        if len(keep):
            obstacles.add_blocked_vias_batch(keep)
    if band is not None and (isinstance(band, dict) or callable(band)
                             or band[0] is not None or band[1] is not None):
        stamp = getattr(obstacles, 'add_static_blocked_cells_batch', None) \
            or obstacles.add_blocked_cells_batch
        for cells in _band_cell_strips(coord, window, band, list(cfg.layers),
                                       band_slack):
            stamp(cells)
    if os.environ.get('BRAID_PAIR_DEBUG'):
        # why a pose is refused: the ladder's cells along each end's
        # direction, centre / P / N, on the map the router searches
        for lbl, m_, dd, L in (('a', _pairs.mid(a_p, a_n), a_dir, a_layer), ('b', _pairs.mid(b_p, b_n), b_dir, b_layer)):
            if dd is None:
                continue
            nn = _pairs._left(dd)
            li = layer_map[L]
            rows = []
            for sb in (0.27, 0.41, 0.54, 0.82, 1.09):
                cx_, cy_ = m_[0] + dd[0] * sb, m_[1] + dd[1] * sb
                cells = [coord.to_grid(cx_, cy_), coord.to_grid(cx_ + nn[0] * half, cy_ + nn[1] * half),
                         coord.to_grid(cx_ - nn[0] * half, cy_ - nn[1] * half)]
                try:
                    vmark = 'v' if not obstacles.is_via_blocked(cells[0][0], cells[0][1]) else '-'
                except Exception:
                    vmark = '?'
                if vmark == '-':
                    # what forbids a barrel here: the pair's own-pad keepout, its own vias, or the map
                    kc = 0
                    for nid_ in own:
                        kp = same_net_pad_via_keepout_cells(pcb, nid_, cfg)
                        if len(kp):
                            kp = np.asarray(kp)
                            kc += int(np.sum((np.abs(kp[:, 0] - cells[0][0]) <= 1) & (np.abs(kp[:, 1] - cells[0][1]) <= 1)))
                    ov = [v for v in window.vias if v.net_id in own and math.hypot(v.x - cx_, v.y - cy_) < 0.6]
                    vmark += f"(keepout {kc}, own vias {len(ov)})"
                
                other = [k_ for k_ in range(len(cfg.layers)) if k_ != li]
                omark = ''.join('X' if obstacles.is_blocked(cells[0][0], cells[0][1], k_) else '.' for k_ in other)
                rows.append(f"{sb:.2f}:" + ''.join('X' if obstacles.is_blocked(g[0], g[1], li) else '.' for g in cells)
                            + vmark + '/' + omark)
                for tag, g in zip(('c', 'P', 'N'), cells):
                    if not obstacles.is_blocked(g[0], g[1], li):
                        continue
                    x_, y_ = coord.to_float(g[0], g[1])
                    why = []
                    if band is not None and callable(band):
                        okb = np.asarray(band(np.array([x_]), np.array([y_]), L), dtype=bool)
                        if not bool(okb.ravel()[0]):
                            why.append('outside band')
                    near = []
                    for o in window.segments:
                        if o.net_id in own or o.layer != L:
                            continue
                        dd_ = _pairs._seg_seg_dist(Segment(x_, y_, x_, y_, 0.0, L, 0), o)
                        if dd_ < 0.45:
                            near.append((dd_, f'{"virtual" if o.net_id == VIRTUAL_NET else "net " + str(o.net_id)} '
                                              f'seg {dd_:.2f}'))
                    for v in window.vias:
                        if v.net_id in own:
                            continue
                        dd_ = math.hypot(v.x - x_, v.y - y_)
                        if dd_ < 0.45:
                            near.append((dd_, f'net {v.net_id} via {dd_:.2f}'))
                    for fp in window.footprints.values():
                        for pad in fp.pads:
                            if pad.net_id in own:
                                continue
                            dd_ = math.hypot(pad.global_x - x_, pad.global_y - y_) - max(pad.size_x, pad.size_y) / 2
                            if dd_ < 0.4:
                                near.append((dd_, f'pad {fp.reference}.{pad.pad_number} {dd_:.2f}'))
                    near.sort()
                    why += [w for _, w in near[:2]]
                    rows.append(f"[{tag}@{sb:.2f}: {', '.join(why) or 'no copper within 0.45 -- static stamp?'}]")
            print(f"    map probe {lbl} from ({m_[0]:.2f},{m_[1]:.2f}) along ({dd[0]:.2f},{dd[1]:.2f}) {L} [centre,P,N]: "
                  + ' '.join(rows))
    if report is not None:
        report['pieces'] = list(connectors) + list(approach)
        report['piece_vias'] = list(conn_vias)
        report['ends'] = (a_p, a_n, a_layer, b_p, b_n, b_layer, a_dir, b_dir)
    pcfg = _copy.copy(cfg)
    pcfg.diff_pair_gap = gap
    pcfg.gnd_via_enabled = False          # no return vias inside a bus lane
    pcfg.diff_pair_intra_match = False
    names = {i: n.name for i, n in pcb.nets.items()}
    dp = DiffPairNet(base_name=names.get(p_id, str(p_id)).rstrip('PN_+-'),
                     p_net_id=p_id, n_net_id=n_id,
                     p_net_name=names.get(p_id), n_net_name=names.get(n_id))
    gp, gn = coord.to_grid(*a_p), coord.to_grid(*a_n)
    hp, hn = coord.to_grid(*b_p), coord.to_grid(*b_n)
    # the router's endpoint tuple: grid P, grid N, layer index, then the
    # exact mm coordinates of P and N (its stub-proximity exemptions)
    src = (gp[0], gp[1], gn[0], gn[1], layer_map[a_layer], a_p[0], a_p[1], a_n[0], a_n[1])
    tgt = (hp[0], hp[1], hn[0], hn[1], layer_map[b_layer], b_p[0], b_p[1], b_n[0], b_n[1])
    # the ESCAPE DIRECTIONS forced at both ends (the router's own sweep
    # picked setbacks among the neighbouring teeth); BRAID_PAIR_APPROACH=0
    # drops the approach pieces (an experiment knob)
    result = route_diff_pair_with_obstacles(window, dp, pcfg, obstacles,
                                            endpoints=(src, tgt),
                                            swap_allowed_ends=(),
                                            forced_source_dir=a_dir,
                                            forced_target_dir=b_dir)
    if not result or result.get('failed'):
        blocked = (list((result or {}).get('blocked_cells_forward') or [])
                   + list((result or {}).get('blocked_cells_backward') or []))
        if result and not blocked and attempt < 2:
            # a refusal with NO frontier is the router's own check on the
            # pose it chose -- its connectors crossing the legs, or an
            # intra-pair graze ("rejecting the pair rather than shipping a
            # short") -- not a wall: the same ends are tried again with
            # STRAIGHT approaches, twice then three times as long, which
            # puts the pose further out and square to the tips (zynq K44
            # DQS1 at its berth, 2026-09-20)
            if os.environ.get('BRAID_PAIR_DEBUG'):
                print(f"    pair route refused with no frontier -- retrying with straight approaches x{attempt + 2}")
            return _connect_pair_prod(pcb, p_id, n_id, a_p0, a_n0, a_layer0, b_p0, b_n0, b_layer0,
                                      cfg, band, margin, band_slack, virtual, window_pts,
                                      virtual_vias, report, gap, a_dir0, b_dir0, half,
                                      a_conn=None, b_conn=None, appr_scale=float(attempt + 2),
                                      attempt=attempt + 1)
        if report is not None and result:
            report['blocked'] = blocked
            report['window'] = window
            report['cfg'] = cfg
        return None
    if _result_escapes_window(result, window, cfg):
        return None
    if not result.get('new_segments'):
        # a probe that stopped at the source returns no failure flag and no
        # copper -- and a lane of approach pieces alone "landed" (K36 SDQS0)
        if attempt < 2 and (a_conn is not None or b_conn is not None):
            # ...most often because a LANE-GUIDED connector's pose has no
            # room at its end (zynq K47 DQS0: the planned lane hooks into
            # the berths from the north, 0.3 mm from them, between a
            # reserved lane and a cap; the same pair lands at once along
            # the berths' own direction): the ends are tried again with
            # the plain approaches along the escape and arrival directions
            if os.environ.get('BRAID_PAIR_DEBUG'):
                print("    pair route made no copper -- retrying with the plain approaches at both ends")
            return _connect_pair_prod(pcb, p_id, n_id, a_p0, a_n0, a_layer0, b_p0, b_n0, b_layer0,
                                      cfg, band, margin, band_slack, virtual, window_pts,
                                      virtual_vias, report, gap, a_dir0, b_dir0, half,
                                      a_conn=None, b_conn=None, appr_scale=appr_scale,
                                      attempt=attempt + 1)
        if report is not None:
            report['empty'] = True
        return None
    return connectors + approach + list(result.get('new_segments') or []), \
        conn_vias + list(result.get('new_vias') or [])


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
