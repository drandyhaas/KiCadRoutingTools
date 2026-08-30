"""
Single-ended net routing functions.

Routes individual nets using A* pathfinding on a grid obstacle map.
"""
from __future__ import annotations

import env_knobs
import math
import time
import numpy as np
from contextlib import contextmanager
from typing import Dict, List, Optional, Set, Tuple
from terminal_colors import YELLOW, GREEN, RESET

from dataclasses import replace
from kicad_parser import PCBData, Segment, Via, pad_is_plated_through
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map, pad_rect_halfspan
from connectivity import (
    get_net_endpoints,
    get_multipoint_net_pads,
    get_copper_connected_terminal_groups,
    compute_component_mst_edges,
)
from obstacle_map import get_same_net_through_hole_positions
from bresenham_utils import walk_line
from geometry_utils import simplify_path

# Import Rust router
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads

try:
    from grid_router import GridObstacleMap, GridRouter
except ImportError:
    GridObstacleMap = None
    GridRouter = None

def _unblock_debug() -> bool:
    """KICAD_UNBLOCK_DEBUG, from the read-once env_knobs cache.

    History: a module-level constant frozen at import (stale for a GUI process
    that set the var after load), then a per-call os.environ read (#382 E10) --
    which put an environ lookup inside hot per-via-record emit paths. The
    cache is the resolution of that tension: hot paths read a Python
    attribute, and an in-process setter calls env_knobs.refresh() to be seen
    (children spawned with a modified environment re-read at import).
    """
    return env_knobs.UNBLOCK_DEBUG


# Pads farther than this from a query point can't change a neck/merge decision:
# every caller thresholds the distance against a clearance/track-width margin that
# is well under a millimetre. Windowing to this radius keeps the result identical
# where it matters (the exact min for anything closer) while letting the numpy
# kernels skip the bulk of the board's pads. Generous (~10x the largest realistic
# margin) so routing stays byte-for-byte identical.
_FOREIGN_PAD_WINDOW = 5.0  # mm


def _pad_corner_radius(pad):
    """Corner radius that turns a pad's local rect (half_x, half_y) into an
    accurate rounded-rect model for the foreign-pad distance kernels:

      * circle / oval -> min(half) : the rect becomes a capsule/circle, so the
        distance to a round BGA ball is exact (not the rect corner, which sticks
        out ~half the ball diameter past the copper and manufactures phantom
        pad grazes -- butterstick DQ5 vs the round DQ6 ball).
      * roundrect      -> roundrect_rratio * min(size) : KiCad's rounded corners.
      * rect / custom  -> 0 : plain bounding rect (custom pads keep the
        conservative over-approximation; their real outline is a polygon).

    Valid for rotated pads too: the kernels evaluate the SDF in the pad's LOCAL
    frame (rotating each query point by -rect_rotation), so the radius applies
    to the true tilted outline. The old board-axis fallback kept the UNROTATED
    rect for tilted pads -- which under-covers a tilted oval by up to half its
    long axis (issue #356, bfo9000 SW_A2.2 at -48 deg) -- it was never a bbox."""
    sx = pad.size_x; sy = pad.size_y
    shape = getattr(pad, 'shape', 'rect')
    if shape in ('circle', 'oval'):
        return min(sx, sy) / 2.0
    if shape == 'roundrect':
        rr = getattr(pad, 'roundrect_rratio', 0.0) or 0.0
        return rr * min(sx, sy)
    return 0.0


def _foreign_pad_arrays(pcb_data, layer):
    """Cached per-layer numpy arrays (net_id, cx, cy, half_x, half_y, corner_r,
    rot_cos, rot_sin, ext_x, ext_y) for every pad on `layer`. Pads never change
    during a route, so this is built once per board and reused across the
    millions of foreign-pad distance queries (the terminal graze/merge checks).
    `corner_r` makes the distance kernels model the pad as a rounded rect
    (accurate for circle/oval/roundrect, exact rect at r=0). `rot_cos`/`rot_sin`
    are the pad's rect_rotation (issue #356: kernels rotate query points into the
    pad's local frame, so tilted pads use their TRUE outline -- the old
    board-axis rect under-covered a tilted oval by up to half its long axis and
    the nudge passes re-bent tracks INTO the pad). `ext_x`/`ext_y` are the
    global-axis half-extents of the (possibly tilted) rect for windowing; equal
    to half_x/half_y for axis-aligned pads. Returns ten parallel arrays."""
    # #665: version the cache on the pads_by_net IDENTITY (+ pad count).
    # The docstring's "pads never change" was true of the FULL board, but a
    # windowed shallow copy (plane_pad_tap) REBINDS pads_by_net to a subset
    # while SHARING this cache dict -- its per-layer rebuild then poisoned
    # the parent's cache with window-only pad arrays, and later full-board
    # clearance checks (the cleanup passes' clears()) accepted copper
    # STRAIGHT THROUGH the invisible pads (the 24 pad-segment violations on
    # the iteration boards). Mirror the seg/via caches: signature tuple +
    # setattr REBIND on mismatch, so each pcb_data view owns its arrays.
    _sig = (id(pcb_data.pads_by_net),
            sum(len(v) for v in pcb_data.pads_by_net.values()))
    cache = getattr(pcb_data, '_foreign_pad_arr_cache', None)
    if cache is None or not isinstance(cache, tuple) or cache[0] != _sig:
        cache = (_sig, {})
        pcb_data._foreign_pad_arr_cache = cache
    cache = cache[1]
    arr = cache.get(layer)
    if arr is None:
        nids, cx, cy, hx, hy, cr = [], [], [], [], [], []
        rc, rs, ex, ey, lc = [], [], [], [], []
        custom = []  # (net_id, pad) -- exact-outline pads handled per-pad
        for nid, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if layer in pad.layers or '*.Cu' in pad.layers:
                    if getattr(pad, 'polygons', None):
                        # CUSTOM pad with real polygon outline(s): the rounded
                        # rect model would use its bounding box, which both
                        # over-blocks gating (phantom grazes, #232 family) and
                        # blinds remediation -- the re-bend/prune passes saw a
                        # 59um "deficit" where KiCad measures 1.5um and could
                        # not find any clearing bend (orangecrab U9). These
                        # pads are rare; they get the exact check_drc distance
                        # in the query kernels instead.
                        custom.append((nid, pad))
                        continue
                    nids.append(nid)
                    cx.append(pad.global_x); cy.append(pad.global_y)
                    half_x = pad.size_x / 2.0; half_y = pad.size_y / 2.0
                    hx.append(half_x); hy.append(half_y)
                    cr.append(_pad_corner_radius(pad))
                    rot = getattr(pad, 'rect_rotation', 0.0) or 0.0
                    if rot:
                        c = math.cos(math.radians(rot)); s = math.sin(math.radians(rot))
                        rc.append(c); rs.append(s)
                        ex.append(abs(half_x * c) + abs(half_y * s))
                        ey.append(abs(half_x * s) + abs(half_y * c))
                    else:
                        rc.append(1.0); rs.append(0.0)
                        ex.append(half_x); ey.append(half_y)
                    lc.append(getattr(pad, 'local_clearance', 0.0) or 0.0)
        arr = (np.asarray(nids, dtype=np.int64), np.asarray(cx), np.asarray(cy),
               np.asarray(hx), np.asarray(hy), np.asarray(cr),
               np.asarray(rc), np.asarray(rs), np.asarray(ex), np.asarray(ey),
               np.asarray(lc), custom)
        cache[layer] = arr
    return arr


def _custom_pad_min_dist(custom, net_id, pts, base_clearance=None):
    """Exact min edge distance from sample points to the CUSTOM pads of other
    nets (check_drc.point_to_pad_distance -- the model kicad-cli agrees with to
    ~0.1um). Windowed by each pad's bbox + _FOREIGN_PAD_WINDOW; same
    base_clearance local-override adjustment as the vectorized kernels."""
    if not custom:
        return 1e9
    from check_drc import point_to_pad_distance
    R = _FOREIGN_PAD_WINDOW
    best = 1e9
    for nid, pad in custom:
        if nid == net_id:
            continue
        ex = pad.size_x / 2.0
        ey = pad.size_y / 2.0
        adj = 0.0
        if base_clearance is not None:
            adj = max((getattr(pad, 'local_clearance', 0.0) or 0.0) - base_clearance, 0.0)
        # Branch-and-bound prune (exact-result-preserving): the polygons are
        # stored in GLOBAL coordinates, so the distance from a sample point to
        # the polygon's bounding BOX is a valid lower bound on its edge
        # distance (a bbox is tighter than a bounding radius on the elongated
        # connector/thermal polygons that dominate real boards). Points are
        # visited lowest-bound first; once the bound can no longer beat
        # `best`, no later point can either. Cuts the exact polygon walk from
        # every windowed sample point (the 5mm window is generous) to the few
        # that matter -- it dominated the #536 smoothing pass.
        bb = getattr(pad, '_poly_bbox', None)
        if bb is None:
            vs = [v for poly in (getattr(pad, 'polygons', None) or ()) for v in poly]
            if vs:
                bb = (min(v[0] for v in vs), min(v[1] for v in vs),
                      max(v[0] for v in vs), max(v[1] for v in vs))
            else:
                bb = (pad.global_x - ex, pad.global_y - ey,
                      pad.global_x + ex, pad.global_y + ey)
            try:
                pad._poly_bbox = bb
            except Exception:
                pass
        cand = []
        for (px, py) in pts:
            if abs(px - pad.global_x) > R + ex or abs(py - pad.global_y) > R + ey:
                continue
            lo = math.hypot(max(bb[0] - px, px - bb[2], 0.0),
                            max(bb[1] - py, py - bb[3], 0.0))
            cand.append((lo, px, py))
        cand.sort()
        for lo, px, py in cand:
            if lo - adj >= best:
                break
            d = point_to_pad_distance(px, py, pad) - adj
            if d < best:
                best = d
    return best


def _pt_foreign_pad_dist(pcb_data, net_id, x, y, layer, base_clearance=None,
                         net_clearances=None):
    """Min edge distance from point (x,y) on `layer` to any pad of a DIFFERENT net.
    The pad is modelled as a rounded rect (accurate for circle/oval/roundrect,
    exact rect at corner_r=0), so a round BGA ball is not over-approximated by its
    bounding-box corner. Vectorized + windowed (see _FOREIGN_PAD_WINDOW); exact
    for any distance within the window, which is all the callers ever look at.

    `base_clearance` (#326 B6): the clearance the CALLER will compare against
    (its config.clearance). When given, each pad's local/footprint clearance
    override above that base is SUBTRACTED from its distance, so the caller's
    unchanged `dist >= base + X` threshold enforces the pad's own requirement
    (an "effective distance"). Overrides are bounded well below the 5mm
    window, so windowing stays exact.

    `net_clearances` (#436, net_id -> class clearance mm): folds each foreign
    pad's netclass EXCESS over `base_clearance` into the effective distance the
    same way local_clearance is (whichever is larger wins), so a caller check
    `dist >= base_clearance + w/2` enforces KiCad's pairwise max(base, classF)
    against a pad in a wider (e.g. controlled-impedance) class. Inert when None."""
    nids, cx, cy, hx, hy, cr, rc, rs, ex, ey, plc, custom = \
        _foreign_pad_arrays(pcb_data, layer)
    best_custom = _custom_pad_min_dist(custom, net_id, ((x, y),), base_clearance)
    if cx.size == 0:
        return best_custom
    R = _FOREIGN_PAD_WINDOW
    # Expand the window by each pad's own half-extent (global axes, so tilted
    # pads window by their true bbox): a large pad's EDGE can be within R even
    # when its centre is past R. Then min edge distance is exact for anything
    # <= R (any excluded pad is > R away, beyond every caller's margin).
    near = (np.abs(cx - x) <= R + ex) & (np.abs(cy - y) <= R + ey) & (nids != net_id)
    if not near.any():
        return best_custom
    fcr = cr[near]
    # Rotate the query offset into each pad's local frame (R(-rot)); identity
    # for axis-aligned pads (cos=1, sin=0), exact tilted outline otherwise.
    ddx = x - cx[near]; ddy = y - cy[near]
    frc, frs = rc[near], rs[near]
    lx = np.abs(ddx * frc + ddy * frs)
    ly = np.abs(-ddx * frs + ddy * frc)
    dx = np.maximum(lx - (hx[near] - fcr), 0.0)
    dy = np.maximum(ly - (hy[near] - fcr), 0.0)
    d = np.hypot(dx, dy) - fcr
    if base_clearance is not None:
        excess = np.maximum(plc[near] - base_clearance, 0.0)
        if net_clearances:
            fcls = np.array([max(0.0, net_clearances.get(int(f), base_clearance) - base_clearance)
                             for f in nids[near]], dtype=float)
            excess = np.maximum(excess, fcls)
        d = d - excess
    return min(float(np.min(d)), best_custom)


def _seg_foreign_pad_dist(pcb_data, net_id, x1, y1, x2, y2, layer,
                          base_clearance=None, net_clearances=None):
    """Min foreign-pad edge distance sampled along a (short, terminal) segment.
    Pads are modelled as rounded rects (corner_r), so round/oval/roundrect pads --
    e.g. BGA balls -- use their true outline, not the bounding-box corner that
    manufactures phantom grazes (#315). Vectorized: windows pads to the segment's
    bbox + margin, then evaluates ALL sample points against them in one matrix op.
    `base_clearance` (#326 B6): see _pt_foreign_pad_dist -- subtracts each pad's
    local-clearance excess so unchanged caller thresholds honor overrides.
    `net_clearances` (#436): see _pt_foreign_pad_dist -- also folds each foreign
    pad's netclass excess so cross-class pad grazes are enforced."""
    nids, cx, cy, hx, hy, cr, rc, rs, ex, ey, plc, custom = \
        _foreign_pad_arrays(pcb_data, layer)
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
    _t = np.linspace(0.0, 1.0, n + 1)
    sx = x1 + (x2 - x1) * _t
    sy = y1 + (y2 - y1) * _t
    best_custom = _custom_pad_min_dist(
        custom, net_id, list(zip(sx, sy)), base_clearance)
    if cx.size == 0:
        return best_custom
    R = _FOREIGN_PAD_WINDOW
    # Pad bbox (centre +/- global half-extent, true bbox for tilted pads) must
    # reach within R of the segment bbox; a pad excluded here is > R from every
    # sample point on the segment.
    near = ((cx + ex >= min(x1, x2) - R) & (cx - ex <= max(x1, x2) + R) &
            (cy + ey >= min(y1, y2) - R) & (cy - ey <= max(y1, y2) + R) & (nids != net_id))
    if not near.any():
        return best_custom
    fcx, fcy, fhx, fhy, fcr = cx[near], cy[near], hx[near], hy[near], cr[near]
    frc, frs = rc[near], rs[near]
    # Rounded-rect signed distance in each pad's LOCAL frame (query offsets
    # rotated by R(-rot); identity for axis-aligned pads): shrink the
    # half-extents by the corner radius, take the outside distance to that
    # inner rect, then subtract the radius.
    ddx = sx[:, None] - fcx[None, :]
    ddy = sy[:, None] - fcy[None, :]
    lx = np.abs(ddx * frc[None, :] + ddy * frs[None, :])
    ly = np.abs(-ddx * frs[None, :] + ddy * frc[None, :])
    dx = np.maximum(lx - (fhx[None, :] - fcr[None, :]), 0.0)
    dy = np.maximum(ly - (fhy[None, :] - fcr[None, :]), 0.0)
    d = np.hypot(dx, dy) - fcr[None, :]
    if base_clearance is not None:
        excess = np.maximum(plc[near] - base_clearance, 0.0)
        if net_clearances:
            fcls = np.array([max(0.0, net_clearances.get(int(f), base_clearance) - base_clearance)
                             for f in nids[near]], dtype=float)
            excess = np.maximum(excess, fcls)
        d = d - excess[None, :]
    return min(float(np.min(d)), best_custom)


def _foreign_seg_arrays(pcb_data, layer):
    """Cached per-layer numpy arrays (net_id, x1, y1, x2, y2, half_width) for every
    routed SEGMENT on `layer`, plus every VIA folded in as a degenerate (zero-length)
    segment of half_width = via radius. Unlike pads, copper changes as nets route, so
    the cache is keyed on the (segment, via) counts and rebuilt when they change.
    Counts alone go stale across rip-reroute (#339: a ripped net re-adds the SAME
    number of segments at new coordinates -- cynthion's refit judged a via against
    MEZZANINE5's OLD track), so the tail elements' geometry joins the signature."""
    # #803: count + TAIL is not enough either. It cannot see an IN-PLACE field
    # mutation of a segment ALREADY in the list -- apply_stub_layer_switch
    # rewrites seg.layer in place ("segments will be modified in place", see
    # stub_layer_switching), which changes which layer's array a segment belongs
    # to while leaving lengths and the tail object identical. It also omitted
    # end_y, layer and width outright. This cache feeds _seg_foreign_seg_dist,
    # hence _unblock_via_refit, which decides the SIZE of a shipped via -- the
    # #339 failure it was hardened against ("cynthion's refit judged a via
    # against MEZZANINE5's OLD track") in a form the #339 fix still admits.
    # Fold the mutable geometry of the tail in, and a cheap whole-list digest of
    # the fields the arrays are BUILT from, so an in-place edit moves the
    # signature.
    segs, vias = pcb_data.segments, pcb_data.vias
    tail = segs[-1] if segs else None
    vtail = vias[-1] if vias else None
    sig = (len(segs), len(vias), id(segs), id(vias),
           (tail.start_x, tail.start_y, tail.end_x, tail.end_y,
            tail.layer, tail.width, tail.net_id) if tail is not None else None,
           (vtail.x, vtail.y, vtail.size, vtail.net_id) if vtail is not None else None,
           sum(map(id, segs)), sum(map(id, vias)),
           hash(tuple(sg.layer for sg in segs)))
    cache = getattr(pcb_data, '_foreign_seg_arr_cache', None)
    if cache is None or cache[0] != sig:
        cache = (sig, {})
        pcb_data._foreign_seg_arr_cache = cache
    per_layer = cache[1]
    arr = per_layer.get(layer)
    if arr is None:
        nid, ax, ay, bx, by, hw = [], [], [], [], [], []
        for s in pcb_data.segments:
            if s.layer == layer:
                nid.append(s.net_id); ax.append(s.start_x); ay.append(s.start_y)
                bx.append(s.end_x); by.append(s.end_y)
                hw.append((s.width if s.width > 0 else 0.0) / 2.0)
        # A via spans its drilled layers; treat every via as present on this copper
        # layer (a conservative over-approximation -- it only ever necks MORE).
        for v in pcb_data.vias:
            r = (v.size if getattr(v, 'size', 0) and v.size > 0 else 0.0) / 2.0
            nid.append(v.net_id); ax.append(v.x); ay.append(v.y)
            bx.append(v.x); by.append(v.y); hw.append(r)
        arr = (np.asarray(nid, dtype=np.int64), np.asarray(ax, dtype=float),
               np.asarray(ay, dtype=float), np.asarray(bx, dtype=float),
               np.asarray(by, dtype=float), np.asarray(hw, dtype=float))
        per_layer[layer] = arr
    return arr


def _seg_foreign_seg_dist(pcb_data, net_id, x1, y1, x2, y2, layer,
                          net_clearances=None, base_clearance=0.0,
                          track_clearances=None):
    """Min edge distance from a (short, terminal) segment to any OTHER-net segment or
    via on `layer` -- the segment analogue of _seg_foreign_pad_dist. Distance is from
    the terminal centreline to the foreign copper EDGE (point-to-segment distance to
    the foreign centreline minus the foreign half-width), sampled along the terminal
    and vectorized over windowed foreign segments. A negative result (centreline
    inside foreign copper) is returned as-is so the caller necks to the floor.

    #436: when `net_clearances` (net_id -> class clearance mm) is given, each
    foreign segment's netclass EXCESS over `base_clearance` (max(0, classF -
    base)) is SUBTRACTED from its distance, so a uniform caller check
    `dist >= base_clearance + w/2` enforces KiCad's pairwise max(base, classF)
    per foreign net. base_clearance should be the moving net's own floor
    (max(global, own class)). Inert when net_clearances is None.

    #735: `track_clearances` (config.track_clearances, {obstacle_net_id: mm})
    folds the SAME way, raise-only on top of the class value -- this channel is
    seg-vs-seg ONLY, which is exactly what this helper measures, so a caller
    passing it here must NOT pass it to the pad/via helpers (KiCad's
    Type=='track' binds tracks to tracks). Inert when the map is empty."""
    nid, fax, fay, fbx, fby, fhw = _foreign_seg_arrays(pcb_data, layer)
    if nid.size == 0:
        return 1e9
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
    R = _FOREIGN_PAD_WINDOW
    fminx = np.minimum(fax, fbx) - fhw; fmaxx = np.maximum(fax, fbx) + fhw
    fminy = np.minimum(fay, fby) - fhw; fmaxy = np.maximum(fay, fby) + fhw
    near = ((fmaxx >= min(x1, x2) - R) & (fminx <= max(x1, x2) + R) &
            (fmaxy >= min(y1, y2) - R) & (fminy <= max(y1, y2) + R) & (nid != net_id))
    if not near.any():
        return 1e9
    ax, ay, bx, by, hw = fax[near], fay[near], fbx[near], fby[near], fhw[near]
    t = np.linspace(0.0, 1.0, n + 1)
    sx = x1 + (x2 - x1) * t
    sy = y1 + (y2 - y1) * t
    abx = bx - ax; aby = by - ay                      # (M,)
    L2 = abx * abx + aby * aby                         # (M,)
    pax = sx[:, None] - ax[None, :]                    # (S, M)
    pay = sy[:, None] - ay[None, :]
    safe_L2 = np.where(L2 > 0, L2, 1.0)
    tt = (pax * abx[None, :] + pay * aby[None, :]) / safe_L2[None, :]
    tt = np.where(L2[None, :] > 0, np.clip(tt, 0.0, 1.0), 0.0)
    projx = ax[None, :] + tt * abx[None, :]
    projy = ay[None, :] + tt * aby[None, :]
    dist = np.hypot(sx[:, None] - projx, sy[:, None] - projy) - hw[None, :]
    if net_clearances or track_clearances:
        # #436: fold each foreign net's class-excess into its distance.
        # The track-rule value raises the same per-foreign requirement (#735).
        fnid = nid[near]
        _nc = net_clearances or {}
        _tc = track_clearances or {}
        excess = np.array([max(0.0,
                               max(_nc.get(int(f), base_clearance),
                                   _tc.get(int(f), 0.0)) - base_clearance)
                           for f in fnid], dtype=float)
        dist = dist - excess[None, :]
    return float(np.min(dist))


def _foreign_via_arrays(pcb_data):
    """Cached (net_id, x, y, radius) numpy arrays for every via on the board. Vias
    are treated as present on ALL layers (a through-hole conservative over-
    approximation, matching the obstacle map / _foreign_seg_arrays). Rebuilt when
    the via count OR tail via changes (count alone goes stale across rip-reroute,
    #339)."""
    vt = pcb_data.vias[-1] if pcb_data.vias else None
    sig = (len(pcb_data.vias), (vt.x, vt.y, vt.net_id) if vt is not None else None)
    cache = getattr(pcb_data, '_foreign_via_arr_cache', None)
    if cache is None or cache[0] != sig:
        nids, cx, cy, rad = [], [], [], []
        for v in pcb_data.vias:
            nids.append(v.net_id)
            cx.append(v.x); cy.append(v.y)
            rad.append((v.size if getattr(v, 'size', 0) and v.size > 0 else 0.0) / 2.0)
        cache = (sig, (np.asarray(nids, dtype=np.int64), np.asarray(cx, dtype=float),
                       np.asarray(cy, dtype=float), np.asarray(rad, dtype=float)))
        pcb_data._foreign_via_arr_cache = cache
    return cache[1]


def _seg_foreign_via_dist(pcb_data, net_id, x1, y1, x2, y2, layer,
                          net_clearances=None, base_clearance=0.0):
    """Min edge distance from a segment to any OTHER-net VIA (body), exact point-to-
    segment minus via radius. The via analogue of _seg_foreign_pad_dist; a negative
    result (centreline inside the via) is returned as-is. #436: `net_clearances`
    subtracts each foreign via's netclass excess over `base_clearance` (see
    _seg_foreign_seg_dist)."""
    nids, cx, cy, rad = _foreign_via_arrays(pcb_data)
    if cx.size == 0:
        return 1e9
    R = _FOREIGN_PAD_WINDOW
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    near = ((np.abs(cx - mx) <= R + abs(x2 - x1) / 2.0 + rad) &
            (np.abs(cy - my) <= R + abs(y2 - y1) / 2.0 + rad) & (nids != net_id))
    if not near.any():
        return 1e9
    fcx, fcy, fr = cx[near], cy[near], rad[near]
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    if l2 <= 0.0:
        d = np.hypot(fcx - x1, fcy - y1) - fr
    else:
        tt = np.clip(((fcx - x1) * dx + (fcy - y1) * dy) / l2, 0.0, 1.0)
        d = np.hypot(fcx - (x1 + tt * dx), fcy - (y1 + tt * dy)) - fr
    if net_clearances:
        fnid = nids[near]
        excess = np.array([max(0.0, net_clearances.get(int(f), base_clearance) - base_clearance)
                           for f in fnid], dtype=float)
        d = d - excess
    return float(np.min(d))


def _foreign_hole_capsules(pcb_data):
    """Cached NPTH (no-copper) drill capsules: (net_id, ax, ay, bx, by, r, lc)
    numpy arrays, one row per pad whose drill carries no copper ring (mechanical
    / mounting holes -- np_thru_hole, or a pad with no copper layer). The pad /
    segment / via distance trio all measure to COPPER, so they never see these
    holes; but a track crossing one is a real fab short (check_drc's track-hole
    rule, issue #233), gated by the higher NPTH-to-track floor. Holes are
    through, so the distance is layer-agnostic. Round drills degenerate to a
    zero-length capsule (a=b). Rebuilt when the board's pad count changes (pads
    are static during routing, so this almost never refires).

    `lc` is the hole pad's OWN resolved `local_clearance` (#760). check_drc
    grades this same geometry at `max(npth_clr, lc)` (#326/#505), so a consumer
    that prices every hole at one flat floor decides below what the grader
    requires on a pad carrying an override above the fab floor (corpus: ulx3s
    AUDIO1, drill 1.700, override 0.400 vs the 0.20 floor). Callers opt in via
    `_seg_foreign_hole_dist(..., base_clearance=...)`; the array is inert for
    the 0.0 that every other corpus NPTH pad carries."""
    from check_drc import _pad_has_no_copper
    from kicad_parser import pad_drill_capsule
    sig = sum(len(p) for p in pcb_data.pads_by_net.values())
    cache = getattr(pcb_data, '_foreign_hole_cap_cache', None)
    if cache is None or cache[0] != sig:
        nid, ax, ay, bx, by, r, lc = [], [], [], [], [], [], []
        for pad_net, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if (getattr(pad, 'drill', 0) or 0) > 0 and _pad_has_no_copper(pad):
                    (p1x, p1y), (p2x, p2y), hr = pad_drill_capsule(pad)
                    nid.append(pad_net)
                    ax.append(p1x); ay.append(p1y); bx.append(p2x); by.append(p2y)
                    r.append(hr)
                    lc.append(getattr(pad, 'local_clearance', 0.0) or 0.0)
        cache = (sig, (np.asarray(nid, dtype=np.int64), np.asarray(ax, dtype=float),
                       np.asarray(ay, dtype=float), np.asarray(bx, dtype=float),
                       np.asarray(by, dtype=float), np.asarray(r, dtype=float),
                       np.asarray(lc, dtype=float)))
        pcb_data._foreign_hole_cap_cache = cache
    return cache[1]


def _seg_foreign_hole_dist(pcb_data, net_id, x1, y1, x2, y2,
                           base_clearance=None):
    """Min edge distance from a segment to any OTHER-net NPTH drill hole (the
    hole analogue of _seg_foreign_via_dist). Exact segment-to-capsule distance
    minus the hole radius; a negative result (segment over the hole) is returned
    as-is. Own-net holes are excluded (a track legitimately reaches its own
    mounting-hole pad). 1e9 when there are no foreign holes.

    #760: with `base_clearance` (the caller's flat NPTH floor), each hole's own
    `local_clearance` EXCESS over that floor is subtracted from its distance --
    the same trick #436 uses for foreign net-class clearance, so the single
    returned number stays directly comparable against the flat floor while
    honoring check_drc's per-hole `max(npth_clr, lc)`. Omitted (the default)
    keeps every existing caller bit-identical, so the sites #617 deliberately
    left flat stay flat."""
    nid, hax, hay, hbx, hby, hr, hlc = _foreign_hole_capsules(pcb_data)
    if nid.size == 0:
        return 1e9
    R = _FOREIGN_PAD_WINDOW
    hminx = np.minimum(hax, hbx) - hr; hmaxx = np.maximum(hax, hbx) + hr
    hminy = np.minimum(hay, hby) - hr; hmaxy = np.maximum(hay, hby) + hr
    near = ((hmaxx >= min(x1, x2) - R) & (hminx <= max(x1, x2) + R) &
            (hmaxy >= min(y1, y2) - R) & (hminy <= max(y1, y2) + R) & (nid != net_id))
    if not near.any():
        return 1e9
    ax, ay, bx, by, rr = hax[near], hay[near], hbx[near], hby[near], hr[near]
    d = _seg_capsule_axis_dist(x1, y1, x2, y2, ax, ay, bx, by) - rr
    if base_clearance is not None:
        d = d - np.maximum(0.0, hlc[near] - base_clearance)
    return float(np.min(d))


def _seg_capsule_axis_dist(x1, y1, x2, y2, ax, ay, bx, by):
    """Exact distance from segment (x1,y1)-(x2,y2) to each capsule AXIS segment
    (ax,ay)-(bx,by) (vectorized over the capsule arrays), returned per capsule.
    Segment-to-segment distance = min of the four endpoint-to-other-segment
    distances, or 0 where the two segments properly cross -- the same measure
    check_drc uses for track-hole."""
    def pt_to_seg(px, py, qx1, qy1, qx2, qy2):
        dx = qx2 - qx1; dy = qy2 - qy1
        L2 = dx * dx + dy * dy
        safe = np.where(L2 > 0, L2, 1.0)
        t = np.clip(((px - qx1) * dx + (py - qy1) * dy) / safe, 0.0, 1.0)
        return np.hypot(px - (qx1 + t * dx), py - (qy1 + t * dy))
    d = np.minimum(pt_to_seg(ax, ay, x1, y1, x2, y2),
                   pt_to_seg(bx, by, x1, y1, x2, y2))
    d = np.minimum(d, pt_to_seg(x1, y1, ax, ay, bx, by))
    d = np.minimum(d, pt_to_seg(x2, y2, ax, ay, bx, by))
    # Proper crossing -> distance 0 (orientation sign test, per capsule).
    sdx, sdy = x2 - x1, y2 - y1
    hdx, hdy = bx - ax, by - ay
    o1 = sdx * (ay - y1) - sdy * (ax - x1)
    o2 = sdx * (by - y1) - sdy * (bx - x1)
    o3 = hdx * (y1 - ay) - hdy * (x1 - ax)
    o4 = hdx * (y2 - ay) - hdy * (x2 - ax)
    crossing = (o1 * o2 < 0) & (o3 * o4 < 0)
    return np.where(crossing, 0.0, d)


def _unblock_via_refit(pcb_data, net_id, x, y, rec, config):
    """Re-validate a registered #189 unblock via against CURRENT copper (#339).

    The registration was validated against the copper of ITS moment; a later
    rip-reroute cascade can move foreign copper closer (cynthion: MEZZANINE5's
    re-route landed 0.35mm from a cell registered when it was legal, and the
    emitted 0.45 via grazed it by 39um). Try the registered size first, then
    the fab-floor ladder's smaller vias (shrink-to-fit, same spirit as #189's
    escalation); return the first that clears foreign copper mm-exactly, or
    None when nothing fits (caller keeps the registered size -- honest DRC)."""
    from fab_tiers import fab_floor_ladder
    import routing_defaults as defaults
    clearance = config.clearance
    eps = defaults.UNBLOCK_REFIT_MARGIN_MM
    layers = [l for l in (pcb_data.board_info.copper_layers or []) if l.endswith('.Cu')]
    ncu = len(layers) or 2
    cands = [rec]
    for f in fab_floor_ladder(ncu):
        pair = (round(f['via_diameter'], 3), round(f['via_drill'], 3))
        if pair[0] < rec[0] - 1e-9 and pair not in cands:
            cands.append(pair)
    for vs, dr in cands:
        need = vs / 2.0 + clearance - eps
        ok = True
        for layer in layers:
            if _seg_foreign_seg_dist(pcb_data, net_id, x, y, x, y, layer) < need:
                ok = False
                break
            if _pt_foreign_pad_dist(pcb_data, net_id, x, y, layer,
                                    base_clearance=clearance) < need:
                ok = False
                break
        if ok and _seg_foreign_via_dist(pcb_data, net_id, x, y, x, y, layers[0] if layers else 'F.Cu') < need:
            ok = False
        if ok:
            return (vs, dr)
    return None


def _emit_via_size(pcb_data, gx, gy, config, net_id=None, x=None, y=None):
    """(size, drill) for a via the route conversion emits at cell (gx, gy). If a #189
    via-in-pad unblock placed a DRC-legal shrunk via here, return THAT size so the
    emitted via matches it -- a full config.via_size via at the same cell would graze
    the neighbouring foreign pad the shrunk via was sized to clear (issue #212).
    With net_id + mm coords, the registered size is RE-VALIDATED against current
    copper and shrunk to fit (#339) -- registrations go stale across rip-reroute.
    Otherwise return the configured via size."""
    sizes = getattr(pcb_data, '_unblock_via_sizes', None)
    rec = sizes.get((gx, gy)) if sizes is not None else None
    if rec is None:
        rec = (config.via_size, config.via_drill)
    # Re-validate EVERY emitted via against current copper (#339): the #189
    # unblock retry's allowed-cell window lets A* place a via on a cell whose
    # via-blocking says no (that is the window's purpose -- reaching a boxed
    # pad), and rip-reroute can move foreign copper toward any cell after its
    # blocking was computed. A via that would ship grazing shrinks to the
    # largest fab-ladder size that clears; clean vias return unchanged (the
    # first candidate fits). If even the smallest grazes, ship rec -- honest.
    if net_id is not None and x is not None and y is not None:
        refit = _unblock_via_refit(pcb_data, net_id, x, y, rec, config)
        if _unblock_debug():
            print(f"      EMIT-REFIT: cell=({gx},{gy}) {rec} -> {refit} net={net_id} at ({x},{y})")
        if refit is not None:
            return refit
    return rec


def _fab_track_floor(pcb_data) -> float:
    """Smallest manufacturable track width for this board (issue #176): the JLC
    fab minimum for the board's copper-layer count (0.0889 mm on 4+ layers,
    0.127 mm on 2). Necking a grazing terminal must not emit copper below this --
    the old 0.05 mm grid-step floor produced sub-fab tracks that pass our
    clearance-only DRC but would fail KiCad's built-in track-width rule."""
    from list_nets import fab_floors
    n = 2
    try:
        n = len(pcb_data.board_info.copper_layers) or 2
    except (AttributeError, TypeError):
        pass
    return fab_floors(n)['track_width']


def _neck_terminal_grazes(segments, term_pts, pcb_data, net_id, config, floor=None):
    """Neck a TERMINAL-connection segment that grazes foreign copper, down to `floor`.

    A route's terminal connects to an off-grid pad / fanout escape: the
    exact-endpoint stub and the first/last on-grid leg are laid geometrically and
    the endpoint region is obstacle-exempt (so the net can reach its own pad), so a
    full-width terminal can sit sub-clearance to NEIGHBOURING foreign copper -- a pad
    (#157, e.g. tigard Net-(R7-Pad2) grazing the VREG pad by 8um) OR another net's
    track/via (#212: a +1V2 terminal into a cap pad grazing a wide +3V3 trace by
    ~15um). Narrowing the offending terminal segment restores clearance without
    moving the centreline, so connectivity is preserved; a graze the floor width
    still can't clear is left for the DRC report. Only segments touching a terminal
    point are considered (the A* body keep-outs already enforce clearance mid-route).

    Returns ``(necked, hard)``. `hard` lists terminal segments whose RAW
    geometric centreline sits closer than floor/2 to foreign track/via copper:
    even at the fab-floor width the copper physically OVERLAPS the foreign net
    -- a shipped SHORT no neck can fix (ux pf8/pf9: a GND terminal bridge into
    C61 slashed across SDRAM_A2's In1.Cu trunk; the neck took it to the 0.0889
    floor and the crossing shipped). Callers at route-attempt boundaries treat
    a non-empty `hard` as route failure -- an honest failure beats a shipped
    short -- while clearance-only grazes stay in the neck regime. The hard test
    uses RAW distances (no #436 class excess, no #326 pad-override adjustment):
    those model CLEARANCE rules, and folding them in would reject legal copper
    that merely violates clearance (which stays a visible DRC flag).

    `floor` defaults to the board's fab track-width minimum (issue #176): necking
    to the grid step (0.05 mm) used to emit sub-fab-floor copper."""
    if floor is None:
        floor = _fab_track_floor(pcb_data)
    # #436: neck against the moving net's own class floor, folding each foreign
    # object's class excess, so a terminal grazing a wider (controlled-impedance)
    # neighbour necks to the pairwise max(classOwn, classForeign), not the flat
    # global clearance. Inert on an all-Default board.
    nc = getattr(config, 'net_clearances', None) or None
    own = (config.obstacle_clearance(net_id)
           if hasattr(config, 'obstacle_clearance') else config.clearance)
    def touches(s):
        for tx, ty in term_pts:
            if (abs(s.start_x - tx) < 1e-6 and abs(s.start_y - ty) < 1e-6) or \
               (abs(s.end_x - tx) < 1e-6 and abs(s.end_y - ty) < 1e-6):
                return True
        return False
    necked = 0
    hard = []
    for s in segments:
        if not touches(s):
            continue
        # Nearest foreign EDGE on this layer -- pad or track/via, whichever is
        # closer. Pad distances are override-adjusted (#326): a pad's local/
        # footprint clearance above the base is subtracted, so the allowed_half
        # below necks to the pad's OWN required clearance.
        # #498: a .kicad_dru layer rule replaces the pair clearance on s.layer.
        own_l = (config.layer_clearance(s.layer, own)
                 if hasattr(config, 'layer_clearance') else own)
        d = min(_seg_foreign_pad_dist(pcb_data, net_id, s.start_x, s.start_y, s.end_x, s.end_y, s.layer,
                                      base_clearance=own_l, net_clearances=nc),
                _seg_foreign_seg_dist(pcb_data, net_id, s.start_x, s.start_y, s.end_x, s.end_y, s.layer,
                                      net_clearances=nc, base_clearance=own_l))
        allowed_half = d - own_l - 1e-4  # 1e-4: stay just inside the rule
        if allowed_half < s.width / 2.0 - 1e-9:
            # Hard test BEFORE necking, on the RAW track/via distance: a
            # centreline within floor/2 of a foreign copper EDGE overlaps it
            # at any emittable width -- a physical short, not a graze.
            d_raw = _seg_foreign_seg_dist(pcb_data, net_id, s.start_x,
                                          s.start_y, s.end_x, s.end_y, s.layer)
            if d_raw < floor / 2.0 - 1e-6:
                hard.append((s, d_raw))
                continue
            new_w = max(floor, 2.0 * allowed_half)
            if new_w < s.width - 1e-9:
                s.width = round(new_w, 4)
                necked += 1
    return necked, hard


def _neck_route_terminal_grazes(segments, path, coord, start_original, end_original,
                                pcb_data, net_id, config):
    """Run _neck_terminal_grazes for a converted multipoint edge/tap, recomputing the
    terminal points from the path endpoints + original pad positions. Called AFTER
    _apply_neckdown_widths / uniform_width so the graze-neck is authoritative: those
    passes rebuild every width from the pad-distance taper and would otherwise restore
    a grazing terminal to full/base width, undoing the neck (issue #212).

    Returns the `hard` list from _neck_terminal_grazes: terminal copper that
    physically OVERLAPS a foreign track/via even at the fab floor. Callers must
    treat a non-empty return as edge/route FAILURE (shipped-short class, ux
    pf9) -- the neck alone cannot fix a crossing."""
    if pcb_data is None or not path:
        return []
    term_pts = [coord.to_float(path[0][0], path[0][1]),
                coord.to_float(path[-1][0], path[-1][1])]
    if start_original:
        term_pts.append((start_original[0], start_original[1]))
    if end_original:
        term_pts.append((end_original[0], end_original[1]))
    _necked, _hard = _neck_terminal_grazes(segments, term_pts, pcb_data,
                                           net_id, config)
    return _hard


def _merge_terminal_to_exact(path, term_idx, neighbor_idx, original, pts,
                             pcb_data, net_id, config, layer_names):
    """#4: route.py is on-grid, but a route's TERMINAL connects to an off-grid pad
    or fanout escape. When the terminal grid cell lands inside a foreign pad's
    clearance (a quantised stand-in for the real, off-grid endpoint) but the EXACT
    endpoint -- and the segment from the neighbour point to it -- clear that pad,
    replace pts[term_idx] with the exact endpoint so the terminal segment runs to
    the clean point and the grazing grid-cell vertex disappears. Returns True if
    merged (caller then skips the separate connection stub). Uses explicit path
    indices, not coordinate matching, so float noise can't pick the wrong segment."""
    if original is None or len(path) < 2:
        return False
    if path[term_idx][2] != path[neighbor_idx][2]:
        return False  # via at the very endpoint -> leave it on grid
    ox, oy, ol = original
    if layer_names[path[term_idx][2]] != ol:
        return False
    fx, fy = pts[term_idx]
    if abs(ox - fx) < 1e-9 and abs(oy - fy) < 1e-9:
        return False  # exact endpoint already is the grid cell
    margin = config.clearance + config.get_net_track_width(net_id, ol) / 2.0
    if _pt_foreign_pad_dist(pcb_data, net_id, fx, fy, ol,
                            base_clearance=config.clearance) >= margin:
        return False  # grid cell already clear -> nothing to fix
    if _pt_foreign_pad_dist(pcb_data, net_id, ox, oy, ol,
                            base_clearance=config.clearance) < margin:
        return False  # exact endpoint also too close (placement) -> can't fix here
    nx, ny = pts[neighbor_idx]
    # Only relocate the endpoint of a SHORT terminal segment. simplify_path (caller,
    # before this) collapses collinear runs, so the terminal segment can be long;
    # moving its far end to an off-grid point would tilt the whole run into a long
    # diagonal that cuts across cells reserved for other copper (keks #158). When the
    # terminal segment is longer than ~1 grid cell, keep the grid endpoint and let the
    # caller's short connecting stub run out to the exact point instead.
    if math.hypot(nx - fx, ny - fy) > 1.5 * config.grid_step:
        return False  # long terminal segment -> keep grid end + short stub
    if _seg_foreign_pad_dist(pcb_data, net_id, ox, oy, nx, ny, ol,
                             base_clearance=config.clearance) < margin - 1e-6:
        return False  # merged terminal segment would graze -> keep grid + stub
    pts[term_idx] = (ox, oy)
    return True


def print_route_stats(stats: dict, print_prefix: str = "  "):
    """Print A* routing statistics in a readable format.

    Args:
        stats: Dictionary of statistics from route_multi_with_stats
        print_prefix: Prefix for each line (default: "  ")
    """
    print(f"{print_prefix}A* Search Statistics:")
    print(f"{print_prefix}  Cells expanded:  {int(stats.get('cells_expanded', 0)):,} (popped from open set)")
    print(f"{print_prefix}  Cells pushed:    {int(stats.get('cells_pushed', 0)):,} (added to open set)")
    print(f"{print_prefix}  Cells revisited: {int(stats.get('cells_revisited', 0)):,} (path improvements)")
    print(f"{print_prefix}  Duplicate skips: {int(stats.get('duplicate_skips', 0)):,} (already in closed)")
    print(f"{print_prefix}  Path length:     {int(stats.get('path_length', 0)):,} grid steps")
    print(f"{print_prefix}  Path cost:       {int(stats.get('path_cost', 0)):,}")
    print(f"{print_prefix}  Via count:       {int(stats.get('via_count', 0)):,}")
    print(f"{print_prefix}  Initial h:       {int(stats.get('initial_h', 0)):,}")
    print(f"{print_prefix}  Final g:         {int(stats.get('final_g', 0)):,}")
    print(f"{print_prefix}  Open set size:   {int(stats.get('open_set_size', 0)):,} (at termination)")
    print(f"{print_prefix}  Closed set size: {int(stats.get('closed_set_size', 0)):,} (unique visited)")

    # Computed ratios
    h_ratio = stats.get('heuristic_ratio', 0)
    if h_ratio > 0:
        # Note: h_ratio > 1.0 is expected when using weighted A* (h_weight > 1.0)
        # The heuristic is multiplied by h_weight to trade optimality for speed
        if abs(h_ratio - 1.0) < 0.01:
            quality = "perfect heuristic"
        elif h_ratio < 1.0:
            quality = "admissible (underestimate)"
        else:
            quality = f"weighted A* (h_weight ~{h_ratio:.1f})"
        print(f"{print_prefix}  Heuristic ratio: {h_ratio:.3f} (h/g, {quality})")

    exp_ratio = stats.get('expansion_ratio', 0)
    if exp_ratio > 0:
        quality = "excellent" if exp_ratio < 2 else "good" if exp_ratio < 5 else "poor" if exp_ratio < 20 else "very poor"
        print(f"{print_prefix}  Expansion ratio: {exp_ratio:.1f}x path length ({quality})")

    revisit_ratio = stats.get('revisit_ratio', 0)
    if revisit_ratio >= 0:
        print(f"{print_prefix}  Revisit ratio:   {revisit_ratio:.3f} (path improvements / expanded)")

    skip_ratio = stats.get('skip_ratio', 0)
    if skip_ratio >= 0:
        print(f"{print_prefix}  Skip ratio:      {skip_ratio:.3f} (duplicates / total pops)")


def _print_obstacle_map(obstacles: 'GridObstacleMap', center_gx: int, center_gy: int, layer: int, radius: int = 20, print_prefix: str = ""):
    """Print a visual map of blocking around a center point."""
    print(f"{print_prefix}  Obstacle map around ({center_gx}, {center_gy}) layer={layer} (radius={radius}):")
    for dy in range(-radius, radius + 1):
        row = []
        for dx in range(-radius, radius + 1):
            cx, cy = center_gx + dx, center_gy + dy
            if dx == 0 and dy == 0:
                row.append('T')
            elif obstacles.is_blocked(cx, cy, layer):
                row.append('#')
            else:
                row.append('.')
        print(f"{print_prefix}    {''.join(row)}")


def _identify_blocking_obstacles(
    blocked_positions: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    current_net_id: int = -1
) -> Dict[int, Tuple[str, int]]:
    """Identify which nets are blocking specific grid positions; net_id -> (name, count).

    The per-cell neighbourhood scan over every foreign segment/via/pad is done in
    Rust (grid_router.identify_blocking_obstacles, 0.16.1+). We build the
    grid-integer arrays for the foreign geometry and call it. There is no Python
    fallback: rebuild the router if the loaded binary predates 0.16.1."""
    try:
        import grid_router
        rust_fn = grid_router.identify_blocking_obstacles
    except (ImportError, AttributeError) as e:
        raise RuntimeError(
            "grid_router.identify_blocking_obstacles is unavailable -- rebuild the "
            "Rust router to 0.16.1+ (python build_router.py --from-source)") from e

    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    num_layers = len(config.layers)
    expansion_grid = max(1, coord.to_grid_dist(config.track_width + config.clearance))
    via_expansion_grid = max(1, coord.to_grid_dist(
        config.via_size / 2 + config.track_width / 2 + config.clearance))

    blocked = np.asarray(blocked_positions, dtype=np.int64) if blocked_positions \
        else np.empty((0, 3), dtype=np.int64)

    # Geometry-array memo (2026-08-14 profiling: 4,522 calls / 53s, each
    # rebuilding these arrays from a full board scan). The ALL-NETS arrays
    # depend only on (copper epoch, grid, layers, clearance/track/via
    # geometry); the per-call net exclusion is a STABLE boolean filter, so
    # the filtered arrays hold the same rows in the same order as the
    # original per-call build -- byte-identical inputs to the Rust scan.
    # (pad.net_id == 0 is excluded unconditionally in the base arrays,
    # exactly as the original loop did.)
    _gkey = (getattr(pcb_data, '_copper_epoch', 0), config.grid_step,
             tuple(config.layers), config.track_width, config.clearance,
             config.via_size)
    _gmemo = getattr(pcb_data, '_blockid_geom_memo', None)
    if _gmemo is not None and _gmemo[0] == _gkey:
        segs_all, vias_all, pads_all = _gmemo[1], _gmemo[2], _gmemo[3]
    else:
        seg_rows = []
        for seg in pcb_data.segments:
            li = layer_map.get(seg.layer)
            if li is None:
                continue
            gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
            gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
            seg_rows.append((gx1, gy1, gx2, gy2, li, seg.net_id))
        segs_all = (np.asarray(seg_rows, dtype=np.int64) if seg_rows
                    else np.empty((0, 6), dtype=np.int64))

        via_rows = []
        for via in pcb_data.vias:
            gx, gy = coord.to_grid(via.x, via.y)
            via_rows.append((gx, gy, via.net_id))
        vias_all = (np.asarray(via_rows, dtype=np.int64) if via_rows
                    else np.empty((0, 3), dtype=np.int64))

        pad_rows = []
        for ref, footprint in pcb_data.footprints.items():
            for pad in footprint.pads:
                if pad.net_id == 0:
                    continue
                gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                if hasattr(pad, 'size_x'):
                    pad_half_x, pad_half_y = pad_rect_halfspan(pad)
                else:
                    pad_half_x = pad_half_y = 0.5
                ex_x = max(1, coord.to_grid_dist(pad_half_x + config.clearance + config.track_width / 2))
                ex_y = max(1, coord.to_grid_dist(pad_half_y + config.clearance + config.track_width / 2))
                if pad.drill and pad.drill > 0:
                    mask = (1 << num_layers) - 1  # through-hole: all layers
                else:
                    mask = 0
                    for layer_name in pad.layers:
                        if layer_name in layer_map:
                            mask |= 1 << layer_map[layer_name]
                if mask:
                    pad_rows.append((gx, gy, ex_x, ex_y, pad.net_id, mask))
        pads_all = (np.asarray(pad_rows, dtype=np.int64) if pad_rows
                    else np.empty((0, 6), dtype=np.int64))
        pcb_data._blockid_geom_memo = (_gkey, segs_all, vias_all, pads_all)

    segs = segs_all[segs_all[:, 5] != current_net_id] if len(segs_all) \
        else segs_all
    vias = vias_all[vias_all[:, 2] != current_net_id] if len(vias_all) \
        else vias_all
    pads = pads_all[pads_all[:, 4] != current_net_id] if len(pads_all) \
        else pads_all

    # Two calls, split by obstacle KIND (run-6 fix): copper (segments+vias,
    # rippable) versus pads (static). The merged count sent the operator on
    # named-rip goose chases -- measured on test-board run 5: GPIO7 kept being
    # cited AFTER its rip because the residue was its unrippable PAD, and the
    # decisive 1-cell track wall ranked under big-perimeter bystanders.
    _no_segs = np.empty((0, 6), dtype=np.int64)
    _no_vias = np.empty((0, 3), dtype=np.int64)
    _no_pads = np.empty((0, 6), dtype=np.int64)
    counts_copper = rust_fn(blocked, segs, vias, _no_pads,
                            int(expansion_grid), int(via_expansion_grid),
                            int(num_layers))
    counts_pads = rust_fn(blocked, _no_segs, _no_vias, pads,
                          int(expansion_grid), int(via_expansion_grid),
                          int(num_layers))
    blockers: Dict[int, Tuple[str, int, int, int]] = {}
    for nid in set(counts_copper) | set(counts_pads):
        net_name = pcb_data.nets[nid].name if nid in pcb_data.nets else f"net_{nid}"
        tc = int(counts_copper.get(nid, 0))
        pc = int(counts_pads.get(nid, 0))
        blockers[nid] = (net_name, tc + pc, tc, pc)
    return blockers


# --- deferred search diagnostics ---------------------------------------------
# The blocked-start dump and the probe/stuck chatter explain why an A* search
# STALLED. A stall is usually recovered moments later (neck-down retry, the
# other direction, rip-up), so these lines mostly narrate nets that route fine:
# over a 15-board corpus run, ~35% of these bytes sat on nets that ended in
# SUCCESS, and the category as a whole was ~50% of all routing output.
#
# So buffer them for the duration of one net's routing attempt and emit only if
# that net actually fails -- which is the moment they are worth reading, and
# where they now appear, directly above the FAILED line. `--verbose` streams
# them live as before.
_DEFERRED_DIAG: Optional[List[str]] = None


def _diag(msg: str) -> None:
    """Emit a search diagnostic: live, or into the active per-net capture."""
    if _DEFERRED_DIAG is None:
        print(msg)
    else:
        _DEFERRED_DIAG.append(msg)


@contextmanager
def deferred_diagnostics(config: GridRouteConfig = None):
    """Capture search diagnostics for one net instead of printing them.

    Yields the buffer (a list of lines), or None when diagnostics stream live --
    under `--verbose`, or when an outer capture is already active and owns them.
    The buffer stays readable after the block exits; pass it to
    `flush_diagnostics()` on the failure path.
    """
    global _DEFERRED_DIAG
    if (config is not None and getattr(config, 'verbose', False)) or _DEFERRED_DIAG is not None:
        yield None
        return
    buf: List[str] = []
    _DEFERRED_DIAG = buf
    try:
        yield buf
    finally:
        _DEFERRED_DIAG = None


def flush_diagnostics(buf: Optional[List[str]]) -> None:
    """Print diagnostics captured by `deferred_diagnostics` (no-op if empty).

    Repeated attempts on one net (wide then neck-down, the #189 via-unblock
    retry) re-run the same probe and re-report the same stall, verbatim. An
    exactly identical line says nothing new the second time, so identical lines
    collapse to their first occurrence, tagged with a repeat count.
    """
    if not buf:
        return
    counts: Dict[str, int] = {}
    order: List[str] = []
    for line in buf:
        if line in counts:
            counts[line] += 1
        else:
            counts[line] = 1
            order.append(line)
    for line in order:
        n = counts[line]
        print(f"{line}  [x{n}]" if n > 1 else line)
    del buf[:]


def _diagnose_blocked_start(obstacles: 'GridObstacleMap', cells: List, label: str, print_prefix: str = "", track_margin=0,
                            pcb_data: PCBData = None, config: GridRouteConfig = None, current_net_id: int = -1):
    """
    Diagnose why routing couldn't start from the given cells.

    Checks blocking status of start cells and their immediate neighbors.
    If pcb_data and config are provided, also identifies which nets are blocking.
    """
    if not cells:
        _diag(f"{print_prefix}  {label}: no cells to check")
        return

    # Check a sample of cells. One is enough to characterize a boxed-in
    # endpoint -- the sampled cells are neighbors of each other and report
    # near-identical blockage; --verbose keeps the wider sample.
    sample_n = 3 if (config is not None and getattr(config, 'verbose', False)) else 1
    sample_cells = cells[:sample_n]

    for gx, gy, layer in sample_cells:
        # Diagnostic sweep radius: the layer's margin, rounded UP to whole cells
        # (#156 margins are fractional and possibly per-layer; ceil is fine for
        # a why-blocked report)
        cell_margin = int(math.ceil(_margin_at(track_margin, layer)))
        # Check if the cell itself is blocked
        cell_blocked = obstacles.is_blocked(gx, gy, layer)

        # Check neighbors (8-connected)
        blocked_neighbors = 0
        total_neighbors = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                total_neighbors += 1
                # Check with margin if specified
                if cell_margin > 0:
                    neighbor_blocked = False
                    for mx in range(-cell_margin, cell_margin + 1):
                        for my in range(-cell_margin, cell_margin + 1):
                            if obstacles.is_blocked(gx + dx + mx, gy + dy + my, layer):
                                neighbor_blocked = True
                                break
                        if neighbor_blocked:
                            break
                else:
                    neighbor_blocked = obstacles.is_blocked(gx + dx, gy + dy, layer)
                if neighbor_blocked:
                    blocked_neighbors += 1

        status = "BLOCKED" if cell_blocked else "ok"
        margin_str = f" (margin={cell_margin})" if cell_margin > 0 else ""
        _diag(f"{print_prefix}  {label} cell ({gx}, {gy}, layer={layer}): {status}, {blocked_neighbors}/{total_neighbors} neighbors blocked{margin_str}")
        # The individual blocked-neighbor coordinates used to be listed here,
        # but only when ALL of them were blocked -- i.e. exactly when they are
        # the eight neighbors of the cell just printed, and so derivable from
        # it. That line alone was 18% of all routing output.

        # Identify what's blocking if pcb_data and config are provided
        if blocked_neighbors > 0 and pcb_data is not None and config is not None:
            # Collect blocked neighbor positions
            blocked_positions = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    if cell_margin > 0:
                        for mx in range(-cell_margin, cell_margin + 1):
                            for my in range(-cell_margin, cell_margin + 1):
                                if obstacles.is_blocked(gx + dx + mx, gy + dy + my, layer):
                                    blocked_positions.append((gx + dx + mx, gy + dy + my, layer))
                    else:
                        if obstacles.is_blocked(gx + dx, gy + dy, layer):
                            blocked_positions.append((gx + dx, gy + dy, layer))

            if blocked_positions:
                blockers = _identify_blocking_obstacles(blocked_positions, pcb_data, config, current_net_id)
                if blockers:
                    # Sort by count descending; say WHICH KIND of copper blocks.
                    # "GPIO7(1 pad)" is not a rip candidate; "GPIO7(2 track)"
                    # is -- the merged count used to read identically.
                    sorted_blockers = sorted(blockers.items(),
                                             key=lambda x: x[1][1], reverse=True)
                    blocker_strs = []
                    for _bnid, (name, _tot, tc, pc) in sorted_blockers[:5]:
                        parts = ([f"{tc} track"] if tc else []) \
                            + ([f"{pc} pad"] if pc else [])
                        blocker_strs.append(f"{name}({', '.join(parts) or 0})")
                    _diag(f"{print_prefix}    Blocking obstacles: {', '.join(blocker_strs)}")
                    # NOTE (1a903195, reverted here): this used to ALSO feed the
                    # track-blocking identities into `pcb_data._stuck_wall_blame`,
                    # which phase3_routing.try_phase3_ripup consumes as
                    # validator-named blockers -- and those sort ahead of every
                    # frontier-inferred tier. That made a DIAGNOSTIC pass a live
                    # input to rip VICTIM SELECTION, which is why the commit
                    # titled "diagnostics:" changed routing.
                    #
                    # Measured on neo6502 (set1): promoting stuck-probe walls
                    # left 8 nets disconnected (+3.3V, /A8, /D7, /D6, /D0, /A1,
                    # /A4, GND -- mostly one stranded pad each) where main
                    # finished clean, and the final --rip-existing-nets '*' step
                    # then repaired NONE of them, against 4-of-4 on main.
                    # It did buy 1 DRC violation -> 0. Keep the split-by-kind
                    # DIAGNOSTICS above (a pad is not a rip candidate, and the
                    # merged count hid that); drop only the rip-cascade feed.


def _via_drill_exclusion_radius(config: 'GridRouteConfig') -> int:
    """Grid-cell radius for the Rust router's same-path via-spacing guard, sized
    from the DRILL hole-to-hole minimum (same-net vias may touch copper but not
    drills). The router's check blocks Chebyshev dist <= 2*radius (issue #230)."""
    if config.hole_to_hole_clearance <= 0 or config.grid_step <= 0:
        return 0
    return max(1, int(math.ceil(
        (config.via_drill + config.hole_to_hole_clearance) / config.grid_step / 2.0)))


def _path_has_close_vias(path: Optional[List], config: 'GridRouteConfig') -> bool:
    """True if the path drops two of its OWN vias closer than the drill hole-to-hole
    minimum (a same-net VIA-DRILL-HOLE the obstacle map can't prevent within one
    A* path). A via is a layer change at a fixed (gx, gy)."""
    if not path or config.hole_to_hole_clearance <= 0:
        return False
    via_cells = []
    for i in range(1, len(path)):
        if path[i][0] == path[i - 1][0] and path[i][1] == path[i - 1][1] \
                and path[i][2] != path[i - 1][2]:
            via_cells.append((path[i][0], path[i][1]))
    if len(via_cells) < 2:
        return False
    min_cc = (config.via_drill + config.hole_to_hole_clearance) / config.grid_step
    min_cc_sq = min_cc * min_cc
    seen = set()
    for gx, gy in via_cells:
        if (gx, gy) in seen:
            continue
        seen.add((gx, gy))
        for ox, oy in seen:
            if (ox, oy) != (gx, gy) and (gx - ox) ** 2 + (gy - oy) ** 2 < min_cc_sq:
                return True
    return False


# #529 absolute ceiling for dynamic iteration extension: bounds the open-set
# heap (~24-32 B/push) and wall clock even when every tranche keeps earning.
DYNAMIC_ITERATIONS_CEILING = 10_000_000


def _dynamic_iterations(config: 'GridRouteConfig') -> Tuple[int, dict]:
    """#529 dynamic iterations (DEFAULT ON; KICAD_DYNAMIC_ITERATIONS=0 reverts to
    static caps): (effective_base, kwargs) for a FULL search. The base is
    min(config.max_iterations, KICAD_DYNAMIC_ITERATIONS_CLAMP) -- CLAMP defaults
    to 1e7, i.e. no clamping (corpus: -29 incomplete nets over 150 boards); set
    CLAMP=200000 as the deliberate speed-over-completion dial. The search may then
    earn +1x base tranches while its closest approach (best_h, tracked in the Rust
    core) keeps improving, up to a flat 1e7 ceiling.
    Scope: probe-scale budgets never extend -- fast-fail retry configs clone the
    config with max_iterations = (2x) max_probe_iterations (5k default) precisely
    to give up early, so any base at or below 10k keeps its static cap. Oracle,
    plane, and pose (diff-pair centerline) searches don't go through this helper
    at all.
    With the knob OFF no kwarg is passed, so such runs are byte-identical to the
    pre-#529 caps and predate-0.19.2 grid_router binaries keep working."""
    if not env_knobs.DYNAMIC_ITERATIONS or config.max_iterations <= 10_000:
        return config.max_iterations, {}
    base = min(config.max_iterations, env_knobs.DYNAMIC_ITERATIONS_CLAMP)
    kwargs = {'max_iterations_ceiling': DYNAMIC_ITERATIONS_CEILING}
    # Quantum/grace dials (#529 A/B): only pass when non-default (they
    # postdate the ceiling kwarg; older binaries lack them).
    cells = env_knobs.DYNAMIC_ITERATIONS_QUANTUM_CELLS
    pct = env_knobs.DYNAMIC_ITERATIONS_QUANTUM_PCT
    if cells != 2.0 or pct != 2.0:
        kwargs['quantum_cells'] = cells
        kwargs['quantum_pct'] = pct
    if env_knobs.DYNAMIC_ITERATIONS_GRACE > 0:
        kwargs['grace_tranches'] = env_knobs.DYNAMIC_ITERATIONS_GRACE
    return base, kwargs


def _note_dynamic_extension(iters: int, base: int,
                            print_prefix: str = "") -> None:
    """One line of attribution when a full search ran past the static cap."""
    if iters > base:
        _diag(f"{print_prefix}dynamic iterations (#529): search extended to "
              f"{iters} (base {base})")


def _probe_route_with_frontier(
    router: 'GridRouter',
    obstacles: 'GridObstacleMap',
    forward_sources: List,
    forward_targets: List,
    config: 'GridRouteConfig',
    print_prefix: str = "",
    direction_labels: Tuple[str, str] = ("forward", "backward"),
    track_margin=0,
    pcb_data: PCBData = None,
    current_net_id: int = -1,
    single_direction: bool = False
) -> Tuple[Optional[List], int, List, List, bool, int, int]:
    """Two-pass wrapper: route with the same-path via-spacing guard OFF (fast),
    and only re-route the same leg with the guard ON if the result actually drops
    two same-net vias closer than the drill floor (#230). The vast majority of
    legs never trip it, so they pay nothing; only the rare offender pays one extra
    route. Falls back to the first (DRC-flawed but connected) path if the guarded
    re-route fails to connect at all."""
    result = _probe_route_with_frontier_once(
        router, obstacles, forward_sources, forward_targets, config,
        print_prefix, direction_labels, track_margin, pcb_data, current_net_id,
        single_direction, via_exclusion_radius=0)
    path = result[0]
    if path is not None and _path_has_close_vias(path, config):
        ver = _via_drill_exclusion_radius(config)
        if ver > 0:
            retry = _probe_route_with_frontier_once(
                router, obstacles, forward_sources, forward_targets, config,
                print_prefix, direction_labels, track_margin, pcb_data, current_net_id,
                single_direction, via_exclusion_radius=ver)
            if retry[0] is not None:
                return retry  # re-routed with manufacturable via spacing
    return result


def _probe_route_with_frontier_once(
    router: 'GridRouter',
    obstacles: 'GridObstacleMap',
    forward_sources: List,
    forward_targets: List,
    config: 'GridRouteConfig',
    print_prefix: str = "",
    direction_labels: Tuple[str, str] = ("forward", "backward"),
    track_margin=0,
    pcb_data: PCBData = None,
    current_net_id: int = -1,
    single_direction: bool = False,
    via_exclusion_radius: int = 0
) -> Tuple[Optional[List], int, List, List, bool, int, int]:
    """
    Probe routing with fail-fast on stuck directions.

    Uses bidirectional probing to detect if either endpoint is blocked early,
    avoiding expensive full searches that will fail anyway.

    Args:
        router: GridRouter instance
        obstacles: Obstacle map
        forward_sources: Source cells for forward direction
        forward_targets: Target cells for forward direction
        config: Routing configuration
        print_prefix: Prefix for print messages (e.g., "  " or "      ")
        direction_labels: Names for forward/backward directions
        track_margin: Extra FRACTIONAL margin in grid cells for wide tracks
            (power/impedance widths); scalar or per-layer list (#156)
        pcb_data: Optional PCB data for blocking obstacle identification
        current_net_id: Current net ID (for excluding from blocking analysis)
        single_direction: If True, only try forward direction (for bus routing)

    Returns:
        (path, total_iterations, forward_blocked, backward_blocked, reversed_path, forward_iters, backward_iters)
        - path: The found path or None
        - total_iterations: Total iterations used
        - forward_blocked: Blocked cells from forward direction (for rip-up analysis)
        - backward_blocked: Blocked cells from backward direction (for rip-up analysis)
        - reversed_path: Whether path was found going backwards
        - forward_iters: Iterations used in forward direction
        - backward_iters: Iterations used in backward direction
    """
    first_label, second_label = direction_labels
    probe_iterations = config.max_probe_iterations
    _ver = via_exclusion_radius
    # #568: rung-aware via legality for THIS search only (set via a config
    # clone by the escalation retry; 0 = baseline, byte-identical in Rust).
    _vrung = getattr(config, 'via_rung', 0)

    # Track iterations per direction (first/second maps to forward/backward based on labels)
    first_total_iters = 0
    second_total_iters = 0

    # Probe forward direction
    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, probe_iterations, track_margin=track_margin, via_exclusion_radius=_ver, via_rung=_vrung,
        collinear_vias=env_knobs.COLLINEAR_VIAS)
    first_probe_iters = iterations
    first_total_iters = first_probe_iters
    first_blocked = blocked_cells
    total_iterations = first_probe_iters
    reversed_path = False

    # Track blocked cells for both directions
    forward_blocked = first_blocked
    backward_blocked = []

    # Helper to map first/second to forward/backward based on labels
    def get_fwd_bwd_iters():
        if first_label == "forward":
            return first_total_iters, second_total_iters
        else:
            return second_total_iters, first_total_iters

    if path is not None:
        # Found in first probe
        forward_blocked = []  # Success - clear blocked cells
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, reversed_path, fwd_iters, bwd_iters

    # For single_direction mode (bus routing), skip backward probe entirely
    if single_direction:
        first_reached_max = first_probe_iters >= probe_iterations
        if not first_reached_max:
            # Forward is stuck
            _diag(f"{print_prefix}{first_label} stuck ({first_probe_iters} < {probe_iterations}) [single-direction bus mode]")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            fwd_iters, bwd_iters = get_fwd_bwd_iters()
            return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

        # Forward probe reached max - do full search
        _diag(f"{print_prefix}Probe: {first_label}={first_probe_iters} iters [single-direction bus mode], trying full iterations...")
        _dyn_base, _dyn_kw = _dynamic_iterations(config)
        path, full_iters, full_blocked = router.route_with_frontier(
            obstacles, forward_sources, forward_targets, _dyn_base, track_margin=track_margin, via_exclusion_radius=_ver, via_rung=_vrung,
        collinear_vias=env_knobs.COLLINEAR_VIAS, **_dyn_kw)
        _note_dynamic_extension(full_iters, _dyn_base, print_prefix)
        first_total_iters += full_iters
        total_iterations += full_iters

        if path is not None:
            forward_blocked = []
        else:
            forward_blocked = full_blocked
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Probe backward direction (bidirectional mode)
    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, forward_targets, forward_sources, probe_iterations, track_margin=track_margin, via_exclusion_radius=_ver, via_rung=_vrung,
        collinear_vias=env_knobs.COLLINEAR_VIAS)
    second_probe_iters = iterations
    second_total_iters = second_probe_iters
    second_blocked = blocked_cells
    total_iterations += second_probe_iters
    backward_blocked = second_blocked

    if path is not None:
        # Found in second probe
        backward_blocked = []  # Success - clear blocked cells
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, True, fwd_iters, bwd_iters

    # Both probes failed to find a path - check if both reached max iterations
    # Only try full search if BOTH probes reached max-probe-iterations (meaning both directions are worth exploring)
    first_reached_max = first_probe_iters >= probe_iterations
    second_reached_max = second_probe_iters >= probe_iterations

    if not (first_reached_max and second_reached_max):
        # At least one probe didn't reach max - that direction is stuck, skip full search
        if not first_reached_max and not second_reached_max:
            _diag(f"{print_prefix}Both directions stuck ({first_label}={first_probe_iters}, {second_label}={second_probe_iters} < {probe_iterations})")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            _diagnose_blocked_start(obstacles, forward_targets, second_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
        elif not first_reached_max:
            _diag(f"{print_prefix}{first_label} stuck ({first_probe_iters} < {probe_iterations}), {second_label}={second_probe_iters}")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
        else:
            _diag(f"{print_prefix}{second_label} stuck ({second_probe_iters} < {probe_iterations}), {first_label}={first_probe_iters}")
            _diagnose_blocked_start(obstacles, forward_targets, second_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            # Print visual obstacle map around the stuck target
            if forward_targets and config.debug_lines:
                tgt = forward_targets[0]
                _print_obstacle_map(obstacles, tgt[0], tgt[1], tgt[2], radius=15, print_prefix=print_prefix)
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Both probes reached max iterations - do full search on forward direction
    _diag(f"{print_prefix}Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {first_label} with full iterations...")

    _dyn_base, _dyn_kw = _dynamic_iterations(config)
    path, full_iters, full_blocked = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, _dyn_base, track_margin=track_margin, via_exclusion_radius=_ver, via_rung=_vrung,
        collinear_vias=env_knobs.COLLINEAR_VIAS, **_dyn_kw)
    _note_dynamic_extension(full_iters, _dyn_base, print_prefix)
    first_total_iters += full_iters
    total_iterations += full_iters

    if path is not None:
        forward_blocked = []
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Forward failed, try backward
    _diag(f"{print_prefix}No route found after {full_iters} iterations ({first_label}), trying {second_label}...")
    forward_blocked = full_blocked

    path, backward_full_iters, backward_full_blocked = router.route_with_frontier(
        obstacles, forward_targets, forward_sources, _dyn_base, track_margin=track_margin, via_exclusion_radius=_ver, via_rung=_vrung,
        collinear_vias=env_knobs.COLLINEAR_VIAS, **_dyn_kw)
    _note_dynamic_extension(backward_full_iters, _dyn_base, print_prefix)
    second_total_iters += backward_full_iters
    total_iterations += backward_full_iters

    if path is not None:
        backward_blocked = []
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, True, fwd_iters, bwd_iters

    backward_blocked = backward_full_blocked
    fwd_iters, bwd_iters = get_fwd_bwd_iters()
    return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters


def _free_on_pad_cells(pad, layer_idx, config, obstacles, coord,
                       skip_cell=None):
    """Grid cells on `pad`'s copper that are FREE for tracks on `layer_idx`,
    keeping the whole track cross-section inside the copper (half-dims minus
    track/2) so a landing there adds no copper edge nearer any obstacle than
    the pad itself already has. Non-axis-aligned (rect_rotation) pads yield
    nothing. Part of the #479 blocked-terminal seeding (see callers)."""
    if getattr(pad, 'rect_rotation', 0.0):
        return []
    half_x = (pad.size_x or 0.0) / 2.0 - config.track_width / 2.0
    half_y = (pad.size_y or 0.0) / 2.0 - config.track_width / 2.0
    if half_x <= 0 or half_y <= 0:
        return []
    round_outline = pad.shape in ('circle', 'oval')
    gx0, gy0 = coord.to_grid(pad.global_x - half_x, pad.global_y - half_y)
    gx1, gy1 = coord.to_grid(pad.global_x + half_x, pad.global_y + half_y)
    cells = []
    for cgx in range(gx0, gx1 + 1):
        cx = cgx * coord.grid_step
        if abs(cx - pad.global_x) > half_x:
            continue
        for cgy in range(gy0, gy1 + 1):
            if skip_cell is not None and (cgx, cgy) == skip_cell:
                continue
            cy = cgy * coord.grid_step
            if abs(cy - pad.global_y) > half_y:
                continue
            if round_outline:
                rx = max(half_x, 1e-9)
                ry = max(half_y, 1e-9)
                if (((cx - pad.global_x) / rx) ** 2
                        + ((cy - pad.global_y) / ry) ** 2) > 1.0:
                    continue
            if obstacles.is_blocked(cgx, cgy, layer_idx):
                continue
            cells.append((cgx, cgy))
    return cells


def _augment_blocked_pad_terminals(endpoints, pcb_data, net_id, config,
                                   obstacles):
    """#479 (kbic65 stabilizer NPTH): when a PAD endpoint's grid cell is
    hard-blocked -- a foreign NPTH keep-out reaching over the pad centre --
    add the pad's FREE on-copper cells as extra endpoint cells, so the route
    can terminate on copper it is allowed to touch instead of failing at an
    unsteppable start. No-op for the overwhelmingly common case of a free
    endpoint cell. Endpoint rows are (gx, gy, layer_idx, orig_x, orig_y)."""
    coord = GridCoord(config.grid_step)
    out = list(endpoints)
    seen = set()
    for ep in endpoints:
        gx, gy, layer_idx, ox, oy = ep[0], ep[1], ep[2], ep[3], ep[4]
        try:
            if not obstacles.is_blocked(gx, gy, layer_idx):
                continue
        except Exception:
            continue
        for pad in pcb_data.pads_by_net.get(net_id, []):
            if abs(pad.global_x - ox) > 1e-3 or abs(pad.global_y - oy) > 1e-3:
                continue
            key = (round(pad.global_x, 4), round(pad.global_y, 4), layer_idx)
            if key not in seen:
                seen.add(key)
                free = _free_on_pad_cells(pad, layer_idx, config, obstacles,
                                          coord, skip_cell=(gx, gy))
                if free:
                    print(f"    endpoint cell blocked for {pad.component_ref}."
                          f"{pad.pad_number} -- seeded {len(free)} free "
                          f"on-pad cell(s)")
                    out.extend((cgx, cgy, layer_idx,
                                cgx * coord.grid_step, cgy * coord.grid_step)
                               for cgx, cgy in free)
            break
    return out


def _augment_all_blocked_pad_side(cells, pad, config, obstacles):
    """Multipoint-edge variant of the #479 blocked-terminal seeding: `cells`
    is one side's (gx, gy, layer_idx) set for a pad (possibly island-widened).
    Only when EVERY cell is blocked -- the route cannot start anywhere -- seed
    the pad's free on-copper cells on each candidate layer."""
    if pad is None or not cells:
        return cells
    try:
        if any(not obstacles.is_blocked(gx, gy, li) for gx, gy, li in cells):
            return cells
    except Exception:
        return cells
    coord = GridCoord(config.grid_step)
    extra = []
    for li in sorted({li for _, _, li in cells}):
        extra.extend((cgx, cgy, li) for cgx, cgy in
                     _free_on_pad_cells(pad, li, config, obstacles, coord))
    if extra:
        print(f"    endpoint cell blocked for {pad.component_ref}."
              f"{pad.pad_number} -- seeded {len(extra)} free on-pad cell(s)")
        return list({*cells, *extra})
    return cells


def route_oracle_links(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                       obstacles: GridObstacleMap, links,
                       attraction_path=None) -> Optional[dict]:
    """Route the plane-finalize oracle's EXACT remaining links (#572, fix
    direction 2). `links` are remaining_links-shaped endpoint pairs
    ((ax, ay, layer, kind), (bx, by, layer, kind)) whose coordinates are
    KiCad's own exact-fill nearest-approach anchors: (ax,ay) sits on cluster
    A's copper (pad/track) or fill ('zone'), likewise (bx,by), so copper
    welded to those floats joins the true clusters. Endpoint derivation is
    deliberately NOT used: the fill-model zone credit merges the two
    clusters into one component, which made every net-level retry of an
    oracle-punted link vacuously "succeed" with zero copper (measured on
    ghoul GND). Each link routes point-to-point via sources/targets
    overrides; a failure is returned VERBATIM (its blocked_cells_* feed the
    caller's frontier analysis, so the standard rip ladder aims at the real
    wall and retries re-enter here) -- which is why this lives in the main
    loop rather than the oracle's own (authority-less) link router.

    Returns a merged result dict over all links, or a failed result on the
    first link that cannot route. A 0-copper "success" is refused as failed:
    on an exact-fill-OPEN link it is the #572 false-weld fingerprint."""
    coord = GridCoord(config.grid_step)
    layer_idx = {name: i for i, name in enumerate(config.layers)}
    merged = None
    for link in links:
        (ax, ay, al, _ak), (bx, by, bl, _bk) = link
        la = layer_idx.get(al)
        lb = layer_idx.get(bl)
        if la is None or lb is None:
            print(f"    forced link endpoint layer not in the routing stack "
                  f"({al} / {bl}) -- link unroutable here")
            return {'failed': True, 'iterations': 0}
        gax, gay = coord.to_grid(ax, ay)
        gbx, gby = coord.to_grid(bx, by)
        print(f"    forced oracle link: ({ax:.3f},{ay:.3f})[{al}] <-> "
              f"({bx:.3f},{by:.3f})[{bl}]")
        result = route_net_with_obstacles(
            pcb_data, net_id, config, obstacles,
            attraction_path=attraction_path,
            sources_override=[(gax, gay, la, ax, ay)],
            targets_override=[(gbx, gby, lb, bx, by)])
        if not result or result.get('failed'):
            if merged and merged.get('new_segments'):
                print(f"    ({len(merged['new_segments'])} segment(s) from "
                      f"earlier forced link(s) discarded with this failure; "
                      f"a retry re-routes every link)")
            return result if result else {'failed': True, 'iterations': 0}
        if not (result.get('new_segments') or result.get('new_vias')):
            print(f"    forced link claimed success with ZERO copper -- "
                  f"refused (#572 false-weld fingerprint)")
            result['failed'] = True
            return result
        if merged is None:
            merged = result
        else:
            merged['new_segments'].extend(result.get('new_segments') or [])
            merged['new_vias'].extend(result.get('new_vias') or [])
            merged['iterations'] = (merged.get('iterations', 0)
                                    + result.get('iterations', 0))
            if result.get('path'):
                merged['path'] = (merged.get('path') or []) + result['path']
    return merged


def route_net_with_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              obstacles: GridObstacleMap,
                              attraction_path: Optional[List[Tuple[int, int, int]]] = None,
                              reverse_direction: bool = False,
                              bounds: Optional[Tuple[int, int, int, int]] = None,
                              sources_override: Optional[List[Tuple]] = None,
                              targets_override: Optional[List[Tuple]] = None) -> Optional[dict]:
    """Route a single net using pre-built obstacles (for incremental routing).

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Routing configuration
        obstacles: Pre-built obstacle map
        attraction_path: Optional path to attract to (for bus routing).
                        List of (gx, gy, layer) tuples from a previously routed neighbor.
        reverse_direction: If True, swap sources and targets (route from targets to sources).
                          Used for bus routing when the clique was formed by targets.
        bounds: Optional (gmin_x, gmin_y, gmax_x, gmax_y) grid-cell window the route
                must stay inside (the scoped net_rescue window, #396). When set,
                endpoints outside it are dropped (a long trunk segment pulled into a
                small window contributes a free end OUTSIDE the window bounds, and
                routing to it drags the A* through the un-modelled exterior), and the
                source/target overrides below are never stamped outside it -- so the
                window fence stays SOLID instead of being punched at the exempt cell.
        sources_override/targets_override: When BOTH are given, use them as the
                route's two sides instead of deriving them (get_net_endpoints
                row shape). The scoped rescue passes an anchor split here: the
                window crop severs copper, and the standard largest-two-groups
                derivation then aims at two fragments of the same trunk while
                the rescued island is dropped entirely.
    """
    # Find endpoints (segments or pads)
    if sources_override is not None and targets_override is not None:
        sources, targets, error = list(sources_override), list(targets_override), None
    else:
        sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    # Swap source/target for bus routing from clustered targets
    if reverse_direction:
        sources, targets = targets, sources

    # #479 (kbic65): a PAD endpoint whose own grid cell is hard-blocked -- a
    # neighboring footprint's stabilizer-NPTH keep-out reaching over the pad
    # centre -- can never take the first step, though free routable cells
    # exist ON the pad's own copper outside the keep-out (the human route
    # lands there). Seed those free on-pad cells as extra endpoint cells: the
    # route terminates on pad copper (a real connection, credited by
    # endpoint-in-pad / mid-span / KiCad alike) without laying copper through
    # the keep-out band. No-op when the endpoint cell is free.
    sources = _augment_blocked_pad_terminals(sources, pcb_data, net_id,
                                             config, obstacles)
    targets = _augment_blocked_pad_terminals(targets, pcb_data, net_id,
                                             config, obstacles)

    # Clamp endpoints to the scoped window: nothing outside the fence may be a
    # source/target, so the A* is never pulled past the fence into un-modelled space.
    if bounds is not None:
        bgx0, bgy0, bgx1, bgy1 = bounds
        sources = [s for s in sources if bgx0 <= s[0] <= bgx1 and bgy0 <= s[1] <= bgy1]
        targets = [t for t in targets if bgx0 <= t[0] <= bgx1 and bgy0 <= t[1] <= bgy1]
        if not sources or not targets:
            return None

    coord = GridCoord(config.grid_step)

    # #544: 2-pad fill anchors -- offer the far side's proven fill regions as
    # extra near-side terminal rows so a pour-covered partner doesn't force a
    # full-span route (the deriver above is fill-blind). Added BEFORE the
    # exemption/allowed/source-target marking so anchors flow through every
    # downstream consumer as ordinary endpoint rows; the start/end-original
    # lookup then welds to the anchor's own fill float (sub-cell bridge).
    # Scoped rescue keeps its exact override split untouched.
    if sources_override is None:
        _pl_src, _pl_tgt = _pour_launch_pair_anchors(
            pcb_data, net_id, sources, targets, config.layers, coord,
            config, bounds)
        if _pl_src or _pl_tgt:
            print(f"  POUR-LAUNCH pair anchors: +{len(_pl_src)} source, "
                  f"+{len(_pl_tgt)} target fill cells")
            sources = sources + _pl_src
            targets = targets + _pl_tgt

    # Endpoint stub-proximity exemption (soft-knobs C5): a target that sits
    # beside ANOTHER net's stub paid full stub cost on the final approach
    # cells; exempt a one-track disk around each endpoint, mirroring the
    # diff-pair caller. Cleared per net in prepare/restore_obstacles_inplace.
    _exempt_r = coord.to_grid_dist(config.track_width + config.clearance)
    obstacles.set_endpoint_exempt(
        [(s0[0], s0[1]) for s0 in sources] + [(t0[0], t0[1]) for t0 in targets],
        _exempt_r)
    layer_names = config.layers

    sources_grid = [(s[0], s[1], s[2]) for s in sources]
    targets_grid = [(t[0], t[1], t[2]) for t in targets]

    # Get stub free ends for proximity zone checking (where routing actually starts/ends)
    free_end_sources, free_end_targets, _ = get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=True)
    if free_end_sources:
        prox_check_sources = [(s[0], s[1], s[2]) for s in free_end_sources]
    else:
        prox_check_sources = sources_grid
    if free_end_targets:
        prox_check_targets = [(t[0], t[1], t[2]) for t in free_end_targets]
    else:
        prox_check_targets = targets_grid

    # Add source and target positions as allowed cells to override BGA zone blocking
    # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
    # When a window `bounds` is given, never exempt a cell in/beyond the fence: the
    # allowed/source-target overrides are the only thing that can breach the fence, so
    # keeping them strictly inside the window keeps the fence solid (#396).
    def _exempt_ok(gx, gy):
        return bounds is None or (bounds[0] <= gx <= bounds[2] and bounds[1] <= gy <= bounds[3])
    allow_radius = 10
    # The exemption block is a rectangle and `bounds` is a rectangle, so CLIP the
    # ranges once per terminal instead of testing every cell. Same cells in the
    # same order -- the predicate was a pure bounds test, so intersecting the
    # ranges admits exactly the set it admitted -- but it removes one Python call
    # per cell. Measured on rp2350: 510,339,830 _exempt_ok calls, 48.6 s, against
    # 723 s for the whole route; the 21x21 block per terminal is what makes it
    # the largest per-cell cost left after the underpad work (#786).
    for gx, gy, _ in sources_grid + targets_grid:
        if bounds is None:
            lo_x, hi_x = gx - allow_radius, gx + allow_radius
            lo_y, hi_y = gy - allow_radius, gy + allow_radius
        else:
            lo_x = gx - allow_radius
            if lo_x < bounds[0]:
                lo_x = bounds[0]
            hi_x = gx + allow_radius
            if hi_x > bounds[2]:
                hi_x = bounds[2]
            lo_y = gy - allow_radius
            if lo_y < bounds[1]:
                lo_y = bounds[1]
            hi_y = gy + allow_radius
            if hi_y > bounds[3]:
                hi_y = bounds[3]
        add_allowed = obstacles.add_allowed_cell
        for ax in range(lo_x, hi_x + 1):
            for ay in range(lo_y, hi_y + 1):
                add_allowed(ax, ay)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    # NOTE: Must pass layer to only allow override on the specific layer of the endpoint
    for gx, gy, layer in sources_grid + targets_grid:
        if _exempt_ok(gx, gy):
            obstacles.add_source_target_cell(gx, gy, layer)

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

    # Check which proximity zones the stub free ends are in for precise heuristic estimate
    src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_sources)
    src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_sources)
    tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_targets)
    tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_targets)
    prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
    if config.verbose:
        zones = []
        if src_in_stub: zones.append("src:stub")
        if src_in_bga: zones.append("src:bga")
        if tgt_in_stub: zones.append("tgt:stub")
        if tgt_in_bga: zones.append("tgt:bga")
        print(f"  proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

    # Calculate bus attraction parameters
    bus_attraction_radius_grid = coord.to_grid_dist(config.bus_attraction_radius) if config.bus_attraction_radius > 0 else 0
    bus_attraction_bonus = config.scaled_cell_units(config.bus_attraction_bonus) if config.bus_attraction_bonus > 0 else 0

    # Cross-layer fraction of the bus attraction bonus (#296 R9 phase B):
    # keeps a planned corridor guiding a member across via transitions.
    # Experimental default 35% when bus routing is on; override with
    # KICAD_BUS_XLAYER_PCT (0 = legacy same-layer-only attraction).
    bus_xlayer_pct = 0
    # #589 owner attraction: plan corridors change layers (~1 via/net in a
    # negotiated plan), so cross-layer pull must arm for plan-attracted
    # nets too, not only bus members.
    if bus_attraction_bonus > 0 and (getattr(config, 'bus_enabled', False)
                                     or env_knobs.GLOBAL_PLAN.get('attract')):
        try:
            bus_xlayer_pct = env_knobs.BUS_XLAYER_PCT
        except ValueError:
            bus_xlayer_pct = 35

    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=config.via_proximity_cost_int(),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=prox_h_cost,
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost,
                        attraction_radius=bus_attraction_radius_grid,
                        attraction_bonus=bus_attraction_bonus,
                        attraction_cross_layer_pct=bus_xlayer_pct,
                        attraction_potential=env_knobs.GLOBAL_PLAN.get('attract_potential', 0))

    # Set attraction path for bus routing (if provided)
    if attraction_path:
        router.set_attraction_path(attraction_path)
        if config.verbose:
            layers_in_path = set(p[2] for p in attraction_path)
            print(f"    Bus attraction: {len(attraction_path)} path points, layers={layers_in_path}, radius={bus_attraction_radius_grid} grid, bonus={bus_attraction_bonus}")

    # Per-layer fractional track margins (#156): the net's extra half-width --
    # power override OR impedance layer width -- over what the obstacle stamps
    # reserved, exact (no ceil, no +1; the swept capsule covers diagonals).
    track_margin = config.track_margins_for_net(net_id)

    # Determine direction order (always deterministic)
    start_backwards = config.direction_order in ("backwards", "backward")

    # Set up forward/backward based on direction preference
    if start_backwards:
        forward_sources, forward_targets = targets_grid, sources_grid
        direction_labels = ("backward", "forward")
    else:
        forward_sources, forward_targets = sources_grid, targets_grid
        direction_labels = ("forward", "backward")

    # Use probe routing helper
    # For bus routing with reverse_direction, use single-direction mode to ensure
    # routes start from the clustered endpoints (where attraction can guide them)
    use_single_direction = reverse_direction
    if config.verbose:
        print(f"    GridRouter sources: {forward_sources[:3]}{'...' if len(forward_sources) > 3 else ''}")
        print(f"    GridRouter targets: {forward_targets[:3]}{'...' if len(forward_targets) > 3 else ''}")
        if use_single_direction:
            print(f"    Bus routing: single-direction mode (start from clustered endpoints)")
    (path, total_iterations, forward_blocked, backward_blocked, reversed_path,
     fwd_iters, bwd_iters, necked_down, uniform_width, unblock_vias,
     unblock_segments) = _route_with_via_unblock(
        router, obstacles, config, forward_sources, forward_targets, track_margin,
        pcb_data, net_id, print_prefix="", direction_labels=direction_labels,
        single_direction=use_single_direction
    )

    # Adjust reversed_path based on start direction
    if start_backwards and path is not None:
        reversed_path = not reversed_path

    if path is None:
        dir_msg = "single direction" if use_single_direction else "both directions"
        _diag(f"No route found after {total_iterations} iterations ({dir_msg})")
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': fwd_iters,
            'iterations_backward': bwd_iters,
        }

    _diag(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    # Collect and print stats if enabled
    if config.collect_stats:
        # Re-run with stats collection on the same direction that succeeded
        # Use the actual source/target that worked
        if reversed_path:
            stats_sources, stats_targets = forward_targets, forward_sources
        else:
            stats_sources, stats_targets = forward_sources, forward_targets
        _stats_base, _stats_kw = _dynamic_iterations(config)
        _, _, stats = router.route_multi(
            obstacles, stats_sources, stats_targets, _stats_base, track_margin=track_margin,
            collinear_vias=env_knobs.COLLINEAR_VIAS, **_stats_kw)
        print_route_stats(stats)

    if reversed_path:
        sources, targets = targets, sources

    path_start = path[0]
    path_end = path[-1]

    start_original = None
    for s in sources:
        if s[0] == path_start[0] and s[1] == path_start[1] and s[2] == path_start[2]:
            start_original = (s[3], s[4], layer_names[s[2]])
            break

    end_original = None
    for t in targets:
        if t[0] == path_end[0] and t[1] == path_end[1] and t[2] == path_end[2]:
            end_original = (t[3], t[4], layer_names[t[2]])
            break

    # Get through-hole pad positions for this net (layer transitions without via)
    through_hole_positions = get_same_net_through_hole_positions(pcb_data, net_id, config)

    # Simplify path by removing collinear intermediate points
    path = simplify_path(path)

    new_segments = []
    new_vias = []

    if start_original:
        first_grid_x, first_grid_y = coord.to_float(path_start[0], path_start[1])
        orig_x, orig_y, orig_layer = start_original
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = coord.to_float(gx1, gy1)
        x2, y2 = coord.to_float(gx2, gy2)

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if (gx1, gy1) not in through_hole_positions:
                _vsz, _vdr = _emit_via_size(pcb_data, gx1, gy1, config,
                                            net_id=net_id, x=x1, y=y1)
                via = Via(
                    x=x1, y=y1,
                    size=_vsz,
                    drill=_vdr,
                    layers=["F.Cu", "B.Cu"],  # Always through-hole
                    net_id=net_id
                )
                new_vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                layer_name = layer_names[layer1]
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.get_net_track_width(net_id, layer_name),
                    layer=layer_name,
                    net_id=net_id
                )
                new_segments.append(seg)

    if end_original:
        last_grid_x, last_grid_y = coord.to_float(path_end[0], path_end[1])
        orig_x, orig_y, orig_layer = end_original
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.get_net_track_width(net_id, orig_layer),
                layer=orig_layer,
                net_id=net_id
            )
            new_segments.append(seg)

    if necked_down:
        # Both endpoints are pads: neck the start side too
        new_segments = _apply_neckdown_widths(new_segments, config, net_id, obstacles,
                                              coord, layer_names, track_margin, neck_start=True)
    elif uniform_width is not None:
        # Short power edge routed at a stepped-down width: every segment is that
        # width, so the obstacle map (reads seg.width) and the output match (#180).
        for _s in new_segments:
            _s.width = uniform_width

    # Neck any terminal-connection segment that grazes a foreign pad (#157): the
    # endpoint stub is laid geometrically with the endpoint region obstacle-exempt,
    # so a full-width terminal can sit sub-clearance to a neighbouring foreign pad.
    # The returned `hard` list is the terminal-bridge SHORT gate (ux pf8/pf9):
    # terminal copper whose centreline overlaps a foreign track/via even at the
    # fab floor is a shipped short no neck can fix -- fail the route instead,
    # so callers (rescue rungs, reroutes, rip ladders) fall to their next
    # option. Clearance-only grazes stay in the neck regime.
    term_pts = [coord.to_float(path[0][0], path[0][1]), coord.to_float(path[-1][0], path[-1][1])]
    if start_original:
        term_pts.append((start_original[0], start_original[1]))
    if end_original:
        term_pts.append((end_original[0], end_original[1]))
    _necked157, _hard157 = _neck_terminal_grazes(new_segments, term_pts,
                                                 pcb_data, net_id, config)
    # #589: a plan probe's result is a hint, never shipped copper -- its
    # terminals legitimately overlap future nets' stubs (the probe map
    # excluded them), so the short gate must not veto the prediction.
    if _hard157 and not config.plan_probe:
        _hs, _hd = _hard157[0]
        print(f"  {YELLOW}terminal copper on {_hs.layer} would OVERLAP a "
              f"foreign track/via (edge dist {_hd:.3f}mm < floor half-width) "
              f"-- rejecting the route rather than shipping a short{RESET}")
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': [],
            'blocked_cells_backward': [],
            'iterations_forward': fwd_iters,
            'iterations_backward': bwd_iters,
        }

    # Fab-floor via dropped inside a boxed source/target pad to unblock this route
    # (issue #189); connects the inner-layer path end to the pad by copper overlap.
    new_vias = list(new_vias) + unblock_vias
    # #535 off-pad escape rung: the pad->via stub ships with its via.
    new_segments = list(new_segments) + unblock_segments

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
    }


# ---------------------------------------------------------------------------
# Guide corridor (waypoint) routing (issue #7)
# ---------------------------------------------------------------------------

def build_corridor_waypoints(pcb_data: PCBData, config: GridRouteConfig) -> List[Tuple[int, int]]:
    """Convert user-drawn guide polylines into ordered grid waypoint cells.

    By default the waypoints are just the endpoints of each drawn line segment
    (the polyline vertices); A* routes near-straight between them, hugging the
    drawn line. If guide_corridor_spacing > 0, long segments are subdivided so
    no two consecutive waypoints are farther apart than that spacing (useful to
    follow a curve more tightly). Returns [] when no guide paths are present.
    """
    if not getattr(config, 'guide_corridor_enabled', False) or not pcb_data.guide_paths:
        return []

    coord = GridCoord(config.grid_step)
    spacing_mm = getattr(config, 'guide_corridor_spacing', 0.0) or 0.0
    spacing = coord.to_grid_dist(spacing_mm) if spacing_mm > 0 else 0

    cells: List[Tuple[int, int]] = []
    for gp in pcb_data.guide_paths:
        pts = list(gp.points)
        if gp.is_closed and len(pts) >= 2:
            pts.append(pts[0])
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            g1 = coord.to_grid(x1, y1)
            g2 = coord.to_grid(x2, y2)
            cells.append(g1)
            # Optionally subdivide a long segment into intermediate waypoints.
            if spacing > 0:
                seg_len = max(abs(g2[0] - g1[0]), abs(g2[1] - g1[1]))
                n = seg_len // spacing
                for k in range(1, int(n) + 1):
                    t = (k * spacing) / seg_len if seg_len else 0
                    if t >= 1.0:
                        break
                    cells.append((round(g1[0] + t * (g2[0] - g1[0])),
                                  round(g1[1] + t * (g2[1] - g1[1]))))
        cells.append(coord.to_grid(*pts[-1]))  # final vertex of this chain

    # Drop consecutive duplicates
    out: List[Tuple[int, int]] = []
    for c in cells:
        if not out or out[-1] != c:
            out.append(c)
    return out


def _cell_margin_clear(obstacles, x, y, layer, margin):
    """True if (x, y) and every cell within `margin` (Chebyshev) is unblocked on layer."""
    if margin <= 0:
        return not obstacles.is_blocked(x, y, layer)
    for ox in range(-margin, margin + 1):
        for oy in range(-margin, margin + 1):
            if obstacles.is_blocked(x + ox, y + oy, layer):
                return False
    return True


def _nearest_free_cell(obstacles, gx, gy, num_layers, max_radius=80, margin=0):
    """BFS for the nearest (gx, gy) unblocked on at least one layer.

    When margin > 0, the cell only qualifies if every cell within `margin`
    (Chebyshev) is also unblocked on that layer, so a track centered there
    clears nearby obstacles instead of clipping them (grid quantization).
    Falls back to margin=0 if no clearer cell is found, so it never fails to
    return when something is free. Returns (gx, gy, layer) or None.
    """
    from collections import deque

    def clear_on_layer(x, y, layer):
        return _cell_margin_clear(obstacles, x, y, layer, margin)

    q = deque([(gx, gy)])
    seen = {(gx, gy)}
    fallback = None  # nearest cell free at margin=0, used if no margin-clear cell exists
    while q:
        x, y = q.popleft()
        for layer in range(num_layers):
            if not obstacles.is_blocked(x, y, layer):
                if fallback is None:
                    fallback = (x, y, layer)
                if clear_on_layer(x, y, layer):
                    return (x, y, layer)
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nx, ny = x + dx, y + dy
            if (nx, ny) not in seen and abs(nx - gx) + abs(ny - gy) <= max_radius:
                seen.add((nx, ny))
                q.append((nx, ny))
    return fallback


def _route_leg(router, obstacles, config, sources, targets, track_margin, pcb_data, net_id):
    """Route one leg (sources -> targets). Returns (path, iterations).

    The path is normalized to run from a source to a target (the bidirectional
    probe may return it reversed), so legs chain correctly end-to-start.
    """
    path, iters, _fb, _bb, reversed_path, _fi, _bi = _probe_route_with_frontier(
        router, obstacles, sources, targets, config,
        print_prefix="      ", track_margin=track_margin,
        pcb_data=pcb_data, current_net_id=net_id)
    if path is not None and reversed_path:
        path = path[::-1]
    return path, iters


def _point_segment_dist2(px, py, ax, ay, bx, by):
    """Squared distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / float(dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def assign_waypoints_to_mst_edges(waypoints, pad_grid, mst_edges):
    """Bucket corridor waypoints onto the MST edge each is nearest to (issue #7).

    Each MST segment then follows the contiguous run of waypoints "in its middle";
    a waypoint lands in exactly one bucket, so once a segment uses it the others
    need not care. Corridor order is preserved within each bucket.

    Args:
        waypoints: ordered list of (gx, gy) grid waypoints.
        pad_grid: list of (gx, gy) grid positions indexed by pad index.
        mst_edges: list of (idx_a, idx_b, dist) MST edges.

    Returns:
        dict mapping frozenset({idx_a, idx_b}) -> ordered list of (gx, gy).
    """
    buckets = {frozenset((ia, ib)): [] for ia, ib, _ in mst_edges}
    if not mst_edges:
        return buckets
    for (wx, wy) in waypoints:
        best_key, best_d = None, None
        for ia, ib, _ in mst_edges:
            ax, ay = pad_grid[ia]
            bx, by = pad_grid[ib]
            d = _point_segment_dist2(wx, wy, ax, ay, bx, by)
            if best_d is None or d < best_d:
                best_d, best_key = d, frozenset((ia, ib))
        buckets[best_key].append((wx, wy))
    return buckets


# Issue #180: a SHORT power-net connection (span <= this, ~max_search_radius) that
# can't fit the full power width is routed at the widest width that DOES fit --
# full -> /2 -> /4 -> ... -> fab floor -- instead of failing. This lets a power
# ball daisy-chain to an adjacent same-net ball with a thin escape, while long
# trunks keep the full power width (current capacity).
SHORT_POWER_EDGE_MM = 10.0


def _impedance_neckdown_allowed():
    """#156 follow-up: may an impedance-width net neck down to the NOMINAL
    track width when its full-width route fails? Completion-first default:
    ALLOW (a nominal-width segment with slightly-off impedance beats a failed
    net). Set KICAD_IMPEDANCE_NECKDOWN=0 to forbid (strict impedance: the net
    fails instead of narrowing). Env-gated experiment -- flag promotion is
    tracked on #465."""
    return env_knobs.IMPEDANCE_NECKDOWN


def _track_margin_for_width(width, layer_width, grid_step):
    """Extra FRACTIONAL grid-cell margin the A* needs for a track of `width`
    over the map's reserved `layer_width` (#156): the exact extra half-width.
    Callers add the #268 stamp-shell quantization guard on top.

    Deliberately NOT lattice-snapped, unlike the single-ended margin
    (routing_config._snap_to_lattice_reach). The snap was tried here (#505) and
    reverted on evidence: this margin feeds the PLANE width-upgrade ladders in
    plane_region_connector / kicad_oracle, where `wide_route_clear` re-checks
    the widened route against the real geometry before accepting it -- so the
    margin is a search heuristic, not the correctness gate. Snapping raised it
    by up to +3 cells on wide straps and bought nothing: sechzig's plane repair
    graded IDENTICALLY (8 kicad / 6 check_drc / 9 connection_width) while
    losing trunk copper (2.0mm straps 9 -> 4, 1.6mm 8 -> 6, absorbed into
    0.8mm). Pure cost. The single-ended path has no such re-check, which is why
    the snap belongs there and not here."""
    extra_half = (width - layer_width) / 2
    return extra_half / grid_step if extra_half > 0 else 0.0


def _margin_at(track_margin, layer_idx):
    """Per-layer value of a track margin that may be a scalar (uniform) or a
    per-layer list (#156 impedance widths)."""
    if isinstance(track_margin, (list, tuple)):
        return track_margin[layer_idx] if 0 <= layer_idx < len(track_margin) else 0.0
    return float(track_margin)


def _power_width_ladder(net_width, layer_width):
    """Widths BELOW the full power width to try, descending: net/2, net/4, ... down
    to the fab floor (layer_width). The full width is tried first by the caller."""
    floor = layer_width
    out = []
    w = net_width / 2
    while w > floor + 1e-9:
        out.append(w)
        w /= 2
    if net_width > floor + 1e-9:
        out.append(floor)
    return out


# _via_rung_retry REMOVED 2026-08-05: the mid-retry "Via rung retry" that
# re-searched a failing wide-net edge with a one-step-smaller via at floor
# width. Measured on a full set5 wave (ab_rip0805/new4 _replay.log): 459
# firings, ~2 wins (<~1% hit rate), with attempts concentrated on the slow
# tail boards (ecp5 236, sechzig 220) where each firing was an expensive
# doomed re-search. Terminal geometry escalation in batch_route (the
# post-rescue whole-net width+via ladder toward the fab floor) replaces it
# at the only point it ever paid: a net that would otherwise ship failed or
# open. The boxed-endpoint rung search below (_rung_search_pair /
# _register_rung_path_vias, #568/#189) is a DIFFERENT mechanism and stays.


def _register_rung_path_vias(pcb_data, obstacles, path, vs, dr):
    """#568: register the small (dia, drill) for the path's via cells where
    the FULL size is blocked -- those transitions exist only by rung-1
    legality, so emission must ship the small rung (#339 re-validates).
    Cells legal at full size are NOT registered (they ship the configured
    via; registering small there would shrink vias needlessly)."""
    sizes = getattr(pcb_data, '_unblock_via_sizes', None)
    if sizes is None:
        sizes = pcb_data._unblock_via_sizes = {}
    for a, b in zip(path, path[1:]):
        if a[2] != b[2] and obstacles.is_via_blocked(a[0], a[1]):
            sizes.setdefault((a[0], a[1]), (vs, dr))


def _rung_search_pair(config, pcb_data):
    """The (dia, drill) a via_rung=1 search would ship, or None when rust
    mode (KICAD_VIA_RUNG=2) is off or no smaller fab rung exists. Shared
    selection rule with the obstacle-cache dual stamping by construction."""
    from obstacle_cache import _small_via_pair
    return _small_via_pair(config, pcb_data)


def _edge_span_mm(sources, targets, grid_step):
    """Min source->target span (mm): a cheap 'is this a short escape vs a trunk' proxy."""
    best = float('inf')
    for s in sources:
        for t in targets:
            best = min(best, math.hypot(s[0] - t[0], s[1] - t[1]))
    return best * grid_step


def _place_shrunk_via_in_pad(pad_obj, obstacles, config, pcb_data, net_id,
                             coord, layer_names):
    """FAILURE-ONLY memo over the #189 in-pad via placement (2026-08-14
    profiling: 2,693 attempts x a windowed plane-map build each, ~95s --
    the same boxed pads re-fail across rescue rungs and rounds).

    Only None results are cached, which is monotone-safe: a geometric
    "no via fits" is pure in (pad, net, via geometry, copper epoch) -- rips
    that could un-fail it bump the epoch -- and the _via_unblock_failed set
    only ever ADDS Nones, so it cannot invalidate a cached failure.
    Successes are never cached (they carry mutable Via/Segment objects and
    the tap path's note_clearance_used side effect -- the exact two hazards
    that sank the full-result memo, reverted same day). Bails to the impl
    when phase-3 in-flight copper is pending: pending copper changes
    placement legality WITHOUT an epoch bump, so a failure under inflight
    state must not be trusted later."""
    from plane_pad_tap import inflight_copper_dicts
    _iv, _isg = inflight_copper_dicts(pcb_data)
    _inflight = bool(_iv) or bool(_isg)
    _memo = None
    if not _inflight:
        _memo = getattr(pcb_data, '_via_place_fail_memo', None)
        if _memo is None:
            _memo = pcb_data._via_place_fail_memo = set()
        _mkey = (net_id, pad_obj.component_ref, pad_obj.pad_number,
                 pad_obj.global_x, pad_obj.global_y,
                 getattr(pcb_data, '_copper_epoch', 0),
                 config.via_size, config.via_drill, config.clearance,
                 config.board_edge_clearance, config.same_net_pad_clearance,
                 tuple(layer_names))
        if _mkey in _memo:
            return None
    r = _place_shrunk_via_in_pad_impl(pad_obj, obstacles, config, pcb_data,
                                      net_id, coord, layer_names)
    if r is None and _memo is not None:
        _memo.add(_mkey)
    return r


def _place_shrunk_via_in_pad_impl(pad_obj, obstacles, config, pcb_data, net_id, coord, layer_names):
    """Issue #189: drop a DRC-legal fab-floor via INSIDE a boxed-in SMD pad so a
    stuck A* can reach the pad on an inner layer. Returns
    (Via, (gx, gy), pad_layer_idx, stub_segments) or None; stub_segments is
    [] for an in-pad via.

    Escalation rung (PR #535's idea): when no in-pad via fits -- or #581's
    same-net pad via clearance forbids via-in-pad entirely -- search
    KICAD_ESCAPE_STUB_RADIUS mm (default 1.0; 0 disables) around the pad for
    the first spot where a through-via is legal on every layer and inject a
    pad->via escape stub. Same tap machinery, nonzero search radius: the tap
    routes the pad->via trace at fine grid / capped clearance, and its via map
    honors #581 automatically (the off-pad via keeps the required distance
    from same-net pads while the TRACE may leave the pad).

    Uses the dedicated local via-obstacle map at EXACT clearance (the ae2069
    plane-tap machinery), NOT the big routing grid, which carries an extra search
    margin and over-blocks a via that is actually fab-legal. Escalates the via
    down the fab floors (deduped by diameter -- the dimension that decides the
    fit) so a tighter pad still gets the largest via that fits. Failures are
    memoised per (net, pad) on pcb_data so a genuinely-boxed pad pays the
    (windowed) board scan at most once per run instead of every reroute pass.
    """
    # SMD pads only: a through-hole pad already reaches every layer, so a stuck
    # route there is not a layer-access problem a via-in-pad would fix.
    if pad_obj is None or getattr(pad_obj, 'drill', 0):
        return None
    # #581: an active (> 0) same-net pad via clearance forbids the IN-PAD arm;
    # the off-pad escape-stub arm below is the compliant rescue.
    _allow_in_pad = getattr(config, 'same_net_pad_clearance', -1.0) <= 0
    _escape_radius = max(0.0, env_knobs.ESCAPE_STUB_RADIUS)
    if not _allow_in_pad and _escape_radius <= 0:
        return None
    if hasattr(pad_obj, 'layers') and '*.Cu' in pad_obj.layers:
        return None
    pad_layer = next((l for l in pad_obj.layers
                      if l.endswith('.Cu') and not l.startswith('*')), None)
    if pad_layer is None or pad_layer not in layer_names:
        return None

    cache = getattr(pcb_data, '_via_unblock_failed', None)
    if cache is None:
        cache = set()
        pcb_data._via_unblock_failed = cache
    key = (net_id, round(pad_obj.global_x, 3), round(pad_obj.global_y, 3))
    if key in cache:
        return None

    from plane_pad_tap import tap_pad_with_escalation, inflight_copper_dicts
    from list_nets import fab_floor_ladder, warn_fab_escalation

    # Board-edge floor for the in-pad via (#448): the old blanket
    # board_edge_clearance=0.0 let the via ring land closer to the milled
    # outline than even the pad's own copper (crkbd rJ1 VBUSR: ring 0.141mm
    # from the edge, sub-fab). Demand the normal edge clearance, RELAXED to
    # the pad's own copper-to-outline distance when the pad itself overhangs
    # the band -- the via then never adds edge exposure beyond what the
    # placed (edge-exempt) pad already establishes.
    _edge_eff = (config.board_edge_clearance if config.board_edge_clearance > 0
                 else config.clearance)
    try:
        from check_drc import (board_edge_geometry, _point_to_rings_distance,
                               _pad_perimeter_points)
        _rings, _, _ = board_edge_geometry(pcb_data.board_info)
        if _rings:
            _pad_edge_d = min(_point_to_rings_distance(px, py, _rings)
                              for px, py in _pad_perimeter_points(pad_obj))
            _edge_eff = min(_edge_eff, max(0.0, _pad_edge_d))
    except Exception:  # noqa: BLE001 -- fall back to the plain edge clearance
        pass
    # Copper stamped in the working obstacle map but not yet committed to
    # pcb_data (phase-3 tap rip-up windows) must block this via too, or it is
    # drilled straight through a pending foreign track (#310, snapdragon
    # ETH_ISOLATEB via on PCIE1_WAKE_N In2.Cu).
    inflight_vias, inflight_segments = inflight_copper_dicts(pcb_data)
    ncu = len([l for l in layer_names if l.endswith('.Cu')]) or 2
    # Forced last-resort via sizes, largest first: the configured via, then the
    # active fab-tier floor ladder (nominal floor, then any escalation rung). The
    # advanced rung is the more-costly small via 'standard' escalates to (#237).
    ladder = fab_floor_ladder(ncu)
    candidates = [(config.via_size, config.via_drill, False)]
    candidates += [(f['via_diameter'], f['via_drill'], i > 0)
                   for i, f in enumerate(ladder)]
    via_pairs, seen_dia, escalated_pair = [], set(), set()
    for vd, dr, is_esc in candidates:
        vd, dr = round(vd, 3), round(dr, 3)
        if dr < vd <= config.via_size + 1e-9 and vd not in seen_dia:
            seen_dia.add(vd)
            via_pairs.append((vd, dr))
            if is_esc:
                escalated_pair.add((vd, dr))
    # Rung order: in-pad (radius 0, unless #581 forbids it), then the off-pad
    # escape stub (PR #535) at KICAD_ESCAPE_STUB_RADIUS. The off-pad rung runs
    # the SAME fab ladder -- a tighter pocket may only fit the smaller via.
    _radii = ([0.0] if _allow_in_pad else []) + \
             ([_escape_radius] if _escape_radius > 0 else [])
    tap_res = None
    _used_radius = 0.0
    for _radius in _radii:
        for vd, dr in via_pairs:
            # try_default=False: skip the default-parameter pass and go straight to
            # the fine (grid 0.05 / capped clearance) placement. Each pass builds via
            # + routing obstacle maps (the profiled bottleneck, ~0.5s each in Rust);
            # the default pass is redundant here -- fine uses min(grid)/min(clearance)
            # so it is strictly >= permissive, and a via-in-pad needs no via->pad
            # trace, so the fine pass never fails where the default would succeed.
            tap_res = tap_pad_with_escalation(
                pad_obj, pad_layer, net_id, pcb_data,
                replace(config, via_size=vd, via_drill=dr,
                        board_edge_clearance=_edge_eff),
                max_search_radius=_radius, via_size=vd, via_drill=dr,
                extra_vias=inflight_vias, extra_segments=inflight_segments,
                try_default=False, fine_for_all=True,
                distant_trace_radius=0.0, disable_reuse=True)
            if tap_res.success and tap_res.via is not None:
                # mm-exact re-check at FULL clearance vs CURRENT copper (#339): the
                # tap's fine pass places at capped clearance, which approved a 0.45
                # via 39um short of a fellow-ripped net's fresh track (cynthion
                # MEZZANINE6 vs MEZZANINE5). A graze at this size falls through to
                # the next (smaller) ladder rung, whose tap may also relocate it.
                _tv = tap_res.via
                if _unblock_via_refit(pcb_data, net_id, _tv['x'], _tv['y'],
                                      (_tv['size'], _tv['drill']), config) != (_tv['size'], _tv['drill']):
                    tap_res = None
                    continue
                if (vd, dr) in escalated_pair:
                    warn_fab_escalation(f"last-resort via for net {net_id} ({vd}/{dr}mm)")
                _used_radius = _radius
                break
        if tap_res is not None and tap_res.success and tap_res.via is not None:
            break
    if tap_res is None or not tap_res.success or tap_res.via is None:
        # Don't memoise a failure caused (possibly) by TRANSIENT in-flight
        # copper: once that window closes the pad may be genuinely tappable.
        if not (inflight_vias or inflight_segments):
            cache.add(key)
            # #331 item 3 (#189): name the committed copper that boxed this
            # pad. The A* frontier never reaches copper directly under the
            # pad (the ottercast SDC0_D3 In1 trace), so frontier attribution
            # blames adjacent bystanders while the decisive blocker stays
            # invisible. The rip-up ladder consumes this to rip the keystone.
            from plane_blocker_detection import find_via_position_blocker
            blocker = find_via_position_blocker(
                pad_obj.global_x, pad_obj.global_y, pcb_data, config,
                net_id, protected_net_ids={0}, quiet=True)
            if blocker is not None:
                blame = getattr(pcb_data, '_via_unblock_blame', None)
                if blame is None:
                    blame = {}
                    pcb_data._via_unblock_blame = blame
                blame.setdefault(net_id, set()).add(blocker)
        return None
    v = tap_res.via
    via = Via(x=v['x'], y=v['y'], size=v['size'], drill=v['drill'],
              layers=v.get('layers', [layer_names[0], layer_names[-1]]), net_id=net_id)
    vgx, vgy = coord.to_grid(v['x'], v['y'])
    # Record this cell's DRC-legal shrunk via size so route conversion emits THAT
    # size (not the full config.via_size) if the path later changes layer here
    # through the registered free via. The free via is what lets the boxed pad
    # connect; a full via at the cell grazes a neighbouring foreign pad (only the
    # shrunk via fits) -- issue #212, glasgow_revC Z5 via vs RN4.6.
    # #589 probe hygiene: plan probes route on a throwaway map and emit no
    # copper, but this registry lives on pcb_data and steers the REAL run's
    # via emission -- seq probes place ~10x more unblock vias than blind
    # ones and the leak measurably changed routing (oc null-control 37 vs
    # 33 with the plan off). Probes keep the unblock via for their own
    # path; they just must not leave a persistent size registration.
    if not getattr(config, 'plan_probe', False):
        sizes = getattr(pcb_data, '_unblock_via_sizes', None)
        if sizes is None:
            sizes = {}
            pcb_data._unblock_via_sizes = sizes
        sizes[(vgx, vgy)] = (v['size'], v['drill'])
    # Off-pad rung: the tap's pad->via trace is the escape stub -- it ships
    # with the via (both kept or both dropped by the caller's used-via check).
    stub_segments = []
    if _used_radius > 0:
        for sd in (tap_res.segments or []):
            stub_segments.append(Segment(
                start_x=sd['start'][0], start_y=sd['start'][1],
                end_x=sd['end'][0], end_y=sd['end'][1],
                width=sd['width'], layer=sd['layer'], net_id=net_id))
    return via, (vgx, vgy), layer_names.index(pad_layer), stub_segments


def _pad_via_conflict_cells(pcb_data, pad, config, coord, layer_names):
    """Frontier-style blocked cells ON the foreign copper that vetoes a
    fab-floor via in `pad` (#424 rip-integrated terminal access).

    The placement validator computes exactly which copper conflicts with the
    via ring -- micron-accurate -- then throws the identity away and returns
    None. Emit synthetic (gx, gy, layer) cells on that copper so the EXISTING
    tap rip-up attribution names and rips the hugging net; the retry re-runs
    the pad-via rung, where placement then succeeds in the freed space. The
    frontier report can't provide this: in a saturated pocket the search dies
    against the first wall, not the copper touching the pad."""
    from list_nets import fab_floor_min
    li_of = {l: i for i, l in enumerate(layer_names)}
    ncu = (len(pcb_data.board_info.copper_layers)
           if pcb_data.board_info.copper_layers else len(layer_names))
    try:
        vmin = fab_floor_min(ncu)['via_diameter']
    except Exception:
        vmin = config.via_size
    need = vmin / 2.0 + config.clearance
    px, py = pad.global_x, pad.global_y
    cells = []
    for s in pcb_data.segments:
        if s.net_id == pad.net_id or s.layer not in li_of:
            continue
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((px - s.start_x) * dx
                                                   + (py - s.start_y) * dy) / L2))
        cx, cy = s.start_x + dx * t, s.start_y + dy * t
        if math.hypot(px - cx, py - cy) < need + (s.width or 0.0) / 2.0:
            g = coord.to_grid(cx, cy)
            cells.append((g[0], g[1], li_of[s.layer]))
    for v in pcb_data.vias:
        if v.net_id == pad.net_id:
            continue
        if math.hypot(v.x - px, v.y - py) < need + v.size / 2.0:
            g = coord.to_grid(v.x, v.y)
            cells.extend((g[0], g[1], li) for li in range(len(layer_names)))
    return cells


def _net_pad_near(pcb_data, net_id, cells, coord):
    """The net's SMD pad that one of the grid `cells` sits inside (a boxed
    endpoint), or None. Tight in-pad test so a tap source on a mid-trace point
    (not a pad) returns None and never gets a spurious via.

    Scans EVERY endpoint cell, not just cells[0]: a boxed side often lists a
    stub tip first (ottercast Net-(C61-Pad1): tip at (114.3, 90.0), 0.1mm
    OUTSIDE the U6.20 pad), and the cells[0]-only lookup returned None there --
    so the #189 via-in-pad unblock silently never fired for a pad the
    placement machinery could in fact tap (a hand-routed 0.45/0.2 via-in-pad
    connects it trivially)."""
    pads = _net_pads_near(pcb_data, net_id, cells, coord)
    return pads[0] if pads else None


def _net_pads_near(pcb_data, net_id, cells, coord):
    """All of the net's SMD pads that some grid cell in `cells` sits inside,
    nearest-first. The via-in-pad second rung iterates these: the closest pad
    (e.g. a cap pad crowded by its neighbours) may refuse a via while another
    endpoint pad of the same side (ottercast C69.1) accepts one."""
    found = {}
    for cell in cells:
        x, y = coord.to_float(cell[0], cell[1])
        for p in pcb_data.pads_by_net.get(net_id, []):
            if getattr(p, 'drill', 0):
                continue
            if (abs(p.global_x - x) <= p.size_x / 2 + 0.05 and
                    abs(p.global_y - y) <= p.size_y / 2 + 0.05):
                d = abs(p.global_x - x) + abs(p.global_y - y)
                if id(p) not in found or d < found[id(p)][0]:
                    found[id(p)] = (d, p)
    # Geometric tie-break: equidistant pads fell back to `found` insertion
    # order (the pad walk), which follows board order.
    return [p for _d, p in sorted(found.values(),
                                  key=lambda t: (t[0], t[1].global_x, t[1].global_y,
                                                 str(t[1].pad_number)))]


def _register_unblock_via(obstacles, vgx, vgy, layer_names):
    """Expose a placed via cell on every layer and let the router transit/place a
    free via there (so the retry A* can reach the pad through it)."""
    obstacles.add_free_via(vgx, vgy)
    for li in range(len(layer_names)):
        obstacles.add_source_target_cell(vgx, vgy, li)
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            obstacles.add_allowed_cell(vgx + dx, vgy + dy)


def _route_with_via_unblock(router, obstacles, config, sources, targets, track_margin,
                            pcb_data, net_id, print_prefix="",
                            direction_labels=("forward", "backward"), single_direction=False,
                            waypoints=None):
    """_route_main_connection plus a via-in-pad unblock (issue #189), generic over
    single-ended, multipoint-main and tap routes.

    If the route fails because an ENDPOINT pad is boxed in -- its probe frontier
    is exhausted BELOW the probe limit (the "stuck (N < 5000)" signal), meaning
    the A* walled itself in at that pad rather than running out of budget -- drop
    a DRC-legal fab-floor via INSIDE that pad so the SAME A* can reach it on an
    open inner layer, and retry. Returns _route_main_connection's 9-tuple plus a
    trailing list of the vias actually used (to append to the caller's output).
    """
    res = _route_main_connection(router, obstacles, config, sources, targets, track_margin,
                                 pcb_data, net_id, print_prefix, direction_labels,
                                 single_direction, waypoints)
    if res[0] is not None:
        return res + ([], [])
    layer_names = config.layers
    if len(layer_names) < 2:
        return res + ([], [])
    fwd_i, bwd_i = res[5], res[6]
    lim = config.max_probe_iterations
    coord = GridCoord(config.grid_step)

    # #568 rust mode: before COMMITTING copper (the #189 pre-placed in-pad
    # via), let the search itself try the small rung -- a boxed pad is often
    # boxed only at the configured via reserve, and a rung-1 via the A* places
    # where it wants beats a forced in-pad via (no free-via cost softness, no
    # windowed board scan, no IPC-4761 via-in-pad note). Capped at the probe
    # budget, same fail-fast rationale as the placement retry below.
    # Pre-placement remains the fallback for genuinely small-rung-boxed pads.
    if (((bwd_i and bwd_i < lim) or (fwd_i and fwd_i < lim))
            and obstacles.get_stats()[7] > 0):
        _rp = _rung_search_pair(config, pcb_data)
        if _rp is not None:
            _rcfg = replace(config, via_rung=1,
                            max_iterations=config.max_probe_iterations)
            _r1 = _route_main_connection(
                router, obstacles, _rcfg, sources, targets, track_margin,
                pcb_data, net_id, print_prefix, direction_labels,
                single_direction, waypoints)
            if _r1[0] is not None:
                print(f"{print_prefix}{GREEN}Boxed endpoint unblocked by "
                      f"rung-{_rp[0]}/{_rp[1]} via search (no pre-placed "
                      f"via){RESET}")
                # #589 probe hygiene: no persistent registration from plan
                # probes (see _place_shrunk_via_in_pad_impl).
                if not getattr(config, 'plan_probe', False):
                    _register_rung_path_vias(pcb_data, obstacles, _r1[0],
                                             _rp[0], _rp[1])
                return _r1 + ([], [])

    _dbg = _unblock_debug()
    placed = []  # (via, vgx, vgy, pad_layer_idx)
    new_sources, new_targets = sources, targets
    # backward probe (from targets) exhausted -> the TARGET pad is boxed
    if bwd_i and bwd_i < lim:
        pad = _net_pad_near(pcb_data, net_id, targets, coord)
        if _dbg:
            print(f"      UNBLOCK: bwd stuck ({bwd_i}<{lim}), target pad="
                  f"{pad.component_ref}.{pad.pad_number}" if pad else
                  f"      UNBLOCK: bwd stuck ({bwd_i}<{lim}), NO pad at targets")
        r = (_place_shrunk_via_in_pad(pad, obstacles, config, pcb_data, net_id, coord, layer_names)
             if pad is not None else None)
        if _dbg and pad is not None:
            print(f"      UNBLOCK: placement {'OK ' + str(r[0]) if r else 'DECLINED'}")
        if r is not None:
            via, (vgx, vgy), pli, stub_segs = r
            _register_unblock_via(obstacles, vgx, vgy, layer_names)
            new_targets = list(targets) + [(vgx, vgy, li) for li in range(len(layer_names))]
            placed.append((via, vgx, vgy, pli, pad, stub_segs))
    # forward probe (from sources) exhausted -> the SOURCE pad is boxed
    if fwd_i and fwd_i < lim:
        pad = _net_pad_near(pcb_data, net_id, sources, coord)
        r = (_place_shrunk_via_in_pad(pad, obstacles, config, pcb_data, net_id, coord, layer_names)
             if pad is not None else None)
        if r is not None:
            via, (vgx, vgy), pli, stub_segs = r
            _register_unblock_via(obstacles, vgx, vgy, layer_names)
            new_sources = list(sources) + [(vgx, vgy, li) for li in range(len(layer_names))]
            placed.append((via, vgx, vgy, pli, pad, stub_segs))
    if not placed:
        return res + ([], [])

    # Cap the retry's full A* at the same budget as the stuck threshold we trigger
    # on (max_probe_iterations): if dropping the via opened the pad, the inner
    # layer beside it is clear and the route is found within the probe budget;
    # if it isn't, grinding to the full max_iterations (1e6 at grid 0.05) just to
    # fail again is wasted -- fail fast and report the pad honestly.
    if _dbg:
        for (via, vgx, vgy, pli, pad, _ss) in placed:
            for li in range(len(layer_names)):
                nb = sum(1 for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx or dy)
                         and obstacles.is_blocked(vgx + dx, vgy + dy, li))
                print(f"      UNBLOCK map: via@({vgx},{vgy}) {layer_names[li]}: "
                      f"cell_blocked={obstacles.is_blocked(vgx, vgy, li)} neighbors_blocked={nb}/8")
    retry_config = replace(config, max_iterations=config.max_probe_iterations)
    res2 = _route_main_connection(router, obstacles, retry_config, new_sources, new_targets, track_margin,
                                  pcb_data, net_id, print_prefix, direction_labels,
                                  single_direction, waypoints)
    if res2[0] is None:
        # Second rung: the stuck side got its via but the retry still failed --
        # the OTHER side often needs layer access too. Its probe burns the full
        # budget wandering the walled outer layer ("forward=5000", not stuck),
        # so it never qualifies above; if it terminates on an SMD pad, via it
        # as well and retry once at a doubled budget. ottercast Net-(C61-Pad1):
        # the hand-routed fix is exactly a via in C69.1 AND U6.20 joined on an
        # inner layer -- one-ended unblock can never find that route.
        second = []
        if new_sources is sources:  # source side never got a via
            side_cells, is_source = sources, True
        elif new_targets is targets:  # target side never got a via
            side_cells, is_source = targets, False
        else:
            side_cells = None
        if side_cells is not None:
            for pad2 in _net_pads_near(pcb_data, net_id, side_cells, coord):
                r2 = _place_shrunk_via_in_pad(pad2, obstacles, config, pcb_data,
                                              net_id, coord, layer_names)
                if _dbg:
                    print(f"      UNBLOCK rung2: {'source' if is_source else 'target'} "
                          f"pad {pad2.component_ref}.{pad2.pad_number} -> "
                          f"{'OK' if r2 else 'DECLINED'}")
                if r2 is None:
                    continue
                via2, (vgx2, vgy2), pli2, stub_segs2 = r2
                _register_unblock_via(obstacles, vgx2, vgy2, layer_names)
                ext = [(vgx2, vgy2, li) for li in range(len(layer_names))]
                if is_source:
                    new_sources = list(side_cells) + ext
                else:
                    new_targets = list(side_cells) + ext
                placed.append((via2, vgx2, vgy2, pli2, pad2, stub_segs2))
                second.append(via2)
                break
        if second:
            retry_config = replace(config, max_iterations=2 * config.max_probe_iterations)
            res2 = _route_main_connection(router, obstacles, retry_config, new_sources, new_targets,
                                          track_margin, pcb_data, net_id, print_prefix,
                                          direction_labels, single_direction, waypoints)
    if res2[0] is None:
        # The via placed but didn't rescue the route. Memoise the pad as failed so
        # route_multipoint_taps' next rip-reroute pass doesn't redo the expensive
        # placement + retry for it -- without this the unblock is re-attempted
        # every pass for a pad it can't help (the cap above makes that more likely).
        cache = pcb_data._via_unblock_failed
        for (_v, _gx, _gy, _pli, pad, _ss) in placed:
            cache.add((net_id, round(pad.global_x, 3), round(pad.global_y, 3)))
        return res + ([], [])  # unblock didn't help; report the original failure
    # Keep only the vias the retry actually used: the new path must terminate on
    # the via cell at a NON-pad layer (it reached the pad through the via). Drop
    # any the route didn't need, so no floating copper is added.
    p2 = res2[0]
    ends = (p2[0], p2[-1])
    used, used_stub_segs, n_offpad = [], [], 0
    for (via, vgx, vgy, pli, pad, stub_segs) in placed:
        if any(e[0] == vgx and e[1] == vgy and (e[2] != pli or stub_segs)
               for e in ends):
            used.append(via)
            used_stub_segs.extend(stub_segs)
            if stub_segs:
                n_offpad += 1
    # #803: COMMIT the kept unblock copper to pcb_data now.
    #
    # _register_unblock_via is purely permissive (free-via + a source/target
    # override on every layer + an 11x11 allowed block) and the via was not in
    # pcb_data, because a multipoint tap result is deferred. Until that result
    # commits the barrel therefore blocks NOBODY. Measured on glasgow_revC at
    # the moment a phase-3 ripped net re-routes:
    #     in_pcb_data=0  via_blocked=True  cells=T,F,F,F   (only the PAD blocks)
    #     in_pcb_data=1  via_blocked=True  cells=T,T,T,T   (committed)
    # so a foreign track routes straight through it on an inner/back layer --
    # /RD over /~{ALERT}'s 0.30/0.15 via, 0.100mm centre-to-axis against 0.250mm
    # to touch, four contact violations.
    #
    # Commit rather than STAMP. A raw add_vias_list_as_obstacles stamp is not
    # net-aware: the obstacle map cannot tell the via's owner from a stranger,
    # so the stamp blocks the net that placed it (its own later MST edges), and
    # it double-counts when the caller's commit recomputes the net's cache --
    # measured, +10588 blocked_cells unaccounted and a broken net. #309's
    # in-progress rings say the same thing in their docstring: the rings are
    # per-route scaffolding because "the committed route's vias get their real
    # keep-outs from the net's recomputed obstacle cache". The cache IS the
    # net-aware channel -- prepare_obstacles_inplace lifts the owner's own
    # copper and leaves it for everyone else -- so put the via where the cache
    # will find it.
    #
    # Safe against double-add: add_route_to_pcb_data dedupes by id() (#195), so
    # the caller's later commit skips objects already present.
    if used and not getattr(config, 'plan_probe', False):
        # plan_probe: probes route on a throwaway map and must emit no copper
        # (the same rule the _unblock_via_sizes registry above obeys -- a leak
        # there measurably changed routing, oc null-control 37 vs 33).
        _have_v = {id(v) for v in pcb_data.vias}
        for _v in used:
            if id(_v) not in _have_v:
                pcb_data.vias.append(_v)
        _have_s = {id(sg) for sg in pcb_data.segments}
        for _sg in used_stub_segs:
            if id(_sg) not in _have_s:
                pcb_data.segments.append(_sg)
        pcb_data._copper_epoch = getattr(pcb_data, '_copper_epoch', 0) + 1
    if used:
        _off = (f" ({n_offpad} off-pad escape stub(s), #535)" if n_offpad else "")
        print(f"{print_prefix}{GREEN}Via-in-pad unblock: dropped {len(used)} fab-floor "
              f"via(s) to reach a boxed endpoint{_off}{RESET}")
    return res2 + (used, used_stub_segs)


def _route_main_connection(router, obstacles, config, sources, targets, track_margin,
                           pcb_data, net_id, print_prefix="",
                           direction_labels=("forward", "backward"), single_direction=False,
                           waypoints=None):
    """Route sources->targets; wide routes that fail retry narrow (issue #72/#180).

    "Wide" covers POWER-configured nets (neck floor = the layer routing width)
    and, when KICAD_IMPEDANCE_NECKDOWN allows (default yes, #465),
    IMPEDANCE-width nets (neck floor = the nominal track width).

    Same return shape as _route_connection_at_margin plus a trailing
    (necked_down, uniform_width):
      - necked_down=True (long trunk): the wide route failed and it re-routed at the
        neck floor; the caller necks down the segments near the pad
        (_neck_width_for_net picks the floor per net class).
      - uniform_width=W (short edge, necked_down=False): a short wide edge routed
        at width W (full -> /2 -> ... -> neck floor); the caller sets EVERY segment
        to W so the trace -- and the obstacle map (which reads seg.width) and the
        written output -- is genuinely that width, not the configured width.
      - both None/False: full width, no rewidthing.
    """
    result = _route_connection_at_margin(
        router, obstacles, config, sources, targets, track_margin,
        pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
    # Two net classes may enter the neck-down ladder when the full-width route
    # fails (#156):
    #   - POWER-wide nets (configured wider than the layer's routing width):
    #     neck toward the LAYER width, exactly as before.
    #   - IMPEDANCE-width nets (layer width above the nominal track_width):
    #     neck toward the NOMINAL width -- completion over strict impedance;
    #     default ALLOW, forbid with KICAD_IMPEDANCE_NECKDOWN=0 (#465).
    # Gate on width comparisons, not margin>0 (#156 gives impedance nets a
    # nonzero track_margin too).
    net_w = config.get_net_track_width(net_id, config.layers[0])
    layer_w = config.get_track_width(config.layers[0])
    power_wide = net_w > layer_w + 1e-9
    imp_wide = (not power_wide and net_w > config.track_width + 1e-9
                and _impedance_neckdown_allowed())
    if result[0] is not None or not (power_wide or imp_wide) or not config.power_tap_neckdown:
        return result + (False, None)
    neck_floor = layer_w if power_wide else config.track_width

    if _edge_span_mm(sources, targets, config.grid_step) <= SHORT_POWER_EDGE_MM:
        # Short edge: step the width down, widest-that-fits wins; segments use it.
        total_iters = result[1]
        for w in _power_width_ladder(net_w, neck_floor):
            tm = config.track_margins_for_width(w)
            r = _route_connection_at_margin(
                router, obstacles, config, sources, targets, tm,
                pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
            total_iters += r[1]
            if r[0] is not None:
                print(f"{print_prefix}{YELLOW}Wide {'power' if power_wide else 'impedance'} route blocked - routed short edge at "
                      f"{w:.4f}mm (down from {net_w:.4f}){RESET}")
                return (r[0], total_iters) + r[2:] + (False, w)
        # (The mid-retry _via_rung_retry that used to fire here was removed
        # 2026-08-05 -- see the note above _rung_search_pair.)
        return (result[0], total_iters) + result[2:] + (False, None)

    # Long trunk: keep the existing single wide->narrow retry + neck-down.
    # Power necks to the LAYER width (base_track_margins: the impedance extra
    # on impedance runs, 0 on plain runs); an impedance net necks to the
    # NOMINAL width (its margins vs the stamps' reserve, usually all-zero).
    print(f"{print_prefix}{YELLOW}Wide route blocked - retrying at default track width (neck-down){RESET}")
    retry_margins = (config.base_track_margins() if power_wide
                     else config.track_margins_for_width(config.track_width))
    retry = _route_connection_at_margin(
        router, obstacles, config, sources, targets, retry_margins,
        pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
    if retry[0] is None:
        # Keep the WIDE attempt's frontier for rip-up analysis: blockers found
        # by the narrow frontier only help a narrow route, but ripped nets
        # re-route at their own wide width and can fail entirely.
        # (The mid-retry _via_rung_retry that used to fire here was removed
        # 2026-08-05 -- see the note above _rung_search_pair.)
        return (result[0], result[1] + retry[1]) + result[2:] + (False, None)
    return (retry[0], result[1] + retry[1]) + retry[2:] + (True, None)


def _route_connection_at_margin(router, obstacles, config, sources, targets, track_margin,
                                pcb_data, net_id, print_prefix="",
                                direction_labels=("forward", "backward"), single_direction=False,
                                waypoints=None):
    """Route sources->targets, steering through the guide corridor (issue #7).

    A drop-in replacement for _probe_route_with_frontier with the SAME return
    shape. When a guide corridor is configured (config.corridor_waypoints) it
    routes sources -> waypoints -> targets as concatenated A* legs; otherwise it
    behaves exactly like _probe_route_with_frontier.

    The waypoints only steer the path BETWEEN the given sources and targets -
    endpoint/pad/MST selection is the caller's and is left untouched. It is
    strictly best-effort: a waypoint that can't be reached (or that would strand
    the target) is dropped, and if no waypoints can be followed it falls back to
    the direct sources->targets route. So a corridor can never make a connection
    fail that would otherwise route, and the worst it can do is be ignored.
    """
    def direct():
        return _probe_route_with_frontier(
            router, obstacles, sources, targets, config,
            print_prefix=print_prefix, direction_labels=direction_labels,
            track_margin=track_margin, pcb_data=pcb_data, current_net_id=net_id,
            single_direction=single_direction)

    # `waypoints` may be a per-segment bucket (multi-point MST edge); when not
    # given, fall back to the whole corridor (single-segment / 2-pad nets).
    if waypoints is None:
        waypoints = getattr(config, 'corridor_waypoints', None)
    # Bus routing (single_direction) has its own neighbor attraction; leave it be.
    if not waypoints or single_direction or not sources or not targets:
        return direct()

    # Legs use the SAME track_margin the direct route would - inflating it can make
    # a leg unroutable where a direct route succeeds, violating "a corridor never
    # makes a route worse than no corridor".
    num_layers = len(config.layers)
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    snap_margin = max(1, int(math.ceil((net_track_width / 2 + config.clearance) / config.grid_step)))

    # Orient waypoints to enter at the end nearest the sources.
    def d2(a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
    s0 = sources[0]
    wp = list(waypoints)
    if d2(s0, wp[0]) > d2(s0, wp[-1]):
        wp.reverse()

    # Drop waypoints sitting essentially on this segment's own endpoints. The user
    # usually draws the guide starting/ending at the pads, but a waypoint right next
    # to a pad is redundant (the route reaches the pad anyway) and, because the pad's
    # clearance halo blocks the current layer there, can force a needless via/detour.
    endpoint_cells = list(sources) + list(targets)
    skip_d = 2 * snap_margin
    wp = [(wx, wy) for (wx, wy) in wp
          if not any(abs(wx - c[0]) + abs(wy - c[1]) <= skip_d for c in endpoint_cells)]
    if not wp:
        return direct()

    def _waypoint_cells(wgx, wgy, prefer_layer):
        """Candidate target cell(s) for a waypoint.

        Each leg is an independent A* that can't see the via cost at a leg
        boundary, so switching layers between waypoints would leave spurious
        vias. Once the route is committed to a layer we therefore only steer to a
        waypoint that's clear on THAT layer; if it isn't, we skip the waypoint
        (return []) rather than change layers - the leg to the next waypoint still
        follows the corridor, and a leg only vias when it genuinely must. Before a
        layer is committed (e.g. a through-hole start spanning layers) we pick any
        clear layer, snapping to the nearest clear cell if the vertex isn't clear.
        """
        if prefer_layer is not None:
            if _cell_margin_clear(obstacles, wgx, wgy, prefer_layer, snap_margin):
                return [(wgx, wgy, prefer_layer)]
            return []
        free = [(wgx, wgy, L) for L in range(num_layers)
                if _cell_margin_clear(obstacles, wgx, wgy, L, snap_margin)]
        if free:
            return free
        nf = _nearest_free_cell(obstacles, wgx, wgy, num_layers, margin=snap_margin)
        return [nf] if nf is not None else []

    spine: List[Tuple[int, int, int]] = []
    total = 0
    current = list(sources)
    # checkpoints[i] = (len(spine), current_sources) after accepting i waypoints.
    checkpoints = [(0, list(sources))]

    def _extend(path):
        if spine and spine[-1] == path[0]:
            spine.extend(path[1:])
        else:
            spine.extend(path)

    for (wgx, wgy) in wp:
        # The committed layer is the one all current sources share (a single cell,
        # or e.g. tap points all on the same track layer); None until committed
        # (a through-hole start spans both). _waypoint_cells keeps the route on the
        # committed layer (skipping waypoints not clear there) to avoid boundary vias.
        cur_layers = set(c[2] for c in current)
        prefer_layer = next(iter(cur_layers)) if len(cur_layers) == 1 else None
        tgts = _waypoint_cells(wgx, wgy, prefer_layer)
        if not tgts:
            continue
        path, iters = _route_leg(router, obstacles, config, current, tgts,
                                 track_margin, pcb_data, net_id)
        total += iters
        if path is None:
            continue  # waypoint unreachable from here -> drop it
        _extend(path)
        current = [path[-1]]
        checkpoints.append((len(spine), current))

    if len(checkpoints) == 1:
        # No waypoints could be placed/followed -> behave exactly like no corridor.
        return direct()

    # Reach the targets, backing off trailing waypoints if the approach is stranded.
    while True:
        path, iters = _route_leg(router, obstacles, config, current, targets,
                                 track_margin, pcb_data, net_id)
        total += iters
        if path is not None:
            _extend(path)
            break
        if len(checkpoints) > 1:
            checkpoints.pop()
            trunc, current = checkpoints[-1]
            del spine[trunc:]
            continue
        # No waypoints could be followed to the target; use the authoritative
        # direct route (also returns blocking info the caller needs for rip-up).
        return direct()

    kept = len(checkpoints) - 1
    dropped = len(wp) - kept
    if dropped > 0:
        print(f"{print_prefix}Guide corridor: followed {kept}/{len(wp)} waypoints "
              f"(dropped {dropped} that would have blocked this segment)")
    elif kept > 0:
        print(f"{print_prefix}Guide corridor: following {kept} waypoint(s)")

    return (spine, total, [], [], False, total, 0)


def _pad_all_layer_reach(pcb_data: PCBData, pad_obj) -> bool:
    """True when the pad is reachable on EVERY routing layer: a through-hole
    pad ('*.Cu'), or an SMD pad with a same-net via whose barrel covers the
    pad center (a stub-layer-switch or via-in-pad escape via). Arriving at
    that cell on ANY layer lands the connection -- if the target set does
    not say so, a route ending on the via under the pad reads as a miss and
    the pad stays 'failed' with its own escape sitting right there."""
    if pad_obj is None or not hasattr(pad_obj, 'layers'):
        return False
    if '*.Cu' in pad_obj.layers:
        return True
    nid = getattr(pad_obj, 'net_id', None)
    if not nid:
        return False
    px, py = pad_obj.global_x, pad_obj.global_y
    for v in pcb_data.vias:
        if v.net_id == nid and \
                math.hypot(v.x - px, v.y - py) <= (v.size or 0) / 2 + 1e-3:
            return True
    return False


def _dense_fanout_refs(pcb_data: PCBData) -> set:
    """References of high-density fanned-out packages (BGA/QFN/QFP with a
    real ball/pin field). Computed once per board and cached on pcb_data --
    package classification walks every footprint's pads."""
    refs = getattr(pcb_data, '_dense_fanout_refs', None)
    if refs is None:
        from kicad_parser import detect_package_type
        refs = {ref for ref, fp in pcb_data.footprints.items()
                if len(fp.pads) >= 16
                and detect_package_type(fp) in ('BGA', 'QFN', 'QFP')}
        pcb_data._dense_fanout_refs = refs
    return refs


def _pour_launch_region_cells(pcb_data, net_id, pad_info, pad_components,
                              layer_names, coord, island_cells):
    """#544/#562 POUR-LAUNCH ladder v3: per-REGION fill launch cells.

    Every pad-covered fill REGION of this net's zones contributes laddered
    launch cells keyed to the component its covered pads belong to. The
    fill-aware grouping (check_net_connectivity) already unioned those pads
    with their region, so the keying is exact -- no majority vote, no
    cross-region credit: attaching to a region unions you with THAT
    region's component only, and the MST plans the region's own join edge.

    Regions covering NO terminal follow the zone's own island policy:
    under island_removal_mode 0 (KiCad's default, remove isolated) they
    are phantom copper -- never a node, never attach surface. Modes 1/2
    keep them at fill time; their pseudo-terminal nodehood is deferred
    (counted + noted, not offered).

    Per straggler, expanding rungs (1/2.5/6/15 mm); candidates = the
    region's fill cells scored dist + fragility (strait/boundary attach
    pays up to _FRAG_MM; fill deeper than _DEEP_MM is free) plus, when
    KICAD_POUR_LAUNCH_COPPER=1, the owning component's existing copper
    cells (no fragility penalty); first rung with any candidate wins,
    best _NBEST kept.

    Returns {component_id: {(gx, gy, layer_idx): (fx, fy)}} for the caller
    to merge into its island-cell map (Phase 1's _island_cells AND Phase
    3's _p3_island_cells -- v2 only fed Phase 1, so the straggler edges
    Phase 3 routes never saw fill targets at all). Default ON;
    KICAD_POUR_LAUNCH=0 disables; {} when off or unavailable.
    """
    import os as _os
    if not env_knobs.POUR_LAUNCH:
        return {}
    # Same-invocation memo: Phase 1 and Phase 3 call this back-to-back with
    # the IDENTICAL pad_components object (main_result carries it), and every
    # rescue reroute of a big plane net repeats the pair -- the scan is pure
    # in (net, grouping), so keying on (net_id, id(pad_components), #pads)
    # halves the ladder cost per invocation and per reroute.
    _mk = (net_id, id(pad_components), len(pad_info))
    _memo = getattr(pcb_data, '_pour_launch_memo', None)
    # The memo HOLDS pad_components (id() alone is unsafe: a freed dict's id
    # can be reused by a different grouping -- the memo's strong ref pins it).
    if (_memo is not None and _memo[0] == _mk
            and _memo[1] is pad_components):
        return _memo[2]
    try:
        from plane_fill_model import get_zone_model
        from plane_fragility import _erode_depth
        import numpy as _np
    except Exception:
        return {}
    _RUNGS = (1.0, 2.5, 6.0, 15.0)   # mm
    _NBEST = 12
    try:
        _FRAG_MM = env_knobs.POUR_LAUNCH_FRAG
    except ValueError:
        _FRAG_MM = 1.0
    _COPPER_RUNGS = env_knobs.POUR_LAUNCH_COPPER
    _DEEP_MM = 0.5
    out = {}
    _bare_kept = 0
    try:
        _zones = [z for z in (pcb_data.zones or []) if z.net_id == net_id
                  and z.layer in layer_names]
        for _z in _zones:
            _m = get_zone_model(pcb_data, _z)
            if _m is None:
                continue
            _li = layer_names.index(_z.layer)
            # region -> covered terminal indices
            _cov = {}
            for _idx, _info in enumerate(pad_info):
                _c = _m.query_component(_info[3], _info[4])
                if _c and _c > 0:
                    _cov.setdefault(_c, []).append(_idx)
            # bare-region census (island policy; nodehood deferred)
            _labels = set(int(x) for x in _np.unique(_m.labels) if x > 0)
            for _lb in _labels - set(_cov):
                if getattr(_z, 'island_removal_mode', 0) in (1, 2):
                    _bare_kept += 1
            for _lb, _idxs in _cov.items():
                from collections import Counter as _Ctr
                _comp = _Ctr(pad_components.get(_i, _i)
                             for _i in _idxs).most_common(1)[0][0]
                _cells_out = out.setdefault(_comp, {})
                _copper = []
                if _COPPER_RUNGS:
                    _copper = [(_fx, _fy, _k[2]) for _k, (_fx, _fy) in
                               (island_cells.get(_comp) or {}).items()]
                _stragglers = [(_info[3], _info[4])
                               for _i, _info in enumerate(pad_info)
                               if pad_components.get(_i, _i) != _comp]
                _dc = max(1, int(round(_DEEP_MM / _m.cell)))
                _depth_cache = {}
                for _sx, _sy in _stragglers:
                    _found = []
                    for _r in _RUNGS:
                        for _fx, _fy, _fl in _copper:
                            _d2 = (_fx - _sx) ** 2 + (_fy - _sy) ** 2
                            if _d2 <= _r * _r:
                                _found.append((_d2 ** 0.5, _fx, _fy, _fl))
                        _i0 = max(0, int((_sx - _r - _m.x0) / _m.cell) - _dc)
                        _i1 = min(_m.nx, int((_sx + _r - _m.x0) / _m.cell) + 1 + _dc)
                        _j0 = max(0, int((_sy - _r - _m.y0) / _m.cell) - _dc)
                        _j1 = min(_m.ny, int((_sy + _r - _m.y0) / _m.cell) + 1 + _dc)
                        if _i0 < _i1 and _j0 < _j1:
                            _ck = (_i0, _i1, _j0, _j1)
                            _dep = _depth_cache.get(_ck)
                            _win = _m.labels[_i0:_i1, _j0:_j1]
                            if _dep is None:
                                _dep = _erode_depth(_win == _lb, _dc)
                                _depth_cache[_ck] = _dep
                            # labels is [x, y] (query_component's indexing)
                            _xi, _yi = _np.nonzero(_win == _lb)
                            for _a, _b in zip(_xi, _yi):
                                _fx = _m.x0 + float(_a + _i0) * _m.cell
                                _fy = _m.y0 + float(_b + _j0) * _m.cell
                                _d2 = (_fx - _sx) ** 2 + (_fy - _sy) ** 2
                                if _d2 > _r * _r:
                                    continue
                                _dmm = float(_dep[_a, _b]) * _m.cell
                                _frag = max(0.0, 1.0 - _dmm / _DEEP_MM)
                                _found.append(
                                    (_d2 ** 0.5 + _FRAG_MM * _frag,
                                     _fx, _fy, _li))
                        if _found:
                            break
                    _found.sort()
                    for _sc, _fx, _fy, _fl in _found[:_NBEST]:
                        _g = coord.to_grid(_fx, _fy)
                        _k = (_g[0], _g[1], _fl)
                        if _k not in _cells_out:
                            _cells_out[_k] = (_fx, _fy)
    except Exception as _e:
        print(f"  POUR-LAUNCH unavailable ({_e})")
        return {}
    if _bare_kept:
        print(f"  POUR-LAUNCH: {_bare_kept} bare fill region(s) kept by the "
              f"zone island policy are not yet nodes (deferred)")
    pcb_data._pour_launch_memo = (_mk, pad_components, out)
    return out


def _pour_launch_pair_anchors(pcb_data, net_id, sources, targets,
                              layer_names, coord, config, bounds=None):
    """#544: fill anchors for the 2-group endpoint path.

    The endpoint deriver is fill-blind: a route to a pad that already sits
    on this net's pour crosses the whole span even though reaching the
    pour's fill ANYWHERE would connect it. Offer laddered cells of the FAR
    side's fill regions as extra terminal rows for the near side,
    region-honest like the multipoint ladder: a region is offered only when
    a far-side terminal provably touches it, probed exactly like the
    fill-aware checker (query_component at the terminal float, size = the
    pad's copper for pad rows -- a thermal-relieved pad's centre cell is
    eroded but its spokes within the radius prove contact -- else the track
    width for stub rows). Each anchor row carries the fill cell's own float
    as its weld point, so the terminal bridge the converter draws is
    sub-cell and lands ON fill copper -- never the any-angle slash back to
    the distant pad that the multipoint owner map exists to prevent.

    When both sides carry regions, a route may weld anchor to anchor: a
    short region-to-region joint connecting the pads through their pours.

    Returns (extra_source_rows, extra_target_rows) in the get_net_endpoints
    row shape (gx, gy, layer_idx, fx, fy). Default ON; KICAD_POUR_LAUNCH=0
    disables; ([], []) when off, no zones, or unavailable.
    """
    import os as _os
    if not env_knobs.POUR_LAUNCH:
        return [], []
    try:
        from plane_fill_model import get_zone_model
        from plane_fragility import _erode_depth
        import numpy as _np
    except Exception:
        return [], []
    _zones = [z for z in (getattr(pcb_data, 'zones', None) or [])
              if z.net_id == net_id and z.layer in layer_names]
    if not _zones:
        return [], []
    _RUNGS = (1.0, 2.5, 6.0, 15.0)   # mm, same ladder as the multipoint side
    _NBEST = 12
    try:
        _FRAG_MM = env_knobs.POUR_LAUNCH_FRAG
    except ValueError:
        _FRAG_MM = 1.0
    _DEEP_MM = 0.5
    _models = []
    for _z in _zones:
        try:
            _models.append(get_zone_model(pcb_data, _z))
        except Exception:
            _models.append(None)
    if not any(_m is not None for _m in _models):
        return [], []
    _net_pads = pcb_data.pads_by_net.get(net_id, [])

    def _probe_size(fx, fy):
        for _p in _net_pads:
            if abs(_p.global_x - fx) < 0.01 and abs(_p.global_y - fy) < 0.01:
                return max(_p.size_x, _p.size_y)
        return config.track_width

    def _regions(rows):
        got = set()
        for _zi, _m in enumerate(_models):
            if _m is None:
                continue
            for _row in rows:
                _c = _m.query_component(_row[3], _row[4],
                                        size=_probe_size(_row[3], _row[4]))
                if _c and _c > 0:
                    got.add((_zi, _c))
        return got

    def _scan(near_rows, far_regions, taken):
        if not far_regions or not near_rows:
            return []
        _cx = sum(r[3] for r in near_rows) / len(near_rows)
        _cy = sum(r[4] for r in near_rows) / len(near_rows)
        _found = []
        for _r in _RUNGS:
            for _zi, _lb in far_regions:
                _m = _models[_zi]
                if _m is None:
                    continue
                _li = layer_names.index(_zones[_zi].layer)
                _dc = max(1, int(round(_DEEP_MM / _m.cell)))
                _i0 = max(0, int((_cx - _r - _m.x0) / _m.cell) - _dc)
                _i1 = min(_m.nx, int((_cx + _r - _m.x0) / _m.cell) + 1 + _dc)
                _j0 = max(0, int((_cy - _r - _m.y0) / _m.cell) - _dc)
                _j1 = min(_m.ny, int((_cy + _r - _m.y0) / _m.cell) + 1 + _dc)
                if _i0 >= _i1 or _j0 >= _j1:
                    continue
                _win = _m.labels[_i0:_i1, _j0:_j1]
                _mask = _win == _lb
                if not _mask.any():
                    continue
                _dep = _erode_depth(_mask, _dc)
                # labels is [x, y] (query_component's indexing)
                _xi, _yi = _np.nonzero(_mask)
                for _a, _b in zip(_xi, _yi):
                    _fx = _m.x0 + float(_a + _i0) * _m.cell
                    _fy = _m.y0 + float(_b + _j0) * _m.cell
                    _d2 = (_fx - _cx) ** 2 + (_fy - _cy) ** 2
                    if _d2 > _r * _r:
                        continue
                    _dmm = float(_dep[_a, _b]) * _m.cell
                    _frag = max(0.0, 1.0 - _dmm / _DEEP_MM)
                    _found.append((_d2 ** 0.5 + _FRAG_MM * _frag,
                                   _fx, _fy, _li))
            if _found:
                break
        _found.sort()
        _out = []
        for _sc, _fx, _fy, _li in _found:
            _g = coord.to_grid(_fx, _fy)
            _k = (_g[0], _g[1], _li)
            if _k in taken:
                continue
            if bounds is not None and not (
                    bounds[0] <= _g[0] <= bounds[2]
                    and bounds[1] <= _g[1] <= bounds[3]):
                continue
            taken.add(_k)
            _out.append((_g[0], _g[1], _li, _fx, _fy))
            if len(_out) >= _NBEST:
                break
        return _out

    try:
        _src_regions = _regions(sources)
        _tgt_regions = _regions(targets)
        _taken = set((r[0], r[1], r[2]) for r in sources)
        _taken.update((r[0], r[1], r[2]) for r in targets)
        _extra_t = _scan(sources, _tgt_regions, _taken)
        _extra_s = _scan(targets, _src_regions, _taken)
        return _extra_s, _extra_t
    except Exception as _e:
        print(f"  POUR-LAUNCH pair anchors unavailable ({_e})")
        return [], []


def _select_multipoint_main_edge(pcb_data, pad_info, pad_components,
                                 mst_edges, attraction_path):
    """Pick which MST edge Phase 1 routes NOW; the rest defer to Phase 3.

    Phase 3 runs after every other net's main route, so the deferred
    terminals route into whatever copper the neighbors left -- WHICH edge
    goes first decides which terminal still has open ground around it.
    Longest-first is the historical default. Two overrides, in priority
    order:

    1. Bus members with an attraction path: the main edge must SPAN the
       corridor. The spanning terminal pair is re-realized from the
       corridor's two endpoints (the MST realizes each component link by
       its CLOSEST pair, which for a BGA-to-chip bus net is typically
       pull-up-resistor-to-trunk -- leaving the dense-ball tap deferred
       until its own siblings seal the ball field). Only off-corridor
       taps stay in Phase 3. Disable with KICAD_BUS_MULTIPOINT_SPAN=0.
    2. KICAD_MULTIPOINT_DENSE_FIRST=1 (experimental): edges landing on a
       fanned-out high-density package (BGA/QFN/QFP) go first -- the same
       seal risk exists without a bus corridor.

    Returns the reordered edge list; element 0 is the main-edge candidate.
    """
    if not mst_edges:
        return mst_edges

    def comp(i):
        return pad_components.get(i, i)

    if attraction_path and env_knobs.BUS_MULTIPOINT_SPAN:
        p0 = attraction_path[0]
        p1 = attraction_path[-1]
        # Best cross-component terminal pair bridging the corridor's ends
        # (orientation-free score, grid Manhattan like the MST itself).
        best = None
        n = len(pad_info)
        for i in range(n):
            gxi, gyi = pad_info[i][0], pad_info[i][1]
            di0 = abs(gxi - p0[0]) + abs(gyi - p0[1])
            di1 = abs(gxi - p1[0]) + abs(gyi - p1[1])
            for j in range(i + 1, n):
                if comp(j) == comp(i):
                    continue
                gxj, gyj = pad_info[j][0], pad_info[j][1]
                dj0 = abs(gxj - p0[0]) + abs(gyj - p0[1])
                dj1 = abs(gxj - p1[0]) + abs(gyj - p1[1])
                score = min(di0 + dj1, di1 + dj0)
                if best is None or score < best[0]:
                    best = (score, i, j)
        if best is not None:
            _, si, sj = best
            span_comps = {comp(si), comp(sj)}
            for pos, (a, b, _d) in enumerate(mst_edges):
                if {comp(a), comp(b)} != span_comps:
                    continue
                dist = (abs(pad_info[si][3] - pad_info[sj][3])
                        + abs(pad_info[si][4] - pad_info[sj][4]))
                if {a, b} != {si, sj}:
                    pa = pad_info[si][5] if len(pad_info[si]) > 5 else None
                    pb = pad_info[sj][5] if len(pad_info[sj]) > 5 else None

                    def _nm(p, k):
                        # Terminal may be a Pad or an _EndpointStub free end
                        ref = getattr(p, 'component_ref', None)
                        return (f"{ref}.{getattr(p, 'pad_number', '?')}"
                                if ref else f"terminal {k}")
                    print(f"  Bus corridor: main edge re-realized to span the"
                          f" corridor ({_nm(pa, si)} <-> {_nm(pb, sj)},"
                          f" length={dist:.2f}mm); off-corridor taps deferred")
                    # The ORIGINAL closest-pair realization stays behind the
                    # span edge as the Phase-1 ladder's next candidate (a
                    # boxed span terminal must not forfeit the trunk route
                    # the closest pair could still win -- USB_D_P). The
                    # duplicate component link is inert in Phase 3: the tap
                    # loop only picks edges with exactly one routed side.
                    return ([(si, sj, dist), (a, b, _d)] + mst_edges[:pos]
                            + mst_edges[pos + 1:])
                if pos != 0:
                    print(f"  Bus corridor: corridor-spanning MST edge"
                          f" promoted to main (was #{pos + 1} by length)")
                return ([(si, sj, dist)] + mst_edges[:pos]
                        + mst_edges[pos + 1:])
            # The spanning components join through intermediates: no single
            # edge to re-realize; promote the best corridor-scoring edge.
            def span_score(e):
                a, b = e[0], e[1]
                da0 = abs(pad_info[a][0] - p0[0]) + abs(pad_info[a][1] - p0[1])
                da1 = abs(pad_info[a][0] - p1[0]) + abs(pad_info[a][1] - p1[1])
                db0 = abs(pad_info[b][0] - p0[0]) + abs(pad_info[b][1] - p0[1])
                db1 = abs(pad_info[b][0] - p1[0]) + abs(pad_info[b][1] - p1[1])
                return min(da0 + db1, da1 + db0)
            # Tie-break on the edge's pad indices, not on position in the list.
            pos = min(range(len(mst_edges)),
                      key=lambda k: (span_score(mst_edges[k]),
                                     mst_edges[k][0], mst_edges[k][1]))
            if pos != 0:
                print(f"  Bus corridor: promoted corridor-nearest MST edge"
                      f" to main (was #{pos + 1} by length)")
                return ([mst_edges[pos]] + mst_edges[:pos]
                        + mst_edges[pos + 1:])
            return mst_edges

    if env_knobs.MULTIPOINT_DENSE_FIRST:
        dense_refs = _dense_fanout_refs(pcb_data)
        if dense_refs:
            def on_dense(i):
                p = pad_info[i][5] if len(pad_info[i]) > 5 else None
                return (p is not None
                        and getattr(p, 'component_ref', None) in dense_refs)
            dense_edges, other = [], []
            for e in mst_edges:
                (dense_edges if on_dense(e[0]) or on_dense(e[1])
                 else other).append(e)
            if dense_edges and other and mst_edges[0] is not dense_edges[0]:
                print(f"  Dense-first: {len(dense_edges)} MST edge(s) on a"
                      f" fanned-out package promoted ahead of {len(other)}")
            if dense_edges:
                return dense_edges + other

    return mst_edges


def route_multipoint_main(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    obstacles: 'GridObstacleMap',
    pad_info: List[Tuple],
    attraction_path: Optional[List[Tuple[int, int, int]]] = None,
    state=None,
    _stub_switch_round: bool = False
) -> Optional[dict]:
    """
    Route only the main (longest MST segment) connection of a multi-point net.

    This is Phase 1 of multi-point routing. It computes an MST between all pads
    and routes the longest segment first, creating a clean 2-point route
    suitable for length matching.

    After length matching is applied, call route_multipoint_taps() to
    complete the remaining connections using the remaining MST edges.

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Grid routing configuration
        obstacles: Pre-built obstacle map
        pad_info: List of (gx, gy, layer_idx, orig_x, orig_y, pad) from get_multipoint_net_pads()
        attraction_path: Optional bus-corridor centerline (grid coords). The
            main edge is chosen to SPAN it (see _select_multipoint_main_edge)
            and the router attracts toward it, exactly as in
            route_net_with_obstacles -- multipoint bus members used to route
            their main edge blind.

    Returns:
        Routing result dict with:
        - 'new_segments', 'new_vias', 'iterations', 'path_length', 'path'
        - 'is_multipoint': True (flag for Phase 3)
        - 'multipoint_pad_info': Full pad_info list for Phase 3
        - 'routed_pad_indices': Set of indices already routed (the longest MST edge)
        - 'mst_edges': List of (idx_a, idx_b, length) for all MST edges
        Or {'failed': True, 'iterations': N} on failure
    """
    if GridRouter is None:
        print("  GridRouter not available")
        return None

    if len(pad_info) < 3:
        print(f"  Multi-point routing requires 3+ pads, got {len(pad_info)}")
        return None

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Extract pad positions for MST computation
    pad_positions = [(info[3], info[4]) for info in pad_info]  # (orig_x, orig_y)

    # Component-based multipoint (issue #317): group the terminals by the
    # net's EXISTING copper using the authoritative overlap-aware definition
    # (check_net_connectivity -- cap overlap, T-junctions, zones, pad
    # outlines), then span the COMPONENTS with an MST realized by the nearest
    # terminal pair between each pair of components. N components take
    # exactly N-1 routed connections; copper the checker already grades
    # connected is never re-tapped (the old pad-position MST + 0.02mm
    # coincidence filter routed redundant loops between overlap-joined
    # escapes -- butterstick DQ5).
    # Island-wide launch sets (KICAD_ISLAND_LAUNCH=0 disables): Phase 1 used
    # to launch each edge from TERMINAL cells only (pad centers / stub free
    # ends), so a route needing its own via 0.9mm behind the launch point
    # paid for fresh copper alongside its own stub to physically reach it
    # (WL_SDIO_D1 retraced its In1 escape with a parallel twin to get to the
    # ball via). Seeding each component's launch set with ALL of its copper
    # -- sampled segment points plus the island's vias on every layer, the
    # same machinery Phase 3 taps already use -- lets the route start at the
    # best point of the island (e.g. directly at the via on B.Cu), and the
    # dead-end sweep then retires whatever stub the route no longer uses.
    from connectivity import get_terminal_component_info
    pad_components, _isl_copper, _segs_by_comp, _vias_by_comp = \
        get_terminal_component_info(pcb_data, net_id, pad_info)
    _island_cells = None
    if env_knobs.ISLAND_LAUNCH:
        _island_cells = {}
        for _cid, _segs in _segs_by_comp.items():
            # #545 F9: the island's vias come from the connectivity graph's
            # own membership (vias_by_component), not a rounded-endpoint-key
            # equality match -- which missed a via 5um across a 0.01-rounding
            # bucket boundary (the COINCIDENCE_TOL soft-joint class the
            # checker grades connected) and every via T-tapped into a
            # segment's MIDDLE (stitching vias, prior tap junctions). Those
            # islands' cells existed only on one layer, so the route paid a
            # fresh via to reach copper the island already spans.
            _vs = _vias_by_comp.get(_cid, [])
            _pts = get_all_segment_tap_points(_segs, coord, layer_names,
                                              vias=_vs)
            # Keyed by cell, valued by the OWNER float point of that cell --
            # the conversion below welds the route to the owner of the cell
            # the router actually launched from (Phase 3's tap_point_map
            # contract). Discarding the owners and welding to the terminal
            # anchor drew a 1.5mm any-angle slash from the anchor to a route
            # that started elsewhere on the island (SDC0_CMD, busstop7).
            _island_cells[_cid] = {(p[0], p[1], p[2]): (p[3], p[4])
                                   for p in _pts}

        # POUR AS LAUNCH SURFACE (KICAD_POUR_LAUNCH=1), ladder v3: shared
        # per-region builder (_pour_launch_region_cells); Phase 3 merges the
        # same cells into ITS map -- v2 fed only this phase-1 map, so the
        # straggler edges Phase 3 routes never saw fill targets.
        _pl_cells = _pour_launch_region_cells(
            pcb_data, net_id, pad_info, pad_components, layer_names, coord,
            _island_cells)
        _pl_added = 0
        for _cid, _cmap in _pl_cells.items():
            _cells = _island_cells.setdefault(_cid, {})
            for _k, _v in _cmap.items():
                if _k not in _cells:
                    _cells[_k] = _v
                    _pl_added += 1
        if _pl_added:
            print(f"  POUR-LAUNCH v3: +{_pl_added} region-laddered target(s) "
                  f"across {len(_pl_cells)} component(s) (phase 1)")
    num_components = len(set(pad_components.values()))
    if num_components < len(pad_info):
        print(f"  Existing copper joins {len(pad_info)} terminals into "
              f"{num_components} group(s)")
    if net_id in (getattr(pcb_data, '_zone_blob_fallback_nets', None) or ()):
        # strict view: the grouping above fell back to zone-outline credit
        # for a zone with no fill model (scipy absent / oversize) -- the
        # fragment view for this net is DEGRADED, disclose it.
        print("  (fragment view degraded to zone-outline credit: no fill "
              "model for a zone on this net)")
    # #479 multi-board: never ATTEMPT an MST edge between two board outlines
    # -- no copper can join them (grading exempts them, and
    # filter_already_routed skips the net entirely once each outline is
    # internally connected). Run the component MST per outline so every
    # edge stays on one board; cross-board "links" are simply not edges.
    _outs = getattr(pcb_data.board_info, 'board_outlines', None) or []
    if len(_outs) >= 2:
        from check_connected import point_in_polygon as _pip

        def _oid(_pt):
            for _i, _poly in enumerate(_outs):
                if _pip(_pt[0], _pt[1], _poly):
                    return _i
            return None
        _by_out: Dict = {}
        for _i, _pt in enumerate(pad_positions):
            _by_out.setdefault(_oid(_pt), []).append(_i)
        mst_edges = []
        for _idxs in _by_out.values():
            if len(_idxs) < 2:
                continue
            _sub_pos = [pad_positions[_i] for _i in _idxs]
            _sub_comp = {_j: pad_components.get(_idxs[_j], _idxs[_j])
                         for _j in range(len(_idxs))}
            mst_edges.extend(
                (_idxs[_a], _idxs[_b], _d) for _a, _b, _d in
                compute_component_mst_edges(_sub_pos, _sub_comp))
    else:
        mst_edges = compute_component_mst_edges(pad_positions, pad_components)

    if not mst_edges:
        print(f"  All pads already connected by existing copper - nothing to route")
        return {
            'new_segments': [],
            'new_vias': [],
            'iterations': 0,
            'path_length': 0,
            'path': [],
            'is_multipoint': True,
            'multipoint_pad_info': pad_info,
            'routed_pad_indices': set(range(len(pad_info))),
            'pad_components': pad_components,
            'original_segments': [],
            'mst_edges': [],
            'waypoint_buckets': {},
            'already_connected': True,
            'tap_edges_routed': 0,
            'tap_edges_failed': 0,
            'tap_pads_connected': len(pad_info),
            'tap_pads_total': len(pad_info),
            # #578: the STRICT component count behind this verdict --
            # with the planner on the strict view, an "already connected"
            # return now really means one fragment per outline.
            'strict_fragments': num_components,
        }

    # Sort MST edges by length (longest first), then let the corridor /
    # dense-first overrides pick which edge Phase 1 actually routes now
    # (everything else waits for Phase 3, AFTER all other nets' mains).
    # Tie-break on the pad indices, not on input order. Sorting by length alone
    # is STABLE, so equal-length edges kept whatever order
    # compute_component_mst_edges produced -- which follows pad/segment order,
    # and the GUI and CLI hold copper in different orders. Measured on eth_tap:
    # two edges both 2.10mm, one front picked "pads 0 and 3", the other
    # "pads 3 and 1", and the whole multi-point route diverged from there.
    # Pad indices are a stable geometric identity here (pads_by_net order is
    # verified identical across fronts), so this is deterministic without
    # changing which edge wins on length.
    mst_edges = sorted(mst_edges, key=lambda e: (-e[2], e[0], e[1]))
    _edges_before = list(mst_edges)
    mst_edges = _select_multipoint_main_edge(pcb_data, pad_info,
                                             pad_components, mst_edges,
                                             attraction_path)
    # Net-story note: how the main edge was chosen (longest-first default,
    # corridor-span re-realization/promotion, or dense-first reorder).
    if mst_edges and _edges_before and mst_edges[0] != _edges_before[0]:
        _sel_note = {'mode': ('corridor-span' if attraction_path
                              else 'dense-first'),
                     'main_edge': list(mst_edges[0][:2]),
                     'displaced_longest': list(_edges_before[0][:2])}
    else:
        _sel_note = {'mode': 'longest-first',
                     'main_edge': list(mst_edges[0][:2]) if mst_edges else None}

    # Distribute guide-corridor waypoints across the MST edges: each waypoint
    # steers the edge it's nearest to, so the net follows the drawn line across
    # its whole topology, not just one edge (issue #7). Buckets are passed to the
    # main edge here and to the tap edges in route_multipoint_taps.
    pad_grid = [(info[0], info[1]) for info in pad_info]
    waypoint_buckets = assign_waypoints_to_mst_edges(
        getattr(config, 'corridor_waypoints', None) or [], pad_grid, mst_edges)

    # Get stub free ends for proximity zone checking
    free_end_sources, free_end_targets, _ = get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=True)
    # Fallback to the net's pad grid positions when there are no stub free ends
    # (a multipoint net with no prior copper, e.g. a fresh all-pad power net).
    # The old code referenced undefined locals `sources`/`targets` here, which
    # crashed route_multipoint_main with UnboundLocalError on exactly those
    # nets (stress test: confirmed on 5+ boards). pad_info rows are
    # (gx, gy, layer_idx, orig_x, orig_y, endpoint_obj).
    pad_prox = [(info[0], info[1], info[2]) for info in pad_info]
    if free_end_sources:
        prox_check_sources = [(s[0], s[1], s[2]) for s in free_end_sources]
    else:
        prox_check_sources = pad_prox
    if free_end_targets:
        prox_check_targets = [(t[0], t[1], t[2]) for t in free_end_targets]
    else:
        prox_check_targets = pad_prox

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

    # Check which proximity zones the stub free ends are in for precise heuristic estimate
    src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_sources)
    src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_sources)
    tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in prox_check_targets)
    tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in prox_check_targets)
    prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
    if config.verbose:
        zones = []
        if src_in_stub: zones.append("src:stub")
        if src_in_bga: zones.append("src:bga")
        if tgt_in_stub: zones.append("tgt:stub")
        if tgt_in_bga: zones.append("tgt:bga")
        print(f"  proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

    # Bus attraction parameters -- same math as route_net_with_obstacles
    # (multipoint members' mains used to skip this entirely, so a bus net
    # whose topology was multipoint routed its trunk OFF the corridor).
    bus_attraction_radius_grid = coord.to_grid_dist(config.bus_attraction_radius) if config.bus_attraction_radius > 0 else 0
    bus_attraction_bonus = config.scaled_cell_units(config.bus_attraction_bonus) if config.bus_attraction_bonus > 0 else 0
    bus_xlayer_pct = 0
    # #589 owner attraction: plan corridors change layers (~1 via/net in a
    # negotiated plan), so cross-layer pull must arm for plan-attracted
    # nets too, not only bus members.
    if bus_attraction_bonus > 0 and (getattr(config, 'bus_enabled', False)
                                     or env_knobs.GLOBAL_PLAN.get('attract')):
        try:
            bus_xlayer_pct = env_knobs.BUS_XLAYER_PCT
        except ValueError:
            bus_xlayer_pct = 35

    # Route farthest pair with probe routing (same as single-ended)
    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=config.via_proximity_cost_int(),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=prox_h_cost,
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost,
                        attraction_radius=bus_attraction_radius_grid,
                        attraction_bonus=bus_attraction_bonus,
                        attraction_cross_layer_pct=bus_xlayer_pct,
                        attraction_potential=env_knobs.GLOBAL_PLAN.get('attract_potential', 0))

    if attraction_path:
        router.set_attraction_path(attraction_path)
        if config.verbose:
            layers_in_path = set(p[2] for p in attraction_path)
            print(f"    Bus attraction: {len(attraction_path)} path points, layers={layers_in_path}, radius={bus_attraction_radius_grid} grid, bonus={bus_attraction_bonus}")

    # Per-layer fractional track margins (#156): exact extra half-width over
    # the stamps' reserve, no ceil, no +1 (see track_margins_for_net).
    track_margin = config.track_margins_for_net(net_id)

    # Route the main edge: try MST edges longest-first until one routes.
    # Issue #101: one boxed pad must not abandon the whole net - the old code
    # gave up entirely when the single longest edge failed, zeroing 55-pad
    # power nets whose other 54 pads had clear connections. A failed edge now
    # falls through to the next candidate; Phase 3 later handles every
    # remaining edge individually with honest failed-pad reporting.
    max_main_attempts = min(len(mst_edges), 8)
    path = None
    total_iterations = 0
    cumulative_iterations = 0
    last_failure = None
    routed_edge_pos = None
    for attempt in range(max_main_attempts):
        idx_a, idx_b, edge_len = mst_edges[attempt]
        label = ("routing longest MST edge" if attempt == 0
                 else f"retrying with next MST edge ({attempt + 1}/{max_main_attempts})")
        print(f"  Multi-point net Phase 1: {label} (pads {idx_a} and {idx_b}, length={edge_len:.2f}mm)")

        # Build source/target for this pad pair
        pad_a = pad_info[idx_a]
        pad_b = pad_info[idx_b]

        # For through-hole pads, create sources/targets on ALL layers (router
        # can reach any layer) - avoids unnecessary vias
        pad_a_obj = pad_a[5] if len(pad_a) > 5 else None
        pad_b_obj = pad_b[5] if len(pad_b) > 5 else None

        if _pad_all_layer_reach(pcb_data, pad_a_obj):
            sources = [(pad_a[0], pad_a[1], layer_idx) for layer_idx in range(len(layer_names))]
        else:
            sources = [(pad_a[0], pad_a[1], pad_a[2])]  # (gx, gy, layer_idx)

        if _pad_all_layer_reach(pcb_data, pad_b_obj):
            targets = [(pad_b[0], pad_b[1], layer_idx) for layer_idx in range(len(layer_names))]
        else:
            targets = [(pad_b[0], pad_b[1], pad_b[2])]

        # Island-wide launch: every cell of each endpoint's copper island is
        # a legal start/land (its stubs, trunks, and vias on all layers) --
        # the route begins at the island's best point instead of paying
        # duplicate copper to walk back to its own via.
        if _island_cells is not None:
            _isl = _island_cells.get(pad_components.get(idx_a, idx_a))
            if _isl:
                sources = list({*sources, *_isl})
            _isl = _island_cells.get(pad_components.get(idx_b, idx_b))
            if _isl:
                targets = list({*targets, *_isl})

        # #479: a pad centre hard-blocked on every candidate layer (foreign
        # stabilizer-NPTH keep-out over the pad) cannot take the first step;
        # seed the pad's free on-copper cells as extra terminals.
        sources = _augment_all_blocked_pad_side(sources, pad_a_obj, config,
                                                obstacles)
        targets = _augment_all_blocked_pad_side(targets, pad_b_obj, config,
                                                obstacles)

        # Mark source/target cells (same-net pad cells; safe to accumulate)
        for gx, gy, layer in sources + targets:
            obstacles.add_source_target_cell(gx, gy, layer)

        # Use probe routing helper, steered through this edge's bucket of
        # corridor waypoints (tap edges follow their own buckets later).
        (path, total_iterations, forward_blocked, backward_blocked, reversed_path,
         fwd_iters, bwd_iters, necked_down, uniform_width, main_unblock_vias,
         main_unblock_segments) = _route_with_via_unblock(
            router, obstacles, config, sources, targets, track_margin,
            pcb_data, net_id, print_prefix="  ", direction_labels=("forward", "backward"),
            waypoints=waypoint_buckets.get(frozenset((idx_a, idx_b)), [])
        )
        cumulative_iterations += total_iterations

        if path is not None:
            routed_edge_pos = attempt
            break

        print(f"  Phase 1 edge (pads {idx_a},{idx_b}) failed after {total_iterations} iterations"
              + (" - trying next MST edge" if attempt + 1 < max_main_attempts else ""))
        last_failure = {
            'failed': True,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': fwd_iters,
            'iterations_backward': bwd_iters,
        }

    if path is None and state is not None and not _stub_switch_round \
            and env_knobs.MULTIPOINT_STUB_SWITCH:
        # Stub layer switch retry for the main edge (for a bus member, the
        # corridor-SPANNING edge -- the leg that must route first) when a
        # terminal's escape stub is walled in on its own layer. Move the
        # stubs at the first edges' endpoints to an open layer (the pad via
        # sizes itself down the fab ladder) and re-run Phase 1 once on
        # freshly-derived terminals. Kept only when the retry routes real
        # main copper; reverted exactly otherwise. Default ON
        # (KICAD_MULTIPOINT_STUB_SWITCH=0 disables); the original gate-off
        # defect was the create_routing_state `or []` alias break dropping
        # the kept switch from the written file, not this retry itself.
        from stub_layer_switching import (switch_boxed_stub_near,
                                          revert_stub_layer_switch)
        from connectivity import get_multipoint_net_pads
        switched = []
        seen_xy = set()
        moved_tips = set()
        for (ia, ib, _elen) in mst_edges[:2]:
            for idx in (ia, ib):
                x, y = pad_info[idx][3], pad_info[idx][4]
                key = (round(x, 2), round(y, 2))
                if key in seen_xy:
                    continue
                seen_xy.add(key)
                sw = switch_boxed_stub_near(pcb_data, net_id, config, x, y,
                                            moved_tips=moved_tips)
                if sw is not None:
                    switched.append(sw)
        if switched:
            new_pad_info = get_multipoint_net_pads(pcb_data, net_id, config)
            retry = None
            if new_pad_info:
                retry = route_multipoint_main(
                    pcb_data, net_id, config, obstacles, new_pad_info,
                    attraction_path=attraction_path, state=state,
                    _stub_switch_round=True)
            if retry and not retry.get('failed') and retry.get('path'):
                print(f"  MULTIPOINT STUB SWITCH: main edge routed after "
                      f"moving {len(switched)} stub(s)")
                for _mods, _vias, _f, _d in switched:
                    state.all_segment_modifications.extend(_mods)
                    state.all_swap_vias.extend(_vias)
                retry['iterations'] = retry.get('iterations', 0) + cumulative_iterations
                return retry
            for _mods, _vias, _f, _d in reversed(switched):
                revert_stub_layer_switch(pcb_data, _mods, _vias)

    if path is None:
        # #348 (ottercast RESETn): every main-edge attempt launches from PAD
        # cells, so a terminal boxed in by static obstacles fails Phase 1
        # outright even when its ISLAND has open copper (a free inner-layer
        # stub end) a human routes from in seconds. When existing copper
        # already joins terminals into fewer groups, don't fail the net:
        # hand Phase 3 a synthetic empty main result rooted at the LARGEST
        # copper-joined component -- its tap loop seeds sources from the
        # island's copper (get_all_segment_tap_points over the existing base
        # copper, see route_multipoint_taps) and has the orphan-pad fallback.
        _has_copper = any(s.net_id == net_id for s in pcb_data.segments) or \
            any(v.net_id == net_id for v in pcb_data.vias)
        if _has_copper:
            # Base = the component whose island carries the most existing
            # copper (RESETn: three 1-terminal components, each its own
            # island -- terminal count can't break the tie, copper can).
            # Stub terminals resolve inside the helper.
            from connectivity import get_terminal_component_info
            _comps, _copper, _, _ = get_terminal_component_info(
                pcb_data, net_id, pad_info)
            # Tie-break geometrically: equal-copper components fell back to
            # the first pad index, i.e. to pad walk order.
            _best = max(range(len(pad_info)),
                        key=lambda i: (_copper.get(_comps.get(i), 0),
                                       -pad_info[i][0], -pad_info[i][1]))
            _best_copper = _copper.get(_comps.get(_best), 0)
            _base_comp = _comps.get(_best)
            _base_idx = [i for i in range(len(pad_info))
                         if _comps.get(i) == _base_comp]
            if _base_idx and mst_edges and _best_copper > 0:
                print(f"  Phase 1 exhausted from the pads; deferring "
                      f"{len(mst_edges)} edge(s) to Phase 3's island-copper "
                      f"sources (base component: {len(_base_idx)} terminal(s))")
                _dummy = (_base_idx[0], _base_idx[0], 0.0)
                return {
                    'new_segments': [],
                    'new_vias': [],
                    'iterations': cumulative_iterations,
                    'path_length': 0,
                    'path': [],
                    'is_multipoint': True,
                    'multipoint_pad_info': pad_info,
                    'routed_pad_indices': set(_base_idx),
                    'pad_components': pad_components,
                    'original_segments': [],
                    'mst_edges': [_dummy] + mst_edges,
                    'edge_selection_note': _sel_note,
                    'waypoint_buckets': waypoint_buckets,
                    'phase1_exhausted': True,
                    'tap_edges_routed': 0,
                    'tap_edges_failed': 0,
                }
        print(f"  Failed to route a main edge after {cumulative_iterations} iterations "
              f"({max_main_attempts} edge(s) tried)")
        # Name pre-existing blockers (#103) for MULTIPOINT failures too: the
        # single-ended loop records this event for the reconciliation's rip
        # escalation, but multipoint edge failures never did -- so a custody
        # plane net (GND-class, always multipoint) could not feed the
        # plane-weld escalation round (0804-wave finding).
        if state is not None and last_failure:
            try:
                from routing_diagnostics import preexisting_blocker_hint
                from routing_state import record_net_event as _rne103
                _cells103 = (
                    (last_failure.get('blocked_cells_forward') or [])
                    + (last_failure.get('blocked_cells_backward') or []))
                _h103, _b103 = preexisting_blocker_hint(
                    _cells103, config, pcb_data, net_id,
                    routed_net_ids=state.routed_net_ids, return_names=True)
                if _h103:
                    from routing_diagnostics import condense_hint as _ch
                    _c103 = _ch(_h103)
                    if _c103:
                        print(f"  {_c103}")
                    _rne103(state, net_id, "preexisting_blockers",
                            {"hint": _h103, "blockers": _b103})
            except Exception:
                pass
        failure = dict(last_failure or {'failed': True})
        failure['iterations'] = cumulative_iterations
        return failure

    # Phase 3 assumes mst_edges[0] is the edge Phase 1 routed - move the
    # successful edge to the front when a fallback edge won.
    if routed_edge_pos:
        mst_edges = ([mst_edges[routed_edge_pos]] + mst_edges[:routed_edge_pos]
                     + mst_edges[routed_edge_pos + 1:])
    total_iterations = cumulative_iterations

    # If path was found in reverse direction, swap pad_a/pad_b for segment generation
    if reversed_path:
        pad_a, pad_b = pad_b, pad_a
        idx_a, idx_b = idx_b, idx_a

    # Get through-hole pad positions for this net (layer transitions without via)
    through_hole_positions = get_same_net_through_hole_positions(pcb_data, net_id, config)

    # Convert path to segments/vias. The originals default to the terminal
    # anchors, but with island-wide launch the route may begin/end at ANY
    # cell of the terminal's island -- weld to the owner point of the cell
    # actually used (Phase 3's tap_point_map contract), never across the
    # island back to the anchor.
    _start_orig = (pad_a[3], pad_a[4], layer_names[pad_a[2]])
    _end_orig = (pad_b[3], pad_b[4], layer_names[pad_b[2]])
    if _island_cells is not None and path:
        _own = _island_cells.get(pad_components.get(idx_a, idx_a), {}).get(tuple(path[0]))
        if _own is not None:
            _start_orig = (_own[0], _own[1], layer_names[path[0][2]])
        _own = _island_cells.get(pad_components.get(idx_b, idx_b), {}).get(tuple(path[-1]))
        if _own is not None:
            _end_orig = (_own[0], _own[1], layer_names[path[-1][2]])
    # Barrel-in-fill completion (#562), Phase-1 parity with the tap edges.
    path, _fill_end = _trim_after_fill_via(path, coord, layer_names,
                                           pcb_data, net_id)
    if _fill_end is not None:
        _end_orig = _fill_end
    segments, vias = _path_to_segments_vias(
        path, coord, layer_names, net_id, config,
        _start_orig,
        _end_orig,
        through_hole_positions,
        pcb_data
    )
    if necked_down:
        # Both endpoints are pads: neck the start side too
        segments = _apply_neckdown_widths(segments, config, net_id, obstacles,
                                          coord, layer_names, track_margin, neck_start=True)
    elif uniform_width is not None:
        # Short power edge routed at a stepped-down width (#180): every segment is
        # that width, so obstacle blocking (reads seg.width) and output match.
        for _s in segments:
            _s.width = uniform_width
    # Re-neck terminal grazes AFTER width assignment (#212): the neckdown/uniform
    # passes above rebuild widths and would otherwise restore a grazing terminal leg
    # to base/power width, undoing the graze-neck applied during conversion.
    _hard_p1 = _neck_route_terminal_grazes(segments, path, coord,
                                           _start_orig[:2], _end_orig[:2],
                                           pcb_data, net_id, config)
    if _hard_p1:
        # Terminal-bridge SHORT gate (ux pf9): the main edge's terminal copper
        # overlaps a foreign track/via at any width -- fail the edge so the
        # rip/retry ladder finds another approach instead of shipping a short.
        _hs, _hd = _hard_p1[0]
        print(f"  {YELLOW}Phase 1 terminal copper on {_hs.layer} would OVERLAP "
              f"a foreign track/via (edge dist {_hd:.3f}mm) -- failing the "
              f"edge rather than shipping a short{RESET}")
        return {'failed': True, 'iterations': total_iterations}
    # Fab-floor via dropped inside a boxed main-edge pad to unblock it (#189);
    # a #535 off-pad escape ships its pad->via stub alongside.
    vias = list(vias) + main_unblock_vias
    segments = list(segments) + main_unblock_segments

    print(f"  Phase 1 routed in {total_iterations} iterations, {len(segments)} segments")

    return {
        'new_segments': segments,
        'new_vias': vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
        'is_multipoint': True,
        'multipoint_pad_info': pad_info,
        'routed_pad_indices': {idx_a, idx_b},
        'pad_components': pad_components,  # Zone-connected component for each pad
        # Store main pad positions for Phase 3 tap filtering
        'main_pad_a': (pad_a[3], pad_a[4]),  # (orig_x, orig_y) of first main pad
        'main_pad_b': (pad_b[3], pad_b[4]),  # (orig_x, orig_y) of second main pad
        # Store original segments for identifying meanders in Phase 3
        'original_segments': segments,
        # Store MST edges for Phase 3 (sorted longest first)
        'mst_edges': mst_edges,
        'edge_selection_note': _sel_note,
        # Per-edge guide-corridor waypoint buckets (issue #7), for Phase 3 taps
        'waypoint_buckets': waypoint_buckets,
        # Initial tap stats (Phase 1 connects 2 pads via 1 edge)
        'tap_edges_routed': 1,
        'tap_edges_failed': 0,
        'tap_pads_connected': 2,
        'tap_pads_total': len(pad_info),
    }


def get_all_segment_tap_points(
    segments: List[Segment],
    coord: GridCoord,
    layer_names: List[str],
    vias: List = None
) -> List[Tuple[int, int, int, float, float]]:
    """
    Get all grid points along existing segments and vias as potential tap sources.

    Returns list of (gx, gy, layer_idx, orig_x, orig_y) for each point.
    Points are sampled at grid resolution along each segment.
    Vias are added on ALL layers (they connect all copper layers).
    Sorted by grid coordinates for deterministic iteration.
    """
    # Use dict keyed by (gx, gy, layer_idx) to deduplicate while keeping original coords
    tap_points = {}  # (gx, gy, layer_idx) -> (orig_x, orig_y)
    layer_map = build_layer_map(layer_names)

    for seg in segments:
        layer_idx = layer_map.get(seg.layer, 0)

        # Sample points along the segment at grid resolution
        dx = seg.end_x - seg.start_x
        dy = seg.end_y - seg.start_y
        length = (dx*dx + dy*dy) ** 0.5

        if length < 0.001:
            # Point segment
            gx, gy = coord.to_grid(seg.start_x, seg.start_y)
            key = (gx, gy, layer_idx)
            if key not in tap_points:
                tap_points[key] = (seg.start_x, seg.start_y)
        else:
            # Sample along segment at grid step intervals
            num_steps = max(1, int(length / coord.grid_step))
            for i in range(num_steps + 1):
                t = i / num_steps
                x = seg.start_x + t * dx
                y = seg.start_y + t * dy
                gx, gy = coord.to_grid(x, y)
                key = (gx, gy, layer_idx)
                if key not in tap_points:
                    tap_points[key] = (x, y)

    # Add vias on ALL layers (vias connect all copper layers)
    if vias:
        for via in vias:
            gx, gy = coord.to_grid(via.x, via.y)
            for layer_idx in range(len(layer_names)):
                key = (gx, gy, layer_idx)
                if key not in tap_points:
                    tap_points[key] = (via.x, via.y)

    # Return sorted list for deterministic iteration
    return sorted([(gx, gy, layer_idx, ox, oy)
                   for (gx, gy, layer_idx), (ox, oy) in tap_points.items()])


def route_multipoint_taps(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    obstacles: 'GridObstacleMap',
    main_result: dict,
    global_offset: int = 0,
    global_total: int = 0,
    global_failed: int = 0
) -> Optional[dict]:
    """route_multipoint_taps with guaranteed cleanup of the in-progress-via
    rings (issue #309). _register_inprogress_via stamps raw ref-counted
    blocked-via rings into `obstacles` while the taps route; when the caller
    passed the PERSISTENT working map (reroute_loop's in-place mode) rather
    than a per-net clone, those rings leaked forever - restore_obstacles_inplace
    only removes its own same-net-via cells. The rings are per-route
    scaffolding (the committed route's vias get their real keep-outs from the
    net's recomputed obstacle cache), so remove exactly the cells added, on
    every exit path. On a clone the removal is harmless."""
    ring_cells: list = []
    try:
        return _route_multipoint_taps_impl(
            pcb_data, net_id, config, obstacles, main_result,
            global_offset, global_total, global_failed, ring_cells)
    finally:
        if ring_cells:
            _rc = np.array(ring_cells, dtype=np.int32)
            obstacles.remove_blocked_vias_batch(_rc)
            try:    # #568: mirror of the ring's small stamp (refcount balance)
                from obstacle_map import _rung_small_armed as _rsa
                if _rsa() and hasattr(obstacles, 'remove_blocked_vias_small_batch'):
                    obstacles.remove_blocked_vias_small_batch(_rc)
            except (AttributeError, ImportError):
                pass


def _route_multipoint_taps_impl(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    obstacles: 'GridObstacleMap',
    main_result: dict,
    global_offset: int = 0,
    global_total: int = 0,
    global_failed: int = 0,
    _ring_cells: list = None
) -> Optional[dict]:
    """
    Route the remaining MST edges for a multi-point net.

    This is Phase 3 of multi-point routing - called AFTER length matching
    has been applied to the main route. It routes the remaining MST edges
    in order of length (longest first), connecting unrouted pads to the
    growing routed network.

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Grid routing configuration
        obstacles: Pre-built obstacle map (should include length-matched segments)
        main_result: Result from route_multipoint_main() with meanders applied

    Returns:
        Updated result dict with tap segments/vias added, or None on failure
    """
    if GridRouter is None:
        print("  GridRouter not available")
        return None

    # #658 power discipline: Phase-3 tap/MST-edge routing is the bulk of a
    # power net's copper (measured: 65 of 82 segments) and previously
    # bypassed the SE loop's per-net config chain -- the leak that kept
    # power trunks on forbidden layers. Same soft override as the loop.
    from global_plan import power_layer_config
    config = power_layer_config(config, config, net_id)

    pad_info = main_result['multipoint_pad_info']
    routed_indices = set(main_result['routed_pad_indices'])
    mst_edges = main_result.get('mst_edges', [])
    pad_components = main_result.get('pad_components', {i: i for i in range(len(pad_info))})
    waypoint_buckets = main_result.get('waypoint_buckets', {})  # per-edge corridor waypoints

    # Build set of "routed components" - components with at least one explicitly routed pad
    # Pads in zone-connected components are effectively routed if any pad in that component is routed
    routed_components = {pad_components.get(idx, idx) for idx in routed_indices}

    # Get the current segments (which may have meanders from length matching)
    all_segments = list(main_result['new_segments'])
    all_vias = list(main_result.get('new_vias', []))

    coord = GridCoord(config.grid_step)
    layer_names = config.layers

    # Cells needing NO new via on a layer change: same-net through-hole pads and
    # same-net vias (each already connects all layers). Seeding with the net's
    # current vias (the main edge + pre-existing) and updating it as each tap edge
    # places vias lets a later edge REUSE a via the main edge already dropped at
    # the same cell, instead of stacking a second coincident one (EPHY_TX_N).
    through_hole_positions = set(get_same_net_through_hole_positions(pcb_data, net_id, config))
    for _v in pcb_data.vias:
        if _v.net_id == net_id:
            through_hole_positions.add(coord.to_grid(_v.x, _v.y))

    # In-progress vias (this net's main + earlier tap edges) are NOT yet in
    # pcb_data, so the per-net obstacle clone doesn't know about them. Register
    # each one in the live map so a LATER edge (1) REUSES it as a zero-cost free
    # via when its path lands on the cell, and (2) cannot drop a SECOND via within
    # hole-to-hole of it. Without this, a later branch dropped a via a sub-mm away
    # -- the VTT multipoint junction double-via (hole_to_hole DRC). The ring skips
    # the via's own cell so reuse stays open.
    _vv_radius = (config.via_size + config.clearance) * coord.inv_step

    try:        # #568: armed once per tap run (see the ring mirror below)
        from obstacle_map import _rung_small_armed as _rsa
        _small_rung_on = _rsa() and hasattr(obstacles, 'add_blocked_via_small')
    except ImportError:
        _small_rung_on = False

    def _register_inprogress_via(v):
        vgx, vgy = coord.to_grid(v.x, v.y)
        obstacles.add_free_via(vgx, vgy)
        # Grow the ring by the via's sub-grid offset so a later same-net via keeps
        # the full spacing from this via's TRUE centre, not its rounded cell --
        # otherwise a fine-grid route drops a via a sub-cell too close (issue #70,
        # mirroring add_same_net_via_clearance).
        off_cells = math.hypot(v.x - vgx * coord.grid_step,
                               v.y - vgy * coord.grid_step) / coord.grid_step
        radius = _vv_radius + off_cells
        rng = int(math.ceil(radius))
        radius_sq = radius * radius
        for ex in range(-rng, rng + 1):
            for ey in range(-rng, rng + 1):
                d = ex * ex + ey * ey
                if 0 < d <= radius_sq:
                    obstacles.add_blocked_via(vgx + ex, vgy + ey)
                    # #568 MIRROR: a rung-1 tap search trusts ONLY the small
                    # map for dynamic copper, so without this it could drop a
                    # small via inside the ring of a via this very net just
                    # placed -- a real same-net hole-to-hole violation. The
                    # wrapper's finally removes both maps' cells (#309).
                    if _small_rung_on:
                        obstacles.add_blocked_via_small(vgx + ex, vgy + ey)
                    # Ref-counted raw add: the wrapper removes these on exit so
                    # they can't leak into a persistent working map (#309).
                    if _ring_cells is not None:
                        _ring_cells.append((vgx + ex, vgy + ey))

    for _v in all_vias:
        _register_inprogress_via(_v)

    # #545 F1/F2: Phase 3 gets the SAME island machinery Phase 1 has. Tap
    # sources used to be only the copper Phase 1 just created plus the single
    # src_pad cell -- every pre-existing fanout stub, prior-pass partial
    # route and existing via of the routed islands was invisible, so taps
    # re-walked copper that already exists (the WL_SDIO_D1 class; the
    # island-launch fix at Phase 1 was never applied here). Tap TARGETS were
    # one cell on one layer -- a tap landing on an existing island could only
    # hit its representative point. Per-component cell->owner maps, computed
    # once (the same get_all_segment_tap_points machinery Phase 1 uses; vias
    # from the graph's authoritative membership, #545 F9). Sources take only
    # ROUTED components' islands (#189: launching from an unconnected island
    # would join the target to that island and mark it routed while the base
    # stays split); targets take exactly the TARGET pad's own island. This
    # subsumes the old phase-1-exhausted-only seeding (#348) -- the islands
    # are seeded on that branch unconditionally, as before.
    from connectivity import get_terminal_component_info
    _p3_comps, _p3_copper, _p3_segs_by_comp, _p3_vias_by_comp = \
        get_terminal_component_info(pcb_data, net_id, pad_info)
    _p3_island_cells: Dict[int, Dict] = {}
    for _cid, _segs in _p3_segs_by_comp.items():
        _pts = get_all_segment_tap_points(_segs, coord, layer_names,
                                          vias=_p3_vias_by_comp.get(_cid, []))
        # Keyed by cell, valued by the OWNER float point (Phase 3's
        # tap_point_map / end-weld contract: weld to the owner of the cell
        # the router actually used, never to a distant anchor).
        _p3_island_cells[_cid] = {(p[0], p[1], p[2]): (p[3], p[4])
                                  for p in _pts}
    # POUR-LAUNCH ladder v3 (#544/#562): fill-region launch cells, keyed by
    # the SAME component-id space the lookups below use -- which in Phase 3
    # is the FRESH labeling (_p3_comps), NOT Phase 1's pad_components.
    # Component ids are union-find roots over graph point ids, so committing
    # Phase-1 copper rewrites the whole id space; keying these cells with the
    # stale map made them either invisible here (every lookup below goes
    # through _p3_comps) or, on a numeric collision, attached to the WRONG
    # island -- the df55059 class, one function over. Sources union across
    # routed_components as the tree grows (#545 F1), so a joined region's
    # fill becomes launchable for every later edge.
    _pl_cells = _pour_launch_region_cells(
        pcb_data, net_id, pad_info, _p3_comps, layer_names, coord,
        _p3_island_cells)
    _pl_added = 0
    for _cid, _cmap in _pl_cells.items():
        _cells = _p3_island_cells.setdefault(_cid, {})
        for _k, _v in _cmap.items():
            if _k not in _cells:
                _cells[_k] = _v
                _pl_added += 1
    if _pl_added:
        print(f"  POUR-LAUNCH v3: +{_pl_added} region-laddered target(s) "
              f"across {len(_pl_cells)} component(s) (phase 3)")
    _p3_use_islands = bool(_p3_island_cells) and (
        env_knobs.ISLAND_LAUNCH or main_result.get('phase1_exhausted'))
    if _p3_use_islands and main_result.get('phase1_exhausted'):
        _n_cells = sum(len(_m) for _c, _m in _p3_island_cells.items())
        print(f"  Seeding tap sources from the base island's existing "
              f"copper ({_n_cells} cell(s) across "
              f"{len(_p3_island_cells)} island(s))")

    # Get remaining MST edges (skip the first one which was routed in Phase 1)
    # MST edges are already sorted longest-first
    remaining_edges = mst_edges[1:] if len(mst_edges) > 1 else []

    if not remaining_edges:
        print(f"  No remaining MST edges to route in Phase 3")
        return main_result

    print(f"  Multi-point net Phase 3: routing {len(remaining_edges)} remaining MST edges (longest first)")

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=config.via_proximity_cost_int(),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=0,  # Set per-route below
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost)

    # Per-layer fractional track margins (#156): exact extra half-width over
    # the stamps' reserve, no ceil, no +1 (see track_margins_for_net).
    track_margin = config.track_margins_for_net(net_id)

    total_iterations = 0

    # Route remaining MST edges in order (longest first)
    # Each edge connects a routed pad to an unrouted pad
    edges_routed = 0
    failed_edges = set()  # Track edges that failed to route
    failed_edge_blocking = {}  # edge_key -> (blocked_cells, tgt_xy, {'fwd','bwd','extra'} dir split)
    fallback_attempted = set()  # Pads attempted directly after their MST edge chain failed
    max_passes = len(remaining_edges) * 2 + len(pad_info)  # Safety limit

    for pass_num in range(max_passes):
        if len(routed_indices) == len(pad_info):
            break  # All pads connected

        # Find an edge that connects routed to unrouted (skip failed edges)
        edge_to_route = None
        for edge in remaining_edges:
            idx_a, idx_b, length = edge
            edge_key = (min(idx_a, idx_b), max(idx_a, idx_b))
            if edge_key in failed_edges:
                continue

            # Check if pad is effectively routed (either explicitly or via zone-connected component)
            a_component = pad_components.get(idx_a, idx_a)
            b_component = pad_components.get(idx_b, idx_b)
            a_routed = idx_a in routed_indices or a_component in routed_components
            b_routed = idx_b in routed_indices or b_component in routed_components

            if a_routed and not b_routed:
                edge_to_route = (idx_a, idx_b, length)  # Route from a to b
                break
            elif b_routed and not a_routed:
                edge_to_route = (idx_b, idx_a, length)  # Route from b to a
                break

        if edge_to_route is None:
            # Fallback: a failed MST edge orphans its entire downstream
            # subtree - those pads' edges are never eligible because their
            # source side never becomes routed. Since tap routing launches
            # from ALL existing copper anyway (the MST edge is only an
            # ordering), attempt each orphaned pad directly once. Even when
            # the attempt fails, its blocked frontier feeds the Phase 3
            # rip-up analysis with the pads' ACTUAL blockers (issues
            # #101/#103: previously a walled-off region produced no frontier
            # data at all, so nothing was ever ripped).
            best = None
            for i in range(len(pad_info)):
                if i in routed_indices or pad_components.get(i, i) in routed_components:
                    continue
                if i in fallback_attempted:
                    continue
                xi, yi = pad_info[i][3], pad_info[i][4]
                for j in routed_indices:
                    d = abs(xi - pad_info[j][3]) + abs(yi - pad_info[j][4])
                    if best is None or d < best[2]:
                        best = (j, i, d)
            if best is not None:
                fallback_attempted.add(best[1])
                edge_to_route = best
                print(f"    Fallback: attempting orphaned pad {best[1]} directly from connected copper")

        if edge_to_route is None:
            # Count effectively unrouted pads (not in routed_indices AND not in a routed component)
            unrouted_pads = sum(1 for i in range(len(pad_info))
                               if i not in routed_indices and pad_components.get(i, i) not in routed_components)
            if unrouted_pads > 0:
                print(f"  {YELLOW}Warning: {unrouted_pads} pad(s) not connected ({len(failed_edges)} MST edge(s) failed){RESET}")
            break

        src_idx, tgt_idx, edge_len = edge_to_route

        src_pad = pad_info[src_idx]
        tgt_pad = pad_info[tgt_idx]

        # Show progress: [current/total] with failure count (global across all nets)
        current_global = global_offset + edges_routed + len(failed_edges) + 1
        total_failed = global_failed + len(failed_edges)
        fail_str = f" ({total_failed} failed)" if total_failed > 0 else ""
        print(f"    [{current_global}/{global_total}]{fail_str} Routing MST edge: pad {src_idx} -> pad {tgt_idx} (length={edge_len:.2f}mm) target=({tgt_pad[3]:.2f}, {tgt_pad[4]:.2f})")

        # Get target pad coordinates
        tgt_x, tgt_y = tgt_pad[3], tgt_pad[4]

        # Get ALL points along existing segments and vias as potential tap sources
        # The router will find the shortest path from ANY of these points
        # Vias are included on ALL layers since they connect all copper layers
        all_tap_points = get_all_segment_tap_points(
            all_segments, coord, layer_names, vias=all_vias)

        # Always include the designated source pad position as a potential source
        # This is critical for zone-connected pads that have no segments to them yet
        src_x, src_y = src_pad[3], src_pad[4]
        src_gx, src_gy = coord.to_grid(src_x, src_y)
        src_pad_obj = src_pad[5]

        # Build initial tap point map from segment/via tap points
        if all_tap_points:
            sources = [(gx, gy, layer_idx) for gx, gy, layer_idx, _, _ in all_tap_points]
            tap_point_map = {(gx, gy, layer_idx): (ox, oy, layer_names[layer_idx])
                            for gx, gy, layer_idx, ox, oy in all_tap_points}
        else:
            sources = []
            tap_point_map = {}

        # Add source pad position as a valid source (on all layers for
        # through-hole pads and pads with a same-net via at their center)
        if _pad_all_layer_reach(pcb_data, src_pad_obj):
            # Reaches any copper layer
            for layer_idx in range(len(layer_names)):
                key = (src_gx, src_gy, layer_idx)
                if key not in tap_point_map:
                    sources.append(key)
                    tap_point_map[key] = (src_x, src_y, layer_names[layer_idx])
        else:
            # SMD pad - use specific layer from pad_info
            key = (src_gx, src_gy, src_pad[2])
            if key not in tap_point_map:
                sources.append(key)
                tap_point_map[key] = (src_x, src_y, layer_names[src_pad[2]])

        # #545 F1: every cell of every ROUTED component's existing island is
        # a legal launch (its stubs, trunks, vias on all layers), exactly
        # like Phase 1's island-wide launch. Restricted to routed components
        # (#189). sorted() for GUI/CLI order-independence.
        # The ids MUST come from the same labeling that keys
        # _p3_island_cells: `routed_components` is Phase-1's labeling, and
        # get_terminal_component_info renumbers densely after Phase-1 copper
        # merges pads -- indexing the fresh map with the stale ids collided
        # a routed pad's old singleton label with an UNROUTED island's fresh
        # label, seeding sources on the target's own island. A* then popped
        # a source that was the goal on iteration 1 and credited a
        # zero-length phantom "routed" edge -- the exact #189 violation this
        # branch guards against (test_432 scenario C: 2/0 edges where the
        # truth was 1/1, and on real boards a suppressed failure record).
        if _p3_use_islands:
            for _cid in sorted({_p3_comps.get(i, i) for i in routed_indices}):
                _isl = _p3_island_cells.get(_cid)
                if not _isl:
                    continue
                for _cell, _owner in _isl.items():
                    if _cell not in tap_point_map:
                        sources.append(_cell)
                        tap_point_map[_cell] = (_owner[0], _owner[1],
                                                layer_names[_cell[2]])

        if not sources:
            print(f"      ERROR: No sources available for routing")
            continue

        # Targets on ALL layers for through-hole pads AND pads with a
        # same-net via at their center (the router can land on any layer)
        tgt_gx, tgt_gy = tgt_pad[0], tgt_pad[1]
        tgt_pad_obj = tgt_pad[5]
        if _pad_all_layer_reach(pcb_data, tgt_pad_obj):
            targets = [(tgt_gx, tgt_gy, layer_idx) for layer_idx in range(len(layer_names))]
        else:
            # SMD pad or specific layer - use the layer from pad_info
            targets = [(tgt_gx, tgt_gy, tgt_pad[2])]

        # #545 F2: the TARGET pad's whole island is a legal landing -- a tap
        # reaching any cell of the island connects the component (the pad is
        # on it by the graph's own membership). The old single-cell,
        # single-layer target made the rip ladder rip neighbours a better
        # landing set would never have needed. The end-weld below maps a
        # landing cell back to ITS owner point, never the distant pad centre.
        _tgt_isl = {}
        if _p3_use_islands:
            # Same ID-space rule as the source seeding above: look the
            # target's island up through the FRESH labeling, not Phase-1's
            # pad_components (the stale id usually missed entirely, so #545
            # F2 target-island landing silently never worked).
            _tgt_isl = _p3_island_cells.get(
                _p3_comps.get(tgt_idx, tgt_idx), {})
            if _tgt_isl:
                _tset = set(targets)
                for _cell in _tgt_isl:
                    if _cell not in _tset:
                        targets.append(_cell)
                        _tset.add(_cell)

        # Mark source/target cells
        for gx, gy, layer in sources + targets:
            obstacles.add_source_target_cell(gx, gy, layer)

        # Add allowed cells around target to escape blocked areas
        allow_radius = 5
        tgt_gx, tgt_gy = tgt_pad[0], tgt_pad[1]
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(tgt_gx + dx, tgt_gy + dy)

        # Check which proximity zones the endpoints are in for precise heuristic estimate
        src_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in sources)
        src_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in sources)
        tgt_in_stub = any(obstacles.get_stub_proximity_cost(gx, gy) > 0 for gx, gy, _ in targets)
        tgt_in_bga = any(obstacles.is_in_bga_proximity(gx, gy) for gx, gy, _ in targets)
        prox_h_cost = config.get_proximity_heuristic_for_zones(src_in_stub, src_in_bga, tgt_in_stub, tgt_in_bga)
        router.set_proximity_heuristic_cost(prox_h_cost)
        if config.verbose:
            zones = []
            if src_in_stub: zones.append("src:stub")
            if src_in_bga: zones.append("src:bga")
            if tgt_in_stub: zones.append("tgt:stub")
            if tgt_in_bga: zones.append("tgt:bga")
            print(f"      proximity_heuristic_cost={prox_h_cost} zones=[{', '.join(zones) if zones else 'none'}]")

        # Route from ANY tap point to target - router finds shortest path
        # Use probe routing helper to detect stuck directions early
        tap_start_time = time.time()

        _via_conflict_extra = []
        # C5 parity for taps (#424): the tap edge's TARGET pad pays full stub
        # proximity on its final approach cells if a foreign stub sits beside
        # it -- single-ended routes got the endpoint exemption, tap edges
        # never did. Replaced per edge; cleared after the loop.
        obstacles.set_endpoint_exempt(
            [(t[0], t[1]) for t in targets[:8]],
            coord.to_grid_dist(config.track_width + config.clearance))

        # Hold this edge's search diagnostics until its outcome is known:
        # the via-in-pad rescue below often turns a stalled edge into a
        # routed one, and then nothing needs explaining.
        with deferred_diagnostics(config) as _edge_diag:
            (path, tap_iterations, forward_blocked, backward_blocked, reversed_tap_path,
             _, _, necked_down, uniform_width, unblock_vias,
             unblock_segments) = _route_with_via_unblock(
                router, obstacles, config, sources, targets, track_margin,
                pcb_data, net_id, print_prefix="      ", direction_labels=("forward", "backward"),
                waypoints=waypoint_buckets.get(frozenset((src_idx, tgt_idx)), [])
            )

            if path is None:
                # Multipoint parity rung (#424): the #189 unblock inside the
                # wrapper fires only on the walled-in signature (probe stuck
                # BELOW its limit); a CONGESTION failure burns the full budget
                # and is never offered the via a human would drop (ottercast
                # R86: both pads of a series resistor stranded in the U1 pocket
                # while the net routes fine alone). Place a validated fab-floor
                # via in the target pad UNCONDITIONALLY and retry once at the
                # probe budget; the memoisation cache in _place_shrunk_via_in_pad
                # keeps repeated failures cheap.
                _pad_obj = pad_info[tgt_idx][5] if len(pad_info[tgt_idx]) > 5 else None
                # Real pads only: tap targets can be _EndpointStub pseudo-pads
                # (mid-trace tap points) -- no copper of their own to via into.
                if getattr(_pad_obj, 'component_ref', None) is None:
                    _pad_obj = None
                _r189 = (_place_shrunk_via_in_pad(_pad_obj, obstacles, config,
                                                  pcb_data, net_id, coord, layer_names)
                         if _pad_obj is not None and not getattr(_pad_obj, 'drill', 0)
                         else None)
                if _unblock_debug():
                    _pname = (f"{_pad_obj.component_ref}.{_pad_obj.pad_number}"
                              if _pad_obj is not None else "NO-PAD-OBJ")
                    print(f"      TAP-RESCUE rung: tgt {_pname} -> "
                          f"{'placed' if _r189 else 'DECLINED'}")
                if _r189 is None and _pad_obj is not None:
                    # Rip-integrated terminal access (#424): name the copper the
                    # validator saw and feed it to the rip cascade as synthetic
                    # frontier cells (see _pad_via_conflict_cells).
                    _via_conflict_extra = _pad_via_conflict_cells(
                        pcb_data, _pad_obj, config, coord, layer_names)
                    if _unblock_debug() and _via_conflict_extra:
                        print(f"      TAP-RESCUE rung: {len(_via_conflict_extra)} "
                              f"conflict cell(s) fed to rip attribution")
                if _r189 is not None:
                    _via189, (_vgx, _vgy), _pli, _stub189 = _r189
                    _register_unblock_via(obstacles, _vgx, _vgy, layer_names)
                    _retry_cfg = replace(config, max_iterations=config.max_probe_iterations)
                    _tgts2 = list(targets) + [(_vgx, _vgy, li)
                                              for li in range(len(layer_names))]
                    (path, _it2, _fb2, _bb2, reversed_tap_path, _, _,
                     necked_down, uniform_width) = _route_main_connection(
                        router, obstacles, _retry_cfg, sources, _tgts2, track_margin,
                        pcb_data, net_id, print_prefix="      ")
                    total_iterations += _it2
                    if path is not None:
                        unblock_vias = list(unblock_vias) + [_via189]
                        unblock_segments = list(unblock_segments) + _stub189
                        # #803: same as the _route_with_via_unblock keep-path --
                        # commit the kept copper to pcb_data NOW. _register_
                        # unblock_via above is purely permissive, and this via is
                        # not in pcb_data until the tap result commits, so until
                        # then it blocks nobody and a foreign track routes
                        # through it. add_route_to_pcb_data dedupes by id()
                        # (#195), so the caller's later commit skips these.
                        if not getattr(config, 'plan_probe', False):
                            _hv = {id(v) for v in pcb_data.vias}
                            if id(_via189) not in _hv:
                                pcb_data.vias.append(_via189)
                            _hs = {id(sg) for sg in pcb_data.segments}
                            for _sg in _stub189:
                                if id(_sg) not in _hs:
                                    pcb_data.segments.append(_sg)
                            pcb_data._copper_epoch = getattr(
                                pcb_data, '_copper_epoch', 0) + 1
                        print(f"      {GREEN}TAP PAD-VIA RESCUE: edge routed after "
                              f"unconditional via-in-pad at "
                              f"{_pad_obj.component_ref}.{_pad_obj.pad_number}{RESET}")

        # If path was found in reverse direction, reverse it so it goes sources -> targets
        if path is not None and reversed_tap_path:
            path = list(reversed(path))

        # Combine blocked cells from both directions for rip-up analysis
        blocked_cells = forward_blocked + backward_blocked

        tap_elapsed = time.time() - tap_start_time
        total_iterations += tap_iterations

        if path is None:
            flush_diagnostics(_edge_diag)
            print(f"      {YELLOW}Failed to route MST edge after {tap_iterations} iterations ({tap_elapsed:.2f}s){RESET}")
            edge_key = (min(src_idx, tgt_idx), max(src_idx, tgt_idx))
            failed_edges.add(edge_key)
            # Store blocking info for potential rip-up analysis; the synthetic
            # via-conflict cells ride along so the cascade can name the
            # pad-hugging copper the frontier never reaches (#424).
            blocked_cells = list(blocked_cells) + _via_conflict_extra
            if blocked_cells:
                # Element 2 keeps the directions SEPARATE (audit #2ii): the
                # backward probe from a walled-in pad drains in dozens of
                # iterations and its frontier is precisely the pocket wall --
                # pooling buried it under the forward flood. The cascade
                # picks the tighter side per edge; 'extra' carries the
                # synthetic via-conflict cells either way.
                failed_edge_blocking[edge_key] = (
                    blocked_cells, (tgt_x, tgt_y),
                    {'fwd': list(forward_blocked), 'bwd': list(backward_blocked),
                     'extra': list(_via_conflict_extra)})
            continue

        print(f"      Routed in {tap_iterations} iterations ({tap_elapsed:.2f}s)")

        # Get the actual tap point used (first point of path)
        path_start = path[0]  # (gx, gy, layer_idx)
        if path_start in tap_point_map:
            tap_x, tap_y, tap_layer = tap_point_map[path_start]
        else:
            # Fallback: convert grid coords back to original
            tap_x, tap_y = coord.to_float(path_start[0], path_start[1])
            tap_layer = layer_names[path_start[2]]

        # Convert path to segments/vias
        # Use the actual end layer from the path (router may reach through-hole pad on any layer)
        path_end_layer = layer_names[path[-1][2]]
        # #545 F2 end-weld: a path that landed on a cell of the target's
        # island welds to THAT cell's owner float point -- welding an island
        # landing to the pad centre would draw a long any-angle slash to a
        # route that ended elsewhere on the island (the SDC0_CMD class,
        # tap_point_map contract mirrored on the target side).
        if path[-1] in _tgt_isl:
            _ex, _ey = _tgt_isl[path[-1]]
            end_original = (_ex, _ey, path_end_layer)
        else:
            end_original = (tgt_x, tgt_y, path_end_layer)
        # Barrel-in-fill completion (#562): the edge is done at the first via
        # piercing the landing region -- truncate the tail before emission.
        path, _fill_end = _trim_after_fill_via(path, coord, layer_names,
                                               pcb_data, net_id)
        if _fill_end is not None:
            end_original = _fill_end
            path_end_layer = _fill_end[2]
        segments, vias = _path_to_segments_vias(
            path, coord, layer_names, net_id, config,
            (tap_x, tap_y, tap_layer),  # start_original (actual tap point used)
            end_original,
            through_hole_positions,
            pcb_data
        )
        if necked_down:
            segments = _apply_neckdown_widths(segments, config, net_id, obstacles,
                                              coord, layer_names, track_margin)
        elif uniform_width is not None:
            # Short power edge routed at a stepped-down width (#180): uniform width
            # so obstacle blocking (reads seg.width) and output match.
            for _s in segments:
                _s.width = uniform_width
        # Re-neck terminal grazes AFTER width assignment (#212): the neckdown/uniform
        # passes rebuild widths and would otherwise restore a grazing terminal leg to
        # base/power width, undoing the graze-neck applied during conversion.
        _hard_tap = _neck_route_terminal_grazes(segments, path, coord,
                                                (tap_x, tap_y), (tgt_x, tgt_y),
                                                pcb_data, net_id, config)
        if _hard_tap:
            # Terminal-bridge SHORT gate (ux pf9: GND's tap edge into C61 --
            # the anchor cell was blocked, the A* ended a cell short, and the
            # bridge slashed across SDRAM_A2's In1.Cu trunk; the #157 neck
            # took it to the fab floor and the crossing SHIPPED). Terminal
            # copper overlapping a foreign track/via at any width is a short
            # no neck can fix: fail the edge like an unroutable one so the
            # rip/retry ladder finds another approach. (A #189 unblock via
            # already registered for this edge stays in the map -- a
            # conservative over-block for later edges, never a short.)
            _hs, _hd = _hard_tap[0]
            print(f"      {YELLOW}terminal copper on {_hs.layer} would "
                  f"OVERLAP a foreign track/via (edge dist {_hd:.3f}mm) -- "
                  f"failing the MST edge rather than shipping a short{RESET}")
            edge_key = (min(src_idx, tgt_idx), max(src_idx, tgt_idx))
            failed_edges.add(edge_key)
            blocked_cells = list(blocked_cells) + _via_conflict_extra
            if blocked_cells:
                failed_edge_blocking[edge_key] = (
                    blocked_cells, (tgt_x, tgt_y),
                    {'fwd': list(forward_blocked),
                     'bwd': list(backward_blocked),
                     'extra': list(_via_conflict_extra)})
            continue
        # Any fab-floor via dropped INSIDE the boxed target pad to unblock this
        # edge (issue #189) -- it connects the inner-layer path end to the pad by
        # copper overlap, no extra trace needed. A #535 off-pad escape via
        # instead ships its pad->via stub alongside.
        vias = list(vias) + unblock_vias
        segments = list(segments) + unblock_segments
        # A tap edge that launches from an off-grid tap point an earlier edge
        # already bridged re-emits that edge's endpoint connector (path[0]
        # maps through tap_point_map back to the sampled copper's ORIGINAL
        # float point, and _path_to_segments_vias bridges original->grid
        # again). Emit only copper the net does not already have, so no
        # duplicate coincident segment ships to the output. Runs last, after
        # the width passes above, since the twin key includes seg.width.
        segments = _drop_segments_already_present(segments, all_segments)
        all_segments.extend(segments)
        all_vias.extend(vias)
        # Make this edge's vias reusable by later edges of the same net, so a
        # later edge changing layers at one of these cells reuses the via, and
        # block a hole-to-hole ring so a later edge can't drop a via beside it.
        for _v in vias:
            through_hole_positions.add(coord.to_grid(_v.x, _v.y))
            _register_inprogress_via(_v)

        # Note: We don't add segments as obstacles since they're the same net
        # and future tap routes can overlap with our own traces

        # Mark target pad as routed and its component as routed
        routed_indices.add(tgt_idx)
        tgt_component = pad_components.get(tgt_idx, tgt_idx)
        routed_components.add(tgt_component)
        remaining_edges = [e for e in remaining_edges if not (
            (e[0] == src_idx and e[1] == tgt_idx) or (e[0] == tgt_idx and e[1] == src_idx)
        )]
        edges_routed += 1

    # C5 parity cleanup: no stale endpoint disks on the shared map.
    obstacles.clear_endpoint_exempt()

    # Count pads that are effectively connected (either explicitly routed or zone-connected to a routed pad)
    pads_connected = sum(1 for i in range(len(pad_info))
                         if i in routed_indices or pad_components.get(i, i) in routed_components)
    pads_total = len(pad_info)
    pads_failed = pads_total - pads_connected

    # Collect detailed info about failed (unconnected) pads
    failed_pads_info = []
    for i in range(len(pad_info)):
        if i not in routed_indices and pad_components.get(i, i) not in routed_components:
            pad = pad_info[i]
            pad_obj = pad[5] if len(pad) > 5 else None
            failed_pads_info.append({
                'pad_idx': i,
                'x': pad[3],  # orig_x
                'y': pad[4],  # orig_y
                'component_ref': getattr(pad_obj, 'component_ref', '?') if pad_obj else '?',
                'pad_number': getattr(pad_obj, 'pad_number', '?') if pad_obj else '?',
            })

    print(f"  Phase 3 routing complete: {edges_routed} edges, {len(all_segments)} total segments, {len(all_vias)} total vias")

    # Update result - preserve original fields, update segments/vias
    updated_result = dict(main_result)
    updated_result['new_segments'] = all_segments
    updated_result['new_vias'] = all_vias
    updated_result['iterations'] = main_result['iterations'] + total_iterations
    updated_result['routed_pad_indices'] = routed_indices
    # Tap routing stats (add Phase 3 to Phase 1 counts)
    updated_result['tap_edges_routed'] = main_result.get('tap_edges_routed', 0) + edges_routed
    updated_result['tap_edges_failed'] = main_result.get('tap_edges_failed', 0) + len(failed_edges)
    updated_result['tap_pads_connected'] = pads_connected
    updated_result['tap_pads_total'] = pads_total
    # Detailed info about unconnected pads (for summary)
    updated_result['failed_pads_info'] = failed_pads_info
    # Blocking info for failed edges (for rip-up analysis)
    updated_result['failed_edge_blocking'] = failed_edge_blocking

    return updated_result


def _drop_segments_already_present(segments: List[Segment],
                                   existing: List[Segment]) -> List[Segment]:
    """Drop segments that exactly duplicate one already in ``existing``
    (same endpoints in either orientation, same layer, width and net).

    The Phase-3 tap flow launches from points ON the net's existing copper
    (get_all_segment_tap_points). When the A* start cell maps back through
    tap_point_map to an off-grid original point that an earlier edge already
    bridged to that same grid cell, _path_to_segments_vias re-emits the
    identical endpoint connector -- a second coincident copy of a tiny
    (~0.02 mm) segment at the stub tip. A geometric twin adds no copper, so
    dropping it changes neither connectivity nor DRC.
    """
    def _key(s):
        return ((round(s.start_x, 6), round(s.start_y, 6)),
                (round(s.end_x, 6), round(s.end_y, 6)),
                s.layer, round(s.width, 6), s.net_id)

    existing_keys = set()
    for s in existing:
        k = _key(s)
        existing_keys.add(k)
        existing_keys.add((k[1], k[0]) + k[2:])
    return [s for s in segments if _key(s) not in existing_keys]


def _terminal_copper_on_layer(pcb_data, net_id, x, y, layer) -> bool:
    """True if this net already has copper at (x, y) on `layer`.

    The terminal connectors below project the ORIGINAL endpoint XY onto the
    layer the A* actually finished on. That is correct for a plated-through
    pad or a via -- the barrel carries own-net copper on every layer it spans,
    so the stub lands on real copper. It is a fabrication when the terminal
    only exists on its own layer: the projected stub then joins nothing, and
    because terminal cells are obstacle-EXEMPT it can be stamped straight
    through foreign copper (#505 cparti_fpga: a +3V3 plane-repair stub laid
    along an FPGA_CFG_D3 track on In2.Cu, a dead short at 0.000mm).
    """
    if pcb_data is None:
        return False
    tol = 1e-6
    pads = (getattr(pcb_data, 'pads_by_net', None) or {}).get(net_id) or ()
    for p in pads:
        if not pad_is_plated_through(p):
            continue
        hw = (getattr(p, 'size_x', 0) or 0) / 2.0
        hh = (getattr(p, 'size_y', 0) or 0) / 2.0
        if abs(x - p.global_x) <= hw + tol and abs(y - p.global_y) <= hh + tol:
            return True
    copper_layers = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    for v in pcb_data.vias:
        if v.net_id != net_id or len(v.layers or ()) < 2:
            continue
        radius = max((getattr(v, 'size', 0) or 0) / 2.0, tol)
        if math.hypot(x - v.x, y - v.y) > radius + tol:
            continue
        span = list(v.layers)
        if copper_layers and span[0] in copper_layers and span[1] in copper_layers:
            i, j = copper_layers.index(span[0]), copper_layers.index(span[1])
            spanned = copper_layers[min(i, j):max(i, j) + 1]
        else:
            spanned = span
        if layer in spanned:
            return True
    return False


def _trim_after_fill_via(path, coord, layer_names, pcb_data, net_id):
    """#562 barrel-in-fill completion (the U2-class tail trim).

    A weld edge is electrically complete at the first VIA whose barrel
    pierces the same surviving fill region its landing cell lies in -- the
    barrel spans every layer, so fill contact at the via IS the connection
    (#424 pour-direct's economy, mirrored on the route side). Without this,
    the A* must run on to stand on an explicit sampled ladder cell, paying
    ~2mm + a second via per affected weld (measured on an SDRAM rail
    cluster). Truncate the path just after that via and weld the end to the
    via's own float point.

    Only fires with pour-launch enabled (default ON; KICAD_POUR_LAUNCH=0
    disables) and only when the path's end cell
    resolves to a fill region of this net (i.e. this IS a pour-launch weld);
    the trim keeps the via pair itself, and the same-region requirement
    keeps completion honest (piercing an unrelated island is not arrival).
    Returns (possibly-truncated path, end_original override or None).
    """
    import os as _os
    if not env_knobs.POUR_LAUNCH or len(path) < 4:
        return path, None
    try:
        from plane_fill_model import get_fill_models
        models = get_fill_models(pcb_data, net_id)
    except Exception:
        return path, None
    if not models:
        return path, None
    egx, egy, eli = path[-1]
    ex, ey = coord.to_float(egx, egy)
    end_model = end_label = None
    for _m in models.get(layer_names[eli], []):
        _c = _m.query_component(ex, ey) or 0
        if _c > 0:
            end_model, end_label = _m, _c
            break
    if end_model is None:
        return path, None
    for i in range(len(path) - 2):
        a, b = path[i], path[i + 1]
        # A via is emitted on ANY consecutive layer change, placed at the
        # FIRST node's cell (_path_to_segments_vias contract) -- the path
        # never duplicates a cell across the transition.
        if a[2] != b[2]:
            vx, vy = coord.to_float(a[0], a[1])
            if (end_model.query_component(vx, vy) or 0) == end_label:
                print(f"      barrel-in-fill: weld complete at via "
                      f"({vx:.2f},{vy:.2f}); trimmed {len(path) - i - 2} "
                      f"tail node(s)")
                return path[:i + 2], (vx, vy, layer_names[b[2]])
    return path, None


def _path_to_segments_vias(
    path: List[Tuple[int, int, int]],
    coord: GridCoord,
    layer_names: List[str],
    net_id: int,
    config: GridRouteConfig,
    start_original: Tuple[float, float, str],
    end_original: Tuple[float, float, str],
    through_hole_positions: Set[Tuple[int, int]] = None,
    pcb_data: PCBData = None
) -> Tuple[List[Segment], List[Via]]:
    """
    Convert a grid path to Segment and Via objects.

    Args:
        path: List of (gx, gy, layer_idx) grid points
        coord: Grid coordinate converter
        layer_names: List of layer names
        net_id: Net ID for segments/vias
        config: Routing config with track width, via size
        start_original: (x, y, layer) of path start in float coords
        end_original: (x, y, layer) of path end in float coords
        through_hole_positions: Optional set of (gx, gy) positions where through-hole
            pads exist on this net. Layer changes at these positions don't need a
            new via since the existing through-hole provides the layer transition.

    Returns:
        (segments, vias): Lists of Segment and Via objects
    """
    segments = []
    vias = []

    if not path:
        return segments, vias

    # Simplify path by removing collinear intermediate points
    path = simplify_path(path)

    path_start = path[0]
    path_end = path[-1]

    # Per-point float positions. Route the two TERMINAL segments to the EXACT
    # off-grid endpoint instead of its grid-cell stand-in when that cell grazes a
    # foreign pad but the exact endpoint clears it (#4 off-grid connection graze).
    pts = [coord.to_float(p[0], p[1]) for p in path]
    merge_start = merge_end = False
    if pcb_data is not None:
        merge_start = _merge_terminal_to_exact(path, 0, 1, start_original, pts,
                                               pcb_data, net_id, config, layer_names)
        merge_end = _merge_terminal_to_exact(path, len(path) - 1, len(path) - 2, end_original, pts,
                                             pcb_data, net_id, config, layer_names)

    # Add connecting segment from original start to first path point if needed
    # (skipped when merged: the first path segment now ends at the exact point)
    if start_original and not merge_start:
        first_grid_x, first_grid_y = pts[0]
        orig_x, orig_y, orig_layer = start_original
        # Use the actual path layer, not the original pad layer
        # (through-hole pads may have orig_layer=F.Cu but router chose In1.Cu)
        path_start_layer = layer_names[path_start[2]]
        # ...but only when the terminal really has copper there (#505): a
        # layer-specific target projected onto a layer the A* merely ended on
        # joins nothing and can short whatever occupies that layer.
        if ((abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001)
                and (orig_layer == path_start_layer or pcb_data is None
                     or _terminal_copper_on_layer(pcb_data, net_id, orig_x, orig_y,
                                                  path_start_layer))):
            seg = Segment(
                start_x=orig_x, start_y=orig_y,
                end_x=first_grid_x, end_y=first_grid_y,
                width=config.get_net_track_width(net_id, path_start_layer),
                layer=path_start_layer,
                net_id=net_id
            )
            segments.append(seg)

    # Convert path points to segments and vias
    for i in range(len(path) - 1):
        gx1, gy1, layer1 = path[i]
        gx2, gy2, layer2 = path[i + 1]

        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]

        if layer1 != layer2:
            # Check if layer change is at an existing through-hole pad
            # If so, skip creating a via - the pad provides the layer transition
            if through_hole_positions and (gx1, gy1) in through_hole_positions:
                # No via needed - existing through-hole pad connects all layers
                pass
            else:
                vx, vy = coord.to_float(gx1, gy1)  # via stays on the grid cell
                _vsz, _vdr = _emit_via_size(pcb_data, gx1, gy1, config,
                                            net_id=net_id, x=vx, y=vy)
                via = Via(
                    x=vx, y=vy,
                    size=_vsz,
                    drill=_vdr,
                    layers=["F.Cu", "B.Cu"],  # Always through-hole
                    net_id=net_id
                )
                vias.append(via)
        else:
            if (x1, y1) != (x2, y2):
                layer_name = layer_names[layer1]
                seg = Segment(
                    start_x=x1, start_y=y1,
                    end_x=x2, end_y=y2,
                    width=config.get_net_track_width(net_id, layer_name),
                    layer=layer_name,
                    net_id=net_id
                )
                segments.append(seg)

    # Add connecting segment from last path point to original end if needed
    # (skipped when merged: the last path segment now ends at the exact point)
    if end_original and not merge_end:
        last_grid_x, last_grid_y = pts[-1]
        orig_x, orig_y, orig_layer = end_original
        # Use the actual path layer, not the original pad layer
        path_end_layer = layer_names[path_end[2]]
        # ...but only when the terminal really has copper there (#505) -- see
        # the start-side note above.
        if ((abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001)
                and (orig_layer == path_end_layer or pcb_data is None
                     or _terminal_copper_on_layer(pcb_data, net_id, orig_x, orig_y,
                                                  path_end_layer))):
            seg = Segment(
                start_x=last_grid_x, start_y=last_grid_y,
                end_x=orig_x, end_y=orig_y,
                width=config.get_net_track_width(net_id, path_end_layer),
                layer=path_end_layer,
                net_id=net_id
            )
            segments.append(seg)

    # Neck any terminal-connection segment that grazes a foreign pad (#157): the
    # exact-endpoint stub / first-last leg is obstacle-exempt at the endpoint, so a
    # full-width terminal can sit sub-clearance to a neighbouring foreign pad.
    if pcb_data is not None:
        term_pts = [pts[0], pts[-1]]
        if start_original:
            term_pts.append((start_original[0], start_original[1]))
        if end_original:
            term_pts.append((end_original[0], end_original[1]))
        # Neck-only here (hard overlaps deliberately not consumed): this
        # converter cannot fail a route, and every caller re-runs the neck via
        # _neck_route_terminal_grazes AFTER its width passes -- THAT call is
        # the authoritative short gate.
        _neck_terminal_grazes(segments, term_pts, pcb_data, net_id, config)

    return segments, vias


def _seg_length(seg) -> float:
    return math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)


def _split_segment_at(seg, dist_from_end: float):
    """Split a segment at dist_from_end mm before its end point.

    Returns (near_part, far_part) where far_part is the dist_from_end-long
    piece touching seg's end. Returns (None, seg) if the segment is shorter
    than dist_from_end.
    """
    length = _seg_length(seg)
    if length <= dist_from_end:
        return None, seg
    t = 1.0 - dist_from_end / length
    mx = seg.start_x + (seg.end_x - seg.start_x) * t
    my = seg.start_y + (seg.end_y - seg.start_y) * t
    near = Segment(start_x=seg.start_x, start_y=seg.start_y, end_x=mx, end_y=my,
                   width=seg.width, layer=seg.layer, net_id=seg.net_id)
    far = Segment(start_x=mx, start_y=my, end_x=seg.end_x, end_y=seg.end_y,
                  width=seg.width, layer=seg.layer, net_id=seg.net_id)
    return near, far


def _segment_fits_wide(seg, obstacles, coord: GridCoord, layer_idx: int, margin: float) -> bool:
    """True if the segment's swept body clears the wide-track margin (#156:
    fractional, checked with the same Euclidean capsule the A* uses -- the old
    per-cell Chebyshev walk both over-covered corners and missed the swept
    body of diagonal segments)."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    if margin > 0:
        return not obstacles.segment_blocked(gx1, gy1, gx2, gy2, layer_idx, float(margin))
    # Zero margin (base-width): segment_blocked's r<=0 fast path only checks the
    # endpoint, so keep the per-cell walk for whole-line coverage.
    for gx, gy in walk_line(gx1, gy1, gx2, gy2):
        if obstacles.is_blocked(gx, gy, layer_idx):
            return False
    return True


def _flip_segments(segments):
    """Reverse a connected segment run end-to-end (order and direction)."""
    return [Segment(start_x=s.end_x, start_y=s.end_y, end_x=s.start_x, end_y=s.start_y,
                    width=s.width, layer=s.layer, net_id=s.net_id)
            for s in reversed(segments)]


def _neck_width_for_net(config: GridRouteConfig, net_id: int, layer: str) -> float:
    """The width a neck-down narrows to on `layer` for this net: a POWER net
    (configured wider than the layer routing width) necks to the LAYER width,
    exactly as always; an IMPEDANCE-width net (layer width above nominal,
    KICAD_IMPEDANCE_NECKDOWN allow, #465) necks to the NOMINAL track width."""
    lw = config.get_track_width(layer)
    if config.get_net_track_width(net_id, layer) > lw + 1e-9:
        return lw
    return min(lw, config.track_width)


def _neck_pass(segments, config: GridRouteConfig, obstacles, coord: GridCoord,
               layer_map: Dict[str, int], track_margin, net_id: int):
    """Narrow the last neckdown_length mm of the run (the pad is at the list
    END); beyond that, keep the wide width only where the wide clearance
    fits. Never re-widens an already-narrow segment (so a second pass from
    the other end preserves the first pass's neck). track_margin may be a
    scalar or a per-layer list (#156)."""
    def fits(s):
        li = layer_map.get(s.layer, 0)
        return _segment_fits_wide(s, obstacles, coord, li, _margin_at(track_margin, li))

    out = []  # built in reverse (pad-first)
    cum = 0.0
    for seg in reversed(segments):
        narrow_w = _neck_width_for_net(config, net_id, seg.layer)
        length = _seg_length(seg)
        if seg.width <= narrow_w:
            out.append(seg)
        elif cum >= config.neckdown_length:
            if not fits(seg):
                seg.width = narrow_w
            out.append(seg)
        elif cum + length > config.neckdown_length:
            # Straddles the neck boundary: split there (the far piece,
            # touching the pad side, is neckdown_length - cum long)
            near, far = _split_segment_at(seg, config.neckdown_length - cum)
            far.width = narrow_w
            out.append(far)
            if not fits(near):
                near.width = narrow_w
            out.append(near)
        else:
            seg.width = narrow_w
            out.append(seg)
        cum += length
    out.reverse()
    return out


def _apply_neckdown_widths(segments, config: GridRouteConfig, net_id: int,
                           obstacles, coord: GridCoord, layer_names: List[str],
                           track_margin, neck_start: bool = False):
    """Assign widths to a neck-down route (issue #72).

    The path was routed at the layer's default width because the power width
    did not fit. Segments within config.neckdown_length of the target pad
    (the END of the list; also the start when neck_start is set, for routes
    that end on pads at both ends) stay narrow; farther segments return to
    the power width wherever the wide clearance fits, with an optional
    stepped taper at each narrow->wide transition.

    Returns a new segment list (segments may be split for the taper).
    """
    layer_map = {name: i for i, name in enumerate(layer_names)}
    out = _neck_pass(segments, config, obstacles, coord, layer_map, track_margin, net_id)
    if neck_start:
        out = _flip_segments(_neck_pass(_flip_segments(out), config, obstacles,
                                        coord, layer_map, track_margin, net_id))
    wide_flags = [s.width > _neck_width_for_net(config, net_id, s.layer) for s in out]

    # Suppress short wide islands (a wide run between narrow pinches that is
    # barely longer than its tapers just adds notch noise)
    min_island = 2 * config.neckdown_taper_length
    i = 0
    while i < len(out):
        if not wide_flags[i]:
            i += 1
            continue
        j = i
        run_len = 0.0
        while j < len(out) and wide_flags[j]:
            run_len += _seg_length(out[j])
            j += 1
        is_island = i > 0 and j < len(out)  # narrow (or pad) on both sides
        if is_island and run_len <= min_island:
            for k in range(i, j):
                out[k].width = _neck_width_for_net(config, net_id, out[k].layer)
                wide_flags[k] = False
        i = j

    if config.neckdown_taper_length <= 0:
        return out

    # Stepped taper wherever a wide segment meets a narrow one on the same
    # layer: carve the wide segment's adjoining end into width steps
    TAPER_STEPS = 4

    def _taper_pieces(seg, narrow_end: str):
        """Split seg into [body + taper steps]; narrow_end is 'start' or 'end'."""
        narrow_w = _neck_width_for_net(config, net_id, seg.layer)
        wide_w = seg.width
        taper_len = min(config.neckdown_taper_length, _seg_length(seg) / 3)
        if taper_len <= 0:
            return [seg]
        flipped = narrow_end == 'start'
        if flipped:  # work as if the narrow side is at the end
            seg = Segment(start_x=seg.end_x, start_y=seg.end_y,
                          end_x=seg.start_x, end_y=seg.start_y,
                          width=seg.width, layer=seg.layer, net_id=seg.net_id)
        body, taper = _split_segment_at(seg, taper_len)
        if body is None:
            return [seg]
        pieces = [body]
        step_len = taper_len / TAPER_STEPS
        remaining = taper
        for s in range(TAPER_STEPS):
            if s < TAPER_STEPS - 1 and _seg_length(remaining) > step_len:
                piece, remaining = _split_segment_at(remaining, _seg_length(remaining) - step_len)
            else:
                piece, remaining = remaining, None
            piece.width = wide_w + (narrow_w - wide_w) * (s + 1) / (TAPER_STEPS + 1)
            pieces.append(piece)
            if remaining is None:
                break
        if flipped:  # restore original direction and order
            pieces = [Segment(start_x=p.end_x, start_y=p.end_y,
                              end_x=p.start_x, end_y=p.start_y,
                              width=p.width, layer=p.layer, net_id=p.net_id)
                      for p in reversed(pieces)]
        return pieces

    tapered = []
    for i, seg in enumerate(out):
        if not wide_flags[i]:
            tapered.append(seg)
            continue
        narrow_after = (i + 1 < len(out) and not wide_flags[i + 1]
                        and out[i + 1].layer == seg.layer)
        narrow_before = (i > 0 and not wide_flags[i - 1]
                         and out[i - 1].layer == seg.layer)
        pieces = [seg]
        if narrow_after:
            pieces = _taper_pieces(seg, 'end')
        if narrow_before:
            head = _taper_pieces(pieces[0], 'start')
            pieces = head + pieces[1:]
        tapered.extend(pieces)
    return tapered
