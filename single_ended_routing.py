"""
Single-ended net routing functions.

Routes individual nets using A* pathfinding on a grid obstacle map.
"""
from __future__ import annotations

import math
import time
import numpy as np
from typing import Dict, List, Optional, Set, Tuple
from terminal_colors import YELLOW, GREEN, RESET

from dataclasses import replace
from kicad_parser import PCBData, Segment, Via
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
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))

try:
    from grid_router import GridObstacleMap, GridRouter, VisualRouter
except ImportError:
    GridObstacleMap = None
    GridRouter = None
    VisualRouter = None

# Read once at import: checked inside per-via-record emit paths (hot).
def _unblock_debug() -> bool:
    """KICAD_UNBLOCK_DEBUG, read PER CALL (#382 E10).

    Was a module-level constant frozen at import, so a GUI process that set the
    env var after the module loaded (or unset it) saw the stale import-time
    value. Reading os.environ each call matches every other KICAD_* debug flag.
    """
    return bool(os.environ.get('KICAD_UNBLOCK_DEBUG'))


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
    cache = getattr(pcb_data, '_foreign_pad_arr_cache', None)
    if cache is None:
        cache = {}
        pcb_data._foreign_pad_arr_cache = cache
    arr = cache.get(layer)
    if arr is None:
        nids, cx, cy, hx, hy, cr = [], [], [], [], [], []
        rc, rs, ex, ey = [], [], [], []
        for nid, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if layer in pad.layers or '*.Cu' in pad.layers:
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
        arr = (np.asarray(nids, dtype=np.int64), np.asarray(cx), np.asarray(cy),
               np.asarray(hx), np.asarray(hy), np.asarray(cr),
               np.asarray(rc), np.asarray(rs), np.asarray(ex), np.asarray(ey))
        cache[layer] = arr
    return arr


def _pt_foreign_pad_dist(pcb_data, net_id, x, y, layer):
    """Min edge distance from point (x,y) on `layer` to any pad of a DIFFERENT net.
    The pad is modelled as a rounded rect (accurate for circle/oval/roundrect,
    exact rect at corner_r=0), so a round BGA ball is not over-approximated by its
    bounding-box corner. Vectorized + windowed (see _FOREIGN_PAD_WINDOW); exact
    for any distance within the window, which is all the callers ever look at."""
    nids, cx, cy, hx, hy, cr, rc, rs, ex, ey = _foreign_pad_arrays(pcb_data, layer)
    if cx.size == 0:
        return 1e9
    R = _FOREIGN_PAD_WINDOW
    # Expand the window by each pad's own half-extent (global axes, so tilted
    # pads window by their true bbox): a large pad's EDGE can be within R even
    # when its centre is past R. Then min edge distance is exact for anything
    # <= R (any excluded pad is > R away, beyond every caller's margin).
    near = (np.abs(cx - x) <= R + ex) & (np.abs(cy - y) <= R + ey) & (nids != net_id)
    if not near.any():
        return 1e9
    fcr = cr[near]
    # Rotate the query offset into each pad's local frame (R(-rot)); identity
    # for axis-aligned pads (cos=1, sin=0), exact tilted outline otherwise.
    ddx = x - cx[near]; ddy = y - cy[near]
    frc, frs = rc[near], rs[near]
    lx = np.abs(ddx * frc + ddy * frs)
    ly = np.abs(-ddx * frs + ddy * frc)
    dx = np.maximum(lx - (hx[near] - fcr), 0.0)
    dy = np.maximum(ly - (hy[near] - fcr), 0.0)
    return float(np.min(np.hypot(dx, dy) - fcr))


def _seg_foreign_pad_dist(pcb_data, net_id, x1, y1, x2, y2, layer):
    """Min foreign-pad edge distance sampled along a (short, terminal) segment.
    Pads are modelled as rounded rects (corner_r), so round/oval/roundrect pads --
    e.g. BGA balls -- use their true outline, not the bounding-box corner that
    manufactures phantom grazes (#315). Vectorized: windows pads to the segment's
    bbox + margin, then evaluates ALL sample points against them in one matrix op."""
    nids, cx, cy, hx, hy, cr, rc, rs, ex, ey = _foreign_pad_arrays(pcb_data, layer)
    if cx.size == 0:
        return 1e9
    n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02) + 1)
    R = _FOREIGN_PAD_WINDOW
    # Pad bbox (centre +/- global half-extent, true bbox for tilted pads) must
    # reach within R of the segment bbox; a pad excluded here is > R from every
    # sample point on the segment.
    near = ((cx + ex >= min(x1, x2) - R) & (cx - ex <= max(x1, x2) + R) &
            (cy + ey >= min(y1, y2) - R) & (cy - ey <= max(y1, y2) + R) & (nids != net_id))
    if not near.any():
        return 1e9
    fcx, fcy, fhx, fhy, fcr = cx[near], cy[near], hx[near], hy[near], cr[near]
    frc, frs = rc[near], rs[near]
    t = np.linspace(0.0, 1.0, n + 1)
    sx = x1 + (x2 - x1) * t
    sy = y1 + (y2 - y1) * t
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
    return float(np.min(np.hypot(dx, dy) - fcr[None, :]))


def _foreign_seg_arrays(pcb_data, layer):
    """Cached per-layer numpy arrays (net_id, x1, y1, x2, y2, half_width) for every
    routed SEGMENT on `layer`, plus every VIA folded in as a degenerate (zero-length)
    segment of half_width = via radius. Unlike pads, copper changes as nets route, so
    the cache is keyed on the (segment, via) counts and rebuilt when they change.
    Counts alone go stale across rip-reroute (#339: a ripped net re-adds the SAME
    number of segments at new coordinates -- cynthion's refit judged a via against
    MEZZANINE5's OLD track), so the tail elements' geometry joins the signature."""
    segs, vias = pcb_data.segments, pcb_data.vias
    tail = segs[-1] if segs else None
    vtail = vias[-1] if vias else None
    sig = (len(segs), len(vias),
           (tail.start_x, tail.start_y, tail.end_x, tail.net_id) if tail is not None else None,
           (vtail.x, vtail.y, vtail.net_id) if vtail is not None else None)
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


def _seg_foreign_seg_dist(pcb_data, net_id, x1, y1, x2, y2, layer):
    """Min edge distance from a (short, terminal) segment to any OTHER-net segment or
    via on `layer` -- the segment analogue of _seg_foreign_pad_dist. Distance is from
    the terminal centreline to the foreign copper EDGE (point-to-segment distance to
    the foreign centreline minus the foreign half-width), sampled along the terminal
    and vectorized over windowed foreign segments. A negative result (centreline
    inside foreign copper) is returned as-is so the caller necks to the floor."""
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


def _seg_foreign_via_dist(pcb_data, net_id, x1, y1, x2, y2, layer):
    """Min edge distance from a segment to any OTHER-net VIA (body), exact point-to-
    segment minus via radius. The via analogue of _seg_foreign_pad_dist; a negative
    result (centreline inside the via) is returned as-is."""
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
    return float(np.min(d))


def _foreign_hole_capsules(pcb_data):
    """Cached NPTH (no-copper) drill capsules: (net_id, ax, ay, bx, by, r) numpy
    arrays, one row per pad whose drill carries no copper ring (mechanical /
    mounting holes -- np_thru_hole, or a pad with no copper layer). The pad /
    segment / via distance trio all measure to COPPER, so they never see these
    holes; but a track crossing one is a real fab short (check_drc's track-hole
    rule, issue #233), gated by the higher NPTH-to-track floor. Holes are
    through, so the distance is layer-agnostic. Round drills degenerate to a
    zero-length capsule (a=b). Rebuilt when the board's pad count changes (pads
    are static during routing, so this almost never refires)."""
    from check_drc import _pad_has_no_copper
    from kicad_parser import pad_drill_capsule
    sig = sum(len(p) for p in pcb_data.pads_by_net.values())
    cache = getattr(pcb_data, '_foreign_hole_cap_cache', None)
    if cache is None or cache[0] != sig:
        nid, ax, ay, bx, by, r = [], [], [], [], [], []
        for pad_net, pads in pcb_data.pads_by_net.items():
            for pad in pads:
                if (getattr(pad, 'drill', 0) or 0) > 0 and _pad_has_no_copper(pad):
                    (p1x, p1y), (p2x, p2y), hr = pad_drill_capsule(pad)
                    nid.append(pad_net)
                    ax.append(p1x); ay.append(p1y); bx.append(p2x); by.append(p2y)
                    r.append(hr)
        cache = (sig, (np.asarray(nid, dtype=np.int64), np.asarray(ax, dtype=float),
                       np.asarray(ay, dtype=float), np.asarray(bx, dtype=float),
                       np.asarray(by, dtype=float), np.asarray(r, dtype=float)))
        pcb_data._foreign_hole_cap_cache = cache
    return cache[1]


def _seg_foreign_hole_dist(pcb_data, net_id, x1, y1, x2, y2):
    """Min edge distance from a segment to any OTHER-net NPTH drill hole (the
    hole analogue of _seg_foreign_via_dist). Exact segment-to-capsule distance
    minus the hole radius; a negative result (segment over the hole) is returned
    as-is. Own-net holes are excluded (a track legitimately reaches its own
    mounting-hole pad). 1e9 when there are no foreign holes."""
    nid, hax, hay, hbx, hby, hr = _foreign_hole_capsules(pcb_data)
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
            if _pt_foreign_pad_dist(pcb_data, net_id, x, y, layer) < need:
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
    Returns the count necked.

    `floor` defaults to the board's fab track-width minimum (issue #176): necking
    to the grid step (0.05 mm) used to emit sub-fab-floor copper."""
    if floor is None:
        floor = _fab_track_floor(pcb_data)
    def touches(s):
        for tx, ty in term_pts:
            if (abs(s.start_x - tx) < 1e-6 and abs(s.start_y - ty) < 1e-6) or \
               (abs(s.end_x - tx) < 1e-6 and abs(s.end_y - ty) < 1e-6):
                return True
        return False
    necked = 0
    for s in segments:
        if not touches(s):
            continue
        # Nearest foreign EDGE on this layer -- pad or track/via, whichever is closer.
        d = min(_seg_foreign_pad_dist(pcb_data, net_id, s.start_x, s.start_y, s.end_x, s.end_y, s.layer),
                _seg_foreign_seg_dist(pcb_data, net_id, s.start_x, s.start_y, s.end_x, s.end_y, s.layer))
        allowed_half = d - config.clearance - 1e-4  # 1e-4: stay just inside the rule
        if allowed_half < s.width / 2.0 - 1e-9:
            new_w = max(floor, 2.0 * allowed_half)
            if new_w < s.width - 1e-9:
                s.width = round(new_w, 4)
                necked += 1
    return necked


def _neck_route_terminal_grazes(segments, path, coord, start_original, end_original,
                                pcb_data, net_id, config):
    """Run _neck_terminal_grazes for a converted multipoint edge/tap, recomputing the
    terminal points from the path endpoints + original pad positions. Called AFTER
    _apply_neckdown_widths / uniform_width so the graze-neck is authoritative: those
    passes rebuild every width from the pad-distance taper and would otherwise restore
    a grazing terminal to full/base width, undoing the neck (issue #212)."""
    if pcb_data is None or not path:
        return
    term_pts = [coord.to_float(path[0][0], path[0][1]),
                coord.to_float(path[-1][0], path[-1][1])]
    if start_original:
        term_pts.append((start_original[0], start_original[1]))
    if end_original:
        term_pts.append((end_original[0], end_original[1]))
    _neck_terminal_grazes(segments, term_pts, pcb_data, net_id, config)


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
    if _pt_foreign_pad_dist(pcb_data, net_id, fx, fy, ol) >= margin:
        return False  # grid cell already clear -> nothing to fix
    if _pt_foreign_pad_dist(pcb_data, net_id, ox, oy, ol) < margin:
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
    if _seg_foreign_pad_dist(pcb_data, net_id, ox, oy, nx, ny, ol) < margin - 1e-6:
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

    seg_rows = []
    for seg in pcb_data.segments:
        if seg.net_id == current_net_id:
            continue
        li = layer_map.get(seg.layer)
        if li is None:
            continue
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        seg_rows.append((gx1, gy1, gx2, gy2, li, seg.net_id))
    segs = np.asarray(seg_rows, dtype=np.int64) if seg_rows else np.empty((0, 6), dtype=np.int64)

    via_rows = []
    for via in pcb_data.vias:
        if via.net_id == current_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        via_rows.append((gx, gy, via.net_id))
    vias = np.asarray(via_rows, dtype=np.int64) if via_rows else np.empty((0, 3), dtype=np.int64)

    pad_rows = []
    for ref, footprint in pcb_data.footprints.items():
        for pad in footprint.pads:
            if pad.net_id == current_net_id or pad.net_id == 0:
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
    pads = np.asarray(pad_rows, dtype=np.int64) if pad_rows else np.empty((0, 6), dtype=np.int64)

    counts = rust_fn(blocked, segs, vias, pads,
                     int(expansion_grid), int(via_expansion_grid), int(num_layers))
    blockers: Dict[int, Tuple[str, int]] = {}
    for net_id, count in counts.items():
        net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"net_{net_id}"
        blockers[net_id] = (net_name, count)
    return blockers


def _diagnose_blocked_start(obstacles: 'GridObstacleMap', cells: List, label: str, print_prefix: str = "", track_margin: int = 0,
                            pcb_data: PCBData = None, config: GridRouteConfig = None, current_net_id: int = -1):
    """
    Diagnose why routing couldn't start from the given cells.

    Checks blocking status of start cells and their immediate neighbors.
    If pcb_data and config are provided, also identifies which nets are blocking.
    """
    if not cells:
        print(f"{print_prefix}  {label}: no cells to check")
        return

    # Check a sample of cells (first few)
    sample_cells = cells[:3] if len(cells) > 3 else cells

    for gx, gy, layer in sample_cells:
        # Check if the cell itself is blocked
        cell_blocked = obstacles.is_blocked(gx, gy, layer)

        # Check neighbors (8-connected)
        blocked_neighbors = 0
        total_neighbors = 0
        blocked_details = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                total_neighbors += 1
                # Check with margin if specified
                if track_margin > 0:
                    neighbor_blocked = False
                    for mx in range(-track_margin, track_margin + 1):
                        for my in range(-track_margin, track_margin + 1):
                            if obstacles.is_blocked(gx + dx + mx, gy + dy + my, layer):
                                neighbor_blocked = True
                                break
                        if neighbor_blocked:
                            break
                else:
                    neighbor_blocked = obstacles.is_blocked(gx + dx, gy + dy, layer)
                if neighbor_blocked:
                    blocked_neighbors += 1
                    blocked_details.append(f"({gx+dx},{gy+dy})")

        status = "BLOCKED" if cell_blocked else "ok"
        margin_str = f" (margin={track_margin})" if track_margin > 0 else ""
        print(f"{print_prefix}  {label} cell ({gx}, {gy}, layer={layer}): {status}, {blocked_neighbors}/{total_neighbors} neighbors blocked{margin_str}")
        # Show which specific neighbors are blocked for debugging
        if blocked_neighbors == total_neighbors and blocked_neighbors > 0:
            print(f"{print_prefix}    ALL neighbors blocked: {', '.join(blocked_details)}")

        # Identify what's blocking if pcb_data and config are provided
        if blocked_neighbors > 0 and pcb_data is not None and config is not None:
            # Collect blocked neighbor positions
            blocked_positions = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    if track_margin > 0:
                        for mx in range(-track_margin, track_margin + 1):
                            for my in range(-track_margin, track_margin + 1):
                                if obstacles.is_blocked(gx + dx + mx, gy + dy + my, layer):
                                    blocked_positions.append((gx + dx + mx, gy + dy + my, layer))
                    else:
                        if obstacles.is_blocked(gx + dx, gy + dy, layer):
                            blocked_positions.append((gx + dx, gy + dy, layer))

            if blocked_positions:
                blockers = _identify_blocking_obstacles(blocked_positions, pcb_data, config, current_net_id)
                if blockers:
                    # Sort by count descending
                    sorted_blockers = sorted(blockers.items(), key=lambda x: x[1][1], reverse=True)
                    blocker_strs = [f"{name}({count})" for net_id, (name, count) in sorted_blockers[:5]]
                    print(f"{print_prefix}    Blocking obstacles: {', '.join(blocker_strs)}")


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


def _probe_route_with_frontier(
    router: 'GridRouter',
    obstacles: 'GridObstacleMap',
    forward_sources: List,
    forward_targets: List,
    config: 'GridRouteConfig',
    print_prefix: str = "",
    direction_labels: Tuple[str, str] = ("forward", "backward"),
    track_margin: int = 0,
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
    track_margin: int = 0,
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
        track_margin: Extra margin in grid cells for wide tracks (power nets)
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

    # Track iterations per direction (first/second maps to forward/backward based on labels)
    first_total_iters = 0
    second_total_iters = 0

    # Probe forward direction
    path, iterations, blocked_cells = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, probe_iterations, track_margin=track_margin, via_exclusion_radius=_ver)
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
            print(f"{print_prefix}{first_label} stuck ({first_probe_iters} < {probe_iterations}) [single-direction bus mode]")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            fwd_iters, bwd_iters = get_fwd_bwd_iters()
            return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

        # Forward probe reached max - do full search
        print(f"{print_prefix}Probe: {first_label}={first_probe_iters} iters [single-direction bus mode], trying full iterations...")
        path, full_iters, full_blocked = router.route_with_frontier(
            obstacles, forward_sources, forward_targets, config.max_iterations, track_margin=track_margin, via_exclusion_radius=_ver)
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
        obstacles, forward_targets, forward_sources, probe_iterations, track_margin=track_margin, via_exclusion_radius=_ver)
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
            print(f"{print_prefix}Both directions stuck ({first_label}={first_probe_iters}, {second_label}={second_probe_iters} < {probe_iterations})")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            _diagnose_blocked_start(obstacles, forward_targets, second_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
        elif not first_reached_max:
            print(f"{print_prefix}{first_label} stuck ({first_probe_iters} < {probe_iterations}), {second_label}={second_probe_iters}")
            _diagnose_blocked_start(obstacles, forward_sources, first_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
        else:
            print(f"{print_prefix}{second_label} stuck ({second_probe_iters} < {probe_iterations}), {first_label}={first_probe_iters}")
            _diagnose_blocked_start(obstacles, forward_targets, second_label, print_prefix, track_margin,
                                    pcb_data=pcb_data, config=config, current_net_id=current_net_id)
            # Print visual obstacle map around the stuck target
            if forward_targets and config.debug_lines:
                tgt = forward_targets[0]
                _print_obstacle_map(obstacles, tgt[0], tgt[1], tgt[2], radius=15, print_prefix=print_prefix)
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Both probes reached max iterations - do full search on forward direction
    print(f"{print_prefix}Probe: {first_label}={first_probe_iters}, {second_label}={second_probe_iters} iters, trying {first_label} with full iterations...")

    path, full_iters, full_blocked = router.route_with_frontier(
        obstacles, forward_sources, forward_targets, config.max_iterations, track_margin=track_margin, via_exclusion_radius=_ver)
    first_total_iters += full_iters
    total_iterations += full_iters

    if path is not None:
        forward_blocked = []
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters

    # Forward failed, try backward
    print(f"{print_prefix}No route found after {full_iters} iterations ({first_label}), trying {second_label}...")
    forward_blocked = full_blocked

    path, backward_full_iters, backward_full_blocked = router.route_with_frontier(
        obstacles, forward_targets, forward_sources, config.max_iterations, track_margin=track_margin, via_exclusion_radius=_ver)
    second_total_iters += backward_full_iters
    total_iterations += backward_full_iters

    if path is not None:
        backward_blocked = []
        fwd_iters, bwd_iters = get_fwd_bwd_iters()
        return path, total_iterations, forward_blocked, backward_blocked, True, fwd_iters, bwd_iters

    backward_blocked = backward_full_blocked
    fwd_iters, bwd_iters = get_fwd_bwd_iters()
    return None, total_iterations, forward_blocked, backward_blocked, False, fwd_iters, bwd_iters


def route_net_with_obstacles(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                              obstacles: GridObstacleMap,
                              attraction_path: Optional[List[Tuple[int, int, int]]] = None,
                              reverse_direction: bool = False) -> Optional[dict]:
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
    """
    # Find endpoints (segments or pads)
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

    coord = GridCoord(config.grid_step)
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
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there even if blocked by
    # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
    # NOTE: Must pass layer to only allow override on the specific layer of the endpoint
    for gx, gy, layer in sources_grid + targets_grid:
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

    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=prox_h_cost,
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost,
                        attraction_radius=bus_attraction_radius_grid,
                        attraction_bonus=bus_attraction_bonus)

    # Set attraction path for bus routing (if provided)
    if attraction_path:
        router.set_attraction_path(attraction_path)
        if config.verbose:
            layers_in_path = set(p[2] for p in attraction_path)
            print(f"    Bus attraction: {len(attraction_path)} path points, layers={layers_in_path}, radius={bus_attraction_radius_grid} grid, bonus={bus_attraction_bonus}")

    # Calculate track margin for wide power tracks
    # Use ceiling + 1 to account for grid quantization and diagonal track approaches
    # Compare against layer-specific width (not base track_width) to handle impedance routing
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    layer_track_width = config.get_track_width(config.layers[0])
    extra_half_width = (net_track_width - layer_track_width) / 2
    track_margin = (int(math.ceil(extra_half_width / config.grid_step)) + 1) if extra_half_width > 0 else 0

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
     fwd_iters, bwd_iters, necked_down, uniform_width, unblock_vias) = _route_with_via_unblock(
        router, obstacles, config, forward_sources, forward_targets, track_margin,
        pcb_data, net_id, print_prefix="", direction_labels=direction_labels,
        single_direction=use_single_direction
    )

    # Adjust reversed_path based on start direction
    if start_backwards and path is not None:
        reversed_path = not reversed_path

    if path is None:
        dir_msg = "single direction" if use_single_direction else "both directions"
        print(f"No route found after {total_iterations} iterations ({dir_msg})")
        return {
            'failed': True,
            'iterations': total_iterations,
            'blocked_cells_forward': forward_blocked,
            'blocked_cells_backward': backward_blocked,
            'iterations_forward': fwd_iters,
            'iterations_backward': bwd_iters,
        }

    print(f"Route found in {total_iterations} iterations, path length: {len(path)}")

    # Collect and print stats if enabled
    if config.collect_stats:
        # Re-run with stats collection on the same direction that succeeded
        # Use the actual source/target that worked
        if reversed_path:
            stats_sources, stats_targets = forward_targets, forward_sources
        else:
            stats_sources, stats_targets = forward_sources, forward_targets
        _, _, stats = router.route_multi(
            obstacles, stats_sources, stats_targets, config.max_iterations, track_margin=track_margin)
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
    term_pts = [coord.to_float(path[0][0], path[0][1]), coord.to_float(path[-1][0], path[-1][1])]
    if start_original:
        term_pts.append((start_original[0], start_original[1]))
    if end_original:
        term_pts.append((end_original[0], end_original[1]))
    _neck_terminal_grazes(new_segments, term_pts, pcb_data, net_id, config)

    # Fab-floor via dropped inside a boxed source/target pad to unblock this route
    # (issue #189); connects the inner-layer path end to the pad by copper overlap.
    new_vias = list(new_vias) + unblock_vias

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


def _track_margin_for_width(width, layer_width, grid_step):
    """Extra grid-cell margin the A* needs for a track of `width` over the layer's
    base width (mirrors the inline computation at the route_* entry points)."""
    extra_half = (width - layer_width) / 2
    return (int(math.ceil(extra_half / grid_step)) + 1) if extra_half > 0 else 0


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


def _edge_span_mm(sources, targets, grid_step):
    """Min source->target span (mm): a cheap 'is this a short escape vs a trunk' proxy."""
    best = float('inf')
    for s in sources:
        for t in targets:
            best = min(best, math.hypot(s[0] - t[0], s[1] - t[1]))
    return best * grid_step


def _place_shrunk_via_in_pad(pad_obj, obstacles, config, pcb_data, net_id, coord, layer_names):
    """Issue #189: drop a DRC-legal fab-floor via INSIDE a boxed-in SMD pad so a
    stuck A* can reach the pad on an inner layer. Returns (Via, (gx, gy),
    pad_layer_idx) or None.

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
    tap_res = None
    for vd, dr in via_pairs:
        # try_default=False: skip the default-parameter pass and go straight to
        # the fine (grid 0.05 / capped clearance) placement. Each pass builds via
        # + routing obstacle maps (the profiled bottleneck, ~0.5s each in Rust);
        # the default pass is redundant here -- fine uses min(grid)/min(clearance)
        # so it is strictly >= permissive, and a via-in-pad needs no via->pad
        # trace, so the fine pass never fails where the default would succeed.
        tap_res = tap_pad_with_escalation(
            pad_obj, pad_layer, net_id, pcb_data,
            replace(config, via_size=vd, via_drill=dr, board_edge_clearance=0.0),
            max_search_radius=0.0, via_size=vd, via_drill=dr,
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
            break
    if tap_res is None or not tap_res.success or tap_res.via is None:
        # Don't memoise a failure caused (possibly) by TRANSIENT in-flight
        # copper: once that window closes the pad may be genuinely tappable.
        if not (inflight_vias or inflight_segments):
            cache.add(key)
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
    sizes = getattr(pcb_data, '_unblock_via_sizes', None)
    if sizes is None:
        sizes = {}
        pcb_data._unblock_via_sizes = sizes
    sizes[(vgx, vgy)] = (v['size'], v['drill'])
    return via, (vgx, vgy), layer_names.index(pad_layer)


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
    return [p for _d, p in sorted(found.values(), key=lambda t: t[0])]


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
        return res + ([],)
    layer_names = config.layers
    if len(layer_names) < 2:
        return res + ([],)
    fwd_i, bwd_i = res[5], res[6]
    lim = config.max_probe_iterations
    coord = GridCoord(config.grid_step)

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
            via, (vgx, vgy), pli = r
            _register_unblock_via(obstacles, vgx, vgy, layer_names)
            new_targets = list(targets) + [(vgx, vgy, li) for li in range(len(layer_names))]
            placed.append((via, vgx, vgy, pli, pad))
    # forward probe (from sources) exhausted -> the SOURCE pad is boxed
    if fwd_i and fwd_i < lim:
        pad = _net_pad_near(pcb_data, net_id, sources, coord)
        r = (_place_shrunk_via_in_pad(pad, obstacles, config, pcb_data, net_id, coord, layer_names)
             if pad is not None else None)
        if r is not None:
            via, (vgx, vgy), pli = r
            _register_unblock_via(obstacles, vgx, vgy, layer_names)
            new_sources = list(sources) + [(vgx, vgy, li) for li in range(len(layer_names))]
            placed.append((via, vgx, vgy, pli, pad))
    if not placed:
        return res + ([],)

    # Cap the retry's full A* at the same budget as the stuck threshold we trigger
    # on (max_probe_iterations): if dropping the via opened the pad, the inner
    # layer beside it is clear and the route is found within the probe budget;
    # if it isn't, grinding to the full max_iterations (1e6 at grid 0.05) just to
    # fail again is wasted -- fail fast and report the pad honestly.
    if _dbg:
        for (via, vgx, vgy, pli, pad) in placed:
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
                via2, (vgx2, vgy2), pli2 = r2
                _register_unblock_via(obstacles, vgx2, vgy2, layer_names)
                ext = [(vgx2, vgy2, li) for li in range(len(layer_names))]
                if is_source:
                    new_sources = list(side_cells) + ext
                else:
                    new_targets = list(side_cells) + ext
                placed.append((via2, vgx2, vgy2, pli2, pad2))
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
        for (_v, _gx, _gy, _pli, pad) in placed:
            cache.add((net_id, round(pad.global_x, 3), round(pad.global_y, 3)))
        return res + ([],)  # unblock didn't help; report the original failure
    # Keep only the vias the retry actually used: the new path must terminate on
    # the via cell at a NON-pad layer (it reached the pad through the via). Drop
    # any the route didn't need, so no floating copper is added.
    p2 = res2[0]
    ends = (p2[0], p2[-1])
    used = [via for (via, vgx, vgy, pli, pad) in placed
            if any(e[0] == vgx and e[1] == vgy and e[2] != pli for e in ends)]
    if used:
        print(f"{print_prefix}{GREEN}Via-in-pad unblock: dropped {len(used)} fab-floor "
              f"via(s) to reach a boxed endpoint{RESET}")
    return res2 + (used,)


def _route_main_connection(router, obstacles, config, sources, targets, track_margin,
                           pcb_data, net_id, print_prefix="",
                           direction_labels=("forward", "backward"), single_direction=False,
                           waypoints=None):
    """Route sources->targets; wide routes that fail retry narrow (issue #72/#180).

    Same return shape as _route_connection_at_margin plus a trailing
    (necked_down, uniform_width):
      - necked_down=True (long trunk): the wide route failed and it re-routed at the
        layer width; the caller necks down the segments near the pad.
      - uniform_width=W (short edge, necked_down=False): a short power edge routed
        at width W (full -> /2 -> ... -> fab floor); the caller sets EVERY segment to
        W so the trace -- and the obstacle map (which reads seg.width) and the written
        output -- is genuinely that width, not the power width.
      - both None/False: full power width, no rewidthing.
    """
    result = _route_connection_at_margin(
        router, obstacles, config, sources, targets, track_margin,
        pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
    if result[0] is not None or track_margin == 0 or not config.power_tap_neckdown:
        return result + (False, None)

    net_w = config.get_net_track_width(net_id, config.layers[0])
    layer_w = config.get_track_width(config.layers[0])

    if _edge_span_mm(sources, targets, config.grid_step) <= SHORT_POWER_EDGE_MM:
        # Short edge: step the width down, widest-that-fits wins; segments use it.
        total_iters = result[1]
        for w in _power_width_ladder(net_w, layer_w):
            tm = _track_margin_for_width(w, layer_w, config.grid_step)
            r = _route_connection_at_margin(
                router, obstacles, config, sources, targets, tm,
                pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
            total_iters += r[1]
            if r[0] is not None:
                print(f"{print_prefix}{YELLOW}Wide power route blocked - routed short edge at "
                      f"{w:.4f}mm (down from {net_w:.4f}){RESET}")
                return (r[0], total_iters) + r[2:] + (False, w)
        return (result[0], total_iters) + result[2:] + (False, None)

    # Long trunk: keep the existing single wide->base retry + neck-down.
    print(f"{print_prefix}{YELLOW}Wide route blocked - retrying at default track width (neck-down){RESET}")
    retry = _route_connection_at_margin(
        router, obstacles, config, sources, targets, 0,
        pcb_data, net_id, print_prefix, direction_labels, single_direction, waypoints)
    if retry[0] is None:
        # Keep the WIDE attempt's frontier for rip-up analysis: blockers found
        # by the narrow frontier only help a narrow route, but ripped nets
        # re-route at their own wide width and can fail entirely
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


def route_net_with_visualization(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                                  obstacles: GridObstacleMap, vis_callback) -> Optional[dict]:
    """Route a single net with real-time visualization.

    Uses VisualRouter for incremental stepping with visualization callbacks.

    Args:
        pcb_data: PCB data
        net_id: Net ID to route
        config: Routing configuration
        obstacles: Pre-built obstacle map
        vis_callback: Visualization callback (implements VisualizationCallback protocol)

    Returns:
        Routing result dict, or None on failure
    """
    if VisualRouter is None:
        print("  VisualRouter not available, falling back to standard routing")
        return route_net_with_obstacles(pcb_data, net_id, config, obstacles)

    # Find endpoints (segments or pads)
    sources, targets, error = get_net_endpoints(pcb_data, net_id, config)
    if error:
        print(f"  {error}")
        return None

    if not sources or not targets:
        print(f"  No valid source/target endpoints found")
        return None

    coord = GridCoord(config.grid_step)
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
    allow_radius = 10
    for gx, gy, _ in sources_grid + targets_grid:
        for dx in range(-allow_radius, allow_radius + 1):
            for dy in range(-allow_radius, allow_radius + 1):
                obstacles.add_allowed_cell(gx + dx, gy + dy)

    # Mark exact source/target cells so routing can start/end there
    for gx, gy, layer in sources_grid + targets_grid:
        obstacles.add_source_target_cell(gx, gy, layer)

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

    # Calculate vertical attraction parameters
    attraction_radius_grid = coord.to_grid_dist(config.vertical_attraction_radius) if config.vertical_attraction_radius > 0 else 0
    attraction_bonus = config.cell_cost(config.vertical_attraction_cost) if config.vertical_attraction_cost > 0 else 0

    # Determine direction order (always deterministic)
    if config.direction_order in ("backwards", "backward"):
        start_backwards = True
    else:
        start_backwards = False

    if start_backwards:
        first_sources, first_targets = targets_grid, sources_grid
        second_sources, second_targets = sources_grid, targets_grid
        first_label, second_label = "backward", "forward"
    else:
        first_sources, first_targets = sources_grid, targets_grid
        second_sources, second_targets = targets_grid, sources_grid
        first_label, second_label = "forward", "backward"

    # Create visual router
    router = VisualRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
                          turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                          vertical_attraction_radius=attraction_radius_grid,
                          vertical_attraction_bonus=attraction_bonus,
                          layer_costs=config.get_layer_costs(),
                          proximity_heuristic_cost=prox_h_cost)

    # Try first direction with visualization
    if config.verbose:
        print(f"    VisualRouter sources: {first_sources[:3]}{'...' if len(first_sources) > 3 else ''}")
        print(f"    VisualRouter targets: {first_targets[:3]}{'...' if len(first_targets) > 3 else ''}")
    router.init(first_sources, first_targets, config.max_iterations)
    path = None
    total_iterations = 0
    direction_used = first_label

    while not router.is_done():
        # Check if we should pause
        while vis_callback.should_pause():
            # Keep handling events while paused
            snapshot = router.step(obstacles, 0)  # 0 iterations, just get current state
            if not vis_callback.on_route_step(snapshot):
                return None  # User quit

        # Get iterations to run
        iters = vis_callback.get_iterations_per_step()
        snapshot = router.step(obstacles, iters)

        # Update visualization
        if not vis_callback.on_route_step(snapshot):
            return None  # User quit

        if snapshot.found:
            path = snapshot.path
            total_iterations = snapshot.iteration
            break

    if path is None:
        total_iterations = router.step(obstacles, 0).iteration
        print(f"No route found after {total_iterations} iterations ({first_label}), trying {second_label}...")

        # Try second direction
        router = VisualRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
                          turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                          vertical_attraction_radius=attraction_radius_grid,
                          vertical_attraction_bonus=attraction_bonus,
                          layer_costs=config.get_layer_costs(),
                          proximity_heuristic_cost=prox_h_cost)
        router.init(second_sources, second_targets, config.max_iterations)
        direction_used = second_label

        while not router.is_done():
            while vis_callback.should_pause():
                snapshot = router.step(obstacles, 0)
                if not vis_callback.on_route_step(snapshot):
                    return None

            iters = vis_callback.get_iterations_per_step()
            snapshot = router.step(obstacles, iters)

            if not vis_callback.on_route_step(snapshot):
                return None

            if snapshot.found:
                path = snapshot.path
                total_iterations += snapshot.iteration
                break

        if path is None:
            total_iterations += router.step(obstacles, 0).iteration

    if path is None:
        print(f"No route found after {total_iterations} iterations (both directions)")
        return {'failed': True, 'iterations': total_iterations, 'direction': 'both'}

    print(f"Route found in {total_iterations} iterations ({direction_used}), path length: {len(path)}")

    # Print debug stats if verbose
    if config.verbose:
        stats = router.get_stats()
        print(f"    VisualRouter stats: expanded={stats.get('cells_expanded', 0)}, pushed={stats.get('cells_pushed', 0)}, duplicates={stats.get('duplicate_skips', 0)}, closed={stats.get('closed_size', 0)}")

    # Swap sources/targets if we used second direction
    reversed_path = (direction_used == second_label)
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

    return {
        'new_segments': new_segments,
        'new_vias': new_vias,
        'iterations': total_iterations,
        'path_length': len(path),
        'path': path,
        'direction': direction_used,
    }


def route_multipoint_main(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    obstacles: 'GridObstacleMap',
    pad_info: List[Tuple]
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
    pad_components = get_copper_connected_terminal_groups(pcb_data, net_id, pad_info)
    num_components = len(set(pad_components.values()))
    if num_components < len(pad_info):
        print(f"  Existing copper joins {len(pad_info)} terminals into "
              f"{num_components} group(s)")
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
        }

    # Sort MST edges by length (longest first)
    mst_edges = sorted(mst_edges, key=lambda e: -e[2])

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

    # Route farthest pair with probe routing (same as single-ended)
    router = GridRouter(via_cost=config.via_cost_units(), h_weight=config.heuristic_weight,
                        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=prox_h_cost,
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost)

    # Calculate track margin for wide power tracks
    # Use ceiling + 1 to account for grid quantization and diagonal track approaches
    # Compare against layer-specific width (not base track_width) to handle impedance routing
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    layer_track_width = config.get_track_width(config.layers[0])
    extra_half_width = (net_track_width - layer_track_width) / 2
    track_margin = (int(math.ceil(extra_half_width / config.grid_step)) + 1) if extra_half_width > 0 else 0

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

        if pad_a_obj and hasattr(pad_a_obj, 'layers') and '*.Cu' in pad_a_obj.layers:
            sources = [(pad_a[0], pad_a[1], layer_idx) for layer_idx in range(len(layer_names))]
        else:
            sources = [(pad_a[0], pad_a[1], pad_a[2])]  # (gx, gy, layer_idx)

        if pad_b_obj and hasattr(pad_b_obj, 'layers') and '*.Cu' in pad_b_obj.layers:
            targets = [(pad_b[0], pad_b[1], layer_idx) for layer_idx in range(len(layer_names))]
        else:
            targets = [(pad_b[0], pad_b[1], pad_b[2])]

        # Mark source/target cells (same-net pad cells; safe to accumulate)
        for gx, gy, layer in sources + targets:
            obstacles.add_source_target_cell(gx, gy, layer)

        # Use probe routing helper, steered through this edge's bucket of
        # corridor waypoints (tap edges follow their own buckets later).
        (path, total_iterations, forward_blocked, backward_blocked, reversed_path,
         fwd_iters, bwd_iters, necked_down, uniform_width, main_unblock_vias) = _route_with_via_unblock(
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
            _comps, _copper, _ = get_terminal_component_info(
                pcb_data, net_id, pad_info)
            _best = max(range(len(pad_info)),
                        key=lambda i: _copper.get(_comps.get(i), 0))
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
                    'waypoint_buckets': waypoint_buckets,
                    'phase1_exhausted': True,
                    'tap_edges_routed': 0,
                    'tap_edges_failed': 0,
                }
        print(f"  Failed to route a main edge after {cumulative_iterations} iterations "
              f"({max_main_attempts} edge(s) tried)")
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

    # Convert path to segments/vias
    segments, vias = _path_to_segments_vias(
        path, coord, layer_names, net_id, config,
        (pad_a[3], pad_a[4], layer_names[pad_a[2]]),  # start_original
        (pad_b[3], pad_b[4], layer_names[pad_b[2]]),  # end_original
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
    _neck_route_terminal_grazes(segments, path, coord,
                                (pad_a[3], pad_a[4]), (pad_b[3], pad_b[4]),
                                pcb_data, net_id, config)
    # Fab-floor via dropped inside a boxed main-edge pad to unblock it (#189).
    vias = list(vias) + main_unblock_vias

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
            obstacles.remove_blocked_vias_batch(np.array(ring_cells, dtype=np.int32))


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
                    # Ref-counted raw add: the wrapper removes these on exit so
                    # they can't leak into a persistent working map (#309).
                    if _ring_cells is not None:
                        _ring_cells.append((vgx + ex, vgy + ey))

    for _v in all_vias:
        _register_inprogress_via(_v)

    # Phase-1-exhausted fallback (#348): the synthetic main result carries NO
    # new copper, so seed the tap sources from the net's EXISTING copper that
    # belongs to the routed base component(s) -- the island the terminals in
    # routed_indices sit on. Sources restricted to the BASE component only:
    # launching from an unconnected island would join the target to that
    # island and mark it routed while the base stays split (#189).
    source_extra_segments: List[Segment] = []
    source_extra_vias: List = []
    if main_result.get('phase1_exhausted'):
        from connectivity import get_terminal_component_info
        _comps, _copper, _segs_by_comp = get_terminal_component_info(
            pcb_data, net_id, pad_info)
        _base_comps = {_comps.get(_i) for _i in routed_indices}
        _base_ends = []
        for _c in _base_comps:
            for _s in _segs_by_comp.get(_c, []):
                # SOURCE-ONLY list: never into all_segments, whose final value
                # becomes the result's new_segments (re-emitting existing
                # copper would duplicate it in the output).
                source_extra_segments.append(_s)
                _base_ends.append((_s.start_x, _s.start_y))
                _base_ends.append((_s.end_x, _s.end_y))
        _net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
        # Existing vias touching the included base copper become tap points
        # too (no in-progress ring: pcb_data vias already govern same-net via
        # spacing in the obstacle build).
        for _v in _net_vias:
            _vr = (getattr(_v, 'size', 0.5) or 0.5) / 2 + 0.02
            if any((abs(_v.x - _ex) <= _vr and abs(_v.y - _ey) <= _vr)
                   for _ex, _ey in _base_ends):
                source_extra_vias.append(_v)
        if source_extra_segments or source_extra_vias:
            print(f"  Seeding tap sources from the base island's existing "
                  f"copper: {len(source_extra_segments)} segment(s), "
                  f"{len(source_extra_vias)} via(s)")

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
                        turn_cost=config.turn_cost, via_proximity_cost=int(config.via_proximity_cost),
                        vertical_attraction_radius=attraction_radius_grid,
                        vertical_attraction_bonus=attraction_bonus,
                        layer_costs=config.get_layer_costs(),
                        proximity_heuristic_cost=0,  # Set per-route below
                        layer_direction_preferences=config.get_layer_direction_preferences(),
                        direction_preference_cost=config.direction_preference_cost)

    # Calculate track margin for wide power tracks
    # Use ceiling + 1 to account for grid quantization and diagonal track approaches
    # Compare against layer-specific width (not base track_width) to handle impedance routing
    net_track_width = config.get_net_track_width(net_id, config.layers[0])
    layer_track_width = config.get_track_width(config.layers[0])
    extra_half_width = (net_track_width - layer_track_width) / 2
    track_margin = (int(math.ceil(extra_half_width / config.grid_step)) + 1) if extra_half_width > 0 else 0

    total_iterations = 0

    # Route remaining MST edges in order (longest first)
    # Each edge connects a routed pad to an unrouted pad
    edges_routed = 0
    failed_edges = set()  # Track edges that failed to route
    failed_edge_blocking = {}  # Track blocking info for failed edges: edge_key -> (blocked_cells, tgt_xy)
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
            all_segments + source_extra_segments, coord, layer_names,
            vias=all_vias + source_extra_vias)

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

        # Add source pad position as a valid source (on all layers for through-hole)
        if hasattr(src_pad_obj, 'layers') and '*.Cu' in src_pad_obj.layers:
            # Through-hole pad - can connect on any copper layer
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

        if not sources:
            print(f"      ERROR: No sources available for routing")
            continue

        # For through-hole pads, create targets on ALL layers (router can reach any layer)
        tgt_gx, tgt_gy = tgt_pad[0], tgt_pad[1]
        tgt_pad_obj = tgt_pad[5]
        if hasattr(tgt_pad_obj, 'layers') and '*.Cu' in tgt_pad_obj.layers:
            # Through-hole pad - can connect on any copper layer
            targets = [(tgt_gx, tgt_gy, layer_idx) for layer_idx in range(len(layer_names))]
        else:
            # SMD pad or specific layer - use the layer from pad_info
            targets = [(tgt_gx, tgt_gy, tgt_pad[2])]

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

        (path, tap_iterations, forward_blocked, backward_blocked, reversed_tap_path,
         _, _, necked_down, uniform_width, unblock_vias) = _route_with_via_unblock(
            router, obstacles, config, sources, targets, track_margin,
            pcb_data, net_id, print_prefix="      ", direction_labels=("forward", "backward"),
            waypoints=waypoint_buckets.get(frozenset((src_idx, tgt_idx)), [])
        )

        # If path was found in reverse direction, reverse it so it goes sources -> targets
        if path is not None and reversed_tap_path:
            path = list(reversed(path))

        # Combine blocked cells from both directions for rip-up analysis
        blocked_cells = forward_blocked + backward_blocked

        tap_elapsed = time.time() - tap_start_time
        total_iterations += tap_iterations

        if path is None:
            print(f"      {YELLOW}Failed to route MST edge after {tap_iterations} iterations ({tap_elapsed:.2f}s){RESET}")
            edge_key = (min(src_idx, tgt_idx), max(src_idx, tgt_idx))
            failed_edges.add(edge_key)
            # Store blocking info for potential rip-up analysis
            if blocked_cells:
                failed_edge_blocking[edge_key] = (blocked_cells, (tgt_x, tgt_y))
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
        segments, vias = _path_to_segments_vias(
            path, coord, layer_names, net_id, config,
            (tap_x, tap_y, tap_layer),  # start_original (actual tap point used)
            (tgt_x, tgt_y, path_end_layer),  # end_original (target pad on actual reached layer)
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
        _neck_route_terminal_grazes(segments, path, coord,
                                    (tap_x, tap_y), (tgt_x, tgt_y),
                                    pcb_data, net_id, config)
        # Any fab-floor via dropped INSIDE the boxed target pad to unblock this
        # edge (issue #189) -- it connects the inner-layer path end to the pad by
        # copper overlap, no extra trace needed.
        vias = list(vias) + unblock_vias
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
        if abs(orig_x - first_grid_x) > 0.001 or abs(orig_y - first_grid_y) > 0.001:
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
        if abs(orig_x - last_grid_x) > 0.001 or abs(orig_y - last_grid_y) > 0.001:
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


def _segment_fits_wide(seg, obstacles, coord: GridCoord, layer_idx: int, margin: int) -> bool:
    """True if every grid cell along the segment clears the wide-track margin."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    for gx, gy in walk_line(gx1, gy1, gx2, gy2):
        if obstacles.is_blocked_with_margin(gx, gy, layer_idx, margin):
            return False
    return True


def _flip_segments(segments):
    """Reverse a connected segment run end-to-end (order and direction)."""
    return [Segment(start_x=s.end_x, start_y=s.end_y, end_x=s.start_x, end_y=s.start_y,
                    width=s.width, layer=s.layer, net_id=s.net_id)
            for s in reversed(segments)]


def _neck_pass(segments, config: GridRouteConfig, obstacles, coord: GridCoord,
               layer_map: Dict[str, int], track_margin: int):
    """Narrow the last neckdown_length mm of the run (the pad is at the list
    END); beyond that, keep the wide width only where the wide clearance
    fits. Never re-widens an already-narrow segment (so a second pass from
    the other end preserves the first pass's neck)."""
    def fits(s):
        return _segment_fits_wide(s, obstacles, coord, layer_map.get(s.layer, 0), track_margin)

    out = []  # built in reverse (pad-first)
    cum = 0.0
    for seg in reversed(segments):
        narrow_w = config.get_track_width(seg.layer)
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
                           track_margin: int, neck_start: bool = False):
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
    out = _neck_pass(segments, config, obstacles, coord, layer_map, track_margin)
    if neck_start:
        out = _flip_segments(_neck_pass(_flip_segments(out), config, obstacles,
                                        coord, layer_map, track_margin))
    wide_flags = [s.width > config.get_track_width(s.layer) for s in out]

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
                out[k].width = config.get_track_width(out[k].layer)
                wide_flags[k] = False
        i = j

    if config.neckdown_taper_length <= 0:
        return out

    # Stepped taper wherever a wide segment meets a narrow one on the same
    # layer: carve the wide segment's adjoining end into width steps
    TAPER_STEPS = 4

    def _taper_pieces(seg, narrow_end: str):
        """Split seg into [body + taper steps]; narrow_end is 'start' or 'end'."""
        narrow_w = config.get_track_width(seg.layer)
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
