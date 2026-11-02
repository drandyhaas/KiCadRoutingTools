"""Flow-aware zone fill model (validator parity with KiCad's filler).

KiCad fills a zone by subtracting every foreign object's clearance from the
outline, enforcing min_thickness (a morphological opening: copper thinner
than the minimum evaporates), and PRUNING isolated islands -- fill fragments
not contiguously connected to the zone's anchored copper are deleted. Our
old zone-credit validator was POINTWISE ("is there room for copper at this
spot?"), which credits exactly those pruned islands: a dogbone via deep in a
BGA field always has a clear ring around its own barrel, but no
min_thickness-wide channel connects that ring to the plane (ottercast: 7
balls graded plane-connected by us, open at fab by KiCad -- and step8's
repair, trusting the same validator, skipped all of them).

This model answers the flow question: rasterize the zone at cell =
min_thickness/2, block cells within the clearance of foreign copper (and
within the hole clearance of a foreign drill, #1608) and cells inside
copperpour-banned keep-out rule areas (#477), take KiCad's min-thickness
opening -- the deflated fill sampled exactly at points half a cell apart,
re-inflated -- and label its connected components (scipy when available; a
vectorized run union-find otherwise). Two items are fill-connected iff they
touch the same component.

Perf contract (review requirement: "not much slower / can use more
memory"): built LAZILY per (zone net, layer) only when zone credit is
actually consulted, cached on the PCBData object; a build is numpy-
vectorized over the zone bbox (a full-board plane at 0.05mm cell is ~2M
uint8 cells, a few MB); queries after the build are O(1) array lookups --
cheaper than the old per-query bucket scan.
"""
from __future__ import annotations

import math
import os

from typing import Optional

import numpy as np

import routing_defaults as defaults
from kicad_parser import pad_drill_capsule

try:
    from scipy import ndimage as _ndi
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

_CACHE_ATTR = '_plane_fill_models'

# Board-level per-net class clearance map ({net_id: mm}), published by
# whichever consumer resolved it (route_planes, the repair passes) and read by
# EVERY model built for that board -- see set_board_net_clearances. It is a
# board property rather than a constructor argument on purpose: models are
# CACHED and shared across consumers (_CACHE_ATTR), and two of the six build
# sites (check_connected's zone credit, pcb_modification's fill anchors) carry
# no config at all, so a per-call argument would hand one consumer a model
# stamped at a different clearance than the next consumer assumes.
_NC_ATTR = '_plane_fill_net_clearances'

# Zone-identity lookup: models built via get_zone_model/get_fill_models are
# ALSO registered here, so consumers that only have the zone object (the
# ~20 removal-pass call sites in pcb_modification that never carried
# pcb_data) still get component-aware credit once any primary consumer
# (grader, oracle, skip gate, repair) has built the model for that board.
# Bounded: cleared when it grows past 64 zones (a board has ~10).
_MODELS_BY_ZONE_ID = {}


def _label_components(free, eight=False):
    """4-connected (8-connected with `eight`) component labeling of a boolean
    grid, scipy-free. Returns (labels int32 array, n_components) matching
    ndimage.label semantics (labels 1..n, 0 = background): the same
    components, though not necessarily the same label numbers.

    Vectorized over runs: every maximal free run along y of each x-slice is
    a node, runs of neighbouring slices that overlap (diagonally too with
    `eight`) are edges, and a hook-and-shortcut union-find joins them in a
    few whole-array rounds rather than one Python step per run."""
    nx, ny = free.shape
    reach = 1 if eight else 0
    pad = np.zeros((nx, ny + 2), dtype=np.int8)    # slice i = row
    pad[:, 1:-1] = free
    d = np.diff(pad, axis=1)
    sr, sc = np.nonzero(d == 1)                    # run starts, row-major
    er, ec = np.nonzero(d == -1)                   # run ends (exclusive)
    k = sr.size
    if k == 0:
        return np.zeros((nx, ny), dtype=np.int32), 0
    W = ny + 3                                     # row stride of the keys
    skey = sr.astype(np.int64) * W + sc
    ekey = er.astype(np.int64) * W + ec
    # Run b (row r+1) touches the runs a of row r with
    # e_a > s_b - reach and s_a < e_b + reach: one index range per b.
    b_idx = np.nonzero(sr > 0)[0]
    base_key = (sr[b_idx].astype(np.int64) - 1) * W
    lo = np.searchsorted(ekey, base_key + sc[b_idx] - reach, side='right')
    hi = np.searchsorted(skey, base_key + ec[b_idx] + reach, side='left')
    cnt = np.maximum(hi - lo, 0)
    parent = np.arange(k, dtype=np.int64)
    if cnt.sum():
        u = np.repeat(b_idx, cnt)
        first = np.repeat(lo, cnt)
        off = np.arange(u.size) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        v = first + off
        while True:
            pu, pv = parent[u], parent[v]
            m = pu != pv
            if not m.any():
                break
            # hook each root onto the smallest root it touches, then
            # shortcut until every run points straight at its root
            np.minimum.at(parent, np.maximum(pu[m], pv[m]),
                          np.minimum(pu[m], pv[m]))
            while True:
                pp = parent[parent]
                if np.array_equal(pp, parent):
                    break
                parent = pp
    roots, comp = np.unique(parent, return_inverse=True)
    lab_run = (comp + 1).astype(np.int32)
    lens = ec - sc
    flat_start = sr.astype(np.int64) * ny + sc
    idx = np.repeat(flat_start - (np.cumsum(lens) - lens), lens)         + np.arange(int(lens.sum()))
    out = np.zeros(nx * ny, dtype=np.int32)
    out[idx] = np.repeat(lab_run, lens)
    return out.reshape(nx, ny), int(roots.size)


#: The longest piece a capsule keep-out is stamped in, mm: a window per piece,
#: so a long diagonal edge does not pay for its whole bounding box.
_CAPSULE_PIECE = 1.0

#: The default model samples its core this many times finer than its fill
#: grid in each axis (#1608).
_CORE_SUB = 2


def _spread_labels(labels, radius_cells):
    """Each cell takes the largest label within `radius_cells` (a disc) of it,
    its own included: a grey dilation by shifted maxima, identical with or
    without scipy so both fronts read the same fill."""
    out = labels.copy()
    nx, ny = labels.shape
    k = int(math.floor(radius_cells))
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            if (di == 0 and dj == 0) or di * di + dj * dj > radius_cells ** 2:
                continue
            src = labels[max(0, di):nx + min(0, di), max(0, dj):ny + min(0, dj)]
            dst = out[max(0, -di):nx + min(0, -di), max(0, -dj):ny + min(0, -dj)]
            np.maximum(dst, src, out=dst)
    return out


def _npth_flashes(p) -> bool:
    """Whether KiCad flashes copper for an NPTH pad (PAD::FlashLayer): not
    when a round or oval pad is no larger than its drill and centred on it."""
    if getattr(p, 'shape', '') not in ('circle', 'oval'):
        return True
    if getattr(p, 'hole_x', None) is not None:
        return True
    dw = getattr(p, 'drill_w', 0.0) or (p.drill or 0.0)
    dh = getattr(p, 'drill_h', 0.0) or (p.drill or 0.0)
    pad = sorted((p.size_x, p.size_y))
    hole = sorted((dw, dh))
    return pad[0] > hole[0] + 1e-6 or pad[1] > hole[1] + 1e-6


#: Round-knock-out pinches whose neck is within this much (mm) above the
#: min-thickness are checked exactly (_bridge_round_pinches); a wider neck
#: holds core samples of its own.
_PINCH_BAND = 0.06
#: How far (mm) a bridged pinch reaches along its waist for the sampled core.
_PINCH_REACH = 0.25
#: KiCad's fill of a pinch between round knock-outs runs a few um narrower
#: than exact geometry (its polygonised arcs and offsets, 0.005 mm arc
#: error). Measured on KiCad 10.0.3 via-pair pinches, as the knock-out
#: growth that reproduces each threshold: 2.6-3.1 um at min_thickness 0.1,
#: 1.8-2.8 at 0.2, 0-1.7 at 0.25. At 3 um the model and KiCad differ only
#: on necks within ~5 um of KiCad's threshold, mostly a join the model
#: misses (one of 252 measured pinches the other way, 1 um under it).
_KICAD_ARC_SURPLUS = 0.003


def _dilate_disc(mask, radius_cells):
    """`mask` grown by a disc of `radius_cells` (shifted ORs: the same with
    or without scipy)."""
    out = mask.copy()
    n_i, n_j = mask.shape
    k = int(math.floor(radius_cells))
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            if (di == 0 and dj == 0) or di * di + dj * dj > radius_cells ** 2:
                continue
            src = mask[max(0, di):n_i + min(0, di), max(0, dj):n_j + min(0, dj)]
            dst = out[max(0, -di):n_i + min(0, -di), max(0, -dj):n_j + min(0, -dj)]
            dst |= src
    return out


def _bridge_round_pinches(links, carves, clips, outer_rings, x0, y0, fc, er):
    """Join the fill across the pinches between two ROUND knock-outs (vias,
    round pads, drill holes, track ends) that KiCad's opening keeps open
    (#1608).

    KiCad's fill is its deflated fill re-inflated by min_thickness/2: a union
    of discs of that radius over the deflated region C. Between two round
    knock-outs, grown by the deflate `er` (and KiCad's arc error,
    _KICAD_ARC_SURPLUS) to radii ra, rb, C either keeps a
    waist (the grown circles do not meet) or splits into two cusps whose
    tips, where the circles cross, are 2 y* apart; the inflate rejoins them
    when 2 y* <= 2 er. Neither shows in a sampled C: at exactly min_thickness
    -- 0.8 mm dogbone vias, 0.2 mm drill, hole clearance 0.25, min_thickness
    0.1 -- the waist is a point, and the sampled core was missing 0.8 mm
    BGA-field joins KiCad makes.

    So each such pair is solved exactly. With P on the centre line where the
    waist is (or where the circles cross), the line through P across the
    centre line is KiCad fill for |t| <= reach when P is inside the zone and
    the board and every OTHER knock-out keeps a slack of at least reach (and
    of at least y*) at P: beyond the tips the two discs only recede, nothing
    else can come nearer than its slack allows, and between the tips the
    line lies within er of a tip. A knock-out round about one of the two
    centres counts as that disc only where its nearest point to P is the
    centre (a track whose end it is, pointing away; a via's hole), so a track
    running past P is an OTHER. That line is painted into `links` (the
    dilated core the labelling reads), where it meets the sampled fill on
    both sides, so every join it adds is one KiCad's fill makes."""
    discs = {}
    pieces = set()
    for g, _bx0, _by0, _bx1, _by1, R in carves:
        if g[0] != 's' or not g[5]:
            continue
        ka = (round(g[1], 6), round(g[2], 6))
        kb = (round(g[3], 6), round(g[4], 6))
        for key in (ka, kb):
            if R > discs.get(key, -1.0):
                discs[key] = R
        if ka != kb:
            pieces.add((ka, kb))
            pieces.add((kb, ka))
    if len(discs) < 2:
        return
    keys = list(discs)
    rad = [discs[q] + er + _KICAD_ARC_SURPLUS for q in keys]
    # Pairs within reach of each other: the common small discs through a
    # grid of their own size, the few large ones (mounting holes) against
    # everything near them.
    rcut = float(np.percentile(rad, 90))
    G = 2.0 * rcut + _PINCH_BAND
    grid = {}
    big = []
    for n, (x, y) in enumerate(keys):
        if rad[n] > rcut:
            big.append(n)
        else:
            grid.setdefault((int(math.floor(x / G)), int(math.floor(y / G))),
                            []).append(n)
    pairs = []
    for (gx, gy), members in grid.items():
        for dx, dy in ((0, 0), (1, -1), (1, 0), (1, 1), (0, 1)):
            others = grid.get((gx + dx, gy + dy))
            if not others:
                continue
            for a in members:
                for b in others:
                    if (dx, dy) != (0, 0) or b > a:
                        pairs.append((a, b))
    rmax = max(rad)
    for i, a in enumerate(big):
        xa, ya = keys[a]
        k = int(math.ceil((rad[a] + rcut + _PINCH_BAND) / G))
        gx, gy = int(math.floor(xa / G)), int(math.floor(ya / G))
        for ix in range(gx - k, gx + k + 1):
            for iy in range(gy - k, gy + k + 1):
                pairs.extend((a, b) for b in grid.get((ix, iy), ()))
        pairs.extend((a, b) for b in big[i + 1:]
                     if math.hypot(keys[b][0] - xa, keys[b][1] - ya)
                     < rad[a] + rmax + _PINCH_BAND)
    cands = []
    for a, b in pairs:
        xa, ya = keys[a]
        xb, yb = keys[b]
        ra, rb = rad[a], rad[b]
        d = math.hypot(xb - xa, yb - ya)
        if d <= abs(ra - rb) or d >= ra + rb + _PINCH_BAND:
            continue
        if (keys[a], keys[b]) in pieces:
            continue                          # one track's two ends
        if d >= ra + rb:                      # a waist: P mid-gap
            xs, ys = ra + (d - ra - rb) / 2.0, 0.0
        else:                                 # two cusps: P between tips
            xs = (d * d + ra * ra - rb * rb) / (2.0 * d)
            ys = math.sqrt(max(0.0, ra * ra - xs * xs))
            if ys >= er:
                continue
        cands.append((a, b, d, xs, ys))
    if not cands:
        return
    # every knock-out within _PINCH_REACH of a cell, for the exact slack
    cg = 1.0
    index = {}
    for k, (_g, bx0, by0, bx1, by1, R) in enumerate(carves):
        m = R + er + _PINCH_REACH
        for ix in range(int(math.floor((bx0 - m) / cg)),
                        int(math.floor((bx1 + m) / cg)) + 1):
            for iy in range(int(math.floor((by0 - m) / cg)),
                            int(math.floor((by1 + m) / cg)) + 1):
                index.setdefault((ix, iy), []).append(k)
    from obstacle_map import _scanline_inside_rows

    def inside(px, py, rings):
        arrs = [np.asarray(r, dtype=np.float64) for r in rings if len(r) >= 3]
        if not arrs:
            return False
        e = [np.concatenate(v) for v in zip(*[
            (q[:, 0], q[:, 1], np.roll(q[:, 0], -1), np.roll(q[:, 1], -1))
            for q in arrs])]
        return bool(_scanline_inside_rows(np.array([px]), np.array([py]),
                                          *e)[0, 0])

    n_i, n_j = links.shape
    for a, b, d, xs, ys in cands:
        (xa, ya), (xb, yb) = keys[a], keys[b]
        ux, uy = (xb - xa) / d, (yb - ya) / d
        px, py = xa + ux * xs, ya + uy * xs
        # A capsule (a disc is one of length 0) ending at one of the two
        # centres, whose nearest point to P is that centre, lies within that
        # disc's knock-out where it matters: its distance only grows across
        # the centre line. Every other knock-out bounds `slack`.
        da = math.hypot(px - xa, py - ya)
        db = math.hypot(px - xb, py - yb)
        slack = _PINCH_REACH
        for k in index.get((int(math.floor(px / cg)),
                            int(math.floor(py / cg))), ()):
            g, _bx0, _by0, _bx1, _by1, R = carves[k]
            gd = _geom_dist(g, px, py)
            sk = gd - R - er
            if sk >= slack:
                continue
            if g[0] == 's' and g[5]:
                ea = ((abs(g[1] - xa) < 1e-6 and abs(g[2] - ya) < 1e-6)
                      or (abs(g[3] - xa) < 1e-6 and abs(g[4] - ya) < 1e-6))
                eb = ((abs(g[1] - xb) < 1e-6 and abs(g[2] - yb) < 1e-6)
                      or (abs(g[3] - xb) < 1e-6 and abs(g[4] - yb) < 1e-6))
                if (ea and gd >= da - 1e-9) or (eb and gd >= db - 1e-9):
                    continue
            slack = sk
            if slack <= ys:
                break
        reach = slack
        if reach <= ys:
            continue
        if outer_rings and not any(inside(px, py, [o]) for o in outer_rings):
            continue
        if not all(inside(px, py, rings) == keep
                   if cx0 <= px <= cx1 and cy0 <= py <= cy1 else not keep
                   for rings, keep, cx0, cy0, cx1, cy1 in clips):
            continue
        n = max(1, int(math.ceil(2.0 * reach / (fc / 2.0))))
        for k in range(n + 1):
            t = -reach + 2.0 * reach * k / n
            i = int((px - uy * t - x0) / fc)
            j = int((py + ux * t - y0) / fc)
            if 0 <= i < n_i and 0 <= j < n_j:
                links[i, j] = True


def _geom_dist(g, x, y):
    """Distance from (x, y) to a recorded knock-out's geometry: ('s', ax, ay,
    bx, by, round_ends) a segment (a point when its ends meet; round_ends
    False for an outline's edge, whose ends are corners), ('r', cx, cy, hx,
    hy, c, s) a rectangle of half-sizes hx, hy turned by (cos, sin) =
    (c, s)."""
    if g[0] == 's':
        _k, ax, ay, bx, by = g[:5]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        return math.hypot(x - ax - t * dx, y - ay - t * dy)
    _k, cx, cy, hx, hy, c, s_ = g
    lx = (x - cx) * c + (y - cy) * s_
    ly = -(x - cx) * s_ + (y - cy) * c
    return math.hypot(max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0))


class ZoneFillModel:
    """Reached-fill bitmap for one zone (net + layer + outline)."""

    def __init__(self, pcb_data, zone, net_clearances=None,
                 conservative=False, hole_clearance=None):
        self.ok = False
        self.conservative = bool(conservative)
        net_id = zone.net_id
        layer = zone.layer
        # Per-net class clearances (#483 item 5). Explicit arg wins (tests /
        # one-off probes); otherwise the board-level map. Absent net => 0.0,
        # i.e. inert -- net_clearance_map_by_id records only NON-Default
        # classes, and the Default class is already inside `zc`.
        _nc = net_clearances if net_clearances is not None \
            else (getattr(pcb_data, _NC_ATTR, None) or {})
        zc = zone.clearance if zone.clearance is not None \
            else defaults.PLANE_ZONE_CLEARANCE
        mth = zone.min_thickness if zone.min_thickness is not None \
            else defaults.PLANE_MIN_THICKNESS
        cell = min(0.08, max(0.04, mth / 2.0))
        xs = [p[0] for p in zone.polygon]
        ys = [p[1] for p in zone.polygon]
        x0, y0 = min(xs) - cell, min(ys) - cell
        nx = int((max(xs) - x0) / cell) + 2
        ny = int((max(ys) - y0) / cell) + 2
        if nx <= 2 or ny <= 2 or nx * ny > 30_000_000:
            return
        self.x0, self.y0, self.cell = x0, y0, cell
        self.nx, self.ny = nx, ny
        # CONSERVATIVE mode (#1581): every keep-out grows by `_cm` and the
        # opening is skipped, so a free cell is a CORE cell -- its centre at
        # least (clearance + min_thickness/2 + cell/2) from every foreign
        # edge, the zone outline and the board edge. Every point of the
        # segment between two 4-adjacent core centres then lies at least
        # (clearance + min_thickness/2) from foreign copper, so KiCad's
        # deflate-then-inflate fill holds a min_thickness-wide strip along
        # it: a core path IS fill KiCad pours, whatever the channel's angle.
        # The slack covers KiCad's arc approximation.
        _cm = (mth / 2.0 + cell / 2.0 + 0.01) if self.conservative else 0.0
        # HOLE clearance (#1608): KiCad's filler keeps the fill
        # max(clearance, hole clearance) from every foreign drill -- a via's,
        # a plated pad's and an NPTH's -- at the rule the board declares,
        # else pcbnew's factory 0.25 (kicad_fill_hole_clearance; probed on
        # 10.0.3). An explicit `hole_clearance` overrides the board's rule.
        _hc = float(kicad_fill_hole_clearance(pcb_data)
                    if hole_clearance is None else (hole_clearance or 0.0))

        # Two masks, stamped together (#1608). `free`: the cell centre is at
        # least the clearance from every foreign edge and inside the outline
        # -- where fill may lie. `core`: at least clearance + min_thickness/2
        # -- KiCad's DEFLATED fill, the erosion half of its min-thickness
        # opening, computed exactly at each of its sample points (half a
        # cell apart) from the same distances rather than by a raster
        # structuring element. The old raster opening (1-cell plus at
        # min_thickness 0.1) passed a 45-degree channel down to ~0.08 mm and
        # missed axis-aligned ones up to ~0.14 mm. The deflate stops 0.5 um
        # short of min_thickness/2: KiCad 10.0.3 keeps a neck 1 um under
        # min_thickness and drops one 2 um under. A conservative model has
        # no separate core: its `free` already is one.
        _er = 0.0 if self.conservative else max(0.0, mth / 2.0 - 0.0005)
        free = np.ones((nx, ny), dtype=bool)
        core = None if self.conservative else True   # allocated below
        # `guard` is the clearance the CURRENT object is stamped at. Base =
        # max(zone clearance, THIS zone's net class) -- the zone's own side of
        # every pair, so each loop below only has to add the foreign side.
        # Each loop raises it per-object to KiCad's real pairwise rule
        # (#483 item 5 / audit H2), mirroring _neck_plane_segments'
        # _pair_clearance: max(zc, class(zone net), class(foreign net),
        # pad override). Two live per-object terms:
        #   - pad local override (#326): a 0.8mm fiducial keep-clear stamped
        #     at a flat 0.2 zc over-credits fill the refill carves away.
        #   - foreign NET CLASS: on honor-classes chains (no --clearance
        #     ceiling) a foreign net with a LOOSER class carves more than the
        #     zone clearance alone, so the model predicted fill KiCad does not
        #     produce. Inert on clamped chains, where the ceiling caps the map.
        guard_base = max(zc, _nc.get(net_id, 0.0))
        guard = guard_base

        cxs = x0 + (np.arange(nx) + 0.5) * cell   # cell-center coords
        cys = y0 + (np.arange(ny) + 0.5) * cell
        # The core is sampled `sub` times finer than the fill grid: a channel
        # a little wider than min_thickness holds core only in a strip
        # narrower than a cell, which a grid of cell spacing can miss.
        sub = _CORE_SUB if core is not None else 1
        fc = cell / sub
        fnx, fny = nx * sub, ny * sub
        fxs = x0 + (np.arange(fnx) + 0.5) * fc
        fys = y0 + (np.arange(fny) + 0.5) * fc
        if core is not None:
            core = np.ones((fnx, fny), dtype=bool)
        _carves, _clips, _outer_rings = [], [], []
        gx1, gy1 = x0 + nx * cell, y0 + ny * cell

        def _gi(v, o):
            return int((v - o) / cell)

        def _rings_inside_rows(sub_x, ys, rings):
            """Even-odd inside test of the (ys x sub_x) sub-grid against the
            union of `rings` (outer + holes concatenated), via the #546
            scanline kernel (#556). Bit-identical to the per-row crossing
            loops it replaces: same strict-straddle condition, the same
            intercept expression up to commutative float ops, and the same
            strict `x < xc` count parity (searchsorted side='right' excludes
            ties exactly like the old `sub_x < xc`). O(rows x edges)
            vectorized instead of a Python loop per row -- a whole-board
            keep-out or outline paid rows x edges Python iterations here.
            Returns (len(ys), len(sub_x)) bool."""
            from obstacle_map import _scanline_inside_rows
            arrs = [np.asarray(r, dtype=np.float64) for r in rings if len(r) >= 3]
            if not arrs:
                return np.zeros((len(ys), len(sub_x)), dtype=bool)
            ex1 = np.concatenate([a[:, 0] for a in arrs])
            ey1 = np.concatenate([a[:, 1] for a in arrs])
            ex2 = np.concatenate([np.roll(a[:, 0], -1) for a in arrs])
            ey2 = np.concatenate([np.roll(a[:, 1], -1) for a in arrs])
            return _scanline_inside_rows(np.asarray(sub_x, dtype=np.float64),
                                         np.asarray(ys, dtype=np.float64),
                                         ex1, ey1, ex2, ey2)

        def _win(lo_x, hi_x, lo_y, hi_y, step, n_x, n_y):
            return (max(0, int((lo_x - x0) / step)),
                    min(n_x, int((hi_x - x0) / step) + 2),
                    max(0, int((lo_y - y0) / step)),
                    min(n_y, int((hi_y - y0) / step) + 2))

        def _carve(d2f, bx0, by0, bx1, by1, R, geom=None):
            """Keep-out of an object whose squared distance from a point is
            d2f(X column, Y row) and whose extent is the box bx0..bx1 x
            by0..by1: `free` loses the cells within R of it, the core those
            within R + the deflate. `geom` (see _geom_dist) is recorded for
            the pinch bridge."""
            if core is not None and geom is not None:
                m = R + _er + _PINCH_REACH      # near enough to matter
                if (bx1 + m >= x0 and bx0 - m <= gx1
                        and by1 + m >= y0 and by0 - m <= gy1):
                    _carves.append((geom, bx0, by0, bx1, by1, R))
            if core is None or sub > 1:
                i0, i1, j0, j1 = _win(bx0 - R, bx1 + R, by0 - R, by1 + R,
                                      cell, nx, ny)
                if i0 < i1 and j0 < j1:
                    free[i0:i1, j0:j1] &= \
                        d2f(cxs[i0:i1, None], cys[None, j0:j1]) >= R * R
                if core is None:
                    return
            Rc = R + _er
            i0, i1, j0, j1 = _win(bx0 - Rc, bx1 + Rc, by0 - Rc, by1 + Rc,
                                  fc, fnx, fny)
            if i0 >= i1 or j0 >= j1:
                return
            d2 = d2f(fxs[i0:i1, None], fys[None, j0:j1])
            core[i0:i1, j0:j1] &= d2 >= Rc * Rc
            if sub == 1:
                free[i0:i1, j0:j1] &= d2 >= R * R

        def _and_fine(mask, i0=0, j0=0):
            """core &= `mask` (a fill-grid window at i0, j0) at the core's
            resolution. Exact for an even-odd clip: a core sample its fill
            cell misjudges lies within 0.36 cell of the ring, and the ring's
            own band stamp (the deflate reaches further) removes it from the
            core either way."""
            n_i, n_j = mask.shape
            for a in range(sub):
                for b in range(sub):
                    core[i0 * sub + a:(i0 + n_i) * sub:sub,
                         j0 * sub + b:(j0 + n_j) * sub:sub] &= mask

        def _clip(rings, keep_inside, box=None):
            """`free` and the core keep only the cells inside (keep_inside)
            or outside the even-odd union of `rings`, within `box`
            (lo_x, hi_x, lo_y, hi_y; None = the whole grid). The caller
            stamps the rings' band."""
            if box is None:
                i0, i1, j0, j1 = 0, nx, 0, ny
            else:
                i0, i1, j0, j1 = _win(*box, cell, nx, ny)
            if i0 >= i1 or j0 >= j1:
                return
            ins = _rings_inside_rows(cxs[i0:i1], cys[j0:j1], rings).T
            if not keep_inside:
                ins = ~ins
            free[i0:i1, j0:j1] &= ins
            if core is not None:
                _and_fine(ins, i0, j0)
                _clips.append((rings, keep_inside,
                               min(q[0] for r in rings for q in r),
                               min(q[1] for r in rings for q in r),
                               max(q[0] for r in rings for q in r),
                               max(q[1] for r in rings for q in r)))

        def _stamp_disc(px, py, r):
            def d2f(X, Y):
                return (X - px) ** 2 + (Y - py) ** 2
            _carve(d2f, px, py, px, py, r + guard + _cm,
                   ('s', px, py, px, py, True))

        def _stamp_rect(px, py, hx, hy, r_extra=0.0):
            # rounded-rect keep-out: distance from cell center to the rect
            def d2f(X, Y):
                ox = np.maximum(np.abs(X - px) - hx, 0.0)
                oy = np.maximum(np.abs(Y - py) - hy, 0.0)
                return ox * ox + oy * oy
            _carve(d2f, px - hx, py - hy, px + hx, py + hy,
                   r_extra + guard + _cm, ('r', px, py, hx, hy, 1.0, 0.0))

        def _stamp_capsule(x1, y1, x2, y2, r, round_ends=True):
            # Exact capsule keep-out (distance to the segment), in pieces of
            # at most _CAPSULE_PIECE mm so a long diagonal does not pay for
            # its whole bounding box. It used to be a disc every 0.9 cell:
            # ~1 um under the capsule and one Python call per disc.
            R = r + guard + _cm
            L = math.hypot(x2 - x1, y2 - y1)
            n = max(1, int(math.ceil(L / _CAPSULE_PIECE)))
            for k in range(n):
                ax = x1 + (x2 - x1) * k / n
                ay = y1 + (y2 - y1) * k / n
                bx = x1 + (x2 - x1) * (k + 1) / n
                by = y1 + (y2 - y1) * (k + 1) / n
                ux, uy = bx - ax, by - ay
                l2 = ux * ux + uy * uy

                def d2f(X, Y, ax=ax, ay=ay, ux=ux, uy=uy, l2=l2):
                    dx = X - ax
                    dy = Y - ay
                    if l2 > 0:
                        t = np.clip((dx * ux + dy * uy) / l2, 0.0, 1.0)
                        dx = dx - t * ux
                        dy = dy - t * uy
                    return dx * dx + dy * dy
                _carve(d2f, min(ax, bx), min(ay, by), max(ax, bx),
                       max(ay, by), R, ('s', ax, ay, bx, by, round_ends))

        def _stamp_seg(s):
            _stamp_capsule(s.start_x, s.start_y, s.end_x, s.end_y,
                           s.width / 2.0)

        def _stamp_hole(p, g):
            # A pad's drill (a slot is a capsule) kept `g` away: KiCad's hole
            # knock-out. `guard` is restored after.
            nonlocal guard
            _g0 = guard
            guard = g
            (ax, ay), (bx, by), hr = pad_drill_capsule(p)
            _stamp_capsule(ax, ay, bx, by, hr)
            guard = _g0

        def _stamp_poly(ring):
            """Polygon keep-out: block cell centers inside the ring (even-odd)
            or within `guard` of an edge -- the same rule the copperpour
            keep-out stamp uses, applied to a custom pad's real outline."""
            rxs = [q[0] for q in ring]
            rys = [q[1] for q in ring]
            _clip([ring], False, (min(rxs), max(rxs), min(rys), max(rys)))
            for k in range(len(ring)):
                rx1, ry1 = ring[k]
                rx2, ry2 = ring[(k + 1) % len(ring)]
                _stamp_capsule(rx1, ry1, rx2, ry2, 0.0, False)

        def _stamp_ring_band(ring):
            """Keep-out BAND along a ring's boundary, interior untouched (#505).

            For a milled inner contour the copper is routed away along the LINE,
            so the pour pulls back from it on BOTH sides -- but the inside is
            still board (these contours enclose pads by definition), so the
            even-odd interior pass _stamp_poly does would wrongly erase the whole
            plane. Only the guard band is stamped."""
            for k in range(len(ring)):
                rx1, ry1 = ring[k]
                rx2, ry2 = ring[(k + 1) % len(ring)]
                _stamp_capsule(rx1, ry1, rx2, ry2, 0.0, False)

        def _stamp_rot_rect(px, py, hx, hy, ang_deg):
            # exact rotated-rect keep-out: rotate cell centers into the
            # pad frame, then the same rounded-rect distance as _stamp_rect
            rad = math.radians(ang_deg)
            ca, sa = math.cos(rad), math.sin(rad)
            reach = math.hypot(hx, hy)

            def d2f(X, Y):
                dx = X - px
                dy = Y - py
                ox = np.maximum(np.abs(dx * ca + dy * sa) - hx, 0.0)
                oy = np.maximum(np.abs(-dx * sa + dy * ca) - hy, 0.0)
                return ox * ox + oy * oy
            _carve(d2f, px - reach, py - reach, px + reach, py + reach,
                   guard + _cm, ('r', px, py, hx, hy, ca, sa))

        def _stamp_pad(p):
            """Shape-faithful pad keep-out. The old bounding-RECT stamp
            carved phantom corners on round pads -- a 3.3mm circle mounting
            pad grew ~2.75mm^2 of false carve, and rings of those squares
            walled off 9.4% of quickfeather's real In1.Cu fill (the pinch
            false-negatives that drove phantom pad repairs, doubled vias,
            and the /PDM.DATA rip). Circle -> disc, oval -> capsule (along
            the pad's long axis, honoring rect_rotation), rect/roundrect ->
            rect (rotated when rect_rotation is set; roundrect's corner
            radius over-carves only micro-slivers). Custom pads keep the
            bbox rect: their true outline is not modeled here (mic-ring
            class) -- a documented over-approximation."""
            px, py = p.global_x, p.global_y
            hx, hy = p.size_x / 2.0, p.size_y / 2.0
            rot = getattr(p, 'rect_rotation', 0.0) or 0.0
            polys = getattr(p, 'polygons', None)
            if polys:
                # Custom pads: the parser provides the REAL copper outline(s)
                # in board mm (#188) -- rasterize those instead of the bbox
                # (the bbox walled off finger channels and, mic-ring class,
                # entombed everything inside ring pads).
                for ring in polys:
                    if len(ring) >= 3:
                        _stamp_poly(ring)
                return
            if p.shape == 'circle' or (p.shape == 'oval'
                                       and abs(hx - hy) < 1e-9):
                _stamp_disc(px, py, max(hx, hy))
            elif p.shape == 'oval':
                r = min(hx, hy)
                ex, ey = (hx - r, 0.0) if hx >= hy else (0.0, hy - r)
                if rot:
                    rad = math.radians(rot)
                    ca, sa = math.cos(rad), math.sin(rad)
                    ex, ey = ex * ca - ey * sa, ex * sa + ey * ca
                _stamp_capsule(px - ex, py - ey, px + ex, py + ey, r)
            elif rot:
                _stamp_rot_rect(px, py, hx, hy, rot)
            else:
                _stamp_rect(px, py, hx, hy)

        # Foreign copper on this layer + every via/through barrel. `guard` is
        # rebound per object to the pairwise clearance and restored after each
        # loop (the closure reads the CURRENT binding).
        f_vias, f_segs = _foreign_copper(pcb_data, net_id, layer)
        for v in f_vias:
            guard = max(guard_base, _nc.get(v.net_id, 0.0))
            _stamp_disc(v.x, v.y, v.size / 2.0)
            if _hc > 0 and (getattr(v, 'drill', 0) or 0) > 0:
                guard = _hc
                _stamp_disc(v.x, v.y, v.drill / 2.0)
        guard = guard_base
        for s in f_segs:
            guard = max(guard_base, _nc.get(s.net_id, 0.0))
            _stamp_seg(s)
        guard = guard_base
        # The copper this model was stamped from, so a cached model can tell
        # whether the board's copper has moved on since (_model_is_current).
        # The objects are HELD, not just their ids: a freed object's id can be
        # reused by a new one, which would read as "still stamped".
        self._stamped = f_vias + f_segs
        self._stamped_ids = frozenset(map(id, self._stamped))
        self._copper_key = _copper_key(pcb_data)
        from kicad_parser import pad_joins_zone
        for plist in pcb_data.pads_by_net.values():
            for p in plist:
                if p.net_id == net_id and pad_joins_zone(p, zone):
                    continue
                is_th = p.drill and p.drill > 0
                on_layer = (layer in p.layers or '*.Cu' in p.layers)
                if not is_th and not on_layer:
                    continue
                if p.net_id == net_id:
                    # Own pad with zone connection NONE (a solder jumper's
                    # GND half): KiCad's filler knocks it out like foreign
                    # copper, at max(zone clearance, physical_clearance rule)
                    # -- the zone's own clearance here, no class pairing.
                    guard = zc
                else:
                    # Per-pad clearance override (#326): KiCad's filler (and
                    # its hole_clearance rule) honors it, so stamp this pad at
                    # max(zc, class pair, override). The closure guard is
                    # rebound for the stamps below and restored after the loop.
                    guard = max(guard_base,
                                _nc.get(p.net_id, 0.0),
                                getattr(p, 'local_clearance', 0.0) or 0.0)
                if p.pad_type == 'np_thru_hole':
                    # KiCad (probed on 10.0.3): an NPTH's hole is kept the
                    # HOLE clearance away, not the zone clearance -- declared
                    # 0 pours to the hole wall -- and only an NPTH whose
                    # pad is larger than its drill flashes copper, kept the
                    # clearance away like any pad.
                    _stamp_hole(p, _hc)
                    if _npth_flashes(p):
                        _stamp_pad(p)
                    continue
                if is_th and not on_layer:
                    # a plated hole with no pad on this layer
                    _stamp_hole(p, max(_hc, guard))
                    continue
                if is_th and p.net_id != net_id and _hc > 0:
                    _stamp_hole(p, _hc)
                _stamp_pad(p)
        guard = guard_base

        # NOT MODELLED HERE: neighbour-zone pullback (#483 item 6). KiCad's
        # filler also keeps zone-to-zone clearance, so a sibling pour on the
        # same layer pushes this fill back from their shared boundary. That
        # is real, but it was MEASURED AT SCALE AND REJECTED (#483 closed):
        # a full trigger-set wave on a deterministic grader (equal-priority
        # UUID ambiguity fixed in 9e6c48c; A/A gate flat first) improved 3
        # boards, regressed 3 and left 2 neutral, aggregate incomplete +2 /
        # DRC +5 / connection-width -1, with per-board deltas from -91 to
        # +366 segments. Correct in principle, but every board it touches
        # becomes a coin flip for no aggregate gain. Code preserved on
        # branches plane-483-fill-fidelity and item6-wave; the test in
        # tests/test_fill_model_netclass_483.py pins the current "a sibling
        # pour does not carve" contract, so landing it must flip that
        # deliberately.

        # Copperpour keep-out areas (#477): KiCad's filler subtracts rule
        # areas marked (copperpour not_allowed) from the fill, so a keep-out
        # crossing a plane splits it into real islands (mokya's antenna
        # keep-outs cut the In1 GND plane) -- invisible here before this
        # stamp, so region repair never saw the splits and only the
        # kicad-oracle caught the resulting opens. Block cells whose center
        # falls inside such an area on this layer (even-odd over outline +
        # holes, KiCad's rule). No clearance term: fill lawfully touches a
        # keep-out edge, and the global min-thickness OPENING below now
        # handles the boundary rim (the old explicit mth/2 rim here would
        # double-count it).
        rim = _cm
        for ko in (getattr(pcb_data.board_info, 'keepouts', None) or []):
            if ko.get('copper_pour_allowed', True):
                continue
            kls = ko.get('layers') or set()
            # #369 A5 layer tokens: literal name, '*.Cu', 'F&B.Cu'/'F&B';
            # an empty list means every layer.
            if kls and layer not in kls and '*.Cu' not in kls and \
                    not (layer in ('F.Cu', 'B.Cu') and
                         ({'F&B.Cu', 'F&B'} & kls)):
                continue
            rings = [ko.get('polygon') or []] + list(ko.get('holes') or [])
            rings = [r for r in rings if len(r) >= 3]
            if not rings:
                continue
            kxs = [p[0] for r in rings for p in r]
            kys = [p[1] for r in rings for p in r]
            _clip(rings, False, (min(kxs), max(kxs), min(kys), max(kys)))
            if rim + _er > 0:
                # the boundary rim: `rim` for a conservative model, the
                # core's deflate for the default one (guard 0: fill may touch
                # a keep-out edge)
                guard = 0.0
                for r in rings:
                    for k in range(len(r)):
                        rx1, ry1 = r[k]
                        rx2, ry2 = r[(k + 1) % len(r)]
                        _stamp_capsule(rx1, ry1, rx2, ry2, 0.0, False)
                guard = guard_base

        # Every Edge.Cuts item below is kept at KiCad's edge gap,
        # max(the zone's clearance, the board's copper-to-edge rule): the
        # filler knocks each one out at that gap before the min-thickness
        # opening, so a strip between the edge and a pad that is thinner than
        # min_thickness pours nothing.
        guard = max(guard_base, board_edge_rule(pcb_data))

        # Board OUTLINE (#1564): KiCad clips the fill to the board outline,
        # but the model clipped only to the zone polygon. A plane drawn as
        # the board's bounding box (route_planes does exactly that) overhangs
        # every notch of a non-rectangular board, and the model filled the
        # notches and the edge band: on one keyboard it joined a 106 mm^2
        # island to the main flood around a stepped edge, over two resistor
        # pads that KiCad's fill does not pass, so the GND split was never
        # repaired. A plain rectangle has no outline ring; its bounds are
        # the ring (rect_ring).
        from check_drc import board_edge_geometry
        _outers = board_edge_geometry(pcb_data.board_info, rect_ring=True)[1]
        if _outers:
            _on_board = np.zeros((ny, nx), dtype=bool)
            for _o in _outers:
                _on_board |= _rings_inside_rows(cxs, cys, [_o])
            free &= _on_board.T
            if core is not None:
                _and_fine(_on_board.T)
                _outer_rings.extend(_outers)
            for _o in _outers:
                _stamp_ring_band(_o)

        # Board CUTOUTS (crkbd class, #490): KiCad's filler pulls fill out
        # of Edge.Cuts openings and keeps clearance from their rings, but the
        # model only clipped to the ZONE polygon -- fill was predicted
        # straight through crkbd's 72 key-switch holes, and the fill-aware
        # via preference steered taps onto the phantom. Stamp each cutout as
        # a keep-out at the edge gap. NOTE: only safe now that
        # the obstacle-map cutout bands are honest (the a4e2ef6 cache-key
        # fix); over broken maps, honest fill = more fragmentation = more
        # repair copper through exactly the unguarded channels.
        for _cut in (getattr(pcb_data.board_info, 'board_cutouts', None)
                     or []):
            if len(_cut) >= 3:
                _stamp_poly(_cut)

        # Milled INNER contours (#505): Edge.Cuts geometry that is not a hole
        # and not an outer ring -- a pad-containing inner outline reclassified
        # by drop_pad_containing_cutouts. KiCad mills along the line, so the
        # pour pulls back from BOTH sides of it; but the interior is board (it
        # encloses pads by definition), so this is a BAND, never the even-odd
        # fill _stamp_poly applies to a real opening. Without it the model
        # predicts pour straight across the mill line -- on crkbd, across the
        # gap between the two keyboard halves -- and the fill-aware via
        # preference then steers plane taps onto copper that fab removes.
        for _ec in (getattr(pcb_data.board_info, 'board_edge_contours', None)
                    or []):
            if len(_ec) >= 3:
                _stamp_ring_band(_ec)
        guard = guard_base

        # Outline clip (#556: scanline kernel, bit-identical to the old
        # per-row ray cast -- see _rings_inside_rows), even-odd over the
        # outline AND its cut-outs (Zone.holes): KiCad never fills a hole.
        _clip([zone.polygon] + list(getattr(zone, 'holes', None) or ()), True)
        if self.conservative or core is not None:
            # KiCad deflates the fill at the zone's own outline and cut-outs
            # too: a conservative model keeps its `_cm` inside them, the
            # default model's core its `_er` (guard 0 leaves `free` alone).
            guard = 0.0
            for _r in [zone.polygon] + list(getattr(zone, 'holes', None) or ()):
                if len(_r) >= 3:
                    _stamp_ring_band(_r)
            guard = guard_base

        # Label the fill into components. Two items are fill-connected iff
        # they touch the SAME component -- the whole parity fix. (No
        # anchor/flood step: seeding from the net's own copper is circular --
        # an isolated via's island contains the via.) Without scipy (KiCad's
        # bundled Python has none: the GUI's grader) a row-run union-find
        # labels the same components.
        if core is None:
            # conservative: `free` is already the core, 4-connected
            if _HAS_SCIPY:
                self.labels, self._n = _ndi.label(free)
            else:
                self.labels, self._n = _label_components(free)
            self.ok = True
            return
        # Min-thickness OPENING (KiCad parity, #1608): KiCad's fill is the
        # core inflated by min_thickness/2, i.e. a union of discs of that
        # radius centred on the core, and two discs touch iff their centres
        # are within min_thickness. So two core cells are joined when they
        # are 8-adjacent (a diagonal step is cell*sqrt(2) <= min_thickness at
        # the model's cell = min_thickness/2; a coarser cell falls back to
        # 4-adjacency), and every join the labelling makes is one KiCad's
        # fill makes. A channel only slightly wider than min_thickness can
        # still hold no core cell centre (a missed join, never a false one).
        # The inflate joins two pieces of deflated fill whose discs overlap,
        # i.e. whose points are within 2 * deflate: the core grown by
        # (deflate - half a sample diagonal) touches 8-adjacently only then,
        # so the labelling of the grown core makes only joins KiCad makes.
        # Without room for that (a min_thickness under ~0.07) the core is
        # labelled as it is.
        rho = _er - fc * math.sqrt(2.0) / 2.0
        if rho > 0:
            links = _dilate_disc(core, rho / fc)
            eight = True
        else:
            links = core
            eight = fc * math.sqrt(2.0) <= 2.0 * _er + 1e-9
        _bridge_round_pinches(links, _carves, _clips, _outer_rings,
                              x0, y0, fc, _er)
        if _HAS_SCIPY:
            labels, self._n = _ndi.label(
                links, structure=np.ones((3, 3), dtype=bool) if eight else None)
        else:
            labels, self._n = _label_components(links, eight=eight)
        if sub > 1:
            # a fill cell carries the largest core label among its samples
            _c = labels[0::sub, 0::sub].copy()
            for _a in range(sub):
                for _b in range(sub):
                    if _a or _b:
                        np.maximum(_c, labels[_a::sub, _b::sub], out=_c)
            labels = _c
        # The inflate half: each fill cell within min_thickness/2 (plus half
        # a cell of sampling slack) of a core cell carries that core's label,
        # so a query at an item's edge still finds the fill lapping it. A
        # free cell no core cell reaches is a sliver KiCad's opening removes.
        labels = _spread_labels(labels, (mth / 2.0) / cell + 0.5)
        labels[~free] = 0
        self.labels = labels
        self.ok = True

    def largest_component(self) -> int:
        """Label id of the biggest fill component (the plane proper); 0 if none."""
        if not self.ok or self._n == 0:
            return 0
        counts = np.bincount(self.labels.ravel())
        counts[0] = 0
        return int(counts.argmax())

    def query_component(self, x, y, size: float = 0.0) -> Optional[int]:
        """Fill component id at (x, y): >0 = a component, 0 = no fill there,
        None = outside this model's coverage (caller falls back). `size`
        widens the probe: an item of that diameter touches a component if
        any cell within size/2 (+1 cell slack) carries its label -- a via
        barrel whose CENTER cell is eroded still touches the fill that laps
        its edge; the nearest such cell answers. A CONSERVATIVE model ignores
        `size` and answers for the point's own cell: the nearest labelled
        cell in a widened window may be a component across a wall from the
        point."""
        if not self.ok:
            return None
        i = int((x - self.x0) / self.cell)
        j = int((y - self.y0) / self.cell)
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return None
        if getattr(self, 'conservative', False):
            size = 0.0
        r = int(math.ceil((size / 2.0) / self.cell)) + 1 if size > 0 else 0
        own = int(self.labels[i, j])
        if r == 0 or own:
            return own          # a labelled cell is its own nearest
        i0, i1 = max(0, i - r), min(self.nx, i + r + 1)
        j0, j1 = max(0, j - r), min(self.ny, j + r + 1)
        win = self.labels[i0:i1, j0:j1]
        ii, jj = np.nonzero(win)
        if ii.size == 0:
            return 0
        # the nearest labelled cell wins, ties to the smallest label
        # (deterministic). It used to be the smallest label in the window
        # (#1608): a pad in a pocket walled off from the flood took the
        # flood's label whenever the flood came within the window.
        d2 = (ii + i0 - i) ** 2 + (jj + j0 - j) ** 2
        near = d2 == d2.min()
        return int(win[ii[near], jj[near]].min())

    def nearest_component_point(self, x, y, comp, max_r_mm):
        """Board coords of the nearest cell carrying label `comp` within
        `max_r_mm` of (x, y). Returns (px, py, dist_mm) or None. Used by the
        fanout near-miss pour-track (#652): find where the plane's main fill
        component actually is, so a ball the fill stops short of can reach it
        with a short track instead of a via."""
        if not self.ok or comp <= 0:
            return None
        i = int((x - self.x0) / self.cell)
        j = int((y - self.y0) / self.cell)
        r = int(math.ceil(max_r_mm / self.cell)) + 1
        i0, i1 = max(0, i - r), min(self.nx, i + r + 1)
        j0, j1 = max(0, j - r), min(self.ny, j + r + 1)
        if i0 >= i1 or j0 >= j1:
            return None
        win = self.labels[i0:i1, j0:j1] == comp
        if not win.any():
            return None
        ii, jj = np.nonzero(win)
        px = self.x0 + (i0 + ii + 0.5) * self.cell
        py = self.y0 + (j0 + jj + 0.5) * self.cell
        d2 = (px - x) ** 2 + (py - y) ** 2
        # rounded before the argmin: a ball at a corner of the fill is
        # EQUALLY near two cells in exact arithmetic, and the last bit of
        # the squared distance otherwise chose its pour-direct stub (the
        # same board shifted in memory tapped the other way, #622 pose
        # gate 2026-09-08); equal stays equal and raster order decides
        k = int(np.round(d2, 9).argmin())
        d = math.sqrt(float(d2[k]))
        if d > max_r_mm:
            return None
        return (float(px[k]), float(py[k]), d)


class IslandFillModel(ZoneFillModel):
    """ZoneFillModel's queries, answered from KiCad's OWN fill of the zone.

    The raster model predicts the fill from the copper; this one is handed
    the fill -- the (net, layer) islands of a KiCad refill
    (`kicad_exact_fill.refill_islands_ex`) -- and labels every cell inside
    island k with k. Each island is one connected polygon (KiCad's fracture
    output), so a corridor thinner than a cell cannot split it the way it
    can split a raster component. Same grid, same `query_component` probe,
    so a grader reading it keeps its own union-find and changes only whose
    fill it believes.

    Own-net pads the fill touches are painted with the island's label: a
    thermal relief's spokes end on the pad's edge, so the pad's centre --
    the point a grader asks about -- lies outside the filled polygon. A pad
    touches when a fill cell's centre is within one cell of its edge, which
    is less than any zone clearance a pad kept apart from the fill has.

    Why it exists: KiCad's filler can pour much less than the copper
    predicts, and no raster model can know it. Measured on KiCad 10.0.0:
    three foreign vias whose knockouts pinch a small pocket of the pour
    (fusb302 at (153.25, 87.85), (153.9, 87.5), (154.1, 87.95); tigard at
    (52.55, 62.85), (53.1, 62.95), (52.8, 63.6)) make the refill pour
    NOTHING for the whole zone, island removal off or on. On fusb302,
    stripped to its outline and pours, the zone pours 1380 mm^2 with any
    one of the three vias removed, with the zone clearance 0.15 or 0.16
    instead of 0.1524, or with min_thickness 0.08 or 0.12 instead of 0.1.
    The raster model fills the plane and grades every pad on it connected.
    """

    def __init__(self, pcb_data, zone, islands):
        self.ok = False
        mth = zone.min_thickness if zone.min_thickness is not None \
            else defaults.PLANE_MIN_THICKNESS
        cell = min(0.08, max(0.04, mth / 2.0))
        xs = [p[0] for p in zone.polygon]
        ys = [p[1] for p in zone.polygon]
        if not xs:
            return
        x0, y0 = min(xs) - cell, min(ys) - cell
        nx = int((max(xs) - x0) / cell) + 2
        ny = int((max(ys) - y0) / cell) + 2
        if nx <= 2 or ny <= 2 or nx * ny > 30_000_000:
            return
        self.x0, self.y0, self.cell = x0, y0, cell
        self.nx, self.ny = nx, ny
        cxs = x0 + (np.arange(nx) + 0.5) * cell
        cys = y0 + (np.arange(ny) + 0.5) * cell
        labels = np.zeros((nx, ny), dtype=np.int32)
        from obstacle_map import _scanline_inside_rows
        n = 0
        for poly in islands or ():
            if len(poly) < 3:
                continue
            n += 1
            a = np.asarray(poly, dtype=np.float64)
            i0 = max(0, int((a[:, 0].min() - x0) / cell))
            i1 = min(nx, int((a[:, 0].max() - x0) / cell) + 2)
            j0 = max(0, int((a[:, 1].min() - y0) / cell))
            j1 = min(ny, int((a[:, 1].max() - y0) / cell) + 2)
            if i0 >= i1 or j0 >= j1:
                continue
            inside = _scanline_inside_rows(
                cxs[i0:i1], cys[j0:j1], a[:, 0], a[:, 1],
                np.roll(a[:, 0], -1), np.roll(a[:, 1], -1)).T
            sub = labels[i0:i1, j0:j1]
            sub[inside & (sub == 0)] = n
        self._n = n
        if n:
            from net_queries import expand_pad_layers
            from kicad_parser import pad_is_plated_through
            cu = list(getattr(pcb_data.board_info, 'copper_layers', None)
                      or [zone.layer])
            for p in (pcb_data.pads_by_net or {}).get(zone.net_id, []) or []:
                if getattr(p, 'pad_type', '') == 'np_thru_hole':
                    continue
                if not (pad_is_plated_through(p)
                        or zone.layer in expand_pad_layers(p.layers or [], cu)):
                    continue
                hx, hy = p.size_x / 2.0, p.size_y / 2.0
                wi0 = max(0, int((p.global_x - hx - cell - x0) / cell))
                wi1 = min(nx, int((p.global_x + hx + cell - x0) / cell) + 1)
                wj0 = max(0, int((p.global_y - hy - cell - y0) / cell))
                wj1 = min(ny, int((p.global_y + hy + cell - y0) / cell) + 1)
                if wi0 >= wi1 or wj0 >= wj1:
                    continue
                # Distance from each cell centre to the pad's rectangle: 0
                # on the pad, a fill cell within one cell of its edge is a
                # spoke (or solid fill) landing on it.
                ex = np.maximum(np.abs(cxs[wi0:wi1] - p.global_x) - hx, 0.0)
                ey = np.maximum(np.abs(cys[wj0:wj1] - p.global_y) - hy, 0.0)
                d2 = ex[:, None] ** 2 + ey[None, :] ** 2
                win = labels[wi0:wi1, wj0:wj1]
                hit = win[(win > 0) & (d2 <= cell * cell)]
                if hit.size == 0:
                    continue
                on_pad = (d2 == 0.0) & (win == 0)
                win[on_pad] = int(hit.min())
        self.labels = labels
        self.ok = True


def install_kicad_fill(pcb_data, islands):
    """Make every fill-model reader of `pcb_data` read KiCad's own fill.

    `islands` is `refill_islands_ex`'s {(net name, layer): [polygon, ...]};
    a pour zone whose (net, layer) is absent poured nothing. Each zone's
    model in `pcb_data`'s cache becomes an IslandFillModel, so
    `check_net_connectivity(..., pcb_data=pcb_data)` and everything built on
    it (improvement_gate.grade_nets) grade the pour KiCad ships. Not
    registered for the zone-object lookup: only readers of this board see it.

    Returns {(net name, layer): island count} for the zones it installed."""
    cache = getattr(pcb_data, _CACHE_ATTR, None)
    if cache is None:
        cache = {}
        setattr(pcb_data, _CACHE_ATTR, cache)
    out = {}
    for z in getattr(pcb_data, 'zones', None) or []:
        net = (pcb_data.nets or {}).get(z.net_id)
        name = getattr(net, 'name', None)
        if not z.net_id or not name or not z.polygon:
            continue
        polys = (islands or {}).get((name, z.layer), [])
        m = IslandFillModel(pcb_data, z, polys)
        if m.ok:
            cache[(z.net_id, z.layer, id(z))] = m
            out[(name, z.layer)] = len(polys)
    return out


def board_edge_rule(pcb_data) -> float:
    """The copper-to-Edge.Cuts rule KiCad fills this board's zones at: the
    `min_copper_edge_clearance` of the board file's sibling .kicad_pro, the
    project KiCad's own refill reads (0.0 with no project or no rule). Both
    fronts set `source_path` to the board file."""
    path = getattr(pcb_data, 'source_path', '') or ''
    if not path:
        return 0.0
    from fix_kicad_drc_settings import read_project_edge_clearance
    return float(read_project_edge_clearance(path) or 0.0)


def set_board_net_clearances(pcb_data, net_clearances):
    """Publish the per-net class clearance map every fill model for this board
    should stamp at (#483 item 5).

    Call this once from whichever consumer resolved the map (route_planes and
    the repair entry points do), BEFORE any model is built. Models are cached
    and shared across consumers, so the map has to be a board property: two
    build sites (check_connected's zone credit, pcb_modification's fill
    anchors) have no config to thread it from, and they must not be handed a
    model stamped under different rules than the one that built it assumed.

    Changing an already-published map drops the model cache -- a model built
    under the old clearances predicts the wrong fill. Publishing the same map
    twice is free, so callers may call it unconditionally.
    """
    nc = {int(k): float(v) for k, v in (net_clearances or {}).items()
          if v}
    if (getattr(pcb_data, _NC_ATTR, None) or {}) == nc:
        return
    try:
        setattr(pcb_data, _NC_ATTR, nc)
    except Exception:
        return          # slotted/frozen board object: models stay flat-zc
    try:
        delattr(pcb_data, _CACHE_ATTR)
    except (AttributeError, TypeError):
        pass
    _MODELS_BY_ZONE_ID.clear()


def _foreign_copper(pcb_data, net_id, layer):
    """The copper a model of `net_id`'s zone on `layer` is carved by: every
    foreign via (a barrel crosses every layer) and the foreign tracks on
    `layer`. Returns (vias, segments) as lists."""
    return ([v for v in pcb_data.vias if v.net_id != net_id],
            [s for s in pcb_data.segments
             if s.net_id != net_id and s.layer == layer])


def _copper_key(pcb_data):
    """A cheap stamp of the board's copper state: the two copper lists (held,
    and compared by identity, so a replaced list never reads as the same one),
    their lengths, and the copper epoch every commit and rip bumps
    (pcb_modification.bump_copper_epoch)."""
    segs, vias = pcb_data.segments, pcb_data.vias
    return (segs, len(segs), vias, len(vias),
            getattr(pcb_data, '_copper_epoch', 0))


def _same_key(a, b):
    return (a is not None and a[0] is b[0] and a[2] is b[2]
            and a[1] == b[1] and a[3] == b[3] and a[4] == b[4])


def _model_is_current(pcb_data, zone, m):
    """Whether cached model `m` still describes `zone`'s fill on the board as
    it is NOW.

    A route step builds a plane net's model the first time anything asks
    (often before a single net has routed) and then lays copper for every
    other net. A model stamped from the input's copper predicts fill under
    and beside those fresh tracks: pour launches are offered from it and pads
    it reaches read as plane-connected, while KiCad pours around the tracks.

    Two tiers, so the common case stays O(1): an unchanged copper key means
    nothing was committed or ripped since the build. A changed key (the zone
    net's OWN commits bump it too, and those never carve its fill) compares
    the foreign copper the model would be stamped from now with what it was
    stamped from. A model handed KiCad's own fill (IslandFillModel) or one
    that could not be built carries no stamp and is kept as it is."""
    stamped = getattr(m, '_stamped_ids', None)
    if stamped is None:
        return True
    key = _copper_key(pcb_data)
    if _same_key(m._copper_key, key):
        return True
    vias, segs = _foreign_copper(pcb_data, zone.net_id, zone.layer)
    if (len(vias) + len(segs) == len(stamped)
            and stamped.issuperset(map(id, vias))
            and stamped.issuperset(map(id, segs))):
        m._copper_key = key
        return True
    return False


def _cached_model(pcb_data, zone):
    """(model, built) for `zone` from `pcb_data`'s cache, (re)built when the
    cached one is missing or no longer matches the board's copper."""
    cache = getattr(pcb_data, _CACHE_ATTR, None)
    if cache is None:
        cache = {}
        try:
            setattr(pcb_data, _CACHE_ATTR, cache)
        except Exception:
            pass
    key = (zone.net_id, zone.layer, id(zone))
    m = cache.get(key)
    if m is not None and _model_is_current(pcb_data, zone, m):
        return m, False
    m = ZoneFillModel(pcb_data, zone)
    cache[key] = m
    # A zone already published for the zone-object lookup gets the fresh
    # model there too, whichever reader rebuilt it.
    if (id(zone) in _MODELS_BY_ZONE_ID
            and not getattr(pcb_data, '_isolated_fill', False)):
        _MODELS_BY_ZONE_ID[id(zone)] = m
    return m, True


def get_fill_models(pcb_data, net_id):
    """Cached ZoneFillModels for a net's zones, keyed on the PCBData object.
    Returns {} when the net has no zones. A cached model is rebuilt when the
    board's foreign copper has changed since it was stamped
    (_model_is_current), so a reader mid-run sees the fill of the copper the
    run has laid so far."""
    out = {}
    zones = [z for z in (getattr(pcb_data, 'zones', None) or [])
             if z.net_id == net_id and z.polygon]
    for z in zones:
        m, _ = _cached_model(pcb_data, z)
        if m.ok:
            out.setdefault(z.layer, []).append(m)
    return out

class CopperView:
    """`base` read through another copper set, with fill models of its own.

    Models are cached ON the board object (`_CACHE_ATTR`) and built from
    whatever copper the board carries when a zone is first consulted. So two
    grades of one board in different copper states, "before" and "after" a
    run, would share one fill: the first state's. A view carries its own
    `segments`, `vias` and model cache; everything else (nets, pads, zones,
    board_info) is the base board's. Its models are not registered for the
    zone-object lookup (`lookup_zone_model`) either, so grading a view never
    changes what a later consumer of the real board reads."""

    _isolated_fill = True

    def __init__(self, base, segments, vias):
        self._base = base
        self.segments = list(segments)
        self.vias = list(vias)
        setattr(self, _CACHE_ATTR, {})

    def __getattr__(self, name):
        return getattr(self._base, name)


def get_zone_model(pcb_data, zone):
    """Cached ZoneFillModel for ONE zone object (see get_fill_models)."""
    m, built = _cached_model(pcb_data, zone)
    if built:
        if not getattr(pcb_data, '_isolated_fill', False):
            if len(_MODELS_BY_ZONE_ID) > 64:
                _MODELS_BY_ZONE_ID.clear()
            _MODELS_BY_ZONE_ID[id(zone)] = m
    return m if m.ok else None


#: pcbnew's factory ``m_HoleClearance`` (probed on 10.0.3: a fresh BOARD()
#: reads 0.25), which KiCad fills and grades at when the project leaves
#: ``min_hole_clearance`` out.
KICAD_FACTORY_HOLE_CLEARANCE = 0.25


def kicad_fill_hole_clearance(pcb_data) -> float:
    """The copper-to-hole clearance KiCad's filler keeps from a foreign drill
    on this board, as KiCad reads it (#1608): the live board's declared rule,
    else the sibling project's ``rules.min_hole_clearance`` -- a declared 0
    is 0, KiCad pours to the hole wall -- else pcbnew's factory 0.25 (a board
    with no project, or a project that leaves the key out). Probed on
    10.0.3. The default ZoneFillModel stamps at it."""
    try:
        from obstacle_map import _live_hole_rule, _file_hole_floor
        live = _live_hole_rule(pcb_data)
        if live is not None:
            return max(0.0, float(live))
        path = getattr(pcb_data, 'source_path', '') or ''
        if path:
            rule, _origin = _file_hole_floor(path)
            if rule is not None:
                return max(0.0, float(rule))
    except Exception:                                           # noqa: BLE001
        pass
    return KICAD_FACTORY_HOLE_CLEARANCE


def fill_hole_clearance(pcb_data) -> float:
    """The copper-to-hole clearance a CONSERVATIVE model keeps from a foreign
    drill on this board: the declared rule
    (obstacle_map.declared_hole_clearance, which also reads the chain's
    fab_floor_origin), else pcbnew's factory value. A project that declares 0
    reads as the factory value here, which only makes a conservative model
    stricter; the default model reads kicad_fill_hole_clearance."""
    try:
        from obstacle_map import declared_hole_clearance
        v, src = declared_hole_clearance(pcb_data)
    except Exception:                                           # noqa: BLE001
        v, src = 0.0, ''
    return float(v) if src and v > 0 else KICAD_FACTORY_HOLE_CLEARANCE


def trusted_zone_model(pcb_data, zone, cache):
    """A fill model whose component joins KiCad's fill is sure to make (#1581),
    for a REMOVAL pass that would leave copper joined to its net only by the
    pour.

    KiCad's own fill when it is installed (an IslandFillModel: it IS the
    fill), else a CONSERVATIVE ZoneFillModel (cores at clearance +
    min_thickness/2, the board's hole clearance stamped), built once per zone
    into the caller's `cache` dict and never published to the shared caches:
    a grader reading it would grade every marginal channel open. When the
    conservative model cannot be built (an oversize zone) the default model
    answers, as it does for every other reader."""
    m = cache.get(id(zone))
    if m is not None:
        return m
    std = get_zone_model(pcb_data, zone)
    if isinstance(std, IslandFillModel):
        m = std
    else:
        c = ZoneFillModel(pcb_data, zone, conservative=True,
                          hole_clearance=fill_hole_clearance(pcb_data))
        m = c if c.ok else std
    cache[id(zone)] = m
    return m


def lookup_zone_model(zone):
    """Prebuilt model for this zone OBJECT, or None. No pcb_data needed --
    for call sites that only carry the zone (build happens elsewhere)."""
    m = _MODELS_BY_ZONE_ID.get(id(zone))
    return m if (m is not None and m.ok) else None
