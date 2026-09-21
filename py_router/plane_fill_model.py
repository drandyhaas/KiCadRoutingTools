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
min_thickness/2, block cells within (zone_clearance + min_thickness/2) of
foreign copper (the cell-center eroded fill) and cells inside
copperpour-banned keep-out rule areas (#477), label connected components
(scipy when available; BFS fallback), and mark REACHED the components
touching the net's own anchor copper. A query point is fill-credited iff
its cell lies in a reached component.

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

import env_knobs
from typing import Optional

import numpy as np

import routing_defaults as defaults

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


def _label_components(free):
    """4-connected component labeling of a boolean grid, scipy-free.
    Returns (labels int32 array, n_components) matching ndimage.label
    semantics (labels 1..n, 0 = background)."""
    nx, ny = free.shape
    labels = np.zeros((nx, ny), dtype=np.int32)
    parent = [0]  # union-find; parent[i] == i for roots

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    prev_row_labels = None
    for j in range(ny):
        col = free[:, j]
        if not col.any():
            prev_row_labels = None
            continue
        # maximal free runs in this column-slice (along x)
        padded = np.concatenate(([False], col, [False]))
        d = np.diff(padded.astype(np.int8))
        starts = np.nonzero(d == 1)[0]
        ends = np.nonzero(d == -1)[0]
        row_labels = np.zeros(nx, dtype=np.int32)
        for s, e in zip(starts, ends):
            parent.append(len(parent))
            lab = len(parent) - 1
            row_labels[s:e] = lab
            if prev_row_labels is not None:
                overlap = prev_row_labels[s:e]
                for prev_lab in np.unique(overlap[overlap > 0]):
                    union(lab, int(prev_lab))
        labels[:, j] = row_labels
        prev_row_labels = row_labels
    # resolve to canonical roots, compact to 1..n
    if len(parent) > 1:
        roots = np.array([find(i) for i in range(len(parent))], dtype=np.int32)
        uniq = np.unique(roots[1:])
        remap = np.zeros(len(parent), dtype=np.int32)
        remap[uniq] = np.arange(1, len(uniq) + 1)
        flat = remap[roots[labels]]
        return flat, len(uniq)
    return labels, 0


class ZoneFillModel:
    """Reached-fill bitmap for one zone (net + layer + outline)."""

    def __init__(self, pcb_data, zone, net_clearances=None):
        self.ok = False
        net_id = zone.net_id
        layer = zone.layer
        # Per-net class clearances (#483 item 5). Explicit arg wins (tests /
        # one-off probes); otherwise the board-level map. Absent net => 0.0,
        # i.e. inert -- net_clearance_map_by_id records only NON-Default
        # classes, and the Default class is already inside `zc`.
        _nc = net_clearances if net_clearances is not None \
            else (getattr(pcb_data, _NC_ATTR, None) or {})
        # A/B kill-switches (same affordance as KICAD_NO_GATE_ORACLE): both
        # fidelity terms below make the model predict LESS fill, and every
        # consumer keys off that prediction, so being able to flip each one
        # without a rebuild is how the corpus A/B attributes a delta.
        if env_knobs.NO_FILL_NETCLASS:
            _nc = {}
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

        # Cell-center erosion at the CLEARANCE boundary: fill can exist at a
        # cell iff the cell center is >= zc from every foreign edge and
        # inside the outline. The min-thickness rule is applied AFTERWARD as
        # a morphological OPENING (erode by mth/2, dilate by mth/2) --
        # KiCad's actual fill algorithm. The old form baked mth/2 into this
        # guard, which models the fill SKELETON, not the fill: it loses a
        # uniform mth/2 rim everywhere and, worse, under-connects -- two
        # fills whose skeletons stay separate still TOUCH after KiCad's
        # dilation whenever the free corridor is mth..2*mth wide
        # (quickfeather: 5% of the real In1.Cu fill missing, U6.29's island
        # falsely severed -> phantom pad repairs, doubled vias, a healthy
        # net ripped and lost).
        free = np.ones((nx, ny), dtype=bool)
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

        def _stamp_disc(px, py, r):
            R = r + guard
            i0, i1 = max(0, _gi(px - R, x0)), min(nx, _gi(px + R, x0) + 2)
            j0, j1 = max(0, _gi(py - R, y0)), min(ny, _gi(py + R, y0) + 2)
            if i0 >= i1 or j0 >= j1:
                return
            dx = cxs[i0:i1, None] - px
            dy = cys[None, j0:j1] - py
            free[i0:i1, j0:j1] &= (dx * dx + dy * dy) >= R * R

        def _stamp_rect(px, py, hx, hy, r_extra=0.0):
            # rounded-rect keep-out: distance from cell center to the rect
            R = r_extra + guard
            i0 = max(0, _gi(px - hx - R, x0))
            i1 = min(nx, _gi(px + hx + R, x0) + 2)
            j0 = max(0, _gi(py - hy - R, y0))
            j1 = min(ny, _gi(py + hy + R, y0) + 2)
            if i0 >= i1 or j0 >= j1:
                return
            ox = np.maximum(np.abs(cxs[i0:i1, None] - px) - hx, 0.0)
            oy = np.maximum(np.abs(cys[None, j0:j1] - py) - hy, 0.0)
            free[i0:i1, j0:j1] &= (ox * ox + oy * oy) >= R * R

        def _stamp_seg(s):
            L = math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
            n = max(1, int(L / (cell * 0.9)))
            for t in range(n + 1):
                _stamp_disc(s.start_x + (s.end_x - s.start_x) * t / n,
                            s.start_y + (s.end_y - s.start_y) * t / n,
                            s.width / 2.0)

        def _stamp_capsule(x1, y1, x2, y2, r):
            L = math.hypot(x2 - x1, y2 - y1)
            n = max(1, int(L / (cell * 0.9)))
            for t in range(n + 1):
                _stamp_disc(x1 + (x2 - x1) * t / n,
                            y1 + (y2 - y1) * t / n, r)

        def _stamp_poly(ring):
            """Polygon keep-out: block cell centers inside the ring (even-odd)
            or within `guard` of an edge -- the same rule the copperpour
            keep-out stamp uses, applied to a custom pad's real outline."""
            rxs = [q[0] for q in ring]
            rys = [q[1] for q in ring]
            i0 = max(0, _gi(min(rxs) - guard, x0))
            i1 = min(nx, _gi(max(rxs) + guard, x0) + 2)
            j0 = max(0, _gi(min(rys) - guard, y0))
            j1 = min(ny, _gi(max(rys) + guard, y0) + 2)
            if i0 >= i1 or j0 >= j1:
                return
            ins = _rings_inside_rows(cxs[i0:i1], cys[j0:j1], [ring])
            free[i0:i1, j0:j1] &= ~ins.T
            for k in range(len(ring)):
                rx1, ry1 = ring[k]
                rx2, ry2 = ring[(k + 1) % len(ring)]
                _stamp_capsule(rx1, ry1, rx2, ry2, 0.0)

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
                _stamp_capsule(rx1, ry1, rx2, ry2, 0.0)

        def _stamp_rot_rect(px, py, hx, hy, ang_deg):
            # exact rotated-rect keep-out: rotate cell centers into the
            # pad frame, then the same rounded-rect distance as _stamp_rect
            R = guard
            rad = math.radians(ang_deg)
            ca, sa = math.cos(rad), math.sin(rad)
            reach = math.hypot(hx, hy) + R
            i0 = max(0, _gi(px - reach, x0))
            i1 = min(nx, _gi(px + reach, x0) + 2)
            j0 = max(0, _gi(py - reach, y0))
            j1 = min(ny, _gi(py + reach, y0) + 2)
            if i0 >= i1 or j0 >= j1:
                return
            dx = cxs[i0:i1, None] - px
            dy = cys[None, j0:j1] - py
            lx = dx * ca + dy * sa
            ly = -dx * sa + dy * ca
            ox = np.maximum(np.abs(lx) - hx, 0.0)
            oy = np.maximum(np.abs(ly) - hy, 0.0)
            free[i0:i1, j0:j1] &= (ox * ox + oy * oy) >= R * R

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
        for v in pcb_data.vias:
            if v.net_id != net_id:
                guard = max(guard_base, _nc.get(v.net_id, 0.0))
                _stamp_disc(v.x, v.y, v.size / 2.0)
        guard = guard_base
        for s in pcb_data.segments:
            if s.net_id != net_id and s.layer == layer:
                guard = max(guard_base, _nc.get(s.net_id, 0.0))
                _stamp_seg(s)
        guard = guard_base
        for plist in pcb_data.pads_by_net.values():
            for p in plist:
                if p.net_id == net_id:
                    continue
                is_th = p.drill and p.drill > 0
                on_layer = (layer in p.layers or '*.Cu' in p.layers)
                if not is_th and not on_layer:
                    continue
                # Per-pad clearance override (#326): KiCad's filler (and its
                # hole_clearance rule) honors it, so stamp this pad at
                # max(zc, class pair, override). The closure guard is rebound
                # for the stamps below and restored after the loop.
                guard = max(guard_base,
                            _nc.get(p.net_id, 0.0),
                            getattr(p, 'local_clearance', 0.0) or 0.0)
                if p.pad_type == 'np_thru_hole':
                    _stamp_disc(p.global_x, p.global_y, (p.drill or 0) / 2.0)
                    continue
                if is_th and not on_layer:
                    _stamp_disc(p.global_x, p.global_y, (p.drill or 0) / 2.0)
                    continue
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
        rim = 0.0
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
            i0 = max(0, _gi(min(kxs) - rim, x0))
            i1 = min(nx, _gi(max(kxs) + rim, x0) + 2)
            j0 = max(0, _gi(min(kys) - rim, y0))
            j1 = min(ny, _gi(max(kys) + rim, y0) + 2)
            if i0 >= i1 or j0 >= j1:
                continue
            ins = _rings_inside_rows(cxs[i0:i1], cys[j0:j1], rings)
            free[i0:i1, j0:j1] &= ~ins.T
            for r in rings:
                for k in range(len(r)):
                    rx1, ry1 = r[k]
                    rx2, ry2 = r[(k + 1) % len(r)]
                    L = math.hypot(rx2 - rx1, ry2 - ry1)
                    n = max(1, int(L / (cell * 0.9)))
                    for t in range(n + 1):
                        px = rx1 + (rx2 - rx1) * t / n
                        py = ry1 + (ry2 - ry1) * t / n
                        ii0 = max(0, _gi(px - rim, x0))
                        ii1 = min(nx, _gi(px + rim, x0) + 2)
                        jj0 = max(0, _gi(py - rim, y0))
                        jj1 = min(ny, _gi(py + rim, y0) + 2)
                        if ii0 >= ii1 or jj0 >= jj1:
                            continue
                        ddx = cxs[ii0:ii1, None] - px
                        ddy = cys[None, jj0:jj1] - py
                        free[ii0:ii1, jj0:jj1] &= \
                            (ddx * ddx + ddy * ddy) >= rim * rim

        # Board CUTOUTS (crkbd class, #490): KiCad's filler pulls fill out
        # of Edge.Cuts openings and keeps clearance from their rings, but the
        # model only clipped to the ZONE polygon -- fill was predicted
        # straight through crkbd's 72 key-switch holes, and the fill-aware
        # via preference steered taps onto the phantom. Stamp each cutout as
        # a keep-out at the zone-clearance guard. NOTE: only safe now that
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

        # Outline clip (#556: scanline kernel, bit-identical to the old
        # per-row ray cast -- see _rings_inside_rows).
        free &= _rings_inside_rows(cxs, cys, [zone.polygon]).T

        # Min-thickness OPENING (KiCad parity): erode by mth/2 then dilate
        # by mth/2 -- exactly the filler's deflate-then-stroke. Regions
        # whose eroded cores are separate but whose strokes overlap come out
        # CONNECTED, as on the real board. Disc structuring element; the
        # numpy fallback approximates it with an iterated 3x3 square
        # (over-opens diagonal slivers by < one cell -- strictly the
        # conservative direction).
        r_cells = int(round((mth / 2.0) / cell))
        if r_cells > 0:
            if _HAS_SCIPY:
                yy, xx = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
                st = (xx * xx + yy * yy) <= r_cells * r_cells
                free = _ndi.binary_dilation(
                    _ndi.binary_erosion(free, structure=st),
                    structure=st)
            else:
                er = free.copy()
                for _ in range(r_cells):
                    sh = np.zeros_like(er)
                    sh[1:-1, 1:-1] = (er[1:-1, 1:-1]
                                      & er[:-2, 1:-1] & er[2:, 1:-1]
                                      & er[1:-1, :-2] & er[1:-1, 2:]
                                      & er[:-2, :-2] & er[2:, 2:]
                                      & er[:-2, 2:] & er[2:, :-2])
                    er = sh
                di = er
                for _ in range(r_cells):
                    sh = di.copy()
                    sh[:-1, :] |= di[1:, :]
                    sh[1:, :] |= di[:-1, :]
                    sh[:, :-1] |= di[:, 1:]
                    sh[:, 1:] |= di[:, :-1]
                    sh[:-1, :-1] |= di[1:, 1:]
                    sh[1:, 1:] |= di[:-1, :-1]
                    sh[:-1, 1:] |= di[1:, :-1]
                    sh[1:, :-1] |= di[:-1, 1:]
                    di = sh
                free = di

        # Label the free space into fill components. Two items are
        # fill-connected iff they touch the SAME component -- the whole
        # parity fix. (No anchor/flood step: seeding from the net's own
        # copper is circular -- an isolated via's island contains the via.)
        if _HAS_SCIPY:
            self.labels, self._n = _ndi.label(free)
        else:
            # Pure-numpy connected-component labeling (KiCad's bundled
            # Python has no scipy -- without this the GUI's grader kept the
            # legacy blob credit and showed N5-class islands as connected
            # while KiCad DRC reported them open). Row-run labeling with
            # union-find: label maximal free runs per row, union with
            # overlapping runs of the previous row. O(cells), ~ny python
            # iterations of vectorized row work.
            self.labels, self._n = _label_components(free)
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
        its edge."""
        if not self.ok:
            return None
        i = int((x - self.x0) / self.cell)
        j = int((y - self.y0) / self.cell)
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return None
        r = int(math.ceil((size / 2.0) / self.cell)) + 1 if size > 0 else 0
        if r == 0:
            return int(self.labels[i, j])
        i0, i1 = max(0, i - r), min(self.nx, i + r + 1)
        j0, j1 = max(0, j - r), min(self.ny, j + r + 1)
        win = self.labels[i0:i1, j0:j1]
        vals = win[win > 0]
        if vals.size == 0:
            return 0
        # nearest labeled cell wins (ties: smallest label -- deterministic)
        return int(vals.min())

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


def get_fill_models(pcb_data, net_id):
    """Cached ZoneFillModels for a net's zones, keyed on the PCBData object.
    Returns {} when the net has no zones. Cache is invalidated implicitly by
    board reparse (new PCBData object); callers that mutate copper mid-run
    and need fresh models should delete the attr."""
    cache = getattr(pcb_data, _CACHE_ATTR, None)
    if cache is None:
        cache = {}
        try:
            setattr(pcb_data, _CACHE_ATTR, cache)
        except Exception:
            pass
    out = {}
    zones = [z for z in (getattr(pcb_data, 'zones', None) or [])
             if z.net_id == net_id and z.polygon]
    for z in zones:
        key = (net_id, z.layer, id(z))
        m = cache.get(key)
        if m is None:
            m = ZoneFillModel(pcb_data, z)
            cache[key] = m
        if m.ok:
            out.setdefault(z.layer, []).append(m)
    return out

def get_zone_model(pcb_data, zone):
    """Cached ZoneFillModel for ONE zone object (see get_fill_models)."""
    cache = getattr(pcb_data, _CACHE_ATTR, None)
    if cache is None:
        cache = {}
        try:
            setattr(pcb_data, _CACHE_ATTR, cache)
        except Exception:
            pass
    key = (zone.net_id, zone.layer, id(zone))
    m = cache.get(key)
    if m is None:
        m = ZoneFillModel(pcb_data, zone)
        cache[key] = m
        if len(_MODELS_BY_ZONE_ID) > 64:
            _MODELS_BY_ZONE_ID.clear()
        _MODELS_BY_ZONE_ID[id(zone)] = m
    return m if m.ok else None

def lookup_zone_model(zone):
    """Prebuilt model for this zone OBJECT, or None. No pcb_data needed --
    for call sites that only carry the zone (build happens elsewhere)."""
    m = _MODELS_BY_ZONE_ID.get(id(zone))
    return m if (m is not None and m.ok) else None
