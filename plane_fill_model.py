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
from typing import Optional

import numpy as np

import routing_defaults as defaults

try:
    from scipy import ndimage as _ndi
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

_CACHE_ATTR = '_plane_fill_models'

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

    def __init__(self, pcb_data, zone):
        self.ok = False
        net_id = zone.net_id
        layer = zone.layer
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

        # Cell-center erosion: fill exists at a cell iff the cell center is
        # >= (zc + mth/2) from every foreign edge (the mth/2 term is the
        # min-thickness opening in cell-center form) and inside the outline.
        free = np.ones((nx, ny), dtype=bool)
        guard = zc + mth / 2.0

        cxs = x0 + (np.arange(nx) + 0.5) * cell   # cell-center coords
        cys = y0 + (np.arange(ny) + 0.5) * cell

        def _gi(v, o):
            return int((v - o) / cell)

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

        # Foreign copper on this layer + every via/through barrel.
        for v in pcb_data.vias:
            if v.net_id != net_id:
                _stamp_disc(v.x, v.y, v.size / 2.0)
        for s in pcb_data.segments:
            if s.net_id != net_id and s.layer == layer:
                _stamp_seg(s)
        for plist in pcb_data.pads_by_net.values():
            for p in plist:
                if p.net_id == net_id:
                    continue
                is_th = p.drill and p.drill > 0
                on_layer = (layer in p.layers or '*.Cu' in p.layers)
                if not is_th and not on_layer:
                    continue
                if p.pad_type == 'np_thru_hole':
                    _stamp_disc(p.global_x, p.global_y, (p.drill or 0) / 2.0)
                    continue
                if is_th and not on_layer:
                    _stamp_disc(p.global_x, p.global_y, (p.drill or 0) / 2.0)
                    continue
                _stamp_rect(p.global_x, p.global_y,
                            p.size_x / 2.0, p.size_y / 2.0)

        # Copperpour keep-out areas (#477): KiCad's filler subtracts rule
        # areas marked (copperpour not_allowed) from the fill, so a keep-out
        # crossing a plane splits it into real islands (mokya's antenna
        # keep-outs cut the In1 GND plane) -- invisible here before this
        # stamp, so region repair never saw the splits and only the
        # kicad-oracle caught the resulting opens. Block cells whose center
        # falls inside such an area on this layer (even-odd over outline +
        # holes, KiCad's rule), plus an mth/2 rim for the min-thickness
        # opening at the boundary. No clearance term: fill lawfully touches
        # a keep-out edge.
        rim = mth / 2.0
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
            sub_x = cxs[i0:i1]
            for j in range(j0, j1):
                yv = cys[j]
                crossings = np.zeros(i1 - i0, dtype=np.int32)
                for r in rings:
                    for k in range(len(r)):
                        rx1, ry1 = r[k]
                        rx2, ry2 = r[(k + 1) % len(r)]
                        if (ry1 > yv) == (ry2 > yv):
                            continue
                        xc = rx1 + (yv - ry1) * (rx2 - rx1) / (ry2 - ry1)
                        crossings += (sub_x < xc)
                free[i0:i1, j] &= (crossings % 2) == 0
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

        # Outline clip (polygon test, vectorized ray cast per row).
        poly = zone.polygon
        inside = np.zeros((nx, ny), dtype=bool)
        px_arr = np.asarray([p[0] for p in poly])
        py_arr = np.asarray([p[1] for p in poly])
        for j in range(ny):
            yv = cys[j]
            crossings = np.zeros(nx, dtype=np.int32)
            for k in range(len(poly)):
                x1, y1 = px_arr[k], py_arr[k]
                x2, y2 = px_arr[(k + 1) % len(poly)], py_arr[(k + 1) % len(poly)]
                if (y1 > yv) == (y2 > yv):
                    continue
                xc = x1 + (yv - y1) * (x2 - x1) / (y2 - y1)
                crossings += (cxs < xc)
            inside[:, j] = (crossings % 2) == 1
        free &= inside

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
