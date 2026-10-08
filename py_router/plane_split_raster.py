"""
Raster finishing of a multi-net plane split (route_planes' spine split): the split's regions on one label grid,
each net's fill modelled the way KiCad pours it, the pieces nothing feeds given to a neighbour that can feed them,
and every region drawn back as an octilinear polygon.

The vector stage (route_planes._reach_and_sheet) decides which net owns what; this stage makes that ownership
something a fill can honour:

1. RASTERISE: one label per cell -- the background or another net -- so the regions the zones are drawn from
   share their boundaries exactly. (The background is still poured as the whole layer under the others, at the
   lowest priority; its own cells cut off inside another region -- its pockets -- become zones of their own.)
2. MODEL THE FILL of every label as KiCad pours it: its cells inside the board's real shape less the edge
   clearance, less every pad, via and hole that is not its own, and less a clearance band to every zone of HIGHER
   priority -- a zone pulls back only from the smaller zones that outrank it, the background sheet from all --
   opened at the zone's minimum width.
3. A PIECE OF FILL HOLDING NONE OF ITS NET'S ANCHORS is fed by nothing: its cells go to the neighbouring region
   whose own fill carries them on to one of ITS anchors. What no neighbour can feed stays with (or goes back to)
   the background sheet, whose fill's island removal (mode 0) drops it once it stays unfed.
4. POLYGONISE each label and simplify its outline to the eight directions a human draws a split in.

Pure numpy + scipy.ndimage + shapely: no pcbnew, so both fronts get the same result.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

NONE = -1          # a cell no region pours

# 8-neighbour and 4-neighbour structuring elements: alternated, they grow an octagon (a human's chamfered blob)
_SQ = np.ones((3, 3), dtype=bool)
_PLUS = ndimage.generate_binary_structure(2, 1)


class RasterGrid:
    """Cells of size `g` over `bounds` (min_x, min_y, max_x, max_y); cell (i, j) is column i, row j, its centre
    at (x0 + (i + 0.5) g, y0 + (j + 0.5) g). Arrays are indexed [row, col]."""

    def __init__(self, bounds, g):
        self.g = float(g)
        self.x0, self.y0 = bounds[0] - 2 * g, bounds[1] - 2 * g
        self.nx = int(math.ceil((bounds[2] - self.x0) / g)) + 2
        self.ny = int(math.ceil((bounds[3] - self.y0) / g)) + 2

    def zeros(self, dtype=bool):
        return np.zeros((self.ny, self.nx), dtype=dtype)

    def cells(self, r):
        """A distance in cells, rounded up (a clearance is never under-kept)"""
        return max(0, int(math.ceil(r / self.g - 1e-9)))

    def index(self, x, y):
        return (int(math.floor((y - self.y0) / self.g)), int(math.floor((x - self.x0) / self.g)))

    def rasterize(self, geom) -> np.ndarray:
        """Cells whose CENTRE lies inside `geom` (polygons with holes; even-odd per row)"""
        out = self.zeros()
        if geom is None or geom.is_empty:
            return out
        rings = []
        for p in getattr(geom, 'geoms', [geom]):
            if p.geom_type != 'Polygon' or p.is_empty:
                continue
            rings.append(np.asarray(p.exterior.coords))
            rings += [np.asarray(r.coords) for r in p.interiors]
        if not rings:
            return out
        ax = np.concatenate([r[:-1, 0] for r in rings])
        ay = np.concatenate([r[:-1, 1] for r in rings])
        bx = np.concatenate([r[1:, 0] for r in rings])
        by = np.concatenate([r[1:, 1] for r in rings])
        lo, hi = np.minimum(ay, by), np.maximum(ay, by)
        # an edge crosses the rows whose centre lies in [lo, hi): rows [j0, j1)
        j0 = np.clip(np.ceil((lo - self.y0) / self.g - 0.5).astype(np.int64), 0, self.ny)
        j1 = np.clip(np.ceil((hi - self.y0) / self.g - 0.5).astype(np.int64), 0, self.ny)
        n = np.maximum(j1 - j0, 0)
        if n.sum() == 0:
            return out
        e = np.repeat(np.arange(len(n)), n)
        j = np.repeat(j0, n) + (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n))
        cyj = self.y0 + (j + 0.5) * self.g
        x = ax[e] + (cyj - ay[e]) * (bx[e] - ax[e]) / (by[e] - ay[e])
        order = np.lexsort((x, j))
        j, x = j[order], x[order]
        # every row holds an even number of crossings: fill between the 1st and 2nd, the 3rd and 4th, ...
        start = np.r_[0, np.flatnonzero(np.diff(j)) + 1]
        rank = np.arange(len(j)) - np.repeat(start, np.diff(np.r_[start, len(j)]))
        a = np.flatnonzero(rank % 2 == 0)
        a = a[a + 1 < len(j)]
        a = a[j[a + 1] == j[a]]
        ca = np.clip(np.ceil((x[a] - self.x0) / self.g - 0.5).astype(np.int64), 0, self.nx)
        cb = np.clip(np.ceil((x[a + 1] - self.x0) / self.g - 0.5).astype(np.int64), 0, self.nx)
        diff = np.zeros((self.ny, self.nx + 1), dtype=np.int32)
        np.add.at(diff, (j[a], ca), 1)
        np.add.at(diff, (j[a], cb), -1)
        return np.cumsum(diff, axis=1)[:, :-1] > 0

    def cells_of(self, kind, shp):
        """(slice, local bool mask) of the cells whose centre is in a 'box' (x0, y0, x1, y1), a 'disc' (x, y, r) or a
        'capsule' (x1, y1, x2, y2, r -- a milled slot); None when it covers none"""
        g = self.g
        if kind == 'box':
            bx0, by0, bx1, by1 = shp
        elif kind == 'disc':
            x, y, r = shp
            bx0, by0, bx1, by1 = x - r, y - r, x + r, y + r
        else:
            ax, ay, bx, by, r = shp
            bx0, by0, bx1, by1 = min(ax, bx) - r, min(ay, by) - r, max(ax, bx) + r, max(ay, by) + r
        c0 = max(0, int(math.ceil((bx0 - self.x0) / g - 0.5)))
        c1 = min(self.nx, int(math.floor((bx1 - self.x0) / g - 0.5)) + 1)
        r0 = max(0, int(math.ceil((by0 - self.y0) / g - 0.5)))
        r1 = min(self.ny, int(math.floor((by1 - self.y0) / g - 0.5)) + 1)
        if c1 <= c0 or r1 <= r0:
            return None
        sl = (slice(r0, r1), slice(c0, c1))
        if kind == 'box':
            return sl, np.ones((r1 - r0, c1 - c0), dtype=bool)
        cx = self.x0 + (np.arange(c0, c1) + 0.5) * g
        cyy = self.y0 + (np.arange(r0, r1) + 0.5) * g
        if kind == 'disc':
            return sl, ((cx[None, :] - x) ** 2 + (cyy[:, None] - y) ** 2) <= r * r
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        px, py = cx[None, :], cyy[:, None]
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0) if L2 > 0 else np.zeros((1, 1))
        return sl, (px - (ax + t * dx)) ** 2 + (py - (ay + t * dy)) ** 2 <= r * r

    def polygonize(self, mask) -> List[Polygon]:
        """The cells as polygons (with holes): row runs as boxes, unioned"""
        if not mask.any():
            return []
        boxes = []
        g, x0, y0 = self.g, self.x0, self.y0
        d = np.diff(np.pad(mask.astype(np.int8), ((0, 0), (1, 1))), axis=1)
        for j in np.flatnonzero(mask.any(axis=1)):
            s = np.flatnonzero(d[j] == 1)
            e = np.flatnonzero(d[j] == -1)
            for a, b in zip(s, e):
                boxes.append(box(x0 + a * g, y0 + j * g, x0 + b * g, y0 + (j + 1) * g))
        u = unary_union(boxes)
        return [p for p in getattr(u, 'geoms', [u]) if p.geom_type == 'Polygon' and not p.is_empty]


def grow(mask, r_cells, octagonal=True):
    """Dilate by r cells: an octagon (alternating 8- and 4-neighbour steps) or a square"""
    if r_cells <= 0:
        return mask.copy()
    out = mask
    for k in range(r_cells):
        out = ndimage.binary_dilation(out, structure=(_PLUS if (octagonal and k % 2) else _SQ))
    return out


def shrink(mask, r_cells, octagonal=True):
    return ~grow(~mask, r_cells, octagonal)


def octilinear(poly: Polygon, g: float) -> Polygon:
    """A raster outline drawn as a human draws a split: its staircases simplified away (tolerance two cells -- a
    slope off the grid's own steps rasterises as uneven ones), each
    remaining edge turned to the nearest of the eight directions and the corners re-cut where the turned edges
    meet. Falls back to the simplified outline wherever the turn would not give a valid polygon of near the same
    area"""
    s = poly.simplify(g * 2.0, preserve_topology=True)
    if s.is_empty or s.geom_type != 'Polygon':
        return poly

    def snap_ring(coords):
        pts = list(coords)[:-1]
        n = len(pts)
        if n < 4:
            return None
        lines = []
        for k in range(n):
            (ax, ay), (bx, by) = pts[k], pts[(k + 1) % n]
            ang = math.atan2(by - ay, bx - ax)
            q = round(ang / (math.pi / 4)) * (math.pi / 4)
            mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
            lines.append((mx, my, math.cos(q), math.sin(q)))
        # merge consecutive edges that turned to the same direction
        merged = []
        for ln in lines:
            if merged and abs(merged[-1][2] - ln[2]) < 1e-9 and abs(merged[-1][3] - ln[3]) < 1e-9:
                continue
            merged.append(ln)
        if len(merged) > 1 and abs(merged[0][2] - merged[-1][2]) < 1e-9 and abs(merged[0][3] - merged[-1][3]) < 1e-9:
            merged.pop()
        if len(merged) < 3:
            return None
        out = []
        m = len(merged)
        for k in range(m):
            x1, y1, dx1, dy1 = merged[k - 1]
            x2, y2, dx2, dy2 = merged[k]
            den = dx1 * dy2 - dy1 * dx2
            if abs(den) < 1e-9:                  # parallel (a reversal): a jog between the two lines
                out.append((x1, y1))
                out.append((x2, y2))
                continue
            t = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / den
            out.append((x1 + t * dx1, y1 + t * dy1))
        return out
    ext = snap_ring(s.exterior.coords)
    if ext is None:
        return s
    holes = [h for h in (snap_ring(r.coords) for r in s.interiors) if h]
    cand = Polygon(ext, holes).buffer(0)
    if cand.is_empty or cand.geom_type != 'Polygon' or abs(cand.area - s.area) > 0.05 * max(s.area, 1e-9) \
            or cand.symmetric_difference(s).area > 0.1 * max(s.area, 1e-9):
        return s
    return cand


def _components(mask):
    """(component id per cell, area in cells per id -- index 0 unused) of a bool mask, 8-connected"""
    comp, k = ndimage.label(mask, structure=_SQ)
    return comp, np.bincount(comp.ravel(), minlength=k + 1)


def finish(labels_geom: Dict[int, object], dom: int, outline, grid: RasterGrid,
           anchors_by_net: Dict[int, Sequence[Tuple[float, float]]],
           copper_items: Sequence[Tuple[int, str, tuple]], hole_items: Sequence[Tuple[str, tuple]],
           zone_clearance: float, min_thickness: float, edge_clearance: float,
           board=None, rounds: int = 4) -> Tuple[Dict[int, List[Polygon]], dict]:
    """labels_geom {net: geometry} the vector stage's regions (the background `dom` everything else inside
    `outline`, the zone outline); anchors_by_net the points a net's fill must reach (its pads, vias, virtual vias);
    copper_items (net, kind, shape) every pad and via on the layer and hole_items (kind, shape) every hole and slot,
    each already grown by its clearance ('box' / 'disc' / 'capsule'); board the board's real shape, its Edge.Cuts
    cutouts out (None: the outline). Returns ({net: [polygons]}, stats)"""
    g = grid
    # whose copper each cell is (-1 nobody's, -2 several nets'): a fill keeps clear of every one but its own
    owner = np.full((g.ny, g.nx), -1, dtype=np.int64)
    for net, kind, shp in copper_items:
        hit = g.cells_of(kind, shp)
        if hit is None:
            continue
        sl, m = hit
        o = owner[sl]
        o[m & (o == -1)] = net
        o[m & (o >= 0) & (o != net)] = -2
    holes = g.zeros()
    for kind, shp in hole_items:
        hit = g.cells_of(kind, shp)
        if hit is not None:
            holes[hit[0]] |= hit[1]
    # (the regions run to the zone outline; the FILL keeps the edge clearance from the board's real edge -- its
    # outline and every cutout -- as KiCad pours it)
    inside = g.rasterize(outline)
    poured = inside & g.rasterize(board) if board is not None else inside
    edge_ok = shrink(poured, g.cells(edge_clearance), octagonal=True) if edge_clearance > 0 else poured
    lab = np.full((g.ny, g.nx), NONE, dtype=np.int64)
    lab[inside] = dom
    # larger regions first, so a region nested inside another keeps its cells (the zones' own priority order)
    for nid, geom in sorted(((n, gm) for n, gm in labels_geom.items() if n != dom and gm is not None),
                            key=lambda t: -t[1].area):
        lab[g.rasterize(geom) & inside] = nid
    nets = sorted(set(np.unique(lab[lab != NONE]).tolist()) | {dom})
    near_anchor = {}
    zc, half = g.cells(zone_clearance), g.cells(min_thickness / 2.0)
    for n in nets:
        m = g.zeros()
        for x, y in anchors_by_net.get(n, ()):
            r, c = g.index(x, y)
            if 0 <= r < g.ny and 0 <= c < g.nx:
                m[r, c] = True
        # (an anchor in a cell the fill opened away still feeds the piece it touches)
        near_anchor[n] = grow(m, half + 1, octagonal=True)
    stats = {'reassigned_cells': 0, 'background_cells': 0, 'dead_pieces': 0}

    def priority_area():
        """Each cell's zone area in cells -- a zone outranks the larger ones it touches (zone_overlap_priorities:
        the smaller first); the background SHEET (its largest piece, poured as the whole layer) outranks nothing"""
        area = np.full((g.ny, g.nx), np.inf)
        for n in nets:
            comp, sizes = _components(lab == n)
            a = sizes[comp].astype(float)
            if n == dom and len(sizes) > 1:
                a[comp == int(np.argmax(sizes[1:]) + 1)] = np.inf
            sel = comp > 0
            area[sel] = a[sel]
        return area

    def fill_of(n, area):
        """What KiCad pours of label n: inside the board less the edge clearance, round what is not its own, pulled
        back by the clearance from every zone that outranks the piece it is in, opened at the minimum width"""
        own = lab == n
        blocked = ((owner != -1) & (owner != n)) | holes
        f = np.zeros_like(own)
        foreign = (lab != n) & (lab != NONE)
        for a in np.unique(area[own]):
            piece = own & (area == a)
            f |= piece & ~grow(foreign & (area < a), zc)
        f &= edge_ok & ~blocked
        if half > 0:
            f = ndimage.binary_opening(f, structure=_SQ, iterations=half)
        return f

    for _ in range(rounds):
        moved = False
        for n in nets:
            f = fill_of(n, priority_area())
            comp, k = ndimage.label(f, structure=_SQ)
            if k == 0:
                continue
            dead = set(np.setdiff1d(np.arange(1, k + 1), np.unique(comp[near_anchor[n] & (comp > 0)])).tolist())
            if not dead:
                continue
            # each of the net's cells belongs to the fill piece nearest it: a dead piece's cells go with it
            own = lab == n
            idx = ndimage.distance_transform_edt(comp == 0, return_distances=False, return_indices=True)
            nearest = np.where(own, comp[idx[0], idx[1]], 0)
            # every dead piece offered at once to the neighbour whose border with it is longest (worked in the
            # piece's own window), then each neighbour's fill modelled ONCE to see which it carries on to one of
            # its anchors -- a model per piece costs a whole board each, and a panel has a hundred such pieces
            offers = {}
            for d, sl in enumerate(ndimage.find_objects(nearest), start=1):
                if d not in dead or sl is None:
                    continue
                win = tuple(slice(max(0, s.start - 1), s.stop + 1) for s in sl)
                cells = nearest[win] == d
                stats['dead_pieces'] += 1
                ring = ndimage.binary_dilation(cells, structure=_PLUS) & ~cells
                labs, counts = np.unique(lab[win][ring], return_counts=True)
                cand = [(c, m) for m, c in zip(labs.tolist(), counts.tolist()) if m not in (n, NONE)]
                m = max(cand)[1] if cand else dom
                offers.setdefault(m, []).append((win, cells))
            for m, pieces in offers.items():
                for win, cells in pieces:
                    lab[win][cells] = m
            area = priority_area()
            for m, pieces in offers.items():
                cm, _ = ndimage.label(fill_of(m, area), structure=_SQ)
                live = set(np.unique(cm[near_anchor[m] & (cm > 0)]).tolist())
                for win, cells in pieces:
                    ids = set(np.unique(cm[win][cells & (cm[win] > 0)]).tolist())
                    if m != dom and ids & live:
                        stats['reassigned_cells'] += int(cells.sum())
                        moved = True
                    elif m != dom:
                        # (no neighbour feeds it: the background sheet pours there regardless, so it is the
                        # background's -- a region's own outline must not carry copper nothing feeds)
                        lab[win][cells] = dom
                        if n != dom:
                            stats['background_cells'] += int(cells.sum())
                            moved = True
                    elif n != dom:
                        stats['background_cells'] += int(cells.sum())
                        moved = True
        if not moved:
            break
    out = {}
    for n in nets:
        polys = []
        for p in g.polygonize(lab == n):
            q = octilinear(p, g.g).intersection(outline)       # (a staircase past the board edge trimmed to it)
            polys += [h for h in getattr(q, 'geoms', [q])
                      if h.geom_type == 'Polygon' and h.area > g.g * g.g * 4]
        if polys:
            out[n] = polys
    return out, stats


def octagon(points: Sequence[Tuple[float, float]], margin: float) -> Optional[Polygon]:
    """The smallest octilinear-convex shape round the points -- their bounding box cut by their 45-degree bounding
    box -- grown by `margin` on each of its eight sides: the chamfered rectangle a human draws round a group"""
    if not points:
        return None
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    u, v = xs + ys, xs - ys
    m, mu = margin, margin * math.sqrt(2.0)
    x0, x1, y0, y1 = xs.min() - m, xs.max() + m, ys.min() - m, ys.max() + m
    u0, u1, v0, v1 = u.min() - mu, u.max() + mu, v.min() - mu, v.max() + mu
    bb = box(x0, y0, x1, y1)
    # the diamond u0 <= x+y <= u1, v0 <= x-y <= v1 as a polygon
    dia = Polygon([((u0 + v0) / 2, (u0 - v0) / 2), ((u0 + v1) / 2, (u0 - v1) / 2),
                   ((u1 + v1) / 2, (u1 - v1) / 2), ((u1 + v0) / 2, (u1 - v0) / 2)]).buffer(0)
    o = bb.intersection(dia)
    return o if (not o.is_empty and o.geom_type == 'Polygon') else bb


def mitred_band(path: Sequence[Tuple[float, float]], half: float):
    """A spine as a human draws a corridor: a band `half` either side, square-ended, mitred at its bends"""
    from shapely.geometry import LineString
    return LineString(path).buffer(half, cap_style=3, join_style=2, mitre_limit=2.0)
