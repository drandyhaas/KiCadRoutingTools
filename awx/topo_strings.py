#!/usr/bin/env python3
"""Topological string extraction (v1) for the #622 bus experiment.

For each requested net: string = polyline from the source tooth FREE END
(endpoint census of its stub) to the target ball center. Initialized as
the straight chord, then relaxed to the taut representative of that
homotopy class (elastic band: Laplacian contraction + push-out-of-
obstacles + shortcut passes). Obstacles: foreign pads (full routing
margin outside the target field, bare-copper margin inside so the ball
gaps stay threadable), foreign F.Cu segments (layer-conditional; B.Cu
copper does not constrain a nominal-F string), and all foreign vias.

Outputs: crossing matrix (straight vs taut), per-string lengths and
hugged obstacles, a strings JSON, and an Eco2.User overlay board for
render_eco.py.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))

TRACK = 0.127
CLEAR = 0.1
MARGIN_OUT = CLEAR + TRACK / 2      # routing margin outside the field
MARGIN_IN = 0.06                     # bare-copper margin inside the field
FREEZE = 0.35                        # no pushes this close to an endpoint
STEP = 0.12                          # densify step (mm)


def d2(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def seg_pt_dist(a, b, p):
    ax, ay = a
    bx, by = b
    px_, py_ = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px_ - ax, py_ - ay)
    t = max(0.0, min(1.0, ((px_ - ax) * dx + (py_ - ay) * dy) / L2))
    return math.hypot(px_ - (ax + t * dx), py_ - (ay + t * dy))


def seg_seg_dist(a, b, c, e):
    # min distance between segments ab and ce
    if seg_x(a, b, c, e, lo=0.0, hi=1.0):
        return 0.0
    return min(seg_pt_dist(a, b, c), seg_pt_dist(a, b, e),
               seg_pt_dist(c, e, a), seg_pt_dist(c, e, b))


def seg_x(a, b, c, e, lo=0.0, hi=1.0):
    # transversal intersection of ab x ce with params in (lo,hi)
    d1x, d1y = b[0] - a[0], b[1] - a[1]
    d2x, d2y = e[0] - c[0], e[1] - c[1]
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-12:
        return None
    t = ((c[0] - a[0]) * d2y - (c[1] - a[1]) * d2x) / den
    u = ((c[0] - a[0]) * d1y - (c[1] - a[1]) * d1x) / den
    if lo < t < hi and lo < u < hi:
        return (a[0] + t * d1x, a[1] + t * d1y)
    return None


class Obstacles:
    """Discs [(x,y,r,name)] + capsules [(a,b,r,name)] with a coarse
    spatial hash for point queries."""

    def __init__(self):
        self.discs = []
        self.caps = []
        self.dnets = []          # net id per disc (None = no net)
        self.cnets = []          # net id per capsule
        self._grid = {}
        self.cell = 1.0

    def signature(self):
        """A hash of the model's content (discs and capsules, rounded to
        0.1 um), so a caller can memoise work that depends only on it."""
        sig = getattr(self, '_sig', None)
        if sig is None:
            import hashlib
            h = hashlib.sha1()
            # a DERIVED model (exclude) hashes its surviving items in
            # order: the same string a model built without them hashes,
            # so the persisted taut memo keyed on it keeps hitting
            xd = getattr(self, '_xd', ())
            xc = getattr(self, '_xc', ())
            for i, (x, y, r, n) in enumerate(self.discs):
                if i in xd:
                    continue
                h.update(f'd{x:.4f},{y:.4f},{r:.4f},{n};'.encode())
            for i, (a, b, r, n) in enumerate(self.caps):
                if i in xc:
                    continue
                h.update(f'c{a[0]:.4f},{a[1]:.4f},{b[0]:.4f},{b[1]:.4f},{r:.4f},{n};'.encode())
            sig = h.hexdigest()          # stable across processes
            self._sig = sig
        return sig

    def add_disc(self, x, y, r, name, net=None):
        self.discs.append((x, y, r, name))
        self.dnets.append(net)

    def add_cap(self, a, b, r, name, net=None):
        self.caps.append((a, b, r, name))
        self.cnets.append(net)

    def _cells_of_disc(self, i):
        x, y, r, _n = self.discs[i]
        return [(gx, gy)
                for gx in range(int((x - r - 0.3) / self.cell),
                                int((x + r + 0.3) / self.cell) + 1)
                for gy in range(int((y - r - 0.3) / self.cell),
                                int((y + r + 0.3) / self.cell) + 1)]

    def _cells_of_cap(self, i):
        a, b, r, _n = self.caps[i]
        x0, x1 = min(a[0], b[0]) - r - 0.3, max(a[0], b[0]) + r + 0.3
        y0, y1 = min(a[1], b[1]) - r - 0.3, max(a[1], b[1]) + r + 0.3
        return [(gx, gy)
                for gx in range(int(x0 / self.cell), int(x1 / self.cell) + 1)
                for gy in range(int(y0 / self.cell), int(y1 / self.cell) + 1)]

    def _pack_cell(self, k):
        dd = tuple((self.discs[i][0], self.discs[i][1], self.discs[i][2])
                   for i in self._near_d.get(k, ()))
        cc = []
        for ci in self._near_c.get(k, ()):
            (ax, ay), (bx, by), r, _n = self.caps[ci]
            dx, dy = bx - ax, by - ay
            cc.append((ax, ay, dx, dy, dx * dx + dy * dy, r))
        return (dd, tuple(cc))

    def exclude(self, nets):
        """This model without the items of `nets`, as a DERIVED model
        that shares the built index and rewrites only the cells those
        items touch. A plan judges 35 nets against the same board and
        each net's model differs from the next's by that net's own few
        pads and vias -- 140 full builds per board, 111 s of a 250 s
        K35 fanout stage (2026-09-06 profile). The candidate order in
        every cell is the base's order with the excluded items removed,
        which is the order a build without them would produce, so
        point_violation and seg_clear answer bit-identically."""
        nets = set(nets)
        out = Obstacles.__new__(Obstacles)
        out.discs, out.caps = self.discs, self.caps
        out.dnets, out.cnets = self.dnets, self.cnets
        out._grid, out._cgrid, out.cell = self._grid, self._cgrid, self.cell
        out._near_d = dict(self._near_d)
        out._near_c = dict(self._near_c)
        out._pack = dict(self._pack)
        xd = {i for i, n in enumerate(self.dnets) if n is not None and n in nets}
        xc = {i for i, n in enumerate(self.cnets) if n is not None and n in nets}
        touched = set()
        for i in xd:
            for (gx, gy) in self._cells_of_disc(i):
                for dx_ in (-1, 0, 1):
                    for dy_ in (-1, 0, 1):
                        touched.add((gx + dx_, gy + dy_))
        for i in xc:
            for (gx, gy) in self._cells_of_cap(i):
                for dx_ in (-1, 0, 1):
                    for dy_ in (-1, 0, 1):
                        touched.add((gx + dx_, gy + dy_))
        for k in touched:
            dd = tuple(i for i in self._near_d.get(k, ()) if i not in xd)
            cc = tuple(i for i in self._near_c.get(k, ()) if i not in xc)
            if dd:
                out._near_d[k] = dd
            else:
                out._near_d.pop(k, None)
            if cc:
                out._near_c[k] = cc
            else:
                out._near_c.pop(k, None)
            if dd or cc:
                out._pack[k] = out._pack_cell(k)
            else:
                out._pack.pop(k, None)
        out._xd, out._xc = xd, xc
        return out

    def build(self):
        for i, (x, y, r, _n) in enumerate(self.discs):
            for gx in range(int((x - r - 0.3) / self.cell),
                            int((x + r + 0.3) / self.cell) + 1):
                for gy in range(int((y - r - 0.3) / self.cell),
                                int((y + r + 0.3) / self.cell) + 1):
                    self._grid.setdefault((gx, gy), []).append(i)
        # capsules hashed by their bounding box too: a point query used
        # to walk EVERY capsule (thousands of foreign fanout stubs on a
        # BGA board), which made the spine relaxation cost minutes
        self._cgrid = {}
        for i, (a, b, r, _n) in enumerate(self.caps):
            x0, x1 = min(a[0], b[0]) - r - 0.3, max(a[0], b[0]) + r + 0.3
            y0, y1 = min(a[1], b[1]) - r - 0.3, max(a[1], b[1]) + r + 0.3
            for gx in range(int(x0 / self.cell), int(x1 / self.cell) + 1):
                for gy in range(int(y0 / self.cell), int(y1 / self.cell) + 1):
                    self._cgrid.setdefault((gx, gy), []).append(i)
        # merged 3x3 neighbourhoods, precomputed ONCE. Point queries
        # used to re-concatenate nine cell lists (discs) and build a
        # set (caps) PER CALL -- 760k point_violation calls put 23 s
        # of the K28 braid's 79 s profile in that churn. The disc
        # sequence reproduces the old per-call concatenation order
        # exactly (same dx/dy nesting), so disc results are
        # bit-identical; caps become an order-stable dedupe (the old
        # set had no defined order for equal-depth ties).
        self._near_d = {}
        self._near_c = {}
        centers = {(gx + dx_, gy + dy_)
                   for k in (set(self._grid) | set(self._cgrid))
                   for gx, gy in (k,)
                   for dx_ in (-1, 0, 1) for dy_ in (-1, 0, 1)}
        for (gx, gy) in centers:
            dd, cc = [], []
            for dx_ in (-1, 0, 1):
                for dy_ in (-1, 0, 1):
                    dd.extend(self._grid.get((gx + dx_, gy + dy_), ()))
                    cc.extend(self._cgrid.get((gx + dx_, gy + dy_), ()))
            if dd:
                self._near_d[(gx, gy)] = tuple(dd)
            if cc:
                self._near_c[(gx, gy)] = tuple(dict.fromkeys(cc))
        # flat per-cell geometry packs for point_violation: the same
        # candidates in the same order, but as plain floats with the
        # capsule direction and L2 precomputed -- the 1.3M-call hot
        # loop then runs without per-candidate list indexing, tuple
        # unpacking or seg_pt_dist call frames (measured: numpy per
        # call is 3x SLOWER at these ~14-candidate sizes, so the win
        # is fewer interpreter frames, not vectorisation; arithmetic
        # and order are bit-identical)
        self._pack = {}
        keys = set(self._near_d) | set(self._near_c)
        for k in keys:
            self._pack[k] = self._pack_cell(k)

    def near_discs(self, p):
        return self._near_d.get((int(p[0] / self.cell),
                                 int(p[1] / self.cell)), ())

    def near_caps(self, p):
        return self._near_c.get((int(p[0] / self.cell),
                                 int(p[1] / self.cell)), ())

    def point_violation(self, p, pad=0.0):
        """Deepest violated obstacle at point p -> (depth, push_dir) or
        None. `pad` inflates every obstacle radius (e.g. for a via body
        wider than the track the margins were built for).

        The built path runs on the per-cell geometry packs: identical
        candidates, identical order, identical arithmetic (math.hypot
        kept -- numpy measured 3x SLOWER per call at ~14 candidates),
        just no per-candidate indexing/unpacking/call frames, and the
        push direction computed once for the winner instead of per
        improvement. Bit-identical results, ~2x fewer frames."""
        pk = getattr(self, '_pack', None)
        if pk is not None:
            px, py = p
            cell = pk.get((int(px / self.cell), int(py / self.cell)))
            if cell is None:
                return None
            discs, caps = cell
            hyp = math.hypot
            best = 0.0
            wdisc = wcap = None
            for x, y, r in discs:
                d = hyp(px - x, py - y)
                depth = r + pad - d
                if depth > best:
                    best = depth
                    wdisc = (x, y, d)
            for ax, ay, dx, dy, L2, r in caps:
                if L2 < 1e-12:
                    tt = 0.0
                else:
                    tt = ((px - ax) * dx + (py - ay) * dy) / L2
                    if tt < 0.0:
                        tt = 0.0
                    elif tt > 1.0:
                        tt = 1.0
                d = hyp(px - (ax + tt * dx), py - (ay + tt * dy))
                depth = r + pad - d
                if depth > best:
                    best = depth
                    wdisc = None
                    wcap = (ax, ay, dx, dy, L2, tt, d)
            if wcap is not None:
                ax, ay, dx, dy, L2, tt, d = wcap
                cx, cy = ax + tt * dx, ay + tt * dy
                if d < 1e-9:
                    dirv = (-dy / math.sqrt(L2), dx / math.sqrt(L2)) \
                        if L2 > 1e-12 else (1.0, 0.0)
                else:
                    dirv = ((px - cx) / d, (py - cy) / d)
                return (best, dirv)
            if wdisc is not None:
                x, y, d = wdisc
                dirv = (1.0, 0.0) if d < 1e-9 else \
                    ((px - x) / d, (py - y) / d)
                return (best, dirv)
            return None

    def seg_clear(self, a, b):
        # disc candidates from the grid, sampled along the segment
        # (sample step 0.9 < cell 1.0, so with the 3x3 neighbourhood
        # every cell the segment touches is covered); caps via a bbox
        # prefilter. Exact checks on the candidates only.
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        nsteps = max(1, int(L / 0.9))
        cand = set()
        for k in range(nsteps + 1):
            t = k / nsteps
            cand.update(self.near_discs((a[0] + t * (b[0] - a[0]),
                                         a[1] + t * (b[1] - a[1]))))
        for i in cand:
            x, y, r, _n = self.discs[i]
            if seg_pt_dist(a, b, (x, y)) < r:
                return False
        xlo, xhi = min(a[0], b[0]), max(a[0], b[0])
        ylo, yhi = min(a[1], b[1]), max(a[1], b[1])
        ccand = set()
        for k in range(nsteps + 1):
            t = k / nsteps
            ccand.update(self.near_caps((a[0] + t * (b[0] - a[0]),
                                         a[1] + t * (b[1] - a[1]))))
        for ci in ccand:
            c, e, r, _n = self.caps[ci]
            if min(c[0], e[0]) - r > xhi or max(c[0], e[0]) + r < xlo \
                    or min(c[1], e[1]) - r > yhi \
                    or max(c[1], e[1]) + r < ylo:
                continue
            if seg_seg_dist(a, b, c, e) < r:
                return False
        return True



def densify(pts, step=STEP):
    out = [pts[0]]
    for q in pts[1:]:
        p = out[-1]
        n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
        for k in range(1, n + 1):
            out.append((p[0] + (q[0] - p[0]) * k / n,
                        p[1] + (q[1] - p[1]) * k / n))
    return out


def shortcut(pts, obs):
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not obs.seg_clear(pts[i], pts[j]):
            j = (i + j) // 2 if j - i > 8 else j - 1
        # linear fallback ensures progress
        while j > i + 1 and not obs.seg_clear(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def relax(src, dst, obs, rounds=400):
    pts = densify([src, dst])
    ends = (src, dst)
    for it in range(rounds):
        moved = 0.0
        for i in range(1, len(pts) - 1):
            p = pts[i]
            if d2(p, ends[0]) < FREEZE ** 2 or d2(p, ends[1]) < FREEZE ** 2:
                continue
            q = (0.5 * p[0] + 0.25 * pts[i - 1][0] + 0.25 * pts[i + 1][0],
                 0.5 * p[1] + 0.25 * pts[i - 1][1] + 0.25 * pts[i + 1][1])
            for _k in range(6):
                v = obs.point_violation(q)
                if v is None:
                    break
                depth, (ux, uy) = v
                q = (q[0] + ux * (depth + 0.01), q[1] + uy * (depth + 0.01))
            moved += math.hypot(q[0] - p[0], q[1] - p[1])
            pts[i] = q
        if it % 25 == 24:
            pts = densify(shortcut(pts, obs))
        if moved < 1e-4 * len(pts):
            break
    return densify(shortcut(pts, obs)), it + 1








