#!/usr/bin/env python3
"""grid_router geometry kernels vs the Python they replace, bit for bit.

* capsule_spans / capsule_cells == the pre-kernel numpy bodies of
  routing_utils.segment_blocked_spans / segment_blocked_cells_array
* point_segment_distances  == geometry_utils.point_to_segment_distance
* poly_keepout_cells       == the pre-kernel _Occ.block_poly cell loop

Randomized, seeded, thousands of cases each, with degenerate segments
(zero and sub-1e-5 length), repeated / collinear polygon points, vertices
on lattice nodes and radii on whole cells so threshold ties are exercised.
"""
import math
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import numpy as np

from grid_router import point_segment_distances, capsule_spans, capsule_cells
from routing_utils import (_capsule_mask, GRID_TIE_EPS, segment_blocked_spans,
                           segment_blocked_cells_array)
from geometry_utils import point_to_segment_distance
from check_drc import _point_in_poly
from bga_fanout.underpad import _Occ


FAILS = []


def _ok(name, cond, detail=''):
    print(('PASS ' if cond else 'FAIL ') + name + (('  ' + detail) if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _old_spans(x1, y1, x2, y2, margin, step):
    """segment_blocked_spans' body before the grid_router kernel, verbatim."""
    xs, ys, _gxg, _gyg, mask = _capsule_mask(x1, y1, x2, y2, margin, step)
    sel = np.flatnonzero(mask.any(axis=1))
    if not sel.size:
        return np.empty((0, 3), dtype=np.int32)
    lo = mask.argmax(axis=1)[sel]
    hi = mask.shape[1] - 1 - mask[:, ::-1].argmax(axis=1)[sel]
    out = np.empty((sel.size, 3), dtype=np.int32)
    out[:, 0] = xs[sel]
    out[:, 1] = ys[lo]
    out[:, 2] = ys[hi]
    assert int((out[:, 2] - out[:, 1] + 1).sum()) == int(mask.sum())
    return out


def _old_cells(x1, y1, x2, y2, margin, step):
    _xs, _ys, gxg, gyg, mask = _capsule_mask(x1, y1, x2, y2, margin, step)
    out = np.empty((int(mask.sum()), 2), dtype=np.int32)
    out[:, 0] = gxg[mask]
    out[:, 1] = gyg[mask]
    return out


def test_capsule():
    rng = random.Random(81600)
    cases = 20000
    bad_s = bad_c = 0
    rows = 0
    for c in range(cases):
        step = rng.choice([0.025, 0.025, 0.05, 0.1, 0.0125, 0.02])
        def co():
            k = rng.random()
            if k < 0.35:
                return rng.randint(-4000, 4000) * step             # on a lattice node
            if k < 0.45:
                return rng.randint(-4000, 4000) * step + step / 2   # half-cell
            return rng.uniform(-100, 100)
        x1, y1 = co(), co()
        k = rng.random()
        if k < 0.15:
            x2, y2 = x1, y1                                       # zero length
        elif k < 0.3:
            x2, y2 = x1 + rng.choice([-1, 0, 1]) * rng.uniform(0, 2), y1
        elif k < 0.4:
            d = rng.uniform(-2, 2)
            x2, y2 = x1 + d, y1 + d                               # 45 degrees
        else:
            x2, y2 = x1 + rng.uniform(-3, 3), y1 + rng.uniform(-3, 3)
        margin = rng.choice([rng.randint(1, 12) * step,            # whole cells: tie cases
                             rng.randint(1, 12) * step + GRID_TIE_EPS,
                             rng.uniform(0, 0.5), 0.0, GRID_TIE_EPS / 2])
        want_s = _old_spans(x1, y1, x2, y2, margin, step)
        got_s = capsule_spans(x1, y1, x2, y2, margin, step, GRID_TIE_EPS)
        if got_s.dtype != np.int32 or got_s.shape != want_s.shape or not np.array_equal(got_s, want_s):
            bad_s += 1
        want_c = _old_cells(x1, y1, x2, y2, margin, step)
        got_c = capsule_cells(x1, y1, x2, y2, margin, step, GRID_TIE_EPS)
        if got_c.dtype != np.int32 or got_c.shape != want_c.shape or not np.array_equal(got_c, want_c):
            bad_c += 1
        rows += len(want_c)
    _ok('capsule_spans == old segment_blocked_spans body (%d capsules)' % cases, bad_s == 0,
        '%d differ' % bad_s)
    _ok('capsule_cells == old segment_blocked_cells_array body (%d cells)' % rows,
        bad_c == 0 and rows > 0, '%d differ' % bad_c)
    # the public cached forms go through the kernel and stay read-only
    a = segment_blocked_spans(0.013, 0.021, 1.234, 0.987, 0.2, 0.025)
    b = segment_blocked_cells_array(0.013, 0.021, 1.234, 0.987, 0.2, 0.025)
    _ok('public capsule forms read-only and consistent',
        not a.flags.writeable and not b.flags.writeable
        and int((a[:, 2] - a[:, 1] + 1).sum()) == len(b))


def _coord(rng):
    k = rng.random()
    if k < 0.3:
        return round(rng.uniform(-5, 5) / 0.025) * 0.025      # lattice node
    if k < 0.4:
        return rng.choice([0.0, 1e-12, -1e-12, 1e-6])
    return rng.uniform(-5, 5)


def _segment(rng):
    x1, y1 = _coord(rng), _coord(rng)
    k = rng.random()
    if k < 0.15:
        return x1, y1, x1, y1                                  # zero length
    if k < 0.3:
        e = rng.choice([1e-6, 3e-6, 9.9e-6, 1.0000001e-5, 2e-5])
        a = rng.uniform(0, 2 * math.pi)
        return x1, y1, x1 + e * math.cos(a), y1 + e * math.sin(a)
    if k < 0.4:
        return x1, y1, x1, _coord(rng)                         # axis-aligned
    return x1, y1, _coord(rng), _coord(rng)


def test_point_segment():
    rng = random.Random(20261118)
    n = 20000
    P = np.empty((n, 2))
    S = np.empty((n, 4))
    for i in range(n):
        S[i] = _segment(rng)
        if rng.random() < 0.2:                                 # on an endpoint / the line
            t = rng.choice([0.0, 1.0, 0.5, rng.random()])
            P[i] = (S[i, 0] + t * (S[i, 2] - S[i, 0]), S[i, 1] + t * (S[i, 3] - S[i, 1]))
        else:
            P[i] = (_coord(rng), _coord(rng))
    got = point_segment_distances(P, S)
    bad = 0
    for i in range(n):
        want = point_to_segment_distance(P[i, 0], P[i, 1], *S[i])
        if got[i] != want and not (math.isnan(got[i]) and math.isnan(want)):
            bad += 1
    _ok('point_segment_distances bit-identical (%d cases)' % n, bad == 0, '%d differ' % bad)


def _old_block_poly(self, polys, r, li=None):
    """_Occ.block_poly before the grid_router kernel, verbatim."""
    pts = [pt for poly in polys for pt in poly]
    if not pts:
        return
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ia, ja = self.cell(min(xs) - r, min(ys) - r)
    ib, jb = self.cell(max(xs) + r, max(ys) + r)
    thr = r + 1e-9
    edges = []
    for poly in polys:
        n = len(poly)
        e = []
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            e.append((min(x1, x2) - thr, max(x1, x2) + thr,
                      min(y1, y2) - thr, max(y1, y2) + thr,
                      x1, y1, x2, y2))
        edges.append((poly, e))

    def _within(x, y):
        for poly, e in edges:
            if _point_in_poly(x, y, poly):
                return True
            for (bx0, bx1e, by0, by1e, x1, y1, x2, y2) in e:
                if x < bx0 or x > bx1e or y < by0 or y > by1e:
                    continue
                if point_to_segment_distance(x, y, x1, y1, x2, y2) <= thr:
                    return True
        return False
    for a in range(max(0, ia), min(self.nx, ib + 1)):
        for b in range(max(0, ja), min(self.ny, jb + 1)):
            x, y = self.xy(a, b)
            if _within(x, y):
                if li is None:
                    for k in range(self.nl):
                        self.grid[k][a * self.ny + b] = 1
                else:
                    self.grid[li][a * self.ny + b] = 1


def _poly(rng, cx, cy):
    k = rng.random()
    n = rng.choice([1, 2, 3, 3, 4, 4, 5, 6, 8, 12])
    if k < 0.25:                                               # lattice vertices
        pts = [(cx + rng.randint(-8, 8) * 0.025, cy + rng.randint(-8, 8) * 0.025) for _ in range(n)]
    else:
        pts = [(cx + rng.uniform(-0.3, 0.3), cy + rng.uniform(-0.3, 0.3)) for _ in range(n)]
    if n > 1 and rng.random() < 0.2:                           # repeated point
        i = rng.randrange(n)
        pts.insert(i, pts[i])
    if rng.random() < 0.1:                                     # collinear run
        x, y = pts[0]
        pts = [(x + i * 0.025, y) for i in range(n)] + pts[1:]
    if rng.random() < 0.05:                                    # sub-1e-5 edge
        x, y = pts[0]
        pts.insert(1, (x + 3e-6, y - 2e-6))
    return pts


def test_block_poly():
    rng = random.Random(4604866)
    cases = 3000
    bad = 0
    cells = 0
    for c in range(cases):
        res = rng.choice([0.025, 0.025, 0.05, 0.0125])
        x0 = rng.choice([0.0, 10.0, rng.uniform(-3, 3)])
        y0 = rng.choice([0.0, -7.5, rng.uniform(-3, 3)])
        bounds = (x0, y0, x0 + rng.uniform(0.5, 1.6), y0 + rng.uniform(0.5, 1.6))
        layers = ['F.Cu', 'In1.Cu', 'B.Cu'][:rng.randint(1, 3)]
        old = _Occ(bounds, res, layers)
        new = _Occ(bounds, res, layers)
        if rng.random() < 0.3:
            old.CELL_EPS = new.CELL_EPS = _Occ.JOINT_CELL_EPS
        cx = rng.uniform(bounds[0] - 0.2, bounds[2] + 0.2)
        cy = rng.uniform(bounds[1] - 0.2, bounds[3] + 0.2)
        polys = [_poly(rng, cx + rng.uniform(-0.2, 0.2), cy + rng.uniform(-0.2, 0.2))
                 for _ in range(rng.choice([1, 1, 2, 3]))]
        r = rng.choice([0.0, res, 2 * res, 3 * res, rng.uniform(0, 0.2)])
        li = None if rng.random() < 0.5 else rng.randrange(len(layers))
        _old_block_poly(old, polys, r, li)
        new.block_poly(polys, r, li)
        if old.grid != new.grid:
            bad += 1
        cells += sum(sum(g) for g in old.grid)
    _ok('poly_keepout_cells == old block_poly (%d cases, %d cells set)' % (cases, cells),
        bad == 0 and cells > 0, '%d differ' % bad)
    g = _Occ((0, 0, 1, 1), 0.025, ['F.Cu'])
    g.block_poly([], 0.1)
    _ok('block_poly no polygons is a no-op', not any(g.grid[0]))


if __name__ == '__main__':
    test_capsule()
    test_point_segment()
    test_block_poly()
    print('ALL PASS' if not FAILS else '%d FAILED' % len(FAILS))
    sys.exit(1 if FAILS else 0)
