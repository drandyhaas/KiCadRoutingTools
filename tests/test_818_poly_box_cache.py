#!/usr/bin/env python3
"""#818: the polygon raster cache stores the BOX, not the gx/gy meshgrid.

`_POLY_RASTER_CACHE` used to hold four parallel per-cell arrays -- gx int32,
gy int32, inside bool, edist float64 = 17 bytes/cell. The first eight of those
bytes are a pure function of the raster's grid bounding box, so they were being
stored, evicted and re-derived for nothing. Measured on a glasgow route before
the change: 37,482 calls, 15,318 misses of which 15,299 (99.9%) were EVICTIONS,
against only 11,122 distinct keys -- i.e. 37.7% more rasterization than the
keyspace requires, out of a cache whose peak (133.6 MB) overshot its own
102.4 MB budget.

The cache now stores `(gx_lo, gy_lo, nx, ny, inside, edist)` at 9 bytes/cell and
consumers derive the coordinates of their MASKED cells with `_box_masked_cells`.
That is exact -- integer arithmetic over the stored box, no float involved -- so
this file asserts identity rather than tolerance. What is easy to get wrong and
is therefore pinned here:

  * the flattening ORDER. `np.meshgrid(gx_range, gy_range)` is (ny, nx) with gx
    FASTEST, so cell i is `(gx_lo + i % nx, gy_lo + i // nx)`. Transposing that
    silently mislocates every blocked cell on a non-square raster, which a
    square test fixture would never catch -- so the polygons below are
    deliberately non-square and the box arithmetic is checked against the
    meshgrid it replaced.
  * NEGATIVE grid coordinates. `//` and `%` on a negative gx_lo are only correct
    because the offset is added AFTER the divmod of a non-negative index; doing
    the arithmetic on absolute coordinates would floor the wrong way.
  * the cache must not change any answer. A repeat call is served from the
    OrderedDict, so every case runs twice and the second result must be
    elementwise identical to the first (and, for the arrays, read-only).

    python3 tests/test_818_poly_box_cache.py
"""
import math
import os
import random
import sys

_R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_R, os.path.join(_R, 'py_router'), os.path.join(_R, 'rust_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

import obstacle_map as OM
from obstacle_map import (_rasterize_polygon, _rasterize_polygon_box,
                          _box_masked_cells, _box_full_cells)
from routing_config import GridCoord

FAILS = []


def check(ok, msg):
    if ok:
        print('  PASS: %s' % msg)
    else:
        print('  FAIL: %s' % msg)
        FAILS.append(msg)


def _poly(cx, cy, rx, ry, n, jitter, rng):
    """A deliberately NON-square polygon (rx != ry) about (cx, cy)."""
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n + rng.uniform(-jitter, jitter)
        f = rng.uniform(0.45, 1.0)
        pts.append((cx + f * rx * math.cos(a), cy + f * ry * math.sin(a)))
    return pts


def test_box_matches_meshgrid():
    """The box form and the compat wrapper describe the same cells."""
    rng = random.Random(818)
    mismatched = 0
    non_square = 0
    negative = 0
    trials = 300
    for _ in range(trials):
        # Half the polygons sit at negative board coordinates so gx_lo/gy_lo go
        # negative -- the case the divmod arithmetic must get right.
        neg = rng.random() < 0.5
        cx = rng.uniform(-60.0, -5.0) if neg else rng.uniform(5.0, 60.0)
        cy = rng.uniform(-60.0, -5.0) if neg else rng.uniform(5.0, 60.0)
        pts = _poly(cx, cy, rng.uniform(0.4, 12.0), rng.uniform(0.4, 12.0),
                    rng.randint(3, 9), 0.25, rng)
        coord = GridCoord(rng.choice([0.05, 0.1, 0.127, 0.2]))
        margin = rng.uniform(0.0, 1.2)
        clip = None
        if rng.random() < 0.4:
            clip = (cx - rng.uniform(0, 9), cy - rng.uniform(0, 9),
                    cx + rng.uniform(0, 9), cy + rng.uniform(0, 9))

        gx_lo, gy_lo, nx, ny, inside, edist = _rasterize_polygon_box(
            pts, coord, margin, clip_bounds=clip)
        gx_flat, gy_flat, w_inside, w_edist = _rasterize_polygon(
            pts, coord, margin, clip_bounds=clip)
        if inside is None:
            if gx_flat is not None:
                mismatched += 1
            continue
        if nx != ny:
            non_square += 1
        if gx_lo < 0 or gy_lo < 0:
            negative += 1

        if (nx * ny != inside.size or inside.size != edist.size
                or not np.array_equal(inside, w_inside)
                or not np.array_equal(edist, w_edist, equal_nan=True)):
            mismatched += 1
            continue
        # the full meshgrid the box implies
        bgx, bgy = _box_full_cells(gx_lo, gy_lo, nx, ny)
        if not (np.array_equal(bgx, gx_flat) and np.array_equal(bgy, gy_flat)):
            mismatched += 1
            continue
        # ... and the masked subset every consumer actually asks for
        mask = inside | (edist <= margin)
        mgx, mgy = _box_masked_cells(gx_lo, gy_lo, nx, mask)
        if not (np.array_equal(mgx, gx_flat[mask])
                and np.array_equal(mgy, gy_flat[mask])):
            mismatched += 1

    check(mismatched == 0,
          'box == meshgrid over %d random rasters (%d non-square, %d at '
          'negative grid coords), 0 mismatches' % (trials, non_square, negative))
    check(non_square >= trials // 4 and negative >= trials // 4,
          'the trial set actually exercised non-square and negative rasters')


def test_masked_cells_negative_and_order():
    """Explicit, hand-checkable divmod cases -- the random sweep above would
    still pass if BOTH sides were transposed the same way."""
    nx, ny = 4, 3        # deliberately nx != ny
    gx_lo, gy_lo = -7, -2
    mask = np.zeros(nx * ny, dtype=bool)
    # flat index 0 -> (gx_lo, gy_lo); 3 -> (gx_lo+3, gy_lo); 4 -> (gx_lo, gy_lo+1)
    for i in (0, 3, 4, 11):
        mask[i] = True
    gx, gy = _box_masked_cells(gx_lo, gy_lo, nx, mask)
    want = [(-7, -2), (-4, -2), (-7, -1), (-4, 0)]
    got = list(zip(gx.tolist(), gy.tolist()))
    check(got == want, 'divmod order + negative origin: %s' % (got,))
    check(gx.dtype == np.int32 and gy.dtype == np.int32,
          'masked cells are int32 (the batch APIs require it)')
    check(_box_masked_cells(gx_lo, gy_lo, nx,
                            np.zeros(nx * ny, dtype=bool))[0].size == 0,
          'an all-False mask yields no cells')


def test_cache_hit_is_identical_and_readonly():
    rng = random.Random(4242)
    coord = GridCoord(0.1)
    pts = _poly(11.0, -4.0, 3.0, 7.0, 6, 0.0, rng)   # rx != ry
    a = _rasterize_polygon_box(pts, coord, 0.3)
    b = _rasterize_polygon_box(pts, coord, 0.3)
    check(a[:4] == b[:4], 'cache hit returns the same box')
    check(a[4] is b[4] and a[5] is b[5],
          'cache hit returns the SAME arrays (shared, not recomputed)')
    check(not a[4].flags.writeable and not a[5].flags.writeable,
          'cached arrays are read-only (consumers share them)')
    # a different margin is a different key
    c = _rasterize_polygon_box(pts, coord, 0.7)
    check(c[4] is not a[4], 'margin participates in the cache key')


def test_bytes_per_cell_accounting():
    """The byte counter must match what is actually stored, or the LRU either
    thrashes or blows the budget."""
    check(OM._POLY_CELL_BYTES == 9,
          '_POLY_CELL_BYTES == 9 (inside bool + edist float64)')
    OM._POLY_RASTER_CACHE.clear()
    OM._POLY_RASTER_BYTES = 0
    rng = random.Random(7)
    coord = GridCoord(0.1)
    for i in range(12):
        _rasterize_polygon_box(_poly(200.0 + 40 * i, 5.0, 2.0, 5.0, 5, 0.0, rng),
                               coord, 0.2)
    live = sum(e[4].size for e in OM._POLY_RASTER_CACHE.values()
               if e[4] is not None)
    check(OM._POLY_RASTER_BYTES == live * OM._POLY_CELL_BYTES,
          'byte counter == sum of live entries (%d cells)' % live)
    check(OM._POLY_RASTER_BYTES == sum(
              e[4].nbytes + e[5].nbytes for e in OM._POLY_RASTER_CACHE.values()
              if e[4] is not None),
          'and == the real nbytes of the stored arrays')


def test_eviction_is_lru_not_wholesale():
    """#818 filed a correction: this cache has been LRU since #546 -- a stale
    block comment said 'wholesale clear on overflow'. Pin the behaviour so the
    comment can never drift back."""
    OM._POLY_RASTER_CACHE.clear()
    OM._POLY_RASTER_BYTES = 0
    rng = random.Random(9)
    coord = GridCoord(0.05)
    polys = [_poly(400.0 + 60 * i, 5.0, 6.0, 14.0, 5, 0.0, rng) for i in range(6)]
    saved = OM.env_knobs.RASTER_CACHE_MB
    try:
        for p in polys[:3]:
            _rasterize_polygon_box(p, coord, 0.4)
        n_before = len(OM._POLY_RASTER_CACHE)
        # squeeze the budget so the next insert must evict, then confirm SOME
        # entries survive (a wholesale clear would leave exactly the new one)
        # Derive the squeeze from the budget FUNCTION, never from a hardcoded
        # share: the #815 rebalance moved the polygon slice 40% -> 30% and a
        # literal 0.4 here silently squeezed to the wrong size (this assertion
        # failed for that reason, not for a behaviour change).
        _share = OM._poly_raster_byte_budget() / (OM.env_knobs.RASTER_CACHE_MB * 1e6)
        OM.env_knobs.RASTER_CACHE_MB = (OM._POLY_RASTER_BYTES / 1e6) / _share * 0.75
        _rasterize_polygon_box(polys[3], coord, 0.4)
        n_after = len(OM._POLY_RASTER_CACHE)
        check(n_before >= 3, 'three distinct rasters cached (%d)' % n_before)
        check(1 < n_after < n_before + 1,
              'overflow evicted LRU-wise, keeping %d of %d entries (not a '
              'wholesale clear)' % (n_after, n_before))
        check(OM._POLY_RASTER_BYTES <= OM._poly_raster_byte_budget(),
              'the counter is back under budget after eviction')
    finally:
        OM.env_knobs.RASTER_CACHE_MB = saved
        OM._POLY_RASTER_CACHE.clear()
        OM._POLY_RASTER_BYTES = 0


def main():
    print('#818 polygon raster cache: box storage')
    for t in (test_box_matches_meshgrid, test_masked_cells_negative_and_order,
              test_cache_hit_is_identical_and_readonly,
              test_bytes_per_cell_accounting, test_eviction_is_lru_not_wholesale):
        print('\n%s:' % t.__name__)
        t()
    print('')
    if FAILS:
        print('FAILED (%d): %s' % (len(FAILS), '; '.join(FAILS)))
        return 1
    print('All #818 polygon box-cache tests passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
