#!/usr/bin/env python3
"""Vectorized region-join closest-approach is bit-identical to the brute force (#351).

`find_region_connection_points` measured closest approach between every region
pair with a pure-Python O(N_i x N_j) double loop over (anchors + subsampled fill
cells) -- the dominant cost of the region-join MST on big multi-region plane nets.
It now uses one numpy argmin over the row-major dist^2 matrix per pair.

The selection MUST be unchanged (the chosen points feed which copper the router
bridges): np.argmin returns the FIRST global minimum in (i-outer, j-inner) order,
exactly the brute force's strict-`<` first-minimum tie-break. This test pins that
equivalence against a verbatim copy of the old loop over randomized inputs
(including ties, empty regions, and a daisho-class perf case).

Run:  python3 tests/test_region_join_nearest_pair.py [-v]
"""
import argparse
import math
import os
import random
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from routing_config import GridCoord
import plane_region_connector as prc
from plane_region_connector import find_region_connection_points, UnionFind


def _reference(region_anchors, region_cells, coord):
    """Verbatim pre-#351 brute force, as the correctness oracle."""
    n = len(region_anchors)
    if n < 2:
        return []
    region_pts = []
    for i in range(n):
        pts = list(region_anchors[i])
        pts.extend(prc._subsample_cell_points(
            region_cells[i] if i < len(region_cells) else (), coord))
        region_pts.append(pts)
    edges = []
    for i in range(n):
        for j in range(i + 1, n):
            bd, bpi, bpj = float('inf'), None, None
            for pi in region_pts[i]:
                for pj in region_pts[j]:
                    d = math.sqrt((pi[0] - pj[0]) ** 2 + (pi[1] - pj[1]) ** 2)
                    if d < bd:
                        bd, bpi, bpj = d, pi, pj
            if bpi and bpj:
                edges.append((bd, i, j, bpi, bpj))
    edges.sort(key=lambda e: e[0])
    uf, mst = UnionFind(), []
    for d, i, j, pi, pj in edges:
        if not uf.connected(i, j):
            uf.union(i, j)
            mst.append((i, j, pi, pj, d))
            if len(mst) == n - 1:
                break
    return mst


def run(verbose=False):
    coord = GridCoord(0.05)
    random.seed(1234)
    for trial in range(200):
        n = random.randint(2, 6)
        ra, rc = [], []
        for _ in range(n):
            na = random.randint(0, 5)
            ra.append([(round(random.uniform(0, 50), 2), round(random.uniform(0, 50), 2))
                       for _ in range(na)])
            nc = random.randint(0, 60)
            rc.append(set((random.randint(0, 1000), random.randint(0, 1000))
                          for _ in range(nc)))
        assert find_region_connection_points(ra, rc, coord) == _reference(ra, rc, coord), \
            f"mismatch on random trial {trial}"

    # Deliberate ties: two regions with identical point sets.
    ra = [[(0.0, 0.0), (1.0, 1.0)], [(1.0, 1.0), (0.0, 0.0)]]
    assert find_region_connection_points(ra, [set(), set()], coord) == \
        _reference(ra, [set(), set()], coord), "tie case mismatch"

    # Empty regions contribute no edge (and must not crash).
    ra = [[], [(2.0, 3.0)], [(2.0, 4.0)]]
    assert find_region_connection_points(ra, [set(), set(), set()], coord) == \
        _reference(ra, [set(), set(), set()], coord), "empty-region mismatch"

    # < 2 regions -> no edges.
    assert find_region_connection_points([[(1.0, 1.0)]], [set()], coord) == []

    # The per-region interior-point memo (#351) must be transparent: cached
    # results identical to uncached, and _nearest_cell_points' sort must NOT
    # corrupt the cached list (it returns a fresh sorted copy).
    big = set((random.randint(0, 300), random.randint(0, 300)) for _ in range(8000))
    cache = {}
    base = prc._subsample_cell_points(big, coord, max_pts=400)
    assert prc._subsample_cell_points(big, coord, max_pts=400, interior_cache=cache) == base
    near = (5.0, 5.0)
    assert prc._nearest_cell_points(big, coord, near) == \
        prc._nearest_cell_points(big, coord, near, interior_cache=cache)
    assert prc._subsample_cell_points(big, coord, max_pts=400, interior_cache=cache) == base, \
        "cached interior list corrupted by _nearest_cell_points sort"

    # Perf/parity on a daisho-class input (many regions, dense anchors + cells).
    random.seed(7)
    n = 20
    ra = [[(round(random.uniform(0, 120), 3), round(random.uniform(0, 120), 3))
           for _ in range(120)] for _ in range(n)]
    rc = [set((random.randint(0, 2400), random.randint(0, 2400)) for _ in range(1500))
          for _ in range(n)]
    t = time.perf_counter()
    got = find_region_connection_points(ra, rc, coord)
    dt = time.perf_counter() - t
    assert got == _reference(ra, rc, coord), "daisho-class result differs from brute force"
    if verbose:
        print(f"  daisho-class n={n}: {dt * 1000:.1f} ms, {len(got)} MST edges")

    print("PASS: region-join nearest-pair is bit-identical to the brute force (#351)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    run(args.verbose)
