#!/usr/bin/env python3
"""py_router/constrained_triangles.py: constrained Delaunay triangles from shapely 2.0 on (the bus step's 2-D
embedding, awx/whole_embed.py, cuts each layer's free space into them).

  python3 tests/test_constrained_triangles.py

1. here they can be made (unavailable() is None);
2. on polygons with holes -- some touching the window's edge, some overlapping -- the triangles cover the polygon
   (their areas sum to its area) and keep to it (none reaches outside it or into a hole), by shapely's own function where
   this shapely has it and by the direct GEOS call either way;
3. where shapely has its own function, the direct GEOS call gives exactly its triangles.
"""
import math
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import shapely  # noqa: E402
from shapely.geometry import box, LineString, Point  # noqa: E402
from shapely.ops import unary_union  # noqa: E402
import constrained_triangles as ct  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f" -- {detail}" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def polys(n=12, seed=3):
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        W = rng.uniform(6, 20)
        holes = []
        for _ in range(rng.randint(3, 30)):
            x, y = rng.uniform(-1, W + 1), rng.uniform(-1, W + 1)
            if rng.random() < 0.5:
                holes.append(Point(x, y).buffer(rng.uniform(0.1, 0.5), 8))
            else:
                a, L = rng.uniform(0, 2 * math.pi), rng.uniform(0.5, 5)
                holes.append(LineString([(x, y), (x + L * math.cos(a), y + L * math.sin(a))]).buffer(0.15, 4))
        g = box(0, 0, W, W).difference(unary_union(holes))
        if not g.is_empty:
            out.append(g)
    return out


def tri_set(gc):
    return {tuple(sorted((round(x, 9), round(y, 9)) for x, y in list(t.exterior.coords)[:3])) for t in gc.geoms}


def main():
    print(f"shapely {shapely.__version__}, GEOS {shapely.geos_version_string}")
    check('1. constrained triangles can be made here', ct.unavailable() is None, ct.unavailable())
    ps = polys()
    ways = [('the direct GEOS call', ct._geos_triangles)]
    own = hasattr(shapely, 'constrained_delaunay_triangles')
    if own:
        ways.insert(0, ("shapely's own", shapely.constrained_delaunay_triangles))
    print('2. the triangles cover the polygon and keep to it')
    for label, fn in ways:
        worst_area = worst_out = 0.0
        for g in ps:
            gc = fn(g)
            area = sum(t.area for t in gc.geoms)
            worst_area = max(worst_area, abs(area - g.area) / g.area)
            worst_out = max(worst_out, max(t.difference(g).area for t in gc.geoms))
        check(f"{label}: areas sum to the polygon's (worst {worst_area:.1e} relative)", worst_area < 1e-9)
        check(f"{label}: none reaches outside it or into a hole (worst {worst_out:.1e} mm^2)", worst_out < 1e-9)
    if own:
        print("3. the direct GEOS call gives shapely's own triangles")
        same = sum(1 for g in ps if tri_set(shapely.constrained_delaunay_triangles(g)) == tri_set(ct._geos_triangles(g)))
        check(f"identical on {same} of {len(ps)} polygons", same == len(ps))
    else:
        print("3. (this shapely has no function of its own: nothing to compare)")
    print(f"\n{'FAIL: ' + '; '.join(FAILS) if FAILS else 'PASS: constrained triangles'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
