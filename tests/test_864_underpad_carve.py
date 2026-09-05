#!/usr/bin/env python3
"""#864: the under-pad escape's foreign-copper carve, at the _Occ level.

Three things the fix relies on, pinned without routing a board:

  1. `_Occ.disk_cells(x, y, r)` is exactly `frozenset(_disk(x, y, r))` and is
     memoised (the same object comes back for the same key), so the eight
     rescue attempts of one net rebuild each foreign disk once, not eight
     times.
  2. The carve's segment sampling -- disks of radius sqrt(keep^2 + (s/2)^2)
     every s = max(res, keep/2) along a track -- covers EVERY cell within
     `keep` of the segment (the old per-cell sampling's union), on random
     segments. The carve may over-block; it must never under-block.
  3. The occupancy grid resolution is floored at 0.01 mm: a grid mis-detected
     at 0.10 mm pitch (system76_launch J7) got pitch/32 = 0.003 mm, on which
     a 7 mm mounting-pad keep-out was a 5-million-cell disk and 264 of them
     were 397 of a 433 s rescue. The floor leaves real pitches alone
     (0.35 mm -> 0.011).
"""
import math
import os
import random
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from bga_fanout.underpad import _Occ  # noqa: E402


def _capsule_cells_exact(occ, x1, y1, x2, y2, keep):
    """Every lattice point within `keep` of the segment (the reference)."""
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    xmin, xmax = min(x1, x2) - keep, max(x1, x2) + keep
    ymin, ymax = min(y1, y2) - keep, max(y1, y2) + keep
    out = set()
    for a in range(int((xmin - occ.x0) / occ.res) - 2, int((xmax - occ.x0) / occ.res) + 3):
        for b in range(int((ymin - occ.y0) / occ.res) - 2, int((ymax - occ.y0) / occ.res) + 3):
            if not occ.inside(a, b):
                continue
            px, py = occ.xy(a, b)
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
            d2 = (px - (x1 + t * dx)) ** 2 + (py - (y1 + t * dy)) ** 2
            if d2 <= keep * keep:
                out.add((a, b))
    return out


def _capsule_cells_sampled(occ, x1, y1, x2, y2, keep):
    """The carve's sampling (mirrors _carve_foreign.add_seg)."""
    dx, dy = x2 - x1, y2 - y1
    slen = math.hypot(dx, dy)
    step = max(occ.res, keep / 2.0)
    n = max(1, int(slen / step) + 1)
    r_eff = math.sqrt(keep * keep + (slen / n / 2.0) ** 2)
    cells = set()
    for i in range(n + 1):
        t = i / n
        cells |= occ.disk_cells(x1 + dx * t, y1 + dy * t, r_eff)
    return cells


def main():
    fails = []
    occ = _Occ((0.0, 0.0, 10.0, 10.0), 0.025, ['F.Cu', 'B.Cu'])
    # 1. memo == _disk, same object on a repeat
    for (x, y, r) in ((5.0, 5.0, 0.4), (2.499, 7.0001, 0.75), (9.9, 0.1, 1.2)):
        a = frozenset(occ._disk(x, y, r))
        b = occ.disk_cells(x, y, r)
        if a != b:
            fails.append(f"disk_cells({x},{y},{r}) != _disk: {len(a)} vs {len(b)} cells")
        if occ.disk_cells(x, y, r) is not b:
            fails.append(f"disk_cells({x},{y},{r}) was rebuilt instead of memoised")
    # 2. sampled capsule covers the exact capsule, on random segments
    rng = random.Random(864)
    worst = 0.0
    for _ in range(60):
        x1, y1 = rng.uniform(1, 9), rng.uniform(1, 9)
        ang, L = rng.uniform(0, math.pi), rng.uniform(0.05, 4.0)
        x2, y2 = x1 + L * math.cos(ang), y1 + L * math.sin(ang)
        x2, y2 = max(1.0, min(9.0, x2)), max(1.0, min(9.0, y2))
        keep = rng.choice((0.12, 0.2, 0.33, 0.5, 0.8))
        exact = _capsule_cells_exact(occ, x1, y1, x2, y2, keep)
        got = _capsule_cells_sampled(occ, x1, y1, x2, y2, keep)
        missing = exact - got
        if missing:
            fails.append(f"segment ({x1:.3f},{y1:.3f})-({x2:.3f},{y2:.3f}) keep {keep}: "
                         f"{len(missing)} cell(s) within keep are NOT carved, e.g. {sorted(missing)[:3]}")
            break
        worst = max(worst, (len(got) - len(exact)) / max(1, len(exact)))
    if worst > 0.12:
        fails.append(f"the sampled capsule over-blocks by {worst:.0%} (> 12%) -- the inflation is too generous")
    # 3. the resolution floor (read straight from the source, one expression)
    src = open(os.path.join(ROOT, 'py_router', 'bga_fanout', 'underpad.py'), encoding='utf-8').read()
    if not re.search(r"res = max\(min\(grid\.pitch_x, grid\.pitch_y\) / 32\.0, 0\.01\)", src):
        fails.append("the occupancy grid resolution is no longer floored at 0.01 mm (pitch/32 alone)")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print(f"PASS: disk_cells == _disk and memoised; the sampled capsule covers every cell within "
          f"keep on 60 random segments (worst over-block {worst:.1%}); the grid resolution is floored at 0.01 mm")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
