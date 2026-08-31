#!/usr/bin/env python3
"""#816: the capsule boundary is resolved OPEN by a 1 nm epsilon, so a capsule's
cell set no longer depends on where it sits on the board.

The bug this fixes is not a wrong answer -- it is an ARBITRARY one. Cell centres
were compared to the segment in ABSOLUTE board coordinates (`gx * grid_step`),
and `grid_step` has no exact binary form, so identical geometry rounded
differently at different positions. Wherever a cell sat EXACTLY on the boundary
the strict `<` was then decided by the last bit. That is not a corner case: the
repo's own defaults land there (track 0.3/2 + clearance 0.25 = 0.4 mm = exactly
4 x a 0.1 mm grid), and ~20% of realistic track/clearance/grid combinations do.

Measured before the epsilon, at those defaults:

  * the IDENTICAL capsule shape produced up to 25 different cell sets across 435
    board positions -- 115 to 138 cells for one shape;
  * 81.5% of capsules disagreed between the absolute and a segment-relative
    rasterization, worst case 63 cells.

Measured after: 1 cell set per shape, and 0 of 692,342 cells disagree.

Why OPEN and not BLOCKED, and why 1 nm -- both ends measured:

  float noise to swamp, over a 600 mm board :     0.000034 nm   (29,000x smaller)
  epsilon                                   :     1 nm          (= KiCad's own unit)
  nearest GENUINE non-tie cell to the edge  :     1,498 nm      (1,498x larger)

so it resolves every tie deterministically and can never open a cell that is
really inside. Opening is the safe direction: the resulting gap is exactly the
clearance, which DRC passes, and KiCad stores nanometre integers so a sub-nm
violation is not representable.

This property is what makes shape-keyed memoization legal (#816): the memo can
only collapse translations if translated copies genuinely agree.

    python3 tests/test_816_capsule_tie_epsilon.py
"""
import collections
import math
import os
import random
import sys

_R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_R, os.path.join(_R, 'py_router'), os.path.join(_R, 'rust_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

import routing_utils as RU
import obstacle_map as OM
from routing_config import GridCoord
from routing_utils import _capsule_mask, GRID_TIE_EPS

FAILS = []


def check(ok, msg):
    print(('  PASS: ' if ok else '  FAIL: ') + msg)
    if not ok:
        FAILS.append(msg)


def cells(x1, y1, x2, y2, margin, step):
    _xs, _ys, gxg, gyg, mask = _capsule_mask(x1, y1, x2, y2, margin, step)
    return set(zip(gxg[mask].tolist(), gyg[mask].tolist()))


def shape_set(gx0, gy0, ldx, ldy, margin, step):
    """The cell set normalised back to the shape frame, so two positions are
    directly comparable."""
    c = cells(gx0 * step, gy0 * step,
              (gx0 + ldx) * step, (gy0 + ldy) * step, margin, step)
    return frozenset((gx - gx0, gy - gy0) for gx, gy in c)


# The exact-tie configuration: margin is an integer multiple of the grid.
TIE_STEP = 0.1
TIE_MARGIN = 0.3 / 2 + 0.25          # 0.4 == 4 * TIE_STEP


def test_margin_is_actually_a_tie():
    """Guard the FIXTURE. If defaults ever move off the tie, this file silently
    stops testing anything -- the failure mode that makes a gate vacuous."""
    q = TIE_MARGIN / TIE_STEP
    check(abs(q - round(q)) < 1e-9,
          'fixture really is an exact tie: margin %.4g = %g x step'
          % (TIE_MARGIN, q))
    import routing_defaults as RD
    check(abs((RD.TRACK_WIDTH / 2 + RD.CLEARANCE) - TIE_MARGIN) < 1e-12,
          'and it is the repo DEFAULT track/clearance (%.4g / %.4g)'
          % (RD.TRACK_WIDTH, RD.CLEARANCE))


def test_position_independence():
    worst = 0
    detail = []
    for ldx, ldy in ((10, 0), (0, 10), (7, 7), (12, 5), (0, 0), (20, 3)):
        seen = collections.Counter()
        for gx0 in range(-1000, 1001, 23):
            for gy0 in (-777, -13, 0, 41, 908):
                seen[shape_set(gx0, gy0, ldx, ldy, TIE_MARGIN, TIE_STEP)] += 1
        worst = max(worst, len(seen))
        if len(seen) != 1:
            detail.append('%s -> %d sets' % ((ldx, ldy), len(seen)))
    check(worst == 1,
          'one cell set per shape over 435 board positions each '
          '(worst %d)%s' % (worst, '; ' + ', '.join(detail) if detail else ''))


def test_relative_frame_is_now_bit_safe():
    """The whole point: a segment-relative rasterization plus an INTEGER shift
    must reproduce the absolute one exactly, or shape-keyed caching is unsound."""
    def rel(x1, y1, x2, y2, margin, step, quant=None):
        inv = 1.0 / step
        g0x = int(math.floor(x1 * inv)); g0y = int(math.floor(y1 * inv))
        fx = x1 - g0x * step; fy = y1 - g0y * step
        dx = x2 - x1; dy = y2 - y1
        if quant is not None:
            fx = round(fx, quant); fy = round(fy, quant)
            dx = round(dx, quant); dy = round(dy, quant)
        _xs, _ys, gxg, gyg, mask = _capsule_mask(fx, fy, fx + dx, fy + dy,
                                                 margin, step)
        return set(zip((gxg[mask] + g0x).tolist(), (gyg[mask] + g0y).tolist()))

    rng = random.Random(816)
    for quant, label in ((None, 'exact float'), (9, '1e-9 mm quantized')):
        bad = badcells = tot = 0
        for _ in range(1500):
            gx0 = rng.randint(-1500, 1500); gy0 = rng.randint(-1500, 1500)
            ldx = rng.randint(-30, 30); ldy = rng.randint(-30, 30)
            x1, y1 = gx0 * TIE_STEP, gy0 * TIE_STEP
            x2, y2 = (gx0 + ldx) * TIE_STEP, (gy0 + ldy) * TIE_STEP
            a = cells(x1, y1, x2, y2, TIE_MARGIN, TIE_STEP)
            b = rel(x1, y1, x2, y2, TIE_MARGIN, TIE_STEP, quant)
            tot += len(a)
            d = len(a ^ b)
            if d:
                bad += 1; badcells += d
        check(bad == 0,
              'segment-relative frame (%s) reproduces absolute exactly: '
              '%d/1500 capsules, %d/%d cells differ' % (label, bad, badcells, tot))


def test_epsilon_headroom():
    """Both sides of the epsilon, so a future edit cannot quietly move it into
    a range where it either fails to resolve ties or opens real cells."""
    check(GRID_TIE_EPS == 1e-6,
          'epsilon is 1 nm (1e-6 mm), KiCad\'s own resolution')
    # (a) it dwarfs the float noise it must swamp
    worst_noise = 0.0
    for gy0 in range(-3000, 3001):
        d = (gy0 + 4) * TIE_STEP - gy0 * TIE_STEP
        worst_noise = max(worst_noise, abs(d - TIE_MARGIN))
    check(worst_noise < GRID_TIE_EPS / 100,
          'float noise %.2e mm is >100x below the epsilon' % worst_noise)
    # (b) it is far below the nearest cell that genuinely sits inside
    best = 1e9
    for st in (0.05, 0.1, 0.2):
        for m in (0.3, 0.4, 0.5, 0.55, 0.625, 0.65):
            for i in range(-40, 41):
                for j in range(-40, 41):
                    gap = abs(math.hypot(i * st, j * st) - m)
                    if gap > 1e-12:
                        best = min(best, gap)
    check(best > GRID_TIE_EPS * 100,
          'nearest genuine non-tie cell is %.6f mm away, >100x the epsilon'
          % best)


def test_tie_cell_is_open_not_blocked():
    """Direction matters: the tie must resolve OPEN. A blocked tie would be the
    conservative choice but would keep the router out of a cell that is exactly
    at clearance -- and would still have to pick a side, which is the bug."""
    step, margin = TIE_STEP, TIE_MARGIN
    c = cells(0.0, 0.0, 10 * step, 0.0, margin, step)
    on_boundary = (5, 4)      # exactly margin above the segment interior
    inside = (5, 3)           # a row closer -- genuinely inside
    check(on_boundary not in c, 'the exact-boundary cell %s is OPEN' % (on_boundary,))
    check(inside in c, 'the cell one row inside %s is still blocked' % (inside,))


def test_cell_and_span_forms_agree():
    """Both public forms derive from _capsule_mask, so the epsilon must reach
    both. A divergence here would block different copper through two code paths."""
    rng = random.Random(5)
    bad = 0
    for _ in range(300):
        gx0 = rng.randint(-500, 500); gy0 = rng.randint(-500, 500)
        ldx = rng.randint(-20, 20); ldy = rng.randint(-20, 20)
        x1, y1 = gx0 * TIE_STEP, gy0 * TIE_STEP
        x2, y2 = (gx0 + ldx) * TIE_STEP, (gy0 + ldy) * TIE_STEP
        cell = RU.segment_blocked_cells_array(x1, y1, x2, y2, TIE_MARGIN, TIE_STEP)
        spans = RU.segment_blocked_spans(x1, y1, x2, y2, TIE_MARGIN, TIE_STEP)
        from_cells = set(map(tuple, cell.tolist()))
        from_spans = set()
        for gx, lo, hi in spans.tolist():
            for gy in range(lo, hi + 1):
                from_spans.add((gx, gy))
        if from_cells != from_spans:
            bad += 1
    check(bad == 0, 'cell form and span form agree on 300 tie-config capsules')


def _distinct_over_positions(fn, offs=None):
    """Rasterize the same shape at many board positions, normalise each result
    back to the shape frame, and count distinct answers. 1 == deterministic."""
    offs = offs or [-1000, -777, -411, -137, 0, 41, 293, 508, 908, 997]
    seen = collections.Counter()
    sizes = set()
    for gx0 in offs:
        for gy0 in (-777, 0, 41, 908):
            r = fn(gx0, gy0)
            seen[r] += 1
            sizes.add(len(r))
    return len(seen), (min(sizes), max(sizes))


def test_polygon_raster_position_independence():
    """The polygon path has the SAME defect shape as the capsule: consumers
    threshold `edge_dist` (measured in absolute board coordinates) against a
    clearance. Measured before the epsilon, one rectangle at clearance 0.2 on a
    0.1 grid produced 8 different cell sets, 299 to 342 cells."""
    coord = GridCoord(TIE_STEP)

    def rect(gx0, gy0, clearance):
        x0, y0 = gx0 * TIE_STEP, gy0 * TIE_STEP
        pts = [(x0, y0), (x0 + 2.0, y0), (x0 + 2.0, y0 + 1.0), (x0, y0 + 1.0)]
        lo_x, lo_y, nx, ny, ins, ed = OM._rasterize_polygon_box(
            pts, coord, clearance + TIE_STEP)
        if ins is None:
            return frozenset()
        mask = ins | (ed < clearance - GRID_TIE_EPS)   # as the consumers do
        gx, gy = OM._box_masked_cells(lo_x, lo_y, nx, mask)
        return frozenset((int(a) - gx0, int(b) - gy0)
                         for a, b in zip(gx.tolist(), gy.tolist()))

    for clr, tie in ((0.2, True), (0.25, False), (0.3, True)):
        q = clr / TIE_STEP
        is_tie = abs(q - round(q)) < 1e-9
        n, sizes = _distinct_over_positions(lambda a, b, c=clr: rect(a, b, c))
        check(n == 1,
              'polygon keep-out at clearance %.2f (%s) gives ONE cell set over '
              '40 positions (got %d, sizes %s)'
              % (clr, 'exact tie' if is_tie else 'no tie', n, sizes))
        check(is_tie == tie, 'fixture: clearance %.2f tie-ness is %s as expected'
              % (clr, is_tie))


def test_drill_disc_position_independence():
    """Drill keep-outs compare an absolute squared distance to
    (drill/2 + track/2 + clearance)^2. Ordinary values land on an exact grid
    multiple -- drill 0.4 + track 0.2 + clearance 0.3 = 0.60 = 6 x a 0.1 grid --
    and that produced 8 different cell sets before the epsilon."""
    def disc(gx0, gy0, required_dist):
        hx, hy = gx0 * TIE_STEP, gy0 * TIE_STEP
        coord = GridCoord(TIE_STEP)
        req_sq = (required_dist - GRID_TIE_EPS) ** 2      # as obstacle_map does
        gx, gy = coord.to_grid(hx, hy)
        expand = coord.to_grid_dist_safe(required_dist) + 1
        exs = np.arange(gx - expand, gx + expand + 1, dtype=np.int64)
        eys = np.arange(gy - expand, gy + expand + 1, dtype=np.int64)
        dx = exs.astype(np.float64) * TIE_STEP - hx
        dy = eys.astype(np.float64) * TIE_STEP - hy
        mask = (dx * dx)[:, np.newaxis] + (dy * dy)[np.newaxis, :] < req_sq
        ii, jj = np.nonzero(mask)
        return frozenset((int(exs[i]) - gx0, int(eys[j]) - gy0)
                         for i, j in zip(ii, jj))

    for rd in (0.40, 0.55, 0.60):
        n, sizes = _distinct_over_positions(lambda a, b, r=rd: disc(a, b, r))
        q = rd / TIE_STEP
        check(n == 1,
              'drill disc radius %.2f (%s) gives ONE cell set over 40 positions '
              '(got %d, sizes %s)'
              % (rd, 'exact tie' if abs(q - round(q)) < 1e-9 else 'no tie',
                 n, sizes))


def test_one_shared_epsilon():
    """The three rasterizers must not drift apart -- they read ONE constant."""
    import obstacle_cache, plane_obstacle_builder
    for mod in (OM, obstacle_cache, plane_obstacle_builder):
        check(getattr(mod, 'GRID_TIE_EPS', None) is GRID_TIE_EPS
              or getattr(mod, 'GRID_TIE_EPS', None) == GRID_TIE_EPS,
              '%s uses the shared GRID_TIE_EPS' % mod.__name__)


def main():
    print('#816 grid keep-out boundary tie epsilon (capsule, polygon, drill)')
    for t in (test_margin_is_actually_a_tie, test_position_independence,
              test_relative_frame_is_now_bit_safe, test_epsilon_headroom,
              test_tie_cell_is_open_not_blocked, test_cell_and_span_forms_agree,
              test_polygon_raster_position_independence,
              test_drill_disc_position_independence, test_one_shared_epsilon):
        print('\n%s:' % t.__name__)
        t()
    print('')
    if FAILS:
        print('FAILED (%d): %s' % (len(FAILS), '; '.join(FAILS)))
        return 1
    print('All #816 capsule tie-epsilon tests passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
