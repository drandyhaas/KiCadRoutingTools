#!/usr/bin/env python3
"""grid_router geometry kernels vs the Python they replace, bit for bit.

* capsule_spans / capsule_cells == the pre-kernel numpy bodies of
  routing_utils.segment_blocked_spans / segment_blocked_cells_array

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

from grid_router import capsule_spans, capsule_cells
from routing_utils import (_capsule_mask, GRID_TIE_EPS, segment_blocked_spans,
                           segment_blocked_cells_array)


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


if __name__ == '__main__':
    test_capsule()
    print('ALL PASS' if not FAILS else '%d FAILED' % len(FAILS))
    sys.exit(1 if FAILS else 0)
