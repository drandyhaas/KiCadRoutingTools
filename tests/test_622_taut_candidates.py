#!/usr/bin/env python3
"""The taut strings' candidate lists, built from the obstacles near each run of points, are the lists every point
against every obstacle gives (awx/taut_fast.py: _batch_candidates, _near_table).

  python3 tests/test_622_taut_candidates.py

A taut string's points are pushed off the nearest obstacles within REACH; the lists of those were found by measuring
every point of every string against every obstacle of the bench, again at every rebuild -- two thirds of the time one
long-laned bus's bench took to plan. The same distances are now measured only where an obstacle's reach meets a run
of points' box, and written into the same table, every other entry past REACH as before. The lists must be the very
ones: the relaxation takes the deepest of a point's candidates, and the first of two equally deep.

1. on random strings and obstacles -- some on a grid, where a point stands exactly between two obstacles, some
   obstacles of no length, more candidates than a list holds -- the two lists (discs, capsules) are the same as the
   whole table's, entry for entry and in the same order;
2. ...with no discs, no capsules, or one point;
3. a point within REACH of an obstacle is never missed where the obstacle's box only touches the run's.
"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import taut_fast as tf  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


def whole_table(P, own, D, C, dnet, cnet):
    """the lists as every point against every obstacle gives them: the definition"""
    m = len(P)
    if len(D):
        dd = np.hypot(P[:, None, 0] - D[None, :, 0], P[:, None, 1] - D[None, :, 1]) - D[None, :, 2]
        dd = np.where((dd <= tf.REACH) & (dnet[None, :] != own[:, None]), dd, np.inf)
        k = min(tf.KMAX, dd.shape[1])
        od = np.argpartition(dd, k - 1, axis=1)[:, :k] if dd.shape[1] > k else np.argsort(dd, axis=1)
        od = np.where(np.take_along_axis(dd, od, 1) < np.inf, od, -1)
    else:
        od = np.full((m, 0), -1)
    if len(C):
        ax, ay, sx, sy, r = C.T
        L2 = np.where(sx * sx + sy * sy > 1e-12, sx * sx + sy * sy, 1.0)
        px = P[:, None, 0] - ax[None]
        py = P[:, None, 1] - ay[None]
        t = np.clip((px * sx[None] + py * sy[None]) / L2[None], 0.0, 1.0)
        dc = np.hypot(px - t * sx[None], py - t * sy[None]) - r[None]
        dc = np.where((dc <= tf.REACH) & (cnet[None, :] != own[:, None]), dc, np.inf)
        k = min(tf.KMAX, dc.shape[1])
        oc = np.argpartition(dc, k - 1, axis=1)[:, :k] if dc.shape[1] > k else np.argsort(dc, axis=1)
        oc = np.where(np.take_along_axis(dc, oc, 1) < np.inf, oc, -1)
    else:
        oc = np.full((m, 0), -1)
    return od, oc


def bench(rng, m, nd, nc, grid=False, dots=False, dense=False):
    P = np.cumsum(rng.normal(0, 0.05, (m, 2)), 0) + rng.uniform(0, 20, 2)
    if grid:
        P = np.round(P / 0.05) * 0.05
    span = 0.4 if dense else 2.0
    lo, hi = P.min(0) - span, P.max(0) + span
    snap = lambda a: np.round(a / 0.05) * 0.05      # noqa: E731
    D = np.column_stack([snap(rng.uniform(lo[0], hi[0], nd)), snap(rng.uniform(lo[1], hi[1], nd)),
                         rng.choice([0.2, 0.325, 0.4], nd)]) if nd else np.zeros((0, 3))
    C = np.column_stack([snap(rng.uniform(lo[0], hi[0], nc)), snap(rng.uniform(lo[1], hi[1], nc)),
                         snap(rng.normal(0, 1.5, nc)), snap(rng.normal(0, 1.5, nc)), rng.choice([0.1, 0.2], nc)]) \
        if nc else np.zeros((0, 5))
    if dots and nc:
        C[: max(1, nc // 4), 2:4] = 0.0
    return P, rng.integers(0, 6, m), D, C, rng.integers(0, 6, nd), rng.integers(0, 6, nc)


def same(args):
    a, b = whole_table(*args), tf._batch_candidates(*args)
    return np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])


rng = np.random.default_rng(11)
cases = [bench(rng, int(rng.integers(2, 2500)), int(rng.integers(1, 250)), int(rng.integers(1, 200)),
               grid=i % 3 == 0, dots=i % 4 == 0, dense=i % 5 == 0) for i in range(30)]
check('1. random strings and obstacles: the same lists, entry for entry, in the same order',
      all(same(c) for c in cases), [i for i, c in enumerate(cases) if not same(c)])
full = [c for c in cases if (tf._batch_candidates(*c)[0] >= 0).all(1).any()]
check('1. ...among them points with more candidates than a list holds', len(full) > 0, len(full))
check('2. no discs', same(bench(rng, 300, 0, 40)))
check('2. no capsules', same(bench(rng, 300, 40, 0)))
check('2. one point', same(bench(rng, 1, 30, 30)))
# 3. an obstacle whose reach just touches a run's box
P = np.column_stack([np.linspace(0.0, 30.0, 601), np.zeros(601)])
own = np.zeros(601, dtype=int)
IN, OUT = tf.REACH + 0.2 - 1e-7, tf.REACH + 0.2 + 1e-7       # (a disc of radius 0.2 just within REACH, and just past)
D = np.array([[10.0, IN, 0.2], [20.0, -IN, 0.2], [25.0, OUT, 0.2]])
C = np.array([[5.0, tf.REACH + 0.1 - 1e-7, 2.0, 0.0, 0.1]])
args = (P, own, D, C, np.array([1, 1, 1]), np.array([1]))
od, oc = tf._batch_candidates(*args)
check('3. obstacles just within REACH of the strings\' line: found for the points beside them, the one just past it '
      'not, and the lists the whole table\'s', same(args) and 0 in od[200] and 1 in od[400] and 2 not in od[500]
      and 0 in oc[120], (od[200], od[400], od[500], oc[120]))

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
