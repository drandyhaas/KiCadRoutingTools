"""In MAX mode the proximity merge hands each source to the map as it is,
without stacking them into one copy first.

The map's write is a per-cell max-insert, so the order and grouping of rows
do not change what it holds. Stacking copied every row of every source on
each prepare -- millions of rows once a rip-up holds dozens of ripped
corridors -- and the stacked memo missed whenever one ghost was added or
dropped.

Rows:
  - overlapping sources (a cell in several, at different costs, plus a cost-0
    row) leave the map holding the per-cell max of the rows with a cost, read
    back cell by cell;
  - no stacked copy is made in MAX mode (np.vstack is not called);
  - a second merge of the same sources leaves the map as it was.

    python3 tests/test_merge_sources_unstacked.py
"""

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

import env_knobs  # noqa: E402
import obstacle_costs as oc  # noqa: E402
from grid_router import GridObstacleMap  # noqa: E402

LAYERS = 3


def _sources(rng):
    def src(n):
        a = np.empty((n, 4), dtype=np.int32)
        a[:, 0] = rng.integers(0, LAYERS, n)
        a[:, 1] = rng.integers(-20, 20, n)
        a[:, 2] = rng.integers(-20, 20, n)
        a[:, 3] = rng.integers(0, 400, n)
        return a
    per_net = {7: src(900), 8: src(600), 9: np.empty((0, 4), dtype=np.int32)}
    ghosts = {11: src(1200), ('history',): src(500), 12: src(800)}
    return per_net, ghosts


def _expected(arrays):
    out = {}
    for a in arrays:
        for l, x, y, c in a.tolist():
            if c > 0 and c > out.get((l, x, y), 0):
                out[(l, x, y)] = c
    return out


def _read(obs):
    return {(l, x, y): v for l in range(LAYERS) for x in range(-21, 21)
            for y in range(-21, 21)
            if (v := obs.get_layer_proximity_cost(x, y, l)) > 0}


def main():
    fails = []

    def ok(label, cond, detail=''):
        print(f"  [{'ok' if cond else 'FAIL'}] {label}"
              f"{'' if cond else ': ' + str(detail)}")
        if not cond:
            fails.append(label)

    if env_knobs.PROXIMITY_SUM:
        print('KICAD_PROXIMITY_SUM is set: this test covers MAX mode')
        return 2
    per_net, ghosts = _sources(np.random.default_rng(0))
    want = _expected(list(per_net.values()) + list(ghosts.values()))

    stacked = []
    real_vstack = oc.np.vstack
    oc.np.vstack = lambda *a, **k: stacked.append(1) or real_vstack(*a, **k)
    try:
        obs = GridObstacleMap(LAYERS)
        oc.merge_track_proximity_costs(obs, per_net, ghost_costs=ghosts)
        got = _read(obs)
        ok('the map holds the per-cell max of every source', got == want,
           f'{len(set(got.items()) ^ set(want.items()))} cells differ')
        ok('no stacked copy is made', not stacked, f'{len(stacked)} vstack call(s)')
        oc.merge_track_proximity_costs(obs, per_net, ghost_costs=ghosts)
        ok('a second merge of the same sources changes nothing', _read(obs) == want)
    finally:
        oc.np.vstack = real_vstack

    print('ALL PASS' if not fails else f'{len(fails)} FAILED')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
