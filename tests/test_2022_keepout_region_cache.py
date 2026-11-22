#!/usr/bin/env python3
"""Each net-filtered keep-out REGION is rasterised once per geometry (#2022
follow-up).

#2022 keeps an edge connector's finger strip clear: per finger a column that
binds every strip net but its own, a tail that binds its own, and one via band
for all. So each finger net binds a DIFFERENT set of regions, and the stamp
cache, keyed per binding set, rasterised the whole strip again for every
finger net, at every geometry and rescue window it routed at. On a board with
a few hundred strip regions that is most of a second per net and geometry,
spent before the routing it delays.

The fix caches the cells per region and gives a binding set the union of its
regions' cells; the native pass stamps every keep-out on its own, so the
union is the joint rasterisation, row for row.

Checks (a synthetic strip of N nets off-origin, two geometries):
  1. every net's (track, via) cells equal the joint rasterisation of its set,
     same rows, order and dtype;
  2. across all nets and both geometries each region is rasterised once per
     geometry (not once per net that binds it);
  3. a second pass hits the cache: no rasterisation at all;
  4. the packed unique equals np.unique(axis=0) on rows with negative
     coordinates and on a span too wide to pack.

    python3 tests/test_2022_keepout_region_cache.py
"""
import os
import sys

import numpy as np

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import net_keepouts as nk  # noqa: E402
import obstacle_map  # noqa: E402
from kicad_parser import BoardInfo, Net  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from synth import make_pcb  # noqa: E402

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
N = 8
fails = []


def ok(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def strip():
    """N fingers 0.6 mm apart, left of the origin and below it."""
    names = frozenset(f'/F{i}' for i in range(N))
    kos = [{'name': 'band', 'polygon': rect(-6.0, -3.0, -6.0 + 0.6 * N, -1.0),
            'holes': [], 'layers': set(LAYERS), 'tracks_allowed': True,
            'vias_allowed': False, 'nets': names, 'clearance': 0.0}]
    for i in range(N):
        x = -6.0 + 0.6 * i
        kos.append({'name': f'col {i}', 'polygon': rect(x, -3.0, x + 0.6, -2.0),
                    'holes': [], 'layers': {'F.Cu'}, 'tracks_allowed': False,
                    'vias_allowed': True, 'nets': names - {f'/F{i}'},
                    'clearance': 0.0})
        kos.append({'name': f'tail {i}', 'polygon': rect(x, -2.0, x + 0.6, -1.5),
                    'holes': [rect(x + 0.2, -1.9, x + 0.4, -1.7)],
                    'layers': {'F.Cu'}, 'tracks_allowed': False,
                    'vias_allowed': True, 'nets': frozenset({f'/F{i}'}),
                    'clearance': 0.0})
    bi = BoardInfo(layers={}, board_bounds=(-8, -5, 4, 3), copper_layers=list(LAYERS))
    pcb = make_pcb(board_info=bi,
                   nets={i + 1: Net(i + 1, f'/F{i}') for i in range(N)})
    nk.attach(pcb, kos)
    return pcb


def joint(pcb, idx, cfg):
    rec = nk._Recorder()
    nk._stamper(pcb, idx)(rec, cfg)
    cells = (np.unique(np.concatenate(rec.cells), axis=0) if rec.cells
             else np.empty((0, 3), dtype=np.int32))
    vias = (np.unique(np.concatenate(rec.vias), axis=0) if rec.vias
            else np.empty((0, 2), dtype=np.int32))
    return cells, vias


def same(a, b):
    return a.dtype == b.dtype and np.array_equal(a, b)


def main():
    pcb = strip()
    cfgs = [GridRouteConfig(clearance=0.1, track_width=0.1, via_size=0.45,
                            via_drill=0.2, layers=list(LAYERS), grid_step=0.05),
            GridRouteConfig(clearance=0.15, track_width=0.2, via_size=0.5,
                            via_drill=0.25, layers=list(LAYERS), grid_step=0.025)]
    nets = sorted(n for n in pcb.nets if n)
    ref = {(n, c): joint(pcb, nk._binding(pcb, (n,)), cfgs[c])
           for n in nets for c in range(len(cfgs))}

    real = obstacle_map.add_rule_area_keepout_obstacles
    count = [0]

    def counting(sink, pcb_data, cfg, *a, **k):
        count[0] += len(pcb_data.board_info.keepouts)
        return real(sink, pcb_data, cfg, *a, **k)

    obstacle_map.add_rule_area_keepout_obstacles = counting
    try:
        diff = []
        for n in nets:
            for c, cfg in enumerate(cfgs):
                got = nk._cells_for(pcb, nk._binding(pcb, (n,)), cfg)
                want = ref[(n, c)]
                if not (same(got[0], want[0]) and same(got[1], want[1])):
                    diff.append((n, c))
        first = count[0]
        for n in nets:
            for cfg in cfgs:
                nk._cells_for(pcb, nk._binding(pcb, (n,)), cfg)
        second = count[0] - first
    finally:
        obstacle_map.add_rule_area_keepout_obstacles = real

    regions = len(pcb._net_keepouts)
    nonempty = all(len(ref[k][0]) and len(ref[k][1]) for k in ref)
    ok('1. per-net cells equal the joint rasterisation', not diff and nonempty,
       f'differ {diff}')
    ok('2. each region rasterised once per geometry',
       first == regions * len(cfgs),
       f'{first} rasterisations, {regions} regions x {len(cfgs)} geometries')
    ok('3. a second pass rasterises nothing', second == 0, second)

    rng = np.random.default_rng(7)
    rows = rng.integers(-40, 40, size=(5000, 3)).astype(np.int32)
    wide = rows.copy()
    wide[:7, 0] = [2 ** 30, -2 ** 30, 0, 1, 2 ** 30, 5, -2 ** 30]
    wide[:7, 1] = [2 ** 30, -2 ** 30, 3, 1, -2 ** 30, 5, 2 ** 30]
    wide[:7, 2] = [2 ** 30, -2 ** 30, 3, 1, 0, 5, 2 ** 30]
    got = [nk._unique_rows([rows[:2500], rows[2000:]], 3),
           nk._unique_rows([wide], 3), nk._unique_rows([rows[:, :2]], 2)]
    want = [np.unique(rows, axis=0), np.unique(wide, axis=0),
            np.unique(rows[:, :2], axis=0)]
    ok('4. packed unique equals np.unique(axis=0)',
       all(same(g, w) for g, w in zip(got, want)))
    print('\n' + ('ALL PASS' if not fails else f'{len(fails)} FAILED: {fails}'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
