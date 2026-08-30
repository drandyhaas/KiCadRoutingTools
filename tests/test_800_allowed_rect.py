#!/usr/bin/env python3
"""#800: `add_allowed_rect` -- one Python->Rust crossing for a block of allowed
cells, instead of one per cell.

The terminal-exemption block in `route_net_with_obstacles` called
`add_allowed_cell` once per cell: 509,109,266 crossings per route on rp2350,
47.2 s of a ~365 s route. All five call sites are RECTANGLES around a terminal,
so the batch this needed is a rect, not the `N x 3` cell array the issue
sketched -- `allowed_cells` is one set keyed by `pack_xy` with NO layer
dimension, so an `N x 3` array carries a column the map cannot store, and a
cell batch would have Python build a 441-row array first, paying much of the
per-cell cost it exists to remove.

This is a PURE PERFORMANCE change, so the only thing worth testing is that it
changed nothing. The arms below compare the rect against the per-cell loop it
replaced, cell for cell, through a real GridObstacleMap -- including the two
cases the callers actually depend on and that are easy to get wrong in Rust:

  * an INVERTED range (min > max) must insert NOTHING. Callers clip their block
    to `bounds` with plain min/max, so a terminal lying wholly outside bounds
    produces lo > hi -- which Python's `range()` renders as an empty loop. A
    Rust `for gx in min..=max` on an inverted range is also empty, but the
    reserve() sizing above it is not, so this is asserted rather than assumed.
  * NEGATIVE and out-of-range coordinates, since the grid is not clamped and
    `pack_xy` must round-trip them the same either way.

Read through the OBSERVABLE (`is_via_blocked` inside a BGA zone consults
`allowed_cells`) rather than a getter, because that is the only thing the
router itself asks of this set.

    python3 tests/test_800_allowed_rect.py
"""
import os
import random
import sys

_R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_R, os.path.join(_R, 'py_router'), os.path.join(_R, 'rust_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import grid_router

fails = []


def check(name, cond):
    print(('  ok  ' if cond else ' FAIL ') + name)
    if not cond:
        fails.append(name)


def _map(build):
    m = grid_router.GridObstacleMap(2)
    # Everything is inside a BGA zone, so is_via_blocked answers purely from
    # allowed_cells -- the observable this set feeds.
    m.set_bga_zone(-10000, -10000, 10000, 10000)
    build(m)
    return m


def _percell(lo_x, lo_y, hi_x, hi_y):
    return _map(lambda m: [m.add_allowed_cell(x, y)
                           for x in range(lo_x, hi_x + 1)
                           for y in range(lo_y, hi_y + 1)])


def _rect(lo_x, lo_y, hi_x, hi_y):
    return _map(lambda m: m.add_allowed_rect(lo_x, lo_y, hi_x, hi_y))


def _same(lo_x, lo_y, hi_x, hi_y, pad=3):
    a, b = _percell(lo_x, lo_y, hi_x, hi_y), _rect(lo_x, lo_y, hi_x, hi_y)
    lo_x, hi_x = min(lo_x, hi_x), max(lo_x, hi_x)
    lo_y, hi_y = min(lo_y, hi_y), max(lo_y, hi_y)
    for x in range(lo_x - pad, hi_x + pad + 1):
        for y in range(lo_y - pad, hi_y + pad + 1):
            if a.is_via_blocked(x, y) != b.is_via_blocked(x, y):
                return False
    return True


def the_api_exists_and_the_old_one_survives():
    check('add_allowed_rect is exported',
          hasattr(grid_router.GridObstacleMap, 'add_allowed_rect'))
    check('add_allowed_cell is still exported (nothing was removed)',
          hasattr(grid_router.GridObstacleMap, 'add_allowed_cell'))


def a_rect_equals_the_per_cell_loop():
    """The shapes the five real call sites emit: 21x21 (the 509M one), 11x11
    twice, and the diff-pair 5x5."""
    for r in (10, 5, 2):
        check(f'{2*r+1}x{2*r+1} block at the origin matches the per-cell loop',
              _same(-r, -r, r, r))
        check(f'{2*r+1}x{2*r+1} block at a NEGATIVE origin matches',
              _same(-500 - r, -300 - r, -500 + r, -300 + r))


def an_inverted_range_inserts_nothing():
    """What clipping a wholly-outside terminal produces. Asserted, not assumed:
    the reserve() sizing sits above the loop and an unguarded (max-min+1) on an
    inverted range underflows a usize."""
    for lo_x, lo_y, hi_x, hi_y in ((5, 5, 4, 9), (5, 5, 9, 4), (5, 5, 4, 4),
                                   (0, 0, -1, -1), (100, 100, -100, -100)):
        m = _rect(lo_x, lo_y, hi_x, hi_y)
        empty = not any(not m.is_via_blocked(x, y)
                        for x in range(-110, 111) for y in range(-110, 111))
        check(f'inverted range ({lo_x},{lo_y})-({hi_x},{hi_y}) adds no cell',
              empty)


def clipping_a_block_to_bounds_matches():
    """The caller's own idiom end to end: build the block, clip it with min/max
    against `bounds`, hand the result over. Includes a terminal wholly outside
    bounds, which is what produces the inverted range above."""
    bounds = (0, 0, 399, 399)
    random.seed(800)
    cases = [(5, 5, 10), (395, 395, 10), (-100, -100, 10), (500, 500, 10),
             (0, 399, 10), (399, 0, 10)]
    cases += [(random.randint(-40, 440), random.randint(-40, 440),
               random.choice([2, 5, 10])) for _ in range(150)]
    bad = 0
    for gx, gy, r in cases:
        lo_x = max(gx - r, bounds[0])
        hi_x = min(gx + r, bounds[2])
        lo_y = max(gy - r, bounds[1])
        hi_y = min(gy + r, bounds[3])
        if not _same(lo_x, lo_y, hi_x, hi_y):
            bad += 1
    check(f'{len(cases)} bounds-clipped terminals match the per-cell loop '
          f'({bad} mismatches)', bad == 0)


def a_rect_is_idempotent_and_composes():
    """allowed_cells is a SET. The diff-pair sites emit two overlapping blocks
    that used to be interleaved cell-by-cell; emitting one then the other must
    admit the identical set."""
    a = _map(lambda m: [m.add_allowed_rect(0, 0, 10, 10),
                        m.add_allowed_rect(5, 5, 15, 15)])
    b = _map(lambda m: [m.add_allowed_cell(x, y)
                        for x in range(0, 11) for y in range(0, 11)]
             + [m.add_allowed_cell(x, y)
                for x in range(5, 16) for y in range(5, 16)])
    same = all(a.is_via_blocked(x, y) == b.is_via_blocked(x, y)
               for x in range(-3, 20) for y in range(-3, 20))
    check('two overlapping rects == the interleaved per-cell loop', same)

    c = _map(lambda m: [m.add_allowed_rect(0, 0, 10, 10)] * 1
             + [m.add_allowed_rect(0, 0, 10, 10)])
    d = _rect(0, 0, 10, 10)
    check('the same rect twice changes nothing (idempotent)',
          all(c.is_via_blocked(x, y) == d.is_via_blocked(x, y)
              for x in range(-3, 14) for y in range(-3, 14)))


def clear_still_clears_it():
    m = _rect(0, 0, 10, 10)
    m.clear_allowed_cells()
    check('clear_allowed_cells drops a rect-added block',
          all(m.is_via_blocked(x, y) for x in range(0, 11)
              for y in range(0, 11)))


def no_call_site_still_crosses_per_cell():
    """Source guard. The point of #800 is that NO routing call site adds
    allowed cells one at a time; a new per-cell loop would silently reintroduce
    the 509M crossings this removed."""
    hits = []
    for rel in ('py_router/single_ended_routing.py',
                'py_router/diff_pair_routing.py'):
        with open(os.path.join(_R, *rel.split('/')), encoding='utf-8') as f:
            for i, line in enumerate(f, 1):
                if 'add_allowed_cell' in line.split('#')[0]:
                    hits.append(f'{rel}:{i}')
    check(f'no routing call site calls add_allowed_cell per cell ({hits})',
          not hits)


def main():
    print('#800 add_allowed_rect -- one crossing per block, same cells\n')
    the_api_exists_and_the_old_one_survives()
    a_rect_equals_the_per_cell_loop()
    an_inverted_range_inserts_nothing()
    clipping_a_block_to_bounds_matches()
    a_rect_is_idempotent_and_composes()
    clear_still_clears_it()
    no_call_site_still_crosses_per_cell()
    print()
    if fails:
        print(f'{len(fails)} FAILURE(S):')
        for f in fails:
            print(f'  - {f}')
        return 1
    print('All #800 checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
