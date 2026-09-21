#!/usr/bin/env python3
"""What each frame layout BUYS, at one pixel budget (#946, #1018).

The layout choice was decided by a measurement and the measurement lived in a
planning document. Every consumer then quoted it in prose -- `frame_layout`'s
module docstring, `render_panels.CELL_MIN_W`'s comment, `docs/route-animation`
and the PR body -- and a quoted number is a claim, not a measurement. The PR's
own fact-checker reconstructed the table and found **`inset`'s px-per-layer-cell
was 32k in every copy and **28 490** in fact, which is the figure `CELL_MIN_W`'s
comment leans on.

So the table has a producer now. It re-derives from `frame_layout.plan_frame`,
which is the thing that actually decides the boxes, and
`tests/test_946_layout_budget.py` compares the shipped numbers against it.

    python3 -X utf8 py_router/layout_budget.py
    python3 -X utf8 py_router/layout_budget.py --json out.json --aspect 1.85

**THE TWO METRICS ARE THE POINT, and they never agree.**

  px/mm on copper     how much detail the BOARD gets -- the board box's width
                      divided by the board's width in mm
  px per layer cell   how much detail ONE LAYER of the strip gets -- the
                      panel's area divided by the cells across it

`inset` wins the first everywhere and loses the second everywhere; `split` is
the mirror image. `stacked` and `sidebar` genuinely swap on board aspect. That
asymmetry is why `stacked`-vs-`sidebar` is INFERRED and `inset`-vs-`split` is
DECLARED -- and it is the honest output of the arithmetic rather than a
recommendation.

The budget is **1.62 Mpx**: the 1000 px board view plus the 0.62 Mpx iso panel,
which is what the two-panel film already costs. Every layout is sized to the
same budget, so the columns compare.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'placement'], 'kind': 'instrument'}

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: 1000 px of board plus the 0.62 Mpx iso panel -- what a two-panel film costs.
BUDGET_PX = 1_620_000

#: The corpus proportion the shipped table is quoted at.
REFERENCE_ASPECT = 1.85

#: How many cells the strip is measured at. Four is the count the original
#: study used, and the metric is per-CELL so it has to fix one.
CELLS = 4

#: The board's width in mm at the reference. Only the RATIO of px to mm
#: matters, so this is a scale, not a claim about any particular board.
BOARD_MM = 100.0

LAYOUTS = ('stacked', 'sidebar', 'inset', 'split')


def _size_for_budget(bounds, layout, budget=BUDGET_PX, lo=100, hi=4000):
    """The `size` whose planned frame is closest to `budget` pixels."""
    import frame_layout
    best, best_err = lo, None
    while lo <= hi:
        mid = (lo + hi) // 2
        g = frame_layout.plan_frame(bounds, layout=layout, size=mid,
                                    panel=True, quiet=True)
        area = g.frame.w * g.frame.h
        err = abs(area - budget)
        if best_err is None or err < best_err:
            best, best_err = mid, err
        if area < budget:
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def measure(aspect=REFERENCE_ASPECT, budget=BUDGET_PX, cells=CELLS):
    """`{layout: {...}}` -- the table, from `plan_frame` itself."""
    import frame_layout
    bounds = (0.0, 0.0, BOARD_MM, BOARD_MM / float(aspect))
    out = {}
    for key in LAYOUTS:
        size = _size_for_budget(bounds, key, budget)
        g = frame_layout.plan_frame(bounds, layout=key, size=size, panel=True,
                                    quiet=True)
        panel = g.panel
        cell_px = ((panel.w * panel.h) / float(cells)) if panel else 0.0
        out[key] = {
            'size': size,
            'frame': [g.frame.w, g.frame.h],
            'frame_area': g.frame.w * g.frame.h,
            'frame_aspect': round(g.frame.w / float(g.frame.h), 3),
            'board': [g.board.w, g.board.h],
            'panel': [panel.w, panel.h] if panel else None,
            # px/mm is the BOARD box against the board's own width; the board
            # is letterboxed inside it, so the honest figure is the scale the
            # copper is actually drawn at.
            'px_per_mm': round(min(g.board.w / BOARD_MM,
                                   g.board.h / (BOARD_MM / float(aspect))), 2),
            'px_per_layer_cell': int(round(cell_px)),
            'overlays_board': g.overlays_board,
        }
    return out


def swing(aspects=(0.5, 0.8, 1.0, 1.25, 1.6, 1.85, 2.5), budget=BUDGET_PX):
    """By how much `stacked` and `sidebar` swap across board shapes.

    This is the "8.8-23.8%" claim, and it is the ONLY reason `auto` exists: if
    the two never swapped, one of them would simply be better.
    """
    rows = []
    for a in aspects:
        m = measure(a, budget)
        st, sb = m['stacked'], m['sidebar']
        for metric in ('px_per_mm', 'px_per_layer_cell'):
            lo = min(st[metric], sb[metric])
            hi = max(st[metric], sb[metric])
            rows.append({'aspect': a, 'metric': metric,
                         'winner': ('stacked' if st[metric] >= sb[metric]
                                    else 'sidebar'),
                         'gap_pct': round(100.0 * (hi - lo) / max(lo, 1), 1)})
    return rows


def format_table(m):
    w = ['%-9s %-11s %-7s %-9s %-9s' % ('layout', 'frame', 'aspect',
                                        'px/mm', 'px/cell')]
    for k in LAYOUTS:
        r = m[k]
        w.append('%-9s %-11s %-7.2f %-9.2f %-9d'
                 % (k, '%dx%d' % tuple(r['frame']), r['frame_aspect'],
                    r['px_per_mm'], r['px_per_layer_cell']))
    best_mm = max(LAYOUTS, key=lambda k: m[k]['px_per_mm'])
    best_cell = max(LAYOUTS, key=lambda k: m[k]['px_per_layer_cell'])
    w.append('')
    w.append('px/mm winner      %s (%.2f)' % (best_mm, m[best_mm]['px_per_mm']))
    w.append('px/cell winner    %s (%d)'
             % (best_cell, m[best_cell]['px_per_layer_cell']))
    if best_mm != best_cell:
        w.append('-> NO LAYOUT WINS BOTH, which is why inset-vs-split is a '
                 'declaration and not an inference.')
        w.append('   the cell penalty is %.1fx'
                 % (m[best_cell]['px_per_layer_cell']
                    / max(1.0, float(m[best_mm]['px_per_layer_cell']))))
    return '\n'.join(w)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--aspect', type=float, default=REFERENCE_ASPECT)
    ap.add_argument('--budget', type=int, default=BUDGET_PX)
    ap.add_argument('--cells', type=int, default=CELLS)
    ap.add_argument('--json', metavar='PATH')
    ap.add_argument('--swing', action='store_true',
                    help='how much stacked and sidebar swap across shapes')
    a = ap.parse_args(argv)
    m = measure(a.aspect, a.budget, a.cells)
    print('budget %s px, board aspect %.2f, %d cells'
          % ('{:,}'.format(a.budget), a.aspect, a.cells))
    print(format_table(m))
    if a.swing:
        print('')
        for r in swing(budget=a.budget):
            print('  aspect %-5.2f %-18s %-8s %5.1f%%'
                  % (r['aspect'], r['metric'], r['winner'], r['gap_pct']))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump({'budget': a.budget, 'aspect': a.aspect,
                       'cells': a.cells, 'layouts': m,
                       'swing': swing(budget=a.budget)}, f, indent=1)
        print('wrote %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
