#!/usr/bin/env python3
"""The layout table has a producer, and the shipped numbers are its output
(#946, #1018).

The layout choice was decided by a measurement and the measurement lived in a
planning document. Every consumer then quoted it in prose -- `frame_layout`'s
module docstring, `render_panels.CELL_MIN_W`'s comment, `docs/route-animation`,
the PR body -- and a quoted number is a claim, not a measurement.

The PR's own fact-checker reconstructed the table and found **`inset`'s
px-per-layer-cell was 32k in every copy and 28 490 in fact** -- a 12% error, in
the number `CELL_MIN_W`'s own comment leans on. Nothing could have caught it,
because nothing computed it.

`py_router/layout_budget.py` computes it from `frame_layout.plan_frame`, which
is the thing that actually decides the boxes. This file pins:

  * **the shipped table IS the instrument's output**, to the digit;
  * **no layout wins both metrics** -- the asymmetry that makes
    `inset`-vs-`split` a declaration rather than an inference. If one ever did
    win both, `auto` should choose it and this file should go red;
  * **`stacked` and `sidebar` genuinely swap**, or `auto` is choosing between
    two things one of which is simply better;
  * **the swap happens at `ADAPTIVE_ASPECT_CUT`**, which is the one number
    `auto` actually branches on -- a cut placed anywhere else is a coin flip
    wearing a measurement's clothes.
"""
import os
import sys

RUN_ALL_TIMEOUT = 600

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import frame_layout as FL          # noqa: E402
import layout_budget as LB         # noqa: E402

#: The table as it is quoted in `docs/route-animation.md` and the PR body.
#: (px/mm, px-per-layer-cell, frame aspect)
SHIPPED = {
    'stacked': (10.00, 113000, 0.62),
    'sidebar': (12.38, 100050, 1.78),
    'inset':   (15.76,  28490, 1.85),
    'split':   (11.06, 128800, 1.60),
}

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def test_the_shipped_table_is_the_instruments_output():
    _mark = len(_FAIL)
    m = LB.measure()
    for key, (mm, cell, aspect) in sorted(SHIPPED.items()):
        got = m.get(key)
        if got is None:
            fail('%s: the instrument does not measure it' % key)
            continue
        if abs(got['px_per_mm'] - mm) > 0.01:
            fail('%s: px/mm is %.2f, the docs say %.2f'
                 % (key, got['px_per_mm'], mm))
        if got['px_per_layer_cell'] != cell:
            fail('%s: px/cell is %d, the docs say %d'
                 % (key, got['px_per_layer_cell'], cell))
        if abs(got['frame_aspect'] - aspect) > 0.01:
            fail('%s: frame aspect is %.2f, the docs say %.2f'
                 % (key, got['frame_aspect'], aspect))
    if len(_FAIL) == _mark:
        print('    ' + LB.format_table(m).replace('\n', '\n    '))
        print('  PASS: every quoted figure re-derives')


def test_no_layout_wins_both_metrics():
    """The asymmetry IS the design rule. If one layout ever won both, `auto`
    should choose it and `inset`-vs-`split` would stop being a stance."""
    _mark = len(_FAIL)
    for aspect in (0.5, 1.0, 1.25, 1.85, 2.5):
        m = LB.measure(aspect)
        best_mm = max(m, key=lambda k: m[k]['px_per_mm'])
        best_cell = max(m, key=lambda k: m[k]['px_per_layer_cell'])
        if best_mm == best_cell:
            fail('at aspect %.2f, %s wins BOTH metrics -- auto should simply '
                 'choose it' % (aspect, best_mm))
        elif aspect == 1.85:
            pen = (m[best_cell]['px_per_layer_cell']
                   / float(m[best_mm]['px_per_layer_cell']))
            print('    aspect 1.85: px/mm %s, px/cell %s, penalty %.1fx'
                  % (best_mm, best_cell, pen))
            if pen < 2.0:
                fail('the cell penalty is only %.1fx; the table calls it a '
                     'reason to declare rather than infer' % pen)
    if len(_FAIL) == _mark:
        print('  PASS: the two metrics never agree')


def test_stacked_and_sidebar_swap_at_the_cut():
    _mark = len(_FAIL)
    rows = [r for r in LB.swing() if r['metric'] == 'px_per_mm']
    winners = {r['aspect']: r['winner'] for r in rows}
    if len(set(winners.values())) < 2:
        fail('stacked and sidebar never swap (%s) -- auto would be choosing '
             'between two things one of which is simply better' % winners)
        return
    below = [a for a, w in winners.items() if a < FL.ADAPTIVE_ASPECT_CUT]
    above = [a for a, w in winners.items() if a > FL.ADAPTIVE_ASPECT_CUT]
    bad = [a for a in below if winners[a] != 'stacked']
    bad += [a for a in above if winners[a] != 'sidebar']
    if bad:
        fail('the swap does not happen at ADAPTIVE_ASPECT_CUT = %.2f: %s'
             % (FL.ADAPTIVE_ASPECT_CUT, {a: winners[a] for a in sorted(bad)}))
    else:
        gaps = [r['gap_pct'] for r in rows]
        print('    crossover at ADAPTIVE_ASPECT_CUT %.2f; the gap runs '
              '%.1f%%-%.1f%%' % (FL.ADAPTIVE_ASPECT_CUT, min(gaps), max(gaps)))
    # and `auto` must actually CHOOSE what the measurement says
    for aspect in (0.8, 1.0, 1.6, 2.5):
        g = FL.plan_frame((0, 0, 100.0, 100.0 / aspect), layout='auto',
                          size=800, panel=True, quiet=True)
        want = winners.get(aspect)
        if want and g.layout != want:
            fail('auto chose %s at aspect %.2f; the measurement says %s'
                 % (g.layout, aspect, want))
    if len(_FAIL) == _mark:
        print('  PASS: the cut is where the measurement puts it, and auto '
              'follows it')


TESTS = (
    test_the_shipped_table_is_the_instruments_output,
    test_no_layout_wins_both_metrics,
    test_stacked_and_sidebar_swap_at_the_cut,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
