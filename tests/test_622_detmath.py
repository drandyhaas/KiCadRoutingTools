#!/usr/bin/env python3
"""#622 `awx/detmath.py`: the whole route's two LPs give one answer per model.

The geometry's LP and the polish's are degenerate: a face of equal optima, and
the vertex the solver lands on is decided by its own rounding, so the same
model could land on another point after a change that should not matter (K28:
the geometry LP's input identical, its answer 10 mm apart). detmath adds a
fixed tie-break per column and rounds the solution's last bits off.

What this asserts:

1. A model with a face of optima solves to one point, by the interior point
   and the simplex alike.
2. lp_round takes the last bits off and lands on its binary grid.
3. No awx module swaps math's or numpy's functions for its process: the whole
   route runs inside KiCad's own process, where such a swap would change every
   later computation there.
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
AWX = os.path.join(os.path.dirname(HERE), 'awx')
sys.path.insert(0, AWX)
import numpy as np  # noqa: E402
import detmath  # noqa: E402

FAIL = []


def check(ok, what):
    print(('  ok   ' if ok else '  FAIL ') + what)
    if not ok:
        FAIL.append(what)


def main():
    print('1. an LP\'s answer is its model\'s')
    from scipy.optimize import linprog
    # max x0 + x1 on x0 + x1 <= 1: an edge of optima, whose point the algorithm picks; with the tie-break, one point
    n = 2
    c = np.array([-1.0, -1.0]) + detmath.lp_tie_break(n)
    A = np.array([[1.0, 1.0]]); b = np.array([1.0])
    xs = [detmath.lp_round(linprog(c, A_ub=A, b_ub=b, bounds=[(0, 1)] * n, method=m_).x) for m_ in ('highs-ipm', 'highs-ds')]
    check(np.array_equal(xs[0], xs[1]) and sorted(xs[0].tolist()) == [0.0, 1.0],
          f'the interior point and the simplex give one vertex of the edge: {xs[0].tolist()} / {xs[1].tolist()}')

    print('2. lp_round')
    q = detmath.lp_round(np.array([0.1, 1 / 3, -2.7, 1e-12]))
    check(np.array_equal(q, detmath.lp_round(q + np.array([1e-15, -1e-16, 2e-15, 0.0]))), 'lp_round takes the last bits off')
    check(all(float(v) * 2 ** 24 == round(float(v) * 2 ** 24) for v in q), 'lp_round lands on its binary grid')

    print('3. no module swaps math\'s or numpy\'s functions')
    swaps = []
    for f in sorted(os.listdir(AWX)):
        if not f.endswith('.py'):
            continue
        src = open(os.path.join(AWX, f)).read()
        for node in ast.walk(ast.parse(src)):
            # setattr(math, ...) / setattr(np, ...), or math.X = ... / np.X = ...
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'setattr'
                    and node.args and isinstance(node.args[0], ast.Name) and node.args[0].id in ('math', 'np', 'numpy')):
                swaps.append(f'{f}:{node.lineno}')
            if isinstance(node, ast.Assign):
                for t in node.targets:
                    if (isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                            and t.value.id in ('math', 'np', 'numpy')):
                        swaps.append(f'{f}:{node.lineno}')
    check(not swaps, 'no math.X = / np.X = / setattr(math|np, ...) in awx' + ('' if not swaps else ': ' + ', '.join(swaps)))

    print(f'\n{"FAILED: " + str(len(FAIL)) if FAIL else "all passed"}')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
