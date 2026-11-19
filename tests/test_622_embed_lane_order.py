#!/usr/bin/env python3
"""Two lanes of a layer on one cell edge, ordered by where they go (awx/whole_embed.py: way_order), and a run read in
loops round fixed copper read again with the loops cut.

  python3 tests/test_622_embed_lane_order.py

whole_embed reads each lane's reference line onto the cells of its layer's free space and puts the lanes that share a
cell edge in order along it. Where a lane's place on an edge was read off a way round fixed copper its reference does
not take (a bridge), or out of a bar of copper, that place is no order: two lanes taken round one side of another
bus's laid track were ordered by such places, the wrong way round on two edges with a corner in common -- a crossing
inside one cell that no round of the embed undid. way_order orders them by where the two runs GO: followed through
the edges they share to the cell they part in, the one leaving by the edge at the shared edge's one end is nearer that
end, and so back cell by cell; asked forward and backward, no word where the two ways disagree (the runs cross
between) or say nothing.

A cell here is a triangle of vertex ids; an edge (a, b); a run a list of its portals, each the edge it crosses and the
cells it comes from and goes to.

1. two runs through one edge, parting in the next cell: the one leaving by the edge at the lower point is nearer it;
2. ...the other way round: farther;
3. two runs sharing two edges, parting in the second cell: the order carried back through the corner the edges share;
4. two runs that pass the edge in opposite directions: read along each run's own way;
5. forward and backward disagree (they cross between): no word;
6. one way says nothing (a run ends on the edge): the other way's word;
7. both runs end on the edge: no word;
8. the next edges are not the two other edges of one cell: no word;
9. the embed asks it where a place was read off a bridge or out of a bar, and keeps a loop cut only where it leaves
   fewer crossings (the calls are there).
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, 'awx', 'whole_embed.py')).read()

BAD = []
N = [0]


def check(what, got, want):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


# way_order alone, out of the script (the script builds a board's whole embedding when it is imported)
fn = next(n for n in ast.parse(SRC).body if isinstance(n, ast.FunctionDef) and n.name == 'way_order')
ns = {}
exec(compile(ast.Module(body=[fn], type_ignores=[]), 'whole_embed.py', 'exec'), ns)
way_order = ns['way_order']


def run(*steps):
    """a run's portals: steps of (a, b, from_cell, to_cell)"""
    return [{'k': 'P', 'a': a, 'b': b, 'from': f, 'to': t} for a, b, f, t in steps]


def order(rx, ix, ry, iy):
    Ps = {k: (l_, {id(p_): i_ for i_, p_ in enumerate(l_)}) for k, l_ in (('x', rx), ('y', ry))}
    return way_order('x', rx[ix], 'y', ry[iy], Ps)


# cells: T0 = (0,1,4) behind edge (0,1), T1 = (0,1,2) ahead of it, T2 = (1,2,3) beyond edge (1,2)
X = run((0, 1, 'T0', 'T1'), (0, 2, 'T1', 'S'))
Y = run((1, 0, 'T0', 'T1'), (1, 2, 'T1', 'T2'))
check('1. the run leaving by the edge at the lower point is nearer it', order(X, 0, Y, 0), True)
check('2. ...and the other run farther', order(Y, 0, X, 0), False)

X = run((0, 1, 'T0', 'T1'), (1, 2, 'T1', 'T2'), (1, 3, 'T2', 'S'))
Y = run((0, 1, 'T0', 'T1'), (2, 1, 'T1', 'T2'), (2, 3, 'T2', 'S'))
check('3. parting a cell on: X hugs vertex 1 all the way, so on (0,1) it is the farther from 0', order(X, 0, Y, 0),
      False)
check('3. ...and Y the nearer', order(Y, 0, X, 0), True)
check('3. ...and on the edge they part from, X the nearer its lower point 1', order(X, 1, Y, 1), True)

X = run((0, 1, 'T0', 'T1'), (0, 2, 'T1', 'S'))
Yb = run((2, 1, 'S', 'T1'), (1, 0, 'T1', 'T0'))          # the other way: from (1,2) back through (0,1)
check('4. a run going the other way is followed back the way it came', order(X, 0, Yb, 1), True)

X = run((0, 4, 'R', 'T0'), (0, 1, 'T0', 'T1'), (1, 2, 'T1', 'S'))
Y = run((1, 4, 'R', 'T0'), (0, 1, 'T0', 'T1'), (0, 2, 'T1', 'S'))
check('5. nearer 0 coming in, nearer 1 going out: they cross between, no word', order(X, 1, Y, 1), None)

X = run((0, 4, 'R', 'T0'), (0, 1, 'T0', 'T1'))
Y = run((1, 4, 'R', 'T0'), (1, 0, 'T0', 'T1'), (1, 2, 'T1', 'S'))
check('6. a run that ends on the edge: the word of the way back, nearer 0 as it came in', order(X, 1, Y, 1), True)
check('7. both runs end on the edge: no word', order(run((0, 1, 'T0', 'T1')), 0, run((0, 1, 'T0', 'T1')), 0), None)
X = run((0, 1, 'T0', 'T1'), (0, 2, 'T1', 'S'))
Y = run((0, 1, 'T0', 'T1'), (0, 5, 'T1', 'S'))
check('8. two next edges at one end of the shared edge are no cell\'s: no word', order(X, 0, Y, 0), None)

i_cmp = SRC.index('def cmp(x, y):')
check('9. the spread asks the runs\' ways where a place was read off a bridge or out of a bar',
      "x[1]['psh'] or y[1]['psh'] or x[1].get('br') or y[1].get('br')" in SRC[i_cmp:i_cmp + 900]
      and 'way_order(x[0], x[1], y[0], y[1], Ps)' in SRC[i_cmp:i_cmp + 900], True)
i_loop = SRC.index('RUN[q_][\'pts\'] = trace(q_, simple=True)')
check('9. ...and a run read again with its loops cut is kept only where that leaves fewer crossings',
      'if len(same_layer_crossings(layout())) < len(xs_):' in SRC[i_loop:i_loop + 300], True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
