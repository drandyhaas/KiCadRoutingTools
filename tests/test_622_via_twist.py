#!/usr/bin/env python3
"""A via the embedding moved across another lane's run is a cut for the solve (awx/whole_embed.py: twisted_vias;
awx/whole_route.py: the loop's `answered`).

  python3 tests/test_622_via_twist.py

The embedding sets a via whose place in the plan has no room in the nearest free space. That space may lie on the far
side of a lane the via's own lane never crosses: its run and that lane's then cross on one layer, a crossing no solve
made and none prices. Left to the audit it was two lanes on top of each other a few millimetres on, read as the ends'
crowding -- the same plan and the same twist every round. twisted_vias names such a via, the embed hands the solve a
cut for it (the change is not to stand there again), and the loop counts a round with such a cut as answered by the
solve.

A run is (lane, k): the lane's run between its changes k - 1 and k; a via (lane, its change's index).

1. a via moved across the other lane's run, the two runs crossing: named;
2. ...moved, on its own side of that run: not named, though the runs cross;
3. ...not moved: not named;
4. no crossing: none, wherever the vias stand;
5. the via at the run's other end is asked too, and each via named once;
6. a run of one point (nothing to cross) is passed over;
7. the embed hands the solve a cut marked `twist` for each, and the loop reads a round with one as answered.
"""
import ast
import os
import sys

from shapely.geometry import LineString

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, 'awx', 'whole_embed.py')).read()
LOOP = open(os.path.join(ROOT, 'awx', 'whole_route.py')).read()

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


# twisted_vias alone, out of the script (the script builds a board's whole embedding when it is imported)
fn = next((n for n in ast.parse(SRC).body if isinstance(n, ast.FunctionDef) and n.name == 'twisted_vias'), None)
if fn is None:
    raise SystemExit('BROKEN TEST: awx/whole_embed.py has no twisted_vias')
ns = {'LineString': LineString}
exec(compile(ast.Module(body=[fn], type_ignores=[]), 'whole_embed.py', 'exec'), ns)
twisted_vias = ns['twisted_vias']

# lane B runs straight up x = 1; lane A's run 1 goes from its via (A, 0) rightward to its via (A, 1)
B_RUN = [(1.0, -2.0), (1.0, 2.0)]


def layout(a0, a1):
    return {('A', 1): [a0, a1], ('B', 0): B_RUN}


X = [(('A', 1), ('B', 0), (1.0, 0.0))]          # the two runs cross
REF = {('A', 0): (2.0, 0.0), ('A', 1): (3.0, 0.0)}      # the plan: both of A's vias right of B, no crossing

moved = {('A', 0): (0.0, 0.0), ('A', 1): (3.0, 0.0)}    # (A, 0) set left of B: across its run
check('1. a via moved across the other lane\'s run, the runs crossing: named',
      twisted_vias(X, layout(moved[('A', 0)], moved[('A', 1)]), moved, REF) == [('A', 0)],
      twisted_vias(X, layout(moved[('A', 0)], moved[('A', 1)]), moved, REF))
ref_l = {('A', 0): (0.5, 0.0), ('A', 1): (3.0, 0.0)}    # the plan itself has A's first via left of B
same = {('A', 0): (0.0, 0.5), ('A', 1): (3.0, 0.0)}     # ...moved, and still left of it
check('2. a via moved on its own side of that run: not named, though the runs cross',
      twisted_vias(X, layout(same[('A', 0)], same[('A', 1)]), same, ref_l) == [])
check('3. a via that was not moved: not named', twisted_vias(X, layout(ref_l[('A', 0)], ref_l[('A', 1)]), dict(ref_l),
                                                             ref_l) == [])
check('4. no crossing: none', twisted_vias([], layout(moved[('A', 0)], moved[('A', 1)]), moved, REF) == [])
both = {('A', 0): (0.0, 0.0), ('A', 1): (0.2, 0.5)}     # both of the run's vias set left of B
got = twisted_vias(X + X, layout(both[('A', 0)], both[('A', 1)]), both, REF)
check('5. the via at the run\'s other end too, each once', got == [('A', 0), ('A', 1)], got)
one = {('A', 1): [moved[('A', 0)], moved[('A', 1)]], ('B', 0): [(1.0, 0.0)]}
check('6. a run of one point is passed over', twisted_vias(X, one, moved, REF) == [])
# the other way round: B's own via moved across A's run
REF_B = {('B', 0): (1.0, 2.0), ('A', 0): (0.0, 0.0), ('A', 1): (3.0, 0.0)}
mov_b = {**REF_B, ('B', 0): (1.0, -1.0)}
plb = {('A', 1): [(0.0, 0.0), (3.0, 0.0)], ('B', 0): [(1.0, -2.0), (1.0, -1.0)], ('B', 1): [(1.0, -1.0), (1.0, 3.0)]}
check('5. ...and either run\'s via, whichever lane moved it',
      twisted_vias([(('A', 1), ('B', 1), (1.0, 0.0))], plb, mov_b, REF_B) == [('B', 0)],
      twisted_vias([(('A', 1), ('B', 1), (1.0, 0.0))], plb, mov_b, REF_B))

# 7. the calls are there
check('7. the embed asks it of the layout\'s crossings and hands the solve a cut marked `twist`',
      '_xv = twisted_vias(same_layer_crossings(PL), PL, VIA, VIA_REF)' in SRC and "'twist': True}" in SRC)
check('7. ...and the loop reads a round with such a cut as answered by the solve',
      "any(c_.get('twist') for c_ in vcuts)" in LOOP)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
