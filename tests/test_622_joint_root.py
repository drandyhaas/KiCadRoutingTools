#!/usr/bin/env python3
"""A coupled solve records no root for a bus's own re-solves (awx/whole_solve.py's root, awx/whole_joint.py).

  python3 tests/test_622_joint_root.py

A solve that proves its plan with no geometry cut records it as its ROOT (the plan's JSON, `root`): the objective a
re-solve of the same model cannot go below, added there as a floor. Buses off one array are solved COUPLED too, every
bus's model built into one and solved by one solver, each bus's part written out by its own model's write-out -- which
read the objective off that solver: all the buses' objectives and the couplings' prices together. A bus whose coupled
plan was proved carried that sum as its own root; its loop's first re-solve, the model still the root's, was floored
millions of vias' worth above any plan it has, had no plan in no time, and the loop ended after one round on the plan
it held with a lane open.

1. the coupled solve writes each bus's part out with the solver of the whole (the call is there);
2. a root is recorded only by a solve of the bus's own model: the record's condition names it;
3. why: of two parts solved in one model, the whole's objective is past what one part alone can reach -- the part
   floored there has no plan, and has one with no such floor.
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
try:
    from ortools.sat.python import cp_model
except ImportError:
    print('SKIP: ortools is not installed (the solves plan with CP-SAT)')
    sys.exit(77)

N, BAD = [0], []


def check(what, got, want):
    N[0] += 1
    ok = got == want
    if not ok:
        BAD.append(what)
    print(f'{"ok  " if ok else "FAIL"} {what}' + ('' if ok else f': got {got!r}, want {want!r}'))


# ---- 1. the coupled solve's write-out of a bus's part: the whole's solver
src_j = open(os.path.join(ROOT, 'awx', 'whole_joint.py')).read()
calls = [n for n in ast.walk(ast.parse(src_j)) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
         and n.func.attr == 'finish']
check('1. each bus\'s part is written out by its own model\'s write-out, given one solver',
      [ast.get_source_segment(src_j, c.args[0]) for c in calls], ['sv'])

# ---- 2. the root's record: a solve of the bus's own model alone
src_s = open(os.path.join(ROOT, 'awx', 'whole_solve.py')).read()
tree = ast.parse(src_s)
solve = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'solve')
check('2. the solve is told when its model is a part of a coupled one', 'joint' in [a.arg for a in solve.args.args], True)
recs = [n for n in ast.walk(solve) if isinstance(n, ast.Assign) and ast.get_source_segment(src_s, n.targets[0]) == "J['root']"]
check('2. one record of the root', len(recs), 1)
fresh = [n for n in ast.walk(recs[0].value) if isinstance(n, ast.IfExp) and isinstance(n.body, ast.Dict)]
check('2. ...a fresh one only under a condition', len(fresh), 1)
cond = ast.get_source_segment(src_s, fresh[0].test) if fresh else ''
check('2. ...that the solve proved its plan, and that it is no coupled solve\'s',
      ('proved' in cond, 'joint is None' in cond), (True, True))
check('2. ...its objective the solver\'s own', "sv.ObjectiveValue()" in ast.get_source_segment(src_s, fresh[0].body), True)


# ---- 3. why: a part floored at the whole's objective has no plan
def part(m, tag, n=4):
    """a part: n choices of 0..3 vias each, at least 5 vias in all; its objective their sum"""
    xs = [m.NewIntVar(0, 3, f'{tag}{i}') for i in range(n)]
    m.Add(sum(xs) >= 5)
    return xs, sum(xs)


def status(floor=None):
    m = cp_model.CpModel()
    _xs, obj = part(m, 'a')
    if floor is not None:
        m.Add(obj >= floor)
    m.Minimize(obj)
    sv = cp_model.CpSolver()
    sv.parameters.num_workers = 1
    return sv.StatusName(sv.Solve(m)), (sv.ObjectiveValue() if floor is None else None)


whole = cp_model.CpModel()
(xa, oa), (xb, ob) = part(whole, 'a'), part(whole, 'b')
both = whole.NewBoolVar('same')                    # (a coupling: the parts' first choices alike, at a price)
whole.Add(xa[0] == xb[0]).OnlyEnforceIf(both.Not())
whole.Minimize(oa + ob + 1000 * both)
sw = cp_model.CpSolver()
sw.parameters.num_workers = 1
st_w = sw.StatusName(sw.Solve(whole))
check('3. the whole proved, its objective both parts\'', (st_w, sw.ObjectiveValue()), ('OPTIMAL', 10.0))
check('3. the part alone: its own plan', status(), ('OPTIMAL', 5.0))
check('3. ...floored at its own proof: a plan', status(5)[0], 'OPTIMAL')
check('3. ...floored at a whole past its reach (its choices hold 12 at most): none',
      status(int(sw.ObjectiveValue()) + 3)[0], 'INFEASIBLE')

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
