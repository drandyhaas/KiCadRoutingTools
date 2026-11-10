#!/usr/bin/env python3
"""A solve the library may die in runs in a process of its own (awx/cp_seed.py: solve_apart).

  python3 tests/test_622_solve_apart.py

CP-SAT 9.15 aborts the process on Linux where a one-worker search stopped at its first solution finds its model
infeasible as it loads (glibc: "corrupted size vs. prev_size", from the destructor of the solver's disjunctive
precedences). The bus step runs its stages in ONE process, so that abort ended the whole step with no board and no
message; another bus's first plan died of a segmentation fault in a worker's propagation, the layer rule off.
whole_solve's first plan takes its solves through cp_seed.solve_apart: the model and the solver's parameters written
out, solved by a child process, the answer read back; a child that dies is a solve with no answer.

1. the same answer as the solver gives in this process: status, every variable's value, a linear expression's value
   (a negated literal in it) -- on a model with intervals, a no-overlap, a hint, one worker stopped at its first plan;
2. the parameters go with it: the child is given the step's seed (BUS_SEED) and the solver's other parameters;
3. an infeasible model: INFEASIBLE and no values;
4. a child that DIES (stood in for: exit -6, as the abort ends it): status UNKNOWN, no values, nothing raised, the
   status's name says the process died and by which signal, and its last lines reach stderr;
5. whole_solve takes every solve of its first plan through it (the call is there, and no solve in this process).
"""
import contextlib
import io
import os
import subprocess
import sys

try:
    from ortools.sat.python import cp_model
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import cp_seed  # noqa: E402

BAD = []
N = [0]


def check(ok, what):
    N[0] += 1
    print(('PASS: ' if ok else 'FAIL: ') + what)
    if not ok:
        BAD.append(what)


def model():
    m = cp_model.CpModel()
    x, y, b = m.NewIntVar(0, 9, 'x'), m.NewIntVar(0, 9, 'y'), m.NewBoolVar('b')
    m.Add(x + y >= 7)
    m.Add(x >= 6).OnlyEnforceIf(b)
    m.AddNoOverlap([m.NewIntervalVar(x, 2, m.NewIntVar(0, 20, 'ex'), ''),
                    m.NewIntervalVar(y, 2, m.NewIntVar(0, 20, 'ey'), '')])
    m.AddHint(x, 3)
    m.AddHint(y, 8)
    return m, (x, y, b), 3 * x + 2 * y + 7 * b.Not() + 5


def solver():
    s = cp_seed.solver()
    s.parameters.num_workers = 1
    s.parameters.interleave_search = True
    s.parameters.max_num_deterministic_batches = 20
    s.parameters.stop_after_first_solution = True
    return s


# ---- 1. the same answer
os.environ['BUS_SEED'] = '5'
m, vs, expr = model()
s_here = solver()
st_here = s_here.Solve(m)
a, st = cp_seed.solve_apart(m, solver())
nv = len(m.Proto().variables)
here = [s_here.Value(m.GetIntVarFromProtoIndex(i)) for i in range(nv)]
check(st == st_here and st in (cp_model.OPTIMAL, cp_model.FEASIBLE) and a.values == here,
      f'the same status ({a.StatusName(st)}) and the same {nv} values as the solver gives in this process')
check(a.Value(expr) == s_here.Value(expr) and all(a.Value(v) == s_here.Value(v) for v in vs),
      f'a linear expression\'s value from them ({a.Value(expr)}), and each variable\'s')

# ---- 2. the parameters go with it
seen = {}
run0 = subprocess.run


def spy(argv, **kw):
    seen['parameters'] = open(argv[-2], encoding='utf-8').read()
    seen['model'] = os.path.getsize(argv[-3])
    return run0(argv, **kw)


cp_seed.subprocess.run = spy
try:
    cp_seed.solve_apart(m, solver())
finally:
    cp_seed.subprocess.run = run0
check('random_seed: 5' in seen.get('parameters', '') and 'stop_after_first_solution: true' in seen['parameters']
      and 'num_workers: 1' in seen['parameters'] and seen['model'] > 0,
      'the child is given the model and the parameters: the step\'s seed, one worker, stopped at its first plan')

# ---- 3. an infeasible model
m2, (x2, y2, _b2), _e2 = model()
m2.Add(x2 + y2 <= 1)
a2, st2 = cp_seed.solve_apart(m2, solver())
check(st2 == cp_model.INFEASIBLE and a2.values is None and a2.StatusName(st2) == 'INFEASIBLE',
      'an infeasible model: INFEASIBLE, no values')

# ---- 4. a child that dies


def dies(argv, **kw):
    return subprocess.CompletedProcess(argv, -6, stdout='', stderr='corrupted size vs. prev_size\nFatal Python error')


cp_seed.subprocess.run = dies
err = io.StringIO()
try:
    with contextlib.redirect_stderr(err):
        a3, st3 = cp_seed.solve_apart(m, solver())
finally:
    cp_seed.subprocess.run = run0
check(st3 == cp_model.UNKNOWN and a3.values is None and a3.died == -6
      and st3 not in (cp_model.OPTIMAL, cp_model.FEASIBLE),
      'a child that dies: status UNKNOWN, no values, nothing raised')
check('died' in a3.StatusName(st3) and 'signal 6' in a3.StatusName(st3) and 'corrupted size' in err.getvalue(),
      f'its status says so ("{a3.StatusName(st3)}"), its last lines on stderr')

# ---- 5. the first plan's one-worker solve goes through it
src = open(os.path.join(ROOT, 'awx', 'whole_solve.py'), encoding='utf-8').read()
i = src.index('def _first(model_, batches_, first_=True, one_=False):')
body = src[i:src.index('t1_ = time.time()', i)]
check('return cp_seed.solve_apart(model_, sf_)' in body and 'sf_.Solve(model_)' not in body,
      'whole_solve\'s first plan: each of its solves is solved apart, none in this process')

print(f'\n{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
