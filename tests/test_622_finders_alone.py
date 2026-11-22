#!/usr/bin/env python3
"""On two routing layers the whole solve's plan-finding search runs with CP-SAT's neighbourhood workers off
(awx/whole_solve.py: finders_alone, solve's `once`).

  python3 tests/test_622_finders_alone.py

An interleaved search shares its deterministic work evenly among all its subsolvers. Beside the three complete
searches the plan-finders name, CP-SAT adds its neighbourhood searches once a plan is known, and they then take most
of every batch -- while only a complete search raises the bound. With them off, each batch is a turn of each complete
search: the same workers, the same batches, the same reproducible search. On two layers the complete searches both
find the best plans and prove them; with more layers the neighbourhood searches find the plans, and the search is
left as it is.

1. finders_alone: true of the plan-finders' own list (and of a copy of it) on two layers, false on more, and false of
   the bound-raising probe's and of a window's (no list);
2. the search: `once` turns the neighbourhood workers off for the plan-finders alone, and changes nothing else of
   them -- the batches the caller gave, interleaved, no clauses shared; the plan-finders' batches and workers are
   what they were;
3. a small model solved both ways under the plan-finders' parameters: the same optimum, proved, and with the
   neighbourhood workers off the solver's own log names no neighbourhood subsolver.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if os.path.basename(
    os.path.dirname(os.path.abspath(__file__))) == 'tests' else os.getcwd()
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
try:
    from ortools.sat.python import cp_model
except ImportError:
    print('SKIP: ortools is not installed (the whole solve plans with CP-SAT)')
    sys.exit(77)

import whole_solve as ws  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


# 1. which searches
check('1. the plan-finders\' own list, on two layers', ws.finders_alone(ws.FALLBACK, 2)
      and ws.finders_alone(list(ws.FALLBACK), 2))
check('1. not on more layers', not any(ws.finders_alone(ws.FALLBACK, n) for n in (3, 4, 6, 8)))
check('1. not the bound-raising probe\'s', not ws.finders_alone(ws.SUBSOLVERS, 2))
check('1. not a window\'s search (no list)', not ws.finders_alone([], 2))

# 2. the search's parameters
SRC = open(os.path.join(ROOT, 'awx', 'whole_solve.py')).read()
m = re.search(r'    def once\(subs, batches, known_(?:, model=None)?(?:, keep=None)?\):\n(.*?)\n        stall = ', SRC, re.S)
body = m.group(1) if m else ''
check('2. once: the neighbourhood workers off for the plan-finders alone',
      re.search(r'if finders_alone\(subs, NL\):\n\s+s_\.parameters\.use_lns = False', body) is not None
      and body.count('use_lns') == 1)
check('2. ...its batches the caller\'s, the search interleaved, no clauses shared',
      all(t in body for t in ('s_.parameters.max_num_deterministic_batches = batches',
                              's_.parameters.interleave_search = True', 's_.parameters.share_glue_clauses = False',
                              's_.parameters.share_binary_clauses = False', 's_.parameters.subsolvers.extend(subs)',
                              's_.parameters.num_workers = SOLVE_WORKERS')))
check('2. the plan-finders\' batches, workers and probe as they were',
      (ws.SOLVE_BATCHES, ws.PROBE_BATCHES, ws.SOLVE_WORKERS, ws.FALLBACK) == (100, 10, 4, ['quick_restart', 'no_lp', 'core']),
      (ws.SOLVE_BATCHES, ws.PROBE_BATCHES, ws.SOLVE_WORKERS, ws.FALLBACK))
check('2. the only use of the switch in the stage', SRC.count('.use_lns') == 1, SRC.count('.use_lns'))


# 3. a small model, both ways
def small():
    mdl = cp_model.CpModel()
    n = 9
    x = [mdl.NewIntVar(0, n - 1, f'x{i}') for i in range(n)]
    mdl.AddAllDifferent(x)
    for i in range(n - 1):
        b = mdl.NewBoolVar(f'b{i}')
        mdl.Add(x[i] - x[i + 1] >= 2).OnlyEnforceIf(b)
        mdl.Add(x[i + 1] - x[i] >= 2).OnlyEnforceIf(b.Not())
    mdl.Minimize(sum((i % 3 + 1) * x[i] for i in range(n)))
    return mdl


def run(alone):
    s = cp_model.CpSolver()
    s.parameters.num_workers = ws.SOLVE_WORKERS
    s.parameters.subsolvers.extend(ws.FALLBACK)
    s.parameters.interleave_search = True
    s.parameters.max_num_deterministic_batches = ws.SOLVE_BATCHES
    s.parameters.share_glue_clauses = False
    s.parameters.share_binary_clauses = False
    if alone:
        s.parameters.use_lns = False
    lines = []
    s.parameters.log_search_progress = True
    s.parameters.log_to_stdout = False
    s.log_callback = lines.append
    st = s.Solve(small())
    return st, s.ObjectiveValue(), '\n'.join(lines)


st0, v0, log0 = run(False)
st1, v1, log1 = run(True)
check('3. the same optimum, proved, both ways', st0 == st1 == cp_model.OPTIMAL and v0 == v1, (st0, v0, st1, v1))
sub1 = re.search(r'interleaved subsolvers?: \[([^\]]*)\]', log1)
check('3. the neighbourhood workers off: the solver names none', '_lns' not in (sub1.group(1) if sub1 else '')
      and 'full problem subsolvers: [core, no_lp, quick_restart]' in log1, sub1.group(0) if sub1 else None)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
