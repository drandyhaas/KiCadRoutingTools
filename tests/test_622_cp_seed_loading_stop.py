#!/usr/bin/env python3
"""The bus step's solver stops a deterministic hinted search at a proof made while loading the model (awx/cp_seed.py:
solver, _solver_class), where CP-SAT 9.15 would abort the process.

  python3 tests/test_622_cp_seed_loading_stop.py

A deterministic search of several workers (interleave_search, num_workers > 1) of a HINTED model that a worker finds
infeasible as it loads it: the worker's hint search runs with no search strategy built and the library aborts ("Check
failed: heuristics.fixed_search != nullptr"). The model here is such a one -- a chain of implications from a literal
held true to one held false, the presolve off so that the load is where the proof is made.

1. control: a plain CpSolver dies on it, for that reason (where the library no longer does, said so, and the rest
   still run);
2. the bus step's solver answers INFEASIBLE, its process alive, and a log reader of its own still reads every line,
   the proof's among them;
3. on a hinted model WITH states, the same deterministic search answers the same -- status, objective, every value --
   through the bus step's solver as through a plain one, and the solver's parameters and log reader are as they were;
4. a one-worker search and an unhinted one are left as they are: their log is not turned on.
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
try:
    import ortools  # noqa: F401  (CP-SAT is the subject)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

fails = []


def check(cond, msg):
    print(('ok   ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


# the model a worker proves infeasible as it loads it, solved in a process of its own (the library may abort it)
CHILD = r'''
import sys
sys.path.insert(0, sys.argv[2])
from ortools.sat.python import cp_model
import cp_seed
m = cp_model.CpModel()
xs = [m.NewBoolVar(f'x{i}') for i in range(30)]
for a, b in zip(xs, xs[1:]):
    m.AddImplication(a, b)
m.Add(xs[0] == 1)
m.Add(xs[-1] == 0)
m.Minimize(sum(xs))
for x in xs:
    m.AddHint(x, 0)
s = cp_seed.solver() if sys.argv[1] == 'seed' else cp_model.CpSolver()
s.parameters.num_workers = 4
s.parameters.subsolvers.extend(['no_lp', 'core'])
s.parameters.interleave_search = True
s.parameters.max_num_deterministic_batches = 50
s.parameters.cp_model_presolve = False
lines = []
if sys.argv[1] == 'seed':
    s.parameters.log_search_progress = True
    s.parameters.log_to_stdout = False
    s.log_callback = lines.append
r = s.Solve(m)
print('STATUS', s.StatusName(r))
print('LINES', len(lines), sum(1 for l_ in lines if l_.startswith('#Done') and '[loading]' in l_))
'''


def child(kind):
    return subprocess.run([sys.executable, '-c', CHILD, kind, AWX], capture_output=True, text=True, errors='replace',
                          timeout=300)


# ---- 1. the control: the library's abort
p = child('plain')
if p.returncode == 0:
    print('note 1. control: this CP-SAT no longer aborts on such a model -- ' + p.stdout.strip().replace('\n', '; '))
else:
    check('fixed_search' in p.stderr, f'1. control: a plain solver dies on the model ({p.returncode}) for the '
          f'library\'s reason -- {[ln for ln in p.stderr.splitlines() if "Check failed" in ln][:1]}')

# ---- 2. the bus step's solver
p = child('seed')
out = dict((ln.split(' ', 1) + [''])[:2] for ln in p.stdout.splitlines() if ln[:6] in ('STATUS', 'LINES '))
check(p.returncode == 0 and out.get('STATUS') == 'INFEASIBLE',
      f'2. the bus step\'s solver answers INFEASIBLE, alive -- exit {p.returncode}, {out.get("STATUS")}'
      + ('' if p.returncode == 0 else f'; {p.stderr.strip().splitlines()[-1:]}'))
n_lines, n_done = (int(x) for x in (out.get('LINES') or '0 0').split())
check(n_lines > 0 and n_done >= 1, f'2. its own log reader read the lines, the proof\'s among them -- {n_lines} '
      f'line(s), {n_done} proof(s) made while loading')

# ---- 3. a model with states: the same answer
from ortools.sat.python import cp_model  # noqa: E402
import cp_seed  # noqa: E402


def model(n=30):
    # (an assignment whose neighbouring rows' diagonal choices pay a bonus: a search its batches leave unproved)
    m = cp_model.CpModel()
    x = [[m.NewBoolVar(f'x{i}_{j}') for j in range(n)] for i in range(n)]
    for i in range(n):
        m.AddExactlyOne(x[i])
        m.AddExactlyOne(x[j][i] for j in range(n))
    cost = [[(7 * i + 11 * j + i * j) % 17 for j in range(n)] for i in range(n)]
    terms = [cost[i][j] * x[i][j] for i in range(n) for j in range(n)]
    for i in range(n - 1):
        for j in range(n - 1):
            p = m.NewBoolVar('')
            m.AddImplication(p, x[i][j])
            m.AddImplication(p, x[i + 1][j + 1])
            terms.append(-5 * p)
    m.Minimize(sum(terms))
    for i in range(n):
        for j in range(n):
            m.AddHint(x[i][j], int(i == j))
    return m


def solve(s, m):
    s.parameters.num_workers = 4
    s.parameters.subsolvers.extend(['no_lp', 'core'])
    s.parameters.interleave_search = True
    s.parameters.max_num_deterministic_batches = 4
    r = s.Solve(m)
    vals = list(s.ResponseProto().solution) if r in (cp_model.OPTIMAL, cp_model.FEASIBLE) else None
    return s.StatusName(r), s.ObjectiveValue() if vals else None, vals


m1, m2 = model(), model()
a = solve(cp_model.CpSolver(), m1)
s3 = cp_seed.solver()
b = solve(s3, m2)
check(a[2] is not None and a == b, f'3. a model with states, its search stopped at its batches: the same status, '
      f'objective and values either way -- {a[:2]} vs {b[:2]}')
if a[0] != 'FEASIBLE':
    print(f'note 3. the search was {a[0]}, not stopped unproved: a weaker check of the same answer')
check(not s3.parameters.log_search_progress and s3.parameters.log_to_stdout and s3.log_callback is None,
      '3. the solver\'s parameters and log reader as they were after the solve -- '
      f'log {s3.parameters.log_search_progress}, to stdout {s3.parameters.log_to_stdout}, reader {s3.log_callback}')

# ---- 4. searches the abort cannot reach are left alone
seen = {}


class Spy(cp_seed._solver_class()):
    def solve(self, model, solution_callback=None):
        seen[self.tag] = self.parameters.log_search_progress
        return super().solve(model, solution_callback)


# (the guard turns the log on inside solve(): read it from the library's side, at the solve)
orig = cp_model.CpSolver.solve


def peek(self, model, solution_callback=None):
    seen[getattr(self, 'tag', '?') + ':inner'] = self.parameters.log_search_progress
    return orig(self, model, solution_callback)


cp_model.CpSolver.solve = peek
try:
    for tag, workers, hinted in (('one-worker', 1, True), ('unhinted', 4, False), ('hinted', 4, True)):
        mm = model(8)
        if not hinted:
            mm.ClearHints()
        sp = Spy()
        sp.tag = tag
        sp.parameters.num_workers = workers
        sp.parameters.interleave_search = True
        sp.parameters.max_num_deterministic_batches = 5
        sp.Solve(mm)
finally:
    cp_model.CpSolver.solve = orig
check(seen.get('one-worker:inner') is False and seen.get('unhinted:inner') is False,
      f'4. a one-worker search and an unhinted one: their log left off -- {seen}')
check(seen.get('hinted:inner') is True, f'4. (and the hinted search of four workers had it on, to read) -- {seen}')

print(f'{"FAILED" if fails else "passed"}: {len(fails)} failure(s)')
sys.exit(1 if fails else 0)
