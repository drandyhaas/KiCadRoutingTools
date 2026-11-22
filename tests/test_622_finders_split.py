#!/usr/bin/env python3
"""On more routing layers than two the whole solve's plan-finding batches are TWO searches: the first with every
neighbourhood worker CP-SAT adds, the rest from its plan and bound with two of them alone beside the complete searches
(awx/whole_solve.py: finders_split, FIND_LNS_BATCHES, FIND_LNS_KEEP, solve's plan-finders).

  python3 tests/test_622_finders_split.py

An interleaved search shares its deterministic work evenly among its subsolvers. A complete search runs one task at a
time, of about a unit of work, a neighbourhood task about a tenth of one, and a batch is twelve tasks: with every
neighbourhood worker on, the complete searches -- the only ones that raise the bound -- get about a quarter of the
work. With two neighbourhood workers beside them they get a task about every other batch each, the two the rest.
On two layers the search is the complete searches' alone (finders_alone), as before.

1. finders_split: true on more routing layers than two, false on two;
2. the budget: the two searches' batches are the plan-finders' SOLVE_BATCHES, the first FIND_LNS_BATCHES; two
   neighbourhood workers kept, by their CP-SAT names; the constants (100, 50, 10, 4 workers, the FALLBACK list) as
   they were;
3. the stage: `once` takes `keep` and, given it, filters the subsolvers to the search's own complete ones and those
   kept (filter_subsolvers), finders_alone's switch untouched and the one use of it; the plan-finders' first search
   is FIND_LNS_BATCHES where finders_split says so, else SOLVE_BATCHES, every neighbourhood worker on; the second,
   only when the first did not prove its plan, the rest of the batches with FIND_LNS_KEEP, from the first's plan
   (every variable hinted) and with its bound a constraint; the plan kept the second's where no dearer;
4. a small model both ways under the plan-finders' parameters (one search; the two searches as the stage runs them):
   the same optimum, proved; the second search's own log names the kept neighbourhood workers and no other, and the
   plan-finders' complete searches all three.
"""
import math
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


# 1. where
check('1. more layers than two', all(ws.finders_split(n) for n in (3, 4, 6, 8)))
check('1. not on two layers', not ws.finders_split(2))

# 2. the budget
check('2. the first search a part of the plan-finders\' batches', 0 < ws.FIND_LNS_BATCHES < ws.SOLVE_BATCHES,
      (ws.FIND_LNS_BATCHES, ws.SOLVE_BATCHES))
check('2. two neighbourhood workers kept, by name', len(ws.FIND_LNS_KEEP) == 2
      and all(k.endswith('_lns') for k in ws.FIND_LNS_KEEP), ws.FIND_LNS_KEEP)
check('2. the constants as they were',
      (ws.SOLVE_BATCHES, ws.FIND_LNS_BATCHES, ws.PROBE_BATCHES, ws.SOLVE_WORKERS, ws.FALLBACK)
      == (100, 50, 10, 4, ['quick_restart', 'no_lp', 'core']))

# 3. the stage
SRC = open(os.path.join(ROOT, 'awx', 'whole_solve.py')).read()
m = re.search(r'    def once\(subs, batches, known_, model=None, keep=None\):\n(.*?)\n        stall = ', SRC, re.S)
body = m.group(1) if m else ''
check('3. once takes keep: the subsolvers filtered to its own complete ones and those kept',
      re.search(r'if keep is not None:\n\s+s_\.parameters\.filter_subsolvers\.extend\(list\(subs\) \+ list\(keep\)\)',
                body) is not None and SRC.count('filter_subsolvers') == 1)
check('3. ...finders_alone\'s switch untouched, the one use of it',
      re.search(r'if finders_alone\(subs, NL\):\n\s+s_\.parameters\.use_lns = False', body) is not None
      and SRC.count('.use_lns') == 1)
check('3. run passes keep to once', 'def run(subs, batches, known=-math.inf, keep=None):' in SRC
      and 's_, st_ = once(subs, batches, known, keep=keep)' in SRC)
fb = SRC[SRC.index('        t_first = secs\n'):SRC.index("        print(f'   plan-finding workers: [")]
check('3. the first search: FIND_LNS_BATCHES where finders_split says so, else SOLVE_BATCHES, every worker',
      'split_ = finders_split(NL)' in fb
      and 'sv, st, proved, known, secs = run(FALLBACK, FIND_LNS_BATCHES if split_ else SOLVE_BATCHES, known)' in fb)
check('3. the second only when the first did not prove, the rest of the batches, FIND_LNS_KEEP alone',
      'if split_ and not proved:' in fb
      and 'run(FALLBACK, SOLVE_BATCHES - FIND_LNS_BATCHES, known, FIND_LNS_KEEP)' in fb)
i_h, i_b, i_r = fb.find('m.AddHint(v_, sv.Value(v_))'), fb.find('m.Add(OBJ >= int(math.ceil(known - 1e-6)))'), \
    fb.find('known, FIND_LNS_KEEP)')
check('3. ...from the first\'s plan, every variable hinted, and its bound a constraint', 0 < i_h < i_r and 0 < i_b < i_r)
check('3. ...the plan kept the second\'s where no dearer, proved on the bound of both',
      'sv2_.ObjectiveValue() <= sv.ObjectiveValue()' in fb
      and 'math.floor(sv.ObjectiveValue() / W_V) <= math.floor(known / W_V)' in fb)


# 4. a small model, both ways
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
    return mdl, x


def search(mdl, batches, keep=None):
    s = cp_model.CpSolver()
    s.parameters.num_workers = ws.SOLVE_WORKERS
    s.parameters.subsolvers.extend(ws.FALLBACK)
    s.parameters.interleave_search = True
    s.parameters.max_num_deterministic_batches = batches
    s.parameters.share_glue_clauses = False
    s.parameters.share_binary_clauses = False
    if keep is not None:
        s.parameters.filter_subsolvers.extend(list(ws.FALLBACK) + list(keep))
    lines = []
    s.parameters.log_search_progress = True
    s.parameters.log_to_stdout = False
    s.log_callback = lines.append
    st = s.Solve(mdl)
    return s, st, '\n'.join(lines)


m0, _ = small()
s0, st0, _ = search(m0, ws.SOLVE_BATCHES)
m1, x1 = small()
s1, st1, _ = search(m1, ws.FIND_LNS_BATCHES)
if st1 in (cp_model.OPTIMAL, cp_model.FEASIBLE):
    m1.ClearHints()
    for i in range(len(m1.Proto().variables)):
        m1.AddHint(m1.GetIntVarFromProtoIndex(i), s1.Value(m1.GetIntVarFromProtoIndex(i)))
m1.Add(sum((i % 3 + 1) * x1[i] for i in range(len(x1))) >= int(math.ceil(s1.BestObjectiveBound() - 1e-6)))
s2, st2, log2 = search(m1, ws.SOLVE_BATCHES - ws.FIND_LNS_BATCHES, ws.FIND_LNS_KEEP)
check('4. the same optimum, proved, one search and two', st0 == cp_model.OPTIMAL and st2 == cp_model.OPTIMAL
      and s0.ObjectiveValue() == s2.ObjectiveValue(), (st0, s0.ObjectiveValue(), st2, s2.ObjectiveValue()))
sub2 = re.search(r'interleaved subsolvers?: \[([^\]]*)\]', log2)
got = sorted(x.strip() for x in sub2.group(1).split(',')) if sub2 else None
check('4. the second search: the kept neighbourhood workers and no other', got == sorted(ws.FIND_LNS_KEEP), got)
check('4. ...beside the plan-finders\' three complete searches', 'full problem subsolvers: [core, no_lp, quick_restart]'
      in log2)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
