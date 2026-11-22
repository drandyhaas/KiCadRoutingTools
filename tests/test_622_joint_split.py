#!/usr/bin/env python3
"""The coupled solve's batches are two searches, as the whole solve's plan-finders' on more routing layers
(awx/whole_joint.py main's search; awx/whole_solve.py FIND_LNS_BATCHES, FIND_LNS_KEEP).

  python3 tests/test_622_joint_split.py

The coupled search runs the plan-finders' complete searches interleaved with every neighbourhood worker CP-SAT adds;
with them on, the complete searches -- the only ones that raise the bound -- get about a quarter of the work. Its
batches are split as each bus's own plan-finders' are: whole_solve.FIND_LNS_BATCHES with every neighbourhood worker,
then the rest of BATCHES from the first's plan and bound with whole_solve.FIND_LNS_KEEP alone.

1. the budget: the coupled search's BATCHES as they were, the first search a part of them;
2. the search: one helper builds both searches -- the plan-finders' complete searches, interleaved, no clauses
   shared, the batches given, and given `keep` the subsolvers filtered to those and the kept ones; the first
   FIND_LNS_BATCHES with every worker; the second, unless the first proved its plan, the rest with FIND_LNS_KEEP,
   from the first's plan (every variable hinted) and with its bound a constraint; the plan kept the second's where
   no dearer, and the bound reported the higher of the two.
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if os.path.basename(
    os.path.dirname(os.path.abspath(__file__))) == 'tests' else os.getcwd()
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
try:
    from ortools.sat.python import cp_model  # noqa: F401
except ImportError:
    print('SKIP: ortools is not installed (the coupled solve plans with CP-SAT)')
    sys.exit(77)

import whole_joint as wj  # noqa: E402
import whole_solve as ws  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


# 1. the budget
check('1. the coupled search\'s batches as they were', wj.BATCHES == 100, wj.BATCHES)
check('1. the first search a part of them', 0 < ws.FIND_LNS_BATCHES < wj.BATCHES, (ws.FIND_LNS_BATCHES, wj.BATCHES))

# 2. the search
SRC = open(os.path.join(ROOT, 'awx', 'whole_joint.py')).read()
m = re.search(r'    def search\(batches, keep=None\):\n(.*?)\n        return s_, s_\.Solve\(m\)\n', SRC, re.S)
body = m.group(1) if m else ''
check('2. one helper builds both searches: the plan-finders\' complete searches, interleaved, no clauses shared',
      all(t in body for t in ('s_.parameters.subsolvers.extend(ws.FALLBACK)', 's_.parameters.interleave_search = True',
                              's_.parameters.max_num_deterministic_batches = batches',
                              's_.parameters.share_glue_clauses = False', 's_.parameters.share_binary_clauses = False',
                              's_.parameters.num_workers = WORKERS')))
check('2. ...given keep, the subsolvers filtered to those and the kept ones',
      re.search(r'if keep is not None:\n\s+s_\.parameters\.filter_subsolvers\.extend\(list\(ws\.FALLBACK\) \+ '
                r'list\(keep\)\)', body) is not None)
check('2. the one solve of the coupled search, in the helper', SRC.count('s_.Solve(m)') == 1
      and SRC.count('.Solve(m)') == 2, (SRC.count('s_.Solve(m)'), SRC.count('.Solve(m)')))
rest = SRC[SRC.index('        return s_, s_.Solve(m)\n'):SRC.index('    secs = time.time() - t0\n    bus_events.emit(')]
check('2. the first search FIND_LNS_BATCHES, every worker', 's_, st = search(ws.FIND_LNS_BATCHES)' in rest)
check('2. the second unless the first proved, the rest of the batches, FIND_LNS_KEEP alone',
      'if st != cp_model.OPTIMAL:' in rest and 'search(BATCHES - ws.FIND_LNS_BATCHES, ws.FIND_LNS_KEEP)' in rest)
i_h, i_b, i_r = rest.find('m.AddHint('), rest.find('m.Add(OBJ >= int(math.ceil(bound_ - 1e-6)))'), \
    rest.find('ws.FIND_LNS_KEEP)')
check('2. ...from the first\'s plan, every variable hinted, and its bound a constraint', 0 < i_h < i_r and 0 < i_b < i_r)
check('2. ...the plan kept the second\'s where no dearer, the bound the higher of the two',
      's2_.ObjectiveValue() <= s_.ObjectiveValue()' in rest and 'bound_ = max(bound_, s2_.BestObjectiveBound())' in rest
      and "'bound': round(bound_ / W_V, 1)" in SRC and "(bound {bound_ / W_V:.1f})" in SRC)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
