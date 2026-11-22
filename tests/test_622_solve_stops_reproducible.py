#!/usr/bin/env python3
"""The whole solve's early stops end its search AT THE END OF A BATCH, so one model stops at one plan on every run,
alone or beside other runs (awx/whole_solve.py: solve's `once` and `run`, and its check).

  python3 tests/test_622_solve_stops_reproducible.py

CP-SAT's interleaved search runs its tasks in batches and merges what they found between two batches, in an order
that does not depend on which task finished first. A new plan, and a rise of the bound, are written to the log by the
worker that found it, in the middle of a batch; a '#Model' line is written between two batches. A search stopped from
a line of the first kind kept whichever of its batch's plans the other workers had handed in by then: one model's
probe, stopped so, ended at three different plans in three runs side by side. So:

1. the proof (the plan's whole vias no more than the bound's) is CP-SAT's own gap test, absolute_gap_limit a whole via
   less one, met at a batch's end -- `once` stops nothing on a plan's or a bound's line;
2. the stall stops only on a '#Model' line, and a rise of the bound sets it back by its callback (the '#Bound' lines
   are thinned by the log's rate limit past twenty);
3. `run` reads the proof off the plan and bound the search ended with, asks the question left where the gap was met
   across a whole via, and reports a plan ended by the gap test as a plan (FEASIBLE), not as one proved optimal;
4. the check on the solve's workers is bounded by batches, not by work, and not stopped at a worker's first plan;
5. a small model solved under the probe's parameters with the gap test, in three processes at once: the same plan,
   value for value, and the same status, value and bound in each.
"""
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if os.path.basename(
    os.path.dirname(os.path.abspath(__file__))) == 'tests' else os.getcwd()
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
try:
    from ortools.sat.python import cp_model  # noqa: F401
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


SRC = open(os.path.join(ROOT, 'awx', 'whole_solve.py')).read()
m_ = re.search(r'\n    def once\((.*?)\n    def run\(', SRC, re.S)
ONCE = m_.group(1) if m_ else ''
m_ = re.search(r'\n    def run\((.*?)\n    # ---- the WARM START', SRC, re.S)
RUN = m_.group(1) if m_ else ''
check('0. once and run found in the solve', bool(ONCE) and bool(RUN))

# 1. the proof
check('1. the proof is CP-SAT\'s gap test, a whole via less one', 's_.parameters.absolute_gap_limit = W_V - 1' in ONCE)
check('1. ...no stop on a plan\'s or a bound\'s line: nothing in once reads a line\'s best or bound',
      'best:' not in ONCE and "'#Bound'" not in ONCE)

# 2. the stall
stops = [mm.start() for mm in re.finditer(r'StopSearch\(\)', ONCE)]
model_ifs = [mm.start() for mm in re.finditer(r"line\.startswith\('#Model'\)", ONCE)]
check('2. one stop in once, the stall\'s, under a \'#Model\' line',
      len(stops) == 1 and any(i < stops[0] for i in model_ifs) and 'SOLVE_STALL' in ONCE[model_ifs[-1]:stops[0]],
      (len(stops), len(model_ifs)))
check('2. ...a rise of the bound sets it back by the bound callback', re.search(
    r"def _risen\(\w*\):\n\s+stall\['n'\] = 0", ONCE) is not None and 's_.best_bound_callback = _risen' in ONCE)
check('2. ...a new plan sets it back', re.search(r"if re\.match\(r'#\\d\+\\s', line\):\n\s+stall\['n'\], stall\['plan'\] = 0, True",
                                                 ONCE) is not None)
check('2. ...not the plan-finders\'', 'subs is not FALLBACK' in ONCE)
check('2. no other stop in the solve but its group ends\' (none in whole_solve outside once)',
      SRC.count('StopSearch()') == 1, SRC.count('StopSearch()'))

# 3. run
check('3. run: the proof read off the plan and the bound it ended with', re.search(
    r'pr_ = st_ in OK_ and math\.floor\(s_\.ObjectiveValue\(\) / W_V\) <= math\.floor\(b_ / W_V\)', RUN) is not None)
check('3. ...the gap met across a whole via: the bound\'s whole vias asked alone, in the same batches',
      'if st_ in OK_ and not pr_ and s_.ObjectiveValue() - b_ < W_V:' in RUN
      and 'mq_.Add(OBJ <= W_V * (f_ + 1) - 1)' in RUN and 'once(subs, batches, b_, model=mq_)' in RUN)
check('3. ...none there proves the plan\'s', 'elif stq_ == cp_model.INFEASIBLE:' in RUN
      and 'b_, pr_ = W_V * (f_ + 1), True' in RUN)
check('3. ...a plan the gap test ended is FEASIBLE', re.search(
    r'if st_ == cp_model\.OPTIMAL and s_\.ObjectiveValue\(\) > s_\.BestObjectiveBound\(\):\n(\s+#[^\n]*\n)*\s+st_ = cp_model\.FEASIBLE',
    RUN) is not None)

# 4. the check on the solve's workers
i_ = SRC.find('s_.parameters.num_workers = 1 if WLIT or RLIT else SOLVE_WORKERS')
blk = SRC[i_:i_ + 2200]
j_ = blk.find('        else:')
one_, many_ = blk[:j_], blk[j_:blk.find('        st_ = None')]
check('4. the check on one worker (a core\'s): its first plan, its work CHECK_WORK',
      'stop_after_first_solution = True' in one_ and 'max_deterministic_time = CHECK_WORK' in one_)
check('4. ...on the solve\'s workers: interleaved, no clauses shared, CHECK_WORK_BATCHES batches, not its first plan',
      all(t in many_ for t in ('interleave_search = True', 'share_glue_clauses = False', 'share_binary_clauses = False',
                               'max_num_deterministic_batches = CHECK_WORK_BATCHES'))
      and 'max_deterministic_time' not in many_ and 'stop_after_first_solution' not in many_)
check('4. ...CHECK_WORK_BATCHES a count', isinstance(getattr(ws, 'CHECK_WORK_BATCHES', None), int)
      and ws.CHECK_WORK_BATCHES > 0)

# 5. a small model, three processes at once
CHILD = r'''
import json, sys, hashlib
from ortools.sat.python import cp_model
W, n, k = 10 ** 6, 36, 6
m = cp_model.CpModel()
x = [[m.NewBoolVar('') for _ in range(k)] for _ in range(n)]
for i in range(n):
    m.AddExactlyOne(x[i])
over = []
for j in range(k):
    load = sum((3 + (i * 7 + j * 5) % 11) * x[i][j] for i in range(n))
    o = m.NewIntVar(0, 400, '')
    m.Add(o >= load - 32)
    over.append(o)
cost = sum(((i * 13 + j * 29) % 97) * x[i][j] for i in range(n) for j in range(k))
pair = []
for i in range(0, n - 1, 2):
    for j in range(k):
        b = m.NewBoolVar('')
        m.AddBoolOr([x[i][j].Not(), x[i + 1][j].Not(), b])
        pair.append(b)
m.Minimize(W * sum(over) + 1000 * sum(pair) + cost)
s = cp_model.CpSolver()
s.parameters.num_workers = 4
s.parameters.subsolvers.extend(sys.argv[1].split(','))
s.parameters.interleave_search = True
s.parameters.max_num_deterministic_batches = int(sys.argv[2])
s.parameters.share_glue_clauses = False
s.parameters.share_binary_clauses = False
s.parameters.random_seed = 2
s.parameters.absolute_gap_limit = W - 1
st = s.Solve(m)
ok = st in (cp_model.OPTIMAL, cp_model.FEASIBLE)
print(json.dumps({'status': s.StatusName(st), 'obj': s.ObjectiveValue() if ok else None,
                  'bound': s.BestObjectiveBound() if ok else None,
                  'sol': hashlib.sha256(json.dumps(list(s.ResponseProto().solution)).encode()).hexdigest() if ok else None}))
'''
procs = [subprocess.Popen([sys.executable, '-c', CHILD, ','.join(ws.SUBSOLVERS), str(ws.PROBE_BATCHES)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) for _ in range(3)]
outs = []
for p in procs:
    o, e = p.communicate(timeout=600)
    try:
        outs.append(json.loads(o.strip().splitlines()[-1]))
    except (ValueError, IndexError):
        outs.append({'error': (e or o)[-300:]})
check('5. three solves at once: each a plan', all(o.get('sol') for o in outs), outs)
check('5. ...the same plan, value for value', len({o.get('sol') for o in outs}) == 1, [o.get('sol', '')[:12] for o in outs])
check('5. ...the same status, value and bound', len({(o.get('status'), o.get('obj'), o.get('bound')) for o in outs}) == 1,
      outs)

print(f'\n{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
