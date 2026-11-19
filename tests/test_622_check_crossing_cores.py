#!/usr/bin/env python3
"""A group's check names EVERY set of crossings with no window between them, not the first alone
(awx/whole_solve.py: solve(cores=True); awx/group_ends.py: the check and its lessons).

  python3 tests/test_622_check_crossing_cores.py

The group's ends model puts each bus's ends to that bus's whole solve; with no plan, the solve names the crossings
whose windows cannot all hold (a core of its assumptions). One core a check: a bus whose ends held three such sets was
refused at three checks running, a repair of the model between each. The solve now sets the first core's crossings
free and asks again, core after core, until a plan stands with them free or what is left has no plan whatever the
windows, and the model learns each core as a refused state of its own.

1. the solve takes `cores` and, asked for it with no rules core, frees each core's crossings and asks again, each ask
   one worker, stopping at its first plan, its work bounded as the check's own;
2. ...it returns every core it found (`cores`) and the union as the core;
3. the group's check asks the solve for its cores;
4. ...reads each core's lanes as a group of its own;
5. ...and teaches the model one refused state per group, each over that group's lanes alone.
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOLVE = open(os.path.join(ROOT, 'awx', 'whole_solve.py')).read()
GROUP = open(os.path.join(ROOT, 'awx', 'group_ends.py')).read()

BAD = []
N = [0]


def check(what, got, want=True):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


sol = next(n for n in ast.parse(SOLVE).body if isinstance(n, ast.FunctionDef) and n.name == 'solve')
check('1. the solve takes `cores`, off unless asked', any(a.arg == 'cores' for a in sol.args.args)
      and ast.literal_eval(sol.args.defaults[[a.arg for a in sol.args.args][-len(sol.args.defaults):].index('cores')])
      is False)
i = SOLVE.index('if cores and not RLIT and out_[\'core\']:')
blk = SOLVE[i:i + 2200]
check('1. ...asked with no rules core, each core\'s crossings set free and the rest asked again',
      'free_w |= set(nxt_)' in blk and "if WLIT[i_] not in free_w" in blk)
check('1. ...each ask one worker, stopping at its first plan, its work bounded as the check\'s',
      'num_workers = 1' in blk and 'stop_after_first_solution = True' in blk
      and 'max_deterministic_time = CHECK_WORK' in blk)
check('1. ...ending where a plan stands with them free, or no core is left',
      'if s5_.Solve(m) != cp_model.INFEASIBLE:' in blk and 'if not nxt_:' in blk)
check('2. every core returned, and the union as the core',
      "out_['cores'] = found_" in blk and "out_['core'] = sorted(free_w)" in blk)

i = GROUP.index('J = whole_solve.solve(ctx, dest_, front=')
check('3. the group\'s check asks the solve for its cores', 'cores=True' in GROUP[i:i + 250])
check('4. each core\'s lanes a group of its own',
      "out['groups'] = [sorted({l_ for k_ in c_ for l_ in k_}) for c_ in (J or {}).get('cores') or [out['core']]]"
      in GROUP)
i = GROUP.index("for g_ in chk.get('groups') or [list(chk['lanes'])]:")
check('5. one refused state taught per group, over its own lanes',
      "for l_ in g_ if l_ in s_}" in GROUP[i:i + 400] and "L_['nogoods'].append(ng_)" in GROUP[i:i + 400])

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
