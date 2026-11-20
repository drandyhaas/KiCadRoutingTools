#!/usr/bin/env python3
"""A costly ball a later model leaves -- a repair after a check, the model solved again for a lesson of the field laid
-- gets its search, however many earlier ones cleared theirs (awx/group_ends.py, COSTLY_TRIES).

  python3 tests/test_622_costly_tries.py

The search after a repair frees the lanes nearest each costly ball (a plane ball fanned out, a tooth on the source's far
face) and searches them again. Counted as a budget for the whole run, two such searches -- each clearing its plane ball
-- left nothing for the last repairs, which handed three plane balls on as bare stubs; the lanes laid round one of them
walled it in. Counted a ball, a new costly ball always gets its search, and a ball two searches cannot clear is let be.

1. the count is kept a ball (a Counter keyed by the ball), not one number for the run;
2. ...the balls searched are those with tries left, and each of them is counted;
3. ...the branch runs whenever the tiers are, with no run-wide count in its condition.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, 'awx', 'group_ends.py')).read()
BAD = []
N = [0]


def check(what, ok):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what)


check('1. a count a ball', 'costly_tries = collections.Counter()' in SRC)
check('2. the balls with tries left, each counted',
      'fan_ = [b_ for b_ in costly_now() if costly_tries[b_] < COSTLY_TRIES]' in SRC
      and 'costly_tries.update(fan_)' in SRC)
check('3. no run-wide count in the condition',
      'costly_tries < COSTLY_TRIES' not in SRC and 'costly_tries += 1' not in SRC)
print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
