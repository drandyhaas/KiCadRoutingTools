#!/usr/bin/env python3
"""The conflict test reaches each via at its own size (select_moves._conflict, `reach` a pair).

An array's rules (group_ends.via_groups -> conflict_groups) hold a via off another move's run by THAT via's radius plus
a clearance and half a track. group_ends.verify_runs checks those rules against select_moves._conflict, which took one
reach for both moves of a pair -- the larger -- so a via-in-pad's finer via was tested at a standard via's reach: on a
memory array's whole menus 0.4 % of the sampled pairs read as "the test alone" though the rules were right (a 0.36 mm
via 0.4 mm off a 0.1524 mm track's centre clears a 0.1016 mm clearance by 0.04 mm). Pins:

  1. a scalar reach is unchanged: both vias at it;
  2. a pair (m's, om's): m's via against om's run at m's reach, om's via against m's run at om's;
  3. the two orders agree (the pair follows its moves);
  4. verify_runs passes each move's own reach (no `max(` of the two left in its call).

    python3 tests/test_622_conflict_reach_per_via.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'awx'), os.path.join(ROOT, 'py_router')]

import select_moves as sm  # noqa: E402
from escape_moves import Move  # noqa: E402

F, B = 'F.Cu', 'B.Cu'
fails = []


def check(label, cond):
    print(('  ok   ' if cond else '  FAIL ') + label)
    if not cond:
        fails.append(label)


# a via-in-pad move whose via stands 0.4 mm off the other move's run, and that run: a surface move along y = 0
V = Move('V', 'via_in_pad', 'up', B, (1.0, 3.0), 1, legs=[((1.0, 0.4), (1.0, 3.0), B)], site=(1.0, 0.4))
R = Move('R', 'surface', 'left', F, (-2.0, 0.0), 0, legs=[((3.0, 0.0), (-2.0, 0.0), F)])
FINE, STD = 0.18 + 0.1016 + 0.0762, 0.225 + 0.1016 + 0.0762      # a fine and a standard via's reach

check('1. a scalar reach at the standard via: held apart', sm._conflict(V, R, strict=True, stack=True, xing=2, reach=STD))
check('1. a scalar reach at the fine via: clear', not sm._conflict(V, R, strict=True, stack=True, xing=2, reach=FINE))
check('2. the via at its own (fine) reach, the run\'s move a standard one: clear',
      not sm._conflict(V, R, strict=True, stack=True, xing=2, reach=(FINE, STD)))
check('2. the via given the standard reach: held apart',
      sm._conflict(V, R, strict=True, stack=True, xing=2, reach=(STD, FINE)))
check('3. the other order, the pair swapped with its moves: clear',
      not sm._conflict(R, V, strict=True, stack=True, xing=2, reach=(STD, FINE)))
check('3. the other order, the via given the standard reach: held apart',
      sm._conflict(R, V, strict=True, stack=True, xing=2, reach=(FINE, STD)))
src = open(os.path.join(ROOT, 'awx', 'group_ends.py'), encoding='utf-8').read()
i = src.index('def verify_runs(')
body = src[i:src.index('\ndef ', i + 1)]
check('4. verify_runs reaches each move\'s via at its own size', 'reach=(ra + ext, rb + ext)' in body
      and 'max(ra, rb)' not in body)
print('PASS' if not fails else f'FAIL: {len(fails)} check(s)')
sys.exit(1 if fails else 0)
