#!/usr/bin/env python3
"""A run taken the other side of a piece of fixed copper where that is the shorter way (awx/whole_embed.py:
shorten_edges, passage_slack, jump_run's `gain`).

  python3 tests/test_622_embed_shorter_side.py

whole_embed reads each lane's reference line onto the cells of its layer's free space, and the cells a run crosses say
which side of each piece of fixed copper it passes. That side is the reference's reading -- taken where its line stood
in the copper's bar or beside it -- and the rounds after it move a run's points along the cells it is in, so a run read
round the far side of a pad stayed there, however much longer. shorten_edges tries each end run of each passage the
other side of the copper it stands next to (the move a run of a passage with no room already makes), and keeps the
way that leaves the run shorter by a pitch at the least. Only a run bent enough near the passage to gain that much is
asked: passage_slack, the stretch the move may leave less the straight line between its two ends.

A run here is a list of points, each a cell edge's two vertices (a, b) and its place along it; vertex 0 stands on one
piece of copper and vertex 1 on another, and a point of the passage is one on the edge between the two.

1. a run straight through the passage has no slack;
2. a run bent round one side has its stretch's length less the straight line;
3. the stretch reaches JUMP_BACK either side of the passage and no further;
4. a run that does not go through the passage has none;
5. the embed asks the move only of a run with a pitch of slack, keeps a way only where it is shorter by that much,
   looks for it through passages with room, and --no-shorter leaves it out (the calls are there).
"""
import ast
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = open(os.path.join(ROOT, 'awx', 'whole_embed.py')).read()

BAD = []
N = [0]


def check(what, got, want):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


# passage_slack alone, out of the script (the script builds a board's whole embedding when it is imported)
fn = next(n for n in ast.parse(SRC).body if isinstance(n, ast.FunctionDef) and n.name == 'passage_slack')
RUN, XY = {}, {}
ns = {'math': math, 'RUN': RUN, 'JUMP_BACK': 2.0, 'rings_of': lambda L_: ([0, 1, -1, -1, -1, -1], None),
      'run_xy': lambda key: XY[key]}
exec(compile(ast.Module(body=[fn], type_ignores=[]), 'whole_embed.py', 'exec'), ns)
passage_slack = ns['passage_slack']
PK = ('F.Cu', 0, 1)


def run(key, line, edges):
    """a run whose line is `line` (its start, a point for each of `edges`, its end) -- each edge the two vertices
    its point stands on"""
    assert len(line) == len(edges) + 2
    RUN[key] = {'pts': [{'k': 'P', 'a': a, 'b': b} for a, b in edges]}
    XY[key] = line


def close(a, b):
    return abs(a - b) < 1e-9


run('straight', [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)], [(2, 3), (0, 1), (2, 3)])
check('1. a run straight through the passage has no slack', close(passage_slack('straight', PK), 0.0), True)

run('bent', [(0, 0), (1, 0), (1, 1), (2, 1), (2, 0), (3, 0)], [(2, 3), (2, 3), (0, 1), (2, 3)])
# (its point in the passage is (1, 1); within 2.0 before it the line goes back to (0, 0), within 2.0 after to (2, 0):
#  a stretch of 4.0 between two points 2.0 apart)
check('2. a run bent round one side: its stretch less the straight line', close(passage_slack('bent', PK), 2.0), True)

run('long', [(-9, 0), (0, 0), (1, 0), (1, 1), (2, 1), (2, 0), (3, 0), (12, 0)],
    [(2, 3), (2, 3), (2, 3), (0, 1), (2, 3), (2, 3)])
check('3. the stretch reaches JUMP_BACK either side of the passage and no further',
      close(passage_slack('long', PK), 2.0), True)

run('elsewhere', [(0, 0), (1, 0), (1, 1), (2, 1), (2, 0)], [(2, 3), (4, 5), (2, 3)])
check('4. a run that does not go through the passage has none', passage_slack('elsewhere', PK), 0.0)

i_s = SRC.index('def shorten_edges():')
body = SRC[i_s:SRC.index('JUMPED = ', i_s)]
check('5. the move is asked only of a run with a pitch of slack',
      'if passage_slack(e_[0], pk) < SHORTER:' in body and 'SHORTER = P_MIN' in SRC, True)
check('5. ...of each end run of a passage, the copper it stands next to there',
      'for e_, rg_ in ((lst[0], pk[1]), (lst[-1], pk[2])):' in body
      and 'jump_run(pk, e_, rg_, on, gain=SHORTER, x0=x0, now=now)' in body, True)
i_j = SRC.index('def jump_run(')
jump = SRC[i_j:SRC.index('def jump_edges():')]
check('5. ...a way kept only where it leaves the run that much shorter',
      'most = old.length + JUMP_MAX if gain is None else old.length - gain' in jump
      and 'ok = new.length <= most and' in jump, True)
check('5. ...looked for through passages with room for the run',
      'if gain is not None and vr[a_] >= 0 and vr[b_] >= 0 and vr[a_] != vr[b_] and not has_room(vr[a_], vr[b_]):'
      in jump, True)
check('5. ...and --no-shorter leaves the move out',
      "SHORTENED = [] if '--no-shorter' in OPT else shorten_edges()" in SRC, True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
