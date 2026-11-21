#!/usr/bin/env python3
"""Each bus brings its reading of its bench to the coupled solve, and a waiting bus looks often (awx/whole_joint.py:
save_plan, brought, put; awx/whole_route.py: joint_sync, joint_loop_sync, JOINT_POLL).

  python3 tests/test_622_joint_brought_plan.py

The coupled solve builds every bus's model again in ONE model. For that it planned every bus's bench itself -- the
board parsed, the taut strings relaxed again, bus after bus in its own process: three quarters of the time a four-bus
group's coupled model took to build, for what each bus's own process had in hand. A bus now writes its plan of its
bench beside its request (whole_joint --plan), and the coupled solve reads it -- only while it is the plan of the very
bench the bus's settings name. And the negotiation's waits looked every five seconds; they look every half second,
which asks that what they wait for is written whole.

1. save_plan writes the plan and, beside it, what it is of: the bench's board (path, size, time written), its nets,
   its destination;
2. brought gives that plan back, read as the stage cache reads one (through whole_ctx._guard);
3. ...and none where a bus brought none, where the mark is missing, where the board was written since, or where the
   settings name other nets or another destination: the coupled solve plans that bench itself, as before;
4. put writes a file whole: beside it, then moved onto it -- nothing partial at the path, nothing left beside it;
5. a request carries its plan, in a round's first exchange and in a loop's (the calls are there), and every wait of
   the negotiation sleeps JOINT_POLL.
"""
import json
import os
import re
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)

import whole_ctx  # noqa: E402
import whole_joint as wj  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


SRC = open(os.path.join(ROOT, 'awx', 'whole_route.py')).read()
real = whole_ctx.plan, whole_ctx.bench, whole_ctx._guard
with tempfile.TemporaryDirectory() as td:
    board = os.path.join(td, 'bench.kicad_pcb')
    open(board, 'w').write('(kicad_pcb)\n')
    bench = {'v': (board, ['N1', 'N2'], 'U2')}
    guarded = []
    whole_ctx.bench = lambda: bench['v']
    whole_ctx.plan = lambda: (types.SimpleNamespace(paths={'N1': [(0.0, 1.5)]}, chi=-1), [])
    whole_ctx._guard = lambda out, nets, dest: (guarded.append((list(nets), dest)), out)[1]
    try:
        p = os.path.join(td, 'plan.pkl')
        wj.save_plan(p)
        mark = json.load(open(p + '.json'))
        check('1. the plan written, and what it is of beside it', os.path.isfile(p) and mark[0] == os.path.abspath(board)
              and mark[1] == os.path.getsize(board) and mark[3:] == [['N1', 'N2'], 'U2'], mark)
        got = wj.brought(p)
        check('2. brought: the plan itself', got is not None and got[0].paths == {'N1': [(0.0, 1.5)]} and got[0].chi == -1)
        check('2. ...read through the guard, on the bench\'s nets and destination', guarded == [(['N1', 'N2'], 'U2')],
              guarded)
        check('3. no plan brought: none', wj.brought(None) is None and wj.brought(os.path.join(td, 'no.pkl')) is None)
        bench['v'] = (board, ['N1', 'N3'], 'U2')
        check('3. ...the settings name other nets: none', wj.brought(p) is None)
        bench['v'] = (board, ['N1', 'N2'], 'U9')
        check('3. ...or another destination: none', wj.brought(p) is None)
        bench['v'] = (board, ['N1', 'N2'], 'U2')
        check('3. (the same bench again: the plan)', wj.brought(p) is not None)
        open(board, 'w').write('(kicad_pcb (segment))\n')
        check('3. ...the board written since: none', wj.brought(p) is None)
        wj.save_plan(p)
        os.remove(p + '.json')
        check('3. ...no mark beside it: none', wj.brought(p) is None)
        # 4. put
        q = os.path.join(td, 'B.joint.json')
        wj.put({'bus': 'B', 'joint': True}, q)
        check('4. put: the file whole at its path, nothing left beside it',
              json.load(open(q)) == {'bus': 'B', 'joint': True} and sorted(os.listdir(td)) ==
              sorted(['bench.kicad_pcb', 'plan.pkl', 'B.joint.json']), sorted(os.listdir(td)))
    finally:
        whole_ctx.plan, whole_ctx.bench, whole_ctx._guard = real

# 5. the calls are there
check('5. a round\'s request and a loop\'s each bring their plan',
      SRC.count("run(['whole_joint.py', '--plan', plan_], env") == 2 and SRC.count("'plan': plan_ if os.path.isfile") == 2)
body = SRC[SRC.index('def joint_sync('):SRC.index('def round_edits(')]
check('5. every wait of the negotiation sleeps JOINT_POLL', len(re.findall(r'time\.sleep\(JOINT_POLL\)', body)) == 8
      and not re.search(r'time\.sleep\(\d', body), re.findall(r'time\.sleep\([^)]*\)', body))
import whole_route  # noqa: E402
check('5. ...half a second', whole_route.JOINT_POLL == 0.5, whole_route.JOINT_POLL)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
