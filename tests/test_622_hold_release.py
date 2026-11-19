#!/usr/bin/env python3
"""A lane a group's model holds to its ends' layers is freed of them by another net's laid track across it there
(awx/whole_solve.py: freed_of_hold).

  python3 tests/test_622_hold_release.py

On a group's ends the model chooses the layers of the lanes that cross another bus's lane, and each bus's own solve
holds those ends on them: the layers are what keeps the two buses apart at the crossings the model had. A track laid
across such a lane on the very layer its end is held on is a crossing the model had not: the loop holds the lane off
that layer there, firm, and held at its end as well the lane has nowhere to change -- the cut broke every round and the
net shipped open. The solve frees such a lane of the group's layers and chooses its ends' layers itself.

1. a firm cut against another net's track on the layer the lane's end is held on: freed;
2. the same cut on a layer neither end is held on: not freed (the lane changes on its way, as any other);
3. a cut that is not firm (an island the lane may be sent round): not freed;
4. a cut against one of the bus's OWN nets (two of its lanes short of their pitch at one's end): not freed;
5. a coupling with another bus's planned lane, a keep-out, a part to pass under, a layer wall: not freed -- the
   group's model had each of them with the hold in force;
6. a lane the group does not hold: not named;
7. a cut naming a pair's leg frees its pair, by either leg's layers;
8. the solve asks it of its cut files and lets the freed lanes' ends be via ends (the call is there).
"""
import os
import sys

try:
    from ortools.sat.python import cp_model  # noqa: F401  (whole_solve imports it)
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import whole_solve  # noqa: E402

BAD = []
N = [0]


def check(what, got, want):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


def cut(lane, island, blocked, firm=True, **more):
    c = {'lane': lane, 'island': island, 'layer': 0, 'box': [0, 0, 1, 1], 'blocked': blocked}
    if firm:
        c['firm'] = True
    c.update(more)
    return c


HELD = {'A', 'B', 'PQ'}                     # the lanes the group holds ('PQ' a pair)
OWN = {'A', 'B', 'C', 'PQ', 'P', 'Q'}       # the bus's own lanes and legs
ENDS = {'A': {3, 1}, 'B': {3, 6}, 'PQ': {2, 5}}
LEGS = {'P': 'PQ', 'Q': 'PQ'}
free = lambda cuts: whole_solve.freed_of_hold(cuts, HELD, OWN, ENDS, LEGS)

check('1. another net\'s track on the layer an end is held on frees the lane', free([cut('A', 'CLK', [3])]), {'A'})
check('1. ...on either end\'s layer, among several layers', free([cut('A', 'CLK', [1, 6])]), {'A'})
check('2. a track on a layer neither end is held on does not', free([cut('A', 'CLK', [6])]), set())
check('3. a cut that is not firm does not', free([cut('A', 'CLK', [3], firm=False)]), set())
check('4. a cut against one of the bus\'s own lanes does not', free([cut('A', 'C', [3])]), set())
check('4. ...nor against a leg of one of its pairs', free([cut('A', 'P', [3])]), set())
for lab in ('bus U9:CLK', 'keepout K0', 'under J1', 'wall In2.Cu', 'area 0', 'track CLK'):
    check(f'5. a cut labelled {lab!r} does not', free([cut('A', lab, [3])]), set())
check('6. a lane the group does not hold is not named', free([cut('C', 'CLK', [3])]), set())
check('7. a pair is freed by a cut naming a leg', free([cut('P', 'CLK', [5])]), {'PQ'})
check('7. ...and by one naming the pair', free([cut('PQ', 'CLK', [2])]), {'PQ'})
check('each lane by its own cuts', free([cut('A', 'CLK', [6]), cut('B', 'CLK', [6]), cut('B', 'bus U9:D0', [3])]), {'B'})
check('no cuts, no lane freed', free([]), set())

src = open(os.path.join(ROOT, 'awx', 'whole_solve.py')).read()
at = src.index('freed_ = freed_of_hold(')
check('8. the solve asks it of its cut files', "for fn_ in cuts for c_ in json.load(open(fn_)).get('lcuts', [])" in
      src[at:at + 400], True)
check('8. ...and the freed lanes leave the held ones before the via ends are named',
      at < src.index('held_ -= freed_') < src.index('VEND = {(n, k_): True for n in M'), True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
