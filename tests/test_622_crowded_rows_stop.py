#!/usr/bin/env python3
"""The crowded-ends repair reads a named lane's findings in the STOPPING round's audit when the round the loop goes on
from has none (awx/whole_route.py: crowded_rows, audit_static_rows; repair_ends reads them).

  python3 tests/test_622_crowded_rows_stop.py

A loop's held round passed its audit; a pair could not be laid at a dive there, the loop went on, and its next round
stopped ENDS CROWDED on two lanes against a plane's via at their teeth. The repair reads the held round's audit -- the
loop goes on from that round -- and found no row of either lane: nothing was moved, and a lane shipped open.

1. audit_static_rows reads a STATIC row (its shortfall, lane, layer, kind, object, point) and a pair's static DIVE row;
2. crowded_rows: the held round's rows as they are, and a NAMED lane with none there read from the stopping round's;
3. ...a lane with rows in the held round keeps those alone, and a lane not named is not read from the stopping round;
4. ...one audit for both rounds (the held round is the stopping one) reads nothing twice;
5. repair_ends asks crowded_rows, with the stopping round its last hot file's.
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    import whole_route as wr  # noqa: E402
except ImportError as exc:
    print('SKIP: %s' % exc)
    sys.exit(77)

BAD = []
N = [0]


def check(what, got, want):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


d = tempfile.mkdtemp()
held = os.path.join(d, 'p1.audit')
stop = os.path.join(d, 'p2.audit')
open(held, 'w').write('PITCH 0 pair(s) short\n'
                      'STATIC LANE_A B -0.030/0.146 copper NET_X at (10.00,20.00)\n')
open(stop, 'w').write('STATIC LANE_B In2.Cu -0.096/0.146 via PWR at (164.40,100.45)\n'
                      'STATIC LANE_C B +0.121/0.146 via PWR at (164.38,100.23)\n'
                      'STATIC LANE_A F -0.200/0.146 pad U9.3 at (11.00,21.00)\n'
                      'STATIC LANE_D F -0.050/0.146 copper NET_Y at (12.00,22.00)\n'
                      'DIVE LANE_B (164.50,100.50) pair static -0.010/0.200 (via PWR)\n')

r_ = wr.audit_static_rows(stop)
check('1. a STATIC row: shortfall, lane, layer (In2.Cu), kind, object, point, not a dive',
      tuple(round(x, 3) if isinstance(x, float) else x for x in r_[0][:4]) + (r_[0][4], r_[0][5], r_[0][6]),
      (-0.242, 'LANE_B', 'In2.Cu', 'via', ['PWR'], (164.40, 100.45), False))
check('1. ...B read as B.Cu', r_[1][2], 'B.Cu')
check('1. a pair\'s static DIVE row read too, its layer every layer (None)',
      [(x[1], x[2], x[6]) for x in r_ if x[6]], [('LANE_B', None, True)])

named = [('LANE_B', 'teeth'), ('LANE_C', 'teeth'), ('LANE_A', 'teeth')]
rows, from_stop, aud_stop = wr.crowded_rows(d, named, held, '2')
check('2. the stopping round\'s audit is the one read for the named lanes', os.path.basename(aud_stop), 'p2.audit')
check('2. the named lanes with no row in the held round read from the stopping round\'s', sorted(from_stop),
      ['LANE_B', 'LANE_C'])
check('2. ...their rows, the dive\'s with them', sorted((x[1], x[6]) for x in rows if x[1] in ('LANE_B', 'LANE_C')),
      [('LANE_B', False), ('LANE_B', True), ('LANE_C', False)])
check('3. a lane with rows in the held round keeps those alone',
      [(x[1], x[3]) for x in rows if x[1] == 'LANE_A'], [('LANE_A', 'copper')])
check('3. a lane the stop did not name is not read from the stopping round', [x for x in rows if x[1] == 'LANE_D'], [])
rows1, fs1, _a1 = wr.crowded_rows(d, named, stop, '2')
check('4. the held round the stopping one: its rows once, none read twice', (len(rows1), sorted(fs1)), (5, []))

src = open(os.path.join(ROOT, 'awx', 'whole_route.py')).read()
body = src[src.index('def repair_ends('):]
body = body[:body.index('\ndef ', 10)]
check('5. repair_ends asks crowded_rows with the stopping round', 'crowded_rows(loopd, named, aud, i_stop)' in body
      and 'i_stop = i_' in body, True)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
