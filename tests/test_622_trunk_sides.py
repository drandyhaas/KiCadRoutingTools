#!/usr/bin/env python3
"""No lane's ends ask it to change sides of the trunk where the frame has no room for it
(awx/whole_frame.py: side_switch; awx/whole_ends.py: the joint model's rule).

  python3 tests/test_622_trunk_sides.py

The frame lays a bus's trunk on the straight spine from the source's box to the destination's, a lane's offset
changing at most SLOPE_MAX a unit of the spine (whole_geo). A lane leaving the source by one side face and going round
the destination by the other side changes sides between the two boxes. Where they stand diagonally, their facing
corners nearly level along the spine, it has a few columns to do it in: the geometry paid 9.6 mm of bound, and four
lanes were laid on one another.

1. side_switch: two boxes side by side, and offset diagonally with room between them, let a lane change sides either
   way;
2. ...two boxes whose facing corners stand nearly level along the spine do not, the way that crosses between those
   corners -- and the other way still does;
3. ...mirrored, the other way is the one refused;
4. the geometry's slope cap is the frame's (one constant);
5. the joint model holds a lane off a tooth on a refused face together with a berth on the refused ring, one row a
   lane, not one a pair of options, and not for a group's destination (its own frame).
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'awx'), os.path.join(ROOT, 'py_router')]
import whole_frame  # noqa: E402

BAD = []
N = [0]


def check(what, got, want=True):
    N[0] += 1
    if got != want:
        BAD.append(what)
    print(('ok   ' if got == want else 'FAIL ') + what + ('' if got == want else f': {got!r}, wanted {want!r}'))


both = {('S', 'N'): True, ('N', 'S'): True}
check('1. side by side: either way', whole_frame.side_switch((0, 0, 10, 10), (15, 0, 25, 10)), both)
check('1. ...offset diagonally with room between', whole_frame.side_switch((0, 0, 10, 10), (12, 8, 22, 18)), both)
# (a source box to the north-west, a destination to the south-east, the source's south-east corner and the
# destination's north-west corner nearly level along the spine between their centres)
SB, DB = (65.0, -67.7, 81.8, -50.9), (84.8, -57.9, 95.2, -47.5)
got = whole_frame.side_switch(SB, DB)
check('2. corners nearly level: from the south face to the north ring, no room', got[('S', 'N')], False)
check('2. ...the other way, room', got[('N', 'S')], True)
mir = lambda b: (b[0], -b[3], b[2], -b[1])  # noqa: E731
got = whole_frame.side_switch(mir(SB), mir(DB))
check('3. mirrored: the north face to the south ring refused', got[('N', 'S')], False)
check('3. ...and the south face to the north ring not', got[('S', 'N')], True)

GEO = open(os.path.join(ROOT, 'awx', 'whole_geo.py')).read()
check('4. the geometry\'s slope cap the frame\'s', 'K_MAX = whole_frame.SLOPE_MAX' in GEO)

ENDS = open(os.path.join(ROOT, 'awx', 'whole_ends.py')).read()
i = ENDS.index('whole_frame.side_switch(SB, DB)')
blk = ENDS[i - 300:i + 1400]
check('5. the joint model reads side_switch, not for a group', 'if E.group is None:' in blk)
check('5. ...a tooth by its face, a berth by its ring',
      'whole_frame.face(E.T[l_][i][1], SB)' in blk and 'whole_frame.ring_side(E.B[l_][j][1], DB, cut)' in blk)
check('5. ...one row a lane: not both a refused face\'s tooth and a refused ring\'s berth',
      'm.Add(sum(ti_) + sum(bj_) <= 1)' in blk)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
