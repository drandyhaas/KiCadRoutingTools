#!/usr/bin/env python3
"""The interval of a column a lane is bounded to where its reference cuts a box's corner (corridor.held_interval).

  python3 tests/test_622_held_interval.py

whole_geo bounds each lane's column to one free interval of offset outside the arrays' boxes: the one its reference
lies in, else the one it was held to a column before. On the zynq DDR bus (the six-rail bench, round 1), DDR3_A8's
berth on U2's top face stands ON the destination's grown box, and its reference -- the taut path round the balls --
runs just inside it, north of the top row. Where the box's corner first splits the column the lane's previous interval
was the whole column, and the most overlap with it held the lane SOUTH of the box over its last columns: the bound
and slope rows are elastic, and the geometry ran A8 up across U2's west column of balls to its berth, into A6's via
in the gap between them (2 statics, A8 open in both the chain's and the joint's round 1).

The columns below are A8's own, as whole_geo built them (offsets in the trunk's frame; the north side is negative):
1. LIVENESS: the old choice (the most overlap) takes the south interval at the first split column -- else the check
   below tests nothing (BROKEN TEST);
2. every split column holds A8 on its terminal's side, north of the box, which its berth (offset -5.879) is on;
3. a lane already held on one side stays there whatever its terminal (K41: lanes leaving the source's south face are
   not to be moved across the box), and a reference inside an interval takes that interval.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
import corridor  # noqa: E402

TERM = -5.879                                           # A8's berth offset (its piece's o1)
# (column, the free intervals, A8's reference) -- k349 the last column the box does not split
COLS = [(349, [(-42.62, 46.98)], -5.589),
        (350, [(-42.65, -5.82), (-5.5, 46.95)], -5.619),
        (351, [(-42.65, -5.85), (-4.9, 46.93)], -5.650),
        (352, [(-42.67, -5.85), (-4.3, 46.93)], -5.680),
        (353, [(-42.7, -5.87), (-3.7, 46.9)], -5.711),
        (354, [(-42.7, -5.9), (-3.1, 46.88)], -5.741),
        (355, [(-42.72, -5.9), (-2.5, 46.88)], -5.772),
        (356, [(-42.75, -5.92), (-1.9, 46.85)], -5.802),
        (357, [(-42.75, -5.87), (-1.75, 46.83)], -5.833),
        (358, [(-42.77, -5.87), (-1.15, 46.83)], -5.863)]

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def most_overlap(iv, ref, was):
    """the choice before: the reference's interval, else the most overlap with the one before, else the nearest"""
    inside = [q for q in iv if q[0] <= ref <= q[1]]
    if inside:
        return inside[0]
    if was is not None and any(min(q[1], was[1]) > max(q[0], was[0]) for q in iv):
        return max(iv, key=lambda q: min(q[1], was[1]) - max(q[0], was[0]))
    return min(iv, key=lambda q: min(abs(ref - q[0]), abs(ref - q[1])))


# 1. liveness
old = most_overlap(COLS[1][1], COLS[1][2], COLS[0][1][0])
if old != COLS[1][1][1]:
    print(f'BROKEN TEST: the old choice at k350 is {old}, not the south interval -- the columns no longer show it')
    sys.exit(2)
check(True, f'liveness: by the most overlap, k350 holds A8 south of the box {old}')

# 2. A8 along its columns
was, held = None, []
for k, iv, ref in COLS:
    was = corridor.held_interval(iv, ref, was, TERM)
    held.append((k, was))
split = [(k, q) for (k, q), (_k, iv, _r) in zip(held, COLS) if len(iv) > 1]
check(all(q[1] < -5.0 for _k, q in split),
      f'every split column holds A8 north of the box, its berth\'s side: {[(k, q[1]) for k, q in split]}')

# 3. held on one side, it stays; a reference inside an interval takes it
s = corridor.held_interval(COLS[2][1], COLS[2][2], COLS[1][1][1], TERM)
check(s == COLS[2][1][1], f'held south a column before, the lane stays south whatever its terminal: {s}')
r = corridor.held_interval(COLS[2][1], -10.0, COLS[1][1][1], TERM)
check(r == COLS[2][1][0], f'a reference inside an interval takes that interval: {r}')
n = corridor.held_interval(COLS[1][1], COLS[1][2], COLS[0][1][0], None)
check(n == old, f'with no fixed terminal at that end the choice is the old one: {n}')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
