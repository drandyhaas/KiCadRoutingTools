#!/usr/bin/env python3
"""A finding at a run's ends is judged by ONE test (awx/ends_near.py), the loop's feedback's and its ends' repair's.

  python3 tests/test_622_ends_near.py

whole_feedback names the lanes whose findings stand at the teeth or the berths two rounds running -- the fanout's to
change -- and whole_route.repair_ends repairs the rows of that finding it finds there. The feedback counted a point
beside a side face within the width the face's lanes stack to; the repair looked a face's front round the ends' box
and no farther. A memory lane's tooth 1.3 mm from a flash part's pad beside the array was named at the teeth, the
repair said "no STATIC row of it there" and moved nothing, the loop ran again on the same ends and stopped on the
same finding, and the lane shipped open.

1. in front of a face: within the front, at the ends; past it, not;
2. beside a side face: past the front yet within the stack of that face's lanes (and two pitches), at the ends; past
   the stack, not; a face with fewer ends reaches less far;
3. a run to several parts: the berths' boxes are one a part -- a point in the gap between two parts is at neither;
4. both callers ask ends_near: the feedback's `near` and the repair's filter of its rows.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))

import ends_near as en  # noqa: E402

BAD = []
N = [0]


def check(ok, what):
    N[0] += 1
    print(('PASS: ' if ok else 'FAIL: ') + what)
    if not ok:
        BAD.append(what)


FRONT, PITCH = 0.9, 0.25
# (teeth round a 10 x 10 box: eight on its south face y = 10, two on its north face y = 0; the berths at two parts)
ends = {f's{i}': ((1.0 + i, 10.0), (30.0 + 0.5 * i, 2.0)) for i in range(8)}
ends.update({f'n{i}': ((4.0 + i, 0.0), (50.0 + 0.5 * i, 2.0)) for i in range(2)})
ends.update({'w': ((0.0, 5.0), (30.0, 4.0)), 'e': ((10.0, 5.0), (51.0, 4.0))})
dest_of = {n: ('P1' if e[1][0] < 40 else 'P2') for n, e in ends.items()}
bxs = en.boxes(ends, dest_of)
near = lambda k, x, y: en.near(ends, bxs, k, x, y, FRONT, PITCH)      # noqa: E731

check(bxs[0] == [(0.0, 0.0, 10.0, 10.0)] and len(bxs[1]) == 2, f'the teeth one box, the berths one a part ({bxs})')
# ---- 1. in front of a face (beside the box, not above or below it)
check(near(0, 10.0 + FRONT - 0.01, 5.0) and not near(0, 10.0 + FRONT + 0.01, 5.0),
      'in front of a face: within the front at the ends, past it not')
check(near(0, 10.5, 10.5) and not near(0, 10.8, 10.8), 'off a corner: by the distance to the box')
# ---- 2. beside a side face: the stack of its lanes
south, north = (8 + 2) * PITCH, (2 + 2) * PITCH
check(south > FRONT and near(0, 5.0, 10.0 + south - 0.01) and not near(0, 5.0, 10.0 + south + 0.01),
      f'beside the side face of eight ends: at the ends to {south} mm, past the front ({FRONT}); past the stack not')
check(near(0, 5.0, 0.0 - north + 0.01) and not near(0, 5.0, 0.0 - north - 0.01),
      f'beside the side face of two ends: to {north} mm only')
check(near(0, 5.0, 10.0 + 1.3), 'a pad 1.3 mm beside the array\'s side face is at the teeth')
# ---- 3. several parts
gapx = (bxs[1][0][2] + bxs[1][1][0]) / 2
check(not near(1, gapx, 3.0) and near(1, bxs[1][0][2] + 0.1, 3.0) and near(1, bxs[1][1][0] - 0.1, 3.0),
      'a point in the gap between two parts\' berths is at neither; one beside either part is')
# ---- 4. the two callers
fb = open(os.path.join(ROOT, 'awx', 'whole_feedback.py'), encoding='utf-8').read()
wr = open(os.path.join(ROOT, 'awx', 'whole_route.py'), encoding='utf-8').read()
rp = wr[wr.index('def repair_ends('):]
rp = rp[:rp.index('\ndef ', 10)]
check('ends_near.near(ends, BOXES, k, x, y, FRONT, bd.LANE_MIN)' in fb and 'def side_reach' not in fb,
      'whole_feedback asks ends_near, and keeps no test of its own')
check('_en.near(ends_, bxs_,' in rp and 'end_box' not in rp,
      'repair_ends filters its rows by ends_near, and keeps no box of its own')

print(f'\n{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
