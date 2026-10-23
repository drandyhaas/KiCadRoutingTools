#!/usr/bin/env python3
"""A WOUND ring goes round the destination however the bundle arrives (corridor.build_wrap_spine, wrap_side).
  python3 tests/test_622_wrap_spine_slant.py
zynq's Ethernet bus to U3 (a QFN north-east of U1) arrives at a slant, so the face it meets is the hull's diagonal
south-west face; the winding cut on U3's south face sent MDC and MDIO, berthing at that face's east end, round the
north. Those stubs lie more than three half turns round from the meeting face's corner, the ring's start was backed up
to just short of them, and the ring ran straight from the trunk THROUGH U3 -- the geometry laid MDC up the west face and
back down it, across every berth there (ENDS CROWDED both rounds). On a 6 x 6 mm box of pads, a ring's start off its
north-west corner and two stubs at the south face's east end:
1. arriving at a slant (from the south-west), the wound ring stays outside the box and passes its north and east faces;
2. arriving square (from the west), the same;
3. a ring that is not wound (wrap_side off) is built as it was: its stubs within three half turns, from the slant.
"""
import math
import os
import sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import corridor as cr  # noqa: E402
fails = []


def check(cond, msg):
    print(('PASS ' if cond else 'FAIL ') + msg)
    if not cond:
        fails.append(msg)


X0, Y0, X1, Y1 = 128.0, -90.6, 134.0, -84.6           # (y down: north is the smaller y)
pads = ([(X0, Y0 + 0.4 * i) for i in range(16)] + [(X1, Y0 + 0.4 * i) for i in range(16)]
        + [(X0 + 0.4 * i, Y0) for i in range(16)] + [(X0 + 0.4 * i, Y1) for i in range(16)])
stubs = [(133.0, Y1 + 0.7), (133.4, Y1 + 0.7)]           # the south face's east end, past the winding cut
start = (124.4, -89.5)                                   # the ring's start, on the trunk's north side


def inside(p, m=0.05):
    return X0 + m < p[0] < X1 - m and Y0 + m < p[1] < Y1 - m


def dense(P, step=0.1):
    out = []
    for (ax, ay), (bx, by) in zip(P, P[1:]):
        k = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / step)))
        out += [(ax + (bx - ax) * i / k, ay + (by - ay) * i / k) for i in range(k + 1)]
    return out


for case, arrive in (('1. slant', (0.82, -0.57)), ('2. square', (1.0, 0.0))):
    sp = cr.build_wrap_spine(pads, stubs, [start], True, arrive, 0.35, wrap_side=True)
    P = [tuple(map(float, p)) for p in sp.P]
    pts = dense(P)
    check(not any(inside(p) for p in pts), f'{case}: the wound ring stays outside the destination -- {len(P)} vertices')
    check(min(p[1] for p in P) < Y0 and max(p[0] for p in P) > X1,
          f'{case}: it passes the north face and the east face (north {min(p[1] for p in P):.2f}, '
          f'east {max(p[0] for p in P):.2f})')
# 3. not wound: the stubs within three half turns of the meeting corner, the old bound -- the ring the short way round
sp = cr.build_wrap_spine(pads, stubs, [start], True, (0.82, -0.57), 0.35, wrap_side=False)
P = [tuple(map(float, p)) for p in sp.P]
check(max(p[0] for p in P) <= X1 + 0.5,
      f'3. not wound: the ring as it was, never round the east face ({max(p[0] for p in P):.2f})')
if fails:
    print(f'FAILED {len(fails)}')
    sys.exit(1)
print('ALL PASS')
