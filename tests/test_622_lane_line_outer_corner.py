#!/usr/bin/env python3
"""corridor.Spine.lane_line at a spine corner, the lanes on its OUTER side: their legs meet at the mitre of each one's
offset there, every column drawn, and two lanes in order at every column are drawn in that order.

  python3 tests/test_622_lane_line_outer_corner.py

The zynq DDR bus (U1 -> U2, two routing layers) round the north ring's first corner (36.5 degrees), as whole_geo laid
it once the joint fanout's other-net stubs stood on U2's facing face: RAS 3.8 mm outside the corner and moving in
0.07 mm a column, ODT a pitch inside it moving in 0.02 to 0.04. Joined where each lane's OWN two lines cross -- the rule
for a lane inside a corner -- RAS's met its other leg 2.3 mm along it, its three columns there were left out, and the
line drawn to that point crossed ODT's twice on F, though ODT stood inside it at every column: two nets open after the
route. The columns below are those two lanes' as the geometry solved them (s 5.0 .. 8.5 mm along the ring).

1. LIVENESS: both lanes are on the corner's outer side, and RAS's own lines (its last two columns before the corner,
   its first two after) meet more than a millimetre along the next leg -- the case that broke. Without it the test
   tests nothing, and says so.
2. FIXED: the two drawn lines do not cross, and no column of either is left out.
3. CHANGE DETECTOR, the inner side kept: a lane mirrored INSIDE the corner, its offset growing into it, still has the
   columns past its own lines' crossing left out (the hook the rule exists for: K44 DQ3).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'awx'))
import corridor  # noqa: E402

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


SPINE = [(100.1804, -117.912), (106.4414, -116.9707), (108.5782, -114.834), (108.5782, -110.4795)]
S0 = 5.0
ODT = [-3.3929, -3.3693, -3.3457, -3.3221, -3.2985, -3.2749, -3.2513, -3.2277, -3.2041, -3.1805, -3.1568, -3.1332,
       -3.1096, -3.086, -3.0624, -3.0209, -2.9793, -2.9072, -2.8352, -2.7631, -2.691, -2.6189, -2.5469, -2.4748,
       -2.4027, -2.3306, -2.2586, -2.1865, -2.1144, -2.0423, -1.9703, -1.8982, -1.8261, -1.754, -1.682, -1.6099]
RAS = [-4.6355, -4.6119, -4.565, -4.4929, -4.4208, -4.3488, -4.2767, -4.2046, -4.1325, -4.0605, -3.9884, -3.9163,
       -3.8442, -3.7722, -3.7001, -3.628, -3.5559, -3.4839, -3.4118, -3.3397, -3.2676, -3.1955, -3.1235, -3.0514,
       -2.9793, -2.9072, -2.8352, -2.7631, -2.691, -2.6189, -2.5469, -2.4748, -2.4027, -2.3306, -2.2586, -2.1865]

sp = corridor.Spine(SPINE)
j, s_c, turn = sp.corners()[0]
so = {n: [(S0 + 0.1 * i, o) for i, o in enumerate(v)] for n, v in (('ODT', ODT), ('RAS', RAS))}


def crossings(a, b):
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    n = 0
    for a0, a1 in zip(a, a[1:]):
        for b0, b1 in zip(b, b[1:]):
            d1, d2, d3, d4 = o(b0, b1, a0), o(b0, b1, a1), o(a0, a1, b0), o(a0, a1, b1)
            n += (d1 > 1e-12) != (d2 > 1e-12) and (d3 > 1e-12) != (d4 > 1e-12) and abs(d1) > 1e-12 and abs(d2) > 1e-12
    return n


def own_meeting(pts):
    """how far along the leg after the corner a lane's own two lines meet, past its first column there"""
    b = next(k for k, (s, _o) in enumerate(pts) if s >= s_c)
    P0, P1 = sp.xy(*pts[b - 2]), sp.xy(*pts[b - 1])
    Q0, Q1 = sp.xy(*pts[b]), sp.xy(*pts[b + 1])
    r, q = (P1[0] - P0[0], P1[1] - P0[1]), (Q1[0] - Q0[0], Q1[1] - Q0[1])
    w = (Q0[0] - P0[0], Q0[1] - P0[1])
    den = r[0] * q[1] - r[1] * q[0]
    u = (w[0] * r[1] - w[1] * r[0]) / den
    return u * math.hypot(*q)


# 1. liveness
outer = all(o * turn < 0 for n in so for _s, o in so[n] if abs(_s - s_c) < 0.2)
check(outer, f'both lanes on the outer side of the {turn:.1f}-degree corner at s {s_c:.3f}')
far = own_meeting(so['RAS'])
check(far > 1.0, f'RAS\'s own two lines meet {far:.2f} mm along the next leg (> 1 mm: the case that broke)')
check(all(a < b for (_s, a), (_t, b) in zip(so['RAS'], so['ODT'])), 'RAS outside ODT at every column')
if BAD:
    raise SystemExit('BROKEN TEST: its input is not the outer-corner case it guards -- ' + '; '.join(BAD))

# 2. fixed
lines = {n: sp.lane_line(v, fixed={0, len(v) - 1}) for n, v in so.items()}
xy = {n: [(x, y) for x, y, _s in L] for n, L in lines.items()}
check(crossings(xy['ODT'], xy['RAS']) == 0, f'the drawn lines cross {crossings(xy["ODT"], xy["RAS"])} time(s): 0')
for n, v in so.items():
    drawn = {round(s, 6) for _x, _y, s in lines[n]}
    left = [round(s, 2) for s, _o in v if round(s, 6) not in drawn]
    check(not left, f'{n}: every column drawn (left out: {left})')

# 3. the inner side as it was
inner = [(s, -o) for s, o in so['RAS']]                      # the same lane mirrored inside the corner
drawn = {round(s, 6) for _x, _y, s in sp.lane_line(inner, fixed={0, len(inner) - 1})}
left = [round(s, 2) for s, _o in inner if round(s, 6) not in drawn]
check(bool(left), f'a lane inside the corner still has the columns past its own lines\' meeting left out ({left})')

print('PASS' if not BAD else f'FAIL ({len(BAD)})')
sys.exit(1 if BAD else 0)
