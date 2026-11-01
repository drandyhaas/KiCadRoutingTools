#!/usr/bin/env python3
"""A rect pad in the plan's obstacle model is its rectangle grown by the margin, exactly, however large it is.

  python3 tests/test_622_large_rect_pad.py

braid._build_obstacles draws a rect pad as capsules: its long axis's (every point of the pad) and the margin's along
each edge, while half its short side is within RECT_EXACT margins. A larger one -- a QFN's exposed pad -- was its
circumscribed disc, which stood (sqrt 2 - 1) half a side past the pad's faces at their middles: a 3.71 mm exposed
pad at a 0.135 mm margin refused the middle pads of two rows their only escape, straight out of the row
(escape_moves: a row part), and the bus left four lanes with no end. It is drawn as lines along its long side, two
margins apart at most, with the edges' capsules.

For a large square pad, a long one past RECT_EXACT and a small one within it: every point of a box round the pad is
in the model exactly where it is within the margin of the rectangle (points within 1 um of that boundary left out),
the pad's deep corners inside it too; and a stub straight out of a face's middle, the way a row pad leaves, is clear
from just past the grown rectangle and not from inside it.
"""
import contextlib
import io
import os
import random
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
with contextlib.redirect_stdout(io.StringIO()):
    import braid  # noqa: E402

M = 0.135               # the margin: 0.09 clearance + half a 0.09 track
BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def model(sx, sy, cx=10.0, cy=10.0):
    pad = types.SimpleNamespace(global_x=cx, global_y=cy, size_x=sx, size_y=sy, shape='rect', layers=['F.Cu'],
                                drill=0.0, pad_type='smd', pad_number='1', net_id=7, rect_rotation=0.0)
    pcb = types.SimpleNamespace(footprints={'U1': types.SimpleNamespace(pads=[pad])}, segments=[], vias=[])
    return braid._build_obstacles(pcb, set(), 'F.Cu', margin=M)


def rect_dist(p, cx, cy, sx, sy):
    dx = max(abs(p[0] - cx) - sx / 2, 0.0)
    dy = max(abs(p[1] - cy) - sy / 2, 0.0)
    return (dx * dx + dy * dy) ** 0.5


def main():
    rnd = random.Random(622)
    for sx, sy, what in ((3.7084, 3.7084, 'a 3.71 mm square exposed pad'), (4.0, 2.0, 'a 4 x 2 mm pad'),
                         (0.5, 0.6, 'a 0.5 x 0.6 mm pad (within RECT_EXACT)')):
        cx, cy = 10.0, 10.0
        o = model(sx, sy, cx, cy)
        wrong, n = [], 0
        for _ in range(20000):
            p = (cx + rnd.uniform(-sx / 2 - 3 * M, sx / 2 + 3 * M), cy + rnd.uniform(-sy / 2 - 3 * M, sy / 2 + 3 * M))
            d = rect_dist(p, cx, cy, sx, sy)
            if abs(d - M) < 1e-3:
                continue
            n += 1
            got = o.point_violation(p) is not None
            if got != (d < M):
                wrong.append((round(p[0], 4), round(p[1], 4), round(d, 4), got))
        check(not wrong, f'{what}: {n} points in the model exactly where within the margin of it'
              + (f' -- {len(wrong)} wrong, e.g. {wrong[:3]}' if wrong else ''))
        # (a stub straight out of each face's middle, as a row pad's escape leaves)
        ok_out = ok_in = True
        for ux, uy, half in ((0, -1, sy / 2), (0, 1, sy / 2), (-1, 0, sx / 2), (1, 0, sx / 2)):
            for r0, want in ((half + M + 0.005, True), (half + M - 0.02, False)):
                a = (cx + ux * r0, cy + uy * r0)
                b = (cx + ux * (r0 + 0.5), cy + uy * (r0 + 0.5))
                if o.seg_clear(a, b) != want:
                    if want:
                        ok_out = False
                    else:
                        ok_in = False
        check(ok_out, f'{what}: a stub out of each face\'s middle, from just past the grown rectangle, is clear')
        check(ok_in, f'{what}: a stub from inside the grown rectangle is not')
    if BAD:
        print(f'FAIL: {len(BAD)} failure(s)')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
