#!/usr/bin/env python3
"""The destination's cut anywhere round it -- WINDING (whole_frame.perim_c / ring_side / cut_gaps / cut_sig, whole_ends.
wound_ride).

  python3 tests/test_622_wind_cut.py

A cut on the destination's far face is the whole frame's own, and a cut anywhere else sends the berths past it round
the other side. On a 10 x 10 box, berths on every face, this pins:

1. a far-face cut (its y, or ('E', y)) unrolls every berth exactly as perim_d always has, and puts each on the ring it
   always has -- the far face's split at the cut, the north and south faces on their own, the facing face on none;
2. a cut on the south face sends the far face's berths and the south face's east of it round the NORTH, and the
   south face's west of it round the south; a cut on the north face the mirror of that;
3. the cuts to try are one a gap between neighbouring berths, a gap round a corner too, never on the facing face;
4. two cuts in one gap unroll the berths in one order (cut_sig), cuts in two gaps in two;
5. a wound lane's ride round the long side is the box-hugging path round that side's corners, longer than the short
   way select_moves.around_box takes.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import whole_frame as wf  # noqa: E402
import whole_ends as we  # noqa: E402
import select_moves as sm  # noqa: E402

DB = (0.0, 0.0, 10.0, 10.0)
# (y down: north is y = 0) the far face's two berths, two on the north face, one on the facing face, two on the south
B = {'E_n': (10.0, 2.0), 'E_s': (10.0, 8.0), 'N_w': (3.0, 0.0), 'N_e': (7.0, 0.0), 'W': (0.0, 5.0),
     'S_w': (3.0, 10.0), 'S_e': (7.0, 10.0)}
fails = []


def check(ok, what):
    print(('PASS' if ok else 'FAIL') + ': ' + what)
    if not ok:
        fails.append(what)


# 1. the far face's cut, as it always was
for c in (5.0, ('E', 5.0)):
    check(all(wf.perim_c(p, DB, c) == wf.perim_d(p, DB, 5.0) for p in B.values()),
          f'a far-face cut {c!r} unrolls every berth as perim_d does')
    want = {'E_n': 'N', 'E_s': 'S', 'N_w': 'N', 'N_e': 'N', 'W': None, 'S_w': 'S', 'S_e': 'S'}
    got = {k: wf.ring_side(p, DB, c) for k, p in B.items()}
    check(got == want, f'a far-face cut {c!r} puts each berth on its ring as always: {got}')
    check(wf.far_cut(c), f'{c!r} is a far-face cut')

# 2. a cut on the south face, and on the north face
got = {k: wf.ring_side(p, DB, ('S', 5.0)) for k, p in B.items()}
check(got == {'E_n': 'N', 'E_s': 'N', 'N_w': 'N', 'N_e': 'N', 'W': None, 'S_w': 'S', 'S_e': 'N'},
      f'a cut on the south face at x 5: the far face and the south face east of it round the north: {got}')
got = {k: wf.ring_side(p, DB, ('N', 5.0)) for k, p in B.items()}
check(got == {'E_n': 'S', 'E_s': 'S', 'N_w': 'N', 'N_e': 'S', 'W': None, 'S_w': 'S', 'S_e': 'S'},
      f'a cut on the north face at x 5: the far face and the north face east of it round the south: {got}')
check(not wf.far_cut(('S', 5.0)) and not wf.far_cut(('N', 5.0)), 'a north or south cut is no far-face cut')
# (unrolled north first from the cut: the berths past a south cut come first, round the north)
u = {k: wf.perim_c(p, DB, ('S', 5.0)) for k, p in B.items()}
check(u['S_e'] < u['E_s'] < u['E_n'] < u['N_e'] < u['N_w'] < u['W'] < u['S_w'],
      f'a south cut unrolls the berths from it round the north: {sorted(u, key=u.get)}')

# 3. the cuts to try
cuts = wf.cut_gaps(list(B.values()), DB)
check(not any(c[0] == 'W' for c in cuts), f'no cut on the facing face: {cuts}')
check(len(cuts) == 5, f'one a gap, the facing face\'s two left out: {len(cuts)} of 7')
check(('N', 9.5) in cuts, f'a gap round the north-east corner gives a cut (N 9.5): {cuts}')
check(('S', 5.0) in cuts and ('E', 5.0) in cuts and ('N', 5.0) in cuts, 'the gaps on the faces give their middles')

# ...and, given the exits a berth could be laid at (`avoid`), a cut stands in the widest stretch of its gap clear of
# them: an exit at the south gap's middle (x 5) moves that cut off it, into the wider stretch beside it
cuts_av = wf.cut_gaps(list(B.values()), DB, avoid=[(5.0, 10.0), (4.0, 10.0)])
s_cut = [c for c in cuts_av if c[0] == 'S' and 3.0 < c[1] < 7.0]
check(len(s_cut) == 1 and abs(s_cut[0][1] - 6.0) < 1e-9,
      f'the south gap\'s cut clear of the exits at x 4 and 5: x 6, the middle of 5..7 ({s_cut})')
check(len(cuts_av) == len(cuts), 'still one cut a gap')

# 4. one order a gap
pts = list(B.values())
check(wf.cut_sig(pts, DB, ('S', 4.0)) == wf.cut_sig(pts, DB, ('S', 6.0)), 'two cuts in one gap: one order')
check(wf.cut_sig(pts, DB, ('S', 5.0)) != wf.cut_sig(pts, DB, 5.0), 'a south cut and the far cut: two orders')

# 5. the wound ride
a = (-5.0, 5.0)
for b, side in ((B['S_e'], 'N'), (B['N_e'], 'S'), (B['E_s'], 'N')):
    long_ = we.wound_ride(a, b, DB, side)
    short = sm.around_box(a, b, DB)
    check(long_ > short + 1.0, f'to {b} round the {side}: {long_:.2f} mm, the short way {short:.2f}')
x0, y0, x1, y1 = -0.3, -0.3, 10.3, 10.3
want = (math.dist(a, (x0, y0)) + math.dist((x0, y0), (x1, y0)) + math.dist((x1, y0), (x1, y1))
        + math.dist((x1, y1), B['S_e']))
check(abs(we.wound_ride(a, B['S_e'], DB, 'N') - want) < 1e-9, 'round the north to the south face: three corners')

print(f'\n{"FAIL" if fails else "PASS"}: winding cut, {len(fails)} failure(s)')
sys.exit(1 if fails else 0)
