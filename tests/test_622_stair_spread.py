#!/usr/bin/env python3
"""#622 `awx/pairs.py` stair_spread: how far a pair laid along a slanted line must stray from it.

The pair router turns 45 degrees at a time and runs straight at least its turning radius after each turn
(turn_straight_steps, after the turn's own step), so a line off the router's eight headings is laid as a staircase
of runs on the two headings either side. The snap (whole_snap) holds a pair within half the smallest such staircase of its planned line, and prices
straying past it. What this asserts:

1. On a router heading there is no staircase: the spread is 0.
2. The spread has the values worked out by hand: at a slope of 1:2 both runs are the minimum run R, and the corners
   stand R / 2 * cos(a) apart (0.1006 at R = 0.225, nine steps of 0.025: the turn's step and eight straight); at
   19.4 degrees off an axis, 0.1373 (the K51 pinch beside C12).
3. It is the same under the board's mirrors and under x and y swapped -- the router's rules are -- so a line 10 degrees
   off vertical spreads as one 10 degrees off horizontal. (The first version folded the heading by 45 degrees and
   gave a line 80 degrees off horizontal the spread of one at 35.)
4. It depends on the heading only, not on the length of (dx, dy).
5. It never exceeds the minimum run: a staircase's corners stand no further apart than one of its runs.
6. stair_dev, along a planned line: 0 on a router heading, half the spread along a long slant, at most half how far a
   short jog runs across its heading (a line straying less than a step is laid straight).
7. plan_audit._with_stair, the audit's bar from a pair to a SINGLE (laid against its staircase, not its line): each
   leg's pieces carry stair_dev at their ends over the allowance they had, the legs read apart.
8. plan_audit.check_pitch reads it for a single with a slanted pair either side (which the snap cannot move off
   either): just past track and clearance from each pair's near leg it is short of both bars; beside ONE pair it is not
   (the snap moves it toward its other side), the same lines as singles are not, nor is a pair where the single was
   (laid against the other pair's steps).
"""
import math
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'awx'))
import pairs  # noqa: E402

FAIL = []
CFG = SimpleNamespace(min_turning_radius=0.2, grid_step=0.025)      # K51's rules: a 9-step run, 0.225 mm
R = (pairs.turn_straight_steps(CFG) + 1) * CFG.grid_step         # a run: the turn's step and the straight after it


def check(cond, what):
    if not cond:
        FAIL.append(what)


def at(deg, length=1.0):
    r = math.radians(deg)
    return pairs.stair_spread(CFG, length * math.cos(r), length * math.sin(r))


check(pairs.turn_straight_steps(CFG) == 8 and abs(R - 0.225) < 1e-12, f'the fixture: 1 + 8 steps of 0.025 (got R {R})')

# 1. the router's own headings
for deg in range(0, 360, 45):
    check(at(deg) == 0.0, f'{deg} degrees is a router heading: spread {at(deg)}')

# 2. worked values
a = math.atan(0.5)
check(abs(at(math.degrees(a)) - R / 2 * math.cos(a)) < 1e-12, f'slope 1:2: {at(math.degrees(a))} != {R / 2 * math.cos(a)}')
check(abs(at(19.4) - 0.1373) < 5e-4, f'19.4 degrees: {at(19.4):.4f} != 0.1373')

# 3. the mirrors and the swap of x and y
for deg in (2, 10, 19.4, 26.565, 35, 40, 44):
    r = math.radians(deg)
    dx, dy = math.cos(r), math.sin(r)
    base = pairs.stair_spread(CFG, dx, dy)
    for mx, my, swap in [(-1, 1, 0), (1, -1, 0), (-1, -1, 0), (1, 1, 1), (-1, 1, 1), (1, -1, 1), (-1, -1, 1)]:
        x, y = mx * dx, my * dy
        if swap:
            x, y = y, x
        got = pairs.stair_spread(CFG, x, y)
        check(abs(got - base) < 1e-12, f'{deg} degrees mirrored ({mx}, {my}, swap {swap}): {got} != {base}')
    check(abs(at(90 - deg) - base) < 1e-12, f'{90 - deg} degrees off horizontal is {deg} off vertical: {at(90 - deg)} != {base}')

# 4. the heading only
for deg in (7, 19.4, 33):
    check(abs(at(deg, 0.013) - at(deg, 17.0)) < 1e-12, f'{deg} degrees: the spread depends on the length of (dx, dy)')

# 5. bounded by one run
worst = max(at(d / 10) for d in range(0, 3600))
check(0 < worst <= R + 1e-12, f'the largest spread {worst} is not within (0, R]')

# 6. stair_dev along a planned line: 0 on a router heading; half the spread along a long slant (h616's DQS pairs ran
#    12 degrees off vertical); no more than half how far a short jog runs across its heading
line = lambda deg, n=60, L=0.1: [(i * L * math.sin(math.radians(deg)), i * L * math.cos(math.radians(deg)))
                                 for i in range(n)]
for deg in (0, 45, 90):
    d_ = pairs.stair_dev(CFG, line(deg))
    check(max(d_) < 1e-9, f'stair_dev on a router heading ({deg} degrees): {max(d_)}')
d_ = pairs.stair_dev(CFG, line(12.0))
check(all(abs(v - at(78.0) / 2) < 1e-9 for v in d_), f'stair_dev along a 12-degree slant: {min(d_)}..{max(d_)}, '
      f'want {at(78.0) / 2} everywhere')
jog = [(0.0, 0.0), (5.0, 0.0), (5.5, 0.1), (10.0, 0.1)]
d_ = pairs.stair_dev(CFG, jog)
check(0 < max(d_) <= 0.05 + 1e-9, f'stair_dev at a 0.1 mm jog: {max(d_)}, want at most half the jog (0.05)')
check(pairs.stair_dev(CFG, [(1.0, 2.0)]) == [0.0], 'stair_dev of a single point')

# 7. the audit's bar from a pair to a single (plan_audit._with_stair): each leg's pieces carry stair_dev at their ends
#    over the allowance they had, the two legs read apart (the N leg's chord is not the P leg's), an axis leg unchanged
import numpy as np  # noqa: E402
import plan_audit as pa_  # noqa: E402
P = np.array
legs = [(P(a_), P(b_), 'F.Cu', 0.0125, 0.0125) for a_, b_ in zip(line(12.0, 30), line(12.0, 30)[1:])]
legs += [(P((5.0 + x_, y_)), P((5.0 + x2_, y2_)), 'F.Cu', 0.0, 0.0)
         for (x_, y_), (x2_, y2_) in zip(line(0.0, 30), line(0.0, 30)[1:])]
got = pa_._with_stair(CFG, legs)
check(len(got) == len(legs), f'_with_stair keeps every piece: {len(got)} of {len(legs)}')
check(all(abs(g_[3] - 0.0125 - at(78.0) / 2) < 1e-9 and abs(g_[4] - 0.0125 - at(78.0) / 2) < 1e-9 for g_ in got[:29]),
      f'_with_stair on the slanted leg: {[round(g_[3], 4) for g_ in got[:3]]}, want {0.0125 + at(78.0) / 2:.4f}')
check(all(g_[3] == 0.0 and g_[4] == 0.0 for g_ in got[29:]), 'the axis leg read with the slanted one\'s chord')

# 8. the audit's pitch check (plan_audit.check_pitch) reads it: a single 0.04 past track and clearance from the near
#    legs of two pairs either side of it along a 12-degree slant is short of both bars (each staircase strays 0.087
#    toward it); with one pair beside it, or as singles, or a PAIR where it was, none
import contextlib  # noqa: E402
import io  # noqa: E402
TW_, CL_ = 0.15, 0.1
nrm = (math.cos(math.radians(12.0)), -math.sin(math.radians(12.0)))
off = lambda pts, o: [(x_ + nrm[0] * o, y_ + nrm[1] * o) for x_, y_ in pts]
base = line(12.0, 40)
pcs = lambda pts: [(a_, b_, 'F.Cu') for a_, b_ in zip(pts, pts[1:])]
HP = 0.125                                   # half the legs' pitch, the test's own
S_ = HP + TW_ + CL_ + 0.04                  # the single's line: 0.04 past track and clearance from PA's near leg
lanes_ = {'PA': pcs(off(base, -HP)) + pcs(off(base, HP)), 'S1': pcs(off(base, S_)),
          'PB': pcs(off(base, 2 * S_ - HP)) + pcs(off(base, 2 * S_ + HP)),        # as far beyond it, its mirror
          'PC': pcs(off(base, S_)) + pcs(off(base, S_ + 2 * HP))}                  # a pair, its near leg where the single is
far_ = (100.0, 100.0)
spine_ = SimpleNamespace(project=lambda xs, ys: (np.zeros(len(xs)), np.zeros(len(xs))))


def pitch_hits(members, pairs_):
    ctx_ = SimpleNamespace(cfg=SimpleNamespace(track_width=TW_, clearance=CL_, grid_step=0.025, layers=['F.Cu', 'B.Cu'],
                                               min_turning_radius=0.2),
                           pcb=SimpleNamespace(board_info=SimpleNamespace(board_bounds=(-20.0, -20.0, 20.0, 20.0))),
                           pairs=pairs_)
    c_ = SimpleNamespace(idx=0, members=members, virtual_of=lambda names: [p_ for n_ in names for p_ in lanes_[n_]],
                         teeth={n_: far_ for n_ in members}, stubs={n_: far_ for n_ in members}, spine=spine_)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        pa_.check_pitch(ctx_, [c_])
    return [ln for ln in buf.getvalue().splitlines() if ln.startswith('PITCH ') and ' min ' in ln]


h_ = pitch_hits(['PA', 'S1', 'PB'], {'PA': 1, 'PB': 1})
check(len(h_) == 2 and all('S1' in ln for ln in h_), f'a single between two slanted pairs, short of both staircases: {h_}')
check(pitch_hits(['PA', 'S1'], {'PA': 1}) == [], 'a single beside ONE slanted pair (the snap moves it off): none short')
check(pitch_hits(['PA', 'S1', 'PB'], {}) == [], 'the same lines as singles: none short')
check(pitch_hits(['PA', 'PC'], {'PA': 1, 'PC': 1}) == [], 'a pair where the single is: none short')

if FAIL:
    print('\n'.join('FAIL ' + f for f in FAIL))
    sys.exit(1)
print('PASS test_622_stair_spread: 0 on the router headings, the worked values, the same under every mirror and '
      f'the swap of x and y, the heading only, at most one run (worst {worst:.4f} of {R}); stair_dev along a line, '
      'capped at a short jog; the audit\'s pair-to-single allowance per leg')
