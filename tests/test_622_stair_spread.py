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

if FAIL:
    print('\n'.join('FAIL ' + f for f in FAIL))
    sys.exit(1)
print('PASS test_622_stair_spread: 0 on the router headings, the worked values, the same under every mirror and '
      f'the swap of x and y, the heading only, at most one run (worst {worst:.4f} of {R})')
