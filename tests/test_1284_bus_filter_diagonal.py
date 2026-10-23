#!/usr/bin/env python3
"""#1284: the strict geometric bus filter keeps parallel buses in EVERY
direction, the -45 deg diagonal included.

``filter_bus_groups_geometric`` took each net's run as the pads extreme along
x + y and compared the runs as DIRECTED vectors. Along the -45 deg diagonal
x + y is constant, so a net there got a zero-length run (rejected by the 5 mm
rule) or a direction whose sign followed sub-mm jitter (two parallel nets
read as anti-parallel, outside the cone). The run is now the farthest pair
among the pads extreme along x, y and both diagonals, compared either way
round.

Checks:
  1. the issue's five cases: horizontal, +45, -45, -45 with 0.1 mm jitter,
     and four -45 nets with jitter -- all kept whole
  2. the same jittered three-net bus rotated through 360 deg in 5 deg steps
     is kept whole at every angle
  3. the filter still filters: a member 45 deg off the bus is dropped, a
     group of short hops is rejected, a two-net group is rejected
  4. a multi-pad net's run spans its farthest pads, so a T-shaped net whose
     x + y extremes run 57 deg off a horizontal bus still joins it

    python3 tests/test_1284_bus_filter_diagonal.py
"""
import math
import os
import sys
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

os.environ.pop('KICAD_BUS_STRICT', None)   # the filter under test, not the bypass

from bus_detection import BusGroup, filter_bus_groups_geometric  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def run(nets):
    """The kept net ids of a one-group filter over ``nets`` (lists of pad
    positions), or None when the group is rejected."""
    pads = {i: [NS(global_x=x, global_y=y) for x, y in pts]
            for i, pts in enumerate(nets)}
    bus = BusGroup(name='bus', net_ids=list(pads))
    out = filter_bus_groups_geometric(NS(pads_by_net=pads), [bus],
                                      NS(power_net_widths={}))
    return out[0].net_ids if out else None


def rotate(nets, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[(x * c - y * s, x * s + y * c) for x, y in pts] for pts in nets]


def t_issue_cases():
    cases = [
        ('horizontal', [[(0, 0), (20, 0)], [(0, 1), (20, 1)], [(0, 2), (20, 2)]]),
        ('+45 diagonal', [[(0, 0), (10, 10)], [(1, 0), (11, 10)], [(2, 0), (12, 10)]]),
        ('-45 diagonal', [[(0, 10), (10, 0)], [(1, 11), (11, 1)], [(2, 12), (12, 2)]]),
        ('-45 diagonal, 0.1mm jitter',
         [[(0, 10), (10, 0.1)], [(1, 11), (11, 0.9)], [(2, 12), (12, 2.1)]]),
        ('-45 diagonal, 4 nets, 0.1mm jitter',
         [[(0, 10), (10, 0.1)], [(1, 11), (11, 0.9)], [(2, 12), (12, 2.1)],
          [(3, 13), (13, 2.9)]]),
    ]
    for label, nets in cases:
        got = run(nets)
        check(f'issue case kept whole: {label}', got == list(range(len(nets))),
              f'got {got}')


def t_rotation_sweep():
    bus = [[(0, 0), (12, 0.1)], [(0, 1), (12, 0.9)], [(0, 2), (12, 2.1)]]
    lost = [deg for deg in range(0, 360, 5) if run(rotate(bus, deg)) != [0, 1, 2]]
    check('a parallel bus is kept at every 5 deg rotation', not lost,
          f'rejected or trimmed at {lost}')


def t_still_filters():
    # Three parallel nets plus one 45 deg off the bus: the stray is dropped.
    for deg in (0, 135, 315):
        nets = rotate([[(0, 0), (12, 0)], [(0, 1), (12, 1)], [(0, 2), (12, 2)],
                       [(0, 3), (9, 12)]], deg)
        got = run(nets)
        check(f'a member 45 deg off the bus is dropped (bus at {deg} deg)',
              got == [0, 1, 2], f'got {got}')
    short = [[(0, 0), (3, -3)], [(1, 1), (4, -2)], [(2, 2), (5, -1)]]
    check('a group of sub-5mm hops is still rejected', run(short) is None,
          f'got {run(short)}')
    pair = [[(0, 10), (10, 0)], [(1, 11), (11, 1)]]
    check('a two-net group is still rejected', run(pair) is None,
          f'got {run(pair)}')


def t_multipad_span():
    # Net 3 is a T beside a horizontal bus: its arm spans (0,3)-(14,3) and a
    # stub pad hangs at (7,-8). Its x + y extremes are the stub and one arm
    # end, a run 57 deg off the bus; its farthest pads are the 14 mm arm.
    nets = [[(0, 0), (14, 0)], [(0, 1), (14, 1)], [(0, 2), (14, 2)],
            [(0, 3), (14, 3), (7, -8)]]
    got = run(nets)
    check('a multi-pad net is measured over its farthest pads',
          got == [0, 1, 2, 3], f'got {got}')


def main():
    print('#1284: geometric bus filter in every direction')
    t_issue_cases()
    t_rotation_sweep()
    t_still_filters()
    t_multipad_span()
    if FAILS:
        print(f'\nFAILED: {len(FAILS)} check(s): {FAILS}')
        return 1
    print('\nAll checks passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
