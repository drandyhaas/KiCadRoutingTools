#!/usr/bin/env python3
"""_wiring_target prices each net by its spanning tree, not by its
connections one by one.

A part joining a net lengthens the net's pad tree by its distance to the
nearest placed pad (Prim's step). So each NET is one vote, however many parts
it reaches:
  1. a net on six parts no longer outvotes a two-pin net 5:1 (voting once per
     (partner, net) pulled the part into the middle of the big net, where no
     partner is);
  2. duplicated pins (a USB-C receptacle's A6/B6) are two points of one tree,
     not two votes -- the run-7 bug, where they pulled the 27R pair 2:1 toward
     the connector, stays fixed;
  3. three nets meeting at one point put the part AT that point (the
     geometric median), where averaging stopped short of it;
  4. the vote goes to a PAD, never to the middle of another net's airwire,
     which the polish's pad tree would not credit;
  5. nets over max_fanout and a missing ref still give None.
"""
import math
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
from placement.seeder import _wiring_target  # noqa: E402


def _part(nets, pads):
    """pads: list of (x, y, net_id)."""
    return SimpleNamespace(nets=set(nets), pad_globals=lambda: list(pads))


def test_a_big_net_is_one_vote():
    # R1: net 1 to P (two-pin, at x=0), net 2 to five parts in a row at
    # x=20, y=0..20. Voting per (partner, net) put R1 5/6 of the way to the
    # row (x=16.7); the tree vote puts it midway between P and the row.
    parts = {'R1': _part([1, 2], [(0.0, 0.0, 1), (1.0, 0.0, 2)]),
             'P': _part([1], [(0.0, 10.0, 1)])}
    for i in range(5):
        parts[f'Q{i}'] = _part([2], [(20.0, 5.0 * i, 2)])
    state = SimpleNamespace(
        parts=parts,
        net_refs={1: ('R1', 'P'), 2: ('R1',) + tuple(f'Q{i}' for i in range(5))})
    x, y = _wiring_target(state, 'R1', placed={'P'} | {f'Q{i}' for i in range(5)})
    assert 0.0 <= x <= 20.0 and abs(x - 10.0) < 0.5, (
        f"expected x near 10 (one vote per net), got {x:.3f} -- the big net "
        f"outvotes the two-pin net again")
    assert abs(y - 10.0) < 0.5, y
    print(f"  PASS: a six-part net and a two-pin net pull equally ({x:.2f}, {y:.2f})")


def test_duplicated_pads_cast_no_extra_vote():
    # R7: net 1 to J1 (TWO pads, A6+B6 at x=0) and net 2 to U1 (x=10). The
    # target is midway between J1's nearer pad and U1, wherever J1's second
    # pad is.
    def target(j1_pads):
        state = SimpleNamespace(
            parts={'R7': _part([1, 2], [(5.0, 5.0, 1), (6.0, 5.0, 2)]),
                   'J1': _part([1], j1_pads),
                   'U1': _part([2], [(10.0, 5.0, 2)])},
            net_refs={1: ('R7', 'J1'), 2: ('R7', 'U1')})
        return _wiring_target(state, 'R7', placed={'J1', 'U1'})
    one = target([(0.0, 5.0, 1)])
    two = target([(0.0, 5.0, 1), (0.0, 5.6, 1)])
    assert math.dist(one, two) < 0.35, (one, two)
    assert abs(one[0] - 5.0) < 1e-6, one
    print(f"  PASS: J1's doubled pad moves the target {math.dist(one, two):.2f} mm")


def test_three_nets_meeting_put_the_part_there():
    # Two nets reach (0, 0), one reaches (12, 0): the least wire puts the
    # part at (0, 0) (2 x 0 + 12), where the mean (4, 0) costs 4+4+8 = 16.
    state = SimpleNamespace(
        parts={'C1': _part([1, 2, 3], [(0, 0, 1), (0, 0, 2), (0, 0, 3)]),
               'A': _part([1, 2], [(0.0, 0.0, 1), (0.0, 0.0, 2)]),
               'B': _part([3], [(12.0, 0.0, 3)])},
        net_refs={1: ('C1', 'A'), 2: ('C1', 'A'), 3: ('C1', 'B')})
    x, y = _wiring_target(state, 'C1', placed={'A', 'B'})
    assert math.hypot(x, y) < 0.05, (x, y)
    print("  PASS: two nets at one point outweigh the third")


def test_the_vote_is_a_pad_not_an_airwire():
    # net 1 runs between A (0, 0) and B (20, 0); net 2 goes to P (10, 6).
    # The middle of A-B's airwire is 6 mm from P, but joining there is not a
    # pad-tree join: the target lies at a pad of net 1, not at (10, 0..6).
    state = SimpleNamespace(
        parts={'R1': _part([1, 2], [(0, 0, 1), (1, 0, 2)]),
               'A': _part([1], [(0.0, 0.0, 1)]),
               'B': _part([1], [(20.0, 0.0, 1)]),
               'P': _part([2], [(10.0, 6.0, 2)])},
        net_refs={1: ('R1', 'A', 'B'), 2: ('R1', 'P')})
    x, y = _wiring_target(state, 'R1', placed={'A', 'B', 'P'})
    near_a = math.dist((x, y), (0.0, 0.0)) + math.dist((x, y), (10.0, 6.0))
    assert abs(near_a - math.dist((0.0, 0.0), (10.0, 6.0))) < 0.05 or \
        abs(math.dist((x, y), (20.0, 0.0)) + math.dist((x, y), (10.0, 6.0))
            - math.dist((20.0, 0.0), (10.0, 6.0))) < 0.05, (x, y)
    print(f"  PASS: the target ({x:.2f}, {y:.2f}) lies between a pad of net 1 and P")


def test_fanout_and_empty_cases_unchanged():
    state = SimpleNamespace(
        parts={'R1': _part([1], [(0.0, 0.0, 1)]),
               'U1': _part([1], [(5.0, 0.0, 1)])},
        net_refs={1: tuple(f'P{i}' for i in range(30))})  # >max_fanout
    assert _wiring_target(state, 'R1', placed={'U1'}) is None
    assert _wiring_target(state, 'MISSING', placed=set()) is None
    print("  PASS: fanout exclusion and missing-ref cases unchanged")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
