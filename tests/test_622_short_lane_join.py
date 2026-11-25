#!/usr/bin/env python3
"""Two readings of a lane's ends in the group's model (awx/group_ends.py: join_runs, far_tooth).

  python3 tests/test_622_short_lane_join.py

A SHORT LANE'S JOIN. The supply ties laid on the fanned-out field keep clear of each lane end's way out -- its
straight run from the exit. A lane two millimetres long is its two ways out and the run between them, too short to
change layer: a tie laid clear of both ways ran BETWEEN them, two decaps' pads joined by an arch with the lane's
tooth inside it and its pin outside, and the lane stayed open in every round of its bus's loop.

A TOOTH THAT TAKES ITS LANE ROUND THE ARRAY. A tooth on the array's far face is a last resort, priced past any count
of vias. A header is a long row beside its array, its end pins past the array's corner: a tooth on the SIDE face
turned from such a pin takes the lane round three faces of the array, and was priced as a side face's.

1. join_runs: a net with two ways out on one layer, their exits within twice the reach: the run between the exits,
   of its own net, as wide as a way out -- none for exits further apart, on two layers, or a net with one way or three;
2. far_tooth: a tooth on the far face; a side-face tooth with every berth of its lane past the array's other side
   face; not one with a berth within the array's span, not one toward the destination, not an option with no move.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('awx', 'py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))
try:
    import rules  # noqa: E402
    rules.install_defaults()
    import group_ends as ge  # noqa: E402
except ImportError as exc:
    print(f'SKIP: {exc}')
    sys.exit(77)

N, BAD = [0], []


def check(what, got, want):
    N[0] += 1
    ok = got == want
    if not ok:
        BAD.append(what)
    print(f'{"ok  " if ok else "FAIL"} {what}' + ('' if ok else f': got {got!r}, want {want!r}'))


# ---- 1. the join of a short lane's two ways out
LAYERS = ['F.Cu', 'In1.Cu', 'B.Cu']
REACH = 2.0
ways = [(1, (10.0, 10.0), 'up', 'B.Cu'), (1, (8.5, 9.2), 'down', 'B.Cu'),          # 1.7 mm apart, one layer
        (2, (20.0, 10.0), 'up', 'B.Cu'), (2, (20.0, 3.0), 'down', 'B.Cu'),         # 7 mm apart
        (3, (30.0, 10.0), 'up', 'B.Cu'), (3, (30.5, 9.0), 'down', 'F.Cu'),         # two layers
        (4, (40.0, 10.0), 'up', 'B.Cu'),                                           # one way out
        (5, (50.0, 10.0), 'up', 'F.Cu'), (5, (50.5, 9.5), 'down', 'F.Cu'), (5, (51.0, 9.0), 'left', 'F.Cu'),
        (6, (60.0, 10.0), 'up', 'In9.Cu'), (6, (60.5, 9.5), 'down', 'In9.Cu')]     # a layer the board has not
got = ge.join_runs(ways, LAYERS, REACH)
w_way = ge.wayout_runs(ways, LAYERS)[0]['width']
check('1. one join: the net whose two exits stand within twice the reach on one layer', [j['net_id'] for j in got], [1])
check('1. ...from one exit to the other, on their layer', (sorted([got[0]['start'], got[0]['end']]), got[0]['layer']),
      ([(8.5, 9.2), (10.0, 10.0)], 'B.Cu'))
check('1. ...as wide as a way out', got[0]['width'], w_way)
check('1. a shorter reach: none', ge.join_runs(ways, LAYERS, 0.8), [])
check('1. no way out at all: none', ge.join_runs([], LAYERS, REACH), [])

# ---- 2. a tooth that takes its lane round the array
E = types.SimpleNamespace(st={'sgrid': types.SimpleNamespace(bbox=(0.0, 0.0, 10.0, 10.0))},
                          B={'past': [(None, (15.0, 14.0)), (None, (15.0, 12.0))],        # every berth past y 10
                             'span': [(None, (15.0, 14.0)), (None, (15.0, 6.0))],         # one within the span
                             'before': [(None, (15.0, -3.0))]})                           # past the other side
opt = lambda d: ([types.SimpleNamespace(direction=d)], (0.0, 0.0))     # noqa: E731
check('2. a tooth on the far face', ge.far_tooth(E, 'span', opt('left')), True)
check('2. a side-face tooth, every berth past the other side face', ge.far_tooth(E, 'past', opt('up')), True)
check('2. ...the same on the other hand', ge.far_tooth(E, 'before', opt('down')), True)
check('2. the side face toward its berths: no', ge.far_tooth(E, 'past', opt('down')), False)
check('2. a berth within the array\'s span: no', ge.far_tooth(E, 'span', opt('up')), False)
check('2. a tooth toward the destination: no', ge.far_tooth(E, 'past', opt('right')), False)
check('2. an option with no move: no', ge.far_tooth(E, 'past', ([], (0.0, 0.0))), False)
check('2. a lane with no berth read: no', ge.far_tooth(E, 'none', opt('up')), False)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
