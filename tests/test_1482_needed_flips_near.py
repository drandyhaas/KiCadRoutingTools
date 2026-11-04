"""#1482: a part that may not flip makes room by flipping only the flippable
parts of its group NEAR where it wants to be.

The face pass used to send every flippable part of the group to the back,
when far members free no room where the needy part needs it. Now
`faces.needed_flips` (given the model) takes the group's flippable parts on
the needy part's face within its longer side of its target, nearest first,
until their courtyards add up to its own; with none in reach, none. A needy
part that is flippable itself still goes alone, and without a model the
whole group goes, as before.
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from placement import faces                                   # noqa: E402
from placement import simple                                  # noqa: E402

FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


class Part:
    def __init__(self, x, y, w, h, sides=frozenset('F')):
        self.q, self.sides = (x - w / 2, y - h / 2, x + w / 2, y + h / 2), sides

    def rect(self):
        return self.q


class State:
    def __init__(self, parts):
        self.parts = parts


class Model:
    def __init__(self, parts):
        self.state = State(parts)


def main():
    # U1 (4 x 4 = 16 mm2, not flippable) wants to be at (0, 0): C1 (2 x 2)
    # touches that spot, C6 (4 x 4) is 0.5 mm off it, C2 1 mm, C3 2.5 mm,
    # C4 far away and C5 on the back already
    parts = {'U1': Part(10, 10, 4, 4),
             'C1': Part(1, 0, 2, 2), 'C2': Part(-2, 0, 2, 2), 'C3': Part(0, 3.5, 2, 2),
             'C4': Part(30, 30, 2, 2), 'C5': Part(0.5, 0.5, 2, 2, frozenset('B')),
             'C6': Part(0, -2.5, 4, 4)}
    model = Model(parts)
    doc = {'groups': [{'name': 'core', 'refs': sorted(parts)}]}
    flippable = {'C1', 'C2', 'C3', 'C4', 'C5', 'C6'}
    saved = simple.target
    simple.target = lambda mdl, ref, placed: (0.0, 0.0)
    try:
        need = {'U1': 'no legal pose'}
        got = faces.needed_flips(need, flippable, doc, set(), model)
        ok('nearest first until their area covers the needy part, no further',
           set(got) == {'C1', 'C6'}, sorted(got))
        ok('a far member and one on the other face are not flipped',
           not {'C4', 'C5'} & set(got), sorted(got))
        ok('each flip says whose room it makes',
           all('room for U1' in why for why in got.values()), got)
        got = faces.needed_flips(need, flippable, doc, {'C1', 'C6'}, model)
        ok('a later round takes the next nearest, not ones already moved',
           set(got) == {'C2', 'C3'}, sorted(got))
        simple.target = lambda mdl, ref, placed: (60.0, 60.0)
        ok('none in reach, none flipped',
           faces.needed_flips(need, flippable, doc, set(), model) == {})
        ok('a flippable needy part goes alone',
           set(faces.needed_flips({'C4': 'past its claim limit'}, flippable, doc, set(),
                                  model)) == {'C4'})
        ok('without a model, the whole group as before',
           set(faces.needed_flips(need, flippable, doc, set())) == flippable)
    finally:
        simple.target = saved
    ok('FACE_ROUNDS allows a second round', faces.FACE_ROUNDS == 2)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
