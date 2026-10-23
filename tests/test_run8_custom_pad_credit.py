#!/usr/bin/env python3
"""A custom pad's copper is its primitives, not a box around its anchor.

The parser stores a CUSTOM pad as a symmetric box about its anchor, sized to
enclose the real primitives. That is deliberate and safe for clearance work
(it over-blocks), and wrong for CONNECTIVITY, because the anchor is frequently
a corner of the copper rather than its centre.

Measured on a recorded board: a MOSFET tab whose real copper is 4.57 x 5.01mm
was modelled as 9.13 x 10.01mm -- four times the area -- with the box centre
3.4mm away from the copper centre. The connectivity graph then credited any
track passing within min(size)/4 = 2.28mm of the ANCHOR as landing on the pad,
and that disc sat largely in empty space. Two islands of one net each reached
the phantom disc, so the net graded CONNECTED with its copper in two pieces --
and the router skipped it as already routed. A silent open.

The exact-distance path was always right; it uses the polygons. Only the credit
disc was derived from the box. The credit is now the exact overlap of the
track's copper with the pad's copper (check_connected.track_reaches_pad), on
the primitives themselves, so the box cannot enter it at all.

Run: python3 -X utf8 tests/test_run8_custom_pad_credit.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

from check_connected import track_reaches_pad                  # noqa: E402

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


class FakePad:
    def __init__(self, x, y, sx, sy, polygons=None, shape='rect'):
        self.global_x, self.global_y = x, y
        self.size_x, self.size_y = sx, sy
        self.polygons = polygons or []
        self.shape, self.roundrect_rratio, self.rect_rotation = shape, 0.0, 0.0


class FakeSeg:
    def __init__(self, x1, y1, x2, y2, width=0.25):
        self.start_x, self.start_y, self.end_x, self.end_y = x1, y1, x2, y2
        self.width = width


def main():
    import math
    print('an ordinary pad: a track through it is on it, one past it is not')
    p = FakePad(10.0, 20.0, 1.0, 2.0)
    check('a track through the pad', track_reaches_pad(FakeSeg(9.0, 20.5, 11.0, 20.5), p))
    check('a track 0.1mm clear of its edge',
          not track_reaches_pad(FakeSeg(10.725, 18.0, 10.725, 22.0), p))

    print('a custom pad is its real primitives')
    # The measured shape: copper 4.57 x 5.01 whose centre is offset from the
    # anchor by (-2.28, -2.5); the parser's box would be 9.13 x 10.01.
    poly = [[(94.407, 78.963), (98.977, 78.963),
             (98.977, 83.973), (94.407, 83.973)]]
    custom = FakePad(98.972, 83.968, 9.13, 10.01, poly, shape='custom')
    check('a track across the real copper is on it',
          track_reaches_pad(FakeSeg(93.0, 81.0, 100.0, 81.0), custom))

    print('a track in the phantom region is not')
    # Through a point just past the anchor, well outside the real copper, and
    # inside the old 2.28mm disc about the anchor.
    ghost = FakeSeg(100.5, 84.5, 100.5, 86.5)
    old_r = min(custom.size_x, custom.size_y) / 4
    check('the old disc about the anchor credited it',
          math.hypot(100.5 - custom.global_x, 85.5 - custom.global_y) <= old_r)
    check('the real copper does not', not track_reaches_pad(ghost, custom))

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
