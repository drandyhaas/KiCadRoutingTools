#!/usr/bin/env python3
"""#1305: a rotated footprint's pads turn with it whatever tokens precede
their `(at ...)`.

KiCad 6 writes `locked` on every pad, `(pad "1" smd rect locked (at ...))`,
and old files write the pad number bare. The placement writer's pattern
allowed exactly two tokens (type, shape) between the quoted number and
`(at`, so on steppenprobe (a KiCad 6 file, 312 such pads) every rotation and
every pile write left the pads at their old absolute angles, turned against
their bodies: 175 overlapping different-net pad pairs on the pile.

    python3 tests/test_1305_locked_pad_rotation.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from placement.writer import _rotate_pad_angles                # noqa: E402

FP = """(footprint "Package_SO:SOIC-8" locked (layer "F.Cu")
    (at 10 20)
    (fp_text reference "U4" (at 0 -3.5) (layer "F.SilkS"))
    (pad "1" smd rect locked (at -2.7 -1.905) (size 1.5 0.6) (layers "F.Cu"))
    (pad "2" smd roundrect locked
      (at -2.7 -0.635 180) (size 1.5 0.6) (layers "F.Cu"))
    (pad 3 smd rect (at -2.7 0.635) (size 1.5 0.6) (layers F.Cu))
    (pad "4" smd rect (at -2.7 1.905) (size 1.5 0.6) (layers "F.Cu"))
  )"""


def _pad_angles(text):
    return {m.group(1): m.group(2) for m in re.finditer(
        r'\(pad\s+"?([^"\s]+)"?[^(]*\(at\s+\S+\s+[^\s)]+(?:\s+([^\s)]+))?\)',
        text)}


def test_every_pad_turns():
    out = _rotate_pad_angles(FP, 90)
    assert _pad_angles(out) == {'1': '90', '2': '270', '3': '90', '4': '90'}, (
        _pad_angles(out))
    # Positions pass through as text; the footprint and text `(at)` are not
    # pads and are left to their own handlers.
    assert '(at -2.7 -1.905 90)' in out and '(at 10 20)' in out
    assert '(at 0 -3.5)' in out
    print("  PASS: locked, bare-number and ordinary pads all turn by 90")


def test_full_turn_drops_the_angle():
    out = _rotate_pad_angles(_rotate_pad_angles(FP, 90), 270)
    assert _pad_angles(out) == {'1': None, '2': '180', '3': None, '4': None}, (
        _pad_angles(out))
    print("  PASS: a full turn writes the pads back at their angles")


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        fn()
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
