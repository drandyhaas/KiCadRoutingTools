#!/usr/bin/env python3
"""check_drc's track-width and via-size floors compare at KiCad's own unit.

KiCad stores every size as a whole number of nanometres. A track routed at
exactly its floor parses back a float rounding below it, and the width check
compared with no tolerance, so tinytapeout_qfn's 31 tracks at a 0.0889 mm
class width graded as `track-width` violations reading "0.0889mm < min
0.0889mm (short 0.0000mm)".

Checks:
  1. a width or via size a float rounding under its floor passes;
  2. one a real 1 um under still fails, with its shortfall;
  3. an explicit `size_margin` above the 1 nm floor still widens it.

    python3 tests/test_check_drc_size_eps.py
"""
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from check_drc import check_track_width, check_via_size     # noqa: E402

FLOOR = 3.5 * 0.0254          # 0.0889 mm, a 3.5 mil class width


def main():
    at = SimpleNamespace(width=0.0889)
    assert FLOOR != at.width and FLOOR - at.width < 1e-9, (FLOOR, at.width)
    assert check_track_width(at, FLOOR) == (False, 0.0), check_track_width(at, FLOOR)
    thin = SimpleNamespace(width=FLOOR - 0.001)
    bad, short = check_track_width(thin, FLOOR)
    assert bad and abs(short - 0.001) < 1e-9, (bad, short)
    assert check_track_width(thin, FLOOR, size_margin=0.002) == (False, 0.0)
    print('  PASS: a track at its floor passes; 1 um under fails; a margin widens it')

    via = SimpleNamespace(size=0.45, drill=0.2)
    d_bad, h_bad, _d, _h = check_via_size(via, 0.45 + 1e-12, 0.2 + 1e-12)
    assert not d_bad and not h_bad, (d_bad, h_bad)
    d_bad, h_bad, _d, _h = check_via_size(via, 0.451, 0.201)
    assert d_bad and h_bad, (d_bad, h_bad)
    print('  PASS: a via at its floors passes; 1 um under fails')
    print('test_check_drc_size_eps: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
