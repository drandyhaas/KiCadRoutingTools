#!/usr/bin/env python3
"""
#1433: a via must keep a copper ring around its hole.

The size and drill floors were each checked on their own, so
`route.py --via-size 0.45 --via-drill 0.508` laid 138 vias whose drill is
wider than their pad, exit 0 and no warning, and check_drc graded the result
clean while kicad-cli reported 138 annular_width.

  1. fab_tiers.via_size_for_annular_ring pins the size UP to drill + 2 x the
     ring the physical floor's via implies (0.05 on the advanced rung), and
     leaves a via that already holds its drill alone;
  2. enforce_fab_floors (route.py, route_diff.py, route_planes.py and
     repair_planes.py all call it) warns, naming both values, and returns
     the pinned size;
  3. check_drc reports a `via-annular` violation for the bad via and none
     for a good one, with the diameter and drill floors set low enough
     that only the ring check can catch it (the issue's case).

Run:
    python3 tests/test_1433_via_annular_ring.py
"""

import contextlib
import io
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from run_utils import tool
import fab_tiers

BOARD = '''(kicad_pcb
\t(version 20240108)
\t(generator "pcbnew")
\t(general (thickness 1.6))
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(31 "B.Cu" signal)
\t\t(44 "Edge.Cuts" user)
\t)
\t(net 0 "")
\t(net 1 "A")
\t(net 2 "B")
\t(gr_rect (start 0 0) (end 30 30) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
\t(via (at 10 10) (size 0.45) (drill 0.25) (layers "F.Cu" "B.Cu") (net 1) (uuid "good"))
\t(via (at 20 20) (size 0.45) (drill 0.508) (layers "F.Cu" "B.Cu") (net 2) (uuid "bad"))
)
'''


def main():
    fails = []

    def check(name, ok, detail=''):
        print(('  PASS ' if ok else '  FAIL ') + name
              + ('' if ok else f'\n       {detail}'))
        if not ok:
            fails.append(name)

    # 1. The shared helper, on the default (advanced-rung) physical floor.
    ring = fab_tiers.min_via_annular_ring(2)
    check('the physical floor implies a 0.05 mm via ring', abs(ring - 0.05) < 1e-9,
          ring)
    check('a drill wider than the pad is pinned up to drill + 2 x ring',
          fab_tiers.via_size_for_annular_ring(0.45, 0.508, 2) == 0.608)
    check('a via that holds its drill is left alone',
          fab_tiers.via_size_for_annular_ring(0.45, 0.25, 2) == 0.45)
    check('a via exactly at the ring floor is left alone',
          fab_tiers.via_size_for_annular_ring(0.25, 0.15, 2) == 0.25)
    check('a missing value is passed through',
          fab_tiers.via_size_for_annular_ring(None, 0.3, 2) is None
          and fab_tiers.via_size_for_annular_ring(0.45, None, 2) == 0.45)

    # 2. The CLI guard every routing main calls.
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        pinned = fab_tiers.enforce_fab_floors(2, via_size=0.45, via_drill=0.508)
    check('enforce_fab_floors pins via_size to 0.608',
          pinned.get('via_size') == 0.608, pinned)
    check('its warning names both values',
          '--via-size 0.45' in out.getvalue()
          and '--via-drill 0.508' in out.getvalue(), out.getvalue())
    with contextlib.redirect_stdout(io.StringIO()):
        check('a good via is not pinned',
              'via_size' not in fab_tiers.enforce_fab_floors(
                  2, via_size=0.45, via_drill=0.25))

    # 3. check_drc grades the ring.
    tmp = tempfile.mkdtemp(prefix='t1433_')
    board = os.path.join(tmp, 'vias.kicad_pcb')
    with open(board, 'w') as f:
        f.write(BOARD)
    r = subprocess.run([sys.executable, tool('check_drc.py'), board,
                        '--clearance', '0.15', '--min-via-diameter', '0.25',
                        '--min-via-drill', '0.15', '--max-print', '0'],
                       capture_output=True, text=True)
    text = r.stdout + r.stderr
    check('check_drc reports exactly one via-annular violation',
          'VIA-ANNULAR violations (1)' in text, text[-1500:])
    check('it is the bad via (net B, ring -0.029)',
          'Via:B annular ring too small' in text and 'ring -0.0290mm' in text
          and 'Via:A annular' not in text, text[-1500:])
    check('check_drc exits non-zero', r.returncode != 0, r.returncode)

    if fails:
        print(f'\nFAIL ({len(fails)}): ' + '; '.join(fails))
        return 1
    print('\nAll checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
