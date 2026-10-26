#!/usr/bin/env python3
"""#1404: copper is cleared against an Edge.Cuts arc at the exact edge floor.

The outline's arcs and circles were linearised with a 5 um chord sag (circles
with 64 fixed segments: 24 um on a 20 mm radius). A chord lies inside its arc,
so beside a convex edge -- a round cutout seen from the copper side -- the
polygon is that sag farther away than the true edge, and a track cleared to
the floor against the polygon sat inside the floor against the arc: KiCad
reported copper_edge_clearance (0.1980 against 0.2000) and check_drc at
--clearance-margin 0 reported it too.

Rows:
  1. Edge.Cuts arcs and circles are linearised at <= 0.5 um sag, on the text
     path (board-level and footprint-embedded) -- measured on the segments.
  2. A round cutout drawn as two gr_arcs whose top vertex sits between grid
     columns: route.py at a 0.2 mm edge floor and 0.2 mm tracks ships copper
     that check_drc grades clean at --clearance-margin 0.

    python3 tests/test_1404_edge_arc_chord.py
"""
import math
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import kicad_parser as kp                                        # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def max_sag(segs, cx, cy, r):
    """Largest distance a chord midpoint falls inside the circle."""
    return max(r - math.hypot((a[0] + b[0]) / 2 - cx, (a[1] + b[1]) / 2 - cy)
               for a, b in segs)


def main():
    # 1. The linearisation of the outline's curves.
    arc = '(gr_arc (start 21.5 10) (mid 20 11.5) (end 18.5 10) (layer "Edge.Cuts"))\n'
    segs = kp._collect_edge_cuts_segments(arc)
    sag = max_sag(segs, 20, 10, 1.5)
    check('1. a gr_arc on Edge.Cuts has a chord sag <= 0.5 um', sag <= 0.0005 + 1e-9,
          f'{sag * 1000:.2f} um over {len(segs)} segment(s)')
    circ = '(gr_circle (center 40 40) (end 60 40) (layer "Edge.Cuts"))\n'
    segs = kp._collect_edge_cuts_segments(circ)
    sag = max_sag(segs, 40, 40, 20)
    check('1. a gr_circle on Edge.Cuts (r 20 mm) has a chord sag <= 0.5 um',
          sag <= 0.0005 + 1e-9, f'{sag * 1000:.2f} um over {len(segs)} segment(s)')
    plain = kp._arc_to_segments((21.5, 10), (20, 11.5), (18.5, 10))
    check('1. precondition: the default sag (copper arcs) is coarser',
          max_sag(plain, 20, 10, 1.5) > 0.0005 and len(plain) < len(
              kp._collect_edge_cuts_segments(arc)))

    # 2. A route beside a round cutout.
    s = open(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'),
             encoding='utf-8').read()
    i = s.rindex('\n\t(embedded_fonts no)')
    arcs = ('\n\t(gr_arc (start 15.55 14.798) (mid 14.05 16.298) (end 12.55 14.798) '
            '(stroke (width 0.05) (type default)) (layer "Edge.Cuts") '
            '(uuid "6a1e0b2c-0000-4000-8000-000000000001"))'
            '\n\t(gr_arc (start 12.55 14.798) (mid 14.05 13.298) (end 15.55 14.798) '
            '(stroke (width 0.05) (type default)) (layer "Edge.Cuts") '
            '(uuid "6a1e0b2c-0000-4000-8000-000000000002"))')
    tmp = tempfile.mkdtemp(prefix='t1404_')
    src = os.path.join(tmp, 'hole.kicad_pcb')
    out = os.path.join(tmp, 'routed.kicad_pcb')
    open(src, 'w', encoding='utf-8').write(s[:i] + arcs + s[i:])
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', src, out,
                        '--nets', '*', '--board-edge-clearance', '0.2',
                        '--track-width', '0.2'], cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
    check('2. route.py ran', os.path.isfile(out), r.stdout[-300:] + r.stderr[-300:])
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py', out,
                        '--clearance-margin', '0'], cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
    check('2. check_drc at margin 0 finds no edge graze', 'NO DRC VIOLATIONS FOUND' in r.stdout,
          r.stdout[-400:])

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
