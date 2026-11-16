#!/usr/bin/env python3
"""#1772: the figures an unseated edge part's reason prints can be copied back.

On #1603's fixture under a 0.4 mm project edge floor, J3's pad floor holds it at
an overhang of 11.5250 mm (its pads 0.405 mm in: the floor plus the seat's 5 um
copper slack). The reason printed that reach rounded to nearest, "at most
11.53mm", so a minimum copied from it was refused with "at most 11.53mm, under
the declared minimum 11.53mm" and "its pads would stand 0.40mm from the edge"
against a 0.40 mm floor. Now:

  1. an "at most" reach rounds DOWN, an "at least" reading UP, a declared number
     prints as declared;
  2. the pad-floor clause says where the seat holds the pads (0.405) beside
     where the minimum would put them, to 0.001 mm, and names the slack when it
     is the whole shortfall;
  3. the printed reach, declared as the minimum, seats the part;
  4. the band test stays exact: a minimum 0.1 um past the reach is refused.

    python3 tests/test_1772_seat_figures_copyable.py
"""
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from test_1603_body_in_board import FIXTURE  # noqa: E402
from test_1663_seat_pad_floor_words import board, seat  # noqa: E402

RUN_ALL_TIMEOUT = 600
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    from placement import edge_seat, floorplan
    from placement.body import board_bodies
    print('the rounding helpers:')
    check('at most rounds down', edge_seat._at_most(11.525000001) == '11.52'
          and edge_seat._at_most(11.53) == '11.53', edge_seat._at_most(11.525000001))
    check('at least rounds up', edge_seat._at_least(12.4700001) == '12.48'
          and edge_seat._at_least(12.47) == '12.47', edge_seat._at_least(12.4700001))
    check('a declared number prints as declared', edge_seat._said(11.525) == '11.525'
          and edge_seat._said(11.53) == '11.53' and edge_seat._said(0.4) == '0.40')
    tmp = tempfile.mkdtemp(prefix='t1772_')
    try:
        path = board(tmp, 'floor04', pro=0.4)
        print("#1663's band (min 11.83):")
        _g, why, _st, _pcb = seat(path, {'min': 11.83, 'max': 12.43})
        text = why.get('J3', '')
        check('the reach prints rounded down', 'at most 11.52mm, under the declared minimum '
              '11.83mm' in text, text)
        check('...the seat holds the pads 0.405 mm in, the floor and the slack named',
              'the seat holds its pads 0.405mm inside the edge, 0.40mm from the board\'s '
              'min_copper_edge_clearance plus its 0.005mm copper slack' in text, text)
        check('...and the minimum would put them 0.100 mm from the edge',
              'at that minimum its pads would stand 0.100mm from the edge' in text, text)
        check('...a 0.30 mm shortfall is not called slack', 'short of it' not in text, text)

        print('the old printed figure, 11.53, declared:')
        _g, why, _st, _pcb = seat(path, {'min': 11.53, 'max': 12.43})
        text = why.get('J3', '')
        check('refused, and the two figures no longer print equal',
              'at most 11.52mm, under the declared minimum 11.53mm' in text, text)
        check('...the minimum would put them 0.400 mm out, the seat holds them 0.405',
              'its pads would stand 0.400mm from the edge' in text
              and 'holds its pads 0.405mm' in text, text)
        check('...and the 5 um shortfall is named as the seat\'s copper slack',
              '0.005mm short of it' in text and 'copper slack' in text, text)

        print('the printed figure, declared:')
        got = re.search(r'at most ([0-9.]+)mm', text)
        low = float(got.group(1)) if got else None
        g, why, st, pcb = seat(path, {'min': low, 'max': 12.43})
        check(f'min {low}: seated', 'J3' in g and 'J3' not in why, str(why.get('J3')))

        print('the band test stays exact:')
        if 'J3' in g:
            probe = edge_seat._Probe(st, pcb, path, board_bodies(pcb, path),
                                     floorplan.outline_state(pcb, path))
            probe.faces['J3'] = g['J3'].face
            rot, line = sorted(g['J3'].lines.items())[0]
            a0, a1 = max(line.free, key=lambda f: f[1] - f[0])
            pose = (line.coord, (a0 + a1) / 2.0, rot)
            reach = probe.band('J3', 'east', pose)
            kind, _r = probe.why({'ref': 'J3', 'edge': 'east',
                                  'overhang_mm': {'min': reach + 1e-4, 'max': 12.43}},
                                 'east', pose, {})
            check('a minimum 0.1 um past the reach is refused', kind == 'band_min',
                  f'reach {reach:.6f}: {kind}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
