#!/usr/bin/env python3
"""#1037: place_portfolio gates on intent errors a candidate ADDS.

`score_candidate` called the same `floorplan.grade` check_floorplan does, but
gated on the ABSOLUTE error count, and the quench itself moves decaps -- so a
board carrying 11 pre-existing errors (glasgow_revC, run 32) could never
produce an admissible candidate, even from the near-identity `poses` strategy.
It now gates on `floorplan.grade_delta(input, candidate)`, the exit gate's own
currency (the seeder's no-worse test uses it too).

Synthetic, always runs: splitflap_driver with `decaps.max_distance_mm 2.0`,
where the INPUT already fails decap_distance on C2 and C8.

1. A candidate identical to the input is admissible, and records
   input_errors 2 / new_errors 0.
2. A candidate with C1 moved 3 mm (+x, a clear spot) is gated, and the reason names
   C1 -- not the inherited C2 / C8.
3. Without `input_violations` (the legacy callers) the absolute gate stands.

Run:
    python3 tests/test_1037_portfolio_new_errors.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan, portfolio  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def score(board, intent, input_violations):
    c = portfolio.Candidate(index=1, strategy='test', board=board,
                            metrics={})
    portfolio.score_candidate(
        c, free=[], baseline_overlap=1e9, baseline_oob=10 ** 6,
        baseline_pad_pairs=10 ** 6, baseline_hole_shortfall=1e9,
        baseline_keepout_parts=10 ** 6, clearance=0.2,
        board_edge_clearance=0.5, grid_step=0.1, ignore_nets=None,
        intent=intent, input_violations=input_violations)
    return c


def main():
    work = tempfile.mkdtemp(prefix='krt1037_')
    try:
        doc = floorplan.emit_intent(parse_kicad_pcb(BOARD), BOARD)
        doc['decaps'] = {'max_distance_mm': 2.0}
        ipath = os.path.join(work, 'intent.json')
        with open(ipath, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        intent = floorplan.load_intent(ipath)

        inp = os.path.join(work, 'input.kicad_pcb')
        copy_board(BOARD, inp)
        g = floorplan.grade(intent, parse_kicad_pcb(inp), inp, clearance=0.2,
                            board_edge_clearance=0.5, with_health=True)
        in_err = sorted((v.rule, v.ref) for v in g.errors)
        check('0. the input already fails decap_distance on C2 and C8',
              in_err == [('decap_distance', 'C2'), ('decap_distance', 'C8')],
              str(in_err))

        same = os.path.join(work, 'same.kicad_pcb')
        copy_board(BOARD, same)
        c = score(same, intent, list(g.violations))
        check('1. an unchanged candidate is admissible',
              c.gates.get('passed') is True, str(c.gates))
        check('1. ...and records input_errors 2, new_errors 0',
              c.intent.get('input_errors') == 2
              and c.intent.get('new_errors') == 0
              and c.intent.get('errors') == 2, str(c.intent))

        moved = os.path.join(work, 'moved.kicad_pcb')
        copy_board(BOARD, moved)
        fp = parse_kicad_pcb(BOARD).footprints['C1']
        write_placed_output(moved, moved, [{
            'reference': 'C1', 'new_x': fp.x + 3.0, 'new_y': fp.y,
            'new_rotation': fp.rotation or 0.0}])
        c = score(moved, intent, list(g.violations))
        why = ' '.join(c.gates.get('reasons') or [])
        check('2. a candidate that adds decap_distance C1 is gated',
              c.gates.get('passed') is False and 'NEW intent error' in why,
              str(c.gates))
        check('2. ...naming C1 only, not the inherited C2 / C8',
              'C1 ' in why and 'C2 ' not in why and 'C8 ' not in why
              and c.intent.get('new_errors') == 1
              and [(d['rule'], d.get('ref')) for d in c.intent['new']]
              == [('decap_distance', 'C1')], why + ' ' + str(c.intent))

        c = score(same, intent, None)
        check('3. without input_violations the absolute gate stands',
              c.gates.get('passed') is False
              and '2 intent violation(s)' in ' '.join(c.gates['reasons']),
              str(c.gates))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if FAILS:
        print('\nFAILED: %d check(s): %s' % (len(FAILS), FAILS))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
