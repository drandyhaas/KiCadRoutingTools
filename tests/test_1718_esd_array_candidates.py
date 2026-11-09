#!/usr/bin/env python3
"""#1718: an ESD array its pins cannot name is an `unknown` candidate.

`emc_roles._esd_arrays` reads an array only by I/O<n> / CH<n> pin
functions. The same USB array on the same nets with its pin functions
removed, or with its lines named after the signals (`D+` / `D-`, as some
stock symbols name them), was no role and no `unknown` either: nothing
proposed its row to the connector and nothing said one was missed, while a
pin-less converter or crystal is reported `unknown` with a `candidate`.

`_esd_candidates` now reads such a part off its nets: a chip or diode with
a pad on ground whose other pads are supplies, lines a receptacle carries
(one receptacle carrying every line) or their flow-through far side. It is
reported `role: unknown`, `candidate: esd array`, naming the connector and
the lines, with no row.

Cases, on a tracked board whose USB array is named I/O1 / I/O2 and routed
flow-through (connector-side and circuit-side nets differ):
  1. as tracked: still the `esd array` role with its row, and no candidate;
  2. pin functions stripped: an `unknown` esd-array candidate at the same
     connector, its four line nets named, no row;
  3. lines renamed D+ / D-: the same candidate;
and the refusals:
  4. a regulator whose input comes off two connectors (its input is a
     supply), on a board whose parts carry no pin functions, is no
     candidate;
  5. logic chips whose every net a pin HEADER breaks out are no candidate.

    python3 tests/test_1718_esd_array_candidates.py
"""
import contextlib
import io
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement.emc_roles import emc_roles                     # noqa: E402
from run_utils import evidence                                # noqa: E402

RUN_ALL_TIMEOUT = 600
ARRAY_BOARD = os.path.join(ROOT, 'kicad_files',
                           'rp2350_fpga_eensy_prePlane.kicad_pcb')
PINLESS_BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
HEADER_BOARD = os.path.join(ROOT, 'kicad_files',
                            'lvds_converter_dualclk.kicad_pcb')
ARRAY, CONN = 'U3', 'J1'
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _roles(path):
    evidence(path, 'tracked board')
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return emc_roles(parse_kicad_pcb(path))['roles']


def _block(text, ref):
    """The paren-balanced footprint block that carries reference `ref`."""
    k = text.index(f'(property "Reference" "{ref}"')
    i = text.rfind('(footprint ', 0, k)
    depth, j = 0, i
    while True:
        depth += {'(': 1, ')': -1}.get(text[j], 0)
        j += 1
        if depth == 0:
            return i, j


def _variant(tmp, name, edit):
    with open(ARRAY_BOARD, encoding='utf-8') as fh:
        text = fh.read()
    i, j = _block(text, ARRAY)
    path = os.path.join(tmp, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text[:i] + edit(text[i:j]) + text[j:])
    return path


def _candidates(roles):
    return [r for r in roles if r['role'] == 'unknown'
            and r.get('candidate') == 'esd array']


def test_tracked_board_keeps_its_role():
    roles = _roles(ARRAY_BOARD)
    mine = [r for r in roles if r['part'] == ARRAY]
    check('as tracked: the array keeps its esd array role and its row',
          [(r['role'], r.get('connector'), bool(r['proximity']))
           for r in mine] == [('esd array', CONN, True)], mine)
    check('as tracked: no esd-array candidate anywhere',
          not _candidates(roles), _candidates(roles))


def test_unnamed_and_line_named_arrays_are_candidates():
    with tempfile.TemporaryDirectory() as tmp:
        for name, edit in (
                ('pinless', lambda b: re.sub(r'\s*\(pinfunction "[^"]*"\)',
                                             '', b)),
                ('line_named', lambda b: b.replace('"I/O1_', '"D+_')
                 .replace('"I/O2_', '"D-_'))):
            roles = _roles(_variant(tmp, name, edit))
            mine = [r for r in roles if r['part'] == ARRAY]
            check(f'{name}: one record for the array, an unknown esd-array '
                  f'candidate at its connector',
                  [(r['role'], r.get('candidate'), r.get('connector'))
                   for r in mine] == [('unknown', 'esd array', CONN)], mine)
            if len(mine) != 1:
                continue
            r = mine[0]
            lines = r['candidate_lines']
            at = [l for l in lines if l['at_connector']]
            check(f'{name}: no row is proposed', r['proximity'] == [], r)
            check(f'{name}: four line nets, two of them the connector\'s, '
                  f'each with the connector pads it shares',
                  len(lines) == 4 and len(at) == 2
                  and all(l['connector_pads'] for l in at), lines)
            check(f'{name}: every line pad is a pad of the array, once',
                  sorted(p for l in lines for p in l['pads'])
                  == ['1', '3', '4', '6'], lines)
            check(f'{name}: the why says the pins are not read and names '
                  f'the connector',
                  'never guessed' in r['why'] and CONN in r['why'], r['why'])


def test_a_regulator_and_a_header_are_no_candidates():
    roles = _roles(PINLESS_BOARD)
    check('a pin-less regulator fed off two connectors is no candidate',
          not _candidates(roles), _candidates(roles))
    roles = _roles(HEADER_BOARD)
    check('logic chips broken out on a pin header are no candidate',
          not _candidates(roles), _candidates(roles))


def main():
    for t in (test_tracked_board_keeps_its_role,
              test_unnamed_and_line_named_arrays_are_candidates,
              test_a_regulator_and_a_header_are_no_candidates):
        print(t.__name__)
        t()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASSED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
