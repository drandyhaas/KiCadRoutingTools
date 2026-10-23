#!/usr/bin/env python3
"""#1334: KiCad's `starved_thermal` is reported by the cross-check.

kicad_drc_compare filtered `starved_thermal` as "known-benign", and no other
grader reads it, so a DONE board shipped two starved-thermal ERRORS (gp14
04_multichannel_mixer: IC1.4 and IC2.4 GND, 1 spoke of 2) that nothing in the
repo named. It is now a labelled channel, like `connection_width` (#406):
graded at the PROJECT's severity, baseline-subtracted, reported with the pads
it names, and never part of the copper match, the CONSISTENT verdict or the
exit code.

No kicad-cli needed: the report parser is fed a JSON report, and the grading
core runs with kicad-cli's half stubbed (check_drc runs for real on
cap_chain).

    python3 tests/test_1334_starved_thermal_channel.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools', os.path.join('tests', 'stress')):
    sys.path.insert(0, os.path.join(ROOT, _d))

import kicad_drc_compare as kdc                      # noqa: E402
from copy_board import copy_board                    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def thermal(ref, pad, x, y, severity='error'):
    return {'type': 'starved_thermal', 'severity': severity,
            'description': 'Thermal relief connection to zone incomplete '
                           '(layer B.Cu; zone min spoke count 2; actual 1)',
            'items': [{'description': 'Zone [GND] on B.Cu',
                       'pos': {'x': 199.46, 'y': 154.66}},
                      {'description': f'Pad {pad} [GND] of {ref} on B.Cu',
                       'pos': {'x': x, 'y': y}}]}


def main():
    print("1: parse_kicad_report")
    rows = kdc.parse_kicad_report({'violations': [
        thermal('IC1', '4', 175.07, 89.63),
        thermal('IC9', '1', 10.0, 10.0, severity='warning'),
        {'type': 'silk_overlap', 'severity': 'error', 'items': []}]})
    th = [r for r in rows if r['type'] == 'starved_thermal']
    ok("an error-severity starved thermal is graded, a warning one is not",
       len(th) == 1 and th[0]['pads'] == ('IC1.4',), str(th))
    ok("it sits at the starved PAD, not the zone's anchor",
       th and th[0]['pos'] == (175.07, 89.63), str(th[0]['pos'] if th else None))
    ok("an out-of-scope class stays out", all(r['type'] != 'silk_overlap'
                                              for r in rows))

    print("2: the grading core")
    final_items = kdc.parse_kicad_report({'violations': [
        thermal('IC1', '4', 175.07, 89.63), thermal('IC2', '4', 145.25, 77.35)]})
    base_items = kdc.parse_kicad_report({'violations': [
        thermal('IC2', '4', 145.25, 77.35)]})
    saved = kdc.kicad_items_for
    kdc.kicad_items_for = lambda b, c=None: (
        (base_items if b.endswith('base.kicad_pcb') else final_items), None)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            d = kdc.compare_board_data(BOARD)
        ok("both are reported, with their pads",
           d['kicad_starved_thermal'] == 2
           and sorted(p for t in d['starved_thermal_items'] for p in t['pads'])
           == ['IC1.4', 'IC2.4'], str(d.get('kicad_starved_thermal')))
        ok("never in the copper count or the verdict",
           d['kicad'] == 0 and d['kicad_only'] == 0
           and d['verdict'] == 'CONSISTENT', f"{d['kicad']} {d['verdict']}")
        ok("a summary key", 'kicad_starved_thermal' in kdc._SUMMARY_KEYS)

        tmp = tempfile.mkdtemp(prefix='t1334_')
        try:
            base = os.path.join(tmp, 'base.kicad_pcb')
            with contextlib.redirect_stdout(io.StringIO()):
                copy_board(BOARD, base)
                d = kdc.compare_board_data(BOARD, baseline=base)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        ok("a baseline twin is subtracted, one for one",
           d['kicad_starved_thermal'] == 1
           and d['starved_thermal_preexisting'] == 1
           and d['starved_thermal_items'][0]['pads'] == ('IC1.4',),
           str((d['kicad_starved_thermal'], d['starved_thermal_preexisting'])))

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            kdc.compare_board(BOARD)
        out = buf.getvalue()
        ok("the report line and the item lines name them",
           'thermal=2' in out and 'STARVED-THERMAL IC1.4' in out, out[-300:])
    finally:
        kdc.kicad_items_for = saved

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
