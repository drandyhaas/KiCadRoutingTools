#!/usr/bin/env python3
"""A writeback that keeps a lower copper-to-hole floor says so.

A routing writer only ever LOWERS `rules.min_hole_clearance`. A later step
whose copper holds 0.25 used to print "Copper-to-hole floor written at 0.25
mm" and then "DRC settings already consistent" while the project kept the 0.2
an earlier step wrote. The step's line now says what its copper HOLDS, and
`fix_project_for_output` says when the project keeps a lower value.

Row: a project declaring 0.2, the writer given 0.25 -> the project keeps 0.2
and the console says the floor stays 0.2, below the 0.25 the step holds.

    python3 tests/test_hole_floor_stays_said.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence                                  # noqa: E402
from synth import RoutingBoard                                  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def project_doc(hole):
    return {"board": {"design_settings": {
                "rules": {"min_clearance": 0.0, "min_hole_clearance": hole,
                          "min_track_width": 0.15, "min_via_diameter": 0.5,
                          "min_through_hole_diameter": 0.2},
                "rule_severities": {}}},
            "meta": {"filename": "in.kicad_pro", "version": 1},
            "net_settings": {"classes": [{"name": "Default", "clearance": 0.2,
                                          "track_width": 0.2,
                                          "via_diameter": 0.6,
                                          "via_drill": 0.3}],
                             "meta": {"version": 3}}}


def main():
    from copy_board import copy_board
    from fix_kicad_drc_settings import fix_project_for_output
    tmp = tempfile.mkdtemp(prefix='hole_stays_')
    try:
        src = os.path.join(tmp, 'in.kicad_pcb')
        b = RoutingBoard(w=20, h=20)
        b.pad('C1', 5, 5, 'GND')
        with open(src, 'w', encoding='utf-8') as f:
            f.write(b.text())
        with open(os.path.join(tmp, 'in.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump(project_doc(0.2), f, indent=2)
        out = os.path.join(tmp, 'out.kicad_pcb')
        copy_board(src, out)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            fix_project_for_output(out, input_pcb=src, clearance=0.2,
                                   hole_clearance=0.25)
        log = buf.getvalue()
        pro = evidence(os.path.splitext(out)[0] + '.kicad_pro', 'output project')
        with open(pro, encoding='utf-8') as f:
            got = json.load(f)['board']['design_settings']['rules'].get(
                'min_hole_clearance')
        check('the project keeps 0.2', got is not None and abs(got - 0.2) < 1e-9,
              str(got))
        said = 'Copper-to-hole floor stays 0.2 mm, below the 0.25 mm' in log
        check('the writer says the floor stays 0.2', said,
              '' if said else log[-300:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASS'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
