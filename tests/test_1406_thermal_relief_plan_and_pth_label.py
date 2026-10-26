#!/usr/bin/env python3
"""#1406: the plan checker and the skill agree on `--thermal-relief`, and the
KiCad cross-check names a KiCad 10 through-hole starved thermal.

  1. route_plan_check R07 refuses `--thermal-relief` on a board with no relief
     pour to keep (and with no --board at all), and accepts it when
     route_planes re-pours a net whose existing pour is thermal-connected
     (SKILL.md Step 3, "A pour the board already has", #1329).
  2. kicad_drc_compare names "PTH pad 7 [GND] of JP1" as JP1.7 at the pad's
     own position, as it names an SMD "Pad 1 [GND] of BUS1".

    python3 tests/test_1406_thermal_relief_plan_and_pth_label.py
"""
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check, evidence                      # noqa: E402

CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                       'scripts', 'route_plan_check.py')
RELIEF = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')      # GND pour: thermal
NO_POUR = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted.kicad_pcb')

PLAN = """#!/bin/bash
set -e
cd /repo
# cwd=/repo
python3 -u -X utf8 py_router/route_planes.py in.kicad_pcb pour.kicad_pcb \
--nets GND --plane-layers B.Cu --thermal-relief
# cwd=/repo
python3 -u -X utf8 py_router/route.py pour.kicad_pcb final.kicad_pcb \
--nets '*' --power-nets GND --power-nets-widths 0.5 --clearance 0.09
# Verification
#   python3 -X utf8 py_router/check_drc.py final.kicad_pcb
#   python3 -X utf8 py_router/check_connected.py final.kicad_pcb
#   python3 -X utf8 py_tools/check_orphan_stubs.py final.kicad_pcb
"""


def checker(plan, *extra, **kw):
    return check([sys.executable, '-X', 'utf8', CHECKER, plan, *extra], **kw)


def plan_checks(tmp):
    plan = os.path.join(tmp, 'plan.sh')
    with open(plan, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(PLAN)
    evidence(plan, 'the plan')
    evidence(RELIEF, 'the relief-pour board')
    evidence(NO_POUR, 'the pour-less board')
    # no board: the claim cannot be shown, so the flag is refused
    checker(plan, code=4, refuse='FAIL  R07')
    # a board whose pour is NOT a relief pour: refused
    checker(plan, '--board', NO_POUR, code=4, refuse='FAIL  R07')
    # re-pouring a thermal-relief pour and keeping its relief: R07 holds
    r = checker(plan, '--board', RELIEF, accept=True)
    assert 'FAIL' not in r.stdout, r.stdout
    print('  [ok] R07 refuses --thermal-relief unless it keeps a relief pour')


def label_checks():
    import kicad_drc_compare as kdc

    def item(desc, x, y):
        return {'type': 'starved_thermal', 'severity': 'error',
                'description': 'Thermal relief connection to zone incomplete',
                'items': [{'description': 'Zone [GND] on B.Cu',
                           'pos': {'x': 79.875, 'y': 34.79}},
                          {'description': desc, 'pos': {'x': x, 'y': y}}]}
    rows = kdc.parse_kicad_report({'violations': [
        item('PTH pad 7 [GND] of JP1', 10.5, 20.5),
        item('Pad 1 [GND] of BUS1 on bottom_copper', 30.0, 40.0)]})
    th = {r['pads']: r for r in rows if r['type'] == 'starved_thermal'}
    assert set(th) == {('JP1.7',), ('BUS1.1',)}, sorted(th)
    assert th[('JP1.7',)]['pos'] == (10.5, 20.5), th[('JP1.7',)]['pos']
    print('  [ok] a KiCad 10 PTH pad is named, at its own position')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as tmp:
        plan_checks(tmp)
    label_checks()
    print('All checks passed')
