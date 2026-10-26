#!/usr/bin/env python3
"""#1449: the in-run audit fills the pours at the edge clearance the run routes at.

route.py routes at the fab-edge pin (#441, 0.2 mm) on a board whose project
declares less, but the pin reached the project only in main()'s writeback,
after the plane finalize's oracle had refilled the board under the INPUT's
sub-fab edge rule. A pour neck beside the board edge, wide enough there but
not at the pin, survived the audit's fill and was gone from the shipped one:
the oracle read the pour whole, strapped nothing, and the output's GND pour
was split. The in-run floor sync (#650, `apply_routed_floors`) now writes the
edge too, the way the writeback does (raised, never lowered).

  1. apply_routed_floors raises a sub-fab edge rule to the run's edge
     clearance, keeps a board rule above it, and leaves the edge alone when no
     edge clearance is given;
  2. (kicad-cli present) routing the fixture -- a GND pour whose only path is
     a neck that closes at the pin -- straps GND in the oracle and ships a
     connected board.

    python3 tests/test_1449_inrun_edge_floor.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from fix_kicad_drc_settings import apply_routed_floors, fab_edge_floor  # noqa: E402
from kicad_oracle import find_kicad_cli                                 # noqa: E402
import run_utils                                                        # noqa: E402

FIXTURE = os.path.join(TESTS_DIR, 'fixtures', '1449', 'edge_neck.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def edge_rule(pro):
    return json.load(open(pro))['board']['design_settings']['rules'].get(
        'min_copper_edge_clearance')


def copy_fixture(tmp, name):
    dst = os.path.join(tmp, f'{name}.kicad_pcb')
    shutil.copyfile(FIXTURE, dst)
    shutil.copyfile(FIXTURE[:-len('.kicad_pcb')] + '.kicad_pro',
                    dst[:-len('.kicad_pcb')] + '.kicad_pro')
    return dst


def set_edge(pcb, value):
    pro = pcb[:-len('.kicad_pcb')] + '.kicad_pro'
    p = json.load(open(pro))
    p['board']['design_settings']['rules']['min_copper_edge_clearance'] = value
    json.dump(p, open(pro, 'w'), indent=2)
    return pro


def unit(tmp):
    run_utils.evidence(FIXTURE, 'the #1449 fixture board')
    fab = fab_edge_floor(FIXTURE)
    check('the fab-edge pin is above the fixture rule (non-vacuity)', fab > 0.05,
          f'pin {fab}')
    pcb = copy_fixture(tmp, 'unit')
    pro = set_edge(pcb, 0.05)

    apply_routed_floors(pcb, clearance=0.2)
    check('no edge clearance given: the edge rule is left alone',
          edge_rule(pro) == 0.05, f'{edge_rule(pro)}')

    changes = apply_routed_floors(pcb, clearance=0.2, edge_clearance=fab)
    check('a sub-fab edge rule is raised to the routed edge',
          abs(edge_rule(pro) - fab) < 1e-9, f'{edge_rule(pro)}')
    check('...and the change is named as the fab-edge pin',
          any('min_copper_edge_clearance' in c and 'fab-edge pin' in c
              for c in changes), str(changes))
    check('idempotent', apply_routed_floors(pcb, clearance=0.2,
                                            edge_clearance=fab) == [])

    set_edge(pcb, 0.5)
    apply_routed_floors(pcb, clearance=0.2, edge_clearance=fab)
    check('a board rule above the routed edge is kept',
          edge_rule(pro) == 0.5, f'{edge_rule(pro)}')


def routed(tmp):
    if not find_kicad_cli(warn=False):
        print('  [skip] kicad-cli not present: the routed leg (oracle refill) '
              'not run')
        return
    pcb = copy_fixture(tmp, 'in')
    out = os.path.join(tmp, 'out.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', pcb, out,
                        '--nets', '*'], cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    log = r.stdout + r.stderr
    check('route.py exits 0', r.returncode == 0, log[-400:] if r.returncode else '')
    run_utils.evidence(out, 'the routed board')
    m = re.search(r'^JSON_ORACLE: (\{.*\})$', log, re.M)
    oracle = json.loads(m.group(1)) if m else {}
    check('the oracle ran', oracle.get('available') is True, str(oracle)[:200])
    check('the in-run sync raised the edge before the audit',
          bool(re.search(r'In-run DRC floors.*\n(?:\s+rules\.\S+.*\n)*\s+rules\.'
                         r'min_copper_edge_clearance: 0\.05 -> ', log)),
          '' if 'In-run DRC floors' in log else 'no in-run floor sync in the log')
    check('the oracle straps the pour split at the pinned edge',
          oracle.get('links_routed', 0) >= 1 and oracle.get('remaining') == 0,
          f"links_routed {oracle.get('links_routed')}, "
          f"remaining {oracle.get('remaining')}")
    c = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_connected.py',
                        out], cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    check('check_connected (with its KiCad refill cross-check) passes the output',
          c.returncode == 0 and 'ALL NETS FULLY CONNECTED' in c.stdout,
          (c.stdout + c.stderr)[-300:])


def main():
    tmp = tempfile.mkdtemp(prefix='t1449_')
    try:
        unit(tmp)
        routed(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
