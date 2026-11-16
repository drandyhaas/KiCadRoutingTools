#!/usr/bin/env python3
"""route_plan_check R19: a board with a bus runs the topo bus step (plan-pcb-routing Step 1a).

  python3 tests/test_622_plan_check_bus_step.py

R19 asks route_bus.bus_plan, at the plan's route.py sizes, for the board's buses. On tracked boards, with --board:

1. the bus boards' orangecrab, its planes poured and no bus copper yet (its ECP5 to its DDR3: a bus), and a plan with
   no awx/route_bus.py step: R19 refuses, naming the bus -- and only R19 (the plan is otherwise compliant);
2. the same plan with the bus step (no --bus: the step routes what it finds): R19 holds;
3. ...and with a --bus naming every bus found: R19 holds; with the step's --find form alone: R19 refuses;
4. qfn_interior_pads (a QFN, no ball array, no bus) and the plan with no bus step: R19 holds;
5. orangecrab_ext_pll, its bus already routed, and the plan with no bus step: R19 holds (the step would strip that
   copper and lay the bus again).
"""
import gzip
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'scripts', 'route_plan_check.py')
FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', 'bus_boards', 'orangecrab.kicad_pcb.gz')
ROUTED_BUS_BOARD = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
NO_BUS_BOARD = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')

PLAN = """#!/bin/bash
set -e
cd /repo
# cwd=/repo
python3 -u -X utf8 py_router/route_planes.py b.kicad_pcb s1.kicad_pcb --nets GND --plane-layers In1.Cu
{bus}# cwd=/repo
python3 -u -X utf8 py_router/route.py {last} final.kicad_pcb --nets '*' --power-nets GND --clearance-ceiling 0.09 \\
--track-width 0.0762 --via-size 0.25
# Verification -- COMMENTS, never executable lines:
#   python3 -X utf8 py_router/check_drc.py final.kicad_pcb
#   python3 -X utf8 py_router/check_connected.py final.kicad_pcb
#   python3 -X utf8 py_tools/check_orphan_stubs.py final.kicad_pcb
"""
STEP = ("# cwd=/repo\npython3 -u -X utf8 awx/route_bus.py s1.kicad_pcb s1a.kicad_pcb --rounds 1 --inproc "
        "--clearance-ceiling 0.09 --track-width 0.0762 --via-size 0.25{extra}\n")

fails = []


def check(ok, what):
    print(('PASS: ' if ok else 'FAIL: ') + what)
    if not ok:
        fails.append(what)


def run(tmp, name, board, bus=''):
    plan = PLAN.format(bus=bus, last='s1a.kicad_pcb' if 'route_bus' in bus and '--find' not in bus else 's1.kicad_pcb')
    path = os.path.join(tmp, name)
    with open(path, 'w') as f:
        f.write(plan)
    r = subprocess.run([sys.executable, '-X', 'utf8', CHECKER, path, '--board', board], capture_output=True, text=True,
                       timeout=600)
    failed = {ln.split()[1] for ln in r.stdout.splitlines() if ln.strip().startswith('FAIL')}
    return r, failed


def main():
    for b in (FIXTURE, NO_BUS_BOARD, ROUTED_BUS_BOARD):
        if not os.path.isfile(b):
            raise SystemExit(f'BROKEN TEST: no board {b}')
    with tempfile.TemporaryDirectory() as tmp:
        BUS_BOARD = os.path.join(tmp, 'orangecrab.kicad_pcb')
        with gzip.open(FIXTURE, 'rb') as fi, open(BUS_BOARD, 'wb') as fo:
            shutil.copyfileobj(fi, fo)
        shutil.copy(FIXTURE[:-len('.kicad_pcb.gz')] + '.kicad_pro', BUS_BOARD[:-len('.kicad_pcb')] + '.kicad_pro')
        r, failed = run(tmp, 'none.sh', BUS_BOARD)
        check(failed == {'R19'} and 'U3:U4' in r.stdout,
              f'1: no bus step on a board with a bus -- R19 alone refuses, naming U3:U4: {sorted(failed)}')
        if 'R19' not in failed:
            print(r.stdout[-1500:])
        r, failed = run(tmp, 'step.sh', BUS_BOARD, STEP.format(extra=''))
        check('R19' not in failed, f'2: the bus step, no --bus -- R19 holds: {sorted(failed)}')
        r, failed = run(tmp, 'named.sh', BUS_BOARD, STEP.format(extra=' --bus U3:U4'))
        check('R19' not in failed, f'3: the bus step naming U3:U4 -- R19 holds: {sorted(failed)}')
        find = '# cwd=/repo\npython3 -u -X utf8 awx/route_bus.py s1.kicad_pcb --find\n'
        r, failed = run(tmp, 'find.sh', BUS_BOARD, find)
        check('R19' in failed, f'3: the --find form alone is no bus step -- R19 refuses: {sorted(failed)}')
        r, failed = run(tmp, 'nobus.sh', NO_BUS_BOARD)
        check('R19' not in failed, f'4: a board with no bus, no bus step -- R19 holds: {sorted(failed)}')
        r, failed = run(tmp, 'routed.sh', ROUTED_BUS_BOARD)
        check('R19' not in failed, f'5: a bus the board carries routed, no bus step -- R19 holds: {sorted(failed)}')
    print('plan check bus step: ' + ('FAIL' if fails else 'PASS'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
