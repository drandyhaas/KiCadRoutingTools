#!/usr/bin/env python3
"""#1709: R15 prices the GND-via floor at the clearance the step routes at.

On the user's own rules a plan passes no `--clearance` / `--clearance-ceiling`,
and route_planes then routes its GND return vias at the board's Default net
class (`board_default_netclass_param(input, 'clearance')`, and the same for
`via_diameter`), falling back to routing_defaults only when the board declares
none. R15 skipped the board and priced the floor at routing_defaults' 0.25 /
0.5, so it refused a correct `--gnd-via-distance 2` on a board whose class is
0.15 / 0.45 (floor 1.80) -- and on a board whose class is wider than 0.25 it
would pass a distance that is too short.

Checks, each through the CLI with `--board`:
  1. the reported plan (via 0.45 on the step, no clearance): not refused; at
     1.7 it is refused at 3x(0.45 + 0.15 [board Default class]) = 1.800;
  2. no `--via-size` either: the via is the board's 0.45, not 0.5;
  3. a board whose Default class is WIDER than 0.25 refuses a distance the
     default would pass;
  4. this step's `--clearance-ceiling` caps the class, as route_planes does
     (`min(class, ceiling)`): below the class it lowers the floor, above it
     it does not raise it;
  5. without `--board`, a floor resting on routing_defaults is NOTED and not
     refused, and the note says to pass --board.

    python3 tests/test_1709_gnd_via_floor_board_class.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import evidence  # noqa: E402

CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                       'scripts', 'route_plan_check.py')
BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')

GV = '--nets GND GND --plane-layers F.Cu B.Cu'
RT = '--power-nets GND --power-nets-widths 0.3 --layers F.Cu B.Cu --layer-costs 1.0 1.0'
OWN = '--escalation board --no-fix-drc-settings'

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def plan(d, gnd_step_flags):
    """The reported shape: pour, route, a GND-via pass, the closing route."""
    return (f'set -e\ncd {ROOT}\n'
            f'python3 -X utf8 py_router/copy_board.py {BOARD} {d}/s0.kicad_pcb\n'
            f'python3 -X utf8 py_router/route_planes.py {d}/s0.kicad_pcb {d}/s1.kicad_pcb {GV} {OWN}\n'
            f"python3 -X utf8 py_router/route.py {d}/s1.kicad_pcb {d}/s2.kicad_pcb --nets '*' {RT} {OWN}\n"
            f'python3 -X utf8 py_router/route_planes.py {d}/s2.kicad_pcb {d}/s3.kicad_pcb {GV} '
            f'--add-gnd-vias {gnd_step_flags} {OWN}\n'
            f'python3 -X utf8 py_router/route.py {d}/s3.kicad_pcb {d}/final.kicad_pcb --nets GND {RT} {OWN}\n'
            f'# python3 -X utf8 py_router/check_drc.py {d}/final.kicad_pcb\n'
            f'# python3 -X utf8 py_router/check_connected.py {d}/final.kicad_pcb\n'
            f'# python3 -X utf8 py_tools/check_orphan_stubs.py {d}/final.kicad_pcb\n')


def r15(d, name, flags, board=BOARD):
    """(exit code, R15 status word, the lines under it) for one plan."""
    path = os.path.join(d, name)
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(plan(d, flags))
    evidence(path, name)
    argv = [sys.executable, '-X', 'utf8', CHECKER, path] + (['--board', board] if board else [])
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=120)
    assert 'Traceback' not in r.stdout + r.stderr, r.stdout + r.stderr
    lines = r.stdout.splitlines()
    at = next(i for i, ln in enumerate(lines) if ln.split()[1:2] == ['R15'])
    under = []
    for ln in lines[at + 1:]:
        if not ln.startswith('          '):
            break
        under.append(ln.strip())
    return r.returncode, lines[at].split()[0], under


def wide_board(td):
    """The fixture with its Default class widened to 0.3 clearance."""
    d = os.path.join(td, 'wide')
    os.makedirs(d)
    dst = os.path.join(d, 'wide.kicad_pcb')
    shutil.copyfile(BOARD, dst)
    with open(BOARD[:-len('.kicad_pcb')] + '.kicad_pro', encoding='utf-8') as fh:
        pro = json.load(fh)
    for c in pro['net_settings']['classes']:
        if c['name'] == 'Default':
            c['clearance'] = 0.3
    with open(dst[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(pro, fh)
    return dst


def main():
    with tempfile.TemporaryDirectory() as td:
        print("1. the reported plan, on the board's own rules")
        rc, st, why = r15(td, 'a.sh', '--gnd-via-distance 2 --via-size 0.45')
        ok('--gnd-via-distance 2 clears 3x(0.45 + 0.15) = 1.80: not refused',
           rc == 0 and st == 'ok', f'rc={rc} {st} {why}')
        rc, st, why = r15(td, 'b.sh', '--gnd-via-distance 1.7 --via-size 0.45')
        ok('1.7 is refused, at the board Default class',
           rc == 4 and st == 'FAIL' and len(why) == 1
           and 'clearance 0.15 [board Default class]' in why[0] and '= 1.800' in why[0],
           f'rc={rc} {st} {why}')

        print("2. no --via-size: the board's via")
        rc, st, why = r15(td, 'c.sh', '--gnd-via-distance 1.7')
        ok('the via is the board Default class 0.45, not routing_defaults 0.5',
           st == 'FAIL' and len(why) == 1 and 'via 0.45 [board Default class]' in why[0]
           and '= 1.800' in why[0], f'{st} {why}')

        print('3. a Default class wider than the default')
        wide = wide_board(td)
        rc, st, why = r15(td, 'd.sh', '--gnd-via-distance 2.2 --via-size 0.45', board=wide)
        ok('2.2 is under 3x(0.45 + 0.3) = 2.25: refused (the default 0.25 floor, '
           '2.10, passed it)',
           rc == 4 and st == 'FAIL' and len(why) == 1
           and 'clearance 0.3 [board Default class]' in why[0] and '= 2.250' in why[0],
           f'rc={rc} {st} {why}')

        print("4. this step's --clearance-ceiling caps the class")
        rc, st, why = r15(td, 'e.sh', '--gnd-via-distance 1.6 --via-size 0.45 '
                                      '--clearance-ceiling 0.1')
        ok('a ceiling under the class lowers the floor: 1.6 refused at 1.650',
           st == 'FAIL' and len(why) == 1 and 'clearance 0.1 [this step]' in why[0]
           and '= 1.650' in why[0], f'{st} {why}')
        rc, st, why = r15(td, 'f.sh', '--gnd-via-distance 1.9 --via-size 0.45 '
                                      '--clearance-ceiling 0.2')
        ok('a ceiling above the class does not raise it: 1.9 clears 1.80',
           rc == 0 and st == 'ok', f'rc={rc} {st} {why}')

        print('5. without --board: noted, not refused')
        rc, st, why = r15(td, 'g.sh', '--gnd-via-distance 2 --via-size 0.45', board=None)
        ok('a floor resting on routing_defaults is a NOTE, exit 0',
           rc == 0 and st == 'NOTE' and len(why) == 1 and why[0].startswith('note:')
           and '[routing_defaults]' in why[0] and 'pass --board' in why[0],
           f'rc={rc} {st} {why}')
        rc, st, why = r15(td, 'h.sh', '--gnd-via-distance 1.5 --via-size 0.45 '
                                      '--clearance 0.09', board=None)
        ok('...while a floor the plan states is still refused without a board',
           rc == 4 and st == 'FAIL' and '= 1.620' in why[0], f'rc={rc} {st} {why}')
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
