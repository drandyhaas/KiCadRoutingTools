"""#1485: the face pass ranks each run on its OWN limit counts.

`outcome()` applied a run's poses and read `claims_broken()` from the
model's cache, which still held the LAST restart's state -- not the kept
run's, whenever every restart left a part without a legal pose. So its NOTE
disagreed with the shipped summary, and the keep decision went wrong both
ways. Now the model is resynced first (`total()`), and the counts are the
run's own.

The case: both faces allowed, a front keep-out that allows every part but
the main chip (so every restart leaves it without a legal pose and the
first is kept), and tight crystal claims whose count varies run to run.
"""

import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
GROUPS = {'schema': 1, 'groups': [
    {'name': 'mcu', 'refs': ['U1', 'Y1', 'C2', 'C4', 'C3'], 'flippable': ['C2', 'C4', 'C3']},
    {'name': 'usb', 'refs': ['USB1', 'R1', 'R2', 'Q1', 'Q2'], 'flippable': ['R1', 'R2', 'Q1', 'Q2']},
    {'name': 'power', 'refs': ['U2', 'C1', 'CON1', 'CON2', 'R3', 'R4'],
     'flippable': ['C1', 'R3', 'R4']},
    {'name': 'marks', 'refs': ['Ref*', 'Ref*~2']}]}
NOTE = re.compile(r'with them flipped: (\d+) part\(s\) without a legal pose, (\d+) decap '
                  r'limit\(s\), (\d+) claim limit\(s\) broken, cost ([\d.]+), against (\d+), '
                  r'(\d+), (\d+), ([\d.]+); (kept|not kept)')
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def run(*args):
    return subprocess.run([sys.executable, '-B', '-X', 'utf8'] + list(args),
                          capture_output=True, text=True, cwd=ROOT)


def main():
    with tempfile.TemporaryDirectory() as td:
        fixed = os.path.join(td, 'fixed.kicad_pcb')
        run(os.path.join(ROOT, 'py_placer', 'place_pose.py'), BOARD, fixed,
            'lock', 'USB1', 'CON1', 'CON2')
        intent = os.path.join(td, 'intent.json')
        run(os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), fixed, '--emit-intent', intent)
        d = json.load(open(intent))
        d['assembly']['sides'] = 'both'
        d['keepouts'] = [{'name': 'no_main_chip_on_front', 'rect': [114.0, 91.0, 145.75, 105.5],
                          'sides': ['F'],
                          'allow': ['C*', 'R*', 'Q*', 'Y*', 'U2', 'CON*', 'USB*']}]
        for row in d['proximity']:
            row['max_mm'] = 1.5
        json.dump(d, open(intent, 'w'))
        groups = os.path.join(td, 'groups.json')
        json.dump(GROUPS, open(groups, 'w'))
        for seed, kept in ((10, True), (5, False)):
            r = run(os.path.join(ROOT, 'py_placer', 'place_simple.py'), fixed,
                    os.path.join(td, f's{seed}.kicad_pcb'), '--groups', groups,
                    '--intent', intent, '--seed', str(seed))
            m = NOTE.search(r.stdout)
            ok(f'seed {seed}: the face pass ran', m, r.stdout[-300:])
            if not m:
                continue
            ship = re.search(r'^place_simple: annealed .*?cost ([\d.]+) .*?(\d+) claim limit\(s\) '
                             r'broken', r.stdout, re.M)
            claims = int(m.group(3)) if m.group(9) == 'kept' else int(m.group(7))
            cost = m.group(4) if m.group(9) == 'kept' else m.group(8)
            ok(f'seed {seed}: the NOTE counts the shipped run as its summary does',
               ship and ship.group(1) == cost and int(ship.group(2)) == claims,
               (m.group(0), ship and ship.group(0)))
            ok(f'seed {seed}: {"kept" if kept else "not kept"} on the true counts',
               (m.group(9) == 'kept') == kept, m.group(0))
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
