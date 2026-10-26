#!/usr/bin/env python3
"""#1384: place_simple seats every declared `edge_connectors` row, the way
`check_floorplan` grades every one.

A row `{"ref": "C1", "edge": "west"}` with no `class` on a part that is not an
edge part by class was NOTEd ("placed as any other part") and left where the
anneal put it, then graded `[ERROR] edge_connector: C1 sits nearest the south
edge but is declared on the west edge`. The grade reads the edge of every row
whatever its class, so the seat now holds every declared row.

Checks, on the tracked esp_prog pile (about 4 s per placement):
  1. the classless row is seated: place_simple's own floorplan line reports no
     error and the NOTE says it was seated as declared;
  2. check_floorplan on the output, against the same intent, reports no
     `edge_connector` error for C1.

    python3 tests/test_1384_classless_edge_row.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
from run_utils import evidence, tool, tool_env          # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def run(argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + argv, capture_output=True,
                       text=True, cwd=ROOT, env=tool_env(), timeout=900)
    return r.returncode, (r.stdout or '') + (r.stderr or '')


def main():
    tmp = tempfile.mkdtemp(prefix='t1384_')
    try:
        pile = evidence(os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb'),
                        'tracked pile')
        designer = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), 'designer board')
        for src, name in ((pile, 'pile'), (designer, 'designer')):
            code, out = run([tool('copy_board.py'), src, os.path.join(tmp, name + '.kicad_pcb')])
            check(f'copy {name}', code == 0, out[-200:])
        code, out = run([tool('check_floorplan.py'), os.path.join(tmp, 'designer.kicad_pcb'),
                         '--emit-intent', os.path.join(tmp, 'intent.json')])
        intent = json.load(open(evidence(os.path.join(tmp, 'intent.json'), 'emitted intent')))
        check('the emitted intent has no edge rows to begin with', not intent.get('edge_connectors'))
        intent['edge_connectors'] = [{'ref': 'C1', 'edge': 'west'}]
        ipath = os.path.join(tmp, 'intent_classless.json')
        json.dump(intent, open(ipath, 'w'))
        groups = {'schema': 1, 'groups': [
            {'name': 'usb', 'role': 'interface', 'refs': ['USB1', 'U1', 'Y1', 'C2', 'C3', 'C4'], 'anchor': 'U1'},
            {'name': 'power', 'role': 'power', 'refs': ['U2', 'C1'], 'anchor': 'U2', 'near': ['usb']},
            {'name': 'prog', 'role': 'interface', 'refs': ['CON1', 'CON2', 'Q1', 'Q2', 'R1', 'R2', 'R3', 'R4'],
             'anchor': 'CON1', 'near': ['usb']},
            {'name': 'fiducials', 'role': 'mechanical', 'refs': ['Ref*', 'Ref*~2']}]}
        gpath = os.path.join(tmp, 'groups.json')
        json.dump(groups, open(gpath, 'w'))
        outp = os.path.join(tmp, 'out.kicad_pcb')
        code, out = run([tool('place_simple.py'), os.path.join(tmp, 'pile.kicad_pcb'), outp,
                         '--seed', '1', '--groups', gpath, '--intent', ipath])
        check('place_simple ran', os.path.isfile(outp), out[-400:])
        check('the seat says it seated the classless row',
              'C1 is declared on the west edge but is not an edge part by class' in out
              and 'seated there as declared' in out, out[-400:])
        check('place_simple reports no floorplan error',
              'floorplan: no error' in out
              and 'floorplan error(s)' not in out, out[-400:])
        code, out = run([tool('check_floorplan.py'), outp, '--intent', ipath])
        check('check_floorplan reports no edge_connector error for C1',
              '[ERROR] edge_connector' not in out, out[-600:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
