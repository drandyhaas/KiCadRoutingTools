#!/usr/bin/env python3
"""#1190: a routed output carries its input's whole sibling set.

Every routing writer (route.py, route_diff, route_planes, repair_planes,
bga_fanout, place_fanout_clearance) ends in ``fix_project_for_output``. It
copied the ``.kicad_pro`` and the ``.kicad_dru`` from a hand-written pair, so
the ``.design-brief.json`` (#711) and the ``.kicad_prl`` never travelled: the
next ``check_floorplan`` read "design brief: none beside this board" and
inferred every edge from the current pose. It now carries every
``copy_board.SIBLING_EXTS`` member it does not rewrite.

Checks:
  1. route.py and route_planes.py outputs of esp_prog, staged with the
     tracked #711 brief plus a .kicad_dru and a .kicad_prl, carry the
     input's whole sibling set, the brief byte-identical.
  2. check_floorplan finds the brief beside the routed output.
  3. fix_project_for_output never overwrites a sibling the output already
     has.

    python3 tests/test_1190_routed_output_siblings.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import SIBLING_EXTS, copy_board                 # noqa: E402
from fix_kicad_drc_settings import fix_project_for_output       # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
BRIEF = os.path.join(TESTS_DIR, 'fixtures', '711', 'esp_prog.design-brief.json')
DRU = '(version 1)\n(rule "inner" (layer inner) (constraint clearance (min 0.15mm)))\n'
PRL = '{"meta": {"filename": "in.kicad_prl", "version": 3}}\n'
PRO = ('{"board": {"design_settings": {"rules": {}, "rule_severities": {}}}, '
       '"meta": {"filename": "in.kicad_pro", "version": 1}, '
       '"net_settings": {"classes": [], "meta": {"version": 0}}}\n')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def siblings(base):
    return sorted(e for e in SIBLING_EXTS if os.path.isfile(base + e))


def run(argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + argv, cwd=ROOT,
                       capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def main():
    tmp = tempfile.mkdtemp(prefix='t1190_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(BOARD, inp)
        base = inp[:-len('.kicad_pcb')]
        shutil.copyfile(BRIEF, base + '.design-brief.json')
        with open(base + '.kicad_dru', 'w') as f:
            f.write(DRU)
        with open(base + '.kicad_prl', 'w') as f:
            f.write(PRL)
        with open(base + '.kicad_pro', 'w') as f:     # esp_prog ships none
            f.write(PRO)
        want = siblings(base)
        check('precondition: the staged input has the whole sibling set',
              want == sorted(SIBLING_EXTS), str(want))

        # 1. Both routing writers carry the set.
        steps = [
            ('route.py', ['py_router/route.py', inp,
                          os.path.join(tmp, 'routed.kicad_pcb'), '--nets', '/EN']),
            ('route_planes.py', ['py_router/route_planes.py', inp,
                                 os.path.join(tmp, 'pour.kicad_pcb'),
                                 '--nets', 'GND', '--plane-layers', 'B.Cu']),
        ]
        for name, argv in steps:
            rc, out = run(argv)
            outb = argv[2][:-len('.kicad_pcb')]
            check(f'{name} exits 0', rc == 0, out[-400:] if rc else '')
            got = siblings(outb)
            check(f'{name} output carries the input sibling set', got == want,
                  f'{got} vs {want}')
            if os.path.isfile(outb + '.design-brief.json'):
                with open(BRIEF, 'rb') as a, \
                        open(outb + '.design-brief.json', 'rb') as b:
                    check(f'{name} output brief is the input brief',
                          a.read() == b.read())

        # 2. The next grade finds the brief.
        rc, out = run(['py_tools/check_floorplan.py',
                       os.path.join(tmp, 'routed.kicad_pcb'),
                       '--emit-intent', os.path.join(tmp, 'intent.json')])
        ok = (rc == 0 and 'design brief routed.design-brief.json' in out
              and 'design brief: none beside' not in out)
        check('check_floorplan reads a brief beside the routed output', ok,
              '' if ok else out[-400:])

        # 3. An existing output sibling is never overwritten.
        out2 = os.path.join(tmp, 'kept.kicad_pcb')
        shutil.copyfile(inp, out2)
        mine = '{"version": 1, "mine": true}\n'
        with open(out2[:-len('.kicad_pcb')] + '.design-brief.json', 'w') as f:
            f.write(mine)
        with contextlib.redirect_stdout(io.StringIO()):
            fix_project_for_output(out2, inp, verbose=False)
        with open(out2[:-len('.kicad_pcb')] + '.design-brief.json') as f:
            check('an existing output brief is kept', f.read() == mine)
        check('the rest of the set is carried beside it',
              siblings(out2[:-len('.kicad_pcb')]) == want)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
