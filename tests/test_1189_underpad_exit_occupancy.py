#!/usr/bin/env python3
"""#1189: the under-pad A*'s exit step passes the occupancy test too.

The neighbour loop tested a cell's occupancy only inside the array window
(`if inwin and _gL[nidx]`), so the exit cell -- the first lattice node
outside the window -- was accepted even inside a foreign via's keep-out,
although the window margin is stamped precisely so that copper there is
seen. glasgow_revC's U30.E8 re-fan shipped a /IO_Banks/U5 escape 0.244 mm
from a /IO_Banks/DB1 via (0.2594 required), exit 0, counted only in JSON
drc_grazes.

Reproduced on the tracked board: U30.E8 alone, under-pad, with a foreign
via added at the spot that makes its natural exit node graze.

Checks:
  1. Precondition: without the via the escape is unchanged by the fix -- it
     ends at (84.2, 100.0) past the array's bottom edge -- and its exit
     column's node (84.175, 99.925) sits inside the added via's keep-out.
  2. With the via, the ball still escapes, every escape endpoint clears the
     via by via r + half-width + clearance, and check_drc finds no
     via-segment violation.

    python3 tests/test_1189_underpad_exit_occupancy.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
NET = '/IO_Banks/U5'                     # U30.E8
VIA = (84.0, 100.1)                      # foreign: /IO_Banks/DB1, 0.25/0.15
VIA_R, HALF_W, CLR = 0.125, 0.0889 / 2, 0.0889
ARGS = ['--component', 'U30', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
        '--layer-costs', '1.0', '4.0', '2.5', '1.0', '--track-width', '0.0762',
        '--clearance-ceiling', '0.0889', '--clearance', '0.0889',
        '--via-size', '0.25', '--via-drill', '0.15', '--nets', NET,
        '--escape-method', 'underpad']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def fan(inp, out):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py',
                        inp, '-o', out] + ARGS, cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
    line = next((l for l in r.stdout.splitlines() if 'JSON_SUMMARY' in l), '')
    js = json.loads(line.split('JSON_SUMMARY:', 1)[1]) if line else {}
    before = {(s.start_x, s.start_y, s.end_x, s.end_y) for s in parse(inp).segments}
    after = parse(out)
    nid = next(i for i, n in after.nets.items() if n.name == NET)
    esc = [s for s in after.segments if s.net_id == nid
           and (s.start_x, s.start_y, s.end_x, s.end_y) not in before]
    return r.returncode, js, esc


def main():
    need = VIA_R + HALF_W + CLR
    tmp = tempfile.mkdtemp(prefix='t1189_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(BOARD, inp)

        # 1. The via-free escape and the precondition.
        rc, js, esc = fan(inp, os.path.join(tmp, 'plain.kicad_pcb'))
        ends = {(round(s.end_x, 3), round(s.end_y, 3)) for s in esc}
        check('without the via the ball escapes past the bottom edge',
              rc == 0 and js.get('escaped') == 1 and (84.2, 100.0) in ends,
              f'rc={rc} escaped={js.get("escaped")} ends={sorted(ends)}')
        d_exit = math.hypot(84.175 - VIA[0], 99.925 - VIA[1])
        check('precondition: the exit column node is inside the via keep-out',
              d_exit < need, f'{d_exit:.4f} < {need:.4f}')

        # 2. With the foreign via.
        text = open(inp, encoding='utf-8').read()
        via = (f'\t(via\n\t\t(at {VIA[0]} {VIA[1]})\n\t\t(size 0.25)\n'
               f'\t\t(drill 0.15)\n\t\t(layers "F.Cu" "B.Cu")\n'
               f'\t\t(net "/IO_Banks/DB1")\n'
               f'\t\t(uuid "11891189-0000-4000-8000-000000001189")\n\t)\n')
        i = text.rstrip().rfind(')')
        vin = os.path.join(tmp, 'via.kicad_pcb')
        open(vin, 'w', encoding='utf-8').write(text[:i] + via + text[i:])
        shutil.copyfile(inp[:-len('.kicad_pcb')] + '.kicad_pro',
                        vin[:-len('.kicad_pcb')] + '.kicad_pro')
        vout = os.path.join(tmp, 'via_out.kicad_pcb')
        rc, js, esc = fan(vin, vout)
        check('with the via the ball still escapes',
              rc == 0 and js.get('escaped') == 1 and esc,
              f'rc={rc} escaped={js.get("escaped")}')
        worst = min((math.hypot(x - VIA[0], y - VIA[1])
                     for s in esc for x, y in ((s.start_x, s.start_y),
                                               (s.end_x, s.end_y))),
                    default=math.inf)
        check('every escape endpoint clears the foreign via',
              worst >= need - 1e-6, f'{worst:.4f} >= {need:.4f}')
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py',
                            vout, '--clearance-margin', '0.1'], cwd=ROOT,
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace')
        vs = [l.strip() for l in r.stdout.splitlines()
              if 'Via:/IO_Banks/DB1' in l and f'Seg:{NET}' in l]
        check('check_drc finds no via-segment violation on the escape',
              r.returncode in (0, 1) and not vs, '; '.join(vs[:2]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
