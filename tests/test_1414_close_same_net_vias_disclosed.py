#!/usr/bin/env python3
"""#1414: a leg whose guarded re-route (#230) cannot connect ships its flawed
path with two own vias under the drill hole-to-hole floor -- and says so.

A foreign F.Cu track spans the whole board between the two pads of SIG, so
SIG must dip to B.Cu and back: two vias straddling the wall, a few mm
apart. With a floor wider than the board no spaced path exists, the guarded
retry fails, and the unspaced path is kept (completion first). Before, the
only trace of that was check_drc's VIA-DRILL-HOLE on the written board.

Rows:
  1. `close_same_net_via_pairs`: names a pair under the floor when one via is
     new, ignores an input-only pair, a pair at the floor, and other nets.
  2. route.py on the wall board ships the pair, prints the WARNING, and
     JSON_SUMMARY carries `close_same_net_vias` naming SIG and the gap.
  3. The named pair is the one check_drc grades as VIA-DRILL-HOLE.

    python3 tests/test_1414_close_same_net_vias_disclosed.py
"""
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from synth import RoutingBoard                                  # noqa: E402
from check_weird import close_same_net_via_pairs                # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def v(x, y, drill=0.3):
    return SimpleNamespace(x=x, y=y, drill=drill, size=0.6)


def run(*argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + list(argv), cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    return r.stdout + r.stderr


def main():
    # 1. The census.
    a, b, c = v(0, 0), v(1.0, 0), v(2.0, 0)
    name = lambda n: f'N{n}'
    got = close_same_net_via_pairs({1: [a, b], 2: [c]}, lambda x: x is b, 1.0, name)
    check('1. a new via 0.4 mm drill gap from an old one is named, gap 0.7',
          len(got) == 1 and got[0]['net'] == 'N1' and got[0]['drill_gap_mm'] == 0.7,
          str(got))
    check('1. an input-only pair is not named',
          not close_same_net_via_pairs({1: [a, b]}, lambda x: False, 1.0, name))
    check('1. a pair exactly at the floor is not named',
          not close_same_net_via_pairs({1: [a, v(1.3, 0)]}, lambda x: True, 1.0, name))
    check('1. vias of different nets are not paired',
          not close_same_net_via_pairs({1: [a], 2: [b]}, lambda x: True, 1.0, name))

    # 2. The wall board.
    bd = RoutingBoard(w=40, h=30)
    bd.pad('A', 10, 15, 'SIG')
    bd.pad('B', 30, 15, 'SIG')
    bd.seg(20, 0.5, 20, 29.5, 'WALL', w=1.0)
    tmp = tempfile.mkdtemp(prefix='t1414_')
    pcb = os.path.join(tmp, 'wall.kicad_pcb')
    out = os.path.join(tmp, 'out.kicad_pcb')
    js = os.path.join(tmp, 'out.json')
    open(pcb, 'w', encoding='utf-8').write(bd.text())
    log = run('py_router/route.py', pcb, out, '--nets', 'SIG', '--layers', 'F.Cu', 'B.Cu',
              '--track-width', '0.2', '--clearance', '0.2', '--via-size', '0.6',
              '--via-drill', '0.3', '--hole-to-hole-clearance', '40.0',
              '--max-ripup', '0', '--json-out', js)
    summ = json.load(open(js)) if os.path.isfile(js) else {}
    rows = summ.get('close_same_net_vias') or []
    check('2. the net routed (completion first)', summ.get('failed') == 0,
          f"failed={summ.get('failed')}")
    check('2. JSON_SUMMARY close_same_net_vias names SIG under the 40 mm floor',
          len(rows) == 1 and rows[0]['net'] == 'SIG' and rows[0]['floor_mm'] == 40.0
          and rows[0]['drill_gap_mm'] < 40.0, str(rows))
    check('2. the console WARNING names the pair', 'same-net via pair(s) closer than' in log)

    # 3. check_drc grades the same pair.
    drc = run('py_router/check_drc.py', out, '--hole-to-hole-clearance', '40.0',
              '--clearance-margin', '0.1', '--nets', 'SIG')
    check('3. check_drc reports VIA-DRILL-HOLE on SIG', 'VIA-DRILL-HOLE' in drc)

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
