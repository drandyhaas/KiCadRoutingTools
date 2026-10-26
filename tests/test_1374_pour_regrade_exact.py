#!/usr/bin/env python3
"""#1374: the final re-grade reads a pour net the fill MODEL calls open on
KiCad's exact fill, and the repair never says "confirmed connected by the
oracle" about a net KiCad still calls split.

Two derived 20 mm two-pad GND pours (tests/fixtures/island_semantics):

  pinch  a 0.12 mm corridor that KiCad pours (min_thickness 0.1) and the
         fill model drops. The plane finalize's oracle ran and cleared the
         net, so the final re-grade used to keep the model's grade: `failed
         1 (GND)` on a board KiCad, check_connected and board_score all call
         connected. It must refill once for that case and ship `failed 0`.
  split  a really split pour. The repair re-graded the model's failing pad
         against the oracle's links by centre distance, found no link
         endpoint at it, and printed "Pads still unconnected: none (all
         model-only; confirmed connected by the oracle)" while the oracle
         still reported the net split. The line must name the net.

Needs KiCad's python (the exact fill); exits 77 without it.

    python3 tests/test_1374_pour_regrade_exact.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

FIX = os.path.join(ROOT, 'tests', 'fixtures', 'island_semantics',
                   'two_islands_tht.kicad_pcb')
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def route(src, out, *extra):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', src, out,
                        '--nets', 'GND', '--layers', 'B.Cu'] + list(extra),
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    return r.stdout + r.stderr


def final_failed(log):
    m = re.findall(r'"failed": (\d+), "failed_single": (\[[^\]]*\])', log)
    return (int(m[-1][0]), json.loads(m[-1][1])) if m else (None, None)


def main():
    from kicad_exact_fill import find_kicad_python
    if not find_kicad_python():
        print('SKIP: no KiCad python for the exact fill')
        return 77
    text = open(FIX, encoding='utf-8').read()
    body = text.rstrip('\n')
    assert body.endswith(')')
    pinch = (body[:-1]
             .replace('(end 10 19.5)', '(end 10 9.29) (locked yes)')
             .replace('(min_thickness 0.2)', '(min_thickness 0.1)')
             + '(segment (start 10 10.71) (end 10 19.5) (locked yes) (width 0.3) '
               '(layer "B.Cu") (net "SIG2") '
               '(uuid "0c016a08-c2a8-4700-9ba0-ab33b5c0358c"))\n)\n')
    split = text.replace('(end 10 19.5)', '(end 10 19.5) (locked yes)')
    tmp = tempfile.mkdtemp(prefix='t1374_')
    for name, src in (('pinch', pinch), ('split', split)):
        open(os.path.join(tmp, name + '.kicad_pcb'), 'w', encoding='utf-8').write(src)

    log = route(os.path.join(tmp, 'pinch.kicad_pcb'), os.path.join(tmp, 'pinch_out.kicad_pcb'),
                '--clearance', '0.45', '--escalation', 'off')
    check('pinch: the gate oracle called the net complete',
          'KiCad reports 0 unconnected link(s)' in log)
    failed, names = final_failed(log)
    check('pinch: the final re-grade ships failed 0 (was: failed 1, GND)',
          failed == 0, f'failed={failed} {names}')
    check('pinch: the re-grade read the pour on KiCad\'s refill',
          '"graded_on": "kicad_refill"' in log)

    log = route(os.path.join(tmp, 'split.kicad_pcb'), os.path.join(tmp, 'split_out.kicad_pcb'))
    check('split: the oracle still reports the pour split',
          re.search(r'gate oracle: KiCad reports [1-9]\d* unconnected link', log) is not None)
    check('split: the repair does not claim "confirmed connected by the oracle"',
          'confirmed connected by the oracle' not in log)
    check('split: the repair names the net KiCad still calls split',
          "exact fill still reports the net(s) split: GND" in log)
    failed, names = final_failed(log)
    check('split: the run still reports GND failed (correct)', failed == 1 and names == ['GND'],
          f'failed={failed} {names}')

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
