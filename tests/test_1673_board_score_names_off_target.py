#!/usr/bin/env python3
"""#1673: board_score's disclosure names a declared net off its ohms, with no flag.

The routing skill says that after `--impedance-declare-only`, check_impedance
"(and board_score's disclosure)" names a pair more than 10% off. board_score's
console said nothing, and its JSON key appeared only under --impedance-nets. Now:

  * no flag: JSON `declared_ohms_off_target`, a console line naming the nets and
    how far off they are;
  * --impedance-nets naming a net: it moves to `nets_ohms_off_target`, is not also
    listed as undeclared, and the line says nothing about grading it (the flag
    grades continuity and gap, never the ohms);
  * never counted: `count` and `blocking` stay as they were, and check_complete
    still says DONE. That is the decision the disclosure was made under (#1630).

The fixture is test_1630's: one pair over an intact GND plane, its class at
94 ohm differential, so a 75 ohm declaration is off target with no crossing.
Control: a declaration the copper meets names nothing.

    python3 tests/test_1673_board_score_names_off_target.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check as run_check, evidence             # noqa: E402
from test_1630_pair_class_one_gap import build                 # noqa: E402

PY = [sys.executable, '-X', 'utf8']
LINE = 'DECLARED IMPEDANCE OFF TARGET'
FAILS = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        FAILS.append(label)


def declared(tmp, tag, ohms):
    board = build(os.path.join(tmp, tag), False)
    out = os.path.join(tmp, tag, 'o.kicad_pcb')
    run_check(PY + [os.path.join(ROOT, 'py_router', 'route_diff.py'), board, out,
                    '--nets', '/X_*', '--layers', 'F.Cu', 'B.Cu',
                    '--impedance', str(ohms), '--impedance-declare-only'], accept=True)
    return evidence(out, f'{tag} output')


def score(board, *flags):
    js = board + '.score.json'
    r = run_check(PY + [os.path.join(ROOT, 'py_tools', 'board_score.py'), board,
                        '--json', js, *flags], accept=True)
    with open(evidence(js, 'the score JSON'), encoding='utf-8') as f:
        doc = json.load(f)
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith(LINE)]
    return doc, lines


def main():
    tmp = tempfile.mkdtemp(prefix='t1673_')
    try:
        off = declared(tmp, 'off', 75)

        doc, lines = score(off)
        imp = doc['components']['impedance']
        check('no flag: the JSON carries both nets',
              imp.get('declared_ohms_off_target') == ['/X_N', '/X_P'], str(imp))
        check('no flag: and how far off they are',
              all(abs((imp.get('ohms_off_pct') or {}).get(n, 0) - 25.7) < 2.5
                  for n in ('/X_N', '/X_P')), str(imp.get('ohms_off_pct')))
        check('no flag: the console line names them with the percentage',
              len(lines) == 1 and '/X_N +' in lines[0] and '/X_P +' in lines[0]
              and '2 net(s)' in lines[0], str(lines))
        check('no flag: still ungraded', imp.get('ran') is False and imp.get('count') is None,
              str(imp))
        check('no flag: blocking stays 0', doc.get('blocking') == 0, str(doc.get('blocking')))
        check('no flag: the failing list stays what it is', imp.get('declared_failing') == [],
              str(imp.get('declared_failing')))

        doc, lines = score(off, '--impedance-nets', '/X_*')
        imp = doc['components']['impedance']
        check('flag: a named net moves to the graded list',
              imp.get('nets_ohms_off_target') == ['/X_N', '/X_P']
              and imp.get('declared_ohms_off_target') == [], str(imp))
        check('flag: the ohms are never counted', imp.get('count') == 0 and doc.get('blocking') == 0,
              str((imp.get('count'), doc.get('blocking'))))
        check('flag: the line still names them, and does not call them ungraded',
              len(lines) == 1 and '/X_P' in lines[0] and 'ungraded' not in lines[0]
              and 'not graded' not in lines[0] and '--impedance-nets' not in lines[0],
              str(lines))

        doc, lines = score(off, '--impedance-nets', '/X_P')
        imp = doc['components']['impedance']
        check('one named: each net on its own list, none on both',
              imp.get('nets_ohms_off_target') == ['/X_P']
              and imp.get('declared_ohms_off_target') == ['/X_N'], str(imp))
        check('one named: the line counts each net once',
              len(lines) == 1 and '2 net(s)' in lines[0], str(lines))

        cc = off + '.cc.json'
        run_check(PY + [os.path.join(ROOT, 'check_complete.py'), off, '--json', cc], accept=True)
        with open(evidence(cc, 'the check_complete JSON'), encoding='utf-8') as f:
            verdict = json.load(f)['verdict']
        check('check_complete: the verdict does not read it', verdict == 'DONE', verdict)

        met = declared(tmp, 'met', 94)
        doc, lines = score(met)
        imp = doc['components']['impedance']
        check('control: a declaration the copper meets names nothing',
              imp.get('declared_ohms_off_target') == [] and not lines, f'{imp} {lines}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
