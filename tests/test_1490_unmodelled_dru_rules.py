"""#1490: a .kicad_dru rule no grader models is NAMED, never passed as graded.

check_drc reads .kicad_dru clearance through three channels: layer-scoped
rules (#498), class track-to-track rules (#735) and coupled-pair rules
(#1293), and since #1491 class/type-scoped rules (`A.NetClass == 'HS' &&
B.NetClass == 'HS'`, `B.Type == 'Pad'`; since #1528 also `A.NetName == ...`).
A clearance rule scoped any other way (`A.insideCourtyard(...)`) got one
console note and was graded by nothing, so board_score said drc 0 and
check_complete said DONE while KiCad reported the errors. It is now named `NOT GRADED (clearance)`, the
clearance half of #1390's size disclosure: board_score lists `drc.dru_rules`
ungraded and check_complete reads INCOMPLETE. The router names an
`annular_width` rule it does not honour (no routing step sizes a via by it)
in `design_rules.unsupported_rules`.

    python3 tests/test_1490_unmodelled_dru_rules.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join('kicad_files', 'flat_hierarchy.kicad_pcb')
DRU = '''(version 1)
(rule "hs gap" (constraint clearance (min 0.3mm))
  (condition "A.NetClass == 'HS' && B.NetClass == 'HS'"))
(rule "hs pad" (constraint clearance (min 0.3mm))
  (condition "A.NetClass == 'HS' && B.Type == 'Pad'"))
(rule "name gap" (constraint clearance (min 0.3mm))
  (condition "A.NetName == '/HS0' && B.NetName == '/HS1'"))
(rule "court gap" (constraint clearance (min 0.3mm))
  (condition "A.insideCourtyard('P3') && A.NetClass == 'HS'"))
(rule "inner tight" (constraint clearance (min 0.15mm)) (layer inner))
(rule "pair gap" (constraint clearance (min 0.12mm))
  (condition "A.NetClass == 'HS' && AB.isCoupledDiffPair()"))
(rule "via ring" (constraint annular_width (min 0.2mm))
  (condition "A.Type == 'Via'"))
'''
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO)


def main():
    td = tempfile.mkdtemp(prefix='krt1490_')
    try:
        b = os.path.join(td, 'b.kicad_pcb')
        run([os.path.join('py_router', 'copy_board.py'), BOARD, b])
        run_utils.evidence(b)
        with open(os.path.join(td, 'b.kicad_dru'), 'w') as fh:
            fh.write(DRU)

        print('1. check_drc names the clearance rules it does not model')
        r = run([os.path.join('py_router', 'check_drc.py'), b])
        named = sorted(set(ln.split("'")[1] for ln in r.stdout.splitlines()
                           if 'NOT GRADED (clearance)' in ln))
        check("the courtyard-scoped rule is named", named == ['court gap'],
              f'{named}\n{r.stdout[-600:]}')
        check('the NetClass-, Type- (#1491) and NetName-scoped (#1528) rules are '
              'graded, so not named', not {'hs gap', 'hs pad', 'name gap'} & set(named))
        check('the layer rule is graded, so not named',
              "NOT GRADED (clearance): rule 'inner tight'" not in r.stdout)
        check('the coupled-pair rule is graded (#1293), so not named',
              "NOT GRADED (clearance): rule 'pair gap'" not in r.stdout)

        print('2. board_score and check_complete follow')
        r = run([os.path.join('py_tools', 'board_score.py'), b, '--quiet'])
        line = next((x for x in r.stdout.splitlines() if x.startswith('SCORE_JSON=')), '')
        score = json.loads(line[len('SCORE_JSON='):]) if line else {}
        drc = (score.get('components') or {}).get('drc') or {}
        check('board_score: drc.ungraded_rules names them',
              drc.get('ungraded_rules') == ['court gap'], json.dumps(drc)[:400])
        check("board_score: 'drc.dru_rules' is ungraded",
              'drc.dru_rules' in (score.get('ungraded') or []), json.dumps(score.get('ungraded')))
        r = run(['check_complete.py', b])
        verdict = next((x for x in r.stdout.splitlines() if x.startswith('VERDICT')), '')
        check("check_complete: not DONE, naming them",
              not verdict.startswith('VERDICT: DONE')
              and "disallow rule(s) nothing grades: 'court gap'" in verdict, verdict[:300])

        print("3. the router names the annular_width rule it does not honour")
        from kicad_parser import parse_kicad_pcb
        from design_rules import DesignRules
        names = [n for n, _w in DesignRules.from_project(parse_kicad_pcb(b), b).unsupported()]
        check("design_rules.unsupported() lists 'via ring'", 'via ring' in names, f'{names}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
