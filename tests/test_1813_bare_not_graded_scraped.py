"""#1813: every `.kicad_dru` rule check_drc names NOT GRADED reaches the verdict.

check_drc's length and diff-pair passes named a rule whose condition they
cannot read with a BARE `NOT GRADED: rule '...'` line, while board_score read
only the parenthesised kinds it knew (`size`, `clearance`, `disallow`,
`via_count`). So a courtyard-scoped length or skew rule was graded by nothing,
listed nowhere, and check_complete said DONE while KiCad reported the errors.
Every NOT GRADED line now carries its kind, board_score reads EVERY kind into
`drc.ungraded_rules` (and `drc.ungraded_rule_kinds`), and check_complete
names the rule with its kind and reads INCOMPLETE.

No KiCad and no routing needed: the rule is named before anything is measured.

    python3 tests/test_1813_bare_not_graded_scraped.py
"""
import ast
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join('kicad_files', 'cap_chain.kicad_pcb')
COURT = "A.insideCourtyard('C1')"
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO,
                          encoding='utf-8', errors='replace')


def stage(td, sub, name, constraint, cond):
    d = os.path.join(td, sub)
    os.makedirs(d)
    b = os.path.join(d, 'b.kicad_pcb')
    run([os.path.join('py_router', 'copy_board.py'), BOARD, b])
    run_utils.evidence(b)
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(f'(version 1)\n(rule "{name}" (constraint {constraint})\n'
                 f'  (condition "{cond}"))\n')
    return b


def score(b):
    r = run([os.path.join('py_tools', 'board_score.py'), b, '--quiet'])
    line = next((x for x in r.stdout.splitlines() if x.startswith('SCORE_JSON=')), '')
    assert line, f'board_score printed no SCORE_JSON:\n{r.stdout[-800:]}{r.stderr[-800:]}'
    s = json.loads(line[len('SCORE_JSON='):])
    return s, (s.get('components') or {}).get('drc') or {}


def complete(b):
    return [os.path.join(REPO, 'check_complete.py'), b, '--skip-slow']


def not_graded_prints():
    """The literal text of every print() in check_drc that says NOT GRADED."""
    with open(os.path.join(REPO, 'py_router', 'check_drc.py'), encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    out = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'print'):
            continue
        text = ''
        for arg in node.args:
            for sub in ast.walk(arg):
                if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
                    text += sub.value
        if 'NOT GRADED' in text:
            out.append((node.lineno, text))
    return out


def main():
    print('1. every NOT GRADED line check_drc prints carries its kind')
    rows = not_graded_prints()
    check('check_drc prints NOT GRADED lines', len(rows) >= 6, str(rows))
    bare = [(ln, t[:60]) for ln, t in rows if 'NOT GRADED (' not in t]
    check('none is bare (board_score reads only `NOT GRADED (<kind>)`)', not bare, str(bare))

    td = tempfile.mkdtemp(prefix='krt1813_')
    try:
        print('2. a courtyard-scoped length rule is listed and blocks')
        b = stage(td, 'length', 'cap stub', 'length (max 1mm)', COURT)
        r = run([os.path.join('py_router', 'check_drc.py'), b])
        check('check_drc: NOT GRADED (length)',
              "NOT GRADED (length): rule 'cap stub'" in r.stdout, r.stdout[-600:])
        s, drc = score(b)
        check("board_score: drc.ungraded_rules == ['cap stub']",
              drc.get('ungraded_rules') == ['cap stub'], json.dumps(drc)[:400])
        check('board_score: its kind is length',
              drc.get('ungraded_rule_kinds') == {'cap stub': ['length']},
              json.dumps(drc.get('ungraded_rule_kinds')))
        check("board_score: 'drc.dru_rules' is ungraded",
              'drc.dru_rules' in (s.get('ungraded') or []), json.dumps(s.get('ungraded')))
        run_utils.check(PY + complete(b), refuse="nothing grades: 'cap stub' (length)", code=4)
        check('check_complete: INCOMPLETE naming the length rule', True)

        print('3. a courtyard-scoped skew rule is listed and blocks')
        b = stage(td, 'skew', 'cap skew', 'skew (max 0.01mm)', COURT)
        s, drc = score(b)
        check("board_score: drc.ungraded_rules == ['cap skew'] (skew)",
              drc.get('ungraded_rules') == ['cap skew']
              and drc.get('ungraded_rule_kinds') == {'cap skew': ['skew']},
              json.dumps(drc)[:400])
        run_utils.check(PY + complete(b), refuse="nothing grades: 'cap skew' (skew)", code=4)
        check('check_complete: INCOMPLETE naming the skew rule', True)

        print('4. controls')
        b = stage(td, 'read', 'net len', 'length (max 20mm)', "A.NetName == 'DPA_P'")
        s, drc = score(b)
        check('a length rule check_drc reads is graded, not listed',
              not drc.get('ungraded_rules')
              and 'drc.dru_rules' not in (s.get('ungraded') or []), json.dumps(drc)[:400])
        r = run(complete(b))
        check('check_complete does not call it ungraded',
              'nothing grades' not in r.stdout, r.stdout[-400:])
        b = stage(td, 'clearance', 'cap stub', 'clearance (min 0.3mm)', COURT)
        s, drc = score(b)
        check('the courtyard clearance rule is listed as before, kind clearance',
              drc.get('ungraded_rules') == ['cap stub']
              and drc.get('ungraded_rule_kinds') == {'cap stub': ['clearance']},
              json.dumps(drc)[:400])
        run_utils.check(PY + complete(b), refuse="nothing grades: 'cap stub' (clearance)", code=4)
        check('check_complete: INCOMPLETE naming the clearance rule', True)
    except AssertionError as exc:
        check('a run_utils assertion held', False, str(exc)[:1500])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
