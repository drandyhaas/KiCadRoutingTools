"""#1813: every `.kicad_dru` rule check_drc names NOT GRADED reaches the verdict.

check_drc's length and diff-pair passes named a rule whose condition they
cannot read with a BARE `NOT GRADED: rule '...'` line, while board_score read
only the parenthesised kinds it knew (`size`, `clearance`, `disallow`,
`via_count`). So a courtyard-scoped length or skew rule was graded by nothing,
listed nowhere, and check_complete said DONE while KiCad reported the errors.
Every NOT GRADED line now carries its kind, board_score reads EVERY kind into
`drc.ungraded_rules` (and `drc.ungraded_rule_kinds`), and check_complete
names the rule with its kind and reads INCOMPLETE.

A rule of a constraint KIND check_drc has no pass for (physical_clearance,
courtyard_clearance, an assertion, ...) printed no line at all, whatever its
condition, so nothing listed it either. Each is now named `NOT GRADED
(<kind>)` and reaches the verdict the same way (row 5). A `hole_clearance`
rule is graded instead, and named only when its condition is not read.

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
UNREAD = "A.getField('x') == 'y'"
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


#: One unconditional rule of each kind check_drc has no pass for. KiCad
#: 10.0.7 reports error rows naming each on the routed cap_chain board.
NO_PASS = (('physical_clearance', '(min 5mm)'),
           ('physical_hole_clearance', '(min 5mm)'),
           ('courtyard_clearance', '(min 50mm)'),
           ('creepage', '(min 50mm)'),
           ('track_segment_length', '(min 50mm)'),
           ('assertion', '"A.Type == \'nope\'"'))


def stage_rules(td, sub, rules):
    d = os.path.join(td, sub)
    os.makedirs(d)
    b = os.path.join(d, 'b.kicad_pcb')
    run([os.path.join('py_router', 'copy_board.py'), BOARD, b])
    run_utils.evidence(b)
    with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + rules)
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
        # a courtyard-scoped clearance rule is graded by position (#1832); a
        # term nothing reads keeps a clearance rule NOT GRADED
        b = stage(td, 'clearance', 'cap stub', 'clearance (min 0.3mm)', UNREAD)
        s, drc = score(b)
        check('an unread clearance rule is listed as before, kind clearance',
              drc.get('ungraded_rules') == ['cap stub']
              and drc.get('ungraded_rule_kinds') == {'cap stub': ['clearance']},
              json.dumps(drc)[:400])
        run_utils.check(PY + complete(b), refuse="nothing grades: 'cap stub' (clearance)", code=4)
        check('check_complete: INCOMPLETE naming the clearance rule', True)

        print('5. a rule of a kind check_drc has no pass for is named and blocks')
        b = stage_rules(td, 'kinds', ''.join(
            f'(rule "r {kind}" (constraint {kind} {value}))\n' for kind, value in NO_PASS))
        r = run([os.path.join('py_router', 'check_drc.py'), b])
        for kind, _value in NO_PASS:
            check(f'check_drc: NOT GRADED ({kind})',
                  f"NOT GRADED ({kind}): rule 'r {kind}' (check_drc has no grade "
                  f"for this constraint)" in r.stdout, r.stdout[-500:])
        # #1831: every NOT GRADED line is a row of --json `not_graded`, --quiet too
        jout = os.path.join(td, 'kinds.json')
        run([os.path.join('py_router', 'check_drc.py'), b, '--json', jout, '--quiet'])
        with open(run_utils.evidence(jout, 'the --json document'), encoding='utf-8') as fh:
            rows = {(x.get('rule'), x.get('kind')) for x in json.load(fh).get('not_graded') or []}
        check('check_drc --json: each is a `not_graded` row, --quiet or not',
              {(f'r {k}', k) for k, _v in NO_PASS} <= rows, repr(sorted(rows)))
        s, drc = score(b)
        check('board_score: each is listed with its kind',
              drc.get('ungraded_rule_kinds') == {f'r {k}': [k] for k, _v in NO_PASS}
              and 'drc.dru_rules' in (s.get('ungraded') or []),
              json.dumps(drc.get('ungraded_rule_kinds')))
        # a hole_clearance rule is graded: rows naming it, nothing listed
        b = stage_rules(td, 'hole', '(rule "r hole" (constraint hole_clearance (min 5mm)))\n')
        r = run_utils.check(PY + [os.path.join('py_router', 'check_drc.py'), b],
                            refuse='PAD-HOLE violations', code=1)
        check("check_drc: a hole_clearance rule is graded, its rows name it, "
              "and it is not NOT GRADED",
              "(rule 'r hole'; global" in r.stdout and 'NOT GRADED' not in r.stdout,
              r.stdout[-600:])
        s, drc = score(b)
        check('board_score: a graded hole_clearance rule is not listed',
              not drc.get('ungraded_rules'), json.dumps(drc)[:400])
        # ...and named, and blocking, where its condition is not read
        b = stage(td, 'hole_unread', 'hole probe', 'hole_clearance (min 5mm)', UNREAD)
        r = run([os.path.join('py_router', 'check_drc.py'), b])
        check('check_drc: NOT GRADED (hole_clearance) for an unread condition',
              "NOT GRADED (hole_clearance): rule 'hole probe'" in r.stdout, r.stdout[-600:])
        run_utils.check(PY + complete(b), refuse="nothing grades: 'hole probe' (hole_clearance)",
                        code=4)
        check('check_complete: INCOMPLETE naming the unread hole_clearance rule', True)
        b = stage_rules(td, 'physical', '(rule "probe" (constraint physical_clearance (min 5mm)))\n')
        try:
            run_utils.check(PY + complete(b),
                            refuse="nothing grades: 'probe' (physical_clearance)", code=4)
            said = ''
        except AssertionError as exc:
            said = str(exc)
        check('check_complete: INCOMPLETE naming the rule and its kind',
              not said, said[:700])
        b = stage_rules(td, 'quiet',
                        '(rule "warn" (severity warning) (constraint physical_clearance (min 5mm)))\n'
                        '(rule "ign" (severity ignore) (constraint hole_clearance (min 5mm)))\n'
                        '(rule "graded" (constraint clearance (min 0.1mm)))\n'
                        '(rule "fill" (constraint zone_connection solid))\n')
        r = run([os.path.join('py_router', 'check_drc.py'), b])
        check('a warning or ignored rule, a kind check_drc grades and a fill '
              'kind are not named (and the rules file was read)',
              'NOT GRADED' not in r.stdout
              and 'Per-layer clearance rules (.kicad_dru' in r.stdout,
              (r.stdout + r.stderr)[-500:])
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
