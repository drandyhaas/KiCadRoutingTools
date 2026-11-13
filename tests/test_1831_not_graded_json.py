#!/usr/bin/env python3
"""#1831: `check_drc --json` names every `.kicad_dru` rule it did not grade.

check_drc prints `NOT GRADED (<kind>): rule '...'` for a rule it cannot
grade, and board_score scrapes those lines. The `--json` document carried
none of them, so a reader of the JSON saw `violations: 0` on a board whose
rules nothing graded; with `--quiet` even the console lines went.

Checks (no routing, no KiCad):
  1. one rule of each kind (clearance, size, length, via_count, skew,
     disallow) whose condition is not read lands in `not_graded` with its
     kind and reason, and with its nets where the pass knows them;
  2. every NOT GRADED console line has its row, and no row lacks a line;
  3. `--quiet` gives the same rows;
  4. run_drc still returns a list, and fills a `not_graded` list it is
     handed; a board with no rules file lists none.

    python3 tests/test_1831_not_graded_json.py
"""
import contextlib
import io
import json
import os
import re
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'cap_chain.kicad_pcb')
UNREAD = "A.getField('x') == 'y'"
#: rule -> (constraint, the kind its NOT GRADED line names)
RULES = {'cap gap': ('clearance (min 0.3mm)', 'clearance'),
         'cap width': ('track_width (min 0.3mm)', 'size'),
         'cap stub': ('length (max 1mm)', 'length'),
         'cap vias': ('via_count (min 1)', 'via_count'),
         'cap skew': ('skew (max 0.01mm)', 'skew'),
         'cap nothing': ('disallow via', 'disallow')}
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru):
    b = os.path.join(td, name + '.kicad_pcb')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, b], accept=True)
    if dru is not None:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return evidence(b, 'the staged board')


def grade(board, out, *extra):
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), board,
                    '--json', out] + list(extra), accept=True)
    with open(evidence(out, 'the --json document'), encoding='utf-8') as fh:
        return r.stdout, json.load(fh)


def main():
    dru = '(version 1)\n' + ''.join(
        f'(rule "{n}" (constraint {c})\n  (condition "{UNREAD}"))\n'
        for n, (c, _k) in RULES.items())
    with tempfile.TemporaryDirectory() as td:
        board = stage(td, 'b', dru)
        print("1. every kind is a row of `not_graded`")
        out, doc = grade(board, os.path.join(td, 'drc.json'))
        rows = doc.get('not_graded')
        ok(isinstance(rows, list), f"`not_graded` is a list: {type(rows).__name__}")
        rows = rows if isinstance(rows, list) else []
        got = {(r.get('rule'), r.get('kind')) for r in rows}
        want = {(n, k) for n, (_c, k) in RULES.items()}
        ok(got == want, f"one row per rule, with its kind: {sorted(got)}")
        ok(all(r.get('reason') for r in rows), "every row says why")
        by = {r.get('rule'): r for r in rows}
        for n in ('cap stub', 'cap vias', 'cap skew'):
            ok(bool(by.get(n, {}).get('nets')), f"'{n}' names the nets it may bind")
        ok(by.get('cap width', {}).get('constraints') == ['track_width'],
           f"the size row names its constraint: {by.get('cap width')}")
        ok(by.get('cap gap', {}).get('clearance_mm') == 0.3,
           f"the clearance row carries its value: {by.get('cap gap')}")

        print("2. the console lines and the rows agree")
        lines = set(re.findall(r"NOT GRADED \(([A-Za-z_]+)\): rule '([^']*)'", out))
        ok(lines == {(k, n) for n, k in got} and len(lines) == len(RULES),
           f"{len(lines)} NOT GRADED line(s), one per row")

        print("3. --quiet keeps the rows")
        qout, qdoc = grade(board, os.path.join(td, 'drcq.json'), '--quiet')
        ok('NOT GRADED' not in qout, "control: --quiet prints no NOT GRADED line")
        ok(qdoc.get('not_graded') == doc.get('not_graded'), "the same rows under --quiet")

        print("4. run_drc in process")
        import check_drc
        sink = []
        with contextlib.redirect_stdout(io.StringIO()):
            ret = check_drc.run_drc(board, quiet=True, print_summary=False,
                                    not_graded=sink)
        ok(isinstance(ret, list), f"run_drc returns a list: {type(ret).__name__}")
        ok(sorted((r['rule'], r['kind']) for r in sink) == sorted(want),
           f"the list it is handed is filled: {len(sink)} row(s)")
        bare = stage(td, 'n', None)
        _o, ndoc = grade(bare, os.path.join(td, 'n.json'))
        ok(ndoc.get('not_graded') == [], f"no rules file, no rows: {ndoc.get('not_graded')}")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
