#!/usr/bin/env python3
"""`--from-rows` must read the file the study itself writes (#703).

The study writes `<out>/rows.jsonl` -- ONE JSON row per line -- and both
`predictor_study.py`'s usage text and `docs/placement-predictors.md` tell the
reader to feed exactly that file back in with `--from-rows`. The loader read
only the `{rows: [...]}` document form, so the documented command died with
`JSONDecodeError: Extra data: line 2 column 1`: a tool refusing the file it had
just written. That is the bug this file's first case pins.

The SECOND case is the one that survived the first fix, and it is nastier
because it is silent. A one-row JSONL parses as a lone JSON object, so a
loader that answers `d.get('rows') or []` reads it as a document that declares
no rows and returns NOTHING. `main()` then prints "carries no study rows (only
0 harvest row(s))" -- blaming the file's CONTENT for what the parser dropped --
and returns 0. A crash that says "I cannot read this" is strictly better than
an exit-0 that says "there was nothing in it", and an interrupted run leaves
exactly a one-row file.

Only a DOCUMENT carries the `rows` key, so anything else that parses as an
object is one row. That is the whole rule, and every shape below follows from
it.

Fixtures are synthesized in a temp dir: no `wk/` tree, no recorded run, so this
carries its own evidence on any machine.

    python3 -X utf8 tests/test_703_from_rows.py
"""
import json
import os
import sys
import tempfile

RUN_ALL_TIMEOUT = 120
#: The integration proxy in run_all is the literal substring 's-u-b-p-r-o-c-e-s-s'
#: anywhere in the source, which this file would trip merely by NAMING it. It
#: starts none and runs in milliseconds.
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

import predictor_study as PS                 # noqa: E402

FAILURES = []
ROW = {'board': 'tigard', 'variant': 'authored', 'source': 'study',
       'blocking': 3, 'hpwl': 1234.5}


def check(name, got, want):
    if got == want:
        print(f'  OK   {name} -- {got} row(s)')
    else:
        print(f'  FAIL {name} -- got {got}, want {want}')
        FAILURES.append(name)


def w(d, name, text):
    p = os.path.join(d, name)
    with open(p, 'w', encoding='utf-8') as f:
        f.write(text)
    return p


def test_every_shape_the_documented_command_can_hand_it(d):
    jsonl = '\n'.join(json.dumps(ROW) for _ in range(3)) + '\n'
    check('the run-s own rows.jsonl (3 rows)',
          len(PS.load_rows(w(d, 'a.jsonl', jsonl))), 3)

    # The regression this file exists for.
    check('a ONE-row rows.jsonl is one row, not zero',
          len(PS.load_rows(w(d, 'b.jsonl', json.dumps(ROW) + '\n'))), 1)

    check('blank lines are skipped, not counted',
          len(PS.load_rows(w(d, 'c.jsonl',
                             json.dumps(ROW) + '\n\n' + json.dumps(ROW)))), 2)

    doc = {'rows': [ROW, ROW]}
    for indent in (None, 0, 2):
        check(f'a {{rows: [...]}} document (indent={indent})',
              len(PS.load_rows(w(d, f'doc{indent}.json',
                                 json.dumps(doc, indent=indent)))), 2)

    check('a bare list of rows',
          len(PS.load_rows(w(d, 'list.json', json.dumps([ROW, ROW])))), 2)


def test_empty_is_empty_but_only_when_it_really_is(d):
    """The fix must not turn "no rows" into a phantom row."""
    check('a document declaring zero rows',
          len(PS.load_rows(w(d, 'e1.json', json.dumps({'rows': []})))), 0)
    check('an empty object', len(PS.load_rows(w(d, 'e2.json', '{}'))), 0)
    check('an empty file', len(PS.load_rows(w(d, 'e3.jsonl', ''))), 0)


def test_a_file_it_cannot_read_says_so_loudly(d):
    """The one thing worse than the crash is a silent empty (see docstring)."""
    p = w(d, 'bad.jsonl', json.dumps(ROW) + '\nthis is not json\n')
    try:
        PS.load_rows(p)
    except SystemExit as e:
        if 'bad.jsonl:2' in str(e):
            print('  OK   an unreadable line names the file AND the line')
        else:
            print(f'  FAIL refusal did not name file:line -- {e}')
            FAILURES.append('refusal names file:line')
    else:
        print('  FAIL an unreadable line was accepted silently')
        FAILURES.append('unreadable line accepted')


def test_the_rows_survive_intact(d):
    """Count is not enough -- a loader could return the right number of wrong
    things. `main()` filters on `source` and reports on `blocking`."""
    got = PS.load_rows(w(d, 'one.jsonl', json.dumps(ROW) + '\n'))
    if got == [ROW]:
        print('  OK   the row round-trips field for field')
    else:
        print(f'  FAIL row content changed -- {got}')
        FAILURES.append('row content')


def main():
    print('--- #703 --from-rows accepts the file the study writes')
    with tempfile.TemporaryDirectory() as d:
        test_every_shape_the_documented_command_can_hand_it(d)
        test_empty_is_empty_but_only_when_it_really_is(d)
        test_a_file_it_cannot_read_says_so_loudly(d)
        test_the_rows_survive_intact(d)
    if FAILURES:
        print(f'\n{len(FAILURES)} failure(s):')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_703_from_rows: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
