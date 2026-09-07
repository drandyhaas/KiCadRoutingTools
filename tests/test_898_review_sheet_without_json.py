#!/usr/bin/env python3
"""#898: `render_placement.py` must not do a requested job silently and exit 0.

The review-sheet writer, the describe narrative and the whole `--gate` verdict
all sat inside `if args.json or args.json_out:`. So:

  * `--review-sheet PATH` alone wrote the panels, wrote NO sheet, printed
    nothing about it, and exited 0 -- and the shipped skill text prescribed
    exactly that form. Run 25's placement half hit it and only noticed because
    the file it wanted to view did not exist.
  * `--gate` alone returned 0 with no verdict at all. That is the same defect in
    the flag whose entire purpose is to decide pass/fail.

The fix hoists the tail out of the block: `doc` is dict assembly with no I/O and
`--gate` cannot read its checklist without it, so both are built on every run,
and only the two JSON EMISSIONS stay gated on the flags that asked for them.

The sheet had a SECOND silent door: with `--view` / `--zoom-group` / an unplaced
board there is no full-board AFTER panel to compose, `write_review_sheet` raises,
and the old code caught it, printed to stderr and still returned 0. Fixing only
the block would have let the no-op back in through that one, so it now exits 2.
Arm 5 is the arm that would go green on a half-fix.

No existing test could see any of this: every `--review-sheet` and every
`--gate` invocation in the suite also passes `--json-out`
(`tests/test_run23_courtyard_channel.py:203`, `:342`, `:403-410`;
`tests/test_defect_render_scale.py:290`).
"""
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import run_utils                                              # noqa: E402

RP = os.path.join(ROOT, 'py_tools', 'render_placement.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb')

fails = []


def check(label, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}"
          + (f"   [{detail}]" if not ok and detail != '' else ''))
    if not ok:
        fails.append(label)


def _run(*extra):
    return subprocess.run([sys.executable, '-X', 'utf8', RP, BOARD] + list(extra),
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, timeout=900)


def test_review_sheet_alone_writes_a_sheet():
    print('\n-- 1. --review-sheet with no --json / --json-out --')
    run_utils.evidence(BOARD, 'the fixture board')
    with tempfile.TemporaryDirectory() as td:
        sheet = os.path.join(td, 'sheet.png')
        r = _run('--review-sheet', sheet, '-o', os.path.join(td, 'r.png'))
        out = r.stdout + r.stderr
        check('exit 0', r.returncode == 0, out[-400:])
        check('the sheet FILE exists and is non-empty',
              os.path.isfile(sheet) and os.path.getsize(sheet) > 0)
        check('it says so on stdout', 'review sheet ->' in r.stdout)
        check('and it did NOT print a JSON_SUMMARY nobody asked for',
              'JSON_SUMMARY:' not in r.stdout)


def test_gate_alone_returns_a_verdict():
    """The arm that returned 0 with nothing printed. This fixture fails the
    checklist (2 courtyards off the outline), so the verdict is FAIL -- what
    matters is that a verdict is REACHED at all without --json*."""
    print('\n-- 2. --gate with no --json / --json-out --')
    with tempfile.TemporaryDirectory() as td:
        r = _run('--gate', '--quiet', '-o', os.path.join(td, 'r.png'))
        check('a verdict line was printed', 'GATE:' in r.stderr, r.stderr[-300:])
        check('and it is carried by the exit code', r.returncode == 4,
              f'exit {r.returncode}')


def test_a_bare_run_still_emits_no_json():
    """The hoist must not start echoing JSON at a caller who asked for none --
    `doc` is built either way, only the EMISSIONS are gated."""
    print('\n-- 3. a bare run --')
    with tempfile.TemporaryDirectory() as td:
        r = _run('-o', os.path.join(td, 'r.png'))
        check('exit 0', r.returncode == 0, r.stderr[-300:])
        check('no JSON_SUMMARY line', 'JSON_SUMMARY:' not in r.stdout)
        check('the narrative still prints (--no-describe is off by default)',
              'DECLUTTER' in r.stdout)


def test_json_out_is_unchanged():
    print('\n-- 4. control: --json-out still writes and echoes --')
    with tempfile.TemporaryDirectory() as td:
        js = os.path.join(td, 'r.json')
        r = _run('--json-out', js, '-o', os.path.join(td, 'r.png'))
        check('the json file was written',
              os.path.isfile(js) and os.path.getsize(js) > 0)
        check('and the stdout echo is still there',
              'JSON_SUMMARY:' in r.stdout)


def test_quiet_still_silences_the_narrative():
    """The hoist regressed --quiet. `_quiet = args.quiet and args.json_out` is
    the run-24 rule for the stdout JSON ECHO ("data is never silenced into
    nowhere"); the narrative was inside the flag block and so was silent for a
    quiet caller by accident. Hoisting it and reusing that rule took `--quiet`
    alone from 3 lines of stdout to 26 -- defeating blind-first for exactly the
    callers who asked for silence."""
    print('\n-- 4b. --quiet on its own --')
    with tempfile.TemporaryDirectory() as td:
        r = _run('--quiet', '-o', os.path.join(td, 'r.png'))
        lines = [l for l in r.stdout.splitlines() if l.strip()]
        check('the narrative is suppressed', 'DECLUTTER' not in r.stdout,
              f'{len(lines)} stdout line(s)')
        check('and stdout is short', len(lines) <= 6, f'{len(lines)} lines')
    with tempfile.TemporaryDirectory() as td:
        r = _run('-o', os.path.join(td, 'r.png'))
        check('control: without --quiet it still prints',
              'DECLUTTER' in r.stdout)


def test_a_crop_still_gets_a_sheet():
    """--view leaves no full-board AFTER panel, so the strict panel filter
    found nothing and the compose raised. The first cut of #898 turned that
    into exit 2 -- which breaks this tool's own stated contract, 'SEEING an
    unplaced or broken board is this tool's job', at exactly the first boundary
    the blind-first step is prescribed at: a pile, which also sets a view.
    Compose what was written instead."""
    print('\n-- 5. --review-sheet under --view --')
    with tempfile.TemporaryDirectory() as td:
        sheet = os.path.join(td, 'sheet.png')
        r = _run('--review-sheet', sheet, '--view', '10,10,40,40',
                 '-o', os.path.join(td, 'r.png'))
        check('a sheet IS written from the crop',
              os.path.isfile(sheet) and os.path.getsize(sheet) > 0)
        check('exit 0', r.returncode == 0, f'exit {r.returncode}')
        check('and it says the panels are not the full board',
              'no full-board AFTER panel' in r.stdout, r.stdout[-300:])


def test_a_sheet_that_really_cannot_be_written_exits_2():
    """The refusal must survive, or the silent no-op returns by another door --
    and it must quote the REAL cause. The compose is wrapped in a bare
    `except Exception`, so an unwritable path is at least as likely as the
    missing-panel case; the first cut asserted the missing-panel cause for
    both and sent the reader after the wrong thing."""
    print('\n-- 6. --review-sheet to a path that cannot be written --')
    with tempfile.TemporaryDirectory() as td:
        sheet = os.path.join(td, 'no_such_dir', 'sheet.png')
        r = _run('--review-sheet', sheet, '-o', os.path.join(td, 'r.png'))
        check('no sheet was written', not os.path.isfile(sheet))
        check('and the run did NOT exit 0', r.returncode == 2,
              f'exit {r.returncode}')
        check('it names the flag', '--review-sheet' in r.stderr,
              r.stderr[-300:])
        check('and quotes the real cause, not a guessed one',
              'No such file or directory' in r.stderr
              or 'cannot find the path' in r.stderr.lower(), r.stderr[-300:])


def test_the_refusal_is_not_hidden_by_a_failing_gate():
    """--gate returns 4 further down. Reporting the sheet failure after it
    meant the message never printed and the exit code said 'checklist failed'
    for a run whose sheet was missing."""
    print('\n-- 7. --review-sheet + --gate, both failing --')
    with tempfile.TemporaryDirectory() as td:
        sheet = os.path.join(td, 'no_such_dir', 'sheet.png')
        r = _run('--review-sheet', sheet, '--gate', '--quiet',
                 '-o', os.path.join(td, 'r.png'))
        check('the sheet failure is reported',
              '--review-sheet' in r.stderr and 'no sheet was written' in r.stderr,
              r.stderr[-400:])
        check('and it is the exit code the caller sees', r.returncode == 2,
              f'exit {r.returncode}')


def main():
    test_review_sheet_alone_writes_a_sheet()
    test_gate_alone_returns_a_verdict()
    test_a_bare_run_still_emits_no_json()
    test_json_out_is_unchanged()
    test_quiet_still_silences_the_narrative()
    test_a_crop_still_gets_a_sheet()
    test_a_sheet_that_really_cannot_be_written_exits_2()
    test_the_refusal_is_not_hidden_by_a_failing_gate()
    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s) failed: {fails}")
        return 1
    print('PASS: --review-sheet writes a sheet or refuses, and --gate returns '
          'its verdict, without --json-out (#898)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
