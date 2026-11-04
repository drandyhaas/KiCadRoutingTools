"""#1538: cover_facts `--refs` names every reference it was given, or refuses.

`--refs` took one comma-separated string, so three spellings failed SILENTLY
with exit 0 and a report that looked complete:
  * `--refs CON1,CONX` (a typo) reported CON1 and never mentioned CONX;
  * `--refs CON1 --refs CON2` kept only the last flag;
  * `--refs "CON1 CON2"` matched no reference and reported no connector.
Now every `--refs` value is split on commas and whitespace, a repeated flag
adds to the list, and a reference the board does not have is refused BY NAME
with exit 2 (by the engine, `facts()`, not only by main()).

    python3 tests/test_1538_cover_facts_refs.py
"""
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
TOOL = os.path.join(REPO, 'py_tools', 'cover_facts.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def reported(board, out, *refs_args):
    """The connectors cover_facts reported for these `--refs` arguments, or
    the refusal's tail when it did not accept them."""
    if os.path.exists(out):
        os.remove(out)
    try:
        run_utils.check(PY + [TOOL, board, *refs_args, '--json', out], accept=True)
    except AssertionError as ex:
        return f'not accepted: {str(ex)[-200:]}'
    run_utils.evidence(out)
    with open(out, encoding='utf-8') as fh:
        return [c['ref'] for c in json.load(fh)['connectors']]


def main():
    td = tempfile.mkdtemp(prefix='krt1538_')
    try:
        board = os.path.join(td, 'b.kicad_pcb')
        subprocess.run(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                             os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb'), board],
                       capture_output=True, text=True, cwd=REPO)
        run_utils.evidence(board)

        print('1. every spelling of two references reports both')
        for label, args in (('comma-separated', ['--refs', 'CON1,CON2']),
                            ('space-separated', ['--refs', 'CON1', 'CON2']),
                            ('one quoted string with a space', ['--refs', 'CON1 CON2']),
                            ('a repeated --refs', ['--refs', 'CON1', '--refs', 'CON2']),
                            ('mixed, with a repeat', ['--refs', 'CON1, CON2', '--refs', 'CON1'])):
            got = reported(board, os.path.join(td, 'ok.json'), *args)
            check(f'{label}: CON1 and CON2, once each', got == ['CON1', 'CON2'], f'{got}')

        print('2. a reference the board does not have is refused by name, exit 2')
        for args in (['--refs', 'CON1,CONX'], ['--refs', 'CON1', '--refs', 'CONX']):
            try:
                run_utils.check(PY + [TOOL, board, *args], refuse='the board has no CONX', code=2)
                check(f'{" ".join(args)}: refused, naming CONX', True)
            except AssertionError as ex:
                check(f'{" ".join(args)}: refused, naming CONX', False, str(ex)[-400:])
        try:
            run_utils.check(PY + [TOOL, board, '--refs', 'CONX', 'CONY', 'CON1'],
                            refuse='the board has no CONX, CONY', code=2)
            check('every missing reference is named, not only the first', True)
        except AssertionError as ex:
            check('every missing reference is named, not only the first', False, str(ex)[-400:])

        print('3. the engine refuses too, for any caller')
        sys.path.insert(0, os.path.join(REPO, 'py_tools'))
        for d in ('py_router', 'py_placer'):
            sys.path.insert(0, os.path.join(REPO, d))
        import cover_facts
        from kicad_parser import parse_kicad_pcb
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(board)
        try:
            cover_facts.facts(pcb, board, ['CON1', 'CONX'])
            check('facts() refuses a reference the board lacks', False, 'returned a report')
        except cover_facts.MissingRefs as ex:
            check('facts() refuses a reference the board lacks', ex.missing == ['CONX'],
                  f'{ex.missing}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
