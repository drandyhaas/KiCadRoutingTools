#!/usr/bin/env python3
"""#901: a ledger row that cannot replay, or cannot be read back, is refused.

`converge.py record` had two holes that turned rows into prose.

1. ONLY `argv[0]` was validated ("exists or on PATH"). Every invocation the
   doctrine teaches starts `python3 -X utf8 <script>`, so token 0 is always
   real and a corruption three tokens later sailed through. Run 25's row 31
   holds `--impedance-nets C:/Program Files/Git/D_P C:/Program Files/Git/D_N`:
   the record call ran in Git Bash without MSYS2_ARG_CONV_EXCL, which rewrites
   any `/`-prefixed argument into a Windows path -- and every KiCad net name is
   `/`-prefixed. `replay` re-executes the stored list verbatim, so that row
   grades impedance on two nets that do not exist and returns null: a vacuous
   pass. 32 of 33 rows replayed; this one did not, and nothing said so.

2. The stop token was validated ONLY when a lens FAILED. With every lens
   passing, any string was stored -- so the routing half's
   `"4 (this half): <reason>"` was accepted while the orchestrator's identical
   shape was refused twice at close-out. One record, two rules, depending on a
   lens.

WHY THIS IS A SEPARATE FILE from tests/test_converge.py: `run_utils.check`
reports an argparse accident or an ImportError as a BROKEN TEST instead of as a
satisfied guard, which is what CLAUDE.md asks for here -- and the converge tests
roll their own `assert r.returncode == 2`, which cannot tell the two apart.
Importing run_utils into that file would reclassify all 26 of its tests as slow
integration and drop them from `--fast`.
"""
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import run_utils                                              # noqa: E402

CV = os.path.join(ROOT, 'py_placer', 'converge.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
LENSES = ['--lens', 'VERDICT=PASS:lens=connectivity',
          '--lens', 'VERDICT=PASS:lens=drc',
          '--lens', 'VERDICT=PASS:lens=spec']
FAIL_LENSES = ['--lens', 'VERDICT=PASS:lens=connectivity',
               '--lens', 'VERDICT=FAIL:lens=drc',
               '--lens', 'VERDICT=PASS:lens=spec']
MANGLED = 'C:/Program Files/Git/D_P'

fails = []


def check(label, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}"
          + (f"   [{detail}]" if not ok and detail != '' else ''))
    if not ok:
        fails.append(label)


def _argv(td, *args, **kw):
    return [sys.executable, '-X', 'utf8', CV, 'record',
            '--ledger', os.path.join(td, 'l.jsonl'), '--board', BOARD] + list(args)


def _accept(td, *args):
    r = run_utils.check(_argv(td, *args), accept=True)
    return json.loads(r.stdout)


def test_a_mangled_argv_token_is_refused():
    print('\n-- 1. an MSYS2-rewritten net name in --argv --')
    run_utils.evidence(BOARD, 'the fixture board')
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        run_utils.check(
            _argv(td, '--lever', 'route the pair',
                  '--argv', sys.executable, '-c', 'pass',
                  '--impedance-nets', MANGLED),
            refuse='MSYS2', code=2)
        check('nothing was written', not os.path.exists(led))
        # ... and the refusal must carry the remedy, not just the diagnosis.
        r = subprocess.run(
            _argv(td, '--argv', sys.executable, '-c', 'pass', MANGLED),
            capture_output=True, text=True, encoding='utf-8', errors='replace',
            cwd=ROOT)
        check('it names the environment variable that prevents it',
              'MSYS2_ARG_CONV_EXCL' in r.stderr, r.stderr[-200:])


def test_a_clean_argv_still_records():
    """The control. Without it, arm 1 could be passing because `record` refuses
    every --argv it is given."""
    print('\n-- 2. control: the same command with real net names --')
    with tempfile.TemporaryDirectory() as td:
        e = _accept(td, '--lever', 'route the pair',
                    '--argv', sys.executable, '-c', 'pass', '/D_P', '/D_N')
        check('recorded', e.get('lever_argv', [])[-2:] == ['/D_P', '/D_N'],
              e.get('lever_argv'))


def test_a_mangled_lever_warns_but_records():
    """The lever is prose for a human: a mangled name there misleads a reader
    without making the row unreplayable, so it warns rather than refusing."""
    print('\n-- 3. the same shape in --lever --')
    with tempfile.TemporaryDirectory() as td:
        r = run_utils.check(
            _argv(td, '--lever', f'routed {MANGLED} at 90 ohm'), accept=True)
        check('it warns', 'WARNING' in r.stderr and 'MSYS2' in r.stderr,
              r.stderr[-200:])
        check('and the row is still written', json.loads(r.stdout).get('lever'))


def test_the_stop_token_is_checked_with_every_lens_passing():
    """The hole: with no FAIL lens, ANY string was accepted and stored."""
    print('\n-- 4. a bogus stop condition, all lenses passing --')
    with tempfile.TemporaryDirectory() as td:
        led = os.path.join(td, 'l.jsonl')
        run_utils.check(
            _argv(td, '--final', '--stop-condition',
                  'plateau: 3 iterations, no new copper') + LENSES,
            refuse='stop condition', code=2)
        check('nothing was written', not os.path.exists(led))


def test_a_token_with_a_reason_is_accepted_both_ways():
    """Run 25's own shape, `4 (this half): <reason>`, was accepted in one place
    and refused in another. It is now legal as printed, with or without a FAIL
    lens, and the token and the reason land in separate fields."""
    print('\n-- 5. token + prose, with and without a FAIL lens --')
    with tempfile.TemporaryDirectory() as td:
        e = _accept(td, '--final', '--stop-condition',
                    '4 (this half): the pair is parity-fixed', *LENSES)
        check('token stored alone', e.get('stop_condition') == '4',
              e.get('stop_condition'))
        check('reason stored apart',
              e.get('stop_reason') == 'the pair is parity-fixed',
              e.get('stop_reason'))
    with tempfile.TemporaryDirectory() as td:
        e = _accept(td, '--final', '--stop-condition',
                    '4: measured unfixable', *FAIL_LENSES)
        check('the same shape is accepted beside a FAIL lens',
              e.get('stop_condition') == '4', e.get('stop_condition'))
        check('and its reason survives',
              e.get('stop_reason') == 'measured unfixable', e.get('stop_reason'))


def test_the_fail_lens_refusals_still_hold():
    """The two rules #901 must NOT relax, now applied to the extracted token."""
    print('\n-- 6. the FAIL-lens contradictions are unchanged --')
    with tempfile.TemporaryDirectory() as td:
        run_utils.check(
            _argv(td, '--final', '--stop-condition',
                  'DONE-EXHAUSTED: everything passed', *FAIL_LENSES),
            refuse='contradiction', code=2)
    with tempfile.TemporaryDirectory() as td:
        run_utils.check(
            _argv(td, '--final', '--stop-condition', '1: done', *FAIL_LENSES),
            refuse='lens FAILED', code=2)


def test_a_reason_given_twice_and_differently_is_refused():
    print('\n-- 7. --stop-reason vs the inline reason --')
    with tempfile.TemporaryDirectory() as td:
        run_utils.check(
            _argv(td, '--final', '--stop-condition', '4: one story',
                  '--stop-reason', 'a different story', *LENSES),
            refuse='given twice', code=2)
    with tempfile.TemporaryDirectory() as td:
        e = _accept(td, '--final', '--stop-condition', '4',
                    '--stop-reason', 'the only story', *LENSES)
        check('--stop-reason alone works',
              e.get('stop_reason') == 'the only story', e.get('stop_reason'))


def test_scope_refs_takes_a_list():
    print('\n-- 8. --scope-refs as a list, as its help promised --')
    with tempfile.TemporaryDirectory() as td:
        e = _accept(td, '--scope-refs', 'R1', 'R2', 'R3')
        check('three refs recorded from one flag',
              e.get('scope_refs') == ['R1', 'R2', 'R3'], e.get('scope_refs'))
    with tempfile.TemporaryDirectory() as td:
        e = _accept(td, '--scope-refs', 'R1', '--scope-refs', 'R2, R3')
        check('repeating the flag and splitting a string both still work',
              e.get('scope_refs') == ['R1', 'R2', 'R3'], e.get('scope_refs'))


def main():
    test_a_mangled_argv_token_is_refused()
    test_a_clean_argv_still_records()
    test_a_mangled_lever_warns_but_records()
    test_the_stop_token_is_checked_with_every_lens_passing()
    test_a_token_with_a_reason_is_accepted_both_ways()
    test_the_fail_lens_refusals_still_hold()
    test_a_reason_given_twice_and_differently_is_refused()
    test_scope_refs_takes_a_list()
    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s) failed: {fails}")
        return 1
    print('PASS: a mangled argv is refused, and the stop token is checked on '
          'every record with its reason kept apart (#901)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
