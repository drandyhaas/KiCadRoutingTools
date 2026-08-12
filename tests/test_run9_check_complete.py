#!/usr/bin/env python3
""""Is this board done?" gets a command, and it fails closed.

`board_score` is the authority on `blocking` and it says "done" too easily read
alone. It exits 0 with four of nine components UNGRADED (their flags were not
passed, and `ungraded` does not touch the exit code); `undersized` runs at the
FAB floor when no spec numbers are given, so 0 there is not "the sizes are
right"; and it has no component at all for orphan stubs, weird copper or pad
overlaps.

Underneath all of it the DRC writeback only ever LOOSENS -- every constraint
set to min(current, target), target being the smallest object actually placed
-- so a board that routed below its own declared minimum rewrites that minimum
and then grades clean against the new one. Measured once at 5629 of 5933
segments below the original floor with every checker reading clean. That was
disclosed in a printed line and enforced by nothing.

What this pins:

  * UNGRADED does not make a board fail, but is always NAMED. A board with no
    spec files has nothing to grade those components against, and making it
    fatal would put every corpus board permanently in the failing state.
  * a relaxed FAB FLOOR is UNSOUND and outranks everything else, because it is
    the one failure where the clean report is itself the evidence.
  * the verdict classes have distinct exit codes.

Run: python3 -X utf8 tests/test_run9_check_complete.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

DONE, USAGE, BOARD_STATE, INCOMPLETE, UNSOUND = 0, 2, 3, 4, 5
BOARD = os.path.join('kicad_files', 'splitflap_driver.kicad_pcb')
FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def run(*args):
    p = subprocess.run(
        [sys.executable, '-X', 'utf8', 'check_complete.py', *args],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT, timeout=1800)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def main():
    print('the verdict classes are distinct')
    check('DONE / INCOMPLETE / UNSOUND / usage / board-state are 5 codes',
          len({DONE, INCOMPLETE, UNSOUND, USAGE, BOARD_STATE}) == 5)

    print('a board that does not exist is a board-state answer, not a verdict')
    code, out = run('no_such_board.kicad_pcb')
    check('missing board -> exit 3', code == BOARD_STATE, f'exit {code}')

    print('UNGRADED is named, and does not decide the verdict by itself')
    code, out = run(BOARD, '--clearance', '0.15', '--skip-slow')
    for comp in ('floorplan', 'impedance', 'length', 'net_widths'):
        check(f'{comp} is named as unexamined', comp in out, out[-200:])
    check('...and called UNEXAMINED, not passed', 'UNEXAMINED' in out,
          out[-200:])
    check('an unrouted board is INCOMPLETE for its blocking, not its ungraded',
          code == INCOMPLETE and 'blocking' in out, f'exit {code}')

    print('a relaxed fab floor is UNSOUND, and outranks the rest')
    with tempfile.TemporaryDirectory() as tmp:
        from fix_kicad_drc_settings import find_project, scan_board_minima
        b = os.path.join(tmp, 'b.kicad_pcb')
        shutil.copyfile(BOARD, b)
        src_pro = find_project(BOARD)
        pro = os.path.join(tmp, 'b.kicad_pro')
        if os.path.isfile(src_pro):
            shutil.copyfile(src_pro, pro)
        else:
            json.dump({'board': {'design_settings': {'rules': {}}}},
                      open(pro, 'w', encoding='utf-8'))

        # Whichever fab floor this board actually has an object for. A
        # copper-free board has no track or via minimum at all, only a hole
        # diameter -- so pinning one key by name would make this test a
        # statement about the fixture rather than the mechanism.
        from fix_kicad_drc_settings import FAB_FLOOR_KEYS
        minima = scan_board_minima(b) or {}
        key, got = next(((k, minima[k]) for k, _lbl in FAB_FLOOR_KEYS
                         if isinstance(minima.get(k), (int, float))),
                        (None, None))
        check('the board reports some fab minimum to compare against',
              key is not None, str(minima))
        if key is None:
            return 1
        label = dict(FAB_FLOOR_KEYS)[key]

        # Declare a floor the board's own copper does not meet. This is the
        # ratchet's shape: the project once said one thing, the copper says
        # another, and every checker reads the project.
        authored = os.path.join(tmp, 'authored.kicad_pcb')
        shutil.copyfile(b, authored)
        doc = {'board': {'design_settings': {'rules': {key: got + 0.05}}}}
        json.dump(doc, open(os.path.join(tmp, 'authored.kicad_pro'), 'w',
                            encoding='utf-8'))

        code, out = run(b, '--clearance', '0.15', '--skip-slow',
                        '--authored-from', authored)
        check('copper below an authored floor -> UNSOUND', code == UNSOUND,
              f'exit {code}: {out[-200:]}')
        check('...and it names the floor that moved',
              label in out and '->' in out, out[-260:])
        check('...and says the clean report is the symptom',
              'the rule moved, not the copper' in out, out[-260:])

        # Without the reference there is nothing to compare against, and
        # saying so is different from saying it passed.
        code2, out2 = run(b, '--clearance', '0.15', '--skip-slow')
        check('with no --authored-from the floor check does not claim a pass',
              code2 != UNSOUND, f'exit {code2}')

    # ---- the document is a GATE INPUT, so it has to survive its own -------
    # ---- failures and be bindable to a board ------------------------------
    print('the close-out document holds up as something a gate can read')
    with tempfile.TemporaryDirectory() as td:
        jp = os.path.join(td, 'close.json')

        # A relative board path from a cwd that is NOT the repo root. _run
        # uses cwd=ROOT, so before the abspath fix board_score was handed a
        # path that does not resolve, returned nothing, and the verdict became
        # a FALSE "INCOMPLETE: blocking is null" wearing a correctly-bound
        # doc['board']. Every other test in this file runs after os.chdir(ROOT),
        # which is exactly why it survived.
        sub = os.path.join(td, 'elsewhere')
        os.makedirs(sub)
        rel = os.path.relpath(os.path.join(ROOT, BOARD), sub)
        p = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'check_complete.py'), rel,
             '--skip-slow', '--json', jp],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=sub, timeout=1800)
        out = (p.stdout or '') + (p.stderr or '')
        check('a relative board path from another cwd still resolves',
              'blocking is null' not in out and 'no score' not in out,
              out[-300:])
        doc = {}
        if os.path.isfile(jp):
            with open(jp, encoding='utf-8') as fh:
                doc = json.load(fh)
        check('...and the document names itself for a shape test',
              doc.get('kind') == 'board-complete' and doc.get('schema') == 1,
              repr({k: doc.get(k) for k in ('kind', 'schema')}))
        # Bind by CONTENT, not by path: a gate comparing paths accepts a
        # close-out for a board that has since been rewritten.
        import hashlib
        h = hashlib.sha256(open(os.path.join(ROOT, BOARD), 'rb').read())
        check('...and carries board_sha, matching the file on disk',
              doc.get('board_sha') == h.hexdigest(),
              f'doc {doc.get("board_sha")!r}')
        # --skip-slow empties `components`, so a gate that only checks keys
        # would accept a document strictly weaker than the score it wraps.
        check('--skip-slow leaves components empty, visibly',
              doc.get('components') == {}, repr(doc.get('components'))[:120])

        # An unreadable board must still produce a document that SAYS so. The
        # --json write used to be the last statement, so anything that raised
        # above it produced no document at all.
        bad = os.path.join(td, 'truncated.kicad_pcb')
        with open(bad, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20221018)\n')      # unterminated
        jp2 = os.path.join(td, 'close2.json')
        code3, out3 = run(bad, '--json', jp2)
        check('an unparseable board still writes a document',
              os.path.isfile(jp2), out3[-300:])
        if os.path.isfile(jp2):
            with open(jp2, encoding='utf-8') as fh:
                d2 = json.load(fh)
            check('...and it is never DONE', d2.get('verdict') != 'DONE',
                  repr(d2.get('verdict')))
            check('...and it says the instrument did not run, not that it passed',
                  'did not' in (d2.get('reason') or '')
                  or 'no score' in (d2.get('reason') or ''),
                  repr(d2.get('reason'))[:220])
        check('...and the exit code is not success', code3 != DONE,
              f'exit {code3}')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
