#!/usr/bin/env python3
"""#1671: Step 2c's small-features recipe spells the ceiling, and the skill says
once whether its route-wide via is a contradiction.

A bare `--clearance` sets only the Default class, so on a board with another
net class the recipe left that class at its own wide clearance, and Step 4 and
the flags table say a chain's routing steps pass `--clearance-ceiling`. The
figure the recipe quotes was screened when a bare `--clearance` capped every
class, which is what the ceiling does now.

Step 4 called the advanced via escape-only and item 1 used it route-wide; Step 4
now says a recipe's route-wide via is a deliberate choice, and item 1 cites it.

Checks: the skill's own text; and, on a board with two net classes, that the
ceiling reaches the other class in the written project and the bare flag does
not (the flag's meaning, read back from the file rather than from prose).

    python3 -X utf8 tests/test_1671_step2c_spelling.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
BOARD = os.path.join('kicad_files', 'flat_hierarchy.kicad_pcb')
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def text_checks():
    with open(run_utils.evidence(SKILL), encoding='utf-8') as f:
        flat = re.sub(r'\s+', ' ', f.read())
    print('A. the recipe')
    check('item 1 spells the ceiling',
          '`--track-width 0.0762 --clearance-ceiling 0.0889 --via-size 0.25 --via-drill 0.15`' in flat)
    check('no recipe spells the floor with a bare --clearance',
          '--clearance 0.0889' not in flat)
    check('the screen is tied to the ceiling',
          'when a bare `--clearance` capped every net class, which is what `--clearance-ceiling` does now' in flat)
    print('B. the via, said once')
    check('Step 4 keeps the default', 'the advanced via is escape-only, not a board-wide default' in flat)
    check("Step 4 says a route-wide via is a recipe's deliberate choice",
          "A route-wide advanced via is a recipe's deliberate choice" in flat)
    check('Step 4 says what the tool line is', 'The "use ONLY" that `--design-rules` prints' in flat)
    check('item 1 cites Step 4', 'The route-wide via belongs to the bundle (Step 4,' in flat)


def classes(out):
    with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as f:
        return {c['name']: c['clearance'] for c in json.load(f)['net_settings']['classes']}


def flag_checks(td):
    print('C. what each spelling does to the other net class')
    src = os.path.join(td, 'in.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'), BOARD, src],
                   capture_output=True, cwd=ROOT)
    run_utils.evidence(src)
    got = {}
    for tag, flag in (('bare', '--clearance'), ('ceiling', '--clearance-ceiling')):
        out = os.path.join(td, f'{tag}.kicad_pcb')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join('py_router', 'route.py'), src, out,
             '--nets', '*', '--track-width', '0.0762', flag, '0.0889',
             '--via-size', '0.25', '--via-drill', '0.15'],
            capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=ROOT)
        if not os.path.isfile(os.path.splitext(out)[0] + '.kicad_pro'):
            raise AssertionError(f'BROKEN TEST: {tag} wrote no project: '
                                 f'exit {r.returncode} {(r.stdout + r.stderr)[-600:]}')
        got[tag] = classes(out)
    check('precondition: the board has another class, wider',
          got['bare'].get('Wide', 0) > 0.0889, f'{got}')
    check('a bare --clearance leaves it at its own clearance', got['bare'].get('Wide') == 0.4, f"{got['bare']}")
    check('the ceiling caps it', abs(got['ceiling'].get('Wide', 9) - 0.0889) < 1e-9, f"{got['ceiling']}")


if __name__ == '__main__':
    text_checks()
    td = tempfile.mkdtemp(prefix='krt1671_')
    try:
        flag_checks(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print(f'  - {f}')
        sys.exit(1)
    print('ALL PASS')
