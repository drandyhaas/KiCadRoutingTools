#!/usr/bin/env python3
"""#1693: the placement skill's grade.py is told whose rules the project holds.

`done` read check_complete's form without --authored-from on every board, so on
the user's own rules it could say done beside `check_complete_authored: UNSOUND`.
The rule (review-routed-board Step 5): on aspirational rules the bare form gates
and the flag's verdict is reported; on the user's own rules the flag gates. What
each case pins:

* one board routed at the fab floor, below the floors its input's project
  declares, reads done on aspirational rules (the default, with no `done_form`)
  and not done on the user's own (exit 4, naming the form);
* the control: a board routed at the input's own rules is done on the user's own
  rules, so the flag gates rather than always refusing;
* an explicit `--whose-rules aspirational` writes what the default writes;
* a baseline that cannot supply the floors is refused before anything grades,
  with the reason: no project, an unreadable one, one declaring no rules, or one
  a real writeback already lowered (where check_complete would compare the board
  with the lowered floors);
* a floor check that did not run is not done on the user's own rules even when
  check_complete says DONE;
* `--mode place` refuses the flag rather than ignoring it;
* the skill's command carries the flag, spelled as the script spells it.

    python3 tests/test_1693_grade_whose_rules.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement')
SCRIPTS = os.path.join(SKILL, 'scripts')
GRADE = os.path.join(SCRIPTS, 'grade.py')
sys.path.insert(0, SCRIPTS)
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import copy_board  # noqa: E402
import grade  # noqa: E402
import run_utils  # noqa: E402

INPUT = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']

FAILS = []


def check(name, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f": {detail}" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def stage(td, name):
    dst = os.path.join(td, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(INPUT, dst)
    return run_utils.evidence(dst)


def route(src, dst, *flags):
    r = subprocess.run(PY + [os.path.join(ROOT, 'py_router', 'route.py'), src, dst,
                             '--nets', '*', *flags],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=900)
    assert r.returncode == 0, f'the route failed: {r.stdout[-800:]}{r.stderr[-800:]}'
    return run_utils.evidence(dst)


def run_grade(board, baseline, td, label, *flags):
    """(exit, the printed verdict line, the grade file)."""
    r = subprocess.run(PY + [GRADE, board, '--baseline', baseline, '--mode', 'full',
                             '--out-dir', td, '--label', label, *flags],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=900)
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith('{')), None)
    assert line, f'no verdict line (exit {r.returncode}): {r.stdout[-400:]}{r.stderr[-800:]}'
    with open(run_utils.evidence(os.path.join(td, f'grade_{label}.json')),
              encoding='utf-8') as fh:
        return r.returncode, json.loads(line), json.load(fh)


def t_the_form_follows_the_answer(td):
    asp_in, own_in = stage(td, 'asp_in.kicad_pcb'), stage(td, 'own_in.kicad_pcb')
    asp = route(asp_in, os.path.join(td, 'asp.kicad_pcb'),
                '--track-width', '0.127', '--clearance-ceiling', '0.1',
                '--via-size', '0.3', '--via-drill', '0.15',
                '--no-bga-zones', '--max-ripup', '5')
    own = route(own_in, os.path.join(td, 'own.kicad_pcb'),
                '--escalation', 'board', '--no-fix-drc-settings')

    rc, line, doc = run_grade(asp, asp_in, td, 'asp_default')
    check('the fab-floor board is done on aspirational rules, the default',
          rc == 0 and line['done'] is True, str(line))
    check('...while the flag\'s verdict is reported',
          line.get('check_complete') == 'DONE'
          and line.get('check_complete_authored') == 'UNSOUND', str(line))
    check('...and that form adds nothing to the output',
          'done_form' not in line and 'done_form' not in doc, str(line))

    rc, line, doc = run_grade(asp, asp_in, td, 'asp_own', '--whose-rules', 'users-own')
    check('the same board is not done on the user\'s own rules',
          rc == 4 and line['done'] is False, f'rc {rc} {line}')
    check('...and the output names the form that decided',
          line.get('done_form') == 'check_complete --authored-from'
          and doc.get('done_form') == line.get('done_form'), str(line))

    rc, line, _doc = run_grade(own, own_in, td, 'own_own', '--whose-rules', 'users-own')
    check('a board routed at the input\'s own rules is done on the user\'s own rules',
          rc == 0 and line['done'] is True
          and line.get('check_complete_authored') == 'DONE'
          and line.get('done_form') == 'check_complete --authored-from', f'rc {rc} {line}')
    return asp, own


def rewrite_project(board, edit):
    pro = run_utils.evidence(os.path.splitext(board)[0] + '.kicad_pro')
    with open(pro, encoding='utf-8') as fh:
        doc = json.load(fh)
    edit(doc)
    with open(pro, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)


def t_a_baseline_that_cannot_supply_the_floors(td, asp, own):
    def refused(name, baseline, reason):
        out = tempfile.mkdtemp(dir=td)
        try:
            run_utils.check(PY + [GRADE, own, '--baseline', baseline, '--mode', 'full',
                                  '--whose-rules', 'users-own',
                                  '--out-dir', out, '--label', 'x'],
                            refuse=reason, code=1)
            wrote = os.listdir(out)
            check(name, not wrote, f'a refused baseline still graded: {wrote}')
        except AssertionError as e:
            check(name, False, str(e)[:600])

    bare = stage(td, 'bare.kicad_pcb')
    os.remove(os.path.splitext(bare)[0] + '.kicad_pro')
    refused('no project beside the baseline', bare, 'no project beside it')

    torn = stage(td, 'torn.kicad_pcb')
    with open(os.path.splitext(torn)[0] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        fh.write('{ not json')
    refused('an unreadable project', torn, 'is unreadable')

    empty = stage(td, 'empty.kicad_pcb')
    rewrite_project(empty, lambda d: d['board']['design_settings'].pop('rules'))
    refused('a project declaring no rules', empty, 'declares no design rules')

    # A chain's own output: its project carries the first floors and the
    # lowered ones, from the route's real writeback.
    refused('a baseline a writeback already lowered', asp, 'a writeback already lowered')
    refused('...and the refusal names the floor', asp, 'min_track_width 0.2 -> 0.127')

    check('a baseline with its project as authored has no problem',
          grade.authored_floors_problem(run_utils.evidence(INPUT)) is None)


def fake_run(authored_doc):
    """Stands in for every grader: all clean, with `authored_doc` as what the
    --authored-from check_complete wrote."""
    def run(argv):
        argv = [str(a) for a in argv]
        tool = os.path.basename(argv[0])
        doc = {}
        if tool == 'check_assembly.py':
            doc = {'buildable': True, 'verdict': 'buildable'}
        elif tool == 'board_score.py':
            doc = {'blocking': 0, 'components': {}, 'quality': {}}
        elif tool == 'check_complete.py':
            doc = authored_doc if '--authored-from' in argv else {'verdict': 'DONE'}
        for flag in ('--json', '--json-out'):
            if flag in argv:
                with open(argv[argv.index(flag) + 1], 'w', encoding='utf-8') as fh:
                    json.dump(doc, fh)
        return 0, ''
    return run


def t_a_floor_check_that_did_not_run(td):
    real = grade._run
    grade._run = fake_run({'verdict': 'DONE',
                           'fab_floors': {'ran': False, 'reason': 'the floor check failed'}})
    err, out = io.StringIO(), io.StringIO()
    try:
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(out):
            rc = grade.main([INPUT, '--baseline', INPUT, '--out-dir', td, '--label', 'u',
                             '--whose-rules', 'users-own'])
            rc_asp = grade.main([INPUT, '--baseline', INPUT, '--out-dir', td, '--label', 'a',
                                 '--whose-rules', 'aspirational'])
            rc_dflt = grade.main([INPUT, '--baseline', INPUT, '--out-dir', td, '--label', 'd'])
    finally:
        grade._run = real
    line = json.loads(out.getvalue().splitlines()[0])
    check('a floor check that did not run is not done, though check_complete says DONE',
          rc == 4 and line['done'] is False
          and line.get('check_complete_authored') == 'DONE', f'rc {rc} {line}')
    check('...and the output says why',
          line.get('authored_floors_unread') == 'the floor check failed', str(line))
    check('that verdict does not reach the aspirational form', rc_asp == 0 and rc_dflt == 0,
          f'{rc_asp} {rc_dflt}')

    def rd(label):
        with open(run_utils.evidence(os.path.join(td, f'grade_{label}.json')), 'rb') as fh:
            return fh.read()
    check('an explicit aspirational writes what the default writes', rd('a') == rd('d'))


def t_place_mode_refuses_it(td):
    out = tempfile.mkdtemp(dir=td)
    for answer in ('aspirational', 'users-own'):
        run_utils.check(PY + [GRADE, INPUT, '--baseline', INPUT, '--mode', 'place',
                              '--whose-rules', answer, '--out-dir', out, '--label', 'x'],
                        refuse='--mode place would ignore it', code=2, allow=('usage:',))
    check('--mode place refuses the flag and grades nothing', not os.listdir(out))


def t_the_skill_commands():
    with open(run_utils.evidence(os.path.join(SKILL, 'SKILL.md')), encoding='utf-8') as fh:
        text = fh.read()
    lines = [ln for ln in text.splitlines() if 'scripts/grade.py <board>' in ln]
    check('the full-mode command carries the flag',
          any('--mode full' in ln and '--whose-rules <rules>' in ln for ln in lines), str(lines))
    check('the place-mode command does not',
          any('--mode place' in ln and '--whose-rules' not in ln for ln in lines), str(lines))
    check('every grade command names its mode', all('--mode <' not in ln for ln in lines), str(lines))
    sec = text[text.index('**Independent grade and timing**'):]
    check('the skill spells the answers as the script does',
          f'`{grade.ASPIRATIONAL}`' in sec and f'`{grade.USERS_OWN}`' in sec, sec[:400])


def main():
    run_utils.evidence(INPUT)
    with tempfile.TemporaryDirectory() as td:
        asp, own = t_the_form_follows_the_answer(td)
        t_a_baseline_that_cannot_supply_the_floors(td, asp, own)
    with tempfile.TemporaryDirectory() as td:
        t_a_floor_check_that_did_not_run(td)
    with tempfile.TemporaryDirectory() as td:
        t_place_mode_refuses_it(td)
    t_the_skill_commands()
    if FAILS:
        print(f"FAIL: {len(FAILS)}: " + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
