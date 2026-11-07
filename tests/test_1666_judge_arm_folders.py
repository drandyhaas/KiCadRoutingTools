#!/usr/bin/env python3
"""#1666: judging two arms keeps both, by plan-pcb-placement 3.7's own recipe.

`judge_seeds` works in the folder judge.json is in and makes its files afresh,
so a second arm in one folder replaced the first's boards, logs and scores with
no word, and `route_review` then read whichever arm ran last. It has no arm
option and needs none: 3.7 gives each arm a folder (a copy of judge.json and of
the intent, and the seeds' boards by copy_board.py).

Pinned here, on two chains cheap enough to run per test (no routing: the
chain's job here is to leave a mark of which arm ran where):
  1. the recipe keeps arm A's files in the run folder through arm B's run, and
     arm B's in its own;
  2. a chain's `{run}/intent.json` is the arm's own copy;
  3. judge_seeds' seed filter reads the arm's judge.json: a seed it skips needs
     no board there; `--seeds` needs the one it names; a named seed with no
     board is refused by reason;
  4. the GUI's candidates table reads the run folder and no arm's;
  5. the skill's text says it: one folder per arm, what to copy, what to keep,
     which arm route_review reads, and that the fixed names are one arm's.

    python3 tests/test_1666_judge_arm_folders.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('tests', 'py_router', 'kicad_routing_plugin'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check, evidence, tool                    # noqa: E402

PY = [sys.executable, '-B', '-X', 'utf8']
JUDGE = tool('judge_seeds.py')
COPY = tool('copy_board.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'SKILL.md')


def chain(path, *lines):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    return path


def arm_folder(run, name, seeds):
    arm = os.path.join(run, name)
    os.makedirs(arm)
    for f in ('judge.json', 'intent.json'):
        shutil.copy(os.path.join(run, f), arm)
    for s in seeds:
        check(PY + [COPY, os.path.join(run, f's{s}.kicad_pcb'),
                    os.path.join(arm, f's{s}.kicad_pcb')], accept=True)
    return arm


def judge(folder, chain_path, *flags, **kw):
    argv = PY + [JUDGE, os.path.join(folder, 'judge.json'), '--chain', chain_path,
                 '--intent', os.path.join(folder, 'intent.json'), *flags]
    return check(argv, allow=('WARNING',), **kw)


def log_of(folder, s=0):
    return open(evidence(os.path.join(folder, f's{s}_route', 'chain.log')), encoding='utf-8').read()


def test_recipe(run):
    a = chain(os.path.join(run, 'judge_chain_A.txt'),
              "python3 -X utf8 py_router/copy_board.py {in} {out}",
              "python3 -X utf8 py_tools/check_floorplan.py {in} --intent {run}/intent.json --quiet")
    b = chain(os.path.join(run, 'judge_chain_B.txt'),
              "python3 -X utf8 py_router/copy_board.py {in} {out}",
              "python3 -X utf8 py_router/copy_board.py {in} {out}",
              "python3 -X utf8 py_tools/check_floorplan.py {in} --intent {run}/intent.json --quiet")
    judge(run, a, accept=True)
    before = open(os.path.join(run, 's0_fast.score.json'), encoding='utf-8').read()
    arm = arm_folder(run, 'arm_B', (0, 1))
    r = judge(arm, b, accept=True)
    assert '"ranking"' in r.stdout, r.stdout[-500:]

    # 1: both arms' files stand, each its own
    assert open(os.path.join(run, 's0_fast.score.json'), encoding='utf-8').read() == before
    assert not os.path.exists(os.path.join(run, 's0_route', '2_copy_board.kicad_pcb')), \
        'arm A\'s route folder must not hold arm B\'s second step'
    assert os.path.isfile(os.path.join(arm, 's0_route', '2_copy_board.kicad_pcb'))
    for f in ('s0_fast.kicad_pcb', 's0_fast.score.json', 's0_route/redo_commands.sh'):
        evidence(os.path.join(run, f))
        evidence(os.path.join(arm, f))
    # 2: {run} is the arm's folder
    assert os.path.join(arm, 'intent.json') in log_of(arm), log_of(arm)[-500:]
    assert os.path.join(run, 'intent.json') in log_of(run) and arm not in log_of(run)
    print('  ok: the recipe keeps both arms\' boards, logs and scores; {run} is the arm\'s folder')
    return a


def test_filter(run, a):
    doc = json.load(open(os.path.join(run, 'judge.json'), encoding='utf-8'))
    # a named seed its judge.json grades unclean: no board needed for it
    arm = arm_folder(run, 'arm_filter', (1,))
    json.dump(dict(doc, floorplan={'0': 2}),
              open(os.path.join(arm, 'judge.json'), 'w', encoding='utf-8'))
    r = judge(arm, a, accept=True)
    assert 'skipping seed(s) 0' in r.stdout, r.stdout[-500:]
    assert not os.path.exists(os.path.join(arm, 's0_route'))
    assert os.path.isfile(os.path.join(arm, 's1_fast.kicad_pcb'))
    # --seeds: only the seed it names
    arm = arm_folder(run, 'arm_named', (0,))
    judge(arm, a, '--seeds', '0', accept=True)
    assert not os.path.exists(os.path.join(arm, 's1_route'))
    # a seed named with no board in the arm's folder
    arm = arm_folder(run, 'arm_empty', ())
    judge(arm, a, refuse='no placement for seed(s) 0, 1', code=2)
    print('  ok: the seed filter reads the arm\'s judge.json; a missing board is refused by reason')


def test_gui_table(run):
    from placement_run import scan_candidates
    rows = {r['seed']: r for r in scan_candidates(run)}
    assert set(rows) == {0, 1}, rows
    assert os.path.dirname(rows[0]['fast']) == run, rows[0]
    print('  ok: the candidates table reads the run folder, no arm\'s')


def test_skill_text():
    t = open(SKILL, encoding='utf-8').read()
    three7 = t[t.index('### 3.7 '):t.index('### 3.8 ')]
    for phrase in (r'\*\*Each arm gets its own folder\.\*\*',
                   r'cp wk/<run>/judge\.json wk/<run>/intent\.json wk/<run>/arm_B/',
                   r'copy_board\.py wk/<run>/s<S>\.kicad_pcb wk/<run>/arm_B/s<S>\.kicad_pcb',
                   r'judge_seeds\.py wk/<run>/arm_B/judge\.json',
                   r'list each arm\'s ranking',
                   r'route_review\.py wk/<run>/s<S>_fast\.kicad_pcb[^\n]*\n[^\n]*the arm the final route will use'):
        assert re.search(phrase, three7), f'3.7 lost: {phrase}'
    two = t[t.index('## 2. '):t.index('## 3. ')]
    assert 'A second judging arm keeps these names in a folder of its' in two
    print('  ok: 3.7 and the fixed-names passage say it')


def main():
    test_skill_text()
    d = tempfile.mkdtemp(prefix='t1666_')
    try:
        fixed = os.path.join(d, 'fixed.kicad_pcb')
        check(PY + [COPY, BOARD, fixed], accept=True)
        intent = os.path.join(d, 'intent.json')
        check(PY + [tool('check_floorplan.py'), fixed, '--emit-intent', intent, '--quiet'],
              accept=True, allow=('WARNING',))
        evidence(intent)
        for s in (0, 1):
            check(PY + [COPY, BOARD, os.path.join(d, f's{s}.kicad_pcb')], accept=True)
        with open(os.path.join(d, 'judge.json'), 'w', encoding='utf-8') as fh:
            json.dump({'judge': [0, 1], 'baseline': fixed}, fh)
        a = test_recipe(d)
        test_filter(d, a)
        test_gui_table(d)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
