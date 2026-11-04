"""#1497: a judge chain line's `NAME=value` prefix is that step's environment.

plan-pcb-routing puts env knobs on individual route steps as a prefix
(`KICAD_SMOOTH_ROUTE=1 python3 ...`), and judge_seeds handed the prefix to
the process as argv: the judge died on PermissionError before scoring any
seed. It now splits the prefix (`redo_record.split_env_prefix`, the parser
beside the recorder that writes it) and runs that step alone with it.

    python3 tests/test_1497_judge_env_prefix.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

import judge_seeds  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def unit():
    print('1. the prefix is split off the command and kept as its environment')
    line = "KICAD_SMOOTH_ROUTE=1 A_B='x y' python3 -X utf8 py_router/route.py {in} {out} --nets '*'"
    env = judge_seeds.step_env(line)
    argv, added = judge_seeds.step_argv(line, {'in': 'i.kicad_pcb', 'out': 'o.kicad_pcb'})
    check('step_env reads both knobs', env == {'KICAD_SMOOTH_ROUTE': '1', 'A_B': 'x y'}, f'{env}')
    check('the command starts at the interpreter, made this one',
          argv[0] == sys.executable and argv[1:4] == ['-X', 'utf8', 'py_router/route.py'], f'{argv}')
    check("'*' still reaches the tool unexpanded, and --quick 2 is still added",
          '*' in argv and added and argv[-2:] == ['--quick', '2'], f'{argv}')
    check('a line with no prefix has no environment', judge_seeds.step_env('python3 a.py {in}') == {})


def end_to_end(td):
    print('2. a two-step chain: the knob reaches its own step and no other')
    run = os.path.join(td, 'run')
    os.makedirs(run)
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'copy_board.py'),
                    os.path.join('kicad_files', 'cap_chain.kicad_pcb'),
                    os.path.join(run, 's0.kicad_pcb')], capture_output=True, cwd=REPO)
    run_utils.evidence(os.path.join(run, 's0.kicad_pcb'))
    with open(os.path.join(run, 'judge.json'), 'w') as fh:
        json.dump({'judge': [0]}, fh)
    with open(os.path.join(run, 'chain.txt'), 'w') as fh:
        fh.write("KICAD_SMOOTH_ROUTE=1 python3 -X utf8 py_router/route.py {in} {out} --nets '*' "
                 "--json-out {dir}/r1.json\n"
                 "python3 -X utf8 py_router/route.py {in} {out} --nets '*' --json-out {dir}/r2.json\n")
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_placer', 'judge_seeds.py'),
                        os.path.join(run, 'judge.json'), '--chain', os.path.join(run, 'chain.txt')],
                       capture_output=True, text=True, cwd=REPO)
    check('the judge runs (exit 0)', r.returncode == 0, (r.stdout + r.stderr)[-500:])
    check('...and scores the seed', os.path.isfile(os.path.join(run, 's0_fast.score.json')))
    knobs = []
    for n in ('r1', 'r2'):
        p = os.path.join(run, 's0_route', f'{n}.json')
        knobs.append(json.load(open(p)).get('env_knobs') if os.path.isfile(p) else None)
    # the knob itself, not the whole set: a runner may export knobs of its
    # own (the cloud image's KICAD_SWEEP_GIT), which every step inherits
    check('step 1 ran with the knob', (knobs[0] or {}).get('KICAD_SMOOTH_ROUTE') == '1',
          f'{knobs}')
    check('step 2 ran without it', knobs[1] is not None
          and 'KICAD_SMOOTH_ROUTE' not in knobs[1], f'{knobs}')


def main():
    unit()
    td = tempfile.mkdtemp(prefix='krt1497_')
    try:
        end_to_end(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
