#!/usr/bin/env python3
"""A tool runs the skill's judging chain over judge.json's seeds (#1327).

plan-pcb-placement 3.7 has every seed judge.json names routed by the same
chain (`--quick 2` on route.py and route_diff.py), its last board copied to
`s<S>_fast.kicad_pcb` and scored into `s<S>_fast.score.json`. No tool did it,
so gp14 boards 08, 11 and 14 each wrote a shell helper (board 08: 9 commands
per seed). `py_placer/judge_seeds.py` runs a chain file with {in} {out}
{dir} {seed} {run} placeholders over the named seeds.

Checks:
  1. the chain file reads as the skill writes it (continuations, comments),
     and a chain that names an unknown placeholder, reads no {in} or writes
     no {out} is refused -- but a KiCad overbar net name (`/~{RESET}`) is not
     a placeholder;
  2. a step's command: placeholders filled per token, `python3` made this
     interpreter, `--quick 2` added to route.py / route_diff.py only when
     the line carries no `--quick`;
  3. end to end on a real board: each named seed gets `s<S>_route/` with
     its chain.log, `s<S>_fast.kicad_pcb` with its `.kicad_pro`, and a
     `s<S>_fast.score.json`, and the summary ranks them;
  4. refusals for the stated reason, nothing run: an empty judge.json
     (pointing at the skill's no-judgeable-seed step), an unknown
     placeholder, a missing placement; a step that writes no board fails
     that seed (exit 4, named);
  5. the GUI names the stage while the runner is the newest command.
"""
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('', 'py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin'):
    _p = os.path.join(ROOT, _d) if _d else ROOT
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import judge_seeds                                            # noqa: E402
import placement_run                                          # noqa: E402
from copy_board import copy_board                             # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

# a committed board WITH its .kicad_pro, so the copy-out's siblings are checkable
BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
RUNNER = tool('judge_seeds.py')


def write(path, text):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def test_chain_reading_and_problems():
    with tempfile.TemporaryDirectory() as td:
        steps = judge_seeds.read_chain(write(os.path.join(td, 'c.txt'), (
            "# the board's chain\n\n"
            "python3 -X utf8 py_router/route_planes.py {in} {out} --nets GND \\\n"
            "    --plane-layers B.Cu\n"
            "python3 -X utf8 py_router/route.py {in} {out} --nets '*' '/~{RESET}' "
            "--json-out {dir}/route.json\n")))
    assert len(steps) == 2 and '--plane-layers B.Cu' in steps[0], steps
    assert judge_seeds.chain_problems(steps) == [], judge_seeds.chain_problems(steps)
    bad = judge_seeds.chain_problems(['python3 x.py {in} {output}'])
    assert any('{output}' in b for b in bad) and any('writes {out}' in b for b in bad), bad
    assert any('reads {in}' in b for b in judge_seeds.chain_problems(['x.py a {out}'])), 'no {in}'
    print('  PASS: chain read with continuations; {output}, no {in}, no {out} refused; '
          '/~{RESET} is a net name')


def test_step_argv():
    sub = {'in': '/r/s3_route/in.kicad_pcb', 'out': '/r/s3_route/1_route.kicad_pcb',
           'dir': '/r/s3_route', 'seed': 3, 'run': '/r'}
    argv, added = judge_seeds.step_argv(
        "python3 -X utf8 py_router/route.py {in} {out} --nets '*' --json-out {dir}/route.json",
        sub)
    assert argv[0] == sys.executable and added, argv
    assert argv[-2:] == ['--quick', '2'] and '*' in argv, argv
    assert '/r/s3_route/route.json' in argv and '/r/s3_route/in.kicad_pcb' in argv, argv
    argv, added = judge_seeds.step_argv(
        'python3 py_router/route_diff.py {in} {out} --quick 2', sub)
    assert not added and argv.count('--quick') == 1, argv
    argv, added = judge_seeds.step_argv('py_router/route_planes.py {in} {out}', sub)
    assert not added and argv[:3] == [sys.executable, '-X', 'utf8'], argv
    print('  PASS: placeholders filled per token, python3 is this interpreter, '
          '--quick 2 added only to a route step without one')


def make_run(td, seeds, judge):
    run = os.path.join(td, 'run')
    os.makedirs(run)
    for s in seeds:
        copy_board(BOARD, os.path.join(run, f's{s}.kicad_pcb'))
    write(os.path.join(run, 'judge.json'), json.dumps({'judge': judge, 'illegal': []}))
    return run


def test_end_to_end():
    evidence(BOARD, 'fixture board')
    evidence(os.path.splitext(BOARD)[0] + '.kicad_pro', 'fixture project')
    with tempfile.TemporaryDirectory() as td:
        run = make_run(td, (0, 1), [1, 0])
        chain = write(os.path.join(td, 'chain.txt'),
                      'python3 -X utf8 py_router/copy_board.py {in} {out}\n')
        r = check([sys.executable, '-X', 'utf8', RUNNER, os.path.join(run, 'judge.json'),
                   '--chain', chain], accept=True)
        for s in (0, 1):
            evidence(os.path.join(run, f's{s}_fast.kicad_pcb'), f'seed {s} fast board')
            evidence(os.path.join(run, f's{s}_fast.kicad_pro'), f'seed {s} fast project')
            sc = json.load(open(evidence(os.path.join(run, f's{s}_fast.score.json')),
                                encoding='utf-8'))
            assert isinstance(sc.get('blocking'), int), sc
            log = open(evidence(os.path.join(run, f's{s}_route', 'chain.log')),
                       encoding='utf-8').read()
            assert 'copy_board.py' in log and 'quick_score.py' in log, log[-500:]
        line = next(x for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY: '))
        doc = json.loads(line[len('JSON_SUMMARY: '):])
        assert sorted(doc['ranking']) == [0, 1] and doc['best'] in (0, 1), doc
    print('  PASS: both seeds routed by the chain, copied out with their project, '
          'scored and ranked')


def test_refusals():
    with tempfile.TemporaryDirectory() as td:
        run = make_run(td, (0,), [])
        ok = write(os.path.join(td, 'ok.txt'),
                   'python3 -X utf8 py_router/copy_board.py {in} {out}\n')
        judge = os.path.join(run, 'judge.json')
        check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', ok],
              refuse='names no seed to judge', code=2)
        write(judge, json.dumps({'judge': [0, 5]}))
        check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', ok],
              refuse='no placement for seed(s) 5', code=2)
        write(judge, json.dumps({'judge': [0]}))
        typo = write(os.path.join(td, 'typo.txt'),
                     'python3 -X utf8 py_router/copy_board.py {in} {ot}\n')
        check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', typo],
              refuse='names {ot}', code=2)
        assert not os.path.exists(os.path.join(run, 's0_route')), 'a refusal routed nothing'
        none = write(os.path.join(td, 'none.txt'),
                     'python3 -X utf8 -c "pass" {in} {out}\n')
        check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', none],
              refuse='wrote no board', code=4)
        assert not os.path.exists(os.path.join(run, 's0_fast.kicad_pcb')), 'no board copied'
        shutil.rmtree(run)
    print('  PASS: empty judge, missing placement and a typo placeholder refused '
          '(nothing run); a step with no board fails its seed (exit 4)')


def test_gui_stage():
    stage = placement_run.derive_stage(
        ['python3 -X utf8 py_placer/judge_seeds.py wk/judge.json --chain wk/c.txt'], None, None)
    assert stage == 'fast-routing the judged seeds', stage
    print(f'  PASS: the GUI reads the runner as "{stage}"')


def main():
    test_chain_reading_and_problems()
    test_step_argv()
    test_end_to_end()
    test_refusals()
    test_gui_stage()
    print('test_1327_judge_seeds: all passed')


if __name__ == '__main__':
    main()
