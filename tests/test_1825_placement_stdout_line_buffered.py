#!/usr/bin/env python3
"""#1825: place_simple and judge_seeds deliver each line to a pipe when it is printed.

With stdout a pipe (a tee, a wrapper, a CI log), both tools were
block-buffered: every line place_simple printed reached the reader when it
exited, and judge_seeds' opening line arrived with its first seed's result,
the one print that flushed. #1541 had started the seed CHILDREN with `-u`;
the parent printed through the default buffer.

Checks:
  1. place_simple --seeds 0,1 --jobs 1, stdout a pipe: the line
     `place_simple: seeds 0, 1` is read before s0.kicad_pcb exists (a seed
     writes its board last);
  2. judge_seeds over one seed with a chain whose step sleeps before it
     writes {out}: the opening line `judge_seeds: seeds 0` is read before
     the step has finished;
  3. control: judge_seeds started with `python -u` prints the same lines
     (timing aside) and exits the same.
"""
import json
import os
import subprocess
import sys
import tempfile
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import evidence                                 # noqa: E402
from copy_board import copy_board                              # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
JUDGE_BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')


def read_live(argv, watch, marker):
    """Run `argv` with stdout a pipe; return (exit code, lines, and whether
    `marker` existed when the first line containing `watch` was read)."""
    proc = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                            errors='replace')
    lines, seen = [], {}

    def reader():
        for line in proc.stdout:
            if watch in line and 'existed' not in seen:
                seen['existed'] = os.path.exists(marker)
            lines.append(line.rstrip('\n'))
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    try:
        proc.wait(timeout=900)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    t.join(timeout=30)
    return proc.returncode, lines, seen.get('existed')


def test_place_simple():
    evidence(BOARD, 'the fixture board')
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'b.kicad_pcb')
        copy_board(BOARD, board)
        evidence(board, 'the copied board')
        intent = os.path.join(td, 'intent.json')
        subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), board,
                        '--emit-intent', intent], cwd=ROOT, capture_output=True)
        evidence(intent, 'the emitted intent')
        groups = os.path.join(td, 'groups.json')
        with open(groups, 'w') as f:
            json.dump({'schema': 1, 'groups': [{'name': 'u1', 'refs': ['U1']}]}, f)
        evidence(groups, 'the groups file')
        rc, lines, existed = read_live(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_placer', 'place_simple.py'),
             board, os.path.join(td, 's{seed}.kicad_pcb'), '--groups', groups,
             '--intent', intent, '--seeds', '0,1', '--jobs', '1'],
            'place_simple: seeds 0, 1', os.path.join(td, 's0.kicad_pcb'))
        evidence(os.path.join(td, 's0.kicad_pcb'), "seed 0's board")
        evidence(os.path.join(td, 's1.kicad_pcb'), "seed 1's board")
        out = '\n'.join(lines)
        assert rc in (0, 4) and 'Traceback' not in out, (rc, out[-1500:])
        assert existed is not None, f'the opening line never arrived:\n{out[-1500:]}'
        assert existed is False, ('`place_simple: seeds 0, 1` reached the pipe only after '
                                  'seed 0 had written its board')
    print('  PASS: place_simple\'s opening line reached the pipe before seed 0 finished')


SLOW_STEP = """import os, sys, time
sys.path.insert(0, {py_router!r})
from copy_board import copy_board
time.sleep(4)
copy_board(sys.argv[1], sys.argv[2])
open({done!r}, 'w').write('done')
"""


def judge_run(td, unbuffered):
    run = os.path.join(td, 'run_u' if unbuffered else 'run')
    os.makedirs(run)
    copy_board(JUDGE_BOARD, os.path.join(run, 's0.kicad_pcb'))
    evidence(os.path.join(run, 's0.kicad_pcb'), "seed 0's board")
    judge = os.path.join(run, 'judge.json')
    with open(judge, 'w') as f:
        json.dump({'judge': [0], 'illegal': []}, f)
    done = os.path.join(run, 'step_done')
    step = os.path.join(run, 'slow_step.py')
    with open(step, 'w') as f:
        f.write(SLOW_STEP.format(py_router=os.path.join(ROOT, 'py_router'), done=done))
    chain = os.path.join(run, 'chain.txt')
    with open(chain, 'w') as f:
        f.write(f'python3 -X utf8 {step} {{in}} {{out}}\n')
    evidence(chain, 'the chain')
    argv = [sys.executable] + (['-u'] if unbuffered else []) + [
        '-X', 'utf8', os.path.join(ROOT, 'py_placer', 'judge_seeds.py'), judge,
        '--chain', chain]
    rc, lines, existed = read_live(argv, 'judge_seeds: seeds 0', done)
    evidence(done, 'the slow step\'s marker (the step ran)')
    return rc, [ln.replace(run, '<run>') for ln in lines], existed


def test_judge_seeds():
    evidence(JUDGE_BOARD, 'the fixture board')
    with tempfile.TemporaryDirectory() as td:
        rc, lines, existed = judge_run(td, False)
        out = '\n'.join(lines)
        assert 'Traceback' not in out, out[-1500:]
        assert existed is not None, f'the opening line never arrived:\n{out[-1500:]}'
        assert existed is False, ('`judge_seeds: seeds 0` reached the pipe only after the '
                                  'chain step had finished')
        rc_u, lines_u, _ = judge_run(td, True)

    def stable(ls):
        return [ln for ln in ls if ' s)' not in ln and ' s ' not in ln
                and 'JSON_SUMMARY' not in ln and 'seconds' not in ln]
    assert rc == rc_u, (rc, rc_u)
    assert stable(lines) == stable(lines_u), (stable(lines), stable(lines_u))
    print(f'  PASS: judge_seeds\' opening line reached the pipe before its step finished; '
          f'the same lines and exit {rc} as under python -u')


if __name__ == '__main__':
    test_place_simple()
    test_judge_seeds()
    print('ALL PASS')
