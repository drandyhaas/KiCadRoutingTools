#!/usr/bin/env python3
"""#1383: the level-2 judging route is a screen, not a verdict, and judge_seeds
could not say so or do anything about it.

Level 2 skips the rescue tail and the plane finalize, so its vias and copper
differ from the full route's by a few percent per seed and, when every seed
routes, its blocking is decided by one open net: the top seeds' order can invert
on the full route. judge_seeds now

  1. NOTEs the seeds that lie within that spread of the best (`close_seeds`:
     the same placement defects, blocking within `CLOSE_BLOCKING`, copper
     within `CLOSE_COPPER`) and says how to settle them;
  2. `--confirm close` / `--confirm K` routes those seeds again at the full
     level (`step_argv(full=True)`: every `--quick` taken off a route step)
     into `s<S>_full_route/`, `s<S>_full.kicad_pcb`, `s<S>_full.score.json`,
     and names the best of the confirmed seeds.

    python3 tests/test_1383_judge_confirm.py
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _d) if _d else ROOT
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import judge_seeds                                            # noqa: E402
from copy_board import copy_board                             # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
RUNNER = tool('judge_seeds.py')
SUB = {'in': '/r/in.kicad_pcb', 'out': '/r/o.kicad_pcb', 'dir': '/r', 'seed': 1, 'run': '/r'}


def write(path, text):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def test_full_takes_the_quick_level_off():
    line = 'python3 py_router/route.py {in} {out} --nets * --quick 2 --json-out {dir}/r.json'
    argv, added = judge_seeds.step_argv(line, SUB, full=True)
    assert '--quick' not in argv and '2' not in argv and not added, argv
    assert argv[-2:] == ['--json-out', '/r/r.json'], argv
    argv, _ = judge_seeds.step_argv('py_router/route_diff.py {in} {out} --quick=1', SUB,
                                    full=True)
    assert not any(t.startswith('--quick') for t in argv), argv
    argv, _ = judge_seeds.step_argv('py_router/route.py {in} {out} --quick', SUB, full=True)
    assert '--quick' not in argv, argv
    # a plane step has none to lose, and the default still adds level 2
    argv, _ = judge_seeds.step_argv('py_router/route_planes.py {in} {out}', SUB, full=True)
    assert argv[-2:] == ['/r/in.kicad_pcb', '/r/o.kicad_pcb'], argv
    argv, added = judge_seeds.step_argv('py_router/route.py {in} {out}', SUB)
    assert added and argv[-2:] == ['--quick', '2'], argv
    print('  PASS: full takes every --quick form off a route step; the default still adds 2')


def _res(seed, blk, cu, fp=0, asm=0):
    return {'seed': seed, 'score': {'blocking': blk, 'blocking_by': {'floorplan': fp,
                                                                     'assembly': asm},
                                    'joins_needed': 0, 'quality': {'copper_mm': cu,
                                                                   'vias': 5, 'segments': 9}}}


def test_close_seeds():
    ranked = [_res(4, 55, 100.0), _res(2, 55, 109.0), _res(1, 56, 104.0),
              _res(5, 55, 111.0), _res(0, 58, 101.0), _res(3, 55, 100.0, fp=1)]
    got = judge_seeds.close_seeds(ranked)
    assert got == [4, 2, 1], got           # 5: copper +11%, 0: blocking +3, 3: a defect
    assert judge_seeds.close_seeds([]) == []
    assert judge_seeds.close_seeds(ranked[:1]) == [4]
    lone = [_res(7, None, None), _res(8, None, None)]
    assert judge_seeds.close_seeds(lone) == [7], 'an unscored blocking is never close'
    print('  PASS: close = same defects, blocking within 1, copper within 10% of the best')


def make_run(td, seeds):
    run = os.path.join(td, 'run')
    os.makedirs(run)
    for s in seeds:
        copy_board(BOARD, os.path.join(run, f's{s}.kicad_pcb'))
    write(os.path.join(run, 'judge.json'), json.dumps({'judge': list(seeds), 'illegal': []}))
    return run


def _summary(out):
    line = next(x for x in out.splitlines() if x.startswith('JSON_SUMMARY: '))
    return json.loads(line[len('JSON_SUMMARY: '):])


def test_end_to_end():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        run = make_run(td, (0, 1, 2))
        chain = write(os.path.join(td, 'chain.txt'),
                      'python3 -X utf8 py_router/copy_board.py {in} {out}\n')
        judge = os.path.join(run, 'judge.json')
        # identical boards score identically: all three are close
        r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain], accept=True)
        doc = _summary(r.stdout)
        assert doc['close'] == doc['ranking'] and len(doc['close']) == 3, doc
        assert 'lie within the quick/full spread' in r.stdout and '--confirm close' in r.stdout, \
            r.stdout[-900:]
        assert doc['confirmed'] == [] and not os.path.exists(
            os.path.join(run, 's0_full.score.json'))
        # --confirm 2: the two best ranked are routed again, full
        r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain,
                   '--confirm', '2'], accept=True)
        doc = _summary(r.stdout)
        assert len(doc['confirmed']) == 2 and doc['best'] == doc['confirmed'][0], doc
        for s in doc['confirmed']:
            evidence(os.path.join(run, f's{s}_full.kicad_pcb'), f'seed {s} full board')
            evidence(os.path.join(run, f's{s}_full.score.json'), f'seed {s} full score')
            evidence(os.path.join(run, f's{s}_full_route', 'chain.log'), 'full chain log')
        assert 'by the full route' in r.stdout and 'confirmed ranking' in r.stdout, r.stdout[-900:]
        # --confirm close: all three are close, so all three
        r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain,
                   '--confirm', 'close'], accept=True)
        assert len(_summary(r.stdout)['confirmed']) == 3, r.stdout[-900:]
        # a lone best: nothing to confirm, said
        r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain, '--seeds', '1',
                   '--confirm', 'close'], accept=True)
        assert 'nothing to confirm' in r.stdout and _summary(r.stdout)['confirmed'] == [], \
            r.stdout[-600:]
        # a bad count is refused, for that reason
        check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain, '--confirm', '0'],
              refuse='is not `close` or a seed count', code=2)
    print('  PASS: the close seeds are NOTEd; --confirm K / close routes them again at the full '
          'level into s<S>_full.*, names the best of them; a lone best and a bad count say so')


def main():
    test_full_takes_the_quick_level_off()
    test_close_seeds()
    test_end_to_end()
    print('test_1383_judge_confirm: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
