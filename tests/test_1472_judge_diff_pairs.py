#!/usr/bin/env python3
"""#1472: a judging chain without a route_diff.py step routes every
differential pair of the board as two single nets, and judge_seeds ranked the
seeds on that routing without a word.

judge_seeds now WARNs once, naming the pair count, when
`list_nets.find_differential_pairs` finds pairs by name on the seeds' board
and no chain step runs route_diff.py (`unpaired_diff_pairs`). It is a
disclosure: the run judges on.

    python3 tests/test_1472_judge_diff_pairs.py
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

#: a pile with one name-detected pair, /D_P and /D_N
BOARD = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
#: a board with none
PLAIN = os.path.join(ROOT, 'kicad_files', 'qfn_underpad_coupling.kicad_pcb')
RUNNER = tool('judge_seeds.py')
ROUTE = "python3 -X utf8 py_router/route.py {in} {out} --nets '*'"
DIFF = "python3 -X utf8 py_router/route_diff.py {in} {out} --nets /D_P /D_N"


def write(path, text):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def test_unpaired_diff_pairs():
    evidence(BOARD, 'fixture pile')
    evidence(PLAIN, 'fixture board without pairs')
    assert judge_seeds.unpaired_diff_pairs(BOARD, [ROUTE]) == [('/D_P', '/D_N')]
    assert judge_seeds.unpaired_diff_pairs(BOARD, [DIFF, ROUTE]) == [], 'a route_diff step pairs them'
    assert judge_seeds.unpaired_diff_pairs(PLAIN, [ROUTE]) == [], 'no pair, nothing to say'
    print('  PASS: the pairs are named only when no chain step runs route_diff.py')


def test_warns_once_and_judges_on():
    with tempfile.TemporaryDirectory() as td:
        run = os.path.join(td, 'run')
        os.makedirs(run)
        copy_board(BOARD, os.path.join(run, 's0.kicad_pcb'))
        judge = write(os.path.join(run, 'judge.json'), json.dumps({'judge': [0], 'illegal': []}))
        # a step that writes no board ends the seed at once: the warning comes before it
        chain = write(os.path.join(td, 'chain.txt'), 'python3 -X utf8 -c "pass" {in} {out}\n')
        r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain],
                  refuse='wrote no board', code=4)
        warns = [x for x in r.stdout.splitlines() if 'differential pair' in x]
        assert len(warns) == 1 and 'WARNING: the board has 1 differential pair(s)' in warns[0] \
            and 'route_diff.py' in warns[0], r.stdout[-900:]
        assert os.path.isfile(os.path.join(run, 's0_route', 'chain.log')), 'the seed was still routed'
    print('  PASS: one WARNING naming the pair count, and the chain still runs')


def main():
    test_unpaired_diff_pairs()
    test_warns_once_and_judges_on()
    print('test_1472_judge_diff_pairs: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
