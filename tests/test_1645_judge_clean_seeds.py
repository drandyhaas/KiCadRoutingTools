#!/usr/bin/env python3
"""judge_seeds routes only the seeds judge.json grades clean, unless named (#1645).

plan-pcb-placement 3.6 says to fast-route only the judged seeds that are
buildable and floorplan-clean; 3.7's command, `judge_seeds.py judge.json`
with no `--seeds`, routed every seed judge.json names. judge.json keeps the
unclean seeds (ranked last, #1296) and carries each seed's `floorplan` and
`assembly` counts beside `judge`, which the runner never read.

Checks:
  1. `placement_defects` names a seed with a floorplan error or an assembly
     count above 0, and not a clean seed or one judge.json did not grade;
  2. with no `--seeds` the runner skips the unclean seeds, saying which and
     why, and routes the rest;
  3. with every named seed unclean it refuses before routing, pointing at
     the skill's "When no seed can be judged" block;
  4. `--seeds` still routes exactly what it names, an unclean seed included;
  5. the GUI's candidates table reads a skipped seed as "not judged", not
     as "routing" forever;
  6. the skill's 3.7 step says what the command does.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import judge_seeds                                            # noqa: E402
import placement_run                                          # noqa: E402
from copy_board import copy_board                             # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
RUNNER = tool('judge_seeds.py')
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'SKILL.md')


def make_run(td, judge, floorplan, assembly):
    run = os.path.join(td, 'run')
    os.makedirs(run)
    for s in judge:
        copy_board(BOARD, os.path.join(run, f's{s}.kicad_pcb'))
    with open(os.path.join(run, 'judge.json'), 'w', encoding='utf-8') as fh:
        json.dump({'judge': judge, 'illegal': [],
                   'floorplan': {str(k): v for k, v in floorplan.items()},
                   'assembly': {str(k): v for k, v in assembly.items()}}, fh)
    chain = os.path.join(td, 'chain.txt')
    with open(chain, 'w', encoding='utf-8') as fh:
        fh.write('python3 -X utf8 py_router/copy_board.py {in} {out}\n')
    return run, chain


def test_defects():
    doc = {'floorplan': {'0': 0, '1': 2, '2': None, '3': 0},
           'assembly': {'0': 0, '1': 0, '2': 0, '3': 1}}
    got = judge_seeds.placement_defects(doc, [0, 1, 2, 3, 4])
    assert sorted(got) == [1, 3], got
    assert 'floorplan' in got[1] and 'NOT BUILDABLE' in got[3], got
    assert judge_seeds.placement_defects({'judge': [0]}, [0]) == {}, 'an older judge.json'
    print('  PASS: a floorplan error or assembly count names a seed; clean, ungraded '
          'and older judge.json seeds are not named')


def test_skip_and_refuse():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        run, chain = make_run(td, [0, 1], {0: 0, 1: 1}, {0: 0, 1: 0})
        judge = os.path.join(run, 'judge.json')
        r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain],
                  accept=True)
        assert 'skipping seed(s) 1' in r.stdout and 'floorplan error' in r.stdout, r.stdout
        evidence(os.path.join(run, 's0_fast.kicad_pcb'), 'the clean seed routed')
        assert not os.path.exists(os.path.join(run, 's1_route')), 'the unclean seed routed'
        # --seeds routes what it names, the unclean seed too
        check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain, '--seeds', '1'],
              accept=True)
        evidence(os.path.join(run, 's1_fast.kicad_pcb'), 'the named seed routed')
    with tempfile.TemporaryDirectory() as td:
        run, chain = make_run(td, [0, 1], {0: 1, 1: 0}, {0: 0, 1: 2})
        check([sys.executable, '-X', 'utf8', RUNNER, os.path.join(run, 'judge.json'),
               '--chain', chain], refuse='When no seed can be judged', code=2)
        routed = [f for f in os.listdir(run) if '_route' in f or '_fast' in f]
        assert not routed, routed
    print('  PASS: unclean seeds skipped and named; all unclean refused, nothing routed; '
          '--seeds routes an unclean seed it names')


def test_gui_not_judged():
    with tempfile.TemporaryDirectory() as td:
        run, _chain = make_run(td, [0, 1], {0: 0, 1: 3}, {0: 0, 1: 0})
        for sd in (0, 1):
            with open(os.path.join(run, f's{sd}.json'), 'w', encoding='utf-8') as fh:
                json.dump({'final': {'cost': 1.0 + sd}, 'illegal': []}, fh)
        rows = {r['seed']: r['status'] for r in placement_run.scan_candidates(run)}
        assert rows == {0: 'routing', 1: 'not judged'}, rows
    print('  PASS: the GUI table reads the skipped seed as "not judged"')


def test_skill_says_so():
    text = open(SKILL, encoding='utf-8').read()
    s37 = text[text.index('### 3.7 '):]
    s37 = s37[:s37.index('\n### ', 4)]
    assert 'skips' in s37 and 'cannot be DONE as placed' in s37, \
        '3.7 step 2 does not say the command skips the unclean seeds'
    print('  PASS: 3.7 says the command skips the seeds that cannot be DONE as placed')


def main():
    test_defects()
    test_skip_and_refuse()
    test_gui_not_judged()
    test_skill_says_so()
    print('test_1645_judge_clean_seeds: all passed')


if __name__ == '__main__':
    main()
