#!/usr/bin/env python3
"""The placement skill has a step for a round with no judgeable seed (#1321).

plan-pcb-placement said never to judge or keep an illegal seed, then
fast-routed the seeds judge.json names -- and had no next step when
judge.json named none (gp14 board 06 round 1: `"judge": []`, all 12 seeds
in `illegal`), nor when every judged seed was legal but NOT BUILDABLE
(boards 04 and 07) or failed `check_floorplan --intent` (board 12).

Checks:
  1. section 3.6 of the skill carries a "When no seed can be judged" block
     naming all three cases, the input-vs-seed decision, the hand repair
     and the re-judge of one seed, and whether placing again is a new pass
     (and how the ledger records it);
  2. `judge_seeds.py` refuses board 06's judge.json shape (every seed
     illegal) before routing anything, naming the illegal seeds and pointing
     at that block, so the refusal and the skill cannot drift apart.
"""
import json
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from run_utils import check, tool                             # noqa: E402

SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'SKILL.md')
HEADING = 'When no seed can be judged'


def section(text, start, end):
    a = text.index(start)
    return text[a:text.index(end, a)]


def test_skill_block():
    text = open(SKILL, encoding='utf-8').read()
    s36 = section(text, '### 3.6 ', '### 3.7 ')
    assert f'**{HEADING}.**' in s36, f'section 3.6 has no "{HEADING}" block'
    block = s36[s36.index(f'**{HEADING}.**'):]      # the last bullet of 3.6
    for case, want in (('empty judge.json', '`judge.json` names no seed'),
                       ('every judged seed unbuildable', 'Every judged seed is NOT BUILDABLE'),
                       ('every judged seed failing the floorplan',
                        'Every judged seed fails the floorplan'),
                       ('the input fix', "is the input's"),
                       ('the hand repair', 'repair it by hand'),
                       ('judging one repaired seed', '--seeds S'),
                       ('whether placing again is a new pass', 'new pass of §3.6'),
                       ('the ledger row for an unusable pass', '--kind placement --rejected')):
        assert want in block, f'the block does not cover {case} ({want!r})'
    # each command it names is a real tool
    for t in set(re.findall(r'(\w+\.py)', block)):
        tool(t)
    print(f'  PASS: section 3.6 has the "{HEADING}" block: three cases, the way out '
          f'of each, and the pass rule')


def test_runner_points_at_the_block():
    with tempfile.TemporaryDirectory() as td:
        run = os.path.join(td, 'r1')
        os.makedirs(run)
        judge = os.path.join(run, 'judge.json')
        # gp14 board 06 round 1, as place_simple wrote it
        with open(judge, 'w', encoding='utf-8') as fh:
            json.dump({'judge': [], 'scattered': list(range(6)), 'clustered': list(range(6, 12)),
                       'anchored': [], 'illegal': list(range(12))}, fh)
        chain = os.path.join(td, 'chain.txt')
        with open(chain, 'w', encoding='utf-8') as fh:
            fh.write('python3 -X utf8 py_router/copy_board.py {in} {out}\n')
        r = check([sys.executable, '-X', 'utf8', tool('judge_seeds.py'), judge, '--chain', chain],
                  refuse=HEADING, code=2)
        assert 'every placed seed left a part illegal' in r.stderr, r.stderr
        assert 'Nothing was routed' in r.stderr, r.stderr
        assert os.listdir(run) == ['judge.json'], os.listdir(run)
    print('  PASS: judge_seeds refuses an all-illegal round, nothing routed, and points '
          'at the skill block')


def main():
    test_skill_block()
    test_runner_points_at_the_block()
    print('test_1321_no_judgeable_seed: all passed')


if __name__ == '__main__':
    main()
