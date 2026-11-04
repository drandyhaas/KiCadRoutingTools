#!/usr/bin/env python3
"""#1473: `--confirm close` full-routes at most `CONFIRM_MAX` of the close
seeds, and neither the NOTE that recommends it nor the confirming run said so:
the seeds past the cap were never confirmed, and only JSON_SUMMARY's `close`
and `confirmed` lists showed the gap.

Both now name the cap, the seeds it leaves out, and the options that confirm
every close seed (`confirm_all`: `--confirm K` when the close seeds are the K
best ranked, else `--seeds <them> --confirm K`); the hint is run here and
must confirm them all.

    python3 tests/test_1473_judge_confirm_cap.py
"""
import json
import os
import re
import shlex
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
HINT = re.compile(r'\((--[^()]*) confirms every one\)')


def _summary(out):
    line = next(x for x in out.splitlines() if x.startswith('JSON_SUMMARY: '))
    return json.loads(line[len('JSON_SUMMARY: '):])


def test_confirm_all():
    assert judge_seeds.confirm_all([5, 4, 9, 10, 6, 3], [5, 4, 9, 10, 6, 3, 1]) == '--confirm 6'
    # a seed ranked among them but not close: --confirm K would take it, --seeds does not
    assert judge_seeds.confirm_all([5, 4, 9, 10, 6], [5, 4, 2, 9, 10, 6]) == \
        '--seeds 5,4,9,10,6 --confirm 5'
    print('  PASS: --confirm K when the close seeds lead the ranking, else --seeds them')


def test_cap_named():
    evidence(BOARD, 'fixture board')
    n = judge_seeds.CONFIRM_MAX + 2
    with tempfile.TemporaryDirectory() as td:
        run = os.path.join(td, 'run')
        os.makedirs(run)
        for s in range(n):
            copy_board(BOARD, os.path.join(run, f's{s}.kicad_pcb'))
        judge = os.path.join(run, 'judge.json')
        with open(judge, 'w', encoding='utf-8') as fh:
            json.dump({'judge': list(range(n)), 'illegal': []}, fh)
        chain = os.path.join(td, 'chain.txt')
        with open(chain, 'w', encoding='utf-8') as fh:
            fh.write('python3 -X utf8 py_router/copy_board.py {in} {out}\n')
        base = [sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain]
        # identical boards: every seed is close, two past the cap
        r = check(base, accept=True)
        doc = _summary(r.stdout)
        assert len(doc['close']) == n, doc
        left = doc['close'][judge_seeds.CONFIRM_MAX:]
        note = next(x for x in r.stdout.splitlines() if 'level 2 cannot rank them' in x)
        assert f'takes at most {judge_seeds.CONFIRM_MAX}' in note and \
            f"leaves out {', '.join(map(str, left))}" in note and HINT.search(note), note
        # --confirm close: the cap, the left-out seeds and the hint, at the confirm and the verdict
        r = check(base + ['--confirm', 'close'], accept=True)
        doc = _summary(r.stdout)
        assert len(doc['confirmed']) == judge_seeds.CONFIRM_MAX, doc
        lines = [x for x in r.stdout.splitlines()
                 if x.startswith('judge_seeds: confirming') or 'best of the confirmed seeds only' in x]
        assert len(lines) == 2 and all(f"leaves out {', '.join(map(str, left))}" in x
                                       for x in lines), r.stdout[-1200:]
        for s in left:
            assert not os.path.exists(os.path.join(run, f's{s}_full.score.json')), s
        # the hint, as printed, confirms every close seed
        r = check(base + shlex.split(HINT.search(lines[0]).group(1)), accept=True)
        doc = _summary(r.stdout)
        assert sorted(doc['confirmed']) == sorted(doc['close']) and len(doc['confirmed']) == n, doc
        assert 'leaves out' not in r.stdout, r.stdout[-900:]
        for s in left:
            evidence(os.path.join(run, f's{s}_full.score.json'), f'seed {s} full score')
    print('  PASS: the NOTE and --confirm close name the cap and the seeds it leaves out; '
          'the printed options confirm them all')


def main():
    test_confirm_all()
    test_cap_named()
    print('test_1473_judge_confirm_cap: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
