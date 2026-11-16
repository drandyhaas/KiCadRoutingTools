#!/usr/bin/env python3
"""#1826: judge_seeds' close band grows with the best seed's level-2 blocking.

The band was `CLOSE_BLOCKING` = 1 net whatever the best's blocking: two
seeds at 60 and 62 blocked nets at level 2 got a winner and no NOTE, and the
full route of the same seeds (2 and 1 blocked) reversed the order. Now the
band is max(1, ceil(0.15 x best blocking)); inside a band so widened
joins_needed is compared the same way and copper is not, the run says every
judged seed leaves N-M nets blocked at level 2, and JSON_SUMMARY names the
seeds it cannot separate (`not_separable`). The GUI's candidates table says
the same from the same function.

Checks (hand-made scores, no route):
  1. 60 and 62 with the same defects are close; `not_separable` names both;
  2. control: 0 and 1 are close, 0 and 2 are not, and no band is widened; a
     seed with a floorplan error is never close to a clean one; at 60, a seed
     far behind in joins_needed is not close;
  3. the GUI's `scan_candidates` marks 60/62 "best, not separable" / "not
     separable", and 0/2 "best" / "routed";
  4. end to end through `main` with a stub chain (copy_board): on an
     unrouted fixture, where every seed blocks tens of nets, the line and
     the JSON key are present and the best-seed line follows the line; on a
     routed fixture (blocking 0) they are absent.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, HERE)

import judge_seeds                                            # noqa: E402
import placement_run                                          # noqa: E402
from copy_board import copy_board                             # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

UNROUTED = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')


def _pad_part(ref, x):
    return (f'  (footprint "t:P" (layer "F.Cu") (at {x} 10)\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu")'
            ' (net 1 "N1")))\n')


def routed_board(path):
    """Two one-pad parts joined by a track: a board that routes (blocking 0)."""
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n  (net 1 "N1")\n'
                 '  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1)'
                 ' (type default)) (layer "Edge.Cuts"))\n'
                 + _pad_part('U1', 10) + _pad_part('U2', 20)
                 + '  (segment (start 10 10) (end 20 10) (width 0.25) (layer "F.Cu")'
                   ' (net 1))\n)\n')
    return evidence(path, 'the routed fixture')
RUNNER = tool('judge_seeds.py')


def res(seed, blk, joins=None, cu=1000.0, fp=0):
    return {'seed': seed, 'score': {
        'blocking': blk, 'joins_needed': blk if joins is None else joins,
        'blocking_by': {'floorplan': fp, 'assembly': 0},
        'quality': {'copper_mm': cu, 'vias': 100, 'segments': 900}}}


def test_band():
    ranked = [res(1, 60, cu=1544.5), res(0, 62, cu=1373.2)]
    assert judge_seeds.close_seeds(ranked) == [1, 0], judge_seeds.close_seeds(ranked)
    assert judge_seeds.not_separable(ranked) == ([1, 0], (60, 62)), \
        judge_seeds.not_separable(ranked)
    print('  PASS: 60 and 62 blocked are close, and not separable at level 2')


def test_controls():
    assert judge_seeds.close_seeds([res(0, 0), res(1, 1)]) == [0, 1]
    assert judge_seeds.close_seeds([res(0, 0), res(1, 2)]) == [0]
    assert judge_seeds.not_separable([res(0, 0), res(1, 1)]) == ([], None)
    assert judge_seeds.close_seeds([res(0, 60), res(1, 60, fp=1)]) == [0]
    assert judge_seeds.close_seeds([res(0, 60, joins=60), res(1, 61, joins=80)]) == [0]
    # a widened band with only the best in it: said, but nothing to separate
    assert judge_seeds.not_separable([res(0, 60), res(1, 80)]) == ([], (60, 80))
    print('  PASS: the 1-net band holds where seeds route; a defect or far joins part them')


def _scored_dir(td, blockings):
    for s, b in enumerate(blockings):
        for suffix, text in (('.kicad_pcb', f'(kicad_pcb s{s})\n'), ('.json', '{}'),
                             ('_fast.kicad_pcb', f'(kicad_pcb s{s} fast)\n')):
            with open(os.path.join(td, f's{s}{suffix}'), 'w') as fh:
                fh.write(text)
        with open(os.path.join(td, f's{s}_fast.score.json'), 'w') as fh:
            json.dump({'blocking': b, 'joins_needed': b, 'blocking_by': {'floorplan': 0},
                       'quality': {'vias': 100, 'copper_mm': 1000.0 + s,
                                   'segments': 900}}, fh)


def test_gui_table():
    with tempfile.TemporaryDirectory() as td:
        _scored_dir(td, [62, 60])
        by = {r['seed']: r['status'] for r in placement_run.scan_candidates(td)}
        assert by == {1: 'best, not separable', 0: 'not separable'}, by
    with tempfile.TemporaryDirectory() as td:
        _scored_dir(td, [0, 2])
        by = {r['seed']: r['status'] for r in placement_run.scan_candidates(td)}
        assert by == {0: 'best', 1: 'routed'}, by
    print('  PASS: the candidates table says the same: not separable at 60/62, best at 0/2')


def _summary(out):
    line = next(x for x in out.splitlines() if x.startswith('JSON_SUMMARY: '))
    return json.loads(line[len('JSON_SUMMARY: '):])


def _judge(td, board, tag):
    run = os.path.join(td, tag)
    os.makedirs(run)
    for s in (0, 1):
        copy_board(board, os.path.join(run, f's{s}.kicad_pcb'))
        evidence(os.path.join(run, f's{s}.kicad_pcb'), f'seed {s} board')
    judge = os.path.join(run, 'judge.json')
    with open(judge, 'w') as fh:
        json.dump({'judge': [0, 1], 'illegal': []}, fh)
    chain = os.path.join(run, 'chain.txt')
    with open(chain, 'w') as fh:
        fh.write('python3 -X utf8 py_router/copy_board.py {in} {out}\n')
    r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain], accept=True)
    return r.stdout, _summary(r.stdout)


def test_end_to_end():
    evidence(UNROUTED, 'the unrouted fixture')
    with tempfile.TemporaryDirectory() as td:
        routed = routed_board(os.path.join(td, 'routed.kicad_pcb'))
        out, doc = _judge(td, UNROUTED, 'unrouted')
        blk = [doc['seeds'][k]['blocking'] for k in ('0', '1')]
        assert min(blk) >= 10, ('the unrouted fixture must block tens of nets', blk)
        assert doc['not_separable'] == doc['ranking'] and len(doc['not_separable']) == 2, doc
        lines = out.splitlines()
        i = next(k for k, x in enumerate(lines) if x.startswith('judge_seeds: every judged seed '
                                                              f'leaves {blk[0]}'))
        j = next(k for k, x in enumerate(lines) if x.startswith('judge_seeds: best seed'))
        assert i < j and 'not separable: seeds' in lines[i], lines[i:j + 1]
        assert 'level 2 does not separate it from seed(s)' in lines[j], lines[j]
        out, doc = _judge(td, routed, 'routed')
        assert all(doc['seeds'][k]['blocking'] == 0 for k in ('0', '1')), doc
        assert doc['not_separable'] == [], doc
        assert 'every judged seed leaves' not in out and 'does not separate' not in out, \
            out[-800:]
    print('  PASS: through main: the line and not_separable on an unrouted fixture, '
          'neither on a routed one')


if __name__ == '__main__':
    test_band()
    test_controls()
    test_gui_table()
    test_end_to_end()
    print('ALL PASS')
