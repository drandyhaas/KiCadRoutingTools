#!/usr/bin/env python3
"""The seed judge ranks a NOT BUILDABLE seed below every buildable one, as it
does a seed with floorplan errors (#1296's follow-up).

The anneal reports `illegal []` for a seed check_assembly grades NOT
BUILDABLE: its overlap term does not model the courtyard pair the gate
blocks. On multichannel_mixer every seed placed on 1021 was NOT BUILDABLE on
its potentiometers' bodies (C31/RV12, C2/RV10, ...), and the judge, ranking
by floorplan errors and cost, named them as freely as a buildable seed. No
route makes such a placement buildable, so it cannot be DONE as placed.

Checks:
  1. `place_simple.judge_order` sums each seed's assembly count with its
     floorplan errors (board_score sums both into `blocking`): a buildable
     seed ranks above a cheaper NOT BUILDABLE one; a report written before
     the assembly grade counts 0 (the old order holds); one check_assembly
     could not grade ranks with the ungraded;
  2. `_name_judged` writes each seed's `assembly` count and the `baseline`
     board to judge.json, and WARNS about a judged NOT BUILDABLE seed;
  3. `quick_score(..., baseline=)` grades the assembly as board_score
     --baseline does (`blocking_by.assembly`, summed into `blocking`, out of
     `ungraded`), and `judge_key` and the GUI's `candidate_key` rank it with
     the floorplan errors;
  4. end to end, `quick_score.py --baseline` on a real board grades the
     assembly (no traceback, `assembly` not in `ungraded`).
"""
import argparse
import contextlib
import io
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('', 'py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin'):
    _p = os.path.join(ROOT, _d) if _d else ROOT
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import board_score                                            # noqa: E402
import place_simple                                           # noqa: E402
import placement_run                                          # noqa: E402
import quick_score                                            # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def report(cost, fp=0, asm='buildable'):
    """A seed's summary as place_simple writes it; asm=None: not graded,
    asm='absent': a report from before the assembly grade."""
    r = {'final': {'cost': cost}, 'illegal': [], 'refused': {},
         'floorplan': {'errors': fp, 'rules': []}}
    if asm == 'buildable':
        r['assembly'] = {'blocking': 0, 'buildable': True, 'fired': []}
    elif asm is None:
        r['assembly'] = {'blocking': None, 'reason': 'check_assembly rc 2'}
    elif asm != 'absent':
        r['assembly'] = {'blocking': asm, 'buildable': False,
                         'fired': ['courtyard_blocking']}
    return r


def test_judge_order():
    reps = {0: report(100, asm=4), 1: report(300), 2: report(200, fp=1),
            3: report(150, asm=None), 4: report(250)}
    got = place_simple.judge_order(reps)
    assert got == [4, 1, 2, 0, 3], got
    old = {0: report(100, asm='absent'), 1: report(50, fp=2, asm='absent')}
    assert place_simple.judge_order(old) == [0, 1], place_simple.judge_order(old)
    print('  PASS: buildable and clean first, then the fewest defects; an '
          'ungraded assembly last; an old report keeps its order')


def test_judge_json_and_warning():
    with tempfile.TemporaryDirectory() as td:
        a = argparse.Namespace(judge=2, out=os.path.join(td, 's{seed}.kicad_pcb'),
                               board=os.path.join(td, 'pile.kicad_pcb'))
        sums = {0: dict(report(100, asm=4), start='scattered'),
                1: dict(report(300, asm=2), start='scattered')}
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            place_simple._name_judged(a, [0, 1], sums)
        doc = json.load(open(os.path.join(td, 'judge.json'), encoding='utf-8'))
    assert doc['assembly'] == {'0': 4, '1': 2}, doc
    assert doc['baseline'] == os.path.abspath(a.board), doc
    assert doc['judge'] == [1, 0], doc
    out = buf.getvalue()
    assert 'WARNING: seed 1 is judged NOT BUILDABLE (courtyard_blocking)' in out, out
    print('  PASS: judge.json carries each seed\'s assembly count and the baseline; '
          'a judged NOT BUILDABLE seed is warned about')


def test_quick_score_and_keys():
    real = board_score.score_assembly
    board_score.score_assembly = lambda root, board, intent, tmp, **kw: {
        'ran': True, 'count': 3, 'count_basis': 'blocking (3); NOT BUILDABLE',
        'buildable': False, 'baseline_seen': kw.get('baseline')}
    try:
        sc = quick_score.quick_score(BOARD, baseline=BOARD)
        plain = quick_score.quick_score(BOARD)
    finally:
        board_score.score_assembly = real
    assert sc['blocking_by']['assembly'] == 3, sc['blocking_by']
    assert sc['blocking'] == plain['blocking'] + 3, (sc['blocking'], plain['blocking'])
    assert 'assembly' not in sc['ungraded'], sc['ungraded']
    assert 'assembly.courtyard_gating' not in sc['ungraded'], sc['ungraded']
    assert 'assembly' in plain['ungraded'], plain['ungraded']
    # judge_key: the assembly count ranks with the floorplan errors
    clean = {'blocking': 2, 'blocking_by': {'floorplan': 0, 'assembly': 0},
             'joins_needed': 2, 'quality': {'vias': 90, 'copper_mm': 900.0, 'segments': 800}}
    stuck = {'blocking': 1, 'blocking_by': {'floorplan': 0, 'assembly': 1},
             'joins_needed': 0, 'quality': {'vias': 40, 'copper_mm': 400.0, 'segments': 300}}
    docs = {'clean': clean, 'stuck': stuck}
    by_key = sorted(docs, key=lambda n: quick_score.judge_key(docs[n]))
    assert by_key == ['clean', 'stuck'], by_key
    with tempfile.TemporaryDirectory() as td:
        rows = []
        for name, d in docs.items():
            p = os.path.join(td, f'{name}.score.json')
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(d, fh)
            sc = placement_run._read_score(p)
            rows.append(dict(zip(('blocking', 'vias', 'copper_mm', 'segments',
                                  'floorplan', 'joins'), sc), name=name))
    gui = [r['name'] for r in sorted(rows, key=placement_run.candidate_key)]
    assert gui == by_key, gui
    print('  PASS: quick_score --baseline sums the assembly into blocking; judge_key '
          'and the GUI rank a NOT BUILDABLE seed below a buildable one')


def test_end_to_end():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'score.json')
        check([sys.executable, '-X', 'utf8', tool('quick_score.py'), BOARD,
               '--baseline', BOARD, '--json', out], refuse='UNGRADED', code=4)
        doc = json.load(open(evidence(out, 'quick_score JSON'), encoding='utf-8'))
    assert 'assembly' not in doc['ungraded'], doc['ungraded']
    assert isinstance(doc['blocking_by'].get('assembly'), int), doc['blocking_by']
    print(f"  PASS: quick_score.py --baseline on a real board grades the assembly "
          f"(assembly {doc['blocking_by']['assembly']})")


def main():
    test_judge_order()
    test_judge_json_and_warning()
    test_quick_score_and_keys()
    test_end_to_end()
    print('test_1296_judge_assembly: all passed')


if __name__ == '__main__':
    main()
