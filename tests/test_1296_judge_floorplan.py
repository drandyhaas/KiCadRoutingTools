#!/usr/bin/env python3
"""The seed judge ranks a seed with floorplan errors below every clean seed (#1296).

place_simple named the judged seeds by `final.cost` among the seeds whose
`illegal` list was empty, and quick_score graded only the route. A seed that
check_floorplan grades with an ERROR (a decap past its pin limit) cannot be
DONE as placed, and the limit it breaks lowers its cost, so it was named
first and kept: gp14 board 05 judged [8, 7, 3, 4, 9, 5] with s8 three decaps
past their 34.08 mm limit; quick_score gave s8 blocking 1 / 107 vias, the
round's best, against blocking 3 for the clean s3.

Checks:
  1. `place_simple.judge_order` puts every floorplan-clean legal seed first
     (cheapest first), then the fewest errors, then the ungraded, and never
     an illegal seed; `_name_judged` writes that order and each seed's error
     count to judge.json and WARNS about a judged seed with errors;
  2. `quick_score --intent` grades the floorplan as board_score does (its
     errors in `blocking_by.floorplan`, summed into `blocking`), and lists
     it in `ungraded` when it is not given or could not run;
  3. `quick_score.judge_key` keeps the clean s3 (blocking 3) above s8
     (blocking 1 + 3 floorplan errors), and the GUI's `candidate_key` ranks
     score files the same way;
  4. end to end, `quick_score.py --intent` on a real board grades the
     floorplan (no traceback, `floorplan` graded, not in `ungraded`).
"""
import argparse
import contextlib
import io
import json
import os
import subprocess
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


def report(cost, errors='clean', illegal=()):
    """A seed's summary as place_simple writes it."""
    fp = ({'errors': 0, 'rules': []} if errors == 'clean'
          else {'errors': None, 'reason': 'check_floorplan.py exited 3'} if errors is None
          else {'errors': errors, 'rules': ['decap_pin_distance']})
    return {'final': {'cost': cost}, 'illegal': list(illegal), 'refused': {},
            'floorplan': fp}


# board 05's round 1 as the issue measured it (s8: 3 decaps past their pin
# limit, s7: 1), the other seeds clean, one ungraded and one illegal
ROUND = {8: report(5329.5, 3), 7: report(5400.0, 1), 3: report(5500.0),
         4: report(5600.0), 9: report(5700.0), 5: report(5800.0),
         0: report(5900.0), 1: report(5000.0, None), 2: report(4000.0, illegal=['U9'])}


def test_judge_order():
    order = place_simple.judge_order(ROUND)
    assert order == [3, 4, 9, 5, 0, 7, 8, 1], order
    # cost alone, as before, named the broken seeds first
    by_cost = [s for _c, s in sorted((r['final']['cost'], s) for s, r in ROUND.items()
                                     if not r['illegal'])]
    assert by_cost[:2] == [1, 8], by_cost
    print(f'  PASS: judge order {order} (by cost alone it was {by_cost})')


def test_name_judged_writes_and_warns():
    with tempfile.TemporaryDirectory() as td:
        a = argparse.Namespace(judge=6, out=os.path.join(td, 's{seed}.kicad_pcb'))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            place_simple._name_judged(a, sorted(ROUND), ROUND)
        doc = json.load(open(os.path.join(td, 'judge.json'), encoding='utf-8'))
    out = buf.getvalue()
    assert doc['judge'] == [3, 4, 9, 5, 0, 7], doc['judge']
    assert doc['floorplan']['8'] == 3 and doc['floorplan']['3'] == 0, doc['floorplan']
    assert doc['floorplan']['1'] is None and doc['illegal'] == [2], doc
    assert 'WARNING: seed 7 is judged with 1 floorplan error(s)' in out, out
    assert 'seed 3 is judged' not in out, out
    print('  PASS: judge.json carries the order and each seed\'s floorplan; a judged '
          'seed with errors is warned about')


def scored(conn_blocking, fp):
    """quick_score.quick_score() with check_connected and check_floorplan
    canned: `conn_blocking` broken joins, `fp` the floorplan component."""
    saved = (board_score.score_connectivity, board_score.quality, board_score.score_floorplan)
    board_score.score_connectivity = lambda root, b: {
        'ran': True, 'count': conn_blocking, 'unrouted': 0, 'broken': conn_blocking,
        'unrouted_net_names': [], 'poured_nets': [],
        'broken_detail': {'/SIG': {'joins_needed': conn_blocking}}}
    board_score.quality = lambda b, pcb=None: {'vias': 100, 'copper_mm': 900.0, 'segments': 2000}
    board_score.score_floorplan = lambda root, b, intent, tmp: fp
    try:
        return quick_score.quick_score('fast.kicad_pcb', intent='intent.json' if fp else None)
    finally:
        (board_score.score_connectivity, board_score.quality,
         board_score.score_floorplan) = saved


def test_quick_score_grades_the_floorplan():
    s8 = scored(1, {'ran': True, 'count': 3, 'violations': ['decap_pin_distance'] * 3})
    assert s8['blocking'] == 4 and s8['blocking_by']['floorplan'] == 3, s8
    assert 'floorplan' not in s8['ungraded'], s8['ungraded']
    assert s8['components']['floorplan']['count'] == 3, s8['components']
    bare = scored(1, None)
    assert bare['blocking'] == 1 and 'floorplan' not in bare['blocking_by'], bare
    assert 'floorplan' in bare['ungraded'], bare['ungraded']
    failed = scored(1, {'ran': False, 'reason': 'check_floorplan.py exited 3: x', 'count': None})
    assert failed['blocking'] == 1 and 'floorplan' in failed['ungraded'], failed
    assert 'exited 3' in failed['components']['floorplan']['reason'], failed['components']
    print('  PASS: --intent sums the floorplan errors into blocking; without it, or '
          'when it cannot run, the floorplan is listed ungraded')


def test_judge_key_and_gui_agree():
    s8 = scored(1, {'ran': True, 'count': 3, 'violations': ['decap_pin_distance'] * 3})
    s3 = dict(scored(3, {'ran': True, 'count': 0, 'violations': []}),
              quality={'vias': 140, 'copper_mm': 990.0, 'segments': 2100})
    s4 = dict(scored(3, {'ran': True, 'count': 0, 'violations': []}),
              quality={'vias': 120, 'copper_mm': 990.0, 'segments': 2100})
    old = scored(2, None)            # a score written without --intent
    docs = {8: s8, 3: s3, 4: s4, 6: old}
    by_judge = sorted(docs, key=lambda s: quick_score.judge_key(docs[s]))
    assert by_judge == [6, 4, 3, 8], by_judge
    with tempfile.TemporaryDirectory() as td:
        rows = []
        for s, d in docs.items():
            p = os.path.join(td, f's{s}_fast.score.json')
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(d, fh)
            sc = placement_run._read_score(p)
            assert sc is not None, (s, d)
            rows.append(dict(zip(('blocking', 'vias', 'copper_mm', 'segments', 'floorplan',
                                  'joins'), sc), seed=s))
    by_gui = [r['seed'] for r in sorted(rows, key=placement_run.candidate_key)]
    assert by_gui == by_judge, (by_gui, by_judge)
    print(f'  PASS: judge_key and the GUI both rank {by_judge}: the clean seeds above s8')


def test_cli_end_to_end():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        intent = os.path.join(td, 'intent.json')
        check([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), BOARD,
               '--emit-intent', intent], accept=True)
        evidence(intent, 'emitted intent')
        out = os.path.join(td, 'score.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool('quick_score.py'), BOARD,
                            '--intent', intent, '--json', out],
                           capture_output=True, text=True, encoding='utf-8', errors='replace',
                           cwd=ROOT)
        assert 'Traceback' not in r.stdout + r.stderr, r.stdout + r.stderr
        assert r.returncode in (0, 4), (r.returncode, r.stdout[-800:], r.stderr[-800:])
        doc = json.load(open(evidence(out, 'quick_score JSON'), encoding='utf-8'))
    fp = doc['blocking_by'].get('floorplan')
    assert isinstance(fp, int), doc['blocking_by']
    assert 'floorplan' not in doc['ungraded'], doc['ungraded']
    assert doc['components']['floorplan']['ran'] is True, doc['components']['floorplan']
    print(f'  PASS: quick_score --intent graded the floorplan of a real board '
          f'({fp} error(s), blocking {doc["blocking"]})')


def main():
    test_judge_order()
    test_name_judged_writes_and_warns()
    test_quick_score_grades_the_floorplan()
    test_judge_key_and_gui_agree()
    test_cli_end_to_end()
    print('test_1296_judge_floorplan: all passed')


if __name__ == '__main__':
    main()
