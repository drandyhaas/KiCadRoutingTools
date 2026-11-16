#!/usr/bin/env python3
"""place_simple's `judge seeds` line counts the run it heads (#1824).

The parenthesis was a fixed string with `--judge`'s value in it: a run of 2
seeds, neither graded clean, printed "(the 6 legal seeds of lowest cost,
buildable and floorplan-clean first)" word for word as a run whose seeds
both graded clean did, and judge_seeds then routed neither seed.

Checks, on hand-made summaries (no anneal):
  1. two seeds with a floorplan error each: the line says none of the 2
     graded clean and that judge_seeds routes none of them without --seeds,
     never "the 6 legal seeds"; each per-seed WARNING gives the run's
     counts ("0 of 2 legal seed(s)"), not "fewer than 6";
  2. two clean seeds (control): the line says each graded clean, and
     judge.json's `judge` is [0, 1] in both runs -- the order is unchanged;
  3. judge.json's `clean` is the judged seeds with both counts 0;
  4. a mixed run counts each kind; no legal seed still prints
     "judge seeds none".
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
for _d in ('py_router', 'py_tools', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import place_simple                                           # noqa: E402


def summary(cost, fp=0, asm=0, illegal=()):
    """A seed's summary as place_simple writes it."""
    return {'final': {'cost': cost}, 'illegal': list(illegal), 'refused': {},
            'floorplan': ({'errors': fp, 'rules': ['proximity'] if fp else []}
                          if fp is not None else
                          {'errors': None, 'reason': 'check_floorplan.py exited 3'}),
            'assembly': {'blocking': asm, 'gating': ['courtyard_blocking'] if asm else []}}


def name(summaries, judge=6):
    with tempfile.TemporaryDirectory() as td:
        a = argparse.Namespace(judge=judge, out=os.path.join(td, 's{seed}.kicad_pcb'))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            place_simple._name_judged(a, sorted(summaries), summaries)
        doc = json.load(open(os.path.join(td, 'judge.json'), encoding='utf-8'))
    out = buf.getvalue()
    banner = next(ln for ln in out.splitlines() if ln.startswith('place_simple: judge seeds'))
    return banner, out, doc


def test_none_clean():
    banner, out, doc = name({0: summary(7683.9, fp=1), 1: summary(8554.5, fp=1)})
    assert 'the 6 legal seeds' not in banner, banner
    assert '2 of 2 legal seed(s), --judge 6' in banner, banner
    assert 'NONE graded buildable and floorplan-clean' in banner, banner
    assert 'judge_seeds routes none of them unless --seeds names them' in banner, banner
    assert 'fewer than 6' not in out, out
    assert out.count('(0 of 2 legal seed(s) buildable and floorplan-clean)') == 2, out
    assert doc['judge'] == [0, 1] and doc['clean'] == [], doc
    print(f'  PASS: no clean seed: {banner}')


def test_all_clean():
    banner, out, doc = name({0: summary(6494.1), 1: summary(7386.8)})
    assert 'the 6 legal seeds' not in banner, banner
    assert '2 of 2 legal seed(s), --judge 6; each graded buildable and floorplan-clean' \
        in banner, banner
    assert 'WARNING' not in out, out
    assert doc['judge'] == [0, 1] and doc['clean'] == [0, 1], doc
    print(f'  PASS: every seed clean: {banner}')


def test_mixed_and_cut():
    sums = {0: summary(100.0), 1: summary(50.0, asm=2), 2: summary(70.0, fp=None),
            3: summary(80.0), 4: summary(10.0, illegal=['U1'])}
    banner, out, doc = name(sums, judge=3)
    # legal seeds 0..3, judged the 3 first: clean 3 and 0, then seed 1
    # (graded, 2 defects) before the ungraded seed 2, which the cut leaves out
    assert doc['judge'] == [3, 0, 1], doc
    assert doc['clean'] == [3, 0], doc
    assert '3 of 4 legal seed(s), --judge 3; 2 graded buildable and floorplan-clean, ' \
        '1 with a placement defect' in banner, banner
    assert '(2 of 4 legal seed(s) buildable and floorplan-clean)' in out, out
    sums[1] = summary(50.0, fp=None)
    banner, _out, doc = name(sums, judge=4)
    assert doc['clean'] == [3, 0] and '2 not graded' in banner, (doc, banner)
    print(f'  PASS: a mixed run counts each kind: {banner}')


def test_no_legal_seed():
    banner, _out, doc = name({0: summary(1.0, illegal=['U1']), 1: summary(2.0, illegal=['U1'])})
    assert banner.startswith('place_simple: judge seeds none'), banner
    assert doc['judge'] == [] and doc['clean'] == [], doc
    print(f'  PASS: no legal seed: {banner}')


if __name__ == '__main__':
    test_none_clean()
    test_all_clean()
    test_mixed_and_cut()
    test_no_legal_seed()
    print('ALL PASS')
