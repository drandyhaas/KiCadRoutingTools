#!/usr/bin/env python3
"""The #554 reach measurement, re-derived and diffed against its committed baseline.

`tests/stress/relocation_reach.py` decides whether #554's whole premise holds: does
letting neighbours yield, in preserved relative order, buy a block materially more
travel toward its connectivity target than freezing them? The recorded answer is
MECHANISM HOLDS, and numbers that live only in a doc drift (#694). So this test
re-derives two declared cells from scratch and diffs every numeric key.

It guards the result in BOTH directions, which is the half `test_553_recall_regen.py`
did not need and this one does, because the recorded outcome here is POSITIVE:

  * a positive result cannot quietly decay into a null -- the recorded material-gain
    cells and the median gain must hold;
  * and a null cannot quietly become a claim -- if the numbers grow, the test fails
    and forces a deliberate re-record rather than letting the doc go stale in the
    other direction.

Three verdict-independent structural assertions come first, because each is a way
the measurement could be void while every number in it looks fine:

  1. `identity_violations == 0`. Every number rests on `s = 0` -- the incumbent
     board -- satisfying the constraint system. It shipped false once (the wall
     edges were not clamped to what each part already has) and the symptom was a
     reach SMALLER than the same block's frozen slide.
  2. The paired arms must be the SAME rule. `frozen_mm` and `reach_mm` both come
     from `relocate.reach` over one graph; `slide_frozen_mm` is a looser, different
     rule and is never compared with them.
  3. `reach_mm >= frozen_mm` is TRUE BY CONSTRUCTION (pinning only removes
     variables), so the summary must not present a win rate as the finding. The
     verdict string is asserted to carry that caveat.

Run: python3 -X utf8 tests/test_554_reach_regen.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

BASELINE = os.path.join(ROOT, 'tests', '554_relocation_reach_baseline.json')
STUDY = os.path.join(ROOT, 'tests', 'stress', 'relocation_reach.py')

#: Re-derived live. esp_prog is the cheapest board that derives blocks at all and
#: is the one cell where BOTH arms move, so it exercises the ratio; tigard's
#: `decap:U3` is a 0 -> moves cell, which is the shape the whole feature is for.
CELLS = (('esp_prog', 'esp_prog/decap:U1'),
         ('tigard', 'tigard/decap:U3'))

TOL = 1e-6

#: The recorded verdict. Both bounds are asserted, so neither direction can drift
#: silently. Re-record with the study's own --out when a change moves them, and say
#: in the commit what moved.
EXPECT_VERDICT = 'MECHANISM HOLDS'
MIN_MATERIAL_CELLS = 11
MIN_MATERIAL_BOARDS = 6
MAX_MATERIAL_CELLS = 20      # a large jump is a finding, not a pass
GAIN_MEDIAN_BAND = (0.10, 1.00)
GAIN_MAX_BAND = (10.0, 40.0)

_fails = []


def check(cond, msg):
    if not cond:
        _fails.append(msg)
    return cond


def load_baseline():
    with open(BASELINE, encoding='utf-8') as fh:
        return json.load(fh)


def t_the_baseline_is_shaped_like_one(base):
    check(isinstance(base.get('cells'), dict) and base['cells'],
          'baseline has no cells: a summary-only baseline compares nothing, which '
          'is how #553 shipped a file that passed while its signal reversed')
    check(isinstance(base.get('summary'), dict), 'baseline has no summary')
    for key, cell in base['cells'].items():
        if cell.get('refusal'):
            continue
        for k in ('want_mm', 'frozen_mm', 'reach_mm', 'slide_frozen_mm',
                  'control_ok', 'identity_violations'):
            check(k in cell, 'baseline cell %s is missing %s' % (key, k))


def t_identity_holds_on_every_cell(base):
    bad = [k for k, c in base['cells'].items() if c.get('identity_violations')]
    check(not bad,
          'identity_violations non-zero on %r: the incumbent board does not '
          'satisfy its own constraint system, so every number in this baseline '
          'describes a board that does not exist' % (bad[:3],))


def t_the_arms_are_paired_and_ordered(base):
    """reach >= frozen on every cell -- and this is a THEOREM, not a result."""
    for k, c in base['cells'].items():
        if c.get('refusal'):
            continue
        check(c['reach_mm'] >= c['frozen_mm'] - TOL,
              'cell %s has reach %.4f < frozen %.4f. Pinning only REMOVES '
              'variables, so this is impossible unless the two arms are not the '
              'same graph or the same rule.' % (k, c['reach_mm'], c['frozen_mm']))


def t_the_verdict_does_not_sell_a_theorem_as_a_finding(base):
    v = base['summary'].get('verdict') or ''
    check(EXPECT_VERDICT in v,
          'recorded verdict changed: %r does not contain %r' % (v, EXPECT_VERDICT))
    check('BY CONSTRUCTION' in v,
          'the verdict dropped the caveat that reach >= frozen is guaranteed. '
          'Without it the reader takes a 24-of-24 ordering for evidence.')
    check('ROUTES' in v,
          'the verdict dropped the statement that this says nothing about '
          'routing -- which is the one thing a reader will assume it measured')


def t_the_recorded_result_is_bounded_in_both_directions(base):
    s = base['summary']
    n = s['cells_material']
    check(MIN_MATERIAL_CELLS <= n <= MAX_MATERIAL_CELLS,
          'cells_material %d outside [%d, %d]: the mechanism result moved. If it '
          'GREW that is a finding and the baseline must be re-recorded '
          'deliberately; if it SHRANK the mechanism is weakening and #554 needs '
          'the withdrawal rule, not a relaxed bound.'
          % (n, MIN_MATERIAL_CELLS, MAX_MATERIAL_CELLS))
    check(len(s['cells_material_boards']) >= MIN_MATERIAL_BOARDS,
          'material gain now spans %d board(s), under the recorded %d: a result on '
          'fewer boards is a weaker result, not the same one'
          % (len(s['cells_material_boards']), MIN_MATERIAL_BOARDS))
    lo, hi = GAIN_MEDIAN_BAND
    check(lo <= s['gain_mm_median'] <= hi,
          'gain_mm_median %.4f outside [%.2f, %.2f]' % (s['gain_mm_median'], lo, hi))
    lo, hi = GAIN_MAX_BAND
    check(lo <= s['gain_mm_max'] <= hi,
          'gain_mm_max %.4f outside [%.2f, %.2f]' % (s['gain_mm_max'], lo, hi))
    check(s['cells_both_zero'] > 0,
          'no both-zero cells recorded. Boards where NEITHER arm moves are the '
          'honest limit of this feature and dropping them would turn the table '
          'into a highlight reel')


def t_refusals_are_named_not_counted(base):
    ref = base['summary'].get('refused') or []
    check(ref, 'no refusals recorded: glasgow_revC carries KiCad-locked footprints '
               'and is the board that exercises that path')
    for r in ref:
        check(r.get('refusal') and not r['refusal'].isdigit(),
              'refusal %r is not a named reason' % (r,))


def t_two_cells_re_derive_exactly(base):
    """Run the study for real and diff. The only part that costs seconds."""
    for board, key in CELLS:
        want = base['cells'].get(key)
        if not check(want is not None, 'baseline has no cell %s' % key):
            continue
        with tempfile.TemporaryDirectory() as td:
            r = subprocess.run(
                [sys.executable, '-X', 'utf8', STUDY, '--boards', board,
                 '--top-k', '3', '--out', td],
                cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                errors='replace', timeout=RUN_ALL_TIMEOUT)
            if not check(r.returncode == 0,
                         'study exited %s for %s:\n%s'
                         % (r.returncode, board, (r.stderr or r.stdout)[-1500:])):
                continue
            rows = [json.loads(x) for x in
                    open(os.path.join(td, 'rows.jsonl'), encoding='utf-8')
                    if x.strip()]
        got = {'%s/%s' % (x['board'], x['block']): x for x in rows}.get(key)
        if not check(got is not None,
                     're-run produced no cell %s (blocks: %r)'
                     % (key, sorted(got or {})[:4] if got else
                        [x['block'] for x in rows][:4])):
            continue
        for k, v in want.items():
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                check(got.get(k) == v,
                      '%s.%s: baseline %r, re-derived %r' % (key, k, v, got.get(k)))
            else:
                g = got.get(k)
                if not check(isinstance(g, (int, float)),
                             '%s.%s: re-derived %r is not numeric' % (key, k, g)):
                    continue
                check(abs(g - v) <= TOL,
                      '%s.%s drifted: baseline %.6f, re-derived %.6f'
                      % (key, k, v, g))


def t_the_study_self_test_passes():
    r = subprocess.run([sys.executable, '-X', 'utf8', STUDY, '--self-test'],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=120)
    check(r.returncode == 0,
          'the study self-test failed, so its numbers are not trustworthy:\n%s'
          % (r.stdout + r.stderr)[-1500:])


def main():
    base = load_baseline()
    t_the_baseline_is_shaped_like_one(base)
    t_identity_holds_on_every_cell(base)
    t_the_arms_are_paired_and_ordered(base)
    t_the_verdict_does_not_sell_a_theorem_as_a_finding(base)
    t_the_recorded_result_is_bounded_in_both_directions(base)
    t_refusals_are_named_not_counted(base)
    t_the_study_self_test_passes()
    t_two_cells_re_derive_exactly(base)

    for f in _fails:
        print('FAIL: %s' % f)
    print('test_554_reach_regen: %s (%d checks failed)'
          % ('FAIL' if _fails else 'PASS', len(_fails)))
    return 1 if _fails else 0


if __name__ == '__main__':
    sys.exit(main())
