#!/usr/bin/env python3
"""The #554 reach measurement, RE-DERIVED IN FULL and diffed against its baseline.

`tests/stress/relocation_reach.py` decides whether #554's premise holds: does
letting neighbours yield, in preserved relative order, buy a block materially more
travel than freezing them? The recorded answer is MECHANISM HOLDS, and numbers that
live only in a doc drift (#694).

THE WHOLE SWEEP IS RE-RUN HERE, and every structural check reads the RE-DERIVED
result, not the committed file. That is not thoroughness for its own sake -- it is
the fix for a measured hole. An earlier version of this test sampled two cells and
checked the other twenty-two against the baseline, i.e. against a committed
constant, so five of its seven checks were tautologies. Demonstrated by mutating
`relocate.reach` to cap at 3.0 mm: both sampled cells are under 3 mm, so they came
back byte-identical and this test printed PASS while the real sweep moved

    gain_mm_max                      16.66  ->  2.87
    ratio_median_where_frozen_moves   5.09  ->  2.80
    cells_material                      11  ->  10

A regen test that samples must re-derive the AGGREGATES too, or it guards only the
cells it happened to name. The full sweep costs about 6 seconds and needs no
router, so there was never a reason to sample.

It guards the result in BOTH directions, which is the half `test_553_recall_regen.py`
did not need and this one does, because the recorded outcome here is POSITIVE:

  * a positive result cannot quietly decay into a null -- the material-cell count,
    the board spread and the gain bands must hold;
  * and a null cannot quietly become a claim -- if the numbers grow past their
    band the test fails and forces a deliberate re-record.

Four verdict-independent structural checks come first, because each is a way the
measurement could be void while every number in it looks fine:

  1. `identity_violations == 0`. Every number rests on `s = 0` -- the incumbent
     board -- satisfying the constraint system. It shipped false once (the wall
     edges were not clamped to what each part already has) and the symptom was a
     reach SMALLER than the same block's frozen slide.
  2. The paired arms must be the SAME rule, so `reach_mm >= frozen_mm` everywhere.
  3. That ordering is a THEOREM (pinning only removes variables), so the verdict
     must not sell it as a finding.
  4. Boards that move in NEITHER arm, and blocks refused by name, must both be
     present -- they are the honest limit of the feature, and a table without them
     is a highlight reel.

Run: python3 -X utf8 tests/test_554_reach_regen.py
"""
import json
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

FULL_SWEEP_TIMEOUT = 420
TOL = 1e-6

#: The recorded verdict. Both bounds are asserted, so neither direction can drift
#: silently. Re-record with the study's own --out when a change moves them, and
#: say in the commit what moved and why.
EXPECT_VERDICT = 'MECHANISM HOLDS'
MIN_MATERIAL_CELLS = 11
MAX_MATERIAL_CELLS = 20      # a large jump is a finding, not a pass
MIN_MATERIAL_BOARDS = 6
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


def run_sweep():
    """The full sweep, from scratch. Returns {'cells': ..., 'summary': ...}."""
    with tempfile.TemporaryDirectory() as td:
        r = subprocess.run([sys.executable, '-X', 'utf8', STUDY, '--out', td],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace',
                           timeout=FULL_SWEEP_TIMEOUT)
        if r.returncode != 0:
            _fails.append('the sweep exited %s:\n%s'
                          % (r.returncode, (r.stderr or r.stdout)[-2000:]))
            return None
        rows = [json.loads(x) for x in
                open(os.path.join(td, 'rows.jsonl'), encoding='utf-8')
                if x.strip()]
        with open(os.path.join(td, 'summary.json'), encoding='utf-8') as fh:
            summary = json.load(fh)
    cells = {'%s/%s' % (x['board'], x['block']): x
             for x in rows if 'skipped' not in x}
    return {'cells': cells, 'summary': summary}


# --------------------------------------------------------------------------
# structural -- all against the RE-DERIVED result
# --------------------------------------------------------------------------

def t_identity_holds_on_every_cell(live):
    bad = [k for k, c in live['cells'].items() if c.get('identity_violations')]
    check(not bad,
          'identity_violations non-zero on %r: the incumbent board does not '
          'satisfy its own constraint system, so every number here describes a '
          'board that does not exist' % (bad[:3],))


def t_the_arms_are_paired_and_ordered(live):
    """reach >= frozen on every cell -- and this is a THEOREM, not a result."""
    for k, c in live['cells'].items():
        if c.get('refusal'):
            continue
        check(c['reach_mm'] >= c['frozen_mm'] - TOL,
              'cell %s has reach %.4f < frozen %.4f. Pinning only REMOVES '
              'variables, so this is impossible unless the two arms are not the '
              'same graph or the same rule.' % (k, c['reach_mm'], c['frozen_mm']))


def t_a_reported_reach_is_achievable(live):
    """Floored, never rounded up: the value is fed back as a dose ceiling.

    Rounding to nearest put 7 of 24 cells 3e-6..4e-5 mm PAST their own envelope,
    and HiGHS answered Infeasible for splitflap_driver/decap:U3 at its own
    published reach.
    """
    for k, c in live['cells'].items():
        for field in ('reach_mm', 'frozen_mm'):
            v = c.get(field)
            if v is None:
                continue
            check(abs(v * 10000 - round(v * 10000)) < 1e-6,
                  '%s.%s = %r is not on the 1e-4 grid the flooring produces'
                  % (k, field, v))


def t_the_verdict_does_not_sell_a_theorem_as_a_finding(live):
    v = live['summary'].get('verdict') or ''
    check(EXPECT_VERDICT in v,
          'the verdict changed: %r does not contain %r' % (v, EXPECT_VERDICT))
    check('BY CONSTRUCTION' in v,
          'the verdict dropped the caveat that reach >= frozen is guaranteed. '
          'Without it a reader takes a 24-of-24 ordering for evidence.')
    check('ROUTES' in v,
          'the verdict dropped the statement that this says nothing about '
          'routing -- the one thing a reader will assume it measured')


def t_the_result_is_bounded_in_both_directions(live):
    s = live['summary']
    n = s['cells_material']
    check(MIN_MATERIAL_CELLS <= n <= MAX_MATERIAL_CELLS,
          'cells_material %d outside [%d, %d]: the mechanism result MOVED. If it '
          'grew that is a finding and the baseline must be re-recorded '
          'deliberately; if it shrank the mechanism is weakening and #554 needs '
          'its withdrawal rule, not a relaxed bound.'
          % (n, MIN_MATERIAL_CELLS, MAX_MATERIAL_CELLS))
    check(len(s['cells_material_boards']) >= MIN_MATERIAL_BOARDS,
          'material gain now spans %d board(s), under the recorded %d: a result '
          'on fewer boards is a weaker result, not the same one'
          % (len(s['cells_material_boards']), MIN_MATERIAL_BOARDS))
    lo, hi = GAIN_MEDIAN_BAND
    check(lo <= s['gain_mm_median'] <= hi,
          'gain_mm_median %.4f outside [%.2f, %.2f]' % (s['gain_mm_median'], lo, hi))
    lo, hi = GAIN_MAX_BAND
    check(lo <= s['gain_mm_max'] <= hi,
          'gain_mm_max %.4f outside [%.2f, %.2f]' % (s['gain_mm_max'], lo, hi))


def t_the_numbers_the_docs_QUOTE_are_pinned(live):
    """The medians the module docstring and both docs pages state, by name.

    This exists because they drifted and nothing caught it. The prose said
    "median 0.1 mm against a median want of 6.1 mm over ten corpus boards"; the
    committed baseline says **0.00 and 10.36 over 9 boards**, and neither quoted
    figure was reproducible from any subset of it -- 6.1 needed the 12 decap
    cells and 0.1 needed a different statistic over a different 27. An
    unreproducible headline that overstates the case for its own feature is the
    #694 shape exactly, and the earlier version of this file pinned
    `cells_material` and `cells_material_boards` while leaving the two numbers a
    reader actually meets unguarded.
    """
    s = live['summary']
    for key, want in (('want_mm_median', 10.3571),
                      ('frozen_mm_median', 0.0),
                      ('reach_mm_median', 1.0259),
                      ('slide_frozen_mm_median', 0.05)):
        got = s.get(key)
        check(isinstance(got, (int, float)) and abs(got - want) <= TOL,
              'summary.%s is %r, and the docs quote %r. Re-record the baseline '
              'AND the prose together, or one of them is lying.'
              % (key, got, want))
    check(len(s['boards']) == 9,
          'the sweep now reports %d board(s) with a live cell, not 9. The docs '
          'say 9 -- glasgow_revC contributes only refusals -- and "10 boards" '
          'was the wrong count that shipped.' % len(s['boards']))


def t_the_limits_of_the_feature_are_still_reported(live):
    s = live['summary']
    check(s['cells_both_zero'] > 0,
          'no both-zero cells. Boards where NEITHER arm moves are the honest '
          'limit of this feature; a table without them is a highlight reel')
    ref = s.get('refused') or []
    check(ref, 'no refusals recorded: glasgow_revC carries KiCad-locked '
               'footprints and is the board that exercises that path')
    for r in ref:
        check(r.get('refusal') and not r['refusal'].isdigit(),
              'refusal %r is not a named reason' % (r,))


def t_a_counted_board_is_never_reported_as_skipped(live):
    """A run that says it skipped a board and then counts it is worse than either.

    Measured: sonde_u printed `SKIP` and its row supplied the 11th material cell
    and the 6th material board -- exactly the two floors this file pins.
    """
    for note in live['summary'].get('skipped') or []:
        board = note.get('board')
        counted = [k for k in live['cells'] if k.startswith(board + '/')]
        if counted:
            check(note.get('counted') is True,
                  'board %s has %d counted cell(s) but its note does not say so: '
                  '%r' % (board, len(counted), note))
            check('skip' not in (note.get('reason') or '').lower(),
                  'board %s is counted and its note still calls it skipped: %r'
                  % (board, note))


# --------------------------------------------------------------------------
# the diff
# --------------------------------------------------------------------------

def t_every_cell_matches_the_baseline(live, base):
    lk, bk = set(live['cells']), set(base['cells'])
    check(lk == bk,
          'the cell SET moved. only re-derived: %r; only in baseline: %r'
          % (sorted(lk - bk)[:4], sorted(bk - lk)[:4]))
    for key in sorted(lk & bk):
        want, got = base['cells'][key], live['cells'][key]
        for k, v in want.items():
            g = got.get(k)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                check(g == v, '%s.%s: baseline %r, re-derived %r' % (key, k, v, g))
            else:
                if not check(isinstance(g, (int, float)),
                             '%s.%s: re-derived %r is not numeric' % (key, k, g)):
                    continue
                check(abs(g - v) <= TOL,
                      '%s.%s drifted: baseline %.6f, re-derived %.6f'
                      % (key, k, v, g))


def t_every_summary_number_matches_the_baseline(live, base):
    """The half that was missing. A sampled diff cannot see an aggregate move."""
    for k, v in sorted(base['summary'].items()):
        g = live['summary'].get(k)
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            if not check(isinstance(g, (int, float)),
                         'summary.%s: re-derived %r is not numeric' % (k, g)):
                continue
            check(abs(g - v) <= TOL,
                  'summary.%s drifted: baseline %r, re-derived %r' % (k, v, g))
        else:
            check(g == v, 'summary.%s: baseline %r, re-derived %r' % (k, v, g))


def t_the_study_self_test_passes():
    r = subprocess.run([sys.executable, '-X', 'utf8', STUDY, '--self-test'],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=120)
    check(r.returncode == 0,
          'the study self-test failed, so its numbers are not trustworthy:\n%s'
          % (r.stdout + r.stderr)[-1500:])


def main():
    base = load_baseline()
    check(isinstance(base.get('cells'), dict) and base['cells'],
          'baseline has no cells')
    check(isinstance(base.get('summary'), dict), 'baseline has no summary')
    t_the_study_self_test_passes()
    live = run_sweep()
    if live is not None:
        for fn in (t_identity_holds_on_every_cell,
                   t_the_arms_are_paired_and_ordered,
                   t_a_reported_reach_is_achievable,
                   t_the_verdict_does_not_sell_a_theorem_as_a_finding,
                   t_the_result_is_bounded_in_both_directions,
                   t_the_numbers_the_docs_QUOTE_are_pinned,
                   t_the_limits_of_the_feature_are_still_reported,
                   t_a_counted_board_is_never_reported_as_skipped):
            fn(live)
        t_every_cell_matches_the_baseline(live, base)
        t_every_summary_number_matches_the_baseline(live, base)

    for f in _fails:
        print('FAIL: %s' % f)
    print('test_554_reach_regen: %s (%d checks failed)'
          % ('FAIL' if _fails else 'PASS', len(_fails)))
    return 1 if _fails else 0


if __name__ == '__main__':
    sys.exit(main())
