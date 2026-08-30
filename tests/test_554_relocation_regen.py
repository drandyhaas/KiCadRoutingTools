#!/usr/bin/env python3
"""The #554 routed study: its aggregation re-derived, its structure enforced.

`tests/stress/block_relocation_study.py` asks whether a bounded block relocation
restores a board that damage made unroutable. Its rows are committed, and this
guards them.

WHAT THIS RE-DERIVES, AND WHAT IT HONESTLY CANNOT
--------------------------------------------------
`test_554_reach_regen.py` re-runs its whole study, because that one costs six
seconds. **This one cannot**: every counted cell needs five full routes plus two
loop invocations, which is hours. So this file is explicit about the split
instead of pretending:

* **RE-DERIVED**: the entire summary, recomputed from the committed rows through
  the study's own `summarise()`. That catches any change to the aggregation --
  the verdict thresholds, the evidence/instrument split, the medians, the
  loop-comparison counts -- which is where a result most easily drifts without
  anyone re-running a route.
* **NOT re-derived**: the routed `blocking` numbers themselves. They are a
  recorded measurement, and if the engine changes underneath them the rows go
  stale silently. That is a real limitation, it is stated here, and the fix if it
  ever matters is to re-run the study and re-record -- not to weaken this file.

The distinction matters because the previous version of the *reach* regen test
sampled cells and read the rest from its own baseline, which made five of seven
checks tautologies over a constant. Sampling is only acceptable when the file
says which aggregates are therefore unguarded. Here, none are.

THE STRUCTURAL RULES, WHICH HOLD WHATEVER THE VERDICT
------------------------------------------------------
Each is a way the study could be void while every number in it looks fine:

  1. Every counted cell carries its UNDAMAGED pairing (`R0`). Without it a repair
     arm cannot tell "it fixed the damage" from "it moves that block on every
     board" -- the defect that voided #553's first recall reading.
  2. `translate` is never counted as evidence: `perturb.block_direction` derives
     the damage direction from the same quantity the solve consumes.
  3. A cell that was skipped is named WITH its reason and its numbers, never
     dropped -- `not_placement_limited` in particular is the RUNBOOK's own
     definition of a non-subject and is a result, not an absence.
  4. The `loop@allon` column is present on every counted cell. #554 is only
     interesting if it beats a tool that already exists.

Run: python3 -X utf8 tests/test_554_relocation_regen.py
"""
import json
import os
import subprocess
import sys

RUN_ALL_TIMEOUT = 300

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

BASELINE = os.path.join(ROOT, 'tests', '554_block_relocation_baseline.json')
STUDY = os.path.join(ROOT, 'tests', 'stress', 'block_relocation_study.py')

_fails = []


def check(cond, msg):
    if not cond:
        _fails.append(msg)
    return cond


def t_the_study_self_test_passes():
    r = subprocess.run([sys.executable, '-X', 'utf8', STUDY, '--self-test'],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=120)
    check(r.returncode == 0,
          'the study self-test failed, so its aggregation is not trustworthy '
          'whatever the committed rows say:\n%s'
          % (r.stdout + r.stderr)[-1200:])


def t_the_summary_re_derives_from_the_committed_rows(base):
    """The half that IS re-derivable, and the half a doc-only number never is."""
    import block_relocation_study as S
    got = S.summarise(base['rows'])
    want = base['summary']
    for k in sorted(want):
        g, w = got.get(k), want[k]
        if isinstance(w, float):
            check(isinstance(g, (int, float)) and abs(g - w) <= 1e-9,
                  'summary.%s re-derives to %r, committed %r' % (k, g, w))
        else:
            check(g == w, 'summary.%s re-derives to %r, committed %r'
                          % (k, g, w))


def t_every_counted_cell_has_its_undamaged_pairing(base):
    for r in base['rows']:
        if 'skipped' in r or not r.get('counted_in_primary'):
            continue
        check(r.get('route_effect_on_control') is not None,
              '%s/%s is counted with no R0 arm. A repair arm without its '
              'undamaged pairing cannot tell "it fixed the damage" from "it '
              'moves that block on every board".' % (r['board'], r['kind']))
        check(r.get('route_recovery_delta') is not None,
              '%s/%s is counted without a delta, so the raw recovery would be '
              'read as the evidence' % (r['board'], r['kind']))
        check(r.get('loop_route_recovery') is not None
              or (r.get('arms', {}).get('L', {}).get('timed_out')),
              '%s/%s has no loop@allon column and no recorded timeout; #554 is '
              'only interesting if it beats the tool that exists'
              % (r['board'], r['kind']))


def t_the_instrument_arm_is_never_evidence(base):
    for r in base['rows']:
        if r.get('kind') in ('translate',):
            check(r.get('counted_in_primary') is False,
                  '%s/translate is counted in the primary. Its damage direction '
                  'comes from the same block_displacements the solve consumes, '
                  'so its recovery is partly arithmetic.' % r['board'])
    check(base['summary'].get('instrument_cells'),
          'the instrument arm is not reported at all; excluding it silently is '
          'how it comes back as evidence later')


def t_skipped_cells_are_named_with_their_numbers(base):
    for r in base['rows']:
        if 'skipped' not in r:
            continue
        why = r['skipped']
        check(why and not why.isdigit(),
              '%s/%s was skipped without a named reason' % (r['board'], r['kind']))
        if 'not_placement_limited' in why:
            check(any(ch.isdigit() for ch in why),
                  'a not_placement_limited skip carries no numbers: %r. It is '
                  'the RUNBOOK\'s definition of a non-subject and is a RESULT.'
                  % why)


def t_the_verdict_is_the_recorded_one(base):
    v = base['summary'].get('verdict') or ''
    check(v == base.get('recorded_verdict'),
          'the verdict changed: %r, recorded %r. Re-run the study and re-record '
          'deliberately -- do not edit one of the two.'
          % (v, base.get('recorded_verdict')))
    check(any(w in v for w in ('SUPPORTED', 'NOT SUPPORTED', 'UNDERPOWERED',
                               'NOT MEASURED')),
          'the verdict is not one of the four the study can produce: %r' % v)


def t_the_contract_checks_are_recorded_on_every_counted_cell(base):
    """A cell can route better and still FAIL #554's own definition."""
    for r in base['rows']:
        if 'skipped' in r or not r.get('counted_in_primary'):
            continue
        c = r.get('contract') or {}
        check('collateral_parts' in c,
              '%s/%s has no collateral count. "Move only diagnosed blocks" is '
              "#554's definition, not its bonus, and a routed win bought by "
              'walking the neighbours is place_route_loop.'
              % (r['board'], r['kind']))


def main():
    if not os.path.exists(BASELINE):
        print('SKIP: %s not present' % os.path.basename(BASELINE))
        return 77
    with open(BASELINE, encoding='utf-8') as fh:
        base = json.load(fh)
    check(isinstance(base.get('rows'), list) and base['rows'],
          'baseline has no rows')
    check(isinstance(base.get('summary'), dict), 'baseline has no summary')
    if base.get('rows'):
        t_the_study_self_test_passes()
        t_the_summary_re_derives_from_the_committed_rows(base)
        t_every_counted_cell_has_its_undamaged_pairing(base)
        t_the_instrument_arm_is_never_evidence(base)
        t_skipped_cells_are_named_with_their_numbers(base)
        t_the_verdict_is_the_recorded_one(base)
        t_the_contract_checks_are_recorded_on_every_counted_cell(base)
    for f in _fails:
        print('FAIL: %s' % f)
    print('test_554_relocation_regen: %s (%d checks failed)'
          % ('FAIL' if _fails else 'PASS', len(_fails)))
    return 1 if _fails else 0


if __name__ == '__main__':
    sys.exit(main())
