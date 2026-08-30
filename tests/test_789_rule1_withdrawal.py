#!/usr/bin/env python3
"""#789: rule 1's crossings clause is withdrawn, and the withdrawal keeps its row.

    python3 -X utf8 tests/test_789_rule1_withdrawal.py

Pre-registered rule 2 (`docs/placement-predictors.md`) said: if a future run
finds >= 1 board on which a rule-1 violator routed to strictly lower `blocking`
than the baseline, the `crossings` clause of `rule1_check` is withdrawn -- and
the withdrawal keeps its row, with its measured direction, in the `rejected`
style `test_placement_ab.py` uses.

The run found it. This is that row.

WHY NOT A ROW IN `test_placement_ab.ROWS`. Those rows are quench-objective A/B
pairs: a board, a corridor set, a flag toggled off and on, graded by
`floorplan.grade`. This finding is a ROUTE, and forcing it in would make `_run`
route -- turning a minutes-long harness into an hours-long one -- while giving
every existing row a new `if` in its judging path, which that file's own
doctrine forbids. So this is the same DISCIPLINE on the object that has the
shape: a committed literal record (`tests/placement_rule1_withdrawal.json`),
re-checked in milliseconds, failing if the mark changes.

Millisecond, no `wk/`, no board, no route: the record stores METRICS, and
`rule1_check` / `rule1_advisory` / `select_best` are pure functions of them.
"""
import json
import os
import sys

RUN_ALL_TIMEOUT = 60
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from placement import portfolio as PF                           # noqa: E402

RECORD = os.path.join(ROOT, 'tests', 'placement_rule1_withdrawal.json')

FAILURES = []


def check(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILURES.append(what)


def load():
    if not os.path.isfile(RECORD) or os.path.getsize(RECORD) == 0:
        FAILURES.append(f'{RECORD} is missing or empty -- the withdrawal has '
                        f'no recorded evidence, which is the one thing rule 2 '
                        f'demanded of it')
        return None
    with open(RECORD, encoding='utf-8') as fh:
        return json.load(fh)


def cand(idx, crossings, hpwl):
    return PF.Candidate(index=idx, strategy='jitter',
                        metrics={'crossings': crossings, 'hpwl': hpwl})


def t_the_clause_no_longer_bars():
    d = load()
    if not d:
        return
    m = d['measured']
    base = cand(0, m['baseline']['crossings'], m['baseline']['hpwl'])
    for row in m['firing_candidates']:
        c = cand(row['index'], row['crossings'], row['hpwl'])
        clauses = PF.rule1_check(c, base)
        check(not any(s.startswith('crossings ') for s in clauses),
              f"candidate {row['index']} ({row['crossings']} crossings vs "
              f"{m['baseline']['crossings']}) is no longer barred on crossings")
        check(not clauses,
              f"...and is not barred at all, since its hpwl is below the "
              f"baseline's ({row['hpwl']:.1f} < {m['baseline']['hpwl']})")


def t_the_direction_is_kept_as_an_advisory():
    """A withdrawn clause that stops MEASURING is a deleted clause. Rule 2 said
    keep the measured direction, so the comparison still runs and still
    reports -- it simply no longer bars."""
    d = load()
    if not d:
        return
    m = d['measured']
    base = cand(0, m['baseline']['crossings'], m['baseline']['hpwl'])
    worst = max(m['firing_candidates'], key=lambda r: r['crossings'])
    adv = PF.rule1_advisory(cand(worst['index'], worst['crossings'],
                                 worst['hpwl']), base)
    check(any(s.startswith('crossings ') for s in adv),
          f'the crossings direction is still measured and reported: {adv}')
    check(any('WITHDRAWN' in s for s in adv),
          f'...and says so, so nobody reads it as a bar: {adv}')
    # A candidate BETTER on crossings has nothing to report.
    better = PF.rule1_advisory(cand(9, m['baseline']['crossings'] - 1,
                                    m['baseline']['hpwl']), base)
    check(not any(s.startswith('crossings ') for s in better),
          f'a candidate with FEWER crossings reports nothing: {better}')


def t_the_hpwl_clause_survives_untouched():
    """Rule 2 withdrew one clause. The other must be exactly as it was."""
    d = load()
    if not d:
        return
    m = d['measured']
    base = cand(0, m['baseline']['crossings'], m['baseline']['hpwl'])
    worse = cand(9, m['baseline']['crossings'] - 5,
                 m['baseline']['hpwl'] + 10.0)
    clauses = PF.rule1_check(worse, base)
    check(any(s.startswith('hpwl ') for s in clauses),
          f'a candidate worse on hpwl is STILL barred: {clauses}')
    # ...and the EPS tolerance is untouched: a hair worse is not worse.
    hair = cand(9, m['baseline']['crossings'] - 5,
                m['baseline']['hpwl'] + 1e-12)
    check(not PF.rule1_check(hair, base),
          'and the hpwl EPS tolerance still absorbs a 1e-12 difference')


def t_select_best_reaches_the_candidate_the_bar_used_to_exclude():
    """The behavioural delta, on the recorded slate.

    With a probe ranking that puts the best-routing candidate first -- which is
    what a probe is FOR -- the bar used to skip it and take a candidate that
    routed worse. This is the concrete mechanism rule 2 existed to detect.
    """
    d = load()
    if not d:
        return
    m = d['measured']
    best = min(m['firing_candidates'], key=lambda r: r['blocking'])
    probe = [best['index'], 2]
    static = [2, best['index']]
    after = PF.select_best(probe, static, set())
    check(after == best['index'],
          f'with the clause withdrawn, select_best reaches candidate '
          f'{best["index"]} (blocking {best["blocking"]})')
    before = PF.select_best(probe, static, {best['index']})
    check(before == 2,
          'with it barred, it fell through to the next index -- which on this '
          f'board routed to {m["best_unbarred_blocking"]}')
    check(best['blocking'] < m['best_unbarred_blocking'],
          f'and that is strictly worse: {best["blocking"]} vs '
          f'{m["best_unbarred_blocking"]}')


def t_the_record_still_satisfies_the_pre_registered_predicate():
    """The row is evidence only while its own numbers still say what they said.

    Editing a literal to make a story nicer fails here, which is the whole
    point of the `rejected` style: the mark is re-checked, not re-read.
    """
    d = load()
    if not d:
        return
    check(d.get('rejected') is True and d.get('expect'),
          'the record carries the rejected-style mark and its expectation')
    m = d['measured']
    b0 = m['baseline']['blocking']
    fires = [r for r in m['firing_candidates']
             if r['crossings'] > m['baseline']['crossings']
             and r['hpwl'] <= m['baseline']['hpwl']
             and r['blocking'] < b0]
    check(len(fires) == len(m['firing_candidates']) and fires,
          f'every recorded firing candidate satisfies rule 2 as written: '
          f'{len(fires)} of {len(m["firing_candidates"])}')
    check(d['null_rate'].get('esp_prog') == 1.0,
          'the null rate is recorded, and it is 1.0 -- the discharge is by a '
          'pre-registered criterion this board satisfies trivially')
    check(d['behavioural_delta'].get('worse_on') == 0,
          'and the withdrawal made no board worse in either arm')


def main():
    for fn in (t_the_clause_no_longer_bars,
               t_the_direction_is_kept_as_an_advisory,
               t_the_hpwl_clause_survives_untouched,
               t_select_best_reaches_the_candidate_the_bar_used_to_exclude,
               t_the_record_still_satisfies_the_pre_registered_predicate):
        print(fn.__name__ + ':')
        fn()
    if FAILURES:
        print(f'\ntest_789_rule1_withdrawal: {len(FAILURES)} failure(s)')
        for f in FAILURES:
            print('  - ' + f)
        return 1
    print('\ntest_789_rule1_withdrawal: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
