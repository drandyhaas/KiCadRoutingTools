#!/usr/bin/env python3
"""#789: the slate study's arithmetic and its refusals, on synthesized rows.

    python3 -X utf8 tests/test_789_slate_harness.py

NO `wk/`, NO ROUTING, NO SUBPROCESS. Every fixture here is built in memory, so
this runs on a clean clone and says something about the analysis even when the
expensive half has never been run. `tests/test_789_slate_regen.py` is the arm
that needs real artifacts; this one is the arm that must never need them.

WHAT IT IS FOR. `slate_study.py` decides two pre-registered questions, and the
ways it can be wrong are mostly not crashes:

  * a tau computed against `ranking_routed` as printed would agree with the
    static order BY CONSTRUCTION, because that ranking's last tiebreak is the
    static position. `t_the_tau_is_not_measuring_its_own_tiebreak` builds the
    exact fixture where the two answers differ, so the circular version cannot
    pass;
  * `FIRE` has a conjunct (`not barred on hpwl`) that is easy to drop and
    changes the verdict;
  * a board that CANNOT fire must be classified, reported and excluded --
    never counted as a board that did not fire, which would put it in the
    denominator of a rule it was never able to satisfy;
  * a `blocking` of None must never read as 0.

WHAT THE BATTERY MEASURED (`python3 tests/mutate_789.py`, 16 rows against
`tests/stress/rank_stats.py` and `tests/stress/slate_study.py`):

    tau-b-uses-tau-a-denominator                  KILLED
    tau-b-drops-the-tie-correction                KILLED
    tau-sign-is-flipped                           KILLED
    tau-returns-zero-not-nan                      KILLED
    tau-min-n-lowered-to-two                      KILLED
    board-tau-drops-the-pooling-guard             KILLED
    fmt-tau-drops-the-LOO-span                    KILLED
    tau-measured-against-the-static-order-itself  KILLED
    fire-drops-the-hpwl-conjunct                  KILLED
    a-clean-baseline-is-scored-not-classified     KILLED
    a-none-blocking-is-read-as-zero               KILLED
    a-saturated-board-scores-tau-zero             KILLED
    rule5-tolerates-one-dissenting-board          KILLED
    rule5-drops-the-three-board-floor             KILLED
    cannot-fire-boards-join-the-denominator       KILLED
    the-null-rate-is-never-computed               KILLED

    16 rows: 16 killed, 0 survived, 0 broken

The first run of that battery was 14 killed, 1 survived, 1 broken, and both
non-kills were real:

  * `the-null-rate-is-never-computed` SURVIVED, because this file called
    `qbar_null` directly and nothing checked that `aggregate` still computed
    it or that `report` still printed it. A null rate nobody sees is a null
    rate nobody was told -- and it is the number that says how little rule 2's
    discharge is worth. Two asserts added.
  * `tau-returns-zero-not-nan` came back BROKEN, anchor matched twice:
    `kendall_tau` and `tau_a` open with the same two lines. Reported rather
    than silently mutating whichever came first, which is the anchor rule
    doing its job.
"""
import os
import sys

RUN_ALL_TIMEOUT = 120
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

import slate_study as SS                                        # noqa: E402
import rank_stats as rs                                         # noqa: E402

FAILURES = []


def check(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILURES.append(what)


def row(board, idx, crossings, hpwl, blocking, viable=True, rank=None):
    """One slate row, with only the fields the analysis reads."""
    return {
        'schema': 1, 'kind': 'slate-row', 'row_id': f'slate:{board}:{idx}',
        'board_key': board, 'index': idx, 'strategy': 'jitter',
        'viable': viable,
        'predictors': {'crossings': crossings, 'hpwl': hpwl,
                       'inversions': 0, 'plane_islands': 0,
                       'health_penalty': 0, 'plane_neck': 0},
        'displacement_rms': 0.0,
        'rank_key': list(rank) if rank else [crossings, 0, 0, 0, hpwl, 0, 0,
                                             0.0, idx],
        'rule1_clauses': [],
        'rule1_would_bar_on_crossings': False,
        'rule1_would_bar_on_hpwl': False,
        'truth': {'blocking': blocking, 'source': 'test'},
        'provenance': {},
    }


def slate(board, spec, base_crossings, base_hpwl, base_blocking):
    """Candidate 0 plus candidates from `spec`, with rule-1 flags derived the
    way `slate_study._row` derives them -- strictly worse than the baseline."""
    rows = [row(board, 0, base_crossings, base_hpwl, base_blocking)]
    for i, (cx, hp, blk) in enumerate(spec, start=1):
        r = row(board, i, cx, hp, blk)
        r['rule1_would_bar_on_crossings'] = cx > base_crossings
        r['rule1_would_bar_on_hpwl'] = hp > base_hpwl + 1e-9
        rows.append(r)
    return rows


def t_qbar_fires_only_on_a_crossings_only_violator():
    # baseline blocking 4. Candidate 1 is barred on crossings alone and routed
    # BETTER -> fires. Candidate 2 is barred on crossings but routed worse.
    # Candidate 3 is barred on BOTH and routed better -- it stays barred after
    # the crossings clause is withdrawn, so it is not evidence about the
    # withdrawal, and only `fire_literal` counts it.
    rows = slate('b', [(30, 100.0, 3), (30, 100.0, 9), (30, 300.0, 1)],
                 20, 200.0, 4)
    q = SS.qbar(rows)
    check(q['verdict'] == 'fires', f"a crossings-only violator fires: {q['verdict']}")
    check(q['fire'] == ['slate:b:1'], f"...and names only it: {q['fire']}")
    check(sorted(q['fire_literal']) == ['slate:b:1', 'slate:b:3'],
          f"the LITERAL reading also counts the both-barred candidate: "
          f"{q['fire_literal']}")
    # Dropping the hpwl conjunct is the mutation this arm exists for.
    check(q['fire'] != q['fire_literal'],
          'the two readings differ on this fixture, so a run cannot silently '
          'substitute one for the other')


def t_a_board_that_cannot_fire_is_classified_not_counted():
    for reason, rows in (
            ('baseline_clean',
             slate('b', [(30, 100.0, 5)], 20, 200.0, 0)),
            ('no_barred_candidate',
             slate('b', [(10, 100.0, 1)], 20, 200.0, 4)),
    ):
        q = SS.qbar(rows)
        check(q['verdict'] == 'cannot_fire' and q['reason'] == reason,
              f'{reason}: {q.get("verdict")}/{q.get("reason")}')
    # ...and such a board is NOT in rule 2's "able to fire" denominator.
    agg = SS.aggregate(slate('b', [(30, 100.0, 5)], 20, 200.0, 0))
    check(agg['rule2']['boards_able_to_fire'] == [],
          'a cannot-fire board is excluded from the denominator, not counted '
          'as a board that did not fire')


def t_a_missing_blocking_is_never_a_zero():
    rows = slate('b', [(30, 100.0, None)], 20, 200.0, 4)
    q = SS.qbar(rows)
    check(q['verdict'] == 'cannot_fire'
          and q['reason'] == 'no_graded_barred_candidate',
          f'an ungraded barred candidate cannot fire: {q.get("reason")}')
    check(q['fire'] == [], 'and None never compares as lower than 4')
    # A baseline with no blocking voids the board rather than scoring it.
    rows = slate('b', [(30, 100.0, 1)], 20, 200.0, None)
    check(SS.qbar(rows)['verdict'] == 'void',
          'a baseline with no blocking VOIDS rather than scores')


def t_the_tau_is_not_measuring_its_own_tiebreak():
    """The circular version and the honest one must DISAGREE on this fixture.

    Static order here is by crossings; the routed `blocking` deliberately runs
    the other way. If a future edit keyed the routed side on the static
    position (which is what `ranking_routed`'s last tiebreak does), tau would
    come back +1.0. It must not.
    """
    rows = slate('b', [(11, 100.0, 4), (12, 100.0, 3), (13, 100.0, 2),
                       (14, 100.0, 1)], 10, 100.0, 5)
    q = SS.qorder(rows)
    check(q['verdict'] == 'measured', f'tau is measurable here: {q}')
    check(q['tau'] < -0.99,
          f'a slate whose routed order is the REVERSE of the static order '
          f'gives tau -1, not +1: {q.get("tau")}')
    # The circular computation, written out, to show what it would have said.
    order = sorted(rows, key=lambda r: tuple(r['rank_key']))
    circular = rs.kendall_tau([i for i, _ in enumerate(order)],
                              [i for i, _ in enumerate(order)])
    check(circular > 0.99 and q['tau'] < -0.99,
          'the circular version returns +1 on the same slate, so the two are '
          'distinguishable by this fixture')


def t_tau_agrees_when_the_key_agrees():
    rows = slate('b', [(11, 100.0, 2), (12, 100.0, 3), (13, 100.0, 4),
                       (14, 100.0, 5)], 10, 100.0, 1)
    q = SS.qorder(rows)
    check(q['verdict'] == 'measured' and q['tau'] > 0.99,
          f'a perfectly agreeing slate gives tau +1: {q.get("tau")}')
    check('tau=+1.000' in q['display'] and 'LOO' in q['display'],
          f'and it renders through fmt_tau: {q.get("display")}')


def t_a_saturated_board_has_no_tau_and_keeps_its_value():
    rows = slate('b', [(11, 100.0, 0), (12, 100.0, 0), (13, 100.0, 0)],
                 10, 100.0, 0)
    q = SS.qorder(rows)
    check(q['verdict'] == 'no_verdict' and 'saturated' in q['reason'],
          f'an all-zero routed column is not a tau of 0: {q}')
    check(q.get('constant_value') == 0,
          'and the constant value is reported, not dropped (rule 4)')
    agg = SS.aggregate(rows)
    check(agg['rule5']['n_boards_with_a_defined_tau'] == 0
          and agg['rule5']['verdict'] == 'NOT SHOWN TO AGREE',
          'a board with no tau is out of the denominator, and N=0 cannot pass')


def t_rule5_needs_three_boards_and_no_dissenter():
    def agreeing(b):
        return slate(b, [(11, 100.0, 2), (12, 100.0, 3), (13, 100.0, 4)],
                     10, 100.0, 1)

    def disagreeing(b):
        return slate(b, [(11, 100.0, 4), (12, 100.0, 3), (13, 100.0, 2)],
                     10, 100.0, 5)
    two = agreeing('a') + agreeing('b')
    check(SS.aggregate(two)['rule5']['verdict'] == 'NOT SHOWN TO AGREE',
          'two agreeing boards is below MIN_SIGN_BOARDS and licenses nothing')
    three = agreeing('a') + agreeing('b') + agreeing('c')
    check(SS.aggregate(three)['rule5']['verdict'] == 'AGREES',
          'three agreeing boards clears the sign rule')
    mixed = agreeing('a') + agreeing('b') + disagreeing('c')
    check(SS.aggregate(mixed)['rule5']['verdict'] == 'NOT SHOWN TO AGREE',
          'one dissenting board blocks it -- right on >= N-1 AND wrong on none')
    allneg = disagreeing('a') + disagreeing('b') + disagreeing('c')
    check('DISAGREES' in SS.aggregate(allneg)['rule5']['verdict'],
          'a consistently negative tau is its own outcome, and it names the '
          'issue rather than performing the reorder')


def t_the_null_rate_is_reported_and_can_be_high():
    """Rule 2's criterion is easy to satisfy, and the harness must say so.

    On a slate where almost every candidate routes better than the baseline,
    ANY permutation puts a good value on a barred candidate, so the null rate
    is 1.0. That is not a bug in the study -- it is the number that says how
    little a discharge is worth, and it is pre-registered as expected-high.
    """
    rows = slate('b', [(30, 100.0, 1), (31, 100.0, 1), (32, 100.0, 1)],
                 20, 200.0, 9)
    check(SS.qbar_null(rows) == 1.0,
          f'an easily-satisfied slate reports a null rate of 1.0, not silence '
          f'({SS.qbar_null(rows)})')
    hard = slate('b', [(30, 100.0, 9), (10, 100.0, 1), (11, 100.0, 1)],
                 20, 200.0, 2)
    n = SS.qbar_null(hard)
    check(n is not None and n < 1.0,
          f'a slate where the barred candidate is the worst reports a LOWER '
          f'rate ({n})')
    # ...and the AGGREGATE carries it, which is what a reader sees. The
    # mutation battery caught this arm calling `qbar_null` directly and
    # therefore surviving a change that stopped `aggregate` computing it at
    # all -- a null rate nobody can see is a null rate nobody was told.
    agg = SS.aggregate(rows)
    check(agg['per_board']['b'].get('qbar_null') == 1.0,
          f"aggregate() carries the per-board null rate: "
          f"{agg['per_board']['b'].get('qbar_null')}")
    lines = chr(10).join(SS.report(agg))
    check('null' in lines and '100%' in lines,
          'and the report PRINTS it, so a discharge cannot be read without it')


def main():
    for fn in (t_qbar_fires_only_on_a_crossings_only_violator,
               t_a_board_that_cannot_fire_is_classified_not_counted,
               t_a_missing_blocking_is_never_a_zero,
               t_the_tau_is_not_measuring_its_own_tiebreak,
               t_tau_agrees_when_the_key_agrees,
               t_a_saturated_board_has_no_tau_and_keeps_its_value,
               t_rule5_needs_three_boards_and_no_dissenter,
               t_the_null_rate_is_reported_and_can_be_high):
        print(fn.__name__ + ':')
        fn()
    if FAILURES:
        print(f'\ntest_789_slate_harness: {len(FAILURES)} failure(s)')
        for f in FAILURES:
            print('  - ' + f)
        return 1
    print('\ntest_789_slate_harness: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
