"""The committed change detector for `tests/measure_792_seeding.py`'s rows.

The measurement itself runs three full seeds per board and takes minutes, so it
is not collected. This file reads the rows it committed and RE-DERIVES every
aggregate the PR body quotes -- it does not sample, and it does not compare a
sampled cell against a constant. `test_554_reach_regen`'s recorded scar is that
an earlier version of exactly this shape sampled two cells and checked the other
twenty-two against the baseline, so five of its seven checks were tautologies
and it printed PASS while the sweep's headline halved.

WHY THE ROWS EXIST AT ALL, rather than a `test_placement_ab.py` row.
That harness's `_run` calls `quench(...)`; nothing in the table calls
`seed_from_intent`, and `_intent_for` emits without `derive_decaps` so every
existing row's intent carries `decaps: {}` and no decap rule can arm. Hosting a
seed row means a new arm kind inside a gate seven pinned rows depend on -- a
change to the gate, made in the PR whose evidence the gate would produce. So the
properties that gate enforces (pairing, >= 3 distinct boards, a direction rather
than an absolute) are enforced HERE instead, and the harness is left alone.
"""
import json
import os
import sys

RUN_ALL_FAST_OK = True

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
ROWS = os.path.join(TESTS_DIR, '792_seeding_rows.jsonl')

#: The same floor `test_placement_ab.gate()` applies: a direction measured on
#: fewer than three distinct boards is a coin flip with a press release.
MIN_BOARDS = 3


def _rows():
    if not os.path.exists(ROWS):
        raise AssertionError(
            f"{ROWS} is missing. Regenerate with "
            f"`python3 tests/measure_792_seeding.py`. A missing artefact is "
            f"not a pass: the numbers this file checks would simply not be "
            f"checked.")
    out = []
    with open(ROWS, encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    assert out, ROWS
    return out


def _armed(rows):
    return [r for r in rows if r.get('armed') and 'error' not in r]


def test_the_rows_cover_the_tracked_corpus_and_say_what_they_skipped():
    rows = _rows()
    armed = _armed(rows)
    inert = [r['board'] for r in rows if not r.get('armed')]
    errs = [(r['board'], r['error']) for r in rows if 'error' in r]
    assert not errs, errs
    assert len(rows) >= 15, len(rows)
    assert len(armed) >= MIN_BOARDS, (len(armed), "fewer than three boards "
                                                  "arm the stage, so no "
                                                  "direction is measurable")
    # A board that cannot arm is RECORDED, not dropped -- "measured and inert"
    # and "never measured" must not look the same.
    for r in rows:
        if not r.get('armed'):
            assert r.get('reason'), r
    print(f"  PASS: {len(rows)} board(s) -- {len(armed)} arm the pin stage, "
          f"{len(inert)} recorded as inert ({', '.join(inert[:4])}...)")


def test_the_control_arm_is_never_what_changed():
    """Every claim below is a comparison BETWEEN arms of the same run, so the
    control's only job is to exist. Its poses are recorded so a later reader
    can diff them against a fresh run and see whether the seeder moved
    underneath the rows."""
    for r in _armed(_rows()):
        assert r['off']['poses'], r['board']
        assert r['off']['claimed'] == 0, (r['board'], r['off']['claimed'])
        assert r['off']['put_back'] == 0, (r['board'], r['off']['put_back'])
    print("  PASS: the control arm claims nothing and puts nothing back on "
          "every armed board")


def test_no_arm_leaves_a_part_unseated_that_the_control_seated():
    """The put-back is MONOTONE by construction -- it falls through to the
    centroid stage rather than appending to `unseated`. This is that claim,
    re-derived per board rather than argued."""
    rows = _armed(_rows())
    for r in rows:
        base = set(r['off']['unseated'])
        for arm in ('on', 'chips'):
            extra = sorted(set(r[arm]['unseated']) - base)
            assert not extra, (r['board'], arm, extra)
    print(f"  PASS: {len(rows)} board(s), no part newly unseated in either "
          f"armed arm")


def test_the_pin_geometry_does_not_get_WORSE_on_any_board():
    """The directional claim, paired and re-derived.

    `pin_gap_sum` is measured by the GRADER on the seeded board, not by the
    seeder's own notes -- counting "decap for ..." notes would be the stage
    grading itself. Direction only: the change must not make any board's supply
    pins collectively further from their caps."""
    rows = [r for r in _armed(_rows())
            if r['off']['pin_gap_sum'] is not None]
    assert len(rows) >= MIN_BOARDS, len(rows)
    worse, better, same = [], [], []
    for r in rows:
        a, b = r['off']['pin_gap_sum'], r['on']['pin_gap_sum']
        if b is None:
            continue
        (worse if b > a + 1e-6 else better if b < a - 1e-6 else same).append(
            (r['board'], round(a, 2), round(b, 2)))
    assert not worse, worse
    print(f"  PASS: {len(rows)} board(s) -- {len(better)} improved, "
          f"{len(same)} unchanged, 0 worse"
          + (f"; best {sorted(better, key=lambda t: t[1] - t[2])[0]}"
             if better else ""))


def test_the_owner_test_arm_is_measured_even_though_it_ships_OFF():
    """`decap_owner_chips` is flagged off, and a flag with no measurement
    beside it is a flag nobody can ever decide about. Recorded here in both
    directions so the decision has evidence when it is made."""
    rows = [r for r in _armed(_rows())
            if r['off']['pin_gap_sum'] is not None
            and r['chips']['pin_gap_sum'] is not None]
    assert len(rows) >= MIN_BOARDS, len(rows)
    worse = [(r['board'], r['on']['pin_gap_sum'], r['chips']['pin_gap_sum'])
             for r in rows
             if r['chips']['pin_gap_sum'] > r['on']['pin_gap_sum'] + 1e-6]
    better = [(r['board'], r['on']['pin_gap_sum'], r['chips']['pin_gap_sum'])
              for r in rows
              if r['chips']['pin_gap_sum'] < r['on']['pin_gap_sum'] - 1e-6]
    claimed = sum(r['chips']['claimed'] - r['on']['claimed'] for r in rows)
    # NOT asserted as an improvement -- that is what the flag is waiting to
    # find out. Asserted only as MEASURED, so the row cannot rot into a claim.
    print(f"  PASS: owner-chips measured on {len(rows)} board(s) -- "
          f"{len(better)} better, {len(worse)} worse, {claimed:+d} pin seats; "
          f"the flag ships OFF and this is the evidence it is waiting on")


def test_the_put_back_actually_fires_somewhere():
    """ANTI-VACUITY. Every claim above is compatible with a change that does
    nothing at all, so at least one board must show the put-back doing work --
    otherwise these rows are measuring an inert edit."""
    rows = _armed(_rows())
    fired = [(r['board'], r['on']['put_back']) for r in rows
             if r['on']['put_back']]
    assert fired, ("no board puts a cap back; the rows cannot distinguish this "
                   "change from no change")
    total = sum(n for _b, n in fired)
    print(f"  PASS: {len(fired)} board(s) put caps back, {total} in total "
          f"({', '.join(f'{b}:{n}' for b, n in fired[:4])})")


TESTS = [
    test_the_rows_cover_the_tracked_corpus_and_say_what_they_skipped,
    test_the_control_arm_is_never_what_changed,
    test_no_arm_leaves_a_part_unseated_that_the_control_seated,
    test_the_pin_geometry_does_not_get_WORSE_on_any_board,
    test_the_owner_test_arm_is_measured_even_though_it_ships_OFF,
    test_the_put_back_actually_fires_somewhere,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
