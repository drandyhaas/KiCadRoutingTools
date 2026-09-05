#!/usr/bin/env python3
"""#788: fourteen rows of the study, committed, so the decision has a detector.

    python3 -X utf8 tests/test_788_marginal_literals.py

WHY LITERALS AND NOT THE ROWS FILE. #703's 276 KB of rows were dropped from the
tree on review -- a regenerable result is not an artifact worth carrying -- and
`docs/placement-predictors.md` records what that cost: no number in it has an
automated change detector any more. The shape drandyhaas suggested instead was
to keep the finding in the doc and regenerate a small DECLARED subset on
demand. This is that subset for #788's decision.

TWO ARMS, and the split is the point.

  Arm A runs ANYWHERE, including a clean clone with no `wk/`. It re-derives
  #788's CLAIM arithmetically from the literals below: which rows the shipped
  L2 conjunction refuses, which ones a graze/hole conjunction would add, and
  how those routed. Editing a literal without re-measuring flips it.

  Arm B runs only where the #703 study tree exists. It re-runs `check_assembly`
  on those fourteen boards and asserts the four counts still match the literals,
  which is what makes Arm A's numbers facts rather than transcription. Absent,
  it SKIPS LOUDLY -- naming the missing path and the command that rebuilds it --
  because a check that quietly does nothing is the failure this repo keeps
  re-finding.

WHAT THE ROWS WERE CHOSEN TO SPAN, so a reader can see this is not a flattering
sample: both marginal rows a graze gate would add (the whole question), the
worst board in the study and the row that shows which conjunct catches it, a
row where the off-outline conjunct fires alone, the six authored boards as the
clean control, and the widest `blocking` a passing board reached.

Counts are graded at each board's own netclass floor, which is what a routed
board carries. `tests/measure_788_censuses.py` reports the same table at the
0.25 fallback too, and the doc quotes both, because the off-outline agreement
is clearance-conditional.
"""
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 900
RUN_ALL_FAST_OK = False

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STUDY = os.path.join(ROOT, 'wk', '703', 'study')
CHECK_ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')

#: board, variant, then check_assembly's counts on the routed board, then the
#: routed `blocking` board_score recorded for that placement.
#: (blocking, oob_pad_count, pad_conflicts, hole_conflicts, routed_blocking)
ROWS = (
    ('esp_prog', 'authored', 0, 0, 0, 0, 0),
    ('esp_prog', 'perturb-translate-d1', 0, 0, 1, 0, 0),
    ('esp_prog', 'perturb-translate-d2', 0, 0, 1, 0, 2),
    ('esp_prog', 'perturb-wrong_side', 1, 3, 1, 0, 32),
    ('kit-dev-coldfire-xilinx_5213', 'authored', 0, 0, 0, 0, 7),
    ('kit-dev-coldfire-xilinx_5213', 'perturb-translate-d0', 0, 0, 0, 0, 9),
    ('sonde_u', 'authored', 0, 0, 0, 0, 0),
    ('splitflap_driver', 'authored', 0, 0, 0, 0, 0),
    ('tigard', 'authored', 0, 0, 0, 0, 2),
    ('tigard', 'perturb-pile', 2783, 0, 2956, 335, 13078),
    ('watchy', 'authored', 0, 0, 0, 0, 1),
    ('watchy', 'perturb-pile', 2202, 1, 2339, 52, 8521),
    # Off-outline ALONE: blocking 0, so the pad-intersection conjunct passes it
    # and only oob_pad_count refuses. Without a row of this shape the
    # complementarity claim beside the gate has no evidence here, and the
    # first version of this file asserted it with a tautology.
    # STALE SINCE #826, and only the first of the two. The portfolio's jitter
    # now snaps its offset to the board's lattice, so a rebuilt study tree
    # produces a different sonde_u:portfolio-3 candidate (sonde_u is imperial,
    # 0.3175). The literal below still describes the slate these numbers were
    # measured on; it is no longer reproducible from HEAD without rebuilding
    # wk/703/study, which test_703_predictor_regen's header prices at ~8.8h.
    # Arm A is arithmetic over these tuples and cannot move; Arm B skips
    # loudly when the tree is absent, which it is on a clean checkout.
    # watchy:portfolio-2 is NOT affected -- watchy declares no lattice (best
    # occupancy 0.238 < floor 0.67), so its jitter stays continuous.
    ('sonde_u', 'portfolio-3', 0, 1, 0, 0, 0),
    ('watchy', 'portfolio-2', 0, 1, 0, 0, 3),
)

FAILURES = []
SKIPPED = []


def check(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILURES.append(what)


def shipped_refuses(row):
    """L2 as it ships: blocking > 0 or oob_pad_count > 0."""
    return row[2] > 0 or row[3] > 0


def graze_refuses(row):
    """The channel #788 asked about: pad_conflicts > 0 or hole_conflicts > 0."""
    return row[4] > 0 or row[5] > 0


def t_arm_a_the_claim_follows_from_the_literals():
    """No `wk/`, no subprocess: the arithmetic #788's decision rests on."""
    refused = [r for r in ROWS if shipped_refuses(r)]
    passed = [r for r in ROWS if not shipped_refuses(r)]
    added = [r for r in passed if graze_refuses(r)]
    check(len(refused) == 5,
          f'the shipped conjunction refuses 5 of these 14 (got {len(refused)})')
    check(len(added) == 2,
          f'a graze/hole conjunction would add 2 more (got {len(added)})')
    check(sorted(r[6] for r in added) == [0, 2],
          f'...and those 2 routed to blocking 0 and 2 '
          f'(got {sorted(r[6] for r in added)})')
    still = [r for r in passed if r not in added]
    check(max(r[6] for r in still) == 9,
          f'the worst row BOTH conjunctions pass routed to 9 '
          f'(got {max(r[6] for r in still)})')
    # The two marginal rows are worse than nothing only if they routed WORSE
    # than what the gate lets through. They did not: 0 and 2 sit inside the
    # 0..9 the shipped conjunction already passes. That is the whole finding.
    check(max(r[6] for r in added) <= max(r[6] for r in still),
          'neither marginal refusal routed worse than a row the gate passes')


def t_arm_a_the_two_conjuncts_are_complementary():
    """Each conjunct catches a row the other does not -- the claim the comment
    beside the gate makes, reduced to the declared subset."""
    only_blocking = [r for r in ROWS if r[2] > 0 and r[3] == 0]
    only_oob = [r for r in ROWS if r[3] > 0 and r[2] == 0]
    check(len(only_blocking) >= 1,
          f'a row is caught by the pad-intersection conjunct ALONE '
          f'({len(only_blocking)})')
    check(len(only_oob) >= 2,
          f'rows are caught by the off-outline conjunct ALONE '
          f'({len(only_oob)}) -- neither conjunct subsumes the other')
    check(max(r[6] for r in only_oob) <= 3,
          f'...and those routed to blocking <= 3, which is why #788 declines '
          f'to add a SECOND marginal channel rather than to remove this one '
          f'(max {max(r[6] for r in only_oob)})')
    worst = max(ROWS, key=lambda r: r[6])
    check(worst[6] == 13078 and worst[3] == 0 and worst[2] > 0,
          f'the worst board is caught by blocking, NOT by oob_pad_count '
          f'(oob={worst[3]}, blocking={worst[2]})')


def t_arm_a_the_literals_are_not_all_zero():
    """A table of zeros would satisfy every arm above without saying anything.

    This is the vacuity guard: two of this repo's own tests have passed while
    asserting against an empty object.
    """
    check(sum(r[4] for r in ROWS) > 0, 'the graze column is not all zeros')
    # The hole column had NO constraint at all until review swept every cell:
    # zeroing it in all 14 rows left every check green, so the gate comment's
    # "7 of the 7 carrying a hole conflict are refused already" had no detector
    # here whatsoever.
    check(sum(r[5] for r in ROWS) > 0, 'the hole column is not all zeros')
    hole_rows = [r for r in ROWS if r[5] > 0]
    check(len(hole_rows) >= 2 and all(shipped_refuses(r) for r in hole_rows),
          f'every row carrying a hole conflict ({len(hole_rows)}) is already '
          f'refused by the shipped conjunction')
    check(len({r[6] for r in ROWS}) >= 6,
          'the routed-blocking column spans a real range')
    check(len({r[0] for r in ROWS}) == 6, 'all six study boards are present')
    check(len(ROWS) == 14, f'the declared subset is 14 rows (got {len(ROWS)})')


def _board(board_key, variant):
    d = os.path.join(STUDY, board_key, variant)
    p = os.path.join(d, 'routed.kicad_pcb')
    return p if os.path.isfile(p) else None


def t_arm_b_the_literals_still_match_the_instrument():
    missing = [f'{b}:{v}' for b, v, *_ in ROWS if _board(b, v) is None]
    if missing:
        msg = (f'SKIP: the #703 study tree is not here, so the literals cannot '
               f'be re-measured ({len(missing)} of {len(ROWS)} boards absent, '
               f'first: {missing[0]}). Arm A above still ran and still holds. '
               f'Rebuild with: python3 -X utf8 '
               f'tests/stress/predictor_study.py --out wk/703/study -j 4')
        print('  ' + msg)
        SKIPPED.append(msg)
        return
    with tempfile.TemporaryDirectory() as tmp:
        for b, v, blk, oob, pad, hole, _routed in ROWS:
            out = os.path.join(tmp, f'{b}_{v}.json')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8', CHECK_ASSEMBLY, _board(b, v),
                 '--json', out], cwd=ROOT, capture_output=True, text=True,
                timeout=900)
            if r.returncode not in (0, 4) or not os.path.isfile(out):
                check(False, f'{b}:{v} could not be graded (exit '
                             f'{r.returncode}) -- not a mismatch, a broken '
                             f'measurement')
                continue
            with open(out, encoding='utf-8') as fh:
                g = json.load(fh)
            got = (g.get('blocking'), g.get('oob_pad_count'),
                   g.get('pad_conflicts'), g.get('hole_conflicts'))
            check(got == (blk, oob, pad, hole),
                  f'{b}:{v} still grades {(blk, oob, pad, hole)} (got {got})')
            # THE SEVENTH COLUMN, which had no detector in either arm until
            # review pointed out that `score.json` -- carrying exactly this
            # number -- sits in the directory Arm B already opens. It is the
            # column the whole finding rests on, and it was transcription.
            sp = os.path.join(STUDY, b, v, 'score.json')
            if not os.path.isfile(sp):
                check(False, f'{b}:{v} has no score.json to check '
                             f'routed_blocking against')
                continue
            with open(sp, encoding='utf-8') as fh:
                got_blk = json.load(fh).get('blocking')
            check(got_blk == _routed,
                  f'{b}:{v} routed to blocking {_routed} (score.json says '
                  f'{got_blk})')


def main():
    for fn in (t_arm_a_the_claim_follows_from_the_literals,
               t_arm_a_the_two_conjuncts_are_complementary,
               t_arm_a_the_literals_are_not_all_zero,
               t_arm_b_the_literals_still_match_the_instrument):
        print(fn.__name__ + ':')
        fn()
    if FAILURES:
        print(f'\ntest_788_marginal_literals: {len(FAILURES)} failure(s)')
        for f in FAILURES:
            print('  - ' + f)
        return 1
    print('\ntest_788_marginal_literals: all checks passed'
          + (f' ({len(SKIPPED)} arm skipped)' if SKIPPED else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
