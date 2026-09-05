#!/usr/bin/env python3
"""#879: the committed abstention census must still be what the tree produces.

`tests/713_abstention_census.json` records, per tracked board, whether the
emit-intent -> grade round trip reports a CLEAN verdict while a channel the
intent ASKED FOR went ungraded. Nothing re-derived it. Its recorder is not
named `test_*.py`, so `run_all.py`'s glob never collected it, and a tree-wide
grep finds no other caller -- so the file was free to drift arbitrarily far
from the tree while still reading as authoritative.

It did. Measured over the four days between its only two recordings:

    rules_run                                   moved on 22 boards
    pass / grade_rc / human_pass_line /
    clean_but_ungraded                          moved on  5 boards
    boards_clean_but_ungraded                   5 -> 0

The second recording was made as part of #837, and the `5 -> 0` was published
in a commit message AND a code comment as that change's effect. It was not:
a reviewer re-ran the census at the base commit with none of the new code
present and found those five boards already failing. The drift was
`budget_abstained` making the grade incomplete, days earlier and unrelated.

A stale baseline is worse than an absent one. Absent, you measure. Stale, you
diff -- and the diff goes out signed with your name.

    python3 -X utf8 tests/test_713_abstention_drift.py

Two arms, and the cheap one is not a formality:

  DRIFT   re-derives the whole census (~45 s) and compares it to the committed
          file, per board and per key.
  MIRROR  asserts `NOT_ASKED` still equals `floorplan._SKIP_REASON`'s values,
          in milliseconds. That set is hand-maintained and has already fallen
          out of step once (#837 added `assembly_side` and did not update it),
          which silently reclassifies an honest skip as a real abstention. The
          DRIFT arm sees only the symptom, and only once somebody re-records;
          this one catches the cause.

WHY NOT `RUN_ALL_FAST_OK`. This shells out 44 times. `run_all`'s marker is
documented for the case where "the shelling-out really is cheap" -- one
`git ls-files` -- and 45 s of `check_floorplan.py` is not that. So it stays in
the integration bucket and `--fast` skips it, deliberately.
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path[:0] = [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer')]

#: 42.6 s measured on the author's machine, dominated by `parse_kicad_pcb` on
#: the four large boards. Declared with headroom so a slower box reports FAIL
#: rather than TIME -- and note the recorder's own per-subprocess timeout is
#: 900 s, so a genuinely hung `check_floorplan` blows that first.
RUN_ALL_TIMEOUT = 900

BASELINE = os.path.join(TESTS, '713_abstention_census.json')
RECORDER = os.path.join(TESTS, '713_abstention_census.py')

#: How to re-record, printed with every failure. A gate that says "this moved"
#: without saying what to do about it gets worked around rather than answered.
REMEDY = ('python3 -X utf8 tests/713_abstention_census.py   '
          '# then READ the diff before committing it')

FAILURES = []


def fail(msg):
    FAILURES.append(msg)


def _load_recorder():
    """Import the recorder WITHOUT running the census.

    Possible only since #879 phase 1 gave it a `main()`; before that, importing
    this module ran a 42 s corpus grade as a side effect. If that regresses,
    this import is where it shows up -- as a 42 s pause in a test that claims
    to be milliseconds.
    """
    spec = importlib.util.spec_from_file_location('abstention_census',
                                                  RECORDER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --- the comparator ---------------------------------------------------------
#
# Verdict vocabulary and message shapes are `tests/test_placement_ab.py`'s
# `compare_baseline` (its docstring explains each), reused rather than
# reinvented. One is deliberately NOT carried over: INVERTED compares the
# direction of an off/on pair, and this document has no arms -- there is
# nothing to reverse. Saying so here is cheaper than leaving the next reader to
# wonder which verdict went missing and why.

def compare(current, expected):
    """Problems, as strings. Every class is fatal; they are kept apart because
    they mean different things."""
    problems = []
    if not isinstance(expected, dict):
        # REPORTED, not raised: crashing after a 45 s re-derive is not a
        # verdict, and the message a reader needs is which shape it found.
        return [f"MALFORMED baseline: expected an object, got "
                f"{type(expected).__name__}"]

    for key in sorted(set(current) | set(expected)):
        if key == 'rows':
            continue
        if key not in expected:
            problems.append(f"NEW KEY {key}: measured {current[key]!r}, "
                            f"the baseline has never seen it")
        elif key not in current:
            problems.append(f"MISSING KEY {key}: the baseline records "
                            f"{expected[key]!r}, this run produced nothing")
        elif current[key] != expected[key]:
            problems.append(f"DRIFT {key}: baseline {expected[key]!r}, "
                            f"measured {current[key]!r}")

    cur_rows = {r.get('board'): r for r in (current.get('rows') or [])}
    exp_rows = expected.get('rows')
    if not isinstance(exp_rows, list):
        problems.append("MALFORMED baseline.rows: expected a list, got "
                        f"{type(exp_rows).__name__}")
        return problems
    exp_rows = {r.get('board'): r for r in exp_rows if isinstance(r, dict)}

    for board in sorted(set(cur_rows) | set(exp_rows)):
        cur, exp = cur_rows.get(board), exp_rows.get(board)
        if exp is None:
            problems.append(f"NEW ROW {board}: not in the baseline")
            continue
        if cur is None:
            problems.append(f"MISSING {board}: in the baseline, not measured "
                            f"in this run")
            continue
        # Per KEY, never a whole-row equality. An aggregate verdict cannot say
        # which of its inputs moved -- #694's lesson, and the reason the
        # misattribution this gate exists for was possible at all.
        for key in sorted(set(cur) | set(exp)):
            if key not in exp:
                problems.append(f"NEW KEY {board}.{key}: measured "
                                f"{cur[key]!r}")
            elif key not in cur:
                problems.append(f"MISSING KEY {board}.{key}: the baseline "
                                f"records {exp[key]!r}")
            elif cur[key] != exp[key]:
                problems.append(f"DRIFT {board}.{key}: baseline {exp[key]!r}, "
                                f"measured {cur[key]!r}")
    return problems


def arm_mirror(census_mod):
    """`NOT_ASKED` is a hand-copy of `floorplan._SKIP_REASON`'s values.

    Both directions. A reason the census does not know falls through
    `classify()` to `'arm'` -- a real abstention -- so a stale mirror does not
    error, it quietly changes the answer. And an entry here that `_SKIP_REASON`
    no longer produces is a claim about a rule that no longer exists.
    """
    from placement.floorplan import _SKIP_REASON
    declared = set(_SKIP_REASON.values())
    mirrored = set(census_mod.NOT_ASKED)
    # `not requested` is raised outside the RULES loop, so it is legitimately
    # in the mirror and not in _SKIP_REASON. Named, not silently subtracted.
    extra_by_design = {'not requested'}

    missing = sorted(declared - mirrored)
    if missing:
        fail(f"NOT_ASKED is missing {len(missing)} _SKIP_REASON value(s), so "
             f"an honest skip is classified as a real abstention: "
             f"{missing}")
    stale = sorted(mirrored - declared - extra_by_design)
    if stale:
        fail(f"NOT_ASKED carries {len(stale)} reason(s) _SKIP_REASON no longer "
             f"produces: {stale}")
    if not missing and not stale:
        print(f"  ok   MIRROR: NOT_ASKED == _SKIP_REASON values "
              f"({len(declared)} reasons, both directions)")


def main():
    print("#879 abstention-census drift")

    census_mod = _load_recorder()
    arm_mirror(census_mod)

    # The corpus comes from `git ls-files`. Where git cannot answer, SKIP
    # loudly rather than grading a set we cannot identify -- the rule
    # `run_utils.corpus_boards` states and `test_714_identity_write_unchanged`
    # follows.
    try:
        boards = census_mod.tracked_boards(ROOT)
    except Exception as exc:                                   # noqa: BLE001
        print(f"SKIP: git cannot name the tracked corpus here "
              f"({type(exc).__name__}), so there is no set to census: {exc}")
        return 77
    if not boards:
        print("SKIP: git named no tracked boards, so this would grade nothing")
        return 77

    if not os.path.isfile(BASELINE):
        # NOT a pass. Deleting the baseline must not delete the protection
        # behind exit 0.
        print(f"FAIL: no baseline at {BASELINE}. Re-record it deliberately:\n"
              f"  {REMEDY}")
        return 1
    try:
        with open(BASELINE, encoding='utf-8') as fh:
            expected = json.load(fh)
    except Exception as exc:                                   # noqa: BLE001
        print(f"FAIL: baseline unreadable: {exc}\n  {REMEDY}")
        return 1

    # Re-derive into a temp dir. The recorder's default `--out` IS the
    # committed file, so a gate that forgot this flag would overwrite the very
    # document it is comparing against and then report a match.
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'fresh.json')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', RECORDER, ROOT, '--out', out, '-q'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT, timeout=RUN_ALL_TIMEOUT)
        if r.returncode != 0 or not os.path.isfile(out):
            tail = (r.stdout + r.stderr).strip()[-800:]
            print(f"FAIL: the recorder did not produce a census "
                  f"(exit {r.returncode}):\n{tail}")
            return 1
        with open(out, encoding='utf-8') as fh:
            current = json.load(fh)

    problems = compare(current, expected)
    if problems:
        fail(f"{len(problems)} baseline problem(s)")
        print(f"\nbaseline: {len(problems)} problem(s) vs "
              f"{os.path.basename(BASELINE)}")
        for p in problems[:40]:
            print(f"  {p}")
        if len(problems) > 40:
            print(f"  ... and {len(problems) - 40} more")
        print("\n  READ THE DIFF BEFORE RE-RECORDING. A number that moved for "
              "someone else's reason becomes yours the moment you re-record "
              "it and cite it -- which is the mistake this gate exists to "
              "prevent (#879).\n"
              f"  {REMEDY}")
    else:
        print(f"  ok   DRIFT: {len(current.get('rows') or ())} board(s) match "
              f"{os.path.basename(BASELINE)}")

    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #879 census drift, "
          f"{len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f"  - {f}")
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
