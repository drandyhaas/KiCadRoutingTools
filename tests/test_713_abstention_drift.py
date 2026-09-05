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

  DRIFT   re-derives the whole census (~40 s) and compares it to the committed
          file, per board and per key.
  ENGINE  asserts the census still ASKS `floorplan._is_not_asked` which skips
          are the honest kind, rather than keeping its own copy of the reason
          strings, in milliseconds. It kept one once, and it went stale (#837
          added `assembly_side` and the copy was not updated), which silently
          reclassifies an honest skip as a real abstention. The DRIFT arm sees
          only the symptom, and only once somebody re-records; this one catches
          the cause.

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
sys.path[:0] = [os.path.join(ROOT, 'py_router'),
                os.path.join(ROOT, 'py_placer')]

#: 40 s measured on an idle machine, dominated by `parse_kicad_pcb` on the four
#: large boards. Declared with headroom so a slower box reports FAIL rather
#: than TIME.
RUN_ALL_TIMEOUT = 900

#: THREE clocks can expire on a hang, and only this one gives a reason, so it
#: is set below the other two rather than equal to them:
#:
#:   run_all's budget          `max(RUN_ALL_TIMEOUT, 600)` = 900, started at
#:                             process spawn -> prints `TIME (... NOT a failed
#:                             assertion)`, which discloses nothing about why
#:   this gate's subprocess     started ~1 s later, after import and the ENGINE
#:                             arm -> `FAIL: the recorder did not finish ...`
#:   the recorder's own        900 s per `check_floorplan` child, started later
#:                             still
#:
#: Set equal, run_all always wins by that ~1 s head start and the named path is
#: unreachable under the runner -- measured: forced equal, standalone printed
#: the reason and `run_all` printed only `TIME`. The margin makes the reason
#: reachable where a reader will actually meet it.
RECORDER_TIMEOUT = RUN_ALL_TIMEOUT - 120

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
# DRIFT, MISSING, MALFORMED and the message shapes are
# `tests/test_placement_ab.py`'s `compare_baseline` (its docstring explains
# each), reused rather than reinvented. The rest of that vocabulary does not
# transfer, and the reader is owed which is which rather than left to wonder:
#
#   INVERTED  NOT carried over. It compares the direction of an off/on pair,
#             and this document has no arms -- there is nothing to reverse.
#   ORPHAN    NOT carried over. There, it means a baseline row that `ROWS` no
#             longer declares; here the row set is the corpus itself, so that
#             case is MISSING (in the baseline, not measured).
#   NEW ROW / NEW KEY / MISSING KEY / DUPLICATE
#             NEW here. This document is a per-board table with a fixed key
#             set, which `placement_ab_baseline.json` is not, so appearing,
#             gaining a field and being recorded twice are all reachable and
#             all mean different things.

def diff_value(label, exp_v, cur_v, problems):
    """One value -- or one level deeper when both sides are objects.

    `channels_seen` is a nested dict, and reporting it whole hands the reader
    two five-key objects to diff by eye. That is the aggregate-verdict mistake
    #694 is about, in the one place in this document where it could still
    happen: `DRIFT channels_seen.budget_abstained` names the input that moved,
    where the whole-dict form only says the dict is different.
    """
    if isinstance(exp_v, dict) and isinstance(cur_v, dict):
        for k in sorted(set(exp_v) | set(cur_v)):
            if k not in exp_v:
                problems.append(f"NEW KEY {label}.{k}: measured {cur_v[k]!r}")
            elif k not in cur_v:
                problems.append(f"MISSING KEY {label}.{k}: the baseline "
                                f"records {exp_v[k]!r}")
            else:
                diff_value(f'{label}.{k}', exp_v[k], cur_v[k], problems)
        return
    # TYPE too, not just value. `1 == True` and `22 == 22.0` in Python, so a
    # baseline whose `pass` is `1` where the run produces `true`, or whose
    # counts have become floats, compares equal and the gate says nothing. It
    # is a shape change in a recorded measurement, and this document's ints and
    # bools carry different meanings -- assert the shape, not just the value.
    if type(exp_v) is not type(cur_v):
        problems.append(f"DRIFT {label}: baseline {exp_v!r} "
                        f"({type(exp_v).__name__}), measured {cur_v!r} "
                        f"({type(cur_v).__name__}) -- same value, different "
                        f"type")
    elif exp_v != cur_v:
        problems.append(f"DRIFT {label}: baseline {exp_v!r}, "
                        f"measured {cur_v!r}")


def index_rows(rows, where):
    """`board` -> row, and the problems found while indexing.

    The obvious `{r['board']: r for r in rows}` is LAST-WINS, and it silently
    drops any row without a usable `board`. Both matter here:

    * A duplicated board makes one of the two copies invisible to every
      comparison downstream -- so a baseline row claiming `pass: false` about a
      board that passes can sit in the committed file while the gate reports
      zero problems. That is not exotic for a 22-row JSON array: it is what a
      merge conflict resolved by keeping both hunks produces, and a
      hand-edited baseline is this gate's entire threat model.
    * A row with no `board` key indexes under `None`, and `sorted()` over a set
      holding `None` and strings raises `TypeError` -- a traceback after a 40 s
      re-derive, which is not a verdict and is indistinguishable from a broken
      test.

    Both are REPORTED, by name and row index.
    """
    indexed, problems, seen = {}, [], {}
    for i, row in enumerate(rows):
        if not isinstance(row, dict):
            problems.append(f"MALFORMED {where}.rows[{i}]: expected an "
                            f"object, got {type(row).__name__}")
            continue
        board = row.get('board')
        if not isinstance(board, str):
            problems.append(f"MALFORMED {where}.rows[{i}]: 'board' is "
                            f"{board!r}, not a string")
            continue
        if board in seen:
            problems.append(f"DUPLICATE {where}.rows[{i}]: {board} is already "
                            f"row {seen[board]}; only one of the two would be "
                            f"compared, so the other never had to be right")
            continue
        seen[board] = i
        indexed[board] = row
    return indexed, problems


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
        else:
            diff_value(key, expected[key], current[key], problems)

    cur_rows, cur_problems = index_rows(current.get('rows') or [], 'measured')
    problems += cur_problems
    exp_raw = expected.get('rows')
    if not isinstance(exp_raw, list):
        problems.append("MALFORMED baseline.rows: expected a list, got "
                        f"{type(exp_raw).__name__}")
        return problems
    exp_rows, exp_problems = index_rows(exp_raw, 'baseline')
    problems += exp_problems

    # `boards_total` is compared baseline-vs-measured above, but never against
    # the rows it counts -- so a document whose header disagrees with its own
    # body on BOTH sides is internally inconsistent and silent. Cheap to hold.
    for doc, where, rows in ((expected, 'baseline', exp_raw),
                             (current, 'measured', current.get('rows') or [])):
        total = doc.get('boards_total')
        if isinstance(total, int) and total != len(rows):
            problems.append(f"MALFORMED {where}: boards_total says {total}, "
                            f"rows holds {len(rows)}")

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
            else:
                diff_value(f'{board}.{key}', exp[key], cur[key], problems)
    return problems


def arm_engine(census_mod):
    """`classify` must ASK the engine, not re-implement it.

    This arm replaces a set-equality check against `floorplan._SKIP_REASON`'s
    VALUES, and the reason it replaces it is the point. The census used to keep
    its own copy of those strings; the arm compared the two sets in both
    directions and would have caught the copy going stale. But the engine
    exports `_is_not_asked(rule, reason)` as, in its own words, "ONE home for
    the distinction", keyed on the RULE precisely because a value match "meant
    an `_ARM` reason that ever happened to equal some OTHER rule's skip reason
    would read as 'nobody asked' -- a real abstention silently downgraded to a
    pass". Two sets can be perfectly equal and that collision still happen, so
    the mirror arm was green in exactly the case it most needed to be red.

    Calling the engine deletes the mirror, and this arm holds it deleted: a
    census that grows its own copy again reintroduces the drift AND the
    collision. Milliseconds, no subprocess.
    """
    from placement.floorplan import _is_not_asked, _WITHHELD_MARK
    if census_mod.classify.__code__.co_argcount != 2:
        fail("classify() no longer takes (rule, reason). Keyed on the reason "
             "alone it cannot tell two rules that share a skip-reason string "
             "apart -- and _SKIP_REASON has 12 keys to 11 distinct values")
        return
    for name in ('NOT_ASKED', 'WITHHELD_MARK', '_SKIP_REASON'):
        if hasattr(census_mod, name):
            fail(f"the census has grown its own `{name}` again. That copy is "
                 f"what #879 is about: it went stale once already, and a "
                 f"value-keyed match is the downgrade `_is_not_asked` exists "
                 f"to prevent. Call the engine")
            return
    # And the call must actually REACH it: agree with the engine on every pair
    # the engine can produce, and on the `not requested` fallback grade()
    # raises outside the RULES loop for a rule with no recorded reason.
    #
    # That fallback is the one place the swap changed an ANSWER. The old
    # hand-copy listed `not requested` as "nobody asked"; `_is_not_asked` says
    # no, because it cannot match a rule that recorded no reason -- so it now
    # counts as an abstention. The engine is right: an unexplained skip is
    # exactly the case that must not read as a pass, and this is the direction
    # its docstring calls safe. Unreachable today (`RULES` and `_SKIP_REASON`
    # have the same 12 keys, so the fallback never fires), which is why the
    # re-record is byte-identical -- but pinned here so the change is a
    # decision on record rather than a silent consequence.
    from placement.floorplan import _SKIP_REASON
    pairs = list(_SKIP_REASON.items()) + [('a_rule_with_no_reason',
                                           'not requested')]
    wrong = [(rule, reason, census_mod.classify(rule, reason))
             for rule, reason in pairs
             if (census_mod.classify(rule, reason) == 'not_asked')
             is not bool(_is_not_asked(rule, reason))]
    if wrong:
        fail(f"classify disagrees with floorplan._is_not_asked on "
             f"{len(wrong)} pair(s): {wrong[:5]}")
        return
    if census_mod._WITHHELD_MARK is not _WITHHELD_MARK:
        fail("the census's WITHHELD mark is not the engine's object")
        return
    print(f"  ok   ENGINE: classify() calls _is_not_asked, agreeing on all "
          f"{len(pairs)} (rule, reason) pair(s) the engine can produce")


def main():
    print("#879 abstention-census drift")

    census_mod = _load_recorder()
    arm_engine(census_mod)

    # The corpus comes from `git ls-files`. Where git cannot answer, SKIP
    # loudly rather than grading a set we cannot identify -- the rule
    # `run_utils.corpus_boards` states and `test_714_identity_write_unchanged`
    # follows.
    try:
        boards = census_mod.tracked_boards(ROOT)
    except (subprocess.CalledProcessError, OSError) as exc:
        # NARROW on purpose. A bare `except Exception` here attributes any
        # recorder-side bug -- an AttributeError, a bad import -- to "git is
        # missing", and turns a broken gate into a clean SKIP that run_all
        # scores as exit 0. These two are what an absent or unhappy git
        # actually raises; everything else must reach the traceback.
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
        try:
            r = subprocess.run(
                [sys.executable, '-X', 'utf8', RECORDER, ROOT,
                 '--out', out, '-q'],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', cwd=ROOT, timeout=RECORDER_TIMEOUT)
        except subprocess.TimeoutExpired:
            # A traceback here reads as a broken test, not as a verdict --
            # CLAUDE.md's rule that a non-zero exit is not evidence, assert the
            # REASON. Say which clock ran out and how long it was given.
            print(f"FAIL: the recorder did not finish within "
                  f"RECORDER_TIMEOUT={RECORDER_TIMEOUT} s (measured 40 s), so "
                  f"there is no census to compare. This is the only one of "
                  f"the three clocks that gives a reason, which is why it is "
                  f"set below run_all's {RUN_ALL_TIMEOUT} s budget.")
            return 1
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
            print(f"  ... and {len(problems) - 40} more (sorted, so what is "
                  f"cut is the tail of the alphabet -- re-record and read the "
                  f"diff for the whole set)")
        print("\n  READ THE DIFF BEFORE RE-RECORDING. A number that moved for "
              "someone else's reason becomes yours the moment you re-record "
              "it and cite it -- which is the mistake this gate exists to "
              "prevent (#879).\n"
              f"  {REMEDY}")
    else:
        # Count the BASELINE's rows, not the fresh run's. The success line is
        # a claim about the committed file, and counting the thing that was
        # just measured would report 22 about a 24-row baseline.
        print(f"  ok   DRIFT: {len(expected.get('rows') or ())} baseline "
              f"row(s) match this run of "
              f"{os.path.basename(RECORDER)}")

    print(f"\n{'FAIL' if FAILURES else 'PASS'}: #879 census drift, "
          f"{len(FAILURES)} failure(s)")
    for f in FAILURES:
        print(f"  - {f}")
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
