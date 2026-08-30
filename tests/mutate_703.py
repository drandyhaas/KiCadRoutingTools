#!/usr/bin/env python3
"""The #703 mutation battery, shipped so its numbers can be re-derived.

`tests/test_703_rank_stats.py` records what each arm kills. A count is only
checkable if the exact source edit is written down -- two reviewers of the #746
branch reconstructed its rows from their names and both got the wrong answer,
because a plausible-looking reconstruction of one row was semantically inert.
So the edits live here, as data, next to the numbers they produced.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole. A row whose verdict does
not match its expectation is reported as WRONG.

WHY THIS BATTERY EXISTS AT ALL, FOR A FILE OF TWENTY-LINE FUNCTIONS. Every one
of these mutations produces a statistics module that RUNS, returns floats of a
believable magnitude, and prints a table. The failure mode #703 is about is a
number that looks measured and is not, so a kernel whose wrong answers are
indistinguishable from its right ones at a glance is exactly the thing that
needs an adversary rather than a reading. Two rows below (`nan-becomes-zero`,
`uncorrected-d2-formula`) are the two most likely ways a reimplementation of
this file would silently differ from it.

One target file (`tests/stress/rank_stats.py`), one grader
(`tests/test_703_rank_stats.py`).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the kernel in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. The file refuses to start on a
dirty target, because restoring would write the COMMITTED text back over
uncommitted work.

    python3 tests/mutate_703.py
    python3 tests/mutate_703.py --row nan-becomes-zero
    python3 tests/mutate_703.py --list

A row is KILLED by a FAILURE **or an ERROR**: several of these mutations make
the grader raise rather than fail, and a battery that counted only failures
would call that a survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor. `str.replace(old, new, 1)` of an absent needle returns the file
unchanged, which is why the count is checked BEFORE the write.

Python `str.replace`, never `sed`: commit `bb8f4477` records two rows of
`mutate_761` leaving a `SyntaxError` behind because `sed` ate an unescaped
metacharacter, and a battery that cannot start reports nothing at all.

THE MEASURED TABLE IS IN THE HEADER OF `test_703_rank_stats.py`, FROM THE RUN
-- never predicted here and never edited afterwards to match.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

RS = os.path.join(_ROOT, 'tests', 'stress', 'rank_stats.py')
TARGETS = {'rs': RS}

T703 = os.path.join(_TESTS, 'test_703_rank_stats.py')

_NONE_GUARD = (
    "        if v is None:\n"
    "            raise StatsRefusal(\n"
    "                'rank() got a None in the column. A missing measurement is not '\n"
    "                'a value: give the row an explicit null and exclude it, or '\n"
    "                'record why it is absent -- do not rank it.')\n")

_TIE_SCAN = (
    "        while j + 1 < len(order) and vals[order[j + 1]] == vals[order[i]]:\n"
    "            j += 1\n"
    "        avg = (i + j) / 2.0 + 1\n")

_DROP_NULL = (
    "        if p is None or d is None:\n"
    "            dropped += 1\n"
    "            continue\n")

_MAPPING_GUARD = (
    "    if not isinstance(rhos_by_board, Mapping):\n"
    "        raise StatsRefusal(\n"
    "            'sign_test takes a MAPPING of board_key -> BoardRho. A flat '\n"
    "            'sequence of rhos has lost which board each came from, which is '\n"
    "            'the one thing that makes an aggregate here meaningful.')\n")

_BOARD_KEY_GUARD = (
    "        if not key:\n"
    "            raise StatsRefusal(\n"
    "                f'row {i} ({r.get(\"row_id\", \"unnamed\")}) has no board_key. '\n"
    "                f'Every correlation here is computed WITHIN a board; a row that '\n"
    "                f'cannot name its board cannot be ranked against anything.')\n")

ROWS = [
    # ---- THE rule: unmeasurable is NaN, never 0.0 --------------------------
    # The single most consequential way a reimplementation goes wrong. 0.0
    # reads as "measured, no relationship"; the truth is "not measurable here".
    ('nan-becomes-zero', 'rs',
     "    if len(a) != len(b) or len(a) < MIN_N:\n"
     "        return NAN\n",
     "    if len(a) != len(b) or len(a) < MIN_N:\n"
     "        return 0.0\n",
     (T703,), 'KILLED'),

    ('constant-side-becomes-zero', 'rs',
     "    return num / (da * db) if da and db else NAN\n",
     "    return num / (da * db) if da and db else 0.0\n",
     (T703,), 'KILLED'),

    # ---- the tie correction ------------------------------------------------
    # The schoolbook 1 - 6*sum(d^2)/(n(n^2-1)) is WRONG in the presence of
    # ties, and agrees with the right answer everywhere else -- so a test suite
    # with no tied fixture cannot tell them apart. On [1,2,2,3] x [1,1,2,3] it
    # gives 0.85 against the correct 0.8333.
    ('uncorrected-d2-formula', 'rs',
     "    ma, mb = sum(ra) / n, sum(rb) / n\n"
     "    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))\n"
     "    da = math.sqrt(sum((x - ma) ** 2 for x in ra))\n"
     "    db = math.sqrt(sum((y - mb) ** 2 for y in rb))\n"
     "    return num / (da * db) if da and db else NAN\n",
     "    if len(set(a)) <= 1 or len(set(b)) <= 1:\n"
     "        return NAN\n"
     "    d2 = sum((x - y) ** 2 for x, y in zip(ra, rb))\n"
     "    return 1.0 - 6.0 * d2 / (n * (n * n - 1))\n",
     (T703,), 'KILLED'),

    ('ties-get-sequential-ranks', 'rs',
     _TIE_SCAN,
     "        avg = i + 1\n",
     (T703,), 'KILLED'),

    ('tie-detection-grows-a-tolerance', 'rs',
     "        while j + 1 < len(order) and vals[order[j + 1]] == vals[order[i]]:\n",
     "        while j + 1 < len(order) and abs(\n"
     "                vals[order[j + 1]] - vals[order[i]]) < 1e-9:\n",
     (T703,), 'KILLED'),

    # ---- the floor on n ----------------------------------------------------
    ('min-n-lowered-to-two', 'rs',
     "MIN_N = 3\n",
     "MIN_N = 2\n",
     (T703,), 'KILLED'),

    # ---- the anti-pooling guard -------------------------------------------
    ('sign-test-accepts-a-sequence', 'rs',
     _MAPPING_GUARD, '',
     (T703,), 'KILLED'),

    ('per-board-buckets-a-missing-key', 'rs',
     _BOARD_KEY_GUARD, '',
     (T703,), 'KILLED'),

    # ---- saturation is reported, never dropped -----------------------------
    ('nan-boards-join-the-denominator', 'rs',
     "        (undefined if br.rho != br.rho else defined)[b] = br\n",
     "        defined[b] = br\n",
     (T703,), 'KILLED'),

    ('saturated-is-reported-as-measurable', 'rs',
     "    return 'saturated' if uniq == {0} else 'starved'\n",
     "    return 'measurable'\n",
     (T703,), 'KILLED'),

    ('a-constant-truth-is-not-named-as-saturation', 'rs',
     "    if side == 'dependent':\n"
     "        return BoardRho(NAN, n, f'truth constant (saturated){note}')\n",
     "    if side == 'dependent':\n"
     "        return BoardRho(NAN, n, f'no variation{note}')\n",
     (T703,), 'KILLED'),

    # ---- the sign rule -----------------------------------------------------
    ('the-rule-tolerates-one-wrong-board', 'rs',
     "              and min(len(pos), len(neg)) == 0)\n",
     "              and min(len(pos), len(neg)) <= 1)\n",
     (T703,), 'KILLED'),

    # ---- the formatter that cannot emit a bare rho -------------------------
    ('fmt-rho-drops-the-LOO-span', 'rs',
     "    span = (f'LOO {fmt(lo, 0)}..{fmt(hi, 0)}' if lo == lo and hi == hi\n"
     "            else 'LOO not computed')\n",
     "    span = ''\n",
     (T703,), 'KILLED'),

    ('fmt-renders-NaN-as-zero', 'rs',
     "    return f'{x:+{w}.3f}' if x == x else f'{\"n/a\":>{w}}'\n",
     "    return f'{x:+{w}.3f}' if x == x else f'{0.0:+{w}.3f}'\n",
     (T703,), 'KILLED'),

    # ---- named refusals, not TypeErrors from deep inside sorted() ----------
    ('rank-accepts-a-None', 'rs',
     _NONE_GUARD, '',
     (T703,), 'KILLED'),

    # ---- a null measurement coerced rather than dropped --------------------
    ('board-rho-coerces-a-null-to-zero', 'rs',
     _DROP_NULL,
     "        if p is None or d is None:\n"
     "            p, d = (p or 0), (d or 0)\n",
     (T703,), 'KILLED'),

    # ---- the anti-pooling guard, EXECUTED --------------------------------
    # These four rows exist because an adversarial review falsified the first
    # version of the claim: `board_rho` never read `board_key`, so the six
    # recorded runs handed to it returned the pooled +0.339 the module docstring
    # names as the trap -- and the introspection check meant to catch that
    # whitelisted `board_rho` and `classify_board` by name, so it could not
    # fail. A guard with no row here is a guard nobody has attacked.
    ('board-rho-drops-the-one-board-guard', 'rs',
     "    _one_board(rows, 'board_rho')\n", '',
     (T703,), 'KILLED'),

    ('classify-board-drops-the-one-board-guard', 'rs',
     "    _one_board(rows, 'classify_board')\n", '',
     (T703,), 'KILLED'),

    ('the-one-board-guard-only-warns', 'rs',
     "    keys = {r.get('board_key') for r in rows if r.get('board_key')}\n"
     "    if len(keys) > 1:\n",
     "    keys = {r.get('board_key') for r in rows if r.get('board_key')}\n"
     "    if False:\n",
     (T703,), 'KILLED'),

    # The one the review found by mutation, which the battery did not carry:
    # bucketing every board into a single group leaves `per_board` returning
    # one pile, which is the pooling this module exists to prevent.
    ('per-board-buckets-every-board-into-one', 'rs',
     "        out.setdefault(key, []).append(r)\n",
     "        out.setdefault('all', []).append(r)\n",
     (T703,), 'KILLED'),

    # ---- renderings that misreport their own scope -------------------------
    ('fmt-rho-hides-a-missing-span', 'rs',
     "            else 'LOO not computed')\n",
     "            else '')\n",
     (T703,), 'KILLED'),

    # The zero COUNT glued to its own marker digit: three zero boards rendered
    # as "30". In a module whose thesis is that a number must not be
    # misreadable, that is on topic.
    ('zero-count-glued-to-its-marker', 'rs',
     "        f'{len(st[\"zero\"])} zero over {st[\"boards_defined\"]} board(s) with a '\n",
     "        f'{len(st[\"zero\"])}0 over {st[\"boards_defined\"]} board(s) with a '\n",
     (T703,), 'KILLED'),

    ('a-NaN-value-is-reported-as-a-null-one', 'rs',
     "            dropped_nan += 1\n"
     "            continue\n"
     "        if isinstance(d, float) and d != d:\n"
     "            dropped_nan += 1\n",
     "            dropped += 1\n"
     "            continue\n"
     "        if isinstance(d, float) and d != d:\n"
     "            dropped += 1\n",
     (T703,), 'KILLED'),

    # The TRUTH-side half of the same arm, anchored ALONE. The row above
    # rewrites both halves and is killed by the predictor-side check, so it
    # attacked the truth-side arm not at all -- an adversarial review mutated
    # this line by itself and both graders stayed green. A battery row whose
    # kill is entirely due to a sibling line is a row that covers nothing.
    ('a-NaN-TRUTH-is-reported-as-a-null-one', 'rs',
     "        if isinstance(d, float) and d != d:\n"
     "            dropped_nan += 1\n",
     "        if isinstance(d, float) and d != d:\n"
     "            dropped += 1\n",
     (T703,), 'KILLED'),

    ('the-NaN-TRUTH-arm-is-deleted', 'rs',
     "        if isinstance(d, float) and d != d:\n"
     "            dropped_nan += 1\n"
     "            continue\n",
     '',
     (T703,), 'KILLED'),

    # ---- the floor a shuffle control forced --------------------------------
    ('the-min-boards-floor-is-removed', 'rs',
     "    passes = (n_boards >= MIN_SIGN_BOARDS\n",
     "    passes = (n_boards >= 1\n",
     (T703,), 'KILLED'),

    ('the-min-boards-floor-drops-to-two', 'rs',
     "MIN_SIGN_BOARDS = 3\n",
     "MIN_SIGN_BOARDS = 2\n",
     (T703,), 'KILLED'),

    # ---- INERT PROBES, which must change nothing ---------------------------
    # A battery with no expected survivors cannot distinguish "my tests are
    # thorough" from "my tests fail on any edit at all". These two are real
    # source changes with no semantic content.
    ('a-comment-names-the-dependent-variable', 'rs',
     "NAN = float('nan')\n",
     "NAN = float('nan')  # blocking, never distance-to-truth\n",
     (T703,), 'SURVIVED'),

    ('fmt-default-width-widens', 'rs',
     "def fmt(x: Optional[float], w: int = 7) -> str:\n",
     "def fmt(x: Optional[float], w: int = 8) -> str:\n",
     (T703,), 'SURVIVED'),
]


def _uncache(path):
    """Delete the target's cached bytecode. MEASURED HAZARD, not hygiene.

    CPython validates a `.pyc` on (source mtime SECONDS, source SIZE), and
    several rows here are single-character edits, so a mutated and a restored
    file are the SAME SIZE. Mutate, run and restore inside one second and the
    `.pyc` compiled from the MUTATED source stays valid for every later import
    in this checkout.

    Found on the #553 branch with the sibling battery: after a clean run,
    `diagnosis.py` on disk read `TOP_K = 3` while `import` reported 4, and a
    study written afterwards silently ran at the mutated value. The battery had
    exited 0 and the tree was clean, so nothing pointed at it.
    """
    import importlib
    import importlib.util
    try:
        cached = importlib.util.cache_from_source(path)
        if os.path.exists(cached):
            os.remove(cached)
    except (OSError, ValueError, NotImplementedError):
        pass
    importlib.invalidate_caches()


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path, base = TARGETS[tgt], orig[tgt]
            edits = old if isinstance(old, list) else [(old, new)]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                print('  ran %-44s BROKEN' % name)
                continue
            mutated = base
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(path, 'w', encoding='utf-8', newline='').write(mutated)
            _uncache(path)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=600, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += [l.strip()[5:].strip()
                           for l in out.splitlines()
                           if l.strip().startswith('FAIL ')]
                if 'Traceback' in out:
                    failed.append('raised: ' + out.strip().splitlines()[-1][:70])
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            _uncache(path)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:4]))
            print('  ran %-44s %s' % (name, results[-1][1]))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
            _uncache(v)

    print()
    w = max(len(r[0]) for r in results)
    wrong = 0
    for name, verdict, expect, cnt, failed in results:
        mark = ''
        if verdict != expect:
            mark = '   <-- WRONG, expected %s' % expect
            wrong += 1
        print('%-*s  %-9s  %-3s%s' % (w, name, verdict, cnt, mark))
        for f in failed:
            print('%s      %s' % (' ' * w, f))
    killed = sum(1 for r in results if r[1] == 'KILLED')
    survived = sum(1 for r in results if r[1] == 'SURVIVED')
    broken = sum(1 for r in results if r[1] == 'BROKEN')
    print('\n%d rows: %d killed, %d survived (%d of them expected), %d broken'
          % (len(results), killed, survived,
             sum(1 for r in results if r[1] == r[2] == 'SURVIVED'), broken))
    if wrong or broken:
        print('%d row(s) did not match their expectation' % (wrong + broken))
    return 1 if (wrong or broken) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', default=None, help='run one row by name')
    ap.add_argument('--list', action='store_true', help='list the row names')
    a = ap.parse_args()
    if a.list:
        for name, tgt, _o, _n, tests, expect in ROWS:
            print('%-44s %-4s %-9s %s' % (
                name, tgt, expect,
                ' '.join(os.path.basename(t) for t in tests)))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
