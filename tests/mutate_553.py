#!/usr/bin/env python3
"""The #553 mutation battery: does `test_553_diagnosis_rank.py` actually bite?

Every mutation below produces a `placement/diagnosis.py` that IMPORTS, RUNS,
and prints a believable per-signal table. That is the whole reason this file
exists. The defect #553 is about is a CONFIDENT ranking with no evidence behind
it, so a module whose wrong answers are indistinguishable from its right ones at
a glance needs an adversary rather than a reading -- and this repo's own record
is two placement tests that passed without ever reaching the branch they named.

Edits live here AS DATA with a per-row EXPECTATION. An inert row recorded as an
expected survivor is a finding; an inert row quietly deleted is a hole. A row
whose verdict does not match its expectation is reported as WRONG.

One target (`py_placer/placement/diagnosis.py`), one grader
(`tests/test_553_diagnosis_rank.py`).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the module in place. One writer per tree -- do not run it while a suite or a
review is reading the same checkout. It refuses to start on a dirty target,
because restoring would write the COMMITTED text back over uncommitted work.

    python3 tests/mutate_553.py
    python3 tests/mutate_553.py --row concat-not-round-robin
    python3 tests/mutate_553.py --list

A row is KILLED by a FAILURE **or an ERROR**: some of these make the grader
raise rather than fail, and a battery that counted only failures would call
that a survivor. An anchor that does not match EXACTLY ONCE is BROKEN rather
than skipped -- a battery that silently applies nothing reports every row as a
survivor. `str.replace`, never `sed`.

THE MEASURED TABLE IS IN THE HEADER OF `test_553_diagnosis_rank.py`, FROM THE
RUN -- never predicted here and never edited afterwards to match.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

DIAG = os.path.join(_ROOT, 'py_placer', 'placement', 'diagnosis.py')
TARGETS = {'d': DIAG}

T553 = os.path.join(_TESTS, 'test_553_diagnosis_rank.py')

_RANK_KEY = ("                  key=lambda kv: (-round(kv[1], RANK_DECIMALS), "
             "kv[0]))\n")

_ROUND_ROBIN = (
    "    while not exhausted and any(pools.get(s) for s in SIGNAL_ORDER):\n"
    "        for sig in SIGNAL_ORDER:\n")

_FANOUT_CUT = (
    "    if max_fanout:\n"
    "        ignored.update(nid for nid, refs in net_refs.items()\n"
    "                       if len(refs) > max_fanout)\n")

_COUNTLESS = (
    "            if 'blocked_count' not in b:\n"
    "                countless.add(net)\n"
    "                continue\n")

_WINNER = (
    "                values['blocker_cells'][win[0]] = (\n"
    "                    values['blocker_cells'].get(win[0], 0) + cells)\n")

_POOLS = ("    pools = {sig: [k for k, _ in ranked[sig][:max(0, top_k)]] "
          "for sig in ranked}\n")


ROWS = [
    # ---- the ranking itself ------------------------------------------------
    # Alphabetical order instead of value order. The module still ranks, still
    # selects, still prints -- it just ranks on the name.
    ('identity-rank', 'd',
     _RANK_KEY,
     "                  key=lambda kv: (kv[0],))\n",
     (T553,), 'KILLED'),

    # Ascending instead of descending: the LEAST displaced block leads.
    ('sign-flip', 'd',
     _RANK_KEY,
     "                  key=lambda kv: (round(kv[1], RANK_DECIMALS), kv[0]))\n",
     (T553,), 'KILLED'),

    # ---- the determinism guarantee, and the fixture that cannot test it ----
    # Comparing raw floats instead of rounded ones. EXPECTED SURVIVOR, and
    # recording it is the finding: every value in the fixture differs by far
    # more than 1e-4, so this battery cannot tell the two apart. The guarantee
    # is real (two processes must not order two near-equal candidates
    # differently) and it is currently unattacked. Closing this needs a fixture
    # whose values differ below RANK_DECIMALS; until one exists, the row stands
    # here as the disclosure rather than as a green tick.
    ('unrounded-compare', 'd',
     _RANK_KEY,
     "                  key=lambda kv: (-kv[1], kv[0]))\n",
     (T553,), 'SURVIVED'),

    # ---- the omission rule `block_displacements` established ---------------
    # "Connects to nothing outside itself" reported as "sits exactly on its
    # partners". Two edits, because the omission only becomes a 0.0 if a zero
    # is also allowed to rank.
    ('omit-becomes-zero', 'd',
     [("            for row in bd:\n",
       "            for _n in blocks:\n"
       "                values['block_displacement'][_n] = 0.0\n"
       "            for row in bd:\n"),
      ("    return sorted(((k, v) for k, v in values.items() if v > 0),\n",
       "    return sorted(((k, v) for k, v in values.items() if v >= 0),\n")],
     None, (T553,), 'KILLED'),

    # ---- inventing the router's evidence ----------------------------------
    # route.py's stage='preexisting' entries carry a net name and nothing else.
    # Imputing 1 cell each is the quietest possible way to fabricate evidence.
    ('preexisting-counts-one', 'd',
     _COUNTLESS,
     "            if 'blocked_count' not in b:\n"
     "                cells[net] = cells.get(net, 0) + 1\n"
     "                continue\n",
     (T553,), 'KILLED'),

    # The rail cut removed. GND owns 96 of ulx3s's 179 parts; without this the
    # signal is near-constant and the ranking is noise wearing a number.
    ('high-fanout-cut-removed', 'd',
     _FANOUT_CUT, '',
     (T553,), 'KILLED'),

    # Cells handed to every owner instead of the plurality owner. Shares then
    # sum far above 1 and every candidate on a blocked net looks guilty.
    ('every-owner-gets-the-cells', 'd',
     _WINNER,
     "                for _k in sorted(tally):\n"
     "                    values['blocker_cells'][_k] = (\n"
     "                        values['blocker_cells'].get(_k, 0) + cells)\n",
     (T553,), 'KILLED'),

    # ---- the combination rule ---------------------------------------------
    # Concatenation instead of a sweep: the first signal in SIGNAL_ORDER takes
    # its whole top-k before any other signal is consulted, which is exactly
    # the silent weighting the module refuses to apply.
    ('concat-not-round-robin', 'd',
     _ROUND_ROBIN,
     "    for sig in SIGNAL_ORDER:\n"
     "        while not exhausted and pools.get(sig):\n",
     (T553,), 'KILLED'),

    # ---- refusing to rank ---------------------------------------------------
    # An all-equal signal ranked anyway: the order is then the tie-break alone,
    # which is alphabetical, which is not evidence.
    ('no-spread-ranks-anyway', 'd',
     "        if len(order) > 1 and len(vals) == 1:\n",
     "        if False:\n",
     (T553,), 'KILLED'),

    # ---- attribution honesty ------------------------------------------------
    # A defect pair internal to one block counted once per endpoint, so a block
    # scores double for a defect that is one defect.
    ('internal-pair-counts-twice', 'd',
     "                for key in {k for k in (ka, kb) if k is not None}:\n",
     "                for key in [k for k in (ka, kb) if k is not None]:\n",
     (T553,), 'KILLED'),

    # A signal claiming credit for everything it ranked, not for what was
    # selected -- so `selected_by` stops meaning "why this part is moving".
    ('selected-by-claims-the-whole-pool', 'd',
     _POOLS,
     _POOLS
     + "    for _s, _ks in pools.items():\n"
       "        for _k in _ks:\n"
       "            selected_by.setdefault(_k, []).append(_s)\n",
     (T553,), 'KILLED'),

    # The budget ignored: a block is still added whole, but nothing stops the
    # sweep, so "budget" becomes a label on a number nobody enforces.
    ('budget-ignored', 'd',
     "            if budget is not None and n_parts >= budget:\n",
     "            if False:\n",
     (T553,), 'KILLED'),

    # ---- the disclosure that must travel with the result -------------------
    ('efficacy-claim-emptied', 'd',
     "NO_EFFICACY_CLAIM = (\n"
     "    'NOT MEASURED: no paired routed A/B of pins vs diagnosis exists. '\n",
     "NO_EFFICACY_CLAIM = (\n"
     "    'diagnosis selects the movers that matter. '\n",
     (T553,), 'KILLED'),

    # ---- INERT PROBES, which must change nothing ---------------------------
    # A battery with no expected survivors cannot distinguish "my tests are
    # thorough" from "my tests fail on any edit at all".
    ('a-comment-restates-the-unit', 'd',
     "RANK_DECIMALS = 4\n",
     "RANK_DECIMALS = 4  # decimals, not millimetres\n",
     (T553,), 'SURVIVED'),

    # TOP_K is a REPORT SIZE, not a calibrated threshold, and widening it must
    # not change a fixture where no signal ranks more than 3 candidates. If
    # this ever turns KILLED, TOP_K has quietly become a tuned constant.
    ('top-k-widens', 'd',
     "TOP_K = 3\n",
     "TOP_K = 4\n",
     (T553,), 'SURVIVED'),
]


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
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=600, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += [ln.strip()[5:].strip()
                           for ln in out.splitlines()
                           if ln.strip().startswith('FAIL ')]
                if 'Traceback' in out:
                    failed.append('raised: ' + out.strip().splitlines()[-1][:70])
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:4]))
            print('  ran %-44s %s' % (name, results[-1][1]))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])

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
