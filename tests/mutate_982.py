#!/usr/bin/env python3
"""The #982 mutation battery: does anything notice when the buckets lie?

`place_seed.split_pad_pairs` decides which pad conflicts in the written board
the seed answers for, and which are against a part it could not seat and were
written at that part's input pose. Every way that split can go wrong while
still looking plausible -- the unseated refs ignored, one side of a pair
checked and not the other, the two buckets swapped, the total not published so
the partition cannot be checked, the console quietly dropping the pairs -- is a
row here, next to the test that must fail when it happens.

Every row carries an EXPECTATION, and a verdict that does not match it is
reported as WRONG. An anchor that does not match its target EXACTLY ONCE is
BROKEN, checked before anything is written (`preflight`, #877).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- run it in a checkout nothing else
is reading. It refuses to start while a target has uncommitted changes,
because it restores by overwriting.

    python3 -X utf8 tests/mutate_982.py
    python3 -X utf8 tests/mutate_982.py --row unseated-refs-ignored
    python3 -X utf8 tests/mutate_982.py --list

A row is KILLED by any non-zero exit of a listed test, a raised error
included -- which is why every killer runs on the UNMUTATED tree first, and
the battery exits 2 if one fails there: a test that cannot run at all exits
non-zero under every mutation too, and would score every row KILLED.
`_uncache` is carried over from `tests/mutate_974.py`: several rows are
same-size edits, and CPython trusts a `.pyc` on (mtime seconds, size).

ONE ROW IS EXPECTED TO SURVIVE, and it is not a test gap:
`gate-charges-the-unseated-pairs` hands the unseated bucket back to
`gate_reason`, which cannot change any exit code. Whenever that bucket is
non-empty something is unseated, and `gate_reason` returns its intent arm --
the same line, the same 4 -- before it looks at pads at all; and when nothing
is unseated the bucket is empty by construction. The row is kept rather than
deleted so that the day the gate's order changes, the battery says so.

THE MEASURED RESULT is recorded below from the run, never predicted.

MEASURED: pending -- this docstring is rewritten with the run's own numbers
before the branch is pushed.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

PLACE_SEED = os.path.join(_ROOT, 'py_placer', 'place_seed.py')
EVIDENCE_MAP = os.path.join(_ROOT, '.claude', 'skills',
                            'plan-pcb-placement-and-routing', 'references',
                            'evidence-map.md')
TARGETS = {'ps': PLACE_SEED, 'em': EVIDENCE_MAP}

T982 = os.path.join(_TESTS, 'test_982_unseated_written_pose.py')
T27S = os.path.join(_TESTS, 'test_run27_seed_gate_shorts.py')
T923 = os.path.join(_TESTS, 'test_923_output_key_claims.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- the split itself ------------------------------------------------
    ('unseated-refs-ignored', 'ps',
     "        (against_unseated if (w[0] in unseated or w[1] in unseated)\n"
     "         else mine).append(w)",
     "        mine.append(w)",
     (T982,), 'KILLED'),
    ('unseated-checked-on-one-side-only', 'ps',
     "        (against_unseated if (w[0] in unseated or w[1] in unseated)\n",
     "        (against_unseated if (w[0] in unseated)\n",
     (T982,), 'KILLED'),
    ('unseated-checked-on-the-other-side-only', 'ps',
     "        (against_unseated if (w[0] in unseated or w[1] in unseated)\n",
     "        (against_unseated if (w[1] in unseated)\n",
     (T982,), 'KILLED'),
    ('buckets-swapped', 'ps',
     "        (against_unseated if (w[0] in unseated or w[1] in unseated)\n"
     "         else mine).append(w)",
     "        (mine if (w[0] in unseated or w[1] in unseated)\n"
     "         else against_unseated).append(w)",
     (T982,), 'KILLED'),
    ('returns-the-two-lists-swapped', 'ps',
     "    return mine, against_unseated",
     "    return against_unseated, mine",
     (T982,), 'KILLED'),
    ('the-boards-own-pairs-are-classified-too', 'ps',
     "        if not (w[0] in seeded or w[1] in seeded):\n"
     "            continue\n",
     "        if False:\n"
     "            continue\n",
     (T982, T27S), 'KILLED'),
    ('seeded-checked-on-one-side-only', 'ps',
     "        if not (w[0] in seeded or w[1] in seeded):\n",
     "        if not (w[0] in seeded):\n",
     (T982,), 'KILLED'),
    ('worst-order-not-kept', 'ps',
     "    for w in worst:\n"
     "        if not (w[0] in seeded or w[1] in seeded):",
     "    for w in reversed(list(worst)):\n"
     "        if not (w[0] in seeded or w[1] in seeded):",
     (T982,), 'KILLED'),

    # ---- the wiring in main() ---------------------------------------------
    ('inherited-does-not-subtract-the-unseated-bucket', 'ps',
     "    _their_pads = max(0, (_pads_out.get('pad_conflicts') or 0)\n"
     "                      - len(_my_pads) - len(_unseated_pads))",
     "    _their_pads = max(0, (_pads_out.get('pad_conflicts') or 0)\n"
     "                      - len(_my_pads))",
     (T982, T27S), 'KILLED'),
    ('unseated-count-reads-the-wrong-list', 'ps',
     "               'pad_conflicts_unseated': len(_unseated_pads),",
     "               'pad_conflicts_unseated': len(_my_pads),",
     (T982,), 'KILLED'),
    ('unseated-pairs-never-published', 'ps',
     "               'pad_conflicts_unseated_pairs': [[a, b, mm]\n"
     "                                                for a, b, mm in _unseated_pads],",
     "               'pad_conflicts_unseated_pairs': [],",
     (T982,), 'KILLED'),
    ('total-not-published', 'ps',
     "               'pad_conflicts_after': _pads_out.get('pad_conflicts') or 0,",
     "               'hole_conflicts_seen': _pads_out.get('hole_conflicts') or 0,",
     (T982, T27S), 'KILLED'),
    ('total-read-off-the-input-board', 'ps',
     "               'pad_conflicts_after': _pads_out.get('pad_conflicts') or 0,",
     "               'pad_conflicts_after': _pads_in.get('pad_conflicts') or 0,",
     (T982, T27S), 'KILLED'),

    # ---- the console: the pairs are copper, so they stay named ------------
    ('console-drops-the-unseated-pairs', 'ps',
     "    if _unseated_pads:\n"
     "        print(f\"  {len(_unseated_pads)} pad conflict(s) against a part this \"",
     "    if False:\n"
     "        print(f\"  {len(_unseated_pads)} pad conflict(s) against a part this \"",
     (T982,), 'KILLED'),
    ('console-says-it-IS-charged', 'ps',
     "              + \" -- reported not charged; seat the part and they go with it\")",
     "              + \" -- charged; seat the part and they go with it\")",
     (T982,), 'KILLED'),

    # ---- the gate: an algebraic no-op, kept as a change detector ----------
    ('gate-charges-the-unseated-pairs', 'ps',
     "    _reason = gate_reason(result['unseated'], own, _my_pads, _hole_delta)",
     "    _reason = gate_reason(result['unseated'], own,\n"
     "                          _my_pads + _unseated_pads, _hole_delta)",
     (T982, T27S), 'SURVIVED'),

    # ---- the documentation gate ------------------------------------------
    ('evidence-map-key-misspelt', 'em',
     "| `pad_conflicts_unseated` / `pad_conflicts_unseated_pairs` |",
     "| `pad_conflicts_unseated` / `pad_conflicts_unseated_pair` |",
     (T923,), 'KILLED'),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _uncache(path):
    """Delete the target's cached bytecode -- see this module's docstring."""
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

    # The battery is only evidence if every killer passes UNMUTATED first.
    for path in TARGETS.values():
        _uncache(path)
    for t in sorted({t for row in rows for t in row[4]}):
        p = subprocess.run([sys.executable, '-X', 'utf8', t],
                           capture_output=True, text=True,
                           encoding='utf-8', errors='replace',
                           timeout=1800, cwd=_ROOT)
        if p.returncode:
            print('BROKEN: %s fails on the UNMUTATED tree (exit %d); every '
                  'verdict below would be meaningless.'
                  % (os.path.basename(t), p.returncode))
            return 2
        print('  unmutated %-40s passes' % os.path.basename(t))

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path, base = TARGETS[tgt], orig[tgt]
            n = base.count(old)
            if n != 1:
                results.append((name, 'BROKEN', expect,
                                'anchor matched %d times' % n, []))
                print('  ran %-52s BROKEN' % name)
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(old, new, 1))
            _uncache(path)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=1800, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += [l.strip()[5:].strip()[:90]
                           for l in out.splitlines()
                           if l.strip().startswith('FAIL')]
                if 'Traceback' in out:
                    failed.append('raised: '
                                  + out.strip().splitlines()[-1][:70])
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            _uncache(path)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:3]))
            print('  ran %-52s %s' % (name, results[-1][1]))
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
        for r in ROWS:
            print('%-52s %-4s %s' % (r[0], r[1], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
