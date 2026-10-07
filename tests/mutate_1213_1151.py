#!/usr/bin/env python3
"""The #1213 + #1151 mutation battery (fa10 P1, phase 1).

#1213: `LegalityContext.pair_shortfall` windows its pad sweep and its cap
(`_pad_windows`), so a hollow part -- rp2350's U8, a ring of edge pins -- is
priced by its pads and not its extent; and the no-pose census names the frozen
part that refuses a seat (`frozen_blocks`, `frozen_lifted`, `frozen_alone`).
#1151: `seeder._dispose_unseated` decides where a part the seed could not seat
is written, so it is never written on top of a neighbour.

One row per load-bearing line, each reverting or bending it, next to the test
that must fail. NOT named `test_*.py`, so `tests/run_all.py` does not collect
it: it REWRITES the sources in place. One writer per tree. It refuses to start
on a dirty target, and it runs every witness UNMUTATED first -- a witness that
already fails would score every row as killed.

    python3 -X utf8 tests/mutate_1213_1151.py
    python3 -X utf8 tests/mutate_1213_1151.py --row window-removed
    python3 -X utf8 tests/mutate_1213_1151.py --list

THE MEASURED RESULT is recorded here from the run, never predicted.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
_PL = os.path.join(_ROOT, 'py_placer', 'placement')

TARGETS = {'leg': os.path.join(_PL, 'legality.py'),
           'seed': os.path.join(_PL, 'seeder.py'),
           'ps': os.path.join(_ROOT, 'py_placer', 'place_seed.py')}

T1213 = os.path.join(_TESTS, 'test_1213_pair_window.py')
T1151 = os.path.join(_TESTS, 'test_1151_unseated_disposition.py')
T834 = os.path.join(_TESTS, 'test_834_cap_branch_side.py')
T761 = os.path.join(_TESTS, 'test_761_legality_npth_keepout.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- #1213: the window --------------------------------------------------
    ('window-removed', 'leg',
     "        wa, wb = _pad_windows(rects_a, ea, rects_b, reach)",
     "        wa, wb = list(enumerate(rects_a)), list(enumerate(rects_b))",
     (T1213,), 'KILLED'),
    ('cap-back-on-the-full-product', 'leg',
     "        if len(wa) * len(wb) > PAIR_TEST_CAP:",
     "        if pa.n_pads * pb.n_pads > PAIR_TEST_CAP:",
     (T1213,), 'KILLED'),
    ('sweep-keys-floors-by-window-position', 'leg',
     "        for ai, (a0, a1, a2, a3, na, sa) in wa:",
     "        for ai, (a0, a1, a2, a3, na, sa) in enumerate("
     "r for _i, r in wa):",
     (T1213,), 'KILLED'),
    ('hole-channel-on-the-windowed-lists', 'leg',
     "        return PairShortfall(pad_short, overlap,",
     "        rects_a = [r for _i, r in wa]\n"
     "        rects_b = [r for _j, r in wb]\n"
     "        return PairShortfall(pad_short, overlap,",
     (T1213,), 'KILLED'),

    # ---- #1213: naming the refusing pair -------------------------------------
    ('frozen-census-never-runs', 'seed',
     "            if not baseline and _fz:",
     "            if False:",
     (T1213,), 'KILLED'),
    ('frozen-alone-never-measured', 'seed',
     "                if open_poses:",
     "                if False:",
     (T1213,), 'KILLED'),
    ('verdict-ignores-the-frozen-census', 'seed',
     "    elif _frozen_refusers(census):",
     "    elif False:",
     (T1213,), 'KILLED'),
    ('a-partial-refusal-is-named', 'seed',
     "            if n == 0 and r not in named:",
     "            if n < open_ and r not in named:",
     (T1213,), 'KILLED'),

    # ---- #1151: the disposition ----------------------------------------------
    ('disposition-never-runs', 'seed',
     "    disposition = _dispose_unseated(",
     "    disposition = (lambda *_a, **_k: {})(",
     (T1151,), 'KILLED'),
    ('every-unseated-part-staged', 'seed',
     "            if pose_ok(state, ref, *pose, exclude=undecided):",
     "            if False:",
     (T1151,), 'KILLED'),
    ('a-left-part-is-no-obstacle', 'seed',
     "            undecided = set(todo[i + 1:]) | set(staged)",
     "            undecided = set(todo) - {ref}",
     (T1151,), 'KILLED'),
    ('staged-parts-not-written', 'seed',
     "                  for ref in sorted(set(placed) | staged)]",
     "                  for ref in sorted(set(placed))]",
     (T1151,), 'KILLED'),
    ('staging-row-on-the-board', 'seed',
     "            y = floor + STAGING_GAP_MM - ly0",
     "            y = floor - 3.0 * STAGING_GAP_MM - ly0",
     (T1151,), 'KILLED'),
    ('polish-moves-the-staged-parts', 'ps',
     "            ignore_nets=args.ignore_nets, lock_refs=_staged or None,",
     "            ignore_nets=args.ignore_nets, lock_refs=None,",
     (T1151,), 'KILLED'),
    ('placed-counts-the-staged-parts', 'ps',
     "    summary = {'placed': len(result['placements']) - _n_staged,",
     "    summary = {'placed': len(result['placements']),",
     (T1151,), 'KILLED'),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _uncache(path):
    """Delete the target's cached bytecode: CPython trusts a `.pyc` on
    (mtime seconds, size), and several rows are same-size edits."""
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


def _run_test(t):
    return subprocess.run([sys.executable, '-X', 'utf8', t],
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', timeout=3600, cwd=_ROOT)


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
        p = _run_test(t)
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
                p = _run_test(t)
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
