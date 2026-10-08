#!/usr/bin/env python3
"""The #1162 mutation battery (fa10 P1, phase 5).

The floorplan `legality_budget.overlap_area` is graded on check_assembly's
drawn outlines (`legality.courtyard_overlap_pairs` over the grade's own
universe), the error names its pairs with both readings, the emitter bakes
that reading, a plan's locked pairs are measured on it, and a pose comparison
(`_grade_worse`, the decap rung) may not let it rise. Each row breaks one of
those next to the test that must fail.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first.

    python3 -X utf8 tests/mutate_1162.py
    python3 -X utf8 tests/mutate_1162.py --row the-budget-on-the-rects
    python3 -X utf8 tests/mutate_1162.py --list

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

TARGETS = {'fp': os.path.join(_PL, 'floorplan.py'),
           'leg': os.path.join(_PL, 'legality.py'),
           's': os.path.join(_PL, 'seeder.py')}

T1162 = os.path.join(_TESTS, 'test_1162_overlap_budget_exact.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    ('the-budget-on-the-rects', 'fp',
     "            got = ex",
     "            got = rc",
     (T1162,), 'KILLED'),
    ('the-pairs-unnamed', 'fp',
     "                        'pairs': pairs}",
     "                        'pairs': []}",
     (T1162,), 'KILLED'),
    ('the-emitter-bakes-the-rects', 'fp',
     "        _budget['overlap_area'] = _ceil4(float(_ex))",
     "        _budget['overlap_area'] = _ceil4(float(leg['overlap_area']))",
     (T1162,), 'KILLED'),
    ('the-plan-measures-locked-pairs-on-rects', 'fp',
     "    for a_, b_, _rc, area in sorted(_ctx_overlap_exact(ctx)[2],",
     "    for a_, b_, area, _ex in sorted(_ctx_overlap_exact(ctx)[2],",
     (T1162,), 'KILLED'),
    ('the-pose-grader-forgets-the-reading', 'fp',
     "        out['overlap_area_exact'] = round(",
     "        out['overlap_area_exact_'] = round(",
     (T1162,), 'KILLED'),
    ('the-universe-counts-logos', 'leg',
     "            if not g.synthetic and g.source != SOURCE_SILK:",
     "            if g.source != SOURCE_SILK:",
     (T1162,), 'KILLED'),
    ('the-budget-counts-silk', 'leg',
     "            if not g.synthetic and g.source != SOURCE_SILK:",
     "            if not g.synthetic:",
     (T1162,), 'KILLED'),
    ('overlap-exact-counts-logos', 'leg',
     "        skip = (set(self.synthetic_refs) | set(self.containers)",
     "        skip = (set()",
     (T1162,), 'KILLED'),
    ('a-pose-comparison-skips-the-exact-reading', 's',
     "LEGALITY_COMPARE_KEYS = ('overlap_area', 'overlap_area_exact', 'oob_amount',",
     "LEGALITY_COMPARE_KEYS = ('overlap_area', 'oob_amount',",
     (T1162,), 'KILLED'),
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
