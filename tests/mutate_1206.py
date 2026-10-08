#!/usr/bin/env python3
"""The #1206 mutation battery (fa10 P1, phase 2).

A through-hole part's far side is one box per CLUSTER of drilled pads
(`legality.far_side_local`, a `FarSide` whose four numbers are still the union
box), read by the grader's pair channel, the seat search, the keep-out test
and the reseat clash check. Each row puts one consumer back on the single box
over all drilled pads -- or breaks the cluster rule -- next to the test that
must fail.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first.

    python3 -X utf8 tests/mutate_1206.py
    python3 -X utf8 tests/mutate_1206.py --row the-union-box-comes-back
    python3 -X utf8 tests/mutate_1206.py --list

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
           'quench': os.path.join(_PL, 'quench.py'),
           'fp': os.path.join(_PL, 'floorplan.py'),
           'reseat': os.path.join(_PL, 'reseat.py')}

T1206 = os.path.join(_TESTS, 'test_1206_far_side_clusters.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    ('the-union-box-comes-back', 'leg',
     "            tht_local = far_side_local(fp)",
     "            tht_local = through_pad_bounds_local(fp)",
     (T1206,), 'KILLED'),
    ('no-pad-ever-clusters', 'leg',
     "FAR_SIDE_CLUSTER_GAP_MM = 2.54",
     "FAR_SIDE_CLUSTER_GAP_MM = -1.0",
     (T1206,), 'KILLED'),
    ('every-pad-clusters', 'leg',
     "FAR_SIDE_CLUSTER_GAP_MM = 2.54",
     "FAR_SIDE_CLUSTER_GAP_MM = 1e9",
     (T1206,), 'KILLED'),
    ('the-gap-ignores-the-clusters', 'leg',
     "        g = far_gap(ra, rb)",
     "        g = rect_gap(ra, rb)",
     (T1206,), 'KILLED'),
    ('the-exact-pass-takes-the-union', 'leg',
     "    ga = pa if pa is not None else far_geom(ra)",
     "    ga = pa if pa is not None else far_geom(tuple(ra))",
     (T1206,), 'KILLED'),
    ('the-search-turns-the-union', 'quench',
     "        self.tht_by_rot = ({r: legality.rotate_far(tlb, r) for r in ROTATIONS}",
     "        self.tht_by_rot = ({r: legality.rotate_local_bounds(*tlb, r) for r in ROTATIONS}",
     (T1206,), 'KILLED'),
    ('the-search-offsets-the-union', 'quench',
     "        return legality.offset_far(b, x, y)",
     "        return (x + b[0], y + b[1], x + b[2], y + b[3])",
     (T1206,), 'KILLED'),
    ('keepout-hit-takes-the-union', 'fp',
     "            hit = max(hit, legality.far_overlap_area(r, entry['rect']))",
     "            hit = max(hit, legality.rect_overlap_area(r, entry['rect']))",
     (T1206,), 'KILLED'),
    ('reseat-clash-takes-the-union', 'reseat',
     "        if b_tht is not None and any(_rects_overlap(a_ct, t)",
     "        if b_tht is not None and any(_rects_overlap(a_ct, b_tht)",
     (T1206,), 'KILLED'),
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
