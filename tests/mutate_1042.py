#!/usr/bin/env python3
"""The #1042 mutation battery: do the placement panels' tests notice?

Each row breaks one claim the placement panels make -- a copper-free
placement lap put back on the verdict axis, a point per FRAME instead of per
board, the SCREEN label or the instrument dropped, the two arrangement units
put on one scale, the copper-free test inverted, "nothing moved" read as a
placement, the routing hand-off lost, the ledger's floorplan ignored, the
locked-parts floor unlabelled -- next to the test that must fail.

NOT named `test_*.py`: it REWRITES the engine in place. One writer per tree.
It refuses to start while a target has uncommitted changes. Every killer runs
on the UNMUTATED tree first; a battery whose killer cannot pass unmutated
exits 2 rather than scoring every row KILLED.

    python3 -X utf8 tests/mutate_1042.py
    python3 -X utf8 tests/mutate_1042.py --row placement-laps-back-on-the-axis

The table row needs wk/run32 (the ledger's floorplan); without it that row's
killer self-reports the absence and the row SURVIVES -- expected only there.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

MA = os.path.join(_ROOT, 'py_router', 'movie_attempts.py')
MP = os.path.join(_ROOT, 'py_router', 'movie_placement.py')
TARGETS = {'ma': MA, 'mp': MP}

T1042 = os.path.join(_TESTS, 'test_1042_placement_panels.py')
T946 = os.path.join(_TESTS, 'test_946_movie_attempts.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    ('placement-laps-back-on-the-axis', 'ma',
     "        if str(e.get('kind') or '') == 'placement':",
     "        if False:",
     (T1042, T946), 'KILLED'),
    ('a-point-per-frame-not-per-board', 'mp',
     "    _series(d, s, xs, Y, th.rgb('status_best'), visible=cur)",
     "    _series(d, s, xs, Y, th.rgb('status_best'), visible=None)",
     (T1042,), 'KILLED'),
    ('screen-label-dropped', 'mp',
     "        d, sub, 'ARRANGEMENT  SCREEN, not the verdict', _leg,",
     "        d, sub, 'ARRANGEMENT', _leg,",
     (T1042,), 'KILLED'),
    ('instrument-not-named', 'mp',
     "        INSTRUMENT + '  (the verdict is the band)', th, fs, debug)",
     "        '(the verdict is the band)', th, fs, debug)",
     (T1042,), 'KILLED'),
    ('crossings-and-hpwl-on-one-scale', 'mp',
     "        lo, hi = 0.0, max(vals or [1.0]) * 1.08",
     "        lo, hi = 0.0, 12000.0",
     (T1042,), 'KILLED'),
    ('copper-free-test-inverted', 'mp',
     "    return not re.search(",
     "    return not not re.search(",
     (T1042,), 'KILLED'),
    ('nothing-moved-reads-as-a-placement', 'mp',
     "        return any(r['moved'] for r in",
     "        return True or any(r['moved'] for r in",
     (T1042,), 'KILLED'),
    ('routing-hand-off-lost', 'mp',
     "                    frame_h=frame_h, routing=(rf is not None and i >= rf))",
     "                    frame_h=frame_h, routing=False)",
     (T1042,), 'KILLED'),
    ('ledger-floorplan-ignored', 'mp',
     "    for key in ('result_sha', 'parent_sha'):",
     "    for key in ():",
     (T1042,), 'KILLED'),
    ('locked-floor-unlabelled', 'mp',
     "        d.text((x1, fy - fs.size - 2), 'floor %s = locked parts' % _fmt(",
     "        d.text((x1, fy - fs.size - 2), 'floor %s' % _fmt(",
     (T1042,), 'KILLED'),
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
