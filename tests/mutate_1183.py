#!/usr/bin/env python3
"""The #1183 mutation battery (fa10 P1, phase 6).

board_score --baseline arms check_assembly's moved-vs-baseline courtyard
gate; an unarmed gate is listed in the top-level `ungraded`; an unreadable
baseline is refused; check_complete forwards it; converge names it when two
scores differ in it; the skill's grade.py grades with it. Each row breaks one
of those next to the test that must fail.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first.

    python3 -X utf8 tests/mutate_1183.py
    python3 -X utf8 tests/mutate_1183.py --row board-score-keeps-the-baseline
    python3 -X utf8 tests/mutate_1183.py --list

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

TARGETS = {'bs': os.path.join(_ROOT, 'py_tools', 'board_score.py'),
           'cv': os.path.join(_ROOT, 'py_placer', 'converge.py'),
           'cc': os.path.join(_ROOT, 'check_complete.py'),
           'gr': os.path.join(_ROOT, '.claude', 'skills', 'pcb-free-agent',
                              'scripts', 'grade.py')}

T1183 = os.path.join(_TESTS, 'test_1183_board_score_baseline.py')
T918 = os.path.join(_TESTS, 'test_918_assembly_verdict.py')
TFA = os.path.join(_TESTS, 'test_pcb_free_agent_scripts.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    ('board-score-keeps-the-baseline', 'bs',
     "    if baseline:\n        args += ['--baseline', baseline]\n    rc, text = run_tool(root, 'check_assembly.py', *args)",
     "    rc, text = run_tool(root, 'check_assembly.py', *args)",
     (T1183,), 'KILLED'),
    ('main-hands-assembly-no-baseline', 'bs',
     "                                  args.clearance, baseline=args.baseline)",
     "                                  args.clearance, baseline=None)",
     (T1183,), 'KILLED'),
    ('an-unarmed-gate-is-not-ungraded', 'bs',
     "                 + (['assembly.courtyard_gating']",
     "                 + ([]",
     (T1183,), 'KILLED'),
    ('an-unreadable-baseline-is-accepted', 'bs',
     "    if args.baseline is not None and not os.path.isfile(args.baseline):",
     "    if False:",
     (T1183,), 'KILLED'),
    ('the-gate-is-not-live', 'bs',
     "                           'pin_in_courtyard', 'courtyard_blocking_gating')",
     "                           'pin_in_courtyard')",
     (T918,), 'KILLED'),
    ('converge-forgets-the-flag', 'cv',
     "                  'assembly.courtyard_gating': '--baseline'}",
     "                  }",
     (T1183,), 'KILLED'),
    ('check-complete-drops-the-baseline', 'cc',
     "                          ('--baseline', a.baseline),",
     "",
     (T1183,), 'KILLED'),
    ('grade-py-grades-unarmed', 'gr',
     "                                        *iflag, '--baseline', baseline,",
     "                                        *iflag,",
     (TFA,), 'KILLED'),
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
