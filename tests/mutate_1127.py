"""The #1127 mutation battery: the placement gate's stack test confirmed on
the pads' outlines (`legality.STACK_EXACT_CONFIRM`, shipped OFF), and the
check_drc caches it relies on.

Kept apart from the other batteries so it can be dropped with the toggle.
The toggle ships off because the pre-registered census
(tests/measure_1127_stack_gate_census.py) was NO-GO, so every row here
guards the ON path a later decision would turn on.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. NOT named `test_*.py`, so `tests/run_all.py` does not
collect it: it REWRITES the sources in place. One writer per tree. It
refuses to start on a dirty target, runs every witness UNMUTATED first,
purges `__pycache__` before each row and runs witnesses with -B.

    python3 tests/mutate_1127.py
    python3 tests/mutate_1127.py --row and-not-stack-dropped

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.

Not covered by a row, and why:
  * `_posed_copper`'s length check: on a part whose copper pads align with
    `pad_rects` (every part, by construction) it never fires -- an
    equivalent mutant.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

TARGETS = {
    'legality': os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py'),
    'drc': os.path.join(_ROOT, 'py_router', 'check_drc.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T = 'test_1127_stack_exact_confirm.py'
GRID = _t(T, 'grid_agrees')
REAL = _t(T, 'real_stack')
LATER = _t(T, 'box_only_hit_after')
CONTACT = _t(T, 'contact_not_nearness')
MODE = _t(T, 'mode_is_fixed')
CACHE = _t(T, 'caches_hold')
IN_PLACE = _t(T, 'changed_in_place')

# (name, target, old, new, tests, expect)
ROWS = [
    ('confirm-never-runs', 'legality',
     "        self.stack_exact = bool(STACK_EXACT_CONFIRM)",
     "        self.stack_exact = False",
     (GRID,), 'KILLED'),
    ('snapshot-never-taken', 'legality',
     "        self.fp_snapshot = _fp_snapshot(fp) if STACK_EXACT_CONFIRM else None",
     "        self.fp_snapshot = None",
     (GRID,), 'KILLED'),
    ('and-not-stack-dropped', 'legality',
     "                if g < 0.0 and not stack:",
     "                if g < 0.0:",
     (LATER,), 'KILLED'),
    ('posed-key-drops-rotation', 'legality',
     "               round(pose[2] % 360.0, 6))",
     "               0)",
     (GRID,), 'KILLED'),
    ('exact-threshold-loose', 'legality',
     "    return bool(hit and over >= eps - 1e-9)",
     "    return bool(hit and over > 0.0)",
     (CONTACT,), 'KILLED'),
    ('confirm-front-layer-only', 'legality',
     "        return _exact_pad_stack(pa_pads[ai], pb_pads[bi], ['F.Cu', 'B.Cu'])",
     "        return _exact_pad_stack(pa_pads[ai], pb_pads[bi], ['F.Cu'])",
     (REAL,), 'KILLED'),
    ('unposable-pad-accepts', 'legality',
     "            return True\n        return _exact_pad_stack(",
     "            return False\n        return _exact_pad_stack(",
     (MODE,), 'KILLED'),
    ('perimeter-cache-serves-another-pad', 'drc',
     "    if hit is not None and hit[2] is pad and hit[0] == fp:",
     "    if hit is not None and hit[0] == fp:",
     (CACHE,), 'KILLED'),
    ('polygon-cache-serves-another-list', 'drc',
     "    if hit is not None and hit[2] is polys and hit[0] == fp:",
     "    if hit is not None and hit[0] == fp:",
     (CACHE,), 'KILLED'),
    ('perimeter-fingerprint-drops-rotation', 'drc',
     "            getattr(pad, 'rect_rotation', 0.0),",
     "            0.0,",
     (IN_PLACE,), 'KILLED'),
    ('perimeter-fingerprint-drops-shape', 'drc',
     "            getattr(pad, 'shape', None),",
     "            None,",
     (IN_PLACE,), 'KILLED'),
    ('perimeter-fingerprint-drops-rratio', 'drc',
     "            getattr(pad, 'roundrect_rratio', None),",
     "            None,",
     (IN_PLACE,), 'KILLED'),
    ('perimeter-fingerprint-drops-polygons', 'drc',
     "            getattr(pad, 'polygons', None))",
     "            None)",
     (IN_PLACE,), 'KILLED'),
]

sys.path.insert(0, _TESTS)
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _purge_pycache():
    """Drop every compiled module under the engine trees before a row is
    applied. Two rows that make same-size edits within one second leave the
    source's (mtime, size) unchanged, so a witness would import the PREVIOUS
    row's bytecode -- a false SURVIVED, measured on this battery's
    overrun-reads-paste-as-copper (mutate_829 has the same guard)."""
    import shutil
    for base in ('py_placer', 'py_router', 'py_tools'):
        for dirpath, dirnames, _files in os.walk(os.path.join(_ROOT, base)):
            if os.path.basename(dirpath) == '__pycache__':
                shutil.rmtree(dirpath, ignore_errors=True)
                dirnames[:] = []


def _run_tests(tests):
    failed = []
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        p = subprocess.run([sys.executable, '-B', '-X', 'utf8', t[0]]
                           + list(t[1:]),
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT, env=env)
        if p.returncode != 0:
            failed.append((os.path.basename(t[0]) + ':' + ','.join(t[1:]),
                           p.returncode,
                           [ln.strip()[:90] for ln in
                            ((p.stdout or '') + (p.stderr or '')).splitlines()
                            if 'FAIL' in ln or 'Error' in ln][:2]))
    return failed


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
    # THE UNMUTATED BASELINE: every witness must pass as the code stands,
    # or a row it "kills" proves nothing.
    witnesses = sorted({t for r in rows for t in r[4]})
    base_fail = _run_tests(witnesses)
    if base_fail:
        print('REFUSING: witnesses fail UNMUTATED -- %s' % base_fail)
        return 2
    print('baseline: %d witnesses pass unmutated' % len(witnesses))
    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            o, n = old, new
            if '\r\n' in base:
                o, n = o.replace('\n', '\r\n'), n.replace('\n', '\r\n')
            if base.count(o) != 1 or o == n:
                results.append((name, 'BROKEN', expect,
                                ['anchor matched %d times' % base.count(o)]))
                continue
            _purge_pycache()
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(o, n, 1))
            try:
                failed = _run_tests(tests)
            finally:
                io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if failed else 'SURVIVED',
                            expect, [str(f)[:150] for f in failed[:2]]))
            print('%-36s %s' % (name, results[-1][1]), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
    wrong = [r for r in results if r[1] != r[2]]
    print('')
    for name, verdict, expect, why in results:
        print('%-36s %-9s%s' % (name, verdict, '' if verdict == expect else
                                '   <-- WRONG, expected %s' % expect))
        for w in why:
            print('      %s' % w)
    print('\n%d rows: %d killed, %d survived, %d broken'
          % (len(results), sum(r[1] == 'KILLED' for r in results),
             sum(r[1] == 'SURVIVED' for r in results),
             sum(r[1] == 'BROKEN' for r in results)))
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', default=None, help='run only this row')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
