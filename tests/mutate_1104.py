"""The #1104/#1106/#1108/#1109 mutation battery: the defects run 38 found on StickHub.

One row per load-bearing line, each reverting it; every row names the test
that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore a defect somebody measured:

  * `metrics-price-waived-overlap` (#1104) -- run 38's decap repair reverted
    six cap moves that would have cleared their finding, on courtyards the
    project waives;
  * `escape-ignores-pads-under-body` (#1106) -- J9 seated under U1's body.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1104.py
    python3 tests/mutate_1104.py --row escape-ignores-pads-under-body

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending.

Not covered by a row: relocate's and the portfolio swap's waived courtyard
arms (#1104), which no test reaches on a waived board.
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

TARGETS = {
    'leg': os.path.join(_PL, 'legality.py'),
    'seeder': os.path.join(_PL, 'seeder.py'),
    'fp': os.path.join(_PL, 'floorplan.py'),
    'quench': os.path.join(_PL, 'quench.py'),
    'reseat': os.path.join(_PL, 'reseat.py'),
    'pstate': os.path.join(_PL, 'placement_state.py'),
    'rp': os.path.join(_ROOT, 'py_router', 'route_planes.py'),
    'film': os.path.join(_ROOT, 'py_tools', 'make_film.py'),
}


def _t(name):
    return os.path.join(_TESTS, name)


T1104 = _t('test_1104_waiver_every_gate.py')
T1106 = _t('test_1106_pads_under_body.py')
T1108 = _t('test_1108_route_planes_exit.py')
T1109 = _t('test_1109_pile_and_film.py')

# (name, target, old, new, tests, expect)
ROWS = [
    # --- #1104: the courtyard waiver at every gate ---------------------------
    ('metrics-price-waived-overlap', 'quench',
     "            out['overlap_area'] = 0.0",
     "            pass",
     (T1104,), 'KILLED'),
    ('posed-view-drops-waiver', 'fp',
     "        self.courtyards_ignored = getattr(state, 'courtyards_ignored', False)",
     "        self.courtyards_ignored = False",
     (T1104,), 'KILLED'),
    ('violation-parts-prices-courtyards', 'quench',
     "        if getattr(self, 'courtyards_ignored', False):\n"
     "            # #1104: the courtyard is waived, so the overlap term is the pad",
     "        if False:\n"
     "            # #1104: the courtyard is waived, so the overlap term is the pad",
     (T1104,), 'KILLED'),
    ('seated-violations-count-courtyards', 'seeder',
     "            gap = (None if getattr(state, 'courtyards_ignored', False)",
     "            gap = (None if False",
     (T1104,), 'KILLED'),
    ('overlap-at-prices-courtyards', 'seeder',
     "        return {}      # #1104: the project waives courtyard overlap",
     "        pass",
     (T1104,), 'KILLED'),
    ('reseat-clash-on-courtyards', 'reseat',
     "    if getattr(state, 'courtyards_ignored', False):\n"
     "        # #1104: the project waives courtyards; the pad copper is what may",
     "    if False:\n"
     "        # #1104: the project waives courtyards; the pad copper is what may",
     (T1104,), 'KILLED'),
    # --- #1106: pads under a body ------------------------------------------
    ('checker-skips-bodyless', 'leg',
     "                if _f >= CONTAINMENT_FRAC:",
     "                if False:",
     (T1106,), 'KILLED'),
    ('plain-seat-ignores-pads-under-body', 'quench',
     "            if legal and self._pads_under_body_at(ref, x, y, rot, exclude):\n"
     "                legal = False\n"
     "        if legal and self.legality_ctx is not None:",
     "            if False:\n"
     "                legal = False\n"
     "        if legal and self.legality_ctx is not None:",
     (T1106,), 'KILLED'),
    ('escape-ignores-pads-under-body', 'quench',
     "        if self._pads_under_body_at(ref, x, y, rot, exclude):\n"
     "            return False",
     "        if False:\n"
     "            return False",
     (T1106,), 'KILLED'),
    ('body-over-bodyless-neighbour', 'quench',
     "        for other_ref in bodyless:",
     "        for other_ref in ():",
     (T1106,), 'KILLED'),
    # --- #1108: route_planes refusals exit non-zero -------------------------
    ('no-output-exits-zero', 'rp',
     "    return 1 if _no_output else 0",
     "    return 0",
     (T1108,), 'KILLED'),
    ('count-mismatch-not-an-arg-error', 'rp',
     "        parser.error(\n"
     "            f\"number of net arguments",
     "        print(\n"
     "            f\"number of net arguments",
     (T1108,), 'KILLED'),
    # --- #1109: one pile predicate; the film says when it fell back ---------
    ('pile-misses-a-staging-ring', 'pstate',
     "        return bool(self.unplaced or sig.get('s3_outside')",
     "        return bool(self.unplaced or False",
     (T1109,), 'KILLED'),
    ('film-fallback-not-a-warning', 'film',
     "        print(\"make_film: WARNING board box: 2D X-ray, NOT the 3D board -- %s\"",
     "        print(\"make_film: board box: 2D X-ray -- %s\"",
     (T1109,), 'KILLED'),
]

sys.path.insert(0, _TESTS)
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _run_tests(tests):
    failed = []
    for t in tests:
        p = subprocess.run([sys.executable, '-X', 'utf8', t],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT)
        if p.returncode not in (0,):
            failed.append((os.path.basename(t), p.returncode,
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
