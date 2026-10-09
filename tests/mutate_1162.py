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
T959 = os.path.join(_TESTS, 'test_959_plan_check.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    ('the-budget-on-the-rects', 'fp',
     "            got = ex",
     "            got = rc",
     (T1162,), 'KILLED'),
    ('the-pairs-unnamed', 'fp',
     "                        'pairs': pairs[:MEASURED_PAIRS_CAP],",
     "                        'pairs': [],",
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
     "        if g.synthetic:",
     "        if False:",
     (T1162,), 'KILLED'),
    ('the-budget-counts-silk', 'leg',
     "        elif g.source == SOURCE_SILK:",
     "        elif False:",
     (T1162,), 'KILLED'),
    ('overlap-exact-counts-logos', 'leg',
     "        skip = (set(self.synthetic_refs) | set(self.containers)",
     "        skip = (set()",
     (T1162,), 'KILLED'),
    ('a-pose-comparison-skips-the-exact-reading', 's',
     "LEGALITY_COMPARE_KEYS = ('overlap_area', 'overlap_area_exact', 'oob_amount',",
     "LEGALITY_COMPARE_KEYS = ('overlap_area', 'oob_amount',",
     (T1162,), 'KILLED'),
    # ---- the phase-5 verifier's cases --------------------------------------
    ('the-zone-bound-on-rects', 'fp',
     "                and excess_x > float(overlap_budget) + legality.EPS):",
     "                and excess > float(overlap_budget) + legality.EPS):",
     (T1162,), 'KILLED'),
    ('the-zone-bound-drops-the-far-face', 'fp',
     "                    per_face_x[far] = per_face_x.get(far, 0.0) + (",
     "                    per_face_x[far] = per_face_x.get(far, 0.0) + 0.0 * (",
     (T959,), 'KILLED'),
    ('the-zone-bound-skips-a-locked-member', 'fp',
     "        b0 = st.parts[r].grade_by_rot[0.0]",
     "        if st.parts[r].locked:\n            continue\n        b0 = st.parts[r].grade_by_rot[0.0]",
     (T1162,), 'KILLED'),
    ('the-board-bound-on-rects', 'fp',
     "            and excess0_x > float(overlap_budget) + legality.EPS:",
     "            and excess0 > float(overlap_budget) + legality.EPS:",
     (T1162,), 'KILLED'),
    ('the-pair-count-dropped', 'fp',
     "                        'pairs_total': len(pairs),",
     "                        'pairs_total': 0,",
     (T1162,), 'KILLED'),
    ('the-excluded-uncounted', 'fp',
     "            ctx.legality['overlap_area_excluded'] = sum(",
     "            ctx.legality['overlap_area_excluded'] = 0 * sum(",
     (T1162,), 'KILLED'),
    ('a-waived-project-prices-exact-overlap', 'fp',
     "    if getattr(st, 'courtyards_ignored', False):\n        ctx._overlap_exact = (0.0, 0.0, [])",
     "    if False:\n        ctx._overlap_exact = (0.0, 0.0, [])",
     (T1162,), 'KILLED'),
    ('the-posed-view-prices-the-frame', 'fp',
     "        self.container_refs = getattr(state, 'container_refs', ()) or ()",
     "        self.container_refs = ()",
     (T1162,), 'KILLED'),
    ('the-decap-rung-ignores-the-exact-key', 's',
     "            was, now = leg0.get(key), leg1.get(key)",
     "            was, now = ((leg0.get(key), leg1.get(key)) if key != 'overlap_area_exact' else (0, 0))",
     (T1162,), 'KILLED'),
    ('the-rect-reading-ignores-the-caller', 'leg',
     "            if rect_of is not None:",
     "            if False:",
     (T1162,), 'KILLED'),
    ('the-pairs-unsorted', 'leg',
     "    pairs.sort(key=lambda p: (-p[3], -p[2], p[0], p[1]))",
     "    pass",
     (T1162,), 'KILLED'),
    # ---- the verifier on 8492b3c1 -----------------------------------------
    ('the-plan-prices-a-waived-project', 'fp',
     "        overlap_budget = None",
     "        pass",
     (T1162,), 'KILLED'),
    ('the-zone-bound-ignores-a-declared-angle', 'fp',
     "    elif rot is not None:",
     "    elif False:",
     (T1162,), 'KILLED'),
    ('the-zone-bound-ignores-the-candidates', 'fp',
     "        rots = list(cands)",
     "        rots = [part.rot]",
     (T1162,), 'KILLED'),
    ('the-zone-bound-turns-a-locked-part', 'fp',
     "    if getattr(part, 'locked', False) or (rot is None and not cands):",
     "    if (rot is None and not cands):",
     (T1162,), 'KILLED'),
    ('the-zone-bound-reads-only-its-own-block', 'fp',
     "            if _anchor_reachable(z, part, tol, _claims.get(r)):",
     "            if _anchor_reachable(z, part, tol, (z.rotation, z.rotation_candidates)):",
     (T1162,), 'KILLED'),
    ('the-zone-bound-reads-the-rotation-cache', 'fp',
     "            zone.rect, rotate_local_bounds(*part.grade_by_rot[0.0], r % 360),",
     "            zone.rect, part.grade_rect(0.0, 0.0, r % 360),",
     (T1162,), 'KILLED'),
    ('the-zone-bound-ignores-the-anchor', 'fp',
     "            if _anchor_reachable(z, part, tol, _claims.get(r)):",
     "            if False:",
     (T1162,), 'KILLED'),
    ('contradictory-claims-bound-on-the-first-claim-only', 'fp',
     "    for _r, _seen in _conflicts.items():",
     "    for _r, _seen in {}.items():",
     (T1162,), 'KILLED'),
    ('contradictory-claims-raise-in-the-plan', 'fp',
     "    _claims = rotations_for_ref(intent, blocks, conflicts=_conflicts)",
     "    _claims = rotations_for_ref(intent, blocks)",
     (T1162,), 'KILLED'),
    ('the-excluded-are-not-named', 'fp',
     "                        'excluded': {k: v for k, v in _excl.items() if v}}",
     "                        'excluded': {}}",
     (T1162,), 'KILLED'),
    ('the-pair-cap-lifted', 'fp',
     "MEASURED_PAIRS_CAP = 50",
     "MEASURED_PAIRS_CAP = 10 ** 9",
     (T1162,), 'KILLED'),
    ('a-container-is-not-named', 'leg',
     "            excluded['containers'].append(r)",
     "            pass",
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
