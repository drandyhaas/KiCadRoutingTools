#!/usr/bin/env python3
"""The #1182 mutation battery (fa10 P1, phase 4).

`body_model` moves only the neighbour/body currency to check_assembly's
occupancy (`quench._Part.grade_rect` keeps the courtyard ladder for every
intent, zone, keep-out and board question); `place_seed --body-model` hands it
to every search a run builds; `--repair --baseline` charges check_assembly's
gating courtyard pairs and re-grades them; `--reseat` and a declared fixed
pose measure overlap on check_assembly's geometry. Each row breaks one of those
next to the test that must fail.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first.

    python3 -X utf8 tests/mutate_1182.py
    python3 -X utf8 tests/mutate_1182.py --row repair-charges-nothing
    python3 -X utf8 tests/mutate_1182.py --list

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

TARGETS = {'q': os.path.join(_PL, 'quench.py'),
           's': os.path.join(_PL, 'seeder.py'),
           'leg': os.path.join(_PL, 'legality.py'),
           'fp': os.path.join(_PL, 'floorplan.py'),
           'rc': os.path.join(_PL, 'reconstruct.py'),
           'pa': os.path.join(_PL, 'parser.py'),
           'ps': os.path.join(_ROOT, 'py_placer', 'place_seed.py')}

T1182 = os.path.join(_TESTS, 'test_1182_body_model_movers.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # -- the two ladders --
    ('the-grade-ladder-is-occupancy', 'q',
     "        self.grade_by_rot = (self.bounds_by_rot if lb is grade_lb else",
     "        self.grade_by_rot = (self.bounds_by_rot if True else",
     (T1182,), 'KILLED'),
    ('candidate-asks-occupancy', 'q',
     "        grects = part.grade_rects(x, y, rot)",
     "        grects = part.rects(x, y, rot)",
     (T1182,), 'KILLED'),
    ('candidate-board-term-on-occupancy', 'q',
     "        legal = not (grect[0] < self.usable[0] or grect[1] < self.usable[1]",
     "        grect = rect\n        legal = not (grect[0] < self.usable[0] or grect[1] < self.usable[1]",
     (T1182,), 'KILLED'),
    ('pose-ok-asks-occupancy', 's',
     "    # floorplan grade's questions; `candidate_valid` asks the neighbours.\n    r, tht = part.grade_rects(x, y, rot)",
     "    # floorplan grade's questions; `candidate_valid` asks the neighbours.\n    r, tht = part.rects(x, y, rot)",
     (T1182,), 'KILLED'),
    ('the-posed-grader-reads-occupancy', 'fp',
     "                                           rect=part.grade_rect(x, y, rot),",
     "                                           rect=part.rect(x, y, rot),",
     (T1182,), 'KILLED'),
    # -- the repair --
    ('repair-charges-nothing', 's',
     "        _charge(ordered[0], charge_mm)",
     "        pass",
     (T1182,), 'KILLED'),
    ('repair-charges-the-unmoved-member', 's',
     "        mine = [r for r in free if r in (moved or ())] or free",
     "        mine = free",
     (T1182,), 'KILLED'),
    ('repair-ignores-the-baseline', 's',
     "            cy_base = _parse(baseline_file)",
     "            cy_base = None",
     (T1182,), 'KILLED'),
    ('repair-honesty-skipped', 's',
     "    if cy_census is not None and cy_base is not None:",
     "    if False:",
     (T1182,), 'KILLED'),
    ('repair-state-unarmed', 's',
     "        # cleared rather than re-seated onto the same pad-box-legal overlap.\n        body_model=body_model)",
     "        # cleared rather than re-seated onto the same pad-box-legal overlap.\n        body_model=False)",
     (T1182,), 'KILLED'),
    # -- the reseat ruler --
    ('measure-ignores-the-ruler', 'rc',
     "    ov = (overlap(state) if overlap is not None",
     "    ov = (overlap(state) if False",
     (T1182,), 'KILLED'),
    ('reseat-ruler-is-the-search-rects', 's',
     "        return _cy_census.grade({r: (p.x, p.y, p.rot)",
     "        return s.legality_metrics().get('overlap_area', 0.0) if True else _cy_census.grade({r: (p.x, p.y, p.rot)",
     (T1182,), 'KILLED'),
    ('overlap-exact-reads-zero', 'leg',
     "        return round(sum(p.area_mm2 for p in self.pairs",
     "        return 0.0 * round(sum(p.area_mm2 for p in self.pairs",
     (T1182,), 'KILLED'),
    ('reseat-inner-seed-unarmed', 's',
     "        dispose_unseated=False,\n        body_model=body_model,",
     "        dispose_unseated=False,\n        body_model=False,",
     (T1182,), 'KILLED'),
    # -- a declared pose --
    ('fixed-pose-screens-on-the-search-rect', 's',
     "        ra = occupancy_rect_at(state.pcb_data, a, pose_a, ra, pcb_file,",
     "        ra = ra if True else occupancy_rect_at(state.pcb_data, a, pose_a, ra, pcb_file,",
     (T1182,), 'KILLED'),
    # -- the warning --
    ('a-declared-pose-drops-the-drawn-courtyard', 'leg',
     "    if lb is None or (courtyard_less_only and lb.from_courtyard):",
     "    if lb is None:",
     # Armed only since the phase-4 verifier (the unarmed screen is the
     # search's rects), so test_1051_hardening's unarmed case no longer
     # reaches it.
     (T1182, os.path.join(_TESTS, 'test_1051_hardening.py')), 'KILLED'),
    ('the-warning-names-every-part', 'pa',
     "                       if sources.get(r) not in (None, 'pad_bbox'))",
     "                       if False)",
     (T1182,), 'KILLED'),
    # -- the flag reaches every build --
    ('seed-unarmed', 'ps',
     "        decap_claim_after_ics=args.decap_claim_after_ics,\n        body_model=args.body_model)",
     "        decap_claim_after_ics=args.decap_claim_after_ics,\n        body_model=False)",
     (T1182,), 'KILLED'),
    ('polish-unarmed', 'ps',
     "            body_model=args.body_model,\n            corridor_weight=args.corridor_weight,",
     "            body_model=False,\n            corridor_weight=args.corridor_weight,",
     (T1182,), 'KILLED'),
    ('post-polish-reseat-unarmed', 'ps',
     "                    # #1182: the same occupancy the seed and polish used.\n                    body_model=args.body_model)",
     "                    # #1182: the same occupancy the seed and polish used.\n                    body_model=False)",
     (T1182,), 'KILLED'),
    ('repair-call-unarmed', 'ps',
     "                baseline_file=args.baseline,\n                body_model=args.body_model)",
     "                baseline_file=args.baseline,\n                body_model=False)",
     (T1182,), 'KILLED'),
    ('reseat-call-unarmed', 'ps',
     "                decap_claim_after_ics=args.decap_claim_after_ics,\n                body_model=args.body_model)",
     "                decap_claim_after_ics=args.decap_claim_after_ics,\n                body_model=False)",
     (T1182,), 'KILLED'),
    # ---- the phase-4 verifier's cases --------------------------------------
    ('the-unarmed-screen-uses-the-body', 's',
     "    if getattr(state, 'body_model', False):",
     "    if True:",
     (T1182,), 'KILLED'),
    ('the-charge-is-unfloored', 's',
     "                    max(1.0, float(q.depth_mm or 0.0))))",
     "                    float(q.depth_mm or 0.0)))",
     (T1182,), 'KILLED'),
    ('the-charge-drops-the-partners', 's',
     "        out.append((q, sorted(mine, key=mover_key),",
     "        out.append((q, sorted(mine, key=mover_key)[:1],",
     (T1182,), 'KILLED'),
    ('the-regrade-misses-a-created-pair', 's',
     "            elif key not in before and r in moved:",
     "            elif False:",
     (T1182,), 'KILLED'),
    ('the-hint-is-always-printed', 's',
     "    hint = (\"\" if body_model else",
     "    hint = (\"\" if False else",
     (T1182,), 'KILLED'),
    ('the-claim-is-unrecorded', 's',
     "            if 'courtyard_blocking' not in unresolved_claims[r]:",
     "            if False:",
     (T1182,), 'KILLED'),
    ('the-summary-drops-the-gate-counts', 'ps',
     "                'courtyard_gating_before': result.get(",
     "                'courtyard_gating_before_x': result.get(",
     (T1182,), 'KILLED'),
    ('a-missing-baseline-is-accepted', 'ps',
     "        if not os.path.isfile(args.baseline):",
     "        if False:",
     (T1182,), 'KILLED'),
    ('the-reseat-empty-gate-on-rects', 's',
     "        _empty_gate = _recon.measure(state, edge_bands or {},",
     "        _empty_gate = _recon.measure(state, edge_bands or {}, overlap=None) if True else _recon.measure(state, edge_bands or {},",
     (T1182,), 'KILLED'),
    ('the-reseat-before-on-rects', 's',
     "    before = _recon.measure(state, gate_bands, overlap=_cy_overlap)",
     "    before = _recon.measure(state, gate_bands)",
     (T1182,), 'KILLED'),
    ('the-reseat-prune-on-rects', 's',
     "                                     overlap=_cy_overlap,",
     "                                     overlap=None,",
     (T1182,), 'KILLED'),
    ('the-reseat-refusal-on-rects', 's',
     "        after = _recon.measure(state, gate_bands, overlap=_cy_overlap)",
     "        after = _recon.measure(state, gate_bands)",
     (T1182,), 'KILLED'),
    ('the-occupancy-ignores-rotation', 'leg',
     "    lx0, ly0, lx1, ly1 = rotate_local_bounds(*lb.local, rot)\n    return (x + lx0",
     "    lx0, ly0, lx1, ly1 = lb.local\n    return (x + lx0",
     (T1182,), 'KILLED'),
    ('moved-at-ignores-the-poses', 'leg',
     "        x, y, rot = poses.get(ref, (fp.x, fp.y, fp.rotation or 0.0))",
     "        x, y, rot = (fp.x, fp.y, fp.rotation or 0.0)",
     (T1182,), 'KILLED'),
    # A grade question asked of the neighbour currency: the inventory and
    # the armed/unarmed differential, three sites the verifier swapped.
    ('the-zone-gate-on-the-neighbour-rect', 's',
     "            return _rect_inside(part.grade_rect(x, y, rot), constraint, tol)",
     "            return _rect_inside(part.rect(x, y, rot), constraint, tol)",
     (T1182,), 'KILLED'),
    ('the-probe-terms-on-the-neighbour-rects', 'q',
     "        out = intent_term_values(s, self.state.parts[ref].grade_rects()) if s \\",
     "        out = intent_term_values(s, self.state.parts[ref].rects()) if s \\",
     (T1182,), 'KILLED'),
    ('the-fixed-pose-rects-on-the-neighbour', 's',
     "    r, tht = part.grade_rects(x, y, rot)\n    outside = state.edge_gate",
     "    r, tht = part.rects(x, y, rot)\n    outside = state.edge_gate",
     (T1182,), 'KILLED'),
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
