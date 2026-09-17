#!/usr/bin/env python3
"""The #975 mutation battery: does anything notice when the seat or the hoist lies?

#975 made two changes whose failures look like success. The hoist
(`legality.EdgeCopperContext`) is an IDENTITY, so a broken one still grades --
the wrong pad, a stale rule row, a pad left where it was. The seat ladder
(`seeder._floor_rung` and its two call sites) is a PREFERENCE with a fallback,
so a broken one still seats -- at a pose the grade refuses, or without saying
why. Every such way is a row here, next to the tests that must fail.

Every row carries an EXPECTATION, and a verdict that does not match it is
reported as WRONG. An anchor that does not match its target EXACTLY ONCE is
BROKEN, checked before anything is written (`preflight`, #877).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- run it in a checkout nothing else
is reading. It refuses to start while a target has uncommitted changes,
because it restores by overwriting.

    python3 -X utf8 tests/mutate_975.py
    python3 -X utf8 tests/mutate_975.py --row move-bound-uses-seat-tolerance
    python3 -X utf8 tests/mutate_975.py --list

A row is KILLED by any non-zero exit of a listed test, a raised error
included -- which is why every killer runs on the UNMUTATED tree first, and
the battery exits 2 if one fails there. `_uncache` is carried over from
`tests/mutate_974.py`: several rows are same-size edits, and CPython trusts a
`.pyc` on (mtime seconds, size).

THE MEASURED RESULT is recorded below from the run, never predicted.

MEASURED on the tree of `#975: round-4 verifier` (fa6677f7, 2026-09-17,
Windows; the four killer tests first run unmutated and green): 72 rows, 69
KILLED, 3 SURVIVED, 0 broken. The survivors were `grade-accepts-skips-the-band`
and `move-skips-the-window` (the whole-grade delta caught what the mutated
guard let through, so no test saw the guard alone) and
`grades-a-clear-first-seat` (only stage 1's identity branch had a test). Each
got a test arm at 54698a19, and a 73rd row was added for stage 1's identity;
those four rows, re-run there: 4 KILLED. Earlier full runs: 60 of 60 at
629851d6, and 51 of 54 before that, whose three survivors (a sampled pad's
zero reading, the floor read at the unrounded pose, the silk branch of the
seat basis) likewise got arms rather than waivers.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

LEGALITY = os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py')
SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')
QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
FLOORPLAN = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
PLACE_SEED = os.path.join(_ROOT, 'py_placer', 'place_seed.py')
TARGETS = {'lg': LEGALITY, 'sd': SEEDER, 'qs': QUENCH, 'fp': FLOORPLAN,
           'ps': PLACE_SEED}

TC = os.path.join(_TESTS, 'test_975_edge_copper_context.py')
TS = os.path.join(_TESTS, 'test_975_seat_edge_floor.py')
T967 = os.path.join(_TESTS, 'test_967_edge_floor.py')
T961 = os.path.join(_TESTS, 'test_961_body_overhang.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- the hoist: EdgeCopperContext is the grader, read once --------------
    ('grade-reads-a-pad-snapshot', 'lg',
     "        pads = fp.pads if pose is None else pads_at_pose(fp, pose)",
     "        pads = (fp.__dict__.setdefault('_s975', [__import__('copy').copy(q) for q in fp.pads])\n"
     "                if pose is None else pads_at_pose(fp, pose))",
     (TC,), 'KILLED'),
    ('rules-rows-shared-with-the-context', 'lg',
     "        rules_unmeasured = deepcopy(list(self.rules_unmeasured))",
     "        rules_unmeasured = list(self.rules_unmeasured)",
     (TC,), 'KILLED'),
    ('rules-rows-shallow-copied', 'lg',
     "        rules_unmeasured = deepcopy(list(self.rules_unmeasured))",
     "        rules_unmeasured = [dict(r) for r in self.rules_unmeasured]",
     (TC,), 'KILLED'),
    ('required-coerced-to-float', 'lg',
     "        self.pcb_data, self.path, self.required = pcb_data, source, required",
     "        self.pcb_data, self.path, self.required = pcb_data, source, float(required)",
     (TC,), 'KILLED'),
    ('sampled-miss-reads-its-raw-amount', 'lg',
     "                yield PadCopper(index, pad, amount if hit else 0.0, None, edge,",
     "                yield PadCopper(index, pad, amount if hit else -1.0, None, edge,",
     (TC,), 'KILLED'),
    ('measured-pad-count-dropped', 'lg',
     "                gap = reading.gap_mm\n                measured += 1",
     "                gap = reading.gap_mm",
     (TC, T967), 'KILLED'),
    ('certified-ignores-reasons', 'lg',
     "            if reading.reason:\n                certified = False\n",
     "            if reading.reason:\n                pass\n",
     (TC,), 'KILLED'),
    ('nan-reading-taken-as-worst', 'lg',
     "            if reading.amount_mm != reading.amount_mm:\n                continue\n",
     "",
     (TC,), 'KILLED'),
    ('clears-ignores-eps', 'lg',
     "worst is None or worst <= EPS, certified, tuple(fallback))",
     "worst is None or worst <= 0.0, certified, tuple(fallback))",
     (TC,), 'KILLED'),
    ('pose-no-size-unbake', 'lg',
     "            if abs(base % 180 - 90) <= _PAD_ORTHO_TOL:",
     "            if False:",
     (TC,), 'KILLED'),
    ('pose-tilt-sign-flipped', 'lg',
     "                size_x, size_y, pad.rotation)",
     "                size_x, size_y, -pad.rotation)",
     (TC,), 'KILLED'),
    ('pose-rotation-sense-reversed', 'lg',
     "        pad.global_x, pad.global_y = x + c*a + s*b, y - s*a + c*b",
     "        pad.global_x, pad.global_y = x + c*a - s*b, y + s*a + c*b",
     (TC,), 'KILLED'),
    ('pose-angle-not-advanced', 'lg',
     "        pad.rotation = (base + delta) % 360",
     "        pad.rotation = base",
     (TC,), 'KILLED'),
    ('pose-polygons-not-moved', 'lg',
     "            pad.polygons = [[(x + c*(u-ox) + s*(v-oy), y - s*(u-ox) + c*(v-oy))\n"
     "                             for u, v in poly] for poly in polygons]",
     "            pad.polygons = polygons",
     (TC,), 'KILLED'),
    ('holder-ignores-the-board-object', 'lg',
     "    if hit is None or hit[0] is not pcb_data or hit[1] != key:",
     "    if hit is None or hit[1] != key:",
     (TC,), 'KILLED'),
    ('holder-ignores-the-key', 'lg',
     "    if hit is None or hit[0] is not pcb_data or hit[1] != key:",
     "    if hit is None or hit[0] is not pcb_data:",
     (TC,), 'KILLED'),
    ('holder-forgets-a-failure', 'lg',
     "            hit = (pcb_data, key, None, f'{type(exc).__name__}: {exc}')\n"
     "        holder._edge_copper = hit",
     "            return None, f'{type(exc).__name__}: {exc}'\n"
     "        holder._edge_copper = hit",
     (TC,), 'KILLED'),
    ('legality-argument-label-lost', 'lg',
     "    edge_grade['argument_mm'] = edge_margin",
     "    edge_grade['argument_mm'] = edge_ctx.required",
     (TC,), 'KILLED'),

    # ---- the floor reading the seat asks ------------------------------------
    ('floor-from-the-gate-margin', 'sd',
     "                                state.clearance, state.board_edge_clearance)\n"
     "    fp = state.pcb_data.footprints.get(ref) if ctx is not None else None",
     "                                state.clearance, state.edge_gate.margin)\n"
     "    fp = state.pcb_data.footprints.get(ref) if ctx is not None else None",
     (TS,), 'KILLED'),
    ('floor-read-at-the-unrounded-pose', 'sd',
     "    reading = ctx.pose_copper(fp, (round(x, 3), round(y, 3), rot))",
     "    reading = ctx.pose_copper(fp, (x, y, rot))",
     (TS,), 'KILLED'),
    ('unmeasured-pad-counts-as-short', 'sd',
     "                    if r.amount_mm is not None and r.amount_mm > EPS),",
     "                    if r.amount_mm is None or r.amount_mm > EPS),",
     (TS,), 'KILLED'),
    ('quench-forgets-the-edge-floor', 'qs',
     "        self.board_edge_clearance = board_edge_clearance",
     "        self.board_edge_clearance = clearance",
     (TS,), 'KILLED'),

    # ---- the inward move ------------------------------------------------------
    ('move-other-side-allowed', 'sd',
     "    if sides != {edge}:",
     "    if edge not in sides:",
     (TS,), 'KILLED'),
    ('move-rounding-guard-zero', 'sd',
     "_FLOOR_SHIFT_GUARD_MM = 0.001",
     "_FLOOR_SHIFT_GUARD_MM = 0.0",
     (TS,), 'KILLED'),
    ('move-bound-uses-seat-tolerance', 'sd',
     "    if amount < lo - EPS:",
     "    if amount < lo - 0.02:",
     (TS,), 'KILLED'),
    ('move-bound-dropped', 'sd',
     "    if amount < lo - EPS:",
     "    if False:",
     (TS,), 'KILLED'),
    ('move-setback-guard-dropped', 'sd',
     "    if _carries_setback(entry) and legacy <= EPS:",
     "    if False:",
     (TS,), 'KILLED'),
    ('move-affinity-setback-dropped', 'sd',
     "            or entry.get('class') in ('edge_receptacle', 'connector_affinity'))",
     "            or entry.get('class') in ('edge_receptacle',))",
     (TS,), 'KILLED'),
    ('move-seat-predicate-skipped', 'sd',
     "    if not edge_seat_ok(state, part, sx, sy, edge, lo, hi, reasons=refused):",
     "    if False:",
     (TS,), 'KILLED'),
    ('move-neighbours-skipped', 'sd',
     "    if crowds(sx, sy):",
     "    if False:",
     (TS,), 'KILLED'),
    ('move-still-short-accepted', 'sd',
     "    if moved is None or moved.short:",
     "    if moved is None:",
     (TS,), 'KILLED'),
    ('move-nearest-edge-skipped', 'sd',
     "    if not _faces_its_edge(state, part, entry, edge, sx, sy):",
     "    if False:",
     (TS,), 'KILLED'),
    ('nearest-edge-always-true', 'sd',
     "    return _nearest_edge(rect, tuple(round(v, 6) for v in bounds)) == edge",
     "    return True",
     (TS,), 'KILLED'),

    # ---- _seat_edge's ladder --------------------------------------------------
    ('seat-preference-dropped', 'sd',
     "                if seat is not None and (first is None or seat != (x, y)",
     "                if seat is not None and (True or seat != (x, y)",
     (TS,), 'KILLED'),
    ('seat-walks-on-any-shortfall', 'sd',
     "                if first is not None and first[3] not in _SLIDE_HELPS:",
     "                if False:",
     (TS,), 'KILLED'),
    ('seat-later-rung-skips-the-grade', 'sd',
     "                                         or _grade_accepts(state, part, entry,\n"
     "                                                           edge, lo, x, y)):",
     "                                         or True):",
     (TS,), 'KILLED'),
    ('seat-neighbours-lambda-false', 'sd',
     "                    lambda a, b: not conflict_free(a, b, rot))",
     "                    lambda a, b: False)",
     (TS,), 'KILLED'),
    ('seat-record-dropped', 'sd',
     "            if disclose is not None:\n                disclose[ref] = seat[2]",
     "            if disclose is not None:\n                pass",
     (TS,), 'KILLED'),
    ('repair-record-dropped', 'sd',
     "                            disclose=edge_floor_fallback, grade=pose_grader)",
     "                            disclose=None, grade=pose_grader)",
     (TS,), 'KILLED'),
    ('repair-grade-not-passed', 'sd',
     "                            disclose=edge_floor_fallback, grade=pose_grader)",
     "                            disclose=edge_floor_fallback, grade=None)",
     (TS,), 'KILLED'),

    # ---- stage 1 --------------------------------------------------------------
    ('stage1-walks-on-any-shortfall', 'sd',
     "                        if (_kept is not None and (_kept[2] or {}).get('why')",
     "                        if (False and (_kept[2] or {}).get('why')",
     (TS,), 'KILLED'),
    ('stage1-later-rung-skips-the-grade', 'sd',
     "                                or _grade_accepts(state, part, c, edge, lo, _x, _y)):",
     "                                or True):",
     (TS,), 'KILLED'),
    ('stage1-kept-frac-not-restored', 'sd',
     "                if _kept is not None:\n                    frac = _kept[0]",
     "                if _kept is not None:\n                    pass",
     (TS,), 'KILLED'),
    ('stage1-neighbours-lambda-false', 'sd',
     "                            _x, _y, lambda a, b: bool(_shorted_by(a, b)))",
     "                            _x, _y, lambda a, b: False)",
     (TS,), 'KILLED'),
    ('stage1-record-dropped', 'sd',
     "                    edge_floor_fallback[ref] = _record",
     "                    pass",
     (TS,), 'KILLED'),
    ('stage1-context-note-dropped', 'sd',
     "    if by_edge:\n        _floor_context_note(state, notes)",
     "    if False:\n        _floor_context_note(state, notes)",
     (TS,), 'KILLED'),
    ('record-kept-at-any-written-pose', 'sd',
     "                and min(turn, 360.0 - turn) < 1e-3):",
     "                or True):",
     (TS,), 'KILLED'),
    ('record-rotation-compared-exactly', 'sd',
     "                and min(turn, 360.0 - turn) < 1e-3):",
     "                and min(turn, 360.0 - turn) < 1e-9):",
     (TS,), 'KILLED'),

    # ---- the grade's own conjuncts, asked of every preferred pose -------------
    ('grade-accepts-skips-the-band', 'sd',
     "    return (_grade_band_refuses(state, part, entry, edge, lo, x, y)[0] is None\n"
     "            and _faces_its_edge(state, part, entry, edge, x, y)\n",
     "    return (True\n"
     "            and _faces_its_edge(state, part, entry, edge, x, y)\n",
     (TS,), 'KILLED'),
    ('grade-band-max-dropped', 'sd',
     "    if hi is not None and amount > float(hi) + EPS:",
     "    if False:",
     (TS,), 'KILLED'),
    ('grade-accepts-skips-the-window', 'sd',
     "            and not _outside_its_along_edge_claim(state, part, entry, edge, x, y))",
     "            )",
     (TS,), 'KILLED'),
    ('move-skips-the-window', 'sd',
     "    if _outside_its_along_edge_claim(state, part, entry, edge, sx, sy):",
     "    if False:",
     (TS,), 'KILLED'),
    ('band-max-at-the-seat-tolerance', 'sd',
     "    if hi is not None and amount > float(hi) + EPS:",
     "    if hi is not None and amount > float(hi) + 0.02:",
     (TS,), 'KILLED'),
    ('walk-after-a-band-refusal', 'sd',
     "_SLIDE_HELPS = frozenset(('along_edge', 'outline_sampled'))",
     "_SLIDE_HELPS = frozenset(('along_edge', 'outline_sampled', 'band_min'))",
     (TS,), 'KILLED'),
    ('no-walk-on-a-sampled-outline', 'sd',
     "_SLIDE_HELPS = frozenset(('along_edge', 'outline_sampled'))",
     "_SLIDE_HELPS = frozenset(('along_edge',))",
     (TS,), 'KILLED'),
    ('stage1-later-rung-asks-only-the-nearest-edge', 'sd',
     "                                or _grade_accepts(state, part, c, edge, lo, _x, _y)):",
     "                                or _faces_its_edge(state, part, c, edge, _x, _y)):",
     (TS,), 'KILLED'),
    ('window-read-at-the-unrounded-pose', 'sd',
     "    probe = SimpleNamespace(rect=part.rect(round(x, 3), round(y, 3), part.rot))",
     "    probe = SimpleNamespace(rect=part.rect(x, y, part.rot))",
     (TS,), 'KILLED'),
    ('window-ignores-a-centre-claim', 'sd',
     "    if entry.get('center_on_edge') is None and entry.get('along_edge_band') is None:",
     "    if entry.get('along_edge_band') is None:",
     (TS,), 'KILLED'),

    # ---- the whole-grade delta -------------------------------------------------
    ('grade-delta-off', 'sd',
     "    if grade is None:\n        return ()\n    from placement import floorplan as _fp",
     "    if True:\n        return ()\n    from placement import floorplan as _fp",
     (TS,), 'KILLED'),
    ('grade-failure-taken', 'sd',
     "        return ({'unavailable': f'{type(exc).__name__}: {exc}'},)",
     "        return ()",
     (TS,), 'KILLED'),
    ('grades-a-clear-first-seat', 'sd',
     "                if seat is not None and first is None and seat == (x, y):",
     "                if False:",
     (TS,), 'KILLED'),
    ('stage1-grades-a-clear-first-seat', 'sd',
     "                        if _seat is not None and _kept is None and _seat == (_x, _y):",
     "                        if False:",
     (TS,), 'KILLED'),
    ('later-rung-graded-against-itself', 'sd',
     "                                         (x, y) if first is None else first[:2],",
     "                                         (x, y),",
     (TS,), 'KILLED'),
    ('stage1-pile-not-left-out', 'sd',
     "                                set(unplaced) - {ref}, _graded)",
     "                                set(), _graded)",
     (TS,), 'KILLED'),
    ('delta-counts-warnings', 'fp',
     "    was = [v for v in before if v.severity == ERROR]\n"
     "    now = [v for v in after if v.severity == ERROR]",
     "    was = list(before)\n"
     "    now = list(after)",
     (TS,), 'KILLED'),
    ('delta-reads-only-a-refd-claim', 'fp',
     "        return (v.rule, v.ref or '', v.block or '',\n"
     "                tuple(sorted((v.expected or {}).keys())))",
     "        return (v.rule, v.ref or '', v.block or '',\n"
     "                tuple(sorted((v.expected or {}).keys()))) if v.ref == 'J1' else ('',)",
     (TS,), 'KILLED'),
    ('budget-growth-ignored', 'fp',
     "                    and b > a + legality.EPS):",
     "                    and False):",
     (TS,), 'KILLED'),
    ('posed-view-keeps-the-pile', 'fp',
     "            if ref in self._exclude:\n                continue\n            x, y, rot = self.pose(ref)",
     "            if False:\n                continue\n            x, y, rot = self.pose(ref)",
     (TS,), 'KILLED'),
    ('nearest-edge-on-the-courtyard-only', 'sd',
     "    rect, _basis = edge_seat_rect(entry, part.rect(px, py, part.rot), body)",
     "    rect, _basis = part.rect(px, py, part.rot), 'courtyard'",
     (TS,), 'KILLED'),

    # ---- the rule's seat basis, now shared ------------------------------------
    ('edge-seat-rect-ignores-silk', 'fp',
     "        if brect is not None and src in ('fab', 'silk'):",
     "        if brect is not None and src in ('fab',):",
     (T961, TS), 'KILLED'),

    # ---- place_seed -----------------------------------------------------------
    ('ps-fresh-key-dropped', 'ps',
     "               'edge_floor_fallback': result.get('edge_floor_fallback') or {},",
     "",
     (TS,), 'KILLED'),
    ('ps-repair-record-not-merged', 'ps',
     "            summary['edge_floor_fallback'].update(\n"
     "                result.get('edge_floor_fallback') or {})",
     "            pass",
     (TS,), 'KILLED'),
    ('ps-written-pose-filter-dropped', 'ps',
     "    summary['edge_floor_fallback'] = seeder.floor_records_at_poses(\n"
     "        summary['edge_floor_fallback'],",
     "    summary['edge_floor_fallback'] = (lambda records, poses: records)(\n"
     "        summary['edge_floor_fallback'],",
     (TS,), 'KILLED'),
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
