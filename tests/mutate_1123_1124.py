"""The #1123 / #1124 mutation battery: the placement graders and the film read
copper and the grader's census, not boxes.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore the defect each issue measured, or the reason it went unseen:

  * `occupancy-on-the-box` / `overrun-on-the-box` (#1123) -- a custom pad's
    size box is symmetric about its anchor, so its empty side enlarged the
    part's occupancy and its empty corner could read as copper off the board;
  * `posed-box-stale` (#1123) -- `pads_at_pose` turned the copper and kept
    the box, which then missed the copper at any turn but a half one;
  * `film-plots-the-box-pairs` / `floor-from-the-box` (#1124) -- the film
    plotted the quench's bounding-box pairs against a box floor: 10 pairs on
    glasgow_revC where render's checklist names 1, and -- wherever six
    FID/MK box contacts were every pair left (run 32's placed boards and
    every board routed from them) -- "floor 6 = locked parts" for phantoms
    the grader confirms none of.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1123_1124.py
    python3 tests/mutate_1123_1124.py --row posed-box-stale

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`: the
test files run only the cases whose names contain one of the substrings.

Not covered by a row, and why:
  * the `oob_pad_copper_basis` text -- a disclosure; test_937 pins its
    `margin 0` clause, which this does not touch.
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
    'pads': os.path.join(_ROOT, 'py_router', 'check_pads.py'),
    'mp': os.path.join(_ROOT, 'py_router', 'movie_placement.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T1123 = 'test_1123_custom_pad_copper_legality.py'
L_PAD = _t(T1123, 'l_pad_occupies')
CORNER = _t(T1123, 'box_corner_off')
SPIKE = _t(T1123, 'spiked_primitive')
QUARTER = _t(T1123, 'quarter_turn')
OBLIQUE = _t(T1123, 'oblique_turn')
CASTELLATED = _t(T1123, 'castellation_reads')
DISJOINT = _t(T1123, 'disjoint_copper')
T1094 = _t('test_1094_rotated_courtyards.py')
T1124 = 'test_1124_film_grader_census.py'
CHECKLIST = _t(T1124, 'reads_renders_checklist')
GLASGOW_FLOOR = _t(T1124, 'phantom_floor_on_glasgow')
BOX_PHANTOM = _t(T1124, 'box_phantom')
EITHER_SIDE = _t(T1124, 'locked_member')
NO_CTX = _t(T1124, 'not_measured')
CENSUS_ERR = _t(T1124, 'census_error')
NO_OUTLINE = _t(T1124, 'no_outline')
EVERY_PAIR = _t(T1124, 'every_pair_locked')

# (name, target, old, new, tests, expect)
ROWS = [
    # -- #1123: custom pads on their copper ----------------------------------
    ('occupancy-on-the-box', 'leg',
     "            pp = custom_pad_copper(pad)",
     "            pp = None",
     (L_PAD,), 'KILLED'),
    ('overrun-on-the-box', 'leg',
     "            copper = custom_pad_copper(p)",
     "            copper = None",
     (CORNER, CASTELLATED), 'KILLED'),
    ('outline-pads-dropped-from-occupancy', 'leg',
     "                pp = Polygon(pts) if len(pts) >= 3 else None",
     "                pp = None",
     (T1094,), 'KILLED'),
    ('copper-keeps-the-spike', 'pads',
     "    parts = areal_parts(_copper_geometry(pad))",
     "    parts = [_copper_geometry(pad)]",
     (SPIKE,), 'KILLED'),
    ('posed-box-stale', 'leg',
     "            pad.size_x, pad.size_y = _custom_box_at_pose(original, pad, delta)",
     "            pad.size_x, pad.size_y = original.size_x, original.size_y",
     (QUARTER, OBLIQUE), 'KILLED'),
    ('quarter-turn-unswapped', 'leg',
     "            return original.size_y, original.size_x",
     "            return original.size_x, original.size_y",
     (QUARTER,), 'KILLED'),
    ('oblique-box-from-the-file', 'leg',
     "    pts = [pt for poly in posed.polygons for pt in poly]",
     "    return original.size_x, original.size_y",
     (OBLIQUE,), 'KILLED'),
    # -- found surviving by #1123's verifier, now witnessed ---------------
    ('vertices-first-part-only', 'leg',
     "    parts = [geom] if geom.geom_type == 'Polygon' else list(geom.geoms)",
     "    parts = [geom] if geom.geom_type == 'Polygon' else list(geom.geoms)[:1]",
     (DISJOINT,), 'KILLED'),
    ('copper-first-part-only', 'pads',
     "    return parts[0] if len(parts) == 1 else unary_union(parts)",
     "    return parts[0]",
     (DISJOINT,), 'KILLED'),
    ('occupancy-drops-multipolygon', 'leg',
     "            if pp.is_valid and not shape.contains(pp):",
     "            if pp.is_valid and not shape.contains(pp) and pp.geom_type == 'Polygon':",
     (DISJOINT,), 'KILLED'),
    ('posed-custom-tilt-turns', 'leg',
     "            pad.rect_rotation = getattr(original, 'rect_rotation', 0.0)",
     "            pad.rect_rotation = (-delta + 90.0) % 180.0 - 90.0",
     (OBLIQUE,), 'KILLED'),
    ('quarter-tolerance-1deg', 'leg',
     "    if abs(math.remainder(delta, 90.0)) <= 1e-9:",
     "    if abs(math.remainder(delta, 90.0)) <= 1.0:",
     (OBLIQUE,), 'KILLED'),
    # -- #1124: the film's LEGALITY panel reads render's checklist ----------
    ('film-plots-the-box-pairs', 'mp',
     "        'conflict_pairs': len(pairs) if ran else None,",
     "        'conflict_pairs': model.metrics.get('pad_conflict_pairs'),",
     (CHECKLIST, BOX_PHANTOM), 'KILLED'),
    ('floor-from-the-box', 'mp',
     "        'locked_pairs': (sum(1 for a, b, *_rest in pairs",
     "        'locked_pairs': (model.metrics.get('locked_contact_pairs') + sum(0 for a, b, *_rest in pairs",
     (GLASGOW_FLOOR,), 'KILLED'),
    ('overlap-is-the-quench-rects', 'mp',
     "                        else fnd.get('courtyard_overlap_mm2')),",
     "                        else model.metrics.get('overlap_area')),",
     (CHECKLIST,), 'KILLED'),
    ('off-outline-is-the-ranking-list', 'mp',
     "        'off_outline': (len(fnd.get('oob_refs_pad_copper_gating') or [])",
     "        'off_outline': (len(fnd.get('oob_refs_pad_copper') or [])",
     (CHECKLIST,), 'KILLED'),
    ('no-ctx-reads-as-zero', 'mp',
     "    ran = getattr(getattr(model, 'state', None), 'legality_ctx', None) \\",
     "    ran = True or getattr(getattr(model, 'state', None), 'legality_ctx', None) \\",
     (NO_CTX,), 'KILLED'),
    ('census-error-reads-as-zero', 'mp',
     "        'overlap_mm2': (None if fnd.get('courtyard_census_error')",
     "        'overlap_mm2': (None if False",
     (CENSUS_ERR,), 'KILLED'),
    ('locked-member-one-sided', 'mp',
     "                             if a in locked or b in locked)",
     "                             if a in locked)",
     (EITHER_SIDE,), 'KILLED'),
    ('no-outline-reads-zero', 'mp',
     "                        if ran and not getattr(model, 'no_outline', False)",
     "                        if ran",
     (NO_OUTLINE,), 'KILLED'),
    ('floor-any-locked', 'mp',
     "            and lb.conflict_pairs <= lb.locked_pairs):",
     "            and lb.locked_pairs >= 1):",
     (EVERY_PAIR,), 'KILLED'),
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
        p = subprocess.run([sys.executable, '-X', 'utf8', t[0]] + list(t[1:]),
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT)
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
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(o, n, 1))
            try:
                failed = _run_tests(tests)
            finally:
                io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if failed else 'SURVIVED',
                            expect, [str(f)[:150] for f in failed[:2]]))
            print('%-40s %s' % (name, results[-1][1]), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
    wrong = [r for r in results if r[1] != r[2]]
    print('')
    for name, verdict, expect, why in results:
        print('%-40s %-9s%s' % (name, verdict, '' if verdict == expect else
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
