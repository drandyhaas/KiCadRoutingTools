"""The #1094-#1099 mutation battery: the checker and placement defects run 36
found on KiCad's StickHub demo.

One row per load-bearing line, each reverting it; every row names the test
that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore a defect somebody measured:

  * `graded-parts-lose-their-outline` (#1094) -- 74 phantom courtyard pairs
    and six phantom containments made the human StickHub NOT BUILDABLE;
  * `off-outline-not-a-conjunct` (#1096) -- run 36 routed with C20 7.84 mm
    below the board on a `buildable`;
  * `plug-keepout-front-only` (#1098) -- 8 back-side parts on the USB tongue;
  * `unseated-parts-unnamed` (#1099) -- the exit line that let C20 through.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1094.py
    python3 tests/mutate_1094.py --row off-outline-not-a-conjunct

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending.
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
    'parser': os.path.join(_PL, 'parser.py'),
    'seeder': os.path.join(_PL, 'seeder.py'),
    'fp': os.path.join(_PL, 'floorplan.py'),
    'quench': os.path.join(_PL, 'quench.py'),
    'asm': os.path.join(_ROOT, 'py_tools', 'check_assembly.py'),
    'seed': os.path.join(_ROOT, 'py_placer', 'place_seed.py'),
    'pose': os.path.join(_PL, 'pose_ops.py'),
    'cfp': os.path.join(_ROOT, 'py_tools', 'check_floorplan.py'),
    # #1111 moved overlap_thickness here, verbatim, for check_pads.
    'geom': os.path.join(_ROOT, 'py_router', 'geometry_utils.py'),
}


def _t(name):
    return os.path.join(_TESTS, name)


T1094 = _t('test_1094_rotated_courtyards.py')
T1095 = _t('test_1095_project_severity.py')
T1096 = _t('test_1096_off_outline_gates.py')
T1098 = _t('test_1098_mating_keepout.py')
T1099 = _t('test_1099_seed_gaps.py')
T1100 = _t('test_1100_1103_run37_followups.py')
T1101 = _t('test_1101_courtyard_waiver_seat.py')

# (name, target, old, new, tests, expect)
ROWS = [
    # --- #1094: courtyards and bodies as drawn -------------------------------
    ('graded-parts-lose-their-outline', 'leg',
     "                              poly=occupancy_shape(fp, lb, bodies.get(ref)),",
     "                              poly=None,",
     (T1094,), 'KILLED'),
    ('fab-bodies-back-to-boxes', 'leg',
     "                ov, _depth, _ix = shape_overlap(sha, shb)",
     "                from shapely.geometry import box as _bx; ov, _depth, _ix = shape_overlap(_bx(*rca), _bx(*rcb))",
     (T1094,), 'KILLED'),
    ('containment-on-box-areas', 'leg',
     "                    _cf = containment_frac_of_areas(ov, sha.area, shb.area)",
     "                    _cf = containment_frac(ov, rca, rcb)",
     (T1094,), 'KILLED'),
    ('occupancy-without-its-pads', 'leg',
     "            if pp.is_valid and not shape.contains(pp):",
     "            if False:",
     (T1094,), 'KILLED'),
    ('pad-turn-grows-the-seed-box', 'leg',
     "                    HX, HY = px * tc + py * ts, px * ts + py * tc",
     "                    HX, HY = hx * tc + hy * ts, hx * ts + hy * tc",
     (T1094,), 'KILLED'),
    ('fixed-pose-seat-on-boxes', 'seeder',
     "        if ga.poly is not None or gb.poly is not None:",
     "        if False:",
     (T1094,), 'KILLED'),
    ('open-outline-no-hull', 'parser',
     "            shape, how = MultiPoint(pts).convex_hull, OUTLINE_HULL",
     "            shape, how = shape, OUTLINE_HULL",
     (T1094,), 'KILLED'),
    ('micron-gap-not-joined', 'parser',
     "            for cand_lines in (ends, shapely.snap(ml, ml, _OUTLINE_JOIN_MM)):",
     "            for cand_lines in ():",
     (T1094,), 'KILLED'),
    # --- #1095: the project's courtyard severity -----------------------------
    ('project-ignore-not-read', 'leg',
     "                                if self.severity == 'ignore' else '')",
     "                                if False else '')",
     (T1095,), 'KILLED'),
    ('project-ignore-stops-at-a-lock', 'leg',
     "        if p.waiver.startswith(PROJECT_SEVERITY_WAIVER):",
     "        if False:",
     (T1095,), 'KILLED'),
    ('legacy-tool-ignore-trusted', 'leg',
     "        if all(saved_all.get(c, sev.get(c)) == 'ignore' for c in legacy):",
     "        if False:",
     (T1095,), 'KILLED'),
    # The PR review: an author's ignore followed by a current relax read as
    # the legacy plan (glasgow_revC 0 -> 21 courtyard-blocking pairs).
    ('legacy-check-blind-to-saved', 'leg',
     "        if all(saved_all.get(c, sev.get(c)) == 'ignore' for c in legacy):",
     "        if all(sev.get(c) == 'ignore' for c in legacy):",
     (T1095,), 'KILLED'),
    # --- #1094 review: how deep, and what a courtyard encloses ---------------
    ('depth-spans-the-rotated-rect', 'geom',
     "            best = max(best, min(short, 2.0 * _inscribed_radius(p)))",
     "            best = max(best, short)",
     (T1094,), 'KILLED'),
    ('thickness-fallback-unmeasured', 'geom',
     "        polylabel(poly, tolerance=THICKNESS_TOL_MM)))",
     "        polylabel(poly, tolerance=THICKNESS_TOL_MM))) * 0.0",
     (T1094,), 'KILLED'),
    ('courtyard-rings-filled', 'parser',
     "                                   even_odd=True)",
     "                                   even_odd=False)",
     (T1094,), 'KILLED'),
    ('fab-inner-circle-a-hole', 'parser',
     "    compose = _nested_even_odd if even_odd else unary_union",
     "    compose = _nested_even_odd",
     (T1094,), 'KILLED'),
    # --- #1096: pad copper off the outline -----------------------------------
    ('off-outline-not-a-conjunct', 'asm',
     "                         or courtyard_gating or off_outline_pads or mating",
     "                         or courtyard_gating or mating",
     (T1096,), 'KILLED'),
    ('overrun-printed-as-the-sum', 'asm',
     "                      + ', '.join(f'{r} ({_overrun.get(r, a)}mm past the '",
     "                      + ', '.join(f'{r} ({a}mm past the '",
     (T1096,), 'KILLED'),
    ('castellated-pads-gate', 'leg',
     "        if getattr(p, 'castellated', False) and any(",
     "        if False and any(",
     (T1096,), 'KILLED'),
    ('castellated-exempt-off-the-board', 'leg',
     "                _on_board(gate, x, y) for x, y in pts):",
     "                True for x, y in pts):",
     (T1096,), 'KILLED'),
    # --- #1098: a PCB-edge plug's mating region ------------------------------
    ('plug-keepout-front-only', 'fp',
     "                    'rect': tuple(round(v, 4) for v in ins),\n"
     "                    'sides': ('F', 'B'),",
     "                    'rect': tuple(round(v, 4) for v in ins),\n"
     "                    'sides': ('F',),",
     (T1098,), 'KILLED'),
    ('plug-slot-not-allowed', 'fp',
     "                    'allow': (_glob.escape(ref),) + free,",
     "                    'allow': (_glob.escape(ref),),",
     (T1098,), 'KILLED'),
    ('net-tie-read-as-a-plug', 'fp',
     "        if getattr(fp, 'net_tie_groups', None):",
     "        if False:",
     (T1098,), 'KILLED'),
    ('plug-not-a-conjunct', 'asm',
     "                         or courtyard_gating or off_outline_pads or mating",
     "                         or courtyard_gating or off_outline_pads",
     (T1098,), 'KILLED'),
    ('grade-blind-to-the-plug', 'fp',
     "        if len(_ks) != len(intent.keepouts or ()):",
     "        if False:",
     (T1098,), 'KILLED'),
    ('seated-plug-free-to-move', 'quench',
     "            if _ref in self.parts:\n"
     "                self.parts[_ref].locked = True",
     "            if False:\n"
     "                self.parts[_ref].locked = True",
     (T1098,), 'KILLED'),
    ('place-pose-moves-the-plug', 'pose',
     "        if _moved_plugs:",
     "        if False:",
     (T1098,), 'KILLED'),
    ('fingers-need-not-reach-the-edge', 'fp',
     "    if near < 2:\n"
     "        return None",
     "    if False:\n"
     "        return None",
     (T1098,), 'KILLED'),
    ('seat-blind-to-the-plug', 'quench',
     "        self.keepouts = _fpk.with_derived_keepouts(keepouts, pcb_data,",
     "        self.keepouts = tuple(keepouts or ()) or _fpk.with_derived_keepouts((), None,",
     (T1098,), 'KILLED'),
    # --- #1099: the seed ------------------------------------------------------
    ('unseated-parts-unnamed', 'seed',
     "    if unseated:",
     "    if False:",
     (T1099,), 'KILLED'),
    ('decaps-from-ignored', 'fp',
     "    if decaps_from:",
     "    if False:",
     (T1099,), 'KILLED'),
    ('no-diagonal-pass', 'seeder',
     "                        for d in (45.0, 135.0, 225.0, 315.0)])",
     "                        for d in ()])",
     (T1099,), 'KILLED'),
    # --- #1100-#1103: run 37's follow-ups -------------------------------------
    ('new-pair-at-a-tie-accepted', 'pose',
     "            if fresh:",
     "            if False:",
     (T1100,), 'KILLED'),
    ('pile-edges-read-off-poses', 'fp',
     "        amt = state.edge_gate.rect_outside_amount(parts[ref].rect)\n"
     "        if _pile and ref not in _pile_locked:",
     "        amt = state.edge_gate.rect_outside_amount(parts[ref].rect)\n"
     "        if False:",
     (T1100,), 'KILLED'),
    ('pin-limit-not-derived', 'fp',
     "            _decaps['max_pin_distance_mm'] = _pin_limit",
     "            pass",
     (T1100,), 'KILLED'),
    ('withheld-pin-limit-is-debt', 'fp',
     "            _census['pin_limit_withheld'] = (",
     "            _withheld['decaps.max_pin_distance_mm'] = (",
     (T1100,), 'KILLED'),
    ('waived-courtyards-still-refused', 'quench',
     "        if legal and getattr(self, 'courtyards_ignored', False):",
     "        if False:",
     (T1101,), 'KILLED'),
    ('waived-seat-prices-at-seat-clearance', 'quench',
     "                        if (sf.pad > EPS_IMPROVE or sf.pad_overlap",
     "                        if (False and sf.pad > EPS_IMPROVE or sf.pad_overlap",
     (T1101,), 'KILLED'),
    ('escape-branch-skips-bodies', 'quench',
     "        if self._body_contained_at(ref, x, y, rot, exclude):\n"
     "            return False\n"
     "        # #1106: and the body-less half of it.",
     "        if False:\n"
     "            return False\n"
     "        # #1106: and the body-less half of it.",
     (T1101,), 'KILLED'),
    ('waived-seat-stacks-holes', 'quench',
     "                            and _drill_conflict(",
     "                            and False and _drill_conflict(",
     (T1101,), 'KILLED'),
    ('waived-fixed-pose-stacks-holes', 'seeder',
     "            elif getattr(state, 'courtyards_ignored', False):",
     "            elif False:",
     (T1101,), 'KILLED'),
    ('census-counts-the-other-face', 'seeder',
     "        if not (part.sides & op.sides):",
     "        if False:",
     (T1101,), 'KILLED'),
    # The #1101 review: with the courtyard waived only containment kept
    # bodies apart (StickHub from a pile: 14 .Fab overlaps, C17 45% in J2).
    ('waived-seat-stacks-bodies', 'quench',
     "            if legal and self._body_overlap_at(ref, x, y, rot, exclude):",
     "            if False:",
     (T1101,), 'KILLED'),
    ('waived-body-on-its-box', 'quench',
     "            if mine.intersection(theirs).area > _BODY_OVERLAP_EPS:",
     "            if True:",
     (T1101,), 'KILLED'),
    # --- #1098/#1099 review follow-ups ----------------------------------------
    ('jumper-read-as-a-plug', 'fp',
     "        if sum(1 for p in pads if p.net_id) < MATING_MIN_FINGERS:",
     "        if sum(1 for p in pads if p.net_id) < 2:",
     (T1098,), 'KILLED'),
    ('plug-seated-across-an-edge', 'fp',
     "    if legality.pad_copper_overrun_mm(pads, gate) > legality.EPS:",
     "    if False:",
     (T1098,), 'KILLED'),
    ('declared-region-not-graded', 'leg',
     "                                         declared=declared_keepouts)",
     "                                         declared=())",
     (T1098,), 'KILLED'),
    ('assembly-drops-declared-region', 'asm',
     "            declared_keepouts = tuple(_intent.keepouts or ())",
     "            declared_keepouts = ()",
     (T1098,), 'KILLED'),
    ('place-pose-drops-declared-region', 'pose',
     "    declared_keepouts = tuple(getattr(intent, 'keepouts', None) or ())",
     "    declared_keepouts = ()",
     (T1098,), 'KILLED'),
    ('pile-plug-locked', 'quench',
     "        for _ref in _fpk.seated_plugs(self.keepouts, pcb_data, pcb_file):",
     "        for _ref in [str(k.get('name', ''))[len(_fpk.MATING_PREFIX):] "
     "for k in self.keepouts]:",
     (T1098,), 'KILLED'),
    ('place-pose-blind-to-declared-plug', 'pose',
     "                with_derived_keepouts(declared_keepouts, pcb, board_path),",
     "                with_derived_keepouts((), pcb, board_path),",
     (T1098,), 'KILLED'),
    ('place-pose-reads-unmeasured-as-clean', 'pose',
     "    if after.get('mating_keepout_error'):",
     "    if False:",
     (T1098,), 'KILLED'),
    ('decaps-from-one-way', 'fp',
     "        _match = (min(_common / len(_theirs), _common / len(_ours))",
     "        _match = (min(_common / len(_theirs), _common / len(_theirs))",
     (T1099,), 'KILLED'),
    ('promotion-said-only-beside-a-pin-limit', 'cfp',
     "            if cen.get('decap_ungraded_promoted'):",
     "            if False:",
     (T1100,), 'KILLED'),
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
