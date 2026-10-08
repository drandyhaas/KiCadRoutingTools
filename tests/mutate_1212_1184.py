#!/usr/bin/env python3
"""The #1212 / #1184 mutation battery (fa10 P1, phase 3).

A container is decided by GEOMETRY (`legality._container_kind`: a pin frame
or a pad-less outline), its rect pairs leave the courtyard channel, and a pin
frame is graded on its drilled HOLES (`CourtyardCensus._pin_pairs`) --
absolutely, in check_assembly, board_score, the seat search, the seeder's
declared-pose check and eviction count, and the repair census. Each row
breaks one of those next to the test that must fail.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first.

    python3 -X utf8 tests/mutate_1212_1184.py
    python3 -X utf8 tests/mutate_1212_1184.py --row frame-rect-pairs-kept
    python3 -X utf8 tests/mutate_1212_1184.py --list

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
           'seeder': os.path.join(_PL, 'seeder.py'),
           'asm': os.path.join(_ROOT, 'py_tools', 'check_assembly.py'),
           'score': os.path.join(_ROOT, 'py_tools', 'board_score.py'),
           'cv': os.path.join(_ROOT, 'py_placer', 'converge.py')}

T1212 = os.path.join(_TESTS, 'test_1212_container_pins.py')
T1184 = os.path.join(_TESTS, 'test_1184_container_classification.py')
T918 = os.path.join(_TESTS, 'test_918_assembly_verdict.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # -- the classifier (#1184) --
    ('a-drawn-courtyard-can-be-a-frame', 'leg',
     "    if source == SOURCE_COURTYARD or synthetic:",
     "    if synthetic:",
     (T1184,), 'KILLED'),
    ('the-drilled-fraction-is-not-asked', 'leg',
     "    if drilled < FRAME_DRILLED_FRAC * len(pads):",
     "    if drilled < 0:",
     (T1184,), 'KILLED'),
    ('an-interior-pin-is-not-asked', 'leg',
     "    if any(ix0 < p.local_x < ix1 and iy0 < p.local_y < iy1 for p in pads):",
     "    if False:",
     (T1184,), 'KILLED'),
    ('a-pad-less-part-is-a-body', 'leg',
     "        return 'outline'",
     "        return None",
     (T1184,), 'KILLED'),
    ('area-alone-makes-a-container', 'leg',
     "            if (getattr(p, 'drill', 0) or 0) > 0 or _pad_carries_copper(p)]",
     "            if False]",
     (T1184,), 'KILLED'),
    ('a-locked-outline-gates', 'leg',
     "        if p.waiver == 'container_class':",
     "        if p.waiver == 'container_class' and not self.locked_refs & {p.a, p.b}:",
     (T1184,), 'KILLED'),
    # -- the grader's pin channel (#1212) --
    ('frame-rect-pairs-kept', 'leg',
     "               if p.a not in self.pin_frames and p.b not in self.pin_frames]",
     "               if True]",
     (T1212,), 'KILLED'),
    ('the-pin-is-its-copper-ring', 'leg',
     "            p1, p2, r = pad_drill_capsule(pad)",
     "            p1, p2, r = pad_drill_capsule(pad); r = max(pad.size_x, pad.size_y) / 2.0",
     (T1212,), 'KILLED'),
    ('pin-pairs-never-gate', 'leg',
     "        pin_blocking = [p for p in pin_pairs if not p.waived",
     "        pin_blocking = [p for p in pin_pairs if False",
     (T1212,), 'KILLED'),
    ('pin-pairs-leave-the-pair-list', 'leg',
     "    pairs: List[BodyOverlapPair] = list(_cg.pairs) + list(_cg.pin_pairs)",
     "    pairs: List[BodyOverlapPair] = list(_cg.pairs)",
     (T1212,), 'KILLED'),
    ('the-search-broad-phase-says-far', 'leg',
     "            if not near:",
     "            if True:",
     (T1212,), 'KILLED'),
    # -- the verdicts --
    ('check-assembly-drops-the-conjunct', 'asm',
     "                         or pin_hits)",
     "                         or False)",
     (T1212,), 'KILLED'),
    ('board-score-drops-the-live-conjunct', 'score',
     "                           'pin_in_courtyard', 'courtyard_blocking_gating')",
     "                           'courtyard_blocking_gating')",
     (T918,), 'KILLED'),
    # -- the search, the seeder and the repair --
    ('the-search-skips-the-pins', 'quench',
     "        if legal and self.pin_frame_refs:",
     "        if False:",
     (T1212,), 'KILLED'),
    ('the-optimizer-skips-the-pins', 'quench',
     "        if self._pin_conflict_at(ref, _px, _py, _pr, exclude) is not None:",
     "        if False:",
     (T1212,), 'KILLED'),
    ('the-search-calls-a-big-body-a-frame', 'quench',
     "        self.container_refs = set(self.container_kinds)",
     "        self.container_refs = set(self.container_kinds) | ({'BAT1'} & set(self.parts))",
     (T1184,), 'KILLED'),
    ('a-declared-pose-ignores-the-pins', 'seeder',
     "            conflicts[_other] = (",
     "            conflicts[_other + '~'] = (",
     (T1212,), 'KILLED'),
    ('a-declared-pose-reads-current-poses', 'seeder',
     "                ref, *pose, poses={o: tuple(p) for o, p in obstacles.items()",
     "                ref, *pose, poses=None or {o: (state.parts[o].x, state.parts[o].y, state.parts[o].rot) for o, p in obstacles.items()",
     (T1212,), 'KILLED'),
    ('a-declared-waiver-is-ignored', 'seeder',
     "            if frozenset((ref, _other)) in waived:",
     "            if False:",
     (T1212,), 'KILLED'),
    ('the-eviction-count-skips-the-pins', 'seeder',
     "            if state._pin_conflict_at(a, pa.x, pa.y, pa.rot,",
     "            if False and state._pin_conflict_at(a, pa.x, pa.y, pa.rot,",
     (T1212,), 'KILLED'),
    ('the-repair-skips-the-pins', 'seeder',
     "    _pins = [q for q in body.get('pin_in_courtyard_pairs', ())",
     "    _pins = [q for q in ()",
     (T1212,), 'KILLED'),
    # -- the phase-3 verifier's findings --
    ('the-pin-takes-courtyards-severity', 'leg',
     "            sev = self.pin_severity.get(p.hole)",
     "            sev = self.severity",
     (T1212,), 'KILLED'),
    ('the-pin-ignores-the-far-courtyard', 'leg',
     "                     getattr(geom, 'far_court_local', None))):",
     "                     None)):",
     (T1212,), 'KILLED'),
    ('a-pin-under-occupancy-gates', 'leg',
     "                        and p.basis == 'courtyard']",
     "                        ]",
     (T1212,), 'KILLED'),
    ('a-declared-pin-pair-gates', 'leg',
     "            sev = self.pin_severity.get(p.hole)\n            if waivers.waiver_sets",
     "            sev = self.pin_severity.get(p.hole)\n            if False and waivers.waiver_sets",
     (T1212,), 'KILLED'),
    ('a-logo-graded-on-pins', 'leg',
     "                if r == frame or r in self.containers or gp.synthetic:",
     "                if r == frame or r in self.containers:",
     (T1212,), 'KILLED'),
    ('a-moved-frame-sees-only-itself', 'leg',
     "            refs = list(self.lbs)",
     "            refs = [ref]",
     (T1212,), 'KILLED'),
    ('the-json-count-reads-zero', 'leg',
     "            'pin_in_courtyard': len(_cg.pin_blocking),",
     "            'pin_in_courtyard': 0,",
     (T1212,), 'KILLED'),
    ('the-search-skips-pins-when-courtyards-are-ignored', 'quench',
     "        if not self.pin_frame_refs:\n            return []\n        if self._pin_census_obj is None:",
     "        if not self.pin_frame_refs or self.courtyards_ignored:\n            return []\n        if self._pin_census_obj is None:",
     (T1212,), 'KILLED'),
    ('the-search-frame-reads-file-poses', 'quench',
     "            refs = (self.parts if ref in self.pin_frame_refs",
     "            refs = (() if ref in self.pin_frame_refs",
     (T1212,), 'KILLED'),
    ('the-search-overlap-keeps-container-rects', 'quench',
     "            [g for g in parts if g.ref not in _cont])",
     "            [g for g in parts])",
     (T1212,), 'KILLED'),
    ('the-repair-moves-a-locked-part', 'seeder',
     "        if other not in state.parts or state.parts[other].locked:",
     "        if other not in state.parts:",
     (T1212,), 'KILLED'),
    ('the-repair-ignores-a-declared-pair', 'seeder',
     "             if frozenset((q.a, q.b)) not in _declared]",
     "             ]",
     (T1212,), 'KILLED'),
    ('an-absent-frame-refuses-a-declared-pose', 'seeder',
     "            if _other not in obstacles:",
     "            if False:",
     (T1212,), 'KILLED'),
    ('converge-forgets-the-pin-veto', 'cv',
     "_NEIGHBOUR_CHECKS = ('courtyard', 'container_pin', 'pads', 'waived_pads',",
     "_NEIGHBOUR_CHECKS = ('courtyard', 'pads', 'waived_pads',",
     (T1212,), 'KILLED'),
    # ---- the second phase-3 verifier's cases ------------------------------
    ('the-hole-is-tested-exact', 'leg',
     "            r = r - self.PIN_HOLE_TOLERANCE_MM",
     "            r = r",
     (T1212,), 'KILLED'),
    ('a-malformed-courtyard-gates', 'leg',
     "                if shape is not None and how == OUTLINE_POLYGON:",
     "                if shape is not None:",
     (T1212,), 'KILLED'),
    ('the-far-courtyard-as-its-bbox', 'leg',
     "                    (getattr(geom, 'far_court_shape_local', None),",
     "                    (None,",
     (T1212,), 'KILLED'),
    ('the-own-courtyard-as-its-bbox', 'leg',
     "                    (geom.court_shape_local, geom.court_shape_how,",
     "                    (None, geom.court_shape_how,",
     (T1212,), 'KILLED'),
    ('the-pin-rule-read-off-the-file-only', 'leg',
     "                    pcb_file or getattr(pcb_data, 'source_path', None),",
     "                    pcb_file,",
     (T1212,), 'KILLED'),
    ('the-broad-phase-skips-the-far-courtyard', 'leg',
     "            if _far is not None:",
     "            if False:",
     (T1212,), 'KILLED'),
    ('the-hole-kind-lost', 'leg',
     "                          'np_thru_hole' else 'pth'))",
     "                          'np_thru_hole_X' else 'pth'))",
     (T1212,), 'KILLED'),
    ('the-npth-rule-reads-pth', 'leg',
     "                                 ('npth', 'npth_inside_courtyard')):",
     "                                 ('npth', 'pth_inside_courtyard')):",
     (T1212,), 'KILLED'),
    ('one-pair-for-both-hole-kinds', 'leg',
     "                            f = found.setdefault((hole, basis), [[], 0.0])",
     "                            f = found.setdefault(('pth', basis), [[], 0.0])",
     (T1212,), 'KILLED'),
    ('a-warning-waives-a-pin', 'leg',
     "            elif sev == 'ignore':",
     "            elif sev in ('ignore', 'warning'):",
     (T1212,), 'KILLED'),
    ('the-saved-value-is-the-courtyards', 'leg',
     "    saved = saved_all.get(rule)",
     "    saved = saved_all.get('courtyards_overlap')",
     (T1212,), 'KILLED'),
    ('the-pin-rule-read-when-told-not-to', 'leg',
     "        if courtyard_severity == 'auto':\n            for _hole, _rule in",
     "        if True:\n            for _hole, _rule in",
     (T1212,), 'KILLED'),
    ('the-waived-path-skips-the-pins', 'quench',
     "            return board, overlap + self._pin_violation(ref, x, y, rot,",
     "            return board, overlap + 0.0 * self._pin_violation(ref, x, y, rot,",
     (T1212,), 'KILLED'),
    ('the-npth-label-lost', 'asm',
     "                  f\"{', '.join(x or f'(unnumbered {_kind})' for x in q.pins)}\"",
     "                  f\"{', '.join(q.pins)}\"",
     (T1212,), 'KILLED'),
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
