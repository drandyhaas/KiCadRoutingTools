"""The #1125/#1126/#1128/#1129 mutation battery: the placement follow-ups of
PR #1130 fixed beside #1105.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore a defect somebody measured:

  * `overrun-reads-paste-as-copper` -- a 1 mm F.Paste pad 3 mm off the board
    gated its part at 3.21 mm (#1128);
  * `caption-prints-the-optimizer` -- glasgow_revC's caption read 70.05 mm2
    against the census's 52.252 (#1126);
  * `free-refs-ignores-intent-locks` -- place_portfolio turned a must_lock U1
    270 -> 90 and the quench froze it there (#1129);
  * `walk-never-runs` -- splitflap's J5 declared [180, 90] kept 180, which
    only crowds J17, where 90 seats clear (#1125).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1125_1126_1128_1129.py
    python3 tests/mutate_1125_1126_1128_1129.py --row caption-prints-the-optimizer

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`;
test_983's witnesses select a case with unittest's `-k`.

Not covered by a row, and why:
  * #1125's walk on an EARLY skip (wider than the edge, outside the window):
    `_stage1_geometry_rot` picks a member those refusals pass whenever one
    exists, so an early skip means no member fits and the walk finds none --
    an equivalent mutant;
  * #1125's `edge_floor_fallback.pop` and `placed.discard` in the undo: no
    fixture's crowded seat carries a floor record, and a ref left in
    `placed` is skipped by `_shorted_by` (`other == ref`) and re-added by the
    next seat -- no observable difference on these boards.
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
    'seeder': os.path.join(_PL, 'seeder.py'),
    'legality': os.path.join(_PL, 'legality.py'),
    'portfolio': os.path.join(_PL, 'portfolio.py'),
    'render': os.path.join(_ROOT, 'py_tools', 'render_placement.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T1128 = 'test_1128_paste_only_not_copper.py'
OVERRUN = _t(T1128, 'overrun_is_the_copper')
OCCUPANCY = _t(T1128, 'outside_the_courtyard')
CONTROLS = _t(T1128, 'still_count')
T1126 = 'test_1126_render_caption_census.py'
CAPTION = _t(T1126, 'caption_is_the_census')
NO_CENSUS = _t(T1126, 'no_census')
PAIR = _t(T1126, 'pair_diff')
T1129 = 'test_1129_portfolio_intent_locks.py'
UNIT = _t(T1129, 'drops_the_gate_locks')
E2E = _t(T1129, 'keeps_its_input_pose')
T983 = 'test_983_seat_grade_bounds.py'
C5 = _t(T983, '-k', 'test_c5_')
C14 = _t(T983, '-k', 'test_c14_')
C15 = _t(T983, '-k', 'test_c15_')
C16 = _t(T983, '-k', 'test_c16_')
C17 = _t(T983, '-k', 'test_c17_')
C18 = _t(T983, '-k', 'test_c18_')
C19 = _t(T983, '-k', 'test_c19_')

# (name, target, old, new, tests, expect)
ROWS = [
    # #1128
    ('occupancy-reads-paste-as-copper', 'legality',
     "        if not _pad_carries_copper(pad):",
     "        if getattr(pad, 'pad_type', '') == 'np_thru_hole':",
     (OCCUPANCY,), 'KILLED'),
    ('overrun-reads-paste-as-copper', 'legality',
     "    over = 0.0\n    for p in pads or ():\n"
     "        if not _pad_carries_copper(p):",
     "    over = 0.0\n    for p in pads or ():\n"
     "        if getattr(p, 'pad_type', '') == 'np_thru_hole':",
     (OVERRUN,), 'KILLED'),
    ('overrun-forgets-npth', 'legality',
     "    over = 0.0\n    for p in pads or ():\n"
     "        if not _pad_carries_copper(p):",
     "    over = 0.0\n    for p in pads or ():\n"
     "        if getattr(p, 'pad_type', '') == 'smd' and not p.layers:",
     (CONTROLS,), 'KILLED'),
    # #1126
    ('caption-prints-the-optimizer', 'render',
     "                         f\"{_f['courtyard_overlap_mm2']:.2f}mm2\")",
     "                         f\"{m['overlap_area']:.2f}mm2\")",
     (CAPTION,), 'KILLED'),
    ('caption-census-error-reads-zero', 'render',
     "                    if _f is None or _f.get('courtyard_census_error')",
     "                    if _f is None",
     (NO_CENSUS,), 'KILLED'),
    ('caption-no-pcb-reads-zero', 'render',
     "              if getattr(spec.model, 'pcb', None) is not None else None)",
     "              if True else None)",
     (NO_CENSUS,), 'KILLED'),
    ('pair-diff-prints-the-optimizer', 'render',
     "             _cen['before'], _cen['after']),",
     "             before_model.metrics.get('overlap_area'), "
     "after_model.metrics.get('overlap_area')),",
     (PAIR,), 'KILLED'),
    # #1129
    ('free-refs-ignores-intent-locks', 'portfolio',
     "        if ref in held:",
     "        if False:",
     (UNIT,), 'KILLED'),
    ('intent-lock-unrecorded', 'portfolio',
     "                refused[ref] = (\"locked by the intent (must_lock or an edge \"",
     "                _unrecorded = (\"locked by the intent (must_lock or an edge \"",
     (UNIT,), 'KILLED'),
    ('generate-passes-no-locks', 'portfolio',
     "                     intent_locks=(qkw.get('intent_gate') or {}).get(",
     "                     intent_locks=({}).get(",
     (E2E,), 'KILLED'),
    # #1125
    ('walk-never-runs', 'seeder',
     "            if (_out not in ('crowded', 'skip_late') or _claim is None",
     "            if (True or _claim is None",
     (C14, C5), 'KILLED'),
    ('walk-keeps-the-last-crowded-member', 'seeder',
     "            if len(_tried) > 1:",
     "            if False:",
     (C15,), 'KILLED'),
    ('notes-not-rolled-back', 'seeder',
     "            del notes[n0:]",
     "            pass",
     (C14,), 'KILLED'),
    ('pose-not-restored', 'seeder',
     "            state.apply_move(ref, *pose)",
     "            pass",
     (C15,), 'KILLED'),
    ('walked-member-not-applied', 'seeder',
     "                _geo_rot = _member1125",
     "                pass",
     (C14,), 'KILLED'),
    ('skip-late-not-walked', 'seeder',
     "            if (_out not in ('crowded', 'skip_late') or _claim is None",
     "            if (_out not in ('crowded',) or _claim is None",
     (C17,), 'KILLED'),
    ('crowded-includes-kept', 'seeder',
     "            return ('crowded' if (_pick is None and _kept is None",
     "            return ('crowded' if (_pick is None or _kept is None",
     (C19,), 'KILLED'),
    ('crowded-member-not-kept', 'seeder',
     "                if _o == 'crowded' and _crowded is None:",
     "                if False:",
     (C18,), 'KILLED'),
    ('walk-order-reversed', 'seeder',
     "    nxt = _stage1_geometry_rot(part, claim, fits=_left)",
     "    nxt = _stage1_geometry_rot(part, (claim[0], tuple(reversed("
     "claim[1]))), fits=_left)",
     (C16,), 'KILLED'),
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
