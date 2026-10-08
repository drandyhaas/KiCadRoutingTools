"""The #1105 mutation battery: seeder stage 3.5, stage 3's jitter draw, and the
emitter's forecast.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore a defect somebody measured:

  * `late-never-runs` -- run 38's StickHub pile: the per-pin stage claimed 0
    of 38 caps because no owner IC was seated before it;
  * `emitter-promises-2.5` -- the emitter promised "will seat 38 cap(s)" on
    that pile;
  * `forecast-drops-backup-owners` -- the phase-1 verifier: watchy's C12 is
    served at U5 once U3's edge seat fails, and the forecast had dropped U5.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1105.py
    python3 tests/mutate_1105.py --row late-never-runs

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`: the
test files run only the cases whose names contain one of the substrings.

Not covered by a row, and why:
  * the trigger's `i >= late_from` half: in the default queue order every IC
    precedes every 2-pin cap, so firing at the first scoped cap is the same
    point (an equivalent mutant); it differs only under anchors-first;
  * the decline's `state.apply_move(ref, *_was)` restore: a declined cap is
    still in `unplaced`, which every later seat excludes, and its own
    centroid turn moves it again -- no observable difference.
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
    'place_seed': os.path.join(_ROOT, 'py_placer', 'place_seed.py'),
    'cf': os.path.join(_ROOT, 'py_tools', 'check_floorplan.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T = 'test_1105_decap_claim_after_ics.py'
FLAT = _t(T, 'flat_board')
PILE = _t(T, 'pile_claims')
EVERY_IC = _t(T, 'every_ic')
TWICE = _t(T, 'served_twice')
NON_U = _t(T, 'non_u_owner')
OFF = _t(T, 'stage_off')
EMIT = _t(T, 'emitter_promises')
AGREE = _t(T, 'forecast_agrees')
FLAGS = _t(T, 'flag_pair')
DEFAULT = _t(T, 'default_is')
LIMIT = _t(T, 'past_the_limit')
GRADED = _t(T, 'measures_as_the_grade')
LIVE = _t(T, 'live_poses')
T1051 = _t('test_1051_seed_arrays.py', 'zero_claim_reports_why')
J = 'test_1105_stage3_jitter.py'
JIT_SAME = _t(J, 'same_with_the_claim')
JIT_OFF = _t(J, 'pre_1105_seeder')
JIT_ANCHORS = _t(J, 'anchors_first')

# (name, target, old, new, tests, expect)
ROWS = [
    ('late-never-runs', 'seeder',
     "        if (late_from is not None and late['at'] is None and i >= late_from",
     "        if (False and late_from is not None and late['at'] is None and i >= late_from",
     (FLAT, PILE, EVERY_IC), 'KILLED'),
    ('late-claimed-cap-reseated', 'seeder',
     "        if ref not in unplaced:",
     "        if False:",
     (FLAT,), 'KILLED'),
    ('late-reserves-early-owners', 'seeder',
     "                set(placed) - decap_owners_early, late_claimed,",
     "                set(placed), late_claimed,",
     (TWICE,), 'KILLED'),
    ('owner-rule-dropped', 'seeder',
     "            if not _decap_owner_ok(owner, chips):",
     "            if False:",
     (NON_U, T1051), 'KILLED'),
    ('late-count-zeroed', 'seeder',
     "        late['claimed'] = len(late_claimed)",
     "        late['claimed'] = 0",
     (FLAT,), 'KILLED'),
    ('late-tag-dropped', 'seeder',
     "                tag=' (stage 3.5)',",
     "                tag='',",
     (TWICE, FLAT), 'KILLED'),
    ('explicit-off-ignored', 'seeder',
     "               else bool(decap_claim_after_ics))",
     "               else True)",
     (OFF,), 'KILLED'),
    ('default-not-the-tables', 'seeder',
     "DECAP_CLAIM_AFTER_ICS_DEFAULT = False",
     "DECAP_CLAIM_AFTER_ICS_DEFAULT = True",
     (DEFAULT, EMIT), 'KILLED'),
    ('decline-limit-ignored', 'seeder',
     "                if _off > decline_beyond:",
     "                if False:",
     (LIMIT,), 'KILLED'),
    # the within-limit check's MEASURE: back to the distance from the pin
    # target, which declines seats the grade accepts (C2, 7.07 mm from its
    # pin, 0.82 mm from U1's pad box)
    ('decline-measures-the-pin-target', 'seeder',
     "                _el, _off = decap_graded_distance(",
     "                _el, _off = (lambda *_a: ('pin', math.hypot("
     "state.parts[ref].x - tx, state.parts[ref].y - ty)))(",
     (LIMIT,), 'KILLED'),
    ('decline-measures-at-the-file-pose', 'seeder',
     "        return footprint_at_pose(pcb_data.footprints[ref], (p.x, p.y, p.rot))",
     "        return pcb_data.footprints[ref]",
     (LIVE,), 'KILLED'),
    ('decline-sees-unplaced-chips', 'seeder',
     "        if c not in placed:",
     "        if False:",
     (GRADED,), 'KILLED'),
    ('decline-elects-its-own-chip', 'seeder',
     "    return _g.elect_live(_posed(cap), cands)",
     "    return (cands[0][0], _g.elect_live(_posed(cap), cands[:1])[1]) "
     "if cands else (None, None)",
     (GRADED,), 'KILLED'),
    ('forecast-blind-to-fixed-poses', 'seeder',
     "                 | {str(f['ref']) for f in intent.fixed_poses}",
     "                 | set()",
     (EMIT,), 'KILLED'),
    ('forecast-claims-ownerless', 'seeder',
     "            out['ownerless'].append(cap)",
     "            out['late'].append(cap)",
     (AGREE,), 'KILLED'),
    ('forecast-drops-backup-owners', 'seeder',
     "            backup_owners.update(o for o in owners if o not in early_set)",
     "            pass",
     (AGREE,), 'KILLED'),
    ('place-seed-flag-dropped', 'place_seed',
     "        diagonal_rotations=args.diagonal_rotations,\n"
     "        decap_claim_after_ics=args.decap_claim_after_ics)",
     "        diagonal_rotations=args.diagonal_rotations,\n"
     "        decap_claim_after_ics=None)",
     (FLAGS,), 'KILLED'),
    ('emitter-promises-2.5', 'cf',
     "    if late and not armed:",
     "    if False:",
     (EMIT,), 'KILLED'),
    ('emitter-forecast-not-taken', 'cf',
     "            _dcen['seeder_forecast'] = _seeder_forecast(doc, pcb, args.board,",
     "            _dcen['seeder_forecast_x'] = _seeder_forecast(doc, pcb, args.board,",
     (EMIT,), 'KILLED'),
    # stage 3's jitter (#1105 sub-issue): drawn per queue entry, after the
    # anchors-first reorder and before stage 3.5 can skip or reorder a turn
    ('jitter-drawn-at-the-turn', 'seeder',
     "        clr, target, jx, jy = _centroid_seat(ref, jit=q_jit[ref])",
     "        clr, target, jx, jy = _centroid_seat(ref)",
     (JIT_SAME, JIT_OFF), 'KILLED'),
    ('jitter-drawn-before-anchors-first', 'seeder',
     "    q_jit = {r: _jitter() for r in queue}",
     "    q_jit = {r: _jitter() for r in _order(sorted(unplaced))}",
     (JIT_ANCHORS,), 'KILLED'),
    ('jitter-drawn-after-the-reorder', 'seeder',
     "    q_jit = {r: _jitter() for r in queue}",
     "    q_jit = {r: _jitter() for r in (sorted(queue, key=lambda r: r in decap_scope) if late_on and DECAP_LATE_AT == 'after_queue' else queue)}",
     (JIT_SAME,), 'KILLED'),
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
