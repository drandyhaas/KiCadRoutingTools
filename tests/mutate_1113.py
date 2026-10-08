"""The #1113 mutation battery: converge's veto disclosure and the seed-level
rotation ranker.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore a defect somebody measured:

  * `veto-reports-nothing` -- run 39: `converge.py poses --ref U1` dropped all
    323 candidates and said only how many;
  * `note-silent` -- the same run: nothing said that a one-part move cannot
    rank an IC's rotation once its decaps are packed against its pins;
  * `rotation-not-injected` -- the ranker's whole point: each arm must seed
    with the part's rotation DECLARED.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1113.py
    python3 tests/mutate_1113.py --row veto-reports-nothing

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`.

Not covered by a row: the `intent` and `tether` labels (no veto test arms an
intent or a tether gate) and the seeder's declared-rotation note wording.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
_PP = os.path.join(_ROOT, 'py_placer')
_PL = os.path.join(_PP, 'placement')

TARGETS = {
    'quench': os.path.join(_PL, 'quench.py'),
    'leg': os.path.join(_PL, 'legality.py'),
    'ps': os.path.join(_PP, 'pose_score.py'),
    'cv': os.path.join(_PP, 'converge.py'),
    'po': os.path.join(_PL, 'pose_ops.py'),
    'rr': os.path.join(_PP, 'rank_rotations.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


V = 'test_1113_pose_veto.py'
AGREE = _t(V, 'agrees_with_valid')
LABEL = _t(V, 'names_the_check')
PART = _t(V, 'partitions')
PUT = _t(V, 'staying_put')
EMPTY = _t(V, 'empty_ranking')
SNAP = _t(V, 'snap_census')
R = 'test_1113_rank_rotations.py'
KEY = _t(R, 'rotation_key')
CLASSIFY = _t(R, 'classify_row')
ELIG = _t(R, 'eligibility')
DERIVED = _t(R, 'derived_intent')
REFUSE = _t(R, 'refusals')

# (name, target, old, new, tests, expect)
ROWS = [
    ('ranker-pointer-on-any-veto', 'cv',
     "    turned = [d for d in by if d['check'] in _NEIGHBOUR_CHECKS",
     "    turned = [d for d in by if True",
     (_t(V, 'staying_put', 'neighbour_veto'),), 'KILLED'),
    ('veto-reports-nothing', 'quench',
     "        return (why.get('check', 'unattributed'), why.get('blocker'))",
     "        return None",
     (AGREE,), 'KILLED'),
    ('bbox-unlabelled', 'quench',
     "            self._veto('board_bbox')",
     "            pass",
     (AGREE, LABEL), 'KILLED'),
    ('courtyard-misnamed', 'quench',
     "                        self._veto('courtyard', other_ref, path='smd')",
     "                        self._veto('board_bbox', path='smd')",
     (LABEL,), 'KILLED'),
    ('pads-unlabelled', 'leg',
     "                    why.update(check='pads', blocker=nb, kind=kind)",
     "                    pass",
     (AGREE,), 'KILLED'),
    ('dropped-by-uncounted', 'ps',
     "                _d['count'] += 1",
     "                pass",
     (PART,), 'KILLED'),
    ('never-all-vetoed', 'ps',
     "        diagnostics['all_moves_vetoed'] = bool(dropped_total) and all(",
     "        diagnostics['all_moves_vetoed'] = False and all(",
     (PUT,), 'KILLED'),
    ('note-silent', 'cv',
     "    if diag.get('all_moves_vetoed') and diag.get('dropped_total'):",
     "    if False:",
     (PUT,), 'KILLED'),
    ('empty-note-unnamed', 'cv',
     "                 'no legal pose, including staying put -- vetoed by '",
     "                 'no legal pose, including staying put; '",
     (EMPTY,), 'KILLED'),
    ('census-drops-dropped-by', 'po',
     "              'dropped_by': diag.get('dropped_by', {}),",
     "              'dropped_by_x': 0,",
     (SNAP,), 'KILLED'),
    ('hard-fail-not-first', 'rr',
     "    return (agg['hard_fail'] is not None,",
     "    return (False,",
     (KEY,), 'KILLED'),
    ('unseated-not-keyed', 'rr',
     "            agg['unseated_max'],",
     "            0,",
     (KEY,), 'KILLED'),
    ('probe-not-outranking', 'rr',
     "            0 if agg['probed'] else 1,",
     "            0,",
     (KEY,), 'KILLED'),
    ('tie-leaves-input', 'rr',
     "            agg['ladder_index'])",
     "            -agg['ladder_index'])",
     (KEY,), 'KILLED'),
    ('default-admits-connectors', 'rr',
     "        elif classify_part(fp, ref).name is not None:",
     "        elif False:",
     (ELIG,), 'KILLED'),
    ('default-admits-declared', 'rr',
     "        elif ref in declared:",
     "        elif False:",
     (ELIG, REFUSE), 'KILLED'),
    ('rotation-applied-unchecked', 'rr',
     "    row['rotation_applied'] = (written_rot is not None",
     "    row['rotation_applied'] = (True or written_rot is not None",
     (CLASSIFY,), 'KILLED'),
    ('rotation-not-injected', 'rr',
     "    doc['blocks'] = blocks",
     "    doc['blocks'] = list(doc.get('blocks') or [])",
     (DERIVED,), 'KILLED'),
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
