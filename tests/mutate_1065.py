"""The #1065 mutation battery: render's pad-clearance list is the grader's.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore the defect #1065 measured:

  * `exact-verdict-ignored` -- an oval pad 0.277 mm from its neighbour read
    0.0215 mm short on the boxes at 0.25, and render flagged a pose the
    grader calls clean (run 33's agent gated its search on it);
  * `mm-from-the-box` -- the reported shortfall was the box's, not the copper's.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1065.py
    python3 tests/mutate_1065.py --row exact-verdict-ignored

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`: the
test files run only the cases whose names contain one of the substrings.

The legality rows mutate the GRADER, which render now calls -- so render and
`grade_pad_legality` move together, and only the corpus arm's PINNED numbers
(recorded at 457959b7) can see it. That is why those rows name `corpus`.

Not covered by a row, and why:
  * the `_copper_at_pose` cache: a performance device; dropping it changes
    no answer, only the time (check_drc's perimeter cache goes cold).
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

TARGETS = {
    'rp': os.path.join(_ROOT, 'py_tools', 'render_placement.py'),
    'leg': os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T = 'test_1065_render_pad_clearance_exact.py'
GRAZE = _t(T, 'oval_graze')
SHORT = _t(T, 'real_shortfall')
OVERRIDE = _t(T, 'pad_override')
SAME_NET = _t(T, 'same_net')
POSE = _t(T, 'models_pose')
CORPUS = _t(T, 'corpus_agrees')
GATE = _t(T, 'gate_reads')
CAPTION = _t(T, 'caption_counts')

# (name, target, old, new, tests, expect)
ROWS = [
    # -- render: the checklist calls the grader -------------------------------
    ('exact-verdict-ignored', 'rp',
     "                    if _hit:",
     "                    if True:",
     (GRAZE, GATE), 'KILLED'),
    ('mm-from-the-box', 'rp',
     "                            [a, bb, round(_mm, 4)])",
     "                            [a, bb, round(sf.pad, 4)])",
     (SHORT,), 'KILLED'),
    ('flat-scalar-requirement', 'rp',
     "                        ctx.clearance, ctx.pad_clearance_model, _exact,",
     "                        ctx.clearance, None, _exact,",
     (OVERRIDE,), 'KILLED'),
    ('file-pose-copper', 'rp',
     "                    footprint_at_pose(state.pcb_data.footprints[r],",
     "                    (lambda _f, _p: _f)(state.pcb_data.footprints[r],",
     (POSE,), 'KILLED'),
    ('caption-reads-the-box-metric', 'rp',
     "        _n = (len(legality_findings(spec.model)['pad_conflict_pairs_refs'])",
     "        _n = (m['pad_conflict_pairs'] if True else len(legality_findings(spec.model)['pad_conflict_pairs_refs'])",
     (CAPTION,), 'KILLED'),
    ('no-exact-check', 'rp',
     "            from check_drc import check_pad_pad_overlap as _exact",
     "            _exact = None",
     (GRAZE,), 'KILLED'),
    # -- legality: the census both now share ---------------------------------
    ('exact-confirmation-dropped', 'leg',
     "                    if not hit:\n                        continue\n"
     "                    pair_mm += over",
     "                    if False:\n                        continue\n"
     "                    pair_mm += over",
     (GRAZE, CORPUS), 'KILLED'),
    ('same-net-graded', 'leg',
     "            if na == nb and na > 0:\n                continue\n"
     "            if not _sides_interact(sa, sb):\n                continue\n"
     "            g = rect_gap((a0, a1, a2, a3), (b0, b1, b2, b3))\n"
     "            if g >= pair_reach - EPS:",
     "            if False:\n                continue\n"
     "            if not _sides_interact(sa, sb):\n                continue\n"
     "            g = rect_gap((a0, a1, a2, a3), (b0, b1, b2, b3))\n"
     "            if g >= pair_reach - EPS:",
     (SAME_NET,), 'KILLED'),
    ('grade-skips-the-exact-check', 'leg',
     "                parts[other], rects_b, pads_by_ref[other],\n"
     "                clearance, model, check_exact, routing_layers)",
     "                parts[other], rects_b, pads_by_ref[other],\n"
     "                clearance, model, None, routing_layers)",
     (CORPUS,), 'KILLED'),
    ('context-hides-its-model', 'leg',
     "        return self._floors",
     "        return None",
     (OVERRIDE,), 'KILLED'),
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
            print('%-34s %s' % (name, results[-1][1]), flush=True)
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
