"""The #1111 mutation battery: check_pads measures a custom pad on its copper.

One row per load-bearing line, each reverting it; every row names the test
case that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore the defect #1111 measured:

  * `custom-pads-on-their-box` -- StickHub's JP1 (0.150 mm apart) read as a
    0.150 mm overlap and blocked check_complete's DONE; rp2350's U5 read 4;
  * `depth-is-the-box` -- a 0.08 mm sliver reported at the box's 0.700.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_1111.py
    python3 tests/mutate_1111.py --row custom-pads-on-their-box

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending. A witness is `(test file, case-name substring...)`: the
test files run only the cases whose names contain one of the substrings.

NO ROW IS EXPECTED TO SURVIVE. An earlier version asked check_drc's
pad-pad check whether the copper touched and declared that row a survivor;
#1111's verifier killed it with a thin crossing check_drc samples past, and
the probe is gone -- `test_a_thin_crossing_is_still_a_short` keeps it gone.

Not covered by a row, and why:
  * the seamed-ring repair `make_valid` does for an unfilled circle: no
    fixture here has a ring pad, and the repair is shapely's, measured in the
    PR (area 1.9977 against 2.0106 analytic, the same as `buffer(0)`).
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
    'pads': os.path.join(_ROOT, 'py_router', 'check_pads.py'),
    'geom': os.path.join(_ROOT, 'py_router', 'geometry_utils.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


T = 'test_1111_custom_pad_exact.py'
JUMPER = _t(T, 'interleaved_jumper')
EMPTY = _t(T, 'empty_side')
SLIVER = _t(T, 'sliver_overlap')
TOL = _t(T, 'tolerance')
CROSS = _t(T, 'cross_footprint')
U5 = _t(T, 'rp2350_u5')
UNTOUCHED = _t(T, 'no_custom_pad_is_untouched')
CLI = _t(T, 'cli_says_so')
BOWTIE = _t(T, 'self_crossing')
CIRCLE = _t(T, 'circle_pad_is_seen')
LOGICAL = _t(T, 'unconnected_pin_drawn_twice')
TIE = _t(T, 'net_tie_is_not')
FANDB = _t(T, 'f_and_b')
T1094 = _t('test_1094_rotated_courtyards.py')

# (name, target, old, new, tests, expect)
ROWS = [
    ('custom-pads-on-their-box', 'pads',
     "            if exact and depth > tolerance and (a.polygons or b.polygons):",
     "            if False:",
     (JUMPER, EMPTY, U5), 'KILLED'),
    ('every-pair-through-the-copper', 'pads',
     "            if exact and depth > tolerance and (a.polygons or b.polygons):",
     "            if exact and depth > tolerance:",
     (UNTOUCHED,), 'KILLED'),
    ('depth-is-the-box', 'pads',
     "        _copper_geometry(a).intersection(_copper_geometry(b)))",
     "        _copper_geometry(a).intersection(_copper_geometry(b))) * 0.0 + "
     "_overlap_depth(_pad_outline_polygon(a), _pad_outline_polygon(b))",
     (SLIVER, TOL), 'KILLED'),
    ('union-is-the-hull', 'pads',
     "                            if len(q) >= 3])",
     "                            if len(q) >= 3]).convex_hull",
     (SLIVER,), 'KILLED'),
    ('repair-drops-a-lobe', 'pads',
     "        return unary_union([make_valid(Polygon(q)) for q in pad.polygons",
     "        return unary_union([Polygon(q).buffer(0) for q in pad.polygons",
     (BOWTIE,), 'KILLED'),
    ('zero-edge-is-an-axis', 'pads',
     "            if L < 1e-12:\n                continue",
     "            if L < 1e-12:\n                L = 1.0",
     (CIRCLE,), 'KILLED'),
    ('unconnected-copies-are-a-short', 'pads',
     "                if (a.pad_number and a.pad_number == b.pad_number",
     "                if (False and a.pad_number == b.pad_number",
     (LOGICAL,), 'KILLED'),
    ('any-copies-are-one-pad', 'pads',
     "                        and _unconnected(a) and _unconnected(b)):",
     "                        ):",
     (LOGICAL,), 'KILLED'),
    ('net-tie-is-a-short', 'pads',
     "                if any(a.pad_number in g and b.pad_number in g",
     "                if False and any(a.pad_number in g and b.pad_number in g",
     (TIE,), 'KILLED'),
    ('f-and-b-is-its-own-layer', 'pads',
     '        if lyr == "F&B.Cu":',
     '        if False:',
     (FANDB,), 'KILLED'),
    ('cross-footprint-unmeasured', 'pads',
     "        return _overlaps_in(pads, tolerance, ties=ties)",
     "        return _overlaps_in(pads, tolerance, exact=False, ties=ties)",
     (CROSS,), 'KILLED'),
    ('per-footprint-unmeasured', 'pads',
     "        hits.extend(_overlaps_in(fp.pads, tolerance, ties=ties))",
     "        hits.extend(_overlaps_in(fp.pads, tolerance, exact=False, ties=ties))",
     (JUMPER, CLI), 'KILLED'),
    ('slivers-dropped-as-arealess', 'geom',
     "        return [geom] if geom.area > AREA_EPS_MM2 else []",
     "        return [geom] if geom.area > 1e9 else []",
     (SLIVER, T1094), 'KILLED'),
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
