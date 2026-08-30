#!/usr/bin/env python3
"""The #793/#799 mutation battery, shipped so its numbers can be re-derived.

Same contract as `tests/mutate_702.py`, which this copies: a row is KILLED by a
failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN, never
silently skipped; `str.replace(old, new, 1)`, never `sed`; originals restored in
a `finally`; and it REFUSES to start on a dirty engine, because restoring would
write the committed text back over uncommitted work.

THE MEASURED TABLES LIVE IN THE HEADERS OF `test_793_keepout_allow_unresolved.py`
and `test_799_zone_pose_feasibility.py`, FROM THE RUN -- never predicted here,
never edited afterwards to match.

One thing this adds over `mutate_702.py`: a `.pyc` DEFENCE. Several rows below
are size-preserving one-token edits, and two rows can run inside the same
second. CPython's source-timestamp check has one-second granularity, so a
cached `.pyc` written for the first mutant can be reused for the second and the
row reports a survivor that was never applied. Children run with `-B` AND every
`__pycache__` under `py_placer/` is removed between rows; `--selftest` proves
the defence rather than asserting it.

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. One writer per tree.

    python3 tests/mutate_799.py
    python3 tests/mutate_799.py --row only-one-rotation-is-tried
    python3 tests/mutate_799.py --list
    python3 tests/mutate_799.py --selftest
"""
from __future__ import annotations

import argparse
import io
import os
import shutil
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

FLOORPLAN = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
SCORE = os.path.join(_ROOT, '.claude', 'skills',
                     'plan-pcb-placement-and-routing', 'scripts',
                     'board_score.py')
TARGETS = {'fp': FLOORPLAN, 'bs': SCORE}

T793 = os.path.join(_TESTS, 'test_793_keepout_allow_unresolved.py')
T799 = os.path.join(_TESTS, 'test_799_zone_pose_feasibility.py')
T702 = os.path.join(_TESTS, 'test_702_quench_intent_gate.py')
T549S = os.path.join(_TESTS, 'test_549_floorplan_schema.py')
TBS = os.path.join(_TESTS, 'test_board_score_floorplan_severity.py')

ROWS = [
    # ---- #799: the four false-ERROR guards --------------------------------
    ('the-degenerate-keepout-still-forbids', 'fp',
     "    if not (k[2] - k[0] > 0.0 and k[3] - k[1] > 0.0):\n"
     "        return None\n",
     "    if False:\n"
     "        return None\n",
     (T799,), 'KILLED'),

    ('the-anchor-box-uses-the-seeders-origin-convention', 'fp',
     "    cx, cy = (b0x + b2x) / 2.0, (b0y + b2y) / 2.0\n",
     "    cx, cy = 0.0, 0.0\n",
     (T799,), 'KILLED'),

    ('the-circle-abstention-is-unscoped', 'fp',
     "    if discs and (not rects or _search(rects) is not None):\n",
     "    if discs:\n",
     (T799,), 'KILLED'),

    ('the-zone-side-eps-slack-is-dropped', 'fp',
     "                    if zone_escape(zone_rect, r, anchor)[0] > tolerance + legality.EPS:\n",
     "                    if zone_escape(zone_rect, r, anchor)[0] > tolerance:\n",
     (T799,), 'SURVIVED'),

    # ---- #799: the search itself ------------------------------------------
    ('only-one-rotation-is-tried', 'fp',
     "    rots = tuple((rot0 + d) % 360 for d in (0.0, 90.0, 180.0, 270.0))\n",
     "    rots = (rot0,)\n",
     (T799,), 'KILLED'),

    ('the-candidates-drop-the-hole-edges', 'fp',
     "    out = sorted(xs)\n"
     "    return out + [(a + b) / 2.0 for a, b in zip(out, out[1:])]\n",
     "    out = sorted(xs)\n"
     "    return [(a + b) / 2.0 for a, b in zip(out, out[1:])]\n",
     (T799,), 'KILLED'),

    ('the-through-hole-rect-is-forgotten', 'fp',
     "                for lb in ((b, t) if t is not None else (b,)):\n",
     "                for lb in (b,):\n",
     (T799,), 'KILLED'),

    ('the-tolerance-is-ignored', 'fp',
     "    anchor = zone_is_anchor(zone_rect, part, tolerance)\n",
     "    tolerance = 0.0\n"
     "    anchor = zone_is_anchor(zone_rect, part, tolerance)\n",
     (T799,), 'KILLED'),

    # ---- #799: what it reports --------------------------------------------
    ('the-check-never-refuses', 'fp',
     "            if v['feasible']:\n                continue\n",
     "            if True:\n                continue\n",
     (T799,), 'KILLED'),

    ('total-coverage-no-longer-runs-first', 'fp',
     "                expected={'overlap': 'partial or none'}))\n"
     "            continue\n",
     "                expected={'overlap': 'partial or none'}))\n"
     "            pass\n",
     (T799,), 'KILLED'),

    ('the-message-says-the-member-cannot-move', 'fp',
     "                    f\". No pose satisfies both, so {ref} can never LEAVE its \"\n",
     "                    f\". No pose satisfies both, so {ref} cannot move: it is \"\n",
     (T799,), 'KILLED'),

    ('the-binding-resolver-is-bypassed', 'fp',
     "            bound = keepouts_for_ref(intent.keepouts, ref, member_sides[ref])\n",
     "            bound = tuple(intent.keepouts)\n",
     (T799,), 'KILLED'),

    ('synthetic-parts-are-graded-too', 'fp',
     "            if lb is None or lb.synthetic:\n",
     "            if lb is None:\n",
     (T799,), 'SURVIVED'),

    # ---- #793 --------------------------------------------------------------
    ('allow-unresolved-fires-at-error', 'fp',
     "                                        default=WARN),\n",
     "                                        default=ERROR),\n",
     (T793,), 'KILLED'),

    ('allow-unresolved-checks-the-tuple-not-each-pattern', 'fp',
     "        dead = [p for p in (k.get('allow') or ())\n"
     "                if not any(allow_pattern_matches(p, r) for r in refs)]\n",
     "        dead = ([] if any(allow_pattern_matches(p, r)\n"
     "                          for p in (k.get('allow') or ()) for r in refs)\n"
     "                else list(k.get('allow') or ()))\n",
     (T793,), 'KILLED'),

    ('the-audit-uses-its-own-matcher', 'fp',
     "        dead = [p for p in (k.get('allow') or ())\n"
     "                if not any(allow_pattern_matches(p, r) for r in refs)]\n",
     "        dead = [p for p in (k.get('allow') or ())\n"
     "                if not any(fnmatch.fnmatch(r, p) for r in refs)]\n",
     (T793,), 'KILLED'),

    ('the-exemption-uses-its-own-matcher', 'fp',
     "        if any(allow_pattern_matches(pat, ref) for pat in (k.get('allow') or ())):\n",
     "        if any(fnmatch.fnmatch(ref, pat) for pat in (k.get('allow') or ())):\n",
     (T793,), 'KILLED'),

    ('grade-does-not-raise-either-finding', 'fp',
     "    violations = (list(validate_intent(intent)) + list(block_problems)\n"
     "                  + list(unresolved_keepout_allows(intent, pcb_data))\n"
     "                  + list(intent_zone_keepout_problems(\n"
     "                      intent, blocks, pcb_data, pcb_file)))\n",
     "    violations = list(validate_intent(intent)) + list(block_problems)\n",
     (T793, T799), 'KILLED'),

    ('the-rule-name-is-not-registered', 'fp',
     "    'intent_zone_in_keepout', 'keepout_allow_unresolved'})\n",
     "    'intent_zone_in_keepout'})\n",
     (T549S,), 'KILLED'),

    # ---- the board_score prerequisite --------------------------------------
    ('blocking-counts-every-violation-again', 'bs',
     "    return {'ran': True, 'count': len(errors),\n",
     "    return {'ran': True, 'count': len(viols),\n",
     (TBS,), 'KILLED'),
]


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=_ROOT)
    return r.returncode == 0


def _drop_pyc():
    """Remove every cached bytecode file under the trees we mutate.

    CPython validates a `.pyc` on the source's mtime with ONE-SECOND
    granularity and its size. Two size-preserving rows applied inside the same
    second can therefore leave the second import reading the FIRST mutant --
    which reports as a survivor for a row that was never really applied.
    """
    for base in (os.path.join(_ROOT, 'py_placer'),
                 os.path.join(_ROOT, 'py_router')):
        for dirpath, dirnames, _files in os.walk(base):
            if os.path.basename(dirpath) == '__pycache__':
                shutil.rmtree(dirpath, ignore_errors=True)
                dirnames[:] = []


def _run(tests):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=_ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', env=env)
        if r.returncode != 0:
            return True, f"{os.path.basename(t)} exit {r.returncode}"
    return False, "all named tests passed"


def _selftest():
    """Prove the .pyc defence instead of asserting it.

    Applies a size-preserving mutation twice within one second and requires the
    SECOND probe to observe the second mutant. Without `-B` plus the cache
    sweep, the second import can be served from the first mutant's bytecode.
    """
    if not _git_clean([FLOORPLAN]):
        print("REFUSED: floorplan.py is dirty", file=sys.stderr)
        return 2
    src = io.open(FLOORPLAN, encoding='utf-8').read()
    anchor = "    rots = tuple((rot0 + d) % 360 for d in (0.0, 90.0, 180.0, 270.0))\n"
    if src.count(anchor) != 1:
        print("BROKEN selftest: anchor matched "
              f"{src.count(anchor)} times", file=sys.stderr)
        return 2
    probe = [sys.executable, '-B', '-X', 'utf8', '-c',
             "import sys; sys.path.insert(0,'py_placer');"
             "from placement import floorplan as f;"
             "p=f._LocalPart(0.0,(-1,-1,1,1));"
             "print(len(f.zone_pose_feasibility((0,0,9,9),0.0,p,"
             "[{'name':'k','rect':(0,0,1,1),'allow':(),'sides':('F','B')}])"
             "['rotations']))"]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    seen = []
    try:
        for repl in ("    rots = tuple((rot0 + d) % 360 for d in (0.0, 90.0, 180.0, 271.0))\n",
                     "    rots = tuple((rot0 + d) % 360 for d in (0.0, 90.0, 271.0, 272.0))\n"):
            _drop_pyc()
            io.open(FLOORPLAN, 'w', encoding='utf-8', newline='').write(
                src.replace(anchor, repl, 1))
            r = subprocess.run(probe, cwd=_ROOT, capture_output=True,
                               text=True, encoding='utf-8', env=env)
            seen.append(r.stdout.strip())
    finally:
        _drop_pyc()
        io.open(FLOORPLAN, 'w', encoding='utf-8', newline='').write(src)
    ok = seen == ['4', '4'] and all(s for s in seen)
    print(f"  selftest: two same-second size-preserving mutations, probe read "
          f"{seen} -- {'OK' if ok else 'the second may be a STALE .pyc'}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print(f"  {exp:9} {name}  [{tgt}] "
                  f"-> {', '.join(os.path.basename(t) for t in tests)}")
        return 0
    if a.selftest:
        return _selftest()

    rows = ROWS
    if a.row:
        unknown = [n for n in a.row if n not in {r[0] for r in ROWS}]
        if unknown:
            print(f"no such row: {', '.join(unknown)}; try --list",
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in set(a.row)]

    if not _git_clean(TARGETS.values()):
        print("REFUSED: the target files are dirty. Restoring would write the "
              "COMMITTED text back over uncommitted work.", file=sys.stderr)
        return 2

    originals = {k: io.open(p, encoding='utf-8').read()
                 for k, p in TARGETS.items()}
    verdicts = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            src = originals[tgt]
            n = src.count(old)
            if n != 1:
                verdicts.append((name, 'BROKEN', f"anchor matched {n} times"))
                print(f"  BROKEN   {name} -- anchor matched {n} times")
                continue
            _drop_pyc()
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(
                src.replace(old, new, 1))
            killed, why = _run(tests)
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(src)
            _drop_pyc()
            got = 'KILLED' if killed else 'SURVIVED'
            mark = 'ok' if got == expect else 'WRONG'
            verdicts.append((name, got, why))
            print(f"  {got:9}{'' if mark == 'ok' else ' WRONG'} "
                  f"{name} -- {why}")
    finally:
        for k, p in TARGETS.items():
            io.open(p, 'w', encoding='utf-8', newline='').write(originals[k])
        _drop_pyc()

    killed = sum(1 for _n, g, _w in verdicts if g == 'KILLED')
    survived = sum(1 for _n, g, _w in verdicts if g == 'SURVIVED')
    broken = [n for n, g, _w in verdicts if g == 'BROKEN']
    wrong = [r[0] for r, (_n, g, _w) in zip(rows, verdicts)
             if g != r[5] and g != 'BROKEN']
    print(f"\n{len(verdicts)} row(s): {killed} killed, {survived} survived, "
          f"{len(broken)} broken, {len(wrong)} disagreeing with expectation")
    if wrong:
        print("  WRONG: " + ', '.join(wrong))
    if broken:
        print("  BROKEN: " + ', '.join(broken))
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
