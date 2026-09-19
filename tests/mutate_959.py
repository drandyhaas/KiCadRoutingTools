#!/usr/bin/env python3
"""The #959 mutation battery: does anything notice when the declare stage
stops asking, stops checking, or starts believing the wrong channel?

One row per load-bearing mechanism of the six sub-issues:

  #997  the rule roster: a dark, applicable, gating rule is OWED at P1; a
        policy rule never is; P1 asks it LAST.
  #998  `plan_check`: an area bound is an ERROR only past a declared budget
        and while every rule it stands for is one; place_seed refuses before
        any write (exit 5); compare_seeds stops at the first refusal;
        `place_pose --intent` refuses a pose that leaves a zone WORSE.
  #999  pad-less blocks are owed at P1; `rank_poses` refuses what it cannot
        rank rather than raising a bare KeyError.
  #1000 connector declarations compile, and only as far as the evidence
        goes: no derived overhang floor, no face from `user_facing`, a
        vertical mount exempt from the seat, a keep-out judged by its name.
  #1001 mechanical.json: a turn is an ERROR, a brief the run wrote is a
        hypothesis, and P1 asks about the reconciled channels.
  #1002 decaps: auto withholds on a pile into the census (never into
        `budget_withheld`), every emitted number is labelled, and a declared
        relation supersedes the inferred tether.

Every row carries an EXPECTATION, and a verdict that does not match it is
reported as WRONG. An anchor that does not match its target EXACTLY ONCE is
BROKEN, checked before anything is written (`preflight`, #877).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- run it in a checkout nothing else
is reading. It refuses to start while a target has uncommitted changes,
because it restores by overwriting.

    python3 -X utf8 tests/mutate_959.py
    python3 -X utf8 tests/mutate_959.py --row plan-severity-ignores-demotion
    python3 -X utf8 tests/mutate_959.py --list

A row is KILLED by any non-zero exit of a listed test, a raised error
included -- which is why every killer runs on the UNMUTATED tree first, and
the battery exits 2 if one fails there. `_uncache` is carried over from
`tests/mutate_975.py`: several rows are same-size edits, and CPython trusts a
`.pyc` on (mtime seconds, size).

EXPECTED SURVIVORS: none declared.

THE MEASURED RESULT is recorded below from the run, never predicted.
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
    'fp': os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py'),
    'db': os.path.join(_ROOT, 'py_placer', 'placement', 'design_brief.py'),
    'rc': os.path.join(_ROOT, 'py_placer', 'placement', 'reconcile.py'),
    'po': os.path.join(_ROOT, 'py_placer', 'placement', 'pose_ops.py'),
    'ps': os.path.join(_ROOT, 'py_placer', 'place_seed.py'),
    'cs': os.path.join(_ROOT, 'py_placer', 'compare_seeds.py'),
    'sc': os.path.join(_ROOT, 'py_placer', 'pose_score.py'),
    'dr': os.path.join(_ROOT, '.claude', 'skills', 'plan-pcb-placement',
                       'scripts', 'placement_driver.py'),
}

T_ROSTER = os.path.join(_TESTS, 'test_959_rule_roster.py')
T_PADLESS = os.path.join(_TESTS, 'test_959_padless_blocks.py')
T_RECON = os.path.join(_TESTS, 'test_959_reconcile.py')
T_PLAN = os.path.join(_TESTS, 'test_959_plan_check.py')
T_POSE = os.path.join(_TESTS, 'test_959_pose_intent.py')
T_CONN = os.path.join(_TESTS, 'test_959_connector_clauses.py')
T_DECAPS = os.path.join(_TESTS, 'test_959_decaps_auto.py')
T_COMMENT = os.path.join(_TESTS, 'test_959_comment_controls.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- #997: the roster -------------------------------------------------
    ('dark-rules-never-owed', 'fp',
     "                needs = (not brief and not disposition",
     "                needs = (False and not brief and not disposition",
     (T_ROSTER,), 'KILLED'),
    ('policy-rules-owed-too', 'fp',
     "        if not policy and applicable and gating:",
     "        if applicable and gating:",
     (T_ROSTER,), 'KILLED'),
    ('p1-never-asks-the-roster', 'dr',
     "    ok_, why_ = _roster_owed(a, intent, pcb, _fp)\n",
     "    ok_, why_ = True, None\n",
     (T_ROSTER,), 'KILLED'),

    # ---- #998: the plan, before any pose -----------------------------------
    ('plan-severity-ignores-demotion', 'fp',
     "    return (ERROR if all(intent.severity_of(r) == ERROR for r in needs)\n"
     "            else WARN)",
     "    return ERROR",
     (T_PLAN,), 'KILLED'),
    ('zone-overfull-without-a-budget', 'fp',
     "        if (overlap_budget is not None\n"
     "                and excess > float(overlap_budget) + legality.EPS):",
     "        if (excess > legality.EPS):",
     (T_PLAN,), 'KILLED'),
    ('board-overfull-ignores-oob', 'fp',
     "    on_board = (intent.legality_budget or {}).get('oob_count') == 0",
     "    on_board = True",
     (T_PLAN,), 'KILLED'),
    ('place-seed-writes-a-refused-plan', 'ps',
     "    if _plan_err and not (args.repair or args.reseat is not None):",
     "    if False:",
     (T_PLAN,), 'KILLED'),
    ('compare-seeds-retries-a-refused-plan', 'cs',
     "        if r.returncode == 5:",
     "        if False:",
     (T_PLAN,), 'KILLED'),
    ('p1-never-checks-the-plan', 'dr',
     "    ok_, why_ = _plan_owed(a, intent, pcb, _fp)\n",
     "    ok_, why_ = True, None\n",
     (T_PLAN,), 'KILLED'),
    ('zone-check-absolute-not-relative', 'po',
     "                and row['outside_mm_after'] > row['outside_mm_before'] + 1e-9):",
     "                and row['outside_mm_after'] > 1e-9):",
     (T_POSE,), 'KILLED'),

    # ---- #999: pad-less blocks ----------------------------------------------
    ('p1-never-asks-about-padless-blocks', 'dr',
     "    ok_, why_ = _padless_owed(a, intent, pcb, padless, covered)\n",
     "    ok_, why_ = True, None\n",
     (T_PADLESS,), 'KILLED'),
    ('rank-poses-refusal-is-a-bare-keyerror', 'sc',
     "        raise PoseUnrankable(\n"
     "            f\"{ref} cannot be ranked: {why}. Place it with `place_pose set` \"",
     "        raise KeyError(\n"
     "            f\"{ref} cannot be ranked: {why}. Place it with `place_pose set` \"",
     (T_PADLESS,), 'KILLED'),

    # ---- #1000: connector clauses --------------------------------------------
    ('vertical-mount-held-to-the-seat', 'fp',
     "        if setback is None and c.get('class') == 'edge_receptacle' \\\n"
     "                and not vertical:",
     "        if setback is None and c.get('class') == 'edge_receptacle' \\\n"
     "                and True:",
     (T_CONN,), 'KILLED'),
    ('through-edge-writes-an-overhang-floor', 'db',
     "            src['max_setback_mm'] = 'mount_mode'\n"
     "            _row(ref, 'mount_mode', 'compiled',\n"
     "                 'the body reaches the edge",
     "            src['max_setback_mm'] = 'mount_mode'\n"
     "            e['overhang_mm'] = {'min': 0.0}\n"
     "            _row(ref, 'mount_mode', 'compiled',\n"
     "                 'the body reaches the edge",
     (T_CONN,), 'KILLED'),
    ('user-facing-compiles-a-face', 'db',
     "        perp = ce in ('perpendicular_top', 'perpendicular_bottom')\n"
     "        if perp:",
     "        perp = ce in ('perpendicular_top', 'perpendicular_bottom')\n"
     "        if perp or e.get('class') == 'edge_receptacle':",
     (T_CONN,), 'KILLED'),
    ('keepout-clause-judged-by-the-intruder', 'fp',
     "            return 'graded_fail' if keepout in ko_err else 'graded_pass'",
     "            return ('graded_fail' if (ref or '') in by_rule_err.get("
     "'keepout', set()) else 'graded_pass')",
     (T_CONN,), 'KILLED'),

    # ---- #1001: mechanical.json -----------------------------------------------
    ('a-turn-is-only-a-warning', 'fp',
     "        default = ERROR if (turned or not fp.pads) else WARN",
     "        default = WARN",
     (T_RECON,), 'KILLED'),
    ('a-brief-the-run-wrote-is-declared', 'rc',
     "    return 'hypothesis', ('written during an unaided run",
     "    return 'declared', ('written during an unaided run",
     (T_RECON,), 'KILLED'),
    ('p1-never-reconciles', 'dr',
     "    ok_, why_ = _mechanical_owed(a, intent, plan, pcb, _bf, _bp, _brep,\n"
     "                                 _berr)\n",
     "    ok_, why_ = True, None\n",
     (T_RECON,), 'KILLED'),

    # ---- #1002: decaps ------------------------------------------------------
    ('auto-derives-on-a-partial-pile', 'fp',
     "        if _st.unplaced or _st.partially_unplaced:",
     "        if _st.unplaced:",
     (T_DECAPS,), 'KILLED'),
    ('auto-withholds-into-budget-withheld', 'fp',
     "                _census['auto_withheld'] = _why",
     "                _withheld['decaps.max_distance_mm'] = _why",
     (T_DECAPS,), 'KILLED'),
    ('emitted-numbers-unlabelled', 'fp',
     "            'basis': _emitted_basis(_decaps, _budget, conns, blocks,\n"
     "                                    _assembly),",
     "            'basis': {},",
     (T_DECAPS,), 'KILLED'),
    ('declared-relation-supersedes-nothing', 'fp',
     "            if cap in sup:\n"
     "                continue        # graded by the declared relation instead",
     "            if False:\n"
     "                continue        # graded by the declared relation instead",
     (T_DECAPS,), 'KILLED'),

    # ---- the comment's own controls -------------------------------------------
    ('carried-fields-drift-nothing', 'db',
     "        for key in ('edge', 'center_on_edge', 'along_edge_band',\n"
     "                    'overhang_mm', 'max_setback_mm', 'side'):",
     "        for key in ('edge', 'center_on_edge', 'along_edge_band',\n"
     "                    'overhang_mm'):",
     (T_COMMENT,), 'KILLED'),
]

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
                           timeout=3600, cwd=_ROOT)
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
                                   timeout=3600, cwd=_ROOT)
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
