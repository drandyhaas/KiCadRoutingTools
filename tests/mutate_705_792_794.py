#!/usr/bin/env python3
"""The #705/#792/#794 mutation battery, shipped so its numbers can be re-derived.

Same contract as `tests/mutate_799.py`, which this copies: a row is KILLED by a
failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN, never
silently skipped; `str.replace(old, new, 1)`, never `sed`; originals restored in
a `finally`; a refusal to start on a dirty engine, because restoring would write
the committed text back over uncommitted work; and the `.pyc` defence, with a
`--selftest` that PROVES it rather than asserting it.

THE MEASURED TABLE LIVES IN THE HEADER OF `test_705_decap_pin_distance.py`,
FROM THE RUN -- never predicted here and never edited afterwards to match.

TWO ROWS ARE EXPECTED SURVIVORS, and both are findings rather than holes. Each
carries its reason inline. A survivor recorded with a reason is a change
detector; a survivor quietly deleted is a hole.

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. One writer per tree -- do not run this while a suite is
reading the same worktree.

    python3 tests/mutate_705_792_794.py
    python3 tests/mutate_705_792_794.py --row ground-pins-are-graded
    python3 tests/mutate_705_792_794.py --list
    python3 tests/mutate_705_792_794.py --selftest
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
GROUPS = os.path.join(_ROOT, 'py_placer', 'placement', 'groups.py')
SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')
NETQ = os.path.join(_ROOT, 'py_router', 'net_queries.py')
TARGETS = {'fp': FLOORPLAN, 'grp': GROUPS, 'sdr': SEEDER, 'nq': NETQ}

T705 = os.path.join(_TESTS, 'test_705_decap_pin_distance.py')
T794 = os.path.join(_TESTS, 'test_794_decap_horizon.py')
T792P = os.path.join(_TESTS, 'test_792_decap_predicate.py')
T792S = os.path.join(_TESTS, 'test_792_decap_seeding.py')
T704 = os.path.join(_TESTS, 'test_704_decap_emission.py')
T549S = os.path.join(_TESTS, 'test_549_floorplan_schema.py')

ROWS = [
    # ---- #794: the horizon rule -------------------------------------------
    ('the-horizon-rule-never-fires', 'fp',
     "    for cap, ic, dist in beyond:\n",
     "    for cap, ic, dist in []:\n",
     (T794,), 'KILLED'),

    ('the-severity-defaults-to-ERROR', 'fp',
     "    sev = ctx.intent.severity_of('decap_ungraded', default=WARN)\n",
     "    sev = ctx.sev('decap_ungraded')\n",
     (T794,), 'KILLED'),

    ('the-worst-beyond-is-the-last-cap-again', 'fp',
     "        'worst_beyond_mm': (round(max(d for _c, _ic, d in beyond), 4)\n",
     "        'worst_beyond_mm': (round(sorted(beyond)[-1][2], 4)\n",
     (T704,), 'KILLED'),

    ('the-withheld-key-disarms-only-decap_distance', 'fp',
     "        ('decap_distance', 'decap_ungraded'),\n",
     "        ('decap_distance',),\n",
     (T794,), 'KILLED'),

    # ---- the single election ----------------------------------------------
    # EXPECTED SURVIVOR, measured, and predicted KILLED. `unaccounted` is a
    # TRIPWIRE: its correct value is 0 on every board, so no test can tell a
    # computed 0 from a literal one without a board where the partition
    # actually leaks. Recorded rather than deleted, and paired with the row
    # below -- which breaks the ELECTION so the tripwire has something to
    # catch, and is what proves the key is not decoration.
    ('the-partition-does-not-add-up', 'fp',
     "        'unaccounted': scope - n - len(beyond) - len(orphans),\n",
     "        'unaccounted': 0,\n",
     (T704,), 'SURVIVED'),

    ('the-election-drops-a-cap-so-the-tripwire-must-fire', 'grp',
     "        out.append((ref, best, best_d))\n",
     "        if ref.endswith('1'):\n"
     "            continue\n"
     "        out.append((ref, best, best_d))\n",
     (T704, T792P), 'KILLED'),

    # EXPECTED SURVIVOR, and it is the finding. Re-inserting the recheck that
    # `_elect_tethers` deleted changes nothing, which is the PROOF it was dead:
    # `power` is a subset of `nets`, and `best` is only assigned inside a
    # branch that already required `power & ic_nets[c]` to be non-empty. Kept
    # rather than dropped so it starts failing the day the election's shape
    # changes enough to make the branch reachable again.
    ('the-dead-electrical-recheck-is-restored', 'grp',
     "        out.append((ref, best, best_d))\n",
     "        if best is not None and not (nets & ic_nets.get(best, set())):\n"
     "            continue\n"
     "        out.append((ref, best, best_d))\n",
     (T792P, T704), 'SURVIVED'),

    ('the-radius-reaches-the-election', 'grp',
     "            if best_d is None or d < best_d:\n",
     "            if d > DECAP_RADIUS_MM:\n"
     "                continue\n"
     "            if best_d is None or d < best_d:\n",
     (T792P, T794), 'KILLED'),

    # ---- #705: the supply-pin ladder --------------------------------------
    ('the-ladder-keys-on-field-presence-not-yield', 'fp',
     "        won = next((c for c in order if chan[c]), None)\n",
     "        _present = {'pintype': any(getattr(p, 'pintype', '')\n"
     "                                   for p, _n in cands),\n"
     "                    'pinfunction': any(getattr(p, 'pinfunction', '')\n"
     "                                       for p, _n in cands),\n"
     "                    'rail_net': True}\n"
     "        won = next((c for c in order if _present[c]), None)\n",
     (T705,), 'KILLED'),

    ('pintype-is-matched-by-equality', 'nq',
     "    tokens = (pintype or '').split('+')\n"
     "    return (('power_in' in tokens or 'power_out' in tokens)\n"
     "            and 'no_connect' not in tokens)\n",
     "    return (pintype or '') in ('power_in', 'power_out')\n",
     (T792P,), 'KILLED'),

    ('pintype-is-matched-by-startswith', 'nq',
     "    tokens = (pintype or '').split('+')\n"
     "    return (('power_in' in tokens or 'power_out' in tokens)\n"
     "            and 'no_connect' not in tokens)\n",
     "    return (pintype or '').startswith(('power_in', 'power_out'))\n",
     (T792P,), 'KILLED'),

    ('ground-pins-are-graded', 'fp',
     "    return None if is_ground_net_name(net) else net\n",
     "    return net\n",
     (T705,), 'KILLED'),

    ('an-unconnected-pad-is-graded', 'fp',
     "    if net_id <= 0 or net.startswith('unconnected-'):\n",
     "    if net_id <= 0:\n",
     (T705, T792P), 'KILLED'),

    ('the-gap-is-measured-to-the-caps-nearest-pad', 'fp',
     "        if q.net_id != net_id:\n",
     "        if False:\n",
     (T705,), 'KILLED'),

    # `return None or X` is X, so the first draft of this row mutated nothing.
    # Emptying the table is the real "the rule runs even when the board cannot
    # answer" -- which is the vacuous pass `--require-rules` exists to catch.
    ('the-arm-table-is-empty-so-the-rule-always-runs', 'fp',
     "_ARM = {'decap_pin_distance': _arm_decap_pins}\n",
     "_ARM = {}\n",
     (T705,), 'KILLED'),

    # EXPECTED SURVIVOR, and the reason lives one level up in the design:
    # channel 3 only admits a pin whose net ALREADY carries a decoupling cap,
    # so an inferred pin's rail can never BE uncovered and this guard cannot
    # fire either way. Belt-and-braces, not load-bearing. Predicted KILLED;
    # kept so it starts failing the day that conjunct is relaxed.
    ('the-uncovered-rail-is-emitted-for-an-INFERRED-pin-too', 'fp',
     "                if not inferred:\n",
     "                if True:\n",
     (T705,), 'SURVIVED'),

    ('inferred-findings-share-the-declared-rule-name', 'fp',
     "            if inferred:\n"
     "                yield Violation(rule='decap_pin_distance_inferred',\n"
     "                                **payload)\n"
     "            else:\n"
     "                yield Violation(rule='decap_pin_distance', **payload)\n",
     "            yield Violation(rule='decap_pin_distance', **payload)\n",
     (T705, T549S), 'KILLED'),

    # ---- #792: the seeder --------------------------------------------------
    ('the-scope-is-syntactic-again', 'sdr',
     "        near, beyond, _orphans = _groups.decap_populations(pcb_data)\n"
     "        tethered = ({c for caps in near.values() for c, _d in caps}\n"
     "                    | {c for c, _ic, _d in beyond})\n"
     "        decap_scope = {r for r in tethered\n"
     "                       if r in state.parts\n"
     "                       and not any(fnmatch.fnmatch(r, pat) for pat in exempt)}\n",
     "        decap_scope = {r for r in state.parts\n"
     "                       if r[0] == 'C' and state.parts[r].pin_count == 2\n"
     "                       and not any(fnmatch.fnmatch(r, pat) for pat in exempt)}\n",
     (T792S,), 'KILLED'),

    ('the-put-back-is-deleted', 'sdr',
     "        for ref in _order(sorted(unplaced & decap_scope)):\n",
     "        for ref in []:\n",
     (T792S,), 'KILLED'),

    # The seat still happens; only the note is suppressed. It SURVIVED the
    # first run, because in the arm that names it the declined cap has a
    # declared zone and the 2.6 put-back emits its own note -- the pass-2 note
    # was masked by the fix that follows it. A no-zone arm now covers it.
    ('the-declined-cap-is-silent-again', 'sdr',
     "                if not _seat(ref, cx2, cy2, cluster[0][1], rail):\n",
     "                if _seat(ref, cx2, cy2, cluster[0][1], rail) and False:\n",
     (T792S,), 'KILLED'),

    # The SECOND silent path, found by writing the arm that kills the first:
    # `zip` truncates to the shorter list, so a cap past the cluster count is
    # never reached by the loop at all and vanished without a note -- while the
    # comment below it claimed the fall-through "reports honestly".
    ('the-cap-no-cluster-wanted-is-silent-again', 'sdr',
     "            for ref in caps_r[len(clusters):]:\n",
     "            for ref in []:\n",
     (T792S,), 'KILLED'),

    ('the-owner-test-drops-the-collinear-guard', 'grp',
     "    return {c.reference\n"
     "            for c in build_chip_list(pcb_data, min_pads=DECAP_MIN_IC_PADS)\n"
     "            if not _pads_are_collinear(pcb_data.footprints.get(c.reference))}\n",
     "    return {c.reference\n"
     "            for c in build_chip_list(pcb_data, min_pads=DECAP_MIN_IC_PADS)}\n",
     (T792S, T792P), 'KILLED'),
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
    The probe must be able to TELL THE TWO MUTANTS APART -- an earlier version
    of this idea in `mutate_799.py` printed a count that was identical for both
    mutants and for the original, and would have reported OK against a
    completely stale cache.
    """
    if not _git_clean([NETQ]):
        print("REFUSED: net_queries.py is dirty", file=sys.stderr)
        return 2
    src = io.open(NETQ, encoding='utf-8').read()
    anchor = ("POWER_PIN_KEYWORDS = ('VCC', 'VDD', 'VSS', 'GND', 'VCCA', "
              "'VSSA', 'VDDA',\n")
    if src.count(anchor) != 1:
        print(f"BROKEN selftest: anchor matched {src.count(anchor)} times",
              file=sys.stderr)
        return 2
    probe = [sys.executable, '-B', '-X', 'utf8', '-c',
             "import sys; sys.path.insert(0,'py_router');"
             "import net_queries as q;"
             "print(q.POWER_PIN_KEYWORDS[0])"]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    seen = []
    try:
        for tok in ("'VCA'", "'VCB'"):
            _drop_pyc()
            repl = anchor.replace("'VCC'", tok)
            io.open(NETQ, 'w', encoding='utf-8', newline='').write(
                src.replace(anchor, repl, 1))
            r = subprocess.run(probe, cwd=_ROOT, capture_output=True,
                               text=True, encoding='utf-8', env=env)
            seen.append(r.stdout.strip())
    finally:
        _drop_pyc()
        io.open(NETQ, 'w', encoding='utf-8', newline='').write(src)
    ok = len(seen) == 2 and all(seen) and seen[0] != seen[1]
    print(f"  selftest: two same-second size-preserving mutations, probe read "
          f"{seen} -- "
          + ('OK: the second import saw the SECOND mutant' if ok else
             'the probe cannot tell the mutants apart, or the second read a '
             'STALE .pyc'))
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
