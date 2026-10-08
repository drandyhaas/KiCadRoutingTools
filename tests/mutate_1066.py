#!/usr/bin/env python3
"""The #1066 mutation battery: `repair_placement`'s intent honesty re-grade.

`tests/test_repair_decap_honesty.py` pins that a violator is reported
repaired only when the grade error it was CHARGED for is gone. Each row below
is a plausible half-fix of that re-grade; each must be KILLED by that file.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
engine, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 tests/mutate_1066.py
    python3 tests/mutate_1066.py --row regrade-only-zero-move
    python3 tests/mutate_1066.py --list

A row is KILLED by a FAILURE **or an ERROR**. An anchor that does not match
EXACTLY ONCE is reported as BROKEN rather than skipped. Python `str.replace`,
never `sed`.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')
PLACE_SEED = os.path.join(_ROOT, 'py_placer', 'place_seed.py')
TARGETS = {'s': SEEDER, 'p': PLACE_SEED}

T1066 = os.path.join(_TESTS, 'test_repair_decap_honesty.py')

ROWS = [
    # The obvious half-fix: re-grade only the parts that did not move. A cap
    # moved off a pad conflict and still too far from its IC reads repaired.
    ('regrade-only-zero-move', 's',
     "    check_refs = [r for r in dict.fromkeys(repaired)\n"
     "                  if r in charged_claims or r in moved_refs]\n",
     "    check_refs = [r for r in dict.fromkeys(repaired)\n"
     "                  if (r in charged_claims or r in moved_refs)\n"
     "                  and r in zero_move]\n",
     (T1066,), 'KILLED'),

    # A charged claim the pose grader never produces (it comes from `grade`
    # outside the rules loop) read as cleared because it is absent after.
    ('invariant-claims-read-as-cleared', 's',
     "                                if c in after_claims\n"
     "                                or c not in claims_regradable\n",
     "                                if c in after_claims\n",
     (T1066,), 'KILLED'),

    # A re-grade that raises treated as "nothing left", i.e. repaired.
    ('a-raising-regrade-reads-as-clear', 's',
     "            except (floorplan.UntrustworthyOutline, ValueError) as exc:\n"
     "                regrade_error = exc\n"
     "        after_claims = ({floorplan.violation_claim(v) for v in after\n",
     "            except (floorplan.UntrustworthyOutline, ValueError) as exc:\n"
     "                after = []\n"
     "        after_claims = ({floorplan.violation_claim(v) for v in after\n",
     (T1066,), 'KILLED'),

    # The census stops recording what each ref was charged for.
    ('charged-claims-not-recorded', 's',
     "                if v.ref in state.parts:\n"
     "                    charged_claims.setdefault(v.ref, []).append(\n",
     "                if False:\n"
     "                    charged_claims.setdefault(v.ref, []).append(\n",
     (T1066,), 'KILLED'),

    # The phase-1 verifier's counterexample: only the CHARGED claims are
    # asked about, so an error the move itself created goes unseen.
    ('a-move-that-creates-an-error-reads-repaired', 's',
     "            for r in who:\n"
     "                created.setdefault(r, []).append(label)\n",
     "            for r in ():\n"
     "                created.setdefault(r, []).append(label)\n",
     (T1066,), 'KILLED'),

    # A created error is attributed to its own ref only, so a cap that
    # stranded its IC's supply pin (the error names the IC) reads repaired.
    ('a-created-pin-error-attributed-to-the-ic-only', 's',
     "            names = {v.ref} | {m.get(k) for k in ('cap', 'ic', 'near')}\n",
     "            names = {v.ref} | {m.get(k) for k in ('ic', 'near')}\n",
     (T1066,), 'KILLED'),

    # Leaving the decap search radius reads as the charge cleared.
    ('leaving-the-radius-clears-the-charge', 's',
     "                                or (c[0] == 'decap_distance'\n"
     "                                    and ref in ungraded)})\n",
     "                                or False})\n",
     (T1066,), 'KILLED'),

    # Round 2: a finding naming no moved ref is left unattributed -- the
    # counterfactual (restore each moved ref alone) is gone.
    ('no-counterfactual-attribution', 's',
     "            if not who:\n"
     "                k, amt = finding_key(v), finding_amount(v)\n",
     "            if False:\n"
     "                k, amt = finding_key(v), finding_amount(v)\n",
     (T1066,), 'KILLED'),

    # A finding that only GREW reads as no change.
    ('a-worse-finding-reads-unchanged', 's',
     "        if a is not None and b is not None and a > b + FINDING_WORSE_EPS_MM:\n",
     "        if False:\n",
     (T1066,), 'KILLED'),

    # The finding is identified by its claim alone: a second stranded pin
    # under a claim the IC already carried is invisible.
    ('a-finding-is-only-its-claim', 's',
     "    return _fp.violation_claim(v) + (str(m.get('pad', '')),\n"
     "                                     str(m.get('net', '')))\n",
     "    return _fp.violation_claim(v) + ('', '')\n",
     (T1066,), 'KILLED'),

    # ---- (b) the decap rung, opt-in ----------------------------------------
    # On by default.
    ('the-rung-defaults-on', 's',
     "                     repair_decaps: bool = False,\n"
     "                     baseline_file: Optional[str] = None,\n",
     "                     repair_decaps: bool = True,\n"
     "                     baseline_file: Optional[str] = None,\n",
     (T1066,), 'KILLED'),

    # The rung moves the IC, not the cap.
    ('the-rung-moves-the-ic', 's',
     "            cap, ic, pad = v.ref, m.get('ic'), None\n",
     "            cap, ic, pad = m.get('ic'), v.ref, None\n",
     (T1066,), 'KILLED'),

    # A fixing pose that adds a finding elsewhere is kept.
    ('the-rung-ignores-what-it-adds', 's',
     "        if added or still:\n",
     "        if still:\n",
     (T1066,), 'KILLED'),

    # No proportion budget: a cap 0.96mm past its limit moves 12.9mm.
    ('the-rung-has-no-proportion', 's',
     "        if d > budget:\n"
     "            state.apply_move(cap, ox, oy, orot)\n",
     "        if False:\n"
     "            state.apply_move(cap, ox, oy, orot)\n",
     (T1066,), 'KILLED'),

    # The rung asks the CLAIM whether its pin is fixed: two caps on two pins
    # of one IC revert each other.
    ('the-rung-asks-the-claim-not-the-pin', 's',
     "        still = (claim in findings_of(after)\n",
     "        still = (claim[:4] in {k[:4] for k in findings_of(after)}\n",
     (T1066,), 'KILLED'),

    # Leaving the decap search radius reads as the rung's fix.
    ('the-rung-calls-leaving-the-radius-a-fix', 's',
     "                 or (claim[0] == 'decap_distance'\n",
     "                 or (False\n",
     (T1066,), 'KILLED'),

    # A seat that grows the courtyard overlap is kept.
    ('the-rung-ignores-legality-growth', 's',
     "                added.append(f'legality.{key}')\n",
     "                pass\n",
     (T1066,), 'KILLED'),

    # A locked cap is searched like any other.
    ('the-rung-moves-a-locked-cap', 's',
     "        if part.locked:\n"
     "            row['result'] = 'locked'\n",
     "        if False:\n"
     "            row['result'] = 'locked'\n",
     (T1066,), 'KILLED'),

    # The refs behind the count never reach JSON_SUMMARY.
    ('unresolved-refs-not-written', 'p',
     "                'unresolved_refs': _unres,\n",
     "",
     (T1066,), 'KILLED'),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten. A stale anchor otherwise reports BROKEN mid-run, after the
# witnesses have been paid for; this is the one second (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=_ROOT)
    return r.returncode == 0


def _run(tests):
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=_ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        if r.returncode != 0:
            return True, f"{os.path.basename(t)} exit {r.returncode}"
    return False, "all named tests passed"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()

    if a.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print(f"  {exp:9} {name}  [{tgt}] "
                  f"-> {', '.join(os.path.basename(t) for t in tests)}")
        return 0

    rows = ROWS
    if a.row:
        unknown = [n for n in a.row if n not in {r[0] for r in ROWS}]
        if unknown:
            print(f"no such row: {', '.join(unknown)}; try --list",
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in set(a.row)]

    if not _git_clean(TARGETS.values()):
        print("REFUSED: the engine files are dirty. Restoring would write the "
              "COMMITTED text back over uncommitted work.", file=sys.stderr)
        return 2

    # The killers, UNMUTATED, must pass first: `_run` counts any non-zero
    # exit as KILLED, so a broken fixture or an ImportError would otherwise
    # score every row killed (mutate_974's gate, and its reason).
    killers = sorted({t for r in rows for t in r[4]})
    red, why = _run(killers)
    if red:
        print(f"REFUSED: the killer tests fail UNMUTATED ({why}) -- every row "
              f"would read KILLED for a reason unrelated to its mutation.",
              file=sys.stderr)
        return 2
    for t in killers:
        print(f"  unmutated {os.path.basename(t):40} passes")
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
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(
                src.replace(old, new, 1))
            killed, why = _run(tests)
            io.open(TARGETS[tgt], 'w', encoding='utf-8',
                    newline='').write(src)
            got = 'KILLED' if killed else 'SURVIVED'
            mark = 'ok' if got == expect else 'WRONG'
            verdicts.append((name, got, why))
            print(f"  {got:9}{'' if mark == 'ok' else ' WRONG'} {name} -- {why}")
    finally:
        for k, p in TARGETS.items():
            io.open(p, 'w', encoding='utf-8', newline='').write(originals[k])

    wrong = [v for v, (name, got, _w) in zip(rows, verdicts)
             if got != v[5]]
    broken = [n for n, g, _w in verdicts if g == 'BROKEN']
    print(f"\n{len(verdicts)} row(s): "
          f"{sum(1 for _n, g, _w in verdicts if g == 'KILLED')} killed, "
          f"{sum(1 for _n, g, _w in verdicts if g == 'SURVIVED')} survived, "
          f"{len(broken)} broken, {len(wrong)} disagreeing with expectation")
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
