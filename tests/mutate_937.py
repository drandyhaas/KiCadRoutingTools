#!/usr/bin/env python3
"""Mutation battery for #937 -- each acceptance criterion, proved to kill.

#937 asks for a catalogue nothing can silently drop a tool from, an entry gate
at the door that had none, a checker that proves a routing plan, and thresholds
that report instead of deciding. Every one of those is a GATE, and this repo's
recurring failure is a new gate that asserts nothing while printing PASS --
three of the arms added for #936 did exactly that before a verifier broke
them, and three more in this PR were caught the same way. So every row below
reverts one fix and names the test that must go red.

    python3 tests/mutate_937.py            # every row
    python3 tests/mutate_937.py --list
    python3 tests/mutate_937.py --row scope-collapsed

A row is KILLED by a FAILURE or an ERROR. An anchor that does not match
EXACTLY ONCE is reported BROKEN, never skipped: a mutation that edited nothing
would otherwise be recorded as a surviving row, which is the opposite of what
it means. Read the EXIT CODE of a killed row, not just the verdict --
3221225794 (0xC000013A) is an interrupt, not a mutation, and a battery
interrupted mid-run prints a wall of meaningless `KILLED ok`.

Refuses to start on a dirty target tree, because it restores the ORIGINAL text
from disk and would write committed text over uncommitted work.

`scope-collapsed` is the anti-vacuity row and the one worth reading twice: it
sets every tool's scope to ONE door, leaving the global count untouched at 72.
A registry gate with only a global floor passes that mutation while every
routing reader is stranded -- which is exactly the failure the per-door floors
exist for.

`arm-ceiling-undeclared` runs `test_431_skill_commands.py`, which costs ~110 s.
It is the one slow row and it earns it: nothing else asserts that
`loop_driver --dump-all` exits 0, and that exit code is what holds the twelve
populated-arm ceilings down.
"""
import argparse
import io
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

KILLED, SURVIVED, BROKEN = 'KILLED', 'SURVIVED', 'BROKEN'

TARGETS = {
    'reg': os.path.join(REPO, 'krt_registry.py'),
    'cyc': os.path.join(REPO, 'py_tools', 'check_cycles.py'),
    'chk': os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing',
                        'scripts', 'route_plan_check.py'),
    'leg': os.path.join(REPO, 'py_placer', 'placement', 'legality.py'),
    'pd': os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement',
                       'scripts', 'placement_driver.py'),
    'ld': os.path.join(REPO, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'scripts',
                       'loop_driver.py'),
    'rt': os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing',
                       'SKILL.md'),
    'cv': os.path.join(REPO, 'py_placer', 'converge.py'),
}

T_REG = 'tests/test_937_tool_registry.py'
T_CHK = 'tests/test_937_route_plan_check.py'
T_OOB = 'tests/test_937_off_outline_channels.py'
T_GATE = 'tests/test_937_entry_gates.py'
T_LEV = 'tests/test_937_lever_identity.py'
T_DRV = 'tests/test_run8_skill_drivers.py'
T_431 = 'tests/test_431_skill_commands.py'

#: (name, target, old, new, tests that must notice, expectation)
ROWS = [
    # ---- the catalogue is complete, and gated ------------------------------
    ('declaration-dropped', 'cyc',
     "KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}",
     "_KRT_TOOL_WAS_HERE = None",
     (T_REG,), KILLED),
    # THE ANTI-VACUITY ROW. The global count does not move; the routing and
    # combined doors empty out. Only a PER-DOOR floor can see this.
    ('scope-collapsed', 'reg',
     "            'scope': list(scope) if isinstance(scope, (list, tuple)) else None,",
     "            'scope': ['placement'],",
     (T_REG,), KILLED),
    # The cross-check that caught itself: a substring scan reports the module
    # that EXPLAINS record_invocation as a caller of it.
    ('self-records-by-substring', 'reg',
     "    for node in ast.walk(tree):\n        if not isinstance(node, ast.Call):\n            continue",
     "    for node in ast.walk(tree):\n        if 'record_invocation' in ast.dump(tree):\n            return True\n        if not isinstance(node, ast.Call):\n            continue",
     (T_REG,), KILLED),

    # ---- the routing door has an entry gate --------------------------------
    ('routing-step0-deleted', 'rt',
     '## Step 0: the placement gate -- is this board ready to route?',
     '## Notes on placement (informational)',
     (T_GATE,), KILLED),
    ('routing-howto-deleted', 'rt',
     '## How to run this skill',
     '## Some background',
     (T_GATE,), KILLED),

    # ---- P2/P3 read the VERDICT, not one of its five conjuncts -------------
    ('guard-damage-reads-blocking', 'pd',
     "        undamaged = (not blocking) if buildable is None else bool(buildable)",
     "        undamaged = not blocking",
     (T_GATE,), KILLED),

    # ---- the plan checker --------------------------------------------------
    # A raw-text rule: invisible to the converted plan steps, which is the
    # design decision this row regression-tests.
    ('checker-verification-rule-blinded', 'chk',
     "        t = tool_of(s.split())\n        if t and t.startswith('check_'):",
     "        t = tool_of(s.split())\n        if False:",
     (T_CHK,), KILLED),
    # Step 5b's two asserts, which shipped in a fenced block no test ran.
    ('checker-net-coverage-blinded', 'chk',
     "    orphans = exclusions ^ impedance_se",
     "    orphans = set()",
     (T_CHK,), KILLED),
    # A refusal that exits 0 is a checker nobody has to obey.
    ('checker-refusal-exits-clean', 'chk',
     "CLEAN, CRASH, USAGE, UNREADABLE, REFUSED = 0, 1, 2, 3, 4",
     "CLEAN, CRASH, USAGE, UNREADABLE, REFUSED = 0, 1, 2, 3, 0",
     (T_CHK,), KILLED),

    # ---- both off-outline channels travel ----------------------------------
    ('per-pad-channel-dropped', 'leg',
     "            if amt > EPS:\n                oob_copper_refs.append([ref, round(amt, 4)])",
     "            if False:\n                oob_copper_refs.append([ref, round(amt, 4)])",
     (T_OOB,), KILLED),

    # ---- the threshold reports, the model disposes -------------------------
    ('congestion-silent-above-the-cut', 'ld',
     "        return True, (\n            f'  CONGESTION READ: hpwl {b:.1f} -> {n:.1f} '",
     "        return True, None\n        return True, (\n            f'  CONGESTION READ: hpwl {b:.1f} -> {n:.1f} '",
     (T_DRV,), KILLED),

    # ---- every populated arm is measured -----------------------------------
    # Drops one arm from the ceiling table; the dump must refuse an arm it
    # cannot hold, and test_431 asserts that dump exits 0. ~110 s.
    ('arm-ceiling-undeclared', 'ld',
     "    'L3': 75, 'L4': 45, 'L5': 40,",
     "    'L4': 45, 'L5': 40,",
     (T_431,), KILLED),

    # ---- the lever identity is structured, never the prose -----------------
    ('lever-read-from-the-prose', 'cv',
     "    argv = entry.get('lever_argv') if isinstance(entry, dict) else None",
     "    argv = (entry.get('lever_argv') or [entry.get('lever')]) \\\n        if isinstance(entry, dict) else None",
     (T_LEV,), KILLED),
]


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_anchors import preflight                       # noqa: E402
preflight(__file__)


def _dirty():
    r = subprocess.run(['git', 'status', '--porcelain']
                       + sorted(set(TARGETS.values())),
                       cwd=REPO, capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def run_row(row, keep=False):
    name, target, old, new, tests, _expect = row
    path = TARGETS[target]
    # newline='' on BOTH sides: read with universal newlines and write with
    # none and every CRLF file comes back LF, so a row that RESTORED its
    # target still left the tree dirty.
    with io.open(path, encoding='utf-8', newline='') as fh:
        original = fh.read()
    if '\r\n' in original:
        old = old.replace('\n', '\r\n')
        new = new.replace('\n', '\r\n')
    if original.count(old) != 1:
        return BROKEN, f'anchor matched {original.count(old)} time(s)'
    try:
        with io.open(path, 'w', encoding='utf-8', newline='') as fh:
            fh.write(original.replace(old, new, 1))
        for t in tests:
            r = subprocess.run([sys.executable, '-X', 'utf8',
                                os.path.join(REPO, t)],
                               cwd=REPO, capture_output=True, text=True,
                               encoding='utf-8', errors='replace')
            if r.returncode != 0:
                return KILLED, f'{t} exit {r.returncode}'
        return SURVIVED, ''
    finally:
        if not keep:
            with io.open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(original)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--row', action='append', default=[])
    a = ap.parse_args()

    if a.list:
        for name, target, _o, _n, tests, expect in ROWS:
            print(f"  {name:34} {target:5} {expect:8} {' '.join(tests)}")
        return 0

    dirty = _dirty()
    if dirty:
        print('REFUSING: the target tree is dirty. This restores the ORIGINAL '
              'text from disk and would write committed text over uncommitted '
              'work.')
        for ln in dirty:
            print(f'  {ln}')
        return 2

    rows = [r for r in ROWS if not a.row or r[0] in a.row]
    if a.row and not rows:
        print(f'no row matches {a.row}')
        return 2
    counts = {KILLED: 0, SURVIVED: 0, BROKEN: 0}
    disagreed = 0
    for row in rows:
        got, detail = run_row(row)
        counts[got] += 1
        flag = 'ok  ' if got == row[5] else 'DISAGREES'
        if got != row[5]:
            disagreed += 1
        print(f'  {row[0]:34} {got:9} {flag} {detail}')
    print(f"\n{len(rows)} rows: {counts[KILLED]} killed, "
          f"{counts[SURVIVED]} survived, {counts[BROKEN]} broken, "
          f"{disagreed} disagreeing with expectation")
    return 1 if (counts[BROKEN] or disagreed) else 0


if __name__ == '__main__':
    sys.exit(main())
