#!/usr/bin/env python3
"""Mutation battery for #923 -- the three blind spots, and their controls.

Green tests are not evidence of coverage, and a gate ADDED to close a blind
spot is exactly the kind of code that can assert nothing while printing PASS.
Every row below breaks one thing the new gates claim to hold down; a row that
survives is a hole, and a row recorded as an expected survivor is a finding
rather than a convenience.

    python3 tests/mutate_923.py            # every row
    python3 tests/mutate_923.py --list
    python3 tests/mutate_923.py --row refusal-flag-returns

A row is KILLED by a FAILURE or an ERROR. An anchor that does not match EXACTLY
ONCE is reported BROKEN, never skipped: a mutation that silently edited nothing
would otherwise be recorded as a surviving row, which is the opposite of what
it means.

Refuses to start on a dirty target tree, because it restores the ORIGINAL text
from disk and would write committed text over uncommitted work.

Four of the thirteen rows mutate the GATES rather than the things they guard.
That is deliberate: the controls inside `test_923_output_key_claims` and the
exit-code analyser in `test_431_skill_commands` are the only reason those two
gates cannot pass on an empty scan, so a battery that never breaks them would
be reporting on a claim nobody tested.
"""
import argparse
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

KILLED, SURVIVED, BROKEN = 'KILLED', 'SURVIVED', 'BROKEN'

TARGETS = {
    'pd': os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement',
                       'scripts', 'placement_driver.py'),
    'ld': os.path.join(REPO, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'scripts',
                       'loop_driver.py'),
    'ps': os.path.join(REPO, '.claude', 'skills', 'plan-pcb-placement',
                       'SKILL.md'),
    'cs': os.path.join(REPO, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'SKILL.md'),
    'rs': os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing',
                       'SKILL.md'),
    'em': os.path.join(REPO, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'references',
                       'evidence-map.md'),
    'rd': os.path.join(REPO, 'py_router', 'routing_defaults.py'),
    'ru': os.path.join(REPO, 'tests', 'run_utils.py'),
    't431': os.path.join(REPO, 'tests', 'test_431_skill_commands.py'),
}

T431 = 'tests/test_431_skill_commands.py'
T923 = 'tests/test_923_output_key_claims.py'

#: (name, target, old, new, tests that must notice, expectation)
ROWS = [
    # ---- blind spot 2: the refusal branches --------------------------------
    # The defect itself. `place_optimize.py` has --suggest-locks-json and no
    # --json, so this recipe exits 2 -- and it sat inside a refusal, where
    # --dump-all never rendered it.
    ('refusal-flag-returns', 'pd',
     "'--suggest-locks-json wk/locks.json')",
     "'--json wk/locks.json')",
     (T431,), KILLED),
    # The anti-drift claim, tested rather than asserted: drop one scenario and
    # the coverage audit must name the line nothing renders.
    ('placement-scenario-dropped', 'pd',
     "        ('a JSON file that does not parse', base + ['--drc-json', unreadable]),\n",
     "",
     (T431,), KILLED),
    ('loop-scenario-dropped', 'ld',
     "        ('a score that is not there', base + ['--score', missing]),\n",
     "",
     (T431,), KILLED),
    # ...and the enumeration the audit compares against. If the AST scan stops
    # finding `err(` sites, "all sites reached" becomes a claim about nothing.
    ('site-enumeration-blinded', 'pd',
     "            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)\n"
     "                    and node.func.id == 'err'):\n"
     "                sites[node.lineno] = (fn.name, 'err', node.end_lineno)",
     "            if False:\n"
     "                sites[node.lineno] = (fn.name, 'err', node.end_lineno)",
     (T431,), KILLED),
    # The wiring: read only the instruction branch again and the citation
    # floor must notice the refusal half has gone.
    ('refusal-text-unread', 't431',
     "        return out + '\\n' + ref",
     "        return out",
     (T431,), KILLED),

    # ---- blind spot 1: what a flag MEANS ------------------------------------
    # #923's acceptance criterion, exactly: move the constant and the skills
    # that quote it must fail.
    ('heuristic-weight-moves', 'rd',
     "HEURISTIC_WEIGHT = 2.3",
     "HEURISTIC_WEIGHT = 2.4",
     (T431,), KILLED),
    # ...and the other direction: the doc quoting the value it used to have.
    ('skill-quotes-the-old-default', 'rs',
     "| `--heuristic-weight 2.3` | 2.3 |",
     "| `--heuristic-weight 1.9` | 1.9 |",
     (T431,), KILLED),
    # The exit-code claim that was true only as a string.
    ('exit-3-claim-returns', 'ps',
     "# Is the board even placed? (report-only, writes nothing -- and it answers on\n"
     "# ANY board: --suggest-locks returns before the board-state gate, so read the\n"
     "# ADVICE, never the exit code)",
     "# Is the board even placed? (report-only, writes nothing, exits 3 if not)",
     (T431,), KILLED),
    # The analyser behind it: if it stops seeing the early return, every
    # exit-code claim reads as fine.
    ('exit-analyser-blinded', 't431',
     "            if any(isinstance(x, ast.Return) for x in ast.walk(node)):\n"
     "                return True",
     "            if any(isinstance(x, ast.Return) for x in ast.walk(node)):\n"
     "                return False",
     (T431,), KILLED),

    # ---- blind spot 3: a claim about a tool's OUTPUT ------------------------
    # The key that started this. `hot` is a local inside check_pockets.
    ('hot-ratio-returns', 'cs',
     "(`windows[0].ratio`",
     "(`hot[].ratio`",
     (T923,), KILLED),
    # A key renamed in the doc rather than in the code.
    ('evidence-map-key-misspelt', 'em',
     "| `metrics.crossings`, `metrics.hpwl` |",
     "| `metrics.crossing`, `metrics.hpwl` |",
     (T923,), KILLED),
    # The resolver, which is the only reason an unresolvable key is a failure.
    ('resolver-accepts-anything', 'ru',
     "        if not nxt:\n            return False",
     "        if not nxt:\n            return True",
     (T923,), KILLED),
    # The filter that keeps file names out of the key scan. Without it
    # `route.py` reads as a key claim and the control says so.
    ('path-filter-dropped', 'ru',
     "    if text.lower().endswith(_PATH_NOT_A_KEY):\n        return None",
     "    if False:\n        return None",
     (T923,), KILLED),
]


def _dirty():
    r = subprocess.run(['git', 'status', '--porcelain'] +
                       sorted(set(TARGETS.values())),
                       cwd=REPO, capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def run_row(row, keep=False):
    name, target, old, new, tests, _expect = row
    path = TARGETS[target]
    with open(path, encoding='utf-8') as fh:
        original = fh.read()
    if original.count(old) != 1:
        return BROKEN, f'anchor matched {original.count(old)} time(s)'
    try:
        with open(path, 'w', encoding='utf-8', newline='') as fh:
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
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(original)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--row', action='append', default=[])
    a = ap.parse_args()

    if a.list:
        for name, target, _o, _n, tests, expect in ROWS:
            print(f"  {name:32} {target:5} {expect:8} {' '.join(tests)}")
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
        print(f'  {row[0]:32} {got:9} {flag} {detail}')
    print(f"\n{len(rows)} rows: {counts[KILLED]} killed, "
          f"{counts[SURVIVED]} survived, {counts[BROKEN]} broken, "
          f"{disagreed} disagreeing with expectation")
    return 1 if (counts[BROKEN] or disagreed) else 0


if __name__ == '__main__':
    sys.exit(main())
