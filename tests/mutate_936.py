#!/usr/bin/env python3
"""Mutation battery for #936 -- each acceptance criterion, proved to kill.

#936 is 34 corrected claims across three skills, two drivers and one crash. A
correction that no gate holds down is a correction the next edit undoes, and
this repo's recurring failure is a NEW gate that asserts nothing while printing
PASS -- three of the arms added for #936 did exactly that before a verifier
broke them. So every row below reverts one fix and names the test that must go
red.

    python3 tests/mutate_936.py            # every row
    python3 tests/mutate_936.py --list
    python3 tests/mutate_936.py --row handler-returns

A row is KILLED by a FAILURE or an ERROR. An anchor that does not match EXACTLY
ONCE is reported BROKEN, never skipped: a mutation that edited nothing would
otherwise be recorded as a surviving row, which is the opposite of what it
means. Read the EXIT CODE of a killed row, not just the verdict -- 3221225794
(0xC000013A) is an interrupt, not a mutation, and a battery interrupted
mid-run prints a wall of meaningless `KILLED ok`.

Refuses to start on a dirty target tree, because it restores the ORIGINAL text
from disk and would write committed text over uncommitted work.

Four rows mutate a GATE rather than the thing it guards
(`list-hand-tuple`, `of-hardcoded`, `composed-flags-blinded`,
`better-line-pinned`). That is deliberate: those gates are the only reason the
corresponding claims cannot drift, so a battery that never breaks them would be
reporting on a claim nobody tested.

`better-line-pinned` runs `test_431_skill_commands.py`, which costs ~170 s.
It is the one slow row and it earns it: it is the acceptance criterion for B6,
and the defect it re-creates is a gate pinning the stale citation it exists to
prevent.
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
    'bs': os.path.join(REPO, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'scripts',
                       'board_score.py'),
    'em': os.path.join(REPO, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'references',
                       'evidence-map.md'),
    'pr': os.path.join(REPO, 'kicad_routing_plugin', 'placement_run.py'),
    'cv': os.path.join(REPO, 'py_placer', 'converge.py'),
    'dfl': os.path.join(REPO, 'tests', 'test_doc_flag_liveness.py'),
    't431': os.path.join(REPO, 'tests', 'test_431_skill_commands.py'),
}

T_WORKLIST = 'tests/test_broken_worklist.py'
T_DRIVERS = 'tests/test_run8_skill_drivers.py'
T_RUN = 'tests/test_placement_run.py'
T_CONVERGE = 'tests/test_converge.py'
T_DFL = 'tests/test_doc_flag_liveness.py'
T_923 = 'tests/test_923_output_key_claims.py'
T_431 = 'tests/test_431_skill_commands.py'

#: (name, target, old, new, tests that must notice, expectation)
ROWS = [
    # ---- C1: the handler names a tool that exists ---------------------------
    ('handler-returns', 'bs',
     "        v['handler'] = ('repair_planes' if name in poured",
     "        v['handler'] = ('route_disconnected_planes' if name in poured",
     (T_WORKLIST,), KILLED),

    # ---- C2: one procedure, one stage count ---------------------------------
    # The defect itself: a hand-written tuple beside the real registry, which
    # had silently lost P-brief -- the only stage that records a design fact.
    ('list-hand-tuple', 'pd',
     "        for key in STAGES:\n",
     "        for key in ('P0', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P-close'):\n",
     (T_DRIVERS,), KILLED),
    # ...and the `of=` count a model reads as its progress. This one MUST be
    # checked on the stage BODY: measured on the cheap arm, P4 refuses, a
    # refusal carries no `of=` tag, and the check passed unconditionally.
    ('of-hardcoded', 'pd',
     '<stage_instructions stage="P4" name="fix loop" of="{len(STAGES)}">',
     '<stage_instructions stage="P4" name="fix loop" of="7">',
     (T_DRIVERS,), KILLED),
    # The GUI half of the same defect: P-brief is the one id that is neither
    # P<digit> nor P-close, so a hand tuple and this pattern skipped it alike
    # and the GUI showed "working..." for the whole stage.
    ('stage-re-drops-p-brief', 'pr',
     '(P-brief|P-close|P[0-6]|L[1-5])',
     '(P-close|P[0-6]|L[1-5])',
     (T_RUN,), KILLED),

    # ---- C3: every Next: line reaches its stage -----------------------------
    # A handoff that names a stage without the flags that stage requires. P4
    # refuses without --render-json, so the reader following it gets exit 4.
    ('next-underspecified', 'pd',
     "      --before {a.board} --render-json <the adopted board's render>",
     "      --before {a.board}",
     (T_DRIVERS,), KILLED),
    # ...and a handoff naming a wk/ file no stage tells the reader to produce.
    # Reaching the stage is not enough if the recipe is missing, and the
    # flag-set arm cannot see this by construction.
    ('next-artifact-orphan', 'pd',
     "      --before {a.board} --render-json <the seed's render>",
     '      --before {a.board} --render-json wk/render_seed.json',
     (T_DRIVERS,), KILLED),

    # ---- C4: the evidence map attributes its rows ---------------------------
    # Attribution is by TOOL NAME ON THE HEADING LINE. Drop it and ~15 rows
    # stop being checked while the page still looks right.
    ('evidence-map-heading-untooled', 'em',
     '## E2. `check_floorplan.py BOARD --intent ... --json wk/floorplan.json`',
     '## E2. The --json PATH document',
     (T_923,), KILLED),

    # ---- D1: a null `blocking` is reported, not raised ----------------------
    # Restores the unguarded `blocking = key[0]` by disabling the guard.
    ('verdict-unguarded', 'cv',
     '    if key is None:\n',
     '    if False:\n',
     (T_CONVERGE,), KILLED),
    # The subtler half: asserting a cause the score never named. `unknown` is
    # the only evidence that a component RAN and could not answer.
    ('verdict-cause-unmeasured', 'cv',
     "        elif isinstance(score.get('unknown'), (list, tuple, set)) \\\n",
     "        elif score.get('unknown') is not None \\\n",
     (T_CONVERGE,), KILLED),

    # ---- the registration holes, and what they were hiding ------------------
    # `--no-ratsnest` is real and composed from an f-string, so no literal
    # exists for the text scan to find. Blinding the resolver must make the
    # placement skill's citation read as dead.
    ('composed-flags-blinded', 'dfl',
     '            out |= _composed_flags(text)\n',
     '            out |= set()\n',
     (T_DFL,), KILLED),

    # ---- B6: a gate must not pin the citation it exists to keep correct -----
    # `def better` moved from 358 to 564. The gate hardcoded 358, so correcting
    # the skill FAILED the test whose job is keeping the skill correct.
    ('better-line-pinned', 't431',
     "    assert 'better()' in skill and f'place_route_loop.py:{_better}' in skill, \\\n",
     "    assert 'better()' in skill and 'place_route_loop.py:358' in skill, \\\n",
     (T_431,), KILLED),
]


# The shared pre-flight (#877): refuses in ONE SECOND on an anchor that matches
# anything other than exactly once, instead of reporting BROKEN forty minutes
# later.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_anchors import preflight                       # noqa: E402
preflight(__file__)


def _dirty():
    r = subprocess.run(['git', 'status', '--porcelain'] +
                       sorted(set(TARGETS.values())),
                       cwd=REPO, capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def run_row(row, keep=False):
    name, target, old, new, tests, _expect = row
    path = TARGETS[target]
    # newline='' on BOTH sides: read with universal newlines and write with
    # none and every CRLF file comes back LF, so a row that RESTORED its target
    # still left the tree dirty.
    with open(path, encoding='utf-8', newline='') as fh:
        original = fh.read()
    if '\r\n' in original:
        # ...and the anchors are written with '\n', so they are translated to
        # the file's own ending rather than the file being normalised to
        # theirs.
        old = old.replace('\n', '\r\n')
        new = new.replace('\n', '\r\n')
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
