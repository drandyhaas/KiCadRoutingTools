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

Two rows mutate a GATE rather than the thing it guards
(`composed-flags-blinded`, `exit3-scanner-blinded`). That is deliberate: those
gates are the only reason the corresponding claims cannot drift, so a battery
that never breaks them would be reporting on a claim nobody tested.

Retired with the staged placement skills (pcb-free-agent replaced them): the
driver's stage-count rows, its `Next:` hand-off rows, its off-outline gate
row, the GUI stage-pattern row, the evidence-map heading row and B6's
`better-line-pinned` (whose assertion lived in a test_431 check of the
retired skill's text).
"""
import argparse
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

KILLED, SURVIVED, BROKEN = 'KILLED', 'SURVIVED', 'BROKEN'

TARGETS = {
    'bs': os.path.join(REPO, 'py_tools', 'board_score.py'),
    'bst': os.path.join(REPO, 'py_placer', 'board_store.py'),
    'cv': os.path.join(REPO, 'py_placer', 'converge.py'),
    # #1088: blocking_defect / blocking_value live here now, imported by
    # converge, check_complete and the film alike
    'ls': os.path.join(REPO, 'py_router', 'ledger_score.py'),
    'dfl': os.path.join(REPO, 'tests', 'test_doc_flag_liveness.py'),
    't431': os.path.join(REPO, 'tests', 'test_431_skill_commands.py'),
    'prun': os.path.join(REPO, 'kicad_routing_plugin', 'placement_run.py'),
    'rw': os.path.join(REPO, 'tests', 'stress', 'run_watch.py'),
    'cc': os.path.join(REPO, 'check_complete.py'),
}

T_WORKLIST = 'tests/test_broken_worklist.py'
T_CONVERGE = 'tests/test_converge.py'
T_DFL = 'tests/test_doc_flag_liveness.py'
T_431 = 'tests/test_431_skill_commands.py'
T_PRUN = 'tests/test_placement_run.py'
T_RW = 'tests/test_run20_run_watch.py'
T_CC = 'tests/test_run9_check_complete.py'

#: (name, target, old, new, tests that must notice, expectation)
ROWS = [
    # ---- C1: the handler names a tool that exists ---------------------------
    # #1112 made every break route.py's; the mutation restores a poured-net
    # branch naming the dead tool.
    ('handler-returns', 'bs',
     "        v['handler'] = 'route'",
     "        v['handler'] = ('route_disconnected_planes' if name in poured "
     "else 'route')",
     (T_WORKLIST,), KILLED),

    # ---- C2, C3, C4: RETIRED -------------------------------------------------
    # The stage registry and `of=` count, the `Next:` hand-offs, the GUI's
    # stage pattern and the evidence map's heading attribution were all
    # properties of the staged placement driver, its skill pages and the tab's
    # stage parser. Those left the tree when the staged skills were retired
    # for pcb-free-agent (the tab now reads progress from ledger rows), and so
    # did these six rows and their killers.

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

    # ---- #1071 #1075 #1076 #1078: `blocking` is a count, or unmeasured ------
    # The #1071 crash itself: a per-term dict ranked, and the plateau test
    # compared two dicts.
    ('score-key-ranks-any-json', 'cv',
     "    b = blocking_value(score.get('blocking'))\n",
     "    b = score.get('blocking')\n",
     (T_CONVERGE,), KILLED),
    # One row per clause of the rule: each lets one kind of non-count rank.
    ('blocking-bool-is-a-count', 'ls',
     "    if isinstance(b, bool):\n        return (f'the boolean",
     "    if False:\n        return (f'the boolean",
     (T_CONVERGE,), KILLED),
    ('blocking-nonfinite-is-a-count', 'ls',
     "    if isinstance(b, float) and not math.isfinite(b):\n",
     "    if False:\n",
     (T_CONVERGE,), KILLED),
    ('blocking-past-float-is-a-count', 'ls',
     "    if b > sys.float_info.max:\n        return (f'an integer",
     "    if False:\n        return (f'an integer",
     (T_CONVERGE,), KILLED),
    # `isfinite` converts an int to float: a 400-digit count raised.
    ('isfinite-on-an-int-again', 'ls',
     "    if isinstance(b, float) and not math.isfinite(b):\n",
     "    if not math.isfinite(b):\n",
     (T_CONVERGE,), KILLED),
    ('blocking-negative-is-a-count', 'ls',
     "    if b < 0:\n        return f'negative",
     "    if False:\n        return f'negative",
     (T_CONVERGE,), KILLED),
    # The append-only ledger's only door.
    ('record-accepts-a-non-count-blocking', 'cv',
     "    if _bad_blocking:\n",
     "    if False:\n",
     (T_CONVERGE,), KILLED),
    ('record-accepts-a-non-object-score', 'cv',
     "    if _score_doc is not None and not isinstance(_score_doc, dict):\n",
     "    if False:\n",
     (T_CONVERGE,), KILLED),
    # `false` as a --score: NO-SCORE must say what it is, never "null".
    ('verdict-calls-a-non-count-null', 'cv',
     "        elif blocking_defect(score.get('blocking')):\n",
     "        elif False:\n",
     (T_CONVERGE,), KILLED),
    # TWO lines: the first alone is also a substring of `_no_score`'s deeper
    # indented copy; the newline + 11 spaces before `'unknown'` is not.
    ('verdict-ungraded-unguarded-again', 'cv',
     "           'ungraded': _names('ungraded'),\n           'unknown'",
     "           'ungraded': sorted(score.get('ungraded') or []),\n"
     "           'unknown'",
     (T_CONVERGE,), KILLED),
    ('verdict-unknown-unguarded-again', 'cv',
     "           'unknown': _names('unknown'),\n",
     "           'unknown': sorted(score.get('unknown') or []),\n",
     (T_CONVERGE,), KILLED),
    ('quality-ranks-a-bool-or-nan', 'cv',
     "                    and not isinstance(v, bool)\n"
     "                    and (isinstance(v, int) or math.isfinite(v))\n",
     "                    and True\n",
     (T_CONVERGE,), KILLED),
    ('quality-isfinite-on-an-int-again', 'cv',
     "                    and (isinstance(v, int) or math.isfinite(v))\n",
     "                    and math.isfinite(v)\n",
     (T_CONVERGE,), KILLED),
    ('unknown-not-a-list-claims-a-component-ran', 'cv',
     "    if doc['unknown'] and isinstance(score.get('unknown'),\n",
     "    if doc['unknown'] and (score.get('unknown'),\n",
     (T_CONVERGE,), KILLED),
    ('ungraded-not-a-list-reads-unexamined', 'cv',
     "    if doc['ungraded'] and not isinstance(score.get('ungraded'),\n",
     "    if False and not isinstance(score.get('ungraded'),\n",
     (T_CONVERGE,), KILLED),
    # The GUI's result document: `isinstance(True, int)` holds.
    ('placement-result-takes-a-bool', 'prun',
     "    if blocking is not None and (isinstance(blocking, bool)\n",
     "    if blocking is not None and (False\n",
     (T_PRUN,), KILLED),
    ('placement-result-takes-a-negative', 'prun',
     "                                 or blocking < 0):\n",
     "                                 or False):\n",
     (T_PRUN,), KILLED),
    ('ledger-keeps-a-non-object-line', 'bst',
     "                    if isinstance(doc, dict):\n",
     "                    if True:\n",
     (T_CONVERGE,), KILLED),
    # #1078's leftover: numbering by the count repeats an iteration after a
    # skipped line.
    ('record-numbers-by-count', 'cv',
     "    entry = {'iteration': lg.next_iteration(), 'kind': a.kind,\n",
     "    entry = {'iteration': len(lg.entries()), 'kind': a.kind,\n",
     (T_CONVERGE,), KILLED),
    ('next-iteration-is-the-count', 'bst',
     "        return max([len(rows)] + [i + 1 for i in used\n",
     "        return max([len(rows)] + [0 for i in used\n",
     (T_CONVERGE,), KILLED),
    # The two readers outside the rule (#1071).
    # The old test, which let `false` and NaN through as counts. (Not `blk =
    # raw`: that dies comparing a dict, killed by a traceback, not a witness.)
    ('watcher-ranks-a-non-count', 'rw',
     "        blk = None if defect else raw\n",
     "        blk = raw if isinstance(raw, (int, float)) else None\n",
     (T_RW,), KILLED),
    ('close-out-takes-false-as-done', 'cc',
     "    if _bdef:\n",
     "    if False:\n",
     (T_CC,), KILLED),
    ('close-out-splits-a-string-ungraded', 'cc',
     "        return sorted(str(x) for x in v), None\n",
     "        return sorted(str(x) for x in v), None\n"
     "    if isinstance(v, str):\n"
     "        return sorted(v), None\n",
     (T_CC,), KILLED),

    # ---- the registration holes, and what they were hiding ------------------
    # `--no-ratsnest` is real and composed from an f-string, so no literal
    # exists for the text scan to find. The placement skill that cited it was
    # retired, so the gate's own positive control is what must go red now.
    ('composed-flags-blinded', 'dfl',
     '            out |= _composed_flags(text)\n',
     '            out |= set()\n',
     (T_DFL,), KILLED),

    # ---- test_431's exit-code scanner, its only live half -------------------
    # The corpus has 0 annotated commands, so a scanner that stopped matching
    # reports the same 0. The positive control is what tells those apart.
    ('exit3-scanner-blinded', 't431',
     "            blk = '\\n'.join(ls[i + 1:i + 4])\n",
     "            blk = ''\n",
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
