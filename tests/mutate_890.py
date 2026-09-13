#!/usr/bin/env python3
"""#890 mutation battery: is the DELEGATION DECISION actually covered?

    python3 tests/mutate_890.py
    python3 tests/mutate_890.py --row l1-agent-back-to-claude
    python3 tests/mutate_890.py --list

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES the driver
in place and restores it, and a suite running beside it would grade a mutated
tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and
an ERROR count the same. A row whose anchor does not match EXACTLY ONCE is
BROKEN, not skipped: an anchor that silently matches nothing reports every
mutation as killed and is the most flattering possible bug.

WHAT MAKES THIS ONE AWKWARD, and why the rows are shaped as they are. #890's
product is TEXT -- a tag in an emitted prompt. Text is trivially assertable
and therefore trivially over-assertable: a row that changes `fork` to `claude`
is caught by any `'fork' in out`, which proves nothing about whether the
decision is the driver's. So the rows attack the DECISION instead: the flag
ignored, the verifier forked, the hand-off not written, the archive holding
the wrong document.

The verifier row is the one worth reading. #890 asked for all three
delegations to be forked; L5 is the one that must not be, because its prompt
ends "Re-derive every number yourself. Do not trust the report" and a fork is
handed the parent's whole transcript including that report. That refusal is a
decision this PR took AGAINST its own issue, so it gets a row.
"""
import argparse
import os
import shutil
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

D = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing',
                 'scripts', 'loop_driver.py')
TARGETS = {'d': D}

T_890 = os.path.join(TESTS, 'test_890_delegation_handoff.py')
T_RUN8 = os.path.join(TESTS, 'test_run8_skill_drivers.py')

#: The driver's own `--self-test`, which carries the runtime L5 assertion that
#: needs the terminal close-out fixture. Run as a gate like any test file.
SELFTEST = os.path.join(TESTS, 'driver_self_test_890.py')

BASELINE = (T_890, SELFTEST)

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the defect, at the site the issue names --------------------------
    # Reverts to the tag as it stood before this PR. Every delegated half then
    # starts with an empty context again.
    ('l1-agent-back-to-claude', 'd',
     '<subagent_prompt agent="{_ag}" description="place '
     '{os.path.basename(a.board)}">',
     '<subagent_prompt agent="claude" description="place '
     '{os.path.basename(a.board)}">',
     (T_890, SELFTEST), 'KILLED'),

    ('l2-agent-back-to-claude', 'd',
     '<subagent_prompt agent="{_ag}" description="route '
     '{os.path.basename(a.board)}">',
     '<subagent_prompt agent="claude" description="route '
     '{os.path.basename(a.board)}">',
     (T_890,), 'KILLED'),

    # The sharper half of the same thing: the flag still parses, the choice is
    # still printed, and every mode emits a fork. A revert nobody would
    # question in review, and `'fork' in out` does not notice it.
    ('the-delegate-mode-flag-is-ignored', 'd',
     "    return 'claude' if getattr(a, 'delegate_mode', 'fork') == 'fresh' "
     "else 'fork'\n",
     "    return 'fork'\n",
     (T_890, SELFTEST), 'KILLED'),

    # ---- the decision this PR took AGAINST its own issue ------------------
    # L5 takes the flag too, so the end-to-end verifier inherits the
    # conclusions it exists to contradict.
    ('the-verifier-becomes-a-fork', 'd',
     '<subagent_prompt agent="claude" description="verify the finished '
     'board">',
     '<subagent_prompt agent="{_agent(a)}" description="verify the finished '
     'board">',
     (T_890, SELFTEST), 'KILLED'),

    # ---- the hand-off ------------------------------------------------------
    # Deleted in the shape a "the log already archives this" tidy-up would
    # take. It does archive the whole STAGE; this file is the PROMPT.
    ('the-prompt-file-is-never-written', 'd',
     "    _write_prompt(a, a.stage, out)\n",
     "",
     (T_890,), 'KILLED'),

    # The file exists and is a plausible artifact, but holds the whole stage
    # rather than the hand-off -- so a diff against `<half>_return.md`
    # compares the wrong two documents.
    ('the-prompt-file-holds-the-whole-stage', 'd',
     "    name = _PROMPT_FILE.get(stage)\n    body = _prompt_body(out)\n",
     "    name = _PROMPT_FILE.get(stage)\n    body = out\n",
     (T_890,), 'KILLED'),

    # The inline arm must stay byte-clean: no tag, no body, no file. Without
    # the emptiness test a `--no-delegate` run would write a stage archive
    # under a name that promises a prompt.
    ('the-inline-arm-writes-a-prompt-file-too', 'd',
     "    if not name or not body:\n        return None\n",
     "    if not name:\n        return None\n",
     (T_890,), 'KILLED'),

    # A work dir must not appear because someone asked for TEXT. Same rule as
    # `_log_invocation`, which returns early with a stderr NOTE.
    ('the-prompt-writer-invents-the-ledger-dir', 'd',
     "        if not os.path.isdir(d):\n"
     "            print(f'loop_driver NOTE: no hand-off prompt written",
     "        if not os.path.isdir(d):\n"
     "            os.makedirs(d, exist_ok=True)\n"
     "            print(f'loop_driver NOTE: no hand-off prompt written",
     (T_890,), 'KILLED'),

    # ---- the context block -------------------------------------------------
    # Names every artifact whether or not it exists, so the half is sent
    # looking for files that are not there -- the behaviour FENCE_CLAUSE
    # exists to prevent.
    ('the-context-block-names-absent-artifacts', 'd',
     "        if path and os.path.isfile(path):\n",
     "        if path:\n",
     (T_890,), 'KILLED'),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten. A stale anchor otherwise reports BROKEN mid-run, after the
# witnesses have been paid for; this is the one second (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


SKIP_EXIT = 77

#: T_890's end-of-run marker. Same purpose and same shape as mutate_903's.
COVERAGE = '890 coverage:'


def run(tests, want_coverage=False):
    """(killed, why) -- killed when ANY named test exits non-zero."""
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True, env=env,
                           timeout=3600)
        out = r.stdout or ''
        if r.returncode == SKIP_EXIT:
            return False, 'SKIP:' + os.path.basename(t)
        # Two skip shapes, as in mutate_903: exit 77 is `run_all`'s contract,
        # and a `SKIP:` line with exit 0 is what the older files do.
        if any(ln.startswith('SKIP:') for ln in out.splitlines()):
            return False, 'SKIP:' + os.path.basename(t)
        if r.returncode != 0:
            return True, os.path.basename(t)
        if (want_coverage and os.path.basename(t) == os.path.basename(T_890)
                and COVERAGE not in out):
            return True, 'NO-COVERAGE:' + os.path.basename(t)
    return False, ''


def _drop_pycache(path):
    """A size-preserving rewrite inside one second is invisible to CPython.

    Several rows here swap one identifier for another of a similar length, and
    the driver is IMPORTED as a module by `test_890_delegation_handoff.py`.
    """
    cache = os.path.join(os.path.dirname(path), '__pycache__')
    if os.path.isdir(cache):
        shutil.rmtree(cache, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for n, t, _o, _w, tests, exp in ROWS:
            print(f'  {n:46s} {os.path.basename(TARGETS[t]):20s} {exp}')
        return 0

    rows = [r for r in ROWS if not a.row or r[0] == a.row]
    if not rows:
        print(f'no row named {a.row!r}')
        return 2

    dirty = subprocess.run(['git', 'status', '--porcelain', '--']
                           + list(TARGETS.values()), cwd=ROOT,
                           capture_output=True, text=True).stdout.strip()
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes:\n' + dirty + '\nA row that dies between the write and '
              'the restore leaves a MUTANT in your tree. Commit first.')
        return 2

    every = tuple(dict.fromkeys(t for r in rows for t in r[4]))
    killed0, why0 = run(every, want_coverage=True)
    if killed0 or why0.startswith('SKIP:'):
        print('BROKEN: the gate does not pass on the UNMUTATED tree ({}). '
              'Every row would score KILLED against it, so nothing here would '
              'be evidence. Fix the tree first.'.format(why0 or 'unknown'))
        return 2
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print(f'  {name:46s} BROKEN (anchor matched {src.count(old)}x)')
                broken += 1
                continue
            with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new, 1))
            _drop_pycache(TARGETS[tgt])
            try:
                died, by = run(tests)
            finally:
                with open(TARGETS[tgt], 'w', encoding='utf-8',
                          newline='') as fh:
                    fh.write(src)
                _drop_pycache(TARGETS[tgt])
            if by.startswith('SKIP:'):
                print(f'  {name:46s} BROKEN ({by} -- it asserted nothing)')
                broken += 1
                continue
            got = 'KILLED' if died else 'SURVIVED'
            mark = '' if got == exp else '   *** DISAGREES with ' + exp
            if got != exp:
                disagree += 1
            killed += died
            survived += not died
            print(f'  {name:46s} {got:9s} {by}{mark}')
    finally:
        for k, v in TARGETS.items():
            with open(v, 'w', encoding='utf-8', newline='') as fh:
                fh.write(originals[k])
            _drop_pycache(v)
    print(f'\n{len(rows)} row(s): {killed} killed, {survived} survived, '
          f'{broken} broken, {disagree} disagreeing with expectation')
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
