#!/usr/bin/env python3
"""#903 mutation battery: is the ARMING actually covered, or do its tests
merely run?

    python3 tests/mutate_903.py
    python3 tests/mutate_903.py --row the-unaided-stager-never-arms
    python3 tests/mutate_903.py --list

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine and
harness files in place and restores them, and a suite running beside it would
grade a mutated tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and
an ERROR count the same, because a mutation that makes the graders crash is
still a mutation the graders noticed. A row whose anchor does not match
EXACTLY ONCE is BROKEN, not skipped: an anchor that silently matches nothing
reports every mutation as killed and is the most flattering possible bug.

WHY THIS BATTERY MATTERS PARTICULARLY HERE. #903's whole content is a CALL
THAT WAS NOT MADE. The instrument it arms was complete, correct and tested
before this change -- `tests/test_provenance_audit.py` was 47 green rows
against an arming that no production path ever performed. So "the tests pass"
was true of the defect, and every row below is a way of putting the defect
back that a reader could mistake for a tidy-up: dropping a call, arming the
wrong dir, hashing the wrong file, reverting one tuple entry.

The first row is the tree as it stood before this PR, exactly.

BYTECODE. Each row rewrites a file and restores it within the same second,
and the registry row is nearly size-preserving -- exactly the (mtime, size)
pair CPython's `.pyc` check treats as unchanged. The runner drops the target's
`__pycache__` and runs every test with `-B`; without that a later row imports
an earlier row's mutant and the results are fiction. `tests/stress/` needs
this as much as `py_placer/` does, because both stagers are imported as
modules by other tests.
"""
import argparse
import os
import shutil
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

SU = os.path.join(ROOT, 'tests', 'stress', 'stage_unaided.py')
SB = os.path.join(ROOT, 'tests', 'stress', 'stage_blind.py')
PV = os.path.join(ROOT, 'py_placer', 'placement', 'provenance.py')
RW = os.path.join(ROOT, 'tests', 'stress', 'run_watch.py')
TARGETS = {'su': SU, 'sb': SB, 'pv': PV, 'rw': RW}

T_903 = os.path.join(TESTS, 'test_903_stagers_arm_the_regime.py')
T_PROV = os.path.join(TESTS, 'test_provenance_audit.py')

#: The cheap gate, run unmutated first. `T_PROV` is the in-process half and
#: `T_903` the subprocess half; a row is only evidence if both are green
#: before anything is rewritten.
BASELINE = (T_903, T_PROV)

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the defect itself, put back --------------------------------------
    # This is `main`'s tree as of the commit before this PR: `stage()` writes
    # the board and never arms. Every downstream symptom follows -- no
    # manifest, no ledger, no refusal, UNPROVEN forever.
    ('the-unaided-stager-never-arms', 'su',
     "    _PV.start_regime(_wd, out_board, mechanical=os.path.abspath(mech),\n"
     "                     restaged_over_rows=_prior)\n",
     "",
     (T_903, T_PROV), 'KILLED'),

    ('the-blind-stager-never-arms', 'sb',
     "    _PV.start_regime(workdir, out)\n",
     "",
     (T_903,), 'KILLED'),

    # ---- armed, but describing the wrong thing ----------------------------
    # The manifest hashes the SOURCE instead of the staged board. Not caught
    # by provenance_audit, which only checks the staged board is READABLE and
    # the source is a perfectly readable file -- so the only thing between
    # this and a manifest that lies (while naming the source path inside the
    # fence) is T_903's two assertions. That is this row's whole job.
    ('the-manifest-describes-the-source', 'su',
     "    _PV.start_regime(_wd, out_board, mechanical=os.path.abspath(mech),",
     "    _PV.start_regime(_wd, src, mechanical=os.path.abspath(mech),",
     (T_903,), 'KILLED'),

    # Armed over the wrong directory: `regime_for` walks UP from the board,
    # so arming the PARENT still governs the work dir and every refusal still
    # fires -- but `provenance_audit --workdir` looks only in the dir it was
    # given and finds nothing. The whole instrument reads UNPROVEN again
    # while every in-process test still passes.
    ('the-regime-is-armed-one-level-up', 'su',
     "    _wd = os.path.dirname(os.path.abspath(out_board))\n",
     "    _wd = os.path.dirname(os.path.dirname(os.path.abspath(out_board)))\n",
     (T_903, T_PROV), 'KILLED'),

    # ---- the registry entry, reverted alone -------------------------------
    # Killed BEHAVIOURALLY, not by a tuple-membership assertion: the restage
    # declares `stage_unaided.py`, `record_write` refuses a lever the tuple
    # does not carry, and the stager is refused by the guard it installed.
    ('the-registry-loses-the-unaided-stager', 'pv',
     "    'perturb.py', 'stage_blind.py', 'stage_unaided.py',\n",
     "    'perturb.py', 'stage_blind.py',\n",
     (T_903, T_PROV), 'KILLED'),

    # ---- the declaration, back in __main__ only ---------------------------
    # The shape this PR found by writing its own test: the CLI still works,
    # so a reviewer sees nothing, and a LIBRARY caller of `stage()` into an
    # armed dir raises "no registered lever" -- the stager refusing itself.
    ('the-stager-declares-only-in-main', 'su',
     "    with (contextlib.nullcontext() if _PV.active_lever() is not None\n"
     "          else _PV.declare_lever('stage_unaided.py')):\n"
     "        write_placed_output(src, out_board, placements)\n",
     "    write_placed_output(src, out_board, placements)\n",
     (T_PROV,), 'KILLED'),

    # The inverse: declare UNCONDITIONALLY. Innermost-wins then replaces the
    # CLI's argv-bearing declaration with an argv-less one, and run_watch's
    # ledger scanner -- which reads `lever_argv` and never `lever` -- goes
    # blind to every staging invocation. Nothing about the board changes.
    ('the-inner-declaration-eats-the-cli-argv', 'su',
     "    with (contextlib.nullcontext() if _PV.active_lever() is not None\n"
     "          else _PV.declare_lever('stage_unaided.py')):\n",
     "    with _PV.declare_lever('stage_unaided.py'):\n",
     (T_903,), 'KILLED'),

    # ---- the disclosure that keeps a restage honest -----------------------
    # Read AFTER the staging write instead of before, so the count includes
    # this call's own row and "restaged over 1 row" becomes true of a dir
    # nothing had restaged. A one-line move that reads as a tidy-up.
    ('restaged-over-rows-counts-its-own-row', 'su',
     "    _prior = len(_PV.read_ledger(_wd)) if os.path.isfile(\n"
     "        os.path.join(_wd, _PV.LEDGER_NAME)) else 0\n",
     "    _prior = 0\n",
     (T_903, T_PROV), 'KILLED'),

    # ---- the watcher gap this PR opened, and closed -----------------------
    ('the-restage-counter-forgets-the-unaided-stager', 'rw',
     "                if any(t.endswith(('stage_blind.py', 'stage_unaided.py'))\n"
     "                       for t in toks):\n",
     "                if any(t.endswith('stage_blind.py') for t in toks):\n",
     (T_903,), 'SURVIVED'),

    # The LEDGER counter is the one that works (neither stager prints a
    # `CMD:` line, so the log counter above can miss the first staging
    # entirely). T_903 asserts on it directly, which is why this row dies and
    # the log-counter row above does not.
    ('the-ledger-restage-counter-names-one-stager', 'rw',
     "                if isinstance(r, dict) and str(r.get('lever') or '') in (\n"
     "                        'stage_unaided.py', 'stage_blind.py'):\n",
     "                if isinstance(r, dict) and str(r.get('lever') or '') in (\n"
     "                        'stage_blind.py',):\n",
     (T_903,), 'KILLED'),

    # ---- declared survivors ----------------------------------------------
    # A REAL hole, recorded rather than dressed up. The nested-regime NOTE is
    # a stderr disclosure with no consumer: nothing downstream reads it, and
    # asserting on it would be asserting on a print. The condition it guards
    # (`regime_for` finding an OUTER dir) is exercised by no committed
    # fixture, because building one means arming a temp dir's PARENT, which
    # then governs every other test running under the same temp root. Left as
    # a disclosure, and named here so "why is this untested" has an answer.
    ('the-nested-regime-note-is-silenced', 'su',
     "    if _outer is not None and os.path.abspath(_outer) != _wd:\n",
     "    if False:\n",
     (T_903, T_PROV), 'SURVIVED'),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten. A stale anchor otherwise reports BROKEN mid-run, after the
# witnesses have been paid for; this is the one second (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


#: `run_all.py`'s self-skip code. A test that exits 77 asserted NOTHING, so
#: reading it as a kill is exactly the "most flattering possible bug" this
#: file's header refuses. It is a THIRD outcome, not a kill.
SKIP_EXIT = 77

#: T_903 prints this when it reached every arm. A gate that ran only its
#: first three checks would still exit 0 on an unmutated tree and would then
#: report the later rows KILLED for free.
COVERAGE = '903 coverage: unaided=yes blind=yes refusal=yes restage=yes'


def run(tests, want_coverage=False):
    """(killed, why) -- killed when ANY named test exits non-zero.

    `why` is `SKIP:<name>` when a test SELF-SKIPPED, which the caller turns
    into BROKEN: a mutation is only shown to be caught if the test that
    caught it actually ran. `want_coverage` additionally requires T_903's
    end-of-run marker on the UNMUTATED pass, so a gate that silently stopped
    covering half its arms is reported rather than believed.
    """
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True, env=env,
                           timeout=3600)
        if r.returncode == SKIP_EXIT:
            return False, 'SKIP:' + os.path.basename(t)
        if r.returncode != 0:
            return True, os.path.basename(t)
        if (want_coverage and os.path.basename(t) == os.path.basename(T_903)
                and COVERAGE not in (r.stdout or '')):
            return True, 'NO-COVERAGE:' + os.path.basename(t)
    return False, ''


def _drop_pycache(path):
    """A size-preserving rewrite inside one second is invisible to CPython.

    The `.pyc` validity check is (source mtime, source size). Rows here change
    a line's content without changing its length, and the whole battery runs
    inside a few seconds -- so a stale `.pyc` would serve the PREVIOUS row's
    mutant to every later import. Both stagers ARE imported as modules by
    other tests, so `tests/stress/__pycache__` matters as much as
    `py_placer/placement/__pycache__`.
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

    # Uncommitted work in the files this rewrites is unrecoverable if the
    # process dies mid-row: a crash between the write and the restore leaves
    # the MUTANT on disk with nothing to put back. `git status --porcelain`
    # rather than `git diff --quiet`, so a STAGED change counts as dirty too.
    dirty = subprocess.run(['git', 'status', '--porcelain', '--']
                           + list(TARGETS.values()), cwd=ROOT,
                           capture_output=True, text=True).stdout.strip()
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes:\n' + dirty + '\nA row that dies between the write and '
              'the restore leaves a MUTANT in your tree. Commit first.')
        return 2

    # The battery is only evidence if the gate passes UNMUTATED first.
    # Without this, a red tree -- or a test that SELF-SKIPS because a fixture
    # is missing -- scores every row KILLED and this file reports full
    # coverage for a gate that asserted nothing.
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
