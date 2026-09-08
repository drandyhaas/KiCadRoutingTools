#!/usr/bin/env python3
"""#892 mutation battery: does anything notice when the pose setter is undone?

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine files
in place and restores them, and a suite running beside it would grade a mutated
tree. One writer per tree -- run this in its own worktree.

A row is KILLED when any named test exits non-zero. A row whose anchor does not
match EXACTLY ONCE is BROKEN, not skipped -- an anchor that matches nothing
reports every mutation as killed, which is the most flattering possible bug.

WHY THIS FILE EXISTS AS A FILE. The first two rounds of this battery ran from a
scratch directory, and a scratch battery is a number nobody can re-run: the
repo's rule is that a measured claim needs a committed measurement. It has also
already earned its keep twice, both times against the TESTS rather than the
tool:

  * neutering the COUNT arm of `worsened()` left every CLI assertion green,
    because the shortfall arm refused the same request. `worsened()` is checked
    arm by arm now.
  * dropping the Euclidean `--radius` bound left every snap assertion green,
    because the case had been loosened to `--radius 8` while the overshoot it
    was written for is 5.0 mm under `--radius 4`.

and a third time against a guard whose deletion changed only the MESSAGE
(`a missing input file is no longer caught`): without the `isfile` check the
parser raises and the run still exits 2 with a summary, so the arm asserts the
reason now.

The measured table lives in the docstring of `tests/test_892_place_pose.py`,
from the run, and is never edited to match a prediction.

    python3 -X utf8 tests/mutate_892.py
    python3 -X utf8 tests/mutate_892.py --list
    python3 -X utf8 tests/mutate_892.py --row face-cycle-reversed
    python3 -X utf8 tests/mutate_892.py --verify-anchors
"""
import argparse
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

OPS = os.path.join(ROOT, 'py_placer', 'placement', 'pose_ops.py')
CLI = os.path.join(ROOT, 'py_placer', 'place_pose.py')
SEEDER = os.path.join(ROOT, 'py_placer', 'placement', 'seeder.py')
PROV = os.path.join(ROOT, 'py_placer', 'placement', 'provenance.py')
PARITY = os.path.join(TESTS, 'gui_parity', 'test_manifest_plan_parity.py')

TARGETS = {'o': OPS, 'c': CLI, 's': SEEDER, 'p': PROV, 'g': PARITY}

T_POSE = os.path.join(TESTS, 'test_892_place_pose.py')
T_REG = os.path.join(TESTS, 'test_892_registries.py')
T_726 = os.path.join(TESTS, 'test_726_writer_resolves_one_block.py')

#: (name, target key, anchor, replacement, tests that must notice, expectation)
ROWS = [
    # ---- the face rule ----------------------------------------------------
    ('face-cycle-reversed', 'o',
     "FACE_CYCLE = ('east', 'north', 'west', 'south')",
     "FACE_CYCLE = ('east', 'south', 'west', 'north')",
     (T_POSE,), 'KILLED'),

    ('the-face-row-is-keyed-by-pad-number-again', 'o',
     "            ix = {id(p): i for i, p in enumerate(fp.pads or ())}",
     "            ix = {id(p): 0 for i, p in enumerate(fp.pads or ())}",
     (T_POSE,), 'KILLED'),

    ('a-face-aim-is-claimed-rather-than-measured', 'o',
     "            if hit * 2 <= len(n['row_pad_ix']):",
     "            if False:",
     (T_POSE,), 'KILLED'),

    # ---- the verdict ------------------------------------------------------
    ('worsened-count-arm-neutered', 'o',
     "    out = [k for k in LEGALITY_KEYS\n"
     "           if (after.get(k) or 0) > (before.get(k) or 0)]",
     "    out = []",
     (T_POSE,), 'KILLED'),

    ('worsened-magnitude-arm-neutered', 'o',
     "    out += [k for k in MAGNITUDE_KEYS\n"
     "            if (after.get(k) or 0.0) > ((before.get(k) or 0.0) "
     "+ MAGNITUDE_EPS)]",
     "    out += []",
     (T_POSE,), 'KILLED'),

    ('off-board-amount-stops-being-an-arm', 'o',
     "MAGNITUDE_KEYS = ('pad_shortfall', 'oob_pad_amount')",
     "MAGNITUDE_KEYS = ('pad_shortfall',)",
     (T_POSE,), 'KILLED'),

    ('is_clean-ignores-the-magnitudes', 'o',
     "    return not (any(report.get(k) for k in LEGALITY_KEYS)",
     "    return True or not (any(report.get(k) for k in LEGALITY_KEYS)",
     (T_POSE,), 'KILLED'),

    ('legal-goes-back-to-meaning-no_worse', 'o',
     "        summary['legal'] = is_clean(after)",
     "        summary['legal'] = not bad",
     (T_POSE,), 'KILLED'),

    ('a-refusal-names-an-output-path-again', 'o',
     "                summary['output'] = None\n"
     "                raise PoseRefusal(summary['refused'], summary=summary)",
     "                raise PoseRefusal(summary['refused'], summary=summary)",
     (T_POSE,), 'KILLED'),

    # The LEGALITY site, not the face one. Mutating the face append survives:
    # the legality block appends after it either way, so both findings still
    # reach the summary. Only the LAST writer can erase what came before -- and
    # erasing it is exactly the defect this row exists for.
    ('a-forced-run-reports-only-the-last-finding', 'o',
     "            findings.append(_refusal_reason(bad, strict, before, after,\n"
     "                                            summary))",
     "            findings[:] = [_refusal_reason(bad, strict, before, after,\n"
     "                                           summary)]",
     (T_POSE,), 'KILLED'),

    # ---- the snap ---------------------------------------------------------
    ('the-snap-ladder-loses-its-lattice-rung', 'o',
     "    ordered = out + lattice",
     "    ordered = out",
     (T_POSE,), 'KILLED'),

    ('the-radius-stops-bounding-the-distance', 'o',
     "    out = [dict(p, rung='ranked') for p in ranked\n"
     "           if (p.get('dist_mm') or 0.0) <= radius + 1e-9]",
     "    out = [dict(p, rung='ranked') for p in ranked]",
     (T_POSE,), 'KILLED'),

    ('the-snapped-pose-is-not-re-staged', 'o',
     "                write_placed_output(board_path, cand, placements)\n"
     "                copy_siblings(board_path, cand)\n"
     "                summary['snapped'] = {",
     "                summary['snapped'] = {",
     (T_POSE,), 'KILLED'),

    # ---- writes and refusals ----------------------------------------------
    ('dry-run-writes-the-board-anyway', 'o',
     "        if dry_run:\n"
     "            # The lock keys are already in the summary",
     "        if False:\n"
     "            # The lock keys are already in the summary",
     (T_POSE,), 'KILLED'),

    ('the-lock-guard-is-skipped', 'o',
     "    if placements:\n"
     "        from placement.parser import extract_locked_refs",
     "    if False:\n"
     "        from placement.parser import extract_locked_refs",
     (T_POSE,), 'KILLED'),

    ('lock-and-unlock-of-one-ref-is-allowed-again', 'o',
     "    both = sorted(set(lock_refs) & set(unlock_refs))",
     "    both = []",
     (T_POSE,), 'KILLED'),

    ('an-unknown-lock-ref-is-accepted-again', 'o',
     "    missing = sorted((set(lock_refs) | set(unlock_refs)) - known)",
     "    missing = []",
     (T_POSE,), 'KILLED'),

    ('a-failed-promote-is-not-atomic-again', 'o',
     "        for src, dst in pairs:\n"
     "            tmp = dst + '.krt-tmp'",
     "        for src, dst in pairs:\n"
     "            tmp = dst",
     (T_POSE,), 'KILLED'),

    ('a-forced-run-is-not-disclosed', 'o',
     "            summary['forced'] = True\n\n        if dry_run:",
     "            summary['forced'] = False\n\n        if dry_run:",
     (T_POSE,), 'KILLED'),

    ('only-the-first-op-is-written', 'o',
     "        if placements:\n"
     "            write_placed_output(board_path, cand, placements)\n"
     "        else:",
     "        if placements:\n"
     "            write_placed_output(board_path, cand, placements[:1])\n"
     "        else:",
     (T_POSE,), 'KILLED'),

    # ---- the CLI ----------------------------------------------------------
    ('the-copper-gate-stops-refusing', 'c',
     "    if st.has_copper and not args.allow_routed:",
     "    if False:",
     (T_POSE,), 'KILLED'),

    ('a-missing-input-file-is-no-longer-named', 'c',
     "        return _refuse(args, \"%s is not a file\" % args.input_file, 2)",
     "        return _refuse(args, \"cannot read it\", 2)",
     (T_POSE,), 'KILLED'),

    ('the-snap-knobs-are-unvalidated-again', 'c',
     "    if args.snap_step <= 0:",
     "    if False:",
     (T_POSE,), 'KILLED'),

    # ---- the lock stamper -------------------------------------------------
    ('stamp_unlocked-removes-nothing', 's',
     "        new_head, n = re.subn(r'\\s*\\(locked\\s+yes\\)', '', head)",
     "        new_head, n = head, 0",
     (T_POSE,), 'KILLED'),

    ('stamp_unlocked-unlocks-every-namesake', 's',
     "        if key not in want:\n            continue\n        head_end",
     "        if _raw_ref not in want:\n            continue\n        head_end",
     (T_726,), 'KILLED'),

    # ---- the registries ---------------------------------------------------
    # Anchored on the comment ABOVE the entry as well as the entry: the bare
    # `'place_pose.py',` line also appears in `krt_capabilities.py` and in the
    # parity gate, and an anchor that matches elsewhere is how a battery ends
    # up mutating prose and reporting a test hole that is not there.
    ('place_pose-leaves-the-lever-registry', 'p',
     "    # this regime is built to refuse (run 25's `pose_assist.py`).\n"
     "    'place_pose.py',",
     "    # this regime is built to refuse (run 25's `pose_assist.py`).",
     (T_REG,), 'KILLED'),

    ('the-parity-gate-goes-back-to-a-hand-picked-list', 'g',
     "    for tool in sorted(m2p.REFUSED_TOOLS):",
     "    for tool in ('place_optimize.py',):",
     (T_REG,), 'KILLED'),
]


def _uncache(path):
    """Delete the target's cached bytecode. MEASURED HAZARD, not hygiene.

    CPython validates a `.pyc` on (source mtime SECONDS, source SIZE), and
    several rows here are single-token edits, so a mutated and a restored file
    are the SAME SIZE. Mutate, run and restore inside one second and the `.pyc`
    compiled from the MUTATED source stays valid for every later import.
    """
    import importlib
    import importlib.util
    try:
        cached = importlib.util.cache_from_source(path)
        if os.path.exists(cached):
            os.remove(cached)
    except (OSError, ValueError, NotImplementedError):
        pass
    importlib.invalidate_caches()


def _write(path, text):
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    _uncache(path)


def run(tests):
    env = dict(os.environ)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['PYTHONHASHSEED'] = '0'
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True, env=env)
        if r.returncode != 0:
            return True, os.path.basename(t)
    return False, ''


def verify_anchors():
    """Every anchor must match its target exactly once AND match no OTHER
    tracked file -- the prose-anchor trap, checked rather than asserted."""
    bad = 0
    src = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    tracked = subprocess.run(['git', 'ls-files', '*.py', '*.md'], cwd=ROOT,
                             capture_output=True, text=True).stdout.split()
    for name, tgt, old, _new, _tests, _exp in ROWS:
        n = src[tgt].count(old)
        others = []
        for rel in tracked:
            abs_rel = os.path.abspath(os.path.join(ROOT, rel))
            if abs_rel in (TARGETS[tgt], os.path.abspath(__file__)):
                continue
            try:
                with open(os.path.join(ROOT, rel), encoding='utf-8',
                          errors='replace') as fh:
                    if old in fh.read():
                        others.append(rel)
            except OSError:
                pass
        flag = '' if (n == 1 and not others) else '   <-- PROBLEM'
        if flag:
            bad += 1
        print('  %-52s %dx in target, %d other file(s)%s'
              % (name, n, len(others), flag))
        for o in others[:3]:
            print('        also in %s' % o)
    print('\n%d row(s), %d problem(s)' % (len(ROWS), bad))
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--verify-anchors', action='store_true')
    a = ap.parse_args()
    if a.list:
        for n, t, _o, _w, _tests, exp in ROWS:
            print('  %-52s %-18s %s'
                  % (n, os.path.basename(TARGETS[t]), exp))
        return 0
    if a.verify_anchors:
        return verify_anchors()

    # A dirty engine tree would be RESTORED to its committed text, silently
    # destroying uncommitted work. Refuse rather than help.
    dirty = subprocess.run(['git', 'diff', '--quiet', '--']
                           + list(TARGETS.values()), cwd=ROOT).returncode
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes.\nRestoring them would write the COMMITTED text back '
              'over your work. Commit first.')
        return 2

    rows = [r for r in ROWS if not a.row or r[0] == a.row]
    if not rows:
        print('no row named %r' % a.row)
        return 2
    originals = {k: open(v, encoding='utf-8').read()
                 for k, v in TARGETS.items()}
    # The BASELINE, run once: a target test that is red before any mutation
    # scores every row KILLED and the battery exits 0 on a lie.
    died, by = run(sorted({t for r in rows for t in r[4]}))
    if died:
        print('REFUSED: %s is already failing on the UNMUTATED tree, so every '
              'row below would report KILLED for the wrong reason.' % by)
        return 2

    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print('  %-52s BROKEN (anchor matched %dx)'
                      % (name, src.count(old)))
                broken += 1
                continue
            _write(TARGETS[tgt], src.replace(old, new))
            try:
                died, by = run(tests)
            finally:
                _write(TARGETS[tgt], src)
            got = 'KILLED' if died else 'SURVIVED'
            mark = '' if got == exp else '   *** DISAGREES with ' + exp
            if got != exp:
                disagree += 1
            killed += died
            survived += not died
            print('  %-52s %-9s %s%s' % (name, got, by, mark))
    finally:
        for k, v in TARGETS.items():
            _write(v, originals[k])
    print('\n%d row(s): %d killed, %d survived, %d broken, %d disagreeing '
          'with expectation' % (len(rows), killed, survived, broken, disagree))
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
