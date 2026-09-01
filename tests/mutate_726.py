#!/usr/bin/env python3
"""#726 mutation battery: does anything actually notice when the fix is undone?

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine files
in place and restores them, and a suite running beside it would grade a mutated
tree. One writer per tree -- run this in its own worktree.

A row is KILLED when any named test exits non-zero. A failed assertion and an
ERROR count the same: a mutation that makes a grader crash is still a mutation
the graders noticed. A row whose anchor does not match EXACTLY ONCE is BROKEN,
not skipped -- an anchor that silently matches nothing reports every mutation
as killed and is the most flattering possible bug.

Expected SURVIVORS are declared with the reason they are not a test hole.

Two traps this battery is built against, both measured in this repo before:

  * **Stale bytecode.** CPython validates a `.pyc` on (source mtime SECONDS,
    source SIZE), and several rows below are single-token edits, so the mutated
    and restored files are the SAME SIZE. Mutate, run and restore inside one
    second and the `.pyc` compiled from the MUTATED source stays valid for
    every later import. `_uncache` (lifted from `mutate_703.py`) is called
    after BOTH writes, and children run with `PYTHONDONTWRITEBYTECODE=1`.

  * **Prose anchors.** `placement/perturb.py` and `placement/placement_state.py`
    describe this writer's behaviour in PROSE, and `tests/stress/stage_blind.py`
    quotes its output line. An anchor that also matches a comment mutates the
    comment and reports SURVIVED, which reads as a test hole when the tests are
    fine. Every anchor below is code with punctuation a comment cannot carry,
    and the exactly-once check is repo-wide-verifiable with `--verify-anchors`.

The measured table lives in the docstring of
`tests/test_726_writer_resolves_one_block.py`, from the run, and is never
edited to match a prediction.

    python3 -X utf8 tests/mutate_726.py
    python3 -X utf8 tests/mutate_726.py --list
    python3 -X utf8 tests/mutate_726.py --row writer-matches-by-name-again
    python3 -X utf8 tests/mutate_726.py --verify-anchors
"""
import argparse
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

PARSER = os.path.join(ROOT, 'py_router', 'kicad_parser.py')
WRITER = os.path.join(ROOT, 'py_placer', 'placement', 'writer.py')
PPARSER = os.path.join(ROOT, 'py_placer', 'placement', 'parser.py')
SEEDER = os.path.join(ROOT, 'py_placer', 'placement', 'seeder.py')
ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')
GUIUTIL = os.path.join(ROOT, 'kicad_routing_plugin', 'gui_utils.py')

TARGETS = {'p': PARSER, 'w': WRITER, 'g': PPARSER, 's': SEEDER,
           'a': ASSEMBLY, 'u': GUIUTIL}

T_KEYS = os.path.join(TESTS, 'test_726_duplicate_reference_keys.py')
T_WRITER = os.path.join(TESTS, 'test_726_writer_resolves_one_block.py')
T_CONS = os.path.join(TESTS, 'test_726_consumers_see_both_blocks.py')
T_PARITY = os.path.join(TESTS, 'test_parser_pcbnew_parity.py')
T_GUI = os.path.join(TESTS, 'gui_parity', 'test_726_parse_path_parity.py')
T_SYNC = os.path.join(TESTS, 'gui_parity', 'test_726_gui_sync.py')

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the parser: both paths, and the shape of the key ------------------
    ('text-path-reverts-to-last-wins', 'p',
     "    for start, end, fp_text, _raw_reference, _block_key in _blocks:",
     "    for start, end, fp_text, _raw_reference, _block_key in _blocks:\n"
     "        _block_key = _raw_reference",
     (T_KEYS, T_PARITY), 'KILLED'),

    ('pcbnew-path-reverts-to-last-wins', 'p',
     "    _live_keys = disambiguate_references(_raw_refs)",
     "    _live_keys = list(_raw_refs)",
     (T_GUI,), 'KILLED'),

    # Mutates ONLY the pcbnew path while the text path stays fixed. If nothing
    # goes red here, the parity gate is not doing its job -- a one-sided fix is
    # exactly what it exists to catch.
    ('only-the-text-path-disambiguates', 'p',
     "    _dups_live = duplicate_reference_counts(_raw_refs)",
     "    _dups_live = duplicate_reference_counts(_raw_refs)\n"
     "    _raw_refs = ['%s#%d' % (r, i) for i, r in enumerate(_raw_refs)]",
     (T_GUI,), 'KILLED'),

    ('the-ordinal-starts-at-the-wrong-block', 'p',
     "        if n == 1:\n            out.append(ref)\n            issued.add(ref)\n            continue",
     "        if n == 0:\n            out.append(ref)\n            issued.add(ref)\n            continue",
     (T_KEYS, T_PARITY), 'KILLED'),

    ('duplicate-references-counts-extras-not-occurrences', 'p',
     "    return {r: c for r, c in counts.items() if c > 1}",
     "    return {r: c - 1 for r, c in counts.items() if c > 1}",
     (T_KEYS, T_CONS), 'KILLED'),

    ('the-key-is-uuid-sorted-not-file-ordered', 'p',
     "    for (start, end, text), r, key in zip(spans, raw,\n"
     "                                          disambiguate_references(raw)):",
     "    for (start, end, text), r, key in zip(spans, sorted(raw),\n"
     "                                          disambiguate_references(sorted(raw))):",
     (T_KEYS, T_PARITY), 'KILLED'),

    # Anchored on the DUP_REF_SEP line above it: the bare `while` line also
    # appears in `measure_726_duplicate_census.py`'s standalone copy of the
    # scheme, and an anchor that reaches two files is an anchor that will one
    # day mutate the wrong one.
    ('the-ordinal-can-shadow-a-real-name', 'p',
     "        cand = '%s%s%d' % (ref, DUP_REF_SEP, n)\n"
     "        while cand in taken or cand in issued:",
     "        cand = '%s%s%d' % (ref, DUP_REF_SEP, n)\n"
     "        while cand in issued:",
     (T_KEYS,), 'KILLED'),

    # `file=sys.stderr)` on its own is in 31 tracked files. Anchored on the
    # argument line above it, which is unique to this warning.
    ('the-warning-goes-to-stdout', 'p',
     "                 DUP_REF_SEP, DUP_REF_SEP, len(footprints)),\n"
     "              file=sys.stderr)",
     "                 DUP_REF_SEP, DUP_REF_SEP, len(footprints)))\n"
     "        _unused = (",
     (T_KEYS,), 'KILLED'),

    # The first version of this row rebound `_block_key` AFTER `reference` had
    # already been taken from it, so it mutated nothing and SURVIVED -- a hole
    # in the battery, not in the tests, and exactly the kind a declared
    # expectation can launder. It now cuts the propagation where it actually
    # happens, at the Pad constructor.
    ('the-key-never-reaches-pad-component_ref', 'p',
     "            pad = Pad(\n                component_ref=reference,",
     "            pad = Pad(\n                component_ref=_raw_reference,",
     (T_KEYS, T_PARITY), 'KILLED'),

    # ---- the writer --------------------------------------------------------
    ('writer-matches-by-name-again', 'w',
     "        placement = placement_by_ref.get(key)",
     "        placement = placement_by_ref.get(_raw_ref)",
     (T_WRITER,), 'KILLED'),

    # A fix that always resolves to the FIRST block passes the "one placement
    # moves one block" arm and is still wrong. This row is what proves the
    # second-twin arm is load-bearing.
    ('writer-resolves-every-duplicate-to-block-zero', 'w',
     "    for start, end, fp_text, _raw_ref, key in reversed(\n"
     "            list(iter_footprint_blocks(content))):\n"
     "        placement = placement_by_ref.get(key)",
     "    _seen_raw = set()\n"
     "    for start, end, fp_text, _raw_ref, key in reversed(\n"
     "            list(iter_footprint_blocks(content))):\n"
     "        placement = placement_by_ref.get(_raw_ref)\n"
     "        if _raw_ref in _seen_raw:\n"
     "            placement = None\n"
     "        _seen_raw.add(_raw_ref)",
     (T_WRITER,), 'KILLED'),

    # `matched.add(key)` appears in BOTH writers; anchored on the line above
    # it, which belongs to the pose writer alone. RE-ANCHORED once already:
    # the Phase-2 review moved `matched.add` below the `(at ...)` guard, so a
    # block that resolves but has nothing to rewrite is reported rather than
    # silently neither modified nor named.
    ('writer-counts-blocks-not-placements', 'w',
     "            continue\n        matched.add(key)",
     "            continue\n        matched.add(key)\n        modified_count += 1",
     (T_WRITER,), 'KILLED'),

    ('writer-drops-an-unmatched-placement-in-silence', 'w',
     "    _report_unapplied(placement_by_ref, matched, unapplied_blocks,\n"
     "                      'placement')",
     "    pass",
     (T_WRITER,), 'KILLED'),

    ('the-label-writer-keeps-its-own-ref-lookup', 'w',
     "        res = by_ref.get(key)",
     "        res = by_ref.get(_raw_ref)",
     (T_WRITER,), 'KILLED'),

    # ---- the consumers -----------------------------------------------------
    ('side-maps-keep-the-last-block', 'g',
     "        yield key, fp_text",
     "        yield _raw_ref, fp_text",
     (T_CONS,), 'KILLED'),

    ('stamp_locked-locks-every-namesake', 's',
     "        if key not in want:",
     "        if _raw_ref not in want:",
     (T_WRITER,), 'KILLED'),

    ('footprint-blocks-uses-the-old-formula', 'a',
     "            'footprint_blocks': len(pcb.footprints),",
     "            'footprint_blocks': len(pcb.footprints) + sum(dup_refs.values())\n"
     "                                - len(dup_refs),",
     (T_CONS,), 'KILLED'),

    # SURVIVED on the first run, because nothing covered this function on a
    # board with duplicates: `test_footprint_position_sync.py` does cover it
    # and stays GREEN through the mutation, since it runs on a 61-block board
    # with 61 distinct references. That is a passing gate on a board that
    # cannot express the defect. `test_726_gui_sync.py` was written because
    # this row said so.
    ('gui-sync-matches-by-bare-reference', 'u',
     "        for bfp, ref in zip(_live, _keys):",
     "        for bfp, ref in zip(_live, _raw):",
     (T_SYNC,), 'KILLED'),
]


def _uncache(path):
    """Delete the target's cached bytecode. MEASURED HAZARD, not hygiene.

    CPython validates a `.pyc` on (source mtime SECONDS, source SIZE), and
    several rows here are single-token edits, so a mutated and a restored file
    are the SAME SIZE. Mutate, run and restore inside one second and the `.pyc`
    compiled from the MUTATED source stays valid for every later import in this
    checkout -- the battery exits 0 with a clean tree and nothing points at it.
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
    # Belt and braces with _uncache: a child that writes no bytecode cannot
    # leave a stale .pyc behind for the next row.
    env['PYTHONDONTWRITEBYTECODE'] = '1'
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
            # Skip the target (where exactly one match is the requirement) and
            # THIS file, which carries every anchor as a string literal by
            # construction.
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
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
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
