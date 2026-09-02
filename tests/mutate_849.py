#!/usr/bin/env python3
"""#849 mutation battery: is the HOIST actually covered, or do its tests
merely run?

    python3 tests/mutate_849.py
    python3 tests/mutate_849.py --row context-accepted-then-ignored
    python3 tests/mutate_849.py --list

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine
files in place and restores them, and a suite running beside it would grade a
mutated tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and
an ERROR count the same, because a mutation that makes the graders crash is
still a mutation the graders noticed. A row whose anchor does not match
EXACTLY ONCE is BROKEN, not skipped: an anchor that silently matches nothing
reports every mutation as killed and is the most flattering possible bug.

WHY THIS BATTERY IS UNUSUALLY LOAD-BEARING. #849 is a hoist: the output is
bit-identical before and after, which is the whole claim. So a row here mostly
cannot be caught by a number moving -- it has to be caught by a MECHANISM arm
(the courtyard parse count) or by a value recorded from BEFORE the change
(`GOLDEN_PRE_HOIST`). Every other assertion in `test_849_lane_context.py` now
runs through `LaneContext` on both sides, so a mutation to what the context
BUILDS moves both sides equally and passes. `mutate_775.py` records the same
shape for the same reason.

Expected SURVIVORS are declared with the reason they are not a test hole.

BYTECODE. Each row rewrites a file and restores it within the same second,
and several rows are size-preserving -- exactly the (mtime, size) pair
CPython's `.pyc` check treats as unchanged. The runner therefore drops the
target's `__pycache__` and runs every test with `-B`; without that a later row
imports an earlier row's mutant and the results are fiction.
"""
import argparse
import os
import shutil
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

ROUT = os.path.join(ROOT, 'py_placer', 'placement', 'routability.py')
CHAN = os.path.join(ROOT, 'py_tools', 'check_channels.py')
TARGETS = {'r': ROUT, 'c': CHAN}

T_849 = os.path.join(TESTS, 'test_849_lane_context.py')
T_RUN6 = os.path.join(TESTS, 'test_run6_check_channels.py')
T_HONESTY = os.path.join(TESTS, 'test_run8_channels_gate_honesty.py')

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the context is threaded, and believed -------------------------
    ('context-accepted-then-ignored', 'r',
     "    ctx = (board_lane_context(pcb_data, clearance, pcb_file=pcb_file)\n"
     "           if context is None\n"
     "           else context.resolved_for(pcb_data, clearance, pcb_file))",
     "    ctx = board_lane_context(pcb_data, clearance, pcb_file=pcb_file)",
     (T_849,), 'KILLED'),

    # ---- the guard --------------------------------------------------------
    ('guard-forgets-the-board', 'r',
     "        if self.pcb_data is not pcb_data:",
     "        if False and self.pcb_data is not pcb_data:",
     (T_849,), 'KILLED'),
    ('guard-forgets-the-file', 'r',
     "        if self.pcb_file != pcb_file:",
     "        if False and self.pcb_file != pcb_file:",
     (T_849,), 'KILLED'),
    ('guard-forgets-the-clearance', 'r',
     "        if abs(self.clearance - float(clearance)) > 1e-9:",
     "        if False and abs(self.clearance - float(clearance)) > 1e-9:",
     (T_849,), 'KILLED'),

    # ---- what the context BUILDS -----------------------------------------
    ('parts-at-a-foreign-clearance', 'r',
     "            self._parts = build_part_pads(self.pcb_data.footprints or {},\n"
     "                                          self.clearance)",
     "            self._parts = build_part_pads(self.pcb_data.footprints or {},\n"
     "                                          0.2)",
     (T_849,), 'KILLED'),
    ('net-owners-sees-one-footprint', 'r',
     "            for r2, f2 in (self.pcb_data.footprints or {}).items():",
     "            for r2, f2 in list(\n"
     "                    (self.pcb_data.footprints or {}).items())[:1]:",
     (T_849,), 'KILLED'),
    ('containers-are-never-exempt', 'r',
     "            self._containers = container_refs(self.pcb_data, self.graded)",
     "            self._containers = set()",
     (T_849,), 'KILLED'),
    ('graded-order-reversed', 'r',
     "            self._graded = graded_parts_from_file(self.pcb_data, self.pcb_file)",
     "            self._graded = list(reversed(\n"
     "                graded_parts_from_file(self.pcb_data, self.pcb_file)))",
     (T_849,), 'KILLED'),

    # ---- the callers ------------------------------------------------------
    ('sweep-context-rebuilt-inside-the-loop', 'c',
     "    ctx = routability.board_lane_context(pcb, clearance, pcb_file=args.board)\n"
     "    for ref in refs:",
     "    for ref in refs:\n"
     "        ctx = routability.board_lane_context(pcb, clearance,\n"
     "                                            pcb_file=args.board)",
     (T_849,), 'KILLED'),
    ('baseline-sweep-reuses-the-board-context', 'c',
     "        base_ctx = routability.board_lane_context(base_pcb, clearance,\n"
     "                                                  pcb_file=args.baseline)",
     "        base_ctx = ctx",
     (T_HONESTY, T_849), 'KILLED'),
    ('anchor-channels-drop-the-context', 'c',
     "        pcb_file=args.board, context=ctx)",
     "        pcb_file=args.board)",
     (T_849,), 'KILLED'),

    # ---- declared survivors ----------------------------------------------
    # `tolerant` only changes behaviour for a footprint whose pad model
    # RAISES, and no committed board has one -- `part_copper_geometry`'s own
    # docstring says the flag exists for hand-built fixtures. The flag itself
    # is covered by tests/test_841_obstruction_rect.py, which asks the
    # geometry directly instead of through a ledger. A test hole would be
    # closing this by inventing a fixture whose only purpose is this row.
    ('parts-built-tolerant', 'r',
     "            self._parts = build_part_pads(self.pcb_data.footprints or {},\n"
     "                                          self.clearance)",
     "            self._parts = build_part_pads(self.pcb_data.footprints or {},\n"
     "                                          self.clearance, tolerant=True)",
     (T_849, T_RUN6), 'SURVIVED'),
    # Dropping `parts=` makes `part_copper_geometry` rebuild its own map
    # (tolerantly) instead of reusing the hoisted one. Same geometry on every
    # committed board -- so this is a PERFORMANCE regression only, which no
    # assertion should claim to detect. mutate_750.py annotates the same
    # shape the same way.
    ('geom-rebuilds-its-own-parts', 'r',
     "            self._geom = part_copper_geometry(\n"
     "                self.pcb_data.footprints or {}, self.clearance,\n"
     "                parts=self.parts)",
     "            self._geom = part_copper_geometry(\n"
     "                self.pcb_data.footprints or {}, self.clearance)",
     (T_849,), 'SURVIVED'),
]


def run(tests):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True, env=env)
        if r.returncode != 0:
            return True, os.path.basename(t)
    return False, ''


def _drop_pycache(path):
    """A size-preserving rewrite inside one second is invisible to CPython.

    The `.pyc` validity check is (source mtime, source size). Several rows
    here change a line's content without changing its length, and the whole
    battery runs inside a few seconds -- so a stale `.pyc` would serve the
    PREVIOUS row's mutant to every later import. Delete it rather than trust
    the timestamp.
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
            print(f'  {n:42s} {os.path.basename(TARGETS[t]):20s} {exp}')
        return 0

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
        print(f'no row named {a.row!r}')
        return 2
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print(f'  {name:42s} BROKEN (anchor matched {src.count(old)}x)')
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
            got = 'KILLED' if died else 'SURVIVED'
            mark = '' if got == exp else '   *** DISAGREES with ' + exp
            if got != exp:
                disagree += 1
            killed += died
            survived += not died
            print(f'  {name:42s} {got:9s} {by}{mark}')
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
