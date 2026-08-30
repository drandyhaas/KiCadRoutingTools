#!/usr/bin/env python3
"""The #554 mutation battery: does `test_554_order_graph.py` actually bite?

Every mutation below produces a `placement/relocate.py` that IMPORTS, RUNS, and
returns a plausible reach in millimetres with a plausible binding chain. That is
the whole reason this file exists: the failure mode #554 is exposed to is a solve
that is confidently wrong, and a wrong envelope reads exactly like a right one --
the number is a float, the chain names real parts, nothing raises. This repo's own
record is two placement tests that passed without ever reaching the branch they
named, and, in Phase 1 of this very issue, THREE defects that each produced
believable numbers: an unclamped wall edge, a solver that relaxed into pinned
nodes, and an instrument whose zero-offset control failed on 9 of 11 cells.

Edits live here AS DATA with a per-row EXPECTATION. An inert row recorded as an
expected survivor is a finding; an inert row quietly deleted is a hole. A row whose
verdict does not match its expectation is reported as WRONG.

One target (`py_placer/placement/relocate.py`), one grader
(`tests/test_554_order_graph.py`).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES the
module in place. One writer per tree -- do not run it while a suite or a review is
reading the same checkout. It refuses to start on a dirty target, because
restoring would write the COMMITTED text back over uncommitted work.

    python3 tests/mutate_554.py
    python3 tests/mutate_554.py --row identity-unclamped-wall
    python3 tests/mutate_554.py --list

A row is KILLED by a FAILURE **or an ERROR**: some of these make the grader raise
rather than fail, and a battery that counted only failures would call that a
survivor. An anchor that does not match EXACTLY ONCE is BROKEN rather than skipped
-- a battery that silently applies nothing reports every row as a survivor.
`str.replace`, never `sed`.

THE MEASURED TABLE IS IN THE HEADER OF `test_554_order_graph.py`, FROM THE RUN --
never predicted here and never edited afterwards to match.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

RELOC = os.path.join(_ROOT, 'py_placer', 'placement', 'relocate.py')
TARGETS = {'r': RELOC}

T554 = os.path.join(_TESTS, 'test_554_order_graph.py')

_WALL_LO = (
    "            sep = lo_edge - usable[i]\n"
    "            need = min(0.0, sep)\n")
_WALL_HI = (
    "            sep = usable[i + 2] - hi_edge\n"
    "            need = min(0.0, sep)\n")

ROWS = [
    # ---- identity feasibility: the property everything else rests on --------
    # The pair gap stops being clamped to what the pair already has, so a board
    # that ships below clearance no longer satisfies its own system. This is the
    # rule that lets the pass hold an existing violation without healing or
    # deepening it.
    ('identity-unclamped-pair', 'r',
     "            need = min(clr, sep)\n",
     "            need = clr\n",
     (T554,), 'KILLED'),

    # THE BUG THIS BRANCH ACTUALLY SHIPPED, as a row. A part hanging outside the
    # usable inset (an edge connector, a castellated row) gets a POSITIVE slack,
    # `s = 0` stops being feasible, and the symptom was watchy reporting a reach
    # SMALLER than the same block's frozen slide.
    ('identity-unclamped-wall', 'r',
     [(_WALL_LO, "            sep = lo_edge - usable[i]\n"
                 "            need = 0.0\n"),
      (_WALL_HI, "            sep = usable[i + 2] - hi_edge\n"
                 "            need = 0.0\n")],
     None, (T554,), 'KILLED'),

    # The guard itself made blind. If this survives, every identity assertion in
    # the grader is decorative.
    ('identity-check-dead', 'r',
     "    return [e for e in edges if e.slack > tol]\n",
     "    return []\n",
     (T554,), 'KILLED'),

    # ---- the frozen disjunction --------------------------------------------
    # Constrain the axis the pair is LESS separated on. Still one edge per pair,
    # still an envelope, still a number -- the wrong one.
    ('axis-flip', 'r',
     "            if round(sep_x, ROUND_MM) >= round(sep_y, ROUND_MM):\n",
     "            if round(sep_x, ROUND_MM) < round(sep_y, ROUND_MM):\n",
     (T554,), 'KILLED'),

    # Relative ORDER inverted: the part on the low side recorded as the high one.
    ('order-flip', 'r',
     "            lo_ref, hi_ref = (a, b) if a_low else (b, a)\n",
     "            lo_ref, hi_ref = (b, a) if a_low else (a, b)\n",
     (T554,), 'KILLED'),

    # ---- rigidity ----------------------------------------------------------
    # Intra-block pairs constrained. A rigid translate leaves that geometry
    # invariant, and on a real board members sit sub-clearance already, so this
    # makes a block veto its own every candidate -- the exact reason
    # `quench.group_move_valid` excludes them.
    ('intra-unit-edge', 'r',
     "            if ua == ub:\n                continue\n",
     "            if False:\n                continue\n",
     (T554,), 'KILLED'),

    # A block containing a locked member stops being pinned, so the solve
    # cheerfully proposes moving a part the user nailed down.
    ('locked-block-not-pinned', 'r',
     "    pinned = frozenset(u for u, refs in members.items()\n"
     "                       if any(state.parts[r].locked for r in refs))\n",
     "    pinned = frozenset()\n",
     (T554,), 'KILLED'),

    # ---- what may be relaxed -----------------------------------------------
    # Fixed nodes take incoming edges again: a locked part, or a pinned
    # neighbour in the control arm, can be relaxed off zero. This is the second
    # defect Phase 1 shipped, and it is invisible in the output -- the answer
    # still describes a board, just not one where the locked part stayed put.
    ('relax-into-fixed', 'r',
     "        if dst in fixed:\n            continue\n",
     "        if False:\n            continue\n",
     (T554,), 'KILLED'),

    # ---- the pairing itself ------------------------------------------------
    # The control arm stops pinning anything, so frozen == yielding and every
    # gain in the study collapses to zero while both numbers stay plausible.
    ('pin-nothing', 'r',
     "                 pinned=frozenset(u for u in units.members if u != unit))\n",
     "                 pinned=units.pinned)\n",
     (T554,), 'KILLED'),

    # The control arm rebuilds the partition instead of sharing it. The two arms
    # would then be measuring two different boards, which is exactly the
    # confound the paired design exists to remove.
    ('pin-rebuilds-partition', 'r',
     "    return Units(of_ref=units.of_ref, members=units.members,\n",
     "    return Units(of_ref=dict(units.of_ref), members=dict(units.members),\n",
     (T554,), 'KILLED'),

    # ---- pair admission ----------------------------------------------------
    # `None` from `gap_to` means "these share no board side and cannot interact".
    # Treating it as a zero gap invents a constraint between a front part and a
    # back one.
    ('none-gap-becomes-zero', 'r',
     "            if gap is None:            # no shared board side: no interaction\n"
     "                continue\n",
     "            if gap is None:\n                gap = 0.0\n",
     (T554,), 'KILLED'),

    # ---- the outline -------------------------------------------------------
    # No walls at all: every block reaches as far as its neighbours allow and
    # then straight off the board.
    ('no-wall-edges', 'r',
     "    usable = getattr(state, 'usable', None)\n    if not usable:\n",
     "    usable = getattr(state, 'usable', None)\n    if True:\n",
     (T554,), 'KILLED'),

    # ---- the direction -----------------------------------------------------
    # The lower envelope ignored, so travel in the negative direction is priced
    # against the upper bound. Every reach stays a positive float.
    ('reach-ignores-lower-envelope', 'r',
     "        bound = r.hi[axis] if comp > 0 else r.lo[axis]\n",
     "        bound = r.hi[axis]\n",
     (T554,), 'KILLED'),

    # ---- the caller guard --------------------------------------------------
    # The move_refs trap re-armed. A state built with move_refs= has no free
    # variables, so the solve answers "no room" on every board and looks like a
    # correct negative result.
    ('movable-guard-off', 'r',
     "    if movable < 2:\n",
     "    if False:\n",
     (T554,), 'KILLED'),

    # ---- the explanation ---------------------------------------------------
    # The binding chain emptied. #459 asks for the constraint graph AS the
    # explanation of why a block sits where it does; a reach with no chain is a
    # number with no justification.
    ('binding-chain-empty', 'r',
     "    out = []\n    seen = set()\n    cur = node\n",
     "    return ()\n    out = []\n    seen = set()\n    cur = node\n",
     (T554,), 'KILLED'),

    # ---- an expected survivor, recorded rather than deleted -----------------
    # Dropping the deterministic sort on the adjacency lists. CPython dicts
    # preserve insertion order and `order_graph` already emits its edges sorted,
    # so the resulting iteration order is stable within a run and this battery
    # cannot tell the two apart. The guarantee is real -- two processes must not
    # produce different binding chains through equal-cost paths -- and it is
    # currently unattacked. Closing this needs a fixture with two genuinely
    # equal-cost predecessors; until one exists the row stands here as the
    # disclosure rather than as a green tick.
    ('unsorted-adjacency', 'r',
     "        adj[k].sort(key=lambda t: (t[0], t[1]))\n",
     "        pass\n",
     (T554,), 'SURVIVED'),
]


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _uncache(path):
    """Delete the target's cached bytecode. MEASURED HAZARD, not hygiene.

    CPython validates a `.pyc` on (source mtime SECONDS, source SIZE). Several
    rows here are size-preserving edits, so a battery that mutates, runs and
    restores inside one second leaves a `.pyc` compiled from the MUTATED source
    that every later import in this checkout accepts as valid. Measured on the
    #553 branch: after a clean run of that battery, the module on disk read
    `TOP_K = 3` while `import` reported 4, and a study written afterwards
    silently ran at the mutated value. The battery had exited 0 and the tree was
    clean, so nothing pointed at it.
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


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash first '
                  '-- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path, base = TARGETS[tgt], orig[tgt]
            edits = old if isinstance(old, list) else [(old, new)]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                print('  ran %-32s BROKEN' % name)
                continue
            mutated = base
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(path, 'w', encoding='utf-8', newline='').write(mutated)
            _uncache(path)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=600, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += [ln.strip()[5:].strip() for ln in out.splitlines()
                           if ln.strip().startswith('FAIL:')]
                if 'Traceback' in out:
                    failed.append('raised: ' + out.strip().splitlines()[-1][:70])
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            _uncache(path)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:3]))
            print('  ran %-32s %s' % (name, results[-1][1]))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
            _uncache(v)

    print()
    w = max(len(r[0]) for r in results)
    wrong = 0
    for name, verdict, expect, cnt, failed in results:
        mark = ''
        if verdict != expect:
            mark = '   <-- WRONG, expected %s' % expect
            wrong += 1
        print('%-*s  %-9s  %-3s%s' % (w, name, verdict, cnt, mark))
        for f in failed:
            print('%s      %s' % (' ' * w, f[:110]))
    killed = sum(1 for r in results if r[1] == 'KILLED')
    survived = sum(1 for r in results if r[1] == 'SURVIVED')
    broken = sum(1 for r in results if r[1] == 'BROKEN')
    print('\n%d rows: %d killed, %d survived (%d of them expected), %d broken'
          % (len(results), killed, survived,
             sum(1 for r in results if r[1] == r[2] == 'SURVIVED'), broken))
    if wrong or broken:
        print('%d row(s) did not match their expectation' % (wrong + broken))
    return 1 if (wrong or broken) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', default=None, help='run one row by name')
    ap.add_argument('--list', action='store_true', help='list the row names')
    a = ap.parse_args()
    if a.list:
        for name, tgt, _o, _n, tests, expect in ROWS:
            print('%-32s %-4s %-9s %s' % (
                name, tgt, expect,
                ' '.join(os.path.basename(t) for t in tests)))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
