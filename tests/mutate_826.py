#!/usr/bin/env python3
"""The #826 mutation battery, shipped so its numbers can be re-derived.

`tests/test_826_portfolio_lattice.py` records what each arm kills. A count is
only checkable if the exact source edit is written down, so the edits live here
as data, next to the numbers they produced.

WHY THIS IS A SEPARATE FILE FROM `mutate_708.py`, which is the obvious place.
That battery scores each row by counting FAIL lines from the suites in the
row's own `tests` tuple, and its seventeen counts are frozen in
`test_708_seed_relative_snap.py`'s header, recorded from a run and never edited
to match. #826's arms call `jitter_lattice`, which calls `infer_board_grid`, so
adding them to #708's suites would make its seven `bg` rows (the occupancy
floor, the tie-break, MIN_PARTS, the sample, the tolerance) newly fail them,
plus any quench row the #826 end-to-end arm also detects -- moving counts #708
froze. On a branch stacked on the open #825 that would force a re-record of the
parent's evidence in order to land a child. One battery per PR is also this
repo's usual shape -- 22 `mutate_*.py` files before this one, though
`mutate_705_792_794.py` shows it is per PR rather than strictly per issue.

TWO ROWS EXIST BECAUSE A POPULATION ARM CANNOT KILL THEM, and both facts were
measured before the arms were written:

  * the zero-offset branch never fires at the shipped radius -- 0 rejections on
    all eleven lattice boards at radius 4.0 AND 2.0, first firing at 1.0. So
    `a-zero-offset-counts-as-a-perturbation` is killed by a SCRIPTED rng, not
    by a board.
  * an outward-snapping draw is 1-6 per board -- likely, not guaranteed. An arm
    resting on one being drawn goes quietly vacuous the day a fixture moves, so
    `the-radius-test-goes-back-before-the-snap` is scripted too.

AND ONE ROW EXISTS BECAUSE THE OBVIOUS FIXTURE CANNOT KILL IT.
`splitflap_driver`, the default board of every other test_portfolio*.py,
perturbs 29 parts of which ZERO have an off-lattice seed -- so
`snap(part.x + dx) - part.x == snap(dx)` and `the-jitter-snaps-the-absolute-
pose` is INVISIBLE there. `interf_u_unrouted` perturbs 22 of 22 with seven
off-lattice. The arm asserts that precondition as a FAILURE rather than a skip.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
engine, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 -X utf8 tests/mutate_826.py
    python3 -X utf8 tests/mutate_826.py --row the-jitter-snap-is-dropped
    python3 -X utf8 tests/mutate_826.py --list

A row is KILLED by a FAILURE **or an ERROR**. An anchor that does not match
EXACTLY ONCE is reported as BROKEN rather than skipped -- `str.replace` of an
absent needle returns the file unchanged, so a stale anchor would report its row
as a survivor.

`_uncache` is carried over from `mutate_708.py` and is NOT hygiene: several of
these are same-size edits, and CPython validates a `.pyc` on (mtime seconds,
size) alone.

THE MEASURED TABLE IS IN THE HEADER OF
`tests/test_826_portfolio_lattice.py`, FROM THE RUN.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

PORTFOLIO = os.path.join(_ROOT, 'py_placer', 'placement', 'portfolio.py')
TARGETS = {'pf': PORTFOLIO}

T_PF = os.path.join(_TESTS, 'test_826_portfolio_lattice.py')
T_STRAT = os.path.join(_TESTS, 'test_portfolio_strategies.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- the snap itself ----------------------------------------------
    ('the-jitter-snap-is-dropped', 'pf',
     "            if lattice is not None:\n"
     "                # The OFFSET, never `part.x + dx`.",
     "            if False:\n"
     "                # The OFFSET, never `part.x + dx`.",
     (T_PF,), 'KILLED'),

    # The row the fixture choice exists for. Invisible on splitflap_driver.
    ('the-jitter-snaps-the-absolute-pose', 'pf',
     "                dx = snap_to_grid(dx, lattice)\n"
     "                dy = snap_to_grid(dy, lattice)",
     "                dx = snap_to_grid(part.x + dx, lattice) - part.x\n"
     "                dy = snap_to_grid(part.y + dy, lattice) - part.y",
     (T_PF,), 'KILLED'),

    # #708's headline row, ported to the fourth site. Would NOT kill on a 0.05
    # board, where every 0.1-multiple is also a 0.05-multiple.
    ('the-offset-snaps-to-the-raster-not-the-lattice', 'pf',
     "                dx = snap_to_grid(dx, lattice)\n"
     "                dy = snap_to_grid(dy, lattice)",
     "                dx = snap_to_grid(dx, 0.1)\n"
     "                dy = snap_to_grid(dy, 0.1)",
     (T_PF,), 'KILLED'),

    # ---- the two guards the snap introduces ---------------------------
    ('a-zero-offset-counts-as-a-perturbation', 'pf',
     "                if dx == 0.0 and dy == 0.0:",
     "                if False:",
     (T_PF,), 'KILLED'),

    ('the-radius-test-goes-back-before-the-snap', 'pf',
     "                if math.hypot(dx, dy) > radius + 1e-9:",
     "                if r > radius + 1e-9:",
     (T_PF,), 'KILLED'),

    # ---- the resolver -------------------------------------------------
    ('the-lattice-falls-back-to-the-grid-step-raster', 'pf',
     "    return step, dict(ev, source='inferred' if step is not None "
     "else 'none',\n"
     "                      resolved=step)",
     "    if step is None:\n"
     "        return 0.1, dict(ev, source='grid_step', resolved=0.1)\n"
     "    return step, dict(ev, source='inferred', resolved=step)",
     (T_PF,), 'KILLED'),

    ('the-radius-guard-is-dropped', 'pf',
     "    if step is not None and step > radius:",
     "    if False:",
     (T_PF,), 'KILLED'),

    # ---- the wiring ---------------------------------------------------
    # Every row above can pass with the sampler correct and `generate` never
    # using it. This is the only one that pins the connection.
    ('the-generator-does-not-pass-the-lattice', 'pf',
     "            poses = perturb_jitter(oracle, free, rng, radius,\n"
     "                                   lattice=lattice)",
     "            poses = perturb_jitter(oracle, free, rng, radius)",
     (T_PF,), 'KILLED'),

    ('the-lattice-is-resolved-per-candidate-from-the-live-state', 'pf',
     "    lattice, lattice_ev = jitter_lattice(pcb, radius)",
     "    lattice, lattice_ev = None, {'source': 'none', 'step': None,\n"
     "                                 'resolved': None, 'reason': 'disabled'}",
     (T_PF,), 'KILLED'),

    # ---- the opt-out that protects perturb.py's positive control ------
    ('the-default-becomes-the-inferred-lattice', 'pf',
     "                   lattice: Optional[float] = None) -> List[Dict]:",
     "                   lattice: Optional[float] = 0.3175) -> List[Dict]:",
     (T_STRAT,), 'KILLED'),

    # ---- the disclosure ----------------------------------------------
    ('the-disclosure-drops-the-board-grid', 'pf',
     "            'board_grid_step': (m.get('board_grid') or {}).get('step'),",
     "            'board_grid_step': None,",
     (T_PF,), 'KILLED'),
]


def _uncache(path):
    """Delete the target's cached bytecode. MEASURED HAZARD, not hygiene."""
    import importlib
    import importlib.util
    try:
        cached = importlib.util.cache_from_source(path)
        if os.path.exists(cached):
            os.remove(cached)
    except (OSError, ValueError, NotImplementedError):
        pass
    importlib.invalidate_caches()


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path, base = TARGETS[tgt], orig[tgt]
            n = base.count(old)
            if n != 1:
                results.append((name, 'BROKEN', expect,
                                'anchor matched %d times' % n, []))
                print('  ran %-52s BROKEN' % name)
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(old, new, 1))
            _uncache(path)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=1800, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += [l.strip()[5:].strip()[:90]
                           for l in out.splitlines()
                           if l.strip().startswith('FAIL')]
                if 'Traceback' in out:
                    failed.append('raised: '
                                  + out.strip().splitlines()[-1][:70])
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            _uncache(path)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed[:3]))
            print('  ran %-52s %s' % (name, results[-1][1]))
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
            print('%s      %s' % (' ' * w, f))
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
        for r in ROWS:
            print('%-52s %-4s %s' % (r[0], r[1], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
