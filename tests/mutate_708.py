#!/usr/bin/env python3
"""The #708 mutation battery, shipped so its numbers can be re-derived.

`tests/test_708_seed_relative_snap.py` records what each arm kills. A count is
only checkable if the exact source edit is written down, so the edits live here
as data, next to the numbers they produced.

WHY THIS BATTERY EXISTS, in this issue's own terms. #708 has two halves that
are easy to confuse and easy to test vacuously:

  * SEED-RELATIVE -- snap the offset, not the absolute position;
  * ON THE BOARD'S LATTICE -- and snap it to the pitch the board was laid out
    on, not to the router's raster.

The second half is the one that does the work, and a test suite that only
checks the first would pass with the fix half-applied. The row
`the-offset-snaps-to-the-raster-not-the-lattice` exists precisely to prove the
arms tell those two apart: with it applied, every offset is still a clean
multiple of `grid_step` and the residue is still preserved -- and the board's
0.3175mm lattice is still destroyed. Measured before writing any of this: at
step=1.0 the raster offsets are {0, +/-1.0, +/-2.0} and only the ZERO offset
lands back on a 0.3175 lattice.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole. A row whose verdict does
not match its expectation is reported as WRONG.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
engine, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 -X utf8 tests/mutate_708.py
    python3 -X utf8 tests/mutate_708.py --row the-tie-break-takes-the-argmax
    python3 -X utf8 tests/mutate_708.py --list

A row is KILLED by a FAILURE **or an ERROR**: several of these make an arm
raise rather than fail, and a battery counting only failures would call that a
survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- `str.replace(old, new, 1)` of an absent needle returns the file
unchanged, so a stale anchor would report its row as a survivor, which reads as
a catastrophic test failure and is really a typo.

`_uncache` is carried over from `tests/mutate_797.py` and is NOT hygiene: two
rows here are single-token edits (`ties[0]` -> `ties[-1]`, `0.67` -> `0.70`)
that leave the file the SAME SIZE, and CPython validates a `.pyc` on
(mtime seconds, size) alone. Mutate, run and restore inside one second and
every later import in this checkout reads the mutant.

THE MEASURED TABLE IS IN THE HEADER OF
`tests/test_708_seed_relative_snap.py`, FROM THE RUN -- never predicted here
and never edited afterwards to match.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

BOARD_GRID = os.path.join(_ROOT, 'py_placer', 'placement', 'board_grid.py')
QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
FANOUT = os.path.join(_ROOT, 'py_placer', 'placement', 'fanout_clearance.py')
RESEAT = os.path.join(_ROOT, 'py_placer', 'placement', 'reseat.py')
TARGETS = {'bg': BOARD_GRID, 'q': QUENCH, 'fc': FANOUT, 'rs': RESEAT}

T_GRID = os.path.join(_TESTS, 'test_708_board_grid.py')
T_SNAP = os.path.join(_TESTS, 'test_708_seed_relative_snap.py')
T_RESEAT = os.path.join(_TESTS, 'test_reseat.py')
T_FANOUT = os.path.join(_TESTS, 'test_fanout_clearance.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- the inference: which rung wins -------------------------------
    ('the-tie-break-takes-the-argmax', 'bg',
     "    out['step'] = ties[0]",
     "    out['step'] = max(out['profile'], key=lambda s: out['profile'][s])",
     (T_GRID,), 'KILLED'),

    ('the-tie-break-takes-the-coarsest', 'bg',
     "    out['step'] = ties[0]",
     "    out['step'] = ties[-1]",
     (T_GRID,), 'KILLED'),

    # ---- the two gates that make it decline ---------------------------
    ('the-min-parts-gate-is-dropped', 'bg',
     "    if n_parts < min_parts:",
     "    if False:",
     (T_GRID,), 'KILLED'),

    ('the-occupancy-floor-is-dropped', 'bg',
     "    if best < floor:",
     "    if False:",
     (T_GRID,), 'KILLED'),

    # The boundary detector. 0.70 is the round number the population gap
    # first suggests, and interf_u_unrouted scores EXACTLY 0.700000, so the
    # verdict there is decided by `>=` against `>` on a float equality.
    ('the-floor-returns-to-the-round-0.70', 'bg',
     "OCCUPANCY_FLOOR = 0.67",
     "OCCUPANCY_FLOOR = 0.70",
     (T_GRID,), 'KILLED'),

    # ---- how occupancy is measured ------------------------------------
    ('the-tolerance-becomes-relative-to-the-step', 'bg',
     "               if abs(v / step - round(v / step)) * step <= tol_mm)",
     "               if abs(v / step - round(v / step)) <= tol_mm)",
     (T_GRID,), 'KILLED'),

    ('the-sample-drops-one-axis', 'bg',
     "    return xs + ys, len(fps)",
     "    return xs, len(fps)",
     (T_GRID,), 'KILLED'),

    # ---- the quench edit: the two halves, separately -------------------
    ('the-candidate-snap-goes-back-to-absolute', 'q',
     "            dx = snap_to_grid(ix * step, lattice)\n"
     "            dy = snap_to_grid(iy * step, lattice)\n"
     "            if math.hypot(dx, dy) > max_disp + 1e-9:\n"
     "                continue\n"
     "            cx = part.seed_x + dx\n"
     "            cy = part.seed_y + dy",
     "            cx = snap_to_grid(part.seed_x + ix * step, lattice)\n"
     "            cy = snap_to_grid(part.seed_y + iy * step, lattice)\n"
     "            if math.hypot(cx - part.seed_x,\n"
     "                          cy - part.seed_y) > max_disp + 1e-9:\n"
     "                continue",
     (T_SNAP,), 'KILLED'),

    # THE row this battery exists for: seed-relative but on the RASTER. The
    # residue is preserved, every offset is a clean multiple of grid_step,
    # and the board's own lattice is still destroyed.
    ('the-offset-snaps-to-the-raster-not-the-lattice', 'q',
     "            dx = snap_to_grid(ix * step, lattice)\n"
     "            dy = snap_to_grid(iy * step, lattice)",
     "            dx = snap_to_grid(ix * step, 0.1)\n"
     "            dy = snap_to_grid(iy * step, 0.1)",
     (T_SNAP,), 'KILLED'),

    ('the-radius-test-goes-back-before-the-snap', 'q',
     "            if math.hypot(dx, dy) > max_disp + 1e-9:\n"
     "                continue\n"
     "            cx = part.seed_x + dx",
     "            if math.hypot(ix * step, iy * step) > max_disp + 1e-9:\n"
     "                continue\n"
     "            cx = part.seed_x + dx",
     (T_SNAP,), 'KILLED'),

    ('the-lattice-is-never-resolved', 'q',
     "    lattice, lattice_evidence = resolve_snap_lattice(pcb_data, grid_step)",
     "    lattice, lattice_evidence = grid_step, {'source': 'grid_step',\n"
     "                                            'step': None,\n"
     "                                            'reason': 'disabled'}",
     (T_SNAP,), 'KILLED'),

    # ---- the group-move probe -----------------------------------------
    ('the-group-probe-goes-back-to-the-absolute-pose', 'q',
     "                if math.hypot(p.x + sdx - p.seed_x,\n"
     "                              p.y + sdy - p.seed_y) > max_disp + 1e-9:",
     "                if math.hypot(snap_to_grid(p.x + sdx, lattice) - p.seed_x,\n"
     "                              snap_to_grid(p.y + sdy, lattice)\n"
     "                              - p.seed_y) > max_disp + 1e-9:",
     (T_SNAP,), 'KILLED'),

    ('the-group-offset-snaps-the-absolute-pose', 'q',
     "            sdx = snap_to_grid(ix * step, lattice)\n"
     "            sdy = snap_to_grid(iy * step, lattice)",
     "            sdx = snap_to_grid(ix * step, 0.1)\n"
     "            sdy = snap_to_grid(iy * step, 0.1)",
     (T_SNAP,), 'KILLED'),

    # ---- the two sites that deliberately do NOT take the lattice -------
    ('the-fanout-snap-goes-back-to-absolute', 'fc',
     "            dx = snap_to_grid(ix * step, grid_step)\n"
     "            dy = snap_to_grid(iy * step, grid_step)\n"
     "            if math.hypot(dx, dy) > max_disp + 1e-9:\n"
     "                continue\n"
     "            cx = cap.seed_x + dx\n"
     "            cy = cap.seed_y + dy",
     "            cx = snap_to_grid(cap.seed_x + ix * step, grid_step)\n"
     "            cy = snap_to_grid(cap.seed_y + iy * step, grid_step)\n"
     "            if math.hypot(cx - cap.seed_x,\n"
     "                          cy - cap.seed_y) > max_disp + 1e-9:\n"
     "                continue",
     (T_FANOUT,), 'KILLED'),

    ('the-reseat-slot-snaps-the-absolute-point', 'rs',
     "        sx = ax + snap_to_grid(x - ax, grid_step)\n"
     "        sy = ay + snap_to_grid(y - ay, grid_step)",
     "        sx = snap_to_grid(x, grid_step)\n"
     "        sy = snap_to_grid(y, grid_step)",
     (T_RESEAT,), 'KILLED'),
]


def _uncache(path):
    """Delete the target's cached bytecode. MEASURED HAZARD, not hygiene --
    see this module's docstring."""
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
