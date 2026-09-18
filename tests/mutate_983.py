#!/usr/bin/env python3
"""The #983 / #987 / #988 mutation battery: does anything notice when the seat
writes a pose its own grade refuses?

Three corrections, each a PREFERENCE that hands back what it was given unless
the grade would flag it -- so a broken one still seats, just at the pose the
grade refuses, or quietly moves a seat that was fine:

  #983  `_window_nudge`: a rung written outside its declared along-edge
        window is stepped one 0.001 mm grid unit along the edge, inside.
  #987  `_band_settle`: a rung whose band reading is outside its declared
        overhang band is moved along the edge normal until it is inside.
  #988  `_stage1_geometry_rot` / `_AtRotation`: stage 1 measures a part at
        the rotation it will write, not its input one.

Every row carries an EXPECTATION, and a verdict that does not match it is
reported as WRONG. An anchor that does not match its target EXACTLY ONCE is
BROKEN, checked before anything is written (`preflight`, #877).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- run it in a checkout nothing else
is reading. It refuses to start while a target has uncommitted changes,
because it restores by overwriting.

    python3 -X utf8 tests/mutate_983.py
    python3 -X utf8 tests/mutate_983.py --row nudge-direction-inverted
    python3 -X utf8 tests/mutate_983.py --list

A row is KILLED by any non-zero exit of a listed test, a raised error
included -- which is why every killer runs on the UNMUTATED tree first, and
the battery exits 2 if one fails there. `_uncache` is carried over from
`tests/mutate_975.py`: several rows are same-size edits, and CPython trusts a
`.pyc` on (mtime seconds, size).

EXPECTED SURVIVORS, with the reason, rather than deleted rows:
- `settle-ignores-the-facing-edge`: the settle moves a seat at most 22 um
  along its own edge's normal, and no fixture found makes a move that small
  flip which edge the part sits nearest. The guard stays as a change
  detector.
- `interior-split-not-compared`: the split differs only when a pose carries
  pads across an interior Edge.Cuts contour's two-pad threshold, and no
  fixture here has one; the guard is #975's (`PoseGrader.interior_split`),
  kept so a correction cannot compare two differently-shaped boards.

THE MEASURED RESULT is recorded below from the run, never predicted.

MEASURED on the tree of `#983: cite the committed measurement's own numbers
for the deepened overlaps` (ab4f4854, 2026-09-18, Windows; the killer test run
unmutated first, and green): **54 rows, 52 KILLED, 2 SURVIVED -- the two
expected ones above -- 0 broken.** That is the run of record.

Earlier runs, each of which changed the tests rather than the table:
- 31 rows at 02a0c091: 28 KILLED, 3 SURVIVED, 2 of them unexpected, and both
  holes in my own tests. The cap test's body sat so far inside that the
  settle's three-step bound refused it first, so the cap never bound. The
  seat test's `seats` answered "not a seat" everywhere, which made the
  settle's raw-must-seat guard equivalent.
- 42 rows at 21872ca6: 40 KILLED, 2 expected survivors.
- 51 rows at 43db6c0a: 48 KILLED, 3 SURVIVED, 1 of them unexpected:
  `step-judged-against-its-own-input`. The ladder fixture meant to catch it
  never reached the compound, because its rung sat mid-window and the step
  never fired after a settle. A4b's unit arm and B7c's wiring spy (e4181e9d),
  then B7d's end-to-end fixture from the round-4 review, pin it now.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')
TARGETS = {'sd': SEEDER}

T983 = os.path.join(_TESTS, 'test_983_seat_grade_bounds.py')

#: (name, target, old, new, tests, expect)
ROWS = [
    # ---- #988: stage 1 measures at the rotation it will write ---------------
    ('geometry-at-the-input-rotation', 'sd',
     "    if claim is not None and claim[0] is not None:\n"
     "        return claim[0] % 360.0",
     "    if False:\n"
     "        return claim[0] % 360.0",
     (T983,), 'KILLED'),
    ('candidate-set-measured-at-its-first-entry', 'sd',
     "    if claim is not None and claim[0] is not None:\n"
     "        return claim[0] % 360.0",
     "    if claim is not None and (claim[0] is not None or claim[1]):\n"
     "        return (claim[0] if claim[0] is not None else claim[1][0]) % 360.0",
     (T983,), 'KILLED'),
    ('view-ignores-its-angle', 'sd',
     "        return self._part.rect(x, y, rot)",
     "        return self._part.rect(x, y, self._part.rot)",
     (T983,), 'KILLED'),
    ('the-part-is-turned-to-measure-it', 'sd',
     "            _geo = _AtRotation(part, _geo_rot)",
     "            state.apply_move(ref, part.x, part.y, _geo_rot); _geo = part",
     (T983,), 'KILLED'),
    ('off-lattice-angle-not-materialised', 'sd',
     "            if _geo_rot != part.rot:\n"
     "                _geo_rot = _materialise_rotation(part, _geo_rot)",
     "            if False:\n"
     "                _geo_rot = _materialise_rotation(part, _geo_rot)",
     (T983,), 'KILLED'),
    ('every-angle-materialised', 'sd',
     "            if _geo_rot != part.rot:\n"
     "                _geo_rot = _materialise_rotation(part, _geo_rot)",
     "            if True:\n"
     "                _geo_rot = _materialise_rotation(part, _geo_rot)",
     (T983,), 'KILLED'),
    ('extents-at-the-input-rotation', 'sd',
     "            f_lo, f_hi = _edge_frac_bounds(_geo, bounds, edge)",
     "            f_lo, f_hi = _edge_frac_bounds(part, bounds, edge)",
     (T983,), 'KILLED'),
    ('start-at-the-input-rotation', 'sd',
     "            frac = ((declared_to_ladder_frac(_geo, bounds, edge,",
     "            frac = ((declared_to_ladder_frac(part, bounds, edge,",
     (T983,), 'KILLED'),
    ('window-at-the-input-rotation', 'sd',
     "                _w_lo = declared_to_ladder_frac(_geo, bounds, edge,\n"
     "                                                _e_lo, _e_hi, _win[0])\n"
     "                _w_hi = declared_to_ladder_frac(_geo, bounds, edge,",
     "                _w_lo = declared_to_ladder_frac(part, bounds, edge,\n"
     "                                                _e_lo, _e_hi, _win[0])\n"
     "                _w_hi = declared_to_ladder_frac(part, bounds, edge,",
     (T983,), 'KILLED'),

    # ---- #983: the along-edge step ------------------------------------------
    ('nudge-off', 'sd',
     "    if not _outside_its_along_edge_claim(state, part, entry, edge, x, y):\n"
     "        return x, y\n"
     "    e_lo, e_hi, _ = _declared_edge_span(state, state.board, edge)",
     "    if True:\n"
     "        return x, y\n"
     "    e_lo, e_hi, _ = _declared_edge_span(state, state.board, edge)",
     (T983,), 'KILLED'),
    ('nudge-direction-inverted', 'sd',
     "    step = _WINDOW_GUARD_MM if mid > centre else -_WINDOW_GUARD_MM",
     "    step = _WINDOW_GUARD_MM if mid < centre else -_WINDOW_GUARD_MM",
     (T983,), 'KILLED'),
    ('nudge-moves-the-normal-axis-too', 'sd',
     "        nx, ny = x, round(round(y, 3) + step, 3)",
     "        nx, ny = round(x, 3) + _WINDOW_GUARD_MM, round(round(y, 3) + step, 3)",
     (T983,), 'KILLED'),
    ('nudge-landing-never-checked', 'sd',
     "    if _outside_its_along_edge_claim(state, part, entry, edge, nx, ny):\n"
     "        return x, y",
     "    if False:\n"
     "        return x, y",
     (T983,), 'KILLED'),
    ('nudge-may-make-a-seat', 'sd',
     "        if raw is None or new is None or not _no_worse(new, raw):",
     "        if new is None or (raw is not None and not _no_worse(new, raw)):",
     (T983,), 'KILLED'),
    ('nudge-ignores-what-it-costs', 'sd',
     "        if raw is None or new is None or not _no_worse(new, raw):",
     "        if raw is None or new is None:",
     (T983,), 'KILLED'),
    ('nudge-raises', 'sd',
     "    except Exception:                                   # noqa: BLE001\n"
     "        # A preference may not cost a seat: anything raised while asking",
     "    except ZeroDivisionError:\n"
     "        # A preference may not cost a seat: anything raised while asking",
     (T983,), 'KILLED'),
    ('window-miss-never-said', 'sd',
     "    if not _outside_its_along_edge_claim(state, part, entry, edge, part.x, part.y):\n"
     "        return None",
     "    if True:\n"
     "        return None",
     (T983,), 'KILLED'),
    ('seat-ladder-not-stepped', 'sd',
     "                    x, y = _window_nudge(state, part, entry, edge, x, y, seats, rung)",
     "                    pass",
     (T983,), 'KILLED'),
    ('stage-one-not-stepped', 'sd',
     "                    _x, _y = _window_nudge(state, part, c, edge, _x, _y, _s1_seats, _rung)",
     "                    pass",
     (T983,), 'KILLED'),
    ('seat-ladder-step-without-its-rung', 'sd',
     "                    x, y = _window_nudge(state, part, entry, edge, x, y, seats, rung)",
     "                    x, y = _window_nudge(state, part, entry, edge, x, y, seats)",
     (T983,), 'KILLED'),
    ('stage-one-step-without-its-rung', 'sd',
     "                    _x, _y = _window_nudge(state, part, c, edge, _x, _y, _s1_seats, _rung)",
     "                    _x, _y = _window_nudge(state, part, c, edge, _x, _y, _s1_seats)",
     (T983,), 'KILLED'),
    ('step-judged-against-its-own-input', 'sd',
     "        raw = seats(*(origin or (x, y)))",
     "        raw = seats(x, y)",
     (T983,), 'KILLED'),
    ('stage-one-kept-pose-re-derived', 'sd',
     "            elif converged and _kept is not None:",
     "            elif False:",
     (T983,), 'KILLED'),

    # ---- #987: the band settle ---------------------------------------------
    ('settle-off', 'sd',
     "        if inside(amount):\n"
     "            return x, y",
     "        if True:\n"
     "            return x, y",
     (T983,), 'KILLED'),
    ('settle-sign-flipped', 'sd',
     "        sign = 1.0 if amount > lo else -1.0",
     "        sign = -1.0 if amount > lo else 1.0",
     (T983,), 'KILLED'),
    ('settle-y-direction-flipped', 'sd',
     "        ix, iy = _INWARD[edge]\n"
     "        rx, ry = round(x, 3), round(y, 3)",
     "        ix, iy = _INWARD[edge]\n"
     "        iy = -iy\n"
     "        rx, ry = round(x, 3), round(y, 3)",
     (T983,), 'KILLED'),
    ('settle-uncapped', 'sd',
     "_BAND_SETTLE_CAP_MM = 0.022",
     "_BAND_SETTLE_CAP_MM = 1.0",
     (T983,), 'KILLED'),
    ('settle-trades-for-a-setback', 'sd',
     "        if sign > 0 and _carries_setback(entry) and legacy <= EPS:\n"
     "            return x, y",
     "        if False:\n"
     "            return x, y",
     (T983,), 'KILLED'),
    ('settle-ignores-what-it-costs', 'sd',
     "            if new is None or (raw is not None and not _no_worse(new, raw)):",
     "            if new is None:",
     (T983,), 'KILLED'),

    # ---- what a correction may not trade for its fix (`_no_worse`) ---------
    ('floor-pads-short-ignored', 'sd',
     "    if n_pads > r_pads or n_worst > r_worst + EPS:",
     "    if n_worst > r_worst + EPS:",
     (T983,), 'KILLED'),
    ('floor-worst-shortfall-ignored', 'sd',
     "    if n_pads > r_pads or n_worst > r_worst + EPS:",
     "    if n_pads > r_pads:",
     (T983,), 'KILLED'),
    ('new-overlap-allowed', 'sd',
     "        if was < _OVERLAP_REPORTED_MM2 <= grown:\n"
     "            return False\n"
     "        if grown <= was + EPS:\n"
     "            continue\n"
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:",
     "        if False:\n"
     "            return False\n"
     "        if grown <= was + EPS:\n"
     "            continue\n"
     "        if was > 0.0 and grown >= 2.0 * was:",
     (T983,), 'KILLED'),
    ('threshold-crossing-boundary-open', 'sd',
     "        if was < _OVERLAP_REPORTED_MM2 <= grown:",
     "        if was < _OVERLAP_REPORTED_MM2 < grown:",
     (T983,), 'KILLED'),
    ('threshold-crossing-within-the-slack', 'sd',
     "        if was < _OVERLAP_REPORTED_MM2 <= grown:\n"
     "            return False",
     "        if False:\n"
     "            return False",
     (T983,), 'KILLED'),
    ('unchanged-pair-not-skipped', 'sd',
     "        if grown <= was + EPS:\n"
     "            continue\n"
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:",
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:",
     (T983,), 'KILLED'),
    ('overlap-growth-refused-too', 'sd',
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:",
     "        if True:",
     (T983,), 'KILLED'),
    ('overlap-reported-threshold-dropped', 'sd',
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:",
     "        if grown >= 2.0 * was:",
     (T983,), 'KILLED'),
    ('overlap-may-grow-past-double', 'sd',
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:",
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 3.0 * was:",
     (T983,), 'KILLED'),
    ('overlap-exactly-double-allowed', 'sd',
     "        if was < _OVERLAP_REPORTED_MM2 or grown >= 2.0 * was:",
     "        if was < _OVERLAP_REPORTED_MM2 or grown > 2.0 * was:",
     (T983,), 'KILLED'),
    ('overlap-summed-not-paired', 'sd',
     "        was = r_ov.get(ref, 0.0)",
     "        was = sum(r_ov.values())",
     (T983,), 'KILLED'),
    ('stage-one-grades-the-pile', 'sd',
     "                                     pose_grader, set(unplaced) - {ref})",
     "                                     pose_grader, ())",
     (T983,), 'KILLED'),
    ('seat-ladder-overlaps-the-pile', 'sd',
     "                                     [o for o in state.parts if o != ref and o not in ex],",
     "                                     [o for o in state.parts if o != ref],",
     (T983,), 'KILLED'),
    ('grade-errors-not-compared', 'sd',
     "        if n_split != r_split or list(_fp.grade_delta(r_err, n_err)):",
     "        if n_split != r_split:",
     (T983,), 'KILLED'),
    ('interior-split-not-compared', 'sd',
     "        if n_split != r_split or list(_fp.grade_delta(r_err, n_err)):",
     "        if list(_fp.grade_delta(r_err, n_err)):",
     (T983,), 'SURVIVED'),
    ('reading-never-asks-the-grade', 'sd',
     "    if grade is None:\n"
     "        return floor + (overlap, None, None)",
     "    if True:\n"
     "        return floor + (overlap, None, None)",
     (T983,), 'KILLED'),
    ('seat-ladder-overlap-with-nobody', 'sd',
     "                                     [o for o in state.parts if o != ref and o not in ex],",
     "                                     [],",
     (T983,), 'KILLED'),
    ('stage-one-overlap-with-nobody', 'sd',
     "                return _seat_reading(state, part, ref, sx, sy, part.rot, sorted(placed),",
     "                return _seat_reading(state, part, ref, sx, sy, part.rot, [],",
     (T983,), 'KILLED'),
    ('settle-may-make-a-seat', 'sd',
     "        if raw is None:\n"
     "            return x, y\n"
     "        # Outward",
     "        if False:\n"
     "            return x, y\n"
     "        # Outward",
     (T983,), 'KILLED'),
    ('settle-enforces-the-seats-own-max', 'sd',
     "        hi = (entry.get('overhang_mm') or {}).get('max')\n"
     "        top = None if hi is None else float(hi) + EPS",
     "        hi = (entry.get('overhang_mm') or {}).get('max', max(2.0 * max(lo, 0.5), lo + 1.0))\n"
     "        top = None if hi is None else float(hi) + EPS",
     (T983,), 'KILLED'),
    ('band-read-at-the-unrounded-pose', 'sd',
     "    px, py = round(x, 3), round(y, 3)\n"
     "    legacy = state.edge_gate.rect_outside_amount(part.rects(px, py, part.rot)[0])\n"
     "    amount, basis, _row = band_amount(",
     "    px, py = x, y\n"
     "    legacy = state.edge_gate.rect_outside_amount(part.rects(px, py, part.rot)[0])\n"
     "    amount, basis, _row = band_amount(",
     (T983,), 'KILLED'),
    ('settle-ignores-the-facing-edge', 'sd',
     "        if (_faces_its_edge(state, part, entry, edge, x, y)\n"
     "                and not _faces_its_edge(state, part, entry, edge, nx, ny)):",
     "        if (False\n"
     "                and not _faces_its_edge(state, part, entry, edge, nx, ny)):",
     (T983,), 'SURVIVED'),
    ('settle-raises', 'sd',
     "        return nx, ny\n"
     "    except Exception:                                   # noqa: BLE001\n"
     "        return x, y",
     "        return nx, ny\n"
     "    except ZeroDivisionError:\n"
     "        return x, y",
     (T983,), 'KILLED'),
    ('seat-ladder-not-settled', 'sd',
     "                    x, y = _band_settle(state, part, entry, edge, lo, x, y, seats)",
     "                    pass",
     (T983,), 'KILLED'),
    ('stage-one-not-settled', 'sd',
     "                    _x, _y = _band_settle(state, part, c, edge, lo, _x, _y, _s1_seats)",
     "                    pass",
     (T983,), 'KILLED'),
]

from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _uncache(path):
    """Delete the target's cached bytecode -- see this module's docstring."""
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

    # The battery is only evidence if every killer passes UNMUTATED first.
    for path in TARGETS.values():
        _uncache(path)
    for t in sorted({t for row in rows for t in row[4]}):
        p = subprocess.run([sys.executable, '-X', 'utf8', t],
                           capture_output=True, text=True,
                           encoding='utf-8', errors='replace',
                           timeout=1800, cwd=_ROOT)
        if p.returncode:
            print('BROKEN: %s fails on the UNMUTATED tree (exit %d); every '
                  'verdict below would be meaningless.'
                  % (os.path.basename(t), p.returncode))
            return 2
        print('  unmutated %-40s passes' % os.path.basename(t))

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
