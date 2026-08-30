#!/usr/bin/env python3
"""The #730 mutation battery, shipped so its numbers can be re-derived.

`tests/test_730_fanout_clearance_npth_local_clearance.py` records what each arm
kills. A count is only checkable if the exact source edit is written down --
two reviewers of the #746 branch reconstructed its rows from their names and
both got the wrong answer, because a plausible-looking reconstruction of one
row was semantically inert. So the edits live here, as data, next to the
numbers they produced.

Every row carries an EXPECTATION. Some mutations are deliberately inert -- the
override list is already filtered to `lc > npth_step`, so re-spelling the
CHARGE as `max(npth_clr, lc)` cannot change an answer -- and an inert row
recorded as an expected survivor is a finding, while an inert row quietly
deleted is a hole. A row whose verdict does not match its expectation is
reported as WRONG.

Two structural differences from `tests/mutate_750.py`, both forced by #730's
shape:

  * rows target TWO engine files (`fanout_clearance.py`, `legality.py`), so
    each row names its target and BOTH are checked clean before anything runs
    and restored in the `finally`.
  * rows are graded by more than one test file -- site D's exclusion is held by
    `test_617` as well as by `test_730` -- so each row names its tests and is
    KILLED if ANY of them exits non-zero. `test_617` is a `run()`-style script
    rather than unittest, so the FAIL/ERROR scraper tolerates its prose.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. The file refuses to start on a
dirty engine, because restoring would write the COMMITTED text back over
uncommitted work.

    python3 tests/mutate_730.py
    python3 tests/mutate_730.py --row site-A-reverted

A row is KILLED by a FAILURE **or an ERROR**: dropping a `getattr` makes an arm
raise rather than fail, and a battery that counted only failures would call
that a survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

FC = os.path.join(_ROOT, 'py_placer', 'placement', 'fanout_clearance.py')
LEG = os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py')
TARGETS = {'fc': FC, 'leg': LEG}

T730 = os.path.join(_TESTS, 'test_730_fanout_clearance_npth_local_clearance.py')
T737 = os.path.join(_TESTS, 'test_737_fanout_clearance_via_hole.py')
T617 = os.path.join(_TESTS, 'test_617_placement_fanout_hole_clearance.py')

_VIA_GATE = """        if override_holes and override_hole_gap(
                v.net_id, nx, ny, nx, ny) < vr - 1e-4:
            return False"""
_CONN_GATE = """        if override_holes and override_hole_gap(
                net_id, sx, sy, ex, ey) < hw - 1e-4:
            return False"""
_E_BLOCK = """                    _lc = ((getattr(p, 'local_clearance', 0.0) or 0.0)
                           if copper_holes else 0.0)
                    npth_grow = max(0.0, max(defaults.NPTH_TO_TRACK_CLEARANCE,
                                             _lc) - clearance)"""

# (name, target, old, new, tests, expect)
ROWS = [
    # ---- site A: the cap keep-out rect ---------------------------------
    ('site-A-reverted', 'fc',
     '                        grow = max(0.0, max(self.npth_floor, lc) - clearance)',
     '                        grow = max(0.0, self.npth_floor - clearance)',
     (T730,), 'KILLED'),
    ('site-A-loses-the-fab-floor', 'fc',
     'max(0.0, max(self.npth_floor, lc) - clearance)',
     'max(0.0, lc - clearance)',
     (T730,), 'KILLED'),
    ('site-A-threshold-at-clearance', 'fc',
     'max(0.0, max(self.npth_floor, lc) - clearance)',
     'max(0.0, max(clearance, lc) - clearance)',
     (T730,), 'KILLED'),
    ('site-A-getattr-dropped', 'fc',
     "                        lc = getattr(p, 'local_clearance', 0.0) or 0.0",
     '                        lc = p.local_clearance',
     (T730,), 'KILLED'),
    ('site-A-lc-becomes-a-PadFloor', 'fc',
     '                            self.foreign_pad_floors.append(None)',
     '                            self.foreign_pad_floors.append(\n'
     '                                None if self._floors is None\n'
     '                                else self._floors.pad_floor(p))',
     (T730,), 'KILLED'),
    # the long-axis trap: identical for a round hole, so ONLY the slot arm
    # can see it. Recorded because "identical on the corpus" is exactly the
    # reason a slot fixture had to be built by hand.
    ('site-A-long-axis-drill', 'fc',
     '                            hr = hd / 2.0 + grow',
     '                            hr = (p.drill or 0.0) / 2.0 + grow',
     (T730,), 'KILLED'),
    # ---- site B: the via gate ------------------------------------------
    ('via-override-gate-deleted', 'fc', _VIA_GATE, '        pass',
     (T730, T737), 'KILLED'),
    # the LOWER direction of the step threshold. Only the lc-0.15-at-gap-0.120
    # arm can see this: at any gap above the override it is inert.
    ('via-override-threshold-at-clearance', 'fc',
     '            if _hlc <= npth_step + 1e-9:',
     '            if _hlc <= clearance + 1e-9:',
     (T730,), 'KILLED'),
    # ...and the UPPER direction.
    ('via-override-threshold-doubled', 'fc',
     '            if _hlc <= npth_step + 1e-9:',
     '            if _hlc <= 2 * npth_step + 1e-9:',
     (T730,), 'KILLED'),
    ('via-override-uses-the-DRILL-radius', 'fc',
     '                v.net_id, nx, ny, nx, ny) < vr - 1e-4:',
     '                v.net_id, nx, ny, nx, ny) < _via_drill_radius(v) - 1e-4:',
     (T730,), 'KILLED'),
    # the mutation _FakeSt.npth_floor = 0.60 exists for. It refuses the two
    # lc=0.00 CONTROL arms while the headline refusal arms still pass -- which
    # is why those controls are not optional.
    ('via-override-uses-the-BOARD-floor', 'fc',
     '                                     hx1, hy1, hx2, hy2) - hr',
     '                                     hx1, hy1, hx2, hy2) - hr'
     '\n            hlc = max(hlc, getattr(st, \'npth_floor\', 0.0))',
     (T730,), 'KILLED'),
    # The classic #617 mutation on the BASE gate, and the row the lc=0.00
    # CONTROL arms exist for. `via-override-uses-the-BOARD-floor` does NOT
    # reach them -- with no override the hole never enters the filtered list,
    # so the gap helper is never consulted -- which the battery reported and
    # this row corrects.
    ('base-via-gate-reads-st-npth_floor', 'fc',
     '                clearance + vr - 1e-4:',
     "                max(clearance, getattr(st, 'npth_floor', 0.0)) "
     '+ vr - 1e-4:',
     (T730, T737), 'KILLED'),
    ('via-override-drops-the-own-net-skip', 'fc',
     '            if hnet == net_id:\n                continue',
     '            if False:\n                continue',
     (T730,), 'KILLED'),
    # ---- site C: the connector gate ------------------------------------
    ('connector-override-gate-deleted', 'fc', _CONN_GATE, '        pass',
     (T730,), 'KILLED'),
    # #617's floor. Measured on the sibling branch: this used to pass
    # test_617, test_370 and test_fanout_clearance and was caught by a source
    # guard ALONE. test_730 now holds it behaviourally as well.
    ('connector-BASE-floor-lowered', 'fc',
     '                npth_clr + hw - 1e-4:',
     '                clearance + hw - 1e-4:',
     (T730, T737, T617), 'KILLED'),
    # ---- the two INERT re-spellings, recorded rather than omitted -------
    # The list is already filtered to `lc > npth_step >= npth_clr`, so
    # `max(npth_clr, hlc) == hlc` for every member. The FILTER carries the
    # arm, not the charge -- which is the thing a reader is most likely to get
    # backwards, and the reason both rows ship.
    ('via-override-charged-as-a-MAX', 'fc',
     '            if d - hlc < best:',
     '            if d - max(npth_clr, hlc) < best:',
     (T730,), 'SURVIVED'),
    ('override-charge-given-the-VIA-step', 'fc',
     '            if d - hlc < best:',
     '            if d - (hlc if hlc > npth_clr else clearance) < best:',
     (T730,), 'SURVIVED'),
    # ---- site D: the EXCLUDED unification ------------------------------
    # Three independent kills, which is what an excluded site deserves.
    ('nudger-npth_clr-made-BOARD-AWARE', 'fc',
     '    npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE)',
     '    from obstacle_map import resolve_hole_clearance as _rhc0\n'
     '    npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE,\n'
     '                   _rhc0(pcb_data, None))',
     (T730, T737, T617), 'KILLED'),
    ('nudger-threshold-reads-st-instead-of-the-board', 'fc',
     '    npth_step = max(npth_clr, resolve_hole_clearance(pcb_data, None))',
     "    npth_step = max(npth_clr, getattr(st, 'npth_floor', 0.0))",
     (T730, T737), 'KILLED'),
    # ---- the guard that stopped holding #617's mutation ------------------
    # A review laundered the board read into the floor -- `npth_clr` reassigned
    # from `npth_step` on the next line -- and every check in test_737's
    # rewritten guard still passed, because the read itself was still pinned to
    # the `npth_step` assignment. The old blanket ban made this unspellable.
    # Both files now pin the floor's assignment COUNT as well.
    ('board-read-laundered-into-the-floor', 'fc',
     '    npth_step = max(npth_clr, resolve_hole_clearance(pcb_data, None))',
     '    npth_step = max(npth_clr, resolve_hole_clearance(pcb_data, None))\n'
     '    npth_clr = npth_step',
     (T730, T737, T617), 'KILLED'),
    # ---- the SLOT / capsule geometry ------------------------------------
    # All three of these survived the whole fanout test family until #730's
    # review built slot fixtures for them. They are identical to the shipped
    # code for a ROUND hole, which every other rig in the file uses -- so no
    # amount of adding round-hole arms would ever have caught one.
    ('override-hole-treated-as-a-POINT', 'fc',
     '            override_holes.append((_h1[0], _h1[1], _h2[0], _h2[1], _hr,',
     '            override_holes.append((_h1[0], _h1[1], _h1[0], _h1[1], _hr,',
     (T730,), 'KILLED'),
    ('site-A-rect-centred-on-the-PAD-not-the-circle', 'fc',
     '                                (hx - hr, hy - hr, hx + hr, hy + hr, -1, None))',
     '                                (p.global_x - hr, p.global_y - hr,'
     '\n                                 p.global_x + hr, p.global_y + hr,'
     ' -1, None))',
     (T730,), 'KILLED'),
    ('site-E-long-axis-drill', 'leg',
     '                            (hx - fp.x, hy - fp.y, hd / 2.0 + npth_grow))',
     '                            (hx - fp.x, hy - fp.y,'
     '\n                             (p.drill or 0.0) / 2.0 + npth_grow))',
     (T730,), 'KILLED'),
    # ---- site E: PartPads ----------------------------------------------
    ('site-E-reverted-to-the-hoist', 'leg',
     [('        self.max_floor = 0.0    # upper bound on this part'
       "'s pad requirements\n",
       '        self.max_floor = 0.0    # upper bound on this part'
       "'s pad requirements\n"
       '        npth_grow = max(0.0, defaults.NPTH_TO_TRACK_CLEARANCE'
       ' - clearance)\n'),
      (_E_BLOCK, '                    pass')],
     None, (T730,), 'KILLED'),
    ('site-E-loses-the-fab-floor', 'leg',
     '                    npth_grow = max(0.0, max(defaults.NPTH_TO_TRACK_CLEARANCE,\n'
     '                                             _lc) - clearance)',
     '                    npth_grow = max(0.0, _lc - clearance)',
     (T730,), 'KILLED'),
    ('site-E-threshold-at-clearance', 'leg',
     '                    npth_grow = max(0.0, max(defaults.NPTH_TO_TRACK_CLEARANCE,\n'
     '                                             _lc) - clearance)',
     '                    npth_grow = max(0.0, max(clearance, _lc) - clearance)',
     (T730,), 'KILLED'),
    ('site-E-silk-gate-dropped', 'leg',
     "                    _lc = ((getattr(p, 'local_clearance', 0.0) or 0.0)\n"
     '                           if copper_holes else 0.0)',
     "                    _lc = getattr(p, 'local_clearance', 0.0) or 0.0",
     (T730,), 'KILLED'),
    # an exact re-spelling of `max`. Recorded so nobody reads its survival as
    # a coverage hole.
    ('site-E-max-respelled-as-a-conditional', 'leg',
     '                    npth_grow = max(0.0, max(defaults.NPTH_TO_TRACK_CLEARANCE,\n'
     '                                             _lc) - clearance)',
     '                    _f = defaults.NPTH_TO_TRACK_CLEARANCE\n'
     '                    npth_grow = max(0.0, (_lc if _lc > _f else _f)\n'
     '                                    - clearance)',
     (T730,), 'SURVIVED'),
]


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
            path = TARGETS[tgt]
            base = orig[tgt]
            # A row may carry several edits: some mutations are only
            # expressible as a coordinated pair (site-E's hoist needs the
            # hoisted line put back AND the per-pad block removed), and
            # applying half of one produces a crash rather than the defect.
            edits = old if isinstance(old, list) else [(old, new)]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                continue
            mutated = base
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(path, 'w', encoding='utf-8', newline='').write(mutated)
            killed, failed = False, []
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=1800, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                if p.returncode:
                    killed = True
                failed += ['%s::%s' % (os.path.basename(t)[5:8],
                                       l.split('(')[0].replace('FAIL: ', '')
                                       .replace('ERROR: ', '').strip())
                           for l in out.splitlines()
                           if l.startswith(('FAIL:', 'ERROR:'))]
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if killed else 'SURVIVED', expect,
                            '%d' % len(failed), failed))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])

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
    ap.add_argument('--row', help='run a single row by name')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
