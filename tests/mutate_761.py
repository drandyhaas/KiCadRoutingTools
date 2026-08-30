#!/usr/bin/env python3
"""The #761 mutation battery, shipped so its numbers can be re-derived.

`tests/test_761_legality_npth_keepout.py` records what each arm kills. A count
is only checkable if the exact source edit is written down -- two reviewers of
the #746 branch reconstructed its rows from their names and both got the wrong
answer, because a plausible-looking reconstruction of one row was semantically
inert. So the edits live here, as data, next to the numbers they produced.

Every row carries an EXPECTATION. Some mutations are deliberately inert -- an
exact re-spelling of a `max` cannot change an answer -- and an inert row
recorded as an expected survivor is a finding, while an inert row quietly
deleted is a hole. A row whose verdict does not match its expectation is
reported as WRONG.

ONE target (`legality.py`), unlike `mutate_730.py`'s two, and rows are graded
by more than one test file: the tripwire and corpus arms live in `test_730`,
the standoff and broad-phase arms in `test_761`, and the renamed inflation
guard in `test_placement_pad_legality`. A row is KILLED if ANY of its named
tests exits non-zero.

This is the THIRD copy of this runner (`mutate_730.py`, `mutate_750.py`). It is
not refactored into a shared one deliberately: that would rewrite two shipped
batteries whose recorded counts are the evidence for two merged reviews, which
is a change to make on its own, not inside a fix.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. The file refuses to start on a
dirty engine, because restoring would write the COMMITTED text back over
uncommitted work.

    python3 tests/mutate_761.py
    python3 tests/mutate_761.py --row keepout-drops-the-clearance

A row is KILLED by a FAILURE **or an ERROR**: dropping a term makes some arms
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

LEG = os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py')
QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
TARGETS = {'leg': LEG, 'quench': QUENCH}

T761 = os.path.join(_TESTS, 'test_761_legality_npth_keepout.py')
T730 = os.path.join(_TESTS, 'test_730_fanout_clearance_npth_local_clearance.py')
TPAD = os.path.join(_TESTS, 'test_placement_pad_legality.py')

_KEEPOUT = """            cache = [(ox, oy, r + clr)
                     for ox, oy, r in self.hole_circles(0.0, 0.0, rot)]"""
_HOLE_LOOPS = """    hole = 0.0
    for cx, cy, r in pb.hole_keepouts(xb, yb, rb):
        for a0, a1, a2, a3, _na, _sa in rects_a:
            hole += _circle_rect_penetration(cx, cy, r, (a0, a1, a2, a3))
    for cx, cy, r in pa.hole_keepouts(xa, ya, ra):
        for b0, b1, b2, b3, _nb, _sb in rects_b:
            hole += _circle_rect_penetration(cx, cy, r, (b0, b1, b2, b3))
    return hole"""

# (name, target, old, new, tests, expect)
ROWS = [
    # ---- the keep-out radius itself ------------------------------------
    ('keepout-drops-the-clearance', 'leg',
     _KEEPOUT,
     """            cache = [(ox, oy, r)
                     for ox, oy, r in self.hole_circles(0.0, 0.0, rot)]""",
     (T761, TPAD), 'KILLED'),
    ('keepout-adds-clearance-twice', 'leg',
     _KEEPOUT,
     """            cache = [(ox, oy, r + clr + clr)
                     for ox, oy, r in self.hole_circles(0.0, 0.0, rot)]""",
     (T761, TPAD), 'KILLED'),
    ('keepout-serves-the-EXTENT-radius', 'leg',
     _KEEPOUT,
     """            cache = [(ox, oy, r + clr)
                     for ox, oy, r in self.holes_extent]""",
     (T761,), 'KILLED'),

    # ---- the consumers -------------------------------------------------
    # ONE of the two loops, not both: a battery that reverts both cannot
    # distinguish "the accessor is wired" from "half of it is".
    ('pair-shortfall-reverts-ONE-loop', 'leg',
     '    for cx, cy, r in pb.hole_keepouts(xb, yb, rb):',
     '    for cx, cy, r in pb.hole_circles(xb, yb, rb):',
     (T761,), 'KILLED'),
    ('pair-shortfall-reverts-the-OTHER-loop', 'leg',
     '    for cx, cy, r in pa.hole_keepouts(xa, ya, ra):',
     '    for cx, cy, r in pa.hole_circles(xa, ya, ra):',
     (T761,), 'KILLED'),
    ('census-reverts-to-hole_circles', 'leg',
     '        holes = pp.hole_keepouts(fp.x, fp.y, fp.rotation or 0.0)  # #761',
     '        holes = pp.hole_circles(fp.x, fp.y, fp.rotation or 0.0)',
     (T761,), 'KILLED'),

    # ---- the broad phase -----------------------------------------------
    ('reach-drops-hole_reach', 'leg',
     '        reach = max(pad_reach, pa.hole_reach, pb.hole_reach)',
     '        reach = max(pad_reach, 0.0)',
     (T761,), 'KILLED'),
    ('census-reach-drops-hole_reach', 'leg',
     '    census_reach = max(clearance, board_max_floor, board_max_hole,',
     '    census_reach = max(clearance, board_max_floor,',
     (T761,), 'KILLED'),
    ('hole_reach-forgets-the-clearance', 'leg',
     '            over = (_kr + self.clearance) - _er',
     '            over = _kr - _er',
     (T761,), 'KILLED'),
    ('pair-cap-goes-back-to-a-hardcoded-zero', 'leg',
     """            return PairShortfall(max(0.0, pad_reach - g), g < 0.0,
                                 _hole_shortfall(pa, xa, ya, ra, rects_b,
                                                 pb, xb, yb, rb, rects_a),
                                 g < 0.0)""",
     """            return PairShortfall(max(0.0, pad_reach - g), g < 0.0, 0.0,
                                 g < 0.0)""",
     (T761,), 'KILLED'),

    # ---- the board floor -----------------------------------------------
    ('npth-floor-ignored', 'leg',
     """        _npth_floor = (defaults.NPTH_TO_TRACK_CLEARANCE
                       if (npth_floor is None or not copper_holes)
                       else max(defaults.NPTH_TO_TRACK_CLEARANCE,
                                float(npth_floor)))""",
     '        _npth_floor = defaults.NPTH_TO_TRACK_CLEARANCE',
     (T761, T730), 'KILLED'),
    ('silk-takes-the-board-floor', 'leg',
     """        _npth_floor = (defaults.NPTH_TO_TRACK_CLEARANCE
                       if (npth_floor is None or not copper_holes)
                       else max(defaults.NPTH_TO_TRACK_CLEARANCE,
                                float(npth_floor)))""",
     """        _npth_floor = (defaults.NPTH_TO_TRACK_CLEARANCE
                       if npth_floor is None
                       else max(defaults.NPTH_TO_TRACK_CLEARANCE,
                                float(npth_floor)))""",
     (T761,), 'KILLED'),
    ('npth-floor-can-LOWER-the-fab-floor', 'leg',
     """                       else max(defaults.NPTH_TO_TRACK_CLEARANCE,
                                float(npth_floor)))""",
     '                       else float(npth_floor))',
     (T761,), 'KILLED'),
    ('the-board-floor-leaks-into-EXTENTS', 'leg',
     """                    ext_grow = max(0.0, defaults.NPTH_TO_TRACK_CLEARANCE
                                   - clearance)""",
     '                    ext_grow = max(0.0, _npth_floor - clearance)',
     (T761,), 'KILLED'),
    ('the-census-passes-no-board-floor', 'leg',
     """    parts = build_part_pads(
        fps, clearance, model,
        npth_floor=resolve_npth_floor(pcb_data, pcb_file, clearance_notes))""",
     '    parts = build_part_pads(fps, clearance, model)',
     (T761, T730), 'KILLED'),
    ('partpads-reads-the-board', 'leg',
     '        _npth_floor = (defaults.NPTH_TO_TRACK_CLEARANCE',
     '        resolve_hole_clearance = None\n'
     '        _npth_floor = (defaults.NPTH_TO_TRACK_CLEARANCE',
     (T730,), 'KILLED'),

    ('cap-branch-charges-the-hole-reach-as-PAD', 'leg',
     '            return PairShortfall(max(0.0, pad_reach - g), g < 0.0,',
     '            return PairShortfall(max(0.0, reach - g), g < 0.0,',
     (T761,), 'KILLED'),
    ('resolve-npth-floor-ignores-pcb_file', 'leg',
     """        return max(floor,
                   float(resolve_hole_clearance(pcb_data, None, pcb_file)))""",
     """        return max(floor,
                   float(resolve_hole_clearance(pcb_data, None, None)))""",
     (T761,), 'KILLED'),
    ('failed-floor-resolution-is-silent', 'leg',
     """        if notes is not None:
            notes.append('copper-to-hole floor unresolved (%s: %s); the NPTH '
                         'keep-out falls back to the %.2fmm fab floor'
                         % (type(e).__name__, e, floor))""",
     '        pass',
     (T761,), 'KILLED'),
    ('quench-passes-no-board-floor', 'quench',
     """            _npth = legality.resolve_npth_floor(pcb_data, pcb_file,
                                                _npth_notes)""",
     '            _npth = None',
     (T761,), 'KILLED'),

    # ---- the disclosure ------------------------------------------------
    ('hole-disclosure-removed', 'leg',
     """                if hole_req > clearance + 1e-9:
                    required.append([key[0], key[1], round(hole_req, 4),
                                     'NPTH hole'])""",
     '                pass',
     (T761,), 'KILLED'),
    ('hole-disclosure-fires-for-EVERY-hole', 'leg',
     '                if hole_req > clearance + 1e-9:',
     '                if hole_req > 0.0:',
     (T761,), 'KILLED'),
    ('holes_req-loses-the-clearance-term', 'leg',
     '                    npth_req = max(clearance, _npth_floor, _lc)',
     '                    npth_req = max(_npth_floor, _lc)',
     (T761,), 'KILLED'),

    # ---- deliberately inert, recorded so its survival is not read as a
    # ---- hole ----------------------------------------------------------
    ('keepout-max-respelled-as-a-sum-of-parts', 'leg',
     _HOLE_LOOPS,
     _HOLE_LOOPS.replace('    hole = 0.0', '    hole = 0.0 + 0.0'),
     (T761,), 'SURVIVED'),
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
