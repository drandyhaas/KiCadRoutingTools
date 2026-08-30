#!/usr/bin/env python3
"""The #750 mutation battery, shipped so its numbers can be re-derived.

`tests/test_750_fanout_clearance_via_drill.py` records what each arm kills. A
count is only checkable if the exact source edit is written down -- two
reviewers of the #746 branch reconstructed its rows from their names and both
got the wrong answer, because a plausible-looking reconstruction of one row
was semantically inert. So the edits live here, as data, next to the numbers
they produced.

Every row carries an EXPECTATION. Some of these mutations are deliberately
inert -- `not d or d < 0` is indistinguishable from `not d or d <= 0`,
because `not d` already catches 0 and -0.0 -- and an inert row recorded as an
expected survivor is a finding, while an inert row quietly deleted is a hole.
A row whose verdict does not match its expectation is reported as WRONG.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an
A/B replay or a review is reading the same checkout. Every row is restored in
a `finally`, and the file refuses to start on a dirty engine.

    python3 tests/mutate_750.py
    python3 tests/mutate_750.py --row guard-reverted-to-or

A row is KILLED by a FAILURE **or an ERROR**: dropping the `getattr` makes
`test_a_via_like_with_no_drill_is_priced_rather_than_crashing` raise rather
than fail, and a battery that counted only failures would call that a
survivor.

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

ENGINE = os.path.join(_ROOT, 'py_placer', 'placement', 'fanout_clearance.py')
ENGINE_TEST = os.path.join(_TESTS, 'test_750_fanout_clearance_via_drill.py')

_GUARD = """    if not d or d <= 0:
        d = default_drill
    return d / 2.0"""

# (name, old, new, expect)
ROWS = [
    # --- the defect itself: the spelling, not the literal -----------------
    ('guard-reverted-to-or',
     "    d = getattr(via, 'drill', None)\n" + _GUARD,
     "    return (getattr(via, 'drill', None) or default_drill) / 2.0",
     'KILLED'),
    # the plausible-looking guard a later "simplification" actually writes:
    # it still admits every negative.
    ('guard-is-none-or-equals-zero',
     _GUARD,
     """    if d is None or d == 0:
        d = default_drill
    return d / 2.0""",
     'KILLED'),
    # --- the two INERT re-spellings, recorded rather than omitted ---------
    # `not d` already catches 0 and -0.0, so `< 0` and `<= 0` cannot differ.
    ('guard-lt-zero-instead-of-le-zero',
     _GUARD,
     """    if not d or d < 0:
        d = default_drill
    return d / 2.0""",
     'SURVIVED'),
    # `d is None` plus `<= 0` covers exactly what `not d` plus `<= 0` does
    # for every numeric input; only a non-numeric falsy value differs, and
    # no parse path produces one.
    ('guard-is-none-instead-of-not-d',
     _GUARD,
     """    if d is None or d <= 0:
        d = default_drill
    return d / 2.0""",
     'SURVIVED'),
    # --- the constant --------------------------------------------------
    # Arm C brackets it to (0.24, 0.40], so BOTH directions are caught.
    ('constant-lowered-to-0.2',
     '_UNREADABLE_VIA_DRILL = 0.3  # mm',
     '_UNREADABLE_VIA_DRILL = 0.2  # mm',
     'KILLED'),
    ('constant-raised-to-0.5',
     '_UNREADABLE_VIA_DRILL = 0.3  # mm',
     '_UNREADABLE_VIA_DRILL = 0.5  # mm',
     'KILLED'),
    # --- the resolver body ----------------------------------------------
    ('returns-the-diameter-not-the-radius',
     '    return d / 2.0',
     '    return d',
     'KILLED'),
    # kills the only arm that passes a via-like with no `drill` at all --
    # as an ERROR, not a failure.
    ('getattr-dropped',
     "    d = getattr(via, 'drill', None)",
     '    d = via.drill',
     'KILLED'),
    # --- the call sites --------------------------------------------------
    # the offender's drill resolved with the MOVING via's. Needs a rig where
    # the two RESOLVE differently, which arm A does NOT: -0.3 and 0.3 both
    # come back 0.15 there. TestEachViaContributesITSOWNDrill exists because
    # this row survived every behavioural arm until it was added.
    ('offender-drill-resolved-with-the-movers',
     'vdr + ovdr + H2H_VIA',
     'vdr + vdr + H2H_VIA',
     'KILLED'),
    # the typo class. Both floors are in scope, so this is NOT a source-guard
    # blind spot as first assumed -- the positive-count guard sees it, because
    # the needle it counts stops matching. Measured: 3 kills, one of them
    # behavioural (the neighbour that must still be ADMITTED).
    ('h2h-via-floor-swapped-for-the-pad-one',
     'vdr + ovdr + H2H_VIA',
     'vdr + ovdr + H2H_PAD',
     'KILLED'),
    # NOTE: this also perturbs test_732's landing (its _rig(DVS_SMALL,
    # [0.30, 0.02]) arm is decided by this gate), so the kill is not
    # attributable to test_750 alone.
    ('h2h-via-gate-deleted',
     """            # net-INDEPENDENT: two holes collide whatever they carry
            if d < vdr + ovdr + H2H_VIA:
                return False""",
     '            pass',
     'KILLED'),
    # the copper radius substituted for the drill radius at the capsule
    # gate. NOTE: an EXISTING arm kills this first -- test_737:691-716 pins
    # that landing to four places -- so test_750 is not its sole gate.
    ('capsule-gate-reads-the-COPPER-radius',
     'vdr + prad + H2H_PAD',
     'vr + prad + H2H_PAD',
     'KILLED'),
    # the evasion an absence-only source guard would miss: it restores the
    # old semantics exactly while still mentioning the constant.
    ('literal-restored-at-the-capsule-gate',
     'vdr + prad + H2H_PAD',
     '(v.drill or _UNREADABLE_VIA_DRILL) / 2.0 + prad + H2H_PAD',
     'KILLED'),
    # --- the precompute --------------------------------------------------
    # Semantically identical -- a performance regression only, so NO
    # behavioural arm can see it. Killed by the two SOURCE guards, which is
    # exactly why they count the positive form and the ordering rather than
    # just the absence of the old literal. Recorded here so nobody reads
    # the kill as evidence that the inlining changes an answer. It does not.
    ('precompute-inlined-per-candidate',
     '        for ov, ovdr in board_via_drills:',
     '        for ov in pcb_data.vias:\n            ovdr = _via_drill_radius(ov)',
     'KILLED'),
    # THE DANGEROUS version of the tidy-up: fold the coordinates into the
    # same tuple, since they are right there. Silent, and wrong for every via
    # relocated on an earlier cap -- measured during review, the second via
    # lands at (5.1061, 6.1061) instead of (5.0500, 6.0000). A 400-seed
    # old-vs-new fuzz did NOT catch it (only 8 seeds move two vias), which is
    # why the guard for it is structural.
    ('positions-cached-alongside-the-radius',
     [('    board_via_drills = [(ov, _via_drill_radius(ov)) for ov in pcb_data.vias]',
       '    board_via_drills = [(ov, _via_drill_radius(ov), ov.x, ov.y) for ov in pcb_data.vias]'),
      ("""        for ov, ovdr in board_via_drills:
            if ov is v:
                continue
            d = math.hypot(nx - ov.x, ny - ov.y)""",
       """        for ov, ovdr, oX, oY in board_via_drills:
            if ov is v:
                continue
            d = math.hypot(nx - oX, ny - oY)""")],
     None,
     'KILLED'),
    # --- the writer ------------------------------------------------------
    # the gate prices an unreadable drill; the payload must still carry what
    # the board declared.
    ('writer-emits-the-RESOLVED-drill',
     "'drill': v.drill,",
     "'drill': _via_drill_radius(v) * 2,",
     'KILLED'),
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
    if _dirty(ENGINE):
        # Restoring would write the COMMITTED text back over uncommitted work.
        print('REFUSING: %s has uncommitted changes. Commit or stash first -- '
              'this battery restores by overwriting.' % os.path.basename(ENGINE))
        return 2

    orig = io.open(ENGINE, encoding='utf-8', newline='').read()
    results = []
    try:
        for name, old, new, expect in rows:
            # A row may carry several edits: the dangerous version of a
            # mutation is sometimes only expressible as a coordinated pair
            # (see positions-cached-alongside-the-radius), and applying half
            # of it produces a crash rather than the silent defect.
            edits = old if isinstance(old, list) else [(old, new)]
            counts = [orig.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                continue
            mutated = orig
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(ENGINE, 'w', encoding='utf-8', newline='').write(mutated)
            p = subprocess.run([sys.executable, '-X', 'utf8', ENGINE_TEST],
                               capture_output=True, text=True, timeout=1800,
                               cwd=_ROOT)
            io.open(ENGINE, 'w', encoding='utf-8', newline='').write(orig)
            out = (p.stderr or '') + (p.stdout or '')
            failed = [l.split('(')[0].replace('FAIL: ', '')
                      .replace('ERROR: ', '').strip()
                      for l in out.splitlines()
                      if l.startswith(('FAIL:', 'ERROR:'))]
            results.append((name, 'KILLED' if p.returncode else 'SURVIVED',
                            expect, '%d' % len(failed), failed))
    finally:
        io.open(ENGINE, 'w', encoding='utf-8', newline='').write(orig)

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
