#!/usr/bin/env python3
"""#862: the enclosure test -- the kernel, on arithmetic derived by hand.

`escape.face_of` calls a pad INTERIOR when its copper box is further than
`max(pad_pitch/2, INTERIOR_EPS)` from all four edges of the part's copper box.
That box is the union of EVERY pad's copper, and a box cannot tell a few small
marks outside the pad field from a ring that encloses it: on ulx3s U1, eight
UNNETTED 0.127 x 0.508mm alignment marks sit 0.954mm beyond the ball field on
all four sides, so all 379 netted balls read interior and the part reports
demand 0 on every face.

This file is the gate for the OCCUPANCY half that answers it. Sections 1-2
cover the kernel alone -- `free_run` and `pad_band` -- because the corpus
numbers that follow in later sections are only meaningful if the arithmetic
under them was specified before it was measured. EVERY expected value here is
derived in the comment above it, from the fixture's own numbers; none was read
off an implementation.

Deliberately unit-only and fixture-only: it runs no board and takes well under
a second, so it stays a FAST test that `run_all.py --fast` collects.

    python3 -X utf8 tests/test_862_enclosure.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'py_placer'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'py_router'))

from placement import escape as E                              # noqa: E402

RUN_ALL_TIMEOUT = 120
RUN_ALL_FAST_OK = True

FAILURES = []
PASSED = []
SECTIONS = 2


def check(name, cond, detail=''):
    if cond:
        PASSED.append(name)
    else:
        FAILURES.append('{}{}'.format(name, ': ' + detail if detail else ''))


def near(a, b, eps=1e-9):
    return abs(a - b) < eps


# ---------------------------------------------------------------- section 1
def the_free_run_kernel():
    """`free_run` returns the largest CONTIGUOUS gap, not the total.

    The distinction is the whole reason this is not `span_eaten`: measured on
    the tracked corpus at clearance 0.09, thresholding on the total instead
    moves nine refs and takes `qfn_interior_pads` U1 from 5 interior pads to
    2 -- while being a value no-op at clearance 0.20 and 0.25, so a test
    written at the census basis alone cannot tell the two apart.
    """
    # Nothing in the way: the whole strip is free.
    check('empty obstacle list gives the whole strip',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, []), 1.0))

    # One obstacle spanning x 0.0..1.0 inside the band covers everything.
    full = [(0.0, 0.0, 1.0, 1.0)]
    check('an obstacle covering the strip leaves nothing',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, full), 0.0))

    # THE DISCRIMINATING FIXTURE. Strip [0, 1]. Three obstacles at
    # x 0.2..0.3, 0.5..0.6 and 0.8..0.9 leave gaps 0.2 / 0.2 / 0.2 / 0.1,
    # total free 0.7 but the largest single run 0.2. A 0.3mm track fits in
    # NONE of them, and a rule that thresholds on the total would pass it.
    three = [(0.2, 0.0, 0.3, 1.0), (0.5, 0.0, 0.6, 1.0), (0.8, 0.0, 0.9, 1.0)]
    got = E.free_run(0.0, 1.0, (0.0, 1.0), True, three)
    check('three small gaps do not add up to one lane', near(got, 0.2),
          'largest run {} (total free is 0.7)'.format(got))

    # The leading and trailing gaps count too: one obstacle at x 0.4..0.6
    # leaves 0.4 before it and 0.4 after it.
    mid = [(0.4, 0.0, 0.6, 1.0)]
    check('the leading and trailing gaps are runs too',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, mid), 0.4))

    # OVERLAPPING obstacles are unioned, not summed: 0.2..0.5 and 0.4..0.7
    # cover 0.2..0.7, leaving 0.3 before and 0.3 after.
    lap = [(0.2, 0.0, 0.5, 1.0), (0.4, 0.0, 0.7, 1.0)]
    check('overlapping obstacles are unioned',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, lap), 0.3))

    # `grow` inflates ALONG the strip. The three obstacles at 0.2..0.3,
    # 0.5..0.6 and 0.8..0.9, grown by 0.1 on each side, become 0.1..0.4,
    # 0.4..0.7 and 0.7..1.0 -- which between them cover 0.1..1.0, so the only
    # free run left is the leading 0.0..0.1. Every 0.2 gap a bare measurement
    # saw is eaten by the clearance a track may not touch.
    grown = [(0.2, 0.0, 0.3, 1.0), (0.5, 0.0, 0.6, 1.0), (0.8, 0.0, 0.9, 1.0)]
    check('grow closes the gaps a clearance eats',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, grown, grow=0.1), 0.1))

    # Grow is NOT applied across the band, so an obstacle that does not reach
    # the band is not pulled into it by the clearance. This one spans
    # y 2.0..3.0 against a band of (0.0, 1.0).
    off = [(0.0, 2.0, 1.0, 3.0)]
    check('grow does not drag a far obstacle into the band',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, off, grow=0.5), 1.0))

    # STRICTNESS. An obstacle whose band-axis span merely TOUCHES the band is
    # not in the way. This is what excludes the escaping pad itself (its rect
    # starts exactly where the band ends) and its own row-mates, with no
    # self-exclusion bookkeeping -- the property the whole design rests on.
    touch = [(0.0, 1.0, 1.0, 2.0)]      # y 1.0..2.0 against band (0.0, 1.0)
    check('an obstacle touching the band edge is not in the way',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, touch), 1.0))
    touch_lo = [(0.0, -1.0, 1.0, 0.0)]  # y -1.0..0.0, touching the far edge
    check('an obstacle touching the far band edge is not in the way',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, touch_lo), 1.0))
    # ... but one that genuinely enters the band is.
    inside = [(0.0, 0.5, 1.0, 2.0)]
    check('an obstacle that enters the band is in the way',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), True, inside), 0.0))

    # The vertical orientation is the same arithmetic with the axes swapped:
    # the run is measured along y and the band is on x.
    vert = [(0.0, 0.4, 1.0, 0.6)]
    check('the vertical orientation measures along y',
          near(E.free_run(0.0, 1.0, (0.0, 1.0), False, vert), 0.4))

    # THE EPSILON, on real coordinates rather than invented ones. ulx3s U1's
    # ball A2 parses to a copper box whose x runs 131.48000000000002 ..
    # 131.88, so its span is 0.39999999999997726 and a bare `>= 0.4` is
    # FALSE for a nominally 0.4mm pad. Measured, 385 of that part's 389 pads
    # are sub-nominal this way, so it is the rule and not a corner case: a
    # bare comparison puts the BGA's whole outer ring back in the interior
    # bucket at track 0.4 while answering correctly at track 0.2.
    #
    # Invented coordinates do NOT reproduce it -- 139.08 - 138.68 is
    # 0.4000000000000057, on the other side of the nominal width -- which is
    # exactly why this fixture carries the parser's numbers.
    lo, hi = 131.48000000000002, 131.88
    span = E.free_run(lo, hi, (0.0, 1.0), True, [])
    check('the kernel reports the true subtracted span', near(span, hi - lo))
    check('a real ball span is BELOW its nominal width', span < 0.4,
          'A2 span {!r} -- if this ever stops being true the epsilon is '
          'untested, not unnecessary'.format(span))
    check('and clears the nominal width once the epsilon is applied',
          span >= 0.4 - E.FREE_RUN_EPS)


# ---------------------------------------------------------------- section 2
def the_pad_band_geometry():
    """`pad_band` is `face_band` turned inward: pad edge to box edge."""
    # A 10x10 copper box with a 1x1 pad at (4,4)..(5,5).
    rect = (0.0, 0.0, 10.0, 10.0)
    box = (4.0, 4.0, 5.0, 5.0)

    # North is toward miny (KiCad y grows downward, as `_face_geometry` has
    # it), so the band runs from the box's miny up to the pad's own py0, and
    # the strip is the pad's x span.
    lo, hi, band, horiz = E.pad_band(rect, 'north', box)
    check('north: strip is the pad x span, band is miny..py0',
          (lo, hi, band, horiz) == (4.0, 5.0, (0.0, 4.0), True))

    lo, hi, band, horiz = E.pad_band(rect, 'south', box)
    check('south: band is py1..maxy',
          (lo, hi, band, horiz) == (4.0, 5.0, (5.0, 10.0), True))

    lo, hi, band, horiz = E.pad_band(rect, 'west', box)
    check('west: strip is the pad y span, band is minx..px0',
          (lo, hi, band, horiz) == (4.0, 5.0, (0.0, 4.0), False))

    lo, hi, band, horiz = E.pad_band(rect, 'east', box)
    check('east: band is px1..maxx',
          (lo, hi, band, horiz) == (4.0, 5.0, (5.0, 10.0), False))

    # A pad ON an edge of the box gets a zero-depth band there -- which is
    # what "already on the box edge" means, and the case the caller reads as
    # an unconditional escape.
    edge = (4.0, 0.0, 5.0, 1.0)
    _lo, _hi, band, _h = E.pad_band(rect, 'north', edge)
    check('a pad on the box edge has a zero-depth band',
          near(band[1] - band[0], 0.0))

    # The strip is the PAD's span and nothing else -- not a pitch-wide window.
    # Measured, a window collapses ulx3s U1 to 0 interior pads at clearance
    # 0.09, and `pad_pitch` reads 0.100 on qfn_interior_pads, so a
    # pitch-derived strip is fragile by construction.
    narrow = (4.4, 4.0, 4.6, 5.0)
    lo, hi, _b, _h = E.pad_band(rect, 'north', narrow)
    check('the strip is the pad span, not a pitch-wide window',
          near(hi - lo, 0.2))


def main():
    for fn in (the_free_run_kernel, the_pad_band_geometry):
        fn()
    for f in FAILURES:
        print('FAIL: {}'.format(f))
    if FAILURES:
        print('\n{} FAILED of {} checks in {} sections'
              .format(len(FAILURES), len(FAILURES) + len(PASSED), SECTIONS))
        return 1
    print('OK -- {} checks in {} sections'.format(len(PASSED), SECTIONS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
