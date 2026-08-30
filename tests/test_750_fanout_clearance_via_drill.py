#!/usr/bin/env python3
"""#750: a via's DRILL must have ONE resolver in the fanout-clearance pass.

`valid_via_pos` priced a via's drill as `(x.drill or 0.3) / 2.0` at three
sites -- the pad drill-capsule gate and both terms of the via-to-via
hole-to-hole gate -- while the COPPER radius on those same two lines already
went through #732's `_via_radius`. One expression, two conventions, and the
drill half had no name at all.

The literal is not the interesting part; the SPELLING is. `or` keeps a
NEGATIVE value, so the drill term goes negative and WEAKENS the gate it is
summed into, and with both drills negative the whole threshold turns negative
and the gate can never fire. Measured at d7658c93 against d04a9c07, on the
rigs below (clearance 0.1, max_shift 0.15, so the only candidate the cap
rect admits is (3.05, 3.00)):

  A. the via-to-via gate, one SAME-NET neighbour at (3.30, 3.00), so the
     copper term at `ov.net_id != v.net_id` is skipped and the hole-to-hole
     term is the ONLY thing that can decide the landing:

       v.drill   ov.drill   old threshold    d7658c93        d04a9c07
        0.3       0.3        +0.5000         no clear spot   no clear spot
       -0.3       0.3        +0.2000         MOVES           no clear spot
        0.0       0.3        +0.5000         no clear spot   no clear spot
       -0.3      -0.3        -0.1000         MOVES           no clear spot

     The last row is the point. A threshold of -0.10 mm is one no distance
     can undercut, so the gate is not weakened, it is OFF -- and the pass
     parks the via 0.2500 mm from the neighbour's centre where the same two
     barrels at a sane 0.3 drill require 0.5000 mm. Exactly HALF the fab
     hole-to-hole floor, from a gate that exists to enforce it.

  B. the pad drill-capsule gate, one NPTH hole (drill 1.0) and no second
     via, so the via-to-via gate is inert:

       v.drill   capsule floor   d7658c93                   d04a9c07
        0.3       1.1000         no clear spot              no clear spot
       -0.3       0.8000         MOVES to 0.9000 of hole    no clear spot
        0.0       1.1000         no clear spot              no clear spot

THE FALLBACK BRANCH HAD NEVER RUN. `synth.make_via` defaults `drill=0.3`, and
every via built anywhere that reaches this function -- test_370, 617, 725,
731, 732, 733, 736, 737, 741, 746 -- carries a positive drill. So no
assertion in the suite has ever executed `(x.drill or 0.3)`'s fallback on
either gate, and no driver has ever passed a via-like carrying no `drill` at
all. That is what `TestTheFallbackResolvesToExactlyPointThree` and the
`getattr` arm are for, and it is why they are worth having even though they
are INERT with respect to the fix.

Two claims an earlier draft of this file made are FALSE and are recorded here
so nobody restores them:

  * "H2H_VIA has no test of any kind." It fires and BINDS today.
    `tests/test_732_...py:257` drives `_rig(DVS_SMALL, [0.30, 0.02])`, which
    parks two vias 0.28mm apart, both FOREIGN -- so the copper term
    short-circuits and H2H_VIA alone decides that arm's landing. What is
    true is weaker: no assertion KEYS on it, so deleting the gate goes green
    there -- measured, test_732 runs 12 tests OK against an engine whose
    H2H_VIA gate is `pass`. It DOES steer that arm's landing (via #2 goes
    (151.32, 116.1389) -> (151.57, 116.5889) without the gate), but the arm
    asserts `moves[0][0]`, the START x, which does not move. So THIS file is
    the sole gate on that row, and trimming
    `test_a_sane_drill_is_refused_by_BOTH_spellings` as redundant with
    test_732 would remove the only thing holding it.
  * "H2H_PAD is covered only as a negative."
    `test_737:693-711`'s `test_the_617_rig_relocates_to_exactly_the_same_place`
    pins a landing to four places that H2H_PAD binds with 0.006588 mm of
    headroom -- landing (1.1148, 1.6772) to NPTH (1.0, 0.98) is 0.706588
    against a drill-gate need of 0.700000. That number is stated in test_737's
    module docstring at :80-84, not beside the arm.

BRACKETING THE CONSTANT, because pinning `_UNREADABLE_VIA_DRILL == 0.3` on
its own is a source assertion, not a behavioural one. Arm C places the NPTH
hole so that the resolved value lands between two decisions:

    hole at x=4.12  ->  REFUSED   => fallback > 0.2714
    hole at x=4.20  ->  ACCEPTED  => fallback <= 0.4000

so the fallback is bracketed to (0.2714, 0.4000] by behaviour, and
`drill=0.0` lands identically to `drill=0.3` in both -- which a fallback of
0, or of `_UNREADABLE_VIA_SIZE`, would not do.

THE LOWER BOUND IS NOT THE AXIAL ARITHMETIC, and a review caught me writing
that it was. (0.24, 0.40] came from the first candidate alone; the rig admits
at least FOUR -- (3.05, 3.0), (3.0462, 3.0191), (3.0354, 3.0354) and
(3.0383, 3.0924) -- and REFUSED means refusing all of them. The farthest is
1.085671 from the hole, so the real bound is f > 0.271342. Confirmed by
patching the constant and running this file:

    constant   REFUSED arm   ACCEPTED arm
      0.25       3 FAIL          ok        <- inside the bracket I first wrote
      0.27       3 FAIL          ok
      0.28         ok            ok
      0.30         ok            ok        <- shipped
      0.40         ok            ok
      0.41         ok          3 FAIL

so both ends bind, and 0.25 -- which the first bracket declared safe -- turns
the REFUSED arm red. Exactly the sort of bound that reads as tight and is
not.

INPUTS WHERE THE TWO SPELLINGS DIFFER, measured over the whole class rather
than assumed. `-0.0` does NOT differ (it is falsy, so both substitute) and
`nan` does NOT differ (both propagate it, and `d < nan + ...` is False either
way) -- do not "tidy" a -0.3 into a -0.0 here, and do not add a NaN arm.
`-1e-9` and `-inf` do differ, but only as SPELLINGS a board cannot use: the
drill class admits no `e`, so the token `1e-9` is unmatchable and the via is
dropped whole. The VALUE is reachable -- `(drill -0.000000001)` matches and
floats to exactly -1e-9 -- so the honest statement is that the reachable
class is plain decimal negatives, which BOTH parse paths admit:
the class carries a `-` and the `float()` after it checks no sign.

The second delta is the `getattr`: the old expression raised AttributeError
on a via-like with no `drill`, and the resolver prices it at the fallback.
Deliberate, and reachable only from a test stand-in.

INERT ON THE CORPUS: 0 of the 602 vias across the 22 boards
`run_utils.corpus_boards()` returns has a falsy or negative drill (the
minimum on the corpus is 0.15). Stated as 602/22, not the issue's 604/27 --
that figure is the repo-wide `*.kicad_pcb` set, five of which live under
tests/fixtures/.

MUTATION BATTERY, 16 rows against the engine: 14 killed, 2 SURVIVED and both
expected, 0 broken. The counts make the `# MUTATION:` notes checkable rather
than decorative, so they are recorded here and the battery itself ships as
`tests/mutate_750.py`:

    returns-the-diameter-not-the-radius     19 assertions  every arithmetic arm
    constant-raised-to-0.5                   8             bracket, upper half
    h2h-via-gate-deleted                     8             + perturbs test_732
    constant-lowered-to-0.2                  7             bracket, lower half
    guard-reverted-to-or                     6             THE DEFECT
    guard-is-none-or-equals-zero             6             still admits negatives
    capsule-gate-reads-the-COPPER-radius     5             vdr -> vr
    offender-drill-resolved-with-the-movers  4             vdr + vdr
    h2h-via-floor-swapped-for-the-pad-one    3             the typo class
    literal-restored-at-the-capsule-gate     3             the source-guard evasion
    precompute-inlined-per-candidate         3             SOURCE guards only
    getattr-dropped                          1             as an ERROR
    positions-cached-alongside-the-radius    1             SOURCE guard only
    writer-emits-the-RESOLVED-drill          1
    guard-lt-zero-instead-of-le-zero         0  SURVIVED, expected: `not d`
    guard-is-none-instead-of-not-d           0  SURVIVED, expected: already covers it

Two rows are killed by a SOURCE guard and by nothing else, and that is a
finding rather than a gap: both are semantically inert (a per-candidate
re-resolve) or need a two-cap geometry a randomized fuzz does not reach (the
cached-position one -- 400 seeds, 0 catches, because only 8 of them move two
vias at all). Where a structural guard is the only guard, the arm says so.

FOUR OF THIS FILE'S `# MUTATION:` NOTES WERE WRONG BEFORE THE BATTERY RAN,
and are corrected in place rather than quietly deleted: two called an arm "a
control, MUTATION: none" when the battery kills it; one said an arm could not
see a constant move when it can; and one credited the `vdr -> vr` swap to an
arm the battery leaves green. A note asserting coverage is a claim like any
other, so write it AFTER the run and put the measured count in it.

Conventions (from #725/#731/#732/#737 and CLAUDE.md): REAL parser
dataclasses; every assertion names the single-line MUTATION that must kill
it, with the count the battery measured; assert you are ON the branch before
asserting about it; every refusal paired with an acceptance that still
happens. The battery itself ships, as `tests/mutate_750.py`.

Runs in-process in ~4 s; the only shelling out is one `git ls-files`.

    python3 tests/test_750_fanout_clearance_via_drill.py
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
import math
import os
import sys
import unittest
from types import SimpleNamespace

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import run_utils
import routing_defaults as defaults
from kicad_parser import BoardInfo, parse_kicad_pcb
from synth import make_pad, make_pcb, make_via
from placement import fanout_clearance as FC
from placement.fanout_clearance import (_UNREADABLE_VIA_DRILL, _via_drill_radius,
                                        _via_radius, nudge_vias_for_unresolved)

# --- the rig ---------------------------------------------------------------
CLEAR = 0.1
MAX_SHIFT = 0.15               # two rings: r = 0.05 and 0.10 (and 0.15)
V_SIZE = 0.5
VR = V_SIZE / 2.0              # 0.25 -- with CLEAR, the cap gate needs 0.35
VIA0 = (3.0, 3.0)              # the offending via, net 3
LANDING = (3.05, 3.0)          # the ONLY candidate the cap rect admits
NEIGH = (3.30, 3.0)            # a SAME-NET neighbour: isolates the drill term
H_DRILL = 1.0                  # the NPTH hole; prad 0.50
PRAD = H_DRILL / 2.0

# The function's own drill floors, mirrored here BY HAND.
#
# They are still function-locals, but since #756 their VALUE is
# `resolve_drill_floors`' answer rather than a literal, so what the mirror
# tracks is that resolver's result on a PROJECT-LESS board -- which is what
# every rig in this file builds. TestOneDrillRule calls it and compares,
# instead of grepping the source for `= 0.2`.
#
# STILL HAND-WRITTEN, deliberately: computing them from the resolver at
# import time would make every ON-THE-BRANCH threshold below
# self-fulfilling, tracking a moved engine value in silence. That is the
# exact failure the guard exists to prevent.
H2H_VIA = 0.2
H2H_PAD = 0.45

# A foreign-net (2) cap pad rect whose right edge is 0.32 from the via -- just
# inside the offender threshold VR + CLEAR = 0.35, so the cap is grazed and the
# offender loop runs -- while the r = 0.05 candidate at (3.05, 3.00) sits 0.37
# from it and is admitted. LANDING is the FIRST candidate the sweep reaches,
# NOT the only admitted one: measured, the rect also admits (3.0462, 3.0191),
# (3.0354, 3.0354) and (3.0383, 3.0924). So `moves == []` says the gate under
# test refused EVERY admitted candidate -- the stronger claim, and the reason
# the bracket's lower bound comes from the farthest of them rather than from
# LANDING (see the docstring).
RECT = (2.0, 2.9, 2.68, 3.1, 2)

# The value the three removed sites hard-coded, mirrored the way test_732
# mirrors OLD_NUDGER_RADIUS. Every "the two spellings differ" arm below is
# vacuous unless the drill it uses forces a branch where they do.
OLD_NUDGER_DRILL_RADIUS = 0.3 / 2.0

# Arm C's two hole positions and the bracket they impose on the fallback.
BRACKET_REFUSE_X = 4.12        # refuses EVERY admitted candidate => f > 0.2714
BRACKET_ACCEPT_X = 4.20        # admits the first one            => f <= 0.4000
# Measured by sweeping the resolver's default at 0.001, NOT derived from the
# axial candidate -- see the docstring for the two wrong brackets that came
# from doing it the other way round.
BRACKET_LO, BRACKET_HI = 0.2714, 0.4000


class _FakeCap:
    """Minimal stand-in for _Cap: fixed pad rects, never moves."""

    def __init__(self, rects):
        self._rects = list(rects)
        self.x = self.y = self.rot = 0.0

    def pad_rects(self, x=None, y=None, rot=None):
        return self._rects


class _FakeSt:
    """The duck-typed `st` the #370/#617 harnesses pass: no resolvers at all,
    so every requirement in the nudger falls back to the flat scalar."""

    #: A REAL `st` is a `_Repair` and carries this. The duck type must too, or
    #: the most likely wrong hole floor -- `max(clearance, getattr(st,
    #: 'npth_floor', 0.0))` -- reads 0.0 here, behaves exactly like the right
    #: one, and is caught by nothing (test_737 measured that). 0.40 is above
    #: every gap these rigs use, so a gate that reads it refuses landings the
    #: checker calls clean.
    npth_floor = 0.40

    def __init__(self, rects):
        self.caps = {'C1': _FakeCap(rects)}
        self.vias = []

    def graze_penalty(self, ref, cap, x, y, rot):
        return 1.0          # permanently unresolved, so the offender loop RUNS


def _nudge(st, pcb, clear=CLEAR, **kw):
    """Drive the real pass, capturing what it printed. The PRINTED OUTPUT is
    half the evidence: a refusal that prints nothing is indistinguishable from
    a pass that never looked (the #732 silent-failure lesson)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        moves, segs = nudge_vias_for_unresolved(st, pcb, clear, **kw)
    return moves, segs, buf.getvalue()


def _bi():
    return BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                     board_bounds=(0.0, 0.0, 8.0, 8.0))


def _neighbour_board(v_drill, ov_drill):
    """Arm A: the offending via plus ONE same-net neighbour, no pads at all."""
    v = make_via(VIA0[0], VIA0[1], net_id=3, size=V_SIZE, drill=v_drill)
    ov = make_via(NEIGH[0], NEIGH[1], net_id=3, size=V_SIZE, drill=ov_drill)
    pcb = make_pcb(board_info=_bi(), vias=[v, ov], segments=[], footprints={},
                   pads_by_net={}, zones=[])
    return v, ov, pcb


def _hole_board(v_drill, hole_x):
    """Arm B/C: the offending via plus ONE NPTH hole, and NO second via, so
    the via-to-via gate cannot contribute."""
    v = make_via(VIA0[0], VIA0[1], net_id=3, size=V_SIZE, drill=v_drill)
    hole = make_pad(net_id=0, x=hole_x, y=VIA0[1], ref='H1REF', num='H1',
                    size_x=H_DRILL, size_y=H_DRILL, shape='circle',
                    layers=['F.Mask', 'B.Mask'], drill=H_DRILL,
                    pad_type='np_thru_hole')
    pcb = make_pcb(board_info=_bi(), vias=[v], segments=[], footprints={},
                   pads_by_net={0: [hole]}, zones=[])
    return v, hole, pcb


def _run(pcb):
    moves, segs, out = _nudge(_FakeSt([RECT]), pcb, max_shift=MAX_SHIFT)
    land = (moves[0][2]['x'], moves[0][2]['y']) if moves else None
    return moves, segs, out, land


def _old_threshold(a, b, floor):
    """What the REMOVED spelling computed. Kept as an executable expression
    rather than a number in a comment, so the numbers in the docstring above
    can be re-derived by reading this file."""
    return (a or 0.3) / 2.0 + (b or 0.3) / 2.0 + floor


class TestTheGateThatStoppedFiring(unittest.TestCase):
    """A negative drill made the via-to-via hole-to-hole term NEGATIVE, and
    two of them switched the gate off entirely."""

    def test_a_sane_drill_is_refused_by_BOTH_spellings(self):
        """The control. Without it every refusal below could be a rig that
        refuses everything."""
        v, ov, pcb = _neighbour_board(0.3, 0.3)
        # ON THE BRANCH: the fallback is NOT taken, so this arm is about the
        # declared value and the two spellings must agree.
        self.assertTrue(v.drill and ov.drill, 'this arm needs declared drills')
        self.assertAlmostEqual(_old_threshold(0.3, 0.3, H2H_VIA), 0.5, places=9)
        moves, _segs, out, _land = _run(pcb)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: h2h-via-gate-deleted -- a refusal that is the gate's, so
    # deleting the gate turns it into a move. Killed by 1 row; it is also the
    # control that makes the -0.3 rows below mean something.

    def test_a_NEGATIVE_drill_no_longer_buys_a_landing(self):
        """d7658c93 moved the via here; d04a9c07 refuses it."""
        v, ov, pcb = _neighbour_board(-0.3, 0.3)
        # ON THE BRANCH, two guards: the removed spelling really did go
        # negative on this input, and the resolver really does not.
        self.assertLess((v.drill or 0.3) / 2.0, 0.0,
                        'the drill is not negative, so the old spelling and '
                        'the resolver agree and this arm cannot fail')
        self.assertAlmostEqual(_via_drill_radius(v), OLD_NUDGER_DRILL_RADIUS,
                               places=9)
        # ... and the landing sits in the window the two thresholds straddle.
        d = math.hypot(LANDING[0] - NEIGH[0], LANDING[1] - NEIGH[1])
        self.assertAlmostEqual(d, 0.25, places=9)
        self.assertLess(_old_threshold(-0.3, 0.3, H2H_VIA), d,
                        'the OLD threshold does not admit this landing, so '
                        'the arm proves nothing about the change')
        self.assertGreater(_via_drill_radius(v) + _via_drill_radius(ov)
                           + H2H_VIA, d)
        moves, _segs, out, _land = _run(pcb)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: the guard back to `or` -> the via moves to (3.05, 3.00) and
    # both the print and the count assertions fail. This arm kills 6 of the
    # battery's 16 rows -- the two guard re-spellings, both constant moves,
    # the diameter/radius slip, and the deleted gate.

    def test_TWO_negative_drills_switched_the_gate_OFF(self):
        """The headline. A threshold of -0.10mm is one no distance can
        undercut -- the gate is not weakened, it is inoperative, and the pass
        parked the via at HALF the fab hole-to-hole floor."""
        v, ov, pcb = _neighbour_board(-0.3, -0.3)
        # ON THE BRANCH: the removed spelling's threshold really is negative.
        old_t = _old_threshold(-0.3, -0.3, H2H_VIA)
        self.assertLess(old_t, 0.0, 'the old threshold is not negative here, '
                                    'so "the gate cannot fire" is not what '
                                    'this arm is testing')
        self.assertAlmostEqual(old_t, -0.1, places=9)
        # What the same two barrels require once the drill is read sanely.
        sane = _old_threshold(0.3, 0.3, H2H_VIA)
        d = math.hypot(LANDING[0] - NEIGH[0], LANDING[1] - NEIGH[1])
        self.assertAlmostEqual(d / sane, 0.5, places=9,
                               msg='the landing is no longer exactly half the '
                                   'sane floor; restate the docstring')
        moves, _segs, out, _land = _run(pcb)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: the guard back to `or` -> 1 move to (3.05, 3.00), 0.2500 from
    # a neighbour whose sane floor is 0.5000.

    def test_a_falsy_drill_is_inert_across_the_change(self):
        """0.0 substitutes identically in both spellings -- the fallback is
        NOT where the behaviour moved, and saying so is half the honesty of
        this file."""
        v, _ov, pcb = _neighbour_board(0.0, 0.3)
        self.assertFalse(v.drill, 'the via declares a drill, so the fallback '
                                  'branch is not taken and this arm is vacuous')
        self.assertAlmostEqual(_old_threshold(0.0, 0.3, H2H_VIA),
                               _via_drill_radius(v) + 0.15 + H2H_VIA, places=9)
        moves, _segs, out, _land = _run(pcb)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: either constant move -> the substituted radius changes and
    # the ON-THE-BRANCH equality between the two spellings fires. Killed by 4
    # rows, of which only 2 are constant moves (the others are the
    # diameter/radius slip and the deleted gate). (An earlier version of this note claimed the arm could NOT see a
    # constant move; the battery says otherwise, which is why the notes are
    # written after it runs and carry its counts.)


class TestEachViaContributesITSOWNDrill(unittest.TestCase):
    """The via-to-via gate sums TWO resolved radii, and arm A cannot tell them
    apart: -0.3 and 0.3 both resolve to 0.15, so `vdr + ovdr` and `vdr + vdr`
    are the same number there. The battery said so -- those mutations were
    killed only by the source guard. These two arms give the neighbour a
    DIFFERENT drill (0.5 -> 0.25) and place it where each mutation flips a
    decision, so the gate is pinned behaviourally as well as structurally.

    v.drill 0.3 -> 0.15, ov.drill 0.5 -> 0.25, so the correct floor is
    0.15 + 0.25 + 0.20 = 0.60. `vdr + vdr` gives 0.50 and the H2H_PAD typo
    gives 0.85, and one landing distance sits inside each gap."""

    #: 0.55 is between the mutant floor (0.50) and the correct one (0.60):
    #: correct REFUSES, `vdr + vdr` accepts.
    NEAR = (LANDING[0] + 0.55, LANDING[1])
    #: 0.70 is between the correct floor (0.60) and the H2H_PAD typo (0.85):
    #: correct ACCEPTS, the typo refuses.
    FAR = (LANDING[0] + 0.70, LANDING[1])

    def _board(self, neigh):
        v = make_via(VIA0[0], VIA0[1], net_id=3, size=V_SIZE, drill=0.3)
        ov = make_via(neigh[0], neigh[1], net_id=3, size=V_SIZE, drill=0.5)
        pcb = make_pcb(board_info=_bi(), vias=[v, ov], segments=[],
                       footprints={}, pads_by_net={}, zones=[])
        return v, ov, pcb

    def test_the_two_drills_really_are_different(self):
        """ON THE BRANCH for both arms below. Without this the pair is arm A
        again, and every mutation that swaps one term for the other survives
        -- which is exactly what the first version of this file did."""
        v, ov, _pcb = self._board(self.NEAR)
        self.assertNotAlmostEqual(_via_drill_radius(v), _via_drill_radius(ov),
                                  places=9,
                                  msg='the neighbour resolves to the same '
                                      'radius as the mover, so vdr + ovdr and '
                                      'vdr + vdr are indistinguishable here')
        self.assertAlmostEqual(_via_drill_radius(v) + _via_drill_radius(ov)
                               + H2H_VIA, 0.6, places=9)
    # MUTATION: returns-the-diameter-not-the-radius -- the two radii double
    # and the 0.6 equality fires. Killed by 1 row; it is the ON-THE-BRANCH
    # guard for the two arms below, not a claim of its own.

    def test_a_neighbour_inside_the_SUMMED_floor_is_refused(self):
        v, ov, pcb = self._board(self.NEAR)
        d = math.hypot(LANDING[0] - ov.x, LANDING[1] - ov.y)
        self.assertAlmostEqual(d, 0.55, places=9)
        self.assertLess(d, _via_drill_radius(v) + _via_drill_radius(ov)
                        + H2H_VIA)
        self.assertGreater(d, 2 * _via_drill_radius(v) + H2H_VIA,
                           'the landing also clears the MOVER-doubled floor, '
                           'so this arm cannot see that mutation')
        moves, _segs, out, _land = _run(pcb)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: `vdr + ovdr` -> `vdr + vdr` at the via-to-via gate -> the
    # floor falls to 0.50, the landing at 0.55 is admitted, and the via moves.

    def test_a_neighbour_outside_it_is_still_ADMITTED(self):
        """The acceptance half. Without it the arm above is satisfied by any
        rig that refuses everything, and the H2H_PAD typo -- which only ever
        makes the gate stricter -- would have nothing to fail."""
        v, ov, pcb = self._board(self.FAR)
        d = math.hypot(LANDING[0] - ov.x, LANDING[1] - ov.y)
        self.assertAlmostEqual(d, 0.7, places=9)
        self.assertGreater(d, _via_drill_radius(v) + _via_drill_radius(ov)
                           + H2H_VIA)
        self.assertLess(d, _via_drill_radius(v) + _via_drill_radius(ov)
                        + H2H_PAD,
                        'the landing also clears the PAD floor, so swapping '
                        'the two floors here would change nothing')
        moves, _segs, _out, land = _run(pcb)
        self.assertEqual(len(moves), 1)
        self.assertEqual(land, LANDING)
    # MUTATION: H2H_VIA -> H2H_PAD at the via-to-via gate -> the floor rises
    # to 0.85, every candidate is refused, and this move disappears.


class TestTheCapsuleGateReadsTheSameResolver(unittest.TestCase):
    """The other site: the via's drill against a board pad's drill capsule.
    A separate arm because a fix applied to one line and not the other is
    exactly the class of bug #750 exists to remove."""

    def test_a_NEGATIVE_drill_no_longer_buys_a_landing_at_the_CAPSULE(self):
        v, _hole, pcb = _hole_board(-0.3, 3.95)
        # ON THE BRANCH: no second via, so the via-to-via gate cannot be what
        # decides this -- the capsule gate is alone.
        self.assertEqual(len(pcb.vias), 1)
        d = abs(LANDING[0] - 3.95)
        self.assertAlmostEqual(d, 0.9, places=9)
        self.assertLess((-0.3) / 2.0 + PRAD + H2H_PAD, d,
                        'the OLD capsule floor does not admit this landing')
        self.assertGreater(_via_drill_radius(v) + PRAD + H2H_PAD, d,
                           'the NEW capsule floor does not refuse it either, '
                           'so the arm separates nothing')
        moves, _segs, out, _land = _run(pcb)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: the guard back to `or` -> 1 move to (3.05, 3.00), 0.9000
    # from a hole whose sane floor is 1.1000. Killed by 3 rows, including
    # `literal-restored-at-the-capsule-gate` -- the evasion that keeps the
    # constant's NAME while restoring the old semantics. NOT by swapping
    # `vdr` for `vr` here: measured, that leaves this arm green (the bracket
    # arms and the writer arm catch it instead).

    def test_the_sane_drill_control_at_the_capsule(self):
        v, _hole, pcb = _hole_board(0.3, 3.95)
        self.assertTrue(v.drill)
        moves, _segs, out, _land = _run(pcb)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: none -- a control.


class TestTheFallbackResolvesToExactlyPointThree(unittest.TestCase):
    """The branch no test in this repo had ever executed, on either gate.

    Pinning `_UNREADABLE_VIA_DRILL == 0.3` is a source assertion. These two
    arms pin it BEHAVIOURALLY, by placing the hole so the resolved value
    lands between an acceptance and a refusal, which brackets it to
    (0.24, 0.40] -- and by showing a declared 0.3 lands identically, which a
    fallback of 0, or of _UNREADABLE_VIA_SIZE (0.5), would not."""

    def _land(self, v_drill, hole_x):
        v, _hole, pcb = _hole_board(v_drill, hole_x)
        # ON THE BRANCH: for the 0.0/None rows the fallback really is taken.
        # `assertFalse(v.drill)` would be TAUTOLOGICAL here -- Via is a plain
        # dataclass with no normalisation, so `v.drill is v_drill` and the
        # assertion cannot fail under `not v_drill`. Assert the RESOLVED value
        # instead, which is what the gate below actually consumes.
        if not v_drill:
            self.assertAlmostEqual(_via_drill_radius(v),
                                   _UNREADABLE_VIA_DRILL / 2.0, places=12,
                                   msg='the fallback branch is not taken')
        moves, _segs, _out, land = _run(pcb)
        return len(moves), land

    def test_the_fallback_is_REFUSED_where_a_smaller_one_would_pass(self):
        d = abs(LANDING[0] - BRACKET_REFUSE_X)
        self.assertAlmostEqual(d, 1.07, places=9)
        for drill in (0.0, None):
            with self.subTest(drill=drill):
                self.assertEqual(self._land(drill, BRACKET_REFUSE_X), (0, None))
        # ... and the same board with the value DECLARED behaves identically,
        # which is the whole claim: the fallback resolves to 0.3, not near it.
        self.assertEqual(self._land(0.3, BRACKET_REFUSE_X), (0, None))
        self.assertGreater(_UNREADABLE_VIA_DRILL, BRACKET_LO,
                           'a fallback at or below %.2f would ACCEPT this '
                           'landing' % BRACKET_LO)
    # MUTATION: the constant to 0.2 (or 0.1, or 0) -> the floor falls below
    # 1.07 and all three rows move.

    def test_the_fallback_is_ACCEPTED_where_a_larger_one_would_not(self):
        d = abs(LANDING[0] - BRACKET_ACCEPT_X)
        self.assertAlmostEqual(d, 1.15, places=9)
        for drill in (0.0, None, 0.3):
            with self.subTest(drill=drill):
                self.assertEqual(self._land(drill, BRACKET_ACCEPT_X),
                                 (1, LANDING))
        self.assertLessEqual(_UNREADABLE_VIA_DRILL, BRACKET_HI,
                             'a fallback above %.2f would REFUSE this landing'
                             % BRACKET_HI)
        # The negative control that keeps the pair honest: a LARGER declared
        # drill on the same board does refuse, so the gate is live here and
        # the acceptance above is not "this rig accepts everything".
        self.assertEqual(self._land(0.5, BRACKET_ACCEPT_X), (0, None))
    # MUTATION: the constant to 0.5 -> the 0.0 and None rows refuse while the
    # 0.3 row still moves, so the subTest reports which row diverged.


class TestTheTwoResolversAgree(unittest.TestCase):
    """#750 is #732's rule applied to the drill, and `via_rad`'s docstring
    already stated that rule for the size. These arms pin that the two
    functions take the same branch on the same inputs -- the property a later
    edit to either would break silently."""

    #: (value, takes-the-fallback). Note what is NOT here and why:
    #: `nan` -- both spellings propagate it and `d < nan + ...` is False
    #: either way, so a NaN arm would be vacuous rather than reassuring.
    CASES = ((None, True), (0, True), (0.0, True), (-0.0, True),
             (False, True), (-1e-9, True), (-0.3, True), (-1.0, True),
             (0.15, False), (0.2, False), (0.3, False), (0.5, False),
             (1.0, False))

    #: Detecting "did it substitute?" by comparing the ANSWER to the
    #: constant does not work, and getting that wrong is the first thing this
    #: file did: `_via_drill_radius(drill=0.3)` returns 0.15, which is exactly
    #: what the fallback returns, so the test passed the value through and
    #: read it as a substitution. Both resolvers take the default as a
    #: PARAMETER, so pass one nothing else could produce and read the branch
    #: off the answer.
    SENTINEL = 9.0

    def test_the_drill_resolver_branches_exactly_as_the_size_one_does(self):
        for val, is_fallback in self.CASES:
            with self.subTest(val=val):
                d = _via_drill_radius(SimpleNamespace(drill=val), self.SENTINEL)
                s = _via_radius(SimpleNamespace(size=val), self.SENTINEL)
                took_d = (d == self.SENTINEL / 2.0)
                took_s = (s == self.SENTINEL / 2.0)
                self.assertEqual(took_d, is_fallback,
                                 'the drill resolver %s the default on %r'
                                 % ('took' if took_d else 'did not take', val))
                self.assertEqual(took_d, took_s,
                                 'the two resolvers take DIFFERENT branches '
                                 'on %r' % (val,))
    # MUTATION: the guard to `if d is None or d == 0` -> every negative row
    # stops taking the fallback and the assertEqual fires with the value.

    def test_minus_zero_does_NOT_differ_and_that_is_why_it_is_here(self):
        """`-0.0` is falsy, so both spellings substitute. Recorded as its own
        arm so nobody "tidies" a -0.3 elsewhere in this file into a -0.0 and
        turns a real arm vacuous."""
        self.assertAlmostEqual((-0.0 or 0.3) / 2.0,
                               _via_drill_radius(SimpleNamespace(drill=-0.0)),
                               places=12)
    # MUTATION: both constant moves and the diameter/radius slip -- killed
    # by 3 rows. It documents a NON-difference between the two SPELLINGS, not
    # an absence of coverage.

    def test_a_via_like_with_no_drill_is_priced_rather_than_crashing(self):
        """The SECOND behavioural delta, and the one an earlier draft of the
        PR body missed. The removed expression raised AttributeError here."""
        blank = SimpleNamespace()
        with self.assertRaises(AttributeError):
            (blank.drill or 0.3) / 2.0          # what the three sites did
        self.assertAlmostEqual(_via_drill_radius(blank),
                               _UNREADABLE_VIA_DRILL / 2.0, places=12)
    # MUTATION: drop the `getattr` for `via.drill` -> this arm ERRORS (not
    # fails), which is why the battery counts an error as a kill.

    def test_the_constant_agrees_with_the_toolchains_own_via_drill_TODAY(self):
        """A self-expiring change detector, NOT a wiring. They are separate
        questions -- see the constant's comment -- and they happen to agree."""
        self.assertEqual(_UNREADABLE_VIA_DRILL, 0.3)
        self.assertEqual(_UNREADABLE_VIA_DRILL, defaults.VIA_DRILL,
                         'routing_defaults.VIA_DRILL has moved. These are '
                         'deliberately NOT wired together (a fab default vs a '
                         'pricing assumption); re-read the constant\'s comment '
                         'and decide, do not just nudge this number')
    # MUTATION: the constant to anything else -> both assertions fire.


class TestOneDrillRule(unittest.TestCase):
    """A source guard, because the bug was not a wrong number -- it was N
    numbers. Scoped to the FUNCTION and comment-stripped: `(v.drill or 0.3)`
    appears in _via_drill_radius's own docstring describing what it replaced,
    and a module-wide raw-line scan would flag it (test_737:480-486 records
    the same false-positive class)."""

    @staticmethod
    def _src():
        return [l.split('#')[0] for l in
                inspect.getsource(FC.nudge_vias_for_unresolved).splitlines()]

    def _hits(self, needle):
        return [i + 1 for i, l in enumerate(self._src()) if needle in l]

    def test_no_site_resolves_a_via_drill_its_own_way(self):
        src = self._src()
        # ON THE BRANCH: the resolver exists at all.
        self.assertTrue(callable(_via_drill_radius))
        for gone, where in (('.drill or 0.3', 'the two hole-to-hole gates'),
                            ('.drill or ', 'any drill fallback at all')):
            hits = ['%d: %s' % (i + 1, l.strip())
                    for i, l in enumerate(src) if gone in l]
            # Report the offending LINES, not the whole function: an
            # assertNotIn on a source haystack prints the haystack (#732
            # measured 393KB for one such failure).
            self.assertEqual(hits, [],
                             '%s still resolves a drill inline -- %s'
                             % (where, '; '.join(hits)))
    # MUTATION: literal-restored-at-the-capsule-gate -- the evasion that
    # keeps the constant's NAME. Killed by 1 row, and this is the ONLY arm
    # that sees it structurally.

    def test_the_resolver_is_actually_CALLED_twice_and_where(self):
        """Absence alone is evadable: `(ov.drill or _UNREADABLE_VIA_DRILL)`
        would pass the arm above and restore the old semantics exactly. So
        count the positive form too, in the shape test_737:487-495 uses."""
        self.assertEqual(len(self._hits('vdr = _via_drill_radius(v)')), 1,
                         'the per-via hoist is gone or duplicated')
        self.assertEqual(len(self._hits('_via_drill_radius(ov)')), 1,
                         'the per-offender resolve is gone or duplicated')
        self.assertEqual(len(self._hits('vdr + prad + H2H_PAD')), 1,
                         'the pad drill-capsule gate no longer reads vdr')
        self.assertEqual(len(self._hits('vdr + ovdr + H2H_VIA')), 1,
                         'the via-to-via gate no longer reads the resolver')
    # MUTATION: 6 rows -- every edit that moves, renames or duplicates a call
    # site. The widest net in the file, and the reason absence alone is not
    # enough: it is what catches `(ov.drill or _UNREADABLE_VIA_DRILL)`.

    def test_the_offender_radii_are_resolved_BEFORE_the_candidate_sweep(self):
        """Not a style point. The gate runs inside the 16-angle x 12-radius
        sweep, so resolving there costs one call per via per CANDIDATE -- and
        this file's own neighbours (board_pads) are flattened here for
        exactly that reason, in a comment that says so."""
        pre = self._hits('_via_drill_radius(ov)')
        use = self._hits('vdr + ovdr + H2H_VIA')
        self.assertEqual((len(pre), len(use)), (1, 1))
        self.assertLess(pre[0], use[0],
                        'the precompute no longer precedes the gate that '
                        'reads it, so it is not a precompute')
    # MUTATION: 4 rows, of which precompute-inlined-per-candidate is the one
    # this arm exists for -- semantically inert, so NO behavioural arm can see
    # it and this is its only gate.

    def test_the_gate_reads_the_offenders_position_LIVE(self):
        """NOT COVERED BEHAVIOURALLY, and disclosed rather than skipped.

        The precompute caches the drill RADIUS and nothing else, and that is
        the whole of what makes it safe -- "nothing mutates pcb_data.vias" is
        true and is NOT the invariant. The nudger relocates vias IN PLACE
        (`v.x, v.y = nx, ny`) and processes caps one after another, so a
        via's coordinates DO change between iterations. Folding them into the
        same tuple is the obvious tidy-up and is a real defect: measured on a
        two-cap rig during review, the second via lands at (5.1061, 6.1061)
        instead of (5.0500, 6.0000). Those coordinates are a REVIEW
        measurement and are deliberately not re-derivable from this repo: the
        rig is not shipped, because a two-cap geometry tuned to expose one
        mutant is not a general guard and would read as one.

        A 400-seed randomized old-vs-new fuzz did NOT catch that mutant --
        only 8 seeds moved two vias at all -- so a behavioural arm here would
        be one geometry pretending to be a general guard. The structural form
        says exactly what it checks."""
        src = self._src()
        gate = [i for i, l in enumerate(src) if 'ovdr' in l]
        self.assertEqual(len(gate), 2, 'the precomputed radius is read in %d '
                                       'places, expected 2' % len(gate))
        self.assertIn('for ov, ovdr in board_via_drills', src[gate[0]])
        # A 2-tuple, so there is nowhere for a position to be cached.
        self.assertIn('[(ov, _via_drill_radius(ov)) for ov in pcb_data.vias]',
                      ' '.join(src),
                      'the precompute now carries more than (via, radius); if '
                      'that is a POSITION, the gate below is reading a stale '
                      'one for every via relocated on an earlier cap')
        # ... and the distance is still taken from the live attributes.
        dist = [l for l in src if 'math.hypot(nx - ov.x, ny - ov.y)' in l]
        self.assertEqual(len(dist), 1,
                         'the via-to-via gate no longer measures to ov.x/ov.y '
                         'as they are NOW')
    # MUTATION: 4 rows, of which positions-cached-alongside-the-radius is the
    # one this arm exists for -- the silent tidy-up, invisible to a 400-seed
    # fuzz, and this is its only gate.

    def test_this_files_mirrored_floors_are_still_the_engines(self):
        """TWO HALVES, because either alone is evadable (#756).

        The mirrors above are no longer mirrors of LITERALS: #756 made both
        floors `resolve_drill_floors`' answer, board-first and raise-only. So
        mirror the ANSWER on the kind of board every rig in this file builds --
        one with no project, which takes the fab floors -- and, separately, pin
        the SHAPE that makes the answer board-derived at all.

        Value alone would pass a resolver hard-wired to `return 0.2, 0.45,
        None, 'fixed default'`; shape alone would pass a resolver whose fab
        fallback silently moved to 0.15. Both halves are load-bearing.

        The mirrors stay HAND-WRITTEN. Computing them from the resolver at
        import time would make every ON-THE-BRANCH threshold in this file
        self-fulfilling -- the arithmetic would track a moved engine value in
        silence, which is the exact failure this guard exists to prevent."""
        # (i) VALUE, on a project-less board: that is what the mirrors are OF.
        # The resolver returns SIX values -- the last two are its fab terms,
        # which the nudger uses as the ladder's second rung -- and on a
        # project-less board all three pairs are the same numbers.
        got = FC.resolve_drill_floors(
            make_pcb(board_info=BoardInfo(layers={},
                                          copper_layers=['F.Cu', 'B.Cu'],
                                          board_bounds=(0.0, 0.0, 4.0, 4.0)),
                     source_path=''))
        self.assertEqual(got,
                         (H2H_VIA, H2H_PAD, None, 'fixed default',
                          H2H_VIA, H2H_PAD),
                         "the engine's drill floors on a project-less board "
                         "are no longer (%s, %s), so this file's mirrors -- "
                         'and every ON-THE-BRANCH threshold computed from them '
                         '-- are stale' % (H2H_VIA, H2H_PAD))
        # (ii) SHAPE: the names are rebound PER RUNG by the ladder loop, so
        # there is no single assignment to count any more -- what must exist is
        # the loop, and what must not is a literal.
        src_lines = self._src()
        lad = [i + 1 for i, l in enumerate(src_lines)
               if 'for H2H_VIA, H2H_PAD in drill_ladder' in l]
        self.assertEqual(len(lad), 1,
                         'expected exactly one ladder loop binding the two '
                         'floors in the nudger; found %d at function-relative '
                         'line(s) %s. Collapsing it back to one rung is the '
                         'change #756 measured and rejected' % (len(lad), lad))
        lit = [i + 1 for i, l in enumerate(src_lines)
               if l.lstrip().startswith(('H2H_VIA =', 'H2H_PAD ='))]
        self.assertEqual(lit, [],
                         'a drill floor is a LITERAL again at function-'
                         'relative line(s) %s -- #756 exists because both '
                         'were' % lit)
    # MUTATION: a fab-floor move (fab_tiers 0.20 -> 0.15) -> killed by (i)
    # only. A literal re-hardcoded AT THE ASSIGNMENT -> killed by (ii) only.
    # NOT CAUGHT HERE, and this note said the opposite until the battery was
    # actually run: the RESOLVER hard-wired to `return 0.2, 0.45, None, 'fixed
    # default'` SURVIVES all three files' guards -- measured. (ii) reads
    # `_src()`, which is scoped to nudge_vias_for_unresolved, so the assignment
    # line still matches; and (i) is exactly the answer a hard-wire returns.
    # That is not a hole in this guard, it is its scope: this file's mirrors
    # are about the PROJECT-LESS value and a hard-wire leaves them correct.
    # `tests/test_756_...py` owns "is it board-derived at all" and kills it.
    # STILL NOT CAUGHT, and it was not before #756 either: H2H_VIA <-> H2H_PAD
    # swapped at the via-to-via gate, since both names stay in scope. The
    # behavioural arms and mutate_750's h2h-via-floor-swapped-for-the-pad-one
    # carry that one.


class TestThePadSideStillRefusesToInvent(unittest.TestCase):
    """The asymmetry is the point, not an oversight: the pad half of the
    capsule comparison is always MEASURED, and only the via half was made up.
    A later "consistency" pass must not level it the other way."""

    def test_a_pad_with_no_drill_gets_no_capsule_rather_than_a_default(self):
        src = ' '.join(l.split('#')[0] for l in
                       inspect.getsource(FC.nudge_vias_for_unresolved)
                       .splitlines())
        self.assertIn('pad_drill_capsule(p) if (p.drill and p.drill > 0)', src,
                      'the pad capsule now has a fallback of its own; #750 '
                      'named the VIA drill precisely because the pad drill '
                      'beside it is measured')
    # MUTATION: NONE in the battery, and that is a disclosure rather than an
    # omission. It guards a line #750 did not touch, against a "consistency"
    # pass that has not been written; mutating it would only be mutating this
    # arm's own subject to watch it fire.

    def test_the_writer_still_emits_the_drill_the_board_declared(self):
        """The gate prices an unreadable drill at 0.3; the writer must still
        re-emit whatever the via actually carried. A resolver that leaked into
        the payload would silently re-drill the board."""
        v, _hole, pcb = _hole_board(0.0, BRACKET_ACCEPT_X)
        moves, _segs, _out, land = _run(pcb)
        # ON THE BRANCH: this board is one where the via DOES move.
        self.assertEqual(land, LANDING)
        self.assertEqual(moves[0][2]['drill'], 0.0,
                         'the nudge re-emitted the RESOLVED drill instead of '
                         'the declared one')
    # MUTATION: `'drill': v.drill` -> `'drill': _via_drill_radius(v) * 2` in
    # the via_moves payload -> this fires with 0.3.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """Every changed expression is gated on a falsy or negative drill, and no
    tracked board has one -- so the change is provably inert on the corpus.
    Asserted rather than narrated, and self-expiring."""

    def test_no_tracked_via_has_a_falsy_or_negative_drill(self):
        boards = run_utils.corpus_boards()
        if not boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        # The COUNT guard comes from the production selector, not from what I
        # happened to observe: corpus_boards() is `git ls-files`, and a plain
        # glob of kicad_files/ returns 33 in a used working copy (#746 shipped
        # a bound whose candidate set excluded its own counterexample).
        self.assertGreaterEqual(len(boards), 20,
                                'the tracked corpus collapsed; this arm '
                                'proves nothing')
        total = 0
        offenders = []
        smallest = None
        for b in boards:
            try:
                pcb = parse_kicad_pcb(b)
            except Exception:                                    # noqa: BLE001
                continue
            for v in pcb.vias:
                total += 1
                if not v.drill or v.drill <= 0:
                    offenders.append((os.path.basename(b), v.x, v.y, v.drill))
                elif smallest is None or v.drill < smallest:
                    smallest = v.drill
        self.assertGreater(total, 500,
                           'only %d vias found; the corpus scan is not '
                           'reading the boards' % total)
        self.assertEqual(offenders, [],
                         'a tracked board now carries a via with a falsy or '
                         'negative drill: %r. The "inert on the corpus" claim '
                         'in the #750 PR has EXPIRED -- re-run the '
                         'before/after sweep and record the new numbers'
                         % (offenders,))
        self.assertGreater(smallest, 0.0)
    # MUTATION: none -- a self-expiring corpus bound, not a fix assertion.


if __name__ == '__main__':
    unittest.main(verbosity=2)
