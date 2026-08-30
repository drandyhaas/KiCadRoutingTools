"""#732: a via's RADIUS must have ONE resolver in the fanout-clearance pass.

`_Repair.__init__` built its keep-out list from
`v.size if v.size and v.size > 0 else default_via_size`, while
`nudge_vias_for_unresolved` priced the SAME barrel at a hard-coded
`(v.size or 0.5) / 2.0` in four places, and its #313 connectivity tolerance at
`max(1e-3, v.size / 2.0)` with no fallback at all. For a via the board did not
size, the grader and the nudger therefore resolved DIFFERENT keep-outs -- the
divergence the module's own comments forbid for the clearance term (#725).

Three failure modes, all measured at HEAD 0bfc0be0 before the fix:

  * `--default-via-size` ABOVE 0.5: grader keep-out 0.500, nudger threshold
    0.350. The grader reports the cap unresolved (penetration 0.080000) and the
    offender list comes back EMPTY, so the `for v in offenders:` body never
    runs -- not even the "no clear spot" line prints. Captured nudger stdout
    was literally ''. The cap stays unresolved forever with nothing on screen.
  * BELOW 0.5 -- which is the SHIPPED CLI DEFAULT of 0.3
    (bga_fanout.constants.DEFAULT_VIA_SIZE): the nudger over-estimates and
    relocates vias whose grader penetration is exactly 0.000000, recording them
    for the writer and laying connector copper for nothing.
  * The tolerance: a size-0 via collapses `tol` to 1e-3, so a same-net stub end
    0.100mm off the via centre is not matched, `conn_layers` comes back EMPTY,
    and `all([])` is True -- the via is relocated with ZERO connectors and the
    stub is orphaned. That is the pre-#313 behaviour the comment above the line
    says was fixed.

WHY THE GRADERS ARE NOT TOUCHED, and why that is a finding rather than an
omission. `--default-via-size` is an OPERATOR's belief about a board being
placed. It is not licence for a DRC/connectivity grader to disagree with KiCad,
and KiCad honours a declared size literally.

Measured on KiCad 10 with two boards identical but for one via's size, graded
through check_connected's KiCad refill cross-check. State the geometry
precisely, because the naive version of this experiment shows NOTHING: what
governs is the PERPENDICULAR distance from the same-net copper to the barrel
centre, not the stub's endpoint offset, and a track wide enough to cover the
barrel keeps the net connected at either size. With ~0.2mm of effective
separation, KiCad reports 0 unconnected links at `(size 0.5)` and 1 at
`(size 0)`; a first attempt at a 0.100mm endpoint offset with a 0.2mm track
reported 0 in BOTH arms, because the track's own copper already reached the
barrel.

`(size 0)` is additionally a hard KiCad DRC error in its own right
(`via_diameter (min 0.5000 mm; actual 0.0000 mm)`, plus `track_dangling`), which
is the strongest form of the point: KiCad does not quietly assume a diameter
for such a via, it rejects it. So a grader that invented one would report
CONNECTED what KiCad reports OPEN -- the direction that hides an open.
`TestTheGradersStillReadTheBoard` pins that decision behaviourally, on
`connectivity.endpoint_reaches_via`, so a later reader does not "fix"
check_drc / check_connected / connectivity into disagreeing with KiCad.

Conventions this file follows (from #697/#725/#731 and CLAUDE.md):

  * REAL parser dataclasses and REAL boards. `_Repair.__init__` reads
    courtyards and locked refs from the file on disk.
  * Every assertion names the single-line MUTATION that must kill it.
  * Assert you are ON the branch before asserting about it -- each test spies
    the value its branch keys on, with a detail string saying why.
  * Every "is not charged" is paired with a NEGATIVE CONTROL that still is.

Runs in-process in ~20 s; nothing here shells out.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
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

from kicad_parser import BoardInfo, parse_kicad_pcb
from synth import make_pcb, make_seg, make_via
from placement import fanout_clearance as FC
from placement.fanout_clearance import _Repair, nudge_vias_for_unresolved

# Declares no netclass, no .kicad_dru and no pad override, so the clearance
# model is INERT: via_required == clearance and _item_reach == clearance on
# BOTH sides. The radius is then the only variable that can differ, which is
# the isolation this issue needs and which orangecrab (6 fiducials carrying
# local_clearance 0.375) cannot give.
BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
CLEAR = 0.1
PREFIX = 'C,R,FB'          # the CLI default; with 'C' alone R-refs never appear
CAP = 'C16'                # a decoupling cap with clear board on both sides
FOREIGN = 5                # neither of C16's own nets (2 and 4)

# The two working points. 0.5 is deliberately avoided: it is the constant the
# nudger used to hard-code, so at 0.5 the two conventions AGREE and every test
# here would pass vacuously.
DVS_BIG = 0.8              # grader radius 0.40 vs the old nudger's 0.25
DVS_SMALL = 0.3            # the shipped CLI default; 0.15 vs 0.25, sign flipped
OLD_NUDGER_RADIUS = 0.5 / 2.0


def _repair(pcb, dvs, path=BOARD):
    """The 10-POSITIONAL construction every test in this family uses. `dvs` is
    argument 8, `default_via_size` -- the knob this issue is about, and the one
    #725's shape contract pins in that slot."""
    return _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, dvs, PREFIX, set())


def _rig(dvs, gaps, sizes=None, stubs=()):
    """A REAL board whose vias are replaced by ones parked at EXACT distances
    from a real cap pad's rect, so `_point_to_rect_dist` IS the requested gap
    and every threshold below is closed-form rather than measured.

    Returns (pcb, st, cap, rect). Vias sit on the pad rect's horizontal centre
    line, outside its right edge, on a foreign net.
    """
    pcb = parse_kicad_pcb(BOARD)
    probe = _repair(pcb, dvs)
    rect = probe.caps[CAP].pad_rects()[1]        # the net-2 pad
    cy = (rect[1] + rect[3]) / 2.0
    if sizes is None:
        sizes = [0.0] * len(gaps)
    pcb.vias[:] = [make_via(rect[2] + g, cy, net_id=FOREIGN, size=sz)
                   for g, sz in zip(gaps, sizes)]
    pcb.segments[:] = list(stubs)
    st = _repair(pcb, dvs)
    # Isolate the via channel: no foreign tracks, and every cap sees every via.
    st.cap_segs = {k: [] for k in st.caps}
    st.cap_vias = {k: st.vias for k in st.caps}
    return pcb, st, st.caps[CAP], rect


def _nudge(st, pcb, **kw):
    """Drive the real pass, capturing what it printed. The PRINTED OUTPUT is
    half the evidence for #732: above 0.5 the old build printed nothing at all,
    which is what made the failure silent."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        moves, segs = nudge_vias_for_unresolved(st, pcb, CLEAR, **kw)
    return moves, segs, buf.getvalue()


class TestOneRadiusRule(unittest.TestCase):
    """There is exactly one implementation of "how wide is this via"."""

    def test_a_size_zero_via_resolves_to_the_operators_default(self):
        pcb = parse_kicad_pcb(BOARD)
        pcb.vias[0].size = 0.0
        st = _repair(pcb, DVS_BIG)
        v, t = pcb.vias[0], st.vias[0]
        # ON THE BRANCH, both halves:
        self.assertFalse(v.size, 'the via carries a size -- the fallback branch '
                                 'is not taken and this test is vacuous')
        self.assertNotAlmostEqual(
            DVS_BIG / 2.0, OLD_NUDGER_RADIUS, places=9,
            msg='at default_via_size 0.5 the old hard-coded fallback EQUALS the '
                'resolver, so this test could not fail')
        self.assertAlmostEqual(st.via_radius(v), DVS_BIG / 2.0, places=12)
        # the tuple and the identity map are built from that same number
        self.assertAlmostEqual(t[3], st.via_radius(v) + CLEAR, places=12)
        self.assertAlmostEqual(st._via_radius_by_id[id(t)][1], st.via_radius(v),
                               places=12)
    # MUTATION: `via_radius` back to `(via.size or 0.5) / 2.0` -> reads 0.25,
    # not 0.40, and all three assertions fail.

    def test_a_via_with_a_REAL_size_ignores_the_default(self):
        """NEGATIVE CONTROL for the test above: the operator's belief must not
        override a size the board actually carries."""
        pcb = parse_kicad_pcb(BOARD)
        pcb.vias[0].size = 0.4
        for dvs in (DVS_SMALL, DVS_BIG):
            st = _repair(pcb, dvs)
            self.assertAlmostEqual(st.via_radius(pcb.vias[0]), 0.2, places=12,
                                   msg='default_via_size %s leaked into a via '
                                       'that declares its own size' % dvs)
    # MUTATION: make `via_radius` return `default_size / 2` unconditionally.

    def test_the_module_has_exactly_ONE_spelling_of_a_via_radius(self):
        """A source guard. The bug was not a wrong number, it was N numbers."""
        src = inspect.getsource(FC)
        # ON THE BRANCH: the resolver exists at all.
        self.assertIn('def via_radius', src)
        lines = src.split(chr(10))
        for gone, where in ((' (v.size or 0.5)', 'the nudger validation/offender sites'),
                            ('(ov.size or 0.5)', 'the foreign-via sites'),
                            ('max(1e-3, v.size / 2.0)', 'the #313 connectivity tolerance'),
                            ('keepout - st._item_reach', 'the locked-part warning')):
            # Report the offending LINES, not the whole 130KB module: an
            # assertNotIn on a source-sized haystack prints the haystack, and
            # one failure produced 393KB of output.
            hits = ['%d: %s' % (i + 1, l.strip())
                    for i, l in enumerate(lines) if gone in l]
            self.assertEqual(
                hits, [],
                '%s still resolves a via radius its own way -- %s'
                % (where, '; '.join(hits)))
    # MUTATION: re-introduce any one of those spellings -> its assertNotIn fires.


class TestNudgerSeesWhatTheGraderFlags(unittest.TestCase):
    """The offender test and the grader must agree about WHICH via offends."""

    def test_above_the_old_fallback_the_nudger_can_SEE_the_flagged_via(self):
        """The silent failure. Grader keep-out 0.500, old nudger 0.350; a via
        at 0.420 sits in the band only the resolved radius can see."""
        pcb, st, cap, rect = _rig(DVS_BIG, [0.42, 1.50])
        near = pcb.vias[0]
        # ON THE BRANCH, both halves:
        self.assertGreater(
            st.graze_penalty(CAP, cap, cap.x, cap.y, cap.rot), FC.EPS,
            'the grader does not flag this cap, so the offender loop never runs')
        self.assertTrue(
            OLD_NUDGER_RADIUS + CLEAR <= 0.42 < st.via_radius(near) + CLEAR,
            'the via is not in the band that separates the two conventions '
            '(old %.3f, resolved %.3f)' % (OLD_NUDGER_RADIUS + CLEAR,
                                           st.via_radius(near) + CLEAR))
        moves, _segs, out = _nudge(st, pcb, max_shift=4.0)
        # The SILENCE is the headline symptom, so assert it first: asserting the
        # move count first shadows this one, and then no mutation can ever make
        # the printed output the reported failure.
        self.assertIn('via-nudge:', out,
                      'the pass printed NOTHING about a cap it reports '
                      'unresolved -- the silent failure #732 is about')
        self.assertEqual(len(moves), 1, 'expected exactly the 0.42 via to move')
        self.assertAlmostEqual(moves[0][0], rect[2] + 0.42, places=6)
        # Where it LANDED, not just which via moved. moves[0][0..1] is the OLD
        # position, so without this nothing checks the destination -- and the
        # destination is what `valid_via_pos` prices with its own `vr`.
        gap = FC._point_to_rect_dist(near.x, near.y, rect[:4])
        self.assertGreaterEqual(
            gap, st.via_radius(near) + CLEAR - 1e-9,
            'the via landed %.4fmm from the pad, inside its resolved keep-out '
            '%.4fmm -- valid_via_pos is pricing the barrel at a constant'
            % (gap, st.via_radius(near) + CLEAR))
        self.assertLess(
            gap, st.via_radius(near) + CLEAR + 0.30,
            'the via was pushed far beyond its keep-out, so this assertion '
            'would hold for a much smaller radius too and proves little')
        # NEGATIVE CONTROL: the via at 1.50 is clear under BOTH conventions and
        # is still left alone.
        self.assertAlmostEqual(pcb.vias[1].x, rect[2] + 1.50, places=6,
                               msg='a genuinely clear via was moved')
    # MUTATION: the offender loop back to `(v.size or 0.5) / 2.0` -> moves == []
    # and `out` is '' (measured at HEAD: exactly that).

    def test_below_it_a_PHANTOM_via_is_no_longer_chased(self):
        """The shipped CLI default. Grader keep-out 0.250, old nudger 0.350: a
        via at 0.300 is a phantom the old build relocated anyway."""
        pcb, st, cap, rect = _rig(DVS_SMALL, [0.30, 0.02])
        phantom = pcb.vias[0]
        # ON THE BRANCH, three guards:
        self.assertGreater(
            st.graze_penalty(CAP, cap, cap.x, cap.y, cap.rot), FC.EPS,
            'no cap is unresolved, so the offender loop never runs')
        self.assertTrue(
            st.via_radius(phantom) + CLEAR <= 0.30 < OLD_NUDGER_RADIUS + CLEAR,
            'the phantom is not in the band only the OLD fallback could see')
        self.assertLessEqual(
            st.via_penalty(cap, cap.x, cap.y, cap.rot,
                           [st.vias[0]], ref=CAP), FC.EPS,
            'the grader DOES charge the 0.30 via -- it is not a phantom, and '
            'this test is measuring the wrong thing')
        moves, _segs, _out = _nudge(st, pcb, max_shift=4.0)
        # NEGATIVE CONTROL is the assertion itself: the real offender still moves.
        self.assertEqual(len(moves), 1,
                         'expected only the 0.02 via (the real offender) to move')
        self.assertAlmostEqual(moves[0][0], rect[2] + 0.02, places=6,
                               msg='the phantom moved instead of the real offender')
    # MUTATION: the offender loop back to `(v.size or 0.5) / 2.0` -> len(moves)
    # becomes 2 and the phantom is relocated (measured at HEAD).

    def test_the_offender_loop_RESOLVES_the_radius_through_st(self):
        """Isolate the OFFENDER LOOP specifically.

        A foreign TRACK makes the cap unresolved, so the loop runs -- but the
        only via is 40mm away, so it offends nobody and neither `valid_via_pos`
        nor `connector_clear` is ever entered. Any call to the resolver
        therefore came from the offender test itself. Priced at a constant, the
        resolver is never called at all, and a cap unresolved because of a
        RAISED via requirement would yield an empty offender list: the pass
        reports it unresolved forever and prints nothing.
        """
        pcb = parse_kicad_pcb(BOARD)
        probe = _repair(pcb, DVS_BIG)
        rect = probe.caps[CAP].pad_rects()[1]
        cy = (rect[1] + rect[3]) / 2.0
        # A foreign track biting into the cap pad's keep-out band.
        bite = make_seg(rect[2] + 0.05, cy - 2.0, rect[2] + 0.05, cy + 2.0,
                        width=0.1, layer='F.Cu', net_id=FOREIGN)
        pcb.segments[:] = [bite]
        pcb.vias[:] = [make_via(rect[2] + 40.0, cy, net_id=FOREIGN, size=0.0)]
        st = _repair(pcb, DVS_BIG)
        st.cap_vias = {k: st.vias for k in st.caps}
        cap = st.caps[CAP]
        # ON THE BRANCH, three guards: the loop only runs on an unresolved cap,
        # only reaches the resolver if a via is in the list, and the via must
        # NOT be an offender or a validator would run and muddy the count.
        unresolved = [r for r in st.caps
                      if st.graze_penalty(r, st.caps[r], st.caps[r].x,
                                          st.caps[r].y, st.caps[r].rot) > FC.EPS]
        self.assertIn(CAP, unresolved,
                      'the track does not make %s unresolved, so the offender '
                      'loop never runs' % CAP)
        self.assertTrue(pcb.vias, 'no vias -- the inner loop never executes')
        self.assertLessEqual(
            st.via_penalty(cap, cap.x, cap.y, cap.rot, st.vias, ref=CAP),
            FC.EPS, 'the far via grazes the cap, so a validator will run and '
                    'this test no longer isolates the offender loop')
        calls, real = [0], st.via_radius

        def counting(v):
            calls[0] += 1
            return real(v)
        st.via_radius = counting
        moves, segs, _out = _nudge(st, pcb)
        self.assertEqual((moves, segs), ([], []),
                         'a via 40mm away should offend nobody')
        self.assertGreater(calls[0], 0,
                           'the offender loop never consulted via_radius -- it '
                           'is pricing the barrel at a constant')
    # MUTATION: the offender loop back to a hard-coded fallback -> calls == 0.


class TestTheConnectivityTolerance(unittest.TestCase):
    """#313: copper terminating within the VIA BODY is joined through it and
    needs a connector on its layer when the via moves."""

    def _stub_rig(self, size):
        pcb, st, cap, rect = _rig(DVS_BIG, [0.42], sizes=[size])
        v = pcb.vias[0]
        near = make_seg(v.x, v.y - 0.30, v.x, v.y - 0.10,
                        width=0.2, layer='F.Cu', net_id=FOREIGN)
        far = make_seg(v.x, v.y + 0.90, v.x, v.y + 1.40,
                       width=0.2, layer='B.Cu', net_id=FOREIGN)
        pcb.segments[:] = [near, far]
        return pcb, st, v

    def test_a_size_zero_vias_stub_still_gets_its_connector(self):
        pcb, st, v = self._stub_rig(0.0)
        # ON THE BRANCH: 0.10mm is inside the resolved body but far outside the
        # 1e-3 the old expression collapsed to.
        self.assertTrue(1e-3 < 0.10 < st.via_radius(v),
                        'the stub end is not in the band that separates the two '
                        'tolerances (resolved radius %.3f)' % st.via_radius(v))
        moves, segs, _out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(len(moves), 1, 'the via did not move; nothing to prove')
        layers = {s['layer'] for s in segs}
        self.assertIn('F.Cu', layers,
                      'the via was relocated with NO connector back to its stub '
                      '-- the #313 open this tolerance exists to prevent')
        # NEGATIVE CONTROL: a same-net stub 0.90mm away is genuinely out of
        # reach of the barrel and must still get nothing.
        self.assertNotIn('B.Cu', layers,
                         'a stub far outside the via body was given a connector')
    # MUTATION: `tol` back to `max(1e-3, v.size / 2.0)` -> conn_layers is EMPTY,
    # `all([])` is True so the move still happens, and segs loses F.Cu.

    # NOT COVERED BEHAVIOURALLY, and here is the full list rather than a silent
    # gap. A mutation run reverted each converted site individually with a
    # guard-evading spelling; these are the ones no test noticed:
    #
    #   * `connector_clear`'s foreign-via term
    #   * `valid_via_pos`'s foreign-via term (`via_rad(ov)`)
    #
    # `valid_via_pos`'s OWN `vr` was in that list too and is now covered, by
    # asserting where the via LANDS rather than only which via moved.
    #
    # For the two that remain: the foreign-via terms only bind when a FOREIGN
    # via sits close enough to matter, and every fixture here parks its foreign
    # vias against the cap pad rather than against each other. I tried to cover
    # `connector_clear`'s and the rig was VACUOUS -- `segs` came back empty, so
    # the assertion loop never iterated.
    #
    # The reason is structural, not laziness. For a foreign via of the same
    # radius r, `valid_via_pos` requires centre-to-centre >= r + r + clearance
    # while `connector_clear` requires point-to-segment >= r + hw + clearance.
    # With r 0.4 and a 0.2mm connector that is 0.9mm versus 0.6mm, so
    # valid_via_pos is STRICTLY stricter: any blocker close enough to trip the
    # connector test has already rejected the candidate position. The term can
    # only bind against the connector's MIDDLE, and a connector is at most
    # max_shift long, so its midpoint sits within half a shift of the old via
    # position -- where the moving via already was, i.e. somewhere that was
    # legal by construction.
    #
    # It is therefore guarded by SPELLING only
    # (test_the_module_has_exactly_ONE_spelling_of_a_via_radius). Filed rather
    # than faked. The same holds for the locked-part warning's `_rec is None`
    # branch, which is unreachable while every tuple comes from __init__.

    def test_a_via_with_a_real_size_is_priced_exactly_as_before(self):
        """The whole fix is gated on a falsy size. A real one must resolve to
        exactly `size / 2`, before and after."""
        pcb, st, v = self._stub_rig(0.5)
        self.assertAlmostEqual(st.via_radius(v), 0.25, places=12)
        self.assertAlmostEqual(st.via_radius(v), OLD_NUDGER_RADIUS, places=12,
                               msg='a real-sized via must be priced identically '
                                   'to the pre-#732 build')
    # MUTATION: make via_radius apply the default even when size > 0.


class TestTheDuckTypedPathStaysInert(unittest.TestCase):
    """`nudge_vias_for_unresolved` is public and the #370/#617 harnesses drive
    it with a stand-in `st` carrying no resolvers at all."""

    class _FakeCap:
        def __init__(self, rects):
            self._rects = list(rects)
            self.side = 'F'
            self.x = self.y = self.rot = 0.0

        def pad_rects(self, x=None, y=None, rot=None):
            return self._rects

    class _FakeSt:
        def __init__(self, rects, cap_cls):
            self.caps = {'C1': cap_cls(rects)}
            self.vias = []

        def graze_penalty(self, ref, cap, x, y, rot):
            return 1.0

    def test_the_fallback_is_the_value_the_nudger_always_used(self):
        """DRIVES the real function with a duck-typed `st`, so the fallback
        branch is actually executed. Asserting on `_via_radius` alone
        would leave `via_rad`'s fallback dead code that a `raise` could replace
        without any test noticing -- which is exactly what a mutation run found
        the first version of this test doing."""
        BAR = (0.2, 0.9, 1.8, 1.1, 2)      # cap pad bar; only +y escapes
        st = self._FakeSt([BAR], self._FakeCap)
        v = make_via(1.0, 1.4, net_id=3, size=0.0)
        pcb = make_pcb(vias=[v], board_info=BoardInfo(
            layers={}, board_bounds=(-5.0, -5.0, 15.0, 50.0),
            copper_layers=['F.Cu', 'B.Cu']),
            footprints={'C1': SimpleNamespace(layer='F.Cu', pads=[])})
        # ON THE BRANCH, both halves:
        self.assertIsNone(getattr(st, 'via_radius', None),
                          'the fake grew a resolver -- the fallback is never '
                          'exercised and this test is vacuous')
        self.assertFalse(v.size, 'the via declares a size, so the fallback '
                                 'branch is not taken')
        self.assertEqual(FC._UNREADABLE_VIA_SIZE, 0.5,
                         'the duck-typed fallback silently changed value; the '
                         'nudger hard-coded 0.5 at four sites before #732 and '
                         'this rename must not move it')
        moves, _segs, _out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(len(moves), 1, 'the fake st did not drive a move, so '
                                        'the fallback was never reached')
        # The relocated via must clear the bar by the FALLBACK radius: the bar
        # spans y in [0.9, 1.1], so a via at radius r must sit at least
        # r + CLEAR above 1.1.
        self.assertGreaterEqual(
            v.y, BAR[3] + OLD_NUDGER_RADIUS + CLEAR - 1e-9,
            'the nudger cleared the bar by %.3f, not by the fallback radius '
            '%.3f' % (v.y - BAR[3] - CLEAR, OLD_NUDGER_RADIUS))
        # NEGATIVE CONTROL: a via that DECLARES a size is priced off the via,
        # not off the fallback -- a wider one must be pushed further.
        st2 = self._FakeSt([BAR], self._FakeCap)
        v2 = make_via(1.0, 1.4, net_id=3, size=1.0)
        pcb2 = make_pcb(vias=[v2], board_info=BoardInfo(
            layers={}, board_bounds=(-5.0, -5.0, 15.0, 50.0),
            copper_layers=['F.Cu', 'B.Cu']),
            footprints={'C1': SimpleNamespace(layer='F.Cu', pads=[])})
        moves2, _s2, _o2 = _nudge(st2, pcb2, max_shift=4.0)
        self.assertEqual(len(moves2), 1)
        self.assertGreater(v2.y, v.y,
                           'a 1.0mm via was not pushed further than a size-0 '
                           'one, so the radius is not being read at all')
    # MUTATION: change UNREADABLE_VIA_SIZE -> the fallback assertion fires.
    # MUTATION: replace via_rad's fallback body with `raise` -> this test dies
    # (it is the only one that reaches that branch).


class TestTheLockedWarningReadsTheMap(unittest.TestCase):
    """The last arithmetic re-derivation of a radius in the file. It agrees
    with the map on every real via -- which is what licenses replacing it."""

    def test_the_arithmetic_and_the_map_agree_on_every_real_via(self):
        pcb = parse_kicad_pcb(BOARD)
        st = _repair(pcb, DVS_SMALL)
        # ON THE BRANCH:
        self.assertTrue(st.vias, 'the board has no vias to compare')
        self.assertEqual(len(st._via_radius_by_id), len(st.vias),
                         'not every tuple is registered, so the map is not the '
                         'authority this test claims it is')
        for t in st.vias:
            self.assertAlmostEqual(
                t[3] - st._item_reach(st.via_floor(t[2])),
                st._via_radius_by_id[id(t)][1], places=12,
                msg='the old subtraction and the map disagree at %r' % (t,))
    # MUTATION: none kills this one -- it is the MEASUREMENT that licenses the
    # locked-warning change (the two forms are numerically identical on a real
    # board, so the swap is inert). The source guard in TestOneRadiusRule is
    # what stops the arithmetic coming back.


class TestTheGradersStillReadTheBoard(unittest.TestCase):
    """MEASURED DECISION, pinned so nobody "fixes" it later.

    KiCad honours a declared via size literally. On two boards identical but
    for one via's size, with a same-net stub end 0.100mm off its centre,
    check_connected's KiCad refill cross-check reported 0 unconnected links at
    `(size 0.5)` and 1 at `(size 0)`. A zero-diameter barrel joins nothing, so
    a grader that substituted `--default-via-size` would report CONNECTED what
    KiCad reports OPEN. The operator's belief governs PLACEMENT, not grading.
    """

    def test_the_grader_mirror_credits_a_size_zero_barrel_at_ZERO(self):
        """BEHAVIOURAL, not a string match.

        An earlier version of this test asserted only
        `'defaults.via_radius' not in source`, which such a mutant defeats with
        `from routing_defaults import via_radius as _vr`, and which would pass
        just as well on a build that hard-coded 0.5 in the grader by hand. It
        pinned a spelling, not the decision.

        `connectivity.endpoint_reaches_via` is the right thing to pin: it is
        public, its docstring explicitly documents itself as the mirror of
        check_connected's via credit, and it is the function a well-meaning
        reader would "fix". Its contract is that the credit is the BARREL
        radius plus the copper's own radius -- so for a via the board sizes at
        0, the barrel contributes nothing and only the copper's own reach and
        the coincidence floor remain.
        """
        import connectivity
        from connectivity import COINCIDENCE_TOL, endpoint_reaches_via
        LAY = ['F.Cu', 'B.Cu']
        big = make_via(0.0, 0.0, net_id=7, size=0.8)
        zero = make_via(0.0, 0.0, net_id=7, size=0.0)
        # A zero-radius probe: the credit is then the barrel alone.
        # 0.25: inside the 0.8 barrel (0.40), far outside the 0.02 floor,
        # and strictly inside a 0.6 invented diameter (0.30) so a mutant
        # that substitutes one is caught. NOT 0.30 -- that equals such a
        # mutant's radius exactly and the strict `<` lets it through.
        d = 0.25
        # ON THE BRANCH: the same probe IS credited against a real barrel, so a
        # False below is the barrel's absence and not a broken fixture.
        self.assertTrue(endpoint_reaches_via(d, 0.0, 0.0, big, LAY, LAY),
                        'a 0.8mm barrel does not reach 0.25mm; the fixture is '
                        'not measuring the barrel')
        self.assertGreater(d, COINCIDENCE_TOL,
                           'the probe is inside the coincidence floor, so this '
                           'would pass whatever the radius is')
        self.assertFalse(
            endpoint_reaches_via(d, 0.0, 0.0, zero, LAY, LAY),
            'a via the board sizes at 0 was credited with 0.25mm of barrel '
            'reach. KiCad honours a declared size literally -- (size 0) is a '
            'via_diameter DRC error there and that barrel joins nothing -- so '
            'inventing a diameter here reports CONNECTED what KiCad reports '
            'OPEN, the direction that hides an open. --default-via-size is an '
            'operator belief about a board being PLACED; it is not licence for '
            'a grader to disagree with KiCad.')
        # And the source of that number is still the board, not a constant.
        self.assertNotIn('_UNREADABLE_VIA_SIZE', inspect.getsource(connectivity),
                         'connectivity now imports the placement pass fallback')
    # MUTATION: give connectivity.endpoint_reaches_via a non-zero fallback for a
    # size-0 via (e.g. restore `(getattr(via,'size',0.6) or 0.6)`) -> the
    # assertFalse fires.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """Every changed expression is gated on a falsy size, and no tracked board
    has one -- so the whole change is provably inert on the corpus. Asserted
    rather than narrated."""

    def test_no_tracked_board_carries_a_via_the_fix_could_touch(self):
        import re
        import subprocess
        # git ls-files, NOT a glob: eleven boards in kicad_files/ are gitignored
        # build products (interf_u_*, sonde_u_*, fanout_output*, ...). Globbing
        # made this test pass only on a machine that had run the examples and
        # FAIL on a fresh clone -- measured, 602 tokens vs the 1520 a populated
        # tree sees.
        try:
            out = subprocess.check_output(
                ['git', 'ls-files', 'kicad_files/*.kicad_pcb'],
                cwd=_ROOT, stderr=subprocess.DEVNULL).decode()
        except Exception as exc:                       # no git, no answer
            raise unittest.SkipTest('git ls-files unavailable: %s' % exc)
        boards = [os.path.join(_ROOT, ln) for ln in out.splitlines()
                  if ln.strip()]
        pat = re.compile(r'\(size\s+([0-9.]+)\s*\)')
        total = zero = 0
        for path in boards:
            with io.open(path, encoding='utf-8', errors='replace') as fh:
                for m in pat.finditer(fh.read()):
                    total += 1
                    if float(m.group(1)) <= 0:
                        zero += 1
        # ON THE BRANCH: the scan actually read the tracked corpus.
        self.assertGreater(len(boards), 15,
                           'git listed only %d tracked boards' % len(boards))
        self.assertGreater(total, 500,
                           'the scan found only %d size tokens across %d tracked '
                           'boards -- it is not reading the corpus'
                           % (total, len(boards)))
        self.assertEqual(zero, 0,
                         '%d tracked via(s) now have size <= 0, so the "provably '
                         'inert on the corpus" claim in the PR is no longer '
                         'true and must be re-measured' % zero)
    # MUTATION: add a `(size 0)` via to any tracked board -> this fails, which
    # is the point: the inertness claim stops being true and must be restated.


if __name__ == '__main__':
    unittest.main(verbosity=2)
