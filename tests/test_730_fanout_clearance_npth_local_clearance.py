#!/usr/bin/env python3
"""#730: an NPTH hole pad's own `local_clearance` must reach every
copper-to-hole decision in py_placer, exactly as check_drc grades it.

KiCad holds copper off a mounting hole at the hole pad's own `(clearance ...)`
when that exceeds the fab floor, and `check_drc` mirrors it -- every
copper-less drilled pad is filed with a required clearance of
`max(npth_clr, lc)` (check_drc.py:2733, #505). The router has honoured that
since #326/#505 (obstacle_map.py:1759-1788). py_placer read
`pad.local_clearance` for hole geometry NOWHERE.

THE TWO ARMS HAVE DIFFERENT SHAPES, and this file's structure follows from it:

    TRACK arm (check_drc.py:2733)   max(npth_clr, lc)                  a MAX
    VIA arm   (check_drc.py:2851)   lc if lc > npth_clr else clearance  a STEP

Five sites, all fixed, all covered here:

  A  _Repair.__init__'s foreign-pad loop -- the cap keep-out rect      TRACK
  B  valid_via_pos's copper-to-hole gate                               VIA
  C  connector_clear's copper-to-hole gate                             TRACK
  D  the two NPTH floors this pass carries -- DELIBERATE, not unified
  E  legality.PartPads.__init__'s hole inflation                       TRACK

MEASURED, on this tree. Every number was produced by running the engine, and
every bound was crossed by sweeping just inside and just outside each end.

  Site A, ulx3s AUDIO1 (hd/2 0.850, lc 0.400, npth_floor 0.200):
        --clearance 0.10   1.0500 -> 1.2500   (check_drc wants 1.2500)
        --clearance 0.25   1.1000 -> 1.2500   (check_drc wants 1.2500)

  Site B, the #737 rig at clearance 0.10 / max_shift 0.55:
        gap 0.300 lc 0.40   0 moves, "no clear spot"    <- the fix
        gap 0.300 lc 0.00   1 move -> (3.4500, 3.0000)  <- not a board floor
        gap 0.500 lc 0.40   1 move                      <- no over-rejection

  Site B, THE STEP, at gap 0.120 (clearance 0.10, npth_step 0.20). The charge
  jumps from `clearance` to `lc` across ONE TENTH OF A MICRON, which is what a
  step means and what a max cannot do:
        lc 0.10 / 0.15 / 0.19 / 0.1999 / 0.2000  -> 1 move (charged clearance)
        lc 0.2001 / 0.21 / 0.25 / 0.40           -> 0 moves (charged lc)
  and the charge itself, at lc 0.25: gap 0.230 refuses, gap 0.250 moves.

  Site C, connector gate (via 0.5/0.2, stub width 1.0 -> hw 0.50, hole 1.0):
        seg_dist 1.300 lc 0.40   0 moves                <- the fix
        seg_dist 1.500 lc 0.40   1 move                 <- control
        seg_dist 1.300 lc 0.00   1 move                 <- not a board floor
        seg_dist 1.300 lc 0.15   1 move                 <- the MAX's low side
        seg_dist 1.150 lc 0.00   0 moves                <- the npth_clr FLOOR
        bound: 1.350 refuses, 1.400 moves == hr + hw + lc

  Site E, one footprint carrying three holes at lc 0.00 / 0.40 / 0.15:
        --clearance 0.10   [0.60, 0.60, 0.60] -> [0.60, 0.80, 0.60]
        --clearance 0.25   [0.50, 0.50, 0.50] -> [0.50, 0.65, 0.50]

CORPUS-INERT, and this file asserts that rather than narrating it. Over the 22
boards `run_utils.corpus_boards()` returns (37 copper-less drilled pads), only
ulx3s carries a binding override, and its nearest foreign copper misses the new
requirement by 0.010mm.

THE BATTERY: `tests/mutate_730.py`, 28 rows -- 25 killed, 3 survived (all three
expected and recorded with their reason), 0 broken. It ships rather than a
number in this docstring because two reviewers of the #746 branch tried to
re-derive its kill counts from row NAMES and both got the wrong answer. It
found four things this file was wrong about, and all four are now recorded at
the arm rather than smoothed over: two arms that did not exist (the `getattr`
and the slot had no engine-side gate at all), one arm crediting itself with a
kill it cannot make, and one source guard that reported green against the very
mutation it was written for.

Conventions, from #697/#725 and CLAUDE.md:

  * REAL parser dataclasses and REAL boards. `_Repair.__init__` reads
    courtyards and locked refs from the file on disk.
  * Every assertion names the single-line MUTATION that must kill it, written
    AFTER `tests/mutate_730.py` ran and carrying the measured kill count.
  * Assert you are ON the branch before asserting about it. Acute here: an
    `lc` at or below `max(npth_floor, clearance)` changes NOTHING at any of the
    five sites, so a rig whose lc is not strictly above the floor is vacuous.

NOT COVERED, deliberately, and named so the next reader does not conclude the
rule is now complete:

  * `PartPads` still cannot see the BOARD's declared min_hole_clearance -- it
    takes `(fp, clearance)` with no board pointer. Asserted as a signature
    property below rather than left to be discovered.
  * An overridden mounting hole ON A MOVABLE CAP is modelled by neither site A
    (the foreign-pad loop never runs for a cap footprint) nor site E.
    Pre-existing, untouched here.
  * check_drc grades two further hole classes this does not touch: a PLATED pad
    whose ring does not span its drill (#441), and any plated pad with lc > 0.
    `_Repair` models plated pads as copper rects with no drill keep-out.

This drives no CLI and routes nothing, but it does not run purely in-process
either: `run_utils.corpus_boards()` shells out ONCE, for `git ls-files`, which
is milliseconds. That is the whole reason the opt-out below is needed and also
the whole reason it is legitimate -- `run_all` classifies by grepping the
SOURCE for `run_utils`, so adopting that helper silently drops a file out of
`--fast`, which is how test_725 was once bucketed as integration and skipped
there. Drop the marker if an arm ever drives a real chain.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
import math
import os
import re
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
from check_drc import _pad_has_no_copper, point_to_pad_distance
from obstacle_map import resolve_hole_clearance
from synth import make_pad, make_pcb, make_seg, make_via
from placement import fanout_clearance as FC
from placement import legality as LEG
from placement.fanout_clearance import _Repair, nudge_vias_for_unresolved
from placement.legality import PartPads, build_part_pads

ULX3S = os.path.join(_ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
WATCHY = os.path.join(_ROOT, 'kicad_files', 'watchy.kicad_pcb')

# ---- ulx3s's AUDIO1, the board the issue measures on ----------------------
AUDIO1_DRILL = 1.700          # the two np_thru_hole pads, the ONLY two on it
AUDIO1_LC = 0.400             # their `(clearance 0.4)`
AUDIO1_HR = AUDIO1_DRILL / 2.0
# check_drc's requirement from the hole CENTRE, equal at both clearances
# because `lc` outranks `max(clearance, NPTH_TO_TRACK_CLEARANCE)` in each.
CHECKER_WANTS = AUDIO1_HR + AUDIO1_LC                                # 1.2500
# What the pass modelled BEFORE #730: `hd/2 + max(npth_floor, clearance)`.
OLD_KEEPOUT = {0.10: AUDIO1_HR + 0.20, 0.25: AUDIO1_HR + 0.25}
# Tightest foreign copper on the corpus, hole WALL to pad copper. This is the
# number that makes site A corpus-inert, and it is the tightest margin in the
# whole change: 1.260 from the centre against a 1.250 requirement.
AUDIO1_NEAREST_COPPER = 0.4100                                       # R60.2

# ---- the ONE hand-mirrored engine constant -------------------------------
# `H2H_PAD` is a function-local inside nudge_vias_for_unresolved. Since #756
# its VALUE is `resolve_drill_floors`' answer rather than a literal, so this
# mirror tracks that resolver's result on a PROJECT-LESS board -- which is
# what every rig in this file builds -- and the guard below CALLS it rather
# than grepping for `= 0.45`. Kept hand-written on purpose: deriving it at
# import time would make every threshold here self-fulfilling.
# `NPTH_FLOOR` is NOT mirrored -- it is imported, right here -- and saying so
# matters, because a reader who believes it is mirrored will look for a guard
# that should not exist.
H2H_PAD = 0.45
NPTH_FLOOR = defaults.NPTH_TO_TRACK_CLEARANCE                        # imported

# ---- the synthetic via rigs (site B/C), test_737's geometry ---------------
CLEAR = 0.10
MAX_SHIFT = 0.55
V_SIZE, V_DRILL = 1.4, 0.3
VR = V_SIZE / 2.0                                                    # 0.70
H_DRILL = 1.0
HR = H_DRILL / 2.0                                                   # 0.50
VIA0 = (3.0, 3.0)
LANDING = (3.45, 3.0)
ALL_CANDIDATES = ((3.45, 3.0), (3.50, 3.0), (3.55, 3.0))
WALLS = [(0.0, 0.0, 2.625, 8.0, 2), (0.0, 3.90, 8.0, 8.0, 2),
         (0.0, 0.0, 8.0, 2.10, 2)]
# copper-edge-to-hole-WALL gap at the first candidate:
#   XH - LANDING.x - HR - VR
XH_REFUSED = 4.95      # gap 0.300, inside lc 0.40
XH_MOVES = 5.15        # gap 0.500, clear of lc 0.40
# THE STEP rig: a gap strictly between `clearance` and an lc at/below npth_clr.
XH_STEP = 4.77         # gap 0.120  (0.020 over CLEAR, 0.030 under LC_LOW)
LC_LOW = 0.15          # <= npth_clr -> the checker charges `clearance`
LC_HIGH = 0.25         # >  npth_clr -> the checker charges `lc`
STEP_MOVES, STEP_REFUSES = 0.2000, 0.2001     # the measured threshold, 1e-4 mm
XH_HIGH_REFUSES, XH_HIGH_MOVES = 4.88, 4.90   # the CHARGE bound at LC_HIGH

# ---- the connector rig (site C) ------------------------------------------
C_V_SIZE, C_V_DRILL = 0.5, 0.2
C_VR = C_V_SIZE / 2.0                                                # 0.25
C_STUB_W = 1.0
C_HW = C_STUB_W / 2.0                                                # 0.50
C_MAX_SHIFT = 0.15
C_LANDING = (3.05, 3.0)
# the along-axis candidates C_MAX_SHIFT actually reaches. 3.05 is the one
# valid_via_pos accepts, so it is the candidate the connector gate decides.
ALL_CANDIDATES_C = ((3.05, 3.0), (3.10, 3.0), (3.15, 3.0))
# The cap footprint sits on B.Cu so its rects cannot gate the F.Cu connector
# (connector_clear skips a rect whose cap layer differs), while the SAME rects
# still box the via -- valid_via_pos has no layer gate. That separation is what
# makes the connector gate the binding one, and the arms attributable.
C_RECTS = [(2.0, 2.9, 2.68, 3.1, 2), (2.0, 3.36, 8.0, 8.0, 2),
           (2.0, 0.0, 8.0, 2.64, 2)]
XH_C_REFUSED = 4.35    # seg_dist 1.300, inside HR + C_HW + 0.40 = 1.400
XH_C_MOVES = 4.55      # seg_dist 1.500
XH_C_FLOOR = 4.20      # seg_dist 1.150: over a `clearance` floor (1.10),
                       # under the `npth_clr` one (1.20)
XH_C_BOUND_REFUSES, XH_C_BOUND_MOVES = 4.40, 4.45     # 1.350 / 1.400

# ---- site E ---------------------------------------------------------------
E_LCS = (0.00, 0.40, 0.15)          # below / above / below the 0.20 floor
E_RADII = {0.10: [0.60, 0.80, 0.60], 0.25: [0.50, 0.65, 0.50]}
E_RADII_SILK = {0.10: [0.60, 0.60, 0.60], 0.25: [0.50, 0.50, 0.50]}


class _FakeCap:
    """Minimal stand-in for _Cap: fixed pad rects, never moves."""

    def __init__(self, rects, side='F'):
        self._rects = list(rects)
        self.side = side
        self.x = self.y = self.rot = 0.0

    def pad_rects(self, x=None, y=None, rot=None):
        return self._rects


class _FakeSt:
    """The duck-typed `st` the #370/#617 harnesses pass: no resolvers at all,
    so every requirement in the nudger falls back to the flat scalar.

    #: `npth_floor` is 0.60 here, NOT the 0.40 test_737/test_750 use. Those
    #: picked 0.40 to catch `max(clearance, getattr(st, 'npth_floor', 0.0))`
    #: reading 0.0; here 0.40 is ALSO the real ulx3s override this file is
    #: about, and a reader could not tell the two numbers apart. 0.60 is above
    #: every gap any rig here uses (largest 0.500), so a gate that wrongly
    #: reads it still refuses everything and every control arm goes red.
    """

    npth_floor = 0.60

    def __init__(self, rects, side='F'):
        self.caps = {'C1': _FakeCap(rects, side)}
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


def _npth(x, y, lc, drill=H_DRILL, ref='H1', net=0):
    p = make_pad(net_id=net, x=x, y=y, ref=ref, num='H1',
                 size_x=drill, size_y=drill, shape='circle',
                 layers=['F.Mask', 'B.Mask'], drill=drill,
                 pad_type='np_thru_hole')
    p.local_clearance = lc
    return p


def _slot(x, y, lc, *, short=0.5, long_=2.5, net=0, ref='H1'):
    """A real `(drill oval short long)` NPTH pad -- a MILLED SLOT.

    `pad_drill_capsule` reads `drill_w`/`drill_h`, not size/shape: a pad with
    only `shape='oval'` set yields a zero-length capsule, and an arm built on
    one would pass while testing nothing.
    """
    p = _npth(x, y, lc, drill=long_, ref=ref, net=net)
    p.drill_w, p.drill_h = short, long_
    p.size_x, p.size_y = short, long_
    p.shape = 'oval'
    return p


def _via_board(xh, lc, *, hole_net=0, v_size=V_SIZE, v_drill=V_DRILL,
               stub_w=0.2, cap_layer='F.Cu'):
    v = make_via(VIA0[0], VIA0[1], net_id=3, size=v_size, drill=v_drill)
    stub = make_seg(2.0, 3.0, VIA0[0], VIA0[1], width=stub_w, layer='F.Cu',
                    net_id=3)
    hole = _npth(xh, 3.0, lc, net=hole_net)
    pcb = make_pcb(board_info=_bi(), vias=[v], segments=[stub],
                   footprints={'C1': SimpleNamespace(layer=cap_layer, pads=[])},
                   pads_by_net={hole_net: [hole]}, zones=[])
    return v, pcb


def _run_via(xh, lc, clear=CLEAR, **kw):
    v, pcb = _via_board(xh, lc, **kw)
    moves, segs, out = _nudge(_FakeSt(WALLS), pcb, clear, max_shift=MAX_SHIFT)
    return v, moves, segs, out


def _run_conn(xh, lc, clear=CLEAR):
    v, pcb = _via_board(xh, lc, v_size=C_V_SIZE, v_drill=C_V_DRILL,
                        stub_w=C_STUB_W, cap_layer='B.Cu')
    moves, segs, out = _nudge(_FakeSt(C_RECTS, side='B'), pcb, clear,
                              max_shift=C_MAX_SHIFT)
    return v, moves, segs, out


def _e_footprint():
    """One footprint carrying THREE NPTH holes with different overrides.
    Three, because one hole cannot distinguish per-hole from hoisted."""
    return SimpleNamespace(
        reference='H1', x=10.0, y=10.0, rotation=0.0, layer='F.Cu',
        pads=[_npth(10.0, 10.0 + 3.0 * i, lc) for i, lc in enumerate(E_LCS)])


def _npth_pads(pcb):
    """Every copper-less drilled pad, selected the way the ENGINE selects --
    check_drc's own predicate, never `pad_type == 'np_thru_hole'` (which misses
    a pad declaring no copper layer) and never a ref-name filter."""
    return [(ref, p) for ref, fp in pcb.footprints.items() for p in fp.pads
            if _pad_has_no_copper(p) and (p.drill or 0) > 0]


def _repair(path, clearance, pcb=None):
    """The 10-POSITIONAL construction every other test in this family uses.
    Calling it positionally is itself part of the #725 shape contract."""
    return _Repair(pcb if pcb is not None else parse_kicad_pcb(path), path,
                   clearance, 0.1, 0.55, 1.0, 2.0, 0.3, 'C', set())


def _npth_rect_halves(st):
    """The half-extent of every NPTH keep-out rect `_Repair` filed. Net -1 is
    the marker the engine stamps on them and nothing else uses."""
    return [round((t[2] - t[0]) / 2.0, 4) for t in st.foreign_pads if t[4] == -1]


class TestTheKeepOutRectHonoursTheHolesOwnOverride(unittest.TestCase):
    """Site A, on the board the issue measures: ulx3s's AUDIO1."""

    @classmethod
    def setUpClass(cls):
        cls.pcb = parse_kicad_pcb(ULX3S)

    def test_the_keep_out_lands_on_check_drcs_own_requirement(self):
        for clearance in (0.10, 0.25):
            with self.subTest(clearance=clearance):
                st = _repair(ULX3S, clearance, pcb=self.pcb)
                # ON THE BRANCH: the override must be STRICTLY above the floor
                # this site already charged, or the arm is vacuous -- `max`
                # with a smaller term changes nothing.
                floor = max(st.npth_floor, st.clearance)
                self.assertGreater(
                    AUDIO1_LC, floor + 1e-9,
                    'lc %.3f is not above max(npth_floor %.3f, clearance %.3f) '
                    '-- this arm cannot see the fix' % (AUDIO1_LC,
                                                        st.npth_floor,
                                                        st.clearance))
                # ...and this board declares NO min_hole_clearance, so the
                # floor is the bare fab floor and the arithmetic below is the
                # arithmetic in the docstring.
                self.assertAlmostEqual(st.npth_floor, NPTH_FLOOR, places=6)
                halves = _npth_rect_halves(st)
                self.assertEqual(len(halves), 2,
                                 'AUDIO1 has exactly two np_thru_hole pads and '
                                 'they are the only ones on this board')
                for half in halves:
                    self.assertAlmostEqual(half + st.clearance, CHECKER_WANTS,
                                           places=4)
                    self.assertGreater(half + st.clearance,
                                       OLD_KEEPOUT[clearance] + 1e-9,
                                       'the keep-out did not move off what this '
                                       'site modelled before #730')
    # MUTATION: site-A-reverted -> 1.0500 / 1.1000 instead of 1.2500. Measured:
    # 3 arms fire (this one twice, once per clearance subTest, plus the SLOT
    # arm). site-A-threshold-at-clearance does NOT fire here -- it needs a hole
    # whose lc is BELOW the fab floor, which is watchy's job.

    def test_watchy_is_the_NEGATIVE_control_and_a_real_one(self):
        """watchy carries eight GENUINE overrides that must change nothing:
        all 0.100, under the 0.20 fab floor, so `max` keeps the floor. A
        control with no override at all would prove much less -- it could not
        tell "the override was ignored" from "there was none"."""
        pcb = parse_kicad_pcb(WATCHY)
        pads = _npth_pads(pcb)
        # ON THE BRANCH: they really are overrides, and really are below.
        lcs = sorted({getattr(p, 'local_clearance', 0.0) or 0.0
                      for _r, p in pads})
        self.assertEqual(len(pads), 8)
        self.assertEqual(lcs, [0.1])
        self.assertLess(lcs[0], NPTH_FLOOR,
                        'watchy s override is no longer under the fab floor; '
                        'it has stopped being a negative control')
        for clearance in (0.10, 0.25):
            with self.subTest(clearance=clearance):
                st = _repair(WATCHY, clearance, pcb=pcb)
                halves = _npth_rect_halves(st)
                expect = round(0.75 / 2.0 + max(0.0, st.npth_floor - clearance),
                               4)
                self.assertEqual(sorted(set(halves)), [expect],
                                 'a sub-floor override moved a keep-out')
    # MUTATION: site-A-loses-the-fab-floor (`max(0.0, lc - clearance)`) and
    # site-A-threshold-at-clearance both shrink these sub-floor holes. Measured:
    # 2 arms each, this one and the attribute-less-pad arm -- and NOT the ulx3s
    # headline, whose lc is above the floor either way. This arm is the only
    # place a BELOW-floor override is graded.

    def test_the_NPTH_entry_still_carries_NO_floor(self):
        """The override must move the RECT, never become a `PadFloor`. The
        copper-to-hole rule is net-independent, so a floor could be amplified
        by a neighbouring pad's netclass and double-count the same millimetre
        -- which is exactly what the #725 note at that site forbids."""
        st = _repair(ULX3S, 0.10, pcb=self.pcb)
        idx = [i for i, t in enumerate(st.foreign_pads) if t[4] == -1]
        self.assertEqual(len(idx), 2)
        for i in idx:
            self.assertIsNone(st.foreign_pad_floors[i])
            self.assertEqual(st.foreign_pad_mf[i], 0.0)
            self.assertIsNone(
                st._fp_floor_by_id.get(id(st.foreign_pads[i])),
                'an NPTH tuple reached the id-keyed floor map, so a '
                'neighbouring netclass can now raise a net-independent rule')
        # ON THE BRANCH, and this is the assertion that makes the arm mean
        # something: ulx3s's model IS active (BAT1's two SMD pads carry
        # lc 0.700), so "the NPTH hole got no floor" is a real outcome and not
        # an artefact of an inert board.
        self.assertIsNotNone(
            st._floors,
            'ulx3s no longer activates PadClearanceModel, so this arm no '
            'longer shows that an ACTIVE model still gives the hole nothing')
    # MUTATION: site-A-lc-becomes-a-PadFloor -> the assertIsNone fires.
    # Measured: 1 arm, this one. Nothing else in the tree notices an NPTH
    # tuple acquiring a floor, which is why the arm is not merely a tidiness
    # check.

    def test_a_SLOT_drill_grows_EVERY_circle(self):
        """`pad_drill_circles` walks a slot as several circles. The grow must
        apply to each, and must come from the CIRCLE's diameter rather than
        from `pad.drill` -- which for an oval is the long axis."""
        # A real `(drill oval 3.0 1.0)`. pad_drill_capsule reads drill_w /
        # drill_h, NOT size/shape -- a pad with only `shape='oval'` set yields
        # ONE circle and the arm would pass while testing nothing.
        p = _npth(50.0, 50.0, AUDIO1_LC, drill=3.0)
        p.drill_w, p.drill_h = 3.0, 1.0
        p.size_x, p.size_y = 3.0, 1.0
        p.shape = 'oval'
        from kicad_parser import pad_drill_circles
        circles = list(pad_drill_circles(p))
        # ON THE BRANCH: a real slot, and the circle diameter is NOT p.drill.
        self.assertGreater(len(circles), 1, 'not a slot; this arm is vacuous')
        self.assertNotAlmostEqual(circles[0][2], p.drill, places=6)
        grow = max(0.0, max(NPTH_FLOOR, AUDIO1_LC) - CLEAR)
        want = round(circles[0][2] / 2.0 + grow, 4)
        halves = [round(hd / 2.0 + grow, 4) for _x, _y, hd in circles]
        self.assertEqual(sorted(set(halves)), [want])
    # MUTATION: none reachable -- this arm computes its expectation the way
    # the engine does, so it is a SHAPE check on pad_drill_circles and not a
    # gate on the engine's use of it. The arm below is what gates that.

    def test_a_SLOT_in_the_ENGINE_grows_from_the_CIRCLE_not_pad_drill(self):
        """The same trap, but through `_Repair` rather than beside it.

        `hr = hd / 2.0 + grow` and `hr = p.drill / 2.0 + grow` are IDENTICAL
        for every round hole, so no board in the corpus can tell them apart --
        `p.drill` on a slot is the LONG axis. A slot has to be injected, and
        the mutation battery reported this row as a survivor until it was.
        """
        import copy
        from kicad_parser import pad_drill_circles
        pcb = copy.deepcopy(self.pcb)
        ref, src = _npth_pads(pcb)[0]
        slot = copy.deepcopy(src)
        slot.drill = 3.0                 # the LONG axis: KiCad's back-compat
        slot.drill_w, slot.drill_h = 3.0, 1.0
        slot.size_x, slot.size_y = 3.0, 1.0
        slot.shape = 'oval'
        pcb.footprints[ref].pads = [slot]
        # ON THE BRANCH: a real slot, and the two spellings really do differ.
        circles = list(pad_drill_circles(slot))
        self.assertGreater(len(circles), 1, 'not a slot; this arm is vacuous')
        self.assertNotAlmostEqual(circles[0][2], slot.drill, places=6)
        st = _repair(ULX3S, CLEAR, pcb=pcb)
        grow = max(0.0, max(st.npth_floor, AUDIO1_LC) - CLEAR)
        want = round(circles[0][2] / 2.0 + grow, 4)
        halves = _npth_rect_halves(st)
        self.assertEqual(len(halves), len(circles),
                         'the slot did not become one keep-out per circle')
        self.assertEqual(sorted(set(halves)), [want])
        # ...and the CENTRES, which the radii alone throw away. Without this a
        # keep-out centred on the PAD rather than on each circle survives the
        # whole fanout test family -- measured by a review, which is how it got
        # here.
        got = sorted((round((x0 + x1) / 2.0, 4), round((y0 + y1) / 2.0, 4))
                     for x0, y0, x1, y1, net, _s in st.foreign_pads if net == -1)
        self.assertEqual(got, sorted((round(cx, 4), round(cy, 4))
                                     for cx, cy, _d in circles))
        self.assertNotAlmostEqual(want, slot.drill / 2.0 + grow, places=4,
                                  msg='the long-axis spelling would give the '
                                      'same answer here, so this arm cannot '
                                      'tell them apart')
    # MUTATION: site-A-long-axis-drill -> 1.8000 instead of 0.8000. 1 arm.

    def test_a_pad_carrying_NO_local_clearance_attribute_is_priced(self):
        """The `getattr`, and the only thing in this file that reaches it.

        The repo's convention (legality.py:1503-1509) is `getattr` and never a
        bare read, because the placement tests build duck-typed pad fakes that
        do not carry the field -- a hard read turns a missing attribute into a
        crash instead of the "no override" it means. Every pad the PARSER
        produces has it, so a real board cannot exercise this and the mutation
        battery reported the row as a survivor until this arm existed.
        """
        import copy
        pcb = copy.deepcopy(self.pcb)
        ref, src = _npth_pads(pcb)[0]
        # A genuine duck type, not a real Pad with the field deleted: `Pad`
        # declares `local_clearance` with a CLASS-level default, so `del` on
        # an instance falls straight back to it and `hasattr` stays True. The
        # ON-THE-BRANCH guard below caught exactly that.
        bare = SimpleNamespace(
            pad_type='np_thru_hole', layers=list(src.layers),
            drill=src.drill, drill_w=0.0, drill_h=0.0,
            hole_x=None, hole_y=None, rotation=0.0,
            global_x=src.global_x, global_y=src.global_y,
            size_x=src.size_x, size_y=src.size_y, net_id=0,
            local_x=src.local_x, local_y=src.local_y,
            rect_rotation=0.0, pad_number='H1', component_ref=ref)
        self.assertFalse(hasattr(bare, 'local_clearance'))
        pcb.footprints[ref].pads = [bare]
        st = _repair(ULX3S, CLEAR, pcb=pcb)          # must not raise
        want = round(bare.drill / 2.0
                     + max(0.0, st.npth_floor - CLEAR), 4)
        self.assertEqual(_npth_rect_halves(st), [want],
                         'a pad with no override attribute was not priced at '
                         'the bare fab floor')
    # MUTATION: site-A-getattr-dropped -> AttributeError, an ERROR rather than
    # a failure, which is why the battery counts an ERROR as a kill. Measured:
    # 1 arm, this one -- and the row reported SURVIVED until this arm existed,
    # because every pad the parser builds carries the field.


class TestARelocatedViaHonoursTheHolesOwnOverride(unittest.TestCase):
    """Site B, the VIA arm."""

    def test_a_via_inside_an_OVERRIDDEN_holes_keep_out_is_refused(self):
        v, moves, segs, out = _run_via(XH_REFUSED, AUDIO1_LC)
        # ON THE BRANCH, four ways:
        #  (i) the override outranks npth_clr, or the gate never fires;
        npth_clr = max(CLEAR, NPTH_FLOOR)
        self.assertGreater(AUDIO1_LC, npth_clr + 1e-9)
        # (ii) the landing sits between what the base gate charges and what
        #      the override charges, so ONLY the override can decide;
        gap = XH_REFUSED - LANDING[0] - HR - VR
        self.assertTrue(CLEAR + 0.04 < gap < AUDIO1_LC - 0.04,
                        'gap %.4f is not strictly inside (clearance, lc)' % gap)
        # (iii) every candidate THE WALLS ADMIT is inside the override, so
        #       `moves == []` is not "the budget ran out". Note what this does
        #       NOT say: the engine's spiral tries 176 candidates here, and
        #       `(3.05, 3.0)` is one the override gate would ACCEPT -- the
        #       walls refuse it first. The refusal is `walls AND override`, and
        #       the thing that makes it attributable to the override is the
        #       paired lc=0.00 control below, not this loop;
        for cx, _cy in ALL_CANDIDATES:
            self.assertLess(XH_REFUSED - cx - HR - VR, AUDIO1_LC + 1e-9)
        # (iv) and the landing is legal for the DRILL gate, so a refusal here
        #      cannot be the hole-to-hole test's.
        self.assertGreater(abs(XH_REFUSED - LANDING[0]),
                           V_DRILL / 2.0 + HR + H2H_PAD + 0.05)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
        self.assertEqual(segs, [])
        self.assertEqual((v.x, v.y), VIA0)
    # MUTATION: via-override-gate-deleted -> the via parks at (3.4500, 3.0000),
    # 0.3000 off a hole whose own (clearance 0.40) makes check_drc grade it.
    # Measured: 6 arms, 5 here and 1 in test_737's source guard (which counts
    # the gates).

    def test_the_same_geometry_with_NO_override_still_moves(self):
        """The arm without which the one above is not about #730 at all: a
        board-aware FLOOR would refuse this too. `_FakeSt.npth_floor` is 0.60,
        above this gap, so a gate that wrongly read it goes red here."""
        v, moves, segs, out = _run_via(XH_REFUSED, 0.0)
        self.assertEqual(len(moves), 1)
        self.assertAlmostEqual(v.x, LANDING[0], places=4)
        self.assertAlmostEqual(v.y, LANDING[1], places=4)
        self.assertIn('moved', out)
    # MUTATION: base-via-gate-reads-st-npth_floor -- `clearance` replaced by
    # `max(clearance, getattr(st, 'npth_floor', 0.0))`, the classic #617 shape.
    # Measured: 15 arms across this file and test_737.
    #
    # And a correction the battery forced. This arm does NOT kill
    # via-override-uses-the-BOARD-floor, which is what the note here first
    # claimed: with lc 0.00 the hole never enters the filtered override list,
    # so the gap helper is never consulted and a mutation INSIDE it cannot
    # run. That row is killed by the four CLEAR-of-the-override controls
    # instead. The arm still earns its place -- it proves the refusal above is
    # the override's and not a floor's -- but not by the route stated.

    def test_a_via_clear_of_the_override_still_moves(self):
        """No over-rejection: 0.10mm more room and the same override moves."""
        v, moves, segs, out = _run_via(XH_MOVES, AUDIO1_LC)
        gap = XH_MOVES - LANDING[0] - HR - VR
        self.assertGreater(gap, AUDIO1_LC + 1e-9)
        self.assertEqual(len(moves), 1)
        self.assertAlmostEqual(v.x, LANDING[0], places=4)
        self.assertEqual(len(segs), 1)
    # MUTATION: via-override-uses-the-BOARD-floor -> measured 4 arms, and ALL
    # FOUR are clear-of-the-override controls like this one. A file with only
    # refusal arms would not see it at all.

    def test_a_SLOT_override_hole_is_measured_along_its_whole_axis(self):
        """A milled slot is a CAPSULE, and the gate must measure to its axis.

        Every other override hole in this file is round, so `pad_drill_capsule`
        hands back a zero-length capsule and the segment-to-segment measure
        degenerates to point-to-point -- meaning the capsule half of it is
        never exercised. A review found three mutations surviving on exactly
        that (collapse the capsule to one end, measure the connector from an
        endpoint only, centre site A's rects on the pad instead of the
        circles), so the slot is built here rather than assumed.

        The via sits off the MIDDLE of the slot, which is the one arrangement
        an endpoint-only measure gets wrong: 1.2500 to the axis, but 1.6008 to
        either end.
        """
        xh = 4.70
        v = make_via(VIA0[0], VIA0[1], net_id=3, size=V_SIZE, drill=V_DRILL)
        stub = make_seg(2.0, 3.0, VIA0[0], VIA0[1], width=0.2, layer='F.Cu',
                        net_id=3)
        hole = _slot(xh, 3.0, AUDIO1_LC)
        from kicad_parser import pad_drill_capsule
        (a, b, hr) = pad_drill_capsule(hole)
        # ON THE BRANCH: a real capsule, the via is off its MIDDLE, and the
        # endpoint measure would clear the requirement while the axis one does
        # not -- without that separation the arm cannot see the mutation.
        self.assertGreater(math.hypot(b[0] - a[0], b[1] - a[1]), 1.0,
                           'not a slot; this arm is vacuous')
        axis_d = abs(xh - LANDING[0])
        end_d = min(math.hypot(LANDING[0] - e[0], LANDING[1] - e[1])
                    for e in (a, b))
        need = hr + VR + AUDIO1_LC
        self.assertTrue(axis_d < need < end_d,
                        'axis %.4f / need %.4f / nearest end %.4f do not '
                        'separate' % (axis_d, need, end_d))
        # ...and the two gates that are NOT under test must both pass here.
        self.assertGreater(axis_d - hr, CLEAR + VR)                  # base
        self.assertGreater(axis_d, V_DRILL / 2.0 + hr + H2H_PAD)     # drill

        pcb = make_pcb(board_info=_bi(), vias=[v], segments=[stub],
                       footprints={'C1': SimpleNamespace(layer='F.Cu',
                                                         pads=[])},
                       pads_by_net={0: [hole]}, zones=[])
        moves, segs, out = _nudge(_FakeSt(WALLS), pcb, CLEAR,
                                  max_shift=MAX_SHIFT)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
        # the same slot with no override moves, so the refusal is #730's and
        # not the capsule merely being large.
        hole2 = _slot(xh, 3.0, 0.0)
        v2 = make_via(VIA0[0], VIA0[1], net_id=3, size=V_SIZE, drill=V_DRILL)
        pcb2 = make_pcb(board_info=_bi(), vias=[v2],
                        segments=[make_seg(2.0, 3.0, VIA0[0], VIA0[1],
                                           width=0.2, layer='F.Cu', net_id=3)],
                        footprints={'C1': SimpleNamespace(layer='F.Cu',
                                                          pads=[])},
                        pads_by_net={0: [hole2]}, zones=[])
        moves2, _s2, _o2 = _nudge(_FakeSt(WALLS), pcb2, CLEAR,
                                  max_shift=MAX_SHIFT)
        self.assertEqual(len(moves2), 1)
    # MUTATION: collapse the override capsule to either endpoint, or measure
    # the gate from a via/connector ENDPOINT rather than along the axis -> this
    # arm moves. It is the only capsule-shaped override hole in the file.

    def test_a_hole_on_the_vias_OWN_net_is_exempt(self):
        """A track legitimately reaches its own mounting-hole pad, and
        check_drc's via arm makes the same exemption."""
        v, moves, _s, _o = _run_via(XH_REFUSED, AUDIO1_LC, hole_net=3)
        self.assertEqual(len(moves), 1, 'an own-net hole blocked its own via')
        v2, moves2, _s2, _o2 = _run_via(XH_REFUSED, AUDIO1_LC, hole_net=0)
        self.assertEqual(moves2, [], 'the positive control stopped refusing')
    # MUTATION: via-override-drops-the-own-net-skip -> measured 1 arm, this
    # one. It is the only own-net fixture in the file.


class TestTheOverrideIsASTEPNotAMax(unittest.TestCase):
    """Site B's threshold, both directions, in ONE geometry.

    check_drc's via arm charges `lc` only when `lc > npth_clr`, and plain
    `clearance` otherwise. So `max(clearance, lc)` -- the obvious spelling --
    over-blocks for every `clearance < lc <= npth_clr`, inventing a refusal
    kicad-cli never reports. Measured at gap 0.120: the charge jumps from
    0.10 to 0.2001 across a single 0.1um step in lc.
    """

    def _moves(self, lc, xh=XH_STEP):
        return len(_run_via(xh, lc)[1])

    def test_an_lc_AT_OR_BELOW_npth_clr_charges_the_via_NOTHING(self):
        npth_clr = max(CLEAR, NPTH_FLOOR)
        gap = XH_STEP - LANDING[0] - HR - VR
        # ON THE BRANCH: the landing is inside the band a `max(clearance, lc)`
        # gate would refuse -- above `clearance`, below `lc` -- while `lc` is
        # itself at or below `npth_clr`. Without all three the arm is vacuous.
        self.assertLessEqual(LC_LOW, npth_clr + 1e-9)
        self.assertTrue(CLEAR + 1e-9 < gap < LC_LOW - 1e-9,
                        'gap %.4f is not strictly inside (clearance, lc_low)'
                        % gap)
        self.assertEqual(self._moves(LC_LOW), 1)
    # MUTATION: via-override-threshold-at-clearance (`_hlc <= npth_step` ->
    # `_hlc <= clearance`) -> this arm refuses. Measured: 2 arms, this one and
    # the exact-threshold sweep below -- NOT the single arm the note first
    # claimed. Every other arm in the file uses an lc above npth_clr, where
    # the two spellings agree.

    def test_an_lc_JUST_ABOVE_npth_clr_charges_it_in_full(self):
        npth_clr = max(CLEAR, NPTH_FLOOR)
        self.assertGreater(LC_HIGH, npth_clr + 1e-9)
        # ...for every candidate the walls admit; see the note in
        # TestARelocatedViaHonoursTheHolesOwnOverride about what that does and
        # does not establish.
        for cx, _cy in ALL_CANDIDATES:
            self.assertLess(XH_STEP - cx - HR - VR, LC_HIGH + 1e-9)
        self.assertEqual(self._moves(LC_HIGH), 0)
    # MUTATION: via-override-threshold-doubled -> measured 7 arms; the upper
    # direction is far better covered than the lower one, which is why the arm
    # above exists.

    def test_the_threshold_sits_exactly_at_npth_clr(self):
        """The bound, crossed rather than derived. Both values are the same
        geometry; only `lc` moves, by one ten-thousandth of a millimetre."""
        npth_clr = max(CLEAR, NPTH_FLOOR)
        self.assertAlmostEqual(STEP_MOVES, npth_clr, places=6)
        self.assertEqual(self._moves(STEP_MOVES), 1,
                         'lc exactly AT npth_clr must charge `clearance` -- '
                         'check_drc s test is a strict `>`')
        self.assertEqual(self._moves(STEP_REFUSES), 0,
                         'lc one 1e-4 above npth_clr must charge `lc`')

    def test_the_charge_itself_is_lc_and_the_bound_is_crossed(self):
        """Having established WHICH holes are charged, this pins WHAT they are
        charged: at lc 0.25 the requirement is 0.25, so 0.230 refuses and
        0.250 moves."""
        self.assertEqual(len(_run_via(XH_HIGH_REFUSES, LC_HIGH)[1]), 0)
        self.assertEqual(len(_run_via(XH_HIGH_MOVES, LC_HIGH)[1]), 1)
        self.assertAlmostEqual(XH_HIGH_MOVES - LANDING[0] - HR - VR, LC_HIGH,
                               places=4)
    # MUTATION: via-override-uses-the-DRILL-radius (`vr` -> the barrel's DRILL
    # radius, 0.15 against 0.70) -> measured 5 arms. The bound rows here are
    # two of them.

    def test_the_two_step_arms_differ_ONLY_in_the_override(self):
        """Without this, "0.15 moves / 0.25 refuses" could be a geometry
        difference rather than the step."""
        a, _pa = _via_board(XH_STEP, LC_LOW)
        b, _pb = _via_board(XH_STEP, LC_HIGH)
        self.assertEqual((a.x, a.y, a.size, a.drill),
                         (b.x, b.y, b.size, b.drill))
        ha = _pa.pads_by_net[0][0]
        hb = _pb.pads_by_net[0][0]
        self.assertEqual((ha.global_x, ha.global_y, ha.drill),
                         (hb.global_x, hb.global_y, hb.drill))
        self.assertNotEqual(ha.local_clearance, hb.local_clearance)
    # MUTATION: none -- a control.


class TestTheConnectorHonoursTheHolesOwnOverride(unittest.TestCase):
    """Site C, the TRACK arm. Its shape differs from site B's and this class
    is what keeps the two from being unified."""

    def test_a_connector_inside_an_OVERRIDDEN_holes_keep_out_is_refused(self):
        v, moves, segs, out = _run_conn(XH_C_REFUSED, AUDIO1_LC)
        seg_dist = XH_C_REFUSED - C_LANDING[0]
        # ON THE BRANCH, and the attribution matters most here:
        #  (i) the connector's half-width EXCEEDS the via radius, or this gate
        #      can never be the binding one and the arm silently becomes a
        #      second site-B test;
        self.assertGreater(C_HW, C_VR)
        # (ii) the VIA gate accepts this landing, with room to spare;
        self.assertGreaterEqual(seg_dist, HR + C_VR + AUDIO1_LC + 0.05)
        # (iii) the landing is between the floor this gate already charged and
        #       what the override charges;
        npth_clr = max(CLEAR, NPTH_FLOOR)
        self.assertTrue(HR + C_HW + npth_clr + 0.05 <= seg_dist
                        <= HR + C_HW + AUDIO1_LC - 0.05)
        # (iv) and every candidate the budget REACHES is inside. C_MAX_SHIFT
        #      is 0.15, so the along-axis candidates are 3.05 / 3.10 / 3.15 --
        #      3.05 being the one `valid_via_pos` accepts, which makes it the
        #      candidate this gate actually decides. (An earlier version of
        #      this loop listed 3.45, three times the budget away, and omitted
        #      3.05 entirely; the assertion held either way, which is why a
        #      review had to find it rather than a failure.)
        for cx, _cy in ALL_CANDIDATES_C:
            self.assertLessEqual(cx - C_LANDING[0], C_MAX_SHIFT + 1e-9)
            self.assertLess(XH_C_REFUSED - cx, HR + C_HW + AUDIO1_LC + 1e-9)
        self.assertIn('no clear spot', out)
        self.assertEqual(moves, [])
    # MUTATION: connector-override-gate-deleted -> the connector is drawn
    # 0.300 off a hole wanting 0.400. Measured: 2 arms, this one and the bound
    # sweep. The via-side rows do NOT fire -- the two gates are separately
    # held, which is the point of splitting them.

    def test_a_connector_clear_of_the_override_is_still_drawn(self):
        v, moves, segs, out = _run_conn(XH_C_MOVES, AUDIO1_LC)
        self.assertEqual(len(moves), 1)
        self.assertEqual(len(segs), 1)
        # ON THE BRANCH: the connector really is 1.0mm wide, so `hw` is 0.50
        # and the arithmetic above is the arithmetic the gate ran.
        self.assertAlmostEqual(segs[0]['width'], C_STUB_W, places=6)
        self.assertAlmostEqual(v.x, C_LANDING[0], places=4)

    def test_the_same_geometry_with_NO_override_still_draws_it(self):
        v, moves, segs, _o = _run_conn(XH_C_REFUSED, 0.0)
        self.assertEqual(len(moves), 1)
        self.assertEqual(len(segs), 1)

    def test_a_SUB_FLOOR_override_charges_the_connector_nothing(self):
        """An `lc` below `npth_step` never enters `override_holes` at all, so
        the connector's requirement stays exactly what it was.

        NOT a MAX-versus-STEP discriminator, which is what this arm's name and
        docstring claimed until a review measured it: at this geometry the wall
        gap clears BOTH candidate charges, so the two spellings agree and the
        arm cannot separate them. What separates them is
        `test_the_connector_floor_is_npth_clr_and_not_clearance` below, which
        sits between the two floors. What THIS arm holds is the filter's lower
        edge -- that a sub-floor declaration is not quietly promoted -- which is
        the same property `override-charge-given-the-VIA-step` survives on, and
        worth a behavioural arm for exactly that reason."""
        npth_clr = max(CLEAR, NPTH_FLOOR)
        self.assertLess(LC_LOW, npth_clr)
        self.assertEqual(len(_run_conn(XH_C_REFUSED, LC_LOW)[1]), 1)
    # MUTATION: override-charge-given-the-VIA-step -> SURVIVED, expected, and
    # recorded as such in mutate_730.py. The list is already filtered to
    # `lc > npth_step >= npth_clr`, so `max(npth_clr, lc)` and the step both
    # collapse to `lc` for every member: the FILTER carries this arm, not the
    # charge. Worth stating, because it is the thing a reader is most likely
    # to get backwards.

    def test_the_requirement_bound_is_crossed(self):
        """`hr + hw + lc` = 1.400, so 1.350 refuses and 1.400 moves. Derived
        intervals are how #746 shipped a bound that excluded its own
        counterexample; this one is swept."""
        self.assertEqual(len(_run_conn(XH_C_BOUND_REFUSES, AUDIO1_LC)[1]), 0)
        self.assertEqual(len(_run_conn(XH_C_BOUND_MOVES, AUDIO1_LC)[1]), 1)
        self.assertAlmostEqual(XH_C_BOUND_MOVES - C_LANDING[0],
                               HR + C_HW + AUDIO1_LC, places=4)


class TestTheConnectorGateStaysAtTheFlatFabFloor(unittest.TestCase):
    """Site D's exclusion, and #737's orphan.

    #617 chose to leave this gate's floor at the flat fab floor and measured
    the cost of raising it. test_737's source guard says its own assertion is
    "the ONLY thing in the tree" holding that floor. It no longer is: this arm
    holds it behaviourally, so a future reader can delete the source guard and
    still be caught.
    """

    def test_the_connector_floor_is_npth_clr_and_not_clearance(self):
        npth_clr = max(CLEAR, NPTH_FLOOR)
        seg_dist = XH_C_FLOOR - C_LANDING[0]
        # ON THE BRANCH: the landing sits BETWEEN the two candidate floors --
        # 0.05 above what `clearance` would require, 0.05 below what
        # `npth_clr` requires -- so the two spellings must disagree here.
        self.assertLess(CLEAR, npth_clr)
        self.assertTrue(HR + C_HW + CLEAR + 1e-9 < seg_dist
                        < HR + C_HW + npth_clr - 1e-9,
                        'seg_dist %.4f does not separate the two floors'
                        % seg_dist)
        self.assertEqual(len(_run_conn(XH_C_FLOOR, 0.0)[1]), 0,
                         'the connector gate stopped charging npth_clr -- '
                         '#617 set that floor deliberately')
    # MUTATION: connector-BASE-floor-lowered (`npth_clr + hw` ->
    # `clearance + hw`) -> 1 move to (3.05, 3.0). Measured: this used to pass
    # test_617, test_370 and test_fanout_clearance, caught by a source guard
    # alone.

    def test_the_two_NPTH_floors_differ_ON_PURPOSE(self):
        """The drift guard for #730's site D. `_Repair.npth_floor` reads the
        board; `nudge_vias_for_unresolved`'s `npth_clr` does not, and #617
        measured that unifying them returns 0 moves and 0 connectors where the
        flat floor returns 1 and 1. A later "consistency" refactor must not
        collapse them silently."""
        src = [l.split('#')[0] for l in
               inspect.getsource(FC.nudge_vias_for_unresolved).splitlines()]
        # `startswith('npth_clr')` alone also matches the connector gate's
        # CONTINUATION line (`npth_clr + hw - 1e-4:`), which is a use and not
        # an assignment. Match the assignment.
        flat = [i for i, l in enumerate(src)
                if re.match(r'\s*npth_clr\s*=', l)]
        self.assertEqual(len(flat), 1,
                         'expected exactly one npth_clr assignment; found %d'
                         % len(flat))
        # A POSITIVE shape check, not an absence one, and the mutation
        # battery is why. The first version asserted that three tokens were
        # absent from the assignment's FIRST line; the obvious mutant puts the
        # board read on a CONTINUATION line, under an aliased import, and
        # evaded both -- it was killed by test_617 alone, and this guard
        # reported green. An absence guard has to anticipate the spelling; a
        # shape guard does not.
        window = ' '.join(src[flat[0]:flat[0] + 3]).split(')')[0] + ')'
        self.assertEqual(
            ' '.join(window.split()),
            'npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE)',
            'npth_clr is no longer the FLAT two-term max. #617 measured the '
            'cost of board-deriving it: 1 move / 1 connector at a 0.2200mm '
            'gap becomes 0 and 0, and the #130 pad-via graze the pass exists '
            'to remove persists.')
        # ...and the far site still DOES read the board, or the two have been
        # unified from the other end -- which loses the cap-side raise #617
        # shipped.
        # `assertTrue(... in ...)`, not `assertIn`: `_Repair.__init__` is 577
        # lines, and assertIn prints the haystack. test_732 measured a 393KB
        # failure message doing exactly this.
        rsrc = inspect.getsource(_Repair.__init__)
        self.assertTrue('resolve_hole_clearance' in rsrc,
                        '_Repair.npth_floor stopped reading the board; #617 '
                        'changed exactly that site on purpose')
    # MUTATION: nudger-npth_clr-made-BOARD-AWARE -> measured 3 kills across 2
    # files: this arm plus test_617's two checks.
    #
    # It took a battery run to make that true. The first version of this guard
    # asserted that three tokens were ABSENT from the assignment's first line;
    # the obvious mutant puts the board read on a CONTINUATION line under an
    # aliased import and evaded both, leaving test_617 as the only killer while
    # this guard reported green. It is a POSITIVE shape check now -- an absence
    # guard has to anticipate the spelling, a shape guard does not.


class TestThePartPadsHoleKeepOutIsPerHole(unittest.TestCase):
    """Site E."""

    def test_holes_on_ONE_footprint_get_THEIR_OWN_radii(self):
        fp = _e_footprint()
        # ON THE BRANCH: all three reach holes_local, and the three overrides
        # straddle the fab floor -- else "different" is vacuous.
        self.assertGreater(E_LCS[1], NPTH_FLOOR)
        self.assertLess(E_LCS[2], NPTH_FLOOR)
        for clearance, want in E_RADII.items():
            with self.subTest(clearance=clearance):
                pp = PartPads(fp, clearance)
                self.assertEqual(len(pp.holes_local), 3)
                self.assertEqual([round(r, 4) for _x, _y, r in pp.holes_local],
                                 want)
    # MUTATION: site-E-reverted-to-the-hoist (measured 2 arms),
    # site-E-loses-the-fab-floor (3), site-E-threshold-at-clearance (3).
    # site-E-max-respelled-as-a-conditional SURVIVES, expected -- an exact
    # re-spelling of `max`, recorded so its survival is not read as a hole.

    def test_a_SLOT_hole_grows_from_the_SHORT_axis(self):
        """Site A has had this fixture since #730 shipped; site E had not, and
        a review measured `hd/2` -> `p.drill/2` surviving the whole placement
        family there. `p.drill` on a slot is the LONG axis -- 2.5 where the
        keep-out radius is 0.25 -- so the two spellings differ by an order of
        magnitude on a slot and not at all on a round hole."""
        from kicad_parser import pad_drill_circles
        slot = _slot(10.0, 10.0, AUDIO1_LC)
        fp = SimpleNamespace(reference='H1', x=10.0, y=10.0, rotation=0.0,
                             layer='F.Cu', pads=[slot])
        circles = list(pad_drill_circles(slot))
        # ON THE BRANCH: a real slot, and the two spellings really do differ.
        self.assertGreater(len(circles), 1, 'not a slot; this arm is vacuous')
        self.assertNotAlmostEqual(circles[0][2], slot.drill, places=6)
        pp = PartPads(fp, CLEAR)
        grow = max(0.0, max(NPTH_FLOOR, AUDIO1_LC) - CLEAR)
        want = round(circles[0][2] / 2.0 + grow, 4)
        self.assertEqual(sorted({round(r, 4) for _x, _y, r in pp.holes_local}),
                         [want])
        self.assertEqual(len(pp.holes_local), len(circles))
        self.assertNotAlmostEqual(want, slot.drill / 2.0 + grow, places=4,
                                  msg='the long-axis spelling would give the '
                                      'same answer here')
        # ...and the EXTENT radii track the same circles, minus the override.
        ext = max(0.0, NPTH_FLOOR - CLEAR)
        self.assertEqual(
            sorted({round(r, 4) for _x, _y, r in pp.holes_extent}),
            [round(circles[0][2] / 2.0 + ext, 4)])
    # MUTATION: site-E-long-axis-drill -> 1.55 instead of 0.55. Measured: this
    # arm alone; it survived test_730, test_placement_pad_legality, test_697
    # and test_beautify_labels before it existed.

    def test_the_SILK_consumer_is_opted_out(self):
        """`local_clearance` is a COPPER rule -- KiCad has no silk-to-hole rule
        at all -- and `hole_circles()` serves both questions. labels.py passes
        `copper_holes=False`; every copper consumer takes the default."""
        fp = _e_footprint()
        for clearance, want in E_RADII_SILK.items():
            with self.subTest(clearance=clearance):
                pp = PartPads(fp, clearance, None, False)
                self.assertEqual([round(r, 4) for _x, _y, r in pp.holes_local],
                                 want)
        # ...and the flag reaches the factory, which is how labels.py sets it.
        parts = build_part_pads({'H1': fp}, 0.10, None, False)
        self.assertEqual([round(r, 4) for _x, _y, r in parts['H1'].holes_local],
                         E_RADII_SILK[0.10])
        src = inspect.getsource(LEG.build_part_pads)
        self.assertIn('copper_holes', src)
    # MUTATION: site-E-silk-gate-dropped -> measured 3 arms, all inside this
    # one test (one per clearance subTest, plus the factory check).

    def test_no_board_read_is_attempted_here_and_that_is_DISCLOSED(self):
        """REWRITTEN by #761, not relaxed.

        #730 shipped this as a disclosure: legality's keep-out was
        `max(clearance, 0.20, lc)` where check_drc's track arm is
        `max(clearance, 0.20, hole_clearance, lc)`, `PartPads` could not see
        the board, and the arm said so out loud -- ending "that changes what
        the six build_part_pads call sites mean and needs its own review".

        #761 DID that review (2 of the 6 read hole keep-outs; the other 4 read
        pad rects and extents only) and supplied the term the way
        `fanout_clearance` already does: resolved ONCE, outside, and passed in
        as a float. So the property this arm defends is unchanged -- the class
        still reads no board -- while the value it lacked now arrives.

        Relaxing it instead would have been the easy move and the wrong one: a
        board POINTER in here re-opens exactly the six-call-site question, so
        the ban stays, positionally, and the new parameter is asserted to be a
        scalar rather than merely present."""
        params = list(inspect.signature(PartPads.__init__).parameters)
        self.assertEqual(params[:6],
                         ['self', 'fp', 'clearance', 'model', 'copper_holes',
                          'npth_floor'])
        for banned in ('pcb_data', 'pcb_file', 'board', 'source_path'):
            self.assertNotIn(banned, params)
        # A SCALAR, not a board wearing a scalar's name: the default must be
        # usable arithmetic-free (None -> the fab floor) and a float must be
        # accepted positionally.
        fp = _e_footprint()
        self.assertEqual(
            [round(r, 4) for _x, _y, r in PartPads(fp, 0.10).holes_local],
            [round(r, 4) for _x, _y, r in
             PartPads(fp, 0.10, None, True, None).holes_local])
        raised = [round(r, 4) for _x, _y, r in
                  PartPads(fp, 0.10, None, True, 0.25).holes_local]
        self.assertNotEqual(
            raised,
            [round(r, 4) for _x, _y, r in PartPads(fp, 0.10).holes_local],
            'npth_floor is inert -- the parameter exists but reaches nothing')
        # CODE only: this fix's comments NAME resolve_hole_clearance when
        # explaining where the read now happens, and a raw-source scan would
        # read that as the read itself.
        code = ' '.join(l.split('#')[0] for l in
                        inspect.getsource(PartPads.__init__).splitlines())
        # assertTrue, not assertNotIn -- see the note in the sibling arm about
        # printing the haystack.
        for banned in ('resolve_hole_clearance', 'obstacle_map',
                       'resolve_npth_floor'):
            self.assertTrue(banned not in code,
                            'PartPads reads the board now (%s); that changes '
                            'what the six build_part_pads call sites mean and '
                            'needs its own review' % banned)
    # MUTATION: 761-partpads-reads-the-board (KILLED). Still a DISCLOSURE arm
    # for the board POINTER; the scalar half is a real fix assertion.


class TestOneHoleRule(unittest.TestCase):
    """Source guards. Reported by LINE NUMBER, never `assertIn` over the module
    source -- test_732 measured a 393KB failure message doing that."""

    def _nudger_src(self):
        return [l.split('#')[0] for l in
                inspect.getsource(FC.nudge_vias_for_unresolved).splitlines()]

    def test_the_override_is_resolved_in_exactly_one_place(self):
        """One selector, one threshold. A second copy is a second answer to
        "which holes carry an override", and they can only be trusted to agree
        if there is one."""
        src = self._nudger_src()
        sel = [i + 1 for i, l in enumerate(src) if 'override_holes' in l
               and '=' in l and 'if' not in l]
        self.assertEqual(len(sel), 2,
                         'expected exactly two `override_holes` assignments '
                         '(the build and the reach filter); found %d at '
                         'function-relative line(s) %s' % (len(sel), sel))
        step = [i + 1 for i, l in enumerate(src) if 'npth_step' in l
                and l.lstrip().startswith('npth_step')]
        self.assertEqual(len(step), 1,
                         'expected exactly one npth_step assignment; found %s'
                         % step)

    def test_the_hole_SET_uses_the_engines_own_predicate(self):
        """Not a re-spelling. The override gate and its two scalar siblings
        must agree about which holes EXIST -- they measure the same board --
        and a locally re-derived predicate is how that agreement is lost,
        silently and by under-blocking."""
        src = self._nudger_src()
        build = [i for i, l in enumerate(src) if 'override_holes.append' in l]
        self.assertEqual(len(build), 1)
        window = ' '.join(src[max(0, build[0] - 12):build[0] + 1])
        self.assertIn('_pad_has_no_copper(', window)
        self.assertIn('pad_drill_capsule(', window)
        self.assertIn('pads_by_net', window,
                      'the override list is not built from pads_by_net -- '
                      '`board_pads` DROPS the pads of movable caps, which the '
                      'sibling gates keep')

    def test_this_files_mirrored_constants_are_still_the_engines(self):
        """`H2H_PAD` is a function-local whose VALUE is now
        `resolve_drill_floors`' answer (#756), not a literal, so the mirror is
        checked against the resolver on the kind of board this file builds --
        one with no project, which takes the fab floor. The literal ban below
        is the second half: a value check alone would pass a resolver
        hard-wired back to 0.45.

        `test_750` owns the assignment-shape half; this file only needs its own
        drill-gate arithmetic to stay honest."""
        pad = FC.resolve_drill_floors(
            make_pcb(board_info=BoardInfo(layers={},
                                          copper_layers=['F.Cu', 'B.Cu'],
                                          board_bounds=(0.0, 0.0, 4.0, 4.0)),
                     source_path=''))[1]
        self.assertEqual(pad, H2H_PAD,
                         "the engine's pad drill floor on a project-less "
                         "board is no longer %s, so this file's mirror -- and "
                         'the drill-test arithmetic in every ON-THE-BRANCH '
                         'guard here -- is stale' % H2H_PAD)
        bad = [i + 1 for i, l in enumerate(self._nudger_src())
               if l.lstrip().startswith('H2H_PAD =')]
        self.assertEqual(bad, [],
                         'H2H_PAD is a literal again at function-relative '
                         'line(s) %s -- #756 exists because it was' % bad)

    def test_the_scalar_mirror_matches_the_shared_kernel(self):
        """The override gates measure with the SHARED seg-to-seg helper, and it
        must agree with `_seg_capsule_axis_dist` -- the kernel the two sibling
        hole gates measure with -- because all four grade the same holes.

        The kernel's crossing test is a STRICT `o1*o2 < 0`. This module's own
        `_segs_cross` counts an orientation of 0 as a crossing and would answer
        0.0 where the kernel does not, which is why the gates do not use it;
        the `endpoint ON the other line` case below is exactly that
        disagreement, and it is the reason this arm exists."""
        import numpy as np
        from check_drc import _seg_seg_dist_coords
        from single_ended_routing import _seg_capsule_axis_dist
        cases = [
            (0.0, 0.0, 1.0, 0.0, 0.5, -1.0, 0.5, 1.0),      # proper crossing
            (0.0, 0.0, 1.0, 0.0, 0.5, 0.0, 0.5, 1.0),       # endpoint ON it
            (0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.0),       # collinear, apart
            (0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0),       # two points
            (0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0),       # X
            (1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 2.0, 0.0),       # point vs segment
        ]
        for c in cases:
            with self.subTest(case=c):
                want = float(_seg_capsule_axis_dist(
                    c[0], c[1], c[2], c[3],
                    np.asarray([c[4]], dtype=float),
                    np.asarray([c[5]], dtype=float),
                    np.asarray([c[6]], dtype=float),
                    np.asarray([c[7]], dtype=float))[0])
                self.assertAlmostEqual(_seg_seg_dist_coords(*c), want,
                                       places=9)
    # MUTATION: none in the battery any more -- the row that killed this
    # (`seg-seg-crossing-test-relaxed`) mutated a LOCAL copy of the kernel that
    # #730 no longer carries, since the gates now call the shared helper. The
    # arm is kept as a CHANGE DETECTOR on that helper: it is used by
    # placement/legality.py at three sites besides these gates, so relaxing its
    # crossing test would be felt well beyond #730.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """Self-expiring bounds. Every clause is selected the way the ENGINE
    selects -- `git ls-files` for the board set, check_drc's own predicate for
    the holes, `resolve_hole_clearance` for the floor -- never a name grep.
    #746 shipped a bound whose candidate set excluded its own counterexample
    because it grepped footprint names where the engine calls a detector."""

    @classmethod
    def setUpClass(cls):
        cls.boards = run_utils.corpus_boards()

    def _scan(self, clearance):
        """(total NPTH pads, {board: [binding pads]}) at this clearance.

        The floor is sites A/B/C's -- it carries the board's declared
        min_hole_clearance, and since #761 site E carries it too: the caller
        resolves it once (`legality.resolve_npth_floor`) and passes the
        scalar, so this scan and site E now select on the SAME floor. It was
        strictly lower while `PartPads` could not see the board, which made a
        hole with `npth_clr < lc <= declared` bind at site E while this scan
        called it non-binding -- unreachable then and gone now, but recorded
        because the fix, not the argument, is what closed it. Inert on the
        corpus either way: the one board declaring above the fab floor is
        flat_hierarchy, and all 6 of its NPTH pads are at
        lc 0.0. Stated because "selected the way the ENGINE selects" is true of
        A/B/C and only approximately true of E.
        """
        total, binding = 0, {}
        for b in self.boards:
            try:
                pcb = parse_kicad_pcb(b)
            except Exception:                                    # noqa: BLE001
                continue
            floor = max(NPTH_FLOOR, resolve_hole_clearance(pcb, None),
                        clearance)
            hit = []
            for ref, p in _npth_pads(pcb):
                total += 1
                if (getattr(p, 'local_clearance', 0.0) or 0.0) > floor + 1e-9:
                    hit.append((ref, p))
            if hit:
                binding[os.path.basename(b)] = hit
        return total, binding

    def test_exactly_one_tracked_board_carries_a_binding_override(self):
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(len(self.boards), 20,
                                'the tracked corpus collapsed; this arm '
                                'proves nothing')
        for clearance in (0.10, 0.25):
            with self.subTest(clearance=clearance):
                total, binding = self._scan(clearance)
                self.assertGreater(total, 25,
                                   'only %d copper-less drilled pads found; '
                                   'the scan is not reading the boards'
                                   % total)
                self.assertEqual(sorted(binding), ['ulx3s.kicad_pcb'],
                                 'the set of boards a #730 override binds on '
                                 'has CHANGED: %r. The "corpus-inert" claim in '
                                 'the #730 PR has EXPIRED -- re-run the '
                                 'before/after sweep and record the numbers'
                                 % {k: len(v) for k, v in binding.items()})
                self.assertEqual(len(binding['ulx3s.kicad_pcb']), 2)

    def test_the_one_binding_board_still_misses_by_ten_microns(self):
        """The tightest number in the whole change, and the reason site A is
        corpus-inert rather than a behaviour win. Asserted against the bound
        itself with the margin printed, not against an invented safety
        subtraction."""
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        pcb = parse_kicad_pcb(ULX3S)
        holes = _npth_pads(pcb)
        self.assertEqual(len(holes), 2)
        worst = None
        for ref, hp in holes:
            hr = hp.drill / 2.0
            for r2, fp2 in pcb.footprints.items():
                if r2 == ref:
                    continue
                for q in fp2.pads:
                    if _pad_has_no_copper(q):
                        continue
                    d = point_to_pad_distance(hp.global_x, hp.global_y, q) - hr
                    if worst is None or d < worst[0]:
                        worst = (d, '%s.%s' % (r2, q.pad_number))
        self.assertIsNotNone(worst)
        margin = worst[0] - AUDIO1_LC
        print('        ulx3s AUDIO1 hole wall -> %s at %.4fmm; the #730 '
              'requirement is %.4f, margin %.4fmm'
              % (worst[1], worst[0], AUDIO1_LC, margin))
        self.assertGreater(margin, 0.0,
                           'foreign copper is now INSIDE the #730 keep-out on '
                           'ulx3s (%s at %.4fmm vs a %.4f requirement). Site A '
                           'has stopped being corpus-inert -- re-measure the '
                           'repair pass before/after and update the PR'
                           % (worst[1], worst[0], AUDIO1_LC))
        self.assertAlmostEqual(worst[0], AUDIO1_NEAREST_COPPER, places=4)
    # MUTATION: none -- self-expiring corpus bounds, not fix assertions.

    def test_no_tracked_board_can_reach_the_via_or_connector_gate(self):
        """Sites B and C are unreachable on the corpus, and this says so with
        the reason rather than leaving it implied: the only board with a
        binding override carries no vias at all, so the reach filter empties
        its override list and both gates cost one truth test."""
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        pcb = parse_kicad_pcb(ULX3S)
        self.assertEqual(len(pcb.vias), 0,
                         'ulx3s now carries vias, so the #730 via/connector '
                         'gates may be reachable -- re-measure')


if __name__ == "__main__":
    unittest.main(verbosity=2)
