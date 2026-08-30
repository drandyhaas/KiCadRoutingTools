"""#733: the board-edge requirement must have ONE resolver in this pass.

`_Repair.__init__` insets `usable` and builds `edge_gate` at
`max(clearance, board_edge_clearance)`, and gates every candidate cap rect on
that. `nudge_vias_for_unresolved` -- which RELOCATES real vias and DRAWS real
connector segments, the only channel here that emits copper rather than moving
parts -- gated its own `edge_ok_point` / `edge_ok_seg` at the bare flat
`clearance`, and took no `board_edge_clearance` parameter at all, so it could
not agree.

Measured on the rig below at HEAD 0f62d190, at the shipped CLI defaults
(`--clearance 0.25`, `--board-edge-clearance 0.55`):

    board top 2.15, cap bar at y 0.9..1.1, one foreign 0.5mm via at (1.0, 1.4)

  * before: the via is relocated to (1.0, 1.600) and the writer keeps it
    (place_fanout_clearance.py:138). Its centre is 0.550mm from Edge.Cuts --
    0.30mm inside the band the cap mover reserves, on copper this pass created.
  * after: no candidate validates, the pass prints "no clear spot", and the
    via stays put.

The same 0.30mm on the GUI path, which passed NO value and silently took the
signature default whatever the board or the operator said.

TIGHTEN-ONLY, and that is the interesting half. `resolve_cap_edge_clearance`
raises the 0.55 default to a board's declared `min_copper_edge_clearance` but
never lowers it, because `fix_project_for_output` PINS that field UP to the fab
copper-to-edge floor 0.20 on every board it writes
(fix_kicad_drc_settings.py:608 -> :748-753, the one raise-allowed key) and
`bga_fanout.py` calls it -- so in the documented pipeline
`bga_fanout.py -> place_fanout_clearance.py` every board arrives declaring
>= 0.20 even when its author declared nothing. Read plainly board-first, that
0.20 comes back tagged "board constraint" -- this pipeline's own default
wearing the board's name -- and the cap margin would silently drop to
max(clearance, 0.20).

A FINDING THIS FILE EXISTS FOR. The two harnesses that drive
`nudge_vias_for_unresolved` with a duck-typed `st` --
tests/test_370_tierb_fixes.py:377-383 and
tests/test_617_placement_fanout_hole_clearance.py:243 -- do NOT pin the
None-fallback at all. Every landing they exercise clears 0.80mm, so a fallback
of 0.55 passes all five of their arms unchanged. `TestTheDuckTypedPathStaysInert`
brackets the fallback to (0.20, 0.30] on a rig built for it, because the
regression a later reader would introduce by "tidying" that line is one nothing
else would catch.

Conventions this file follows (from #697/#725/#731/#732 and CLAUDE.md):

  * REAL parser dataclasses, and a REAL board wherever the outline matters.
  * Every assertion names the single-line MUTATION that must kill it.
  * Assert you are ON the branch before asserting about it.
  * Every "is refused" is paired with a NEGATIVE CONTROL that still passes.
  * No fixture sits ON a threshold. Measured: at clearance 0.1 the r=0.05
    candidate lands 0.34999999999999987 from a 0.35 requirement and is
    float-rejected, so every rig here keeps >= 0.05mm of headroom.

Runs in-process in a few seconds; nothing here shells out.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
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

from kicad_parser import BoardInfo, PCBData, Segment, Via, parse_kicad_pcb
from check_drc import (board_edge_geometry, _point_on_board,
                       _point_to_rings_distance)
from placement import fanout_clearance as FC
from placement.fanout_clearance import (CAP_EDGE_CLEARANCE, _Repair,
                                        nudge_vias_for_unresolved,
                                        resolve_cap_edge_clearance)

# The inert board (#732's, for the same reason): no netclass, no .kicad_dru, no
# pad override and NO sibling .kicad_pro, so every pair requirement collapses to
# `clearance` and the EDGE MARGIN is the only variable this file can move.
BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
# The only tracked board with real interior Edge.Cuts rings that also carries no
# vias of its own, so its outline can be borrowed without foreign copper.
WATCHY = os.path.join(_ROOT, 'kicad_files', 'watchy.kicad_pcb')

CLEAR = 0.25               # routing_defaults.CLEARANCE, the shipped --clearance
BEC = 0.55                 # the shipped --board-edge-clearance == CAP_EDGE_CLEARANCE
VIA_R = 0.25               # a 0.5mm via: needs 0.50 under the old convention and
                           # 0.80 under the new one -- a 0.30mm band.
PREFIX = 'C,R,FB'
FOREIGN = 3

HBAR = (0.2, 0.9, 1.8, 1.1, 2)   # cap pad bar on net 2; the only escape is +y


class _FakeCap:
    """Minimal stand-in for _Cap: fixed pad rects, never moves."""

    def __init__(self, rects):
        self._rects = rects
        self.x = self.y = self.rot = 0.0

    def pad_rects(self, x=None, y=None, rot=None):
        return self._rects


class _FakeSt:
    """The duck-typed `st` the #370/#617 harnesses pass: no resolvers at all.

    `edge_margin` is ABSENT, not None, so `getattr(st, 'edge_margin', None)`
    takes its default -- the branch TestTheDuckTypedPathStaysInert pins.
    """

    def __init__(self, rects):
        self.caps = {'C1': _FakeCap(rects)}
        self.vias = []

    def graze_penalty(self, ref, cap, x, y, rot):
        return 1.0          # permanently unresolved, so the offender loop RUNS


class _MarginSt(_FakeSt):
    """...plus the one resolver this issue is about."""

    def __init__(self, rects, margin):
        super().__init__(rects)
        self.edge_margin = margin


class _SpySt(_FakeSt):
    """Counts reads of `edge_margin`, so "the nudger READ st" is provable."""

    def __init__(self, rects, margin):
        super().__init__(rects)
        self._margin = margin
        self.reads = 0

    @property
    def edge_margin(self):
        self.reads += 1
        return self._margin


def _repair(pcb, bec=BEC, clear=CLEAR, path=BOARD):
    """The 10-POSITIONAL construction this family uses (#725's shape contract
    pins the slots); `bec` is argument 5."""
    return _Repair(pcb, path, clear, 0.1, bec, 1.0, 2.0, 0.8, PREFIX, set())


def _bbox_board(top, vias, segments=()):
    bi = BoardInfo(layers={}, board_bounds=(0.0, 0.0, 2.0, top),
                   copper_layers=['F.Cu', 'B.Cu'])
    return PCBData(board_info=bi, nets={},
                   footprints={'C1': SimpleNamespace(layer='F.Cu', pads=[])},
                   vias=list(vias), segments=list(segments), pads_by_net={})


def _graze_via(x=1.0, y=1.4):
    return Via(x=x, y=y, size=2 * VIA_R, drill=0.3,
               layers=['F.Cu', 'B.Cu'], net_id=FOREIGN)


def _nudge(st, pcb, clear=CLEAR, **kw):
    """Drive the real pass, capturing what it printed. The PRINTED OUTPUT is
    half the evidence: a refusal that prints nothing is indistinguishable from
    a pass that never looked (the #732 silent-failure lesson)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        moves, segs = nudge_vias_for_unresolved(st, pcb, clear, **kw)
    return moves, segs, buf.getvalue()


def _rig_a(top, st, clear=CLEAR, segments=()):
    """Bbox rig: one foreign via boxed by a cap bar so its ONLY escape is +y,
    straight at the top edge. Measured landing is (1.0, 1.600) whenever a move
    happens at all, so `top - 1.6` IS the edge distance under test. A sideways
    escape would need r >= 1.3, well past the 0.6mm search budget."""
    v = _graze_via()
    pcb = _bbox_board(top, [v], segments)
    moves, segs, out = _nudge(st, pcb, clear)
    return v, pcb, moves, segs, out


class TestTheBoardEdgeRequirementHasOneResolver(unittest.TestCase):
    """The invariant #733 is about, on a real board: one number, not two."""

    def test_the_cap_mover_and_the_nudger_read_the_same_margin(self):
        pcb = parse_kicad_pcb(BOARD)
        st = _repair(pcb)
        # ON THE BRANCH: if the two conventions coincided, this whole file
        # would pass vacuously.
        self.assertGreater(BEC, CLEAR,
                           'board_edge_clearance does not exceed clearance, so '
                           'max() is a no-op and nothing here is under test')
        self.assertAlmostEqual(st.edge_margin, BEC, places=12,
                               msg='edge_margin is not max(clearance, bec)')
        self.assertAlmostEqual(st.edge_margin, st.edge_gate.margin, places=12,
                               msg='the rect gate and the scalar disagree -- '
                                   'there are two storages again')
        b = pcb.board_info.board_bounds
        self.assertAlmostEqual(st.usable[0] - b[0], st.edge_margin, places=12,
                               msg='the bbox inset is not the same number')
        self.assertAlmostEqual(b[2] - st.usable[2], st.edge_margin, places=12)
    # MUTATION: `return self.edge_gate.margin` -> `return self.clearance`.

    def test_a_clearance_above_the_edge_flag_wins_the_max(self):
        pcb = parse_kicad_pcb(BOARD)
        big = _repair(pcb, bec=0.55, clear=0.6)
        self.assertAlmostEqual(big.edge_margin, 0.6, places=12,
                               msg='a clearance above the edge flag must win')
        self.assertAlmostEqual(big.edge_gate.margin, 0.6, places=12)
        # NEGATIVE CONTROL: below it, the edge flag still governs.
        small = _repair(pcb, bec=0.55, clear=0.1)
        self.assertAlmostEqual(small.edge_margin, 0.55, places=12,
                               msg='the edge flag stopped governing')
    # MUTATION: `margin = max(clearance, board_edge_clearance)` ->
    # `margin = board_edge_clearance` -- the first assertion reads 0.55.

    def test_the_nudger_never_gates_below_the_clearance_it_was_called_with(self):
        """The flat `clearance` argument is this function's own floor: an st
        must never LOWER the edge test below it.

        Defensive rather than load-bearing, and worth saying so: no LIVE caller
        can hand the nudger a sub-clearance margin today. `_Repair` is built at
        one site and its margin is already max(clearance, ...); the margin-ZERO
        outline gate render_placement builds is a shallow COPY it keeps beside
        `state.edge_gate` (render_placement.py:275-284), not a `_Repair`. An
        earlier version of this docstring claimed otherwise and the #733
        fact-check caught it. A duck-typed st CAN, and this is that."""
        st = _MarginSt([HBAR], 0.0)
        # 1.90 - 1.6 = 0.30 of room; max(0.25, 0.0) needs 0.25 + 0.25 = 0.50.
        _v, _pcb, moves, _segs, out = _rig_a(1.90, st)
        self.assertIn('no clear spot', out,
                      'a margin-0 st silently lowered the edge gate below the '
                      'clearance the caller passed')
        self.assertEqual(moves, [], 'the via was relocated at a 0.0 margin')
        # NEGATIVE CONTROL: the same st on a roomier board still moves it, so
        # the rig is not simply dead.
        st2 = _MarginSt([HBAR], 0.0)
        _v2, _pcb2, moves2, _s2, _o2 = _rig_a(2.15, st2)
        self.assertEqual(len(moves2), 1,
                         'the margin-0 arm rejects everything; the first '
                         'assertion proves nothing')
    # MUTATION: `max(clearance, _edge_m)` -> `_edge_m` -- edge_margin becomes
    # 0.0, need drops to 0.25 <= 0.30, and the via moves.


class TestARelocatedViaHonoursTheBoardEdgeMargin(unittest.TestCase):
    """edge_ok_point, both signs."""

    def test_a_via_in_the_band_between_the_two_conventions_is_refused(self):
        """The headline. At top 2.15 the only landing sits 0.550mm from
        Edge.Cuts: clear under `clearance` (needs 0.500), inside the margin
        (needs 0.800)."""
        # ON THE BRANCH: the landing must be IN the band, or this test is a
        # statement about some other geometry.
        self.assertTrue(VIA_R + CLEAR <= 0.55 < VIA_R + BEC,
                        'the landing is not in the band that separates the two '
                        'conventions (old %.3f, new %.3f)'
                        % (VIA_R + CLEAR, VIA_R + BEC))
        v, _pcb, moves, segs, out = _rig_a(2.15, _MarginSt([HBAR], BEC))
        # The PRINT first: assert the move count first and no mutation can ever
        # make the silence the reported failure (#732's lesson).
        self.assertIn('no clear spot', out,
                      'the pass said nothing about a via it declined to move')
        self.assertEqual(moves, [], 'the via was relocated into the edge band')
        self.assertEqual(segs, [], 'connector copper was drawn for no move')
        self.assertEqual((v.x, v.y), (1.0, 1.4), 'the via was mutated in place')
        # NEGATIVE CONTROL, same board: without the resolver the old convention
        # accepts exactly this landing -- which is what #733 reports.
        v2, _p2, moves2, _s2, out2 = _rig_a(2.15, _FakeSt([HBAR]))
        self.assertEqual(len(moves2), 1,
                         'the permissive arm did not move either, so the two '
                         'conventions are not being separated here')
        self.assertAlmostEqual(v2.y, 1.6, places=4)
        self.assertIn('moved', out2)
    # MUTATION: `need = r + edge_margin` -> `need = r + clearance` -- the via
    # moves and stdout says "moved" instead of "no clear spot".

    def test_a_via_with_real_room_still_moves_at_the_raised_margin(self):
        """The over-rejection guard: raising the margin must not simply stop
        the pass working."""
        v, _pcb, moves, _segs, out = _rig_a(2.60, _MarginSt([HBAR], BEC))
        self.assertEqual(len(moves), 1,
                         'a via with 1.00mm of room was refused at a 0.80mm '
                         'requirement -- the gate over-rejects')
        self.assertAlmostEqual(v.y, 1.6, places=4)
        self.assertIn('moved', out)
    # MUTATION: `need = r + edge_margin` -> `need = r + 2 * edge_margin`.


class TestAConnectorSegmentHonoursTheBoardEdgeMargin(unittest.TestCase):
    """edge_ok_seg. The connector is copper this pass DRAWS, so it is the half
    that matters most -- and it is priced on its own half-width."""

    def _rig_b(self, width, st):
        """Rig A at top 2.60 plus a same-net stub on B.Cu, so the connector is
        drawn there. B.Cu ON PURPOSE: connector_clear's cap-pad term is
        layer-gated (cap pads exist only on the cap's own side), so a wide F.Cu
        connector would be killed by the bar and the edge term never reached.
        On B.Cu the ONLY live gate is edge_ok_seg -- no board pads, no foreign
        vias (the moving via is same-net), no foreign segments, no NPTH."""
        stub = Segment(start_x=1.0, start_y=1.6, end_x=1.0, end_y=1.4,
                       width=width, layer='B.Cu', net_id=FOREIGN)
        return _rig_a(2.60, st, segments=[stub])

    def test_a_wide_connector_is_gated_on_its_own_half_width_plus_the_margin(self):
        # ON THE BRANCH: the POINT test passes at this board size, so a refusal
        # here is attributable to edge_ok_seg and to nothing else.
        self.assertGreaterEqual(2.60 - 1.6, VIA_R + BEC,
                                'the via itself does not clear the margin, so '
                                'edge_ok_point could be doing the rejecting')
        self.assertLess(2.60 - 1.6, 1.2 / 2.0 + BEC,
                        'a 1.2mm connector already clears the margin; this rig '
                        'separates nothing')
        _v, _pcb, moves, segs, out = self._rig_b(1.2, _MarginSt([HBAR], BEC))
        self.assertIn('no clear spot', out,
                      'the pass said nothing about a connector it declined')
        self.assertEqual(moves, [], 'the via moved despite an illegal connector')
        self.assertEqual(segs, [])
        # NEGATIVE CONTROL: the old convention draws exactly that connector.
        _v2, _p2, moves2, segs2, _o2 = self._rig_b(1.2, _FakeSt([HBAR]))
        self.assertEqual(len(moves2), 1)
        self.assertEqual([(s['layer'], s['width']) for s in segs2],
                         [('B.Cu', 1.2)],
                         'the permissive arm did not draw the wide connector, '
                         'so the two conventions are not being separated')
    # MUTATION: `need = hw + edge_margin` -> `need = hw + clearance`.

    def test_a_narrow_connector_on_the_same_board_still_moves(self):
        """Isolates the half-width term: without this, a gate that ignored `hw`
        entirely would still pass the test above."""
        _v, _pcb, moves, segs, _out = self._rig_b(0.2, _MarginSt([HBAR], BEC))
        self.assertEqual(len(moves), 1,
                         'a 0.2mm connector with 1.00mm of room was refused')
        self.assertEqual([(s['layer'], s['width']) for s in segs],
                         [('B.Cu', 0.2)])
    # MUTATION: `need = hw + edge_margin` -> `need = 2 * hw + edge_margin`
    # over-rejects and this move disappears.


class TestTheRealOutlineBranch(unittest.TestCase):
    """Both helpers have a rings branch and a bbox branch. The tests above
    exercise the bbox one; this exercises the rings one on real Edge.Cuts."""

    def _watchy(self):
        bi = parse_kicad_pcb(WATCHY).board_info
        rings, outer, cutouts = board_edge_geometry(bi)
        # Derived, never hard-coded: a watchy re-export must not silently
        # un-calibrate this fixture.
        cut_y = max(y for _x, y in cutouts[0])
        return bi, rings, outer, cutouts, cut_y

    def _run(self, st):
        bi, rings, outer, cutouts, cut_y = self._watchy()
        v = Via(x=85.0, y=cut_y + 0.95, size=2 * VIA_R, drill=0.3,
                layers=['F.Cu', 'B.Cu'], net_id=FOREIGN)
        bar = (84.2, cut_y + 1.25, 85.8, cut_y + 1.45, 2)
        pcb = PCBData(board_info=bi, nets={},
                      footprints={'C1': SimpleNamespace(layer='F.Cu', pads=[])},
                      vias=[v], segments=[], pads_by_net={})
        st = st([bar]) if st is _FakeSt else st([bar], BEC)
        moves, segs, out = _nudge(st, pcb)
        return v, moves, out, rings, outer, cutouts, bi

    def test_a_via_beside_a_real_milled_slot_is_refused_at_the_board_margin(self):
        # NEGATIVE CONTROL FIRST, because it establishes the geometry the
        # positive assertion depends on.
        v, moves, out, rings, outer, cutouts, bi = self._run(_FakeSt)
        self.assertTrue(rings, 'watchy yielded no rings -- this is the bbox '
                               'branch again, not the outline branch')
        self.assertEqual(len(moves), 1,
                         'the permissive arm did not move; nothing to compare')
        d = _point_to_rings_distance(v.x, v.y, rings)
        self.assertTrue(_point_on_board(v.x, v.y, outer, cutouts),
                        'the permissive arm parked the via off the board, '
                        'which is a different bug from the one under test')
        # ON THE BRANCH, twice over: the landing is in the band, AND the bbox
        # branch would happily accept it -- so the rings are what reject it.
        self.assertTrue(VIA_R + CLEAR <= d < VIA_R + BEC,
                        'the landing is %.4f from Edge.Cuts, not in the band '
                        '(%.3f, %.3f)' % (d, VIA_R + CLEAR, VIA_R + BEC))
        b = bi.board_bounds
        self.assertGreater(min(v.x - b[0], b[2] - v.x, v.y - b[1], b[3] - v.y),
                           VIA_R + BEC,
                           'the bounding box alone would also reject this '
                           'point, so the ring geometry proves nothing')
        v2, moves2, out2, *_ = self._run(_MarginSt)
        self.assertIn('no clear spot', out2)
        self.assertEqual(moves2, [],
                         'the via was parked %.4fmm from a real milled slot' % d)
    # MUTATION: `need = r + edge_margin` -> `need = r + clearance` in the RING
    # branch -- the via is parked 0.719mm from Edge.Cuts.


class TestTheDuckTypedPathStaysInert(unittest.TestCase):
    """#370/#617 pass an st with no resolvers. Their fixtures do NOT pin the
    fallback -- every landing they exercise clears 0.80mm -- so it is pinned
    here or nowhere."""

    def test_the_fallback_is_exactly_the_clearance_argument(self):
        st = _FakeSt([HBAR])
        # ON THE BRANCH: the fake must not have grown a margin.
        self.assertIsNone(getattr(st, 'edge_margin', None),
                          'the fake carries a margin, so the fallback branch '
                          'is never taken and this test is vacuous')
        # Bracket it. 2.15 leaves 0.55 of room and needs fallback + 0.25 <= 0.55;
        # 2.05 leaves 0.45 and must still refuse. Together the fallback is pinned
        # to the OPEN interval (0.20, 0.30) -- open at BOTH ends, measured: 0.30
        # itself fails the 2.15 arm, because the bbox branch has no -1e-6
        # tolerance where the rings branch does. CLEAR is 0.25, comfortably
        # inside. This BRACKETS the fallback; it does not prove it EQUALS
        # `clearance`, and an earlier version of this docstring overclaimed
        # that. What it rules out is the regression that matters: a 0.55
        # fallback, which neither #370 nor #617 would notice.
        _v, _p, moves_hi, _s, _o = _rig_a(2.15, _FakeSt([HBAR]))
        self.assertEqual(len(moves_hi), 1,
                         'the fallback is above 0.30 -- a 0.55 fallback lands '
                         'here, and neither #370 nor #617 would notice')
        _v2, _p2, moves_lo, _s2, _o2 = _rig_a(2.05, _FakeSt([HBAR]))
        self.assertEqual(moves_lo, [],
                         'the fallback is at or below 0.20 -- lower than the '
                         'clearance the caller passed')
        # ...and an st that declares exactly CLEAR is indistinguishable from an
        # st with no margin at all, on every arm.
        for top in (2.05, 2.15, 2.60):
            a = _rig_a(top, _FakeSt([HBAR]))[2]
            b = _rig_a(top, _MarginSt([HBAR], CLEAR))[2]
            self.assertEqual(len(a), len(b),
                             'the absent-margin path and an explicit %s '
                             'diverge at top %s' % (CLEAR, top))
    # MUTATION: `getattr(st, 'edge_margin', None)` -> `..., 0.55)` -- the 2.15
    # arm gives 0 moves. Verified: NEITHER #370 NOR #617 catches this.

    def test_the_nudger_reads_st_s_margin_rather_than_recomputing_it(self):
        st = _SpySt([HBAR], BEC)
        _v, _p, moves, _s, out = _rig_a(2.15, st)
        self.assertEqual(st.reads, 1,
                         'edge_margin was read %d times; the #725 shim block '
                         'reads each resolver exactly once, outside the '
                         'candidate sweep' % st.reads)
        # Paired with the behavioural half, so a mutant that reads the value
        # and then discards it is caught too.
        self.assertEqual(moves, [], 'the value was read but not used')
        self.assertIn('no clear spot', out)
    # MUTATION: `edge_margin = clearance` (ignore st) -> reads == 0 AND the
    # via moves.

    def test_no_edge_test_in_the_nudger_spells_the_bare_clearance(self):
        """Guards the two sites by spelling as well as behaviour, because a
        third edge test could be added tomorrow with the old convention."""
        src = inspect.getsource(FC.nudge_vias_for_unresolved).splitlines()
        bad = [f'{i + 1}: {ln.strip()}' for i, ln in enumerate(src)
               if 'need = ' in ln and 'edge_margin' not in ln]
        self.assertEqual(bad, [],
                         'an edge test still gates on something other than '
                         'edge_margin: %s' % bad)
        # ON THE BRANCH: the resolver is read at all.
        self.assertTrue(any('edge_margin' in ln for ln in src),
                        'the nudger does not mention edge_margin, so the guard '
                        'above is vacuously satisfied')
    # MUTATION: re-introduce `need = r + clearance` at either site.


class TestTheEdgeMarginResolver(unittest.TestCase):
    """resolve_cap_edge_clearance: tighten-only, and every branch reachable."""

    def _board_declaring(self, tmp, value, name):
        pcb = os.path.join(tmp, name + '.kicad_pcb')
        shutil.copyfile(BOARD, pcb)
        with open(os.path.splitext(pcb)[0] + '.kicad_pro', 'w',
                  encoding='utf-8') as f:
            json.dump({'board': {'design_settings': {
                'rules': {'min_copper_edge_clearance': value}}}}, f)
        return pcb

    def test_an_explicit_positive_value_is_honoured_in_both_directions(self):
        self.assertEqual(resolve_cap_edge_clearance(BOARD, 0.2),
                         (0.2, 'cli'),
                         'an operator asking for LESS was overridden')
        self.assertEqual(resolve_cap_edge_clearance(BOARD, 0.9),
                         (0.9, 'cli'))
    # MUTATION: drop the `if explicit is not None and explicit > 0` early return.

    def test_an_explicit_ZERO_is_unset_rather_than_a_margin_of_zero(self):
        """The footgun the #733 review found in this change: the GUI's Min Edge
        Clearance spin is CREATED at defaults.BOARD_EDGE_CLEARANCE (0.0) with a
        range minimum of 0.0, so "ticked the override, typed nothing" hands this
        a 0. Honoured literally, the cap margin would fall to max(clearance, 0)
        -- the bare clearance -- which is the 0.30mm under-block #733 closes,
        re-opened through a different door and LOOSER than the plugin's old
        0.55. Same rule resolve_cli_floor applies to every routing floor."""
        for zero in (0, 0.0, -0.1):
            self.assertEqual(
                resolve_cap_edge_clearance(BOARD, zero),
                (CAP_EDGE_CLEARANCE, 'fixed default'),
                'an explicit %r was taken as a real margin' % (zero,))
        # NEGATIVE CONTROL: a small POSITIVE value is still honoured, so the
        # guard has not swallowed the operator's override wholesale.
        self.assertEqual(resolve_cap_edge_clearance(BOARD, 0.01),
                         (0.01, 'cli'))
    # MUTATION: `explicit is not None and explicit > 0` -> `explicit is not
    # None` -- the 0 arms return (0.0, 'cli').

    def test_a_board_asking_for_more_room_raises_the_margin(self):
        with tempfile.TemporaryDirectory() as tmp:
            pcb = self._board_declaring(tmp, 0.9, 'roomy')
            # ON THE BRANCH: the declaration must exceed the default, or this
            # is the fixed-default branch wearing a different name.
            self.assertGreater(0.9, CAP_EDGE_CLEARANCE)
            self.assertEqual(resolve_cap_edge_clearance(pcb),
                             (0.9, 'board constraint, raised'))
    # MUTATION: `declared > CAP_EDGE_CLEARANCE` -> `declared is not None`
    # still passes here; it is the NEXT test that kills that one.

    def test_a_board_asking_for_less_can_never_lower_the_margin(self):
        """The reason this is tighten-only. 0.20 is not hypothetical: it is the
        fab copper-to-edge floor fix_project_for_output PINS into every board
        this pipeline writes, so a plainly board-first read would take this
        pipeline's own default back as the board's declaration."""
        with tempfile.TemporaryDirectory() as tmp:
            for value in (0.2, 0.5, 0.0):
                pcb = self._board_declaring(tmp, value, 'tight%s' % value)
                self.assertEqual(
                    resolve_cap_edge_clearance(pcb),
                    (CAP_EDGE_CLEARANCE, 'fixed default'),
                    'a declared %s LOWERED the cap margin' % value)
    # MUTATION: `declared > CAP_EDGE_CLEARANCE` -> `declared is not None and
    # declared > 0` -- the 0.2 and 0.5 arms drop the margin.

    def test_a_board_with_no_project_and_an_unreadable_one_take_the_default(self):
        self.assertEqual(resolve_cap_edge_clearance(BOARD),
                         (CAP_EDGE_CLEARANCE, 'fixed default'))
        # ON THE BRANCH: BOARD really has no sibling project, or the assertion
        # above is testing the wrong branch.
        self.assertFalse(
            os.path.exists(os.path.splitext(BOARD)[0] + '.kicad_pro'),
            'the inert board grew a .kicad_pro; pick another')
        # Not a path at all -- an unsaved GUI board reaches this.
        self.assertEqual(resolve_cap_edge_clearance(None),
                         (CAP_EDGE_CLEARANCE, 'fixed default'))
    # MUTATION: remove the try/except -> the None case raises TypeError.

    def test_the_engine_resolves_an_omitted_margin_and_says_so(self):
        """The default is None, and the ENGINE resolves it -- which is what
        makes the CLI, the plugin and the animator agree without any of them
        carrying a copy."""
        import placement.fanout_clearance as mod
        sig = inspect.signature(mod.repair_fanout_clearance)
        self.assertIsNone(sig.parameters['board_edge_clearance'].default,
                          'the engine still carries a numeric default, so a '
                          'front end that passes nothing gets it silently')
        pcb = parse_kicad_pcb(BOARD)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            mod.repair_fanout_clearance(pcb, BOARD, clearance=CLEAR,
                                        max_displacement=0.0, max_passes=1)
        self.assertIn('board-edge margin for caps: 0.55mm (fixed default)',
                      buf.getvalue(),
                      'the engine resolved the margin without disclosing it')
        # ...and it discloses an EXPLICIT one too. Printing only the resolved
        # case would disclose exactly the branch that needs no explaining, and
        # an operator reading a transcript would have to know which of three
        # fronts invoked this to know which margin it used.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            mod.repair_fanout_clearance(pcb, BOARD, clearance=CLEAR,
                                        board_edge_clearance=0.2,
                                        max_displacement=0.0, max_passes=1)
        self.assertIn('board-edge margin for caps: 0.2mm (cli)', buf.getvalue(),
                      'a caller-supplied margin is applied silently')

    def test_the_resolved_margin_actually_reaches_the_repair_state(self):
        """The wire, not the report. Everything else here either constructs
        _Repair directly with an explicit margin or reads the PRINTED line, so
        a build that resolved the margin, printed it, and then handed _Repair
        the UNRESOLVED value would pass the lot -- the "asserts a reported
        value rather than the one used" hole the #733 fact-check went looking
        for. Spy on the constructor and read what it was actually given."""
        import placement.fanout_clearance as mod
        seen = []
        real = mod._Repair

        class _Spy(real):
            def __init__(self, pcb, path, clearance, grid_step, bec, *a, **kw):
                seen.append(bec)
                super().__init__(pcb, path, clearance, grid_step, bec, *a, **kw)

        pcb = parse_kicad_pcb(BOARD)
        mod._Repair = _Spy
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                mod.repair_fanout_clearance(pcb, BOARD, clearance=CLEAR,
                                            max_displacement=0.0, max_passes=1)
                mod.repair_fanout_clearance(parse_kicad_pcb(BOARD), BOARD,
                                            clearance=CLEAR,
                                            board_edge_clearance=0.9,
                                            max_displacement=0.0, max_passes=1)
        finally:
            mod._Repair = real
        # ON THE BRANCH: the spy must have run, or `seen` is empty and every
        # assertion below is vacuously true.
        self.assertEqual(len(seen), 2, 'the _Repair spy never ran')
        self.assertEqual(seen[0], CAP_EDGE_CLEARANCE,
                         'the omitted margin was RESOLVED and PRINTED but '
                         '%r reached _Repair' % (seen[0],))
        self.assertEqual(seen[1], 0.9,
                         'an explicit margin did not reach _Repair')
    # MUTATION: pass `board_edge_clearance` (the parameter) to _Repair instead
    # of the resolved value -> seen[0] is None and this is the only test that
    # notices.
    # MUTATION: `board_edge_clearance: Optional[float] = None` -> `= 0.55`.


class TestTheFrontEndsDoNotCarryTheirOwnCopy(unittest.TestCase):
    """#733's other two facts: the CLI hardcoded 0.55 and the GUI passed
    nothing at all. Both must now defer to the engine."""

    def test_the_two_clis_default_to_none_so_the_engine_resolves(self):
        # assertTrue on a computed boolean, never assertIn against a whole
        # file: a failing assertIn prints the ENTIRE haystack, and a 100KB
        # failure message buries the one line that matters (#732's lesson).
        for script in ('py_placer/place_fanout_clearance.py',
                       'py_tools/animate_fanout_clearance.py'):
            with open(os.path.join(_ROOT, script), encoding='utf-8') as f:
                src = f.read()
            self.assertTrue(
                '"--board-edge-clearance", type=float, default=None' in src,
                '%s still carries its own numeric edge default' % script)
    # MUTATION: put `default=0.55` back in either script.

    def test_the_plugin_passes_the_margin_to_the_engine(self):
        with open(os.path.join(_ROOT, 'kicad_routing_plugin', 'fanout_gui.py'),
                  encoding='utf-8') as f:
            src = f.read()
        self.assertTrue(
            "board_edge_clearance=fanout_config.get('cap_board_edge_clearance')"
            in src,
            'the plugin does not pass the cap edge margin, so it takes the '
            'engine signature default whatever the operator asked for')
        # ...and the key is actually populated, on BOTH entry points.
        #
        # SOURCE CHANGED (#733 follow-up), intent unchanged. This used to
        # require two `'cap_board_edge_clearance': shared.get(` lines -- the
        # dialog's SHARED "Min Edge Clearance" control feeding both paths. That
        # control is the SIGNAL copper-to-edge keep-out, and because
        # resolve_cap_edge_clearance honours an explicit positive value in BOTH
        # directions, ticking it at a normal signal 0.20 dropped the cap margin
        # from 0.55 to 0.20 -- the same 0.35mm relaxation the negative control
        # below was written to prevent, arriving through the operator instead of
        # through the key. The margin now has its OWN knob on the Cap Placement
        # box, so the two CLI tools' independently-settable flags stay
        # independently settable here too.
        #
        # Both entry points therefore carry it via the PANEL config: the inline
        # path spreads the panel's cap_* keys, the apply path copies the whole
        # panel config. Asserted as the two mechanisms rather than a line count.
        self.assertIn("if k.startswith('cap_')", src,
                      'the inline path no longer spreads the panel cap_* keys, '
                      'so the cap edge margin cannot reach the engine')
        self.assertIn("dict(self.bga_options.get_config())", src,
                      'the apply path no longer copies the panel config, so the '
                      'cap edge margin cannot reach the engine')
        self.assertNotIn("'cap_board_edge_clearance': shared.get(", src,
                         'the cap margin is sourced from the SHARED signal edge '
                         'control again -- ticking Min Edge Clearance for signal '
                         'routing would loosen cap placement with it')
        # NEGATIVE CONTROL, and the trap this design had to avoid: the QFN
        # SIGNAL keep-out is a DIFFERENT key that answers 0.0 on a board with
        # no edge rule (fab-floored to 0.20 by _effective_board_edge_clearance)
        # and also feeds update_live_drc_floors. Reusing it would have inset
        # caps at 0.20 instead of 0.55 -- a 0.35mm relaxation shipped inside
        # the parity fix. Measured on the real headless dialog: unchecked, the
        # signal key reads 0.2 and the cap key reads None.
        self.assertTrue(
            "board_edge_clearance=shared.get('board_edge_clearance', 0.0)"
            in src,
            'the QFN signal keep-out kwarg changed; the cap margin was meant '
            'to be a SEPARATE key and this control no longer proves it is')
    # MUTATION: drop the board_edge_clearance kwarg from the engine call.

    def test_a_recorded_manifest_carries_an_explicit_flag_into_the_plan(self):
        sys.path.insert(0, os.path.join(_TESTS, 'stress'))
        import manifest_to_plan as M
        step = M.cap_optimization_step(
            ['py_placer/place_fanout_clearance.py', 'b.kicad_pcb',
             '--board-edge-clearance', '0.3', '--near-margin', '1.5'])
        # #772: the name CHANGED, and this arm is re-pointed rather than
        # relaxed. When it was written, the converter emitted the Basic-tab
        # SIGNAL name and the comment below justified it by
        # _GEOMETRY_OVERRIDE_CHECKS carrying that name to the engine.
        # Measured on the real headless dialog, it did not: the value set
        # the SIGNAL control, ticked its override (leaking into the next
        # step's routing), and the cap engine received None. The right name
        # is the CAP control's, and the executor now reaches it because
        # bga_options is one of the cap action's owners.
        self.assertEqual(step['params'].get('cap_board_edge_clearance'),
                         0.3,
                         'an explicit --board-edge-clearance is dropped on '
                         'replay, so the GUI plan places caps at a '
                         'different margin')
        self.assertIsNone(step['params'].get('board_edge_clearance'),
                          'the cap step still converts to the Basic tab '
                          'SIGNAL name, which is a different quantity and '
                          'whose override box then leaks into the next step')
        # ON THE BRANCH: the converter is doing real work, not returning {}.
        self.assertEqual(step['params'].get('cap_near_margin'), 1.5)
        # ...and the param name is one the plan executor can apply -- which
        # for a cap knob means the OWNER table, not the override map.
        with open(os.path.join(_ROOT, 'kicad_routing_plugin', 'ai_plan.py'),
                  encoding='utf-8') as f:
            src = f.read()
        self.assertTrue("'optimize_caps': ('fanout_tab', ('bga_options',))"
                        in src,
                        'the cap action no longer searches the panel that '
                        'owns cap_board_edge_clearance, so a plan carrying '
                        'it would be logged "no control, ignored"')
        self.assertTrue("'board_edge_clearance': 'edge_clearance_check'"
                        in src,
                        'the SIGNAL override map lost its row; a ROUTE step '
                        'carrying --board-edge-clearance would set a '
                        'disabled control and be ignored')
    # MUTATION: remove the CAP_FLAG_PARAMS row; drop bga_options from the
    # cap action's owners.


class TestTheCorpusBound(unittest.TestCase):
    """What the change can and cannot reach on boards this repo can measure."""

    def _tracked(self):
        # EVERY tracked board, not just kicad_files/ -- five more live under
        # tests/fixtures/, and an earlier version of this scan missed them while
        # the PR called its result "the tracked corpus".
        out = subprocess.run(['git', 'ls-files', '*.kicad_pcb'],
                             cwd=_ROOT, capture_output=True, text=True)
        return [os.path.join(_ROOT, p) for p in out.stdout.split()
                if p.endswith('.kicad_pcb')]

    def test_no_tracked_board_carries_a_milled_ring_this_change_could_refuse(self):
        """The cap mover exempts a milled ring a cap's own pads sit in (#628);
        the nudger has no analogue, because that exemption exists for a
        COURTYARD RECT artifact a via does not have. Bound the asymmetry: it is
        unreachable on every tracked board. When that stops being true this
        test fails, which is the point."""
        boards = self._tracked()
        self.assertGreater(len(boards), 25,
                           'git ls-files returned %d boards -- never glob the '
                           'directory, 11 boards in kicad_files/ alone are '
                           'gitignored build products' % len(boards))
        milled, ringed = [], []
        for b in boards:
            bi = parse_kicad_pcb(b).board_info
            if [c for c in (getattr(bi, 'board_edge_contours', None) or [])
                    if len(c) >= 3]:
                milled.append(os.path.basename(b))
            rings, _o, _c = board_edge_geometry(bi)
            if rings:
                ringed.append(os.path.basename(b))
        self.assertEqual(milled, [],
                         'a tracked board now carries a milled contour (%s) -- '
                         'the "#628 asymmetry is unreachable" claim in the PR '
                         'must be re-measured' % milled)
        # ON THE BRANCH: some boards DO yield rings, so the scan is real.
        self.assertTrue(ringed, 'no tracked board yielded any Edge.Cuts ring, '
                                'so this scan proves nothing')
    # MUTATION: add a milled Edge.Cuts contour to any tracked board.


if __name__ == '__main__':
    unittest.main(verbosity=2)
