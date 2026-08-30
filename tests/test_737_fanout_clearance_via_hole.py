"""#737: the relocated via's COPPER must be tested against an NPTH hole.

`nudge_vias_for_unresolved` has two sibling validators, written to mirror each
other. `connector_clear` tests the connector TRACK's copper against copper-less
drill holes; `valid_via_pos` had NO equivalent for the via it relocates. Its
only drilled-pad gate is drill-to-drill at the JLC `H2H_PAD` floor, which
measures the DRILL -- so it covered the annular ring only while

    ring = (size - drill) / 2  <=  H2H_PAD - clearance

which is **0.20 at the shipped `--clearance` 0.25** and 0 at clearance >= 0.45,
where every via binds. (The issue quotes a constant 0.25; that is the value for
the TRACK floor, and this gate is not charged at the track floor -- see below.)
Past that bound the pass parks ring copper inside a mounting hole's keep-out --
and it WRITES that via (placement/writer.py on the CLI, the plugin's own pcbnew
mirror in the GUI).

Measured at HEAD before the fix, on the rig below (`--clearance 0.1`, a 1.4/0.3
via boxed so its only escape is +x): the via relocates to (3.4500, 3.0000),
0.0500mm of copper-to-hole-WALL gap where the graded requirement is 0.1000, and
`check_drc` grades that board with its own via arm of the same rule:

    VIA-HOLE violations (1):
      Hole:net_0 (MH1.H1) <-> Via:net_1 (copper-to-hole)
        Overlap: 0.050mm    Hole: (4.70,3.00)    Via: (3.45,3.00)

The overlap IS the proof of the requirement: 0.1000 - 0.0500. After the fix no
candidate validates, `via-nudge: no clear spot` prints, and the via stays put. A
via with real room still relocates.

WHY THE FLOOR IS `clearance` AND NOT THE SIBLING'S `npth_clr`. The asymmetry is
the point, not an oversight, and it was decided by measurement after a first
version of this file shipped the sibling's floor.

`npth_clr` is `max(clearance, NPTH_TO_TRACK_CLEARANCE)` -- a routing policy for
TRACKS, which is what `connector_clear` gates. `check_drc` grades a VIA against
a plain NPTH hole at `clearance` instead
(`kicad_req = req_clr if req_clr > npth_clr else clearance`) and records why:
charging a via the track floor "invents items kicad-cli never reports" (crkbd,
7 phantoms at 0.016-0.023mm). `obstacle_map` stamps the same asymmetry -- a
plain NPTH is a track keep-out, not a via-copper one.

Charging `npth_clr` here does not merely cost search room, it COSTS THE REPAIR.
Measured, and pinned by `TestTheTrackFloorWouldAbandonARepair`: at
`--clearance 0.1` a landing 0.1500mm off the hole wall is graded CLEAN by
check_drc (`NO DRC VIOLATIONS FOUND`), the track floor refuses it, no other
candidate validates, and the cap keeps the #130 pad-via graze this pass exists
to remove. That is the exact failure `obstacle_map.resolve_hole_clearance`
names for this function BY NAME -- an all-or-nothing repair whose one clearing
candidate must not be refused -- and it is the same balance #617 struck for the
connector gate.

The two floors are EQUAL at `--clearance >= 0.20`, so this only ever differs
below the fab floor. That is not hypothetical: `place_fanout_clearance.py`'s own
`--help` example is `--clearance 0.1`, and the GUI control's minimum is 0.05.

NOT A REVERSAL OF #617, and `TestTheFloorIsTheVIARuleNotTheTRACKFloor` pins it
behaviourally: #617 measured that RAISING this pass's hole floor -- to a board's
DECLARED `min_hole_clearance` -- turns 1 move / 1 connector into 0 / 0. The
declared value is still unread here, and the floor charged is the LOWER of the
two candidates, so this pass refuses strictly less than the version review
rejected.

CLOSED SINCE, by #730: neither hole gate honoured the hole pad's own
`local_clearance`, which check_drc does honour -- a wrong VALUE where this was
a missing GATE. Each gate now has an ADDITIVE override sibling beside it, and
the two arms keep their different shapes (this one's is a STEP, the
connector's a MAX). The source guard below counts four hole gates for that
reason. Behavioural coverage lives in
tests/test_730_fanout_clearance_npth_local_clearance.py.

Conventions this file follows (#697/#725/#731/#732/#733 and CLAUDE.md):

  * Every assertion names the single-line MUTATION that must kill it.
  * Assert you are ON THE BRANCH before asserting about it.
  * Every "is refused" is paired with a NEGATIVE CONTROL that still moves.
  * No fixture in the rig below sits within 0.05mm of the quantity its
    assertion measures -- the three hole positions sit at gaps 0.050 / 0.150 /
    0.250 against a 0.200 floor, i.e. exactly 0.05 clear on both sides. Four
    tighter boundaries exist and are named here rather than left for a reviewer
    to find, because a blanket claim that ignored them would be false:
      - the CAP gate, 0.025 either side of nx = 3.425. That is the sweep's own
        0.05mm radial quantum, and no assertion here measures it.
      - #617's INHERITED rig, whose landing clears the drill gate by 0.006588
        (0.706588 vs 0.700000) -- that margin is what CHOOSES the landing, and
        it is #617's geometry, not a fixture this file picked. The arm that
        reads it asserts the landing exactly, so it is a change detector for
        precisely that.
      - the same rig's connector, 0.0200 clear of the connector's own floor
        (0.3200 vs 0.3000) -- again inherited, and asserted exactly (0.2200).
      - the corpus bound, where the widest tracked ring 0.1500 sits 0.0500
        under the 0.2000 threshold. That IS the finding, not a fixture.

A 17-mutation run over the engine gate kills 17. Getting there took two wrong
explanations of ONE mutation, `<` -> `<=`, and both are recorded because the
second is the more seductive:

  1. "no candidate lands on an exact tie" -- false. This rig generates two
     (XH_MOVES at r = 0.50 gives a gap of exactly 0.2000, and
     XH_SEPARATES_THE_FLOORS at r = 0.50 exactly 0.1000); they are simply never
     REACHED, because an earlier radius wins.
  2. "the `- 1e-4` moves the comparison boundary off the floor, so the two
     spellings are identical for EVERY input" -- also false, and it reasons
     about the wrong quantity. The tie that matters is at the COMPARISON
     boundary `floor + vr - 1e-4`, not at the floor, and that boundary is
     reachable from ordinary 1um-grid KiCad geometry.

`TestTheComparisonIsSTRICT` builds it: an NPTH hole of drill 0.500100 at
x = 4.499950 makes the helper return exactly 0.7998999999999999, which IS
`clearance + vr - 1e-4` in float. The shipped `<` accepts that candidate and
the via relocates; `<=` refuses it and the repair is lost. So the survival was
a FIXTURE GAP, not a proof of equivalence -- and the fixture now exists.

This is the one place the file's "no fixture within 0.05mm of the quantity
under test" convention is deliberately inverted: an exact tie IS the quantity
under test there, and the arm asserts the tie before asserting about it.

Eight of the seventeen were added by successive review rounds, and all eight
die: the epsilon hidden behind a trailing comment, the floor hidden behind one,
the CONNECTOR gate unified onto this gate's floor, `H2H_PAD` moved, the staged
project's declaring key mistyped, the track floor SPELT as `clearance`
(`max(clearance, 0.20)`, which the source guard alone cannot catch), the same
via the named constant, and a board-aware `getattr(st, 'npth_floor', 0.0)`.
Two FALSE-POSITIVE probes -- a comment mentioning `_seg_foreign_hole_dist(`,
and one mentioning `st.npth_floor` -- correctly change nothing, which the
earlier raw-line version of the source guard did not manage.

Dropping the `- 1e-4` is killed by the source guard only; it was measured to
leave the entire suite green on behaviour alone (it was predicted to break
test_732's DVS_BIG rig; it does not).

Runs in-process in a couple of seconds; the corpus class shells out once for
`git ls-files`.

    python3 tests/test_737_fanout_clearance_via_hole.py
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
import json
import math
import os
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

import routing_defaults as defaults
from bga_fanout.constants import DEFAULT_VIA_SIZE
from check_drc import _pad_has_no_copper
from kicad_parser import BoardInfo, parse_kicad_pcb
from single_ended_routing import _seg_foreign_hole_dist
from synth import make_pad, make_pcb, make_seg, make_via
from placement import fanout_clearance as FC
from placement.fanout_clearance import nudge_vias_for_unresolved

# --- the rig ---------------------------------------------------------------
CLEAR = 0.1                    # the VIA gate's floor, and check_drc's
                               # -> npth_clr 0.20, the SIBLING's floor
NPTH_FLOOR = defaults.NPTH_TO_TRACK_CLEARANCE       # 0.20
# Hand-mirrored: since #756 the engine's H2H_PAD is `resolve_drill_floors`'
# answer on a project-less board (what every rig here builds), not a
# literal. The guard below calls the resolver and compares; it stays
# hand-written so the thresholds cannot track a moved value in silence.
H2H_PAD = 0.45
V_SIZE, V_DRILL = 1.4, 0.3     # vr 0.70, ring 0.55 -- past the masking bound
VR = V_SIZE / 2.0
H_DRILL = 1.0                  # hr 0.50
HR = H_DRILL / 2.0
MAX_SHIFT = 0.55
VIA0 = (3.0, 3.0)              # the offending via, net 3
LANDING = (3.45, 3.0)          # the FIRST landing the walls admit

# Foreign-net (2) walls that box the search to +x, so the search is
# one-dimensional: T/B admit only |dy| <= 0.10, and L needs nx >= 3.425, so the
# first admissible ring is r = 0.45 -> (3.45, 3.00). Two further candidates are
# admitted at r = 0.50 and 0.55, both also at k = 0 -- (3.50, 3.00) and
# (3.55, 3.00). Naming LANDING "the only one" would be false, and the refusal
# arms do not rest on it being the only one. They rest on something stronger:
# every admissible candidate lies on the +x ray and the hole is FURTHER along
# it, so the copper-to-hole gap SHRINKS as r grows -- (3.45) is the roomiest of
# the three, and refusing it refuses the other two a fortiori. That is why a
# refusal arm ends with an empty `moves` rather than with a different landing.
# ALL_CANDIDATES pins the set, and the arms assert every member.
ALL_CANDIDATES = ((3.45, 3.0), (3.50, 3.0), (3.55, 3.0))
WALL_L = (0.0, 0.0, 2.625, 8.0, 2)
WALL_T = (0.0, 3.90, 8.0, 8.0, 2)
WALL_B = (0.0, 0.0, 8.0, 2.10, 2)
WALLS = [WALL_L, WALL_T, WALL_B]

# Hole x positions. Distance from the landing is XH - 3.45; the gate measures
# to the hole WALL, so the requirement is `floor + VR` on `XH - 3.45 - HR`.
XH_REFUSED_BY_BOTH = 4.70      # wall gap 0.050 -- refused at either floor
XH_SEPARATES_THE_FLOORS = 4.80  # wall gap 0.150 -- MOVES at 0.10, refused at
                               # 0.20; check_drc grades this landing CLEAN
XH_MOVES = 4.90                # wall gap 0.250 -- moves at 0.20, not at 0.40


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

    #: A REAL `st` is a `_Repair`, which carries `npth_floor` (the board-aware
    #: hole floor). The duck type must carry one too, or the most likely wrong
    #: floor -- `max(clearance, getattr(st, 'npth_floor', 0.0))` -- reads 0.0
    #: here, behaves exactly like the right one, and is caught only by the
    #: source guard. Measured: without this attribute that mutation survives
    #: every behavioural arm in this file. 0.40 is above every gap the rig
    #: uses, so a gate that reads it refuses landings the checker calls clean.
    npth_floor = 0.40

    def __init__(self, rects):
        self.caps = {'C1': _FakeCap(rects)}
        self.vias = []

    def graze_penalty(self, ref, cap, x, y, rot):
        return 1.0          # permanently unresolved, so the offender loop RUNS


def _board(xh, *, hole_net=0, hole_ref='H1REF', source_path=None,
           v_size=V_SIZE, v_drill=V_DRILL, h_drill=H_DRILL):
    bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 8.0, 8.0))
    v = make_via(VIA0[0], VIA0[1], net_id=3, size=v_size, drill=v_drill)
    stub = make_seg(2.0, 3.0, VIA0[0], VIA0[1], width=0.2, layer='F.Cu',
                    net_id=3)
    hole = make_pad(net_id=hole_net, x=xh, y=3.0, ref=hole_ref, num='H1',
                    size_x=h_drill, size_y=h_drill, shape='circle',
                    layers=['F.Mask', 'B.Mask'], drill=h_drill,
                    pad_type='np_thru_hole')
    kw = {} if source_path is None else {'source_path': source_path}
    pcb = make_pcb(board_info=bi, vias=[v], segments=[stub],
                   footprints={'C1': SimpleNamespace(layer='F.Cu', pads=[])},
                   pads_by_net={hole_net: [hole]}, zones=[], **kw)
    return v, pcb


def _nudge(st, pcb, clear=CLEAR, **kw):
    """Drive the real pass, capturing what it printed. The PRINTED OUTPUT is
    half the evidence: a refusal that prints nothing is indistinguishable from
    a pass that never looked (the #732 silent-failure lesson)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        moves, segs = nudge_vias_for_unresolved(st, pcb, clear, **kw)
    return moves, segs, buf.getvalue()


def _rig(xh, **kw):
    v, pcb = _board(xh, **kw)
    moves, segs, out = _nudge(_FakeSt(WALLS), pcb, max_shift=MAX_SHIFT)
    return v, pcb, moves, segs, out


def _wall_gap(xh, at=LANDING, hr=HR):
    """The copper-edge-to-hole-WALL gap the new gate actually measures."""
    return math.hypot(at[0] - xh, at[1] - 3.0) - hr - VR


class TestARelocatedViaHonoursTheHoleFloor(unittest.TestCase):
    """The headline, both signs."""

    def test_a_via_whose_RING_reaches_into_a_mounting_hole_is_refused(self):
        # ON THE BRANCH 1: the ring must be past the bound the DRILL test
        # masks, or this rig is a statement about some other geometry.
        self.assertGreater((V_SIZE - V_DRILL) / 2.0, H2H_PAD - CLEAR,
                           'the via ring does not exceed H2H_PAD - clearance, '
                           'so the drill test would reject this landing on '
                           'its own and the new gate is not what is measured')
        # ON THE BRANCH 2: the landing is DRILL-test-legal, so the refusal
        # below is attributable to the new gate and to nothing else.
        d_axis = abs(LANDING[0] - XH_REFUSED_BY_BOTH)
        self.assertGreaterEqual(d_axis, V_DRILL / 2.0 + HR + H2H_PAD + 0.05,
                                'the landing is inside the drill-to-drill '
                                'keep-out, so the drill test refuses it too')
        self.assertLess(_wall_gap(XH_REFUSED_BY_BOTH), CLEAR - 0.04,
                        'the landing is not inside the copper-to-hole band')
        # ON THE BRANCH 3: an empty `moves` means EVERY admissible candidate was
        # refused, not just the first. The walls admit three, all on the +x ray,
        # so the gap grows monotonically with r and the far ones are the ones
        # that could plausibly escape. Assert they do not -- otherwise this arm
        # would silently become a statement about the search budget.
        for cand in ALL_CANDIDATES:
            self.assertLess(_wall_gap(XH_REFUSED_BY_BOTH, at=cand),
                            CLEAR - 0.04,
                            'candidate %s clears the gate, so a refusal here '
                            'would be about the 0.55mm budget, not the gate'
                            % (cand,))
        v, _pcb, moves, segs, out = _rig(XH_REFUSED_BY_BOTH)
        # The PRINT first: assert it before the counts and no mutation can
        # make the silence be the reported failure.
        self.assertIn('no clear spot', out,
                      'the pass said nothing about a via it declined to move')
        self.assertEqual(moves, [], 'the via was relocated into the hole band')
        self.assertEqual(segs, [], 'connector copper was drawn for no move')
        self.assertEqual((v.x, v.y), VIA0, 'the via was mutated in place')
    # MUTATION: delete the `_seg_foreign_hole_dist` gate in valid_via_pos --
    # the via is parked at (3.4500, 3.0000) with 0.0500mm of copper-to-hole gap
    # against a 0.1000 requirement, and stdout says "moved" (check_drc:
    # VIA-HOLE, overlap 0.050mm -- which IS 0.1000 - 0.0500).

    def test_a_via_with_real_room_still_moves(self):
        """The over-rejection guard, and the negative control for every
        refusal in this file: the same rig with the hole 0.20mm further off."""
        # Clear of BOTH candidate floors, so it is a control whichever one
        # the gate charges -- it cannot be quietly invalidated by that choice.
        self.assertGreaterEqual(_wall_gap(XH_MOVES),
                                max(CLEAR, NPTH_FLOOR) + 0.04,
                                'the control landing is not clear of the gate')
        v, _pcb, moves, segs, out = _rig(XH_MOVES)
        self.assertEqual(len(moves), 1,
                         'a via with 0.25mm of copper-to-hole gap was refused '
                         'at a 0.10mm requirement -- the gate over-rejects')
        self.assertAlmostEqual(v.x, LANDING[0], places=4)
        self.assertAlmostEqual(v.y, LANDING[1], places=4)
        self.assertEqual([(s['layer'], s['width']) for s in segs],
                         [('F.Cu', 0.2)])
        self.assertIn('moved', out)
    # MUTATION: `clearance + vr` -> `clearance + 2 * vr` -- the gate
    # over-rejects and this move disappears.

    def test_the_gate_prices_the_BARREL_not_the_DRILL(self):
        """What separates #737's gate from the drill test beside it."""
        v, _pcb, moves, _segs, _out = _rig(XH_REFUSED_BY_BOTH)
        self.assertGreater(VR, FC._via_drill_radius(v) + 0.05,
                           'barrel and drill radius are too close for this '
                           'rig to tell the two conventions apart')
        # A drill-priced gate would need only 0.15 + 0.50 + 0.20 = 0.85 of
        # centre distance; the landing has 1.25, so it would accept.
        self.assertGreater(abs(LANDING[0] - XH_REFUSED_BY_BOTH),
                           V_DRILL / 2.0 + HR + CLEAR + 0.05,
                           'a drill-priced gate would refuse this landing too')
        self.assertEqual(moves, [])
    # MUTATION: `clearance + vr` -> `clearance + _via_drill_radius(v)` -- the
    # requirement falls to 0.75 of centre distance and the via moves. (#750
    # named that resolver; the mutant used to be spelled with the bare
    # `(v.drill or 0.3) / 2.0` this function no longer contains.)


class TestTheFloorIsTheVIARuleNotTheTRACKFloor(unittest.TestCase):
    """The via gate charges `clearance`; its sibling charges `npth_clr`. That
    asymmetry is deliberate -- check_drc grades a track and a via against a
    plain NPTH hole at different values, and so does obstacle_map -- and these
    arms are what stop a later reader unifying the two."""

    def test_the_floor_is_NOT_the_siblings_track_floor(self):
        # ON THE BRANCH: at clearance >= 0.20 the two coincide and this arm
        # asserts nothing at all.
        self.assertLess(CLEAR, NPTH_FLOOR,
                        'at this clearance npth_clr == clearance, so this '
                        'test cannot separate the two conventions')
        gap = _wall_gap(XH_SEPARATES_THE_FLOORS)
        self.assertGreater(gap, CLEAR + 0.04,
                           'the landing is inside the via floor too, so it '
                           'would be refused either way and this arm proves '
                           'nothing')
        self.assertLess(gap, NPTH_FLOOR - 0.04,
                        'the landing clears the track floor as well, so the '
                        'two conventions are not being separated here')
        v, _pcb, moves, segs, out = _rig(XH_SEPARATES_THE_FLOORS)
        self.assertEqual(len(moves), 1,
                         'the via gate is charging the TRACK floor: it '
                         'refused a landing check_drc grades clean')
        self.assertAlmostEqual(v.x, LANDING[0], places=4)
        self.assertEqual(len(segs), 1)
        self.assertIn('moved', out)
    # MUTATION: `clearance + vr` -> `npth_clr + vr` -- the requirement rises to
    # a 0.20mm wall gap, this landing is refused, and (see the next arm) the
    # repair is abandoned rather than merely relocated.

    def test_the_track_floor_would_ABANDON_a_repair_check_drc_calls_clean(self):
        """Why the asymmetry is not pedantry. `nudge_vias_for_unresolved` is an
        all-or-nothing repair -- obstacle_map.resolve_hole_clearance names this
        function BY NAME as one whose single clearing candidate must not be
        refused. On this rig the track floor refuses every candidate, so the
        cap would keep the #130 pad-via graze the pass exists to remove.

        check_drc's own requirement for a via against a plain NPTH hole is
        `hr + via.size/2 + clearance`, i.e. a wall gap of `clearance`.
        Reproduced with the real checker on the equivalent board:

            python3 py_router/check_drc.py rig48.kicad_pcb --clearance 0.1
            -> NO DRC VIOLATIONS FOUND
        """
        self.assertGreaterEqual(
            _wall_gap(XH_SEPARATES_THE_FLOORS), CLEAR,
            'this landing is NOT clean at check_drc s via requirement, so '
            'refusing it costs nothing and this arm is not about what it says')
        # EVERY candidate the walls admit is inside the TRACK floor, so
        # charging it loses the repair outright rather than moving it.
        for cand in ALL_CANDIDATES:
            self.assertLess(_wall_gap(XH_SEPARATES_THE_FLOORS, at=cand),
                            NPTH_FLOOR,
                            'candidate %s clears the track floor, so charging '
                            'it would relocate rather than abandon, and this '
                            'arm overstates the cost' % (cand,))
        moves = _rig(XH_SEPARATES_THE_FLOORS)[2]
        self.assertEqual(len(moves), 1,
                         'the repair was abandoned on a landing check_drc '
                         'grades CLEAN -- the exact failure mode '
                         'obstacle_map.resolve_hole_clearance warns about for '
                         'this function by name')
    # MUTATION: `clearance + vr` -> `npth_clr + vr` -- 0 moves, 0 connectors,
    # and `no clear spot` printed, on geometry the checker calls clean.

    def test_a_DECLARED_min_hole_clearance_changes_nothing(self):
        """A board declaring 0.40 would refuse this landing if the gate read
        the board; it does not, so the #130 repair is kept.

        The reason the declared value must stay unread is stronger than the
        #617 appeal it used to rest on, and it is checkable: it changes
        nothing in check_drc's VIA arm either. The declared floor enters that
        arm only through the `req_clr > npth_clr` THRESHOLD, so with a
        declared 0.40 and no pad override the graded requirement is still
        `clearance`. A gate that read the board would therefore refuse copper
        its own checker calls clean.

        `_FakeSt.npth_floor` exists for this arm: without it the
        `max(clearance, getattr(st, 'npth_floor', 0.0))` mutation reads 0.0
        and slips through every behavioural test in this file.
        """
        self.assertGreater(_wall_gap(XH_MOVES), NPTH_FLOOR + 0.04,
                           'the landing does not clear the flat floor')
        self.assertLess(_wall_gap(XH_MOVES), 0.40 - 0.04,
                        'the landing clears 0.40 as well, so a board-aware '
                        'gate would accept it and this arm is vacuous')
        with tempfile.TemporaryDirectory() as td:
            pcb_path = os.path.join(td, 'b.kicad_pcb')
            with open(pcb_path, 'w', encoding='utf-8') as f:
                f.write('(kicad_pcb (version 20240108))\n')
            with open(os.path.join(td, 'b.kicad_pro'), 'w',
                      encoding='utf-8') as f:
                json.dump({'board': {'design_settings':
                                     {'rules': {'min_hole_clearance': 0.40}}}},
                          f)
            v, pcb = _board(XH_MOVES, source_path=pcb_path)
            # VERIFY THE INPUT before trusting the output (CLAUDE.md's
            # run_utils.evidence rule). The assertion below is an ABSENCE --
            # "the declared value changed nothing" -- so a fixture that
            # declares nothing passes it forever, for the wrong reason. Mistype
            # the key and resolve_hole_clearance returns 0.0; this line is what
            # makes that a failure instead of a green.
            from obstacle_map import resolve_hole_clearance
            self.assertAlmostEqual(
                resolve_hole_clearance(pcb, None), 0.40, places=6,
                msg='the staged project does not actually declare 0.40, so '
                    'this arm would pass even if the gate DID read the board')
            moves, segs, out = _nudge(_FakeSt(WALLS), pcb, max_shift=MAX_SHIFT)
        self.assertEqual(len(moves), 1,
                         'the declared 0.40 reached this gate -- #617 '
                         'measured that raising it costs the repair entirely')
        self.assertAlmostEqual(v.x, LANDING[0], places=4)
        self.assertEqual(len(segs), 1)
        self.assertIn('moved', out)
    # MUTATION: `npth_clr` -> `st.npth_floor` (or a resolve_hole_clearance
    # read) -- this arm refuses while the project-less arm above still moves.

    def test_the_two_hole_gates_keep_their_DIFFERENT_floors(self):
        """A source guard. The two sibling validators drifting apart IS
        this issue -- but they must agree about which HOLES exist, not
        about what one COSTS: a track is charged npth_clr and a via
        clearance, and unifying them in either direction is a defect.
        Reports the offending LINES, never assertIn over the whole source
        (test_732 measured a 393KB failure message)."""
        # CODE only. `l.lstrip().startswith('#')` drops a full-line comment
        # but not a trailing one, and the engine's comment block around this
        # very gate names `_seg_foreign_hole_dist`, `npth_clr`, `st.npth_floor`
        # and `1e-4` repeatedly -- so reading raw lines both invented gates
        # (measured: one `# see _seg_foreign_hole_dist(` line took the count to
        # 3) and let a real one hide behind `npth_clr + vr:  # the 1e-4 was
        # here`, which passed every arm of this file.
        src = [l.split('#')[0]
               for l in inspect.getsource(FC.nudge_vias_for_unresolved)
               .splitlines()]
        calls = [i for i, l in enumerate(src)
                 if '_seg_foreign_hole_dist(' in l]
        self.assertEqual(len(calls), 2,
                         'expected exactly two copper-to-hole gates (the via '
                         'and the connector); found %d at function-relative '
                         'line(s) %s' % (len(calls), [i + 1 for i in calls]))
        # The two gates must spell DIFFERENT floors, and this is the guard
        # that stops a later reader "tidying" them into one. The via gate
        # (`v.net_id`) charges `clearance`; the connector gate (`net_id`, `hw`)
        # charges `npth_clr`.
        via_gate = [i for i in calls if 'v.net_id' in src[i]]
        conn_gate = [i for i in calls if i not in via_gate]
        self.assertEqual((len(via_gate), len(conn_gate)), (1, 1),
                         'could not tell the via gate from the connector gate '
                         'at function-relative line(s) %s'
                         % [i + 1 for i in calls])
        vtxt = ' '.join(src[via_gate[0]:via_gate[0] + 3])
        ctxt = ' '.join(src[conn_gate[0]:conn_gate[0] + 3])
        self.assertNotIn('npth_clr', vtxt,
                         'the VIA gate spells npth_clr -- that is the TRACK '
                         'floor, and charging it abandons repairs check_drc '
                         'grades clean')
        self.assertIn('clearance', vtxt,
                      'the VIA gate does not spell `clearance`, which is what '
                      'check_drc s via-hole arm grades at')
        # This assertion WAS the only thing in the tree holding the connector
        # floor, and that is no longer true -- amended here rather than left
        # as a stale claim. Measured when this file shipped: rewriting
        # `npth_clr + hw` to `clearance + hw` passed test_617, test_370 and
        # test_fanout_clearance (20 tests), including test_617's own
        # "UNCHANGED site" arm, which exists to pin exactly that floor.
        # #730 added a BEHAVIOURAL arm for it --
        # test_730's TestTheConnectorGateStaysAtTheFlatFabFloor puts a
        # connector 1.150mm off a hole wall, over what `clearance` would
        # require and under what `npth_clr` does -- so the mutation is now
        # killed twice, once here and once there. This guard is kept as
        # belt-and-braces: a source property is still the honest place to hold
        # a source property.
        self.assertIn('npth_clr', ctxt,
                      'the CONNECTOR gate no longer spells npth_clr -- #617 '
                      'set that floor deliberately')
        # The tolerance too. This one is pinned HERE and nowhere else, and
        # deliberately so: measured, dropping `- 1e-4` from the via gate leaves
        # the whole suite green, because it only decides an exact tie and no
        # candidate the 0.05mm spiral produces lands on one. It is kept because
        # the two sibling gates must read as one expression, which is a source
        # property, so a source guard is the honest place to hold it.
        # #730 added two OVERRIDE gates beside these two, and they carry the
        # same tolerance for the same reason -- four gates measuring the same
        # holes must read as one expression. Counted by their own call, so a
        # third scalar gate is still caught by the `len(calls) == 2` above.
        ogates = [i for i, l in enumerate(src) if 'override_hole_gap(' in l
                  and 'def ' not in l]
        self.assertEqual(len(ogates), 2,
                         'expected exactly two #730 override-hole gates (via '
                         'and connector); found %d at function-relative '
                         'line(s) %s' % (len(ogates), [i + 1 for i in ogates]))
        eps = [i + 1 for i in calls + ogates
               if '1e-4' not in ' '.join(src[i:i + 3])]
        self.assertEqual(eps, [],
                         'a hole gate dropped the 1e-4 the others carry, '
                         'at function-relative line(s) %s' % eps)
        # THE BOARD READ. This was a blanket ban until #730, and the ban was
        # coarser than the decision it encoded. #617 refused board-deriving a
        # FLOOR, because a raised floor abandons landings check_drc grades
        # clean -- measured, and still measured: declared 0.25 gives 0 moves
        # and 0 connectors where the flat floor gives 1 and 1.
        #
        # #730's `npth_step` is a THRESHOLD, not a floor. It decides whether a
        # pad's own `(clearance ...)` outranks the fab floor, so raising it
        # makes the override gate fire LESS often -- it can only ever charge
        # less, never more, which is the opposite direction. And the direction
        # is not the whole argument: what keeps that gate from costing repairs
        # is that it fires only for an explicit, author-declared per-pad
        # keep-clear, never for a board-wide floor. That is the distinction
        # from #617.
        #
        # So the guard is now POSITIONAL, because "which direction" is a
        # property of the EXPRESSION and not of the identifier:
        # `npth_step = max(npth_clr, resolve_hole_clearance(...))` relaxes and
        # `npth_clr = max(npth_clr, resolve_hole_clearance(...))` -- literally
        # the #617 mutation -- raises, and a guard phrased as "board reads are
        # fine" would pass both.
        board = [i for i, l in enumerate(src)
                 if 'resolve_hole_clearance(' in l and 'import' not in l]
        self.assertEqual(len(board), 1,
                         'expected exactly one board-derived hole read (#730 s '
                         'step threshold); found %d at function-relative '
                         'line(s) %s' % (len(board), [i + 1 for i in board]))
        self.assertRegex(src[board[0]].lstrip(), r'^npth_step\s*=',
                         'the board-derived read is not the `npth_step` '
                         'assignment -- #617 refused it in every other '
                         'position, at function-relative line %d'
                         % (board[0] + 1))
        # ...and it must not be LAUNDERED into the floor. Pinning the read to
        # `npth_step` is not enough on its own: `npth_clr = npth_step` on the
        # next line reinstates #617's mutation while every check above still
        # passes, and a review found exactly that. So pin the floor's
        # assignment count too -- one, and the shape guard in test_730 says
        # what that one must be.
        flat = [i + 1 for i, l in enumerate(src)
                if l.lstrip().startswith('npth_clr =')]
        self.assertEqual(len(flat), 1,
                         'npth_clr is assigned %d times (at function-relative '
                         'line(s) %s) -- a second assignment is how a board-'
                         'derived value reaches the floor #617 kept flat'
                         % (len(flat), flat))
        # ...and no GATE may add it. A threshold that leaks into a gate line is
        # a floor wearing a threshold's name.
        for i in calls + ogates:
            self.assertNotIn('npth_step', ' '.join(src[i:i + 3]),
                             'a hole gate ADDS the board-derived value at '
                             'function-relative line %d -- threshold only'
                             % (i + 1))
        # `st.npth_floor` stays banned outright, and for its OWN reason rather
        # than #617's: this function is public and its harnesses pass duck
        # types, so `getattr(st, 'npth_floor', ...)` silently takes the
        # default -- and the default is the LOW threshold, the direction that
        # costs repairs. It is also the wrong number, omitting `clearance`.
        # `pcb_data` is a real argument on every path, so the read goes there.
        drift = [i + 1 for i, l in enumerate(src) if 'npth_floor' in l]
        self.assertEqual(drift, [],
                         'the nudger reads st.npth_floor at function-relative '
                         'line(s) %s -- a duck-typed harness carries none, so '
                         'the getattr default decides silently' % drift)
        # And the one constant this file HAND-MIRRORS. Since #756 `H2H_PAD`
        # is `resolve_drill_floors`' answer rather than a literal, so the
        # mirror is checked against the resolver on a project-less board --
        # which is what every rig here builds, and which takes the fab floor.
        # Three ON-THE-BRANCH guards above reason about the drill test using
        # the mirror and would silently reason about the wrong number if the
        # engine's moved. The literal ban is the other half: a value check
        # alone would pass a resolver hard-wired back to 0.45.
        self.assertEqual(
            FC.resolve_drill_floors(
                make_pcb(board_info=BoardInfo(layers={},
                                              copper_layers=['F.Cu', 'B.Cu'],
                                              board_bounds=(0.0, 0.0, 4.0, 4.0)),
                         source_path=''))[1],
            H2H_PAD,
            "the engine's pad drill floor on a project-less board is no "
            "longer %s, so this file's mirror -- and the drill-test "
            'arithmetic in every ON-THE-BRANCH guard here -- is stale'
            % H2H_PAD)
        self.assertEqual(
            [i + 1 for i, l in enumerate(src)
             if l.lstrip().startswith('H2H_PAD =')], [],
            'H2H_PAD is a literal again -- #756 exists because it was')
    # MUTATION: re-spell either gate's floor, drop any of the four 1e-4s, add
    # a fifth hole gate, move the board read out of `npth_step`, or assign
    # `npth_clr` twice -- one of the counts or lists above changes. (The
    # trailer used to name a `bad` list; there has never been one in this
    # method. Corrected while #730 rewrote the guard around it.) This arm is
    # still the ONLY thing that kills the 1e-4 mutation; `<` -> `<=` is killed
    # by nothing at all, and is left declared rather than papered over,
    # because any fixture that could catch it would sit on the threshold.


class TestTheComparisonIsSTRICT(unittest.TestCase):
    """`<`, not `<=`. The only mutation this file could not kill for two
    rounds, because both explanations offered for its survival were wrong (see
    the module docstring). A gap exactly ON the comparison boundary
    `clearance + vr - 1e-4` is reachable from ordinary 1um-grid geometry, and
    the two spellings disagree there."""

    #: Chosen so `_seg_foreign_hole_dist` returns the boundary EXACTLY in
    #: float. Both are 1um-grid values a real board could carry.
    TIE_DRILL = 0.500100
    TIE_X = 4.499950

    def _tie_board(self):
        bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                       board_bounds=(0.0, 0.0, 8.0, 8.0))
        v = make_via(VIA0[0], VIA0[1], net_id=3, size=V_SIZE, drill=V_DRILL)
        stub = make_seg(2.0, 3.0, VIA0[0], VIA0[1], width=0.2, layer='F.Cu',
                        net_id=3)
        hole = make_pad(net_id=0, x=self.TIE_X, y=3.0, ref='H1REF', num='H1',
                        size_x=self.TIE_DRILL, size_y=self.TIE_DRILL,
                        shape='circle', layers=['F.Mask', 'B.Mask'],
                        drill=self.TIE_DRILL, pad_type='np_thru_hole')
        pcb = make_pcb(board_info=bi, vias=[v], segments=[stub],
                       footprints={'C1': SimpleNamespace(layer='F.Cu',
                                                         pads=[])},
                       pads_by_net={0: [hole]}, zones=[])
        return v, pcb

    def test_a_candidate_exactly_ON_the_boundary_is_ACCEPTED(self):
        """Deliberately inverts this file's own no-fixture-on-a-threshold rule:
        an exact tie IS the quantity under test here."""
        v, pcb = self._tie_board()
        # ON THE BRANCH: assert the tie before asserting about it. Float
        # equality is the point, so `assertEqual`, not assertAlmostEqual.
        boundary = CLEAR + VR - 1e-4
        self.assertEqual(
            _seg_foreign_hole_dist(pcb, 3, LANDING[0], LANDING[1],
                                   LANDING[0], LANDING[1]), boundary,
            'the fixture no longer lands on the comparison boundary, so this '
            'arm cannot tell `<` from `<=` and proves nothing')
        moves, segs, out = _nudge(_FakeSt(WALLS), pcb, max_shift=MAX_SHIFT)
        self.assertEqual(len(moves), 1,
                         'a candidate exactly ON the boundary was refused -- '
                         'the comparison is `<=`, and a gap equal to the '
                         'requirement is legal, not a violation')
        self.assertAlmostEqual(v.x, LANDING[0], places=4)
        self.assertEqual(len(segs), 1)
        self.assertIn('moved', out)
    # MUTATION: `<` -> `<=` -- this candidate is refused, `no clear spot`
    # prints, and the repair is lost. Nothing else in the file catches it.


class TestTheTwoSiblingValidatorsSeeTheSameHoles(unittest.TestCase):
    """Why the gate calls the shared helper instead of reusing `board_pads`."""

    @staticmethod
    def _board_pads_would_see(pcb, cap_refs):
        """`board_pads`' own filter, rebuilt here so the claim below is
        measured rather than asserted from reading the source."""
        return [p for ps in pcb.pads_by_net.values() for p in ps
                if getattr(p, 'component_ref', None) not in cap_refs]

    def test_an_NPTH_hole_on_a_MOVABLE_CAP_is_still_seen(self):
        """`board_pads` drops the pads of movable caps; the helper keeps them.
        This is the one arm that separates the two implementations on
        BEHAVIOUR. The source guard co-kills that mutation for a structural
        reason -- any inline version necessarily removes a
        `_seg_foreign_hole_dist(` call site -- so "the only thing that catches
        it" would be false; this is the only thing that catches it by running
        the pass."""
        v, pcb, moves, segs, out = _rig(XH_REFUSED_BY_BOTH, hole_ref='C1')
        # ON THE BRANCH: the pad really is invisible to a board_pads-based
        # gate, and really is visible to the helper.
        self.assertEqual(self._board_pads_would_see(pcb, {'C1'}), [],
                         'the hole pad is NOT filtered out of board_pads, so '
                         'this rig does not exercise the scope difference')
        self.assertAlmostEqual(
            _seg_foreign_hole_dist(pcb, 3, LANDING[0], LANDING[1],
                                   LANDING[0], LANDING[1]), 0.75, places=4,
            msg='the helper does not see this hole either, so the outcome '
                'below is not about the scope difference')
        self.assertIn('no clear spot', out)
        self.assertEqual((moves, segs), ([], []))
        self.assertEqual((v.x, v.y), VIA0)
    # MUTATION: reimplement the gate inline over `board_pads` (reusing its
    # `cap_` capsule) -- this arm moves. The source guard also fires, on the
    # missing call site; this is the only BEHAVIOURAL arm that does.

    def test_a_hole_on_the_VIAS_OWN_NET_is_exempt(self):
        """NEGATIVE CONTROL for the net filter. A via on the hole's own net
        legitimately lands there; check_drc's via arm exempts it the same way
        (`if via.net_id == hnet: continue`)."""
        v, pcb, moves, segs, out = _rig(XH_REFUSED_BY_BOTH, hole_net=3)
        self.assertEqual(
            _seg_foreign_hole_dist(pcb, 3, LANDING[0], LANDING[1],
                                   LANDING[0], LANDING[1]), 1e9,
            'the helper still sees this hole, so the exemption under test is '
            'not the reason for the outcome below')
        self.assertEqual(len(moves), 1,
                         'an OWN-NET mounting hole blocked its own net')
        self.assertEqual(len(segs), 1)
        self.assertAlmostEqual(v.x, LANDING[0], places=4)
        self.assertIn('moved', out)
        # POSITIVE CONTROL, identical geometry on a foreign net.
        self.assertEqual(_rig(XH_REFUSED_BY_BOTH)[2], [],
                         'the foreign-net arm moves too, so this rig does not '
                         'separate own-net from foreign-net at all')
    # MUTATION: pass `0` (or drop the net argument) instead of `v.net_id` --
    # a net-tied mounting hole blocks its own net's via and this arm refuses.


class TestTheDrillTestIsUnchanged(unittest.TestCase):
    """#617's own rig, verbatim. The change detector that says the #130 repair
    is kept -- and the measurement that licenses the change."""

    NPTH = (1.0, 0.98)
    ND = 0.2
    BAR = (0.2, 0.9, 1.8, 1.1, 2)

    def _rig617(self):
        bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                       board_bounds=(0.0, 0.0, 3.0, 3.0))
        v = make_via(1.0, 1.4, net_id=3, size=0.5, drill=0.3)
        stub = make_seg(1.0, 1.6, 1.0, 1.4, width=0.2, layer='F.Cu', net_id=3)
        hole = make_pad(net_id=0, x=self.NPTH[0], y=self.NPTH[1], ref='BUS1',
                        num='H1', size_x=self.ND, size_y=self.ND,
                        shape='circle', layers=['F.Mask', 'B.Mask'],
                        drill=self.ND, pad_type='np_thru_hole')
        pcb = make_pcb(board_info=bi, vias=[v], segments=[stub],
                       footprints={'C1': SimpleNamespace(layer='F.Cu',
                                                         pads=[])},
                       pads_by_net={0: [hole]}, zones=[])
        return v, pcb

    def test_the_617_rig_relocates_to_exactly_the_same_place(self):
        v, pcb = self._rig617()
        moves, segs, out = _nudge(_FakeSt([self.BAR]), pcb)
        self.assertEqual((len(moves), len(segs)), (1, 1),
                         'the #130 pad-via repair #617 measured was traded '
                         'away by this change')
        self.assertAlmostEqual(v.x, 1.1148, places=4)
        self.assertAlmostEqual(v.y, 1.6772, places=4)
        self.assertIn('moved', out)
        gap = _seg_foreign_hole_dist(
            pcb, segs[0]['net_id'], segs[0]['start'][0], segs[0]['start'][1],
            segs[0]['end'][0], segs[0]['end'][1]) - segs[0]['width'] / 2.0
        self.assertAlmostEqual(gap, 0.22, places=4,
                               msg='#617 records this connector landing at '
                                   '0.2200mm; it moved')
    # MUTATION: any change to the gate that rejects this landing -- the counts
    # go to (0, 0) and the refusal line appears instead.

    def test_the_new_gate_is_numerically_unreachable_on_a_NORMAL_via(self):
        """Why #617 stays green, as an inequality rather than a hope: on a
        0.5/0.3 via the DRILL test is the stricter of the two, so the new gate
        can never be the binding one there."""
        drill_need = 0.3 / 2.0 + self.ND / 2.0 + H2H_PAD          # 0.70
        gate_need = 0.5 / 2.0 + self.ND / 2.0 + CLEAR             # 0.45
        self.assertGreater(drill_need, gate_need + 0.20,
                           'the two requirements are close enough that the '
                           'new gate could become the binding one here')
        v, pcb = self._rig617()
        _nudge(_FakeSt([self.BAR]), pcb)
        d = math.hypot(v.x - self.NPTH[0], v.y - self.NPTH[1])
        self.assertGreater(d, gate_need + 0.20,
                           'the achieved landing has less than 0.20mm of '
                           'headroom on the new gate')
        self.assertAlmostEqual(d, 0.706588, places=5)
    # MUTATION: none kills these -- they are the measurement that licenses the
    # change. Their job is to fail loudly if a later edit moves the landing.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """A self-expiring bound. The gate is UNREACHABLE on every board this repo
    tracks, which is why the demonstration above is synthetic on purpose."""

    def _tracked(self):
        """The boards git TRACKS. A checkout with no git, or a source export
        outside a repo, cannot identify that set at all -- and must SKIP rather
        than grade against a set it cannot name. Before this, a `git ls-files`
        that exited 128 with empty output failed the count assertion below and
        reported the gitignored-build-products reason, which is the WRONG
        cause: `out.returncode` was captured and never read."""
        try:
            out = subprocess.run(['git', 'ls-files', '-z', '*.kicad_pcb'],
                                 cwd=_ROOT, capture_output=True, text=True)
        except OSError as e:
            self.skipTest('SKIP: cannot run git to identify the tracked '
                          'corpus: %s' % e)
        if out.returncode != 0:
            self.skipTest('SKIP: git ls-files exited %d (%s) -- the tracked '
                          'corpus cannot be identified here'
                          % (out.returncode, out.stderr.strip()[:120]))
        return [os.path.join(_ROOT, q) for q in out.stdout.split(chr(0))
                if q.endswith('.kicad_pcb')]

    def test_no_tracked_via_is_fat_enough_for_the_gate_to_bind(self):
        boards = self._tracked()
        self.assertGreater(len(boards), 25,
                           'git ls-files returned %d boards -- never glob the '
                           'directory, 11 boards in kicad_files/ alone are '
                           'gitignored build products' % len(boards))
        widest, where, total, with_both = 0.0, None, 0, []
        for b in boards:
            pcb = parse_kicad_pcb(b)
            vias = [v for v in pcb.vias if (v.size or 0) > 0]
            holes = [p for ps in pcb.pads_by_net.values() for p in ps
                     if (getattr(p, 'drill', 0) or 0) > 0
                     and _pad_has_no_copper(p)]
            total += len(vias)
            if not vias:
                continue
            r = max((v.size - (v.drill or 0)) / 2.0 for v in vias)
            if r > widest:
                widest, where = r, os.path.basename(b)
            if holes:
                with_both.append(os.path.basename(b))
        self.assertGreater(total, 500,
                           'only %d sized vias across the corpus -- the bound '
                           'below would be vacuous' % total)
        bound = H2H_PAD - defaults.CLEARANCE                       # 0.20
        # Compared against the bound ITSELF, with the margin reported. An
        # arbitrary safety subtraction here would be the tightest number in
        # the file (0.1500 against 0.1600) while LOOKING like the loosest --
        # and it is the bound, not a safety band, that the claim is about.
        self.assertLess(widest, bound,
                        'a tracked board now carries a via ring of %.4f (%s), '
                        'at or over the %.4f bound where this gate starts to '
                        'bind at the shipped --clearance %.2f. The "provably '
                        'inert on the corpus" claim in this PR must be '
                        're-measured.' % (widest, where, bound,
                                          defaults.CLEARANCE))
        self.assertGreater(bound - widest, 0.04,
                           'the widest tracked ring %.4f is now within 0.04mm '
                           'of the %.4f bound -- still inert, but the margin '
                           'this PR reported as 0.0500 has shrunk'
                           % (widest, bound))
        # The boards carrying BOTH vias and copper-less holes are the only
        # ones where this gate could ever fire; name them, so a change there
        # is visible rather than silent.
        self.assertEqual(sorted(with_both),
                         ['orangecrab_ext_pll.kicad_pcb',
                          'rp2350_fpga_eensy_prePlane.kicad_pcb'],
                         'the set of boards carrying both vias and '
                         'copper-less holes changed; re-run the corpus A/B')
    # MUTATION: add a via with size - drill > 0.32 to any tracked board -- the
    # inertness claim stops being true and this fails with the board's name.

    def test_every_via_this_pipeline_CREATES_is_below_the_bound(self):
        """The corpus is a snapshot; these are the sizes the tool itself
        emits, and they are what keep the gate inert on boards it produces."""
        bound = H2H_PAD - defaults.CLEARANCE
        for name, size, drill in (
                ('BGA_VIA', defaults.BGA_VIA_SIZE, defaults.BGA_VIA_DRILL),
                ('VIA', defaults.VIA_SIZE, defaults.VIA_DRILL),
                ('DEFAULT_VIA_SIZE', DEFAULT_VIA_SIZE, defaults.VIA_DRILL)):
            with self.subTest(name):
                self.assertLess((size - drill) / 2.0, bound - 0.04,
                                '%s (%s/%s) now has a ring at the %.4f bound'
                                % (name, size, drill, bound))
    # MUTATION: raise BGA_VIA_SIZE past 0.62 -- the fanout vias this pass
    # moves start binding and the inertness claim must be restated.


if __name__ == '__main__':
    unittest.main(verbosity=2)
