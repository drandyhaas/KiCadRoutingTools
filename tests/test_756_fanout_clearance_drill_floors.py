#!/usr/bin/env python3
"""#756: the via-nudge's DRILL-to-drill floors must be the board's, not two
hand-copied literals -- and PREFERRED rather than imposed.

`nudge_vias_for_unresolved` spelled them as function-locals::

    H2H_VIA = 0.2    # JLC via-hole to via-hole floor
    H2H_PAD = 0.45   # JLC via-hole to pad-hole floor

hand-copied from `fab_tiers` and reading nothing -- not the board's declared
`min_hole_to_hole`, not `--fab-tier`, not a `--fab-overrides` file. Meanwhile
`check_drc` grades the same geometry at `max(0.20, min_hole_to_hole)`: `_pin_up`
(check_drc.py:3686) raises the one knob and BOTH drill arms add it (via/via
:2605, pad/via :2667). So on a declaring board this pass parked a relocated
barrel at a spacing its own checker then flagged.

WHAT SHIPPED IS A LADDER, NOT A RAISE, and that distinction is the whole design.
A one-rung raise was written first, measured, and REJECTED: an adversarial
review swept 8673 configurations of this file's own rig shape and found 625 that
LOSE the repair at the shipped 0.6mm budget -- 13 of them abandoning a landing
`check_drc` grades CLEAN, i.e. printing "no clear spot" while leaving a
violation the flat floor had CURED. `drill_ladder` is `[declared]` or
`[declared, fab]`; the sweep runs in full at rung 1 and descends only when it
cleared nothing, so rung 2 IS the pre-#756 behaviour and the pass cannot lose a
repair it used to make. `TestTheLadderCannotLoseARepair` measures that rather
than asserting it.

That also RETIRES the objection this change had to answer.
`obstacle_map.resolve_hole_clearance` names this function by name as an
all-or-nothing repair whose floor must not be raised, and `_Repair.__init__`
states the same rule. Both are still true; the site simply stopped being
all-or-nothing for this floor.

TWO CLAIMS AN EARLIER DRAFT OF THIS FILE MADE ARE FALSE, recorded so nobody
restores them:

  * "the via/via drill gate is the ONE floor in this function sitting strictly
    BELOW its own grader". `check_drc` also `_pin_up`s `hole_clearance` from
    `min_hole_clearance` (:3697) while the CONNECTOR's copper-to-hole gate here
    stays flat, so on #617's own declaring rig the connector this pass emits
    grades as a 0.030mm violation. That gap is #617's measured, disclosed
    choice; it is not a counterexample to #756 and #756 is not a licence to
    close it by raising.
  * "the pad arm stays 0.25mm stricter than the grader, and #756 does not
    change that". The margin is `max(d, 0.45) - max(0.20, d)`, which DECAYS --
    0.25 at d <= 0.20, 0.15 at 0.30, 0 at 0.45 and above. #756 does change it,
    toward agreement. Pinned by `test_the_pad_arm_margin_DECAYS`.

WHAT #756's OWN ISSUE GETS WRONG, corrected here rather than repeated. It says
"`H2H_VIA` has no test that keys on it anywhere in the repo". False, and
`tests/test_750_...py:52-64` already records the identical claim as one nobody
should restore. What is true is narrower -- no assertion keys on it IN test_732,
whose rig parks two vias 0.28mm apart on the SAME net so H2H_VIA alone decides
the landing, but which asserts `moves[0][0]`, the PRE-move x. So the arms below
are not the first H2H_VIA assertions; they are the first BOARD-DERIVED ones.

Conventions (from #725/#731/#732/#733/#737/#750 and CLAUDE.md): REAL parser
dataclasses; every assertion names the single-line MUTATION that must kill it,
with the count the battery measured; assert you are ON the branch before
asserting about it; every refusal paired with an acceptance that still happens.
The battery ships as `tests/mutate_756.py`.

FIVE ARMS HERE EXIST BECAUSE A FACT-CHECK FOUND THEM MISSING OR HOLLOW, and are
labelled where they sit: the source-grep arm was satisfied by a COMMENT until it
started comment-stripping like its siblings; both disclosure arms passed with
their two numbers swapped; the bga acceptance fixtures sat exactly 1e-6 from the
gate, so the engine's own epsilon was their entire margin; nothing exercised the
`--fab-overrides` channel this docstring indicts the old literals for ignoring;
and nothing drove the PAD gate with a board-raised floor, so the pad half of the
resolver was a return value nobody spent.

MUTATION BATTERY, 31 rows across TWO targets: 31 KILLED, 0 survived, 0 broken.
The counts make the `# MUTATION:` notes checkable rather than decorative, so
they are recorded here and the battery ships as `tests/mutate_756.py`:

    pad-floor-reads-the-VIA-fab-key                    22 assertions
    via-floor-reads-the-PAD-fab-key                    18
    resolver-hard-wired-to-the-old-literals            12  the source-guard evasion
    h2h-via-gate-deleted                               11  + perturbs test_750
    resolver-never-reads-the-board                     10  THE DEFECT
    via-loop-self-skip-deleted                          6
    resolver-drops-the-fab-wrap                         4
    resolver-drops-the-fab-wrap-on-the-PAD-arm-only     4
    ladder-collapsed-to-the-fab-rung-only               4      ladder-order-reversed                               4   |  THE LADDER: each
    ladder-collapsed-to-the-declared-rung-only          3   |  restores the class
    sweep-does-not-descend-the-ladder                   2  /   #756 rejected
    bga-pad-arm-adopts-the-nudgers-045                  4  the refused tidy-up
    pad-gate-charges-the-VIA-floor                      3
    bga-never-reads-the-board                           3
    resolver-deaf-to-the-fab-tier-and-overrides         2  had NO test before
    disclosure-fires-at-the-packaged-default-too        2
    bga-pad-arm-reverted-to-the-flat-constant           2
    resolver-drops-the-fab-wrap-on-the-VIA-arm-only     1
    cwd-probe-guard-dropped                             1
    cwd-probe-guard-keeps-empty-but-drops-isdir         1
    fallback-passed-instead-of-None                     1
    raised-disclosure-deleted                           1
    raised-disclosure-swaps-its-two-floors              1  had NO test before
    below-fab-disclosure-deleted                        1
    below-fab-disclosure-names-the-declared-value...    1  had NO test before
    fallback-is-silent                                  1
    bga-via-arm-reverted-to-the-flat-constant           1  had NO test before
    bga-drops-the-fab-wrap                              1  had NO test before
    bga-cwd-probe-guard-dropped                         1  the HIGH finding
    bga-announces-on-a-call-with-no-routes              1

NO EXPECTED SURVIVORS, unlike the sibling batteries -- and that is a change from
the first #756 run, which had two. Both described the old one-rung shape (an
assignment that could be moved above the early return; a layer bucket that could
be forced) and neither mutation exists against the ladder. The layer-count
inertness is still recorded, as an assertion in `TestTheLayerCountChoice` rather
than as a row.

THE BATTERY FOUND FIVE DEFECTS IN THIS FILE AND IN ITSELF, corrected in place
rather than quietly fixed, because that is what the counts are evidence of:

  * THREE bga rows reported BROKEN on a line-ending mismatch: an LF anchor
    matched zero times against a CRLF working tree. NOT a property of the
    files -- both are LF in git (`.gitattributes`: `*.py text eol=lf`), and a
    fact-check refuted the first telling of this, which said the two targets
    disagree. What converts a tree is an edit written through Python's text
    mode on Windows. The runner now translates the anchor to whatever the
    target on disk uses, so BROKEN keeps meaning "stale anchor".
  * `bga-drops-the-fab-wrap` SURVIVED: `manage_vias` hands `board_floor`
    HOLE_TO_HOLE_CLEARANCE as its FALLBACK, so only a sub-fab declaration
    exercises the wrap and no arm made one.
  * `bga-via-arm-reverted-to-the-flat-constant` SURVIVED: of that function's
    TWO drill arms the rig only ever presented a PAD drill.
  * `bga-announces-on-a-call-with-no-routes` SURVIVED: the arm passed an
    UNDECLARED board, so the branch short-circuits before the `routes` term and
    removing it changes nothing.
  * `assignment-moved-back-above-the-early-return` was KILLED against its
    expectation, because the row INSERTED a second assignment rather than
    moving the one that was there.

Runs in-process; ~40-90 s depending on the machine and on contention (the ladder
sweep dominates). The only shelling out is one `git ls-files`.

    python3 tests/test_756_fanout_clearance_drill_floors.py
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 1200

import contextlib
import inspect
import io
import json
import math
import os
import shutil
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

import run_utils
import fab_tiers
import routing_defaults as defaults
from fab_tiers import fab_floor_min
from kicad_parser import BoardInfo, Pad, Segment, parse_kicad_pcb
from list_nets import read_design_rules
from placement import fanout_clearance as FC
from placement.fanout_clearance import (nudge_vias_for_unresolved,
                                        repair_fanout_clearance,
                                        resolve_drill_floors)
from synth import make_pcb, make_via

# --- the rig ---------------------------------------------------------------
CLEAR = 0.1
VIA = (1.0, 1.4)
BAR = (0.2, 0.9, 1.8, 1.1, 2)      # a FOREIGN cap pad the via grazes
DRILL_R = 0.15                     # both vias carry synth's default 0.3 drill

# The fab floors, IMPORTED rather than mirrored -- saying so because a reader
# who believes they are mirrored will look for a source guard that should not
# exist. #756 is precisely the change that made them importable: before it they
# were function-local literals and three test files hand-copied them.
#
# AND THAT MEANS EVERY `== FAB_VIA` BELOW COMPARES THE ENGINE TO ITS OWN TABLE.
# A fact-check measured the consequence: moving `pad_hole_to_hole` 0.45 -> 0.40
# inside `fab_tiers` leaves this whole file green, while test_730/737/750 (which
# hand-write the mirror) all go red. That is the intended division of labour --
# the siblings own "is the number still 0.45", this file owns "does the board
# reach it" -- but it should not be read as this file pinning the fab table.
FAB = fab_floor_min(2)
FAB_VIA, FAB_PAD = FAB['hole_to_hole'], FAB['pad_hole_to_hole']

# The landing the rig's sweep reaches when nothing constrains it. Pinned by
# `test_ON_THE_BRANCH_the_free_landing_is_where_this_file_says`, which CALLS the
# engine; the offsets below are derived from it arithmetically.
FREE_LANDING = (1.0707, 1.4707)

# Neighbour offsets whose free-landing drill gap is the named value. Derived
# from FREE_LANDING, not guessed: gap = hypot(0.0707, D - 0.0707) - 0.30.
OFFSET_FOR_GAP = {0.21: 0.5758, 0.24: 0.6061, 0.26: 0.6262, 0.29: 0.6564}


def _bi(bounds=(0.0, 0.0, 4.0, 4.0), layers=('F.Cu', 'B.Cu')):
    return BoardInfo(layers={}, copper_layers=list(layers),
                     board_bounds=bounds)


def _projectless():
    """The board every rig in the sibling files builds: real BoardInfo, no
    `source_path`, so the resolver takes the fab floors without a disk read."""
    return make_pcb(board_info=_bi(), source_path='')


def _nudger_src():
    """COMMENT-STRIPPED, and that is not a detail.

    `test_737:493-495` and `test_750:662-663` both strip before grepping this
    function, and say why. The arm below did not, and a fact-check showed it
    passing while the code it names had been DELETED -- because the needle also
    occurs in the prose two thousand lines above. Any source assertion about
    this function strips first.
    """
    return [l.split('#')[0] for l in
            inspect.getsource(FC.nudge_vias_for_unresolved).splitlines()]


class _FakeCap:
    """Minimal stand-in for _Cap, the #370 B3 harness shape."""

    def __init__(self, rects):
        self._rects = list(rects)
        self.side = 'F'
        self.x = self.y = self.rot = 0.0

    def pad_rects(self, x=None, y=None, rot=None):
        return self._rects


class _FakeSt:
    """Minimal stand-in for _Repair: one cap, permanently 'unresolved'."""

    def __init__(self, rects):
        self.caps = {'C1': _FakeCap(rects)}
        self.vias = []

    def graze_penalty(self, ref, cap, x, y, rot):
        return 1.0


def _stub_board(tmp, name, rules=None):
    """A bare board file, plus a sibling project when `rules` is not None.

    `rules={}` writes a project that EXISTS and declares nothing -- a different
    case from no project at all, and the one an earlier draft of this file
    named in two arms while building neither (the fixture skipped the write
    whenever the dict was falsy, so "a project that declares nothing" was
    really "no project"). Only the PROJECT has to exist on disk; the PCBData is
    synthetic.
    """
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20240108))\n')
    if rules is not None:
        with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump({'board': {'design_settings': {'rules': rules}}}, f)
    return pcb


def _nudge_rig(path, neighbour_offset, drill=0.3):
    """The via the cap grazes, plus a SAME-NET neighbour parked directly above.

    Same net on purpose: the copper term (`ov.net_id != v.net_id`) then
    short-circuits and the via/via DRILL gate ALONE decides how far up the
    mover may go -- the shape tests/test_732's `_rig` uses for the same reason.
    The neighbour sits 0.8mm+ from the bar at every offset tried, so it is
    never itself an offender and exactly one via moves.
    """
    v = make_via(VIA[0], VIA[1], net_id=3, drill=drill)
    nb = make_via(VIA[0], VIA[1] + neighbour_offset, net_id=3, drill=drill)
    stub = Segment(start_x=VIA[0], start_y=VIA[1] - 0.2, end_x=VIA[0],
                   end_y=VIA[1], width=0.2, layer='F.Cu', net_id=3)
    pcb = make_pcb(board_info=_bi((0.0, 0.0, 6.0, 6.0)), vias=[v, nb],
                   segments=[stub],
                   footprints={'C1': SimpleNamespace(layer='F.Cu', pads=[])},
                   pads_by_net={}, source_path=path, zones=[])
    return v, nb, pcb


def _nudge(path, neighbour_offset, max_shift=0.6, drill=0.3):
    """Returns (moves, segs, landing_or_None, drill_gap_or_None, printed)."""
    v, nb, pcb = _nudge_rig(path, neighbour_offset, drill)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        moves, segs = nudge_vias_for_unresolved(_FakeSt([BAR]), pcb, CLEAR,
                                                max_shift=max_shift)
    if not moves:
        return moves, segs, None, None, buf.getvalue()
    gap = math.hypot(v.x - nb.x, v.y - nb.y) - 2 * (drill / 2.0)
    return moves, segs, (v.x, v.y), gap, buf.getvalue()


class _TmpCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._tmp, ignore_errors=True)

    def board(self, name, **rules):
        return _stub_board(self._tmp, name, rules)


class TestTheGateIsBoardDerived(_TmpCase):
    """The arms #756 exists for. Each pairs a REFUSAL with an acceptance that
    still happens, so none of them can pass on a rig that refuses everything."""

    def test_ON_THE_BRANCH_the_free_landing_is_where_this_file_says(self):
        """Every offset below is derived from FREE_LANDING. If the sweep moved,
        the offsets measure something else and every arm here is quietly about
        a different geometry. This arm CALLS the engine; the derivations
        elsewhere are arithmetic on its result."""
        _m, _s, land, _g, _o = _nudge(self.board('free'), 5.0)
        self.assertIsNotNone(land, 'the unconstrained rig no longer moves at '
                                   'all; the whole class is inert')
        self.assertAlmostEqual(land[0], FREE_LANDING[0], places=4)
        self.assertAlmostEqual(land[1], FREE_LANDING[1], places=4)
        for gap, off in OFFSET_FOR_GAP.items():
            self.assertAlmostEqual(
                math.hypot(land[0] - VIA[0], land[1] - (VIA[1] + off))
                - 2 * DRILL_R, gap, places=4,
                msg='OFFSET_FOR_GAP[%s] no longer produces that gap at the '
                    'MEASURED landing' % gap)
    # MUTATION: 4 rows -- anything that moves the sweep's first accepted
    # candidate. It licenses the offsets rather than asserting the fix, and its
    # job is to fail loudly if a later edit moves them.

    def test_a_declaring_board_PREFERS_a_landing_the_fab_floor_would_not(self):
        """THE DEFECT, and the fix, in one pair. At the flat 0.20 the pass
        parked the barrel 0.2100 from a neighbour drill on a board declaring
        0.25 -- copper check_drc then flags, since 0.25 - 0.21 = 0.0400 exceeds
        its 0.05*0.25 = 0.0125 tolerance."""
        off = OFFSET_FOR_GAP[0.21]
        moves, segs, land, gap, _o = _nudge(self.board('d21',
                                                       min_hole_to_hole=0.25),
                                            off)
        self.assertEqual((len(moves), len(segs)), (1, 1),
                         'the repair was traded away: preferring the declared '
                         'floor must move the landing, not abandon the via')
        self.assertGreaterEqual(
            gap, 0.25 - 1e-9,
            'the accepted landing is still inside the declared floor')
        self.assertNotAlmostEqual(
            land[1], FREE_LANDING[1], places=4,
            msg='the landing did not move, so the gate did not bind and this '
                'arm is measuring nothing')
    # MUTATION: 6 rows -- every edit that drops the board read, drops the
    # max(), or re-hardcodes the literal.

    def test_the_SAME_board_declaring_nothing_keeps_the_old_landing(self):
        """The negative control. Without it the arm above could be passing on
        a rig that refuses every candidate near the neighbour."""
        moves, segs, land, gap, _o = _nudge(self.board('undeclared'),
                                            OFFSET_FOR_GAP[0.21])
        self.assertEqual((len(moves), len(segs)), (1, 1))
        self.assertAlmostEqual(land[1], FREE_LANDING[1], places=4)
        self.assertAlmostEqual(gap, 0.21, places=4,
                               msg='a board that declares nothing must be '
                                   'byte-identical to the pre-#756 pass')
    # MUTATION: 4 rows -- anything that makes the fab fallback larger, or that
    # reads a declaration where there is none.

    def test_a_declaration_BELOW_the_fab_floor_cannot_lower_it(self):
        """RAISE-ONLY, in the code and not only in the docstring. `board_floor`
        is board-AUTHORITATIVE and hands back a declared 0.10 unwrapped, so
        without the max() this pass would relocate a via to a drill pair no fab
        can punch -- and would do it silently."""
        via, pad, decl, src, fv, fp = resolve_drill_floors(
            make_pcb(board_info=_bi(),
                     source_path=self.board('tiny', min_hole_to_hole=0.10)))
        self.assertEqual((via, pad), (FAB_VIA, FAB_PAD))
        self.assertEqual((fv, fp), (FAB_VIA, FAB_PAD))
        self.assertEqual((decl, src), (0.10, 'board constraint'),
                         'the declaration must still be REPORTED -- the '
                         'disclosure branch keys on it')
    # MUTATION: `max(declared, fab_via)` -> `declared` at either floor. 3 rows.


class TestTheTranscript(_TmpCase):
    """The disclosure lines, asserted by their NUMBERS AND THEIR ROLES.

    A fact-check found both of these arms passing with the two floors SWAPPED
    in the format string -- "via-hole 0.45mm, pad-hole 0.25mm" -- because they
    only asserted that some substring was present. Each arm below now pins the
    full clause, so a wrong number in the wrong slot is a failure."""

    def test_a_raised_board_names_both_floors_and_which_one_it_raised(self):
        _m, _s, _l, _g, out = _nudge(
            self.board('loud', min_hole_to_hole=0.25), 5.0)
        self.assertIn('via-hole 0.25mm, pad-hole 0.45mm', out,
                      'the two resolved floors are not both named, in order')
        self.assertIn("min_hole_to_hole 0.25mm raised via-hole", out,
                      'the line must say WHICH floor the board moved -- on a '
                      'typical declaration only the via one moves and the '
                      '0.45 is still the fab number')
        self.assertIn('fab floor 0.2/0.45', out)
        self.assertIn('PREFERRED', out,
                      'the line must not read as a hard floor; it is a ladder')
    # MUTATION: 3 rows (the print deleted, the two format fields swapped, the
    # `_moved` term dropped).

    def test_a_sub_fab_declaration_says_it_was_floored_UP(self):
        """A board file cannot lower a fab floor without the transcript saying
        it tried, and the line must name the FAB value as the one in force."""
        _m, _s, _l, _g, out = _nudge(
            self.board('tiny2', min_hole_to_hole=0.10), 5.0)
        self.assertIn('Board min_hole_to_hole 0.1mm is below the 0.2mm fab '
                      'hole-to-hole floor; using 0.2/0.45.', out)
    # MUTATION: 3 rows (the print deleted, and either number re-pointed at the
    # wrong variable).

    def test_a_board_at_EXACTLY_the_fab_floor_says_nothing(self):
        """A board declaring the packaged default has nothing to disclose, and
        a line on every such run would be noise in a last-resort repair most
        runs never reach."""
        _m, _s, _l, _g, out = _nudge(
            self.board('exact', min_hole_to_hole=0.20), 5.0)
        self.assertNotIn('min_hole_to_hole', out)
    # MUTATION: `>` -> `>=` in the disclosure guard. 1 row.

    def test_the_FALLBACK_is_announced_when_it_happens(self):
        """The ladder's second rung is not silent: an operator who wanted the
        declared floor honoured has to be able to see that it was not."""
        _m, _s, _l, _g, out = _nudge(
            self.board('fb', min_hole_to_hole=0.25), OFFSET_FOR_GAP[0.21],
            max_shift=0.15)
        self.assertIn("no spot cleared the board's 0.25mm min_hole_to_hole",
                      out)
        self.assertIn('fell back to the 0.2mm fab floor', out)
    # MUTATION: 2 rows (the fallback print deleted; the ladder collapsed to one
    # rung, which removes the line by removing the behaviour).


class TestTheLadderCannotLoseARepair(_TmpCase):
    """The property the ladder was chosen FOR, measured rather than asserted.

    A one-rung raise was written first and an adversarial review swept 8673
    configurations of this rig shape: 625 lost the repair at the shipped 0.6mm
    budget, 13 of them abandoning a landing check_drc grades CLEAN. This class
    re-runs the same comparison, smaller, on every commit."""

    DRILLS = (0.30, 0.40, 0.50, 0.63)
    OFFSETS = tuple(round(0.34 + 0.04 * i, 2) for i in range(12))

    def test_no_configuration_loses_a_repair_the_flat_floor_would_make(self):
        flat = _stub_board(self._tmp, 'sw_flat', None)
        lost, upgraded, same, neither = [], 0, 0, 0
        for declared in (0.25, 0.30):
            decl = self.board('sw_%s' % declared, min_hole_to_hole=declared)
            for dr in self.DRILLS:
                for off in self.OFFSETS:
                    _a, _b, _l, ga, _o = _nudge(flat, off, drill=dr)
                    _c, _d, _l2, gb, _o2 = _nudge(decl, off, drill=dr)
                    if ga is None and gb is None:
                        neither += 1
                    elif ga is not None and gb is None:
                        lost.append((declared, dr, off, ga))
                    elif gb is not None and ga is None:
                        upgraded += 1
                    elif gb > ga + 1e-9:
                        upgraded += 1
                    else:
                        same += 1
        n = 2 * len(self.DRILLS) * len(self.OFFSETS)
        self.assertGreaterEqual(same + upgraded, n // 3,
                                'the sweep no longer moves a via in most '
                                'configurations, so "loses nothing" is vacuous'
                                ' (same=%d upgraded=%d neither=%d)'
                                % (same, upgraded, neither))
        self.assertEqual(lost, [],
                         'the ladder LOST a repair the flat floor makes: %r. '
                         'That is the property it was chosen for -- rung 2 is '
                         'the pre-#756 behaviour, so this can only happen if '
                         'the ladder was collapsed' % (lost,))
        self.assertGreater(upgraded, 0,
                           'nothing was upgraded, so the declared rung never '
                           'won and this class is measuring nothing')
    # MUTATION: 3 rows -- the ladder collapsed either way, or its order
    # reversed. The `upgraded > 0` half is what catches a ladder that silently
    # always takes rung 2.


class TestParityWithTheChecker(unittest.TestCase):
    """The rule #756 rests on: raise a floor at this site iff check_drc raises
    it -- and only where the site can afford it. Mirrored here BY LINE, and
    said out loud to be a mirror."""

    def test_check_drc_raises_hole_to_hole_from_the_same_board_key(self):
        import check_drc
        src = inspect.getsource(check_drc)
        self.assertIn("_pin_up('hole_to_hole_clearance'", src,
                      'check_drc no longer board-derives its hole-to-hole '
                      "floor, so #756's parity argument has expired -- "
                      're-derive it before trusting the resolver')
        self.assertIn("get('min_hole_to_hole')", src,
                      'the key check_drc pins from has changed')
    # MUTATION: none -- a mirror of another module, and a change detector on it.

    def test_the_connector_gate_is_ALSO_below_its_grader_and_stays_flat(self):
        """The claim an earlier draft got wrong, pinned so it cannot come back.
        check_drc `_pin_up`s `hole_clearance` from `min_hole_clearance` too,
        while this pass's connector gate spends the flat `npth_clr`. That gap
        is #617's measured choice; #756 neither closes nor licenses closing
        it."""
        import check_drc
        self.assertIn("_pin_up('hole_clearance'", inspect.getsource(check_drc),
                      'check_drc no longer board-derives copper-to-hole, so '
                      "the #756 docstring's account of the OTHER gap is stale")
        joined = ' '.join(_nudger_src())
        self.assertIn(
            'npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE)',
            joined, 'the connector floor moved; #756 said it stays flat')
    # MUTATION: none -- it pins a NON-change, which is the point.

    def test_the_pad_arm_margin_DECAYS(self):
        """check_drc grades via-drill<->pad-drill at the SAME single
        hole-to-hole value, not at 0.45, so this pass over-blocks that pair.
        By HOW MUCH is `max(d, 0.45) - max(0.20, d)` -- an earlier draft called
        it a flat 0.25 that #756 did not change, and both halves were wrong."""
        self.assertGreater(FAB_PAD, FAB_VIA)
        for d, want in ((0.0, 0.25), (0.20, 0.25), (0.30, 0.15),
                        (0.45, 0.0), (0.60, 0.0)):
            got = max(d, FAB_PAD) - max(FAB_VIA, d)
            self.assertAlmostEqual(got, want, places=9,
                                   msg='the pad-arm margin at a declared %s '
                                       'is %s, not %s' % (d, got, want))
    # MUTATION: `fmin['pad_hole_to_hole']` -> `fmin['hole_to_hole']`. 1 row.


class TestTheResolverItself(_TmpCase):

    def test_no_project_takes_the_fab_floors_without_reading_the_disk(self):
        """`read_design_rules("")` probes ".kicad_pro" RELATIVE TO THE PROCESS
        CWD, so an empty `source_path` must short-circuit BEFORE the read --
        otherwise a stray file of that name is read as this board's rules.
        Proven by planting exactly that file and cd-ing into it. A
        DIRECTORY-shaped path is the same hazard: `splitext` leaves it intact.
        """
        d = os.path.join(self._tmp, 'cwdtrap')
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, '.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump({'board': {'design_settings':
                                 {'rules': {'min_hole_to_hole': 9.0}}}}, f)
        cwd = os.getcwd()
        try:
            os.chdir(d)
            for spelling in ('', '.', './'):
                self.assertEqual(
                    resolve_drill_floors(
                        make_pcb(board_info=_bi(), source_path=spelling)),
                    (FAB_VIA, FAB_PAD, None, 'fixed default', FAB_VIA, FAB_PAD),
                    'source_path %r read the CWD project' % spelling)
        finally:
            os.chdir(cwd)
    # MUTATION: 2 rows -- the `if not path` guard dropped (empty spelling), and
    # the `os.path.isdir` term dropped (the other two).

    def test_a_declaring_project_is_read_and_TAGGED(self):
        p = self.board('decl', min_hole_to_hole=0.25)
        self.assertEqual(resolve_drill_floors(make_pcb(board_info=_bi(),
                                                       source_path=p)),
                         (0.25, FAB_PAD, 0.25, 'board constraint',
                          FAB_VIA, FAB_PAD))
    # MUTATION: 5 rows.

    def test_a_declaration_above_the_PAD_floor_moves_the_pad_arm_too(self):
        """One board rule behind BOTH numbers, and that is not an
        approximation: min_hole_to_hole governs any two holes, which is why
        `list_nets.effective_floors` floors both of its keys at it. No board
        this repo tracks declares above 0.45, so this arm is the only place the
        pad arm is shown to move at all."""
        p = self.board('huge', min_hole_to_hole=0.60)
        self.assertEqual(resolve_drill_floors(make_pcb(board_info=_bi(),
                                                       source_path=p))[:2],
                         (0.60, 0.60))
    # MUTATION: `max(declared, fab_pad)` -> `fab_pad`. 1 row.

    def test_the_PAD_floor_actually_reaches_the_PAD_gate(self):
        """A separate question from the resolver returning it, and a fact-check
        found nothing in this file asking it: swapping `H2H_PAD` for `H2H_VIA`
        at the capsule test left every other arm green. Here a through-hole pad
        drill is what the relocated barrel must clear, on a board declaring
        above the 0.45 pad fab floor."""
        # A through-hole pad drill BESIDE the via, not above it, and the two
        # rejected placements are worth recording because both looked right:
        #
        #   1.00mm above -> NEITHER floor binds; both arms land identically and
        #                   the arm measures nothing. The very defect class it
        #                   was added to catch.
        #   0.85mm above -> the 0.60 floor binds, but nothing in the 0.6mm
        #                   budget clears it, so the LADDER FALLS BACK and both
        #                   arms land identically again. Correct engine, wrong
        #                   rig.
        #
        # Beside it, the escape (+y, forced by the cap bar) has somewhere to go
        # that gains distance from the hole. Probed, not reasoned: this gives
        # 0.4739 flat and 0.6359 declared, clearing the 0.45 pad floor by 0.024
        # and the declared 0.60 by 0.036 -- both far above the gates' 1e-4.
        HOLE = (1.7, 1.4)

        def rig(path):
            v = make_via(VIA[0], VIA[1], net_id=3)
            hole = Pad(pad_number='1', net_id=0, net_name='',
                       global_x=HOLE[0], global_y=HOLE[1], local_x=0.0,
                       local_y=0.0, size_x=0.6, size_y=0.6, shape='circle',
                       layers=['F.Cu', 'B.Cu'],
                       drill=0.3, pad_type='thru_hole', component_ref='U9')
            stub = Segment(start_x=VIA[0], start_y=VIA[1] - 0.2, end_x=VIA[0],
                           end_y=VIA[1], width=0.2, layer='F.Cu', net_id=3)
            pcb = make_pcb(board_info=_bi((0.0, 0.0, 6.0, 6.0)), vias=[v],
                           segments=[stub],
                           footprints={'C1': SimpleNamespace(layer='F.Cu',
                                                             pads=[])},
                           pads_by_net={0: [hole]}, source_path=path,
                           zones=[])
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                moves, _s = nudge_vias_for_unresolved(_FakeSt([BAR]), pcb,
                                                      CLEAR)
            if not moves:
                return None
            return math.hypot(v.x - HOLE[0], v.y - HOLE[1]) - 0.30

        flat = rig(_stub_board(self._tmp, 'pg_flat', None))
        big = rig(self.board('pg_big', min_hole_to_hole=0.60))
        self.assertIsNotNone(flat, 'the flat rig no longer moves; nothing to '
                                   'compare')
        self.assertIsNotNone(big, 'the declaring rig lost the repair -- the '
                                  'ladder should have fallen back')
        self.assertGreaterEqual(flat, FAB_PAD - 1e-6,
                                'the shipped pad floor is not binding on this '
                                'rig (%.4f), so the comparison is not about it'
                                % flat)
        self.assertGreater(big, flat + 1e-6,
                           'a declaration above the pad fab floor did not move '
                           'the pad GATE (%.4f vs %.4f) -- the resolver '
                           'returns it and nothing spends it' % (big, flat))
    # MUTATION: 2 rows -- `H2H_PAD` -> `H2H_VIA` at the capsule gate, and
    # `max(declared, fab_pad)` -> `fab_pad`.

    def test_a_project_that_EXISTS_and_declares_nothing(self):
        """A real `.kicad_pro` with an empty rules block -- not the same
        fixture as "no project", which is what an earlier draft actually built
        here. Both must report no declaration; only this one proves the read
        happened and came back empty."""
        p = _stub_board(self._tmp, 'empty_rules', {})
        via, pad, decl, _src, _fv, _fp = resolve_drill_floors(
            make_pcb(board_info=_bi(), source_path=p))
        self.assertEqual((via, pad, decl), (FAB_VIA, FAB_PAD, None),
                         'a project with no min_hole_to_hole must not report '
                         'a declaration -- the disclosure keys on it')
    # MUTATION: `fallback=None` -> `defaults.HOLE_TO_HOLE_CLEARANCE`, which
    # makes `declared` a fallback comparing against itself. 1 row.

    def test_a_duck_typed_pcb_data_does_not_explode(self):
        """The function is reached from harnesses that pass SimpleNamespace,
        and from the GUI with a live board. Neither may raise. This arm owns
        the `or []` guard on the copper-layer read -- `board_info=None` is the
        only input that reaches it."""
        want = (FAB_VIA, FAB_PAD, None, 'fixed default', FAB_VIA, FAB_PAD)
        self.assertEqual(resolve_drill_floors(SimpleNamespace()), want)
        self.assertEqual(
            resolve_drill_floors(SimpleNamespace(board_info=None,
                                                 source_path=None)), want)
    # MUTATION: 2 rows -- either `getattr` chain replaced by a direct attribute
    # read, and `(_cu or [])` -> `_cu`.

    def test_it_agrees_with_list_nets_effective_floors(self):
        """The independent oracle. `effective_floors` computes the same two
        numbers from the same board key, and #756 deliberately hand-composes
        instead of calling it (it counts copper layers off the FILE, which is 0
        for an unsaved GUI board, and returns no source tag). Pinning them
        EQUAL turns "we mirror the shared rule" from prose into a failing
        test."""
        boards = [b for b in run_utils.corpus_boards()
                  if os.path.exists(os.path.splitext(b)[0] + '.kicad_pro')]
        if not boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(
            len(boards), 2,
            'the set of tracked boards carrying a sibling project has '
            'SHRUNK to %d; this arm has no witness left' % len(boards))
        for b in boards:
            eff = read_design_rules(b)['effective']
            got = resolve_drill_floors(parse_kicad_pcb(b))[:2]
            self.assertEqual(
                got, (eff['drc_hole_to_hole'], eff['pad_hole_to_hole']),
                '%s: the resolver and effective_floors disagree'
                % os.path.basename(b))
    # MUTATION: 4 rows -- any change to either fab key or to the max().


class TestTheFabTierChannel(unittest.TestCase):
    """The half of #756's own headline claim that had NO test. The module
    docstring indicts the old literals for reading "not --fab-tier, not a
    --fab-overrides file"; a fact-check made the resolver deaf to both
    (`fab_floor_min(n, tier='standard', overrides={})`) and every arm stayed
    green."""

    def tearDown(self):
        fab_tiers.set_default_fab_tier('standard', {})

    def test_an_overrides_file_moves_both_floors(self):
        fab_tiers.set_default_fab_tier('standard',
                                       {'hole_to_hole': 0.30,
                                        'pad_hole_to_hole': 0.60})
        self.assertEqual(resolve_drill_floors(_projectless())[:2], (0.30, 0.60))
    # MUTATION: `fab_floor_min(...)` pinned to the packaged tier. 1 row -- and
    # this arm is the only thing that kills it.

    def test_the_tier_selector_alone_cannot_move_them(self):
        """Both tiers declare 0.20/0.45, so `--fab-tier advanced` is inert
        here. Said out loud because the README would otherwise imply it is a
        knob, and an operator would go looking for an effect."""
        for tier in ('standard', 'advanced'):
            fab_tiers.set_default_fab_tier(tier, {})
            self.assertEqual(resolve_drill_floors(_projectless())[:2],
                             (FAB_VIA, FAB_PAD))
    # MUTATION: none -- a change detector on the fab table.

    def test_an_overrides_file_BELOW_the_old_literals_is_honoured(self):
        """A relaxation path that did not exist before #756: the pre-change
        literals were immune to the override file. Disclosed rather than
        prevented -- the file states the operator's real fab limits, which is
        its documented meaning."""
        fab_tiers.set_default_fab_tier('standard', {'hole_to_hole': 0.05})
        self.assertEqual(resolve_drill_floors(_projectless())[0], 0.05)
    # MUTATION: a max() against the packaged default added "for safety". 1 row.


class TestTheLayerCountChoice(unittest.TestCase):
    """The resolver picks a fab bucket by copper-layer count. Today it cannot
    be wrong about it, and that is worth ASSERTING rather than narrating."""

    def test_the_layer_count_cannot_change_either_floor_TODAY(self):
        """A CHANGE DETECTOR, not a wiring test. `_FAB_FLOORS` carries 0.20 and
        0.45 in all four cells, and `fab_floor_ladder` overlays a
        LAYER-INDEPENDENT override dict, so `fab_floor_min(n)[k]` is
        n-independent for both keys under every tier and every override file.
        The resolver therefore reads `board_info.copper_layers` for a bucket it
        cannot currently mis-pick -- and this arm is what turns a future
        per-layer hole floor into a failure instead of a silent mis-bucket."""
        for k in ('hole_to_hole', 'pad_hole_to_hole'):
            self.assertEqual(fab_floor_min(2)[k], fab_floor_min(4)[k],
                             '%s now differs by layer bucket; re-read '
                             "resolve_drill_floors' layer-count choice, and "
                             'note that the two halves of the pass derive it '
                             'separately' % k)
    # MUTATION: none in the battery, and that is a disclosure rather than an
    # omission -- it guards a `fab_tiers` table #756 does not touch, so no edit
    # to the code under test can move it.


class TestTheLadderIsNotCollapsible(unittest.TestCase):
    """A source guard, because the ladder's whole value is its SHAPE and a
    later reader tidying it into one `max()` would restore the class of defect
    #756 rejected. Comment-stripped, for the reason `_nudger_src` gives."""

    def test_the_sweep_iterates_a_ladder(self):
        src = _nudger_src()
        self.assertEqual(
            len([l for l in src
                 if 'for H2H_VIA, H2H_PAD in drill_ladder' in l]), 1,
            'the candidate sweep no longer descends drill_ladder')
        self.assertEqual(
            len([l for l in src if 'drill_ladder.append(' in l]), 1,
            'the second rung is gone, so a declaring board can no longer fall '
            'back to the fab floor')
    # MUTATION: 3 rows -- the ladder collapsed either way, or its order
    # reversed. The behavioural class above owns the consequences; this owns
    # the shape, so a refactor that keeps today's numbers cannot silently drop
    # it.


class TestEveryExistingNudgerRigIsUnmoved(unittest.TestCase):
    """#617's doctrine is cited against this change; its rig cannot reach the
    gate. Asserted rather than argued."""

    def test_no_nudger_harness_declares_min_hole_to_hole(self):
        """So every one of them resolves to the fab floors and is
        byte-identical under #756. If a future harness starts declaring it,
        this fails and its author has to look at what moved."""
        names = ('test_370_tierb_fixes.py',
                 'test_617_placement_fanout_hole_clearance.py',
                 'test_725_fanout_clearance_pad_floors.py',
                 'test_730_fanout_clearance_npth_local_clearance.py',
                 'test_732_fanout_clearance_via_radius.py',
                 'test_733_fanout_clearance_edge_margin.py',
                 'test_736_fanout_clearance_regrade_view.py',
                 'test_737_fanout_clearance_via_hole.py',
                 'test_741_via_nudge_tenting.py',
                 'test_746_fanout_clearance_resolved_credit.py',
                 'test_750_fanout_clearance_via_drill.py')
        present = [n for n in names if os.path.exists(os.path.join(_TESTS, n))]
        self.assertGreaterEqual(len(present), 10,
                                'the nudger harness set has shrunk to %d; '
                                'this arm is no longer about the row it names'
                                % len(present))
        declaring = [n for n in present
                     if 'min_hole_to_hole' in open(os.path.join(_TESTS, n),
                                                   encoding='utf-8').read()]
        self.assertEqual(declaring, [],
                         'a nudger harness now declares min_hole_to_hole: %r. '
                         "#756's claim that every existing rig is unmoved has "
                         'EXPIRED -- re-run them and record what changed'
                         % declaring)
    # MUTATION: none -- a claim about the OTHER files, and a change detector.

    def test_the_617_rig_cannot_reach_the_via_to_via_gate_at_all(self):
        """One via, so the self-skip fires and the loop body never runs. This
        is why #617's measurement -- real, and re-derived under #730 -- is
        silent about H2H_VIA.

        COMMENT-STRIPPED. A fact-check found this arm green against an engine
        whose self-skip had been DELETED, because the needle also appears in
        the prose above. That is the false-positive class test_737:480-486 and
        test_750:653-657 both record."""
        src = _nudger_src()
        hits = [i + 1 for i, l in enumerate(src) if 'if ov is v:' in l]
        self.assertEqual(len(hits), 1,
                         'expected exactly one self-skip in the via loop; '
                         'found %d at function-relative line(s) %s'
                         % (len(hits), hits))
    # MUTATION: the self-skip deleted -> killed here AND by four behavioural
    # arms. 1 row.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """The row's self-expiring bound. #756 is inert on the tracked corpus at
    file poses for THREE independent reasons, and a "0 diffs" run proves
    nothing unless all three are stated: 20 of 22 boards carry no project at
    all, the 2 that do never reach the via-nudge, and none declares above 0.45.

    So this asserts the REASONS, not just the outcome."""

    def setUp(self):
        self.boards = run_utils.corpus_boards()
        if not self.boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(len(self.boards), 22,
                                'the tracked corpus collapsed to %d boards; '
                                'nothing below is a bound'
                                % len(self.boards))

    def test_only_two_tracked_boards_can_declare_anything(self):
        withpro = [os.path.basename(b) for b in self.boards
                   if os.path.exists(os.path.splitext(b)[0] + '.kicad_pro')]
        self.assertEqual(
            sorted(withpro),
            ['flat_hierarchy.kicad_pcb', 'routed_output.kicad_pcb'],
            'the set of tracked boards carrying a sibling project has '
            'CHANGED: %r. The "inert on the corpus" claim in the #756 PR has '
            'EXPIRED -- re-run the before/after sweep and record the new '
            'numbers' % sorted(withpro))

    def test_no_tracked_board_declares_above_the_PAD_fab_floor(self):
        """Which is why the pad arm has no corpus witness and is demonstrated
        synthetically instead. Said out loud rather than left to be noticed."""
        over = []
        for b in self.boards:
            _via, _pad, decl, _s, _fv, _fp = resolve_drill_floors(
                parse_kicad_pcb(b))
            if decl is not None and decl > FAB_PAD:
                over.append((os.path.basename(b), decl))
        self.assertEqual(over, [],
                         'a tracked board now declares above the %s pad-hole '
                         'fab floor: %r. The pad arm now HAS a corpus '
                         'witness -- measure it and record it'
                         % (FAB_PAD, over))

    def test_the_via_nudge_emits_nothing_on_any_tracked_board(self):
        """NOT REACHABLE, which is a different claim from NO EFFECT, and the
        PR must not blur them. Measured at the shipped defaults: zero via_moves
        on every tracked board, so the corpus cannot witness this change at
        file poses either way."""
        noisy = []
        for b in self.boards:
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    r = repair_fanout_clearance(parse_kicad_pcb(b), b,
                                                clearance=0.2)
            except Exception as e:                            # noqa: BLE001
                noisy.append((os.path.basename(b), 'ERROR', repr(e)[:60]))
                continue
            if r.get('via_moves') or r.get('new_segments'):
                noisy.append((os.path.basename(b), len(r['via_moves']),
                              len(r['new_segments'])))
        self.assertEqual(noisy, [],
                         'a tracked board now reaches the via-nudge at the '
                         'shipped defaults: %r. The "inert on the corpus" '
                         'claim in the #756 PR has EXPIRED -- re-run the '
                         'before/after sweep and record the new numbers'
                         % noisy)
    # MUTATION: 2 rows -- it really does drive the resolver through
    # repair_fanout_clearance on every tracked board, so anything that makes
    # the resolver raise on a real board is caught here.


class TestTheBgaSibling(_TmpCase):
    """`bga_fanout.manage_vias`' via_in_pad_conflict had the identical defect,
    named in its own docstring. Closed in the same PR, narrowly.

    FIXTURES SIT 0.05mm OFF THE GATE, not on it. An earlier draft used
    separations whose entire margin was the engine's own `- 1e-6` epsilon, so
    an epsilon or float-association change would have failed as "the board read
    regressed"."""

    # drill-edge gap = sep - (0.10 + 0.15). Every gap is 0.05mm clear of
    # both the 0.20 fab floor and the 0.30 declaration tested below --
    # 0.15 was the first draft's smallest, and it is BELOW the fab floor,
    # so the undeclared row was refused for a reason that had nothing to
    # do with #756 and the arm passed while measuring that.
    SEPS = (0.50, 0.60, 0.70)          # gaps 0.25, 0.35, 0.45

    def _run(self, path, sep):
        from bga_fanout import manage_vias
        from bga_fanout.types import FanoutRoute

        def pad(x, y, **kw):
            d = dict(pad_number='1', net_id=0, net_name='', global_x=x,
                     global_y=y, local_x=0.0, local_y=0.0, size_x=0.5,
                     size_y=0.5, shape='circle', layers=['F.Cu', 'B.Cu'],
                     drill=0.0, pad_type='smd', component_ref='X1')
            d.update(kw)
            return Pad(**d)

        ball = pad(10.0, 10.0, net_id=7, layers=['F.Cu'], component_ref='U1',
                   pad_number='A1')
        foreign = pad(10.0 + sep, 10.0, drill=0.3, size_x=0.6, size_y=0.6,
                      pad_type='thru_hole', component_ref='U2')
        r = FanoutRoute(pad=ball, pad_pos=(10.0, 10.0), stub_end=(10.5, 10.5),
                        exit_pos=(11.0, 10.5), layer='B.Cu')
        pcb = make_pcb(board_info=_bi((0.0, 0.0, 20.0, 20.0),
                                      ('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu')),
                       vias=[], segments=[],
                       pads_by_net={7: [ball], 0: [foreign]},
                       source_path=path, zones=[])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            add, _rm, blocked = manage_vias([r], pcb, 'F.Cu', 0.45, 0.2, 0.1)
        return len(add), len(blocked)

    def test_the_verdict_flips_at_the_declared_floor(self):
        """A refusal AND an acceptance at every declaration, so no row can pass
        on a rig that refuses everything."""
        self.assertEqual([self._run(_stub_board(self._tmp, 'bga_none', None),
                                    s)[0] for s in self.SEPS], [1, 1, 1],
                         'a board declaring nothing must be byte-identical to '
                         'the pre-#756 pass')
        self.assertEqual([self._run(self.board('bga_20',
                                               min_hole_to_hole=0.20), s)[0]
                          for s in self.SEPS], [1, 1, 1],
                         'a board declaring exactly the fab floor must also '
                         'be unchanged')
        self.assertEqual([self._run(self.board('bga_30',
                                               min_hole_to_hole=0.30), s)[0]
                          for s in self.SEPS], [0, 1, 1])
    # MUTATION: 4 rows in the bga target.

    def test_a_refusal_DROPS_the_escape(self):
        """The cost of the board-first read here, which the via-nudge does not
        pay: this pass has no second rung, so a refusal sends the route to
        `via_blocked_routes` and its net to `failed_nets`. Asserted so the PR's
        disclosure is checkable and so nobody 'fixes' it by loosening."""
        added, blocked = self._run(self.board('bga_drop',
                                              min_hole_to_hole=0.30), 0.50)
        self.assertEqual((added, blocked), (0, 1),
                         'the refused escape is no longer reported as '
                         'blocked; either it silently succeeded or the count '
                         'moved')
    # MUTATION: 2 rows -- either bga arm reverted to the flat constant.

    def test_the_via_to_via_drill_arm_is_board_derived_TOO(self):
        """`via_in_pad_conflict` has two arms and #756 moved both. The battery
        caught this one having NO test: the rig above only ever presents a PAD
        drill. The foreign via is small in copper (0.2) and normal in drill
        (0.3) so `would_overlap_existing_via` (needs 0.425) is satisfied and
        the DRILL gate alone decides."""
        from bga_fanout import manage_vias
        from bga_fanout.types import FanoutRoute

        def run(path, sep):
            ball = Pad(pad_number='A1', net_id=7, net_name='', global_x=10.0,
                       global_y=10.0, local_x=0.0, local_y=0.0, size_x=0.5,
                       size_y=0.5, shape='circle', layers=['F.Cu'], drill=0.0,
                       pad_type='smd', component_ref='U1')
            r = FanoutRoute(pad=ball, pad_pos=(10.0, 10.0),
                            stub_end=(10.5, 10.5), exit_pos=(11.0, 10.5),
                            layer='B.Cu')
            ov = make_via(10.0 + sep, 10.0, net_id=0, size=0.2, drill=0.3)
            pcb = make_pcb(board_info=_bi((0.0, 0.0, 20.0, 20.0),
                                          ('F.Cu', 'In1.Cu', 'In2.Cu',
                                           'B.Cu')),
                           vias=[ov], segments=[], pads_by_net={7: [ball]},
                           source_path=path, zones=[])
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                add, _rm, _b = manage_vias([r], pcb, 'F.Cu', 0.45, 0.2, 0.1)
            return len(add)

        # drill gap = sep - 0.25; 0.50 -> 0.25, 0.65 -> 0.40. 0.05 clear of
        # both the 0.20 fab floor and the 0.30 declaration.
        none = _stub_board(self._tmp, 'bgav_none', None)
        d30 = self.board('bgav_30', min_hole_to_hole=0.30)
        self.assertEqual(run(none, 0.50), 1,
                         'a board declaring nothing must be byte-identical on '
                         'the via arm too')
        self.assertEqual(run(d30, 0.50), 0,
                         'the via/via drill arm ignored the declaration')
        self.assertEqual(run(d30, 0.65), 1)
    # MUTATION: bga-via-arm-reverted-to-the-flat-constant. 1 row -- and this
    # arm is the ONLY thing that kills it.

    def test_a_bga_board_declaring_BELOW_the_fab_floor_is_floored_up(self):
        """The raise-only wrap, which nothing exercised until the battery said
        so: `manage_vias` hands `board_floor` HOLE_TO_HOLE_CLEARANCE as its
        FALLBACK, so `_h2h_decl` can only fall below 0.20 when a board declares
        below it -- and no arm did."""
        tiny = self.board('bga_tiny', min_hole_to_hole=0.10)
        # gap 0.15 (sep 0.40): under the 0.20 fab floor, over a declared
        # 0.10 -- the ONLY band where the wrap is what decides.
        self.assertEqual(self._run(tiny, 0.40)[0], 0,
                         'a sub-fab declaration lowered the drill spacing; '
                         'the max() against the fab floor is gone')
        self.assertEqual(self._run(tiny, 0.50)[0], 1)
    # MUTATION: bga-drops-the-fab-wrap. 1 row -- and this arm is the ONLY
    # thing that kills it.

    def test_an_unsaved_board_does_not_read_the_process_CWD(self):
        """The HIGH finding a review caught: this pass had no `source_path`
        guard, so `build_pcb_data_from_board`'s empty path -- an unsaved GUI
        board, which is the GUI fanout path -- made `board_floor("")` probe the
        process CWD. Measured before the guard: a planted project declaring 5.0
        made the pass announce it as "the board's own"."""
        from bga_fanout import manage_vias
        d = os.path.join(self._tmp, 'bga_cwd')
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, '.kicad_pro'), 'w', encoding='utf-8') as f:
            json.dump({'board': {'design_settings':
                                 {'rules': {'min_hole_to_hole': 5.0}}}}, f)
        cwd = os.getcwd()
        try:
            os.chdir(d)
            self.assertEqual(self._run('', 0.50)[0], 1,
                             "the CWD project was read as this board's "
                             'rules')
        finally:
            os.chdir(cwd)
        # ...and a call with NO routes announces nothing. On a
        # DECLARING board: an undeclared one takes the 'fixed default'
        # branch and would pass with the `routes` guard removed, which
        # is how the first draft of this arm let that mutation survive.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            manage_vias([], make_pcb(board_info=_bi(),
                                     source_path=self.board(
                                         'bga_noroutes',
                                         min_hole_to_hole=0.30)),
                        'F.Cu', 0.45, 0.2, 0.1)
        self.assertEqual(buf.getvalue().strip(), '',
                         'a call with no routes still announces a floor')
    # MUTATION: 2 rows -- the guard dropped, and the `routes` guard dropped.

    def test_it_did_NOT_adopt_the_nudgers_045_pad_floor(self):
        """The two passes disagree by 0.25mm about the very same
        via-drill<->pad-drill pair -- 0.45 in the via-nudge, the single
        hole-to-hole value here. #756 reconciles neither; unifying them would
        move keep-outs on every fine-pitch BGA via-in-pad escape and needs its
        own before/after. Pinned so nobody 'tidies' it in passing.

        gap 0.30 at sep 0.55: 0.10 above the 0.20 fab hole-to-hole that is
        in force, and 0.15 below the 0.45 it must NOT have adopted."""
        self.assertEqual(
            self._run(_stub_board(self._tmp, 'bga_pad', None), 0.55)[0], 1,
            'the bga pass now charges the pad-hole floor; that is a THIRD '
            'change with its own before/after, not a tidy-up')
    # MUTATION: `_h2h` -> the pad-hole floor at the capsule arm. 1 row.


if __name__ == '__main__':
    unittest.main(verbosity=2)
