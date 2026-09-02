#!/usr/bin/env python3
"""#620: `manage_vias` never tested the vias of ONE call against each other.

`vias_to_add` was appended to and never read back, and both guards that gate an
append iterate `pcb_data.vias` only (`would_overlap_existing_via`,
`via_in_pad_conflict`'s drill loop). So every via a call placed was spaced
against the INPUT board and against nothing else, and two placed in one call
could ship holes closer than the fab floor -- or, for two routes at one
coordinate, two `(via ...)` s-exprs stacked at the same point, since
`kicad_writer` does not dedupe.

MEASURED, because the issue deliberately left the impact unmeasured and its own
worked example was wrong to worry: on an empty board at CLI-ish defaults (via
0.45 / drill 0.2 / clearance 0.1) the pre-fix pass shipped

    pitch 0.30, pad 0.25  ->  holes 0.15mm apart, floor 0.20, added=2 blocked=0
    pitch 0.25, pad 0.20  ->  holes 0.10mm apart, floor 0.20, added=2 blocked=0
    pitch 0.30, pad 0.20  ->  holes 0.15mm apart, floor 0.20, added=2 blocked=0

while the issue's own 0.50mm-pitch example is LEGAL, exactly as it hedged --
`clamp_via_to_pad` shrinks the via into the ball pad first. **No board in this
repo reaches the refusal branch at CLI defaults** (seven fanout runs, zero
illegal emitted pairs; the tightest emitted pair anywhere is orangecrab U4 at
0.2907mm against a 0.20 floor), so the fixture here is CONSTRUCTED and the
corpus arms are a safety check, not the headline. Said plainly so no reader
mistakes a synthetic number for a board measurement.

THE SCOPE IS ASYMMETRIC AND THAT IS THE DESIGN (`PendingVias`):

  * the DRILL is always tested -- the balls are SMD and carry no hole, so every
    hole in the neighbourhood is one this pass creates;
  * the RING is tested only when a via BULGES past its pad (clamp status
    `'floor'`), because a via that fits inside its pad occupies no copper the
    pad did not already occupy, so a ring test on such a pair restates the
    board's own ball-to-ball spacing.

That split is a MEASUREMENT, not a principle: swept over all 6565 physical
(pitch, pad) combinations at clearances 0.10/0.15/0.20/0.25/0.30, the ring-only
rejections whose own pads are NOT already sub-clearance number
0/90/155/195/210, and 100% of them are bulging vias at every clearance.
`TestTheRingArmIsOnlyForBulgingVias` re-derives both halves so the claim stays
a change detector.

A REFUSAL DROPS THE ESCAPE -- this pass has no re-sweep -- so the conflict
branch descends the fab drill ladder first (`thin_drill_to_clear`). That is not
decoration: at pitch 0.36 / pad 0.32 both escapes survive with the second
drill thinned 0.17 -> 0.15, where a refuse-only design drops one. And it is not
a cure-all: `--fab-overrides` collapses the ladder to a single hard rung by
design, which is precisely the configuration the #620 contributor measured as
pure loss (`via_drill = 0.35` at 0.5mm pitch: unmeetable floor AND no rung).

Conventions (from #725/#731/#732/#733/#737/#750/#756 and CLAUDE.md): REAL
parser dataclasses; every refusal paired with an acceptance that still happens,
so no arm can pass on a rig that refuses everything; assert you are ON the
branch before asserting about it; every assertion names the mutation that must
kill it.
"""
import contextlib
import io
import json
import math
import os
import random
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'py_router'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kicad_parser import Pad, BoardInfo                          # noqa: E402
from synth import make_pcb                                       # noqa: E402
import fab_tiers                                                 # noqa: E402
from fab_tiers import fab_floor_ladder, fab_floor_min            # noqa: E402
from bga_fanout import manage_vias                               # noqa: E402
from bga_fanout.types import FanoutRoute                         # noqa: E402
from bga_fanout.geometry import (PendingVias, thin_drill_to_clear,  # noqa: E402
                                 clamp_via_to_pad)

CU = ('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu')
H2H = fab_floor_min(len(CU))['hole_to_hole']          # 0.20, standard tier
LADDER = fab_floor_ladder(len(CU))


def _ball(x, y, net_id, size, num='A1'):
    return Pad(pad_number=num, net_id=net_id, net_name=f'/N{net_id}',
               global_x=x, global_y=y, local_x=0.0, local_y=0.0,
               size_x=size, size_y=size, shape='circle', layers=['F.Cu'],
               drill=0.0, pad_type='smd', component_ref='U1')


def _stub_board(tmp, name, rules=None):
    """A bare board file, plus a sibling project when `rules` is not None.

    Only the PROJECT has to exist on disk (`board_floor` reads it); the PCBData
    is synthetic. `rules={}` is a project that EXISTS and declares nothing,
    which is a different case from no project at all.
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


def _two_balls(pitch, pad_size, *, same_net=False, source_path='',
               via_size=0.45, via_drill=0.2, clearance=0.1):
    """Two balls `pitch` apart on a board carrying NO copper at all.

    The empty board is the point: every guard that predates #620 scans
    `pcb_data`, so on an empty board they are all vacuously satisfied and the
    ONLY thing that can refuse the pair is a test of the two candidates against
    each other. Returns (vias_to_add, via_blocked_routes, transcript).
    """
    a = _ball(10.0, 10.0, 7, pad_size, 'A1')
    b = _ball(10.0 + pitch, 10.0, 7 if same_net else 8, pad_size, 'A2')
    ra = FanoutRoute(pad=a, pad_pos=(a.global_x, a.global_y),
                     stub_end=(a.global_x, a.global_y + 0.5),
                     exit_pos=(a.global_x, a.global_y + 1.0), layer='B.Cu')
    rb = FanoutRoute(pad=b, pad_pos=(b.global_x, b.global_y),
                     stub_end=(b.global_x, b.global_y - 0.5),
                     exit_pos=(b.global_x, b.global_y - 1.0), layer='B.Cu')
    pcb = make_pcb(board_info=BoardInfo(layers={}, copper_layers=list(CU),
                                        board_bounds=(0.0, 0.0, 20.0, 20.0)),
                   vias=[], segments=[],
                   pads_by_net=({7: [a, b]} if same_net else {7: [a], 8: [b]}),
                   source_path=source_path, zones=[])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        add, _rm, blocked = manage_vias([ra, rb], pcb, 'F.Cu', via_size,
                                        via_drill, clearance)
    return add, blocked, buf.getvalue()


def _clamped(pad_size, via_size=0.45, via_drill=0.2):
    return clamp_via_to_pad(via_size, via_drill, _ball(0, 0, 7, pad_size),
                            LADDER)


class _TmpCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._tmp, ignore_errors=True)
        # `warn_fab_escalation` dedupes per context string for the whole
        # PROCESS, so an arm that runs second sees no warning even when it
        # escalated. Clearing here is what makes the transcript arms
        # independent of test ORDER.
        fab_tiers._escalation_warned.clear()

    def board(self, name, **rules):
        return _stub_board(self._tmp, name, rules)


class TestOnTheBranch(_TmpCase):
    """Every geometry below is derived from what `clamp_via_to_pad` does to
    these pads. If the clamp moves, the arms measure a different geometry and
    say nothing about #620 -- so pin the derivation before using it."""

    def test_the_clamp_puts_these_pads_where_this_file_says(self):
        self.assertEqual(_clamped(0.25)[:2], (0.25, 0.15),
                         'the 0.25 pad no longer clamps to the advanced rung; '
                         'every 0.25-pad arm below is about other geometry')
        self.assertEqual(_clamped(0.20)[2], 'floor',
                         'the 0.20 pad no longer BULGES; the ring arms below '
                         'lose the only case they exist for')
        self.assertEqual(_clamped(0.32)[:2], (0.32, 0.17),
                         'the ladder rig no longer starts above the deepest '
                         'drill rung, so nothing can be thinned')
        self.assertEqual(H2H, 0.20,
                         'the standard-tier hole-to-hole floor moved; the '
                         'pitches below were chosen against 0.20')


class TestThePairTest(_TmpCase):
    """The gap #620 names, with a refusal and an acceptance at every rig."""

    def test_a_sub_floor_pair_is_no_longer_shipped(self):
        """MUTATION: drop the `_pending.verdict` call, or make its 'conflict'
        branch `pass`. Killed by the blocked count, not by the via count --
        a via count alone cannot tell a refusal from a twin."""
        add, blocked, _t = _two_balls(0.30, 0.25)
        self.assertEqual((len(add), len(blocked)), (1, 1),
                         'two holes 0.15mm apart against a 0.20mm floor are '
                         'still both shipped')

    def test_a_clearing_pair_is_still_shipped(self):
        """The acceptance half. MUTATION: widen the floor (`_h2h` -> a larger
        constant) and this arm dies while the refusal above still passes."""
        add, blocked, _t = _two_balls(0.35, 0.25)
        self.assertEqual((len(add), len(blocked)), (2, 0),
                         'a LEGAL pair (0.20mm hole gap at a 0.20mm floor) is '
                         'being refused -- the gate is over-tight')

    def test_the_issue_s_own_worked_example_is_legal(self):
        """#620 worried about 0.5mm pitch with via 0.45 + clearance 0.1, and
        hedged that the clamp might legalize it. It does. Pinned so nobody
        'fixes' the hedge by tightening the gate onto ordinary BGAs.

        MUTATION: extend the ring arm to non-bulging vias -- this arm dies
        (0.5 < 0.45 + 0.1) while every drill arm above still passes."""
        add, blocked, _t = _two_balls(0.50, 0.25)
        self.assertEqual((len(add), len(blocked)), (2, 0),
                         'an ordinary 0.5mm-pitch BGA is now losing escapes; '
                         'that is the phantom rejection this scope avoids')

    def test_the_floor_is_the_BOARD_s_when_the_board_declares_one(self):
        """#620's fix inherits #756's board-first floor rather than a second
        constant. A 0.30 declaration refuses a pair the 0.20 fab floor allows.

        MUTATION: read `_h2h_fab` instead of `_h2h` in the verdict."""
        legal_at_fab = _two_balls(0.35, 0.25,
                                  source_path=_stub_board(self._tmp, 'none'))
        self.assertEqual((len(legal_at_fab[0]), len(legal_at_fab[1])), (2, 0),
                         'the undeclared board must be unchanged')
        declared = _two_balls(0.35, 0.25,
                              source_path=self.board('h30',
                                                     min_hole_to_hole=0.30))
        self.assertEqual((len(declared[0]), len(declared[1])), (1, 1),
                         'a board declaring 0.30 still gets 0.20 spacing '
                         'between the vias of one call')


class TestTwinsShareOneHole(_TmpCase):
    """Coincident same-net sites are ONE physical hole, not a mutual refusal.

    This is the half that makes the fix safe: an exposed pad modelled as an
    F.Cu + B.Cu pair puts two routes at one coordinate on 5 of this repo's 22
    boards (`interf_u` BUS1 has 31 such sites on one component), and distance 0
    is below every threshold, so a plain spacing test drops the net whole. That
    is the mechanism behind the #620 contributor's measured GND strap-pool
    collapse (53 -> 0): `_has_copper` treats a `vias_to_add` entry as a ball's
    anchor, so refusing the anchors leaves the extras nothing to strap to.
    """

    def test_two_routes_at_one_site_get_one_via(self):
        """MUTATION: return 'conflict' instead of 'twin' for a same-net
        coincident hit -- this arm dies with (0, 1) or (1, 1)."""
        add, blocked, _t = _two_balls(0.0, 0.5, same_net=True)
        self.assertEqual((len(add), len(blocked)), (1, 0),
                         'coincident same-net sites are not sharing one via')

    def test_the_shared_via_sits_where_both_balls_are(self):
        """A via merged to the WRONG place disconnects both balls while
        keeping the count right, so the count alone is not evidence."""
        add, _b, _t = _two_balls(0.0, 0.5, same_net=True)
        self.assertAlmostEqual(add[0]['x'], 10.0, places=6)
        self.assertAlmostEqual(add[0]['y'], 10.0, places=6)
        self.assertEqual(add[0]['net_id'], 7)

    def test_two_NETS_at_one_site_are_a_conflict_not_a_twin(self):
        """One hole cannot carry two nets. MUTATION: key the twin test on
        position alone and drop the net comparison."""
        add, blocked, _t = _two_balls(0.0, 0.5, same_net=False)
        self.assertEqual((len(add), len(blocked)), (1, 1),
                         'two different nets are sharing one hole, which is a '
                         'short, or the second was silently skipped')

    def test_the_pre_620_pass_STACKED_them(self):
        """The behaviour being replaced, asserted from the writer's side so the
        improvement is not just a claim: two identical dicts at one point are
        two `(via ...)` s-exprs, because `kicad_writer` does not dedupe.

        This arm pins that the FIX does not produce them. MUTATION: append
        without consulting `_pending` -- duplicates return."""
        add, _b, _t = _two_balls(0.0, 0.5, same_net=True)
        keys = [(round(v['x'], 6), round(v['y'], 6), v['net_id']) for v in add]
        self.assertEqual(len(keys), len(set(keys)),
                         'stacked same-net vias at one point are back')


class TestTheLadderKeepsTheEscape(_TmpCase):
    """A refusal here drops the escape, so the ladder is what keeps this fix
    from being the net negative the contributor measured."""

    def test_a_conflict_a_deeper_rung_rescues_keeps_BOTH_escapes(self):
        """MUTATION: delete the `thin_drill_to_clear` call and refuse
        immediately -- this arm dies (1, 1)."""
        add, blocked, _t = _two_balls(0.36, 0.32)
        self.assertEqual((len(add), len(blocked)), (2, 0),
                         'the ladder no longer rescues a pair a deeper drill '
                         'rung can space')
        self.assertEqual(sorted(v['drill'] for v in add), [0.15, 0.17],
                         'both vias kept their original drill, so the pair '
                         'that ships is still sub-floor')

    def test_the_thinned_pair_actually_clears_the_floor(self):
        """Asserting the drill VALUE is not the same as asserting the pair is
        legal; a wrong rung would satisfy the arm above. Re-derive it."""
        add, _b, _t = _two_balls(0.36, 0.32)
        a, b = add[0], add[1]
        gap = (math.hypot(a['x'] - b['x'], a['y'] - b['y'])
               - a['drill'] / 2 - b['drill'] / 2)
        self.assertGreaterEqual(round(gap, 6), H2H,
                                f'the thinned pair still ships {gap:.4f}mm of '
                                f'hole-to-hole against a {H2H}mm floor')

    def test_the_escalation_is_DISCLOSED(self):
        """A silent tier escalation is a fab cost the operator did not choose.
        MUTATION: drop the `warn_fab_escalation` call."""
        _a, _b, transcript = _two_balls(0.36, 0.32)
        self.assertIn('thinned', transcript,
                      'the drill thinning is now silent')
        self.assertIn('escalated standard->advanced fab floor', transcript,
                      'the thinning no longer routes through '
                      'warn_fab_escalation, so --fab-tier advice is lost')

    def test_no_rung_means_an_honest_drop_not_a_shipped_violation(self):
        """The deepest-rung case. MUTATION: return the deepest drill from
        `thin_drill_to_clear` regardless of `clears` -- this arm dies."""
        add, blocked, transcript = _two_balls(0.30, 0.25)
        self.assertEqual((len(add), len(blocked)), (1, 1))
        self.assertIn('escape(s) dropped', transcript,
                      'a dropped escape is not disclosed at all')
        self.assertIn('hole-to-hole floor', transcript)


class TestTheRingArmIsOnlyForBulgingVias(_TmpCase):
    """The measured scope decision, re-derived rather than restated.

    A via clamped INTO its pad adds no copper the pad did not already have, so
    testing its ring against a sibling only restates the board's own ball
    spacing. A via the clamp could not fit (status 'floor') really does bulge,
    and is tested. The sweep says every ring-only rejection with a
    non-sub-clearance pad gap is of the second kind, at every clearance.
    """

    def test_a_FITTING_pair_inside_ring_distance_is_kept(self):
        """pitch 0.45, pad 0.40: via 0.40, so ring needs 0.40 + 0.10 = 0.50 and
        the pair has 0.45 -- a ring test would refuse it. The pads themselves
        are 0.05mm apart, so that refusal fixes nothing the board did not ship.

        MUTATION: drop the `bulges` condition from the ring arm."""
        self.assertEqual(_clamped(0.40)[2], 'clamped', 'rig no longer fits')
        add, blocked, _t = _two_balls(0.45, 0.40)
        self.assertEqual((len(add), len(blocked)), (2, 0),
                         'a fitting via pair is being refused on ring '
                         'clearance -- the phantom rejection')

    def test_the_ring_arm_CANNOT_fire_alone_at_the_default_clearance(self):
        """Measured while writing this file, and worth pinning because it makes
        two of the arms here look wrong until you see it.

        A bulging via is the deepest rung's, 0.25/0.15. Its ring needs
        `0.25 + clearance`; its drill needs `0.15 + 0.20`. So the ring can only
        refuse a pair the drill accepts when

            via + clearance > drill + h2h   <=>   clearance > h2h - (via - drill)

        which at the standard tier is `clearance > 0.10`. At the CLI default of
        exactly 0.10 the two floors COINCIDE and the ring arm can never be the
        deciding one -- which is why the (pitch, pad) sweep found 0 ring-only
        real rejections at clearance 0.10 and 90/155/195/210 at 0.15/0.20/
        0.25/0.30. The arms below therefore run at 0.20.

        MUTATION: change the ring arm's `+ self._clearance` to a constant --
        this arm's arithmetic no longer describes the code."""
        vs, vd, st, _r = _clamped(0.10)
        self.assertEqual((vs, vd, st), (0.25, 0.15, 'floor'))
        self.assertAlmostEqual(vs + 0.10, vd + H2H, places=9,
                               msg='the ring and drill floors no longer '
                                   'coincide at clearance 0.10; the sweep '
                                   'numbers quoted here are about other '
                                   'geometry')

    def test_a_BULGING_pair_inside_ring_distance_is_refused(self):
        """The other half, so the arm above cannot pass by the ring arm being
        dead. pad 0.10 cannot take even the deepest rung's 0.25 via, so the via
        is held at the floor and bulges 0.075mm past the pad on every side.

        Run at clearance 0.20 for the reason the previous arm measures, and at
        a pitch where the DRILL clears -- otherwise the refusal cannot be
        attributed to the ring at all.

        MUTATION: delete the ring arm entirely -- this arm dies while every
        drill arm still passes."""
        vs, vd, st, _r = _clamped(0.10)
        self.assertEqual(st, 'floor', 'rig no longer bulges')
        self.assertGreaterEqual(0.40, vd + H2H,
                                'the rig pitch must CLEAR the drill floor, or '
                                'this arm cannot attribute the refusal to the '
                                'ring')
        self.assertLess(0.40, vs + 0.20,
                        'the rig pitch must be inside ring distance')
        add, blocked, _t = _two_balls(0.40, 0.10, clearance=0.20)
        self.assertEqual((len(add), len(blocked)), (1, 1),
                         'a bulging via pair inside ring clearance ships '
                         'anyway; the ring arm is dead')

    def test_a_FITTING_pair_at_the_same_distance_is_kept(self):
        """The pair to the arm above, holding pitch and clearance fixed and
        moving ONLY the bulge. Without this, 'refused at 0.40' could be any
        rule at all.

        pad 0.30 takes the 0.30 rung exactly, so the via fits and does not
        bulge; its drill is thinned to 0.15 by the annular ring, so the drill
        floor (0.35) still clears at 0.40."""
        vs, vd, st, _r = _clamped(0.30)
        self.assertEqual(st, 'clamped', 'rig no longer fits its pad')
        self.assertGreaterEqual(0.40, vd + H2H)
        self.assertLess(0.40, vs + 0.20, 'rig is not inside ring distance, so '
                                         'it proves nothing about the ring')
        add, blocked, _t = _two_balls(0.40, 0.30, clearance=0.20)
        self.assertEqual((len(add), len(blocked)), (2, 0),
                         'a FITTING via pair is refused on ring clearance -- '
                         'the phantom rejection the scope exists to avoid')

    def test_the_sweep_THIS_SCOPE_RESTS_ON_still_says_so(self):
        """The scope decision is a measurement, so re-derive it here rather
        than quoting it in prose. A number that lives only in a terminal is
        exactly the kind that goes stale inside a confident comment.

        For every physically possible (pitch, pad) -- pad < pitch, so the balls
        do not overlap -- classify the pair the way the two candidate rules
        would. The claim: of the pairs a RING rule would reject and the DRILL
        rule would not, every one whose own pads are NOT already sub-clearance
        is a BULGING via. If that ever stops holding, the ring arm's `bulges`
        condition is refusing (or admitting) something new and this file's
        argument for it no longer applies.

        MUTATION: none in the engine -- this arm guards the DESIGN. It fails if
        `clamp_via_to_pad` or the fab ladder changes shape, which is precisely
        when the scope wants re-deciding."""
        recorded = {0.10: 0, 0.15: 90, 0.20: 155, 0.25: 195, 0.30: 210}
        pitches = [round(0.20 + i * 0.01, 4) for i in range(101)]
        pads = [round(0.05 + i * 0.01, 4) for i in range(116)]
        for clearance, expect in sorted(recorded.items()):
            ring_only_real = 0
            not_bulging = []
            combos = 0
            for pitch in pitches:
                for psize in pads:
                    if psize >= pitch:
                        continue
                    combos += 1
                    vs, vd, _st, _r = _clamped(psize)
                    d_bad = pitch < vd + H2H - 1e-9
                    r_bad = pitch < vs + clearance - 1e-9
                    if d_bad or not r_bad:
                        continue
                    if pitch - psize < clearance - 1e-9:
                        continue                       # phantom: pads already
                    ring_only_real += 1
                    if vs <= psize + 1e-9:
                        not_bulging.append((pitch, psize, vs, vd))
            self.assertEqual(combos, 6565,
                             'the sweep grid moved; the recorded counts below '
                             'are about a different population')
            self.assertEqual(
                not_bulging, [],
                f'at clearance {clearance}: {len(not_bulging)} ring-only '
                f'rejection(s) are NOT bulging vias, e.g. {not_bulging[:3]} -- '
                f'the `bulges` condition now excludes a real catch, so the '
                f'drill-only-unless-bulging scope needs re-deciding')
            self.assertEqual(
                ring_only_real, expect,
                f'at clearance {clearance} the sweep now finds '
                f'{ring_only_real} ring-only real rejections, not the '
                f'{expect} this scope was chosen against')

    def test_the_ring_arm_is_foreign_net_only(self):
        """Same-net copper in contact is not a clearance violation. Distance
        0.40 clears the drill floor (0.35) so only the ring can decide.

        MUTATION: drop the `onet != net_id` condition -- this arm dies."""
        p = PendingVias(H2H, 0.20)
        p.add(10.0, 10.0, 0.25, 0.15, 7, bulges=True)
        same = p.verdict(10.40, 10.0, 0.25, 0.15, 7, bulges=True)
        self.assertEqual(same[0], 'clear',
                         'a same-net bulging pair is refused on ring '
                         'clearance it does not owe')
        foreign = p.verdict(10.40, 10.0, 0.25, 0.15, 8, bulges=True)
        self.assertEqual(foreign[0], 'conflict',
                         'the ring arm is dead: a foreign bulging pair 0.40 '
                         'apart needs 0.45')


class TestPendingViasItself(unittest.TestCase):
    """The helper, directly. Module-level and pure precisely so it can be."""

    def test_the_broad_phase_agrees_with_brute_force(self):
        """The window is an optimisation, and a wrong window is INVISIBLE in
        every aggregate count -- it just silently stops refusing. Compared
        against an exhaustive scan over pseudo-random sites, fixed seed so a
        failure is reproducible.

        MUTATION: shrink the window (drop the `self._clearance` term, or use
        `d` instead of `d/2 + max/2`) -- this arm dies."""
        rng = random.Random(620)
        for trial in range(200):
            h2h = rng.choice((0.15, 0.20, 0.30))
            clearance = rng.choice((0.10, 0.20))
            p = PendingVias(h2h, clearance)
            rows = []
            for _ in range(rng.randint(1, 25)):
                x = round(rng.uniform(0.0, 3.0), 4)
                y = round(rng.uniform(0.0, 3.0), 4)
                s = rng.choice((0.25, 0.30, 0.45))
                d = rng.choice((0.15, 0.20, 0.30))
                n = rng.randint(1, 3)
                b = rng.random() < 0.5
                cand = (x, y, s, d, n, b)
                got = p.verdict(*cand)[0]
                want = _brute(rows, cand, h2h, clearance)
                self.assertEqual(got, want,
                                 f'trial {trial}: broad phase disagrees with '
                                 f'brute force at {cand} over {len(rows)} '
                                 f'placed')
                if got == 'clear':
                    p.add(*cand)
                    rows.append(cand)

    def test_it_uses_math_hypot_not_numpy(self):
        """#786/#787 closed on the finding that numpy's hypot is not CPython's
        Neumaier-compensated one -- they disagree by 1 ULP on ~17% of off-grid
        inputs, and each disagreement feeds a `dist < floor` comparison. A
        vectorised port here would be a behaviour change needing a corpus A/B,
        not a free win; it is also slower at this size (2.0ms vs 12.8ms on the
        largest in-repo BGA). Asserted on the SOURCE because there is no other
        way to catch a well-meaning port.

        Stripped of comments first: this file's own prose mentions numpy, and
        so does the class docstring, so a naive grep passes on the explanation
        of why not to.
        """
        import inspect
        import bga_fanout.geometry as g
        src = inspect.getsource(g.PendingVias)
        code = '\n'.join(ln for ln in src.splitlines()
                         if not ln.lstrip().startswith('#'))
        code = code.split('"""')[0] + '"""'.join(code.split('"""')[2:])
        self.assertIn('math.hypot(', code)
        self.assertNotIn('numpy', code)
        self.assertNotIn('np.', code)


def _brute(rows, cand, h2h, clearance, tol=1e-6, site_tol=0.001):
    """The window-free spec `PendingVias.verdict` must match."""
    x, y, s, d, net, bulges = cand
    best = None
    for (ox, oy, os_, od, onet, obulges) in rows:
        dist = math.hypot(ox - x, oy - y)
        if dist <= site_tol:
            return 'twin' if onet == net else 'conflict'
        if dist < d / 2 + od / 2 + h2h - tol:
            if best is None or dist < best:
                best = dist
            continue
        if (bulges or obulges) and onet != net:
            if dist < s / 2 + os_ / 2 + clearance - tol:
                if best is None or dist < best:
                    best = dist
    return 'conflict' if best is not None else 'clear'


class TestTheLadderHelper(unittest.TestCase):
    def test_it_returns_the_LARGEST_rung_that_clears(self):
        """A ladder that jumps straight to the deepest rung escalates the fab
        tier further than the geometry needs. MUTATION: iterate ascending."""
        self.assertEqual(thin_drill_to_clear(0.30, LADDER, 0,
                                             lambda d: d <= 0.20), 0.20)

    def test_it_returns_None_when_no_rung_clears(self):
        """MUTATION: return the last candidate instead of None -- the caller
        then ships a violation instead of dropping honestly."""
        self.assertIsNone(thin_drill_to_clear(0.30, LADDER, 0,
                                              lambda d: False))

    def test_an_override_file_leaves_NO_rung_to_descend(self):
        """`fab_floor_ladder` collapses to one hard rung under an override
        file, by design. That is the contributor's `via_drill = 0.35` arm, and
        why this fix cannot rescue it. MUTATION: make the ladder fall back to
        the packaged tiers when overrides are set."""
        one = fab_floor_ladder(4, overrides={'via_drill': 0.35})
        self.assertEqual(len(one), 1,
                         'an override file no longer collapses the ladder; '
                         'a run that pinned its fab limits can now escalate '
                         'past them')
        self.assertIsNone(thin_drill_to_clear(0.35, one, 0, lambda d: True),
                          'a single-rung ladder must offer nothing thinner')


if __name__ == '__main__':
    unittest.main(verbosity=2)
