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
    hole in the neighbourhood is one this pass creates, and a thinner drill is
    a lever that can actually cure a conflict;
  * the RING is tested only when a via BULGES past its pad (clamp status
    `'floor'`), because a via clamped INTO its pad asks the fab for the etch
    pitch the footprint already demands, while a bulging one asks for a tighter
    one -- and because a via-in-pad site has no lever at all: it is the ball
    centre by definition and a bulging via is already at the deepest fab rung.

AN EARLIER VERSION OF THIS FILE JUSTIFIED THAT SPLIT WITH A SWEEP -- "of the
ring-only rejections whose pads are not already sub-clearance, 100% are bulging
vias, at every clearance" -- and an adversarial review showed the claim is a
TAUTOLOGY: `pitch >= pad + clearance` and `pitch < via + clearance` give
`pad < via`, which IS the bulge, for any clamp function whatsoever. It is kept
below AS an identity (`test_the_bulge_EQUIVALENCE_is_an_identity...`) so nobody
re-derives it and reads it as evidence, and the contingent quantity is measured
separately: what a bulge-blind ring arm would additionally reject.

It is ALSO NOT TRUE that a fitting via adds no copper, which is what that sweep
was standing in for. A ball pad is `['F.Cu']` and the via spans F.Cu to B.Cu,
so on the inner layers there is no pad under the ring at all. The honest
argument is the etch-pitch one above, not a claim that the copper is already
there.

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

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from kicad_parser import Pad, BoardInfo, parse_kicad_pcb        # noqa: E402
from synth import make_pcb                                       # noqa: E402
import fab_tiers                                                 # noqa: E402
from fab_tiers import fab_floor_ladder, fab_floor_min            # noqa: E402
from bga_fanout import manage_vias, ball_has_copper             # noqa: E402
from bga_fanout.types import FanoutRoute                         # noqa: E402
from bga_fanout.geometry import (PendingVias, thin_drill_to_clear,  # noqa: E402
                                 clamp_via_to_pad, via_anchors_route)

CU = ('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu')
H2H = fab_floor_min(len(CU))['hole_to_hole']          # 0.20, standard tier
# #857: the standard tier is a HARD floor now; these tests exercise the
# standard->advanced descent, which lives under --fab-tier auto.
fab_tiers.set_default_fab_tier('auto')
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
        # #857: the descent under test is the auto tier's.
        fab_tiers.set_default_fab_tier('auto')
        fab_tiers.set_escalation_policy('fab')

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

    def test_a_via_the_route_cannot_REACH_is_not_its_twin(self):
        """#854. THIS ARM WAS REVERSED, deliberately, with a measurement.

        It used to read `test_a_twin_FURTHER_OUT_than_either_floor_is_still_a
        _twin` and assert that two routes 0.9mm apart inside one 2.0mm pad get
        ONE via -- keyed on the candidate's pad rectangle. That is the defect
        #854 reports: the surviving via has a 0.225mm radius, so it is 0.9mm
        from the second route's track start and touches nothing. The pad does
        not carry the connection -- the ball pad is on the top layer and the
        route's track is on an inner one, so the pad reaches its own track only
        THROUGH the via. One via there ships an inner-layer track connected to
        nothing, while the route still counts as escaped.

        Measured on a tracked board before the change (kit-dev-coldfire-
        xilinx_5213, VR201, a TO-263-5 whose pad 3 is a 10.8 x 9.4mm SMD tab
        plus four 5.25 x 4.55mm paste sub-pads, all net GND): routing the
        sub-pad first made the TAB adopt its via, 3.6853mm from the tab route's
        track start against a 0.35mm reach, and the run printed
        `#620: 1 coincident same-net site(s) share one via` for a pair 3.69mm
        apart. Tab-first emitted two vias. After the change both orders emit
        two and neither route is stranded.

        Every case #620 needs still merges: those are at distance 0 (an exposed
        pad modelled as an F.Cu + B.Cu pair -- interf_u BUS1 has 31 such sites
        on one component), which is inside any reach.

        MUTATION: widen the twin test back to a pad-box containment -- this
        arm dies."""
        p = PendingVias(H2H, 0.1)
        p.add(10.0, 10.0, 0.45, 0.2, 7)
        self.assertEqual(p.verdict(10.9, 10.0, 0.45, 0.2, 7,
                                   track_width=0.1)[0], 'clear',
                         'a via 0.9mm from this route track start, with a '
                         '0.225mm ring, was called its anchor')
        # ... and end to end, where the second route now gets the via it needs.
        add, blocked, _t = _two_balls(0.9, 2.0, same_net=True)
        self.assertEqual((len(add), len(blocked)), (2, 0),
                         'two routes 0.9mm apart in one 2.0mm pad shared a '
                         'via that reaches neither')
        for r in add:
            self.assertTrue(
                any(via_anchors_route(r['x'], r['y'], r['size'], pos, 0.1)
                    for pos in ((10.0, 10.0), (10.9, 10.0))),
                'an emitted via anchors no route')

    def test_TIGHTENING_the_survivor_must_not_break_the_reach(self):
        """The interaction #854 created, found by review before any board hit.

        `verdict` decides 'twin' from the COMMITTED via's size, and the branch
        then calls `tighten` to replace it with the tighter pad's clamp (#202,
        so the shared via cannot bulge past the smaller pad). A smaller barrel
        reaches less far -- so a merge can be justified by a via that, once
        tightened, no longer touches the second route's track start. That is
        the very stranding #854 is about, recreated by the fix for it. It could
        not happen under the old pad-BOX rule, which does not depend on via
        size at all, so the hazard is new with this change.

        The numbers: a 0.60 via committed at (0, 0), a second route 0.30mm away
        with a 0.30mm track. Reach is 0.30 + 0.15 = 0.45 >= 0.30, so it is a
        twin. Tighten to that route's 0.25 clamp and reach becomes
        0.125 + 0.15 = 0.275 < 0.30 -- it no longer reaches.

        MUTATION: drop the post-tighten re-check in the twin branch."""
        p = PendingVias(0.20, 0.25)
        p.add(0.0, 0.0, 0.60, 0.30, 7)
        self.assertEqual(
            p.verdict(0.30, 0.0, 0.25, 0.15, 7, track_width=0.30)[0], 'twin',
            'the rig no longer produces a twin here, so it cannot detect the '
            'shrink breaking one')
        self.assertFalse(
            via_anchors_route(0.0, 0.0, min(0.60, 0.25), (0.30, 0.0), 0.30),
            'the rig no longer SHRINKS out of reach, so it tests nothing')
        big = _ball(0.0, 0.0, 7, 1.40, 'BIG')
        small = _ball(0.30, 0.0, 7, 0.25, 'SML')
        r_big = FanoutRoute(pad=big, pad_pos=(0.0, 0.0), stub_end=(0.0, 1.0),
                            exit_pos=(0.0, 1.5), layer='B.Cu')
        r_small = FanoutRoute(pad=small, pad_pos=(0.30, 0.0),
                              stub_end=(0.30, -1.0), exit_pos=(0.30, -1.5),
                              layer='B.Cu')
        pcb = make_pcb(board_info=BoardInfo(layers={}, copper_layers=list(CU),
                                            board_bounds=(-5.0, -5.0,
                                                          5.0, 5.0)),
                       vias=[], segments=[], pads_by_net={7: [big, small]},
                       source_path='', zones=[])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            add, _rm, _blocked = manage_vias([r_big, r_small], pcb, 'F.Cu',
                                             0.60, 0.30, 0.25,
                                             track_width=0.30)
        for r in (r_big, r_small):
            self.assertTrue(
                any(via_anchors_route(v['x'], v['y'], v['size'], r.pad_pos,
                                      0.30) for v in add),
                f'route at {r.pad_pos} has no via reaching its track start '
                f'after the merge tightened the shared one: '
                f'{[(v["x"], v["y"], v["size"]) for v in add]}')

    def test_the_REACH_term_in_the_broad_phase_window(self):
        """`verdict`'s window is `max(drill term, ring term, reach term)`, and
        the reach term (widest committed via + half a track) is the one nothing
        else makes binding.

        A wide track inverts the usual order: with via 0.25 / clearance 0.05 /
        h2h 0.2 the drill term is 0.35 and the ring term 0.30, while a 0.5mm
        track puts reach at 0.375. A twin 0.36mm away is inside reach and
        OUTSIDE the other two windows, so without the reach term it is never
        scanned and the second route gets a second hole it does not need.

        MUTATION: drop the reach term from `window = max(...)` -- this arm
        dies."""
        p = PendingVias(0.2, 0.05)
        p.add(10.0, 10.0, 0.25, 0.15, 7)
        drill_term = 0.15 / 2 + 0.15 / 2 + 0.2
        ring_term = 0.25 / 2 + 0.25 / 2 + 0.05
        reach_term = 0.25 / 2 + 0.5 / 2
        self.assertGreater(reach_term, max(drill_term, ring_term),
                           'the rig no longer makes the REACH term the '
                           'binding one, so it cannot detect its loss')
        self.assertEqual(p.verdict(10.36, 10.0, 0.25, 0.15, 7,
                                   track_width=0.5)[0], 'twin',
                         'a same-net via this route reaches was outside the '
                         'broad-phase window, so it was never scanned')

    def test_a_LARGE_pad_does_not_swallow_a_SMALLER_overlapping_one(self):
        """#854's own case, and the third in this family.

        A scalar radius merged DISTINCT oblong pads (fixed in PR #852). A
        per-axis rectangle closed the equal-size case -- two same-net pads of
        the same size can only be twins if they overlap by more than half --
        and left this one: a BIG pad's box reaches a SMALL pad that only just
        overlaps it, so the big pad adopts the small one's via and ships an
        inner-layer track starting where no via reaches.

        The maintainer's probe, reproduced: a 1.0 x 1.0mm pad at (10.0, 10.0)
        and a 0.25 x 0.25mm pad at (10.4, 10.0), same net, both routed on B.Cu,
        via 0.45 / drill 0.2 / clearance 0.1 on an empty board. Small-first
        used to emit ONE via and strand the big route; big-first emitted two.
        Order-dependence was the tell.

        MUTATION: key the twin test on the candidate's pad box again -- this
        arm dies."""
        for label, first_big in (('small-first', False), ('big-first', True)):
            big = _ball(10.0, 10.0, 7, 1.0, 'BIG')
            small = _ball(10.4, 10.0, 7, 0.25, 'SML')
            r_big = FanoutRoute(pad=big, pad_pos=(10.0, 10.0),
                                stub_end=(10.0, 10.9), exit_pos=(10.0, 11.4),
                                layer='B.Cu')
            r_small = FanoutRoute(pad=small, pad_pos=(10.4, 10.0),
                                  stub_end=(10.4, 9.1), exit_pos=(10.4, 8.6),
                                  layer='B.Cu')
            order = [r_big, r_small] if first_big else [r_small, r_big]
            pcb = make_pcb(
                board_info=BoardInfo(layers={}, copper_layers=list(CU),
                                     board_bounds=(0.0, 0.0, 20.0, 20.0)),
                vias=[], segments=[], pads_by_net={7: [big, small]},
                source_path='', zones=[])
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                add, _rm, blocked = manage_vias(order, pcb, 'F.Cu', 0.45, 0.2,
                                                0.1, track_width=0.1)
            for r in order:
                self.assertTrue(
                    any(via_anchors_route(v['x'], v['y'], v['size'],
                                          r.pad_pos, 0.1) for v in add),
                    f'{label}: route at {r.pad_pos} ships an inner-layer '
                    f'track with no via reaching its start, and still counts '
                    f'as escaped -- vias at '
                    f'{[(v["x"], v["y"], v["size"]) for v in add]}')
            self.assertEqual(len(blocked), 0, f'{label}: a route was dropped')

    def test_a_REAL_board_reaches_the_swallow(self):
        """The issue says no board in kicad_files/ reaches this. It is wrong,
        and the counter-example is worth keeping: a bound that drops its own
        counterexample proves nothing.

        kit-dev-coldfire-xilinx_5213 VR201 is a TO-263-5 whose pad 3 is a
        10.8 x 9.4mm SMD tab PLUS four 5.25 x 4.55mm SMD paste sub-pads, all
        net GND -- ordinary KiCad geometry, not a constructed case. The tab's
        old anchor box was (5.41, 4.71) and the nearest sub-pad sits at
        dx 2.775 / dy 2.425, inside it on both axes, so sub-pad-first made the
        tab adopt a via 3.6853mm from its own track start.

        Caveat kept honestly: VR201 has exactly 6 GND pads, so an UNFILTERED
        run excludes it at `fanout_candidate_nets`' plane_min_pads (default 6);
        a --nets filter admits it. The geometry is real either way, which is
        what this arm tests.

        MUTATION: key the twin test on the candidate's pad box again."""
        board = os.path.join(ROOT, 'kicad_files',
                             'kit-dev-coldfire-xilinx_5213.kicad_pcb')
        if not os.path.exists(board):
            self.skipTest('corpus board not present')
        pcb = parse_kicad_pcb(board)
        fp = pcb.footprints['VR201']
        smd = [p for p in fp.pads
               if not (p.drill or 0) and p.net_id
               and any(l.endswith('.Cu') for l in (p.layers or []))]
        tab = max(smd, key=lambda p: p.size_x * p.size_y)
        sub = min((p for p in smd if p is not tab),
                  key=lambda p: math.hypot(p.global_x - tab.global_x,
                                           p.global_y - tab.global_y))
        self.assertLessEqual(abs(sub.global_x - tab.global_x),
                             tab.size_x / 2 + 0.01)
        self.assertLessEqual(abs(sub.global_y - tab.global_y),
                             tab.size_y / 2 + 0.01)
        sep = math.hypot(sub.global_x - tab.global_x,
                         sub.global_y - tab.global_y)
        self.assertGreater(sep, 3.0,
                           'the rig no longer has a FAR pair inside the box, '
                           'so it cannot detect the swallow')
        routes = []
        for pad, dy in ((sub, 1.0), (tab, -1.0)):     # sub FIRST: the order
            routes.append(FanoutRoute(                # that used to strand
                pad=pad, pad_pos=(pad.global_x, pad.global_y),
                stub_end=(pad.global_x, pad.global_y + dy),
                exit_pos=(pad.global_x, pad.global_y + 2 * dy), layer='B.Cu'))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            add, _rm, blocked = manage_vias(routes, pcb, 'F.Cu', 0.45, 0.2,
                                            0.1, track_width=0.25)
        for r in routes:
            self.assertTrue(
                any(via_anchors_route(v['x'], v['y'], v['size'], r.pad_pos,
                                      0.25) for v in add),
                f'VR201 route at {r.pad_pos} is stranded: nearest via is '
                f'{min(math.hypot(v["x"] - r.pad_pos[0], v["y"] - r.pad_pos[1]) for v in add):.4f}'
                f'mm away, reach is {add[0]["size"] / 2 + 0.125:.4f}mm')

    def test_the_DOWNSTREAM_ball_anchor_test_asks_reach_too(self):
        """#854's second site, and the one that made the stranding silent.

        `ball_has_copper` is what decides whether an unescaped extra ball gets
        strapped to its net's fanout. Its VIA arm used to ask
        `max(size_x, size_y) / 2 + 0.01` in BOTH axes -- the scalar-radius shape
        PR #852's review had already removed from `PendingVias.verdict` -- and
        the twin branch cites it by name as its justification: "the ball-anchor
        test downstream looks for a via inside the pad, and this is that via".
        So fixing `verdict` alone leaves a swallowed ball reading as ANCHORED
        and never strapped: two graders agreeing in the wrong direction.

        The numbers are the maintainer's own repro. A 1.0mm pad's old tolerance
        is 0.51mm, so a via 0.4mm away -- with a 0.225mm ring, touching nothing
        on the inner layer -- passed.

        This arm exists because the mutation row for that revert SURVIVED while
        the predicate was a closure: nothing could call it.

        MUTATION: revert the via arm to the pad box -- this arm dies."""
        pad = _ball(10.0, 10.0, 7, 1.0, 'BIG')
        far = [{'net_id': 7, 'x': 10.4, 'y': 10.0, 'size': 0.45}]
        self.assertFalse(
            ball_has_copper(pad, far, [], 0.1),
            'a via 0.4mm from this ball, ring 0.225mm, counts as its copper -- '
            'the ball will never be strapped and ships unconnected')
        near = [{'net_id': 7, 'x': 10.1, 'y': 10.0, 'size': 0.45}]
        self.assertTrue(ball_has_copper(pad, near, [], 0.1),
                        'a via that DOES reach the ball is not credited')
        self.assertFalse(
            ball_has_copper(pad, [dict(near[0], net_id=8)], [], 0.1),
            'a FOREIGN-net via is credited as this ball\'s copper')
        # The track arm keeps the pad-box tolerance on purpose: a track
        # endpoint anywhere on the pad copper does connect it.
        on_pad = [{'net_id': 7, 'layer': 'F.Cu', 'start': (10.4, 10.0),
                   'end': (12.0, 10.0)}]
        self.assertTrue(
            ball_has_copper(pad, [], on_pad, 0.1),
            'a track ENDPOINT on this ball pad is no longer credited -- the '
            'via fix was applied to the wrong arm')
        off_layer = [{'net_id': 7, 'layer': 'In1.Cu', 'start': (10.4, 10.0),
                      'end': (12.0, 10.0)}]
        self.assertFalse(
            ball_has_copper(pad, [], off_layer, 0.1),
            'copper crossing the pad on ANOTHER layer counts as a connection')

    def test_an_OBLONG_pad_does_not_swallow_its_NEIGHBOUR(self):
        """The regression an adversarial review found in the twin rule's first
        version, which took `max(size_x, size_y) / 2 + 0.01` as a RADIUS and
        compared it against a straight-line distance. On a 0.30 x 1.50 finger
        that radius is 0.76mm -- more than twice the pad's own half-width --
        so two same-net fingers 0.50mm apart, whose copper is 0.20mm apart and
        NOT touching, merged into ONE via and the second route shipped with no
        via while still counting as escaped. Exactly the defect the sibling
        commit fixes, re-created by the merge. 17 footprints on 11 in-repo
        boards match this geometry (glasgow U1/J5/U21, kit-dev U102/U301,
        watchy U4/J3, rp2350 U6, tigard U5/U6, lvds IC2/IC3, orangecrab J6).

        MUTATION: make `anchor_box` a scalar radius again, or compare `dist`
        instead of the per-axis deltas -- this arm dies."""
        a = _ball(10.0, 10.0, 7, 0.3, 'A1')
        a.size_y = 1.5
        b = _ball(10.5, 10.0, 7, 0.3, 'A2')
        b.size_y = 1.5
        ra = FanoutRoute(pad=a, pad_pos=(10.0, 10.0), stub_end=(10.0, 10.9),
                         exit_pos=(10.0, 11.4), layer='B.Cu')
        rb = FanoutRoute(pad=b, pad_pos=(10.5, 10.0), stub_end=(10.5, 9.1),
                         exit_pos=(10.5, 8.6), layer='B.Cu')
        pcb = make_pcb(board_info=BoardInfo(layers={}, copper_layers=list(CU),
                                            board_bounds=(0.0, 0.0, 20.0, 20.0)),
                       vias=[], segments=[], pads_by_net={7: [a, b]},
                       source_path='', zones=[])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            add, _rm, blocked = manage_vias([ra, rb], pcb, 'F.Cu', 0.45, 0.2,
                                            0.1)
        self.assertEqual(
            (len(add), len(blocked)), (2, 0),
            'two DISTINCT same-net pads 0.50mm apart were merged into one '
            'via, so one route ships with no via and still counts escaped')
        for pad in (a, b):
            self.assertTrue(
                any(abs(v['x'] - pad.global_x) <= pad.size_x / 2 + 0.01
                    and abs(v['y'] - pad.global_y) <= pad.size_y / 2 + 0.01
                    for v in add),
                f'pad at ({pad.global_x}, {pad.global_y}) has no via inside it')

    def test_there_is_no_ONE_MICRON_CLIFF(self):
        """An exact-match twin rule had one, and an adversarial review found
        it: two same-net sites 0.0010mm apart merged into one via and both
        balls kept their escapes, while 0.0011mm apart DROPPED one outright --
        because no fab rung can space two holes 1.1 um apart, so the conflict
        branch had nothing to descend to. A 100-nanometre difference in a
        footprint's modelling decided whether a ball escaped.

        The twin rule keys on the ball's own ANCHOR radius instead (the
        `_has_copper` spelling), so anything inside the pad is a via that
        already connects this ball.

        MUTATION: replace `anchor_box` with `self._tol` in `verdict` -- the
        0.0011 row dies."""
        for sep in (0.0, 0.0005, 0.0010, 0.0011, 0.05, 0.2):
            add, blocked, _t = _two_balls(sep, 0.5, same_net=True)
            self.assertEqual(
                (len(add), len(blocked)), (1, 0),
                f'same-net balls {sep}mm apart (well inside a 0.5mm pad) '
                f'gave {len(add)} via(s) and {len(blocked)} dropped escape(s)')

    def test_two_NETS_at_one_site_are_a_conflict_not_a_twin(self):
        """One hole cannot carry two nets. MUTATION: key the twin test on
        position alone and drop the net comparison."""
        add, blocked, _t = _two_balls(0.0, 0.5, same_net=False)
        self.assertEqual((len(add), len(blocked)), (1, 1),
                         'two different nets are sharing one hole, which is a '
                         'short, or the second was silently skipped')

    def test_no_two_vias_share_a_point(self):
        """MUTATION: append without consulting `_pending` -- duplicates return.

        An earlier docstring here claimed this arm asserted "from the writer's
        side", which it never did -- it only checks the dicts. A reviewer
        checked the underlying claim instead and it holds:
        `add_tracks_and_vias_to_pcb` on `splitflap_driver` with two identical
        via dicts emits TWO `(via ...)` at one point. The claim is true and is
        pinned by `test_the_writer_really_does_not_dedupe` below, which calls
        the writer rather than describing it."""
        add, _b, _t = _two_balls(0.0, 0.5, same_net=True)
        keys = [(round(v['x'], 6), round(v['y'], 6), v['net_id']) for v in add]
        self.assertEqual(len(keys), len(set(keys)),
                         'stacked same-net vias at one point are back')

    def test_the_writer_really_does_not_dedupe(self):
        """The premise the twin rule rests on, CALLED rather than asserted
        about. If the writer ever grows a dedupe, sharing one via stops being
        an improvement over stacking two and this file's argument changes.

        MUTATION: none -- this pins a fact about a different module."""
        from kicad_writer import add_tracks_and_vias_to_pcb
        board = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'kicad_files',
            'splitflap_driver.kicad_pcb')
        if not os.path.isfile(board):
            self.skipTest('fixture board missing')
        with open(board, encoding='utf-8') as f:
            before = f.read().count('(via')
        via = {'x': 100.0, 'y': 100.0, 'size': 0.4, 'drill': 0.2,
               'layers': ['F.Cu', 'B.Cu'], 'net_id': 0}
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'w.kicad_pcb')
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                add_tracks_and_vias_to_pcb(board, out, [],
                                           [dict(via), dict(via)])
            with open(out, encoding='utf-8') as f:
                after = f.read().count('(via')
        self.assertEqual(after - before, 2,
                         'the writer now dedupes identical vias, so stacking '
                         'is no longer the thing the twin rule avoids')

    def test_the_surviving_twin_via_is_the_TIGHTER_pad_s_clamp(self):
        """Which via survives a merge used to be whichever route arrived first,
        so two coincident same-net pads of 0.25 and 0.60 kept a 0.45 via that
        bulges past the 0.25 pad -- the #202 violation `clamp_via_to_pad`
        exists to prevent, re-created by the merge. Found by an adversarial
        review. Asserted in BOTH orders, because the defect was order-only and
        one order passed the whole time.

        MUTATION: delete the `_pending.tighten` block -- the big-pad-first
        order ships a 0.45 via."""
        for order in ((0.25, 0.60), (0.60, 0.25)):
            a = _ball(10.0, 10.0, 7, order[0], 'A1')
            b = _ball(10.0, 10.0, 7, order[1], 'A2')
            ra = FanoutRoute(pad=a, pad_pos=(10.0, 10.0),
                             stub_end=(10.5, 10.5), exit_pos=(11.0, 10.5),
                             layer='B.Cu')
            rb = FanoutRoute(pad=b, pad_pos=(10.0, 10.0),
                             stub_end=(10.5, 9.5), exit_pos=(11.0, 9.5),
                             layer='B.Cu')
            pcb = make_pcb(board_info=BoardInfo(
                layers={}, copper_layers=list(CU),
                board_bounds=(0.0, 0.0, 20.0, 20.0)),
                vias=[], segments=[], pads_by_net={7: [a, b]},
                source_path='', zones=[])
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                add, _rm, blocked = manage_vias([ra, rb], pcb, 'F.Cu', 0.45,
                                                0.2, 0.1)
            self.assertEqual((len(add), len(blocked)), (1, 0),
                             f'order {order}: the twins did not merge')
            self.assertLessEqual(
                add[0]['size'], min(order) + 1e-9,
                f'order {order}: the surviving via is {add[0]["size"]}mm, '
                f'which bulges past the {min(order)}mm pad (#202)')


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

    def test_the_THINNED_drill_is_what_gets_recorded(self):
        """The pending record must carry the drill that actually ships, or the
        NEXT candidate is spaced against a hole that does not exist. Passing
        the pre-thin drill is over-strict rather than unsafe.

        THIS ARM RUNS `manage_vias`, and that is the whole point. An earlier
        version asserted on `PendingVias` directly -- it built one record with
        0.15 and one with 0.17 and checked they differ -- which is true of the
        class and says NOTHING about which one the call site passes. A reviewer
        applied the exact mutation the docstring named (capture the drill
        before the ladder, hand THAT to `_pending.add`) and all 32 tests stayed
        green while a real escape was lost.

        Three balls in a row: the first pair conflicts and is rescued by the
        ladder, and the third is spaced against the SECOND via's recorded
        drill. With the thinned value recorded it clears; with the pre-thin
        value it is refused.

        MUTATION: `_pending.add(..., <the pre-thin drill>, ...)` -- this arm
        dies (2 added, 1 blocked)."""
        pads, routes = [], []
        for i, x in enumerate((10.0, 10.36, 10.71)):
            p = _ball(x, 10.0, 7 + i, 0.32, f'A{i}')
            pads.append(p)
            routes.append(FanoutRoute(
                pad=p, pad_pos=(x, 10.0), stub_end=(x, 10.5),
                exit_pos=(x, 11.0), layer='B.Cu'))
        pcb = make_pcb(board_info=BoardInfo(layers={},
                                            copper_layers=list(CU),
                                            board_bounds=(0.0, 0.0, 20.0, 20.0)),
                       vias=[], segments=[],
                       pads_by_net={7 + i: [p] for i, p in enumerate(pads)},
                       source_path='', zones=[])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            add, _rm, blocked = manage_vias(routes, pcb, 'F.Cu', 0.45, 0.2, 0.1)
        self.assertEqual(
            (len(add), len(blocked)), (3, 0),
            'the third ball was refused, which means it was spaced against a '
            'drill wider than the one actually recorded')
        self.assertEqual(sorted(v['drill'] for v in add), [0.15, 0.15, 0.17],
                         'the drills that shipped are not the ladder result')

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

    def test_the_bulge_EQUIVALENCE_is_an_identity_not_a_measurement(self):
        """An earlier version of this file swept 6565 (pitch, pad) pairs and
        reported "of the ring-only rejections whose pads are not already
        sub-clearance, 100% are bulging vias, at every clearance" as the
        measurement the scope rested on. An adversarial review showed it is a
        TAUTOLOGY, and the honest thing is to keep it -- as an identity, so
        nobody re-derives it and mistakes it for evidence:

            ring-only rejection   =>  pitch <  via + clearance
            pads not sub-clearance =>  pitch >= pad + clearance
            therefore                  pad  <  via, which IS the bulge

        It holds for ANY clamp function, so this arm proves it with clamps that
        have nothing to do with the real one. If it ever fails, the two
        definitions have drifted apart and the whole framing needs redoing.

        MUTATION: none. This arm guards a claim, not a branch."""
        clamps = {'huge': lambda p: 5.0, 'tiny': lambda p: 0.01,
                  'triple': lambda p: 3 * p, 'exact': lambda p: p,
                  'real': lambda p: _clamped(p)[0]}
        for name, via_of in clamps.items():
            for clearance in (0.05, 0.10, 0.20, 0.35, 0.50):
                bad = []
                for i in range(20, 121):
                    pitch = round(i * 0.01, 4)
                    for j in range(5, 121):
                        psize = round(j * 0.01, 4)
                        if psize >= pitch:
                            continue
                        vs = via_of(psize)
                        if pitch >= vs + clearance - 1e-9:
                            continue                   # not a ring rejection
                        if pitch - psize < clearance - 1e-9:
                            continue                   # pads already that close
                        if vs <= psize + 1e-9:
                            bad.append((pitch, psize, vs))
                self.assertEqual(
                    bad, [], f'clamp {name!r} at clearance {clearance}: '
                             f'{len(bad)} ring-only non-phantom pair(s) that '
                             f'do NOT bulge, e.g. {bad[:2]} -- the identity '
                             f'this file documents has broken')

    def test_what_a_BULGE_BLIND_ring_arm_would_additionally_reject(self):
        """The contingent quantity the tautology above is not. How many real
        (pitch, pad) combinations does the shipped arm KEEP that a ring arm
        without the `bulges` condition would refuse? That number is the cost of
        the alternative design, and unlike the identity it can move.

        Counted over the same grid at the CLI-default clearance and at three
        wider ones. Recorded, so a change in `clamp_via_to_pad` or the fab
        ladder shows up here as a moved number rather than silently."""
        recorded = {0.10: 150, 0.15: 310, 0.20: 495, 0.25: 705}
        for clearance, expect in sorted(recorded.items()):
            extra = 0
            combos = 0
            for i in range(20, 121):
                pitch = round(i * 0.01, 4)
                for j in range(5, 121):
                    psize = round(j * 0.01, 4)
                    if psize >= pitch:
                        continue
                    combos += 1
                    vs, vd, _st, _r = _clamped(psize)
                    if pitch < vd + H2H - 1e-9:
                        continue          # the drill arm refuses it anyway
                    if pitch < vs + clearance - 1e-9 and vs <= psize + 1e-9:
                        extra += 1        # ring-blind would refuse; we keep
            self.assertEqual(combos, 6565,
                             'the grid moved; the counts here are about a '
                             'different population')
            self.assertEqual(
                extra, expect,
                f'at clearance {clearance} a bulge-blind ring arm would '
                f'additionally refuse {extra} combination(s), not the {expect} '
                f'recorded when this scope was chosen')

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


class TestTheSilentSkipIsNowAnHonestDrop(_TmpCase):
    """#620's other half, in the SAME two guards.

    `would_overlap_existing_via` used to gate the append as `if not ...:
    append`, so when it fired the via was dropped and the ROUTE was kept: its
    inner-layer track shipped anyway, connected to nothing, while the ball
    still counted as escaped. The guard two lines above does the opposite --
    `via_blocked_routes`, whose tracks the caller strips -- and the #508
    comment at this function's call site says exactly why that shape was
    adopted: the old code "left the sibling routes in `routes` -- still counted
    escaped, shipping via-in-pad balls with no track."

    THE REFUSAL SET IS UNCHANGED; only the bookkeeping is. Measured on
    orangecrab_ext_pll U3 at defaults -- the one in-repo board that reaches
    this branch -- it fires 11 times over the retry passes (4 distinct nets,
    every blocker foreign) and the written board does not change, because each
    stranded route was removed by a later filter anyway.

    The rig needs a blocker that trips the RING guard while CLEARING the drill
    guard that runs first, or the arm attributes the drop to the wrong rule: a
    via of size 0.6 and drill 0.1 at 0.5mm needs 0.625 of ring (fires) and
    0.35 of drill (clears).
    """

    def _run(self, sep, ov_size=0.6, ov_drill=0.1):
        from synth import make_via
        ball = _ball(10.0, 10.0, 7, 0.5)
        r = FanoutRoute(pad=ball, pad_pos=(10.0, 10.0), stub_end=(10.5, 10.5),
                        exit_pos=(11.0, 10.5), layer='B.Cu')
        ov = make_via(10.0 + sep, 10.0, net_id=9, size=ov_size, drill=ov_drill)
        pcb = make_pcb(board_info=BoardInfo(layers={},
                                            copper_layers=list(CU),
                                            board_bounds=(0.0, 0.0, 20.0, 20.0)),
                       vias=[ov], segments=[], pads_by_net={7: [ball]},
                       source_path='', zones=[])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            add, _rm, blocked = manage_vias([r], pcb, 'F.Cu', 0.45, 0.2, 0.1)
        return len(add), len(blocked)

    def test_ON_THE_BRANCH_the_blocker_trips_the_ring_and_clears_the_drill(self):
        """Without this, a drop at 0.5mm could be the drill guard and the arm
        below would be about a different rule entirely."""
        v_drill, ov_drill, h2h = 0.2, 0.1, H2H
        self.assertGreaterEqual(0.5, v_drill / 2 + ov_drill / 2 + h2h,
                                'the rig blocker no longer clears the DRILL '
                                'guard, which runs first')
        self.assertLess(0.5, 0.45 / 2 + 0.6 / 2 + 0.1,
                        'the rig blocker no longer trips the RING guard, so '
                        'nothing refuses and the arm is inert')

    def test_the_refused_escape_is_REPORTED_not_silently_stranded(self):
        """MUTATION: restore `if not would_overlap_existing_via(...): append`
        -- the via count stays 0 and only the BLOCKED count catches it, which
        is exactly why this arm asserts the pair."""
        self.assertEqual(self._run(0.5), (0, 1),
                         'a via refused on ring clearance leaves its route in '
                         'place again: an inner-layer track with no via')

    def test_a_clearing_blocker_still_gets_its_via(self):
        """The acceptance half, so the arm above cannot pass on a rig that
        refuses everything. 0.7 > 0.625."""
        self.assertEqual(self._run(0.7), (1, 0),
                         'the ring guard is now refusing a via that clears it')


class TestPendingViasItself(unittest.TestCase):
    """The helper, directly. Module-level and pure precisely so it can be."""

    def test_the_broad_phase_agrees_with_brute_force(self):
        """The window is an optimisation, and a wrong window is INVISIBLE in
        every aggregate count -- it just silently stops refusing. Compared
        against an exhaustive scan over pseudo-random sites, fixed seed so a
        failure is reproducible.

        MUTATION: shrink the window (drop the `self._clearance` term, or use
        `d` instead of `d/2 + max/2`) -- this arm dies.

        THE PARAMETER GRID IS THE TEST. An earlier version of this arm swept
        drills 0.15/0.20/0.30 against clearances 0.10/0.20 and named that
        second mutation in its own docstring while being UNABLE TO KILL IT:
        with those numbers the ring term always dominated the max(), so the
        drill half of the window was never load-bearing and could be corrupted
        freely. An adversarial review found it, and the fix is not a new
        assertion but a grid where each term binds in turn -- drills that
        exceed every ring, rings that exceed every drill, and floors of zero.
        Several seeds, because one seed is a sample of one."""
        for seed in (620, 1, 7, 12345, 99991):
            rng = random.Random(seed)
            for trial in range(120):
                # Ranges chosen so the drill term and the ring term each
                # dominate the window in some trials, and neither always.
                h2h = rng.choice((0.0, 0.05, 0.20, 0.30, 1.10))
                clearance = rng.choice((0.0, 0.02, 0.10, 0.20, 0.90))
                p = PendingVias(h2h, clearance)
                rows = []
                for _ in range(rng.randint(1, 25)):
                    x = round(rng.uniform(-2.0, 3.0), 4)
                    y = round(rng.uniform(-2.0, 3.0), 4)
                    s = round(rng.choice((0.05, 0.25, 0.45, 1.20, 2.40)), 4)
                    d = round(rng.choice((0.0, 0.05, 0.20, 1.10, 2.40)), 4)
                    n = rng.randint(1, 3)
                    b = rng.random() < 0.5
                    cand = (x, y, s, d, n, b)
                    got = p.verdict(*cand)[0]
                    # Feed the brute force the SAME rows in the SAME order the
                    # scan sees them (x-sorted). The window is what is under
                    # test; iteration order is not, and at a zero floor two
                    # rows can both sit inside the site tolerance, where first-
                    # hit-wins would make an order difference look like a
                    # window bug.
                    want = _brute(list(p._rows), cand, h2h, clearance)
                    self.assertEqual(got, want,
                                     f'seed {seed} trial {trial}: broad phase '
                                     f'disagrees with brute force at {cand} '
                                     f'over {len(rows)} placed')
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


def _brute(rows, cand, h2h, clearance, tol=1e-6, site_tol=0.001,
           track_width=0.0):
    """The window-free spec `PendingVias.verdict` must match.

    The twin arm CALLS `via_anchors_route` rather than restating it: what this
    spec exists to test is the broad-phase WINDOW, and a hand-copied reach
    formula here would just be a second place for the two to drift apart.
    """
    x, y, s, d, net, bulges = cand
    best = None
    for (ox, oy, os_, od, onet, obulges) in rows:
        dist = math.hypot(ox - x, oy - y)
        if onet == net and via_anchors_route(ox, oy, os_, (x, y), track_width):
            return 'twin'
        if dist <= site_tol:
            return 'conflict'
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
