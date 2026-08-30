#!/usr/bin/env python3
"""#735: the via-nudger's connector copper is priced at the board's TRACK-scoped
`.kicad_dru` rules, and at those rules ONLY where both sides are tracks.

THE DEFECT. `nudge_vias_for_unresolved` draws a connector back to a relocated
via's stub and gates that copper itself, in `connector_clear`. Its seg-vs-seg
arm resolved through `required`, which carries net classes and the `.kicad_dru`
LAYER rules but not the TRACK-scoped ones -- a separate channel that check_drc
applies at the seg-seg site, which is exactly the arm in question. On a board
declaring a track-to-track rule the pass therefore drew copper closer to a
foreign track than its own checker accepts. It UNDER-blocked, so the violation
shipped rather than the landing being refused.

THE FIX has three parts and this file pins each separately, because they fail
for different reasons: `kicad_dru.track_pair_clearance` is the one binding
predicate (check_drc delegates to it), `PadClearanceModel` carries the rules
and the class memberships, and `_Repair.track_required` is the resolver the
nudger reads through `getattr` like every other.

THE RULE IS A PREFERENCE, NOT A GATE, and that is the load-bearing decision.
A hard refusal would abandon the via -- which leaves the #130 pad-via graze
this pass exists to remove sitting on the board, and `check_drc` counts that
too. So a gate would trade one counted violation for another AND lose the
repair. The sweep therefore runs every drill rung honouring the rule first,
and only if nothing clears does it run again at the base requirement and keep
the repair, saying so on stdout. Same doctrine as the #756 drill rungs.

THE MEASURED LADDER. The connector is 0.2 wide, the foreign track is 0.2 wide,
so the requirement between them is `0.1 + 0.1 + R`: 0.30 with no rule, 0.65
under a 0.45 rule. Placing one foreign track a chosen perpendicular distance
from the connector's unruled path brackets that number from both sides:

    dperp   no dru        with the 0.45 rule
    0.28    ALT           ALT + fallback   <- the base gate is live
    0.45    RIG_LANDING   RIG + fallback   <- nothing satisfies it; repair KEPT
    0.61    RIG_LANDING   ALT              <- THE HEADLINE: the rule steers
    0.75    RIG_LANDING   RIG_LANDING      <- above 0.65 it binds nothing

The 0.75 and 0.28 rows are what make the 0.61 row mean something: without them
"the two arms differ" is equally consistent with a rule that changes every
landing and with a broken fixture. Measured, not predicted -- the 0.65 boundary
was located by sweeping and the true bracket is (0.650, 0.652].

THE 0.61 ROW IS THE NARROWEST MARGIN IN THIS FILE, and it is stated rather than
hidden: the steering band measures ~[0.563, 0.651), 0.088 wide, so 0.61 sits
0.047 from the lower edge and 0.041 from the upper -- under the 0.05 this file
otherwise keeps. Every other constant here clears its boundary by 0.10 or more.
(An earlier version of this note said the band was [0.58, 0.65), which is both
mis-measured and internally inconsistent -- that interval is 0.07 wide, not the
0.06 the note claimed. A fact-check caught it; the real margins are larger than
the wrong ones were.)

WHAT `tests/mutate_735.py` MEASURES AGAINST THIS FILE AND ITS `_e2e` SIBLING --
23 rows, 21 killed, 2 survived (both expected), 0 broken. FROM THE RUN:

     1 revert the s2 arm ........................... 6 killers
     2 swap the pair's two nets .................... SURVIVED, and must
     3 the ladder becomes an all-or-nothing gate ... 5
     4 the relaxed rung runs FIRST ................. 4
     5 the fallback is taken SILENTLY .............. 3
     6 the second rung runs on every board ......... SURVIVED, honestly
     7 the via-vs-track arm acquires the rule ...... 3
     8 track_required ignores the channel .......... 7
     9 track_required drops the flat fallback ...... 1
    10 the two handles are merged .................. 5
    11 _floors widened to admit a track-only board . 2
    12 the resolver is read without getattr ........ 2
    13 the shim falls back to the flat scalar ...... 1
    14 for_board never reads the track rules ....... 16
    15 `active` absorbs the track channel .......... 2
    16 the model forgets the memberships ........... 10
    17 track_pair is disabled ...................... 8
    18 the rule LEAKS into the pad pair resolver ... 8
    19 the predicate stops being raise-only ........ 3
    20 other_only stops exempting siblings ......... 2
    21 the rule identity is not reported ........... 6
    22 board_track_rules hands back no memberships . 10
    23 check_drc stops delegating .................. 4

ROW 2 IS A DELIBERATE SURVIVOR: the binding predicate is symmetric in its two
nets today, so swapping them at the call site cannot change an answer. Kept as
the change detector for the day it stops being symmetric.

ROW 6 IS AN HONEST SURVIVOR: dropping the `_trk_on` guard makes a board that
declares nothing sweep twice for the same answer. That is a cost, not a
different result, so no behavioural arm can catch it and none pretends to.

ROW 13 IS WHY THIS BATTERY EXISTS. It SURVIVED the first run: every fixture
here declared its net classes AT the run's clearance, so `req` and the flat
scalar were the same number and "the shim falls back through `req`" was a
claim no arm made. `test_an_st_that_predates_the_resolver_keeps_its_NETCLASS_
answer` closes it, and the row now kills. The gap was real and invisible from
a green suite.

ROWS 3-5 AND 18 EXIST BECAUSE A REVIEW FOUND WHAT THEY GUARD. Rows 3-5 pin the
ladder, which an earlier version of this change did not have -- it was a hard
gate that abandoned the repair. Row 18 is the "did the rule leak into the PAD
resolver" control, which had no row at all: the row that claimed to be it
disabled `track_pair` instead, duplicating row 8 one layer down.

CONVENTIONS FOLLOWED (the #725/#736/#747 family's):

  * REAL parser dataclasses and a REAL board -- `_Repair.__init__` reads
    courtyards and locked refs off the file on disk.
  * Every assertion names the single-line MUTATION that must kill it.
  * Assert you are ON the branch before asserting about it: a landing arm
    first checks the via actually moved, or an inert run passes it too.
  * Every "is not touched" is paired with a NEGATIVE CONTROL that is.
  * Source guards read CODE ONLY, with trailing comments stripped, and every
    count arm is paired with a positive control so a rename fails HERE rather
    than silently disarming it.

The child-process half -- the paired check_drc grade of the copper this pass
emits -- is in the `_e2e` sibling, so this file stays in the fast bucket.
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
import sys
import tempfile
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import run_utils
import check_drc
import kicad_dru
from copy_board import copy_board
from kicad_dru import TrackRule, board_track_rules, track_pair_clearance
from kicad_parser import Net, parse_kicad_pcb
from synth import make_seg, make_via
from placement import fanout_clearance as FC
from placement.legality import PadClearanceModel
from placement.fanout_clearance import _Repair, nudge_vias_for_unresolved

# The one board in the repo on which the nudge can be made to FIRE; the same
# board and cap test_725 and test_747 rig, for the same reason.
BOARD = os.path.join(_ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
CLEAR = 0.1
CAP = 'C67'
PREFIX = 'C,R,FB'          # the CLI default
RULE = 0.45

# An INNER layer on purpose. The connector necessarily starts at the OLD via
# position, which sits inside the grazed cap's keep-out; on the cap's own side
# its pads refuse every landing, and the pass emits nothing to price. Measured:
# F.Cu and B.Cu report "no clear spot" for this rig with no foreign track at
# all, In1..In4 all land identically.
LAYER = 'In1.Cu'

# Measured on the rig below at CLEAR. RIG_LANDING is test_747's number too,
# which is the cheapest available check that this rig is the family's rig.
RIG_START = (156.7307, 101.4006)
RIG_LANDING = (157.6084, 101.7641)
ALT_LANDING = (155.6807, 101.4006)

# The two nets, by name, so a board edit that renumbers them fails loudly here
# rather than quietly grading a pair that is not the pair.
CRIT_NET, CRIT_NAME = 1, '/sheetHyperRAM/RAM_VDDQ'
FOREIGN_NET, FOREIGN_NAME = 2, 'RAM_D13'

# The rule text and the project shape are copied from
# tests/test_735_dru_track_clearance_e2e.py, so the two files cannot come to
# disagree about what a track rule looks like.
DRU_BOTH = ('(version 1)\n(rule crit_space (condition "A.Type==\'track\' && '
            'B.Type==\'track\' && A.NetClass==\'CRIT\'") '
            '(constraint clearance (min %gmm)))\n' % RULE)
DRU_OTHER = ('(version 1)\n(rule crit_space (condition "A.Type==\'track\' && '
             'B.Type==\'track\' && A.NetClass==\'CRIT\' && '
             'B.NetClass!=\'CRIT\'") '
             '(constraint clearance (min %gmm)))\n' % RULE)
DRU_LAYER = ('(version 1)\n(rule r (layer "%s") '
             '(constraint clearance (min 0.5mm)))\n' % LAYER)

# The four rungs of the ladder in the docstring.
D_BASE = 0.28        # under the 0.30 base requirement: BOTH arms refuse
D_REFUSE = 0.45      # over 0.30, under 0.65: only the ruled arm refuses
D_RELOCATE = 0.61    # the ruled arm finds a DIFFERENT landing
D_INERT = 0.75       # over 0.65: the rule binds nothing, both arms agree

# A bare board that declares a track rule and NOTHING else -- no class above
# the floor, no layer rule, no pad override. The whole point of `_track` being
# a second handle is that this board keeps `_floors is None`.
BARE = ('(kicad_pcb (version 20240108)\n'
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
        '  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.05) '
        '(type solid)) (layer "Edge.Cuts"))\n)\n')


def _pro(pattern=CRIT_NAME):
    """A project declaring CRIT, at the SAME clearance as Default and as the
    run. Deliberate: `PadClearanceModel` admits a class only when it exceeds
    the board-wide floor, so this leaves `net_floor` empty and the track rule
    is the only thing the model learned from the board."""
    return {'net_settings': {
        'classes': [{'name': 'Default', 'clearance': CLEAR},
                    {'name': 'CRIT', 'clearance': CLEAR}],
        'netclass_assignments': {},
        'netclass_patterns': [{'pattern': pattern, 'netclass': 'CRIT'}]}}


def _stage(tmp, name, dru=None, pro=None, bare=False):
    """A COPY of the rig board (siblings carried by copy_board), plus a project
    and an optional .kicad_dru. `bare=True` writes the minimal outline board
    above instead -- NO board in this repo ships a .kicad_dru, so every arm
    that needs one writes it."""
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, 'b.kicad_pcb')
    if bare:
        with open(dst, 'w', encoding='utf-8') as f:
            f.write(BARE)
    else:
        copy_board(BOARD, dst)
    stem = os.path.splitext(dst)[0]
    with open(stem + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump(_pro() if pro is None else pro, f)
    if dru is not None:
        with open(stem + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write(dru)
    return dst


def _repair(path, pcb):
    """The 10-POSITIONAL construction every test in this family uses. Calling
    it positionally is itself part of the #725 shape contract."""
    return _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, PREFIX, set())


def _foreign(dperp, half_len=0.15):
    """One short track parallel to the OFF connector, `dperp` from it.

    SHORT and centred on the connector's midpoint so its distance to either
    endpoint stays well above the via's own requirement -- the via arm must not
    be what refuses a landing, or the connector arm this file is about is never
    reached. Measured at D_REFUSE: 0.55 to each endpoint against a 0.45 via
    requirement."""
    dx, dy = RIG_LANDING[0] - RIG_START[0], RIG_LANDING[1] - RIG_START[1]
    n = math.hypot(dx, dy)
    ux, uy = dx / n, dy / n
    cx = (RIG_START[0] + RIG_LANDING[0]) / 2.0 + dperp * -uy
    cy = (RIG_START[1] + RIG_LANDING[1]) / 2.0 + dperp * ux
    return make_seg(round(cx - half_len * ux, 4), round(cy - half_len * uy, 4),
                    round(cx + half_len * ux, 4), round(cy + half_len * uy, 4),
                    layer=LAYER, net_id=FOREIGN_NET, width=0.2)


def _rig(path, dperp=None):
    """A REAL board carrying ONE offending via beside C67's first pad, a
    same-net stub so a connector is actually drawn, and optionally ONE foreign
    track. Everything else is cleared so the connector arm is what decides."""
    pcb = parse_kicad_pcb(path)
    st = _repair(path, pcb)
    rect = st.caps[CAP].pad_rects()[0]
    vx, vy = rect[2] + 0.20, (rect[1] + rect[3]) / 2.0
    segs = [make_seg(vx, vy - 0.6, vx, vy, layer=LAYER, net_id=CRIT_NET,
                     width=0.2)]
    if dperp is not None:
        segs.append(_foreign(dperp))
    pcb.vias[:] = [make_via(vx, vy, net_id=CRIT_NET, size=0.5, drill=0.3)]
    pcb.segments[:] = segs
    st.vias = [st._register_via(vx, vy, CRIT_NET, radius=0.25)]
    st._via_radius_by_id = {id(st.vias[0]): (st.vias[0], 0.25)}
    st.cap_vias = {k: st.vias for k in st.caps}
    st.segments = []
    st.cap_segs = {k: [] for k in st.caps}
    return pcb, st, (vx, vy)


def _nudge(st, pcb, **kw):
    """Drive the real pass, capturing what it printed."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        moves, segs = nudge_vias_for_unresolved(st, pcb, CLEAR, **kw)
    return moves, segs, buf.getvalue()


def _landing(moves):
    return (None if not moves
            else (round(moves[0][2]['x'], 4), round(moves[0][2]['y'], 4)))


class _TmpCase(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.addCleanup(self._td.cleanup)

    def stage(self, name, **kw):
        return _stage(self._td.name, name, **kw)

    def run_ladder(self, dperp):
        """(off_landing, on_landing) for one rung, from two independent
        stages. Two stages, not one re-read: a shared directory would let the
        first run's project writeback reach the second."""
        out = []
        for tag, dru in (('off', None), ('on', DRU_BOTH)):
            p = self.stage('%s_%s' % (tag, str(dperp).replace('.', '')),
                           dru=dru)
            pcb, st, _v0 = _rig(p, dperp=dperp)
            moves, segs, text = _nudge(st, pcb, max_shift=4.0)
            out.append((_landing(moves), len(segs), text, st))
        return out


# ---------------------------------------------------------------------------
# 1. The shared resolver
# ---------------------------------------------------------------------------
class TestTheBindingPredicateIsOneFunction(unittest.TestCase):
    R_BOTH = TrackRule('crit_space', 'CRIT', False, RULE)
    R_OTHER = TrackRule('crit_space', 'CRIT', True, RULE)
    CRIT = frozenset({'CRIT'})
    NONE = frozenset()

    def test_a_member_versus_a_NON_member_binds(self):
        eff, rule = track_pair_clearance([self.R_BOTH], self.CRIT, self.NONE,
                                         CLEAR)
        self.assertEqual((eff, rule), (RULE, self.R_BOTH))
        # symmetric: which side carries the class cannot matter
        self.assertEqual(track_pair_clearance([self.R_BOTH], self.NONE,
                                              self.CRIT, CLEAR)[0], RULE)
    # MUTATION: `binds = (a_in and not b_in)` -> the mirrored call goes flat.

    def test_two_members_bind_unless_the_rule_says_other_only(self):
        self.assertEqual(track_pair_clearance([self.R_BOTH], self.CRIT,
                                              self.CRIT, CLEAR)[0], RULE)
        self.assertEqual(track_pair_clearance([self.R_OTHER], self.CRIT,
                                              self.CRIT, CLEAR),
                         (CLEAR, None))
        # NEGATIVE CONTROL: other_only must still bind the mixed pair, or the
        # arm above is satisfied by a rule that binds nothing at all.
        self.assertEqual(track_pair_clearance([self.R_OTHER], self.CRIT,
                                              self.NONE, CLEAR)[0], RULE)
    # MUTATION: drop `and not r.other_only` -> the sibling pair is charged.

    def test_two_NON_members_never_bind(self):
        self.assertEqual(track_pair_clearance([self.R_BOTH], self.NONE,
                                              self.NONE, CLEAR),
                         (CLEAR, None))
    # MUTATION: `binds = True` -> every pair on the board is charged.

    def test_it_is_RAISE_only_and_reports_no_rule_when_it_does_not_raise(self):
        # A resolved value already above the rule keeps its own value AND its
        # empty rule slot: the identity is what check_drc uses to call a pair
        # rule-governed, so a rule that changed nothing must not claim it.
        self.assertEqual(track_pair_clearance([self.R_BOTH], self.CRIT,
                                              self.NONE, 0.6), (0.6, None))
        self.assertEqual(track_pair_clearance([], self.CRIT, self.NONE, CLEAR),
                         (CLEAR, None))
    # MUTATION: `>` -> `>=`, or return `rule` unconditionally.

    def test_the_largest_binding_rule_wins_and_owns_the_identity(self):
        small = TrackRule('small', 'CRIT', False, 0.2)
        got = track_pair_clearance([self.R_BOTH, small], self.CRIT, self.NONE,
                                   CLEAR)
        self.assertEqual(got, (RULE, self.R_BOTH))
        # order must not decide it
        self.assertEqual(track_pair_clearance([small, self.R_BOTH], self.CRIT,
                                              self.NONE, CLEAR),
                         (RULE, self.R_BOTH))
    # MUTATION: `eff, rule = r.clearance_mm, r` without the `>` test.

    def test_check_drc_calls_it_rather_than_carrying_a_second_copy(self):
        """The delegation is the whole point of extracting it, and a source
        arm is the only thing that catches a re-inlining."""
        src = [l.split('#')[0] for l in
               inspect.getsource(check_drc).splitlines()]
        # ONE, not two: the import spells the name without the paren, so it is
        # counted separately below. An earlier draft of this arm expected 2 and
        # the run said 1 -- recorded rather than quietly re-fitted, because an
        # expectation edited to match its result measures nothing.
        hits = [i + 1 for i, l in enumerate(src)
                if 'track_pair_clearance(' in l]
        self.assertEqual(len(hits), 1, 'expected exactly ONE call site; '
                                       'lines %r' % hits)
        self.assertEqual(len([l for l in src
                              if 'import' in l and 'track_pair_clearance' in l]),
                         1, 'check_drc no longer imports the shared resolver')
        # ANTI-VACUITY: the predicate's own text must NOT be back in check_drc.
        # The needle is the ASSIGNMENT, not `r.other_only` -- that field is
        # also named by the announce line three lines above, so the obvious
        # needle reports a re-inlining that has not happened. Caught by this
        # arm failing on its own first run.
        self.assertEqual([i + 1 for i, l in enumerate(src) if 'binds = (' in l],
                         [], 'the binding predicate has been re-inlined')
        self.assertIn('def track_pair_clearance', inspect.getsource(kicad_dru),
                      'the resolver was renamed; every count arm above is now '
                      'searching for a dead string')
    # MUTATION: paste the loop back into `_track_pair_cl`.


# ---------------------------------------------------------------------------
# 2. The model carries the channel, and `active` did not move
# ---------------------------------------------------------------------------
class TestTheModelCarriesTheChannel(_TmpCase):
    def test_for_board_reads_the_rules_and_the_memberships(self):
        p = self.stage('read', dru=DRU_BOTH)
        m = PadClearanceModel.for_board(parse_kicad_pcb(p), CLEAR, p)
        # NOT an `active is False` arm: this fixture is a copy of a board that
        # carries pad `local_clearance` overrides, so `active` is True here for
        # reasons that have nothing to do with the track channel. A review
        # caught the earlier comment claiming otherwise. The `active` claim is
        # made where it can be made -- on the bare board, in
        # TestTheTrackHandleIsSeparateFromTheFloors.
        self.assertEqual(m.track_rules, [TrackRule('crit_space', 'CRIT',
                                                   False, RULE)])
        self.assertEqual(m.net_classes.get(CRIT_NET), frozenset({'CRIT'}))
        self.assertIsNone(m.net_classes.get(FOREIGN_NET))
    # MUTATION: drop the board_track_rules call from for_board -> empty.

    def test_track_pair_is_the_last_tier_and_raise_only(self):
        p = self.stage('tier', dru=DRU_BOTH)
        m = PadClearanceModel.for_board(parse_kicad_pcb(p), CLEAR, p)
        self.assertEqual(m.track_pair(CRIT_NET, FOREIGN_NET, CLEAR)[0], RULE)
        self.assertEqual(m.track_pair(CRIT_NET, FOREIGN_NET, 0.6)[0], 0.6)
        # NEGATIVE CONTROL: `pair` must NOT have learned about it.
        fa = m.pad_floor(make_pad_like(CRIT_NET))
        fb = m.pad_floor(make_pad_like(FOREIGN_NET))
        self.assertEqual(m.pair(fa, fb), CLEAR,
                         'a track rule reached the PAD pair resolver')
    # MUTATION: fold the track raise into `pair_with_source`.

    def test_other_only_survives_the_round_trip_into_the_model(self):
        p = self.stage('oo', dru=DRU_OTHER)
        m = PadClearanceModel.for_board(parse_kicad_pcb(p), CLEAR, p)
        self.assertTrue(m.track_rules and m.track_rules[0].other_only)
        self.assertEqual(m.track_pair(CRIT_NET, CRIT_NET, CLEAR),
                         (CLEAR, None))
        self.assertEqual(m.track_pair(CRIT_NET, FOREIGN_NET, CLEAR)[0], RULE)
    # MUTATION: parse other_only as False -> the sibling pair is charged.

    def test_a_LAYER_rule_does_not_leak_into_the_track_list(self):
        p = self.stage('layer', dru=DRU_LAYER)
        m = PadClearanceModel.for_board(parse_kicad_pcb(p), CLEAR, p)
        self.assertEqual(m.track_rules, [])
        # NEGATIVE CONTROL: it must have landed in the LAYER map, or this arm
        # is satisfied by a dru nobody read at all.
        self.assertEqual(round(m.layer_rules.get(LAYER, 0.0), 6), 0.5)
    # MUTATION: let a layer-scoped rule through _parse_track_condition.

    def test_the_notes_are_not_filed_twice_and_do_not_OVERCLAIM(self):
        """Two things at once, because they have the same cause.

        The two readers call `_parse_dru` separately and their note lists come
        back identical, so collecting both would double every line.

        And nothing here may say the track channel was HONOURED: `notes`
        reaches the operator as `pad clearance: ...` from `grade_pad_legality`
        and the quench census, neither of which reads `track_rules` -- a
        review measured that an earlier draft printed a rule those two passes
        do not apply. The pass that does apply it discloses it where it acts.
        """
        p = self.stage('notes', dru=DRU_BOTH)
        m = PadClearanceModel.for_board(parse_kicad_pcb(p), CLEAR, p)
        parse = [n for n in m.notes if 'handled by the track channel' in n]
        self.assertEqual(len(parse), 1, m.notes)
        # ON THE BRANCH: the rule really was read, so "no honoured-note" below
        # is not just an empty channel.
        self.assertEqual(len(m.track_rules), 1)
        self.assertEqual([n for n in m.notes if 'track rule' in n], [],
                         'a note claims the track channel was honoured; two '
                         'of this constructor\'s three callers never apply it')
    # MUTATION: `notes.extend(...)` the track read's notes -> two copies;
    # or re-add the per-rule "honoured" note -> the second arm fails.

    def test_a_board_declaring_no_dru_is_a_strict_no_op(self):
        p = self.stage('none')
        m = PadClearanceModel.for_board(parse_kicad_pcb(p), CLEAR, p)
        self.assertEqual((m.track_rules, m.net_classes), ([], {}))
        self.assertEqual([n for n in m.notes if 'track' in n], [])
    # MUTATION: return the rules unconditionally from board_track_rules.

    def test_board_track_rules_answers_empty_rather_than_raising(self):
        """Quiet by construction: the quiet reader is what lets a pass that
        worked before a rules file appeared keep working when one is
        unreadable."""
        p = self.stage('bad', dru='(this is not a dru')
        pcb = parse_kicad_pcb(p)
        self.assertEqual(board_track_rules(pcb, p), ([], {}))
        self.assertEqual(board_track_rules(pcb, ''), ([], {}))
        # NEGATIVE CONTROL: the same call on a GOOD file returns rules, so the
        # arm above is not satisfied by a reader that always answers empty.
        good = self.stage('good', dru=DRU_BOTH)
        self.assertEqual(len(board_track_rules(parse_kicad_pcb(good),
                                               good)[0]), 1)
    # MUTATION: drop the try/except -> the malformed arm raises.


def make_pad_like(net_id):
    """A pad-shaped object carrying only what `pad_floor` reads. Deliberately
    not a parser Pad: the claim is about the model, and a real pad would drag
    the board's own overrides into an arm that is not about them."""
    from types import SimpleNamespace
    return SimpleNamespace(net_id=net_id, local_clearance=0.0,
                           layers=[LAYER], drill=0.0, pad_type='smd')


# ---------------------------------------------------------------------------
# 3. `active` did not move, and `_Repair` keeps the model anyway
# ---------------------------------------------------------------------------
class TestTheTrackHandleIsSeparateFromTheFloors(_TmpCase):
    def _bare(self, name, dru):
        p = self.stage(name, dru=dru, bare=True)
        pcb = parse_kicad_pcb(p)
        pcb.nets = {CRIT_NET: Net(net_id=CRIT_NET, name=CRIT_NAME),
                    FOREIGN_NET: Net(net_id=FOREIGN_NET, name=FOREIGN_NAME)}
        return p, pcb

    def test_a_track_only_board_leaves_active_False_and_the_floors_None(self):
        """THE decision this change rests on. `self._floors` switches nine
        consumers between their flat and their resolved path; a track rule can
        price none of the pairs they ask about, so it must not flip it."""
        p, pcb = self._bare('bare_on', DRU_BOTH)
        m = PadClearanceModel.for_board(pcb, CLEAR, p)
        self.assertFalse(m.active, 'a track rule reached `active`')
        self.assertEqual(len(m.track_rules), 1, 'the rule was not read at all')
        st = _repair(p, pcb)
        self.assertIsNone(st._floors, 'the track rule flipped `_floors`')
        self.assertIsNotNone(st._track, 'the track channel is not live')
    # MUTATION: `active = bool(... or self.track_rules)`; or
    # `self._floors = _model if (_model.active or _model.track_rules)`.

    def test_the_resolver_still_prices_the_pair_with_the_floors_gone(self):
        """The raise is keyed on NETS, so it is correct over the flat fallback
        -- which is the whole answer on this board."""
        p, pcb = self._bare('bare_price', DRU_BOTH)
        st = _repair(p, pcb)
        self.assertIsNone(st._floors)          # ON THE BRANCH
        self.assertEqual(st.required(None, None), CLEAR)
        self.assertEqual(st.track_required(None, None, CRIT_NET, FOREIGN_NET),
                         RULE)
        # NEGATIVE CONTROL: a pair the rule does not bind stays flat.
        self.assertEqual(
            st.track_required(None, None, FOREIGN_NET, FOREIGN_NET), CLEAR)
    # MUTATION: `if self._floors is None: return base` in track_required.

    def test_no_dru_leaves_the_track_handle_None(self):
        p, pcb = self._bare('bare_off', None)
        st = _repair(p, pcb)
        self.assertIsNone(st._track)
        self.assertEqual(st.track_required(None, None, CRIT_NET, FOREIGN_NET),
                         st.required(None, None))
    # MUTATION: `self._track = _model` unconditionally -> AttributeError-free
    # but the guard below stops being exercised.


# ---------------------------------------------------------------------------
# 4. The ladder: the connector gate honours the rule, and only where it binds
# ---------------------------------------------------------------------------
class TestTheConnectorGateHonoursTheRule(_TmpCase):
    def test_the_rule_STEERS_the_landing(self):
        """THE HEADLINE. Where a rule-satisfying landing exists, the ruled arm
        takes it and the unruled arm takes the nearer one that does not."""
        (off_land, _, off_out, _), (on_land, on_n, on_out, on_st) = \
            self.run_ladder(D_RELOCATE)
        # ON THE BRANCH: the unruled arm must actually have moved, or
        # "they differ" is what a broken fixture looks like too.
        self.assertEqual(off_land, RIG_LANDING,
                         'the fixture stopped moving; out=%r' % off_out)
        self.assertIsNotNone(on_st._track, 'the rule never reached the pass')
        self.assertEqual(on_land, ALT_LANDING,
                         'expected the rule to steer the landing, got %r; '
                         'out=%r' % (on_land, on_out))
        self.assertEqual(on_n, 1, 'the repair was abandoned, not steered')
        # It found one on the STRICT rung, so it must not have disclosed a
        # fallback. A pass that relaxed here would land somewhere too.
        self.assertNotIn('took a landing at the base clearance', on_out)
    # MUTATION: `track_req(...)` -> `req(...)` at the s2 arm.

    def test_when_nothing_satisfies_the_rule_the_repair_is_KEPT_and_disclosed(self):
        """THE LADDER. At D_REFUSE no landing in the whole sweep satisfies the
        rule. The pass must NOT abandon the via: the #130 pad-via graze it
        exists to remove would simply stay on the board, and check_drc counts
        that too -- so a hard gate would trade one counted violation for
        another and lose the repair. Prefer, then fall back, and say so.

        This is the same doctrine as the #756 drill rungs, which the comment
        beside them states as 'prefer the board, keep the repair'. An earlier
        version of this change made it a hard gate; an adversarial review
        measured the trade and it is recorded here rather than in a note."""
        (off_land, off_n, off_out, _), (on_land, on_n, on_out, on_st) = \
            self.run_ladder(D_REFUSE)
        self.assertEqual(off_land, RIG_LANDING,
                         'the fixture stopped moving; out=%r' % off_out)
        self.assertIsNotNone(on_st._track, 'the rule never reached the pass')
        self.assertEqual(on_land, off_land,
                         'the repair was LOST rather than relaxed; out=%r'
                         % on_out)
        self.assertEqual((on_n, off_n), (1, 1))
        self.assertIn('took a landing at the base clearance', on_out,
                      'the pass shipped copper that does not meet the '
                      "board's rule and did not say so")
    # MUTATION: `trk_ladder = [True]` -> the repair is abandoned.

    def test_above_its_own_requirement_the_rule_binds_NOTHING(self):
        """The upper bracket. At D_INERT the gap exceeds 0.1 + 0.1 + the rule,
        so the two arms must agree exactly AND no fallback is disclosed --
        which is what makes the steering above attributable to the rule's
        VALUE and not merely to its presence."""
        (off_land, off_n, _, _), (on_land, on_n, on_out, on_st) = \
            self.run_ladder(D_INERT)
        self.assertIsNotNone(on_st._track, 'the rule was not even read')
        self.assertEqual((on_land, on_n), (off_land, off_n), on_out)
        self.assertEqual(on_land, RIG_LANDING)
        self.assertNotIn('took a landing at the base clearance', on_out)
    # MUTATION: charge the rule unconditionally instead of raise-only.

    def test_the_BASE_gate_is_live_without_any_rule_at_all(self):
        """The lower bracket. Below 0.1 + 0.1 + clearance the unruled arm
        already refuses RIG_LANDING, so the arms above are measuring a gate
        that exists rather than a rule that is the only gate."""
        (off_land, _, off_out, off_st), (on_land, _, on_out, _) = \
            self.run_ladder(D_BASE)
        self.assertIsNone(off_st._track, 'the unruled arm read a rule')
        self.assertEqual(off_land, ALT_LANDING, off_out)
        # Both arms land in the same place; the ruled one gets there on the
        # relaxed rung and says so.
        self.assertEqual(on_land, ALT_LANDING, on_out)
        self.assertIn('took a landing at the base clearance', on_out)

    def test_the_repair_is_never_LOST_on_any_rung(self):
        """The doctrine, asserted directly rather than inferred from the four
        rows above: on every rung of the ladder the ruled arm still produces a
        landing. A future 'tidy' that turns the preference back into a gate
        fails HERE with the reason, not three arms later with a coordinate."""
        for d in (D_BASE, D_REFUSE, D_RELOCATE, D_INERT):
            p = self.stage('never_%s' % str(d).replace('.', ''), dru=DRU_BOTH)
            pcb, st, _v0 = _rig(p, dperp=d)
            self.assertIsNotNone(st._track)          # ON THE BRANCH
            moves, segs, out = _nudge(st, pcb, max_shift=4.0)
            self.assertIsNotNone(_landing(moves),
                                 'the ruled arm abandoned the repair at '
                                 'dperp=%g; out=%r' % (d, out))
            self.assertEqual(len(segs), 1)
    # MUTATION: `trk_ladder = [True]`, or a hard `return False` on the rule.
    # MUTATION: none -- this arm exists to catch a fixture whose only live
    # gate is the one under test.

    def test_a_duck_typed_st_grades_flat_and_does_not_crash(self):
        """The nine harnesses in this family carry no resolver at all. The
        read must be a `getattr`, or every one of them dies here."""
        p = self.stage('duck', dru=DRU_BOTH)
        pcb, st, _v0 = _rig(p, dperp=D_REFUSE)
        fake = _FakeSt(st)
        # ON THE BRANCH: prove the guarded read is REACHED, not skipped.
        self.assertIsNone(getattr(fake, 'track_required', None))
        moves, segs, out = _nudge(fake, pcb, max_shift=4.0)
        self.assertEqual(_landing(moves), RIG_LANDING,
                         'the duck-typed harness stopped grading flat; '
                         'out=%r' % out)
        self.assertEqual(len(segs), 1)
    # MUTATION: `_trk_req = st.track_required` -> AttributeError here and in
    # test_370 / test_617 / test_756.


    def test_an_st_that_predates_the_resolver_keeps_its_NETCLASS_answer(self):
        """The shim falls back through `req`, not to the flat scalar.

        Added because `mutate_735`'s `the-shim-falls-back-to-the-flat-scalar`
        row SURVIVED on the first run: every other fixture here declares its
        classes AT the run's clearance, so `req` and `clearance` are the same
        number and the distinction was untested. This stage declares CRIT at
        0.35, which makes them differ by 0.25."""
        pro = {'net_settings': {
            'classes': [{'name': 'Default', 'clearance': CLEAR},
                        {'name': 'CRIT', 'clearance': 0.35}],
            'netclass_assignments': {},
            'netclass_patterns': [{'pattern': CRIT_NAME,
                                   'netclass': 'CRIT'}]}}
        p = self.stage('wideclass', pro=pro)
        pcb, st, _v0 = _rig(p, dperp=D_REFUSE)
        # ON THE BRANCH: the class must actually raise the pair, or `req` and
        # `clearance` agree here too and this arm proves nothing.
        self.assertEqual(st.required(st.seg_floor(CRIT_NET, LAYER),
                                     st.seg_floor(FOREIGN_NET, LAYER)), 0.35)
        moves, _segs, out = _nudge(_NoTrackResolver(st), pcb, max_shift=4.0)
        self.assertIsNone(_landing(moves),
                          'the fallback dropped to the flat scalar: 0.45mm '
                          'clears 0.30 but not the 0.55 this class demands; '
                          'out=%r' % out)
        # NEGATIVE CONTROL: the same st DOES move when the gap clears 0.55, so
        # the refusal above is the requirement and not a broken stand-in.
        pcb2, st2, _v1 = _rig(p, dperp=0.90)
        moves2, _s2, out2 = _nudge(_NoTrackResolver(st2), pcb2, max_shift=4.0)
        self.assertIsNotNone(_landing(moves2), out2)
    # MUTATION: `return req(fa, fb)` -> `return clearance` in the shim.


class _NoTrackResolver:
    """A real `_Repair` with `track_required` hidden -- the shape of an `st`
    that resolves pairs but predates this resolver. Not a hand-built fake: the
    claim is about which of TWO resolvers the shim falls back to, so both must
    be the genuine ones."""

    def __init__(self, real):
        self._real = real

    def __getattr__(self, name):
        if name == 'track_required':
            raise AttributeError(name)
        return getattr(self._real, name)


class _FakeSt:
    """Minimal stand-in carrying only what the nudger's offender loop needs --
    the #370 B3 harness shape, built from a real _Repair so the fixture cannot
    drift from the one the arms above use."""

    def __init__(self, st):
        self.caps = st.caps
        self.vias = list(st.vias)

    def graze_penalty(self, ref, cap, x, y, rot):
        return 1.0 if ref == CAP else 0.0


# ---------------------------------------------------------------------------
# 5. Where the rule must NOT reach
# ---------------------------------------------------------------------------
class TestTheRuleReachesExactlyOnePairKind(_TmpCase):
    def test_the_via_versus_track_arm_is_not_raised(self):
        """`valid_via_pos` charges a BARREL against a foreign track. KiCad's
        condition names a track on both sides, so that pair is out of scope --
        and it must stay out, or the pass refuses landings check_drc grades
        clean.

        Asserted BEHAVIOURALLY, on the geometry, not by comparing two
        resolvers: a review measured that the resolver comparison stays true
        under the very mutation this arm names, so it could not fail on it.
        The foreign track sits at a gap that clears a BARREL's own requirement
        (0.25 + 0.1 + 0.1) and violates the rule band (0.25 + 0.1 + 0.45), so
        if the via arm acquired the rule this landing would be rejected."""
        p = self.stage('viaseg', dru=DRU_BOTH)
        pcb, st, _v0 = _rig(p, dperp=None)
        via_gap = 0.50
        # On ANOTHER copper layer, which is what isolates the two arms:
        # `valid_via_pos`'s segment loop has no layer gate (a barrel spans
        # copper), while `connector_clear`'s skips a foreign track that is not
        # on the connector's layer. Same-layer, the connector arm reacts to
        # this track too and the arm measures both at once -- measured: the
        # landing moved to ALT and the failure was unattributable.
        pcb.segments.append(make_seg(
            RIG_LANDING[0] - 0.4, RIG_LANDING[1] + via_gap,
            RIG_LANDING[0] + 0.4, RIG_LANDING[1] + via_gap,
            layer='In2.Cu', net_id=FOREIGN_NET, width=0.2))
        # ON THE BRANCH: the gap must sit strictly between the two bands, or
        # the arm is satisfied by a position neither would reject.
        self.assertTrue(0.25 + 0.1 + CLEAR < via_gap < 0.25 + 0.1 + RULE,
                        'the fixture gap %g brackets nothing' % via_gap)
        self.assertIsNotNone(st._track)
        moves, _segs, out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(_landing(moves), RIG_LANDING,
                         'the via arm acquired the track rule and rejected a '
                         'barrel position check_drc grades clean; out=%r'
                         % out)
    # MUTATION: route valid_via_pos's segment loop through track_req.

    def test_the_cap_pad_and_board_pad_arms_are_not_raised(self):
        """The cap-rect arm of `connector_clear`, behaviourally.

        Same correction as the arm above: `st.required(...) == CLEAR` is true
        with the whole branch reverted, so it tested nothing. Here the CAP's
        own pad is the far side and the connector runs well inside the rule
        band of it -- which must not refuse the landing, because a cap pad is
        not a track."""
        p = self.stage('padarms', dru=DRU_BOTH)
        pcb, st, _v0 = _rig(p, dperp=None)
        cap = st.caps[CAP]
        self.assertTrue(cap.pad_floors, 'the cap carries no floors to price')
        # ON THE BRANCH: the cap pad must actually be inside the rule band of
        # the connector's start, or "it was not refused" is trivially true.
        rect = cap.pad_rects()[0]
        self.assertLess(abs(rect[2] - RIG_START[0]), RULE,
                        'the cap pad is too far away to be a control')
        self.assertIsNotNone(st._track)
        moves, segs, out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(_landing(moves), RIG_LANDING,
                         'a cap pad was priced as a track; out=%r' % out)
        self.assertEqual(len(segs), 1)
    # MUTATION: route connector_clear's cap-rect arm through track_req.

    def test_the_broad_phase_over_reach_is_not_inflated(self):
        """`_register_segment`'s keep-out is a cap-pad-vs-track reach that
        `_seg_effs` strips back. Inflating it would be cancelled on a
        declaring board and would leak into a pad pair on an inert one.

        Registered on CRIT_NET, not the foreign net: only CRIT is a member of
        the ruled class, so a mutated `_item_reach` that consulted the track
        channel would raise nothing for a non-member and this arm would stay
        green while broken. A review caught that."""
        rows = []
        for dru in (None, DRU_BOTH):
            p = self.stage('reach_%s' % ('on' if dru else 'off'), dru=dru)
            pcb, st, _v0 = _rig(p, dperp=D_REFUSE)
            if dru is not None:
                # ON THE BRANCH: the net registered below is IN the ruled
                # class, so a leaking reach would have something to raise.
                m = PadClearanceModel.for_board(parse_kicad_pcb(p), CLEAR, p)
                self.assertIn('CRIT', m.net_classes.get(CRIT_NET, ()))
            t = st._register_segment(0.0, 0.0, 1.0, 0.0, CRIT_NET, 0.2, LAYER)
            rows.append(t[5])
        self.assertEqual(rows[0], rows[1],
                         'the track rule reached the broad-phase reach')
        # NEGATIVE CONTROL: the number is a real reach, not a constant zero.
        self.assertGreater(rows[0], 0.1)
    # MUTATION: `self._item_reach` consulting the track channel.

    def test_the_rule_reaches_exactly_one_expression_in_the_nudger(self):
        src = [l.split('#')[0] for l in
               inspect.getsource(FC.nudge_vias_for_unresolved).splitlines()]

        def n(needle):
            return len([l for l in src if needle in l])

        self.assertEqual(n('track_req('), 2, 'expected the def and ONE call')
        self.assertEqual(n('req(pfl, cfl)'), 2)
        self.assertEqual(n('req(cfl, via_fl(ov.net_id))'), 1)
        self.assertEqual(n('req(vfl, seg_fl('), 1)
        # ANTI-ROT: every count above passes after a rename. Assert the
        # positive controls so a rename fails HERE.
        for token, floor in (('track_req', 3), ('_trk_req', 2), ('req(', 8)):
            self.assertGreaterEqual(n(token), floor,
                                    '%r has been renamed' % token)
    # MUTATION: a second `track_req(` call site, or reverting the one.

    def test_track_required_is_read_from_exactly_one_place(self):
        mod = [l.split('#')[0] for l in
               inspect.getsource(FC).splitlines()]
        self.assertEqual(len([l for l in mod if 'self._track' in l]), 4,
                         'expected the assignment, the two reads in '
                         'track_required, and the one in track_rules_active')
        self.assertEqual(len([l for l in mod if 'def track_required' in l]), 1)
        self.assertEqual(len([l for l in mod if 'self._floors = ' in l]), 1)
        # The two handles must not be defined in terms of each other -- that
        # merge is exactly the change this file exists to prevent.
        for l in mod:
            if 'self._track = ' in l:
                self.assertNotIn('_floors', l)
            if 'self._floors = ' in l:
                self.assertNotIn('_track', l)
    # MUTATION: `self._floors = _model if (_model.active or
    # _model.track_rules) else None` and delete `_track`.


# ---------------------------------------------------------------------------
# 6. Inert on everything this repo actually ships
# ---------------------------------------------------------------------------
class TestInertOnTheTrackedCorpus(unittest.TestCase):
    def test_no_tracked_board_declares_a_track_rule(self):
        """A self-expiring bound. Every expression this change touches is
        gated on a non-empty rule list, so on the tracked corpus the change is
        inert by construction -- and the day a board ships a .kicad_dru, this
        says so instead of the claim quietly becoming false."""
        boards = run_utils.corpus_boards()
        if not boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no tracked corpus')
        self.assertGreaterEqual(len(boards), 10,
                                'the corpus shrank; this bound is now about '
                                'a different set')
        declaring = [b for b in boards
                     if os.path.isfile(os.path.splitext(
                         os.path.join(_ROOT, b))[0] + '.kicad_dru')]
        self.assertEqual(declaring, [],
                         'a tracked board now ships a .kicad_dru; the '
                         'inertness claim needs re-measuring, not deleting')

    def test_the_rig_board_prices_identically_with_no_rules(self):
        """The paired null: the same rig, staged twice with no dru either
        time, must agree -- so a difference in the ladder above cannot be the
        staging."""
        with tempfile.TemporaryDirectory() as td:
            out = []
            for tag in ('a', 'b'):
                p = _stage(td, tag)
                pcb, st, _v0 = _rig(p, dperp=D_REFUSE)
                moves, segs, _ = _nudge(st, pcb, max_shift=4.0)
                out.append((_landing(moves), len(segs), st._track))
            self.assertEqual(out[0][:2], out[1][:2])
            self.assertEqual(out[0][0], RIG_LANDING)
            self.assertIsNone(out[0][2])


if __name__ == '__main__':
    unittest.main(verbosity=2)
