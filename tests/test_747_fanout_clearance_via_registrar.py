#!/usr/bin/env python3
"""#747: a via tuple has ONE construction site, and the nudger REPORTS.

`_Repair.__init__` owned three things about a graded via: the 4-tuple's shape
`(x, y, net_id, keepout)`, the keep-out formula, and the id-keyed radius map
whose VALUE holds the tuple so the map itself pins it alive.
`nudge_vias_for_unresolved` kept a second copy of all three -- it rebuilt the
graded via list inline and hand-carried the map invariant, reaching the private
map through a `getattr`. It was not WRONG at HEAD 988634d0; commit 6af3495a had
already fixed one bug in exactly that carry-over. It was a second place that has
to be kept in step, which is the shape #725, #731, #732, #733 and #736 have each
fixed once, and #736 is the direct precedent: one builder, one registrar, called
from `repair_fanout_clearance` and never from the duck-typed nudger.

WHAT THIS FILE HOLDS, class by class:

  TestTheNudgerReportsAndDoesNotMutate   the headline: `st.vias` survives the
                                         call as the SAME list of the SAME
                                         tuples, and the registrar then
                                         reproduces what the inline rebuild did
  TestTheRelocationCarriesTheRadius      the invariant test_725 used to own
  TestTheKeepOutIsCarriedNotRecomputed   an absurd keep-out survives, so a
                                         recomputing builder dies on a VALUE
  TestAnInjectedTupleStaysUnpriced       absent from the map -> stays absent
  TestTheListIdentityChanges             the memo-invalidation property
  TestCoincidentViasAreMatchedByNet      position AND net, with the negative
                                         control that position alone moves both
  TestAViaMovedTwice                     the second report's OLD position is the
                                         first report's landing
  TestAnUnmatchedMoveIsANoOp             an injected barrel this object never
                                         graded, and a net that disagrees
  TestOneViaRule                         the source guard, plus a directory
                                         sweep for a second construction site
                                         in a module `inspect` cannot see
  TestTheOneRealBoardArmWhereTheRegistrarRUNS
                                         the whole pass, on orangecrab, at the
                                         one configuration in the repo that
                                         actually relocates vias
  TestInertOnTheTrackedCorpus            the self-expiring bound

WHAT `tests/mutate_747.py` MEASURES AGAINST IT -- 22 rows, 20 killed,
2 survived (both expected), 0 broken, recorded from the run rather than
predicted:

     1 recompute the keep-out ................. 2 killers
     2 the derive marker is None again ........ 1     <- only arm
     3 file a radius for an unmapped tuple .... 1     <- only arm
     4 map value drops the tuple ............ . 2
     5 map value order swapped ............... 45
     6 the no-information guard is dropped .... 1     <- only arm
     7 drop the radius carry-over ............. 5
     8 drop the keep-out carry-over ........... 3
     9 match on coordinates only .............. 2
    10 match on net only ...................... 3
    11 single pass against the original list .. 3
    12 skip the rebind when nothing matched ... 1     <- only arm
    13 mutate the list in place ............... 3
    14 count moves, not tuples ................ 4
    15 skip the registrar call in the caller .. 2
    16 call the registrar from the nudger ..... 9
    17 re-inline the tuple in __init__ ........ 2
    18 survivors rebuilt instead of carried ... 4
    19 the nudger writes the graded view again  5
    20 relocate AFTER the per-cap refresh ..... 8
    21 a trailing comment names a literal ..... SURVIVED, and must
    22 the match tolerance `<` -> `<=` ........ SURVIVED, honestly

FOUR rows have exactly ONE killer -- 2, 3, 6 and 12 -- which is why those arms
exist at all. Two of them cover a latent hazard an adversarial review measured
(a keep-out slot that is itself None), and one covers a rebind whose only
consequence is structural.

ROW 20's EXPECTATION WAS WRONG, and the first run said so. It was recorded as
an expected SURVIVOR on the reasoning that no fixture relocates a via -- wrong
twice over, since this file's own real-board arm relocates nine and #736's and
#746's rigs relocate one each. The wrong prediction is left in the battery with
the correction beside it, because a battery whose expectations are edited to
match its results measures nothing.

THE HOUSE CONVENTIONS THIS FILE FOLLOWS, and why each earned its place:

  * REAL parser dataclasses and a REAL board. `_Repair.__init__` reads
    courtyards and locked refs off the file on disk.
  * Every assertion names the single-line MUTATION that must kill it, in a
    trailing comment. Those notes were written AFTER `tests/mutate_747.py` ran,
    because a coverage note is a claim like any other.
  * Assert you are ON the branch before asserting about it -- the relocation
    arms first check the via actually MOVED, or the carry-over branch never ran
    and the assertions below pass vacuously.
  * Every "is not touched" is paired with a NEGATIVE CONTROL that is.
  * The source guard reads CODE ONLY, with trailing comments stripped. This
    module's own prose names `relocate_vias` and `_register_via` in several
    places, and #756 recorded a source arm passing against a DELETED line
    because a comment two thousand lines up carried the needle.

Runs in-process in ~60 s; the corpus arm makes one `git ls-files` call.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 900

import contextlib
import inspect
import io
import os
import re
import sys
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
from kicad_parser import detect_package_type, parse_kicad_pcb
from synth import make_via
from placement import fanout_clearance as FC
from placement.fanout_clearance import (_Repair, nudge_vias_for_unresolved,
                                        repair_fanout_clearance)

# The one board in the repo on which the nudge can be made to FIRE, and the cap
# test_725 already rigs for the same reason: no tracked board relocates a via at
# the shipped 0.6mm budget, so every relocation arm here widens `max_shift`.
BOARD = os.path.join(_ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
CLEAR = 0.1
CAP = 'C67'
PREFIX = 'C,R,FB'          # the CLI default; with 'C' alone R-refs never appear

# Measured on the rig below at CLEAR: the barrel starts 0.20mm off C67's first
# pad rect and lands 0.877mm to the right and 0.364mm down. Recorded so a rig
# that stops moving is caught as a broken FIXTURE rather than as a broken fix.
RIG_LANDING = (157.6084, 101.7641)
RIG_START = (156.7307, 101.4006)


def _repair(pcb=None, path=BOARD):
    """The 10-POSITIONAL construction every test in this family uses. Calling
    it positionally is itself part of the #725 shape contract."""
    return _Repair(pcb if pcb is not None else parse_kicad_pcb(path), path,
                   CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, PREFIX, set())


def _rig(nets=None, radius=0.25, keepout=None, file_radius=True):
    """A REAL board carrying ONE (or several coincident) foreign via(s) just
    outside C67's first pad rect, with the board otherwise cleared so the via
    channel is the only thing in play.

    Returns (pcb, st, tuples, (vx, vy)). `st.vias` is assigned WHOLESALE, which
    is the shape the contract beside `_via_radius_by_id` is about: a tuple this
    object did not build carries whatever keep-out the caller gave it.
    """
    pcb = parse_kicad_pcb(BOARD)
    st = _repair(pcb)
    cap = st.caps[CAP]
    rect = cap.pad_rects()[0]
    own = {q[4] for q in cap.pads}
    fnet = next(n for n in pcb.nets if n and n not in own)
    nets = [fnet] if nets is None else nets
    vx, vy = rect[2] + 0.20, (rect[1] + rect[3]) / 2.0
    pcb.vias[:] = [make_via(vx, vy, net_id=n, size=2 * radius, drill=0.3)
                   for n in nets]
    pcb.segments[:] = []
    ko = (radius + st._item_reach(st.via_floor(nets[0]))
          if keepout is None else keepout)
    tuples = [(vx, vy, n, ko) for n in nets]
    st.vias = list(tuples)
    st._via_radius_by_id = ({id(t): (t, radius) for t in tuples}
                            if file_radius else {})
    st.cap_vias = {k: st.vias for k in st.caps}
    st.segments = []
    st.cap_segs = {k: [] for k in st.caps}
    return pcb, st, tuples, (vx, vy)


def _nudge(st, pcb, **kw):
    """Drive the real pass, capturing what it printed."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        moves, segs = nudge_vias_for_unresolved(st, pcb, CLEAR, **kw)
    return moves, segs, buf.getvalue()


def _move(ox, oy, nx, ny, net):
    """A move report shaped exactly as the nudger emits one. Hand-built where
    the arm is about the REGISTRAR rather than about the nudger's search, so
    the input is stated rather than discovered."""
    return (ox, oy, {'x': nx, 'y': ny, 'net_id': net, 'size': 0.5,
                     'drill': 0.3, 'layers': ['F.Cu', 'B.Cu'],
                     'tenting_attrs': {}})


def _old_inline_rebuild(st, moves):
    """The DELETED loop, re-implemented from 988634d0's text, so the headline
    arm is a genuine A/B rather than an assertion about the new code alone.

    Deliberately keeps the OLD match -- position only -- so the one place the
    two are allowed to disagree is visible rather than hidden.
    """
    radii = getattr(st, '_via_radius_by_id', None)
    for old_x, old_y, spec in moves:
        rebuilt = []
        for w2 in st.vias:
            if abs(w2[0] - old_x) < 1e-6 and abs(w2[1] - old_y) < 1e-6:
                moved = (spec['x'], spec['y'], w2[2], w2[3])
                if radii is not None and id(w2) in radii:
                    radii[id(moved)] = (moved, radii[id(w2)][1])
                rebuilt.append(moved)
            else:
                rebuilt.append(w2)
        st.vias = rebuilt
    return st.vias


class TestTheNudgerReportsAndDoesNotMutate(unittest.TestCase):
    """The headline. Before #747 the nudger rebuilt the graded via view itself,
    reaching a private map of `_Repair` through a getattr. It now returns a
    report and touches nothing, and the report is enough."""

    def test_the_graded_view_survives_the_call_untouched(self):
        pcb, st, tuples, _pos = _rig()
        before = st.vias
        moves, _segs, out = _nudge(st, pcb, max_shift=4.0)
        # ON THE BRANCH: the via must actually have moved, or "unchanged" is
        # what an inert run looks like too and this arm proves nothing.
        self.assertEqual(len(moves), 1, 'no via was relocated; out=%r' % out)
        self.assertIs(st.vias, before,
                      'the nudger reassigned the graded via list')
        self.assertIs(st.vias[0], tuples[0],
                      'the nudger replaced a graded via tuple')
        self.assertEqual(len(st._via_radius_by_id), 1,
                         'the nudger filed a radius entry of its own')
    # MUTATION: restore the inline rebuild in `nudge_vias_for_unresolved`.

    def test_the_registrar_reproduces_the_deleted_inline_rebuild(self):
        """A/B against the loop 988634d0 carried, on the rig that fires it.
        The two are byte-identical here because the rig has ONE via, which is
        every case the old match and the new one agree on."""
        pcb_a, st_a, _t_a, _p = _rig()
        moves, _segs, _out = _nudge(st_a, pcb_a, max_shift=4.0)
        self.assertEqual(len(moves), 1, 'no via was relocated')
        self.assertEqual(st_a.relocate_vias(moves), 1)

        pcb_b, st_b, _t_b, _q = _rig()
        old = _old_inline_rebuild(st_b, moves)

        self.assertEqual(st_a.vias, old,
                         'the registrar and the deleted loop disagree')
        rec_a = st_a._via_radius_by_id.get(id(st_a.vias[0]))
        rec_b = st_b._via_radius_by_id.get(id(old[0]))
        self.assertIsNotNone(rec_a)
        self.assertIsNotNone(rec_b)
        self.assertAlmostEqual(rec_a[1], rec_b[1], places=12,
                               msg='the carried radius differs')
    # MUTATION: any change to the tuple the registrar builds.

    def test_the_landing_is_where_the_fixture_says_it_is(self):
        """A FIXTURE guard, not a claim about the fix. If the search ever moves
        this via somewhere else, every arm above still passes while measuring a
        different experiment."""
        pcb, st, _t, pos = _rig()
        self.assertAlmostEqual(pos[0], RIG_START[0], places=3)
        self.assertAlmostEqual(pos[1], RIG_START[1], places=3)
        moves, _segs, _out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(len(moves), 1)
        self.assertAlmostEqual(moves[0][2]['x'], RIG_LANDING[0], places=3)
        self.assertAlmostEqual(moves[0][2]['y'], RIG_LANDING[1], places=3)
    # MUTATION: none -- this arm exists to catch a fixture that stopped
    # exercising what the file says it exercises.


class TestTheRelocationCarriesTheRadius(unittest.TestCase):
    """The invariant `tests/test_725_...py::test_a_RELOCATED_via_keeps_its_radius`
    used to own inside the nudger, now held against its new owner. A relocated
    via is a NEW tuple; without carrying its entry across it drops out of the
    radius map and is graded at its keep-out SLOT -- the prune OVER-reach --
    instead of at the pair's requirement."""

    def test_the_moved_tuple_keeps_its_radius_and_the_map_holds_it(self):
        pcb, st, _t, _p = _rig()
        moves, _segs, _out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(len(moves), 1, 'no via was relocated')
        self.assertEqual(st.relocate_vias(moves), 1)
        moved = st.vias[0]
        rec = st._via_radius_by_id.get(id(moved))
        self.assertIsNotNone(rec, 'the relocated via lost its radius')
        self.assertAlmostEqual(rec[1], 0.25, places=9)
        self.assertIs(rec[0], moved,
                      'the map does not hold the tuple it keys on -- a '
                      'recycled id would return another via\'s radius')
    # MUTATION: drop the `radius=` carry-over in the registrar -> rec is None;
    # or file the radius alone instead of the (tuple, radius) pair.

    def test_the_OLD_entry_is_not_deleted(self):
        """It is what keeps the pre-move tuple alive. Freeing that id lets the
        very next tuple this builder makes be handed it, silently. The map
        therefore grows past `len(self.vias)` on a nudged board, which is a
        fact about the board rather than a leak."""
        pcb, st, tuples, _p = _rig()
        moves, _segs, _out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(len(moves), 1, 'no via was relocated')
        st.relocate_vias(moves)
        self.assertEqual(len(st._via_radius_by_id), 2,
                         'the map no longer carries both the old tuple and '
                         'the new one')
        self.assertIn(id(tuples[0]), st._via_radius_by_id)
        self.assertIs(st._via_radius_by_id[id(tuples[0])][0], tuples[0])
    # MUTATION: `del self._via_radius_by_id[id(t)]` before filing the new one.


class TestTheKeepOutIsCarriedNotRecomputed(unittest.TestCase):
    """Element 3 of a tuple this object did not build is the CALLER's keep-out
    convention, and a relocation must not re-price it. The rig makes the two
    answers differ by two orders of magnitude, so a recomputing builder dies on
    a value rather than surviving a coincidence."""

    def test_an_absurd_keepout_survives_a_relocation(self):
        st = _repair()
        t = (40.0, 40.0, 11, 99.0)
        st.vias = [t]
        st._via_radius_by_id = {id(t): (t, 0.25)}
        # ON THE BRANCH: the two answers really are different here.
        recomputed = 0.25 + st._item_reach(st.via_floor(11))
        self.assertNotAlmostEqual(
            recomputed, 99.0, places=3,
            msg='the rig is degenerate: recomputing gives the same number')
        self.assertEqual(st.relocate_vias([_move(40.0, 40.0, 41.0, 40.0, 11)]),
                         1)
        self.assertAlmostEqual(st.vias[0][3], 99.0, places=12,
                               msg='the keep-out was recomputed, not carried')
    # MUTATION: drop the `keepout=` argument at the registrar's call site, so
    # the builder takes its computing branch.

    def test_the_BUILD_branch_still_computes_the_construction_time_value(self):
        """The negative control for the arm above: with no keep-out given, the
        builder must produce exactly what __init__ produced inline."""
        st = _repair()
        t = st._register_via(1.0, 2.0, 3, radius=0.25)
        self.assertAlmostEqual(
            t[3], 0.25 + st._item_reach(st.via_floor(3)), places=12)
        self.assertEqual(t[:3], (1.0, 2.0, 3))
    # MUTATION: change the over-reach spelling in `_register_via`.

    def test_a_keep_out_that_IS_None_is_carried_rather_than_derived(self):
        """The marker for "derive one" is `_DERIVE_KEEPOUT`, not None, and this
        is the arm that makes the distinction pay. Element 3 of a tuple a
        caller assigned wholesale can itself be None; spelled with None the two
        meanings collide, and an adversarial review MEASURED both halves of the
        collision -- an unmapped tuple raised, and a mapped one had its
        keep-out silently re-derived to 0.35. Nothing in the module builds such
        a tuple, so this pins a latent hazard rather than a live bug."""
        for filed in (False, True):
            st = _repair()
            t = (10.0, 20.0, 7, None)
            st.vias = [t]
            st._via_radius_by_id = {id(t): (t, 0.25)} if filed else {}
            self.assertEqual(
                st.relocate_vias([_move(10.0, 20.0, 11.0, 21.0, 7)]), 1)
            self.assertIsNone(
                st.vias[0][3],
                'a None keep-out was %s with the tuple %s the radius map'
                % ('derived away', 'in' if filed else 'absent from'))
    # MUTATION: spell the builder's default `keepout=None` again, and test
    # both halves against it -- the unmapped case raises, the mapped one
    # silently prices at 0.35.

    def test_neither_a_radius_nor_a_keepout_is_refused(self):
        """A contract violation rather than a default. Unreachable today -- the
        only caller that omits a radius is relocating a 4-tuple, which always
        carries element 3 -- so it is named instead of surfacing later as an
        arithmetic error on a None."""
        st = _repair()
        with self.assertRaises(ValueError):
            st._register_via(1.0, 2.0, 3)
    # MUTATION: delete the guard -> TypeError instead of ValueError.


class TestAnInjectedTupleStaysUnpriced(unittest.TestCase):
    """A tuple ABSENT from the radius map is graded at its own keep-out slot
    verbatim, i.e. exactly as injected. Relocating one must produce another
    ABSENT tuple, or the move starts pricing a via that `_via_effs` and
    `via_penalty`'s flat path both agreed not to price."""

    def test_an_unmapped_tuple_relocates_without_gaining_an_entry(self):
        st = _repair()
        t = (30.0, 30.0, 9, 99.0)
        st.vias = [t]
        st._via_radius_by_id = {}
        self.assertEqual(st.relocate_vias([_move(30.0, 30.0, 31.0, 30.0, 9)]),
                         1)
        self.assertEqual(st.vias, [(31.0, 30.0, 9, 99.0)])
        self.assertEqual(st._via_radius_by_id, {},
                         'the registrar invented a radius for a tuple this '
                         'object never sized')
    # MUTATION: file the entry unconditionally in `_register_via`.

    def test_the_graders_still_read_it_at_its_own_slot(self):
        """The NEGATIVE CONTROL that makes the arm above matter: an unmapped
        tuple is priced at element 3 by the very reader this protects."""
        pcb, st, tuples, _p = _rig(file_radius=False)
        cap = st.caps[CAP]
        effs = st._via_effs(CAP, cap, st.vias)
        if effs is None:
            self.skipTest('the clearance model is inert on this board, so the '
                          'flat path already reads element 3')
        self.assertTrue(any(abs(v - tuples[0][3]) < 1e-12
                            for row in effs for v in row),
                        'no eff row reads the injected keep-out slot')
    # MUTATION: none -- this arm measures the reader the contract is about.


class TestTheListIdentityChanges(unittest.TestCase):
    """`_via_effs` revalidates its per-cap memo on the identity of the list it
    is HANDED, so a rebind is what makes those rows rebuild -- and the version
    of that sentence which does NOT hold is worth stating, because an
    adversarial review measured it.

    On the REAL path the list `_via_effs` receives is the per-cap pruned
    view, not the graded list this rebinds, and `refresh_cap_vias` rebuilds
    that on the next line regardless. Mutating in place there leaves the
    end-to-end result byte-identical; only the arms below go red. The rebind
    is load-bearing for a holder that points `cap_vias` AT the graded list --
    which since #775 is a TEST idiom only, because the engine no longer points
    it there -- and for the state any second call would find.

    So these are MECHANISM arms, and this docstring says so rather than
    implying a numeric consequence the real path does not have."""

    def test_a_matched_move_rebinds_the_list(self):
        pcb, st, _t, _p = _rig()
        before = st.vias
        moves, _segs, _out = _nudge(st, pcb, max_shift=4.0)
        self.assertEqual(len(moves), 1, 'no via was relocated')
        st.relocate_vias(moves)
        self.assertIsNot(st.vias, before,
                         'the graded via list kept its identity, so the '
                         'per-cap eff memos will not rebuild')
    # MUTATION: `self.vias[i] = ...` in place of the rebuild.

    def test_an_UNMATCHED_move_rebinds_it_too(self):
        """Deliberately unconditional. A "skip the rebind when nothing matched"
        optimisation leaves a memo built over the pre-move list alive on a run
        that DID move a barrel this object does not hold -- which is exactly
        the injected-via case the harnesses in this family create."""
        st = _repair()
        t = (50.0, 50.0, 13, 0.35)
        st.vias = [t]
        before = st.vias
        self.assertEqual(st.relocate_vias([_move(99.0, 99.0, 98.0, 98.0, 13)]),
                         0)
        self.assertIsNot(st.vias, before)
        self.assertIs(st.vias[0], t)
    # MUTATION: guard the rebind on `if n:`.


class TestCoincidentViasAreMatchedByNet(unittest.TestCase):
    """The match is on POSITION AND NET, which is what the writer's own removal
    pass has matched on since #313/#344 and what the plugin's pcbnew twin
    mirrors. Position alone relocated every tuple at the vacated spot,
    including a coincident via of a DIFFERENT net that had not moved -- a
    phantom at the landing and a hole where it really is."""

    @staticmethod
    def _two():
        st = _repair()
        a = (10.0, 10.0, 3, 0.35)
        b = (10.0, 10.0, 4, 0.35)
        st.vias = [a, b]
        st._via_radius_by_id = {id(a): (a, 0.25), id(b): (b, 0.25)}
        return st, a, b

    def test_only_the_matching_net_moves(self):
        st, _a, b = self._two()
        self.assertEqual(st.relocate_vias([_move(10.0, 10.0, 12.0, 10.0, 3)]),
                         1)
        self.assertEqual(st.vias[0], (12.0, 10.0, 3, 0.35))
        self.assertIs(st.vias[1], b,
                      'the coincident via of another net was relocated too')
    # MUTATION: drop `t[2] == net` from the match.

    def test_the_NEGATIVE_control_shows_position_alone_moves_both(self):
        """The pre-#747 behaviour, re-run through the deleted loop so the
        difference is measured rather than asserted."""
        st, _a, _b = self._two()
        _old_inline_rebuild(st, [_move(10.0, 10.0, 12.0, 10.0, 3)])
        self.assertEqual([t[:3] for t in st.vias],
                         [(12.0, 10.0, 3), (12.0, 10.0, 4)],
                         'the deleted loop did not have the defect this '
                         'change fixes, so the fix has no subject')
    # MUTATION: none -- this arm pins the OLD behaviour, from a local copy.

    def test_a_net_that_disagrees_at_the_same_position_moves_nothing(self):
        st = _repair()
        g = (60.0, 60.0, 15, 0.35)
        st.vias = [g]
        self.assertEqual(st.relocate_vias([_move(60.0, 60.0, 61.0, 60.0, 16)]),
                         0)
        self.assertIs(st.vias[0], g)
    # MUTATION: drop `t[2] == net` -> the count is 1.


class TestAViaMovedTwice(unittest.TestCase):
    """The offender loop runs per unresolved cap, rebuilds its candidates from
    the board each time and keeps no moved-set, so a barrel that offends two
    caps is relocated twice and the SECOND report's old position IS the first
    report's landing. The moves must therefore be applied in order, each
    against the result of the previous."""

    def test_the_tuple_ends_at_the_SECOND_landing(self):
        st = _repair()
        c = (20.0, 20.0, 7, 0.4)
        st.vias = [c]
        st._via_radius_by_id = {id(c): (c, 0.3)}
        n = st.relocate_vias([_move(20.0, 20.0, 21.0, 20.0, 7),
                              _move(21.0, 20.0, 22.5, 20.0, 7)])
        self.assertEqual(n, 2, 'the second hop matched nothing')
        self.assertEqual(st.vias, [(22.5, 20.0, 7, 0.4)])
    # MUTATION: resolve every move against a snapshot of the pre-nudge list.

    def test_the_radius_survives_BOTH_hops(self):
        st = _repair()
        c = (20.0, 20.0, 7, 0.4)
        st.vias = [c]
        st._via_radius_by_id = {id(c): (c, 0.3)}
        st.relocate_vias([_move(20.0, 20.0, 21.0, 20.0, 7),
                          _move(21.0, 20.0, 22.5, 20.0, 7)])
        rec = st._via_radius_by_id.get(id(st.vias[0]))
        self.assertIsNotNone(rec, 'the twice-moved via lost its radius')
        self.assertAlmostEqual(rec[1], 0.3, places=12)
        self.assertEqual(len(st._via_radius_by_id), 3,
                         'expected the seed entry plus one per hop')
    # MUTATION: carry the radius only on the first hop.


class TestAnUnmatchedMoveIsANoOp(unittest.TestCase):
    """Several test files in this family inject a via into `pcb_data.vias`
    AFTER construction, so the nudger legitimately relocates barrels this
    object never graded. That is neither an error nor worth a warning; the
    returned count is what tells it apart from a relocation."""

    def test_a_move_for_a_via_this_object_never_graded(self):
        st = _repair()
        f = (50.0, 50.0, 13, 0.35)
        st.vias = [f]
        self.assertEqual(st.relocate_vias([_move(99.0, 99.0, 98.0, 98.0, 13)]),
                         0)
        self.assertIs(st.vias[0], f)
    # MUTATION: raise (or warn) when a move matches nothing.

    def test_the_count_is_TUPLES_replaced_not_moves_handed_in(self):
        st = _repair()
        a = (10.0, 10.0, 3, 0.35)
        st.vias = [a]
        n = st.relocate_vias([_move(10.0, 10.0, 11.0, 10.0, 3),
                              _move(70.0, 70.0, 71.0, 70.0, 3)])
        self.assertEqual(n, 1, 'the count reports moves rather than tuples')
    # MUTATION: `return len(via_moves)`.

    def test_an_empty_report_changes_nothing(self):
        st = _repair()
        before = list(st.vias)
        self.assertEqual(st.relocate_vias([]), 0)
        self.assertEqual(st.vias, before)
    # MUTATION: none -- a guard against a registrar that assumes a non-empty
    # list. The caller only reaches it under a non-empty guard, so this is a
    # statement about the METHOD, which is public.


class TestOneViaRule(unittest.TestCase):
    """The source guard, in the shape #736's is: assert the invariant no
    fixture can reach, report offending LINE NUMBERS rather than dumping a
    130KB module, and carry an anti-vacuity arm so a rename fails HERE instead
    of silently disarming every `== 1` and `== 0` row.

    It reads CODE ONLY. This module's prose names both new members in several
    places, and #756 recorded a source arm passing against a DELETED line
    because a comment two thousand lines up carried the needle."""

    @staticmethod
    def _code(obj):
        return [l.split('#')[0] for l in inspect.getsource(obj).splitlines()]

    def _sites(self, lines, literal):
        return [i + 1 for i, l in enumerate(lines) if literal in l]

    def setUp(self):
        self.mod = self._code(FC)
        self.rep = self._code(FC._Repair)
        self.builder = self._code(FC._Repair._register_via)
        self.registrar = self._code(FC._Repair.relocate_vias)
        self.nudger = self._code(FC.nudge_vias_for_unresolved)

    def test_the_radius_map_has_exactly_one_writer(self):
        sites = self._sites(self.mod, 'self._via_radius_by_id[')
        self.assertEqual(
            len(sites), 1,
            'the radius map is written at %d sites (module-relative line(s) '
            '%s). #747 exists because the nudger was a second writer, '
            'reaching it through a getattr. A second is that defect again.'
            % (len(sites), sites))
        self.assertTrue(any('self._via_radius_by_id[' in l
                            for l in self.builder),
                        'the map write moved out of _register_via')
    # MUTATION: file the entry from the registrar as well.

    def test_a_via_tuple_is_APPENDED_in_exactly_one_place(self):
        sites = self._sites(self.mod, 'self.vias.append(')
        self.assertEqual(len(sites), 1,
                         'the graded via list is appended to at %d sites '
                         '(module-relative line(s) %s)' % (len(sites), sites))
    # MUTATION: append from the registrar instead of substituting.

    def test_the_list_is_REBOUND_in_exactly_one_place(self):
        """A regex, not a literal: `self.vias: List[...] = []` in __init__ is
        the ANNOTATED DECLARATION, not a second writer, and a literal count
        would either miss the rebind or charge the declaration."""
        asg = re.compile(r'self\.vias\s*=[^=]')
        writes = [i + 1 for i, l in enumerate(self.mod) if asg.search(l)]
        self.assertEqual(len(writes), 1,
                         'the graded via list is rebound at %d sites '
                         '(module-relative line(s) %s); the registrar must '
                         'be the only one' % (len(writes), writes))
        self.assertTrue(any(asg.search(l) for l in self.registrar),
                        'the rebind moved out of relocate_vias')
        inplace = re.compile(r'self\.vias\[[^\]]*\]\s*=')
        bad = [i + 1 for i, l in enumerate(self.mod) if inplace.search(l)]
        self.assertEqual(bad, [],
                         'the graded via list is mutated IN PLACE at line(s) '
                         '%s -- the _cap_via_eff memo revalidates on the '
                         'list IDENTITY, so the rows would never rebuild'
                         % bad)
    # MUTATION: `self.vias[i] = ...`, or rebind from __init__ as well.

    def test_neither_new_member_is_reached_from_the_duck_typed_nudger(self):
        """`nudge_vias_for_unresolved` takes a duck-typed `st` -- nine test
        files drive it with a stand-in, and one of those carries no via list
        at all. A resolver there has an honest flat fallback; a MUTATION does
        not, because "silently do nothing" IS this defect."""
        for tok in ('relocate_vias', '_register_via', '_via_radius_by_id',
                    'st.vias'):
            self.assertEqual(
                self._sites(self.nudger, tok), [],
                '%r is reached from the nudger again' % tok)
        calls = [i + 1 for i, l in enumerate(self.mod)
                 if 'relocate_vias(' in l and 'def ' not in l]
        self.assertEqual(len(calls), 1,
                         'expected exactly one call site for the registrar; '
                         'found %d at %s' % (len(calls), calls))
    # MUTATION: move the call inside the nudger -> nine test files
    # AttributeError on their stand-in.

    def test_the_registrar_does_not_touch_the_per_cap_view(self):
        """`refresh_cap_vias` owns the per-cap view (#775), and the caller
        calls it on the line below -- exactly where the rebuild was inline. A
        registrar that also refreshed would be a second place deciding what a
        cap can see.

        A WRITE-shape regex, not `'cap_vias' not in ...`, and not a
        read-shape one either: the method's own docstring names the attribute
        AND indexes it, and a docstring is a code line that no comment-stripper
        removes. Two successive versions of this arm failed on that prose --
        which is the trap this file's own header warns about, sprung twice by
        the guard written to catch it. It now asserts what it always meant:
        the registrar does not ASSIGN the per-cap view, by any spelling."""
        write = re.compile(r'self\.cap_vias(?:\[[^\]]*\])?\s*=[^=]'
                           r'|self\.cap_vias\s*\.\w+\(')
        bad = [i + 1 for i, l in enumerate(self.registrar) if write.search(l)]
        self.assertEqual(bad, [],
                         'the registrar writes the per-cap via view at '
                         'method-relative line(s) %s' % bad)
    # MUTATION: refresh the per-cap lists from inside the registrar.

    def test_no_OTHER_module_builds_a_graded_via_tuple(self):
        """A DIRECTORY sweep, not inspect: the arms above are scoped to this
        one module, so a second construction site in a NEW file would fire
        none of them.

        `py_tools/animate_fanout_clearance.py` unpacks the 4-tuple to draw it
        and is the known, allowed reader -- it never re-enters `_Repair` and
        never writes the radius map. It is named here so the sweep stays a
        change detector rather than a permanent red."""
        allowed = {'py_placer/placement/fanout_clearance.py',
                   'py_tools/animate_fanout_clearance.py'}
        hits = set()
        for sub in ('py_placer', 'py_router', 'py_tools',
                    'kicad_routing_plugin'):
            for dirpath, _dirs, files in os.walk(os.path.join(_ROOT, sub)):
                if '__pycache__' in dirpath:
                    continue
                for fn in files:
                    if not fn.endswith('.py'):
                        continue
                    p = os.path.join(dirpath, fn)
                    with io.open(p, encoding='utf-8', errors='replace') as f:
                        txt = f.read()
                    if '_via_radius_by_id' in txt:
                        hits.add(os.path.relpath(p, _ROOT)
                                 .replace(os.sep, '/'))
        self.assertEqual(hits, {'py_placer/placement/fanout_clearance.py'},
                         'the radius map is reached from %s' % sorted(hits))
        self.assertTrue(allowed)      # the docstring's list is not vestigial
    # MUTATION: write the radius map from any other module under py_placer/.

    def test_the_count_arms_are_not_searching_for_dead_strings(self):
        """ANTI-ROT. Every count arm above passes after a rename. Assert the
        positive controls so a rename fails HERE instead of disarming them."""
        for token, lines, floor in (('_via_radius_by_id', self.mod, 5),
                                    ('self.vias', self.mod, 4),
                                    ('_register_via', self.mod, 3),
                                    ('relocate_vias', self.mod, 2),
                                    ('via_moves', self.nudger, 3)):
            hits = self._sites(lines, token)
            self.assertGreaterEqual(
                len(hits), floor,
                '%r appears at %d code lines; the arms above were written '
                'against at least %d and are now searching for a dead string'
                % (token, len(hits), floor))
    # MUTATION: rename either new member -> this arm fails instead of the
    # `== 1` rows passing vacuously.


class TestTheOneRealBoardArmWhereTheRegistrarRUNS(unittest.TestCase):
    """Every other arm in this file either hand-builds a report or widens
    `max_shift` past anything a chain would use. This one drives the WHOLE pass
    on a tracked board, at a configuration that genuinely relocates vias, so
    the new code executes on real geometry.

    It took finding: at the shipped defaults 0 of the 33 in-repo boards print a
    single `via-nudge: moved` line. The three kwargs below are each
    load-bearing -- with the fallback off but the default displacement and pass
    count, this same board reports nothing unresolved and the nudger is never
    called. Recorded so the next person does not conclude the code is
    unreachable on real boards.
    """

    BOXED = dict(clearance=0.1, max_displacement=0.0, max_passes=1,
                 via_clear_fallback=False)

    # Measured at 42e09a1a AND at the merge base 988634d0 -- identical, which
    # is the whole claim of this change.
    N_MOVES = 9
    N_CONNECTORS = 17
    N_UNRESOLVED = 10
    N_PLACEMENTS = 18
    VIA_FREED = ['C19', 'C44', 'C45']

    def _run(self):
        pcb = parse_kicad_pcb(BOARD)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            r = repair_fanout_clearance(pcb, BOARD, **self.BOXED)
        return pcb, r, buf.getvalue()

    def test_the_whole_pass_relocates_nine_vias_and_credits_three_caps(self):
        pcb, r, out = self._run()
        self.assertEqual(len(r['via_moves']), self.N_MOVES)
        self.assertEqual(len(r['new_segments']), self.N_CONNECTORS)
        self.assertEqual(len(r['unresolved']), self.N_UNRESOLVED)
        self.assertEqual(len(r['placements']), self.N_PLACEMENTS)
        self.assertEqual(sorted(r['via_resolved']), self.VIA_FREED)
        self.assertEqual(
            sum(1 for l in out.splitlines() if 'via-nudge: moved' in l),
            self.N_MOVES,
            'the transcript and the returned report disagree about how many '
            'barrels moved')
    # MUTATION: any change to the registrar that alters the re-grade -- e.g.
    # skipping the call, or refreshing the per-cap view before it.

    def test_every_relocated_barrel_is_where_the_report_says_it_is(self):
        """The end-to-end form of the whole issue: after the pass, the PARSER's
        vias and the report must agree, and neither may still sit at a vacated
        position. Reads the board through the parser rather than the grader, so
        it cannot be satisfied by the view this change repairs."""
        pcb, r, _out = self._run()
        live = {(round(v.x, 6), round(v.y, 6)) for v in pcb.vias}
        for old_x, old_y, vd in r['via_moves']:
            landing = (round(vd['x'], 6), round(vd['y'], 6))
            self.assertIn(landing, live,
                          'a reported landing carries no via: %r' % (landing,))
            self.assertNotIn(
                (round(old_x, 6), round(old_y, 6)), live,
                'a via is still at a vacated position %r' % ((old_x, old_y),))
    # MUTATION: none in the battery -- this arm measures the NUDGER's own
    # bookkeeping, which #747 does not touch. It is here as a change detector
    # for the fixture, so a configuration that stops relocating is caught.


class TestInertOnTheTrackedCorpus(unittest.TestCase):
    """A self-expiring bound. This change's whole claim is "nothing observable
    changes", and the strongest form of that on real boards is that the branch
    is never taken: at the shipped defaults no tracked board MOVES a via, so
    the registrar is never reached and no output can differ. If that stops
    being true, the claim has expired and this says so rather than quietly
    passing."""

    @staticmethod
    def _reaching():
        boards = run_utils.corpus_boards()
        if not boards:
            return None
        out = []
        for b in boards:
            try:
                pcb = parse_kicad_pcb(b)
            except Exception:                                    # noqa: BLE001
                continue
            if not pcb.vias:
                continue
            # The ENGINE's own selector, not a name grep -- detect_package_type
            # also answers 'BGA' for LGA / CSP / WLCSP / WLP / CGA and for a
            # geometric ball field with no keyword at all. #746 measured that a
            # name grep selects 3 of the tracked boards where the detector
            # selects 4, and the missed one is exactly the board that reaches
            # the nudger. A bound that excludes its own counterexample is not a
            # bound.
            if not any(detect_package_type(f) == 'BGA'
                       for f in pcb.footprints.values()):
                continue
            out.append(b)
        return out

    def test_no_tracked_board_moves_a_via_at_the_shipped_defaults(self):
        reaching = self._reaching()
        if reaching is None:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        self.assertGreaterEqual(len(reaching), 4,
                                'the candidate set collapsed; the filter is '
                                'wrong and this arm proves nothing')
        moved = []
        for b in reaching:
            for clr in (0.25, 0.10):
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    r = repair_fanout_clearance(parse_kicad_pcb(b), b,
                                                clearance=clr)
                if r.get('via_moves'):
                    moved.append((os.path.basename(b), clr,
                                  len(r['via_moves'])))
        self.assertEqual(moved, [],
                         'a tracked board now relocates a via at the shipped '
                         'defaults: %r. The "inert on the corpus" claim in '
                         'the #747 PR has EXPIRED -- A/B that board on both '
                         'arms and record the numbers.' % (moved,))
    # MUTATION: none -- this arm measures the CORPUS, not the change. It is the
    # bound the PR's evidence rests on, and it expires loudly.


if __name__ == '__main__':
    unittest.main(verbosity=2)
