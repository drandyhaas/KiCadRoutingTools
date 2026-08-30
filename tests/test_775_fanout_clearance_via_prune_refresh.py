"""#775: the per-cap VIA prune has one spelling, and the post-nudge refresh
uses it instead of throwing it away.

`_Repair.__init__` prunes each cap's via list to the barrels that cap could
ever be charged against -- an EXACT reachable-disk bound anchored on the cap's
SEED pose. After a nudge, `repair_fanout_clearance` pointed every cap at the
whole board's via list instead. That is the via analogue of what #736 named
for the track channel (`_prune_segs`) and #747 named for the via BUILDER
(`_register_via`) and the via REGISTRAR beside it; the prune predicate is the
third member, and this file pins it.

IT IS A COST DEFECT, NOT A GRADE DEFECT, and this file is written the way that
demands: the arm that LICENSES the change is
`TestTheGradeIsUNCHANGEDByThePrune`, which asserts bit equality between the
pruned view and the de-pruned one. Everything else pins a mechanism that
equality would otherwise let anyone break silently.

THE MEASUREMENT, on kicad_files/orangecrab_ext_pll.kicad_pcb at --clearance
0.1 -- the one tracked board that relocates barrels (9 of them, 17 connectors,
18 cap placements), in the two in-repo configurations that reach the nudger:

    config                          cells pruned  de-pruned  re-grade
    max_displacement_cap 3.0 SHIPPED       1,914     17,680  17.7 vs 49.5ms
    max_displacement_cap 0.0 FORCING         144     17,680   2.8 vs 35.9ms

with an identical summary line in every arm:

    Moved 18 cap(s); resolved 4/14 initial violations
    (3 freed by via-nudge); 10 unresolved.

THE ISSUE'S OWN NOTE IS INVERTED and this file records that rather than
quietly using the better number. #775 worried that `max_displacement_cap=0.0`
might be too small a reach to demonstrate a saving. It is the opposite: a
smaller reach prunes harder, so 0.0 is the FLATTERING arm and the shipped 3.0
is the conservative one. The engine docstring leads with 3.0.

WHY #736's GUARD ARM HAD TO BE RE-ARMED, recorded here because the reason
belongs with #775. `TestTheRefreshIsSkippedWhenNothingMoved` kills the
mutation `if via_moves:` -> `if True:`, and its only observable was
`cap_vias[ref] is not st.vias`: an unguarded refresh DE-pruned, so the lists
became the whole-board list itself. A re-pruning refresh never produces
`st.vias`, so that assertion went vacuous -- MEASURED, not predicted: with
this change in and the guard mutated to `if True:`, test_736 passed. It is now
armed on the identity of the list `__init__` built, captured at the seed frame
through the supported `on_move` hook.

THE MUTATION BATTERY, AS RUN (`tests/mutate_775.py`): **15 rows, 13 killed,
2 survived -- both expected -- 0 broken.**

  1  revert the refresh to a de-prune ..... 7 arms
  2  delete the refresh call ............. 14 arms (775 + 736 + 746)
  3  refresh only the first cap ........... 5 arms
  4  prune from the MOVED pose ............ 2 arms
  5  drop the via_slack term .............. 1 arm  (725, the #725 invariant)
  6  drop the keep-out term ............... 1 arm  <- see below
  7  drop the span term ................... 2 arms
  8  flat clearance instead of the keep-out  1 arm  <- see below
  9  extend the existing list in place .... 8 arms
  10 fallback skips the geomless cap ...... 2 arms
  11 fallback aliases the whole list ...... 1 arm
  12 guard made unconditional ............. 1 arm  (736, the re-armed guard)
  13 init prunes from an empty source ..... 14 arms
  14 INERT: a trailing comment names the de-pruned write ... SURVIVED
  15 INERT: the refresh iterates in sorted order ........... SURVIVED

ROWS 6 AND 8 SURVIVED THE FIRST RUN, and that is why
`TestEveryTermOfTheBoundIsLoadBearing` exists. Both NARROW the reachable
disk, so they are visible only to a via sitting in the band the keep-out term
covers -- and no fixture in this family had one, on any board. The bound could
have been narrowed by half a millimetre with every arm in every file green.
The span and via_slack rows were killed on that same first run, so it was
specifically the keep-out term that was unfalsifiable. Recording them as
expected survivors would have been the wrong call: the repo's rule protects a
mutation nothing CAN catch, not one nothing HAPPENS to catch, and this one
needed a via in the band the rig can place. Re-run after the arm: both KILLED.

THE #747 BATTERY was re-run because this branch changed its target. Its
`relocate-after-the-per-cap-refresh` row is re-anchored to the new spelling
(the note recording that its expectation was corrected by ITS first run stays
verbatim) and gains this file as a fourth grader: **22 rows, 20 killed, 2
survived -- both expected -- 0 broken**, with that row now killed by 12 arms.

Runtime ~15s (measured 12.0s): THREE real-board passes on orangecrab, the
synthetic rigs, and one corpus sweep that constructs (but never runs) a
_Repair per board.
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

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import run_utils
from kicad_parser import parse_kicad_pcb
from placement import fanout_clearance as FC
from placement.fanout_clearance import _Repair, repair_fanout_clearance
from synth import make_via
from test_736_fanout_clearance_regrade_view import (  # noqa: E402
    CAP_XY, CLEAR, DECL_CLASS, DECL_LC, VIA_CLEAR, V_DRILL, V_SIZE,
    _board, _run, _stub)

BOARD = os.path.join(_ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')

# The two in-repo configurations that relocate a barrel. BOXED is the SHIPPED
# max_displacement_cap (3.0) -- the conservative arm, and the one the numbers
# lead with; FORCING pins it at 0.0, which prunes harder.
BOXED = dict(clearance=CLEAR, max_displacement=0.0, max_passes=1,
             via_clear_fallback=False)
FORCING = dict(BOXED, max_displacement_cap=0.0)

# A via far enough from every cap that no reachable-disk bound can keep it:
# the rig's board is 60x20 and its caps sit at x=10, against a reach of at most
# (3.0 + span + keep-out + slack) ~ 4.3mm.
FAR_VIA_X = 55.0
FAR_NET = 9

# _Repair's positional signature, spelled once: (clearance, grid_step,
# board_edge_clearance, near_margin, capture_radius, default_via_size,
# cap_prefix, extra_locked).
REPAIR_ARGS = (CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3)


def _repair(pcb, path, prefix='C'):
    return _Repair(pcb, path, *REPAIR_ARGS, prefix, set())


def _real(**kw):
    """Run the pass on the real board; return (result, live _Repair, stdout).

    `on_move` is the supported hook: it is handed the state at the seed frame
    and every frame is the SAME object, so what is read after the run is the
    state the pass finished with.
    """
    seen, buf = [], io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = repair_fanout_clearance(parse_kicad_pcb(BOARD), BOARD,
                                      on_move=lambda st: seen.append(st), **kw)
    return res, (seen[0] if seen else None), buf.getvalue()


class TestThePredicateIsONESpelling(unittest.TestCase):
    """SOURCE GUARD, the via twin of #736's. The whole point of #775 is that
    the predicate exists once; a second spelling is the defect again, and no
    behavioural arm can catch it, because two spellings agree on the day they
    are written and diverge later.

    CODE only -- `l.split('#')[0]` drops a trailing comment as well as a
    full-line one, because the engine's prose names every one of these symbols
    repeatedly. Offending line numbers are REPORTED rather than the source
    dumped (#732 measured a 393KB failure message from an assertIn over this
    module).
    """

    @staticmethod
    def _code(obj):
        return [l.split('#')[0] for l in inspect.getsource(obj).splitlines()]

    def _sites(self, lines, literal):
        return [i + 1 for i, l in enumerate(lines) if literal in l]

    def setUp(self):
        self.mod = self._code(FC)
        self.rep = self._code(FC._Repair)
        self.pruner = self._code(FC._Repair._prune_vias)
        self.refresh = self._code(FC._Repair.refresh_cap_vias)
        self.nudger = self._code(FC.nudge_vias_for_unresolved)

    def test_the_VIA_prune_predicate_has_exactly_one_spelling(self):
        """The via channel has no named distance helper to count -- it
        measures centre to centre with `math.hypot`, which `_Repair` calls for
        several other things -- so the literals are the predicate's own
        argument shape and the #725 slack term that only it carries."""
        for lit in ('math.hypot(v[0] - ', 'via_slack'):
            in_class = self._sites(self.rep, lit)
            in_pruner = self._sites(self.pruner, lit)
            self.assertTrue(in_class,
                            '%r is a DEAD STRING -- the predicate was '
                            'respelled and this arm proves nothing' % lit)
            self.assertEqual(
                len(in_class), len(in_pruner),
                '%r occurs %d times in _Repair but only %d inside the pruner, '
                'so there is a second spelling of the via prune'
                % (lit, len(in_class), len(in_pruner)))
        calls = self._sites(self.mod, 'self._prune_vias(')
        self.assertEqual(len(calls), 2,
                         'expected exactly two callers of the via prune -- '
                         '__init__ and the refresh; found %d at module-'
                         'relative line(s) %s' % (len(calls), calls))
    # MUTATION: re-inline the prune loop in either caller.

    def test_cap_vias_is_ASSIGNED_and_never_mutated_in_place(self):
        asg = re.compile(r'self\.cap_vias\[[^\]]+\]\s*=[^=]')
        writes = [i + 1 for i, l in enumerate(self.mod) if asg.search(l)]
        self.assertEqual(len(writes), 2,
                         'expected two cap_vias assignments (__init__ and the '
                         'refresh); found %d at %s' % (len(writes), writes))
        inplace = re.compile(r'cap_vias\[[^\]]*\]\s*(?:\.append|\.extend|\+=)')
        bad = [i + 1 for i, l in enumerate(self.mod) if inplace.search(l)]
        self.assertEqual(bad, [],
                         'cap_vias is mutated IN PLACE at line(s) %s -- '
                         '_via_effs revalidates its memo on the list IDENTITY, '
                         'so the rows are never rebuilt and the shortfall loop '
                         'indexes past the end of its row' % bad)
    # MUTATION: `self.cap_vias[ref] += ...` in the refresh.

    def test_the_refresh_has_one_caller_and_is_not_the_nudger_s(self):
        """`nudge_vias_for_unresolved` is duck-typed on `st` -- nine test
        files drive it with a stand-in, one of which carries no via list at
        all -- so every read there goes through getattr with a flat fallback.
        (Eleven files call it; the count is approximate, and it is not what
        this arm asserts.)
        A RESOLVER has an honest flat fallback; a MUTATION does not, because
        "silently do nothing" is precisely the staleness this fixes."""
        calls = self._sites(self.mod, 'refresh_cap_vias()')
        self.assertEqual(len(calls), 1,
                         'the refresh is called %d times (line(s) %s); it has '
                         'one caller by design' % (len(calls), calls))
        self.assertEqual([], self._sites(self.nudger, 'refresh_cap_vias'),
                         'the nudger REPORTS; a real _Repair applies')
    # MUTATION: call it from the nudger; call it twice.

    def test_the_refresh_returns_a_count(self):
        """A returned count is what makes "refresh only the first cap"
        killable structurally as well as behaviourally."""
        self.assertEqual(
            str(inspect.signature(FC._Repair.refresh_cap_vias)),
            "(self) -> 'int'")
        self.assertTrue(any(l.strip() == 'return n' for l in self.refresh))


class TestBothCallSitesAgree(unittest.TestCase):
    """Construction and the refresh must produce the SAME lists, or the
    extraction bought nothing. Driven on the real board so the positive
    control is not vacuous: a two-via fixture satisfies this trivially."""

    @classmethod
    def setUpClass(cls):
        cls.st = _repair(parse_kicad_pcb(BOARD), BOARD, 'C,R,FB')

    def test_construction_agrees_with_the_pruner_for_every_cap(self):
        st = self.st
        for ref, cap in st.caps.items():
            want = st._prune_vias(cap, st._cap_geom[ref], st.vias)
            self.assertEqual(st.cap_vias[ref], want, ref)
            self.assertIsNot(st.cap_vias[ref], want,
                             'the pruner must return a NEW list -- _via_effs '
                             'memoises on identity')

    def test_the_prune_is_not_vacuous_on_this_board(self):
        """POSITIVE CONTROL. Without it, the equalities above hold on a board
        where the prune keeps everything, and this class measures the prune
        existing rather than the prune working."""
        st = self.st
        self.assertGreater(len(st.vias), 100, 'the fixture board changed')
        kept_all = sorted(r for r in st.caps
                          if len(st.cap_vias[r]) >= len(st.vias))
        self.assertEqual(kept_all, [],
                         'cap(s) %s keep the WHOLE board via list, so the '
                         'prune is inert for them' % kept_all)


class TestTheRefreshRePrunesInsteadOfDePruning(unittest.TestCase):
    """THE HEADLINE. After a nudging run every cap holds a PRUNED list, not
    the whole-board one -- and a via no cap can reach is on the board and in
    nobody's view."""

    @classmethod
    def setUpClass(cls):
        with _stub() as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu', second_cap=True)
            pcb.vias.append(make_via(FAR_VIA_X, CAP_XY[1], net_id=FAR_NET,
                                     size=V_SIZE, drill=V_DRILL))
            cls.res, cls.st, cls.out, cls.pre = _run(pcb, path)

    def test_the_run_actually_moved_a_via(self):
        self.assertTrue(self.res['via_moves'],
                        'no via moved, so the refresh never ran and every arm '
                        'in this class is vacuous:\n' + self.out)

    def test_no_cap_holds_the_whole_board_list(self):
        st = self.st
        for ref in st.caps:
            self.assertIsNot(st.cap_vias[ref], st.vias,
                             'cap %s was DE-pruned onto the whole-board list '
                             '-- that is #775 itself' % ref)

    def test_the_unreachable_via_is_on_the_board_and_in_no_caps_view(self):
        st = self.st
        far = [t for t in st.vias if abs(t[0] - FAR_VIA_X) < 1e-6]
        self.assertEqual(len(far), 1, 'the far via left the graded list')
        for ref in st.caps:
            self.assertNotIn(far[0], st.cap_vias[ref],
                             'cap %s can be charged against a barrel 45mm '
                             'away' % ref)

    def test_the_refresh_actually_REBUILT_every_view(self):
        """#736's `_run` snapshots the per-cap lists at the seed frame, so
        this class can say the refresh RAN rather than only that the lists
        look pruned -- which they would if it had never been called at
        all."""
        st = self.st
        self.assertTrue(self.pre, 'the seed frame never fired')
        for ref in st.caps:
            self.assertIsNot(st.cap_vias[ref], self.pre[ref],
                             'cap %s kept its construction-time list '
                             'through a run that moved a via' % ref)

    def test_at_least_one_view_is_strictly_shorter(self):
        st = self.st
        self.assertTrue(
            any(len(st.cap_vias[r]) < len(st.vias) for r in st.caps),
            'every per-cap list is as long as the board list, so this class '
            'cannot tell a prune from a de-prune')
    # MUTATION: revert the refresh to `{r: st.vias for r in st.caps}`.


class TestTheRefreshSeesTheMOVEDBarrel(unittest.TestCase):
    """Why the refresh may not SKIP a cap the way the track registrar may.

    That one leaves a cap it has no seed pose for exactly as the caller built
    it, because every tuple already filed still describes real copper at the
    same coordinates. Here the registrar SUBSTITUTED the tuples it moved, so a
    skipped cap keeps the PRE-move tuple: a via at the landing that is not
    there, and a hole where the barrel really is.
    """

    @classmethod
    def setUpClass(cls):
        cls.res, cls.st, cls.out = _real(**BOXED)

    def test_the_run_relocated_barrels(self):
        self.assertEqual(len(self.res['via_moves']), 9,
                         'the real-board rig stopped relocating barrels, so '
                         'every arm here is vacuous')

    def test_every_tuple_in_every_view_is_a_LIVE_board_tuple(self):
        st = self.st
        live = {id(t) for t in st.vias}
        stale = sorted({r for r in st.caps
                        if any(id(t) not in live for t in st.cap_vias[r])})
        self.assertEqual(stale, [],
                         'cap(s) %s hold a PRE-move via tuple -- a phantom '
                         'barrel at the landing and a hole where the real one '
                         'is' % stale)
    # MUTATION: refresh BEFORE the registrar; skip a cap in the refresh.


class TestThePruneIsAnchoredOnTheSEEDPose(unittest.TestCase):
    """The bound is the reachable-DISK argument: a cap moves at most
    max_displacement_cap from its SEED, so the radius is exact only measured
    from there. A MOVED pose silently redefines what "exact" means -- and
    required_rows grades the via shortfalls at the seed pose too, so a
    moved-pose radius is a superset for neither."""

    def test_a_via_inside_the_SEED_reach_survives_a_walked_cap(self):
        with _stub() as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu')
            st = _repair(pcb, path)
            ref = 'C1'
            cap = st.caps[ref]
            ccx, _ccy, span, _r = st._cap_geom[ref]
            via = st.vias[0]
            reach = st._max_disp_cap + span + via[3]
            self.assertLess(abs(via[0] - ccx), reach - 0.05,
                            'the fixture via is not inside the seed reach')
            self.assertIn(via, st.cap_vias[ref])
            # walk the cap AWAY from the via, far enough that a moved-pose
            # radius would drop it by more than the 0.05 this file allows
            cap.x = ccx - (reach - abs(via[0] - ccx)) - 0.05
            n = st.refresh_cap_vias()
            self.assertEqual(n, len(st.caps))
            self.assertIn(via, st.cap_vias[ref],
                          'the via left the view when the cap walked -- the '
                          'prune is anchored on the MOVED pose')
    # MUTATION: build geom from `cap.rect()` inside the refresh.


class TestEveryTermOfTheBoundIsLoadBearing(unittest.TestCase):
    """The predicate is `hypot(v - seed) <= max_disp + span + v[3] + slack`,
    and each term has to be REACHABLE by a fixture or it is unfalsifiable.

    THIS CLASS EXISTS BECAUSE THE BATTERY SAID SO. Its first run killed the
    span and via_slack rows and left two SURVIVORS: dropping `v[3]` (the via's
    radius plus its own keep-out) and swapping it for the flat scalar. Both
    NARROW the disk, so they can only be seen by a via that sits in the band
    the term covers -- and no fixture in this family had one, on any board.
    The bound could have been narrowed by half a millimetre with every arm
    green. Rather than record that as an expected survivor, the band gets a
    via.

    The geometry is COMPUTED from the live objects rather than hardcoded, so
    the arm follows the rig instead of silently drifting out of the band the
    day a pad size or the default displacement cap changes. Measured on this
    rig: max_disp 3.0000 + span 0.8544 opens the band at 3.8544, and v[3]
    (0.4 radius + 0.1 keep-out) closes it at 4.3544. The flat-scalar mutation
    moves the edge to 3.9544, so a probe in the middle of the band separates
    all three spellings.
    """

    def _probe(self, offset_frac):
        """Place one via at `max_disp + span + offset_frac * v[3]` from C1's
        seed centre and report whether the prune kept it."""
        with _stub() as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu')
            st = _repair(pcb, path)
            ccx, ccy, span, _r = st._cap_geom['C1']
            v3 = st.vias[0][3]
            d = st._max_disp_cap + span + offset_frac * v3
            pcb.vias.append(make_via(ccx + d, ccy, net_id=FAR_NET,
                                     size=V_SIZE, drill=V_DRILL))
            st2 = _repair(pcb, path)
            probe = [t for t in st2.vias if abs(t[0] - (ccx + d)) < 1e-9]
            self.assertEqual(len(probe), 1, 'the probe via was not graded')
            return probe[0] in st2.cap_vias['C1'], d, v3

    def test_a_via_inside_the_KEEPOUT_band_is_KEPT(self):
        kept, d, v3 = self._probe(0.5)
        self.assertTrue(kept,
                        'a via %.4fmm from the seed centre was pruned away, '
                        'but it sits INSIDE the reach by half of v[3] '
                        '(%.4fmm). The keep-out term was dropped or replaced '
                        'by a smaller one, and the prune is no longer exact.'
                        % (d, v3))
    # MUTATION: `+ v[3]` -> `+ 0.0`; `v[3]` -> `self.clearance`.

    def test_a_via_BEYOND_the_full_reach_is_dropped(self):
        """The other side of the bracket. Without it the arm above is
        satisfied by a prune that keeps everything, which is the de-prune this
        whole branch removes."""
        kept, d, _v3 = self._probe(1.5)
        self.assertFalse(kept,
                         'a via %.4fmm away is kept, which is beyond the '
                         'reachable disk -- the bound was widened' % d)
    # MUTATION: drop the distance test; widen any term.


class TestTheGeomlessCapFallsBackToTheWholeList(unittest.TestCase):
    """A cap this object never pruned for has no seed pose to measure a reach
    from. The track registrar may `continue` past it; the refresh may not (see
    TestTheRefreshSeesTheMOVEDBarrel), so it over-blocks instead."""

    def test_a_cap_with_no_seed_geometry_gets_a_COPY_of_the_whole_list(self):
        with _stub() as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu')
            st = _repair(pcb, path)
            st.caps['ZZ'] = st.caps['C1']
            n = st.refresh_cap_vias()
            self.assertEqual(n, len(st.caps), 'the geomless cap was skipped')
            self.assertEqual(st.cap_vias['ZZ'], st.vias)
            self.assertIsNot(st.cap_vias['ZZ'], st.vias,
                             'the fallback ALIASED the graded list; every '
                             'value in this view must be a list the refresh '
                             'made, with no exception a caller has to know '
                             'about')
    # MUTATION: `continue` for a geomless cap; alias `self.vias`.

    def test_a_geomless_cap_still_sees_the_MOVED_tuple(self):
        """The fallback is not merely tidy -- it is what stops the skip from
        shipping a phantom."""
        with _stub() as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu')
            st = _repair(pcb, path)
            old = st.vias[0]
            del st._cap_geom['C1']
            st.relocate_vias([(old[0], old[1],
                               {'net_id': old[2], 'x': old[0] + 0.5,
                                'y': old[1]})])
            st.refresh_cap_vias()
            self.assertNotIn(old, st.cap_vias['C1'],
                             'the geomless cap kept the PRE-move tuple')
            self.assertIn(st.vias[0], st.cap_vias['C1'])


class TestTheGradeIsUNCHANGEDByThePrune(unittest.TestCase):
    """THE ARM THAT LICENSES #775, and the reason it is a cost change rather
    than a behaviour change.

    Bit equality, not `assertAlmostEqual`, and it is guaranteed rather than
    lucky for two reasons worth stating: a pruned-away via is outside the pair
    keep-out by the reachable-disk bound, so it contributes NO addend at all
    rather than a small one; and the kept vias remain an order-preserving
    subsequence, so the floating-point summation order is unchanged too.
    `required_rows` is invariant for a third reason -- it filters on the nets
    actually charged and prices each by a PER-NET floor, so adding or removing
    vias of an already-charged net cannot move the reported mm.
    """

    def _both_ways(self, kw):
        res, st, out = _real(**kw)
        self.assertTrue(res['via_moves'],
                        'nothing moved, so the refresh never ran:\n' + out)
        caps = list(st.caps.items())
        gp = {r: st.graze_penalty(r, c, c.x, c.y, c.rot) for r, c in caps}
        sf = {r: st._via_shortfalls(r, c, c.x, c.y, c.rot) for r, c in caps}
        with contextlib.redirect_stdout(io.StringIO()):
            rows = st.required_rows()
        lens = {r: len(st.cap_vias[r]) for r in st.caps}
        # ...and now grade the SAME poses through the de-pruned view
        st.cap_vias = {r: st.vias for r in st.caps}
        st._cap_via_eff.clear()
        gp2 = {r: st.graze_penalty(r, c, c.x, c.y, c.rot) for r, c in caps}
        sf2 = {r: st._via_shortfalls(r, c, c.x, c.y, c.rot) for r, c in caps}
        with contextlib.redirect_stdout(io.StringIO()):
            rows2 = st.required_rows()
        return (gp, gp2), (sf, sf2), (rows, rows2), lens, len(st.vias)

    def _check(self, kw, label):
        (gp, gp2), (sf, sf2), (rows, rows2), lens, nv = self._both_ways(kw)
        nz = sum(1 for v in gp.values() if v)
        self.assertGreater(nz, 0,
                           '%s: EVERY cap grades zero, so the equalities below '
                           'are equalities between zeros' % label)
        self.assertTrue(any(n < nv for n in lens.values()),
                        '%s: the pruned view IS the whole board, so nothing '
                        'is being compared' % label)
        self.assertEqual(gp, gp2,
                         '%s: graze_penalty MOVED when the view was de-pruned '
                         '-- the prune is not exact' % label)
        self.assertEqual(sf, sf2, '%s: the via shortfalls moved' % label)
        # NOT paired with a positive control, and that is now DELIBERATE
        # rather than an oversight: on this board required_rows returns []
        # on BOTH sides, because every cap's max_floor is 0.0 so best()
        # emits nothing. So this line is a CHANGE DETECTOR here, not a
        # measurement, and the arm that actually measures the disclosure
        # is test_the_DISCLOSURE_is_identical_on_a_declaring_board below.
        # An adversarial review of this branch caught the class docstring
        # presenting it as tested when it was [] == [].
        self.assertEqual(rows, rows2, '%s: the disclosure moved' % label)
        self.assertEqual(rows, [],
                         '%s: required_rows is no longer empty on this\n'
                         'board -- the note above is stale, and this arm\n'
                         'is now a real measurement rather than a change\n'
                         'detector' % label)

    def test_the_grade_is_identical_at_the_SHIPPED_displacement_cap(self):
        self._check(BOXED, 'BOXED')

    def test_the_grade_is_identical_at_the_forcing_config(self):
        self._check(FORCING, 'FORCING')
    # MUTATION: drop the keep-out, the span or the via_slack term.

    def test_the_DISCLOSURE_is_identical_on_a_declaring_board(self):
        """The third channel, MEASURED rather than argued.

        required_rows is invariant under the prune for a reason of its
        own: it filters on the nets actually charged and prices each by a
        PER-NET floor, so adding or removing vias of an already-charged
        net cannot move the reported mm. The real board cannot show that
        -- every one of its caps has max_floor 0.0, so the report is empty
        on both sides and the comparison is between two empty lists.

        This uses #736's DECLARING rig instead: a netclass plus a pad
        override gives C1 a floor of 0.5, which puts a REAL `via` row in
        the report -- and a far via keeps the pruned view strictly
        smaller, so both halves of the claim are exercised at once.
        """
        with _stub(DECL_CLASS) as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu', second_cap=True,
                             lc=DECL_LC)
            pcb.vias.append(make_via(FAR_VIA_X, CAP_XY[1],
                                     net_id=FAR_NET, size=V_SIZE,
                                     drill=V_DRILL))
            st = _repair(pcb, path)
            with contextlib.redirect_stdout(io.StringIO()):
                rows = st.required_rows()
            self.assertTrue(
                any(str(r[1]).startswith('via') for r in rows),
                'the declaring rig stopped emitting a via row, so this arm '
                'no longer measures the disclosure: %r' % (rows,))
            self.assertTrue(
                any(len(st.cap_vias[r]) < len(st.vias) for r in st.caps),
                'the pruned view IS the whole board, so nothing is being '
                'compared')
            st.cap_vias = {r: st.vias for r in st.caps}
            st._cap_via_eff.clear()
            with contextlib.redirect_stdout(io.StringIO()):
                rows2 = st.required_rows()
            self.assertEqual(rows, rows2,
                             'the disclosure MOVED when the via view was '
                             'de-pruned -- the prune is not exact')
    # MUTATION: drop any term of the predicate. NOT the refresh: this arm
    # builds a _Repair directly and never runs the pass, so it is provably
    # insensitive to a de-pruned refresh -- an adversarial review caught
    # the note claiming otherwise, which is the kind of over-claim that
    # makes a battery's expectations unreadable.


class TestTheExactnessMarginStaysPositive(unittest.TestCase):
    """The via bound is an ORDER OF MAGNITUDE thinner than the track one, and
    #775 makes it load-bearing in a second place, so the number is asserted
    rather than merely written down.

    Residual slack is `span - grid_overshoot - R_max`, where the overshoot
    is real and pre-existing (`_candidate_positions` snaps AFTER its radius
    test, so a final pose can sit up to `grid_step*sqrt(2)/2` past
    max_displacement_cap). The track pruner records +1.009mm for the same
    quantity, because its bound carries `2*span + clearance` where this one
    carries `span` alone.

    R_MAX IS MEASURED FROM THE PRUNE'S OWN ANCHOR, and over the poses that
    are actually graded. Both of those were wrong in the first version of
    this arm, and an adversarial review caught them: it measured from
    `cap.seed_x/seed_y`, the footprint ORIGIN, where the disk is centred on
    `_cap_geom`'s rect CENTRE; and it swept `(0, 90, 180, 270)` where the
    graded set is those plus the cap's OWN seed rotation -- which is not
    among them for a footprint placed at a non-orthogonal angle. Both
    corrections are INERT on today's corpus (every cap's rect centre
    coincides with its origin, and no cap has a non-orthogonal seed
    rotation), so the numbers below are unchanged. An arm that exists to
    catch the board which has not shipped yet should defend the quantity
    the bound actually needs.

    MEASURED, and printed by the arm below so a shift is visible rather
    than merely non-negative: over the 115 caps of the 5 tracked boards
    that offer one, the minimum is +0.1267mm (orangecrab_ext_pll R14,
    span 0.7826, R_max 0.5852) and the maximum +0.2716mm.
    """

    def test_no_cap_of_any_tracked_board_has_a_negative_margin(self):
        boards = run_utils.corpus_boards()
        if not boards:
            print('SKIP: git cannot identify the tracked corpus')
            self.skipTest('no git')
        worst, best, n_caps, n_boards = None, None, 0, 0
        for b in boards:
            try:
                st = _repair(parse_kicad_pcb(b), b, 'C,R,FB')
            except Exception:                                    # noqa: BLE001
                continue
            if not st.caps:
                continue
            n_boards += 1
            overshoot = st.grid_step * math.sqrt(2.0) / 2.0
            for ref, cap in st.caps.items():
                ccx, ccy, span, _r = st._cap_geom[ref]
                rmax = 0.0
                # the prune's anchor is the rect CENTRE, and the graded
                # poses are the four reachable rotations PLUS the cap's
                # own seed one
                for rot in (0.0, 90.0, 180.0, 270.0, cap.seed_rot):
                    for r in cap.pad_rects(cap.seed_x, cap.seed_y, rot):
                        x1, y1, x2, y2 = r[0], r[1], r[2], r[3]
                        for px, py in ((x1, y1), (x1, y2),
                                       (x2, y1), (x2, y2)):
                            rmax = max(rmax,
                                       math.hypot(px - ccx, py - ccy))
                n_caps += 1
                slack = span - overshoot - rmax
                if worst is None or slack < worst[0]:
                    worst = (slack, os.path.basename(b), ref, span, rmax)
                # the MAXIMUM is reported as well as the minimum: two
                # docstrings quote it and no arm was printing it, and a
                # number nothing reports is a number that drifts silently
                # (a fact-check of this branch made exactly that point)
                if best is None or slack > best:
                    best = slack
        self.assertGreater(n_caps, 50,
                           'only %d caps over %d boards -- the corpus arm '
                           'collapsed and proves nothing' % (n_caps, n_boards))
        self.assertIsNotNone(worst)
        print('#775 exactness margin: min %+.4fmm (%s %s, span %.4f, '
              'R_max %.4f), max %+.4fmm, over %d caps of %d boards'
              % (worst[0], worst[1], worst[2], worst[3], worst[4],
                 best, n_caps, n_boards))
        self.assertGreater(
            worst[0], 0.0,
            'the via prune bound went NEGATIVE for %s on %s (span %.4f, '
            'R_max %.4f). The prune is no longer exact, and since #775 it is '
            'applied at the post-nudge refresh as well as at construction -- '
            'so this needs re-measuring, not relaxing.'
            % (worst[2], worst[1], worst[3], worst[4]))


class TestTheSiblingGatesAreRegistered(unittest.TestCase):
    """A claim this file cannot make itself must be made SOMEWHERE."""

    def test_the_mutation_battery_exists(self):
        bat = os.path.join(_TESTS, 'mutate_775.py')
        self.assertTrue(os.path.isfile(bat),
                        'the battery is not optional -- the source guards '
                        'above are only as good as the mutations actually run '
                        'against them')

    def test_the_inertness_bound_lives_with_the_via_registrar(self):
        """#775 is unreachable at the shipped defaults for the same reason
        #747 is, and that bound is asserted once, in test_747, rather than
        copied here where the two would rot apart."""
        sib = os.path.join(_TESTS,
                           'test_747_fanout_clearance_via_registrar.py')
        with io.open(sib, encoding='utf-8') as f:
            src = f.read()
        self.assertIn(
            'test_no_tracked_board_moves_a_via_at_the_shipped_defaults', src,
            'the shared inertness bound moved or was deleted')


if __name__ == '__main__':
    unittest.main(verbosity=2)
