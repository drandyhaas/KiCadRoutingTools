"""#779: `required_rows`' SEED half must grade seed BARREL positions.

`required_rows` grades each cap at the SEED pose as well as the final one, so
that a pass which SUCCEEDS still discloses the raised requirement that did the
work. The TRACK kind scopes its seed half with `upto`; the VIA kind passed no
scope at all, on a comment that said only the track kind needs one.

That stopped being true at #747, which made `relocate_vias` SUBSTITUTE a moved
barrel at its new coordinates. So the seed half charged the cap's SEED pose
against POST-NUDGE via positions -- a configuration no board ever had.

IT HID REAL GRAZES AS WELL AS INVENTING ONES, which is the opposite of what
#736's track defect did and the reason this file leads with C19. A barrel that
moved AWAY from a cap reads clear where it now sits, so the seed-era graze
vanishes from the report. Measured on kicad_files/orangecrab_ext_pll.kicad_pcb
at the one in-repo configuration that relocates barrels (9 moves), seed half
before -> after the fix:

    C19   {}                                 -> {19: 0.2718}     <- reported NOTHING
    C44   {6: 0.0230}                        -> {6: 0.3230}      <- 14x understated
    C45   {29: 0.0209}                       -> {29: 0.2750}
    C53   {128: 0.2000}                      -> {127: 0.0438, 128: 0.2000}
    C7    {71: 0.0945, 154: 0.2370}          -> + {153: 0.0140}
    C59   {39: 0.0472, 43: 0.1472, 78: 0.25} -> {39: 0.1740, ...}
    R19   {23: 0.2700}                       -> + {35: 0.2200, 39: 0.2700}

7 of 65 caps changed; 43 of 65 hold a relocated barrel.

WHY THE FIX IS A COORDINATE SWAP AND NOT A FILTER. `_via_effs`' rows are
index-aligned with `cap_vias[ref]`, so dropping or reordering entries would
mis-index them; substituting coordinates at the same indices cannot. It is also
why `upto` has no via analogue: `register_new_segments` APPENDS, so its
seed-era entries are a prefix, while a substituted via changes no length.

NOTHING IS DISCLOSED ON THE TRACKED CORPUS TODAY, and this file says so rather
than implying a shipped bad report: `required_rows` returns [] on orangecrab
because every one of its caps has `max_floor == 0.0`, so `best()` emits
nothing. The wrong number was computed and then discarded. The synthetic
declaring rig below is what makes the report non-empty.

THE MUTATION BATTERY, AS RUN (`tests/mutate_779.py`): **11 rows, 8 killed,
3 survived -- all three expected -- 0 broken.**

  1  revert the seed-position lookup .......... 3 arms
  2  record nothing at all .................... 6 arms
  3  the via kind stops asking for it ......... 2 arms  <- was 1, a GREP
  4  drop the swap in the EFF loop ............ 3 arms
  5  drop the swap in the FLAT loop ........... SURVIVED, expected -- below
  6  a second hop overwrites the original seed  1 arm
  7  the map stops holding its tuple .......... 2 arms
  8  seed_pos defaults ON ..................... 1 arm
  9  record the LANDING instead of the seed ... 5 arms

ROW 3 IS THE ONE TO READ. Before TestTheDisclosureCHANGESEndToEnd existed it
was killed by a single arm, and that arm was a SUBSTRING MATCH OVER SOURCE --
so the only thing making this fix reach production was a grep. Five rows
gained a killer when the behavioural arm landed; this is the row where it
mattered.
  10 INERT: a trailing comment names the kwarg ........ SURVIVED
  11 INERT: the resolve is a generator, not a list .... SURVIVED

ROW 5 IS A RECORDED HOLE, not a pass. The FLAT loop runs only for a cap
offering neither floors nor layers -- the duck-typed path -- and orangecrab
resolves an eff matrix for every one of its caps, so no arm in this family
reaches it. The row is kept with its reason rather than deleted, because an
inert row recorded is a finding and an inert row removed is a hole. If it ever
flips to KILLED, that is the news.

ROW 8 IS KILLED BY EXACTLY ONE ARM, and it is the signature arm rather than a
behavioural one -- test_775 and test_725 are both indifferent to the default
flipping, because neither drives a run that relocates a barrel. That is worth
knowing before someone deletes the signature arm as redundant.

Runtime ~25s: two real-board passes plus the declaring rig.
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

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import fanout_clearance as FC  # noqa: E402
from placement.fanout_clearance import _Repair  # noqa: E402
from test_736_fanout_clearance_regrade_view import (  # noqa: E402
    CAP_XY, CLEAR, DECL_CLASS, DECL_LC, VIA_CLEAR, VIA_DEEP, V_DRILL,
    V_SIZE, _board, _stub)

BOARD = os.path.join(_ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
# the one tracked configuration that relocates barrels
BOXED = dict(clearance=CLEAR, max_displacement=0.0, max_passes=1,
             via_clear_fallback=False)
N_MOVES = 9


def _real():
    seen, buf = [], io.StringIO()
    with contextlib.redirect_stdout(buf):
        res = FC.repair_fanout_clearance(
            parse_kicad_pcb(BOARD), BOARD,
            on_move=lambda st: seen.append(st), **BOXED)
    return res, seen[0], buf.getvalue()


class TestTheSeedPositionsAreRecorded(unittest.TestCase):
    """One entry per relocation, holding its tuple."""

    @classmethod
    def setUpClass(cls):
        cls.res, cls.st, cls.out = _real()

    def test_the_rig_still_relocates(self):
        self.assertEqual(len(self.res['via_moves']), N_MOVES,
                         'the real-board rig stopped relocating barrels, so '
                         'every arm in this file is vacuous')

    def test_one_entry_per_TUPLE_REPLACED(self):
        """Not per move HANDED IN, and relocate_vias' own docstring
        draws the distinction: its return counts tuples replaced, and a
        move can match nothing (several test files inject a via after
        construction) or -- in principle -- two coincident same-net
        tuples. They coincide on this board; asserting the engine's own
        count rather than the move list is what keeps that an assumption
        the arm states instead of one it hides.
        """
        self.assertEqual(len(self.st._via_seed_xy), N_MOVES,
                         'entries != moves on this board, so the 1:1 '
                         'assumption below has broken')

    def test_each_entry_holds_its_tuple(self):
        """A bare id can be recycled by the very next tuple the builder
        makes, and the entry would then describe someone else's barrel --
        the hazard the radius map's own note already records.

        DELIBERATELY NOT asserting that every key is still live. A barrel
        relocated TWICE leaves the intermediate entry behind on purpose,
        exactly as _via_radius_by_id does -- 'a fact about a nudged board
        rather than a leak', in that map's own words. An earlier draft of
        this arm required liveness and would have failed the engine for
        behaving correctly the first time a board double-hopped; it passed
        only because orangecrab happens not to.
        """
        for key, rec in self.st._via_seed_xy.items():
            self.assertEqual(len(rec), 3)
            self.assertEqual(id(rec[0]), key,
                             'the entry does not hold the tuple it is keyed '
                             'on, so its id can be recycled')
    # MUTATION: store (sx, sy) without the tuple.

    def test_the_recorded_position_is_where_the_barrel_WAS(self):
        """A SUBSET, not an equality. Under a two-hop the second
        entry's seed is the ORIGINAL, so the first hop's `old` never
        appears as a recorded position -- correct behaviour that an
        equality would report as 'the map holds landing positions', which
        is backwards. Equality holds on this board and the arm below says
        so separately, so nothing is lost by stating the weaker relation
        that is actually invariant.
        """
        st, moved = self.st, self.res['via_moves']
        olds = {(round(ox, 6), round(oy, 6)) for ox, oy, _s in moved}
        lands = {(round(s['x'], 6), round(s['y'], 6))
                 for _ox, _oy, s in moved}
        got = {(round(r[1], 6), round(r[2], 6))
               for r in st._via_seed_xy.values()}
        self.assertTrue(got <= olds,
                        'the map holds a position no move started from: '
                        '%s' % sorted(got - olds))
        self.assertFalse(got & (lands - olds),
                         'the map holds a LANDING position: %s'
                         % sorted(got & (lands - olds)))
        self.assertEqual(got, olds,
                         'no barrel double-hopped on this board, so the '
                         'subset above should be an equality here')


class TestTheSeedHalfUsesThem(unittest.TestCase):
    """THE HEADLINE. The seed half must charge each barrel where it sat."""

    @classmethod
    def setUpClass(cls):
        cls.res, cls.st, cls.out = _real()

    def _seed(self, seed_pos):
        st = self.st
        return {r: st._via_shortfalls(r, c, c.seed_x, c.seed_y, c.seed_rot,
                                      seed_pos=seed_pos)
                for r, c in st.caps.items()}

    def test_the_seed_half_CHANGES_when_it_uses_seed_positions(self):
        """If this ever reports 0 caps, the fix has become inert on the only
        board that can exercise it and the file is measuring nothing."""
        old, new = self._seed(False), self._seed(True)
        diff = sorted(r for r in old if old[r] != new[r])
        self.assertTrue(diff,
                        'no cap grades differently, so either no barrel moved '
                        'into or out of a cap reach, or seed_pos is not wired')
        self.assertGreaterEqual(len(diff), 5, diff)

    def test_a_MISSED_seed_graze_is_now_reported(self):
        """C19 is the case that makes this a hidden-defect fix rather than a
        phantom-removal one: the post-nudge view reported NOTHING."""
        old, new = self._seed(False), self._seed(True)
        missed = [r for r in old if not old[r] and new[r]]
        self.assertTrue(missed,
                        'no cap went from "nothing charged" to a real seed '
                        'graze -- the strongest half of #779 is untested here')
    # MUTATION: drop the `at[j]` substitution in either loop.

    def test_the_kwarg_reaches_the_SEED_call_ONLY(self):
        """`both()` must pass seed_kw to the seed call and not to the final
        one, or the final half would be graded at seed barrel positions too --
        the mirror-image defect.

        A source guard because the behavioural version is not available: the
        two calls differ only in the pose they are given, so no fixture can
        distinguish "the kwarg went to both" from "the poses coincide"."""
        src = inspect.getsource(FC._Repair.required_rows)
        self.assertIn('out = set(fn(ref, cap, sx, sy, srot, '
                      '**(seed_kw or {})))', src,
                      'the SEED call no longer takes seed_kw')
        self.assertIn('out |= set(fn(ref, cap, x, y, rot))', src,
                      'the FINAL call takes a kwarg; it must be the bare pose')

    def test_seed_pos_defaults_OFF(self):
        """Every other caller -- cost(), hard_blocked, the accept gate --
        grades live copper and must keep seeing current positions."""
        sig = inspect.signature(FC._Repair._via_shortfalls)
        self.assertIs(sig.parameters['seed_pos'].default, False)

    def test_required_rows_asks_for_it(self):
        """A source guard: the wiring is one keyword, and losing it is silent
        -- the seed half would simply grade the wrong barrels again."""
        src = inspect.getsource(FC._Repair.required_rows)
        self.assertIn("both(self._via_shortfalls, {'seed_pos': True})", src)
    # MUTATION: revert the via kind to `both(self._via_shortfalls)`.


class TestTheDisclosureOnADeclaringBoard(unittest.TestCase):
    """`required_rows` is EMPTY on the tracked corpus (no cap declares a
    floor), so the wrong number was computed and discarded. This rig makes the
    report non-empty, which is what gives the fix an observable end to end."""

    def test_the_declaring_rig_emits_a_via_row(self):
        with _stub(DECL_CLASS) as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu', second_cap=True, lc=DECL_LC)
            st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C',
                         set())
            with contextlib.redirect_stdout(io.StringIO()):
                rows = st.required_rows()
        self.assertTrue(any(str(r[1]).startswith('via') for r in rows),
                        'the rig stopped emitting a via row: %r' % (rows,))

    def test_the_real_board_discloses_nothing_and_that_is_why(self):
        """Pins the reachability claim the module docstring makes, so it
        cannot rot into folklore: the numbers are wrong AND unprinted."""
        _res, st, _out = _real()
        self.assertEqual([c for c in st.caps.values() if c.max_floor], [],
                         'a cap now declares a floor -- the "computed and '
                         'discarded" claim has expired and #779 is printing')
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(st.required_rows(), [])


class TestTheDisclosureCHANGESEndToEnd(unittest.TestCase):
    """THE BEHAVIOURAL ARM, and the one this file shipped without.

    The first draft asserted the wiring with a source grep -- `assertIn(
    "both(self._via_shortfalls, {'seed_pos': True})", src)` -- and its
    header claimed no end-to-end arm was available, because the tracked
    board discloses nothing and the declaring rig relocates nothing. An
    adversarial review showed that was false: the two halves compose. Take
    the DECLARING rig, relocate its barrel by hand exactly as the double-hop
    arm below already does, and `required_rows` itself moves.

    THE SHIFT IS LOAD-BEARING and was measured rather than picked. At 0.60mm
    the barrel still grazes C1 at its LANDING, so the pre-fix grading reports
    the same row and the arm cannot discriminate. At 1.00mm it clears the
    landing but not the seed, which is exactly the case #779 is about: the
    seed-era graze is real and the post-nudge view cannot see it.
    """

    SHIFT = 1.0

    def _rows(self, clear_map):
        with _stub(DECL_CLASS) as path:
            pcb, _v = _board(VIA_DEEP, 'F.Cu', second_cap=True, lc=DECL_LC)
            st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C',
                         set())
            t = st.vias[0]
            st.relocate_vias([(t[0], t[1], {'net_id': t[2],
                                            'x': t[0] + self.SHIFT,
                                            'y': t[1]})])
            st.refresh_cap_vias()
            if clear_map:
                # a faithful simulation of the pre-#779 engine: without the
                # map the swap is skipped and the seed half grades the
                # post-nudge coordinates, which is precisely the old code
                st._via_seed_xy.clear()
            with contextlib.redirect_stdout(io.StringIO()):
                return st.required_rows()

    def test_the_seed_era_graze_is_DISCLOSED(self):
        rows = self._rows(clear_map=False)
        self.assertTrue(any(str(r[1]).startswith('via') for r in rows),
                        'required_rows reports no via row for a barrel that '
                        'grazed at the seed: %r' % (rows,))

    def test_and_the_PRE_FIX_grading_would_have_missed_it(self):
        """The control. Without it the arm above passes on a rig where
        the row would have appeared anyway, and proves nothing."""
        self.assertEqual(
            [r for r in self._rows(clear_map=True)
             if str(r[1]).startswith('via')], [],
            'the pre-fix grading reports the via row too, so this rig does '
            'not discriminate and the arm above is not a measurement')
    # MUTATION: revert the via kind to `both(self._via_shortfalls)`; drop
    # either coordinate swap; record the landing instead of the seed.


class TestASecondRelocationKeepsTheORIGINALSeed(unittest.TestCase):
    """`relocate_vias` can move one barrel twice in a call -- its own docstring
    says so -- and the second hop must not overwrite the seed with the first
    hop's landing."""

    def test_two_hops_record_the_first_position(self):
        with _stub() as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu')
            st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C',
                         set())
            t0 = st.vias[0]
            x0, y0, net = t0[0], t0[1], t0[2]
            st.relocate_vias([(x0, y0, {'net_id': net, 'x': x0 + 0.5,
                                        'y': y0})])
            st.relocate_vias([(x0 + 0.5, y0, {'net_id': net, 'x': x0 + 1.0,
                                              'y': y0})])
            recs = list(st._via_seed_xy.values())
            live = [r for r in recs if r[0] in st.vias]
            self.assertEqual(len(live), 1, 'expected one live entry')
            self.assertAlmostEqual(live[0][1], x0, places=9,
                                   msg='the second hop overwrote the seed '
                                       'with the first landing')
            self.assertAlmostEqual(live[0][2], y0, places=9)
    # MUTATION: record (t[0], t[1]) unconditionally instead of carrying it.


class TestInertWhereNothingMoved(unittest.TestCase):
    """A run that relocates nothing is byte-identical: the map is
    empty, so `seed_pos=True` and the old behaviour agree by
    construction.

    DOCUMENTATION, NOT DETECTION, and labelled so rather than left to look
    like a guard. With an empty map both calls take the identical branch,
    so no mutation in the battery can flip this arm -- it records the
    inertness claim the commit message makes, and nothing more."""

    def test_no_map_means_no_difference(self):
        with _stub() as path:
            pcb, _v = _board(VIA_CLEAR, 'F.Cu', second_cap=True)
            st = _Repair(pcb, path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3, 'C',
                         set())
            self.assertEqual(st._via_seed_xy, {})
            for r, c in st.caps.items():
                self.assertEqual(
                    st._via_shortfalls(r, c, c.seed_x, c.seed_y, c.seed_rot),
                    st._via_shortfalls(r, c, c.seed_x, c.seed_y, c.seed_rot,
                                       seed_pos=True))


if __name__ == '__main__':
    unittest.main(verbosity=2)
