"""#1257: the seat search refuses a pose whose PAD COPPER lies past the
outline, not only one whose courtyard does.

`seeder.pose_ok` -- the predicate place_simple seats and settles parts with
(`simple.legal`) -- held the COURTYARD inside the outline and never looked at
the pads. A part whose pad lies outside its own courtyard (`_scoped`'s R2: a
3 mm pad 4 mm east of a 2 x 1.2 mm courtyard) was seated with that pad past
the east edge. The pad copper is now a conjunct of `pose_ok`, judged by
check_assembly's own gate (`legality.pad_copper_gating_mm`: per pad, margin
0, an edge-straddling castellated pad exempt), and asked only of a part whose
pads reach past its courtyard by more than the edge margin
(`seeder._pad_copper_overhang`), so every other part pays one cached compare.

  P. THE PREDICATE
     P1 R2 at its old lap-0 seat (pad 0.904 mm past the east edge) is refused,
        1 mm west is a seat; blind twin (the conjunct off) admits the old one.
     P2 the conjunct holds at a turn: at 90 degrees the pad points north.
     P3 a castellated pad STRADDLING the outline is exempt, as check_assembly
        exempts it; one wholly past it, or a plain pad, is refused.
     P4 the precompute: only a part reaching past its courtyard by more than
        the margin is ever pad-tested (a 0.35 mm reach is not; R2 is).

The quench's candidate check and a declared fixed pose asked only the
courtyard too; both now ask the same pad question (second #1257 commit):

  Q. THE QUENCH (`candidate_veto`)
     Q1 the old seat is refused, veto `pad_copper_outline`; blind admits it.
     Q2 the off-board unfreeze branch: coming home, a part may not end with
        its pad copper further past the outline than it starts -- refused
        from a start whose pads are on the board, admitted from one whose
        pads are further out.
  F. A FIXED POSE (`seeder._fixed_pose_check`, which place_simple and the
     edge seat read) whose courtyard is inside and whose pad copper is past
     the outline is refused with its measurement; the pose with its pad on
     the board is contained.
"""

import contextlib
import io
import os
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import pose_score                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                     # noqa: E402
from placement import legality, seeder                       # noqa: E402
from test_1174_emit_round_trip import _synthetic             # noqa: E402

#: R2's seat at #1241 (main c0d75d93): its pad's east end at 40.904, past
#: the 40 mm board's east edge by 0.904 mm.
OLD_SEAT = (35.404, 21.482, 0.0)


def _part(ref, x, y, pads, hw=1.0, hh=0.6, locked=False):
    """A part with a +-hw x +-hh courtyard and `pads` =
    [(number, x, y, size_x, size_y, net)] in its own frame."""
    lk = '(locked yes) ' if locked else ''
    return (f'  (footprint "t:P" {lk}(layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start {-hw} {-hh}) (end {hw} {hh}) (stroke'
            f' (width 0.05) (type default)) (layer "F.CrtYd"))\n'
            + ''.join(f'    (pad "{n}" smd rect (at {px} {py}) (size {sx} {sy})'
                      f' (layers "F.Cu") (net {net} "N{net}"))\n'
                      for n, px, py, sx, sy, net in pads)
            + '  )\n')


def _scoped(extra=()):
    """P1-P3 piled, the rest placed. R2's pad 1 sits 4 mm east of its
    courtyard, 0.5 mm inside the east edge, net-tied to a LOCKED R3 at that
    edge."""
    return [
        _part(f'P{i}', 10, 30, [(1, -0.5, 0, 0.5, 0.6, 1),
                                (2, 0.5, 0, 0.5, 0.6, 2)])
        for i in (1, 2, 3)] + [
        _part('R2', 34, 20, [(1, 4.0, 0, 3.0, 0.6, 3),
                             (2, -0.5, 0, 0.5, 0.6, 3)]),
        _part('R3', 38.8, 23, [(1, 0.5, 0, 0.2, 0.6, 3)], hw=0.6,
              hh=0.6, locked=True),
        _part('R5', 25, 12, [(1, 0, 0, 0.5, 0.6, 5)]),
        _part('R4', 30, 10, [(1, 0, 0, 0.5, 0.6, 4)])] + list(extra)


def _blind():
    """The conjunct off: `pose_ok` as it was before #1257."""
    return patch.object(seeder, '_pad_copper_overhang', lambda *a, **k: 0.0)


def _state(path):
    return pose_score.make_state(parse_kicad_pcb(path), path, clearance=0.25,
                                 board_edge_clearance=0.55)


def _ok(st, ref, pose):
    """`pose_ok` with every other part excluded: the board alone decides."""
    return seeder.pose_ok(st, ref, *pose, set(st.parts) - {ref})


def _gating(path, ref):
    """check_assembly's verdict on `ref` in a WRITTEN board: its overrun in
    `oob_pad_copper_overrun_mm` if it is in the gating list, else 0.0."""
    rep = legality.grade_pad_legality(parse_kicad_pcb(path), 0.25, worst_n=0,
                                      edge_margin=0.55, pcb_file=path)
    if ref not in rep['oob_pad_copper_gating_refs']:
        return 0.0
    return rep['oob_pad_copper_overrun_mm'][ref]


def _castellated(ref, x, lx, castellated=True):
    """A +-1 x +-0.6 mm courtyard with one 1 x 1 mm pad at local (lx, 0)."""
    prop = ' (property pad_prop_castellated)' if castellated else ''
    return (f'  (footprint "t:C" (layer "F.Cu") (at {x} 20)\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -1 -0.6) (end 1 0.6) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at {lx} 0) (size 1 1) (layers "F.Cu")'
            f'{prop} (net 2 "N2"))\n  )\n')


class ThePredicate(unittest.TestCase):

    def test_p1_the_old_seat_is_refused(self):
        with tempfile.TemporaryDirectory() as td:
            st = _state(_synthetic(td, _scoped()))
            self.assertAlmostEqual(seeder._pad_copper_overhang(st, 'R2'), 4.5)
            self.assertAlmostEqual(
                seeder._pad_copper_off_board(st, 'R2', *OLD_SEAT), 0.904)
            self.assertFalse(_ok(st, 'R2', OLD_SEAT))
            self.assertTrue(_ok(st, 'R2', (34.404, 21.482, 0.0)))
            # The blind twin: the courtyard alone admits the old seat.
            with _blind():
                self.assertTrue(_ok(st, 'R2', OLD_SEAT))

    def test_p2_a_turned_part_is_held_too(self):
        # At 90 degrees R2's pad sits 4 mm NORTH of its origin: y 3.0 puts
        # the courtyard 2 mm in and the pad's centre 1 mm past the edge.
        with tempfile.TemporaryDirectory() as td:
            st = _state(_synthetic(td, _scoped()))
            self.assertGreater(
                seeder._pad_copper_off_board(st, 'R2', 20.0, 3.0, 90.0), 1.0)
            self.assertFalse(_ok(st, 'R2', (20.0, 3.0, 90.0)))
            self.assertTrue(_ok(st, 'R2', (20.0, 6.0, 90.0)))
            with _blind():
                self.assertTrue(_ok(st, 'R2', (20.0, 3.0, 90.0)))

    def test_p3_a_castellated_pad_on_the_edge_is_exempt(self):
        # The courtyard ends 0.8 mm inside the east edge at x 38.2, clear of
        # the 0.55 mm margin, so only the pad can refuse the pose.
        for lx, castellated, x, seat in (
                (1.9, True, 38.2, True),      # 39.6..40.6: straddles
                (1.9, False, 38.2, False),    # same copper, not castellated
                (3.0, True, 38.4, False)):    # 40.9..41.9: wholly past
            with self.subTest(lx=lx, castellated=castellated), \
                    tempfile.TemporaryDirectory() as td:
                path = _synthetic(td, [_castellated('C1', 10, lx,
                                                    castellated)])
                st = _state(path)
                self.assertEqual(_ok(st, 'C1', (x, 20.0, 0.0)), seat)
                # The predicate's verdict is check_assembly's, at that pose
                # written: the same gate, so the seat is never charged.
                with open(path, encoding='utf-8') as fh:
                    text = fh.read()
                with open(path, 'w', encoding='utf-8') as fh:
                    fh.write(text.replace('(at 10 20)', f'(at {x} 20)', 1))
                self.assertEqual(_gating(path, 'C1') > legality.EPS,
                                 not seat)

    def test_p4_only_a_part_reaching_past_the_margin_is_pad_tested(self):
        # R6 reaches 0.35 mm past its courtyard, under the 0.55 mm margin:
        # its copper is on the board wherever its courtyard is, so pose_ok
        # never measures it. R2 (4.5 mm) is measured.
        r6 = _part('R6', 12, 12, [(1, 1.0, 0, 0.7, 0.6, 6)])
        calls = set()
        real = seeder._pad_copper_off_board

        def spy(state, ref, *pose):
            calls.add(ref)
            return real(state, ref, *pose)
        with tempfile.TemporaryDirectory() as td:
            path = _synthetic(td, _scoped([r6]))
            st = _state(path)
            self.assertAlmostEqual(seeder._pad_copper_overhang(st, 'R6'),
                                   0.35)
            self.assertEqual(seeder._pad_copper_overhang(st, 'R4'), 0.0)
            with patch.object(seeder, '_pad_copper_off_board', spy):
                for ref in ('R6', 'R4', 'R2'):
                    p = st.parts[ref]
                    _ok(st, ref, (p.x, p.y, p.rot))
        self.assertEqual(calls, {'R2'})


def _veto(st, ref, pose):
    """`candidate_veto` with every other part excluded."""
    with contextlib.redirect_stdout(io.StringIO()):
        return st.candidate_veto(ref, *pose, exclude=set(st.parts) - {ref})


class TheQuench(unittest.TestCase):

    def test_q1_the_old_seat_is_refused(self):
        with tempfile.TemporaryDirectory() as td:
            st = _state(_synthetic(td, _scoped()))
            self.assertEqual(_veto(st, 'R2', OLD_SEAT),
                             ('pad_copper_outline', None))
            self.assertIsNone(_veto(st, 'R2', (34.404, 21.482, 0.0)))
            with _blind():
                self.assertIsNone(_veto(st, 'R2', OLD_SEAT))

    def test_q2_coming_home_the_copper_may_not_go_further_out(self):
        # R2's courtyard pokes past the east edge either way, so the
        # candidate (37, 20, 0) -- courtyard inside, pad 2.5 mm past --
        # reaches the unfreeze branch, whose courtyard term improves.
        home = (37.0, 20.0, 0.0)
        with tempfile.TemporaryDirectory() as td:
            st = _state(_synthetic(td, _scoped()))
            # From (39.6, 20, 90): pad pointing north, on the board.
            st.apply_move('R2', 39.6, 20.0, 90.0)
            self.assertEqual(st._pad_copper_past('R2', 39.6, 20.0, 90.0), 0.0)
            self.assertEqual(_veto(st, 'R2', home),
                             ('pad_copper_outline', None))
            with _blind():
                self.assertIsNone(_veto(st, 'R2', home))
            # From (39.5, 20, 0): pad 5 mm past -- the candidate is better.
            st.apply_move('R2', 39.5, 20.0, 0.0)
            self.assertAlmostEqual(
                st._pad_copper_past('R2', 39.5, 20.0, 0.0), 5.0)
            self.assertIsNone(_veto(st, 'R2', home))



class TheFixedPose(unittest.TestCase):

    def test_f_the_check_names_the_copper(self):
        with tempfile.TemporaryDirectory() as td:
            st = _state(_synthetic(td, _scoped()))
            got = seeder._fixed_pose_check(st, 'R2', OLD_SEAT, {})
            self.assertEqual(got, (None, ['pad copper 0.904mm past the '
                                          'outline'], {}))
            self.assertEqual(seeder._fixed_pose_check(
                st, 'R2', (34.404, 21.482, 0.0), {})[0], 'contained')
            with _blind():
                self.assertEqual(seeder._fixed_pose_check(
                    st, 'R2', OLD_SEAT, {})[0], 'contained')



if __name__ == '__main__':
    unittest.main()
