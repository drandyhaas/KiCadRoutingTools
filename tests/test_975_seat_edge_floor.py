"""#975: an edge seat PREFERS a pose whose pad copper clears the board-edge
floor, and otherwise keeps the pose it always chose -- disclosed.

The seat used to measure pad CENTRES at the gate margin and never pad copper
against the floor, so a legal seat could leave copper inside it (tigard J7's
MP tabs 0.073 mm short, ulx3s AUDIO1 0.385 mm). Refusing such a seat would
turn a clearance shortfall into an unseated -- so unrouted -- connector, so
the floor is a preference. Every arm below uses the unmodified ladder as its
own counterfactual: `_floor_rung` patched to accept every rung is exactly the
ladder before #975 (the first conflict-free seat), which is how each arm
proves that the case it tests would have gone the other way.

  A.  A LATER RUNG THAT CLEARS WINS over a first rung that is short on a side
      moving inward cannot fix (a pad reaching past a courtyard at a corner).
  A2. When every rung is short that way, the kept record says `along_edge`.
  B.  A RUNG MOVED INWARD by the derived distance is taken when the band
      allows it: synthetic, and tigard J7 on its emitted band.
  B2. The 1 um guard is what survives `apply_move`'s 3-dp rounding.
  C.  NO IN-BAND SEAT CLEARS: the pose is today's to the micron, with a record
      (`band_min`) and a NOTE; C2 pins the grade's lower bound, not the seat's
      0.02 tolerance.
  S.  The receptacle SETBACK guard refuses a move that leaves no overhang.
  D.  A pad with no measurable amount never blocks the preference, and is
      listed when a record is written.
  F.  A ROTATION IS NEVER CHANGED to clear the floor.
  G.  STAGE 1 takes the same tiers, and keeps its crowding fallback.
  H.  repair_placement and every place_seed summary carry the key.
  I.  THE SEAT AND THE GRADE AGREE: a record exists exactly when the written
      board's `pad_edge` grade has a finding for the part, with the same worst
      shortfall and pads.
  L.  The floor is the edge clearance, not the gate margin.
  N.  An unreadable project turns the preference off, with one note.
"""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
for folder in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, str(ROOT / folder))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan, seeder  # noqa: E402
from placement import legality as L  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402
import pose_score  # noqa: E402

RUN_ALL_TIMEOUT = 900
EPS = L.EPS
SEED = str(ROOT / 'py_placer' / 'place_seed.py')

#: A body reaching 3 mm west of the origin; two pads 2 mm west of it, one
#: east. Seated on the west edge, the west pads' copper is what the floor sees.
BODY = '(fp_rect (start -3 -1) (end 1 1) (layer "F.Fab"))'
PADS = ('(pad "1" smd rect (at -2.0 -0.5) (size .5 .5) (layers "F.Cu"))\n'
        '    (pad "2" smd rect (at -2.0 0.5) (size .5 .5) (layers "F.Cu"))\n'
        '    (pad "3" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))')
#: The same part with a courtyard that does NOT enclose its first pad, which
#: reaches 0.3-0.55 mm past the courtyard's top: the corner case.
CORNER_BODY = BODY + ' (fp_rect (start -3 -1) (end 1 1) (layer "F.CrtYd"))'
CORNER_PADS = ('(pad "1" smd rect (at -2.0 -1.5) (size .5 .5) (layers "F.Cu"))\n'
               '    (pad "3" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))')
TRAPEZOID = ('\n    (pad "4" smd trapezoid (at -2.3 0.9) (size .5 .5) '
             '(rect_delta 0 .2) (layers "F.Cu"))')


def base_ladder():
    """The ladder before #975: every conflict-free rung is accepted as is."""
    return patch.object(seeder, '_floor_rung',
                        lambda st, part, entry, edge, lo, hi, x, y, crowds:
                        ((x, y), None, None))


def intent_doc(**entry):
    return {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': [dict(entry)]}


def west(lo, hi, **extra):
    return dict({'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': lo, 'max': hi}},
                **extra)


class _Boards(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='t975_')
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def board(self, name, body=BODY, pads=PADS, at='10 10 0', siblings=()):
        path = self.root / name
        path.write_text(
            '(kicad_pcb (version 20241229) (generator "t975")\n'
            '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
            f'  (footprint "t" (layer "F.Cu") (at {at})\n'
            '    (property "Reference" "J1")\n'
            f'    {body}\n    {pads}))\n', encoding='utf-8')
        for suffix, text in siblings:
            path.with_suffix(suffix).write_text(text, encoding='utf-8')
        return str(path)

    def state(self, path, clearance=.25, edge=.55):
        return pose_score.make_state(parse_kicad_pcb(path), path,
                                     clearance=clearance, board_edge_clearance=edge)

    def seat(self, path, entry, base=False, **kw):
        """`_seat_edge` on a fresh state -> (ok, pose, record, notes)."""
        st = self.state(path)
        notes, disclose = [], {}
        if base:
            with base_ladder():
                ok = seeder._seat_edge(st, 'J1', dict(entry), set(), notes, **kw)
        else:
            ok = seeder._seat_edge(st, 'J1', dict(entry), set(), notes,
                                   disclose=disclose, **kw)
        p = st.parts['J1']
        return ok, (p.x, p.y, p.rot), disclose.get('J1'), notes

    def findings(self, path, ref, pose, required=.55):
        """The written board's `pad_edge` findings for `ref` at `pose`."""
        out = str(self.root / f'written_{abs(hash((path, ref, pose)))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': ref, 'new_x': pose[0],
                                         'new_y': pose[1], 'new_rotation': pose[2]}])
        grade = L.grade_pad_edge_clearance(parse_kicad_pcb(out), required, out)
        return [f for f in grade['findings'] if f['pad_ref'].split('.')[0] == ref]

    def agree(self, path, ref, pose, record, required=.55):
        """I. A record exists exactly when the grade finds the part short."""
        found = self.findings(path, ref, pose, required)
        if record is None:
            self.assertEqual(found, [], (ref, pose))
            return
        self.assertTrue(found, (ref, pose, record))
        self.assertAlmostEqual(record['shortfall_mm'],
                               max(f['shortfall_mm'] for f in found), delta=1e-6)
        self.assertEqual(record['n_pads_short'], len(found))
        self.assertLessEqual({p['pad_index'] for p in record['pads']},
                             {f['pad_index'] for f in found})
        self.assertEqual(list(record['pose']), [round(pose[0], 3), round(pose[1], 3), pose[2]])


class LaterRung(_Boards):
    def test_a_later_rung_that_clears_wins(self):
        path = self.board('corner.kicad_pcb', CORNER_BODY, CORNER_PADS)
        entry = west(0.0, 0.6)
        ok, before, _, _ = self.seat(path, entry, base=True, target=(10.0, 0.0))
        self.assertTrue(ok)
        short = self.findings(path, 'J1', before)
        self.assertTrue(short, 'the counterfactual must be short, or this arm tests nothing')
        self.assertEqual({f['edge'] for f in short}, {'bottom'})   # north: inward cannot fix it
        ok, after, record, _ = self.seat(path, entry, target=(10.0, 0.0))
        self.assertTrue(ok)
        self.assertIsNone(record)
        self.assertEqual(after[0], before[0])
        self.assertAlmostEqual(after[1] - before[1], 0.05 * 20.0, places=6)  # one rung
        self.agree(path, 'J1', after, None)

    def test_every_rung_short_along_the_edge_keeps_the_first_and_says_why(self):
        path = self.board('corner_pinned.kicad_pcb', CORNER_BODY, CORNER_PADS)
        entry = west(0.0, 0.6, along_edge_band={'from': 0.08, 'to': 0.105})
        ok, before, _, _ = self.seat(path, entry, base=True, target=(10.0, 0.0))
        ok2, after, record, notes = self.seat(path, entry, target=(10.0, 0.0))
        self.assertTrue(ok and ok2)
        self.assertEqual(after, before)
        self.assertEqual(record['why'], 'along_edge')
        self.assertEqual(record['kept'], 'conflict_free')
        self.agree(path, 'J1', after, record)


class InwardMove(_Boards):
    def test_the_derived_move_clears_the_floor_inside_the_band(self):
        path = self.board('move.kicad_pcb')
        entry = west(0.0, 0.6)
        _, before, _, _ = self.seat(path, entry, base=True)
        short = self.findings(path, 'J1', before)
        self.assertTrue(short)
        ok, after, record, notes = self.seat(path, entry)
        self.assertTrue(ok)
        self.assertIsNone(record)
        self.assertEqual(after[1:], before[1:])
        expected = max(f['shortfall_mm'] for f in short) + seeder._FLOOR_SHIFT_GUARD_MM
        self.assertAlmostEqual(after[0] - before[0], expected, delta=0.0005)
        self.agree(path, 'J1', after, None)
        graded = floorplan.grade(floorplan.intent_from_dict(intent_doc(**entry)),
                                 parse_kicad_pcb(self._written(path, after)),
                                 self._written(path, after), clearance=.25,
                                 board_edge_clearance=.55)
        self.assertEqual([v.message for v in graded.errors if v.rule == 'edge_connector'], [])

    def _written(self, path, pose):
        out = str(self.root / 'graded.kicad_pcb')
        write_placed_output(path, out, [{'reference': 'J1', 'new_x': pose[0],
                                         'new_y': pose[1], 'new_rotation': pose[2]}])
        return out

    def test_tigard_j7_on_its_emitted_band(self):
        path = str(ROOT / 'kicad_files' / 'tigard.kicad_pcb')
        entry = {'ref': 'J7', 'edge': 'south', 'overhang_mm': {'min': 0.0, 'max': 1.155}}
        results = {}
        for base in (True, False):
            st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.2,
                                       board_edge_clearance=.55)
            disclose = {}
            with (base_ladder() if base else patch.object(seeder, '_FLOOR_RECORD_PADS', 4)):
                ok = seeder._seat_edge(st, 'J7', dict(entry), set(), [],
                                       disclose=disclose)
            self.assertTrue(ok)
            p = st.parts['J7']
            results[base] = ((p.x, p.y, p.rot), disclose.get('J7'))
        before, after = results[True][0], results[False][0]
        self.assertEqual(sorted(f['pad_ref'] for f in self.findings(path, 'J7', before)),
                         ['J7.MP', 'J7.MP'])
        self.assertEqual(after[0], before[0])
        self.assertLess(after[1], before[1])             # south edge: inward is -y
        self.assertLess(before[1] - after[1], 0.08)
        self.assertIsNone(results[False][1])
        self.agree(path, 'J7', after, None)

    def test_the_rounding_guard_is_what_survives_apply_move(self):
        # A pad 0.4 um further out: the exact clearing position has four
        # decimals, and 3-dp rounding takes the last one back.
        pads = PADS.replace('(at -2.0 -0.5)', '(at -2.0004 -0.5)').replace(
            '(at -2.0 0.5)', '(at -2.0004 0.5)')
        path = self.board('round.kicad_pcb', pads=pads)
        st = self.state(path)
        seat, floor, why = seeder._floor_rung(st, st.parts['J1'], west(0.0, 0.6), 'west',
                                              0.0, 0.6, 2.7, 10.0, lambda a, b: False)
        self.assertTrue(floor.short)
        self.assertIsNotNone(seat, why)
        self.assertEqual(seat[0], round(seat[0], 3))
        self.assertEqual(seeder._floor_at(st, 'J1', seat[0], seat[1], 0.0).short, ())
        unguarded = round(2.7 + floor.short[0][0], 3)
        self.assertTrue(seeder._floor_at(st, 'J1', unguarded, 10.0, 0.0).short,
                        'without the guard the rounded move must still be short')


class NoInBandSeat(_Boards):
    def test_the_pose_is_kept_and_disclosed(self):
        path = self.board('keep.kicad_pcb')
        entry = west(0.25, 0.35)
        _, before, _, _ = self.seat(path, entry, base=True)
        ok, after, record, notes = self.seat(path, entry)
        self.assertTrue(ok)
        self.assertEqual(after, before)
        self.assertEqual(record['why'], 'band_min')
        self.assertEqual(record['kept'], 'conflict_free')
        self.assertEqual(record['required_mm'], .55)
        self.assertEqual(record['band_min_mm'], .25)
        self.assertLess(record['overhang_after_mm'], .25)
        self.assertEqual({p['pad_index'] for p in record['pads']}, {0, 1})
        self.assertEqual({p['side'] for p in record['pads']}, {'west'})
        self.assertEqual(record['n_unmeasured_pads'], 0)
        note = [n for n in notes if 'edge_floor_fallback' in n]
        self.assertEqual(len(note), 1)
        for forbidden in ('rotation', 'CHECK THIS', 'every position on the declared'):
            self.assertNotIn(forbidden, note[0])
        self.assertLess(len(json.dumps(record)), 1200)
        self.agree(path, 'J1', after, record)

    def test_the_lower_bound_is_the_grades_not_the_seats(self):
        path = self.board('bound.kicad_pcb')
        st = self.state(path)
        part = st.parts['J1']
        # At x 2.7 the body overhangs 0.300 and the west pads are 0.100 short,
        # so the move lands the body at 0.199: inside the seat's own 0.02
        # tolerance of a 0.21 minimum, outside the grade's.
        seat, floor, why = seeder._floor_rung(st, part, west(0.21, 0.6), 'west',
                                              0.21, 0.6, 2.7, 10.0, lambda a, b: False)
        self.assertIsNone(seat)
        self.assertEqual(why['why'], 'band_min')
        self.assertAlmostEqual(why['overhang_after_mm'], 0.199, places=4)
        seat, _, why = seeder._floor_rung(st, part, west(0.19, 0.6), 'west',
                                          0.19, 0.6, 2.7, 10.0, lambda a, b: False)
        self.assertIsNotNone(seat, why)

    def test_a_crowded_move_is_not_taken(self):
        path = self.board('crowd.kicad_pcb')
        st = self.state(path)
        seat, _, why = seeder._floor_rung(st, st.parts['J1'], west(0.0, 0.6), 'west',
                                          0.0, 0.6, 2.7, 10.0, lambda a, b: True)
        self.assertIsNone(seat)
        self.assertEqual(why['why'], 'crowds')


class Setback(_Boards):
    def test_a_receptacle_is_not_moved_to_no_overhang(self):
        # No body: the band is the pad box's occupancy at the gate margin, so
        # clearing the floor takes the reading to zero, where the grade starts
        # charging a receptacle's setback.
        pads = ('(pad "1" smd rect (at 0 -0.5) (size .5 .5) (layers "F.Cu"))\n'
                '    (pad "2" smd rect (at 0 0.5) (size .5 .5) (layers "F.Cu"))')
        path = self.board('setback.kicad_pcb', body='', pads=pads)
        st = self.state(path)
        part = st.parts['J1']
        args = ('west', 0.0, 1.0, 0.7, 10.0, lambda a, b: False)
        seat, _, why = seeder._floor_rung(st, part, west(0.0, 1.0, **{'class': 'edge_receptacle'}),
                                          *args)
        self.assertIsNone(seat)
        self.assertEqual(why['why'], 'setback')
        seat, _, why = seeder._floor_rung(st, part, west(0.0, 1.0, max_setback_mm=0.5), *args)
        self.assertEqual(why['why'], 'setback')
        seat, _, why = seeder._floor_rung(st, part,
                                          west(0.0, 1.0, **{'class': 'connector_affinity'}), *args)
        self.assertEqual(why['why'], 'setback')
        seat, _, why = seeder._floor_rung(st, part, west(0.0, 1.0), *args)
        self.assertIsNotNone(seat, why)


class MoveRefused(_Boards):
    """Every re-check of the moved pose is load-bearing, not only the band."""

    def test_a_keep_out_over_the_moved_pose(self):
        path = self.board('ko.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        keepout = {'name': 'k', 'sides': ('F', 'B'), 'allow': (), 'rect': (3.52, 9.0, 3.9, 11.0)}
        st = pose_score.make_state(pcb, path, clearance=.25, board_edge_clearance=.55,
                                   keepouts=[keepout])
        part = st.parts['J1']
        self.assertTrue(seeder.edge_seat_ok(st, part, 2.5, 10.0, 'west', 0.0, 0.6))
        seat, _, why = seeder._floor_rung(st, part, west(0.0, 0.6), 'west', 0.0, 0.6,
                                          2.5, 10.0, lambda a, b: False)
        self.assertIsNone(seat)
        self.assertEqual(why['why'], 'refused')
        self.assertEqual(why['refused_by'], ["keep-out 'k'"])

    def test_a_move_that_the_floor_still_reads_short(self):
        path = self.board('still.kicad_pcb')
        st = self.state(path)
        stuck = seeder._floor_at(st, 'J1', 2.5, 10.0, 0.0)
        self.assertTrue(stuck.short)
        with patch.object(seeder, '_floor_at', lambda *a, **k: stuck):
            seat, _, why = seeder._floor_rung(st, st.parts['J1'], west(0.0, 0.6), 'west',
                                              0.0, 0.6, 2.5, 10.0, lambda a, b: False)
        self.assertIsNone(seat)
        self.assertEqual(why['why'], 'still_short')

    def test_a_sampled_outline_derives_no_move(self):
        path = self.board('sampled.kicad_pcb')
        text = Path(path).read_text(encoding='utf-8').replace(
            '(layer "Edge.Cuts"))\n',
            '(layer "Edge.Cuts"))\n  (gr_circle (center 15 15) (end 16 15) (layer "Edge.Cuts"))\n', 1)
        Path(path).write_text(text, encoding='utf-8')
        st = self.state(path)
        seat, floor, why = seeder._floor_rung(st, st.parts['J1'], west(0.0, 0.6), 'west',
                                              0.0, 0.6, 2.5, 10.0, lambda a, b: False)
        self.assertIsNone(seat)
        self.assertTrue(floor.short)
        self.assertEqual(why['why'], 'outline_sampled')


#: A locked neighbour 3.9 mm in: clear of J1's first seat, crowded by its move.
NEIGHBOUR = ('  (footprint "n" (locked yes) (layer "F.Cu") (at 3.9 10 0)\n'
             '    (property "Reference" "J2")\n'
             '    (pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu")))\n')


class Crowding(_Boards):
    """Both call sites hand `_floor_rung` their own neighbour test."""

    def board_with_neighbour(self, name):
        path = self.board(name)
        text = Path(path).read_text(encoding='utf-8')
        Path(path).write_text(text[:text.rfind(')')] + NEIGHBOUR + ')\n', encoding='utf-8')
        return path

    def test_seat_edge(self):
        path = self.board_with_neighbour('crowd_seat.kicad_pcb')
        _, before, _, _ = self.seat(path, west(0.0, 0.6), base=True)
        ok, after, record, _ = self.seat(path, west(0.0, 0.6))
        self.assertTrue(ok)
        self.assertEqual(after, before)
        self.assertEqual(record['why'], 'crowds')

    def test_stage_one(self):
        path = self.board_with_neighbour('crowd_s1.kicad_pcb')
        res = StageOne.seed(self, path, west(0.0, 0.6))
        base = StageOne.seed(self, path, west(0.0, 0.6), base=True)
        self.assertEqual(StageOne.pose(res), StageOne.pose(base))
        self.assertEqual(res['edge_floor_fallback']['J1']['why'], 'crowds')


#: The grade reads this part's nearest edge off its courtyard, which is wider
#: than its pads: moved inward near a corner it reads nearest the side edge.
NEAR_BODY = '(fp_rect (start -2 -1.25) (end 2 1.25) (layer "F.CrtYd"))'
NEAR_PADS = ('(pad "1" smd rect (at -1 -0.95) (size .5 .6) (layers "F.Cu"))\n'
             '    (pad "2" smd rect (at 1 -0.95) (size .5 .6) (layers "F.Cu"))')
SLIDE_PADS = NEAR_PADS.replace('-0.95', '-0.93')


class NearestEdge(_Boards):
    """The phase-2 verifier's blocker: a pose the preference picks must still
    read nearest its declared edge, or the grade refuses the seat (rc 4)."""

    def grade_errors(self, path, pose, entry):
        out = str(self.root / 'nearest_written.kicad_pcb')
        write_placed_output(path, out, [{'reference': 'J1', 'new_x': pose[0],
                                         'new_y': pose[1], 'new_rotation': pose[2]}])
        graded = floorplan.grade(floorplan.intent_from_dict(intent_doc(**entry)),
                                 parse_kicad_pcb(out), out, clearance=.25,
                                 board_edge_clearance=.55)
        return [v.message for v in graded.errors if v.rule == 'edge_connector']

    def test_a_move_that_reads_nearest_another_edge_is_not_taken(self):
        path = self.board('near.kicad_pcb', NEAR_BODY, NEAR_PADS)
        st = self.state(path)
        part = st.parts['J1']
        for band, side in (({'from': 0.1, 'to': 0.15}, 'west'), ({'from': 0.85, 'to': 0.9}, 'east')):
            entry = {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 0.6},
                     'along_edge_band': band}
            with self.subTest(band=band):
                base = StageOne.seed(self, path, entry, base=True)
                res = StageOne.seed(self, path, entry)
                pose = StageOne.pose(res)
                self.assertEqual(pose, StageOne.pose(base))
                self.assertEqual(res['edge_floor_fallback']['J1']['why'], 'nearest_edge')
                self.assertEqual(self.grade_errors(path, pose, entry), [])
                # The move itself is what the grade would refuse.
                shift = res['edge_floor_fallback']['J1']['shift_mm']
                moved = (pose[0], round(pose[1] + shift, 3), pose[2])
                self.assertFalse(seeder._faces_its_edge(st, part, entry, 'north', *moved[:2]))
                self.assertTrue([m for m in self.grade_errors(path, moved, entry)
                                 if f'nearest the {side} edge' in m])

    def test_a_short_first_seat_does_not_walk_to_a_corner(self):
        # Short on its own edge, the move blocked by the band: the ladder
        # CHOOSES not to walk (`_SLIDE_HELPS`), and here walking would have
        # found only a corner the grade reads as the wrong edge.
        path = self.board('walk.kicad_pcb', NEAR_BODY, SLIDE_PADS)
        entry = {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.04, 'max': 0.06}}
        _, before, _, _ = self.seat(path, entry, base=True, target=(6.5, 0.0))
        ok, after, record, _ = self.seat(path, entry, target=(6.5, 0.0))
        self.assertTrue(ok)
        self.assertEqual(after, before)
        self.assertEqual(record['why'], 'band_min')
        self.assertEqual(self.grade_errors(path, after, entry), [])

    def test_a_slide_armed_by_an_unrelated_part_does_not_walk(self):
        locked = ('  (footprint "r" (locked yes) (layer "F.Cu") (at 12.5 15 0)\n'
                  '    (property "Reference" "R9")\n'
                  '    (fp_rect (start -1 -0.6) (end 1 0.6) (layer "F.CrtYd"))\n'
                  '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))\n'
                  '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu")))\n')
        path = self.board('armed.kicad_pcb', NEAR_BODY, SLIDE_PADS)
        text = Path(path).read_text(encoding='utf-8').replace('(end 20 20)', '(end 25 20)')
        Path(path).write_text(text[:text.rfind(')')] + locked + ')\n', encoding='utf-8')
        entry = {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.04, 'max': 0.06}}
        base = StageOne.pose(StageOne.seed(self, path, entry, base=True))
        res = StageOne.seed(self, path, entry)
        self.assertEqual(StageOne.pose(res), base)
        self.assertEqual(res['edge_floor_fallback']['J1']['why'], 'band_min')
        self.assertEqual(self.grade_errors(path, base, entry), [])

    def test_a_later_rung_reads_nearest_its_edge_before_it_is_taken(self):
        path = self.board('later_face.kicad_pcb', CORNER_BODY, CORNER_PADS)
        entry = west(0.0, 0.6)
        real = seeder._faces_its_edge
        asked = []

        def spy(st, part, e, edge, x, y):
            asked.append((round(x, 3), round(y, 3)))
            return False
        with patch.object(seeder, '_faces_its_edge', spy):
            ok, after, record, _ = self.seat(path, entry, target=(10.0, 0.0))
        _, before, _, _ = self.seat(path, entry, base=True, target=(10.0, 0.0))
        self.assertTrue(ok)
        self.assertTrue(asked)
        self.assertEqual(after, before)              # every later rung refused
        self.assertEqual(record['why'], 'along_edge')
        self.assertIs(seeder._faces_its_edge, real)


class Unmeasured(_Boards):
    def test_a_pad_with_no_amount_never_blocks_the_preference(self):
        path = self.board('trap.kicad_pcb', pads=PADS + TRAPEZOID)
        pcb = parse_kicad_pcb(path)
        trapezoid = [i for i, p in enumerate(pcb.footprints['J1'].pads) if p.shape == 'trapezoid']
        self.assertEqual(len(trapezoid), 1)
        self.assertFalse(L.pad_shape_is_modelled(pcb.footprints['J1'].pads[trapezoid[0]]))
        ok, after, record, _ = self.seat(path, west(0.0, 0.6))
        self.assertTrue(ok)
        self.assertIsNone(record)
        # The trapezoid's copper is inside the floor, and nothing measured it.
        st = self.state(path)
        fp = st.pcb_data.footprints['J1']
        ctx, _ = L.edge_copper_for(st, st.pcb_data, st.pcb_file, .25, .55)
        reading = ctx.pose_copper(fp, (after[0], after[1], after[2]))
        self.assertEqual(reading.fallback, tuple(trapezoid))
        self.assertTrue(reading.clears)

    def test_it_is_listed_when_a_record_is_written(self):
        path = self.board('trap_keep.kicad_pcb', pads=PADS + TRAPEZOID)
        ok, _, record, _ = self.seat(path, west(0.25, 0.35))
        self.assertTrue(ok)
        self.assertEqual(record['n_unmeasured_pads'], 1)
        self.assertEqual(len(record['unmeasured_pads']), 1)

    def test_the_zero_margin_copper_rule_still_refuses(self):
        path = self.board('trap_off.kicad_pcb', pads=PADS + TRAPEZOID)
        st = self.state(path)
        reasons = []
        # The trapezoid's box reaches 2.55 mm west of the origin.
        self.assertFalse(seeder.edge_seat_ok(st, st.parts['J1'], 2.5, 10.0, 'west',
                                             0.0, 1.0, reasons))
        self.assertTrue([r for r in reasons if 'pad copper' in r], reasons)


class Rotation(_Boards):
    def test_a_declared_ladder_is_not_walked_to_clear_the_floor(self):
        # Without the east pad, which at 180 degrees would leave the board.
        path = self.board('rot.kicad_pcb', pads=PADS.rsplit('\n', 1)[0])
        entry = west(0.25, 0.35)
        # At 180 degrees the west pads face east and the part clears the floor.
        st = self.state(path)
        with base_ladder():
            self.assertTrue(seeder._seat_edge(st, 'J1', dict(entry), set(), [],
                                              rotations=[180.0]))
        p = st.parts['J1']
        self.assertEqual(self.findings(path, 'J1', (p.x, p.y, p.rot)), [])
        ok, after, record, _ = self.seat(path, entry, rotations=[0.0, 180.0])
        self.assertTrue(ok)
        self.assertEqual(after[2], 0.0)
        self.assertEqual(record['why'], 'band_min')


class StageOne(_Boards):
    def seed(self, path, entry, base=False, **kw):
        import random
        intent = floorplan.intent_from_dict(intent_doc(**entry))
        call = lambda: seeder.seed_from_intent(parse_kicad_pcb(path), path, intent,
                                               random.Random(0), clearance=.25,
                                               board_edge_clearance=.55, **kw)
        if base:
            with base_ladder():
                return call()
        return call()

    @staticmethod
    def pose(res, ref='J1'):
        (p,) = [q for q in res['placements'] if q['reference'] == ref]
        return (p['new_x'], p['new_y'], p['new_rotation'])

    def test_tier_one_moves_inward(self):
        path = self.board('s1_move.kicad_pcb')
        before = self.pose(self.seed(path, west(0.0, 0.6), base=True))
        self.assertTrue(self.findings(path, 'J1', before))
        res = self.seed(path, west(0.0, 0.6))
        after = self.pose(res)
        self.assertEqual(res['edge_floor_fallback'], {})
        self.assertGreater(after[0], before[0])
        self.agree(path, 'J1', after, None)

    def test_tier_two_keeps_the_pose_and_records_it(self):
        path = self.board('s1_keep.kicad_pcb')
        before = self.pose(self.seed(path, west(0.25, 0.35), base=True))
        res = self.seed(path, west(0.25, 0.35))
        after = self.pose(res)
        self.assertEqual(after, before)
        record = res['edge_floor_fallback']['J1']
        self.assertEqual((record['kept'], record['why']), ('conflict_free', 'band_min'))
        self.assertTrue([n for n in res['notes']
                         if n.startswith('edge connector J1: its seat on the west edge')])
        self.agree(path, 'J1', after, record)

    def test_tier_one_takes_a_later_rung_when_the_shortfall_faces_a_corner(self):
        path = self.board('s1_corner.kicad_pcb', CORNER_BODY, CORNER_PADS)
        entry = west(0.0, 0.6, along_edge_band={'from': 0.05, 'to': 0.15})
        before = self.pose(self.seed(path, entry, base=True))
        short = self.findings(path, 'J1', before)
        self.assertEqual({f['edge'] for f in short}, {'bottom'})
        res = self.seed(path, entry)
        after = self.pose(res)
        self.assertEqual(res['edge_floor_fallback'], {})
        self.assertEqual(after[0], before[0])
        self.assertGreater(after[1], before[1])
        self.agree(path, 'J1', after, None)

    def test_tier_two_on_an_armed_slide_keeps_the_first_rung_not_the_last(self):
        # A declared position arms the 13-rung slide; every rung is short and
        # blocked by the band, so the kept seat is the FIRST conflict-free
        # rung -- the walk has run on past it to the last.
        entry = west(0.25, 0.35, center_on_edge={'tolerance_mm': 5.0})
        path = self.board('s1_armed.kicad_pcb')
        before = self.pose(self.seed(path, entry, base=True))
        res = self.seed(path, entry)
        after = self.pose(res)
        self.assertEqual(after, before)
        self.assertEqual(res['edge_floor_fallback']['J1']['why'], 'band_min')
        self.agree(path, 'J1', after, res['edge_floor_fallback']['J1'])

    def test_the_crowding_fallback_is_kept_and_named(self):
        # esp_prog USB1 on a band that no seat clears of its neighbours: the
        # pose #961 pinned, now with its copper shortfall disclosed.
        path = str(ROOT / 'kicad_files' / 'esp_prog.kicad_pcb')
        entry = {'ref': 'USB1', 'edge': 'west', 'overhang_mm': {'min': 1.25, 'max': 1.35}}
        res = self.seed(path, entry, seed_refs={'USB1'})
        after = self.pose(res, 'USB1')
        self.assertAlmostEqual(after[0], 116.2, places=3)
        self.assertAlmostEqual(after[1], 98.25, places=3)
        record = res['edge_floor_fallback']['USB1']
        self.assertEqual((record['kept'], record['why']), ('crowding', 'crowding'))
        self.agree(path, 'USB1', after, record)

    def test_a_record_about_a_pose_not_written_is_dropped(self):
        st = self.state(self.board('drop.kicad_pcb'))
        p = st.parts['J1']
        record = {'pose': [round(p.x, 3), round(p.y, 3), p.rot]}
        self.assertEqual(seeder._floor_records_at_final_pose(st, {'J1': record}),
                         {'J1': record})
        st.apply_move('J1', p.x + 1.0, p.y, p.rot)
        self.assertEqual(seeder._floor_records_at_final_pose(st, {'J1': record}), {})


class Repair(_Boards):
    def test_repair_placement_returns_the_record(self):
        path = self.board('repair.kicad_pcb', at='10 10 0')
        intent = floorplan.intent_from_dict(intent_doc(**west(0.25, 0.35)))
        res = seeder.repair_placement(parse_kicad_pcb(path), path, intent,
                                      clearance=.25, board_edge_clearance=.55)
        self.assertIn('J1', res['repaired'], res['notes'])
        (move,) = [m for m in res['moves'] if m['reference'] == 'J1']
        record = res['edge_floor_fallback']['J1']
        self.assertEqual(record['why'], 'band_min')
        self.agree(path, 'J1', (move['new_x'], move['new_y'], move['new_rotation']), record)


class Summaries(_Boards):
    def run_seed(self, *argv):
        env = dict(os.environ, PYTHONHASHSEED='0', KRT_NO_BANNER='1')
        proc = subprocess.run([sys.executable, '-X', 'utf8', SEED, *argv],
                              capture_output=True, text=True, env=env, timeout=600)
        summary = None
        for line in proc.stdout.splitlines():
            if line.startswith('JSON_SUMMARY: '):
                summary = json.loads(line[len('JSON_SUMMARY: '):])
        self.assertIsNotNone(summary, proc.stdout[-2000:] + proc.stderr[-2000:])
        return proc.returncode, summary

    def test_every_summary_carries_the_key(self):
        placed = self.board('cli.kicad_pcb', at='10 10 0')
        ipath = str(self.root / 'cli.json')
        Path(ipath).write_text(json.dumps(intent_doc(**west(0.25, 0.35))), encoding='utf-8')
        out = str(self.root / 'out.kicad_pcb')
        rc, fresh = self.run_seed(placed, out, '--intent', ipath, '--force', '--no-polish')
        self.assertEqual(fresh['edge_floor_fallback']['J1']['why'], 'band_min', fresh)
        self.assertEqual(fresh['unseated_refs'], [])
        rc, repair = self.run_seed(placed, str(self.root / 'rep.kicad_pcb'), '--intent', ipath,
                                   '--repair')
        self.assertEqual(repair['edge_floor_fallback']['J1']['why'], 'band_min', repair)
        rc, dry = self.run_seed(placed, str(self.root / 'dry.kicad_pcb'), '--intent', ipath,
                                '--repair', '--dry-run')
        self.assertIn('J1', dry['edge_floor_fallback'])
        rc, reseat = self.run_seed(out, str(self.root / 'reseat.kicad_pcb'), '--intent', ipath,
                                   '--reseat', 'J1')
        self.assertEqual(reseat['edge_floor_fallback'], {})


class FloorIsTheEdgeClearance(_Boards):
    def test_not_the_gate_margin(self):
        path = self.board('floor.kicad_pcb')
        st = self.state(path, clearance=.6, edge=.3)
        self.assertEqual(st.edge_gate.margin, .6)
        floor = seeder._floor_at(st, 'J1', 2.7, 10.0, 0.0)     # west pads 0.45 mm in
        self.assertEqual(floor.required, .3)
        self.assertEqual(floor.short, ())

    def test_read_at_the_pose_apply_move_writes(self):
        path = self.board('written.kicad_pcb')
        st = self.state(path)
        # Unrounded, the west pads sit 0.5496 mm in -- 0.4 um short of 0.55;
        # written, x is 2.8 and they are exactly on the floor.
        self.assertEqual(seeder._floor_at(st, 'J1', 2.7996, 10.0, 0.0).short, ())
        self.assertTrue(seeder._floor_at(st, 'J1', 2.7994, 10.0, 0.0).short)


class SeatBasis(unittest.TestCase):
    """`floorplan.edge_seat_rect` is the rule's own choice of rect, now shared."""

    def test_a_receptacle_reads_its_drawn_body_fab_or_silk(self):
        part, body = (0, 0, 1, 1), (5, 5, 6, 6)
        rect = floorplan.edge_seat_rect
        for source in ('fab', 'silk'):
            with self.subTest(source=source):
                self.assertEqual(rect({'class': 'edge_receptacle'}, part,
                                      lambda s=source: (body, s)), (body, f'body:{source}'))
                self.assertEqual(rect({'context': {'mount_mode': 'edge_mount'}}, part,
                                      lambda s=source: (body, s)), (body, f'body:{source}'))
        self.assertEqual(rect({'class': 'edge_receptacle'}, part, lambda: (body, 'pad_bbox')),
                         (part, 'courtyard'))
        self.assertEqual(rect({'class': 'edge_receptacle'}, part, lambda: (None, 'none')),
                         (part, 'courtyard'))

        def never():
            raise AssertionError('a non-receptacle entry must not read its body')
        self.assertEqual(rect({'class': 'connector_affinity'}, part, never), (part, 'courtyard'))


class Bounded(_Boards):
    def test_a_record_names_four_pads_and_counts_the_rest(self):
        # STAGGERED in x, 2 microns a step, so each pad is a different distance
        # from the west edge while all twenty stay SHORT of the floor -- and
        # stepped the way round that puts the WORST pad LAST by number, so
        # worst-first ordering and pad-number ordering disagree. Both halves are
        # load-bearing: with every pad at one x a sort by pad number SURVIVED
        # this whole file, and with the worst pad first by number it survived
        # again, because the two orders then agree. A coarser stagger is no good
        # either -- at 0.02mm a step fifteen of the twenty clear the floor.
        pads = '\n    '.join(
            f'(pad "{i + 1}" smd rect (at {-2.0 - 0.002 * i:.3f} '
            f'{-0.95 + 0.1 * i:.2f}) (size .5 .05) (layers "F.Cu"))'
            for i in range(20))
        path = self.board('many.kicad_pcb', pads=pads)
        ok, _, record, _ = self.seat(path, west(0.25, 0.35))
        self.assertTrue(ok)
        self.assertEqual(record['n_pads_short'], 20)
        self.assertEqual(len(record['pads']), seeder._FLOOR_RECORD_PADS)
        self.assertLess(len(json.dumps(record)), 1200)
        # WORST FIRST, and the headline numbers are that pad's: pad 1 sits
        # furthest west, so it is the one short by the most.
        amounts = [p['shortfall_mm'] for p in record['pads']]
        self.assertEqual(amounts, sorted(amounts, reverse=True))
        self.assertEqual(len(set(amounts)), len(amounts),
                         'the stagger must make the shortfalls distinct, or '
                         'the ordering is untested again')
        self.assertEqual(record['shortfall_mm'], amounts[0])
        self.assertEqual(record['min_gap_mm'], record['pads'][0]['gap_mm'])
        # Pad 20 sits furthest west, so it is the worst -- and it is LAST by
        # number, which is what makes this an assertion about the ordering
        # rather than about the pad list's order of arrival.
        self.assertEqual(record['pads'][0]['pad_ref'], 'J1.20')


class UnreadableProject(_Boards):
    def test_the_preference_is_off_and_said_once(self):
        path = self.board('bad_pro.kicad_pcb', siblings=[('.kicad_pro', '[]')])
        entry = west(0.0, 0.6)
        with base_ladder():
            _, before, _, _ = self.seat(path, entry)
        st = self.state(path)
        notes, disclose = [], {}
        self.assertTrue(seeder._seat_edge(st, 'J1', dict(entry), set(), notes,
                                          disclose=disclose))
        self.assertTrue(seeder._seat_edge(st, 'J1', dict(entry), set(), notes,
                                          disclose=disclose))
        p = st.parts['J1']
        self.assertEqual((p.x, p.y, p.rot), before)
        self.assertEqual(disclose, {})
        self.assertEqual(len([n for n in notes if 'could not be read' in n]), 1)

    def test_stage_one_says_it_too(self):
        path = self.board('bad_pro_s1.kicad_pcb', siblings=[('.kicad_pro', '[]')])
        res = StageOne.seed(self, path, west(0.0, 0.6))
        base = StageOne.seed(self, path, west(0.0, 0.6), base=True)
        self.assertEqual(StageOne.pose(res), StageOne.pose(base))
        self.assertEqual(res['edge_floor_fallback'], {})
        self.assertEqual(len([n for n in res['notes'] if 'could not be read' in n]), 1)


def grade_errors_at(self, path, pose, entry, ref='J1'):
    """`edge_connector` grade errors with `ref` written at `pose`."""
    out = str(self.root / f'graded_{abs(hash((path, pose)))}.kicad_pcb')
    write_placed_output(path, out, [{'reference': ref, 'new_x': pose[0],
                                     'new_y': pose[1], 'new_rotation': pose[2]}])
    graded = floorplan.grade(floorplan.intent_from_dict(intent_doc(**entry)),
                             parse_kicad_pcb(out), out, clearance=.25,
                             board_edge_clearance=.55)
    return [v.message for v in graded.errors if v.rule == 'edge_connector']


class GradeConjuncts(_Boards):
    """The delta verifiers' cases: a preferred pose other than the first seat
    must pass the grade's own band, setback, nearest edge and along-edge
    window -- not only the seat's 0.02 mm tolerance -- on every path that can
    pick one. Each blind arm also switches the whole-grade delta off, which
    catches these too: the conjunct guards are what a caller WITHOUT an intent
    (`repair_placement` with none, `_seat_edge` called bare) still has."""

    def write(self, name, text):
        path = self.root / name
        path.write_text(text, encoding='utf-8')
        return str(path)

    def test_a_later_rung_under_the_grades_band_minimum_is_not_taken(self):
        path = self.write('band_later.kicad_pcb',
            '(kicad_pcb (version 20241229) (generator "t975")\n'
            '  (gr_rect (start 0 0) (end 14.3 19.6) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 1.88 6.97 180)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -1.18 -1.89) (end 1.54 1.82) (layer "F.CrtYd"))\n'
            '    (fp_rect (start -2.49 -2.12) (end 0.82 2.14) (layer "F.Fab"))\n'
            '    (pad "1" smd rect (at -1.04 -0.79) (size 0.63 0.6) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at -1.14 0.62) (size 0.82 0.66) (layers "F.Cu"))\n'
            '    (pad "3" smd rect (at -0.57 0.84) (size 0.64 0.55) (layers "F.Cu"))\n'
            '    (pad "4" smd rect (at 1.44 -0.34) (size .5 .5) (layers "F.Cu")))\n'
            '  (footprint "r" (locked yes) (layer "F.Cu") (at 11.46 5.96 0)\n'
            '    (property "Reference" "R9")\n'
            '    (fp_rect (start -1 -0.6) (end 1 0.6) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))))\n')
        entry = {'ref': 'J1', 'edge': 'south', 'overhang_mm': {'min': 0.25, 'max': 0.85}}
        intent = floorplan.intent_from_dict(intent_doc(**entry))

        def repair(accepts=None):
            call = lambda: seeder.repair_placement(parse_kicad_pcb(path), path, intent,
                                                   clearance=.25, board_edge_clearance=.55)
            if accepts is None:
                res = call()
            else:
                with patch.object(seeder, '_grade_accepts', accepts), \
                        patch.object(seeder, '_grade_worse', lambda *a, **k: ()):
                    res = call()
            (move,) = [m for m in res['moves'] if m['reference'] == 'J1']
            return (move['new_x'], move['new_y'], move['new_rotation']), res
        pose, res = repair()
        self.assertEqual(grade_errors_at(self, path, pose, entry), [])
        # The band guard holds on its own, with the whole-grade delta off (a
        # caller with no intent has only the guards).
        guarded, _ = repair(seeder._grade_accepts)
        self.assertEqual(grade_errors_at(self, path, guarded, entry), [])
        # Anti-vacuity: asking only the nearest edge, as the fix before this
        # did, takes the rung the grade refuses.
        nearest_only = lambda st, part, e, edge, lo, x, y: seeder._faces_its_edge(
            st, part, e, edge, x, y)
        bad, _ = repair(nearest_only)
        self.assertNotEqual(bad, pose)
        self.assertTrue([m for m in grade_errors_at(self, path, bad, entry)
                         if 'under the declared minimum' in m])

    def test_the_band_is_read_at_both_of_the_grades_bounds(self):
        path = self.board('bounds.kicad_pcb')
        st = self.state(path)
        part = st.parts['J1']
        # x 2.5 puts the drawn body 0.5 mm past the west edge.
        refuse = seeder._grade_band_refuses
        self.assertEqual(refuse(st, part, west(0.0, 0.2), 'west', 0.0, 2.5, 10.0)[0], 'band_max')
        self.assertEqual(refuse(st, part, west(0.6, 0.9), 'west', 0.6, 2.5, 10.0)[0], 'band_min')
        self.assertIsNone(refuse(st, part, west(0.4, 0.6), 'west', 0.4, 2.5, 10.0)[0])
        # No declared maximum: the grade has none either.
        self.assertIsNone(refuse(st, part, {'ref': 'J1', 'edge': 'west',
                                            'overhang_mm': {'min': 0.0}},
                                 'west', 0.0, 2.5, 10.0)[0])

    def test_the_band_maximum_is_the_grades_not_the_seats(self):
        path = self.board('bound_max.kicad_pcb')
        st = self.state(path)
        refuse = seeder._grade_band_refuses
        # 0.5 mm of body past the edge against a 0.49 maximum: inside the
        # seat's 0.02 tolerance, outside the grade's EPS.
        self.assertEqual(refuse(st, st.parts['J1'], west(0.0, 0.49), 'west', 0.0, 2.5, 10.0)[0],
                         'band_max')

    def test_the_window_is_read_at_the_written_pose_and_for_a_centre_claim(self):
        path = self.write('window_unit.kicad_pcb',
            '(kicad_pcb (version 20241229) (generator "t975")\n'
            '  (gr_rect (start 0 0) (end 28.3 18.0) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 14.15 9.0 270)\n'
            '    (property "Reference" "J1")\n'
            '    (pad "1" smd oval (at 2.75 -1.41) (size 0.69 1.03) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.72 -0.09) (size .5 .5) (layers "F.Cu"))))\n')
        st = self.state(path)
        part = st.parts['J1']
        part.rot = 270.0
        claim = seeder._outside_its_along_edge_claim
        band = {'ref': 'J1', 'edge': 'west', 'along_edge_band': {'from': 0.89, 'to': 0.92}}
        # Walk y across the window's start in 0.1 um steps: the verdict at an
        # unrounded pose must always be the verdict at the pose apply_move
        # writes, and the walk must cross the boundary for that to mean anything.
        verdicts = []
        for k in range(-30, 31):
            y = 14.152 + k * 0.0001
            verdicts.append(claim(st, part, band, 'west', 0.711, y))
            self.assertEqual(verdicts[-1],
                             claim(st, part, band, 'west', 0.711, round(y, 3)), y)
        self.assertEqual(set(verdicts), {True, False})
        # A centre claim is a window too: somewhere along the edge it holds,
        # and far from the centre it does not.
        centre = {'ref': 'J1', 'edge': 'west', 'center_on_edge': {'tolerance_mm': 0.5}}
        sweep = {claim(st, part, centre, 'west', 0.711, 2.0 + 0.25 * k) for k in range(57)}
        self.assertEqual(sweep, {True, False})
        self.assertFalse(claim(st, part, {'ref': 'J1', 'edge': 'west'}, 'west', 0.711, 14.152))

    def test_a_pose_rounded_out_of_its_along_edge_window_is_not_taken(self):
        # Round-3 verifier's board: the -0.4 rung sits ON the window's start,
        # and its inward move, rounded to 3 decimals, lands 0.5 um before it.
        path = self.write('along_min.kicad_pcb',
            '(kicad_pcb (version 20241229) (generator "t975")\n'
            '  (gr_rect (start 0 0) (end 28.3 18.0) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 14.15 9.0 270)\n'
            '    (property "Reference" "J1")\n'
            '    (pad "1" smd oval (at 2.75 -1.41) (size 0.69 1.03) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.72 -0.09) (size .5 .5) (layers "F.Cu"))))\n')
        entry = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.0, 'max': 1.5},
                 'along_edge_band': {'from': 0.89, 'to': 0.92}}
        intent = floorplan.intent_from_dict(intent_doc(**entry))

        def repair(window=True, delta=True, pull=True):
            call = lambda: seeder.repair_placement(parse_kicad_pcb(path), path, intent,
                                                   clearance=.25, board_edge_clearance=.55)
            claim = seeder._outside_its_along_edge_claim if window else (lambda *a, **k: False)
            worse = seeder._grade_worse if delta else (lambda *a, **k: ())
            # #983 pulls a rung whose written pose is outside the window
            # before any of this is asked; `pull=False` puts the rung back on
            # the window end, so the move's own window guard is what is tested.
            pulled = (seeder._window_frac if pull else
                      (lambda st, part, e, edge, bounds, ov, frac, ends, seats=None: frac))
            with patch.object(seeder, '_outside_its_along_edge_claim', claim), \
                    patch.object(seeder, '_grade_worse', worse), \
                    patch.object(seeder, '_window_frac', pulled):
                res = call()
            (move,) = [m for m in res['moves'] if m['reference'] == 'J1']
            return (move['new_x'], move['new_y'], move['new_rotation'])
        pose = repair()
        self.assertEqual(grade_errors_at(self, path, pose, entry), [])
        # The window guard on the move holds on its own, with the delta and the
        # #983 pull off.
        self.assertEqual(grade_errors_at(self, path, repair(delta=False, pull=False), entry), [])
        blind = repair(window=False, delta=False)
        self.assertTrue([m for m in grade_errors_at(self, path, blind, entry)
                         if 'outside the declared band' in m])
        # The same question is part of what a LATER rung must pass: at the
        # rounded-out pose every other conjunct holds and the window alone
        # refuses it.
        st = self.state(path)
        part = st.parts['J1']
        part.rot = blind[2]
        x, y = blind[:2]
        self.assertIsNone(seeder._grade_band_refuses(st, part, entry, 'west', 0.0, x, y)[0])
        self.assertTrue(seeder._faces_its_edge(st, part, entry, 'west', x, y))
        self.assertFalse(seeder._grade_accepts(st, part, entry, 'west', 0.0, x, y))

    def test_stage_one_asks_a_later_rung_its_nearest_edge(self):
        path = self.write('s1_later_face.kicad_pcb',
            '(kicad_pcb (version 20241229) (generator "t975")\n'
            '  (gr_rect (start 0 0) (end 21.0 23.3) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 10.5 11.65 180)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -1.36 -0.45) (end 2.28 2.04) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.67 0.85) (size 0.74 0.53) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at -0.52 -1.45) (size 0.9 0.34) (layers "F.Cu"))\n'
            '    (pad "3" smd rect (at -0.61 1.21) (size 0.46 0.64) (layers "F.Cu"))\n'
            '    (pad "4" smd rect (at 0.72 -0.33) (size .5 .5) (layers "F.Cu"))))\n')
        entry = {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.05, 'max': 0.65},
                 'class': 'edge_receptacle', 'along_edge_band': {'from': 0.82, 'to': 1.0}}
        pose = StageOne.pose(StageOne.seed(self, path, entry))
        self.assertEqual(grade_errors_at(self, path, pose, entry), [])
        with patch.object(seeder, '_faces_its_edge', lambda *a, **k: True), \
                patch.object(seeder, '_grade_worse', lambda *a, **k: ()):
            blind = StageOne.pose(StageOne.seed(self, path, entry))
        self.assertTrue([m for m in grade_errors_at(self, path, blind, entry)
                         if 'sits nearest the' in m])

    def test_the_nearest_edge_is_read_off_a_receptacles_drawn_body(self):
        path = self.write('body_basis.kicad_pcb',
            '(kicad_pcb (version 20241229) (generator "t975")\n'
            '  (gr_rect (start 0 0) (end 14.1 19.5) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 7.05 9.75 0)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -3.25 -0.46) (end 2.47 1.34) (layer "F.CrtYd"))\n'
            '    (fp_rect (start -3.32 -1.58) (end 1.29 2.13) (layer "F.Fab"))\n'
            '    (pad "1" smd rect (at -2.8 2.27) (size 0.64 0.31) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 1.14 -0.73) (size .5 .5) (layers "F.Cu"))))\n')
        entry = {'ref': 'J1', 'edge': 'east', 'overhang_mm': {'min': 0.0, 'max': 0.02},
                 'class': 'edge_receptacle', 'along_edge_band': {'from': 0.75, 'to': 1.0}}
        pose = StageOne.pose(StageOne.seed(self, path, entry))
        self.assertEqual(grade_errors_at(self, path, pose, entry), [])
        from placement.floorplan import _nearest_edge

        def courtyard_only(st, part, e, edge, x, y):
            bounds = st.pcb_data.board_info.board_bounds
            return _nearest_edge(part.rect(round(x, 3), round(y, 3), part.rot),
                                 tuple(round(v, 6) for v in bounds)) == edge
        with patch.object(seeder, '_faces_its_edge', courtyard_only), \
                patch.object(seeder, '_grade_worse', lambda *a, **k: ()):
            blind = StageOne.pose(StageOne.seed(self, path, entry))
        self.assertTrue([m for m in grade_errors_at(self, path, blind, entry)
                         if 'sits nearest the' in m])


class WhichReasonsWalk(_Boards):
    """The walk's CHOICE, pinned on its own: a later rung is considered only
    after an `along_edge` or `outline_sampled` shortfall, never after any
    other reason, and then only if the grade accepts it -- at both call sites.
    The floor readings are scripted so no geometry can hide the choice."""

    SHORT = seeder._Floor(0.55, ((0.1, 0, 'west', 0.45, '1'),), ())

    def scripted(self, first_why):
        calls = []

        def floor_rung(st, part, entry, edge, lo, hi, x, y, crowds):
            calls.append((round(x, 3), round(y, 3)))
            if len(calls) == 1:
                return None, self.SHORT, {'why': first_why}
            return (x, y), self.SHORT, None
        return calls, floor_rung

    def seat(self, first_why, accepts=True, stage_one=False):
        path = self.board(f'walk_{first_why}_{accepts}_{stage_one}.kicad_pcb')
        entry = west(0.0, 0.6, center_on_edge={'tolerance_mm': 8.0})
        calls, floor_rung = self.scripted(first_why)
        with patch.object(seeder, '_floor_rung', floor_rung), \
                patch.object(seeder, '_grade_accepts', lambda *a, **k: accepts):
            if stage_one:
                res = StageOne.seed(self, path, entry)
                return calls, StageOne.pose(res), res['edge_floor_fallback'].get('J1')
            st = self.state(path)
            disclose = {}
            self.assertTrue(seeder._seat_edge(st, 'J1', dict(entry), set(), [],
                                              disclose=disclose))
            p = st.parts['J1']
            return calls, (p.x, p.y, p.rot), disclose.get('J1')

    def test_reasons_about_the_move_keep_the_first_seat(self):
        for stage_one in (False, True):
            for why in ('band_min', 'band_max', 'setback', 'refused', 'crowds',
                        'still_short', 'nearest_edge', 'along_edge_window'):
                with self.subTest(why=why, stage_one=stage_one):
                    calls, pose, record = self.seat(why, stage_one=stage_one)
                    self.assertEqual(len(calls), 1)
                    self.assertEqual((round(pose[0], 3), round(pose[1], 3)), calls[0])
                    self.assertEqual(record['why'], why)

    def test_reasons_a_rung_can_change_walk_on(self):
        for stage_one in (False, True):
            for why in ('along_edge', 'outline_sampled'):
                with self.subTest(why=why, stage_one=stage_one):
                    calls, pose, record = self.seat(why, stage_one=stage_one)
                    self.assertEqual(len(calls), 2)
                    self.assertNotEqual(calls[0], calls[1])
                    self.assertEqual((round(pose[0], 3), round(pose[1], 3)), calls[1])
                    self.assertIsNone(record)

    def test_a_later_rung_the_grade_refuses_is_not_taken(self):
        for stage_one in (False, True):
            with self.subTest(stage_one=stage_one):
                calls, pose, record = self.seat('along_edge', accepts=False,
                                                stage_one=stage_one)
                self.assertGreater(len(calls), 2)
                self.assertEqual((round(pose[0], 3), round(pose[1], 3)), calls[0])
                self.assertEqual(record['why'], 'along_edge')


#: The round-4 verifier's J1: a body 3 mm west of its origin, pads inside it.
GD_J1 = ('  (footprint "t" (layer "F.Cu") (at 15 5 0)\n'
         '    (property "Reference" "J1")\n'
         '    (fp_rect (start -3 -1) (end 1 1) (layer "F.Fab"))\n'
         '    (pad "1" smd rect (at -2.0 -0.5) (size .5 .5) (layers "F.Cu"))\n'
         '    (pad "2" smd rect (at -2.0 0.5) (size .5 .5) (layers "F.Cu"))\n'
         '    (pad "3" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu")))\n')


def gd_part(ref, x, y, *, locked=True, half=1.5):
    return (f'  (footprint "r" {"(locked yes) " if locked else ""}(layer "F.Cu") (at {x} {y} 0)\n'
            f'    (property "Reference" "{ref}")\n'
            f'    (fp_rect (start -{half} -0.6) (end {half} 0.6) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu")))\n')


class GradeDelta(_Boards):
    """The round-4 verifier's blocker: a preferred pose that passes every edge
    conjunct can still break a rule OUTSIDE `rule_edge_connector` -- the
    overlap budget, the part's own zone, a proximity claim -- and place_seed
    then exits 4 on a board its first seat passed. The preference now asks the
    WHOLE intent grade (`floorplan.PoseGrader`) at both poses, pile left out.
    Every arm shows the error with the delta off, on the written board."""

    def write(self, name, parts, size=20):
        path = self.root / name
        path.write_text('(kicad_pcb (version 20241229) (generator "t975")\n'
                        f'  (gr_rect (start 0 0) (end {size} {size}) (layer "Edge.Cuts"))\n'
                        + ''.join(parts) + ')\n', encoding='utf-8')
        return str(path)

    @staticmethod
    def intent(**extra):
        doc = intent_doc(**west(0.0, 0.6))
        doc.update(extra)
        return floorplan.intent_from_dict(doc)

    def errors(self, path, pose, intent, ref='J1'):
        out = str(self.root / f'gd_{abs(hash((path, pose)))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': ref, 'new_x': pose[0],
                                         'new_y': pose[1], 'new_rotation': pose[2]}])
        graded = floorplan.grade(intent, parse_kicad_pcb(out), out, clearance=.25,
                                 board_edge_clearance=.55)
        return sorted(v.message for v in graded.errors)

    def repair(self, path, intent, delta=True):
        def call():
            return seeder.repair_placement(parse_kicad_pcb(path), path, intent,
                                           clearance=.25, board_edge_clearance=.55)
        if delta:
            res = call()
        else:
            with patch.object(seeder, '_grade_worse', lambda *a, **k: ()):
                res = call()
        (move,) = [m for m in res['moves'] if m['reference'] == 'J1']
        return (move['new_x'], move['new_y'], move['new_rotation']), res

    def seed(self, path, intent, delta=True, grade_worse=None):
        import random
        call = lambda: seeder.seed_from_intent(parse_kicad_pcb(path), path, intent,
                                               random.Random(0), clearance=.25,
                                               board_edge_clearance=.55)
        patched = (lambda *a, **k: ()) if not delta else grade_worse
        if patched is None:
            res = call()
        else:
            with patch.object(seeder, '_grade_worse', patched):
                res = call()
        return StageOne.pose(res), res

    def test_repair_does_not_raise_the_overlap_budget(self):
        path = self.write('ov1.kicad_pcb', [GD_J1, gd_part('R9', 4.901, 5)])
        intent = self.intent(legality_budget={'overlap_area': 0.0})
        blind, _ = self.repair(path, intent, delta=False)
        self.assertTrue([m for m in self.errors(path, blind, intent) if 'overlap' in m])
        pose, res = self.repair(path, intent)
        self.assertEqual(self.errors(path, pose, intent), [])
        record = res['edge_floor_fallback']['J1']
        self.assertEqual(record['why'], 'grade_delta')
        self.assertEqual(record['grade_delta'][0]['rule'], 'legality')
        self.assertTrue([n for n in res['notes'] if 'the intent grade' in n])

    def overlap_at(self, path, pose, ref='J1'):
        """The written board's courtyard overlap with `ref` at `pose`."""
        out = str(self.root / f'ovl_{abs(hash((path, pose)))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': ref, 'new_x': pose[0],
                                         'new_y': pose[1],
                                         'new_rotation': pose[2]}])
        state = pose_score.make_state(parse_kicad_pcb(out), out, clearance=.25,
                                      board_edge_clearance=.55)
        return state.legality_metrics()['overlap_area']

    def test_overlap_is_refused_even_with_no_budget_declared(self):
        """A pre-push reviewer's blocker: the GRADE alone cannot see this.

        `_run_rules` skips `legality` when the intent declares no
        `legality_budget`, and `emit_intent` WITHHOLDS that budget on exactly
        the boards that already carry blocking body pairs or unwaived courtyard
        interpenetration. So where overlap is the live risk the rule that would
        catch a move buying more of it is not armed -- and no pad or hole
        predicate sees courtyard overlap either. Measured before the fix: the
        same 0.301 mm inward move, 0.18 mm2 of new interpenetration with a
        LOCKED part, no record, no error, and the exit code unmoved.
        """
        path = self.write('ov_nb.kicad_pcb', [GD_J1, gd_part('R9', 4.901, 5)])
        intent = self.intent()                     # no legality_budget at all
        self.assertFalse(getattr(intent, 'legality_budget', None),
                         'the fixture must declare no budget, or this arm '
                         'tests the armed path instead')
        blind, _ = self.repair(path, intent, delta=False)
        pose, res = self.repair(path, intent)
        self.assertNotEqual(blind[:2], pose[:2])
        self.assertGreater(self.overlap_at(path, blind), L.EPS,
                           'the unguarded pose must really buy overlap, or '
                           'this arm asserts nothing')
        kept = self.overlap_at(path, pose)
        self.assertLessEqual(kept, L.EPS,
                             f'the kept pose carries {kept}mm2 of overlap')
        record = res['edge_floor_fallback']['J1']
        self.assertEqual(record['why'], 'grade_delta')
        self.assertEqual([d.get('budget') for d in record['grade_delta']],
                         ['overlap_area'])
        self.assertEqual(record['n_grade_delta'], len(record['grade_delta']))

    def test_stage_one_does_not_raise_the_overlap_budget(self):
        path = self.write('ov2.kicad_pcb', [GD_J1, gd_part('R9', 4.901, 10)])
        intent = self.intent(legality_budget={'overlap_area': 0.0})
        blind, _ = self.seed(path, intent, delta=False)
        self.assertTrue([m for m in self.errors(path, blind, intent) if 'overlap' in m])
        pose, res = self.seed(path, intent)
        self.assertEqual(self.errors(path, pose, intent), [])
        self.assertEqual(res['edge_floor_fallback']['J1']['why'], 'grade_delta')

    def test_the_pile_is_left_out_of_both_grades(self):
        # R5 is still in the pile, at a meaningless coordinate just past the
        # west edge, overlapping J1's first seat. Graded WITH the pile, the
        # move sheds more of that overlap than it adds against R9, the budget
        # value falls, and the move -- which adds 0.18 mm2 against a part
        # that is really there -- would be taken.
        path = self.write('pile.kicad_pcb', [GD_J1, gd_part('R9', 4.901, 10),
                                             gd_part('R5', -1.0, 10, locked=False)])
        intent = self.intent(legality_budget={'overlap_area': 0.0})
        real = seeder._grade_worse

        def unmasked(grade, ref, rot, first, seat, exclude, memo):
            return real(grade, ref, rot, first, seat, set(), memo)
        blind, _ = self.seed(path, intent, grade_worse=unmasked)
        pose, res = self.seed(path, intent)
        self.assertNotEqual(blind, pose)
        self.assertEqual(res['edge_floor_fallback']['J1']['why'], 'grade_delta')

    def test_a_budget_already_over_may_not_grow(self):
        # Two locked parts already overlap: the budget error exists at both
        # poses, so only its measured value can show what the move adds.
        path = self.write('grow.kicad_pcb', [GD_J1, gd_part('R9', 4.901, 10),
                                             gd_part('R6', 12.0, 16.0),
                                             gd_part('R7', 13.0, 16.0)])
        intent = self.intent(legality_budget={'overlap_area': 0.0})
        blind, _ = self.seed(path, intent, delta=False)
        pose, res = self.seed(path, intent)
        self.assertNotEqual(blind, pose)
        delta = res['edge_floor_fallback']['J1']['grade_delta']
        self.assertEqual([d.get('budget') for d in delta], ['overlap_area'])
        self.assertGreater(delta[0]['after'], delta[0]['before'])

    def test_a_later_rung_stays_in_its_own_zone(self):
        corner = CORNER_BODY + '\n    ' + CORNER_PADS
        path = self.write('zone.kicad_pcb', [
            '  (footprint "t" (layer "F.Cu") (at 10 10 0)\n'
            '    (property "Reference" "J1")\n    ' + corner + ')\n'])
        intent = self.intent(blocks=[{'name': 'io', 'refs': ['J1'],
                                      'zone': [-1.0, 0.5, 5.0, 3.0]}])
        blind, _ = self.repair(path, intent, delta=False)
        self.assertTrue([m for m in self.errors(path, blind, intent) if "block 'io'" in m])
        pose, res = self.repair(path, intent)
        self.assertEqual(self.errors(path, pose, intent), [])
        self.assertEqual(res['edge_floor_fallback']['J1']['why'], 'along_edge')
        # A rule the intent demotes to a warning is not the gate's, so the
        # delta does not stand in its way. (Asked of the seat directly: with
        # the zone only a warning, repair finds nothing to repair.)
        zone = [{'name': 'io', 'refs': ['J1'], 'zone': [-1.0, 0.5, 5.0, 3.0]}]
        for severity, expect_taken in (({}, False), ({'zone_containment': 'warn'}, True)):
            with self.subTest(severity=severity):
                intent = self.intent(blocks=zone, severity=severity)
                st = self.state(path)
                grader = floorplan.PoseGrader(intent, st,
                                              blocks=floorplan.resolve_blocks(intent, st.pcb_data, ())[0],
                                              clearance=.25, board_edge_clearance=.55)
                disclose = {}
                self.assertTrue(seeder._seat_edge(st, 'J1', dict(west(0.0, 0.6)), set(), [],
                                                  target=(2.0, 1.75), disclose=disclose,
                                                  grade=grader))
                p = st.parts['J1']
                self.assertEqual((p.x, p.y, p.rot) == blind, expect_taken)
                self.assertEqual('J1' not in disclose, expect_taken)

    def test_a_proximity_claim_named_by_either_part(self):
        corner = CORNER_BODY + '\n    ' + CORNER_PADS
        parts = ['  (footprint "t" (layer "F.Cu") (at 10 10 0)\n'
                 '    (property "Reference" "J1")\n    ' + corner + ')\n',
                 gd_part('R9', 5.5, 0.9, half=1.0)]
        path = self.write('prox.kicad_pcb', parts)
        zone = [{'name': 'io', 'refs': ['J1'], 'zone': [-1.0, 0.5, 5.0, 3.5],
                 'tolerance_mm': 5.0}]
        for subject, near in (('J1', 'R9'), ('R9', 'J1')):
            with self.subTest(claim=f'{subject} near {near}'):
                intent = self.intent(blocks=zone, proximity=[
                    {'ref': subject, 'near': near, 'max_mm': 1.5}])
                blind, _ = self.repair(path, intent, delta=False)
                self.assertTrue([m for m in self.errors(path, blind, intent)
                                 if 'mm from' in m])
                pose, res = self.repair(path, intent)
                self.assertEqual(self.errors(path, pose, intent), [])

    def test_a_first_seat_that_clears_is_never_graded(self):
        path = self.board('clear.kicad_pcb')
        calls = []
        real = floorplan.PoseGrader.violations

        def spy(self_, **kw):
            calls.append(kw)
            return real(self_, **kw)
        with patch.object(floorplan.PoseGrader, 'violations', spy):
            StageOne.seed(self, path, west(0.0, 0.6, center_on_edge={'tolerance_mm': 8.0}))
            tigard = str(ROOT / 'kicad_files' / 'tigard.kicad_pcb')
            doc = floorplan.emit_intent(parse_kicad_pcb(tigard), tigard)
            ip = self.root / 'tigard.json'
            ip.write_text(json.dumps(doc), encoding='utf-8')
            import random
            before = len(calls)
            res = seeder.seed_from_intent(parse_kicad_pcb(tigard), tigard,
                                          floorplan.load_intent(str(ip)), random.Random('0'),
                                          clearance=.2, board_edge_clearance=.55, grid_step=0.1)
        self.assertGreater(before, 0)                  # the synthetic J1's move was graded
        # tigard: only J7 is short of the floor, graded twice (first seat and
        # its move) -- the other edge connectors clear at their first seat.
        self.assertEqual(len(calls) - before, 2)
        (j7,) = [p for p in res['placements'] if p['reference'] == 'J7']
        self.assertEqual((round(j7['new_x'], 3), round(j7['new_y'], 3)), (52.999, 72.673))
        self.assertNotIn('J7', res['edge_floor_fallback'])

    def test_seat_edge_never_grades_a_first_seat_that_clears(self):
        # Pads 1 mm in from the body's west face: the first seat clears.
        pads = PADS.replace('(at -2.0 -0.5)', '(at -1.0 -0.5)').replace(
            '(at -2.0 0.5)', '(at -1.0 0.5)')
        path = self.board('clear_seat.kicad_pcb', pads=pads)
        st = self.state(path)
        intent = self.intent()
        grader = floorplan.PoseGrader(intent, st, blocks={}, clearance=.25,
                                      board_edge_clearance=.55)
        calls = []
        real = grader.violations
        grader.violations = lambda **kw: (calls.append(kw), real(**kw))[1]
        _, base, _, _ = self.seat(path, west(0.0, 0.6), base=True)
        self.assertTrue(seeder._seat_edge(st, 'J1', dict(west(0.0, 0.6)), set(), [],
                                          grade=grader))
        p = st.parts['J1']
        self.assertEqual((p.x, p.y, p.rot), base)
        self.assertEqual(calls, [])

    def test_a_grade_that_cannot_be_asked_keeps_the_first_seat(self):
        path = self.write('fails.kicad_pcb', [GD_J1, gd_part('R9', 4.901, 10)])
        intent = self.intent()
        base, _ = self.seed(path, intent, delta=False)

        def boom(self_, **kw):
            raise RuntimeError('forced')
        with patch.object(floorplan.PoseGrader, 'violations', boom):
            pose, res = self.seed(path, intent)
        record = res['edge_floor_fallback']['J1']
        self.assertEqual(record['why'], 'grade_delta')
        self.assertIn('unavailable', record['grade_delta'][0])
        self.assertNotEqual(pose, base)
        # The NOTE may not turn "could not be asked" into a verdict about the
        # grade. It said "does not pass the intent grade" here once.
        note = [n for n in res['notes'] if 'J1' in n and 'floor' in n]
        self.assertTrue(note, f'no floor NOTE in {res["notes"]}')
        self.assertIn('could not be compared', note[0])
        self.assertNotIn('does not pass', note[0])
        self.assertEqual(pose[1], base[1])


class PoseGraderParity(unittest.TestCase):
    """`PoseGrader` at poses nothing has written equals `floorplan.grade` of the
    board written at those poses -- message and measured value -- with parts
    moved and turned, on boards whose intents arm every rule family."""

    def check(self, path, intent_path, moves, clearance=.2, edge=.55):
        import shutil
        pcb = parse_kicad_pcb(path)
        intent = floorplan.load_intent(intent_path)
        state = pose_score.make_state(pcb, path, clearance=clearance, board_edge_clearance=edge)
        blocks, _ = floorplan.resolve_blocks(intent, pcb, ())
        grader = floorplan.PoseGrader(intent, state, blocks=blocks, clearance=clearance,
                                      board_edge_clearance=edge)
        posed = sorted((v.sort_key(), v.severity, json.dumps(v.measured, sort_keys=True))
                       for v in grader.violations(poses=moves))
        with tempfile.TemporaryDirectory(prefix='t975p_') as tmp:
            out = os.path.join(tmp, 'posed.kicad_pcb')
            write_placed_output(path, out, [{'reference': r, 'new_x': x, 'new_y': y,
                                             'new_rotation': rot}
                                            for r, (x, y, rot) in moves.items()])
            for ext in ('.kicad_pro', '.kicad_dru'):
                sibling = os.path.splitext(path)[0] + ext
                if os.path.exists(sibling):
                    shutil.copyfile(sibling, os.path.splitext(out)[0] + ext)
            written = parse_kicad_pcb(out)
            graded = floorplan.grade(intent, written, out, clearance=clearance,
                                     board_edge_clearance=edge)
            outside = {v.sort_key() for v in
                       list(floorplan.validate_intent(intent))
                       + list(floorplan.resolve_blocks(intent, written, ())[1])
                       + list(floorplan.unresolved_keepout_allows(intent, written))
                       + list(floorplan.intent_zone_keepout_problems(
                           intent, floorplan.resolve_blocks(intent, written, ())[0],
                           written, out))}
            full = sorted((v.sort_key(), v.severity, json.dumps(v.measured, sort_keys=True))
                          for v in graded.violations if v.sort_key() not in outside)
        self.assertTrue(full, 'a parity check over no violations checks nothing')
        self.assertEqual(posed, full)

    def test_emitted_intents_with_connectors_moved_and_turned(self):
        for name in ('tigard', 'splitflap_driver'):
            with self.subTest(board=name), tempfile.TemporaryDirectory(prefix='t975i_') as tmp:
                path = str(ROOT / 'kicad_files' / f'{name}.kicad_pcb')
                pcb = parse_kicad_pcb(path)
                doc = floorplan.emit_intent(pcb, path)
                doc.setdefault('legality_budget', {})['overlap_area'] = 0.0
                ip = os.path.join(tmp, 'intent.json')
                with open(ip, 'w', encoding='utf-8') as stream:
                    json.dump(doc, stream)
                refs = sorted(c['ref'] for c in doc['edge_connectors'])[:2]
                moves = {}
                for turn, ref in zip((0, 90), refs):
                    fp = pcb.footprints[ref]
                    moves[ref] = (round(fp.x + 3.0, 3), round(fp.y + 2.0, 3),
                                  (fp.rotation + turn) % 360)
                self.check(path, ip, moves)

    def test_the_run_27_esp_prog_plan(self):
        fixture = ROOT / 'tests' / 'fixtures' / '975' / 'esp_prog_run27'
        with tempfile.TemporaryDirectory(prefix='t975e_') as tmp:
            pile = json.loads((fixture / 'pile.json').read_text(encoding='utf-8'))
            board = os.path.join(tmp, 'board.kicad_pcb')
            x, y, rot = pile['pose']
            write_placed_output(str(ROOT / pile['source']), board,
                                [{'reference': r, 'new_x': x, 'new_y': y, 'new_rotation': rot}
                                 for r in pile['refs']])
            import shutil
            shutil.copyfile(fixture / 'board.kicad_pro', os.path.join(tmp, 'board.kicad_pro'))
            self.check(board, str(fixture / 'zone_plan.json'),
                       {'USB1': (117.2, 98.0, 180.0), 'CON1': (141.15, 98.25, 0.0),
                        'U1': (126.0, 99.0, 90.0)}, clearance=.15, edge=.3)


class WrittenPoses(_Boards):
    def test_a_record_is_kept_only_at_the_pose_it_describes(self):
        record = {'pose': [2.7, 10.0, 0.0]}
        keep = seeder.floor_records_at_poses
        self.assertEqual(keep({'J1': record}, {'J1': (2.7, 10.0, 360.0)}), {'J1': record})
        self.assertEqual(keep({'J1': record}, {'J1': (2.7004, 10.0, 0.0)}), {'J1': record})
        self.assertEqual(keep({'J1': record}, {'J1': (3.7, 10.0, 0.0)}), {})
        self.assertEqual(keep({'J1': record}, {'J1': (2.7, 10.0, 90.0)}), {})
        self.assertEqual(keep({'J1': record}, {}), {})
        # The writer prints angles with %g: seated at 0.1234567, written at
        # 0.123457 -- the same seat.
        tilted = {'pose': [2.7, 10.0, 0.1234567]}
        self.assertEqual(keep({'J1': tilted}, {'J1': (2.7, 10.0, 0.123457)}), {'J1': tilted})
        self.assertEqual(keep({'J1': tilted}, {'J1': (2.7, 10.0, 0.13)}), {})

    def test_place_seed_drops_a_record_its_post_polish_reseat_moved(self):
        # J1 is also a zone member, so place_seed's post-polish re-seat pulls
        # it into the zone after stage 1 recorded its edge seat.
        body = BODY
        pads = ('(pad "1" smd rect (at -2.0 -0.5) (size .5 .5) (layers "F.Cu") (net 1 "A"))\n'
                '    (pad "2" smd rect (at -2.0 0.5) (size .5 .5) (layers "F.Cu") (net 2 "B"))\n'
                '    (pad "3" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))')
        r1 = ('  (footprint "r" (layer "F.Cu") (at 12 12 0)\n    (property "Reference" "R1")\n'
              '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu") (net 1 "A"))\n'
              '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu") (net 2 "B")))\n')
        path = self.root / 'zone.kicad_pcb'
        path.write_text('(kicad_pcb (version 20241229) (generator "t975")\n'
                        '  (net 0 "") (net 1 "A") (net 2 "B")\n'
                        '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
                        '  (footprint "t" (layer "F.Cu") (at 10 10 0)\n'
                        '    (property "Reference" "J1")\n'
                        f'    {body}\n    {pads})\n{r1})\n', encoding='utf-8')
        ipath = self.root / 'zone.json'
        doc = intent_doc(**west(0.25, 0.35))
        doc['blocks'] = [{'name': 'conn', 'refs': ['J1'], 'zone': [10.0, 8.0, 16.0, 14.0],
                          'tolerance_mm': 0.1}]
        ipath.write_text(json.dumps(doc), encoding='utf-8')
        # Anti-vacuity: stage 1 does record J1's edge seat before the re-seat.
        import random
        seeded = seeder.seed_from_intent(parse_kicad_pcb(str(path)), str(path),
                                         floorplan.load_intent(str(ipath)), random.Random(0),
                                         clearance=.25, board_edge_clearance=.55)
        self.assertIn('J1', seeded['edge_floor_fallback'])
        out = self.root / 'zone_out.kicad_pcb'
        _rc, summary = Summaries.run_seed(self, str(path), str(out), '--intent', str(ipath),
                                          '--force')
        written = parse_kicad_pcb(str(out)).footprints['J1']
        # Anti-vacuity: the re-seat really moved J1 off its stage-1 seat.
        self.assertGreater(written.x, 5.0)
        self.assertEqual(summary['edge_floor_fallback'], {})


#: A board with an interior Edge.Cuts ring. The parser calls such a ring a
#: CUTOUT -- a hole, which puts anything inside it off the board -- until it
#: encloses two pad centres, when it becomes a milled edge instead
#: (`kicad_parser.drop_pad_containing_cutouts`). A part that moves in or out of
#: it therefore changes how the board itself reads, and a grader holding the
#: classification it was built with stops describing the board that would be
#: written. A verifier measured that on a real 0.6 mm connector move.
RING_BOARD = ('(kicad_pcb (version 20241229) (generator "t975")\n'
              '  (net 0 "") (net 1 "A") (net 2 "B")\n'
              '  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts"))\n'
              '  (gr_rect (start 10 6) (end 20 14) (layer "Edge.Cuts"))\n'
              '  (footprint "t" (layer "F.Cu") (at %s %s 0)\n'
              '    (property "Reference" "J1")\n'
              '    (fp_rect (start -1.2 -0.8) (end 1.2 0.8) (layer "F.CrtYd"))\n'
              '    (pad "1" smd rect (at -0.6 0) (size .5 .5) (layers "F.Cu") (net 1 "A"))\n'
              '    (pad "2" smd rect (at 0.6 0) (size .5 .5) (layers "F.Cu") (net 2 "B")))\n'
              '  (footprint "r" (layer "F.Cu") (at 25 4 0)\n'
              '    (property "Reference" "R1")\n'
              '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu") (net 1 "A"))\n'
              '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu") (net 2 "B")))\n)\n')


class InteriorContours(_Boards):
    """A pose that changes how the BOARD reads is not a pose to compare at.

    `PoseGrader` keeps the state's `edge_gate`, and with it the parser's split
    of interior contours into holes and milled edges. That split is not
    pose-invariant, so two poses on opposite sides of its two-pad threshold
    give grades of two differently-shaped boards. `interior_split` says so and
    `_grade_worse` then reports the delta unavailable, which keeps the seat.
    """

    def ring_board(self, x, y, name='ring.kicad_pcb'):
        path = self.root / name
        path.write_text(RING_BOARD % (x, y), encoding='utf-8')
        return str(path)

    def grader(self, path, **kw):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=.25,
                                      board_edge_clearance=.55)
        intent = floorplan.intent_from_dict(intent_doc(**west(0.0, 0.6)))
        blocks, _ = floorplan.resolve_blocks(intent, pcb, ())
        return floorplan.PoseGrader(intent, state, blocks=blocks, clearance=.25,
                                    board_edge_clearance=kw.get('edge', .55))

    def written_split(self, path, poses):
        """What the PARSER says about the board written at `poses` -- the thing
        `interior_split` has to predict. Called, never mirrored."""
        out = str(self.root / f'w_{abs(hash((path, tuple(sorted(poses.items())))))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': r, 'new_x': p[0], 'new_y': p[1],
                                         'new_rotation': p[2]} for r, p in poses.items()])
        info = parse_kicad_pcb(out).board_info
        return (len(getattr(info, 'board_cutouts', None) or []),
                len(getattr(info, 'board_edge_contours', None) or []))

    def test_a_board_without_interior_contours_answers_nothing(self):
        plain = self.root / 'plain.kicad_pcb'
        plain.write_text('(kicad_pcb (version 20241229) (generator "t975")\n'
                         '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
                         '  (footprint "r" (layer "F.Cu") (at 5 5 0)\n'
                         '    (property "Reference" "R1")\n'
                         '    (pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu")))\n)\n',
                         encoding='utf-8')
        grader = self.grader(str(plain))
        self.assertEqual(grader.interior_split(), ())
        self.assertEqual(grader.interior_split({'R1': (9.0, 9.0, 0.0)}), ())

    def test_the_split_follows_the_pads_and_matches_the_written_board(self):
        # J1 starts OUTSIDE the ring: the ring is a hole. Moved inside, its two
        # pad centres reclassify it as a milled edge.
        path = self.ring_board(25, 16)
        grader = self.grader(path)
        outside = grader.interior_split()
        inside = grader.interior_split({'J1': (15.0, 10.0, 0.0)})
        self.assertEqual((outside, inside), ((False,), (True,)))
        # The parser agrees, on the boards those poses would write.
        self.assertEqual(self.written_split(path, {'J1': (25.0, 16.0, 0.0)}), (1, 0))
        self.assertEqual(self.written_split(path, {'J1': (15.0, 10.0, 0.0)}), (0, 1))

    def test_a_move_of_another_part_is_not_masked_by_the_cache(self):
        """The counts of parts the caller is NOT asking about are cached, and
        must be re-read when the search has moved one.

        A reviewer's reproducer: R1 sits inside the ring at the first call, so
        the board reads as a milled edge; the search then moves R1 out, and the
        ring becomes a hole again. With the base cached once and never re-read,
        `interior_split` answered the old classification for BOTH poses, the
        guard stayed silent, and the delta compared two differently-shaped
        boards -- exactly the failure this method exists to catch. Stage 1
        shares one grader across every declared connector and moves parts
        between them, so it is reachable.
        """
        # R1 has two pads inside the ring at its file pose, J1 none.
        path = self.ring_board(25, 16, name='ring_other.kicad_pcb')
        text = (self.root / 'ring_other.kicad_pcb').read_text(encoding='utf-8')
        (self.root / 'ring_other.kicad_pcb').write_text(
            text.replace('(at 25 4 0)\n', '(at 15 10 0)\n'), encoding='utf-8')
        grader = self.grader(path)
        self.assertEqual(grader.interior_split(), (True,),
                         'R1 must start inside the ring, or this asserts nothing')
        pcb = parse_kicad_pcb(path)
        state = grader.state
        state.apply_move('R1', 25.0, 4.0, 0.0)
        self.assertEqual(grader.interior_split(), (False,),
                         'the cached count must be re-read after the search '
                         'moved R1 out of the ring')
        # And the parser agrees on the two boards those states would write.
        self.assertEqual(self.written_split(path, {'R1': (15.0, 10.0, 0.0)}),
                         (0, 1))
        self.assertEqual(self.written_split(path, {'R1': (25.0, 4.0, 0.0)}),
                         (1, 0))
        self.assertIsNotNone(pcb)

    def test_one_pad_inside_is_not_enough(self):
        # The parser's threshold is TWO centres; a part with one pad in the
        # ring leaves the board reading as it did.
        path = self.ring_board(25, 16)
        grader = self.grader(path)
        self.assertEqual(grader.interior_split({'J1': (10.4, 10.0, 0.0)}), (False,))
        self.assertEqual(self.written_split(path, {'J1': (10.4, 10.0, 0.0)}), (1, 0))

    def test_the_delta_is_unavailable_across_the_threshold(self):
        path = self.ring_board(25, 16)
        grader = self.grader(path)
        memo = {}
        across = seeder._grade_worse(grader, 'J1', 0.0, (25.0, 16.0),
                                     (15.0, 10.0), (), memo)
        self.assertEqual(len(across), 1)
        self.assertIn('interior contours', across[0]['unavailable'])
        # ... and the memo was never filled, so nothing was compared.
        self.assertEqual(memo, {})
        # A move that stays on one side of the threshold is compared normally.
        same = seeder._grade_worse(grader, 'J1', 0.0, (25.0, 16.0),
                                   (25.0, 15.0), (), memo)
        self.assertFalse([d for d in same if 'unavailable' in d])
        self.assertEqual(memo.get('pose'), (25.0, 16.0, 0.0))

    def test_the_guard_reads_the_rounded_pose(self):
        # `apply_move` writes 3 decimals, so the grade is asked about the pose
        # that would be written, in the guard as in the grades.
        seen = []

        class Spy:
            def interior_split(self, poses=None):
                seen.append(('split', poses))
                return ()

            def violations(self, *, exclude=(), poses=None):
                seen.append(('grade', poses))
                return []

        seeder._grade_worse(Spy(), 'J1', 90.0, (2.70049, 10.00051),
                            (3.30049, 10.00051), (), {})
        self.assertTrue(seen)
        for _kind, poses in seen:
            self.assertEqual([round(v, 3) for v in poses['J1'][:2]],
                             list(poses['J1'][:2]))
            self.assertEqual(poses['J1'][2], 90.0)

    def test_ring_ownership_is_read_at_the_view_pose(self):
        # J1 sits INSIDE the ring in the file, so the state's own cache says it
        # owns that ring. Viewed at a pose outside it, it owns nothing -- which
        # is what a grade of the written board would say.
        path = self.ring_board(15, 10)
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=.25, board_edge_clearance=.55)
        here = floorplan._PosedState(state, (), None)
        moved = floorplan._PosedState(state, (), {'J1': (25.0, 16.0, 0.0)})
        self.assertTrue(here._owned_rings('J1'),
                        'the fixture must own a ring at its file pose, or this '
                        'asserts nothing')
        self.assertFalse(moved._owned_rings('J1'))

    def test_the_floors_reach_the_grader_but_not_its_errors(self):
        """What `PoseGrader`'s floors do and do not decide, measured.

        They reach `_Ctx.requested_floors`, which `connector_copper` reads --
        and that channel is EVIDENCE: the clearance it measures raises no
        violation, while the copper-past-the-outline finding the rule does
        grade is geometric and the same at any floor. So no delta can see the
        floors, and a mutation dropping them survives every test here. This
        arm records that rather than leaving it to be rediscovered: graded
        with the real floors and with none at all, on a board with a declared
        connector at a moved pose, the violations are identical.

        Measured the same way on four real boards -- tigard, splitflap_driver,
        ulx3s and watchy, three moved connectors each: identical both ways.
        The battery carries the row as an expected survivor.
        """
        path = self.ring_board(1.65, 10.0)
        pcb = parse_kicad_pcb(path)
        intent = floorplan.intent_from_dict(intent_doc(**west(0.0, 0.6)))
        blocks, _ = floorplan.resolve_blocks(intent, pcb, ())
        state = pose_score.make_state(pcb, path, clearance=.25,
                                      board_edge_clearance=.55)
        poses = {'J1': (1.65, 10.0, 0.0)}

        def marks(clearance, edge):
            grader = floorplan.PoseGrader(intent, state, blocks=blocks,
                                          clearance=clearance,
                                          board_edge_clearance=edge)
            return sorted((v.sort_key(), v.severity)
                          for v in grader.violations(poses=poses))
        self.assertEqual(marks(.25, .55), marks(None, None))

    def test_a_delta_claim_keeps_the_expected_keys_it_is_about(self):
        """One rule, one ref, two different findings: still an error ADDED.

        The claim key carries the EXPECTED keys precisely so that a finding
        swapped for a different finding of the same rule on the same ref is not
        read as no change. Nothing pinned that component -- a mutation dropping
        it from the key SURVIVED the whole file, while a unit probe showed a
        `legality` overlap error swapped for an oob error, and a connector's
        band error swapped for its pad-copper error, both reading as [].
        """
        def v(**expected):
            return floorplan.Violation(
                rule='edge_connector', severity=floorplan.ERROR,
                message='J1 something', ref='J1', block=None,
                measured={'mm': 1.0}, expected=expected)
        band = v(overhang_max_mm=0.6)
        copper = v(required_mm=0.55)
        added = floorplan.grade_delta([band], [copper])
        self.assertEqual([(d['rule'], d['ref'], d['added']) for d in added],
                         [('edge_connector', 'J1', 1)])
        self.assertEqual(floorplan.grade_delta([band], [band]), [])

    def test_a_delta_claim_keeps_the_ref_it_is_about(self):
        # Two parts, one rule, the same expected keys: an error that moves from
        # one ref to another is an error ADDED for the second.
        def v(ref):
            return floorplan.Violation(rule='zone_containment', severity=floorplan.ERROR,
                                       message=f'{ref} is outside its zone', ref=ref,
                                       block='ics', measured={'outside_mm': 1.0},
                                       expected={'zone': [0, 0, 1, 1]})
        added = floorplan.grade_delta([v('A')], [v('B')])
        self.assertEqual([(d['rule'], d['ref'], d['added']) for d in added],
                         [('zone_containment', 'B', 1)])
        self.assertEqual(floorplan.grade_delta([v('A')], [v('A')]), [])


if __name__ == '__main__':
    unittest.main()
