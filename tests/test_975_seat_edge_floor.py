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
        # Short on its own edge, the move blocked by the band: no later rung
        # can fix that on a rectangle, so the ladder does not walk to one.
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


class Bounded(_Boards):
    def test_a_record_names_four_pads_and_counts_the_rest(self):
        pads = '\n    '.join(
            f'(pad "{i + 1}" smd rect (at -2.0 {-0.95 + 0.1 * i:.2f}) (size .5 .05) '
            f'(layers "F.Cu"))' for i in range(20))
        path = self.board('many.kicad_pcb', pads=pads)
        ok, _, record, _ = self.seat(path, west(0.25, 0.35))
        self.assertTrue(ok)
        self.assertEqual(record['n_pads_short'], 20)
        self.assertEqual(len(record['pads']), seeder._FLOOR_RECORD_PADS)
        self.assertLess(len(json.dumps(record)), 1200)


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


class WrittenPoses(_Boards):
    def test_a_record_is_kept_only_at_the_pose_it_describes(self):
        record = {'pose': [2.7, 10.0, 0.0]}
        keep = seeder.floor_records_at_poses
        self.assertEqual(keep({'J1': record}, {'J1': (2.7, 10.0, 360.0)}), {'J1': record})
        self.assertEqual(keep({'J1': record}, {'J1': (2.7004, 10.0, 0.0)}), {'J1': record})
        self.assertEqual(keep({'J1': record}, {'J1': (3.7, 10.0, 0.0)}), {})
        self.assertEqual(keep({'J1': record}, {'J1': (2.7, 10.0, 90.0)}), {})
        self.assertEqual(keep({'J1': record}, {}), {})

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


if __name__ == '__main__':
    unittest.main()
