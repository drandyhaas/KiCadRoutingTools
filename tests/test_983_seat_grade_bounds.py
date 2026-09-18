"""#983, #987 and #988: an edge seat writes the pose its own grade accepts.

Each arm seats a connector, WRITES the pose, re-parses the board and runs the
real intent grade (`floorplan.grade`), classifying `edge_connector` errors by
the `measured` key the rule writes rather than by message text. Every arm that
tests a correction FIRING has a blind twin that patches it back to identity
and must show the error it exists to remove, so no such arm can pass on a
fixture that was never broken; unit arms (the step's geometry, the reading,
refusals) pin the function directly. A10 and B7 go through
`repair_placement`, the production caller, which hands the seat a grader.

  A.  (#983) A RUNG WRITTEN OUTSIDE ITS DECLARED ALONG-EDGE WINDOW is stepped
      one 0.001 mm grid unit along the edge, inside (`_window_nudge`); a rung
      the grade accepts is returned bit-identical.
      A1  the issue's two blocker positions (both window ends), to the micron.
      A2  a lattice slice through both ladders; a seat differs from the blind
          one only when a step fired (A6).
      A3  an asymmetric part at 0/90/180/270: the end that rounds out, and
          only it, moves one grid unit inward on the edge axis.
      A4  a window narrower than the grid is never refused, and a miss is
          named; A4b the step gives way to the raw rung when it would cost
          the seat or deepen the floor.
      A5  `center_on_edge` with `tolerance_mm: 0`: met on the grid, named off
          it.
      A8  a centre claim's window end.
      A9  on a notched outline the step reads the EDGE's span.
  B.  (#987) A RUNG WHOSE WRITTEN POSE READS OUTSIDE ITS DECLARED OVERHANG BAND
      is moved along the edge normal until the grade's own reading is inside
      (`_band_settle`), when the moved pose still seats, leaves the floor no
      shorter and trades no setback; a rung the grade accepts is untouched.
      B1  a drawn body past its courtyard, both band ends, both ladders.
      B2  no declared max, the target ON the minimum, the body off the grid.
      B3  a gate margin under the walk's own 0.02 mm tolerance.
      B4  refusals keep the raw pose: a floor it would deepen, a setback it
          would trade for, a body set back further than the cap, a raise.
  C.  (#988) STAGE 1 MEASURES AT THE ROTATION IT WRITES. The part's extents, the
      declared start fraction and the declared window are computed at the
      DECLARED rotation, not the input one (`_stage1_geometry_rot`).
      C1  splitflap_driver J5 declared at 0/90/270 lands inside its centre
          claim and its band (was 10.00 / 5.65 / 4.60 mm off centre).
      C2  a part that fits the edge ONLY at its declared rotation is seated
          by stage 1 instead of being refused as wider than the edge.
      C3  a part stage 1 skips keeps its input rotation (the geometry turn
          is a measurement, not a move); C1 also asserts the #893 turn is
          still made and said, which a read that turned the part would not.
      C4  the declared START is converted at the declared angle, on and off
          the 90-degree lattice (a 30 or 135 needs its box materialised).
      C5  a rotation CANDIDATE set is measured at the part's own angle.
      C6  an undeclared angle is read exactly as before, cache quirks and all.
"""
import os
from pathlib import Path
import random
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
for folder in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, str(ROOT / folder))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan, seeder  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

RUN_ALL_TIMEOUT = 900
SPLIT = str(ROOT / 'kicad_files' / 'splitflap_driver.kicad_pcb')


def intent_doc(entry, blocks=()):
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
           'edge_connectors': [dict(entry)]}
    if blocks:
        doc['blocks'] = [dict(b) for b in blocks]
    return doc


def kinds(violations, ref):
    """`edge_connector` errors on `ref`, by the `measured` key the rule writes."""
    out = []
    for v in violations:
        if v.ref != ref or v.rule != 'edge_connector':
            continue
        m = v.measured or {}
        if 'along_edge_fraction' in m or 'along_edge_offset_mm' in m:
            out.append('along_edge')
        elif 'overhang_mm' in m:
            out.append('band')
        elif 'edge_clearance_mm' in m:
            out.append('setback')
        elif 'edge' in m:
            out.append('nearest_edge')
        else:
            out.append('other')
    return sorted(out)


class _Graded(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='t983_')
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, name, text):
        path = self.root / name
        path.write_text(text, encoding='utf-8')
        return str(path)

    def graded(self, path, poses, doc, clearance=.25, edge=.55):
        """Write `poses` ({ref: (x, y, rot)}), re-parse, grade: {ref: kinds}."""
        out = str(self.root / f'graded_{abs(hash((path, tuple(sorted(poses.items())))))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': r, 'new_x': round(p[0], 3),
                                         'new_y': round(p[1], 3), 'new_rotation': p[2]}
                                        for r, p in poses.items()])
        g = floorplan.grade(floorplan.intent_from_dict(doc), parse_kicad_pcb(out), out,
                            clearance=clearance, board_edge_clearance=edge)
        return {r: kinds(g.errors, r) for r in poses}

    def stage1(self, path, doc, clearance=.25, edge=.55, **kw):
        return seeder.seed_from_intent(parse_kicad_pcb(path), path,
                                       floorplan.intent_from_dict(doc), random.Random(0),
                                       clearance=clearance, board_edge_clearance=edge, **kw)

    @staticmethod
    def pose(res, ref):
        (p,) = [q for q in res['placements'] if q['reference'] == ref]
        return (p['new_x'], p['new_y'], p['new_rotation'])

    def repair(self, path, doc, blind=False):
        """J1's pose from `repair_placement` -- the PRODUCTION caller, which
        hands `_seat_edge` a live grader."""
        call = lambda: seeder.repair_placement(parse_kicad_pcb(path), path,
                                               floorplan.intent_from_dict(doc),
                                               clearance=.25, board_edge_clearance=.55)
        if blind:
            with no_nudge(), no_settle():
                res = call()
        else:
            res = call()
        (m,) = [q for q in res['moves'] if q['reference'] == 'J1']
        return (m['new_x'], m['new_y'], m['new_rotation'])

    def full_grade(self, path, pose, doc):
        """(edge kinds, legality errors, board overlap_area) of J1 at `pose`."""
        out = str(self.root / f'fg_{abs(hash((path, pose, str(doc))))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': 'J1', 'new_x': round(pose[0], 3),
                                         'new_y': round(pose[1], 3), 'new_rotation': pose[2]}])
        g = floorplan.grade(floorplan.intent_from_dict(doc), parse_kicad_pcb(out), out,
                            clearance=.25, board_edge_clearance=.55)
        return (kinds(g.errors, 'J1'), [v for v in g.errors if v.rule == 'legality'],
                g.legality.get('overlap_area'))


def input_rotation():
    """The blind twin of C: stage 1 measures at the input rotation again."""
    return patch.object(seeder, '_stage1_geometry_rot', lambda part, claim: part.rot)


class StageOneRotation(_Graded):
    J5 = {'ref': 'J5', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 0.8}}

    def seat_j5(self, claim, rot, blind=False):
        doc = intent_doc(dict(self.J5, **claim),
                         blocks=[{'name': 'j5', 'refs': ['J5'], 'rotation': rot}])
        if blind:
            with input_rotation():
                res = self.stage1(SPLIT, doc, seed_refs={'J5'})
        else:
            res = self.stage1(SPLIT, doc, seed_refs={'J5'})
        pose = self.pose(res, 'J5')
        self.assertAlmostEqual(pose[2] % 360.0, rot % 360.0, delta=1e-9)
        return pose, self.graded(SPLIT, {'J5': pose}, doc)['J5'], res

    def test_c1_the_declared_window_is_met_at_the_declared_rotation(self):
        blind_dirty = 0
        for claim in ({'center_on_edge': {'tolerance_mm': 1.0}},
                      {'along_edge_band': {'from': 0.2, 'to': 0.3}}):
            for rot in (0, 90, 180, 270):
                with self.subTest(claim=sorted(claim)[0], rot=rot):
                    _, errs, res = self.seat_j5(claim, rot)
                    self.assertNotIn('along_edge', errs)
                    # The turn is still #893's to make, and to say: a read
                    # that turned the part itself would leave nothing to turn.
                    said = [n for n in res['notes']
                            if n.startswith('edge connector J5: seated at the declared rotation')]
                    self.assertEqual(len(said), int(rot != 180), res['notes'])
                    _, blind, _ = self.seat_j5(claim, rot, blind=True)
                    blind_dirty += 'along_edge' in blind
        # Measured on the unfixed ladder: 0/90/270 for the centre claim and 0
        # for the band. Anti-vacuity: the fixture must be broken without C.
        self.assertGreaterEqual(blind_dirty, 4)

    WIDE = ('(kicad_pcb (version 20241229) (generator "t983")\n'
            '  (gr_rect (start 0 0) (end 8 30) (layer "Edge.Cuts"))\n'
            '  (footprint "t" (layer "F.Cu") (at 4 15 0)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -6 -1) (end 6 1) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -4.5 0) (size .5 .5) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 4.5 0) (size .5 .5) (layers "F.Cu"))))\n')

    def test_c2_a_part_that_fits_only_at_its_declared_rotation_is_seated(self):
        path = self.write('wide.kicad_pcb', self.WIDE)
        doc = intent_doc({'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 1.0}},
                         blocks=[{'name': 'j1', 'refs': ['J1'], 'rotation': 90}])
        res = self.stage1(path, doc)
        self.assertFalse([n for n in res['notes'] if 'wider than the north' in n], res['notes'])
        pose = self.pose(res, 'J1')
        self.assertAlmostEqual(pose[2] % 360.0, 90.0, delta=1e-9)
        self.assertEqual(self.graded(path, {'J1': pose}, doc)['J1'], [])
        with input_rotation():
            blind = self.stage1(path, doc)
        self.assertTrue([n for n in blind['notes'] if 'wider than the north' in n],
                        'the fixture must be refused at its input rotation, or C2 tests nothing')

    def test_c3_a_part_stage_one_skips_keeps_its_input_rotation(self):
        # The window [0.99, 1.0] cannot hold J5 at any rotation (its half
        # extent keeps its centre inside ~0.96), so stage 1 refuses it by name
        # and must not have turned it: the geometry turn is a measurement.
        doc = intent_doc(dict(self.J5, along_edge_band={'from': 0.99, 'to': 1.0}),
                         blocks=[{'name': 'j5', 'refs': ['J5'], 'rotation': 90}])
        res = self.stage1(SPLIT, doc, seed_refs={'J5'})
        self.assertTrue([n for n in res['notes']
                         if n.startswith('edge connector J5: the declared along-edge window')],
                        res['notes'])
        self.assertFalse([n for n in res['notes']
                          if n.startswith('edge connector J5: seated at the declared rotation')])

    #: An asymmetric part alone on a board: nothing crowds it, so its first
    #: rung -- the declared START -- is the seat, and where it lands says
    #: whether the start was converted at the right angle.
    ALONE = ('(kicad_pcb (version 20241229) (generator "t983")\n'
             '  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts"))\n'
             '  (footprint "t" (layer "F.Cu") (at 15 10 0)\n'
             '    (property "Reference" "J1")\n'
             '    (fp_rect (start -0.3 -2.6) (end 4.1 0.9) (layer "F.CrtYd"))\n'
             '    (pad "1" smd rect (at 1.9 -0.8) (size .5 .5) (layers "F.Cu"))))\n')

    def centre_of(self, path, pose):
        """The written courtyard centre along the north edge (x)."""
        import pose_score
        out = str(self.root / f'c_{abs(hash((path, pose)))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': 'J1', 'new_x': round(pose[0], 3),
                                         'new_y': round(pose[1], 3), 'new_rotation': pose[2]}])
        st = pose_score.make_state(parse_kicad_pcb(out), out, clearance=.25,
                                   board_edge_clearance=.55)
        p = st.parts['J1']
        r = p.rect(p.x, p.y, p.rot)
        return (r[0] + r[2]) / 2.0

    def test_c4_the_start_is_converted_at_the_declared_angle_on_and_off_the_lattice(self):
        path = self.write('alone.kicad_pcb', self.ALONE)
        entry = {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 1.0},
                 'center_on_edge': {'tolerance_mm': 1.0}}
        for rot in (90, 270, 30, 135):
            with self.subTest(rot=rot):
                doc = intent_doc(entry, blocks=[{'name': 'j1', 'refs': ['J1'], 'rotation': rot}])
                pose = self.pose(self.stage1(path, doc), 'J1')
                self.assertAlmostEqual(pose[2] % 360.0, float(rot), delta=1e-9)
                # On the declared start, to the grid: not merely inside the
                # tolerance, which a start converted at 0 and then clamped
                # into the window would also be.
                self.assertAlmostEqual(self.centre_of(path, pose), 15.0, delta=0.0006)
                self.assertEqual(self.graded(path, {'J1': pose}, doc)['J1'], [])

    #: -31.91 is kept by the quench at `rot` 328.09 but its rotated box is
    #: keyed one ulp away (328.09000000000003), so `rect` answers with the
    #: 0-degree box. A pre-existing quirk this change must NOT quietly fix
    #: for an undeclared part (the phase-3 verifier's finding): the poses
    #: below are the base tree's own output (faa03526), recorded, not re-derived.
    ULP = ('(kicad_pcb (version 20241229) (generator "t983")\n'
           '  (gr_rect (start 0 0) (end 40 20) (layer "Edge.Cuts"))\n'
           '  (footprint "t" (layer "F.Cu") (at 20 10 -31.91)\n'
           '    (property "Reference" "J1")\n'
           '    (fp_rect (start -2.2 -1.4) (end 3.1 0.9) (layer "F.CrtYd"))\n'
           '    (pad "1" smd rect (at -1.2 0) (size .5 .5) (layers "F.Cu"))\n'
           '    (pad "2" smd rect (at 1.2 0) (size .5 .5) (layers "F.Cu"))))\n')

    def test_c6_an_undeclared_angle_is_read_exactly_as_before(self):
        path = self.write('ulp.kicad_pcb', self.ULP)
        base = {(): (20.0, 18.565, 328.09), ('center_on_edge',): (19.55, 18.565, 328.09)}
        for extra, want in base.items():
            with self.subTest(extra=extra):
                entry = {'ref': 'J1', 'edge': 'south', 'overhang_mm': {'min': 0.0, 'max': 1.0}}
                if extra:
                    entry['center_on_edge'] = {'tolerance_mm': 2.0}
                pose = self.pose(self.stage1(path, intent_doc(entry)), 'J1')
                self.assertEqual(tuple(round(v, 6) for v in pose), want)

    def test_c5_a_candidate_set_is_measured_at_the_parts_own_angle(self):
        # Stage 1 does not apply a candidate SET, so it must not measure at
        # one of its members either: identical to measuring at the input.
        claim = {'center_on_edge': {'tolerance_mm': 1.0}}
        doc = intent_doc(dict(self.J5, **claim),
                         blocks=[{'name': 'j5', 'refs': ['J5'], 'rotation_candidates': [0, 90]}])
        res = self.stage1(SPLIT, doc, seed_refs={'J5'})
        with input_rotation():
            base = self.stage1(SPLIT, doc, seed_refs={'J5'})
        self.assertEqual(self.pose(res, 'J5'), self.pose(base, 'J5'))
        self.assertEqual(seeder._stage1_geometry_rot(
            type('P', (), {'rot': 180.0})(), (None, (0.0, 90.0))), 180.0)


ISSUE_J1 = ('  (footprint "t" (layer "F.Cu") (at 14.15 9.0 270)\n'
            '    (property "Reference" "J1")\n'
            '    (pad "1" smd oval (at 2.75 -1.41) (size 0.69 1.03) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.72 -0.09) (size .5 .5) (layers "F.Cu")))\n')
ALONG = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': 1.5},
         'along_edge_band': {'from': 0.89, 'to': 0.92}}


def r9(bx, by):
    """#983's blocker: a locked two-pad part with a courtyard (test_975's R9)."""
    return (f'  (footprint "r" (locked yes) (layer "F.Cu") (at {bx} {by} 0)\n'
            '    (property "Reference" "R9")\n'
            '    (fp_rect (start -1 -0.6) (end 1 0.6) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))\n'
            '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu")))\n')


def board(size, *footprints):
    return ('(kicad_pcb (version 20241229) (generator "t983")\n'
            f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (layer "Edge.Cuts"))\n'
            + ''.join(footprints) + ')\n')


def key(pads=0, worst=0.0, overlap=None):
    """A `_seat_reading` for a unit test: floor and overlap, no grader."""
    return (pads, worst, dict(overlap or {}), None, None)


def no_nudge():
    """The blind twin of A: every rung is written where the clamp put it."""
    return patch.object(seeder, '_window_nudge',
                        lambda st, part, e, edge, x, y, seats=None, origin=None: (x, y))


class AlongEdgeWindow(_Graded):
    def seat(self, path, entry, blind=False, **kw):
        import pose_score
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        notes = []
        if blind:
            with no_nudge():
                ok = seeder._seat_edge(st, entry['ref'], dict(entry), set(), notes, **kw)
        else:
            ok = seeder._seat_edge(st, entry['ref'], dict(entry), set(), notes, **kw)
        p = st.parts[entry['ref']]
        return ok, (p.x, p.y, p.rot), notes

    def issue_board(self, bx, by):
        return self.write(f'issue_{bx}_{by}.kicad_pcb', board((28.3, 18.0), ISSUE_J1, r9(bx, by)))

    def test_a1_the_issues_two_positions_are_written_inside_the_window(self):
        # #983's own rows: the blind pose is the issue's, to the micron, and
        # outside the window; the stepped one is inside it at both ends.
        for (bx, by), issue_pose in (((0.5, 14.6), (0.468, 14.693, 270.0)),
                                     ((0.5, 15.65), (-0.04, 14.152, 270.0))):
            with self.subTest(blocker=(bx, by)):
                path = self.issue_board(bx, by)
                ok, blind, _ = self.seat(path, ALONG, blind=True, target=(2.18, -0.24))
                self.assertTrue(ok)
                self.assertEqual(blind, issue_pose)
                before = self.graded(path, {'J1': blind}, intent_doc(ALONG))['J1']
                self.assertIn('along_edge', before)
                ok, pose, notes = self.seat(path, ALONG, target=(2.18, -0.24))
                self.assertTrue(ok)
                after = self.graded(path, {'J1': pose}, intent_doc(ALONG))['J1']
                self.assertNotIn('along_edge', after)
                # Nothing traded for it. (14.6's pose also sits nearest the
                # SOUTH edge at 92% of the west one; that error is the blind
                # seat's too, and not this issue's.)
                self.assertTrue(all(after.count(k) <= before.count(k) for k in after),
                                (before, after))
                self.assertFalse([n for n in notes if 'written outside' in n])

    def test_a2_stage_one_and_a_lattice_slice(self):
        # Every third blocker y at x 0.5, both ladders: the blind arm must be
        # dirty somewhere and the fixed one nowhere. A6, bit-identity: a seat
        # differs from the blind one only when a rung was actually STEPPED --
        # not "only when the blind seat was dirty", because a stepped rung can
        # now pass the #975 move's own window check and win where the blind
        # ladder walked on (measured here: (0.198, 14.422) -> (0.711, 14.153)).
        # This arm calls `_seat_edge` BARE, as #983's recipe did, so the #975
        # move is not asked `_grade_worse`; through `repair_placement` it is,
        # and A10 pins that path.
        dirty_blind = {'seat': 0, 'stage1': 0}
        fired = []
        real = seeder._window_nudge

        def spy(st, part, e, edge, x, y, seats=None, origin=None):
            out = real(st, part, e, edge, x, y, seats, origin)
            fired.append(out != (x, y))
            return out
        for k in range(0, 120, 3):
            by = round(12.0 + 0.05 * k, 2)
            path = self.issue_board(0.5, by)
            doc = intent_doc(ALONG)
            for ladder in ('seat', 'stage1'):
                with self.subTest(by=by, ladder=ladder):
                    del fired[:]
                    if ladder == 'seat':
                        ok_b, blind, _ = self.seat(path, ALONG, blind=True, target=(2.18, -0.24))
                        with patch.object(seeder, '_window_nudge', spy):
                            ok, pose, _ = self.seat(path, ALONG, target=(2.18, -0.24))
                    else:
                        with no_nudge():
                            rb = self.stage1(path, doc)
                        with patch.object(seeder, '_window_nudge', spy):
                            r = self.stage1(path, doc)
                        blind, pose = self.pose(rb, 'J1'), self.pose(r, 'J1')
                        ok_b = ok = True
                    self.assertEqual(ok, ok_b)
                    if not ok:
                        continue
                    gb = self.graded(path, {'J1': blind}, doc)['J1']
                    g = self.graded(path, {'J1': pose}, doc)['J1']
                    self.assertNotIn('along_edge', g)
                    dirty_blind[ladder] += 'along_edge' in gb
                    if not any(fired):
                        self.assertEqual(pose, blind)
                    if 'along_edge' in gb:
                        self.assertTrue(any(fired))
        self.assertGreaterEqual(dirty_blind['seat'], 8, dirty_blind)
        self.assertGreaterEqual(dirty_blind['stage1'], 4, dirty_blind)

    #: An ASYMMETRIC courtyard whose centre offsets have a 4th decimal of 2 or
    #: 8 at every rotation, so a window end converted to an origin is never on
    #: the 1 um grid and never on a rounding tie.
    ASYM = ('  (footprint "t" (layer "F.Cu") (at 14.15 9.0 0)\n'
            '    (property "Reference" "J1")\n'
            '    (fp_rect (start -0.5004 -2.3006) (end 3.1 0.7) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at 1.3 -0.8) (size .5 .5) (layers "F.Cu"))))\n')

    def unit_state(self, path, rot):
        import pose_score
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        part = st.parts['J1']
        seeder._materialise_rotation(part, rot)
        part.rot = rot
        return st, part

    def end_poses(self, st, part, entry, edge='west', overhang=0.75):
        """The two rungs clamped to the declared window's ends."""
        e_lo, e_hi, _ = seeder._declared_edge_span(st, st.board, edge)
        win = seeder._declared_frac_window(entry, e_hi - e_lo)
        to = lambda f: seeder.declared_to_ladder_frac(part, st.board, edge, e_lo, e_hi, f)
        return [seeder._edge_pose(part, st.board, edge, to(f), overhang) for f in win]

    def test_a3_both_ends_at_every_rotation_on_an_asymmetric_part(self):
        # A mid-edge window, so the part's own extents cut neither end at any
        # rotation. At each rotation exactly one end rounds OUT (anti-vacuity),
        # and it -- only it -- is stepped one grid unit along the edge, inward.
        mid = dict(ALONG, along_edge_band={'from': 0.45, 'to': 0.55})
        path = self.write('asym.kicad_pcb', board((28.3, 18.0), self.ASYM))
        for rot in (0.0, 90.0, 180.0, 270.0):
            with self.subTest(rot=rot):
                st, part = self.unit_state(path, rot)
                flagged = 0
                for x, y in self.end_poses(st, part, mid):
                    out = seeder._outside_its_along_edge_claim(st, part, mid, 'west', x, y)
                    flagged += out
                    nx, ny = seeder._window_nudge(st, part, mid, 'west', x, y)
                    self.assertFalse(seeder._outside_its_along_edge_claim(
                        st, part, mid, 'west', nx, ny), (rot, y, ny))
                    self.assertEqual(nx, x)                     # the normal axis untouched
                    if out:
                        self.assertAlmostEqual(abs(ny - round(y, 3)), 0.001, delta=1e-9)
                        # inward: towards the origin that centres the part in
                        # the window (courtyard centre 9.0 on this 18 mm edge)
                        r = part.rect(0.0, 0.0, rot)
                        home = 9.0 - (r[1] + r[3]) / 2.0
                        self.assertLess(abs(ny - home), abs(round(y, 3) - home))
                    else:
                        self.assertEqual(ny, y)                 # bit-identical
                self.assertEqual(flagged, 1, rot)

    def test_a4_a_window_narrower_than_the_grid_is_never_refused(self):
        path = self.issue_board(6.0, 12.0)
        entry = dict(ALONG, along_edge_band={'from': 0.5, 'to': 0.50001})
        for ladder in ('seat', 'stage1'):
            with self.subTest(ladder=ladder):
                if ladder == 'seat':
                    ok, pose, notes = self.seat(path, entry, target=(2.18, -0.24))
                    self.assertTrue(ok, notes)
                else:
                    res = self.stage1(path, intent_doc(entry))
                    notes, pose = res['notes'], self.pose(res, 'J1')
                self.assertFalse([n for n in notes if 'does not intersect' in n], notes)
                errs = self.graded(path, {'J1': pose}, intent_doc(entry))['J1']
                named = [n for n in notes if 'written outside its declared along-edge' in n]
                # Either the grid met it, or the seat says it did not.
                self.assertEqual('along_edge' in errs, bool(named), (errs, notes))

    def test_a4b_a_step_that_would_cost_the_seat_or_the_floor_is_not_taken(self):
        path = self.write('asym_b.kicad_pcb', board((28.3, 18.0), self.ASYM))
        mid = dict(ALONG, along_edge_band={'from': 0.45, 'to': 0.55})
        st, part = self.unit_state(path, 0.0)
        (x, y), = [p for p in self.end_poses(st, part, mid)
                   if seeder._outside_its_along_edge_claim(st, part, mid, 'west', *p)]
        stepped = seeder._window_nudge(st, part, mid, 'west', x, y)
        self.assertNotEqual(stepped, (x, y))
        # the stepped pose is not a seat, the raw one is: keep the raw one
        seat_only_raw = lambda sx, sy: key() if (sx, sy) == (x, y) else None
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, seat_only_raw), (x, y))
        # both seat, the stepped one deeper under the floor: keep the raw one
        deeper = lambda sx, sy: key(1, 0.682) if (sx, sy) == (x, y) else key(1, 0.683)
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, deeper), (x, y))
        # Each floor count on its own (not a lexicographic key): one more pad
        # short at the same worst, or the same pads with a deeper worst.
        more = lambda sx, sy: key(1, 0.3) if (sx, sy) == (x, y) else key(2, 0.3)
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, more), (x, y))
        fewer_deeper = lambda sx, sy: key(2, 0.3) if (sx, sy) == (x, y) else key(1, 0.4)
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, fewer_deeper), (x, y))
        # Courtyard overlap: none where there was none; an existing one may
        # deepen by the step's own micron.
        def ov(raw_pairs, new_pairs):
            return lambda sx, sy: (key(overlap=raw_pairs) if (sx, sy) == (x, y)
                                   else key(overlap=new_pairs))
        nudge = lambda f: seeder._window_nudge(st, part, mid, 'west', x, y, f)
        self.assertEqual(nudge(ov({}, {'R9': 0.0012})), (x, y))              # new
        self.assertEqual(nudge(ov({'R9': 0.0441}, {'R9': 0.0453})), stepped) # deeper
        # PER PAIR: an overlap R8 already has does not license a new one with R9.
        # (R8's 0.004 is over half R9's new 0.0072, so a SUM of the two would
        # not be refused by the doubling clause either: only per pair is.)
        self.assertEqual(nudge(ov({'R8': 0.004}, {'R8': 0.004, 'R9': 0.0072})), (x, y))
        # "Existing" means the grade would report it, and it may not double.
        self.assertEqual(nudge(ov({'R9': 1.2e-5}, {'R9': 0.0132})), (x, y))
        self.assertEqual(nudge(ov({'R9': 3e-5}, {'R9': 5.9e-5})), (x, y))    # under double,
        #                                                   but never reported before
        self.assertEqual(nudge(ov({'R9': 0.01}, {'R9': 0.0201})), (x, y))
        self.assertEqual(nudge(ov({'R9': 0.01}, {'R9': 0.0199})), stepped)
        self.assertEqual(nudge(ov({'R9': 0.01}, {'R9': 0.02})), (x, y))       # exactly double
        # The threshold, from both sides: 6e-5 is reported and may deepen;
        # 4.95e-5 is not, and may not creep past it within the 1e-6 slack.
        self.assertEqual(nudge(ov({'R9': 6e-5}, {'R9': 1.1e-4})), stepped)
        self.assertEqual(nudge(ov({'R9': 5e-5}, {'R9': 9e-5})), stepped)     # at it: reported
        self.assertEqual(nudge(ov({'R9': 4.95e-5}, {'R9': 5.04e-5})), (x, y))
        self.assertEqual(nudge(ov({'R9': 1e-5}, {'R9': 1.9e-5})), (x, y))    # unreported: no growth
        # A pair that does not change leaves the step free, reported or not.
        self.assertEqual(nudge(ov({'R9': 3e-5}, {'R9': 3e-5})), stepped)
        self.assertEqual(nudge(ov({'R9': 0.2}, {'R9': 0.2})), stepped)
        # One bound per RUNG: judged against `origin`, the rung before a band
        # settle moved it. The round-3 compound -- 6e-5 at the rung, 1.15e-4
        # settled, 1.38e-4 stepped -- is under double of the settled pose and
        # 2.3x of the rung.
        origin = (round(x - 0.011, 3), y)
        readings = {origin: key(overlap={'R9': 6e-5}), (x, y): key(overlap={'R9': 1.15e-4})}
        comp = lambda sx, sy: readings.get((sx, sy), key(overlap={'R9': 1.38e-4}))
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, comp), stepped)
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, comp, origin), (x, y))
        # the raw one does not seat at all: a step never makes a seat of it
        only_step = lambda sx, sy: None if (sx, sy) == (x, y) else key()
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, only_step), (x, y))
        # equal floors: taken
        same = lambda sx, sy: key(1, 0.5)
        self.assertEqual(seeder._window_nudge(st, part, mid, 'west', x, y, same), stepped)

    def test_a4c_a_step_that_raises_keeps_the_seat(self):
        # The issue's two rows with the floor unreadable at every pose: the
        # seat is kept where the ladder had it (and graded as #983 found it),
        # rather than the error escaping the seat and unseating the part.
        for (bx, by), issue_pose in (((0.5, 14.6), (0.468, 14.693, 270.0)),
                                     ((0.5, 15.65), (-0.04, 14.152, 270.0))):
            with self.subTest(blocker=(bx, by)):
                path = self.issue_board(bx, by)
                real = seeder._window_step

                def boom(*a, **k):
                    raise RuntimeError('boom')
                with patch.object(seeder, '_window_step', boom):
                    ok, pose, _ = self.seat(path, ALONG, target=(2.18, -0.24))
                self.assertTrue(ok)
                self.assertIs(seeder._window_step, real)
                self.assertEqual(pose, issue_pose)

    def test_a10_the_issue_row_through_the_production_caller(self):
        # (0.5, 14.6): J1 already overlaps R9's courtyard by 0.0441 mm2 at
        # the window end, and every in-window pose overlaps at least that.
        # With no budget the step is taken -- the overlap deepens by the
        # step's own micron, disclosed -- and the along-edge error is gone.
        path = self.issue_board(0.5, 14.6)
        doc = intent_doc(ALONG)
        blind = self.repair(path, doc, blind=True)
        pose = self.repair(path, doc)
        bk, _bl, b_ov = self.full_grade(path, blind, doc)
        k, legal, ov = self.full_grade(path, pose, doc)
        self.assertIn('along_edge', bk)
        self.assertNotIn('along_edge', k)
        self.assertGreater(b_ov, 0.04)
        self.assertLess(ov - b_ov, 0.003)
        # A declared budget the deeper overlap would cross is a grade error the
        # step may not add: refused, and the pose is the blind one.
        tight = dict(doc, legality_budget={'overlap_area': round(b_ov + 0.0005, 4)})
        self.assertEqual(self.repair(path, tight), self.repair(path, tight, blind=True))
        loose = dict(doc, legality_budget={'overlap_area': round(b_ov + 0.01, 4)})
        self.assertNotEqual(self.repair(path, loose), self.repair(path, loose, blind=True))

    def test_a5_a_zero_tolerance_centre_is_met_on_grid_and_named_off_it(self):
        centre = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': 1.5},
                  'center_on_edge': {'tolerance_mm': 0.0}}
        # On the grid: a courtyard symmetric about its origin puts the centre
        # on 9.000 exactly.
        sym = ('  (footprint "t" (layer "F.Cu") (at 14.15 9.0 0)\n'
               '    (property "Reference" "J1")\n'
               '    (fp_rect (start -1 -1.5) (end 1 1.5) (layer "F.CrtYd"))\n'
               '    (pad "1" smd rect (at 0 0) (size .5 .5) (layers "F.Cu"))))\n')
        for name, fp, want_clean in (('sym', sym, True), ('asym', self.ASYM, False)):
            path = self.write(f'tol0_{name}.kicad_pcb', board((28.3, 18.0), fp))
            for ladder in ('seat', 'stage1'):
                with self.subTest(fixture=name, ladder=ladder):
                    if ladder == 'seat':
                        ok, pose, notes = self.seat(path, centre, target=(2.18, -0.24))
                        self.assertTrue(ok, notes)
                    else:
                        res = self.stage1(path, intent_doc(centre))
                        notes, pose = res['notes'], self.pose(res, 'J1')
                    errs = self.graded(path, {'J1': pose}, intent_doc(centre))['J1']
                    named = [n for n in notes if 'written outside its declared along-edge' in n]
                    if want_clean:
                        self.assertNotIn('along_edge', errs)
                        self.assertEqual(named, [])
                    else:
                        # Unavoidable on a 1 um grid: seated, graded, and SAID
                        # -- and not moved, since a step that cannot land
                        # inside the window buys nothing.
                        self.assertIn('along_edge', errs)
                        self.assertEqual(len(named), 1, notes)
                        if ladder == 'seat':
                            _, blind, _ = self.seat(path, centre, blind=True,
                                                    target=(2.18, -0.24))
                        else:
                            with no_nudge():
                                blind = self.pose(self.stage1(path, intent_doc(centre)), 'J1')
                        self.assertEqual(pose, blind)

    def test_a8_a_centre_claim_window_end(self):
        centre = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': 1.5},
                  'center_on_edge': {'tolerance_mm': 0.5}}
        path = self.issue_board(0.5, 7.5)
        ok, blind, _ = self.seat(path, centre, blind=True, target=(2.18, -0.24))
        self.assertIn('along_edge', self.graded(path, {'J1': blind}, intent_doc(centre))['J1'])
        ok, pose, _ = self.seat(path, centre, target=(2.18, -0.24))
        self.assertTrue(ok)
        self.assertEqual(self.graded(path, {'J1': pose}, intent_doc(centre))['J1'], [])

    def test_a9_the_step_is_read_on_the_edge_span_of_a_notched_outline(self):
        # interf_u's south edge is not its bounding box, and a declared
        # fraction is a fraction of the EDGE: a window end converted on it,
        # at every rotation, is stepped inside (or was inside already).
        import pose_score
        path = str(ROOT / 'kicad_files' / 'interf_u_unrouted_placed.kicad_pcb')
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        e_lo, e_hi, _ = seeder._declared_edge_span(st, st.board, 'south')
        x0, _, x1, _ = st.board
        self.assertGreater(abs((e_hi - e_lo) - (x1 - x0)), 1.0)
        ref = sorted(r for r, p in st.parts.items()
                     if p.rect(0, 0, p.rot)[2] - p.rect(0, 0, p.rot)[0] > 1)[0]
        part = st.parts[ref]
        # A window whose ends convert off the grid for this part.
        entry = {'ref': ref, 'edge': 'south', 'overhang_mm': {'min': 0, 'max': 1},
                 'along_edge_band': {'from': 0.30037, 'to': 0.60041}}
        stepped = 0
        for rot in (0.0, 90.0, 180.0, 270.0):
            with self.subTest(rot=rot):
                seeder._materialise_rotation(part, rot)
                part.rot = rot
                for x, y in self.end_poses(st, part, entry, edge='south'):
                    nx, ny = seeder._window_nudge(st, part, entry, 'south', x, y)
                    self.assertFalse(seeder._outside_its_along_edge_claim(
                        st, part, entry, 'south', nx, ny), (rot, x, nx))
                    self.assertEqual(ny, y)
                    stepped += (nx, ny) != (x, y)
        self.assertGreaterEqual(stepped, 2)


BODY = '(fp_rect (start -3 -1) (end 1 1) (layer "F.Fab"))'
PADS3 = ('(pad "1" smd rect (at -1.905 -0.5) (size .5 .5) (layers "F.Cu"))',
         '(pad "2" smd rect (at -1.905 0.5) (size .5 .5) (layers "F.Cu"))',
         '(pad "3" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))')


def j1(graphics, pads=PADS3, at='10 10 0'):
    return (f'  (footprint "t" (layer "F.Cu") (at {at})\n'
            '    (property "Reference" "J1")\n'
            + ''.join(f'    {g}\n' for g in graphics)
            + ''.join(f'    {p}\n' for p in pads) + '  )\n')


def no_settle():
    """The blind twin of B: every rung is written where the walk left it."""
    return patch.object(seeder, '_band_settle',
                        lambda st, part, e, edge, lo, x, y, seats=None: (x, y))


class OverhangBand(_Graded):
    def both(self, path, entry, clearance=.25, edge=.55):
        """{ladder: (blind pose, blind kinds, pose, kinds, notes)}."""
        import pose_score
        doc = intent_doc(entry)
        out = {}
        for ladder in ('seat', 'stage1'):
            poses = []
            for blind in (True, False):
                if ladder == 'seat':
                    st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=clearance,
                                               board_edge_clearance=edge)
                    notes = []
                    if blind:
                        with no_settle():
                            ok = seeder._seat_edge(st, 'J1', dict(entry), set(), notes,
                                                   target=(0.0, 10.0))
                    else:
                        ok = seeder._seat_edge(st, 'J1', dict(entry), set(), notes,
                                               target=(0.0, 10.0))
                    self.assertTrue(ok, notes)
                    p = st.parts['J1']
                    pose = (p.x, p.y, p.rot)
                else:
                    if blind:
                        with no_settle():
                            res = self.stage1(path, doc, clearance, edge)
                    else:
                        res = self.stage1(path, doc, clearance, edge)
                    pose, notes = self.pose(res, 'J1'), res['notes']
                poses.append((pose, self.graded(path, {'J1': pose}, doc, clearance, edge)['J1'],
                              notes))
            (bp, bk, _), (fp, fk, notes) = poses
            out[ladder] = (bp, bk, fp, fk, notes)
        return out

    def floor(self, path, pose, edge=.55):
        from placement import legality
        out = str(self.root / f'fl_{abs(hash((path, pose)))}.kicad_pcb')
        write_placed_output(path, out, [{'reference': 'J1', 'new_x': round(pose[0], 3),
                                         'new_y': round(pose[1], 3), 'new_rotation': pose[2]}])
        g = legality.grade_pad_edge_clearance(parse_kicad_pcb(out), edge, out)
        short = [f['shortfall_mm'] for f in g['findings'] if f['pad_ref'].startswith('J1.')]
        return (len(short), round(max(short, default=0.0), 6))

    def assert_settled(self, path, entry, **kw):
        """Blind dirty on the band, fixed clean on it, nothing traded, the
        along-edge coordinate untouched, the floor no shorter."""
        for ladder, (bp, bk, fp, fk, _notes) in self.both(path, entry, **kw).items():
            with self.subTest(ladder=ladder):
                self.assertIn('band', bk, (ladder, bp))
                self.assertNotIn('band', fk, (ladder, fp))
                self.assertTrue(all(fk.count(k) <= bk.count(k) for k in fk), (bk, fk))
                self.assertEqual(fp[1], bp[1])
                self.assertLessEqual(abs(fp[0] - bp[0]), seeder._BAND_SETTLE_CAP_MM + 1e-9)
                edge = kw.get('edge', .55)
                self.assertLessEqual(self.floor(path, fp, edge), self.floor(path, bp, edge))

    def test_b1_a_drawn_body_past_its_courtyard_both_band_ends(self):
        # The walk lands the courtyard on the target, so the body reads
        # target - margin + (how far it reaches past the courtyard): a body
        # 0.4918 past it reads 0.24 on {0.25, 0.35}, one 0.6601 past it
        # reads 0.51 on {0.3, 0.5}.
        for band, cl in (({'min': 0.25, 'max': 0.35}, -2.5082),
                         ({'min': 0.3, 'max': 0.5}, -2.3399),
                         ({'min': 0.0, 'max': 0.02}, -2.4317)):
            with self.subTest(band=band, cl=cl):
                crt = f'(fp_rect (start {cl} -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
                path = self.write(f'b1_{cl}.kicad_pcb', board((20, 20), j1((BODY, crt))))
                self.assert_settled(path, {'ref': 'J1', 'edge': 'west', 'overhang_mm': band})

    def test_b2_no_declared_maximum_and_the_target_on_the_minimum(self):
        g = '(fp_rect (start -4.9996 -1) (end 1 1) (layer "F.Fab"))'
        path = self.write('b2.kicad_pcb', board((20, 20), j1((g,))))
        self.assert_settled(path, {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.6}})

    def test_b2b_a_south_edge_moves_along_y_both_ways(self):
        # Every other settle arm is on the west edge, where the normal is x;
        # here it is y. The body reaches 0.4918 / 0.6601 past the courtyard's
        # SOUTH side, as B1's west-edge pair does past its west side.
        for band, cb in (({'min': 0.25, 'max': 0.35}, 2.5082),
                         ({'min': 0.3, 'max': 0.5}, 2.3399)):
            with self.subTest(band=band):
                g = '(fp_rect (start -1 -1) (end 1 3) (layer "F.Fab"))'
                crt = f'(fp_rect (start -1.1 -1.1) (end 1.1 {cb}) (layer "F.CrtYd"))'
                pads = ('(pad "1" smd rect (at -0.5 1.905) (size .5 .5) (layers "F.Cu"))',
                        '(pad "2" smd rect (at 0.5 1.905) (size .5 .5) (layers "F.Cu"))',
                        '(pad "3" smd rect (at 0 -0.5) (size .5 .5) (layers "F.Cu"))')
                path = self.write(f'b2b_{cb}.kicad_pcb', board((20, 20), j1((g, crt), pads)))
                entry = {'ref': 'J1', 'edge': 'south', 'overhang_mm': band}
                for ladder, (bp, bk, fp, fk, _n) in self.both(path, entry).items():
                    with self.subTest(ladder=ladder):
                        self.assertIn('band', bk, (ladder, bp))
                        self.assertNotIn('band', fk, (ladder, fp))
                        self.assertEqual(fp[0], bp[0])            # x untouched
                        self.assertNotEqual(fp[1], bp[1])

    def test_b3_a_gate_margin_under_the_walks_tolerance(self):
        crt = '(fp_rect (start -2.3 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        path = self.write('b3.kicad_pcb', board((20, 20), j1((crt,))))
        self.assert_settled(path, {'ref': 'J1', 'edge': 'west',
                                   'overhang_mm': {'min': 0.0, 'max': 0.015}},
                            clearance=0.01, edge=0.01)

    def test_b4_a_settle_that_would_trade_keeps_the_raw_pose(self):
        cases = []
        # The floor: 0.01 mm out to meet 0.3 would put J1.1/J1.2 inside it.
        crt = '(fp_rect (start -2.56 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        cases.append(('floor', board((20, 20), j1((BODY, crt))),
                      {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.3, 'max': 0.5}}))
        # The setback: in to meet {0, 0} leaves no overhang on a receptacle.
        rpads = ('(pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))',
                 '(pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu"))')
        rcrt = '(fp_rect (start -1.5 -1.0004) (end 1.5 1) (layer "F.CrtYd"))'
        cases.append(('setback', board((20, 20), j1((rcrt,), rpads)),
                      {'ref': 'J1', 'edge': 'north', 'overhang_mm': {'min': 0.0, 'max': 0.0},
                       'class': 'edge_receptacle'}))
        for name, text, entry in cases:
            path = self.write(f'b4_{name}.kicad_pcb', text)
            for ladder, (bp, bk, fp, fk, _n) in self.both(path, entry).items():
                with self.subTest(case=name, ladder=ladder):
                    self.assertIn('band', bk)                  # the fixture is broken
                    self.assertEqual(fp, bp)                   # and stays as it was
                    self.assertEqual(fk, bk)

    def test_b4b_the_cap_and_a_raise_keep_the_raw_pose(self):
        import pose_score
        # A body set back 0.5 mm inside a 0.01 minimum: the reading is 0 and
        # the move it needs is far past the cap.
        g = '(fp_rect (start -1.2 -1) (end 1 1) (layer "F.Fab"))'
        crt = '(fp_rect (start -2.4 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        path = self.write('b4b.kicad_pcb', board((20, 20), j1((g, crt))))
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        part = st.parts['J1']
        entry = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.01, 'max': 0.5}}
        x, y = 2.95, 10.0
        amount, _b, _l = seeder._band_reading(st, part, 'west', x, y)
        self.assertLess(amount, 0.01)                          # it IS short
        self.assertEqual(seeder._band_settle(st, part, entry, 'west', 0.01, x, y), (x, y))
        # A rung that is not a seat is never moved, however short it reads:
        # a correction does not make a seat of what the seat refused.
        # x 1.19 puts the body's west end 0.01 past the edge: 0.01 short of a
        # 0.02 minimum, well inside the cap.
        self.assertAlmostEqual(seeder._band_reading(st, part, 'west', 1.19, 10.0)[0], 0.01,
                               delta=1e-6)
        c2 = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.02, 'max': 0.5}}
        self.assertNotEqual(seeder._band_settle(st, part, c2, 'west', 0.02, 1.19, 10.0),
                            (1.19, 10.0))                  # it would move ...
        only_moved = lambda sx, sy: None if (sx, sy) == (1.19, 10.0) else key()
        self.assertEqual(seeder._band_settle(st, part, c2, 'west', 0.02, 1.19, 10.0,
                                             only_moved), (1.19, 10.0))
        # The cap binds on its own: a body 0.015 inside the edge against a
        # 0.01 minimum reads 0 twice (clipped), then 0.007, and would reach
        # the band only at 0.026 mm of travel -- past the 0.022 cap.
        self.assertEqual(seeder._band_reading(st, part, 'west', 1.215, 10.0)[0], 0.0)
        c3 = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.01, 'max': 0.5}}
        self.assertEqual(seeder._band_settle(st, part, c3, 'west', 0.01, 1.215, 10.0),
                         (1.215, 10.0))
        with patch.object(seeder, '_BAND_SETTLE_CAP_MM', 1.0):
            self.assertNotEqual(seeder._band_settle(st, part, c3, 'west', 0.01, 1.215, 10.0),
                                (1.215, 10.0))
        # And anything the reading raises leaves the pose as it was.
        with patch.object(seeder, '_band_reading', side_effect=RuntimeError('boom')):
            self.assertEqual(seeder._band_settle(st, part, entry, 'west', 0.01, x, y), (x, y))

    def test_b7_a_settle_that_would_buy_new_overlap_keeps_the_raw_pose(self):
        # The pre-push review's blocker: body 0.6601 past the courtyard reads
        # 0.51 on {0.3, 0.5}, and settling it 0.011 mm inward would push its
        # courtyard 6 um into a LOCKED R9 sitting 5 um clear of it -- a new
        # overlap, and a `legality` error under a zero budget.
        crt = '(fp_rect (start -2.3399 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        r9_at = ('  (footprint "r" (locked yes) (layer "F.Cu") (at 4.595 10 0)\n'
                 '    (property "Reference" "R9")\n'
                 '    (fp_rect (start -1 -0.6) (end 1 0.6) (layer "F.CrtYd"))\n'
                 '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))\n'
                 '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu")))\n')
        path = self.write('b7.kicad_pcb', board((20, 20), j1((BODY, crt)), r9_at))
        entry = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.3, 'max': 0.5}}
        for budget in (None, 0.0):
            doc = intent_doc(entry)
            if budget is not None:
                doc['legality_budget'] = {'overlap_area': budget}
            with self.subTest(budget=budget, caller='repair'):
                blind, pose = self.repair(path, doc, blind=True), self.repair(path, doc)
                self.assertIn('band', self.full_grade(path, blind, doc)[0])
                self.assertEqual(pose, blind)
                if budget is not None:          # the rule only runs under one
                    self.assertEqual(self.full_grade(path, pose, doc)[1], [])
            with self.subTest(budget=budget, caller='stage1'):
                with no_settle():
                    blind = self.pose(self.stage1(path, doc), 'J1')
                pose = self.pose(self.stage1(path, doc), 'J1')
                self.assertEqual(pose, blind)
                if budget is not None:
                    self.assertEqual(self.full_grade(path, pose, doc)[1], [])
        # A locked R8 whose courtyard already overlaps J1's top by 4 um
        # (0.008 mm2, more than half R9's new 0.0072, so the doubling clause
        # alone would not refuse a SUM) must not license the new overlap with
        # R9 (the round-2 re-review's case).
        r8_at = ('  (footprint "r" (locked yes) (layer "F.Cu") (at 1.49 7.904 0)\n'
                 '    (property "Reference" "R8")\n'
                 '    (fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd"))\n'
                 '    (pad "1" smd rect (at -0.5 -0.6) (size .3 .3) (layers "F.Cu"))\n'
                 '    (pad "2" smd rect (at 0.5 -0.6) (size .3 .3) (layers "F.Cu")))\n')
        both = self.write('b7_r8.kicad_pcb', board((20, 20), j1((BODY, crt)), r9_at, r8_at))
        doc = intent_doc(entry)
        self.assertEqual(self.repair(both, doc), self.repair(both, doc, blind=True))
        # Anti-vacuity: with no neighbour in the way the same rung IS settled.
        alone = self.write('b7_alone.kicad_pcb', board((20, 20), j1((BODY, crt))))
        doc = intent_doc(entry)
        self.assertNotEqual(self.repair(alone, doc), self.repair(alone, doc, blind=True))

    def test_b7c_the_step_is_handed_the_rung_as_found(self):
        # One bound per RUNG (the round-3 review): when a settle moved a rung,
        # the step that follows is judged against the rung BEFORE the settle
        # (A4b pins what that judgement refuses). Pinned here on the wiring, in
        # both ladders: every step right after a settle is handed the settled
        # pose to move and the rung as found as its `origin`.
        crt = '(fp_rect (start -2.3399 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        path = self.write('b7c.kicad_pcb', board((20, 20), j1((BODY, crt))))
        entry = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.3, 'max': 0.5},
                 'along_edge_band': {'from': 0.3, 'to': 0.7}}
        doc = intent_doc(entry)
        real_settle, real_nudge = seeder._band_settle, seeder._window_nudge
        for caller in ('repair', 'stage1'):
            with self.subTest(caller=caller):
                log = []

                def settle(st, part, e, edge, lo, x, y, seats=None):
                    out = real_settle(st, part, e, edge, lo, x, y, seats)
                    log.append(('settle', (x, y), tuple(out)))
                    return out

                def nudge(st, part, e, edge, x, y, seats=None, origin=None):
                    log.append(('nudge', (x, y), origin))
                    return real_nudge(st, part, e, edge, x, y, seats, origin)
                with patch.object(seeder, '_band_settle', settle), \
                        patch.object(seeder, '_window_nudge', nudge):
                    if caller == 'repair':
                        self.repair(path, doc)
                    else:
                        self.stage1(path, doc)
                pairs = [(a, b) for a, b in zip(log, log[1:])
                         if a[0] == 'settle' and b[0] == 'nudge']
                self.assertTrue([a for a, _b in pairs if a[2] != a[1]],
                                'a settle must move a rung, or this arm tests nothing')
                for a, b in pairs:
                    self.assertEqual(b[1], a[2])        # it moves the settled pose
                    self.assertEqual(b[2], a[1])        # judged against the rung

    def test_b7b_the_pile_is_not_a_neighbour(self):
        # What the seat ignores as meaningless coordinates -- `_seat_edge`'s
        # `exclude`, stage 1's not-yet-placed parts -- neither refuses a
        # correction nor is graded against it: here an UNLOCKED R7 in stage
        # 1's pile, and the same part handed to `_seat_edge` as `exclude`,
        # sits where the settled pose would overlap it.
        import pose_score
        crt = '(fp_rect (start -2.3399 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        r7 = ('  (footprint "r" (layer "F.Cu") (at 4.595 10 0)\n'
              '    (property "Reference" "R7")\n'
              '    (fp_rect (start -1 -0.6) (end 1 0.6) (layer "F.CrtYd"))\n'
              '    (pad "1" smd rect (at -0.5 0) (size .5 .5) (layers "F.Cu"))\n'
              '    (pad "2" smd rect (at 0.5 0) (size .5 .5) (layers "F.Cu")))\n')
        path = self.write('b7b.kicad_pcb', board((20, 20), j1((BODY, crt)), r7))
        entry = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.3, 'max': 0.5}}
        doc = dict(intent_doc(entry), legality_budget={'overlap_area': 0.0})
        with no_settle():
            blind = self.pose(self.stage1(path, doc), 'J1')
        self.assertNotEqual(self.pose(self.stage1(path, doc), 'J1'), blind)
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        notes = []
        self.assertTrue(seeder._seat_edge(st, 'J1', dict(entry), set(), notes,
                                          exclude={'R7'}, target=(0.0, 10.0)))
        self.assertNotEqual(round(st.parts['J1'].x, 3), round(blind[0], 3))

    def test_b6b_no_declared_maximum_means_no_upper_bound(self):
        # The seat's own `hi_eff` for a band with no `max` is max(2T, lo + 1);
        # the grade has no upper bound at all, so neither may the settle.
        import pose_score
        path = self.write('b6b.kicad_pcb', board((20, 20), j1((BODY,))))
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        part = st.parts['J1']
        x, y = 1.99, 10.0                                  # the body reads 1.01
        self.assertAlmostEqual(seeder._band_reading(st, part, 'west', x, y)[0], 1.01, delta=1e-6)
        no_max = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.0}}
        self.assertEqual(seeder._band_settle(st, part, no_max, 'west', 0.0, x, y), (x, y))
        capped = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.0, 'max': 1.0}}
        self.assertNotEqual(seeder._band_settle(st, part, capped, 'west', 0.0, x, y), (x, y))

    def test_b6_a_rung_the_grade_accepts_is_untouched(self):
        # Wide bands on the same body: every seat grades clean blind, and the
        # settle must hand every rung back as it came.
        calls = []
        real = seeder._band_settle

        def spy(st, part, e, edge, lo, x, y, seats=None):
            out = real(st, part, e, edge, lo, x, y, seats)
            calls.append(out == (x, y))
            return out
        crt = '(fp_rect (start -2.45 -1.1) (end 1.1 1.1) (layer "F.CrtYd"))'
        path = self.write('b6.kicad_pcb', board((20, 20), j1((BODY, crt))))
        for band in ({'min': 0.0, 'max': 0.8}, {'min': 0.2, 'max': 0.7}):
            with patch.object(seeder, '_band_settle', spy):
                res = self.both(path, {'ref': 'J1', 'edge': 'west', 'overhang_mm': band})
            for ladder, (bp, bk, fp, fk, _n) in res.items():
                self.assertEqual(bk, [])
                self.assertEqual(fp, bp)
        self.assertTrue(calls and all(calls))


if __name__ == '__main__':
    unittest.main(verbosity=2)
