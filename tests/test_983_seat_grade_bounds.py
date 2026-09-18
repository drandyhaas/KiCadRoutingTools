"""#983 and its two siblings: an edge seat writes the pose its own grade accepts.

Each arm seats a connector, WRITES the pose, re-parses the board and runs the
real intent grade (`floorplan.grade`), classifying `edge_connector` errors by
the `measured` key the rule writes rather than by message text. Each arm also
has a blind twin that patches the correction back to identity and must show
the error the correction exists to remove -- so no arm can pass on a fixture
that was never broken.

  C.  STAGE 1 MEASURES AT THE ROTATION IT WRITES. The part's extents, the
      declared start fraction and the declared window are computed at the
      DECLARED rotation, not the input one (`_stage1_geometry_rot`).
      C1  splitflap_driver J5 declared at 0/90/270 lands inside its centre
          claim and its band (was 10.00 / 5.65 / 4.60 mm off centre).
      C2  a part that fits the edge ONLY at its declared rotation is seated
          by stage 1 instead of being refused as wider than the edge.
      C3  a part stage 1 skips keeps its input rotation (the geometry turn
          is a measurement, not a move).
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
                    _, errs, _ = self.seat_j5(claim, rot)
                    self.assertNotIn('along_edge', errs)
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
        seen = []
        real = seeder._stage1_geometry_rot

        def spy(part, claim):
            seen.append(part.rot)
            return real(part, claim)
        with patch.object(seeder, '_stage1_geometry_rot', spy):
            res = self.stage1(SPLIT, doc, seed_refs={'J5'})
        self.assertTrue([n for n in res['notes']
                         if n.startswith('edge connector J5: the declared along-edge window')],
                        res['notes'])
        self.assertFalse([n for n in res['notes']
                          if n.startswith('edge connector J5: seated at the declared rotation')])
        self.assertEqual(seen, [180.0])


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


def no_pull():
    """The blind twin of A: every rung is written where the clamp put it."""
    return patch.object(seeder, '_window_frac',
                        lambda st, part, e, edge, bounds, ov, frac, ends, seats=None: frac)


class AlongEdgeWindow(_Graded):
    def seat(self, path, entry, blind=False, **kw):
        import pose_score
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        notes = []
        if blind:
            with no_pull():
                ok = seeder._seat_edge(st, entry['ref'], dict(entry), set(), notes, **kw)
        else:
            ok = seeder._seat_edge(st, entry['ref'], dict(entry), set(), notes, **kw)
        p = st.parts[entry['ref']]
        return ok, (p.x, p.y, p.rot), notes

    def issue_board(self, bx, by):
        return self.write(f'issue_{bx}_{by}.kicad_pcb', board((28.3, 18.0), ISSUE_J1, r9(bx, by)))

    def test_a1_the_issues_two_positions_are_written_inside_the_window(self):
        # #983's own rows: the blind pose is the issue's, to the micron, and
        # outside the window; the pulled one is inside it at both ends.
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
        # differs from the blind one only when a rung was actually PULLED --
        # not "only when the blind seat was dirty", because a pulled rung can
        # now pass the #975 move's own window check and win where the blind
        # ladder walked on (measured here: (0.198, 14.422) -> (0.711, 14.154)).
        dirty_blind = {'seat': 0, 'stage1': 0}
        fired = []
        real = seeder._window_frac

        def spy(st, part, e, edge, bounds, ov, frac, ends, seats=None):
            out = real(st, part, e, edge, bounds, ov, frac, ends, seats)
            fired.append(out != frac)
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
                        with patch.object(seeder, '_window_frac', spy):
                            ok, pose, _ = self.seat(path, ALONG, target=(2.18, -0.24))
                    else:
                        with no_pull():
                            rb = self.stage1(path, doc)
                        with patch.object(seeder, '_window_frac', spy):
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

    def window_unit(self, path, entry, rot):
        import pose_score
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        part = st.parts['J1']
        seeder._rotated_bounds(part, rot)
        part.rot = rot
        edge = entry['edge']
        e_lo, e_hi, _ = seeder._declared_edge_span(st, st.board, edge)
        win = seeder._declared_frac_window(entry, e_hi - e_lo)
        to = lambda f: seeder.declared_to_ladder_frac(part, st.board, edge, e_lo, e_hi, f)
        f_lo, f_hi = seeder._edge_frac_bounds(part, st.board, edge)
        n_lo, n_hi = max(f_lo, to(win[0])), min(f_hi, to(win[1]))
        ends = seeder._window_ends(part, st.board, edge, e_lo, e_hi, win, n_lo, n_hi)
        return st, part, (n_lo, n_hi), ends

    def test_a3_both_ends_at_every_rotation_on_an_asymmetric_part(self):
        # A mid-edge window, so the part's own extents cut neither end at any
        # rotation (0.89-0.92 loses its far end to them at 90 and 180).
        mid = dict(ALONG, along_edge_band={'from': 0.45, 'to': 0.55})
        path = self.write('asym.kicad_pcb', board((28.3, 18.0), self.ASYM))
        for rot in (0.0, 90.0, 180.0, 270.0):
            with self.subTest(rot=rot):
                st, part, (n_lo, n_hi), ends = self.window_unit(path, mid, rot)
                flagged = 0
                for f in (n_lo, n_hi):
                    x, y = seeder._edge_pose(part, st.board, 'west', f, 0.75)
                    flagged += seeder._outside_its_along_edge_claim(st, part, mid, 'west', x, y)
                    g = seeder._window_frac(st, part, mid, 'west', st.board, 0.75, f, ends)
                    gx, gy = seeder._edge_pose(part, st.board, 'west', g, 0.75)
                    self.assertFalse(seeder._outside_its_along_edge_claim(
                        st, part, mid, 'west', gx, gy), (rot, f, g))
                # Anti-vacuity: at every rotation one end rounds OUT.
                self.assertEqual(flagged, 1, rot)

    def test_a4_a_window_narrower_than_two_guards_is_never_refused(self):
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
        st, part, (n_lo, n_hi), ends = self.window_unit(path, entry, 270.0)
        self.assertEqual(ends, ((n_lo + n_hi) / 2.0,) * 2)

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
                        # Unavoidable on a 1 um grid: seated, graded, and SAID.
                        self.assertIn('along_edge', errs)
                        self.assertEqual(len(named), 1, notes)

    def test_a8_a_centre_claim_window_end(self):
        centre = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0, 'max': 1.5},
                  'center_on_edge': {'tolerance_mm': 0.5}}
        path = self.issue_board(0.5, 7.5)
        ok, blind, _ = self.seat(path, centre, blind=True, target=(2.18, -0.24))
        self.assertIn('along_edge', self.graded(path, {'J1': blind}, intent_doc(centre))['J1'])
        ok, pose, _ = self.seat(path, centre, target=(2.18, -0.24))
        self.assertTrue(ok)
        self.assertEqual(self.graded(path, {'J1': pose}, intent_doc(centre))['J1'], [])

    def test_a9_the_inset_is_one_micron_of_edge_on_a_notched_outline(self):
        import pose_score
        path = str(ROOT / 'kicad_files' / 'interf_u_unrouted_placed.kicad_pcb')
        st = pose_score.make_state(parse_kicad_pcb(path), path, clearance=.25,
                                   board_edge_clearance=.55)
        e_lo, e_hi, _ = seeder._declared_edge_span(st, st.board, 'south')
        x0, _, x1, _ = st.board
        # The point of the arm: the edge's span is not the bounding box's.
        self.assertGreater(abs((e_hi - e_lo) - (x1 - x0)), 1.0)
        part = next(p for p in st.parts.values() if p.rect(0, 0, p.rot)[2] - p.rect(0, 0, p.rot)[0] > 1)
        win = (0.3, 0.6)
        for rot in (0.0, 90.0, 180.0, 270.0):
            with self.subTest(rot=rot):
                seeder._rotated_bounds(part, rot)
                view = seeder._AtRotation(part, rot)
                lo, hi = seeder._window_ends(view, st.board, 'south', e_lo, e_hi, win, -10.0, 10.0)
                span = e_hi - e_lo
                back = lambda f: seeder.ladder_to_declared_frac(view, st.board, 'south', e_lo, e_hi, f)
                self.assertAlmostEqual(back(lo) * span, win[0] * span + 0.001, delta=1e-9)
                self.assertAlmostEqual(back(hi) * span, win[1] * span - 0.001, delta=1e-9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
