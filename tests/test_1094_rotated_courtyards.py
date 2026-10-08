"""#1094: a courtyard is graded as DRAWN, at any rotation.

check_assembly graded a part at 45 degrees as the box around its rotated
courtyard box. On KiCad 10's StickHub demo (39 parts at +-45/+-135 degrees)
that reported 74 courtyard-blocking pairs involving a diagonal part, where
KiCad's own DRC -- with `courtyards_overlap` forced to error -- reports 0, and
six fab CONTAINMENTS against U1 that turned the human board NOT BUILDABLE.

The fix keeps the rects as the broad phase and measures a pair the rects say
overlaps on the drawn outlines (courtyard and .Fab), with the pad copper
united pad by pad. The same measure seats a declared fixed pose (#1054), so
the generator does not refuse what the checker accepts.

The synthetic boards below are sign-independent: two 2x2 squares at 45
degrees are diamonds whatever the rotation convention, so the expected
answer needs no geometry of its own.
"""

import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

#: No subprocess; the StickHub arm reads the KiCad install when present.
RUN_ALL_FAST_OK = True

#: KiCad's own demo. It is CC BY-NC-SA, so it is read from the install and
#: never copied into this MIT repository; its tests skip without it.
STICKHUB_CANDIDATES = (
    os.environ.get('KICAD_STICKHUB_DEMO', ''),
    'C:/Program Files/KiCad/10.0/share/kicad/demos/stickhub/StickHub.kicad_pcb',
    '/usr/share/kicad/demos/stickhub/StickHub.kicad_pcb',
    '/Applications/KiCad/KiCad.app/Contents/SharedSupport/demos/stickhub/'
    'StickHub.kicad_pcb',
)


def stickhub():
    return next((p for p in STICKHUB_CANDIDATES if p and os.path.isfile(p)),
                None)


def _diag(rot):
    m = (rot or 0.0) % 90.0
    return 1e-3 < m < 90.0 - 1e-3


PART = (
    '  (footprint "t:{name}" (layer "F.Cu") (at {x} {y} {rot})\n'
    '    (property "Reference" "{ref}" (at 0 0 {rot}) (layer "F.SilkS"))\n'
    '{court}'
    '    (fp_rect (start -0.9 -0.9) (end 0.9 0.9) (stroke (width 0.05)'
    ' (type default)) (layer "F.Fab"))\n'
    '    (pad "1" smd rect (at 0 0 {rot}) (size 0.4 0.4) (layers "F.Cu")'
    ' (net {net} "N{net}")))\n')
SQUARE = ('    (fp_rect (start -1 -1) (end 1 1) (stroke (width 0.05)'
          ' (type default)) (layer "F.CrtYd"))\n')
CIRCLE = ('    (fp_circle (center 0 0) (end 1 0) (stroke (width 0.05)'
          ' (type default)) (layer "F.CrtYd"))\n')


def board(parts, td):
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n'
            + ''.join(PART.format(name=r, ref=r, x=x, y=y, rot=rot, court=c,
                                  net=i + 1)
                      for i, (r, x, y, rot, c) in enumerate(parts))
            + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def grade(path):
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_body_overlap
    return grade_body_overlap(parse_kicad_pcb(path), 0.1, pcb_file=path,
                              courtyard_severity=None)


def courtyard_pairs(g):
    return {(p.a, p.b) for p in g['pairs'] if p.kind == 'courtyard'}


class TestSynthetic(unittest.TestCase):
    def test_diamonds_clear_on_the_diagonal_do_not_pair(self):
        """Two 2x2 courtyards at 45 degrees, 1.5 mm apart on each axis. As
        diamonds they clear by 0.12 mm; their rotated boxes (+-1.414) overlap
        by 1.33 x 1.33 mm, which is the pair the old grade reported."""
        with tempfile.TemporaryDirectory() as td:
            p = board([('A', 10, 10, 45, SQUARE),
                       ('B', 11.5, 11.5, 45, SQUARE)], td)
            g = grade(p)
        self.assertEqual(courtyard_pairs(g), set(), g['pairs'])

    def test_diamonds_that_touch_do_pair(self):
        """The control: 1.2 mm apart the diamonds really overlap, and the
        pair is still found -- the fix did not stop measuring."""
        with tempfile.TemporaryDirectory() as td:
            p = board([('A', 10, 10, 45, SQUARE),
                       ('B', 11.2, 11.2, 45, SQUARE)], td)
            g = grade(p)
        self.assertEqual(courtyard_pairs(g), {('A', 'B')}, g['pairs'])
        pair = next(q for q in g['pairs'] if q.kind == 'courtyard')
        # In the squares' own frame the offset (1.2, 1.2) is 1.2*sqrt2 along
        # one side and 0 along the other, so the overlap is a
        # (2 - 1.2*sqrt2) x 2 rectangle, and its short side is the depth.
        self.assertAlmostEqual(pair.area_mm2, (2 - 1.2 * 2 ** 0.5) * 2,
                               places=3)
        self.assertAlmostEqual(pair.depth_mm, 2 - 1.2 * 2 ** 0.5, places=3)

    def test_a_round_courtyard_stays_round(self):
        """r=1 circles 2.1 mm apart at 45 degrees: the old reader kept a
        circle's bbox corners, which rotate out to 1.414."""
        with tempfile.TemporaryDirectory() as td:
            p = board([('A', 10, 10, 45, CIRCLE),
                       ('B', 12.1, 10, 45, CIRCLE)], td)
            g = grade(p)
        self.assertEqual(courtyard_pairs(g), set(), g['pairs'])

    def test_an_axis_aligned_pair_measures_as_before(self):
        """At 0 degrees the drawn square IS its box: same area, same depth."""
        with tempfile.TemporaryDirectory() as td:
            p = board([('A', 10, 10, 0, SQUARE),
                       ('B', 11.5, 10.5, 0, SQUARE)], td)
            g = grade(p)
        pair = next(q for q in g['pairs'] if q.kind == 'courtyard')
        self.assertAlmostEqual(pair.area_mm2, 0.5 * 1.5, places=4)
        self.assertAlmostEqual(pair.depth_mm, 0.5, places=4)

    def test_fab_bodies_clear_on_the_diagonal_do_not_contain(self):
        """The fab channel: 1.8 mm bodies at 45 degrees, 1.35 mm apart per
        axis, clear as diamonds (1.273 < 1.35) and overlap as boxes."""
        with tempfile.TemporaryDirectory() as td:
            p = board([('A', 10, 10, 45, ''),
                       ('B', 11.35, 11.35, 45, '')], td)
            g = grade(p)
        self.assertEqual([q for q in g['pairs'] if q.kind == 'fab'], [])

    def test_the_fixed_pose_seat_measures_what_the_checker_does(self):
        """#1054's fixed-pose check on the clear diamonds: the rects overlap,
        the seat must not refuse, because check_assembly passes it."""
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        with tempfile.TemporaryDirectory() as td:
            p = board([('A', 10, 10, 45, SQUARE),
                       ('B', 11.5, 11.5, 45, SQUARE)], td)
            st = pose_score.make_state(parse_kicad_pcb(p), p, clearance=0.1,
                                       board_edge_clearance=0.1)
            for r in ('A', 'B'):
                seeder._materialise_rotation(st.parts[r], 45.0)
            from placement.legality import pair_overlap_area
            pa, pb = st.parts['A'], st.parts['B']
            rect_area = pair_overlap_area(
                pa.sides, pa.side, pa.rect(10, 10, 45.0), None,
                pb.sides, pb.side, pb.rect(11.5, 11.5, 45.0), None)
            area, _w, _h = seeder._courtyard_overlap(
                st, 'A', (10, 10, 45.0), 'B', (11.5, 11.5, 45.0))
            area_hit, _w, _h = seeder._courtyard_overlap(
                st, 'A', (10, 10, 45.0), 'B', (11.2, 11.2, 45.0))
        self.assertGreater(rect_area, 1.0)
        self.assertLessEqual(area, seeder.FIXED_OVERLAP_EPS_MM2)
        self.assertGreater(area_hit, seeder.FIXED_OVERLAP_EPS_MM2)


def raw_board(td, body):
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n' + body + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def fp(ref, x, y, inner, rot=0, net=1):
    return (f'  (footprint "t:{ref}" (layer "F.Cu") (at {x} {y} {rot})\n'
            f'    (property "Reference" "{ref}" (at 0 0 {rot})'
            f' (layer "F.SilkS"))\n' + inner
            + f'    (pad "9" smd rect (at 0 0 {rot}) (size 0.2 0.2)'
            f' (layers "F.Cu") (net {net} "N{net}")))\n')


def line(a, b, layer='F.CrtYd'):
    return (f'    (fp_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke'
            f' (width 0.05) (type default)) (layer "{layer}"))\n')


class TestOutlineDetails(unittest.TestCase):
    """The hunks the #1094 verifier could revert without a test noticing."""

    def test_pad_copper_past_the_courtyard_still_occupies(self):
        """A's pad pokes 0.6 mm past its 1 x 1 courtyard; B's courtyard
        overlaps only that pad, by 0.1 mm. The occupancy is the drawing UNITED with the
        pads, so the pair is found."""
        body = (
            '  (footprint "t:A" (layer "F.Cu") (at 10 10)\n'
            '    (property "Reference" "A" (at 0 0) (layer "F.SilkS"))\n'
            '    (fp_rect (start -0.5 -0.5) (end 0.5 0.5) (stroke (width'
            ' 0.05) (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at 0.8 0) (size 0.6 0.4) (layers "F.Cu")'
            ' (net 1 "N1")))\n'
            + fp('B', 11.8, 10,
                 '    (fp_rect (start -0.8 -0.5) (end 0.8 0.5) (stroke'
                 ' (width 0.05) (type default)) (layer "F.CrtYd"))\n',
                 net=2))
        with tempfile.TemporaryDirectory() as td:
            g = grade(raw_board(td, body))
        self.assertEqual(courtyard_pairs(g), {('A', 'B')})

    def test_an_open_courtyard_falls_back_to_its_hull(self):
        """Three sides of a square drawn: the outline does not close, so the
        convex hull stands in -- never less than what was drawn -- and a
        part inside the open square is still paired."""
        from placement.parser import OUTLINE_HULL, extract_courtyard_shapes
        u = (line((-2, -2), (2, -2)) + line((2, -2), (2, 2))
             + line((2, 2), (-2, 2)))
        body = (fp('A', 10, 10, u)
                + fp('B', 9.2, 10, SQUARE.replace('-1 -1', '-0.4 -0.4')
                     .replace('1 1', '0.4 0.4'), net=2))
        with tempfile.TemporaryDirectory() as td:
            p = raw_board(td, body)
            shp = extract_courtyard_shapes(p)['A']['F']
            g = grade(p)
        self.assertEqual(shp[1], OUTLINE_HULL)
        self.assertAlmostEqual(shp[0].area, 16.0, places=3)
        self.assertEqual(courtyard_pairs(g), {('A', 'B')})

    def test_a_join_a_few_microns_open_still_closes(self):
        """ulx3s BAT1's courtyard: ends that miss by ~4 um are joined, the
        outline is the drawing, not its hull."""
        from placement.parser import OUTLINE_POLYGON, extract_courtyard_shapes
        tri = (line((0, 0), (4, 0)) + line((4, 0), (0, 3))
               + line((0.004, 3.004), (0, 0)))
        with tempfile.TemporaryDirectory() as td:
            shp = extract_courtyard_shapes(raw_board(
                td, fp('A', 10, 10, tri)))['A']['F']
        self.assertEqual(shp[1], OUTLINE_POLYGON)
        # The join moves the stray end by up to its gap, so the area is the
        # triangle's within a few hundredths; the HOW is what tells a joined
        # drawing from its hull (both a triangle here).
        self.assertAlmostEqual(shp[0].area, 6.0, delta=0.05)

    def test_containment_is_measured_on_the_drawn_body(self):
        """B's .Fab body is a right triangle (area 2, box 4) wholly inside
        A's: contained_frac is 1.0 on the drawn areas, 0.5 on the boxes."""
        tri_fab = (line((0, 0), (2, 0), 'F.Fab') + line((2, 0), (0, 2), 'F.Fab')
                   + line((0, 2), (0, 0), 'F.Fab'))
        a_fab = ('    (fp_rect (start -3 -3) (end 3 3) (stroke (width 0.05)'
                 ' (type default)) (layer "F.Fab"))\n')
        with tempfile.TemporaryDirectory() as td:
            g = grade(raw_board(td, fp('A', 10, 10, a_fab)
                                + fp('B', 9, 9, tri_fab, net=2)))
        fab = [q for q in g['pairs'] if q.kind == 'fab']
        self.assertEqual(len(fab), 1, g['pairs'])
        self.assertAlmostEqual(fab[0].area_mm2, 2.0, places=3)
        self.assertEqual(fab[0].contained_frac, 1.0)

    def test_concentric_circles_are_a_ring(self):
        """Two concentric courtyard circles are a RING, as KiCad assembles a
        courtyard (glasgow MK1-MK4: 34 mm2 in pcbnew's GetCourtyard, 78.5
        as a filled disc). A part inside the hole does not pair; one on the
        ring does. A .Fab drawing's inner circle stays a body detail."""
        import math
        from placement.parser import (extract_courtyard_shapes,
                                      extract_fab_shapes)
        ring = ''.join(
            f'    (fp_circle (center 0 0) (end {r} 0) (stroke (width 0.05)'
            f' (type default)) (layer "{ly}"))\n'
            for r in (3, 2) for ly in ('F.CrtYd', 'F.Fab'))
        small = ('    (fp_rect (start -0.6 -0.6) (end 0.6 0.6) (stroke (width'
                 ' 0.05) (type default)) (layer "F.CrtYd"))\n')
        for bx, want in ((11.0, set()), (12.5, {('A', 'B')})):
            with tempfile.TemporaryDirectory() as td:
                p = raw_board(td, fp('A', 10, 10, ring)
                              + fp('B', bx, 10, small, net=2))
                crt = extract_courtyard_shapes(p)['A']['F'][0]
                fab = extract_fab_shapes(p)['A']['F'][0]
                g = grade(p)
            self.assertAlmostEqual(crt.area, math.pi * (9 - 4), delta=0.1)
            self.assertAlmostEqual(fab.area, math.pi * 9, delta=0.1)
            self.assertEqual(courtyard_pairs(g), want, bx)

    def test_an_l_shaped_graze_is_as_thin_as_it_is(self):
        """B (3.2 mm square) sits in the corner notch of A's stepped
        courtyard and grazes both arms 0.2 mm deep: an L of 0.84 mm2. Its
        thickness is ~0.23 mm, under the 0.3 mm blocking floor; the short
        side of its minimum rotated rectangle is 2.2 mm, which made it
        COURTYARD BLOCKING."""
        from placement.legality import COURTYARD_BLOCKING_MIN_DEPTH_MM
        step = [(-5, -7), (5, -7), (5, -5), (7, -5), (7, 5), (5, 5), (5, 7),
                (-5, 7), (-5, 5), (-7, 5), (-7, -5), (-5, -5)]
        stepped = ''.join(line(step[i], step[(i + 1) % len(step)])
                          for i in range(len(step)))
        sq = ('    (fp_rect (start -1.6 -1.6) (end 1.6 1.6) (stroke (width'
              ' 0.05) (type default)) (layer "F.CrtYd"))\n')
        with tempfile.TemporaryDirectory() as td:
            g = grade(raw_board(td, fp('A', 10, 10, stepped)
                                + fp('B', 16.4, 16.4, sq, net=2)))
        pair = next(q for q in g['pairs'] if q.kind == 'courtyard')
        self.assertAlmostEqual(pair.area_mm2, 0.84, places=3)
        self.assertAlmostEqual(pair.depth_mm, 0.2343, delta=0.002)
        self.assertLess(pair.depth_mm, COURTYARD_BLOCKING_MIN_DEPTH_MM)
        self.assertEqual(g['courtyard_blocking_pairs'], [])

    def test_thickness_without_maximum_inscribed_circle(self):
        """shapely before 2.1 has no `maximum_inscribed_circle`; the
        polylabel fallback measures the same thickness."""
        import shapely
        from shapely.geometry import Polygon, box
        from placement.legality import overlap_thickness
        ell = Polygon([(0, 0), (3, 0), (3, 0.2), (0.2, 0.2), (0.2, 3),
                       (0, 3)])
        rect = box(0, 0, 2, 0.7)
        want = [overlap_thickness(ell), overlap_thickness(rect)]
        mic = getattr(shapely, 'maximum_inscribed_circle', None)
        if mic is not None:
            del shapely.maximum_inscribed_circle
        try:
            got = [overlap_thickness(ell), overlap_thickness(rect)]
        finally:
            if mic is not None:
                shapely.maximum_inscribed_circle = mic
        self.assertAlmostEqual(want[1], 0.7, places=9)
        self.assertAlmostEqual(got[1], 0.7, places=9)
        self.assertAlmostEqual(got[0], want[0], delta=1e-3)
        self.assertLess(want[0], 0.3)

    def test_a_pad_turned_off_the_lattice_is_boxed_from_its_own_size(self):
        """PartPads at a 45-degree delta: a 0.4 x 1.2 pad on a part seeded
        at 0 boxes to (0.2 + 0.6) / sqrt2 half-extents; the same pad on a
        part seeded AT 45 and turned back to 0 is its own 0.2 x 0.6 --
        not the seed box grown twice (0.8 x 0.8)."""
        from kicad_parser import parse_kicad_pcb
        from placement.legality import PartPads
        body = ''.join(
            f'  (footprint "t:{r}" (layer "F.Cu") (at {x} 10 {rot})\n'
            f'    (property "Reference" "{r}" (at 0 0 {rot}) (layer'
            f' "F.SilkS"))\n'
            f'    (pad "1" smd rect (at 0 0 {rot}) (size 0.4 1.2) (layers'
            f' "F.Cu") (net 1 "N1")))\n'
            for r, x, rot in (('P0', 5, 0), ('P45', 15, 45)))
        with tempfile.TemporaryDirectory() as td:
            pcb = parse_kicad_pcb(raw_board(td, body))
        d = 0.8 / 2 ** 0.5
        r0 = PartPads(pcb.footprints['P0'], 0.1).pad_rects(0, 0, 45.0)[0]
        self.assertAlmostEqual(r0[2] - r0[0], 2 * d, places=4)
        self.assertAlmostEqual(r0[3] - r0[1], 2 * d, places=4)
        r45 = PartPads(pcb.footprints['P45'], 0.1).pad_rects(0, 0, 0.0)[0]
        w, h = sorted((r45[2] - r45[0], r45[3] - r45[1]))
        self.assertAlmostEqual(w, 0.4, places=4)
        self.assertAlmostEqual(h, 1.2, places=4)


class TestStickHub(unittest.TestCase):
    def setUp(self):
        self.path = stickhub()
        if not self.path:
            self.skipTest('KiCad StickHub demo not installed')

    def test_no_diagonal_part_gates_and_nothing_is_contained(self):
        from kicad_parser import parse_kicad_pcb
        pcb = parse_kicad_pcb(self.path)
        rot = {r: fp.rotation for r, fp in pcb.footprints.items()}
        self.assertGreaterEqual(sum(1 for v in rot.values() if _diag(v)), 39)
        g = grade(self.path)
        diag = [(q.a, q.b, q.area_mm2) for q in g['courtyard_blocking_pairs']
                if _diag(rot[q.a]) or _diag(rot[q.b])]
        self.assertEqual(diag, [])
        self.assertEqual(g['containment_blocking'], 0,
                         g['containment_blocking_pairs'])

    def test_u1_and_y1_seat_at_the_human_poses(self):
        """U1 at -135 and Y1 at their human poses: the rects overlap by
        7.975 mm2 (the first pair #1094 names), the drawn outlines do not."""
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement import seeder
        pcb = parse_kicad_pcb(self.path)
        st = pose_score.make_state(pcb, self.path, clearance=0.15,
                                   board_edge_clearance=0.1)
        u1, y1 = pcb.footprints['U1'], pcb.footprints['Y1']
        pu = (u1.x, u1.y, seeder._materialise_rotation(st.parts['U1'],
                                                       u1.rotation))
        py = (y1.x, y1.y, seeder._materialise_rotation(st.parts['Y1'],
                                                       y1.rotation))
        area, _w, _h = seeder._courtyard_overlap(st, 'U1', pu, 'Y1', py)
        self.assertLessEqual(area, seeder.FIXED_OVERLAP_EPS_MM2)


if __name__ == '__main__':
    unittest.main()
