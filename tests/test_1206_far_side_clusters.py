"""#1206: a through-hole part's far side is one box per CLUSTER of drilled
pads, not one box over all of them.

CM5's Module302 is a B-side SMD connector with two NPTH mount holes 48 mm
apart. Its far (F-side) obstruction was the box over both holes -- a 3 x 51 mm
strip where Module302 has nothing -- and 11 parts the designer placed between
the holes read as COURTYARD-BLOCKING on the designer's own board.
`legality.far_side_local` now gives one box per cluster
(`FAR_SIDE_CLUSTER_GAP_MM`), a `FarSide` tuple whose four numbers are still the
union box, so a reader that ignores `.boxes` is stricter, never looser.

What must hold:

  * between two far-apart posts there is no pair, and the seat admits a part
    there; ON a post there is a pair, and the seat refuses it -- the grader
    and the generator switched together;
  * a pin row is one box, a DIP's two rows are two;
  * no real far-side contact is lost on the corpus, and no pair is added
    (`tests/measure_1206_far_side_clusters.py`'s classification);
  * a FarSide survives pickling, rotation and offset, and still equals its
    union box as a tuple.
"""

import os
import pickle
import sys
import tempfile
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import legality                               # noqa: E402

#: The fa10 CM5 truth board is not tracked; point KRT_CM5_HUMAN at a copy.
CM5 = os.environ.get('KRT_CM5_HUMAN') or (
    'C:/Users/rob/AppData/Local/Temp/claude/fa10_verify/truth/'
    'CM5_MINIMA_3/human.kicad_pcb')


def _board(td, parts):
    """An 80 x 40 board: MOD, a B-side SMD part with two NPTH posts 48 mm
    apart, and F-side 2x1 mm resistors at the given x positions."""
    mod = ('  (footprint "t:MOD" (layer "B.Cu") (at 40 20)\n'
           '    (property "Reference" "MOD" (at 0 0) (layer "B.SilkS"))\n'
           '    (fp_rect (start -3 -3) (end 3 3) (stroke (width 0.05)'
           ' (type default)) (layer "B.CrtYd"))\n'
           '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "B.Cu")'
           ' (net 1 "N1"))\n'
           '    (pad "" np_thru_hole circle (at -24 0) (size 3 3) (drill 3)'
           ' (layers "*.Cu" "*.Mask"))\n'
           '    (pad "" np_thru_hole circle (at 24 0) (size 3 3) (drill 3)'
           ' (layers "*.Cu" "*.Mask")))\n')
    res = ''.join(
        f'  (footprint "t:R" (layer "F.Cu") (at {x} 20)\n'
        f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
        '    (fp_rect (start -1 -0.5) (end 1 0.5) (stroke (width 0.05)'
        ' (type default)) (layer "F.CrtYd"))\n'
        '    (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu")'
        ' (net 2 "N2"))\n'
        '    (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu")'
        ' (net 2 "N2")))\n' for ref, x in parts)
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            '  (gr_rect (start 0 0) (end 80 40) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n' + mod + res + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def _pad(x, y, drill, size=None):
    s = size if size is not None else drill
    return SimpleNamespace(local_x=x, local_y=y, drill=drill, size_x=s,
                           size_y=s, rect_rotation=0.0)


def _fp(pads, rot=0.0):
    return SimpleNamespace(pads=pads, rotation=rot)


class TheModule302Shape(unittest.TestCase):

    def test_between_the_posts_is_clear_and_on_a_post_is_not(self):
        import pose_score
        from placement import seeder
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, [('R1', 40.0), ('R2', 64.0)])
            pcb = parse_kicad_pcb(path)
            far = legality.far_side_local(pcb.footprints['MOD'])
            self.assertIsInstance(far, legality.FarSide)
            self.assertEqual(len(far.boxes), 2)
            g = legality.grade_body_overlap(pcb, 0.2, pcb_file=path,
                                            courtyard_severity=None)
            pairs = {frozenset((p.a, p.b)) for p in g['pairs']
                     if p.kind == 'courtyard'}
            # R1 sits between the posts, 22 mm from either: no pair. With
            # the union box it read as MOD's strip.
            self.assertNotIn(frozenset(('MOD', 'R1')), pairs)
            # R2 sits ON the right post: a real far-side pair, kept.
            self.assertIn(frozenset(('MOD', 'R2')), pairs)
            # The generator agrees: the seat admits R1 between the posts and
            # refuses it on the left post.
            st = pose_score.make_state(pcb, path, clearance=0.2)
            self.assertTrue(seeder.pose_ok(st, 'R1', 40.0, 20.0, 0.0,
                                           exclude={'R2'}))
            self.assertFalse(seeder.pose_ok(st, 'R1', 16.0, 20.0, 0.0,
                                            exclude={'R2'}))

    def test_the_union_model_is_what_the_issue_reported(self):
        """The control: with `far_side_local` patched back to the single box,
        R1 between the posts IS a pair -- so the arm above is measuring the
        cluster model, not a fixture that never overlapped."""
        real = legality.far_side_local
        legality.far_side_local = (
            lambda fp, *_a, **_k: legality.through_pad_bounds_local(fp))
        try:
            with tempfile.TemporaryDirectory() as td:
                path = _board(td, [('R1', 40.0)])
                pcb = parse_kicad_pcb(path)
                g = legality.grade_body_overlap(pcb, 0.2, pcb_file=path,
                                                courtyard_severity=None)
                self.assertTrue(any({p.a, p.b} == {'MOD', 'R1'}
                                    for p in g['pairs']
                                    if p.kind == 'courtyard'))
        finally:
            legality.far_side_local = real

    @unittest.skipUnless(os.path.isfile(CM5), 'the fa10 CM5 truth board is '
                         'local, not tracked')
    def test_cm5_module302(self):
        pcb = parse_kicad_pcb(CM5)
        g = legality.grade_body_overlap(pcb, 0.2, pcb_file=CM5)
        m302 = sorted(p.b if p.a == 'Module302' else p.a for p in g['pairs']
                      if p.kind == 'courtyard' and 'Module302' in (p.a, p.b))
        self.assertEqual(m302, ['Module301'], m302)


class TheOtherConsumers(unittest.TestCase):
    """The decision sites besides the pair channel read the clusters too."""

    def test_keepout_hit_reads_the_clusters(self):
        from placement.floorplan import keepout_hit
        fs = legality.FarSide([(0, 0, 1, 1), (10, 0, 11, 1)])
        between = {'name': 'k', 'rect': (5.0, 0.0, 6.0, 1.0)}
        on_post = {'name': 'k', 'rect': (10.2, 0.2, 10.8, 0.8)}
        self.assertEqual(keepout_hit(between, (None, fs)), 0.0)
        self.assertGreater(keepout_hit(on_post, (None, fs)), 0.0)
        disc = {'name': 'k', 'circle': (5.5, 0.5, 0.4)}
        self.assertEqual(keepout_hit(disc, (None, fs)), 0.0)

    def test_reseat_clash_reads_the_clusters(self):
        import pose_score
        from placement import reseat
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, [('R1', 40.0)])
            pcb = parse_kicad_pcb(path)
            st = pose_score.make_state(pcb, path, clearance=0.2)
            self.assertFalse(reseat._clashes_with_seated(
                st, 'R1', (40.0, 20.0, 0.0), {'MOD': (40.0, 20.0, 0.0)}))
            self.assertTrue(reseat._clashes_with_seated(
                st, 'R1', (16.0, 20.0, 0.0), {'MOD': (40.0, 20.0, 0.0)}))


class Clusters(unittest.TestCase):

    def test_a_pin_row_is_one_box_and_a_dip_is_two(self):
        row = _fp([_pad(2.54 * i, 0.0, 1.0, 1.7) for i in range(10)])
        self.assertEqual(len(legality.through_pad_clusters_local(row)), 1)
        self.assertNotIsInstance(legality.far_side_local(row),
                                 legality.FarSide)
        dip = _fp([_pad(2.54 * i, y, 0.8, 1.6) for i in range(8)
                   for y in (0.0, 7.62)])
        self.assertEqual(len(legality.through_pad_clusters_local(dip)), 2)
        # The union is unchanged: through_pad_bounds_local is the bbox.
        self.assertEqual(tuple(legality.far_side_local(dip)),
                         legality.through_pad_bounds_local(dip))

    def test_the_gap_threshold_is_inclusive_and_named(self):
        g = legality.FAR_SIDE_CLUSTER_GAP_MM
        near = _fp([_pad(0.0, 0.0, 1.0), _pad(1.0 + g, 0.0, 1.0)])
        far = _fp([_pad(0.0, 0.0, 1.0), _pad(1.0 + g + 0.01, 0.0, 1.0)])
        self.assertEqual(len(legality.through_pad_clusters_local(near)), 1)
        self.assertEqual(len(legality.through_pad_clusters_local(far)), 2)

    def test_farside_survives_pickle_rotate_offset(self):
        fs = legality.FarSide([(0, 0, 1, 1), (10, 0, 11, 1)])
        self.assertEqual(tuple(fs), (0.0, 0.0, 11.0, 1.0))
        self.assertEqual(pickle.loads(pickle.dumps(fs)).boxes, fs.boxes)
        r = legality.rotate_far(fs, 90.0)
        self.assertIsInstance(r, legality.FarSide)
        self.assertEqual(len(r.boxes), 2)
        o = legality.offset_far(r, 5.0, 5.0)
        self.assertEqual([tuple(round(v, 6) for v in b) for b in o.boxes],
                         [tuple(round(v + 5.0, 6) for v in b)
                          for b in r.boxes])
        # Between the boxes the far side is clear; the union box is not.
        mid = (5.0, 0.2, 6.0, 0.8)
        self.assertEqual(legality.far_overlap_area(fs, mid), 0.0)
        self.assertGreater(legality.rect_overlap_area(tuple(fs), mid), 0.0)
        self.assertGreater(legality.far_gap(fs, mid), 3.0)


class TheCorpus(unittest.TestCase):

    def test_no_real_contact_is_lost_and_nothing_is_added(self):
        import measure_1206_far_side_clusters as m
        import run_utils
        boards = run_utils.corpus_boards()
        if not boards:
            self.skipTest('git cannot list the corpus here')
        moved, real = 0, 0
        for b in boards:
            rows, bad = m.measure(os.path.join(ROOT, b))
            self.assertEqual(bad, [], b)
            moved += len(rows)
            real += sum(1 for r in rows if r[0] == 'real')
            # ...and no pair turns BLOCKING (the relative floor's
            # denominator shrinks with the far side, so area alone cannot
            # say).
            self.assertEqual([v for v in m.verdict_changes(
                os.path.join(ROOT, b)) if v[0] == 'BLOCKING-ADDED'], [], b)
        # Not vacuous: the corpus has phantom strips to remove, and real
        # far-side contact the classifier must keep.
        self.assertGreater(moved, 10)
        self.assertEqual(real, 4)


def _tht(ref, x, y, posts, crt=1.0, fab=None, layer='F', crt_far=None,
         drill=1.0, size=1.6):
    """A `layer`-side through-hole part at (x, y): a square courtyard of half
    `crt`, an optional square .Fab body of half `fab`, an optional far-face
    courtyard rect, and one drilled pad per local (px, py) in `posts`."""
    far = 'B' if layer == 'F' else 'F'
    s = (f'  (footprint "t:{ref}" (layer "{layer}.Cu") (at {x} {y})\n'
         f'    (property "Reference" "{ref}" (at 0 0) (layer "{layer}.SilkS"))\n'
         f'    (fp_rect (start {-crt} {-crt}) (end {crt} {crt}) (stroke'
         f' (width 0.05) (type default)) (layer "{layer}.CrtYd"))\n')
    if fab:
        s += (f'    (fp_rect (start {-fab} {-fab}) (end {fab} {fab}) (stroke'
              f' (width 0.1) (type default)) (layer "{layer}.Fab"))\n')
    if crt_far:
        s += (f'    (fp_rect (start {crt_far[0]} {crt_far[1]}) (end'
              f' {crt_far[2]} {crt_far[3]}) (stroke (width 0.05)'
              f' (type default)) (layer "{far}.CrtYd"))\n')
    for i, (px, py) in enumerate(posts):
        s += (f'    (pad "{i + 1}" thru_hole circle (at {px} {py}) (size'
              f' {size} {size}) (drill {drill}) (layers "*.Cu" "*.Mask")'
              f' (net 1 "N1"))\n')
    return s + '  )\n'


def _smd(ref, x, y, layer='B', half=(1.0, 0.5), fab=None):
    s = (f'  (footprint "t:{ref}" (layer "{layer}.Cu") (at {x} {y})\n'
         f'    (property "Reference" "{ref}" (at 0 0) (layer "{layer}.SilkS"))\n'
         f'    (fp_rect (start {-half[0]} {-half[1]}) (end {half[0]} {half[1]})'
         f' (stroke (width 0.05) (type default)) (layer "{layer}.CrtYd"))\n')
    if fab:
        s += (f'    (fp_rect (start {-fab[0]} {-fab[1]}) (end {fab[0]} {fab[1]})'
              f' (stroke (width 0.1) (type default)) (layer "{layer}.Fab"))\n')
    return s + (f'    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers'
                f' "{layer}.Cu") (net 2 "N2"))\n  )\n')


def _write(td, name, parts, size=(40, 40)):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
                 ' (44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
                 f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (stroke'
                 ' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')
    return path


class TheVerifiersSurvivors(unittest.TestCase):
    """The phase-2 verifier's surviving mutants, one test each: every place a
    far side is turned, offset, measured or drawn keeps the clusters."""

    def test_every_rotation_fill_keeps_the_clusters(self):
        """R1/R2: `_Part.ensure_rotation` is the one fill, and an off-
        lattice angle (a declared 45, a swapped partner's) keeps a FarSide."""
        import pose_score
        from placement import seeder
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, [])
            st = pose_score.make_state(parse_kicad_pcb(path), path,
                                       clearance=0.2)
        part = st.parts['MOD']
        part.ensure_rotation(45.0)
        self.assertIsInstance(part.tht_by_rot[45.0], legality.FarSide)
        self.assertEqual(part.tht_by_rot[45.0].boxes, legality.rotate_far(
            part.tht_by_rot[0.0], 45.0).boxes)
        self.assertEqual(seeder._materialise_rotation(part, 390.0), 30.0)
        self.assertIsInstance(part.tht_by_rot[30.0], legality.FarSide)

    def test_no_other_site_fills_the_far_side_cache(self):
        """The structural half: outside `_Part.__init__` and
        `_Part.ensure_rotation`, nothing in the placement engine assigns a
        `tht_by_rot[...]` entry -- a new inline copy is how the union came
        back four times."""
        import ast
        allowed = {('quench.py', '__init__'), ('quench.py', 'ensure_rotation')}
        bad = []
        pl = os.path.join(ROOT, 'py_placer', 'placement')
        for name in sorted(os.listdir(pl)):
            if not name.endswith('.py'):
                continue
            with open(os.path.join(pl, name), encoding='utf-8') as fh:
                tree = ast.parse(fh.read())
            for fn in ast.walk(tree):
                if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                for node in ast.walk(fn):
                    if not isinstance(node, ast.Assign):
                        continue
                    for t in node.targets:
                        if (isinstance(t, ast.Subscript)
                                and isinstance(t.value, ast.Attribute)
                                and t.value.attr == 'tht_by_rot'
                                and (name, fn.name) not in allowed):
                            bad.append((name, fn.name, node.lineno))
        self.assertEqual(bad, [])

    def test_zone_feasibility_reads_the_clusters(self):
        """R3/R4: a keep-out between a part's courtyard and its right post.
        The zone forces y and leaves x in [10, 21]; only x = 17 clears both
        posts and the courtyard. The union box covers the keep-out at every
        x, so it called the zone infeasible."""
        from placement import floorplan
        far = legality.FarSide([(-6.5, -0.5, -5.5, 0.5), (5.5, -0.5, 6.5, 0.5)])
        k = [{'name': 'k', 'rect': (11.5, 9.0, 15.0, 11.0)}]
        part = floorplan._LocalPart(0.0, (-2, -1, 2, 1), far)
        got = floorplan.zone_pose_feasibility((8, 9, 23, 11), 0.0, part, k)
        self.assertTrue(got['feasible'], got)
        self.assertAlmostEqual(got['witness'][0], 17.0, places=6)
        union = floorplan._LocalPart(0.0, (-2, -1, 2, 1), tuple(far))
        self.assertFalse(floorplan.zone_pose_feasibility(
            (8, 9, 23, 11), 0.0, union, k)['feasible'])

    def test_zone_feasibility_proposes_the_inner_post_edges(self):
        """R4, the CANDIDATES: keep-outs ka right of the part and kb left
        of it leave x in [19.5, 20.0], bounded on both sides by the
        INNER edge of a post's forbidden interval (the right post clearing
        ka, the left post clearing kb). Every other candidate -- the zone's
        own edges, the courtyard's, the union box's outer ones -- is
        refused, so the search finds the witness only if the cluster boxes
        propose their edges. (The fixture above is found from the
        courtyard's edges alone and could not tell.)"""
        from placement import floorplan
        far = legality.FarSide([(-6.5, -0.5, -5.5, 0.5), (5.5, -0.5, 6.5, 0.5)])
        k = [{'name': 'ka', 'rect': (24.5, 9.0, 25.0, 11.0)},
             {'name': 'kb', 'rect': (14.5, 9.0, 15.0, 11.0)}]
        part = floorplan._LocalPart(0.0, (-2, -1, 2, 1), far)
        # Origin x in [18.1, 20.5]: both ends and their midpoint (19.3) are
        # refused, which is all a search proposing no post edge would try.
        got = floorplan.zone_pose_feasibility((16.1, 9, 22.5, 11), 0.0, part,
                                              k)
        self.assertTrue(got['feasible'], got)
        self.assertTrue(19.5 - 1e-6 <= got['witness'][0] <= 20.0 + 1e-6, got)

    def test_the_mating_region_reads_the_clusters(self):
        """R5: P's two posts straddle a USB tongue and its courtyard is clear
        of it: nothing of P is on the mating region. The union box crossed
        the tongue."""
        import test_1098_mating_keepout as t1098
        from placement import floorplan
        with tempfile.TemporaryDirectory() as td:
            path = t1098.board(td, r1=(5, 5, 'F.Cu'), hole=False)
            with open(path, encoding='utf-8') as fh:
                text = fh.read().rstrip()
            assert text.endswith(')')
            text = (text[:-1] + _tht('P', 15, 17, [(-7, 4), (7, 4)])
                    + ')\n')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(text)
            pcb = parse_kicad_pcb(path)
            refs = {m['ref'] for m in floorplan.mating_keepout_findings(
                pcb, path)}
            self.assertNotIn('P', refs)
            # The control: the same part with a post ON the tongue is found.
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(text.replace('(at 7 4)', '(at 3 4)'))
            pcb = parse_kicad_pcb(path)
            self.assertIn('P', {m['ref'] for m in floorplan.
                                mating_keepout_findings(pcb, path)})

    def test_the_body_seam_reads_the_clusters(self):
        """R10: R sits on B between M's posts. M and R share B only at the
        posts, 5 mm away; the union box made them a -1.0 mm 'seam'."""
        with tempfile.TemporaryDirectory() as td:
            path = _write(td, 'seam', [
                _tht('M', 20, 20, [(-6, 0), (6, 0)], fab=1.0),
                _smd('R', 20, 20, fab=(1.0, 0.5))])
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path)
        seam = g['body_seam']
        self.assertIsNotNone(seam)
        self.assertGreater(seam['mm'], 3.0, seam)

    def test_far_vs_far_is_measured_on_the_clusters(self):
        """R15: A's posts at 0 and 20 mm, B's at 10 and 20 mm -- they meet at
        ONE post on the far face. The union strips overlap 10.8 x 1.6."""
        with tempfile.TemporaryDirectory() as td:
            path = _write(td, 'ff', [
                _tht('A', 5, 20, [(-0, 0), (20, 0)]),
                _tht('B', 15, 20, [(0, 0), (10, 0)])])
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path,
                                            courtyard_severity=None)
        far = [p for p in g['pairs'] if p.kind == 'courtyard'
               and {p.a, p.b} == {'A', 'B'} and p.side == 'B']
        self.assertEqual(len(far), 1, g['pairs'])
        self.assertAlmostEqual(far[0].area_mm2, 1.6 * 1.6, places=2)

    def test_the_helpers_sum_and_take_the_minimum(self):
        """R14 / R16: a rect meeting both boxes overlaps their SUM; the gap
        is the NEAREST box's."""
        fs = legality.FarSide([(0, 0, 1, 1), (2, 0, 3, 1)])
        self.assertAlmostEqual(legality.far_overlap_area(fs, (0, 0, 3, 1)),
                               2.0)
        fs = legality.FarSide([(0, 0, 1, 1), (10, 0, 11, 1)])
        self.assertAlmostEqual(legality.far_gap(fs, (2, 0, 3, 1)), 1.0)

    def test_the_cluster_sweep_does_not_depend_on_pad_order(self):
        """R17: the sweep stops at the first box too far right, so it must
        run in x order -- in file order [0, 7.62, 2.54, 5.08] it stopped
        before ever linking 0 to 2.54."""
        row = [_pad(x, 0.0, 1.0) for x in (0.0, 7.62, 2.54, 5.08)]
        self.assertEqual(len(legality.through_pad_clusters_local(_fp(row))), 1)

    def test_the_render_draws_one_box_per_cluster(self):
        """R11: the review sheet draws what is graded -- two posts, not the
        strip between them."""
        from render_placement import draw_courtyards
        drawn = []
        d = SimpleNamespace(rectangle=lambda box, **k: drawn.append(box),
                            line=lambda *a, **k: None)
        r = SimpleNamespace(tf=SimpleNamespace(pt=lambda x, y: (x, y),
                                               length=lambda mm: mm))
        fs = legality.FarSide([(0, 0, 1, 1), (10, 0, 11, 1)])
        model = SimpleNamespace(rect=lambda ref: (4, 0, 6, 1),
                                side=lambda ref: 'F',
                                sides=lambda ref: {'F', 'B'},
                                far_rect=lambda ref: fs)
        draw_courtyards(d, r, model, ['MOD'], side='B')
        self.assertEqual(sorted(map(tuple, drawn)),
                         [(0, 0, 1, 1), (10, 0, 11, 1)])


class AFarFaceCourtyardIsFarSide(unittest.TestCase):
    """A footprint that DRAWS a courtyard on its far face is graded on it
    there, as KiCad grades it: MM's B.CrtYd spans its two leads and R1 sits
    under it on B -- kicad-cli reports courtyards_overlap MM <-> R1 (phase-2
    verifier), and the clusters alone lost the pair."""

    def test_the_pair_is_kept(self):
        with tempfile.TemporaryDirectory() as td:
            path = _write(td, 'mid', [
                _tht('MM', 20, 20, [(-3.5, 0), (3.5, 0)], crt=2.0,
                     crt_far=(-4.5, -2, 4.5, 2)),
                _smd('R1', 20, 20, half=(1.5, 0.8))])
            g = legality.grade_body_overlap(parse_kicad_pcb(path), 0.2,
                                            pcb_file=path,
                                            courtyard_severity=None)
        self.assertTrue([p for p in g['courtyard_blocking_pairs']
                         if {p.a, p.b} == {'MM', 'R1'} and p.side == 'B'],
                        g['pairs'])

    def test_a_lone_far_courtyard_is_not_counted_twice(self):
        self.assertIsNone(legality.far_courtyard_of({'B': (0, 0, 1, 1)}, 'F'))
        self.assertEqual(legality.far_courtyard_of(
            {'F': (0, 0, 2, 2), 'B': (0, 0, 1, 1)}, 'F'), (0, 0, 1, 1))
        self.assertIsNone(legality.far_courtyard_of(None, 'F'))


if __name__ == '__main__':
    unittest.main()
