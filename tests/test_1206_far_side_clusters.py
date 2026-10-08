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

CM5 = ('C:/Users/rob/AppData/Local/Temp/claude/fa10_verify/truth/'
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
        legality.far_side_local = legality.through_pad_bounds_local
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
        moved = 0
        for b in run_utils.corpus_boards():
            rows, bad = m.measure(os.path.join(ROOT, b))
            self.assertEqual(bad, [], b)
            moved += len(rows)
        # Not vacuous: the corpus has phantom strips to remove.
        self.assertGreater(moved, 10)


if __name__ == '__main__':
    unittest.main()
