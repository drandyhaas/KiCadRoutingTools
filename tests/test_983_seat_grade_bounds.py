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


if __name__ == '__main__':
    unittest.main(verbosity=2)
