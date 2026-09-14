"""Behavioral contract for drawn bodies, declared seating and independent copper."""
import hashlib
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for folder in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, str(ROOT / folder))

from kicad_parser import parse_kicad_pcb
from placement import floorplan, seeder, reconstruct
from placement.connector_geometry import ConnectorGeometry, candidate_copper
from placement.writer import write_placed_output
from placement.portfolio import copy_siblings
import pose_score

SOURCE = ROOT / 'kicad_files/esp_prog.kicad_pcb'
RUN_ALL_FAST_OK = True


def grade(path, band, clearance=.25, edge=.25, **extra):
    intent = floorplan.intent_from_dict({
        'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
        'edge_connectors': [dict(ref='USB1', edge='west', overhang_mm=band, **extra)]})
    return floorplan.grade(intent, parse_kicad_pcb(str(path)), str(path),
                           clearance=clearance, board_edge_clearance=edge)


class ConnectorContract(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def board(self, dx):
        path = self.root / (str(dx) + '.kicad_pcb')
        write_placed_output(str(SOURCE), str(path), [
            {'reference': 'USB1', 'new_x': 117.5+dx, 'new_y': 100., 'new_rotation': 180.}])
        copy_siblings(str(SOURCE), str(path))
        return path

    def test_public_margin_matrix_and_nonbinding_zero(self):
        # Git checkouts differ only by LF/CRLF; pin identical source content.
        self.assertEqual(hashlib.sha256(SOURCE.read_bytes().replace(b'\r\n', b'\n')).hexdigest(),
            'a9945bb0940f79672b7c6e32b7a6b9d0b135bf78030e19fcdb88e65c2139903f')
        for dx in (0., -1.45, -2.1, .4, -.1):
            path = self.board(dx)
            for copper, edge in ((.25, .25), (.55, .55), (.25, 0.), (.55, .25)):
                for band in ({'min': .05, 'max': .2}, {'min': 0., 'max': .65}):
                    row = grade(path, band, copper, edge).edge_seating[0]
                    self.assertAlmostEqual(row['overhang_mm'], max(0, -dx))
                    self.assertAlmostEqual(row['body_setback_mm'], max(0, dx))
                    self.assertEqual(row['overhang_mm'], row['body_overhang_mm'])
                    self.assertEqual(row['overhang_basis'], 'F.Fab')
                    self.assertAlmostEqual(row['pad_copper_declared_edge_gap_mm'], 1.6+dx)
                    expected = 'pass' if band['min'] <= max(0, -dx) <= band['max'] else 'fail'
                    self.assertEqual(row['measurements']['body_overhang']['disposition'], expected)
                    self.assertEqual(row['measurements']['body_setback']['disposition'], 'not_declared')
        row = grade(self.board(.4), {'min': 0., 'max': .65}, max_setback_mm=.2).edge_seating[0]
        self.assertEqual(row['measurements']['body_setback']['disposition'], 'fail')

    def test_legal_body_does_not_waive_copper(self):
        path = self.board(-1.3)
        for edge, disposition in ((.25, 'pass'), (.55, 'fail')):
            row = grade(path, {'min': 1.25, 'max': 1.35}, edge=edge).edge_seating[0]
            self.assertEqual(row['measurements']['body_overhang']['disposition'], 'pass')
            self.assertEqual(row['measurements']['pad_copper_edge_gap']['disposition'], disposition)
        pcb = parse_kicad_pcb(str(path))
        state = pose_score.make_state(pcb, str(path), clearance=.25,
                                     board_edge_clearance=.55, grid_step=.1)
        self.assertFalse(seeder.edge_seat_ok(state, state.parts['USB1'], 116.2, 100.,
                                            'west', 1.25, 1.35))
        self.assertEqual(reconstruct.pad_oob_amount(state),
                         reconstruct.pad_oob_amount(state, {'USB1': 999.}))

    def test_envelope_rotation_and_missing_geometry(self):
        # Triangle's transformed *vertices*, not its local bounding rectangle.
        template = '''(kicad_pcb (version 20241229)
          (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))
          (footprint "test" (layer "F.Cu") (at 2.5 10 45)
            (property "Reference" "J1") BODY
            (pad "" smd rect (at 0 0 45) (size .5 .5) (layers "F.Cu"))))'''
        for drawing, measured in (
            ('(fp_poly (pts (xy 0 0) (xy 4 0) (xy 0 4)) (layer "F.Fab"))', True),
            ('(fp_line (start 0 0) (end 4 0) (layer "F.Fab"))', False),
            ('(fp_circle (center 0 0) (end 1 0) (layer "F.Fab"))', False),
            ('', False)):
            path = self.root / 'shape.kicad_pcb'
            path.write_text(template.replace('BODY', drawing), encoding='utf8')
            pcb = parse_kicad_pcb(str(path))
            row = ConnectorGeometry(pcb, str(path)).measure('J1', 'west')
            self.assertEqual(row['body_measured'], measured)
            if measured:
                self.assertAlmostEqual(row['body_setback_mm'], 2.5)
                self.assertAlmostEqual(row['body_bounds_mm'][2], 2.5+math.sqrt(8))
            else:
                self.assertIsNone(row['overhang_mm'])
                self.assertTrue(row['body_unmeasured_reason'])

    def test_candidate_copper_rotations_match_written_boards(self):
        path = self.board(0.)
        pcb = parse_kicad_pcb(str(path))
        state = pose_score.make_state(pcb, str(path), clearance=.25,
                                     board_edge_clearance=.55, grid_step=.1)
        from placement.legality import grade_pad_edge_clearance
        from copy import copy
        for rot in (0., .5, 1.0001, 37., 89.5, 90., 180.5, 359.5):
            state.parts['USB1'].rot = rot
            trial = candidate_copper(state, 'USB1', 118., 100.)
            out = self.root / 'rot.kicad_pcb'
            write_placed_output(str(path), str(out), [{'reference': 'USB1',
                'new_x': 118., 'new_y': 100., 'new_rotation': rot}])
            written = parse_kicad_pcb(str(out))
            one = copy(written)
            one.footprints = {'USB1': written.footprints['USB1']}
            actual = grade_pad_edge_clearance(one, .55, str(out))
            for edge in ('west', 'east', 'north', 'south'):
                self.assertAlmostEqual(trial['minimum_gap_by_edge_mm'][edge],
                                       actual['minimum_gap_by_edge_mm'][edge], places=5)


if __name__ == '__main__':
    unittest.main()
