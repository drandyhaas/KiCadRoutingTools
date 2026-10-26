"""#1403: an edge connector's overhang reads ONE distance, past its DECLARED edge.

`courtyard_overhang` is `rect_outside_amount` at zero margin -- a ranking cost
summed over every side a courtyard crosses and over each off-board corner of a
non-rectangular outline. The band was graded on it, so a courtyard 1.4624 mm
past the east edge of a many-sided outline read 4.3872 (3x), and a part flush
with its declared east edge but 1.75 mm past the NORTH edge read an east
overhang of 5.26. The band is now `courtyard_overhang_past`: the signed
distance past the declared edge along its normal (a corner hanging off a
rounded outline counts by its distance to the outline), and what the part does
past another edge is reported on its own (the evidence row's
`other_edge_overhang_mm`), an `edge_connector` ERROR above the band's maximum.
"""
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb              # noqa: E402
from placement import floorplan                       # noqa: E402
from placement.connector_geometry import (courtyard_overhang,   # noqa: E402
                                          courtyard_overhang_past, geometry_for)
from run_utils import evidence                         # noqa: E402


class Holder:
    pass


BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'watchy.kicad_pcb'), 'tracked board')
PCB = parse_kicad_pcb(BOARD)
GEO = geometry_for(Holder(), PCB, BOARD)
B = GEO.bounds


def rect_past_east(over, north_over=0.0):
    y = 93.68
    top = B[1] - north_over
    return (B[2] - 6.0 + over - 0.0, top if north_over else y - 3.0,
            B[2] + over, (top + 6.0) if north_over else y + 3.0)


class TestOneDistance(unittest.TestCase):
    def test_the_old_reading_is_a_ranking_cost_not_a_distance(self):
        r = rect_past_east(1.4624)
        self.assertGreater(courtyard_overhang(GEO, r), 1.4624 * 2.5)

    def test_past_the_declared_edge_reads_that_distance(self):
        for over in (0.4624, 1.4624, 1.9624):
            amount, others = courtyard_overhang_past(GEO, rect_past_east(over), 'east')
            self.assertAlmostEqual(amount, over, places=4)
            self.assertTrue(all(v == 0.0 for v in others.values()), others)

    def test_another_edges_overhang_is_not_the_declared_edges(self):
        # on a rectangular board: flush with the east edge, 1.7524 mm past
        # the north one (the summed reading booked that to the east band)
        rect_pcb = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
        geo = geometry_for(Holder(), parse_kicad_pcb(rect_pcb), rect_pcb)
        b = geo.bounds
        r = (b[2] - 6.0, b[1] - 1.7524, b[2], b[1] + 4.0)
        amount, others = courtyard_overhang_past(geo, r, 'east')
        self.assertAlmostEqual(amount, 0.0, places=4)
        self.assertAlmostEqual(others['north'], 1.7524, places=4)
        self.assertGreater(courtyard_overhang(geo, r), 1.7)

    def test_inside_reads_zero(self):
        amount, others = courtyard_overhang_past(GEO, rect_past_east(-2.0), 'east')
        self.assertEqual(amount, 0.0)

    def test_the_grade_reports_the_other_edge_on_its_own(self):
        doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
               'edge_connectors': [{'ref': 'J2', 'class': 'edge_receptacle',
                                    'edge': 'east', 'overhang_mm': {'min': 0.0, 'max': 6.0}}]}
        pcb = parse_kicad_pcb(BOARD)
        # J2 as the designer placed it: only the east edge is crossed, by
        # 1.4624 mm (the summed reading said 4.3872).
        res = floorplan.grade(floorplan.intent_from_dict(doc, source_path='t'), pcb, BOARD)
        ev = [e for e in res.edge_connector_evidence if e['ref'] == 'J2'][0]
        self.assertAlmostEqual(ev['overhang_mm'], 1.4624, places=3)
        self.assertLess(ev['overhang_mm'], 2.0)
        self.assertEqual(ev['other_edge_overhang_mm'], {})

    def test_another_edge_past_the_band_maximum_is_an_error(self):
        import subprocess
        from run_utils import tool, tool_env
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'm.kicad_pcb')
            subprocess.run([sys.executable, '-X', 'utf8', tool('place_pose.py'), BOARD, out,
                            'set', 'J2', '98.6176', '73.0', '--rot', '90', '--force'],
                           capture_output=True, text=True, env=tool_env(), cwd=ROOT, timeout=600)
            pcb = parse_kicad_pcb(evidence(out, 'posed board'))
            for hi, errors in ((6.0, 0), (1.0, 1)):
                doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
                       'edge_connectors': [{'ref': 'J2', 'class': 'edge_receptacle',
                                            'edge': 'east',
                                            'overhang_mm': {'min': 0.0, 'max': hi}}]}
                res = floorplan.grade(floorplan.intent_from_dict(doc, source_path='t'), pcb, out)
                ev = [e for e in res.edge_connector_evidence if e['ref'] == 'J2'][0]
                self.assertAlmostEqual(ev['overhang_mm'], 0.0, places=3)
                self.assertAlmostEqual(ev['other_edge_overhang_mm']['north'], 1.7524, places=3)
                bad = [v for v in res.violations
                       if v.rule == 'edge_connector' and 'overhangs the north edge' in v.message]
                self.assertEqual(len(bad), errors, [v.message for v in res.violations])


if __name__ == '__main__':
    unittest.main()
