"""#1392: an edge part whose class is exempt from the facing conjunct SAYS so.

A side-push switch (class `edge_actuator`) declared on an edge is exempt from
the facing conjunct (its plastic says nothing about which way it is pressed),
and `check_floorplan` printed `overhang ... pass` with no facing word -- so a
switch whose actuator points along its edge read as graded and passed. The
clause now reads `facing exempt (<policy>: ...)`, and a declared `mating_face`
turns the conjunct on (`facing north [declared, strong] FAIL`).
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                              # noqa: E402
from run_utils import evidence                                # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'watchy.kicad_pcb'), 'tracked board')


def _tail(entry):
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
           'units': 'mm', 'edge_connectors': [entry]}
    r = floorplan.grade(floorplan.intent_from_dict(doc, source_path='t'),
                        parse_kicad_pcb(BOARD), BOARD)
    ev = [e for e in r.edge_connector_evidence if e['ref'] == entry['ref']]
    return ev[0]['facing'], floorplan._facing_tail(ev[0]['facing'])


class TheExemptFacingClause(unittest.TestCase):
    def test_an_exempt_actuator_says_it_is_not_graded(self):
        f, tail = _tail({'ref': 'SW1', 'edge': 'east', 'class': 'edge_actuator'})
        self.assertEqual(f['disposition'], 'exempt')
        self.assertIn('facing exempt', tail)
        self.assertIn('edge_actuator', tail)
        self.assertIn('mating_face', tail)
        self.assertNotIn('pass', tail)

    def test_a_declared_face_grades_it(self):
        f, tail = _tail({'ref': 'SW1', 'edge': 'east', 'class': 'edge_actuator',
                         'mating_face': '-y'})
        self.assertIn(f['disposition'], ('pass', 'fail'))
        self.assertNotIn('exempt', tail)
        self.assertIn('[declared, strong]', tail)


if __name__ == '__main__':
    unittest.main()
