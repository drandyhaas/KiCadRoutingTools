"""connector_affinity: generic connectors get a WEAK class (run-23).

Run 23 seated J2/J5/J6/J7 mid-board and no instrument could say so: generic
connector keywords deliberately map to NO edge class (a JST wire-to-board
part is legitimately interior -- that guard stands), and a part with no
class is invisible to every intent rule. The weak class exists to be
DECLARED (--declare-classes) and graded at ADVISORY severity: an INTERIOR
pose (past part_class.INTERIOR_AFFINITY_MM from every edge) is a warning for
the boundary review, never an error, and `pass` never flips on it.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer')):
    sys.path.insert(0, p)

PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')
SPLITFLAP = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _cf(*argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestClass(unittest.TestCase):
    def test_generic_connectors_classify_weak_not_edge(self):
        from kicad_parser import parse_kicad_pcb
        from placement.part_class import (MECHANICAL_CLASSES, classify_part,
                                          pose_plausible)
        pcb = parse_kicad_pcb(PLACED)
        for ref in ('J2', 'J4', 'J5', 'J6', 'J7'):
            cls = classify_part(pcb.footprints[ref], ref)
            self.assertEqual(cls.name, 'connector_affinity', ref)
            self.assertEqual(cls.confidence, 'low', ref)
        # The class makes NO pose claim and is NOT mechanically pinned --
        # both would change reconstruct/stager behavior, which this class
        # must never do.
        self.assertNotIn('connector_affinity', MECHANICAL_CLASSES)
        self.assertTrue(pose_plausible('connector_affinity', 0.0, 99.0))
        # The receptacle guard stands: J1 (USB-C) is still edge_receptacle.
        self.assertEqual(classify_part(pcb.footprints['J1'], 'J1').name,
                         'edge_receptacle')


class TestDeclareAndGrade(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.TemporaryDirectory()
        cls.intent = os.path.join(cls.td.name, 'intent.json')
        r = _cf(PLACED, '--emit-intent', cls.intent, '--declare-classes')
        assert r.returncode == 0, r.stdout[-400:]
        cls.grade = os.path.join(cls.td.name, 'grade.json')
        cls.gr = _cf(PLACED, '--intent', cls.intent, '--json', cls.grade)

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def test_declare_classes_declares_the_connector_family(self):
        doc = json.load(open(self.intent, encoding='utf-8'))
        by_ref = {c['ref']: c for c in doc['edge_connectors']}
        for ref in ('J2', 'J4', 'J5', 'J6', 'J7'):
            self.assertIn(ref, by_ref, 'connector not declared')
            self.assertEqual(by_ref[ref]['class'], 'connector_affinity')
            # NO invented edge (the run-4 rule): the note carries the
            # measured distance instead.
            self.assertNotIn('edge', by_ref[ref])
            self.assertIn('measured', by_ref[ref]['note'])
        self.assertEqual(by_ref['J1']['class'], 'edge_receptacle')

    def test_interior_connector_flags_advisory_pass_survives(self):
        self.assertEqual(self.gr.returncode, 0, self.gr.stdout[-400:])
        doc = json.load(open(self.grade, encoding='utf-8'))
        self.assertTrue(doc['pass'])
        sev = [v['severity'] for v in doc['violations']]
        self.assertNotIn('error', sev)
        # J4 sits 6.03mm interior: flagged, as a WARNING.
        j4 = [v for v in doc['violations'] if v['ref'] == 'J4']
        self.assertEqual([v['severity'] for v in j4], ['warn'])
        self.assertEqual(j4[0]['expected'], {'max_setback_mm': 3.0})
        self.assertIn('J4 is an edge part seated', self.gr.stdout)
        self.assertIn('[warn ]', self.gr.stdout)
        # J7 (2.55mm, inside the affinity band) must NOT be flagged --
        # legitimately-interior connectors are the false-positive guard.
        self.assertNotIn('J7 is an edge part seated', self.gr.stdout)

    def test_healthy_board_gains_no_errors(self):
        with tempfile.TemporaryDirectory() as td:
            intent = os.path.join(td, 'i.json')
            r = _cf(SPLITFLAP, '--emit-intent', intent, '--declare-classes')
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            g = os.path.join(td, 'g.json')
            r2 = _cf(SPLITFLAP, '--intent', intent, '--json', g)
            self.assertEqual(r2.returncode, 0, r2.stdout[-400:])
            doc = json.load(open(g, encoding='utf-8'))
            self.assertTrue(doc['pass'])
            self.assertNotIn(
                'error', [v['severity'] for v in doc['violations']])


if __name__ == '__main__':
    unittest.main(verbosity=1)
