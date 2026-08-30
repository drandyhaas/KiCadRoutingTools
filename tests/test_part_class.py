"""Run-4 A: the part-class KB and its consumers.

The class question is pose-INDEPENDENT ("what determines this part's
position"); pose enters only through the separate plausibility verdict.
The named tigard refs appear here as INSTANCES of their classes -- the
rules themselves carry no board-specific values (generalization gate).

The false-positive guard is J7: a JST wire-to-board connector is
legitimately interior and must never classify as an edge part.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
from run_utils import tool as _tool, tool_env as _tool_env  # #522: resolve a moved CLI, loudly
from placement.part_class import (classify_part, default_band,  # noqa: E402
                                  pose_plausible)

SWAP = os.path.join(ROOT, 'wk', 'b2', 'tigard__swap', 'd0',
                    'perturbed.kicad_pcb')
CONTROL = os.path.join(ROOT, 'wk', 'b2', 'tigard__swap', 'd0',
                       'perturbed.control.kicad_pcb')


class _FP:
    """Minimal footprint stub: name + pads with pad_type/pinfunction."""
    def __init__(self, name, pads=()):
        self.footprint_name = name
        self.pads = list(pads)


class _Pad:
    def __init__(self, pad_type='smd', pinfunction=None,
                 global_x=0.0, global_y=0.0):
        self.pad_type = pad_type
        self.pinfunction = pinfunction
        self.global_x = global_x
        self.global_y = global_y


class TestClassifier(unittest.TestCase):
    def test_usb_receptacle_by_name_and_pins(self):
        fp = _FP('tigard:USB_C_Receptacle_HRO_TYPE-C-31-M-12',
                 [_Pad(pinfunction='CC1'), _Pad(pinfunction='VBUS')])
        pc = classify_part(fp, 'J1')
        self.assertEqual(pc.name, 'edge_receptacle')
        self.assertEqual(pc.confidence, 'high')

    def test_receptacle_needs_name_or_pinfunction_never_prefix_alone(self):
        pc = classify_part(_FP('house:Mystery_1x04', [_Pad()]), 'J9')
        self.assertIsNone(pc.name)

    def test_jst_wire_to_board_is_not_edge_class(self):
        """THE false-positive guard: run 3's J7."""
        fp = _FP('Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal',
                 [_Pad() for _ in range(4)])
        pc = classify_part(fp, 'J7')
        self.assertNotIn(pc.name, ('edge_receptacle', 'edge_actuator'))

    def test_slide_switch_is_edge_actuator(self):
        pc = classify_part(_FP('tigard:SW_ALPS_SSSS224100_DP4T',
                               [_Pad() for _ in range(10)]), 'SW1')
        self.assertEqual(pc.name, 'edge_actuator')

    def test_npth_only_part_is_mount_hole(self):
        pc = classify_part(_FP('MountingHole:MountingHole_3.2mm_M3',
                               [_Pad(pad_type='np_thru_hole')]), 'H1')
        self.assertEqual(pc.name, 'mount_hole')
        self.assertEqual(pc.confidence, 'high')

    def test_pinheader_is_not_edge_class(self):
        pc = classify_part(_FP('Connector_PinHeader_2.54mm:PinHeader_1x09',
                               [_Pad() for _ in range(9)]), 'J3')
        self.assertNotIn(pc.name, ('edge_receptacle', 'edge_actuator'))


class TestPlausibility(unittest.TestCase):
    def test_receptacle_interior_is_implausible(self):
        # run-3 J1 calibration: outside 0.0, edge_clearance 2.45
        self.assertFalse(pose_plausible('edge_receptacle', 0.0, 2.45))

    def test_receptacle_overhanging_or_seated_is_plausible(self):
        self.assertTrue(pose_plausible('edge_receptacle', 1.6, 0.0))
        self.assertTrue(pose_plausible('edge_receptacle', 0.0, 0.3))

    def test_actuator_makes_no_claim(self):
        # tactile buttons are legitimately interior
        self.assertTrue(pose_plausible('edge_actuator', 0.0, 20.0))

    def test_unknown_geometry_is_ungraded_not_false(self):
        self.assertIsNone(pose_plausible('edge_receptacle', None, None))

    def test_bands_are_part_geometry_not_board_history(self):
        b = default_band('edge_actuator', _FP('x', []), observed_overhang=1.6)
        self.assertEqual(b, {'min': 0.0, 'max': 2.1})
        b = default_band('edge_actuator', _FP('x', []))
        self.assertEqual(b, {'min': 0.0, 'max': 3.0})
        pads = [_Pad(global_x=0, global_y=0), _Pad(global_x=6.6, global_y=10.6)]
        b = default_band('edge_receptacle', _FP('x', pads))
        self.assertEqual(b['min'], 0.0)
        self.assertLessEqual(b['max'], 2.0)


def _run(tool, *argv):
    env = dict(_tool_env(), PYTHONPATH=_tool_env()["PYTHONPATH"], PYTHONIOENCODING='utf-8')
    return subprocess.run(
        [sys.executable, '-X', 'utf8', _tool(tool)] + list(argv),
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestAdvisorDemotion(unittest.TestCase):
    def test_implausible_receptacle_leaves_the_paste_list(self):
        """Run 3's J1: three lexical connector signals, MEDIUM -- a
        medium-cutoff round trip would have frozen it 15.8mm from its edge.
        The class demotion drops it to LOW, out of the default list."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        with tempfile.TemporaryDirectory() as td:
            js = os.path.join(td, 'advice.json')
            r = _run('place_optimize.py', SWAP, '--suggest-locks',
                     '--suggest-locks-json', js)
            self.assertEqual(r.returncode, 0, r.stdout[-500:] + r.stderr[-500:])
            doc = json.load(open(js, encoding='utf-8'))
            j1 = [f for f in doc['findings'] if f['ref'] == 'J1']
            self.assertTrue(j1)
            self.assertEqual(j1[0]['part_class'], 'edge_receptacle')
            self.assertIs(j1[0]['pose_plausible'], False)
            self.assertEqual(j1[0]['confidence'], 'low')
            self.assertNotIn('J1', doc['lock_patterns'])

    def test_control_board_receptacle_stays_confident(self):
        if not os.path.exists(CONTROL):
            self.skipTest('control board not present')
        with tempfile.TemporaryDirectory() as td:
            js = os.path.join(td, 'advice.json')
            r = _run('place_optimize.py', CONTROL, '--suggest-locks',
                     '--suggest-locks-json', js)
            self.assertEqual(r.returncode, 0)
            doc = json.load(open(js, encoding='utf-8'))
            j1 = [f for f in doc['findings'] if f['ref'] == 'J1']
            self.assertTrue(j1)
            self.assertIsNot(j1[0]['pose_plausible'], False)
            self.assertNotEqual(j1[0]['confidence'], 'low')


class TestDeclareClasses(unittest.TestCase):
    def test_damaged_board_declares_j1_and_sw1_not_j7(self):
        """Run 3's hand intent had only SW1; the auto pipeline must declare
        BOTH edge parts with zero hand edits, and not the JST."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'intent.json')
            r = _run('check_floorplan.py', SWAP, '--emit-intent', out,
                     '--declare-classes')
            self.assertEqual(r.returncode, 0, r.stdout[-500:] + r.stderr[-500:])
            doc = json.load(open(out, encoding='utf-8'))
            by_ref = {c['ref']: c for c in doc['edge_connectors']}
            self.assertIn('J1', by_ref)
            self.assertIn('SW1', by_ref)
            # The original pin here was `J7 not declared at all` -- written
            # when generic connectors produced NO entry. The invariant it
            # protected was narrower: J7 (a JST wire-to-board part,
            # legitimately interior) must never be given an EDGE. run-23's
            # connector_affinity class declares it -- weakly, no edge, no
            # implausibility claim -- so mid-board connectors stop being
            # invisible to every rule. The protected invariant stands:
            self.assertEqual(by_ref['J7'].get('class'), 'connector_affinity')
            self.assertNotIn('edge', by_ref['J7'])
            # J1's pose is implausible: class-default band, NO edge invented
            self.assertEqual(by_ref['J1'].get('class'), 'edge_receptacle')
            self.assertNotIn('edge', by_ref['J1'])
            # SW1's overhang READS as by-design but the board's own pattern
            # vectors say a displaced pose lands there: run-5 suspect-and-
            # derive withholds the edge (run 4 blessed it, and the blessed
            # wrong overhang was the user-named recurrence). Class-only.
            self.assertEqual(by_ref['SW1'].get('class'), 'edge_actuator')
            self.assertNotIn('edge', by_ref['SW1'])
            self.assertIn('SUSPECT', by_ref['SW1'].get('note') or '')

    def test_damaged_board_fails_the_proximity_rule(self):
        """--declare-classes deliberately breaks grades-clean-by-construction
        on a DAMAGED board: the implausibly-posed receptacle must FAIL."""
        if not os.path.exists(SWAP):
            self.skipTest('swap corpus board not present')
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'intent.json')
            _run('check_floorplan.py', SWAP, '--emit-intent', out,
                 '--declare-classes')
            r = _run('check_floorplan.py', SWAP, '--intent', out)
            self.assertEqual(r.returncode, 4, r.stdout[-800:])
            self.assertIn('J1', r.stdout)
            self.assertIn('mating face cannot reach the edge', r.stdout)

    def test_control_board_auto_intent_grades_clean(self):
        if not os.path.exists(CONTROL):
            self.skipTest('control board not present')
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'intent.json')
            r = _run('check_floorplan.py', CONTROL, '--emit-intent', out,
                     '--declare-classes')
            self.assertEqual(r.returncode, 0)
            r = _run('check_floorplan.py', CONTROL, '--intent', out)
            self.assertEqual(r.returncode, 0, r.stdout[-800:])


if __name__ == '__main__':
    unittest.main()
