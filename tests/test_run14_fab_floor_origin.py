#!/usr/bin/env python3
"""The fab-floor banner must baseline on the board's ORIGINAL floor (run 14).

`_fab_floor_disclosure` compared the output project against its IMMEDIATE
input, which is correct for a one-step run and silent for every step after the
first in a chain. Measured on run 14 (castor_pollux):

    R1  min_via_diameter 0.5 -> 0.25   banner fired, once
    R4  added 7 more sub-0.5 vias      no banner: input was already 0.25
    R5  added 10 more                  no banner, same reason
    final board: 10 vias under the board's own declared 0.5 mm, and
                 check_drc / board_score / KiCad's DRC all read clean

So the origin is now recorded in the project on the first writeback
(`kicad_routing_tools.fab_floor_origin`) and carried down the chain, and a step
that merely INHERITS an already-relaxed floor still says the board is under its
original declaration.

Also pinned here: the writeback lists the keys it moved instead of printing
only a count. Run 14's netclass `via_diameter` went 0.6 -> 0.25 and
`min_hole_clearance` 0.25 -> 0.175 inside a "wrote 17 value(s)" summary that
named three.
"""
import io
import json
import contextlib
import os
import shutil
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

import fix_kicad_drc_settings as F  # noqa: E402

ORIGIN_KEY = 'fab_floor_origin'


def _proj(rules, origin=None):
    p = {"board": {"design_settings": {"rules": dict(rules),
                                       "rule_severities": {}}},
         "net_settings": {"classes": [], "meta": {"version": 0}}}
    if origin is not None:
        p["kicad_routing_tools"] = {ORIGIN_KEY: dict(origin)}
    return p


class OriginBaselineTest(unittest.TestCase):
    """_fab_floor_disclosure, unit level."""

    def test_first_step_behaves_exactly_as_before(self):
        before = {"min_via_diameter": 0.5}
        after = _proj({"min_via_diameter": 0.25})
        out = F._fab_floor_disclosure('nonexistent.kicad_pcb', before, after)
        self.assertTrue(out)
        self.assertIn('0.5 -> 0.25', ' '.join(out))

    def test_inherited_relaxation_is_still_reported(self):
        """THE RUN-14 CASE: input already 0.25, output 0.25, origin 0.5."""
        before = {"min_via_diameter": 0.25}
        after = _proj({"min_via_diameter": 0.25})
        out = F._fab_floor_disclosure('nonexistent.kicad_pcb', before, after,
                                      {"min_via_diameter": 0.5})
        self.assertTrue(out, 'a step under the ORIGINAL floor must not be silent')
        text = ' '.join(out)
        self.assertIn('still below', text)
        self.assertIn('ORIGINAL 0.5', text)

    def test_without_origin_the_inherited_case_is_silent(self):
        """Documents the old behaviour this test exists to change."""
        before = {"min_via_diameter": 0.25}
        after = _proj({"min_via_diameter": 0.25})
        self.assertEqual(
            F._fab_floor_disclosure('nonexistent.kicad_pcb', before, after),
            [], 'no origin -> no baseline -> nothing to say (the old bug)')

    def test_a_step_that_lowers_further_says_so_against_the_origin(self):
        before = {"min_via_diameter": 0.25}
        after = _proj({"min_via_diameter": 0.2})
        out = F._fab_floor_disclosure('nonexistent.kicad_pcb', before, after,
                                      {"min_via_diameter": 0.5})
        text = ' '.join(out)
        self.assertIn('0.5 -> 0.2', text, 'baseline is the origin, not 0.25')

    def test_tightening_back_to_the_origin_is_silent(self):
        before = {"min_via_diameter": 0.25}
        after = _proj({"min_via_diameter": 0.5})
        self.assertEqual(
            F._fab_floor_disclosure('nonexistent.kicad_pcb', before, after,
                                    {"min_via_diameter": 0.5}),
            [], 'a floor restored to its original is not a relaxation')

    def test_clearance_is_still_not_a_fab_floor(self):
        before = {"min_clearance": 0.2}
        after = _proj({"min_clearance": 0.1562})
        self.assertEqual(
            F._fab_floor_disclosure('nonexistent.kicad_pcb', before, after,
                                    {"min_clearance": 0.2}), [],
            'clearance is a grading decision, not a manufacturing floor')


class ChainCarryTest(unittest.TestCase):
    """fix_project_for_output, on real files, two steps deep."""

    BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='ffo14_')

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _stage(self, name, rules):
        pcb = os.path.join(self.tmp, name + '.kicad_pcb')
        shutil.copyfile(self.BOARD, pcb)
        with open(os.path.join(self.tmp, name + '.kicad_pro'), 'w') as f:
            json.dump(_proj(rules), f)
        return pcb

    def _run(self, pcb, **kw):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            F.fix_project_for_output(pcb, verbose=True, **kw)
        return buf.getvalue()

    def test_origin_is_seeded_then_carried(self):
        pcb = self._stage('s1', {"min_via_diameter": 0.5,
                                 "min_track_width": 0.2})
        out1 = self._run(pcb, via_diameter=0.25)
        with open(os.path.join(self.tmp, 's1.kicad_pro')) as f:
            pro = json.load(f)
        origin = (pro.get('kicad_routing_tools') or {}).get(ORIGIN_KEY) or {}
        self.assertEqual(origin.get('min_via_diameter'), 0.5,
                         'the first writeback must record the original floor')
        self.assertIn('FAB FLOOR RELAXED', out1)

        # step 2: same project, nothing further to lower. The old code said
        # nothing at all here; it must now still report the board is under its
        # own original declaration.
        out2 = self._run(pcb, via_diameter=0.25)
        self.assertIn('ORIGINAL 0.5', out2,
                      'step 2 inherited a relaxed floor and must still say so')

    def test_writeback_lists_the_keys_it_moved(self):
        pcb = self._stage('s2', {"min_via_diameter": 0.5})
        out = self._run(pcb, via_diameter=0.25)
        self.assertIn('wrote', out)
        self.assertIn('rules.min_via_diameter', out,
                      'the moved keys must be listed, not just counted')

    def test_origin_survives_a_project_copy_to_a_new_output(self):
        """copy_board / fix_project_for_output carry it, like protected_nets."""
        src = self._stage('s3', {"min_via_diameter": 0.5})
        self._run(src, via_diameter=0.25)
        dst = os.path.join(self.tmp, 's4.kicad_pcb')
        shutil.copyfile(src, dst)
        out = self._run(dst, input_pcb=src, via_diameter=0.25)
        with open(os.path.join(self.tmp, 's4.kicad_pro')) as f:
            pro = json.load(f)
        self.assertEqual(
            ((pro.get('kicad_routing_tools') or {}).get(ORIGIN_KEY) or {})
            .get('min_via_diameter'), 0.5)
        self.assertIn('ORIGINAL 0.5', out)


if __name__ == '__main__':
    unittest.main(verbosity=2)
