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


class MidRunWriterSeedsOriginTest(unittest.TestCase):
    """The #650 mid-run copper writer must seed the origin too.

    `apply_routed_floors` runs BEFORE the authoritative writeback (route.py
    calls it so the in-run plane/oracle audit grades what ships), and it can
    lower `min_hole_clearance`. It did that without recording
    `fab_floor_origin`, so the writeback then seeded the origin from the
    ALREADY-LOWERED value and compared it against itself.

    Measured on eurorack_pmod (6-layer, declares min_hole_clearance 0.25): the
    mid-run pass took it straight to 0.127, the origin recorded 0.127, and
    `FAB FLOOR RELAXED` said NOTHING about a real 0.25 -> 0.127 relaxation. On
    rp2350_dev the mid-run pass stopped at 0.2, so the banner fired but
    understated the relaxation as "0.2 -> 0.127".

    MUTATION: drop the seed_fab_floor_origin call from `apply_routed_floors` --
    both arms below die.
    """

    BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='ffo650_')
        self.pcb = os.path.join(self.tmp, 'm.kicad_pcb')
        shutil.copyfile(self.BOARD, self.pcb)
        with open(os.path.join(self.tmp, 'm.kicad_pro'), 'w') as f:
            json.dump(_proj({"min_hole_clearance": 0.25}), f)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _origin(self):
        with open(os.path.join(self.tmp, 'm.kicad_pro')) as f:
            pro = json.load(f)
        return (pro.get('kicad_routing_tools') or {}).get(ORIGIN_KEY) or {}

    def test_mid_run_pass_records_the_declared_floor_before_lowering_it(self):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            changes = F.apply_routed_floors(self.pcb, clearance=0.127,
                                            verbose=True)
        self.assertTrue(changes, 'the mid-run pass lowered nothing to test')
        self.assertEqual(
            self._origin().get('min_hole_clearance'), 0.25,
            'the mid-run pass lowered a fab floor without recording the '
            'board ORIGINAL, so the writeback will baseline on its own output')

    def test_the_relaxation_is_still_disclosed_after_the_mid_run_pass(self):
        """End to end: mid-run pass, then the writeback. The banner must name
        the board's 0.25, which is the half that went silent."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            F.apply_routed_floors(self.pcb, clearance=0.127, verbose=False)
            F.fix_project_for_output(self.pcb, clearance=0.127, verbose=True)
        out = buf.getvalue()
        self.assertIn('FAB FLOOR RELAXED', out,
                      'a real 0.25 -> 0.127 relaxation shipped with no banner')
        self.assertIn('ORIGINAL 0.25', out,
                      'the banner must baseline on the board 0.25, not on the '
                      'value the mid-run pass had already written')

    def test_a_second_mid_run_pass_does_not_re_seed(self):
        """Once recorded, the origin is the board's, not each pass's input."""
        with contextlib.redirect_stdout(io.StringIO()):
            F.apply_routed_floors(self.pcb, clearance=0.127, verbose=False)
            F.apply_routed_floors(self.pcb, clearance=0.1, verbose=False)
        self.assertEqual(self._origin().get('min_hole_clearance'), 0.25)


class LiveBoardOriginTest(unittest.TestCase):
    """The GUI's live-board writers (apply_targets_to_board, then
    gui_utils.update_live_drc_floors) lowered the same floors with no origin
    recorded, so a manual GUI run that relaxed a fab floor said nothing and a
    later CLI step baselined on the already-lowered value. Fakes stand in for
    pcbnew (design settings in nm); the real board runs in
    tests/gui_parity/test_live_fab_floor_origin.py."""

    class _BDS:
        def __init__(self, track, via):
            self.m_TrackMinWidth = int(track * 1e6)
            self.m_ViasMinSize = int(via * 1e6)

    class _Board:
        def __init__(self, path, bds):
            self.path, self.bds = path, bds

        def GetFileName(self):
            return self.path

        def GetDesignSettings(self):
            return self.bds

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.pcb = os.path.join(self.tmp, 'b.kicad_pcb')
        with open(self.pcb, 'w') as f:
            f.write('(kicad_pcb)\n')
        F._LIVE_FAB_ORIGIN.clear()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)
        F._LIVE_FAB_ORIGIN.clear()

    def _write_pro(self, origin=None):
        with open(self.pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w') as f:
            json.dump(_proj({"min_via_diameter": 0.5}, origin), f)

    def _pro_origin(self):
        with open(self.pcb[:-len('.kicad_pcb')] + '.kicad_pro') as f:
            return (json.load(f).get('kicad_routing_tools') or {}).get(ORIGIN_KEY)

    def test_the_live_floors_are_recorded_before_they_are_lowered(self):
        self._write_pro()
        bds = self._BDS(0.2, 0.5)
        board = self._Board(self.pcb, bds)
        origin = F.seed_live_fab_floor_origin(board)
        self.assertEqual(origin, {'min_track_width': 0.2, 'min_via_diameter': 0.5})
        self.assertEqual(self._pro_origin(), origin, 'recorded in the project')
        bds.m_ViasMinSize = int(0.3 * 1e6)          # the step lowers it
        again = F.seed_live_fab_floor_origin(board)
        self.assertEqual(again['min_via_diameter'], 0.5,
                         'a second writer must not re-seed from the lowered value')
        self.assertEqual(self._pro_origin()['min_via_diameter'], 0.5)

    def test_an_origin_already_in_the_project_wins(self):
        """A GUI step after a CLI chain keeps the chain's original."""
        self._write_pro(origin={'min_via_diameter': 0.8})
        board = self._Board(self.pcb, self._BDS(0.2, 0.3))
        self.assertEqual(F.seed_live_fab_floor_origin(board)['min_via_diameter'], 0.8)

    def test_a_board_with_no_project_keeps_it_for_the_session(self):
        bds = self._BDS(0.2, 0.5)
        board = self._Board(self.pcb, bds)
        F.seed_live_fab_floor_origin(board)
        bds.m_ViasMinSize = int(0.3 * 1e6)
        self.assertEqual(F.seed_live_fab_floor_origin(board)['min_via_diameter'], 0.5)
        self.assertFalse(os.path.exists(self.pcb[:-len('.kicad_pcb')] + '.kicad_pro'),
                         'no project is created just to hold the record')

    def test_the_live_disclosure_names_the_original_and_counts(self):
        origin = {'min_track_width': 0.2, 'min_via_diameter': 0.5}
        after = {'min_track_width': 0.2, 'min_via_diameter': 0.3}
        out = ' '.join(F.live_fab_floor_disclosure(
            origin, after, {'min_via_diameter': [0.3, 0.45, 0.6]}))
        self.assertIn('FAB FLOOR RELAXED', out)
        self.assertIn('via diameter: 0.5 -> 0.3 mm', out)
        self.assertIn('2 of 3 object(s)', out)
        self.assertNotIn('track width', out, 'an unmoved floor is not reported')
        self.assertEqual(F.live_fab_floor_disclosure(origin, dict(origin), {}), [],
                         'nothing under its origin -> silent')


if __name__ == '__main__':
    unittest.main(verbosity=2)
