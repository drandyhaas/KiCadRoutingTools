"""The cap step holds a cap pad off a via's DRILL at the copper-to-hole
clearance KiCad grades, not only off its copper at the copper clearance.

check_drc grades another net's pad copper against a via's hole at the
project's `min_hole_clearance` (#1914), and a `.kicad_dru` hole_clearance rule
that binds the pair replaces that value. A via whose ring is narrower than the
difference between the two clearances (a 0.3/0.2 via, ring 0.05, at a 0.1
copper clearance under a 0.2 hole clearance) leaves a pad that clears its
copper inside the hole band, and the cap step used to read that pad clean.

The rig: a real board cut to a BGA and one cap, and one foreign via beside a
cap pad, its copper 0.02 mm clear of the copper clearance and its hole 0.03
mm inside the hole clearance. The truth is check_drc's own predicate
(`check_drc.pad_via_hole_required`) and distance (`point_to_pad_distance`) on
the real pads, turned by `legality.footprint_at_pose`.

1. The pair is charged at the seed, by exactly the hole graze.
2. The pass moves the cap clear of it, as check_drc measures the moved pads.
3. A rule below the project value replaces it: nothing is charged.
4. A rule above it raises the charge to the rule.
5. A board whose project declares nothing is held at the NPTH floor, the
   value the step's project writeback declares for it.

Runs in-process in a few seconds; nothing shells out.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True

import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

from kicad_parser import parse_kicad_pcb
from check_drc import (HoleRuleBinder, board_design_rules, pad_via_hole_required,
                       point_to_pad_distance)
from synth import make_via
from placement.legality import footprint_at_pose
from placement.fanout_clearance import (_Repair, repair_fanout_clearance,
                                        resolve_via_hole_clearance)
import routing_defaults as defaults

BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
CAP, BGA = 'C16', 'U2'
CLEAR = 0.1
HC = 0.2
NET = 5000
SIZE, DRILL = 0.3, 0.2
# the via's copper this far past the copper clearance from the pad...
COPPER_SLACK = 0.02
# ...which leaves its hole this far inside the hole clearance
HOLE_GRAZE = (DRILL / 2 + HC) - (SIZE / 2 + CLEAR + COPPER_SLACK)


def _copper(fp):
    return [p for p in fp.pads if any(str(l).endswith('.Cu') for l in p.layers)]


class _Rig:
    def __init__(self, declared=HC, dru=None):
        self.tmp = tempfile.mkdtemp()
        self.path = os.path.join(self.tmp, 'b.kicad_pcb')
        shutil.copyfile(BOARD, self.path)
        rules = {} if declared is None else {'min_hole_clearance': declared}
        with open(os.path.join(self.tmp, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({'board': {'design_settings': {'rules': rules}}}, fh)
        if dru is not None:
            with open(os.path.join(self.tmp, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
                fh.write(dru)
        with contextlib.redirect_stdout(io.StringIO()):
            self.pcb = parse_kicad_pcb(self.path)
        for ref in list(self.pcb.footprints):
            if ref not in (CAP, BGA):
                del self.pcb.footprints[ref]
        self.fp = self.pcb.footprints[CAP]
        pad = _copper(self.fp)[0]
        # beyond the pad's outer edge, on the cap's axis (away from its other pad)
        ux, uy = pad.global_x - self.fp.x, pad.global_y - self.fp.y
        n = (ux * ux + uy * uy) ** 0.5
        ux, uy = ux / n, uy / n
        half = abs(pad.size_x if abs(ux) > abs(uy) else pad.size_y) / 2.0
        reach = half + SIZE / 2 + CLEAR + COPPER_SLACK
        self.pad = pad
        self.via = make_via(pad.global_x + ux * reach, pad.global_y + uy * reach,
                            net_id=NET, size=SIZE, drill=DRILL)
        self.pcb.vias[:] = [self.via]
        self.pcb.segments[:] = []

    def state(self):
        with contextlib.redirect_stdout(io.StringIO()):
            return _Repair(self.pcb, self.path, CLEAR, 0.1, 0.55, 1.0, 2.0, 0.3,
                           'C,R,FB', set())

    def close(self):
        shutil.rmtree(self.tmp, ignore_errors=True)


def _graze(pads, via, req):
    """check_drc's overlap for each pad against the via's hole."""
    return max(req - (point_to_pad_distance(via.x, via.y, p) - via.drill / 2.0)
               for p in pads)


class TestTheRig(unittest.TestCase):
    def test_copper_clears_and_the_hole_does_not(self):
        rig = _Rig()
        try:
            d = point_to_pad_distance(rig.via.x, rig.via.y, rig.pad)
            self.assertAlmostEqual(d - SIZE / 2, CLEAR + COPPER_SLACK, places=6,
                                   msg='the via copper is not where the rig puts it')
            self.assertGreater(HOLE_GRAZE, 0.01, 'the hole clears: this tests nothing')
            self.assertIn(CAP, rig.state().caps, 'the cap is not a mover')
        finally:
            rig.close()


class TestThePairIsPricedAtTheHole(unittest.TestCase):
    def test_seed_charge_is_the_hole_graze(self):
        rig = _Rig()
        try:
            st = rig.state()
            cap = st.caps[CAP]
            req = pad_via_hole_required(rig.pad, rig.via, HC,
                                        rig.pcb.board_info.copper_layers)
            self.assertAlmostEqual(req, HC, places=9)
            self.assertAlmostEqual(st.via_hole_keepout(rig.pad, rig.via),
                                   DRILL / 2 + HC, places=9)
            got = st._via_shortfalls(CAP, cap, cap.x, cap.y, cap.rot)
            self.assertAlmostEqual(got.get(NET, 0.0), HOLE_GRAZE, places=6,
                                   msg='the seed charge is not the hole graze')
        finally:
            rig.close()

    def test_the_pass_moves_the_cap_clear_of_the_hole(self):
        rig = _Rig()
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                res = repair_fanout_clearance(rig.pcb, rig.path, clearance=CLEAR,
                                              near_margin=8.0)
            moved = {p['reference']: p for p in res['placements']}
            self.assertIn(CAP, moved, 'the cap did not move')
            self.assertNotIn(CAP, res.get('unresolved') or ())
            # the pass counted the hole graze as the violation it cleared
            self.assertIn(CAP, res.get('resolved') or (),
                          'the cap was not an initial violation the pass resolved')
            m = moved[CAP]
            pads = _copper(footprint_at_pose(rig.fp, (m['new_x'], m['new_y'],
                                                      m['new_rotation'])))
            self.assertLessEqual(_graze(pads, rig.via, HC), 1e-6,
                                 'check_drc still measures the moved pads inside '
                                 'the hole clearance')
        finally:
            rig.close()


class TestARuleReplacesTheProjectValue(unittest.TestCase):
    def _charge(self, rule_min):
        rig = _Rig(dru='(version 1)\n(rule hc (constraint hole_clearance '
                       f'(min {rule_min}mm)))\n')
        try:
            st = rig.state()
            cap = st.caps[CAP]
            binder = HoleRuleBinder(board_design_rules(rig.pcb))
            req = pad_via_hole_required(rig.pad, rig.via, HC,
                                        rig.pcb.board_info.copper_layers, binder)
            return req, st._via_shortfalls(CAP, cap, cap.x, cap.y, cap.rot).get(NET, 0.0)
        finally:
            rig.close()

    def test_a_lower_rule_charges_nothing(self):
        req, got = self._charge(0.12)
        self.assertAlmostEqual(req, 0.12, places=9)
        self.assertEqual(got, 0.0, 'the project value was charged under a rule '
                                   'that replaces it')

    def test_a_higher_rule_charges_the_rule(self):
        req, got = self._charge(0.25)
        self.assertAlmostEqual(req, 0.25, places=9)
        self.assertAlmostEqual(got, HOLE_GRAZE + 0.05, places=6)


class TestAnUndeclaredProject(unittest.TestCase):
    def test_held_at_the_floor_the_writeback_declares(self):
        rig = _Rig(declared=None)
        try:
            v, src = resolve_via_hole_clearance(rig.pcb, rig.path)
            self.assertAlmostEqual(v, defaults.NPTH_TO_TRACK_CLEARANCE, places=9)
            self.assertIn('NPTH floor', src)
            self.assertAlmostEqual(rig.state().via_hole_clearance, v, places=9)
        finally:
            rig.close()


if __name__ == '__main__':
    unittest.main()
