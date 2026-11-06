"""#1686: a via the nudge lands below the board's own min_hole_to_hole leaves a
hole pair KiCad fails, so the part it freed is not resolved and the run says so.

The ladder (#756) prefers the board's floor and falls back to the fab floor
rather than lose the repair. It said so in one line about the floor and then
counted the part resolved, ending in `0 unresolved`. The landing itself stands:
measured on a board declaring 0.45, the via left where it stood is a SHORT
(KiCad shorting_items) where the landing is a 0.327 mm hole pair above the 0.2
fab floor. So what changes is the verdict.

The rig: a real board cut to a BGA and one cap that cannot move
(its displacement budgets 0), a foreign via on its pad, and a ring of that via's
own net around it, close enough that no landing within the nudge's reach clears
a declared 0.45 and some clear the 0.2 fab floor.

1. Declared 0.45: the via lands, the drill gap really is below 0.45 and at or
   above 0.2, the cap is in `unresolved` and not `resolved` or `via_resolved`,
   `below_hole_floor` carries the pair, and the run prints it.
2. Declared at the fab floor: the same landing is simply a repair, the control.

Runs in-process in a few seconds; nothing shells out.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True

import contextlib
import io
import json
import math
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
from check_drc import point_to_pad_distance
from synth import make_via
from placement.fanout_clearance import repair_fanout_clearance

BOARD = os.path.join(_ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
CAP, BGA = 'C16', 'U2'
CLEAR = 0.1
NET = 5000
SIZE, DRILL = 0.3, 0.3 - 0.1
RING = 0.95


def _run(declared):
    tmp = tempfile.mkdtemp()
    try:
        path = os.path.join(tmp, 'b.kicad_pcb')
        shutil.copyfile(BOARD, path)
        with open(os.path.join(tmp, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({'board': {'design_settings': {'rules': {
                'min_hole_to_hole': declared}}}}, fh)
        pcb = parse_kicad_pcb(path)
        fp = pcb.footprints[CAP]
        for ref in list(pcb.footprints):
            if ref not in (CAP, BGA):
                del pcb.footprints[ref]
        pads = [p for p in fp.pads if any(str(l).endswith('.Cu') for l in p.layers)]
        target = make_via(pads[0].global_x, pads[0].global_y, net_id=NET,
                          size=SIZE, drill=DRILL)
        ring = []
        for k in range(24):
            a = k * math.pi / 12
            v = make_via(target.x + RING * math.cos(a),
                         target.y + RING * math.sin(a), net_id=NET,
                         size=SIZE, drill=DRILL)
            # only the one via is the cap's offender
            if all(point_to_pad_distance(v.x, v.y, p) >= SIZE / 2 + CLEAR + 1e-3
                   for p in pads):
                ring.append(v)
        pcb.vias[:] = [target] + ring
        pcb.segments[:] = []
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res = repair_fanout_clearance(pcb, path, clearance=CLEAR,
                                          allow_rotations=False,
                                          max_displacement=0.0,
                                          max_displacement_cap=0.0,
                                          near_margin=8.0)
        gap = min(math.hypot(target.x - v.x, target.y - v.y) - DRILL
                  for v in ring)
        return res, gap, buf.getvalue()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


class TestBelowTheBoardsFloor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res, cls.gap, cls.out = _run(0.45)

    def test_the_rig_landed_below_the_floor(self):
        self.assertEqual(len(self.res['via_moves']), 1,
                         'the nudge did not move the via: this tests nothing')
        self.assertLess(self.gap, 0.45)
        self.assertGreaterEqual(self.gap, 0.2 - 1e-6)
        self.assertIn('fell back to the 0.2mm fab floor', self.out)

    def test_the_part_is_not_resolved(self):
        self.assertIn(CAP, self.res['unresolved'])
        self.assertNotIn(CAP, self.res['resolved'])
        self.assertNotIn(CAP, self.res['via_resolved'])
        self.assertIn('1 unresolved', self.out)
    # MUTATION: `or ref in below_parts` deleted from _grade -> `0 unresolved`.

    def test_the_pair_is_carried_and_printed(self):
        rows = self.res['below_hole_floor']
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['part'], CAP)
        self.assertEqual(rows[0]['board_mm'], 0.45)
        self.assertAlmostEqual(rows[0]['gap_mm'], self.gap, places=3)
        self.assertIn('BELOW min_hole_to_hole, KiCad fails this hole pair', self.out)

    def test_a_short_is_not_also_claimed(self):
        self.assertEqual(self.res['shorted'], [],
                         'the via moved off the pad: nothing is in contact')
        self.assertEqual(self.res['parted'], [],
                         "a part named below the floor no longer grazes, so "
                         "the 'only grazes' line would be false")


class TestAtTheFabFloor(unittest.TestCase):
    def test_the_same_landing_is_a_plain_repair(self):
        res, gap, out = _run(0.2)
        self.assertEqual(len(res['via_moves']), 1)
        self.assertGreaterEqual(gap, 0.2 - 1e-6)
        self.assertEqual(res['below_hole_floor'], [])
        self.assertIn(CAP, res['via_resolved'])
        self.assertEqual(res['unresolved'], [])
        self.assertNotIn('BELOW min_hole_to_hole', out)


if __name__ == '__main__':
    unittest.main()
