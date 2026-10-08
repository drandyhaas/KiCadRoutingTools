#!/usr/bin/env python3
"""#1104: the courtyard waiver reaches every placement gate, not only the seat.

#1101 let the seeder's searched seats honour a project's own
`courtyards_overlap = ignore`. The later steps kept pricing courtyard
`overlap_area`: run 38's `--repair-decaps` reverted C2, C6, C11, C15, C16 and
C17 moves that would have CLEARED their decap finding (`"added":
["legality.overlap_area"], "still": false`), and `--reseat` was refused on
"overlap 78.3->79.1". Measured on run 38's rs_3_1: 4 such refusals -> 0, and
the repaired board's decap errors 4 -> 2, still 0 pad/hole conflicts at the
board's 0.15 mm and buildable.

Pinned here on a synthetic board: two parts whose courtyards overlap and
whose pads are far apart. On an `ignore` project every gate below must read
that pair as clean; at `error` it must still read the overlap; and a real pad
short must still count in both.
"""
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BOARD = (
    '(kicad_pcb (version 20240108) (generator pcbnew)\n'
    '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
    '  (net 0 "") (net 1 "A") (net 2 "B")\n'
    '  (gr_rect (start 0 0) (end 20 10) (stroke (width 0.1) (type default))'
    ' (layer "Edge.Cuts"))\n'
    '{parts})\n')
PART = (
    '  (footprint "t:{ref}" (layer "F.Cu") (at {x} 5)\n'
    '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -2.5 -1.5) (end 2.5 1.5) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (pad "1" smd rect (at -0.4 0) (size 0.5 0.5) (layers "F.Cu")'
    ' (net 1 "A"))\n'
    '    (pad "2" smd rect (at 0.4 0) (size 0.5 0.5) (layers "F.Cu")'
    ' (net 2 "B")))\n')

#: A at x=6, B at x=10: courtyards [3.5, 8.5] and [7.5, 12.5] overlap by
#: 1 x 3 mm; pads [5.35, 6.65] and [9.35, 10.65] are 2.7 mm apart.
OVERLAP_ONLY = (6, 10)
#: B at x=6.6: its pads land on A's (a short whatever the severity).
PAD_SHORT = (6, 6.6)


def make(td, xs, severity):
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD.format(parts=PART.format(ref='A', x=xs[0])
                              + PART.format(ref='B', x=xs[1])))
    sev = {} if severity is None else {'courtyards_overlap': severity}
    with open(os.path.join(td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rule_severities': sev}}},
                  fh)
    import pose_score
    from kicad_parser import parse_kicad_pcb
    return pose_score.make_state(parse_kicad_pcb(path), path, clearance=0.15,
                                 board_edge_clearance=0.1)


class TestWaiverReachesEveryGate(unittest.TestCase):
    def _both(self, xs, fn):
        out = {}
        for sev in ('ignore', 'error'):
            with tempfile.TemporaryDirectory() as td:
                st = make(td, xs, sev)
                self.assertEqual(st.courtyards_ignored, sev == 'ignore')
                out[sev] = fn(st)
        return out

    def test_legality_metrics_overlap_area(self):
        r = self._both(OVERLAP_ONLY, lambda s: s.legality_metrics())
        self.assertEqual(r['ignore']['overlap_area'], 0.0)
        self.assertGreater(r['ignore']['overlap_area_waived'], 2.9)
        self.assertGreater(r['error']['overlap_area'], 2.9)
        self.assertNotIn('overlap_area_waived', r['error'])

    def test_a_posed_view_keeps_the_waiver(self):
        from placement.floorplan import _PosedState
        r = self._both(OVERLAP_ONLY,
                       lambda s: _PosedState(s).legality_metrics())
        self.assertEqual(r['ignore']['overlap_area'], 0.0)
        self.assertGreater(r['error']['overlap_area'], 2.9)

    def test_violation_parts(self):
        r = self._both(OVERLAP_ONLY, lambda s: s.violation_parts('B')[1])
        self.assertEqual(r['ignore'], 0.0)
        self.assertGreater(r['error'], 0.0)
        short = self._both(PAD_SHORT, lambda s: s.violation_parts('B')[1])
        self.assertGreater(short['ignore'], 0.0)
        self.assertGreater(short['error'], 0.0)

    def test_seated_violations(self):
        from placement.seeder import _seated_violations
        r = self._both(OVERLAP_ONLY,
                       lambda s: _seated_violations(s, {'A', 'B'}))
        self.assertEqual(r['ignore'], (0, 0.0))
        self.assertEqual(r['error'][0], 1)
        short = self._both(PAD_SHORT,
                           lambda s: _seated_violations(s, {'A', 'B'}))
        self.assertGreaterEqual(short['ignore'][0], 1)

    def test_overlap_at(self):
        from placement.seeder import _overlap_at
        r = self._both(OVERLAP_ONLY, lambda s: _overlap_at(
            s, s.parts['B'], s.parts['B'].x, s.parts['B'].y, ['A']))
        self.assertEqual(r['ignore'], {})
        self.assertIn('A', r['error'])

    def test_reseat_cluster_clash(self):
        from placement.reseat import _clashes_with_seated

        def clash(xs):
            return self._both(xs, lambda s: _clashes_with_seated(
                s, 'B', (s.parts['B'].x, s.parts['B'].y, 0.0),
                {'A': (s.parts['A'].x, s.parts['A'].y, 0.0)}))
        r = clash(OVERLAP_ONLY)
        self.assertFalse(r['ignore'])
        self.assertTrue(r['error'])
        short = clash(PAD_SHORT)
        self.assertTrue(short['ignore'])


if __name__ == '__main__':
    unittest.main()
