"""A header seated flush on its edge is not a part leaving the board.

`place_simple` seats every `edge_connectors[]` row that names an edge, a
`connector_affinity` header's included, flush with the low end of its
overhang band, read on the drawn body. A header's courtyard is drawn round
that body, so it stands past the edge by design -- and `oob_count` counted
it: a run that declared `legality_budget {oob_count: 0}` had every one of
its seeds graded with a legality ERROR for one header 0.78 mm past the
edge. The grade's exemption now reads the parts the seat holds: a row that
names an edge, whatever its class, and a band with no `max` ends at its
`min` (where the seat puts it), never unbounded.

Pinned on a synthetic board (40 x 30 mm): P1's F.Fab body ends ON the east
edge, its courtyard 0.5 mm past it, its pads inside.
"""

import os
import shutil
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                              # noqa: E402


def _board(td, x):
    """The board with P1 at (x, 15): body x in [x-1.5, x+2], courtyard
    [x-2, x+2.5], pads at x."""
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n'
            f'  (footprint "t:HDR" (layer "F.Cu") (at {x} 15)\n'
            '    (property "Reference" "P1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            '    (fp_rect (start -1.5 -3) (end 2 3) (stroke (width 0.1)'
            ' (type default)) (layer "F.Fab"))\n'
            '    (fp_rect (start -2 -3.5) (end 2.5 3.5) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at 0 -1.27) (size 1 1) (layers "F.Cu") (net 1 "N1"))\n'
            '    (pad "2" smd rect (at 0 1.27) (size 1 1) (layers "F.Cu") (net 2 "N2"))\n'
            '  )\n)\n')
    path = os.path.join(td, f'b{x}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _intent(row):
    return floorplan.intent_from_dict(
        {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
         'legality_budget': {'oob_count': 0, 'oob_amount': 0.0},
         'edge_connectors': [row]}, '')


HEADER = {'ref': 'P1', 'class': 'connector_affinity', 'edge': 'east',
          'overhang_mm': {'min': 0.0}}


class FlushHeader(unittest.TestCase):

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='krt_flush_')

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def _grade(self, row, x):
        path = _board(self.td, x)
        return floorplan.grade(_intent(row), parse_kicad_pcb(path), path)

    @staticmethod
    def _legality(res):
        return [v for v in res.violations if v.rule == 'legality']

    def test_flush_on_its_edge_is_exempt(self):
        r = self._grade(HEADER, 38.0)
        self.assertEqual(r.legality['oob_count'], 1, r.legality)   # the census counts it
        self.assertEqual(r.legality['oob_count_exempt'], 1, r.legality)
        self.assertEqual(self._legality(r), [], [v.message for v in self._legality(r)])

    def test_past_its_band_is_counted(self):
        # the body 1 mm past the edge: outside a flush band
        r = self._grade(HEADER, 39.0)
        self.assertEqual(r.legality['oob_count_exempt'], 0, r.legality)
        oob = [v for v in self._legality(r) if 'oob_count' in v.expected]
        self.assertEqual(len(oob), 1, [v.message for v in self._legality(r)])

    def test_a_row_naming_no_edge_vouches_for_nothing(self):
        row = {k: v for k, v in HEADER.items() if k != 'edge'}
        r = self._grade(row, 38.0)
        self.assertEqual(r.legality['oob_count_exempt'], 0, r.legality)
        self.assertTrue([v for v in self._legality(r) if 'oob_count' in v.expected])

    def test_a_receptacle_with_no_max_is_flush_too(self):
        r = self._grade(dict(HEADER, **{'class': 'edge_receptacle'}), 38.0)
        self.assertEqual(r.legality['oob_count_exempt'], 1, r.legality)
        self.assertEqual(self._legality(r), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
