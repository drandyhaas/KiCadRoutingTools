"""#1475: `blocks[].fence_mm`, a band round a block's zone no part may use.

A shield can's wall stands on a band round the circuit it covers, and the
band must stay clear while the members are still moving. The block declares
it once; the loader compiles it into four keep-outs round the zone (no
`allow`, so members are kept out too), and every reader of `keepouts` -- the
grade, the seat -- holds it.
"""

import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                              # noqa: E402
from test_1212_container_pins import _frame_board            # noqa: E402

ZONE = [12, 9, 28, 21]
FENCE = 1.5


def _intent(**block):
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
           'units': 'mm', 'min_reader': floorplan.FENCE_READER,
           'blocks': [dict({'name': 'can', 'refs': ['A', 'B'], 'zone': ZONE,
                            'side': 'F', 'fence_mm': FENCE}, **block)]}
    return floorplan.intent_from_dict(doc, source_path='t.json')


def _fence_hits(path):
    r = floorplan.grade(_intent(), parse_kicad_pcb(path), path)
    return sorted(v.ref for v in r.errors + r.warnings
                  if v.rule == 'keepout' and 'fence:' in v.message)


class TheFenceCompiles(unittest.TestCase):

    def test_four_bands_round_the_zone(self):
        ks = {k['name']: k for k in _intent().keepouts}
        self.assertEqual(sorted(ks), ['fence:can:E', 'fence:can:N',
                                      'fence:can:S', 'fence:can:W'])
        x0, y0, x1, y1 = ZONE
        self.assertEqual(ks['fence:can:N']['rect'],
                         (x0 - FENCE, y0 - FENCE, x1 + FENCE, y0))
        self.assertEqual(ks['fence:can:E']['rect'], (x1, y0, x1 + FENCE, y1))
        for k in ks.values():
            self.assertEqual(k['allow'], ())
            self.assertEqual(k['sides'], ('F',))
        # no side: both faces
        self.assertEqual({k['sides'] for k in _intent(side=None).keepouts},
                         {('F', 'B')})

    def test_refusals(self):
        with self.assertRaisesRegex(floorplan.IntentError, 'needs a `zone`'):
            _intent(zone=None)
        with self.assertRaisesRegex(floorplan.IntentError, 'more than 0'):
            _intent(fence_mm=0)
        with self.assertRaisesRegex(floorplan.IntentError, 'reserved'):
            floorplan.intent_from_dict(
                {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
                 'keepouts': [{'name': 'fence:x', 'rect': [0, 0, 1, 1]}]})


class TheGradeHoldsIt(unittest.TestCase):

    def test_a_member_and_a_stranger_in_the_band(self):
        # A (a member) sits on the zone's west edge, in the band; B (a
        # member) well inside; C (a stranger) on the band's east side; D
        # clear of it. FR, the frame round them all, is a container.
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [('A', 12, 15, 1.0), ('B', 20, 15, 1.0),
                                     ('C', 28.5, 15, 1.0), ('D', 34, 15, 1.0)])
            self.assertEqual(_fence_hits(path), ['A', 'C'])

    def test_a_container_is_graded_on_its_pads(self):
        # control for the frame's exemption above: a keep-out over one of
        # FR's pins (pin 1 at 3.0, 1.5) still binds it
        with tempfile.TemporaryDirectory() as td:
            path = _frame_board(td, [('A', 20, 15, 1.0)])
            intent = floorplan.intent_from_dict(
                {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
                 'keepouts': [{'name': 'post', 'rect': [2, 0.5, 4, 2.5]}]})
            r = floorplan.grade(intent, parse_kicad_pcb(path), path)
        self.assertEqual(sorted(v.ref for v in r.errors + r.warnings
                                if v.rule == 'keepout'), ['FR'])


if __name__ == '__main__':
    unittest.main()
