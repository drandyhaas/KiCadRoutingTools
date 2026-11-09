"""#1631: a zone member with no pose clear of a part that will not move.

`plan_check` asked whether a zone's members fit the zone, and whether a
keep-out or a stranger's exclusive zone left them a pose -- never what a
FIXED part already covers. A one-member zone wholly inside the courtyard of a
part the same intent seats in `fixed_poses[]` (or that the file locks) read
0 errors before placing, and every seed then ended with the member illegal
against that part. `plan_zone_on_fixed` asks `zone_pose_feasibility` the
question with each fixed courtyard as a keep-out on its face: a WARN, and an
ERROR only when a declared overlap budget is short of the overlap the member
is forced into, on the budget's own geometry.
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

#: Wholly inside BIG's 12 x 12 courtyard (14..26, 9..21).
COVERED = [17, 12, 23, 18]
#: Overlapping BIG's courtyard, with 2 mm of free zone west of it.
PARTLY = [10, 12, 16, 18]


def _board(td, locked=()):
    """A frame-ringed board (`_frame_board`; the frame FR is a container)
    with A (the zone's 2 x 2 member) and BIG (12 x 12 at the centre), each
    `locked` one FILE-locked."""
    path = _frame_board(td, [('A', 5, 25, 1.0), ('BIG', 20, 15, 6.0)])
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    for ref in locked:
        old = f'(property "Reference" "{ref}"'
        assert text.count(old) == 1, ref
        text = text.replace(old, '(locked yes)\n    ' + old)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _intent(zone, fixed=(), budget=None):
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
           'units': 'mm',
           'blocks': [{'name': 'z', 'refs': ['A'], 'zone': zone,
                       'side': 'F', 'tolerance_mm': 0}]}
    if fixed:
        doc['min_reader'] = 7
        doc['fixed_poses'] = [dict(f, rot=0, side='F', basis='declared',
                                   why='t') for f in fixed]
    if budget is not None:
        doc['legality_budget'] = {'overlap_area': budget}
    return floorplan.intent_from_dict(doc, source_path='t.json')


def _found(path, intent):
    v, _m = floorplan.plan_check(intent, parse_kicad_pcb(path), path)
    return [x for x in v if x.rule == 'plan_zone_on_fixed']


class AZoneOnAFixedPart(unittest.TestCase):

    def test_a_file_locked_part_over_the_zone(self):
        with tempfile.TemporaryDirectory() as td:
            got = _found(_board(td, ('BIG',)), _intent(COVERED))
        self.assertEqual([(x.ref, x.severity) for x in got],
                         [('A', floorplan.WARN)])
        self.assertEqual(got[0].measured['fixed'], ['BIG'])
        self.assertIn('BIG (locked)', got[0].message)

    def test_a_declared_pose_over_the_zone(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            got = _found(path, _intent(COVERED,
                                       [{'ref': 'BIG', 'x': 20, 'y': 15}]))
            self.assertEqual([(x.ref, x.severity) for x in got],
                             [('A', floorplan.WARN)])
            self.assertIn('BIG (fixed_poses[])', got[0].message)
            # control: the same part declared clear of the zone
            self.assertEqual(_found(path, _intent(
                COVERED, [{'ref': 'BIG', 'x': 20, 'y': 15 + 13}])), [])

    def test_an_error_only_past_a_declared_budget(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, ('BIG',))
            got = _found(path, _intent(COVERED, budget=0))
            self.assertEqual([(x.ref, x.severity) for x in got],
                             [('A', floorplan.ERROR)])
            # the whole of A's 2 x 2 outline is on BIG's
            self.assertAlmostEqual(got[0].measured['forced_overlap_mm2'],
                                   4.0, places=3)
            # a budget that covers the forced overlap stays a WARN
            got = _found(path, _intent(COVERED, budget=5))
            self.assertEqual([x.severity for x in got], [floorplan.WARN])

    def test_controls(self):
        with tempfile.TemporaryDirectory() as td:
            # nothing fixed: the placer may move BIG
            self.assertEqual(_found(_board(td), _intent(COVERED)), [])
        with tempfile.TemporaryDirectory() as td:
            # a zone with room left beside the fixed part
            self.assertEqual(_found(_board(td, ('BIG',)), _intent(PARTLY)), [])
        with tempfile.TemporaryDirectory() as td:
            # a locked container over the zone is no stranger
            self.assertEqual(_found(_board(td, ('FR',)), _intent(COVERED)), [])


if __name__ == '__main__':
    unittest.main()
