"""#1463: a container is no stranger to an exclusive zone.

A container (a pin frame or an outline other parts sit inside,
`legality.container_kinds`) is graded on its holes and pads, never on its
frame rect -- the courtyard channel and the anneal's overlap already say so.
`rule_zone_exclusive` and the seat gate (`quench.build_zone_spec`) measured
that rect, so a locked enclosure over a reserved region failed every seed,
and a declared one refused its own pose. Both now skip containers; a
non-container stranger is still graded.

A FIXED stranger -- a file lock, or a `fixed_poses[]` pose -- already inside
an exclusive zone fails every seed too, and `plan_check` now names it before
anything is placed (`plan_fixed_in_exclusive_zone`).
"""

import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan, quench                      # noqa: E402
import pose_score                                            # noqa: E402
from test_1212_container_pins import _frame_board            # noqa: E402

#: Reserved for A, inside the frame FR (whose pins ring a 40 x 30 board).
ZONE = [12, 9, 28, 21]


def _board(td, locked=()):
    """`_frame_board` with A (the zone's member), B (a stranger inside the
    zone) and C (outside it), each `locked` one FILE-locked."""
    path = _frame_board(td, [('A', 15, 12, 1.0), ('B', 25, 18, 1.0),
                             ('C', 20, 25, 1.0)])
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    for ref in locked:
        old = f'(property "Reference" "{ref}"'
        assert text.count(old) == 1, ref
        text = text.replace(old, '(locked yes)\n    ' + old)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _intent(fixed=()):
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
           'units': 'mm',
           'blocks': [{'name': 'rf', 'refs': ['A'], 'zone': ZONE,
                       'exclusive': True, 'tolerance_mm': 0}]}
    if fixed:
        doc['min_reader'] = 7
        doc['fixed_poses'] = [dict(f, rot=0, side='F', basis='mechanical',
                                   why='t') for f in fixed]
    return floorplan.intent_from_dict(doc, source_path='t.json')


def _exclusive(viols):
    return sorted(v.ref for v in viols if v.rule == 'zone_exclusive')


class TheContainerIsNoStranger(unittest.TestCase):

    def test_the_grade_skips_the_frame_and_keeps_the_stranger(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            r = floorplan.grade(_intent(), parse_kicad_pcb(path), path)
        # FR's frame rect covers the whole zone; B's courtyard is in it.
        self.assertEqual(_exclusive(r.errors), ['B'])

    def test_the_seat_gate_skips_the_frame(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            st = pose_score.make_state(parse_kicad_pcb(path), path)
        self.assertEqual(st.container_refs, {'FR'})
        zones = [{'name': 'rf', 'refs': ['A'], 'rect': ZONE,
                  'exclusive': True, 'side': None, 'tolerance_mm': 0.0}]
        spec = quench.exclusive_spec(zones, st.parts,
                                     containers=st.container_refs)
        self.assertNotIn('FR', spec)
        self.assertIn('B', spec)
        # control: without the exemption the frame is bound like B
        self.assertIn('FR', quench.exclusive_spec(zones, st.parts))


class AFixedStrangerIsFoundBeforePlacing(unittest.TestCase):

    def _found(self, path, intent):
        v, _m = floorplan.plan_check(intent, parse_kicad_pcb(path), path)
        return sorted((x.ref, x.severity) for x in v
                      if x.rule == 'plan_fixed_in_exclusive_zone')

    def test_a_locked_stranger_inside(self):
        with tempfile.TemporaryDirectory() as td:
            # FR locked too: a locked container is still no stranger
            self.assertEqual(self._found(_board(td, ('B', 'FR')), _intent()),
                             [('B', floorplan.ERROR)])
        with tempfile.TemporaryDirectory() as td:
            # control: the same stranger unlocked is the placer's to move
            self.assertEqual(self._found(_board(td), _intent()), [])

    def test_a_declared_pose_inside(self):
        with tempfile.TemporaryDirectory() as td:
            path = _board(td)
            self.assertEqual(self._found(path, _intent(
                [{'ref': 'C', 'x': 20, 'y': 15}])), [('C', floorplan.ERROR)])
            # control: declared outside the zone, and a declared container
            self.assertEqual(self._found(path, _intent(
                [{'ref': 'C', 'x': 20, 'y': 25},
                 {'ref': 'FR', 'x': 20, 'y': 15}])), [])


if __name__ == '__main__':
    unittest.main()
