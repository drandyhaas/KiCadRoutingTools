"""A mechanical datum at its pose leaves the outline budget; nothing else does.

A module's mating connector is a `fixed_poses[]` entry of basis `mechanical`.
Its courtyard entering the edge-clearance band is a fact of the mating part,
so counted, it made `legality_budget {oob_count: 0, oob_amount: 0}`
unsatisfiable by any seed (remoteForwarder cm5-io: J501/J502 at 0.11 mm).
Exempt parts leave `oob_amount` too; the raw total travels beside it.
"""

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for folder in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, str(ROOT / folder))

from kicad_parser import parse_kicad_pcb           # noqa: E402
from placement import floorplan                    # noqa: E402

COURTYARD = '(fp_rect (start -1 -1) (end 1 1) (layer "F.CrtYd"))'
PAD_INBOARD = '(pad "1" smd rect (at .5 0) (size .5 .5) (layers "F.Cu"))'
PAD_OFF = '(pad "1" smd rect (at -1.5 0) (size .5 .5) (layers "F.Cu"))'
#: #962: a drawn F.Cu tab 0.4 mm wide whose outer 0.5 mm is past the west edge
#: with J1 at x = 1.1 -- copper off the board that no pad extent sees.
TAB_OFF = ('(fp_rect (start -1.6 -0.3) (end -1.2 0.3) (stroke (width 0.1) '
           '(type solid)) (fill solid) (layer "F.Cu"))')
BUDGET = {'oob_count': 0, 'oob_amount': 0}


def _legality(result):
    return [v for v in result.violations if v.rule == 'legality']


class MechanicalDatum(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='tmech_')
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def board(self, pad=PAD_INBOARD, x=1.1, extra=''):
        """J1's courtyard 0.1 mm from the west edge, inside the 0.25 band."""
        path = self.root / f'j1_{x:g}_{len(pad)}_{len(extra)}.kicad_pcb'
        path.write_text(
            '(kicad_pcb (version 20241229) (generator "tmech")\n'
            '  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts"))\n'
            f'  (footprint "t" (layer "F.Cu") (at {x} 10 0)\n'
            '    (property "Reference" "J1")\n'
            f'    {COURTYARD}\n    {pad}\n    {extra}))\n', encoding='utf-8')
        return path

    def grade(self, path, **pose):
        entry = dict({'ref': 'J1', 'x': 1.1, 'y': 10, 'rot': 0, 'side': 'F',
                      'basis': 'mechanical', 'why': 'module datum'}, **pose)
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'fixed_poses': [entry],
            'legality_budget': dict(BUDGET)})
        return floorplan.grade(intent, parse_kicad_pcb(str(path)), str(path),
                               clearance=.25, board_edge_clearance=.25)

    def test_datum_at_its_pose_is_exempt_from_count_and_amount(self):
        r = self.grade(self.board())
        self.assertEqual(r.legality.get('oob_count'), 1)
        self.assertEqual(r.legality.get('oob_count_exempt'), 1)
        self.assertFalse(_legality(r))

    def test_declared_basis_stays_counted(self):
        r = self.grade(self.board(), basis='declared')
        self.assertEqual(r.legality.get('oob_count_exempt'), 0)
        self.assertEqual(len(_legality(r)), 2)

    def test_datum_off_its_pose_stays_counted(self):
        r = self.grade(self.board(x=1.0))
        self.assertEqual(r.legality.get('oob_count_exempt'), 0)
        self.assertEqual(len(_legality(r)), 2)

    def test_pad_copper_off_the_outline_is_never_licensed(self):
        r = self.grade(self.board(pad=PAD_OFF))
        self.assertEqual(r.legality.get('oob_count_exempt'), 0)
        self.assertTrue(_legality(r))

    def test_datum_on_the_other_face_stays_counted(self):
        """At its declared x, y and rotation but on F.Cu where the entry
        declares B.Cu: not the part the datum describes (stage 0 refuses to
        seat it there), so not exempt."""
        r = self.grade(self.board(), side='B')
        self.assertEqual(r.legality.get('oob_count_exempt'), 0)
        self.assertEqual(len(_legality(r)), 2)

    def test_graphic_copper_off_the_outline_is_never_licensed(self):
        """#962: footprint GRAPHIC copper past the edge is copper off the
        board as much as a pad is, and a lock is not a waiver for it. Every
        pad is on the board here, so only the graphic census can see it."""
        r = self.grade(self.board(extra=TAB_OFF))
        self.assertEqual(r.legality.get('oob_count_exempt'), 0)
        self.assertEqual(len(_legality(r)), 2)

    def test_an_in_band_edge_connector_leaves_the_amount_too(self):
        """#961's exemption now reaches `oob_amount` as well as
        `oob_count`: J1 declared on the west edge with a 0..1 mm band is
        counted 0.15 mm by the census and exempt, so `{oob_amount: 0}` holds.
        Left in, an in-band connector made that budget unsatisfiable."""
        path = self.board()
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': [
                {'ref': 'J1', 'edge': 'west',
                 'overhang_mm': {'min': 0, 'max': 1}}],
            'legality_budget': dict(BUDGET)})
        r = floorplan.grade(intent, parse_kicad_pcb(str(path)), str(path),
                            clearance=.25, board_edge_clearance=.25)
        self.assertEqual(r.legality.get('oob_count_exempt'), 1)
        self.assertGreater(r.legality.get('oob_amount'), 0.1)
        self.assertFalse(_legality(r))

    def test_amount_publishes_the_raw_total(self):
        r = self.grade(self.board(), basis='declared')
        amount = [v for v in _legality(r) if 'oob_amount' in v.measured]
        self.assertEqual(len(amount), 1)
        self.assertNotIn('oob_amount_raw', amount[0].measured)


if __name__ == '__main__':
    unittest.main()
