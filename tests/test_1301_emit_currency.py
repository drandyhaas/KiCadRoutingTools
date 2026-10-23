"""#1301, the emitter: `emit_intent` reads the board in the currencies its
grade reads, not at a fixed 0.55 edge clearance.

It built its state at (routing_defaults.CLEARANCE, 0.55) and read three
things off that one occupancy reading: which parts sit AT the edge (within
0.55 of it), each band's `max` (that reading + 0.5) and each part's edge. The
grade reads the band at zero margin (#1301), so every emitted band sat 0.55
above it, and the edge at the board's own edge clearance. Now:

  * the floors are the board's (`floorplan.resolved_floors`, as
    check_floorplan grades at them), or the ones passed in;
  * a part is AT the edge within `part_class.SEAT_TOL_MM` (the seat
    tolerance the grade gives a receptacle) or past it -- a mechanical
    distance, so no DRC floor decides it;
  * its band is read at zero margin (`courtyard_overhang`), max = that + 0.5;
  * its edge at `edge_reach_mm`, the grade's reach.

The fixture (40 x 30 mm, edge clearance unset, so 0.2): three courtyard-only
headers -- J1 1.0 mm past the west edge, J2 0.3 mm inside the north edge
(inside the seat tolerance, beyond the edge clearance), J3 0.7 mm inside the
south edge (beyond both).

  1. J1's band is its zero-margin overhang + 0.5 (1.5), at any floor passed;
     the old 0.55 reading would have written 2.05 (the control);
  2. J2 is still observed on its edge, band max 0.5; J3 is not observed;
  3. the emitted intent grades with no error on its own board at the
     board's floors, on a routed copy's (the writeback rewrites them), and at
     a declared 0.3 edge clearance;
  4. its `oob_count` budget is the count the grade reads at those floors.

    python3 tests/test_1301_emit_currency.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan, legality                    # noqa: E402

NAME = 'Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical'


def _fp(ref, x, y, crt):
    x0, y0, x1, y1 = crt
    return (f'  (footprint "{NAME}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1"))\n'
            '  )\n')


BOARD = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
         '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
         '  (net 0 "") (net 1 "N1")\n'
         '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))'
         ' (layer "Edge.Cuts"))\n'
         + _fp('J1', 1.0, 15.0, (-2, -3, 2, 3))          # 1.0 past the west edge
         + _fp('J2', 20.0, 1.8, (-2, -1.5, 2, 1.5))      # 0.3 inside the north edge
         + _fp('J3', 20.0, 27.8, (-2, -1.5, 2, 1.5))     # 0.7 inside the south edge
         + ')\n')


class EmitCurrency(unittest.TestCase):

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='krt1301emit_')
        self.path = os.path.join(self.td, 'b.kicad_pcb')
        with open(self.path, 'w', encoding='utf-8') as fh:
            fh.write(BOARD)
        self.project({})

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def project(self, rules, clearance=0.25):
        with open(os.path.join(self.td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({'net_settings': {'classes': [
                {'name': 'Default', 'clearance': clearance, 'track_width': 0.2}]},
                'board': {'design_settings': {'rules': rules}}}, fh)

    def emit(self, **kw):
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            return floorplan.emit_intent(parse_kicad_pcb(self.path), self.path, **kw)

    def grade(self, doc, clearance, edge):
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            return floorplan.grade(floorplan.intent_from_dict(doc, 'emitted.json'),
                                   parse_kicad_pcb(self.path), self.path,
                                   clearance=clearance, board_edge_clearance=edge)

    @staticmethod
    def rows(doc):
        return {c['ref']: c for c in doc.get('edge_connectors', [])}

    def test_the_band_is_the_zero_margin_overhang(self):
        for edge in (None, 0.2, 0.55):
            with self.subTest(board_edge_clearance=edge):
                kw = {} if edge is None else {'board_edge_clearance': edge}
                j1 = self.rows(self.emit(**kw)).get('J1') or {}
                self.assertEqual(j1.get('edge'), 'west')
                self.assertAlmostEqual(j1['overhang_mm']['max'], 1.5, places=3)
        # the control: the occupancy reading at the old fixed 0.55 margin
        pcb = parse_kicad_pcb(self.path)
        gate = legality.BoardOutlineGate(pcb.board_info, 0.55)
        old = gate.rect_outside_amount((-1.0, 12.0, 3.0, 18.0)) + 0.5
        self.assertAlmostEqual(old, 2.05, places=3)

    def test_at_the_edge_is_the_seat_tolerance(self):
        rows = self.rows(self.emit())
        self.assertEqual(rows.get('J2', {}).get('edge'), 'north', rows)
        self.assertAlmostEqual(rows['J2']['overhang_mm']['max'], 0.5, places=3)
        self.assertNotIn('J3', rows)

    def test_the_emitted_intent_grades_clean_on_its_board(self):
        from fix_kicad_drc_settings import fix_project_for_output
        from list_nets import board_floor_knobs
        doc = self.emit()
        c, e, _k = board_floor_knobs(self.path)
        self.assertEqual((round(c, 4), round(e, 4)), (0.25, 0.2))
        res = self.grade(doc, c, e)
        self.assertEqual([v.message for v in res.errors], [])
        # the routing writeback lowers the class; the intent still grades clean
        fix_project_for_output(self.path, self.path, clearance=0.1, verbose=False)
        c2, e2, _k = board_floor_knobs(self.path)
        self.assertEqual([v.message for v in self.grade(doc, c2, e2).errors], [])
        # and emitted and graded at a declared 0.3 edge clearance
        self.project({'min_copper_edge_clearance': 0.3})
        c3, e3, _k = board_floor_knobs(self.path)
        self.assertEqual(e3, 0.3)
        self.assertEqual([v.message for v in self.grade(self.emit(), c3, e3).errors], [])

    def test_the_oob_budget_is_the_grades_count(self):
        doc = self.emit()
        budget = (doc.get('legality_budget') or {}).get('oob_count')
        with contextlib.redirect_stdout(io.StringIO()):
            ctx = floorplan._grade_ctx(                          # noqa: SLF001
                floorplan.intent_from_dict(doc, 'emitted.json'),
                parse_kicad_pcb(self.path), self.path, clearance=0.25,
                board_edge_clearance=0.2)[0]
        self.assertEqual(budget, int(ctx.legality['oob_count']))


if __name__ == '__main__':
    unittest.main(verbosity=2)
