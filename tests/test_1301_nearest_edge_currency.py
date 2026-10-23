"""#1301, the nearest-edge conjunct: the sides a part REACHES are read at the
board's edge clearance (`floorplan.edge_reach_mm`), not at the gate's margin
`max(clearance, edge clearance)`.

#1301 made the overhang band read at zero margin and an unset or sub-fab
edge clearance read the floor routing writes, so the band reads one value on
a placed board and on its routed copy. `_nearest_edge` was left on the gate's
margin, and the clearance half of that max is a netclass value a routing
step's DRC writeback lowers (`--clearance-ceiling`): sonde_xilinx's class
0.254 is written back as 0.2. A part within 0.254 but not 0.2 of a second
side reached it placed and not routed, so the edge it read, and the
edge_connector verdict, could change with nothing moved.

The fixture (a 40 x 30 mm board, edge clearance 0.2): J1, courtyard-only so
it has no mating face, 12 mm long lying along the south edge, flush with the
west edge and 0.23 mm inside the south edge, declared west.

  1. the grade reads one edge, west, and no nearest-edge violation, at every
     clearance from 0.1 to 0.3 (the placed and the routed floors among them);
  2. so does a real round trip: graded at the project's own floors, then
     again after the routing writeback (`fix_project_for_output`, clearance
     0.2) has rewritten them;
  3. the edge seat's probe (`edge_seat._Probe.why`) and reconcile's
     `_edge_of` read that edge too, at both clearances;
  4. a declared edge clearance moves the reach with it (0.3 reaches the
     south side here; the read is then decided by the along-edge rung);
  5. NEGATIVE CONTROL: with `edge_reach_mm` put back on the gate's margin,
     the same pose reads south at class clearance 0.254 and west at 0.2 --
     the defect this pins, so the fixture discriminates.

    python3 tests/test_1301_nearest_edge_currency.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import edge_seat, floorplan, reconcile        # noqa: E402

EDGE = 0.2
#: the class clearances graded at: sonde's placed 0.254 and routed 0.2,
#: daplink's routed 0.1818, a ceiling of 0.1, and one above 0.254
CLEARANCES = (0.254, 0.2, 0.1818, 0.1, 0.3)

BOARD = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
         '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
         '  (net 0 "") (net 1 "N1")\n'
         '  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))'
         ' (layer "Edge.Cuts"))\n'
         '  (footprint "t:CONN" (layer "F.Cu") (at 6 28.77)\n'
         '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
         '    (fp_rect (start -6 -1) (end 6 1) (stroke (width 0.05) (type default))'
         ' (layer "F.CrtYd"))\n'
         '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1"))\n'
         '  )\n)\n')

ENTRY = {'ref': 'J1', 'edge': 'west', 'overhang_mm': {'min': 0.0}}
INTENT = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
          'edge_connectors': [ENTRY]}


class Fixture(unittest.TestCase):

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='krt1301ne_')
        self.path = os.path.join(self.td, 'b.kicad_pcb')
        with open(self.path, 'w', encoding='utf-8') as fh:
            fh.write(BOARD)
        self.project(0.254, EDGE)
        self.intent = floorplan.intent_from_dict(INTENT, '')

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def project(self, clearance, edge):
        with open(os.path.join(self.td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({'net_settings': {'classes': [
                {'name': 'Default', 'clearance': clearance, 'track_width': 0.2}]},
                'board': {'design_settings': {'rules': {
                    'min_copper_edge_clearance': edge}}}}, fh)

    def grade(self, clearance, edge=EDGE):
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            return floorplan.grade(self.intent, parse_kicad_pcb(self.path), self.path,
                                   clearance=clearance, board_edge_clearance=edge)

    @staticmethod
    def nearest(res):
        """The edge the nearest-edge conjunct read, when it failed; else the
        declared one."""
        hit = [v for v in res.violations
               if v.rule == 'edge_connector' and 'sits nearest' in v.message]
        return hit[0].measured['edge'] if hit else ENTRY['edge']

    def state(self, clearance, edge=EDGE):
        import pose_score
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(self.path)
            return pcb, pose_score.make_state(pcb, self.path, clearance=clearance,
                                              board_edge_clearance=edge)


class TheGradeReadsOneEdge(Fixture):

    def test_one_edge_at_every_clearance(self):
        for clr in CLEARANCES:
            with self.subTest(clearance=clr):
                res = self.grade(clr)
                self.assertEqual(self.nearest(res), 'west')
                self.assertEqual([v.message for v in res.violations
                                  if v.rule == 'edge_connector'], [])

    def test_placed_and_routed_project_read_the_same(self):
        from fix_kicad_drc_settings import fix_project_for_output
        from list_nets import board_floor_knobs
        clr, edge, _k = board_floor_knobs(self.path)
        self.assertEqual((round(clr, 4), round(edge, 4)), (0.254, EDGE))
        placed = self.grade(clr, edge)
        # the routing writeback: the class lowered to the routed 0.2
        fix_project_for_output(self.path, self.path, clearance=0.2, verbose=False)
        clr2, edge2, _k = board_floor_knobs(self.path)
        self.assertEqual((round(clr2, 4), round(edge2, 4)), (0.2, EDGE))
        routed = self.grade(clr2, edge2)
        self.assertEqual(self.nearest(placed), self.nearest(routed))
        self.assertEqual(sorted(v.message for v in placed.violations),
                         sorted(v.message for v in routed.violations))

    def test_a_declared_edge_clearance_moves_the_reach(self):
        # at 0.3 the south side (0.23 away) is reached too, and the part runs
        # along it farther than along the west: the along-edge rung reads it
        self.assertEqual(self.nearest(self.grade(0.1, 0.3)), 'south')
        self.assertEqual(self.nearest(self.grade(0.254, 0.3)), 'south')


class ThePlacersReadTheSameEdge(Fixture):

    def test_edge_seat_probe(self):
        from placement.body import board_bodies
        for clr in (0.254, 0.2):
            with self.subTest(clearance=clr):
                pcb, st = self.state(clr)
                probe = edge_seat._Probe(                     # noqa: SLF001
                    st, pcb, self.path, board_bodies(pcb, self.path),
                    floorplan.outline_state(pcb, self.path))
                p = st.parts['J1']
                kind, read = probe.why(ENTRY, 'west', (p.x, p.y, p.rot), {})
                self.assertNotEqual(kind, 'edge', read)

    def test_reconcile_edge_of(self):
        for clr in (0.254, 0.2):
            with self.subTest(clearance=clr):
                pcb, st = self.state(clr)
                geo = SimpleNamespace(state=st, pcb=pcb, bodies={})
                p = st.parts['J1']
                self.assertEqual(reconcile._edge_of(               # noqa: SLF001
                    p.rect(p.x, p.y, p.rot), pcb.board_info.board_bounds,
                    geo, 'J1', p.rot), 'west')


class NegativeControl(Fixture):
    """The old currency, put back: the reach at the gate's margin."""

    def test_the_gate_margin_reads_two_edges(self):
        old = mock.patch.object(floorplan, 'edge_reach_mm',
                                lambda st: float(st.edge_gate.margin))
        with old:
            placed = self.nearest(self.grade(0.254))
            routed = self.nearest(self.grade(0.2))
            pcb, st = self.state(0.254)
            p = st.parts['J1']
            seat = reconcile._edge_of(                             # noqa: SLF001
                p.rect(p.x, p.y, p.rot), pcb.board_info.board_bounds,
                SimpleNamespace(state=st, pcb=pcb, bodies={}), 'J1', p.rot)
        self.assertEqual((placed, routed), ('south', 'west'))
        self.assertEqual(seat, 'south')


if __name__ == '__main__':
    unittest.main(verbosity=2)
