"""#1607: a locked connector's band reading is its own geometry against the
outline, whether or not the other parts are placed.

crkbd's J1 (a USB-C locked at an unchanged pose) read `overhang 0.0000mm`
on the placed board and 4.6315 mm after `make_unplaced.py --keep-locked`
piled the rest, so the intent `--emit-intent` wrote from the placed board
failed on the pile every placement run starts from. Two causes, both fixed:

  1. The board's interior depended on other parts' poses.
     `drop_pad_containing_cutouts` (#291/#505) reclassifies a cutout that
     encloses >= 2 pad centres as a milled inner outline, whose interior is
     board. It counted NPTH holes: crkbd's milled gap (the slot between its
     halves and the panel rails, no copper pad inside) enclosed two
     mouse-bite holes of its breakaway tabs while they were placed, so it was
     board there and a hole on the pile. Only pads that carry copper count
     now: an NPTH hole is itself a hole, and #291 protects routing
     endpoints.
  2. The emitter read its band on the SUMMED ranking reading
     (`courtyard_overhang`) while the grade reads the distance past the
     declared edge (`courtyard_overhang_past`, #1403). A courtyard hanging
     into a notch of the outline read 1.80 summed and 4.63 past its edge,
     so the emitted band failed its own board. The emitter now widens to
     the number the grade reads.

The fixture is crkbd's shape in miniature: a panel frame, a milled gap with a
tongue reaching down into the board, a breakaway tab (two NPTH holes) whose
holes sit in the gap, and a locked USB-C whose courtyard hangs into the
tongue while its corners stay on the board. The pile is the same file with
the tab moved off the board, as make_unplaced does.

    python3 tests/test_1607_locked_overhang_pile_invariant.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                               # noqa: E402
from run_utils import evidence                                # noqa: E402

USB = 'Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal'

# The milled gap: a slot 5..7 across the panel with a tongue 28..32 reaching
# down to y = 10, the way crkbd's gap reaches down to its USB-C.
GAP = [(10, 5), (50, 5), (50, 7), (32, 7), (32, 10), (28, 10), (28, 7),
       (10, 7)]


def _poly(points):
    pts = ' '.join(f'(xy {x} {y})' for x, y in points)
    return (f'  (gr_poly (pts {pts}) (stroke (width 0.05) (type default))'
            ' (fill none) (layer "Edge.Cuts"))\n')


def board(tab_at):
    tx, ty = tab_at
    return ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user)'
            ' (49 "F.Fab" user) (47 "F.CrtYd" user) (37 "F.SilkS" user))\n'
            '  (net 0 "") (net 1 "VBUS") (net 2 "GND")\n'
            '  (gr_rect (start 0 0) (end 60 40) (stroke (width 0.05) (type default))'
            ' (fill none) (layer "Edge.Cuts"))\n'
            + _poly(GAP)
            # the breakaway tab: two NPTH mouse-bite holes, in the gap
            + f'  (footprint "kbd:Breakaway_Tabs" (layer "F.Cu") (at {tx} {ty})\n'
            '    (property "Reference" "BT1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            '    (pad "" np_thru_hole circle (at -0.4 0) (size 0.5 0.5) (drill 0.5)'
            ' (layers "*.Cu" "*.Mask"))\n'
            '    (pad "" np_thru_hole circle (at 0.4 0) (size 0.5 0.5) (drill 0.5)'
            ' (layers "*.Cu" "*.Mask"))\n'
            '  )\n'
            # the locked USB-C: courtyard 27..33 x 8..16, 2 mm into the
            # tongue, its corners on the board; copper pads on the board
            + f'  (footprint "{USB}" (locked yes) (layer "F.Cu") (at 30 13)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            '    (fp_rect (start -3 -5) (end 3 3) (stroke (width 0.05) (type default))'
            ' (fill none) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -1 2) (size 0.6 1) (layers "F.Cu") (net 1 "VBUS"))\n'
            '    (pad "2" smd rect (at 1 2) (size 0.6 1) (layers "F.Cu") (net 2 "GND"))\n'
            '  )\n'
            ')\n')


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **kw)


class LockedOverhangPileInvariant(unittest.TestCase):

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='krt1607_')
        self.placed = self._write('placed', (20, 6))     # holes in the gap
        self.pile = self._write('pile', (100, 100))      # piled off the board

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def _write(self, name, tab_at):
        path = os.path.join(self.td, name + '.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(board(tab_at))
        return evidence(path, name + ' board')

    def _gap_role(self, path):
        bi = quiet(parse_kicad_pcb, path).board_info
        def has(rings):
            return any(len(r) == len(GAP) and min(p[1] for p in r) == 5.0
                       and max(p[1] for p in r) == 10.0 for r in rings or ())
        return has(bi.board_cutouts), has(getattr(bi, 'board_edge_contours', None))

    def test_npth_holes_do_not_make_the_gap_board(self):
        # The control: the tab's holes really are inside the gap on the
        # placed board (two pad centres, the count #291's rule reads).
        from kicad_parser import _pt_in_ring
        pcb = quiet(parse_kicad_pcb, self.placed)
        inside = [p for p in pcb.footprints['BT1'].pads
                  if _pt_in_ring(p.global_x, p.global_y, GAP)]
        self.assertEqual(len(inside), 2)
        for path in (self.placed, self.pile):
            with self.subTest(board=os.path.basename(path)):
                self.assertEqual(self._gap_role(path), (True, False))

    def _j1(self, doc, path):
        res = quiet(floorplan.grade, floorplan.intent_from_dict(doc, 'emitted.json'),
                    quiet(parse_kicad_pcb, path), path)
        ev = [e for e in res.edge_connector_evidence if e['ref'] == 'J1']
        self.assertEqual(len(ev), 1, res.edge_connector_evidence)
        errs = [v.message for v in res.violations
                if v.rule == 'edge_connector' and v.ref == 'J1'
                and v.severity == 'error']
        return ev[0], errs

    def test_the_reading_and_the_emitted_band_agree_placed_and_piled(self):
        rows = {}
        for path in (self.placed, self.pile):
            doc = quiet(floorplan.emit_intent, quiet(parse_kicad_pcb, path), path)
            rows[path] = [c for c in doc.get('edge_connectors', [])
                          if c['ref'] == 'J1']
        placed_rows = rows[self.placed]
        self.assertEqual(len(placed_rows), 1, placed_rows)
        j1 = placed_rows[0]
        self.assertEqual(j1.get("edge"), "north")
        # the pile (J1 locked) emits the same claim the placed board does
        self.assertEqual(rows[self.pile], placed_rows)
        doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
               'units': 'mm', 'edge_connectors': placed_rows}
        readings = {}
        for path in (self.placed, self.pile):
            with self.subTest(board=os.path.basename(path)):
                ev, errs = self._j1(doc, path)
                readings[path] = ev['overhang_mm']
                self.assertEqual(errs, [])
        # one reading, the courtyard's 2 mm into the tongue, on both boards
        self.assertAlmostEqual(readings[self.placed], 2.0, delta=0.1)  # sampled
        self.assertAlmostEqual(readings[self.pile], readings[self.placed], places=6)
        # and the band is read on that number, not the summed ranking reading
        self.assertGreaterEqual(j1['overhang_mm']['max'],
                                readings[self.placed] + 0.5 - 1e-3)  # max is rounded to 3 places


if __name__ == '__main__':
    unittest.main(verbosity=2)
