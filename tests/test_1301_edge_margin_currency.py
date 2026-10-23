"""#1301: an edge connector's `overhang_mm` band, graded on the courtyard,
changed value when a routing step rewrote the project floors.

Two halves, each pinned here on a synthetic board (40 x 30 mm, rectangular
outline, courtyard-only connectors so no drawn body is measured):

1. An UNSET `min_copper_edge_clearance` (0 or absent) resolves to the fab
   edge floor every routing step writes back -- `fab_edge_floor`, 0.20 --
   not a fixed 0.55, so `board_floor_knobs` answers the same value before
   and after `fix_project_for_output` (the routing writeback) runs. A
   declared value, an explicit one and a caller's own default are unchanged.

2. The band is a MECHANICAL measure: the courtyard reading takes no margin
   (`legacy_occupancy@margin=0`), and neither does the setback conjunct's
   "no overhang" test, so the grade reads one pose the same at any
   (clearance, edge clearance). Measured on daplink before the fix: J2/J3
   read 7.0062 placed (margin 0.55) and 6.6562 routed (margin 0.2), 3 band
   errors after routing with nothing moved. The grouped branch's seat
   (`edge_seat._Probe`) reads the band in the same currency, tested where
   that module exists; main's place_seed keeps its own reading (#1301 leaves
   it alone).
"""

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
from placement import floorplan                              # noqa: E402

EPS = 1e-6
FAB_SOURCE = 'fab edge floor (what routing writes)'
#: (clearance, board edge clearance) pairs the grade is asked at: daplink's
#: placed and routed floors, sonde's, and a wide one.
MARGINS = ((0.2, 0.55), (0.1818, 0.2), (0.254, 0.01), (0.2223, 0.2),
           (0.3, 1.0))


def _board(td, body, size=(40, 30)):
    """A rectangular board `size` mm with `body` (footprint blocks) on it."""
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1")\n'
            f'  (gr_rect (start 0 0) (end {size[0]} {size[1]}) (stroke'
            ' (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
            + body + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def _fp(ref, x, y, crt, pad=(0.0, 0.0)):
    """A courtyard-only footprint: no Fab, no SilkS, so its band takes the
    legacy (courtyard) path."""
    x0, y0, x1, y1 = crt
    return (f'  (footprint "t:CONN" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS")'
            ' (hide yes))\n'
            f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width'
            ' 0.05) (type default)) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at {pad[0]} {pad[1]}) (size 1 1)'
            ' (layers "F.Cu") (net 1 "N1"))\n'
            '  )\n')


#: J1: courtyard x in [-1.0, 3.0] -> 1.0 mm past the west edge (x = 0).
#: J2: an edge receptacle 0.52 mm inside the north edge (y = 0), no overhang.
BODY = (_fp('J1', 1.0, 15.0, (-2, -3, 2, 3))
        + _fp('J2', 20.0, 2.02, (-2, -1.5, 2, 1.5)))

INTENT = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
          'edge_connectors': [
              {'ref': 'J1', 'edge': 'west',
               'overhang_mm': {'min': 0.95, 'max': 1.05}},
              {'ref': 'J2', 'edge': 'north', 'class': 'edge_receptacle',
               'overhang_mm': {'min': 0.0}}]}


def _project(pcb_path, rules, clearance=0.2):
    with open(os.path.splitext(pcb_path)[0] + '.kicad_pro', 'w',
              encoding='utf-8') as f:
        json.dump({'net_settings': {'classes': [
            {'name': 'Default', 'clearance': clearance,
             'track_width': 0.2}]},
            'board': {'design_settings': {'rules': rules}}}, f)


class UnsetEdgeIsTheFloorRoutingWrites(unittest.TestCase):

    def test_unset_reads_the_fab_edge_floor_before_and_after_routing(self):
        from fix_kicad_drc_settings import fab_edge_floor, fix_project_for_output
        from list_nets import board_floor_knobs
        for rules in ({'min_copper_edge_clearance': 0.0}, {}):
            with self.subTest(rules=rules), \
                    tempfile.TemporaryDirectory() as td:
                path = _board(td, BODY, size=(40, 30))
                _project(path, rules)
                floor = fab_edge_floor(path)
                self.assertAlmostEqual(floor, 0.2)
                _c, edge, knobs = board_floor_knobs(path)
                self.assertAlmostEqual(edge, floor)
                self.assertEqual(knobs['board_edge_clearance']['source'],
                                 FAB_SOURCE)
                # The routing writeback (every route/plane/fanout step).
                fix_project_for_output(path, path, clearance=0.1818,
                                       verbose=False)
                with open(os.path.splitext(path)[0] + '.kicad_pro') as f:
                    wrote = json.load(f)['board']['design_settings'][
                        'rules']['min_copper_edge_clearance']
                self.assertAlmostEqual(wrote, floor)
                _c, after, knobs = board_floor_knobs(path)
                self.assertAlmostEqual(after, edge)
                self.assertEqual(knobs['board_edge_clearance']['source'],
                                 'board constraint')

    def test_declared_explicit_and_caller_default_are_unchanged(self):
        from list_nets import board_floor_knobs
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, BODY, size=(40, 30))
            _project(path, {'min_copper_edge_clearance': 0.5})
            self.assertEqual(board_floor_knobs(path)[1], 0.5)
            self.assertEqual(board_floor_knobs(path, None, 0.3)[1], 0.3)
            _project(path, {'min_copper_edge_clearance': 0.0})
            _c, edge, knobs = board_floor_knobs(path, edge_default=0.55)
            self.assertEqual((edge, knobs['board_edge_clearance']['source']),
                             (0.55, 'fixed default'))

    def test_a_value_under_the_floor_is_raised_as_routing_raises_it(self):
        from fix_kicad_drc_settings import effective_board_edge_clearance
        from list_nets import board_floor_knobs
        with tempfile.TemporaryDirectory() as td:
            path = _board(td, BODY, size=(40, 30))
            _project(path, {'min_copper_edge_clearance': 0.01})   # sonde's
            _c, edge, knobs = board_floor_knobs(path)
            self.assertAlmostEqual(edge, effective_board_edge_clearance(path, 0.0))
            self.assertAlmostEqual(edge, 0.2)
            k = knobs['board_edge_clearance']
            self.assertEqual((k['source'], k['declared'], k['raised_to']),
                             ('board constraint', 0.01, FAB_SOURCE))
            _c, edge, knobs = board_floor_knobs(path, None, 0.1)
            self.assertAlmostEqual(edge, effective_board_edge_clearance(path, 0.1))
            self.assertEqual((knobs['board_edge_clearance']['source'],
                              knobs['board_edge_clearance']['declared']), ('cli', 0.1))
            _project(path, {'min_copper_edge_clearance': 0.3})
            _c, edge, knobs = board_floor_knobs(path)
            self.assertEqual(edge, 0.3)
            self.assertNotIn('declared', knobs['board_edge_clearance'])

    def test_no_board_file_reads_the_same_floor(self):
        from list_nets import unset_board_edge_clearance
        self.assertEqual(unset_board_edge_clearance(None), (0.2, FAB_SOURCE))


class TheBandTakesNoMargin(unittest.TestCase):

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='krt1301_')
        self.path = _board(self.td, BODY, size=(40, 30))
        self.intent = floorplan.intent_from_dict(INTENT, '')

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def _grade(self, clr, edge):
        return floorplan.grade(self.intent, parse_kicad_pcb(self.path),
                               self.path, clearance=clr,
                               board_edge_clearance=edge)

    @staticmethod
    def _ev(res, ref):
        return next(e for e in res.edge_connector_evidence
                    if e['ref'] == ref)

    @staticmethod
    def _viol(res, ref):
        return sorted(v.message for v in res.violations
                      if v.ref == ref and v.rule == 'edge_connector')

    def test_one_pose_reads_one_overhang_at_every_margin(self):
        seen = set()
        for clr, edge in MARGINS:
            r = self._grade(clr, edge)
            ev = self._ev(r, 'J1')
            self.assertEqual(ev['overhang_basis'], 'legacy_occupancy@margin=0')
            # the courtyard's 1.0 mm past the west edge, not 1.0 + margin
            self.assertAlmostEqual(ev['overhang_mm'], 1.0, places=4)
            self.assertEqual(ev['overhang_disposition'], 'pass')
            self.assertEqual(self._viol(r, 'J1'), [], (clr, edge))
            seen.add((ev['overhang_mm'], tuple(self._viol(r, 'J1'))))
        self.assertEqual(len(seen), 1, seen)

    def test_the_setback_no_overhang_test_takes_no_margin(self):
        # J2's courtyard is 0.52 mm inside: no overhang at ANY margin, so its
        # 0.5 mm receptacle seat tolerance applies at every one. At a 0.55
        # margin it used to read 0.03 of "overhang" and skip the seat.
        for clr, edge in MARGINS:
            r = self._grade(clr, edge)
            msgs = self._viol(r, 'J2')
            self.assertEqual(len(msgs), 1, (clr, edge, msgs))
            self.assertIn('seated 0.52mm from the nearest edge with no '
                          'overhang', msgs[0])
            self.assertAlmostEqual(self._ev(r, 'J2')['overhang_mm'], 0.0)

    def test_oob_exempt_reads_the_band_in_the_same_currency(self):
        for clr, edge in MARGINS:
            ctx = floorplan._grade_ctx(                     # noqa: SLF001
                self.intent, parse_kicad_pcb(self.path), self.path,
                clearance=clr, board_edge_clearance=edge)[0]
            self.assertAlmostEqual(ctx.oob_exempt().get('J1'), 1.0, places=4)


def _module(name):
    try:
        return __import__('placement.' + name, fromlist=[name])
    except ImportError:
        return None


EDGE_SEAT = _module('edge_seat')


class ThePlacersSeatInTheSameCurrency(unittest.TestCase):

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='krt1301_')
        self.path = _board(self.td, BODY, size=(40, 30))

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def _state(self, clr, edge):
        import pose_score
        pcb = parse_kicad_pcb(self.path)
        return pcb, pose_score.make_state(pcb, self.path, clearance=clr,
                                          board_edge_clearance=edge)

    @unittest.skipUnless(EDGE_SEAT, "the grouped edge_seat is not in this tree")
    def test_edge_seat_probe(self):
        from placement.body import board_bodies
        for clr, edge in MARGINS:
            pcb, st = self._state(clr, edge)
            probe = EDGE_SEAT._Probe(                       # noqa: SLF001
                st, pcb, self.path, board_bodies(pcb, self.path),
                floorplan.outline_state(pcb, self.path))
            p = st.parts['J1']
            amount, basis = probe.band_read('J1', 'west', (p.x, p.y, p.rot))
            self.assertEqual(basis, 'legacy_occupancy@margin=0')
            self.assertAlmostEqual(amount, 1.0, places=4)


if __name__ == '__main__':
    unittest.main(verbosity=2)
