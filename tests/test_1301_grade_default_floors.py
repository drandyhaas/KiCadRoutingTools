"""#1301, the grade's own default: a grade handed no floors reads the board's,
never a fixed 0.55 edge clearance.

`floorplan._grade_ctx` filled a floor its caller left None with
(routing_defaults.CLEARANCE, 0.55): the edge clearance #1301 retired from
every placement reader. With the nearest-edge reach on the board's edge
clearance (#1301), that default would read which side a part reaches at
0.55 wherever a caller passes nothing. It now goes through
`floorplan.resolved_floors`: a value passed is used as passed, a value left
None is the board's own (`list_nets.board_floor_knobs`: an unset or sub-fab
edge clearance is the fab floor routing writes, 0.2), and with no board file
`routing_defaults.CLEARANCE` and that floor.

  1. `resolved_floors` on unset, declared (0.5), sub-fab (0.01) and absent
     project rules, with no board file, and with explicit values (passed
     through untouched, as before);
  2. `_grade_ctx` with no floors builds its state at the board's;
  3. `grade()` with no floors reads the nearest edge the way check_floorplan
     does on the same board (the board's floors), and the NEGATIVE CONTROL --
     `resolved_floors` put back on the old (0.25, 0.55) default -- reads
     another edge on that pose;
  4. the callers that pass no floors are unaffected: on tigard,
     `fanout_clearance._decap_violations` (45 decap findings under a tight
     limit) and `emc_roles._measure_keep_away` read the same under the old
     default and the new one;
  5. `pose_score.make_state` (the state reconcile, lock_advisor and the pose
     tools build) reads the board's floors when handed none, and reconcile
     reads a part's edge at the floors check_floorplan graded at -- the
     NEGATIVE CONTROL (the old 0.55 default) reads the corner part south.

    python3 tests/test_1301_grade_default_floors.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import routing_defaults                                       # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan                              # noqa: E402

TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')

#: J1 flush with the west edge, 0.23 mm inside the south edge, lying along
#: south, declared west: at a 0.55 reach it reads south, at 0.2 west
#: (tests/test_1301_nearest_edge_currency.py's fixture).
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
INTENT = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
          'edge_connectors': [{'ref': 'J1', 'edge': 'west',
                               'overhang_mm': {'min': 0.0}}]}


def _old_default(pcb_file, clearance=None, board_edge_clearance=None):
    """What `_grade_ctx` filled a None floor with before."""
    return (routing_defaults.CLEARANCE if clearance is None else clearance,
            0.55 if board_edge_clearance is None else board_edge_clearance)


def _quiet():
    stack = contextlib.ExitStack()
    stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
    stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
    return stack


class _Board(unittest.TestCase):
    """The fixture board, with its project; no tests of its own."""

    def setUp(self):
        self.td = tempfile.mkdtemp(prefix='krt1301floors_')
        self.path = os.path.join(self.td, 'b.kicad_pcb')
        with open(self.path, 'w', encoding='utf-8') as fh:
            fh.write(BOARD)
        self.project({})

    def tearDown(self):
        shutil.rmtree(self.td, ignore_errors=True)

    def project(self, rules):
        with open(os.path.join(self.td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({'net_settings': {'classes': [
                {'name': 'Default', 'clearance': 0.254, 'track_width': 0.2}]},
                'board': {'design_settings': {'rules': rules}}}, fh)


class DefaultFloors(_Board):

    def test_resolved_floors(self):
        rf = floorplan.resolved_floors
        self.assertEqual(rf(self.path), (0.254, 0.2))
        self.project({'min_copper_edge_clearance': 0.5})
        self.assertEqual(rf(self.path), (0.254, 0.5))
        self.project({'min_copper_edge_clearance': 0.01})
        self.assertEqual(rf(self.path), (0.254, 0.2))
        self.assertEqual(rf(self.path, 0.3, 0.01), (0.3, 0.01))
        self.assertEqual(rf(self.path, None, 0.4), (0.254, 0.4))
        self.assertEqual(rf(''), (routing_defaults.CLEARANCE, 0.2))
        self.assertEqual(rf(None, 0.1), (0.1, 0.2))

    def test_grade_ctx_reads_the_boards_floors(self):
        intent = floorplan.intent_from_dict(INTENT, '')
        with _quiet():
            st = floorplan._grade_ctx(intent, parse_kicad_pcb(self.path),   # noqa: SLF001
                                      self.path)[2]
        self.assertEqual((st.clearance, st.board_edge_clearance), (0.254, 0.2))
        with _quiet():
            st = floorplan._grade_ctx(intent, parse_kicad_pcb(self.path),   # noqa: SLF001
                                      self.path, clearance=0.3,
                                      board_edge_clearance=0.55)[2]
        self.assertEqual((st.clearance, st.board_edge_clearance), (0.3, 0.55))

    def _nearest(self):
        intent = floorplan.intent_from_dict(INTENT, '')
        with _quiet():
            res = floorplan.grade(intent, parse_kicad_pcb(self.path), self.path)
        hit = [v for v in res.violations if 'sits nearest' in v.message]
        return hit[0].measured['edge'] if hit else 'west'

    def test_a_grade_with_no_floors_reads_as_check_floorplan_does(self):
        from list_nets import board_floor_knobs
        c, e, _k = board_floor_knobs(self.path)
        intent = floorplan.intent_from_dict(INTENT, '')
        with _quiet():
            cli = floorplan.grade(intent, parse_kicad_pcb(self.path), self.path,
                                  clearance=c, board_edge_clearance=e)
        hit = [v for v in cli.violations if 'sits nearest' in v.message]
        self.assertEqual((self._nearest(), hit), ('west', []))
        # NEGATIVE CONTROL: the old default reads the same pose on another edge
        with mock.patch.object(floorplan, 'resolved_floors', _old_default):
            self.assertEqual(self._nearest(), 'south')

    def test_the_callers_that_pass_no_floors_are_unaffected(self):
        from placement import emc_roles, fanout_clearance
        intent = floorplan.intent_from_dict(
            {'schema': 1, 'kind': floorplan.KIND, 'units': 'mm',
             'decaps': {'max_pin_distance_mm': 0.3, 'max_distance_mm': 0.5}}, TIGARD)

        def both():
            with _quiet():
                pcb = parse_kicad_pcb(TIGARD)
                decaps = sorted((v.rule, v.ref, v.message) for v in
                                fanout_clearance._decap_violations(        # noqa: SLF001
                                    intent, pcb, TIGARD))
                keep = emc_roles._measure_keep_away(                       # noqa: SLF001
                    pcb, ['/USB_DP:+3V3:3.0', 'GND:/USB_DN:2.0'])
            return decaps, keep
        new = both()
        with mock.patch.object(floorplan, 'resolved_floors', _old_default):
            old = both()
        self.assertGreater(len(new[0]), 0)
        self.assertTrue(all(r.get('state') != 'not measured' for r in new[1].values()),
                        new[1])
        self.assertEqual(new, old)


class StateDefaultFloors(_Board):
    """`pose_score.make_state`, the state every pose tool, reconcile and
    lock_advisor build, defaulted its floors to (0.25, 0.55) as well:
    reconcile read a part's edge at a 0.55 reach while the grade beside it
    read 0.2. Now a None floor is the board's (`resolved_floors`), and
    reconcile's geometry takes the floors check_floorplan graded at."""

    def _edge(self, floors_used=None):
        from placement import reconcile
        with _quiet():
            pcb = parse_kicad_pcb(self.path)
            geo = reconcile._Geometry(pcb, self.path, floors_used)  # noqa: SLF001
            p = geo.state.parts['J1']
            return reconcile._edge_of(                              # noqa: SLF001
                p.rect(p.x, p.y, p.rot), pcb.board_info.board_bounds, geo, 'J1', p.rot)

    def test_make_state_reads_the_boards_floors(self):
        import pose_score
        with _quiet():
            st = pose_score.make_state(parse_kicad_pcb(self.path), self.path)
            st2 = pose_score.make_state(parse_kicad_pcb(self.path), self.path,
                                        clearance=0.3, board_edge_clearance=0.55)
        self.assertEqual((st.clearance, st.board_edge_clearance), (0.254, 0.2))
        self.assertEqual((st2.clearance, st2.board_edge_clearance), (0.3, 0.55))

    def test_reconcile_reads_the_edge_the_grade_reads(self):
        self.assertEqual(self._edge(), 'west')
        knobs = {'clearance': {'value': 0.254, 'source': 'board netclass'},
                 'board_edge_clearance': {'value': 0.3, 'source': 'cli'}}
        # at a 0.3 edge clearance J1 reaches the south side too, and lies
        # along it: the grade reads south there, and so does reconcile
        self.assertEqual(self._edge(knobs), 'south')
        # NEGATIVE CONTROL: the old (0.25, 0.55) default reads south here
        with mock.patch.object(floorplan, 'resolved_floors', _old_default):
            self.assertEqual(self._edge(), 'south')


if __name__ == '__main__':
    unittest.main(verbosity=2)
