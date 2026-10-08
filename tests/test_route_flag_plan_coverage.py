#!/usr/bin/env python3
"""Every routing-CLI flag must replay through a GUI plan, or say why it does not.

`route.py --bus` has a GUI home -- the Advanced-options checkbox `bus_enabled`
-- and a recorded manifest replayed through a GUI plan still routed with bus
mode OFF: the converter's unknown-flag fallthrough spelled the param `bus`,
which matches no control and no alias, so the plan executor logged "no control
for bus, ignored". The converter-parity gate never saw it, because it checks
only the flags a recorded manifest uses AND a hand-kept table names, and
`--bus` was on no table.

`tests/gui_parity/test_manifest_plan_parity.py` now carries
`check_flag_coverage`, which enumerates EVERY flag the argparse of route.py,
route_diff.py, route_planes.py, bga_fanout.py and qfn_fanout.py accepts and
requires each to
be reached, listed CLI-only with a reason, or listed as a known gap. THIS IS
ITS WX-FREE, RUN_ALL HALF: `run_all.py`'s flat
glob never collects `tests/gui_parity/`, so a gate living only there is one
this suite cannot fail on. It runs the gate (in-process and on the fixture)
and pins the fixes the enumeration forced. The NEGATIVE CONTROLS -- proof
that the gate fails for the right reason -- are in
tests/test_route_flag_plan_coverage_controls.py: each re-runs the whole gate
on a mutated temp copy of the tree, about 25 runs, so they live in the
integration lane while this file stays in the fast one.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True

import io
import os
import sys
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in (_TESTS, os.path.join(_TESTS, 'stress'),
           os.path.join(_TESTS, 'gui_parity')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                              # noqa: E402

GATE = os.path.join(_TESTS, 'gui_parity', 'test_manifest_plan_parity.py')
FIXTURE = os.path.join(_TESTS, 'gui_parity', 'fixtures',
                       'sample_redo_commands.sh')


def _src(path):
    with io.open(path, encoding='utf-8', newline='') as f:
        return f.read()


class TheEnumeration(unittest.TestCase):
    """In-process: every flag of every FLAG_COVERAGE tool is accounted for,
    and exactly once."""

    TOOLS = ('route.py', 'route_diff.py', 'route_planes.py', 'bga_fanout.py',
             'qfn_fanout.py')

    @classmethod
    def setUpClass(cls):
        import test_manifest_plan_parity as gate
        cls.gate = gate
        cls.results = {t: gate.check_flag_coverage(t) for t in cls.TOOLS}

    def test_the_routing_tools_are_enumerated(self):
        self.assertEqual(set(self.gate.FLAG_COVERAGE), set(self.TOOLS))

    def test_every_flag_is_accounted_for(self):
        for tool, (bad, _rows) in self.results.items():
            with self.subTest(tool=tool):
                self.assertEqual(bad, [], '\n'.join(
                    '%s %s: %s' % ((tool,) + b) for b in bad))

    def test_it_enumerates_the_parser_not_a_list(self):
        """The universe is krt_capabilities.script_flags, which
        tests/test_798_registrar_flags.py holds EXACT against argparse. A
        hand list only covers the flags someone thought of."""
        import krt_capabilities as caps
        for tool, (_bad, rows) in self.results.items():
            with self.subTest(tool=tool):
                real = set(caps.script_flags(caps._tool_path(caps.ROOT, tool)))
                self.assertEqual(set(rows), real)
        self.assertIn('--bus', self.results['route.py'][1])

    def test_each_flag_has_one_disposition(self):
        for tool, (_bad, rows) in self.results.items():
            spec = self.gate.FLAG_COVERAGE[tool]
            with self.subTest(tool=tool):
                self.assertEqual(
                    set(spec['cli_only']) & set(spec['known_gaps']), set())
                for flag, (disp, detail) in rows.items():
                    self.assertIn(disp, ('reached', 'cli-only', 'known-gap'),
                                  '%s: %s' % (flag, detail))

    def test_the_known_gaps_are_still_gaps(self):
        """A known-gap entry is a promise that the flag does NOT reach the
        GUI yet; the gate fails the day it does, so the list cannot rot."""
        for tool, (_bad, rows) in self.results.items():
            for flag in self.gate.FLAG_COVERAGE[tool]['known_gaps']:
                self.assertEqual(rows[flag][0], 'known-gap',
                                 '%s %s' % (tool, flag))

    def test_no_reached_control_is_parked_as_a_leak(self):
        """keepout_check and guide_corridor_check leaked between plan steps
        until their reset lines landed; the list that held them is empty,
        and a new leak must be fixed in the reset, not parked there."""
        self.assertEqual(self.gate.ROUTE_RESET_KNOWN_GAPS, {})


class TheBusFix(unittest.TestCase):
    """The defect itself, pinned on both halves of the path."""

    def test_the_converter_emits_the_control_name(self):
        import manifest_to_plan as m2p
        step = m2p.parse_command(
            ['python3', 'py_router/route.py', 'in.kicad_pcb', 'out.kicad_pcb',
             '--nets', '*', '--bus', '--bus-detection-radius', '4'])
        self.assertIs(step['params'].get('bus_enabled'), True)
        self.assertNotIn('bus', step['params'])
        self.assertEqual(step['params'].get('bus_detection_radius'), 4)

    def test_a_plan_converted_before_the_row_still_reaches_it(self):
        import test_manifest_plan_parity as gate
        aliases, _special = gate._ai_plan_tables()
        self.assertEqual(aliases.get('bus'), 'bus_enabled')
        self.assertEqual(gate._class_widgets()['RoutingDialog'].get(
            'bus_enabled'), 'CheckBox')

    def test_the_control_is_reset_and_persisted(self):
        """CLAUDE.md: a plan-settable control must be in
        reset_params_to_defaults or it leaks between steps -- and a
        persisted one must be saved AND restored."""
        import test_manifest_plan_parity as gate
        self.assertIn('bus_enabled', gate._reset_touched())
        persist = _src(os.path.join(_ROOT, 'kicad_routing_plugin',
                                    'settings_persistence.py'))
        self.assertIn("'bus_enabled': dialog.bus_enabled.GetValue()", persist)
        self.assertIn("if 'bus_enabled' in settings:", persist)


class TheComponentScope(unittest.TestCase):
    """route.py --component, converter half. The route selection used to
    ignore step['component'] entirely, so a replayed `--component U1` routed
    every net; it now composes the refs as route.py does, which needs the
    step to say whether it named any pattern at all."""

    def _step(self, *flags):
        import manifest_to_plan as m2p
        return m2p.parse_command(['python3', 'py_router/route.py',
                                  'in.kicad_pcb', 'out.kicad_pcb', *flags])

    def test_a_component_only_step_names_no_pattern(self):
        """route.py drops power/ground from the component's nets ONLY when no
        pattern is given; a '*' fallback would keep them."""
        step = self._step('--component', 'U1')
        self.assertEqual(step.get('component'), 'U1')
        self.assertEqual(step['nets'], [])

    def test_patterns_given_are_kept_for_the_intersection(self):
        step = self._step('--component', 'U1', '--nets', '*', '!GND')
        self.assertEqual(step['nets'], ['*', '!GND'])

    def test_several_refs_survive(self):
        step = self._step('--component', 'U3', 'U4', 'J1*')
        self.assertEqual(step.get('components'), ['U3', 'U4', 'J1*'])
        self.assertEqual(step['nets'], [])

    def test_a_step_with_no_scope_at_all_still_means_every_net(self):
        self.assertEqual(self._step()['nets'], ['*'])


class TheBgaZoneRefs(unittest.TestCase):
    """route.py / route_diff.py --no-bga-zones is nargs='*': bare disables
    every BGA exclusion zone, `U1 U3` only those components'. The converter
    held it as a switch, so every recorded ref list replayed as "disable
    ALL" (23 recorded commands)."""

    def _step(self, tool, *flags):
        import manifest_to_plan as m2p
        return m2p.parse_command(['python3', tool, 'in.kicad_pcb',
                                  'out.kicad_pcb', *flags])

    def test_refs_survive(self):
        step = self._step('route.py', '--nets', '*', '--no-bga-zones', 'U1',
                          'U3', '--clearance', '0.1')
        self.assertEqual(step['params'].get('no_bga_zone'), ['U1', 'U3'])
        self.assertEqual(step['params'].get('clearance'), 0.1)

    def test_bare_still_means_every_zone(self):
        step = self._step('route.py', '--no-bga-zones', '--clearance', '0.1')
        self.assertIs(step['params'].get('no_bga_zone'), True)

    def test_a_ref_is_not_read_as_a_net(self):
        """With no --nets, a trailing ref used to land as a POSITIONAL net
        glob, so the step routed a net called `U1` instead of every net."""
        step = self._step('route.py', '--no-bga-zone', 'U1')
        self.assertEqual(step['nets'], ['*'])
        self.assertEqual(step['params'].get('no_bga_zone'), ['U1'])

    def test_route_diff_takes_refs_too(self):
        step = self._step('route_diff.py', '--no-bga-zones', 'U2')
        self.assertEqual(step['params'].get('no_bga_zone'), ['U2'])

    def test_the_plane_tools_keep_their_plain_switch(self):
        step = self._step('route_planes.py', '--nets', 'GND',
                          '--plane-layers', 'In1.Cu', '--no-bga-zones')
        self.assertIs(step['params'].get('no_bga_zone'), True)


class TheGuiParityGate(unittest.TestCase):
    """The counterpart this file is the run_all half of."""

    def test_the_gate_exists_and_its_main_runs_the_enumeration(self):
        self.assertTrue(os.path.isfile(GATE))
        main = _src(GATE).split('\ndef main(', 1)[1]
        self.assertIn('check_flag_coverage(tool)', main)
        self.assertIn('for tool in FLAG_COVERAGE:', main)

    def test_the_whole_converter_gate_passes_on_the_fixture(self):
        r = run_utils.check([sys.executable, '-X', 'utf8', GATE, FIXTURE],
                            accept=True, timeout=300)
        for tool in TheEnumeration.TOOLS:
            self.assertIn('Flag coverage, %s: OK' % tool, r.stdout)
        self.assertIn('0 mismatch(es)', r.stdout)


class TheNegativeControlsExist(unittest.TestCase):
    """The proof the gate can fail lives in the integration lane; this is the
    pointer that keeps it from being dropped quietly."""

    CONTROLS = os.path.join(_TESTS, 'test_route_flag_plan_coverage_controls.py')

    def test_the_controls_file_exists(self):
        self.assertTrue(os.path.isfile(self.CONTROLS))

    def test_every_enumerated_tool_has_a_control(self):
        src = _src(self.CONTROLS)
        for path in ('py_router/route.py', 'py_router/route_diff.py',
                     'py_router/route_planes.py',
                     'py_router/bga_fanout/__init__.py',
                     'py_router/qfn_fanout/__init__.py'):
            self.assertIn("'%s'" % path, src)


if __name__ == '__main__':
    unittest.main(verbosity=2)
