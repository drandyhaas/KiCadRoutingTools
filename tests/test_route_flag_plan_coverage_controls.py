#!/usr/bin/env python3
"""Negative controls for the flag-enumeration gate: it FAILS, for the stated
reason, when a fix is reverted or a flag is added to nothing.

The gate is tests/gui_parity/test_manifest_plan_parity.py's
check_flag_coverage; its fast run_all half is
tests/test_route_flag_plan_coverage.py. Every control here re-runs the whole
gate as a subprocess on a mutated TEMP COPY of the tree (never the repo) and
asserts the refusal's REASON through run_utils.check, which reports a
traceback or an import failure as a broken test rather than a held guard:

  * the --bus converter row AND legacy alias removed -> --bus NOT REACHED;
  * only the alias removed -> the legacy `bus` name resolves nowhere;
  * only the row removed -> the fixture's --bus expectation fails;
  * a made-up flag added to route.py's parser and to nothing else, and one
    each in route_diff.py's, route_planes.py's, bga_fanout.py's and
    qfn_fanout.py's parsers, each named on its own tool's owners;
  * a --no-X flag "fixed" with a plain alias onto its POSITIVE checkbox;
  * a stale CLI-only entry for a flag that reaches the GUI;
  * the route selection no longer reading --component's refs;
  * --no-bga-zones refs, and --plane-net-layers, dropped by the converter;
  * a reset line removed: bus_enabled, fix_drc_check, and each of the five
    keepout / guide-corridor lines, one at a time.
"""
from __future__ import annotations

import io
import os
import shutil
import sys
import tempfile
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import run_utils                                              # noqa: E402

# The tools the gate enumerates; the precondition checks each one.
TOOLS = ('route.py', 'route_diff.py', 'route_planes.py', 'bga_fanout.py',
         'qfn_fanout.py')


def _src(path):
    with io.open(path, encoding='utf-8', newline='') as f:
        return f.read()


# --- negative controls ---------------------------------------------------------
# The gate is run as a subprocess on a TEMP COPY of what it reads: the
# plugin's four GUI files and ai_plan.py, the converter, the gate itself and
# its fixture, and py_router / py_tools / py_placer + krt_capabilities.py for
# the flag scan. Each control mutates one or two files in the copy, asserts
# the gate refuses for the STATED reason (run_utils.check reports a traceback
# or an import failure as a broken test, not a held guard), and restores the
# copy. The repository is never written.

_COPY_DIRS = ('py_router', 'py_tools', 'py_placer', 'tests/stress')
_COPY_FILES = ('krt_capabilities.py',
               'kicad_routing_plugin/ai_plan.py',
               'kicad_routing_plugin/routing_dialog.py',
               'kicad_routing_plugin/differential_gui.py',
               'kicad_routing_plugin/fanout_gui.py',
               'kicad_routing_plugin/planes_gui.py',
               'tests/gui_parity/test_manifest_plan_parity.py',
               'tests/gui_parity/fixtures/sample_redo_commands.sh')

M2P = 'tests/stress/manifest_to_plan.py'
AI_PLAN = 'kicad_routing_plugin/ai_plan.py'
# routing_dialog.py is ipc-migration's swig_gui.py (renamed by the IPC port).
SWIG = 'kicad_routing_plugin/routing_dialog.py'
ROUTE = 'py_router/route.py'
ROUTE_DIFF = 'py_router/route_diff.py'
ROUTE_PLANES = 'py_router/route_planes.py'
BGA_FANOUT = 'py_router/bga_fanout/__init__.py'
QFN_FANOUT = 'py_router/qfn_fanout/__init__.py'
GATE_REL = 'tests/gui_parity/test_manifest_plan_parity.py'


class NegativeControls(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix='route_flag_cov_')
        ign = shutil.ignore_patterns('__pycache__', '*.so', '*.pyd',
                                     '*.kicad_pcb', '*.kicad_pro', '*.png')
        for d in _COPY_DIRS:
            shutil.copytree(os.path.join(_ROOT, d), os.path.join(cls.tmp, d),
                            ignore=ign)
        for f in _COPY_FILES:
            dst = os.path.join(cls.tmp, f)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy(os.path.join(_ROOT, f), dst)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def _path(self, rel):
        return os.path.join(self.tmp, rel)

    def _mutate(self, rel, old, new, within=None):
        """Replace ONE exact occurrence in the copy; restored on cleanup.

        Anchors are SINGLE-LINE and carry no newline, so they match an LF and
        a CRLF checkout alike (read and written with newline=''); a removed
        statement becomes `pass`, so the file stays valid Python. `within`
        names a `def` whose body the anchor must be unique in -- the reset
        lines repeat the constructor's own lines byte for byte."""
        path = self._path(rel)
        before = _src(path)
        start, end = 0, len(before)
        if within is not None:
            start = before.index('def %s(' % within)
            nxt = before.find('\n    def ', start + 1)
            end = nxt if nxt != -1 else end
        region = before[start:end]
        self.assertNotIn('\n', old)
        self.assertEqual(region.count(old), 1,
                         'mutation anchor %r is not unique in %s%s -- the '
                         'control would test nothing'
                         % (old, rel, ' ' + within if within else ''))
        with io.open(path, 'w', encoding='utf-8', newline='') as f:
            f.write(before[:start] + region.replace(old, new) + before[end:])

        def restore():
            with io.open(path, 'w', encoding='utf-8', newline='') as f:
                f.write(before)
        self.addCleanup(restore)
        return restore

    def _run(self, **kw):
        gate = self._path(GATE_REL)
        fixture = self._path('tests/gui_parity/fixtures/'
                             'sample_redo_commands.sh')
        return run_utils.check([sys.executable, '-X', 'utf8', gate, fixture],
                               cwd=self.tmp, timeout=300, **kw)

    def test_0_the_unmutated_copy_passes(self):
        """The precondition: without it, a refusal below could be the COPY's
        fault rather than the mutation's."""
        r = self._run(accept=True)
        for tool in TOOLS:
            self.assertIn('Flag coverage, %s: OK' % tool, r.stdout)

    def test_bus_row_and_alias_removed(self):
        self._mutate(M2P, "'--bus': 'bus_enabled',", '')
        self._mutate(AI_PLAN, "'bus': 'bus_enabled',", '')
        self._run(refuse="--bus: NOT REACHED -- param 'bus' -> 'bus', which "
                         "is no settable control")

    def test_bus_alias_removed_only(self):
        """The converter still emits bus_enabled, so the ENUMERATION holds;
        a plan converted before the row carries `bus`, and that name must
        still resolve."""
        self._mutate(AI_PLAN, "'bus': 'bus_enabled',", '')
        self._run(refuse='bus: no control, no alias')

    def test_bus_row_removed_only(self):
        """The legacy alias still reaches the control, so the ENUMERATION
        holds; the fixture's independent expectation is what catches a
        converter that stopped emitting the control's own name."""
        self._mutate(M2P, "'--bus': 'bus_enabled',", '')
        self._run(refuse='--bus: bool flag not set (bus_enabled)')

    def test_a_made_up_route_flag_added_to_nothing(self):
        anchor = 'parser.add_argument("--bus", action="store_true",'
        self._mutate(ROUTE, anchor,
                     'parser.add_argument("--zz-probe-knob", type=float, '
                     'default=1.0, help="negative control"); ' + anchor)
        self._run(refuse="--zz-probe-knob: NOT REACHED -- param "
                         "'zz_probe_knob'")

    def test_a_made_up_flag_on_each_other_tool(self):
        """One per tool, each refused on ITS OWN action's owners: the probe,
        the owner chain and the lists are per tool, and a control that only
        exercised route.py would prove nothing about the others."""
        for path, anchor, flag, owners in (
                (ROUTE_DIFF,
                 'parser.add_argument("--ac-couple-match", action="store_true",',
                 '--zz-diff-probe', "route_diff owners ['differential_tab'"),
                (ROUTE_PLANES,
                 'parser.add_argument("--dry-run", action="store_true",',
                 '--zz-planes-probe', "route_planes owners ['create_options'"),
                (BGA_FANOUT,
                 "parser.add_argument('--check-for-previous', "
                 "action='store_true',",
                 '--zz-bga-probe', "fanout owners ['bga_options'"),
                (QFN_FANOUT,
                 "parser.add_argument('--layer', '-l', default=None,",
                 '--zz-qfn-probe', "fanout owners ['bga_options'")):
            with self.subTest(tool=path):
                restore = self._mutate(
                    path, anchor,
                    'parser.add_argument("%s", type=float, default=1.0, '
                    'help="negative control"); %s' % (flag, anchor))
                try:
                    self._run(refuse="%s: NOT REACHED -- param %r -> %r, "
                                     "which is no settable control on the %s"
                                     % (flag, flag[2:].replace('-', '_'),
                                        flag[2:].replace('-', '_'), owners))
                finally:
                    restore()

    def test_the_bga_coupled_pair_gap_row_reverted(self):
        """Without its bga_fanout row the gap converts to the diff tab's
        name. The enumeration still resolves it (the fanout block re-homes
        that legacy name), so it is the fixture's expectation that fails."""
        self._mutate(M2P, "'bga_fanout.py': {'--diff-pair-gap': "
                          "'bga_diff_pair_gap'},", "'bga_fanout.py': {},")
        self._run(refuse='--diff-pair-gap: want 0.1143 got None')

    def test_the_bga_coupled_pair_gap_reset_line_is_load_bearing(self):
        self._mutate(SWIG, "('bga_diff_pair_gap', defaults.BGA_DIFF_PAIR_GAP)):",
                     "):", within='reset_params_to_defaults')
        self._run(refuse='--diff-pair-gap: LEAKS between plan steps: '
                         'bga_diff_pair_gap is not restored')

    def test_plane_net_layers_dropped_by_the_converter_again(self):
        """The largest fix the enumeration found: 26 kept bga_fanout steps
        whose future-pour declaration the converter collected and never
        copied into the step."""
        self._mutate(M2P, "'--rip-existing-nets', '--plane-net-layers', '--keep-away'):",
                     "'--rip-existing-nets', '--keep-away'):")
        self._run(refuse='--plane-net-layers: NOT REACHED -- the converter '
                         'consumes it and emits nothing')

    def test_a_no_flag_aliased_onto_its_positive_checkbox(self):
        """A plain alias renames; it cannot invert. `no_smoothing -> smoothing`
        would TICK the box the flag exists to untick."""
        self._mutate(M2P, "'--no-smoothing': 'smoothing',", '')
        self._mutate(AI_PLAN, "'bus': 'bus_enabled',",
                     "'bus': 'bus_enabled', 'no_smoothing': 'smoothing',")
        self._run(refuse="a --no-X switch landing on the POSITIVE checkbox "
                         "'smoothing' must untick it")

    def test_the_bga_zone_refs_are_dropped_again(self):
        """The converter's old reading: every --no-bga-zones is a bare switch.
        The fixture's route_diff step names U7 U9, so the refs must show."""
        self._mutate(M2P, "step['params'][tool_optional_lists[a]] = vals or True",
                     "step['params'][tool_optional_lists[a]] = True")
        self._run(refuse="--no-bga-zones: want ['U7', 'U9'] (refs, or True "
                         "for bare) got True")

    def test_the_fix_drc_reset_line_is_load_bearing(self):
        """Without it, a step replaying --no-fix-drc-settings would leave the
        box unticked, and every later step would skip the DRC-floor writeback
        that route.py performs for any step without the flag."""
        self._mutate(SWIG, 'self.fix_drc_check.SetValue(True)', 'pass',
                     within='reset_params_to_defaults')
        self._run(refuse='--no-fix-drc-settings: LEAKS between plan steps: '
                         'fix_drc_check is not restored')

    def test_the_route_selection_stops_reading_the_component(self):
        """What shipped until now: the converter carried the refs and the
        route selection never looked at them, so the step routed every net."""
        self._mutate(AI_PLAN, 'refs = _step_component_refs(step)', 'refs = []',
                     within='apply_step_selection')
        self._run(refuse="--component: NOT REACHED -- step['component'], "
                         "which the route selection never reads")

    def test_a_stale_cli_only_entry(self):
        self._mutate(GATE_REL, "ROUTE_CLI_ONLY = {",
                     "ROUTE_CLI_ONLY = {'--bus': 'negative control',")
        self._run(refuse='--bus: STALE list entry: it now reaches the GUI')

    def test_a_control_missing_from_the_reset(self):
        self._mutate(SWIG, 'self.bus_enabled.SetValue(False)', 'pass',
                     within='reset_params_to_defaults')
        self._run(refuse='--bus: LEAKS between plan steps: bus_enabled is '
                         'not restored by reset_params_to_defaults')

    def test_each_keepout_and_corridor_reset_line_is_load_bearing(self):
        """The five reset lines that ended the keepout / guide-corridor leak.
        keepout_check and guide_corridor_check had leaked since they were
        aliased; each line is removed alone and must be named."""
        for line, flag, ctrl in (
                ('self.keepout_check.SetValue(defaults.KEEPOUT_ENABLED)',
                 '--keepout', 'keepout_check'),
                ('self.keepout_layer_ctrl.SetValue(defaults.KEEPOUT_LAYER)',
                 '--keepout-layer', 'keepout_layer_ctrl'),
                ('self.guide_corridor_check.SetValue('
                 'defaults.GUIDE_CORRIDOR_ENABLED)',
                 '--guide-corridor', 'guide_corridor_check'),
                ('self.guide_corridor_layer_ctrl.SetValue('
                 'defaults.GUIDE_CORRIDOR_LAYER)',
                 '--guide-corridor-layer', 'guide_corridor_layer_ctrl'),
                ('self.guide_corridor_spacing_ctrl.SetValue('
                 'str(defaults.GUIDE_CORRIDOR_SPACING))',
                 '--guide-corridor-spacing', 'guide_corridor_spacing_ctrl')):
            with self.subTest(flag=flag):
                restore = self._mutate(SWIG, line, 'pass',
                                       within='reset_params_to_defaults')
                try:
                    self._run(refuse='%s: LEAKS between plan steps: %s is not '
                                     'restored' % (flag, ctrl))
                finally:
                    restore()


if __name__ == '__main__':
    unittest.main(verbosity=2)
