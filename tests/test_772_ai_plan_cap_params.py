"""#772: an `optimize_caps` plan step's params must reach the cap engine.

`ai_plan.apply_step_params._owners()` returned `[dialog]` for any action other
than route_diff / fanout / route_planes / repair_planes. `optimize_caps` was not
in that list, and all ten "Cap Placement (advanced)" controls live on
`fanout_tab.bga_options` -- so every `cap_*` param a converted manifest carries
was logged "no control, ignored" and the engine ran at its signature defaults.
`--board-edge-clearance` DID resolve, onto the Basic tab's SIGNAL
copper-to-edge control, which is a different quantity and left its override
box ticked for the next step.

THIS FILE IS THE WX-FREE HALF. It asserts the SHAPE of the fix -- the owner
table, the skip, the re-homing block, the one defaults table, the scoped reset
-- none of which needs a dialog. The behavioural claim, "the value the engine is
actually handed", cannot be made without wx and is made by
`tests/gui_parity/test_772_cap_params_reach_engine.py`, whose existence this
file asserts because `run_all.py`'s flat glob never collects that directory.

WHY A SHAPE GATE IS NOT ENOUGH ON ITS OWN, and why this file says so rather
than pretending otherwise: #772 shipped *past* two green gates. The converter
gate asserted the flag survives into `step['params']` (it did), and the
resolution gate asserted the param name matches some `self.X = ...` somewhere
across the four GUI files (every cap_* control has always existed). Neither
knew about OWNERS. So the arms here are deliberately about the owner table --
the thing whose absence was invisible -- and the real gate measures delivery.
"""
from __future__ import annotations

RUN_ALL_FAST_OK = True

import ast
import io
import os
import sys
import unittest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

AI_PLAN = os.path.join(_ROOT, 'kicad_routing_plugin', 'ai_plan.py')
FANOUT_GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'fanout_gui.py')
SWIG_GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'swig_gui.py')
M2P = os.path.join(_ROOT, 'tests', 'stress', 'manifest_to_plan.py')

# The ten knobs, spelled here INDEPENDENTLY of the engine table on purpose: an
# arm that imports the table it is checking agrees with itself by construction.
CAP_KNOBS = (
    'cap_capture_radius', 'cap_near_margin', 'cap_step',
    'cap_max_displacement', 'cap_max_displacement_cap',
    'cap_displacement_growth', 'cap_board_edge_clearance',
    'cap_max_passes', 'cap_prefix', 'cap_allow_rotation',
)


def _src(path):
    with io.open(path, encoding='utf-8') as f:
        return f.read()


def _literal(path, name):
    """AST-extract a module-level literal without importing (these import wx)."""
    for node in ast.parse(_src(path)).body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == name:
                    return ast.literal_eval(node.value)
    return None


class TestTheOwnerTable(unittest.TestCase):
    """`_ACTION_OWNERS` is the single decision about what a step can reach."""

    def setUp(self):
        self.owners = _literal(AI_PLAN, '_ACTION_OWNERS')

    def test_the_table_is_a_module_level_literal(self):
        """It has to be, or the parity gate cannot AST-extract it -- and that
        extraction is the check whose absence let #772 ship."""
        self.assertIsInstance(self.owners, dict)
        self.assertIn('optimize_caps', self.owners)

    def test_optimize_caps_searches_the_bga_options_panel(self):
        tab, subs = self.owners['optimize_caps']
        self.assertEqual(tab, 'fanout_tab')
        self.assertIn('bga_options', subs,
                      'the cap step cannot reach a single cap control without '
                      'the panel in its owner list -- this IS #772')
    # MUTATION: drop 'bga_options' -> the real-dialog gate reports 10 dropped.

    def test_the_planes_actions_keep_their_sub_panel(self):
        """A regression guard on the branch this table replaced: the planes
        actions were the ONLY ones that descended into a sub-panel, and the
        rewrite must not have lost that."""
        for act in ('route_planes', 'repair_planes'):
            self.assertEqual(self.owners[act], ('planes_tab',
                                                ('create_options',)), act)

    def test_route_is_deliberately_absent(self):
        """Its controls are all on the dialog, so it must fall through to the
        [dialog] default rather than acquiring a tab it does not have."""
        self.assertNotIn('route', self.owners)


class TestTheEdgeClearanceIsRehomed(unittest.TestCase):
    """`--board-edge-clearance` means a different quantity on a cap step than
    on a route step, and the #733 follow-up split the two controls apart."""

    def test_the_converter_emits_the_CAP_name(self):
        flags = _literal(M2P, 'CAP_FLAG_PARAMS')
        self.assertEqual(flags['--board-edge-clearance'],
                         'cap_board_edge_clearance')
        self.assertNotIn('board_edge_clearance', set(flags.values()),
                         'a cap step still converts to the Basic tab SIGNAL '
                         'name, which ai_plan ticks edge_clearance_check for')

    def test_the_route_flag_is_untouched(self):
        """The SAME spelling on a ROUTE step is the signal keep-out and must
        keep mapping to the Basic tab control."""
        self.assertEqual(_literal(M2P, 'FLAG_PARAMS')['--board-edge-clearance'],
                         'board_edge_clearance')

    def test_the_grid_step_row_exists(self):
        self.assertEqual(_literal(M2P, 'CAP_FLAG_PARAMS')['--grid-step'],
                         'grid_step')

    def test_the_generic_loop_skips_the_legacy_spelling_on_a_cap_step(self):
        src = _src(AI_PLAN)
        self.assertIn('"optimize_caps": {"board_edge_clearance"},', src,
                      'without the skip the generic loop puts a PLACEMENT '
                      'margin on the SIGNAL control and ticks its override')

    def test_clearance_is_NOT_skipped(self):
        """#768's GIVEN branch runs through the generic loop: setting Min
        Clearance and ticking its override IS the GUI's spelling of
        `--clearance was given`, and it delivers netclass_ceiling too.

        `_GENERIC_SKIP` is a function LOCAL, so it cannot be read as a
        module literal. The first version of this arm tried, fell into an
        `if skip is None` branch that sliced 60 characters of source and
        asserted a substring the slice could never contain, and was
        therefore unfalsifiable -- an adversarial review killed it by
        adding `clearance` to the skip through a second mechanism and
        watching this file stay green.

        Parsed properly instead: walk the AST for the assignment inside
        apply_step_params and read the real value. The behavioural claim --
        that clearance and netclass_ceiling reach the engine -- is arm 8 of
        the real-dialog gate; this one pins the mechanism that makes it
        possible."""
        skip = None
        for node in ast.walk(ast.parse(_src(AI_PLAN))):
            if (isinstance(node, ast.FunctionDef)
                    and node.name == 'apply_step_params'):
                for n in ast.walk(node):
                    if (isinstance(n, ast.Assign) and len(n.targets) == 1
                            and isinstance(n.targets[0], ast.Name)
                            and n.targets[0].id == '_GENERIC_SKIP'):
                        skip = ast.literal_eval(n.value)
        self.assertIsInstance(
            skip, dict,
            '_GENERIC_SKIP could not be read, so this arm proves nothing')
        self.assertIn('optimize_caps', skip)
        self.assertEqual(skip['optimize_caps'], {'board_edge_clearance'},
                         'the cap skip changed; `clearance` in particular '
                         'must NOT be there, or #768\'s GIVEN branch stops '
                         'running and netclass_ceiling goes None')

    def test_the_drc_floor_harvest_excludes_a_cap_steps_edge_margin(self):
        """place_fanout_clearance's own writeback: "a placement margin, not a
        routing-enforced floor, and must not tighten the rule"."""
        src = _src(AI_PLAN)
        self.assertIn("if step.get('action') == 'optimize_caps':", src)
        self.assertIn("if k != 'board_edge_clearance')", src)


class TestTheOneDefaultsTable(unittest.TestCase):
    """CLAUDE.md: add a control to reset_params_to_defaults or it leaks between
    steps. Eight of the ten had never been in it."""

    def test_every_cap_knob_is_in_the_table(self):
        src = _src(FANOUT_GUI)
        i = src.index('CAP_PARAM_DEFAULTS')
        block = src[i:src.index(')', src.index('cap_allow_rotation', i)) + 1]
        for name in CAP_KNOBS:
            self.assertIn("'%s'" % name, block,
                          '%s is not in CAP_PARAM_DEFAULTS, so neither reset '
                          'covers it and ai_plan cannot see that a step named '
                          'it' % name)

    def test_the_full_reset_delegates_rather_than_copying(self):
        src = _src(SWIG_GUI)
        self.assertIn('def reset_cap_params_to_defaults(self):', src)
        self.assertIn('self.reset_cap_params_to_defaults()', src)
        # ...and the two rows it used to carry are gone, so there is ONE table
        for stale in ("('cap_allow_rotation', True),",
                      "('cap_max_passes', 30),"):
            self.assertNotIn(stale, src,
                             'a second copy of %r survives the delegation; '
                             'two lists in two files is the shape #772 exists '
                             'to remove' % stale)

    def test_the_scoped_reset_is_conditional_on_the_step_having_params(self):
        """Unconditional, it would destroy the inheritance the reset
        exception exists for -- the auto-inserted step's whole purpose.

        THE CONDITION IS "the plan specified this step", not "the step
        names a cap knob". An adversarial review measured the difference:
        a manifest step converted from `--clearance 0.1 --grid-step 0.05`
        carries params but names no CAP knob, so the name-based test
        skipped the reset and that step ran at the PREVIOUS cap step's
        knobs. `params` is exact, because _insert_cap_optimization emits
        no params key at all."""
        src = _src(AI_PLAN)
        self.assertIn('reset_cap_params_to_defaults', src)
        self.assertIn('_given = sorted(step.get("params") or {})', src)
        self.assertIn('if _given and hasattr(self.dialog,', src)
        self.assertNotIn('& _names)', src,
                         'the reset is gated on cap-knob NAMES again, so a '
                         'step carrying only Basic-tab params inherits the '
                         'previous cap step')


class TestTheAutoInsertedStepCarriesNoDeadKey(unittest.TestCase):

    def test_no_top_level_cap_prefix(self):
        """It sat OUTSIDE `params`, which is the only place apply_step_params
        reads, so it was inert while reading as though it set the prefix."""
        src = _src(AI_PLAN)
        self.assertIn('steps.insert(last_bga + 1, {"action": "optimize_caps"})',
                      src)
        self.assertNotIn('{"action": "optimize_caps", "cap_prefix"', src)


class TestTheRealDialogGateIsRegistered(unittest.TestCase):
    """A behavioural claim this file cannot make must be made SOMEWHERE.

    `run_all.py` globs `tests/test_*.py` only, so nothing in the default suite
    runs `tests/gui_parity/**`. This is the pointer that keeps the gate from
    being quietly dropped."""

    def test_the_delivery_gate_exists(self):
        gate = os.path.join(_ROOT, 'tests', 'gui_parity',
                            'test_772_cap_params_reach_engine.py')
        self.assertTrue(os.path.isfile(gate),
                        'the wx-free arms above assert the SHAPE of the fix; '
                        'only the real-dialog gate measures what the engine is '
                        'handed, and #772 shipped past two gates that checked '
                        'shape alone')

    def test_the_gate_asserts_delivery_and_not_conversion(self):
        with io.open(os.path.join(_ROOT, 'tests', 'gui_parity',
                                  'test_772_cap_params_reach_engine.py'),
                     encoding='utf-8') as f:
            src = f.read()
        # it must spy on the SOURCE module: _optimize_decoupling_caps imports
        # the engine inside the method, so a fanout_gui patch records nothing
        self.assertIn('_fc.repair_fanout_clearance = _spy', src)
        self.assertIn('from placement import fanout_clearance as _fc', src)
        # ...and it must refuse a run where the engine was never reached
        self.assertIn('the engine was not reached', src)


if __name__ == '__main__':
    unittest.main(verbosity=2)
