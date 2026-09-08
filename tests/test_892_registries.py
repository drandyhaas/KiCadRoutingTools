#!/usr/bin/env python3
"""#892: a new pose-writing CLI has to JOIN the registries, or it is invisible.

Four of them, each with a different failure if the entry is missing, and none
of the four failures looks like "a registration is missing" from the outside:

* `provenance.LEVER_REGISTRY` -- every board the tool writes under an unaided
  work dir raises `UnaidedViolation`, and `provenance_audit` exits 4;
* `manifest_to_plan.REFUSED_TOOLS` -- a recorded `place_pose.py` step converts
  through the UNKNOWN-tool path, which only bumps a `skipped` counter, so the
  converted plan looks complete while the replayed chain diverges at a step
  nobody named;
* `test_cli_postpass_coverage.CLI_MAINS` -- a finalization pass added to the
  CLI's `main()` later would never be scanned for a GUI counterpart;
* `krt_capabilities` -- a consumer that cannot discover the sanctioned pose
  setter writes the hand script instead, which is the whole failure #892 ends.

THIS IS THE WX-FREE HALF, and it also asserts that its `tests/gui_parity/`
counterpart exists at all: `run_all.py`'s flat glob never collects that
directory, so a gate living only there is a gate this suite cannot fail on.
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
for _p in (_ROOT, os.path.join(_ROOT, 'py_router'),
           os.path.join(_ROOT, 'py_tools'), os.path.join(_ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

TOOL = 'place_pose.py'


def _module_constant(path, name):
    """A module-level list/tuple constant, read by AST (no import, no wx)."""
    with io.open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read(), filename=path)
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for tgt in node.targets:
            if isinstance(tgt, ast.Name) and tgt.id == name:
                return ast.literal_eval(node.value)
    raise AssertionError("%s has no module-level %s" % (path, name))


class LeverRegistry(unittest.TestCase):
    def test_place_pose_may_author_poses(self):
        from placement import provenance
        self.assertIn(TOOL, provenance.LEVER_REGISTRY)

    def test_the_gate_reads_the_constant(self):
        # The refusal at `record_write` tests the module constant, not the
        # `lever_registry` snapshot `start_regime` writes into a manifest --
        # so an OLD work dir's manifest is a record of what was registered
        # then, and not a second gate that also has to be updated. Asserted
        # because the opposite belief would send someone editing manifests.
        from placement import provenance
        with io.open(provenance.__file__, encoding='utf-8') as f:
            src = f.read()
        self.assertIn("if lever['lever'] not in LEVER_REGISTRY:", src)


class ManifestConverter(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, os.path.join(_ROOT, 'tests', 'stress'))
        import manifest_to_plan
        self.m2p = manifest_to_plan

    def test_registered_as_refused(self):
        self.assertIn(TOOL, self.m2p.REFUSED_TOOLS)

    def test_a_recorded_step_is_refused_loudly(self):
        step = self.m2p.parse_command(
            ['python3', TOOL, 'a.kicad_pcb', 'b.kicad_pcb',
             'set', 'U1', '1', '2'])
        self.assertTrue(step, "converted to nothing -- the chain link is lost")
        self.assertIn('_refused', step)

    def test_the_refusal_says_why(self):
        self.assertIn('placement step', self.m2p.REFUSED_TOOLS[TOOL])


class PostPassCoverage(unittest.TestCase):
    GATE = os.path.join(_TESTS, 'gui_parity', 'test_cli_postpass_coverage.py')

    def test_gate_exists(self):
        self.assertTrue(os.path.isfile(self.GATE))

    def test_place_pose_is_scanned(self):
        self.assertIn('py_placer/%s' % TOOL,
                      _module_constant(self.GATE, 'CLI_MAINS'))

    def test_the_engine_owns_the_board_work(self):
        # The gate stays green only because `main()` runs no post-engine pass:
        # the sibling carry and the legality re-grade live in `pose_ops`, which
        # both fronts call. If they migrate into `main()` this assertion is the
        # one that notices before the parity gate does.
        with io.open(os.path.join(_ROOT, 'py_placer', TOOL),
                     encoding='utf-8') as f:
            src = f.read()
        main_src = src.split('def main(', 1)[1]
        for leaked in ('copy_siblings', 'write_placed_output',
                       'grade_pad_legality', 'fix_project_for_output'):
            self.assertNotIn(leaked, main_src,
                             "%s moved into place_pose.main(); it belongs in "
                             "placement/pose_ops.py, where the GUI can reach "
                             "it too" % leaked)


class ParityGateForRefusals(unittest.TestCase):
    GATE = os.path.join(_TESTS, 'gui_parity', 'test_manifest_plan_parity.py')

    def test_gate_exists(self):
        self.assertTrue(os.path.isfile(self.GATE))

    def test_gate_walks_every_registered_refusal(self):
        # It used to name four of the seven refused tools by hand, so an
        # eighth could join REFUSED_TOOLS and never be exercised. A whitelist
        # is where a guard fails.
        with io.open(self.GATE, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('for tool in sorted(m2p.REFUSED_TOOLS):', src)
        self.assertIn("'%s'" % TOOL, src)


class Capabilities(unittest.TestCase):
    def test_inventoried(self):
        import krt_capabilities as caps
        self.assertIn(TOOL, caps.KNOWN_MODULES)

    def test_deliberately_not_a_flag_script(self):
        """FLAG_SCRIPTS is a CONTRACT, and this tool cannot meet it.

        `tests/test_798_registrar_flags.py` reads that tuple as "every flag
        the source registers is visible in --help as an option AND accepted by
        the top-level parser". `place_pose`'s `--rot` / `--near` /
        `--relative` belong to per-VERB parsers, so they are neither, and
        listing the tool there turned the gate red for telling the truth
        (measured: `place_pose.py claims flags argparse rejects: ['--near',
        '--relative', '--rot']`). The verbs are registered as SUBPARSERS
        instead, which is how `--help` and the documented-flag gate find them.
        """
        import krt_capabilities as caps
        self.assertNotIn(TOOL, caps.FLAG_SCRIPTS)

    def test_the_verb_flags_are_discoverable_anyway(self):
        import argparse
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            'place_pose_probe', os.path.join(_ROOT, 'py_placer', TOOL))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        def options(parser):
            out = {s for a in parser._actions for s in a.option_strings}
            for a in parser._actions:
                if isinstance(a, argparse._SubParsersAction):
                    for sub in a.choices.values():
                        out |= options(sub)
            return out

        found = options(mod.build_parser())
        for flag in ('--rot', '--near', '--relative'):
            self.assertIn(flag, found, "%s is documented in the skill and the "
                                       "docs; test_431_skill_commands walks "
                                       "the subparsers action to find it"
                          % flag)
        # ...and the parser `main()` actually parses with must NOT carry them,
        # or the first verb's flags would be eaten globally and a second verb
        # could not carry its own.
        self.assertNotIn('--rot', options(mod.build_parser(with_verbs=False)))

    def test_resolvable_on_disk(self):
        self.assertTrue(os.path.isfile(
            os.path.join(_ROOT, 'py_placer', TOOL)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
