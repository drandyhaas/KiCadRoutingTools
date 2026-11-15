"""The route-plan CLI uses the recorded command format without the stress harness."""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'py_router'))

from redo_record import parse_manifest


VERIFY = (
    '# python3 -X utf8 py_router/check_drc.py final.kicad_pcb\n'
    '# python3 -X utf8 py_router/check_connected.py final.kicad_pcb\n'
    '# python3 -X utf8 py_tools/check_orphan_stubs.py final.kicad_pcb\n'
)
POUR = 'python3 -X utf8 py_router/route_planes.py {in} {out} --nets GND --plane-layers B.Cu\n'
ROUTE = "python3 -X utf8 py_router/route.py {in} {out} --nets '*' --power-nets GND --json-out {dir}/route.json\n"


class StandaloneRoutePlanTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'runtime'
        files = [
            'krt_cli.py',
            'krt_standalone/__init__.py',
            'krt_standalone/cli.py',
            'krt_standalone/process.py',
            'krt_standalone/tools.py',
            'py_placer/_path.py',
            'py_placer/judge_seeds.py',
            'py_router/routing_defaults.py',
            'py_router/fab_tiers.py',
            'py_router/list_nets.py',
            'py_router/kicad_parser.py',
            'py_router/swig_compat.py',
            'py_router/paste_apertures.py',
            'py_router/env_knobs.py',
            'py_router/plane_groups.py',
            'py_router/make_plan.py',
            'py_router/redo_record.py',
            '.claude/skills/plan-pcb-routing/scripts/route_plan_check.py',
        ]
        for name in files:
            target = self.root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, target)

    def run_plan(self, text, name='chain.txt'):
        plan = Path(self.temp.name) / name
        if text is not None:
            plan.write_text(text, encoding='utf-8')
        env = dict(os.environ)
        env.pop('PYTHONPATH', None)
        return subprocess.run(
            [sys.executable, '-B', str(self.root / 'krt_cli.py'), 'check-route-plan', str(plan)],
            cwd=self.temp.name, env=env, capture_output=True, text=True,
        )

    def test_judging_chain_runs_without_stress_harness(self):
        result = self.run_plan(VERIFY + POUR + ROUTE)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('a judging chain', result.stdout)
        self.assertFalse((self.root / 'tests').exists())

    def test_shell_manifest_runs_without_stress_harness(self):
        plan = 'set -e\ncd /repo\n' + VERIFY + POUR + ROUTE
        plan = plan.replace('{in}', 'input.kicad_pcb').replace('{out}', 'output.kicad_pcb')
        result = self.run_plan(plan.replace('{dir}', '.'), 'plan.sh')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn('a judging chain', result.stdout)

    def test_refused_plan_names_the_failed_rule(self):
        result = self.run_plan(VERIFY + POUR + ROUTE + POUR)
        self.assertEqual(result.returncode, 4, result.stdout + result.stderr)
        self.assertIn('FAIL  R04', result.stdout)
        self.assertIn('last board-mutating command is route_planes.py', result.stdout)

    def test_malformed_and_missing_plans_are_unreadable(self):
        result = self.run_plan(VERIFY + POUR + ROUTE.replace('{dir}', '{unknown}'))
        self.assertEqual(result.returncode, 3, result.stdout + result.stderr)
        self.assertIn('names {unknown}', result.stderr)
        missing = self.run_plan(None, 'missing.txt')
        self.assertEqual(missing.returncode, 3, missing.stdout + missing.stderr)
        self.assertIn('missing.txt', missing.stderr)


class RecordedManifestTests(unittest.TestCase):
    def test_cwd_exports_and_multiline_arguments_survive_parsing(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest = Path(temp) / 'plan.sh'
            manifest.write_text(
                'cd "run directory"\n'
                'export KICAD_PLANE_FRAGILITY_COST=0 REDO_MANIFEST=ignored\n'
                'python3 py_router/route.py input.kicad_pcb output.kicad_pcb '
                "--net-clearances '{\n\"/SIGNAL\": 0.2\n}'\n"
                'cp output.kicad_pcb final.kicad_pcb\n',
                encoding='utf-8',
            )
            rows = parse_manifest(manifest)
            self.assertEqual(len(rows), 2)
            self.assertEqual([cwd for cwd, _argv in rows], [str(Path(temp) / 'run directory')] * 2)
            self.assertEqual(rows[0][1][0], 'KICAD_PLANE_FRAGILITY_COST=0')
            self.assertEqual(rows[0][1][-1], '{\n"/SIGNAL": 0.2\n}')
            self.assertEqual(rows[1][1], [
                'KICAD_PLANE_FRAGILITY_COST=0', 'cp', 'output.kicad_pcb', 'final.kicad_pcb',
            ])

    def test_unterminated_argument_does_not_drop_the_final_command(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest = Path(temp) / 'plan.sh'
            manifest.write_text("python3 py_router/route.py 'unfinished\n", encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'unterminated quote'):
                parse_manifest(manifest)


if __name__ == '__main__':
    unittest.main()
