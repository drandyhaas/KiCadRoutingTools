"""The plugin must not load the standalone bus solver (#1938).

OR-Tools and KiCad ship incompatible Windows DLLs with the same names: loading
OR-Tools after pcbnew raises WinError 127 before the routing dialog opens. On
main the SWIG plugin's dependency dialog (deps_check) excludes it from its
probes and install list. ipc-migration has no deps_check: KiCad provisions the
IPC plugin's venv from requirements.txt itself, so the same contract is that
the file KiCad provisions from does not list OR-Tools (it cannot share an
environment with kicad-python either: see requirements.txt) and no plugin
module imports it. The standalone bus gate still probes it.

Run with: python3 tests/test_plugin_cli_dependencies.py
"""
import ast
import os
import re
import sys
import unittest
from unittest.mock import patch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from startup_checks import IMPORT_TESTS, check_bus_dependencies   # noqa: E402

PLUGIN = os.path.join(ROOT, 'kicad_routing_plugin')


def _requirement_names(path):
    names = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if line:
                names.append(re.split(r'[\s;<>=~!\[]', line, maxsplit=1)[0].lower())
    return names


class PluginCliDependenciesTest(unittest.TestCase):
    def test_provisioned_venv_excludes_bus_solver(self):
        names = _requirement_names(os.path.join(ROOT, 'requirements.txt'))
        self.assertIn('kicad-python', names)          # the file KiCad provisions
        self.assertNotIn('ortools', names)

    def test_no_plugin_module_imports_bus_solver(self):
        hits = []
        for fn in sorted(os.listdir(PLUGIN)):
            if not fn.endswith('.py'):
                continue
            with open(os.path.join(PLUGIN, fn), encoding='utf-8') as f:
                tree = ast.parse(f.read(), fn)
            for node in ast.walk(tree):
                mods = ([a.name for a in node.names] if isinstance(node, ast.Import)
                        else [node.module or ''] if isinstance(node, ast.ImportFrom)
                        else [])
                hits += [f"{fn}:{node.lineno}" for m in mods
                         if m == 'ortools' or m.startswith('ortools.')]
        self.assertEqual(hits, [])

    def test_standalone_bus_still_probes_ortools(self):
        # Keeping it out of the plugin must not bypass the CLI gate.
        with patch.dict(IMPORT_TESTS,
                        {'ortools': "raise RuntimeError('bus solver probed')"}):
            with self.assertRaisesRegex(RuntimeError, 'bus solver probed'):
                check_bus_dependencies()


if __name__ == '__main__':
    unittest.main()
