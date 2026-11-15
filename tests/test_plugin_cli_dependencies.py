"""The embedded GUI must not load the standalone bus solver's native DLLs.

OR-Tools and KiCad ship incompatible Windows DLLs with the same names. Loading
OR-Tools after pcbnew raises WinError 127 before the routing dialog opens.
Run with: python3 tests/test_plugin_cli_dependencies.py
"""
import os
import sys
import types
import unittest
from unittest.mock import patch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.modules.setdefault('wx', types.ModuleType('wx'))

from kicad_routing_plugin import deps_check
from startup_checks import check_bus_dependencies


class PluginCliDependenciesTest(unittest.TestCase):
    def test_gui_does_not_load_bus_solver(self):
        # Exercise the real dependency gate with the Windows failure planted
        # at the import seam; no Windows or installed OR-Tools is required.
        with patch.object(deps_check, '_parse_requirements',
                          return_value=['ortools']), patch.dict(
                deps_check.IMPORT_TESTS,
                {'ortools': "raise OSError(127, 'DLL procedure not found')"}):
            self.assertTrue(deps_check.ensure_dependencies())

    def test_gui_install_list_excludes_bus_solver(self):
        with patch.object(deps_check, '_parse_requirements',
                          return_value=['numpy', 'Pillow', 'ortools']):
            self.assertEqual([name for name, _ in deps_check._required_packages()],
                             ['numpy', 'Pillow'])

    def test_missing_gui_dependencies_are_still_reported(self):
        with patch.object(deps_check, '_parse_requirements',
                          return_value=['numpy', 'Pillow', 'ortools']), patch.dict(
                deps_check.IMPORT_TESTS, {
                    'numpy': 'import _krt_missing_gui_numpy',
                    'Pillow': 'import _krt_missing_gui_pillow',
                    'ortools': "raise OSError(127, 'DLL procedure not found')",
                }):
            blocking, optional = deps_check._missing_packages()
        self.assertEqual(blocking, ['numpy'])
        self.assertEqual(optional, ['Pillow'])

    def test_standalone_bus_still_probes_ortools(self):
        # Excluding it from the embedded GUI must not bypass the CLI gate.
        with patch.dict(deps_check.IMPORT_TESTS,
                        {'ortools': "raise RuntimeError('bus solver probed')"}):
            with self.assertRaisesRegex(RuntimeError, 'bus solver probed'):
                check_bus_dependencies()


if __name__ == '__main__':
    unittest.main()
