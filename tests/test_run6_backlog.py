"""Run-6 backlog fixes: qfn auto-detect ranking, bga_fanout None guard."""

import os
import subprocess
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# #718/#522: qfn_fanout.py and bga_fanout.py moved to py_router/ in ee860796,
# which is not an ancestor of this file's branch -- so the joined ROOT path went
# stale on merge and CPython died with "can't open file" on STDERR while these
# asserts read STDOUT. `tool()` raises HERE, naming what it looked for.
from run_utils import tool as _tool, tool_env as _tool_env  # noqa: E402

POUR = os.path.join(ROOT, 'wk', 'run5', 's1_pour.kicad_pcb')


def _run(script, *argv):
    env = _tool_env(dict(os.environ, PYTHONIOENCODING='utf-8',
                         KRT_NO_BANNER='1'))
    return subprocess.run(
        [sys.executable, '-X', 'utf8', _tool(script), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestQfnAutoDetect(unittest.TestCase):
    def test_ranking_prefers_the_real_qfn(self):
        """File order used to pick J1 (a rect-pad USB-C the geometric
        fallback classifies QFN) over the 64-pin U3."""
        if not os.path.exists(POUR):
            self.skipTest('run-5 chain board not present')
        # tempdir, not os.devnull (edgehero, PR #719): the tool writes a
        # board here, and NUL is not a portable destination for one.
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            r = _run('qfn_fanout.py', POUR, '-o',
                     os.path.join(td, 'x.kicad_pcb'),
                     '--clearance', '0.09', '--width', '0.127')
        self.assertIn('Auto-detected QFN/QFP component: U3', r.stdout)


class TestBgaFanoutGuard(unittest.TestCase):
    def test_qfn_ref_does_not_crash(self):
        """bga_fanout -c <QFN>: analyze_bga_grid returns None for perimeter
        packages and the plane-drop pass crashed on grid.pitch_x."""
        if not os.path.exists(POUR):
            self.skipTest('run-5 chain board not present')
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            r = _run('bga_fanout.py', POUR, '-o',
                     os.path.join(td, 'x.kicad_pcb'), '-c', 'U3',
                     '--clearance', '0.09')
            self.assertNotIn('Traceback', r.stderr)
            self.assertNotIn('AttributeError', r.stderr)
            self.assertIn('qfn_fanout.py', r.stdout)


if __name__ == '__main__':
    unittest.main()
