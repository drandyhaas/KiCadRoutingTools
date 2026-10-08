#!/usr/bin/env python3
"""#1108: route_planes refusals exit non-zero.

Run 38: `--nets GND --plane-layers B.Cu In1.Cu` printed "Error: Number of net
arguments (1) must match ..." and exited 0 with no board, so the chain carried
on into a step that then failed on the missing input. The engine's own
refusals (unknown net, not a copper layer, zone conflict, no outline) wrote
nothing and also exited 0 with `status: ok`.
"""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from run_utils import check, evidence  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
SCRIPT = os.path.join(ROOT, 'py_router', 'route_planes.py')


class TestRoutePlanesExit(unittest.TestCase):
    def setUp(self):
        evidence(BOARD, 'splitflap_driver board')
        self.td = tempfile.mkdtemp()

    def test_count_mismatch_is_an_argument_error(self):
        out = os.path.join(self.td, 'a.kicad_pcb')
        check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, out,
               '--nets', 'GND', '--plane-layers', 'B.Cu', 'F.Cu'],
              refuse='must match number of plane layers', code=2,
              allow=('error:',))
        self.assertFalse(os.path.exists(out))

    def test_unknown_net_writes_nothing_and_fails(self):
        out = os.path.join(self.td, 'b.kicad_pcb')
        r = check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, out,
                   '--nets', 'NO_SUCH_NET_1108', '--plane-layers', 'B.Cu'],
                  refuse="Net 'NO_SUCH_NET_1108' not found", code=1)
        self.assertIn('"status": "no_output"', r.stdout)
        self.assertFalse(os.path.exists(out))

    def test_dry_run_still_succeeds(self):
        out = os.path.join(self.td, 'c.kicad_pcb')
        check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, out,
               '--nets', 'GND', '--plane-layers', 'B.Cu', '--dry-run'],
              accept=True)


if __name__ == '__main__':
    unittest.main()
