"""#1112: a route step with nothing to route still runs the plane finalize.

On a poured board, "All nets are already fully connected" is the router's
FILL MODEL talking. The plane finalize is the pass that checks that verdict
against KiCad's exact fill (its oracle leg is ungated on purpose), and the
early return used to skip it -- so a poured net only the exact fill sees split
shipped split, and the only tool left was a standalone repair_planes run at
the board's net-class via and track instead of the route step's sizes.

The chain here is the pours-first shape: pour GND, route '*', then route '*'
AGAIN on the finished board. The second step has nothing to route and must
still reach the finalize. Two controls keep the old early return where the
finalize would not run anyway: a scope holding no zone net, and the
KICAD_PLANE_FINALIZE=0 kill switch.

The "Plane finalize (#562)" line prints before the oracle leg, so this holds
on an image without kicad-cli too.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from run_utils import evidence  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
EARLY = 'All nets are already fully connected - nothing to route!'
CONTINUE = 'continuing to the plane finalize'
FINALIZE = 'Plane finalize (#562)'


def _run(script, *args, env_extra=None):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1', MSYS2_ARG_CONV_EXCL='*')
    env.update(env_extra or {})
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', script),
         *args],
        capture_output=True, text=True, env=env, cwd=ROOT)
    assert r.returncode == 0, f'{script} rc={r.returncode}\n{r.stdout[-1500:]}\n{r.stderr[-800:]}'
    return r.stdout


def _min_lines(log):
    from route_summary import SUMMARY_MIN_RE
    return [json.loads(m) for m in SUMMARY_MIN_RE.findall(log)]


class TestNoOpRouteStillFinalizes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.td = tempfile.TemporaryDirectory()
        d = cls.td.name
        cls.poured = os.path.join(d, 'poured.kicad_pcb')
        cls.routed = os.path.join(d, 'routed.kicad_pcb')
        _run('route_planes.py', BOARD, cls.poured,
             '--nets', 'GND', '--plane-layers', 'B.Cu')
        evidence(cls.poured, 'the poured board')
        _run('route.py', cls.poured, cls.routed, '--nets', '*')
        evidence(cls.routed, 'the routed board')
        cls.d = d

    @classmethod
    def tearDownClass(cls):
        cls.td.cleanup()

    def _again(self, name, *args, env_extra=None):
        return _run('route.py', self.routed,
                    os.path.join(self.d, name + '.kicad_pcb'), *args,
                    env_extra=env_extra)

    def test_nothing_to_route_continues_to_the_finalize(self):
        log = self._again('again', '--nets', '*')
        # The precondition, asserted rather than assumed: the step really had
        # nothing to route. Without it a pass would prove nothing.
        self.assertIn('Skipping 83 already-routed net(s)', log)
        self.assertIn(CONTINUE, log)
        self.assertNotIn(EARLY, log)
        self.assertIn(FINALIZE, log,
                      'a no-op route step on a poured board must reach the '
                      'plane finalize (#1112)')
        mins = _min_lines(log)
        self.assertEqual(len(mins), 1, f'{len(mins)} MIN lines')
        self.assertEqual(mins[0]['routed'], 0)
        self.assertEqual(mins[0]['failed'], 0)

    def test_scope_without_a_zone_net_keeps_the_early_return(self):
        # GND is the only poured net; a step scoped away from it has nothing
        # for the finalize to do, so the cheap early return stays.
        log = self._again('scoped', '--nets', '/CLOCK_IN')
        self.assertIn(EARLY, log)
        self.assertNotIn(FINALIZE, log)
        mins = _min_lines(log)
        self.assertEqual([m.get('status') for m in mins], ['already_connected'])

    def test_kill_switch_keeps_the_early_return(self):
        log = self._again('killed', '--nets', '*',
                          env_extra={'KICAD_PLANE_FINALIZE': '0'})
        self.assertIn(EARLY, log)
        self.assertNotIn(FINALIZE, log)


if __name__ == '__main__':
    unittest.main()
