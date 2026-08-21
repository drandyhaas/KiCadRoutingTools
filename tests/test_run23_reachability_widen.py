"""check_reachability auto-widens NO-TARGET; net_forensics emits JSON.

Run 23, measured: U3.30 (/TXLED, partner D4.1 20.27mm away) returned exit 2
NO-TARGET at the default +/-4mm view; the manual retry ladder cost 128s at
--margin 12, 306s at 18, and a 10-minute timeout WITH NO DATA at 25, because
cells scale as (span/step)^2 at the fixed 0.01 step. The auto-widen locates
the nearest other island by vector union-find, widens the view to hold it,
and coarsens the step so the grid stays ~1200^2 -- one bounded invocation.
And the island gap itself (the number rip-set decisions need) was text-only
in net_forensics; --json makes it consumable.
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')


def _run(tool, *argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', tool), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestAutoWiden(unittest.TestCase):
    def test_u3_30_answers_in_one_bounded_invocation(self):
        t0 = time.time()
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'r.json')
            r = _run('check_reachability.py', PLACED, '--pad', 'U3.30',
                     '--json-out', jp)
            wall = time.time() - t0
            # Exit 0 = PASSABLE, the true verdict run 23 needed three manual
            # retries to reach. Bounded: well under the 10-minute timeout
            # the fixed-step ladder hit (CI slack included).
            self.assertEqual(r.returncode, 0, r.stdout[-500:])
            self.assertLess(wall, 240)
            self.assertIn('auto-widening', r.stdout)
            doc = json.load(open(jp, encoding='utf-8'))
            self.assertEqual(doc['verdict'], 'PASSABLE')
            aw = doc['auto_widened']
            self.assertAlmostEqual(aw['nearest_island_mm'], 20.27, delta=0.1)
            # The coarsened step is DISCLOSED (readings are only comparable
            # at their own step).
            self.assertGreater(aw['step_mm'], 0.01)
            self.assertGreater(doc['margin_um'], 0)

    def test_explicit_view_is_never_overridden(self):
        r = _run('check_reachability.py', PLACED, '--pad', 'U3.30',
                 '--view', '55,58,60,63')
        self.assertNotIn('auto-widening', r.stdout)


class TestForensicsJson(unittest.TestCase):
    def test_gap_is_machine_readable(self):
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'nf.json')
            r = _run('net_forensics.py', PLACED, '--nets', '/TXLED',
                     '--json', jp)
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            doc = json.load(open(jp, encoding='utf-8'))
            net = doc['nets'][0]
            self.assertEqual(net['net'], '/TXLED')
            self.assertEqual(len(net['islands']), 2)
            self.assertAlmostEqual(net['gap']['mm'], 20.267, delta=0.05)


if __name__ == '__main__':
    unittest.main(verbosity=1)
