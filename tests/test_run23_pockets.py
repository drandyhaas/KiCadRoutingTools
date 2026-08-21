"""check_pockets: the aggregate handoff census per-net gates cannot give.

Run 23: check_channels said 0 starved faces, check_reachability said every
failing pad PASSABLE, crossings sat below the damaged baseline -- and two
nets then died across ~20 route laps in one pocket (RN7/U6/J4/RN8, where
committed copper wrapped RN7 at 0.095mm against the 0.45mm lane need). The
census here must NAME that geography pre-route, and must stay REPORT-ONLY
(exit 0): an absolute threshold on a young metric is how phantom gates get
born.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run23',
                      'tigard_placed.kicad_pcb')
SPLITFLAP = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _run(*argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_pockets.py'), *argv],
        capture_output=True, text=True, env=env, cwd=ROOT)


class TestPocketCensus(unittest.TestCase):
    def test_names_the_run23_pocket(self):
        with tempfile.TemporaryDirectory() as td:
            jp = os.path.join(td, 'p.json')
            r = _run(PLACED, '--json', jp, '--top', '8')
            self.assertEqual(r.returncode, 0, r.stdout[-400:])
            # The failure geography by name: the top windows must carry the
            # nets that ate run 23's endgame and the parts that cage them.
            self.assertIn('/ICE_CDONE', r.stdout)
            self.assertIn('RN8', r.stdout)
            self.assertIn('U6', r.stdout)
            # The lane arithmetic the forensics measured the cage against.
            self.assertIn('0.45', r.stdout)
            doc = json.load(open(jp, encoding='utf-8'))
            self.assertEqual(doc['lane_mm'], 0.45)
            self.assertGreater(len(doc['windows']), 10)

    def test_report_only_and_multi_net_ranking(self):
        r = _run(PLACED, '--top', '5')
        self.assertEqual(r.returncode, 0)
        self.assertIn('REPORT-ONLY', r.stdout)
        # Every RANKED row is a >=2-net window -- a single-net sliver inside
        # one part's own pad field means nothing for simultaneous
        # routability and must not top the list.
        for ln in r.stdout.splitlines():
            ln = ln.strip()
            if ln.startswith('[') and 'demand' in ln:
                n = int(ln.split('demand')[1].split('net(s)')[0].strip())
                self.assertGreaterEqual(n, 2, ln)

    def test_exit_zero_on_a_healthy_board(self):
        r = _run(SPLITFLAP, '--top', '3')
        self.assertEqual(r.returncode, 0, r.stdout[-400:])


if __name__ == '__main__':
    unittest.main(verbosity=1)
