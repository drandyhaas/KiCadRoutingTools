"""#1401: a part wholly inside another part's CLOSED silk-drawn body gates.

A silk-sourced body never gated (#896), because silk is whatever the library
drew and on a hand-rolled one it is often a corner bracket or an assembly
mark, not a housing. That left a part placed under a housing whose only body
is a closed silk outline (a battery holder, an IC package that draws no .Fab)
reading `buildable (blocking 0)`. The silk exemption now stands only for silk
that does NOT close into a loop; a silk loop that holds another part's pad
copper (>= 99.9%) is a body: the pair gates where a member MOVED vs
--baseline, as courtyard contacts do, and is reported without one.

Fixture: the tracked esp_prog board. U1 (SSOP) draws no courtyard and no .Fab,
only a CLOSED silk rectangle; a 0402 put between its pad rows is under it.
Control: the board as designed stays buildable (its SOT89 draws open corner
brackets, the #896 case).
"""
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence, tool, tool_env  # noqa: E402

BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), 'tracked board')


def run(*argv):
    r = subprocess.run([sys.executable, '-X', 'utf8'] + [str(a) for a in argv],
                       capture_output=True, text=True, env=tool_env(), cwd=ROOT,
                       timeout=600)
    return r.returncode, r.stdout + r.stderr


class TestClosedSilkBody(unittest.TestCase):
    def test_a_part_under_a_closed_silk_body_gates(self):
        with tempfile.TemporaryDirectory() as td:
            moved = os.path.join(td, 'moved.kicad_pcb')
            code, out = run(tool('copy_board.py'), BOARD, os.path.join(td, 'base.kicad_pcb'))
            self.assertEqual(code, 0, out)
            code, out = run(tool('place_pose.py'), os.path.join(td, 'base.kicad_pcb'), moved,
                            'set', 'R1', '125.5', '100.5', '--rot', '0')
            self.assertEqual(code, 0, out[-600:])
            code, out = run(tool('check_assembly.py'), evidence(moved, 'moved board'),
                            '--baseline', os.path.join(td, 'base.kicad_pcb'))
            self.assertIn('NOT BUILDABLE', out, out[-800:])
            self.assertIn('CONTAINED 100%', out)
            self.assertIn('GATES (moved: R1)', out)
            # #1465: the headline counts the pair and the body-coverage text
            # names the gate, so the report agrees with its own verdict
            self.assertIn('silk_enclosed 1', out)
            self.assertIn('wholly inside a CLOSED silk loop, which gates', out)
            # moved again but already inside on its baseline: the baseline's own
            moved2 = os.path.join(td, 'moved2.kicad_pcb')
            code, out = run(tool('place_pose.py'), moved, moved2,
                            'set', 'R1', '125.6', '100.5', '--rot', '0')
            self.assertEqual(code, 0, out[-600:])
            code, out = run(tool('check_assembly.py'), evidence(moved2, 'moved again'),
                            '--baseline', moved)
            self.assertIn("baseline's own (the baseline had it)", out, out[-800:])
            self.assertIn('buildable (blocking 0)', out, out[-800:])
            # without a baseline nothing says who moved: reported, not gated
            code, out = run(tool('check_assembly.py'), moved)
            self.assertIn('INSIDE A CLOSED SILK BODY (1): REPORT-ONLY', out, out[-800:])
            self.assertIn('buildable (blocking 0)', out, out[-800:])

    def test_open_silk_brackets_still_do_not_gate(self):
        code, out = run(tool('check_assembly.py'), BOARD)
        self.assertIn('buildable (blocking 0)', out, out[-800:])


if __name__ == '__main__':
    unittest.main()
