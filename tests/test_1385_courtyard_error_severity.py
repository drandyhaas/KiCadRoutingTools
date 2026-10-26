"""#1385: on a project that declares `courtyards_overlap` an ERROR, a courtyard
pair a run's own move created gates whatever its size.

The run-23 floors (0.5 mm2 or a quarter of the smaller courtyard, AND 0.3 mm
deep) keep a by-design sliver advisory so a pristine board does not flip. They
did it on an error project too, where kicad-cli reports the sliver the run
made: `place_pose --near` put C17 on Y2's courtyard (0.046 mm2) and
check_assembly said `buildable`. Now, with the project severity `error`:

- `grade_body_overlap` lists every real unwaived courtyard pair
  (`courtyard_error_pairs`), deeper than the round-off of two touching
  polygons, whose parts both draw a courtyard;
- check_assembly --baseline gates the ones a MOVED member created and not the
  baseline's own;
- `place_pose` grades the same pairs: a pose that makes a new one is refused.

Any other severity (warning, unset) keeps the floors, as before.
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
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from test_1095_project_severity import board, grade  # noqa: E402
from run_utils import evidence, tool, tool_env  # noqa: E402

#: 2 mm courtyards 2.1 mm apart (clear) and 1.95 mm apart (0.05 mm deep, 0.1 mm2:
#: far under both run-23 floors).
CLEAR = [('A', 10, 1.0, 0.4), ('B', 12.1, 1.0, 0.4)]
SLIVER = [('A', 10, 1.0, 0.4), ('B', 11.95, 1.0, 0.4)]


def assembly(path, baseline):
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('check_assembly.py'), path,
                        '--baseline', baseline], capture_output=True, text=True,
                       env=tool_env(), cwd=ROOT, timeout=600)
    return r.stdout + r.stderr


class TestCourtyardErrorSeverity(unittest.TestCase):
    def test_error_project_lists_the_sliver_and_the_floors_still_pass_it(self):
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, SLIVER, 'error'))
        self.assertEqual([(q.a, q.b) for q in g['courtyard_error_pairs']],
                         [('A', 'B')])
        self.assertEqual(g['courtyard_blocking_pairs'], [])

    def test_other_severities_list_none(self):
        for sev in ('warning', None, 'ignore'):
            with tempfile.TemporaryDirectory() as td:
                g = grade(board(td, SLIVER, sev))
            self.assertEqual(g['courtyard_error_pairs'], [], sev)

    def test_a_graze_under_the_polygon_noise_is_not_a_finding(self):
        touching = [('A', 10, 1.0, 0.4), ('B', 11.995, 1.0, 0.4)]
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, touching, 'error'))
        self.assertEqual(g['courtyard_error_pairs'], [])

    def test_gating_is_the_moved_pairs_the_baseline_lacked(self):
        from placement import legality
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, SLIVER, 'error'))
        pairs = g['courtyard_error_pairs']
        self.assertEqual(legality.courtyard_error_gating(pairs, {'B'}), pairs)
        self.assertEqual(legality.courtyard_error_gating(pairs, {'Z'}), [])
        self.assertEqual(legality.courtyard_error_gating(pairs, {'B'}, {('A', 'B')}), [])

    def test_check_assembly_gates_the_created_sliver_on_an_error_project(self):
        for sev, gates in (('error', True), ('warning', False), (None, False)):
            with tempfile.TemporaryDirectory() as td:
                base = os.path.join(td, 'base')
                now = os.path.join(td, 'now')
                os.mkdir(base)
                os.mkdir(now)
                b = evidence(board(base, CLEAR, sev), 'baseline board')
                n = evidence(board(now, SLIVER, sev), 'moved board')
                out = assembly(n, b)
            self.assertIn('VERDICT', out, out[-400:])
            self.assertEqual('NOT BUILDABLE' in out, gates, '%s: %s' % (sev, out[-600:]))
            self.assertEqual('COURTYARD ERRORS' in out, gates, sev)

    def test_the_baselines_own_sliver_does_not_gate(self):
        with tempfile.TemporaryDirectory() as td:
            base = os.path.join(td, 'base')
            now = os.path.join(td, 'now')
            os.mkdir(base)
            os.mkdir(now)
            b = evidence(board(base, SLIVER, 'error'), 'baseline board')
            n = evidence(board(now, SLIVER, 'error'), 'same board')
            out = assembly(n, b)
        self.assertNotIn('COURTYARD ERRORS', out)
        self.assertNotIn('NOT BUILDABLE', out)

    def test_place_pose_refuses_a_pose_that_makes_a_new_pair(self):
        from kicad_parser import parse_kicad_pcb
        from placement import pose_ops

        def graded(path):
            return pose_ops.grade(parse_kicad_pcb(path), path, 0.1)
        with tempfile.TemporaryDirectory() as td:
            before = graded(board(td, CLEAR, 'error'))
        with tempfile.TemporaryDirectory() as td:
            after = graded(board(td, SLIVER, 'error'))
        self.assertEqual(before['courtyard_error_pairs'], [])
        self.assertEqual(after['courtyard_error_pairs'], [['A', 'B']])
        self.assertIn('courtyard_error_pairs', pose_ops.worsened(before, after))
        self.assertEqual(pose_ops.worsened(after, after), [])

    def test_place_pose_honours_the_intents_courtyard_waivers(self):
        # check_assembly hands the intent's waivers to its courtyard grade, so
        # place_pose must too: a waived pair is not an error pair.
        from kicad_parser import parse_kicad_pcb
        from placement import pose_ops
        with tempfile.TemporaryDirectory() as td:
            path = board(td, SLIVER, 'error')
            pcb = parse_kicad_pcb(path)
            for waivers in ((), ()), ((('A', 'B'),), ()), ((), (('B', 'A'),)):
                got = pose_ops.grade(pcb, path, 0.1, waivers=waivers)
                self.assertEqual(got['courtyard_error_pairs'],
                                 [] if any(waivers) else [['A', 'B']], waivers)


if __name__ == '__main__':
    unittest.main()
