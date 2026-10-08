"""#1183: board_score --baseline arms check_assembly's courtyard gate.

check_assembly gates a courtyard pair only when a member MOVED against a
baseline (its fifth conjunct). board_score had `--baseline` since #962 and
handed it to check_drc alone, so `board_score --baseline <input>` still
published `courtyard_gating_armed: false` with a reason saying no baseline
was passed -- and One-Air-Max's seed scored `buildable` while check_assembly
--baseline gated 22 of 22 pairs. The unarmed conjunct was also absent from
the top-level `ungraded`, the list a loop compares (#964 item 2).

What must hold:

  * board_score forwards --baseline to check_assembly: the gate is armed
    and gates (tigard's run-23 fixtures: a damaged board against its placed
    one);
  * unarmed, 'assembly.courtyard_gating' is in `ungraded`, and the reason
    says what arms it;
  * an unreadable baseline is refused (exit 2), never silently unarmed;
  * check_complete --baseline reaches board_score;
  * converge calls an armed and an unarmed score incommensurable, naming
    --baseline.
"""

import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                              # noqa: E402

RUN_ALL_TIMEOUT = 1800
FIX = os.path.join(ROOT, 'tests', 'fixtures', 'run23')
DAMAGED = os.path.join(FIX, 'tigard_damaged.kicad_pcb')
PLACED = os.path.join(FIX, 'tigard_placed.kicad_pcb')


def _run(argv, out):
    """Run a grader that exits 4 on a NOT-BUILDABLE board; the evidence is
    its JSON, which must exist and parse."""
    import subprocess
    r = subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=1500)
    assert r.returncode in (0, 4), (r.returncode, r.stdout[-800:],
                                    r.stderr[-800:])
    run_utils.evidence(out, 'grader json')
    with open(out, encoding='utf-8') as fh:
        return r, json.load(fh)


def _score(td, *extra):
    out = os.path.join(td, 'score_%d.json' % len(extra))
    return _run([os.path.join(ROOT, 'py_tools', 'board_score.py'), DAMAGED,
                 '--json', out, '--quiet'] + list(extra), out)


class BoardScore(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        run_utils.evidence(DAMAGED, 'fixture')
        run_utils.evidence(PLACED, 'fixture')
        cls._td = tempfile.TemporaryDirectory()
        cls.armed = _score(cls._td.name, '--baseline', PLACED)[1]
        cls.unarmed = _score(cls._td.name)[1]

    @classmethod
    def tearDownClass(cls):
        cls._td.cleanup()

    def test_the_baseline_arms_the_gate(self):
        a = self.armed['components']['assembly']
        self.assertTrue(a['courtyard_gating_armed'], a)
        self.assertIsNone(a['courtyard_gating_reason'])
        self.assertGreater(a['conjuncts']['courtyard_blocking_gating'], 0, a)
        self.assertFalse(a['buildable'], a)
        self.assertIn('courtyard_blocking_gating', a['conjuncts_fired'])
        self.assertNotIn('assembly.courtyard_gating', self.armed['ungraded'])

    def test_unarmed_it_is_ungraded_and_says_what_arms_it(self):
        a = self.unarmed['components']['assembly']
        self.assertFalse(a['courtyard_gating_armed'], a)
        self.assertIn('--baseline', a['courtyard_gating_reason'])
        self.assertIn('assembly.courtyard_gating', self.unarmed['ungraded'])

    def test_converge_calls_them_incommensurable(self):
        import converge
        got = converge.commensurability(self.unarmed, self.armed)
        self.assertIsNotNone(got)
        differ, why, _false_improvement = got
        self.assertIn('assembly.courtyard_gating', differ)
        self.assertIn('--baseline', why)


class Refusals(unittest.TestCase):

    def test_an_unreadable_baseline_is_refused(self):
        with tempfile.TemporaryDirectory() as td:
            run_utils.check([sys.executable, '-X', 'utf8',
                             os.path.join(ROOT, 'py_tools', 'board_score.py'),
                             DAMAGED, '--baseline',
                             os.path.join(td, 'nope.kicad_pcb')],
                            refuse='baseline not found', code=2)


class CheckComplete(unittest.TestCase):

    def test_the_baseline_reaches_board_score(self):
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, 'cc.json')
            _r, doc = _run([os.path.join(ROOT, 'check_complete.py'), DAMAGED,
                            '--baseline', PLACED, '--skip-slow', '--json',
                            out], out)
        a = doc['score']['components']['assembly']
        self.assertTrue(a['courtyard_gating_armed'], a)


if __name__ == '__main__':
    unittest.main()
