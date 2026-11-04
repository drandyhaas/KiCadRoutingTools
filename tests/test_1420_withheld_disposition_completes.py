"""#1420: a withheld budget key the plan ANSWERS counts as answered.

An intent emitted from a pile withholds `overlap_area` (a pile's overlap is
staging, not a baseline) in `context.budget_withheld`, so every later grade
reported `budget_abstained 1`, `pass: false` and exit 4 whatever the placement.
Writing why in `dispositions.withheld.<key>` -- the remedy the roster prints --
changed the roster to `[dispositioned]` and nothing else. A key answered that
way is now acknowledged: it leaves `budget_abstained` and `not_graded`, rides in
`budget_dispositioned`, and the grade is complete.

Fixture: the tracked esp_prog pile, an intent emitted from it, graded against
the tracked esp_prog board.
"""
import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import floorplan                                # noqa: E402
from run_utils import evidence, tool, tool_env                 # noqa: E402

PILE = evidence(os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb'), 'pile')
BOARD = evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), 'tracked board')


def graded(raw):
    return floorplan.grade(floorplan.intent_from_dict(raw, source_path='emitted.json'),
                           parse_kicad_pcb(BOARD), BOARD)


class TestWithheldDisposition(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import contextlib
        import io
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            cls.raw = floorplan.emit_intent(parse_kicad_pcb(PILE), PILE)

    def test_an_unanswered_withheld_key_is_an_incomplete_grade(self):
        r = graded(json.loads(json.dumps(self.raw)))
        self.assertIn('overlap_area', r.budget_abstained)
        self.assertEqual(r.not_graded.get('budget_abstained'), len(r.budget_abstained))
        self.assertFalse(r.complete)

    def test_an_answered_key_is_acknowledged(self):
        raw = json.loads(json.dumps(self.raw))
        withheld = sorted(raw['context']['budget_withheld'])
        raw['dispositions'] = {'withheld': {k: 'a pile has no baseline to derive it from'
                                            for k in withheld}}
        r = graded(raw)
        self.assertEqual(r.budget_abstained, {})
        self.assertEqual(sorted(r.budget_dispositioned), withheld)
        self.assertNotIn('budget_abstained', r.not_graded)
        self.assertTrue(r.complete)
        doc = floorplan.to_json(r)
        self.assertEqual(sorted(doc['budget_dispositioned']), withheld)

    def test_one_unanswered_key_still_leaves_it_incomplete(self):
        raw = json.loads(json.dumps(self.raw))
        withheld = sorted(raw['context']['budget_withheld'])
        if len(withheld) < 2:
            raw['context']['budget_withheld']['oob_count'] = 'withheld for the test'
            withheld = sorted(raw['context']['budget_withheld'])
        raw['dispositions'] = {'withheld': {withheld[0]: 'answered'}}
        r = graded(raw)
        self.assertEqual(sorted(r.budget_abstained), withheld[1:])
        self.assertFalse(r.complete)

    def test_the_cli_exits_zero_once_answered(self):
        raw = json.loads(json.dumps(self.raw))
        raw['dispositions'] = {'withheld': {k: 'a pile has no baseline'
                                            for k in raw['context']['budget_withheld']}}
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, 'intent.json')
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(raw, fh)
            r = subprocess.run([sys.executable, '-X', 'utf8', tool('check_floorplan.py'), BOARD,
                                '--intent', path], capture_output=True, text=True,
                               env=tool_env(), cwd=ROOT, timeout=600)
        self.assertNotIn('NOT FULLY GRADED', r.stdout + r.stderr)
        self.assertEqual(r.returncode, 0, (r.stdout + r.stderr)[-800:])


if __name__ == '__main__':
    unittest.main()
