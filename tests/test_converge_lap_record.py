#!/usr/bin/env python3
"""What a lap has to record so a later reader can act on it.

`converge record` is the loop's ledger. Two things a lap needs and did not
have:

  * the re-entry SHAPE it acted on, the same word the loop's own gate demands,
    so "which shape did we re-enter at, and did it work" becomes a question
    the record can answer.
  * a `kind` for a lap that only DECIDES where to re-enter. Filed as
    `systemic` it looked like a tool change; filed as `placement` or
    `completion` it would make that half look like it was still improving,
    because the plateau test compares consecutive laps of a half.

    python3 -X utf8 tests/test_converge_lap_record.py
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

CONVERGE = os.path.join(ROOT, 'py_placer', 'converge.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _run(*argv):
    env = dict(os.environ, PYTHONPATH=ROOT, PYTHONIOENCODING='utf-8',
               KRT_NO_BANNER='1')
    return subprocess.run([sys.executable, '-X', 'utf8', *argv],
                          capture_output=True, text=True, env=env, cwd=ROOT)


def _record(ledger, score=None, **flags):
    argv = [CONVERGE, 'record', '--ledger', ledger, '--board', BOARD]
    for k, v in flags.items():
        argv += ['--' + k.replace('_', '-')] + ([] if v is True else [str(v)])
    if score is not None:
        sp = os.path.join(os.path.dirname(ledger), 'score.json')
        with open(sp, 'w', encoding='utf-8') as f:
            json.dump(score, f)
        argv += ['--score-file', sp]
    return _run(*argv)


def _rows(ledger):
    with open(ledger, encoding='utf-8') as f:
        return [json.loads(ln) for ln in f if ln.strip()]


class TestShape(unittest.TestCase):
    def test_the_shape_lands_in_the_row(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            self.assertEqual(
                _record(ledger, kind='placement', lever='reseat U4',
                        shape='placement').returncode, 0)
            self.assertEqual(_rows(ledger)[0]['shape'], 'placement')

    def test_only_the_three_shapes_the_gate_names_are_accepted(self):
        """A free-text shape is a paragraph again, which is the defect."""
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            r = _record(ledger, kind='placement', lever='x', shape='vibes')
            self.assertEqual(r.returncode, 2, r.stdout[-200:])
            self.assertFalse(os.path.exists(ledger), 'it wrote a bad row')


class TestClassificationKind(unittest.TestCase):
    def test_it_is_accepted_and_recorded(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            r = _record(ledger, kind='classification', shape='floorplan',
                        lever='the residual is a floorplan finding')
            self.assertEqual(r.returncode, 0, r.stderr[-400:])
            self.assertEqual(_rows(ledger)[0]['kind'], 'classification')

    def test_it_belongs_to_neither_half(self):
        """`_HALF` is what the plateau test buckets a lap by.

        A classification lap changes no board, so it must not be able to make
        a half look like it is still improving.
        """
        import importlib.util
        sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
        spec = importlib.util.spec_from_file_location('_cv', CONVERGE)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self.assertNotIn('classification', mod._HALF)
        self.assertNotIn('systemic', mod._HALF)
        # ...and the two that DO change a board are still there, or the
        # assertion above would pass for the wrong reason.
        self.assertEqual(mod._HALF,
                         {'placement': 'placement', 'completion': 'routing'})


class TestRecordIsStillOneDocument(unittest.TestCase):
    """`record` stdout is a JSON API; callers json.loads the whole stream."""

    def test_record_parses_as_one_json_doc(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            r = _record(ledger, score={'blocking': 2, 'quality': {}},
                        kind='classification', shape='parameter', lever='lap')
            self.assertEqual(r.returncode, 0, r.stderr[-300:])
            self.assertEqual(json.loads(r.stdout)['kind'], 'classification')


if __name__ == '__main__':
    unittest.main(verbosity=1)
