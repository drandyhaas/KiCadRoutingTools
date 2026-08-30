#!/usr/bin/env python3
"""What a lap has to record so a later reader can act on it.

`converge record` is the loop's ledger. Three things a lap needs and did not
have:

  * the MEASUREMENT it was aimed at. Run 20 recorded "throat 0.409mm vs 0.450
    needed, blocked by U4.53/R7.2" as English inside `lever`. A person could
    read it; nothing else could.
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


class TestDefectRecord(unittest.TestCase):
    def test_the_contents_are_inlined_not_the_path(self):
        """A path into a work dir is not a measurement.

        Work dirs are deleted; the ledger outlives them.
        """
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            dj = os.path.join(td, 'defect.json')
            with open(dj, 'w', encoding='utf-8') as f:
                json.dump({'ref': 'U4', 'throat_mm': 0.409,
                           'needed_mm': 0.450}, f)
            r = _record(ledger, score={'blocking': 4, 'quality': {}},
                        kind='placement', lever='reseat U4', defect_json=dj)
            self.assertEqual(r.returncode, 0, r.stderr[-400:])
            row = _rows(ledger)[0]
            self.assertEqual(row['defects'][0]['throat_mm'], 0.409)
            self.assertEqual(row['defects'][0]['needed_mm'], 0.450)

    def test_it_is_repeatable(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            argv = [CONVERGE, 'record', '--ledger', ledger, '--board', BOARD,
                    '--kind', 'placement', '--lever', 'two defects']
            for i, ref in enumerate(('U4', 'R7')):
                p = os.path.join(td, 'd%d.json' % i)
                with open(p, 'w', encoding='utf-8') as f:
                    json.dump({'ref': ref}, f)
                argv += ['--defect-json', p]
            self.assertEqual(_run(*argv).returncode, 0)
            self.assertEqual([d['ref'] for d in _rows(ledger)[0]['defects']],
                             ['U4', 'R7'])

    def test_an_unreadable_record_is_kept_as_an_error_not_dropped(self):
        """Naming a record we cannot read is not the same as naming none.

        Collapsing the two silently is how a ledger stops being evidence.
        """
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            bad = os.path.join(td, 'bad.json')
            with open(bad, 'w', encoding='utf-8') as f:
                f.write('{not json')
            self.assertEqual(
                _record(ledger, kind='placement', lever='x',
                        defect_json=bad).returncode, 0)
            d = _rows(ledger)[0]['defects'][0]
            self.assertEqual(d['path'], bad)
            self.assertIn('error', d)

    def test_no_flag_leaves_the_key_null(self):
        """The absence has to be readable too, not an absent key."""
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            self.assertEqual(
                _record(ledger, kind='placement', lever='x').returncode, 0)
            self.assertIsNone(_rows(ledger)[0]['defects'])


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
            r = _record(ledger, kind='classification',
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


class TestStatusIsStillOneDocument(unittest.TestCase):
    """`status` stdout is a JSON API; callers json.loads the whole stream."""

    def test_status_parses_as_one_json_doc(self):
        with tempfile.TemporaryDirectory() as td:
            ledger = os.path.join(td, 'ledger.jsonl')
            _record(ledger, score={'blocking': 2, 'quality': {}},
                    kind='classification', lever='lap')
            r = _run(CONVERGE, 'status', '--ledger', ledger)
            self.assertEqual(r.returncode, 0, r.stderr[-300:])
            doc = json.loads(r.stdout.split('\nNOTE:')[0])
            self.assertEqual(doc['total'], 1)


if __name__ == '__main__':
    unittest.main(verbosity=1)
