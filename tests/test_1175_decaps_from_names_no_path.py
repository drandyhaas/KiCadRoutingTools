#!/usr/bin/env python3
"""#1175: an intent emitted with `--decaps-from` names the reference, not its path.

The basis keys always held only the reference's basename
(`context.basis['decaps.max_distance_mm'] = 'reference:ref.kicad_pcb'`), but
`context.decap_census.reference_board` held the path exactly as given --
absolute when it was passed absolute -- and a refused reference repeated it in
`context.budget_withheld['decaps.max_distance_mm']` and
`decap_census.pin_limit_withheld`. In the flow the plan-pcb-placement skill
prescribes, the reference is a human layout, so the run then graded against a
document naming the file it must not read (fa10 post-processed every intent
for it).

Now the census records the basename and a `reference_board_sha256` of the
file's bytes (provenance without a pointer), and both withheld strings name
the basename. Pinned on an ABSOLUTE reference path in a directory whose name
must not appear anywhere in the document, for an accepted reference (through
the CLI, reading the file it writes) and for a refused one.

Run: python3 -X utf8 tests/test_1175_decaps_from_names_no_path.py
"""
import contextlib
import hashlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import floorplan as fp                          # noqa: E402
from run_utils import check, evidence                          # noqa: E402

PY = sys.executable
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
OTHER = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
#: A directory name that must not leak: it stands for the truth directory.
SECRET = 't1175_TRUTH_DIR_'


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _sha(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


class TheReferenceIsNamedNotPointedAt(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.td = os.path.abspath(tempfile.mkdtemp(prefix=SECRET))
        cls.ref = os.path.join(cls.td, 'ref.kicad_pcb')
        check([PY, '-X', 'utf8', os.path.join(ROOT, 'py_router',
                                              'copy_board.py'),
               BOARD, cls.ref], accept=True)
        evidence(cls.ref, 'the copied reference')
        cls.other = os.path.join(cls.td, 'other.kicad_pcb')
        check([PY, '-X', 'utf8', os.path.join(ROOT, 'py_router',
                                              'copy_board.py'),
               OTHER, cls.other], accept=True)
        evidence(cls.other, 'the other design')
        assert os.path.isabs(cls.ref)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.td, ignore_errors=True)

    def _no_path(self, text):
        self.assertNotIn(SECRET, text)
        self.assertNotIn(self.td, text)
        self.assertNotIn(self.td.replace(os.sep, '/'), text)

    def test_an_accepted_reference_through_the_cli(self):
        out = os.path.join(self.td, 'i.json')
        r = check([PY, '-X', 'utf8',
                   os.path.join(ROOT, 'py_tools', 'check_floorplan.py'),
                   self.ref, '--emit-intent', out,
                   '--decaps-from', self.ref], accept=True)
        with open(evidence(out, 'the emitted intent'),
                  encoding='utf-8') as fh:
            text = fh.read()
        self._no_path(text)
        doc = json.loads(text)
        cen = doc['context']['decap_census']
        self.assertIn('max_distance_mm', doc['decaps'])
        self.assertEqual(cen['reference_board'], 'ref.kicad_pcb')
        self.assertEqual(cen['reference_board_sha256'], _sha(self.ref))
        self.assertEqual(cen['decaps_basis'], 'reference:ref.kicad_pcb')
        # The console says the same, so a log is no leak either.
        self.assertIn('on the reference ref.kicad_pcb', r.stdout)

    def test_a_refused_reference_names_its_basename(self):
        """Another design is refused, in budget_withheld and in
        pin_limit_withheld -- both of which repeated the path."""
        doc = _quiet(fp.emit_intent, _quiet(parse_kicad_pcb, BOARD), BOARD,
                     decaps_from=self.other)
        self._no_path(json.dumps(doc))
        why = doc['context']['budget_withheld']['decaps.max_distance_mm']
        self.assertTrue(why.startswith('the reference board other.kicad_pcb: '
                                       'it is not a placement of this design'),
                        why)
        cen = doc['context']['decap_census']
        self.assertTrue(cen['pin_limit_withheld'].startswith(
            'the reference board other.kicad_pcb: '), cen['pin_limit_withheld'])
        self.assertEqual(cen['reference_board'], 'other.kicad_pcb')
        self.assertEqual(cen['reference_board_sha256'], _sha(self.other))


if __name__ == '__main__':
    unittest.main()
