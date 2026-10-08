"""#1095: the courtyard channel is graded at the board's own severity.

KiCad's StickHub demo sets `courtyards_overlap` to `ignore` in its project
(the JST ports J2-J8 and their parts are packed on purpose) and KiCad's DRC
reports 0. check_assembly read no severity and graded 19 of those pairs
COURTYARD BLOCKING. It now reads the project the way check_drc reads
`copper_edge_clearance` (#427), through one reader, `check_drc.rule_severity`:

- ignore: every courtyard pair the intent does not already waive is
  labelled `project_severity_ignore` and never gates -- KiCad runs no
  courtyard check then;
- warning / error / unset: graded as before. KiCad still REPORTS a warning,
  and `fix_kicad_drc_settings --relax-severities` writes that demotion on
  the promise that check_assembly stays the arbiter;
- the fab CONTAINMENT channel is not KiCad's courtyard rule and is graded
  as before whatever the severity says.
"""

import json
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

from test_1094_rotated_courtyards import stickhub  # noqa: E402
from placement.legality import LEGACY_SEVERITY_PLAN_IGNORES  # noqa: E402

PART = (
    '  (footprint "t:{ref}" (layer "F.Cu") (at {x} 10)\n'
    '    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
    '    (fp_rect (start -{c} -{c}) (end {c} {c}) (stroke (width 0.05)'
    ' (type default)) (layer "F.CrtYd"))\n'
    '    (fp_rect (start -{f} -{f}) (end {f} {f}) (stroke (width 0.05)'
    ' (type default)) (layer "F.Fab"))\n'
    '    (pad "1" smd rect (at 0 0) (size 0.3 0.3) (layers "F.Cu")'
    ' (net {n} "N{n}")))\n')


def board(td, parts, severity=None, extra_sev=None, saved=None,
          locked=()):
    """`parts` = [(ref, x, courtyard half, fab half)]; a .kicad_pro beside
    it carries `severity` for courtyards_overlap (None: key absent), plus
    `extra_sev` severities and a `saved_severities` record."""
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal)'
            ' (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "N1") (net 2 "N2")\n'
            '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1)'
            ' (type default)) (layer "Edge.Cuts"))\n'
            + ''.join(PART.format(ref=r, x=x, c=c, f=f, n=i + 1)
                      .replace('(layer "F.Cu") (at',
                               '(layer "F.Cu") (locked yes) (at'
                               if r in locked else '(layer "F.Cu") (at')
                      for i, (r, x, c, f) in enumerate(parts)) + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    sev = {} if severity is None else {'courtyards_overlap': severity}
    sev.update(extra_sev or {})
    doc = {'board': {'design_settings': {'rule_severities': sev}}}
    if saved is not None:
        doc['kicad_routing_tools'] = {
            'saved_severities': {'courtyards_overlap': saved}}
    with open(os.path.join(td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return path


def grade(path, **kw):
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_body_overlap
    return grade_body_overlap(parse_kicad_pcb(path), 0.1, pcb_file=path, **kw)


#: A deep orthogonal courtyard overlap (1 x 2 mm, depth 1) with fab bodies
#: clear of each other: courtyard-blocking at error.
DEEP = [('A', 10, 1.0, 0.4), ('B', 11, 1.0, 0.4)]


class TestSynthetic(unittest.TestCase):
    def _blocking(self, severity, **kw):
        with tempfile.TemporaryDirectory() as td:
            return grade(board(td, DEEP, severity), **kw)

    def test_error_warning_and_unset_gate(self):
        for sev in ('error', 'warning', None):
            g = self._blocking(sev)
            self.assertEqual([(q.a, q.b) for q in
                              g['courtyard_blocking_pairs']], [('A', 'B')],
                             sev)
            self.assertEqual(g['courtyard_severity_waiver'], '')

    def test_ignore_waives_by_name(self):
        for sev in ('ignore',):
            g = self._blocking(sev)
            self.assertEqual(g['courtyard_blocking_pairs'], [], sev)
            self.assertEqual(g['courtyard_severity'], sev)
            cy = [q for q in g['pairs'] if q.kind == 'courtyard']
            self.assertEqual([q.waiver for q in cy],
                             ['project_severity_' + sev])
            self.assertTrue(cy[0].waived)

    def test_the_off_arm_grades_at_error(self):
        g = self._blocking('ignore', courtyard_severity=None)
        self.assertEqual(len(g['courtyard_blocking_pairs']), 1)

    def test_an_authored_waiver_keeps_its_label(self):
        g = self._blocking('ignore', intent_waivers=[('A', 'B')])
        cy = [q for q in g['pairs'] if q.kind == 'courtyard']
        self.assertEqual([q.waiver for q in cy], ['intent_declared'])

    def test_ignore_also_covers_a_locked_part(self):
        """KiCad's own DRC skips the rule for locked parts too, so the
        project waiver outranks run-8's no-class-waiver-on-locked rule."""
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, DEEP, 'ignore', locked=('A',)))
        self.assertEqual(g['courtyard_blocking_pairs'], [])
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, DEEP, 'error', locked=('A',)))
        self.assertEqual(len(g['courtyard_blocking_pairs']), 1)

    def test_a_legacy_tool_written_ignore_is_graded_at_error(self):
        """The pre-#856 severity plan ignored courtyards_overlap together
        with every other category it managed; such a project's ignore is
        this repo's, not the author's."""
        from fix_kicad_drc_settings import (COURTYARD_CATS, FOOTPRINT_CATS,
                                            MASK_CATS)
        legacy = {c: 'ignore' for c in COURTYARD_CATS + MASK_CATS
                  + FOOTPRINT_CATS}
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, DEEP, 'ignore', extra_sev=legacy))
        self.assertEqual(len(g['courtyard_blocking_pairs']), 1)
        self.assertIsNone(g['courtyard_severity'])
        self.assertTrue(g['courtyard_severity_basis'].startswith(
            'legacy severity plan'))

    def _relax(self, path):
        """Apply the CURRENT `fix_kicad_drc_settings --relax-severities`
        plan to the board's project, through the real writer."""
        from fix_kicad_drc_settings import (apply_targets_to_project,
                                            severity_plan)
        pro = os.path.splitext(path)[0] + '.kicad_pro'
        with open(pro, encoding='utf-8') as fh:
            doc = json.load(fh)
        apply_targets_to_project(doc, {}, severity_plan())
        with open(pro, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        return doc

    def test_an_author_ignore_survives_a_later_relax(self):
        """The author ignores courtyards_overlap; a current relax then
        ignores the other seven legacy categories (and records each one it
        changed). All eight read 'ignore' now, but the ignore is still the
        author's: glasgow_revC went 0 -> 21 courtyard-blocking pairs here
        before the check looked at `saved_severities`."""
        with tempfile.TemporaryDirectory() as td:
            path = board(td, DEEP, 'ignore')
            doc = self._relax(path)
            sev = doc['board']['design_settings']['rule_severities']
            self.assertTrue(all(sev.get(c) == 'ignore' for c in
                                LEGACY_SEVERITY_PLAN_IGNORES), sev)
            g = grade(path)
        self.assertEqual(g['courtyard_severity'], 'ignore')
        self.assertEqual(g['courtyard_severity_basis'], 'project')
        self.assertEqual(g['courtyard_blocking_pairs'], [])

    def test_a_legacy_ignore_stays_legacy_after_a_later_relax(self):
        """A legacy project a current tool relaxes again records nothing for
        the legacy categories (they are already at ignore), so it still
        reads as the pre-#856 tool's."""
        legacy = {c: 'ignore' for c in LEGACY_SEVERITY_PLAN_IGNORES}
        with tempfile.TemporaryDirectory() as td:
            path = board(td, DEEP, 'ignore', extra_sev=legacy)
            self._relax(path)
            g = grade(path)
        self.assertIsNone(g['courtyard_severity'])
        self.assertTrue(g['courtyard_severity_basis'].startswith(
            'legacy severity plan'))
        self.assertEqual(len(g['courtyard_blocking_pairs']), 1)

    def test_a_saved_author_value_wins_over_a_tool_ignore(self):
        """A current tool that loosened the project records the author's
        value; that value is what is graded."""
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, DEEP, 'ignore', saved='error'))
        self.assertEqual(len(g['courtyard_blocking_pairs']), 1)
        with tempfile.TemporaryDirectory() as td:
            g = grade(board(td, DEEP, 'ignore', saved='ignore'))
        self.assertEqual(g['courtyard_blocking_pairs'], [])

    def test_containment_is_not_the_courtyard_rule(self):
        """B's fab body wholly inside A's: contained at every severity."""
        parts = [('A', 10, 1.0, 0.9), ('B', 10.2, 0.5, 0.3)]
        for sev in ('ignore', 'error'):
            with tempfile.TemporaryDirectory() as td:
                g = grade(board(td, parts, sev))
            self.assertEqual([(q.a, q.b) for q in
                              g['containment_blocking_pairs']], [('A', 'B')],
                             sev)


class TestStickHub(unittest.TestCase):
    def setUp(self):
        self.path = stickhub()
        if not self.path:
            self.skipTest('KiCad StickHub demo not installed')

    def _cli(self, *extra):
        with tempfile.TemporaryDirectory() as td:
            js = os.path.join(td, 'a.json')
            r = subprocess.run([sys.executable, '-X', 'utf8',
                                os.path.join(ROOT, 'py_tools',
                                             'check_assembly.py'),
                                self.path, '--json', js, *extra],
                               capture_output=True, text=True, cwd=ROOT)
            self.assertTrue(os.path.isfile(js), r.stderr[-2000:])
            with open(js, encoding='utf-8') as fh:
                return r, json.load(fh)

    def test_the_demo_grades_at_its_own_severity(self):
        r, d = self._cli()
        self.assertEqual(d['courtyard_severity'], 'ignore')
        self.assertEqual(d['courtyard_blocking'], 0)
        self.assertTrue(d['buildable'], d['verdict'])
        self.assertIn("courtyards_overlap to 'ignore'", r.stdout)

    def test_the_flag_grades_it_at_error(self):
        _r, d = self._cli('--ignore-project-severity')
        self.assertIsNone(d['courtyard_severity'])
        # The deliberately packed JST ports: J2<->J3 ... J7<->J8.
        pairs = {(q['a'], q['b']) for q in d['courtyard_blocking_pairs']}
        self.assertIn(('J2', 'J3'), pairs)
        self.assertGreaterEqual(d['courtyard_blocking'], 5)


if __name__ == '__main__':
    unittest.main()
