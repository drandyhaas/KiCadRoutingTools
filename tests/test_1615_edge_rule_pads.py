#!/usr/bin/env python3
"""#1615: a `.kicad_dru` rule that lowers the board-edge clearance for named
pads is read by every placement tool, as KiCad's DRC reads it.

Edge-launch connectors need it: the vendor land runs the contact pad up to
the board edge while the rest of the board keeps a floor such as 0.4 mm. The
fixture is tests/fixtures/1603/edge_sma.kicad_pcb (kicad_files/cap_chain with
KiCad's stock SMA_Amphenol_132289_EdgeMount as J3), staged with a 0.4 mm
project floor and, in the RULE arm, a rule lowering the edge clearance to 0
for J3's pads only. At x=28.41 J3's pads end 0.05 mm inside the east edge
(x=31): kicad-cli 10 reports 0 copper_edge_clearance errors on J3 with the
rule and 5 without it (measured in the issue).

  1. `design_rules.PadEdgeRules`: J3's pads read 0 by the rule, other pads
     the floor; a rule whose condition is not modelled (a predicate, or a
     term on B, the edge item) gives a RANGE, and binds nothing it cannot;
  2. `grade_pad_legality`: 0 pad-edge conflicts and no coarse-outline count
     with the rule, 5 conflicts without it;
  3. `place_pose set` to that pose is accepted with the rule and refused
     without it;
  4. `check_drc --check-pad-edge` flags J3 without the rule only;
  5. the edge seat seats J3 with its pads within 0.1 mm of the edge under
     the rule, and has no seat for it without one.

    python3 tests/test_1615_edge_rule_pads.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import unittest

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

RUN_ALL_TIMEOUT = 600
FIXTURE = os.path.join(TESTS, 'fixtures', '1603', 'edge_sma.kicad_pcb')
#: J3's origin x putting its pads 0.05 mm inside the east edge, and 1 mm in
EDGE_X, INNER_X = 28.41, 27.41
EAST = 31.0
RULE = ('(version 1)\n(rule "edge_launch" (condition "A.Type == \'Pad\' && '
        'A.memberOfFootprint(\'J3\')")\n  (constraint edge_clearance (min 0mm)))\n')


def stage(work, name, x, dru=None):
    """The fixture with J3 at (x, 15), a 0.4 mm project edge floor, and
    `dru` as its .kicad_dru when given."""
    text = open(run_utils.evidence(FIXTURE), encoding='utf-8').read()
    assert text.count('(at 28.46 15)') == 1
    path = os.path.join(work, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text.replace('(at 28.46 15)', f'(at {x:g} 15)'))
    with open(os.path.join(work, name + '.kicad_pro'), 'w') as fh:
        json.dump({'board': {'design_settings': {'rules': {
            'min_copper_edge_clearance': 0.4}}}}, fh)
    if dru is not None:
        with open(os.path.join(work, name + '.kicad_dru'), 'w') as fh:
            fh.write(dru)
    return path


def j3_east(path):
    fp = parse_kicad_pcb(path).footprints['J3']
    return max(p.global_x + p.size_x / 2 for p in fp.pads)


class EdgeRulePads(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.work = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def test_fixture_pose(self):
        self.assertAlmostEqual(j3_east(stage(self.work, 'b', EDGE_X, RULE)),
                               EAST - 0.05, places=6)

    def test_pad_edge_rules(self):
        from design_rules import PadEdgeRules
        path = stage(self.work, 'rule', EDGE_X, RULE)
        pcb = parse_kicad_pcb(path)
        rules = PadEdgeRules(pcb, path)
        self.assertTrue(rules.active)
        self.assertEqual(rules.unmodelled, [])
        for pad in pcb.footprints['J3'].pads:
            self.assertEqual(rules.for_pad(pad, 0.4), (0.0, 0.0, 'edge_launch'))
        self.assertEqual(rules.for_pad(pcb.footprints['C1'].pads[0], 0.4), (0.4, 0.4, ''))
        # a courtyard term reads as that footprint's pads, and says so
        path = stage(self.work, 'cy', EDGE_X,
                     '(version 1)\n(rule "cy" (condition "A.insideCourtyard(\'J3\')")\n'
                     '  (constraint edge_clearance (min 0mm)))\n')
        pcb = parse_kicad_pcb(path)
        rules = PadEdgeRules(pcb, path)
        self.assertEqual(rules.unmodelled, [])
        self.assertIn('courtyard', rules.notes['cy'])
        self.assertEqual(rules.for_pad(pcb.footprints['J3'].pads[0], 0.4), (0.0, 0.0, 'cy'))
        self.assertEqual(rules.for_pad(pcb.footprints['C1'].pads[0], 0.4), (0.4, 0.4, ''))
        # not modelled: a predicate, and a term on B (the edge item)
        for cond, c1 in (("A.memberOfSheet('/rf/')", (0.0, 0.4, '')),
                         ("B.Layer == 'Edge.Cuts' && A.memberOfFootprint('J3')",
                          (0.4, 0.4, ''))):
            with self.subTest(cond=cond):
                path = stage(self.work, 'maybe', EDGE_X,
                             f'(version 1)\n(rule "maybe" (condition "{cond}")\n'
                             f'  (constraint edge_clearance (min 0mm)))\n')
                pcb = parse_kicad_pcb(path)
                rules = PadEdgeRules(pcb, path)
                self.assertEqual([r['rule'] for r in rules.unmodelled], ['maybe'])
                self.assertEqual(rules.for_pad(pcb.footprints['J3'].pads[0], 0.4),
                                 (0.0, 0.4, ''))
                self.assertEqual(rules.for_pad(pcb.footprints['C1'].pads[0], 0.4), c1)
        # a board with no edge rule is inert
        path = stage(self.work, 'none', EDGE_X)
        rules = PadEdgeRules(parse_kicad_pcb(path), path)
        self.assertFalse(rules.active)

    def test_grade(self):
        from placement.legality import grade_pad_legality
        for name, dru, conflicts in (('rule', RULE, 0), ('norule', None, 5)):
            with self.subTest(arm=name):
                path = stage(self.work, name, EDGE_X, dru)
                g = grade_pad_legality(parse_kicad_pcb(path), 0.25, pcb_file=path)
                self.assertEqual(g['pad_edge_conflicts'], conflicts)
                if dru:
                    self.assertEqual(g['oob_pad_count'], 0)
                    edge = g['pad_edge']
                    self.assertEqual(edge['rules_unmeasured'], [])
                    self.assertEqual([(r['rule'], r['required_mm'], len(r['pads']))
                                      for r in edge['rules_applied']],
                                     [('edge_launch', 0.0, 5)])
                else:
                    self.assertEqual({f['required_mm'] for f in g['pad_edge']['findings']},
                                     {0.4})

    def test_place_pose(self):
        for name, dru in (('rule', RULE), ('norule', None)):
            with self.subTest(arm=name):
                src = stage(self.work, name, INNER_X, dru)
                argv = [sys.executable, '-X', 'utf8',
                        os.path.join(REPO, 'py_placer', 'place_pose.py'), src,
                        os.path.join(self.work, name + '_out.kicad_pcb'),
                        'set', 'J3', str(EDGE_X), '15']
                if dru:
                    run_utils.check(argv, accept=True, cwd=REPO)
                else:
                    run_utils.check(argv, refuse='pad_edge_conflicts 0 -> 5', code=4,
                                    cwd=REPO)

    def test_check_drc(self):
        for name, dru, flagged in (('rule', RULE, 0), ('norule', None, 5)):
            with self.subTest(arm=name):
                path = stage(self.work, name, EDGE_X, dru)
                r = subprocess.run([sys.executable, '-X', 'utf8',
                                    os.path.join(REPO, 'py_router', 'check_drc.py'), path,
                                    '--check-pad-edge', '--board-edge-clearance', '0.4'],
                                   cwd=REPO, capture_output=True, text=True,
                                   encoding='utf-8', timeout=300)
                self.assertNotIn('Traceback', r.stdout + r.stderr)
                self.assertIn('Checking pad-to-board-edge clearances', r.stdout)
                self.assertEqual(len(re.findall(r'\bJ3\.\d', r.stdout)), flagged, r.stdout)

    def test_seat(self):
        overhang = None
        for name, dru in (('rule', RULE), ('norule', None)):
            with self.subTest(arm=name):
                board = stage(self.work, name, EDGE_X, dru)
                emitted = os.path.join(self.work, name + '_emit.json')
                run_utils.check([sys.executable, '-X', 'utf8',
                                 os.path.join(REPO, 'py_tools', 'check_floorplan.py'), board,
                                 '--emit-intent', emitted, '--allow-unplaced'],
                                accept=True, cwd=REPO)
                intent = json.load(open(run_utils.evidence(emitted)))
                if overhang is None:
                    # the band J3 reads at the pose KiCad passes
                    intent['edge_connectors'] = [{'ref': 'J3', 'edge': 'east',
                                                  'overhang_mm': {'min': 0, 'max': 30},
                                                  'why': 'probe'}]
                    probe = os.path.join(self.work, 'probe_intent.json')
                    json.dump(intent, open(probe, 'w'))
                    graded = os.path.join(self.work, 'grade.json')
                    subprocess.run([sys.executable, '-X', 'utf8',
                                    os.path.join(REPO, 'py_tools', 'check_floorplan.py'),
                                    board, '--intent', probe, '--json', graded],
                                   cwd=REPO, capture_output=True, timeout=300)
                    row = [x for x in json.load(open(run_utils.evidence(graded)))
                           ['edge_seating'] if x['ref'] == 'J3'][0]
                    overhang = row['overhang_mm']
                    self.assertEqual(row['pad_copper_edge']['disposition'], 'pass')
                    self.assertEqual(row['pad_copper_edge']['required_mm'], 0.0)
                intent['edge_connectors'] = [{
                    'ref': 'J3', 'edge': 'east', 'why': 'edge launch',
                    'overhang_mm': {'min': round(overhang - 0.05, 3),
                                    'max': round(overhang + 0.05, 3)}}]
                seat = os.path.join(self.work, name + '_seat.json')
                json.dump(intent, open(seat, 'w'))
                refs = sorted(parse_kicad_pcb(board).footprints)
                groups = os.path.join(self.work, name + '_groups.json')
                json.dump({'groups': [{'name': 'all', 'refs': refs}]}, open(groups, 'w'))
                out = os.path.join(self.work, name + '_s{seed}.kicad_pcb')
                r = subprocess.run([sys.executable, '-X', 'utf8',
                                    os.path.join(REPO, 'py_placer', 'place_simple.py'), board,
                                    out, '--groups', groups, '--intent', seat,
                                    '--json', os.path.join(self.work, name + '_s{seed}.json'),
                                    '--scattered', '1', '--clustered', '0', '--moves', '500'],
                                   cwd=REPO, capture_output=True, text=True,
                                   encoding='utf-8', timeout=600)
                self.assertNotIn('Traceback', r.stdout + r.stderr)
                log = open(run_utils.evidence(
                    os.path.join(self.work, name + '_s0.log')), encoding='utf-8').read()
                unseated = 'J3 is declared on an edge but has no seat' in log
                if dru:
                    self.assertFalse(unseated, log)
                    gap = EAST - j3_east(out.format(seed=0))
                    self.assertTrue(0.0 <= gap <= 0.1, gap)
                else:
                    self.assertTrue(unseated, log)


if __name__ == '__main__':
    unittest.main()
