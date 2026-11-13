#!/usr/bin/env python3
"""#1797: `&&` and `||` mixed at one level of a `.kicad_dru` condition bind as
KiCad binds them -- `||` TIGHTER than `&&`.

`TRUTH` is {condition: is it true on the one NA track}, measured on kicad-cli
10.0.6 with an `assertion` rule per row (KiCad reports an assertion_failure for
each item a row is false on). T and F are net-name terms true and false on that
track. Mixed rows are where the reading differs: `F && F || T` is
`F && (F || T)` = false in KiCad, and was `(F && F) || T` = true here.

The model half (`parse_condition` + `_eval`, and `_eval3` on
`parse_condition_maybe`) runs in a second. With kicad-cli the same rows are
re-measured in one DRC pass, and `CLEARANCE` holds a clearance rule built on
the two-track fixture to KiCad's count of clearance errors.

    python3 tests/test_1797_condition_precedence.py
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
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from kicad_oracle import find_kicad_cli  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from design_rules import (DesignRules, RuleItem, _eval, _eval3, parse_condition,  # noqa: E402
                          parse_condition_maybe, kicad_condition_refusal)

CLI = find_kicad_cli(warn=False)

T = "A.NetName == 'NA'"
F = "A.NetName == 'ZZ'"
#: {condition with T/F: truth on the NA track}, kicad-cli 10.0.6
TRUTH = {
    'T': True,
    'F': False,
    'F && F || T': False,
    'T || F && F': False,
    'F && T || T': False,
    'T || T && F': False,
    '(F && F) || T': True,
    'T || (F && F)': True,
    'T && F || T && T': True,
    'F || T && F || T': True,
    '!F && F': False,
    '!F || T': True,
    '!T && T || T': False,
}

#: {clearance-rule condition: KiCad's clearance errors on the NA/NB tracks,
#:  0.3 mm apart under a 1.0 mm rule and a 0.1 mm class}, kicad-cli 10.0.6
CLEARANCE = {
    "B.NetName == 'NB'": 1,
    "(A.NetName == 'NONE' && A.NetName == 'NONE') || B.NetName == 'NB'": 1,
    "A.NetName == 'NONE' && A.NetName == 'NONE' || B.NetName == 'NB'": 0,
    "B.NetName == 'NB' || A.NetName == 'NONE' && A.NetName == 'NONE'": 0,
}

BOARD = '''(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")
 (general (thickness 1.6) (legacy_teardrops no))
 (paper "A4")
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
 (setup (pad_to_mask_clearance 0))
 (net 0 "") (net 1 "NA") (net 2 "NB")
 (gr_rect (start 90 90) (end 120 110) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))
 (segment (start 100 100) (end 110 100) (width 0.2) (layer "F.Cu") (net 1) (uuid "b0000000-0000-0000-0000-000000000001"))
 (segment (start 100 100.5) (end 110 100.5) (width 0.2) (layer "F.Cu") (net 2) (uuid "b0000000-0000-0000-0000-000000000002"))
)
'''
PRO = {'board': {'design_settings': {'rules': {}}},
       'meta': {'filename': 'p.kicad_pro', 'version': 3},
       'net_settings': {'classes': [{'name': 'Default', 'clearance': 0.1, 'track_width': 0.2,
                                     'via_diameter': 0.6, 'via_drill': 0.3}],
                        'meta': {'version': 4}, 'netclass_patterns': []}}


def spell(row):
    return row.replace('T', '\0').replace('F', F).replace('\0', T)


def stage(work, name, dru):
    path = os.path.join(work, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(BOARD)
    with open(os.path.join(work, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    with open(os.path.join(work, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(dru)
    return path


def kicad_drc(path):
    out = path + '.drc.json'
    subprocess.run([CLI, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', out, path],
                   capture_output=True, text=True, timeout=300)
    with open(run_utils.evidence(out, 'the kicad-cli report'), encoding='utf-8') as fh:
        return json.load(fh)['violations']


NA = RuleItem(type='track', net_id=1, net_name='NA')
NB = RuleItem(type='track', net_id=2, net_name='NB')


class Model(unittest.TestCase):
    def test_truth_table(self):
        for row, want in TRUTH.items():
            src = spell(row)
            self.assertIsNone(kicad_condition_refusal(src), src)
            self.assertEqual(bool(_eval(parse_condition(src), NA, None, 'F.Cu')), want, row)
            self.assertEqual(_eval3(parse_condition_maybe(src), NA, None, 'F.Cu'), want, row)

    def test_mixed_reads_as_kicad(self):
        self.assertEqual(parse_condition(spell('F && F || T'))[0], 'and')
        self.assertEqual(parse_condition(spell('T || F && F'))[0], 'and')

    def test_clearance_rule(self):
        with tempfile.TemporaryDirectory() as work:
            for i, (cond, errors) in enumerate(CLEARANCE.items()):
                path = stage(work, f'c{i}', f'(version 1)\n(rule "probe" (constraint clearance '
                                            f'(min 1.0mm)) (condition "{cond}"))\n')
                pcb = parse_kicad_pcb(run_utils.evidence(path, 'the fixture'))
                dr = DesignRules.from_project(pcb, path)
                self.assertEqual(len(dr.rules), 1, cond)
                self.assertFalse(dr.rules[0].unsupported, cond)
                got = dr.resolve('clearance', dr.item_for_net(1, 'track', 'F.Cu'),
                                 dr.item_for_net(2, 'track', 'F.Cu'), 'F.Cu').min
                self.assertAlmostEqual(got, 1.0 if errors else 0.1, places=6, msg=cond)


@unittest.skipUnless(CLI, 'no kicad-cli')
class KiCad(unittest.TestCase):
    def test_truth_table(self):
        rows = list(TRUTH)
        with tempfile.TemporaryDirectory() as work:
            path = stage(work, 'assert', '(version 1)\n' + ''.join(
                f'(rule "row{i}" (condition "{T} && A.Type == \'Track\'") '
                f'(constraint assertion "{spell(r)}"))\n' for i, r in enumerate(rows)))
            failed = {rows[int(re.search(r'row(\d+)', v['description']).group(1))]
                      for v in kicad_drc(path) if v['type'] == 'assertion_failure'}
        self.assertEqual({r: r not in failed for r in rows}, TRUTH)

    def test_clearance_rule(self):
        with tempfile.TemporaryDirectory() as work:
            for i, (cond, errors) in enumerate(CLEARANCE.items()):
                path = stage(work, f'c{i}', f'(version 1)\n(rule "probe" (constraint clearance '
                                            f'(min 1.0mm)) (condition "{cond}"))\n')
                got = sum(v['type'] == 'clearance' for v in kicad_drc(path))
                self.assertEqual(got, errors, cond)


if __name__ == '__main__':
    unittest.main(verbosity=2)
