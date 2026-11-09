#!/usr/bin/env python3
"""#1660: `A.Pad_Number` in a `.kicad_dru` condition is read as KiCad reads it,
so an edge rule freeing a connector's named pads gives `PadEdgeRules.for_pad`
KiCad's value and the seat reaches the pose KiCad passes.

The fixture is #1615's (test_1615_edge_rule_pads.stage): a connector whose
pads end 0.05 mm inside the east edge under a 0.4 mm project floor. `TABLE`
stages it with the connector's five pads renumbered and a track and a via
beside the edge, and records which of those seven items each condition frees
from the floor. `test_table_is_kicad` holds the table to kicad-cli and
`test_table_is_the_model` holds the model to the table, so the second row of
that pair runs where there is no KiCad.

What the table pins:
  1. a pad number is a string: '1' is not '01', '3' is not '3.0';
  2. it compares as a NetName does: case-insensitive, a '*'/'?' literal on the
     right a wildcard, '[A]1' no character class;
  3. a track or a via has none, so == and != are both false on it, and a
     property alone (`A.Pad_Number`, `A.NetName`) is false;
  4. KiCad tries a failed condition again with the items swapped, and here the
     other item is the Edge.Cuts shape: `!(A.Pad_Number == '1')` frees every
     item, pad 1 included, while the same term scoped to a footprint does not.

A plain number (`A.Pad_Number == 1`) is a compile error in KiCad, which then
enforces no rule of the file: `test_plain_number_is_named`.

    python3 tests/test_1660_pad_number_rule.py
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
from kicad_exact_fill import find_kicad_python  # noqa: E402
from kicad_oracle import find_kicad_cli  # noqa: E402
from test_1615_edge_rule_pads import EAST, EDGE_X, INNER_X, j3_east, stage  # noqa: E402

RUN_ALL_TIMEOUT = 900
NUMBERS = ('1', 'A1', '02', '3.0', '')
PADS = frozenset(NUMBERS)
ALL = PADS | {'track', 'via'}
NONE = frozenset()


def _but(*numbers):
    return PADS - set(numbers)


#: condition -> the items it frees from the floor, as kicad-cli 10.0.6 grades them
TABLE = {
    # a string, and only a string
    "A.Pad_Number == '1'": {'1'},
    "A.Pad_Number == '01'": NONE,
    "A.Pad_Number == '2'": NONE,
    "A.Pad_Number == '3'": NONE,
    "A.Pad_Number == '3.0'": {'3.0'},
    "A.Pad_Number == ''": {''},
    "A.Pad_Number != ''": _but(''),
    "A.Pad_Number != '1'": _but('1'),
    "A.Pad_Number == '3.0' || A.Pad_Number == '02'": {'3.0', '02'},
    # the spelling of the property, and of the value
    "A.pad_number == '1'": {'1'},
    "A.PAD_NUMBER == '1'": {'1'},
    "A.Pad_Number == 'a1'": {'A1'},
    # a literal on the right is a wildcard, one on the left is not
    "A.Pad_Number == 'A*'": {'A1'},
    "A.Pad_Number == 'a*'": {'A1'},
    "A.Pad_Number == '?1'": {'A1'},
    "A.Pad_Number == '*'": PADS,
    "A.Pad_Number != 'A*'": _but('A1'),
    "A.Pad_Number == '[A]1'": NONE,
    "'1' == A.Pad_Number": {'1'},
    "'a1' == A.Pad_Number": {'A1'},
    "'A*' == A.Pad_Number": NONE,
    "'1' != A.Pad_Number": _but('1'),
    # the property against itself, and alone
    "A.Pad_Number == A.Pad_Number": PADS,
    "A.Pad_Number != A.Pad_Number": NONE,
    "A.Pad_Number": NONE,
    # with the terms a rule has around it
    "A.Type == 'Pad' && A.Pad_Number == '1'": {'1'},
    "A.Pad_Number == '1' && A.Pad_Type == 'SMD'": {'1'},
    "A.memberOfFootprint('J3') && A.Pad_Number != '1'": _but('1'),
    "A.memberOfFootprint('J3') && !(A.Pad_Number == '1')": _but('1'),
    "A.Type == 'Pad' && !(A.Pad_Number == '1')": _but('1'),
    # the swap: A is the Edge.Cuts shape, which has no pad number
    "!(A.Pad_Number == '1')": ALL,
    "!A.Pad_Number": ALL,
    "A.Type == 'Pad' || !(A.Pad_Number == '1')": ALL,
    "!(A.Pad_Number == '1') && !(A.Pad_Number == 'A1')": ALL,
    "B.Pad_Number == '1'": {'1'},
    "B.Pad_Number != '1'": _but('1'),
    # the swap on what the shape does have
    "!(A.memberOfFootprint('J3'))": ALL,
    "!(A.Type == 'Pad')": ALL,
    "!(A.Type == 'Track')": ALL,
    "A.Type == 'Graphic'": ALL,
    "A.Layer == 'Edge.Cuts'": ALL,
    "A.Layer != 'F.Cu'": ALL,
    "!(A.Layer == 'F.Cu')": ALL,
    "A.NetName == ''": ALL,
    "!(A.NetName == '/x')": ALL,
    "A.NetClass == 'Default'": ALL,
    "!(A.NetClass == 'Default')": NONE,
    "A.NetClass == 'HS'": NONE,
    "!(A.NetClass == 'HS')": ALL,
    "A.hasNetclass('HS')": NONE,
    "!A.hasNetclass('HS')": ALL,
    "!A.hasNetclass('Default')": ALL,
    "!A.hasExactNetclass('Default')": ALL,
    "!A.isPlated()": ALL,
    "!A.isMicroVia()": ALL,
    # a property only a pad or only a via has is undefined on the rest
    "A.Pad_Type == 'SMD'": PADS,
    "A.Pad_Type != 'SMD'": NONE,
    "A.Pad_Type != 'Through-hole'": PADS,
    "!(A.Pad_Type == 'SMD')": ALL,
    "A.Via_Type != 'Through'": NONE,
    "!(A.Via_Type == 'Through')": ALL,
    # a string alone is false
    "A.NetName": NONE,
    "A.NetClass": NONE,
    "A.Type": NONE,
    "A.Layer": NONE,
    "A.Pad_Type": NONE,
    "A.Via_Type": NONE,
    "!A.NetName": ALL,
    "!A.Type": ALL,
    # the controls
    "A.Type == 'Track'": {'track'},
    "A.Type == 'Via'": {'via'},
}

#: A plain number: KiCad refuses the condition and enforces no rule of the file
REFUSED = ("A.Pad_Number == 1", "A.Pad_Number != 1", "1 == A.Pad_Number",
           "A.Pad_Number == 1.0", "!(A.Pad_Number == 1)",
           "A.Pad_Number == 1 || A.Pad_Number == '02'")


#: Runs in KiCad's python: each pad's requirement from build_pcb_data_from_board
PCBNEW_PROBE = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
import pcbnew
from kicad_parser import build_pcb_data_from_board
from design_rules import PadEdgeRules
pcb = build_pcb_data_from_board(pcbnew.LoadBoard(sys.argv[2]))
rules = PadEdgeRules(pcb, sys.argv[2])
print('RESULT ' + json.dumps({p.pad_number: rules.for_pad(p, 0.4)[1]
                              for p in pcb.footprints['J3'].pads}))
'''


def edge_rule(*conds):
    rules = ''.join(f'(rule "r{i}" (condition "{c}")\n  (constraint edge_clearance (min 0mm)))\n'
                    for i, c in enumerate(conds))
    return '(version 1)\n' + rules


def renumbered(work, name, dru, extra=''):
    """The fixture with J3's five pads numbered NUMBERS and a track and a via
    0.1 and 0.4 mm off the east edge."""
    path = stage(work, name, EDGE_X, dru)
    text = open(run_utils.evidence(path), encoding='utf-8').read()
    head, sep, tail = text.partition(f'(at {EDGE_X:g} 15)')
    assert sep, 'the staged connector pose is not in the file'
    numbers = iter(NUMBERS)
    tail, n = re.subn(r'\(pad "[12]" smd rect',
                      lambda m: f'(pad "{next(numbers)}" smd rect', tail, count=len(NUMBERS))
    assert n == len(NUMBERS), n
    extra += ('\t(segment (start 30.9 7) (end 30.9 9) (width 0.1) (layer "F.Cu") (net "DPB_N") '
              '(uuid "11111111-1111-1111-1111-111111111111"))\n'
              '\t(via (at 30.6 22) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "DPB_N") '
              '(uuid "11111111-1111-1111-1111-111111111112"))\n')
    body = (head + sep + tail).rstrip()
    end = body.rfind(')')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(body[:end] + extra + body[end:] + '\n')
    return path


def kicad_edge_flagged(cli, path):
    """The items kicad-cli reports past their edge clearance, as TABLE names them."""
    out = path + '.drc.json'
    run_utils.check([cli, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', out, path],
                    accept=True)
    flagged = set()
    for v in json.load(open(run_utils.evidence(out)))['violations']:
        if v['type'] != 'copper_edge_clearance':
            continue
        for item in v['items']:
            d = item['description']
            m = re.match(r'Pad(?: (\S+))? \[', d)
            if m:
                flagged.add(m.group(1) or '')
            elif d.startswith('Track'):
                flagged.add('track')
            elif d.startswith('Via'):
                flagged.add('via')
    return flagged


def model_freed(path):
    """The items the model frees from the 0.4 mm floor: each pad by `for_pad`,
    the track and the via by the rule walk `for_pad` makes, on their own layer.
    A range here is a failure: every TABLE row is decided."""
    from design_rules import DesignRules, PadEdgeRules, _EDGE_ITEM, rule_may_match
    pcb = parse_kicad_pcb(path)
    rules = PadEdgeRules(pcb, path)
    freed = set()
    for pad in pcb.footprints['J3'].pads:
        lo, hi, _rule = rules.for_pad(pad, 0.4)
        assert lo == hi, f'pad {pad.pad_number!r} reads a range {lo}..{hi}: {rules.unmodelled}'
        if hi == 0.0:
            freed.add(pad.pad_number)
    dr = DesignRules.from_project(pcb, path)
    net = next(n for n, x in pcb.nets.items() if x.name == 'DPB_N')
    items = (('track', dr.item_for_net(net, 'track', 'F.Cu')),
             ('via', dr.item_for_net(net, 'via', 'F.Cu', via_type='through')))
    ctx = ('edge_clearance', dr._net_name_set())
    for name, item in items:
        if any(rule_may_match(rule, ast, item, _EDGE_ITEM, 'F.Cu', dr.copper_layers, ctx)
               for rule, ast, _v in rules.rules):
            freed.add(name)
    return freed


class PadNumberRule(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.work = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def test_table_is_the_model(self):
        for i, (cond, want) in enumerate(TABLE.items()):
            with self.subTest(cond=cond):
                path = renumbered(self.work, f'm{i}', edge_rule(cond))
                self.assertEqual(model_freed(path), set(want))

    @unittest.skipUnless(find_kicad_cli(warn=False), 'no kicad-cli')
    def test_table_is_kicad(self):
        cli = find_kicad_cli(warn=False)
        base = kicad_edge_flagged(cli, renumbered(self.work, 'none', None))
        self.assertEqual(base, set(ALL), 'without a rule KiCad flags every item')
        for i, (cond, want) in enumerate(TABLE.items()):
            with self.subTest(cond=cond):
                path = renumbered(self.work, f'k{i}', edge_rule(cond))
                self.assertEqual(ALL - kicad_edge_flagged(cli, path), set(want),
                                 'the table is stale')

    def test_for_pad_decides_and_names_the_rule(self):
        from design_rules import PadEdgeRules
        path = renumbered(self.work, 'decide', edge_rule(
            "A.memberOfFootprint('J3') && (A.Pad_Number == '1' || A.Pad_Number == 'A1')"))
        pcb = parse_kicad_pcb(path)
        rules = PadEdgeRules(pcb, path)
        self.assertEqual(rules.unmodelled, [])
        got = {p.pad_number: rules.for_pad(p, 0.4) for p in pcb.footprints['J3'].pads}
        self.assertEqual(got, {'1': (0.0, 0.0, 'r0'), 'A1': (0.0, 0.0, 'r0'),
                               '02': (0.4, 0.4, ''), '3.0': (0.4, 0.4, ''), '': (0.4, 0.4, '')})
        # another part's pad 1 is not J3's
        self.assertEqual(rules.for_pad(pcb.footprints['C1'].pads[0], 0.4), (0.4, 0.4, ''))

    def test_a_unit_still_reads_a_range(self):
        from design_rules import PadEdgeRules
        path = renumbered(self.work, 'unit', edge_rule("A.Pad_Number == 1mm"))
        pcb = parse_kicad_pcb(path)
        rules = PadEdgeRules(pcb, path)
        self.assertEqual([r['rule'] for r in rules.unmodelled], ['r0'])
        self.assertEqual(rules.for_pad(pcb.footprints['J3'].pads[0], 0.4), (0.0, 0.4, ''))

    def test_plain_number_is_named(self):
        from design_rules import DesignRules, PadEdgeRules, Unsupported, parse_condition
        for i, cond in enumerate(REFUSED):
            with self.subTest(cond=cond):
                with self.assertRaisesRegex(Unsupported, 'KiCad refuses'):
                    parse_condition(cond)
                path = renumbered(self.work, f'r{i}', edge_rule(cond))
                pcb = parse_kicad_pcb(path)
                rules = PadEdgeRules(pcb, path)
                self.assertEqual([r['rule'] for r in rules.unmodelled], ['r0'])
                self.assertIn('enforces no rule of this file',
                              rules.unmodelled[0]['reason'])
                self.assertEqual(rules.for_pad(pcb.footprints['J3'].pads[0], 0.4),
                                 (0.0, 0.4, ''))
                lines = DesignRules.from_project(pcb, path).report_lines()
                self.assertIn('KiCad ENFORCES NO RULE OF THIS FILE', lines[0])
                line = [ln for ln in lines if ln.startswith(".kicad_dru rule 'r0'")]
                self.assertEqual(len(line), 1, line)
                self.assertIn('NOT honoured', line[0])
                self.assertNotIn('KiCad will still enforce', line[0])

    @unittest.skipUnless(find_kicad_cli(warn=False), 'no kicad-cli')
    def test_plain_number_drops_the_file_in_kicad(self):
        cli = find_kicad_cli(warn=False)
        files = [(cond, "A.Type == 'Track'") for cond in REFUSED]
        files.append(("A.Type == 'Track'", REFUSED[0]))
        for i, conds in enumerate(files):
            with self.subTest(conds=conds):
                path = renumbered(self.work, f'f{i}', edge_rule(*conds))
                self.assertEqual(kicad_edge_flagged(cli, path), set(ALL),
                                 'the file carries a rule that binds the track')

    def test_both_parse_paths(self):
        """The number reaches the rule from the board pcbnew loads too."""
        # ipc-migration: the second parse path here is the kipy builder, which
        # takes each pad's number from kipy as the string the file spells; the
        # pcbnew builder this arm drives was deleted with the SWIG front. Not
        # covered: the kipy builder's pad numbers against a live KiCad.
        self.skipTest('ipc-migration: the pcbnew builder this arm drives is '
                      'deleted; the kipy builder reads pad.number as the file '
                      'spells it, and no live-KiCad arm runs here')
        python = find_kicad_python()
        if not python:
            self.skipTest('no KiCad python')
        path = renumbered(self.work, 'gui', edge_rule(
            "A.Pad_Number == '1' || A.Pad_Number == 'a1' || A.Pad_Number == ''"))
        r = subprocess.run([python, '-c', PCBNEW_PROBE, os.path.join(REPO, 'py_router'), path],
                           capture_output=True, text=True, encoding='utf-8', timeout=300)
        got = [ln for ln in r.stdout.splitlines() if ln.startswith('RESULT ')]
        self.assertEqual(len(got), 1, r.stdout + r.stderr)
        self.assertEqual(json.loads(got[0][len('RESULT '):]),
                         {'1': 0.0, 'A1': 0.0, '': 0.0, '02': 0.4, '3.0': 0.4})

    def _clearance_board(self, name, dru):
        """A track 0.1 mm off pad 1 and another 0.1 mm off pad A1, on another net."""
        path = renumbered(self.work, name, dru)
        pads = {p.pad_number: p for p in parse_kicad_pcb(path).footprints['J3'].pads}
        one, a1 = pads['1'], pads['A1']
        sign = 1 if one.global_y > a1.global_y else -1
        extra = ''
        for k, (pad, side) in enumerate(((one, -sign), (a1, -sign))):
            y = pad.global_y + side * (pad.size_y / 2 + 0.1 + 0.05)
            extra += (f'\t(segment (start {pad.global_x - 0.4:.4f} {y:.4f}) '
                      f'(end {pad.global_x + 0.2:.4f} {y:.4f}) (width 0.1) (layer "F.Cu") '
                      f'(net "DPB_N") (uuid "22222222-2222-2222-2222-22222222222{k}"))\n')
        text = open(path, encoding='utf-8').read().rstrip()
        end = text.rfind(')')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text[:end] + extra + text[end:] + '\n')
        return path

    def test_clearance_rule_binds_the_numbered_pad(self):
        from design_rules import DesignRules
        path = self._clearance_board('clr', '''(version 1)
(rule "pad1" (condition "A.Pad_Number == '1' && B.Type == 'Track'")
  (constraint clearance (min 0.05mm)))
''')
        pcb = parse_kicad_pcb(path)
        dr = DesignRules.from_project(pcb, path)
        net = next(n for n, x in pcb.nets.items() if x.name == 'DPB_N')
        track = dr.item_for_net(net, 'track', 'F.Cu')
        got = {p.pad_number: dr.resolve('clearance', dr.item_for_pad(p, 'F.Cu'), track, 'F.Cu')
               for p in pcb.footprints['J3'].pads}
        self.assertEqual(got['1'].min_rule, 'pad1')
        self.assertEqual(got['1'].min, 0.05)
        self.assertIsNone(got['A1'].min_rule)
        # no routing step prices one pad, and the run says so
        self.assertEqual([n for n, _why in dr.unsupported()], ['pad1'])
        line = [ln for ln in dr.report_lines() if "'pad1'" in ln]
        self.assertEqual(len(line), 1, line)
        self.assertIn('NOT honoured (clearance scoped by A.Pad_Number', line[0])
        self.assertIn('KiCad will still enforce it', line[0])
        # commutative: the track as A
        self.assertEqual(dr.resolve('clearance', track, dr.item_for_pad(
            pcb.footprints['J3'].pads[0], 'F.Cu'), 'F.Cu').min_rule, 'pad1')

    @unittest.skipUnless(find_kicad_cli(warn=False), 'no kicad-cli')
    def test_clearance_rule_in_kicad(self):
        cli = find_kicad_cli(warn=False)
        path = self._clearance_board('clrk', '''(version 1)
(rule "pad1" (condition "A.Pad_Number == '1' && B.Type == 'Track'")
  (constraint clearance (min 0.05mm)))
''')
        bare = self._clearance_board('clrb', None)

        def pads_too_close(p):
            out = p + '.json'
            run_utils.check([cli, 'pcb', 'drc', '--format', 'json', '--severity-error',
                             '-o', out, p], accept=True)
            found = set()
            for v in json.load(open(run_utils.evidence(out)))['violations']:
                if v['type'] == 'clearance':
                    for item in v['items']:
                        m = re.match(r'Pad (\S+) \[', item['description'])
                        if m:
                            found.add(m.group(1))
            return found
        self.assertEqual(pads_too_close(bare), {'1', 'A1'})
        self.assertEqual(pads_too_close(path), {'A1'})

    def test_check_drc_grades_the_clearance_rule_as_unverified(self):
        path = self._clearance_board('cd', '''(version 1)
(rule "pad1" (condition "A.Pad_Number == '1' && B.Type == 'Track'")
  (constraint clearance (min 0.05mm)))
''')
        out = path + '.json'
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(REPO, 'py_router', 'check_drc.py'), path, '--json', out],
                           cwd=REPO, capture_output=True, text=True, encoding='utf-8', timeout=300)
        self.assertNotIn('Traceback', r.stdout + r.stderr)
        self.assertIn('NOT GRADED', r.stdout)
        self.assertIn('pad1', r.stdout)
        rows = [v for v in json.load(open(run_utils.evidence(out)))['items']
                if v.get('type') == 'pad-segment']
        tags = {v.get('pad_ref'): (v.get('unverified_rule'), v.get('unverified_binds'))
                for v in rows}
        self.assertEqual(tags.get('J3.1'), ('pad1', 'yes'), rows)
        self.assertEqual(tags.get('J3.A1'), (None, None), rows)

    def test_tools_read_the_rule(self):
        """The issue's two conditions bind the same pads, so every tool agrees."""
        from placement.legality import grade_pad_legality
        good = ("A.Type == 'Pad' && (A.Pad_Number == '1' || A.Pad_Number == '2') "
                "&& A.memberOfFootprint('J3')")
        absent = "A.Type == 'Pad' && A.Pad_Number == '9' && A.memberOfFootprint('J3')"
        for name, cond, conflicts in (('good', good, 0), ('absent', absent, 5)):
            with self.subTest(arm=name):
                path = stage(self.work, name, EDGE_X, edge_rule(cond))
                g = grade_pad_legality(parse_kicad_pcb(path), 0.25, pcb_file=path)
                self.assertEqual(g['pad_edge_conflicts'], conflicts)
                self.assertEqual(g['pad_edge']['rules_unmeasured'], [])
                r = subprocess.run([sys.executable, '-X', 'utf8',
                                    os.path.join(REPO, 'py_router', 'check_drc.py'), path,
                                    '--check-pad-edge', '--board-edge-clearance', '0.4'],
                                   cwd=REPO, capture_output=True, text=True,
                                   encoding='utf-8', timeout=300)
                self.assertNotIn('Traceback', r.stdout + r.stderr)
                self.assertEqual(len(re.findall(r'\bJ3\.\d', r.stdout)), conflicts, r.stdout)
                src = stage(self.work, name + '_in', INNER_X, edge_rule(cond))
                argv = [sys.executable, '-X', 'utf8',
                        os.path.join(REPO, 'py_placer', 'place_pose.py'), src,
                        os.path.join(self.work, name + '_out.kicad_pcb'),
                        'set', 'J3', str(EDGE_X), '15']
                if conflicts:
                    run_utils.check(argv, refuse='pad_edge_conflicts 0 -> 5', code=4, cwd=REPO)
                else:
                    run_utils.check(argv, accept=True, cwd=REPO)

    @unittest.skipUnless(find_kicad_cli(warn=False), 'no kicad-cli')
    def test_kicad_on_the_issues_conditions(self):
        cli = find_kicad_cli(warn=False)

        def on_j3(path):
            out = path + '.json'
            run_utils.check([cli, 'pcb', 'drc', '--format', 'json', '--severity-error',
                             '-o', out, path], accept=True)
            return sum(v['type'] == 'copper_edge_clearance'
                       and any('J3' in i['description'] for i in v['items'])
                       for v in json.load(open(run_utils.evidence(out)))['violations'])
        self.assertEqual(on_j3(stage(self.work, 'kg', EDGE_X, edge_rule(
            "A.Type == 'Pad' && (A.Pad_Number == '1' || A.Pad_Number == '2') "
            "&& A.memberOfFootprint('J3')"))), 0)
        self.assertEqual(on_j3(stage(self.work, 'ka', EDGE_X, edge_rule(
            "A.Type == 'Pad' && A.Pad_Number == '9' && A.memberOfFootprint('J3')"))), 5)

    def test_seat(self):
        cond = ("A.Type == 'Pad' && (A.Pad_Number == '1' || A.Pad_Number == '2') "
                "&& A.memberOfFootprint('J3')")
        board = stage(self.work, 'seat', EDGE_X, edge_rule(cond))
        emitted = os.path.join(self.work, 'seat_emit.json')
        run_utils.check([sys.executable, '-X', 'utf8',
                         os.path.join(REPO, 'py_tools', 'check_floorplan.py'), board,
                         '--emit-intent', emitted, '--allow-unplaced'], accept=True, cwd=REPO)
        intent = json.load(open(run_utils.evidence(emitted)))
        intent['edge_connectors'] = [{'ref': 'J3', 'edge': 'east', 'why': 'probe',
                                      'overhang_mm': {'min': 0, 'max': 30}}]
        probe = os.path.join(self.work, 'seat_probe.json')
        json.dump(intent, open(probe, 'w'))
        graded = os.path.join(self.work, 'seat_grade.json')
        subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(REPO, 'py_tools', 'check_floorplan.py'), board,
                        '--intent', probe, '--json', graded],
                       cwd=REPO, capture_output=True, timeout=300)
        row = [x for x in json.load(open(run_utils.evidence(graded)))['edge_seating']
               if x['ref'] == 'J3'][0]
        self.assertEqual(row['pad_copper_edge']['required_mm'], 0.0)
        over = row['overhang_mm']
        intent['edge_connectors'][0]['overhang_mm'] = {'min': round(over - 0.05, 3),
                                                       'max': round(over + 0.05, 3)}
        seat = os.path.join(self.work, 'seat_intent.json')
        json.dump(intent, open(seat, 'w'))
        groups = os.path.join(self.work, 'seat_groups.json')
        json.dump({'groups': [{'name': 'all', 'refs': sorted(parse_kicad_pcb(board).footprints)}]},
                  open(groups, 'w'))
        out = os.path.join(self.work, 'seat_s{seed}.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(REPO, 'py_placer', 'place_simple.py'), board, out,
                            '--groups', groups, '--intent', seat,
                            '--json', os.path.join(self.work, 'seat_s{seed}.json'),
                            '--scattered', '1', '--clustered', '0', '--moves', '500'],
                           cwd=REPO, capture_output=True, text=True, encoding='utf-8', timeout=600)
        self.assertNotIn('Traceback', r.stdout + r.stderr)
        log = open(run_utils.evidence(os.path.join(self.work, 'seat_s0.log')),
                   encoding='utf-8').read()
        self.assertNotIn('J3 is declared on an edge but has no seat', log)
        gap = EAST - j3_east(out.format(seed=0))
        self.assertTrue(0.0 <= gap <= 0.1, gap)


if __name__ == '__main__':
    unittest.main()
