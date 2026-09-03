#!/usr/bin/env python3
"""design_rules.py: KiCad's precedence, one tier boundary per test (#530).

Every row cites the KiCad engine fact it encodes (pcbnew/drc/drc_engine.cpp,
9.0 and master, read 2026-09-03). Where tests/oracle/constraint_agreement.py
can measure the same boundary against kicad-cli, the row name matches its
fixture name so the two stay in step.
"""
import json
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from design_rules import (DesignRules, RuleItem, Unsupported, parse_condition,  # noqa: E402
                          parse_dru)


def _item(net_id=1, classes=(), eff=None, type='track', layers=(), **kw):
    classes = frozenset(classes)
    if eff is None:
        eff = sorted(classes)[0] if classes else 'Default'
    return RuleItem(type=type, net_id=net_id, net_name=kw.pop('net_name', f'N{net_id}'),
                    netclasses=classes, effective_class=eff,
                    layers=frozenset(layers), **kw)


def _rules(board_min=None, classes=None, memberships=None, dru='', copper=('F.Cu', 'B.Cu'),
           fab=None, cli=None, net_names=None):
    dr = DesignRules()
    dr.board_min = dict(board_min or {})
    dr.classes = {k: dict(v) for k, v in (classes or {}).items()}
    dr.memberships = {k: frozenset(v) for k, v in (memberships or {}).items()}
    dr.net_names = dict(net_names or {})
    dr.copper_layers = list(copper)
    dr.fab_floor = dict(fab or {})
    dr.cli = dict(cli or {})
    if dru:
        dr.rules, dr.notes = parse_dru(dru)
    dr._finish()
    return dr


class Parser(unittest.TestCase):
    def test_every_constraint_kind_and_every_field_is_read(self):
        rules, notes = parse_dru('''(version 1)
            (rule "w" (layer outer)
              (constraint track_width (min 0.1mm) (opt 0.2mm) (max 1mm))
              (constraint diff_pair_gap (min 4mil) (opt 0.16mm))
              (constraint via_diameter (min 0.4mm))
              (constraint hole_size (min 0.15mm))
              (constraint edge_clearance (min 0.3mm))
              (constraint hole_to_hole (min 0.25mm))
              (constraint annular_width (min 0.1mm))
              (constraint via_count (max 2))
              (constraint length (min 42.5mm) (max 43.5mm) (opt 43mm))
              (constraint disallow buried_via micro_via))''')
        self.assertEqual(len(rules), 1)
        r = rules[0]
        self.assertIsNone(r.unsupported)
        c = r.constraints
        self.assertEqual(c['track_width'], {'min': 0.1, 'opt': 0.2, 'max': 1.0})
        self.assertAlmostEqual(c['diff_pair_gap']['min'], 0.1016)
        self.assertEqual(c['via_diameter'], {'min': 0.4})
        self.assertEqual(c['hole_size'], {'min': 0.15})
        self.assertEqual(c['via_count'], {'max': 2})
        self.assertEqual(c['length'], {'min': 42.5, 'max': 43.5, 'opt': 43.0})
        self.assertEqual(c['disallow'], {'disallow': {'buried_via', 'micro_via'}})
        self.assertEqual(r.layer_clause, 'outer')
        # via_count / length are parsed but nobody consumes them yet: SAID.
        self.assertEqual(r.unconsumed_kinds, ['length'])
        self.assertEqual(notes, [])

    def test_severity_ignore_and_unknown_kinds_are_kept_and_marked(self):
        rules, notes = parse_dru('''(version 1)
            (rule "a" (severity ignore) (constraint clearance (min 1mm)))
            (rule "b" (constraint frobnicate (min 1mm)))''')
        self.assertEqual(rules[0].unsupported, 'severity ignore')
        self.assertIsNone(rules[1].unsupported)
        self.assertTrue(any('frobnicate' in n for n in notes))

    def test_unsupported_condition_marks_the_rule_not_drops_it(self):
        rules, _ = parse_dru('''(version 1)
            (rule "area" (constraint clearance (min 0.1mm))
              (condition "A.intersectsArea('underFPGA')"))
            (rule "len" (constraint clearance (min 0.1mm))
              (condition "A.Length > 3mm"))''')
        self.assertEqual(len(rules), 2)
        self.assertIn('intersectsArea', rules[0].unsupported)
        self.assertIn('Length', rules[1].unsupported)


class Conditions(unittest.TestCase):
    def ev(self, src, a, b=None, layer=None):
        from design_rules import _eval
        return _eval(parse_condition(src), a, b, layer)

    def test_netclass_forms(self):
        a = _item(classes={'hs50'}, eff='hs50')
        d = _item(net_id=2)
        self.assertTrue(self.ev("A.NetClass == 'hs50'", a))
        self.assertFalse(self.ev("A.NetClass == 'hs50'", d))
        self.assertTrue(self.ev("A.hasNetclass('hs50')", a))
        self.assertTrue(self.ev("A.hasExactNetclass('hs50')", a))
        self.assertTrue(self.ev("A.NetClass != 'hs50'", d))
        self.assertTrue(self.ev("A.NetClass == 'Default'", d))
        self.assertTrue(self.ev("B.hasExactNetclass('Default')", a, d))

    def test_net_and_netname(self):
        a = _item(1, net_name='GND')
        b = _item(2, net_name='VCC')
        self.assertTrue(self.ev("A.Net != B.Net", a, b))
        self.assertFalse(self.ev("A.Net != B.Net", a, a))
        self.assertTrue(self.ev("A.NetName == 'GND'", a))
        self.assertTrue(self.ev("B.NetName != 'GND'", a, b))

    def test_type_and_via_and_pad(self):
        t = _item(type='track')
        v = _item(type='via', via_type='micro')
        p = _item(type='pad', pad_type='smd')
        self.assertTrue(self.ev("A.Type == 'Track'", t))
        self.assertTrue(self.ev("A.Type == 'track'", t))
        self.assertTrue(self.ev("A.Type == 'Via' && A.isMicroVia()", v))
        self.assertFalse(self.ev("A.isMicroVia()", t))
        self.assertTrue(self.ev("A.Type == 'Via' && !A.isMicroVia()", _item(type='via', via_type='through')))
        self.assertTrue(self.ev("B.Pad_Type == 'SMD'", t, p))
        self.assertTrue(self.ev("(A.Type == 'Pad' || A.Type == 'Via') && (B.Type == 'Pad' || B.Type == 'Via')", p, v))

    def test_layers_and_L(self):
        a = _item(layers={'F.Cu'})
        self.assertTrue(self.ev("A.onLayer('F.Cu')", a))
        self.assertTrue(self.ev("A.existsOnLayer('F.Cu') || B.onLayer('B.Cu')", a, a))
        self.assertFalse(self.ev("A.onLayer('B.Cu')", a))
        self.assertTrue(self.ev("A.Layer == 'In1.Cu'", _item(), layer='In1.Cu'))

    def test_footprint_and_group(self):
        p = _item(type='pad', footprint_ref='C12', groups={'ESCAPE'})
        self.assertTrue(self.ev("A.memberOfFootprint('C*')", p))
        self.assertFalse(self.ev("A.memberOfFootprint('R*')", p))
        self.assertTrue(self.ev("A.memberOfGroup('ESCAPE')", p))

    def test_the_real_mez_rx_conditions_parse(self):
        ok = [
            "A.Net != B.Net",
            "A.hasNetclass('power')",
            "A.Type == 'Track' && A.hasNetclass('power')",
            "A.Type == 'Track' && A.NetName == 'GND'",
            "A.Type == 'Via'",
            "A.Type == 'Via' && !A.isMicroVia()",
            "A.Type == 'Track' || A.Type == 'Pad'",
            "(A.Type == 'Pad' || A.Type == 'Via') && (B.Type == 'Pad' || B.Type == 'Via')",
            "A.Type == 'Zone' && A.Layer == '*.Cu'",
            "A.Type == 'Track' && B.Type == 'Track' && (A.hasNetclass('hs50') || A.hasNetclass('diff100')) "
            "&& !(B.hasNetclass('hs50') || B.hasNetclass('diff100')) && B.NetName != 'GND'",
            "A.NetClass == 'loosediffpair' && B.NetClass == 'Default'",
            "A.NetClass == '90R_DP' && B.NetName == 'GND'",
            "A.Type == 'Via' && B.Pad_Type == 'SMD' && (B.memberOfFootprint('*0402*') || B.memberOfFootprint('*0603*'))",
        ]
        for src in ok:
            parse_condition(src)
        for src in ("A.intersectsArea('underFPGA') || A.intersectsArea('underDDR')",
                    "A.inDiffPair('*')",
                    "A.NetClass == 'DDR4_CMD' && A.fromTo('IC14-*','IC13-*')",
                    "(A.Type == 'Track' && A.memberOfGroup('X')) || B.Length > 1mm"):
            with self.assertRaises(Unsupported):
                parse_condition(src)


class Precedence(unittest.TestCase):
    """One test per tier boundary of drc_engine.cpp::EvalRules."""

    def test_netclass_clearance_is_pairwise_max_of_the_two_nets(self):
        dr = _rules(classes={'Default': {'clearance': 0.2}, 'HV': {'clearance': 1.0, 'priority': 0}},
                    memberships={2: {'HV'}})
        a, b = dr.item_for_net(1), dr.item_for_net(2)
        self.assertEqual(dr.resolve('clearance', a, b).min, 1.0)
        self.assertEqual(dr.resolve('clearance', a, a).min, 0.2)
        self.assertIn('HV', dr.resolve('clearance', a, b).source)

    def test_board_min_clearance_floors_a_class_value(self):
        # harness row board_min_clearance_floors_class (KiCad 10.0.0: AGREE)
        dr = _rules(board_min={'min_clearance': 0.2}, classes={'Default': {'clearance': 0.1}})
        self.assertEqual(dr.resolve('clearance', dr.item_for_net(1), dr.item_for_net(1)).min, 0.2)

    def test_board_min_clearance_does_NOT_floor_an_explicit_rule(self):
        # harness row board_min_clearance_vs_rule: KiCad 10.0.0 enforces the
        # 0.12 rule under a 0.25 board minimum (measured, bisected to ~0.119).
        dr = _rules(board_min={'min_clearance': 0.25},
                    classes={'Default': {'clearance': 0.2}},
                    dru='(version 1)(rule "r" (constraint clearance (min 0.12mm)))')
        self.assertEqual(dr.resolve('clearance', dr.item_for_net(1), dr.item_for_net(1)).min, 0.12)

    def test_board_min_clearance_floors_a_pad_override(self):
        # harness row pad_override_below_board_min (KiCad 10.0.0: AGREE)
        dr = _rules(board_min={'min_clearance': 0.15}, classes={'Default': {'clearance': 0.2}})
        pad = _item(2, type='pad', clearance_override=0.1)
        self.assertEqual(dr.resolve('clearance', dr.item_for_net(1), pad).min, 0.15)

    def test_size_board_minimum_is_overridden_by_a_later_rule_in_either_direction(self):
        # KiCad: TRACK_WIDTH has NO post-loop floor; the board minimum is only
        # the first rule in the vector.
        dr = _rules(board_min={'min_track_width': 0.2},
                    dru='(version 1)(rule "neck" (constraint track_width (min 0.1mm)))')
        self.assertEqual(dr.resolve('track_width', dr.item_for_net(1)).min, 0.1)
        dr2 = _rules(board_min={'min_track_width': 0.2},
                     dru='(version 1)(rule "fat" (constraint track_width (min 0.5mm)))')
        self.assertEqual(dr2.resolve('track_width', dr2.item_for_net(1)).min, 0.5)
        # ...and with no rule the board minimum stands.
        self.assertEqual(_rules(board_min={'min_track_width': 0.2})
                         .resolve('track_width', _item()).min, 0.2)

    def test_netclass_width_is_opt_not_min(self):
        dr = _rules(board_min={'min_track_width': 0.127},
                    classes={'Default': {'track_width': 0.25}})
        c = dr.resolve('track_width', dr.item_for_net(1))
        self.assertEqual((c.min, c.opt), (0.127, 0.25))
        self.assertEqual(dr.draw_size('track_width', 1), 0.25)

    def test_rules_apply_per_field_last_wins(self):
        dr = _rules(dru='''(version 1)
            (rule "a" (constraint track_width (min 0.1mm) (opt 0.3mm)))
            (rule "b" (constraint track_width (opt 0.15mm)))''')
        c = dr.resolve('track_width', dr.item_for_net(1))
        self.assertEqual((c.min, c.opt), (0.1, 0.15))      # b overrode opt, kept a's min
        self.assertEqual(c.source, 'rule "b"')

    def test_layer_clause_scopes_a_rule(self):
        dr = _rules(copper=('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'),
                    dru='''(version 1)
            (rule "in" (layer inner) (constraint track_width (opt 0.11mm)))
            (rule "out" (layer outer) (constraint track_width (opt 0.2mm)))''')
        a = dr.item_for_net(1)
        self.assertEqual(dr.resolve('track_width', a, None, 'In1.Cu').opt, 0.11)
        self.assertEqual(dr.resolve('track_width', a, None, 'F.Cu').opt, 0.2)
        self.assertIsNone(dr.resolve('track_width', a, None, None).opt)

    def test_conditioned_rule_binds_only_the_matching_pair(self):
        dr = _rules(classes={'Default': {'clearance': 0.15}, 'hs': {'clearance': 0.15, 'priority': 0}},
                    memberships={7: {'hs'}}, net_names={7: 'CLK', 8: 'GND', 9: 'X'},
                    dru='''(version 1)
            (rule "iso" (constraint clearance (min 0.6mm))
              (condition "A.Type == 'Track' && B.Type == 'Track' && A.hasNetclass('hs') && !B.hasNetclass('hs') && B.NetName != 'GND'"))''')
        hs, gnd, x = dr.item_for_net(7), dr.item_for_net(8), dr.item_for_net(9)
        self.assertEqual(dr.resolve('clearance', hs, x).min, 0.6)
        self.assertEqual(dr.resolve('clearance', hs, gnd).min, 0.15)   # GND excluded
        self.assertEqual(dr.resolve('clearance', x, gnd).min, 0.15)
        self.assertEqual(dr.resolve('clearance', hs, dr.item_for_net(9, 'pad')).min, 0.15)  # pads not tracks

    def test_pad_override_replaces_rules_and_floors_at_board_min(self):
        # drc_engine.cpp: GetClearanceOverrides -> max(overrides), floored at
        # m_MinClearance, RETURN before net classes and custom rules.
        dr = _rules(board_min={'min_clearance': 0.05},
                    classes={'Default': {'clearance': 0.3}},
                    dru='(version 1)(rule "r" (constraint clearance (min 0.4mm)))')
        pad = _item(2, type='pad', clearance_override=0.1)
        trk = dr.item_for_net(1)
        c = dr.resolve('clearance', trk, pad)
        self.assertEqual((c.min, c.source), (0.1, 'pad override'))
        low = _item(3, type='pad', clearance_override=0.01)
        self.assertEqual(dr.resolve('clearance', trk, low).min, 0.05)   # board min floor
        two = _item(4, type='pad', clearance_override=0.5)
        self.assertEqual(dr.resolve('clearance', pad, two).min, 0.5)    # max of the two

    def test_zone_local_clearance_is_max_with_rules(self):
        dr = _rules(classes={'Default': {'clearance': 0.3}})
        z_small = _item(2, type='zone', local_clearance=0.2)
        z_big = _item(3, type='zone', local_clearance=0.5)
        t = dr.item_for_net(1)
        self.assertEqual(dr.resolve('clearance', t, z_small).min, 0.3)
        self.assertEqual(dr.resolve('clearance', t, z_big).min, 0.5)

    def test_diff_pair_gap_is_floored_at_min_clearance(self):
        dr = _rules(board_min={'min_clearance': 0.15}, classes={'Default': {'diff_pair_gap': 0.1}})
        c = dr.resolve('diff_pair_gap', dr.item_for_net(1))
        self.assertEqual((c.min, c.opt), (0.15, 0.1))
        self.assertEqual(dr.draw_size('diff_pair_gap', 1), 0.15)

    def test_aggregate_class_takes_each_property_from_the_highest_priority_member(self):
        dr = _rules(classes={'Default': {'clearance': 0.2, 'track_width': 0.2},
                             'A': {'clearance': 0.5, 'priority': 1},
                             'B': {'clearance': 0.3, 'track_width': 0.4, 'priority': 0}},
                    memberships={1: {'A', 'B'}})
        self.assertEqual(dr.effective_class(1), 'B')
        self.assertEqual(dr.class_value(1, 'clearance'), 0.3)   # B outranks A
        self.assertEqual(dr.class_value(1, 'track_width'), 0.4)
        dr2 = _rules(classes={'Default': {'clearance': 0.2, 'track_width': 0.2},
                              'A': {'clearance': 0.5, 'priority': 1}},
                     memberships={1: {'A'}})
        self.assertEqual(dr2.class_value(1, 'track_width'), 0.2)  # falls to Default

    def test_fab_floor_is_raise_only_and_disclosed(self):
        dr = _rules(board_min={'min_track_width': 0.05}, fab={'track_width': 0.127})
        c = dr.resolve('track_width', dr.item_for_net(1))
        self.assertEqual((c.min, c.kicad_min, c.fab_bound), (0.127, 0.05, True))
        dr2 = _rules(board_min={'min_track_width': 0.2}, fab={'track_width': 0.127})
        c2 = dr2.resolve('track_width', dr2.item_for_net(1))
        self.assertEqual((c2.min, c2.fab_bound), (0.2, False))

    def test_disallow_collects_the_matching_rules(self):
        dr = _rules(dru='''(version 1)
            (rule "nb" (constraint disallow buried_via))
            (rule "nm" (constraint disallow micro_via) (condition "A.Type == 'Via'"))''')
        self.assertEqual(dr.resolve('disallow', _item(type='via')).disallow,
                         frozenset({'buried_via', 'micro_via'}))
        self.assertEqual(dr.resolve('disallow', _item(type='track')).disallow,
                         frozenset({'buried_via'}))

    def test_stack_resolution_takes_the_strictest_layer(self):
        dr = _rules(copper=('F.Cu', 'In1.Cu', 'B.Cu'),
                    classes={'Default': {'clearance': 0.1}},
                    dru='(version 1)(rule "i" (layer inner) (constraint clearance (min 0.3mm)))')
        a, b = dr.item_for_net(1), dr.item_for_net(2)
        self.assertEqual(dr.resolve_stack('clearance', a, b, ['F.Cu', 'In1.Cu', 'B.Cu']).min, 0.3)
        self.assertEqual(dr.resolve_stack('clearance', a, b, ['F.Cu', 'B.Cu']).min, 0.1)

    def test_draw_size_precedence_cli_over_rule_opt_over_class(self):
        dru = '(version 1)(rule "z" (constraint track_width (opt 0.11mm) (max 0.3mm)))'
        dr = _rules(classes={'Default': {'track_width': 0.25}}, dru=dru)
        self.assertEqual(dr.draw_size('track_width', 1), 0.11)
        dr_cli = _rules(classes={'Default': {'track_width': 0.25}}, dru=dru, cli={'track_width': 0.2})
        self.assertEqual(dr_cli.draw_size('track_width', 1), 0.2)
        dr_max = _rules(classes={'Default': {'track_width': 0.25}}, dru=dru, cli={'track_width': 0.8})
        self.assertEqual(dr_max.draw_size('track_width', 1), 0.3)     # clamped to rule max
        self.assertEqual(_rules().draw_size('track_width', 1, default=0.3), 0.3)
        self.assertIsNone(_rules().draw_size('track_width', 1))


class Loader(unittest.TestCase):
    def test_from_project_reads_minimums_classes_patterns_and_dru(self):
        with tempfile.TemporaryDirectory() as tmp:
            pcb = os.path.join(tmp, 'b.kicad_pcb')
            open(pcb, 'w').write('(kicad_pcb (version 20240108) (layers (0 "F.Cu" signal) (31 "B.Cu" signal)))')
            json.dump({'board': {'design_settings': {'rules': {
                           'min_clearance': 0.1, 'min_track_width': 0.127, 'min_via_diameter': 0.0}}},
                       'net_settings': {'classes': [
                           {'name': 'Default', 'clearance': 0.2, 'track_width': 0.25,
                            'via_diameter': 0.6, 'via_drill': 0.3, 'priority': 2147483647},
                           {'name': 'power', 'clearance': 0.3, 'track_width': 0.5, 'priority': 0}],
                           'netclass_patterns': [{'pattern': 'V*', 'netclass': 'power'}],
                           'netclass_assignments': {'GND': ['power']}}},
                      open(os.path.join(tmp, 'b.kicad_pro'), 'w'))
            open(os.path.join(tmp, 'b.kicad_dru'), 'w').write(
                '(version 1)(rule "p" (constraint track_width (min 0.2mm)) (condition "A.hasNetclass(\'power\')"))')
            pcb_data = SimpleNamespace(
                nets={1: SimpleNamespace(name='VCC'), 2: SimpleNamespace(name='GND'),
                      3: SimpleNamespace(name='SIG')},
                board_info=SimpleNamespace(copper_layers=['F.Cu', 'B.Cu']),
                groups={}, source_path=pcb)
            dr = DesignRules.from_project(pcb_data, pcb)
        self.assertEqual(dr.board_min, {'min_clearance': 0.1, 'min_track_width': 0.127})
        self.assertEqual(dr.memberships, {1: frozenset({'power'}), 2: frozenset({'power'})})
        self.assertEqual(dr.effective_class(3), 'Default')
        self.assertEqual(dr.draw_size('track_width', 1), 0.5)
        self.assertEqual(dr.draw_size('track_width', 3), 0.25)
        self.assertEqual(dr.floor('track_width', 1), 0.2)          # the rule
        self.assertEqual(dr.floor('track_width', 3), 0.127)        # board minimum
        self.assertEqual(dr.resolve('clearance', dr.item_for_net(1), dr.item_for_net(3)).min, 0.3)
        self.assertEqual(dr.draw_size('via_diameter', 3), 0.6)
        self.assertEqual(dr.draw_size('hole_size', 3), 0.3)
        self.assertEqual(dr.table()['rules'][0]['name'], 'p')
        self.assertEqual(dr.unsupported(), [])

    def test_a_board_with_nothing_declares_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            pcb = os.path.join(tmp, 'n.kicad_pcb')
            open(pcb, 'w').write('(kicad_pcb (version 20240108))')
            pcb_data = SimpleNamespace(nets={}, board_info=SimpleNamespace(copper_layers=[]),
                                       groups={}, source_path=pcb)
            dr = DesignRules.from_project(pcb_data, pcb)
        self.assertEqual(dr.board_min, {})
        self.assertEqual(list(dr.classes), ['Default'])
        self.assertIsNone(dr.floor('clearance', 1))
        self.assertIsNone(dr.draw_size('track_width', 1))


if __name__ == '__main__':
    unittest.main()
