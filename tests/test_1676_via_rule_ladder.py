#!/usr/bin/env python3
"""#1676: the vias a default run lays start at the board's own `.kicad_dru` via
rules, and a rule a run breaks is named.

Under the default escalation the vias were the same with and without the rules,
so `via_diameter` min 0.45 and `hole_size` min 0.2 were broken by 0.25/0.15 vias
in silence, and under `--escalation board` every via was under an `annular_width`
min 0.127 rule. The decided policy stands: completion first, disclosed, and a
descent may go below a declared minimum to the tier floor. So:

  1. `via_floors` reads the rule file as KiCad does: a rule's condition decides
     which vias it binds, a project severity of `ignore` retires it, and a kind
     no rule sets is absent. `via_meeting_rules` is raise-only.
  2. the ladder under `fab`: the first rung meets the rules, the tier's own rungs
     follow it, so going below a rule is the last thing tried; under `board` no
     rung is below them; with no rule file it is exactly the tier's; `off` has no
     ladder.
  3. `rule_floors` carries a net-conditioned rule per net.
  4. the end-of-step grade names each rule: `via_size_grade`, its lines, and
     `disclose_step_rules`, which JSON_SUMMARY.design_rules carries.
  5. the working via `list_nets` and autoplan hand a step is the smallest the
     rule file accepts, replacing the Board Setup minimum of its kind.
  6. routed end to end on a small board: the rule arm starts at the raised via;
     a control arm with no rule file keeps the via it had, and every rule is
     named per rule at the end.

    python3 tests/test_1676_via_rule_ladder.py
"""
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)

from run_utils import evidence  # noqa: E402
import fab_tiers  # noqa: E402
from design_rules import (DesignRules, disclose_step_rules, via_size_grade_lines)  # noqa: E402
from list_nets import effective_floors  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-u', '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')

#: The via rules of a fab's 4-layer rule file.
FAB_RULES = '''(version 1)
(rule "Min. Via hole size" (constraint hole_size (min 0.2mm)) (condition "A.Type == 'Via'"))
(rule "Min. Via Diameter" (constraint via_diameter (min 0.45mm)) (condition "A.Type == 'Via'"))
(rule "Minimum annular ring" (constraint annular_width (min 0.127mm))
  (condition "A.Type != 'Pad'"))
'''


def rules_for(td, dru, severities=None):
    stem = os.path.join(td, 'b')
    with open(stem + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': {},
                                                 'rule_severities': severities or {}}},
                   'net_settings': {'classes': [{'name': 'Default', 'via_diameter': 0.5,
                                                 'via_drill': 0.3},
                                                {'name': 'HV', 'priority': 0}],
                                    'meta': {'version': 3},
                                    'netclass_patterns': [{'pattern': 'HV*', 'netclass': 'HV'}]}},
                  f)
    with open(stem + '.kicad_dru', 'w', encoding='utf-8') as f:
        f.write(dru)
    pcb = SimpleNamespace(nets={1: SimpleNamespace(name='A'), 2: SimpleNamespace(name='HV1')},
                          board_info=SimpleNamespace(copper_layers=['F.Cu', 'B.Cu']),
                          groups={}, footprints={}, source_path=stem + '.kicad_pcb')
    return DesignRules.from_project(pcb, stem + '.kicad_pcb')


def via(net=1, size=0.5, drill=0.3):
    return SimpleNamespace(size=size, drill=drill, net_id=net, x=1.0, y=1.0)


def pairs(rungs):
    return [(round(r['via_diameter'], 3), round(r['via_drill'], 3)) for r in rungs]


def ladder(policy, floors=None, extra=None, n=4):
    fab_tiers.set_default_fab_tier('auto')
    fab_tiers.set_escalation_policy(policy, {})
    fab_tiers.set_rule_via_floors(floors or {})
    try:
        return pairs(fab_tiers.escalation_rungs(n, extra_floors=extra))
    finally:
        fab_tiers.set_rule_via_floors({})
        fab_tiers.set_escalation_policy('fab', {})


def main():
    evidence(BOARD, 'small board fixture')
    with tempfile.TemporaryDirectory(prefix='krt1676_') as td:
        print('1. which vias a rule binds')
        dr = rules_for(td, FAB_RULES)
        floors = dr.via_floors(1)
        assert floors == {'via_diameter': 0.45, 'via_drill': 0.2, 'annular': 0.127}, floors
        assert dr.via_meeting_rules(0.5, 0.3) == (0.554, 0.3)
        assert dr.via_meeting_rules(0.25, 0.15) == (0.454, 0.2)
        assert dr.via_meeting_rules(0.635, 0.3048) == (0.635, 0.3048), 'raise-only'

        pad_only = rules_for(td, '''(version 1)
(rule p (constraint annular_width (min 0.3mm)) (condition "A.Type == 'Pad'"))
(rule q (constraint hole_size (min 0.5mm)) (condition "A.Type == 'Pad' && !A.isPlated()"))
''')
        assert pad_only.via_floors(1) == {}, 'a pad rule binds no via'

        cls = rules_for(td, '''(version 1)
(rule hv (constraint via_diameter (min 0.8mm)) (condition "A.NetClass == 'HV'"))
''')
        assert cls.via_floors(1) == {} and cls.via_floors(2) == {'via_diameter': 0.8}
        off = rules_for(td, FAB_RULES, {'annular_width': 'ignore', 'drill_out_of_range': 'ignore'})
        assert off.via_floors(1) == {'via_diameter': 0.45}, 'KiCad flags nothing for those'
        print('   PASS')

        print('2. the ladder')
        tier = ladder('fab')
        assert tier == [(0.45, 0.2), (0.3, 0.15), (0.25, 0.15)], tier
        got = ladder('fab', floors)
        assert got[0] == (0.454, 0.2) and got[-2:] == [(0.3, 0.15), (0.25, 0.15)], got
        assert got == sorted(got, reverse=True), 'largest first: a descent is the last thing tried'
        big = ladder('fab', {'via_diameter': 0.55, 'via_drill': 0.25})
        assert big == [(0.55, 0.25)] + tier, big
        unchanged = ladder('fab', {'via_diameter': 0.4})
        assert unchanged == tier, 'a rule the first rung meets moves nothing'
        boardl = ladder('board', floors)
        assert all(d >= 0.45 and h >= 0.2 and (d - h) / 2 >= 0.127 - 1e-9 for d, h in boardl), boardl
        assert ladder('off', floors) == [], 'off has no descent'
        two = ladder('fab', floors, n=2)
        assert two[0] == (0.454, 0.2), two
        print(f'   PASS: {got}')

        print('3. a net-conditioned rule is per net')
        from routing_config import GridRouteConfig
        fab_tiers.set_escalation_policy('fab', {})
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
        cfg.rules = cls
        assert cfg.rule_floors(1) == {}, 'the rule does not bind this net'
        extra = cfg.rule_floors(2)
        assert extra == {'first_via_diameter': 0.8}, extra
        assert ladder('fab', None, extra)[0] == (0.8, 0.2), 'the net starts at its rule'
        print('   PASS')

        print('4. the end-of-step grade names each rule')
        vias = [via(), via(size=0.25, drill=0.15), via(size=0.454, drill=0.2)]
        rows = dr.via_size_grade(vias)
        assert [(r['rule'], r['kind'], r['vias'], r['below']) for r in rows] == [
            ('Min. Via Diameter', 'via_diameter', 3, 1),
            ('Min. Via hole size', 'hole_size', 3, 1)], rows
        lines = via_size_grade_lines(rows)
        assert "'Min. Via Diameter' (via_diameter min 0.45 mm): this step's 3 via(s) it binds: " \
               "1 BELOW it (smallest 0.25 mm)" in lines[0], lines
        assert 'check_drc VIA-SIZE' in lines[0] and 'check_drc VIA-DRILL-SIZE' in lines[1], lines
        met = via_size_grade_lines(dr.via_size_grade([via(size=0.454, drill=0.2)]))
        assert all('meet it' in ln and 'BELOW' not in ln for ln in met), met
        pcb = SimpleNamespace(nets={}, board_info=SimpleNamespace(
            copper_layers=['F.Cu', 'B.Cu'], board_outlines=[], board_outline=None,
            board_bounds=None, board_cutouts=[], board_edge_contours=[]),
            groups={}, footprints={}, source_path=dr.source, segments=[], vias=[])
        out = disclose_step_rules(dr, pcb, [{'new_vias': vias, 'new_segments': []}])
        assert set(out) == {'annular_rules', 'via_rules'}, out
        assert out['annular_rules'][0]['below'] == 2, out
        print('   PASS')

        print('5. the working via')
        constraints = {'min_through_hole_diameter': 0.3, 'min_via_annular_width': 0.05,
                       'min_via_diameter': 0.4}
        eff = effective_floors(constraints, 4)
        assert (eff['working_via_diameter'], eff['working_via_drill']) == (0.45, 0.3), eff
        eff = effective_floors(constraints, 4, floors)
        assert (eff['working_via_diameter'], eff['working_via_drill']) == (0.454, 0.2), eff
        print('   PASS')

        print('6. routed')
        results = {}
        for arm, dru in (('ctl', None), ('rule', FAB_RULES)):
            src = os.path.join(td, arm + '.kicad_pcb')
            subprocess.run(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, src],
                           capture_output=True, text=True, timeout=60, check=True)
            if dru:
                with open(os.path.join(td, arm + '.kicad_dru'), 'w', encoding='utf-8') as f:
                    f.write(dru)
            out = os.path.join(td, arm + '_out.kicad_pcb')
            r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'route.py'), src, out,
                                     '--nets', '*'], capture_output=True, text=True, timeout=280,
                               check=False)
            assert os.path.isfile(out), r.stdout[-1500:] + r.stderr[-500:]
            results[arm] = r.stdout
        assert 'raised to' not in results['ctl'], 'no rule file: the via is what it was'
        assert '.kicad_dru rule' not in results['ctl'], 'no rule file: nothing to name'
        assert 'Via 0.5/0.3 raised to 0.554/0.3' in results['rule'], results['rule'][:1500]
        js = json.loads(next(ln for ln in reversed(results['rule'].splitlines())
                             if ln.startswith('JSON_SUMMARY:'))[len('JSON_SUMMARY:'):])
        dr_js = js['design_rules']
        named = {r['rule'] for r in dr_js.get('via_rules', [])} | \
            {r['rule'] for r in dr_js.get('annular_rules', [])}
        assert named == {'Min. Via hole size', 'Min. Via Diameter', 'Minimum annular ring'}, named
        for rule in named:
            assert f"rule '{rule}' (" in results['rule'], rule
        print(f"   PASS: {dr_js['via_rules']}")
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
