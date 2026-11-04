#!/usr/bin/env python3
"""#1649: a step says whether its vias meet an annular_width rule, and --design-rules lists a relaxing rule.

route.py printed `.kicad_dru rule '<name>': NOT honoured (annular_width ...)`
for every rule with that constraint, before routing, whatever the vias were:
the same line when a relaxing rule's vias met it and when a tightening rule's
vias broke it. And `list_nets.py --design-rules`, which the skill sends the
reader to for the sizes to request, printed no `.kicad_dru` rule, so a rule
relaxing a minimum for named nets was invisible there.

Checks (no routing; the issue's own repro routed the board and graded the
same vias with check_drc and KiCad, agreeing with these rows):
  1. `DesignRules.annular_grade` counts a via against the rule that sets its
     ring and only those: a relaxing rule at 0.07 met by a 0.075 ring, a
     tightening one at 0.2 broken by it, a via on an unbound net not counted;
  2. `annular_grade_lines` says "meet it" or "BELOW it" accordingly;
  3. `list_nets.py --design-rules` names the relaxing rule, its condition and
     the Board Constraint it relaxes, and names no rule that relaxes nothing;
  4. the plan-pcb-routing own-rules text says to scope such a request and
     what the NOT honoured line means.
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
SKILL = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
PRO = {'board': {'design_settings': {'rules': {
    'min_via_annular_width': 0.13, 'min_via_diameter': 0.3, 'min_through_hole_diameter': 0.15,
    'min_clearance': 0.15, 'min_track_width': 0.15}}},
    'net_settings': {'meta': {'version': 4}, 'classes': [
        {'name': 'Default', 'clearance': 0.15, 'track_width': 0.2, 'via_diameter': 0.6,
         'via_drill': 0.3}], 'netclass_patterns': []}}


def dru(min_ring):
    return ('(version 1)\n(rule "rf ring"\n'
            "  (condition \"A.NetName == '/RTS' || A.NetName == '/DTR'\")\n"
            f'  (constraint annular_width (min {min_ring}mm)))\n'
            '(rule "loose gap" (constraint clearance (min 0.2mm)))\n')


def setup(td, min_ring):
    b = os.path.join(td, 'b.kicad_pcb')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, b], accept=True)
    with open(os.path.join(td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    with open(os.path.join(td, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(dru(min_ring))
    return evidence(b)


def grade(b):
    from kicad_parser import parse_kicad_pcb, Via
    from design_rules import DesignRules
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(b)
    ids = {n.name: nid for nid, n in pcb.nets.items()}
    rules = DesignRules.from_project(pcb, b)
    vias = [Via(0, 0, 0.3, 0.15, ['F.Cu', 'B.Cu'], ids['/RTS']),
            Via(1, 0, 0.3, 0.15, ['F.Cu', 'B.Cu'], ids['/DTR']),
            Via(2, 0, 0.41, 0.15, ['F.Cu', 'B.Cu'], ids['GND'])]
    return rules.annular_grade(vias)


def main():
    evidence(BOARD, 'fixture board')
    from design_rules import annular_grade_lines
    with tempfile.TemporaryDirectory(prefix='krt1649_') as td:
        print('1-2. relaxing rule, met')
        rows = grade(setup(td, 0.07))
        assert [(r['rule'], r['vias'], r['below'], r['smallest_ring']) for r in rows] == \
            [('rf ring', 2, 0, 0.075)], rows
        line = annular_grade_lines(rows)[0]
        assert 'meet it' in line and 'BELOW' not in line, line
        print(f'  PASS: {line}')

        print('3. --design-rules names the relaxing rule')
        r = check(PY + [os.path.join(REPO, 'py_router', 'list_nets.py'),
                        os.path.join(td, 'b.kicad_pcb'), '--design-rules'], accept=True)
        assert "RELAX a Board Constraint" in r.stdout, r.stdout[-800:]
        assert "'rf ring': annular_width min 0.07 (board min_via_annular_width 0.13)" \
            in r.stdout and "A.NetName == '/RTS'" in r.stdout, r.stdout[-800:]
        assert "'loose gap'" not in r.stdout, 'a rule at 0.2 over a 0.15 board clearance relaxes nothing'
        print('  PASS: the rule, its condition and the constraint it relaxes are printed')
    with tempfile.TemporaryDirectory(prefix='krt1649_') as td:
        print('1-2. tightening rule, broken')
        rows = grade(setup(td, 0.2))
        assert [(r['rule'], r['vias'], r['below']) for r in rows] == [('rf ring', 2, 2)], rows
        line = annular_grade_lines(rows)[0]
        assert '2 BELOW it' in line and 'smallest ring 0.075' in line, line
        print(f'  PASS: {line}')
        r = check(PY + [os.path.join(REPO, 'py_router', 'list_nets.py'),
                        os.path.join(td, 'b.kicad_pcb'), '--design-rules'], accept=True)
        assert 'RELAX a Board Constraint' not in r.stdout, 'a tightening rule listed as relaxing'
        print('  PASS: a tightening rule is not listed as relaxing')

    print('4. the skill')
    text = open(SKILL, encoding='utf-8').read()
    sec = text[text.index('### Whose rules does the project hold?'):]
    sec = sec[:sec.index('\n### ', 5)]
    for want in ('Request no size below the minimum that binds the net',
                 'RELAX a Board', '--nets <those nets>', 'no chain removes',
                 'BELOW it'):
        assert want in sec, f'the own-rules section does not say {want!r}'
    print('  PASS: the own-rules section scopes the request and reads the line')
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
