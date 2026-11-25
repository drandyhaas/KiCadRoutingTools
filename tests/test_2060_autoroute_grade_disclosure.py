#!/usr/bin/env python3
"""#2060: autoroute's DRC count says what it is not.

`autoroute.py --board-rules` printed `"drc": 0` while kicad-cli, grading the
output against the INPUT's own rules, found clearance violations. Two of the
causes were the grade's, not the copper's:

- --board-rules narrows a net that cannot complete at its class down to the
  board's Board Setup minimums (--escalation board, as its help says). Such a
  descent lowers the class in the output project (#1160's record, #1492's per
  class), and check_drc grades the board at the class it ships with, so the
  count is at the LOWERED class. The summary said nothing of it.
- check_drc named some `.kicad_dru` rules NOT GRADED (its --json
  `not_graded` rows, #1831). KiCad still grades them; the summary did not
  carry them.

Invariants:
1. `Chain.grade` carries `ungraded_rules` (how many rules check_drc named NOT
   GRADED) and `ungraded_rule_names`; a rule it does grade -- the
   `A.hasNetclass(..) && A.Net != B.Net` form #2055 fixed -- is not counted.
2. `relaxed_rules` lists each class the shipped project carries below the
   clearance an automatic descent started from, Default first, with
   `below_board_min` when it sits below the chain input's own minimum
   clearance; a class raised back since is not listed.
3. `drc_phrase` says `N DRC at relaxed rules (Default 0.2 -> 0.15, ...)` and
   `; K .kicad_dru rule(s) NOT GRADED (...)`.
4. End to end (`--finish` on a board with nothing open, so no pass runs):
   AUTOROUTE_SUMMARY carries the three keys, and the run's last prose line
   says the count is at relaxed rules and names the ungraded rules.

    python3 tests/test_2060_autoroute_grade_disclosure.py
"""
import json
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'oracle'))

import autoroute as AR                                        # noqa: E402
from constraint_agreement import write_board                  # noqa: E402
from run_utils import check as run_check, evidence            # noqa: E402

SCRIPT = os.path.join(REPO, 'py_router', 'autoroute.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + str(detail)) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


#: A graded rule (#2055's form) and two check_drc cannot read.
DRU = '\n'.join([
    '(rule "hv" (constraint clearance (min 0.6mm)) '
    '(condition "A.hasNetclass(\'HV\') && A.Net != B.Net"))',
    '(rule "sheet" (constraint clearance (min 0.3mm)) (condition "A.memberOfSheet(\'/x\')"))',
    '(rule "plated" (constraint clearance (min 0.3mm)) (condition "A.isPlated()"))'])
HV = [{'name': 'HV', 'clearance': 0.2, 'priority': 0}]


def _board(path, default_now=0.15, record=True, min_clearance=0.2, other=None):
    """Two tracks, no pads (so nothing is open); the project's Default class
    at DEFAULT_NOW, and, with RECORD, the #1160 record of a descent from 0.2."""
    write_board(path, classes=HV, patterns=[('A', 'HV')], dru=DRU,
                rules={'min_clearance': min_clearance},
                segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1), (5, 12, 15, 12, 0.2, 'F.Cu', 2)])
    pro = path[:-len('.kicad_pcb')] + '.kicad_pro'
    proj = json.load(open(pro))
    for c in proj['net_settings']['classes']:
        if c['name'] == 'Default':
            c['clearance'] = default_now
        elif c['name'] in (other or {}):
            c['clearance'] = other[c['name']]['to']    # the class as lowered
    ns = {}
    if record:
        ns['class_clearance_relaxed'] = {'from': 0.2, 'to': 0.15, 'nets': ['B']}
    if other:
        ns['class_clearance_relaxed_classes'] = other
    if ns:
        proj['kicad_routing_tools'] = ns
    json.dump(proj, open(pro, 'w'), indent=2)
    return path


def grade(td):
    print('1. the grade counts the rules check_drc did not grade')
    b = _board(os.path.join(td, 'g.kicad_pcb'))
    ch = AR.Chain(b, td, clearance=0.2, baseline=b)
    g = ch.grade(b)
    check('the board grades', g is not None)
    if g is None:
        return
    check('ungraded_rules counts the two rules check_drc cannot read',
          g.get('ungraded_rules') == 2, g.get('ungraded_rules'))
    check('ungraded_rule_names names them, and not the HV rule (#2055)',
          g.get('ungraded_rule_names') == ['plated', 'sheet'], g.get('ungraded_rule_names'))
    rel = g.get('relaxed_rules')
    check('relaxed_rules carries the Default class the descent lowered',
          rel == [{'class': 'Default', 'from': 0.2, 'to': 0.15, 'nets': ['B'],
                   'below_board_min': 0.2}], rel)


def relaxed(td):
    print('2. relaxed_rules reads the records the shipped project carries')
    if not hasattr(AR, 'relaxed_rules'):
        check('autoroute.relaxed_rules exists', False)
        return
    b = _board(os.path.join(td, 'r1.kicad_pcb'), default_now=0.2)
    check('a Default class raised back to its `from` is not listed',
          AR.relaxed_rules(b) == [], AR.relaxed_rules(b))
    b = _board(os.path.join(td, 'r2.kicad_pcb'), record=False)
    check('no record, nothing listed', AR.relaxed_rules(b) == [], AR.relaxed_rules(b))
    b = _board(os.path.join(td, 'r3.kicad_pcb'),
               other={'HV': {'from': 0.2, 'to': 0.12, 'nets': ['A']}})
    rel = AR.relaxed_rules(b, board_min=0.15)
    check('the Default class first, then each other class; below_board_min only '
          'where the class sits under the input minimum',
          [(r['class'], r['to'], r.get('below_board_min')) for r in rel]
          == [('Default', 0.15, None), ('HV', 0.12, 0.15)], rel)


def phrase():
    print('3. the count is printed with what it is not')
    if not hasattr(AR, 'drc_phrase'):
        check('autoroute.drc_phrase exists', False)
        return
    g = {'drc': 0, 'ungraded_rules': 2, 'ungraded_rule_names': ['plated', 'sheet'],
         'relaxed_rules': [{'class': 'Default', 'from': 0.2, 'to': 0.15, 'nets': ['B']}]}
    s = AR.drc_phrase(g)
    check('at relaxed rules, and the ungraded rules',
          s == '0 DRC at relaxed rules (Default 0.2 -> 0.15); '
               '2 .kicad_dru rule(s) NOT GRADED (plated, sheet)', s)
    check('a plain grade is the plain count',
          AR.drc_phrase({'drc': 3, 'ungraded_rules': 0, 'relaxed_rules': []}) == '3 DRC')
    check('a grade from before the keys existed reads as the plain count',
          AR.drc_phrase({'drc': 1}) == '1 DRC')


def end_to_end(td):
    print('4. AUTOROUTE_SUMMARY and the last line')
    b = evidence(_board(os.path.join(td, 'in.kicad_pcb')), 'input board')
    out = os.path.join(td, 'out_e2e.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', SCRIPT, b, out, '--finish'], accept=True)
    lines = r.stdout.splitlines()
    summ = [json.loads(ln.split(': ', 1)[1]) for ln in lines
            if ln.startswith('AUTOROUTE_SUMMARY: ')]
    check('one AUTOROUTE_SUMMARY', len(summ) == 1, len(summ))
    if not summ:
        return
    s = summ[0]
    check('the summary grades the shipped board with nothing open',
          s.get('open') == 0 and s.get('drc') == 0, {k: s.get(k) for k in ('open', 'drc')})
    check('the summary carries ungraded_rules and their names',
          s.get('ungraded_rules') == 2 and s.get('ungraded_rule_names') == ['plated', 'sheet'],
          {k: s.get(k) for k in ('ungraded_rules', 'ungraded_rule_names')})
    check('the summary carries the relaxed Default class',
          [(x['class'], x['from'], x['to']) for x in s.get('relaxed_rules') or []]
          == [('Default', 0.2, 0.15)], s.get('relaxed_rules'))
    last = [ln for ln in lines if ln.startswith('AUTOROUTE: shipped ')]
    check('the last prose line says the count is at relaxed rules and names the '
          'ungraded rules',
          len(last) == 1 and 'DRC at relaxed rules (Default 0.2 -> 0.15' in last[0]
          and '2 .kicad_dru rule(s) NOT GRADED (plated, sheet)' in last[0], last)


def main():
    with tempfile.TemporaryDirectory(prefix='t2060_') as td:
        grade(td)
        relaxed(td)
        phrase()
        end_to_end(td)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
