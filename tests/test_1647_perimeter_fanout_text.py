#!/usr/bin/env python3
"""#1647: the perimeter-fanout rule covers a 2-row SMD socket and says when and at which level to compare.

plan-pcb-routing Step 3 rule 3 limited the perimeter candidate to QFN/QFP,
cited only the measurement that went worse, and said neither when nor at
which level to compare; a 2-row SMD socket at 0.5 mm fell outside both
sentences (the predicate flags it). plan-pcb-placement gave the seed judge
one chain, and both failure handovers sent a part's clustered failures to
re-placement without asking whether the chain fanned it out.

Checks (text only):
  1. rule 3 names the 2-row SMD connector or socket, says to compare at the
     FULL level, when to make the comparison, and cites both directions;
  2. the fanout table has a row for that shape;
  3. plan-pcb-placement makes the comparison an arm of the seed judge, and
     its routing-failure classification asks the fanout question first;
  4. diagnose-routing-failures asks it before handing back to placement.
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILLS = os.path.join(ROOT, '.claude', 'skills')


def read(name):
    return open(os.path.join(SKILLS, name, 'SKILL.md'), encoding='utf-8').read()


def main():
    r = read('plan-pcb-routing')
    rule = r[r.index('### Does this part actually BENEFIT from fanout?'):]
    rule = rule[:rule.index('\n### ', 5)]
    for want in ('2-row SMD connector or socket at <=0.65mm', 'compare it',
                 'the FULL route is better', 'when you first route the board',
                 '4 to 1, 8 to 4, 2 to 6', 'blocking 30 vs 25',
                 'a 2-row socket at 0.5 mm'):
        assert want in rule, f'rule 3 does not say {want!r}'
    print('  PASS: rule 3 covers the socket, names the level and the moment, cites both ways')
    table = r[r.index('### Fanout Tool Selection'):]
    table = table[:table.index('\n### ', 5)]
    assert '| 2-row SMD connector or socket at <=0.65 mm |' in table, 'no table row'
    print('  PASS: the fanout table has the 2-row SMD socket row')
    p = read('plan-pcb-placement')
    assert 'the chains with and without\n   it are arms too' in p, '3.7 gives the judge one chain'
    assert 'route that arm\n    (plan-pcb-routing Step 3, rule 3)' in p, \
        'the routing classification does not ask the fanout question'
    print('  PASS: plan-pcb-placement judges the comparison as arms and asks it first')
    d = read('diagnose-routing-failures')
    row = next(ln for ln in d.splitlines() if 'Placement-shaped' in ln)
    assert 'route that arm' in row and row.index('route that arm') < row.index('stop retrying'), row
    print('  PASS: diagnose-routing-failures asks the fanout question before re-placing')
    print('test_1647_perimeter_fanout_text: all passed')


if __name__ == '__main__':
    main()
