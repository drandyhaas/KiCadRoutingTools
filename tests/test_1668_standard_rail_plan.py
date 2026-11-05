#!/usr/bin/env python3
"""#1668: plan-pcb-routing gives a STANDARD board one rail plan and one split price.

Step 5a-tuned pours the dominant rail and lets up to six rails share a split
layer; Step 10 rule 8 and the multi-layer section's precedence note said wide
traces "no matter how many pads a rail has". autoplan follows Step 5a-tuned, so
the two passages now say what it says, and keep what rule 8 measured: outer
floods against the inner-only map. "Rail co-pours" stay DENSE-only because the
term means a rail sharing an OUTER flood, which a split on an inner layer is not.

A split layer's price was 2.5 in the STANDARD recipe and 1.0-1.5 in Step 2.

Static: reads the skill and autoplan's source, runs nothing.

    python3 -X utf8 tests/test_1668_standard_rail_plan.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
AUTOPLAN = os.path.join(ROOT, 'py_router', 'autoplan.py')
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def main():
    with open(SKILL, encoding='utf-8') as f:
        text = f.read()
    flat = re.sub(r'\s+', ' ', text)

    print('A. one rail plan')
    check('the "no matter how many pads" plan is gone', 'no matter how many pads' not in flat)
    check('the multi-layer note no longer says "+ rails as wide traces"',
          'inner-only pours + rails as wide traces' not in flat)
    rule8 = re.search(r' 8\. \*\*The Step 5a-tuned density gate.*? 9\. \*\*Never split', flat)
    check('rule 8 is there', rule8 is not None)
    r8 = rule8.group(0) if rule8 else ''
    for phrase in ('inner-only pours', 'the dominant rail poured', 'rails past six as wide traces',
                   'rule 9', 'flood boards against the inner-only map',
                   'DENSE-tier moves only', 'per-zone fragility `=0`'):
        check(f'rule 8 says "{phrase}"', phrase in r8, r8[:120] if phrase not in r8 else '')
    check('rule 8 defines a rail co-pour as sharing an outer flood',
          'a rail sharing an outer flood' in r8)
    note = re.search(r'\*\*Precedence: the Step 5a-tuned DENSITY GATE.*?Read this section', flat)
    check("the multi-layer note carries Step 5a-tuned's map",
          note is not None and 'the dominant rail poured, rails past six as wide traces' in note.group(0))
    check('the aggressive map no longer lists many-rail splits', 'many-rail splits' not in flat)

    print('B. it is the plan the tool follows')
    with open(AUTOPLAN, encoding='utf-8') as f:
        tool = f.read()
    check('autoplan pours the dominant rail', 'nets[1] = rails[0][0]' in tool)
    check('Step 5a-tuned pours the dominant rail, up to six on a split',
          'the dominant rail (most pads) on the second inner (price 2.5)' in flat
          and 'up to six rails share a split layer' in flat)

    print('C. one split price')
    for gone in ('1.0–1.5 on split', '1.5 split-or-highway'):
        check(f'"{gone}" is gone', gone not in flat)
    check('Step 2 prices a split layer 2.5', '2.5 on a split/route+pour layer' in flat
          and '2.5 a split layer' in flat)
    check('it is stated as one price on both tiers', 'A split layer has one price, 2.5, on both tiers' in flat)
    check('it says no run isolates the number', 'no run isolates this one number' in flat)
    check('Step 2c item 3 agrees', 'rail/split pour layers **2.5**' in flat)


if __name__ == '__main__':
    main()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print(f'  - {f}')
        sys.exit(1)
    print('ALL PASS')
