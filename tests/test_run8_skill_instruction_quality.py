#!/usr/bin/env python3
"""A skill must not hand the executor a default where it owes a measurement.

The defect this exists to catch, in the words that caused it:

    "Step 0: Placement gate (usually SKIPPED)"

That is not a neutral hint, it is an instruction, and the cheapest way to obey
it is to skip. What was being skipped was the two-command copper-free check --
the only thing in the whole chain that can see two parts stacked on each other,
because everything after it looks at copper and at that point there is none. A
board shipped with a stacked pair and every instrument green.

The general shape: a frequency word ("usually", "normally", "most of the time")
attached to a decision the tools can MEASURE. It reads as guidance and lands as
a default, and defaults win over measurements because they cost nothing.

Allowed, and deliberately so:

  * prohibitions -- "NEVER skip the assessment", "do not skip it";
  * frequency words about the WORLD rather than the executor's behaviour --
    "most boards have no fine-pitch parts" describes boards, and the driver
    computes that per board anyway;
  * "optional" for something genuinely optional (an aesthetic flag), as long
    as it is not a gate.

Run: python3 -X utf8 tests/test_run8_skill_instruction_quality.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILLS = os.path.join(ROOT, '.claude', 'skills')
OWNED = ('plan-pcb-placement', 'plan-pcb-routing',
         'plan-pcb-placement-and-routing')

FAILURES = []

# A frequency word attached to something the EXECUTOR does or does not do.
#
# The distinction that matters, and the reason this is not merely "usually"
# near "not": a claim about the WORLD is fine and often valuable ("schematic
# sheets usually are not one contiguous area" describes boards, and the tool
# measures it per board anyway). A claim about the executor's BEHAVIOUR is the
# defect -- it reads as guidance and lands as a default, and a default costs
# nothing to obey.
#
# So the second half must name an action: skipping, or not doing one of the
# verbs these skills are made of.
DEFAULT_OVER_MEASUREMENT = re.compile(
    r'\b(usually|normally|typically|generally|most of the time|in most cases|'
    r'rarely|seldom)\b[^.\n]{0,70}?'
    r'(\b(skip|skipped|skipping)\b'
    r'|\b(do not|don\'t|never|no need to|not)\s+'
    r'(run|check|measure|verify|need|use|pass|read|grade)\b'
    # "usually routes better WITHOUT a fanout" pre-decides a question the
    # stage just told the executor to measure four ways, and the verb list
    # above did not cover it: the defect is the frequency word deciding, not
    # the particular verb it decides with. Any "without/instead of/rather
    # than X" is the same shape.
    r'|\b(without|instead of|rather than)\b'
    r'|\bthe answer is no\b)', re.I)

# Hedges: they turn an instruction into a suggestion, and a suggestion into
# nothing. The drivers already ban these in their own self-tests; the skill
# bodies were never checked.
HEDGES = ('you may want to', "if you're not sure", 'if you are not sure',
          'if unsure', 'probably fine', 'no need to bother',
          "don't bother", 'do not bother')

# A prohibition is the opposite of the defect and must not trip it.
PROHIBITION = re.compile(
    r'\b(never|not|do not|don\'t|cannot|must not|refuses?|refusing)\b'
    r'[^.\n]{0,30}\b(skip|skipped|skipping|optional)\b', re.I)


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def skill_files():
    for name in OWNED:
        d = os.path.join(SKILLS, name)
        if not os.path.isdir(d):
            continue
        for root, _dirs, files in os.walk(d):
            for f in sorted(files):
                if f.endswith(('.md', '.py')):
                    yield os.path.join(root, f)


SELF_CHECK = [
    # (text, should_be_caught, why)
    ('## Step 0: Placement gate (usually SKIPPED)', True, 'the original defect'),
    ('Most of the time the answer is no, and running placement on a good board',
     True, 'the same defect in the body'),
    ('placement is normally skipped on a placed board', True, 'a paraphrase'),
    ('you typically do not need to run the lock advisor', True,
     'a different verb'),
    ('A part that passes 1 and 2 with room to spare usually routes better '
     'WITHOUT a fanout', True,
     'the same defect reached through a verb the first list did not carry'),
    ('Schematic sheets usually are not one contiguous area', False,
     'a claim about BOARDS, not behaviour'),
    ('NEVER skip the assessment', False, 'a prohibition'),
    ('most boards have no fine-pitch parts', False, 'a claim about boards'),
]


def main():
    files = list(skill_files())
    check('there are skills to lint', len(files) >= 3)

    # A lint that cannot fail is decoration. Prove it catches the sentence it
    # was written for, and leaves the legitimate ones alone.
    print('the lint itself')
    for text, want, why in SELF_CHECK:
        got = (bool(DEFAULT_OVER_MEASUREMENT.search(text))
               and not bool(PROHIBITION.search(text)))
        check(f'{"catches" if want else "allows"}: {why}', got == want, text)

    print('no frequency word standing in for a measurement')
    hits = []
    for path in files:
        rel = os.path.relpath(path, ROOT)
        for i, line in enumerate(open(path, encoding='utf-8'), 1):
            if DEFAULT_OVER_MEASUREMENT.search(line) \
                    and not PROHIBITION.search(line):
                hits.append(f'{rel}:{i}: {line.strip()[:100]}')
    check('no "usually ... skip/not" phrasing survives', not hits,
          '\n        '.join(hits[:8]))

    print('no hedges in the skill bodies')
    hedged = []
    for path in files:
        rel = os.path.relpath(path, ROOT)
        text = open(path, encoding='utf-8').read()
        low = text.lower()
        for h in HEDGES:
            start = 0
            while True:
                j = low.find(h, start)
                if j < 0:
                    break
                line = text.count('\n', 0, j) + 1
                # A driver's own self-test lists these strings in order to ban
                # them; that is the opposite of using one.
                ctx = text[max(0, j - 120):j + 60]
                if 'want(' not in ctx and 'for phrase in' not in ctx:
                    hedged.append(f'{rel}:{line}: {h!r}')
                start = j + 1
    check('no hedging phrase turns an instruction into a suggestion',
          not hedged, '\n        '.join(hedged[:8]))

    print('the gate itself is stated as mandatory')
    place = open(os.path.join(SKILLS, 'plan-pcb-placement', 'SKILL.md'),
                 encoding='utf-8').read()
    route = open(os.path.join(SKILLS, 'plan-pcb-routing', 'SKILL.md'),
                 encoding='utf-8').read()
    check('the placement gate says measure first',
          'measure first, then decide' in (place + route).lower())
    check('...and says NOT looking is never an answer',
          'never an answer is not looking' in place.lower()
          or 'never skip the assessment' in place.lower(), place[:0])
    check('...and both outcomes are named as outcomes',
          'measures clean' in place.lower() or 'both clean' in place.lower())

    print('and the frontmatter does not pre-decide it')
    for name in OWNED:
        p = os.path.join(SKILLS, name, 'SKILL.md')
        if not os.path.isfile(p):
            continue
        head = open(p, encoding='utf-8').read().split('---')[1]
        bad = DEFAULT_OVER_MEASUREMENT.search(head)
        check(f'{name} description does not say "usually not"',
              not bad, head.strip()[:160] if bad else '')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
