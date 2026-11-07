#!/usr/bin/env python3
"""#1672: the skills give one answer to "does check_complete --authored-from gate?"

On a board routed at the fab floor the form without the flag says DONE and the
form with it says UNSOUND (exit 5), by design (#1368). The review skill gated on
the flag, the verifier called it "not gating" unconditionally, the routing skill's
DONE named no form, and only the routing skill knew the decision that settles it.
The one answer: on aspirational rules the form without the flag gates and the
flag's verdict is reported; on the user's own rules the flag gates.

    python3 tests/test_1672_one_statement_of_the_floor_gate.py
"""
import os
import re
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

SKILLS = os.path.join(REPO, '.claude', 'skills')
FAILS = []


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def read(*parts):
    with open(run_utils.evidence(os.path.join(SKILLS, *parts), 'skill'),
              encoding='utf-8') as f:
        return f.read()


def t_review():
    text = read('review-routed-board', 'SKILL.md')
    sec = text[text.index('## Step 5: The Sign-Off Report'):]
    cmds = re.findall(r'^python3 -X utf8 check_complete\.py[^\n]*', sec, re.M)
    ok('both forms have a command',
       len(cmds) == 2 and sum('--authored-from' in c for c in cmds) == 1, cmds)
    ok('aspirational rules: the bare form gates, the flag is reported',
       '**Aspirational rules:** the form without the flag gates' in sec
       and 'verdict is reported' in sec, sec[:200])
    ok("the user's own rules: the flag gates",
       "**The user's own rules:** `--authored-from` gates" in sec)
    ok('it says how to learn which rules apply',
       'plan report' in sec and 'Whose rules does the project hold?' in sec
       and '--no-fix-drc-settings' in sec)


def t_verifier():
    text = read('plan-pcb-placement', 'references', 'verifier.md')
    ok('the brief is told whose rules', '**Whose rules the project holds' in text)
    line = next(ln for ln in text.splitlines()
                if '--authored-from <INPUT BOARD>`' in ln)
    ok('the verifier carries both cases',
       'on aspirational rules **report it, not gating**' in line
       and "on the user's own rules **it gates**" in line, line[:300])
    ok('on the rules where it gates, it carries the spec flags', '<SPEC FLAGS>' in line,
       line[:200])


def t_routing():
    text = read('plan-pcb-routing', 'SKILL.md')
    sec = text[text.index('### Stop conditions'):]
    sec = sec[:sec.index('"This is taking a long time"')]
    ok('the stop condition names the form, by the rule decision',
       '--authored-from' in sec and 'Whose rules' in sec, sec[:400])
    ok('it cites the commands rather than restating them',
       '/review-routed-board' in sec and 'python3' not in sec, sec[:400])


def main():
    t_review()
    t_verifier()
    t_routing()
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
