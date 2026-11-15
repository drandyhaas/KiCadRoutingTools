#!/usr/bin/env python3
"""A `.kicad_dru` length rule may leave nets out by name: `&& A.NetName != 'x'`.

The length / skew channel read a condition only as an ||-list of net terms.
"These classes, but not those nets" -- an ||-list ANDed with
`A.NetName != '...'` factors -- was "not an ||-list of net terms", so the rule
was NOT GRADED. Worse, a rule whose scope is not read may bind any net, and
the walk is last rule first: every net an EARLIER rule would have graded
stopped at it and went ungraded too.

The fixture is tests/fixtures/1846/time_rule: nets P and Q, both in class
TIMED, 120 and 126 ps. Each variant below is one condition on a rule
`length (min 1000ps)`, which every net it binds fails; the nets kicad-cli
10.0.8 reports are in the table (measured).

Rows:
  1. each condition binds the nets KiCad's does, and nothing is NOT GRADED;
  2. KiCad's `!=` is the negation of its wildcard, case-insensitive `==`;
  3. a condition that ANDs anything else stays unsupported, as before;
  4. an earlier rule's nets are graded again when a later rule of this
     shape does not bind them.
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '1846', 'time_rule')
FAILS = []

#: condition -> the nets kicad-cli 10.0.8 reports under `length (min 1000ps)`
KICAD = {
    "A.hasNetclass('TIMED') && A.NetName != 'Q'": ['P'],
    "A.hasNetclass('TIMED') && A.NetName != 'q'": ['P'],
    "A.hasNetclass('TIMED') && A.NetName != 'Q*'": ['P'],
    "A.hasNetclass('TIMED') && A.NetName != 'P' && A.NetName != 'Q'": [],
    "(A.hasNetclass('TIMED') || A.NetName == 'Z') && A.NetName != 'P'": ['Q'],
    "A.NetName != 'Q'": ['P'],
    "A.hasNetclass('TIMED') && A.NetName != '?'": [],
}


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def grade(td, tag, dru):
    sub = os.path.join(td, tag)
    os.makedirs(sub)
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(BOARD + ext, 'the fixture'), os.path.join(sub, 'b' + ext))
    with open(os.path.join(sub, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(dru)
    out = os.path.join(sub, 'drc.json')
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                    os.path.join(sub, 'b.kicad_pcb'), '--json', out],
              **({'accept': True} if 'CLEAN' in tag else {'refuse': 'DRC VIOLATIONS', 'code': 1}))
    with open(evidence(out, 'the report'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return (sorted(v['net1'] for v in doc['items'] if v['type'] == 'length-out-of-range'),
            doc.get('not_graded') or [], (r.stdout or ''))


def rule(cond, name='PQ timing', bound='(min 1000ps)'):
    return (f'(rule "{name}"\n  (constraint length {bound})\n'
            f'  (condition "{cond}"))\n')


def main():
    from kicad_dru import parse_dru_length_rules, length_rule_matches
    with tempfile.TemporaryDirectory() as td:
        print("1. each condition binds the nets KiCad's does")
        for i, (cond, want) in enumerate(KICAD.items()):
            got, ng, _t = grade(td, f"v{i}{'' if want else 'CLEAN'}", '(version 1)\n' + rule(cond))
            ok(got == want and not ng, f"{cond}: {got}" + ('' if got == want else f" want {want}"))
        print("2. the reader")
        rules, _ = parse_dru_length_rules('(version 1)\n' + rule(
            "A.hasNetclass('TIMED') && A.NetName != 'CS*' && A.NetName != 'RESET'"),
            time_domain=True)
        r = rules[0]
        ok(r.terms == (('netclass', 'TIMED'),) and r.exclude == ('CS*', 'RESET'),
           f"the ||-list and the names left out: {r.terms} / {r.exclude}")
        ok(length_rule_matches(r, 'D0', ['TIMED']) is True
           and length_rule_matches(r, 'cs1_a', ['TIMED']) is False
           and length_rule_matches(r, 'reset', ['TIMED']) is False
           and length_rule_matches(r, 'D0', ['OTHER']) is False,
           "`!=` leaves out what the wildcard, case-insensitive `==` matches")
        print("3. anything else ANDed in stays unsupported")
        for cond in ("A.NetName == 'P' && A.Type == 'track'",
                     "A.hasNetclass('TIMED') && !A.inDiffPair('*')",
                     "A.hasNetclass('TIMED') && A.NetName != 'P' && B.NetName != 'Q'"):
            rr, _ = parse_dru_length_rules('(version 1)\n' + rule(cond), time_domain=True)
            ok(rr[0].terms is None and 'not an ||-list' in rr[0].unsupported
               and rr[0].exclude == (), f"{cond}")
        print("4. an earlier rule's nets are graded when the later rule leaves them out")
        dru = ('(version 1)\n' + rule("A.NetName == 'Q'", 'first')
               + rule("A.hasNetclass('TIMED') && A.NetName != 'Q'", 'later', '(max 500ps)'))
        got, ng, text = grade(td, 'order', dru)
        ok(got == ['Q'] and not ng and 'NOT GRADED' not in text,
           f"Q is graded by the first rule (the later one binds P only): {got}")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
