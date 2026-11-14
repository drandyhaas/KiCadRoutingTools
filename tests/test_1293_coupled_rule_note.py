#!/usr/bin/env python3
"""A coupled-pair clearance rule is not announced as skipped.

A `.kicad_dru` clearance rule that ANDs in `AB.isCoupledDiffPair()` (or
`A.` / `B.`, #1874) binds a diff pair's P against its own N. The pair step
builds the pair at it and check_drc grades it (#1293, the coupled-pair
channel). The per-layer clearance reader, which every routing step prints the
notes of, does not handle that shape itself -- and said so with the line it
uses for a rule nothing reads: "has a non-layer scope ... skipped, KiCad will
still enforce it". A user read their pair-gap rules as ignored.

Rows:
  1. the coupled shape, with `AB.`, `A.` and a `(layer outer)` clause: the
     note names the coupled-pair channel, and never says skipped;
  2. a rule whose scope nothing here reads still says skipped, and so does
     the coupled shape check_drc does not grade -- a (severity exclusion)
     rule and one on a non-copper (layer "F.SilkS") -- decided on the parsed
     rule by check_drc's own test (`r.coupled and not r.unsupported`);
  3. the rule stays in the reader's unmodelled list (check_drc filters the
     ones its coupled channel grades by name, as before).
"""
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_dru import _parse_dru_full  # noqa: E402

CU = ['F.Cu', 'In1.Cu', 'B.Cu']
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def notes(cond, layer=''):
    text = (f'(version 1)\n(rule "r" {layer}\n  (condition "{cond}")\n'
            f'  (constraint clearance (min 0.12mm)))\n')
    res = _parse_dru_full(text, CU)
    return ' | '.join(res[2]), [r[0] for r in res[3]]


def main():
    print("1. the coupled shape")
    for tag, cond, layer in (
            ('AB.', "A.hasNetclass('HS') && AB.isCoupledDiffPair()", ''),
            ('A.', "A.hasNetclass('HS') && A.isCoupledDiffPair()", ''),
            ('with (layer outer) and item types',
             "A.hasNetclass('HS') && A.isCoupledDiffPair() && A.Type == 'Track' "
             "&& B.Type == 'Track'", '(layer outer)')):
        text, skipped = notes(cond, layer)
        ok('coupled-pair channel (#1293)' in text and 'skipped' not in text,
           f"{tag}: {text[:110]}")
        ok(skipped == ['r'], f"{tag}: still in the reader's unmodelled list")
    print("2. a scope nothing here reads")
    text, skipped = notes("A.memberOfSheet('/x')")
    ok('skipped, KiCad will still enforce it' in text and skipped == ['r'],
       f"still says skipped: {text[:110]}")
    from design_rules import parse_dru
    pn = "A.hasNetclass('HS') && AB.isCoupledDiffPair()"
    for tag, extra in (('(severity exclusion)', '(severity exclusion)'),
                       ('(layer "F.SilkS")', '(layer "F.SilkS")')):
        text, skipped = notes(pn, extra)
        rule = parse_dru(f'(version 1)\n(rule "r" {extra}\n  (condition "{pn}")\n'
                         f'  (constraint clearance (min 0.12mm)))\n')[0][0]
        # the premise: check_drc's coupled channel does not grade this rule
        ok(not (rule.coupled and not rule.unsupported),
           f"{tag}: check_drc does not grade it ({rule.unsupported})")
        # MUTATION: decide on the condition alone (`_coupled_terms` of the
        # parsed condition) -> "check_drc grades it" for a rule it does not
        ok('coupled-pair channel' not in text and 'grades it' not in text
           and skipped == ['r'], f"{tag}: not named as graded: {text[:110]}")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
