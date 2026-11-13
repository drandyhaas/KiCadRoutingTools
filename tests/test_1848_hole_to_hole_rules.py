#!/usr/bin/env python3
"""#1848: check_drc grades drill pairs at the `.kicad_dru` hole_to_hole rule
that binds them.

The drill-pair passes graded every pair at one number (the board's
min_hole_to_hole, else the fab floor) and never read a `hole_to_hole` rule.

The fixture (tests/fixtures/1848) is synthetic, vias only: Board Setup
min_hole_to_hole 0.2; through vias `TA`/`TB` 0.36 mm apart hole to hole; a
through via `TC` and a micro via `UC` 0.358 mm apart; micro vias `UA`/`UB`
0.25 mm apart. Its rules file asks 0.4 mm of every drill pair, then 0.2 mm
when either via is a micro via.

Every row of `ROWS` is one variant with the hole_to_hole pairs kicad-cli
reports on it (measured on 10.0.8 and re-measured on 10.0.6, the rows below
the first six on 10.0.6), as {(net, net): the rule named, or None for Board
Setup's value}:
  1. each row's check_drc pairs are KiCad's -- the last matching rule sets a
     pair's spacing, a via against a pad's drill follows the same rule, two
     vias whose spans share no copper layer are not graded, a rule with
     `(severity ignore)` leaves the pairs it binds ungraded, and a pair with
     a laser-drilled micro via (two adjacent layers) is held to its rule or
     Board Setup's value with no fab floor. KiCad grades no pair of two
     laser-drilled micro vias, and neither does check_drc: it counts them in
     one NOT GRADED line (`LASER_UNGRADED`). A via typed micro over more than
     two adjacent layers is a mechanical drill (#1844), and a pair of two is
     graded as such though KiCad grades it as a micro via pair
     (`MECHANICAL_ONLY`);
  2. a rule-bound finding carries the rule and its value;
  3. a rule below the fab floor is pinned up to it for two mechanical holes
     and says so (this tool's floor, not KiCad's), and a rule the resolver
     cannot read is named NOT GRADED.
"""
import json
import os
import re
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
from run_utils import evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '1848', 'hole_to_hole_rule')
DRILL_TYPES = ('via-drill-hole', 'pad-drill-via-drill', 'pad-drill-via-drill-same-net')

ONE = '(version 1)\n(rule "drill to drill"\n  (constraint hole_to_hole (min 0.4mm)))\n'
TIGHT = '(version 1)\n(rule "tight"\n  (constraint hole_to_hole (min 0.1mm)))\n'
UNREAD = ('(version 1)\n(rule "unread"\n  (constraint hole_to_hole (min 0.4mm))\n'
          '  (condition "A.getField(\'x\') == \'y\'"))\n')
PAD = ('\t(footprint "TH"\n\t\t(layer "F.Cu")\n\t\t(at 16 3)\n'
       '\t\t(property "Reference" "J1"\n\t\t\t(at 0 0 0)\n\t\t\t(layer "F.SilkS")\n\t\t)\n'
       '\t\t(pad "1" thru_hole circle\n\t\t\t(at 0 0)\n\t\t\t(size 0.5 0.5)\n'
       '\t\t\t(drill 0.3)\n\t\t\t(layers "*.Cu")\n\t\t\t(net "P1")\n\t\t)\n\t)\n'
       '\t(via\n\t\t(at 16.6 3)\n\t\t(size 0.46)\n\t\t(drill 0.2)\n'
       '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "TD")\n\t)\n')
# the second via of a pair moved to 0.16 mm from the first, on the first's net
LASER = (('(at 5.35 6)', '(at 5.26 6)'), ('(net "UB")', '(net "UA")'))
DRILLED = (('(at 13.56 6)', '(at 13.36 6)'), ('(net "TB")', '(net "TA")'))



def via(kind, x, layers, net, size=0.46, drill=0.2):
    """One via block of the fixture's spelling (y 6)."""
    return (f'(via{" " + kind if kind else ""}\n\t\t(at {x} 6)\n\t\t(size {size})\n'
            f'\t\t(drill {drill})\n\t\t(layers {" ".join(chr(34) + la + chr(34) for la in layers)})'
            f'\n\t\t(net "{net}")')


FB, F1, I12, I2B = ('F.Cu', 'B.Cu'), ('F.Cu', 'In1.Cu'), ('In1.Cu', 'In2.Cu'), ('In2.Cu', 'B.Cu')
TA0, TB0 = via('', 13, FB, 'TA'), via('', 13.56, FB, 'TB')
UA0, UB0 = via('micro', 5, F1, 'UA', 0.25, 0.1), via('micro', 5.35, F1, 'UB', 0.25, 0.1)
# TB moved to 0.16 mm from TA, on TA's net, as two blind vias
BLIND_APART = ((TA0, via('blind', 13, F1, 'TA')), (TB0, via('blind', 13.36, I2B, 'TA')))
BLIND_SHARED = ((TA0, via('blind', 13, F1, 'TA')), (TB0, via('blind', 13.36, I12, 'TA')))
THROUGH_MICRO = ((TB0, via('micro', 13.31, I2B, 'TA', 0.25, 0.1)),)
# UB stacked on UA: F.Cu-In1.Cu over In1.Cu-In2.Cu at one point
STACKED = ((UB0, via('micro', 5, I12, 'UA', 0.25, 0.1)),)
# UA/UB typed micro over the whole stack, 0.16 mm apart: mechanical drills
MECH_MICRO = ((UA0, via('micro', 5, FB, 'UA')), (UB0, via('micro', 5.36, FB, 'UA')))
TC_CLOSE = (('(at 9.41 3.3)', '(at 9.32 3)'),)        # TC/UC 0.17 mm apart
MICRO_RULE = ('(rule "micro via drill to drill"\n  (constraint hole_to_hole (min {}mm))\n'
              '  (condition "A.Via_Type == \'Micro\' || B.Via_Type == \'Micro\'"))\n')
IGNORED = '(rule "ignored"\n  (constraint hole_to_hole (min 0.4mm))\n  (severity ignore))\n'

#: name -> (rules file text or None, board text edits, text to add, KiCad's pairs)
ROWS = {
    "the issue's board": ('FIXTURE', (), '', {('TA', 'TB'): 'drill to drill'}),
    'without the micro via rule': (
        ONE, (), '', {('TA', 'TB'): 'drill to drill', ('TC', 'UC'): 'drill to drill'}),
    'no rules file': (None, (), '', {}),
    'two micro vias 0.16 mm apart': (None, LASER, '', {}),
    'two through vias 0.16 mm apart': (None, LASER + DRILLED, '', {('TA', 'TA'): None}),
    'a via beside a pad drill': (
        'FIXTURE', (), PAD, {('TA', 'TB'): 'drill to drill', ('P1', 'TD'): 'drill to drill'}),
    'two blind vias whose spans share no layer': (None, BLIND_APART, '', {}),
    'two blind vias sharing In1.Cu': (None, BLIND_SHARED, '', {('TA', 'TA'): None}),
    'a through via and a micro via on In2.Cu-B.Cu': (
        None, THROUGH_MICRO, '', {('TA', 'TA'): None}),
    'two micro vias stacked': (None, STACKED, '', {}),
    'Board Setup 0.15, a micro via 0.17 mm from a through via': (None, TC_CLOSE, '', {}),
    'a 0.15 mm micro via rule, 0.17 mm': (
        '(version 1)\n' + MICRO_RULE.format(0.15), TC_CLOSE, '', {}),
    'an ignored rule over Board Setup 0.38': ('(version 1)\n' + IGNORED, (), '', {}),
    'an ignored rule, then a micro via rule': (
        '(version 1)\n' + IGNORED + MICRO_RULE.format(0.4), (), '',
        {('TC', 'UC'): 'micro via drill to drill'}),
    'two vias typed micro over the whole stack, 0.16 mm': (None, MECH_MICRO, '', {}),
}

#: name -> Board Setup values the row's project changes
RULES = {
    'Board Setup 0.15, a micro via 0.17 mm from a through via': {'min_hole_to_hole': 0.15},
    'an ignored rule over Board Setup 0.38': {'min_hole_to_hole': 0.38},
    'an ignored rule, then a micro via rule': {'min_hole_to_hole': 0.38},
}

#: name -> the pairs of two laser-drilled micro vias the row's NOT GRADED line
#: counts (KiCad reports none of them); a row not named has none
LASER_UNGRADED = {
    'without the micro via rule': 1,
    'two micro vias 0.16 mm apart': 1,
    'two through vias 0.16 mm apart': 1,
    'two micro vias stacked': 1,
    'an ignored rule over Board Setup 0.38': 1,
    'an ignored rule, then a micro via rule': 1,
}

#: name -> the pairs check_drc adds to KiCad's: two vias typed micro but
#: mechanically drilled (#1844), graded as two mechanical holes
MECHANICAL_ONLY = {
    'two vias typed micro over the whole stack, 0.16 mm': {('UA', 'UA'): None},
}

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru, edits=(), add='', rules=None):
    with open(evidence(BOARD + '.kicad_pcb'), encoding='utf-8') as fh:
        t = fh.read()
    for old, new in edits:
        assert t.count(old) == 1, f'the fixture has one {old}'
        t = t.replace(old, new)
    if add:
        k = t.rindex('(embedded_fonts')
        k = t.rindex('\n', 0, k) + 1
        t = t[:k] + add + t[k:]
    b = os.path.join(td, name + '.kicad_pcb')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(t)
    with open(evidence(BOARD + '.kicad_pro'), encoding='utf-8') as fh:
        proj = fh.read()
    if rules:
        doc = json.loads(proj)
        doc['board']['design_settings']['rules'].update(rules)
        proj = json.dumps(doc, indent=2)
    with open(os.path.join(td, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        fh.write(proj)
    if dru == 'FIXTURE':
        with open(evidence(BOARD + '.kicad_dru'), encoding='utf-8') as fh:
            dru = fh.read()
    if dru:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return evidence(b, 'the staged board')


def grade(path):
    out = path[:-len('.kicad_pcb')] + '.drc.json'
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), path,
                             '--json', out],
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    text = (r.stdout or '') + (r.stderr or '')
    if 'Traceback' in text or r.returncode not in (0, 1):
        raise AssertionError(f"BROKEN TEST: check_drc died\n{text[-1500:]}")
    with open(evidence(out, 'the check_drc report'), encoding='utf-8') as fh:
        return json.load(fh), text


def pairs(doc):
    return {tuple(sorted((v['net1'], v['net2']))): v.get('rule')
            for v in doc['items'] if v['type'] in DRILL_TYPES and not v.get('accepted')}


def laser_ungraded(text):
    m = re.search(r'NOT GRADED \(hole_to_hole\): (\d+) pair\(s\) of two laser-drilled', text)
    return int(m.group(1)) if m else 0


def rows(td):
    print("1. check_drc reports KiCad's hole_to_hole pairs on every variant")
    for i, (name, (dru, edits, add, want)) in enumerate(ROWS.items()):
        doc, text = grade(stage(td, f'r{i}', dru, edits, add, RULES.get(name)))
        got = pairs(doc)
        want = {**want, **MECHANICAL_ONLY.get(name, {})}
        ok(got == want, f"{name}: {got}" + ('' if got == want else f" want {want}"))
        got, want = laser_ungraded(text), LASER_UNGRADED.get(name, 0)
        ok(got == want, f"   laser pairs left ungraded: {got}"
           + ('' if got == want else f" want {want}"))


def naming(td):
    print("2. a rule-bound finding carries the rule and its value")
    doc, text = grade(stage(td, 'n', 'FIXTURE'))
    row = [v for v in doc['items'] if v['type'] == 'via-drill-hole']
    ok(len(row) == 1 and abs(row[0].get('required_mm', 0) - 0.4) < 1e-9
       and abs(row[0]['overlap_mm'] - 0.04) < 1e-6,
       f"TA/TB: required {row[0].get('required_mm') if row else None}, "
       f"short {row[0]['overlap_mm'] if row else None}")
    ok("rule 'drill to drill'" in text, "and the listing prints the rule")
    _doc, text = grade(stage(td, 'l', None, LASER))
    ok("KiCad's hole_to_hole grades no such pair" in text,
       "a pair of two laser-drilled micro vias left ungraded says KiCad grades none")
    doc, _ = grade(stage(td, 'i', '(version 1)\n' + IGNORED, (), '',
                         RULES['an ignored rule over Board Setup 0.38']))
    row = [v for v in doc['items'] if v['type'] == 'via-drill-hole'
           and {v['net1'], v['net2']} == {'TA', 'TB'}]
    ok(len(row) == 1 and row[0].get('rule') == 'ignored'
       and row[0].get('accepted') == 'severity-ignore',
       "a pair an ignored rule binds is listed accepted under that rule, not counted")


def edges(td):
    print("3. a rule below the fab floor, and a rule that cannot be read")
    doc, text = grade(stage(td, 't', TIGHT, LASER + DRILLED))
    row = [v for v in doc['items'] if v['type'] == 'via-drill-hole']
    ok(len(row) == 1 and abs(row[0].get('required_mm', 0) - 0.2) < 1e-9
       and "fab floor, over rule 'tight'" in text,
       "a 0.1 mm rule is pinned up to the 0.2 mm fab floor, and the finding says so")
    doc, text = grade(stage(td, 'u', UNREAD))
    ok("NOT GRADED (hole_to_hole): rule 'unread'" in text, "printed NOT GRADED")
    ok(any(e.get('rule') == 'unread' and e.get('kind') == 'hole_to_hole'
           for e in doc.get('not_graded') or []), "and listed in the --json document")
    ok(not pairs(doc), "its drills are graded at the board's 0.2 and pass")


def main():
    with tempfile.TemporaryDirectory() as td:
        rows(td)
        naming(td)
        edges(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
