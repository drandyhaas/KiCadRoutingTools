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
10.0.8 reports on it (measured), as {(net, net): the rule named, or None for
Board Setup's value}:
  1. each row's check_drc pairs are KiCad's -- the last matching rule sets a
     pair's spacing, and a via against a pad's drill follows the same rule --
     plus the pairs of two micro vias (`LASER_ONLY`): KiCad's hole_to_hole
     grades a laser drill against a mechanical one but never two laser
     drills against each other, and the spacing is still a fab limit, so
     check_drc grades the pair at the value that binds it and its row says
     KiCad does not report it;
  2. a rule-bound finding carries the rule and its value;
  3. a rule below the fab floor is pinned up to it and says so (this tool's
     floor, not KiCad's), and a rule the resolver cannot read is named
     NOT GRADED.
"""
import json
import os
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
}

#: name -> the pairs of two micro vias check_drc adds to that row's (KiCad
#: reports none of them)
LASER_ONLY = {
    'without the micro via rule': {('UA', 'UB'): 'drill to drill'},
    'two micro vias 0.16 mm apart': {('UA', 'UA'): None},
    'two through vias 0.16 mm apart': {('UA', 'UA'): None},
}

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru, edits=(), add=''):
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


def pairs(doc, laser=False):
    return {tuple(sorted((v['net1'], v['net2']))): v.get('rule')
            for v in doc['items'] if v['type'] in DRILL_TYPES and not v.get('accepted')
            and bool(v.get('laser_pair')) == laser}


def rows(td):
    print("1. check_drc reports KiCad's hole_to_hole pairs on every variant")
    for i, (name, (dru, edits, add, want)) in enumerate(ROWS.items()):
        doc, _ = grade(stage(td, f'r{i}', dru, edits, add))
        got = pairs(doc)
        ok(got == want, f"{name}: {got}" + ('' if got == want else f" want {want}"))
        got, want = pairs(doc, laser=True), LASER_ONLY.get(name, {})
        ok(got == want, f"   two micro vias: {got}" + ('' if got == want else f" want {want}"))


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
    ok("KiCad's hole_to_hole does not grade this pair" in text,
       "a pair of two micro vias says KiCad does not report it")


def edges(td):
    print("3. a rule below the fab floor, and a rule that cannot be read")
    doc, text = grade(stage(td, 't', TIGHT, LASER + DRILLED))
    row = [v for v in doc['items'] if v['type'] == 'via-drill-hole'
           and not v.get('laser_pair')]
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
