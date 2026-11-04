#!/usr/bin/env python3
"""#1558: check_drc's JSON `short` names copper of two nets in contact, never a
drill-spacing item.

A JSON item's `short` (and its type's `contacts_by_type` count) read
`overlap_mm >= clearance`. For a drill-to-drill item the overlap is measured
against the HOLE-TO-HOLE floor, so a pair of drills 0.35 mm apart graded at a
0.5 mm floor (overlap 0.15) read as touching against a 0.09 mm copper
clearance. The chain's re-fan and the routing skill act on unaccepted
`short: true` items, so a hole-spacing item read as a copper short.

On a synthetic board (vias of two nets, 0.45/0.3, copper clearance 0.09,
hole-to-hole 0.5):
1. Drills 0.35 mm apart, copper 0.2 mm apart: one via-drill-hole item, not a
   short, not counted in contacts_by_type.
2. Copper overlapping (centres 0.4 mm): the via-via item IS a short (control).
3. Drills overlapping (centres 0.25 mm): the via-via item is a short; the
   drill item is not (its barrels' copper is the via-via item), but its own
   contact (drill reaching drill) is counted in contacts_by_type.

Run:
    python3 tests/test_1558_drill_items_not_shorts.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def board(work, name, dx):
    """A 20x20 board with a /A via at (10, 10) and a /B via DX mm to its right."""
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(
            '(kicad_pcb (version 20260206) (generator "pcbnew")\n'
            ' (general (thickness 1.6)) (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (setup)\n'
            ' (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type solid)) '
            '(fill none) (layer "Edge.Cuts"))\n'
            ' (via (at 10 10) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") '
            '(net "/A") (uuid "va"))\n'
            ' (via (at %s 10) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") '
            '(net "/B") (uuid "vb"))\n)\n' % (10 + dx))
    return p


def grade(work, path):
    js = path + '.json'
    run_check([sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'py_router', 'check_drc.py'), path,
               '--clearance', '0.09', '--hole-to-hole-clearance', '0.5',
               '--clearance-margin', '0', '--no-size-checks', '--json', js],
              refuse='VIA', code=1)
    evidence(js)
    with open(js, encoding='utf-8') as fh:
        d = json.load(fh)
    return d, {i['type']: i for i in d['items'] if not i.get('accepted')}


def main():
    work = tempfile.mkdtemp(prefix='krt1558_')
    try:
        d, by = grade(work, board(work, 'spaced', 0.65))
        h = by.get('via-drill-hole')
        check('1. drills 0.35 mm apart at a 0.5 mm floor: one via-drill-hole item',
              h is not None and abs(h['overlap_mm'] - 0.15) < 1e-6
              and 'via-via' not in by, str(d['items']))
        check('1. ... and it is not a short', h is not None and h['short'] is False, str(h))
        check('1. ... nor a contact', 'via-drill-hole' not in d['contacts_by_type'],
              str(d['contacts_by_type']))

        d, by = grade(work, board(work, 'copper', 0.4))
        check('2. copper overlapping: the via-via item is a short',
              by.get('via-via', {}).get('short') is True, str(d['items']))
        check('2. ... the drill item (drills 0.1 mm apart) is not',
              by.get('via-drill-hole', {}).get('short') is False, str(d['items']))

        d, by = grade(work, board(work, 'drills', 0.25))
        check('3. drills overlapping: the via-via item is a short, the drill item is not',
              by.get('via-via', {}).get('short') is True
              and by.get('via-drill-hole', {}).get('short') is False, str(d['items']))
        check('3. ... the drill item counts as a contact at its own floor',
              d['contacts_by_type'].get('via-drill-hole') == 1,
              str(d['contacts_by_type']))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)} check(s)")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
