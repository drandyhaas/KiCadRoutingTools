#!/usr/bin/env python3
"""#1563: check_drc's JSON `short` is true on a crossing of two nets' tracks.

`short` read `overlap_mm >= clearance`, and a `segment-crossing` item carries
no `overlap_mm` -- two tracks crossing on one layer are in contact by
definition -- so every different-net crossing read `short: false`, and was
left out of `contacts_by_type` and the console's `in CONTACT` count. The
chain's re-fan (autoroute `_shorts`) and the routing skill act on unaccepted
`short: true` items, so a crossing short was invisible to them.

On a synthetic board (two 0.2 mm tracks on F.Cu, clearance 0.1):
1. /A and /B cross: one segment-crossing item, a short, counted as a contact
   in the JSON and on the console.
2. Two /A tracks cross: no item is a short, nothing is a contact (control).

Run:
    python3 tests/test_1563_crossing_is_short.py
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


def board(work, name, net2):
    """A 20x20 board with a /A track along y=10 and a NET2 track along x=10."""
    p = os.path.join(work, name + '.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(
            '(kicad_pcb (version 20260206) (generator "pcbnew")\n'
            ' (general (thickness 1.6)) (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (setup)\n'
            ' (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type solid)) '
            '(fill none) (layer "Edge.Cuts"))\n'
            ' (segment (start 5 10) (end 15 10) (width 0.2) (layer "F.Cu") '
            '(net "/A") (uuid "sa"))\n'
            ' (segment (start 10 5) (end 10 15) (width 0.2) (layer "F.Cu") '
            '(net "%s") (uuid "sb"))\n)\n' % net2)
    return p


def grade(path, refuse=None):
    js = path + '.json'
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_router', 'check_drc.py'), path,
            '--clearance', '0.1', '--clearance-margin', '0',
            '--no-size-checks', '--json', js]
    r = (run_check(argv, refuse=refuse, code=1) if refuse
         else run_check(argv, accept=True))
    evidence(js)
    with open(js, encoding='utf-8') as fh:
        d = json.load(fh)
    return d, [i for i in d['items'] if not i.get('accepted')], r.stdout


def main():
    work = tempfile.mkdtemp(prefix='krt1563x_')
    try:
        d, items, out = grade(board(work, 'cross', '/B'), 'SEGMENT-CROSSING')
        x = [i for i in items if i['type'] == 'segment-crossing']
        check('1. /A x /B: one segment-crossing item', len(x) == 1, str(items))
        check('1. ... and it is a short', bool(x) and x[0]['short'] is True, str(x))
        check('1. ... counted in contacts_by_type',
              d['contacts_by_type'].get('segment-crossing') == 1,
              str(d['contacts_by_type']))
        check('1. ... and on the console',
              'SEGMENT-CROSSING violations (1) -- 1 in CONTACT' in out,
              [l for l in out.splitlines() if 'SEGMENT-CROSSING' in l])

        d, items, _ = grade(board(work, 'same', '/A'))
        check('2. /A x /A (control): no violation, nothing a short',
              not any(i.get('short') for i in d['items'])
              and 'segment-crossing' not in d['contacts_by_type'],
              str(d['items']))
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
