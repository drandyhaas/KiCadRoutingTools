#!/usr/bin/env python3
"""check_drc grades two nets of a class tighter than Default at their class.

KiCad's clearance between two nets is the larger of their own classes'
values. The Default class is the value of the nets IN it, not a floor under
every pair: two nets of a 0.10 mm class on a board whose Default class is
0.25 mm are legal 0.2 mm apart. check_drc took max(Default, class A, class B)
and kept only classes above the Default, so it flagged them. On a board whose
bus classes sit below its Default class that is every such pair, a
differential pair's own P and N first.

The board is tests/fixtures/1897/pairs with its project rewritten here:
Default 0.25, BUS (nets A*) 0.10, no rule file. On In1.Cu and F.Cu, A1 runs
0.2 mm from A2 (both BUS) and 0.2 mm from X (Default), tracks and vias alike.

Measured with kicad-cli 10.0.8: 12 clearance rows, every one against X at
"netclass 'Default' clearance 0.2500 mm", none between A1 and A2.

Rows:
  1. every pair check_drc reports names X, and none is A1 against A2;
  2. X against A1 is still reported, tracks and vias (a pair with a Default
     net takes the Default value);
  3. with BUS raised to 0.3 the A1/A2 pairs are reported (a class above the
     Default is graded as before);
  4. with both classes at 0.1 the board is clean.
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1897')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, tag, default, bus):
    sub = os.path.join(td, tag)
    os.makedirs(sub)
    shutil.copy(evidence(os.path.join(FIX, 'pairs.kicad_pcb'), 'the fixture'),
                os.path.join(sub, 'b.kicad_pcb'))
    with open(evidence(os.path.join(FIX, 'pairs.kicad_pro'), 'its project'),
              encoding='utf-8') as fh:
        pro = json.load(fh)
    for c in pro['net_settings']['classes']:
        c['clearance'] = default if c['name'] == 'Default' else bus
    with open(os.path.join(sub, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(pro, fh, indent=2)
    with open(os.path.join(sub, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n')
    return os.path.join(sub, 'b.kicad_pcb')


def grade(board, clean=False):
    out = os.path.join(os.path.dirname(board), 'drc.json')
    argv = PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), board,
                 '--clearance-margin-mm', '0.0005', '--json', out]
    if clean:
        check(argv, accept=True)
    else:
        check(argv, refuse='DRC VIOLATIONS', code=1)
    with open(evidence(out, 'the report'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return [tuple(sorted((r.get('net1') or '', r.get('net2') or ''))) for r in doc['items']]


def main():
    with tempfile.TemporaryDirectory() as td:
        print("1. Default 0.25, BUS 0.10: the two BUS nets 0.2 mm apart")
        pairs = grade(stage(td, 'low', 0.25, 0.10))
        ok(pairs and all('X' in p for p in pairs), f"every reported pair names X: {sorted(set(pairs))}")
        ok(('A1', 'A2') not in pairs, "A1 against A2 is not reported")
        print("2. a pair with a Default net")
        ok(('A1', 'X') in pairs and len(pairs) >= 3,
           f"X against A1 is reported on tracks and vias: {len(pairs)} rows")
        print("3. BUS raised above the Default (0.25 / 0.30)")
        up = grade(stage(td, 'high', 0.25, 0.30))
        ok(('A1', 'A2') in up, "A1 against A2 is reported at the class's 0.3")
        print("4. both classes at 0.10")
        ok(grade(stage(td, 'flat', 0.10, 0.10), clean=True) == [], "the board is clean")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
