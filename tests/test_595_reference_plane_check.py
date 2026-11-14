#!/usr/bin/env python3
"""#595, the measurement: where signal tracks leave their reference plane.

A track crossing a gap in its adjacent plane -- a split between two pours, a
void, a slot cut by a track drawn on the plane layer -- breaks its return
path, and nothing measured it. `py_tools/check_reference_plane.py`
(`reference_plane.reference_plane_report`) does, on a routed board,
report-only. This is the instrument the issue's cost would be judged by; it
prices and routes nothing.

The fixture (tests/fixtures/595/split) is synthetic: a 4-layer board with two
pours on In1.Cu, GND for x < 14.9 and PWR for x > 15.1, and on F.Cu:
  SIG_A  x 5..25 at y 6: over the split, from GND to PWR;
  SIG_B  x 5..12 at y 8: over GND only;
  SIG_C  x 17..25 at y 8: over PWR only;
  SIG_D  x 5..12 at y 3.5: over GND, crossing a 0.2 mm track of net SLOT
         drawn on In1.Cu, which the pour clears by 0.2 mm either side.

Rows:
  1. the reference layers: F.Cu and In2.Cu refer to In1.Cu;
  2. SIG_A: one void crossing and one reference change, GND to PWR at x 15;
  3. SIG_B and SIG_C: none of either, and no track over no copper;
  4. SIG_D: one crossing and no change, the void about the 0.6 mm the pour
     clears around the plane-layer track;
  5. the pours' own nets are not measured, and a track on a layer with no
     poured neighbour is totalled as unreferenced;
  6. the CLI: the totals line, --json, --nets, and exit 2 for a pattern that
     names no net.
"""
import json
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '595', 'split.kicad_pcb')
TOOL = os.path.join(REPO, 'py_tools', 'check_reference_plane.py')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def main():
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    from reference_plane import reference_plane_report, reference_layers
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(BOARD, 'the fixture'))
        rep = reference_plane_report(pcb)
    by = rep['by_net']

    print("1. the reference layers")
    refs = {l: v for l, v in reference_layers(pcb).items() if v}
    ok(refs == {'F.Cu': ['In1.Cu'], 'In2.Cu': ['In1.Cu']}, f"{refs}")

    print("2. a track over the split between two pours")
    a = by.get('SIG_A') or {}
    ch = [r for r in rep['rows'] if r['net'] == 'SIG_A' and r['kind'] == 'change']
    ok(a.get('crossings') == 1 and a.get('reference_changes') == 1,
       f"one crossing, one change: {a}")
    ok(len(ch) == 1 and ch[0]['from'] == 'GND' and ch[0]['to_net'] == 'PWR'
       and abs(ch[0]['at'][0] - 15.0) <= 0.2,
       f"from GND to PWR at x 15: {ch}")

    print("3. tracks over one pour")
    for n in ('SIG_B', 'SIG_C'):
        r = by.get(n) or {}
        ok(r.get('crossings') == 0 and r.get('reference_changes') == 0
           and r.get('void_mm') == 0.0 and r.get('length_mm', 0) > 6,
           f"{n}: {r}")

    print("4. a track over a slot cut by a plane-layer track")
    d = by.get('SIG_D') or {}
    ok(d.get('crossings') == 1 and d.get('reference_changes') == 0
       and 0.4 <= d.get('void_mm', 0) <= 0.8, f"one crossing, about 0.6 mm: {d}")

    print("5. what is not measured")
    ok('GND' not in by and 'PWR' not in by, "the pours' own nets are left out")
    ok(by.get('SLOT', {}).get('unreferenced_mm') == 3.0
       and by.get('SLOT', {}).get('length_mm') == 0.0,
       f"the plane-layer track has no poured neighbour: {by.get('SLOT')}")

    print("6. the CLI")
    with tempfile.TemporaryDirectory() as td:
        js = os.path.join(td, 'r.json')
        r = check(PY + [TOOL, BOARD, '--json', js], accept=True)
        ok('Reference planes: 2 void crossing(s), 1 reference change(s)' in (r.stdout or ''),
           "the totals line")
        with open(evidence(js, 'the report'), encoding='utf-8') as fh:
            doc = json.load(fh)
        ok(doc['totals']['crossings'] == 2 and len(doc['rows']) == 3
           and doc['unmodelled_zones'] == [], f"--json: {doc['totals']}")
        r = check(PY + [TOOL, BOARD, '--nets', 'SIG_B'], accept=True)
        ok('0 void crossing(s), 0 reference change(s)' in (r.stdout or ''),
           "--nets limits the report")
        check(PY + [TOOL, BOARD, '--nets', 'NOPE*'], refuse='NOPE*', code=2)
        ok(True, "a pattern naming no net exits 2 and is named")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
