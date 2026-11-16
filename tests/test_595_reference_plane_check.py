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

A second board, built here (ISLAND), is the usual split plane: a board-wide
GND pour on In1.Cu (priority 0) under a PWR island x 12..18, y 5..15
(priority 1), both at 0.3 mm clearance, and a GND pour on B.Cu:
  SIG_E  F.Cu x 4..26 at y 10, over the island;
  SIG_F  F.Cu x 4..12 at y 2.5 in two segments joined at x 8, over a 0.2 mm
         SLOT track on In1.Cu at x 8 -- the void spans the joint;
  SIG_G  F.Cu x 2..10 at y 17.5, 0.5 mm beside a via of another net at
         (6, 17), whose clearance hole it passes over;
  SIG_H  In2.Cu x 4..26 at y 12, between In1.Cu and the B.Cu pour.

  7. the GND pour gives way to the island: SIG_E crosses the 0.3 mm gap at
     each side (2 crossings, 2 changes); the zone fill model alone reads GND
     there and found no crossing;
  8. a void across a joint between two segments is one crossing;
  9. a via's clearance hole is a hole crossing, not a void;
 10. a track between two reference layers crossing the gap on one of them
     while the other stays solid: 2 partial crossings, 2 changes on In1.Cu.
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

ISLAND = '''(kicad_pcb (version 20240108) (generator pcbnew)
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal)
    (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "GND") (net 2 "PWR") (net 3 "SIG_E") (net 4 "SIG_F") (net 5 "SIG_G")
  (net 6 "SIG_H") (net 7 "SLOT") (net 8 "OTHER")
  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))
  (segment (start 4 10) (end 26 10) (width 0.2) (layer "F.Cu") (net 3))
  (segment (start 4 2.5) (end 8 2.5) (width 0.2) (layer "F.Cu") (net 4))
  (segment (start 8 2.5) (end 12 2.5) (width 0.2) (layer "F.Cu") (net 4))
  (segment (start 8 1) (end 8 4) (width 0.2) (layer "In1.Cu") (net 7))
  (segment (start 2 17.5) (end 10 17.5) (width 0.2) (layer "F.Cu") (net 5))
  (via (at 6 17) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 8))
  (segment (start 4 12) (end 26 12) (width 0.2) (layer "In2.Cu") (net 6))
  (zone (net 1) (net_name "GND") (layer "In1.Cu") (hatch edge 0.5)
    (connect_pads (clearance 0.3)) (min_thickness 0.2)
    (fill (thermal_gap 0.3) (thermal_bridge_width 0.3))
    (polygon (pts (xy 0.5 0.5) (xy 29.5 0.5) (xy 29.5 19.5) (xy 0.5 19.5))))
  (zone (net 2) (net_name "PWR") (layer "In1.Cu") (priority 1) (hatch edge 0.5)
    (connect_pads (clearance 0.3)) (min_thickness 0.2)
    (fill (thermal_gap 0.3) (thermal_bridge_width 0.3))
    (polygon (pts (xy 12 5) (xy 18 5) (xy 18 15) (xy 12 15))))
  (zone (net 1) (net_name "GND") (layer "B.Cu") (hatch edge 0.5)
    (connect_pads (clearance 0.3)) (min_thickness 0.2)
    (fill (thermal_gap 0.3) (thermal_bridge_width 0.3))
    (polygon (pts (xy 0.5 0.5) (xy 29.5 0.5) (xy 29.5 19.5) (xy 0.5 19.5))))
)
'''


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def island_rows():
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    from reference_plane import reference_plane_report
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'island.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(ISLAND)
        with contextlib.redirect_stdout(io.StringIO()):
            rep = reference_plane_report(parse_kicad_pcb(p))
    by, rows = rep['by_net'], rep['rows']

    print("7. a power island in a board-wide ground pour")
    e = by.get('SIG_E') or {}
    ok(e.get('crossings') == 2 and e.get('reference_changes') == 2,
       f"two crossings of the gap, two changes: {e}")
    xs = sorted(r['at'][0] for r in rows if r['net'] == 'SIG_E' and r['kind'] == 'void')
    ok(len(xs) == 2 and 11.6 <= xs[0] <= 12.0 and 18.0 <= xs[1] <= 18.4,
       f"at the island's edges, x 12 and 18: {xs}")

    print("8. a void across a joint")
    f = by.get('SIG_F') or {}
    ok(f.get('crossings') == 1 and 0.6 <= f.get('void_mm', 0) <= 1.0,
       f"one crossing over the joint at x 8: {f}")

    print("9. a via's clearance hole")
    g = by.get('SIG_G') or {}
    ok(g.get('crossings') == 0 and g.get('void_mm') == 0.0
       and g.get('hole_crossings') == 1 and 0.5 <= g.get('hole_mm', 0) <= 0.9,
       f"a hole crossing, not a void: {g}")

    print("10. a stripline over a split in one of its two reference layers")
    h = by.get('SIG_H') or {}
    part = [r for r in rows if r['net'] == 'SIG_H' and r['kind'] == 'partial_void']
    ok(h.get('crossings') == 0 and h.get('partial_crossings') == 2
       and h.get('reference_changes') == 2
       and {r['reference_layer'] for r in part} == {'In1.Cu'},
       f"two partial crossings on In1.Cu, B.Cu solid: {h}")


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
        ok(('Reference planes: 2 void crossing(s), 0 on one of two reference layers, '
            '1 reference change(s)') in (r.stdout or ''),
           "the totals line")
        with open(evidence(js, 'the report'), encoding='utf-8') as fh:
            doc = json.load(fh)
        ok(doc['totals']['crossings'] == 2 and len(doc['rows']) == 3
           and doc['unmodelled_zones'] == [], f"--json: {doc['totals']}")
        r = check(PY + [TOOL, BOARD, '--nets', 'SIG_B'], accept=True)
        ok('0 void crossing(s), 0 on one of two reference layers, 0 reference change(s)' in (r.stdout or ''),
           "--nets limits the report")
        check(PY + [TOOL, BOARD, '--nets', 'NOPE*'], refuse='NOPE*', code=2)
        ok(True, "a pattern naming no net exits 2 and is named")
    island_rows()
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
