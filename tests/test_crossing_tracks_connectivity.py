#!/usr/bin/env python3
"""Two same-layer tracks crossing in an X are connected (#1279 review).

A load track crossing a Kelvin sense trace mid-span: neither endpoint of
either track lies on the other, so check_net_connectivity's endpoint
(T-junction) rule never joined them and graded the net as two islands, while
KiCad's connectivity counts the overlapping copper as one. Two straight
tracks that do NOT cross are closest at an endpoint of one of them, which
that rule already credits, so a proper crossing is the only case it missed.

Negative controls: the same X on two different layers stays split, and two
tracks that come close without crossing stay split unless their copper
reaches (the T-junction rule's own credit, unchanged).

    python3 tests/test_crossing_tracks_connectivity.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Pad, Segment  # noqa: E402
from check_connected import check_net_connectivity, run_connectivity_check  # noqa: E402


def _pad(ref, x, y):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=1.0, size_y=1.0, shape='rect',
               layers=['F.Cu'], net_id=1, net_name='VDD', rotation=0.0)


def _seg(x1, y1, x2, y2, layer='F.Cu', width=0.2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=width, layer=layer, net_id=1)


BOARD = """(kicad_pcb (version 20241229) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (net 0 "") (net 1 "VDD")
  (footprint "T:P" (layer "F.Cu") (at 3 5)
    (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "VDD")))
  (footprint "T:P" (layer "F.Cu") (at 15 8)
    (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "VDD")))
  (segment (start 3 5) (end 27 5) (width 0.2) (layer "F.Cu") (net 1) (uuid "11111111-0000-0000-0000-000000000001"))
  (segment (start 15 2) (end 15 8) (width 0.3) (layer "F.Cu") (net 1) (uuid "11111111-0000-0000-0000-000000000002"))
)
"""


def main():
    results = []
    c1, u2 = _pad('C1', 3, 5), _pad('U2', 15, 8)
    trace = _seg(3, 5, 27, 5)

    # The X: (15, 2)-(15, 8) crosses the trace at (15, 5); its ends are 3 mm
    # off the trace on either side, and the trace's ends are 12 mm away.
    r = check_net_connectivity(1, [trace, _seg(15, 2, 15, 8, width=0.3)], [], [c1, u2])
    results.append(("an X crossing on one layer is connected", r['connected']))

    # A diagonal X ending on U2, crossing the trace at (13.63, 5).
    r = check_net_connectivity(1, [trace, _seg(12.3, 2.1, 15, 8)], [], [c1, u2])
    results.append(("a diagonal X crossing is connected", r['connected']))

    # Negative: the same X with the crossing track on B.Cu (no via).
    r = check_net_connectivity(1, [trace, _seg(15, 2, 15, 8, layer='B.Cu')], [], [c1, u2])
    results.append(("an X across two layers stays split", not r['connected']))

    # Negative: a track stopping 0.5 mm short of the trace does not cross it.
    r = check_net_connectivity(1, [trace, _seg(15, 5.5, 15, 8)], [], [c1, u2])
    results.append(("a track short of the trace stays split", not r['connected']))

    # Whole-board checker: the X board reports no issue.
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False,
                                     encoding='utf-8') as f:
        f.write(BOARD)
        path = f.name
    try:
        issues = run_connectivity_check(path, quiet=True)
        results.append(("board check: the X board is connected", not issues))
    finally:
        os.unlink(path)

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} crossing-track connectivity tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
