#!/usr/bin/env python3
"""Regression test for issue #285 (icepi_zero USB-C CC2 nets).

A 2-pad net whose opposite-layer pads sit ~0.06mm apart can be legitimately
bridged by a single via that physically overlaps both pads (no segments at
all) -- the cycle-prune pass verifies this with the union-find before removing
the redundant loop copper. check_connected.py used to classify any net with
zero SEGMENTS as "Unrouted" without looking at vias, grading the (connected)
pruned net as broken.

    python3 tests/test_via_only_net_connectivity.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Via
from check_connected import check_net_connectivity, run_connectivity_check


def _pad(ref, x, y, layers):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=0.6, size_y=0.45, shape='roundrect',
               layers=layers, net_id=7, net_name='Net-(J3-CC2)', rotation=0.0)


BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (net 0 "") (net 7 "Net-(J3-CC2)")
  (footprint "test:J3" (layer "F.Cu") (at 158.25 114.07)
    (property "Reference" "J3" (at 0 0) (layer "F.SilkS"))
    (pad "B5" smd roundrect (at 0 0) (size 0.6 0.45) (layers "F.Cu") (net 7 "Net-(J3-CC2)")))
  (footprint "test:R26" (layer "B.Cu") (at 158.31 114.04)
    (property "Reference" "R26" (at 0 0) (layer "B.SilkS"))
    (pad "2" smd roundrect (at 0 0) (size 0.6 0.45) (layers "B.Cu") (net 7 "Net-(J3-CC2)")))
  (via (at 158.25 114.05) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") (net 7) (uuid "00000000-0000-0000-0000-000000000001"))
)
"""


def main():
    results = []

    # Unit level: the union-find credits the via bridging both pads.
    pad_f = _pad('J3', 158.25, 114.07, ['F.Cu'])
    pad_b = _pad('R26', 158.31, 114.04, ['B.Cu'])
    via = Via(x=158.25, y=114.05, size=0.45, drill=0.2,
              layers=['F.Cu', 'B.Cu'], net_id=7)
    r = check_net_connectivity(7, [], [via], [pad_f, pad_b], [])
    results.append(("union-find: single via bridges opposite-layer pads",
                    r['connected']))

    # File level: the whole-board checker must NOT report the net as unrouted
    # (it has no segments, but the via is real copper connecting the pads).
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD)
        path = f.name
    try:
        issues = run_connectivity_check(path, quiet=True)
        results.append(("board check: via-only net not flagged", not issues))
    finally:
        os.unlink(path)

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} via-only-net connectivity tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
