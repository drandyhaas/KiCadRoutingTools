#!/usr/bin/env python3
"""Regression test for multi-board files (#479, len42_filter2).

A file whose Edge.Cuts draws TWO separate board outlines (a module + control
board mated by connectors, split keyboards) has nets spanning both boards --
no copper route can ever join them. check_connected must grade such nets PER
OUTLINE: within each outline the net's pads must be connected; the
board-to-board edge is exempt. A net split WITHIN one outline stays flagged,
and single-outline boards are untouched.

    python3 tests/test_multi_outline_connectivity.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_connected import run_connectivity_check


BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (net 0 "") (net 1 "CROSS") (net 2 "SPLIT") (net 3 "LINK")
  (gr_rect (start 0 0) (end 30 30) (layer "Edge.Cuts") (width 0.1))
  (gr_rect (start 40 0) (end 70 30) (layer "Edge.Cuts") (width 0.1))
  (footprint "test:A" (layer "F.Cu") (at 5 5)
    (property "Reference" "A1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "CROSS"))
    (pad "2" smd rect (at 0 5) (size 1 1) (layers "F.Cu") (net 2 "SPLIT"))
    (pad "3" smd rect (at 0 10) (size 1 1) (layers "F.Cu") (net 3 "LINK")))
  (footprint "test:B" (layer "F.Cu") (at 15 5)
    (property "Reference" "B1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "CROSS"))
    (pad "2" smd rect (at 0 5) (size 1 1) (layers "F.Cu") (net 2 "SPLIT")))
  (footprint "test:C" (layer "F.Cu") (at 45 5)
    (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "CROSS"))
    (pad "3" smd rect (at 0 10) (size 1 1) (layers "F.Cu") (net 3 "LINK")))
  (footprint "test:D" (layer "F.Cu") (at 55 5)
    (property "Reference" "D1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "CROSS")))
  (segment (start 5 5) (end 15 5) (width 0.25) (layer "F.Cu") (net 1) (uuid "00000000-0000-0000-0000-000000000001"))
  (segment (start 45 5) (end 55 5) (width 0.25) (layer "F.Cu") (net 1) (uuid "00000000-0000-0000-0000-000000000002"))
)
"""


def main():
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD)
        path = f.name

    results = []
    try:
        issues = run_connectivity_check(path, quiet=True)
        flagged = {i['net_name'] for i in issues}
        # CROSS: A1-B1 joined on board 1, C1-D1 joined on board 2; only the
        # board-to-board edge is missing -> complete.
        results.append(("cross-board net connected per outline not flagged",
                        'CROSS' not in flagged))
        # SPLIT: two pads on board 1 with NO copper between them -> a real
        # failure that per-outline grading must NOT excuse.
        results.append(("net split WITHIN one outline still flagged",
                        'SPLIT' in flagged))
        # LINK: exactly one pad per board, no copper anywhere -> nothing on
        # either board to route -> not an unrouted-net issue.
        results.append(("one-pad-per-board link net not flagged",
                        'LINK' not in flagged))
    finally:
        os.unlink(path)

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} multi-outline connectivity tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
