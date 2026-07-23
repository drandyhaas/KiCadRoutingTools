#!/usr/bin/env python3
"""Regression test for issue #479 (ch32v003_usb / ch32v006_dev shield nets).

KiCad auto-names no-connect nets 'unconnected-(...)'; a USB connector's shield
tabs share ONE such net across several pads (joined mechanically by the
connector shell). Wildcard routing skips these nets (net_queries.
expand_net_patterns) and human-routed reference boards leave them unrouted,
but check_connected.py used to grade them as "Unrouted nets" -- so a board
whose only "failure" was a shield net could never reach 100% completion.

Default grading must skip multi-pad 'unconnected-*' nets; an explicit --nets
pattern must still check them.

    python3 tests/test_noconnect_net_grading.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from check_connected import run_connectivity_check


BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (net 0 "") (net 1 "SIG") (net 17 "unconnected-(J1-SHIELD-PadS1)")
  (footprint "test:J1" (layer "F.Cu") (at 150 90)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
    (pad "S1" thru_hole circle (at 0 0) (size 2.2 2.2) (drill 1.7)
      (layers "*.Cu" "*.Mask") (net 17 "unconnected-(J1-SHIELD-PadS1)"))
    (pad "S1" thru_hole circle (at 8 0) (size 2.2 2.2) (drill 1.7)
      (layers "*.Cu" "*.Mask") (net 17 "unconnected-(J1-SHIELD-PadS1)"))
    (pad "1" smd rect (at 2 4) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG"))
    (pad "2" smd rect (at 6 4) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG")))
  (segment (start 152 94) (end 156 94) (width 0.25) (layer "F.Cu") (net 1) (uuid "00000000-0000-0000-0000-000000000001"))
)
"""


def main():
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD)
        path = f.name

    results = []
    try:
        # Default grading: the 2-pad shield no-connect net must NOT be
        # reported (the routed SIG net is connected, so no issues at all).
        issues = run_connectivity_check(path, quiet=True)
        results.append(("default grading skips multi-pad 'unconnected-*' net",
                        not issues))

        # Explicit --nets pattern still checks it (override).
        issues = run_connectivity_check(path, net_patterns=['unconnected-*'],
                                        quiet=True)
        flagged = [i['net_name'] for i in issues if i.get('unrouted')]
        results.append(("explicit --nets pattern still grades it",
                        flagged == ['unconnected-(J1-SHIELD-PadS1)']))
    finally:
        os.unlink(path)

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} no-connect grading tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
