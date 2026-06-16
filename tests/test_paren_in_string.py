#!/usr/bin/env python3
"""
Regression tests for string-aware paren matching in the parser (issue #113).

KiCad property values can contain a lone, unbalanced parenthesis -- e.g. an MPN
like ``"TCR2EF115,LM(CT"``. The footprint/pad/zone block scanners used to count
parens without skipping quoted strings, so such a value made the scan run far
past the block's real end and swallow the pads of following footprints. On the
cynthion board this duplicated U4's pads ~250x and made hundreds of nets appear
disconnected.

This builds a tiny synthetic board entirely in-memory (no external fixtures) and
asserts each footprint keeps exactly its own pads.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, find_matching_paren


BOARD = r'''(kicad_pcb (version 20240108) (generator pcbnew)
  (general (thickness 1.6))
  (paper "A4")
  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (44 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "/A")
  (net 2 "/B")
  (gr_line (start 0 0) (end 100 0) (layer "Edge.Cuts") (width 0.1))
  (footprint "Package_TO_SOT_SMD:SOT-23-5"
    (layer "F.Cu")
    (at 10 10)
    (property "Reference" "U4")
    (property "Value" "TCR2EF115,LM(CT")
    (pad "1" smd rect (at 0 0) (size 0.6 0.7) (layers "F.Cu") (net 1 "/A"))
    (pad "2" smd rect (at 1 0) (size 0.6 0.7) (layers "F.Cu") (net 2 "/B"))
  )
  (footprint "Resistor_SMD:R_0402"
    (layer "F.Cu")
    (at 20 20)
    (property "Reference" "R1")
    (property "Value" "10k")
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A"))
    (pad "2" smd rect (at 1 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "/B"))
  )
)
'''


def main():
    failures = []

    # Unit-level: find_matching_paren must skip parens inside quoted strings.
    s = '(a "x(y" (b) )'
    end = find_matching_paren(s, 0)
    if s[:end] != s.strip():
        failures.append(f"find_matching_paren skipped string paren wrong: end={end} -> {s[:end]!r}")
    else:
        print("  PASS  find_matching_paren ignores '(' inside a quoted string")

    with tempfile.NamedTemporaryFile("w", suffix=".kicad_pcb", delete=False) as f:
        f.write(BOARD)
        path = f.name
    try:
        pcb = parse_kicad_pcb(path)
    finally:
        os.unlink(path)

    u4 = pcb.footprints.get("U4")
    r1 = pcb.footprints.get("R1")

    if u4 is None or len(u4.pads) != 2:
        failures.append(f"U4 should have 2 pads, got {len(u4.pads) if u4 else 'missing'}")
    else:
        print("  PASS  U4 (value with unbalanced paren) keeps exactly its 2 pads")

    if r1 is None or len(r1.pads) != 2:
        failures.append(f"R1 should have 2 pads, got {len(r1.pads) if r1 else 'missing'}")
    else:
        print("  PASS  R1 (following footprint) is not swallowed -- keeps its 2 pads")

    # No phantom duplicate pad instances anywhere on the board.
    allpads = [p for fp in pcb.footprints.values() for p in fp.pads]
    seen = {}
    for p in allpads:
        key = (p.component_ref, p.pad_number, round(p.global_x, 3), round(p.global_y, 3))
        seen[key] = seen.get(key, 0) + 1
    dups = sum(v - 1 for v in seen.values() if v > 1)
    if dups:
        failures.append(f"{dups} duplicate pad instances found board-wide")
    else:
        print("  PASS  no duplicate pad instances board-wide")

    print()
    if failures:
        for msg in failures:
            print(f"  FAIL  {msg}")
        print(f"\n{len(failures)} failure(s)")
        sys.exit(1)
    print("All scenarios passed")


if __name__ == "__main__":
    main()
