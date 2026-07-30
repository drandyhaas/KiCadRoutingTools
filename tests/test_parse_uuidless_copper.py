#!/usr/bin/env python3
"""Issue #74: uuid-less segments and vias must parse.

KiCad accepts copper without a (uuid ...) token, and tool-injected copper
(stitch vias, escape stubs) frequently omits it. The strict via/segment
patterns required uuid, so such copper was SILENTLY absent from the model:
the router planned straight through real via barrels (AQM-1K Tier-1: 52 of
203 vias invisible; a +3V3 reroute crossed an invisible GND stitch via at
0.014mm), and same-net vias could not serve as route terminals.

    python3 tests/test_parse_uuidless_copper.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rust_router"))

from kicad_parser import parse_kicad_pcb

BOARD = """(kicad_pcb (version 20241229) (generator "test")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (44 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "SIG")
  (net 2 "GND")
  (segment (start 10 10) (end 20 10) (width 0.2) (layer "F.Cu") (net 1) (uuid "aaaa"))
  (segment (start 20 10) (end 30 10) (width 0.2) (layer "F.Cu") (net 1))
  (via (at 30 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) (uuid "bbbb"))
  (via (at 15 12) (size 0.25) (drill 0.15) (layers "F.Cu" "B.Cu") (net 2))
)
"""


def main():
    with tempfile.NamedTemporaryFile("w", suffix=".kicad_pcb",
                                     delete=False) as f:
        f.write(BOARD)
        path = f.name
    try:
        pcb = parse_kicad_pcb(path)
    finally:
        os.unlink(path)

    assert len(pcb.segments) == 2, \
        f"expected 2 segments (1 uuid-less), got {len(pcb.segments)}"
    assert len(pcb.vias) == 2, \
        f"expected 2 vias (1 uuid-less), got {len(pcb.vias)}"
    uuidless_via = [v for v in pcb.vias if v.net_id == 2]
    assert uuidless_via and uuidless_via[0].size == 0.25
    assert uuidless_via[0].uuid == ""
    uuidless_seg = [s for s in pcb.segments if s.start_x == 20.0]
    assert uuidless_seg and uuidless_seg[0].uuid == ""

    print("OK: #74 uuid-less segments and vias parse")


if __name__ == "__main__":
    main()
