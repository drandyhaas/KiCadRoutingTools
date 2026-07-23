#!/usr/bin/env python3
"""Regression test for the mid-span track-through-pad false negative (#479).

steppenprobe C7: a +1V8 fanout tap threads straight through the capacitor's
SMD pad -- centreline 0.22mm from the pad centre, both endpoints >1mm outside
the pad. The router's terminal selector (connectivity._pad_on_group) credits
that and correctly drops the pad from routing, but check_net_connectivity only
credited endpoint-in-pad -- so the pad graded as a phantom disconnect, every
retry did nothing ("nothing to route"), and the board could never reach 100%.
KiCad grades the pad connected.

check_net_connectivity must mirror _pad_on_group's conservative gate: credit
when the centreline reaches within seg.width/2 + min(pad_dims)/4 of the pad
CENTRE; a track merely grazing the pad outline stays disconnected.

    python3 tests/test_track_through_pad_connectivity.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Segment
from check_connected import check_net_connectivity, run_connectivity_check


def _pad(ref, x, y):
    # steppenprobe C7.1's real geometry: 0.875 x 0.950 roundrect on F.Cu.
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=0.875, size_y=0.95,
               shape='roundrect', layers=['F.Cu'], net_id=10, net_name='+1V8',
               rotation=0.0)


def _seg(x1, y1, x2, y2, width=0.15):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=width, layer='F.Cu', net_id=10)


BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (net 0 "") (net 10 "+1V8")
  (footprint "test:C7" (layer "F.Cu") (at 132.2125 78.6)
    (property "Reference" "C7" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd roundrect (at 0 0) (size 0.875 0.95) (layers "F.Cu")
      (net 10 "+1V8")))
  (footprint "test:U1" (layer "F.Cu") (at 131.05 77.75)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
    (pad "49" smd roundrect (at 0 0) (size 0.5 0.5) (layers "F.Cu")
      (net 10 "+1V8")))
  (segment (start 131.05 77.75) (end 132.8 79.5) (width 0.15) (layer "F.Cu") (net 10) (uuid "00000000-0000-0000-0000-000000000001"))
)
"""


def main():
    results = []

    # Unit: steppenprobe C7's exact geometry -- the tap threads the pad
    # mid-span (centreline 0.221mm from centre; credit reach = 0.075 +
    # 0.875/4 = 0.294), both endpoints 1.07/1.44mm from the pad centre.
    pad = _pad('C7', 132.2125, 78.6)
    far = _pad('U1', 131.05, 77.75)     # anchors the segment's component
    seg = _seg(131.05, 77.75, 132.8, 79.5)
    r = check_net_connectivity(10, [seg], [], [pad, far], [])
    results.append(("mid-span track through pad credits connection",
                    r['connected']))

    # Negative: same track shifted so the centreline passes 0.60mm from the
    # pad centre -- outside the 0.294 gate (grazes only the outline corner).
    seg_far = _seg(131.05 + 0.55, 77.75 - 0.55, 132.8 + 0.55, 79.5 - 0.55)
    r = check_net_connectivity(10, [seg_far], [], [pad, far], [])
    results.append(("grazing track does NOT credit connection",
                    not r['connected']))

    # Board level: the whole-board checker agrees (no issues on the fixture).
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD)
        path = f.name
    try:
        issues = run_connectivity_check(path, quiet=True)
        results.append(("board check: threaded pad not flagged", not issues))
    finally:
        os.unlink(path)

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} track-through-pad connectivity tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
