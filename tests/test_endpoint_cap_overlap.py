#!/usr/bin/env python3
"""Regression test for issue #285 (eit_mux /S1 phantom gap).

Two track segments whose round end caps physically overlap are connected
copper in KiCad: centreline endpoints 0.05mm apart on 0.1mm tracks overlap in
a 0.087mm-wide lens. check_net_connectivity used a width/2 threshold, which
(a) under-credited cap-cap overlap and (b) made EXACT tangency flip on FP
epsilon across the file write/parse round-trip -- the graze-prune gate said
"connected" pre-write while check_connected.py said "split" post-write, on
identical copper (verified against kicad-cli DRC: no unconnected items).

The rule is now: a segment endpoint connects when centreline distance is
under (w_point + w_seg)/2 (minus epsilon). Exact tangency -- zero-width
copper -- is still a break, so a real end-to-end gap stays flagged.

    python3 tests/test_endpoint_cap_overlap.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Segment
from check_connected import check_net_connectivity


def _pad(ref, x, y):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=0.4, size_y=0.4, shape='circle',
               layers=['F.Cu'], net_id=9, net_name='/T', rotation=0.0)


def _chain(gap):
    """Pad A -- seg1 -- <gap> -- seg2 -- pad B, all on F.Cu, 0.1mm tracks."""
    seg1 = Segment(start_x=10.0, start_y=10.0, end_x=15.0, end_y=10.0,
                   width=0.1, layer='F.Cu', net_id=9)
    seg2 = Segment(start_x=15.0 + gap, start_y=10.0, end_x=20.0, end_y=10.0,
                   width=0.1, layer='F.Cu', net_id=9)
    pads = [_pad('A', 10.0, 10.0), _pad('B', 20.0, 10.0)]
    return [seg1, seg2], pads


def main():
    cases = [
        # (gap between endpoint centres, expected connected)
        (0.05, True),    # caps overlap by a 0.087mm lens (the eit_mux /S1 case)
        (0.099, True),   # still a real (thin) overlap
        (0.1, False),    # exact tangency: zero-width copper, not credited
        (0.15, False),   # genuine gap
    ]
    results = []
    for gap, want in cases:
        segs, pads = _chain(gap)
        r = check_net_connectivity(9, segs, [], pads)
        ok = r['connected'] == want
        results.append(ok)
        print(f"  {'PASS' if ok else 'FAIL'}  gap {gap}mm on 0.1mm tracks -> "
              f"{'connected' if want else 'split'} (got {'connected' if r['connected'] else 'split'})")

    # T-joint through a segment interior still works (endpoint cap overlapping
    # the other track's body): perpendicular stub ending 0.08mm off the
    # centreline of a 0.1mm track (0.08 < (0.1+0.1)/2).
    trunk = Segment(start_x=10.0, start_y=10.0, end_x=20.0, end_y=10.0,
                    width=0.1, layer='F.Cu', net_id=9)
    stub = Segment(start_x=15.0, start_y=10.08, end_x=15.0, end_y=12.0,
                   width=0.1, layer='F.Cu', net_id=9)
    pads = [_pad('A', 10.0, 10.0), _pad('B', 15.0, 12.0)]
    r = check_net_connectivity(9, [trunk, stub], [], pads)
    ok = r['connected']
    results.append(ok)
    print(f"  {'PASS' if ok else 'FAIL'}  T-joint stub cap overlapping trunk body -> connected")

    passed = sum(results)
    print(f"\n{passed}/{len(results)} endpoint cap-overlap tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
