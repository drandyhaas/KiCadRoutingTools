#!/usr/bin/env python3
"""
Regression test for issue #165: multipoint diff-pair escape direction.

`get_pair_end_direction` averages the P and N stub directions from
`get_stub_direction`, which is documented to take the stub FREE END. The
multipoint diff-pair path instead passes the PAD position, so for a pad that has
an escape stub (a fanned-out QFN/connector pin) `get_stub_direction` returns the
direction reversed - pointing INTO the component body. That backed the diff-pair
centerline setback into the chip, so the coupled route died on the pad field and
the pair fell to single-ended (the tigard /USB_D coupled-route failure).

The fix orients the stub-derived escape away from the owning component's pad
centroid (toward the routing area). This test builds a chip whose centroid is to
the +x side, with a P/N pad pair on the left edge and escape stubs running -x
(outward), and asserts the escape direction comes back pointing -x (outward),
whether the call is given the pad position (multipoint) or the stub free end
(single-pair) - never +x into the body.

Run:
    python3 tests/test_diff_multipoint_escape_dir.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import Pad, Footprint, Segment, PCBData
from diff_pair_routing import get_pair_end_direction

P_NET, N_NET = 10, 20


def _pad(ref, num, net, x, y):
    name = '/DP' if net == P_NET else '/DN'
    return Pad(ref, num, x, y, 0, 0, 0.25, 0.25, 'rect', ['F.Cu'],
               net, name, 0, None, 0.0, None, 0.25, 0.0, None)


def _seg(net, x0, y0, x1, y1):
    return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1,
                   width=0.15, layer='F.Cu', net_id=net)


def main():
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    # Chip U1: a column of pads on the left edge, body extending to +x so the
    # pad centroid is well to the +x (right) of the diff pins.
    p_pad = _pad('U1', '8', P_NET, 45.0, 50.20)
    n_pad = _pad('U1', '7', N_NET, 45.0, 49.80)
    filler = [_pad('U1', str(i), 0, 45.0 + 0.5 * i, 50.0) for i in range(1, 12)]  # body to +x
    u1 = Footprint('U1', 'lib:QFN', 47.0, 50.0, 0, 'F.Cu', [p_pad, n_pad] + filler, '')

    # Escape stubs running -x (outward, away from the body) to free ends at x=44.
    p_seg = _seg(P_NET, 45.0, 50.20, 44.0, 50.20)
    n_seg = _seg(N_NET, 45.0, 49.80, 44.0, 49.80)

    pcb = PCBData(footprints={'U1': u1}, nets={}, segments=[p_seg, n_seg], vias=[],
                  board_info=None,
                  pads_by_net={P_NET: [p_pad], N_NET: [n_pad]})

    # Multipoint path: called with the PAD positions (the bug trigger).
    dx, dy, synth = get_pair_end_direction(
        pcb, P_NET, N_NET, [p_seg], [n_seg],
        45.0, 50.20, 45.0, 49.80, 'F.Cu', other_center=(30.0, 50.0))
    check("pad-position call escapes OUTWARD (-x), not into the body",
          dx < -0.5, f"dir=({dx:.2f},{dy:.2f})")

    # Single-pair path: called with the stub FREE ENDS - must stay outward too.
    dx2, dy2, _ = get_pair_end_direction(
        pcb, P_NET, N_NET, [p_seg], [n_seg],
        44.0, 50.20, 44.0, 49.80, 'F.Cu', other_center=(30.0, 50.0))
    check("free-end call still escapes OUTWARD (-x)", dx2 < -0.5, f"dir=({dx2:.2f},{dy2:.2f})")

    # 2-PAD COMPONENT (e.g. termination resistor): the footprint's only two pads
    # ARE the pair, so the centroid coincides with the terminal center - there's no
    # body to point away from. The escape must orient toward the OTHER terminal.
    rp = _pad('R1', '1', P_NET, 50.0, 50.20)
    rn = _pad('R1', '2', N_NET, 50.0, 49.80)
    r1 = Footprint('R1', 'lib:R', 50.0, 50.0, 0, 'F.Cu', [rp, rn], '')
    rp_seg = _seg(P_NET, 50.0, 50.20, 49.0, 50.20)  # stub runs -x toward the route
    rn_seg = _seg(N_NET, 50.0, 49.80, 49.0, 49.80)
    pcb2 = PCBData(footprints={'R1': r1}, nets={}, segments=[rp_seg, rn_seg], vias=[],
                   board_info=None, pads_by_net={P_NET: [rp], N_NET: [rn]})
    dx3, dy3, _ = get_pair_end_direction(
        pcb2, P_NET, N_NET, [rp_seg], [rn_seg],
        50.0, 50.20, 50.0, 49.80, 'F.Cu', other_center=(40.0, 50.0))  # other end at -x
    check("2-pad component escapes toward the other terminal (-x)",
          dx3 < -0.5, f"dir=({dx3:.2f},{dy3:.2f})")

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
