#!/usr/bin/env python3
"""Regression test for issue #89 (via-in-pad subcase).

A via dropped inside an SMD pad's copper connects that pad to a plane even when
the via centre is offset from the pad centre. The geometric connectivity check
(check_net_connectivity) used to union a via to a pad only by a tight
centre-to-centre proximity (~0.15mm), so an offset via-in-pad on a larger plane
pad read as unconnected despite a physical via in it (lumenpnp C89/C103,
splitflap U5.13/U9.13, bitaxe Q1, ...). It must credit a via whose centre lies
within the pad outline on a shared copper layer.

    python3 tests/test_via_in_pad_connectivity.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Via, Zone
from check_connected import check_net_connectivity, _point_in_pad


def _pad(ref, x, y, sx, sy, layers, shape='roundrect', rot=0.0):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=sx, size_y=sy, shape=shape,
               layers=layers, net_id=5, net_name='GND', rotation=rot)


def scenarios():
    results = []

    # 1. Cross-layer via-in-pad: F.Cu plane pad, via to In1 plane, offset within pad.
    pad_f = _pad('C89', 10.0, 10.0, 0.6, 1.95, ['F.Cu'])         # tall plane pad
    on_plane = _pad('U1', 20.0, 10.0, 0.6, 0.6, ['In1.Cu'])      # already on plane
    via = Via(x=10.0, y=10.8, size=0.5, drill=0.3, layers=['F.Cu', 'In1.Cu'], net_id=5)
    zone = Zone(net_id=5, net_name='GND', layer='In1.Cu',
                polygon=[(5, 5), (25, 5), (25, 15), (5, 15)])
    r = check_net_connectivity(5, [], [via], [pad_f, on_plane], [zone])
    results.append(("offset via-in-pad connects cross-layer plane", r['connected']))

    # 2. A via genuinely OUTSIDE the pad must NOT be credited (no false positive).
    far_via = Via(x=10.0, y=12.5, size=0.5, drill=0.3, layers=['F.Cu', 'In1.Cu'], net_id=5)
    r = check_net_connectivity(5, [], [far_via], [pad_f, on_plane], [zone])
    results.append(("via outside pad stays disconnected", not r['connected']))

    # 3. Rotated pad: via inside the rotated rectangle is credited. A drawn
    # 0.6x1.95 pad rotated 90deg is presented by the parser in BOARD space:
    # size_x/size_y swapped (-> wide in x) with rect_rotation 0 for the
    # orthogonal angle. _point_in_pad reads that resolved geometry, not the
    # total rotation, so the fixture must supply the swapped sizes.
    rpad = _pad('C90', 30.0, 30.0, 1.95, 0.6, ['F.Cu'])  # board-resolved: wide in x
    rvia = Via(x=30.8, y=30.0, size=0.5, drill=0.3, layers=['F.Cu', 'In1.Cu'], net_id=5)
    zone3 = Zone(net_id=5, net_name='GND', layer='In1.Cu',
                 polygon=[(25, 25), (35, 25), (35, 35), (25, 35)])
    on_plane3 = _pad('U2', 33.0, 30.0, 0.6, 0.6, ['In1.Cu'])
    r = check_net_connectivity(5, [], [rvia], [rpad, on_plane3], [zone3])
    results.append(("via inside rotated pad is credited", r['connected']))

    # 4. _point_in_pad sanity.
    results.append(("_point_in_pad inside", _point_in_pad(10.0, 10.8, pad_f)))
    results.append(("_point_in_pad outside", not _point_in_pad(10.0, 11.5, pad_f)))

    return results


def main():
    print("=" * 60)
    print("Issue #89 via-in-pad connectivity regression test")
    print("=" * 60)
    results = scenarios()
    for name, ok in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    n_pass = sum(1 for _, ok in results if ok)
    print(f"\n{n_pass}/{len(results)} checks passed")
    print("=" * 60)
    return 0 if n_pass == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
