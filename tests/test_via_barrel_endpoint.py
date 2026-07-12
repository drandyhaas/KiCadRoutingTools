#!/usr/bin/env python3
"""Regression test for the #217 glasgow /SDA checker false positive.

A stub whose endpoint lands INSIDE a same-net via's barrel (but further from
the via centre than the size/4 point tolerance) is physically connected --
KiCad grades it connected -- but check_net_connectivity graded the net split.
The via point's on-segment tolerance must be (via_diameter + track_width)/2
(copper overlap), mirroring the #285 endpoint-cap rule; exact tangency is
still not credited.

    python3 tests/test_via_barrel_endpoint.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Via, Segment
from check_connected import check_net_connectivity


def _pad(ref, x, y):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=0.35, size_y=0.35, shape='circle',
               layers=['F.Cu'], net_id=2, net_name='/SDA', rotation=0.0)


def _via(x, y):
    return Via(x=x, y=y, size=0.35, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=2)


def main():
    results = []

    # glasgow /SDA geometry: via-in-pad at (81.3,97.1); In1 stub ends 0.141mm
    # from the via centre -- inside the 0.175 barrel radius, outside size/4.
    pads = [_pad('U30', 81.3, 97.1), _pad('TP4', 83.0, 99.0)]
    vias = [_via(81.3, 97.1), _via(83.0, 99.0)]
    segs = [Segment(start_x=81.2, start_y=97.2, end_x=81.65, end_y=97.65,
                    width=0.127, layer='In1.Cu', net_id=2),
            Segment(start_x=81.65, start_y=97.65, end_x=83.0, end_y=99.0,
                    width=0.127, layer='In1.Cu', net_id=2)]
    r = check_net_connectivity(2, segs, vias, pads, [])
    results.append(("stub ending inside via barrel graded connected",
                    r['connected']))

    # Same geometry but the stub ends clearly OUTSIDE barrel + cap reach
    # (0.30mm > 0.175 + 0.0635): must stay disconnected.
    segs_far = [Segment(start_x=81.088, start_y=97.312, end_x=81.65, end_y=97.65,
                        width=0.127, layer='In1.Cu', net_id=2),
                segs[1]]
    r2 = check_net_connectivity(2, segs_far, vias, pads, [])
    results.append(("stub ending beyond barrel+cap stays disconnected",
                    not r2['connected']))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} via-barrel endpoint tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
