#!/usr/bin/env python3
"""Unit tests for check_pads.find_pad_overlaps (pad-geometry sanity check).

Self-contained: builds tiny synthetic footprints rather than depending on board
files, so it exercises the overlap/rotation/layer logic directly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Footprint, PCBData, BoardInfo
from check_pads import find_pad_overlaps


def _pad(num, x, y, sx, sy, net_id, net_name="n", rot=0.0, layers=None, drill=0.0):
    return Pad(
        component_ref="U1", pad_number=num, global_x=x, global_y=y,
        local_x=x, local_y=y, size_x=sx, size_y=sy, shape="rect",
        layers=layers if layers is not None else ["F.Cu"],
        net_id=net_id, net_name=net_name, rotation=0.0, drill=drill,
        rect_rotation=rot,
    )


def _board(pads):
    fp = Footprint(reference="U1", footprint_name="TEST", x=0, y=0,
                   rotation=0.0, layer="F.Cu", value="")
    fp.pads = pads
    bi = BoardInfo(layers={}, copper_layers=["F.Cu", "B.Cu"], board_bounds=None)
    return PCBData(board_info=bi, nets={}, footprints={"U1": fp},
                   vias=[], segments=[], pads_by_net={})


def _check(name, pads, expect):
    n = len(find_pad_overlaps(_board(pads)))
    ok = (n == expect)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: overlaps={n} (expect {expect})")
    return ok


def main():
    results = []

    # Two long pads side by side at 0.5 mm pitch, 0.3 wide -> clear gap, no overlap.
    results.append(_check(
        "adjacent non-overlapping",
        [_pad("1", 0.0, 0.0, 0.3, 1.5, 11), _pad("2", 0.5, 0.0, 0.3, 1.5, 22)],
        0))

    # Same pads but 1.5 wide -> deep overlap, different nets -> 1 hit.
    results.append(_check(
        "overlapping different-net",
        [_pad("1", 0.0, 0.0, 1.5, 0.3, 11), _pad("2", 0.5, 0.0, 1.5, 0.3, 22)],
        1))

    # Overlapping but SAME net -> legitimate, no hit.
    results.append(_check(
        "overlapping same-net",
        [_pad("1", 0.0, 0.0, 1.5, 0.3, 11), _pad("2", 0.5, 0.0, 1.5, 0.3, 11)],
        0))

    # Overlapping in XY but on different copper layers -> cannot short, no hit.
    results.append(_check(
        "overlapping different-layer",
        [_pad("1", 0.0, 0.0, 1.5, 0.3, 11, layers=["F.Cu"]),
         _pad("2", 0.5, 0.0, 1.5, 0.3, 22, layers=["B.Cu"])],
        0))

    # Net 0 (unconnected/mechanical) pads are ignored even if they overlap.
    results.append(_check(
        "net-0 ignored",
        [_pad("1", 0.0, 0.0, 1.5, 0.3, 0), _pad("2", 0.5, 0.0, 1.5, 0.3, 0)],
        0))

    # Rotation matters: two vertical (rot via dims) pads stepped diagonally clear,
    # but if one is modelled at the wrong orientation they collide. A correctly
    # tilted -45 pad and its neighbour stay disjoint.
    results.append(_check(
        "diagonal pad disjoint",
        [_pad("1", 0.0, 0.0, 1.5, 0.3, 11, rot=-45.0),
         _pad("2", 1.2, 1.2, 1.5, 0.3, 22, rot=-45.0)],
        0))

    # ...the same two pads modelled axis-aligned (rotation dropped) DO overlap -
    # this is exactly the bug check_pads catches.
    results.append(_check(
        "diagonal pad mis-modelled overlaps",
        [_pad("1", 0.0, 0.0, 1.5, 0.3, 11, rot=0.0),
         _pad("2", 1.2, 0.0, 1.5, 0.3, 22, rot=0.0)],
        1))

    passed = sum(results)
    print(f"\n{passed}/{len(results)} check_pads tests passed")
    print("=" * 60)
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
