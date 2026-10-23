#!/usr/bin/env python3
"""check_drc grades a via only on the layers it spans: an F.Cu->In1.Cu GND
microvia under a segment on In2.Cu or B.Cu is no overlap (it vetoed every In2
diff-pair candidate under a GND pad on cm5-io), a through via is an overlap on
every layer.

The TYPE word makes a via blind, not its layer pair. KiCad loads an untyped
`(via ... (layers "F.Cu" "In1.Cu"))` as a through via -- kicad-cli flags a B.Cu
track beside it -- and this repo's writer emits that shape, so check_drc must
grade it on every layer too.

The same holds for the other two places check_drc puts a via against copper
on one layer: a pad (an SMD pad on B.Cu does not meet an F.Cu->In1.Cu
microvia, a through-hole pad does) and the interior of a filled copper
graphic. kicad-cli 10.0.3 on the board below reports the U2 and U3 pads and
nothing else.

    python3 tests/test_via_segment_span.py
"""
import os
import sys
import tempfile
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "py_router"))
from check_drc import check_pad_via_overlap, check_via_segment_overlap, run_drc  # noqa: E402

LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "B.Cu"]

# Microvias (F.Cu->In1.Cu, net A) beside a B.Cu pad (U1), beside an F.Cu pad
# (U2) and inside a filled B.Cu graphic; an UNTYPED F.Cu/In1.Cu via -- a
# through via to KiCad -- beside another B.Cu pad (U3). Every via sits 0.8 mm
# from its pad's centre: the 1 mm pad and the 0.6 mm via touch.
BOARD = '''(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(4 "In1.Cu" signal)
\t\t(6 "In2.Cu" signal)
\t\t(8 "In3.Cu" signal)
\t\t(10 "In4.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(25 "Edge.Cuts" user)
\t)
\t(net 0 "")
\t(net 1 "A")
\t(net 2 "B")
\t(footprint "t:pad" (layer "B.Cu") (uuid "f1") (at 10 10)
\t\t(property "Reference" "U1" (at 0 0 0) (layer "B.SilkS") (uuid "r1"))
\t\t(pad "1" smd rect (at 0 0) (size 1 1) (layers "B.Cu") (net 2 "B") (uuid "p1"))
\t)
\t(footprint "t:pad" (layer "F.Cu") (uuid "f2") (at 30 10)
\t\t(property "Reference" "U2" (at 0 0 0) (layer "F.SilkS") (uuid "r2"))
\t\t(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 2 "B") (uuid "p2"))
\t)
\t(footprint "t:pad" (layer "B.Cu") (uuid "f3") (at 10 16)
\t\t(property "Reference" "U3" (at 0 0 0) (layer "B.SilkS") (uuid "r3"))
\t\t(pad "1" smd rect (at 0 0) (size 1 1) (layers "B.Cu") (net 2 "B") (uuid "p3"))
\t)
\t(gr_line (start 0 0) (end 40 0) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e1"))
\t(gr_line (start 40 0) (end 40 20) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e2"))
\t(gr_line (start 40 20) (end 0 20) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e3"))
\t(gr_line (start 0 20) (end 0 0) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e4"))
\t(gr_poly (pts (xy 17 7) (xy 23 7) (xy 23 13) (xy 17 13)) (stroke (width 0.1) (type solid)) (fill yes) (layer "B.Cu") (uuid "g1"))
\t(via micro (at 10.8 10) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") (free yes) (net 1) (uuid "v1"))
\t(via micro (at 30.8 10) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") (free yes) (net 1) (uuid "v2"))
\t(via micro (at 20 10) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") (free yes) (net 1) (uuid "v3"))
\t(via (at 10.8 16) (size 0.6) (drill 0.3) (layers "F.Cu" "In1.Cu") (free yes) (net 1) (uuid "v4"))
)
'''


def seg(layer):
    return SimpleNamespace(start_x=0.0, start_y=0.0, end_x=2.0, end_y=0.0, width=0.2, layer=layer)


def pad(layers, drill=0.0):
    return SimpleNamespace(global_x=1.0, global_y=0.5, size_x=0.5, size_y=0.5,
                           shape="rect", layers=layers, drill=drill,
                           pad_type="thru_hole" if drill else "smd",
                           rect_rotation=0.0, roundrect_rratio=0.0,
                           custom_polygon=None)


def main():
    micro = SimpleNamespace(x=1.0, y=0.1, size=0.25, layers=["F.Cu", "In1.Cu"], via_type="micro")
    through = SimpleNamespace(x=1.0, y=0.1, size=0.25, layers=["F.Cu", "B.Cu"], via_type="through")
    untyped = SimpleNamespace(x=1.0, y=0.1, size=0.25, layers=["F.Cu", "In1.Cu"], via_type="through")
    assert check_via_segment_overlap(micro, seg("F.Cu"), 0.1)[0]
    assert check_via_segment_overlap(micro, seg("In1.Cu"), 0.1)[0]
    assert not check_via_segment_overlap(micro, seg("In2.Cu"), 0.1)[0]
    assert not check_via_segment_overlap(micro, seg("B.Cu"), 0.1)[0]
    for layer in ("F.Cu", "In2.Cu", "B.Cu"):
        assert check_via_segment_overlap(through, seg(layer), 0.1)[0], layer
        assert check_via_segment_overlap(untyped, seg(layer), 0.1)[0], f"untyped {layer}"

    # pad-via: the via sits 0.15 mm from the pad edge, inside 0.25/2 + 0.1.
    assert check_pad_via_overlap(pad(["F.Cu"]), micro, 0.1, LAYERS)[0]
    assert not check_pad_via_overlap(pad(["B.Cu"]), micro, 0.1, LAYERS)[0]
    assert check_pad_via_overlap(pad(["*.Cu"], drill=0.3), micro, 0.1, LAYERS)[0]
    assert check_pad_via_overlap(pad(["B.Cu"]), through, 0.1, LAYERS)[0]
    assert check_pad_via_overlap(pad(["B.Cu"]), untyped, 0.1, LAYERS)[0]

    # The whole grader on a board, both via arms that are not segments.
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "span.kicad_pcb")
        with open(path, "w", encoding="utf-8") as f:
            f.write(BOARD)
        rows = run_drc(path, clearance=0.2, quiet=True, print_summary=False,
                       check_sizes=False)
    pad_via = sorted(r["pad_ref"] for r in rows if r["type"] == "pad-via")
    assert pad_via == ["U2.1", "U3.1"], pad_via
    via_rows = [r for r in rows if r["type"] == "via-segment"]
    assert not via_rows, via_rows
    print("PASS")


if __name__ == "__main__":
    main()
