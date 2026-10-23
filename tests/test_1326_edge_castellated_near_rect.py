#!/usr/bin/env python3
"""#1326: the pad-edge clearance channel exempts an edge-straddling
castellated pad, and an outline that is a rectangle but for drawing
tolerance is measured as one.

`EdgeCopperContext` graded rp2350's Teensy U8 half-holes as 33 edge
violations that the off-board channel exempts by design, and it certified
only an exactly rectangular outline: complex_hierarchy draws one corner at
y=51.816 against 51.8, so every pad read 'sampled' and place_pose's `legal`
was false on every pose.

Checks, on boards built here:
  1. A castellated pad straddling the top edge is no finding; a plain pad
     beside it is one. The reading at a trial pose (`pad_copper`) agrees.
  2. A 20 x 10 mm outline with one corner 16 um low is measured as a
     rectangle shrunk by that stray: `complete`, `outline_inset_mm` 0.016,
     and a pad's gap read against the shrunk side.

    python3 tests/test_1326_edge_castellated_near_rect.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import legality as L                            # noqa: E402


def _edge(points):
    pts = list(points) + [points[0]]
    return ''.join(
        f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
        f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n'
        for a, b in zip(pts, pts[1:]))


def _board(corners, pads):
    text = f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
{_edge(corners)}  (footprint "M:Module" (layer "F.Cu") (at 10 0)
    (property "Reference" "M1" (at 0 -3) (layer "F.SilkS"))
{pads}  )
)
"""
    path = os.path.join(tempfile.mkdtemp(prefix='t1326_'), 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return parse_kicad_pcb(path), path


RECT = [(0, 0), (20, 0), (20, 10), (0, 10)]
EDGE_PADS = """    (pad "1" thru_hole circle (at -2 0) (size 1.6 1.6) (drill 1)
      (layers "*.Cu" "*.Mask") (property pad_prop_castellated) (net 0 ""))
    (pad "2" thru_hole circle (at 2 0) (size 1.6 1.6) (drill 1)
      (layers "*.Cu" "*.Mask") (net 0 ""))
"""


def test_castellated_pad_on_the_edge_is_exempt():
    pcb, path = _board(RECT, EDGE_PADS)
    ctx = L.EdgeCopperContext(pcb, 0.5, path)
    assert ctx.rectangular
    g = ctx.grade()
    assert [f['pad_ref'] for f in g['findings']] == ['M1.2'], g['findings']
    assert g['complete'], g['unmeasured']
    fp = pcb.footprints['M1']
    posed = ctx.pad_copper(fp, (fp.x, fp.y, fp.rotation))
    assert [r.index for r in posed if r.amount_mm > L.EPS] == [1], posed
    assert [r.basis for r in posed] == ['castellated', 'analytic'], posed
    print("  PASS: the castellated half-hole is exempt, the plain pad is not")


def test_near_rectangle_is_measured_inset():
    skew = [(0, 0), (20, 0), (20, 10.016), (0, 10)]
    inner = """    (pad "1" smd rect (at 0 9.4) (size 1 1) (layers "F.Cu") (net 0 ""))
"""
    pcb, path = _board(skew, inner)
    ctx = L.EdgeCopperContext(pcb, 0.05, path)
    assert ctx.rectangular and abs(ctx.inset - 0.016) < 1e-9, ctx.inset
    g = ctx.grade()
    assert g['complete'] and not g['unmeasured'], g['unmeasured']
    assert abs(g['outline_inset_mm'] - 0.016) < 1e-9
    # the pad's far side is at y = 9.9, the shrunk top side at 10.016 - 0.016
    assert abs(g['minimum_gap_mm'] - 0.1) < 1e-6, g['minimum_gap_mm']
    off = [(0, 0), (20, 0), (20, 10.5), (0, 10)]
    pcb2, path2 = _board(off, inner)
    assert not L.EdgeCopperContext(pcb2, 0.05, path2).rectangular, (
        "a corner 0.5 mm off is not drawing tolerance")
    print("  PASS: a 16 um stray reads as a rectangle, shrunk by it")


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        fn()
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
