#!/usr/bin/env python3
"""#418: custom pads whose primitives are UNFILLED (stroke-only) must include
the stroke width in their resolved copper extent.

An unfilled gr_circle of centerline radius r and stroke width w has copper out
to r + w/2 (pcbnew strokes the outline of filled primitives too). The parser
under-sized such pads: ottercast MK1 pad 1 (Panasonic WM55A103 mic, rotated
315 deg) resolved to 4.218mm instead of pcbnew's 5.009mm. Also covers the
rotated-circle extent (axis-extreme sampling under-covered rotated pads) and
the gr_poly outline stroke in _custom_pad_global_polygons.

    python3 tests/test_custom_pad_stroke_extent.py
"""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# The ottercast MK1 shape: ring centerline r=1.35 centred at local (1.35, 0),
# stroke 0.4, UNFILLED -> copper is an annulus out to 1.35 + 0.2 = 1.55.
# One copy rotated 315 deg (the repro), one axis-aligned.
BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "GND")
  (footprint "test:mic_rotated" (layer "F.Cu")
    (at 10 10 315)
    (property "Reference" "MK1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd custom (at -1.35 0 315) (size 0.4 0.4) (layers "F.Cu")
      (net 1 "GND")
      (options (clearance outline) (anchor circle))
      (primitives
        (gr_circle (center 1.35 0) (end 2.7 0) (width 0.4) (fill no))
      ))
  )
  (footprint "test:mic_flat" (layer "F.Cu")
    (at 20 10)
    (property "Reference" "MK2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd custom (at 0 0) (size 0.4 0.4) (layers "F.Cu")
      (net 1 "GND")
      (options (clearance outline) (anchor circle))
      (primitives
        (gr_circle (center 1.35 0) (end 2.7 0) (width 0.4) (fill no))
      ))
  )
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
)
"""


def _board():
    from kicad_parser import parse_kicad_pcb
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(BOARD)
        path = f.name
    try:
        return parse_kicad_pcb(path)
    finally:
        os.unlink(path)


def test_rotated_unfilled_circle_size_includes_stroke():
    """The ottercast repro: pcbnew resolves 5.009 x 5.009; the parser must too.
    Exact extent per side = |centre rotated 45deg|.x + r + w/2
                          = 1.35*cos(45) + 1.35 + 0.2 = 2.50459..."""
    pcb = _board()
    pad = pcb.footprints['MK1'].pads[0]
    expected = 2.0 * (1.35 * math.cos(math.radians(45)) + 1.35 + 0.2)  # 5.00919
    assert abs(pad.size_x - expected) < 1e-6, (pad.size_x, expected)
    assert abs(pad.size_y - expected) < 1e-6, (pad.size_y, expected)
    # Regression floor: the pre-#418 parser produced 4.218 here.
    assert pad.size_x > 5.0, pad.size_x


def test_axis_aligned_unfilled_circle_size_includes_stroke():
    """Unrotated: x extent = |cx| + r + w/2 = 1.35 + 1.55 = 2.9 per side is the
    +x side; symmetric box about the anchor -> size 5.8 x 3.1."""
    pcb = _board()
    pad = pcb.footprints['MK2'].pads[0]
    assert abs(pad.size_x - 5.8) < 1e-6, pad.size_x
    assert abs(pad.size_y - 3.1) < 1e-6, pad.size_y


def test_polygon_disc_includes_stroke():
    """pad.polygons models the ring as a disc out to r + w/2 = 1.55 (interior
    kept solid -- conservative for clearance to external copper)."""
    pcb = _board()
    pad = pcb.footprints['MK2'].pads[0]
    assert pad.polygons, "custom pad should carry polygons"
    disc = pad.polygons[0]
    max_r = max(math.hypot(x - (pad.global_x + 1.35), y - pad.global_y)
                for x, y in disc)
    assert abs(max_r - 1.55) < 1e-3, max_r


def test_gr_poly_outline_stroke_grows_polygon():
    """A stroked gr_poly's copper reaches width/2 beyond the outline: a 2x2
    square with width 0.4 must cover a point 0.1 outside the bare outline."""
    from kicad_parser import _custom_pad_global_polygons
    PAD = """(pad "1" smd custom (at 0 0 0) (size 0.1 0.1)
      (primitives (gr_poly (pts (xy 0 0) (xy 2 0) (xy 2 2) (xy 0 2))
        (width 0.4) (fill yes))))"""
    polys = _custom_pad_global_polygons(PAD, 100.0, 50.0, 0.0)
    assert polys and len(polys) == 2, polys  # gr_poly + anchor (#337)
    xs = [x for x, y in polys[0]]
    ys = [y for x, y in polys[0]]
    # Outline bbox local [0,2]x[0,2] grown by w/2=0.2 -> global [99.8,102.2].
    assert abs(min(xs) - 99.8) < 1e-6 and abs(max(xs) - 102.2) < 1e-6, (min(xs), max(xs))
    assert abs(min(ys) - 49.8) < 1e-6 and abs(max(ys) - 52.2) < 1e-6, (min(ys), max(ys))


def main():
    tests = [
        test_rotated_unfilled_circle_size_includes_stroke,
        test_axis_aligned_unfilled_circle_size_includes_stroke,
        test_polygon_disc_includes_stroke,
        test_gr_poly_outline_stroke_grows_polygon,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print("=" * 60)
    if failed:
        print(f"{failed} test(s) failed")
        return 1
    print(f"{len(tests)}/{len(tests)} custom-pad stroke-extent tests passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
