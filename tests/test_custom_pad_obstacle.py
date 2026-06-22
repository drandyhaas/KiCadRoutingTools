#!/usr/bin/env python3
"""A KiCad "custom" pad (e.g. a SolderJumper) carries most of its copper in (primitives ...),
not the small (size ...) anchor. The obstacle map must cover the real primitive copper, or a
track routes across the unmodelled copper and shorts to it.

Regression for the SolderJumper short: a SolderJumper-3 pad 1 is a 1.0x0.5 (size) anchor whose
(primitives ...) reach y = +/-0.75 (real copper 1.5mm tall, 3x the anchor). Before the fix the
obstacle modelled only the 0.5mm anchor, so a track could sit at y~0.5mm - clear of the modelled
keepout but on top of the real copper - and short. After the fix the obstacle covers the copper.

    python3 tests/test_custom_pad_obstacle.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import _pad_custom_copper_extremes, extract_footprints_and_pads
from routing_utils import pad_blocked_cells_array

# A real KiCad SolderJumper-3 pad 1: 1.0x0.5 anchor; two gr_circle + a gr_poly reaching y=+/-0.75.
SJ_PAD = '''(pad "1" smd custom (at -1.3 0) (size 1 0.5) (layers "F.Cu") (net 1 "dc")
      (primitives
        (gr_circle (center 0 0.25) (end 0.5 0.25) (width 0) (fill yes))
        (gr_circle (center 0 -0.25) (end 0.5 -0.25) (width 0) (fill yes))
        (gr_poly (pts (xy 0.55 0.75) (xy 0 0.75) (xy 0 -0.75) (xy 0.55 -0.75)) (width 0) (fill yes))
      ))'''

FOOTPRINT = '''(footprint "test:SJ" (layer "F.Cu") (at 50 50 0)
    (property "Reference" "SJ1" (at 0 0) (layer "F.SilkS"))
    ''' + SJ_PAD + '''
    (pad "2" smd rect (at 0 0) (size 1 1.5) (layers "F.Cu") (net 2 "vref"))
  )'''


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return cond


def main():
    r = []

    # --- 1. the primitives helper sees the full +/-0.75 copper, not the 0.25 anchor ---
    pts = _pad_custom_copper_extremes(SJ_PAD)
    ys = [y for _, y in pts]
    xs = [x for x, _ in pts]
    r.append(_ok("primitives parsed", len(pts) > 0))
    r.append(_ok("primitive copper reaches y = +/-0.75 (gr_poly)",
                 abs(min(ys) + 0.75) < 1e-9 and abs(max(ys) - 0.75) < 1e-9))
    r.append(_ok("primitive copper reaches x = +0.55 (gr_poly) and <= -0.5 (circle radius)",
                 abs(max(xs) - 0.55) < 1e-9 and min(xs) <= -0.5 + 1e-9))
    r.append(_ok("a pad with no (primitives ...) yields no extremes",
                 _pad_custom_copper_extremes('(pad "9" smd rect (at 0 0) (size 1 1))') == []))

    # gr_arc must be LINEARIZED, not point-sampled: a >180-deg arc (start/end near +x, mid at
    # -x) bulges to y~+/-0.5, but its 3 defining points only span y=+/-0.1. Sampling start/mid/
    # end would under-cover the bulge (the exact failure class); linearization must catch it.
    arc_pad = ('(pad "1" smd custom (at 0 0) (size 0.2 0.2) (net 1 "n") (primitives '
               '(gr_arc (start 0.5 0.1) (mid -0.5 0) (end 0.5 -0.1) (width 0))))')
    ays = [y for _, y in _pad_custom_copper_extremes(arc_pad)]
    r.append(_ok("gr_arc bbox covers the bulge (linearized): |y| reaches ~0.5, not the 0.1 chord",
                 ays and max(ays) > 0.4 and min(ays) < -0.4))

    # --- 2. parse: the custom pad gets a custom_obstacle covering the copper; a rect pad does not ---
    footprints, _pbn = extract_footprints_and_pads(FOOTPRINT, {}, {})
    pads = {p.pad_number: p for fp in footprints.values() for p in fp.pads}
    sj, rect = pads['1'], pads['2']
    r.append(_ok("custom pad has a custom_obstacle", sj.custom_obstacle is not None))
    if sj.custom_obstacle is not None:
        cx, cy, hx, hy = sj.custom_obstacle
        # pad center = footprint (50,50) + pad local (-1.3,0) = (48.7, 50); copper spans y +/-0.75
        r.append(_ok("custom_obstacle half_y covers the real copper (~0.75, not the 0.25 anchor)",
                     abs(hy - 0.75) < 1e-6))
        r.append(_ok("custom_obstacle y-span covers +/-0.75 of the pad center",
                     cy - hy <= 50.0 - 0.75 + 1e-6 and cy + hy >= 50.0 + 0.75 - 1e-6))
        r.append(_ok("custom_obstacle is wider than the 1.0 anchor (gr_poly extends +x)",
                     hx > 0.5 + 1e-9))
    r.append(_ok("standard rect pad keeps custom_obstacle == None", rect.custom_obstacle is None))

    # a rotated footprint (fp_rot=90) must transform the custom_obstacle correctly: the 1.5mm-tall
    # copper becomes 1.5mm-WIDE (half_x ~0.75, half_y ~0.525), proving the pad->footprint->board
    # transform handles rotation, not just the axis-aligned SolderJumper at fp_rot=0.
    rfps, _ = extract_footprints_and_pads(FOOTPRINT.replace('(at 50 50 0)', '(at 50 50 90)'), {}, {})
    rsj = {p.pad_number: p for fp in rfps.values() for p in fp.pads}['1']
    r.append(_ok("rotated (fp_rot=90) custom pad has a custom_obstacle", rsj.custom_obstacle is not None))
    if rsj.custom_obstacle is not None:
        _, _, rhx, rhy = rsj.custom_obstacle
        r.append(_ok("fp_rot=90 swaps the copper extent: half_x ~0.75 (was half_y), half_y ~0.525",
                     abs(rhx - 0.75) < 1e-6 and abs(rhy - 0.525) < 1e-6))

    # --- 3. obstacle coverage: the primitive extent blocks a cell the (size) anchor missed ---
    grid, margin = 0.05, 0.30      # track 0.2 / clearance 0.2 -> 0.1 + 0.2
    def blocked(half_h):
        cells = pad_blocked_cells_array(0, 0, 0.525, half_h, margin, grid,
                                        corner_radius=0.0, off_x=0.0, off_y=0.0)
        return {(int(x), int(y)) for x, y in cells}
    target = (0, 12)               # y = 0.60mm: outside the anchor keepout (0.25+0.30=0.55),
                                   # inside the real-copper keepout (0.75+0.30=1.05) -> the short.
    r.append(_ok("size-only anchor (half_y=0.25) leaves the y=0.60 cell free (the short)",
                 target not in blocked(0.25)))
    r.append(_ok("primitive copper (half_y=0.75) blocks the y=0.60 cell (short prevented)",
                 target in blocked(0.75)))

    passed = sum(r)
    print(f"\n{passed}/{len(r)} custom-pad obstacle tests passed")
    print("=" * 60)
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())
