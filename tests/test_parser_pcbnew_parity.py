#!/usr/bin/env python3
"""Text-parser fixes found by GUI validate-board parity checks (thunderscope /
comexpress7). Each case here used to make parse_kicad_pcb model the board
differently from pcbnew (build_pcb_data_from_board):

1. Custom pads whose gr_poly outline is drawn from (arc ...) entries only
   (thunderscope U11 B0QFN): extent came back (0,0) -> pad modelled at its
   0.2mm anchor.
2. Custom pads drawn from standalone gr_arc / gr_line primitives (comexpress7
   H1..H10 thermal-spoke pads).
3. Pad (primitives ...) blocks leaking into board-level graphic scans: a
   primitive gr_line stitched to a later element's (layer "User.1") tag made
   a phantom guide path with pad-local coordinates.
4. Reference-less footprints -- (footprint "") with no Reference property
   (thunderscope's 86 locked NPTH drill dots): dropped entirely, losing their
   hole obstacles. Now keyed by their uuid.
5. Multi-layer (layers "F.Cu" "B.Cu" ...) zones: pinned to the first
   filled_polygon's layer instead of one Zone per copper layer.
6. KiCad-10 (net "") copper: "" must map to net 0 (no-net), not a synthetic id.

The full parity check against pcbnew is validate_pcb_data.py (needs KiCad's
bundled python); this test locks the text-parser side without pcbnew.

    python3 tests/test_parser_pcbnew_parity.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb

BOARD = '''(kicad_pcb (version 20260206) (generator test)
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (39 "User.1" user) (44 "Edge.Cuts" user))
 (gr_line (start 0 0) (end 30 0) (layer "Edge.Cuts") (width 0.1))
 (gr_line (start 30 0) (end 30 30) (layer "Edge.Cuts") (width 0.1))
 (gr_line (start 30 30) (end 0 30) (layer "Edge.Cuts") (width 0.1))
 (gr_line (start 0 30) (end 0 0) (layer "Edge.Cuts") (width 0.1))
 (footprint "test:arcpad" (layer "F.Cu") (at 10 10)
  (property "Reference" "U1" (at 0 -2) (layer "F.SilkS"))
  (pad "2" smd custom (at 0 0) (size 0.2 0.2)
   (layers "F.Cu")
   (net "SIG")
   (options (clearance outline) (anchor rect))
   (primitives
    (gr_poly
     (pts
      (arc (start 0.5 -0.375) (mid 0.535355 -0.360355) (end 0.55 -0.325))
      (arc (start 0.55 0.325) (mid 0.535355 0.360355) (end 0.5 0.375))
      (arc (start -0.1 0.375) (mid -0.135355 0.360355) (end -0.15 0.325))
      (arc (start -0.15 -0.325) (mid -0.135355 -0.360355) (end -0.1 -0.375))
     )
     (width 0) (fill yes)
    )
   )
  )
  (pad "3" smd custom (at 3 0) (size 0.62 0.62)
   (layers "F.Cu")
   (net "SIG2")
   (options (clearance outline) (anchor circle))
   (primitives
    (gr_arc (start -0.25 1.279) (mid 0.285 0.294) (end 1.266 -0.247) (width 0.2))
    (gr_line (start 1.379 -0.25) (end 1.379 0.25) (width 0.2))
   )
  )
 )
 (footprint "" (locked yes) (layer "F.Cu")
  (uuid "aaaa1111-0000-0000-0000-000000000001")
  (at 20 20)
  (property "Reference" "" (at 0 0 0) (layer "F.SilkS"))
  (pad "" np_thru_hole circle (at 0 0) (size 0.38 0.38) (drill 0.38) (layers "*.Cu" "*.Mask"))
 )
 (footprint "" (locked yes) (layer "F.Cu")
  (uuid "aaaa1111-0000-0000-0000-000000000002")
  (at 22 20)
  (property "Reference" "" (at 0 0 0) (layer "F.SilkS"))
  (pad "" np_thru_hole circle (at 0 0) (size 0.38 0.38) (drill 0.38) (layers "*.Cu" "*.Mask"))
 )
 (fp_line (start 5 5) (end 9 5) (stroke (width 0.1) (type solid)) (layer "User.1"))
 (segment (start 1 1) (end 2 1) (width 0.2) (layer "F.Cu") (net "") (uuid "seg-nonet-1"))
 (zone
  (layers "F.Cu" "B.Cu")
  (uuid "zz")
  (net "GND")
  (hatch edge 0.5)
  (polygon (pts (xy 1 1) (xy 5 1) (xy 5 5) (xy 1 5)))
  (filled_polygon (layer "F.Cu") (pts (xy 1 1) (xy 5 1) (xy 5 5) (xy 1 5)))
  (filled_polygon (layer "B.Cu") (pts (xy 1 1) (xy 5 1) (xy 5 5) (xy 1 5)))
 )
)
'''


def run():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        if not cond:
            fails.append(name)

    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        # KiCad's writer puts zones at top level with one tab of indentation;
        # _iter_zone_blocks depends on that, so normalize the test file the
        # same way.
        f.write(BOARD.replace('\n (', '\n\t(').replace('\n  (', '\n\t\t(')
                     .replace('\n   (', '\n\t\t\t(').replace('\n    (', '\n\t\t\t\t(')
                     .replace('\n     (', '\n\t\t\t\t\t(')
                     .replace('\n      (', '\n\t\t\t\t\t\t('))
        path = f.name
    try:
        pcb = parse_kicad_pcb(path)

        # 1. arc-only gr_poly custom pad: extent spans x [-0.15,0.55] y +/-0.375
        u1 = pcb.footprints['U1']
        p2 = next(p for p in u1.pads if p.pad_number == '2')
        check("arc-poly pad width covers primitives (>=1.0)", p2.size_x >= 1.0)
        check("arc-poly pad height covers primitives (>=0.7)", p2.size_y >= 0.7)

        # 2. gr_arc / gr_line primitives pad
        p3 = next(p for p in u1.pads if p.pad_number == '3')
        check("gr_arc primitive pad grew past its anchor", p3.size_x > 0.63 and p3.size_y > 0.63)

        # 3. no phantom guide path from pad primitives; the real User.1
        #    top-level line is NOT a gr_line (it's an fp_line at top level,
        #    which KiCad doesn't write, so guide paths here must be empty)
        check("no phantom guide paths", len(pcb.guide_paths) == 0)

        # 4. reference-less footprints keyed by uuid, pads kept
        noref = [r for r in pcb.footprints if r.startswith('#')]
        check("both no-ref footprints kept", len(noref) == 2)
        check("no-ref NPTH pads kept",
              all(len(pcb.footprints[r].pads) == 1 for r in noref))

        # 5. multi-layer zone -> one Zone per copper layer
        gnd_zones = [(z.layer) for z in pcb.zones]
        check("zone on both copper layers", sorted(gnd_zones) == ['B.Cu', 'F.Cu'])

        # 6. (net "") copper maps to net 0
        seg = [s for s in pcb.segments if abs(s.start_x - 1) < 1e-6 and abs(s.start_y - 1) < 1e-6]
        check("(net \"\") segment parsed", len(seg) == 1)
        check("(net \"\") maps to net 0", seg and seg[0].net_id == 0)
    finally:
        os.unlink(path)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
