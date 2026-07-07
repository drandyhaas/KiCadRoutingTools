#!/usr/bin/env python3
"""
Regression test for check_drc broad-phase misses (false CLEAN verdicts).

Two holes in SpatialIndex candidate generation let real violations through:

1. MID-SEGMENT PAD: the pad-segment check queried pads around the segment's
   two ENDPOINTS only (+-1 cell of the 2mm grid), so a long straight segment
   grazing a pad >~4mm from both ends never saw that pad as a candidate.
   Fix: get_nearby_pads_for_segment walks every cell the segment crosses.

2. ROTATED PAD BULGE: add_pad indexed the UNROTATED size_x/size_y bbox. A
   long pad at an angle protrudes past that bbox (an 8x1 pad at 30deg bulges
   ~2.4mm in y); copper sitting in never-indexed cells was invisible to the
   endpoint/via queries. Fix: _pad_half_extents rotates the bbox (and covers
   custom-pad polygons).

Both are exercised end-to-end through run_drc on a synthetic .kicad_pcb.

Run:
    python3 tests/test_drc_broadphase.py
"""
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (44 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "NETA")
  (net 2 "NETB")
  (net 3 "NETC")
  (gr_line (start 0 0) (end 60 0) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 60 0) (end 60 40) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 60 40) (end 0 40) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 0 40) (end 0 0) (layer "Edge.Cuts") (width 0.1))
  (footprint "test:PAD1" (layer "F.Cu") (at 25 10.2)
    (property "Reference" "P1")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "NETA"))
  )
  (footprint "test:PAD2" (layer "F.Cu") (at 45 30)
    (property "Reference" "P2")
    (pad "1" smd rect (at 0 0 45) (size 20 1) (layers "F.Cu") (net 2 "NETB"))
  )
  (segment (start 5 10) (end 55 10) (width 0.2) (layer "F.Cu") (net 3) (uuid "s1"))
  (segment (start 48 22.45) (end 58 22.45) (width 0.2) (layer "F.Cu") (net 3) (uuid "s2"))
)
"""


def main():
    from check_drc import run_drc
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'bp.kicad_pcb')
        with open(board, 'w') as f:
            f.write(BOARD)
        result = run_drc(board, clearance=0.2, quiet=True)
        viols = result['violations'] if isinstance(result, dict) else result
        ps = [v for v in viols if v.get('type') == 'pad-segment']

        # 1) NETA pad at (25,10.2): the 50mm NETC segment along y=10 passes
        #    0.2mm-edge-to-edge under it, 20mm from either endpoint.
        check("mid-segment pad graze detected (was endpoint-only false clean)",
              any(v.get('pad_ref') == 'P1.1' or 'NETA' in (v.get('net1'), v.get('net2'))
                  for v in ps))

        # 2) NETB 20x1 pad rotated 45deg at (45,30): its tip reaches
        #    ~(52.1, 22.9), >4mm (2+ index cells) below the unrotated bbox
        #    (y in 29.5..30.5), so the old axis-aligned indexing never put the
        #    pad in the cells the second NETC segment queries.
        check("rotated-pad corner graze detected (unrotated bbox false clean)",
              any(v.get('pad_ref') == 'P2.1' or 'NETB' in (v.get('net1'), v.get('net2'))
                  for v in ps))

    if fails:
        print(f"\nFAIL ({len(fails)})")
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
