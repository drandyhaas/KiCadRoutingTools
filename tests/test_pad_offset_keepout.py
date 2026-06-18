#!/usr/bin/env python3
"""Pad via/track keepout must be measured from the REAL pad center, not the
quantized grid cell, so a track centerline cannot sit a sub-cell closer to a
pad than the clearance on the side the pad's center rounds toward (issue #70:
residual sub-cell PAD-SEGMENT violations at fine grid).

    python3 tests/test_pad_offset_keepout.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from routing_utils import pad_blocked_cells_array, iter_pad_blocked_cells


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    grid = 0.05
    # A pad whose real center is 0.016mm above the grid (like kit U203.13 at
    # y=77.216 -> cell 77.20). Rect half_height 0.30, margin 0.30 (track 0.2 +
    # clearance 0.2 -> 0.1+0.2=0.30). The cell 12 rows up (0.60mm from the cell
    # center) is at real distance 0.60 - 0.016 = 0.584mm from the real center;
    # its rect distance is 0.584 - 0.30 = 0.284 < 0.30 margin -> must block.
    # Measured from the quantized cell it is exactly 0.30 -> escapes the strict
    # '<' test (the 0.016mm overlap bug).
    pad_gx, pad_gy = 0, 0
    half_w, half_h = 0.975, 0.30
    margin = 0.30
    off_y = 0.016  # real center above the grid cell

    def blocked(off):
        cells = pad_blocked_cells_array(pad_gx, pad_gy, half_w, half_h, margin,
                                        grid, corner_radius=0.0, off_x=0.0, off_y=off)
        return {(int(x), int(y)) for x, y in cells}

    target = (0, 12)  # 12 rows up = 0.60mm at 0.05 grid

    # With no offset (legacy), the cell at exactly margin is NOT blocked.
    check("legacy (off=0) leaves the exactly-at-margin cell unblocked",
          target not in blocked(0.0))
    # With the real sub-cell offset, the cell is now within margin -> blocked.
    check("offset blocks the sub-cell-inside cell", target in blocked(off_y))
    # A cell one row further out stays free (no gross over-block).
    check("cell beyond margin stays free", (0, 13) not in blocked(off_y))

    # iter_pad_blocked_cells (plane generator) must agree with the array twin.
    gen = {(gx, gy) for gx, gy in
           iter_pad_blocked_cells(pad_gx, pad_gy, half_w, half_h, margin, grid,
                                  corner_radius=0.0, off_y=off_y)}
    check("generator blocks the sub-cell-inside cell too", target in gen)
    check("generator matches array twin", gen == blocked(off_y))

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  pad keepout measured from the real center: blocks the")
    print("        sub-cell-inside cell (array + generator agree), no over-block")
    print("\n5/5 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
