#!/usr/bin/env python3
"""Tests for bga_fanout handling of non-orthogonally-placed BGAs (issue #137).

Builds a small synthetic ball grid placed at 45° and checks that:
  - analyze_bga_grid FAILS on it in the global frame (the original bug), and
  - the rotate-into-local-frame transform recovers a correct axis-aligned grid,
  - the forward/back transforms round-trip.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Footprint, PCBData, BoardInfo, local_to_global
from bga_fanout.grid import analyze_bga_grid
from bga_fanout.rotate_frame import (is_orthogonal, to_axis_aligned_frame,
                                     back_transform_results)


def _make_bga(ref, cx, cy, rotation, n=5, pitch=0.8):
    """n x n ball grid, footprint placed at `rotation` about (cx, cy)."""
    fp = Footprint(reference=ref, footprint_name="BGA-TEST", x=cx, y=cy,
                   rotation=rotation, layer="F.Cu", value="")
    pads = []
    half = (n - 1) / 2.0
    k = 1
    for r in range(n):
        for c in range(n):
            lx = (c - half) * pitch
            ly = (r - half) * pitch
            gx, gy = local_to_global(cx, cy, rotation, lx, ly)
            pads.append(Pad(
                component_ref=ref, pad_number=f"{k}", global_x=gx, global_y=gy,
                local_x=lx, local_y=ly, size_x=0.4, size_y=0.4, shape="circle",
                layers=["F.Cu"], net_id=k, net_name=f"n{k}", rotation=0.0))
            k += 1
    fp.pads = pads
    bi = BoardInfo(layers={}, copper_layers=["F.Cu", "B.Cu"], board_bounds=None)
    pbn = {}
    for p in pads:
        pbn.setdefault(p.net_id, []).append(p)
    pcb = PCBData(board_info=bi, nets={}, footprints={ref: fp}, vias=[],
                  segments=[], pads_by_net=pbn)
    return pcb


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return cond


def main():
    results = []

    # 0/90/180/270 are orthogonal; 45/-45/30 are not.
    results.append(_ok("is_orthogonal classifies angles",
                       is_orthogonal(0) and is_orthogonal(90) and is_orthogonal(-90)
                       and not is_orthogonal(45) and not is_orthogonal(-45)
                       and not is_orthogonal(30)))

    # A 5x5 BGA at 45° is NOT recognised in the global frame (the bug).
    pcb45 = _make_bga("U1", 100.0, 100.0, 45.0, n=5)
    g_global = analyze_bga_grid(pcb45.footprints["U1"])
    bad = g_global is None or len(g_global.rows) != 5 or len(g_global.cols) != 5
    results.append(_ok("global-frame grid analysis fails on 45° BGA", bad))

    # After rotating into the footprint frame, the grid is recovered (5x5).
    rp, back = to_axis_aligned_frame(pcb45, "U1")
    g_local = analyze_bga_grid(rp.footprints["U1"])
    good = (g_local is not None and len(g_local.rows) == 5 and len(g_local.cols) == 5
            and abs(g_local.pitch_x - 0.8) < 0.01 and abs(g_local.pitch_y - 0.8) < 0.01)
    results.append(_ok("transformed grid recovers 5x5 / pitch 0.8", good))

    # The transformed footprint is axis-aligned (rotation 0).
    results.append(_ok("transformed footprint rotation == 0",
                       rp.footprints["U1"].rotation == 0.0))

    # Forward (real->rotated) then back (rotated->real) round-trips a point.
    fp = pcb45.footprints["U1"]
    px, py = fp.pads[7].global_x, fp.pads[7].global_y      # real position
    rpx, rpy = rp.footprints["U1"].pads[7].global_x, rp.footprints["U1"].pads[7].global_y
    bx, by = back(rpx, rpy)
    results.append(_ok("back(transform(p)) == p",
                       abs(bx - px) < 1e-6 and abs(by - py) < 1e-6))

    # back_transform_results maps track/via dicts in place.
    tracks = [{"start": (rpx, rpy), "end": (rpx + 1.0, rpy)}]
    vias = [{"x": rpx, "y": rpy}]
    back_transform_results(tracks, vias, [], back)
    results.append(_ok("back_transform_results maps a track start to the real ball",
                       abs(tracks[0]["start"][0] - px) < 1e-6
                       and abs(tracks[0]["start"][1] - py) < 1e-6
                       and abs(vias[0]["x"] - px) < 1e-6))

    passed = sum(results)
    print(f"\n{passed}/{len(results)} bga rotation tests passed")
    print("=" * 60)
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
