#!/usr/bin/env python3
"""Plane-pad straps must not chain past the strap radius.

The repair pass's trace-to-connected-copper step (#180, at strap scale since
#349) accepts any same-net SMD pad with same-net copper on it as a target. A
strap laid by an earlier tap counts as that copper, so each new strap becomes
the next pad's target and a row of same-net pads grows into one web whose far
end is several mm of track from the only via (on ulx3s GND, U1.W16 reached its
via through 7 other pads and 5.6 mm of track). The radius must bound the whole
track from the pad to the plane, not one link.

Board: pad A carries the via; pad B, 1.3 mm away, is strapped to A; pad C sits
1.3 mm past B. C->B is within the 1.5 mm strap radius, but C's track to the
plane through B would be 2.6 mm, so C must get its own via. A pad strapped to a
directly escaped neighbour (the #349 case) still connects by trace.

Run:  python3 tests/test_plane_pad_strap_chain.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import Pad, Via, Segment, PCBData, BoardInfo
from routing_config import GridRouteConfig
from plane_pad_tap import try_tap_pad
import routing_defaults as defaults


def _pad(num, x):
    return Pad(component_ref="U1", pad_number=num, global_x=x, global_y=10.0,
               local_x=0.0, local_y=0.0, size_x=0.4, size_y=0.4, shape="circle",
               layers=["B.Cu"], net_id=1, net_name="GND")


def _board(strap_b_to_a=True):
    pad_a, pad_b, pad_c = _pad("1", 10.0), _pad("2", 11.3), _pad("3", 12.6)
    via_a = Via(x=10.0, y=10.0, size=0.4, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=1)
    segs = []
    if strap_b_to_a:
        segs.append(Segment(start_x=11.3, start_y=10.0, end_x=10.0, end_y=10.0,
                            width=0.127, layer="B.Cu", net_id=1))
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = (0.0, 0.0, 20.0, 20.0)
    pcb = PCBData(board_info=bi, nets={}, footprints={}, vias=[via_a],
                  segments=segs, pads_by_net={1: [pad_a, pad_b, pad_c]})
    return pcb, pad_b, pad_c


def _config():
    return GridRouteConfig(track_width=0.127, clearance=0.1, via_size=0.4,
                           via_drill=0.2, grid_step=0.05, board_edge_clearance=0.2,
                           hole_to_hole_clearance=0.2)


def _tap(pad, pcb):
    return try_tap_pad(pad, "B.Cu", 1, pcb, _config(),
                       max_search_radius=3.0, via_size=0.4, via_drill=0.2,
                       distant_trace_radius=defaults.PLANE_PAD_STRAP_RADIUS)


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # C's only in-radius target is B, whose own track to the plane is 1.3 mm:
    # 1.3 + 1.3 > 1.5, so C must not strap to B.
    pcb, _b, pad_c = _board()
    r = _tap(pad_c, pcb)
    check("chained pad connects", r.success)
    check("chained pad gets its own via, no strap to the strapped neighbour",
          r.success and r.via is not None,
          f"via={r.via} reused={r.reused_via_pos}")

    # B next to the via-carrying pad A still straps by trace (unchanged).
    pcb2, pad_b2, _c = _board(strap_b_to_a=False)
    r2 = _tap(pad_b2, pcb2)
    check("pad next to an escaped pad still straps (no new via)",
          r2.success and r2.via is None and len(r2.segments) > 0,
          f"via={r2.via}")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(run())
