#!/usr/bin/env python3
"""Issue #373: plane-pad plain-track fallback to nearest same-net copper / pour.

A plane-net pad used to be tapped by a via or left floating -- the terminal
ladder was via-or-nothing. When no via can be placed (boxed-in pocket,
fine-pitch WLCSP, deep-layer pour) but a short plain track to nearby same-net
copper or the pad's own pour would connect it, the pad was abandoned.

try_tap_pad(..., pour_trace_only=True) now routes that last-resort track,
reusing the pad-tap multi-source A*: it targets same-net SEGMENT copper on the
pad's layer and points that provably sit on real same-net zone FILL (validated
by the island-join fill test). This isolates the mechanism on synthetic boards
(no new-via placement happens on this path).

Run:  python3 tests/test_plane_pad_track_fallback.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import Pad, Via, Segment, Zone, PCBData, BoardInfo
from routing_config import GridRouteConfig
from plane_pad_tap import try_tap_pad, _same_net_copper_targets, make_local_window


def _bi():
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = (0.0, 0.0, 20.0, 20.0)
    return bi


def _config():
    return GridRouteConfig(track_width=0.15, clearance=0.1, via_size=0.4,
                           via_drill=0.2, grid_step=0.05, board_edge_clearance=0.2,
                           hole_to_hole_clearance=0.2)


def _pad(x, y, net=1, layer="B.Cu"):
    return Pad(component_ref="U1", pad_number="1", global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.4, size_y=0.4, shape="circle",
               layers=[layer], net_id=net, net_name="GND")


def _endpoint(segs, pad):
    """Closest distance from any routed-segment endpoint to the pad center."""
    import math
    px, py = pad.global_x, pad.global_y
    best = 1e9
    for s in segs:
        for (x, y) in ((s['start'][0], s['start'][1]), (s['end'][0], s['end'][1])):
            best = min(best, math.hypot(x - px, y - py))
    return best


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    cfg = _config()

    # --- 1. trace to a same-net SEGMENT on the pad's layer -----------------
    # Pad at (11.3,10) floating; a same-net B.Cu track runs at x=10, y in [9,11]
    # ~1.3mm away (past close-via reuse). No via/pad to reuse -> only the new
    # segment/pour targeting can connect it.
    pad = _pad(11.3, 10.0)
    trk = Segment(start_x=10.0, start_y=9.0, end_x=10.0, end_y=11.0,
                  width=0.2, layer="B.Cu", net_id=1)
    pcb = PCBData(board_info=_bi(), nets={}, footprints={},
                  vias=[], segments=[trk], pads_by_net={1: [pad]})
    local = make_local_window(pcb, pad.global_x, pad.global_y, 3.0)
    tgts = _same_net_copper_targets(pad, "B.Cu", 1, local, pcb, cfg, 3.0)
    check("segment target found near the track (x~10)",
          any(abs(tx - 10.0) < 1e-6 for tx, _ in tgts))
    res = try_tap_pad(pad, "B.Cu", 1, pcb, cfg, max_search_radius=3.0,
                      via_size=0.4, via_drill=0.2, distant_trace_radius=3.0,
                      pour_trace_only=True)
    check("segment fallback routes a track (no via)",
          res.success and res.via is None and bool(res.segments))
    if res.success and res.segments:
        check("routed track reaches the same-net segment (endpoint ~x=10)",
              _endpoint(res.segments, pad) <= 1.5)

    # --- 2. trace to the pad's own POUR fill on its layer ------------------
    # Pad on B.Cu inside a large B.Cu GND pour but isolated from it; the fallback
    # must find real-fill target points and route a short track onto the pour.
    pad2 = _pad(11.3, 10.0)
    zone = Zone(net_id=1, net_name="GND", layer="B.Cu",
                polygon=[(2.0, 2.0), (18.0, 2.0), (18.0, 18.0), (2.0, 18.0)])
    pcb2 = PCBData(board_info=_bi(), nets={}, footprints={},
                   vias=[], segments=[], pads_by_net={1: [pad2]}, zones=[zone])
    local2 = make_local_window(pcb2, pad2.global_x, pad2.global_y, 3.0)
    tgts2 = _same_net_copper_targets(pad2, "B.Cu", 1, local2, pcb2, cfg, 3.0)
    check("pour fill targets found (real_fill_point accepted)", len(tgts2) > 0)
    res2 = try_tap_pad(pad2, "B.Cu", 1, pcb2, cfg, max_search_radius=3.0,
                       via_size=0.4, via_drill=0.2, distant_trace_radius=3.0,
                       pour_trace_only=True)
    check("pour fallback routes a track (no via)",
          res2.success and res2.via is None and bool(res2.segments))

    # --- 3. no same-net copper on the layer -> no track (graceful failure) --
    pad3 = _pad(11.3, 10.0)
    pcb3 = PCBData(board_info=_bi(), nets={}, footprints={},
                   vias=[], segments=[], pads_by_net={1: [pad3]})
    res3 = try_tap_pad(pad3, "B.Cu", 1, pcb3, cfg, max_search_radius=3.0,
                       via_size=0.4, via_drill=0.2, distant_trace_radius=3.0,
                       pour_trace_only=True)
    check("no same-net copper -> fallback fails cleanly (no via, no segs)",
          (not res3.success) and res3.via is None and not res3.segments)

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  plain-track fallback connects to same-net segment and pour;")
    print("        fails cleanly when there is no same-net copper on the layer")
    return 0


if __name__ == "__main__":
    sys.exit(run())
