#!/usr/bin/env python3
"""Issue #349: adjacent same-net plane pads share one via + straps.

A cluster of adjacent same-net pads used to get one tap via PER PAD: via reuse
only fired when an existing via was the cheapest target for that specific pad,
and nothing knew the pads were one cluster (castor_pollux U9/U13 +3.3VA pads
6/7/8 = three vias with ~1.5mm stubs). Two changes:

- create_plane (route_planes.py): before drilling, strap to an adjacent
  already-connected same-net pad within PLANE_PAD_STRAP_RADIUS (verified on
  castor_pollux: 10 straps, 128 -> 118 vias, DRC/connectivity identical).
- the plane repair pass (route_disconnected_planes) now enables the
  trace-to-connected-copper step (#180) at strap scale even without
  --rip-blocker-nets.

This test isolates the repair-side mechanism: a pad 1.3mm from its
plane-connected same-net neighbour (just past the close-via-reuse radius)
must connect by TRACE (no new drill) when the strap-scale radius is on, and
only by a NEW via when it is off (the old default).

Run:  python3 tests/test_plane_pad_strap.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import Pad, Via, PCBData, BoardInfo
from routing_config import GridRouteConfig
from plane_pad_tap import try_tap_pad
import routing_defaults as defaults


def _board():
    """Pad A at (10,10) carries a same-net via (plane-connected); pad B sits
    1.3mm away -- past close-via reuse (via_size*2.5 = 1.0) but within the
    strap radius (1.5)."""
    pad_a = Pad(component_ref="U1", pad_number="6", global_x=10.0, global_y=10.0,
                local_x=0.0, local_y=0.0, size_x=0.4, size_y=0.4, shape="circle",
                layers=["B.Cu"], net_id=1, net_name="+3.3VA")
    pad_b = Pad(component_ref="U1", pad_number="8", global_x=11.3, global_y=10.0,
                local_x=0.0, local_y=0.0, size_x=0.4, size_y=0.4, shape="circle",
                layers=["B.Cu"], net_id=1, net_name="+3.3VA")
    via_a = Via(x=10.0, y=10.0, size=0.4, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=1)
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = (0.0, 0.0, 20.0, 20.0)
    pcb = PCBData(board_info=bi, nets={}, footprints={},
                  vias=[via_a], segments=[], pads_by_net={1: [pad_a, pad_b]})
    return pcb, pad_b


def _config():
    return GridRouteConfig(track_width=0.127, clearance=0.1, via_size=0.4,
                           via_drill=0.2, grid_step=0.05, board_edge_clearance=0.2,
                           hole_to_hole_clearance=0.2)


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # Strap-scale radius ON (the #349 repair-loop default): connect by trace.
    pcb, pad_b = _board()
    r = try_tap_pad(pad_b, "B.Cu", 1, pcb, _config(),
                    max_search_radius=3.0, via_size=0.4, via_drill=0.2,
                    distant_trace_radius=defaults.PLANE_PAD_STRAP_RADIUS)
    check("strap radius on: pad connects", r.success)
    check("strap radius on: NO new via drilled", r.success and r.via is None,
          f"via={r.via}")
    check("strap radius on: real strap copper emitted",
          r.success and len(r.segments) > 0, f"{len(r.segments)} seg(s)")

    # Strap-scale radius OFF (the old no-rip-blocker-nets default): the same pad
    # gets its own drill.
    pcb2, pad_b2 = _board()
    r2 = try_tap_pad(pad_b2, "B.Cu", 1, pcb2, _config(),
                     max_search_radius=3.0, via_size=0.4, via_drill=0.2,
                     distant_trace_radius=0.0)
    check("strap radius off: pad still connects", r2.success)
    check("strap radius off: a NEW via is drilled (the #349 waste)",
          r2.success and r2.via is not None)

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(run())
