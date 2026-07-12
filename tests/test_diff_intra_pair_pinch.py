#!/usr/bin/env python3
"""Issue #357: route_diff intra-pair P/N pinches below clearance.

Two of the three #357 sub-classes have direct unit coverage here (the third,
the bare-pad target-swap stub geometry, is covered by the ulx5m/open_weather_
station board replays):

1. `_collapse_leg_attach_join` gated its fix on the FULL clearance -- but
   partner_segs is the same pair's other half, which legitimately runs at the
   design gap (< clearance on open_weather_station: gap 0.15, clearance 0.2).
   The gate could therefore never pass and the grid->float join jog that
   pinched RD+/RD- to 0.25mm center (0.123 edge, floor 0.15) shipped. The gate
   now uses the intra-pair floor min(clearance, diff_pair_gap).

2. `stub_clear_of_foreign_tracks` compared CENTERLINE distance against
   config.track_width/2 + clearance -- ignoring the other track's half-width
   and the moved stub's real (netclass) width. ulx5m: a 0.125-wide stub
   layer-swapped onto a corridor 0.2mm from a 0.125-wide foreign stub passed
   (0.2 >= 0.15) but needs 0.225.

    python3 tests/test_diff_intra_pair_pinch.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Segment, Net, PCBData, BoardInfo
from routing_config import GridRouteConfig
from diff_pair_routing import _collapse_leg_attach_join
from stub_layer_switching import stub_clear_of_foreign_tracks


def _seg(x1, y1, x2, y2, w, layer, net):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=w, layer=layer, net_id=net)


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # --- 1. collapse gate at the intra-pair floor (open_weather_station RD) ---
    # Exact board geometry: RD- runs from its via along y=138.4, overshoots the
    # coupled-leg start (114.15) to the grid cell at 114.2, and a 0.05 back-jog
    # bridges the gap -- tips 0.25 from the partner copper on the same line
    # (edge 0.10 < floor 0.15). Collapsing onto the float attach point gives
    # 0.30 center = 0.15 edge = exactly the floor.
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.15,
                          clearance=0.2, diff_pair_gap=0.15)
    leg = [_seg(113.0, 138.4, 114.2, 138.4, 0.15, 'B.Cu', 146),
           _seg(114.2, 138.4, 114.15, 138.4, 0.15, 'B.Cu', 146)]
    partner = [_seg(114.45, 138.4, 114.45, 137.9, 0.15, 'B.Cu', 147),
               _seg(114.5, 138.4, 114.45, 138.4, 0.15, 'B.Cu', 147)]
    _collapse_leg_attach_join(leg, (114.15, 138.4), cfg, None, 146, partner)
    check("join jog collapsed at the intra-pair floor", len(leg) == 1,
          f"{len(leg)} segs left")
    check("leg corner moved onto the float attach point",
          abs(leg[-1].end_x - 114.15) < 1e-9 and abs(leg[-1].end_y - 138.4) < 1e-9)

    # A junction that is CLEAN at the floor must be left untouched.
    leg2 = [_seg(113.0, 138.4, 114.2, 138.4, 0.15, 'B.Cu', 146),
            _seg(114.2, 138.4, 114.15, 138.4, 0.15, 'B.Cu', 146)]
    far_partner = [_seg(114.8, 138.4, 114.8, 137.9, 0.15, 'B.Cu', 147)]
    _collapse_leg_attach_join(leg2, (114.15, 138.4), cfg, None, 146, far_partner)
    check("clean junction left untouched", len(leg2) == 2)

    # --- 2. width-aware swap validator (ulx5m TX2_P onto TX1_P's corridor) ---
    cfg2 = GridRouteConfig(layers=['F.Cu', 'In3.Cu', 'B.Cu'], track_width=0.1,
                           clearance=0.1)
    resident = _seg(92.16, 83.28, 92.16, 85.41, 0.125, 'In3.Cu', 148)
    pcb = PCBData(footprints={}, nets={148: Net(148, 'TX1_P'), 134: Net(134, 'TX2_P')},
                  segments=[resident], vias=[],
                  board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'In3.Cu', 'B.Cu']),
                  pads_by_net={})
    moved = [_seg(92.36, 82.48, 92.36, 85.41, 0.125, 'In3.Cu', 134)]
    ok, why = stub_clear_of_foreign_tracks(moved, 'In3.Cu', 134, pcb, cfg2, set())
    check("0.125-wide stub 0.2 from 0.125-wide track is rejected (needs 0.225)",
          not ok, why)

    # Same corridor at nominal 0.1 widths needs exactly 0.2 -> still legal.
    resident.width = 0.1
    moved01 = [_seg(92.36, 82.48, 92.36, 85.41, 0.1, 'In3.Cu', 134)]
    ok2, why2 = stub_clear_of_foreign_tracks(moved01, 'In3.Cu', 134, pcb, cfg2, set())
    check("0.1-wide stub 0.2 from 0.1-wide track still passes (needs 0.2)",
          ok2, why2)

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())
