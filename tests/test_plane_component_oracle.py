#!/usr/bin/env python3
"""T6 "mutual-floating-strap" false success: plane-component oracle.

A plane tap used to accept ANY nearby same-net copper as its target -- including
a FLOATING island/strap of the plane (copper another tap just placed, a stale
input via outside the pour). Two pads could strap to each other's island and
both report success while both stayed disconnected from the plane; the
authoritative check_net_connectivity later graded the net incomplete.

PlaneComponentOracle computes the net's connected components once from the same
union-find graph the grader uses, identifies the MAIN plane component (the
zone-credited component with the most pads), and try_tap_pad(plane_oracle=...)
rejects tap targets outside it -- with an oracle-strict terminal gate so a
would-be false success becomes an honest failure.

Run:  python3 tests/test_plane_component_oracle.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import Pad, Via, Segment, Zone, PCBData, BoardInfo
from routing_config import GridRouteConfig
from plane_component_oracle import PlaneComponentOracle
from plane_pad_tap import try_tap_pad
from check_connected import check_net_connectivity

GND = 1


def _bi():
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = (0.0, 0.0, 30.0, 30.0)
    return bi


def _config():
    return GridRouteConfig(track_width=0.15, clearance=0.1, via_size=0.4,
                           via_drill=0.2, grid_step=0.05, board_edge_clearance=0.2,
                           hole_to_hole_clearance=0.2)


def _pad(x, y, ref="U1", num="1", layer="F.Cu"):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5, shape="circle",
               layers=[layer], net_id=GND, net_name="GND")


def _board():
    """GND net: a B.Cu pour on the LEFT half (2..14), anchored by a via inside
    it; two floating F.Cu pads on the RIGHT (x~24) next to a FLOATING same-net
    via + strap island at (25, 10) that reaches no plane copper."""
    zone = Zone(net_id=GND, net_name="GND", layer="B.Cu",
                polygon=[(2.0, 2.0), (14.0, 2.0), (14.0, 28.0), (2.0, 28.0)])
    plane_via = Via(x=8.0, y=8.0, size=0.4, drill=0.2,
                    layers=["F.Cu", "B.Cu"], net_id=GND)
    # Pads that anchor the main component (through the zone credit).
    main_pad = _pad(8.0, 12.0, ref="U9", num="1", layer="B.Cu")
    main_pad2 = _pad(8.0, 16.0, ref="U9", num="2", layer="B.Cu")
    # Floating island: a via + short strap far outside the pour outline.
    float_via = Via(x=25.0, y=10.0, size=0.4, drill=0.2,
                    layers=["F.Cu", "B.Cu"], net_id=GND)
    float_seg = Segment(start_x=25.0, start_y=10.0, end_x=25.0, end_y=11.0,
                        width=0.2, layer="F.Cu", net_id=GND)
    # The two unconnected pads sitting next to the island (T6 victims).
    pad_a = _pad(24.3, 10.0, ref="C1", num="1")
    pad_b = _pad(24.3, 11.0, ref="C2", num="1")
    pcb = PCBData(board_info=_bi(), nets={}, footprints={},
                  vias=[plane_via, float_via], segments=[float_seg],
                  pads_by_net={GND: [main_pad, main_pad2, pad_a, pad_b]},
                  zones=[zone])
    return pcb, pad_a, pad_b, float_via, float_seg, plane_via, main_pad


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    cfg = _config()
    pcb, pad_a, pad_b, float_via, float_seg, plane_via, main_pad = _board()

    # --- 1. Oracle classification -----------------------------------------
    oracle = PlaneComponentOracle(pcb, GND)
    check("oracle is active (net has a zone)", not oracle.inert)
    check("in-pour via is on the main plane", oracle.via_on_plane(plane_via.x, plane_via.y))
    check("floating via is NOT on the main plane",
          not oracle.via_on_plane(float_via.x, float_via.y))
    check("floating strap segment is NOT on the main plane",
          not oracle.segment_on_plane(float_seg))
    check("zone-covered pad is on the main plane", oracle.pad_on_plane(main_pad))
    check("floating pad is NOT on the main plane", not oracle.pad_on_plane(pad_a))
    check("point inside the pour reaches the plane",
          oracle.point_reaches_plane(8.0, 20.0))
    check("point at the floating island does NOT reach the plane",
          not oracle.point_reaches_plane(25.0, 10.5))

    # --- 2. WITHOUT the oracle: the T6 false success ------------------------
    # pad_a sits 0.7mm from the floating via -- inside close-via reuse range.
    # The tap straps to it and reports success while pad_a never reaches the
    # plane (the pour is >10mm away, outside the search radius).
    res_old = try_tap_pad(pad_a, "F.Cu", GND, pcb, cfg, max_search_radius=2.0,
                          via_size=0.4, via_drill=0.2, distant_trace_radius=2.0)
    check("without oracle: tap reports success (the bug)", res_old.success)
    if res_old.success:
        segs = [Segment(start_x=s['start'][0], start_y=s['start'][1],
                        end_x=s['end'][0], end_y=s['end'][1], width=s['width'],
                        layer=s['layer'], net_id=GND) for s in res_old.segments]
        r = check_net_connectivity(
            GND, pcb.segments + segs, pcb.vias, pcb.pads_by_net[GND], pcb.zones)
        check("without oracle: grader still says net incomplete (false success)",
              not r['connected'])

    # --- 3. WITH the oracle: honest failure, no island strap ----------------
    res_new = try_tap_pad(pad_a, "F.Cu", GND, pcb, cfg, max_search_radius=2.0,
                          via_size=0.4, via_drill=0.2, distant_trace_radius=2.0,
                          plane_oracle=oracle)
    ok_honest = (not res_new.success) or oracle.verify_tap(res_new)
    check("with oracle: no unverified success (honest failure or true connect)",
          ok_honest)
    if res_new.success:
        segs = [Segment(start_x=s['start'][0], start_y=s['start'][1],
                        end_x=s['end'][0], end_y=s['end'][1], width=s['width'],
                        layer=s['layer'], net_id=GND) for s in res_new.segments]
        vias = list(pcb.vias)
        if res_new.via is not None:
            vias.append(Via(x=res_new.via['x'], y=res_new.via['y'],
                            size=res_new.via['size'], drill=res_new.via['drill'],
                            layers=["F.Cu", "B.Cu"], net_id=GND))
        r = check_net_connectivity(
            GND, pcb.segments + segs, vias, pcb.pads_by_net[GND], pcb.zones)
        # pad_b is still floating, but pad_a must no longer be disconnected.
        still = {(round(x, 3), round(y, 3)) for x, y, _l, _ref in r['disconnected_pads']}
        check("with oracle: a success really ties the pad to the plane",
              (round(pad_a.global_x, 3), round(pad_a.global_y, 3)) not in still)

    # --- 4. Mutual-strap scenario: pad_b targeting pad_a's copper -----------
    # Pretend pad_a "succeeded" onto the island (the old behavior): its strap
    # copper is same-net copper next to pad_b. The oracle must reject pad_a's
    # island copper as a target for pad_b too (no success chain off an island).
    pcb.segments.append(Segment(start_x=24.3, start_y=10.0, end_x=25.0, end_y=10.0,
                                width=0.15, layer="F.Cu", net_id=GND))
    oracle2 = PlaneComponentOracle(pcb, GND)
    check("island strap of pad_a is NOT main for pad_b's oracle",
          not oracle2.segment_on_plane(pcb.segments[-1]))
    res_b = try_tap_pad(pad_b, "F.Cu", GND, pcb, cfg, max_search_radius=2.0,
                        via_size=0.4, via_drill=0.2, distant_trace_radius=2.0,
                        plane_oracle=oracle2)
    check("with oracle: pad_b does not false-succeed onto pad_a's island",
          (not res_b.success) or oracle2.verify_tap(res_b))
    pcb.segments.pop()

    # --- 5. Transitive credit: a VERIFIED tap's copper becomes a target -----
    # Register a verified via for pad_a inside the pour; pad_b may then target
    # pad_a (its pad is now main-credited).
    good_via = {'x': 8.0, 'y': 20.0, 'size': 0.4, 'drill': 0.2,
                'layers': ['F.Cu', 'B.Cu'], 'net_id': GND}
    oracle.note_tap_committed(pad_a, [good_via], [])
    check("committed via is credited as main", oracle.via_on_plane(8.0, 20.0))
    check("committed pad is credited as main", oracle.pad_on_plane(pad_a))

    # --- 6. Verified new-via path: via inside a MAIN zone passes the gate ---
    pad_near = _pad(13.0, 20.0, ref="C3", num="1")  # 1mm from pour interior
    pcb.pads_by_net[GND].append(pad_near)
    oracle3 = PlaneComponentOracle(pcb, GND)
    res_v = try_tap_pad(pad_near, "F.Cu", GND, pcb, cfg, max_search_radius=3.0,
                        via_size=0.4, via_drill=0.2,
                        plane_oracle=oracle3)
    check("pad near the pour: oracle tap succeeds with a real via",
          res_v.success and res_v.via is not None
          and oracle3.verify_tap(res_v))

    # --- 7. No zones -> oracle inert (permissive, pre-oracle behavior) ------
    pcb_nz = PCBData(board_info=_bi(), nets={}, footprints={},
                     vias=[Via(x=5.0, y=5.0, size=0.4, drill=0.2,
                               layers=["F.Cu", "B.Cu"], net_id=GND)],
                     segments=[], pads_by_net={GND: [_pad(5.5, 5.0)]})
    o_nz = PlaneComponentOracle(pcb_nz, GND)
    check("zone-less net: oracle inert, all queries permissive",
          o_nz.inert and o_nz.via_on_plane(5.0, 5.0)
          and o_nz.point_reaches_plane(9.9, 9.9))

    print("=" * 60)
    if fails:
        print(f"\n{len(fails)} failure(s): {fails}")
        return 1
    print("  PASS  oracle identifies the main plane component, rejects floating")
    print("        islands as tap targets, and turns false successes honest")
    return 0


if __name__ == "__main__":
    sys.exit(run())
