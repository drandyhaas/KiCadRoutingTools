#!/usr/bin/env python3
"""Plane repair must not restore ripped copper ON TOP of the corridor it freed
(#495/#496, the #141 class).

_tap_pad_with_ripup rips signal nets to clear a blocked plane-pad tap. On
SUCCESS it hands every ripped piece through a collision filter before giving it
back. The FINAL-FAILURE path used to be an unconditional

    for blocker, rsegs, rvias in reversed(ripped_local):
        pcb_data.segments.extend(rsegs)
        pcb_data.vias.extend(rvias)

with no collision test at all, justified by "a failed tap adds no copper, so an
immediate restore cannot create the #141 restore-shorts". That reasoning does
not hold: other repair copper (region straps, earlier pads' taps) lands in the
freed corridor while the net is ripped, and the blanket restore then puts the
failed net's stale copper back on top of it -- exactly the behaviour issue #141
removed and this module's own docstring says was removed (rp2350
/RP2354A/FPGA.MOSI: both taps failed and its via went back over a GND strap,
0.114 mm overlap at a 0.09 rule).

The fix reuses the shared rip-up/restore collision contract
(rip_up_reroute._saved_route_collides, #134): non-colliding copper is still
given back (so #329's zero-copper nets do not return), colliding pieces are
left ripped for the mandated follow-up route.py pass.

Run:  python3 tests/test_plane_restore_on_failure.py
"""
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

from kicad_parser import BoardInfo, Net, PCBData, Pad, Segment, Via
from routing_config import GridRouteConfig
import repair_planes as rdp

GND, SIG = 2, 5
FAILURES = []


def _check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{(' -- ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(name)


def _pad(ref, num, x, y, net_id, name):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.6, size_y=0.6, shape="rect",
               layers=["F.Cu"], net_id=net_id, net_name=name, pad_type="smd")


def _board():
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = (0.0, 0.0, 40.0, 40.0)
    # The signal net runs two pieces, each anchored on its own pad so the
    # orphan filter cannot be what drops them.
    safe = Segment(start_x=2.0, start_y=2.0, end_x=6.0, end_y=2.0,
                   width=0.2, layer="F.Cu", net_id=SIG)
    doomed = Segment(start_x=20.0, start_y=20.0, end_x=24.0, end_y=20.0,
                     width=0.2, layer="F.Cu", net_id=SIG)
    pads = {
        SIG: [_pad("U1", "1", 2.0, 2.0, SIG, "/SIG"),
              _pad("U1", "2", 6.0, 2.0, SIG, "/SIG"),
              _pad("U2", "1", 20.0, 20.0, SIG, "/SIG"),
              _pad("U2", "2", 24.0, 20.0, SIG, "/SIG")],
        GND: [_pad("J9", "1", 30.0, 30.0, GND, "GND")],
    }
    pcb = PCBData(board_info=bi,
                  nets={GND: Net(GND, "GND"), SIG: Net(SIG, "/SIG")},
                  footprints={}, vias=[], segments=[safe, doomed],
                  pads_by_net=pads)
    return pcb, safe, doomed


def main():
    print("plane-repair restore-on-failure collision filter (#495/#496):")
    pcb, safe, doomed = _board()
    cfg = GridRouteConfig(track_width=0.2, clearance=0.09, via_size=0.45,
                          via_drill=0.2, grid_step=0.05)

    # While the signal net is ripped, plane repair routes a GND strap straight
    # down the freed corridor -- right over the 'doomed' segment.
    def fake_tap(*a, **k):
        if not any(s.net_id == GND for s in pcb.segments):
            pcb.segments.append(Segment(start_x=20.0, start_y=20.0,
                                        end_x=24.0, end_y=20.0, width=0.8,
                                        layer="F.Cu", net_id=GND))
        return SimpleNamespace(success=False, segments=[], via=None,
                               clearance_used=None, blocked_cells=[])

    orig_tap = rdp.tap_pad_with_escalation
    orig_blk = rdp._via_site_consensus_blocker
    rdp.tap_pad_with_escalation = fake_tap
    rdp._via_site_consensus_blocker = lambda *a, **k: SIG
    ripped_net_ids, partial_restores = [], []
    try:
        res = rdp._tap_pad_with_ripup(
            pad=pcb.pads_by_net[GND][0], pad_layer="F.Cu", net_id=GND,
            pcb_data=pcb, tap_config=cfg, blocker_config=cfg,
            max_search_radius=2.0, via_size=0.45, via_drill=0.2,
            max_rip_nets=1, protected_net_ids={GND},
            first_failure=SimpleNamespace(blocked_cells=[]),
            ripped_net_ids=ripped_net_ids, verbose=False,
            partial_restores=partial_restores)
    finally:
        rdp.tap_pad_with_escalation = orig_tap
        rdp._via_site_consensus_blocker = orig_blk

    _check("the tap still reports failure", res is None)

    sig = [s for s in pcb.segments if s.net_id == SIG]
    gnd = [s for s in pcb.segments if s.net_id == GND]
    _check("the GND strap placed in the freed corridor survives", len(gnd) == 1)

    # The doomed piece overlaps the strap exactly; it must NOT be restored.
    on_top = [s for s in sig if abs(s.start_x - 20.0) < 1e-6
              and abs(s.start_y - 20.0) < 1e-6]
    _check("colliding piece is NOT restored on top of the strap",
           not on_top, f"{len(on_top)} colliding segment(s) came back")

    # ...but the non-colliding piece still is (#329: do not strand whole nets).
    kept = [s for s in sig if abs(s.start_x - 2.0) < 1e-6]
    _check("non-colliding piece IS restored", len(kept) == 1,
           f"{len(kept)} kept")

    _check("the partial restore is recorded for the writer",
           len(partial_restores) == 1 and partial_restores[0][0] == SIG,
           f"partial_restores={[(p[0], p[3]) for p in partial_restores]}")

    if FAILURES:
        print(f"\nFAIL: {len(FAILURES)} case(s): {', '.join(FAILURES)}")
        return 1
    print("\nPASS: failure-path restore honors the shared collision contract.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
