#!/usr/bin/env python3
"""SharedViaMaps (issue #263): cross-pad whole-board via-map reuse.

The plane-repair pass shares ONE whole-board via-placement obstacle map across
all pads of a net-pass instead of rebuilding a window map per pad. This is only
correct if:

  1. EQUIVALENCE: for any pad, the shared whole-board map answers the tap's
     queries (pad-centre blocked + open_via_cells_within the search radius)
     exactly like the per-pad window map it replaces.
  2. INCREMENTAL ADD: after a tap's via is appended to pcb_data and notified,
     every cached map matches a fresh whole-board rebuild (the maps are
     refcounted, #208 -- the add must stamp the builder's exact multiset).
  3. INCREMENTAL RIP: after a blocker net is removed and notified, ditto for
     the remove multiset (over-removal wrongly unblocks copper other nets
     still need; under-removal leaves stale blocks).
  4. STALENESS GUARD: a pcb_data copper change WITHOUT a notify must not leave
     a stale cached map in use.

Run:  python3 tests/test_shared_via_maps.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb, Via
from routing_config import GridRouteConfig, GridCoord
from plane_obstacle_builder import build_via_obstacle_map
from plane_pad_tap import SharedViaMaps, make_local_window, _WINDOW_MARGIN
from pcb_modification import remove_net_from_pcb_data

FIXTURE = os.path.join(ROOT, "kicad_files", "routed_output.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu",
          "In5.Cu", "In6.Cu", "In7.Cu", "In8.Cu", "B.Cu"]
RIP_NET = "Net-(U2A-DATA_4)"
SEARCH_RADIUS = 3.0  # mm


def _net_id(pcb, name):
    return next(nid for nid, n in pcb.nets.items() if n.name == name)


def main():
    pcb = parse_kicad_pcb(FIXTURE)
    gnd = _net_id(pcb, "GND")
    config = GridRouteConfig(layers=LAYERS, grid_step=0.05, track_width=0.127,
                             clearance=0.1, via_size=0.5, via_drill=0.3,
                             hole_to_hole_clearance=0.5)
    coord = GridCoord(config.grid_step)
    pads = [p for p in pcb.pads_by_net[gnd] if p.drill == 0][:4]
    assert pads, "fixture must have SMD GND pads"

    shared = SharedViaMaps(pcb, gnd)

    # Below the reuse threshold get() defers to the window path.
    assert shared.get(config) is None and shared.get(config) is None, \
        "keys below the reuse threshold must not build a whole-board map"
    m = shared.get(config)
    assert m is not None, "third use must build the shared map"

    # 1. Equivalence vs the per-pad window map, per pad.
    for pad in pads:
        local = make_local_window(pcb, pad.global_x, pad.global_y,
                                  SEARCH_RADIUS + _WINDOW_MARGIN)
        ref = build_via_obstacle_map(local, config, gnd, verbose=False)
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        assert m.is_via_blocked(gx, gy) == ref.is_via_blocked(gx, gy)
        r = coord.to_grid_dist(SEARCH_RADIUS)
        assert m.open_via_cells_within(gx, gy, r) == \
            ref.open_via_cells_within(gx, gy, r), \
            f"shared map != window map around {pad.component_ref}.{pad.pad_number}"
    print(f"PASS: shared map == window map for {len(pads)} pads "
          f"({SEARCH_RADIUS}mm radius)")

    # 2. Incremental add of a new tap via == fresh rebuild.
    pad = pads[0]
    new_via = Via(x=pad.global_x, y=pad.global_y, size=config.via_size,
                  drill=config.via_drill, layers=['F.Cu', 'B.Cu'], net_id=gnd)
    pcb.vias.append(new_via)
    shared.note_pass_copper([new_via], [])
    shared.verify_maps_full()  # asserts vs fresh whole-board rebuild
    print("PASS: incremental via add == fresh rebuild")

    # 3. Incremental rip of a dense signal net == fresh rebuild.
    rip = _net_id(pcb, RIP_NET)
    assert any(s.net_id == rip for s in pcb.segments), "rip net must have copper"
    shared.note_net_ripped(rip)
    remove_net_from_pcb_data(pcb, rip)
    shared.resync()
    shared.verify_maps_full()
    print(f"PASS: incremental rip of {RIP_NET} == fresh rebuild")

    # 4. Un-notified copper change drops cached maps instead of serving stale.
    pcb.vias.pop()  # silently undo the tap via
    m2 = shared.get(config)
    assert m2 is not None and m2 is not m, \
        "un-notified copper change must invalidate cached maps"
    shared.verify_maps_full()
    print("PASS: un-notified copper change invalidates the cache")


if __name__ == "__main__":
    main()
