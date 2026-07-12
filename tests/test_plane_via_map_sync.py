#!/usr/bin/env python3
"""Plane obstacle-map sync: incremental rip == fresh rebuild (issue #208).

route_planes places plane tap vias against an obstacle map it updates
INCREMENTALLY for speed: when it rips a blocker net to make room, it removes that
net's cells with `obstacle_map.remove_blocked_vias_batch(precompute_via_placement_
obstacles(net).blocked_vias)` (and the analogous routing-cell removal) instead of
rebuilding. The map is REFERENCE COUNTED, so this is only correct if the cell
multiset a net's REMOVE decrements is byte-identical to the multiset its ADD
(`build_via_obstacle_map` / `build_routing_obstacle_map`) stamped.

It wasn't: the remove cache (`precompute_via_placement_obstacles`) stamped each
segment as a bresenham line x circle disc with no half-cell cushion, while the add
side uses the exact capsule + cushion. The remove decremented a near cell ~7x where
the add added it once, so ripping ONE net drove SHARED cells' ref-counts to zero
and wrongly UNBLOCKED copper a still-present net needed -> route_planes then placed
a tap via grazing it (the diff-pair-dense DRC regression in #208).

INVARIANT (this test): for any net, build the full maps, incrementally remove that
net, and compare to a FRESH rebuild of the board without that net. They must agree
cell-for-cell over the net's region:
  * OVER-removal (incremental free, fresh blocked) == 0  -> the graze-causing bug.
  * UNDER-removal (incremental blocked, fresh free) == 0 -> stale leftover blocks.

The same property should hold for route.py's incremental obstacle path
(obstacle_map.add/remove_*_list_as_obstacles); see the stub at the bottom -- the
add/remove SYMMETRY half is already covered by test_obstacle_addremove_parity.py.

Run:  python3 tests/test_plane_via_map_sync.py [-v]
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np
from kicad_parser import parse_kicad_pcb
from routing_config import GridRouteConfig, GridCoord
from plane_obstacle_builder import build_via_obstacle_map, build_routing_obstacle_map
from obstacle_cache import precompute_via_placement_obstacles
from pcb_modification import remove_net_from_pcb_data

# A committed, routed multilayer board with a GND plane and dense signal nets.
FIXTURE = os.path.join(ROOT, "kicad_files", "routed_output.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu",
          "In5.Cu", "In6.Cu", "In7.Cu", "In8.Cu", "B.Cu"]
# Dense signal nets (segments + vias) whose keep-outs overlap their neighbours --
# exactly the shared-cell case where over-removal shows. GND is the excluded
# (plane) net being routed, as in a real route_planes pass.
RIP_NETS = ["Net-(U2A-~{SIWU})", "Net-(U2A-~{WAKEUP})",
            "Net-(U2A-DATA_4)", "Net-(U2A-DATA_6)"]
EXCLUDE_NET = "GND"


def _config():
    return GridRouteConfig(layers=LAYERS, grid_step=0.05, track_width=0.127,
                           clearance=0.1, via_size=0.5, via_drill=0.3)


def _net_id(pcb, name):
    for nid, n in pcb.nets.items():
        if n.name == name:
            return nid
    return None


def _net_bbox_grid(pcb, net_id, coord):
    xs, ys = [], []
    for s in pcb.segments:
        if s.net_id == net_id:
            xs += [s.start_x, s.end_x]; ys += [s.start_y, s.end_y]
    for v in pcb.vias:
        if v.net_id == net_id:
            xs.append(v.x); ys.append(v.y)
    g_lo = coord.to_grid(min(xs) - 1.0, min(ys) - 1.0)
    g_hi = coord.to_grid(max(xs) + 1.0, max(ys) + 1.0)
    return g_lo[0], g_lo[1], g_hi[0], g_hi[1]


def _via_map_mismatch(inc, fresh, bbox):
    """(over, under) cells where the incremental and fresh VIA maps disagree."""
    g0x, g0y, g1x, g1y = bbox
    over = under = 0
    for gx in range(g0x, g1x + 1):
        for gy in range(g0y, g1y + 1):
            a = inc.is_via_blocked(gx, gy); b = fresh.is_via_blocked(gx, gy)
            if a and not b:
                under += 1
            elif b and not a:
                over += 1
    return over, under


def _routing_map_mismatch(inc, fresh, bbox):
    g0x, g0y, g1x, g1y = bbox
    over = under = 0
    for gx in range(g0x, g1x + 1):
        for gy in range(g0y, g1y + 1):
            a = inc.is_blocked(gx, gy, 0); b = fresh.is_blocked(gx, gy, 0)
            if a and not b:
                under += 1
            elif b and not a:
                over += 1
    return over, under


def run(verbose=False):
    config = _config()
    coord = GridCoord(config.grid_step)
    fails = []

    base = parse_kicad_pcb(FIXTURE)
    excl = _net_id(base, EXCLUDE_NET)
    if excl is None:
        return [f"fixture missing excluded net {EXCLUDE_NET}"]

    for name in RIP_NETS:
        pcb = parse_kicad_pcb(FIXTURE)
        net = _net_id(pcb, name)
        if net is None:
            fails.append(f"{name}: not in fixture"); continue
        bbox = _net_bbox_grid(pcb, net, coord)

        # --- VIA placement map -------------------------------------------------
        mv = build_via_obstacle_map(pcb, config, excl)
        cache = precompute_via_placement_obstacles(pcb, net, config, LAYERS)
        if len(cache.blocked_vias):
            mv.remove_blocked_vias_batch(cache.blocked_vias)
        pcb_minus = parse_kicad_pcb(FIXTURE)
        remove_net_from_pcb_data(pcb_minus, net)
        mv_fresh = build_via_obstacle_map(pcb_minus, config, excl)
        over, under = _via_map_mismatch(mv, mv_fresh, bbox)
        if over:
            fails.append(f"{name}: VIA map OVER-removed {over} cells (graze risk)")
        if under:
            fails.append(f"{name}: VIA map UNDER-removed {under} cells (stale)")
        if verbose:
            print(f"  {name:22} VIA  over={over} under={under}")

        # --- Routing map (one representative layer the net actually uses) -------
        seg_layers = {s.layer for s in pcb.segments if s.net_id == net and s.layer in LAYERS}
        for layer in sorted(seg_layers)[:1]:
            mr = build_routing_obstacle_map(pcb, config, excl, layer)
            cells = cache.blocked_cells_by_layer.get(layer)
            if cells is not None and len(cells):
                c3 = np.column_stack([cells, np.zeros(len(cells), dtype=np.int32)])
                mr.remove_blocked_cells_batch(c3)
            mr_fresh = build_routing_obstacle_map(pcb_minus, config, excl, layer)
            rover, runder = _routing_map_mismatch(mr, mr_fresh, bbox)
            if rover:
                fails.append(f"{name}: ROUTING[{layer}] OVER-removed {rover} cells")
            if runder:
                fails.append(f"{name}: ROUTING[{layer}] UNDER-removed {runder} cells")
            if verbose:
                print(f"  {name:22} ROUT[{layer}] over={rover} under={runder}")

    return fails


# --- Extension point ---------------------------------------------------------
# route.py uses obstacle_map.{add,remove}_{segments,vias}_list_as_obstacles for the
# same incremental rip/restore. The add/remove SYMMETRY (add then remove -> empty)
# is covered by test_obstacle_addremove_parity.py. To also assert the build-minus
# equivalence here, build_base_obstacle_map(pcb) - remove(net) should equal
# build_base_obstacle_map(pcb_minus_net) over the net bbox -- same _*_mismatch
# helpers, swapping the builder/remover. Left as a stub until route.py grows an
# incremental-removal regression worth pinning.


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    print("=== plane via/routing map: incremental rip == fresh rebuild (#208) ===")
    fails = run(args.verbose)
    if fails:
        print("\nFAIL:\n  " + "\n  ".join(fails))
        return 1
    print(f"\nPASS: incremental removal == fresh rebuild for {len(RIP_NETS)} nets "
          f"(via + routing maps, no over- or under-removal)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
