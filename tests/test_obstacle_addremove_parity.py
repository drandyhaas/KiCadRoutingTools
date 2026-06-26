#!/usr/bin/env python3
"""Obstacle-map invariants for #198 (add/remove ref-count symmetry) and #199
(build_base vs cache construction parity).

Two properties every obstacle stamp must hold, or routing drifts silently:

  A. ADD then REMOVE leaves the map EMPTY -- the remove path must compute the
     same cell multiset the add path did, or the Rust ref-counts leak (stale
     blocks -> phantom congestion) or underflow (drop a real block -> graze).
     Tested for axis-aligned, diagonal, and OFF-GRID-endpoint segments + vias.

  B. The SAME copper produces the SAME blocked cells whether stamped by the
     build_base primitive (`_add_segment_obstacle`) or the obstacle-cache
     primitive (`_collect_segment_obstacles`). The two coexist in one working
     map (`base + cache`), so they must agree.

Run:  python3 tests/test_obstacle_addremove_parity.py [-v]
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np
from kicad_parser import Segment, Via
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import (GridObstacleMap, build_layer_map,
                          add_segments_list_as_obstacles, remove_segments_list_from_obstacles,
                          add_vias_list_as_obstacles, remove_vias_list_from_obstacles,
                          _add_segment_obstacle, _add_via_obstacle)
from obstacle_cache import _collect_segment_obstacles, _collect_via_obstacles

LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]


def _config():
    return GridRouteConfig(layers=LAYERS, grid_step=0.05, track_width=0.1,
                           clearance=0.1, via_size=0.3, via_drill=0.2)


# Representative segments: axis-aligned, 45-degree diagonal, and one with
# OFF-GRID float endpoints (not multiples of grid_step) -- the case where the
# bresenham/square approximations historically drifted (#70/#173).
SEGS = {
    "axis":     Segment(start_x=10.00, start_y=10.00, end_x=12.00, end_y=10.00, width=0.1, layer="In2.Cu", net_id=1),
    "diagonal": Segment(start_x=10.00, start_y=10.00, end_x=11.40, end_y=11.40, width=0.1, layer="In2.Cu", net_id=1),
    "offgrid":  Segment(start_x=10.137, start_y=10.073, end_x=11.412, end_y=11.388, width=0.1, layer="In2.Cu", net_id=1),
}
VIAS = {
    "ongrid":  Via(x=20.00, y=20.00, size=0.3, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=2),
    "offgrid": Via(x=20.123, y=20.077, size=0.3, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=2),
}


def _blocked_sets(m, coord, x_lo, y_lo, x_hi, y_hi):
    """Set of blocked (gx,gy,layer) cells and via-blocked (gx,gy) over a bbox."""
    g_lo = coord.to_grid(x_lo - 1.0, y_lo - 1.0)
    g_hi = coord.to_grid(x_hi + 1.0, y_hi + 1.0)
    cells, vias = set(), set()
    for gx in range(g_lo[0], g_hi[0] + 1):
        for gy in range(g_lo[1], g_hi[1] + 1):
            if m.is_via_blocked(gx, gy):
                vias.add((gx, gy))
            for li in range(len(LAYERS)):
                if m.is_blocked(gx, gy, li):
                    cells.add((gx, gy, li))
    return cells, vias


def test_add_remove_empty(verbose):
    """A: add then remove leaves the map byte-empty (no leak, no underflow)."""
    config = _config()
    fails = []
    for name, seg in SEGS.items():
        m = GridObstacleMap(len(LAYERS))
        add_segments_list_as_obstacles(m, [seg], config)
        nblk = m.get_stats()[0]
        if nblk == 0:
            fails.append(f"seg/{name}: add blocked nothing")
        remove_segments_list_from_obstacles(m, [seg], config)
        cells, vias = m.get_stats()[0], m.get_stats()[1]
        if cells or vias:
            fails.append(f"seg/{name}: NOT empty after remove (cells={cells}, vias={vias})")
        elif verbose:
            print(f"  seg/{name}: add {nblk} cells -> remove -> empty  OK")
    for name, via in VIAS.items():
        m = GridObstacleMap(len(LAYERS))
        add_vias_list_as_obstacles(m, [via], config)
        nblk = m.get_stats()[0] + m.get_stats()[1]
        if nblk == 0:
            fails.append(f"via/{name}: add blocked nothing")
        remove_vias_list_from_obstacles(m, [via], config)
        cells, vias_ct = m.get_stats()[0], m.get_stats()[1]
        if cells or vias_ct:
            fails.append(f"via/{name}: NOT empty after remove (cells={cells}, vias={vias_ct})")
        elif verbose:
            print(f"  via/{name}: add -> remove -> empty  OK")
    return fails


def test_base_cache_parity(verbose):
    """B: build_base primitive and cache primitive stamp the same cells."""
    config = _config()
    coord = GridCoord(config.grid_step)
    lm = build_layer_map(LAYERS)
    fails = []
    for name, seg in SEGS.items():
        li = lm[seg.layer]
        ltw = config.get_track_width(seg.layer)
        sw = seg.width if seg.width > 0 else ltw
        expansion_mm = ltw / 2 + sw / 2 + config.clearance
        via_block_mm = config.via_size / 2 + sw / 2 + config.clearance
        expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
        via_block_grid = max(1, coord.to_grid_dist_safe(via_block_mm))

        # build_base primitive
        mb = GridObstacleMap(len(LAYERS))
        _add_segment_obstacle(mb, seg, coord, li, expansion_mm, via_block_mm)

        # cache primitive: collect then batch into a fresh map
        mc = GridObstacleMap(len(LAYERS))
        bc, bv = [], []
        _collect_segment_obstacles(seg, coord, li, expansion_mm, bc, bv, via_block_mm)
        if bc:
            mc.add_blocked_cells_batch(np.ascontiguousarray(np.vstack(bc).astype(np.int32)))
        if bv:
            vv = np.vstack(bv).astype(np.int32)
            mc.add_blocked_vias_batch(np.ascontiguousarray(vv))

        x_lo, x_hi = min(seg.start_x, seg.end_x), max(seg.start_x, seg.end_x)
        y_lo, y_hi = min(seg.start_y, seg.end_y), max(seg.start_y, seg.end_y)
        cb, vb = _blocked_sets(mb, coord, x_lo, y_lo, x_hi, y_hi)
        cc, vc = _blocked_sets(mc, coord, x_lo, y_lo, x_hi, y_hi)
        if cb != cc:
            fails.append(f"seg/{name}: track cells differ (base {len(cb)} vs cache {len(cc)}, "
                         f"sym-diff {len(cb ^ cc)})")
        if vb != vc:
            fails.append(f"seg/{name}: via cells differ (base {len(vb)} vs cache {len(vc)}, "
                         f"sym-diff {len(vb ^ vc)})")
        if not (cb != cc or vb != vc) and verbose:
            print(f"  seg/{name}: base==cache ({len(cb)} track, {len(vb)} via cells)  OK")

    # via-as-obstacle parity. Uniform track width here, so build_base's MAX-width
    # single value equals the cache's per-layer list. (For per-LAYER/impedance
    # widths build_base uses max_track_width -- a conservative over-cover, not a
    # graze; documented on #199, intentionally not asserted bit-identical.)
    max_tw = config.get_max_track_width()
    inv = 1.0 / config.grid_step
    for name, via in VIAS.items():
        vs = via.size
        via_track_grid = max(1, coord.to_grid_dist_safe(vs / 2 + max_tw / 2 + config.clearance))
        via_via_grid = max(1.0, (vs / 2 + config.via_size / 2 + config.clearance) * inv)
        layer_widths = [config.get_net_track_width(via.net_id, l) for l in LAYERS]
        via_track_list = [max(1, coord.to_grid_dist_safe(vs / 2 + lw / 2 + config.clearance))
                          for lw in layer_widths]

        mb = GridObstacleMap(len(LAYERS))
        _add_via_obstacle(mb, via, coord, len(LAYERS), via_track_grid, via_via_grid, diagonal_margin=0.25)

        mc = GridObstacleMap(len(LAYERS))
        bc, bv = [], []
        _collect_via_obstacles(via, coord, len(LAYERS), via_track_list, via_via_grid, 0.25, bc, bv)
        if bc:
            mc.add_blocked_cells_batch(np.ascontiguousarray(np.vstack(bc).astype(np.int32)))
        if bv:
            mc.add_blocked_vias_batch(np.ascontiguousarray(np.vstack(bv).astype(np.int32)))

        cb, vb = _blocked_sets(mb, coord, via.x, via.y, via.x, via.y)
        cc, vc = _blocked_sets(mc, coord, via.x, via.y, via.x, via.y)
        if cb != cc:
            fails.append(f"via-obstacle/{name}: track cells differ (base {len(cb)} vs cache {len(cc)})")
        if vb != vc:
            fails.append(f"via-obstacle/{name}: via cells differ (base {len(vb)} vs cache {len(vc)})")
        if cb == cc and vb == vc and verbose:
            print(f"  via-obstacle/{name}: base==cache ({len(cb)} track, {len(vb)} via cells)  OK")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    fails = []
    print("A: add/remove ref-count symmetry (#198)")
    fails += test_add_remove_empty(args.verbose)
    print("B: build_base vs cache parity (#199)")
    fails += test_base_cache_parity(args.verbose)

    if fails:
        print("\nFAIL:\n  " + "\n  ".join(fails))
        return 1
    print("\nPASS: add/remove leaves the map empty, and build_base == cache for "
          "axis / diagonal / off-grid segments + vias")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
