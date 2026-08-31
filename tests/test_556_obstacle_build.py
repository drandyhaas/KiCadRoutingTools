#!/usr/bin/env python3
"""#556: base obstacle build — progress feedback + byte-equivalent holes pass.

1. The rule-area keep-out HOLES pass was the last dense (cells x edges)
   kernel in the build: per hole it evaluated _points_inside_polygon +
   _points_edge_distance over the WHOLE outer raster. It now rasterizes the
   hole on its own (scanline + banded) raster and maps back into the outer
   meshgrid. This test pins byte-equivalence against an inline copy of the
   dense reference on ring keep-outs (hole centered, hole overhanging the
   outer polygon, multiple holes), by probing the resulting obstacle maps.
2. build_base_obstacle_map(progress_callback=...) reports sub-phase
   progress (GUI liveness for #556's "stuck on Building base obstacle map");
   reporting must not change the map.

Run:  python3 tests/test_556_obstacle_build.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

import numpy as np

from routing_config import GridRouteConfig, GridCoord
from obstacle_map import (add_rule_area_keepout_obstacles, _rasterize_polygon,
                          _points_inside_polygon, _points_edge_distance,
                          _block_cells_on_layers, build_base_obstacle_map)
from routing_utils import GRID_TIE_EPS
from grid_router import GridObstacleMap


class _BoardInfo:
    def __init__(self, keepouts, bounds):
        self.keepouts = keepouts
        self.board_bounds = bounds
        self.copper_layers = ['F.Cu', 'B.Cu']
        self.board_outline = None
        self.board_outlines = None
        self.board_cutouts = []
        self.board_edge_contours = []


class _PCB:
    def __init__(self, keepouts, bounds):
        self.board_info = _BoardInfo(keepouts, bounds)
        self.segments = []
        self.vias = []
        self.zones = []
        self.footprints = {}
        self.nets = {}
        self.pads_by_net = {}


def _dense_reference(obstacles, pcb, config):
    """Inline copy of the pre-#556 holes pass (dense kernels over the outer
    raster) for the byte-equivalence comparison."""
    coord = GridCoord(config.grid_step)
    layer_list = config.layers
    track_clear = config.clearance + config.track_width / 2
    via_clear = config.clearance + config.via_size / 2
    clip = pcb.board_info.board_bounds
    for ko in pcb.board_info.keepouts:
        poly = ko.get('polygon') or []
        if len(poly) < 3:
            continue
        margin = max(track_clear, via_clear) + coord.grid_step
        gx_flat, gy_flat, inside, edge_dist = _rasterize_polygon(
            poly, coord, margin, clip_bounds=clip)
        if gx_flat is None:
            continue
        # Same boundary convention as production (#816): a cell centre sitting
        # EXACTLY at the clearance resolves OPEN, so the reference models the
        # same predicate. Without this the reference and the production path
        # disagree only on tie cells -- which is what the via legs below hit,
        # since clearance + via_size/2 lands on an exact grid multiple here.
        track_mask = inside | (edge_dist < track_clear - GRID_TIE_EPS)
        via_mask = inside | (edge_dist < via_clear - GRID_TIE_EPS)
        for hole in ko.get('holes') or []:
            hp = np.asarray(hole, dtype=np.float64)
            if hp.shape[0] < 3:
                continue
            px = gx_flat.astype(np.float64) * coord.grid_step
            py = gy_flat.astype(np.float64) * coord.grid_step
            hx1, hy1 = hp[:, 0], hp[:, 1]
            hx2, hy2 = np.roll(hp[:, 0], -1), np.roll(hp[:, 1], -1)
            h_inside = _points_inside_polygon(px, py, hx1, hy1, hx2, hy2)
            h_edge = _points_edge_distance(px, py, hx1, hy1, hx2, hy2)
            track_mask &= ~(h_inside & (h_edge >= track_clear - GRID_TIE_EPS))
            via_mask &= ~(h_inside & (h_edge >= via_clear - GRID_TIE_EPS))
        _block_cells_on_layers(obstacles, gx_flat, gy_flat, track_mask,
                               range(len(layer_list)))
        if via_mask.any():
            obstacles.add_blocked_vias_batch(
                np.column_stack([gx_flat[via_mask], gy_flat[via_mask]]))


def _probe(obstacles, coord, bounds, num_layers):
    """(cells, vias) blocked sets over the probe window."""
    gx0, gy0 = coord.to_grid(bounds[0] - 2, bounds[1] - 2)
    gx1, gy1 = coord.to_grid(bounds[2] + 2, bounds[3] + 2)
    cells, vias = set(), set()
    for gx in range(gx0, gx1 + 1):
        for gy in range(gy0, gy1 + 1):
            for li in range(num_layers):
                if obstacles.is_blocked(gx, gy, li):
                    cells.add((gx, gy, li))
            if obstacles.is_via_blocked(gx, gy):
                vias.add((gx, gy))
    return cells, vias


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    config = GridRouteConfig(grid_step=0.1, clearance=0.15, track_width=0.2,
                             via_size=0.5, via_drill=0.3,
                             layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(config.grid_step)
    bounds = (0.0, 0.0, 30.0, 30.0)
    outer = [(2.0, 2.0), (28.0, 2.0), (28.0, 28.0), (2.0, 28.0)]

    cases = [
        ("centered hole", [[(8.0, 8.0), (22.0, 8.0), (22.0, 22.0), (8.0, 22.0)]]),
        ("hole overhangs outer", [[(20.0, 20.0), (35.0, 20.0), (35.0, 35.0), (20.0, 35.0)]]),
        ("two holes, one tiny", [[(5.0, 5.0), (12.0, 5.0), (12.0, 12.0), (5.0, 12.0)],
                                 [(18.05, 18.05), (19.95, 18.05), (19.0, 19.95)]]),
    ]
    for name, holes in cases:
        keepouts = [{'polygon': outer, 'holes': holes,
                     'tracks_allowed': False, 'vias_allowed': False,
                     'layers': set()}]
        pcb = _PCB(keepouts, bounds)
        new_map = GridObstacleMap(2)
        add_rule_area_keepout_obstacles(new_map, pcb, config)
        ref_map = GridObstacleMap(2)
        _dense_reference(ref_map, pcb, config)
        nc, nv = _probe(new_map, coord, bounds, 2)
        rc, rv = _probe(ref_map, coord, bounds, 2)
        check(f"{name}: track cells identical ({len(nc)} cells)", nc == rc)
        check(f"{name}: via cells identical ({len(nv)} cells)", nv == rv)

    # --- progress feedback: callback fires, map unchanged by reporting ------
    pcb = _PCB([], bounds)
    pcb.board_info.board_outline = outer  # polygon edge pass -> band progress
    calls = []
    m1 = build_base_obstacle_map(pcb, config, [],
                                 progress_callback=lambda i, n, msg: calls.append(msg))
    m2 = build_base_obstacle_map(pcb, config, [])
    c1, v1 = _probe(m1, coord, bounds, 2)
    c2, v2 = _probe(m2, coord, bounds, 2)
    check("progress callback fired with phase labels",
          any(m.startswith("Obstacles:") for m in calls))
    check("reporting does not change the map", c1 == c2 and v1 == v2)

    # --- fill-model scanline conversion: bit-identity vs the old row loop ---
    # The plane_fill_model row loops (outline clip, keep-out rings, custom-pad
    # rings) were converted to the scanline kernel. Reproduce the OLD loop
    # verbatim here and compare on randomized multi-ring geometry, including
    # cell centres exactly on ring vertices' y (straddle tie cases).
    from obstacle_map import _scanline_inside_rows
    rng = np.random.RandomState(42)
    for trial in range(20):
        n_rings = rng.randint(1, 4)
        rings = []
        for _ in range(n_rings):
            n_pts = rng.randint(3, 12)
            cx_c, cy_c = rng.uniform(2, 28, 2)
            ang = np.sort(rng.uniform(0, 2 * np.pi, n_pts))
            rad = rng.uniform(0.5, 8.0, n_pts)
            rings.append([(cx_c + r * np.cos(a), cy_c + r * np.sin(a))
                          for a, r in zip(ang, rad)])
        sub_x = np.arange(0.05, 30.0, 0.1)
        ys = np.arange(0.05, 30.0, 0.1)
        # force some exact ties: a row through a vertex, a column at a vertex x
        ys[3] = rings[0][0][1]
        sub_x[5] = rings[0][1][0]
        old = np.zeros((ys.size, sub_x.size), dtype=bool)
        for j, yv in enumerate(ys):
            crossings = np.zeros(sub_x.size, dtype=np.int32)
            for r in rings:
                for k in range(len(r)):
                    rx1, ry1 = r[k]
                    rx2, ry2 = r[(k + 1) % len(r)]
                    if (ry1 > yv) == (ry2 > yv):
                        continue
                    xc = rx1 + (yv - ry1) * (rx2 - rx1) / (ry2 - ry1)
                    crossings += (sub_x < xc)
            old[j] = (crossings % 2) == 1
        arrs = [np.asarray(r, dtype=np.float64) for r in rings]
        new = _scanline_inside_rows(
            sub_x, ys,
            np.concatenate([a[:, 0] for a in arrs]),
            np.concatenate([a[:, 1] for a in arrs]),
            np.concatenate([np.roll(a[:, 0], -1) for a in arrs]),
            np.concatenate([np.roll(a[:, 1], -1) for a in arrs]))
        if not np.array_equal(old, new):
            fails.append(f"fill-model scanline mismatch on trial {trial}")
            print(f"  FAIL: fill-model scanline mismatch on trial {trial}")
            break
    else:
        print("  PASS: fill-model scanline bit-identical over 20 randomized "
              "multi-ring trials (incl. vertex ties)")

    if fails:
        print(f"\n{len(fails)} FAILURE(S)")
        return 1
    print("\nAll #556 obstacle-build tests passed")
    return 0


if __name__ == '__main__':
    sys.path.insert(0, os.path.join(ROOT_DIR, 'rust_router'))
    sys.exit(main())
