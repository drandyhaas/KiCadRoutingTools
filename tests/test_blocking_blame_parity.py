#!/usr/bin/env python3
"""Issue #203: the rip-up blame map must agree with the real obstacle map.

`blocking_analysis.compute_net_obstacle_cells` re-derives, per routed net, the
cells that net blocks -- and that set is intersected with the stuck frontier to
pick which net to rip. If it uses a different obstacle shape than the obstacle
map actually uses (it used a SQUARE track box + bresenham line, while the map
uses the exact capsule), a frontier cell genuinely blocked by net A's capsule
can also fall inside net B's square-corner over-reach, and the WRONG net gets
ripped. This test pins the blame map's per-net TRACK cells to the cells the
obstacle map actually blocks for that net (`add_net_stubs_as_obstacles`), for
axis-aligned, diagonal, and off-grid-endpoint segments.

Run:  python3 tests/test_blocking_blame_parity.py [-v]
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from types import SimpleNamespace
from kicad_parser import Segment, Via
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import GridObstacleMap, add_net_stubs_as_obstacles
from blocking_analysis import compute_net_obstacle_cells, _unpack_xy

LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]

# Same representative segments as the obstacle add/remove parity test: the
# diagonal and off-grid cases are exactly where the old square/bresenham stamp
# drifted from the capsule.
SEGS = {
    "axis":     Segment(start_x=10.00, start_y=10.00, end_x=12.00, end_y=10.00, width=0.1, layer="In2.Cu", net_id=1),
    "diagonal": Segment(start_x=10.00, start_y=10.00, end_x=11.40, end_y=11.40, width=0.1, layer="In2.Cu", net_id=1),
    "offgrid":  Segment(start_x=10.137, start_y=10.073, end_x=11.412, end_y=11.388, width=0.1, layer="In2.Cu", net_id=1),
}


def _config():
    return GridRouteConfig(layers=LAYERS, grid_step=0.05, track_width=0.1,
                           clearance=0.1, via_size=0.3, via_drill=0.2)


def _pcb_with(seg):
    # add_net_stubs_as_obstacles / compute_net_obstacle_cells only read
    # .segments and .vias, so a light stand-in is enough.
    return SimpleNamespace(segments=[seg], vias=[])


def _map_track_cells(seg, config):
    """(gx,gy,layer) cells the real obstacle map blocks for this net's stub."""
    m = GridObstacleMap(len(LAYERS))
    add_net_stubs_as_obstacles(m, _pcb_with(seg), seg.net_id, config)
    coord = GridCoord(config.grid_step)
    g_lo = coord.to_grid(min(seg.start_x, seg.end_x) - 1.0, min(seg.start_y, seg.end_y) - 1.0)
    g_hi = coord.to_grid(max(seg.start_x, seg.end_x) + 1.0, max(seg.start_y, seg.end_y) + 1.0)
    cells = set()
    for gx in range(g_lo[0], g_hi[0] + 1):
        for gy in range(g_lo[1], g_hi[1] + 1):
            for li in range(len(LAYERS)):
                if m.is_blocked(gx, gy, li):
                    cells.add((gx, gy, li))
    return cells


def _blame_track_cells(seg, config):
    """(gx,gy,layer) cells the blame map attributes to this net's stub."""
    track_keys, _via_keys = compute_net_obstacle_cells(
        _pcb_with(seg), seg.net_id, None, config)
    gx, gy = _unpack_xy(track_keys)
    layer = (track_keys & 0xff)
    return set(zip(gx.tolist(), gy.tolist(), layer.tolist()))


def test_blame_matches_map(verbose, config):
    fails = []
    for name, seg in SEGS.items():
        mapped = _map_track_cells(seg, config)
        blamed = _blame_track_cells(seg, config)
        if not mapped:
            fails.append(f"seg/{name}: obstacle map blocked nothing")
        if mapped != blamed:
            fails.append(f"seg/{name}: blame != map (map {len(mapped)} vs blame {len(blamed)}, "
                         f"sym-diff {len(mapped ^ blamed)})")
        elif verbose:
            print(f"  seg/{name}: blame == map ({len(mapped)} track cells)  OK")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    fails = []
    for label, config in [("uniform width", _config())]:
        print(f"=== {label} ===")
        fails += test_blame_matches_map(args.verbose, config)

    if fails:
        print("\nFAIL:\n  " + "\n  ".join(fails))
        return 1
    print("\nPASS: blame map track cells == obstacle map for axis / diagonal / off-grid segments")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
