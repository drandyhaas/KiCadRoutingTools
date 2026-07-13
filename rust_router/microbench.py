#!/usr/bin/env python3
"""Microbenchmark for the Rust grid router hot path (route_with_frontier).

Recreated from the rust-router review (0.18.0 batch): a serpentine-wall maze
on a GridObstacleMap forces a long snaking A* search that is dominated by the
per-expansion costs this batch optimizes (node-map ops, closed-set lookups,
is_blocked). Only the route_with_frontier call is timed; obstacle-map
construction is outside the timer. The reported number is the MIN over
--repeats runs (default 9), which is the standard way to strip scheduler /
allocator noise from a deterministic workload.

The maze: vertical walls every WALL_PITCH columns spanning the full height
except for a one-slot gap that alternates between top and bottom, so the
route from the left edge to the right edge must serpentine through every
wall. Both layers carry the same walls, so via escapes don't shortcut the
maze (they just widen the explored state space, like a real board).

Usage:
    python3 rust_router/microbench.py            # default 9 repeats
    python3 rust_router/microbench.py --repeats 5 --size 500

The printed `iterations` must be IDENTICAL across binaries for the
byte-identical 0.18.0 items (S1, S2, C1); only the time may change.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from grid_router import GridObstacleMap, GridRouter, __version__


def build_serpentine_maze(size: int, wall_pitch: int, gap: int, num_layers: int):
    """Serpentine-wall maze: walls every wall_pitch columns, alternating gaps."""
    cells = []
    for i, wx in enumerate(range(wall_pitch, size - 1, wall_pitch)):
        # Alternate the gap between the top and the bottom of the wall
        if i % 2 == 0:
            wall_rows = range(0, size - gap)      # gap at the bottom
        else:
            wall_rows = range(gap, size)          # gap at the top
        for wy in wall_rows:
            for layer in range(num_layers):
                cells.append((wx, wy, layer))
    # Outer border so the search cannot escape the maze
    for c in range(-1, size + 1):
        for layer in range(num_layers):
            cells.extend([(c, -1, layer), (c, size, layer),
                          (-1, c, layer), (size, c, layer)])
    obstacles = GridObstacleMap(num_layers)
    obstacles.add_blocked_cells_batch(np.array(cells, dtype=np.int32))
    return obstacles


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--repeats', type=int, default=9)
    ap.add_argument('--size', type=int, default=400, help='maze side in grid cells')
    ap.add_argument('--wall-pitch', type=int, default=20)
    ap.add_argument('--gap', type=int, default=8)
    ap.add_argument('--layers', type=int, default=2)
    args = ap.parse_args()

    obstacles = build_serpentine_maze(args.size, args.wall_pitch, args.gap, args.layers)
    router = GridRouter(via_cost=5000, h_weight=1.0, turn_cost=1000)

    sources = [(1, 1, 0)]
    targets = [(args.size - 2, args.size - 2, 0)]
    max_iterations = 50_000_000

    times = []
    iters = None
    path_len = None
    for _ in range(args.repeats):
        t0 = time.perf_counter()
        path, iterations, blocked = router.route_with_frontier(
            obstacles, sources, targets, max_iterations)
        times.append(time.perf_counter() - t0)
        if iters is None:
            iters = iterations
            path_len = len(path) if path else 0
        else:
            assert iterations == iters, "non-deterministic iteration count!"

    print(f"grid_router {__version__}  maze={args.size}x{args.size} "
          f"pitch={args.wall_pitch} gap={args.gap} layers={args.layers}")
    print(f"iterations={iters} path_len={path_len}")
    print(f"min={min(times)*1000:.1f} ms  (over {args.repeats} repeats; "
          f"all: {', '.join(f'{t*1000:.1f}' for t in times)})")


if __name__ == '__main__':
    main()
