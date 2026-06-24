#!/usr/bin/env python3
"""PoseRouter layer-cost bias (issue #193).

The coupled differential-pair router (PoseRouter) gained per-layer cost
multipliers, mirroring GridRouter. This locks the two invariants:
  - default (all 1.0x) leaves routing on the source layer (behavior unchanged),
  - penalizing the source layer pushes the route onto the cheaper layer via vias.

    python3 tests/test_pose_layer_costs.py
"""
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rust_router"))
import grid_router  # noqa: E402


def _route(layer_costs):
    obs = grid_router.GridObstacleMap(2)  # 2 layers, fully open
    pr = grid_router.PoseRouter(
        via_cost=500, h_weight=1.0, turn_cost=10, min_radius_grid=2.0,
        via_proximity_cost=10, diff_pair_spacing=0, max_turn_units=4,
        layer_costs=layer_costs,
    )
    # Straight east run on layer 0; with layer 0 cheap there is no reason to via.
    path, _iters, _blocked, _gnd = pr.route_pose_with_frontier(
        obs, 2, 2, 0, 0, 60, 2, 0, 0, 500000, None)
    return path


def run():
    fails = []

    default = _route([1000, 1000])
    if not default:
        fails.append("default route failed to find a path")
    else:
        layers = Counter(p[3] for p in default)
        if set(layers) != {0}:
            fails.append(f"default (all 1.0x) should stay on layer 0, got {dict(layers)}")

    penalized = _route([50000, 1000])
    if not penalized:
        fails.append("penalized route failed to find a path")
    else:
        layers = Counter(p[3] for p in penalized)
        # Expensive layer 0 -> the bulk of the run should move to the cheap layer 1.
        if layers.get(1, 0) <= layers.get(0, 0):
            fails.append(f"penalizing layer 0 should push the route to layer 1, got {dict(layers)}")

    if fails:
        print("FAIL:")
        for f in fails:
            print("  -", f)
        return False
    print("PASS: default stays on layer 0; penalizing layer 0 vias to layer 1")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
