#!/usr/bin/env python3
"""PoseRouter FORBIDDEN LAYER (`--layer-costs -1` => a negative entry in the i32
layer_costs vector). The router must never place a track on, nor end a via on, a
forbidden layer; the layer still acts as an obstacle and through-vias may SPAN it.

  - BAN: with the cheaper escape layer forbidden, the route stays on the costly
    source layer rather than v-ing onto the forbidden one (control proves the bias
    WOULD have used it).
  - CLEAN FAIL: a source placed on a forbidden layer cannot lay track (G2) and
    returns no path rather than silently routing on it.

(The through-via SPAN over a forbidden middle layer is covered by
 test_forbidden_layer_grid.py -- the PoseRouter does not route a goal two layers
 away in an open map, so a 0->2 span is not exercisable here.)

    python3 tests/test_forbidden_layer_pose.py        # needs the built grid_router
"""
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rust_router"))
import grid_router  # noqa: E402

FORBIDDEN = -1


def _pr(layer_costs):
    return grid_router.PoseRouter(
        via_cost=500, h_weight=1.0, turn_cost=10, min_radius_grid=2.0,
        via_proximity_cost=10, diff_pair_spacing=0, max_turn_units=4,
        layer_costs=layer_costs,
    )


def _route(layer_costs, n_layers, start_layer, goal_layer):
    obs = grid_router.GridObstacleMap(n_layers)  # fully open
    pr = _pr(layer_costs)
    path, _i, _b, _g = pr.route_pose_with_frontier(
        obs, 2, 2, 0, start_layer, 60, 2, 0, goal_layer, 2_000_000, None)
    return path


def run():
    fails = []

    # CONTROL: layer 0 costly, layer 1 cheap+ALLOWED -> the route SHOULD via to layer 1.
    ctrl = _route([50000, 1000], 2, 0, 0)
    ctrl_layers = Counter(p[3] for p in ctrl) if ctrl else {}
    if not ctrl or ctrl_layers.get(1, 0) <= 0:
        fails.append(f"control: costly layer 0 should via to the cheaper layer 1, got {dict(ctrl_layers)}")

    # BAN: same bias, but layer 1 FORBIDDEN -> the route must stay on layer 0.
    banned = _route([50000, FORBIDDEN], 2, 0, 0)
    if not banned:
        fails.append("ban: route failed (should complete on the costly-but-allowed layer 0)")
    else:
        layers = Counter(p[3] for p in banned)
        if 1 in layers:
            fails.append(f"ban: forbidden layer 1 carries {layers[1]} node(s) (must be 0): {dict(layers)}")

    # CLEAN FAIL: source ON a forbidden layer (1) -> PoseRouter cannot lay track (G2) and
    # cannot gather the straight steps to via off -> returns no path (no copper on layer 1).
    stuck = _route([1000, FORBIDDEN, 1000], 3, 1, 0)
    if stuck and any(p[3] == 1 for p in stuck):
        fails.append(f"clean-fail: a forbidden source laid track on layer 1: {Counter(p[3] for p in stuck)}")

    if fails:
        print("FAIL:")
        for f in fails:
            print("  -", f)
        return False
    print("PASS: forbidden layer carries no track, and a forbidden source lays no copper")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
