#!/usr/bin/env python3
"""GridRouter FORBIDDEN LAYER (`--layer-costs -1` => a negative entry in the i32
layer_costs vector). The router must never place a track on, nor end a via on, a
forbidden layer; the layer still acts as an obstacle and through-vias may SPAN it.

  - BAN: with the source layer made costly, the route would normally via onto the
    cheaper layer; forbidding that layer keeps the route on the costly source layer
    (control proves the bias WOULD have used it). A high cost alone is only a soft
    avoid -- the ban must be hard.
  - SPAN: a source on layer 0 / target on layer 2 routes via a single 0->2
    through-via that SPANS the forbidden middle layer 1 (no node ever on layer 1) --
    identical to the non-forbidden case, because a via is one direct edge.
  - CLEAN FAIL: a target ON a forbidden layer is unreachable (no via may land there)
    and the router returns no path rather than silently ending on it.

    python3 tests/test_forbidden_layer_grid.py        # needs the built grid_router
"""
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "rust_router"))
import grid_router  # noqa: E402

FORBIDDEN = -1


def _route(layer_costs, n_layers, src, tgt):
    obs = grid_router.GridObstacleMap(n_layers)  # fully open
    r = grid_router.GridRouter(via_cost=500, h_weight=1.0, turn_cost=10, layer_costs=layer_costs)
    path, _iters, _stats = r.route_multi(obs, [src], [tgt], 2_000_000, False, 0, None, None, 0, 0)
    return path


def run():
    fails = []

    # CONTROL: layer 0 costly, layer 1 cheap+ALLOWED -> the route SHOULD via to layer 1.
    ctrl = _route([50000, 1000], 2, (2, 2, 0), (60, 2, 0))
    ctrl_layers = Counter(p[2] for p in ctrl) if ctrl else {}
    if not ctrl or ctrl_layers.get(1, 0) <= 0:
        fails.append(f"control: costly layer 0 should via to the cheaper layer 1, got {dict(ctrl_layers)}")

    # BAN: same bias, but layer 1 FORBIDDEN -> the route must stay on layer 0 (hard ban,
    # not just discouraged). Zero nodes on layer 1.
    banned = _route([50000, FORBIDDEN], 2, (2, 2, 0), (60, 2, 0))
    if not banned:
        fails.append("ban: route failed (should complete on the costly-but-allowed layer 0)")
    else:
        layers = Counter(p[2] for p in banned)
        if 1 in layers:
            fails.append(f"ban: forbidden layer 1 carries {layers[1]} node(s) (must be 0): {dict(layers)}")

    # SPAN: source layer 0, target layer 2, middle layer 1 FORBIDDEN -> one 0->2 through-via
    # spans layer 1. Must match the non-forbidden route exactly (the via never touches layer 1).
    span = _route([1000, FORBIDDEN, 1000], 3, (2, 2, 0), (60, 2, 2))
    span_ref = _route([1000, 1000, 1000], 3, (2, 2, 0), (60, 2, 2))
    if not span:
        fails.append("span: route 0->2 failed across a forbidden middle layer")
    else:
        layers = set(p[2] for p in span)
        if 1 in layers:
            fails.append(f"span: forbidden middle layer 1 was used: {sorted(layers)}")
        if 2 not in layers:
            fails.append(f"span: route never reached the layer-2 target: {sorted(layers)}")
        if span_ref and Counter(p[2] for p in span) != Counter(p[2] for p in span_ref):
            fails.append("span: forbidding the (unused) middle layer changed the route")

    # CLEAN FAIL: target ON a forbidden layer -> unreachable (no via lands on it) -> no path.
    stuck = _route([1000, FORBIDDEN], 2, (2, 2, 0), (60, 2, 1))
    if stuck:
        fails.append(f"clean-fail: a route ended on the forbidden target layer: {Counter(p[2] for p in stuck)}")

    if fails:
        print("FAIL:")
        for f in fails:
            print("  -", f)
        return False
    print("PASS: hard ban (zero copper on the forbidden layer), through-via spans it, "
          "forbidden target is unreachable")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
