"""The price of leaving a net's lane.

A net may be given a LANE -- the way it is to keep to: a bus member's corridor
(or the path of the member routed before it), a plan's way for the net
(global_plan), a pair leg's partner. The router charges for each cell entered
OFF the lane (GridRouter lane_free / lane_ramp / lane_prices, set_lane_path):
free within `free` of the lane on the lane's own layer there, rising over `ramp`
to the full price, which every cell farther off costs on every layer.

A price for leaving, never a reward for keeping to it. The search is weighted
(f = g + w h, h at the full step price): a discount along a lane lowers g there
and leaves w h as it was, so a lane that bends from the straight line loses to
the straight line at every step, whatever the discount. A price off the lane
raises g on the straight line instead, and the lane is taken.

The full price is RELATIVE to a step on the lane: a step off it costs
1 + lane_cost times a step on the lane's dearest layer, whichever layer it is
taken on -- a lane on a layer that costs more than another is not undersold by
the cheaper layer's steps beside it.
"""
import env_knobs
import routing_defaults as defaults


def lane_cost(config=None):
    """what a step off a net's lane costs, in steps on the lane (KICAD_GLOBAL_PLAN_LANE_COST, else the default)"""
    return env_knobs.GLOBAL_PLAN.get('lane_cost', 0) or defaults.LANE_COST


def lane_terms(config, net_id, lane_path, cost=None, free_mm=None, ramp_mm=None):
    """(lane_free, lane_ramp, lane_prices) for GridRouter, for the net `net_id` routed along `lane_path`
    [(gx, gy, layer index)] -- grid units, and the full price a cell off the lane costs per layer (cost units, 0 on a
    layer the run does not route on). (0, 0, None) with no lane.

    The free half-width is the widest of the default, the plan's (KICAD_GLOBAL_PLAN_LANE_FREE) and the net's own
    (config._lane_free_mm, where a lane is shared: a bus's corridor is its members' together, half the bus wide)."""
    if not lane_path:
        return 0, 0, None
    k = env_knobs.GLOBAL_PLAN
    price = lane_cost(config) if cost is None else cost
    g = config.grid_step
    if free_mm is None:
        free_mm = max(k.get('lane_free', defaults.LANE_FREE),
                      (getattr(config, '_lane_free_mm', None) or {}).get(net_id, 0.0))
    if ramp_mm is None:
        ramp_mm = k.get('lane_ramp', defaults.LANE_RAMP)
    costs = [c / 1000.0 for c in config.get_layer_costs()]          # (1.0 a plain layer; negative: not routed on)
    on = {p[2] for p in lane_path}
    c_way = max([costs[li] for li in on if li < len(costs) and costs[li] > 0] or [1.0])
    prices = [int(round(1000 * max((1.0 + price) * c_way - c, 0.0))) if c > 0 else 0 for c in costs]
    return int(round(free_mm / g)), max(1, int(round(ramp_mm / g))), prices
