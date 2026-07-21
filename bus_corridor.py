"""Bus corridor planning (#296 R9 / #424): plan ONE centerline per bus group.

The neighbor-attraction bus router has a structural flaw: the guide (middle)
member routes FIRST with no guidance, so the group's corridor is whatever the
guide's solo shortest path happened to be -- chosen with no knowledge that
N-1 siblings must fit beside it. When that corridor threads a one-track gap,
the siblings' attraction loses to real obstacle costs and the group scatters
(measured corpus-wide in #296: 2.57 vs 1.24 vias/member vs the human).

This module plans the corridor BEFORE any member routes, with capacity chosen
SOFTLY (no hard rejection): the group's representative net is probe-routed at
a ladder of inflated clearances -- extra clearance k*(track+clearance) is
"room for k sibling tracks on each side" -- and the winner minimizes

    score = probe_length_mm + (k_max - k) * room_mm * ROOM_WEIGHT

i.e. a corridor with room for the whole bus wins unless it costs too much
length. The winning probe's path (grid coords + layers) is then fed as the
attraction path to EVERY member, including the guide, so the whole group
packs into a corridor that was chosen with group knowledge. Probes commit
nothing; members still route individually with full rip-up, so everything
stays soft -- a member that genuinely cannot follow the corridor routes
around it exactly as before.

The inflation mechanism is the existing wide-power-net machinery: the probe
temporarily gives the representative a track width of track + 2k*(track+
clearance), so the router itself keeps k sibling-rooms of margin per side on
the SHARED obstacle map (track_margin) -- no per-rung map builds. The power
neck-down ladder is disabled for probes (power_tap_neckdown=False) so a wide
probe either routes at its full margin or fails the rung honestly. One probe
map is still built, excluding ALL bus members' stubs (the siblings will pack
into the corridor; their present stubs must not veto it).
"""

from dataclasses import replace
from typing import Dict, List, Optional, Tuple

from bus_detection import BusGroup, get_bus_routing_order
from obstacle_map import build_base_obstacle_map

# A missing sibling-room costs this many mm of detour before the planner
# prefers the narrower corridor: k rungs below k_max is penalized like
# ROOM_WEIGHT * k * (track + clearance) millimetres of extra length.
ROOM_WEIGHT = 40.0
# Rung cap: k_max = min(ceil((N-1)/2), MAX_RUNG) sibling-rooms per side.
MAX_RUNG = 3
# Probe via-cost multiplier (#296 R9 phase C): the human routes a bus as a
# river on ONE layer; a centerline probed at normal via cost hops layers
# freely and then guides every member incoherently. Pricing vias up in the
# PROBE ONLY makes the planned corridor layer-coherent; members still pay
# normal via costs. Override with KICAD_BUS_CORRIDOR_VIA_MULT (1 = off).
CORRIDOR_VIA_COST_MULT = 4.0


def _sample_dense(path: List[Tuple[int, int, int]], step: int = 1
                  ) -> List[Tuple[int, int, int]]:
    """Densify a simplified (gx, gy, layer) waypoint path so the Rust
    attraction lookup sees a point every `step` grid cells (same contract as
    single_ended_loop's neighbor-path sampling)."""
    if len(path) < 2:
        return list(path)
    out: List[Tuple[int, int, int]] = []
    for (x1, y1, l1), (x2, y2, l2) in zip(path, path[1:]):
        dx, dy = x2 - x1, y2 - y1
        n = max(abs(dx), abs(dy))
        if n == 0:
            out.append((x1, y1, l1))
            continue
        for i in range(0, n, step):
            t = i / n
            out.append((int(round(x1 + dx * t)), int(round(y1 + dy * t)), l1))
    out.append(path[-1])
    return out


def plan_bus_corridors(pcb_data, config, bus_groups: List[BusGroup],
                       verbose: bool = False
                       ) -> Dict[str, List[Tuple[int, int, int]]]:
    """Plan a corridor centerline per bus group. Returns {group.name: dense
    (gx, gy, layer) path}. Groups whose every probe rung fails are absent
    (callers fall back to neighbor attraction)."""
    # Deferred import: single_ended_routing must not import this module.
    from single_ended_routing import route_net_with_obstacles
    from net_queries import calculate_route_length

    corridors: Dict[str, List[Tuple[int, int, int]]] = {}
    if not bus_groups:
        return corridors

    all_member_ids = [nid for g in bus_groups for nid in g.net_ids]
    room_mm = config.track_width + config.clearance
    probe_map = build_base_obstacle_map(
        pcb_data, config, all_member_ids,
        net_clearances=config.net_clearances)

    print(f"\n=== Bus corridor planning: {len(bus_groups)} group(s), "
          f"room unit {room_mm:.3f}mm ===")
    for g in bus_groups:
        rep = get_bus_routing_order(g)[0]
        rep_name = pcb_data.nets[rep].name if rep in pcb_data.nets else str(rep)
        k_max = min(MAX_RUNG, max(1, (len(g.net_ids)) // 2))
        best = None      # (score, k, length, result)
        for k in range(k_max, -1, -1):
            # Probe via the wide-power mechanism: a track k sibling-rooms
            # wider per side keeps that much margin from every obstacle on
            # the shared map. Neck-down is disabled so the rung fails
            # honestly instead of silently degrading to rung 0.
            import os
            try:
                via_mult = float(os.environ.get('KICAD_BUS_CORRIDOR_VIA_MULT',
                                                str(CORRIDOR_VIA_COST_MULT)))
            except ValueError:
                via_mult = CORRIDOR_VIA_COST_MULT
            cfg_k = replace(
                config, power_tap_neckdown=False,
                via_cost=int(round(config.via_cost * via_mult)),
                power_net_widths={**config.power_net_widths,
                                  rep: config.track_width + 2 * k * room_mm})
            result = route_net_with_obstacles(
                pcb_data, rep, cfg_k, probe_map,
                reverse_direction=(g.clique_endpoint == "target"))
            if not result or result.get('failed') or not result.get('path'):
                if verbose:
                    print(f"  {g.name}: rung {k} (+{k * room_mm:.2f}mm "
                          f"clearance) no route")
                continue
            length = calculate_route_length(
                result['new_segments'], result.get('new_vias', []), pcb_data)
            score = length + (k_max - k) * room_mm * ROOM_WEIGHT
            if verbose:
                print(f"  {g.name}: rung {k} routes, length {length:.1f}mm, "
                      f"score {score:.1f}")
            if best is None or score < best[0]:
                best = (score, k, length, result)
        if best is None:
            print(f"  {g.name}: no corridor at any rung -- falling back to "
                  f"neighbor attraction")
            continue
        _score, k, length, result = best
        corridors[g.name] = _sample_dense(result['path'], step=1)
        lay = sorted({p[2] for p in result['path']})
        print(f"  {g.name}: corridor via {rep_name} at rung {k} "
              f"(room for {k} sibling(s)/side), {length:.1f}mm, "
              f"layers {lay}, {len(corridors[g.name])} pts")
    return corridors
