"""Single-tap relocation (#424, planes-first chains).

When a route fails and the decisive blocker is PRE-EXISTING plane-net
copper, the rip ladder is helpless: pre-existing nets are not rippable, and
whole-net --rip-existing-nets is catastrophically mis-grained for "this one
tap via is in my way" (ripping GND rips - and then reroutes as tracks - the
entire net; see the reconciliation-exclusion fix).

This module implements the right-grained move: remove ONE pre-existing
plane-net via (a tap or dogbone placed by an earlier planes-first / fanout
step) plus its short attached stub, retry the failing net, and rely on the
plane-repair step later in the chain to re-tap the affected pad from the
pour (the net-coverage invariant: copper removed by one stage is re-claimed
by a later stage that owns that net's connectivity -
route_disconnected_planes detects and repairs disconnected plane components
by design).

Obstacle-map custody (the part that must be exact): per-net cached cell
data may be deduplicated and the working map is ref-counted (#309), so
removing a hand-computed "this via's cells" set can underflow cells shared
with the net's other copper. Relocation therefore NEVER does per-item map
surgery. It uses whole-net cache recompute with existing primitives:

    remove_net_obstacles_from_cache(map, cache[net])   # net's old data out
    drop the via + stub from pcb_data
    cache[net] = precompute_net_obstacles(...)          # rebuilt from what remains
    add_net_obstacles_from_cache(map, cache[net])       # net's new data in

which is exact regardless of overlap or dedup. Prerequisite: the plane net
must have a cache entry (be base-map-excluded) - the KICAD_TAP_RELOCATION
registration in batch_route arranges this for every net with a filled zone,
WITHOUT adding them to routed_results (the whole-net rip ladder can never
touch them).

Eligibility is deliberately narrow: only nets with a filled zone (repairable
from the pour), only vias within a small radius of the failing endpoint (or
validator-blamed), at most KICAD_TAP_RELOCATION_MAX (default 2) per failing
net. Env-gated overall by KICAD_TAP_RELOCATION=1.
"""
from __future__ import annotations

import math
import os
from typing import List, Optional, Set, Tuple

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig


def tap_relocation_enabled() -> bool:
    return os.environ.get('KICAD_TAP_RELOCATION', '') in ('1', 'true', 'on')


def tap_relocation_max() -> int:
    try:
        return int(os.environ.get('KICAD_TAP_RELOCATION_MAX', '2') or 2)
    except ValueError:
        return 2


def plane_zone_net_ids(pcb_data: PCBData) -> Set[int]:
    """Nets with a filled zone: removing one of their vias is repairable
    from the pour by the plane-repair step."""
    return {z.net_id for z in pcb_data.zones if z.net_id > 0}


def _attached_stub(pcb_data: PCBData, via: Via, max_len_mm: float = 2.0
                   ) -> List[Segment]:
    """Short same-net segments touching the via (a dogbone arm / tap stub);
    long trunks are left alone - relocation targets TAPS, not routes."""
    out = []
    for s in pcb_data.segments:
        if s.net_id != via.net_id:
            continue
        for (px, py) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if abs(px - via.x) < 1e-3 and abs(py - via.y) < 1e-3:
                if math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) <= max_len_mm:
                    out.append(s)
                break
    return out


def find_relocation_candidate(pcb_data: PCBData, config: GridRouteConfig,
                              net_obstacles_cache: dict,
                              endpoint_xy: Optional[Tuple[float, float]],
                              blamed_net_ids: Optional[Set[int]] = None,
                              radius_mm: float = 3.0,
                              exclude_via_ids: Optional[Set[int]] = None
                              ) -> Optional[Via]:
    """The nearest relocatable plane-net via to the failing endpoint.
    Only nets that are BOTH zone-backed and cache-registered qualify (the
    cache entry is what makes exact map surgery possible)."""
    if endpoint_xy is None:
        return None
    plane_ids = plane_zone_net_ids(pcb_data) & set(net_obstacles_cache.keys())
    if blamed_net_ids is not None:
        preferred = plane_ids & blamed_net_ids
        if preferred:
            plane_ids = preferred
    if not plane_ids:
        return None
    ex, ey = endpoint_xy
    best, best_d = None, radius_mm
    excl = exclude_via_ids or set()
    for v in pcb_data.vias:
        if v.net_id not in plane_ids or id(v) in excl:
            continue
        d = math.hypot(v.x - ex, v.y - ey)
        if d < best_d:
            best, best_d = v, d
    return best


def relocate_tap(pcb_data: PCBData, config: GridRouteConfig,
                 working_obstacles, net_obstacles_cache: dict,
                 via: Via) -> Optional[dict]:
    """Remove the via + attached short stub via whole-net cache recompute.
    Returns an undo token for restore_tap(), or None if the net has no
    cache entry (not registered relocatable)."""
    import routing_defaults as defaults
    from obstacle_cache import (add_net_obstacles_from_cache,
                                precompute_net_obstacles,
                                remove_net_obstacles_from_cache)
    nid = via.net_id
    old_data = net_obstacles_cache.get(nid)
    if old_data is None or working_obstacles is None:
        return None
    stubs = _attached_stub(pcb_data, via)

    remove_net_obstacles_from_cache(working_obstacles, old_data)
    pcb_data.vias = [v for v in pcb_data.vias if id(v) != id(via)]
    stub_ids = {id(s) for s in stubs}
    pcb_data.segments = [s for s in pcb_data.segments if id(s) not in stub_ids]

    new_data = precompute_net_obstacles(
        pcb_data, nid, config, extra_clearance=0.0,
        diagonal_margin=defaults.DIAGONAL_MARGIN)
    net_obstacles_cache[nid] = new_data
    add_net_obstacles_from_cache(working_obstacles, new_data)

    # The pad this tap served (nearest same-net pad): the re-tap
    # obligation travels in the token -- commit is CONDITIONAL on
    # re-tapping it (c1 defect: relying on a later repair step detached
    # pads permanently in the finisher steps and shipped DRC grazes).
    pad = None
    best_d = 2.0
    for p in pcb_data.pads_by_net.get(nid, []):
        d = math.hypot(p.global_x - via.x, p.global_y - via.y)
        if d < best_d:
            pad, best_d = p, d
    name = pcb_data.nets[nid].name if nid in pcb_data.nets else str(nid)
    print(f"  TAP RELOCATION: removed {name} via at ({via.x:.2f}, {via.y:.2f})"
          f" + {len(stubs)} stub seg(s); re-tap required before commit")
    return {'via': via, 'stubs': stubs, 'old_data': old_data,
            'new_data': new_data, 'net_id': nid, 'pad': pad}


def retap_pad(pcb_data: PCBData, config: GridRouteConfig,
              working_obstacles, net_obstacles_cache: dict,
              token: dict, coord=None, layer_names=None) -> bool:
    """Place a replacement tap via in/at the detached pad (the ae2069
    plane-tap machinery via _place_shrunk_via_in_pad). Success mutates
    pcb_data + swaps the net's cache entry (the balanced pattern) and
    returns True; a token with no pad (stitching via) is vacuously True."""
    pad = token.get('pad')
    if pad is None:
        return True
    from routing_config import GridCoord
    from single_ended_routing import _place_shrunk_via_in_pad
    from obstacle_cache import (add_net_obstacles_from_cache,
                                precompute_net_obstacles,
                                remove_net_obstacles_from_cache)
    import routing_defaults as defaults
    nid = token['net_id']
    coord = coord or GridCoord(config.grid_step)
    layer_names = layer_names or config.layers
    placed = _place_shrunk_via_in_pad(pad, working_obstacles, config,
                                      pcb_data, nid, coord, layer_names)
    if placed is None:
        return False
    new_via = placed[0]
    entry = net_obstacles_cache.get(nid)
    if working_obstacles is not None and entry is not None:
        remove_net_obstacles_from_cache(working_obstacles, entry)
        pcb_data.vias.append(new_via)
        net_obstacles_cache[nid] = precompute_net_obstacles(
            pcb_data, nid, config, extra_clearance=0.0,
            diagonal_margin=defaults.DIAGONAL_MARGIN)
        add_net_obstacles_from_cache(working_obstacles,
                                     net_obstacles_cache[nid])
    else:
        pcb_data.vias.append(new_via)
    print(f"  TAP RELOCATION: re-tapped {pad.component_ref}.{pad.pad_number} "
          f"with a fresh via at ({new_via.x:.2f}, {new_via.y:.2f})")
    return True


def restore_tap(pcb_data: PCBData, working_obstacles,
                net_obstacles_cache: dict, token: dict) -> None:
    """Inverse of relocate_tap (retry failed; the removal bought nothing):
    swap the recomputed data back out, reinsert the copper, restore the
    original cache entry."""
    from obstacle_cache import (add_net_obstacles_from_cache,
                                remove_net_obstacles_from_cache)
    nid = token['net_id']
    remove_net_obstacles_from_cache(working_obstacles, token['new_data'])
    pcb_data.vias.append(token['via'])
    pcb_data.segments.extend(token['stubs'])
    net_obstacles_cache[nid] = token['old_data']
    add_net_obstacles_from_cache(working_obstacles, token['old_data'])
    v = token['via']
    print(f"  TAP RELOCATION: retry failed, restored via at ({v.x:.2f}, {v.y:.2f})")
