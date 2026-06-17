"""
Single-pad plane tap placement with fine-pitch parameter escalation.

Shared by route_planes.py (fine-pitch retry of failed taps, issue #104) and
route_disconnected_planes.py (per-pad repair pass, issue #99).

A "tap" is a stitching via near a pad plus an optional short trace from the
via to the pad on the pad's layer. The default route_planes parameters
(grid 0.1mm, clearance 0.25mm, track 0.3mm) cannot thread between the pads
of 0.4-0.65mm pitch QFN/LQFP parts; the proven fine parameters from the
castor_pollux stress run (grid 0.05mm, clearance 0.15mm, track <= 0.15mm)
recover those pads. The fine retry is scoped to a single pad: obstacle maps
are built on a small window around the pad so a fine grid stays cheap even
on large boards.
"""

import copy
import math
from dataclasses import dataclass, replace, field
from typing import List, Dict, Optional, Tuple, Set

from kicad_parser import PCBData, Pad, Via, Segment
from routing_config import GridRouteConfig, GridCoord
from plane_obstacle_builder import (
    build_via_obstacle_map,
    build_routing_obstacle_map,
    _smd_pad_reaches_layer,
)

# Fine-pitch detection thresholds (issue #104)
FINE_PITCH_NEIGHBOR_DIST = 0.65   # mm - same-component neighbor pad spacing
FINE_PITCH_MIN_PAD_DIM = 0.35     # mm - pad min dimension below this is fine-pitch
_DIST_TOL = 1e-3                  # mm - tolerance so exactly-0.65mm pitch qualifies

# Fine tap parameters (verified on castor_pollux: 27 failures -> 1)
FINE_TAP_GRID_STEP = 0.05         # mm
FINE_TAP_CLEARANCE = 0.15         # mm
FINE_TAP_TRACK_WIDTH = 0.15       # mm (capped by pad min dimension)
FINE_TAP_SEARCH_RADIUS = 3.0      # mm - via search radius for the fine retry

# Window half-size margin beyond the via search radius
_WINDOW_MARGIN = 3.0              # mm
_ITEM_MARGIN = 2.0                # mm - include items slightly outside the window


def pad_is_fine_pitch(pad: Pad, pcb_data: PCBData) -> bool:
    """Return True if a pad qualifies for the fine-parameter tap retry.

    A pad is fine-pitch when a neighboring pad of the same component is
    within FINE_PITCH_NEIGHBOR_DIST (center-to-center), or the pad's
    smaller dimension is below FINE_PITCH_MIN_PAD_DIM.
    """
    if min(pad.size_x, pad.size_y) < FINE_PITCH_MIN_PAD_DIM:
        return True
    footprint = pcb_data.footprints.get(pad.component_ref)
    if footprint is None:
        return False
    for other in footprint.pads:
        if other.pad_number == pad.pad_number:
            continue
        d = math.hypot(other.global_x - pad.global_x,
                       other.global_y - pad.global_y)
        if 0.0 < d <= FINE_PITCH_NEIGHBOR_DIST + _DIST_TOL:
            return True
    return False


def make_fine_tap_config(config: GridRouteConfig, pad: Pad) -> GridRouteConfig:
    """Build the scoped fine-parameter config for one pad's tap retry.

    grid 0.05 / clearance 0.15 / track = min(pad min dimension, 0.15);
    never coarser/wider than what the caller already uses. The via is NOT
    shrunk here -- the caller chooses the via (the standard working via, or the
    smaller fine-pitch escape via that plan-pcb-routing passes on 4+ layer
    fine-pitch boards), so the fab-capability gating lives in one place.
    """
    fine_track = min(min(pad.size_x, pad.size_y),
                     FINE_TAP_TRACK_WIDTH, config.track_width)
    return replace(
        config,
        grid_step=min(config.grid_step, FINE_TAP_GRID_STEP),
        clearance=min(config.clearance, FINE_TAP_CLEARANCE),
        track_width=fine_track,
    )


def _segment_overlaps_window(seg: Segment, min_x: float, min_y: float,
                             max_x: float, max_y: float) -> bool:
    r = seg.width / 2
    return not (max(seg.start_x, seg.end_x) + r < min_x or
                min(seg.start_x, seg.end_x) - r > max_x or
                max(seg.start_y, seg.end_y) + r < min_y or
                min(seg.start_y, seg.end_y) - r > max_y)


def _polygon_overlaps_window(points, min_x, min_y, max_x, max_y) -> bool:
    if not points:
        return False
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return not (max(xs) < min_x or min(xs) > max_x or
                max(ys) < min_y or min(ys) > max_y)


def make_local_window(pcb_data: PCBData, cx: float, cy: float,
                      half_size: float) -> PCBData:
    """Build a shallow PCBData copy restricted to a square window around (cx, cy).

    Used so per-pad obstacle maps (especially at fine grid steps) only pay for
    geometry near the pad instead of the whole board. The window's bounds are
    installed as board_bounds, so the obstacle builders treat the window edge
    as a board edge - callers must keep the via search radius comfortably
    inside half_size.
    """
    bb = pcb_data.board_info.board_bounds
    min_x, min_y = cx - half_size, cy - half_size
    max_x, max_y = cx + half_size, cy + half_size
    if bb:
        min_x = max(min_x, bb[0])
        min_y = max(min_y, bb[1])
        max_x = min(max_x, bb[2])
        max_y = min(max_y, bb[3])

    m = _ITEM_MARGIN
    wmin_x, wmin_y, wmax_x, wmax_y = min_x - m, min_y - m, max_x + m, max_y + m

    def in_window(x: float, y: float, reach: float = 0.0) -> bool:
        return (wmin_x - reach <= x <= wmax_x + reach and
                wmin_y - reach <= y <= wmax_y + reach)

    local = copy.copy(pcb_data)
    local.vias = [v for v in pcb_data.vias if in_window(v.x, v.y, v.size)]
    local.segments = [s for s in pcb_data.segments
                      if _segment_overlaps_window(s, wmin_x, wmin_y, wmax_x, wmax_y)]
    pads_by_net: Dict[int, List[Pad]] = {}
    for nid, pads in pcb_data.pads_by_net.items():
        kept = [p for p in pads
                if in_window(p.global_x, p.global_y, max(p.size_x, p.size_y))]
        if kept:
            pads_by_net[nid] = kept
    local.pads_by_net = pads_by_net

    board_info = copy.copy(pcb_data.board_info)
    board_info.board_bounds = (min_x, min_y, max_x, max_y)
    board_info.board_cutouts = [
        c for c in pcb_data.board_info.board_cutouts
        if _polygon_overlaps_window(c, wmin_x, wmin_y, wmax_x, wmax_y)
    ]
    keepouts = getattr(pcb_data.board_info, 'keepouts', None)
    if keepouts:
        board_info.keepouts = [
            k for k in keepouts
            if _polygon_overlaps_window(k.get('polygon') or [],
                                        wmin_x, wmin_y, wmax_x, wmax_y)
        ]
    local.board_info = board_info
    if getattr(pcb_data, 'keepout_zones', None):
        local.keepout_zones = [
            z for z in pcb_data.keepout_zones
            if _polygon_overlaps_window(z.points, wmin_x, wmin_y, wmax_x, wmax_y)
        ]
    return local


@dataclass
class TapResult:
    """Result of a single-pad tap attempt."""
    success: bool
    via: Optional[Dict] = None          # New via dict, or None if an existing via was reused
    segments: List[Dict] = field(default_factory=list)
    reused_via_pos: Optional[Tuple[float, float]] = None
    params_label: str = ""              # 'default' or 'fine'
    # Failure diagnostics for rip-up (route_disconnected_planes --rip-blocker-nets):
    via_blocked: bool = False           # True if NO via position could be found
    blocked_cells: List = field(default_factory=list)  # frontier from a failed via->pad route


def _try_distant_pad_trace(pad, pad_layer, net_id, local, routing_obs, config,
                           route_via_to_pad_fn, radius: float):
    """Last-resort connection: route a trace from `pad` to the nearest same-net
    via or same-net pad reachable on `pad_layer`, like a human routing a USB
    connector's GND pin to its adjacent shield pad. Used when no via can be
    placed in/near the pad (e.g. a tiny B.Cu connector pin amid congestion) and
    no same-net via is within the close-reuse radius.

    Returns a successful TapResult, a failure TapResult carrying the blocked
    frontier (so the caller can rip the blocker and retry), or None if there is
    no same-net target within `radius` (then the caller reports via_blocked)."""
    if not pad_layer:
        return None
    px, py = pad.global_x, pad.global_y
    cands: List[Tuple[float, Tuple[float, float]]] = []
    for v in local.vias:
        if v.net_id == net_id:
            d = math.hypot(v.x - px, v.y - py)
            if 1e-6 < d <= radius:
                cands.append((d, (v.x, v.y)))
    for opad in local.pads_by_net.get(net_id, []):
        if (opad.component_ref == pad.component_ref
                and opad.pad_number == pad.pad_number):
            continue
        reachable = (pad_layer in opad.layers
                     or any(l.startswith('*') for l in opad.layers)
                     or opad.drill > 0)  # via reachable on any layer / through-hole
        if not reachable:
            continue
        d = math.hypot(opad.global_x - px, opad.global_y - py)
        if 1e-6 < d <= radius:
            cands.append((d, (opad.global_x, opad.global_y)))
    if not cands:
        return None
    cands.sort(key=lambda c: c[0])
    best_frontier: List = []
    for _d, pos in cands:
        rr = route_via_to_pad_fn(pos, pad, pad_layer, net_id, routing_obs, config,
                                 verbose=False, return_blocked_cells=True)
        if rr.success and rr.segments:
            return TapResult(success=True, via=None, segments=rr.segments,
                             reused_via_pos=pos)
        if rr.blocked_cells and not best_frontier:
            best_frontier = rr.blocked_cells
    return TapResult(success=False, blocked_cells=best_frontier)


def try_tap_pad(
    pad: Pad,
    pad_layer: Optional[str],
    net_id: int,
    pcb_data: PCBData,
    config: GridRouteConfig,
    max_search_radius: float,
    via_size: float,
    via_drill: float,
    same_net_pad_clearance: float = -1.0,
    pending_pads: Optional[List[Dict]] = None,
    extra_vias: Optional[List[Dict]] = None,
    extra_segments: Optional[List[Dict]] = None,
    verbose: bool = False,
    routing_clearance_cushion: bool = False,
    distant_trace_radius: float = 0.0,
) -> TapResult:
    """Attempt to connect one pad to the plane with the given parameters.

    Builds via-placement and routing obstacle maps on a local window around
    the pad (cheap even at fine grid steps), then tries:
      1. routing a trace to a close same-net via (reuse),
      2. placing a new via (pad center / in-pad / spiral search) and routing
         a trace to the pad if the via is off-center.

    extra_vias/extra_segments are dicts for copper placed earlier in this
    session that is not yet part of pcb_data.

    Returns a TapResult; on success the caller is responsible for adding
    result.via / result.segments to its output lists and to pcb_data.
    """
    # Imported lazily: route_planes imports this module at top level.
    from route_planes import find_via_position, route_via_to_pad

    half_size = max_search_radius + _WINDOW_MARGIN
    local = make_local_window(pcb_data, pad.global_x, pad.global_y, half_size)

    if extra_vias:
        for v in extra_vias:
            if abs(v['x'] - pad.global_x) <= half_size + _ITEM_MARGIN and \
               abs(v['y'] - pad.global_y) <= half_size + _ITEM_MARGIN:
                local.vias.append(Via(x=v['x'], y=v['y'], size=v['size'],
                                      drill=v['drill'], layers=v['layers'],
                                      net_id=v['net_id']))
    if extra_segments:
        for s in extra_segments:
            seg = Segment(start_x=s['start'][0], start_y=s['start'][1],
                          end_x=s['end'][0], end_y=s['end'][1],
                          width=s['width'], layer=s['layer'],
                          net_id=s['net_id'])
            bbx = (pad.global_x - half_size - _ITEM_MARGIN,
                   pad.global_y - half_size - _ITEM_MARGIN,
                   pad.global_x + half_size + _ITEM_MARGIN,
                   pad.global_y + half_size + _ITEM_MARGIN)
            if _segment_overlaps_window(seg, *bbx):
                local.segments.append(seg)

    obstacles = build_via_obstacle_map(
        local, config, net_id, verbose=False,
        same_net_pad_clearance=same_net_pad_clearance)
    routing_obs = None
    if pad_layer:
        # Optional half-grid clearance cushion: build_routing_obstacle_map
        # blocks cells by center distance, so routed traces can end up a
        # fraction of a grid step (~0.015mm at 0.05 grid) closer to other
        # copper than the configured clearance. Used for the fine-parameter
        # retry, where the clearance is checked with zero margin.
        routing_config = config
        if routing_clearance_cushion:
            routing_config = replace(
                config, clearance=config.clearance + config.grid_step / 2)
        routing_obs = build_routing_obstacle_map(
            local, routing_config, net_id, pad_layer,
            skip_pad_blocking=False, verbose=False)

    coord = GridCoord(config.grid_step)

    # 1. Try reusing a close same-net via (mirrors route_planes' close-via path)
    close_radius = via_size * 2.5
    best = None
    for v in local.vias:
        if v.net_id != net_id:
            continue
        d2 = (v.x - pad.global_x) ** 2 + (v.y - pad.global_y) ** 2
        if d2 <= close_radius * close_radius and (best is None or d2 < best[0]):
            best = (d2, (v.x, v.y))
    if best is not None and pad_layer:
        segs = route_via_to_pad(best[1], pad, pad_layer, net_id,
                                routing_obs, config, verbose=False)
        if segs:  # non-empty trace: actually connects the pad to the via
            return TapResult(success=True, via=None, segments=segs,
                             reused_via_pos=best[1])

    # 2. Place a new via near the pad
    failed_positions: Set[Tuple[int, int]] = set()
    via_pos = find_via_position(
        pad, obstacles, coord, max_search_radius,
        routing_obstacles=routing_obs,
        config=config,
        pad_layer=pad_layer,
        net_id=net_id,
        verbose=verbose,
        failed_route_positions=failed_positions,
        pending_pads=pending_pads,
    )
    if via_pos is None:
        # No via site anywhere in the search radius. Before giving up, try
        # connecting to a nearby same-net pad/via with a trace (the human's
        # connector-pin-to-shield-pad strategy) - this is the only way a tiny
        # pad amid congestion can connect, and its blocked frontier lets a
        # rip-up caller free the path.
        if distant_trace_radius > 0:
            r = _try_distant_pad_trace(pad, pad_layer, net_id, local, routing_obs,
                                       config, route_via_to_pad, distant_trace_radius)
            if r is not None:
                return r
        # No via site anywhere in the search radius - via placement is blocked.
        return TapResult(success=False, via_blocked=True)

    segments: List[Dict] = []
    # A via landing inside the pad's own copper connects it directly (via-in-pad)
    # -- no trace needed. find_via_position may return an off-centre but still
    # in-pad position when the exact centre is blocked (issue #99).
    in_pad = (abs(via_pos[0] - pad.global_x) <= pad.size_x / 2 + 1e-6 and
              abs(via_pos[1] - pad.global_y) <= pad.size_y / 2 + 1e-6)
    if pad_layer and not in_pad:
        # Capture the search frontier on failure so a caller doing rip-up can
        # identify which net is blocking the via->pad trace (route_disconnected_
        # planes --rip-blocker-nets).
        route_result = route_via_to_pad(via_pos, pad, pad_layer, net_id,
                                        routing_obs, config, verbose=False,
                                        return_blocked_cells=True)
        if not route_result.success:
            return TapResult(success=False, blocked_cells=route_result.blocked_cells or [])
        segments = route_result.segments

    via = {'x': via_pos[0], 'y': via_pos[1], 'size': via_size,
           'drill': via_drill, 'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id}
    return TapResult(success=True, via=via, segments=segments)


def tap_pad_with_escalation(
    pad: Pad,
    pad_layer: Optional[str],
    net_id: int,
    pcb_data: PCBData,
    config: GridRouteConfig,
    max_search_radius: float,
    via_size: float,
    via_drill: float,
    same_net_pad_clearance: float = -1.0,
    pending_pads: Optional[List[Dict]] = None,
    extra_vias: Optional[List[Dict]] = None,
    extra_segments: Optional[List[Dict]] = None,
    verbose: bool = False,
    try_default: bool = True,
    fine_for_all: bool = False,
    distant_trace_radius: float = 0.0,
) -> TapResult:
    """Tap a pad, escalating to scoped fine parameters for fine-pitch pads.

    Escalation (issue #104): try the caller's parameters first (skippable via
    try_default=False when they are already known to fail); if that fails and
    the pad is fine-pitch, retry with grid 0.05 / clearance 0.15 /
    track = min(pad min dimension, 0.15) and a smaller via search radius.

    fine_for_all=True drops the fine-pitch gate and escalates every failed
    pad (used by the route_disconnected_planes repair pass, where only a
    handful of last-resort pads remain and small pads in congested
    fine-pitch neighborhoods also benefit from the fine parameters).
    """
    last_failure = TapResult(success=False)
    if try_default:
        result = try_tap_pad(
            pad, pad_layer, net_id, pcb_data, config, max_search_radius,
            via_size, via_drill, same_net_pad_clearance,
            pending_pads, extra_vias, extra_segments, verbose,
            distant_trace_radius=distant_trace_radius)
        if result.success:
            result.params_label = 'default'
            return result
        last_failure = result

    if fine_for_all or pad_is_fine_pitch(pad, pcb_data):
        fine_config = make_fine_tap_config(config, pad)
        result = try_tap_pad(
            pad, pad_layer, net_id, pcb_data, fine_config,
            min(max_search_radius, FINE_TAP_SEARCH_RADIUS),
            via_size, via_drill, same_net_pad_clearance,
            pending_pads, extra_vias, extra_segments, verbose,
            routing_clearance_cushion=True,
            distant_trace_radius=distant_trace_radius)
        if result.success:
            result.params_label = 'fine'
            return result
        last_failure = result

    # Return the last failure so a rip-up caller sees its via_blocked /
    # blocked_cells diagnostics.
    return last_failure


def find_unconnected_plane_pads(
    pcb_data: PCBData,
    net_id: int,
    zone_layers: Set[str],
) -> List[Tuple[Pad, str]]:
    """Find pads of a plane net with no connection to the plane (issue #99).

    A pad is considered connected when any of these geometric conditions hold:
    - it is a through-hole pad (the zone fill connects it on the plane layer),
    - it sits on a zone layer (or '*.Cu'), so the fill reaches it directly,
    - existing same-net copper (segments/vias/through-hole pads, including
      vias landed inside the pad's own copper) gives it an electrical path
      to a zone layer.

    Zone-fill islands are NOT considered here - the island repair pass in
    route_disconnected_planes handles those.

    Returns list of (pad, pad_layer) for pads needing repair.
    """
    unconnected: List[Tuple[Pad, str]] = []
    for pad in pcb_data.pads_by_net.get(net_id, []):
        if pad.drill > 0:
            continue
        if '*.Cu' in pad.layers or any(zl in pad.layers for zl in zone_layers):
            continue
        pad_layer = None
        for layer in pad.layers:
            if layer.endswith('.Cu') and not layer.startswith('*'):
                pad_layer = layer
                break
        if pad_layer is None:
            continue
        if any(_smd_pad_reaches_layer(pad, zl, net_id, pcb_data)
               for zl in zone_layers):
            continue
        unconnected.append((pad, pad_layer))
    return unconnected
