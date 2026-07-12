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
from __future__ import annotations

import copy
import math
import os
from dataclasses import dataclass, replace, field
from typing import List, Dict, Optional, Tuple, Set

from kicad_parser import PCBData, Pad, Via, Segment, pad_is_plated_through
from routing_config import GridRouteConfig, GridCoord
from routing_utils import point_in_pad_rect
from plane_obstacle_builder import (
    build_via_obstacle_map,
    build_routing_obstacle_map,
    _smd_pad_reaches_layer,
)

# Fine-pitch detection thresholds and fine-tap escalation parameters are
# centralized in routing_defaults (issues #104/#226).
from routing_defaults import (
    FINE_PITCH_NEIGHBOR_DIST,
    FINE_PITCH_MIN_PAD_DIM,
    FINE_TAP_GRID_STEP,
    FINE_TAP_CLEARANCE_STEPS,
    FINE_TAP_SEARCH_RADIUS,
)
from list_nets import fab_floors

_DIST_TOL = 1e-3                  # mm - tolerance so exactly-0.65mm pitch qualifies

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


def note_clearance_used(pcb_data: PCBData, clearance: float) -> None:
    """Record that a routing step used ``clearance`` mm of copper clearance, so
    the board's running minimum (``board_info.min_clearance_used``) tracks the
    tightest clearance any step actually routed to. Downstream the routers fold
    this into the .kicad_pro DRC floor and JSON_SUMMARY so check_drc grades at
    the true routed clearance rather than the looser nominal one."""
    if clearance is None or clearance <= 0:
        return
    bi = pcb_data.board_info
    cur = getattr(bi, 'min_clearance_used', None)
    if cur is None or clearance < cur:
        bi.min_clearance_used = clearance
    # Bridge engine -> main (main holds a different PCBData): see clearance_ledger.
    import clearance_ledger
    clearance_ledger.record(clearance)


def _clearance_ladder(nominal: float, fab_floor: float, n_steps: int) -> List[float]:
    """Descending clearances from just below ``nominal`` down to ``fab_floor``
    (inclusive) in ``n_steps`` even steps. If ``nominal`` is already at/below the
    fab floor, the single ``min(nominal, fab_floor)`` value is returned. Replaces
    the old hard-coded 0.15 mm jump: the floor is the manufacturing limit, and the
    caller stops at the first clearance that routes so the loosest one wins (#226)."""
    floor = min(nominal, fab_floor)
    if nominal <= floor or n_steps < 1:
        return [round(floor, 4)]
    step = (nominal - floor) / n_steps
    return [round(nominal - step * i, 4) for i in range(1, n_steps + 1)]


def fab_floor_clearance_track(pcb_data: PCBData):
    """(clearance, track_width) manufacturing floor for this board's layer count."""
    n_layers = len(pcb_data.board_info.copper_layers) or 2
    floors = fab_floors(n_layers)
    return floors['clearance'], floors['track_width']


def fine_tap_configs(config: GridRouteConfig, pad: Pad, pcb_data: PCBData):
    """Yield progressively tighter tap configs for a fine-pitch pad: a finer grid
    and a narrowed track (floored at the fab track minimum), with the clearance
    stepped DOWN from the caller's value toward the fab clearance floor for the
    board's layer count. Never coarser/wider than the caller already uses. The via
    is NOT shrunk here -- the caller chooses the via, so fab-capability via gating
    lives in one place. Replaces the single hard-coded fine-parameter jump (#226)."""
    fab_clear, fab_track = fab_floor_clearance_track(pcb_data)
    fine_grid = min(config.grid_step, FINE_TAP_GRID_STEP)
    # Narrow the tap track to fit between fine-pitch pads, never below the fab floor.
    fine_track = max(fab_track, min(min(pad.size_x, pad.size_y), config.track_width))
    for clearance in _clearance_ladder(config.clearance, fab_clear, FINE_TAP_CLEARANCE_STEPS):
        yield replace(config, grid_step=fine_grid, clearance=clearance,
                      track_width=fine_track)


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


# --- Spatial index for make_local_window (speedup 3) ---------------------------
# make_local_window filters every board segment/via/pad to a small window around
# one pad. Doing that as an O(all-board) scan per call is fine once, but the
# plane-repair sweep and the via-in-pad escape call it for many pads; a coarse
# bucket index makes each window query O(window) instead of O(board). The index
# is cached on pcb_data and rebuilt only when the copper count changes, so it is
# correct as routing adds/removes copper (the signature catches every mutation
# that changes a count) while being reused across the many static-copper queries
# of the repair sweep.
_TAP_INDEX_CELL = 8.0  # mm bucket size


def _cells_for_bbox(min_x, min_y, max_x, max_y):
    cell = _TAP_INDEX_CELL
    cx0, cy0 = int(math.floor(min_x / cell)), int(math.floor(min_y / cell))
    cx1, cy1 = int(math.floor(max_x / cell)), int(math.floor(max_y / cell))
    for cx in range(cx0, cx1 + 1):
        for cy in range(cy0, cy1 + 1):
            yield (cx, cy)


def _build_tap_spatial_index(pcb_data: PCBData):
    seg_b: Dict[Tuple[int, int], list] = {}
    via_b: Dict[Tuple[int, int], list] = {}
    pad_b: Dict[Tuple[int, int], list] = {}
    for s in pcb_data.segments:
        r = s.width / 2
        for k in _cells_for_bbox(min(s.start_x, s.end_x) - r, min(s.start_y, s.end_y) - r,
                                 max(s.start_x, s.end_x) + r, max(s.start_y, s.end_y) + r):
            seg_b.setdefault(k, []).append(s)
    for v in pcb_data.vias:
        r = v.size
        for k in _cells_for_bbox(v.x - r, v.y - r, v.x + r, v.y + r):
            via_b.setdefault(k, []).append(v)
    for nid, pads in pcb_data.pads_by_net.items():
        for p in pads:
            entry = (nid, p)  # ONE tuple per pad, shared across its cells, so
            hw = max(p.size_x, p.size_y)  # _query_bucket's id()-dedup works
            for k in _cells_for_bbox(p.global_x - hw, p.global_y - hw,
                                     p.global_x + hw, p.global_y + hw):
                pad_b.setdefault(k, []).append(entry)
    return seg_b, via_b, pad_b


def _tap_spatial_index(pcb_data: PCBData):
    sig = (len(pcb_data.segments), len(pcb_data.vias),
           sum(len(v) for v in pcb_data.pads_by_net.values()))
    cached = getattr(pcb_data, '_tap_spatial_index_cache', None)
    if cached is not None and cached[0] == sig:
        return cached[1]
    idx = _build_tap_spatial_index(pcb_data)
    pcb_data._tap_spatial_index_cache = (sig, idx)
    return idx


def _query_bucket(bucket, min_x, min_y, max_x, max_y):
    seen = set()
    out = []
    for k in _cells_for_bbox(min_x, min_y, max_x, max_y):
        for item in bucket.get(k, ()):
            i = id(item)
            if i not in seen:
                seen.add(i)
                out.append(item)
    return out


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

    # Query the cached spatial index for candidates whose bbox overlaps the
    # window, then apply the same precise filters (speedup 3). Falls back to the
    # full board only via the index, which is built from the full board, so the
    # result is identical to the old O(board) scan.
    seg_b, via_b, pad_b = _tap_spatial_index(pcb_data)
    local = copy.copy(pcb_data)
    local.vias = [v for v in _query_bucket(via_b, wmin_x, wmin_y, wmax_x, wmax_y)
                  if in_window(v.x, v.y, v.size)]
    local.segments = [s for s in _query_bucket(seg_b, wmin_x, wmin_y, wmax_x, wmax_y)
                      if _segment_overlaps_window(s, wmin_x, wmin_y, wmax_x, wmax_y)]
    pads_by_net: Dict[int, List[Pad]] = {}
    for nid, p in _query_bucket(pad_b, wmin_x, wmin_y, wmax_x, wmax_y):
        if in_window(p.global_x, p.global_y, max(p.size_x, p.size_y)):
            pads_by_net.setdefault(nid, []).append(p)
    local.pads_by_net = pads_by_net

    board_info = copy.copy(pcb_data.board_info)
    board_info.board_bounds = (min_x, min_y, max_x, max_y)
    # A rectangular board's outline is no longer rectangular W.R.T. the window's
    # clamped bounds, so the edge-blocking rectangularity test would send the
    # WINDOW build down the polygon path while the whole-board map uses the
    # integer rect band. At a row EXACTLY at the edge clearance the two paths
    # break the float tie differently (one-cell disagreement), making window
    # maps inconsistent with the shared map (TAP_MAP_VERIFY, #309 hunt). Drop
    # the outline from the window copy for genuinely rectangular boards so both
    # builds use the same rect-band semantics; polygonal boards keep the
    # outline and use the polygon path in both builds.
    from plane_obstacle_builder import _is_rectangular_outline
    if (bb and pcb_data.board_info.board_outline
            and len(getattr(pcb_data.board_info, 'board_outlines', None) or []) <= 1
            and _is_rectangular_outline(pcb_data.board_info.board_outline, bb)):
        board_info.board_outline = []
        board_info.board_outlines = []
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


# --- In-flight copper registry (issue #310) ------------------------------------
# Phase-3 tap rip-up retries stamp their tap copper into the WORKING obstacle
# map while the ripped victims re-route, and only commit it to pcb_data
# afterwards (phase3_routing). During that window the copper is invisible to
# anything that rebuilds obstacles from pcb_data - in particular the #189
# via-in-pad unblock, whose local via map let a fab-floor via drill straight
# through a pending foreign track (snapdragon ETH_ISOLATEB via on PCIE1_WAKE_N
# In2.Cu, #308/#310). Producers push the pending copper here for the window's
# duration; try_tap_pad callers merge it via extra_vias/extra_segments so via
# placement sees the same copper the A* does.

def push_inflight_copper(pcb_data: PCBData, segments, vias):
    """Register copper that is stamped in the working obstacle map but not yet
    in pcb_data. Returns a token for pop_inflight_copper. Nestable (rip-up
    cascades push their own windows)."""
    entry = (list(segments), list(vias))
    stack = getattr(pcb_data, '_inflight_copper', None)
    if stack is None:
        stack = []
        pcb_data._inflight_copper = stack
    stack.append(entry)
    return entry


def pop_inflight_copper(pcb_data: PCBData, entry) -> None:
    """Unregister a push_inflight_copper window (copper was committed to
    pcb_data or removed from the obstacle map)."""
    stack = getattr(pcb_data, '_inflight_copper', None)
    if not stack:
        return
    for i in range(len(stack) - 1, -1, -1):
        if stack[i] is entry:
            del stack[i]
            return


def inflight_copper_dicts(pcb_data: PCBData):
    """All currently in-flight copper as (extra_vias, extra_segments) dict
    lists in try_tap_pad's format; (None, None) when no window is open."""
    stack = getattr(pcb_data, '_inflight_copper', None)
    if not stack:
        return None, None
    extra_vias, extra_segments = [], []
    for segments, vias in stack:
        for v in vias:
            extra_vias.append({'x': v.x, 'y': v.y, 'size': v.size,
                               'drill': v.drill, 'layers': list(v.layers),
                               'net_id': v.net_id})
        for s in segments:
            extra_segments.append({'start': (s.start_x, s.start_y),
                                   'end': (s.end_x, s.end_y),
                                   'width': s.width, 'layer': s.layer,
                                   'net_id': s.net_id})
    return (extra_vias or None), (extra_segments or None)


# --- Cross-pad via-obstacle-map reuse (issue #263) -----------------------------
# In a plane-repair net-pass every tapped pad excludes the SAME plane net, so
# build_via_obstacle_map yields an identical map for every pad at a given
# parameter set (only other-net copper blocks vias, and the grid is absolute).
# Rebuilding a ~15k-segment window map per pad (~0.29s x hundreds of taps on
# daisho) dominated the repair pass; instead build the map ONCE on the whole
# board per parameter set and reuse it across pads, applying copper changes
# incrementally. Query results are identical: within the via search radius the
# window map and the whole-board map block the same cells (the window pulls in
# every item whose keep-out can reach the search area, and the window-edge band
# sits beyond the search radius) -- asserted per tap under TAP_MAP_VERIFY=1.
_TAP_MAP_VERIFY = os.environ.get('TAP_MAP_VERIFY') == '1'


class SharedViaMaps:
    """Whole-board via-placement obstacle maps shared across the pads of one
    plane-net repair pass, keyed by the config fields the via map depends on.

    Correctness invariants:
    - Maps are refcounted (issue #208): every incremental add/remove must stamp
      the exact cell multiset build_via_obstacle_map stamps for that copper.
      Both notify paths delegate to obstacle_cache.precompute_via_placement_
      obstacles, the builder's verified mirror (tests/test_plane_via_map_sync.py).
    - Callers must notify every pcb_data copper change while an instance is
      live: note_pass_copper after appending a tap's via/segments, and
      note_net_ripped BEFORE a net's copper is removed (it reads the copper)
      followed by resync() after. get() drops all cached maps if the copper
      counts changed without a notify (correct, just slower).

    A key's whole-board map (~0.7s build) is only built once the key has been
    requested _REUSE_THRESHOLD times -- rarely-used fine-escalation configs
    stay on the cheaper per-pad window build. _MAX_MAPS bounds memory (a
    whole-board map at grid 0.05 is tens of MB); least-recently-used maps are
    evicted and rebuilt on demand.
    """

    _REUSE_THRESHOLD = 3
    _MAX_MAPS = 6

    def __init__(self, pcb_data: PCBData, exclude_net_id: int):
        self.pcb_data = pcb_data
        self.exclude_net_id = exclude_net_id
        # key -> [config, same_net_pad_clearance, map, last_use_tick]
        self._maps: Dict[Tuple, list] = {}
        self._use_counts: Dict[Tuple, int] = {}
        self._tick = 0
        self._seg_count = len(pcb_data.segments)
        self._via_count = len(pcb_data.vias)

    @staticmethod
    def _key(config: GridRouteConfig, same_net_pad_clearance: float) -> Tuple:
        return (config.grid_step, config.clearance, config.via_size,
                config.via_drill, config.hole_to_hole_clearance,
                config.board_edge_clearance, same_net_pad_clearance)

    def get(self, config: GridRouteConfig, same_net_pad_clearance: float = -1.0):
        """Whole-board via map for this config, or None when the caller should
        build its own per-pad window map (key below the reuse threshold)."""
        if (len(self.pcb_data.segments) != self._seg_count or
                len(self.pcb_data.vias) != self._via_count):
            # Copper changed without a notify -- cached maps are stale. Drop
            # them all rather than risk querying a wrong map.
            self._maps.clear()
            self.resync()
        self._tick += 1
        key = self._key(config, same_net_pad_clearance)
        entry = self._maps.get(key)
        if entry is not None:
            entry[3] = self._tick
            return entry[2]
        n = self._use_counts.get(key, 0) + 1
        self._use_counts[key] = n
        if n < self._REUSE_THRESHOLD:
            return None
        if len(self._maps) >= self._MAX_MAPS:
            lru = min(self._maps, key=lambda k: self._maps[k][3])
            del self._maps[lru]
        m = build_via_obstacle_map(self.pcb_data, config, self.exclude_net_id,
                                   verbose=False,
                                   same_net_pad_clearance=same_net_pad_clearance)
        self._maps[key] = [config, same_net_pad_clearance, m, self._tick]
        return m

    def resync(self):
        """Record the current pcb_data copper counts as the notified state."""
        self._seg_count = len(self.pcb_data.segments)
        self._via_count = len(self.pcb_data.vias)

    # TAP_MAP_VERIFY: number of note_pass_copper (add-side) updates that get the
    # expensive full-map divergence check. The add path stamps the same
    # single-via multiset every time, so a handful of checks covers it; rips
    # (the #208-risk remove path) are ALWAYS checked via verify_maps_full().
    _VERIFY_ADD_BUDGET = 5

    def verify_maps_full(self):
        """TAP_MAP_VERIFY=1: assert each cached map's blocked-status agrees
        EVERYWHERE with a fresh whole-board rebuild from the current pcb_data.

        The per-tap check in try_tap_pad only covers cells that tap queries; a
        refcount desync from an incremental add/remove (issue #208 risk class)
        could hide where no later tap happens to look. This sweeps the whole
        board, so a count driven to zero early (wrongly unblocked) or a stale
        leftover (wrongly blocked) is caught at the update that caused it."""
        bb = self.pcb_data.board_info.board_bounds
        if not bb:
            return
        for key, (config, snpc, m, _t) in self._maps.items():
            fresh = build_via_obstacle_map(self.pcb_data, config,
                                           self.exclude_net_id, verbose=False,
                                           same_net_pad_clearance=snpc)
            coord = GridCoord(config.grid_step)
            cx, cy = coord.to_grid((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2)
            r = int(max(bb[2] - bb[0], bb[3] - bb[1]) / config.grid_step / 2) + 50
            a = m.open_via_cells_within(cx, cy, r)
            b = fresh.open_via_cells_within(cx, cy, r)
            assert a == b, (
                f"TAP_MAP_VERIFY: shared via map diverged from a fresh rebuild "
                f"after an incremental update (key={key}, {len(a)} vs {len(b)} "
                f"open cells)")

    def _stamp_cells(self, segments, vias, config) -> "np.ndarray":
        """Via-map cell multiset build_via_obstacle_map stamps for these items,
        via the #208-verified builder mirror. Items may span several nets."""
        from obstacle_cache import precompute_via_placement_obstacles
        import numpy as np

        class _Shim:
            pass

        shim = _Shim()
        shim.segments = [s for s in segments if s.net_id != self.exclude_net_id]
        shim.vias = list(vias)
        chunks = []
        for nid in ({s.net_id for s in shim.segments} | {v.net_id for v in shim.vias}):
            d = precompute_via_placement_obstacles(shim, nid, config, [])
            if len(d.blocked_vias):
                chunks.append(d.blocked_vias)
        if not chunks:
            return np.empty((0, 2), dtype=np.int32)
        return np.concatenate(chunks)

    def note_pass_copper(self, new_vias, new_segments=()):
        """Stamp copper just appended to pcb_data into every cached map.

        New vias block via placement regardless of net; segments only when on a
        foreign net (the excluded plane net's own segments never enter the via
        map, and in this pass new segments are always the plane net's)."""
        for entry in self._maps.values():
            cells = self._stamp_cells(new_segments, new_vias, entry[0])
            if len(cells):
                entry[2].add_blocked_vias_batch(cells)
        self.resync()
        if _TAP_MAP_VERIFY and self._VERIFY_ADD_BUDGET > 0:
            self._VERIFY_ADD_BUDGET -= 1  # instance attr shadows the class default
            self.verify_maps_full()

    def note_net_ripped(self, rip_net_id: int):
        """Remove a ripped net's stamps from every cached map. Must run BEFORE
        the net's copper is removed from pcb_data (the stamps are computed from
        it); call resync() after the removal."""
        from obstacle_cache import precompute_via_placement_obstacles
        for entry in self._maps.values():
            d = precompute_via_placement_obstacles(self.pcb_data, rip_net_id,
                                                   entry[0], [])
            if len(d.blocked_vias):
                entry[2].remove_blocked_vias_batch(d.blocked_vias)

    def note_net_restored(self, net_id: int):
        """Inverse of note_net_ripped for a failed rip-up that put the net's
        copper back (#329). Must run AFTER the copper is restored to pcb_data
        (the stamps are computed from it); calls resync() itself."""
        from obstacle_cache import precompute_via_placement_obstacles
        for entry in self._maps.values():
            d = precompute_via_placement_obstacles(self.pcb_data, net_id,
                                                   entry[0], [])
            if len(d.blocked_vias):
                entry[2].add_blocked_vias_batch(d.blocked_vias)
        self.resync()
        if _TAP_MAP_VERIFY:
            self.verify_maps_full()


def _verify_shared_via_map(shared_obs, local, config, net_id,
                           same_net_pad_clearance, pad, max_search_radius,
                           full_pcb_data=None):
    """TAP_MAP_VERIFY=1: assert the shared whole-board map answers this tap's
    queries exactly like a freshly built per-pad window map (the pre-#263
    behavior). find_via_position reads only is_via_blocked(pad centre) and
    open_via_cells_within(centre, radius); the open list covers every other
    cell it can look at, so list equality proves the tap sees identical maps
    (the Rust query is a deterministic stable sort over a fixed scan order)."""
    ref = build_via_obstacle_map(local, config, net_id, verbose=False,
                                 same_net_pad_clearance=same_net_pad_clearance)
    coord = GridCoord(config.grid_step)
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    assert shared_obs.is_via_blocked(gx, gy) == ref.is_via_blocked(gx, gy), \
        f"TAP_MAP_VERIFY: pad-centre blocked mismatch at {pad.component_ref}.{pad.pad_number}"
    radius = coord.to_grid_dist(max_search_radius)
    a = shared_obs.open_via_cells_within(gx, gy, radius)
    b = ref.open_via_cells_within(gx, gy, radius)
    if a != b:
        sa, sb = set(a), set(b)
        only_shared = sorted(sa - sb)   # open in shared, blocked in window
        only_window = sorted(sb - sa)   # open in window, blocked in shared
        print(f"TAP_MAP_VERIFY divergence at {pad.component_ref}.{pad.pad_number} "
              f"pad=({pad.global_x:.3f},{pad.global_y:.3f}) radius={radius} cells:")
        for label, cells in (("open-in-SHARED-only (shared missing a block)", only_shared),
                             ("open-in-WINDOW-only (window missing a block)", only_window)):
            print(f"  {label}: {len(cells)}")
            for cgx, cgy in cells[:10]:
                print(f"    grid ({cgx},{cgy}) = mm ({cgx * coord.grid_step:.3f},"
                      f"{cgy * coord.grid_step:.3f})")
        dump = os.environ.get("TAP_MAP_VERIFY_DUMP")
        if dump:
            with open(dump, "w") as fh:
                for tag, cells in (("shared_only", only_shared), ("window_only", only_window)):
                    for cgx, cgy in cells:
                        fh.write(f"{tag} {cgx} {cgy}\n")
            # Diagnose: does a fresh WHOLE-BOARD build block these cells?
            if full_pcb_data is not None:
                fresh_full = build_via_obstacle_map(full_pcb_data, config, net_id,
                                                    verbose=False,
                                                    same_net_pad_clearance=same_net_pad_clearance)
                sample = (only_shared or only_window)[:5]
                for cgx, cgy in sample:
                    print(f"    cell ({cgx},{cgy}): shared={shared_obs.is_via_blocked(cgx, cgy)} "
                          f"window={ref.is_via_blocked(cgx, cgy)} "
                          f"fresh_whole_board={fresh_full.is_via_blocked(cgx, cgy)}")
                print(f"    local board_bounds={local.board_info.board_bounds} "
                      f"real={full_pcb_data.board_info.board_bounds} "
                      f"edge_clr={config.board_edge_clearance} clr={config.clearance} "
                      f"grid={config.grid_step} via={config.via_size}")
    assert a == b, (
        f"TAP_MAP_VERIFY: open-via-cell mismatch at {pad.component_ref}."
        f"{pad.pad_number} (shared {len(a)} vs window {len(b)} cells)")


@dataclass
class TapResult:
    """Result of a single-pad tap attempt."""
    success: bool
    via: Optional[Dict] = None          # New via dict, or None if an existing via was reused
    segments: List[Dict] = field(default_factory=list)
    reused_via_pos: Optional[Tuple[float, float]] = None
    params_label: str = ""              # 'default' or 'fine'
    clearance_used: float = 0.0         # copper clearance this tap was routed at
    # Failure diagnostics for rip-up (route_disconnected_planes --rip-blocker-nets):
    via_blocked: bool = False           # True if NO via position could be found
    blocked_cells: List = field(default_factory=list)  # frontier from a failed via->pad route


def _try_distant_pad_trace(pad, pad_layer, net_id, local, routing_obs, config,
                           route_multi_fn, radius: float):
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
                     or pad_is_plated_through(opad))  # any-layer / plated barrel (#328)
        if not reachable:
            continue
        d = math.hypot(opad.global_x - px, opad.global_y - py)
        if 1e-6 < d <= radius:
            cands.append((d, (opad.global_x, opad.global_y)))
    if not cands:
        return None
    # One multi-source A* over ALL candidates (issue #259) instead of one A* per
    # candidate: the router routes the pad to whichever same-net target is
    # shortest-routable, and a genuinely boxed-in pad is proven unreachable in a
    # single frontier exhaustion (whose blocked cells drive the rip-up retry).
    positions = [pos for _d, pos in cands]
    rr, pos = route_multi_fn(positions, pad, pad_layer, net_id, routing_obs, config,
                             verbose=False, return_blocked_cells=True)
    if rr.success and rr.segments:
        return TapResult(success=True, via=None, segments=rr.segments,
                         reused_via_pos=pos)
    return TapResult(success=False, blocked_cells=rr.blocked_cells or [])


def _pad_has_same_net_copper(opad, net_id, local, tol: float = 0.2) -> bool:
    """True if a same-net via or segment endpoint sits on `opad` -- i.e. the pad has
    already been escaped/routed, a cheap proxy for "connected to the plane" (the
    authoritative graph check is in find_unconnected_plane_pads; this stays local)."""
    ox, oy = opad.global_x, opad.global_y
    for v in local.vias:
        if v.net_id == net_id and abs(v.x - ox) < tol and abs(v.y - oy) < tol:
            return True
    for s in local.segments:
        if s.net_id == net_id and (
                (abs(s.start_x - ox) < tol and abs(s.start_y - oy) < tol) or
                (abs(s.end_x - ox) < tol and abs(s.end_y - oy) < tol)):
            return True
    return False


def _try_trace_to_plane_connected(pad, pad_layer, net_id, local, routing_obs, config,
                                  route_multi_fn, radius: float):
    """Issue #180: before dropping a NEW via, connect the pad by a trace to the
    nearest EXISTING same-net copper within `radius`:
      - a same-net via (spans to the plane layer) or through-hole pad -- always
        plane-connected; or
      - a same-net SMD pad that has already been escaped (same-net via/segment on
        it), e.g. an adjacent BGA ball -- ottercast U1.N4 GND connects to its
        neighbour ball U1.L4 this way.
    Such copper already reaches the plane, so a new via would be redundant -- and a
    redundant via can box in a neighbouring foreign pad (castor_pollux U11 -12V).

    Unlike step-1 close-via reuse (radius via_size*2.5 ~1.25mm), this reaches the
    full distant-trace radius so a slightly-farther existing target is preferred
    over a brand-new via. Returns a successful TapResult (via=None) or None."""
    if not pad_layer or radius <= 0:
        return None
    px, py = pad.global_x, pad.global_y
    cands: List[Tuple[float, Tuple[float, float]]] = []
    for v in local.vias:
        if v.net_id == net_id:
            d = math.hypot(v.x - px, v.y - py)
            if 1e-6 < d <= radius:
                cands.append((d, (v.x, v.y)))
    for opad in local.pads_by_net.get(net_id, []):
        if opad.component_ref == pad.component_ref and opad.pad_number == pad.pad_number:
            continue
        # Through-hole same-net pads are always plane-connected. An SMD same-net pad
        # is a valid target only if it is already escaped (has same-net copper on
        # it) -- a bare ball would itself be unconnected and tracing to it would
        # just bond two floating pads -- AND it lies on the tap's OWN layer: a
        # same-layer trace cannot join an SMD pad on the opposite side without a via,
        # so tracing to a cross-layer pad's (x,y) lands the tap on a floating island,
        # not the plane (glasgow +3V3: an F.Cu U27.1 tap traced to B.Cu C69.1,
        # leaving orphaned F.Cu copper that grazed a signal track). Mirrors the
        # layer-reachability guard in _try_distant_pad_trace.
        same_layer = (pad_layer in opad.layers
                      or any(l.startswith('*') for l in opad.layers))
        if pad_is_plated_through(opad) or (same_layer and _pad_has_same_net_copper(opad, net_id, local)):
            d = math.hypot(opad.global_x - px, opad.global_y - py)
            if 1e-6 < d <= radius:
                cands.append((d, (opad.global_x, opad.global_y)))
    if not cands:
        return None
    # One multi-source A* over all existing-copper candidates (issue #259): route
    # the pad to whichever same-net copper is shortest-routable, building the trace
    # directly from the winning path.
    positions = [pos for _d, pos in cands]
    segs, pos = route_multi_fn(positions, pad, pad_layer, net_id, routing_obs,
                               config, verbose=False)
    if segs:  # non-empty trace: the pad now reaches the existing plane copper
        return TapResult(success=True, via=None, segments=segs, reused_via_pos=pos)
    return None


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
    disable_reuse: bool = False,
    shared_via_maps: Optional[SharedViaMaps] = None,
) -> TapResult:
    """Attempt to connect one pad to the plane with the given parameters.

    ``disable_reuse`` skips the "route a trace to existing same-net copper
    instead of dropping a via" shortcuts (steps 1 and 1b) and goes straight to
    placing a NEW via. The caller uses it to force a real via when a prior
    reuse-tap reported success but the pad turned out not to actually reach the
    plane (a stale/ripped or not-itself-plane-connected reuse target).

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
    from route_planes import (find_via_position, route_via_to_pad,
                              route_multi_source_to_pad)

    if max_search_radius > 0:
        half_size = max_search_radius + _WINDOW_MARGIN
    else:
        # Via-IN-pad only (no spiral): the window need only cover the pad plus a
        # via's clearance halo -- any foreign copper farther than that can't block
        # a via placed inside the pad. Sizing it to the pad instead of the fixed
        # 3mm spiral margin shrinks the Rust obstacle-map build ~8x at grid 0.05
        # (#189 via-in-pad unblock). Conservative: full via diameter + clearance +
        # a couple grid cells of slack, and make_local_window still pulls in items
        # within its _ITEM_MARGIN beyond the window, so no near obstacle is missed.
        half_size = (max(pad.size_x, pad.size_y) / 2 + via_size
                     + config.clearance + 2 * config.grid_step)
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

    # Cross-pad via-map reuse (#263): use the pass-shared whole-board map when
    # available. Equivalent to the per-pad window map for every cell this tap
    # can query (see SharedViaMaps); the window path is kept for callers with
    # session-local extra copper (not in pcb_data, so not in the shared map)
    # and for the tiny via-in-pad-only window (max_search_radius == 0), whose
    # window-edge band sits close enough to the pad to matter.
    obstacles = None
    if (shared_via_maps is not None and max_search_radius > 0
            and shared_via_maps.exclude_net_id == net_id
            and not extra_vias and not extra_segments):
        obstacles = shared_via_maps.get(config, same_net_pad_clearance)
        if obstacles is not None and _TAP_MAP_VERIFY:
            _verify_shared_via_map(obstacles, local, config, net_id,
                                   same_net_pad_clearance, pad, max_search_radius,
                                   full_pcb_data=pcb_data)
    if obstacles is None:
        obstacles = build_via_obstacle_map(
            local, config, net_id, verbose=False,
            same_net_pad_clearance=same_net_pad_clearance)
    routing_obs = None
    # The routing-obstacle map is consulted ONLY to route a via->pad TRACE -- i.e.
    # when reusing nearby same-net copper (steps 1/1b, gated on not disable_reuse),
    # for the distant-trace fallback (distant_trace_radius>0), or when
    # find_via_position lands a via OUTSIDE the pad (possible only with
    # max_search_radius>0, since the in-pad ring search ignores it). A pure
    # via-IN-pad placement (max_search_radius=0, disable_reuse, no distant trace)
    # connects by copper overlap and never routes a trace, so its routing map is
    # built-but-unused -- skip the ~0.5s Rust build entirely (#189 perf; the
    # via-in-pad unblock hits this path). find_via_position tolerates a None
    # routing map (it only does the routability check when one is provided).
    _needs_trace_map = (not disable_reuse) or distant_trace_radius > 0 or max_search_radius > 0
    if pad_layer and _needs_trace_map:
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
    if best is not None and pad_layer and not disable_reuse:
        segs = route_via_to_pad(best[1], pad, pad_layer, net_id,
                                routing_obs, config, verbose=False)
        if segs:  # non-empty trace: actually connects the pad to the via
            return TapResult(success=True, via=None, segments=segs,
                             reused_via_pos=best[1])

    # 1b. Before placing a NEW via, prefer a trace to an existing same-net via /
    # through-hole pad within the distant-trace radius -- that copper already
    # reaches the plane, so a new via would be redundant and can box a neighbouring
    # foreign pad (issue #180, castor_pollux U11). Gated on distant_trace_radius>0
    # (the rip-blocker plane repair), same as the distant-trace fallback below.
    if pad_layer and distant_trace_radius > 0 and not disable_reuse:
        r = _try_trace_to_plane_connected(
            pad, pad_layer, net_id, local, routing_obs, config,
            route_multi_source_to_pad, distant_trace_radius)
        if r is not None:
            return r

    # 2. Place a new via near the pad. When the net has zone outline(s), the
    # via must land INSIDE one of them (issue #287, neptune): on a Voronoi-
    # shared plane layer a via in the gap between cells touches no fill --
    # DRC-clean but electrically floating -- while the tap reports success.
    # Nets without zones (pure trace/via repair) are unconstrained as before.
    zone_filter = None
    _net_zone_polys = [z.polygon for z in (getattr(pcb_data, 'zones', None) or [])
                       if z.net_id == net_id and getattr(z, 'polygon', None)
                       and len(z.polygon) >= 3]
    if _net_zone_polys:
        from check_connected import point_in_polygon

        def zone_filter(x, y):
            return any(point_in_polygon(x, y, poly) for poly in _net_zone_polys)

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
        position_filter=zone_filter,
    )
    if via_pos is None:
        # No via site anywhere in the search radius. Before giving up, try
        # connecting to a nearby same-net pad/via with a trace (the human's
        # connector-pin-to-shield-pad strategy) - this is the only way a tiny
        # pad amid congestion can connect, and its blocked frontier lets a
        # rip-up caller free the path.
        if distant_trace_radius > 0:
            r = _try_distant_pad_trace(pad, pad_layer, net_id, local, routing_obs,
                                       config, route_multi_source_to_pad, distant_trace_radius)
            if r is not None:
                return r
        # No via site anywhere in the search radius - via placement is blocked.
        return TapResult(success=False, via_blocked=True)

    segments: List[Dict] = []
    # A via landing inside the pad's own copper connects it directly (via-in-pad)
    # -- no trace needed. find_via_position may return an off-centre but still
    # in-pad position when the exact centre is blocked (issue #99).
    in_pad = point_in_pad_rect(via_pos[0], via_pos[1], pad, 1e-6)
    if pad_layer and not in_pad:
        # The via landed OUTSIDE the pad copper, so a via->pad trace IS needed --
        # but _needs_trace_map can be False (e.g. find_via_position returns an
        # in-bbox ring spot that point_in_pad_rect rejects on a rotated/non-rect
        # pad), leaving routing_obs None. Build it lazily here instead of
        # dereferencing None in route_via_to_pad.
        if routing_obs is None:
            routing_obs = build_routing_obstacle_map(
                local, config, net_id, pad_layer, skip_pad_blocking=False, verbose=False)
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
    disable_reuse: bool = False,
    shared_via_maps: Optional[SharedViaMaps] = None,
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
            distant_trace_radius=distant_trace_radius, disable_reuse=disable_reuse,
            shared_via_maps=shared_via_maps)
        if result.success:
            result.params_label = 'default'
            result.clearance_used = config.clearance
            note_clearance_used(pcb_data, config.clearance)
            return result
        last_failure = result

    if fine_for_all or pad_is_fine_pitch(pad, pcb_data):
        # Step the clearance down toward the fab floor (finer grid + narrower
        # track), trying each rung until one routes so the LOOSEST working
        # clearance wins (#226). The fine pass searches for a NEW via site at a
        # thin trace / fine grid; keep its NEW-via search capped
        # (FINE_TAP_SEARCH_RADIUS) rather than the full max_search_radius: placing
        # a brand-new via far from the pad at fine width is disruptive (it
        # butterflies neighbouring plane pads -- measured on castor_pollux).
        # Reaching a far EXISTING via is handled separately and safely by
        # distant_trace_radius (= max_search_radius); see try_tap_pad step 1b.
        #
        # Shrink the tap via to fit INSIDE a fine-pitch pad (via-in-pad),
        # borrowing the underpad-fanout clamp (#360). A default 0.5 mm via cannot
        # drop into a 0.4 mm-pitch WLCSP ball: its ring grazes the neighbour balls
        # so no via site exists however fine the grid (the 6-pad completion
        # ceiling on rp2350_fpga_eensy). Clamping the via to the pad's smallest
        # dimension -- down to the fab via floor, escalating standard→advanced
        # (0.25/0.15) with the SAME warning the fanout emits, and honouring an
        # explicit --fab-tier / --fab-overrides pin -- lets the fine pass place a
        # via-in-pad tap where a full via never fit. clamp_via_to_pad is a no-op
        # ('fits') for pads a full via already fits, so normal taps are unchanged.
        # Shrink BOTH the emitted via (scalar args) AND config.via_size, which
        # find_via_position / build_via_obstacle_map use for the placement halo.
        from bga_fanout.geometry import clamp_via_to_pad
        from list_nets import fab_floor_ladder, warn_fab_escalation
        fine_via_size, fine_via_drill = via_size, via_drill
        n_layers = len(pcb_data.board_info.copper_layers) or 2
        cvs, cvd, vstatus, vrung = clamp_via_to_pad(
            via_size, via_drill, pad, fab_floor_ladder(n_layers))
        # Only adopt the clamp when it genuinely produces a SMALLER via than the
        # caller's -- a pad the caller's via already fits ('fits'), or a pad so
        # small the clamp can't shrink below the fab floor it's already at, leaves
        # the via unchanged and must not emit a misleading escalation warning.
        if cvs < via_size - 1e-9:
            fine_via_size, fine_via_drill = cvs, cvd
            if vrung > 0:
                warn_fab_escalation(f"plane tap {pad.component_ref}.{pad.pad_number}")
        via_config = replace(config, via_size=fine_via_size, via_drill=fine_via_drill)
        for fine_config in fine_tap_configs(via_config, pad, pcb_data):
            result = try_tap_pad(
                pad, pad_layer, net_id, pcb_data, fine_config,
                min(max_search_radius, FINE_TAP_SEARCH_RADIUS),
                fine_via_size, fine_via_drill, same_net_pad_clearance,
                pending_pads, extra_vias, extra_segments, verbose,
                routing_clearance_cushion=True,
                distant_trace_radius=distant_trace_radius, disable_reuse=disable_reuse,
                shared_via_maps=shared_via_maps)
            if result.success:
                result.params_label = 'fine'
                result.clearance_used = fine_config.clearance
                note_clearance_used(pcb_data, fine_config.clearance)
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
    # A pad clearly outside the Edge.Cuts outline can never reach the plane and
    # a repair trace toward it (or between two off-board same-net pads) ships as
    # board-edge DRC (issue #291, framework_dock: 170 off-board GND segments) --
    # skip it instead of "repairing" it.
    from check_drc import make_off_board_test
    off_board = make_off_board_test(pcb_data.board_info)
    skipped_off_board = 0

    unconnected: List[Tuple[Pad, str]] = []
    for pad in pcb_data.pads_by_net.get(net_id, []):
        if off_board is not None and off_board(pad.global_x, pad.global_y):
            skipped_off_board += 1
            continue
        if pad_is_plated_through(pad):
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
    if skipped_off_board:
        print(f"  Skipped {skipped_off_board} pad(s) OUTSIDE the board outline "
              f"(unreachable, no repair copper drawn, #291)")
    return unconnected
