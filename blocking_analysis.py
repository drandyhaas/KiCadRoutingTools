#!/usr/bin/env python3
"""
Blocking analysis for failed routes.

When a route fails, analyzes which previously-routed nets are blocking.
Uses frontier data from the router (cells that the A* search tried to
expand into but found blocked) for accurate analysis.

Usage:
    from blocking_analysis import analyze_frontier_blocking

    # After a route fails with frontier data:
    path, iterations, blocked_cells = router.route_with_frontier(...)

    if path is None:
        blockers = analyze_frontier_blocking(
            blocked_cells,      # From router
            pcb_data, config,
            routed_net_paths,   # Dict of net_id -> path
        )
        print_blocking_analysis(blockers)
"""
from __future__ import annotations

from typing import List, Tuple, Dict, Set, Optional
from dataclasses import dataclass

import numpy as np

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord
from check_drc import point_to_pad_distance
from routing_utils import build_layer_map, segment_blocked_cells_array, pad_rect_halfspan

_PACK_OFFSET = 1 << 20  # grid coords stay well within +/-2^20 at any allowed grid step
_COORD_MASK = (1 << 21) - 1


def _pack_cells(gxy: "np.ndarray", layer) -> "np.ndarray":
    """Pack (N,2) grid coords + layer (scalar or (N,) array) into int64 keys."""
    key = ((gxy[:, 0].astype(np.int64) + _PACK_OFFSET) << 21) | \
          (gxy[:, 1].astype(np.int64) + _PACK_OFFSET)
    return (key << 8) | layer


def _unpack_xy(keys: "np.ndarray") -> Tuple["np.ndarray", "np.ndarray"]:
    """Recover (gx, gy) arrays from packed int64 cell keys."""
    gx = (keys >> 29) - _PACK_OFFSET
    gy = ((keys >> 8) & _COORD_MASK) - _PACK_OFFSET
    return gx, gy


def invalidate_obstacle_cache(cache: Dict, net_id: int) -> None:
    """Remove all cache entries for a net_id.

    Cache keys are (net_id, extra_clearance) tuples, so we need to remove
    all entries where the first element matches.
    """
    keys_to_remove = [k for k in cache if k[0] == net_id]
    for k in keys_to_remove:
        del cache[k]


# Geometry-keyed memo under the per-loop obstacle_cache dicts (#341). Those
# dicts are recreated per loop (and the nested phase-3 rip/reroute calls pass
# obstacle_cache=None), so every new dict re-rasterizes every routed net even
# though almost none of their copper changed. This memo keeps the last computed
# cell sets per (net_id, extra_clearance) TOGETHER WITH the exact geometry +
# config values they were computed from; a hit requires the stored signature to
# equal the net's current signature, so a reused value is byte-for-byte what a
# fresh compute_net_obstacle_cells call would return. It only serves MISSES of
# the caller's obstacle_cache -- cache-hit semantics there are untouched.
_NET_CELLS_MEMO: Dict[Tuple[int, float], Tuple[tuple, Tuple[frozenset, frozenset]]] = {}


def _net_geometry_signature(net_id, path, segments, vias, config, extra_clearance):
    """Value-based signature of every input compute_net_obstacle_cells reads.

    Numeric geometry is stored as raw numpy byte blobs rather than tuples of
    Python numbers: exact same value-equality (identical floats/ints have
    identical bit patterns; a -0.0/0.0 flip merely forces a spurious
    recompute), at ~7x less retained memory per path point / segment.
    """
    params = (config.grid_step, config.clearance, config.track_width,
              config.via_size, tuple(config.layers),
              tuple(config.get_track_width(l) for l in config.layers),
              extra_clearance)
    if path:
        path_arr = np.asarray(path)
        if path_arr.dtype == object:  # ragged/exotic contents: keep exact tuples
            path_sig = tuple(map(tuple, path))
        else:
            path_sig = (path_arr.dtype.str, path_arr.shape, path_arr.tobytes())
    else:
        path_sig = None
    seg_sig = (np.array([(s.start_x, s.start_y, s.end_x, s.end_y, s.width)
                         for s in segments], dtype=np.float64).tobytes(),
               tuple(s.layer for s in segments))
    via_sig = np.array([(v.x, v.y, v.size) for v in vias],
                       dtype=np.float64).tobytes()
    return (params, path_sig, seg_sig, via_sig)


class BlockingReport(list):
    """analyze_frontier_blocking's result: a list of BlockingInfo plus
    frontier coverage accounting (audit #5). attributed_cells counts the
    unique frontier cells matched to at least one ROUTED net; the remainder
    is static/unrippable copper (pads, planes, pre-existing tracks, edge)
    that no amount of ripping can clear. Old consumers see a plain list."""
    frontier_cells: int = 0
    attributed_cells: int = 0


@dataclass
class BlockingInfo:
    """Information about how much a net blocks a route."""
    net_id: int
    net_name: str
    blocked_count: int  # Number of frontier cells blocked by this net
    track_cells: int    # Cells blocked by tracks
    via_cells: int      # Cells blocked by vias
    unique_cells: int   # Cells where this net is the ONLY blocker
    near_target_cells: int  # Cells within proximity of target (more critical)
    near_source_cells: int  # Cells within proximity of source
    details: str        # Human-readable details
    # Asserted by a geometry VALIDATOR (via placement, swap validation) rather
    # than inferred from the frontier -- micron-exact identity, sorts first
    # (audit RIPUP_ATTRIBUTION_REVIEW improvement 3).
    validator_named: bool = False


def compute_net_obstacle_cells(
    pcb_data: PCBData,
    net_id: int,
    path: Optional[List[Tuple[int, int, int]]],
    config: GridRouteConfig,
    extra_clearance: float = 0.0,
    net_segments: Optional[List[Segment]] = None,
    net_vias: Optional[List[Via]] = None,
) -> Tuple["np.ndarray", "np.ndarray"]:
    """
    Compute all obstacle cells for a net (tracks and vias).

    net_segments/net_vias: this net's copper, pre-bucketed by the caller (in
    pcb_data list order) to skip the full pcb_data scan; None = scan here.

    Returns (track_keys, via_keys): sorted unique int64 arrays of packed
    (gx, gy, layer) cell keys (see _pack_cells).
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    num_layers = len(config.layers)
    grid_step = config.grid_step
    all_layers = np.arange(num_layers, dtype=np.int64)

    track_parts: List["np.ndarray"] = []
    via_parts: List["np.ndarray"] = []

    def add_track_segment(x1, y1, x2, y2, layer_idx, seg_width):
        # Exact capsule keep-out from the TRUE float segment, matching
        # obstacle_map.add_net_stubs_as_obstacles / _add_segment_obstacle. The old
        # square box + bresenham line over-reached by ~sqrt(2) in diagonal corners
        # and shifted off-grid endpoints, so a frontier cell blocked by one net's
        # real capsule also fell inside another net's square over-reach and the
        # wrong net got ripped (#203). Distances are measured from the real segment.
        layer_track_width = config.get_track_width(config.layers[layer_idx])
        # PR392 parity (audit #5): expand at the obstacle net's pairwise
        # clearance, not the flat base -- on multi-class boards the flat
        # value under-attributes wide-clearance nets' real blocking reach.
        _clr = config.obstacle_clearance(net_id) if hasattr(config, 'obstacle_clearance') else config.clearance
        expansion_mm = layer_track_width / 2 + seg_width / 2 + _clr + extra_clearance
        cells = segment_blocked_cells_array(x1, y1, x2, y2, expansion_mm, grid_step)
        if len(cells):
            track_parts.append(_pack_cells(cells, layer_idx))

    def add_via_keepout(x, y, via_size):
        # A via blocks tracks on EVERY layer within its via->track keep-out radius.
        # A zero-length capsule is a disc, measured from the true float via centre
        # (so an off-grid via is covered without the old floored-circle drift).
        _clr = config.obstacle_clearance(net_id) if hasattr(config, 'obstacle_clearance') else config.clearance
        via_margin = via_size / 2 + config.track_width / 2 + _clr + extra_clearance
        cells = segment_blocked_cells_array(x, y, x, y, via_margin, grid_step)
        if len(cells):
            base = _pack_cells(cells, 0)
            via_parts.append((base[:, None] | all_layers[None, :]).ravel())

    # Add cells from routed path
    if path:
        for i in range(len(path) - 1):
            gx1, gy1, layer1 = path[i]
            gx2, gy2, layer2 = path[i + 1]
            if layer1 != layer2:
                # via blocks all layers
                vx, vy = coord.to_float(gx1, gy1)
                add_via_keepout(vx, vy, config.via_size)
            else:
                x1, y1 = coord.to_float(gx1, gy1)
                x2, y2 = coord.to_float(gx2, gy2)
                add_track_segment(x1, y1, x2, y2, layer1, config.track_width)

    # Add cells from original stubs
    if net_segments is None:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    for seg in net_segments:
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        seg_width = seg.width if getattr(seg, 'width', 0) > 0 else config.get_track_width(seg.layer)
        add_track_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y, layer_idx, seg_width)

    # Add cells from existing vias (block all layers)
    if net_vias is None:
        net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
    for via in net_vias:
        via_size = via.size if getattr(via, 'size', 0) > 0 else config.via_size
        add_via_keepout(via.x, via.y, via_size)

    track_keys = np.unique(np.concatenate(track_parts)) if track_parts else np.empty(0, dtype=np.int64)
    via_keys = np.unique(np.concatenate(via_parts)) if via_parts else np.empty(0, dtype=np.int64)
    return track_keys, via_keys


def analyze_frontier_blocking(
    blocked_cells: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    routed_net_paths: Dict[int, List[Tuple[int, int, int]]],
    exclude_net_ids: Optional[Set[int]] = None,
    extra_clearance: float = 0.0,
    target_xy: Optional[Tuple[float, float]] = None,
    source_xy: Optional[Tuple[float, float]] = None,
    obstacle_cache: Optional[Dict[int, Tuple[Set, Set]]] = None,
    known_blockers: Optional[List[Tuple[int, int]]] = None,
) -> List[BlockingInfo]:
    """
    Analyze which nets are blocking based on frontier data.

    Args:
        blocked_cells: List of (gx, gy, layer) cells that blocked the search
                      (from router's route_with_frontier)
        pcb_data: PCB data with segments, vias, etc.
        config: Routing configuration
        routed_net_paths: Dict mapping net_id -> routed path for previously routed nets
        exclude_net_ids: Net IDs to exclude from analysis (e.g., the current net)
        extra_clearance: Extra clearance for diff pair centerline routing
        target_xy: Optional (x, y) target coordinates in mm for proximity analysis
        source_xy: Optional (x, y) source coordinates in mm for proximity analysis
        obstacle_cache: Optional cache of net_id -> (track_cells, via_cells) to avoid
                       recomputing obstacle cells. Pass same dict across retry iterations.

    Returns:
        List of BlockingInfo sorted by blocking priority
    """
    if not blocked_cells:
        return []

    exclude_net_ids = exclude_net_ids or set()
    blocked_arr = np.asarray(list(blocked_cells), dtype=np.int64)
    blocked_keys = np.unique(_pack_cells(blocked_arr[:, :2], blocked_arr[:, 2]))

    # Compute source/target grid coords and proximity threshold
    coord = GridCoord(config.grid_step)
    target_gx, target_gy = None, None
    source_gx, source_gy = None, None
    # "Near" = within 3mm (30 grid cells at 0.1mm grid)
    near_radius_grid = int(3.0 / config.grid_step)
    if target_xy is not None:
        target_gx, target_gy = coord.to_grid(target_xy[0], target_xy[1])
    if source_xy is not None:
        source_gx, source_gy = coord.to_grid(source_xy[0], source_xy[1])

    # First pass: compute obstacle cells for each net and intersect with the
    # blocked frontier (all arrays are sorted unique packed keys)
    net_blocking_data = {}  # net_id -> (blocking_track, blocking_via, blocking_total)

    # Use provided cache or create local one
    local_cache = obstacle_cache if obstacle_cache is not None else {}

    # The blocked frontier is small and constant across the net loop; iterate IT
    # against each net's (large) obstacle-cell SET rather than re-sorting the big
    # array with np.intersect1d every call (~8s of the signal route). blocked_list
    # is sorted-unique, so filtering it in order yields the same sorted result
    # intersect1d did -- byte-for-byte identical blocker counts/ranking.
    blocked_fs = frozenset(blocked_keys.tolist())

    # One pass over pcb_data buckets every candidate net's copper (#341); built
    # lazily on the first obstacle_cache miss, since a fully-cached call needs
    # neither the buckets nor the signatures.
    segs_by_net = None
    vias_by_net = None

    for net_id, path in routed_net_paths.items():
        if net_id in exclude_net_ids:
            continue

        # Get obstacle cells from cache or compute. Cache key includes
        # extra_clearance since it affects expansion radius. Cached as frozensets
        # for O(1) membership (the values are private to this intersection).
        cache_key = (net_id, extra_clearance)
        cached = local_cache.get(cache_key)
        if cached is None:
            if segs_by_net is None:
                candidate_ids = set(routed_net_paths) - exclude_net_ids
                segs_by_net = {nid: [] for nid in candidate_ids}
                for seg in pcb_data.segments:
                    bucket = segs_by_net.get(seg.net_id)
                    if bucket is not None:
                        bucket.append(seg)
                vias_by_net = {nid: [] for nid in candidate_ids}
                for via in pcb_data.vias:
                    bucket = vias_by_net.get(via.net_id)
                    if bucket is not None:
                        bucket.append(via)
            net_segments = segs_by_net.get(net_id, [])
            net_vias = vias_by_net.get(net_id, [])
            # Rasterizing is the phase-3 hot path (#341); the geometry-keyed
            # memo returns the identical cell sets when nothing this net's
            # cells depend on has changed since they were last computed.
            sig = _net_geometry_signature(net_id, path, net_segments, net_vias,
                                          config, extra_clearance)
            memo_entry = _NET_CELLS_MEMO.get(cache_key)
            if memo_entry is not None and memo_entry[0] == sig:
                cached = memo_entry[1]
            else:
                track_keys, via_keys = compute_net_obstacle_cells(
                    pcb_data, net_id, path, config, extra_clearance,
                    net_segments=net_segments, net_vias=net_vias
                )
                cached = (frozenset(track_keys.tolist()), frozenset(via_keys.tolist()))
                _NET_CELLS_MEMO[cache_key] = (sig, cached)
            local_cache[cache_key] = cached
        track_set, via_set = cached

        # Count how many blocked frontier cells this net is responsible for.
        # frozenset & frozenset is a C-level intersection that iterates the SMALLER
        # (blocked) set, so it costs O(blocked) without a Python loop or re-sorting
        # the net's big cell array. sorted() restores intersect1d's ordering, so the
        # blocker counts/ranking stay byte-for-byte identical.
        bt = blocked_fs & track_set
        bv = blocked_fs & via_set
        if not bt and not bv:
            continue
        blocking_track = np.array(sorted(bt), dtype=np.int64)
        blocking_via = np.array(sorted(bv), dtype=np.int64)
        blocking_total = np.union1d(blocking_track, blocking_via)
        net_blocking_data[net_id] = (blocking_track, blocking_via, blocking_total)

    # Cells blocked by exactly one net (each net contributes each cell once)
    if net_blocking_data:
        all_blocking = np.concatenate([d[2] for d in net_blocking_data.values()])
        cells, counts = np.unique(all_blocking, return_counts=True)
        solo_cells = cells[counts == 1]
        n_attributed = int(cells.size)
    else:
        solo_cells = np.empty(0, dtype=np.int64)
        n_attributed = 0

    # Second pass: count unique blocking and near-source/target blocking
    results = []
    for net_id, (blocking_track, blocking_via, blocking_total) in net_blocking_data.items():
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"Net {net_id}"

        # Count cells where this net is the ONLY blocker
        unique_count = int(np.isin(blocking_total, solo_cells, assume_unique=True).sum())

        # Count cells near target and source
        near_target_count = 0
        near_source_count = 0
        gx, gy = _unpack_xy(blocking_total)
        if target_gx is not None:
            near_target_count = int(((gx - target_gx) ** 2 + (gy - target_gy) ** 2
                                     <= near_radius_grid ** 2).sum())
        if source_gx is not None:
            near_source_count = int(((gx - source_gx) ** 2 + (gy - source_gy) ** 2
                                     <= near_radius_grid ** 2).sum())

        details = f"{len(blocking_track)} track, {len(blocking_via)} via cells on frontier"
        results.append(BlockingInfo(
            net_id=net_id,
            net_name=net_name,
            blocked_count=len(blocking_total),
            track_cells=len(blocking_track),
            via_cells=len(blocking_via),
            unique_cells=unique_count,
            near_target_cells=near_target_count,
            near_source_cells=near_source_count,
            details=details
        ))

    # Validator-asserted blockers (audit #3): identities a geometry validator
    # proved (via placement, swap validation) -- not inferred from the
    # frontier. Flagged to sort ahead of every inferred tier; a net the
    # frontier never touched still enters the candidate list.
    apply_known_blockers(results, known_blockers, exclude_net_ids, pcb_data)

    rank_blockers(results, getattr(config, 'ripup_blocker_select', 'count'))

    report = BlockingReport(results)
    report.frontier_cells = int(blocked_keys.size)
    report.attributed_cells = n_attributed
    return report


def apply_known_blockers(results, known_blockers, exclude_net_ids, pcb_data):
    """Fold validator-asserted blocker identities into a BlockingInfo list
    (audit #3): flag entries already attributed by the frontier, append
    entries the frontier never touched. Shared by analyze_frontier_blocking
    and the per-edge merge in the phase-3 tap cascade. Mutates `results`."""
    if not known_blockers:
        return
    exclude_net_ids = exclude_net_ids or set()
    by_id = {b.net_id: b for b in results}
    for nid, n_cells in known_blockers:
        if nid in exclude_net_ids:
            # Never re-inject the failing net or a rip-chain ancestor
            # (cycle guard parity with the frontier path).
            continue
        b = by_id.get(nid)
        if b is not None:
            b.validator_named = True
        else:
            net = pcb_data.nets.get(nid)
            results.append(BlockingInfo(
                net_id=nid,
                net_name=net.name if net else f"Net {nid}",
                blocked_count=n_cells, track_cells=n_cells, via_cells=0,
                unique_cells=n_cells, near_target_cells=n_cells,
                near_source_cells=0,
                details=f"validator-named ({n_cells} conflict cells)",
                validator_named=True))


def merge_blocking_max(a: BlockingInfo, b: BlockingInfo) -> BlockingInfo:
    """Merge two per-edge attributions of the SAME net by max of each
    signal (audit #2, per-edge attribution): a net decisive at one failed
    edge must not be diluted by edges it is irrelevant to, so counts take
    the strongest edge rather than the sum over edges."""
    return BlockingInfo(
        net_id=a.net_id, net_name=a.net_name,
        blocked_count=max(a.blocked_count, b.blocked_count),
        track_cells=max(a.track_cells, b.track_cells),
        via_cells=max(a.via_cells, b.via_cells),
        unique_cells=max(a.unique_cells, b.unique_cells),
        near_target_cells=max(a.near_target_cells, b.near_target_cells),
        near_source_cells=max(a.near_source_cells, b.near_source_cells),
        details=a.details if a.blocked_count >= b.blocked_count else b.details,
        validator_named=a.validator_named or b.validator_named)


def _count_sort_key(x):
    """The historical default ranking ('count'): 100%-unique blockers in a
    top tier, then a weighted score (unique full, near-endpoint-unique extra,
    shared half). validator_named entries always sort first (audit #3)."""
    near_endpoint = x.near_target_cells + x.near_source_cells
    named = 1 if getattr(x, 'validator_named', False) else 0
    if x.blocked_count > 0 and x.unique_cells == x.blocked_count:
        # 100% unique - highest priority, use near_endpoint as tiebreaker
        return (named, 2, x.unique_cells, near_endpoint)
    else:
        # Weighted score: unique counts full, near-endpoint unique counts extra, shared counts half
        shared = x.blocked_count - x.unique_cells
        # Near-endpoint unique cells are extra valuable
        near_endpoint_unique = min(near_endpoint, x.unique_cells)
        score = x.unique_cells + near_endpoint_unique + 0.5 * shared
        return (named, 1, score, near_endpoint)


def rank_blockers(results, algo: str = 'count',
                  bidir_both_sets=None) -> None:
    """Order BlockingInfo in place by the selected algorithm (#424 audit;
    --ripup-blocker-select).

    'count'       - the historical weighted-count ranking (default; exact
                    legacy ordering modulo the validator_named-first rule).
    'near-target' - endpoint-proximity first (audit improvement 2): the true
                    last-mile blocker hugs the failing endpoint but has FEW
                    cells; count ranking buries it under big far walls.
                    Key: near-endpoint-unique, then unique, then count.
    'bidir'       - both-directions boost (audit improvement 4): a net
                    attributed in BOTH search directions' frontiers lies on a
                    genuine separating wall; bystanders behind one endpoint
                    appear in only one. Weighted-count score doubled for
                    both-direction nets (bidir_both_sets = set of net_ids).
    'mincut' ranking is not a sort: the probe (mincut_probe_order) reorders
    candidates by actual path crossings; after that reorder the list is
    already in rip order and this function is not called.
    """
    if algo == 'near-target':
        def key(x):
            near = x.near_target_cells + x.near_source_cells
            named = 1 if getattr(x, 'validator_named', False) else 0
            return (named, min(near, x.unique_cells), near, x.unique_cells,
                    x.blocked_count)
        results.sort(key=key, reverse=True)
    elif algo == 'bidir':
        both = bidir_both_sets or set()
        def key(x):
            base = _count_sort_key(x)
            boost = 2.0 if x.net_id in both else 1.0
            # scale the score component of the count key
            return (base[0], base[1], base[2] * boost, base[3])
        results.sort(key=key, reverse=True)
    else:
        results.sort(key=_count_sort_key, reverse=True)


def mincut_probe_order(pcb_data, config, working_obstacles, net_id,
                       rippable_blockers, net_obstacles_cache):
    """Min-cut probe (#424 audit improvement 1; --ripup-blocker-select=mincut).

    The frontier report is the PERIMETER of the reachable pocket -- per-net
    cell counts measure exposure, not cut-ness, so ranking guesses. This
    replaces the guess with a measurement: on a CLONE of the working map
    (zero mutation risk), lift every rippable candidate's hard blocks, stamp
    its cells as a large soft cost instead, and run the failed search once.
    A path now always exists if the net is routable at ANY rip depth -- and
    because A* minimizes cost, it threads the CHEAPEST crossing set: the
    (near-)minimal set of nets separating source from target, jointly.

    Returns (ordered_net_ids, feasible): candidates the probe path actually
    crosses, in first-crossing order (rip these, in this order); feasible
    False means even the soft-cost search failed -- the separating wall is
    static (unrippable) copper. Callers should treat that as ADVISORY, not
    proof the ladder is useless: rip retries also unlock pad-via placement
    and swaps, topology changes this path probe does not model.
    """
    from dataclasses import replace as _replace
    from obstacle_cache import remove_net_obstacles_from_cache
    import numpy as np

    cand_ids = [b.net_id for b in rippable_blockers
                if b.net_id in net_obstacles_cache]
    if not cand_ids:
        return [], True

    clone = working_obstacles.clone_fresh()
    # In the phase-3 cascade the probed net's OWN phase-1 copper is already
    # stamped in the working map (unlike the SE ladder, where the net has no
    # copper yet); lift it or the probe collides with itself and reports a
    # false infeasible.
    own = net_obstacles_cache.get(net_id)
    if own is not None:
        remove_net_obstacles_from_cache(clone, own)
    cell_sets = {}
    soft_rows = []
    cost = config.cell_cost(5.0)   # ~5mm-equiv per cell: cross only if needed
    for nid in cand_ids:
        data = net_obstacles_cache[nid]
        cells = getattr(data, 'blocked_cells', None)
        if cells is None or not len(cells):
            continue
        remove_net_obstacles_from_cache(clone, data)
        cs = set()
        for i in range(len(cells)):
            gx, gy, li = cells[i]
            cs.add((int(gx), int(gy), int(li)))
            soft_rows.append((int(li), int(gx), int(gy), cost))
        cell_sets[nid] = cs
    if not soft_rows:
        return [], True
    clone.set_layer_proximity_batch(np.array(soft_rows, dtype=np.int32))

    from single_ended_routing import route_net_with_obstacles
    probe_cfg = _replace(config, max_rip_up_count=0) \
        if hasattr(config, 'max_rip_up_count') else config
    result = route_net_with_obstacles(pcb_data, net_id, probe_cfg, clone)
    if not result or result.get('failed') or not result.get('path'):
        return [], False

    # Densify the path so cells between waypoints register crossings.
    path = result['path']
    dense = []
    for (x1, y1, l1), (x2, y2, l2) in zip(path, path[1:]):
        dense.append((x1, y1, l1))
        if l1 == l2:
            n = max(abs(x2 - x1), abs(y2 - y1))
            for k in range(1, n):
                dense.append((x1 + round((x2 - x1) * k / n),
                              y1 + round((y2 - y1) * k / n), l1))
    dense.append(path[-1])

    order = []
    seen = set()
    for cell in dense:
        for nid, cs in cell_sets.items():
            if nid not in seen and cell in cs:
                seen.add(nid)
                order.append(nid)
    return order, True


def analyze_static_blockers(
    blocked_cells: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    nets_to_route: Optional[Set[int]] = None,
) -> Dict[str, List[str]]:
    """
    Analyze what static obstacles are blocking the given cells.

    Returns a dict with categories of blockers:
    - 'pads': list of "NetName (pad ref)" strings for blocking pads
    - 'tracks': list of "NetName" strings for pre-existing tracks
    - 'zones': count of cells in BGA exclusion zones
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    blocked_set = set(blocked_cells)
    nets_to_route = nets_to_route or set()

    result = {
        'pads': [],
        'tracks': [],
        'zone_cells': 0,
    }

    # Track which nets' pads are blocking
    pad_blockers = {}  # net_name -> set of pad refs

    # Check pads from other nets
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route:
            continue
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"Net {net_id}"
        for pad in pads:
            # Get pad grid area
            pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

            # Calculate pad expansion (matches obstacle_map.py formula)
            # margin = track_width/2 + clearance (route needs half-width from center + clearance from pad edge)
            # rotated-rect bbox so a tilted pad's blocked cells are covered
            pad_half_w, pad_half_h = pad_rect_halfspan(pad)
            margin = config.track_width / 2 + config.clearance
            expand_x = max(1, coord.to_grid_dist(pad_half_w + margin))
            expand_y = max(1, coord.to_grid_dist(pad_half_h + margin))

            # Check if any blocked cells fall within this pad's area. The bbox
            # (expand_x/y) is a fast PREFILTER; confirm with the pad's TRUE copper
            # shape so a round pad's bbox corner doesn't over-attribute a blocked
            # cell it doesn't actually cover (matches the shape-aware obstacle map).
            for cell in blocked_set:
                gx, gy, layer_idx = cell
                layer_name = config.layers[layer_idx] if layer_idx < len(config.layers) else None
                if layer_name and layer_name not in pad.layers:
                    continue
                if abs(gx - pad_gx) <= expand_x and abs(gy - pad_gy) <= expand_y:
                    cx, cy = coord.to_float(gx, gy)
                    if point_to_pad_distance(cx, cy, pad) > margin:
                        continue  # inside the bbox but outside the real pad shape
                    if net_name not in pad_blockers:
                        pad_blockers[net_name] = set()
                    pad_blockers[net_name].add(pad.component_ref)
                    break  # Found blocking, move to next pad

    # Format pad blockers
    for net_name, refs in sorted(pad_blockers.items(), key=lambda x: -len(x[1])):
        if len(refs) <= 3:
            result['pads'].append(f"{net_name} ({', '.join(sorted(refs))})")
        else:
            result['pads'].append(f"{net_name} ({len(refs)} pads)")

    # Check pre-existing tracks from other nets
    track_blockers = set()
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        # Get segment grid area
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        expansion_grid = max(1, coord.to_grid_dist(seg.width / 2 + config.clearance))

        # Check if any blocked cells fall near this segment
        for cell in blocked_set:
            gx, gy, cell_layer = cell
            if cell_layer != layer_idx:
                continue
            # Simple bounding box check
            min_gx = min(gx1, gx2) - expansion_grid
            max_gx = max(gx1, gx2) + expansion_grid
            min_gy = min(gy1, gy2) - expansion_grid
            max_gy = max(gy1, gy2) + expansion_grid
            if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                net = pcb_data.nets.get(seg.net_id)
                net_name = net.name if net else f"Net {seg.net_id}"
                track_blockers.add(net_name)
                break

    result['tracks'] = sorted(track_blockers)

    # Check BGA exclusion zones
    zone_cells = 0
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        for cell in blocked_set:
            gx, gy, _ = cell
            if gmin_x <= gx <= gmax_x and gmin_y <= gy <= gmax_y:
                zone_cells += 1
    result['zone_cells'] = zone_cells

    return result


def print_blocking_analysis(
    blockers: List[BlockingInfo],
    max_display: int = 10,
    prefix: str = "  ",
    blocked_cells: Optional[List[Tuple[int, int, int]]] = None,
    pcb_data: Optional[PCBData] = None,
    config: Optional[GridRouteConfig] = None,
    nets_to_route: Optional[Set[int]] = None,
):
    """Print blocking analysis results."""
    if not blockers:
        msg = f"{prefix}No previously-routed nets blocking"
        # If we have the data, analyze static blockers
        if blocked_cells and pcb_data and config:
            static = analyze_static_blockers(blocked_cells, pcb_data, config, nets_to_route)
            details = []
            if static['pads']:
                details.append(f"pads: {', '.join(static['pads'][:5])}")
                if len(static['pads']) > 5:
                    details[-1] += f" (+{len(static['pads'])-5} more)"
            if static['tracks']:
                details.append(f"pre-existing tracks: {', '.join(static['tracks'][:3])}")
                if len(static['tracks']) > 3:
                    details[-1] += f" (+{len(static['tracks'])-3} more)"
            if static['zone_cells'] > 0:
                details.append(f"BGA zone ({static['zone_cells']} cells)")
            if details:
                print(f"{msg}")
                print(f"{prefix}Static blockers: {'; '.join(details)}")
            else:
                print(f"{msg} (likely blocked by pads/stubs/zones)")
        else:
            print(f"{msg} (likely blocked by pads/stubs/zones)")
        return

    total_blocked = sum(b.blocked_count for b in blockers)
    total_unique = sum(b.unique_cells for b in blockers)
    total_near_target = sum(b.near_target_cells for b in blockers)
    total_near_source = sum(b.near_source_cells for b in blockers)

    summary = f"{prefix}Frontier blocked by {len(blockers)} nets ({total_blocked} cells, {total_unique} unique"
    if total_near_source > 0 or total_near_target > 0:
        summary += f"; near: {total_near_source} src, {total_near_target} tgt"
    summary += ")"
    print(summary)

    # Coverage line (audit #5): frontier cells NO routed net matched are
    # static/unrippable copper -- when that share dominates, the ladder is
    # ripping bystanders and the real wall is not rippable at all.
    frontier_total = getattr(blockers, 'frontier_cells', 0)
    if frontier_total:
        attributed = getattr(blockers, 'attributed_cells', 0)
        unmatched = frontier_total - attributed
        if unmatched > 0:
            print(f"{prefix}Coverage: {attributed}/{frontier_total} frontier cells "
                  f"attributed to routed nets; {unmatched} static/unrippable")

    print(f"{prefix}Top blockers:")
    for i, info in enumerate(blockers[:max_display]):
        pct = 100.0 * info.blocked_count / total_blocked if total_blocked > 0 else 0
        unique_pct = 100.0 * info.unique_cells / info.blocked_count if info.blocked_count > 0 else 0

        # Build output string
        parts = [f"{info.blocked_count} ({pct:.1f}%)"]
        if info.unique_cells > 0:
            parts.append(f"{info.unique_cells} uniq ({unique_pct:.0f}%)")
        else:
            parts.append("no uniq")
        # Show near-source and near-target if either is non-zero
        if info.near_source_cells > 0 or info.near_target_cells > 0:
            parts.append(f"near: {info.near_source_cells} src, {info.near_target_cells} tgt")

        print(f"{prefix}  {i+1}. {info.net_name}: {', '.join(parts)}")

    if len(blockers) > max_display:
        remaining = sum(b.blocked_count for b in blockers[max_display:])
        print(f"{prefix}  ... and {len(blockers) - max_display} more nets ({remaining} cells)")


def filter_rippable_blockers(
    blockers: List[BlockingInfo],
    routed_results: Dict,
    diff_pair_by_net_id: Dict,
    get_canonical_net_id_func
) -> Tuple[List[BlockingInfo], Set[int]]:
    """
    Filter blockers to only those that can be ripped (in routed_results),
    deduplicating by diff pair (P and N count as one).

    Args:
        blockers: List of BlockingInfo from analyze_frontier_blocking
        routed_results: Dict of net_id -> result for routed nets
        diff_pair_by_net_id: Dict mapping net_id -> (pair_name, pair)
        get_canonical_net_id_func: Function to get canonical net ID

    Returns:
        Tuple of (rippable_blockers, seen_canonical_ids)
    """
    rippable_blockers = []
    seen_canonical_ids = set()
    for b in blockers:
        if b.net_id in routed_results:
            canonical = get_canonical_net_id_func(b.net_id, diff_pair_by_net_id)
            if canonical not in seen_canonical_ids:
                seen_canonical_ids.add(canonical)
                rippable_blockers.append(b)
    return rippable_blockers, seen_canonical_ids


if __name__ == "__main__":
    print("Blocking analysis module")
    print("Use analyze_frontier_blocking() with data from route_with_frontier()")
