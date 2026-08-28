"""Conservative invariants checked before any live-board mutation."""

from __future__ import annotations

from functools import lru_cache
import math

from .context import SpatialIndex, line_bbox
from .geometry import point_segment_distance, segment_distance
from .model import GlossResult, Segment, segment_key
from .pads import segment_hits_pad


def _addition_clearance(model, addition):
    if addition.clearance >= 0.0:
        return max(model.minimum_clearance, addition.clearance)
    return max(model.minimum_clearance,
               model.net_clearances.get(addition.net_id, 0.0))


def validate_result(model, eligible_keys, result: GlossResult,
                    check_connectivity=True):
    known = {segment_key(s): s for s in model.segments}
    if len(result.remove_keys) != len(set(result.remove_keys)):
        raise ValueError("The candidate removes the same track more than once")
    if any(key not in eligible_keys for key in result.remove_keys):
        raise ValueError("The candidate attempts to remove an unselected track")
    if any(key not in known for key in result.remove_keys):
        raise ValueError("The candidate references a track no longer present")
    removed = [known[key] for key in result.remove_keys]
    if removed and not result.additions:
        raise ValueError("The candidate removes copper without replacing it")
    removed_nets = {segment.net_id for segment in removed}
    added_nets = {segment.net_id for segment in result.additions}
    if removed_nets - added_nets:
        raise ValueError("The candidate does not replace every affected net")
    allowed_geometry = {(segment.net_id, segment.layer, round(segment.width, 6))
                        for segment in removed}
    for new in result.additions:
        if new.width <= 0 or new.net_id <= 0 or new.start == new.end:
            raise ValueError("Invalid replacement segment")
        if removed and (new.net_id, new.layer, round(new.width, 6)) not in allowed_geometry:
            raise ValueError("Replacement segment changes net, layer, or width")
    additions = list(result.additions)
    addition_positions = {id(item): index for index, item in enumerate(additions)}
    max_width = max([item.width for item in additions] + [0.0])
    max_clearance = max(
        [model.minimum_clearance] + list(model.net_clearances.values()) +
        [_addition_clearance(model, item) for item in additions] + [0.0])
    addition_index = SpatialIndex(
        additions, lambda item: line_bbox(item.start, item.end), 5.0)
    for index, first in enumerate(additions):
        margin = max_clearance + (first.width + max_width) / 2.0
        for second in addition_index.query(
                line_bbox(first.start, first.end, margin)):
            if addition_positions[id(second)] <= index:
                continue
            if first.layer != second.layer or first.net_id == second.net_id:
                continue
            clearance = max(
                _addition_clearance(model, first),
                _addition_clearance(model, second))
            required = clearance + (first.width + second.width) / 2.0
            if segment_distance(first.start, first.end,
                                second.start, second.end) < required - 1e-6:
                raise ValueError("Candidate additions violate inter-net clearance")
    if check_connectivity:
        _validate_connectivity(model, eligible_keys, result)


# A complete planning round can validate hundreds of composed candidates.  The
# unchanged-board partitions are shared, while each candidate contributes a
# short-lived "after" state; a tiny cache evicted the reusable baseline before
# later candidates reached it.  This bound keeps the cache finite while
# covering normal whole-board rounds.
@lru_cache(maxsize=1024)
def _connectivity_partition(segments, obstacles, pad_regions,
                            immutable_keys, net_id, tolerance):
    """Return stable connected terminal groups for one net.

    Arguments are immutable tuples/frozensets so the unchanged-board side of
    repeated candidate checks is calculated once.  Spatial indexes only prune
    impossible contacts; the existing exact geometry predicates still decide
    every union.
    """
    segs = [s for s in segments if s.net_id == net_id]
    # When exact pad regions are present, do not let their older conservative
    # obstacle circles prove connectivity outside real pad copper.
    obs = [o for o in obstacles if o.net_id == net_id and
           (not pad_regions or o.kind != "pad")]
    pads = [pad for pad in pad_regions if pad.net_id == net_id]
    count = len(segs) + len(obs) + len(pads)
    parent = list(range(count))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[b] = a

    segment_positions = {id(item): index for index, item in enumerate(segs)}
    max_width = max([item.width for item in segs] + [0.0])
    spatial = SpatialIndex(
        segs, lambda item: line_bbox(
            (item.start_x, item.start_y), (item.end_x, item.end_y)), 5.0)
    obstacle_spatial = SpatialIndex(
        obs, lambda item: (item.x, item.y, item.x, item.y), 5.0)
    pad_spatial = SpatialIndex(
        pads, lambda item: (item.x, item.y, item.x, item.y), 5.0)
    obstacle_positions = {id(item): index for index, item in enumerate(obs)}
    pad_positions = {id(item): index for index, item in enumerate(pads)}
    max_obstacle_radius = max([item.radius for item in obs] + [0.0])
    max_pad_radius = max(
        [math.hypot(item.width, item.height) / 2.0 for item in pads] + [0.0])
    for i, a in enumerate(segs):
        aa, ab = (a.start_x, a.start_y), (a.end_x, a.end_y)
        margin = (a.width + max_width) / 2.0 + tolerance
        for b in spatial.query(line_bbox(aa, ab, margin)):
            j = segment_positions[id(b)]
            if j <= i:
                continue
            if a.layer != b.layer:
                continue
            if segment_distance(aa, ab, (b.start_x, b.start_y),
                                (b.end_x, b.end_y)) <= \
                    (a.width + b.width) / 2.0 + tolerance:
                union(i, j)
        obstacle_margin = max_obstacle_radius + a.width / 2.0 + tolerance
        for obstacle in obstacle_spatial.query(
                line_bbox(aa, ab, obstacle_margin)):
            if obstacle.layers and a.layer not in obstacle.layers:
                continue
            if point_segment_distance((obstacle.x, obstacle.y), aa, ab) <= \
                    obstacle.radius + a.width / 2.0 + tolerance:
                union(i, len(segs) + obstacle_positions[id(obstacle)])
        pad_margin = max_pad_radius + a.width / 2.0 + tolerance
        for pad in pad_spatial.query(line_bbox(aa, ab, pad_margin)):
            if pad.layers and a.layer not in pad.layers:
                continue
            if segment_hits_pad(
                    pad, aa, ab, margin=a.width / 2.0 + tolerance):
                union(i, len(segs) + len(obs) + pad_positions[id(pad)])

    labels = []
    for i, s in enumerate(segs):
        if segment_key(s) in immutable_keys:
            labels.append(("track:" + segment_key(s), find(i)))
    labels.extend((f"obstacle:{i}", find(len(segs) + i)) for i in range(len(obs)))
    labels.extend((f"pad:{i}", find(len(segs) + len(obs) + i))
                  for i in range(len(pads)))
    groups = {}
    for label, root in labels:
        groups.setdefault(root, set()).add(label)
    return tuple(sorted(
        (frozenset(group) for group in groups.values() if len(group) > 1),
        key=lambda group: tuple(sorted(group))))


def _connectivity_signature(segments, obstacles, pad_regions,
                            immutable_keys, net_id, tolerance):
    """Return a cached signature keyed only by the affected net."""
    net_segments = tuple(segment for segment in segments
                         if segment.net_id == net_id)
    net_obstacles = tuple(obstacle for obstacle in obstacles
                          if obstacle.net_id == net_id)
    net_pads = tuple(pad for pad in pad_regions if pad.net_id == net_id)
    net_keys = {segment_key(segment) for segment in net_segments}
    return _connectivity_partition(
        net_segments, net_obstacles, net_pads,
        frozenset(key for key in immutable_keys if key in net_keys), net_id,
        tolerance)


def _validate_connectivity(model, eligible_keys, result):
    removed = set(result.remove_keys)
    affected_nets = {s.net_id for s in model.segments if segment_key(s) in removed}
    if not affected_nets:
        return

    # Build compact per-net inputs once.  Candidate composition frequently
    # validates hundreds of prefixes; rebuilding and hashing a whole-board
    # after-state for every affected net dominated runtime on large boards.
    before_by_net = {net_id: [] for net_id in affected_nets}
    immutable_by_net = {net_id: set() for net_id in affected_nets}
    for segment in model.segments:
        if segment.net_id not in affected_nets:
            continue
        before_by_net[segment.net_id].append(segment)
        key = segment_key(segment)
        if key not in eligible_keys:
            immutable_by_net[segment.net_id].add(key)
    obstacles_by_net = {
        net_id: [item for item in model.obstacles if item.net_id == net_id]
        for net_id in affected_nets}
    pads_by_net = {
        net_id: [item for item in model.pad_regions if item.net_id == net_id]
        for net_id in affected_nets}
    additions_by_net = {net_id: [] for net_id in affected_nets}
    for addition in result.additions:
        if addition.net_id in additions_by_net:
            additions_by_net[addition.net_id].append(Segment(
                addition.start[0], addition.start[1],
                addition.end[0], addition.end[1], addition.width,
                addition.layer, addition.net_id,
                clearance=addition.clearance))

    for net_id in affected_nets:
        before = before_by_net[net_id]
        after = [segment for segment in before
                 if segment_key(segment) not in removed]
        after.extend(additions_by_net[net_id])
        obstacles = obstacles_by_net[net_id]
        pads = pads_by_net[net_id]
        immutable = immutable_by_net[net_id]
        before_groups = _connectivity_signature(
            before, obstacles, pads, immutable, net_id,
            model.coordinate_quantum_mm)
        after_groups = _connectivity_signature(
            after, obstacles, pads, immutable, net_id,
            model.coordinate_quantum_mm)
        after_by_label = {
            label: group for group in after_groups for label in group
        }
        lost = any(
            not before_group.issubset(after_by_label.get(label, frozenset()))
            for before_group in before_groups
            for label in (next(iter(before_group)),))
        if lost:
            raise ValueError("Candidate would degrade connectivity on net {}".format(net_id))
