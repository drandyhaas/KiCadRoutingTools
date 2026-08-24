"""Conservative invariants checked before any live-board mutation."""

from __future__ import annotations

from collections import Counter

from .geometry import point_segment_distance, segment_distance
from .model import AddedSegment, GlossResult, Segment, segment_key


def immutable_fingerprint(segments, eligible_keys):
    return Counter(
        (segment_key(s), s.start_x, s.start_y, s.end_x, s.end_y,
         s.width, s.layer, s.net_id, s.locked, s.arc)
        for s in segments if segment_key(s) not in eligible_keys
    )


def validate_result(model, eligible_keys, result: GlossResult):
    known = {segment_key(s): s for s in model.segments}
    if any(key not in eligible_keys for key in result.remove_keys):
        raise ValueError("The candidate attempts to remove an unselected track")
    if any(key not in known for key in result.remove_keys):
        raise ValueError("The candidate references a track no longer present")
    removed = [known[key] for key in result.remove_keys]
    old_by_net = Counter(s.net_id for s in removed)
    new_by_net = Counter(s.net_id for s in result.additions)
    if old_by_net != new_by_net and not result.additions:
        raise ValueError("The candidate removes copper without replacing it")
    for new in result.additions:
        if new.width <= 0 or new.net_id <= 0 or new.start == new.end:
            raise ValueError("Invalid replacement segment")
    for index, first in enumerate(result.additions):
        for second in result.additions[index + 1:]:
            if first.layer != second.layer or first.net_id == second.net_id:
                continue
            clearance = max(
                model.minimum_clearance,
                model.net_clearances.get(first.net_id, 0.0),
                model.net_clearances.get(second.net_id, 0.0))
            required = clearance + (first.width + second.width) / 2.0
            if segment_distance(first.start, first.end,
                                second.start, second.end) < required - 1e-6:
                raise ValueError("Candidate additions violate inter-net clearance")
    _validate_connectivity(model, eligible_keys, result)


def _connectivity_signature(segments, obstacles, immutable_keys, net_id):
    segs = [s for s in segments if s.net_id == net_id]
    obs = [o for o in obstacles if o.net_id == net_id]
    count = len(segs) + len(obs)
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

    for i, a in enumerate(segs):
        aa, ab = (a.start_x, a.start_y), (a.end_x, a.end_y)
        for j in range(i + 1, len(segs)):
            b = segs[j]
            if a.layer != b.layer:
                continue
            if segment_distance(aa, ab, (b.start_x, b.start_y),
                                (b.end_x, b.end_y)) <= (a.width + b.width) / 2.0 + 1e-5:
                union(i, j)
        for j, obstacle in enumerate(obs):
            if obstacle.layers and a.layer not in obstacle.layers:
                continue
            if point_segment_distance((obstacle.x, obstacle.y), aa, ab) <= \
                    obstacle.radius + a.width / 2.0 + 1e-5:
                union(i, len(segs) + j)

    labels = []
    for i, s in enumerate(segs):
        if segment_key(s) in immutable_keys:
            labels.append(("track:" + segment_key(s), find(i)))
    labels.extend((f"obstacle:{i}", find(len(segs) + i)) for i in range(len(obs)))
    connected = set()
    for i, (left, root) in enumerate(labels):
        for right, other_root in labels[i + 1:]:
            if root == other_root:
                connected.add(tuple(sorted((left, right))))
    return connected


def _validate_connectivity(model, eligible_keys, result):
    removed = set(result.remove_keys)
    after = [s for s in model.segments if segment_key(s) not in removed]
    after.extend(Segment(a.start[0], a.start[1], a.end[0], a.end[1], a.width,
                         a.layer, a.net_id) for a in result.additions)
    immutable = {segment_key(s) for s in model.segments if segment_key(s) not in eligible_keys}
    affected_nets = {s.net_id for s in model.segments if segment_key(s) in removed}
    for net_id in affected_nets:
        before_pairs = _connectivity_signature(model.segments, model.obstacles, immutable, net_id)
        after_pairs = _connectivity_signature(after, model.obstacles, immutable, net_id)
        lost = before_pairs - after_pairs
        if lost:
            raise ValueError("Candidate would degrade connectivity on net {}".format(net_id))
