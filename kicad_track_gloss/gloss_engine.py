"""
Selected-track gloss implementation for KiCad.

This file is derived from the post-route cleanup and octolinear smoothing
code originally developed for KiCadRoutingTools by DrAndyHaas.

Original project:
https://github.com/drandyhaas/KiCadRoutingTools

The original copyright notices and license terms remain applicable.
Standalone plugin adaptation and selected-track scoping: KiCad Track Gloss.
"""

from __future__ import annotations

from collections import defaultdict
import math

from .connectivity import validate_result
from .geometry import length, octolinear_paths, path_hits_polygon, point_segment_distance, segment_distance
from .model import AddedSegment, GlossResult, segment_key


def _vertex(x, y):
    return round(x, 6), round(y, 6)


def _path_clear(model, path, moving, replaced_keys, clearance):
    for a, b in zip(path, path[1:]):
        for other in model.segments:
            if segment_key(other) in replaced_keys:
                continue
            if other.layer != moving.layer or other.net_id == moving.net_id:
                continue
            required = clearance + (moving.width + other.width) / 2.0
            if segment_distance(a, b, (other.start_x, other.start_y),
                                (other.end_x, other.end_y)) < required - 1e-6:
                return False
        for obstacle in model.obstacles:
            if obstacle.net_id == moving.net_id:
                continue
            if obstacle.layers and moving.layer not in obstacle.layers:
                continue
            required = clearance + moving.width / 2.0 + obstacle.radius
            if point_segment_distance((obstacle.x, obstacle.y), a, b) < required - 1e-6:
                return False
        for keepout in model.keepouts:
            if keepout.layers and moving.layer not in keepout.layers:
                continue
            if path_hits_polygon(a, b, list(keepout.points), clearance + moving.width / 2.0):
                return False
    return True


def smooth_selected_chains(model, eligible_segment_keys, *, min_gain=0.01,
                           allow_equal_length_simpler=False, clearance=0.1,
                           equal_length_tolerance=0.001):
    """Return an immutable edit plan; never mutates ``model``.

    Original octolinear smoothing algorithm: KiCadRoutingTools, DrAndyHaas.
    Standalone adaptation: only explicit KIID/geometric keys enter a chain.
    Non-selected copper remains in incidence and obstacle calculations.
    """
    eligible = {str(key) for key in eligible_segment_keys}
    result = GlossResult()
    all_incidence = defaultdict(int)
    for s in model.segments:
        all_incidence[(s.net_id, _vertex(s.start_x, s.start_y))] += 1
        all_incidence[(s.net_id, _vertex(s.end_x, s.end_y))] += 1

    obstacle_vertices = defaultdict(set)
    for o in model.obstacles:
        if o.net_id > 0:
            obstacle_vertices[o.net_id].add(_vertex(o.x, o.y))

    groups = defaultdict(list)
    for s in model.segments:
        if (segment_key(s) in eligible and not s.locked and not s.arc and
                s.net_id > 0 and length((s.start_x, s.start_y), (s.end_x, s.end_y)) > 1e-9):
            groups[(s.net_id, s.layer, round(s.width, 6))].append(s)

    for (net_id, layer, width), candidates in groups.items():
        adjacency = defaultdict(list)
        for s in candidates:
            adjacency[_vertex(s.start_x, s.start_y)].append(s)
            adjacency[_vertex(s.end_x, s.end_y)].append(s)

        def interior(v):
            return (len(adjacency[v]) == 2 and all_incidence[(net_id, v)] == 2 and
                    v not in obstacle_vertices[net_id])

        anchors = [v for v in adjacency if not interior(v)]
        used = set()
        for anchor in anchors:
            for first in adjacency[anchor]:
                if segment_key(first) in used:
                    continue
                chain, points = [], []
                current, seg = anchor, first
                points.append((seg.start_x, seg.start_y) if _vertex(seg.start_x, seg.start_y) == current
                              else (seg.end_x, seg.end_y))
                while True:
                    key = segment_key(seg)
                    used.add(key)
                    chain.append(seg)
                    next_point = ((seg.end_x, seg.end_y) if _vertex(seg.start_x, seg.start_y) == current
                                  else (seg.start_x, seg.start_y))
                    points.append(next_point)
                    current = _vertex(*next_point)
                    if current == anchor or not interior(current) or len(chain) >= 100:
                        break
                    nxt = [s for s in adjacency[current] if segment_key(s) not in used]
                    if not nxt:
                        break
                    seg = nxt[0]
                if current == anchor or len(chain) < 2:
                    continue
                result.chains_considered += 1
                cumulative = [0.0]
                for a, b in zip(points, points[1:]):
                    cumulative.append(cumulative[-1] + length(a, b))
                spans = {}
                i = 0
                while i < len(chain) - 1:
                    found = None
                    for j in range(len(chain), i + 1, -1):
                        old_len = cumulative[j] - cumulative[i]
                        replaced = {segment_key(s) for s in chain[i:j]}
                        for path in octolinear_paths(points[i], points[j]):
                            new_len = sum(length(a, b) for a, b in zip(path, path[1:]))
                            new_count = len(path) - 1
                            gain = old_len - new_len
                            acceptable = gain >= min_gain or (
                                allow_equal_length_simpler and abs(gain) <= equal_length_tolerance
                                and new_count < j - i)
                            if acceptable and _path_clear(model, path, chain[i], replaced, clearance):
                                found = (j, path, max(0.0, gain), replaced)
                                break
                        if found:
                            break
                    if found:
                        spans[i] = found
                        i = found[0]
                    else:
                        i += 1
                if not spans:
                    continue
                k = 0
                chain_changed = False
                while k < len(chain):
                    if k not in spans:
                        k += 1
                        continue
                    j, path, gain, replaced = spans[k]
                    result.remove_keys.extend(segment_key(s) for s in chain[k:j])
                    result.additions.extend(
                        AddedSegment(a, b, width, layer, net_id)
                        for a, b in zip(path, path[1:]) if length(a, b) > 1e-6)
                    result.saved_mm += gain
                    chain_changed = True
                    k = j
                if chain_changed:
                    result.chains_changed += 1

    validate_result(model, eligible, result)
    return result

