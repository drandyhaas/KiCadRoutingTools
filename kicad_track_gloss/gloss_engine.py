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


def find_track_terminal_targets(model, eligible_segment_keys, tolerance=1e-5):
    """Find selected vertices electrically terminating on immutable tracks.

    KiCad permits a track endpoint to land on the middle of another track
    without splitting the through-track object. The vertex is a chain boundary,
    but its contact point may slide along the immutable through-track.
    """
    eligible = {str(key) for key in eligible_segment_keys}
    immutable = [segment for segment in model.segments
                 if segment_key(segment) not in eligible]
    targets = defaultdict(list)
    for segment in model.segments:
        if segment_key(segment) not in eligible:
            continue
        for point in ((segment.start_x, segment.start_y),
                      (segment.end_x, segment.end_y)):
            terminal = (segment.net_id, segment.layer, _vertex(*point))
            for other in immutable:
                if (other.net_id != segment.net_id or other.layer != segment.layer or
                        other.arc):
                    continue
                if point_segment_distance(
                        point, (other.start_x, other.start_y),
                        (other.end_x, other.end_y)) <= tolerance:
                    targets[terminal].append(other)
    return {terminal: tuple(sorted(found, key=segment_key))
            for terminal, found in targets.items()}


def find_track_terminal_vertices(model, eligible_segment_keys, tolerance=1e-5):
    return set(find_track_terminal_targets(
        model, eligible_segment_keys, tolerance))


def _project_to_segment(point, segment):
    a = (segment.start_x, segment.start_y)
    b = (segment.end_x, segment.end_y)
    dx, dy = b[0] - a[0], b[1] - a[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1e-18:
        return a
    t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / denominator
    t = max(0.0, min(1.0, t))
    return _vertex(a[0] + t * dx, a[1] + t * dy)


def _sliding_contact_points(reference, original, targets):
    """Return deterministic octolinear-critical contacts on target tracks."""
    contacts = {_vertex(*original)}
    directions = ((1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, -1.0))
    for target in targets:
        a = (target.start_x, target.start_y)
        b = (target.end_x, target.end_y)
        contacts.update((_vertex(*a), _vertex(*b), _project_to_segment(reference, target)))
        wx, wy = b[0] - a[0], b[1] - a[1]
        for vx, vy in directions:
            denominator = vx * wy - vy * wx
            if abs(denominator) <= 1e-12:
                continue
            cx, cy = a[0] - reference[0], a[1] - reference[1]
            t = (cx * vy - cy * vx) / denominator
            if -1e-9 <= t <= 1.0 + 1e-9:
                contacts.add(_vertex(a[0] + t * wx, a[1] + t * wy))
    return sorted(contacts)


def _movable_endpoint_pairs(start, end, start_targets, end_targets):
    starts = (_sliding_contact_points(end, start, start_targets)
              if start_targets else [start])
    ends = (_sliding_contact_points(start, end, end_targets)
            if end_targets else [end])
    if start_targets and end_targets:
        initial_starts, initial_ends = list(starts), list(ends)
        starts = sorted(set(starts).union(
            point for candidate_end in initial_ends
            for point in _sliding_contact_points(candidate_end, start, start_targets)))
        ends = sorted(set(ends).union(
            point for candidate_start in initial_starts
            for point in _sliding_contact_points(candidate_start, end, end_targets)))
    return ((candidate_start, candidate_end)
            for candidate_start in starts for candidate_end in ends)


def _path_clear(model, path, moving, replaced_keys, clearance):
    moving_clearance = max(clearance, model.minimum_clearance,
                           model.net_clearances.get(moving.net_id, 0.0))
    for a, b in zip(path, path[1:]):
        if model.board_bounds:
            x0, y0, x1, y1 = model.board_bounds
            edge_margin = model.copper_edge_clearance + moving.width / 2.0
            if any(not (x0 + edge_margin <= p[0] <= x1 - edge_margin and
                        y0 + edge_margin <= p[1] <= y1 - edge_margin)
                   for p in (a, b)):
                return False
        for other in model.segments:
            if segment_key(other) in replaced_keys:
                continue
            if other.layer != moving.layer or other.net_id == moving.net_id:
                continue
            pair_clearance = max(moving_clearance,
                                 model.net_clearances.get(other.net_id, 0.0))
            required = pair_clearance + (moving.width + other.width) / 2.0
            if segment_distance(a, b, (other.start_x, other.start_y),
                                (other.end_x, other.end_y)) < required - 1e-6:
                return False
        for obstacle in model.obstacles:
            if obstacle.net_id == moving.net_id:
                continue
            if obstacle.layers and moving.layer not in obstacle.layers:
                continue
            required = max(moving_clearance, obstacle.clearance) + \
                moving.width / 2.0 + obstacle.radius
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
                           equal_length_tolerance=0.001,
                           span_strategy="farthest", path_preference=0):
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
    track_terminal_targets = find_track_terminal_targets(model, eligible)

    groups = defaultdict(list)
    for s in model.segments:
        if (segment_key(s) in eligible and not s.locked and not s.arc and
                s.net_id > 0 and length((s.start_x, s.start_y), (s.end_x, s.end_y)) > 1e-9):
            groups[(s.net_id, s.layer, round(s.width, 6))].append(s)

    for net_id, layer, width in sorted(groups):
        candidates = sorted(groups[(net_id, layer, width)], key=segment_key)
        adjacency = defaultdict(list)
        for s in candidates:
            adjacency[_vertex(s.start_x, s.start_y)].append(s)
            adjacency[_vertex(s.end_x, s.end_y)].append(s)

        def interior(v):
            return (len(adjacency[v]) == 2 and all_incidence[(net_id, v)] == 2 and
                    v not in obstacle_vertices[net_id] and
                    (net_id, layer, v) not in track_terminal_targets)

        for touching in adjacency.values():
            touching.sort(key=segment_key)
        anchors = sorted(v for v in adjacency if not interior(v))
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
                    nxt = sorted((s for s in adjacency[current] if segment_key(s) not in used),
                                 key=segment_key)
                    if not nxt:
                        break
                    seg = nxt[0]
                if current == anchor or not chain:
                    continue
                result.chains_considered += 1
                cumulative = [0.0]
                for a, b in zip(points, points[1:]):
                    cumulative.append(cumulative[-1] + length(a, b))
                def choices_at(i):
                    choices = []
                    for j in range(len(chain), i, -1):
                        old_len = cumulative[j] - cumulative[i]
                        replaced = {segment_key(s) for s in chain[i:j]}
                        start_terminal = (net_id, layer, _vertex(*points[i]))
                        end_terminal = (net_id, layer, _vertex(*points[j]))
                        start_targets = (track_terminal_targets.get(start_terminal, ())
                                         if i == 0 else ())
                        end_targets = (track_terminal_targets.get(end_terminal, ())
                                       if j == len(chain) else ())
                        if j == i + 1 and not (start_targets or end_targets):
                            continue
                        paths = []
                        for candidate_start, candidate_end in _movable_endpoint_pairs(
                                points[i], points[j], start_targets, end_targets):
                            if length(candidate_start, candidate_end) <= 1e-9:
                                continue
                            for path in octolinear_paths(candidate_start, candidate_end):
                                if path not in paths:
                                    paths.append(path)
                        if path_preference:
                            paths.reverse()
                        for path in paths:
                            new_len = sum(length(a, b) for a, b in zip(path, path[1:]))
                            new_count = len(path) - 1
                            gain = old_len - new_len
                            acceptable = gain >= min_gain or (
                                allow_equal_length_simpler and abs(gain) <= equal_length_tolerance
                                and new_count < j - i)
                            if acceptable and _path_clear(model, path, chain[i], replaced, clearance):
                                choices.append((j, path, max(0.0, gain), replaced,
                                                new_count, old_len))
                        if choices and span_strategy == "farthest":
                            break
                    return choices

                spans = {}
                if span_strategy == "global":
                    # Weighted interval scheduling: maximize total saved length
                    # over the WHOLE chain, then segment reduction. This avoids
                    # the A-before-B bias of greedy shortcut acceptance.
                    options = {i: choices_at(i) for i in range(len(chain))}
                    best = {len(chain): (0.0, 0, [])}
                    for i in range(len(chain) - 1, -1, -1):
                        skip = best.get(i + 1, (0.0, 0, []))
                        candidates_dp = [skip]
                        for choice in options.get(i, ()):
                            j, path, gain, replaced, new_count, _old_len = choice
                            tail = best.get(j, (0.0, 0, []))
                            reduction = (j - i) - new_count
                            candidates_dp.append((gain + tail[0], reduction + tail[1],
                                                  [(i, choice[:4])] + tail[2]))

                        def dp_key(candidate):
                            signature = tuple((start, span[0], tuple(span[1]))
                                              for start, span in candidate[2])
                            return (round(candidate[0], 9), candidate[1],
                                    tuple((-a, -b, path) for a, b, path in signature))

                        best[i] = max(candidates_dp, key=dp_key)
                    spans = {start: span for start, span in best[0][2]}
                else:
                    i = 0
                    while i < len(chain):
                        choices = choices_at(i)
                        if choices:
                            if span_strategy == "max_gain":
                                choices.sort(key=lambda c: (-c[2], c[4], -c[0], tuple(c[1])))
                            elif span_strategy == "farthest":
                                choices.sort(key=lambda c: (-c[0], -c[2], c[4], tuple(c[1])))
                            found = choices[0][:4]
                        else:
                            found = None
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


def generate_candidate_plans(model, eligible_segment_keys, **kwargs):
    """Generate deterministic alternatives for selection by KiCad's DRC oracle."""
    plans = []
    seen = set()

    def add_plan(plan):
        signature = (
            tuple(sorted(plan.remove_keys)),
            tuple(sorted((a.start, a.end, a.width, a.layer, a.net_id)
                         for a in plan.additions)),
        )
        if signature not in seen:
            seen.add(signature)
            plans.append(plan)

    for strategy in ("global", "max_gain", "farthest"):
        for path_preference in (0, 1):
            plan = smooth_selected_chains(
                model, eligible_segment_keys, span_strategy=strategy,
                path_preference=path_preference, **kwargs)
            add_plan(plan)

    # Batch fallback pool. If KiCad rejects the combined optimum, try leaving
    # out one independently scoped group, then each group alone. This keeps one
    # difficult track from blocking every other selected track while retaining
    # deterministic, gain-ranked behavior.
    eligible = set(eligible_segment_keys)
    groups = defaultdict(set)
    for segment in model.segments:
        key = segment_key(segment)
        if key in eligible:
            groups[(segment.net_id, segment.layer, round(segment.width, 6))].add(key)
    group_plans = []
    for group_key in sorted(groups):
        plan = smooth_selected_chains(model, groups[group_key], span_strategy="global",
                                      path_preference=0, **kwargs)
        if plan.changed:
            group_plans.append(plan)

    def merge(selected):
        merged = GlossResult()
        for plan in selected:
            merged.remove_keys.extend(plan.remove_keys)
            merged.additions.extend(plan.additions)
            merged.saved_mm += plan.saved_mm
            merged.chains_considered += plan.chains_considered
            merged.chains_changed += plan.chains_changed
            merged.warnings.extend(plan.warnings)
        validate_result(model, eligible, merged)
        return merged

    if len(group_plans) > 1:
        add_plan(merge(group_plans))
        for omitted in range(len(group_plans)):
            add_plan(merge([p for index, p in enumerate(group_plans) if index != omitted]))
    for plan in group_plans:
        add_plan(plan)
    plans.sort(key=lambda p: (-p.saved_mm, len(p.additions),
                              tuple(sorted(p.remove_keys))))
    return plans
