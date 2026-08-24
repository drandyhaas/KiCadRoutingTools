"""
Selected-track gloss implementation for KiCad.

This file is derived from the post-route cleanup and octolinear smoothing
code originally developed for KiCadRoutingTools by DrAndyHaas.

Original project:
https://github.com/drandyhaas/KiCadRoutingTools

The original copyright notices and license terms remain applicable.
Standalone plugin adaptation and selected-track scoping: KiCad Track Gloss.
Standalone modifications in this branch were created with ChatGPT/Codex
(OpenAI), at the project owner's direction.
"""

from __future__ import annotations

from collections import defaultdict

from .geometry import (length, octolinear_paths, path_hits_polygon,
                       point_segment_distance, segment_distance)
from .model import AddedSegment, GlossResult, segment_key
from .terminals import (find_pad_terminal_targets, find_track_terminal_targets,
                        movable_endpoint_pairs, vertex)
from .validation import validate_result


def _copper_signature(start, end, width, layer, net_id):
    a = (round(start[0], 6), round(start[1], 6))
    b = (round(end[0], 6), round(end[1], 6))
    return (min(a, b), max(a, b), round(width, 6), layer, net_id)


def _retain_identity_replacements(model, result):
    """Keep original native items instead of removing and recreating them."""
    removed = set(result.remove_keys)
    originals = defaultdict(list)
    for segment in model.segments:
        key = segment_key(segment)
        if key in removed:
            originals[_copper_signature(
                (segment.start_x, segment.start_y),
                (segment.end_x, segment.end_y), segment.width,
                segment.layer, segment.net_id)].append(key)

    cancelled = set()
    additions = []
    for addition in result.additions:
        signature = _copper_signature(
            addition.start, addition.end, addition.width,
            addition.layer, addition.net_id)
        matches = originals.get(signature)
        if matches:
            cancelled.add(matches.pop())
        else:
            additions.append(addition)
    if cancelled:
        result.remove_keys = [key for key in result.remove_keys
                              if key not in cancelled]
        result.additions = additions


def _path_clear(model, path, moving, replaced_keys, clearance):
    moving_clearance = max(clearance, model.minimum_clearance,
                           model.net_clearances.get(moving.net_id, 0.0))
    replaced_segments = [segment for segment in model.segments
                         if segment_key(segment) in replaced_keys]

    def unchanged_copper(a, b):
        """True when the candidate only retains part of removed copper.

        Pad obstacles use conservative enclosing circles.  Rechecking an
        unchanged subsegment against those circles can therefore invent a DRC
        violation that the original KiCad geometry never had.
        """
        return any(
            point_segment_distance(a, (segment.start_x, segment.start_y),
                                   (segment.end_x, segment.end_y)) <= 1e-6 and
            point_segment_distance(b, (segment.start_x, segment.start_y),
                                   (segment.end_x, segment.end_y)) <= 1e-6
            for segment in replaced_segments)

    for a, b in zip(path, path[1:]):
        if unchanged_copper(a, b):
            continue
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
        all_incidence[(s.net_id, vertex(s.start_x, s.start_y))] += 1
        all_incidence[(s.net_id, vertex(s.end_x, s.end_y))] += 1

    obstacle_vertices = defaultdict(set)
    for o in model.obstacles:
        if o.net_id > 0:
            obstacle_vertices[o.net_id].add(vertex(o.x, o.y))
    track_terminal_targets = find_track_terminal_targets(model, eligible)
    pad_terminal_targets = find_pad_terminal_targets(model, eligible)

    groups = defaultdict(list)
    for s in model.segments:
        if (segment_key(s) in eligible and not s.locked and not s.arc and
                s.net_id > 0 and length((s.start_x, s.start_y), (s.end_x, s.end_y)) > 1e-9):
            groups[(s.net_id, s.layer, round(s.width, 6))].append(s)

    for net_id, layer, width in sorted(groups):
        candidates = sorted(groups[(net_id, layer, width)], key=segment_key)
        adjacency = defaultdict(list)
        for s in candidates:
            adjacency[vertex(s.start_x, s.start_y)].append(s)
            adjacency[vertex(s.end_x, s.end_y)].append(s)

        def interior(v):
            return (len(adjacency[v]) == 2 and all_incidence[(net_id, v)] == 2 and
                    v not in obstacle_vertices[net_id] and
                    (net_id, layer, v) not in track_terminal_targets and
                    (net_id, layer, v) not in pad_terminal_targets)

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
                points.append((seg.start_x, seg.start_y) if vertex(seg.start_x, seg.start_y) == current
                              else (seg.end_x, seg.end_y))
                while True:
                    key = segment_key(seg)
                    used.add(key)
                    chain.append(seg)
                    next_point = ((seg.end_x, seg.end_y) if vertex(seg.start_x, seg.start_y) == current
                                  else (seg.start_x, seg.start_y))
                    points.append(next_point)
                    current = vertex(*next_point)
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
                        start_terminal = (net_id, layer, vertex(*points[i]))
                        end_terminal = (net_id, layer, vertex(*points[j]))
                        start_targets = (track_terminal_targets.get(start_terminal, ())
                                         if i == 0 else ())
                        end_targets = (track_terminal_targets.get(end_terminal, ())
                                       if j == len(chain) else ())
                        start_pads = (pad_terminal_targets.get(start_terminal, ())
                                      if i == 0 else ())
                        end_pads = (pad_terminal_targets.get(end_terminal, ())
                                    if j == len(chain) else ())
                        if j == i + 1 and not (
                                start_targets or end_targets or start_pads or end_pads):
                            continue
                        paths = []
                        for candidate_start, candidate_end in movable_endpoint_pairs(
                                points[i], points[j], start_targets, end_targets,
                                start_pads, end_pads):
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

    _retain_identity_replacements(model, result)
    validate_result(model, eligible, result)
    return result


def generate_candidate_plans(model, eligible_segment_keys, **kwargs):
    """Generate deterministic global and isolated-group fallback plans."""
    plans = []
    seen = set()
    rejected = []

    def add_plan(plan):
        signature = (
            tuple(sorted(plan.remove_keys)),
            tuple(sorted((a.start, a.end, a.width, a.layer, a.net_id)
                         for a in plan.additions)),
        )
        if signature not in seen:
            seen.add(signature)
            plans.append(plan)

    # Global weighted interval scheduling now evaluates every endpoint and
    # elbow candidate deterministically, so the older greedy/path-order passes
    # cannot improve its objective and only multiply runtime.
    try:
        add_plan(smooth_selected_chains(
            model, eligible_segment_keys, span_strategy="global",
            path_preference=0, **kwargs))
    except ValueError as error:
        rejected.append(str(error))

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
    # With a single group the global plan above is exactly the same call.  Do
    # not repeat an expensive no-op search (formerly visible as a GUI freeze
    # on dense tuned connections).
    for group_key in sorted(groups) if len(groups) > 1 else ():
        try:
            plan = smooth_selected_chains(
                model, groups[group_key], span_strategy="global",
                path_preference=0, **kwargs)
            if plan.changed:
                group_plans.append(plan)
        except ValueError as error:
            rejected.append(str(error))

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
        try:
            add_plan(merge(group_plans))
        except ValueError as error:
            rejected.append(str(error))
        for omitted in range(len(group_plans)):
            try:
                add_plan(merge([p for index, p in enumerate(group_plans)
                                if index != omitted]))
            except ValueError as error:
                rejected.append(str(error))
    for plan in group_plans:
        add_plan(plan)
    plans.sort(key=lambda p: (-p.saved_mm, len(p.additions),
                              tuple(sorted(p.remove_keys))))
    if not plans:
        plans.append(GlossResult(warnings=sorted(set(rejected))))
    return plans
