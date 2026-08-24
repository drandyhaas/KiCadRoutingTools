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
from dataclasses import replace

from .geometry import (length, octolinear_paths, path_hits_polygon,
                       point_segment_distance, segment_distance)
from .model import AddedSegment, GlossResult, Segment, Transformation, segment_key
from .pads import pad_contains, segment_hits_pad
from .statistics import classify_transformation
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


def _path_blocker(model, path, moving, replaced_keys, clearance):
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
                return "board_edge"
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
                return "foreign_track_clearance"
        for obstacle in model.obstacles:
            if obstacle.net_id == moving.net_id:
                continue
            if obstacle.layers and moving.layer not in obstacle.layers:
                continue
            required = max(moving_clearance, obstacle.clearance) + \
                moving.width / 2.0 + obstacle.radius
            if point_segment_distance((obstacle.x, obstacle.y), a, b) < required - 1e-6:
                return obstacle.kind + "_clearance"
        for pad in model.pad_regions:
            if pad.net_id == moving.net_id:
                continue
            if pad.layers and moving.layer not in pad.layers:
                continue
            pad_clearance = max(moving_clearance, pad.clearance)
            margin = max(0.0, pad_clearance + moving.width / 2.0 -
                         _PAD_CLEARANCE_TOLERANCE_MM)
            enclosing_radius = (pad.width * pad.width +
                                pad.height * pad.height) ** 0.5 / 2.0
            if point_segment_distance((pad.x, pad.y), a, b) >= \
                    enclosing_radius + margin:
                continue
            if segment_hits_pad(
                    pad, a, b, margin=margin):
                return "pad_clearance"
        for keepout in model.keepouts:
            if keepout.layers and moving.layer not in keepout.layers:
                continue
            if path_hits_polygon(a, b, list(keepout.points), clearance + moving.width / 2.0):
                return "keepout"
    return None


def _increment(counts, key):
    counts[key] = counts.get(key, 0) + 1


def _endpoint_moved_into_pad(original, candidate, terminal, pad_targets):
    """Distinguish an actual pad contact from another movable termination."""
    if length(original, candidate) <= 1e-6:
        return False
    return any(pad_contains(region, candidate, tolerance=1e-6)
               for region in pad_targets.get(terminal, ()))


_SYNTHETIC_PREFIX = "__track_gloss__"
_PAD_CLEARANCE_TOLERANCE_MM = 0.002
_REFINEMENT_SCOPE_LIMIT = 128


def _proper_intersection(first, second):
    """Return a centreline crossing strictly inside ``first`` when present."""
    a = (first.start_x, first.start_y)
    b = (first.end_x, first.end_y)
    c = (second.start_x, second.start_y)
    d = (second.end_x, second.end_y)
    rx, ry = b[0] - a[0], b[1] - a[1]
    sx, sy = d[0] - c[0], d[1] - c[1]
    denominator = rx * sy - ry * sx
    if abs(denominator) <= 1e-12:
        return None
    qx, qy = c[0] - a[0], c[1] - a[1]
    t = (qx * sy - qy * sx) / denominator
    u = (qx * ry - qy * rx) / denominator
    if 1e-7 < t < 1.0 - 1e-7 and -1e-7 <= u <= 1.0 + 1e-7:
        return vertex(a[0] + t * rx, a[1] + t * ry)
    return None


def _split_eligible_intersections(model, eligible, pass_index):
    """Split movable copper where another same-net centreline crosses it."""
    segments = []
    next_eligible = set(eligible)
    counter = 0
    for moving in model.segments:
        key = segment_key(moving)
        if key not in eligible:
            segments.append(moving)
            continue
        points = []
        for other in model.segments:
            if (other is moving or other.net_id != moving.net_id or
                    other.layer != moving.layer):
                continue
            point = _proper_intersection(moving, other)
            if point is not None and point not in points:
                points.append(point)
        if not points:
            segments.append(moving)
            continue
        start = (moving.start_x, moving.start_y)
        end = (moving.end_x, moving.end_y)
        points.sort(key=lambda point: length(start, point))
        next_eligible.discard(key)
        for a, b in zip([start] + points, points + [end]):
            if length(a, b) <= 1e-6:
                continue
            synthetic = "{}split-{}-{}".format(
                _SYNTHETIC_PREFIX, pass_index, counter)
            counter += 1
            segments.append(Segment(
                a[0], a[1], b[0], b[1], moving.width, moving.layer,
                moving.net_id, synthetic, False, False, moving.net_name))
            next_eligible.add(synthetic)
    return replace(model, segments=segments), next_eligible


def _point_is_terminal(model, segment, point, excluded_key):
    for other in model.segments:
        if (segment_key(other) == excluded_key or
                other.net_id != segment.net_id or other.layer != segment.layer):
            continue
        if point_segment_distance(
                point, (other.start_x, other.start_y),
                (other.end_x, other.end_y)) <= 1e-6:
            return True
    return any(
        pad.net_id == segment.net_id and
        (not pad.layers or segment.layer in pad.layers) and
        pad_contains(pad, point, tolerance=1e-6)
        for pad in model.pad_regions)


def _crosses_wider_track(model, segment, point, excluded_key):
    return any(
        segment_key(other) != excluded_key and
        other.net_id == segment.net_id and other.layer == segment.layer and
        other.width > segment.width + 1e-6 and
        point_segment_distance(
            point, (other.start_x, other.start_y),
            (other.end_x, other.end_y)) <= 1e-6
        for other in model.segments)


def _prune_generated_stubs(model, eligible):
    """Drop synthetic tails extending beyond a newly created T contact."""
    segments = list(model.segments)
    removed_stubs = []
    while True:
        endpoint_counts = defaultdict(int)
        for segment in segments:
            endpoint_counts[(segment.net_id, segment.layer,
                             vertex(segment.start_x, segment.start_y))] += 1
            endpoint_counts[(segment.net_id, segment.layer,
                             vertex(segment.end_x, segment.end_y))] += 1
        removed = None
        current_model = replace(model, segments=segments)
        for segment in segments:
            key = segment_key(segment)
            if key not in eligible or not key.startswith(_SYNTHETIC_PREFIX):
                continue
            ends = ((segment.start_x, segment.start_y),
                    (segment.end_x, segment.end_y))
            for outer, junction in (ends, tuple(reversed(ends))):
                outer_count = endpoint_counts[
                    (segment.net_id, segment.layer, vertex(*outer))]
                junction_count = endpoint_counts[
                    (segment.net_id, segment.layer, vertex(*junction))]
                junction_crossing = _point_is_terminal(
                    current_model, segment, junction, key)
                outer_terminal = _point_is_terminal(
                    current_model, segment, outer, key)
                if (outer_count == 1 and not outer_terminal and
                        (junction_count >= 3 or junction_crossing) and
                        _crosses_wider_track(
                            current_model, segment, junction, key)):
                    removed = segment
                    break
            if removed is not None:
                break
        if removed is None:
            break
        segments.remove(removed)
        eligible.discard(segment_key(removed))
        removed_stubs.append(removed)
    return replace(model, segments=segments), eligible, removed_stubs


def _apply_to_model(model, eligible, plan, pass_index):
    removed = set(plan.remove_keys)
    names = {segment.net_id: segment.net_name for segment in model.segments}
    segments = [segment for segment in model.segments
                if segment_key(segment) not in removed]
    next_eligible = set(eligible) - removed
    for index, addition in enumerate(plan.additions):
        key = "{}pass-{}-{}".format(_SYNTHETIC_PREFIX, pass_index, index)
        segments.append(Segment(
            addition.start[0], addition.start[1], addition.end[0],
            addition.end[1], addition.width, addition.layer, addition.net_id,
            key, False, False, names.get(addition.net_id, "")))
        next_eligible.add(key)
    return replace(model, segments=segments), next_eligible


def _merge_final_collinear(model, eligible):
    """Undo analytical splits on straight through-tracks before live apply."""
    segments = list(model.segments)
    counter = 0
    while True:
        merged = None
        for index, first in enumerate(segments):
            first_key = segment_key(first)
            if first_key not in eligible or not first_key.startswith(_SYNTHETIC_PREFIX):
                continue
            first_ends = ((first.start_x, first.start_y),
                          (first.end_x, first.end_y))
            for second in segments[index + 1:]:
                second_key = segment_key(second)
                if (second_key not in eligible or
                        not second_key.startswith(_SYNTHETIC_PREFIX) or
                        (first.net_id, first.layer, round(first.width, 6)) !=
                        (second.net_id, second.layer, round(second.width, 6))):
                    continue
                second_ends = ((second.start_x, second.start_y),
                               (second.end_x, second.end_y))
                shared = set(first_ends) & set(second_ends)
                if len(shared) != 1:
                    continue
                joint = next(iter(shared))
                outer_first = first_ends[1] if first_ends[0] == joint else first_ends[0]
                outer_second = second_ends[1] if second_ends[0] == joint else second_ends[0]
                cross = ((joint[0] - outer_first[0]) *
                         (outer_second[1] - joint[1]) -
                         (joint[1] - outer_first[1]) *
                         (outer_second[0] - joint[0]))
                if abs(cross) > 1e-7:
                    continue
                key = "{}merge-{}".format(_SYNTHETIC_PREFIX, counter)
                counter += 1
                merged = (first, second, Segment(
                    outer_first[0], outer_first[1], outer_second[0],
                    outer_second[1], first.width, first.layer, first.net_id,
                    key, False, False, first.net_name))
                break
            if merged is not None:
                break
        if merged is None:
            break
        first, second, replacement = merged
        segments.remove(first)
        segments.remove(second)
        eligible.discard(segment_key(first))
        eligible.discard(segment_key(second))
        segments.append(replacement)
        eligible.add(segment_key(replacement))
    return replace(model, segments=segments), eligible


def _compose_refined_plan(original_model, original_eligible, final_model,
                          final_eligible, passes, pruned_stubs):
    final_model, final_eligible = _merge_final_collinear(
        final_model, set(final_eligible))
    original_keys = {segment_key(segment) for segment in original_model.segments}
    final_keys = {segment_key(segment) for segment in final_model.segments}
    result = GlossResult()
    result.remove_keys = sorted(set(original_eligible) - final_keys)
    for segment in final_model.segments:
        key = segment_key(segment)
        if key in final_eligible and key not in original_keys:
            result.additions.append(AddedSegment(
                (segment.start_x, segment.start_y),
                (segment.end_x, segment.end_y), segment.width,
                segment.layer, segment.net_id))
    before_mm = sum(
        length((segment.start_x, segment.start_y),
               (segment.end_x, segment.end_y))
        for segment in original_model.segments
        if segment_key(segment) in original_eligible)
    after_mm = sum(
        length((segment.start_x, segment.start_y),
               (segment.end_x, segment.end_y))
        for segment in final_model.segments
        if segment_key(segment) in final_eligible)
    result.saved_mm = max(0.0, before_mm - after_mm)
    result.chains_considered = sum(plan.chains_considered for plan in passes)
    result.chains_changed = sum(plan.chains_changed for plan in passes)
    for plan in passes:
        result.warnings.extend(plan.warnings)
        result.transformations.extend(plan.transformations)
        for key, value in plan.search_counts.items():
            result.search_counts[key] = result.search_counts.get(key, 0) + value
    for stub in pruned_stubs:
        stub_mm = length((stub.start_x, stub.start_y),
                         (stub.end_x, stub.end_y))
        result.transformations.append(Transformation(
            "track_slide", "segment_simplification", stub.net_id,
            stub.net_name, stub.layer, stub.width, stub_mm, 0.0, 1, 0))
    _retain_identity_replacements(original_model, result)
    validate_result(original_model, set(original_eligible), result)
    return result


def _refine_plan(model, eligible, initial, planner_kwargs, max_passes=3):
    if not initial.changed:
        return initial
    simulated, current_eligible = _apply_to_model(
        model, set(eligible), initial, 0)
    passes = [initial]
    pruned_stubs = []
    for pass_index in range(1, max_passes + 1):
        simulated, current_eligible = _split_eligible_intersections(
            simulated, current_eligible, pass_index)
        simulated, current_eligible, removed = _prune_generated_stubs(
            simulated, current_eligible)
        pruned_stubs.extend(removed)
        step = smooth_selected_chains(
            simulated, current_eligible, span_strategy="global",
            path_preference=0, **planner_kwargs)
        if not step.changed:
            break
        passes.append(step)
        simulated, current_eligible = _apply_to_model(
            simulated, current_eligible, step, pass_index)
    return _compose_refined_plan(
        model, eligible, simulated, current_eligible, passes, pruned_stubs)


def smooth_selected_chains(model, eligible_segment_keys, *, min_gain=0.01,
                           allow_equal_length_simpler=False, clearance=0.1,
                           equal_length_tolerance=0.001,
                           span_strategy="farthest", path_preference=0,
                           collect_statistics=True):
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
                            if collect_statistics:
                                _increment(result.search_counts, "paths_evaluated")
                            new_len = sum(length(a, b) for a, b in zip(path, path[1:]))
                            new_count = len(path) - 1
                            gain = old_len - new_len
                            acceptable = gain >= min_gain or (
                                allow_equal_length_simpler and abs(gain) <= equal_length_tolerance
                                and new_count < j - i)
                            if not acceptable:
                                if collect_statistics:
                                    _increment(result.search_counts, "not_improving")
                                continue
                            blocker = _path_blocker(
                                model, path, chain[i], replaced, clearance)
                            if blocker:
                                if collect_statistics:
                                    _increment(result.search_counts, blocker)
                                continue
                            if collect_statistics:
                                _increment(result.search_counts, "accepted_options")
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
                    start_terminal = (net_id, layer, vertex(*points[k]))
                    end_terminal = (net_id, layer, vertex(*points[j]))
                    if collect_statistics:
                        moved_start = length(path[0], points[k]) > 1e-6
                        moved_end = length(path[-1], points[j]) > 1e-6
                        pad_moved = (
                            _endpoint_moved_into_pad(
                                points[k], path[0], start_terminal,
                                pad_terminal_targets) or
                            _endpoint_moved_into_pad(
                                points[j], path[-1], end_terminal,
                                pad_terminal_targets))
                        track_moved = (
                            (moved_start and
                             start_terminal in track_terminal_targets) or
                            (moved_end and
                             end_terminal in track_terminal_targets))
                        mechanism = ("pad_slide" if pad_moved else
                                     "track_slide" if track_moved else
                                     "fixed_endpoints")
                        result.transformations.append(classify_transformation(
                            chain[k:j], path, mechanism, equal_length_tolerance))
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
    max_refinement_passes = kwargs.pop("max_refinement_passes", 3)
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
            merged.transformations.extend(plan.transformations)
            for key, value in plan.search_counts.items():
                merged.search_counts[key] = merged.search_counts.get(key, 0) + value
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
    elif (plans[0].changed and max_refinement_passes and
          len(set(eligible_segment_keys)) <= _REFINEMENT_SCOPE_LIMIT):
        try:
            plans[0] = _refine_plan(
                model, eligible_segment_keys, plans[0], dict(kwargs),
                max_refinement_passes)
            plans.sort(key=lambda p: (-p.saved_mm, len(p.additions),
                                      tuple(sorted(p.remove_keys))))
        except ValueError as error:
            plans[0].warnings.append("Refinement rejected: " + str(error))
    return plans
